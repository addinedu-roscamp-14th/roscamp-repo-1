"""Run homography-corrected Pick/Place through the JetCobot direct API."""

import math
import threading
import time
from collections import deque
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import numpy as np

from porter_interfaces.srv import ExecutePickPlace

from pymycobot.mycobot280 import MyCobot280

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from sensor_msgs.msg import JointState

from std_msgs.msg import Bool, String

from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformException, TransformListener

from .model import (
    MarkerObservation,
    build_pick_place_steps,
    calibration_levels_for_surface,
    calibrate_target,
    destination_level,
    load_floor_calibration,
    parse_stations,
    safe_z_candidates,
    select_symmetric_yaw,
    target_with_command_yaw,
    wrap_degrees,
)


WORK_STATES = frozenset({
    'IDLE',
    'WORK_STARTED',
    'SEARCHING',
    'PICK_STARTED',
    'PICK_COMPLETED',
    'PLACE_STARTED',
    'PLACE_COMPLETED',
    'WORK_COMPLETED',
    'STOP_REQUESTED',
    'STOPPED',
    'FAILED',
})


def yaw_from_quaternion(rotation):
    """Extract wrapped base-frame yaw in degrees."""
    sin_yaw = 2.0 * (
        rotation.w * rotation.z + rotation.x * rotation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        rotation.y * rotation.y + rotation.z * rotation.z
    )
    return wrap_degrees(math.degrees(math.atan2(sin_yaw, cos_yaw)))


def pose_errors(actual, target):
    """Return absolute XYZ metres and wrapped RPY degree errors."""
    xyz = tuple(abs(float(a) - float(b)) for a, b in zip(
        actual[:3], target[:3]
    ))
    rpy = tuple(abs(wrap_degrees(float(a) - float(b))) for a, b in zip(
        actual[3:], target[3:]
    ))
    return xyz, rpy


class CoordinateMoveError(RuntimeError):
    """A controller coordinate command was rejected or did not arrive."""


class HomographyPickPlace(Node):
    """Search configured stations, freeze both markers, then manipulate."""

    def __init__(self):
        """Load calibration, connect hardware, and expose start/stop."""
        super().__init__('pick_place')
        self._declare_parameters()
        self.base_frame = str(self.parameter('base_frame'))
        self.command_frame = str(self.parameter('command_frame'))
        self.pick_frame = str(self.parameter('pick_marker_frame'))
        self.place_frame = str(self.parameter('place_marker_frame'))
        self.pick_marker_id = int(self.parameter('pick_marker_id'))
        self.place_marker_id = int(self.parameter('place_marker_id'))
        self.stations = parse_stations(str(self.parameter('stations_json')))
        self.floors = load_floor_calibration(
            str(self.parameter('calibration_file'))
        )
        self.minimum_samples = int(
            self.parameter('minimum_stable_samples')
        )
        self.translation_std = float(
            self.parameter('max_translation_std_m')
        )
        self.yaw_spread = float(
            self.parameter('max_yaw_spread_deg')
        )
        self.marker_age = float(self.parameter('max_marker_age_sec'))
        self.safe_z = float(self.parameter('safe_z_m'))
        self.yaw_offset = float(self.parameter('marker_yaw_offset_deg'))
        self.place_yaw_offset = float(
            self.parameter('place_marker_yaw_offset_deg')
        )
        self.cross_station_place_yaw_offset = float(
            self.parameter('cross_station_place_yaw_offset_deg')
        )
        self.position_tolerance = float(
            self.parameter('position_tolerance_m')
        )
        self.angle_tolerance = float(
            self.parameter('angle_tolerance_deg')
        )
        self._validate_parameters()
        self._log_calibration_quality()

        self.callback_group = ReentrantCallbackGroup()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        history_length = max(50, self.minimum_samples * 8)
        self.histories = {
            self.pick_frame: deque(maxlen=history_length),
            self.place_frame: deque(maxlen=history_length),
        }
        self.history_lock = threading.Lock()
        self.serial_lock = threading.Lock()
        self.operation_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.external_stop_requested = False
        self.motion_thread = None
        self.last_stamps = {}
        self.detection_enabled = False
        self.detection_started_ns = None
        self.active_station = ''

        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.detection_control = self.create_publisher(
            Bool,
            '/arm/pick_place/detection_enabled',
            control_qos,
        )
        self.status = self.create_publisher(
            String, '/arm/pick_place/status', 10
        )
        self.work_state = self.create_publisher(
            String,
            '/arm/pick_place/work_state',
            control_qos,
        )
        self.current_work_state = ''
        self.configure_targets_client = self.create_client(
            ExecutePickPlace,
            '/arm/pick_place/configure_targets',
            callback_group=self.callback_group,
        )
        self.create_service(
            ExecutePickPlace,
            '/arm/pick_place/execute',
            self.execute,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '/arm/pick_place/start',
            self.start,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '/arm/pick_place/stop',
            self.stop,
            callback_group=self.callback_group,
        )
        self.create_timer(
            0.05,
            self.collect_marker_samples,
            callback_group=self.callback_group,
        )

        self.robot = MyCobot280(
            str(self.parameter('serial_port')),
            int(self.parameter('baud_rate')),
        )
        time.sleep(1.0)
        self.robot.set_fresh_mode(1)
        if self.robot.is_power_on() != 1:
            self.robot.power_on()
            time.sleep(0.5)
        self.joint_state_publisher = self.create_publisher(
            JointState, '/arm/joint_states', 10
        )
        rate = float(self.parameter('joint_state_rate_hz'))
        self.create_timer(
            1.0 / rate,
            self.publish_joint_state,
            callback_group=self.callback_group,
        )
        self.set_detection_enabled(False)
        self.publish_work_state('IDLE')
        self.publish_status(
            'READY: direct send_coords + get_coords polling; '
            f'stations={[item.name for item in self.stations]}, '
            f'default_pick_id={self.pick_marker_id}, '
            f'default_place_id={self.place_marker_id}; '
            'dynamic service=/arm/pick_place/execute'
        )

    def _declare_parameters(self):
        self.declare_parameter('base_frame', 'arm/base_link')
        self.declare_parameter('command_frame', 'arm/controller_coords')
        self.declare_parameter('pick_marker_frame', 'arm/pick_marker')
        self.declare_parameter('place_marker_frame', 'arm/place_marker')
        self.declare_parameter('pick_marker_id', 2)
        self.declare_parameter('place_marker_id', 8)
        self.declare_parameter(
            'calibration_file',
            str(
                Path(
                    get_package_share_directory(
                        'arm_pick_place'
                    )
                )
                / 'config'
                / 'floor_calibration.yaml'
            ),
        )
        self.declare_parameter(
            'stations_json',
            '[{"name":"station_agv","calibration_surface":"agv",'
            '"joint_angles_deg":'
            '[15.38,35.59,-2.81,-90.96,4.13,-37.26],'
            '"timeout_sec":3.0},'
            '{"name":"station_a","calibration_surface":"station",'
            '"joint_angles_deg":'
            '[-86.39,57.12,-15.46,-88.15,7.99,-36.82],'
            '"timeout_sec":5.0}]',
        )
        self.declare_parameter('observation_settle_sec', 1.0)
        self.declare_parameter('observation_speed', 10)
        self.declare_parameter('observation_joint_tolerance_deg', 3.0)
        self.declare_parameter('observation_correction_attempts', 2)
        self.declare_parameter('minimum_stable_samples', 7)
        self.declare_parameter('max_translation_std_m', 0.003)
        self.declare_parameter('max_yaw_spread_deg', 5.0)
        self.declare_parameter('max_marker_age_sec', 1.0)
        self.declare_parameter('maximum_floor_error_m', 0.010)
        self.declare_parameter('minimum_floor_separation_m', 0.015)
        self.declare_parameter('maximum_h_extrapolation_m', 0.025)
        self.declare_parameter('command_area_margin_m', 0.020)
        self.declare_parameter('safe_z_m', 0.220)
        self.declare_parameter('safe_z_lowering_step_m', 0.010)
        self.declare_parameter('maximum_safe_z_lowering_steps', 3)
        self.declare_parameter('minimum_safe_clearance_m', 0.020)
        self.declare_parameter('marker_yaw_offset_deg', -45.0)
        self.declare_parameter('place_marker_yaw_offset_deg', 0.0)
        self.declare_parameter(
            'cross_station_place_yaw_offset_deg', -45.0
        )
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('observation_motion_timeout_sec', 20.0)
        self.declare_parameter('motion_timeout_sec', 20.0)
        self.declare_parameter('coordinate_poll_interval_sec', 0.1)
        self.declare_parameter('coordinate_required_stable_samples', 3)
        self.declare_parameter('place_stop_poll_interval_sec', 0.1)
        self.declare_parameter('place_stop_required_samples', 3)
        self.declare_parameter('place_stop_timeout_sec', 10.0)
        self.declare_parameter('speed', 20)
        self.declare_parameter('gripper_open_value', 100)
        self.declare_parameter('gripper_closed_value', 20)
        self.declare_parameter('gripper_speed', 50)
        self.declare_parameter('gripper_wait_sec', 1.0)
        self.declare_parameter('joint_state_rate_hz', 10.0)
        # Temporarily relaxed while validating the basic Pick/Place flow.
        self.declare_parameter('position_tolerance_m', 0.020)
        self.declare_parameter('angle_tolerance_deg', 6.0)

    def parameter(self, name):
        """Return a current ROS parameter value."""
        return self.get_parameter(name).value

    def _validate_parameters(self):
        if self.minimum_samples < 3:
            raise ValueError('minimum_stable_samples must be at least 3')
        positive = {
            'max_translation_std_m': self.translation_std,
            'max_yaw_spread_deg': self.yaw_spread,
            'max_marker_age_sec': self.marker_age,
            'safe_z_m': self.safe_z,
            'safe_z_lowering_step_m': float(
                self.parameter('safe_z_lowering_step_m')
            ),
            'minimum_safe_clearance_m': float(
                self.parameter('minimum_safe_clearance_m')
            ),
            'position_tolerance_m': self.position_tolerance,
            'angle_tolerance_deg': self.angle_tolerance,
            'observation_settle_sec': float(
                self.parameter('observation_settle_sec')
            ),
            'joint_state_rate_hz': float(
                self.parameter('joint_state_rate_hz')
            ),
            'coordinate_poll_interval_sec': float(
                self.parameter('coordinate_poll_interval_sec')
            ),
            'place_stop_poll_interval_sec': float(
                self.parameter('place_stop_poll_interval_sec')
            ),
            'place_stop_timeout_sec': float(
                self.parameter('place_stop_timeout_sec')
            ),
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be positive')
        highest_z = max(
            max(
                value
                for value in (floor.pick_z_m, floor.place_z_m)
                if value is not None
            )
            for floor in self.floors.values()
            if floor.pick_z_m is not None or floor.place_z_m is not None
        )
        if self.safe_z <= highest_z + 0.010:
            raise ValueError(
                'safe_z_m must be at least 10 mm above every taught Z'
            )
        if not 1 <= int(self.parameter('speed')) <= 100:
            raise ValueError('speed must be within 1..100')
        if not 1 <= int(self.parameter('observation_speed')) <= 100:
            raise ValueError('observation_speed must be within 1..100')
        if self.position_tolerance > 0.020:
            raise ValueError('position_tolerance_m may not exceed 0.020')
        if self.angle_tolerance > 10.0:
            raise ValueError('angle_tolerance_deg may not exceed 10.0')
        if int(self.parameter('coordinate_required_stable_samples')) < 1:
            raise ValueError(
                'coordinate_required_stable_samples must be at least 1'
            )
        if int(self.parameter('place_stop_required_samples')) < 1:
            raise ValueError('place_stop_required_samples must be at least 1')
        if int(self.parameter('maximum_safe_z_lowering_steps')) < 0:
            raise ValueError(
                'maximum_safe_z_lowering_steps must be non-negative'
            )

    def _log_calibration_quality(self):
        for number, floor in self.floors.items():
            if floor.pick_z_m is None or floor.place_z_m is None:
                height_text = 'geometry-only'
            else:
                height_text = (
                    f'pick_z={floor.pick_z_m:.5f} m, '
                    f'place_z={floor.place_z_m:.5f} m'
                )
            self.get_logger().info(
                f'level {number}: {height_text}, plane_training_max='
                f'{floor.plane_max_training_error_m * 1000.0:.2f} mm, '
                f'H inliers={floor.homography_inlier_count}/'
                f'{floor.homography_sample_count}'
            )
            if floor.homography_inlier_count <= 4:
                self.get_logger().warning(
                    f'level {number} H has only four inliers; zero fit '
                    'error is interpolation, not independent accuracy'
                )

    def publish_status(self, text):
        """Publish a status topic and matching ROS log."""
        message = String()
        message.data = text
        self.status.publish(message)
        self.get_logger().info(text)

    def publish_work_state(self, state):
        """Publish a stable machine-readable state for central control."""
        normalized = str(state).strip().upper()
        if normalized not in WORK_STATES:
            raise ValueError(f'unsupported work state: {state}')
        self.current_work_state = normalized
        message = String()
        message.data = normalized
        self.work_state.publish(message)
        self.get_logger().info(f'WORK_STATE: {normalized}')

    def set_detection_enabled(self, enabled):
        """Gate vision and clear samples across every motion boundary."""
        self.detection_enabled = bool(enabled)
        if enabled:
            self.detection_started_ns = self.get_clock().now().nanoseconds
        else:
            self.detection_started_ns = None
        with self.history_lock:
            for history in self.histories.values():
                history.clear()
        message = Bool()
        message.data = bool(enabled)
        self.detection_control.publish(message)

    def publish_joint_state(self):
        """Publish measured joints while this node owns the serial port."""
        if not self.serial_lock.acquire(blocking=False):
            return
        try:
            angles = self.robot.get_angles()
        except Exception:
            return
        finally:
            self.serial_lock.release()
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f'{index}_Joint' for index in range(1, 7)]
        message.position = [math.radians(float(value)) for value in angles]
        self.joint_state_publisher.publish(message)

    def collect_marker_samples(self):
        """Collect fresh TF only during a stationary observation window."""
        if not self.detection_enabled:
            return
        now_ns = self.get_clock().now().nanoseconds
        for frame in self.histories:
            try:
                transform = self.buffer.lookup_transform(
                    self.base_frame, frame, Time()
                )
            except TransformException:
                continue
            stamp_ns = (
                int(transform.header.stamp.sec) * 1_000_000_000
                + int(transform.header.stamp.nanosec)
            )
            if (
                self.detection_started_ns is None
                or stamp_ns < self.detection_started_ns
                or stamp_ns == self.last_stamps.get(frame)
            ):
                continue
            age = (now_ns - stamp_ns) / 1e9
            if age < 0.0 or age > self.marker_age:
                continue
            translation = transform.transform.translation
            sample = (
                stamp_ns,
                np.array([
                    translation.x, translation.y, translation.z
                ]),
                yaw_from_quaternion(transform.transform.rotation),
            )
            with self.history_lock:
                self.histories[frame].append(sample)
                self.last_stamps[frame] = stamp_ns

    def stable_marker(self, frame, station):
        """Return a stable mean pose or None while samples are insufficient."""
        with self.history_lock:
            samples = list(self.histories[frame])[-self.minimum_samples:]
        if len(samples) < self.minimum_samples:
            return None
        newest_age = (
            self.get_clock().now().nanoseconds - samples[-1][0]
        ) / 1e9
        if newest_age < 0.0 or newest_age > self.marker_age:
            return None
        xyz = np.asarray([item[1] for item in samples])
        if float(np.max(np.std(xyz, axis=0))) > self.translation_std:
            return None
        yaws = np.radians([item[2] for item in samples])
        mean_yaw = math.atan2(
            float(np.mean(np.sin(yaws))),
            float(np.mean(np.cos(yaws))),
        )
        spread = max(
            abs(wrap_degrees(math.degrees(value - mean_yaw)))
            for value in yaws
        )
        if spread > self.yaw_spread:
            return None
        mean = np.mean(xyz, axis=0)
        return MarkerObservation(
            float(mean[0]),
            float(mean[1]),
            float(mean[2]),
            wrap_degrees(math.degrees(mean_yaw)),
            station,
        )

    def start(self, _request, response):
        """Start the current target pair as a manual compatibility path."""
        accepted, message = self._accept_operation(
            self.pick_marker_id, self.place_marker_id
        )
        response.success = accepted
        response.message = message
        return response

    def execute(self, request, response):
        """Accept one LLM-selected Pick/Place marker pair."""
        accepted, message = self._accept_operation(
            int(request.pick_id), int(request.place_id)
        )
        response.accepted = accepted
        response.message = message
        return response

    @staticmethod
    def _validate_target_ids(pick_id, place_id):
        if not 0 <= int(pick_id) <= 49:
            return 'pick_id must be within the DICT_5X5_50 range 0..49'
        if not 0 <= int(place_id) <= 49:
            return 'place_id must be within the DICT_5X5_50 range 0..49'
        if int(pick_id) == int(place_id):
            return 'pick_id and place_id must be different'
        return ''

    def _configure_detector_targets(self, pick_id, place_id):
        if not self.configure_targets_client.wait_for_service(
            timeout_sec=3.0
        ):
            return False, 'dynamic ArUco target service is unavailable'
        request = ExecutePickPlace.Request()
        request.pick_id = int(pick_id)
        request.place_id = int(place_id)
        completed = threading.Event()
        future = self.configure_targets_client.call_async(request)
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(3.0):
            return False, 'dynamic ArUco target service timed out'
        try:
            response = future.result()
        except Exception as exc:
            return False, f'dynamic ArUco target service failed: {exc}'
        return bool(response.accepted), str(response.message)

    def _accept_operation(self, pick_id, place_id):
        with self.command_lock:
            if self.motion_thread is not None and self.motion_thread.is_alive():
                return False, 'another Pick/Place operation is running'
            error = self._validate_target_ids(pick_id, place_id)
            if error:
                return False, error
            configured, message = self._configure_detector_targets(
                pick_id, place_id
            )
            if not configured:
                return False, message
            self.pick_marker_id = int(pick_id)
            self.place_marker_id = int(place_id)
            self.last_stamps.clear()
            self.set_detection_enabled(False)
            self.stop_event.clear()
            self.external_stop_requested = False
            self.motion_thread = threading.Thread(
                target=self.run_operation, daemon=True
            )
            self.motion_thread.start()
            return True, (
                f'Pick/Place accepted: pick_id={self.pick_marker_id}, '
                f'place_id={self.place_marker_id}'
            )

    def stop(self, _request, response):
        """Request detector shutdown and immediate JetCobot stop."""
        self.external_stop_requested = True
        self.stop_event.set()
        self.publish_work_state('STOP_REQUESTED')
        self.set_detection_enabled(False)
        try:
            with self.serial_lock:
                self.robot.stop()
        except Exception as exc:
            self.publish_work_state('FAILED')
            response.success = False
            response.message = f'stop requested but robot.stop failed: {exc}'
            return response
        response.success = True
        response.message = 'stop requested'
        self.publish_work_state('STOPPED')
        return response

    def run_operation(self):
        """Search stations once, then execute the frozen Pick/Place plan."""
        if not self.operation_lock.acquire(blocking=False):
            return
        try:
            self.publish_work_state('WORK_STARTED')
            self.publish_status(
                '작업 시작: ArUco Pick/Place '
                f'pick_id={self.pick_marker_id}, '
                f'place_id={self.place_marker_id}'
            )
            self.publish_work_state('SEARCHING')
            detections = self.search_stations()
            calibrated = {}
            for role, (observation, target, errors) in detections.items():
                calibrated[role] = target
                error_text = str({
                    key: round(value * 1000.0, 2)
                    for key, value in errors.items()
                })
                common = (
                    f'station={target.station}, '
                    f'raw=({observation.x_m:.4f}, '
                    f'{observation.y_m:.4f}, {observation.z_m:.4f}), '
                    f'H[{target.marker_floor}]-xy='
                    f'({target.x_m:.4f}, {target.y_m:.4f}), '
                    f'yaw={target.yaw_deg:.2f}, '
                    f'floor_errors_mm={error_text}'
                )
                if role == 'pick':
                    self.publish_status(
                        f'pick: pick_floor={target.marker_floor}, {common}'
                    )
                else:
                    destination = destination_level(target.marker_floor)
                    self.publish_status(
                        f'place: support_floor={target.marker_floor}, '
                        f'destination_floor={destination}, '
                        + common
                    )
            steps = self.select_feasible_plan(calibrated)
            self.execute_steps(steps)
            self.publish_work_state('WORK_COMPLETED')
            self.publish_status('작업 종료: Place 완료')
        except Exception as exc:
            self.stop_event.set()
            if self.external_stop_requested:
                self.publish_work_state('STOPPED')
            else:
                self.publish_work_state('FAILED')
            self.publish_status(f'작업 실패 및 정지: {exc}')
            try:
                with self.serial_lock:
                    self.robot.stop()
            except Exception:
                pass
        finally:
            self.set_detection_enabled(False)
            self.active_station = ''
            self.operation_lock.release()

    def select_feasible_plan(self, calibrated):
        """Select independent safe heights and start with a direct plan."""
        pick = calibrated['pick']
        place = calibrated['place']
        destination_floor = destination_level(place.marker_floor)
        pick_data = self.floors.get(pick.marker_floor)
        if pick_data is None or pick_data.pick_z_m is None:
            raise RuntimeError(
                f'pick floor {pick.marker_floor} has no taught Pick Z'
            )
        if destination_floor not in self.floors:
            raise RuntimeError(
                f'place destination floor {destination_floor} is unsupported'
            )
        destination_data = self.floors[destination_floor]
        if destination_data.place_z_m is None:
            raise RuntimeError(
                f'place destination floor {destination_floor} has no '
                'taught Place Z'
            )
        minimum_clearance = float(
            self.parameter('minimum_safe_clearance_m')
        )
        minimum_pick_safe_z = (
            pick_data.pick_z_m + minimum_clearance
        )
        minimum_place_safe_z = (
            destination_data.place_z_m + minimum_clearance
        )
        pick_candidates = safe_z_candidates(
            self.safe_z,
            float(self.parameter('safe_z_lowering_step_m')),
            int(self.parameter('maximum_safe_z_lowering_steps')),
            minimum_pick_safe_z,
        )
        place_candidates = safe_z_candidates(
            self.safe_z,
            float(self.parameter('safe_z_lowering_step_m')),
            int(self.parameter('maximum_safe_z_lowering_steps')),
            minimum_place_safe_z,
        )
        if not pick_candidates:
            raise RuntimeError(
                f'configured Pick safe_z={self.safe_z:.3f} m is below '
                f'required minimum {minimum_pick_safe_z:.3f} m'
            )
        if not place_candidates:
            raise RuntimeError(
                f'configured Place safe_z={self.safe_z:.3f} m is below '
                f'required minimum {minimum_place_safe_z:.3f} m'
            )
        pick_safe_z = pick_candidates[-1]
        place_safe_z = place_candidates[-1]
        with self.serial_lock:
            current_coords = self.robot.get_coords()
        if not isinstance(current_coords, (list, tuple)) or len(
            current_coords
        ) != 6:
            raise RuntimeError(
                f'cannot read current yaw for symmetric selection: '
                f'{current_coords}'
            )
        current_yaw = float(current_coords[5])
        if not math.isfinite(current_yaw):
            raise RuntimeError(
                f'current yaw is non-finite: {current_yaw}'
            )
        pick_nominal_yaw = wrap_degrees(pick.yaw_deg + self.yaw_offset)
        pick_yaw, pick_branch, pick_rotation = select_symmetric_yaw(
            pick_nominal_yaw, current_yaw
        )
        cross_station = pick.station != place.station
        applied_place_offset = (
            self.cross_station_place_yaw_offset
            if cross_station else self.place_yaw_offset
        )
        place_nominal_yaw = wrap_degrees(
            place.yaw_deg + applied_place_offset
        )
        place_yaw, place_branch, place_rotation = select_symmetric_yaw(
            place_nominal_yaw, pick_yaw
        )
        pick = target_with_command_yaw(
            pick, pick_yaw, self.yaw_offset
        )
        place = target_with_command_yaw(
            place, place_yaw, self.yaw_offset
        )
        self.publish_status(
            'symmetric yaw selection: '
            f'current={current_yaw:.2f}, '
            f'pick_nominal={pick_nominal_yaw:.2f}, '
            f'pick_offset={self.yaw_offset:.2f}, '
            f'pick_selected={pick_yaw:.2f}, '
            f'pick_branch={pick_branch:.0f}, '
            f'pick_rotation={pick_rotation:.2f}, '
            f'place_nominal={place_nominal_yaw:.2f}, '
            f'cross_station={cross_station}, '
            f'place_offset={applied_place_offset:.2f}, '
            f'place_selected={place_yaw:.2f}, '
            f'place_branch={place_branch:.0f}, '
            f'place_rotation_from_pick={place_rotation:.2f}; '
            'IK precheck disabled; trying combined Pick approach first: '
            f'pick_safe_z={pick_safe_z:.3f} m, '
            f'place_safe_z={place_safe_z:.3f} m'
        )
        return build_pick_place_steps(
            pick,
            place,
            self.floors,
            pick_safe_z,
            place_safe_z,
            self.yaw_offset,
        )

    def search_stations(self):
        """Scan every pose and classify detections only within its surface."""
        found = {}
        role_frames = {
            'pick': self.pick_frame,
            'place': self.place_frame,
        }
        for station in self.stations:
            remaining = [role for role in role_frames if role not in found]
            self.active_station = station.name
            self.set_detection_enabled(False)
            self.publish_status(
                f'관찰 이동: station={station.name}, '
                f'surface={station.calibration_surface}, wanted={remaining}'
            )
            self.move_observation(station.joint_angles_deg)
            if self.stop_event.wait(
                float(self.parameter('observation_settle_sec'))
            ):
                raise RuntimeError('operation stopped')
            if not remaining:
                self.publish_status(
                    f'{station.name} 관찰 도착: targets already complete'
                )
                continue
            allowed_floors = calibration_levels_for_surface(
                self.floors, station.calibration_surface
            )
            if not allowed_floors:
                self.publish_status(
                    f'{station.name} 탐색 건너뜀: no calibrated levels for '
                    f'surface={station.calibration_surface}'
                )
                continue
            self.set_detection_enabled(True)
            self.publish_status(
                f'ArUco 탐색: station={station.name}, '
                f'levels={list(allowed_floors)}, '
                f'timeout={station.timeout_sec:.1f}s'
            )
            deadline = time.monotonic() + station.timeout_sec
            rejected = set()
            while time.monotonic() < deadline:
                if self.stop_event.wait(0.05):
                    raise RuntimeError('operation stopped')
                for role in remaining:
                    if role in found:
                        continue
                    marker = self.stable_marker(
                        role_frames[role], station.name
                    )
                    if marker is not None:
                        try:
                            target, errors = calibrate_target(
                                marker,
                                allowed_floors,
                                float(self.parameter(
                                    'maximum_floor_error_m'
                                )),
                                float(self.parameter(
                                    'minimum_floor_separation_m'
                                )),
                                float(self.parameter(
                                    'maximum_h_extrapolation_m'
                                )),
                                float(self.parameter(
                                    'command_area_margin_m'
                                )),
                            )
                        except ValueError as exc:
                            if role not in rejected:
                                rejected.add(role)
                                self.publish_status(
                                    f'ArUco 분류 보류: role={role}, '
                                    f'station={station.name}, reason={exc}'
                                )
                            continue
                        found[role] = (marker, target, errors)
                        self.publish_status(
                            f'ArUco 고정: role={role}, '
                            f'station={station.name}, '
                            f'level={target.marker_floor}'
                        )
                if all(role in found for role in remaining):
                    break
            self.set_detection_enabled(False)
            missing = [role for role in role_frames if role not in found]
            if missing:
                self.publish_status(
                    f'{station.name} 탐색 종료: missing={missing}'
                )
        missing = [role for role in role_frames if role not in found]
        if missing:
            raise RuntimeError(
                f'all station poses exhausted; missing ArUco={missing}'
            )
        return found

    def move_observation(self, target):
        """Move to and verify a station-specific six-joint pose."""
        attempts = int(self.parameter('observation_correction_attempts')) + 1
        timeout = int(math.ceil(
            float(self.parameter('observation_motion_timeout_sec'))
        ))
        for attempt in range(1, attempts + 1):
            with self.serial_lock:
                result = self.robot.sync_send_angles(
                    list(target),
                    int(self.parameter('observation_speed')),
                    timeout=timeout,
                )
                measured = self.robot.get_angles()
            if result is False:
                raise RuntimeError('station observation joint move timed out')
            if not isinstance(measured, (list, tuple)) or len(measured) != 6:
                raise RuntimeError(
                    f'cannot read observation joints: {measured}'
                )
            errors = [
                abs(wrap_degrees(float(actual) - float(goal)))
                for actual, goal in zip(measured, target)
            ]
            if max(errors) <= float(
                self.parameter('observation_joint_tolerance_deg')
            ):
                return
            if attempt < attempts:
                self.publish_status(
                    f'observation correction {attempt}: '
                    f'max joint error={max(errors):.2f} deg'
                )
        raise RuntimeError(
            f'observation pose verification failed: errors={errors}'
        )

    def execute_steps(self, steps):
        """Execute only direct coordinate and gripper commands."""
        if len(steps) != 9:
            raise RuntimeError(
                f'unsupported motion plan length: {len(steps)}'
            )
        labels = (
            'PICK: gripper open',
            'PICK: try combined safe XY + vertical gripper approach',
            'PICK: descend',
            'PICK: gripper close',
            'PICK: rise to safe Z',
            'PLACE: safe XY/RPY approach',
            'PLACE: descend',
            'PLACE: verify stopped, then gripper open',
            'PLACE: rise to safe Z',
        )
        self.publish_work_state('PICK_STARTED')
        for index, step in enumerate(steps):
            if self.stop_event.is_set():
                raise RuntimeError('operation stopped')
            if index == 5:
                self.publish_work_state('PICK_COMPLETED')
                self.publish_work_state('PLACE_STARTED')
            self.publish_status(labels[index])
            if step.action == 'move':
                try:
                    self.move_coords(step.pose)
                except CoordinateMoveError as exc:
                    if index != 1:
                        raise
                    self.execute_split_pick_approach(step.pose, exc)
            elif step.action == 'gripper_open':
                self.command_gripper(True)
            elif step.action == 'gripper_open_after_stop':
                self.wait_until_robot_stopped()
                self.publish_status('PLACE: robot stopped; gripper open')
                self.command_gripper(True)
            elif step.action == 'gripper_close':
                self.command_gripper(False)
            else:
                raise RuntimeError(f'unknown motion action: {step.action}')
        self.publish_work_state('PLACE_COMPLETED')

    def execute_split_pick_approach(self, target_pose, direct_error):
        """Retry only a failed Pick approach as XY then vertical RPY."""
        self.publish_status(
            f'PICK combined approach failed ({direct_error}); '
            f'switching to split approach'
        )
        with self.serial_lock:
            self.robot.stop()
        self.wait_until_robot_stopped('PICK fallback')
        with self.serial_lock:
            measured = self.robot.get_coords()
        if not isinstance(measured, (list, tuple)) or len(measured) != 6:
            raise RuntimeError(
                f'PICK fallback cannot read current RPY: {measured}'
            )
        current_rpy = tuple(float(value) for value in measured[3:])
        if not all(math.isfinite(value) for value in current_rpy):
            raise RuntimeError(
                f'PICK fallback current RPY is invalid: {current_rpy}'
            )
        preserve_rpy_pose = (
            target_pose[0],
            target_pose[1],
            target_pose[2],
            *current_rpy,
        )
        self.publish_status(
            'PICK split 1/2: move above marker while preserving RPY=' +
            str([round(value, 2) for value in current_rpy])
        )
        self.move_coords(preserve_rpy_pose)
        self.publish_status(
            'PICK split 2/2: make gripper vertical above marker'
        )
        self.move_coords(target_pose)

    def wait_until_robot_stopped(self, context='PLACE'):
        """Require consecutive hardware stop states before continuing."""
        poll_interval = float(self.parameter('place_stop_poll_interval_sec'))
        required = int(self.parameter('place_stop_required_samples'))
        deadline = time.monotonic() + float(
            self.parameter('place_stop_timeout_sec')
        )
        stopped_samples = 0
        last_state = None
        started = time.monotonic()
        while time.monotonic() < deadline:
            if self.stop_event.wait(poll_interval):
                raise RuntimeError('operation stopped')
            try:
                with self.serial_lock:
                    state = self.robot.is_moving()
            except Exception as exc:
                self.get_logger().warning(
                    f'is_moving polling failed temporarily: {exc}'
                )
                stopped_samples = 0
                continue
            last_state = state
            if state == 0:
                stopped_samples += 1
                if stopped_samples >= required:
                    self.publish_status(
                        f'{context}: hardware stop confirmed: '
                        f'elapsed={time.monotonic() - started:.2f}s, '
                        f'stable_samples={stopped_samples}'
                    )
                    return
            else:
                stopped_samples = 0
        raise RuntimeError(
            f'{context}: robot did not report a stable stop before '
            f'timeout; last_is_moving={last_state}'
        )

    def move_coords(self, pose):
        """Send once, then poll get_coords using our configured tolerance."""
        coords = [
            pose[0] * 1000.0,
            pose[1] * 1000.0,
            pose[2] * 1000.0,
            *pose[3:],
        ]
        self.publish_status(
            'send_coords -> '
            + str([round(value, 2) for value in coords])
        )
        with self.serial_lock:
            result = self.robot.send_coords(
                coords,
                int(self.parameter('speed')),
                mode=0,
            )
        if result is False:
            raise CoordinateMoveError(
                f'JetCobot rejected send_coords: {coords}'
            )

        started = time.monotonic()
        deadline = started + float(self.parameter('motion_timeout_sec'))
        poll_interval = float(
            self.parameter('coordinate_poll_interval_sec')
        )
        required = int(
            self.parameter('coordinate_required_stable_samples')
        )
        stable_samples = 0
        last_measured = None
        last_xyz = None
        last_rpy = None
        while time.monotonic() < deadline:
            if self.stop_event.wait(poll_interval):
                raise RuntimeError('operation stopped')
            try:
                with self.serial_lock:
                    measured = self.robot.get_coords()
            except Exception as exc:
                self.get_logger().warning(
                    f'get_coords polling failed temporarily: {exc}'
                )
                stable_samples = 0
                continue
            if not isinstance(measured, (list, tuple)) or len(measured) != 6:
                stable_samples = 0
                continue
            actual = (
                float(measured[0]) / 1000.0,
                float(measured[1]) / 1000.0,
                float(measured[2]) / 1000.0,
                *[float(value) for value in measured[3:]],
            )
            xyz, rpy = pose_errors(actual, pose)
            last_measured = actual
            last_xyz = xyz
            last_rpy = rpy
            if (
                max(xyz) <= self.position_tolerance
                and max(rpy) <= self.angle_tolerance
            ):
                stable_samples += 1
                if stable_samples >= required:
                    self.publish_status(
                        'get_coords target accepted: '
                        f'elapsed={time.monotonic() - started:.2f}s, '
                        f'xyz_error_mm='
                        f'{[round(value * 1000.0, 2) for value in xyz]}, '
                        f'rpy_error_deg='
                        f'{[round(value, 2) for value in rpy]}, '
                        f'stable_samples={stable_samples}'
                    )
                    return
            else:
                stable_samples = 0
        if last_measured is None:
            raise CoordinateMoveError(
                'get_coords polling timed out without a valid robot pose'
            )
        raise CoordinateMoveError(
            'get_coords polling timed out: '
            f'actual={last_measured}, '
            f'xyz_error_mm='
            f'{[round(value * 1000.0, 2) for value in last_xyz]}, '
            f'rpy_error_deg={[round(value, 2) for value in last_rpy]}'
        )

    def command_gripper(self, open_gripper):
        """Command the direct gripper and wait for completion."""
        value = int(self.parameter(
            'gripper_open_value' if open_gripper
            else 'gripper_closed_value'
        ))
        with self.serial_lock:
            result = self.robot.set_gripper_value(
                value, int(self.parameter('gripper_speed'))
            )
        if result is False:
            raise RuntimeError(f'gripper command failed: value={value}')
        if self.stop_event.wait(float(self.parameter('gripper_wait_sec'))):
            raise RuntimeError('operation stopped')


def main(args=None):
    """Run the direct Pick/Place coordinator."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = HomographyPickPlace()
        executor.add_node(node)
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
