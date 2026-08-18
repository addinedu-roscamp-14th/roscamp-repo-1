#!/usr/bin/env python3

"""Expose remote AMR costmap parameters as central ROS parameters for RQT."""

from __future__ import annotations

import threading

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python


def remote_costmap_node(vehicle_id, scope):
    vehicle = str(vehicle_id).strip('/')
    costmap_scope = str(scope).strip().lower()
    if vehicle not in ('agv1', 'agv2'):
        raise ValueError('vehicle_id must be agv1 or agv2')
    if costmap_scope not in ('global', 'local'):
        raise ValueError('scope must be global or local')
    return f'/{vehicle}/{costmap_scope}_costmap/{costmap_scope}_costmap'


def tuning_defaults(scope):
    if scope == 'global':
        return {
            'inflation_layer.inflation_radius': 0.20,
            'inflation_layer.cost_scaling_factor': 20.0,
            'keepout_inflation_layer.inflation_radius': 0.08,
            'keepout_inflation_layer.cost_scaling_factor': 6.0,
        }
    if scope == 'local':
        return {
            'inflation_layer.inflation_radius': 0.20,
            'inflation_layer.cost_scaling_factor': 10.0,
        }
    raise ValueError('scope must be global or local')


class CostmapParameterProxy(Node):
    """Mirror a remote costmap's tunable parameters into the central graph."""

    def __init__(self):
        super().__init__('costmap_parameter_proxy')
        identity = ParameterDescriptor(read_only=True)
        self.declare_parameter('vehicle_id', 'agv1', identity)
        self.declare_parameter('scope', 'global', identity)
        self.vehicle_id = str(self.get_parameter('vehicle_id').value)
        self.scope = str(self.get_parameter('scope').value)
        self.remote_node = remote_costmap_node(self.vehicle_id, self.scope)
        self.parameter_names = tuple(tuning_defaults(self.scope))
        self._syncing = False
        self._initial_sync_started = False
        self._service_timeout_sec = 5.0
        for name, value in tuning_defaults(self.scope).items():
            self.declare_parameter(name, value)

        callback_group = ReentrantCallbackGroup()
        self._get_client = self.create_client(
            GetParameters,
            f'{self.remote_node}/get_parameters',
            callback_group=callback_group,
        )
        self._set_client = self.create_client(
            SetParameters,
            f'{self.remote_node}/set_parameters',
            callback_group=callback_group,
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        self._sync_timer = self.create_timer(
            1.0, self._start_initial_sync, callback_group=callback_group)
        self.get_logger().info(
            f'RQT costmap proxy ready: {self.get_fully_qualified_name()} '
            f'-> {self.remote_node}')

    def _start_initial_sync(self):
        if self._initial_sync_started or not self._get_client.service_is_ready():
            return
        self._initial_sync_started = True
        request = GetParameters.Request()
        request.names = list(self.parameter_names)
        future = self._get_client.call_async(request)
        future.add_done_callback(self._finish_initial_sync)

    def _finish_initial_sync(self, future):
        try:
            response = future.result()
            values = [parameter_value_to_python(value) for value in response.values]
            if len(values) != len(self.parameter_names):
                raise RuntimeError('remote response length mismatch')
            self._syncing = True
            try:
                results = self.set_parameters([
                    Parameter(name=name, value=value)
                    for name, value in zip(self.parameter_names, values)
                ])
            finally:
                self._syncing = False
            if not all(result.successful for result in results):
                raise RuntimeError('could not mirror remote parameter values')
            self.get_logger().info(
                f'Loaded current parameters from {self.remote_node}')
        except Exception as exc:
            self._initial_sync_started = False
            self.get_logger().warning(
                f'Costmap parameter initial sync failed: {exc}')

    def _on_parameters_changed(self, parameters):
        if self._syncing:
            return SetParametersResult(successful=True)
        changed = [
            parameter for parameter in parameters
            if parameter.name in self.parameter_names
        ]
        if not changed:
            return SetParametersResult(successful=True)
        if not self._set_client.wait_for_service(timeout_sec=0.2):
            return SetParametersResult(
                successful=False,
                reason=f'{self.remote_node} parameter service unavailable',
            )
        request = SetParameters.Request()
        request.parameters = [parameter.to_parameter_msg() for parameter in changed]
        future = self._set_client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(self._service_timeout_sec):
            return SetParametersResult(
                successful=False,
                reason=f'{self.remote_node} parameter request timed out',
            )
        try:
            response = future.result()
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        failures = [result.reason for result in response.results if not result.successful]
        if failures:
            return SetParametersResult(
                successful=False,
                reason='; '.join(filter(None, failures)) or 'remote rejected value',
            )
        self.get_logger().info(
            f'Updated {self.remote_node}: '
            + ', '.join(f'{item.name}={item.value}' for item in changed)
        )
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapParameterProxy()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
