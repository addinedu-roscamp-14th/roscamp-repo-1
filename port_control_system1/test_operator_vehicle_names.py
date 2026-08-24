from ros_control_bridge import operator_vehicle_id, operator_vehicle_text


def test_operator_vehicle_id_keeps_ros_names_out_of_dashboard():
    assert operator_vehicle_id('agv1') == 'amr1'
    assert operator_vehicle_id('AGV2') == 'amr2'
    assert operator_vehicle_id('other') == 'other'


def test_operator_vehicle_text_formats_zone_occupancy():
    assert operator_vehicle_text(
        'B-1:agv1;A:FREE;PARK1:FREE;PARK2:agv2'
    ) == 'B-1:amr1;A:FREE;PARK1:FREE;PARK2:amr2'
