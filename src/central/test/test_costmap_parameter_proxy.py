from central.costmap_parameter_proxy import remote_costmap_node, tuning_defaults
import pytest


def test_remote_costmap_node_uses_vehicle_namespace():
    assert remote_costmap_node('agv2', 'local') == (
        '/agv2/local_costmap/local_costmap'
    )


def test_global_defaults_match_nav2_configuration():
    defaults = tuning_defaults('global')

    assert defaults['inflation_layer.inflation_radius'] == 0.20
    assert defaults['inflation_layer.cost_scaling_factor'] == 20.0
    assert defaults['keepout_inflation_layer.inflation_radius'] == 0.08
    assert defaults['keepout_inflation_layer.cost_scaling_factor'] == 6.0


@pytest.mark.parametrize('vehicle_id', ['', 'agv3'])
def test_invalid_vehicle_is_rejected(vehicle_id):
    with pytest.raises(ValueError):
        remote_costmap_node(vehicle_id, 'global')
