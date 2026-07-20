"""Tests for the arm2-prefixed MoveIt robot model."""

from xml.etree import ElementTree

from arm2.arm2_moveit_config import _prefix_model_links


def test_prefixes_urdf_links_and_srdf_references():
    urdf = """
    <robot name="robot">
      <link name="base_link"/>
      <link name="TCP"/>
      <joint name="joint" type="fixed">
        <parent link="base_link"/>
        <child link="TCP"/>
      </joint>
    </robot>
    """
    srdf = """
    <robot name="robot">
      <group name="arm_group">
        <chain base_link="base_link" tip_link="TCP"/>
      </group>
      <virtual_joint name="world_joint" type="fixed"
        parent_frame="world" child_link="base_link"/>
    </robot>
    """

    prefixed_urdf, prefixed_srdf = _prefix_model_links(urdf, srdf)
    urdf_root = ElementTree.fromstring(prefixed_urdf)
    srdf_root = ElementTree.fromstring(prefixed_srdf)

    assert [link.get('name') for link in urdf_root.findall('link')] == [
        'arm2/base_link',
        'arm2/TCP',
    ]
    assert urdf_root.find('joint/parent').get('link') == 'arm2/base_link'
    assert urdf_root.find('joint/child').get('link') == 'arm2/TCP'
    assert srdf_root.find('group/chain').get('base_link') == (
        'arm2/base_link'
    )
    assert srdf_root.find('group/chain').get('tip_link') == 'arm2/TCP'
    assert srdf_root.find('virtual_joint').get('parent_frame') == 'world'
    assert srdf_root.find('virtual_joint').get('child_link') == (
        'arm2/base_link'
    )
