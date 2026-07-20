"""Build a link-prefixed MoveIt model for the second JetCobot."""

from xml.etree import ElementTree

from moveit_configs_utils import MoveItConfigsBuilder


ARM2_LINK_PREFIX = 'arm2/'


def _prefix_model_links(urdf_xml, srdf_xml, prefix=ARM2_LINK_PREFIX):
    """Prefix every URDF link and the corresponding SRDF references."""
    urdf_root = ElementTree.fromstring(urdf_xml)
    link_names = {
        element.attrib['name']
        for element in urdf_root.findall('link')
    }

    urdf_root.set('name', 'arm2_jetcobot')
    for element in urdf_root.iter():
        if element.tag == 'link' and element.get('name') in link_names:
            element.set('name', prefix + element.get('name'))
        for attribute in ('link', 'reference'):
            if element.get(attribute) in link_names:
                element.set(attribute, prefix + element.get(attribute))

    srdf_root = ElementTree.fromstring(srdf_xml)
    srdf_root.set('name', 'arm2_jetcobot')
    for element in srdf_root.iter():
        for attribute, value in element.attrib.items():
            if value in link_names:
                element.set(attribute, prefix + value)

    return (
        ElementTree.tostring(urdf_root, encoding='unicode'),
        ElementTree.tostring(srdf_root, encoding='unicode'),
    )


def build_arm2_moveit_config():
    """Return the shared JetCobot MoveIt config with arm2 link names."""
    config = MoveItConfigsBuilder(
        'jetcobot', package_name='jetcobot_moveit_config'
    ).to_moveit_configs()
    urdf_xml, srdf_xml = _prefix_model_links(
        config.robot_description['robot_description'],
        config.robot_description_semantic['robot_description_semantic'],
    )
    config.robot_description['robot_description'] = urdf_xml
    config.robot_description_semantic[
        'robot_description_semantic'
    ] = srdf_xml
    return config
