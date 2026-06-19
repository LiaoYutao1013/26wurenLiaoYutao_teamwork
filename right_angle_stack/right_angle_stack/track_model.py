import xml.etree.ElementTree as ET

# 从赛道 SDF 中读取静态锥桶位置，供内置感知兜底调试/使用。


def load_cones_from_sdf(path):
    tree = ET.parse(path)
    root = tree.getroot()
    cones = []
    for include in root.findall('.//include'):
        uri = include.findtext('uri', default='')
        name = include.findtext('name', default='')
        pose_text = include.findtext('pose', default='0 0 0 0 0 0')
        pose_values = [float(value) for value in pose_text.split()]

        x = pose_values[0] if len(pose_values) > 0 else 0.0
        y = pose_values[1] if len(pose_values) > 1 else 0.0
        z = pose_values[2] if len(pose_values) > 2 else 0.0

        label = f'{uri} {name}'.lower()
        if 'blue' in label:
            color = 'blue'
        elif 'yellow' in label:
            color = 'yellow'
        elif 'red' in label:
            color = 'red'
        else:
            color = 'unknown'

        cones.append((color, x, y, z))
    return cones
