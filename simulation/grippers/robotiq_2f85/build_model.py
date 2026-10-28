"""Reproduce the Kinova-derived MJCF and mesh conversion (stdlib + NumPy).

The pinned Xacro and COLLADA files in upstream/ are the source of truth.
Only identity COLLADA scene transforms and triangle primitives are supported.
Run from any directory; generated paths are relative to this script.
"""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parent
PREFIX = 'rh_2f85_'
XACRO = '{http://ros.org/wiki/xacro}'


def save(root, path):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def convert_mesh(src, dst):
    tree = ET.parse(src).getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    for node in tree.findall('.//c:node', ns):
        if any(child.tag.rsplit('}', 1)[-1] != 'instance_geometry' for child in node):
            raise ValueError('Unexpected scene transform in ' + str(src))
    geometries = tree.findall('.//c:geometry', ns)
    if len(geometries) != 1:
        raise ValueError('Expected one geometry')
    mesh = geometries[0].find('c:mesh', ns)
    sources = {s.get('id'): s for s in mesh.findall('c:source', ns)}
    vertices = {v.get('id'): v for v in mesh.findall('c:vertices', ns)}
    faces = []
    for primitive in mesh:
        tag = primitive.tag.rsplit('}', 1)[-1]
        if tag in ('source', 'vertices'):
            continue
        if tag != 'triangles':
            raise ValueError('Only triangle primitives supported')
        inputs = primitive.findall('c:input', ns)
        vertex = next(i for i in inputs if i.get('semantic') == 'VERTEX')
        position = next(i for i in vertices[vertex.get('source')[1:]]
                        if i.get('semantic') == 'POSITION')
        source = sources[position.get('source')[1:]]
        accessor = source.find('c:technique_common/c:accessor', ns)
        if int(accessor.get('stride', '1')) != 3 or int(accessor.get('offset', '0')) != 0:
            raise ValueError('Unsupported position accessor')
        points = np.fromstring(source.find('c:float_array', ns).text, sep=' ').reshape(-1, 3)
        # Kinova Xacro scales each of these DAE meshes by 0.001.
        points *= 0.001
        width = 1 + max(int(i.get('offset', '0')) for i in inputs)
        indices = np.fromstring(primitive.find('c:p', ns).text, sep=' ', dtype=int).reshape(-1, width)
        triangles = indices[:, int(vertex.get('offset', '0'))].reshape(-1, 3)
        if len(triangles) != int(primitive.get('count')):
            raise ValueError('Triangle count mismatch')
        faces.extend(points[triangles])
    with dst.open('wb') as out:
        out.write(b'Kinova ROS Kortex mesh, units metres'.ljust(80, b'\0'))
        out.write(struct.pack('<I', len(faces)))
        for triangle in faces:
            normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            normal /= max(np.linalg.norm(normal), 1e-30)
            out.write(struct.pack('<12fH', *normal, *triangle.ravel(), 0))


def inertial(body, original):
    src = original.find('inertial')
    values = src.find('inertia').attrib
    ET.SubElement(body, 'inertial', pos=src.find('origin').get('xyz'),
                  mass=src.find('mass').get('value'),
                  fullinertia=' '.join(values[k] for k in ('ixx','iyy','izz','ixy','ixz','iyz')))


def build():
    parts = ('inner_finger', 'inner_knuckle', 'outer_finger', 'outer_knuckle')
    for name in parts:
        for suffix in ('', '_visual'):
            convert_mesh(ROOT/'upstream'/(name+suffix+'.dae'), ROOT/'assets'/(name+suffix+'.stl'))
    xacro = ET.parse(ROOT/'upstream/robotiq_arg2f_85_model_macro.xacro').getroot()
    macros = {m.get('name'): m for m in xacro.findall(XACRO+'macro')}
    base_macro = ET.parse(ROOT/'upstream/robotiq_arg2f.xacro').getroot().find(XACRO+'macro')
    defs = ET.Element('mujoco')
    defaults = ET.SubElement(ET.SubElement(defs, 'default'), 'default', {'class': PREFIX+'mechanism'})
    # These are simulation tuning parameters, not Kinova physical measurements.
    ET.SubElement(defaults, 'joint', damping='0.05', frictionloss='0', armature='0.0001',
                  limited='true', solreflimit='0.005 1')
    assets = ET.SubElement(defs, 'asset')
    for name in ('base',)+tuple(n+s for n in parts for s in ('','_visual')):
        ET.SubElement(assets, 'mesh', name=PREFIX+name, file='robotiq_2f85/assets/'+name+'.stl')
    bodyroot = ET.Element('mujoco')
    # In GR3's end-effector frame the old hand points along -Z.
    # RotX(pi) is the GR3 adapter; Kinova's original RotZ(1.57) is retained below.
    mount = ET.SubElement(bodyroot, 'body', name=PREFIX+'mount', quat='0 1 0 0', childclass=PREFIX+'mechanism')
    base = ET.SubElement(mount, 'body', name=PREFIX+'base', quat='0.7073882692 0 0 0.7068251811')
    inertial(base, base_macro.find('link'))

    def geoms(body, meshname):
        visual = meshname if meshname == 'base' else meshname+'_visual'
        ET.SubElement(body, 'geom', name=body.get('name')+'_visual', type='mesh',
                      mesh=PREFIX+visual, contype='0', conaffinity='0', group='2', mass='0', rgba='0.18 0.18 0.18 1')
        ET.SubElement(body, 'geom', name=body.get('name')+'_collision', type='mesh',
                      mesh=PREFIX+meshname, group='3', mass='0', contype='1', conaffinity='1')

    geoms(base, 'base')
    equality = ET.SubElement(defs, 'equality')
    contact = ET.SubElement(defs, 'contact')
    main_joint = PREFIX+'finger_joint'
    for side, sign, rotation in [('left', -1, '0 0 0 1'), ('right', 1, '1 0 0 0')]:
        stem = PREFIX+side+'_'
        outer = ET.SubElement(base, 'body', name=stem+'outer_knuckle', pos=f'0 {sign*0.0306011} 0.054904', quat=rotation)
        inertial(outer, macros['outer_knuckle'].find('link'))
        jointname = main_joint if side == 'left' else stem+'outer_knuckle_joint'
        ET.SubElement(outer, 'joint', name=jointname, axis='1 0 0', range='0 0.8' if side=='left' else '0 0.81')
        geoms(outer, 'outer_knuckle')
        if side == 'right':
            ET.SubElement(equality, 'joint', joint1=jointname, joint2=main_joint, polycoef='0 1 0 0 0', solref='0.005 1')
        finger = ET.SubElement(outer, 'body', name=stem+'outer_finger', pos='0 0.0315 -0.0041')
        inertial(finger, macros['outer_finger'].find('link'))
        geoms(finger, 'outer_finger')
        inner = ET.SubElement(finger, 'body', name=stem+'inner_finger', pos='0 0.0061 0.0471')
        inertial(inner, macros['inner_finger'].find('link'))
        ET.SubElement(inner, 'joint', name=stem+'inner_finger_joint', axis='1 0 0', range='-0.8757 0.8757')
        ET.SubElement(equality, 'joint', joint1=stem+'inner_finger_joint', joint2=main_joint, polycoef='0 -1 0 0 0', solref='0.005 1')
        geoms(inner, 'inner_finger')
        pad = ET.SubElement(inner, 'body', name=stem+'pad', pos='0 -0.0220203446692936 0.03242')
        # Original pad has no inertial: fixed, massless geometry follows its parent.
        ET.SubElement(pad, 'geom', name=stem+'pad_contact', type='box', size='0.011 0.003175 0.01875',
                      mass='0', contype='1', conaffinity='1', rgba='0.65 0.65 0.65 1', friction='0.7 0.005 0.0001')
        ET.SubElement(pad, 'site', name=stem+'pad_center', size='0.001', rgba='0 0 0 0')
        knuckle = ET.SubElement(base, 'body', name=stem+'inner_knuckle', pos=f'0 {sign*0.0127} 0.06142', quat=rotation)
        inertial(knuckle, macros['inner_knuckle'].find('link'))
        geoms(knuckle, 'inner_knuckle')
        ET.SubElement(knuckle, 'joint', name=stem+'inner_knuckle_joint', axis='1 0 0', range='0 0.8757')
        ET.SubElement(equality, 'joint', joint1=stem+'inner_knuckle_joint', joint2=main_joint, polycoef='0 1 0 0 0', solref='0.005 1')
        # Prevent overlapping linkage collision meshes fighting the mimic constraints.
        for a,b in [('base', side+'_outer_knuckle'), ('base', side+'_inner_knuckle'),
                    (side+'_outer_knuckle', side+'_inner_knuckle'),
                    (side+'_outer_finger', side+'_inner_knuckle'),
                    (side+'_inner_finger', side+'_inner_knuckle')]:
            ET.SubElement(contact, 'exclude', body1=PREFIX+a, body2=PREFIX+b)
    actuator = ET.SubElement(defs, 'actuator')
    # Conservative initial simulation tuning, not the Xacro's 1000 N*m effort bound.
    ET.SubElement(actuator, 'position', name=PREFIX+'actuator', joint=main_joint, kp='20',
                  ctrllimited='true', ctrlrange='0 0.8', forcelimited='true', forcerange='-2 2')
    scene = ET.Element('mujoco', model='kinova_robotiq_2f85_standalone')
    ET.SubElement(scene, 'compiler', angle='radian', autolimits='true', meshdir='..')
    ET.SubElement(scene, 'option', timestep='0.001', gravity='0 0 -9.81')
    ET.SubElement(scene, 'include', file='definitions.xml')
    world = ET.SubElement(scene, 'worldbody')
    ET.SubElement(world, 'include', file='body.xml')
    save(defs, ROOT/'definitions.xml')
    save(bodyroot, ROOT/'body.xml')
    save(scene, ROOT/'standalone.xml')


if __name__ == '__main__':
    build()
