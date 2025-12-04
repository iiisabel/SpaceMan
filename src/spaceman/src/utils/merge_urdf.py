import os
import shutil
from pathlib import Path
from xml.dom import minidom
import xml.etree.ElementTree as ET

class URDFMerger:
    def __init__(self, urdf1_path, urdf2_path):
        # Force it under 'spaceman'
        output_dir = root_path /f"{os.path.splitext(os.path.basename(urdf1_path))[0]}_combine_{os.path.splitext(os.path.basename(urdf2_path))[0]}"
        os.makedirs(output_dir, exist_ok=True)

        self.output_dir = output_dir
        self.meshes_dir = output_dir
        self.textures_dir = output_dir
        os.makedirs(self.meshes_dir, exist_ok=True)
        os.makedirs(self.textures_dir, exist_ok=True)

    def _scale_robot(self, root, prefix_name, scale_factor=5.0):
        print(f"\n=== SCALING ROBOT2 BY FACTOR {scale_factor} ===")
        
        scale_factor_3 = scale_factor ** 3  # 体积
        scale_factor_5 = scale_factor ** 5  # 惯性矩
        
        robot_links = []
        robot_joints = []
        
        for elem in root:
            if elem.tag == 'link' and elem.get('name', '').startswith(prefix_name):
                robot_links.append(elem)
            elif elem.tag == 'joint' and elem.get('name', '').startswith(prefix_name):
                robot_joints.append(elem)
        
        print(f"Found {len(robot_links)} links and {len(robot_joints)} joints to scale")
        
        # 几何模型___visual___collision
        mesh_count = 0
        for link in robot_links:
            visual = link.find('visual')
            if visual is not None:
                geometry = visual.find('geometry')
                if geometry is not None:
                    mesh = geometry.find('mesh')
                    if mesh is not None:
                        mesh.set('scale', f"{scale_factor} {scale_factor} {scale_factor}")
                        mesh_count += 1
            collision = link.find('collision')
            if collision is not None:
                geometry = collision.find('geometry')
                if geometry is not None:
                    mesh = geometry.find('mesh')
                    if mesh is not None:
                        mesh.set('scale', f"{scale_factor} {scale_factor} {scale_factor}")
                        mesh_count += 1
        print(f"✅ Scaled {mesh_count} mesh geometries")
        
        # 惯性___mass___origin___inertial
        inertial_count = 0
        for link in robot_links:
            inertial = link.find('inertial')
            if inertial is not None:
                mass_elem = inertial.find('mass')
                if mass_elem is not None:
                    try:
                        original_mass = float(mass_elem.get('value', 0))
                        new_mass = original_mass * scale_factor_3
                        mass_elem.set('value', str(new_mass))
                    except ValueError:
                        print(f"Warning: Invalid mass value in {link.get('name')}")
                origin_elem = inertial.find('origin')
                if origin_elem is not None and origin_elem.get('xyz'):
                    try:
                        xyz_parts = origin_elem.get('xyz').split()
                        if len(xyz_parts) == 3:
                            new_xyz = [str(float(coord) * scale_factor) for coord in xyz_parts]
                            origin_elem.set('xyz', ' '.join(new_xyz))
                    except ValueError:
                        print(f"Warning: Invalid origin coordinates in {link.get('name')}")
                inertia_elem = inertial.find('inertia')
                if inertia_elem is not None:
                    for attr in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']:
                        value = inertia_elem.get(attr)
                        if value and value != '0':
                            try:
                                new_value = float(value) * scale_factor_5
                                inertia_elem.set(attr, str(new_value))
                            except ValueError:
                                print(f"Warning: Invalid inertia value {attr}={value} in {link.get('name')}")
                inertial_count += 1
        print(f"✅ Scaled {inertial_count} inertial parameters")
        
        # 关节位置___xyz
        joint_count = 0
        for joint in robot_joints:
            origin_elem = joint.find('origin')
            if origin_elem is not None and origin_elem.get('xyz'):
                try:
                    xyz_parts = origin_elem.get('xyz').split()
                    if len(xyz_parts) == 3:
                        new_xyz = [str(float(coord) * scale_factor) for coord in xyz_parts]
                        origin_elem.set('xyz', ' '.join(new_xyz))
                        joint_count += 1
                except ValueError:
                    print(f"Warning: Invalid joint origin coordinates in {joint.get('name')}")
        print(f"✅ Scaled {joint_count} joint positions")

    def _find_resource_directory(self, urdf_path, resource_type='meshes', max_depth=5):
        current_dir = Path(urdf_path).parent
        for depth in range(max_depth):
            resource_dir = current_dir / resource_type
            if resource_dir.exists() and resource_dir.is_dir():
                return current_dir
            
            parent_dir = current_dir.parent
            if parent_dir == current_dir:
                break
            current_dir = parent_dir
        return None

    def _extract_resource_path(self, resource_url):
        if resource_url.startswith('package://'):
            resource_url = resource_url.replace('package://', '')
        path_parts = resource_url.split('/')
        
        for i, part in enumerate(path_parts):
            if part in ['meshes', 'textures']:
                return '/'.join(path_parts[i:])
        return ""

    def _copy_used_resources(self, root, urdf1_path, urdf2_path):
        print("\n=== COPYING USED RESOURCES ===")
        
        resources_to_copy = set()
        
        for mesh in root.findall('.//mesh'):
            filename = mesh.get('filename', '')
            if filename.startswith('package://'):
                relative_path = self._extract_resource_path(filename)
                resources_to_copy.add(('mesh', relative_path))
        for texture in root.findall('.//texture'):
            filename = texture.get('filename', '')
            if filename.startswith('package://'):
                relative_path = self._extract_resource_path(filename)
                resources_to_copy.add(('texture', relative_path))
        for material in root.findall('.//material'):
            texture = material.find('texture')
            if texture is not None:
                filename = texture.get('filename', '')
                if filename.startswith('package://'):
                    relative_path = self._extract_resource_path(filename)
                    resources_to_copy.add(('texture', relative_path))
        print(f"Found {len(resources_to_copy)} resources to copy")
        
        urdf1_root = self._find_resource_directory(urdf1_path)
        urdf2_root = self._find_resource_directory(urdf2_path)
        print(f"URDF1 root: {urdf1_root}")
        print(f"URDF2 root: {urdf2_root}")
        
        copied_files = []
        
        for resource_type, relative_path in resources_to_copy:
            source_path = None
            if urdf1_root:
                potential_path = urdf1_root / relative_path
                if potential_path.exists():
                    source_path = potential_path
                    source_root = urdf1_root
                    print(f"Found in URDF1: {relative_path}")
            if source_path is None and urdf2_root:
                potential_path = urdf2_root / relative_path
                if potential_path.exists():
                    source_path = potential_path
                    source_root = urdf2_root
                    print(f"Found in URDF2: {relative_path}")
            
            if source_root:
                if source_path:
                    if resource_type == 'mesh':
                        target_dir = self.meshes_dir
                    else:  # texture
                        target_dir = self.textures_dir
                    
                    relative_dir = os.path.dirname(relative_path)
                    if relative_dir:
                        target_subdir = os.path.join(target_dir, relative_dir)
                        os.makedirs(target_subdir, exist_ok=True)
                    
                    target_path = os.path.join(target_dir, relative_path)
                    shutil.copy2(source_path, target_path)
                    copied_files.append((resource_type, relative_path, target_path))
                    print(f"✅ Copied {resource_type}: {relative_path}")
                else:
                    print(f"❌ Resource not found: {source_path}")
            else:
                print(f"WARNING: Could not determine source for: {relative_path}")
        
        print(f"Successfully copied {len(copied_files)} files")
        return copied_files

    def _update_resource_paths(self, root):
        print("\n=== UPDATING RESOURCE PATHS ===")
        for mesh in root.findall('.//mesh'):
            filename = mesh.get('filename', '')
            if filename.startswith('package://'):
                relative_path = self._extract_resource_path(filename)
                new_path = os.path.join('./', relative_path)
                mesh.set('filename', new_path)
                print(f"Updated mesh: {filename} -> {new_path}")
        for texture in root.findall('.//texture'):
            filename = texture.get('filename', '')
            if filename.startswith('package://'):
                relative_path = self._extract_resource_path(filename)
                new_path = os.path.join('./', relative_path)
                texture.set('filename', new_path)
                print(f"Updated texture: {filename} -> {new_path}")
        for material in root.findall('.//material'):
            texture = material.find('texture')
            if texture is not None:
                filename = texture.get('filename', '')
                if filename.startswith('package://'):
                    relative_path = self._extract_resource_path(filename)
                    new_path = os.path.join('./', relative_path)
                    texture.set('filename', new_path)
                    print(f"Updated material texture: {filename} -> {new_path}")

    def _clean_xml_format(self, xml_str):
        lines = []
        for line in xml_str.split('\n'):
            stripped_line = line.strip()
            if stripped_line:
                lines.append(line.rstrip())
        return '\n'.join(lines)
    
    def merge_urdfs(self, urdf1_path, urdf2_path, urdf1_name='robot1', urdf2_name='robot2', output_name='combined_robot',
                     parent_link='robot1_base_link', child_link='robot2_base_link',
                     connection_xyz='0 0 0', connection_rpy='0 0 0', scale_robot=None):
        """
        Merging two URDF files
        ATTENTION : RPY (order) x, y, z AND external rotation

        Corner cases:
        1. For potential naming conflicts of links and joints after merging, add prefixes before merging
        """
        try:
            tree1 = ET.parse(urdf1_path)
            tree2 = ET.parse(urdf2_path)
        except Exception as e:
            print(f"Error parsing URDF files: {e}")
            return None

        root = ET.Element('robot', {'name': output_name})

        robot1_root = tree1.getroot()
        for elem in robot1_root:
            if elem.tag in ['link', 'joint']:
                old_name = elem.get('name')
                new_name = f"{urdf1_name}_{old_name}" if not old_name.startswith(urdf1_name) else old_name
                elem.set('name', new_name)
                
                if elem.tag == 'joint':
                    parent = elem.find('parent')
                    child = elem.find('child')
                    if parent is not None:
                        parent.set('link', f"{urdf1_name}_{parent.get('link')}" if not parent.get('link').startswith(urdf1_name) else parent.get('link'))
                    if child is not None:
                        child.set('link', f"{urdf1_name}_{child.get('link')}" if not child.get('link').startswith(urdf1_name) else child.get('link'))
                    # additional : mimic joint
                    mimic = elem.find('mimic')
                    if mimic is not None:
                        mimic.set('joint', f"{urdf1_name}_{mimic.get('joint')}" if not mimic.get('joint').startswith(urdf1_name) else mimic.get('joint'))
            root.append(elem)

        robot2_root = tree2.getroot()
        for elem in robot2_root:
            if elem.tag in ['link', 'joint']:
                old_name = elem.get('name')
                new_name = f"{urdf2_name}_{old_name}" if not old_name.startswith(urdf2_name) else old_name
                elem.set('name', new_name)
                
                if elem.tag == 'joint':
                    parent = elem.find('parent')
                    child = elem.find('child')
                    if parent is not None:
                        parent.set('link', f"{urdf2_name}_{parent.get('link')}" if not parent.get('link').startswith(urdf2_name) else parent.get('link'))
                    if child is not None:
                        child.set('link', f"{urdf2_name}_{child.get('link')}" if not child.get('link').startswith(urdf2_name) else child.get('link'))
                    # deal with mimic joint
                    mimic = elem.find('mimic')
                    if mimic is not None:
                        mimic.set('joint', f"{urdf2_name}_{mimic.get('joint')}" if not mimic.get('joint').startswith(urdf2_name) else mimic.get('joint'))
            root.append(elem)

        connection_joint = ET.SubElement(root, 'joint', {
            'name': f"{urdf1_name}_{urdf2_name}_connection_joint", 
            'type': 'fixed'
        })
        
        parent_link_final = f"{urdf1_name}_{parent_link}" if not parent_link.startswith(urdf1_name) else parent_link
        child_link_final = f"{urdf2_name}_{child_link}" if not child_link.startswith(urdf2_name) else child_link
        parent_elem = ET.SubElement(connection_joint, 'parent', {
            'link': parent_link_final
        })
        child_elem = ET.SubElement(connection_joint, 'child', {
            'link': child_link_final
        })
        origin = ET.SubElement(connection_joint, 'origin', {
            'xyz': connection_xyz,
            'rpy': connection_rpy
        })

        if scale_robot is not None:
            self._scale_robot(root, urdf2_name, scale_robot)

        copied_files = self._copy_used_resources(root, urdf1_path, urdf2_path)
        self._update_resource_paths(root)
        rough_string = ET.tostring(root, encoding='utf-8')

        xml_str = minidom.parseString(rough_string).toprettyxml(indent="  ")
        xml_str = self._clean_xml_format(xml_str)

        file_path = os.path.join(self.output_dir, f'{output_name}.urdf')
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(xml_str)

        print(f"URDF merged and saved: {file_path}")
        print(f"Connection: {parent_link_final} -> {child_link_final}")
        return file_path
    
current_file_path = Path(__file__).resolve().parent
root_path = current_file_path.parent.parent
asset_path = root_path / 'src' / 'assets'

urdf1_path = asset_path / 'urdf' / 'satellite' / 'urdf' / 'satellite.urdf'
urdf2_path = asset_path / 'urdf' / 'panda_bullet' / 'panda.urdf'

urdf_merger = URDFMerger(urdf1_path, urdf2_path)
merged_urdf_path = urdf_merger.merge_urdfs(
    urdf1_path, 
    urdf2_path, 
    urdf1_name = os.path.splitext(os.path.basename(urdf1_path))[0], 
    urdf2_name = os.path.splitext(os.path.basename(urdf2_path))[0], 
    output_name = f"{os.path.splitext(os.path.basename(urdf1_path))[0]}_combine_{os.path.splitext(os.path.basename(urdf2_path))[0]}",
    parent_link = 'attachment',
    child_link = 'panda_link0',
    connection_xyz = '0 0 1.0',
    connection_rpy = '0 0 0',
    scale_robot = 3.0
)

