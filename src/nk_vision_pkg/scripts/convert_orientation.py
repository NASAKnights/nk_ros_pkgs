import yaml
import numpy as np
from scipy.spatial.transform import Rotation

def apply_rotation(pose, rotation_quaternion):
    # Convert the rotation to a rotation matrix
    rotation_matrix = Rotation.from_quat(rotation_quaternion)

    rotation_vector = Rotation.from_quat([pose['rotation']['x'], 
                                          pose['rotation']['y'], 
                                          pose['rotation']['z'],
                                          pose['rotation']['w']])
    rotated_rotation = rotation_vector*rotation_matrix

    # Update the pose with the rotated values
    rotated_rotation = rotated_rotation.as_quat().astype(float)
    pose['rotation']['x'] = float(rotated_rotation[0])
    pose['rotation']['y'] = float(rotated_rotation[1])
    pose['rotation']['z'] = float(rotated_rotation[2])
    pose['rotation']['w'] = float(rotated_rotation[3])

    return pose
def process_yaml_file(file_pathr, file_pathw):
    with open(file_pathr, 'r') as file:
        data = yaml.safe_load(file)

        if 'boards' in data and 'world' in data['boards']:
            # Iterate through poses and apply the rotation
            for pose_entry in data['boards']['world']:
                if 'pose' in pose_entry:
                    pose_entry['pose'] = apply_rotation(pose_entry['pose'], [0.5,-0.5,0.5,-0.5])

    with open(file_pathw, 'w') as file:
        yaml.dump(data, file)

if __name__ == "__main__":
    yaml_file_pathr = "/home/ccresta/workspace/nk_ros_pkgs/src/nk_vision_pkg/config/2025_aruco_layout.yaml"  # Replace with your YAML file path
    yaml_file_pathw = "/home/ccresta/workspace/nk_ros_pkgs/src/nk_vision_pkg/config/2025_aruco_layout_new.yaml"  # Replace with your YAML file path
    process_yaml_file(yaml_file_pathr, yaml_file_pathw)
