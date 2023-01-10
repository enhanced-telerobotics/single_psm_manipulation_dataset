# Single PSM Simple Tissue Phantom Dataset

![Sample Image](/img_2894.jpg)

The Single PSM Simple Tissue Phantom Dataset is a multimodal dataset of teleoperate manipulations performed using only the right patient side manipulator (PSM) from a da Vinci Research Kit on a silicone tissue phantom stretched over a sponge base layer. The data are collected in different robot and camera configurations relative to the tissue stage, with different surgical tools (Cadiere and Maryland Bipolar forceps) and with different silicone material (Limbs and Things tissue and DragonSkin).

## Download Link


If you use this dataset, please cite our paper where the dataset was first presented.

Z. Chua, A. M. Jarc, and A. M. Okamura, “Toward Force Estimation in Robot-Assisted Surgery using Deep Learning with Vision and Robot State,” in 2021 IEEE International Conference on Robotics and Automation (ICRA), May 2021, pp. 12335–12341. doi: 10.1109/ICRA48506.2021.9560945.

@inproceedings{chuaForceEstimationRobotAssisted2021,
  title = {Toward Force Estimation in Robot-Assisted Surgery Using Deep Learning with Vision and Robot State},
  booktitle = {2021 IEEE International Conference on Robotics and Automation (ICRA)},
  author = {Chua, Zonghe and Jarc, Anthony M. and Okamura, Allison M.},
  year = {2021},
  month = may,
  pages = {12335--12341},
  doi = {10.1109/ICRA48506.2021.9560945},
}


## Dataset Format

### File Naming Convention

The data is in the ROS ".bag" format. With the nomenclature: Configuration_Material_Tool_Index.bag


| Configuration      | Description |
| -------------------| ----------- |
| C                  | Reference Configuration      |
| R1                 | Robot Base and Camera Shift -1.5cm in X in robot base frame coords |
| R2                 | Robot Base and Camera Shift -3cm in X in robot base frame coords |
| R3                 | Robot Base and Camera Shift -4cm in X in robot base frame coords |
| L1                 | Robot Base and Camera Shift +1.5cm in X in robot base frame coords |
| L2                 | Robot Base and Camera Shift +3cm in X in robot base frame coords |
| L3                 | Robot Base and Camera Shift +4cm in X in robot base frame coords |
| Z1                 | Robot Base and Camera Shift -0.635cm in Z in robot base frame coords |
| Z2                 | Robot Base and Camera Shift -1.270cm in Z in robot base frame coords |
| Z3                 | Robot Base and Camera Shift -1.905cm in Z in robot base frame coords|

| Material      | Description |
| --------------| ----------- |
| M1            | Limbs and Things|
| M2            | DragonSkin      |

| Tool      | Description |
| ----------| ----------- |
| T1        | Cadiere     |
| T2        | Maryland    |

### ROS Topics

| Topic   | Type |
|---------|------|
|/camera/left/image_color/compressed     |sensor_msgs/CompressedImage
|/camera/right/image_color/compressed    |sensor_msgs/CompressedImage
|/dvrk/PSM2/jacobian_body                |std_msgs/Float64MultiArray
|/dvrk/PSM2/jacobian_spatial             |std_msgs/Float64MultiArray
|/dvrk/PSM2/position_cartesian_current   |geometry_msgs/PoseStamped
|/dvrk/PSM2/position_cartesian_desired   |geometry_msgs/PoseStamped
|/dvrk/PSM2/state_jaw_current            |sensor_msgs/JointState
|/dvrk/PSM2/state_jaw_desired            |sensor_msgs/JointState
|/dvrk/PSM2/state_joint_current          |sensor_msgs/JointState
|/dvrk/PSM2/state_joint_desired          |sensor_msgs/JointState
|/dvrk/PSM2/twist_body_current           |geometry_msgs/TwistStamped
|/dvrk/PSM2/wrench_body_current          |geometry_msgs/WrenchStamped
|/force_sensor                           |geometry_msgs/WrenchStamped
