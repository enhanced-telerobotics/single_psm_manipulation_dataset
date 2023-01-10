# Single PSM Simple Tissue Phantom Dataset

![Sample Image](/img_2894.jpg)

The Single PSM Simple Tissue Phantom Dataset is a multimodal dataset of teleoperate manipulations performed using only the right patient side manipulator (PSM) from a da Vinci Research Kit on a silicone tissue phantom stretched over a sponge base layer. The data are collected in different robot and camera configurations relative to the tissue stage, with different surgical tools (Cadiere and Maryland Bipolar forceps) and with different silicone material (Limbs and Things tissue and DragonSkin).

## Download Link


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