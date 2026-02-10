# 机械臂结构模型 / Mechanical Structure Models

本目录存放 EL-A3 机械臂的结构设计文件。

This directory contains structural design files for the EL-A3 robotic arm.

## 目录说明 / Directory Description

| 子目录 / Subdirectory | 说明 / Description |
|---|---|
| `step/` | STEP 格式 3D 装配体和零件模型 / STEP format 3D assembly and part models |
| `stl/` | STL 格式网格文件（用于 3D 打印和可视化）/ STL mesh files (for 3D printing and visualization) |
| `drawings/` | 工程图纸（DXF/DWG/PDF）/ Engineering drawings (DXF/DWG/PDF) |

## 相关目录 / Related Directories

- `el_a3_description/meshes/` — ROS2 URDF 使用的 STL 碰撞/可视化模型
- `EDULITE-A3/meshes/` — 原始 URDF 导出的 PART/STL 文件
- `EL_A3_urdf/EL-A3/meshes/` — 早期 URDF 版本的 STL 文件

## 文件命名规范 / File Naming Convention

```
EL-A3_<部件名>_<版本>.<扩展名>
例：EL-A3_base_v2.step
    EL-A3_assembly_full_v1.step
```
