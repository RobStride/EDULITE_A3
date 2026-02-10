# 电路板文件 / Electronics & PCB Files

本目录存放 EL-A3 机械臂相关的电路板设计文件和电气资料。

This directory contains PCB design files and electrical documentation for the EL-A3 robotic arm.

## 目录说明 / Directory Description

| 子目录 / Subdirectory | 说明 / Description |
|---|---|
| `schematic/` | 原理图文件（KiCad/AD/PDF）/ Schematic files (KiCad/Altium/PDF) |
| `pcb/` | PCB 布局文件 / PCB layout files |
| `gerber/` | Gerber 生产文件 / Gerber manufacturing files |
| `bom/` | 物料清单 / Bill of Materials |
| `datasheet/` | 元器件数据手册 / Component datasheets |

## 相关资料 / Related Documentation

- `电机通信协议汇总.md` — Robstride 电机 CAN 通信协议
- `el_a3_hardware/` — ROS2 CAN 驱动代码

## 文件命名规范 / File Naming Convention

```
EL-A3_<板卡名>_<版本>.<扩展名>
例：EL-A3_driver_board_v1.kicad_pcb
    EL-A3_power_board_schematic_v2.pdf
```
