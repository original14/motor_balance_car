# MSPM0G3507 平衡车工程

本工程是从 `motor` 双电机闭环工程另存得到的平衡车版本，保留了 TB6612FNG、左右编码器、双轮速度 PID、串口命令和 VOFA+ 遥测基础。

当前代码仍是底盘电机速度环，不包含 IMU 驱动、姿态解算、直立环或转向环。接入平衡车硬件前，请先阅读 [README_BALANCE_CAR.md](README_BALANCE_CAR.md)，并保持电机悬空完成方向与编码器检查。
