# MSPM0G3507 balance car

This CCS Theia project targets the MSPM0G3507 and contains:

- ICM42688-P SPI sampling and a 500 Hz complementary-filter attitude estimate;
- LEVEL/IMUZERO pitch calibration;
- a 500 Hz balance PID/PD controller;
- existing independent left/right 100 Hz encoder speed PID loops;
- TB6612FNG output, runtime serial tuning and VOFA telemetry;
- stop, sensor-validity, start-angle, fall-angle and encoder feedback protection.

The current balance architecture is cascaded: the balance controller produces
wheel-speed targets, and the existing left/right speed PIDs produce the final
PWM commands. Balance mode remains OFF after reset and motors do not start
automatically.

Read these before hardware testing:

- [README_BALANCE_CONTROL.md](README_BALANCE_CONTROL.md)
- [README_BALANCE_SPEED_CASCADE.md](README_BALANCE_SPEED_CASCADE.md)
- [README_MOTOR_TEST.md](README_MOTOR_TEST.md)
- [README_ICM42688.md](README_ICM42688.md)
