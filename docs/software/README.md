# Software

Software for the [MD-300-150](../actuator/MD-300-150/README.md) actuator on Mini Pupper.

| Folder | What it is | Use it when |
| ------ | ---------- | ----------- |
| [servo_sdk](servo_sdk/README.md) | C library (ESP32, SPI) that sends goals to the 12 servos and reads them back | You write firmware that moves the robot |
| [servo_studio](servo_studio/README.md) | Web page served by the robot (Servo Studio) to set, calibrate and watch each servo | You set up, tune or check servos, no code needed |

The actuator itself (specifications, Control Table, SPI frames) is described in [docs/actuator/MD-300-150](../actuator/MD-300-150/README.md).
