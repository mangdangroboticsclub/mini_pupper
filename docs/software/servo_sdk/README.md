# Servo SDK (ESP32)

The Servo SDK is the C library the Mini Pupper ESP32 uses to control its 12 [MD-300-150](../../actuator/MD-300-150/README.md) servos. It runs on ESP-IDF and talks to the four actuator modules (one per leg, 3 servos each) over SPI.

> [!NOTE]
> This page describes the SDK for actuator **firmware 0.0.9**. The SDK is the `driver_board.c` / `driver_board.h` pair of the Mini Pupper ESP32 firmware.

## Contents

- [Hardware Setup](#hardware-setup)
- [Servo ID](#servo-id)
- [Quick Start](#quick-start)
- [API Reference](#api-reference)
  - [Setup](#setup)
  - [Motion](#motion)
  - [Feedback](#feedback)
  - [Configuration](#configuration)
  - [Live Values](#live-values)
- [Notes](#notes)

## Hardware Setup

| Item | Value |
| ---- | ----- |
| Host | ESP32-S3 (MangDang MP4 ESP32 CORE) |
| Bus | SPI2_HOST, mode 0, 12 [MHz] |
| MOSI / MISO / CLK | IO6 / IO17 / IO16 |
| CS | One per module, see below |

| Leg | Connector | CS pin | Servo ID |
| --- | --------- | ------ | -------- |
| Front Right | CN4 | IO7 | 1, 2, 3 |
| Front Left | CN5 | IO4 | 4, 5, 6 |
| Rear Right | CN3 | IO15 | 7, 8, 9 |
| Rear Left | CN6 | IO5 | 10, 11, 12 |

## Servo ID

Every function takes the **servo ID 1 ~ 12** shown above. Inside each leg the order is hip, thigh, calf.

If a robot is wired differently (for example hip and calf plugged the other way round), set `SERVO_BOARD` in `driver_board.h`. The SDK then maps the ID to the right channel, so the rest of the code does not change.

## Quick Start

```c
#include "driver_board.h"

void app_main(void)
{
    driver_board_init();

    uint16_t pos[12], cur[12];
    for (int i = 0; i < 12; i++) {
        pos[i] = 511;   /* centre (0 ~ 1023 scale)          */
        cur[i] = 1000;  /* current limit 1000 [mA]          */
    }

    for (;;) {
        driver_board_sync_write(pos, cur);   /* 4 SPI frames, all 12 servos */

        printf("servo 1: pos %u, %d mA, %.1f C\n",
               driver_board_present_position(1),
               driver_board_present_current(1),
               driver_board_present_temperature(1));

        vTaskDelay(pdMS_TO_TICKS(5));        /* 200 [Hz] */
    }
}
```

## API Reference

### Setup

| Function | Description |
| -------- | ----------- |
| `void driver_board_init(void)` | Start the SPI bus and the four modules. Call once at start-up. |
| `void driver_board_set_scale_dd(float v)` | Full scale of the 0 ~ 1023 position scale, in 0.1 [°] (default 2700 = 270.0 [°]). Keep it equal to [Range Position(3)](../../actuator/MD-300-150/README.md#range-position3) × 10. |
| `float driver_board_get_scale_dd(void)` | Read the full scale above. |

### Motion

| Function | Description |
| -------- | ----------- |
| `void driver_board_sync_write(const uint16_t pos[12], const uint16_t cur_mA[12])` | Send all 12 goals in one pass. `pos` = goal position 0 ~ 1023 (511 = centre), `cur_mA` = current limit [mA]. Index 0 = servo ID 1. Feedback of the same transaction is cached. |
| `bool driver_board_direct(int servo, uint16_t mode, float pos_deg, int16_t current_mA)` | Command one servo. `mode` = `DB_MODE_IDLE` / `DB_MODE_POSITION` / `DB_MODE_TORQUE`, `pos_deg` = angle [°], `current_mA` = current limit (position) or goal current (torque). The other 2 servos of the module keep their last goal. |
| `bool driver_board_poll(int servo)` | Refresh the feedback of one module without changing its goals. |

### Feedback

Values come from the last `driver_board_sync_write()`, `driver_board_direct()` or `driver_board_poll()`. `ch` = servo ID 1 ~ 12.

| Function | Unit | Control Table |
| -------- | ---- | ------------- |
| `uint16_t driver_board_present_position(int ch)` | 0 ~ 1023 | [Present Position(16)](../../actuator/MD-300-150/README.md#present-position16-22) |
| `uint16_t driver_board_present_posdd(int ch)` | 0.1 [°] | Present Position(16), raw |
| `int16_t driver_board_present_current(int ch)` | [mA] | [Present Current(17)](../../actuator/MD-300-150/README.md#present-current17-26) |
| `float driver_board_present_temperature(int ch)` | [°C] | [Present Temperature(18)](../../actuator/MD-300-150/README.md#present-temperature18-31) |

### Configuration

Read and write the EEPROM area (Control Table No. 0 ~ 12). A write goes to RAM and works at once; save it to keep it after power-off.

| Function | Description |
| -------- | ----------- |
| `bool driver_board_set_param(int servo, int param_id, float value)` | Write one item (RAM). |
| `bool driver_board_get_param(int servo, int param_id, float *out)` | Read one item. |
| `bool driver_board_save_config(int board)` | Store the configuration in flash. `board` = 0 ~ 3, or -1 for all. |
| `bool driver_board_factory_restore(int board)` | Load the factory configuration (RAM). `board` = 0 ~ 3, or -1 for all. |
| `bool driver_board_save_config_n(...)`, `driver_board_factory_restore_n(...)` | Same, and report how many modules answered. |
| `const char *driver_board_param_name(int param_id)` | Item name, for example `"kp_position"`. |
| `int driver_board_param_id(const char *name)` | Item ID from its name, -1 if unknown. |

| `param_id` | Control Table |
| ---------- | ------------- |
| `DB_PARAM_REVERSE_POSITION_SENSOR` | Reverse Position Sensor(0) |
| `DB_PARAM_MIN_POSITION_ADC` | Min Position ADC(1) |
| `DB_PARAM_MAX_POSITION_ADC` | Max Position ADC(2) |
| `DB_PARAM_RANGE_POSITION_DEG` | Range Position(3) |
| `DB_PARAM_REVERSE_MOTOR` | Reverse Motor(4) |
| `DB_PARAM_KP_POSITION` | Position P Gain(5) |
| `DB_PARAM_KD_POSITION` | Position D Gain(6) |
| `DB_PARAM_KP_CURRENT` | Current P Gain(7) |
| `DB_PARAM_KFF_CURRENT` | Current Feed-forward Gain(8) |
| `DB_PARAM_MAX_PWM_DUTY_CYCLE` | PWM Limit(9) |
| `DB_PARAM_KI_POSITION` | Position I Gain(10) |
| `DB_PARAM_CONTROL_MODE` | Control Mode(11): `DB_CONTROL_MODE_CURRENT` (0) or `DB_CONTROL_MODE_PWM` (1) |
| `DB_PARAM_PWM_SCALE` | PWM Scale(12) |

### Live Values

`bool driver_board_get_live(int servo, int live_id, float *out)` reads one live value of the control loop (Control Table No. 19 ~ 31, `live_id` = No. − 19).

| `live_id` | Control Table |
| --------- | ------------- |
| `DB_LIVE_POS_ADC` | Position Input ADC(19) |
| `DB_LIVE_CUR_ADC` | Current Input ADC(20) |
| `DB_LIVE_SETPOINT_POS_DEG` | Setpoint Position(21) |
| `DB_LIVE_PRESENT_POS_DEG` | Present Position (float)(22) |
| `DB_LIVE_ERROR_POS_DEG` | Position Error(23) |
| `DB_LIVE_MAX_CURRENT_MA` | Current Limit (in use)(24) |
| `DB_LIVE_SETPOINT_CUR_MA` | Controller Output(25) |
| `DB_LIVE_PRESENT_CUR_MA` | Present Current (float)(26) |
| `DB_LIVE_ERROR_CUR_MA` | Current Error(27) |
| `DB_LIVE_PWM_DUTY` | Present PWM(28) |
| `DB_LIVE_MODE` | Present Mode(29) |
| `DB_LIVE_LOOP_COUNTER` | Loop Counter(30) |
| `DB_LIVE_TEMPERATURE_C` | Present Temperature (float)(31) |

## Notes

- Do not call the configuration or live functions while another task calls `driver_board_sync_write()`. Pause the motion loop first (Servo Studio does this when you press **Connect**).
- A configuration request takes two SPI transactions: the module answers in the next one. Each call takes about 1 [ms].
- The module has no motion profile. Send small position steps (for example every 5 [ms]) for smooth motion.
