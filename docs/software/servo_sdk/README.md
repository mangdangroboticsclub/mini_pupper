# Servo SDK (ESP32)

The Servo SDK is the C library the minipupper_4_mcu firmware uses to control the 12 [MD-300-150](../../acurator/MD-300-150/README.md) servos of the robot. It runs on ESP-IDF and talks to the four actuator modules (one per leg, 3 servos each) over SPI.

> [!NOTE]
> This page describes the SDK for actuator **firmware 0.0.9**. The SDK is `main/robot/driver_board.c` / `driver_board.h` of the [minipupper_4_mcu](https://github.com/mangdangroboticsclub/minipupper_4_mcu) firmware; the pins are in `main/robot/board_config.h`.

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
  - [Raw Host Link](#raw-host-link)
- [Notes](#notes)

## Hardware Setup

| Item | Value |
| ---- | ----- |
| Host | ESP32-S3 (MangDang MP4 ESP32 CORE, ESP32-S3-WROOM-1-N16R8) |
| Bus | SPI2_HOST, mode 0, 12 [MHz], used only by the four modules |
| MOSI / MISO / CLK | IO6 / IO17 / IO16 |
| CS | One per module: CN3 = IO15, CN4 = IO7, CN5 = IO4, CN6 = IO5 |
| Power | The servo connectors are on the battery directly; there is no power switch pin |

Which leg is on which connector depends on the robot's harness. Choose it with `SERVO_BOARD` in `driver_board.h` and rebuild:

| `SERVO_BOARD` | Front Right (1 ~ 3) | Front Left (4 ~ 6) | Rear Right (7 ~ 9) | Rear Left (10 ~ 12) |
| ------------- | ------------------- | ------------------ | ------------------ | ------------------- |
| 1 (black board) | CN4 | CN5 | CN3 | CN6 |
| **2 (default)** | **CN3** | **CN5** | **CN4** | **CN6** |
| 3 (purple board) | CN5 | CN4 | CN6 | CN3 |

With board 3 the thigh and calf are also swapped inside each leg; the SDK handles that too.

> [!TIP]
> Check the setting before you trust the gait: put the robot on a stand, move servo 1 from Servo Studio and watch which joint answers. Servo 1 is the front right hip.

## Servo ID

Every function takes the **servo ID 1 ~ 12** from the table above. Inside each leg the order is hip, thigh, calf. The SDK maps the ID to the right connector and channel (`db_phys()`), so the rest of the code does not change between boards.

## Quick Start

```c
#include "driver_board.h"

void app_main(void)
{
    if (!driver_board_init())             /* SPI bus + 4 modules            */
        return;
    driver_board_probe_scale();           /* read Range Position(3) once    */

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

All functions can be called from several tasks: one internal lock keeps each SPI exchange whole.

### Setup

| Function | Description |
| -------- | ----------- |
| `bool driver_board_init(void)` | Start the SPI bus and the four modules. Call once at start-up. Returns false if the bus or a module device could not be set up. |
| `float driver_board_probe_scale(void)` | Ask the modules for [Range Position(3)](../../acurator/MD-300-150/README.md#range-position3) and use it as the full scale. Call once after `driver_board_init()`, before the motion task starts. Returns the scale in use. |
| `bool driver_board_probe_answered(void)` | True if a module answered the probe, false if the scale is still the default. |
| `void driver_board_set_scale_dd(float v)` | Full scale of the 0 ~ 1023 position scale, in 0.1 [°] (default 3100 = 310.0 [°], allowed 1000 ~ 4000). Keep it equal to Range Position(3) × 10. |
| `float driver_board_get_scale_dd(void)` | Read the full scale above. |
| `void driver_board_power(bool on)` | Does nothing on this board (no power switch pin). Kept for compatibility. |

### Motion

| Function | Description |
| -------- | ----------- |
| `void driver_board_sync_write(const uint16_t pos[12], const uint16_t cur_mA[12])` | Send all 12 goals in one pass (4 SPI frames). `pos` = goal position 0 ~ 1023 (511 = centre), `cur_mA` = current limit [mA]. Index 0 = servo ID 1. The feedback of the same transaction is cached. |
| `int driver_board_sync_idle(void)` | All 12 servos to Idle (motor off, the joints go limp) in one pass, reading feedback on the way. Returns how many of the 4 modules answered. The next `driver_board_sync_write()` turns position control on again. |
| `bool driver_board_direct(int servo, uint16_t mode, float pos_deg, int16_t current_mA)` | Command one servo. `mode` = `DB_MODE_IDLE` / `DB_MODE_POSITION` / `DB_MODE_TORQUE`, `pos_deg` = angle in the module's own degrees (0 ~ Range Position(3)), `current_mA` = current limit (position) or goal current (torque). The other 2 servos of the module keep their last goal. |
| `void driver_board_stage(int servo, uint16_t mode, float pos_deg, int16_t current_mA, uint16_t kp, uint16_t kd)` | Prepare one servo's goal without sending it. Same units as `driver_board_direct()`. Pass `kp` = `kd` = 0: firmware 0.0.9 ignores them and uses its stored gains. |
| `bool driver_board_commit(void)` | Send everything prepared with `driver_board_stage()` in one pass (4 SPI frames instead of 12). Returns false if a module did not answer with feedback. |
| `bool driver_board_poll(int servo)` | Refresh the feedback of the servo's module without changing its goals. |
| `bool driver_board_poll_board(int board)` | Same for a whole module, `board` = 0 ~ 3 (FR, FL, RR, RL). |

### Feedback

Values come from the last motion or poll call. `ch` = servo ID 1 ~ 12.

| Function | Unit | Control Table |
| -------- | ---- | ------------- |
| `uint16_t driver_board_present_position(int ch)` | 0 ~ 1023 | [Present Position(16)](../../acurator/MD-300-150/README.md#present-position16-22) |
| `int16_t driver_board_present_current(int ch)` | [mA] | [Present Current(17)](../../acurator/MD-300-150/README.md#present-current17-26) |
| `float driver_board_present_temperature(int ch)` | [°C], `DB_TEMP_INVALID` (−273) if the servo never answered | [Present Temperature(18)](../../acurator/MD-300-150/README.md#present-temperature18-31) |
| `bool driver_board_present_fresh(int ch)` | true / false | Did this servo's module answer the last `driver_board_sync_idle()`? |

### Configuration

Read and write the EEPROM area (Control Table No. 0 ~ 12). A write goes to RAM and works at once; save it to keep it after power-off.

| Function | Description |
| -------- | ----------- |
| `bool driver_board_set_param(int servo, int param_id, float value)` | Write one item (RAM). |
| `bool driver_board_get_param(int servo, int param_id, float *out)` | Read one item. |
| `bool driver_board_save_config(int board)` | Store the configuration in flash. `board` = 0 ~ 3, or -1 for all. |
| `bool driver_board_factory_restore(int board)` | Load the factory configuration (RAM). `board` = 0 ~ 3, or -1 for all. |
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
| `DB_LIVE_SUPPLY_CURRENT_MA` (13) | Battery-side current [mA]. Not in the Control Table; check that your module firmware provides it. |

### Raw Host Link

For a host that commands every joint itself, for example a Raspberry Pi running a walking policy at 200 [Hz]. Positions are in the module's own 0.1 [°] units, without the 0 ~ 1023 scale, direction flip or calibration offset.

| Function | Description |
| -------- | ----------- |
| `void driver_board_sync_raw(const uint16_t mode[12], const uint16_t pos_dd[12], const int16_t cur_mA[12], uint8_t *answered, uint8_t *fresh)` | Send all 12 goals in one pass. `mode` = `DB_MODE_IDLE` / `DB_MODE_POSITION` / `DB_MODE_TORQUE`, or `DB_MODE_HOLD` to keep the last position. `answered` / `fresh` (may be NULL): bit L = leg L answered / sent real feedback. |
| `bool driver_board_raw_feedback(int servo, uint16_t *pos_dd, int16_t *cur_mA, int16_t *temp_dc, uint16_t *status)` | What the last `driver_board_sync_raw()` read for one servo: position [0.1 °], current [mA], temperature [0.1 °C], status. Returns false if the module gave no feedback. |
| `void driver_board_raw_invalidate(void)` | Forget the raw feedback until the next `driver_board_sync_raw()`. Call it when a new host session starts. |

## Notes

- A configuration or live request takes two SPI transactions: the module answers in the next one. The SDK keeps the pair together. Each call takes about 1 [ms].
- Pause the motion loop before you change settings, so the gait does not move the servos while you tune them. Servo Studio does this when you press **Connect**.
- `driver_board_bus_lock()` / `driver_board_bus_unlock()` are only needed if another part of the firmware later shares the SPI2 bus.
- The module has no motion profile. Send small position steps (for example every 5 [ms]) for smooth motion.
