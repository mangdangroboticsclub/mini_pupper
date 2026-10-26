# Servo Studio

Servo Studio is part of the MPX app, the web app the robot serves itself. With it you can read and change the settings of each [MD-300-150](../../actuator/MD-300-150/README.md) servo, calibrate its position sensor and watch it live. Nothing to install: any phone or computer with a browser works.

> [!NOTE]
> This page describes Servo Studio of the [mpx-dog](https://github.com/mangdangroboticsclub/mpx-dog) firmware for actuator **firmware 0.0.9**. The page is `pwa-claw/src/lib/ServoStudio.svelte`; it uses the [HTTP API](#http-api) below, which calls the [Servo SDK](../servo_sdk/README.md).

## Contents

- [Open Servo Studio](#open-servo-studio)
- [Connect](#connect)
- [Choose a Servo](#choose-a-servo)
- [Settings](#settings)
  - [Gain Presets](#gain-presets)
  - [Save to Flash](#save-to-flash)
- [Torque and Live Poll](#torque-and-live-poll)
- [Pot Calibration](#pot-calibration)
- [Graph](#graph)
- [Disconnect](#disconnect)
- [HTTP API](#http-api)

## Open Servo Studio

1. Connect your phone or computer to the robot's own Wi-Fi hotspot (**MPX-Dog …**), or to the same Wi-Fi as the robot.
2. Open the robot's address in a browser. On the hotspot this is `http://192.168.2.1`.
3. Go to **Settings** → **MangDang Servo Studio**.

## Connect

Turn on **Connect**. The robot pauses walking (studio mode) so the servos can be read and changed safely, and Servo Studio scans all 12 servos. Servos that do not answer are marked.

> [!WARNING]
> Keep the robot on a stand while you work on it. A servo that is set or calibrated wrong can move suddenly.

**Scan** looks for the servos again; **Refresh** reads all values again.

## Choose a Servo

The servos are listed by leg (Front right, Front left, Rear right, Rear left) with their connector, three joints per leg. Pick one, or choose **All servos** to change every servo at once. The arrows step to the previous or next servo.

For one servo you see a gauge with its position, its current against the current limit, its PWM duty and its temperature.

## Settings

The **Settings** tab lists every item of the EEPROM area of the [Control Table](../../actuator/MD-300-150/README.md#control-table) (Addr = Control Table No.) with the value you set and the value the servo reports. Below it, **Live telemetry** shows position, goal, position error, current, current limit and temperature (read-only).

Tap an item to edit it: the editor shows the value on the servo, the range and the step. Enter a value, or use **−** / **+** or the slider, and press **Set**.

A value you set goes to RAM: it works at once but is lost at power-off until you save it.

### Gain Presets

Presets set the gains of the chosen servo, or of all servos, in one step. You confirm before anything is written.

| Preset | Use |
| ------ | --- |
| Default | Shipped gains. Quiet, for walking. |
| Strong | Stiff and snappy, for flips. Holds harder and draws more current, so the motor runs warmer: switch back to Default for long walks. |

Both presets also set [Control Mode(11)](../../actuator/MD-300-150/README.md#control-mode11) to 0 (CURRENT).

### Save to Flash

Press **Save to flash** to keep your settings after power-off. The number on the button is how many changes are not saved yet.

## Torque and Live Poll

| Switch | On | Off |
| ------ | -- | --- |
| **Torque** | The servo holds its present position (current limit 200 [mA]) | Motor off, the joint is limp |
| **Live poll** | The readings refresh (2.5 times per second, faster while graphing or calibrating) | Readings paused |

## Pot Calibration

Calibration finds [Min/Max Position ADC(1, 2)](../../actuator/MD-300-150/README.md#minmax-position-adc1-2) and [Range Position(3)](../../actuator/MD-300-150/README.md#range-position3) of the position sensor. Press **Calibrate** on a servo.

> [!WARNING]
> Remove the leg or horn first. Auto sweep spins the shaft round and round in torque mode.

1. **Scale · counts per degree.** Park the horn at a measured angle (**Reference A angle**) and press **Capture**, turn it a known amount (**Reference B angle**) and press **Capture** again, then **Compute from A & B**. Measure once per potentiometer model; it holds for every servo of that model.
2. **Travel · auto or manual.**
   - **Auto sweep:** the servo turns by itself at **Spin current (mA)** until **Timeout (s)** and logs the sensor. **Close** always stops it.
   - **Manual:** press **Motor off**, then **Record sweep**, turn the horn by hand through its whole range and press again to stop. The lowest and highest counts are captured for you.
3. **Result.** Check Stop A (low), Stop B (high), Margin, Min ADC, Max ADC and Range, then press **Write + save to flash** (or **Write to servo only** to try it first).

If the sensor direction changes, Servo Studio flips [Reverse Motor(4)](../../actuator/MD-300-150/README.md#reverse-motor4) with it, so the servo keeps holding instead of running into its end stop.

## Graph

Open **Graph** and press **Start** to record live data of the chosen servo. The last 15 seconds are shown. Turn each trace on or off:

| Trace | Control Table |
| ----- | ------------- |
| Position | [Present Position(22)](../../actuator/MD-300-150/README.md#present-position16-22) |
| Goal | [Setpoint Position(21)](../../actuator/MD-300-150/README.md#setpoint-position21-position-error23) |
| Error | [Position Error(23)](../../actuator/MD-300-150/README.md#setpoint-position21-position-error23) |
| Current | [Present Current(26)](../../actuator/MD-300-150/README.md#present-current17-26) |
| Current cap | Current Limit in use(24) |
| PWM duty ×10 | [Present PWM(28)](../../actuator/MD-300-150/README.md#present-pwm28) |

**Reset** clears the graph, **Capture** saves it as a picture, **Save CSV** stores the recording and **Load CSV** opens one again for comparison.

## Disconnect

Turn off **Connect** when you are done. The robot resumes walking.

## HTTP API

Servo Studio uses these routes; your own tools can use them too. All are `GET` and answer JSON with `"ok": true` or `"ok": false` and an `"err"` text. `id` = servo ID 1 ~ 12. Routes marked **studio** need studio mode on first (`/v1/studio/mode?on=1`).

| Route | Studio | Description |
| ----- | ------ | ----------- |
| `/v1/studio/mode[?on=0\|1]` | | Read, or turn on / off, studio mode (the gait pauses while it is on) |
| `/v1/studio/status` | | Position, current and temperature of all 12 servos; works while walking |
| `/v1/studio/temps` | | Temperature of all 12 servos; works while walking |
| `/v1/studio/scan` | studio | Which servos answer |
| `/v1/studio/dump?id=N` | studio | Every EEPROM item of one servo |
| `/v1/studio/set?id=N&p=P&v=V` | studio | Write item `P` (Control Table No. 0 ~ 12) to RAM and read it back |
| `/v1/studio/setall?p=P&v=V[&save=1]` | studio | Write item `P` to all 12 servos, and save to flash with `save=1` |
| `/v1/studio/live?id=N` | studio | The live control-loop values (No. 19 ~ 31) of one servo |
| `/v1/studio/save?id=N` | studio | Store the configuration of the servo's module in flash (`id=0`: all modules) |
| `/v1/studio/restore?id=N` | studio | Load the factory configuration into RAM (`id=0`: all modules); follow with `save` to keep it |
| `/v1/studio/direct?id=N&m=M&deg=D&cur=C` | studio | Command one servo: `m` = 0 Idle, 1 Position, 2 Torque; `deg` = angle in the module's own degrees; `cur` = current limit or goal current [mA] |
