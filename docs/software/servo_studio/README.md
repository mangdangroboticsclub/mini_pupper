# Servo Studio

Servo Studio is a web page served by the Mini Pupper itself. With it you can read and change the settings of each [MD-300-150](../../actuator/MD-300-150/README.md) servo, calibrate its position sensor and watch it live. Nothing to install: any phone or computer with a browser works.

> [!NOTE]
> This page describes Servo Studio for actuator **firmware 0.0.9**.

## Contents

- [Open Servo Studio](#open-servo-studio)
- [Connect](#connect)
- [Choose a Servo](#choose-a-servo)
- [Settings](#settings)
  - [Gain Presets](#gain-presets)
  - [Control Mode](#control-mode)
  - [Save to Flash](#save-to-flash)
- [Pot Calibration](#pot-calibration)
- [Live Graph](#live-graph)
- [Disconnect](#disconnect)

## Open Servo Studio

1. Connect your phone or computer to the same Wi-Fi as the robot (or to the robot's own hotspot).
2. Open the robot's web page in a browser: `http://<robot IP address>`.
3. Click **MangDang Studio**, or open `http://<robot IP address>/wizard` directly.

## Connect

Turn on **Connect**. The robot pauses walking so the servos can be read and changed safely, and Servo Studio reads all 12 servos. Servos that are not plugged in are greyed out.

> [!WARNING]
> Keep the robot on a stand while you work on it. A servo that is set or calibrated wrong can move suddenly.

## Choose a Servo

Pick a servo by leg (Front Right, Front Left, Rear Right, Rear Left) and joint. Choose **All servos** to change every servo at once. Use **Previous servo** / **Next servo** to step through them.

## Settings

Every item of the EEPROM area of the [Control Table](../../actuator/MD-300-150/README.md#control-table) is listed with its present value, range and description. Enter a **New value** and press **Set**.

A value you set goes to RAM: it works at once but is lost at power-off until you save it.

### Gain Presets

Presets set the position and current gains of all 12 servos in one step. You confirm before anything is written.

| Preset | Use |
| ------ | --- |
| Default | Shipped gains. Quiet, for walking. |
| Normal | Softer and quieter, for walking and demos. |
| Strong | Stiff and snappy, for flips and jumps. Draws more current and runs warmer: switch back to Default for long walks. |

### Control Mode

Switch between **PWM mode** (default) and **CURRENT mode** at run time, no reflash. See [Control Mode(11)](../../actuator/MD-300-150/README.md#control-mode11). Items that only work in one mode are marked.

### Save to Flash

Press **Save to flash** to keep your settings after power-off. It saves all four modules.

## Pot Calibration

Calibration finds [Min/Max Position ADC(1, 2)](../../actuator/MD-300-150/README.md#minmax-position-adc1-2) and [Range Position(3)](../../actuator/MD-300-150/README.md#range-position3) of the position sensor.

1. Remove the leg or horn from the servo first.
2. Choose the travel method:
   - **Auto sweep**: the servo turns itself through its whole range at a low current (**Spin current**) and logs the sensor.
   - **Manual**: turn the horn by hand through its whole range while recording.
3. Give two reference angles (**Reference A**, **Reference B**) with their ADC readings. One wrap-to-wrap turn is exactly 360°, so no protractor is needed.
4. Check the result (counts per degree, Stop A, Stop B) and write it. Servo Studio also checks that [Reverse Motor(4)](../../actuator/MD-300-150/README.md#reverse-motor4) matches the sensor direction, and flips it if needed.
5. Press **Save to flash**.

## Live Graph

Press **Start** to record live data of the chosen servo (the last 15 seconds are shown):

- Goal and present position, position error
- Current limit, current setpoint, present current
- PWM duty, temperature, raw ADC values

**Save CSV** stores the recording; **Load CSV** opens one again for comparison. You can also save the graph as a picture.

**Motor off** sets the chosen servo to Idle at once.

## Disconnect

Turn off **Connect** when you are done. The robot resumes walking.
