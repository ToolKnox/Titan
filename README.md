# Titan

A four-legged robot that folds its feet into tank tracks. Walks over
obstacles when the terrain is rough, drops onto tracks for speed across
flat ground.

This repo is the **firmware** — ESP32 code that drives the servos, the
track motors, the IMU, the OLED status display, the RGB lighting, and
the WiFi/TCP link to the Android control app.

> Build instructions, prints, BOM and the Android app live with the
> Printables project. This README only covers the firmware.

## Hardware target

- **MCU:** ESP32-WROVER-E (V2.0 boards use ESP32-PICO-D4)
- **Servo drive:** PCA9685 over I²C, plus Hiwonder bus servos on a
  half-duplex serial bus *(see `src/Drive/Hiwonder.cpp` — confirm whether
  your build uses bus servos, PWM servos, or both)*
- **Track motors:** 4× 12 V DC motors via an H-bridge driver
- **IMU:** MPU6050 (6-axis) over I²C
- **Display:** 0.96" OLED via U8g2
- **Lighting:** WS2812B RGB
- **Audio:** piezo buzzer

## Repo layout

```
platformio.ini                 PlatformIO board / framework / libs
src/
  main.cpp                     Entry point, setup() / loop()
  Drive/
    IOs.{h,cpp}                GPIO pin map + low-level I/O
    IMU.{h,cpp}                MPU6050 wrapper
    ServoDrive.{h,cpp}         PCA9685 PWM servo control
    DCMotorDrive.{h,cpp}       Track motor PWM + direction
    Hiwonder.{h,cpp}           Hiwonder bus-servo protocol
  Dynamics/
    FKIK.{h,cpp}               Per-leg forward / inverse kinematics
    MotionControl.{h,cpp}      Posture & gait controller (the brain)
lib/                           Vendored deps — see "Dependencies"
```

`lib/` holds pinned copies of every external library. PlatformIO picks
those up automatically; you don't need to install anything by hand.

## Build & flash

You need:

- [VSCode](https://code.visualstudio.com/) with the
  [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode), **or**
  [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) on the CLI
- A USB cable that actually carries data (not a charge-only cable)

Then:

```bash
git clone https://github.com/ToolKnox/Titan.git
cd Titan

# CLI:
pio run                  # compile
pio run -t upload        # flash over USB
pio device monitor       # serial console
```

Or open the folder in VSCode and use the PlatformIO toolbar
(✓ build, → upload, 🔌 monitor).

## Configure

Before flashing, set your WiFi credentials and any tunables. The exact
location depends on the current source — search for `WIFI_SSID` /
`ssid` in `src/main.cpp`.

> TODO: confirm where SSID / password / TCP port live and document the
> exact lines a new builder needs to change. If they're hard-coded in
> `main.cpp`, consider moving them to a gitignored `secrets.h`.

## Pin assignments

Pin numbers live in `src/Drive/IOs.h`. Treat that file as the source of
truth — the table below is for orientation only.

| Function          | GPIO        | Notes |
|-------------------|-------------|-------|
| PCA9685 SDA / SCL | TODO / TODO | I²C bus shared with MPU6050 |
| MPU6050 INT       | TODO        | optional, for DMP interrupt |
| OLED SDA / SCL    | TODO / TODO | typically same I²C bus |
| WS2812B data      | TODO        | one data line, 5 V level-shift recommended |
| Buzzer            | TODO        |  |
| DC motor 1 IN1/IN2/PWM | TODO   |  |
| DC motor 2 IN1/IN2/PWM | TODO   |  |
| DC motor 3 IN1/IN2/PWM | TODO   |  |
| DC motor 4 IN1/IN2/PWM | TODO   |  |
| Hiwonder bus TX/RX | TODO       | half-duplex, single wire bus |

> TODO: fill in this table from `src/Drive/IOs.h`. Keep it in sync — if
> you reshuffle pins, update this table in the same commit.

## Run

On boot the firmware:

1. Brings up I²C, the IMU, the OLED, the LEDs and the buzzer.
2. Centres all servos and arms the motor driver.
3. Joins the configured WiFi network and starts a TCP listener.
4. Waits for the Android app to connect, then streams gait / pose
   commands.

The OLED shows status; if WiFi join fails or the IMU doesn't respond,
that's where you'll see it first. Serial monitor (115200 baud — confirm
in `platformio.ini`) gives the verbose log.

## Dependencies

Vendored under `lib/`, no external install needed:

- [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) — WS2812B
- [Adafruit_PWM_Servo_Driver_Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) — PCA9685
- [MPU6050](https://github.com/jrowberg/i2cdevlib) (jrowberg's i2cdevlib copy) — IMU + DMP
- [U8g2](https://github.com/olikraus/u8g2) — OLED

If you upgrade any of these, do it via the PlatformIO registry rather
than dropping new sources into `lib/`, and update this list.

## Troubleshooting

- **`pio run` fails on a missing header from `lib/…`** — make sure you
  cloned with submodules / didn't shallow-clone. The libs are committed
  in-tree, so a normal `git clone` should be enough.
- **Board not detected on upload** — usually a charge-only USB cable, or
  the ESP32 needs `BOOT` held while you press `EN`. The WROVER-E dev
  boards generally auto-reset; bare modules don't.
- **MPU6050 reads zeros / 0xFF** — pull-ups missing on SDA/SCL, or the
  AD0 pin is floating and the address is wrong. Check the I²C scan
  output on serial.
- **Servos jitter / brown out** — track motors and servos sharing a
  rail. Heat sinks on the motor drivers are not optional under sustained
  tracked-mode current; same goes for keeping logic 5 V isolated from
  motor 12 V except at one star ground.
- **OLED blank** — wrong I²C address (0x3C vs 0x3D) or U8g2 constructor
  doesn't match your panel. The constructor lives near the top of
  `src/main.cpp`.

## Contributing

PRs welcome — especially gait tuning, MG996R validation (the alternate
servo from the BOM that hasn't been load-tested), and anything that
trims `MotionControl.cpp`.

## License

> TODO: pick a license and add a `LICENSE` file. Until then assume "all
> rights reserved" — don't redistribute without asking.

## See also

- [Project page on Printables](https://www.printables.com/) — prints, BOM, build photos *(TODO: link)*
- Titan Main Controller PCB — [PCBWay](https://www.pcbway.com/project/shareproject/Titan_Quaddruped_PCBs_bfb8a382.html)
- Titan Servo Controller PCB — [PCBWay](https://www.pcbway.com/project/shareproject/Titan_Servo_Controller_f9480679.html)