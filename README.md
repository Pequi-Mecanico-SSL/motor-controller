# motor-controller

Firmware for the wheel motor ESCs of our RoboCup SSL robot: one
[B-G431B-ESC1](https://www.st.com/en/evaluation-tools/b-g431b-esc1.html) per
wheel, running SimpleFOC (velocity loop over FOC current control, hall sensors)
and commanded over classic CAN by the Raspberry Pi. The Pi side lives in
`pequi_ssl_el` (`robot_control/can_bridge.py`).

- `firmware/` — Arduino sketch (STM32duino core)
- `PROTOCOL.md` — CAN IDs, opcodes, telemetry layout

## Hardware

- B-G431B-ESC1 (STM32G431CB), 4 boards on one 500 kbit/s bus
- Nanotec DF45L024048-A BLDC, 8 pole pairs, hall sensors on `A_HALL1/2/3`
- Bus wiring on J1 (GND, CANL, CANH, 5V). One ESC enables its onboard 120 Ω
  termination (`CAN_TERM_DEFAULT`), the Pi HAT is the other end.

## Build and flash

Dependencies: `arduino-cli`, STM32duino core 2.12, Simple FOC 2.4.0,
SimpleFOCDrivers 1.0.9, `st-flash` (stlink).

```sh
arduino-cli core install STMicroelectronics:stm32 --additional-urls \
    https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
arduino-cli lib install "Simple FOC@2.4.0" "SimpleFOCDrivers@1.0.9"

arduino-cli compile -b "STMicroelectronics:stm32:Disco:pnum=B_G431B_ESC1" \
    --output-dir firmware/build firmware

st-flash --reset write firmware/build/firmware.ino.bin 0x8000000
# or copy the .bin onto the DIS_G431CB USB drive
```

`firmware/build_opt.h` adds `-DHAL_OPAMP_MODULE_ENABLED`; without it current
sensing silently returns garbage.

## Per-board config

Each board picks its motor id from `uid_table` in `firmware.ino` (MCU UID,
printed on the serial console at boot). Unknown boards fall back to `MOTOR_ID`.
Set `CAN_TERM_DEFAULT 1` on exactly one board of the bus.

## Serial console

115200 baud on the ST-LINK virtual COM port, SimpleFOC Commander syntax:
`T<rad/s>` target, `C<A>` Iq cap, `E`/`D` enable/disable, `M...` motor
commands, `S1`/`S0` toggle a 100 Hz debug stream of velocity, current and hall
edges.

Gains changed over CAN or serial are volatile; hardcode tuned values in
`setup()`.
