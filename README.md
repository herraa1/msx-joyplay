# Sony Playstation controller adapter for MSX (msx-joyplay) v2

[<img src="images/msx-joyplay-logo-same-color.png" width="256"/>](images/msx-joyplay-logo-same-color.png)

Connect [Sony PlayStation controllers](https://en.wikipedia.org/wiki/PlayStation_(console)#Controllers) to [MSX computers](https://www.msx.org/wiki/).

## Introduction

The msx-joyplay v2 is an adapter that allows connecting Sony PlayStation controllers to [MSX general purpose I/O ports](https://www.msx.org/wiki/General_Purpose_port).

The main features of the msx-joyplay v2 adapter are:
* small footprint
* made of widely available electronic components
* behaves as a cord extension between the MSX computer and the Sony PlayStation controller
* uses a female standard DE9 connector on the adapter's MSX joystick side
* uses a PlayStation Controller socket on the adapter's Sony PlayStation controller side
* no need for external power supply, the adapter draws current from the MSX port
* low power consumption (<20mA)
* serial debug provides information about the operation of the adapter

## [Hardware](hardware/kicad/)

The msx-joyplay v2 adapter uses an [Atmega328p](https://en.wikipedia.org/wiki/ATmega328) to convert the [Sony PlayStation controller signalling](https://store.curiousinventor.com/guides/PS2) to the [MSX joystick standard signalling](https://www.msx.org/wiki/Joystick_control).

A two-sided printed circuit board (PCB) is used to put together all components:
* An Atmega328p as the main MCU
* An AMS1117-3.3 regulator to convert the 5V from the joystick port to the 3.3V used for the Atmega328p chip and the PlayStation controller
* Two 74LS03 quad 2-input NAND gates with open collectors to completely mimic the standard MSX joystick behavior
* A PTC fuse to minimize damage to the MSX computer in case something goes wrong with the board
* Several additional required components (crystal, diode, leds, resistors, ceramic capacitors and a tantalum capacitor)
* An angled PSX socket is used to directly connect the Sony PlayStation controller
* A PH2.0 connector is used to connect the MSX cable extension
* A 2.54 pitch debug header is added for serial debug and programming
* An ICSP header is provided for burning a bootloader or to flash the chip using a hardware programmer

[<img src="images/msx-joyplay-v2-build2-front-render.png" width="512"/>](images/msx-joyplay-v2-build2-front-render.png)
[<img src="images/msx-joyplay-v2-build2-back-render.png" width="512"/>](images/msx-joyplay-v2-build2-back-render.png)

Connection to the MSX general purpose I/O port is implemented using a DE9 joystick extension cable with a female DE9 connector on one side and a loose end on the other side.
The MSX joystick extension cable loose end is wired according to the following pinout mapping.

| [<img src="images/msx_joystick-white-bg.png" height="100"/>](images/msx_joystick-white-bg.png) |
|:--|
| MSX joystick connector pinout, from controller plug side |

| MSX side pin | Cable color (may vary) | Signal |
| ------------ | ---------------------- | ------ |
| 5            | Brown                  | +5v    |
| 4            | Orange                 | RIGHT  |
| 3            | Grey                   | LEFT   |
| 2            | Black                  | DOWN   |
| 1            | Red                    | UP     |
| 6            | Green                  | TRIGA  |
| 7            | White                  | TRIGB  |
| 8            | Blue                   | OUT    |
| 9            | Yellow                 | GND    |

The msx-joyplay-v2 is fully compatible with MSX joysticks and honors the pin8 (OUT) signal.

The adapter uses open collector outputs (using 74LS03 Quad 2-Input NAND gates with open collector outputs) which makes the adapter safer [^2] than the
standard MSX joystick schematic depicted in the MSX Technical Data Book, as it avoids a series of undesired conditions that can lead to bus contention/short circuits.

Power is drawn from the +5V signal of the MSX general purpose I/O port, which is capable of delivering up to 50mA [^1].
The msx-joyplay v2 draws below 20mA from the port when a controller is connected, so it is on the safe side. Anyway, the msx-joyplay adapter uses a Positive Temperature Coeficient (PTC) resettable fuse of 50mA (F1) to limit current in case something goes wrong.

Also on the power side, a [1N5819 Schottky diode](https://www.diodes.com/assets/Datasheets/1N5819HW.pdf) (D1) is used to avoid leaking current from the msx-joyplay adapter to the MSX in case power is applied to one of the VCC pins available on the ICSP or Debug headers while the adapter is plugged into an MSX.

Connection to the Sony PlayStation controller is done via a Sony PlayStation controller socket.

| [<img src="images/psx-wiring.jpg" height="512"/>](images/psx-wiring.jpg) |
|:--|
| Sony PlayStation connector pinout, from Sony PlayStation controller side |

| PSX socket pin | Cable color official | Signal       | Comment                                                                   |
| -------------- | ---------------------| ------------ | ------------------------------------------------------------------------- |
| 1              | Brown                | DAT  / MISO  | data line, controller -> msx, open-collector signal                       |
| 2              | Orange               | CMD  / MOSI  | cmd line, msx -> controller, uses 3v3 logic                               |
| 3              | Grey                 | RUMBLE POWER | 7.6V power, not used (rumble not supported)                               |
| 4              | Black                | GND          | ground                                                                    |
| 5              | Red                  | +3V3         | +3V3 power                                                                |
| 6              | Yellow               | ATTN / SS    | select line, msx -> controller, uses 3v3 logic                            |
| 7              | Blue                 | CLK  / SCK   | clock line, msx -> controller, uses 3v3 logic                             |
| 8              | n/c                  |              |                                                                           |
| 9              | Green                | ACK          | ack line, controller -> msx, open-collector signal                        |

The Sony PlayStation controller uses 3.3V for power and logic, except for the rumble motor which uses 7.6V.

Power for the Sony PlayStation controller is provided by the 3.3V regulator on the msx-joyplay board.

### Recommended Build

Please, use [msx-joyplay-v2 Build2](#build2) for making new boards if you want something field-tested.

### [Build2](hardware/kicad/msx-joyplay-v2-build2)

[Bill Of Materials (BoM)](https://html-preview.github.io/?url=https://raw.githubusercontent.com/herraa1/msx-joyplay-v2/main/hardware/kicad/msx-joyplay-v2-build2/bom/ibom.html)

The Build2 adapter mimics completely the standard MSX joystick behaviour:

* When pin8 is HIGH, the adapter puts all stick and triggers signals in high impedance mode irrespective of their status (as if stick and triggers were not hold in the standard MSX joystick schematic), which become HIGH on the MSX side via the MSX PSG related circuitry pull-ups (matching the expected behavior)
* When pin8 is LOW
  * if a stick direction or trigger is hold, the corresponding signal is pulled down to GND causing it to be LOW (matching the expected behavior)
  * if a stick direction or trigger is not hold, the corresponding signal is put in high impedance mode, which becomes HIGH on the MSX side via the MSX PSG related circuitry pull-ups (matching the expected behaviour)

This build uses discrete logic components to honor the pin8 signaling (two [74LS03DR quad 2-input positive-nand gates with open collector outputs](https://www.ti.com/lit/ds/symlink/sn74ls03.pdf)) and uses open collector outputs which makes the adapter safer [^2] than the standard MSX joystick schematic depicted in the MSX Technical Data Book, as it avoids a series of undesired conditions that can lead to bus contention/short circuits.

## [Firmware](firmware/msx-joyplay-v2/)

The msx-joyplay v2 adapter firmware uses [SukkoPera's PsxNewLib library](https://github.com/SukkoPera/PsxNewLib) to read the Sony PlayStation controller status.

The following elements are used as inputs:
* digital pad (D-Pad), as direction arrows
* left analog pad, as direction arrows
* square button, as Trigger 1
* triangle button, as Trigger 2

Those elements' status are processed by the msx-joyplay firmware and transformed into MSX general purpose I/O port's signals on the fly.

## [Enclosure](enclosure/)

### Acrylic

A simple acrylic enclosure design for the project is provided to protect the electronic components and provide strain relief for the extension cords.

The enclosure uses a 3mm acrylic sheet.

[<img src="images/msx-joyplay-v2-with-acrylic-enclosure.png" width="400"/>](images/msx-joyplay-v2-with-acrylic-enclosure.png)

## References

MSX general purpose I/O port
* https://www.msx.org/wiki/General_Purpose_port

PlayStation controller pinout and protocol
* https://store.curiousinventor.com/guides/PS2

Atmega328p pinout
- https://camo.githubusercontent.com/18796d8b7673e3b6c14b6bdfb61d86b58c6f0d670398973abd4207afce960277/68747470733a2f2f692e696d6775722e636f6d2f6e6177657145362e6a7067

SukkoPera PsxNewLib project
* https://github.com/SukkoPera/PsxNewLib

[^1]: https://www.msx.org/wiki/General_Purpose_port
[^2]: https://www.msx.org/wiki/Joystick/joypad_controller (see "Undesired Conditions")

## Image Sources

* https://www.oshwa.org/open-source-hardware-logo/
* https://en.wikipedia.org/wiki/File:Numbered_DE9_Diagram.svg
