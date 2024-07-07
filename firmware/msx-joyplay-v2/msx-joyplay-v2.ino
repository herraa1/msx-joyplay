/*
 * msx-joyplay v2, a PlayStation controller to MSX joystick adapter
 * Copyright (C) 2024 Albert Herranz
 *
 * This code is released under GPL V2.0
 *
 * Valid-License-Identifier: GPL-2.0-only
 * SPDX-URL: https://spdx.org/licenses/GPL-2.0-only.html
 *
 * Based on previous work by:
 * - SukkoPera PsxNewLib project, https://github.com/SukkoPera/PsxNewLib
 *
 * Other references:
 * - https://store.curiousinventor.com/guides/PS2
 * - https://www.msx.org/wiki/General_Purpose_port
 * - https://github.com/MCUdude/MiniCore
 * - https://www.arduino.cc/en/software
 */

/*******************************************************************************
 * Copyright (C) 2019-2020 by SukkoPera <software@sukkology.net>               *
 *                                                                             *
 * PsxNewLib is free software: you can redistribute it and/or                  *
 * modify it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or           *
 * (at your option) any later version.                                         *
 *                                                                             *
 * PsxNewLib is distributed in the hope that it will be useful,                *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * You should have received a copy of the GNU General Public License           *
 * along with PsxNewLib. If not, see http://www.gnu.org/licenses.              *
 *******************************************************************************
 */

/*

Built using Arduino 2.2.1
Libraries:
- DigitalIO 1.0.1
- PSXNewLib 0.4.0
Boards:
- Minicore 2.2.2
  - Board: Atmega328
  - BOD 1.8V
  - Bootloader: Yes (UART0)
  - Clock: External 8MHz
  - EEPROM: Retained
  - Compiler LTO: Disabled
  - Variant: 328P/328PA

PSX Pin         Wire   Direction            Type           V     Function           atmega238p/Arduino pin 
--------------- ------ -------------------- -------------- ----- ------------------ ------------
 1 DATA         BROWN  controller -> msx    open-collector 3.3V  miso               16/12
 2 COMMAND      ORANGE msx -> controller                   3.3V  mosi               15/11
 3 RUMBLE POWER GREY                                       7.6V                     unused
 4 GND          BLACK                                                               GND
 5 +3V3         RED                                                                 +3V3
 6 ATTN         YELLOW msx -> controller                   3.3V  ss (slave select)  14/10
 7 CLK          BLUE   msx -> controller                   3.3V  sck                17/13
 8 UNK          n/c                                                                 unused
 9 ACK          GREEN  controller -> msx   open-collector  3.3V                     unused

*/

#include <DigitalIO.h>
#include <PsxControllerHwSpi.h>

#include <avr/pgmspace.h>
typedef const __FlashStringHelper* FlashStr;
typedef const byte* PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper*>(s)

/* SS, Slave Select */
const byte PIN_PS2_ATT = 10;

/* LEDs */
const byte PIN_BUTTONPRESS = A1;    /* not used on current board */
const byte PIN_HAVECONTROLLER = A0; /* green led */

/*
 * Arduino Nano pins for MSX signals.
 * We use PORTD for all MSX joystick arrow and buttons signals.
 * This allows to set them all at the same time.
 */
#define PORT_MSX_JOYSTICK PORTD
#define DDR_MSX_JOYSTICK DDRD

/* Sony Playstation controller pad axis valid range */
#define PSX_AXIS_MIN 0
#define PSX_AXIS_MAX 255

/* MSX general purpose signals as mapped to Arduino Nano PORTD bits */
const int PORT_MSX_JOYSTICK_UP = 7;       /* PD7, MSX joystick pin1 */
const int PORT_MSX_JOYSTICK_DOWN = 6;     /* PD6, MSX joystick pin2 */
const int PORT_MSX_JOYSTICK_LEFT = 5;     /* PD5, MSX joystick pin3 */
const int PORT_MSX_JOYSTICK_RIGHT = 4;    /* PD4, MSX joystick pin4 */
const int PORT_MSX_JOYSTICK_TRIGGER1 = 3; /* PD3, MSX joystick pin6 */
const int PORT_MSX_JOYSTICK_TRIGGER2 = 2; /* PD2, MSX joystick pin7 */

volatile uint8_t curr_msx_joystick_signals = 0x00;

/*
 * Discrete thresholds for Sony Playstation joystick analog pad axis:
 * - Values below the min threshold on the horizontal/vertical axis indicate
 *   left/down active, respectively.
 * - Values above the max threshold on the horizontal/vertical axis indicate
 *   right/up active, respectively.
 */
int axis_threshold_min, axis_threshold_max;

PsxControllerHwSpi<PIN_PS2_ATT> psx;
boolean haveController = false;

/* serial output width control */
uint8_t output_width_count = 0;

#ifdef DEBUG
const char buttonSelectName[] PROGMEM = "Select";
const char buttonL3Name[] PROGMEM = "L3";
const char buttonR3Name[] PROGMEM = "R3";
const char buttonStartName[] PROGMEM = "Start";
const char buttonUpName[] PROGMEM = "Up";
const char buttonRightName[] PROGMEM = "Right";
const char buttonDownName[] PROGMEM = "Down";
const char buttonLeftName[] PROGMEM = "Left";
const char buttonL2Name[] PROGMEM = "L2";
const char buttonR2Name[] PROGMEM = "R2";
const char buttonL1Name[] PROGMEM = "L1";
const char buttonR1Name[] PROGMEM = "R1";
const char buttonTriangleName[] PROGMEM = "Triangle";
const char buttonCircleName[] PROGMEM = "Circle";
const char buttonCrossName[] PROGMEM = "Cross";
const char buttonSquareName[] PROGMEM = "Square";

const char* const psxButtonNames[PSX_BUTTONS_NO] PROGMEM = {
  buttonSelectName,
  buttonL3Name,
  buttonR3Name,
  buttonStartName,
  buttonUpName,
  buttonRightName,
  buttonDownName,
  buttonLeftName,
  buttonL2Name,
  buttonR2Name,
  buttonL1Name,
  buttonR1Name,
  buttonTriangleName,
  buttonCircleName,
  buttonCrossName,
  buttonSquareName
};

void print_hex8(uint8_t data)
{
  char tmp[2 + 1];
  byte nibble;

  nibble = (data >> 4) | 48;
  tmp[0] = (nibble > 57) ? nibble + (byte)39 : nibble;
  nibble = (data & 0x0f) | 48;
  tmp[1] = (nibble > 57) ? nibble + (byte)39 : nibble;
  tmp[2] = 0;

  Serial.print(tmp);
}

void print_hex16(uint16_t data)
{
  print_hex8((uint8_t)(data >> 8));
  print_hex8((uint8_t)(data & 0xff));
}

byte psxButtonToIndex(PsxButtons psxButtons)
{
  byte i;

  for (i = 0; i < PSX_BUTTONS_NO; ++i) {
    if (psxButtons & 0x01) {
      break;
    }

    psxButtons >>= 1U;
  }

  return i;
}

FlashStr getButtonName(PsxButtons psxButton)
{
  FlashStr ret = F("");

  byte b = psxButtonToIndex(psxButton);
  if (b < PSX_BUTTONS_NO) {
    PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxButtonNames[b])));
    ret = PSTR_TO_F(bName);
  }

  return ret;
}

void dumpButtons(PsxButtons psxButtons)
{
  static PsxButtons lastB = 0;

  if (psxButtons != lastB) {
    lastB = psxButtons;  // Save it before we alter it

    Serial.print(F("Pressed: "));

    for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
      byte b = psxButtonToIndex(psxButtons);
      if (b < PSX_BUTTONS_NO) {
        PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxButtonNames[b])));
        Serial.print(PSTR_TO_F(bName));
      }
      psxButtons &= ~(1 << b);

      if (psxButtons != 0) {
        Serial.print(F(", "));
      }
    }

    Serial.println();
  }
}

void dumpAnalog(const char* str, const byte x, const byte y)
{
  Serial.print(str);
  Serial.print(F(" analog: x = "));
  Serial.print(x);
  Serial.print(F(", y = "));
  Serial.println(y);
}
#endif

void print_rolling_sequence(void)
{
  static char rolling_chars[] = { '-', '\\', '|', '/' };
  static uint8_t rolling_index = 0;

  //Serial.write(8);
  Serial.print(rolling_chars[rolling_index++]);
  if (rolling_index >= sizeof(rolling_chars))
    rolling_index = 0;
}

const char ctrlTypeUnknown[] PROGMEM = "Unknown";
const char ctrlTypeDualShock[] PROGMEM = "Dual Shock";
const char ctrlTypeDsWireless[] PROGMEM = "Dual Shock Wireless";
const char ctrlTypeGuitHero[] PROGMEM = "Guitar Hero";
const char ctrlTypeOutOfBounds[] PROGMEM = "(Out of bounds)";

const char* const controllerTypeStrings[PSCTRL_MAX + 1] PROGMEM = {
  ctrlTypeUnknown,
  ctrlTypeDualShock,
  ctrlTypeDsWireless,
  ctrlTypeGuitHero,
  ctrlTypeOutOfBounds
};

inline void __update_msx_signals(uint8_t signals)
{
  /*
     * signals variable bit definitions
     * - bit set   : joystick arrow/button is pressed
     *               configure associated GPIO as HIGH/OUTPUT (logic "1")
     * - bit unset : joystick arrow/button is not pressed
     *               configure associated GPIO as LOW/OUTPUT (logic "0")
     */

  /* write all signal states at once to MSX side */
  PORT_MSX_JOYSTICK = (signals);
  DDR_MSX_JOYSTICK = (~0);
}

void setup()
{
  int axis_threshold;

  __update_msx_signals(curr_msx_joystick_signals);

  /*
     * Calculate PSX controller analog pad thresholds.
     * Analog pad needs to be pushed from its origin in one direction at least
     * 1/6 of the complete run to get active
     */
  axis_threshold = (PSX_AXIS_MAX - PSX_AXIS_MIN) / 3;
  axis_threshold_min = PSX_AXIS_MIN + axis_threshold;
  axis_threshold_max = PSX_AXIS_MAX - axis_threshold;

  fastPinMode(PIN_BUTTONPRESS, OUTPUT);
  fastPinMode(PIN_HAVECONTROLLER, OUTPUT);

  Serial.begin(115200);
  Serial.println(F("msx-joyplay-v2 20240629_1"));

  delay(300);
}

void loop() {
  static byte slx, sly, srx, sry;

  fastDigitalWrite(PIN_HAVECONTROLLER, haveController);

  if (!haveController) {
    curr_msx_joystick_signals = 0x00;
    __update_msx_signals(curr_msx_joystick_signals);

    /* write rolling sequence while not connected */
    print_rolling_sequence();
    /* wrap output at 80 columns ... */
    if (output_width_count++ > 79) {
      output_width_count = 0;
      Serial.println("");
    }

    if (psx.begin()) {
      Serial.println(F("Controller found!"));
      output_width_count = 0;
      delay(300);
      if (!psx.enterConfigMode()) {
        Serial.println(F("Cannot enter config mode"));
      } else {
        PsxControllerType ctype = psx.getControllerType();
        PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte>(ctype) : PSCTRL_MAX])));
        Serial.print(F("Controller Type is: "));
        Serial.println(PSTR_TO_F(cname));

        if (!psx.enableAnalogSticks()) {
          Serial.println(F("Cannot enable analog sticks"));
        }

        if (!psx.enableAnalogButtons()) {
          Serial.println(F("Cannot enable analog buttons"));
        }

        if (!psx.exitConfigMode()) {
          Serial.println(F("Cannot exit config mode"));
        }
      }

      haveController = true;
    }
  } else {
    if (!psx.read()) {
      Serial.println(F("Controller lost :("));
      haveController = false;
    } else {
      PsxButtons psxButtons = psx.getButtonWord();
      fastDigitalWrite(PIN_BUTTONPRESS, !!psxButtons);
#ifdef DEBUG
      dumpButtons(psxButtons);
#endif

      byte lx, ly;
      psx.getLeftAnalog(lx, ly);
      if (lx != slx || ly != sly) {
#ifdef DEBUG
        dumpAnalog("Left", lx, ly);
#endif
        slx = lx;
        sly = ly;
      }

#ifdef DEBUG
      byte rx, ry;
      psx.getRightAnalog(rx, ry);
      if (rx != srx || ry != sry) {
        dumpAnalog("Right", rx, ry);
        srx = rx;
        sry = ry;
      }
#endif

      uint8_t msx_joystick_signals = 0;

      if (!!(psxButtons & PSB_PAD_UP))
        msx_joystick_signals |= (1 << PORT_MSX_JOYSTICK_UP);
      if (!!(psxButtons & PSB_PAD_DOWN))
        msx_joystick_signals |= (1 << PORT_MSX_JOYSTICK_DOWN);
      if (!!(psxButtons & PSB_PAD_LEFT))
        msx_joystick_signals |= (1 << PORT_MSX_JOYSTICK_LEFT);
      if (!!(psxButtons & PSB_PAD_RIGHT))
        msx_joystick_signals |= (1 << PORT_MSX_JOYSTICK_RIGHT);
      if (!!(psxButtons & PSB_SQUARE))
        msx_joystick_signals |= (1 << PORT_MSX_JOYSTICK_TRIGGER1);
      if (!!(psxButtons & PSB_TRIANGLE))
        msx_joystick_signals |= (1 << PORT_MSX_JOYSTICK_TRIGGER2);

      /* before activating UP check that DOWN is not activated */
      if ((ly < axis_threshold_min) && !(msx_joystick_signals & (1 << PORT_MSX_JOYSTICK_DOWN)))
        msx_joystick_signals |= (1 << PORT_MSX_JOYSTICK_UP);

      /* before activating DOWN check that UP is not activated */
      if ((ly > axis_threshold_max) && !(msx_joystick_signals & (1 << PORT_MSX_JOYSTICK_UP)))
        msx_joystick_signals |= (1 << PORT_MSX_JOYSTICK_DOWN);

      /* before activating LEFT check that RIGHT is not activated */
      if ((lx < axis_threshold_min) && !(msx_joystick_signals & (1 << PORT_MSX_JOYSTICK_RIGHT)))
        msx_joystick_signals |= (1 << PORT_MSX_JOYSTICK_LEFT);

      /* before activating RIGHT check that LEFT is not activated */
      if ((lx > axis_threshold_max) && !(msx_joystick_signals & (1 << PORT_MSX_JOYSTICK_LEFT)))
        msx_joystick_signals |= (1 << PORT_MSX_JOYSTICK_RIGHT);

      curr_msx_joystick_signals = msx_joystick_signals;

      __update_msx_signals(curr_msx_joystick_signals);
    }
  }

  delay(1000 / 60);
}
