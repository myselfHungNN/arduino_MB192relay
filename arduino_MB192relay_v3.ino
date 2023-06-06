#include <SimpleModbusSlave.h>
#include <stdint.h>

// UART - RS485 pin
#define RX 0
#define TX 1
#define RS485 2

#define SER1 3
#define SRCLK1 4
#define RCLK1 5

#define SER2 6
#define SRCLK2 7
#define RCLK2 8

#define SC_enable 9

#define ledpin LED_BUILTIN

#define input0 10
#define input1 11
#define input2 12
// #define input3 12

#define eMIC1 A3
#define eMIC2 A5
#define eMIC3 A4
#define eMIC4 A2
#define eMIC5 A1
#define eMIC6 A0


#define INPUTNo 3
#define OUTPUTNo 15

const uint8_t INPUT_pins[INPUTNo] = { input0, input1, input2 };
const uint8_t OUTPUT_pins[OUTPUTNo] = { RS485, SER1, SRCLK1, RCLK1, SER2, SRCLK2, RCLK2,
                                        ledpin, eMIC1, eMIC2, eMIC3, eMIC4, eMIC5, eMIC6, SC_enable };
const uint8_t eMICpins[7] = { eMIC1, eMIC2, eMIC3, eMIC4, eMIC5, eMIC6, SC_enable };

enum {                    // Bo sung them cac Holding Regiters
  Relay_HoldingRegs_01,   // 0 DPDT relays
  Relay_HoldingRegs_11,   // 1 DPDT relays
  Relay_HoldingRegs_12,   // 2 DPDT relays
  Relay_HoldingRegs_13,   // 3 DPDT relays
  Relay_HoldingRegs_14,   // 4 DPDT relays
  Relay_HoldingRegs_21,   // 5 DPDT relays
  Relay_HoldingRegs_22,   // 6 DPDT relays
  Relay_HoldingRegs_23,   // 7 DPDT relays
  Relay_HoldingRegs_24,   // 8 DPDT relays
  Relay_HoldingRegs_30,   // 9 DPDT relays
  Relay_HoldingRegs_31,   // 10 DPDT relays
  Relay_HoldingRegs_40,   // 11 SPDT relays
  Relay_HoldingRegs_41,   // 12	SPDT relays
  Relay_HoldingRegs_42,   // 13	 Reserved
  Relay_HoldingRegs_43,   // 14	 Reserved
  Relay_HoldingRegs_44,   // 15	 Reserved
  Relay_HoldingRegs_45,   // 16	 Reserved
  Opto_HoldingRegs_50_1,  // 17   Short Circuits
  Opto_HoldingRegs_50_2,  // 18  Short Circuits
  Opto_HoldingRegs_51_1,  // 19  Short Circuits
  Opto_HoldingRegs_51_2,  // 20  Short Circuits
  Opto_HoldingRegs_52_1,  // 21  input 0-7 (arduino- MB id = 1)
  Opto_HoldingRegs_52_2,  // 22  Reserved
  Opto_HoldingRegs_53_1,  // 23  Reserved
  Opto_HoldingRegs_53_2,  // 24  Reserved
  counterIdx,             // 25
  // add more Regs before this line=======
  HoldingRegsSize
};

#define _boardID 1  // Modbus Address

#define _board595 8
#define numberOf74HC595 (_board595 << 1)

uint16_t holdingRegs[HoldingRegsSize];  // function 3 and 16 register array

uint8_t Relay_value[numberOf74HC595];

/* Array index
0-3  Relay_HoldingRegs_11 (DPDT Relays)
4-7  Relay_HoldingRegs_12 (DPDT Relays)
8-11 Relay_HoldingRegs_13 (DPDT Relays)
12-15 Relay_HoldingRegs_14 (DPDT Relays)
16-19 Relay_HoldingRegs_40 (DPDT Relays)
20-21 Relay_HoldingRegs_41 (SPDT Relays)
*/
//=========================================================
void setup() {
  for (uint8_t i = 0; i < OUTPUTNo; i++) {
    pinMode(OUTPUT_pins[i], OUTPUT);
  }
  for (uint8_t i = 0; i < 6; i++) {
    digitalWrite(eMICpins[i], HIGH);
  }
  digitalWrite(SC_enable, HIGH);

  digitalWrite(ledpin, LOW);

  for (uint8_t i = 0; i < numberOf74HC595; i++) {
    Relay_value[i] = 0;
  }

  write_relay595_1(Relay_value, 8);
  write_relay595_2(Relay_value, 8);
  write_relayIO(0);

  /* parameters(long baudrate,
                  unsigned char ID,
                  unsigned char transmit enable pin,
                  unsigned int holding registers size,
                  unsigned char low latency)

       The transmit enable pin is used in half duplex communication to activate a MAX485 or similar
       to deactivate this mode use any value < 2 because 0 & 1 is reserved for Rx & Tx.
       Low latency delays makes the implementation non-standard
       but practically it works with all major modbus master implementations.
    */
  modbus_configure(9600, _boardID, RS485, HoldingRegsSize, 0);
  // Uart config: 9600, databit-8bit, Even, Stopbit-1bit
}
//=============================================================
void loop() {
  static uint32_t last_ms;
  static bool bitstatus;
  static uint8_t Relay_value_1[numberOf74HC595] = { 1 };
  uint16_t INPUTpins;

  modbus_update(holdingRegs);

  copyHoldingRegsToRelayVal(Relay_value, holdingRegs);

  if (checkNewRelayValue(Relay_value_1, Relay_value) == true) {
    write_relay595_1(Relay_value, 8);
    write_relay595_2(Relay_value, 8);
    // write_relay595(Relay_value, numberOf74HC595);
  }

  write_relayIO(holdingRegs[Relay_HoldingRegs_01]);

  delay(50);

  if (millis() - last_ms >= 1000) {
    digitalWrite(ledpin, bitstatus);
    bitstatus ^= true;
    last_ms = millis();
  }
}
//============================================================
bool checkNewRelayValue(uint8_t* Relay_value_1, const uint8_t* Relay_value) {
  uint8_t idx;
  bool check = false;

  for (idx = 0; idx < numberOf74HC595; idx++) {
    if (*(Relay_value_1 + idx) != *(Relay_value + idx)) {
      check = true;
      break;
    } else {
      check = false;
    }
  }

  if (check) {
    for (idx = 0; idx < numberOf74HC595; idx++) {
      *(Relay_value_1 + idx) = *(Relay_value + idx);
    }
  }
  return (check);
}
//============================================================
void copyHoldingRegsToRelayVal(uint8_t* RelayVal, const uint16_t* RegHolding) {
  uint8_t idx = 0;
  uint16_t u16temp;
  uint8_t u8_H, u8_L;
  for (idx = 0; idx < 5; idx++) {
    switch (idx) {
      case 0:
        u16temp = holdingRegs[Relay_HoldingRegs_11];
        break;
      case 1:
        u16temp = holdingRegs[Relay_HoldingRegs_12];
        break;
      case 2:
        u16temp = holdingRegs[Relay_HoldingRegs_13];
        break;
      case 3:
        u16temp = holdingRegs[Relay_HoldingRegs_14];
        break;
      case 4:
        u16temp = holdingRegs[Relay_HoldingRegs_40];
        break;
      default:
        break;
    }
    u8_H = u16temp >> 8;
    u8_L = u16temp;
    *(RelayVal + 4 * idx) = NibbleToByte(u8_L & 0x0F);
    *(RelayVal + 4 * idx + 1) = NibbleToByte((u8_L & 0xF0) >> 4);
    *(RelayVal + 4 * idx + 2) = NibbleToByte(u8_H & 0x0F);
    *(RelayVal + 4 * idx + 3) = NibbleToByte((u8_H & 0xF0) >> 4);

    if (idx == 4) {
      u16temp = holdingRegs[Relay_HoldingRegs_40];
      u8_H = u16temp >> 8;
      u8_L = u16temp;
      *(RelayVal + 4 * idx) = u8_L;
      *(RelayVal + 4 * idx + 1) = u8_H;

      u16temp = holdingRegs[Relay_HoldingRegs_41];
      u8_H = u16temp >> 8;
      u8_L = u16temp;
      *(RelayVal + 4 * idx + 2) = u8_L;
      *(RelayVal + 4 * idx + 3) = u8_H;
    }
  }
}
//============================================================
void write_relay595_1(const uint8_t* relay_values, const uint8_t len) {
  uint8_t idx;

  digitalWrite(RCLK1, LOW);
  for (idx = len; idx > 0; idx--) {
    shiftOut(SER1, SRCLK1, MSBFIRST, ~relay_values[idx - 1]);
    // delay(1);
  }
  digitalWrite(RCLK1, HIGH);
  digitalWrite(RCLK1, LOW);
}
//==============================================================
void write_relay595_2(const uint8_t* relay_values, const uint8_t len) {
  uint8_t idx;

  digitalWrite(RCLK2, LOW);
  for (idx = len; idx > 0; idx--) {
    shiftOut(SER2, SRCLK2, MSBFIRST, ~relay_values[idx + 7]);
    // delay(1);
  }
  digitalWrite(RCLK2, HIGH);
  digitalWrite(RCLK2, LOW);
}
//=============================================================
uint8_t NibbleToByte(const uint8_t u8nibble) {
  uint8_t bit3, bit2, bit1, bit0;
  uint8_t u8temp;
  bit0 = ((1 << 0) & u8nibble) >> 0;
  bit1 = ((1 << 1) & u8nibble) >> 1;
  bit2 = ((1 << 2) & u8nibble) >> 2;
  bit3 = ((1 << 3) & u8nibble) >> 3;
  u8temp = ((bit3 << 7) | (bit3 << 6) | (bit2 << 5) | (bit2 << 4) | (bit1 << 3) | (bit1 << 2) | (bit0 << 1) | (bit0 << 0));
  return (u8temp);
}
//=============================================================
void write_relayIO(const uint16_t val) {
  uint8_t bittemp = val & 0xff;
  static bool staticbit[7] = { 0, 0, 0, 0, 0, 0, 0 };

  for (uint8_t i = 0; i < 7; i++) {
    if ((bittemp & 1) != staticbit[i]) {
      staticbit[i] = bittemp & 1;
      digitalWrite(eMICpins[i], !staticbit[i]);
    }
    if (i < 6) bittemp >>= 1;
  }

}
