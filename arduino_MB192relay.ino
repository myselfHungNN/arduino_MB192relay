#include <SimpleModbusSlave.h>
#include <stdint.h>

#define SER1 3
#define RCLK1 4
#define SRCLK1 5
#define OE1 6

#define SER2 7
#define RCLK2 8
#define SRCLK2 9
#define OE2 10

#define RS485 2
#define TX 1
#define RX 0

#define input0 11
#define input1 12
#define input2 A0
#define input3 A1
#define input4 A2
#define input5 A3
#define input6 A4

#define ledPin 13     // onboard led
#define buttonPin 11  // push button

/* This example code has 9 holding registers. 6 analogue inputs, 1 button, 1 digital output
   and 1 register to indicate errors encountered since started.
   Function 5 (write single coil) is not implemented so I'm using a whole register
   and function 16 to set the onboard Led on the Atmega328P.

   The modbus_update() method updates the holdingRegs register array and checks communication.

   Note:
   The Arduino serial ring buffer is 128 bytes or 64 registers.
   Most of the time you will connect the arduino to a master via serial
   using a MAX485 or similar.

   In a function 3 request the master will attempt to read from your
   slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
   and two BYTES CRC the master can only request 122 bytes or 61 registers.

   In a function 16 request the master will attempt to write to your
   slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS,
   NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
   118 bytes or 59 registers.

   Using the FTDI USB to Serial converter the maximum bytes you can send is limited
   to its internal buffer which is 60 bytes or 30 unsigned int registers.

   Thus:

   In a function 3 request the master will attempt to read from your
   slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
   and two BYTES CRC the master can only request 54 bytes or 27 registers.

   In a function 16 request the master will attempt to write to your
   slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS,
   NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
   50 bytes or 25 registers.

   Since it is assumed that you will mostly use the Arduino to connect to a
   master without using a USB to Serial converter the internal buffer is set
   the same as the Arduino Serial ring buffer which is 128 bytes.
*/


// Using the enum instruction allows for an easy method for adding and
// removing registers. Doing it this way saves you #defining the size
// of your slaves register array each time you want to add more registers
// and at a glimpse informs you of your slaves register layout.

//////////////// registers of your slave ///////////////////
enum {                    // Bo sung them cac thanh ghi Holding Regiters
  Relay_HoldingRegs_01,   // 0
  Relay_HoldingRegs_11,   // 1
  Relay_HoldingRegs_12,   // 2
  Relay_HoldingRegs_13,   // 3
  Relay_HoldingRegs_14,   // 4
  Relay_HoldingRegs_21,   // 5
  Relay_HoldingRegs_22,   // 6
  Relay_HoldingRegs_23,   // 7
  Relay_HoldingRegs_24,   // 8
  Relay_HoldingRegs_30,   // 9
  Relay_HoldingRegs_31,   // 10
  Relay_HoldingRegs_40,   // 11   SPDT 1
  Relay_HoldingRegs_41,   // 12	  SPDT 2
  Relay_HoldingRegs_42,   //  13	 Reserved
  Relay_HoldingRegs_43,   //  14	 Reserved
  Relay_HoldingRegs_44,   //  15	 Reserved
  Relay_HoldingRegs_45,   //  16	 Reserved
  Opto_HoldingRegs_50_1,  // 17
  Opto_HoldingRegs_50_2,  // 18
  Opto_HoldingRegs_51_1,  // 19
  Opto_HoldingRegs_51_2,  // 20
  Opto_HoldingRegs_52_1,  // 21  Reserved
  Opto_HoldingRegs_52_2,  // 22  Reserved
  Opto_HoldingRegs_53_1,  // 23  Reserved
  Opto_HoldingRegs_53_2,  // 24  Reserved
  counterIdx,             // 25
  TOTAL_ERRORS,
  // add more Regs before this line=======
  HoldingRegsSize
};
#define _boardID 2
#define _board595 12
#define numberOf74HC595 (_board595 << 1)

uint16_t holdingRegs[HoldingRegsSize];  // function 3 and 16 register array
uint8_t Relay_value[numberOf74HC595];
/*
0-3  Relay_HoldingRegs_11 (DPDT)
4-7  Relay_HoldingRegs_12 (DPDT)
8-11 Relay_HoldingRegs_01 (DPDT)
12-15 Relay_HoldingRegs_13 (DPDT)
16-19 Relay_HoldingRegs_14 (DPDT)
20-21 Relay_HoldingRegs_40 (SPDT)
22-23 Relay_HoldingRegs_40 (SPDT)
*/
////////////////////////////////////////////////////////////

void setup() {

  pinMode(RS485, OUTPUT);
  pinMode(SRCLK1, OUTPUT);
  pinMode(RCLK1, OUTPUT);
  pinMode(SER1, OUTPUT);

  pinMode(OE1, OUTPUT);   digitalWrite(OE1, HIGH);

  pinMode(SRCLK2, OUTPUT);
  pinMode(RCLK2, OUTPUT);
  pinMode(SER2, OUTPUT);
  pinMode(OE2, OUTPUT);   digitalWrite(OE2, HIGH);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

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

  modbus_configure(115200, _boardID, 2, HoldingRegsSize, 0);
}

void loop() {
  static uint32_t last_ms;
  static bool bitstatus;
  static uint8_t Relay_value_1[numberOf74HC595];

  // modbus_update() is the only method used in loop(). It returns the total error
  // count since the slave started. You don't have to use it but it's useful
  // for fault finding by the modbus master.
  modbus_update(holdingRegs);

  copyHoldingRegsToRelayVal(Relay_value, holdingRegs);

  if (checkNewRelayValue(Relay_value, Relay_value_1)) {
    write_relay(Relay_value, numberOf74HC595);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (millis() - last_ms > 500) {
    //  digitalWrite(LED_BUILTIN, bitstatus);
    bitstatus ^= true;
    last_ms = millis();
  }
  delay(1);
  }
//=================================
bool checkNewRelayValue(uint8_t* Relay_value_1, const uint8_t* Relay_value) {
  unsigned char idx;
  bool check = false;
  for (idx = 0; idx < numberOf74HC595; idx++) {
    if (*(Relay_value_1 + idx) != *(Relay_value + idx)) {
      check = true;
      break;
    }
  }
  if (check) {
    for (idx = 0; idx < numberOf74HC595; idx++) {
      *(Relay_value_1 + idx) = *(Relay_value + idx);
    }
  }
  return (check);
}
//=================================
void copyHoldingRegsToRelayVal(uint8_t* RelayVal, const uint16_t* RegHolding) {
  unsigned char idx = 0;
  unsigned int u16temp;
  unsigned char u8_H, u8_L;
  for (idx = 0; idx < 6; idx++) {
    switch (idx) {
      case 0:
        u16temp = holdingRegs[Relay_HoldingRegs_11];
        break;
      case 1:
        u16temp = holdingRegs[Relay_HoldingRegs_12];
        break;
      case 2:
        u16temp = holdingRegs[Relay_HoldingRegs_01];
        break;
      case 3:
        u16temp = holdingRegs[Relay_HoldingRegs_13];
        break;
      case 4:
        u16temp = holdingRegs[Relay_HoldingRegs_14];
        break;
      case 5:
        u16temp = holdingRegs[Relay_HoldingRegs_40];
        break;
    }
    u8_H = u16temp >> 8;
    u8_L = u16temp;
    *(RelayVal + 4 * idx) = NibbleToByte(u8_L & 0x0F);
    *(RelayVal + 4 * idx + 1) = NibbleToByte((u8_L & 0xF0) >> 4);
    *(RelayVal + 4 * idx + 2) = NibbleToByte(u8_H & 0x0F);
    *(RelayVal + 4 * idx + 3) = NibbleToByte((u8_H & 0xF0) >> 4);
  }
}
//==============================================
void write_relay(const uint8_t* relay_values, const uint8_t len) {
  unsigned char idx;
  digitalWrite(RCLK1, LOW);
  for (idx = len; idx > 0; idx--) {
    shiftOut(SER1, SRCLK1, MSBFIRST, ~relay_values[idx - 1]);
  }
  digitalWrite(RCLK1, HIGH);
  digitalWrite(OE1, LOW);
}
//=====================
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
