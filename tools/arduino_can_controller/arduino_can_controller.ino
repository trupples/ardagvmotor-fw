/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Arduino_CAN.h>

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void RXPDO(int nodeId, int pdoId, int numBytes, uint8_t *bytes) {
  uint8_t b[16];
  memcpy(b, bytes, numBytes);
  CAN.write(CanMsg(0x100 + 0x100 * pdoId + nodeId, numBytes, b));
}

void TXPDO(int nodeId, int pdoId, int numBytes, uint8_t *bytes) {
  uint8_t b[16];
  memcpy(b, bytes, numBytes);
  CAN.write(CanMsg(0x080 + 0x100 * pdoId + nodeId, numBytes, b));
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { }

  if (!CAN.begin(CanBitRate::BR_500k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }

  Serial.println("Setting operational");

  uint8_t mesaj[8] = { 1, 0 };
  CanMsg set_opertaional(0, 2, mesaj);
  CAN.write(set_opertaional);

  for(int drive = 0x15; drive <= 0x16; drive++) {
    Serial.print("Bringing drive ");
    Serial.print(drive, HEX);
    Serial.println("to operation enable");
    // Switch on disabled -> ready to switch on 2
    uint8_t transition2[] = {0b00110, 0};
    RXPDO(drive, 1, 2, transition2);
    delay(100);
    // Ready to switch on -> switched on 3
    uint8_t transition3[] = {0b00111, 0};
    RXPDO(drive, 1, 2, transition3);
    delay(100);
    // Switched on -> operation enable 4
    uint8_t transition4[] = {0b01111, 0};
    RXPDO(drive, 1, 2, transition4);
    delay(100);
  }
}

static uint32_t msg_cnt = 0;

void loop()
{

  while(CAN.available()) {
    CanMsg msg = CAN.read();
    char buf[101];
    char *p = buf;
    p += snprintf(buf, 100, "%03X ", msg.id);
    for(int i = 0; i < msg.data_length; i++) {
      p += snprintf(p, 100, "%02X ", msg.data[i]);
    }
    Serial.println(buf);
  }

  static int last_set = 0;
  int now = millis();
  if(last_set + 100 <= now) {
    last_set = now;
    union {
      int32_t v;
      uint8_t b[4];
    };
    Serial.print("Setting left = ");
    Serial.print(v);
    v = (int) (sin(now / 1000.0f) * 1000000.0f);
    uint8_t set_speed_left[] = {
      0b01111, 0, /* Controlword */
      b[0], b[1], b[2], b[3]
    };
    v = (int) (cos(now / 1000.0f) * 1000000.0f);
    uint8_t set_speed_right[] = {
      0b01111, 0, /* Controlword */
      b[0], b[1], b[2], b[3]
    };
    Serial.print(" right = ");
    Serial.print(v);
    Serial.println();
    RXPDO(0x15, 4, 6, set_speed_left);
    RXPDO(0x16, 4, 6, set_speed_right);
  }
}
