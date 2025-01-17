#include <AlfredoCRSF.h>
#include <Arduino_CAN.h>

AlfredoCRSF crsf;

void RXPDO(int nodeId, int pdoId, int numBytes, uint8_t *bytes) {
  uint8_t b[16];
  memcpy(b, bytes, numBytes);
  CAN.write(CanMsg(0x100 + 0x100 * pdoId + nodeId, numBytes, b));
}

void startDrive(int nodeId) {
  // NMT: set operational
  uint8_t nmt_set_operational_buf[8] = { 1, nodeId };
  CanMsg nmt_set_operational(0, 2, nmt_set_operational_buf);
  CAN.write(nmt_set_operational);

  // CANopen CiA 402 bodge:

  // Switch on disabled -> ready to switch on 2
  uint8_t transition2[] = {0b00110, 0};
  RXPDO(nodeId, 1, 2, transition2);
  delay(100);
  // Ready to switch on -> switched on 3
  uint8_t transition3[] = {0b00111, 0};
  RXPDO(nodeId, 1, 2, transition3);
  delay(100);
  // Switched on -> operation enable 4
  uint8_t transition4[] = {0b01111, 0};
  RXPDO(nodeId, 1, 2, transition4);
}

void setDriveSpeed(int nodeId, int speed) {
    union {
      int32_t v;
      uint8_t b[4];
    };
    v = speed;
    uint8_t set_speed_buf[] = {
      0b01111, 0, /* Controlword */
      b[0], b[1], b[2], b[3]
    };
    RXPDO(nodeId, 4, 6, set_speed_buf);
}


void setup()
{
  Serial.begin(115200);
  Serial.println("Serial initialized");

  Serial.println("Initializing UART1");  
  Serial1.begin(CRSF_BAUDRATE);
  if(!Serial1) {
    Serial.println("UART1 failed");
    for (;;) {}
  }

  Serial.println("Initializing CRSF");
  crsf.begin(Serial1);

  Serial.println("Initializing CAN");
  if (!CAN.begin(CanBitRate::BR_500k))
  {
    Serial.println("CAN failed");
    for (;;) {}
  }

  Serial.println("Starting CAN nodes");
  startDrive(0x15);
  startDrive(0x16);

  Serial.println("Init done!");
}

void loop()
{
  static int last = 0;
  int now = millis();

  crsf.update();

  // 20Hz
  if(last + 50 <= now) {
    last = now;

    const int throttle_range = 200; // internal velocity units, TODO change to natural units
    const int steering_range = 200; // internal velocity units, TODO change to natural units

    Serial.print("Channels: ");
    for(int i = 1; i <= 8; i++) {
      Serial.print(crsf.getChannel(i));
      Serial.print(" ");
    }
    Serial.println();

    int throttle = map(crsf.getChannel(2), 1000, 2000, -throttle_range, throttle_range);
    int steering = map(crsf.getChannel(1), 1000, 2000, -steering_range, steering_range);

    Serial.print("Throttle, steering: ");
    Serial.print(throttle);
    Serial.print(" ");
    Serial.print(steering);
    Serial.println();

    int spdTgtL = 0, spdTgtR = 0;

    
    if ((throttle < 10) && (throttle > -10) && ((steering < -10) || (steering > 10))) {
      // Turn on the spot
      spdTgtL = steering;
      spdTgtR = (-1) * steering;
    } else {
      spdTgtL = throttle * ((-steering_range - steering) * 1.0f / -steering_range);
      spdTgtR = throttle * ((steering_range - steering) * 1.0f / steering_range);
      
      if (throttle > 0 && spdTgtL > throttle)
        spdTgtL = throttle;
      if (throttle > 0 && spdTgtR > throttle)
        spdTgtR = throttle;
      if (throttle < 0 && spdTgtL < throttle)
        spdTgtL = throttle;
      if (throttle < 0 && spdTgtR < throttle)
        spdTgtR = throttle;
    }
    spdTgtR = spdTgtR * (-1); // motors are mirrored

    Serial.print("Speed:\t");
    Serial.print(spdTgtL);
    Serial.print("\t");
    Serial.print(spdTgtR);
    Serial.println();

    setDriveSpeed(0x15, spdTgtL * 20000);
    setDriveSpeed(0x16, spdTgtR * 20000);
  }
}
