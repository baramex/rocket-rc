#include <LoRa.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

// Rates
unsigned long lastTransmit = 0, lastPulse = 0, lastReceive = 0, pressedTime = 0;
const unsigned transmitDelay = 300, pulseDelay = 700, pressDelay = 300;
bool pulseState = 0;

// Pinout
const byte buttonIn = 5, buttonOut = 4, buttonLight = 3;
const byte jsX = A2, jsY = A3;

// Control
enum Mode {
  LOCKED,
  UNLOCKED
};
Mode mode = LOCKED;
enum PlDirection {
  STRAIGHT,
  LEFT,
  RIGHT,
  DOWN,
  UP
};
PlDirection direction = STRAIGHT;
bool pressed = false;
bool actioned = false;

// From PL
enum State {
  PREPARING = 1,
  READY = 2,
  TAKINGOFF = 3,
  GLIDING = 4
};
float latitude, longitude, altitude;
State plState;
byte rightMotor, leftMotor;
int rssi;
//byte buffer[4];
bool lostSignal = true;

// Display
byte degreeSymbol[8] = {
  0b00110,
  0b01001,
  0b01001,
  0b00110,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

void setup() {
  // Debug
  Serial.begin(9600);

  // Screen
  /*lcd.init();
  lcd.backlight();
  lcd.setCursor(5, 1);
  lcd.print("FourSight");
  lcd.setCursor(1, 2);
  lcd.print("Remote Controller");
  lcd.createChar(0, degreeSymbol);*/

  // Pinout
  pinMode(buttonIn, INPUT_PULLUP);
  pinMode(buttonOut, OUTPUT);
  digitalWrite(buttonOut, LOW);
  pinMode(buttonLight, OUTPUT);

  pinMode(jsX, INPUT);
  pinMode(jsY, INPUT);

  // LoRa
  LoRa.setPins(10, 8, 2);
  while (!LoRa.begin(433E6)) {
    Serial.print(".");
    delay(10);
  }
  LoRa.enableCrc();
  LoRa.setSyncWord(0xD5);
  LoRa.setTxPower(20);

  //lcd.clear();

  Serial.println("Init");

  updateSignalStatus();
  updateDirection();
}

void loop() {
  delay(1);
  unsigned long now = millis();

  // Send packet
  if ((now - lastTransmit) >= transmitDelay) {
    transmitToPL();
    lastTransmit = now;
  }
  delay(5);

  // Receive packet
  //const unsigned packetSize = LoRa.parsePacket(/*16*/);
  /*if (packetSize && now - lastReceive >= 500) {
    if (receiveFromPL()) {
      flushLoRa();
      updateRssi();
      Serial.println("Receive " + String(now - lastReceive));
      if (lostSignal) {
        lostSignal = false;
        updateSignalStatus();
      }
      lastReceive = now;
    }
  }*/

  if (lastReceive && now - lastReceive > 5000 && !lostSignal) {
    lostSignal = true;
    updateSignalStatus();
  }

  // Read joystick
  if (mode == UNLOCKED) {
    PlDirection oldDirection = direction;
    direction = getJoystickDirection();
    if (oldDirection != direction) {
      updateDirection();
    }
  }

  // Control button
  controlButton(now);
}

bool receiveFromPL() {
  if (LoRa.read() != 0xA4) return false;
  State tempState;
  bool stateDiff = false;
  bool dirDiff = false;
  //tempState = LoRa.read();
  tempState = LoRa.parseInt();
  if (tempState != plState && tempState >= 1 && tempState <= 4) {
    plState = tempState;
    stateDiff = true;

    if (plState == TAKINGOFF || plState == GLIDING) {
      mode = UNLOCKED;
      dirDiff = true;
    }
  }

  bool motorDiff = false;
  byte temp;
  temp = LoRa.parseInt();
  //temp = LoRa.read();
  if (rightMotor != temp) {
    motorDiff = true;
    rightMotor = temp;
  }
  temp = LoRa.parseInt();
  //temp = LoRa.read();
  if (leftMotor != temp) {
    motorDiff = true;
    leftMotor = temp;
  }

  bool altDiff = false;
  float ftemp;
  ftemp = LoRa.parseFloat();
  //ftemp = readFloat();
  if (abs(ftemp - altitude) > 0.1) {
    altitude = ftemp;
    altDiff = true;
  }

  bool gpsDiff = false;
  ftemp = LoRa.parseFloat();
  //ftemp = readFloat();
  if (ftemp > -90 && ftemp < 90) {
    if (abs(latitude - ftemp) > 0.00001) {
      gpsDiff = true;
    }
    latitude = ftemp;
  }
  //ftemp = readFloat();
  ftemp = LoRa.parseFloat();
  if (ftemp > -180 && ftemp < 180) {
    if (abs(longitude - ftemp) > 0.00001) {
      gpsDiff = true;
    }
    longitude = ftemp;
  }

  rssi = LoRa.packetRssi();

  if (stateDiff) updateState();
  if (dirDiff) updateDirection();
  if (motorDiff) updateMotorPosition();
  if (altDiff) updateAltitude();
  if (gpsDiff) updateCoordinates();
  return true;
}

void flushLoRa() {
  while (LoRa.available() > 0) {
    LoRa.read();
  }
}

void transmitToPL() {
  if (!LoRa.beginPacket()) return;
  if (mode == LOCKED) direction = STRAIGHT;
  LoRa.write(0xC4);
  LoRa.print(mode);
  LoRa.print(",");
  LoRa.print(direction);
  Serial.println("Sending");
  LoRa.endPacket();
  Serial.println("Send");
}

void controlButton(const unsigned long& now) {
  if (mode == UNLOCKED) {
    if ((now - lastPulse) >= pulseDelay) {
      digitalWrite(buttonLight, pulseState);
      pulseState = !pulseState;
      lastPulse = now;
    }
  } else {
    digitalWrite(buttonLight, LOW);
  }

  if (!digitalRead(buttonIn)) {
    if (!pressed) {
      pressed = true;
      pressedTime = now;
    }
    if (now - pressedTime >= pressDelay && !actioned) {
      mode = mode == LOCKED ? UNLOCKED : LOCKED;
      updateDirection();
      actioned = true;
    }
  } else {
    pressed = false;
    actioned = false;
  }
}

PlDirection getJoystickDirection() {
  const int x = map(analogRead(jsX), 0, 1028, 0, 100);
  const int y = map(analogRead(jsY), 0, 1028, 0, 100);

  if ((x < 20 || x > 80) && (y < 20 || y > 80) && direction != STRAIGHT) {
    return direction;
  }

  if (x < 30) return RIGHT;
  if (x > 70) return LEFT;
  if (y < 30) return DOWN;
  if (y > 70) return UP;

  return STRAIGHT;
}

void updateRssi() {
  //lcd.setCursor(0, 0);
  //printSection(String(rssi), 4);
  Serial.println("Update RSSI: " + String(rssi));
}

void updateSignalStatus() {
  //lcd.setCursor(4, 0);
  //printSection(lostSignal ? "NO SIGNAL" : "OK", 9);
}

void updateCoordinates() {
  /*lcd.setCursor(0, 1);
  lcd.print(String(latitude, 5));
  lcd.write(0);
  lcd.print("," + String(longitude, 5));
  lcd.write(0);*/
  Serial.println("Update coordinates: " + String(latitude, 5) + " " + String(longitude, 5));
}

void updateAltitude() {
  /*lcd.setCursor(0, 2);
  printSection(String(altitude, 1) + "m", 8);*/
  Serial.println("Update altitude: " + String(altitude, 1));
}

void updateState() {
  /*lcd.setCursor(8, 2);
  printSection(getStringState(), 12);*/
  Serial.println("Update state: " + getStringState());
}

void updateMotorPosition() {
  /*lcd.setCursor(0, 3);
  printSection("L:" + String(leftMotor * 100 / 255) + "%,R:" + String(rightMotor * 100 / 255) + "%", 12);*/
  Serial.println("Update motor positon: L:" + String(leftMotor * 100 / 255) + "%,R:" + String(rightMotor * 100 / 255) + "%");
}

void updateDirection() {
  //lcd.setCursor(12, 3);
  String dir;
  switch (direction) {
    case RIGHT: dir = "RIGHT"; break;
    case LEFT: dir = "LEFT"; break;
    case UP: dir = "UP"; break;
    case DOWN: dir = "DOWN"; break;
    case STRAIGHT: dir = "STRAIGHT"; break;
  }
  //printSection(mode == LOCKED ? "LOCKED" : dir, 8);
  Serial.println("Update direction: " + dir);
}

String getStringState() {
  switch (plState) {
    case PREPARING: return "Preparing...";
    case READY: return "Ready";
    case TAKINGOFF: return "Taking Off";
    case GLIDING: return "Gliding";
    default: return "Unknown";
  }
}

/*float readFloat() {
  if (LoRa.available() < 4) return 0;
  LoRa.readBytes(buffer, 4);
  float fl;
  memcpy(&fl, buffer, 4);
  return fl;
}*/

void printSection(const String& str, const uint8_t width) {
  uint8_t size = str.length();
  if (width > size) {
    for (uint8_t i = 0; i < width - size; i++) {
      str += " ";
    }
  }
  lcd.print(str);
}