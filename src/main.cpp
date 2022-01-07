#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "CytronMotorDriver.h"

#define __IS_ON_FRONT_LINE (isOnLine[0][0] || isOnLine[0][1] || isOnLine[0][2])
#define __IS_ON_RIGHT_LINE (isOnLine[1][0] || isOnLine[1][1] || isOnLine[1][2])
#define __IS_ON_BACK_LINE (isOnLine[2][0] || isOnLine[2][1] || isOnLine[2][2])
#define __IS_ON_LEFT_LINE (isOnLine[3][0] || isOnLine[3][1] || isOnLine[3][2])

#define SPEED 215
#define MAX 255

#define SPEAKER 33
#define OpenMV Serial7

double speed2 = 0;

CytronMD motor1(PWM_DIR, 5, 4);
CytronMD motor2(PWM_DIR, 7, 6);
CytronMD motor3(PWM_DIR, 3, 2);
CytronMD motor4(PWM_DIR, 9, 8);

MPU6050 mpu;
static uint8_t mpuIntStatus;
static bool dmpReady = false;
static uint16_t packetSize;
int16_t Gyro_Now = 0, Gyro = 0, Gyro_Offset = 0;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
int Gyro_X, Gyro_Y, Gyro_Z, Accel_Z;

int ave_motor_power[4][10] = {0};
int ave_mpPlus = 0;

void Gyro_init(void);
void motor(int angle);
void motorStop();
void line_motor(int angle);
void debug();
void tone_err();
void mpu_err();
void tone_setup();
void tone_isOnLine();
void kick();
void lineMotor();

int GyroGet(void);
int getVah(int f);
int IRval(int i);
int getIR();
int getCam();

int prevIR, dirPlus, cnt;
int dirIR = 0;

void tone_setup() {
   int t = 130;
   tone(SPEAKER,3000,t);
   delay(t);
   tone(SPEAKER,2500,t);
   delay(t);
   tone(SPEAKER,3600,t * 2);
   delay(t * 4);
   tone(SPEAKER,2900,t * 4);
   delay(t * 4);
}

void tone_isOnLine() {
   tone(SPEAKER, 2000, 1);
}

void tone_err() {
   tone(SPEAKER, 100, 1000);
   delay(2000);
}

void mpu_err() {
   tone(SPEAKER, 100, 1000);
   delay(1000);
}

int GyroGet(void) {
   mpuIntStatus = false;
   mpuIntStatus = mpu.getIntStatus();
   fifoCount = mpu.getFIFOCount();

   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
   }
   else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) {
         fifoCount = mpu.getFIFOCount();
      }
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Gyro_Now = degrees(ypr[0]) + 180;
      Gyro = Gyro_Now + Gyro_Offset - 180;
      if (Gyro < 0) {
         Gyro += 360;
      }
      if (Gyro > 359) {
         Gyro -= 360;
      }
   }
   return Gyro;
}

void Gryo_init() {
   mpu.initialize();
   if (mpu.testConnection() != true) {
      Serial.println("MPU disconection");
   }
   if (mpu.dmpInitialize() != 0) {
      Serial.println("MPU break");
   }
   mpu.setXGyroOffset(Gyro_X);
   mpu.setYGyroOffset(Gyro_Y);
   mpu.setZGyroOffset(Gyro_Z);
   mpu.setZAccelOffset(Accel_Z);
   mpu.setDMPEnabled(true);
   mpuIntStatus = mpu.getIntStatus();
   dmpReady = true;
   packetSize = mpu.dmpGetFIFOPacketSize();
}

void selectGyro(int number) {
   switch (number) {
      case 0:
         Gyro_X = -93, Gyro_Y = 77, Gyro_Z = 10, Accel_Z = 1561;
         break;
      case 1:
         Gyro_X = 114, Gyro_Y = 61, Gyro_Z = 95, Accel_Z = 1719;
         break;
      case 2:
         Gyro_X = -118, Gyro_Y = 22, Gyro_Z = -61, Accel_Z = 797;
         break;
      case 3:
         Gyro_X = -364, Gyro_Y = -29, Gyro_Z = 9, Accel_Z = 1846;
         break;
      case 4:
         Gyro_X = 50, Gyro_Y = 22, Gyro_Z = 12, Accel_Z = 1347;
         break;
      default:
         Serial.println("error in selectGyro()");
         Gyro_X = 0, Gyro_Y = 0, Gyro_Z = 0, Accel_Z = 0;
         break;
   }
}

int getVah(int f) {
   byte val = 0;
   Wire.beginTransmission(0x0E);
   Wire.write(f);
   Wire.endTransmission();
   Wire.requestFrom(0x0E, 1);
   while (Wire.available()) {
      val = Wire.read();
   }
   return (int)val;
}

int IRval(int i) {
   int a = getVah(0x04);
   int b = getVah(0x05);
   int c = getVah(0x06);
   int d = getVah(0x07);
   int re_angle;
   int re_strength;

   if (d < 10) {
      re_angle = a;
      re_strength = b;
   }
   else {
      re_angle = c;
      re_strength = d;
   }

   if(i != 1) {
      return re_strength;
   }
   else {
      return re_angle*5;
   }
}

int globalCamVal = 0;

void motor(int angle) {

   int GyroVal = 0;
   int camVal = getCam();

   if (!(camVal > 0 && camVal <= 70)) {
      camVal = 0;
   }

   if (camVal != 0) {
      globalCamVal = camVal - 35;
   }
   else {
      GyroVal = GyroGet();
   }

   while (GyroVal > 359) {
      GyroVal -= 359;
   }

   double motor_power[4];
   double max_power;

   motor_power[0] = cos((45 - angle) / 180.0 * PI);
   motor_power[1] = cos((135 - angle) / 180.0 * PI);
   motor_power[2] = cos((-45 - angle) / 180.0 * PI);
   motor_power[3] = cos((-135 - angle) / 180.0 * PI);

   for (int i = 0; i < 4; i++) {
      if (abs(motor_power[i]) > max_power) {
         max_power = abs(motor_power[i]);
      }
   }

   speed2 = SPEED;

   for (int i = 0; i < 4; i++) {
      motor_power[i] = speed2 * motor_power[i] / max_power;
      for (int j = 9; j > 0; j--) {
         ave_motor_power[i][j] = ave_motor_power[i][j - 1];
      }
      ave_motor_power[i][0] = motor_power[i];
      ave_mpPlus = 0;
      for (int k = 0; k < 10; k++) {
         ave_mpPlus = ave_mpPlus + ave_motor_power[i][k];
      }
      motor_power[i] = ave_mpPlus / 10;
   }

   if (globalCamVal >= -34 && globalCamVal <= 35) {
      int rollPower = SPEED / 2;
      if ((GyroVal >= 36) && (GyroVal < 90)) {
         motor1.setSpeed(rollPower);
         motor2.setSpeed(rollPower);
         motor3.setSpeed(rollPower);
         motor4.setSpeed(rollPower);
      }
      else if ((GyroVal >= 90) && (GyroVal <= 180)) {
         motor1.setSpeed(MAX);
         motor2.setSpeed(MAX);
         motor3.setSpeed(MAX);
         motor4.setSpeed(MAX);
      }
      else if ((GyroVal <= 323) && (GyroVal > 270)) {
         motor1.setSpeed(-rollPower);
         motor2.setSpeed(-rollPower);
         motor3.setSpeed(-rollPower);
         motor4.setSpeed(-rollPower);
      }
      else if ((GyroVal <= 270) && (GyroVal > 180)) {
         motor1.setSpeed(-MAX);
         motor2.setSpeed(-MAX);
         motor3.setSpeed(-MAX);
         motor4.setSpeed(-MAX);
      }
      else {
         motor1.setSpeed(motor_power[0] - globalCamVal);
         motor2.setSpeed(motor_power[1] - globalCamVal);
         motor3.setSpeed(motor_power[2] - globalCamVal);
         motor4.setSpeed(motor_power[3] - globalCamVal);
      }
   }
}

void lineMotor(int angle) {
   int GyroVal = 0;
   int camVal = getCam();

   if (!(camVal > 0 && camVal <= 70)) {
      camVal = 0;
   }
   if (camVal != 0) {
      globalCamVal = camVal - 35;
   }
   else {
      GyroVal = GyroGet();
   }

   while (GyroVal > 359) {
      GyroVal -= 359;
   }

   double motor_power[4];
   double max_power;

   motor_power[0] = cos((45 - angle) / 180.0 * PI);
   motor_power[1] = cos((135 - angle) / 180.0 * PI);
   motor_power[2] = cos((-45 - angle) / 180.0 * PI);
   motor_power[3] = cos((-135 - angle) / 180.0 * PI);

   for (int i = 0; i < 4; i++) {
      if (abs(motor_power[i]) > max_power) {
         max_power = abs(motor_power[i]);
      }
   }

   speed2 = SPEED;

   for (int i = 0; i < 4; i++) {
      motor_power[i] = speed2 * motor_power[i] / max_power;
   }

   motor1.setSpeed(motor_power[0]);
   motor2.setSpeed(motor_power[1]);
   motor3.setSpeed(motor_power[2]);
   motor4.setSpeed(motor_power[3]);

}

void motorStop() {
   motor1.setSpeed(0);
   motor2.setSpeed(0);
   motor3.setSpeed(0);
   motor4.setSpeed(0);
}

void kick() {
   digitalWrite(12,HIGH);
   delay(75);
   digitalWrite(12,LOW);
   delay(1);
}

void debug() {
   char debug[64];
   sprintf(debug, "IR-> %d GY-> %d canRun-> %d camera-> %d", IRval(1), GyroGet(), digitalRead(32), getCam());
   Serial.println(debug);
}

int GetLine(int head, int num) {
   int analogPins[4][3] = {
      {11, 12, 13},
      {3, 6, 7},
      {8, 9, 10},
      {0, 1, 2}
   };
   return analogRead(analogPins[head][num]);
}

int camAngle = 0;
bool isCatchBall = true;

int getCam() {
   int re = 0;
   if (OpenMV.available()) {
      int a = OpenMV.read();
      if (a >= 0 && a <= 70) {
         re = a;
      }
      else {
         re = 0;
      }
   }
   return re;
}

void setup() {
   Serial.begin(19200);
   OpenMV.begin(19200);
   Wire.begin();
   selectGyro(0);
   Gryo_init();
   tone_setup();
   pinMode(32, INPUT);
   pinMode(30, OUTPUT);
   pinMode(31, OUTPUT);
   pinMode(12, OUTPUT);
}

void loop() {

   // debug();

   bool canRun = 0;
   canRun = digitalRead(32);

   int avoidLineCnt = 0;

   char front[64];
   sprintf(front, "(0.0) ->  %d (0.1) -> %d", GetLine(0, 0), GetLine(0, 1));
   char left[64];
   sprintf(left, "(1.0) ->  %d (1.1) ->  %d (1.2) ->  %d", GetLine(1, 0), GetLine(1, 1), GetLine(1, 2));
   char right[64];
   sprintf(right, "(1.0) ->  %d (1.1) ->  %d (1.2) ->  %d", GetLine(2, 0), GetLine(2, 1), GetLine(2, 2));
   char back[64];
   sprintf(back, "(2.0) ->  %d (2.1) ->  %d (2.2) ->  %d", GetLine(3, 0), GetLine(3, 1), GetLine(3, 2));

   // Serial.println(front);

   if (canRun) {

      bool isAvoidLines = false;

      if(isAvoidLines) {
         float vectorX = 0.0, vectorY = 0.0;
         float vec = 1.0;
         String isOnLines = "";
         if ((GetLine(0, 0) > 1023) || (GetLine(0, 1) > 125)) {
            vectorX += -vec;
            vectorY += 0.0;
            avoidLineCnt++;
            isOnLines += "Front";
         }
         if ((GetLine(1, 0) > 200) || (GetLine(1, 1) > 90) || (GetLine(1, 2) > 100)) {
            vectorX += 0.0;
            vectorY += vec;
            avoidLineCnt++;
            isOnLines += " Left ";
         }
         if ((GetLine(2, 0) > 50) || (GetLine(2, 1) > 70) || (GetLine(2, 2) > 200)) {
            vectorX += 0.0;
            vectorY += -vec;
            avoidLineCnt++;
            isOnLines += " Right ";
         }
         if ((GetLine(3, 0) > 200) || (GetLine(3, 1) > 40) || (GetLine(3, 2) > 90)) {
            vectorX += vec;
            vectorY += 0.0;
            avoidLineCnt++;
            isOnLines += " Back ";
         }
         if(avoidLineCnt != 0) {
            // char XY[64];
            // sprintf(XY, "vectorX -> %f vectorY -> %f", vectorX, vectorY);
            // double finalAngle = atan2(vectorX, vectorY) * 180.0 / PI;
            // Serial.println(finalAngle);
            // motorStop();
            // lineMotor(int(finalAngle));
            // delay(20);
            char XY[64];
            sprintf(XY, "vectorX -> %f vectorY -> %f", vectorX, vectorY);
            double finalAngle = atan2(vectorX, vectorY) * 180.0 / PI;
            lineMotor(int(finalAngle));
            Serial.println(finalAngle);
            delay(20);
         }
      }

      if(avoidLineCnt == 0) {
         dirIR = IRval(1);
         if (abs(prevIR - dirIR) > 110) {
            cnt++;
            if (cnt == 5) {
               cnt = 0;
               prevIR = dirIR;
            }
            else {
               dirIR = prevIR;
            }
         }
         else {
            cnt = 0;
            prevIR = dirIR;
         }
         if (dirIR <= 35) {
            dirPlus = dirIR ;
         }
         else if (dirIR >= 325) {
            dirPlus = (360 - dirIR);
         } 
         else {
            dirPlus = 50;
         }

         dirPlus = dirPlus + 10;

         if (getVah(0x05) <= 50 && getVah(0x07) <= 10) {
            motor(dirIR);
         }
         else if (getVah(0x05) >= 100 && getVah(0x07) >= 15) {
            if (dirIR <= 5 || dirIR >= 355) {
               motor(0);
            } 
            else {
               if (dirIR <= 180) {
                  motor(dirIR + dirPlus * 2);
               } 
               else {
                  motor(dirIR - dirPlus * 2);
               }
            }
         } 
         else {
            if (dirIR <= 180) {
               motor(dirIR + dirPlus);
            } 
            else {
               motor(dirIR - dirPlus);
            }
         }
      }
   }
   else {
      motorStop();
   }
}