#include <PID_v1.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LED PC13
#define M4_1 PB0
#define M4_2 PB1
#define M3_1 PA6
#define M3_2 PA7
#define M2_1 PA2
#define M2_2 PA3
#define M1_1 PA0
#define M1_2 PA1

#define E4_A PB12
#define E4_B PB13
#define E3_A PB14
#define E3_B PB15
#define E2_A PA8
#define E2_B PA9
#define E1_A PB3
#define E1_B PA15
//                      RX    TX
HardwareSerial Serial3(PB11, PB10);
HardwareTimer timer(TIM4);
LiquidCrystal_I2C lcd(0x27, 16, 2);

double kp = 1.2, ki = 45, kd = 0.001;
double in1 = 0, out1 = 0, setpoint1 = 0;
double in2 = 0, out2 = 0, setpoint2 = 0;
double in3 = 0, out3 = 0, setpoint3 = 0;
double in4 = 0, out4 = 0, setpoint4 = 0;
PID PIDM1(&in1, &out1, &setpoint1, kp, ki, kd, DIRECT);
PID PIDM2(&in2, &out2, &setpoint2, kp, ki, kd, DIRECT);
PID PIDM3(&in3, &out3, &setpoint3, kp, ki, kd, DIRECT);
PID PIDM4(&in4, &out4, &setpoint4, kp, ki, kd, DIRECT);

int32_t count1, count2, count3, count4;
int16_t speed[4];
struct data
{
  int8_t speed1;
  int8_t speed2;
  int8_t speed3;
  int8_t speed4;
};
data dataRev;

void setup()
{
  lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("init...");
  pinMode(PA10, INPUT);
  Serial.begin(115200);
  Serial.println("init...");
  Serial3.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(E1_A, INPUT_PULLUP);
  pinMode(E1_B, INPUT_PULLUP);
  pinMode(E2_A, INPUT_PULLUP);
  pinMode(E2_B, INPUT_PULLUP);
  pinMode(E3_A, INPUT_PULLUP);
  pinMode(E3_B, INPUT_PULLUP);
  pinMode(E4_A, INPUT_PULLUP);
  pinMode(E4_B, INPUT_PULLUP);
  pinMode(M1_1, OUTPUT);
  pinMode(M1_2, OUTPUT);
  pinMode(M2_1, OUTPUT);
  pinMode(M2_2, OUTPUT);
  pinMode(M3_1, OUTPUT);
  pinMode(M3_2, OUTPUT);
  pinMode(M4_1, OUTPUT);
  pinMode(M4_2, OUTPUT);
  analogWriteFrequency(5000);
  attachInterrupt(digitalPinToInterrupt(E1_A), countM1_1, RISING);
  attachInterrupt(digitalPinToInterrupt(E1_B), countM1_2, RISING);
  attachInterrupt(digitalPinToInterrupt(E2_A), countM2_1, RISING);
  attachInterrupt(digitalPinToInterrupt(E2_B), countM2_2, RISING);
  attachInterrupt(digitalPinToInterrupt(E3_A), countM3_1, RISING);
  attachInterrupt(digitalPinToInterrupt(E3_B), countM3_2, RISING);
  attachInterrupt(digitalPinToInterrupt(E4_A), countM4_1, RISING);
  attachInterrupt(digitalPinToInterrupt(E4_B), countM4_2, RISING);
  timer.pause();
  timer.setOverflow(25000, MICROSEC_FORMAT);
  timer.attachInterrupt(calculate);
  timer.resume();

  PIDM1.SetMode(AUTOMATIC);
  PIDM1.SetSampleTime(25);
  PIDM1.SetOutputLimits(-255, 255);

  PIDM2.SetMode(AUTOMATIC);
  PIDM2.SetSampleTime(25);
  PIDM2.SetOutputLimits(-255, 255);

  PIDM3.SetMode(AUTOMATIC);
  PIDM3.SetSampleTime(25);
  PIDM3.SetOutputLimits(-255, 255);

  PIDM4.SetMode(AUTOMATIC);
  PIDM4.SetSampleTime(25);
  PIDM4.SetOutputLimits(-255, 255);

  lcd.clear();
}
uint32_t lastRev, lastPrint1;
void loop()
{
  if (Serial.available() > 0)
  {
  }
  if (Serial3.available() > 0)
  {
    Serial3.readBytes((uint8_t*)&dataRev, sizeof(dataRev));
    setpoint1 = dataRev.speed1;
    setpoint2 = dataRev.speed2;
    setpoint3 = dataRev.speed3;
    setpoint4 = dataRev.speed4;
    lastRev = millis();
    digitalWrite(LED, !digitalRead(LED));
  }
  if (millis() - lastRev> 300)
  {
    setpoint1 = 0;
    setpoint2 = 0;
    setpoint3 = 0;
    setpoint4 = 0;
  }
  
  
  in1 = speed[0];
  in2 = speed[1];
  in3 = speed[2];
  in4 = speed[3];
  PIDM1.Compute();
  PIDM2.Compute();
  PIDM3.Compute();
  PIDM4.Compute();
  controlMotor(M1_1, M1_2, out1);
  controlMotor(M2_1, M2_2, out2);
  controlMotor(M3_1, M3_2, out3);
  controlMotor(M4_1, M4_2, out4);
  if (millis() - lastPrint1 > 100)
  {
    lastPrint1 = millis();
    lcd.setCursor(0,0);
    lcd.print(speed[0]);
    lcd.print("   ");
    lcd.setCursor(8,0);
    lcd.print(speed[1]);
    lcd.print("   ");
    lcd.setCursor(0,1);
    lcd.print(speed[2]);
    lcd.print("   ");
    lcd.setCursor(8,1);
    lcd.print(speed[3]);
    lcd.print("   ");
  }
  

}


void countM1_1()
{
  if (!digitalRead(E1_B))
  {
    count1++;
  }
  else
  {
    count1--;
  }
}

void countM1_2()
{
  if (digitalRead(E1_A))
  {
    count1++;
  }
  else
  {
    count1--;
  }
}
void countM2_1()
{
  if (!digitalRead(E2_B))
  {
    count2++;
  }
  else
  {
    count2--;
  }
}
void countM2_2()
{
  if (digitalRead(E2_A))
  {
    count2++;
  }
  else
  {
    count2--;
  }
}
void countM3_1()
{
  if (!digitalRead(E3_B))
  {
    count3++;
  }
  else
  {
    count3--;
  }
}
void countM3_2()
{
  if (digitalRead(E3_A))
  {
    count3++;
  }
  else
  {
    count3--;
  }
}

void countM4_1()
{
  if (!digitalRead(E4_B))
  {
    count4++;
  }
  else
  {
    count4--;
  }
}
void countM4_2()
{
  if (digitalRead(E4_A))
  {
    count4++;
  }
  else
  {
    count4--;
  }
}
void calculate()
{
  speed[0] = count1;
  speed[1] = count2;
  speed[2] = count3;
  speed[3] = count4;
  count1 = 0;
  count2 = 0;
  count3 = 0;
  count4 = 0;
}
void controlMotor(uint8_t pin1, uint8_t pin2, int16_t pwm)
{

  if (pwm > 15)
  {
    analogWrite(pin1, pwm);
    analogWrite(pin2, 0);
  }
  else if (pwm < -15)
  {
    analogWrite(pin2, -pwm);
    analogWrite(pin1, 0);
  }
  else
  {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}
