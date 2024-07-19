#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX
struct data
{
  int8_t speed1;
  int8_t speed2;
  int8_t speed3;
  int8_t speed4;
};
data dataSend;
void setup() {
  // put your setup code here, to run once:
mySerial.begin(115200);
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int a = map(analogRead(A0), 0, 1024, -80, 80) - 1;
  Serial.println(a);
  dataSend.speed1 = a;
  dataSend.speed2 = a;
  dataSend.speed3 = a;
  dataSend.speed4 = a;
  mySerial.write((uint8_t *)&dataSend, sizeof(dataSend));
  delay(100);

}
