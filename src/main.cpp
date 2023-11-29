#include <Arduino.h>
#define HWUART // comment this if you want to connect to USBSerial

// logic when not using USB
#if defined(HWUART)
#define Serial Serial1
#endif

// motor driver pins
#define AIN1 PB4
#define AIN2 PB5
#define PWMA PB6
#define PWMB PB7
#define BIN2 PB8
#define BIN1 PB9
// #define STBY PB0 // hardwired to HIGH 3V3

// ir sensor pins
#define IR1 PB12
#define IR2 PB13
#define IR3 PB14
#define IR4 PB15
#define IR5 PA8

int pos = 0;
int lsp, rsp;
int Speed = 255;
int correction_count;
int avg_speed = 200;
bool Lost = false;
float kp = 0.5;
float ki = 0.3;
float kd = 0.3;
float pVal, iVal, dVal, pidValue, error;
float prevError = 0;
float integral = 0;
float derivative = 0;
int currentTime, prevTime, dt;

// function prototypes
void line_follow();
void print_ir();
void signal_1();
void pwm_test();
void position();
void reverse(int speed);
void stop();

void setup()
{
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  Serial.begin(9600);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2,LOW);
  lsp = avg_speed;
  rsp = avg_speed;
}

void loop()
{
  int curretTime = millis();
  dt = currentTime - prevTime;
  position();
  float pidVal = pidControl();
  prevTime = currentTime;
  lsp = lsp - pidVal;
  rsp = rsp + pidVal;
  
  analogWrite(PWMA, lsp);
  analogWrite(PWMB, rsp);
  signal_1();
  print_ir();

}

void position(){
  if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 1)){
      pos = -4;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 1) and (digitalRead(IR5) == 1)){
      pos = -3;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 1) and (digitalRead(IR5) == 0)){
      pos = -2;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 1) and (digitalRead(IR4) == 1) and (digitalRead(IR5) == 0)){
      pos = -1;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 1) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0)){
      pos = 0;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 1) and (digitalRead(IR3) == 1) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0)){
      pos = 1;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 1) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0)){
      pos = 2;
  }
  else if ((digitalRead(IR1) == 1) and (digitalRead(IR2) == 1) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0)){
      pos = 3;
  }
  else if ((digitalRead(IR1) == 1) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0)){
      pos = 4;
  }
  else{
    stop();
  }
}

int pidControl()
{
    error = pos * 0.06;
    pVal = kp*error;
    integral += error * dt;
    iVal = ki*integral;
    derivative = (error - prevError)*dt;
    dVal = kd*derivative;
    pidValue = pVal + iVal + dVal;
    return pidValue;
}

void forward()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 255);
  analogWrite(PWMB, 255);
}

void reverse(int speed)
{
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void sharp_right(int speed)
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, 0);
}

void sharp_left(int speed)
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, speed);
}

void go(int speed)
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void stop()
{
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void print_ir()
{
  Serial.print(digitalRead(IR1));
  Serial.print("::");
  Serial.print(digitalRead(IR2));
  Serial.print("::");
  Serial.print(digitalRead(IR3));
  Serial.print("::");
  Serial.print(digitalRead(IR4));
  Serial.print("::");
  Serial.println(digitalRead(IR5));
}

void signal_1()
{
  for (int i = 1; i <= 3; ++i)
  {
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(100);                     // wait for a second
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
    delay(100);                     // wait for a second
  }
}

void pwm_test()
{
  int8_t x = 3;
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  for (int i = 0; i <= 255; i++)
  {
    analogWrite(PWMA, i);
    analogWrite(PWMB, i);
    delay(x*10);
  }
  for (int i = 255; i >= 0; i--)
  {
    analogWrite(PWMA, i);
    analogWrite(PWMB, i);
    delay(x*10);
  }
}