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

// encoder pins
#define encoderInA PA1
#define encoderInB PA3


int pos = 0;
int lsp, rsp;
int Speed = 255;
int avg_speed = 150;

float kp = 8; //1.2;
float ki = 0; //0.7;
float kd = 1; //0.5;
float pVal, iVal, dVal, pidValue, error;
float prevError = 0;
float integral = 0;
float derivative = 0;
// int currentTime, prevTime, dt;
bool isPos;

//encoder
volatile int countA = 0;
volatile int countB = 0;

float pidVal;

// function prototypes
// void line_follow();
void print_ir();
// void signal_1();
// void pwm_test();
int position();
// void reverse(int speed);
void stop();
float pidControl();
void sharp_right(int spd);
void sharp_left(int spd);
void brake_free();
void brake_fast();

void encIncrementA() {
  countA ++;
}

void encIncrementB() {
  countB ++;
}

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
  pinMode (encoderInA, INPUT);
  attachInterrupt (digitalPinToInterrupt (encoderInA), encIncrementA, RISING);
  pinMode (encoderInB, INPUT);
  attachInterrupt (digitalPinToInterrupt (encoderInB), encIncrementB, RISING);
  Serial.begin(9600);
  pidValue = 0;
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  // digitalWrite(BIN1, HIGH);
  // digitalWrite(BIN2, LOW);
  // lsp = avg_speed;
  // rsp = avg_speed;

}

void loop()
{

  // int curretTime = getCurrentMillis();
  // dt = currentTime - prevTime;
  pos = position();
  pidVal = pidControl();
  if (isPos)
  {
    Serial.println("in isPos");
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    lsp = avg_speed - pidVal;
    rsp = avg_speed + pidVal;
    if (lsp > 225)
    {
      lsp = 225;
    }
    if (lsp < 75)
    {
      lsp = 75;
    }
    if (rsp > 225)
    {
      rsp = 225;
    }
    if (rsp < 75)
    {
      rsp = 75;
    }
    analogWrite(PWMA, lsp);
    analogWrite(PWMB, rsp);
  }
  // print_ir();
  Serial.println(pidVal);
  Serial.println(isPos);
  avg_speed = 150;
  Serial.println(countA);
  delay(20);
}

int position()
{
  isPos = true;
  if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 1))
  {
    pos = -4;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 1) and (digitalRead(IR5) == 1))
  {
    pos = -3;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 1) and (digitalRead(IR5) == 0))
  {
    pos = -2;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 1) and (digitalRead(IR4) == 1) and (digitalRead(IR5) == 0))
  {
    pos = -1;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 1) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0))
  {
    pos = 0;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 1) and (digitalRead(IR3) == 1) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0))
  {
    pos = 1;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 1) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0))
  {
    pos = 2;
  }
  else if ((digitalRead(IR1) == 1) and (digitalRead(IR2) == 1) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0))
  {
    pos = 3;
  }
  else if ((digitalRead(IR1) == 1) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0))
  {
    pos = 4;
  }
  else if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 0) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0))
  {
    isPos = false;
    if (pos < 0)
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      analogWrite(PWMA, 85);
      analogWrite(PWMB, 85);
      delay(200);
    }
    else if (pos > 0)
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      analogWrite(PWMA, 85);
      analogWrite(PWMB, 85);
      delay(200);
    }
  }
  else if (((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 1) and (digitalRead(IR4) == 1) and (digitalRead(IR5) == 1)) or ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR3) == 1) and (digitalRead(IR4) == 1) and (digitalRead(IR5) == 1)))
  {
    isPos = false;
    stop();
    delay(200);
    analogWrite(PWMA, 150);
    analogWrite(PWMB, 150);
    delay(225);
    stop();
    countA = 0;
    while(countA<6150){
        sharp_right(150);
    }
    brake_free();
    delay(200);
  }
    else if ((digitalRead(IR1) == 1) and (digitalRead(IR2) == 1) and (digitalRead(IR3) == 1 and (digitalRead(IR4) == 0) and (digitalRead(IR5) ==0 )))
  {
    isPos = false;
    stop();
    delay(200);
    analogWrite(PWMA, 150);
    analogWrite(PWMB, 150);
    delay(225);
    stop();
    if (digitalRead(IR3) == 1){
      analogWrite(PWMA, 150);
      analogWrite(PWMB, 150);
      delay(100);
      stop();
    }
    else if (digitalRead(IR3) == 0){
    countB = 0;
    while(countB<6150){
        sharp_left(150);
    }
    brake_free();
    delay(200);
    }
  }
  return pos;
}

float pidControl()
{
  if(isPos){
  error = pos * 4;
  pVal = kp * error;
  integral += error;
  // integral += error * dt;
  iVal = ki * integral;
  derivative = (error - prevError);
  // derivative = (error - prevError)*dt;
  dVal = kd * derivative;
  pidValue = pVal + iVal + dVal;
  }
  else{
    prevError = 0;
    integral = 0;
    derivative = 0;
    error = 0;
  }
  return pidValue;
}

// void forward()
// {
//   digitalWrite(AIN1, HIGH);
//   digitalWrite(AIN2, LOW);
//   digitalWrite(BIN1, HIGH);
//   digitalWrite(BIN2, LOW);
//   analogWrite(PWMA, 255);
//   analogWrite(PWMB, 255);
// }

// void reverse(int speed)
// {
//   digitalWrite(AIN1, LOW);
//   digitalWrite(AIN2, HIGH);
//   digitalWrite(BIN1, LOW);
//   digitalWrite(BIN2, HIGH);
//   analogWrite(PWMA, speed);
//   analogWrite(PWMB, speed);
// }

// void go(int speed)

// void sharp_right(int speed)
// {
//   digitalWrite(AIN1, HIGH);
//   digitalWrite(AIN2, LOW);
//   digitalWrite(BIN1, LOW);
//   digitalWrite(BIN2, HIGH);
//   analogWrite(PWMA, speed);
//   analogWrite(PWMB, speed);
// }

// void sharp_left(int speed)
// {
//   digitalWrite(AIN1, LOW);
//   digitalWrite(AIN2, HIGH);
//   digitalWrite(BIN1, HIGH);
//   digitalWrite(BIN2, LOW);
//   analogWrite(PWMA, speed);
//   analogWrite(PWMB, speed);
// }

// {
//   digitalWrite(AIN1, HIGH);
//   digitalWrite(AIN2, LOW);
//   digitalWrite(BIN1, HIGH);
//   digitalWrite(BIN2, LOW);
//   analogWrite(PWMA, speed);
//   analogWrite(PWMB, speed);
// }

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

// void signal_1()
// {
//   for (int i = 1; i <= 3; ++i)
//   {
//     digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
//     delay(100);                      // wait for a second
//     digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
//     delay(100);                      // wait for a second
//   }
// }

// void pwm_test()

// {
//   int8_t x = 3;
//   digitalWrite(AIN1, HIGH);
//   digitalWrite(AIN2, LOW);
//   digitalWrite(BIN1, HIGH);
//   digitalWrite(BIN2, LOW);
//   for (int i = 0; i <= 255; i++)
//   {
//     analogWrite(PWMA, i);
//     analogWrite(PWMB, i);
//     delay(x * 10);
//   }
//   for (int i = 255; i >= 0; i--)
//   {
//     analogWrite(PWMA, i);
//     analogWrite(PWMB, i);
//     delay(x * 10);
//   }
// }

void sharp_right(int spd) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, spd);
  analogWrite(PWMB,0);
}
void sharp_left(int spd) {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2,LOW);
  analogWrite(PWMB, spd);
  analogWrite(PWMA,0);
}

void brake_fast() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  delay(20);
}

void brake_free() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  delay(20);
}

