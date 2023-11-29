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

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int Speed = 255;
int correction_count;
int avg_speed = 200;
bool Lost = false;

float Kp = 0;
float Kd = 0;
float Ki = 0;

// function prototypes
void line_follow();
void print_ir();
void signal_1();
void pwm_test();
int position();
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
}

void loop()
{
  signal_1();
  print_ir();
 // pwm_test();
  if (not(Lost)){
    line_follow();
  }
  else{
    stop();

  }
}

void line_follow(){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

  if (digitalRead(IR1) == 1 && digitalRead(IR5) == 0 )
  {
    Serial.println("to left");
    analogWrite(PWMA, 0);
    analogWrite(PWMB, Speed);
    correction_count = 0;
  }

  else if (digitalRead(IR5) == 1 && digitalRead(IR1) == 0)
  { Serial.println("to right");
    analogWrite(PWMA, Speed);
    analogWrite(PWMB, 0);
    correction_count = 0;
  }
  else if (digitalRead(IR3) == 0)
  { if ((digitalRead(IR1) == 0) and (digitalRead(IR2) == 0) and (digitalRead(IR4) == 0) and (digitalRead(IR5) == 0)){
    Serial.println("Missed");
    reverse(100);
    delay(500);
    stop();
    correction_count += 1;
    if (correction_count >= 5){
      Lost = true;
    }else{
      Lost = false;
    }
  }
  else{
    Serial.println("Away");
    Kp = .06;
    Kd = 6 * Kp;
    Ki = 0;
    error = digitalRead(IR2) - digitalRead(IR4);
    P = error;
    I = I + error;
    D = error - previousError;
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
    previousError = error;
    lsp = avg_speed - PIDvalue;
    rsp = avg_speed +  PIDvalue;
    if (lsp > 255)
    {
      lsp = 255;
    }
    if (lsp < 0)
    {
      lsp = 0;
    }
    if (rsp > 255)
    {
      rsp = 255;
    }
    if (rsp < 0)
    {
      rsp = 0;
    }
    analogWrite(PWMA, lsp);
    analogWrite(PWMB, rsp);
    correction_count = 0;
  }
    }
  else{
    correction_count = 0;
     if ((digitalRead(IR1) == 1) and (digitalRead(IR2) == 1) and (digitalRead(IR4) == 1) and (digitalRead(IR5) == 1)){
    //     analogWrite(PWMA, 0);
    //   analogWrite(PWMB, 0);
    //   delay(500);
    //   analogWrite(PWMA, 150);
    //  analogWrite(PWMB, 150);
    //  delay(500);
    stop();
    }
    else{
      analogWrite(PWMA,avg_speed);
      analogWrite(PWMB, avg_speed);
  }
}
}


// void linefollow()
// {
//   int error = (digitalRead(IR2) - digitalRead(IR4));

//   P = error;
//   I = I + error;
//   D = error - previousError;

//   PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
//   previousError = error;

//   lsp = lfspeed - PIDvalue;
//   rsp = lfspeed + PIDvalue;

//   if (lsp > 255)
//   {
//     lsp = 255;
//   }
//   if (lsp < 0)
//   {
//     lsp = 0;
//   }
//   if (rsp > 255)
//   {
//     rsp = 255;
//   }
//   if (rsp < 0)
//   {
//     rsp = 0;
//   }
//   // motor1.drive(lsp);
//   // motor2.drive(rsp);
// }


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
// void right()
//  {
//   digitalWrite(mr1,LOW);
//   digitalWrite(mr2,HIGH);
//    digitalWrite(ml1,HIGH);
//   digitalWrite(ml2,LOW);
//   analogWrite (enr,tspeed);
//    analogWrite (enl,tspeed);
//   delay(tdelay);
//  }

// void left()
//  {
//    digitalWrite(mr1,HIGH);
//   digitalWrite(mr2,LOW);
//   digitalWrite(ml1,LOW);
//    digitalWrite(ml2,HIGH);
//   analogWrite (enr,tspeed);
//   analogWrite (enl,tspeed);
//    delay(tdelay);
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