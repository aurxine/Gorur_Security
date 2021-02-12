#include <Arduino.h>

#define Trig_Pin 11 // Trigger Pin of Ultrasonic Sensor
#define Echo_Pin 10 // Echo Pin of Ultrasonic Sensor
#define Vcc 8
#define Alarm_Pin 9 //Connected to relay which will ring an Alarm
#define Key_Lock_Pin 2 
#define Indicator_LED_Pin 5


float Fixed_Distance = 13.00; // distance from sonar ssensor to wood mounting
float Allowable_Range = 2.00; //distance change in this range is allowed

bool unclocked = false;
bool Alarm = false;

void Unlock()
{
  Serial.println("key change");
  if(digitalRead(Key_Lock_Pin) == 0)
  {
    unclocked = true;
    Alarm = false;  
  }
  else
  unclocked = false;
}


long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

float Distance()
{
  long duration, dist;
  pinMode(Trig_Pin, OUTPUT);
  digitalWrite(Trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, LOW);
  pinMode(Echo_Pin, INPUT);
  duration = pulseIn(Echo_Pin, HIGH);
  dist = microsecondsToCentimeters(duration);
  delay(100);
  return dist;
}

void LED_Lightup(int times, int Delay)
{
  if(Delay == 0)
  {
    int power = times;
    analogWrite(Indicator_LED_Pin, power);
  }

  else if(Delay != 0 && times == 0)
  {
    digitalWrite(Indicator_LED_Pin, LOW);
  }
  else
  {
    for(int i = 0; i < times; i++)
    {
      digitalWrite(Indicator_LED_Pin, HIGH);
      delay(Delay);
      digitalWrite(Indicator_LED_Pin, LOW);
      delay(Delay);
    }
    
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(Trig_Pin, OUTPUT);
  pinMode(Echo_Pin, INPUT);
  pinMode(Vcc, OUTPUT);
  pinMode(Alarm_Pin, OUTPUT);
  pinMode(Indicator_LED_Pin, OUTPUT);
  pinMode(Key_Lock_Pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Key_Lock_Pin), Unlock, CHANGE);

  digitalWrite(Vcc, HIGH);
}

void loop() {
  float dist = Distance();
  if(unclocked)
  {
    float difference = dist - Fixed_Distance;
    int led_power = abs(difference) * 50;

    if(led_power > 1023)
    led_power = 1023;
    else if(led_power < 0)
    led_power = 0;

    LED_Lightup(led_power, 0);
    Serial.println(led_power);
  }

  else
  {
    if(abs(dist - Fixed_Distance) - Allowable_Range > 0)
    {
      Serial.println(abs(dist - Fixed_Distance) - Allowable_Range);
      
      Alarm = true;
      
    }
  }

  if(Alarm)
  {
    Serial.println("Alarm!");
    digitalWrite(Alarm_Pin, HIGH);
    // LED_Lightup(50, 200);
  }
  else
  {
    digitalWrite(Alarm_Pin, LOW);
  }
  
  // Serial.print(dist);
  // Serial.print("cm");
  // Serial.println();
  // delay(1000);
  
}