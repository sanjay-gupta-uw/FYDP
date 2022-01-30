#include <LiquidCrystal_I2C.h>
#include <Servo.h>;

//buzzer pin
int buzzerPin = 7;

//servo
Servo myServo;
int servoPin = 9;
int servoPos = 90;
int servoAdd = 1;

//hall pin
volatile int hallPin = 2;
volatile int hallState;

// "ok" push button
int okButton = 10;
int okButtonState;

//potentiometer pin
int potPin = A0;
int potState;

//motor relay
int motorPin = 3;

int totalWinding;
volatile int windCount = 0;
int mappedPotState;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  Serial.begin(9600);
  //the usual stuff
  pinMode(okButton, INPUT);
  pinMode(hallPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  myServo.attach(servoPin);

  okButtonState = digitalRead(okButton);

  //lcd related stuff
  lcd.init();
  lcd.backlight();

  introFunction();
  promtFunction();
  promptFunction();

  lcd.clear();
  windCalc();
  lcd.clear();
  countDown();
  lcd.clear();
  attachInterrupt(digitalPinToInterrupt(hallPin), windCounter, FALLING);

  motorSpinFunction();
}

void loop()
{
  hallState = digitalRead(hallPin);
  Serial.println(hallState);
  //lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Windings: ");
  lcd.print(windCount);

  if (servoPos == 135 || servoPos == 40)
  {
    servoAdd = -servoAdd;
  }
  servoPos = servoPos + servoAdd;
  myServo.write(servoPos);

  while (windCount >= totalWinding)
  {
    //digitalWrite (motorPin,0);
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
    delay(500);
    switchOnFunction();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Winding Complete");
    lcd.setCursor(0, 1);
    lcd.print("Reset --> restart");
    delay(1000);
    //myServo.write (0);
  }
}

void promtFunction()
{
  lcd.setCursor(0, 0);
  lcd.print("Enter Weight: ");
  lcd.setCursor(0, 1);
  lcd.print("Ok to confirm");
  delay(1000);
}

void introFunction()
{
  lcd.setCursor(0, 0);
  lcd.print("Coil Winder");
  lcd.setCursor(0, 1);
  lcd.print("A Prabal Product");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  Developed by  ");
  lcd.setCursor(0, 1);
  lcd.print("   Team Prabal   ");
  delay(2000);
  lcd.clear();
}

int promptFunction()
{
  while (okButtonState == HIGH)
  {
    potState = analogRead(potPin);
    mappedPotState = map(potState, 0, 1023, 0, 50);
    lcd.setCursor(14, 0);
    lcd.print(mappedPotState);
    delay(500);
    if (digitalRead(okButton) == LOW)
    {
      break;
    }
  }
}

int windCalc()
{
  lcd.setCursor(0, 0);
  lcd.print("Chosen wght=");
  lcd.print(mappedPotState);
  lcd.print("g");
  totalWinding = mappedPotState * 46.666666666;
  lcd.setCursor(0, 1);
  lcd.print("Windings:");
  lcd.print(totalWinding);
  delay(3000);
}

void countDown()
{

  lcd.print("Starting in ");
  lcd.setCursor(0, 0);
  lcd.setCursor(0, 1);
  lcd.print("Reset -> Cancel");
  for (int i = 9; i >= 0; i--)
  {
    lcd.setCursor(13, 0);
    lcd.print(i);
    delay(1000);
  }
}

volatile int windCounter()
{
  hallState = digitalRead(hallPin);
  if (hallState == LOW)
  {
    windCount = windCount + 1;
    detachInterrupt(hallPin);
  }
}

void switchOnFunction()
{
  myservo.write(150);
  delay(1000);
  myservo.write(90);
}
