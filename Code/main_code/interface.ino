#include <Keypad.h>
#include <String.h>
#include <LiquidCrystal.h>


const int KEY_ROWS = 4;
const int KEY_COLS = 3;

const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// initialize pin connections for LCD
const int RS; // #;
const int Enable; // #;
const int D4; // #;
const int D5; // #;
const int D6; // #;
const int D7; // #;

LiquidCrystal lcd(RS, Enable, D4, D5, D6, D7); // Create LiquidCrystal obj

char keys[KEY_ROWS][KEY_COLS] = 
{
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'},
};

// initialize pin connections
byte pin_rows[KEY_ROWS]; // = {#, #, #, #}; 
byte pin_cols[KEY_COLS]; // = {#, #, #}; 

Keypad keypad = Keypad( makeKeymap(keys), pin_rows, pin_cols, KEY_ROWS, KEY_COLS); // create key object from keypad.h

String guage = "";
String winding_id = "";
String windings = "";

String inputs [] = {guage, winding_id, windings};
int i = 0;

void formatLCD(String str);

void setup()
{
  Serial.begin(9600);
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.print("Initializing...");  // Prints from LCD
}

bool begin = true;

void loop()
{
  if (begin)
  {
    char key = keypad.getKey();
    if (key) 
    {
      switch (i)
      {
        case 0:
          formatLCD("Wire Gauge: ", key);
          break;
        case 1:
          formatLCD("Wire ID: ", key);
          break;
        case 2: 
          formatLCD("# Windings: ", key);
          break;
        case 3:
          Serial.println("Wire Gauge: " + inputs[0]);
          Serial.println("Wire ID: " + inputs[1]);
          Serial.println("Total Windingss: " + inputs[2]);
      }
    }
  }
  // Execute winding procedure otherwise
}

void formatLCD(String str, char key)
{
  lcd.print(str + inputs[i]);
  if ((key >= '0') && (key <= '9'))
    inputs[i] += key;
    lcd.print(str + inputs[i]);
  if ((key == '*') || (key == '#'))
    ++i;
}
