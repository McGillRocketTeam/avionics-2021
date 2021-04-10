#include <LiquidCrystal.h>

const int rs = 4, en = 5, d4 = 6, d5 = 7, d6 = 8, d7 =9;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int count = 0;

void setup() {
  Serial.begin(38400);
  lcd.clear();
  lcd.setCursor(0,0);
}


void loop() {
  for (int i=0; i<10; i++)
  {
    Serial.println("hello world!");
    delay(200);
  }
  
  Serial.println("start");
  char rxBuf[5];
  for (int i = 0; i < 10; i++)
  {
    Serial.println("0"); // need to send zero
    if (Serial.available() > 0)
    {
//      int message = Serial.parseInt();
//      lcd.clear();
//      lcd.setCursor(0,0);
//      lcd.print(message);

      for (int i = 0; i < 5; i++)
      {
        rxBuf[i] = Serial.read();
      }
      lcd.clear();
      lcd.print((String) rxBuf);
    }
    delay(1000);
    
  }

  Serial.println("1");
  while(1);

  
}

//  if (count > 9)
//    count = 0;
//  Serial.println(count);
//  delay(2500);
//  Serial.println("this is 10");
//  delay(500);
//  count += 1;
