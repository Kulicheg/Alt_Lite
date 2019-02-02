// 950 - 951 - Max Speed
// 947 - 947 - NumRec
// 945 - 946 - Apogee

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

#include <Wire.h>
#include <EEPROM.h>




#define OLED_I2C_ADDRESS 0x3C
#define I2C_ADDRESS 0x3C
#define RST_PIN -1


SSD1306AsciiAvrI2c oled;
Adafruit_BMP085 bmp;


float SEALEVELPRESSURE_HPA;

int Altitude;
long int Pressure;
int Temperature;
int Speed;
int Apogee;
int EEPOS;
int Maxspeed;

float Alt1, Alt2;


int msgCount;
boolean Fallen;
byte NumRec;

long int Start, Start2, Finish, Finish2, routineTime;
long int FirstTime, SecondTime, oldAltitude, newAltitude, SecondTimeM, FirstTimeM;


struct SystemLog
{
  unsigned long timestamp;
  char message[20];
};

struct SystemLog capitansLog;


byte BUTTON = 4;


void setup()
{

  Fallen = false;
  Apogee = 0;


  pinMode(BUTTON, INPUT_PULLUP); // BUTTON PIN


  Serial.begin(115200);


  Wire.begin();
  oled.begin(&Adafruit128x32, OLED_I2C_ADDRESS);

  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.set2X();
  oled.println("Alt_Lite ");
  delay(1000);


  if (!bmp.begin()) {
    oled.clear();
    oled.set1X();
    oled.println("BMP280 NOT FOUND!");
    Serial.println("BMP280 NOT FOUND!");
    while (1) {}
  }

  oled.clear();
  oled.setFont(Callibri15);

  SEALEVELPRESSURE_HPA = bmp.readPressure();
}

//------------------------------------------------------------------------------

void loop()
{

  if (!digitalRead(BUTTON))
  {
    LOGonOSD();
  }


  oled.clear();
  oled.set1X();
  oled.print("Alt = ");
  oled.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  delay (2000);
  oled.print("Prs = ");
  oled.print(bmp.readPressure() / 133.322, 1);
  oled.println(" mm");
  delay (2000);

  oled.clear();
  oled.set1X();
  oled.println("Waiting orders...");

  while (digitalRead(BUTTON))
  {
  }


  oled.clear();
  oled.set2X();
  oled.println("POEKHALI!");
  delay (1000);
  oled.clear();
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                                     FIRST STAGE                                               //
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  toLog ("Start Logging");

  for (int q = 945; q < 952; q++)
  {
    EEPROM.write(q, 0);
  }

  for (int FSTage = 1; FSTage <= 100; FSTage++)
  {
    Start2 = millis();

    getdata();        // Получаем данные с датчиков в структуру
    fallingSense ();  // Не падаем ли?

    // delay(5);
    Finish2 = millis();
    routineTime = Finish2 - Start2;

    oled.clear();
    oled.set1X();
    oled.println(Altitude);
    oled.println(Speed);



  }

  toLog ("Finish Logging");

  EEPROM.put(950, Maxspeed);
  toLog ("Max Speed=" + String (Maxspeed));

  oled.clear();
  while (1);
}
///////////////////////////////////////////////////////////////////////////////////////////////////









void getdata()
{

  Pressure = bmp.readPressure();

  Altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  Temperature = bmp.readTemperature();

  Speed       = speedOmeter();




  ///////////////////////////////////////////////////////////////////////////////////////////////////
}



void fallingSense ()
{

  if (!Fallen) {

    if ((oldAltitude - newAltitude) > 2)
    {
      Apogee = oldAltitude;
      toLog ("Falling detected " + String(Apogee));
      EEPROM.put(945, Apogee);
      Fallen = true;
    }



    FirstTime = millis();
    if (FirstTime - SecondTime >= 1000)
    {


      SecondTime = millis();
      oldAltitude = newAltitude;
      newAltitude = Altitude;


    }


  }

}

void toLog (String message)
{

  if (EEPOS < 936)
  {
    int eventSize = sizeof (capitansLog);
    char event [20];
    message.toCharArray (event, 20);
    memcpy(capitansLog.message, event, 20);
    capitansLog.timestamp = millis();
    EEPROM.put(EEPOS, capitansLog);
    EEPOS = EEPOS + eventSize;
    NumRec = EEPOS / eventSize;
    EEPROM.write(947, NumRec);

  }
}


float speedOmeter()
{

  float FloatSpeed;
  Alt2  =  bmp.readAltitude(SEALEVELPRESSURE_HPA);

  FirstTimeM = millis();

  if (FirstTimeM - SecondTimeM >= 500)
  {

    FloatSpeed = (Alt2 - Alt1) / (FirstTimeM - SecondTimeM) * 100000;

    Speed = FloatSpeed / 100;
    Alt1 = Alt2;
    SecondTimeM = millis();

    if (Speed > Maxspeed) {
      Maxspeed = Speed;
    }
  }
  return Speed;

}



void LOGonOSD()
{
  EEPROM.get (945, Apogee);
  oled.setFont(Callibri15);
  oled.clear();
  oled.set1X();
  oled.print("Capitan's Log: ");
  oled.print(EEPROM.read(947));
  oled.println(" rec");
  oled.print("Apogee = ");
  oled.println(Apogee);
  delay (5000);

  int eventSize = sizeof (capitansLog);
  NumRec = EEPROM.read(947);
  oled.clear();
  oled.set1X();
  for (int msgCount = 0; msgCount < (eventSize * NumRec);  msgCount = msgCount + eventSize)
  {
    EEPROM.get(msgCount, capitansLog);
    String str(capitansLog.message);
    oled.println(capitansLog.timestamp / 1000);
    oled.println(str);
    delay(5000);
    oled.clear();
  }

  delay(10300);



}
