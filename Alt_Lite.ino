// 000 - 935 - Data
// 945 - 946 - Apogee
// 947 - 947 - NumRec
// 950 - 951 - Max Speed



#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#include <GyverTimer.h>
#include <Wire.h>
#include <EEPROM.h>


#define OLED_I2C_ADDRESS 0x3C
#define I2C_ADDRESS 0x3C
#define RST_PIN -1

SSD1306AsciiAvrI2c oled;
Adafruit_BMP085 bmp;

float SEALEVELPRESSURE_HPA;

long int TimeStamp;
int Cycles, Tick;
byte BUTTON = 2;
int Altitude;
long int Pressure;
int Temperature;
int Speed;
int Apogee;
int EEPOS;
int Maxspeed;
unsigned int tenths;
long int LogTime;
float Alt1, Alt2;
byte PackSize;
int EEXPos;

int msgCount;
boolean Fallen;
boolean latch;
byte NumRec;

long int Start, Start2, Finish, Finish2, routineTime;
long int FirstTime, SecondTime, oldAltitude, newAltitude, SecondTimeM, FirstTimeM;



struct telemetrystruct
{
  unsigned int tenths;
  int Altitude;
  int Speed;
};

struct telemetrystruct telemetry;

void setup()
{
  Fallen = false;
  Apogee = 0;
  EEXPos = 0;
  PackSize = sizeof (telemetry);
  NumRec = 0;
  Cycles = 150;
  Tick = 148;

  pinMode(BUTTON, INPUT_PULLUP); // BUTTON PIN

  Serial.begin(115200);

  Serial.println("Start");
  Serial.println("PackSize = " + String(PackSize));

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
  Altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

//------------------------------------------------------------------------------

void loop()
{

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

    LOGonOSD();
  }

  oled.clear();
  oled.set2X();
  oled.println("POEKHALI!");
  delay (1000);
  oled.clear();

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                                     FIRST STAGE                                               //
  ///////////////////////////////////////////////////////////////////////////////////////////////////


  for (int q = 945; q < 955; q++)
  {
    EEPROM.update(q, 0);
  }

  oled.set1X();

  LogTime = millis();

  for (int FSTage = 0; FSTage < Cycles; FSTage++)
  {

    //    while (Altitude < 3)
    //    {
    //      Altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    //      oled.clear();
    //LogTime = millis();
    //      oled.println(Altitude);
    //    }

    Start2 = millis();

    getdata();

    delay(Tick);
    Finish2 = millis();
    routineTime = Finish2 - Start2;

    oled.clear();
    oled.println(FSTage);

  }

  EEPROM.put(950, Maxspeed);
  oled.clear();

  while (1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void getdata()
{

  Pressure    = bmp.readPressure();
  Altitude    = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  Speed       = speedOmeter();
  TimeStamp = (millis() - LogTime);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


float speedOmeter()
{
  if (!Fallen) {

    if ((oldAltitude - newAltitude) > 2)
    {
      Apogee = oldAltitude;
      EEPROM.put(945, Apogee);
      Fallen = true;
    }
  }

  float FloatSpeed;
  Alt2  =  bmp.readAltitude(SEALEVELPRESSURE_HPA);

  FirstTimeM = millis();

  if (FirstTimeM - SecondTimeM > 100)
  {
    oldAltitude = newAltitude;
    newAltitude = Altitude;

    FloatSpeed = (Alt2 - Alt1) / (FirstTimeM - SecondTimeM) * 100000;
    Speed = FloatSpeed / 100;
    Alt1 = Alt2;


    if (Speed > Maxspeed) Maxspeed = Speed;

    telemetry.Altitude = Altitude;
    telemetry.Speed = Speed;
    Writelog();
    EEPROM.write(947, NumRec);
    SecondTimeM = millis();

  }
  return Speed;
}



void LOGonOSD()
{
  EEXPos = 0;
  EEPROM.get (945, Apogee);
  EEPROM.get (950, Maxspeed);
  EEPROM.get(947, NumRec);

  oled.setFont(Callibri15);
  oled.clear();
  oled.set1X();

  oled.print("NumRec: ");
  oled.print(NumRec);
  oled.println("recs");
  delay (2000);

  oled.clear();
  PackSize = sizeof (telemetry);
  byte Packet[PackSize];
  oled.println("Apogee = " + String (Apogee));
  oled.println("Maxspeed = " + String (Maxspeed));
  delay (2000);

  Serial.println("NumRec = " + String (NumRec));
  Serial.println("Apogee = " + String (Apogee));
  Serial.println("Maxspeed = " + String (Maxspeed));
  Serial.println(" ");
  Serial.println("Time \t Altitude \t Speed");

  for (int Rec = 0; Rec < NumRec; Rec++)
  {
    for (int intern = 0; intern < PackSize; intern++)
    {
      Packet[intern] = EEPROM.read(EEXPos + intern);
    }
    memcpy(&telemetry, Packet, sizeof(telemetry));

    Altitude      = telemetry.Altitude;
    Speed         = telemetry.Speed;
    TimeStamp     = telemetry.tenths;
    Serial.println(String (TimeStamp) + "\t" + String (Altitude) + "\t" + String (Speed));

    EEXPos = EEXPos + PackSize;
  }
  Serial.println(" ");
}


void Writelog()
{
  byte bufwrite;

  if (EEXPos < 901)
  {
    unsigned char * telemetry_bytes;

    telemetry_bytes = (unsigned char *) &telemetry;

    for (bufwrite = 0; bufwrite < PackSize; bufwrite++)
    {
      EEPROM.write(EEXPos + bufwrite, telemetry_bytes[bufwrite]);
    }
    EEXPos = EEXPos + PackSize;
    NumRec++;
    EEPROM.write(947, NumRec);
  }

}
