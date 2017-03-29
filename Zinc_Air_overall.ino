#include <PololuWheelEncoders.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

#include <Wire.h>
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  //tsl.setGain(TSL2591_GAIN_MAX);
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  //Serial.println("------------------------------------");
  //Serial.print  ("Gain:         ");
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      //Serial.println("1x (Low)");
      break;
    case TSL2591_GAIN_MED:
      //Serial.println("25x (Medium)");
      break;
    case TSL2591_GAIN_HIGH:
      //Serial.println("428x (High)");
      break;
    case TSL2591_GAIN_MAX:
      //Serial.println("9876x (Max)");
      break;
  }
  //Serial.print  ("Timing:       ");
  //Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  //Serial.println(" ms");
  //Serial.println("------------------------------------");
  //Serial.println("");

  
}

void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  static unsigned long t0=0; //the time that liquid was stabilized
  static unsigned long t1=0; //the time that the liquid went dark
  static unsigned long time1=0; //time it takes for the liquid to go from injected to dark
  static int f1=0; // flag to indicate phase, phase 0 is the default where nothing has happened. 
  ir = lum >> 16;
  full = lum & 0xFFFF;
  unsigned int a=tsl.calculateLux(full, ir);
  Serial.print(millis()); Serial.print("          "); Serial.print(a); Serial.print("             "); Serial.print(t0);  Serial.print("                                 "); Serial.print(t1); Serial.print("                              "); Serial.println(time1);
  if(a<40000 && millis()>=200 && f1==0)
  {
    f1=1;//liquid has been injected. 
  }
  if(f1==1 && a>40000)
  {
    f1=2;//liquid has been stabilized 
    t0=millis();
  }
  if(f1==2 && a<40000)
  {
    //liquid has turned dark
    t1=millis();
    time1=t1-t0;  
    if (time1>10000)
    {
      f1=3;
    }
  }
}
void setup() {
Serial.begin(115200);
  
  Serial.println("Starting Adafruit TSL2591 Test!");
  
  if (tsl.begin()) 
  {
    //Serial.println("Found a TSL2591 sensor");
  } 
  else 
  {
    //Serial.println("No sensor found ... check your wiring?");
    while (1);
  }
    
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Configure the sensor */
  configureSensor();
    
  // Now we're ready to get readings ... move on to loop()!

  Serial.println("Dual MC33926 Motor Shield");
  PololuWheelEncoders::init(3,5,6,11);
  pinMode(8,OUTPUT);
}


void loop() {
  // put your main code here, to run repeatedly:
uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  static unsigned long t0=0; //the time that liquid was stabilized
  static unsigned long t1=0; //the time that the liquid went dark
  static unsigned long time1=0; //time it takes for the liquid to go from injected to dark
  static int f1=0; // flag to indicate phase, phase 0 is the default where nothing has happened. 
  ir = lum >> 16;
  full = lum & 0xFFFF;
  unsigned int a=tsl.calculateLux(full, ir);
  Serial.print(millis()); Serial.print("          "); Serial.print(a); Serial.print("             "); Serial.print(t0);  Serial.print("                                 "); Serial.print(t1); Serial.print("                              "); Serial.println(time1);
  if(a<40000 && millis()>=200 && f1==0)
  {
    f1=1;//liquid has been injected. 
  }
  if(f1==1 && a>40000)
  {
    f1=2;//liquid has been stabilized 
    t0=millis();
  }
  if(f1==2 && a<40000)
  {
    //liquid has turned dark
    t1=millis();
    time1=t1-t0;  
    if (time1>10000)
    {
      f1=3;
    }
  }
  if (f1==3)
  {
    digitalWrite(8,LOW);
  }
  else
  {
    digitalWrite(8,HIGH);
  }
}
