

#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <Wire.h>
#include "DualMC33926MotorShield.h"

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
DualMC33926MotorShield md;

#define Motor_Tick_per_rotation 3591.84
#define motor1_speed 319//adjust for speed of motor 1, out of 400
#define motor2_speed 307//adjust for speed of motor 2, out of 400
#define car_speed 0.39//speed of the car in m/s

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
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  tsl.setGain(TSL2591_GAIN_MAX);
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
  pinMode(8,OUTPUT);
}


void loop() {
  //static float total2;
  static float total1=0;
  static int flag1=0;
  static int flag2=0;
  static int flag3=0;
  static int flag4=0;
  static int flag5=0;
  static boolean car_moved =false;
  static float RunTime=0;
  static int speed1;
  static int speed2;
  static int co;
  long int car_start_time;
  static signed int adjust=1;//adjustment to the speed everytime the loop is run
  static int counter=0;//count the total times the loop has been run
  static int lux;
  static int preLux=0;
  static long int count=0;
  static long timer1=0;//Count until max is reached
  static long timer2=0;//Count until lux 30 is reached
  static long currentT;

  static float tagertRotate=1000;
  static int mode=0;
  static int t1=millis(); //t1 is the time it starts

  /***************************sensor******************/
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  //Serial.print("Visible: "); Serial.print(full - ir); Serial.print("  ");
  lux=tsl.calculateLux(full, ir);
  Serial.print("Lux: "); Serial.println(tsl.calculateLux(full, ir));
  Serial.print("m: "); Serial.print(tagertRotate*3.14159265359*.09004); Serial.print(" / "); Serial.print(tagertRotate*3.14159265359*.09004*1.01);  Serial.print("    ");Serial.println(RunTime);
  
  Serial.print("preLux: "); Serial.println(preLux);
  Serial.print("timer1: "); Serial.println(timer1);
  Serial.print("timer2: "); Serial.println(timer2);
  Serial.print("flag1: "); Serial.println(flag1);
    Serial.print("co: "); Serial.println(co);
    Serial.print("millis: "); Serial.println(millis());
  //Serial.print("total1: "); Serial.println(total1);
  
     /*******Clockreset********/
  if (preLux>(lux+5)) //this is used to determine max lux
  {
    co = co+1;
  }
  else 
  {
    co=0;
    }
  if ( preLux>lux && co>30 && (preLux-lux)>20 && flag1==0)
  {
    flag1=1;  
  }
  if ( preLux<lux && flag1 == 0)
  {
    preLux=lux; 
    timer1=millis(); 
  }

  if(flag1==1 && lux<30 &&flag3==0 && millis()>5000)//change the lux parameter
 {
  timer2=millis()-timer1;
  flag3=1;
 //tagertRotate=((testtime-998)/1464.8)/(0.09004*3.14159265359*1.01);//this equation will convert the time to the amount of rotation the car will go
                  //0.09 is the dimameter of the wheel
                  //3.14159265359*1.02 basically pi
                  //18874 is a made up coefficient 
  tagertRotate=((timer2+4130.841176)/1447.25941)/(0.09004*3.14159265359*1.01);//this is the formula used to determine the number of rotations
  RunTime=1000*tagertRotate*0.09004*3.14159265359*1.01/0.16;
 //unit of rotation                  
 } 

 if(analogRead(A3) < 500 && car_moved==false)
 {
         Serial.println("It has not run");
 }

 if(analogRead(A3) > 500 && !car_moved)
 {
         car_moved=!car_moved;
         car_start_time=millis();
         md.setM1Speed(motor1_speed);
         md.setM2Speed(motor2_speed);
         Serial.println("It started running");

 }

  if (flag3==1 && (millis()-car_start_time)>RunTime && car_moved)
  {
    md.setM1Speed(0);
    md.setM2Speed(0);
  }
  else
  {
    md.setM1Speed(motor1_speed);
    md.setM2Speed(motor2_speed);
  }
  
}
