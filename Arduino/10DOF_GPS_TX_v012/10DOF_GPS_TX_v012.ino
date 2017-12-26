
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file
#include "HMC5883L.h"
#include "BMP085.h"
#include "Wire.h"

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 9 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int YPR[3];

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// Magnetometer class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;
int16_t mx, my, mz;

// Barometer class default I2C address is 0x77
// specific I2C addresses may be passed as a parameter here
// (though the BMP085 supports only one address)
BMP085 barometer;

float temperature;
float pressure;
int32_t lastMicros;
static const float ACCEL_SENS = 16384.0; // Accel Sensitivity with default +/- 2g scale
static const float GYRO_SENS  = 131.0;   // Gyro Sensitivity with default +/- 250 deg/s scale

SoftwareSerial mySerial(4, 3);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00011";
//const uint64_t address = {0xF0F0F0F0E1LL};
//uint8_t address[] = { 0xCC,0xCE,0xCC,0xCE,0xCC };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
     radio.begin();
     radio.openWritingPipe(address);
     radio.setPALevel(RF24_PA_MAX);
     radio.stopListening();
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    delay(50);
    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(16);
    mpu.setYGyroOffset(-74);
    mpu.setZGyroOffset(-13);
    mpu.setZAccelOffset(929); // 1688 factory default for my test chip
    mpu.setXAccelOffset(-743); 
    mpu.setYAccelOffset(261); 
    mpu.setI2CBypassEnabled(true);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

      GPS.begin(9600);
      
      // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      // uncomment this line to turn on only the "minimum recommended" data
      //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
      // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
      // the parser doesn't care about other sentences at this time
      
      // Set the update rate
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
      // For the parsing code to work nicely and have time to sort thru the data, and
      // print it out we don't suggest using anything higher than 1 Hz
    
      // Request updates on antenna status, comment out to keep quiet
      GPS.sendCommand(PGCMD_ANTENNA);
    
      // the nice thing about this code is you can have a timer0 interrupt go off
      // every 1 millisecond, and read data from the GPS for you. that makes the
      // loop code a heck of a lot easier!
      useInterrupt(true);
    
      delay(1000);
      // Ask for firmware version
      mySerial.println(PMTK_Q_RELEASE);
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
        
    }
      // ==================== HMC5883L ============================

  mag.initialize();
  //Serial.print("Testing Mag...  ");
  //Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  // ==================== BMP085 ============================
  barometer.initialize();
  //Serial.print("Testing Pressure...  ");
  //Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
//uint32_t timer = millis();

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            YPR[0] = (int)(ypr[0] * 180/M_PI);
            YPR[1] = (int)(ypr[1] * 180/M_PI);
            YPR[2] = (int)(ypr[2] * 180/M_PI);
            
            //Serial.print("yprha\t");
            //Serial.print(YPR[0]);
           // Serial.print("\t");
            //Serial.print(YPR[1]);
            //Serial.print("\t");
            //Serial.print(YPR[2]);
            //Serial.print("\t");

     // read raw heading measurements
    mag.getHeading(&mx, &my, &mz);
        
    // To calculate heading in degrees. 0 degree indicates North
        float heading = atan2(my, mx);
    if(heading < 0) heading += 2 * M_PI;
    int int_heading = (int)(heading * 180/M_PI);
    //Serial.print(int_heading); Serial.print("\t");

        // request temperature
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    
    // wait appropriate time for conversion (4.5ms delay)
    lastMicros = micros();
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    // read calibrated temperature value in degrees Celsius
    temperature = barometer.getTemperatureC();

    // request pressure (3x oversampling mode, high detail, 23.5ms delay)
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    // read calibrated pressure value in Pascals (Pa)
    pressure = barometer.getPressure();
    long pressure_int = (long)pressure;
    //long SeaLevelPressure = 102200;
    //Serial.print(pressure); Serial.println("\t");
    //double altitude = 44330 * (1 - pow((pressure / SeaLevelPressure), (1/5.255)));
    //long altitdue_100 = altitude *100;

     if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
//    if (GPSECHO)
//      if (c) Serial.print(c);
  }
    // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  /*
    // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
  */
    int GPS_Fix = (int)GPS.fix;
    long GPS_la_10000= (long)(GPS.latitudeDegrees*10000);
    long GPS_lo_10000= (long)(GPS.longitudeDegrees*10000);
    float GPS_Sp_raw = GPS.speed;
    //int GPS_Sp = (int)GPS_Sp_raw;
    long GPS_Sp_100= (long)(GPS_Sp_raw*100);
    float GPS_al = GPS.altitude;
    
    
    char data_1[24];
    data_1[0] = 49;
    data_1[1] = 89;  //Y
    if(YPR[0] < 0){ data_1[2] = 45;} //-
    else{data_1[2] = 43;} //+
    if(abs(YPR[0]) < 100){ data_1[3] = 48;} //0
    else{data_1[3] = 49;} //1
    if(abs(YPR[0]) < 10){ data_1[4] = 48;} //0
    else{data_1[4] = (int)((abs(YPR[0])/10)%10)+48;}
    if(YPR[0] == 0){ data_1[5] = 48;} //0
    else{data_1[5] = (int)(abs(YPR[0])%10)+48;}
    data_1[6] = 80;  //P
    if(YPR[1] < 0){ data_1[7] = 45;} //-
    else{data_1[7] = 43;} //+
    if(abs(YPR[1]) < 10){ data_1[8] = 48;} //0
    else{data_1[8] = (int)((abs(YPR[1])/10)%10)+48;}
    if(YPR[1] == 0){ data_1[9] = 48;} //0
    else{data_1[9] = (int)(abs(YPR[1])%10)+48;}
    data_1[10] = 82;  //R    
    if(YPR[2] < 0){ data_1[11] = 45;} //-
    else{data_1[11] = 43;} //+
    if(abs(YPR[2]) < 10){ data_1[12] = 48;} //0
    else{data_1[12] = (int)((abs(YPR[2])/10)%10)+48;}
    if(YPR[2] == 0){ data_1[13] = 48;} //0
    else{data_1[13] = (int)(abs(YPR[2])%10)+48;}
    data_1[14] = 72;  //H
    data_1[15]=(int_heading/100)%10+48;
    data_1[16]=(int_heading/10)%10+48;
    data_1[17]=int_heading%10+48;
    data_1[18] = 80;  //P
    data_1[19]=(pressure_int/100000)%10+48;
    data_1[20]=(pressure_int/10000)%10+48;
    data_1[21]=(pressure_int/1000)%10+48;
    data_1[22]=(pressure_int/100)%10+48;
    data_1[23]=(pressure_int/10)%10+48;
    data_1[24]= pressure_int%10+48;
    data_1[25]=0;
    
    char data_2[27];
    data_2[0]=50;
    data_2[1]= 71; //G
    data_2[2] = GPS_Fix + 48;
    data_2[3]= 108; //l
    data_2[4]= 97; //a
    data_2[5]=(GPS_la_10000/1000000)%10+48;
    data_2[6]=(GPS_la_10000/100000)%10+48;
    data_2[7]=(GPS_la_10000/10000)%10+48;
    data_2[8]=(GPS_la_10000/1000)%10+48;
    data_2[9]=(GPS_la_10000/100)%10+48;
    data_2[10]=(GPS_la_10000/10)%10+48;
    data_2[11]=GPS_la_10000%10+48;    
    data_2[12]= 108; //l
    data_2[13]= 111; //o
    data_2[14]=(GPS_lo_10000/1000000)%10+48;
    data_2[15]=(GPS_lo_10000/100000)%10+48;
    data_2[16]=(GPS_lo_10000/10000)%10+48;
    data_2[17]=(GPS_lo_10000/1000)%10+48;
    data_2[18]=(GPS_lo_10000/100)%10+48;
    data_2[19]=(GPS_lo_10000/10)%10+48;
    data_2[20]=GPS_lo_10000%10+48;
    data_2[21] = 83; //S
    data_2[22]=(GPS_Sp_100/10000)%10+48;
    data_2[23]=(GPS_Sp_100/1000)%10+48;
    data_2[24]=(GPS_Sp_100/100)%10+48;
    data_2[25]=(GPS_Sp_100/10)%10+48;
    data_2[26]=GPS_Sp_100%10+48;    
    data_2[27] = 0;     

        //Serial.println(data_1);
        radio.write(&data_1, sizeof(data_1));
       //Serial.println(data_2);
        radio.write(&data_2, sizeof(data_2));
        
        
        // blink LED to indicate activity0
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    } 
}
