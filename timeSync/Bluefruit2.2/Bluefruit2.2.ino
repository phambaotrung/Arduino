#include <Adafruit_BluefruitLE_UART.h>

#include "BluefruitConfig.h"

#include <Adafruit_NeoPixel.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
         
                                Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    PIN                       Which pin on the Arduino is connected to the NeoPixels?
    NUMPIXELS                 How many NeoPixels are attached to the Arduino?
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

#define PIN                         8
#define NUMPIXELS                   1
#define DEVICE_NAME        "AT+GAPDEVNAME=Bluefruit2"
/*=========================================================================*/

/* Assign a unique base ID for this sensor */

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, PIN);

Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

// variable
unsigned long long CFlora;

unsigned long long epoch_time;
unsigned long long start_ms;
unsigned long long current_ms;

unsigned long long tau;
unsigned long long delta;
long offset;
long diff;
unsigned long prev_time, curr_time;

bool sync_done = false;
bool first_sync = true, first_data, start_session = true;

unsigned long long t1, t2, t3, t4, t3_minus_t2, cflora;
int iteration = 0;
int beforeX, beforeY, beforeZ;
int afterX, afterY, afterZ;

bool isFirstStage = true;
bool isSecondStage = false;
bool lastCue = false;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);


  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);


  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);

}

void setup() {
  // For testing purpose
  //Serial.println(millis());
  while (!Serial);  // required for Flora & Micro
  delay(500);
  //Serial.println(millis());

  initPixel();

  Serial.begin(115200);

  initSensor();
  initBluefruitLE();
}

void loop() {
  if (isFirstStage) {
    // waiting for command to send data to pc
    // Check for incoming characters from Bluefruit
    //ble.println("AT+BLEUARTRX");
    ble.readline();
    if (strcmp(ble.buffer, "OK") == 0) {
      // no data
      return;
    }

    switch (ble.buffer[0]) {
      case 'S': // first time
        // get sensor data and send to pc
        epoch_time = 0;
        start_ms = prev_time = millis();
        
        isFirstStage = false;
        for (int i = 1, j = 0; i < strlen(ble.buffer); i++, j += 2)
        {
          if ((ble.buffer[i] >= '0') && (ble.buffer[i] <= '9'))
          {
            epoch_time = (epoch_time * 10) + (ble.buffer[i] - '0');
            //sprintlnULLD(epoch_time);
          }
          delay(50);
        }
        //Serial.print(F("[Recv] "));

        //printCurrentTime();
        Serial.print("got epoch: "); sprintlnULLD(epoch_time);
        changePixelColor(0x07, 0x07, 0); // first stage (yellow)
        break;
      default:
        // not the data we want; thus, do nothing
        return;
    }

    ble.waitForOK();

    delay(250);
  } else {
    sync_done = false;

    prev_time = millis();
    curr_time = millis();

    // wasting time
    while ((curr_time - prev_time) <= 700) {
      syncSession();

      curr_time = millis();
    }
  }
}

void syncSession(void) {
  if (sync_done) {
    //Serial.println(millis()-prev_time);
    return;
  }

  if (start_session) {
    char buffer[100];
    if (!lastCue) {
      sensorCue();
    }
    t1 = getCurrentTime();
    delay(15);

    //ble.print("AT+BLEUARTTX=");

    if (!isSecondStage){
    
    ble.print('$');
    sprintf(buffer, "%0ld", t1 / 1000000L);
    
    ble.print(buffer);
    sprintf(buffer, "%0ld", t1 % 1000000L);
    ble.println(buffer);
    ble.print('#');
    Serial.print("Sent t1: "); sprintlnULLD(t1);

    //while (!ble.waitForOK());
    } else {
       ble.print('$');
    ble.print(offset);
    ble.print('#'); 
    Serial.print("Sending offset = ");Serial.println(offset);
      
      }
    t1 = getCurrentTime();

    delay(10);

    changePixelColor(0x07, 0, 0x07); // first phase of sync session (purple)

    start_session = false;
  } else {
    changePixelColor(0x05, 0x06, 0x07); // start of second phase (purple)

    //ble.println("AT+BLEUARTRX");
    ble.readline();
    if (strcmp(ble.buffer, "OK")==0) {
      // no data or wrong packet
      //Serial.println("checkpoint -1");
      return;
    }

    if (!isSecondStage){
      lastCue = false;
    //Serial.println("checkpoint 0");
    Serial.println(ble.buffer);
    
    getResponseStage1();

    }else{
      
      getResponseStage2();
      }

  }
}

void sprintULLD (unsigned long long ts) {
  sprintULLD(ts, false);
}

void sprintlnULLD (unsigned long long ts) {
  sprintULLD(ts, true);
}

void sprintULLD(unsigned long long ts, bool newline) {
  char buffer[100];

  sprintf(buffer, "%0ld", ts / 1000000L);
  Serial.print(buffer);
  sprintf(buffer, "%0ld", ts % 1000000L);
  Serial.print(buffer);

  if (newline) Serial.println(" ");
  else Serial.print(" ");
}

void sendAccel(sensors_event_t accel) {
  ble.print("AT+BLEUARTTX=");

  ble.print("A"); ble.print(accel.acceleration.x);
  ble.print("|"); ble.print(accel.acceleration.y);
  ble.print("|"); ble.println(accel.acceleration.z);

  // check response stastus
  if (! ble.waitForOK() ) {
    Serial.println(F("Failed to send?"));
    sendAccel(accel);
  }
}

void sendMag(sensors_event_t mag) {
  ble.print("AT+BLEUARTTX=");

  // print out magnetometer data
  ble.print("M"); ble.print(mag.magnetic.x);
  ble.print("|"); ble.print(mag.magnetic.y);
  ble.print("|"); ble.println(mag.magnetic.z);

  // check response stastus
  if (! ble.waitForOK() ) {
    Serial.println(F("Failed to send?"));
    sendMag(mag);
  }
}

void sendGyro(sensors_event_t gyro) {
  ble.print("AT+BLEUARTTX=");

  // print out gyroscopic data
  ble.print("G"); ble.print(gyro.gyro.x);
  ble.print("|"); ble.print(gyro.gyro.y);
  ble.print("|"); ble.println(gyro.gyro.z);

  // check response stastus
  if (! ble.waitForOK() ) {
    Serial.println(F("Failed to send?"));
    sendGyro(gyro);
  }
}

unsigned long long getCurrentTime(void) {
  unsigned long long t = 0;
  current_ms = millis();

  t = epoch_time + (current_ms - start_ms);

  return t;
}

void printCurrentTime(void) {
  char buffer[100];
  unsigned long long et = getCurrentTime();

  sprintf(buffer, "%0ld", et / 1000000L);
  Serial.print(buffer);
  sprintf(buffer, "%0ld", et % 1000000L);
  Serial.println(buffer);
}

void initPixel(void) {
  // turn off neopixel
  pixel.begin(); // This initializes the NeoPixel library.
  changePixelColor(0, 0, 0); // off
}

void initBluefruitLE() {
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      changePixelColor(0x10,0,0); // couldn't factory reset (red)
      error(F("Couldn't factory reset"));
    }

    
  }

    Serial.println("Setting up name");
  ble.println(DEVICE_NAME);


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  //Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  //ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!
  
  /* Wait for connection */
  while (! ble.isConnected()) {
    changePixelColor(0,0,0x10); // waiting (blue)
    delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));

    changePixelColor(0,0x10,0); // connected (green)
  } else {
    changePixelColor(0x07,0x07,0); // not supported (red)
  }
  ble.setMode(BLUEFRUIT_MODE_DATA);
}

void initSensor() {
  /* Initialise the sensor */
  if (!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while (1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));

  /* Display some basic information on this sensor */
  //displaySensorDetails();

  /* Setup the sensor gain and integration time */
  configureSensor();
}

void changePixelColor(uint8_t r, uint8_t g, uint8_t b) {
  for (uint8_t i = 0; i < NUMPIXELS; i++) {
    pixel.setPixelColor(i, pixel.Color(r, g, b));
  }
  pixel.show();
}

void sensorCue() {
  lastCue = true;

  lsm.read();
  beforeX = lsm.accelData.x;
  afterX = lsm.accelData.x;
  beforeY = lsm.accelData.y;
  afterY = lsm.accelData.y;
  beforeZ = lsm.accelData.z;
  afterZ = lsm.accelData.z;
  while (true) {
    //Serial.println("wating for cue");
    lsm.read();
    afterX = lsm.accelData.x;
    afterY = lsm.accelData.y;
    afterZ = lsm.accelData.z;
    //Serial.println(after);
    if (abs(afterX - beforeX) > 1500 || abs(afterY - beforeY) > 1500 || abs(afterZ - beforeZ) > 1500) {
      break;
    }
    beforeX = afterX;
    beforeY = afterY;
    beforeZ = afterZ;

  }



}

void getResponseStage1() {
  //Serial.println("Checkpoint 1");
  switch (ble.buffer[0]) {
    case 'R': // get response from pc
      
      delay(15);

      diff = 0;

      for (int i = 1, j = 0; i < strlen(ble.buffer); i++, j += 2)
      {
        if ((ble.buffer[i] >= '0') && (ble.buffer[i] <= '9'))
        {
          diff = (diff * 10) + (ble.buffer[i] - '0');
        }
        delay(50);
      }
      //Serial.println("Checkpoint 2");

      changePixelColor(0, 0x07, 0x07); // last phase (white purple-ist)
      //Serial.print("Diff is: "); sprintlnULLD(diff);



      //delay(10);


      epoch_time = epoch_time + diff;



      start_session = true;
      sync_done = true;

      //delay(175);
      break;
   case 'C': // first time
        
        isFirstStage = false;
        isSecondStage = true;
        start_session = true;
        sync_done = true;
        
        
        break;    
    default:
      // not the data we want; thus, do nothing
      return;
  }


}

void getResponseStage2() {
  //Serial.println("in stage 2");
  switch (ble.buffer[0]) {
    case 'R': // get response from pc
      t4 = getCurrentTime();
      delay(15);

      first_data = true;
      t3 = t3_minus_t2 = 0;

      for (int i = 1, j = 0; i < strlen(ble.buffer); i++, j += 2)
      {
        if ((ble.buffer[i] >= '0') && (ble.buffer[i] <= '9'))
        {
          if (first_data) {
            t3 = (t3 * 10) + (ble.buffer[i] - '0');
          } else {
            t3_minus_t2 = (t3_minus_t2 * 10) + (ble.buffer[i] - '0');
          }
        }
        else
        {
          first_data = false;
        }
        delay(10);
      }

      t2 = t3 - t3_minus_t2;


      changePixelColor(0, 0x07, 0x07); // last phase (white purple-ist)

      tau    = ((t4 - t1) - t3_minus_t2) / 2;
      cflora = t3 + tau;
      offset = t4 - cflora;

      //delay(10);

      sprintULLD(tau);
      Serial.print("\t");
      Serial.println(offset);
      //Serial.print(" \t");


      if (t4 > (cflora + 7)) { // is positive
        epoch_time = epoch_time - abs(offset * 0.25); //0.705
      } else if (t4 < (cflora - 7)) { // is negative
        epoch_time = epoch_time + abs(offset * 0.25); //0.805
      } else if (t4 > (cflora + 1)) { // is positive
        epoch_time = epoch_time - 3; //0.705
      } else if (t4 < (cflora - 1)) { // is negative
        epoch_time = epoch_time + 3; //0.805
      }




      start_session = true;
      sync_done = true;

      //delay(175);
      break;
    default:
      // not the data we want; thus, do nothing
      return;
  }


}


