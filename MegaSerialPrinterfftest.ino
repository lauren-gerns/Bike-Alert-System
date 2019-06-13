
/*
  BIKE ALERT SYSTEM
  -- some code taken from Radiohead library example, Scanse Sweep library example,
  and the Big Easy Motor Driver
*/

#include <BlynkSimpleSerialBLE.h>
#include <ScanPacket.h>
#include <Sweep.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <SoftwareSerial.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "53e919aa3eb8408bb9fb1037306140d4"; //authorization code for blynk app

SoftwareSerial SerialBLE(10, 11); // RX, TX = 10
WidgetLED led2(V2);     //leds for blynk app
WidgetLED led3(V3);
WidgetLED led4(V4);

//BlynkTimer timer;
bool ledStatus2 = false;
bool ledStatus3 = false;
bool ledStatus4 = false;



// Create a Sweep device using Serial #1 (RX1 & TX1)
Sweep device(Serial1);

// keeps track of how many scans have been collected
uint8_t scanCount = 0;
// keeps track of how many samples have been collected
uint16_t sampleCount = 0;

// Arrays to store attributes of collected scans
bool syncValues[500];         // 1 -> first reading of new scan, 0 otherwise
float angles[500];            // in degrees (accurate to the millidegree)
uint16_t distances[500];      // in cm
uint8_t signalStrengths[200]; // 0:255, higher is better

// Finite States for the program sequence
const uint8_t STATE_WAIT_FOR_USER_INPUT = 0;
const uint8_t STATE_ADJUST_DEVICE_SETTINGS = 1;
const uint8_t STATE_VERIFY_CURRENT_DEVICE_SETTINGS = 2;
const uint8_t STATE_BEGIN_DATA_ACQUISITION = 3;
const uint8_t STATE_GATHER_DATA = 4;
const uint8_t STATE_STOP_DATA_ACQUISITION = 5;
const uint8_t STATE_REPORT_COLLECTED_DATA = 6;
const uint8_t STATE_RESET = 7;
const uint8_t STATE_ERROR = 8;

// Current state in the program sequence
uint8_t currentState;

// String to collect user input over serial
String userInput = "";

// Int values for finding strongest RSSI signal
int max_bike1;
int max_bikey1;
int max_bike2;
int max_bikey2;
int rec_home = 0;
int max_bike3;
int max_bikey3;
String virtual_distance = "";   //message for Blynk app

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega2560__)  // Feather 328P w/wing
#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#endif


// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

//motor setup
#define stp 13
#define dir 12
#define MS1 6
#define MS2 7
#define MS3 9
#define EN  8



void setup()
{
  //Bluetooth Setup
  SerialBLE.begin(9600);
  Blynk.begin(SerialBLE, auth);

  //Motor Setup
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(EN, OUTPUT);
  resetBEDPins(); //Set step, direction, microstep and enable pins to default states

  // Initialize serial
  Serial.begin(115200);    // serial terminal on the computer
  Serial1.begin(115200); // sweep device

  // reserve space to accumulate user message
  userInput.reserve(50);

  // initialize counter variables and reset the current state
  reset();

  //Radio Setup
  Serial.begin(115200);   //change from 115200
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);

  // Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

// Loop functions as an FSM (finite state machine)
void loop()
{
  Blynk.run();   //starting blynk app and leds
  led2.off();
  led3.off();
  led4.off();
  virtual_distance = String("NO BIKE YET");  //send message to app
  Blynk.virtualWrite(1, virtual_distance);
  uint8_t x1 = 0;   //rssi storage variables
  uint8_t x2 = 0;
  uint8_t x3 = 0;
  uint8_t x4 = 0;
  uint8_t y1 = 0;
  uint8_t y2 = 0;
  uint8_t y3 = 0;
  uint8_t y4 = 0;
  uint8_t z = 0;

  //Case statement for LIDAR
  switch (currentState)
  {
    case STATE_WAIT_FOR_USER_INPUT:
      if (listenForUserInput())
        currentState = STATE_ADJUST_DEVICE_SETTINGS;
      break;
    case STATE_ADJUST_DEVICE_SETTINGS:
      currentState = adjustDeviceSettings() ? STATE_VERIFY_CURRENT_DEVICE_SETTINGS : STATE_ERROR;
      break;
    case STATE_VERIFY_CURRENT_DEVICE_SETTINGS:
      currentState = verifyCurrentDeviceSettings() ? STATE_BEGIN_DATA_ACQUISITION : STATE_ERROR;
      break;
    case STATE_BEGIN_DATA_ACQUISITION:

      currentState = beginDataCollectionPhase() ? STATE_GATHER_DATA : STATE_ERROR;
      break;
    case STATE_GATHER_DATA:
      gatherSensorReading();
      if (scanCount > 1)
        currentState = STATE_STOP_DATA_ACQUISITION;

      break;
    case STATE_STOP_DATA_ACQUISITION:
      currentState = stopDataCollectionPhase() ? STATE_REPORT_COLLECTED_DATA : STATE_ERROR;

      break;
    case STATE_REPORT_COLLECTED_DATA:
      printCollectedData();
      currentState = STATE_RESET;
      break;
    case STATE_RESET:
      Serial.println("\n\nAttempting to reset and run the program again...");
      reset();
      currentState = STATE_WAIT_FOR_USER_INPUT;
      break;
    default: // there was some error
      Serial.println("\n\nAn error occured. Attempting to reset and run program again...");
      reset();
      currentState = STATE_WAIT_FOR_USER_INPUT;
      break;
  }

  if (rf69.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    char delimiter[] = ",";
    char* valPosition;

    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;

      if (strstr((char *)buf, "Bike")) {   //parsing if bike 1, 2, or other
        // Send a reply!
        //  Serial.println((char*)buf);
        valPosition = strtok((char *)buf, delimiter);
        // Serial.println(valPosition);
        int bikenumber = atoi(valPosition);
        // Serial.println(bikenumber);
        if (bikenumber == 1)
        {
          //  Serial.println("Hello Bike 1");
        }
        else if (bikenumber == 2)
        {
          //  Serial.println("Hello Bike 2");
        }
        else
        {
          Serial.print("Hello Bike:");
          Serial.println(bikenumber);

          Serial.print("Received [");
          Serial.print(len);
          Serial.print("]: ");
          Serial.println((char*)buf);
          Serial.print("RSSI: ");
          Serial.println(rf69.lastRssi(), DEC);
        }
        if (rec_home == 0)  //starts if LIDAR is ready
        {
          //following code gets the rssi values, compares them to find the smallest value and
          //sends that to the lidar

          if (bikenumber == 1) {    //gets rssi value
            Serial.print("Received from bike 1 [");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 1: ");
            Serial.println(rf69.lastRssi(), DEC);
            x1 = abs((rf69.lastRssi()));     //stores rssi value
            Serial.print("Received first x value:");
          }
          else if (bikenumber == 2) { //gets rssi value
            Serial.print("Received from bike 2[");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 2: ");
            Serial.println(rf69.lastRssi(), DEC);

            y1 = abs((rf69.lastRssi()));   //stores rssi value
            Serial.print("y value:");
            Serial.println(y1);
          }
          else
          {
            Serial.print("Received from unknown bike[");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Unknown Bike : ");
            Serial.println(rf69.lastRssi(), DEC); //gets rssi value

            z = abs((rf69.lastRssi()));
            Serial.print("z value:");
            Serial.println(z);
          }

          digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
          ForwardStepDefault();   //rotating motor
          resetBEDPins();
          delay(3000);

          if (bikenumber == 1) {
            Serial.print("Received from bike 1 ["); //gets rssi value
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 1: ");
            Serial.println(rf69.lastRssi(), DEC);
            Serial.print("Read Rssi value");
            Serial.println(rf69.rssiRead(), DEC);

            x2 = abs((rf69.lastRssi()));
            Serial.print("Received second x value:");
            Serial.println(x2);
          }
          else if (bikenumber == 2) {  //gets rssi value
            Serial.print("Received from bike 2[");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 2: ");
            Serial.println(rf69.lastRssi(), DEC);

            y2 = abs((rf69.lastRssi()));
            Serial.print("y value:");
            Serial.println(y2);
          }
          else
          {
            Serial.print("Received from unknown bike[");  //gets rssi value
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Unknown Bike : ");
            Serial.println(rf69.lastRssi(), DEC);

            z = abs((rf69.lastRssi()));
            Serial.print("z value:");
            Serial.println(z);
          }

          if (x1 != 0)   //stops avg rssi if no bike 1
          {
            max_bike1 = min(x1, x2);
            //Serial.println(max_bike1);   compare rssi values
            if (max_bike1 == x1)
            {
              Serial.print(" Max 1 is x1");
              Serial.println("x1");
            }
            if (max_bike1 == x2)
            {
              Serial.print(" Max 1 is x2");
              Serial.println("x2");
            }
          }
          if (y1 != 0 )   //stops avg rssi if no bike 2
          {
            max_bikey1 = min(y1, y2);   //compare rssi values
            //Serial.println(max_bike1);
            if (max_bikey1 == y1)
            {
              Serial.println("y1");
            }
            if (max_bikey1 == y2)
            {
              Serial.println("y2");
            }
          }

          digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
          ForwardStepDefault();   //move motor
          resetBEDPins();
          delay(3000);

          if (bikenumber == 1) {   //get rssi value
            Serial.print("Received from bike 1 [");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 1: ");
            Serial.println(rf69.lastRssi(), DEC);

            x3 = abs((rf69.lastRssi()));
            Serial.print("Received 3rd x value:");
            Serial.println(x3);
          }
          else if (bikenumber == 2) {   //get rssi value
            Serial.print("Received from bike 2[");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 2: ");
            Serial.println(rf69.lastRssi(), DEC);

            y3 = abs((rf69.lastRssi()));
            // Serial.print("y value:");
            //Serial.println(y3);
          }
          else
          {
            Serial.print("Received from unknown bike["); //get rssi value
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Unknown Bike : ");
            Serial.println(rf69.lastRssi(), DEC);

            z = abs((rf69.lastRssi()));
            Serial.print("z value:");
            Serial.println(z);
          }

          if (max_bike1 != 0)    //stops avg rssi if no bike 1
          {
            max_bike2 = min(max_bike1, x3);  //comparing rssi values

            if (max_bike2 == max_bike1)
            {
              Serial.print("Max is prev value");
              Serial.println("max_bike1");
            }
            if (max_bike1 == x3)
            {
              Serial.print("Max is new value");
              Serial.println("x3");
            }
          }
          if (max_bikey1 != 0)   //comparing rssi values
          {
            max_bikey2 = min(max_bikey1, y3);
            //Serial.println(max_bike1);
            if (max_bikey2 == max_bikey1)
            {
              Serial.println("max_bikey1");
            }
            if (max_bikey1 == y3)
            {
              Serial.println("y3");
            }
          }

          digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
          ForwardStepDefault();
          resetBEDPins();
          delay(3000);

          if (bikenumber == 1) {

            Serial.print("Received from bike 1 [");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 1: ");
            Serial.println(rf69.lastRssi(), DEC);
            x4 = abs((rf69.lastRssi()));
            Serial.print("4th x value:");
            Serial.println(x4);
          }

          else if (bikenumber == 2) {
            Serial.print("Received from bike 2[");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 2: ");
            Serial.println(rf69.lastRssi(), DEC);

            y4 = abs((rf69.lastRssi()));
          }
          else
          {
            Serial.print("Received from unknown bike[");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Unknown Bike : ");
            Serial.println(rf69.lastRssi(), DEC);

            z = abs((rf69.lastRssi()));
            Serial.print("z value:");
            Serial.println(z);
          }

          if (max_bike2 != 0)
          {
            max_bike3 = min(max_bike2, x4);
            //Serial.println(max_bike1);
            if (max_bike3 == max_bike2)
            {
              Serial.print("max value is old value");
              Serial.println("max_bike2");
            }
            if (max_bike3 == x4)
            {
              Serial.print("max value is max bike 4");
              Serial.print(max_bike3);
              Serial.println("x4");
            }
          }
          if (max_bikey2 != 0)
          {
            max_bikey3 = min(max_bikey2, y4);
            //Serial.println(max_bike1);
            if (max_bikey3 == max_bikey2)
            {
              Serial.println("max_bikey2");
            }
            if (max_bikey3 == y4)
            {
              Serial.println("y4");
            }
          }

          digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
          StepHomeDefault();  //move motor
          resetBEDPins();
          delay(3000);

          rec_home = 1;  //go to lidar
        }
      }
    }
  }
}

// checks if the user has communicated anything over serial
// looks for the user to send "start"
bool listenForUserInput()
{
  int count_start = 0;
  count_start++;
  if (count_start == 1)
  {
    Serial.println("Starting LIDAR!");
    return true;
  }
  else
    return false;
}

// Adjusts the device settings
bool adjustDeviceSettings()
{
  // Set the motor speed to 5HZ (codes available from 1->10 HZ)
  bool bSuccess = device.setMotorSpeed(MOTOR_SPEED_CODE_5_HZ);
  Serial.println(bSuccess ? "\nSuccessfully set motor speed." : "\nFailed to set motor speed");

  return bSuccess;
}

// Querries the current device settings (motor speed and sample rate)
// and prints them to the console
bool verifyCurrentDeviceSettings()
{
  // Read the current motor speed and sample rate
  int32_t currentMotorSpeed = device.getMotorSpeed();
  if (currentMotorSpeed < 0)
  {
    Serial.println("\nFailed to get current motor speed");
    return false;
  }
  int32_t currentSampleRate = device.getSampleRate();
  if (currentSampleRate < 0)
  {
    Serial.println("\nFailed to get current sample rate");
    return false;
  }

  // Report the motor speed and sample rate to the computer terminal
  Serial.println("\nMotor Speed Setting: " + String(currentMotorSpeed) + " HZ");
  Serial.println("Sample Rate Setting: " + String(currentSampleRate) + " HZ");

  return true;
}

// Initiates the data collection phase (begins scanning)
bool beginDataCollectionPhase()
{

  // Attempt to start scanning
  Serial.println("\nWaiting for motor speed to stabilize and calibration routine to complete...");
  bool bSuccess = device.startScanning();
  Serial.println(bSuccess ? "\nSuccessfully initiated scanning..." : "\nFailed to start scanning.");

  if (bSuccess)
    Serial.println("\nGathering 1 scan...");
  return bSuccess;
}

// Gathers individual sensor readings until 1 complete scan has been collected
void gatherSensorReading()
{
  //Serial.println("Gather sensor readings");
  // attempt to get the next scan packet
  // Note: getReading() will write values into the "reading" variable
  bool success = false;
  ScanPacket reading = device.getReading(success);
  if (success)
  {
    // check if this reading was the very first reading of a new 360 degree scan
    if (reading.isSync())
      scanCount++;

    // don't collect more than 3 scans
    if (scanCount > 1)
      return;

    // store the info for this sample
    syncValues[sampleCount] = reading.isSync();
    angles[sampleCount] = reading.getAngleDegrees();
    distances[sampleCount] = reading.getDistanceCentimeters();
    signalStrengths[sampleCount] = reading.getSignalStrength();

    // increment sample count
    sampleCount++;
  }
}

// Terminates the data collection phase (stops scanning)
bool stopDataCollectionPhase()
{
  // Attempt to stop scanning
  bool bSuccess = device.stopScanning();

  Serial.println(bSuccess ? "\nSuccessfully stopped scanning." : "\nFailed to stop scanning.");

  return bSuccess;
}

// Prints the collected data to the console
// (only prints the complete scans, ignores the first partial)

void printCollectedData()
{

  if (rec_home == 1)
  {
    Serial.println("\nPrinting info for the collected scans (NOT REAL-TIME):");

    int indexOfFirstSyncReading = 0;
    // don't print the trailing readings from the first partial scan
    while (!syncValues[indexOfFirstSyncReading])
    {
      indexOfFirstSyncReading++;
    }
    // print the readings for all the complete scans
    // prints scan values for specific angles
    //max_bike3 is for bike1 and max_bikey3 is for bike 2
    for (int i = indexOfFirstSyncReading; i < sampleCount; i++)
    {
      if (syncValues[i])
      {
        Serial.print("Max Bike3 Value:");
        Serial.println(max_bike3);
        Serial.println("\n----------------------NEW SCAN----------------------");
      }
      if (max_bike3 > 24 && max_bike3 < 29)   //0-60 degrees
      {
        led2.on();  //turns led of phone on
        virtual_distance = String("BIKE CLOSE");  //send phone message
        Blynk.virtualWrite(1, virtual_distance);

        if (uint16_t(distances[i]) > 100 && uint16_t(distances[i]) < 1000)
        {
          Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
        }

      }

      if (max_bikey3 > 24 && max_bikey3 < 29)  //60 degrees
      {
        led2.on();  //turns led of phone on
        virtual_distance = String("BIKE CLOSE");
        Blynk.virtualWrite(1, virtual_distance);  //send phone message
        // Serial.print("O-60 degrees");
        // if (angles[i] > 0 && angles[i] < 46)
        //  {
        if ((distances[i]) > 200 && (distances[i]) < 1000)
        {
          Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
          //     virtual_distance = (distances[i]);
          //   Blynk.virtualWrite(1, virtual_distance);
        }
        //}
      }

      if (max_bike3 > 28 && max_bike3 < 35)  //61-120 degrees
      {
        led3.on();
        // Serial.print("61-120 degrees");
        virtual_distance = String("BIKE CLOSE");
        Blynk.virtualWrite(1, virtual_distance);
        // Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
        // if (angles[i] > 239 && angles[i] < 300)
        //{
        if ((distances[i]) > 200 && (distances[i]) < 1000)
        {
          Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));

        }
      }
      //   }

      if (max_bikey3 > 28 && max_bikey3 < 35)
      {
        led3.on();
        // Serial.print("61-120 degrees");
        virtual_distance = String("BIKE CLOSE");
        Blynk.virtualWrite(1, virtual_distance);
        // Serial.print("611-120 degrees");
        // Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
        // if (angles[i] > 239 && angles[i] < 300)
        //  {
        if ((distances[i]) > 200 && (distances[i]) < 1000)
        {
          Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
          //   virtual_distance = (distances[i]);
          //   Blynk.virtualWrite(1, virtual_distance);
        }
        //  }
      }


      if (max_bike3 > 34 && max_bike3 < 52)  //121-180 degrees
      {
        led4.on();
        virtual_distance = String("BIKE CLOSE");
        Blynk.virtualWrite(1, virtual_distance);
        // Serial.print("121-180 degrees");
        Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
        //  if (angles[i] > 179 && angles[i] < 240)
        //  {
        if ((distances[i]) > 200 && (distances[i]) < 1000)
        {
          Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
          //   virtual_distance = (distances[i]);
          //   Blynk.virtualWrite(1, virtual_distance);
        }
        //  }
      }

      if (max_bikey3 > 34 && max_bikey3 < 52)  //121-180 degrees
      {
        led4.on();
        virtual_distance = String("BIKE CLOSE");
        Blynk.virtualWrite(1, virtual_distance);

        //  Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
        //   if (angles[i] > 90 && angles[i] < 136)
        //   {
        if ((distances[i]) > 200 && (distances[i]) < 1000)
        {
          Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
          //   virtual_distance = (distances[i]);
          //   Blynk.virtualWrite(1, virtual_distance);
        }
      }
    }
    rec_home = 0;  //go back to rssi readings
  }
}

// Resets the variables and state so the sequence can be repeated
void reset()
{
  scanCount = 0;
  sampleCount = 0;
  // reset the sensor
  device.reset();
  delay(50);
  Serial.flush();
  userInput = "";
  Serial.println("\n\nWhenever you are ready, type \"start\" to to begin the sequence...");
  currentState = 0;
}

//reset for new motor value
void resetBEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  digitalWrite(EN, HIGH);
}
//moves motor forward
void ForwardStepDefault()
{
  int x;
  Serial.println("Moving forward at default step mode.");
  digitalWrite(dir, HIGH); //Pull direction pin high to move in "reverse"
  // digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, HIGH); //Pull MS1,MS2, and MS3 high to set logic to 1/16th microstep resolution
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);
  for (x = 1; x < 600; x++) //Loop the stepping enough times for motion to be visible
  {
    digitalWrite(stp, HIGH); //Trigger one step
    delay(1);
    digitalWrite(stp, LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  // Serial.println("Enter new option");
  // Serial.println();
}

//moves motor back to start
void StepHomeDefault()
{
  int x;
  Serial.println("Moving forward at default step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, HIGH); //Pull MS1,MS2, and MS3 high to set logic to 1/16th microstep resolution
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);
  for (x = 1; x < 2100; x++) //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp, HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp, LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
}


