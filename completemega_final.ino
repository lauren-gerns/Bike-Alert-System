#include <ScanPacket.h>
#include <Sweep.h>

// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include <SoftwareSerial.h>


//*** Set Up Sweep
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
uint8_t signalStrengths[500]; // 0:255, higher is better

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

//****Bluetooth Setup
int bluetoothTx = 6;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 7;  // RX-I pin of bluetooth mate, Arduino D3



SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);


int rec1 = 0;
int max_bike1;
 int max_bikey1;
 int max_bike2;
 int max_bikey2;

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

/*
  #if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
  #endif

  #if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
  #endif

*/
/*
  #if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  //
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
  #endif
*/
/*
  #if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
  #endif

  #if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
  #endif
*/
#if defined (__AVR_ATmega2560__)  // Feather 328P w/wing
#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#define LED           51
#endif



// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

//Average Setup
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int avgvalue = 0;

//motor setup
#define stp 13
#define dir 12
#define MS1 11
#define MS2 10
#define MS3 9
#define EN  8

int max_bike3;
int max_bikey3;

void setup()
{


  //Motor Setup
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(EN, OUTPUT);
  resetBEDPins(); //Set step, direction, microstep and enable pins to default states

  //Bluetooth Setup
  Serial.begin(115200);  // Begin the serial monitor at 9600bps

  ///Sweep Setup
  Serial1.begin(115200); // sweep device

  // reserve space to accumulate user message
  userInput.reserve(50);

  // initialize counter variables and reset the current state
  reset();

  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$$$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600


  //Average Setup
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  //Radio Setup
  Serial.begin(115200);   //change from 115200
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);
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

  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

void loop() {

  //int max_bike3;
  //****Sweep
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
      if (scanCount > 3)
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

  //****Bluetooth
  
  if (bluetooth.available()) // If the bluetooth sent any characters
  {
    // Send any characters the bluetooth prints to the serial monitor
    Serial.print((char)bluetooth.read());
  }
  if (Serial.available()) // If stuff was typed in the serial monitor
  {
    //bluetooth.print(("greetings"));
    //bluetooth.print(rf69.lastRssi(), DEC);
    // Send any characters the Serial monitor prints to the bluetooth
    bluetooth.print((char)Serial.read());
  }



  //Radio

  
  if (rf69.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    char delimiter[] = ",";
    char* valPosition;

    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;

      if (strstr((char *)buf, "Bike")) {
        // Send a reply!
        Serial.println((char*)buf);
        valPosition = strtok((char *)buf, delimiter);
        // Serial.println(valPosition);
        int bikenumber = atoi(valPosition);
        Serial.println(bikenumber);
        if (bikenumber == 1)
        {
          Serial.println("Hello Bike 1");
        }
        else if (bikenumber == 2)
        {
          Serial.println("Hello Bike 2");
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



        uint8_t x1 = 0;
        uint8_t x2 = 0;
        uint8_t x3 = 0;
        uint8_t x4 = 0;
        uint8_t y1 = 0;
        uint8_t y2 = 0;
        uint8_t y3 = 0;
        uint8_t y4 = 0;
        uint8_t z = 0;


        if (bikenumber == 1) {
          Serial.print("Received from bike 1 [");
          Serial.print(len);
          Serial.print("]: ");
          Serial.println((char*)buf);
          Serial.print("RSSI Bike 1: ");
          Serial.println(rf69.lastRssi(), DEC);
          x1 = abs((rf69.lastRssi()));
          Serial.print("x value:");
          Serial.println(x1);
        //  rec1 = 1;
        }


        else if (bikenumber == 2) {
          Serial.print("Received from bike 2[");
          Serial.print(len);
          Serial.print("]: ");
          Serial.println((char*)buf);
          Serial.print("RSSI Bike 2: ");
          Serial.println(rf69.lastRssi(), DEC);

          y1 = abs((rf69.lastRssi()));
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
          Serial.println(rf69.lastRssi(), DEC);

          z = abs((rf69.lastRssi()));
          Serial.print("z value:");
          Serial.println(z);
        }


*/

        ///// Average Testing//////
        // subtract the last reading:
        // total = total - readings[readIndex];
        // read from the sensor:
        //   readings[readIndex] = x;
        // add the reading to the total:
        //   total = total + readings[readIndex];
        // advance to the next position in the array:
        //   readIndex = readIndex + 1;

        // if we're at the end of the array...
        //  if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        //     readIndex = 0;
        //   }

        // calculate the average:
        //   average = total / numReadings;
        // send it to the computer as ASCII digits

        //   avgvalue++;
        ////   Serial.print("Loop number for avg: ");
        //   Serial.println(avgvalue);
        //    Serial.print("average number: ");
        //     Serial.println(average);
        //    delay(1);        // delay in between reads for stability



        //Back to Radio

        //  if (strstr((char *)buf, "Hello World")) {
        // Send a reply!
        //   uint8_t data[] = "And hello back to you";
        //   rf69.send(data, sizeof(data));
        //   rf69.waitPacketSent();
        //    Serial.println("Sent a reply");
        //Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks

        // if(bluetooth.available())  // If the bluetooth sent any characters
        //{
        // Send any characters the bluetooth prints to the serial monitor
        // Serial.print((char)bluetooth.read());


        //  bluetooth.print(rf69.lastRssi(), DEC);
        //  bluetooth.print('\n');
        //  delay(1000);
        // }
        //  }
        //  } else {
        //     Serial.println("Receive failed");



        if (rec1 == 2)
        {
          digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
          ForwardStepDefault();
          resetBEDPins();
         // delay(1000);
          if (bikenumber == 1) {
            Serial.print("Received from bike 1 [");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 1: ");
            Serial.println(rf69.lastRssi(), DEC);
            x2 = abs((rf69.lastRssi()));
            Serial.print("x value:");
            Serial.println(x2);
          }

          else if (bikenumber == 2) {
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

          
          //Serial.print(x1 );
          //Serial.println(x2);
          max_bike1 = max(x1, x2);
          //Serial.println(max_bike1);
          if (max_bike1 == x1)
          {
            Serial.println("x1");
          }
          if (max_bike1 == x2)
          {
            Serial.println("x2");
          }

         

          //Serial.print(x1 );
          //Serial.println(x2);
          max_bikey1 = max(y1, y2);
          //Serial.println(max_bike1);
          if (max_bikey1 == y1)
          {
            Serial.println("y1");
          }
          if (max_bikey1 == y2)
          {
            Serial.println("y2");
          }
          rec1 = 1;
        }
        
        if (rec1 == 2)
        {
          digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control

          ForwardStepDefault();
          resetBEDPins();
          delay(1000);

          if (bikenumber == 1) {
            Serial.print("Received from bike 1 [");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 1: ");
            Serial.println(rf69.lastRssi(), DEC);
            x3 = abs((rf69.lastRssi()));
            Serial.print("x value:");
            Serial.println(x3);
          }

          else if (bikenumber == 2) {
            Serial.print("Received from bike 2[");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 2: ");
            Serial.println(rf69.lastRssi(), DEC);

            y3 = abs((rf69.lastRssi()));
            Serial.print("y value:");
            Serial.println(y3);
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

          
          //Serial.print(x1 );
          //Serial.println(x2);
          max_bike2 = max(max_bike1, x3);
          //Serial.println(max_bike1);
          if (max_bike2 == max_bike1)
          {
            Serial.println("max_bike1");
          }
          if (max_bike1 == x3)
          {
            Serial.println("x3");
          }

          //Serial.print(x1 );
          //Serial.println(x2);
          max_bikey2 = max(max_bikey1, y3);
          //Serial.println(max_bike1);
          if (max_bikey2 == max_bikey1)
          {
            Serial.println("max_bikey1");
          }
          if (max_bikey1 == y3)
          {
            Serial.println("y3");
          }
          rec1 = 1;
        }

        if (rec1 == 2)
        {
          digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control

          ForwardStepDefault();
          resetBEDPins();
          delay(1000);

          if (bikenumber == 1) {
            Serial.print("Received from bike 1 [");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char*)buf);
            Serial.print("RSSI Bike 1: ");
            Serial.println(rf69.lastRssi(), DEC);
            x4 = abs((rf69.lastRssi()));
            Serial.print("x value:");
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
            Serial.print("y value:");
            Serial.println(y4);
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

          //Serial.print(x1 );
          //Serial.println(x2);
          max_bike3 = max(max_bike2, x4);
          //Serial.println(max_bike1);
          if (max_bike3 == max_bike2)
          {
            Serial.println("max_bike2");
          }
          if (max_bike3 == x4)
          {
            Serial.println("x4");
          }

          // int max_bikey3;
          //Serial.print(x1 );
          //Serial.println(x2);
          max_bike3 = max(max_bikey2, y4);
          //Serial.println(max_bike1);
          if (max_bikey3 == max_bikey2)
          {
            Serial.println("max_bikey2");
          }
          if (max_bikey3 == y4)
          {
            Serial.println("y4");
          }

          rec1 = 1;
        }

        if (rec1 == 2) {
          digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control

          StepHomeDefault();
          resetBEDPins();
          //delay(1000);
          rec1 = 0;
        }
    */  }
      
   // }
 // }
//}



/*
  void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
  }
*/
/////***Sweep Functions

// checks if the user has communicated anything over serial
// looks for the user to send "start"
bool listenForUserInput()
{
  int count_start = 0;
  count_start++;
  if (count_start == 1 )
  {
    Serial.println("Registered user start.");
    return true;
  }
  else
    //  while (Serial.available())
    // {
    //  userInput += (char)Serial.read();

    //  }
    //  if (userInput.indexOf("start") != -1)
    ////  {
    //Serial.println("Registered user start.");
    // return true;
    // }
    return false;
}


// Adjusts the device settings
bool adjustDeviceSettings()
{
  // Set the motor speed to 5HZ (codes available from 1->10 HZ)
  bool bSuccess = device.setMotorSpeed(MOTOR_SPEED_CODE_5_HZ);
  Serial.println(bSuccess ? "\nSuccessfully set motor speed." : "\nFailed to set motor speed");

  /*
    // Device will always default to 500HZ scan rate when it is powered on.
    // Snippet below is left for reference.
    // Set the sample rate to 500HZ (codes available for 500, 750 and 1000 HZ)
    bool bSuccess = device.setSampleRate(SAMPLE_RATE_CODE_500_HZ);
    Serial.println(bSuccess ? "\nSuccessfully set sample rate." : "\nFailed to set sample rate.");
  */
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
  //if (rec1 == 1)

    // Attempt to start scanning
    Serial.println("\nWaiting for motor speed to stabilize and calibration routine to complete...");
  bool bSuccess = device.startScanning();

  Serial.println(bSuccess ? "\nSuccessfully initiated scanning..." : "\nFailed to start scanning.");
  if (bSuccess)
    Serial.println("\nGathering 3 scans...");
  rec1 = 2;

  return bSuccess;
}

// Gathers individual sensor readings until 3 complete scans have been collected
void gatherSensorReading()
{
  // attempt to get the next scan packet
  // Note: getReading() will write values into the "reading" variable
  bool success = false;
  ScanPacket reading = device.getReading(success);
  if (success)
  {

    // check if this reading was the very first reading of a new 360 degree scan
    if (reading.isSync())
      Serial.print("success");
    scanCount++;
   // Serial.print("Scan Count:");
  //  Serial.println(scanCount);

    // don't collect more than 1 scans
    if (scanCount > 3)
      Serial.print("Scan Count:");
    Serial.println(scanCount);

  Serial.print("Sync values");
   Serial.println(sampleCount);
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
  Serial.println("Stopped Scanning");
  Serial.println(bSuccess ? "\nSuccessfully stopped scanning." : "\nFailed to stop scanning.");
  return bSuccess;
}

// Prints the collected data to the console
// (only prints the complete scans, ignores the first partial)
void printCollectedData()
{
  Serial.println("\nPrinting info for the collected scans (NOT REAL-TIME):");

  int indexOfFirstSyncReading = 0;
  // don't print the trailing readings from the first partial scan
  while (!syncValues[indexOfFirstSyncReading])
  {
    indexOfFirstSyncReading++;
   // Serial.print(sampleCount);
  }
  // print the readings for all the complete scans
  for (int i = indexOfFirstSyncReading; i < sampleCount; i++)
  {
    Serial.print("i");
   Serial.print(i);
    if (syncValues[i])
   {
      Serial.println("\n----------------------NEW SCAN----------------------");
   }
    Serial.println("Angle: " + String(angles[i], 3) + ", Distance: " + String(distances[i]) + ", Signal Strength: " + String(signalStrengths[i]));
  }
}
/*void printCollectedData()
{
  Serial.println("\nPrinting info for the collected scans (NOT REAL-TIME):");

  int indexOfFirstSyncReading = 0;
  // don't print the trailing readings from the first partial scan
  while (!syncValues[indexOfFirstSyncReading])
  {
    //Serial.print("index of first sync reading");
    //Serial.println(indexOfFirstSyncReading);
    indexOfFirstSyncReading++;
  }
  // print the readings for all the complete scans
  for (int i = indexOfFirstSyncReading; i < sampleCount; i++)
  {
    Serial.print("sample:");
    Serial.println(sampleCount);
    if (syncValues[i])
    {
      Serial.print("Max Bike3 Value:");
      Serial.println(max_bike3);
      Serial.println("\n----------------------NEW SCAN----------------------");
    }

    Serial.print("Max Bike3 Value:");
    Serial.println(max_bike3);
    if (max_bike3 > 25 && max_bike3 < 28)
    {
      Serial.print("O-40 degrees");
      if (angles[i] > 0 && angles[i] < 41)
      {
        Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
      }
    }

    if (max_bikey3 > 25 && max_bikey3 < 28)
    {
      Serial.print("O-40 degrees");
      if (angles[i] > 0 && angles[i] < 41)
      {
        Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
      }
    }

    if (max_bike3 > 27 && max_bike3 < 30)
    {
      Serial.print("41-89 degrees");
      if (angles[i] > 40 && angles[i] < 90)
      {
        Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
      }
    }

    if (max_bikey3 > 27 && max_bikey3 < 30)
    {
      Serial.print("41-89 degrees");
      if (angles[i] > 40 && angles[i] < 90)
      {
        Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
      }
    }


    if (max_bike3 > 29 && max_bike3 < 35)
    {
      Serial.print("90-109 degrees");
      if (angles[i] > 89 && angles[i] < 110)
      {
        Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
      }
    }

    if (max_bikey3 > 30 && max_bikey3 < 35)
    {
      Serial.print("90-109 degrees");
      if (angles[i] > 89 && angles[i] < 110)
      {
        Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
      }
    }

    if (max_bike3 > 34 && max_bike3 < 42)
    {
      Serial.print("110-180 degrees");
      if (angles[i] > 109 && angles[i] < 181)
      {
        Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
      }
    }

    if (max_bikey3 > 34 && max_bikey3 < 42)
    {
      Serial.print("110-180 degrees");
      if (angles[i] > 109 && angles[i] < 181)
      {
        Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
      }
    }

*/


    /*
      if (average > 2 && average < 30)
      {
      Serial.print("Got Average.");
      if (angles[i] > 19 && angles[i] < 31)
      {
        Serial.println("Angle: " + String(angles[i], 3) + "Distance: " + String(distances[i]));
      }
      }
    */


    // Serial.println("Angle: " + String(angles[i], 3) + ", Distance: " + String(distances[i]) + ", Signal Strength: " + String(signalStrengths[i]));
 // }
//}
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

void ForwardStepDefault()
{
  int x;
  Serial.println("Moving forward at default step mode.");
  digitalWrite(dir, HIGH); //Pull direction pin high to move in "reverse"
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

void StepHomeDefault()
{
  int x;
  Serial.println("Moving forward at default step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  for (x = 1; x < 1800; x++) //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp, HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp, LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  //Serial.println("Enter new option");
  // Serial.println();
}

//Reset Big Easy Driver pins to default states
void resetBEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  digitalWrite(EN, HIGH);
}
