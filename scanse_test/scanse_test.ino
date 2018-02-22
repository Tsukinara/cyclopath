#include <Sweep.h>
#include <SoftwareSerial.h>

int SWP_RX = 3;
int SWP_TX = 2;

// Create a Sweep device using Serial #1 (RX1 & TX1)
SoftwareSerial sweep_str(SWP_TX, SWP_RX);
Sweep device(sweep_str);

// keeps track of how many scans have been collected
uint8_t scanCount = 0;
// keeps track of how many samples have been collected
uint16_t sampleCount = 0;

// Arrays to store attributes of collected scans
bool syncValue;         // 1 -> first reading of new scan, 0 otherwise
float angle;            // in degrees (accurate to the millidegree)
uint16_t distance;      // in cm

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

void setup()
{
  // Initialize serial
  Serial.begin(9600);    // serial terminal on the computer
  sweep_str.begin(115200); // sweep device

  // reserve space to accumulate user message
  userInput.reserve(50);

  // initialize counter variables and reset the current state
  reset();
}

// Loop functions as an FSM (finite state machine)
void loop()
{
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
}

// checks if the user has communicated anything over serial
// looks for the user to send "start"
bool listenForUserInput()
{
  while (Serial.available())
  {
    userInput += (char)Serial.read();
  }
  if (userInput.indexOf("start") != -1)
  {
    Serial.println("Registered user start.");
    return true;
  }
  return false;
}

// Adjusts the device settings
bool adjustDeviceSettings()
{
  // Set the motor speed to 5HZ (codes available from 1->10 HZ)
  bool bSuccess = device.setMotorSpeed(MOTOR_SPEED_CODE_10_HZ);
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
  // Attempt to start scanning
  Serial.println("\nWaiting for motor speed to stabilize and calibration routine to complete...");
  bool bSuccess = device.startScanning();
  Serial.println(bSuccess ? "\nSuccessfully initiated scanning..." : "\nFailed to start scanning.");
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
    angle = reading.getAngleDegrees();
    distance = reading.getDistanceCentimeters();
    Serial.println("Angle: " + String(angle, 3) + "\t |  Distance: " + String(distance) + " cm");
  } else {
    Serial.println("Failed to get sensor reading.");
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
