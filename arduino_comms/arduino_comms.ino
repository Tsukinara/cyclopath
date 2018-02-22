#include <Sweep.h>
#include <SoftwareSerial.h>

/* pin data */

// interrupt pin for the hall sensor
int HALL_INT = 2;

// serial pins for bluetooth
int BT_TX = 4;
int BT_RX = 5;

// serial pints for sweep
int SWP_TX = 6;
int SWP_RX = 7;

// output pins
int LED_0 = 8;

/* software serial setup for bluetooth and sweep streams */
SoftwareSerial bluetooth(BT_TX, BT_RX);
SoftwareSerial sweep_str(SWP_TX, SWP_RX);

Sweep sweep_dev(sweep_str);

/* global constants */
const float WHEEL_RAD = 0.6;  // wheel radius in meters
const int THERM_CNT = 8;      // total number of thermistors
const int THERM_FREQ = 1000;  // frequency at which to send temperature data

/* global variables */
unsigned long prev_turn;      // time of previous rotation
float curr_vel;               // current velocity of the bicycle
bool vel_updated = true;      // boolean for whether or not to send updated velocity data

char curr_cmd[16];            // most recently received command via Bluetooth
int curr_ind;                 // current index within the command

int therm_timer;              // timer variable for sending temperature data

/**
 * setup: initializes Arduino settings upon reset
 */
void setup() {
  pinMode(HALL_INT, INPUT_PULLUP);
  pinMode(LED_0, OUTPUT);
  
  attachInterrupt(HALL_INT, calc_vel, RISING);
  
  bluetooth.begin(115200);
  sweep_str.begin(115200);
  Serial.begin(9600);

  reset();
}

/**
 * loop: indefinitely receives and transmits data over BlueTooth
 */
void loop() {
  bluetooth_rcv();
  bluetooth_snd(); 
}

/**
 * bluetooth_rcv: receives data from Bluetooth if available
 */
void bluetooth_rcv() {
  char c;
  
  /* receive bluetooth data */
  if (bluetooth.available()) {
    c = (char)bluetooth.read();
    switch (c) {
      case '%':   
        curr_cmd[0] = '%'; curr_ind = 1;
        break;
      case '\n':
        execute();
        curr_cmd[0] = '\0'; curr_ind = 0;
        break;
      default:
        if (strlen(curr_cmd) > 0 && curr_cmd[0] == '%') { 
          curr_cmd[curr_ind++] = c;
        }
        break;
    }
    curr_cmd[curr_ind] = '\0';
  }
}

/**
 * bluetooth_snd: sends gathered telemetry via Bluetooth, if available
 */
void bluetooth_snd() {
  int i;
  
  /* send velocity data */
  if (vel_updated) {
    bluetooth.print("%VEL: "); bluetooth.println(curr_vel);
    vel_updated = false;
  }

  /* send temperature data once every THERM_FREQ loops */
  if ((therm_timer++) % THERM_FREQ == 0) {
    for (i = 0; i < THERM_CNT; i++) {
      bluetooth.print("%TH"); bluetooth.print((i+1)); bluetooth.print(": ");
      bluetooth.println(get_battery_temp(i));
    }
    therm_timer = 0;
  }
}

/**
 * execute: executes the command in the current cmd string
 */
void execute() {
  if (!strcmp(curr_cmd, "%LD0U")) { digitalWrite(LED_0, HIGH); }
  if (!strcmp(curr_cmd, "%LD0D")) { digitalWrite(LED_0, LOW); }
}

/**
 * get_battery_temp: gets the average temperature returned the specified thermistor, in C
 */
float get_battery_temp(int thermistor_id) {
  if (thermistor_id >= 0 && thermistor_id < 8) {
    return 1;
  }
  return 0;
}

/**
 * calc_speed: upon a new reading from the hall sensor, calculates the new
 * current velocity of the bike.
 */
void calc_vel() {
  unsigned long curr = millis();
  
  /* noise cut : assume more than 80ms / revolution */
  if (curr - prev_turn > 80) {
    curr_vel = WHEEL_RAD / ((float)curr - prev_turn / 1000) * 3.6;
    prev_turn = curr;
  }
  vel_updated = true;
}

void reset() {
  // reset FSM state
//  currentState = STATE_WAIT_ON_RESET;

  // reset velocity calculation variables
  prev_turn = millis();
  curr_vel = 0;
  therm_timer = 0;

  // reset Bluetooth commands
  curr_cmd[0] = '\0';
  curr_ind = 0;
  
  // reset sweep device
  sweep_dev.reset();
}

