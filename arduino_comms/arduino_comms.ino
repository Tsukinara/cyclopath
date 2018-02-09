#include <Sweep.h>
#include <SoftwareSerial.h>

int BT_TX = 4;
int BT_RX = 5;
int BT_LED = 6;

int SWP_TX = 7;
int SWP_RX = 8;

int HALL_INT = 2;

SoftwareSerial bluetooth(BT_TX, BT_RX);
SoftwareSerial sweep(SWP_TX, SWP_RX);

Sweep sweep(sweep);

unsigned long prev_turn;
float curr_vel;
float wheel_rad = 0.6;

void setup() {
  attachInterrupt(HALL_INT, calc_speed, RISING);
  bluetooth.begin(115200);
  
  prev_turn = millis();
  curr_vel = 0;
}

void loop() {
  if (check_bluetooth()) {
      bluetooth_rcv();
      bluetooth_snd(); 
  }
}

bool check_bluetooth() {
  if (bluetooth.available()) {
    digitalWrite(BT_LED, HIGH);
    return true;
  } else {
    digitalWrite(BT_LED, LOW);
    return false;
  }
}
void bluetooth_rcv() {
  /* receive bluetooth data here */
}

void bluetooth_snd() {
  /* send bluetooth data here */
}

void calc_speed() {
  unsigned long curr = millis();
  
  /* noise cut : assume more than 80ms / revolution */
  if (curr - prev_turn > 80) {
    curr_vel = wheel_rad / ((float)curr - prev_turn / 1000) * 3.6;
    prev_turn = curr;
  }
}

