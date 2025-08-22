/*
 * servo_controller.ino
 * Author: Treggon Owens for W8less.io
 * Description: ROS-integrated Arduino code for controlling 16-channel PCA9685 servos,
 * reading 2 PWM inputs, with OE pin safety, watchdog timeout, and heartbeat mechanism.
 * 8/22/2025 Created
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <servo_controller_msgs/ServoArray.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 1000
#define SERVOMAX 2000
#define CHANNELS 16
#define ENABLE_PIN 7  // OE pin on PCA9685
#define TIMEOUT_MS 1000
#define HEARTBEAT_INTERVAL_MS 500

// for outputs
#define PWM_INPUT_1 2 // (Sonic Sensor)
#define PWM_INPUT_2 3 // (Sonic Sensor)

int servoAngles[CHANNELS];
bool servoEnabled[CHANNELS];
unsigned long lastCommandTime = 0;
unsigned long lastHeartbeatTime = 0;
bool heartbeatActive = false;

ros::NodeHandle nh;
servo_controller_msgs::ServoArray servo_msg;
ros::Publisher servo_pub("servo_state", &servo_msg);
void commandCallback(const servo_controller_msgs::ServoArray &msg);
ros::Subscriber<servo_controller_msgs::ServoArray> servo_sub("servo_cmd", &commandCallback);

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  pinMode(PWM_INPUT_1, INPUT);
  pinMode(PWM_INPUT_2, INPUT);

  for (int i = 0; i < CHANNELS; i++) {
    servoAngles[i] = 90;
    servoEnabled[i] = true;
    pwm.setPWM(i, 0, map(servoAngles[i],0,180,SERVOMIN,SERVOMAX));
  }

  lastCommandTime = millis();
  lastHeartbeatTime = millis();
  heartbeatActive = false;

  nh.initNode();
  nh.advertise(servo_pub);
  nh.subscribe(servo_sub);
}

void setServoAngle(int channel, int angle) {
  if(channel < 0 || channel >= CHANNELS) return;
  servoAngles[channel] = angle;
  if(servoEnabled[channel]) pwm.setPWM(channel, 0, map(angle,0,180,SERVOMIN,SERVOMAX));
  else pwm.setPWM(channel, 0, 0);
}

void setServoEnabled(int channel, bool enable) {
  if(channel < 0 || channel >= CHANNELS) return;
  servoEnabled[channel] = enable;
  if(enable) pwm.setPWM(channel, 0, map(servoAngles[channel],0,180,SERVOMIN,SERVOMAX));
  else pwm.setPWM(channel, 0, 0);
}

void disableAllServos() {
  digitalWrite(ENABLE_PIN, HIGH);
  for(int i=0; i<CHANNELS; i++) pwm.setPWM(i,0,0);
}

void enableAllServos() {
  digitalWrite(ENABLE_PIN, LOW);
  for(int i=0; i<CHANNELS; i++) {
    if(servoEnabled[i]) pwm.setPWM(i, 0, map(servoAngles[i],0,180,SERVOMIN,SERVOMAX));
  }
}

unsigned long readPWM(int pin) {
  return pulseIn(pin,HIGH,25000);
}

void commandCallback(const servo_controller_msgs::ServoArray &msg) {
  for(int i=0; i<CHANNELS; i++) {
    setServoAngle(i,msg.angles[i]);
    setServoEnabled(i,msg.enabled[i]);
  }
  lastCommandTime = millis();
  heartbeatActive = true;
  enableAllServos();
}

void loop() {
  nh.spinOnce();

  // Timeout: disable outputs if no commands received
  if(millis() - lastCommandTime > TIMEOUT_MS) {
    disableAllServos();
    heartbeatActive = false;
  }

  // Heartbeat: if active, ensure outputs are enabled
  if(heartbeatActive && millis() - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
    enableAllServos();
    lastHeartbeatTime = millis();
  }

  // Publish servo states and PWM inputs
  for(int i=0; i<CHANNELS; i++){
    servo_msg.angles[i] = servoAngles[i];
    servo_msg.enabled[i] = servoEnabled[i];
  }
  servo_msg.pwm_input_1 = readPWM(PWM_INPUT_1);
  servo_msg.pwm_input_2 = readPWM(PWM_INPUT_2);

  servo_pub.publish(&servo_msg);

  delay(10);
}
