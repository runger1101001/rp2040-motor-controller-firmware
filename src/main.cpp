#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "./hw_setup.h"

#include "encoders/as5048a/MagneticSensorAS5048A.h"



#define SERIAL_SPEED 115200
#define FIRMWARE_VERSION "0.1"

#define DEFAULT_VOLTAGE_LIMIT 6.0f

#define MOTOR_PP 7
#define MOTOR_PHASE_RESISTANCE 12.6f




// motor 0
Encoder sensor0 = Encoder(ENCODER0_A_PIN, ENCODER0_B_PIN, 1024, ENCODER0_Z_PIN);
void handleA0() { sensor0.handleA(); };
void handleB0() { sensor0.handleB(); };
void handleZ0() { sensor0.handleIndex(); };
BLDCDriver6PWM driver0 = BLDCDriver6PWM(M0_INUH_PIN, M0_INUL_PIN, M0_INVH_PIN, M0_INVL_PIN, M0_INWH_PIN, M0_INWL_PIN);
//LowsideCurrentSense current0 = LowsideCurrentSense(1.0f, 1.0f/CURRENT_VpA, M0_AOUTU_PIN, M0_AOUTV_PIN, M0_AOUTW_PIN);
BLDCMotor motor0 = BLDCMotor(MOTOR_PP, MOTOR_PHASE_RESISTANCE);

// motor 1
Encoder sensor1 = Encoder(ENCODER1_A_PIN, ENCODER1_B_PIN, 1024, ENCODER1_Z_PIN);
void handleA1() { sensor1.handleA(); };
void handleB1() { sensor1.handleB(); };
void handleZ1() { sensor1.handleIndex(); };
BLDCDriver6PWM driver1 = BLDCDriver6PWM(M1_INUH_PIN, M1_INUL_PIN, M1_INVH_PIN, M1_INVL_PIN, M1_INWH_PIN, M1_INWL_PIN);
//LowsideCurrentSense current1 = LowsideCurrentSense(1.0f, 1.0f/CURRENT_VpA, M1_AOUTU_PIN, M1_AOUTV_PIN, M1_AOUTW_PIN);
BLDCMotor motor1 = BLDCMotor(MOTOR_PP, MOTOR_PHASE_RESISTANCE);

// Commander
Commander commander = Commander(Serial, '\n', false);
void onMotor0(char* cmd){commander.motor(&motor0, cmd);}
void onMotor1(char* cmd){commander.motor(&motor1, cmd);}

// main loop speed tracking
int count = 0;
unsigned long ts = 0;


void setup() {
  // initialize serial port on USB, debug output goes to this
  Serial.begin(SERIAL_SPEED);
  SimpleFOCDebug::enable();
  while (!Serial) ; // wait for serial port to connect - remove this later
  SimpleFOCDebug::print("Welcome to Twisted Fields RP2040 firmware, v");
  SimpleFOCDebug::println(FIRMWARE_VERSION);  

  // configure motor drivers, 6-PWM
  driver0.voltage_power_supply = DEFAULT_VOLTAGE_LIMIT;
  driver1.voltage_power_supply = DEFAULT_VOLTAGE_LIMIT;
  driver0.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  driver1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  SimpleFOCDebug::println("Initializing driver 0...");
  if (!driver0.init())
    SimpleFOCDebug::println("Driver 0 init failed!");
  SimpleFOCDebug::println("Initializing driver 1...");
  if (!driver1.init())
    SimpleFOCDebug::println("Driver 1 init failed!"); 

  sensor0.init();
  sensor0.enableInterrupts(handleA0, handleB0, handleZ0);
  sensor1.init();
  sensor1.enableInterrupts(handleA1, handleB1, handleZ1);

  motor0.voltage_limit = motor1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  motor0.velocity_limit = motor1.velocity_limit = 200.0f; // 200rad/s is pretty fast
  motor0.PID_velocity.P = motor1.PID_velocity.P = 0.2f;
  motor0.PID_velocity.I = motor1.PID_velocity.I = 2.0f;
  motor0.PID_velocity.D = motor1.PID_velocity.D = 0.0f;
  motor0.PID_velocity.output_ramp = motor1.PID_velocity.output_ramp = 200.0f;
  motor0.PID_velocity.limit = motor1.PID_velocity.limit = DEFAULT_VOLTAGE_LIMIT; // TODO check this
  motor0.P_angle.P = motor1.P_angle.P = 20.0f;
  motor0.LPF_velocity.Tf = motor1.LPF_velocity.Tf = 0.05f;
  motor0.foc_modulation = motor1.foc_modulation = FOCModulationType::SinePWM;
  motor0.controller = motor1.controller = MotionControlType::velocity;
  motor0.torque_controller = motor1.torque_controller = TorqueControlType::voltage;

  motor0.motion_downsample = motor1.motion_downsample = 4;
  motor1.motion_cnt = 2; // stagger the calls to move()

  motor0.target = motor1.target = 1.0f; // TODO set to 0.0f

  if (driver0.initialized) {
    motor0.init();
    SimpleFOCDebug::println("Initializing current sense 0...");
    // if (current0.init()!=1)
    //   SimpleFOCDebug::println("Current sense 0 init failed!");
    // else
    //   motor0.linkCurrentSense(&current0);
    if (motor0.motor_status==FOCMotorStatus::motor_uncalibrated) {
      motor0.linkSensor(&sensor0);
      SimpleFOCDebug::println("Initializing FOC motor 0...");
      motor0.initFOC();
      SimpleFOCDebug::println("Motor 0 intialized.");
    }
    else
      SimpleFOCDebug::println("Motor 0 init failed!");
  }
  else
    SimpleFOCDebug::println("Motor 0 not initialized.");

  if (driver1.initialized) {
    motor1.init();
    // SimpleFOCDebug::println("Initializing current sense 1...");
    // if (current1.init()!=1)
    //   SimpleFOCDebug::println("Current sense 1 init failed!");
    // else
    //   motor1.linkCurrentSense(&current1);
    if (motor1.motor_status==FOCMotorStatus::motor_uncalibrated) {
        motor1.linkSensor(&sensor1);
        SimpleFOCDebug::println("Initializing FOC motor 1...");
        motor1.initFOC();
        SimpleFOCDebug::println("Motor 1 intialized.");
    }
    else
      SimpleFOCDebug::println("Motor 1 init failed!");
  }
  else
    SimpleFOCDebug::println("Motor 1 not initialized.");

  commander.add('M',onMotor0, "Motor 0");
  commander.add('N',onMotor1, "Motor 1");

  SimpleFOCDebug::println("Startup complete.");
  ts = millis();  

}



void loop() {
  if (motor0.motor_status==FOCMotorStatus::motor_uncalibrated || motor0.motor_status==FOCMotorStatus::motor_ready)
    motor0.move();
  if (motor1.motor_status==FOCMotorStatus::motor_uncalibrated || motor1.motor_status==FOCMotorStatus::motor_ready)
    motor1.move();
  if (motor0.motor_status==FOCMotorStatus::motor_ready)
    motor0.loopFOC();
  if (motor1.motor_status==FOCMotorStatus::motor_ready)
    motor1.loopFOC();

  count++;
  if (ts + 1000uL < millis()) {
    ts = millis();
    SimpleFOCDebug::print("loop/s: ");
    SimpleFOCDebug::println(count);
    count = 0;
  }

  commander.run();
}