#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "./hw_setup.h"
#include "./led_signals.h"
#include "./motor_utils.h"
#include "gpio.h"

#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

/*
 IMPORTANT: Twisted fields controller uses active-low polarity for low-side switches! 
 Be sure to set -DSIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH=false in the build.
 If you don't do this, you'll be producing guaranteed shoot-through, and will probably burn out the driver board.
*/
#if not(defined(SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH)) || (SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH == true)
#error "Please set -DSIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH=false in the build."
#endif



#define SERIAL_SPEED 115200
#define FIRMWARE_VERSION "0.1"

#define DEFAULT_VOLTAGE_POWER_SUPPLY 15.0f
#define DEFAULT_VOLTAGE_LIMIT 3.0f
#define ALIGNMENT_VOLTAGE_LIMIT 3.0f

#define MOTOR_PP 7
#define MOTOR_KV NOT_SET
//#define MOTOR_PHASE_RESISTANCE 0.2f
#define MOTOR_PHASE_RESISTANCE NOT_SET




// motor 0
// Encoder sensor0 = Encoder(ENCODER0_A_PIN, ENCODER0_B_PIN, 1024, ENCODER0_Z_PIN);
// void handleA0() { sensor0.handleA(); };
// void handleB0() { sensor0.handleB(); };
// void handleZ0() { sensor0.handleIndex(); };
//arduino::MbedSPI sensorSPI(digitalPinToPinName(3), SPI_MOSI, SPI_SCK);
MagneticSensorMT6701SSI sensor0 = MagneticSensorMT6701SSI(SENSOR0_CS_PIN);
BLDCDriver6PWM driver0 = BLDCDriver6PWM(M0_INUH_PIN, M0_INUL_PIN, M0_INVH_PIN, M0_INVL_PIN, M0_INWH_PIN, M0_INWL_PIN);
//LowsideCurrentSense current0 = LowsideCurrentSense(1.0f, 1.0f/CURRENT_VpA, M0_AOUTU_PIN, M0_AOUTV_PIN, NOT_SET);
InlineCurrentSense current0 = InlineCurrentSense(1.0f, 1.0f/CURRENT_VpA, M0_AOUTU_PIN, M0_AOUTV_PIN, NOT_SET);
BLDCMotor motor0 = BLDCMotor(MOTOR_PP, MOTOR_PHASE_RESISTANCE, MOTOR_KV);

// motor 1
Encoder sensor1 = Encoder(ENCODER1_A_PIN, ENCODER1_B_PIN, 1024); //, ENCODER1_Z_PIN);
void handleA1() { sensor1.handleA(); };
void handleB1() { sensor1.handleB(); };
void handleZ1() { sensor1.handleIndex(); };
BLDCDriver6PWM driver1 = BLDCDriver6PWM(M1_INUH_PIN, M1_INUL_PIN, M1_INVH_PIN, M1_INVL_PIN, M1_INWH_PIN, M1_INWL_PIN);
//LowsideCurrentSense current1 = LowsideCurrentSense(1.0f, 1.0f/CURRENT_VpA, NOT_SET, M1_AOUTV_PIN, M1_AOUTW_PIN);
InlineCurrentSense current1 = InlineCurrentSense(1.0f, 1.0f/CURRENT_VpA, NOT_SET, M1_AOUTV_PIN, M1_AOUTW_PIN);
BLDCMotor motor1 = BLDCMotor(MOTOR_PP, MOTOR_PHASE_RESISTANCE, MOTOR_KV);

// Commander
Commander commander = Commander(Serial, '\n', false);
void onMotor0(char* cmd){commander.motor(&motor0, cmd);}
void onMotor1(char* cmd){commander.motor(&motor1, cmd);}
void onUtil(char* cmd){dispatch_util(cmd, &motor0, &motor1);}

// LEDs
LEDSignals leds = LEDSignals();

// main loop speed tracking
int count = 0;
unsigned long ts = 0;


void setup() {
  // setup LEDs
  leds.init(50);
  // initialize serial port on USB, debug output goes to this
  Serial.begin(SERIAL_SPEED);
  SimpleFOCDebug::enable();
  delay(1000);
  while (!Serial) ; // wait for serial port to connect - remove this later
  SimpleFOCDebug::print("Welcome to Twisted Fields RP2040 firmware, v");
  SimpleFOCDebug::println(FIRMWARE_VERSION);  

  // configure motor drivers, 6-PWM
  driver0.voltage_power_supply = DEFAULT_VOLTAGE_POWER_SUPPLY;
  driver1.voltage_power_supply = DEFAULT_VOLTAGE_POWER_SUPPLY;
  driver0.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  driver1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  SimpleFOCDebug::println("Initializing driver 0...");
  if (!driver0.init())
    SimpleFOCDebug::println("Driver 0 init failed!");
  motor0.linkDriver(&driver0);
  SimpleFOCDebug::println("Initializing driver 1...");
  if (!driver1.init())
    SimpleFOCDebug::println("Driver 1 init failed!"); 
  motor1.linkDriver(&driver1);


  sensor0.init();
  gpio_set_function(LED_WS2812_PIN, GPIO_FUNC_PIO0); //setting this makes NeoPixel work but SPI fail :-(
  //sensor0.enableInterrupts(handleA0, handleB0, handleZ0);
  sensor1.pullup = Pullup::USE_INTERN;
  sensor1.init();
  sensor1.enableInterrupts(handleA1, handleB1, handleZ1);
  leds.signalInitState(1);

  motor0.voltage_limit = motor1.voltage_limit = DEFAULT_VOLTAGE_LIMIT / 2.0f;
  motor0.voltage_sensor_align = motor1.voltage_sensor_align = ALIGNMENT_VOLTAGE_LIMIT;
  motor0.velocity_limit = motor1.velocity_limit = 200.0f; // 200rad/s is pretty fast
  motor0.PID_velocity.P = motor1.PID_velocity.P = 0.2f;
  motor0.PID_velocity.I = motor1.PID_velocity.I = 0.6f;
  motor0.PID_velocity.D = motor1.PID_velocity.D = 0.0f;
  motor0.PID_velocity.output_ramp = motor1.PID_velocity.output_ramp = 200.0f;
  //motor0.PID_velocity.limit = motor1.PID_velocity.limit = DEFAULT_VOLTAGE_LIMIT; // TODO check this
  motor0.P_angle.P = motor1.P_angle.P = 20.0f;
  motor0.LPF_velocity.Tf = motor1.LPF_velocity.Tf = 0.05f;
  motor0.foc_modulation = motor1.foc_modulation = FOCModulationType::SinePWM;
  motor0.controller = motor1.controller = MotionControlType::velocity;
  motor0.torque_controller = motor1.torque_controller = TorqueControlType::voltage;

  current0.skip_align = true;
  motor0.PID_current_q.P = 5;
  motor0.PID_current_q.I = 1000;
  motor0.PID_current_q.D = 0;
  motor0.PID_current_q.limit = motor0.voltage_limit; 
  motor0.PID_current_q.output_ramp = 1e6;
  motor0.LPF_current_q.Tf= 0.005;
  motor0.PID_current_d.P = 5;
  motor0.PID_current_d.I = 1000;
  motor0.PID_current_d.D = 0;
  motor0.PID_current_d.limit = motor0.voltage_limit; 
  motor0.PID_current_d.output_ramp = 1e6;
  motor0.LPF_current_d.Tf= 0.005;

  motor0.motion_downsample = motor1.motion_downsample = 4;
  motor1.motion_cnt = 2; // stagger the calls to move()

  motor0.target = motor1.target = 0.0f;

  if (driver0.initialized) {
    motor0.init();
    SimpleFOCDebug::println("Initializing current sense 0...");
    current0.linkDriver(&driver0);
    if (current0.init()!=1)
      SimpleFOCDebug::println("Current sense 0 init failed!");
    else {
      // SimpleFOCDebug::println("Found offset A ", current0.offset_ia);
      // SimpleFOCDebug::println("Found offset B ", current0.offset_ib);
      // SimpleFOCDebug::println("Found offset C ", current0.offset_ic);
      motor0.linkCurrentSense(&current0);
    }
    leds.signalInitState(2);
    if (motor0.motor_status==FOCMotorStatus::motor_uncalibrated) {
      motor0.linkSensor(&sensor0);
      SimpleFOCDebug::println("Initializing FOC motor 0...");
      if (motor0.initFOC())
        SimpleFOCDebug::println("Motor 0 ready for closed loop.");
      else
        SimpleFOCDebug::println("Motor 0 ready for open loop.");        
      leds.signalInitState(3);
    }
    else
      SimpleFOCDebug::println("Motor 0 init failed!");
  }
  else
    SimpleFOCDebug::println("Motor 0 not initialized.");

  if (driver1.initialized) {
    motor1.init();
    // current1.linkDriver(&driver1);
    // SimpleFOCDebug::println("Initializing current sense 1...");
    // if (current1.init()!=1)
    //   SimpleFOCDebug::println("Current sense 1 init failed!");
    // else
    //   motor1.linkCurrentSense(&current1);
    if (motor1.motor_status==FOCMotorStatus::motor_uncalibrated) {
        //motor1.linkSensor(&sensor1);
        //motor1.linkSensor(&sensor0);
        SimpleFOCDebug::println("Initializing FOC motor 1...");
        if (motor1.initFOC())
          SimpleFOCDebug::println("Motor 1 ready for closed loop.");
        else
          SimpleFOCDebug::println("Motor 1 ready for open loop.");
    }
    else
      SimpleFOCDebug::println("Motor 1 init failed!");
  }
  else
    SimpleFOCDebug::println("Motor 1 not initialized.");

  commander.add('M', onMotor0, "Motor 0");
  commander.add('N', onMotor1, "Motor 1");
  commander.add('U', onUtil, "Motor utilities");

  SimpleFOCDebug::println("Startup complete.");
  ts = millis();  
  leds.signalInitState(3);
}

extern volatile int rp2040_intcount;
int last_intcount = 0;

void loop() { 

  if (motor0.motor_status==FOCMotorStatus::motor_uncalibrated 
      || motor0.motor_status==FOCMotorStatus::motor_calib_failed 
      || motor0.motor_status==FOCMotorStatus::motor_ready)
    motor0.move();

  if (motor1.motor_status==FOCMotorStatus::motor_uncalibrated 
      || motor1.motor_status==FOCMotorStatus::motor_calib_failed 
      || motor1.motor_status==FOCMotorStatus::motor_ready)
    motor1.move();

  if (motor0.motor_status==FOCMotorStatus::motor_ready)
    motor0.loopFOC();
  else
    sensor0.update();

  if (motor1.motor_status==FOCMotorStatus::motor_ready)
    motor1.loopFOC();
  else
    sensor1.update();

  count++;
  if (ts + 1000uL < millis()) {
    ts = millis();
    // SimpleFOCDebug::print("loop/s: ");
    // SimpleFOCDebug::print(count);
    // SimpleFOCDebug::print(" a0: ");
    // SimpleFOCDebug::print(sensor0.getAngle());
    // SimpleFOCDebug::print(" v0: ");
    // SimpleFOCDebug::print(sensor0.getVelocity());
    // SimpleFOCDebug::print(" c0a: ");
    // SimpleFOCDebug::print(current0.getPhaseCurrents().a);
    // SimpleFOCDebug::print(" c0b: ");
    // SimpleFOCDebug::print(current0.getPhaseCurrents().b);
    // SimpleFOCDebug::print(" cnt: ");
    // SimpleFOCDebug::println(rp2040_intcount-last_intcount);
    Serial.print(current0.getPhaseCurrents().a, 4);
    Serial.print(" ");
    Serial.print(current0.getPhaseCurrents().b, 4);
    Serial.print(" ");
    Serial.println(current0.getPhaseCurrents().c, 4);
    last_intcount=rp2040_intcount;
    count = 0;
  }

  leds.signalInitState(1);

  commander.run();
}