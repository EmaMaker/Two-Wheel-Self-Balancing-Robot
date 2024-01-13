#include <ArduPID.h>

#define MOT_DX 0
#define MOT_SX 1

constexpr float MOT_SX_MULT = 1.0;
constexpr float MOT_DX_MULT = 1.0;

ArduPID angleCtrl, speedCtrl;

// Calculated with matlab, then adjusted by hand

/* With KP = 10, about  12 periods of oscillation in 1sec
Pc = 0.083

PID Kp = 0.6Kc, Ti = 0.5Pc Td = 0.125Pc
Ki = KpTc/ti
Kd = KpTd/Tc
*/ 

constexpr double kponkd = 935.9697;
constexpr double kionkd = 1.3189e+04;
constexpr double KD = 0.1;
constexpr double KP = 5.8;
constexpr double KI = 0.2;

//double setpoint = -0.015;
double setpoint = 0.0;
double output = 0;
double input = 0;

double speed_setpoint = 0.0;
double speed_output = 0.0;
double speed_input = 0.0;

double roll{ 0 }, pitch{ 0 }, yaw{ 0 };

constexpr double max_torque_sx = 0.392;
constexpr double max_torque_dx = 0.36;
constexpr double max_torque = max_torque_dx+max_torque_sx;
constexpr double torque_sx_coeff = max_torque_sx/max_torque;
constexpr double torque_dx_coeff = max_torque_dx/max_torque;

void setup(void) {
  Serial.begin(9600);
  delay(500);

  setup_imu();
  setup_motors();
 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Let the initial error from madgwick filter discharge without affecting the integral term of the PID
  unsigned long t = millis();
  while (millis() - t < 3000) {
    update_imu();
  }

  angleCtrl.begin(&input, &output, &setpoint, KP, KI, KD, P_ON_E, FORWARD);
  angleCtrl.setOutputLimits(-max_torque, max_torque);  // double of max torque motors can exhert
  // angleCtrl.setWindUpLimits(-0.2 , 0.02);
  angleCtrl.setSampleTime(1);
  angleCtrl.start();

  speedCtrl.begin(&speed_input, &speed_output, &speed_setpoint, 0, 0.12, 0, P_ON_E, BACKWARD);
  speedCtrl.setOutputLimits(-max_torque, max_torque);  // double of max torque motors can exhert
  speedCtrl.setSampleTime(1);
  speedCtrl.start();



  digitalWrite(LED_BUILTIN, LOW);
}

double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  compute();
}

void compute(){
  static unsigned long last_time;
  unsigned long t = millis();
  unsigned long dt = millis() - t;
  last_time = t;

  speedCtrl.compute();
  Serial.print(speed_input);
  Serial.print(" ");
  Serial.println(speed_output);

  update_imu();
  input = speed_output-pitch;

  angleCtrl.compute();

  double torque = abs(output);

  double s = sign(output);
  double pwm_dx = torque <= 0.005 ? 30*s : torque_to_pwm_dx(torque*torque_dx_coeff)*s;
  double pwm_sx = torque <= 0.005 ? 30*s : torque_to_pwm_sx(torque*torque_sx_coeff)*s;

  move_pwm(MOT_SX, pwm_sx);
  move_pwm(MOT_DX, pwm_dx);


  speed_input = torque*0.5*s;
}
