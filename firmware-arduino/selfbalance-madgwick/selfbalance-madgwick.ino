#include <ArduPID.h>

#define MOT_DX 0
#define MOT_SX 1

constexpr float MOT_SX_MULT = 1.0;
constexpr float MOT_DX_MULT = 1.0;

ArduPID myController;

// Calculated with matlab, then adjusted by hand
// Decent values?
/*constexpr double KP = 2.5;
constexpr double KI = 0.04;
constexpr double KD = 0.03;*/

// Event better
constexpr double KP = 4;
constexpr double KI = 5;
constexpr double KD = 0.1;

//double setpoint = -0.015;
double setpoint = 0.0;
double output = 0;
double input = 0;

double roll{ 0 }, pitch{ 0 }, yaw{ 0 };

void setup(void) {
  Serial.begin(9600);
  delay(500);

  setup_imu();
  setup_motors();
 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  myController.begin(&input, &output, &setpoint, KP, KI, KD, P_ON_E, FORWARD);
  myController.setOutputLimits(-0.72, 0.72);  // double of max torque motors can exhert
  myController.setWindUpLimits(-0.2, 0.02);
  myController.setSampleTime(1);
  myController.start();

  // Let the initial error from madgwick filter discharge without affecting the integral term of the PID
  unsigned long t = millis();
  while (millis() - t < 3000) {
    compute();
  }

  move_pwm(MOT_SX, 0);
  move_pwm(MOT_DX, 0);
  myController.reset();
  delay(1000);

  digitalWrite(LED_BUILTIN, LOW);
}

double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  compute();
}

void compute(){
  update_imu();

  input = pitch;

  myController.compute();

  double torque = abs(output) * 0.5;

  double s = sign(output);
  double pwm_dx = torque <= 0.005 ? 30*s : torque_to_pwm_dx(torque)*s;
  double pwm_sx = torque <= 0.005 ? 30*s : torque_to_pwm_sx(torque)*s;

  move_pwm(MOT_SX, pwm_sx);
  move_pwm(MOT_DX, pwm_dx);
}
