#include <ArduPID.h>

#define MOT_DX 0
#define MOT_SX 1

constexpr float MOT_SX_MULT = 1.0;
constexpr float MOT_DX_MULT = 1.0;

ArduPID myController;

// Calculated with matlab
constexpr double KP = 1;
constexpr double KI = 0.02;
constexpr double KD = 0.05;

double setpoint = 0;
double output = 0;
double input = 0;

double setpoint_front = 0;
double output_front;

double angleX{0}, angleY{0}, angleZ{0};
double angAccX{0}, angAccY{0};
double accX{0}, accY{0};

void setup(void) {
  Serial.begin(9600);
  delay(500);
  
  setup_imu(5);;
  Serial.println("IMU up");
  setup_motors();
  Serial.println("Motors up");
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(200);
   
  myController.begin(&input, &output, &setpoint, KP, KI, KD, P_ON_E, FORWARD);
  myController.setOutputLimits(-0.4, 0.4); // max torque motors can exhert
  myController.setSampleTime(2);
  myController.start();

  delay(200);
  digitalWrite(LED_BUILTIN, LOW);

  test_motors_torque(1);
  /*frontController.begin(&angleZ, &output_front, &setpoint_front, 100, 0, 1, P_ON_E, FORWARD);
  frontController.setOutputLimits(-100,100);  
  myController.setWindUpLimits(-PI, PI);
  frontController.setSampleTime(1);
  frontController.start();*/

}

double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned long t = 0;
double oldpwm = 0;

void loop(){
  computeAngles();

  //static double oldAngleY = 0;
  //oldAngleY = 0.99 * oldAngleY + 0.01 * angleY;
  input = angleY;
  
  
  myController.compute();

  double torque = abs(output) * 0.5;
  double pwm = map_double(torque, 0, 0.2, 80, 255);
  pwm *= sign(output);

  Serial.println("input\toutput\ttorque\tpwm");
  Serial.print(input, 6);
  Serial.print("\t");
  Serial.print(output, 6);
  Serial.print("\t");
  Serial.print(torque);
  Serial.print("\t");
  Serial.println(pwm);
  
  move_pwm(MOT_SX, pwm*MOT_SX_MULT);
  move_pwm(MOT_DX, pwm*MOT_DX_MULT);

  oldpwm = pwm;
}
