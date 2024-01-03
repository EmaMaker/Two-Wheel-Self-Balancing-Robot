
int mot_pins[][3] = {
  {17, 16, 20},
  {19, 18, 21}
};


// PWM value at which the motors start moving (empirical)
constexpr int base_pwm = 100  ; // Actually starts moving at 70
// Max PWM value
constexpr int max_pwm = 255;

void setup_motors(){
  pinMode(mot_pins[0][0], OUTPUT);
  pinMode(mot_pins[0][1], OUTPUT);
  pinMode(mot_pins[1][0], OUTPUT);
  pinMode(mot_pins[1][1], OUTPUT);

  Serial.println("Motors up");
}

double torque_to_pwm_sx(double t){
  return 547.323 * t + 50.874;
}

double torque_to_pwm_dx(double t){
  return 684.284 * t + 44.29;
}

double speed_to_pwm(double speed){
  return map(speed, 0, 100, 0, 255);
}

int sign(auto b){
  return b == 0 ? 0 : b > 0 ? 1 : -1;
}

void move(int mot, double speed){
  double pwm = speed_to_pwm(speed);
  move_pwm(mot, pwm*sign(speed));
}

void move_pwm(int mot, int pwm){
  if(pwm == 0){
    digitalWrite(mot_pins[mot][0], LOW);
    digitalWrite(mot_pins[mot][1], LOW);
  }else if(pwm < 0){
    digitalWrite(mot_pins[mot][0], HIGH);
    digitalWrite(mot_pins[mot][1], LOW);
  }else{
    digitalWrite(mot_pins[mot][1], HIGH);
    digitalWrite(mot_pins[mot][0], LOW);
  }

  analogWrite(mot_pins[mot][2], abs(pwm));
}

void test_motors(){
  while(!Serial) delay(10);

  int m = 0;
  while(true){
    digitalWrite(mot_pins[m][0], LOW);
    digitalWrite(mot_pins[m][1], HIGH);
    
    for(int i = 0; i < 255; i++){
      Serial.print("PWM: ");
      Serial.println(i);
      analogWrite(mot_pins[m][2], i);
      delay(100);
    }
    delay(1000);
    digitalWrite(mot_pins[m][0], LOW);
    digitalWrite(mot_pins[m][1], LOW);
    m = 1-m;
  }
}

void test_motors_nodeadzone(){
  int m = 0;
  while(true){
    for(int i = 0; i < 100; i++){
      Serial.print("Speed: ");
      Serial.println(i);
      move(m, i);
      delay(100);
    }
    delay(1000);
    move(m, 0);
    m = 1-m;
  }
}

void test_motors_rpm(){
  while(!Serial) delay(10);

  unsigned long tstart = millis();
  while(millis() - tstart < 200) {
  move(0, 20);
  }
  move(0,0);
}

void test_motors_diff(){
  move(MOT_SX, 50*MOT_SX_MULT);
  move(MOT_DX, 50*MOT_DX_MULT);
}


void test_motors_torque(int motor, int dir){
  while(!Serial){;}
  
  while(1){
    delay(1000);
    Serial.println("Setup, waiting 8 secs");
    delay(8000);
    
  for(int i = 0; i <= 255; i+= 15){
    digitalWrite(mot_pins[motor][1], dir > 0 ? LOW : HIGH);
    digitalWrite(mot_pins[motor][0], dir > 0 ? HIGH : LOW);
    Serial.print("PWM: ");
    Serial.println(i);

    analogWrite(mot_pins[motor][2], i);
    delay(8000);
    
    digitalWrite(mot_pins[motor][0], LOW);
    digitalWrite(mot_pins[motor][1], LOW);
    delay(2000);
    
    }
  }
}
