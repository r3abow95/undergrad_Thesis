/*
*  Linear Control routine for Inverted Pendulum
* @Author: Alvin Reabow [RBWALV001]
* @Date: 2017/11/12
* =========================================================================
*/

#define _BV(bit) (1 << (bit))

const byte ChannelA1 = 19;
const byte ChannelB1 = 18;

const byte ChannelA2 = 20;
const byte ChannelB2 = 21;


volatile long pend_count = 0;
volatile long arm_count = 0;
static int8_t lookup_table[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};


//model parameters for the energy swing-up
const double m = 0.009;
const double l = 0.225;
const double J = 2.0963e-4;
const double g = 9.81;
const double pi = 3.14159265359;

//states
double theta, theta_dot, phi, phi_dot, w;
double theta0 = 0;
double phi0 = 0;
double w0 = 0;

//controls
double u;
long PWM;


//debugging
int i = 0;

//
volatile int previous_time = millis();
volatile int current_time;

void setup() {

  //Initialise serial communication for sending data to PC
  Serial.begin(9600);
  delay(5);
  //Initialize interrupts for the encoders
  //Interrupt Setup for Pendulum
  pinMode(ChannelA1, INPUT_PULLUP);
  pinMode(ChannelB1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ChannelA1), pend_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ChannelB1), pend_encoder_isr, CHANGE);

  //Interrupt Setup for Arm
  pinMode(ChannelA2, INPUT_PULLUP);
  pinMode(ChannelB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ChannelA2), arm_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ChannelB2), arm_encoder_isr, CHANGE);
  //Initialise PWM
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(6, HIGH);
  digitalWrite(5, HIGH);
  analogWrite(7, 0);
}
//Controller Gains & Deadzone
float k1 = -0.1;
float k2 = -0.1;
float k3 = -100;
float k4 = -2;
float ki = -2;
int deadzone = 60;

void loop() {
  current_time = millis();
  int delta_t = current_time - previous_time;
  if (delta_t >= 25) {
    theta = scale(pend_count / 1024.0 * 2 * pi);
    phi = scale2((arm_count * 16.0 / 13.0) / 1024.0 * 2 * pi);
    theta_dot = (theta - theta0) / 0.025;
    phi_dot = (phi - phi0) / 0.025;

    if (abs(theta) <= 0.8) {
      w = ki * (0 - theta) + w0;
      u =  w - (k1 * phi + k2 * phi_dot + k3 * (theta) + k4 * theta_dot);

    }
    else {
      u = SwingUp(theta, theta_dot, 500, 2);
    }
    theta0 = theta;
    phi0 = phi;
    w = w0;
    previous_time = current_time;
    updateMotor(u);

    //Print system to the serial port for reading of system data
    Serial.print(theta);
    Serial.print(' ');
    Serial.print(phi);
    Serial.print(' ');
    Serial.print(phi_dot);
    Serial.print(' ');
    Serial.print(theta_dot);
    Serial.print(' ');
    Serial.println(u);
  }
}
/*
 * The following function applies the enrgy based swing-up control
 * @param theta, theta_dot, k, n
 * @return control signal u
 */
double SwingUp(double theta, double theta_dot, double k, double n) {

  double E = 0.5 * J * sq(theta_dot) + m * g * l * (cos(theta) - 1);
  if (theta_dot * cos(theta) >= 0) {
    u = 1;
  }
  else {
    u = -1;
  }
  u *= (E - 0) * k;
  if (u > n * g) {
    u = n * g;
  }
  if (u < n * g) {
    u = -n * g;
  }
  return u;
}
/*
 * Pendulum ISR for determining the pendulum encoder position
 */
void pend_encoder_isr() {
  static uint8_t pend_val = 0;
  pend_val = (pend_val << 2);
  pend_val |= ((PIND & 0b1100 ) >> 2);
  pend_count += lookup_table[pend_val & 0b1111];
}
/*
 * ISR for determining the arm encoder position
 */
void arm_encoder_isr() {
  static uint8_t arm_val = 0;
  arm_val = (arm_val << 2);
  arm_val |= ((PIND & 0b0011 ));
  arm_count += lookup_table[arm_val & 0b1111];
}

/*
 * Function to update the mtor speed and position
 * @param u, the input control sugnal to convert to a direction and PWM value
 */
void updateMotor(double u) {
  PWM = abs(u) * 255.0 / 18.0;
  PWM += deadzone;
  if (PWM > 255) {
    PWM = 255;
  }
  analogWrite(7, PWM);
  if (u > 0) {
    digitalWrite(6, LOW);
    digitalWrite(5, HIGH);
  }
  else if (u < 0) {
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
  }
  else {
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
  }
}

/*
 * Scaling function to ensure that the pendulum and arm position are between [-pi,pi]
 */
double scale2(double angle) {
  while (angle > pi) {
    angle -= 2 * pi;
  }
  while (angle < -pi) {
    angle += 2 * pi;
  }
  return -1 * angle;
}
double scale(double angle) {
  //angle += pi;
  while (angle > pi) {
    angle -= 2 * pi;
  }
  while (angle < -pi) {
    angle += 2 * pi;
  }
  return -1 * angle;
}
//===========================================================================

