/*
*  Data Acquisition routine for Inverted Pendulum
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
static int8_t lookup_table[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

volatile int previous_time = millis();
volatile int current_time;


void setup() {
  
  //Initialise serial communication for sending data to PC
  Serial.begin(9600);

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

  //set up PWM
  DDRB |= ( _BV(4) | _BV(5) );       //set digital pins 10 & 11 to output
  PORTB &= ~( _BV(4) | _BV(5) );     //mask pins, i.e make low       
  //setup PWM
  // Set PB6 as outputs. [**Digital pin 12 on Arduino Mega!**]
  DDRB |= (1 << DDB6);
  TCCR1A &= 0;
  TCCR1A |= (1<<WGM11) | (1<<COM1B1);
  TCCR1B &= 0;
  TCCR1B |= (1 << WGM13) | (1 << WGM12);
  TCCR1B &= ~0x07;
  TCCR1B |=  0x01;
  ICR1 = 255;                     //note that increasing the number of bits of the timer increases the clk prescalar?
  OCR1B = 0;


}


void loop() {
  sample(25);  
  
}

//Send system measurements over serial port
void sample(int sample){
  
  if (checkSample(sample)){
      read_input();
      Serial.print(pend_count,DEC);
      Serial.print(' ');
      Serial.print(arm_count,DEC);
      Serial.println("");
      
  }
}
//Reads serial data command sent to Arduino
void read_input(){
  if (Serial.available()>0){
    int dir = Serial.parseInt();
    
    int duty = Serial.parseInt();
    
    if (Serial.read() == '\n'){
      updateMotor(dir, duty);
    }
  }
}

void pend_encoder_isr(){
  static uint8_t pend_val = 0;
  pend_val = (pend_val << 2);
  pend_val |= ((PIND & 0b1100 ) >> 2);
  pend_count += lookup_table[pend_val & 0b1111];
}

void arm_encoder_isr(){
  static uint8_t arm_val = 0;
  arm_val = (arm_val << 2);
  arm_val |= ((PIND & 0b0011 ));
  arm_count += lookup_table[arm_val & 0b1111];
}

boolean checkSample(int sampleRate){
  current_time = millis();
  int delta_time = current_time - previous_time;
  if (delta_time >= sampleRate){
    previous_time = current_time;
    return true;   
  }
  return false;
}

/*
*	This function changes the motor speed and direction
*	THe following structure is used:
*		Send data over serial port as "Dir PWM\n"
*		Where => 'Dir' is the motor direction
					- 0 = clockwise
					- 1 = aniticlockwise
			  => 'PWM' is the integert PWM value to be written for motor speed
			  => use new line character to terminate the command
*/
void updateMotor(int dir, int duty){
  OCR1B = duty;                      //set duty cycle
  if (dir == 0){
    PORTB &= ~( _BV(4) | _BV(5) ); //clear direction pins
    PORTB |= ( _BV(5) );          //set motor in clockwise direction
  }
  else if (dir == 1){
    PORTB &= ~( _BV(4) | _BV(5) ); //clear direction pins
    PORTB |= ( _BV(4) );          //set motor in anti-clockwise direction
  }
  else{
    PORTB &= ~( _BV(4) | _BV(5) ); //brake; the direction received was invalid
  }
}




