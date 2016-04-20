
/*    MR ROBOT - SCORBOT-ER III Robot Controller Arduino implementation
 *    
 *    Authors: Pete Sterrantino
 *   
 *  
 */

int encoder1_count;  // int to allow negatives
int encoder2_count;
int encoder3_count;
int encoder4_count;
int encoder5_count;
int encoder6_count;

int motor1_PWM_pin = 4;
int motor2_PWM_pin = 5;
int motor3_PWM_pin = 6;
int motor4_PWM_pin = 7;
int motor5_PWM_pin = 8;
int motor6_PWM_pin = 9;

#define forward 1
#define reverse 0


void setup() {
  // setup code here, to run once:
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // prints title with ending line break
  Serial.println("Mr Robot - Control Software");
  Serial.println("Initializing hardware...");

  // initialize PWM pins as outputs.
  pinMode(4, OUTPUT); // motor 1 pwm output
  pinMode(5, OUTPUT); // motor 2 pwm output
  pinMode(6, OUTPUT); // motor 3 pwm output
  pinMode(7, OUTPUT); // motor 4 pwm output
  pinMode(8, OUTPUT); // motor 5 pwm output
  pinMode(9, OUTPUT); // motor 6 pwm output

  pinMode(22, OUTPUT); // motor 1 IN1 output  Motor control H-bridge #1
  pinMode(23, OUTPUT); // motor 1 IN2 output
  pinMode(24, OUTPUT); // motor 2 IN3 output
  pinMode(25, OUTPUT); // motor 2 IN4 output
  
  pinMode(26, OUTPUT); // motor 3 IN1 output  Motor control H-Bridge #2
  pinMode(27, OUTPUT); // motor 3 IN2 output
  pinMode(28, OUTPUT); // motor 4 IN3 output
  pinMode(29, OUTPUT); // motor 4 IN4 output
  
  pinMode(30, OUTPUT); // motor 5 IN1 output  Motor control H-bridge #3 
  pinMode(31, OUTPUT); // motor 5 IN2 output
  pinMode(32, OUTPUT); // motor 6 IN3 output
  pinMode(33, OUTPUT); // motor 6 IN4 output
  
  
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);  // On board Led
}



/*Function Declarations */
  
/*  -------------------- motor1_forward - moves motor 1 in forward direction ---------------
 *  Arguments:
 *  direction = forward or reverse
 *  speed = 0 - 255 (0 same as stop) 
 *  pulses = number of encoder pulses to move motor 
 */
void motor1_control(int direction, int speed, int pulses) {  
  // limit inputs
  if (speed > 255) {speed = 255; }
  if (speed < 0) {speed = 0; }

  if ((direction != forward) && (direction != reverse)){
    return;  //do nothing
  }
  
  // set forward pins on motor controller
  if (direction == forward) {
    digitalWrite(22, LOW);  
    digitalWrite(23, HIGH); 
    Serial.println("Motor 1 forward");
  }

  // set reverse pins on motor controller
  if (direction == reverse) {
    digitalWrite(22, HIGH);  
    digitalWrite(23, LOW); 
    Serial.println("Motor 1 reverse");
  }

  //set PWM value as speed
  analogWrite(motor1_PWM_pin, speed);
  Serial.print("Speed set to: ");
  Serial.println(speed);

  //set number of encoder pulses 
  encoder1_count = pulses;    //decremeted in ISR
  
}

/*  --------------------- motor1_stop - stop motor 1 ------------------------
 */
void motor1_stop() {  
  analogWrite(motor1_PWM_pin, 0);
  digitalWrite(22, LOW);  
  digitalWrite(23, LOW); 
  Serial.println("Motor 1 stopped");
}
  



void loop() {
  // put your main code here, to run repeatedly:
  
  
  motor1_control(forward, 25, 1000);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second

  motor1_stop();
  delay(1000);              // wait for a second

  motor1_control(reverse, 25, 1000);
  delay(1000);              // wait for a second

  motor1_stop();            // stop the motor at the end

  //motor monitor
  if (encoder1_count <= 0) {
    encoder1_count = 0;
    motor1_stop();
  }
  if (encoder2_count <= 0) {
    //encoder2_count = 0;
    //motor2_stop();
  }
  if (encoder3_count <= 0) {
    //encoder3_count = 0;
    //motor3_stop();
  }
  if (encoder4_count <= 0) {
    //encoder4_count = 0;
    //motor4_stop();
  }
  if (encoder5_count <= 0) {
    //encoder5_count = 0;
    //motor5_stop();
  }
  if (encoder6_count <= 0) {
    //encoder6_count = 0;
    //motor6_stop();
  }


  
  
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second

  
}
