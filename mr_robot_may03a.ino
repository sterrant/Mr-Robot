
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

const int motor1_PWM_pin = 4;
const int motor2_PWM_pin = 5;
const int motor3_PWM_pin = 6;
const int motor4_PWM_pin = 7;
const int motor5_PWM_pin = 8;
const int motor6_PWM_pin = 9;

const int motor1_interruptPin = 2;

const int NES_Latch_pin = 40;
const int NES_Pulse_pin = 41;
const int NES_Data_pin = 42;
unsigned int controllerState = 0;         // variable for reporting the NES pushbutton status

#define forward 1
#define reverse 0
#define A_Button      B10000000
#define B_Button      B01000000
#define Select_Button B00100000
#define Start_Button  B00010000
#define Up_Button     B00001000
#define Down_Button   B00000100
#define Left_Button   B00000010
#define Right_Button  B00000001


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

  // initialize motor control pins as outputs
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
  
  // initialize pins for NES controller
  pinMode(NES_Latch_pin, OUTPUT); // NES_Latch output
  pinMode(NES_Pulse_pin, OUTPUT); // NES_Pulse output
  pinMode(NES_Data_pin, INPUT_PULLUP);   // NES_Data input
  
  // initialize digital pin 13 as an output for led.
  pinMode(13, OUTPUT);  // On board Led

  // Initialize Interrupts
  attachInterrupt(digitalPinToInterrupt(motor1_interruptPin), encoder1_ISR, RISING);
  
}


void loop() {
  // put your main code here, to run repeatedly:
  
  
  //motor1_control(forward, 20, 10000);
  
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

  //Serial.print("Encoder 1 counter is: ");
  //Serial.println(encoder1_count);
    
 // delay(5000);              // wait for a few seconds
  
 // motor1_stop();
  
  //delay(2000);              // wait for a second

  //motor1_control(reverse, 200, 10000);
  //delay(5000);              // wait for a few seconds

  //motor1_stop();            // stop the motor at the end

  //motor monitor
  if (encoder1_count <= 0) {
    encoder1_count = 0;
    //motor1_stop();
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

  read_NES();
  print_NES_Status();
  Serial.print("controllerState = ");
  Serial.println(controllerState);
   
  
  
}

/* ---------------- Function Declarations ---------------- */
  
/*  -------------------- motor1_control - moves motor 1 in forward/reverse direction ---------------
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


/*  --------------------- print_NES_Status - print status of NES Controller ------------------------
 */
void print_NES_Status() {  
  if (controllerState & A_Button){
    Serial.println("A_Button pressed");
  }
  if (controllerState & B_Button){
    Serial.println("B_Button pressed");
  }
  if (controllerState & Select_Button){
    Serial.println("Select_Button pressed");
  }
  if (controllerState & Start_Button){
    Serial.println("Start_Button pressed");
  }
  if (controllerState & Up_Button){
    Serial.println("Up_Button pressed");
  }
  if (controllerState & Down_Button){
    Serial.println("Down_Button pressed");
  }
  if (controllerState & Left_Button){
    Serial.println("Left_Button pressed");
  }
  if (controllerState & Right_Button){
    Serial.println("Right_Button pressed");
  }
}
/*  --------------------- read_NES - read NES controller ------------------------
 * 
 * NES_Latch output (pin 40)
 * NES_Pulse output (pin 41)
 * NES_Data input (pin 42)
 *                                              
 */
void read_NES() {  
  // need a temp to hold controller state and then invert (asserted low)
  unsigned int NES_temp = 0;
  
  digitalWrite(NES_Latch_pin, LOW);  // make sure latch and pulse are low when starting  
  digitalWrite(NES_Pulse_pin, LOW); 
  digitalWrite(NES_Latch_pin, HIGH); // take latch high
  delayMicroseconds(12);             // attempt to approximate NES response
  digitalWrite(NES_Latch_pin, LOW);  // take latch low
  NES_temp |= digitalRead(NES_Data_pin);  //read "A" bit
  NES_temp = NES_temp << 1;          // and shift it over   
  delayMicroseconds(6);              // attempt to approximate NES response

  //  clock in first bit (A button)
  digitalWrite(NES_Pulse_pin, HIGH); // take pulse high
  delayMicroseconds(6);              // attempt to approximate NES response
  NES_temp |= digitalRead(NES_Data_pin);  //read bit
  NES_temp = NES_temp << 1;          // and shift it over          
  digitalWrite(NES_Pulse_pin, LOW);  // take pulse low
  delayMicroseconds(6);              // attempt to approximate NES response

  //  clock in second bit (B button)
  digitalWrite(NES_Pulse_pin, HIGH); // take pulse high
  delayMicroseconds(6);              // attempt to approximate NES response
  NES_temp |= digitalRead(NES_Data_pin);  //read bit
  NES_temp = NES_temp << 1;          // and shift it over          
  digitalWrite(NES_Pulse_pin, LOW);  // take pulse low
  delayMicroseconds(6);              // attempt to approximate NES response

  
  //  clock in third bit (select button)
  digitalWrite(NES_Pulse_pin, HIGH); // take pulse high
  delayMicroseconds(6);              // attempt to approximate NES response
  NES_temp |= digitalRead(NES_Data_pin);  //read bit
  NES_temp = NES_temp << 1;          // and shift it over          
  digitalWrite(NES_Pulse_pin, LOW);  // take pulse low
  delayMicroseconds(6);              // attempt to approximate NES response

  
  //  clock in fourth bit (start button)
  digitalWrite(NES_Pulse_pin, HIGH); // take pulse high
  delayMicroseconds(6);              // attempt to approximate NES response
  NES_temp |= digitalRead(NES_Data_pin);  //read bit
  NES_temp = NES_temp << 1;          // and shift it over          
  digitalWrite(NES_Pulse_pin, LOW);  // take pulse low
  delayMicroseconds(6);              // attempt to approximate NES response

  
  //  clock in fifth bit (Up button)
  digitalWrite(NES_Pulse_pin, HIGH); // take pulse high
  delayMicroseconds(6);              // attempt to approximate NES response
  NES_temp |= digitalRead(NES_Data_pin);  //read bit
  NES_temp = NES_temp << 1;          // and shift it over          
  digitalWrite(NES_Pulse_pin, LOW);  // take pulse low
  delayMicroseconds(6);              // attempt to approximate NES response


  //  clock in sixth bit (Down Button)
  digitalWrite(NES_Pulse_pin, HIGH); // take pulse high
  delayMicroseconds(6);              // attempt to approximate NES response
  NES_temp |= digitalRead(NES_Data_pin);  //read bit
  NES_temp = NES_temp << 1;          // and shift it over          
  digitalWrite(NES_Pulse_pin, LOW);  // take pulse low
  delayMicroseconds(6);              // attempt to approximate NES response

  
  //  clock in seventh bit (Left button)
  digitalWrite(NES_Pulse_pin, HIGH); // take pulse high
  delayMicroseconds(6);              // attempt to approximate NES response
  NES_temp |= digitalRead(NES_Data_pin);  //read in eighth bit   
  digitalWrite(NES_Pulse_pin, LOW);  // take pulse low
  delayMicroseconds(6);              // attempt to approximate NES response

  //  clock in eighth bit (Right button)
  digitalWrite(NES_Pulse_pin, HIGH); // take pulse high
  delayMicroseconds(6);              // attempt to approximate NES response
  digitalWrite(NES_Pulse_pin, LOW);  // take pulse low

  controllerState = ~NES_temp;       // invert
 
}
  
/*  --------------------- encoder1_ISR ------------------------
 */
void encoder1_ISR() {
  encoder1_count = encoder1_count - 1;    // decrement encoder count
 
}

