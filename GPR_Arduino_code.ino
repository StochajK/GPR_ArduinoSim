// LIBRARIES
#include <Servo.h>


// CONSTANTS

//initialize servos
Servo myservo;
//Servo myservo1;
Servo myservo2;
Servo myservo3;

//initialize ultrasonic pinskljklklkj
int ultrasonicInputPin1 = A0;  // ultrasonic module   ECHO to A0
int ultrasonicOutputPin1 = A1;  // ultrasonic module  TRIG to A1
int ultrasonicInputPin2 = A0;  // ultrasonic module   ECHO to A0
int ultrasonicOutputPin2 = A1;  // ultrasonic module  TRIG to A1

//initialize motor pins
#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  11    //pin of controlling speed---- ENB of motor driver board
int pinLB = 6;             //pin of controlling turning---- IN1 of motor driver board
int pinLF = 7;             //pin of controlling turning---- IN2 of motor driver board
int pinRB = 9;            //pin of controlling turning---- IN3 of motor driver board
int pinRF = 10;            //pin of controlling turning---- IN4 of motor driver board

//motor values
unsigned char Lpwm_val = 30; //initialized left wheel speed at 250
unsigned char Rpwm_val = 30; //initialized right wheel speed at 250
int motor_high = 5;

//car values
int Car_state = 0;             //the working state of car
int myangle;                //defining variable of angle
int pulsewidth;              //defining variable of pulse width
//unsigned char DuoJiao = 180;    //initialized angle of motor at 90°

//delay times
int loop_delay = 20; //initially 300 ms
int loop_halt = 10;
int backup_delay = 300; //initially 100 ms
int turn_delay = 200; //initially 400 ms
int stop_distance = 15; //initially 15 cm
int scan_distance = 30; //initially 30 cm

//ground distance
int ground_distance;
int ground_tolerance = 7;

//robotic arm values
int pos1 = 120, pos2_rest = 45, pos2_active = 120, pos3_rest = 60, pos3_active = 30, pos4=120;


//FUNCTIONS

//initialize move pins
void M_Control_IO_config(void) {
  pinMode(pinLB,OUTPUT); // /pin 8
  pinMode(pinLF,OUTPUT); // pin 4
  pinMode(pinRB,OUTPUT); // pin 7
  pinMode(pinRF,OUTPUT);  // pin 2
  pinMode(Lpwm_pin,OUTPUT);  // pin 5 (PWM) 
  pinMode(Rpwm_pin,OUTPUT);  // pin 6(PWM)   
}

//speed and distance functions
void Set_Speed(unsigned char Left,unsigned char Right){ //function of setting speed
  analogWrite(Lpwm_pin,Left);
  analogWrite(Rpwm_pin,Right);
}

int get_distance(){ //function of ultrasonic distance detecting ，MODE=1，displaying，no displaying under other situation
  digitalWrite(ultrasonicOutputPin1, LOW);  
  delayMicroseconds(2); 
  digitalWrite(ultrasonicOutputPin1, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(ultrasonicOutputPin1, LOW);    
  int distance = pulseIn(ultrasonicInputPin1, HIGH);  // reading the duration of high level
  distance = distance/58;   // Transform pulse time to distance
  Serial.print("\n H = ");
  Serial.print(distance, DEC);
  return distance;
} 

int get_ground_distance(){ //function of ultrasonic distance detecting ，MODE=1，displaying，no displaying under other situation
  digitalWrite(ultrasonicOutputPin2, LOW);  
  delayMicroseconds(2); 
  digitalWrite(ultrasonicOutputPin2, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(ultrasonicOutputPin2, LOW);    
  int distance = pulseIn(ultrasonicInputPin2, HIGH);  // reading the duration of high level
  distance = distance/58;   // Transform pulse time to distance 
  Serial.print("\n G = ");
  Serial.print(distance, DEC);
  return distance;
} 

//move functions
void advance(){    //  going forward
  digitalWrite(pinRB,motor_high);  // making motor move towards right rear
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,motor_high);  // making motor move towards left rear
  digitalWrite(pinLF,LOW); 
  Car_state = 1;   
}
    
void turnR(){        //turning right(dual wheel)
  digitalWrite(pinRB,LOW);  //making motor move towards right rear
  digitalWrite(pinRF,motor_high);
  digitalWrite(pinLB,motor_high);
  digitalWrite(pinLF,LOW);  //making motor move towards left front
  Car_state = 4;
}
    
void turnL(){         //turning left(dual wheel)
  digitalWrite(pinRB,motor_high);
  digitalWrite(pinRF,LOW );   //making motor move towards right front
  digitalWrite(pinLB,LOW);   //making motor move towards left rear
  digitalWrite(pinLF,motor_high);
  Car_state = 3;
}  
      
void stopp(){        //stop
  digitalWrite(pinRB,motor_high);
  digitalWrite(pinRF,motor_high);
  digitalWrite(pinLB,motor_high);
  digitalWrite(pinLF,motor_high);
  Car_state = 5;
}
    
void back(){         //back up
  digitalWrite(pinRB,LOW);  //making motor move towards right rear     
  digitalWrite(pinRF,motor_high);
  digitalWrite(pinLB,LOW);  //making motor move towards left rear
  digitalWrite(pinLF,motor_high);
  Car_state = 2;   
}

// Robotic arm functions
void arm_rest(){
  //myservo1.write(pos1_rest);
  myservo2.write(pos2_rest);
  myservo3.write(pos3_rest);
}

void arm_active(){
  //myservo1.write(pos1_active);
  myservo2.write(pos2_active);
  myservo3.write(pos3_active);
}

//self control function


void Self_Control(void){ //self-going, ultrasonic obstacle avoidance
  int G = get_ground_distance();

  Serial.print("\n ground_distance, ground_tolerance: ");
  Serial.print(ground_distance, DEC);
  Serial.print(" "); 
  Serial.print(ground_tolerance, DEC); 
  
  if (G >= ground_distance + ground_tolerance){
    //align the robot backwards over the hole and put the drill down
         stopp();  
         arm_active();
         delay(600);
         arm_rest();
         delay(500);
         arm_active();
         delay(600);
         arm_rest();
         delay(500);
         arm_active();
         delay(600);
         arm_rest();
         delay(1000);
         back(); 
         delay(1000);   
         stopp();        
       delay(50);   
  }
  else{
    advance();
    delay(loop_delay);
    stopp();
    delay(loop_halt);
  }             
}

//main
void setup(){ 
  //car setup
  myservo.attach(A2);
  M_Control_IO_config();     //motor controlling the initialization of IO
  Set_Speed(Lpwm_val,Rpwm_val);  //setting initialized speed
  //myservo.write(DuoJiao);       //setting initialized motor angle
  pinMode(ultrasonicInputPin1, INPUT);      //starting receiving IR remote control signal
  pinMode(ultrasonicOutputPin1, OUTPUT);    //IO of ultrasonic module
  pinMode(ultrasonicInputPin2, INPUT);      //starting receiving IR remote control signal
  pinMode(ultrasonicOutputPin2, OUTPUT);    //IO of ultrasonic module
  Serial.begin(9600);            //initialized serial port , using Bluetooth as serial port, setting baud 
  stopp();                       //stop
  delay(1000);

  ground_distance = get_ground_distance();

  //robotic arm setup
  //myservo1.attach(1);    // set the control pin of servo 1 to 3 digital I/0
  myservo2.attach(12);    // set the control pin of servo 2 to 5 digital I/0
  myservo3.attach(1);    // set the control pin of servo 3 to 6 digital I/0
  Serial.begin(9600);
} 

void loop(){ 
  Self_Control();   
}
