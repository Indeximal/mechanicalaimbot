
struct s_stepper1{
  const int dirPin = 6;
  const int stepPin = 5;

  int step_state = 0;
   };

struct s_stepper2{
  const int dirPin = 3;
  const int stepPin = 4;

  int step_state = 0;
};

struct s_smallStepper{
  int dirPin = 0;
  int stepPin = 0;

  int step_state = 0;
  int time_now = 0;
  int steps = 0;

  float time_min = 0;
};

struct s_bigStepper{
  int dirPin = 0;
  int stepPin = 0;

  int step_state = 0;
  int time_now = 0;
  int steps = 0;

  const float time_min = 0.1; //tested minimal delay between HIGH and LOW to make a step.
};


//microstep 1/32
//const int m11 = 8;
//const int m12 = 7;
//const int m21 = 10;
//const int m22 = 9;


s_stepper1 s1;
s_stepper2 s2;
s_bigStepper big;
s_smallStepper small;


int incoming[2];


int readULongFromBytes() {
  union u_tag {
    byte b[1];
    int ulval;
  } u;
  u.b[0] = Serial.read();

  return u.ulval;
}

int start = 0;


void setup() {

  Serial.begin(9600);
  
  // initialize pins
  pinMode(s1.stepPin, OUTPUT); 
  pinMode(s1.dirPin, OUTPUT);
  pinMode(s2.stepPin, OUTPUT);
  pinMode(s2.dirPin, OUTPUT);

}

void loop() {

  if(Serial.available()== 2){
      for(int i = 0; i < 2; i++){
        incoming[i] = readULongFromBytes(); 
      }
  
      Serial.write(incoming[0]);
      Serial.write(incoming[1]);//write back success message in form of zero-byte
      
      //incoming[0] = 13+g*2;
      //incoming[1] = 15-g*2; //this works perfectly. interpretation of byte broken
      
      if(incoming[0]<127){ //set direction 
        digitalWrite(s1.dirPin, HIGH);
      }
      else{
        digitalWrite(s1.dirPin, LOW);
        incoming[0] = incoming[0] - 127;//shift number
      }
  
      if(incoming[1]<127){ //set direction 
        digitalWrite(s2.dirPin, LOW);
      }
      else{
        digitalWrite(s2.dirPin, HIGH);
        incoming[1] = incoming[1] - 127;
      }
  
      s1.step_state = LOW;
      s2.step_state = LOW;
  
      digitalWrite(s1.stepPin, s1.step_state);
      digitalWrite(s2.stepPin, s2.step_state);
      

        
      if(incoming[0] > incoming[1]){
        big.steps = incoming[0];
        small.steps = incoming[1];
        big.stepPin = s1.stepPin;
        big.step_state = s1.step_state;
        small.step_state = s2.step_state;
        small.stepPin = s2.stepPin;
        small.time_min = (big.steps*2-1)/(small.steps*2-1);
        big.steps = big.steps*2-1;
        small.steps = small.steps*2-1;
      }
      else{
        big.steps = incoming[1];
        small.steps = incoming[0];
        big.stepPin = s2.stepPin;
        big.step_state = s2.step_state;
        small.step_state = s1.step_state;
        small.stepPin = s1.stepPin;
        small.time_min = (big.steps*2-1)/(small.steps*2-1);
        big.steps = big.steps*2-1;
        small.steps = small.steps*2-1;
    
      }

      start = 1;
        
    }

  if((micros()- big.time_now > big.time_min) && (big.steps > 0) && start){
    big.time_now = micros();
    digitalWrite(big.stepPin, !big.step_state);
    big.step_state = !big.step_state;
    big.steps--;
      
  }
  if((micros()-small.time_now > small.time_min) && (small.steps > 0)){
    small.time_now = micros();
    digitalWrite(small.stepPin, !small.step_state);
    small.step_state = !small.step_state;
    small.steps;
  }  
  


 
}
