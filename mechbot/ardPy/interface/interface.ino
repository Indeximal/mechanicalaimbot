struct s_stepper{
  int dirPin;
  int stepPin;
  int microhalfPin;
  int currentPosition = 0;
  int step_state;
  int steps;
  int dir;
  

  unsigned long time_now = 0;

  float time_min;
};
const float time_min = 20;

s_stepper s1;
s_stepper s2;

s_stepper *big = &s1;
s_stepper *small = &s2;

int incoming[4];
int start = 0;

int readen = 0;

void setup() {
  s2.dirPin = 6;
  s2.stepPin = 5;
  s2.microhalfPin = 10;
  s1.dirPin = 3;
  s1.stepPin = 4;
  s1.microhalfPin = 8;

  
  Serial.begin(9600);
  
  // initialize pins
  pinMode(s1.stepPin, OUTPUT); 
  pinMode(s1.dirPin, OUTPUT);
  pinMode(s2.stepPin, OUTPUT);
  pinMode(s2.dirPin, OUTPUT);
  pinMode(s1.microhalfPin, OUTPUT);
  pinMode(s2.microhalfPin, OUTPUT);

  digitalWrite(s2.microhalfPin, HIGH);
  digitalWrite(s1.microhalfPin, HIGH);
  
}
//3 Fälle:
//aktiv High: warte, low setze, warte
//aktiv low: warte
//passiv low: nichts

//Hardware: Y-Axis ish Spiegelachse

void loop() {
  while(Serial.available() >= 4){ //read 4-Byte message
    for(int i = 0; i < 4; i++){
      incoming[i] = Serial.read();  //second and third Byte: step1 and step2
    }
    readen = 1;
  }
  if(readen){  
    if(incoming[0] == 0xff){
        s1.currentPosition = 0;
        s2.currentPosition = 0;
        Serial.write(0xff);
        Serial.write(0);
    }
    if(incoming[0] == 1){ //first Byte: identifier
      
      incoming[1] = incoming[1] - s1.currentPosition - 127;
      incoming[2] = incoming[2] - s2.currentPosition - 127; 
      
      //if(incoming[3] == 0){                 
        //Serial.write(0xfe);
        //Serial.write(0);                    
      //}  
      if(incoming[1] < 0){ //set direction 
        digitalWrite(s1.dirPin, HIGH);
        s1.steps = abs(incoming[1]);
        s1.dir = -1;
      }
      else{
        digitalWrite(s1.dirPin, LOW);
        s1.steps = incoming[1];
        s1.dir = 1;
      }
      
      if(incoming[2] < 0){ //set direction 
        digitalWrite(s2.dirPin, HIGH);
        s2.steps = abs(incoming[2]);
        s2.dir = -1;
      }
      else{
        digitalWrite(s2.dirPin, LOW);
        s2.steps = incoming[2];
        s2.dir = 1;
      }
          
      if(s1.steps > s2.steps){
        big = &s1;
        small = &s2;
      }
      else{
        big = &s2;
        small = &s1;
      }
      
      big->steps = big->steps*2-1;
      small->steps = small->steps*2-1;
      small->time_min = (big->steps/small->steps)*time_min;
      big->time_min = time_min;
  
      start = 1;

      while(!((millis()-small->time_now > small->time_min) && (millis()- big->time_now > big->time_min))){}
      
      if(s1.step_state == 1 or s2.step_state == 1){   
        s1.step_state = LOW;
        s2.step_state = LOW;
        
        digitalWrite(s1.stepPin, s1.step_state);
        digitalWrite(s2.stepPin, s2.step_state);
    
        small->time_now = millis();
        big->time_now = millis();
      }
    }
      readen = 0;
  }
    
    if((millis()- big->time_now > big->time_min) && (big->steps > 0) && start){
      digitalWrite(big->stepPin, !big->step_state);
      big->time_now = millis();
      big->step_state = !big->step_state;
      big->steps--;
      if(big->step_state == HIGH){
        big->currentPosition = big->currentPosition + big->dir;
        Serial.write(1);
        Serial.write(s1.currentPosition + 127);
        Serial.write(s2.currentPosition + 127);
        Serial.write(0);
      }
    } 
    if((millis()-small->time_now > small->time_min) && (small->steps > 0) && start){
      digitalWrite(small->stepPin, !small->step_state);
      small->time_now = millis();
      small->step_state = !small->step_state;
      small->steps--;
      if(small->step_state == HIGH){
        small->currentPosition = small->currentPosition + small->dir;
        Serial.write(1);
        Serial.write(s1.currentPosition + 127);
        Serial.write(s2.currentPosition + 127);
        Serial.write(0);
      }
    }  
}
