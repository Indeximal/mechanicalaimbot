struct s_stepper{
  int dirPin;
  int stepPin;
  int currentPosition;
  int step_state;
  int steps;
  int dir;

  unsigned long time_now = 0;

  float time_min;
};
const float time_min = 1;

s_stepper s1;
s_stepper s2;

s_stepper *big = &s1;
s_stepper *small = &s2;

int incoming[2];
int start = 0;

void setup() {
  Serial.begin(9600);
  
  // initialize pins
  pinMode(s1.stepPin, OUTPUT); 
  pinMode(s1.dirPin, OUTPUT);
  pinMode(s2.stepPin, OUTPUT);
  pinMode(s2.dirPin, OUTPUT);

  s1.dirPin = 6;
  s1.stepPin = 5;
  s2.dirPin = 3;
  s2.stepPin = 4;
}
void loop() {
  if(Serial.available() >= 4){             //read 4-Byte message
    if(Serial.read() == 1){               //first Byte: identifier
      for(int i = 0; i < 2; i++){
        incoming[i] = Serial.read()-127;  //second and third Byte: step1 and step2
      }
    }
    
    if(Serial.read() == 0){               //last Byte: 0 Byte indicates message-end
      Serial.write(1);                    //send back 1 to python as indication that message was successfully readen
    }
    
    (*big).currentPosition = (*big).currentPosition * (*big).dir;
    (*small).currentPosition = (*small).currentPosition * (*small).dir;
    
    incoming[0] = incoming[0] - s1.currentPosition;
    incoming[1] = incoming[1] - s2.currentPosition; 
    
    s1.currentPosition = 0;
    s2.currentPosition = 0;
      
    if(incoming[0] < 0){ //set direction 
      digitalWrite(s1.dirPin, HIGH);
      s1.steps = abs(incoming[0]);
      s1.dir = -1;
    }
    else{
      digitalWrite(s1.dirPin, LOW);
      s1.steps = incoming[0];
      s1.dir = 1;
    }
    
    if(incoming[1] < 0){ //set direction 
      digitalWrite(s2.dirPin, LOW);
      s2.steps = abs(incoming[1]);
      s1.dir = -1;
    }
    else{
      digitalWrite(s2.dirPin, HIGH);
      s2.steps = incoming[1];
      s1.dir = 1;
    }
        
    if(s1.steps > s2.steps){
      big = &s1;
      small = &s2;
    }
    else{
      big = &s1;
      small = &s2;;
    }
    
    (*big).steps = (*big).steps*2-1;
    (*small).steps = (*small).steps*2-1;
    (*small).time_min = (*big).steps/(*small).steps;
    (*big).time_min = time_min;

    start = 1;
        
    while(!((millis()-(*small).time_now > (*small).time_min) && (millis()- (*big).time_now > (*big).time_min))){}
    
    s1.step_state = LOW;
    s2.step_state = LOW;
  
    digitalWrite(s1.stepPin, s1.step_state);
    digitalWrite(s2.stepPin, s2.step_state);
  }
  
  if((millis()- (*big).time_now > (*big).time_min) && ((*big).steps > 0) && start){
    (*big).time_now = millis();
    digitalWrite((*big).stepPin, !(*big).step_state);
    (*big).step_state = !(*big).step_state;
    (*big).steps--;

    if((*big).step_state == HIGH){
      (*big).currentPosition++;
    }
  } 
  if((millis()-(*small).time_now > (*small).time_min) && ((*small).steps > 0)){
    (*small).time_now = millis();
    digitalWrite((*small).stepPin, !(*small).step_state);
    (*small).step_state = !(*small).step_state;
    (*small).steps--; 
    
    if((*small).step_state == HIGH){
      (*small).currentPosition++;
    }
  }  
}
