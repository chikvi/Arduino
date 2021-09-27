#define numActuators 2
int LPWM[numActuators] = {5, 10};
int RPWM[numActuators] = {6, 11};

int opticalPins[numActuators] = {A0, A1};

double constants[numActuators] = {0.0126984126984127, 0.0128342245989305};
double analogStartValues[numActuators] = {0, 0};

int Direction;
double diff_dist_allowed = 0.04;   //  Max allowed distance between two actuators
int Speed = 255;

unsigned long start_millis = 0;
double del_time = 0;
unsigned long current_millis = 0;
unsigned long old_time = 0;

double delDiff = 0.0;

double act0_curr_pos = 0, act1_curr_pos = 0, del_diff = 0;
double del_len_act[numActuators] = {0, 0};
double prev_len_act[numActuators] = {0, 0};
double prev_len_act0 = 0, prev_len_act1 = 0;

double del_len_act0 = 0, del_len_act1 = 0;

unsigned long firstFrame = 0, lastFrame = 0;
unsigned long callingInterval = 100;


void setup()
{
  //baud rate
  Serial.begin(9600);

  //Input setup
  for(int i = 0; i< numActuators; i++)
  {
    pinMode(opticalPins[i], INPUT);

    pinMode(LPWM[i], OUTPUT);
    pinMode(RPWM[i], OUTPUT);  
  } 

  Direction = 1;

  firstFrame = millis();
}


void loop()
{
  //Calculate Del Time
  calculateDelTime();
  
  act0_curr_pos = readActLen(0, analogStartValues[0]);
  act1_curr_pos = readActLen(1, analogStartValues[1]);

  del_diff = act0_curr_pos - act1_curr_pos;

  calculateDelLen();

  lastFrame = millis();

   execute(del_diff);
}


void execute(double del_diff)
{
  del_diff -= 0.01;
  Serial.println(del_diff);
  
  if(Direction == -1)
    del_diff *= -1;

  //TODO: Look for diff_dist_allowed
  if(del_diff >0)
  {
    float del_speed = calculateDelSpeed(0);
    float act0_speed = clamp(255 * (del_speed - del_diff)/del_speed);

    Serial.print("S_act1: ");
    Serial.println(del_speed - del_diff);
    //Move Actuautor 0
    driveActuator(0, Direction, act0_speed);

    Serial.print("Act1 Speed: ");
    Serial.println(act0_speed);
    
  }else if(del_diff < 0)
  {
    float del_speed = calculateDelSpeed(1);
    float act1_speed = clamp(255 * (del_speed - del_diff)/del_speed);
1
    //Move Actuator 1
    driveActuator(1, Direction, act1_speed);

    Serial.print("S_act2: ");
    Serial.println(del_speed - del_diff);
    
    Serial.print("Act2 Speed: ");
    Serial.println(act1_speed);
    
  }else{
    driveActuator(0, Direction, 255);
    driveActuator(1, Direction, 255);
  }             
}


void driveActuator(int actuatorNum, int Direction, int s)
{
  switch(Direction){
    case -1:
      // Retract Actuator
     analogWrite(LPWM[actuatorNum], 0);
     analogWrite(RPWM[actuatorNum], s);

     break;
    case 1:
      // Extend Actuator
      analogWrite(LPWM[actuatorNum], s);
      analogWrite(RPWM[actuatorNum], 0);

     break;
   case 0:
      //Actually stopping actuator
      analogWrite(LPWM[actuatorNum], 0);
      analogWrite(RPWM[actuatorNum], 0);

      break;
    default:
      //Serial.println("Invalid Direction");
      break;
  }
}


void stopActuator(int idx)
{
  unsigned long firstFrame = millis();
  unsigned long currentFrame = millis();

  //synchronization bar is set to 200 ms
  while(currentFrame < firstFrame + 200)
  {
    analogWrite(LPWM[idx], 0);
    analogWrite(RPWM[idx], 0);
    
    currentFrame = millis();
  }
}


void calculateDelTime(){
  
  unsigned long current_time = millis();
  del_time = (double)(current_time - old_time)/1000;
  old_time = current_time;
}


/*Function to read Potentiometer and Convert it to Inches*/
double readActLen(int idx, int start_analog){  
   return constants[idx]*(analogRead(opticalPins[idx]) - start_analog); // 0 ADC is equal to 0"
}


void calculateDelLen(){
  double curr_len_act[numActuators] = {0, 0};

  // Read Actuator 1 Length
  curr_len_act[0] = readActLen(0, 0);
  del_len_act0 = curr_len_act[0] - prev_len_act0;
  prev_len_act0 = curr_len_act[0];
  
  // Read Actuator 2 Length
  curr_len_act[1] = readActLen(1, 0);
  del_len_act1 = curr_len_act[1] - prev_len_act1;
  prev_len_act1 = curr_len_act[1];

}


double calculateDelSpeed(int idx)
{
  if(idx == 0)
    return del_len_act0/(float)del_time;
  else if(idx == 1)
    return del_len_act1/(float)del_time;
    
  return 0;
}


float clamp(float speed)
{
  if(speed < 0.0)
  {
    return 0.0;
  }else if(speed > 255.0)
  {
    return 255.0;  
  }
  return speed;
}
