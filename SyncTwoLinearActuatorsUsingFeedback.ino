#define numOfActuators 2
int pinSet1[numOfActuators] = {5, 10};
int pinSet2[numOfActuators] = {6, 11};

int opticalPins[numOfActuators] = {A0, A1};

double constants[numOfActuators] = {0.012565445026178, 0.0126448893572181};
float analogStartValues[numOfActuators] = {0, 0};

int counter[numOfActuators] = {};
float current_pos[numOfActuators] = {0.0, 0.0};

//errors measured for safeguarding
float extension_diff_errors = -0.09;
float retraction_diff_errors = -0.09;

int Direction;
float diff_dist_allowed = 0.04;   //  Max allowed distance between two actuators
float present_diff = 0.0;
int Speed = 255;

unsigned long startMillis = 0;
unsigned long lastFrame = 0;

int synchronizeInterval = 200;

double act1_curr_pos = 0.0, act2_curr_pos = 0.0;
double max_diff = 0.25;

                                
void setup() {
  
  pinMode(A0, INPUT);         // Configure Analog In pin 0 as an Input
  pinMode(A1, INPUT);         // Configure Analog In pin 1 as an Input
  
  pinMode(pinSet1[0], OUTPUT);        // Configure pin 5 as an Output
  pinMode(pinSet1[1], OUTPUT);        // Configure pin 6 as an Output
  
  pinMode(pinSet2[0], OUTPUT);        // Configure pin 10 as an Output
  pinMode(pinSet2[1], OUTPUT);        // Configure pin 11 as an Output
  
  Serial.begin(9600);

  startMillis = millis();

  Direction = 1;
}


void loop() {

  //Current positions
  act1_curr_pos = readPotentiometer(0, analogStartValues[0]);
  act2_curr_pos = readPotentiometer(1, analogStartValues[1]); 

  //Note: This difference is based on Actuator1 and Actuator2 
  present_diff = act1_curr_pos - act2_curr_pos;
  
  Serial.print("Diff :\t");
  Serial.println(present_diff);
  
  moveActuatorParallel(Direction);
  
  //Exit Program after extension or retraction
  if((act1_curr_pos || act2_curr_pos) > 11.9 && Direction == 1)
    exit(0);
  if((act1_curr_pos || act2_curr_pos) < 0.01 && Direction == -1)
    exit(0); 
}


void moveActuatorParallel(int Direction)
{
  //This will make a little or no difference
  float actual_dist_allowed = 0;
  if(Direction == 1)
  {
      actual_dist_allowed = abs(extension_diff_errors) + abs(diff_dist_allowed);
  }else if(Direction == -1)
  {
      actual_dist_allowed = abs(retraction_diff_errors) + abs(diff_dist_allowed);
  }

  
  if(abs(present_diff) > actual_dist_allowed +0.03){

      //If direction is negative the diff will always be negative
    if(Direction==-1)
      present_diff *= -1;

      //float speedCoef = calcSpeedCo(present_diff);
      float speedCoef =  calcSpeedCo1(present_diff);

      if(speedCoef == 0)
      {
          driveActuator(0, Direction, 0);
          driveActuator(1, Direction, 0);

          exit(0);
      }
      
      if(present_diff>0)
      {
        Serial.println("YOU HU");

          driveActuator(0, Direction, Speed * speedCoef);
          driveActuator(1, Direction, Speed); //Try 1- speedCoef Later

          Serial.print("Speed Actuator 1: ");
          Serial.println(Speed * speedCoef);
          Serial.print("Speed Actuator 2: ");
          Serial.println(Speed);
      
        
      }else{
        Serial.println("HU YOU");
          driveActuator(1, Direction, Speed * speedCoef);
          driveActuator(0, Direction, Speed); 

          Serial.print("Speed Actuator 1: ");
          Serial.println(Speed);
          Serial.print("Speed Actuator 2: ");
          Serial.println(Speed * speedCoef);
      
      }

  }else{
    startMillis = millis();
    
    driveActuator(1, Direction, Speed);
    driveActuator(0, Direction, Speed); 
  }
}


void driveActuator(int actuatorNum, int Direction, int s)
{
  switch(Direction){
    case -1:
      // Retract Actuator
     analogWrite(pinSet1[actuatorNum], 0);
     analogWrite(pinSet2[actuatorNum], s);

     break;
    case 1:
      // Extend Actuator
      analogWrite(pinSet1[actuatorNum], s);
      analogWrite(pinSet2[actuatorNum], 0);

     break;
    default:
      Serial.println("Invalid Direction");
      break;
  }
}


void stopActuator(int idx){

  float firstFrame = millis();
  float currentFrame = millis();

  //synchronization bar is set to 200
  while(currentFrame < firstFrame + 200)
  {
    analogWrite(pinSet1[idx], 0);
    analogWrite(pinSet2[idx], 0);
    
    currentFrame = millis();
  }
}


/*Function to read Potentiometer and Convert it to Inches*/
float readPotentiometer(int idx, int start_analog){
  //Serial.println("Reading the position of potentiometer:");  
   return constants[idx]*(analogRead(opticalPins[idx]) - start_analog); // 0 ADC is equal to 0"
}


double calcSpeedCo1(double diff)
{
  if(abs(diff) > max_diff)
    return 0;

  return (1 - pow((abs(diff)/max_diff), 0.5));
}
