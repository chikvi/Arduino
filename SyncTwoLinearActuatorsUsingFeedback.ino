//Actuator 1
float conNum1 = 0.01272534465;         // Constant to convert ADC to Inches
                                // Equal to (943 (ADC at 12") - 0 (ADC at 0")/12")^-1
float conNum2 = 0.01243339254;         // Constant to convert ADC to Inches
                                // Equal to (1023 (ADC at 12") - 475 (ADC at 0")/12")^-1
                                //.011730205

#define numOfActuators 2
int pinSet1[numOfActuators] = {5, 10};
int pinSet2[numOfActuators] = {6, 11};

int opticalPins[numOfActuators] = {A0, A1};

float constants[numOfActuators] = {0.01272534465, 0.011730205279};
float analogStartValues[numOfActuators] = {0, 0};

int counter[numOfActuators] = {};
float current_pos[numOfActuators] = {0.0, 0.0};

int Direction;
float diff_dist_allowed = 0.04;   //  Max allowed distance between two actuators
float present_diff = 0.0;
int Speed = 255;

unsigned long startMillis = 0;

unsigned long lastFrame = 0;

int synchronizeInterval = 200;

float act1_curr_pos = 0.0, act2_curr_pos = 0.0;
                                
void setup() {
  
  pinMode(A0, INPUT);         // Configure Analog In pin 0 as an Input
  pinMode(A1, INPUT);         // Configure Analog In pin 1 as an Input
  
  pinMode(pinSet1[0], OUTPUT);        // Configure pin 5 as an Output
  pinMode(pinSet1[1], OUTPUT);        // Configure pin 6 as an Output
  
  pinMode(pinSet2[0], OUTPUT);        // Configure pin 10 as an Output
  pinMode(pinSet2[1], OUTPUT);        // Configure pin 11 as an Output
  
  Serial.begin(9600);

  startMillis = millis();

  Direction = -1;

  //Just start the code whatever
  driveActuator(0, Direction, Speed*0.25);
  driveActuator(1, Direction, Speed*0.25);
}


void loop() {

  //Current positions
  act1_curr_pos = readPotentiometer(0, analogStartValues[0]);
  act2_curr_pos = readPotentiometer(1, analogStartValues[1]); //measured error for potentiometer


  //Note: This difference is based on Actuator1 and Actuator2 
  present_diff = act1_curr_pos - act2_curr_pos;

  Serial.print("Actuator 1:\t");
  Serial.print(act1_curr_pos);
  Serial.print("\tActuator 2:\t");
  Serial.print(act2_curr_pos);
  Serial.print("\tDiff :\t");
  Serial.println(present_diff);
  
  moveActuatorParallel(Direction);
  

  lastFrame = millis();
  
  //Exit Program after extension or retraction
  if((act1_curr_pos || act2_curr_pos) >11.9 && Direction == 1)
    exit(0);
  if((act1_curr_pos || act2_curr_pos) <0.01 && Direction == -1)
    exit(0); 
}


void moveActuatorParallel(int Direction)
{
  //if(millis() - startMillis < synchronizeInterval)
  //{
    if(abs(present_diff) > diff_dist_allowed){

      //If direction is negative the diff will always be negative
      if(Direction==-1)
        present_diff *= -1;

      float speedCoef = calcSpeedCo(present_diff);
      if(present_diff>0)
      {
        //Serial.println("YOU HU");
        //stopActuator(0);
        driveActuator(0, Direction, Speed * speedCoef);
        driveActuator(1, Direction, Speed); 
      }else{
        //Serial.println("HU YU");
        //stopActuator(1);
        driveActuator(1, Direction, Speed * speedCoef);
        driveActuator(0, Direction, Speed); 
      }
    //}

  }else{
    startMillis = millis();
    
  }
  //exit(0);
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


/*Function to read Potentiometer and Convert it to Inches*/
float readPotentiometer(int idx, int start_analog){
  //Serial.println("Reading the position of potentiometer:");  
   return constants[idx]*(analogRead(opticalPins[idx]) - start_analog); // 0 ADC is equal to 0"
}


float calcSpeedCo(float diff)
{
  return (1.0 - (abs(diff)/12.0)); // CalculateCoeff
}

void printAnalogValues(){
  Serial.println("Analog Value 1:");
  Serial.println(analogRead(A0));
  Serial.println("Analog Value 2:");
  Serial.println(analogRead(A1));
}