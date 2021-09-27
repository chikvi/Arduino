float pos1;                      // Actuator 1 Position
float pos2;                     //Actuator 2 Position

//Actuator 1
float conNum1 = 0.012725;         // Constant to convert ADC to Inches
                                // Equal to (943 (ADC at 12") - 0 (ADC at 0")/12")^-1
float conNum2 = 0.01173020528;
//0.021505376344086;         // Constant to convert ADC to Inches
                                // Equal to (1023 (ADC at 12") - 475 (ADC at 0")/12")^-1

#define numOfActuators 2
int pinSet1[numOfActuators] = {5, 6};
int pinSet2[numOfActuators] = {10, 11};

int opticalPins[numOfActuators] = {A0, A1};
                                
void setup() {
  
  pinMode(A0, INPUT);         // Configure Analog In pin 0 as an Input
  pinMode(A1, INPUT);         // Configure Analog In pin 1 as an Input
  
  pinMode(pinSet1[0], OUTPUT);        // Configure pin 5 as an Output
  pinMode(pinSet1[1], OUTPUT);        // Configure pin 6 as an Output
  
  pinMode(pinSet2[0], OUTPUT);        // Configure pin 10 as an Output
  pinMode(pinSet2[1], OUTPUT);        // Configure pin 11 as an Output
  
  Serial.begin(9600);
}


void loop() {

  int sensorValue = 1;
  
  if(sensorValue){
   // Retract Actuator
   analogWrite(pinSet1[0], 0);
   analogWrite(pinSet1[1], 255);

   pos1 = readPotentiometer1(); // Print position value to the Serial Display
   Serial.println("Actuator 1:");
   Serial.println(pos1);
   
   analogWrite(pinSet2[0], 0);
   analogWrite(pinSet2[1], 255);
   
   pos2 = readPotentiometer2(); // Print position value to the Serial Display
   Serial.println("Actuator 2:");
   Serial.println((pos2 + 4));
   
   delay(1);
  }
  else if(sensorValue == 0){
    // Extend Actuator
    analogWrite(pinSet1[0], 255);
    analogWrite(pinSet1[1], 0);

    Serial.println("Actuator 1:");
    pos1 = readPotentiometer1();
    Serial.println(pos1);  // Print position value to the Serial Display
    
    analogWrite(pinSet2[0], 255);
    analogWrite(pinSet2[1], 0);
    
    Serial.println("Actuator 2:");
    pos2 = readPotentiometer2();
    Serial.println(pos2 +4);  // Print position value to the Serial Display
    
    delay(1);
  }
  else{
    // Stop Actuator1
    analogWrite(pinSet1[0], 0);
    analogWrite(pinSet1[1], 0);

    //Stop Actuator2
    analogWrite(pinSet2[0], 0);
    analogWrite(pinSet2[1], 0);
  }

  printAnalogValues();
}


/*Function to Read Potentiometer 1 and Convert it to Inches*/
float readPotentiometer1(void){
  Serial.println("Reading the position of potentiometer 1:");
  float pos;
  pos = conNum1*(analogRead(A0) - 0); // 0 ADC is equal to 0"
  return pos;
}

/*Function to Read Potentiometer and Convert it to Inches*/
float readPotentiometer2(void){
  Serial.println("Reading the position of potentiometer 2:");
  float pos;
  pos = conNum2*(analogRead(A1) - 0); // 460 ADC is equal to 0"

  return pos;
}

void printAnalogValues(){
  Serial.println("Analog Value 1:");
  Serial.println(analogRead(A0));
  Serial.println("Analog Value 2:");
  Serial.println(analogRead(A1));
}
