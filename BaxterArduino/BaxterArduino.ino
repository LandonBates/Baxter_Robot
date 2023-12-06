int i;

int directionPinTurn = 12;

int pwmPinTurn = 3;

int brakePinTurn = 9;



// uncomment if using channel B

int directionPinNod = 13;

int pwmPinNod = 11;

int brakePinNod = 8;


//boolean to switch direction

bool directionState;

// limit switches
int limitRight = 4;
int limitNod = 2;

int limitLeftState;
int limitRightState;
int limitNodState;
int newlimitRightState;
int newlimitNodState;

void setup() {

  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

//define pins

pinMode(directionPinTurn, OUTPUT);

pinMode(pwmPinTurn, OUTPUT);

pinMode(brakePinTurn, OUTPUT);



pinMode(directionPinNod, OUTPUT);

pinMode(pwmPinNod, OUTPUT);

pinMode(brakePinNod, OUTPUT);


pinMode(limitRight, INPUT);
pinMode(limitNod, INPUT);

}


void loop() {
String commandString = Serial.readString();
//Serial.println(commandString);
int l = commandString.length();
for(i=0;l>i; i++)
{
//newlimitRightState=digitalRead(limitRight);
//newlimitNodState=digitalRead(limitNod);
newlimitRightState=0;
newlimitNodState=0;
  char command = commandString[i];
  Serial.println(command);
  if( command == 'W')
  {
    digitalWrite(directionPinNod, HIGH);
    digitalWrite(pwmPinNod,50);
    delay(10);
    digitalWrite(pwmPinNod,0);
  }
  if( command == 's')
  {
    digitalWrite(directionPinNod, LOW);
    digitalWrite(pwmPinNod,10);
    delay(1);
    digitalWrite(pwmPinNod,0);
  }
  if( command == 'S')
  {
    digitalWrite(directionPinNod, LOW);
    digitalWrite(pwmPinNod,50);
    delay(10);
    digitalWrite(pwmPinNod,0);
  }

  if( command == 'w')
  {
    digitalWrite(directionPinNod, HIGH);
    digitalWrite(pwmPinNod,10);
    delay(1);
    digitalWrite(pwmPinNod,0);
  }
 if( command == 'a')
  {
    digitalWrite(directionPinTurn, HIGH);
    digitalWrite(pwmPinTurn,50);
    delay(1);
    digitalWrite(pwmPinTurn,0);
  }
 if( command == 'A')
  {
    digitalWrite(directionPinTurn, HIGH);
    digitalWrite(pwmPinTurn,100);
    delay(10);
    digitalWrite(pwmPinTurn,0);
  }

  if (command == 'd')
  {
    digitalWrite(directionPinTurn, LOW);
    digitalWrite(pwmPinTurn,50);
    delay(1);
    digitalWrite(pwmPinTurn,0);
  }
  if (command == 'D')
  {
    digitalWrite(directionPinTurn, LOW);
    digitalWrite(pwmPinTurn,100);
    delay(10);
    digitalWrite(pwmPinTurn,0);
  }

// if (newlimitRightState== HIGH)
// {
//   
//     if(newlimitRightState != limitRightState)
//   {
//     digitalWrite(brakePinTurn, HIGH);
//     digitalWrite(pwmPinTurn,0);
//     limitRightState = newlimitRightState;
//     break;
//   }
//  
// }
// if (newlimitNodState== HIGH)
// {
//  
//       if(newlimitNodState != limitNodState)
//   {
//     digitalWrite(brakePinNod, HIGH);
//     digitalWrite(pwmPinTurn,0);
//     limitNodState = newlimitNodState;
//     break;
//   }
// }

}
if (newlimitRightState== HIGH)
 {
   Serial.print("1,");
}else{
Serial.print("0,");
}
 if (newlimitNodState== HIGH)
 {
   Serial.println("1");
   }else{
    Serial.println("0");
    }

}











 
