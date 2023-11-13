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
int limitLeft = 7;
int limitRight = 4;
int limitNod = 2;

int limitLeftState;
int limitRightState;
int limitNodState;


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

pinMode(limitLeft, INPUT);
pinMode(limitRight, INPUT);
pinMode(limitNod, INPUT);

}


void loop() {
String commandString = Serial.readString();
Serial.println(commandString);
int l = commandString.length();
if (l> 1)
{
  Serial.println(l);

for(i=0;l>i; i++)
{
int newlimitLeftState=digitalRead(limitLeft);
int newlimitRightState=digitalRead(limitRight);
int newlimitNodState=digitalRead(limitNod);
  char command = commandString[i];
  Serial.println(command);
  if( command == 'w')
  {
    digitalWrite(directionPinNod, HIGH);
    digitalWrite(pwmPinNod,5);
    delay(50);
    digitalWrite(pwmPinNod,0);
  }
  if( command == 's')
  {
    digitalWrite(directionPinNod, LOW);
    digitalWrite(pwmPinNod,5);
    delay(50);
    digitalWrite(pwmPinNod,0);
  }
  if( command == 'a')
  {
    digitalWrite(directionPinTurn, HIGH);
    digitalWrite(pwmPinTurn,100);
    delay(10);
    digitalWrite(pwmPinTurn,0);
  }
  if (command == 'd')
  {
    digitalWrite(directionPinTurn, LOW);
    digitalWrite(pwmPinTurn,100);
    delay(10);
    digitalWrite(pwmPinTurn,0);
  }
  if (newlimitLeftState== HIGH)
 {
   
   if(newlimitLeftState != limitLeftState)
   {
     digitalWrite(brakePinTurn, HIGH);
     digitalWrite(pwmPinTurn,0);
     limitLeftState = newlimitLeftState;
     break;
   }
 }
 else
 {
   
   analogWrite(pwmPinTurn, 20);

 }
 if (newlimitRightState== HIGH)
 {
   
     if(newlimitRightState != limitRightState)
   {
     digitalWrite(brakePinTurn, HIGH);
     digitalWrite(pwmPinTurn,0);
     limitRightState = newlimitRightState;
     break;
   }
  
 }
 else
 {
   
   analogWrite(pwmPinTurn, 20);
 }
 if (newlimitNodState== HIGH)
 {
  
       if(newlimitNodState != limitNodState)
   {
     digitalWrite(brakePinNod, HIGH);
     digitalWrite(pwmPinTurn,0);
     limitNodState = newlimitNodState;
     break;
   }
 }
}

   if (newlimitLeftState== HIGH)
 {
   Serial.print("1,");
   }else{
   Serial.print("0,");
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
   }else(Serial.println"0");}

}











 





}
