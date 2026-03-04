//declare variables
long dur;
float dist;

void setup() { 
  //set echo and trigger pins 
  pinMode(8, INPUT); //set pin D8 as input for echo
  pinMode(9, OUTPUT); //set pin D9 as output for trigger

  //set LED pins as output and clear them
  pinMode(2, OUTPUT); //green
  pinMode(3, OUTPUT); //yellow
  pinMode(4, OUTPUT); //red
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  Serial.begin(9600); //used for serial monitor feedback
}

void loop() {
  //clear trigger pin
  digitalWrite(9, LOW);
  delay(0.002); //delay 2 microseconds 

  //send pulse to trigger pin for 10 microseconds
  //this will initiate the distance measurement sequence 
  digitalWrite(9, HIGH);
  delay(0.01); //delay 10 microseconds
  digitalWrite(9, LOW);

  //read echo pin pulse duration and calculate corresponding distance
  dur = pulseIn(8, HIGH); 
  dist = dur/1778.5; //calculates distance in feet
  Serial.println("Distance in feet:");
  Serial.println(dist);

  //if statements to determine which LED turns on
  if(dist >= 4){
    //turn green LED on and turn others off
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  }else if(dist < 4 && dist > 2){
    //turn yellow LED on and turn others off
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
  }else if(dist <= 2){
    //turn red LED on and turn others off
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
  }

  delay(500); //delay between iterations, can decrease to update sensor more frequently 
}
