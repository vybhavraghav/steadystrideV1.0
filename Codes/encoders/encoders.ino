long encoderValueL = 0;
long encoderValueR = 0;
int c=0;

long lastEncoded= 0;
int angleR= 0;
int angleL= 0;

int motor1Pin1 = 6;
int motor1Pin2 = 7;
int motor2Pin1 = 4;
int motor2Pin2 = 5;
int motorDirection = 1;


float kp = -0.6;

void encoderHandlerL() {
  int MSB = digitalRead(2); //MSB = most significant bit
  int LSB = digitalRead(3); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValueL--; //CW
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValueL++; //CCW
  }

  lastEncoded = encoded; //store this value for next time
  angleL = encoderValueL*360/810;
  Serial.print("L:");Serial.println(angleL);


}

void encoderHandlerR() {
  int MSB = digitalRead(20); //MSB = most significant bit
  int LSB = digitalRead(21); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValueR++; //CW
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValueR--; //CCW
  }

  lastEncoded = encoded; //store this value for next time
  angleR = encoderValueR*360/810;
  Serial.print("R:");Serial.println(angleR);



}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  attachInterrupt(digitalPinToInterrupt(20), encoderHandlerR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), encoderHandlerR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), encoderHandlerL,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoderHandlerL, CHANGE);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  // int motorSpeed = 0;//abs(output);  // Map the absolute output to the motor speed range

  // if (output <= 0.00) {
  //   motorDirection = -1;
  // } else {
  //   motorDirection = 1;
  // }


  int e = angleL-angleR;

  Serial.println(e);
  int off = kp * e;

  if (off <= 0.00) {
    motorDirection = -1;
  } else {
    motorDirection = 1;
  }

  int motorSpeed =0;
  if motor
  if (motorDirection == 1) {
    analogWrite(motor1Pin1, motorSpeed);
    analogWrite(motor1Pin2, 0);
    analogWrite(motor2Pin1, motorSpeed-off );
    analogWrite(motor2Pin2, 0);
  } else if (motorDirection == -1) {
    analogWrite(motor1Pin1, 0);
    analogWrite(motor1Pin2, motorSpeed);
    analogWrite(motor2Pin1, 0);
    analogWrite(motor2Pin2, motorSpeed+off);
  }
  // Serial.println(off);
  // delay(1000);


}
