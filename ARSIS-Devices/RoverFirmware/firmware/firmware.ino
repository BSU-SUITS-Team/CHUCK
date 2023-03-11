byte leftMotorSpeed = 0;
byte rightMotorSpeed = 0;

#define LEFT_MOTOR_FORWARD 2
#define LEFT_MOTOR_BACKWARD 3
#define RIGHT_MOTOR_FORWARD 4
#define RIGHT_MOTOR_BACKWARD 5
#define BASE_SPEED 128

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  //This is very unoptimized and filled with bad programming practices,
  //but it works for now.
  if (Serial.available() > 0) {
    leftMotorSpeed = Serial.read();
    rightMotorSpeed = Serial.read();
  }

  //send telemetry data
  //
  // {DATA FROM SENSORS}
  //
  //Serial.writeln(data);

  // motor pwm loop
  if (leftMotorSpeed >> 7 == 1) {
    analogWrite(LEFT_MOTOR_FORWARD, (leftMotorSpeed & 127) != 0 ? BASE_SPEED + (leftMotorSpeed & 127) : 0);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
  } else {
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, leftMotorSpeed ? BASE_SPEED + leftMotorSpeed : 0);
  }
  if (rightMotorSpeed >> 7 == 1) {
    analogWrite(RIGHT_MOTOR_FORWARD, rightMotorSpeed & 127 != 0 ? BASE_SPEED + (rightMotorSpeed & 127) : 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  } else {
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, rightMotorSpeed ? BASE_SPEED + rightMotorSpeed: 0);
  }
}
