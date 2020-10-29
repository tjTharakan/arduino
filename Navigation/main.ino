#include <PID_v1.h>

/******************************************************************************

   Resources:
   This library uses the Arduino Wire.h to complete I2C transactions.

 ******************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_VL6180X.h>
#include <PID_v1.h>

#define VL6180X_ADDRESS     0x29   // Each slave device replaced with new address-
#define NEWVL6180X_ADDRESS  0x30   //  to enable I2C Communication
#define NEW1VL6180X_ADDRESS 0x31
#define NEW2VL6180X_ADDRESS 0x32

#define BAUDRATE  9600
#define PWM_MIN   0
#define PWM_MAX   255
#define PWM_INIT  200
#define RPM_MIN   0
#define RPM_MAX   60

#define BNO055_SAMPLERATE_DELAY_MS (100)


#define DELAY_100_MS               (100)
#define DELAY_200_MS               (200)

#define LOOPLENGTH                 (4300)                               /* Encoder Ticks count taken by the Robot to complete 180 degree */

#define GPIO_PIN2   2
#define GPIO_PIN3   3
#define GPIO_PIN4   4
#define GPIO_PIN5   5
#define GPIO_PIN6   6
#define GPIO_PIN7   7
#define GPIO_PIN8   8
#define GPIO_PIN9   9
#define GPIO_PIN11 11
#define GPIO_PIN24 24
#define GPIO_PIN26 26
#define GPIO_PIN28 28
#define GPIO_PIN30 30

#define DEBUG   0
#define RIGHT_WHL_PPR 2700
#define LEFT_WHL_PPR  2700
#define TO_MINUTES 600

#define RIGHT_DROP_THRESH 30
#define LEFT_DROP_THRESH 30
/* Initialize IMU Sensor*/
Adafruit_BNO055 BNO_IMU = Adafruit_BNO055(55, BNO055_ADDRESS_A); // Header config ADDRESS A - 0 * 27 ADDRESS B - 0 * 28

/*Initialize drop detection sensors */
VL6180xIdentification identification;

VL6180x rearLeftDropSensor(VL6180X_ADDRESS);
VL6180x frontLeftDropSensor(VL6180X_ADDRESS);
VL6180x frontRightDropSensor(VL6180X_ADDRESS);
VL6180x rearRightDropSensor(VL6180X_ADDRESS);

int enable1 = GPIO_PIN24; // GPIO PINS front right
int enable2 = GPIO_PIN26; // GPIO PINS rear right
int enable3 = GPIO_PIN28; // GPIO PINS rear left
int enable4 = GPIO_PIN30; // GPIO PINS front left

//3. Declare PWM variables for Pololu DRV8838 H-Bridge
double right_pwm_value  = PWM_INIT;
double left_pwm_value = PWM_INIT;

// Global Variables 

byte cornerCount = 0;
bool start = false;
bool loopingStarted = false;
bool firstTime = true;
bool robotTurning = false;
bool turnDirection = false;

//4. Declare Direction and PWM Pins for Pololu DRV8838 H-Bridge
//4.1 Motor 1 (left)
const int directionPinLeftWheel = GPIO_PIN8; //8 //8
const  int speedPinLeftWheel = GPIO_PIN9; // 9  //11 Needs to be a PWM pin to be able to control motor speed


//4.2 Motor 2 (right)
const int directionPinRightWheel = GPIO_PIN6;//6  //9
const int speedPinRightWheel = GPIO_PIN5; //5 //5  Needs to be a PWM pin to be able to control motor speed


// 5. GPIOs for the encoder inputs
// 5.1 Encoder 1_right
const byte  RH_ENCODER_A = GPIO_PIN2;
const byte  RH_ENCODER_B = GPIO_PIN4;

//timer variables
// 5.2 Encoder 2_left
const byte  LH_ENCODER_A = GPIO_PIN3;
const byte  LH_ENCODER_B = GPIO_PIN7;


//5.3 variables to store the number of encoder pulses for each motor
//changed to signed long variables to hold the negative values as well
volatile signed long leftEncoderCount = 0;
volatile signed long rightEncoderCount = 0, prevLeftEncoderCount = 0, prevRightEncoderCount = 0, tempLeftEncoderCount = 0, tempRightEncoderCount = 0;
volatile signed long globalLeftEncoderCount[4];
volatile signed long globalRightEncoderCount[4];
unsigned long previousTime, currentTime, timeDifference = 0;

// 7. Cleaning Components
int totalTime[4];
int startTime = 0, runningTime = 0;
int numberOfLoops = 0, loopsCompleted = 0;


const double leftWheelSetPoint = 35.00;                                                       /* The robot will move with a constant RPM of 40 RPM */
const double rightWheelSetPoint = 35.00;                                                      /* The robot will move with a constant RPM of 40 RPM */
double currentImuAngle = 0.0;
double previousImuAngle = 0.0;
double leftCalculatedRPM = 0.0;
double rightCalculatedRPM = 0.0;

//PID variables
double Pl = 3.6, Il = 0.99, Dl = 0.0;    //black robot model
double Pr = 3.6, Ir = 0.99, Dr = 0.0;    //black robot model

PID leftWheelPID(&leftCalculatedRPM, &left_pwm_value, &leftWheelSetPoint, Pl, Il, Dl, DIRECT);                   /* PID to maintain a constant RPM of setpoint value */
PID rightWheelPID(&rightCalculatedRPM, &right_pwm_value, &rightWheelSetPoint, Pr, Ir, Dr, DIRECT);               /* PID to maintain a constant RPM of setpoint value */

void setup() {
  pinMode (enable1, OUTPUT);
  pinMode (enable2, OUTPUT);
  pinMode (enable3, OUTPUT);
  pinMode (enable4, OUTPUT);


  Serial.begin(BAUDRATE);                                                                                         //Start Serial at 9600bps
  Wire.begin();                                                                                                   //Start I2C library
  delay(DELAY_100_MS);                                                                                            // delay .1s

  digitalWrite(enable1, HIGH);
  digitalWrite(enable2, LOW);
  digitalWrite(enable3, LOW);
  digitalWrite(enable4, LOW);


  /*Define Pololu DRV8838 H-Bridge Motor Controller No 1 for left motor Pins*/
  pinMode(directionPinLeftWheel, OUTPUT);
  pinMode(speedPinLeftWheel, OUTPUT);

  /*Define Pololu DRV8838 H-Bridge Motor Controller No 2 for right motor Pins*/
  pinMode(directionPinRightWheel, OUTPUT);
  pinMode(speedPinRightWheel, OUTPUT);


  /* Initialise encoders*/
  // Left encoder
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);

  //right encoder
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);

  /*Initialize hardware interrupts*/
  attachInterrupt(1, leftEncoderEvent, CHANGE);                                             /* Interrupt to count the left wheel encoder ticks */
  attachInterrupt(0, rightEncoderEvent, CHANGE);                                            /* Interrupt to count the right wheel encoder ticks */

  delay(DELAY_100_MS);

  rearLeftDropSensor.getIdentification(&identification);
  printIdentification(&identification);

  if (rearLeftDropSensor.VL6180xInit() != NULL) {
    Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
  };
  rearLeftDropSensor.VL6180xDefautSettings();
  delay(DELAY_200_MS); // delay 1s
  rearLeftDropSensor.changeAddress(VL6180X_ADDRESS, NEWVL6180X_ADDRESS);

  digitalWrite(enable2, HIGH);
  frontLeftDropSensor.getIdentification(&identification);

  if (frontLeftDropSensor.VL6180xInit() != NULL) {
    Serial.println("FAILED TO INITALIZE Sensor 2"); //Initialize device and check for errors

  };
  frontLeftDropSensor.VL6180xDefautSettings(); //Load default settings to get started.
  delay(DELAY_200_MS);
  frontLeftDropSensor.changeAddress(VL6180X_ADDRESS, NEW1VL6180X_ADDRESS);

  digitalWrite(enable3, HIGH);
  frontRightDropSensor.getIdentification(&identification);

  if (frontRightDropSensor.VL6180xInit() != NULL) {
    Serial.println("FAILED TO INITALIZE Sensor 3"); //Initialize device and check for errors

  };
  frontRightDropSensor.VL6180xDefautSettings(); //Load default settings to get started.
  delay(DELAY_200_MS);
  frontRightDropSensor.changeAddress(VL6180X_ADDRESS, NEW2VL6180X_ADDRESS);

  digitalWrite(enable4, HIGH);
  rearRightDropSensor.getIdentification(&identification);

  if (rearRightDropSensor.VL6180xInit() != NULL) {
    Serial.println("FAILED TO INITALIZE Sensor 4"); //Initialize device and check for errors

  };
  rearRightDropSensor.VL6180xDefautSettings(); //Load default settings to get started.
  delay(DELAY_200_MS);
  if (!BNO_IMU.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  sensors_event_t orientationData , linearAccelData;
  BNO_IMU.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  BNO_IMU.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);


  leftWheelPID.SetMode(AUTOMATIC);                              /* Enable PID for left wheel */
  rightWheelPID.SetMode(AUTOMATIC);                             /* Enable PID for left wheel */

}
void loop() {

  if (Serial.available() > 0) {                                 /* If any user input is available */
    int inByte = Serial.read();                                 /* Read the serially received user imput value */

    switch (inByte) {                                           /* Perform the actions based on received user command */

      case 'c': // Motor Forward
        motorForward();                                         /* Move the Robot in forward direction */
        start = true;                                           /* Make the variable true to mark the start of autonomous working */
        startTime = millis();                                   /* Mark the starting time of the autonmous mode */
        getIMUData();
        break;

      case 'w':
        if (!start)
          motorForward();                                         /* Move the Robot in forward direction */
        break;

      case 's':
        motorStop();                                            /* Stop the Robot motion */
        start = false;                                          /* Make the variable false to mark the end of autonomous working by user */
        break;

      case 'z':
        if (!start)
          motorBackward(500);
        break;

      case 'y':
        if (!start)
          motorBackward(500);                                     /* Move the Robot in backward direction */
        break;

      case 'd':
        if (!start)
          TurnRight();                                            /* Turn the Robot in Right direction */
        break;

      case 'a':
        if (!start)
          TurnLeft();                                             /* Turn the Robot in Left direction */
        break;
    }
  }
  if (start)                                                    /* If Autonomous mode is ON */
  {
    getIMUData();                                               /* Get the current IMU data to calculate deviation later to be given as input for PID calculation  */
#if(DEBUG)
    motorStop();
    while(1)
    {
      Serial.print("frontRightDropSensor:\t");
      Serial.println(frontRightDropSensor.getDistance());
      Serial.print("rearRightDropSensor:\t");
      Serial.println(rearRightDropSensor.getDistance());
      Serial.print("frontLeftDropSensor:\t");
      Serial.println(frontLeftDropSensor.getDistance());
      Serial.print("rearLeftDropSensor:\t");
      Serial.println(rearLeftDropSensor.getDistance());
    }
#endif

    leftCalculatedRPM =  (((leftEncoderCount - prevLeftEncoderCount ) * TO_MINUTES) / (LEFT_WHL_PPR));               /*Calculate the Left Wheel RPM for Black model*/
#if(DEBUG)
    Serial.print("Left calculated RPM:\t");
    Serial.println(leftCalculatedRPM);
#endif

    rightCalculatedRPM = (((rightEncoderCount - prevRightEncoderCount) * TO_MINUTES) / (RIGHT_WHL_PPR));              /*Calculate the right wheel RPM for Black model*/

#if(DEBUG)
    Serial.print("Right calculated RPM:\t");
    Serial.println(rightCalculatedRPM);
#endif

    leftWheelPID.Compute();                                                                           /* Compute the required PWM value to match the set point RPM using the PID algorithm*/
    rightWheelPID.Compute();                                                                          /* Compute the required PWM value to match the set point RPM using the PID algorithm*/

    if (loopingStarted)                                                                               /* IF robot is in inner area coverage navigation mode */
      right_pwm_value = right_pwm_value - 38;                                                         /* This value '38' is a temporary fix to counter the drift towards left side due to left wheel assembly imbalance.
                                                                                                         This must be fixed and PID should be fine tuned with the new model (PoC5) */
    analogWrite(speedPinLeftWheel, left_pwm_value);                                                   /* Apply the calculated PWM to the wheels */
    analogWrite(speedPinRightWheel, right_pwm_value);                                                 /* Apply the calculated PWM to the wheels */

#if(DEBUG)
    Serial.print("PWMs - Left : Right -----------> \t");
    Serial.print(left_pwm_value);
    Serial.print(":\t");
    Serial.println(right_pwm_value);
#endif

    prevLeftEncoderCount = leftEncoderCount;                                                          /* Store the current values as previous values for the next loop */
    prevRightEncoderCount = rightEncoderCount;                                                        /* Store the current values as previous values for the next loop */

#if(DEBUG)
    Serial.print("Left Encoder ---------------------->\t");
    Serial.println(leftEncoderCount);
    Serial.print("Right Encoder ---------------------->\t");
    Serial.println(rightEncoderCount);
#endif

    if (( frontRightDropSensor.getDistance() >= RIGHT_DROP_THRESH))                                                  /* If front right drop sensor has detected a drop */
    {
      motorStop();
      delay(DELAY_100_MS);
      motorForward();
      delay(150);
      motorStop();
      delay(200);

      if ((frontLeftDropSensor.getDistance() >= LEFT_DROP_THRESH))                                                  /* If front left sensor also has detected a drop */
      { // Corner Detected
        if (!loopingStarted) {
          updateData();                                                                                /* Method called to update the encoder ticks into the array for every corner detected */
          cornerCount = cornerCount + 1;                                                               /* Increment the corner counter */
          cornerDetected();                                                                            /* If both left and right drop sensors detect a drop, consider it as an edge/corner */
          robotTurning = false;                                                                        /* This variable is used to control the increment of encoder ticks. */
        }
      }
      else {
        if ((firstTime) && (cornerCount < 1))                                                          /* This should execute only for the first detection of edge */
        {
          robotTurning = true;                                                                         /* Disable increment of encoder ticks. */
          alignTheEdge();                                                                              /* Align the robot front parallel to the mattress edge */
          robotTurning = false;                                                                        /* Enable increment of encoder ticks. */
          motorBackward(2500);
          robotTurning = true;
          turn(false, 90.00);                                                                          /* Turn the robot 90 degree to the RIGHT */
          robotTurning = false;
          firstTime = false;                                                                           /* Make the variable false as the loop shouldnt be executed more than once */
        }
        else {
          robotTurning = true;
          turnLeftWheel();                                                                             /* Turn Left wheel to bring the robot back in alignment with the mattress edge */
          robotTurning = false;
        }
      }
    }
    else if (( frontLeftDropSensor.getDistance() >= LEFT_DROP_THRESH))                                                /* If front right sensor also has detected a drop */
    {
      motorStop();
      delay(500);
      motorForward();
      delay(300);
      motorStop();

      if (( frontRightDropSensor.getDistance() >= RIGHT_DROP_THRESH))                                                   /* If both right and left drop sensors detect a drop, consider it as an edge/corner */
      {
        if (!loopingStarted) {
          updateData();                                                                                  /* Method called to update the encoder ticks into the array for every corner detected */
          cornerCount = cornerCount + 1;                                                                 /* Increment the corner counter */
          cornerDetected();                                                                              /* If both left and right drop sensors detect a drop, consider it as an edge/corner */
          robotTurning = false;
        }
      }
      else {
        if ((firstTime) && (cornerCount < 1))                                                            /* This should execute only for the first detection of corner */
        {
          robotTurning = true;
          alignTheEdge();                                                                                /* Align the robot front parallel to the mattress edge */
          robotTurning = false;
          motorBackward(2500);
          robotTurning = true;
          turn(false, 90.00);                                                                            /* Turn the robot 90 degree to the RIGHT */
          robotTurning = false;
          firstTime = false;                                                                             /* Make the variable false as the loop shouldnt be executed more than once */
        }
        else
        {
          robotTurning = true;
          turnRightWheel();                                                                              /* Turn Right wheel to bring the robot back in alignment with the mattress edge */
          robotTurning = false;
        }

      }

    }

    if (loopingStarted)                                                                                  /* Check whether the Robot is still in edge following mode or has started to clean the inner area of the mattress */
    {
      if (leftEncoderCount > (tempLeftEncoderCount - 6500))                                              /* check whether the encoder ticks has reached the desired count to take 180 degree turn */
      {
        motorStop();

        loopsCompleted = loopsCompleted + 1;                                                             /* Increment the count of the numner of completed loops */
        motionInLoop(loopsCompleted);                                                                    /* Method called at the end of every loop to update data and turn the robot 180 degree */
      }
      else                                                                                               /* If encoder ticks count has not reached the desired count */
      {
#if(DEBUG)
        Serial.print("temp count ------- >\t");
        Serial.println(tempLeftEncoderCount - 6500);
#endif
      }

    }

    motorForward();                                                                                      /* Continue forward */
  }
  delay(DELAY_100_MS);
}

void printIdentification(struct VL6180xIdentification *temp) {

  Serial.println(temp->idModel);
  Serial.print(temp->idModelRevMajor);
  Serial.println(temp->idModelRevMinor);

}

/*
   Method to increment the right encoder ticks. The encoder ticks are
   incremented only during forward motion and are disabled during turns
   and backward movements to make the encoder count more accurate as the
   motion of the robot to cover the inner area of mattress purely depends
   on this value
*/
void rightEncoderEvent() {
  if (!robotTurning)
  {
    if (digitalRead(RH_ENCODER_A) == HIGH) {
      if (digitalRead(RH_ENCODER_B) == LOW) {
        rightEncoderCount++;
      } else {
        //rightEncoderCount--;
      }
    } else {
      if (digitalRead(RH_ENCODER_B) == LOW) {
        //rightEncoderCount--;
      } else {
        //rightEncoderCount++;
      }
    }
  }
}

/*
   Method to increment the left encoder ticks. The encoder ticks are
   incremented only during forward motion and are disabled during turns
   and backward movements to make the encoder count more accurate as the
   motion of the robot to cover the inner area of mattress purely depends
   on this value
*/
void leftEncoderEvent() {
  if (!robotTurning)
  {
    if (digitalRead(LH_ENCODER_A) == HIGH) {
      if (digitalRead(LH_ENCODER_B) == LOW) {
        leftEncoderCount++;
      } else {
        //leftEncoderCount--;
      }
    } else {
      if (digitalRead(LH_ENCODER_B) == LOW) {
        // leftEncoderCount--;
      } else {
        //leftEncoderCount++;
      }
    }
  }
}

/*
   This method is called to turn the Robot 90 degree to right or left.
   The parameter passed values are LEFT : 1 and RIGHT 0
   This method will be called whenever the robot detects a corner.
*/

void turn(int turnDirection, int angle) {

  float currentAngle = 0;
  float desiredAngle = 0;
  float tempAngle = 0;
  float prevAngle = 0;
  robotTurning = true;

  imu::Vector<3> imuForTurn = BNO_IMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  imuForTurn = BNO_IMU.getVector(Adafruit_BNO055::VECTOR_EULER);                         /*Read current Euler Angle for every iteration */
  currentAngle = imuForTurn.x();

  if (turnDirection) {

    desiredAngle = currentAngle - 90.00 ;                                                 /* Calculate the angle to achieve 90 degree turn */
    prevAngle = currentAngle;                                                            /* Load current angle value as previous angle*/

#if(DEBUG)
    Serial.print("CurrentAngle\t");
    Serial.println(currentAngle);
    Serial.print("desiredAngle\t");
    Serial.println(desiredAngle);
#endif

    if (desiredAngle < 0)  // orientation values in negative
    {
      tempAngle = map ( desiredAngle, -90.00, -1.00, 270.00, 360.00 );                  /* Map the target angle to a range of 270 to 360 if current angle is between 0 to 90.
                                                                                           Because, the angle value decreases when robot is turning left (value goes CCW) and
                                                                                           counts from 360 after 0 degree. */
      TurnLeft();
      delay(200);
      while ((tempAngle <= currentAngle) || (currentAngle <= prevAngle))
      {
        imuForTurn = BNO_IMU.getVector(Adafruit_BNO055::VECTOR_EULER); //Read current Euler Angle for every iteration
        currentAngle = imuForTurn.x();
        delay(50);
#if(DEBUG)
        Serial.print("CurrentAngle\t");
        Serial.println(currentAngle);
        Serial.print("desiredAngle\t");
        Serial.println(desiredAngle);
        Serial.print("TempAngle\t");
        Serial.println(tempAngle);
#endif
      }
      motorStop();
    }
    else //Non negative values
    {
      TurnLeft();
      delay(200);
      while (currentAngle >= desiredAngle)
      {
        imuForTurn = BNO_IMU.getVector(Adafruit_BNO055::VECTOR_EULER); //Read current Euler Angle for every iteration
        currentAngle = imuForTurn.x();
        delay(50);
#if(DEBUG)
        Serial.print("CurrentAngle\t");
        Serial.println(currentAngle);
        Serial.print("desiredAngle\t");
        Serial.println(desiredAngle);
#endif
      }
    }
  }
  else if (!turnDirection)
  {
    desiredAngle = currentAngle + 90.00; //90° sharp turn
    prevAngle = currentAngle;
#if(DEBUG)
    Serial.print("CurrentAngle\t");
    Serial.println(currentAngle);
    Serial.print("desiredAngle\t");
    Serial.println(desiredAngle);
#endif
    if (desiredAngle > 360)  // orientation values in negative
    {
      tempAngle = map ( desiredAngle, 361, 450, 0.00, 90.00 );                            /* Map the target angle to a range of 0 to 90 if current angle is between 270 to 360.
                                                                                           Because, the angle value increases when robot is turning left (value goes CW) and
                                                                                           counts from 0 after 360 degree. */
      TurnRight();
      delay(200);
      while ((tempAngle >= currentAngle) || (currentAngle >= prevAngle))
      {
        imuForTurn = BNO_IMU.getVector(Adafruit_BNO055::VECTOR_EULER); //Read current Euler Angle for every iteration
        currentAngle = imuForTurn.x();
        delay(50);
#if(DEBUG)
        Serial.print("CurrentAngle\t");
        Serial.println(currentAngle);
        Serial.print("desiredAngle\t");
        Serial.println(desiredAngle);
        Serial.print("TempAngle\t");
        Serial.println(tempAngle);
#endif
      }
      motorStop();
    }
    else //Non negative values
    {
      TurnRight();
      delay(200);
      while (desiredAngle >= currentAngle)
      {
        imuForTurn = BNO_IMU.getVector(Adafruit_BNO055::VECTOR_EULER); //Read current Euler Angle for every iteration
        currentAngle = imuForTurn.x();
        delay(50);
#if(DEBUG)
        Serial.print("CurrentAngle\t");
        Serial.println(currentAngle);
        Serial.print("desiredAngle\t");
        Serial.println(desiredAngle);
#endif
      }
    }
  }
}

/*
   This method is called to move the Robot backward
*/
void motorBackward(int delayValue)
{
  analogWrite(speedPinLeftWheel, PWM_INIT);
  analogWrite(speedPinRightWheel, PWM_INIT);
  digitalWrite(directionPinLeftWheel, LOW);
  digitalWrite(directionPinRightWheel, HIGH);
  delay(delayValue);
}

/*
   This method is called to move the Robot forward
*/
void motorForward()
{
  analogWrite(speedPinLeftWheel, left_pwm_value);
  analogWrite(speedPinRightWheel, right_pwm_value);
  digitalWrite(directionPinLeftWheel, HIGH);
  digitalWrite(directionPinRightWheel, LOW);

}
/*
   This method is called to stop the Robot
*/

void motorStop()
{
  analogWrite(speedPinLeftWheel, 0);
  analogWrite(speedPinRightWheel, 0);
  digitalWrite(directionPinLeftWheel, LOW);
  digitalWrite(directionPinRightWheel, LOW);
}

/*
   The method is called when Right drop sensor detects a drop.
   The robot is made to rotate left to align the robot parallel to edge.
*/
void turnLeftWheel()
{
  robotTurning = true;
  motorBackward(1000);
  motorStop();
#if(DEBUG)
  Serial.println("Turning Left Wheel\t");
#endif
  {
    analogWrite(speedPinRightWheel, right_pwm_value);
    digitalWrite(directionPinRightWheel, LOW);
    analogWrite(speedPinLeftWheel, left_pwm_value);
    digitalWrite(directionPinLeftWheel, LOW);
    delay(150);
  }
  motorStop();
}

/*
   The method is called when Left drop sensor detects a drop.
   The robot is made to rotate right to align the robot parallel to edge
*/
void turnRightWheel()
{
  robotTurning = true;
  motorBackward(1000);
  motorStop();
#if(DEBUG)
  Serial.println("Turning Right Wheel\t");
#endif
  {
    analogWrite(speedPinRightWheel, right_pwm_value);
    digitalWrite(directionPinRightWheel, HIGH);
    analogWrite(speedPinLeftWheel, left_pwm_value);
    digitalWrite(directionPinLeftWheel, HIGH);
    delay(150);
  }
  motorStop();
}

void getIMUData()
{
  imu::Vector<3> imuForPID = BNO_IMU.getVector(Adafruit_BNO055::VECTOR_EULER); //Read current Euler Angle for every iteration
  imuForPID = BNO_IMU.getVector(Adafruit_BNO055::VECTOR_EULER); //Read current Euler Angle for every iteration
  currentImuAngle = imuForPID.x();
}

/*
   This method is called everytime a corner is detected by the robot.
   The method is called to update the counters and arrays.
*/
void updateData()
{
  runningTime = millis();

  globalLeftEncoderCount[cornerCount] = leftEncoderCount;

  globalRightEncoderCount[cornerCount] = rightEncoderCount;

  //totalTime[cornerCount] = runningTime - startTime;
#if(DEBUG)
  Serial.print("Corner Count ----------------------------------------> \t");
  Serial.println(cornerCount);
  Serial.print("Left Encoder Count  ---------------------------------> \t");
  Serial.println(globalLeftEncoderCount[cornerCount]);
  Serial.print("Right Encoder Count ---------------------------------> \t");
  Serial.println(globalRightEncoderCount[cornerCount]);
#endif
}

/*
   This method calculates the required number of loops (180 degree turns )
   to cover the inner area of the mattress. The loop count  is calculated based on
   the encoder ticks count stored in the area during each corener detection.
   LOOPLENGTH is the number of ticks per 180 degree turn. Total length of mattress
   divided by the ticks for 180 degree turn gives the number of loops.
   The calculated loops count is returned.
*/
signed long countTheLoops(signed long leftTicks, signed long rightTicks)
{
  signed long averageDistance = (leftTicks + rightTicks) / 2;
  numberOfLoops = (averageDistance / LOOPLENGTH) + 2;
#if(DEBUG)
  Serial.println("************** STARTING LOOPING ************************************)");
  Serial.print("Number of Loops ------------------------------->\t");
  Serial.println(numberOfLoops);
#endif
  return numberOfLoops;
}

/*
   This method is responsible for turning the robot 180 degree for the required
   number of times.
*/
void motionInLoop(int loopsCompleted)
{
  motorStop();
#if(DEBUG)
  Serial.println("*********************** LOOPING ************************************)");
#endif
  if (loopsCompleted != numberOfLoops )
  {
    if (loopsCompleted == 0) {
      turnDirection = false;
      motorForward();
    }
    else {
      robotTurning = true;
      turn(turnDirection, 90);                        /* Turn the bot 90 degree */
      motorStop();
      delay(DELAY_100_MS);
      turn(turnDirection, 90);                        /* Turn the bot 90 degree once again to achieve a complete 180 degree turn */
      motorStop();
      robotTurning = false;
      turnDirection = !turnDirection;                 /* Toggle the turn direction as the direction of turn switches when travelling in loops */
      leftEncoderCount = 0;                           /* Reset the encoder ticks count to avoid suddent variation in RPM and also to maintain a constant RPM */
      prevLeftEncoderCount = 0;
      rightEncoderCount = 0;
      prevRightEncoderCount = 0;
      motorBackward(200);
      motorStop();
      delay(DELAY_100_MS);
      motorForward();
    }
#if(DEBUG)
    Serial.print("Number of Loops ------------------------------->\t");
    Serial.println(numberOfLoops);
    Serial.print("Number of Loops Completed --------------------->\t");
    Serial.println(loopsCompleted);
#endif
  }
  else if (loopsCompleted == numberOfLoops ) {
    Serial.println("CLEANING COMPLETED");
   
    
    while (1);                                        /* End of algorithm. In the next PoC4 SW version,Replace this 
                                                         blocking call with a method to reset all counters, flags 
                                                         and variables so that the bot is back to ready state for 
                                                         next manual/autonomous cleaning.*/      
  }
}

/*
   This method is called when the robot detects a corner.
   Both the drop sensors must detect a drop at the same time
   for this method to be called.
*/
void cornerDetected()
{
  motorStop();
  if (cornerCount < 5) {
    motorBackward(2500);
    turn(turnDirection, 90.00);
    startTime = millis();
    leftEncoderCount = prevLeftEncoderCount = 0;
    rightEncoderCount = prevRightEncoderCount = 0;
    motorStop();
    // gotoEdge();
  }
  if (cornerCount == 5)
  {
    motorStop();
    delay(DELAY_100_MS);
    motorBackward(2000);
    motorStop();
    motorBackward(1000);
    Serial.println("Corner 5, Bot should turn here");
    turn(false, 90.00);
    // alignRearEnd();
    leftEncoderCount = 0;
    prevLeftEncoderCount = 0;
    rightEncoderCount = 0;
    prevRightEncoderCount = 0;
    tempLeftEncoderCount = globalLeftEncoderCount[cornerCount - 2];                     /* Required number of left encoder ticks to be travelled before 180 degree turn */
    tempRightEncoderCount = globalRightEncoderCount[cornerCount - 2];                   /* Required number of right encoder ticks to be travelled before 180 degree turn */
#if(DEBUG)
    Serial.println("5 corners detected");
    for (int i = 0; i < cornerCount; i++) {

      Serial.print(" Left Encoder Count:");
      Serial.print(i);
      Serial.print("\t");
      Serial.println(globalLeftEncoderCount[i]);

      Serial.print(" Right Encoder Count:");
      Serial.print(i);
      Serial.print("\t");
      Serial.println(globalRightEncoderCount[i]);
    }
#endif
    if (numberOfLoops == 0) {
      numberOfLoops = countTheLoops(globalLeftEncoderCount[2], globalRightEncoderCount[2]);
    }

    loopingStarted = true;                                                              /* Variable set to indicate the robot has started to cover the inner area of mattress */
    motionInLoop(loopsCompleted);
  }
}

/*
   This method is called only once. It is called to align the
   robot parallelly with the edge of the mattress. This method is
   called when either of the drop sensors detect a drop for
   the very first time after the start of the robot.
*/
void alignTheEdge()
{
  robotTurning = true;
#if(DEBUG)

  Serial.println("Aligning the Edge");
#endif
  if (( frontRightDropSensor.getDistance() >= RIGHT_DROP_THRESH))
  {
    analogWrite(speedPinRightWheel, right_pwm_value);
    digitalWrite(directionPinRightWheel, HIGH);
    analogWrite(speedPinLeftWheel, left_pwm_value);
    digitalWrite(directionPinLeftWheel, HIGH);
    while ((frontLeftDropSensor.getDistance() <= LEFT_DROP_THRESH))
    {
#if(DEBUG)
      Serial.println(frontLeftDropSensor.getDistance());
#endif
    }

  }
  else if (( frontLeftDropSensor.getDistance() >= LEFT_DROP_THRESH))
  {
    analogWrite(speedPinRightWheel, right_pwm_value);
    digitalWrite(directionPinRightWheel, LOW);
    analogWrite(speedPinLeftWheel, left_pwm_value);
    digitalWrite(directionPinLeftWheel, LOW);
    while ((frontRightDropSensor.getDistance() <= RIGHT_DROP_THRESH))
    {
#if(DEBUG)
      Serial.println(frontRightDropSensor.getDistance());
#endif
    }
  }
  motorStop();

#if(DEBUG)
  Serial.println("Aligning the Edge Completed");
#endif
  leftEncoderCount = 0;
  prevLeftEncoderCount = 0;
  rightEncoderCount = 0;
  prevRightEncoderCount = 0;
}

/*
   Turns the Robot to RIGHT when in Manual Control.
   The Robot will continue to turn RIGHT until next
   command is received from user.
*/
void TurnRight()
{
  robotTurning = true;
  digitalWrite(directionPinLeftWheel, HIGH);
  digitalWrite(directionPinRightWheel, HIGH);
  analogWrite(speedPinLeftWheel, left_pwm_value);
  analogWrite(speedPinRightWheel, 0);
  Serial.print("Matress Cleaner turns right");
  Serial.println("-------------------------------");

}

/*
   Turns the Robot to LEFT when in Manual Control.
   The Robot will continue to turn LEFT until next
   command is received from user.
*/
void TurnLeft()
{
  robotTurning = true;
  digitalWrite(directionPinLeftWheel, LOW);
  digitalWrite(directionPinRightWheel, LOW);
  analogWrite(speedPinLeftWheel, 0);
  analogWrite(speedPinRightWheel, right_pwm_value);
  Serial.print("Matress Cleaner turns left");
  Serial.println("-------------------------------");
}

/*
   This method is to take the robot closer to the left edge
   of the mattress when the robot is travelling in CCW direction.
   This method can be used after a turn at the corner to take the bot
   closer to the edge for edge following functionality.
*/
void gotoEdge()
{
  if (!( frontLeftDropSensor.getDistance() >= LEFT_DROP_THRESH)) {
    robotTurning = true;
    while (!( frontLeftDropSensor.getDistance() >= LEFT_DROP_THRESH)) {
      digitalWrite(directionPinLeftWheel, HIGH);
      digitalWrite(directionPinRightWheel, LOW);
      analogWrite(speedPinLeftWheel, 30);
      analogWrite(speedPinRightWheel, right_pwm_value);
    }
  }
  motorStop();
  digitalWrite(directionPinLeftWheel, LOW);
  digitalWrite(directionPinRightWheel, HIGH);
  analogWrite(speedPinLeftWheel, right_pwm_value - 50);
  analogWrite(speedPinRightWheel, right_pwm_value);
  motorStop();
  robotTurning = false;
}
/*
   This method is to align the rear end of the robot parallel
   with the edge of the mattress based on the rear end drop
   sensor output. This method can be used to align the angle
   of the robot before forward straight drive.
*/
void alignRearEnd() {

  analogWrite(speedPinLeftWheel, PWM_INIT);
  analogWrite(speedPinRightWheel, PWM_INIT);
  digitalWrite(directionPinLeftWheel, LOW);
  digitalWrite(directionPinRightWheel, HIGH);

  while ((rearLeftDropSensor.getDistance() <= LEFT_DROP_THRESH) && (rearRightDropSensor.getDistance() <= RIGHT_DROP_THRESH))
  {
    /* Do nothing */
  }

  motorStop();
  if ((rearLeftDropSensor.getDistance() >= LEFT_DROP_THRESH)) {
    while ((rearRightDropSensor.getDistance() <= RIGHT_DROP_THRESH))
    {
      analogWrite(speedPinLeftWheel, 0);
      analogWrite(speedPinRightWheel, 120);
      digitalWrite(directionPinLeftWheel, LOW);
      digitalWrite(directionPinRightWheel, HIGH);

    }
    motorStop();
  }
  else if ((rearRightDropSensor.getDistance() >= RIGHT_DROP_THRESH))
  {
    while ((rearLeftDropSensor.getDistance() <= LEFT_DROP_THRESH))
    {
      analogWrite(speedPinLeftWheel, 120);
      analogWrite(speedPinRightWheel, 0);
      digitalWrite(directionPinLeftWheel, LOW);
      digitalWrite(directionPinRightWheel, HIGH);

    }
    delay(DELAY_100_MS);
    motorStop();

  }

  motorForward();
  delay(1000);
  motorStop();
  delay(DELAY_100_MS);
}
