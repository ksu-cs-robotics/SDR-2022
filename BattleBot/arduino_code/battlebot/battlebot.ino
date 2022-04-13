#include "general.h"
#include "mpu.h"
#include "motor_driver.h"


MPU6050 mpu;
Turtlebot3MotorDriver motor_driver;


void setup() 
{  
  // Initalize MPU6050 
  dmpDataReady();     
  dmp_setup();        

  // Initalize dynamixel driver and odom
  motor_driver.init(NAME);
  initOdom();

  pinMode(led_pin, OUTPUT);
  prev_update_time = millis();
}


int32_t le=0;
int32_t re=0;
int32_t rst=0;
int32_t rst_err=0;
void loop() 
{
  static uint32_t tTime[4];

  // serial communication
  recieved_py();

  // IMU 
  dmp_loop();

  // motor control 
  motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);

  readMotorInfo();
  calcMotorInfo();
  
  if( (millis() - tTime[2]) >= 20 )
  {
    tTime[2] = millis();

//    shooting_led();

    String odom_temp[15] = {String(odom_pose[0]), String(odom_pose[1]), String(odom_pose[2]), String(odom_vel[0]), String(odom_vel[2])};
//    String(left_encoder), String(right_encoder), String(rst), String(rst_err),
//    String(motor_driver.err[0]), String(motor_driver.err[1]), String(motor_driver.err[2]), String(motor_driver.err[3]), String(motor_driver.err[4]), String(motor_driver.err[5]), };
    send_py(odom_temp, 5, ODOM_MSG);
  }

}
    

/////////////////////////////////////////
//              Robot                  //
/////////////////////////////////////////

void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

}

void readMotorInfo(void)
{
  // read motor
  bool dxl_comm_result = false;
  dxl_comm_result = motor_driver.readEncoder(left_encoder, right_encoder);

  if (dxl_comm_result == true){
    updateMotorInfo(left_encoder, right_encoder);
    rst++;
  }
  else{
    rst_err++;
    return;
  }
}

void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};

  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

void calcMotorInfo(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  
  calcOdometry((double)(step_time * 0.001));
}

bool calcOdometry(double diff_time)
{
//  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;
  
  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}

void shooting_led(){
  if (shoot == true){
    digitalWrite(led_pin, LOW);
    shootting_blink += 1;
    if (shootting_blink == 5){
      digitalWrite(led_pin, HIGH);
    }
    else if (shootting_blink == 10){
      digitalWrite(led_pin, LOW);
    }
    else if (shootting_blink == 15){
      digitalWrite(led_pin, HIGH);
      shootting_blink = 0;
      shoot = false;
    }
  }
}

/////////////////////////////////////////
//                Data                 //
/////////////////////////////////////////

void recieved_py(){
  if (Serial.available() > 0){
    char tmpChar = Serial.read();
    if ((msgBufferPointer == 0) && (tmpChar == '<')){
      msgBuffer[msgBufferPointer] = tmpChar; 
      msgBufferPointer++;
    }
    else if (msgBufferPointer == 1){
      if (tmpChar == '!'){ 
        msgBuffer[msgBufferPointer] = tmpChar; 
        msgBufferPointer++; 
      }
      else{
        if (tmpChar == '<'){
          msgBuffer[0] = tmpChar;
          msgBufferPointer = 1;
        }else{
          msgBufferPointer = 0;          
        }
      }
    }

    else if (msgBufferPointer >= 2){
      if (tmpChar == '<'){ 
        msgBuffer[0] = tmpChar;
        msgBufferPointer = 1;
      }
      else if (tmpChar == '>'){
        msgBuffer[msgBufferPointer] = tmpChar;
        msgBufferPointer = 0;
        evaluateCommand();
      }
      else{
        msgBuffer[msgBufferPointer] = tmpChar;
        msgBufferPointer++;
      }
    }
  }
}

void evaluateCommand(){
  char* command = strtok(msgBuffer, ",");
  int temp = 0;
  String type = "none";
  while (command != 0){
    if (temp == 1){
      if (String(command) == TORQUE_MSG){
        type = TORQUE_MSG;
      }
      else if (String(command) == CMD_MSG){
        type = CMD_MSG;
      }
    }

    if (type == TORQUE_MSG){
      if (temp == 3){
        if (String(command) == "true"){
          motor_driver.setTorque(true);
        }
        else if (String(command) == "false"){
          motor_driver.setTorque(false);
        }
      }
    }

    else if (type == CMD_MSG){
      if (temp == 3){
        goal_velocity[0] = String(command).toFloat();
      }
      else if (temp == 4){
        goal_velocity[1] = String(command).toFloat();
      }
    }

//    else if (type == GENERAL_MSG){
//      Serial.println("i am in########################################################");
//      if (temp == 3){
//        Serial.println("getting something------------------------------------------------------------");
//        if (String(command) == "shoot"){
//          Serial.println("shoot!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//          shoot = true;
//        }
//      }
//    }
    
    temp += 1;
    command = strtok(0, ",");
  }
}

void send_py(String data[], int size, String type){
  String data_out;
  data_out = "<!," + type + "," + String(size) + ",";
  for(int i = 0; i < size; i++){
    data_out += data[i];
    data_out += ",";
  }
  data_out += "#>";

  Serial.print(data_out);
  Serial.print("\n");
}
/////////////////////////////////////////
//                IMU                  //
/////////////////////////////////////////
/*---------------------------------------------------------------------------
     TITLE   : dmpDataReady
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void dmpDataReady() {
    mpuInterrupt = true;
}

/*---------------------------------------------------------------------------
     TITLE   : dmp_setup
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void dmp_setup() {
    I2Cdev::begin(400);
 
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //  while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
  //  while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(189);
    mpu.setYGyroOffset(44);
    mpu.setZGyroOffset(-17);
    mpu.setZAccelOffset(1534); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

/*---------------------------------------------------------------------------
     TITLE   : dmp_loop
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void dmp_loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        orientation[0] = q.w;
        orientation[1] = q.x;
        orientation[2] = q.y;
        orientation[3] = q.z;
        // store roll, pitch, yaw
//        yaw = ypr[0] * 180/M_PI;
//        roll = ypr[1] * 180/M_PI;
//        pitch = ypr[2] * 180/M_PI;                       
    }
}
