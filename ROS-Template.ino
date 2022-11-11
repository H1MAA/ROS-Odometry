// #include ALL MESSAGE TYPES HERE
#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

//MPU Library
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#define INTERRUPT_PIN PB14    // use pin 2 on Arduino Uno & most boards

MPU6050 mpu;

// MPU control/status vars
uint8_t fifoBuffer[256];      // FIFO storage buffer
Quaternion q;                 // [w, x, y, z]         quaternion container
VectorFloat gravity;          // [x, y, z]            gravity vector
float ypr[3];                 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//Node Declaration
ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
geometry_msgs::Pose pose_msg;

ros::Publisher imuNode("imu_data", &imu_msg);
ros::Publisher poseNode("Position_data", &pose_msg);


//DECLARE ALL NODES AND TOPICS HERE
//ros::Subscriber<std_msgs::MSG_TYPE> NAME("TOPIC", &messageCb );
//ros::Publisher NAME("TOPIC", &ex_msg);


//Hardware Serial for STM32
HardwareSerial Serial3(PB11, PB10);



#define BAUD 115200

//Encoder Pins
#define ENCODER_X1 PA8
#define ENCODER_X2 PA9
#define ENCODER_Y1 PA8
#define ENCODER_Y2 PA9

long long countX;
long long countY;

#define r 0.05 //5cm radius for wheels
#define pi = PI

long long posX;
long long posY;

void setup() {
  // put your setup code here, to run once:
  (nh.getHardware())->setPort(&Serial3);
  (nh.getHardware())->setBaud(BAUD);
  nh.initNode();
  //  nh.subscribe(NAME);
  //  nh.advertise(NAME);

  pinMode(ENCODER_X1, INPUT_PULLUP);
  pinMode(ENCODER_X2, INPUT_PULLUP);
  attachInterrupt(ENCODER_X1, ISR_encX1, CHANGE);
  attachInterrupt(ENCODER_X2, ISR_encX1, CHANGE);
  
  pinMode(ENCODER_Y1, INPUT_PULLUP);
  pinMode(ENCODER_Y2, INPUT_PULLUP);
  attachInterrupt(ENCODER_Y1, ISR_encY1, CHANGE);
  attachInterrupt(ENCODER_Y2, ISR_encY2, CHANGE);

  mpu_init(); //run all mpu init processes

//  advertise node
  nh.advertise(imuNode);
  nh.advertise(poseNode);
  
  imu_msg.header.frame_id = 0;
  imu_msg.orientation.x = q.x;
  imu_msg.orientation.y = q.y;
  imu_msg.orientation.z = q.z;
  imu_msg.orientation.w = q.w;


}

void loop() {
  // put your main code here, to run repeatedly:


  nh.spinOnce();

}

void ISR_encX1(void){
  if (digitalRead(ENCODER_X1) != digitalRead(ENCODER_X2))
    countX++;
  else
    countX--; 
}

void ISR_encX2(void){
  if (digitalRead(ENCODER_X1) == digitalRead(ENCODER_X2))
    countX++;
  else
    countX--;  
}


void ISR_encY1(void){
  if (digitalRead(ENCODER_Y1) != digitalRead(ENCODER_Y2))
    countY++;
  else
    countY--; 
}

void ISR_encY2(void){
  if (digitalRead(ENCODER_Y1) == digitalRead(ENCODER_Y2))
    countY++;
  else
    countY--;  
}


void mpu_init(void){
  mpu.initialize();
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  mpu.dmpInitialize();
 
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  
  // make sure it worked (returns 0 if so)
  //Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
       
  mpu.setDMPEnabled(true);  // turn on the DMP, now that it's ready

  //enable Arduino interrupt detection
  //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
  //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
  //Serial.println(F(")..."));
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
   
  
}


void mpu_getData(){
    
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
  
    //display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
//        Serial.print("ypr\t");
//        Serial.print(ypr[0] * 180/M_PI);
//        Serial.print("\t");
//        Serial.print(ypr[1] * 180/M_PI);
//        Serial.print("\t");
//        Serial.println(ypr[2] * 180/M_PI);
  } 
}

void calc_pos(){
//  TODO
}
