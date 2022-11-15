 //includes

#include <ros.h>
#include <SPI.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <Arduino.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <Servo.h>
#include <sensor_msgs/PointCloud.h>
#include <DueTimer.h>
#include <ros/node_handle.h>

//defines
#define ssid "sveclab_wifi"
#define pwd "sveclab16"
#define sockType "TCP"
#define IpAddr "192.168.1.3" //is the coumputer/server ip address 
//192.168.1.2 svec's lab locoal wifi ip address
#define port 11411 // default for ROS server
#define pinRearSteering  4
#define pinFrontSteering  6
#define pinFrontDrive  5
#define pinRearDrive  7
#define lidarRefPin  9     // the number of the pushbutton pin {CHECK NO NEED}
#define slaveSelectPin  10 // set pin 10 as the slave select for the digital port
#define value  0xAA

//stepper
int DIRpin = 11; //Direction pin
int STEPpin = 9; //Stepper pin

Servo rearSteering;
Servo frontSteering;
Servo rearDrive;
Servo frontDrive;
LIDARLite lidarLite;
std_msgs::String thisStr;
sensor_msgs::PointCloud thisPos;
sensor_msgs::PointCloud thisDist;
ros::Publisher testing("Shouter", &thisStr);
ros::Publisher posRead("posread", &thisPos);
ros::Publisher lidarRead("lidarread", &thisDist);
//ros::Subscriber<std_msgs::String> serverIn("ServerMsg", msgCb );

// Declaration of SPI Paras
volatile BitOrder bitOrder = MSBFIRST;
volatile static float lidarServoSpeed;
volatile unsigned long counter = 0;
volatile unsigned long pastLidarTime;
volatile unsigned int last_i;
volatile boolean state = false;
volatile unsigned int stepCount = 0;
volatile static float pii=3.141592654;
//volatile static float pii=3.14;
geometry_msgs::Point32 points_storage[300];
geometry_msgs::Point32 points_storage1[1];

char frameid[] = "/odom";
volatile unsigned long range_timer;
volatile signed long reading;
volatile float angle;
volatile unsigned int i;
volatile  unsigned long sendingFreq;

class ESP
{
    boolean transparentWifi = true;
    boolean quary = false;

  public:
    ESP() {};

    // any initialization code necessary to use the serial port
    void init() {
      String cmd;
      String msg;

      Serial3.begin(115200);
      delay(5000);
      // Restart ESP8266
          cmd = "AT+RST";
          Serial3.println(cmd);
          delay(7000);
          Serial.println(cmd);
          msg = Serial3.readString();
          Serial.print("Received msg from ESP: ");
          Serial.println(msg);
      
          cmd = "AT+CWMODE=1";
          Serial3.println(cmd);
          delay(5000);
          Serial.println(cmd);
          msg = Serial3.readString();
          Serial.print("Received msg from ESP: ");
          Serial.println(msg);
      
          cmd = "AT+CIPSTA=\"";
          cmd += "192.168.1.4";
          cmd += "\",\"";
          cmd += "192.168.1.2";
          cmd += "\",\"";
          cmd += "255.255.255.0";
          cmd += "\"";
          Serial3.println(cmd);
          delay(7000);
          Serial.println(cmd);
          msg = Serial3.readString();
          Serial.print("Received msg from ESP: ");
          Serial.println(msg);

          cmd = "AT+CWJAP=\"";
          cmd += ssid;
          cmd += "\",\"";
          cmd += pwd;
          cmd += "\"";
          Serial3.println(cmd);
          delay(7000);
          Serial.println(cmd);
          msg = Serial3.readString();
          Serial.print("Received msg from ESP: ");
          Serial.println(msg);

      cmd = "AT+CWJAP?";
      Serial3.println(cmd);
      delay(7000);
      Serial.println(cmd);
      msg = Serial3.readString();
      Serial.print("Received msg from ESP: ");
      Serial.println(msg);

      if (transparentWifi) {
        // Quary mode
        cmd = "+++";
        Serial3.print(cmd);
        //Serial.print(cmd);
        delay(2000);
        msg = Serial3.readString();
        //Serial.print("Received msg from ESP: ");
        //Serial.println(msg);
        delay(1000);
      };

      if (quary) {

        // Quary mode
        cmd = "AT+GMR\r\n";
        Serial3.print(cmd);
        Serial.print(cmd);
        delay(1000);
        msg = Serial3.readString();
        Serial.print("Received msg from ESP: ");
        Serial.println(msg);
        // Quary mode
        cmd = "AT+CMD?\r";
        Serial3.println(cmd);
        Serial.print(cmd);
        delay(1000);
        msg = Serial3.readString();
        Serial.print("Received msg from ESP: ");
        Serial.println(msg);
      }

      // Set single connection mode
      cmd = "AT+CIPMUX=0";
      Serial3.println(cmd);
      //Serial.println(cmd);
      delay(1000);
      msg = Serial3.readString();
      //Serial.print("Received msg from ESP: ");
      //Serial.println(msg);


      // Establish TCP connection to remote client
      cmd = "AT+CIPSTART=\"";
      cmd += sockType;
      cmd += "\",\"";
      cmd += IpAddr;
      cmd += "\",";
      cmd += port;
      Serial3.println(cmd);
      Serial.println(cmd);
      delay(2000);
      msg = Serial3.readString();
      Serial.print("Received msg from ESP: ");
      Serial.println(msg);

      if (transparentWifi) {
        // Set the ESP8266 to be transparent
        cmd = "AT+CIPMODE=1";
        Serial3.println(cmd);
        Serial.println(cmd);
        delay(1000);
        msg = Serial3.readString();
        Serial.print("Received msg from ESP: ");
        Serial.println(msg);

        // Start sending mode
        cmd = "AT+CIPSEND";
        Serial3.println(cmd);
        delay(1000);
        msg = Serial3.readString();
        Serial.print("Received msg from ESP: ");
        Serial.println(msg);
      }
    };

    // read a byte from the serial port. -1 = failure
    int read() {
      int msg0 = Serial3.read();
      return msg0;
    };

    // write data to the connection to ROS
    void write(uint8_t* data, int length) {
      //Serial.print("[ROS Node]: sending --> ");
      if (transparentWifi) {
        for (int jj = 0; jj < length; jj++) {
          Serial3.write(data[jj]);
          //Serial.print(data[jj],HEX);
        };
      } else {
        for (int jj = 0; jj < length; jj++) {
          Serial3.print(data[jj]);
          //Serial.print(data[jj],HEX);
        };
        Serial3.print("\r");
        Serial3.println();
      }
      Serial3.println();
      //Serial.print(" <-- length = ");
    };

    // returns milliseconds since start of program
    unsigned long time() {
      return millis();
    }
};

void msgCb(const std_msgs::String& msgIn) {
  Serial.print("New msg: ");
  Serial.println(msgIn.data);
};


uint16_t transfer(uint16_t _data) {
  uint16_t d = _data;
  uint16_t rx = 0x0;

  digitalWrite(slaveSelectPin, LOW);
  while ((REG_SPI0_SR & SPI_SR_TDRE) == 0);
  REG_SPI0_TDR = d;

  while ((REG_SPI0_SR & SPI_SR_RDRF) == 0);
  rx = REG_SPI0_RDR;

  delayMicroseconds(1000);
  digitalWrite(slaveSelectPin, HIGH);

  return rx;
}
ros::NodeHandle_<ESP> nh;
void positionInfo() {
  // uses SPI to request posread from UNO
  thisPos.points[0].x = float( transfer(1) * 2.0);
  thisPos.points[0].y = float( transfer(2) * 2.0);
  thisPos.points[0].z = 0.0;
}
void tick()
{

 
  digitalWrite(STEPpin, HIGH);
  //delay(1);
  digitalWrite(STEPpin, LOW);

  stepCount++;
  if(stepCount >= 1333)
  {
    stepCount = 0;
    
  }
 
}
void lidarInfo() 
{ 
  reading = lidarLite.distance();
  angle = ((float)stepCount / 1333.0) * 360.0;
}

void printout()
{
 //Serial.print(angle);
 // Serial.print(",");
 //Serial.println(reading);
 Serial.print(thisDist.points[i].x);
 Serial.print(",");
  Serial.print(thisDist.points[i].y);
   Serial.println(",");
  
}

/* delete
void refAngle() {
  noInterrupts();
  unsigned long t = micros();
  if (long(t - pastLidarTime) >= 150000)
  {
    if (state)
      counter = counter + 1;
    else {
      counter = 1;
      state = !state;
    }
    pastLidarTime = t;
    angle = 0;
  }
  interrupts();
}

void lidarSpeedCal()
{
  // We need to get the status to clear it and allow the interrupt to fire again
  noInterrupts();
  TC_GetStatus(TC2, 1);
  float revpersec = float(counter) / 10;
  lidarServoSpeed = revpersec * 2 * 3.1415;
  state = !state;
  interrupts();
}

void sensorReading()
{
  // We need to get the status to clear it and allow the interrupt to fire again
  noInterrupts();
  lidarInfo(last_i, sendingFreq);
  thisDist.points[i].x = reading * cos(angle);
  thisDist.points[i].y = reading * sin(angle);
  i = i + 1;
  if ( i >= 500) i = 0;
  interrupts();
}
*/

void setup() {
  Serial.begin (115200);   // debugging
  Serial.println("Hi this is DUE -> SPI -> ESP8266 -> ROS");

  // set up SPI Interface
  pinMode(MOSI, OUTPUT);    //Configuring pins as input and output
  pinMode(SCK, OUTPUT);
  pinMode(slaveSelectPin, OUTPUT);
  pinMode(MISO, INPUT);
  SPI.setClockDivider(21);
  SPI.begin(slaveSelectPin);                   // waking up SPI bus
  SPI.setDataMode(10, SPI_MODE0);
  REG_SPI0_CR |= 0x01;          // SPI enable
  REG_SPI0_WPMR = 0x0;  // Write Protection disable
  REG_SPI0_MR = 0x00000017;     // DLYBCS=0, PCS=0, PS=1, MSTR=1
  REG_SPI0_CSR = 0x1502;          // DLYBCT=0, DLYBS=0, SCBR=168, 16 bit transfer, Clock Phase = 1 for SPI mode 1
  digitalWrite(slaveSelectPin, HIGH);    //keeping slave device unactive
  
  thisPos.header.seq = 0;
  thisPos.header.frame_id =  frameid;

  thisPos.points = points_storage1;
  thisPos.points[0].x = 0;
  thisPos.points[0].y = 0;
  thisPos.points_length = 1;

  thisDist.header.seq = 0;
  thisDist.header.frame_id =  frameid;

  thisDist.points = points_storage;
  thisDist.points[0].x = 0;
  thisDist.points[0].y = 0;
  thisDist.points_length = 11;
  //initate ROS node.
  nh.initNode();
  nh.spinOnce(); // may be useful.
  Serial.print("Node Initiated, ");

  // Set up Publisher
  bool isPub =  nh.advertise(testing);
  nh.spinOnce();
  if (isPub) {
    Serial.println("#Shouter -> Topic advertised!");
  }
  delay(3000);
  nh.spinOnce();

  isPub =  nh.advertise(posRead);
  nh.spinOnce();
  if (isPub) {
    Serial.println("Pozyx - #posRead -> Topic advertised!");
  }
  delay(3000);

  isPub =  nh.advertise(lidarRead);
  nh.spinOnce();
  if (isPub) {
    Serial.println("Lidar - #lidarRead -> Topic advertised!");
  }
  delay(3000);
  // below added to wait for the param server to work.
  bool isFirstTime = true;
  while (!nh.connected()) {
    //if (isFirstTime == true) {
    Serial.println("Not Connected Yet...");
    isFirstTime = false;
    //}
    nh.spinOnce();
    delay(200);
  }
  if (nh.connected())
    Serial.println("[ROS] Node Handle Connected");

  nh.spinOnce();
  delay(200);

  thisStr.data = "Init";
  testing.publish(&thisStr);
  nh.spinOnce();
  /*
    pinMode(pinRearSteering, OUTPUT);
    pinMode(pinFrontSteering, OUTPUT);
    pinMode(pinFrontDrive, OUTPUT);
    pinMode(pinRearDrive, OUTPUT);
    rearSteering.attach(pinRearSteering);
    frontSteering.attach(pinFrontSteering);
    rearDrive.attach(pinRearDrive);
    frontDrive.attach(pinFrontDrive);
    rearSteering.write(90);
    delay(200);
    frontSteering.write(90);
    delay(200);
    rearDrive.write(45);
    delay(200);
    frontDrive.write(45);
  */
  /* DELETE
  pinMode(pinServo, OUTPUT);
  lidarServo.attach(pinServo);
  // init angle of servo inbetween two limitations
  lidarServo.writeMicroseconds(1350);
  delay(200);
*/
// stepper
  pinMode(DIRpin, OUTPUT);
  pinMode(STEPpin, OUTPUT);

//
 // pinMode(lidarRefPin, INPUT);// check if no need
//  attachInterrupt(digitalPinToInterrupt(lidarRefPin), refAngle, LOW );
  lidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  lidarLite.configure(0); // Change this number to try out alternate configurations
  angle = 0;
  range_timer = millis();
  pastLidarTime = micros();
  Timer1.attachInterrupt(tick).setFrequency(22.2*20).start();
  positionInfo();
}


void loop() {
  // start us sensor distance measurement
  lidarInfo();
  
  thisDist.points[i].x = ( reading * cos((angle*pii)/180))/100;//recheck if rad.
  thisDist.points[i].y = ( reading * sin((angle*pii)/180))/100;
  i = i + 1;
  if ( i >= 300) i = 0;
//printout();
  //publish x and y with ros
  // publish the range value every 150 milliseconds
  if ( millis() - range_timer > 60)
  {
    sendingFreq = millis() - range_timer;
    thisPos.header.stamp = nh.now();
    positionInfo();
    if (nh.connected()) {
      posRead.publish(&thisPos);
      nh.spinOnce();
    }
    thisDist.points_length = i;
    thisDist.header.stamp = nh.now();
    if (nh.connected()) {
      lidarRead.publish(&thisDist);
      nh.spinOnce();
    }
    //Serial.println("Sending position to ROS.");
    last_i = i;
    i = 0;
    range_timer =  millis();
  }
//  Serial.print("sent=");
//  Serial.print(last_i);
//  Serial.print("sent time=");
//  Serial.print(sendingFreq);
//  Serial.print("angle=");
//  Serial.print(angle);
//  Serial.print(" lidar speed=");
//  Serial.print(float(lidarServoSpeed));
//  Serial.print(" t1=");
//  Serial.print(range_timer);
//  Serial.print(" counter=");
//  Serial.println(counter);
}

