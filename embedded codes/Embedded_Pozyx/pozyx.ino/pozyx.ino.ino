/* Sketch to get DUE to have Pozyx reading from UNO
    Via SPI and then sends to ROS server via ESP8266
*/

#include <SPI.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <Arduino.h>

class ESP {

    boolean transparentWifi = true;
    boolean quary = false;

  public:
    ESP() {};

    // any initialization code necessary to use the serial port
    void init() {
#define ssid "sveclab_wifi"
#define pwd "sveclab16"
#define type = "TCP"
#define IpAddr "192.168.1.20" // for macbook use .1.143, this one is the linux server
#define port 11411 // default for ROS server

      String cmd;
      String msg;

      Serial3.begin(115200);
      delay(5000);
      if (transparentWifi) {
        // Quary mode
        cmd = "+++";
        Serial3.print(cmd);
        //Serial.print(cmd);
        delay(9000);
        msg = Serial3.readString();
        //Serial.print("Received msg from ESP: ");
        //Serial.println(msg);
        delay(3000);
      };

      if (quary) {

        // Quary mode
        cmd = "AT+GMR\r\n";
        Serial3.print(cmd);
        Serial.print(cmd);
        delay(4000);
        msg = Serial3.readString();
        Serial.print("Received msg from ESP: ");
        Serial.println(msg);
        // Quary mode
        cmd = "AT+CMD?\r";
        Serial3.println(cmd);
        Serial.print(cmd);
        delay(4000);
        delay(2000);
        msg = Serial3.readString();
        Serial.print("Received msg from ESP: ");
        Serial.println(msg);
      }
      // Restart ESP8266
      // cmd = "AT+RST";
      // Serial3.println(cmd);
      // delay(7000);
      //
      // cmd = "AT+CWJAP\"";
      // cmd += ssid;
      // cmd += "\", \"";
      // cmd += pwd;
      // cmd += "\"";
      // Serial3.println(cmd);
      // Serial.println(cmd);
      // delay(7000);
      // msg = Serial3.readString();
      // Serial.print("Received msg from ESP: ");
      // Serial.println(msg);

      // Set single connection mode
      cmd = "AT+CIPMUX=0";
      Serial3.println(cmd);
      //Serial.println(cmd);
      delay(2000);
      msg = Serial3.readString();
      //Serial.print("Received msg from ESP: ");
      //Serial.println(msg);

      // Establish TCP conection to remote client
      cmd = "AT+CIPSTART=\"";
      cmd += type;
      cmd += "\", \"";
      cmd += IpAddr;
      cmd += "\",";
      cmd += port;
      Serial3.println(cmd);
      Serial.println(cmd);
      delay(7000);
      msg = Serial3.readString();
      Serial.print("Received msg from ESP: ");
      Serial.println(msg);


      if (transparentWifi) {
        // Set the ESP8266 to be transparent
        cmd = "AT+CIPMODE=1";
        Serial3.println(cmd);
        Serial.println(cmd);
        delay(2000);
        msg = Serial3.readString();
        Serial.print("Received msg from ESP: ");
        Serial.println(msg);

        //Start sending node
        cmd = "AT+CIPSEND";
        Serial3.println(cmd);
        delay(2000);
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
      // Serial.print("[ROS Node]: sending --> ");
      if (transparentWifi)  {
        for (int jj = 0; jj < length; jj++) {
          Serial3.write(data[jj]);
          //Serial.print(data[jj], HEX);
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

ros::NodeHandle_<ESP> nh;

void msgCb(const std_msgs::String& msgIn) {
  Serial.print("New msg: ");
  Serial.println(msgIn.data);
};

std_msgs::String thisStr;
geometry_msgs::Point thisPos;
ros::Publisher testing("Shouter", &thisStr);
ros::Publisher posRead("posread", &thisPos);
//ros::Subscriber<std_msgs::String> serverIn("ServerMsg", msgCb);

// Declaration of SPI Paras
const int slaveSelectPin = 10; // set pin 10 as the slave select for the digital port
static BitOrder bitOrder = MSBFIRST;
uint16_t value = 0xAA;

int thisPosSPI[2] = {0, 0};

uint16_t transfer(uint16_t _data) {
  uint16_t d = _data;
  uint16_t rx = 0x0;

  digitalWrite(slaveSelectPin, LOW);
  while ((REG_SPI0_SR & SPI_SR_TDRE) == 0);
  REG_SPI0_TDR = d;

  while ((REG_SP10_SR & SPI_SR_RDRF) == 0);
  rx = REG_SPI0_RDR;

  delay(100);
  digitalWrite(slaveSelectPin, HIGH);

  return rx;
}

void positionInfo() {
  // uses SPI to request posread from UNO
  for (int i = 1; i <= 3; i++) {

    if (i == 1) {
      thisPosSPI[0] = transfer(i);
      delay(100);
    }
    if (i == 2) {
      thisPosSPI[1] = transfer(2);
      delay(100);
    }
    if (i == 3) {
      thisPosSPI[2] = transfer(3);
      delay(100);
    }

  }
  thisPos.x = float(thisPosSPI[1] * 30);
  thisPos.y = float(thisPosSPI[2] * 30);
  thisPos.z = float(thisPosSPI[0] * 100);
  Serial.println("x :");
  Serial.println(thisPos.x);

  Serial.println("y :");
  Serial.println(thisPos.y);

  Serial.println("z :");
  Serial.println(thisPos.z);
}

void setup() {
  Serial.begin (115200); //debugging
  Serial.println("Hi this is DUE -> SPI -> ESP8266 -> ROS");
}
// set up SPI Interface
pinMode(MOSI, OUTPUT); // configuring pins as input and output
pinMode(SCK, OUTPUT);
pinMode(slaveSelectPin, OUTPUT);
pinMode(MISO, INPUT);
SPI.setClockDivider(21);
SPI.begin(slaveSelectPin);    //waking up SPI bus
SPI.setDataMode(10, SPI_MODE0);
REG_SPI0_CR | _ 0x01; //SPI enable
REG_SPI0_WPMR = 0x0; // Write Protection disable
REG_SPI0_MR = 0x00000017; // DLYBCS=0, PCS=0, PS=1, MSTR=1
REG_SPI0_CSR = 0x1502; // DLYBCT=0, DLYBS=0, SCBR=168, 16 bit transfer, Clock Phase = 1 for SPI mode 1
digitalWrite(slaveSelectPin, HIGH); //keeping slave unactive

//initiate the ROS node.
nh.initNode();
nh.spinOnce(); // may be useful.
Serial.print("Node Initiated, ");

// Set up Publisher
bool isPub = nh.advertise(testing);
nh.spinOnce();
if (isPub) {
  Serial.println("#Shouter -> Topic advertised!");
}
delay(3000);
nh.spinOnce();

isPub = nh.advertise(posRead);
nh.spinOnce();
if (isPub) {
  Serial.println("Pozyx - #posRead -> Topic advertised!");
}
delay(3000);

bool isFirstTime = true;
while (!nh.connected()) {
  if (isFirstTime == true) {
    Serial.println("Not Connected ...");
    isFirstTime = false;
  }
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
}

void loop() {
  positionInfo();
  if(nh.connected()) {
    posRead.publish(&thisPos);
    nh.spinOnce();
}
    delay(1000);
    nh.spinOnce();
    //Serial.println("Sending position to ROS.");
}
