//reading pozyx cordinates and sending it to ros via serial port

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

//creating the node handle 
ros::NodeHandle  nh;
geometry_msgs::Twist pozyx_cordinates;
ros::Publisher pub_pozyx( "cordinates", &pozyx_cordinates);

// debug params
bool isPozyxBegin = false;

char disp;
// -----------------------------------------------------------

// Anchors ID Declaration
uint16_t anchors[6] = {0x6710, 0x6767, 0x6759, 0x6703, 0x673c, 0x675e}; // network ids

// Manual Calibration
int32_t anchors_x[6] = {0, 3375, 8438, 8759,6000,332};              // anchor x-coorindates in mm
int32_t anchors_y[6] = {234, 502,0 ,2600, 2600, 2946};              // anchor y-coordinates in mm
int32_t heights_z[6] = {2050, 537, 1896, 1449, 886, 1703};          // anchor z-coordinates in mm

// Position Algorithms
uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;
uint8_t dimension = POZYX_3D;
coordinates_t position;// set up the buffer to store pos


// -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  // initiialzing the node
   nh.initNode();
   nh.advertise(pub_pozyx);

  // prepare the Pozyx shield
  if(Pozyx.begin(true, MODE_INTERRUPT,  POZYX_INT_MASK_ALL, POZYX_INT_PIN1) == POZYX_FAILURE){
    delay(100);
    Serial.println(F("Pozyx Initialise Failure."));
    delay(1000);
    Serial.println(F("Aborting."));
    abort(); //the abort is cancelled but entering debug mode
  }
  else{
    isPozyxBegin = true; //means pozyx succeeds!
    delay(400);
    Serial.println(F("Pozyx Initialise Success."));
    Pozyx.clearDevices(NULL);// clear all previous devices in the device list
    setAnchorsManual();// Do manual calibration
    
  }
  
}

//-------------END OF SETUP -----------------


void loop() {
  // get pozyx reading
  Pozyx.doPositioning(&position, POZYX_3D, POZYX_POS_ALG_UWB_ONLY);
  delay(800);
  printCoordinates(position);
}

// -----------------------------------------------------------
void printCoordinates(coordinates_t coor){
  
    
    // publish
    pozyx_cordinates.linear.x=coor.x;
    pozyx_cordinates.linear.y=coor.y;
    pozyx_cordinates.linear.z=coor.z;
    
  pub_pozyx.publish(&pozyx_cordinates);
  nh.spinOnce();
  delay(10);
} // this function may be redundant, reserved here for debugging purpose.




void setAnchorsManual(){
  for (int i = 0; i < 4; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights_z[i];
    Pozyx.addDevice(anchor,NULL);
  }
}
