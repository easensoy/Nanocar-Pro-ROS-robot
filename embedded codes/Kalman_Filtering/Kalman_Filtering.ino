
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <stdlib.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t remote_id = 0x6000;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID

boolean use_processing = false;                         // set this to true to output data for the processing sketch

//creating the node handle
ros::NodeHandle  nh;
geometry_msgs::Vector3 pozyx_cordinates;
ros::Publisher pub_pozyx( "pozyx_cordinates", &pozyx_cordinates);



/// anchor codinates set0UP
const uint8_t num_anchors = 6; // the number of anchors

uint16_t anchors[num_anchors]  =  {0x6710, 0x6767, 0x6759, 0x6703, 0x670c, 0x675e};       // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[num_anchors] =  {0, 3954, 8389, 8814, 5963, 370};                       // anchor x-coorindates in mm
int32_t anchors_y[num_anchors] =  {0 , 0 , 0 , 2582, 2610, 2732};                         // anchor y-coordinates in mm
int32_t heights[num_anchors]   =   {2088, 1008, 1898, 1469, 878, 1723};                   // anchor z-coordinates in mm
             // anchor z-coordinates in mm

uint8_t algorithm = POZYX_POS_ALG_TRACKING;             // positioning algorithm Kalman filtering to use. try POZYX_POS_ALG_TRACKING for fast moving objects.
uint8_t dimension = POZYX_3D;                           // positioning dimension
int32_t height = 1000;                                  // height of device, required in 2.5D positioning


////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);
  // initiialzing the node
  nh.initNode();
  nh.advertise(pub_pozyx);

  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

  if(!remote){
    remote_id = NULL;
  }


  // clear all previous devices in the device list
  Pozyx.clearDevices(remote_id);
  // sets the anchor manually
  setAnchorsManual();
  // sets the positioning algorithm
  Pozyx.setPositionAlgorithm(algorithm, dimension, remote_id);


  delay(2000);

  Serial.println(F("Starting positioning: "));
}

void loop(){
  coordinates_t position;
  int status;
  
    status = Pozyx.doPositioning(&position, dimension, algorithm);

  if (status == POZYX_SUCCESS){
    // prints out the result
    printCoordinates(position);
  }else{
    // prints out the error code
    printErrorCode("positioning");
  }
}

// prints the coordinates for either humans or for processing
void printCoordinates(coordinates_t coor){
// publish
  pozyx_cordinates.x = coor.x;
  pozyx_cordinates.y = coor.y;
  pozyx_cordinates.z = coor.z;

  pub_pozyx.publish(&pozyx_cordinates);
  nh.spinOnce();


  
}

// error printing function for debugging
void printErrorCode(String operation){
  uint8_t error_code;
  if (remote_id == NULL){
    Pozyx.getErrorCode(&error_code);
   
    return;
  }
  int status = Pozyx.getErrorCode(&error_code, remote_id);
  if(status == POZYX_SUCCESS){
  
  }else{
    Pozyx.getErrorCode(&error_code);
   
  }
}


// function to manually set the anchor coordinates
void setAnchorsManual(){
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
    Pozyx.addDevice(anchor, remote_id);
  }
  if (num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
  }
}
