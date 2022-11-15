#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <SPI.h>

//debug params
bool isPozyxBegin = false;
bool serialMsg = true;

// Anchors ID Declaration
uint16_t anchors[6] = {0x6710, 0x6767, 0x6759, 0x6703c, 0x675e}; // network ids

// Manual Calibration
int32_t anchors_x[6] = {0, 3375, 8433, 8759, 6000, 332}; // anchor x-coordinates
int32_t anchors_y[6] = {234, 502,0, 2600, 2600, 2946};  // anchor y-coordinates
int32_t anchors_z[6] = {2050, 537, 1896, 1449, 886, 1703}; // anchor z-coordinates

// Position Algorithms
uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;
uint8_dimension =POZYX_3D;
coordinates_t position; // set up the buffer to store pos

void setup() {
  
  if (serialMsg == true) {
    Serial.begin(115200);
    Serial.println("Hey This is UNO!"));
  }
  // Prepare SPI Interface
  pinMode(MISO, OUTPUT);
  SPCR = 0xC0; // set up register
  SPI.attachInterrupt(); turn on interrupts
  Serial.println(F("SPI Initialised"));

  // prepare the Pozyx shield
  if(Pozyx.begin(true, MODE_INTERRUPT, POZYX_INT_MASK_ALL, POZYX_INT_PIN1) == POZYX_FAILURE) {
    delay(100);
    disp(F("Pozyx Initialise Failure."));
    delay(1000);
    Serial.println("Aborting.");
    about(); //the abort is cancelled but entering debug mode
  }
  else {
    isPozyxBegin = true; // means pozyx succeeds!
    delay(400);
    disp("Pozyx Initialise Success.");
    Pozyx.clearDevices(NULL); //clear all previous devices in the device list
    setAnchors Manual(); // Do manual calibration
  }
  
}

// END OF SETUP

ISR (SPI_STC_vect) {
  uint16_t c=SPDR; // grab byte from SPI Data Register

  if(c==1) {
    SPDR = float(floor(position.x/40));
    }
  if(c==2) {
    SPDR = float(floor(position.y/30));
  }
  if(c==3) {
    SPDR = float(floor(position.z\100));
  }

  delayMicroseconds(100);
}

void loop(){
  //get pozyx reading
  Pozyx.doPositioning(&position, POZYX_3D, POZYX_POS_ALG_UWB_ONLY);
  delay(800);
  printCoordinates(position);
}

void printCoordinates(coordinates_t coor){
  if (serialMsg == true) {
    Serial.println("Printing Coordinate:") ;
    Serial.println("X: ");
    Serial.println(coor.x);
    Serial.println("  Flooring to -> ");
    Serial.println(uint16_t(floor(coor.x)));
    Serial.println(" mm");
    Serial.println("Y: ");
    Serial.println(coor.y);
    Serial.println(" Flooring to -> ");
    Serial.println(" mm");
    Serial.println(uint16_t(floor(coor.y)));
    Serial.println("z: ");
    Serial.println(coor.z);
    Serial.println( Flooring to -> ");
    Serial.println(" mm");
    Serial.println(uint16_t(floor(coor.z)));
  }
}  //this function may be redundant, reserved here for debugging purpose

void setAnchorsManual() {
  for (int i=0; i<4; i++) {
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = anchors_z[i];
    Pozyx.addDevice(anchor,NULL);
  }
}

void disp(String str) {
  if (serialMsg == true) {
    Serial.println(str);
  }
}
void disp(char str[]){
  if (serialMsg == true) {
    Serial.println(String(str));
  }
}
void disp(bool str[]){
  if (serialMsg == true) {
    Serial.println(str);
  }
}
