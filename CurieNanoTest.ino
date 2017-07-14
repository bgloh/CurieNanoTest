#include <CurieIMU.h>
#include <CurieBLE.h>
#include <MadgwickAHRS.h>
#define LED 13
#define UPDATE_RATE 25 //UPDATE REATE IN HZ
#define BYTES_TO_SEND 6 // NUMBER OF BYTES TO SEND
// #define SERIAL_SEND

// GLOBAL VARIABLES
Madgwick filter;
BLEService EulerService("19B1180F-E8F2-537E-4F6C-D104768A1214"); // BLE Euler Service
// BLE Battery Level Characteristic"
BLECharacteristic EulerAngleChar("19B12A19-E8F2-537E-4F6C-D104768A1214",  // standard 16-bit characteristic UUID
                                                     BLERead | BLENotify | BLEWrite, BYTES_TO_SEND);     // remote clients will be able to
uint8_t mData[BYTES_TO_SEND] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
                                                     
// get notifications if this characteristic changes
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
volatile unsigned long int loopCounter; 

void setup() {
  Serial.begin(9600);
  pinMode(LED,OUTPUT);  

  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(UPDATE_RATE * 10);
  CurieIMU.setAccelerometerRate(UPDATE_RATE * 10);
 
  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

 // Start Magwick filter
  filter.begin(UPDATE_RATE);

  // BLE Setup
  { 
  BLE.begin();
  BLE.setLocalName("EulerMonitor");
  
  BLE.setAdvertisedService(EulerService);  // add the service UUID

  // add service and characteristics
  EulerService.addCharacteristic(EulerAngleChar);
  BLE.addService(EulerService);   

  // assign event handler for connected and disconnected peripherals
  BLE.setEventHandler(BLEConnected, OnConnectionHandler);
  BLE.setEventHandler(BLEDisconnected, DisconnectedHandler);
 // BLE.setEventHandler(BLEValueUpdated, NotificatioHandler);
// BLE.setEventHandler(BLESubscribed, SubscriptionHandler);

  // assign event handler for characteristic
  EulerAngleChar.setEventHandler(BLEWritten, CharacteristWrittenHandler);
  EulerAngleChar.setEventHandler(BLESubscribed, SubscriptionHandler);
  EulerAngleChar.setEventHandler(BLEValueUpdated, NotificationHandler);

    
  // set initial value for the characteristic
  EulerAngleChar.setValue(mData,BYTES_TO_SEND);   // initial 2 bytes value for this characteristic
  
  // start advertising
  BLE.advertise(); 
  }
  // End of BLE setup 
  Serial.println("Bluetooth device active, waiting for connections...");
   
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / UPDATE_RATE;
  microsPrevious = micros();
}

void loop() {
  int aix, aiy, aiz,gix, giy, giz;  // acceleration and gyroscope in int16 
  float ax, ay, az,gx, gy, gz;    // acceleration and gyroscope in float
  float roll, pitch, heading;    // Euler angles
  unsigned long microsNow;

 //
  BLE.poll();
 
  // check if it's time to read data and update the filter
  microsNow = micros();
  
  if (microsNow - microsPrevious >= microsPerReading) {
    blinkLED(20); // blink
    static unsigned int loop_cnt =0;
    loop_cnt++;
    
   // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // update the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    if(loop_cnt == 5) {
 
    uint8_t RollPitchYaw[6];  // Roll_high_byte Roll_low_byte Pitch_high_byte Pitch_low_byte ...
       
    // preparing data to send
    RollPitchYawByteStream(roll,pitch,heading, RollPitchYaw);

    // update characteristics
    EulerAngleChar.setValue( RollPitchYaw, BYTES_TO_SEND);
    
    loop_cnt = 0;
    Serial.print(roll);Serial.print("\t");Serial.print(pitch);Serial.print("\t");Serial.println(heading);
    }
    
    
    // Send Euler angle to Serial port
    #ifdef SERIAL_SEND
    Serial.print("Orientation: ");Serial.print(heading);
    Serial.print(" ");Serial.print(pitch);
    Serial.print(" ");Serial.println(roll);
    #endif
   
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void blinkLED(  const unsigned char DESIRED_COUNT)
{
  static unsigned char counter =0;
  static char toggle = 0;
  counter++;
  if (counter == DESIRED_COUNT)
  {
    toggle ^= 1;
    digitalWrite(LED,toggle);
    counter = 0;
  }
 }

 
 void OnConnectionHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
 }
void DisconnectedHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void CharacteristWrittenHandler(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  const unsigned char *data = characteristic.value();
  Serial.print("Characteristic event, written: ");
  Serial.println(*data);
}

void SubscriptionHandler(BLEDevice central, BLECharacteristic characteristic){
  Serial.println("subscribed");
   //EulerAngleChar.setValue(mData,2);   // initial 2 bytes value for this characteristic
}

void NotificationHandler(BLEDevice central, BLECharacteristic characteristic){
  Serial.println("notification hadler");
  //EulerAngleChar.setValue(mData,2);
}

uint8_t * convertInt16toUchar8(int16_t input) {
  uint8_t output[2] ; 
  output[0]= (uint8_t) ((input & 0xFF00)>>8); // High Byte
  output[1]= (uint8_t) (input & 0x00FF);      // Low  Byte
  return output;
} 

void convertInt16toUchar8_v2(int16_t input, uint8_t* output) {
  
  output[0]= (uint8_t) ((input & 0xFF00)>>8); // High Byte
  output[1]= (uint8_t) (input & 0x00FF);      // Low  Byte
  
} 

 void RollPitchYawByteStream(float roll, float pitch, float heading, uint8_t* RollPitchYawAngle){

   int16_t roll_int16, pitch_int16, yaw_int16;
   uint8_t RollM[2], PitchM[2], YawM[2];
   //uint8_t RollPitchYawAngle[6];  // RollHighByte RollLowByte PitchHighByte PitchLowByte ...
  
    roll_int16  =   (int16_t) ( roll * 10);
    pitch_int16 =   (int16_t) ( pitch * 10);
    yaw_int16   =   (int16_t) ( heading * 10);
    
    convertInt16toUchar8_v2(roll_int16, RollM);
    convertInt16toUchar8_v2(pitch_int16, PitchM);
    convertInt16toUchar8_v2(yaw_int16, YawM);
    
    RollPitchYawAngle[0] =  RollM[0];  //HB
    RollPitchYawAngle[1] =  RollM[1]; //LB
    RollPitchYawAngle[2] =  PitchM[0];
    RollPitchYawAngle[3] =  PitchM[1];
    RollPitchYawAngle[4] =  YawM[0];
    RollPitchYawAngle[5] =  YawM[1];
}


