#include <CurieIMU.h>
#include <CurieBLE.h>
#include <MadgwickAHRS.h>
#define LED 13
#define UPDATE_RATE 25 //UPDATE REATE IN HZ
// #define SERIAL_SEND

// GLOBAL VARIABLES
Madgwick filter;
BLEService EulerService("19B1180F-E8F2-537E-4F6C-D104768A1214"); // BLE Euler Service
// BLE Battery Level Characteristic"
BLECharacteristic EulerAngleChar("19B12A19-E8F2-537E-4F6C-D104768A1214",  // standard 16-bit characteristic UUID
                                                     BLERead | BLENotify | BLEWrite, 2);     // remote clients will be able to
uint8_t mData[2] = {0xFF, 0xFF};
                                                     
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
  EulerAngleChar.setValue(mData,2);   // initial 2 bytes value for this characteristic
  
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

    if(loop_cnt == 10) {
      static int16_t index=0;
      index++;
    mData[0]= (uint8_t) ((index & 0xFF00)>>8);
    mData[1]= (uint8_t) (index & 0x00FF);
    EulerAngleChar.setValue(mData,2);
    loop_cnt = 0;
    Serial.print(index);Serial.print("\t");Serial.print(mData[0]);Serial.print("\t");Serial.println(mData[1]);
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
  Serial.println("Characteristic event, written: ");
 // Serial.println(characteristic.value());
}
/*void SubscriptionHandler(BLEDevice central , BLECharacteristic characteristic)){
  Serial.println("Subscribed");
}*/
void SubscriptionHandler(BLEDevice central, BLECharacteristic characteristic){
  Serial.println("subscribed");
   //EulerAngleChar.setValue(mData,2);   // initial 2 bytes value for this characteristic
}

void NotificationHandler(BLEDevice central, BLECharacteristic characteristic){
  //characteristic.setValue(mData,2);
}
 

