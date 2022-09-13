#include "stdio.h"
#include "Protocol.h"
#include "command.h"
#include "FlexiTimer2.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DxlMaster.h>
#include <JsAr.h>
#include <TrackingCamDxl.h>

#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256


TrackingCamDxl trackingCam(51);

const char* ssid = "ESP32";  
const char* password = "01234567";


String ptr = "<!DOCTYPE html> <html>\n";
IPAddress local_ip(192,168,2,1);
IPAddress gateway(192,168,2,1);
IPAddress subnet(255,255,255,0);
WebServer server(80);


DynamixelDevice deviceButton(0x03);
DynamixelDevice deviceRGB(0x15);
DynamixelDevice deviceDIOD(0x09);
DynamixelDevice deviceBuzzer(0x18);

int THE_STATE = 0;
uint64_t gQueuedCmdIndex;
uint8_t valueButton = 0;

unsigned long previousMillis = 0;


EndEffectorParams gEndEffectorParams;

JOGJointParams  gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd          gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd          gPTPCmd;

PTPCmd          ParkingPoint;

PTPCmd          PointStartUp;
PTPCmd          PointStartDown;
PTPCmd          PointEndUp1;
PTPCmd          PointEndDown1;
PTPCmd          PointEndUp2;
PTPCmd          PointEndDown2;


void setup() {

#if defined(__AVR__)
  Serial1.begin(115200);
#elif defined(ESP32)
  Serial1.begin(115200, SERIAL_8N1, 4, 2);  
  JsAr.begin();
#endif

  printf_begin();
  FlexiTimer2::set(100,Serialread); 
  FlexiTimer2::start();
  
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
  strInflater(-1);
  server.on("/", handle_OnConnect);
  server.on("/led1off", handle_button);
  server.begin();
 
  DxlMaster.begin(57600);
  Serial.begin(115200);
  deviceButton.init();
  deviceRGB.init();
  deviceDIOD.init();
  deviceBuzzer.init();

}

void Serialread()
{
  while(Serial1.available()) {
        uint8_t data = Serial1.read();
        if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
            RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
        }
  }
}

int Serial_putc( char c, struct __file * )
{
    Serial.write( c );
    return c;
}

void printf_begin(void)
{
    #if defined(__AVR__)
    fdevopen( &Serial_putc, 0 );
    #endif
}

void InitRAM(void)
{
    gJOGJointParams.velocity[0] = 100;
    gJOGJointParams.velocity[1] = 100;
    gJOGJointParams.velocity[2] = 100;
    gJOGJointParams.velocity[3] = 100;
    gJOGJointParams.acceleration[0] = 80;
    gJOGJointParams.acceleration[1] = 80;
    gJOGJointParams.acceleration[2] = 80;
    gJOGJointParams.acceleration[3] = 80;

    gJOGCoordinateParams.velocity[0] = 100;
    gJOGCoordinateParams.velocity[1] = 100;
    gJOGCoordinateParams.velocity[2] = 100;
    gJOGCoordinateParams.velocity[3] = 100;
    gJOGCoordinateParams.acceleration[0] = 80;
    gJOGCoordinateParams.acceleration[1] = 80;
    gJOGCoordinateParams.acceleration[2] = 80;
    gJOGCoordinateParams.acceleration[3] = 80;

    gJOGCommonParams.velocityRatio = 50;
    gJOGCommonParams.accelerationRatio = 50;
   
    gJOGCmd.cmd = AP_DOWN;
    gJOGCmd.isJoint = JOINT_MODEL;
    
    gPTPCoordinateParams.xyzVelocity = 100;
    gPTPCoordinateParams.rVelocity = 100;
    gPTPCoordinateParams.xyzAcceleration = 80;
    gPTPCoordinateParams.rAcceleration = 80;

    gPTPCommonParams.velocityRatio = 50;
    gPTPCommonParams.accelerationRatio = 50;

    gPTPCmd.ptpMode = MOVL_XYZ;
    gPTPCmd.x = 200;
    gPTPCmd.y = 0;
    gPTPCmd.r = 0;

    ParkingPoint.ptpMode = MOVL_XYZ;
    ParkingPoint.x = 0;
    ParkingPoint.y = -200;
    ParkingPoint.r = 0;
  
    PointStartUp.x = 265;
    PointStartUp.y = -115;
    PointStartUp.z = 0;
    PointStartUp.r = 0;
    PointStartUp.ptpMode = MOVJ_XYZ;

    PointStartDown.x = 265;
    PointStartDown.y = -115; //157
    PointStartDown.z = -85;
    PointStartDown.r = 0;
    PointStartDown.ptpMode = MOVJ_XYZ;

    PointEndUp1.x = 220;
    PointEndUp1.y = -25;
    PointEndUp1.z = 0;
    PointEndUp1.r = 0;
    PointEndUp1.ptpMode = MOVJ_XYZ;

    PointEndDown1.x = 220;
    PointEndDown1.y = -25;
    PointEndDown1.z = -70;
    PointEndDown1.r = 0;
    PointEndDown1.ptpMode = MOVJ_XYZ;

    PointEndUp2.x = 270;
    PointEndUp2.y = -25;
    PointEndUp2.z = 0;
    PointEndUp2.r = 0;
    PointEndUp2.ptpMode = MOVJ_XYZ;

    PointEndDown2.x = 270;
    PointEndDown2.y = -25;
    PointEndDown2.z = -70;
    PointEndDown2.r = 0;
    PointEndDown2.ptpMode = MOVJ_XYZ;
    
    gQueuedCmdIndex = 0;
}
int mydelay = 1500;
bool FlagPoint = false;

void loop() 
{
    InitRAM();
    ProtocolInit(); 
    SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);
    SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);
    SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);
    printf("\r\n======Enter demo application======\r\n"); 
    for( ; ; )
    {
      server.handleClient();
      static uint32_t timer = millis();
      static uint32_t timerButton = millis();
      static uint32_t count = 0;

      if (Serial.available() > 0) {
        int temp = Serial.read();
        Serial.println(temp);

        if (temp == 57) {
          THE_STATE = 0;
          Message tempMessage;
            memset(&tempMessage, 0, sizeof(Message));
            tempMessage.id = 31;
            tempMessage.rw = true;
            tempMessage.isQueued = false;
            tempMessage.paramsLen = 0;
            memcpy(tempMessage.params, 0, tempMessage.paramsLen);

            MessageWrite(&gSerialProtocolHandler, &tempMessage);

            ProtocolProcess();

            delay(15000);
            THE_STATE = 1;
        }
      }

      if (millis() - timerButton > 50) {
        timerButton = millis();

        deviceButton.ping();
        deviceButton.read((uint8_t)27, (uint8_t)1, &valueButton);
//        int tempType = ballsDetect();
        if (valueButton) {
          THE_STATE = 2;
        }


        if (THE_STATE == 1) {
          deviceDIOD.ping();
          deviceDIOD.write(26, 0);

          deviceRGB.ping();
          deviceRGB.write(26, 0);
          deviceRGB.write(27, 0);
          deviceRGB.write(28, 255);

          SetPTPCmd(&ParkingPoint, true, &gQueuedCmdIndex);
          ProtocolProcess();
        }

        if (THE_STATE == 2) {
          int tempType = ballsDetect();

          if (tempType == 0){
            THE_STATE = 3;
          }
            

            
          if (tempType == 255) {

          deviceBuzzer.write(28, 240);
          deviceBuzzer.write(26, (uint16_t) 512);
          delay(1000);
          deviceBuzzer.write(28, 0);
          deviceBuzzer.write(26, (uint16_t) 0);
//          tempType = -1;
          THE_STATE = 1;
          }
        }

        if (THE_STATE == 3) {
          
          deviceDIOD.ping();
          deviceDIOD.write(26, 0);

          deviceRGB.ping();
          deviceRGB.write(26, 255);
          deviceRGB.write(27, 0);
          deviceRGB.write(28, 0);

          if (!FlagPoint) {
            SetPTPCmd(&PointStartUp, true, &gQueuedCmdIndex);             
            SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);                                       
            delay(mydelay);
            ProtocolProcess();
  
            SetPTPCmd(&PointStartDown, true, &gQueuedCmdIndex);     
            SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);                
            delay(mydelay);        
            ProtocolProcess();
  
            SetPTPCmd(&PointStartUp, true, &gQueuedCmdIndex); 
            SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
            delay(mydelay);
            deviceDIOD.ping();
            deviceDIOD.write(26, 255);                            
            ProtocolProcess();
  
            SetPTPCmd(&PointEndUp1, true, &gQueuedCmdIndex); 
            SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);         
            delay(mydelay);
            ProtocolProcess();
        
            SetPTPCmd(&PointEndDown1, true, &gQueuedCmdIndex);

            SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
            delay(mydelay);
            ProtocolProcess();           
            
            SetPTPCmd(&PointEndUp1, true, &gQueuedCmdIndex);
            SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
            delay(mydelay); 
            deviceDIOD.ping();
            deviceDIOD.write(26, 0);            
            ProtocolProcess();
  
            delay(3000);
            THE_STATE = 1;
            FlagPoint = true;
          } else {
            SetPTPCmd(&PointStartUp, true, &gQueuedCmdIndex);             
            SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);                                       
            delay(mydelay);
            ProtocolProcess();
  
            SetPTPCmd(&PointStartDown, true, &gQueuedCmdIndex);     
            SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);                
            delay(mydelay);          
            ProtocolProcess();
  
            SetPTPCmd(&PointStartUp, true, &gQueuedCmdIndex); 
            SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
            delay(mydelay);
            deviceDIOD.ping();
            deviceDIOD.write(26, 255);                            
            ProtocolProcess();
  
            SetPTPCmd(&PointEndUp2, true, &gQueuedCmdIndex); 
            SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);         
            delay(mydelay);
            ProtocolProcess();
        
            SetPTPCmd(&PointEndDown2, true, &gQueuedCmdIndex);
            SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);

            delay(mydelay);
            ProtocolProcess();           
            
            SetPTPCmd(&PointEndUp2, true, &gQueuedCmdIndex);
            SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
            delay(mydelay);             
            ProtocolProcess();
            deviceDIOD.ping();
            deviceDIOD.write(26, 0);
            delay(3000);
            THE_STATE = 1;
            FlagPoint = false;
          }
        }        
      }       
    }
} 

String strInflater(int num) {
  ptr = "<!DOCTYPE html> <html>\n";
  
  ptr +="<meta http-equiv=\"Content-type\" content=\"text/html; charset=utf-8\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>Управление светодиодом</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>ESP32 Веб-интерфейс</h1>\n";
  ptr +="<h3>Режим точка доступа WiFi (AP)</h3>\n";
  if (num == 0) {
    ptr +="<p>Цвет шарика: оранжевый.</p><a class=\"button button-off\" href=\"/led1off\">Check</a>\n";
  }
  if (num == -1) {
    ptr +="<p>Цвет шарика: неизвестен.</p><a class=\"button button-off\" href=\"/led1off\">Check</a>\n";
  }
  if (num == 255) {
    ptr +="<p>Цвет шарика: красный.</p><a class=\"button button-off\" href=\"/led1off\">Check</a>\n";
  }

  
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
  
  
}

void handle_button() 
{
  int temp = ballsDetect();
  server.send(200, "text/html", strInflater(temp));
  THE_STATE = 2; 
}

int ballsDetect() {
  delay(500);
  uint8_t n = trackingCam.readBlobs(5);
  Serial.print("ID шарика: ");
  Serial.println(trackingCam.blob[0].type, DEC);
  return trackingCam.blob[0].type;
}

void handle_OnConnect() {
  server.send(200, "text/html", ptr); 
}
