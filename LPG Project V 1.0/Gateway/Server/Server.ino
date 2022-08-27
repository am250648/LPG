

//LEDS
int red = 14;
int blue = 2;
int green = 12;
int devices_counter = 0;
#define SIM
#define WIFI
/******************WIFI***************/
 #ifdef WIFI
#include <WiFi.h>
#include <PubSubClient.h>
/***********wifi manager ***********************/
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "SPIFFS.h"

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "iCity_R&D";
const char* PARAM_INPUT_2 = "Icity@iot80";
const char* PARAM_INPUT_3 = "192.168.1.250";
const char* PARAM_INPUT_4 = "192.168.1.1";
const char* PARAM_INPUT_5 = "Internet.vodafone.net";


//Variables to save values from HTML form
String ssid_2;
String pass_2;
String ip;
String gateway;
String apn;

// File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* ipPath = "/ip.txt";
const char* gatewayPath = "/gateway.txt";
const char* apnPath = "/apn.txt";
IPAddress localIP;
//IPAddress localIP(192, 168, 1, 200); // hardcoded

// Set your Gateway IP address
IPAddress localGateway;
//IPAddress localGateway(192, 168, 1, 1); //hardcoded
IPAddress subnet(255, 255, 0, 0);

// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)

// Set LED GPIO
const int ledPin = 20;
// Stores LED state

String ledState;

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Read File from SPIFFS
String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return String();
  }
  
  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;     
  }
  return fileContent;
}

// Write file to SPIFFS
void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- frite failed");
  }
}

// Initialize WiFi
bool initWiFi() {
  if(ssid_2=="" || ip==""){
    Serial.println("Undefined SSID or IP address.");
    return false;
  }

  WiFi.mode(WIFI_STA);
  localIP.fromString(ip.c_str());
  localGateway.fromString(gateway.c_str());


  if (!WiFi.config(localIP, localGateway, subnet)){
    Serial.println("STA Failed to configure");
    return false;
  }
  WiFi.begin(ssid_2.c_str(), pass_2.c_str());
  Serial.println("Connecting to WiFi...");

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

 /* while(WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      Serial.println("Failed to connect.");
      return false;
    }
  }


  Serial.println(WiFi.localIP());
  */
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

// Replaces placeholder with LED state value
String processor(const String& var) {
  if(var == "STATE") {
    if(digitalRead(ledPin)) {
      ledState = "ON";
    }
    else {
      ledState = "OFF";
    }
    return ledState;
  }
  return String();
}

void wifi_manager()
{
  initSPIFFS();

  // Set GPIO 2 as an OUTPUT
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  // Load values saved in SPIFFS
  ssid_2 = readFile(SPIFFS, ssidPath);
  pass_2 = readFile(SPIFFS, passPath);
  ip = readFile(SPIFFS, ipPath);
  gateway = readFile (SPIFFS, gatewayPath);
  apn = readFile (SPIFFS, apnPath);
  Serial.println(ssid_2);
  Serial.println(pass_2);
  Serial.println(ip);
  Serial.println(gateway);
  Serial.println(apn);

  if(initWiFi()) {
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });
    server.serveStatic("/", SPIFFS, "/");
    
    // Route to set GPIO state to HIGH
    server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request) {
      digitalWrite(ledPin, HIGH);
      request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });

    // Route to set GPIO state to LOW
    server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request) {
      digitalWrite(ledPin, LOW);
      request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });
    server.begin();
  }
  else {
    // Connect to Wi-Fi network with SSID and password
    Serial.println("Setting AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("ESP-WIFI-MANAGER", NULL);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP); 

    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/wifimanager.html", "text/html");
    });
    
    server.serveStatic("/", SPIFFS, "/");
    
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            ssid_2 = p->value().c_str();
            Serial.print("SSID set to: ");
            Serial.println(ssid_2);
            // Write file to save value
            writeFile(SPIFFS, ssidPath, ssid_2.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            pass_2 = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass_2);
            // Write file to save value
            writeFile(SPIFFS, passPath, pass_2.c_str());
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            ip = p->value().c_str();
            Serial.print("IP Address set to: ");
            Serial.println(ip);
            // Write file to save value
            writeFile(SPIFFS, ipPath, ip.c_str());
          }
          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_4) {
            gateway = p->value().c_str();
            Serial.print("Gateway set to: ");
            Serial.println(gateway);
            // Write file to save value
            writeFile(SPIFFS, gatewayPath, gateway.c_str());
          }

          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_5) {
            apn = p->value().c_str();
            Serial.print("apn set to: ");
            Serial.println(apn);
            // Write file to save value
            writeFile(SPIFFS, apnPath, apn.c_str());
          }
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }
      request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to IP address: " + ip);
      delay(3000);
      ESP.restart();
    });
    server.begin();
  }
}
/*******************************************/
const char* ssid = "iCity_R&D";
const char* password = "Icity@iot80";

const char* mqtt_server = "test.mosquitto.org";
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
#endif

/**************************************/
/***************GSM *************************************/
#ifdef SIM
bool flag = false;
#define RXD2 4
#define TXD2 5
String readvalue;

       

#define TINY_GSM_MODEM_SIM800

#include <TinyGsmClient.h>
#include <PubSubClient.h>


//Network details
const char gsm_apn[]  = "Internet.vodafone.net";
const char user[] = "internet";
const char pass[] = "internet";

// MQTT details
const char* broker = "test.mosquitto.org";
const char* topicOut = "LPG";
const char* topicIn = "LPG";

TinyGsm modem(Serial2);
TinyGsmClient client2(modem);
PubSubClient mqtt(client2);

void myGSM()
{
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  Serial.println("System start.");
  modem.restart();
  Serial.println("Modem: " + modem.getModemInfo());
  Serial.println("Searching for network provider.");
  if(!modem.waitForNetwork())
  {
    Serial.println("fail");
    //while(true);
  }
  Serial.println("Connected to network.");
  Serial.println("Signal Quality: " + String(modem.getSignalQuality()));

  Serial.println("Connecting to GPRS network.");
  if (!modem.gprsConnect(gsm_apn, user, pass))
  {
    Serial.println("fail");
    //while(true);
  }
  Serial.println("Connected to GPRS: " + String(apn));
  
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
  Serial.println("Connecting to MQTT Broker: " + String(broker));
  while(mqttConnect()==false) continue;
  Serial.println();

  mqtt.publish(topicOut, "hello mqtt");
}

#endif
/****************************************************/
#include <NimBLEDevice.h>

static NimBLEServer* pServer;

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */  
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        Serial.println("Client connected");
        Serial.println("Multi-connect support: start advertising");
        NimBLEDevice::startAdvertising();

        devices_counter++;
      if(devices_counter > 0)
      {
        digitalWrite(red, LOW); // turn the LED on
        digitalWrite(blue, HIGH); // turn the LED on
      }
    };
    /** Alternative onConnect() method to extract details of the connection. 
     *  See: src/ble_gap.h for the details of the ble_gap_conn_desc struct.
     */  
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        Serial.print("Client address: ");
        Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
        /** We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments, try for 5x interval time for best results.  
         */
        //pServer->updateConnParams(desc->conn_handle, 24, 48, 0, 60);
    };
    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("Client disconnected - start advertising");
        NimBLEDevice::startAdvertising();


      devices_counter--;
      
      
      if(devices_counter <= 0)
      {
        digitalWrite(blue, LOW); // turn the LED on
        digitalWrite(red, HIGH); // turn the LED on
      }

        
    };
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
    };
    
/********************* Security handled here **********************
****** Note: these are the same return values as defaults ********/
    uint32_t onPassKeyRequest(){
        Serial.println("Server Passkey Request");
        /** This should return a random 6 digit number for security 
         *  or make your own static passkey as done here.
         */
        return 123456; 
    };

    bool onConfirmPIN(uint32_t pass_key){
        Serial.print("The passkey YES/NO number: ");Serial.println(pass_key);
        /** Return false if passkeys don't match. */
        return true; 
    };

    void onAuthenticationComplete(ble_gap_conn_desc* desc){
        /** Check that encryption was successful, if not we disconnect the client */  
        if(!desc->sec_state.encrypted) {
            NimBLEDevice::getServer()->disconnect(desc->conn_handle);
            Serial.println("Encrypt connection failed - disconnecting client");
            return;
        }
        Serial.println("Starting BLE work!");
    };
};

/** Handler class for characteristic actions */
class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic){
        Serial.print(pCharacteristic->getUUID().toString().c_str());
        Serial.print(": onRead(), value: ");
        Serial.println(pCharacteristic->getValue().c_str());
    };

    void onWrite(NimBLECharacteristic* pCharacteristic) {
        Serial.print(pCharacteristic->getUUID().toString().c_str());
        Serial.print(": onWrite(), value: ");
        Serial.println(pCharacteristic->getValue().c_str());
        pCharacteristic->notify(true);

        #ifdef WIFI
        client.publish("LPG", pCharacteristic->getValue().c_str());

        #endif
        #ifdef SIM
        String readvalue = pCharacteristic->getValue().c_str();

        #endif

        
    };
    /** Called before notification or indication is sent, 
     *  the value can be changed here before sending if desired.
     */
    void onNotify(NimBLECharacteristic* pCharacteristic) {
        Serial.println("Sending notification to clients");
    };


    /** The status returned in status is defined in NimBLECharacteristic.h.
     *  The value returned in code is the NimBLE host return code.
     */
    void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code) {
        String str = ("Notification/Indication status code: ");
        str += status;
        str += ", return code: ";
        str += code;
        str += ", "; 
        str += NimBLEUtils::returnCodeToString(code);
        Serial.println(str);
    };

    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
        String str = "Client ID: ";
        str += desc->conn_handle;
        str += " Address: ";
        str += std::string(NimBLEAddress(desc->peer_ota_addr)).c_str();
        if(subValue == 0) {
            str += " Unsubscribed to ";
        }else if(subValue == 1) {
            str += " Subscribed to notfications for ";
        } else if(subValue == 2) {
            str += " Subscribed to indications for ";
        } else if(subValue == 3) {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID()).c_str();

        Serial.println(str);
    };
};
    
/** Handler class for descriptor actions */    
class DescriptorCallbacks : public NimBLEDescriptorCallbacks {
    void onWrite(NimBLEDescriptor* pDescriptor) {
        std::string dscVal((char*)pDescriptor->getValue(), pDescriptor->getLength());
        Serial.print("Descriptor witten value:");
        Serial.println(dscVal.c_str());
    };

    void onRead(NimBLEDescriptor* pDescriptor) {
        Serial.print(pDescriptor->getUUID().toString().c_str());
        Serial.println(" Descriptor read");
    };
};


/** Define callback instances globally to use for multiple Charateristics \ Descriptors */ 
static DescriptorCallbacks dscCallbacks;
static CharacteristicCallbacks chrCallbacks;
static CharacteristicCallbacks chrCallbacks2;

void setup() {

  //LEDS
  // initialize the digital pin as an output.
pinMode(red, OUTPUT);
pinMode(green, OUTPUT);
pinMode(blue, OUTPUT);
digitalWrite(red, HIGH);
digitalWrite(green, LOW);
digitalWrite(blue, LOW);
/**************************/
    Serial.begin(115200);
    Serial.println("Starting NimBLE Server");

 /*********************WIFI*********/
    #ifdef WIFI
    
    //wifi_manager();
    
    setup_wifi();
  
    
   
  #endif
  /*************************************/

    /** sets device name */
    NimBLEDevice::init("LPG");

    /** Optional: set the transmit power, default is 3db */
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
    
    /** Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_DISPLAY_ONLY    - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // use passkey
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

    /** 2 different ways to set security - both calls achieve the same result.
     *  no bonding, no man in the middle protection, secure connections.
     *   
     *  These are the default values, only shown here for demonstration.   
     */ 
    //NimBLEDevice::setSecurityAuth(false, false, true); 
    NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());


  
//Add sensor characteristic
    NimBLEService* pBaadService = pServer->createService("DEAD");
    NimBLECharacteristic* pSensorCharacteristic = pBaadService->createCharacteristic(
                                               "FEED",
                                               NIMBLE_PROPERTY::READ |
                                               NIMBLE_PROPERTY::WRITE |
                                               NIMBLE_PROPERTY::NOTIFY
                                              );

    pSensorCharacteristic->setValue("Please add sensor to get readings");
    pSensorCharacteristic->setCallbacks(&chrCallbacks);

//Add Sleep characteristic
    NimBLECharacteristic* pSleepCharacteristic = pBaadService->createCharacteristic(
                                               "F00D",
                                               NIMBLE_PROPERTY::READ |
                                               NIMBLE_PROPERTY::WRITE |
                                               NIMBLE_PROPERTY::NOTIFY
                                              );

    //pSleepCharacteristic->setValue("Please add sensor to get readings");
    pSleepCharacteristic->setCallbacks(&chrCallbacks2);    

    /** Note a 0x2902 descriptor MUST NOT be created as NimBLE will create one automatically
     *  if notification or indication properties are assigned to a characteristic.
     */

    /** Custom descriptor: Arguments are UUID, Properties, max length in bytes of the value */
    NimBLEDescriptor* pC01Ddsc = pSensorCharacteristic->createDescriptor(
                                               "C01D",
                                               NIMBLE_PROPERTY::READ | 
                                               NIMBLE_PROPERTY::WRITE|
                                               NIMBLE_PROPERTY::WRITE_ENC, // only allow writing if paired / encrypted
                                               100
                                              );
    pC01Ddsc->setValue("Send it back!");
    pC01Ddsc->setCallbacks(&dscCallbacks);

    /** Start the services when finished creating all Characteristics and Descriptors */  
    //pDeadService->start();
    pBaadService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    /** Add the services to the advertisment data **/
    //pAdvertising->addServiceUUID(pDeadService->getUUID());
    pAdvertising->addServiceUUID(pBaadService->getUUID());
    /** If your device is battery powered you may consider setting scan response
     *  to false as it will extend battery life at the expense of less data sent.
     */
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("Advertising Started");

/*****************GSM***********************************/
#ifdef SIM
   myGSM();
#endif
/****************************************************/

   
}


void loop() {

  /********************WIFI*****************/
  #ifdef WIFI
  
if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("LPG/outTopic", msg);
  }
  #endif
  /******************************************/
/****************************************************/   
#ifdef SIM

if(mqtt.connected())
  {
    mqtt.loop();
  }
  else
  {
    Serial.println("mqtt disconected .. trying to connect");
    mqttConnect();
  }
if(flag)
{
  mqtt.publish(topicOut, readvalue.c_str());
  flag = false;
}
#endif

/****************************************************/
  
}


#ifdef WIFI

void setup_wifi() {

  //ssid = ssid_2.c_str();
 // password = pass_2.c_str();

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    //digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    //digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
#endif


#ifdef SIM
boolean mqttConnect()
{
  if(!mqtt.connect("GsmClientTest"))
  {
    Serial.print(".");
    return false;
  }
  Serial.println("Connected to broker.");
  mqtt.subscribe(topicIn);
  return mqtt.connected();
}

void mqttCallback(char* topic, byte* payload, unsigned int len)
{
  Serial.print("Message receive: ");
  Serial.write(payload, len);
  Serial.println();
}

#endif
/****************************************************/
