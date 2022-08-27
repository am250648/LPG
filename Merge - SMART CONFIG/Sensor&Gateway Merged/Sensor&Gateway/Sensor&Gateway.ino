#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#define SLEEP
#define WIFI
#define MGR

/****************SMART CONFIG***************/
#ifdef MGR

#include "SPIFFS.h"
const char* Path = "/bootcount.txt";
#include "EEPROM.h"
#define LENGTH(x) (strlen(x) + 1)   // length of char string
#define EEPROM_SIZE 200             // EEPROM size
#define WiFi_rst 0                  //WiFi credential reset pin (Boot button on ESP32)
String ssid = "iCity Wi-Fi";                        //string variable to store ssid
String pss = "iEgyptCity";                         //string variable to store password
unsigned long rst_millis;

 int reset_count = 0;
#endif

/************ SLEEP *****************/
#ifdef SLEEP
RTC_DATA_ATTR int sleepTime = 900;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define SLEEP_30_MIN 900  /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_5  5  /* Time ESP32 will go to sleep (in seconds) */
bool sleep_flag = false;
#define LED_BUILTIN 2
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int SleepCounter = 0;
#endif
bool  sent_done = false;
int Timeout = 400;
/********************************************/
/*****************wifi***********************/

#ifdef WIFI
#include <WiFi.h>
#include <PubSubClient.h>

// Update these with boot_values suitable for your network.

const char* ssid1 = "ICity Wi-Fi";
const char* password1 = "iEgyptCity";
const char* mqtt_server = "test.mosquitto.org";


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int boot_value = 0;

void setup_wifi() {

  vTaskDelay(10/portTICK_RATE_MS);
  // We start by connecting to a WiFi network
  
  //Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pss.c_str());

 int i = 1;
 int j = 1;
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(10/portTICK_RATE_MS);
    i++;
    if(i == 100)
    {
      i = 0;
      Serial.print(String(j++)+ " ");
    }
    
    
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("/***********************/");
}
/********************MQTT**********************/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected() ) {
    
    //Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "LPG01";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected to MQTT");
      // Once connected, publish an announcement...
      client.publish("LPG/ping", "hello LPG");
      // ... and resubscribe
      client.subscribe("LPG/wifi");
    } else {
     // Serial.print("failed, rc=");
      //Serial.print(client.state());
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(500/portTICK_RATE_MS);
      
    }
  }
}

#endif
/*******************************************************/




/***************Ultrasonic***************************/

#define RXD2 4
#define TXD2 5

char     out_txt[ 16 ] = { 0 };

void ultrasonic_init(unsigned char RX, unsigned char TX, long  baud)
{
  Serial2.begin(baud, SERIAL_8N1, RX, TX);
}
uint16_t ultrasonic_read(uint8_t duration)
{

  static uint32_t trigger_cnt = 0;
  char     out_txt[ 16 ] = { 0 };
  uint16_t distance = 0xffff;
  // if(millis() - trigger_cnt > duration )
  {
    Serial2.write(0xff);

    static uint8_t  recv_buf[ 10 ] = { 0 };
    static uint16_t recv_buf_cnt   = 0;
    uint16_t        len            = Serial2.readBytes( ( uint8_t* )&recv_buf, 4 );
    if ( len == 4 && recv_buf[ 0 ] == 0xff )
    {
      uint8_t sum = recv_buf[ 0 ] + recv_buf[ 1 ] + recv_buf[ 2 ];
      if ( sum == recv_buf[ 3 ] )
      {
        distance      = recv_buf[ 1 ] << 8 | recv_buf[ 2 ];//05FE

        sprintf( out_txt, "%d mm \r\n", distance );

        // Serial.print( out_txt );

      }
    }
    trigger_cnt = millis();

  }
  Serial.println(String(distance));
  return distance;

}

/***************END OF Ultrasonic***************************/


/**************TASKS***************/
TaskHandle_t serverTaskHandle = NULL;
void serverTask(void* arg)
{
    /****************SMART CONFIG***************/
#ifdef MGR
  pinMode(WiFi_rst, INPUT);
  if (!EEPROM.begin(EEPROM_SIZE)) { //Init EEPROM
    Serial.println("failed to init EEPROM");
    vTaskDelay(1000/portTICK_RATE_MS);
  }
  else
  {
    Serial.println("Wifi Credentials");
    ssid = readStringFromFlash(0); // Read SSID stored at address 0
    Serial.print("SSID = ");
    Serial.println(ssid);
    pss = readStringFromFlash(40); // Read Password stored at address 40
    Serial.print("psss = ");
    Serial.println(pss);
    Serial.println("/***********************/");
  }

//Wifi Connection
 
  if (boot_value >= 3) // pairing mode
  {
    Timeout = 1000;
    
    //Init WiFi as Station, start SmartConfig
    WiFi.mode(WIFI_AP_STA);
    WiFi.beginSmartConfig();

    //Wait for SmartConfig packet from mobile
    int i = 0;
   
    Serial.println("Waiting for SmartConfig.");
    while (!WiFi.smartConfigDone()) {
      
      vTaskDelay(50/portTICK_RATE_MS);
      i++;
      if(i == 20)
      {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        Serial.print(".");
        i = 0;
        
      }
      
      
      
    }

    Serial.println("");
    Serial.println("SmartConfig received.");

    //Wait for WiFi to connect to AP
    Serial.println("Waiting for WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(50/portTICK_RATE_MS);
      //Serial.print(".");
    }
    digitalWrite(LED_BUILTIN, LOW);

    Serial.println("WiFi Connected.");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // read the connected WiFi SSID and password
    ssid = WiFi.SSID();
    pss = WiFi.psk();
    Serial.print("SSID:");
    Serial.println(ssid);
    Serial.print("PSS:");
    Serial.println(pss);
    Serial.println("Store SSID & PSS in Flash");
    writeStringToFlash(ssid.c_str(), 0); // storing ssid at address 0
    writeStringToFlash(pss.c_str(), 40); // storing pss at address 40
    //Timeout = 0;
    Serial.println("/***********************/");
  }
  
  setup_wifi();

#endif
  while(1)
  {
   
  }
}

TaskHandle_t mainTaskHandle = NULL;
void mainTask(void* arg)
{

  //setup wifi
 
 // setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  while (1)
  {
    //Serial.println("main task");
    
  

    if (!client.connected() )
    {
      reconnect();
    }
    client.loop();
    String msg = String(ultrasonic_read(100));
    //String msg = String(random(0, 600));
    //Serial.println(msg);
   
    client.publish("LPG/wifi", msg.c_str());
    Serial.println("waiting");
    Serial.println("Publishing Data to the Server");
    Serial.println("/***********************/");
    //client.publish("LPG/wifi", String(random(0, 600) ).c_str());
    vTaskDelay(300/portTICK_RATE_MS);
    sent_done = true;

    vTaskDelay(100/portTICK_RATE_MS);
  }
}



TaskHandle_t sleepTaskHandle = NULL;
void sleepTask(void* arg)
{
  int Ticks = 0;
  while (1) {

   // Serial.println("Sleep will be after: " + String(Ticks) );
    if (Ticks >= Timeout || sent_done)
    {
      Ticks = 0;
      //sleep for low period
      if (!sent_done && SleepCounter < 5)
      {
        SleepCounter++;
        Serial.println(String(SleepCounter));
        //sleep for 15 seconds
        esp_sleep_enable_timer_wakeup(SLEEP_5 * uS_TO_S_FACTOR);

        Serial.println("/***********************/");
        Serial.println("Setting Device to sleep for " + String(SLEEP_5) +
                       " Seconds");
        
        Serial.println("Going to sleep now");
        Serial.println("/***********************/");
        Serial.flush();
        esp_deep_sleep_start();
        Serial.println("This will never be printed");

      }

      //Sleep for high period

      sent_done  = false;

      //sleep for 30 min
      esp_sleep_enable_timer_wakeup(sleepTime * uS_TO_S_FACTOR);
      Serial.println("/***********************/");
      Serial.println("Setting Device to sleep for " + String(sleepTime) +
                     " Seconds.. Number of trials : " + String(SleepCounter));
      
      SleepCounter = 0;
      Serial.println("Going to sleep now");
      Serial.println("/***********************/");
      Serial.flush();
      esp_deep_sleep_start();
      Serial.println("This will never be printed");

    }

    Ticks++;
    vTaskDelay(50 / portTICK_RATE_MS);


  }
}
/***********************************/
#ifdef SLEEP

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : //Serial.println("Wakeup caused by external signal using RTC_IO"); 
    break;
    case ESP_SLEEP_WAKEUP_EXT1 : //Serial.println("Wakeup caused by external signal using RTC_CNTL"); 
    break;
    case ESP_SLEEP_WAKEUP_TIMER : //Serial.println("Wakeup caused by timer"); 
    break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : //Serial.println("Wakeup caused by touchpad"); 
    break;
    case ESP_SLEEP_WAKEUP_ULP : //Serial.println("Wakeup caused by ULP program"); break;
    default : //Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); 
    break;
  }
}
#endif

/*****************************/





void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brown out
  Serial.begin(115200);
  Serial.println("/***********************/");
  Serial.println("Starting System");
  Serial.println("/***********************/");

  /********************************/
 /****************Reset Count***************/
 //current time
  int current_Time = millis();
  //init spiffs
  initSPIFFS();

  //read current boot count boot_value
  String str = readFile(SPIFFS, Path);
  boot_value = str.toInt();
  boot_value++;
  char string[8];
  itoa( boot_value, string, 10 );
  //Write new boot count+1
  //if(!bootCount)

    Serial.println("Writing to flash");
    Serial.println("/***********************/");
    writeFile(SPIFFS, Path, string);
    
  
  
  //current - previous > Reset_Timeout
  while(millis() - current_Time < 1000)
  ;
  
  //count = 0
  delay(500);
  
    Serial.println("Writing Zero to flash");
    Serial.println("/***********************/");
    writeFile(SPIFFS, Path, 0);
    
  Serial.println("Boot Counter :" + String(boot_value));
  Serial.println("/***********************/");
  
  /********************************/


  


  /************ SLEEP *****************/
#ifdef SLEEP
  ++bootCount;
  Serial.println("Sleep Boot Counter: " + String(bootCount));
  Serial.println("/***********************/");
  print_wakeup_reason();



#endif

  /*****************************/
  ultrasonic_init(RXD2, TXD2, 9600);



  pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH);

  
    xTaskCreate(sleepTask,"sleepTask",4096,NULL,8,&sleepTaskHandle);
    xTaskCreate(serverTask,"serverTask",4096,NULL,10,&serverTaskHandle);
    xTaskCreate(mainTask, "mainTask", 4096, NULL, 9, &mainTaskHandle);
    

}
  
  


void loop(){ 
}

#ifdef MGR
void writeStringToFlash(const char* toStore, int startAddr) {
  int i = 0;
  for (; i < LENGTH(toStore); i++) {
    EEPROM.write(startAddr + i, toStore[i]);
  }
  EEPROM.write(startAddr + i, '\0');
  EEPROM.commit();
}


String readStringFromFlash(int startAddr) {
  char in[128]; // char array of size 128 for reading the stored data 
  int i = 0;
  for (; i < 128; i++) {
    in[i] = EEPROM.read(startAddr + i);
  }
  return String(in);
}

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

#endif
