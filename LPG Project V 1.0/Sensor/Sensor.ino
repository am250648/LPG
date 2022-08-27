#define SLEEP
/************ SLEEP *****************/
#ifdef SLEEP
RTC_DATA_ATTR int sleepTime = 6;
 
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define SLEEP_30_MIN 900  /* Time ESP32 will go to sleep (in seconds) */
#define SLEEP_15  5  /* Time ESP32 will go to sleep (in seconds) */
bool sleep_flag = false;
#define LED_BUILTIN 2
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int SleepCounter = 0;
#endif
bool  ble_done = false;


#include <NimBLEDevice.h>

void scanEndedCB(NimBLEScanResults results);

static NimBLEAdvertisedDevice* advDevice;

static bool connected = false;
static bool doScan = false;
static bool doConnect = false;
static uint32_t scanTime = 0; /** 0 = scan forever */


/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
      connected = true;
        Serial.println("Connected");
        /** After connection we should change the parameters if we don't need fast response times.
         *  These settings are 150ms interval, 0 latency, 450ms timout.
         *  Timeout should be a multiple of the interval, minimum is 100ms.
         *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
         *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
         */
        //pClient->updateConnParams(120,120,0,60);
        
    };

    void onDisconnect(NimBLEClient* pClient) {
      connected = false;
        Serial.print(pClient->getPeerAddress().toString().c_str());
        Serial.println(" Disconnected - Starting scan");
        NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
    };

    /** Called when the peripheral requests a change to the connection parameters.
     *  Return true to accept and apply them or false to reject and keep
     *  the currently used parameters. Default will return true.
     */
    bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) {
        if(params->itvl_min < 24) { /** 1.25ms units */
            return false;
        } else if(params->itvl_max > 40) { /** 1.25ms units */
            return false;
        } else if(params->latency > 2) { /** Number of intervals allowed to skip */
            return false;
        } else if(params->supervision_timeout > 100) { /** 10ms units */
            return false;
        }

        return true;
    };

    /********************* Security handled here **********************
    ****** Note: these are the same return values as defaults ********/
    uint32_t onPassKeyRequest(){
        Serial.println("Client Passkey Request");
        /** return the passkey to send to the server */
        return 123456;
    };

    bool onConfirmPIN(uint32_t pass_key){
        Serial.print("The passkey YES/NO number: ");
        Serial.println(pass_key);
    /** Return false if passkeys don't match. */
        return true;
    };

    /** Pairing process complete, we can check the results in ble_gap_conn_desc */
    void onAuthenticationComplete(ble_gap_conn_desc* desc){
        if(!desc->sec_state.encrypted) {
            Serial.println("Encrypt connection failed - disconnecting");
            /** Find the client with the connection handle provided in desc */
            NimBLEDevice::getClientByID(desc->conn_handle)->disconnect();
            return;
        }
    };
};


/** Define a class to handle the callbacks when advertisments are received */
class AdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {

    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        Serial.print("Advertised Device found: ");
        Serial.println(advertisedDevice->toString().c_str());
        if(advertisedDevice->isAdvertisingService(NimBLEUUID("DEAD")))
        {
            Serial.println("Found Our Service");
            /** stop scan before connecting */
            NimBLEDevice::getScan()->stop();
            /** Save the device reference in a global for the client to use*/
            advDevice = advertisedDevice;
            /** Ready to connect now */
            doConnect = true;
            doScan = true;
        }
    };
};


/** Notification / Indication receiving handler callback */
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify){
    std::string str = (isNotify == true) ? "Notification" : "Indication";
    str += " from ";
    /** NimBLEAddress and NimBLEUUID have std::string operators */
    str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
    str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
    str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
    str += ", Value = " + std::string((char*)pData, length);
    Serial.println(str.c_str());
}

/** Callback to process the results of the last scan or restart it */
void scanEndedCB(NimBLEScanResults results){
    Serial.println("Scan Ended");
}


/** Create a single global instance of the callback class to be used by all clients */
static ClientCallbacks clientCB;


/** Now we can read/write/subscribe the charateristics of the services we are interested in */
    NimBLERemoteService* pSvc = nullptr;
    NimBLERemoteCharacteristic* pChr = nullptr;
    NimBLERemoteDescriptor* pDsc = nullptr;
/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToServer() {
    NimBLEClient* pClient = nullptr;

    /** Check if we have a client we should reuse first **/
    if(NimBLEDevice::getClientListSize()) {
        /** Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if(pClient){
            if(!pClient->connect(advDevice, false)) {
                Serial.println("Reconnect failed");
                return false;
            }
            Serial.println("Reconnected client");
        }
        /** We don't already have a client that knows this device,
         *  we will check for a client that is disconnected that we can use.
         */
        else {
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if(!pClient) {
        if(NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS) {
            Serial.println("Max clients reached - no more connections available");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        Serial.println("New client created");

        pClient->setClientCallbacks(&clientCB, false);
        /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
         */
        pClient->setConnectionParams(12,12,0,51);
        /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
        pClient->setConnectTimeout(5);


        if (!pClient->connect(advDevice)) {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(pClient);
            Serial.println("Failed to connect, deleted client");
            return false;
        }
    }

    if(!pClient->isConnected()) {
        if (!pClient->connect(advDevice)) {
            Serial.println("Failed to connect");
            return false;
        }
    }

    Serial.print("Connected to: ");
    Serial.println(pClient->getPeerAddress().toString().c_str());
    Serial.print("RSSI: ");
    Serial.println(pClient->getRssi());

    

  

    pSvc = pClient->getService("DEAD");
    if(pSvc) {     /** make sure it's not null */
        pChr = pSvc->getCharacteristic("F00D");

        if(pChr) {     /** make sure it's not null */
            if(pChr->canRead()) {
                Serial.print(pChr->getUUID().toString().c_str());
                Serial.print(" Value: ");
                Serial.println(pChr->readValue().c_str());
            }

            pDsc = pChr->getDescriptor(NimBLEUUID("C01D"));
            if(pDsc) {   /** make sure it's not null */
                Serial.print("Descriptor: ");
                Serial.print(pDsc->getUUID().toString().c_str());
                Serial.print(" Value: ");
                Serial.println(pDsc->readValue().c_str());
            }

           

                if(pChr->canRead()) {
                    Serial.print("The value of: ");
                    Serial.print(pChr->getUUID().toString().c_str());
                    Serial.print(" is now: ");
                    Serial.println(pChr->readValue().c_str());
                }
            }

            /** registerForNotify() has been deprecated and replaced with subscribe() / unsubscribe().
             *  Subscribe parameter defaults are: notifications=true, notifyCallback=nullptr, response=false.
             *  Unsubscribe parameter defaults are: response=false.
             */
            if(0)//pChr->canNotify()) 
            {
                if(!pChr->registerForNotify(notifyCB)) {
                if(!pChr->subscribe(true, notifyCB)) {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
            
                }

               


            
            }
        

     else {
        Serial.println("BAAD service not found.");
    }

    Serial.println("Done with this device!");
    return true;
}


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
      return distance;
  
}

/***************END OF Ultrasonic***************************/


/**************TASKS***************/
TaskHandle_t mainTaskHandle = NULL;
void mainTask(void* arg)
{
  while(1)
  {
    //Serial.println("main task");
    
    // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) 
  {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
    
  }
  
  
  if (connected && !ble_done) 
  {
    //String newValue = String(ultrasonic_read(100));
    String newValue = String(bootCount);

    Serial.println("Setting new characteristic value to \"" + newValue + "\"");

    // Set the characteristic's value to be the array of bytes that is actually a string.
           pChr->writeValue(newValue);
    pChr = pSvc->getCharacteristic("FeeD");
            if(pChr) {     /** make sure it's not null */
            if(pChr->canRead()) {
                Serial.print(pChr->getUUID().toString().c_str());
                Serial.print(" Value: ");
                
                sleepTime = atoi( pChr->readValue().c_str());
                Serial.println(String(sleepTime));

                //save to EEPROM
                
            }
            }
          
    ble_done = true;
    }
    else if(doScan)
    {
      NimBLEDevice::getScan()->start(scanTime,scanEndedCB);
      
     }
     vTaskDelay(100/portTICK_RATE_MS);
   
  }
  
}



TaskHandle_t sleepTaskHandle = NULL;
void sleepTask(void* arg)
{
  int Ticks = 0;
  while(1){

    Serial.println("Sleep will be after: " + String(Ticks) );
    if(Ticks >= 40 || ble_done)
    {
      Ticks = 0;
      //sleep for low period
      if(!ble_done && SleepCounter < 5)
      {
        SleepCounter++;
        Serial.println(String(SleepCounter));
        //sleep for 15 seconds
        esp_sleep_enable_timer_wakeup(SLEEP_15 * uS_TO_S_FACTOR);
        
        Serial.println("/***********************/");
        Serial.println("Setup ESP32 to sleep for " + String(SLEEP_15) +
        " Seconds");
        Serial.println("/***********************/");
        //pClient->disconnect();
        Serial.println("Going to sleep now");
        Serial.flush();
        esp_deep_sleep_start();
        Serial.println("This will never be printed");
        
      }

      //Sleep for high period
      
      ble_done  = false;
      
      //sleep for 30 min
        esp_sleep_enable_timer_wakeup(sleepTime * uS_TO_S_FACTOR);
        Serial.println("/***********************/");
        Serial.println("Setup ESP32 to sleep for every " + String(sleepTime) +
        " Seconds.. sleepcounter : " + String(SleepCounter));
        Serial.println("/***********************/");
        SleepCounter = 0;
      Serial.println("Going to sleep now");
      sleep_flag = true;
      if(connected)
      //pClient->disconnect();
      Serial.flush(); 
      esp_deep_sleep_start();
      Serial.println("This will never be printed");

    }
    
    Ticks++;
    vTaskDelay(50/portTICK_RATE_MS);
    
    
    }
}
/***********************************/
#ifdef SLEEP

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}
#endif

/*****************************/


void setup (){
    Serial.begin(115200);
    Serial.println("Starting NimBLE Client");


         /************ SLEEP *****************/
#ifdef SLEEP
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();

  

#endif

/*****************************/
  ultrasonic_init(RXD2,TXD2,9600);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("sensor");

   

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

    /************************BLE Layer *************************/
    /** Initialize NimBLE, no device name spcified as we are not advertising */
    NimBLEDevice::init("");


 #ifdef SLEEP
  xTaskCreate(sleepTask,"sleepTask",4096,NULL,3,&sleepTaskHandle);
  xTaskCreate(mainTask,"mainTask",4096,NULL,3,&mainTaskHandle);
  #endif
    /** Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_KEYBOARD_ONLY    - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

    /** 2 different ways to set security - both calls achieve the same result.
     *  no bonding, no man in the middle protection, secure connections.
     *
     *  These are the default values, only shown here for demonstration.
     */
    //NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

    /** Optional: set the transmit power, default is 3db */
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */

    /** Optional: set any devices you don't want to get advertisments from */
    // NimBLEDevice::addIgnored(NimBLEAddress ("aa:bb:cc:dd:ee:ff"));

    /** create new scan */
    NimBLEScan* pScan = NimBLEDevice::getScan();

    /** create a callback that gets called when advertisers are found */
    pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());

    /** Set scan interval (how often) and window (how long) in milliseconds */
    pScan->setInterval(45);
    pScan->setWindow(15);

    /** Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
    pScan->setActiveScan(true);
    /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
     *  Optional callback for when scanning stops.
     */
    pScan->start(scanTime, scanEndedCB);
    /************************END OF BLE Layer *************************/
}


void loop (){
  
    
}
