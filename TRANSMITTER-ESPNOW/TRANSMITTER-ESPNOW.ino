#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()
#include <ezButton.h>


// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

// NAT: PINES PARA EL JOYSTICK
#define VRX_PIN 32
#define VRY_PIN 33

//Button
ezButton mySwitch(18);
#define buttonPin     19
#define buttonPin2    21
uint8_t buttonState;
uint8_t buttonState2;
bool pressed = false;
bool released = false;

//Shooting
uint8_t level = 0;
bool shooting = false;


// Structure example to send data
// Must match the receiver structure

//NAT: ESTRUCTURA PARA MANDAR LOS DATOS. TIENE QUE COINCIDIR CON
//LA ESTRUCTURA DEL RECEIVER. HASTA AHORA NO HAY LIMITE DE ATRIBUTOS.
typedef struct struct_message {
  uint8_t x;
  uint8_t y;
  uint8_t buttonState;
  uint8_t buttonState2;
  uint8_t level;
  bool pressed;
  bool released;
  bool shooting;
} struct_message;

struct_message myData;

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void ScanForSlave() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL); // Scan only on one channel
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with Slave
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if ( exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}


void sendData() {
  //NAT: AQUI SE PONE LA DATA QUE SE MANDA EN LA ESTRUCTURA. 
  //myData.(el atributo que aparece en la estructura)
  myData.x = analogRead(VRX_PIN);
  myData.y = analogRead(VRY_PIN);
  myData.buttonState = buttonState;
  myData.buttonState2 = buttonState2;
  myData.level = level;
  myData.pressed = pressed;
  myData.released = released;
  Serial.println("Level");
  Serial.println(level);
  Serial.println("buttonState");
  Serial.println(buttonState);
  Serial.println("buttonState2");
  Serial.println(buttonState2);
  Serial.println("pressed");
  Serial.println(pressed);
  Serial.println("released");
  Serial.println(released);

  const uint8_t *peer_addr = slave.peer_addr;

  Serial.print("Sending: "); 
  //NAT: Solo hay que llamarlo una vez porque se manda solo una estructura
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &myData, sizeof(myData)); 

  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL "); Serial.println(WiFi.channel());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  mySwitch.setDebounceTime(50);
}

void loop() {
  //NAT: LO PUSE AQUI POR SI ACASO, NO SE SI HACE DIFERENCIA
  Serial.println(level);
  myData.x = analogRead(VRX_PIN);
  myData.y = analogRead(VRY_PIN);
  mySwitch.loop();
  Serial.println("Joy stick values:");
  Serial.println(myData.x);
  Serial.println(myData.y);
  int xAngle = map(myData.y, 0, 255, 0, 360);
  Serial.print("xAngle:");
  Serial.println(xAngle);
  buttonState = digitalRead(buttonPin);
  buttonState2 = digitalRead(buttonPin2);
  myData.buttonState = buttonState;
  myData.buttonState = buttonState2;

  if(buttonState == 1 && !shooting && level<5){ 
    level = level + 1;
    myData.level = level;
  } 
  if(mySwitch.isPressed()){
    Serial.println("Pressed");
    pressed = true;
    released = false;
    myData.pressed = pressed;
    myData.released = released;

  }
  if(mySwitch.isReleased()){
    Serial.println("Released");
    pressed = false;
    released = true;
    myData.released = true;
    myData.pressed = false;
  }
  if(buttonState2 == 1 && level != 0){
    level = level - 1;
    myData.level = level;
  }
  if(mySwitch.isPressed() && level != 0){
    shooting = true; 
    myData.shooting = true;
    level = 0;
    myData.level = level;
  }
  else if(level == 5 || (mySwitch.isReleased())){
    shooting = false;
    myData.shooting = false;
  }




  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in slave variable
  // We will check if slave is defined and then we proceed further
  if (slave.channel == CHANNEL) { // check if slave channel is defined
    // slave is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    if (isPaired) {
      // pair success or already paired
      // Send data to device
      sendData();
    } else {
      // slave pair failed
      Serial.println("Slave pair failed!");
    }
  }
  else {
    // No slave found to process
  }

}
