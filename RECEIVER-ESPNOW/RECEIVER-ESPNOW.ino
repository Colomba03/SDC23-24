#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define CHANNEL 1

//PINES PARA EL SERVO
Servo myservo;
int xAngle = 0;
#define SERVO_X_PIN  33 // ESP32 pin GPIO33 connected to Servo motor 1


//PINES PARA DC MOTOR
int motor1Pin1 = 0; 
int motor1Pin2 = 0; 
// int motor2Pin1 = 0; 
// int motor2Pin2 = 0; 
int enable1Pin = 0; 
// int enable2Pin = 0;
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

// Structure example to receive data
// Must match the sender structure

//NAT: ESTRUCTURA PARA RECIBIR LOS DATOS. TIENE QUE COINCIDIR CON
//LA ESTRUCTURA DEL TRANSMITTER. HASTA AHORA NO HAY LIMITE DE ATRIBUTOS.

typedef struct struct_message {
  uint8_t x;
  uint8_t y;
} struct_message;

// Create a struct_message called myData
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

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

  //SERVO
  Serial.begin(115200);
  myservo.attach(SERVO_X_PIN);  // attaches the servo on pin X to the servo object
  
  // DC Motors
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  // pinMode(motor2Pin1, OUTPUT);
  // pinMode(motor2Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  // pinMode(enable2Pin, OUTPUT);

  // // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);
  // ledcAttachPin(enable2Pin, pwmChannel);
}

//NAT: PARA MOVER EL DC MOTOR ADELANTE Y ATRAS. POR AHORA SOLO UN MOTOR.
void dc_motors(int x){
  if(x < 125){
    Serial.println("Moving Forward");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    // digitalWrite(motor2Pin1, LOW);
    // digitalWrite(motor2Pin2, HIGH);
  }

  //NAT: EL RANGO DE VALORES (QUE ME SALIO) CUANDO EL JOYSTICK ESTABA SIN MOVERSE
  if(x > 125 && x < 135){
    Serial.println("Motor stopped");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    // digitalWrite(motor2Pin1, LOW);
    // digitalWrite(motor2Pin2, LOW);
  }

  if(x > 135){
    Serial.println("Moving Backwards");
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    // digitalWrite(motor2Pin1, HIGH);
    // digitalWrite(motor2Pin2, LOW);
  }
}

//NAT: PARA MOVER EL SERVO (CON UN SERVO 360 CREO QUE SOLO PUEDE CAMBIAR DIRECCION)
void servo_movement(int x){
  xAngle = map(x, 0, 255, 0, 360);
  Serial.print("xAngle:");
  Serial.println(xAngle);
  myservo.write(map(xAngle, 0, 255, 0, 360));
}

// callback when data is recv from Master
// List<int> signalList{};
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  memcpy(&myData, data, sizeof(myData));
  Serial.print("X: ");
  Serial.println(myData.x);
  Serial.print("Y: ");
  Serial.println(myData.y);

  //NAT: LLAMANDO LOS MOVIMIENTOS DEL SERVO Y DEL DC MOTOR
  servo_movement(myData.x);
  dc_motors(myData.x);
}

//NAT: No se necesita usar el loop como OnDataRecv ya se repite
void loop() {

}