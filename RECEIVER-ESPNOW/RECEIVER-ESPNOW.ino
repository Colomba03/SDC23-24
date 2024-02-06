#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <L298N.h>

#define CHANNEL 1

//PINES PARA EL SERVO
Servo myservo;
int xAngle = 0;
int step = 1000;
#define SERVO_X_PIN  33 // ESP32 pin GPIO33 connected to Servo motor 1


L298N motor1(14,27,26);
L298N motor2(13,19,18);
//PINES PARA DC MOTOR
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int motor2Pin1 = 18; 
int motor2Pin2 = 19; 
int enable1Pin = 13; 
int enable2Pin = 14;
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
  myservo.setPeriodHertz(50);
  myservo.attach(SERVO_X_PIN, 500, 2400);  // attaches the servo on pin X to the servo object
  
  // // DC Motors
  // pinMode(motor1Pin1, OUTPUT);
  // pinMode(motor1Pin2, OUTPUT);
  // pinMode(motor2Pin1, OUTPUT);
  // pinMode(motor2Pin2, OUTPUT);
  // pinMode(enable1Pin, OUTPUT);
  // pinMode(enable2Pin, OUTPUT);

  // // configure LED PWM functionalitites
  // ledcSetup(pwmChannel, freq, resolution);
  
  // // attach the channel to the GPIO to be controlled
  // ledcAttachPin(enable1Pin, pwmChannel);
  // ledcAttachPin(enable2Pin, pwmChannel);
}

//NAT: PARA MOVER EL DC MOTOR ADELANTE Y ATRAS. POR AHORA SOLO UN MOTOR.
void dc_motors(int x){
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  if(x < 100){
    Serial.println("Moving Forward");
    motor1.forward();
    motor2.forward();
  }

  //NAT: EL RANGO DE VALORES (QUE ME SALIO) CUANDO EL JOYSTICK ESTABA SIN MOVERSE
  if(x > 100 && x < 135){
    Serial.println("Motor stopped");
    motor1.stop();
    motor2.stop();
  }

  if(x > 135){
    Serial.println("Moving Backwards");
    motor1.backward();
    motor2.backward();
  }
}

//NAT: PARA MOVER EL SERVO (CON UN SERVO 360 CREO QUE SOLO PUEDE CAMBIAR DIRECCION)
void servo_movement(int x){
  xAngle = map(x, 0, 255, 0, 360);
  Serial.print("xAngle:");
  Serial.println(xAngle);
  if(200 < xAngle && xAngle < 220){
    myservo.write(90);  
    // myservo.writeMicroseconds(1500);
    Serial.println("Stop");
    step = 0;
  }else if(xAngle < 200) {
    Serial.println("One direction");
    myservo.write(xAngle);
    // myservo.writeMicroseconds(2000);
  } else if(xAngle > 220) {
    // myservo.writeMicroseconds(1000);
    Serial.println("the other direction");
    myservo.write(xAngle);
  }
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
  servo_movement(myData.y);
  dc_motors(myData.x);
}

//NAT: No se necesita usar el loop como OnDataRecv ya se repite
void loop() {

}
