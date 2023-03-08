/* Algoritmo que va el el gateway, esta vez con el PMS.*/

/******* -- USO DE LOS PROCESADORES *******/
/******* -- LIBRERIAS -- *******/
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>

/*********************** -- DEFINICIONES -- ************************/
/************************* -- SETTINGS -- **************************/

/******* -- STRUCTURAS DE DATOS EN COMUNICACION ESP NOW -- *********/
// Estructura del mensaje a recibir
typedef struct struct_message {
  byte mac[6];        // MAC del sender
  byte s2write;       // bandera de estados
  int variable;      // que variables vamos a afectar
  int tsample, nsamples;  // tiempoapertuta y numero de muestras
  int pos[2];         // en que posiciones¿afectar?
  float datos[50];    // en si los datos
} struct_message;
// Estructura del mensage a enviar
typedef struct struct_message2 {
  byte request;           // Necesito esto apa!!!
  }struct_message2;

/********************* -- VARIABLES GLOBALES -- ********************/
// Creemos las variables de envio y recepcion de  ESP_NOW,
struct_message myData;
struct_message2 data2send;

// MAC Address de los Sensores - edit as required
uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x96, 0x7F, 0xAC};

// variables para extraer en pruebas
uint8_t mys2write, count;
byte mac[6];
float accx[1750], accy[1750], accz[1750];
int datapack = 50;

/************ --REVEICED AND SENT CALLBACKS -- ************/
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.print("Data received: ");
  //Serial.println(len);
  /*Serial.print("La Mac de este equipo es:");
  Serial.print(myData.mac[0], HEX);Serial.print(":");
  Serial.print(myData.mac[1], HEX);Serial.print(":");
  Serial.print(myData.mac[2], HEX);Serial.print(":");
  Serial.print(myData.mac[3], HEX);Serial.print(":");
  Serial.print(myData.mac[4], HEX);Serial.print(":");
  Serial.println(myData.mac[5], HEX);
  Serial.print("La bandera de esta en : ");
  Serial.println(myData.s2write, BIN);
  Serial.print("La variable a afectar es ");
  Serial.println(myData.variable);
  Serial.print("La durción de la toma en ms es : ");
  Serial.println(myData.tsample);
  Serial.print("El numero de muestras tomadas es de ");
  Serial.println(myData.nsamples);
  Serial.print("las posciones a afectar serian: ");
  Serial.print(myData.pos[0]);Serial.print(" hasta la ");
  Serial.println(myData.pos[1]);
  //recepmeas(myData.variable);
  */
  if (myData.variable == 0x0001){
      recepack(accx, myData.pos);
    }else if(myData.variable == 0x0002){
      
      recepack(accy, myData.pos);
    }else if(myData.variable == 0x0004){
      
      recepack(accz, myData.pos);
    }
  //Serial.println();
}

// Peer info
esp_now_peer_info_t peerInfo;

// Callback function called para enviar datos
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/************* -- FUNCIONES PARA LA RECEPCI0N DE PAQUETES -- *************/
// Funcion que recive una matriz entera. 
void recepack(float medida[1750], int ind[2]){
  // Yo creo que con solo esto será suficiente
  for (int x = ind[0]; x < ind[1]; x++){
    medida[x] = myData.datos[x-ind[0]];
  // Serial.print(accx[x]);Serial.print(" ,");
  }
}

void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);


  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

   // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }  

  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Aqui estamos ");
}
 
void loop() {
  String msg;
   if (Serial.available() > 0){
    msg = Serial.readString();
    if (msg == "meas1"){
      for(int i=0; i <1750; i++){
        Serial.print("x:");Serial.println(accx[i]);  
      }
    }
    else if (msg == "meas2"){
      for(int i=0; i <1750; i++){
        Serial.print("y:"); Serial.println(accy[i]);
       }
    }
    else if (msg == "meas3"){
      for(int i=0; i <1750; i++){
        Serial.print("z:");Serial.println(accz[i]);
      }
    }
    else if (msg == "meas4"){
      data2send.request = 0x03;  
      // Send message via ESP-NOW  -- ahora incluyendo el paquete
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data2send, sizeof(data2send)); 
      if (result == ESP_OK) {Serial.print("Sending confirmed ");
        }else {Serial.println("Sending error");
      }
    }
    else if (msg == "meas5"){
      StaticJsonDocument<200> docm;
      docm["duration"]= myData.tsample;
      docm["samples"]= myData.nsamples;
      serializeJson(docm, Serial);
      Serial.println();
    }
    else if (msg == "meas6"){ 
      StaticJsonDocument<200> doc;
      for (int x = 0; x < myData.nsamples; x++){
        doc["X"] = accx[x];
        doc["Y"] = accy[x];
        doc["Z"] = accz[x];
        serializeJson(doc,Serial);
        Serial.println();
        delay(10);
      }
    }
    else {
      Serial.println("No se reconoce la query");  
    }
  }
}
