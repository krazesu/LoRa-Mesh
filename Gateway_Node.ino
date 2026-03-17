/*
  Project: LoRa Mesh Sensor Network for Air Quality and Exposure Data in Jeepneys  
  Author: Urmeneta, Kriz Greg
  Device: Gateway Node
  Hardware:
    -NodeMCU ESP8266
    -LoRa module (RA-02)
    -PM2.5 sensor (PMS7003)
  Dependencies:
    -PMS.h
    -SoftwareSerial.h
    -SPI.h
    -LoRa.h
*/

// ================== LIBRARIES ==================

///Lora Libraries
#include <SPI.h>
#include <LoRa.h>

/// wifi library
#include <ESP8266WiFi.h>
#include "HTTPSRedirect.h"

//ThingSpeak Library
#include "ThingSpeak.h"


// ================== PIN DEFINITIONS ==================

#define ss 15   //NSS - D8
#define rst 16  //RST - D0
#define dio0 4  //DIO0 - D2


// ================== GLOBAL VARIABLES ==================
/*
Node0 = 5814650
Node1 = 3980550
Node2 = 15754626
Node3 = 4024082
Node4 = 14715324
*/
uint8_t nodeUID = 0;  //DEVICE UNIQUE ID

int nodes_id[4] = {
  3980550, 15754626, 4024082, 14715324
};

//const char* ssid = "Smart_WiFi_D5A28";
//const char* password = "Giyongau2297";

const char* ssid = "Kraze";
const char* password = "qwerty123";

WiFiClient clientTS;

unsigned long channelId = 3062290;
const char* writeAPIKey = "C2QNXQLHM9753OEA";

const char* GScriptId = "AKfycbyMeTIn3a3jzUDFl-e0aZiIO3nHnOehtg1EMXWRCgPgIHY34lIxwHYUR2_nySFxkNeY";


// payload format : {"command": "append_row", "values0" : "-1,-1", "values1" : "-1,-1", "values2" : "-1,-1", "values3" : "-1,-1", "NETR1" : "0,0,0,0", "NETR2" : "0,0,0,0", "NETR3" : "0,0,0,0", "NETR4" : "0,0,0,0"}
String payloadBase = "{\"command\": \"append_row\"";
String payload = "";

// Google Sheets setup
const char* host = "script.google.com";
const int httpsPort = 443;
const char* fingerprint = "";
String url = String("/macros/s/") + GScriptId + "/exec";
HTTPSRedirect* client = nullptr;

unsigned long previousMillis = 0, previousMillis2 = 0;  // last time event triggered
const long interval = 30000;                            // Interval (30s)

//Struct for Packets
struct dataPacket {
  int packet_id;       // ID of the packet
  int owner_id;        // ID of the owner/creator of the packet
  int source_id;       //ID of the sender/forwarder node
  int destination_id;  // ID of the destination node of packet
  uint8_t packetType;  //0 = Data, 1 = RREQ, 2 = RREP, 3 = RERR, 4 = Ack
  float sensorValue;   // Sensor reading
};

int rootNodes[10];  //array of nodes directly connected to gateway (root nodes)
uint8_t numRoot = 0;

int nodeID = nodeUID;

dataPacket packet;
dataPacket recordPkt;
dataPacket rrepBuffer[4];
uint8_t rrepBufferSize = 0;

bool send = false;
bool sendDataAck = false;
bool sendRupd = false;

struct data {
  int id = -1;
  float value = 0.0;
};

data dataBuffer[4][70];
uint8_t bufferSize[4] = { 0, 0, 0, 0 };
uint8_t bufferChecker[4] = { 0, 0, 0, 0 };

data recentData[4];

struct nodePerformance{
  unsigned long totalBytes = 0;
  unsigned long packetSent = 0;
  int aveRTT = 0;
  float throughput = 0;
};

nodePerformance networkRecord[4];



// ================== SETUP ==================

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  //print UID for identification,debugging
  Serial.println("--------GATEWAY NODE UID: " + String(nodeUID) + "--------");

  while (!Serial);
  Serial.println("LoRa Gateway");
  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    delay(100);
    while (1)
      ;
  }

  WiFi.mode(WIFI_STA);
  wifiConnect();

  ThingSpeak.begin(clientTS);  // Initialize ThingSpeak

  httpsSetup();  //Initialize for HTTPS connection for logging data on Gsheets

  //LoRa.setSpreadingFactor(7); //Supported values are between 6 and 12
  //LoRa.setSignalBandwidth(signalBandwidth); //Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
  //LoRa.setCodingRate4(codingRateDenominator); //Supported values are between 5 and 8
  //LoRa.setPreambleLength(preambleLength); //Supported values are between 6 and 65535
  //LoRa.setTxPower(txPower, outputPin); Supported values are 2 to 20

  LoRa.onReceive(onReceive);  // register the receive callback

  LoRa.receive();  // put the radio into receive mode
}


// ================== MAIN LOOP ==================

void loop() {
  //send data on interval
  if (millis() - previousMillis >= interval) {

    wifiReconnect();

    if (numRoot > 0) {
      recordData();
    }

    sendToThingSpeak();

    Serial.println("INFO: Number of Nodes Connected: " + String(numRoot));

    previousMillis = millis();
  }

  //send general packets like RREP
  if (send) {

    //creates time slots for root nodes to transmit data to lessen collisions
    if (millis() - previousMillis2 >= 1000) {
      sendPacket(rrepBuffer[0]);
      Serial.println("SEND: RREP sent to Node " + String(rrepBuffer[0].destination_id));
      removeFromRrepBuffer();

      if (rrepBufferSize > 0) {
        send = true;
      } else {
        send = false;
      }

      previousMillis2 = millis();
    }
  }

  //send ack to acknowledge received data packet
  if (sendDataAck) {
    sendAck(0, recordPkt);  //send ack
    sendDataAck = false;
  }

  //send route updates
  if (sendRupd){
    sendRUPD(0);
    sendRupd = false;    
  }
}


// ================== FUNCTIONS ==================

//// sending packet function
void sendPacket(dataPacket sendPacket) {
  // Packet format: /packet ID/Source node/Packet type/Sensor reading
  String dataString = "/" + String(sendPacket.packet_id) + "/" + String(sendPacket.owner_id) + "/" + String(sendPacket.source_id) +
  "/" + String(sendPacket.destination_id) + "/" + String(sendPacket.packetType) + "/" + String(sendPacket.sensorValue);

  //Send data packet
  LoRa.beginPacket();
  LoRa.print(dataString);
  LoRa.endPacket();
  LoRa.receive();
}

//// Execute when receiving packet
void onReceive(int packetSize) {
  // received a packet
  String receivedPacket = "";

  // read packet
  for (int i = 0; i < packetSize; i++) {
    receivedPacket += (char)LoRa.read();
  }

  dataPacket receivedPkt = getPacketInfo(receivedPacket);
  uint8_t ackType;

  switch (receivedPkt.packetType) {
    case 0:  //Data
      if(receivedPkt.destination_id == 0) {
        if (!isInBuffer(receivedPkt.packet_id, receivedPkt.owner_id)) {
          addToBuffer(receivedPkt.owner_id, receivedPkt.packet_id, receivedPkt.sensorValue);
          addToRecentData(receivedPkt.owner_id, receivedPkt.packet_id, receivedPkt.sensorValue);
          recordPkt.packet_id = receivedPkt.packet_id;
          recordPkt.owner_id = receivedPkt.owner_id;
          recordPkt.source_id = receivedPkt.source_id;
          recordPkt.destination_id = 0;
          recordPkt.packetType = 0;
          recordPkt.sensorValue = receivedPkt.sensorValue;

          sendDataAck = true;
        }
      } 
      else{
        redirectNode();
        addRootNode(receivedPkt.source_id);
      }
      break;

    case 1:  //RREQ

      //RREP packet
      packet.packet_id = 0;
      packet.owner_id = nodeID;
      packet.source_id = nodeID;
      packet.destination_id = receivedPkt.source_id;
      packet.packetType = 2;
      packet.sensorValue = 0; //Hop count

      if (!isInRrepBuffer(receivedPkt.owner_id)) {
        Serial.println("RECEIVED: received RREQ from Node " + String(receivedPkt.owner_id));

        addToRrepBuffer(packet);
        send = true;
        if(!isRootNode(receivedPkt.owner_id)){
          addRootNode(receivedPkt.owner_id);
        }
      }

      break;

    case 2:  //Ignore RREP
      break;

    case 3:  //Ignore RERR
      break;

    case 4:  //ACK
      ackType = receivedPkt.sensorValue;

      switch (ackType) {
        case 1:
          //add as root node if not yet
          if (receivedPkt.destination_id == nodeID) {
            if(!isRootNode(receivedPkt.owner_id)){
              addRootNode(receivedPkt.owner_id);
            }
          }
          break;
      }
      break;

    case 5: //Ignore RUPD
      break;

    case 6: //Network Record
      uint8_t index;
      for (int i = 0; i < sizeof(bufferSize); i++) {
        if (nodes_id[i] == receivedPkt.owner_id) {
          index = i;
          break; 
        }
      }

      networkRecord[index].totalBytes = receivedPkt.destination_id;
      networkRecord[index].packetSent = receivedPkt.source_id;
      networkRecord[index].aveRTT = receivedPkt.packet_id;
      networkRecord[index].throughput = receivedPkt.sensorValue;

      break;
  }
    
}

void httpsSetup() {
  client = new HTTPSRedirect(httpsPort);
  client->setInsecure();
  client->setPrintResponseBody(true);
  client->setContentTypeHeader("application/json");

  Serial.print("Connecting to ");
  Serial.println(host);

  // Try to connect for a maximum of 5 times
  bool flag = false;
  for (int i = 0; i < 5; i++) {
    int retval = client->connect(host, httpsPort);
    if (retval == 1) {
      flag = true;
      Serial.println("Connected");
      break;
    } else
      Serial.println("Connection failed. Retrying...");
  }
  if (!flag) {
    Serial.print("Could not connect to server: ");
    Serial.println(host);
    return;
  }
  delete client;     // delete HTTPSRedirect object
  client = nullptr;  // delete HTTPSRedirect object
}

void recordData() {

  static bool flag = false;
  if (!flag) {
    client = new HTTPSRedirect(httpsPort);
    client->setInsecure();
    flag = true;
    client->setPrintResponseBody(true);
    client->setContentTypeHeader("application/json");
  }
  if (client != nullptr) {
    if (!client->connected()) {
      client->connect(host, httpsPort);
    }
  } else {
    Serial.println("Error creating client object!");
  }

  // Create json object string to send to Google Sheets
  payload = getPayload();

  // Publish data to Google Sheets
  Serial.println("NOTIF: Recording data on Google Sheets...");

  LoRa.idle();
  if (client->POST(url, host, payload)) {
    // do stuff here if publish was successful
    clearBufferAfterRecord();
    LoRa.receive();
  } else {
    // do stuff here if publish was not successful
    Serial.println("Error while connecting");
  }
}

//// get the data to be sent as json object string
String getPayload() {
  String payload = "";
  payload = payloadBase;
  for (int i = 0; i < sizeof(bufferSize); i++) {
    payload += ", \"values" + String(i) + "\" : \"";
    for (int x = 0; x < bufferSize[i]; x++) {

      String packetId = String(dataBuffer[i][x].id);
      String value = String(dataBuffer[i][x].value);

      payload = payload + packetId + "," + value;
      payload += ",";
      bufferChecker[i]++;
    }
    if (bufferSize[i] == 0) {
      payload += "-1,-1\"";
    } else {
      payload += "\"";
    }
  }

  for (int i = 0; i < sizeof(bufferSize); i++){
    payload += ", \"netr" + String(i) + "\" : \"";

    payload += String(networkRecord[i].totalBytes) + "," + String(networkRecord[i].packetSent) + "," + String(networkRecord[i].aveRTT) + "," + String(networkRecord[i].throughput) + "\"";
  }

  return payload += "}";
}

////connect to WiFi
void wifiConnect() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

//// reconnect to WiFi if disconnected
void wifiReconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to reconnect");
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);
      delay(5000);
    }
    Serial.println("\nConnected.");
  }
}

////adds node as root node
void addRootNode(int root_id) {
  if (!isRootNode(root_id)) {
    rootNodes[numRoot] = root_id;
    numRoot += 1;  //increase number of root nodes

    Serial.println("NOTIF: Node " + String(root_id) + " is added as a root node");
  }
}

//// checks node if it is a root node
bool isRootNode(int root_id) {
  for (int i = 0; i < numRoot; i++) {
    if (rootNodes[i] == root_id) {
      return true;  // node found in array
    }
  }
  return false;  // node not found
}

////redirect a node to gateway
void redirectNode() {
  sendRupd = true;
}

//// Send Ack packets
void sendAck(uint8_t ackType, dataPacket pkt) {
  dataPacket ackPkt;

  switch (ackType) {
    case 0:  //Send ack to child node to acknowledge receipt of data
      //ack format: packet_id/Source Node/Packet Type/Ack Type
      ackPkt.packet_id = pkt.packet_id; // packet ID
      ackPkt.owner_id = pkt.owner_id;   // owner ID
      ackPkt.source_id = 0;             // ID of this node
      ackPkt.destination_id = pkt.source_id; //sender of data packet
      ackPkt.packetType = 4;            // packet type
      ackPkt.sensorValue = 0;           //Ack Type
      sendPacket(ackPkt);

      break;
  }
}

//// sends route update to child nodes
void sendRUPD(uint8_t hopCount) {
  dataPacket rupdPkt;
  rupdPkt.packet_id = 0;
  rupdPkt.owner_id = nodeID;
  rupdPkt.source_id = 0;
  rupdPkt.destination_id = 0;
  rupdPkt.packetType = 5;
  rupdPkt.sensorValue = hopCount;
  sendPacket(rupdPkt);
}


//// adds data to buffer containing the id and the sensor value
void addToBuffer(int owner_id, int packet_id, float sensorValue) {
  uint8_t index;
  for (int i = 0; i < sizeof(bufferSize); i++) {
    if (nodes_id[i] == owner_id) {
      index = i;
      break; 
    }
  }
  if (bufferSize[index] < 70) {
    dataBuffer[index][bufferSize[index]].id = packet_id;
    dataBuffer[index][bufferSize[index]].value = sensorValue;
    bufferSize[index]++;
  }
}

////checks of a data packet is already in a buffer to avoid packet duplication
bool isInBuffer(int packet_id, int owner_id) {
  uint8_t index;
  for (int i = 0; i < sizeof(bufferSize); i++) {
    if (nodes_id[i] == owner_id) {
      index = i;
      break; 
    }
  }
  for (int i = bufferSize[index] - 1; i >= 0; i--) {
    if (dataBuffer[index][i].id == packet_id) {
      return true;
    }
  }
  return false;
}

////clears buffer after recording the data inside to the GSHEETS
void clearBufferAfterRecord() {
  for (int i = 0; i < sizeof(bufferSize); i++) {
    if (bufferChecker[i] >= bufferSize[i]) {
      bufferSize[i] = 0;  // If x is larger than or equal to the size, the array becomes empty
      bufferChecker[i] = 0;
    } else {
      for (int x = bufferChecker[i]; x < bufferSize[i]; x++) {
        dataBuffer[i][x - bufferChecker[i]] = dataBuffer[i][x];
      }

      bufferSize[i] -= bufferChecker[i];
      bufferChecker[i] = 0;
    }
  }
}

//// adds RREP to buffer for sending
void addToRrepBuffer(dataPacket pkt) {
  if (rrepBufferSize < sizeof(rrepBuffer)) {
    rrepBuffer[rrepBufferSize] = pkt;
    rrepBufferSize++;
  }
}

////
bool isInRrepBuffer(int id) {
  for (int i = 0; i < rrepBufferSize; i++) {
    if (rrepBuffer[i].sensorValue == id) {
      return true;
    }
  }
  return false;
}

//// removes 1st entry from buffer after transmitting
void removeFromRrepBuffer() {
  for (int i = 1; i < rrepBufferSize; i++) {
    rrepBuffer[i - 1] = rrepBuffer[i];
  }
  rrepBufferSize--;
}

//// gets packet infromation from the packet String
dataPacket getPacketInfo(String receivedPacket){
  dataPacket receivedPkt;

  int first = receivedPacket.indexOf("/", 1);
  int second = receivedPacket.indexOf("/", first + 1);
  int third = receivedPacket.indexOf("/", second + 1);
  int fourth = receivedPacket.indexOf("/", third + 1);
  int fifth = receivedPacket.indexOf("/", fourth + 1);
  int sixth = receivedPacket.indexOf("/", fifth + 1);

  receivedPkt.packet_id = receivedPacket.substring(1, first).toInt();
  receivedPkt.owner_id = receivedPacket.substring(first + 1, second).toInt();
  receivedPkt.source_id = receivedPacket.substring(second + 1, third).toInt();
  receivedPkt.destination_id = receivedPacket.substring(third + 1, fourth).toInt();
  receivedPkt.packetType = receivedPacket.substring(fourth + 1, fifth).toInt();
  receivedPkt.sensorValue = receivedPacket.substring(fifth + 1, sixth).toFloat();

  return receivedPkt;
}

//// get most recent sensor reading to be sent to ThingSpeak channel
void addToRecentData(int owner_id, int packet_id,float sensorValue){
  uint8_t index;
  for (int i = 0; i < sizeof(bufferSize); i++) {
    if (nodes_id[i] == owner_id) {
      index = i;
      break; 
    }
  }
  if(packet_id > recentData[index].id){
    recentData[index].id = packet_id;
    recentData[index].value = sensorValue;
  }
}

////writes data to ThingSpeak channel
void sendToThingSpeak() {
  ThingSpeak.setField(1, recentData[0].value);
  ThingSpeak.setField(2, recentData[1].value);
  ThingSpeak.setField(3, recentData[2].value);
  ThingSpeak.setField(4, recentData[3].value);
  int x = ThingSpeak.writeFields(channelId, writeAPIKey);
    
  if (x == 200) {
    Serial.println("Channel update successful.");
  } else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
}
