/*
  Project: LoRa Mesh Sensor Network for Air Quality and Exposure Data in Jeepneys  
  Author: Urmeneta, Kriz Greg
  Device: Sensor Node
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

/*
  TODO:
    - Change LoRa settings as needed
    - add sequence number to avoid loop
    - (Potential) if gateway receive dataPacket from nodes that are not 
      directly connected, try reconnecting it directly to gateway
  Notes:
    - Data packets are retransmitted continuously until ACKED
    - if buffer is 50% full broadcast RERR, consider disconnected from mesh netwrok
      and find other routes
    - Wait 30 seconds to reconnect if still not connected, broadcast RERR for child nodes to find other route
*/

// ================== LIBRARIES ==================

///Sensor Libraries
#include "PMS.h"
#include <SoftwareSerial.h>

///Lora Libraries
#include <SPI.h>
#include <LoRa.h>


// ================== PIN DEFINITIONS ==================

#define ss 15   //NSS - D8
#define rst 16  //RST - D0
#define dio0 4  //DIO0 - D2

SoftwareSerial pmsSerial(2, 0);  //RX - D3    TX - D4

PMS pms(pmsSerial);
PMS::DATA data;


// ================== GLOBAL VARIABLES ==================
/*
Node0 = 5814650
Node1 = 3980550
Node2 = 15754626
Node3 = 4024082
Node4 = 14715324
*/
int nodeUID = ESP.getChipId();  //DEVICE UNIQUE ID

float reading;

//interval1 for Data collecting ; interval2 for data sending from buffer; interval 3 for reconnection; interval 4 to wait for RREPs
unsigned long previousMillis = 0, previousMillis2 = 0, previousMillis3 = 0, previousMillis4 = 0, previousMillis5 = 0;  // last time event triggered
const long interval = 5000, interval2 = 2000, interval3 = 120000, interval4 = 5000;                 // intervals

//Struct for Packets
struct dataPacket {
  int packet_id;       // ID of the packet
  int owner_id;        // ID of the owner/creator of the packet
  int source_id;       //ID of the sender/forwarder node
  int destination_id;  // ID of the destination node of packet
  int routeNumber;     // Used to check if in the same existing route to avoid loop
  uint8_t packetType;  //0 = Data, 1 = RREQ, 2 = RREP, 3 = RERR, 4 = Ack, 5 = RUPD, 6 = NETR
  float sensorValue;   // Sensor reading
};

dataPacket packet;
dataPacket forwardPkt;
dataPacket packetBuffer[100];
uint8_t bufferSize = 0;

int aveRTT = 0;
float throughput = 0;
unsigned long RTTsize = 0;
unsigned long packetTime_start;
unsigned long packetSent=0;
unsigned long totalBytes=0;


//Struct for routing
struct routeTable {
  int parentNode = 0.;
  int childNodes[10];
  uint8_t numChildren = 0;
  uint8_t hopCount = 255;
};

routeTable route;

int packetCounter = 0;
int nodeID = nodeUID;

int candidateNodes[10][4];
int candidates = 0;
bool meshConnected = false;
bool reconnecting = false;
int retransmission = 0;

bool send = false;
bool sendDataAck = false;
bool sendRupd = false;

// ================== SETUP ==================

void setup() {
  Serial.begin(9600);
  pmsSerial.begin(9600);

  //print UID for identification,debugging
  //Serial.println("--------SENSOR NODE UID: " + String(nodeUID) + "--------");

  while (!Serial);
  //Serial.println("LoRa MESH SENSOR NODE STARTING...");
  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(433E6)) {
    //Serial.println("Starting LoRa failed!");
    delay(100);
    while (1)
      ;
  }

  //LoRa.setSpreadingFactor(7); //Supported values are between 6 and 12
  //LoRa.setSignalBandwidth(signalBandwidth); //Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
  //LoRa.setCodingRate4(codingRateDenominator); //Supported values are between 5 and 8
  //LoRa.setPreambleLength(preambleLength); //Supported values are between 6 and 65535
  //LoRa.setTxPower(txPower, outputPin); Supported values are 2 to 20

  LoRa.onReceive(onReceive);  // register the receive callback

  LoRa.receive();  // put the radio into receive mode
  

  while (!meshConnected) {
    broadcastRREQ();  // broadcast RREQ to find nodes
    delay(5000);      //wait for RREPs for 3 seconds
    if (meshConnected == false && candidates > 0) {
      findParentNode();  //find for best node based on Hop count and RSSI
      meshConnected = true;
    }
  }
}


// ================== MAIN LOOP ==================

void loop() {

  unsigned long currentMillis = millis();  // Get current time

  //get sensor reading
  if (pms.read(data)) {
    reading = data.PM_AE_UG_2_5;
  }
    //send data on interval
    if (currentMillis - previousMillis >= interval) {
      dataPacket dataPkt;

      packetCounter += 1;  //increase packet id
      dataPkt.packet_id = packetCounter;
      dataPkt.owner_id = nodeID;
      dataPkt.source_id = nodeID;
      dataPkt.destination_id = route.parentNode;
      dataPkt.packetType = 0;
      dataPkt.sensorValue = reading;
      if (inBuffer(dataPkt.packet_id, dataPkt.owner_id) == -1) {
        appendToBuffer(dataPkt);
      }

      sendNETR();
      previousMillis = currentMillis;
    }

  //if there are packets to send in FIFO buffer/queue
  if (bufferSize > 0 && meshConnected) {
    
    if (currentMillis - previousMillis2 >= interval2) {
      //Serial.println("SEND: Data Packet #" + String(packetBuffer[0].packet_id) + " from Node " + String(packetBuffer[0].owner_id) + " SENT to parent Node " + String(route.parentNode));

      packetTime_start = millis();
      forwardPacket(packetBuffer[0]);  //send a data packet from buffer

      retransmission++;
      

      String stringByte = "/" + String(packetBuffer[0].packet_id) + "/" + String(packetBuffer[0].owner_id) + "/" + String(packetBuffer[0].source_id) + +"/" + String(packetBuffer[0].destination_id) + "/" + String(packetBuffer[0].packetType) + "/" + String(packetBuffer[0].sensorValue);
      totalBytes += stringByte.length();

      previousMillis2 = currentMillis;
    }
  }

  //if retranmission timeout of 120seconds, try to find other route
  if (retransmission > 60) {
    retransmission = 0;
    //set connected to false and reconnecting to true
    if (meshConnected) {
      meshConnected = false;
      reconnecting = true;
      previousMillis3 = currentMillis;
    }
  }

  //if this node still trying to reconnect keep broadcasting RREQ
  if (reconnecting) {
    meshConnect();
    //reconnection timeout after 120s of reconnecting, broadcast RERR and keep reconnecting to mesh network
    if (currentMillis - previousMillis3 >= interval3) {
      broadcastRERR();
      clearRouteTable();
      previousMillis3 = currentMillis;
    }
  }

  //send packets (general i.e. RREP)
  if (send) {
    delay(random(301));
    sendPacket(packet);
    send = false;
  }

  //send ack for received data packet
  if (sendDataAck) {
    sendAck(0, forwardPkt);  //send ack
    //Serial.println("SEND: Data Packet #" + String(forwardPkt.packet_id) + " from Node " + String(forwardPkt.owner_id) + " is ACKNOWLEDGED!");

    sendDataAck = false;
  }

  //send route updates to child nodes
  if (sendRupd) {
    sendRUPD(route.hopCount);
    sendRupd = false;
  }
}


// ================== FUNCTIONS ==================

//// sending packet function
void sendPacket(dataPacket sendPacket) {
  // Packet format: /packet ID/Packet owner/Source node/Packet type/Sensor reading
  String dataString = "/" + String(sendPacket.packet_id) + "/" + String(sendPacket.owner_id) + "/" + String(sendPacket.source_id)  + "/" + String(sendPacket.destination_id) + "/" + String(sendPacket.packetType) + "/" + String(sendPacket.sensorValue);

  //Send data packet
  LoRa.beginPacket();
  LoRa.print(dataString);
  LoRa.endPacket();
  LoRa.receive();
}

//// forwarding packet to parent node function
void forwardPacket(dataPacket sendPacket) {
  // Packet format: /packet ID/Packet owner/Source node/Packet type/Sensor reading
  String dataString = "/" + String(sendPacket.packet_id) + "/" + String(sendPacket.owner_id) + "/" + String(sendPacket.source_id)  + "/" + String(route.parentNode) + "/" + String(sendPacket.packetType) + "/" + String(sendPacket.sensorValue);

  //Send data packet
  LoRa.beginPacket();
  LoRa.print(dataString);
  LoRa.endPacket();
  LoRa.receive();
}

//// Execute when receiving packet
//Packet String format: Packet ID/Packet Owner/Packet Sender/Packet Destination/Packet Type/Sensor Value
void onReceive(int packetSize) {
  String receivedPacket = "";

  // read packet
  for (int i = 0; i < packetSize; i++) {
    receivedPacket += (char)LoRa.read();
  }

  dataPacket receivedPkt = getPacketInfo(receivedPacket);  //parses packet information from the String

  int hopCount = 0;
  uint8_t ackType = receivedPkt.sensorValue;
  int index = -1;

  switch (receivedPkt.packetType) {
    case 0:  //Data

      // only forward data packet if sender is a child node
      if (receivedPkt.destination_id == nodeID) {
        forwardPkt.packet_id = receivedPkt.packet_id;
        forwardPkt.owner_id = receivedPkt.owner_id;
        forwardPkt.source_id = nodeID;
        forwardPkt.destination_id = route.parentNode;
        forwardPkt.packetType = 0;
        forwardPkt.sensorValue = receivedPkt.sensorValue;
        if (inBuffer(forwardPkt.packet_id, forwardPkt.owner_id) == -1) {
          appendToBuffer(forwardPkt);  //append the received packet to buffer to be forwarded
        }

        forwardPkt.source_id = receivedPkt.source_id;
        sendDataAck = true;
      }

      break;

    case 1:  //RREQ
      //Serial.println("RECEIVED: received RREQ from Node " + String(receivedPkt.owner_id));

      //if this owner node is connected to the mesh network and RREQ send is not the parent node, reply with an RREP
      if (meshConnected && receivedPkt.owner_id != route.parentNode && route.hopCount < receivedPkt.sensorValue) {
        packet.packet_id = 0;  //Route number
        packet.owner_id = nodeID;
        packet.source_id = nodeID;
        packet.destination_id = receivedPkt.source_id;
        packet.packetType = 2;
        packet.sensorValue = route.hopCount;  //Hop count
        send = true;

        //Serial.println("SEND: RREP sent to Node " + String(receivedPkt.owner_id));
      }
      break;

    case 2:  //RREP
      //Serial.println("RECEIVED: received RREP from Node " + String(receivedPkt.owner_id));

      //check if RREP is destined to this node
      if (receivedPkt.destination_id == nodeID) {
        if (!meshConnected) {
          //RREP format: /Hop count/Owner id/Source node/Packet Type/Don't care
          hopCount = receivedPkt.sensorValue;
          if (hopCount == 0) {  //if requested node is the gateway node
          
            route.parentNode = 0;
            route.hopCount = 1;

            //Ack gateway's RREP and confirm it to be parent node
            packet.packet_id = 0;
            packet.owner_id = nodeID;
            packet.source_id = nodeID;
            packet.destination_id = 0;
            packet.packetType = 4;
            packet.sensorValue = 1;
            send = true;

            meshConnected = true;
            reconnecting = false;
            retransmission = 0;
          } else {  //else add to candidates of parent node
            candidateNodes[candidates][0] = receivedPkt.owner_id;
            candidateNodes[candidates][1] = hopCount;
            candidateNodes[candidates][2] = LoRa.packetRssi();
            candidates += 1;

            //Serial.println("NOTIF: Node " + String(receivedPkt.owner_id) + "'s RSSI:" + String(LoRa.packetRssi()));
          }
        }
      }

      break;

    case 3:  //RERR

      if (receivedPkt.owner_id == route.parentNode) {
        if (meshConnected) {
          meshConnected = false;
          reconnecting = true;
          previousMillis3 = millis();
        }
      } else if (isChildNode(receivedPkt.owner_id)) {
        removeChild(receivedPkt.owner_id);
      }

      break;

    case 4:  //Ack
      if (receivedPkt.destination_id == nodeID) {
        switch (ackType) {
          case 0:  //ACKs for own data packets sent and forwarded data packets
            index = inBuffer(receivedPkt.packet_id, receivedPkt.owner_id);
            if (index != -1) {
              removeFromBuffer(index);
              packetSent++;
              recordNETR();
              //Serial.println("RECEIVED: Data Packet #" + String(receivedPkt.packet_id) + " from Node " + String(receivedPkt.owner_id) + " ACKED and REMOVED from buffer!");
            }
            break;
          case 1:  //ACKs for RREQ to confirm chosen parent node/route
            if (!isChildNode(receivedPkt.owner_id)) {
              addChildNode(receivedPkt.owner_id);

              //Serial.println("RECEIVED: Node " + String(receivedPkt.owner_id) + " is added as a child node");
            }
            break;
        }
      }
      break;

    case 5:  //RUPD

      //update hop count and send route updates to child
      if (receivedPkt.source_id == route.parentNode) {
        route.hopCount = receivedPkt.sensorValue + 1;
        sendRupd = true;
      }
      //else if route update is from gateway
      else if(receivedPkt.source_id == 0){
        route.parentNode = 0;
        route.hopCount = 1;
        sendRupd = true;
      }
      break;
  }
}

//// Broadcast RREQ packets
void broadcastRREQ() {
  //Send RREQ
  String RREQ = "/0/" + String(nodeID) + "/" + String(nodeID) + "/0/1/" + route.hopCount;
  LoRa.beginPacket();
  LoRa.print(RREQ);
  LoRa.endPacket();
  LoRa.receive();

  //Serial.println("SEND: Connecting to Mesh Network, RREQ sent!");
}

//// find optimal node to forward data packets
void findParentNode() {
  int optimalNode = candidateNodes[0][0];
  uint8_t optimalHop = candidateNodes[0][1];
  int optimalRssi = candidateNodes[0][2];

  for (int i = 1; i < candidates; i++) {
    uint8_t hop = candidateNodes[i][1];
    int rssi = candidateNodes[i][2];

    if (hop < optimalHop) {
      optimalHop = hop;
      optimalRssi = rssi;
      optimalNode = candidateNodes[i][0];
    }

    else if (hop == optimalHop && rssi > optimalRssi) {
      optimalRssi = rssi;
      optimalNode = candidateNodes[i][0];
    }
  }

  route.parentNode = optimalNode;
  route.hopCount = optimalHop + 1;
  candidates = 0;

  //Serial.println("NOTIF: Optimal RSSI: " + String(optimalRssi));

  dataPacket pkt;
  pkt.packet_id = 0;
  pkt.owner_id = 0;
  pkt.source_id = 0;
  pkt.destination_id = optimalNode;
  pkt.packetType = 0;
  pkt.sensorValue = 0;  //send optimal node's ID
  sendAck(1, pkt);
}

////Tries to connect to mesh network
void meshConnect() {
  unsigned long currentMillis = millis();  // Get current time

  // broadcast RREQ to find nodes
  if (currentMillis - previousMillis5 >= interval) {  
    broadcastRREQ(); 
    previousMillis5 = currentMillis;
  }  

  if (currentMillis - previousMillis4 >= interval4) {
    if (meshConnected == false && candidates > 0) {
      findParentNode();  //find for best node based on Hop count and RSSI
      meshConnected = true;
      reconnecting = false;
      retransmission = 0;

      //send route updates to children
      sendRUPD(route.hopCount);
    }
    previousMillis4 = currentMillis;
  }
}

////Broadcast RERR packets
void broadcastRERR() {
  //Send RERR
  String RERR = "/0/" + String(nodeID) + "/" + String(nodeID) + "/0/3/0";
  LoRa.beginPacket();
  LoRa.print(RERR);
  LoRa.endPacket();
  LoRa.receive();

  //Serial.println("SEND: Reconnecting, RERR sent!");
}

//// Send Ack packets
void sendAck(uint8_t ackType, dataPacket pkt) {
  dataPacket ackPkt;

  switch (ackType) {
    case 0:  //Send ack to child node to acknowledge receipt of data
      //ack format: Packet_id/Owner Node/Source Node/Destination Node/Packet Type/Ack Type
      ackPkt.packet_id = pkt.packet_id;       //packet ID
      ackPkt.owner_id = pkt.owner_id;         //owner ID
      ackPkt.source_id = nodeID;              //ID of this node
      ackPkt.destination_id = pkt.source_id;  // ID of the child node who sent/forwarded the data
      ackPkt.packetType = 4;
      ackPkt.sensorValue = 0;  //Ack Type
      sendPacket(ackPkt);

      break;

    case 1:  //Send ack to parent node for RREQ
      //ack format: Receiver Node/Owner Node/Source Node/Destination Node/Packet Type/Ack Type
      ackPkt.packet_id = 0;                        //dont care
      ackPkt.owner_id = nodeID;                    //ID of this node
      ackPkt.source_id = nodeID;                   //ID of this node
      ackPkt.destination_id = pkt.destination_id;  //id of optimal node
      ackPkt.packetType = 4;
      ackPkt.sensorValue = 1;  //Ack Type
      sendPacket(ackPkt);

      //Serial.println("SEND: Node " + String(ackPkt.destination_id) + " chosen as parent node");
      break;
  }
}

//// sends route update to child nodes
void sendRUPD(uint8_t hopCount) {
  dataPacket rupdPkt;
  rupdPkt.packet_id = 0;
  rupdPkt.owner_id = nodeID;
  rupdPkt.source_id = nodeID;
  rupdPkt.destination_id = 0;
  rupdPkt.packetType = 5;
  rupdPkt.sensorValue = hopCount;
  sendPacket(rupdPkt);

  //Serial.println("SEND: Route Updates sent to Child Nodes");
}


////adds a child node on the routing table
void addChildNode(int childId) {
  if (!isChildNode(childId)) {
    route.childNodes[route.numChildren] = childId;
    route.numChildren++;  //increase number of children
  }
}

//// checks node if it is a child node
bool isChildNode(int childId) {
  for (int i = 0; i < route.numChildren; i++) {
    if (route.childNodes[i] == childId) {
      return true;  // node found in table
    }
  }
  return false;  // node not found
}

//// removes specific child node from route table
void removeChild(int childId) {
  int index = -1;
  for (int i = 0; i < route.numChildren; i++) {
    if (route.childNodes[i] == childId) {
      index = i;
      break;
    }
  }

  if (index == -1) {
    return;
  }

  for (int i = index; i < route.numChildren - 1; i++) {
    route.childNodes[i] = route.childNodes[i + 1];
  }

  route.numChildren--;
}

//// adds a data packet to the buffer, only remove when ack is received from receiver
void appendToBuffer(dataPacket pkt) {
  if (bufferSize < 100) {
    packetBuffer[bufferSize] = pkt;
    bufferSize++;
  }

  //remove least recently added (FIFO)
  else{
    for(int i=1; i<100; i++){
      packetBuffer[i-1] = packetBuffer[i];
    }
    packetBuffer[99] = pkt;
  }
}

//// removes the packet that was acknowledged
void removeFromBuffer(uint8_t index) {
  for (int i = index; i < bufferSize - 1; i++) {
    packetBuffer[i] = packetBuffer[i + 1];
  }

  bufferSize--;
}

////checks if data packet is in buffer waiting to be acknowledged by receiver return -1 if in not in buffer
int inBuffer(int pkt_id, int owner_id) {
  int index = -1;
  for (int i = 0; i < bufferSize; i++) {
    if (pkt_id == packetBuffer[i].packet_id && owner_id == packetBuffer[i].owner_id) {
      return index = i;
    }
  }

  return index;
}

////Clears routing table
void clearRouteTable() {
  for (int i = 0; i < route.numChildren; i++) {
    route.childNodes[i] = 0;
  }
  route.numChildren = 0;
}

//// gets packet infromation from the packet String
dataPacket getPacketInfo(String receivedPacket) {
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

//send Network record to update records of network performance including average RTT and Througput
void recordNETR(){
    int RTT = millis() - packetTime_start;
    
    RTTsize++;
    aveRTT = (aveRTT*(RTTsize-1) + RTT)/RTTsize;
    throughput = (totalBytes/packetSent)/(aveRTT*0.001);
}

void sendNETR(){
  dataPacket netrPkt;
  netrPkt.packet_id = aveRTT; //average round-trip-time of packets
  netrPkt.owner_id = nodeID;
  netrPkt.source_id = packetSent; //total number of packets sent/forwarded
  netrPkt.destination_id = totalBytes; //total number of bytes sent/forwarded
  netrPkt.packetType = 6;
  netrPkt.sensorValue = throughput; //Througput
  sendPacket(netrPkt);
}