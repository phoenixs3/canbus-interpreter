#include <FlexCAN.h>     //https://github.com/collin80/FlexCAN_Library
#include <CANBUS.h>      //https://github.com/seb43654/canbus-interpreter
CAN_message_t outMsg;    //Data structure for outbound messages
CAN_message_t inMsg;     //Data structure for inbound messages
CANBUS canbus;           //Initiate class for canbus-interpreter
char msgString[128];

void setup() {
  Serial.begin(115200);
  Serial.print("CANBUS Interpreter example init...");
  Can0.begin(500000);                                   //Initialise teensy 3.2 canbus
  CAN_filter_t allPassFilter;
  allPassFilter.id = 0;
  allPassFilter.ext = 1;
  allPassFilter.rtr = 0;
  for (int filterNum = 4; filterNum < 16; filterNum++) {Can0.setFilter(allPassFilter, filterNum);}
  Serial.setTimeout(20);
  Serial.println("done");
}

void loop() {
  if (Serial.available() > 0){
    int tempint = Serial.parseInt();
    //Sets requested charging current to serial input (providing its within allowable range)
    if (tempint > 0){
      Serial.print("Your input: ");
      Serial.println(tempint);
      outMsg.id  = 0x025;
      outMsg.len = 8;
      outMsg.ext = 0;
      outMsg.buf[0] = 0x00;  
      outMsg.buf[1] = 0x00;
      outMsg.buf[2] = 0x00;
      outMsg.buf[3] = 0x00;
      outMsg.buf[4] = 0x00;
      outMsg.buf[5] = 0x00;
      outMsg.buf[6] = 0x00;
      outMsg.buf[7] = 0x00;
      outMsg = canbus.encode(outMsg, tempint, 3, 12, "MSB", "SIGNED", 1.5, 0);
      canSend(outMsg);
    }
  }
  while (Can0.available()){canRead();}
}

void canRead(){
  Can0.read(inMsg);
  switch (inMsg.id) {
    case 0x123:
      Serial.print("Example decoded data: ");
      Serial.println(canbus.decode(inMsg, 14, 16, "LSB", "SIGNED", 0.5, -2));
    break; 
    default:
      break;
  }
}

void canSend(CAN_message_t msg){
  sprintf(msgString, "Sent ID: 0x%.3lX, Data: 0x%.3lX, 0x%.3lX, 0x%.3lX, 0x%.3lX, 0x%.3lX, 0x%.3lX, 0x%.3lX, 0x%.3lX", 
  msg.id, msg.buf[0],msg.buf[1],msg.buf[2],msg.buf[3],msg.buf[4],msg.buf[5],msg.buf[6],msg.buf[7]);
  Serial.println(msgString);
  Can0.write(msg);
}
