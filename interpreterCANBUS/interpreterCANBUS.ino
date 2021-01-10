//#include <ArduinoJson.h>
//#include <mcp_can.h>
#include <FlexCAN.h>  //https://github.com/collin80/FlexCAN_Library
CAN_message_t msg;    //Data structure for outbound messages
CAN_message_t inMsg;  //Data structure for inbound messages

//Data varianbles for CAN
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

char msgString[128]; 

struct signalData {
  int sigLength;
  int startBit;
  int bitLength;
  int byteOrder;
  int dataType;
  double Scale;
  double bias;
  double minVal;
  double maxVal;
};

void setup() {
  Serial.begin(115200);
  Serial.print("CANBUS Interpreter init...");

  //Initialise teensy 3.2 canbus
  Can0.begin(500000);
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
      
      Serial.print("Received text: ");
      Serial.println(tempint);

      msg.id  = 0x123;
      msg.len = 8;
      msg.ext = 0;
      msg.buf[0] = 0x00;  
      msg.buf[1] = 0x00;
      msg.buf[2] = 0x01;
      msg.buf[3] = 0x01;
      msg.buf[4] = 0x01;
      msg.buf[5] = 0x01;
      msg.buf[6] = 0x01;
      msg.buf[7] = 0x01;

      Serial.println("");
      Serial.println("Outbound message before encode:");
      Serial.print("ID: ");
      Serial.println(msg.id);
      Serial.print("LENGTH: ");
      Serial.println(msg.len);
      Serial.print("EXT: ");
      Serial.println(msg.ext);
      Serial.println("DATA: ");
      Serial.println(msg.buf[0]);
      Serial.println(msg.buf[1]);
      Serial.println(msg.buf[2]);
      Serial.println(msg.buf[3]);
      Serial.println(msg.buf[4]);
      Serial.println(msg.buf[5]);
      Serial.println(msg.buf[6]);
      Serial.println(msg.buf[7]);
      Serial.println("");

      msg = encodeCAN(msg, 2014, 14, 16, "LSB", "UNSIGNED", 1, 0);

      Serial.println("");
      Serial.println("Outbound message after encode:");
      Serial.print("ID: ");
      Serial.println(msg.id);
      Serial.print("LENGTH: ");
      Serial.println(msg.len);
      Serial.print("EXT: ");
      Serial.println(msg.ext);
      Serial.println("DATA: ");
      Serial.println(msg.buf[0]);
      Serial.println(msg.buf[1]);
      Serial.println(msg.buf[2]);
      Serial.println(msg.buf[3]);
      Serial.println(msg.buf[4]);
      Serial.println(msg.buf[5]);
      Serial.println(msg.buf[6]);
      Serial.println(msg.buf[7]);
      Serial.println("");

      SendCan(msg);

    }
    
  }
  while (Can0.available()){canread();}
}

void canread(){
  Can0.read(inMsg);
  switch (inMsg.id) {
    case 0x12C:
      Serial.print("decoded data: ");
      double decodeddata = decodeCAN(inMsg.buf, inMsg.len, 38, 10, "LSB", "UNSIGNED", 2, -0.5);
      Serial.print(decodeddata);
      Serial.println(" units");
    break;
    
    default:
      break;
  }
}

CAN_message_t encodeCAN(CAN_message_t msg, double inputData, int startBit, int bitLength, String byteOrder, String dataType, double Scale, double bias){

    //////////////////////////////////////////////////////////////////////////////
    //Step 1 Reverse Scale and bias
    //////////////////////////////////////////////////////////////////////////////
    inputData = (1/Scale) * (inputData - bias);
    Serial.print("After scale and bias: ");
    Serial.println(inputData);
    //ok

    //////////////////////////////////////////////////////////////////////////////
    //Step 2 Account for signed values
    //////////////////////////////////////////////////////////////////////////////
    int maxTheoraticalValue = (pow(2,bitLength)-1);                                                        //Minus one to account for zero
    Serial.print("max theoretical value:");
    Serial.println(maxTheoraticalValue);
    
    if(dataType == "SIGNED" || dataType == "signed"){
      if(inputData < 0){inputData = inputData + (maxTheoraticalValue+1);}                                  //Convert to signed value
      Serial.print("After being signed: ");
      Serial.println(inputData);
    }
    //ok(i think)
    
    //////////////////////////////////////////////////////////////////////////////
    //Step 3 Convert to string
    //////////////////////////////////////////////////////////////////////////////
    String DataBinaryString = toBinary((int)inputData, bitLength);
    Serial.print("DataBinaryString: ");
    Serial.println(DataBinaryString);
    //ok


    //////////////////////////////////////////////////////////////////////////////
    //Step 4 - Byte order correction
    //////////////////////////////////////////////////////////////////////////////
    if(byteOrder == "LSB" || byteOrder == "lsb" || byteOrder == "intel" || byteOrder == "INTEL"){
      DataBinaryString = reverseString(DataBinaryString);  
    }
    Serial.print("After byte order correction: ");
    Serial.println(DataBinaryString);
    //not working!!!

    //////////////////////////////////////////////////////////////////////////////
    //Step 5 - Calculate section of string to insert into
    //////////////////////////////////////////////////////////////////////////////
    int startbitcalc;
    int endbitcalc;
    
    if(byteOrder == "MSB" || byteOrder == "msb" || byteOrder == "motorola" || byteOrder == "MOTOROLA"){
        //needs redoing
        if(startBit < 8 && startBit > -1) {startbitcalc = (7 - startBit);}
        if(startBit < 16 && startBit > 7) {startbitcalc = (15 - startBit) + 8;}
        if(startBit < 24 && startBit > 15){startbitcalc = (23 - startBit) + 16;}
        if(startBit < 32 && startBit > 23){startbitcalc = (31 - startBit) + 24;}
        if(startBit < 40 && startBit > 31){startbitcalc = (39 - startBit) + 32;}
        if(startBit < 48 && startBit > 39){startbitcalc = (47 - startBit) + 40;}
        if(startBit < 56 && startBit > 47){startbitcalc = (55 - startBit) + 48;}
        if(startBit < 64 && startBit > 55){startbitcalc = (63 - startBit) + 56;}
        endbitcalc = ((startbitcalc)+bitLength);
    } else {
      startbitcalc = startBit;
      endbitcalc = (startBit+bitLength);
      //checked, ok for lsb (endbit maybe one too many though
    }    
    Serial.print("startbitcalc: ");
    Serial.println(startbitcalc);
    Serial.print("endbitcalc: ");
    Serial.println(endbitcalc);

    //////////////////////////////////////////////////////////////////////////////
    //Step 6 - Gets current data and make very long string
    //////////////////////////////////////////////////////////////////////////////
    String inputDataBinaryString;
    for(int x=0; x<msg.len; x++){    
      String tempdata = toBinary(msg.buf[x], 8); 
      if(byteOrder == "LSB" || byteOrder == "lsb" || byteOrder == "intel" || byteOrder == "INTEL"){
        tempdata = reverseString(tempdata);
      }
      inputDataBinaryString = inputDataBinaryString + tempdata;                                         //Merge into mega long string
    } 
    Serial.print("Raw Binary Data input: ");
    Serial.println(inputDataBinaryString);
    
    //////////////////////////////////////////////////////////////////////////////
    //Step 7 - Split input data string into chunks for left and right and merge in encoded data
    //////////////////////////////////////////////////////////////////////////////
    String leftportion = "";
    String rightportion = "";
    
    if(startbitcalc > 0){                                                                 //Check if not at very start of string
      leftportion = inputDataBinaryString.substring(0, startbitcalc);
    }
    if(endbitcalc < inputDataBinaryString.length()){                                      //Check if not all the way at the end
      rightportion = inputDataBinaryString.substring(endbitcalc, inputDataBinaryString.length());
    }

    Serial.print("leftportion: ");
    Serial.println(leftportion);
    Serial.print("rightportion: ");
    Serial.println(rightportion);

    DataBinaryString = leftportion + DataBinaryString + rightportion;                     //Merge together
    Serial.print("After merge together:  ");
    Serial.println(DataBinaryString);
    //ok
    
    //////////////////////////////////////////////////////////////////////////////
    //Step 8 - convert bytes to decimal and return message
    //////////////////////////////////////////////////////////////////////////////

    //need to swap each byte again for lsb!!!
    char byte0[9];
    char byte1[9];
    char byte2[9];
    char byte3[9];
    char byte4[9];
    char byte5[9];
    char byte6[9];
    char byte7[9];
    if(byteOrder == "LSB" || byteOrder == "lsb" || byteOrder == "intel" || byteOrder == "INTEL"){
      String byte0s = reverseString(DataBinaryString.substring(0,8));
      String byte1s = reverseString(DataBinaryString.substring(8,16));
      String byte2s = reverseString(DataBinaryString.substring(16,24));
      String byte3s = reverseString(DataBinaryString.substring(24,32));
      String byte4s = reverseString(DataBinaryString.substring(32,40));
      String byte5s = reverseString(DataBinaryString.substring(40,48));
      String byte6s = reverseString(DataBinaryString.substring(48,56));
      String byte7s = reverseString(DataBinaryString.substring(56,64));
      byte0s.toCharArray(byte0,9);
      byte1s.toCharArray(byte1,9);
      byte2s.toCharArray(byte2,9);
      byte3s.toCharArray(byte3,9);
      byte4s.toCharArray(byte4,9);
      byte5s.toCharArray(byte5,9);
      byte6s.toCharArray(byte6,9);
      byte7s.toCharArray(byte7,9); 
    }else{
    DataBinaryString.substring(0,8).toCharArray(byte0,9);
    DataBinaryString.substring(9,16).toCharArray(byte1,9);
    DataBinaryString.substring(16,24).toCharArray(byte2,9);
    DataBinaryString.substring(24,32).toCharArray(byte3,9);
    DataBinaryString.substring(32,40).toCharArray(byte4,9);
    DataBinaryString.substring(40,48).toCharArray(byte5,9);
    DataBinaryString.substring(48,56).toCharArray(byte6,9);
    DataBinaryString.substring(56,64).toCharArray(byte7,9); 
    }
    msg.buf[0] = strtol(byte0, NULL, 2);
    msg.buf[1] = strtol(byte1, NULL, 2);
    msg.buf[2] = strtol(byte2, NULL, 2);
    msg.buf[3] = strtol(byte3, NULL, 2);
    msg.buf[4] = strtol(byte4, NULL, 2);
    msg.buf[5] = strtol(byte5, NULL, 2);
    msg.buf[6] = strtol(byte6, NULL, 2);
    msg.buf[7] = strtol(byte7, NULL, 2);
    return msg;
    //ok
}

//Function for decoding canbus data, returns double of decoded data
//Arguments: Input data array, frame length, starting bit, signal bit length, byte order, 
//           return data type (unsigned, signed), scale, bias, minValue, maxValue, mux, muxValue
double decodeCAN(unsigned char rxBufD[], int lengthD, int startBit, int bitLength, String byteOrder, String dataType, double Scale, double bias){


    //////////////////////////////////////////////////////////////////////////////
    //Step 1 - Gets all received data and converts to super long string
    //////////////////////////////////////////////////////////////////////////////
    String DataBinaryString;
    for(int x=0; x<lengthD; x++){    
      String tempdata = toBinary(rxBufD[x], 8); 
      if(byteOrder == "LSB" || byteOrder == "lsb" || byteOrder == "intel" || byteOrder == "INTEL"){
        tempdata = reverseString(tempdata);                                                               //For LSB byte order flip each byte around first
      }
      DataBinaryString = DataBinaryString + tempdata;                                                     //Merge into mega long string
    } 
    //Serial.print("Raw Binary Data input: ");
    //Serial.println(DataBinaryString);


    //////////////////////////////////////////////////////////////////////////////
    //Step 2 - Calculate section of string to select
    //////////////////////////////////////////////////////////////////////////////
    if(byteOrder == "MSB" || byteOrder == "msb" || byteOrder == "motorola" || byteOrder == "MOTOROLA"){
        int startbitcalc = 0;
        if(startBit < 8 && startBit > -1) {startbitcalc = (7 - startBit);}
        if(startBit < 16 && startBit > 7) {startbitcalc = (15 - startBit) + 8;}
        if(startBit < 24 && startBit > 15){startbitcalc = (23 - startBit) + 16;}
        if(startBit < 32 && startBit > 23){startbitcalc = (31 - startBit) + 24;}
        if(startBit < 40 && startBit > 31){startbitcalc = (39 - startBit) + 32;}
        if(startBit < 48 && startBit > 39){startbitcalc = (47 - startBit) + 40;}
        if(startBit < 56 && startBit > 47){startbitcalc = (55 - startBit) + 48;}
        if(startBit < 64 && startBit > 55){startbitcalc = (63 - startBit) + 56;}
        //Serial.print("startbitcalc: ");
        //Serial.println(startbitcalc);
        //Serial.print("endbit: ");
        //Serial.println((startbitcalc)+bitLength);     
        DataBinaryString = DataBinaryString.substring(startbitcalc, ((startbitcalc)+bitLength));
    } else {DataBinaryString = DataBinaryString.substring(startBit, (startBit+bitLength));}
    //Serial.print("Extracted data portion: ");
    //Serial.println(DataBinaryString);


    //////////////////////////////////////////////////////////////////////////////
    //Step 3 - Byte order correction
    //////////////////////////////////////////////////////////////////////////////
    if(byteOrder == "LSB" || byteOrder == "lsb" || byteOrder == "intel" || byteOrder == "INTEL"){
      DataBinaryString = reverseString(DataBinaryString);  
    }
    //Serial.print("After byte order correction: ");
    //Serial.println(DataBinaryString); 


    //////////////////////////////////////////////////////////////////////////////
    //Step 4 - Convert to integer
    //////////////////////////////////////////////////////////////////////////////
    int maxTheoraticalValue = (pow(2,(DataBinaryString.length()))-1);                                    //Minus one to account for zero
    //Serial.print("max theoretical value:");
    //Serial.println(maxTheoraticalValue);
    char tempholder[64];                                                                                 //char array with max size of data
    DataBinaryString.toCharArray(tempholder,64);                                                         //string to char array
    int dataInteger = strtol(tempholder, NULL, 2);                                                       //Convert from binary (base 2) to decimal
    //Serial.print("Raw integer: ");
    //Serial.println(dataInteger);
    
    
    //////////////////////////////////////////////////////////////////////////////
    //Step 4 - Account for signed values
    //////////////////////////////////////////////////////////////////////////////
    if(dataType == "SIGNED" || dataType == "signed"){
      if(dataInteger > (maxTheoraticalValue/2)){dataInteger = dataInteger - (maxTheoraticalValue+1);}    //Convert to signed value
    }
    //Serial.print("After being signed: ");
    //Serial.println(DataBinaryString);


    //////////////////////////////////////////////////////////////////////////////
    //Step 5 - Scale and bias
    //////////////////////////////////////////////////////////////////////////////
    //Referenced from: https://www.csselectronics.com/screen/page/can-dbc-file-database-intro/language/en
    double returnData = bias + (Scale*dataInteger);   
    
    return(returnData); 
}

String reverseString(String inputString){
      //Simply reverses string (used for MSB > LSB conversion)
      String returnString;
      for(int x=inputString.length(); x>0; x--){returnString = returnString + inputString.substring(x, (x-1));}
      return(returnString);  
}

String toBinary(int input1, int length1){ 
    //Function that returns a string of the input binary,and pads out with zeros to match the length requested
    String tempString = String(input1, BIN);
    if(tempString.length() == length1){return tempString;} else {
      int paddingToAdd = length1-tempString.length();
      String zeros = "";
      for (int i = 1; i <= paddingToAdd; i++) {zeros = zeros + "0";}
      tempString = zeros + tempString;
    }  
}

void SendCan(CAN_message_t msg){
  sprintf(msgString, "Sent ID: 0x%.3lX, Data: 0x%.3lX, 0x%.3lX, 0x%.3lX, 0x%.3lX, 0x%.3lX, 0x%.3lX, 0x%.3lX, 0x%.3lX", msg.id, msg.buf[0],msg.buf[1],msg.buf[2],msg.buf[3],msg.buf[4],msg.buf[5],msg.buf[6],msg.buf[7]);
  Serial.println(msgString);
  Can0.write(msg);
}
