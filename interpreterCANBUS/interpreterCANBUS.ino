//#include <ArduinoJson.h>
//#include <mcp_can.h>
#include <FlexCAN.h>  //https://github.com/collin80/FlexCAN_Library
CAN_message_t msg;    //Data structure for outbound messages
CAN_message_t inMsg;  //Data structure for inbound messages

//Data varianbles for CAN
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

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

  //Some fake Data
  rxId = 0x256;
  len = 8;
  rxBuf[0] = 0x7F;
  rxBuf[1] = 0x7E;
  rxBuf[2] = 0x01;
  rxBuf[3] = 0x01;
  rxBuf[4] = 0x01;
  rxBuf[5] = 0x01;
  rxBuf[6] = 0x01;
  rxBuf[7] = 0x01;

  Serial.setTimeout(20);
  Serial.println("done");
}

void loop() {
  if (Serial.available() > 0){
    int tempint = Serial.parseInt();
    //Sets requested charging current to serial input (providing its within allowable range)
    if (tempint > 0){rxBuf[1] = tempint;}

      Serial.print("Received text:");
      Serial.println(tempint);

    //Still undiceded if i want to switch to struct for input variable
     signalData testSignal= {
     };

     double testdata = decodeCAN(rxBuf, len, 0, 8, "MSB", "SIGNED", 1, 0, 0, 100);

     Serial.println("");
     Serial.println("");
     Serial.print("Decoded Data: ");
     Serial.println(testdata);
     Serial.println("");
    
  }
 
  while (Can0.available()){canread();}
}

void canread(){
  Can0.read(inMsg);
  switch (inMsg.id) {
    case 0x123:
    //do somthing
      Serial.print("decoded message:: ");
      double iboosterstroke = decodeCAN(inMsg.buf, inMsg.len, 47, 17, "LSB", "UNSIGNED", 1, 0, -200000, 200000);
      Serial.print(iboosterstroke);
      Serial.println("mm");
    break;
    
    default:
      break;
  }
}

//Function for decoding canbus data, returns double of decoded data
//Arguments: Input data array, frame length, starting bit, signal bit length, byte order, 
//           return data type (UINT, SINT), scale, bias, minValue, maxValue, mux, muxValue
double decodeCAN(unsigned char rxBufD[], int lengthD, int startBit, int bitLength, String byteOrder, 
                              String dataType, double Scale, double bias, double minVal, double maxVal){

    //Gets all received data and converts to super long string
    String DataBinaryString;
    for(int x=0; x<lengthD; x++){    
      //DataBinaryString = DataBinaryString + toBinary(rxBufD[x], 8);

      String tempdata = toBinary(rxBufD[x], 8);

      //For LSB byte order flip each individual chunk of 8 bits around
      if(byteOrder == "LSB" || byteOrder == "lsb" || byteOrder == "intel" || byteOrder == "INTEL"){
        tempdata = reverseString(tempdata);
      }
      DataBinaryString = DataBinaryString + tempdata;     
    }
    
    Serial.print("Raw Binary Data input: ");
    Serial.println(DataBinaryString);

    //Todo: handle out of bounds selection
    if(byteOrder == "MSB" || byteOrder == "msb" || byteOrder == "motorola" || byteOrder == "MOTOROLA"){
        int startbitcalc = 0;
        if(startBit < 8 && startBit > -1){startbitcalc = (7 - startBit);}
        if(startBit < 16 && startBit > 7){startbitcalc = (15 - startBit) + 8;}
        if(startBit < 24 && startBit > 15){startbitcalc = (23 - startBit) + 16;}
        if(startBit < 32 && startBit > 23){startbitcalc = (31 - startBit) + 24;}
        if(startBit < 40 && startBit > 31){startbitcalc = (39 - startBit) + 32;}
        if(startBit < 48 && startBit > 39){startbitcalc = (47 - startBit) + 40;}
        if(startBit < 56 && startBit > 47){startbitcalc = (55 - startBit) + 48;}
        if(startBit < 64 && startBit > 55){startbitcalc = (63 - startBit) + 56;}

        Serial.print("startbitcalc: ");
        Serial.println(startbitcalc);

        Serial.print("endbit: ");
        Serial.println((startbitcalc)+bitLength);
        
        DataBinaryString = DataBinaryString.substring(startbitcalc, ((startbitcalc)+bitLength));
    } else {DataBinaryString = DataBinaryString.substring(startBit, (startBit+bitLength));}

    Serial.print("Extracted data portion: ");
    Serial.println(DataBinaryString);

    //Todo: add detection for invalid byteorder arguments
    if(byteOrder == "LSB" || byteOrder == "lsb" || byteOrder == "intel" || byteOrder == "INTEL"){
      DataBinaryString = reverseString(DataBinaryString);  
    }

    Serial.print("After byte order correction: ");
    Serial.println(DataBinaryString); 

    //First check the length so if signed it can be adjusted
    int maxTheoraticalValue = pow(2,(DataBinaryString.length()))-1;   //Minus one to account for zero

    //Data can now be converted to integer
    char tempholder[64];  //Max Size Data could be
    DataBinaryString.toCharArray(tempholder,64);
    int dataInteger = strtol(tempholder, NULL, 2);

    Serial.print("Raw integer: ");
    Serial.println(dataInteger);

    if(dataType == "SIGNED"){dataInteger = dataInteger - (maxTheoraticalValue/2);}

    Serial.print("After being signed: ");
    Serial.println(dataInteger);

    //Scale & bias/offset
    //Referenced from: https://www.csselectronics.com/screen/page/can-dbc-file-database-intro/language/en
    double returnData = bias + (Scale*dataInteger);

    //Minval, maxval
    if(returnData < minVal){returnData = minVal;}
    if(returnData > maxVal){returnData = maxVal;}
    
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
