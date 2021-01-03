#include <mcp_can.h>

//Data varianbles for CAN
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];


//test variables
int bitlen = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("CANBUS Interpreter init...");

  //Some fake Data
  rxId = 0x256;
  len = 8;
  rxBuf[0] = 0x01;
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
      
     Serial.print("Decoded Data: ");
     Serial.println(decodeCAN(rxBuf, len, 8, 8, "MSB", "SIGNED", 1, 0, 0, 100, false, 0));
    
  }
 
  //delay(1000);

}


//Function for decoding canbus data, returns double of decoded data
//Arguments: Input data array, frame length, starting bit, signal bit length, byte order, return data type (UINT, SINT), scale, bias, minValue, maxValue, mux, muxValue
double decodeCAN(unsigned char rxBufD[], int lengthD, int startBit, int bitLength, String byteOrder, String dataType, double Scale, double bias, double minVal, double maxVal, bool mux, int muxVal){

    //Gets all received data and converts to super long string
    String DataBinaryString;
    for(int x=0; x<lengthD; x++){DataBinaryString = DataBinaryString + toBinary(rxBufD[x], 8);}
    
    Serial.print("Raw Binary Data input: ");
    Serial.println(DataBinaryString);

    //Todo: handle out of bounds selection
    DataBinaryString = DataBinaryString.substring(startBit, (startBit+bitLength));

    Serial.print("Extracted data portion: ");
    Serial.println(DataBinaryString);

    //Todo: add detection for invalid byteorder arguments
    if(byteOrder == "LSB"){DataBinaryString = reverseString(DataBinaryString);}

    Serial.print("Data after byte order correction: ");
    Serial.println(DataBinaryString);  
    
    //First check the length so if signed it can be adjusted
    int maxTheoraticalValue = pow(2,(DataBinaryString.length()))-1;   //Minus one to account for zero

    //Data can now be converted to integer
    char tempholder[64];  //Max Size Data could be
    DataBinaryString.toCharArray(tempholder,16);
    int dataInteger = strtol(tempholder, NULL, 2);

    Serial.print("Data as integer: ");
    Serial.println(dataInteger);

    if(dataType == "SIGNED"){dataInteger = dataInteger - (maxTheoraticalValue/2);}

    Serial.print("Data after being signed: ");
    Serial.println(dataInteger);

    return(dataInteger);
  
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
