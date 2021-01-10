/*
  CANBUS Library, created by Seb Smith
*/

#ifndef CANBUS_h
#define CANBUS_h

#include "WProgram.h"
#include <FlexCAN.h>

class CANBUS
{
  public:
    CAN_message_t CANBUS::encodeCAN(CAN_message_t msg, double inputData, int startBit, int bitLength, String byteOrder, String dataType, double Scale, double bias);
    double decode(CAN_message_t msg, int startBit, int bitLength, String byteOrder, String dataType, double Scale, double bias);
  private:
  	String reverseString(String inputString);
  	String toBinary(int input1, int length1);
};

#endif

