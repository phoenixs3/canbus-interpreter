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
    void encode();
    double decode(CAN_message_t msg, int startBit, int bitLength, String byteOrder, String dataType, double Scale, double bias);
  private:
  	String reverseString(String inputString);
  	String toBinary(int input1, int length1);
};

#endif

