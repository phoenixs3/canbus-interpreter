# canbus-interpreter

A simple library that allows encoding and decoding of canbus signals from messages

## Installation

* Install Flexcan dependency from https://github.com/collin80/FlexCAN_Library
* Download latest release .zip file or click code > download zip
* In Arduino IDE go to Sketch > Include Library, select the .zip file you just downloaded

## Usage

**An example is provided with library (File>Examples)**

### Initialise
Create an instance of the class:
``` c++
CANBUS canbus;
```

### Decoding CAN signals

Call the decode method of the canbus class:  
``` c++
canbus.decode(CAN_message_t msg, int startBit, int bitLength, String byteOrder, String dataType, double Scale, double bias)`
```

For example  
``` c++
canbus.decode(inMsg, 14, 16, "LSB", "SIGNED", 0.5, -2)`
```

returns a variable of data type double with the requsted signal details

### Encoding CAN signals  
  
Call the encode method of the canbus class:  
``` c++
canbus.encode(CAN_message_t msg, double inputData, int startBit, int bitLength, String byteOrder, String dataType, double Scale, double bias)
```

Be sure to create an empty message beforehand with relevant ID and length. The function returns a `CAN_message_t` data type allowing the encodeCAN method to be called multiple times to pack several signals into one message

``` c++
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
outMsg = canbus.encode(outMsg, tempint, 3, 12, "MSB", "SIGNED", 1.5, 0);`
```
