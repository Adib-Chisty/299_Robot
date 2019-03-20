/*
  QSerial.cpp - 300 baud IR serial library
  Adapted from SoftwareSerial by Stan Simmons. 
  Variables that are internal to the library have
  names that start with an underscore (a legal character)
  to make them stand out to the user.
*/

/*
*****************************************************************************
* Includes
*****************************************************************************
*/

#include "Arduino.h"
#include "QSerial.h"

/*
*****************************************************************************
* Constructor
*****************************************************************************
*/

QSerial::QSerial()
{
  _receivePin = -1; //not attached yet
  _transmitPin = -1;//not attached yet	
}

/*
*****************************************************************************
* User API
*****************************************************************************
*/

void QSerial::attach(int receivePin, int transmitPin)
{
  _receivePin = receivePin;
  _transmitPin = transmitPin;
  _baudRate = 300;
  _bitPeriod = 1000000 / _baudRate;// in microseconds

  digitalWrite(_transmitPin, LOW); //will hold IR transitter's 555 in reset state (IDLE)
}

void QSerial::detach()
{
  _receivePin = -1; //not attached
  _transmitPin = -1; //not atatched
}


int QSerial::receive(int timeout_msecs)
{ //returns 0 if no byte, or if not attached
  //returns received byte value unless an error(then returns -1 or -2)
  int val = 0;
  long totwait_usecs=0;
  int offset;
  int bitDelay = _bitPeriod - clockCyclesToMicroseconds(50); // each loop is about 50 cycles
  // one byte of serial data (LSB first)
  // ...--\    /--\/--\/--\/--\/--\/--\/--\/--\/--...
  //	 \--/\--/\--/\--/\--/\--/\--/\--/\--/
  //	start  0   1   2   3   4   5   6   7 stop

    if( _receivePin== -1) //not attached?
	return 0;
    //loop waiting for start bit (subject to timeout if no start bit seen)
    while (digitalRead(_receivePin)==HIGH && totwait_usecs < 1000*timeout_msecs )
    {
	//wait in chunks of 1/30'th of a bit duration 
	totwait_usecs += _bitPeriod/30; 
	delayMicroseconds(_bitPeriod/30);
    }
    
    if(digitalRead(_receivePin) == HIGH) //if still IDLE
	return 0; //nothing received, so use return 0 value
    //ELSE keep going to verify start bit and receive rest of bits

    //FIRST, confirm that this is a real start bit
    // byte start indicated by a falling edge and low start bit
    // jump to the middle of the low start bit
    delayMicroseconds(_bitPeriod/2);
    if(digitalRead(_receivePin) != LOW) //if not still LOW
	 return -1; //must have been a false start, so return -1 value

    // offset of the bit in the byte: from 0 (LSB) to 7 (MSB)
    for (offset = 0; offset < 8; offset++) 
    {
	// jump to middle of next bit
	delayMicroseconds(bitDelay);
	// read bit and "OR" it into the result at the proper bit offset
	val = val | digitalRead(_receivePin) << offset;
    }
    delayMicroseconds(_bitPeriod); //wait for stop bit
    if(digitalRead(_receivePin) != HIGH) //missing expected HIGH for stop bit?
	 return -2; //framing error, so return -2 value
    else
	return val; //return byte value
}

void QSerial::transmit(byte b)
{  //uses inverted levels so it can directly drive a 555's active-low reset pin
  
  int bitDelay;
  byte mask;

   if(_transmitPin== -1) //not attached?
	return; //do nothing

  digitalWrite(_transmitPin, HIGH); //turn ON 555 for start bit transmission (receiver will go LOW)
  delayMicroseconds(_bitPeriod);

  bitDelay = _bitPeriod - clockCyclesToMicroseconds(50); // each loop is about 50 cycles
  for (mask = 0x01; mask; mask <<= 1) 
  {
    if (b & mask) // get the next bit
      digitalWrite(_transmitPin,LOW); // a bit '1' is transmitted as absence of IR (555 is reset)
    else
      digitalWrite(_transmitPin,HIGH); //  a bit '0' is transmitted as presence of IR (555 is enabled)
    delayMicroseconds(bitDelay);
  }
  digitalWrite(_transmitPin, LOW); //tack on the "STOP" bit (receiver will go HIGH)
  delayMicroseconds(_bitPeriod);
}


