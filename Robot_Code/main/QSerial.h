//QSerial - 300 baud IR serial library
//Adapted from SoftwareSerial by Stan Simmons. 


//next bit are directives to make sure the compiler only processes
//this .h file once  even if there are multiple #include's
#ifndef QSerial_h
#define QSerial_h

typedef unsigned char byte;

class QSerial
{
  private:
    int _receivePin;
    int _transmitPin;
    int _baudRate; //in bits per second
    int _bitPeriod; //in usecs
  public:
    QSerial(); //minimalist constructor
    void attach(int rxpin, int txpin); //associates the pins
    void detach(); //disassociates the pins
    int receive(int timeout_msecs); //receives IR-detected output on rxpin (subject to timeout)
    void transmit(byte bval); //transmits immediately the bits of bval on txpin (inverted)
};

#endif

