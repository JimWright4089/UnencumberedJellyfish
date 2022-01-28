#ifndef Jims_Serial_h
#define Jims_Serial_h

// C library headers
#include <stdio.h>
#include <string.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <iostream>
#include <sstream>
#include <stdio.h>   	// Standard input/output definitions
#include <string>
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions
#include <system_error>	// For throwing std::system_error
#include <sys/ioctl.h> // Used for TCGETS2, which is required for custom baud rates
#include <cassert>
#include <asm/ioctls.h>
#include <algorithm>
#include <iterator>
#include <chrono>
#include <thread>

using namespace std;

const int WAIT_TIME = 1;

class Jims_Serial {
public:
  Jims_Serial();
  Jims_Serial(string portName);
  Jims_Serial(string portName, uint32_t baud);
  void end(void);

  bool flush();
  bool readBuffer(uint8_t *buffer, uint16_t len);
  bool writeBuffer(const uint8_t *buffer, uint16_t len);
  bool write_then_read(const uint8_t *write_buffer, uint16_t write_len,
                       uint8_t *read_buffer, uint16_t read_len);
  bool setBaud(uint32_t baud);

private:

  void openPort();

  string mPortName = "/dev/ttyACM0";
  int mPort = 0;
  uint32_t mBaud = 115200;
  bool mBegun;
  uint16_t mMaxBufferSize;
  bool _read(uint8_t *buffer, uint16_t len, bool stop);
};
#endif
