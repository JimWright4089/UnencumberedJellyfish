#include "Jims_Serial.h"

//#define SERIAL_DEBUG 1

Jims_Serial::Jims_Serial(){
  openPort();
}

Jims_Serial::Jims_Serial(string portName) :
 mPortName(portName)
{
  openPort();
}

Jims_Serial::Jims_Serial(string portName, uint32_t baud) :
  mPortName(portName),
  mBaud(baud)
{
  openPort();
}

void Jims_Serial::openPort()
{
  if(0 != mPort)
  {
    close(mPort);
  }

  mPort = open(mPortName.c_str(), O_RDWR );
  if (mPort == -1)
  {
    perror(mPortName.c_str());
    return;
  }
 
  // Flush away any bytes previously read or written.
  int result = tcflush(mPort, TCIOFLUSH);
  if (result)
  {
    perror("tcflush failed");  // just a warning, not a fatal error
  }
 
  // Get the current configuration of the serial port.
  struct termios options;
  result = tcgetattr(mPort, &options);
  if (result)
  {
    perror("tcgetattr failed");
    close(mPort);
    return;
  }

  options.c_cflag=CS8|CLOCAL|CREAD;
  options.c_iflag=IGNPAR;
  options.c_oflag=0;
  options.c_lflag=0;
  options.c_cc[VMIN]=0;
  options.c_cc[VTIME]=1;
  options.c_lflag &= ~ICANON;
  options.c_iflag &= ~(IXON | IXOFF | IXANY);

  cfsetispeed(&options, mBaud);
  cfsetospeed(&options, mBaud); 

  result = tcsetattr(mPort, TCSANOW, &options);
  if (result)
  {
    perror("tcsetattr failed");
    close(mPort);
    return;
  }
}

void Jims_Serial::end(void)
{
  close(mPort);
}

bool Jims_Serial::flush()
{
  return(tcflush(mPort, TCIOFLUSH));
}

bool Jims_Serial::readBuffer(uint8_t *buffer, uint16_t len)
{
  size_t received = 0;
  while (received < len)
  {
    ssize_t r = read(mPort, buffer + received, len - received);
    if (r < 0)
    {
      perror("failed to read from port");
      return -1;
    }
    if (r == 0)
    {
      // Timeout
      break;
    }
    received += r;
  }

#ifdef SERIAL_DEBUG
  printf("Read %d %ld:",len,received);
  for(int i=0;i<received;i++)
  {
    printf("%2x ",buffer[i]);
  }
  printf("\n");
#endif 

  return received;
}

bool Jims_Serial::writeBuffer(const uint8_t *buffer, uint16_t len)
{
#ifdef SERIAL_DEBUG
  printf("Write %d:",len);
  for(int i=0;i<len;i++)
  {
    printf("%2x ",buffer[i]);
  }
  printf("\n");
#endif

	size_t sent = write(mPort, buffer, len);
  std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));

  return sent == len;
}

bool Jims_Serial::write_then_read(const uint8_t *write_buffer, uint16_t write_len,
                      uint8_t *read_buffer, uint16_t read_len)
{
  bool results = writeBuffer(write_buffer, write_len);
  std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));
  if(true == results)
  {
    results = readBuffer(read_buffer,read_len);
  }
  return results;
}
