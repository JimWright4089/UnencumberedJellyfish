// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <iostream>
#include <sstream>
#include <stdio.h>   	// Standard input/output definitions
#include <string.h>  	// String function definitions
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions
// #include <termios.h> 	// POSIX terminal control definitions (struct termios)
#include <system_error>	// For throwing std::system_error
#include <sys/ioctl.h> // Used for TCGETS2, which is required for custom baud rates
#include <cassert>
// #include <asm/termios.h> // Terminal control definitions (struct termios)
#include <asm/ioctls.h>
#include <algorithm>
#include <iterator>

#include <chrono>
#include <thread>

const int WAIT_TIME = 1;


unsigned int crc16(unsigned char *packet, int nBytes)
{
  unsigned int crc = 0;
  for (int byte = 0; byte < nBytes; byte++)
  {
    crc = crc ^ ((unsigned int)packet[byte] << 8);
    for (unsigned char bit = 0; bit < 8; bit++)
    {
      if (crc & 0x8000)
      {
        crc = (crc << 1) ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

int open_serial_port(const char * device, uint32_t baud_rate)
{
  int fd = open(device, O_RDWR );
  if (fd == -1)
  {
    perror(device);
    return -1;
  }
 
  // Flush away any bytes previously read or written.
  int result = tcflush(fd, TCIOFLUSH);
  if (result)
  {
    perror("tcflush failed");  // just a warning, not a fatal error
  }
 
  // Get the current configuration of the serial port.
  struct termios options;
  result = tcgetattr(fd, &options);
  if (result)
  {
    perror("tcgetattr failed");
    close(fd);
    return -1;
  }
 
  // Turn off any options that might interfere with our ability to send and
  // receive raw binary bytes.
  //options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  //options.c_oflag &= ~(ONLCR | OCRNL);
  //options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

/*
    serial->tio.c_cflag=baudrate|databits|checkparity|stopbits|CLOCAL|CREAD;
    serial->tio.c_iflag=IGNPAR;
    serial->tio.c_oflag=0;
    serial->tio.c_lflag=0;
    serial->tio.c_cc[VMIN]=28;
    serial->tio.c_cc[VTIME]=6;
*/

  printf("cflag=%4x\n",options.c_cflag);
  printf("iflag=%4x\n",options.c_iflag);
  printf("oflag=%4x\n",options.c_oflag);
  printf("lflag=%4x\n",options.c_lflag);

  printf("cflag=%4x\n",options.c_cflag);
  printf("iflag=%4x\n",options.c_iflag);
  printf("oflag=%4x\n",options.c_oflag);
  printf("lflag=%4x\n",options.c_lflag);
//  printf("cc   =%4x\n",options.c_cc);

  options.c_cflag=CS8|CLOCAL|CREAD;
  options.c_iflag=IGNPAR;
  options.c_oflag=0;
  options.c_lflag=0;
  options.c_cc[VMIN]=0;
  options.c_cc[VTIME]=1;
  options.c_lflag &= ~ICANON;
  options.c_iflag &= ~(IXON | IXOFF | IXANY);

 
  // Set up timeouts: CCLOCALalls to read() will return as soon as there is
  // at least one byte available or when 100 ms has passed.
//  options.c_cc[VTIME] = 1;
//  options.c_cc[VMIN] = 1;
 
  // This code only supports certain standard baud rates. Supporting
  // non-standard baud rates should be possible but takes more work.
/*
  switch (baud_rate)
  {
  case 4800:   cfsetospeed(&options, B4800);   break;
  case 9600:   cfsetospeed(&options, B9600);   break;
  case 19200:  cfsetospeed(&options, B19200);  break;
  case 38400:  cfsetospeed(&options, B38400);  break;
  case 115200: cfsetospeed(&options, B115200); break;
  default:
    fprintf(stderr, "warning: baud rate %u is not supported, using 9600.\n",
      baud_rate);
    cfsetospeed(&options, B9600);
    break;
  }
*/

  cfsetispeed(&options, B38400);
  cfsetospeed(&options, B38400); 

  result = tcsetattr(fd, TCSANOW, &options);
  if (result)
  {
    perror("tcsetattr failed");
    close(fd);
    return -1;
  }
 
  return fd;
}



void set_blocking (int fd, int should_block) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                perror ("error from tggetattr");
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                perror ("error setting term attributes");
}

ssize_t read_port(int fd, uint8_t * buffer, size_t size)
{
  size_t received = 0;
  while (received < size)
  {
    ssize_t r = read(fd, buffer + received, size - received);
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
  return received;
}




int main(void)
{
  int count = 0;

	printf("Opening\n");

        int serial_port = open_serial_port("/dev/ttyACM0",38400);


/*
	int serial_port = open("/dev/ttyS0", O_RDWR | O_NOCTTY );

	// Check for errors
	if (serial_port < 0) 
	{
    		printf("Error %i from open: %s\n", errno, strerror(errno));
	}

        tcflush(serial_port, TCIOFLUSH);

	printf("Set Baud\n");
*/
	struct termios tty;
/*
	int result = tcgetattr(serial_port, &tty);

        if (result)
        {
    	    printf("tcgetattr failed\n");
	    close(serial_port);
	    return -1;
  	}


        tty.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
        tty.c_oflag &= ~(ONLCR | OCRNL);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	tty.c_lflag &= ~ICANON;
        tty.c_cc[VTIME] = 1;
        tty.c_cc[VMIN] = 0;

        cfsetospeed(&tty, B38400);
        cfsetispeed(&tty, cfgetospeed(&tty));

	result = tcsetattr(serial_port, TCSANOW, &tty);

        if (result)
        {
          perror("tcsetattr failed");
          close(serial_port);
          return -1;
        }

*/
        int result = tcgetattr(serial_port, &tty);
        printf("timeout=%d\n",tty.c_cc[VTIME]);
	printf("ICONON=%d\n",tty.c_lflag);
	printf("Blocks=%d\n",tty.c_cc[VMIN]);


  printf("cflag=%4x\n",tty.c_cflag);
  printf("iflag=%4x\n",tty.c_iflag);
  printf("oflag=%4x\n",tty.c_oflag);
  printf("lflag=%4x\n",tty.c_lflag);

        tcflush(serial_port, TCIOFLUSH);

	printf("Write\n");

	unsigned char msg[] = { 128, 0, 45, 'l', 'o' };

        unsigned int crc = crc16(msg, 3);
        msg[3] = (crc>>8)&0xff;
        msg[4] = (crc)&0xff;

        for(int i=0;i<sizeof(msg);i++)
        {
          printf("%2x ",msg[i]);
        }
        printf("\n");

        tcflush(serial_port, TCIOFLUSH);
	write(serial_port, msg, sizeof(msg));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));

	printf("Read\n");

	uint8_t read_buf [256];

//        int n = read_port(serial_port, read_buf, 1);


        int n = read(serial_port, &read_buf, 1);

        count = 0;
        while((0 == n)&&(count < 3))
	{
          printf("loop\n");
//	  set_blocking(serial_port,false);
//          tcflush(serial_port, TCIOFLUSH);
//          write(serial_port, msg, sizeof(msg));
          std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));
          n = read(serial_port, &read_buf, 1);
          count++;
//          std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));
        }


	printf("Read=%d\n",n);
	for(int i=0;i<n;i++)
	{
		printf("%x ",read_buf[i]);
	}
	printf("\n");


	for(int j=0;j<10;j++)
        {

        printf("Write\n");

        unsigned char msg[] = { 128, 16, 45, 'l', 'o' };
        unsigned int crc = crc16(msg, 2);
        msg[2] = (crc>>8)&0xff;
        msg[3] = (crc)&0xff;

        for(int i=0;i<4;i++)
        {
          printf("%2x ",msg[i]);
        }
        printf("\n");

        write(serial_port, msg, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));

        printf("Read\n");
        char read_buf [256];

        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME
        //int n = read(serial_port, &read_buf, sizeof(read_buf));
        int n = read(serial_port, &read_buf, 7);
        count = 0;
        while((0 == n)&&(count < 10))
        {
          n = read(serial_port, &read_buf, 7);
          count++;
        }
        printf("Read=%d\n",n);
        for(int i=0;i<n;i++)
        {
                printf("%x ",read_buf[i]);
        }


	long encValue = 0;
	encValue = (read_buf[0]<<24)+(read_buf[1]<<16)+(read_buf[2]<<8)+read_buf[3];


        printf("%ld\n",encValue);
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));
        }


        printf("Write\n");

        unsigned char msg2[] = { 128, 0, 0, 'l', 'o' };

        crc = crc16(msg2, 3);
        msg2[3] = (crc>>8)&0xff;
        msg2[4] = (crc)&0xff;

        for(int i=0;i<sizeof(msg2);i++)
        {
          printf("%2x ",msg2[i]);
        }
        printf("\n");

        write(serial_port, msg2, sizeof(msg2));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));

        printf("Read\n");

        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME
        //int n = read(serial_port, &read_buf, sizeof(read_buf));
        n = read(serial_port, &read_buf, 1);

        printf("Read=%d\n",n);
        for(int i=0;i<n;i++)
        {
                printf("%x ",read_buf[i]);
        }
        printf("\n");


	close(serial_port);
}


