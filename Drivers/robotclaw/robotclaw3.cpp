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

const int WAIT_TIME = 500;


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

int main(void)
{
  int count = 0;

	printf("Opening\n");

	int serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY );

	// Check for errors
	if (serial_port < 0)
	{
    		printf("Error %i from open: %s\n", errno, strerror(errno));
	}

	printf("Set Baud\n");
	struct termios tty;

	int result = tcgetattr(serial_port, &tty);

        if (result)
        {
    	    printf("tcgetattr failed\n");
	    close(serial_port);
	    return -1;
  	}

	tty.c_lflag &= ~ICANON;
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;

        cfsetospeed(&tty, B38400);
        cfsetispeed(&tty, B38400);

	result = tcsetattr(serial_port, TCSANOW, &tty);

        if (result)
        {
          perror("tcsetattr failed");
          close(serial_port);
          return -1;
        }

        result = tcgetattr(serial_port, &tty);
        printf("timeout=%d\n",tty.c_cc[VTIME]);
	printf("ICONON=%d\n",tty.c_lflag);
	printf("Blocks=%d\n",tty.c_cc[VMIN]);


        printf("cflag=%4x\n",tty.c_cflag);
        printf("iflag=%4x\n",tty.c_iflag);
        printf("oflag=%4x\n",tty.c_oflag);
        printf("lflag=%4x\n",tty.c_lflag);

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

/*
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
        printf("\n");
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

*/
	close(serial_port);
}


