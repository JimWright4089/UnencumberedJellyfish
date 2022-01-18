/* Better port reading program
   v1.0
   23-10-96
   
   This test program uses quasi-POSIX compliant UNIX functions to
   open the ABU port and read.
   
   Uses termio functions to initialise the port to 9600 baud, at
   8 data bits, no parity, no hardware flow control,
   and features character buffering.
   The 20ms delay after the port read indicates that characters are
   buffered if a button is pressed many times.
   
   This program was derived from instructions at
    http://www.easysw.com/~mike/serial/
 */

 #include <stdio.h>   /* Standard input/output definitions */
 #include <string.h>  /* String function definitions */
 #include <unistd.h>  /* UNIX standard function definitions */
 #include <fcntl.h>   /* File control definitions */
 #include <errno.h>   /* Error number definitions */
 #include <termios.h> /* POSIX terminal control definitions */


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



 /*
  * 'open_port()' - Open serial port 1.
  *
  * Returns the file descriptor on success or -1 on error.
  */

 int open_port(void)
 {
   int fd;                                   /* File descriptor for the port */

   fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);

   if (fd == -1)
   {                                              /* Could not open the port */
     fprintf(stderr, "open_port: Unable to open /dev/ttyS1 - %s\n",
             strerror(errno));
   }

   return (fd);
 }

int main()
{
 int mainfd=0;                                            /* File descriptor */
 char chout;
 struct termios options;
   
 mainfd = open_port();

 fcntl(mainfd, F_SETFL, FNDELAY);                  /* Configure port reading */
                                     /* Get the current options for the port */
 tcgetattr(mainfd, &options);
 cfsetispeed(&options, B38400);                 /* Set the baud rates to 9600 */
 cfsetospeed(&options, B38400);
    
                                   /* Enable the receiver and set local mode */
 options.c_cflag |= (CLOCAL | CREAD);
 options.c_cflag &= ~PARENB; /* Mask the character size to 8 bits, no parity */
 options.c_cflag &= ~CSTOPB;
 options.c_cflag &= ~CSIZE;
 options.c_cflag |=  CS8;                              /* Select 8 data bits */
 options.c_cflag &= ~CRTSCTS;               /* Disable hardware flow control */  
 
                                 /* Enable data to be processed as raw input */
 options.c_lflag &= ~(ICANON | ECHO | ISIG);
       
                                        /* Set the new options for the port */
 tcsetattr(mainfd, TCSANOW, &options);

/*
  sleep(2); //required to make flush work, for some reason
  tcflush(mainfd,TCIOFLUSH);


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

        write(mainfd, msg, 4);
*/



                         
 while (1)
 {
   read(mainfd, &chout, sizeof(chout));          /* Read character from ABU */
   
   if (chout != 0)
      printf("Got %x.\n", chout);

   chout=0;
   usleep(200);
 }
                                                    /* Close the serial port */
  close(mainfd);
  return 0;
 }
