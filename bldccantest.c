
#include <stdio.h>
#include <string.h>


#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include "bldc.h"


int main(int varc, char* varv[])
{  
  int can_socket;
  const char *ifname = varv[1];
  struct ifreq can_ifr;
  struct sockaddr_can can_addr;
  struct can_frame test_frame;
  int nbytes;

  printf("bldctest \n");
  printf("run with arguments:\n");
  printf("  1. caninterface\n\n");

  printf("Starting application\n");

  // Setup CAN interface:
  can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  strcpy(can_ifr.ifr_name, ifname);
  //can_ifr.ifr_name = "can0";
  if (ioctl(can_socket, SIOCGIFINDEX, &can_ifr) < 0)
  {
    printf("ERROR unable to connect to interface: %s\n", can_ifr.ifr_name);
    return -1;
  }
  
  printf("Connected to socket: %s\n", can_ifr.ifr_name);
  can_addr.can_family = AF_CAN;
  can_addr.can_ifindex = can_ifr.ifr_ifindex;

  if(bind(can_socket, (struct sockaddr *)&can_addr, sizeof(can_addr)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

  bldc_set_erpm(&test_frame, 45, 9000);

  struct can_frame test_frames[10];
  //int nframes = bldc_reboot(test_frames, 2);
  int nframes = bldc_get_values(test_frames, 2);  

  printf("Doing transmit of..: \n");
  int i,j;
  nbytes = 0;
  for (i=0; i<2; i++)
  {
    printf("Frame:");
     printf("%20x  ", test_frames[i].can_id);
     for (j=0; j<test_frames[i].can_dlc; j++) printf("%02x", test_frames[i].data[j]);
     printf("\n");

     nbytes += write(can_socket, &test_frames[i], sizeof(struct can_frame));
  }

  // now read the returned bytes:
  int read_can_port = 1;
  fd_set readSet;
  uint8_t rxbuf[100];
  for (i=0; i<100; i++) rxbuf[i] = 0;
  struct can_frame recv_frame;
  struct timeval can_timeout;
  can_timeout.tv_sec = 1;
  can_timeout.tv_usec = 0;
  while (read_can_port)
  {
    FD_ZERO(&readSet);
    FD_SET(can_socket, &readSet);
    if (select((can_socket + 1), &readSet, NULL, NULL, &can_timeout) >= 0)
    {
      if (!read_can_port) break;

      if (FD_ISSET(can_socket, &readSet))
      {
        nbytes = read(can_socket, &recv_frame, sizeof(struct can_frame));
	if(nbytes)
  	{
          // Test out the new read function:
          int rval = bldc_fill_rxbuf(&recv_frame, 0,  rxbuf, 100);
          printf("completed packet read with value: %i\n", rval);
          //printf(" %2x %2x %2x %2x %2x %2x %2x %2x\n", rxbuf[0], rxbuf[1], rxbuf[2],
          //rxbuf[3], rxbuf[4], rxbuf[5], rxbuf[6], rxbuf[7]);
          //printf(" %2x %2x %2x %2x %2x %2x %2x %2x\n", rxbuf[8], rxbuf[9], rxbuf[10],
          //rxbuf[11], rxbuf[12], rxbuf[13], rxbuf[14], rxbuf[15]); 
        }
      }
    } 
  }

  //nbytes = write(can_socket, &test_frame, sizeof(struct can_frame));

  printf("Wrote %d bytes\n", nbytes);
  return 0;
}
