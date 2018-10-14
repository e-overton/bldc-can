
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

  //bldc_set_erpm(&test_frame, 45, 9000);
  float servo_ms, servo_ms_last;

  bldc_get_decoded_ppm(can_socket, 6, &servo_ms, &servo_ms_last, NULL);
  printf("BLDC ppm: %f, %f\n", servo_ms, servo_ms_last); 

  return 0;
}
