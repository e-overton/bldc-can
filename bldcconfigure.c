#include <stdio.h>
#include <string.h>

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include "bldc.h"

int main(int argc, char** argv){

  int can_socket;
  const char *ifname = argv[1];
  struct ifreq can_ifr;
  struct sockaddr_can can_addr;
  struct can_frame test_frame;
  int nbytes;
  struct timeval cmd_timeout;
  
  cmd_timeout.tv_sec = 5;
  cmd_timeout.tv_usec = 0;

  printf("bldcconfigure \n");
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

  // Now try to find the firmware version
  uint8_t major, minor;
  
  bldc_get_firmware(can_socket, 3, &major, &minor, &cmd_timeout);

  printf("Read firmware from VESC: %i.%i\n", major, minor);

  bldc_mc_configuration mc;
  bldc_get_mc(can_socket, 3, &mc, &cmd_timeout);
  printf ("Current max: %f\n", mc.l_current_max);
  mc.l_current_max = 80;  

  bldc_set_mc(can_socket, 2, &mc, &cmd_timeout);
}
