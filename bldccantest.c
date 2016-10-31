
#include <stdio.h>
#include <string.h>


#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include "bldc.h"


int main(int varc, char* varv[])
{
  printf("bldctest \n");
  printf("run with arguments:\n");
  printf("  1. caninterface\n");

  // Setup CAN interface:
  int can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  struct ifreq can_ifr;
  strcpy(can_ifr.ifr_name, "can0");
  //can_ifr.ifr_name = "can0";
  if (ioctl(can_socket, SIOCGIFINDEX, &can_ifr) < 0)
  {
    printf("ERROR unable to connect to interface: %s\n", can_ifr.ifr_name);
  }

    struct sockaddr_can can_addr;
    can_addr.can_family = AF_CAN;


  struct can_frame test_frame;
  bldc_set_erpm(&test_frame, 3, 9000);

  return 0;
}
