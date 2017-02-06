/*
 * bldclog.c
 *
 * This program reads the data from a bldc (vesc) speed controller
 * using the linux SocketCAN interface, and records the infomation
 * to a file.
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <libgen.h>
#include <string.h>
#include <time.h>

#include <sys/select.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

//From this library:
#include <bldc.h>



void print_usage(char *prg)
{
  fprintf(stderr, "\nUsage: %s [options] <CAN interface> <Output File> \n", prg);
}


int main(int argc, char** argv){

  int opt;
  char *canifname, *logfilename;

  // CAN stuff
  struct ifreq can_ifr;
  struct sockaddr_can can_addr;
  struct can_frame recv_frame;
  struct timeval can_timeout;
  fd_set readSet;
  int can_socket;
  int nbytes;
  int read_can_port = 1;


  //File
  FILE * logfile;
  uint8_t update = 0;

  // Motor status:
  int bldc_id = -1;
  bldc_status mot_status_1 = {0};
  bldc_status2 mot_status_2 = {0};
  bldc_status3 mot_status_3 = {0};
  
  // Time keeping:
  struct timespec tstart={0,0}, tnow={0,0};
  clock_gettime(CLOCK_MONOTONIC, &tstart);
  double time_diff;

  printf("bldclog\n");

  // Argument parsing and checking:
  while ( (opt = getopt(argc, argv, "?")) != -1) {
    switch (opt) {

    case 'h':
    default:
      print_usage(basename(argv[0]));
      exit(1);
      break;

    }
  }

  if (optind+2 > argc) {
    print_usage(basename(argv[0]));
    exit(0);
  }

  // Read remaining arguments into can/file names:
  canifname = argv[optind++];
  logfilename = argv[optind++];
  bldc_id = atoi(argv[optind++]);

  printf("INFO using interface: %s\n", canifname);
  printf("INFO logging to: %s\n", logfilename);
  printf("Recording BLDC: %i", bldc_id);

  // Open file to log to:
  logfile = fopen(logfilename, "w");
  fprintf(logfile, "time id duty erpm i_mot v_bat i_bat t_mot t_fet\n");

  // Setup & bind the socket:
  can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  strcpy(can_ifr.ifr_name, canifname);
  if (ioctl(can_socket, SIOCGIFINDEX, &can_ifr) < 0)
  {
    printf("ERROR unable to connect to interface: %s\n", can_ifr.ifr_name);
    return -1;
  }

  can_timeout.tv_sec = 1;
  can_timeout.tv_usec = 0;

  can_addr.can_family = AF_CAN;
  can_addr.can_ifindex = can_ifr.ifr_ifindex;

  fcntl(can_socket, F_SETFL, O_NONBLOCK);
  bind(can_socket, (struct sockaddr *)&can_addr, sizeof(can_addr));

  // Recieve
  //nbytes = read(can_socket, &recv_frame, sizeof(struct can_frame));
  update = 0;
  while (read_can_port)
  {
    FD_ZERO(&readSet);
    FD_SET(can_socket, &readSet);
    if (select((can_socket + 1), &readSet, NULL, NULL, &can_timeout) >= 0)
    {
      if (!read_can_port)
      {
	break;
      }
      if (FD_ISSET(can_socket, &readSet))
	{
	  nbytes = read(can_socket, &recv_frame, sizeof(struct can_frame));
	  if(nbytes)
  	  {
            int got_data = 0;
	    //printf("dlc = %d, data = %s\n", recv_frame.can_dlc,recv_frame.data);
	    // Get data..
	    if ((recv_frame.can_id & 0xFF00) == (BLDC_PACKET_STATUS << 8))
	    {
	      bldc_status temp_status = {0};
	      bldc_get_status(&recv_frame, &temp_status);
              if ((bldc_id < 0) || (bldc_id == temp_status.id))
              {
                got_data = 1;
                mot_status_1 = temp_status;
	        update |= 0x1;
	        printf ("got bldc frame1: %4f eprm, %.2f%% \n", mot_status_1.erpm,
	        mot_status_1.duty_now);
              }
	    }

	    if ((recv_frame.can_id & 0xFF00) == (BLDC_PACKET_STATUS2 << 8))
	    {
              bldc_status2 temp_status = {0};
	      bldc_get_status2(&recv_frame, &temp_status);
              if ((bldc_id < 0) || (bldc_id == temp_status.id))
              {
                got_data = 1;
                mot_status_2 = temp_status; 
	        update |= 0x01<<1;
	        printf ("got bldc frame2: %2.1fV, %2.1fA \n", mot_status_2.voltage_input,
		        mot_status_2.current_input );
              }
	    }

	    if ((recv_frame.can_id & 0xFF00) == (BLDC_PACKET_STATUS3 << 8))
	    {
	      bldc_status3 temp_status = {0};
              bldc_get_status3(&recv_frame, &temp_status);
              if ((bldc_id < 0) || (bldc_id == temp_status.id))
              { 
                got_data = 1;
                mot_status_3 = temp_status;
                update |= 0x1<<2;
	        printf ("got bldc frame3: %2.1fC, %2.1fC \n", mot_status_3.temperature_motor,
		mot_status_3.temperature_mos1 );
	        printf ("%i\n", update);
              }
	    }
	    
	    //if (update == 7)
	    if (got_data)
            {
	      //Update time keeping:
	      clock_gettime(CLOCK_MONOTONIC, &tnow);
	      time_diff = ((double)tnow.tv_sec + 1.0e-9*tnow.tv_nsec) - 
           		  ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec);
	
	      //fprintf(logfile, "time id duty erpm i_mot v_bat i_bat t_mot t_fet\n");
	      fprintf(logfile, "%7.3f %2i %.2f %4f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
		      time_diff,
		      mot_status_1.id, mot_status_1.duty_now, mot_status_1.erpm,
		      mot_status_1.current_motor, mot_status_2.voltage_input,
		      mot_status_2.current_input, mot_status_3.temperature_motor,
		      mot_status_3.temperature_mos1);
	      printf("%7.3f %2i %.2f %4f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
		     time_diff,
		     mot_status_1.id, mot_status_1.duty_now, mot_status_1.erpm,
		     mot_status_1.current_motor, mot_status_2.voltage_input,
		     mot_status_2.current_input, mot_status_3.temperature_motor,
		     mot_status_3.temperature_mos1);
	      fflush(logfile);
	      update = 0;
	    }
	    
	    
	  }
	}
      else
	{
	  //printf("nodata\n");
	}
    }
  }

  fclose(logfile);
  return 0;
}
