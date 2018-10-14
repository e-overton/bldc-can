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
  fprintf(stderr, "\nUsage: %s [options] <CAN interface> <Output File> <Controller ID> \n", prg);
}


int main(int argc, char** argv){

  int opt;
  char *canifname, *logfilename;
  int verbose = 1;

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

  // Motor status:
  int bldc_id = -1;
  bldc_status mot_status = {0};
  
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

  if (optind+3 > argc) {
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
  mot_status.id = bldc_id;

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

  int packets = 0;
  while (read_can_port)
  {
    if (packets > 5000) break;
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
			  // Attempt to read data from the can frame, rely on function to tell 
			  // us if it was a valid frame.
			  int rval = bldc_get_status(&recv_frame, &mot_status);
			  packets++;
			  // Echo infomation to the terminal, if in verbose mode.
			  if (verbose > 0)
			  {
				if (rval == 1)
				{
				  printf ("got bldc frame1: %4f eprm, %.2f%% \n", mot_status.erpm,
						  mot_status.duty_now);
				}
				else if (rval == 2)
				{
				  printf ("got bldc frame2: %2.1fV, %2.1fA \n", mot_status.voltage_input,
						  mot_status.current_input );
				}
				else if (rval == 3)
				{
				  printf ("got bldc frame3: %2.1fC, %2.1fC \n", mot_status.temperature_motor,
						  mot_status.temperature_mos1 );
				}
			  }

			  // Write the new data (if new data was retrived)
				if (rval > 0)
			  {
				//Update time keeping:
				clock_gettime(CLOCK_MONOTONIC, &tnow);
				time_diff = ((double)tnow.tv_sec + 1.0e-9*tnow.tv_nsec) - 
							  ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec);

				fprintf(logfile, "%7.3f %2i %.2f %4f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
					time_diff,
					mot_status.id, mot_status.duty_now, mot_status.erpm,
					mot_status.current_motor, mot_status.voltage_input,
					mot_status.current_input, mot_status.temperature_motor,
					mot_status.temperature_mos1);

				if (verbose > 0)
				printf("%7.3f %2i %.2f %4f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
				   time_diff,
				   mot_status.id, mot_status.duty_now, mot_status.erpm,
				   mot_status.current_motor, mot_status.voltage_input,
				   mot_status.current_input, mot_status.temperature_motor,
				   mot_status.temperature_mos1);
				fflush(logfile);
				}
			
			
		    }
		}
		  else
		{
		  usleep(100);
		}
    }
  }

  fclose(logfile);
  return 0;
}
