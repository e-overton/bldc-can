#include "bldc.h"
#include <stdio.h>
#include <string.h>

void bldc_buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index)
{
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void bldc_buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
  bldc_buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void bldc_buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
  bldc_buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

int16_t bldc_buffer_get_int16(const uint8_t *buffer, int32_t *index) {
  int16_t res =   ((uint16_t) buffer[*index]) << 8 |
                  ((uint16_t) buffer[*index + 1]);
  *index += 2;
  return res;
}

uint16_t bldc_buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
  uint16_t res =  ((uint16_t) buffer[*index]) << 8 |
                  ((uint16_t) buffer[*index + 1]);
  *index += 2;
  return res;
}

int32_t bldc_buffer_get_int32(const uint8_t *buffer, int32_t *index) {
  int32_t res =   ((uint32_t) buffer[*index]) << 24 |
                  ((uint32_t) buffer[*index + 1]) << 16 |
                  ((uint32_t) buffer[*index + 2]) << 8 |
                  ((uint32_t) buffer[*index + 3]);
  *index += 4;
  return res;
}

uint32_t bldc_buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
  uint32_t res =  ((uint32_t) buffer[*index]) << 24 |
                  ((uint32_t) buffer[*index + 1]) << 16 |
                  ((uint32_t) buffer[*index + 2]) << 8 |
                  ((uint32_t) buffer[*index + 3]);
  *index += 4;
  return res;
}

float bldc_buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
  return (float)bldc_buffer_get_int16(buffer, index) / scale;
}

float bldc_buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
  return (float)bldc_buffer_get_int32(buffer, index) / scale;
}

canid_t bldc_gen_can_id(BLDC_PACKET_ID pack_id, uint8_t vesc_can_id)
{
  uint32_t id = vesc_can_id;     // LSB byte is the controller id.
  id |= (uint32_t)pack_id << 8;  // Next lowest byte is the packet id.
  id |= 0x80000000;              // Send in Extended Frame Format.
  return id;
}

//------------------------------------------------------------------------------------
void bldc_set_duty(struct can_frame *frame, int id, float duty)
{
  int len = 0;
  frame->can_id = bldc_gen_can_id(BLDC_PACKET_SET_DUTY, id);
  bldc_buffer_append_int32(frame->data, duty*100000.0 , &len);
  frame->can_dlc = len;
}

void bldc_set_current(struct can_frame *frame, int id, float current)
{
  int len = 0;
  frame->can_id = bldc_gen_can_id(BLDC_PACKET_SET_CURRENT, id);
  bldc_buffer_append_int32(frame->data, current*1000.0 , &len);
  frame->can_dlc = len;
}

void bldc_set_current_brake(struct can_frame *frame, int id, float current)
{
  int len = 0;
  frame->can_id = bldc_gen_can_id(BLDC_PACKET_SET_CURRENT_BRAKE, id);
  bldc_buffer_append_int32(frame->data, current*1000.0 , &len);
  frame->can_dlc = len;
}

void bldc_set_erpm(struct can_frame *frame, int id, int32_t erpm)
{
  int len = 0;
  frame->can_id = bldc_gen_can_id(BLDC_PACKET_SET_RPM, id);
  bldc_buffer_append_int32(frame->data, erpm , &len);
  frame->can_dlc = len;
}

//-------------------------------------------------------------------------------------

int bldc_get_status(const struct can_frame *frame, bldc_status *status)
{
  int ind = 0;
  uint8_t recv_id = frame->can_id;
  uint8_t pack_id = frame->can_id >> 8;  

  // Verify the frame id and can id:
  if ( (status->id >0) && (recv_id != status->id))
    return -1;

  //printf ("can id:%i, frame id:%i\n", recv_id, pack_id);

  // Read the selected data:
  switch (pack_id)
  {
    case BLDC_PACKET_STATUS:
      status->erpm = (float)bldc_buffer_get_int32(frame->data, &ind);
      status->current_motor = (float)bldc_buffer_get_int16(frame->data, &ind) / 100.0;
      status->duty_now = (float)bldc_buffer_get_int16(frame->data, &ind) / 1000.0;
      return 1;
    
    case BLDC_PACKET_STATUS2:
      status->current_input = (float)bldc_buffer_get_int16(frame->data, &ind) /100.0;
      status->voltage_input = (float)bldc_buffer_get_int16(frame->data, &ind) /1000.0;
      status->uptime = bldc_buffer_get_uint32(frame->data, &ind);
      return 2;

    case BLDC_PACKET_STATUS3:
      status->temperature_mos1 = (float)bldc_buffer_get_uint16(frame->data, &ind) /100.0;;
      status->temperature_motor = (float)bldc_buffer_get_uint16(frame->data, &ind) /100.0;
      status->fault_code = frame->data[ind++];
      return 3;

    default:
      return 0;
  }
}


//-------------------------------------------------------------------------------------

uint8_t bldc_reboot(struct can_frame frames[], int id)
{
   // The reboot command only has one identifier, the command itsef.
   // Issuing this command will cause the VESC to lockup and the
   // watchdog to reboot it (hopefully).
   uint8_t tx_buffer[1];
   tx_buffer[0] = COMM_REBOOT;

   bldc_gen_proc_cmd(frames, id, tx_buffer, 1);

   int i,j;
   for (i=0; i<2; i++)
   {
      printf("%20x  ", frames[i].can_id);
      for (j=0; j<frames[i].can_dlc; j++) printf("%02x", frames[i].data[j]);
      printf("\n");
   }

   return 2;
};

uint8_t bldc_get_values(struct can_frame frames[], int id)
{
   uint8_t tx_buffer[1];
   tx_buffer[0] = COMM_GET_VALUES;

   bldc_gen_proc_cmd(frames, id, tx_buffer, 1);

   int i,j;
   for (i=0; i<2; i++)
   {
      printf("%20x  ", frames[i].can_id);
      for (j=0; j<frames[i].can_dlc; j++) printf("%02x", frames[i].data[j]);
      printf("\n");
   }

   return 2;
}

void bldc_gen_proc_cmd(struct can_frame frames[], int id, const uint8_t tx_buffer[], uint8_t tx_len)
{
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t frame_cnt = 0;

  // Generate a CRC for the encoded data:
  uint16_t crc = bldc_crc16(tx_buffer, tx_len);

  // Generate the frames to transderr tx_buffer to the vesc rx_buffer.
  for (i=0; i<tx_len; i++)
  {
    if (j==8)
    {
       frame_cnt+=1;
       j=0;
    }

    if (j==0)
    {
       frames[frame_cnt].can_id = bldc_gen_can_id(BLDC_PACKET_FILL_RX_BUFFER, id);
       frames[frame_cnt].can_dlc = 1;
       frames[frame_cnt].data[0] = frame_cnt;
       j++;
    }

    frames[frame_cnt].data[j] = tx_buffer[i];
    frames[frame_cnt].can_dlc++;
    j++;
  }

  frame_cnt++;

  // Second, generate the process command:
  frames[frame_cnt].can_id = bldc_gen_can_id(BLDC_PACKET_PROCESS_RX_BUFFER, id);
  frames[frame_cnt].data[0] = 0x0; // rx_buffer_last_id - the address to send the command to
  frames[frame_cnt].data[1] = 0x0; // set to true if the command is to be relayed.
  frames[frame_cnt].data[2] = 0xFF & (tx_len>>8); // upper byte of txlen
  frames[frame_cnt].data[3] = 0xFF & tx_len;   // lower byte of txlen
  frames[frame_cnt].data[4] = 0xFF & (crc>>8); // crc high
  frames[frame_cnt].data[5] = 0xFF & crc;   // crc low
  frames[frame_cnt].can_dlc = 6;
}

int bldc_fill_rxbuf(const struct can_frame *frame, int id,  uint8_t rx_buffer[], uint16_t maxlen)
{
  uint16_t index;
  uint16_t rx_len;
  uint16_t controller_id;
  uint16_t crc;
  int rval;

  // Read a fill frame
  if (frame->can_id == bldc_gen_can_id(BLDC_PACKET_FILL_RX_BUFFER, id))
  {
      index = (uint8_t)frame->data[0];
      if (index + frame->can_dlc - 1 < maxlen)
      {
        memcpy(rx_buffer+index, frame->data+1, frame->can_dlc -1);
      }
    rval = 0;
  }

  // Read a fill frame, for index > 255.
  if (frame->can_id == bldc_gen_can_id(BLDC_PACKET_FILL_RX_BUFFER_LONG, id))
  {
      index = (uint16_t)frame->data[0] << 8 | (uint16_t) frame->data[1];
      if (index + frame->can_dlc - 1 < maxlen)
      {
        memcpy(rx_buffer+index, frame->data+2, frame->can_dlc -2);
      }
    rval = 0;
  }

  // Read the final frame:
  if (frame->can_id == bldc_gen_can_id(BLDC_PACKET_PROCESS_RX_BUFFER, id))
  {
     controller_id = frame->data[0];
     rx_len = ((uint16_t) frame->data[2] << 8) | (uint16_t) frame->data[3];
     crc = (uint16_t) frame->data[4] << 8 | (uint16_t) frame->data[5];
     

     if (crc == bldc_crc16(rx_buffer, rx_len))
     {
        rval = rx_len;
     }
     else
     {
       rval = -rx_len;
     }
  }



  return rval;
}


// Table for CRC16 generation.
const uint16_t crc16_tab[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
		0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
		0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
		0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
		0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
		0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
		0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
		0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
		0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
		0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
		0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
		0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
		0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
		0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
		0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
		0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
		0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
		0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
		0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
		0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
		0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
		0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
		0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
		0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
		0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };

// Function to generate CRC:
uint16_t bldc_crc16(const uint8_t* buf, unsigned int len)
{
	unsigned int i;
	unsigned short cksum = 0;
	for (i = 0; i < len; i++) {
		cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
	}
	return cksum;
}

