#include "bldc.h"

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

//------------------------------------------------------------------------------------
void bldc_set_duty(struct can_frame *frame, int id, float duty)
{
  BLDC_PACKET_ID ctl = BLDC_PACKET_SET_DUTY;
  frame->can_id = (id & 0xFF) + ((uint8_t)ctl << 8) + 0x80000000;
  int len = 0;
  bldc_buffer_append_int32(frame->data, duty*100000.0 , &len);
  frame->can_dlc = len;
}

void bldc_set_current(struct can_frame *frame, int id, float current)
{
  BLDC_PACKET_ID ctl = BLDC_PACKET_SET_CURRENT;
  frame->can_id = (id & 0xFF) + ((uint8_t)ctl << 8) + 0x80000000;
  int len = 0;
  bldc_buffer_append_int32(frame->data, current*1000.0 , &len);
  frame->can_dlc = len;
}

void bldc_set_current_brake(struct can_frame *frame, int id, float current)
{
  BLDC_PACKET_ID ctl = BLDC_PACKET_SET_CURRENT_BRAKE;
  frame->can_id = (id & 0xFF) + ((uint8_t)ctl << 8) + 0x80000000;
  int len = 0;
  bldc_buffer_append_int32(frame->data, current*1000.0 , &len);
  frame->can_dlc = len;
}

void bldc_set_erpm(struct can_frame *frame, int id, int32_t erpm)
{
  BLDC_PACKET_ID ctl = BLDC_PACKET_SET_RPM;
  frame->can_id = (id & 0xFF) + ((uint8_t)ctl << 8) + 0x80000000;
  int len = 0;
  bldc_buffer_append_int32(frame->data, erpm , &len);
  frame->can_dlc = len;
}

//-------------------------------------------------------------------------------------

void bldc_get_status(struct can_frame *frame, bldc_status *status)
{
  int ind = 0;
  status->id = frame->can_id;
  status->erpm = (float)bldc_buffer_get_int32(frame->data, &ind);
  status->current_motor = (float)bldc_buffer_get_int16(frame->data, &ind) / 100.0;
  status->duty_now = (float)bldc_buffer_get_int16(frame->data, &ind) / 1000.0;
}

void bldc_get_status2(struct can_frame *frame, bldc_status2 *status)
{
  int ind = 0;
  status->id = frame->can_id;
  status->current_input = (float)bldc_buffer_get_int16(frame->data, &ind) /100.0;
  status->voltage_input = (float)bldc_buffer_get_int16(frame->data, &ind) /1000.0;
  status->uptime = bldc_buffer_get_uint32(frame->data, &ind);
}

void bldc_get_status3(struct can_frame *frame, bldc_status3 *status)
{
  int ind = 0;
  status->id = frame->can_id;
  status->temperature_mos1 = (float)bldc_buffer_get_uint16(frame->data, &ind) /100.0;;
  status->temperature_motor = (float)bldc_buffer_get_uint16(frame->data, &ind) /100.0;
  status->fault_code = frame->data[ind++];

}

