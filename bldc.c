#include "bldc.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/select.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>

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

void bldc_reset(int can_socket, int id)
{
   struct timeval timeout;
   uint8_t tx_buffer[1];
   tx_buffer[0] = COMM_REBOOT;
   bldc_comm_buffer(can_socket, id, tx_buffer, 1, NULL, 0, &timeout);
}

uint8_t bldc_get_firmware(int can_socket, int id, uint8_t * major, uint8_t * minor, struct timeval *timeout)
{
  uint8_t tx_buffer[1];
  uint8_t rx_buffer[20];
  tx_buffer[0] = COMM_FW_VERSION;

  int val = bldc_comm_buffer(can_socket, id, tx_buffer, 1, rx_buffer, 20, timeout);
  if (val > 0)
  {
    *major = rx_buffer[1];
    *minor = rx_buffer[2];
    //printf("Read FW Version %i.%i", *major, *minor);
  }

}

uint8_t bldc_get_mc(int can_socket, int id, bldc_mc_configuration * mcconf, struct timeval *timeout)
{
  uint8_t tx_buffer[1];
  uint8_t rx_buffer[BLDC_RX_BUF_LEN];
  tx_buffer[0] = COMM_GET_MCCONF;
  int ind = 1;

  int val = bldc_comm_buffer(can_socket, id, tx_buffer, 1, rx_buffer, BLDC_RX_BUF_LEN, timeout);
  if (val > 0)
  {
    printf("Parsing config...\n");

    mcconf->pwm_mode = rx_buffer[ind++];
    mcconf->comm_mode = rx_buffer[ind++];
    mcconf->motor_type = rx_buffer[ind++];
    mcconf->sensor_mode = rx_buffer[ind++];

    mcconf->l_current_max = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_current_min = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_in_current_max = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_in_current_min = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_abs_current_max = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_min_erpm = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_max_erpm = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_max_erpm_fbrake = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_max_erpm_fbrake_cc = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_min_vin = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_max_vin = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_battery_cut_start = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_battery_cut_end = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_slow_abs_current = rx_buffer[ind++];
    mcconf->l_rpm_lim_neg_torque = rx_buffer[ind++];
    mcconf->l_temp_fet_start = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_temp_fet_end = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_temp_motor_start = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_temp_motor_end = bldc_buffer_get_float32(rx_buffer, 1000.0, &ind);
    mcconf->l_min_duty = bldc_buffer_get_float32(rx_buffer, 1000000.0, &ind);
    mcconf->l_max_duty = bldc_buffer_get_float32(rx_buffer, 1000000.0, &ind);

    mcconf->lo_current_max = mcconf->l_current_max;
    mcconf->lo_current_min = mcconf->l_current_min;
    mcconf->lo_in_current_max = mcconf->l_in_current_max;
    mcconf->lo_in_current_min = mcconf->l_in_current_min;

    mcconf->sl_min_erpm = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000.0;
    mcconf->sl_min_erpm_cycle_int_limit = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000.0;
    mcconf->sl_max_fullbreak_current_dir_change = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000.0;
    mcconf->sl_cycle_int_limit = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000.0;
    mcconf->sl_phase_advance_at_br = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000.0;
    mcconf->sl_cycle_int_rpm_br = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000.0;
    mcconf->sl_bemf_coupling_k = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000.0;

    memcpy(mcconf->hall_table, rx_buffer + ind, 8);
    ind += 8;
    mcconf->hall_sl_erpm = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000.0;

    mcconf->foc_current_kp = bldc_buffer_get_float32(rx_buffer, 1e5, &ind);
    mcconf->foc_current_ki = bldc_buffer_get_float32(rx_buffer, 1e5, &ind);
    mcconf->foc_f_sw = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    mcconf->foc_dt_us = bldc_buffer_get_float32(rx_buffer, 1e6, &ind);
    mcconf->foc_encoder_inverted = rx_buffer[ind++];
    mcconf->foc_encoder_offset = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    mcconf->foc_encoder_ratio = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    mcconf->foc_sensor_mode = rx_buffer[ind++];
    mcconf->foc_pll_kp = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    mcconf->foc_pll_ki = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    mcconf->foc_motor_l = bldc_buffer_get_float32(rx_buffer, 1e8, &ind);
    mcconf->foc_motor_r = bldc_buffer_get_float32(rx_buffer, 1e5, &ind);
    mcconf->foc_motor_flux_linkage = bldc_buffer_get_float32(rx_buffer, 1e5, &ind);
    mcconf->foc_observer_gain = bldc_buffer_get_float32(rx_buffer, 1e0, &ind);
    mcconf->foc_duty_dowmramp_kp = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    mcconf->foc_duty_dowmramp_ki = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    mcconf->foc_openloop_rpm = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    mcconf->foc_sl_openloop_hyst = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    mcconf->foc_sl_openloop_time = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    mcconf->foc_sl_d_current_duty = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    mcconf->foc_sl_d_current_factor = bldc_buffer_get_float32(rx_buffer, 1e3, &ind);
    memcpy(mcconf->foc_hall_table, rx_buffer + ind, 8);
    ind += 8;
    mcconf->foc_sl_erpm = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000.0;

    mcconf->s_pid_kp = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000000.0;
    mcconf->s_pid_ki = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000000.0;
    mcconf->s_pid_kd = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000000.0;
    mcconf->s_pid_min_erpm = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000.0;

    mcconf->p_pid_kp = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000000.0;
    mcconf->p_pid_ki = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000000.0;
    mcconf->p_pid_kd = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000000.0;
    mcconf->p_pid_ang_div = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 100000.0;

    mcconf->cc_startup_boost_duty = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000000.0;
    mcconf->cc_min_current = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000.0;
    mcconf->cc_gain = (float)bldc_buffer_get_int32(rx_buffer, &ind) / 1000000.0;
    mcconf->cc_ramp_step_max = bldc_buffer_get_float32(rx_buffer, 1e6, &ind);

    mcconf->m_fault_stop_time_ms = bldc_buffer_get_int32(rx_buffer, &ind);
    mcconf->m_duty_ramp_step = (float)bldc_buffer_get_float32(rx_buffer, 1000000.0, &ind);
    mcconf->m_duty_ramp_step_rpm_lim = (float)bldc_buffer_get_float32(rx_buffer, 1000000.0, &ind);
    mcconf->m_current_backoff_gain = (float)bldc_buffer_get_float32(rx_buffer, 1000000.0, &ind);
    mcconf->m_encoder_counts = bldc_buffer_get_uint32(rx_buffer, &ind);
    mcconf->m_sensor_port_mode = rx_buffer[ind++];
  }
}

uint8_t bldc_set_mc(int can_socket, int id, bldc_mc_configuration * mcconf, struct timeval *timeout)
{
  uint8_t tx_buffer[BLDC_RX_BUF_LEN];
  uint8_t rx_buffer[BLDC_RX_BUF_LEN];
  int ind = 0;

  // using get command until code is checked (for sanity).
  tx_buffer[ind++] = COMM_SET_MCCONF;

  // Bystream the config:

  tx_buffer[ind++] = mcconf->pwm_mode;
  tx_buffer[ind++] = mcconf->comm_mode;
  tx_buffer[ind++] = mcconf->motor_type;
  tx_buffer[ind++] = mcconf->sensor_mode;

  bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_current_max * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_current_min * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_in_current_max * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_in_current_min * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_abs_current_max * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_min_erpm * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_max_erpm * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_max_erpm_fbrake * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_max_erpm_fbrake_cc * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_min_vin * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_max_vin * 1000.0), &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->l_battery_cut_start, 1000.0, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->l_battery_cut_end, 1000.0, &ind);
	tx_buffer[ind++] = mcconf->l_slow_abs_current;
	tx_buffer[ind++] = mcconf->l_rpm_lim_neg_torque;
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_temp_fet_start * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_temp_fet_end * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_temp_motor_start * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_temp_motor_end * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_min_duty * 1000000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->l_max_duty * 1000000.0), &ind);

	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->sl_min_erpm * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->sl_min_erpm_cycle_int_limit * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->sl_max_fullbreak_current_dir_change * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->sl_cycle_int_limit * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->sl_phase_advance_at_br * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->sl_cycle_int_rpm_br * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->sl_bemf_coupling_k * 1000.0), &ind);

	memcpy(tx_buffer + ind, mcconf->hall_table, 8);
	ind += 8;
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->hall_sl_erpm * 1000.0), &ind);

	bldc_buffer_append_float32(tx_buffer, mcconf->foc_current_kp, 1e5, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_current_ki, 1e5, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_f_sw, 1e3, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_dt_us, 1e6, &ind);
	tx_buffer[ind++] = mcconf->foc_encoder_inverted;
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_encoder_offset, 1e3, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_encoder_ratio, 1e3, &ind);
	tx_buffer[ind++] = mcconf->foc_sensor_mode;
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_pll_kp, 1e3, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_pll_ki, 1e3, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_motor_l, 1e8, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_motor_r, 1e5, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_motor_flux_linkage, 1e5, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_observer_gain, 1e0, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_duty_dowmramp_kp, 1e3, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_duty_dowmramp_ki, 1e3, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_openloop_rpm, 1e3, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_sl_openloop_hyst, 1e3, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_sl_openloop_time, 1e3, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_sl_d_current_duty, 1e3, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->foc_sl_d_current_factor, 1e3, &ind);
	memcpy(tx_buffer + ind, mcconf->foc_hall_table, 8);
	ind += 8;
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->foc_sl_erpm * 1000.0), &ind);

	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->s_pid_kp * 1000000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->s_pid_ki * 1000000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->s_pid_kd * 1000000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->s_pid_min_erpm * 1000.0), &ind);

	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->p_pid_kp * 1000000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->p_pid_ki * 1000000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->p_pid_kd * 1000000.0), &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->p_pid_ang_div, 1e5, &ind);

	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->cc_startup_boost_duty * 1000000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->cc_min_current * 1000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->cc_gain * 1000000.0), &ind);
	bldc_buffer_append_int32(tx_buffer, (int32_t)(mcconf->cc_ramp_step_max * 1000000.0), &ind);

	bldc_buffer_append_int32(tx_buffer, mcconf->m_fault_stop_time_ms, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->m_duty_ramp_step, 1000000.0, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->m_duty_ramp_step_rpm_lim, 1000000.0, &ind);
	bldc_buffer_append_float32(tx_buffer, mcconf->m_current_backoff_gain, 1000000.0, &ind);
	bldc_buffer_append_uint32(tx_buffer, mcconf->m_encoder_counts, &ind);
	tx_buffer[ind++] = mcconf->m_sensor_port_mode;

  printf("TX Length: %i\n",ind);

  int val = bldc_comm_buffer(can_socket, id, tx_buffer, ind, rx_buffer, BLDC_RX_BUF_LEN, timeout);
  if (val > 0)
  {
    printf("Got responce");

  }
}

uint8_t bldc_get_decoded_ppm(int can_socket, int id, float *servo_ms, float *servo_ms_last, struct timeval *timeout)
{
  uint8_t tx_buffer[1];
  uint8_t rx_buffer[20];
  int index = 0;
  tx_buffer[0] = COMM_GET_DECODED_PPM;

  int val = bldc_comm_buffer(can_socket, id, tx_buffer, 1, rx_buffer, 20, timeout);
  if (val > 0)
  {
    index = 1;
    *servo_ms = (float)bldc_buffer_get_int32(rx_buffer, &index) / 1000000.0;
    *servo_ms_last = (float)bldc_buffer_get_int32(rx_buffer, &index) / 1000000.0;
  }

}

int bldc_gen_proc_cmd(struct can_frame frames[], int id, const uint8_t tx_buffer[], const uint16_t tx_len)
{
  uint16_t i = 0;
  uint16_t j = 0;
  uint16_t frame_cnt = 0;
  uint16_t tx_pos = 0;
  uint8_t framebytes = 0;

  // TODO: Use a single packet for short commands.

  // Fill teh frames:
  while (tx_pos < tx_len)
  {
    if (tx_pos < 255)
    {
      framebytes = (tx_len - tx_pos > 7) ? 7: tx_len - tx_pos;
      printf("Writing Frame %i, with %i bytes\n", frame_cnt, framebytes);
      frames[frame_cnt].can_id = bldc_gen_can_id(BLDC_PACKET_FILL_RX_BUFFER, id);
      frames[frame_cnt].can_dlc = framebytes+1;
      frames[frame_cnt].data[0] = tx_pos;
      memcpy(frames[frame_cnt].data+1, tx_buffer+tx_pos, framebytes);
    }
    else
    {
      framebytes = (tx_len - tx_pos > 6) ? 6: tx_len - tx_pos;
      frames[frame_cnt].can_id = bldc_gen_can_id(BLDC_PACKET_FILL_RX_BUFFER_LONG, id);
      frames[frame_cnt].can_dlc = framebytes+2;
      frames[frame_cnt].data[0] = tx_pos >> 8;
      frames[frame_cnt].data[1] = tx_pos;
      memcpy(frames[frame_cnt].data+2, tx_buffer+tx_pos, framebytes);
    }

    tx_pos += framebytes;
    frame_cnt++;
  }

  // Generate a CRC for the encoded data:
  uint16_t crc = bldc_crc16(tx_buffer, tx_len);

  // Second, generate the process command:
  frames[frame_cnt].can_id = bldc_gen_can_id(BLDC_PACKET_PROCESS_RX_BUFFER, id);
  frames[frame_cnt].data[0] = 0x0; // rx_buffer_last_id - the address to send the command to
  frames[frame_cnt].data[1] = 0x0; // set to true if the command is to be relayed.
  frames[frame_cnt].data[2] = 0xFF & (tx_len>>8); // upper byte of txlen
  frames[frame_cnt].data[3] = 0xFF & tx_len;   // lower byte of txlen
  frames[frame_cnt].data[4] = 0xFF & (crc>>8); // crc high
  frames[frame_cnt].data[5] = 0xFF & crc;   // crc low
  frames[frame_cnt].can_dlc = 6;

  return frame_cnt+1;
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

  // Read a short buffer:
  if (frame->can_id == bldc_gen_can_id(BLDC_PACKET_PROCESS_SHORT_BUFFER, id))
  {
     //printf("Processing a short buffer: %i\n", frame->can_dlc); 
     memcpy(rx_buffer, frame->data+2, frame->can_dlc -2);
     rval = frame->can_dlc -2; 

  }


  return rval;
}

// wrap up the other two functions in a more user friendly function call...
int bldc_comm_buffer(int can_socket, int id, const uint8_t tx_buffer[], const uint16_t tx_len,
                     uint8_t rx_buffer[], uint16_t rx_len, struct timeval *timeout)
{

  struct timespec max_time, now_time;
  fd_set readSet;
  struct timeval can_timeout;
  uint8_t read_can_port=0;
  struct can_frame tx_frames[BLDC_CANFRAME_BUF_LEN];
  struct can_frame rx_frame;
  uint16_t i, n_tx_frames;
  uint32_t nbytes;
  int rval=0;
  uint8_t tx_retry;

  // Set abs_time to maximum time the function should be called for.
  if (timeout != NULL)
  {
    clock_gettime(CLOCK_MONOTONIC, &max_time);
    max_time.tv_sec += timeout->tv_sec;
    max_time.tv_nsec += timeout->tv_usec*1000;
  }

  // Generate and transmit thte frames:
  n_tx_frames = bldc_gen_proc_cmd(tx_frames, id, tx_buffer, tx_len);
  for (i=0; i<n_tx_frames; i++)
  {
    tx_retry = 0;
    while (tx_retry < 5)
    {
      nbytes = write(can_socket, &tx_frames[i], sizeof(struct can_frame));
      if (nbytes == sizeof(struct can_frame))
      {
        break;
      }
      else if (tx_retry == 5)
      {
        printf("Error in TX\n");
      }
      else
      { 
        usleep(10);
        printf("Retrying packet send\n");
      }
    }
  }
  
  // Check we should be reading the returned data..
  if ((rx_buffer != NULL) && (rx_len > 0))
  {
    read_can_port = 1;
  }

  // Read the port 
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
        nbytes = read(can_socket, &rx_frame, sizeof(struct can_frame));
        if(nbytes)
        {
          // Test out the new read function:
          rval = bldc_fill_rxbuf(&rx_frame, 0,  rx_buffer, rx_len);
          if (rval != 0)
          {
            read_can_port = 0;
          }
        }
      }
    }
    // Check timeout:
    if (timeout != NULL)
    {
      clock_gettime(CLOCK_MONOTONIC, &now_time);
      if ((now_time.tv_sec > max_time.tv_sec) && (now_time.tv_nsec > max_time.tv_nsec))
      {
        printf("Timeout exceeded");
        read_can_port = 0;
        rval = -1;
      }
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

