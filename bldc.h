#ifndef libbldc_H
#define libbldc_H

#include<linux/can.h>
#include <stdint.h>

/** \file
    \brief  BLDC Library

    Functions to allow encoding/decoding of CAN commands
    used on the VESC speed controller. The name of this
    library reflects the naming convention of the project 'bldc'.
 */

/// A struct containing all the extended status information available from a VESC.
/** The struct contains all the status (including extended status) information
    sent from the VESC. This data is sent using three CAN packet identifiers,
    and can be retrieved from a can packet using a specific function of this
    library.
*/
typedef struct {
  /// CAN id of VESC
  int id;
  /// Electrical RPM of the motor    
  float erpm;
  /// Current being delivered to the motor     
  float current_motor;
  /// Duty cycle reported by the VESC
  float duty_now;
  /// Current input to the speed controller from the battery
  float current_input;
  /// Voltage input to the speed controller from the battery
  float voltage_input;
  /// Speed controller uptime in milliseconds
  uint32_t uptime;
  /// Temperature of the speed controller
  float temperature_mos1;
  /// Temperature of the motor
  float temperature_motor;
  /// Reported fault codes
  uint8_t fault_code;       
} bldc_status;

/// The Packet ID is an identifier used by the VESC to identify types of data sent.
/** The Packet ID identifies what type of data the CAN packet contains, and is
    stored in bits 8+ the CAN ID field.
*/
typedef enum {
  /// Set the duty cycle of the motor
  BLDC_PACKET_SET_DUTY = 0,
  /// Set the motor current  
  BLDC_PACKET_SET_CURRENT,  
  /// Set the braking current        
  BLDC_PACKET_SET_CURRENT_BRAKE,
  /// Set the motor ERPM    
  BLDC_PACKET_SET_RPM,
  /// Set the motor position            
  BLDC_PACKET_SET_POS,
  /// Copy data to the VESC recieve buffer            
  BLDC_PACKET_FILL_RX_BUFFER,
  /// Copy data to the VESC recieve buffer
  BLDC_PACKET_FILL_RX_BUFFER_LONG,
  /// Process a command with data from the recieve buffer
  BLDC_PACKET_PROCESS_RX_BUFFER,
  /// Process a command with data from the recieve buffer
  BLDC_PACKET_PROCESS_SHORT_BUFFER,
  /// The motor status (1 of 3)
  BLDC_PACKET_STATUS,
  /// The motor status (2 of 3)             
  BLDC_PACKET_STATUS2,
  /// The motor status (3 of 3)   
  BLDC_PACKET_STATUS3              
} BLDC_PACKET_ID;

/// Commands which can be run on a VESC using the RX buffer.
/** The Packet ID identifies what type of data the CAN packet contains, and is
    stored in bits 8+ the CAN ID field.
*/
typedef enum {
	COMM_FW_VERSION = 0,
	COMM_JUMP_TO_BOOTLOADER,
	COMM_ERASE_NEW_APP,
	COMM_WRITE_NEW_APP_DATA,
	COMM_GET_VALUES,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_POS,
	COMM_SET_DETECT,
	COMM_SET_SERVO_POS,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_GET_MCCONF_DEFAULT,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_GET_APPCONF_DEFAULT,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_DETECT_MOTOR_R_L,
	COMM_DETECT_MOTOR_FLUX_LINKAGE,
	COMM_DETECT_ENCODER,
	COMM_DETECT_HALL_FOC,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_ADC,
	COMM_GET_DECODED_CHUK,
	COMM_FORWARD_CAN,
	COMM_SET_CHUCK_DATA,
	COMM_CUSTOM_APP_DATA
} COMM_PACKET_ID;

/// A VESC compatible function to append a 16bit integer to a bit stream.
/** The integer is written to the index position of the buffer, while the
    index parameter is incremented to point to the next available position
    in the buffer.
    \param buffer destination buffer for the integer.
    \param number number to be appended to the buffer.
    \param index position in buffer to write the number.
*/
void bldc_buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);

/// A VESC compatible function to append an unsigned 16bit integer to a bit stream.
/** The integer is written to the index position of the buffer, while the
    index parameter is incremented to point to the next available position
    in the buffer.
    \param buffer destination buffer for the integer.
    \param number number to be appended to the buffer.
    \param index position in buffer to write the number.
*/
void bldc_buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);

/// A VESC compatible function to append an integer to a bit stream.
/** The integer is written to the index position of the buffer, while the
    index parameter is incremented to point to the next available position
    in the buffer.
    \param buffer destination buffer for the integer.
    \param number number to be appended to the buffer.
    \param index position in buffer to write the number.
*/
void bldc_buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);

/// A VESC compatible function to append an unsigned integer to a bit stream.
/** The integer is written to the index position of the buffer, while the
    index parameter is incremented to point to the next available position
    in the buffer.
    \param buffer destination buffer for the integer.
    \param number number to be appended to the buffer.
    \param index position in buffer to write the number.
*/
void bldc_buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);

/// A VESC compatible function to append an float to a bit stream.
/** The function scales number by the scaling factor before converting it to
    and integer, the integer is then appended to the bit stream. 
    The integer is written to the index position of the buffer, while the
    index parameter is incremented to point to the next available position
    in the buffer.
    \param buffer destination buffer for the integer.
    \param number number to be appended to the buffer.
    \param scale amount to scale the floating point by before appending to the stream
    \param index position in buffer to write the number.
*/
void bldc_buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index);

/// A VESC compatible function to append an float to a bit stream.
/** The function scales number by the scaling factor before converting it to
    and integer, the integer is then appended to the bit stream.
    The integer is written to the index position of the buffer, while the
    index parameter is incremented to point to the next available position
    in the buffer.
   \param buffer destination buffer for the integer.
   \param number number to be appended to the buffer.
   \param scale amount to scale the floating point by before appending to the stream
   \param index position in buffer to write the number.
*/
void bldc_buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);

/// A VESC compatible function to read a 16 bit integer from a bit stream.
/** The integer is read starting from the position indicated by index,
    which is automatic incremented
   \param buffer to read the integer from.
   \param index position in buffer to read the integer.
   \return the integer stored at position index of buffer.
*/
int16_t  bldc_buffer_get_int16(const uint8_t *buffer, int32_t *index);

/// A VESC compatible function to read a 16 bit unsigned integer from a bit stream.
/** The integer is read starting from the position indicated by index,
    which is automatic incremented
   \param buffer to read the integer from.
   \param index position in buffer to read the integer.
   \return the integer stored at position index of buffer.
*/
uint16_t bldc_buffer_get_uint16(const uint8_t *buffer, int32_t *index);

/// A VESC compatible function to read a integer from a bit stream.
/** The integer is read starting from the position indicated by index,
    which is automatic incremented
   \param buffer to read the integer from.
   \param index position in buffer to read the integer.
   \return the integer stored at position index of buffer.
*/
int32_t  bldc_buffer_get_int32(const uint8_t *buffer, int32_t *index);

/// A VESC compatible function to read a  unsigned integer from a bit stream.
/** The integer is read starting from the position indicated by index,
    which is automatic incremented
   \param buffer to read the integer from.
   \param index position in buffer to read the integer.
   \return the integer stored at position index of buffer.
*/
uint32_t bldc_buffer_get_uint32(const uint8_t *buffer, int32_t *index);

/// A VESC compatible function to read a integer and convert to a float from a bit stream.
/** An integer is read from position index of the buffer and index is incremented.
    The read value is scaled by a scaling factor scale, before being returned
    by the function.
   \param buffer to read the integer from.
   \param scale ammount to scale the value from the datastream by
   \param index position in buffer to read the integer.
   \return the value stored at position index of buffer, after scaling.
*/
float    bldc_buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);

/// A VESC compatible function to read a integer and convert to a float from a bit stream.
/** An integer is read from position index of the buffer and index is incremented.
    The read value is scaled by a scaling factor scale, before being returned
    by the function.
   \param buffer to read the integer from.
   \param scale ammount to scale the value from the datastream by
   \param index position in buffer to read the integer.
   \return the value stored at position index of buffer, after scaling.
*/
float    bldc_buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);

/// Generate a CAN id to be used for the can frame
/** Generate the CAN id for a can frame which combines the packet
    id and target vesc can id.
    \param pack_id, id of packet 
    \param vesc_id, can id of VESC
    \return can frame id for data type and vesc can id.
*/
canid_t bldc_gen_can_id(BLDC_PACKET_ID pack_id, uint8_t vesc_can_id);

/// Generate a CAN frame to set the motor duty cycle.
/** Function to generate a can frame which sets the duty cycle of the
    motor with CAN identifier id.
    \param frame frame to be written to
    \param id CAN id of target controller
    \param duty duty cucle to set, where -ve puts the motor in reverse.
*/
void bldc_set_duty(struct can_frame *frame, int id, float duty);

/// Generate a CAN frame to set the motor current.
/** Function to generate a can frame which sets the current of the
    motor with CAN identifier id.
    \param frame frame to be written to
    \param id CAN id of target controller
    \param current current to set on the motor.
*/
void bldc_set_current(struct can_frame *frame, int id, float current);

/// Generate a CAN frame to set the motor braking current current.
/** Function to generate a can frame which sets the current of the
    motor with CAN identifier id.
    \param frame frame to be written to
    \param id CAN id of target controller
    \param current braking current to set on the motor.
*/
void bldc_set_current_brake(struct can_frame *frame, int id, float current);

/// Generate a CAN frame to set the motor electrical rpm.
/** Function to generate a can frame which sets the electrical rpm of the
    motor with CAN identifier id.
    \param frame frame to be written to
    \param id CAN id of target controller
    \param erpm electrical rpm to set on the motor.
*/
void bldc_set_erpm(struct can_frame *frame, int id, int32_t erpm);

/// Read the status frame sent by a controller, with matching can identifier.
/** Function to read part of the status infomation sent from a VESC
    and update the status structure accordingly.
    Note that the recieved packet must come from a controller with
    id matching the id stored in the structure. If the read is
    successful the function will return 1, if the status was
    from an incorrect id the function will return -1.

    \param frame frame to read the data from.
    \param status the status structure to update.
    \retrun 1 if the packet was read successfully, or -1 if the
    id does not match. 0 if the packet identifier was not a status
    packet.
*/
int bldc_get_status(const struct can_frame *frame, bldc_status *status);

// Functions to write long commands to the VESC's buffer.
uint8_t bldc_reboot(struct can_frame frames[], int id);
uint8_t bldc_get_values(struct can_frame frames[], int id);

/// Function to generate the CAN frames needed to process a command on the VESC
/** Function to read part of the status infomation sent from a VESC
    and update the status structure accordingly.
    Note that the recieved packet must come from a controller with
    id matching the id stored in the structure. If the read is
    successful the function will return 1, if the status was
    from an incorrect id the function will return -1.

    \param frames Array of frames to write data into.
    \param id Target VESC CAN id
    \param txbuffer buffer to be transferred to the VESC rx buffer
    \param len length of txbuffer.
    \retrun number of frames generated.
*/
void bldc_gen_proc_cmd(struct can_frame frames[], int id, const uint8_t tx_buffer[], uint8_t len);


/// Function to take recieved data from the VESC and fill a recieve buffer.
/** Fills data from a can frame into the RX buffer.
    \param frame Input can frame.
    \param id CAN ID of sender
    \param rx_buffer[] destination for filled data.
    \param maxlen max length of the recieve buffer.
*/
int bldc_fill_rxbuf(const struct can_frame * frame, int id,  uint8_t rx_buffer[], uint16_t maxlen);

/// Generate a CRC16 from a array of bytes, to check data integrity on the VESC
/** Function to generate a CRC16 checksum which the VESC will use to check
    data integrity. This functionality is used for processing commands writtent
    to the RX buffer of the VESC.
    \param buf buffer to checksum
    \param len length of bufer
    \return CRC16 of data
*/
uint16_t bldc_crc16(const uint8_t* buf, unsigned int len);
#endif //libbldc_H
