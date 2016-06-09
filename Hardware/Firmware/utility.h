/**
  ******************************************************************************
  * @file    ueisart.h
  * @author  Oliver Glauser, IGL, ETH Zurich
  * @version V0.2
  * @date    14-January-2015
  * @brief   Some definition for a half-duplex multi slave local tree usart deviation
  ******************************************************************************

  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UTILITY_FUNCTIONS_H
#define __UTILITY_FUNCTIONS_H

#include "uart_queue.h"
#include "stdio.h"
#include "string.h"
#include "bus.h"
#include "mlx90363.h"
#include "74HC4051D.h"

typedef enum{
	NODE_COLLECTOR  = 0x00,
	NODE_JOINT_TRT  = 0xF4, // 3DOF, Twist-Bend-Twist
	NODE_JOINT_R    = 0x94, // 1DOF, Bend
	NODE_JOINT_T    = 0x54, // 1DOF, Twist
	NODE_JOINT_TUT_TU	= 0x74, // Twist-universal-twist 3DOF, twist+universal part
	NODE_JOINT_TUT_T	= 0x84, // Twist-universal-twist 3DOF twist-only part
	NODE_SPLITTER_GENERIC = 0x08, // Splitter 2 out-going joints
	NODE_SPLITTER_2 = 0x28, // Splitter 2 out-going joints
	NODE_SPLITTER_3 = 0x38, // Splitter 3 out-going joints
	NODE_SPLITTER_5 = 0x58, // Splitter 5 out-going joints
	NODE_UNDEFINED  = 0xFF
} NodeType;

typedef enum{
	IDLE,
	SENDING,
	DELAY,
	RECEIVING,
	END
} NodeState;


typedef struct{
	uint16_t zero_angles[4];
	uint16_t splitter_roll[8];
	uint16_t splitter_pitch[8];
	uint16_t splitter_yaw[8];
	uint8_t color[3];
	uint16_t type;
	uint32_t id;
	uint8_t splitter_outputs[8];
	uint8_t initied_0xAA;
	uint8_t hardware;
	uint8_t polarity;
} config_data_flash_struct;

void LED_init();
void led_on();
void led_off();
void led_set(uint8_t red, uint8_t blue, uint8_t green);
void set_in_Flash(config_data_flash_struct *setup);
void get_from_Flash();
uint8_t process_uart_command(Line *l);
uint8_t is_splitter(uint16_t type);
uint8_t is_joint(uint16_t type);
void reset_joint();
void part_init();

void create_up_message(Line* message_line, uint8_t type, uint8_t* content_8_bytes);
void create_up_messageZ(Line* message_line, uint8_t type);
void update_sensor_messages();

void get_up(uint8_t reset);

void delayUS(uint16_t delayus);

// defined

#define BROADCAST_ID 0xFFFFFFFF


#endif /* __UTILITY_FUNCTIONS_H */
