// here a large number of global variables is defined
// to be included in the source files

#ifdef DEFINEGLOBALSHERE
#define GLOBAL
#else
#define GLOBAL extern
#endif

// node states from main.c
GLOBAL NodeState UP_STATE;
GLOBAL NodeState DOWN_STATE;

// special node state from main.c
GLOBAL uint8_t DIRECTION_SET;

GLOBAL config_data_flash_struct setup_data;

// number of splitter from main.c
GLOBAL uint8_t n_splitter;

// splitter counters, poll & command
GLOBAL uint8_t i_s_splitter;
GLOBAL uint8_t i_p_splitter;

// from main.c
GLOBAL int interrupt_up_timeout_time;
GLOBAL int interrupt_down_timeout_time;

// from main.c
GLOBAL Line send_down_line;
GLOBAL uint16_t angles[4];

// timout counters from main.c
GLOBAL uint8_t down_poll_counter;
GLOBAL uint8_t up_poll_counter;

// timout counters from main.c
GLOBAL int down_timeout_time;
GLOBAL int up_timeout_time;

GLOBAL uint8_t transmittedUP;
GLOBAL uint8_t receivedUP;
GLOBAL uint8_t transmittedDOWN;
GLOBAL uint8_t receivedDOWN;

GLOBAL uint16_t transmittedUPtime;
GLOBAL uint16_t transmittedDOWNtime;
GLOBAL uint16_t receivedUPtime;
GLOBAL uint16_t receivedDOWNtime;

GLOBAL SPI_HandleTypeDef hspi1;

GLOBAL Line init_message_line;
GLOBAL Line splitter_message_lines[8];
GLOBAL Line ack_message_line;
GLOBAL Line sensor_message_line;
GLOBAL Line current_command_line;
GLOBAL Line send_down_line_now;

GLOBAL int get_up_time;
GLOBAL uint8_t data_mode;

GLOBAL uint8_t SET_FLASH;
GLOBAL uint8_t PROGRAMM_SENSOR;

GLOBAL void (*MX_SPI1_Init_Pointer)(void);

GLOBAL UART_HandleTypeDef *huart_DOWN;
GLOBAL UART_HandleTypeDef *huart_UP;
