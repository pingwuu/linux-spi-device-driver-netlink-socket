
#ifndef GENL_TEST_H
#define GENL_TEST_H

#include <linux/netlink.h>
#include <stdbool.h>

#ifdef __KERNEL__
#include <linux/delay.h>
#endif

#ifndef __KERNEL__
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#endif

#define GENL_TEST_FAMILY_NAME		"genl_test"
#define GENL_TEST_MCGRP0_NAME		"genl_mcgrp0"
#define GENL_TEST_MCGRP1_NAME		"genl_mcgrp1"
#define GENL_TEST_MCGRP2_NAME		"genl_mcgrp2"

#define GENL_TEST_ATTR_MSG_MAX		500
#define GENL_TEST_HELLO_INTERVAL	5000

enum {
	GENL_TEST_C_UNSPEC,		/* Must NOT use element 0 */
	GENL_TEST_C_MSG,
};

enum genl_test_multicast_groups {
	GENL_TEST_MCGRP0,
	GENL_TEST_MCGRP1,
	GENL_TEST_MCGRP2,
};
#define GENL_TEST_MCGRP_MAX		3

#ifndef __KERNEL__
static char* genl_test_mcgrp_names[GENL_TEST_MCGRP_MAX] = {
	GENL_TEST_MCGRP0_NAME,
	GENL_TEST_MCGRP1_NAME,
	GENL_TEST_MCGRP2_NAME,
};
#endif

enum genl_test_attrs {
	GENL_TEST_ATTR_UNSPEC,		/* Must NOT use element 0 */

	GENL_TEST_ATTR_MSG,

	__GENL_TEST_ATTR__MAX,
};
#define GENL_TEST_ATTR_MAX (__GENL_TEST_ATTR__MAX - 1)

static struct nla_policy genl_test_policy[GENL_TEST_ATTR_MAX+1] = {
	[GENL_TEST_ATTR_MSG] = {
		.type = NLA_STRING,
#ifdef __KERNEL__
		.len = GENL_TEST_ATTR_MSG_MAX
#else
		.maxlen = GENL_TEST_ATTR_MSG_MAX
#endif
	},
};

#define FIFO_SIZE 5
#define PATTERN_ARRAY_SIZE 10
#define LED_CONTROL_PATTERN_SIZE 26
#define MAX_DEVICES  1

//int valid_trigger_pins[4] = {0, 1, 10, 12}; //IO pins are restricted to IO8 IO15
int invalid_echo_pins[8] = {7, 8, 14, 15, 16, 17, 18, 19}; //These pins don't have interrupt for "both" edges

int valid_spi_pins[2] = {11, 13}; //Only these pins have SPI MOSI and SCK support
// Array containing the spi gpio pin configuration
int spi_mapping_array[20][4] = {
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
											{-1,24,44,72}, //IO11 MOSI
											{15,42,-1,-1}, //IO12 used as CS
											{-1,30,46,-1}, //IO13 SCK
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
};

// Array containing the gpio pin configuration
int spi_values_array[20][4] = {
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
											{-1,0,1,0}, //IO11 MOSI
											{0,0,-1,-1}, //IO12 used as CS
											{-1,0,1,-1}, //IO13 SCK
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
			                                {-1,-1,-1,-1},
};

int gpio_mapping_array[20][4] = {
			                                {11,32,-1,-1},
							{12,28,45,-1},
							{13,34,77,-1},
 							{14,16,76,64}, //14, 16, 76, 64
							{6,36,-1,-1},
							{0,18,66,-1},
							{1,20,68,-1},
							{38,-1,-1,-1},
							{40,-1,-1,-1},
							{4,22,70,-1},
							{10,26,74,-1},
							{-1,24,44,72}, //IO11 MOSI
							{15,42,-1,-1}, //IO12 used as CS
							{-1,30,46,-1}, //IO13 SCK
							{48,-1,-1,-1},
							{50,-1,-1,-1},
							{52,-1,-1,-1},
							{54,-1,-1,-1},
							{56,-1,60,78},
							{58,-1,60,79} //7, 30, 60, 79
};

// Array containing the gpio pin configuration
int gpio_values_array[20][4] = {
			                               {0,0,-1,-1},
							{0,0,0,-1},
							{0,1,0,-1},
 							{0,1,0,0},
							{0,0,-1,-1},
							{0,0,0,-1},
							{0,0,0,-1},
							{0,-1,-1,-1},
							{0,-1,-1,-1},
							{0,0,0,-1},
							{0,0,0,-1},
							{-1,0,1,0}, //IO11 MOSI
							{0,0,-1,-1}, //IO12 used as CS
							{-1,0,1,-1}, //IO13 SCK
							{0,-1,-1,-1},
							{0,-1,-1,-1},
							{0,-1,-1,-1},
							{0,-1,-1,-1},
							{0,-1,1,1},
							{0,-1,1,1}
};

#define ROW_SIZE(arr) ((int) (sizeof (arr) / sizeof (arr)[0]))
#define COLUMN_SIZE(arr) ((int) sizeof ((arr)[0]) / sizeof (int))

struct hcsrdata {
	bool is_valid;
	uint64_t data;
	uint64_t timestamp;
};

struct led_rep {
	uint8_t led[LED_CONTROL_PATTERN_SIZE]; //1 byte represents 1 row of LEDs
};

enum {
	PATTERN_DIR_TOO_CLOSE = -1,
	PATTERN_DIR_FORWARD = 0,
	PATTERN_DIR_REVERSE = 1
};

#ifdef __KERNEL__
/* per device structure */
struct kbuf_dev {
	bool is_measuring;
	bool is_gpio_requested;
	bool is_irq_requested;
	bool irq_type_flag;
	bool display_has_exited;	
	unsigned int trigger_pin;
	unsigned int echo_pin;
	unsigned int cs_pin;
	unsigned int mosi_pin;
	unsigned int sck_pin;
	unsigned int m_samples;
	unsigned int delta;
	unsigned int trigger_interval;
	unsigned int current_sample_idx;
	unsigned int irq_count;
	unsigned int current_fifo_write_idx;
	unsigned int current_fifo_read_idx;
	unsigned int current_fifo_length;
	unsigned int dev_no;
	unsigned int enable;
	char* name;
	uint64_t time1, time2;
	uint64_t last_distance;
	unsigned int spi_speed_hz;
	unsigned int spi_msg_len;
	uint8_t* spi_tx_buff;
	int pattern_dir;
	struct led_rep pattern_array[PATTERN_ARRAY_SIZE];
	struct hcsrdata hcsr_data[FIFO_SIZE];
	struct mutex read_lock;
	struct mutex flag_lock;
	struct semaphore signal_lock;
	uint64_t* temp_buf;
	struct task_struct *spi_kthread_task;
	struct task_struct *hcsr_kthread_task;
	struct work_struct irq_work;
	struct device* hcsr_device;
	struct spi_device *spi;
	struct class *kbuf_dev_class;          /* Tie with the device model */
};

uint8_t display_config [] = {
	0x0F,0x01,
	0x0F,0x01,
	0x0F,0x00,
	0x09,0x00,
	0x0A,0x0F,
	0x0B,0x07,
	0x0C,0x01,
};

//Prototypes
//static void greet_group(unsigned int group);
int configure_irq(struct kbuf_dev* kbuf_devp, bool rising);
irqreturn_t irq_handler(int irq, void *dev_id);
void send_msg_to_user(struct kbuf_dev* kbuf_devp);
void update_pattern(struct kbuf_dev* kbuf_devp);
#endif

enum msg_type {
	MSG_CONFIG_PINS,
	MSG_CONFIG_PATTERN,
	MSG_START_MEASUREMENT,
	MSG_READ_DISTANCE,
	
	NUM_MSGS
};


struct netlink_userdata {
	enum msg_type msg;
	unsigned int trigger_pin;
	unsigned int echo_pin;
	unsigned int cs_pin;
	unsigned int mosi_pin;
	unsigned int sck_pin;
	unsigned int m_samples;
	unsigned int delta;
	unsigned enable;
	int distance_cm;
	int pattern_dir;
	struct led_rep pattern_array[PATTERN_ARRAY_SIZE];
};

struct led_rep pattern[] = {
	{
	{
	0x0F, 0x00,           
	0x0C, 0x01,          
	0x09, 0x00,          
	0x0A, 0x0F,          
	0x0B, 0x07,

	0x01, 0x00,
	0x02, 0x00,
	0x03, 0x00,
	0x04, 0x8B,
	0x05, 0x4B,
	0x06, 0x3F,
	0x07, 0x4B,
	0x08, 0x88
	}	
	},
	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,

	0x01, 0x00,
	0x02, 0x00,
	0x03, 0x13,
	0x04, 0xCB,
	0x05, 0x3F,
	0x06, 0xCB,
	0x07, 0x10,
	0x08, 0x00
	}	
	},
	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,

	0x01, 0x00,
	0x02, 0x8B,
	0x03, 0x4B,
	0x04, 0x3F,
	0x05, 0x4B,
	0x06, 0x88,
	0x07, 0x00,
	0x08, 0x00
	}	
	},
	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,

	0x01, 0x00,
	0x02, 0x13,
	0x03, 0xCB,
	0x04, 0x3F,
	0x05, 0xCB,
	0x06, 0x10,
	0x07, 0x00,
	0x08, 0x00,
	}	
	},
/*	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,
	0x01, 0x00,
	0x02, 0x8B,
	0x03, 0x4B,
	0x04, 0x3F,
	0x05, 0x4B,
	0x06, 0x88,
	0x07, 0x00,
	0x08, 0x00
	}	
	},
	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,
	0x01, 0x00,
	0x02, 0x13,
	0x03, 0xCB,
	0x04, 0x3F,
	0x05, 0xCB,
	0x06, 0x10,
	0x07, 0x00,
	0x08, 0x00	
	}	
	},*/
	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,

	0x01, 0x03,
	0x02, 0x1B,
	0x03, 0xFF,
	0x04, 0x1B,
	0x05, 0x00,
	0x06, 0x00,
	0x07, 0x00,
	0x08, 0x00,
	}	
	},
	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,

	0x01, 0x88,
	0x02, 0x4B,
	0x03, 0x3F,
	0x04, 0x4B,
	0x05, 0x8B,
	0x06, 0x00,
	0x07, 0x00,
	0x08, 0x00,
	}
	},

	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,

	0x01, 0x00,
	0x02, 0x10,
	0x03, 0xCB,
	0x04, 0x3F,
	0x05, 0xCB,
	0x06, 0x13,
	0x07, 0x00,
	0x08, 0x00,
	}	
	},

	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,

	0x01, 0x00,
	0x02, 0x00,
	0x03, 0x88,
	0x04, 0x4B,
	0x05, 0x3F,
	0x06, 0x4B,
	0x07, 0x8B,
	0x08, 0x00
	}	
	},

	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,

	0x01, 0x00,
	0x02, 0x00,
	0x03, 0x10,
	0x04, 0xCB,
	0x05, 0x3F,
	0x06, 0xCB,
	0x07, 0x13,
	0x08, 0x00
	}	
	},

/*	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,
	0x01, 0x00,
	0x02, 0x00,
	0x03, 0x88,
	0x04, 0x4B,
	0x05, 0x3F,
	0x06, 0x4B,
	0x07, 0x8B,
	0x08, 0x00
	}	
	},
	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,
	0x01, 0x00,
	0x02, 0x00,
	0x03, 0x10,
	0x04, 0xCB,
	0x05, 0x3F,
	0x06, 0xCB,
	0x07, 0x13,
	0x08, 0x00
	}	
	}, */
	{
	{
	0x0F, 0x00,           
        0x0C, 0x01,          
        0x09, 0x00,          
        0x0A, 0x0F,          
        0x0B, 0x07,

	0x01, 0x00,
	0x02, 0x00,
	0x03, 0x00,
	0x04, 0x00,
	0x05, 0x1B,
	0x06, 0xFF,
	0x07, 0x1B,
	0x08, 0x03
	}	
	}
};



#endif
