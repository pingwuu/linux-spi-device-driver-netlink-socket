#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <linux/timer.h>
#include <linux/export.h>
#include <net/genetlink.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>

#include "genl_ex.h"

struct kbuf_dev* kbuf_devp;

static struct genl_family genl_test_family;

static inline uint64_t __attribute__((__always_inline__))
rdtsc(void)
{
    uint32_t a, d;
    __asm __volatile("rdtsc" : "=a" (a), "=d" (d));
    return ((uint64_t) a) | (((uint64_t) d) << 32);
}

int is_gpio_64_to_79(int gpio) { //Check for gpio's which do not have a direction file
	if(gpio >= 64 && gpio <= 79) {
		return 1; //return true
	}
	return 0; //return false
}

int mux_spi_set(struct kbuf_dev* kbuf_devp, int gpio, int value)
{
	if(gpio == -1 || value == -1) {
		printk("-1 found in mux_gpio_set\n");
		return -1;
	}
	printk(KERN_ALERT "Gpio %d Value %d\n", gpio, value);
	gpio_request(gpio, "sysfs");

	if(is_gpio_64_to_79(gpio)) { //No direction file present for these GPIOs
		printk(KERN_ALERT "Gpio is between 64 and 79 %d\n", gpio);
//		gpio_set_value(gpio, value);
	} else {
		gpio_direction_output(gpio, value);
		printk(KERN_ALERT "Not setting direction, gpio %d", gpio);
	}
	return 0;
}

int mux_gpio_set(struct kbuf_dev* kbuf_devp, int gpio, int value, bool out_dir, int column)
{
	if(gpio == -1 || value == -1) {
		printk("-1 found in mux_gpio_set\n");
		return -1;
	}
	printk(KERN_ALERT "mux_gpio_set: dev=%s gpio_request gpio%d\n", kbuf_devp->name, gpio);
	gpio_request(gpio, "sysfs");

	if(column == 0) { //Actual pins
		if(out_dir)
			gpio_direction_output(gpio, value);
		else
			gpio_direction_input(gpio); //for input pins
		
		printk(KERN_ALERT "mux_gpio_set: dev=%s gpio_direction_op/ip %d gpio %d value %d\n", kbuf_devp->name, out_dir, gpio, value);
	} else if(column == 1) {
		if(!out_dir) {
			gpio_direction_output(gpio, 1); //Set level shifter High for input pin
		} else {
			gpio_direction_output(gpio, 0); //Set level shifter low for output pinn
		}
	} else if(!is_gpio_64_to_79(gpio)) {
		gpio_direction_output(gpio, value);
		printk(KERN_ALERT "mux_gpio_set: dev=%s gpio_direction_output gpio%d %d\n", kbuf_devp->name, gpio, value);
	} else {
		gpio_set_value_cansleep(gpio, value);
		printk(KERN_ALERT "mux_gpio_set: dev=%s gpio_set_value_cansleep gpio%d %d\n", kbuf_devp->name, gpio, value);
	}
	return 0;
}

void configure_spi_pins(struct kbuf_dev* kbuf_devp, int spi_array_row)
{                    
    int column;
    int columns_of_spi_array = COLUMN_SIZE(spi_mapping_array);

    for(column = 0; column < columns_of_spi_array; column++) {
        int read_value = spi_mapping_array[spi_array_row][column];
        if(read_value != -1) {
		printk("\n\n Configure pins for IO %d - %d\n\n", spi_array_row, spi_mapping_array[spi_array_row][column]);
		//output for trigger pins and input for echo pins
		mux_spi_set(kbuf_devp, spi_mapping_array[spi_array_row][column],/* 1,*/ spi_values_array[spi_array_row][column]);
	}
    }
}

void configure_pins(struct kbuf_dev* kbuf_devp, int gpio_array_row, bool out_dir) {
    int column;
    int columns_of_gpio_array = 4;

    for(column = 0; column < columns_of_gpio_array; column++) {
        int read_value = gpio_mapping_array[gpio_array_row][column];
        if(read_value != -1) {
			printk(KERN_ALERT "Configure pins: dev=%s  IO%d - %d\n", kbuf_devp->name, gpio_array_row, gpio_values_array[gpio_array_row][column]);
			mux_gpio_set(kbuf_devp, gpio_mapping_array[gpio_array_row][column], gpio_values_array[gpio_array_row][column], out_dir, column);				
        }
    }
}

bool is_valid_spi_pin(int pin) {
	int t;
	if(pin < 0 || pin > 19) { //Works for trigger pins as well
		return false;
	}
	for(t = 0; t < ROW_SIZE(valid_spi_pins); t++) {
		if(pin == valid_spi_pins[t]) {
			return true;
		}
	}
	return false;
}

bool is_valid_pin(int pin, bool echo_pin) {
	int t;
	if(pin < 0 || pin > 19) { //Works for trigger pins as well
		return false;
	}
	if(echo_pin) {
		for(t = 0; t < ROW_SIZE(invalid_echo_pins); t++) {
			if(pin == invalid_echo_pins[t]) {
				return false;
			}
		}
	}
	return true;
}

void cleanup_gpio(struct kbuf_dev* kbuf_devp) {
	int k, read_value;

	if(kbuf_devp->is_irq_requested) {
		free_irq(gpio_to_irq(gpio_mapping_array[kbuf_devp->echo_pin][0]), kbuf_devp);
		printk(KERN_ALERT "cleanup_gpio: dev=%s free_irq on gpio%d\n", kbuf_devp->name, gpio_mapping_array[kbuf_devp->echo_pin][0]);
		kbuf_devp->is_irq_requested = false;
	}
	
	if(kbuf_devp->is_gpio_requested) {
		for (k = 0; k < COLUMN_SIZE(gpio_mapping_array); k++) {
			read_value = gpio_mapping_array[kbuf_devp->trigger_pin][k];
		    if(read_value != -1) {
				gpio_free(read_value);
				printk(KERN_ALERT "cleanup_gpio: dev=%s gpio_free gpio%d\n", kbuf_devp->name, read_value);
//				printk(KERN_ALERT "Gpio free %d\n", read_value);
			}
			read_value = gpio_mapping_array[kbuf_devp->echo_pin][k];
		    if(read_value != -1) {
				gpio_free(read_value);
				printk(KERN_ALERT "cleanup_gpio: dev=%s gpio_free gpio%d\n", kbuf_devp->name, read_value);
//				printk(KERN_ALERT "Gpio free %d\n", read_value);
			}
			read_value = gpio_mapping_array[kbuf_devp->cs_pin][k];
		    if(read_value != -1) {
				gpio_free(read_value);
				printk(KERN_ALERT "cleanup_gpio: dev=%s gpio_free gpio%d\n", kbuf_devp->name, read_value);
//				printk(KERN_ALERT "Gpio free %d\n", read_value);
			}
			read_value = gpio_mapping_array[kbuf_devp->mosi_pin][k];
		    if(read_value != -1) {
				gpio_free(read_value);
				printk(KERN_ALERT "cleanup_gpio: dev=%s gpio_free gpio%d\n", kbuf_devp->name, read_value);
//				printk(KERN_ALERT "Gpio free %d\n", read_value);
			}
			read_value = gpio_mapping_array[kbuf_devp->sck_pin][k];
		    if(read_value != -1) {
				gpio_free(read_value);
				printk(KERN_ALERT "cleanup_gpio: dev=%s gpio_free gpio%d\n", kbuf_devp->name, read_value);
//				printk(KERN_ALERT "Gpio free %d\n", read_value);
			}
		}
		kbuf_devp->is_gpio_requested = false;
	}
}


//inline void send_trigger(struct kbuf_dev* kbuf_devp);

void send_trigger(struct kbuf_dev* kbuf_devp) {
	int i;
	gpio_set_value_cansleep(gpio_mapping_array[kbuf_devp->trigger_pin][0], 0);
	for(i = 0; i < (kbuf_devp->m_samples+2); i++) {
		printk(KERN_ALERT "send_trigger: dev=%s %d trigger(s) on gpio%d\n", kbuf_devp->name, ((kbuf_devp->m_samples)+2), gpio_mapping_array[kbuf_devp->trigger_pin][0]);
		gpio_set_value_cansleep(gpio_mapping_array[kbuf_devp->trigger_pin][0], 1);
		udelay(20);
		gpio_set_value_cansleep(gpio_mapping_array[kbuf_devp->trigger_pin][0], 0);
		msleep(kbuf_devp->trigger_interval); //Trigger seperation
	}
	printk(KERN_ALERT "send_trigger: dev=%s Done triggering on gpio%d\n", kbuf_devp->name, gpio_mapping_array[kbuf_devp->trigger_pin][0]);
}

int thread_function(void *data) {
	struct kbuf_dev* kbuf_devp = data;
	printk(KERN_ALERT "thread_function: dev=%p %s\n", kbuf_devp, kbuf_devp->name);
//	while(!kthread_should_stop()) {
		send_trigger(kbuf_devp);
//	}
	
	return 0;
}

void start_measurement(struct kbuf_dev* kbuf_devp) {
	printk(KERN_NOTICE "start_measurement: struct is at %p\n", kbuf_devp);
	//Start the measurement
	kbuf_devp->is_measuring = true;
	printk(KERN_ALERT "start_measurement: dev=%s kthread_run\n", kbuf_devp->name);
	kbuf_devp->hcsr_kthread_task = kthread_run(&thread_function, kbuf_devp, kbuf_devp->name);
}


int configure_irq(struct kbuf_dev* kbuf_devp, bool rising) {	
	int ret = gpio_to_irq(gpio_mapping_array[kbuf_devp->echo_pin][0]);
	printk(KERN_ALERT "configure_irq: dev=%s gpio_to_irq %d gpio%d\n", kbuf_devp->name, ret, gpio_mapping_array[kbuf_devp->echo_pin][0]);
	if(ret < 0) {
		return ret;
	}
	if(kbuf_devp->is_irq_requested) {
		free_irq(ret, kbuf_devp);
		kbuf_devp->is_irq_requested = false;
	}
//	printk(KERN_ALERT "Calling request_irq w/ %d %d %s %p", ret,  IRQ_TYPE_EDGE_RISING, kbuf_devp->name, (void*) kbuf_devp);

	if(rising) {
		ret = request_irq(ret, irq_handler,  /*IRQF_TRIGGER_RISING*/ IRQ_TYPE_EDGE_RISING, kbuf_devp->name, (void*) kbuf_devp);
		if(ret >= 0)
			kbuf_devp->is_irq_requested = true;
		
		printk(KERN_ALERT "configure_irq: dev=%s request_irq (rising) %d\n", kbuf_devp->name, ret);
	}
	else {
		ret = request_irq(ret, irq_handler,  /*IRQF_TRIGGER_FALLING*/ IRQ_TYPE_EDGE_FALLING, kbuf_devp->name, (void*) kbuf_devp);
		printk(KERN_ALERT "configure_irq: dev=%s request_irq (falling) %d\n", kbuf_devp->name, ret);
	}
	return ret;
}

void work_handler(struct work_struct *data) {
	struct kbuf_dev* kbuf_devp = container_of(data, struct kbuf_dev, irq_work);
	uint64_t distance_cm = div_u64((kbuf_devp->time2 - kbuf_devp->time1) * 170, 4000000);
	printk(KERN_ALERT "work_handler: dev=%s distance (cm) %llu\n", kbuf_devp->name, distance_cm);
	
	//Populate the temp buffer
	if(kbuf_devp->current_sample_idx < (kbuf_devp->m_samples+2)) {
		kbuf_devp->temp_buf[kbuf_devp->current_sample_idx++] = distance_cm;
	}
	
	if(kbuf_devp->current_sample_idx == (kbuf_devp->m_samples+2)) {
		//remove outliers, take avg, store in fifo buffer
		int i;
		uint64_t sum = 0, average;
		int min = kbuf_devp->temp_buf[0];
		int max = kbuf_devp->temp_buf[0];
		int min_idx = 0;
		int max_idx = 0;
		
		kbuf_devp->current_sample_idx = 0; //Reset the temp buffer for next samples
		
		for(i = 0; i < kbuf_devp->m_samples+2; i++) {
			if(kbuf_devp->temp_buf[i] < min) {
				min = kbuf_devp->temp_buf[i];
				min_idx = i;
			}
			if(kbuf_devp->temp_buf[i] > max) {
				max = kbuf_devp->temp_buf[i];
				max_idx = i;
			}
		}
		printk(KERN_ALERT "work_handler: dev=%s outliers %llu %llu\n", kbuf_devp->name, kbuf_devp->temp_buf[min_idx], kbuf_devp->temp_buf[max_idx]);
		kbuf_devp->temp_buf[min_idx] = 0;
		kbuf_devp->temp_buf[max_idx] = 0;
		
		for(i = 0; i < kbuf_devp->m_samples+2; i++) {
			sum += kbuf_devp->temp_buf[i];
		}
		average = div_u64(sum, kbuf_devp->m_samples);
		kbuf_devp->last_distance = average;
		printk(KERN_ALERT "work_handler: dev=%s average (cm) %llu\n", kbuf_devp->name, average);
		
		
		if(kbuf_devp->current_fifo_write_idx == FIFO_SIZE) {
			printk(KERN_ALERT "dev=%s FIFO wrap around\n", kbuf_devp->name);
			kbuf_devp->current_fifo_write_idx = 0; //Wrap around
		}
	
		//Write to FIFO
/*		if(kbuf_devp->hcsr_data[kbuf_devp->current_fifo_write_idx].is_valid) {
			printk(KERN_ALERT "dev=%s FIFO overwrite at %d. New read_idx=%d\n", kbuf_devp->name, kbuf_devp->current_fifo_write_idx, ++kbuf_devp->current_fifo_read_idx); //Adjust the read index to point to the oldest data in the FIFO
		}*/

		printk(KERN_ALERT "work_handler: dev=%s FIFO entry write_idx=%d value=%llu", kbuf_devp->name, kbuf_devp->current_fifo_write_idx, average);
		kbuf_devp->hcsr_data[kbuf_devp->current_fifo_write_idx].data = average;
		kbuf_devp->hcsr_data[kbuf_devp->current_fifo_write_idx].timestamp = rdtsc();
		kbuf_devp->hcsr_data[kbuf_devp->current_fifo_write_idx++].is_valid = true;

		if(kbuf_devp->current_fifo_length < FIFO_SIZE) {
			kbuf_devp->current_fifo_length++; //This param indicates if buffer is full
		} else {
			printk(KERN_ALERT "work_handler: dev=%s FIFO full, overwritten. oldest_data_idx=%d", kbuf_devp->name, kbuf_devp->current_fifo_read_idx);
		}

		kbuf_devp->is_measuring = false;

		printk(KERN_ALERT "work_handler: dev=%s Measurement complete sem=%d\n", kbuf_devp->name, kbuf_devp->signal_lock.count);
		
		send_msg_to_user(kbuf_devp);
		return;
	}
}

void send_msg_to_user(struct kbuf_dev* kbuf_devp) {
    void *hdr;
    int res, flags = GFP_ATOMIC;
    struct sk_buff* skb = genlmsg_new(NLMSG_DEFAULT_SIZE, flags);
	struct netlink_userdata* userdata = kzalloc(sizeof(struct netlink_userdata), GFP_KERNEL);
	userdata->msg = MSG_READ_DISTANCE;
	userdata->distance_cm = kbuf_devp->last_distance;
		
    if (!skb) {
        printk(KERN_ERR "%d: OOM!!", __LINE__);
        return;
    }

    hdr = genlmsg_put(skb, 0, 0, &genl_test_family, flags, GENL_TEST_C_MSG);
    if (!hdr) {
        printk(KERN_ERR "%d: Unknown err !", __LINE__);
        goto nlmsg_fail;
    }

	res = nla_put(skb, GENL_TEST_ATTR_MSG, sizeof(struct netlink_userdata), userdata);
    if (res) {
        printk(KERN_ERR "%d: err %d ", __LINE__, res);
        goto nlmsg_fail;
    }

    genlmsg_end(skb, hdr);
    genlmsg_multicast(&genl_test_family, skb, 0, 0, flags);
    return;

nlmsg_fail:
    genlmsg_cancel(skb, hdr);
    nlmsg_free(skb);
    return;
}

irqreturn_t irq_handler(int irq, void *dev_id) {
	struct kbuf_dev* kbuf_devp = dev_id; //Retrieve our per device struct
	if(!kbuf_devp->irq_type_flag) {
		kbuf_devp->time1 = rdtsc();
		irq_set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);
		kbuf_devp->irq_type_flag = true;
//		printk(KERN_ALERT "rising\n");
	} else {
		kbuf_devp->time2 = rdtsc();
		schedule_work(&kbuf_devp->irq_work);

		irq_set_irq_type(irq, IRQ_TYPE_EDGE_RISING);
		kbuf_devp->irq_type_flag = false;
		printk(KERN_ALERT "irq_handler: (falling) dev=%s count %d\n", kbuf_devp->name, ++kbuf_devp->irq_count);
	}
	return IRQ_HANDLED;
}

int spi_transfer(struct kbuf_dev* kbuf_devp)
{          //transfer data to be displayed on matrix
	int retval;
	struct spi_message message;
	struct spi_transfer *msg_buf;

	msg_buf = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);
	spi_message_init(&message);

	msg_buf->len = kbuf_devp->spi_msg_len;
//	msg_buf->speed_hz = kbuf_devp->spi_speed_hz; //Use default
	msg_buf->bits_per_word = 8;
/*	msg_buf->speed_hz = 10000000;
	msg_buf->delay_usecs = 1;
	msg_buf->cs_change = 1;
*/
	msg_buf->tx_buf = kbuf_devp->spi_tx_buff;
//	printk("spi_transfer: dev=%s Writing %x %x\n", kbuf_devp->name, kbuf_devp->spi_tx_buff[0], kbuf_devp->spi_tx_buff[1]);

	spi_message_add_tail(msg_buf, &message);
	gpio_set_value(spi_mapping_array[kbuf_devp->cs_pin][0], 0);
	retval=spi_sync(kbuf_devp->spi, &message);
	gpio_set_value(spi_mapping_array[kbuf_devp->cs_pin][0],1);

//	printk("spi_transfer: spi_sync done dev=%s ret=%d\n", kbuf_devp->name, retval);
	kfree(msg_buf);
	return retval;
}

void configure_display(struct kbuf_dev* kbuf_devp) {
	int clr;
	
	for(clr = 0; clr < ROW_SIZE(display_config); clr+=2) {
		kbuf_devp->spi_tx_buff[0] = display_config[clr];
		kbuf_devp->spi_tx_buff[1] = display_config[clr+1];
		spi_transfer(kbuf_devp);
	}
	//Clear all LEDs
	for(clr=0x00; clr < 0x09; clr++)
	{
		kbuf_devp->spi_tx_buff[0] = clr;
		kbuf_devp->spi_tx_buff[1] = 0xFF;
		spi_transfer(kbuf_devp);
	}
	printk(KERN_INFO"configure_display: dev=%s led matrix configured successfully\n", kbuf_devp->name);
}


bool check_all_params(struct kbuf_dev* kbuf_devp) {
	if((kbuf_devp->trigger_pin == kbuf_devp->echo_pin) || !is_valid_pin(kbuf_devp->trigger_pin, false)
		|| !is_valid_pin(kbuf_devp->echo_pin, true) || !is_valid_pin(kbuf_devp->cs_pin, false)
	    || !is_valid_spi_pin(kbuf_devp->mosi_pin) || !is_valid_spi_pin(kbuf_devp->sck_pin)) {
		printk(KERN_ALERT "check_all_params: failed dev=%s Invalid pins in the input\n", kbuf_devp->name);
		return false;
	}
	if(kbuf_devp->m_samples <= 0 || kbuf_devp->delta <= 0) {
		printk(KERN_ALERT "check_all_params: failed dev=%s m and/or delta value(s) cannot be <= 0\n", kbuf_devp->name);
		return false;
	}
	kbuf_devp->trigger_interval = div_u64(kbuf_devp->delta, (kbuf_devp->m_samples+2));
	if(kbuf_devp->trigger_interval < 60) {
		kbuf_devp->trigger_interval = 0;
		printk(KERN_ALERT "check_all_params: failed dev=%s delta/(m+2) < 60 is not allowed\n", kbuf_devp->name);
		return false;
	}
	return true;
}

void check_and_set_params(struct kbuf_dev* kbuf_devp) {
	if(!check_all_params(kbuf_devp)) {
		printk(KERN_ALERT "check_and_set_params: failed dev=%s Set all params before enabling measurement\n", kbuf_devp->name);
		return;
	}
	if(kbuf_devp->is_gpio_requested) {
		cleanup_gpio(kbuf_devp);
	}
	configure_pins(kbuf_devp, kbuf_devp->trigger_pin, true); //trigger pin
	configure_pins(kbuf_devp, kbuf_devp->echo_pin, false); //echo pin
//	configure_pins(kbuf_devp, kbuf_devp->cs_pin, true); //chip select
//	configure_pins(kbuf_devp, kbuf_devp->mosi_pin, true); //mosi
//	configure_pins(kbuf_devp, kbuf_devp->sck_pin, true); //sck
	
	configure_spi_pins(kbuf_devp, kbuf_devp->mosi_pin); //mosi	
	configure_spi_pins(kbuf_devp, kbuf_devp->cs_pin); //cs
	configure_spi_pins(kbuf_devp, kbuf_devp->sck_pin); //sck

	kbuf_devp->is_gpio_requested = true; //Led's have been configured

	INIT_WORK(&kbuf_devp->irq_work, work_handler); //Initialize an interrupt bottom half for calculating distance
	configure_irq(kbuf_devp, true); //Setup irq	
	kbuf_devp->temp_buf = (uint64_t*) kzalloc((kbuf_devp->m_samples+2) * sizeof(uint64_t), GFP_KERNEL); //Allocate a FIFO

	//SPI
	kbuf_devp->spi_msg_len = 2; //Default
	kbuf_devp->spi_tx_buff = (uint8_t*) kzalloc(kbuf_devp->spi_msg_len * sizeof(uint8_t), GFP_KERNEL); //Allocate a FIFO
	configure_display(kbuf_devp);
}

void check_and_begin_measurement(struct kbuf_dev* kbuf_devp) {
	if(!check_all_params(kbuf_devp)) {
		printk(KERN_ALERT "check_and_begin_measurement: failed dev=%s Set all params before enabling measurement\n", kbuf_devp->name);
		return;
	}
	//Measurement stops automatically after collecting m+2 samples
	printk("check_and_begin_measurement: dev=%s enable=%d is_measuring=%d\n", kbuf_devp->name, kbuf_devp->enable, kbuf_devp->is_measuring);
	if(kbuf_devp->enable && !kbuf_devp->is_measuring) {
		start_measurement(kbuf_devp);
	}
}

void spi_clear_pattern(struct kbuf_dev* kbuf_devp) {
	int i;
	for(i = 0x00; i < 0x09; i++) {
		kbuf_devp->spi_tx_buff[0] = i;
		kbuf_devp->spi_tx_buff[1] = 0x00;			
		spi_transfer(kbuf_devp);
	}
}

void spi_send_leds(struct led_rep* led_struct) {
	int j;
	for(j = 0; j < LED_CONTROL_PATTERN_SIZE; j+=2) {
		kbuf_devp->spi_tx_buff[0] = led_struct->led[j];
		kbuf_devp->spi_tx_buff[1] = led_struct->led[j+1];			
		spi_transfer(kbuf_devp);
	}
}

int display_thread(void *data) {
	struct kbuf_dev* kbuf_devp = data;
	if(!kbuf_devp) {
		return -1;
	}
	printk(KERN_ALERT "display_thread: dev=%p %s\n", kbuf_devp, kbuf_devp->name);
	while(!kthread_should_stop()) {
		update_pattern(kbuf_devp);
	}
	spi_clear_pattern(kbuf_devp);
	kbuf_devp->display_has_exited = true;
	return 0;
}

void start_or_update_display(struct kbuf_dev* kbuf_devp) {
	if(kbuf_devp->display_has_exited) {
		printk(KERN_ALERT "start_display: dev=%s kthread_run\n", kbuf_devp->name);	
		kbuf_devp->spi_kthread_task = kthread_run(&display_thread, kbuf_devp, kbuf_devp->name);
	} else {
		if(kbuf_devp->spi_kthread_task) {
			kthread_stop(kbuf_devp->spi_kthread_task);
			msleep(10);
		}
		kbuf_devp->spi_kthread_task = kthread_run(&display_thread, kbuf_devp, kbuf_devp->name);		
	}
}

void update_pattern(struct kbuf_dev* kbuf_devp) {	
	int i;
	switch(kbuf_devp->pattern_dir) {
		case PATTERN_DIR_FORWARD:
			for(i = 0; i < PATTERN_ARRAY_SIZE; i++) {
				spi_send_leds(&kbuf_devp->pattern_array[i]);
				udelay(100 * kbuf_devp->last_distance); //Change pattern speed with most recent distance value
			}				
			break;
			
		case PATTERN_DIR_REVERSE:
			for(i = PATTERN_ARRAY_SIZE-1; i >= 0; i++) {
				spi_send_leds(&kbuf_devp->pattern_array[i]);
				udelay(100 * kbuf_devp->last_distance);
			}
			break;
			
		case PATTERN_DIR_TOO_CLOSE:
			for(i = 0x08; i >= 0x00; i--) {
				kbuf_devp->spi_tx_buff[0] = i;
				kbuf_devp->spi_tx_buff[1] = 0x00;
				spi_transfer(kbuf_devp);
				udelay(100 * kbuf_devp->last_distance);
			}
			break;			
	}
}

static int genl_test_rx_msg(struct sk_buff* skb, struct genl_info* info)
{
	struct netlink_userdata* userdata;
    if (!info->attrs[GENL_TEST_ATTR_MSG]) {
        printk(KERN_ERR "empty message from %d!!\n",
            info->snd_portid);
        printk(KERN_ERR "%p\n", info->attrs[GENL_TEST_ATTR_MSG]);
        return -EINVAL;
    }
	userdata = nla_data(info->attrs[GENL_TEST_ATTR_MSG]);

	if(!userdata) {
		printk(KERN_ERR "Received null message\n");
	}

	printk(KERN_NOTICE "Portid %u received msg type=%d\n", info->snd_portid, userdata->msg);

	switch(userdata->msg) {
		case MSG_CONFIG_PINS:
			printk(KERN_ALERT "MSG_CONFIG_PINS\n");
			kbuf_devp->trigger_pin = userdata->trigger_pin;
			kbuf_devp->echo_pin = userdata->echo_pin;
			kbuf_devp->cs_pin = userdata->cs_pin;
			kbuf_devp->mosi_pin = userdata->mosi_pin;
			kbuf_devp->sck_pin = userdata->sck_pin;
			kbuf_devp->m_samples = userdata->m_samples;
			kbuf_devp->delta = userdata->delta;
			memcpy(kbuf_devp->pattern_array, userdata->pattern_array, sizeof(struct led_rep) * 10);
			//setup gpios and irq
			check_and_set_params(kbuf_devp);
			break;
		
		case MSG_CONFIG_PATTERN:
			printk(KERN_ALERT "MSG_CONFIG_PATTERN\n");
			kbuf_devp->pattern_dir = userdata->pattern_dir;
			//Call display update function
			start_or_update_display(kbuf_devp);
			break;
			
		case MSG_START_MEASUREMENT:
			printk(KERN_ALERT "MSG_START_MEASUREMENT\n");
			kbuf_devp->enable = userdata->enable;
			//Call state update function
			check_and_begin_measurement(kbuf_devp);
			break;
		
		default:
			return -EINVAL;
			break;
	}
    return 0;
}

static const struct genl_ops genl_test_ops[] = {
    {
        .cmd = GENL_TEST_C_MSG,
        .policy = genl_test_policy,
        .doit = genl_test_rx_msg,
        .dumpit = NULL,
    },
};

static const struct genl_multicast_group genl_test_mcgrps[] = {
    [GENL_TEST_MCGRP0] = { .name = GENL_TEST_MCGRP0_NAME, },
    [GENL_TEST_MCGRP1] = { .name = GENL_TEST_MCGRP1_NAME, },
    [GENL_TEST_MCGRP2] = { .name = GENL_TEST_MCGRP2_NAME, },
};

static struct genl_family genl_test_family = {
    .name = GENL_TEST_FAMILY_NAME,
    .version = 1,
    .maxattr = GENL_TEST_ATTR_MAX,
    .netnsok = false,
    .module = THIS_MODULE,
    .ops = genl_test_ops,
    .n_ops = ARRAY_SIZE(genl_test_ops),
    .mcgrps = genl_test_mcgrps,
    .n_mcgrps = ARRAY_SIZE(genl_test_mcgrps),
};

static int __init genl_test_init(void)
{
	int status, i;
	struct spi_master* master;
	struct spi_device* spidevice;
	struct spi_board_info spidev_info;
	char* hcsr_name;
	
	/* Allocate memory for the per-device structure */
	kbuf_devp = kzalloc(sizeof(struct kbuf_dev) * MAX_DEVICES, GFP_KERNEL);
		
	if (!kbuf_devp) {
		printk("Bad Kmalloc\n"); return -ENOMEM;
	}

	for(i=0; i < MAX_DEVICES; i++) {
		hcsr_name = kzalloc(10 * sizeof(char), GFP_KERNEL);
		sprintf(hcsr_name,"hcsr_%d",i);
 		kbuf_devp[i].name = hcsr_name; //Name for logs		

		memset(&spidev_info, 0, sizeof(struct spi_board_info));
		memcpy(spidev_info.modalias, "spidev", 6*sizeof(char));
		spidev_info.max_speed_hz		= 1000000;
		spidev_info.bus_num		= 1;
		spidev_info.chip_select		= i+1; //Since 0 is already used on galileo
		spidev_info.mode			= SPI_MODE_0;
		spidev_info.platform_data		= &kbuf_devp[i];

		//spi device
		master = spi_busnum_to_master(1);
		if(!master) {
			printk(KERN_ALERT "No master registered on bus 1\n");
			continue;
		}
		spidevice = spi_new_device(master, &spidev_info);
		status = PTR_ERR_OR_ZERO(spidevice);
		if(status != 0) {
			continue;
		}
		kbuf_devp[i].spi = spidevice;

		printk(KERN_ALERT "spi_new_device: registered status %d\n", status);
	}
	
	printk(KERN_INFO "hcsr driver initialized\n");
    printk(KERN_INFO "genl_test: initializing netlink\n");

    status = genl_register_family(&genl_test_family);
    if (status) {
		printk(KERN_DEBUG "error occurred in %s\n", __func__);
		return -EINVAL;		
	}

/*#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,14,0)
    init_timer(&timer);
    timer.data = 0;
    timer.function = genl_test_periodic;
    timer.expires = jiffies + msecs_to_jiffies(GENL_TEST_HELLO_INTERVAL);
    add_timer(&timer);
#else
    timer_setup(&timer, genl_test_periodic, 0);
    mod_timer(&timer, jiffies + msecs_to_jiffies(GENL_TEST_HELLO_INTERVAL));
#endif
*/
    return 0;
}
module_init(genl_test_init);

static void genl_test_exit(void)
{
	int i;
//    del_timer(&timer);
    genl_unregister_family(&genl_test_family);

	if(kbuf_devp->spi_kthread_task) {
		kthread_stop(kbuf_devp->spi_kthread_task);
	}
	spi_clear_pattern(kbuf_devp);
	if(kbuf_devp->spi)
		spi_unregister_device(kbuf_devp->spi);

	for(i=0; i < MAX_DEVICES; i++) {
		kbuf_devp[i].is_gpio_requested = true;
		cleanup_gpio(&kbuf_devp[i]);
	}
	
	kfree(kbuf_devp->temp_buf);
	kfree(kbuf_devp->spi_tx_buff);
	kfree(kbuf_devp->name);
	kfree(kbuf_devp);
}
module_exit(genl_test_exit);

MODULE_LICENSE("GPL");