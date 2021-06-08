/**
 ******************************************************************************
 * @file    em125-reader.c
 * @author  Vitor Gomes <vitor.gomes@csgd.com.br>
 * @version V0.0.0
 * @date    2021-05-20
 * @brief   Magnetic Reader Linux Kernel module using GPIO interrupts.
 ******************************************************************************
 * @attention
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 ******************************************************************************
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <asm/dma.h>
#include <asm/io.h>
#include "em125-reader.h"
#include "systime.h"


static int em125_minor = 0;
static DECLARE_WAIT_QUEUE_HEAD(get_id_waitq);
static volatile int ev_get_id = 0;

static int cnt = 0;
static int v1, v2;
static struct timespec start_uptime;  
static int start_here = 0;
static unsigned long used_time;
static int bits = 0;
static uint64_t my_pl_card_id;
static uint64_t real_card_id;

#define IO_IDCARD_IN	0x16
#define A33_PL4    356
#define A33_PL4_IRQ 	(__gpio_to_irq(A33_PL4))
#define BELOW1BIT	128		//128us (128*1us)
#define ABOVE1BIT	384		//384us (384*1us)
#define ABOVE2BITS	640		//640us (640*1us)
#define ERRORCOUNTS 20

static unsigned char RFIDBuf128[16];
static unsigned char RFIDDecodeOK;
static unsigned char RFIDCustomerID;
static unsigned long RFIDLastCardID;
static unsigned char const RFIDMask[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
static unsigned char tmp_bit1;


static unsigned char rfid_read_bit(void)
{
	return gpio_get_value(A33_PL4)? 1: 0;
}

static unsigned char _crotl__(u8 value,u8 count)
{
	unsigned int buff,buff1;
	unsigned char a,b;
	buff1 = value;
	buff = buff1 << count;

	a = buff & 0xff;
	b = buff >> 8;
	//pr_info("%x,%x,%x,%x\n",buff1,buff,a,b);
	return a | b;
}

static void RFIDBuf64Shift(void)
{
	unsigned char d,i;
	
	d = (RFIDBuf128[0]>>7) & 0x01;
	for(i=0;i<8;i++)
		RFIDBuf128[i] = _crotl__(RFIDBuf128[i],1);
	for(i=0;i<7;i++)
		RFIDBuf128[i] = (RFIDBuf128[i] & 0xFE) | (RFIDBuf128[i+1] & 0x01);
	RFIDBuf128[7] = (RFIDBuf128[7] & 0xFE) | d;
}

static unsigned char RFIDParityCheck(void)
{
	unsigned char i,t1,t2;
	unsigned char bits;
	unsigned char ok;
	
	for(i=0;i<64;i++)
	{
		ok = 1;
		if ( (RFIDBuf128[0]!=0xFF) || ((RFIDBuf128[1]&0x80)!=0x80) || (RFIDBuf128[7] & 0x01) )
		{
			ok = 0;
			RFIDBuf64Shift();
			continue;
		}
		for(t1=0;t1<10;t1++)	//10 rows parity check
		{
			bits = t1*5+9;
			tmp_bit1 = 0;
			for(t2=0;t2<5;t2++)
			{
				if ( RFIDBuf128[(bits+t2)>>3] & RFIDMask[(bits+t2)&0x07] ) tmp_bit1 = ~tmp_bit1;
			}
			if ( tmp_bit1 )
			{
				ok = 0;
				break;
			}
		}
		if ( !ok )
		{
			RFIDBuf64Shift();
			continue;
		}
		
		for(t1=0;t1<4;t1++)	//4 columns parity check
		{
			bits = t1+9;
			tmp_bit1 = 0;
			for(t2=bits;t2<bits+55;t2+=5)
			{
				if ( RFIDBuf128[t2>>3] & RFIDMask[t2&0x07] ) tmp_bit1 = ~tmp_bit1;
			}
			if ( tmp_bit1 )
			{
				ok = 0;
				break;
			}
		}
		if ( !ok )
		{
			RFIDBuf64Shift();
			continue;
		}
		ok = 1;
		break;
	}
	return ok;
}
	
static unsigned long get_time_use(void)
{
        struct timespec uptime, temp;  
        getrawmonotonic(&uptime);  	
        if ((uptime.tv_nsec-start_uptime.tv_nsec)<0) {
		temp.tv_sec = uptime.tv_sec-start_uptime.tv_sec-1;
		temp.tv_nsec = 1000000000+uptime.tv_nsec-start_uptime.tv_nsec;
	} else {
		temp.tv_sec = uptime.tv_sec-start_uptime.tv_sec;
	 	temp.tv_nsec = uptime.tv_nsec-start_uptime.tv_nsec;
	}
	return (temp.tv_sec * 1000000000 + temp.tv_nsec)/1000;  /* unit us */
}

int get_start_action(void)
{
    int ret = -1;
    if(cnt == 0){
         v1 = rfid_read_bit();
    }else{
        v2 = rfid_read_bit();
    }

    cnt++;

    if(cnt == 2){
        cnt = 0;
        if(v1 != v2){
            ret = 0;
        }else{
            ret =  -1;
        }
    }
    return ret;
}


static void start_to_decode(struct s_em125_driver * p)
{
    unsigned char t1,t2;

    if(start_here){
        used_time = get_time_use();
        if(used_time > ABOVE2BITS){
            RFIDDecodeOK = 0;
            start_here = 0;
            bits = 0;
        }else{
            if ( used_time < BELOW1BIT ){
                RFIDDecodeOK = 0;
                start_here = 0;
                bits = 0;
            }

            if(rfid_read_bit()){
                if ( used_time>=ABOVE1BIT ){
                    RFIDBuf128[bits>>3] &= ~RFIDMask[bits&0x07];	//save double bits of 0
                    if(bits < 127){
                        bits++;
                    }
                }
                RFIDBuf128[bits>>3] &= ~RFIDMask[bits&0x07];		//save single bits of 0
            }else{
                if ( used_time >= ABOVE1BIT ){
                    RFIDBuf128[bits >>3] |= RFIDMask[bits & 0x07];	//save double bits of 1
                    if(bits < 127){
                        bits++;
                    }
                } 
                RFIDBuf128[bits>>3] |= RFIDMask[bits&0x07];	//save single bit of 1
            }
            bits++;
        }

        if(bits == 128){
            for(bits=0;bits<64;bits++){
                if ( RFIDBuf128[bits>>2] & RFIDMask[(bits<<1)&0x07] )
                    RFIDBuf128[bits>>3] |= RFIDMask[bits&0x07];
                else
                    RFIDBuf128[bits>>3] &= ~RFIDMask[bits&0x07];
            }
            RFIDDecodeOK = 1;
        }

        if ( RFIDDecodeOK && !RFIDParityCheck() ){
            for(t1=0;t1<10;t1++) 
                RFIDBuf128[t1] = ~RFIDBuf128[t1];	//to invert the data bits
            if ( !RFIDParityCheck() ) {
                //pr_info("RFIDParityCheck   error\n");
                RFIDDecodeOK = 0;
                start_here = 0;
                bits = 0;
            }
        }

        if(RFIDDecodeOK){
            for(t1=0;t1<7;t1++)	{               //to remove start bits (9 bits)
                RFIDBuf128[t1] = RFIDBuf128[t1+1];
            }

            RFIDBuf64Shift();
		
            RFIDCustomerID = RFIDBuf128[0] & 0xF0;	//to get customer ID
            for(t1=0;t1<5;t1++) 
			    RFIDBuf64Shift();

            RFIDCustomerID = RFIDCustomerID | (RFIDBuf128[0]>>4);
            
            my_pl_card_id = 0L;			//to get Card ID
            for(t1=0;t1<8;t1++){
                for(t2=0;t2<5;t2++) 
                    RFIDBuf64Shift();
                my_pl_card_id = (my_pl_card_id <<4) | (RFIDBuf128[0]>>4);
            }
		
            if ( my_pl_card_id == RFIDLastCardID ){     
                real_card_id = ((unsigned long) RFIDCustomerID * 0x100000000) + my_pl_card_id;

                //pr_info("real_card_id : %llu\n", real_card_id);
                sprintf(p->info.buffer, "%014llu", real_card_id);

                ev_get_id = 1;
                wake_up_interruptible(&get_id_waitq);  
                start_here = 0;
                bits = 0;
                RFIDDecodeOK = 0;
            }else{
                RFIDLastCardID = my_pl_card_id;
                RFIDDecodeOK = 0;
                start_here = 0;
                bits = 0;
            }
        }
        getrawmonotonic(&start_uptime);  	
    }
}

/**
 * @brief The interrupt service routine called on data
 * @param irq   : Interrupt value that was fired
 * @param data  : Data pointer to this driver interrupt
 * @return \ref irqreturn_t
 */
static irqreturn_t em125_reader_data_irq(int irq, void *data) {
    struct s_em125_driver * p = (struct s_em125_driver *) data;
    
    if(start_here == 0){
        p->info.in_progress = true;
        p->info.pulses++;
        if(0 == get_start_action()){
            start_here = 1;
            bits = 0;
            getrawmonotonic(&start_uptime);  	
            return IRQ_RETVAL(IRQ_HANDLED);
        }
    }

    start_to_decode(p);
    systime_start(p->info.stamp);

	return IRQ_RETVAL(IRQ_HANDLED);
}

/**
 * \brief Remove data of the device driver
 * @param pdev  : plataform device driver
 */
void em125_reader_destroy(struct platform_device *pdev) {
    s_em125_driver_t * em125 = platform_get_drvdata(pdev);

    if (em125 != NULL) {
        em125_setup_remove(em125);
        if (em125->data_irq > 0)
            free_irq(em125->data_irq, em125);
        if (em125->dev)
            device_unregister(em125->dev);

        devm_kfree(&pdev->dev, em125);
    }
}

/**
 * \brief Create data of the device driver
 * @param pdev  : plataform device driver
 */
int em125_reader_create(struct platform_device *pdev, struct class * drv_class) {
    int err;
    s_em125_driver_t * em125 = NULL;
    struct device_node *np = NULL;
    struct pwm_state state;
    struct device *dev = &pdev->dev;

    //*****************************************************************************************************************
    em125 = devm_kzalloc(&pdev->dev, sizeof(*em125), GFP_KERNEL);
    if (em125 == NULL)
        return -ENOMEM;

    em125->name = NULL;
    em125->dev = NULL;
    em125->minor = em125_minor;
    em125->data_irq = -1;
    em125->info.pulses = 0;
    em125->info.in_progress = 0;
    memset(em125->info.buffer, 0, EM125READER_BUF_SIZE);
    spin_lock_init(&em125->spinlock);

    platform_set_drvdata(pdev, em125);

    // Load label name from device tree
    np = of_node_get(pdev->dev.of_node);

    // Determine label to char device
    err = of_property_read_string_index(np, "label", 0, &em125->name);

    if (err < 0) {
        em125->name = pdev->dev.of_node->name;
    }

    pr_info(" node: %s\n", em125->name);

    // Set data interrupt
    em125->data_irq = platform_get_irq_byname(pdev, "data");

    if (em125->data_irq < 0) {
        pr_err(" data: invalid IRQ %d\n", em125->data_irq);
        return -EIO;
    }

    err = request_irq(em125->data_irq,
                      em125_reader_data_irq,
                      (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
                      "em125_data",
                      em125);

    if (err < 0) {
        em125->data_irq = -1;
        pr_err(" data: cannot register IRQ %d\n", em125->data_irq);
        return -EIO;
    }

    // create device driver to system class
    em125->dev = device_create(drv_class,
                             NULL,
                             MKDEV(0, 0),
                             em125,
                             "%s%d",
                             "em125-in",
                             em125->minor);

    if (IS_ERR(em125->dev)) {
        dev_warn(&pdev->dev,
             "device_create failed for em125 sysfs\n");
        return -EIO;
    }

    // create sysfs group
    err = em125_setup_initializes(em125);

    if (err < 0)
        return -EIO;

    em125_minor++;

    /* Inicializar oscilação de antena 125KHz*/

    em125->antena = devm_pwm_get(dev, NULL);
	if (IS_ERR(em125->antena)) {
		err = PTR_ERR(em125->antena);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "Failed to request PWM device: %d\n",
				err);
		return err;
	}

	pwm_get_state(em125->antena, &state);

	state.enabled = true;
	state.period = 8000;
    state.duty_cycle = 4000;

    err = pwm_apply_state(em125->antena, &state);
    if (err) {
		dev_err(dev, "failed to apply initial PWM state: %d\n",
			err);
		return err;
	}

    return 0;
}
