/**
 * @file   tacho.c
 * @author Florian Scholz & Jan Sacher
 * @date   6. June 2018
 * @brief  A kernel module for measuring odometry with photoelectric sensors
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/interrupt.h>            // Required for the IRQ code
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florian Scholz & Jan Sacher");
MODULE_DESCRIPTION("A driver for the odometry measuring ");
MODULE_VERSION("0.1");

#define GPIO_HIGH 1
#define DEVICE_NAME "tachol"
#define CLASS_NAME "mvsfs"



enum TriggeredEdge { Rising = 0, Falling = 1};

static enum TriggeredEdge EdgeA = Falling;
static enum TriggeredEdge EdgeB = Falling;

static enum TriggeredEdge EdgeC = Falling;
static enum TriggeredEdge EdgeD = Falling;

static short reedState_l = 0;
static short reedState_r = 0;
static int last_l = 0;
static int last_r = 0;
static unsigned int gpioPortA = 17;	//PortA
static unsigned int gpioPortB = 27;	//portB
static unsigned int gpioPortC = 13;
static unsigned int gpioPortD = 19;

static unsigned int irqNumberA;		///< Used to share the IRQ number within this file
static unsigned int irqNumberB;
static unsigned int irqNumberC;
static unsigned int irqNumberD;

static int majorNumber;

static irq_handler_t  ulsgpio_irq_handler_l(unsigned int irq, void *dev_id, struct pt_regs *regs);
static irq_handler_t  ulsgpio_irq_handler_r(unsigned int irq, void *dev_id, struct pt_regs *regs);
static struct class*  ulscharClass  = NULL; ///< The device-driver class struct pointer
static struct device* ulscharDevice = NULL; ///< The device-driver device struct pointer
//static wait_queue_head_t wq;

//static enum Device_State deviceState = NotInitialized;

//static spinlock_t time_lock;
static int ulsgpio_open(struct inode *inodep, struct file *filep){
   return 0;
}

#if 0
static void print_ktime(const char* info, ktime_t sample) {
  struct timespec ts_last = ktime_to_timespec(sample);
  printk("%s: %ld\n",info, ts_last.tv_nsec);
}
#endif
/*
static void ulsgpio_send_trigger(void) {
	gpio_set_value(gpioTriggerPort, 1); 
	deviceState = Ready;
	usleep_range(10,11);
	gpio_set_value(gpioTriggerPort, 0);
}
*/
static ssize_t ulsgpio_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
	static char pbuf[256];
	struct timespec ts_last;
	int cnt;
	int values[2] = {reedState_l, reedState_r};
	len = sizeof(int)*2 -  copy_to_user((int*)buffer, values, min(len, sizeof(int)*2));
	//cnt = snprintf(pbuf,min(sizeof(size_t)*256, len),"%d %d\n", reedState_l, reedState_r );
	//len = cnt - copy_to_user(buffer, pbuf, cnt);
	return len;
	
}

static struct file_operations fops =
{
	.owner = THIS_MODULE,
	.open = ulsgpio_open,
	.read = ulsgpio_read
};

static int __init ulsgpio_init(void){
	int result = 0;
	//measurement_started = measurement_stopped = ktime_set(0,0);
	//spin_lock_init(&time_lock);
	printk(KERN_INFO "GPIO_TEST: Initializing the GPIO_TEST LKM\n");

	if (!gpio_is_valid(gpioPortA)){
		printk(KERN_INFO "GPIO_TEST: invalid GPIO A\n");
		return -ENODEV;
	}
	
	if (!gpio_is_valid(gpioPortB)){
		printk(KERN_INFO "GPIO_TEST: invalid GPIO B\n");
		return -ENODEV;
	}


	if (!gpio_is_valid(gpioPortC)){
		printk(KERN_INFO "GPIO_TEST: invalid GPIO C\n");
		return -ENODEV;
	}
	
	if (!gpio_is_valid(gpioPortD)){
		printk(KERN_INFO "GPIO_TEST: invalid GPIO D\n");
		return -ENODEV;
	}

	majorNumber = register_chrdev(0, DEVICE_NAME, &fops);

	if (majorNumber<0){
		printk(KERN_ALERT "ULTRASONIC: failed to register a major number\n");
		return majorNumber;
	}

	ulscharClass = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(ulscharClass)) {
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "Failed to register device class\n");
		return PTR_ERR(ulscharClass);
	}
	
	ulscharDevice = device_create(ulscharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
	if (IS_ERR(ulscharDevice)) {
		class_destroy(ulscharClass);
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "Failed to create the device\n");
		return PTR_ERR(ulscharDevice);
	}

	gpio_request(gpioPortA, "sysfs");
	gpio_direction_input(gpioPortA);
	gpio_export(gpioPortA, false);

	gpio_request(gpioPortB, "sysfs");
	gpio_direction_input(gpioPortB);
	gpio_export(gpioPortB, false);

	gpio_request(gpioPortC, "sysfs");
	gpio_direction_input(gpioPortC);
	gpio_export(gpioPortC, false);

	gpio_request(gpioPortD, "sysfs");
	gpio_direction_input(gpioPortD);
	gpio_export(gpioPortD, false);

	//init_waitqueue_head(&wq);


	irqNumberA = gpio_to_irq(gpioPortA);
	irqNumberB = gpio_to_irq(gpioPortB);
	irqNumberC = gpio_to_irq(gpioPortC);
	irqNumberD = gpio_to_irq(gpioPortD);

	result = request_irq(irqNumberA,
	(irq_handler_t) ulsgpio_irq_handler_l,
	IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	"uls_gpio_handlerl",
	NULL);


	result = request_irq(irqNumberB,
	(irq_handler_t) ulsgpio_irq_handler_l,
	IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	"uls_gpio_handlerl",
	NULL);


	result = request_irq(irqNumberC,
	(irq_handler_t) ulsgpio_irq_handler_r,
	IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	"uls_gpio_handlerr",
	NULL);


	result = request_irq(irqNumberD,
	(irq_handler_t) ulsgpio_irq_handler_r,
	IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	"uls_gpio_handlerr",
	NULL);

return result;
}


static void ulsgpio_release_gpio_port(unsigned int gpioPort) {
	gpio_unexport(gpioPort);
	gpio_free(gpioPort);;
}

static void __exit ulsgpio_exit(void) {
	 /* Removing the IRQ routine */
	free_irq(irqNumberA, 0);
	free_irq(irqNumberB, 0);
	free_irq(irqNumberC, 0);
	free_irq(irqNumberD, 0);

	ulsgpio_release_gpio_port(gpioPortA);
	ulsgpio_release_gpio_port(gpioPortB);
	ulsgpio_release_gpio_port(gpioPortC);
	ulsgpio_release_gpio_port(gpioPortD);

	/* Remove udev stuff */
	device_destroy(ulscharClass, MKDEV(majorNumber, 0));
	class_unregister(ulscharClass);
	class_destroy(ulscharClass);

	/* Remove character device */
	unregister_chrdev(majorNumber, DEVICE_NAME);
}

static irq_handler_t ulsgpio_irq_handler_l(unsigned int irq, void *dev_id, struct pt_regs *regs){
	/*int gpio_number = gpioEchoPort;*/
	if (irq == irqNumberA){
		int value = gpio_get_value(gpioPortA);
                if(value == GPIO_HIGH)
                {
			if(EdgeA == Falling) {
				EdgeA = Rising;
				if(last_l == 1)
				{
					printk("L: PortA, Rising Edge\n");
					if(EdgeB == Rising)
					{
						reedState_l --;
						printk("L: R , %d \n", reedState_l);
					}
					else
					{
						reedState_l ++;
						printk("L: V , %d \n", reedState_l);
					}
				}
				last_l = 0;
			}
                	return (irq_handler_t) IRQ_HANDLED;
                }
                else
                {
			if(EdgeA == Rising){
				EdgeA = Falling;
				if(last_l == 1)
				{
					printk("L: PortA, Falling Edge\n");
					if(EdgeB == Falling)
					{
						reedState_l --;
						printk("L: R , %d \n", reedState_l);
					}
					else
					{
						reedState_l ++;
						printk("L :V , %d \n", reedState_l);
					}
				}
				last_l = 0;
			}
                	return (irq_handler_t) IRQ_HANDLED;
                }
	}

	if (irq == irqNumberB)
	{
		int value = gpio_get_value(gpioPortB);
                if(value == GPIO_HIGH)
                {
			if(EdgeB == Falling) {
				EdgeB = Rising;
				if(last_l == 0)
				{
					printk("L: PortB, Rising Edge\n");
					if(EdgeA == Rising)
					{
						reedState_l ++;
						printk("L: V , %d \n", reedState_l);
					}
					else
					{
						reedState_l --;
						printk("L: R , %d \n", reedState_l);
					}
				}
				last_l = 1;
			}
                	return (irq_handler_t) IRQ_HANDLED;
                }
                else
                {
			if(EdgeB == Rising) {
				EdgeB = Falling;
				if(last_l == 0)
				{
					printk("L: PortB, Falling Edge\n");
					if(EdgeA == Falling)
					{
						reedState_l ++;
						printk("L: V, %d \n", reedState_l);
					}
					else
					{
						reedState_l --;
						printk("L: R, %d \n", reedState_l);
					}
				}
				last_l = 1;
			}
                	return (irq_handler_t) IRQ_HANDLED;
                }
	}
	return (irq_handler_t) IRQ_HANDLED;
}


static irq_handler_t ulsgpio_irq_handler_r(unsigned int irq, void *dev_id, struct pt_regs *regs){
	/*int gpio_number = gpioEchoPort;*/
	if (irq == irqNumberC){
		int value = gpio_get_value(gpioPortC);
                if(value == GPIO_HIGH)
                {
			if(EdgeC == Falling) {
				EdgeC = Rising;
				if(last_r == 1)
				{
					printk("R: PortC, Rising Edge\n");
					if(EdgeD == Rising)
					{
						reedState_r --;
						printk("R: R , %d \n", reedState_r);
					}
					else
					{
						reedState_r ++;
						printk("R: V , %d \n", reedState_r);
					}
				}
				last_r = 0;
			}
                	return (irq_handler_t) IRQ_HANDLED;
                }
                else
                {
			if(EdgeC == Rising){
				EdgeC = Falling;
				if(last_r == 1)
				{
					printk("R: PortC, Falling Edge\n");
					if(EdgeD == Falling)
					{
						reedState_r --;
						printk("R: R , %d \n", reedState_r);
					}
					else
					{
						reedState_r++;
						printk("R: V , %d \n", reedState_r);
					}
				}
				last_r = 0;
			}
                	return (irq_handler_t) IRQ_HANDLED;
                }
	}


	if (irq == irqNumberD)
	{
		int value = gpio_get_value(gpioPortD);
                if(value == GPIO_HIGH)
                {
			if(EdgeD == Falling) {
				EdgeD = Rising;
				if(last_r == 0)
				{
					printk("R: PortD, Rising Edge\n");
					if(EdgeC == Rising)
					{
						reedState_r ++;
						printk("R: V , %d \n", reedState_r);
					}
					else
					{
						reedState_r --;
						printk("R: R , %d \n", reedState_r);
					}
				}
				last_r = 1;
			}
                	return (irq_handler_t) IRQ_HANDLED;
                }
                else
                {
			if(EdgeD == Rising) {
				EdgeD = Falling;
				if(last_r == 0)
				{
					printk("R: PortD, Falling Edge\n");
					if(EdgeC == Falling)
					{
						reedState_r ++;
						printk("R: V, %d \n", reedState_r);
					}
					else
					{
						reedState_r --;
						printk("R: R, %d \n", reedState_r);
					}
				}
				last_r = 1;
			}
                	return (irq_handler_t) IRQ_HANDLED;
                }
	}
	return (irq_handler_t) IRQ_HANDLED;
}
module_init(ulsgpio_init);
module_exit(ulsgpio_exit);
