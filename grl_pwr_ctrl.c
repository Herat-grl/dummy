/******************************************************
 *  \file       grl_pwr_ctrl.c
 *
 *  \details    Simple Linux device driver (Signals)
 *
 *
 ******************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>      //kmalloc()
#include <linux/uaccess.h>   //copy_to/from_user()
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/signal.h>
#include <linux/kthread.h>  //kernel threads

#define DRIVER_NAME "GRL_PWR_CTRL"
#define JETSON_POWER_OFF_GPIO         353   //249
//#define TEGRA194_MAIN_GPIO_PORT_B       256  //outgpio

#define SIGETX 44
#define STAT_SUCCESS 0
#define STAT_FAILURE -1

#define REG_CURRENT_TASK _IOW('a', 'a', int32_t*)

#define IRQ_NO 11

/* Signaling to Application */
static struct task_struct *task;
static int signum;
char *argv[] = {"/bin/bash", "/usr/bin/grl_shutdown.sh", NULL};
static char *envp[] = {"HOME=/", "TERM=linux",
	"PATH=/sbin:/bin:/usr/sbin:/usr/bin", NULL};

dev_t dev;
static struct class *dev_class;
static struct cdev grl_cdev;
struct siginfo info;

static int __init grl_pwr_ctrl_init(void);
static void __exit grl_pwr_ctrl_exit(void);
static int grl_open(struct inode *inode, struct file *file);
static int grl_release(struct inode *inode, struct file *file);
static ssize_t grl_read(struct file *filp, char __user *buf,
		size_t len, loff_t *off);
static ssize_t grl_write(struct file *filp, const char *buf,
		size_t len, loff_t *off);
static long grl_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static struct task_struct *grl_irq_thread;
int wait_queue_grl_irq_evt;

static const struct file_operations fops = {
	.owner			= THIS_MODULE,
	.read			= grl_read,
	.write			= grl_write,
	.open			= grl_open,
	.unlocked_ioctl		= grl_ioctl,
	.release		= grl_release,
};

static int grl_irq_handler_fn(void *unused)
{
	while (!kthread_should_stop()) {
		if (wait_queue_grl_irq_evt) {
			pr_info(DRIVER_NAME " going to shutdown system\n");
			wait_queue_grl_irq_evt = 0;
			call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);
		}
	}
	do_exit(0);
	return 0;
}

//Interrupt handler for IRQ 11
static irqreturn_t irq_handler(int irq, void *dev_id)
{
	int lgpioval = 0;

	lgpioval = gpio_get_value(JETSON_POWER_OFF_GPIO);
	pr_info(DRIVER_NAME " lgpioval= %d\n", lgpioval);
	if (lgpioval == 1)
		wait_queue_grl_irq_evt = 1;

	return IRQ_HANDLED;
}

static int grl_open(struct inode *inode, struct file *file)
{
	pr_info(DRIVER_NAME " Device File Opened...!!!\n");
	return 0;
}

static int grl_release(struct inode *inode, struct file *file)
{
	struct task_struct *ref_task = get_current();

	pr_info(DRIVER_NAME " Device File Closed...!!!\n");
	//delete the task
	if (ref_task == task)
		task = NULL;

	return 0;
}

static ssize_t grl_read(struct file *filp, char __user *buf,
		size_t len, loff_t *off)
{
	pr_info(DRIVER_NAME " Read Function\n");
	return 0;
}

static ssize_t grl_write(struct file *filp, const char __user *buf,
		size_t len, loff_t *off)
{
	pr_info(DRIVER_NAME " Write function\n");
	return 0;
}

static long grl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pid *pid_struct = NULL;

	if (cmd == REG_CURRENT_TASK) {
		pr_info(DRIVER_NAME " arg = %ld\n", arg);
		pid_struct = find_get_pid(arg);
		pr_info(DRIVER_NAME " REG_CURRENT_TASK\n");
		task = pid_task(pid_struct, PIDTYPE_PID);
		if (task == NULL) {
			pr_err(DRIVER_NAME " Cannot find pid from user program\r\n");
			return -1;
		}
		memset(&info, 0, sizeof(struct siginfo));
		info.si_signo = SIGETX;
		info.si_code = SI_QUEUE;
		info.si_int = 1;
		signum = SIGETX;
	}

	return 0;
}

static int __init grl_pwr_ctrl_init(void)
{
	int irq;
	uint16_t status = STAT_SUCCESS;
	/*Allocating Major number*/
	if ((alloc_chrdev_region(&dev, 0, 1, "grl_Dev")) < 0) {
		pr_err(DRIVER_NAME " Cannot allocate major number\n");
		return -1;
	}
	pr_info(DRIVER_NAME " Major = %d Minor = %d\n", MAJOR(dev), MINOR(dev));

	/*Creating cdev structure*/
	cdev_init(&grl_cdev, &fops);

	/*Adding character device to the system*/
	if ((cdev_add(&grl_cdev, dev, 1)) < 0) {
		pr_err(DRIVER_NAME " Cannot add the device to the system\n");
		goto r_class;
	}

	/*Creating struct class*/
	dev_class = class_create(THIS_MODULE, "grl_class");
	if (dev_class == NULL) {
		pr_err(DRIVER_NAME " Cannot create the struct class\n");
		goto r_class;
	}

	/*Creating device*/
	if ((device_create(dev_class, NULL, dev, NULL, "phy_intr")) == NULL) {
		pr_err(DRIVER_NAME " Cannot create the Device\n");
		goto r_device;
	}

	gpio_request(JETSON_POWER_OFF_GPIO, "ingpio");
	//gpio_request(TEGRA194_MAIN_GPIO_PORT_B, "outgpio");

	gpio_direction_input(JETSON_POWER_OFF_GPIO);
	//gpio_direction_output(TEGRA194_MAIN_GPIO_PORT_B,0);

	grl_irq_thread = kthread_create(grl_irq_handler_fn, NULL,
			"grl_irq_thread");
	if (grl_irq_thread) {
		pr_info(DRIVER_NAME " Thread Created successfully\n");
		wake_up_process(grl_irq_thread);
	} else {
		pr_err(DRIVER_NAME " Thread creation failed\n");
	}

	irq = gpio_to_irq(JETSON_POWER_OFF_GPIO);
	status = request_irq(irq, irq_handler, IRQF_TRIGGER_RISING,
			"phy_intr", &dev);
	if (status != STAT_SUCCESS) {
		pr_err(DRIVER_NAME " my_device: cannot register IRQ\n");
		goto irq;
	}

	pr_info(DRIVER_NAME " Device Driver Insert...Done!!!\n");

	return 0;
irq:
	free_irq(irq, (void *)(irq_handler));
r_device:
	class_destroy(dev_class);
r_class:
	unregister_chrdev_region(dev, 1);
	return -1;
}

static void __exit grl_pwr_ctrl_exit(void)
{
	int irq;

	if (grl_irq_thread)
		kthread_stop(grl_irq_thread);

	irq = gpio_to_irq(JETSON_POWER_OFF_GPIO);
	free_irq(irq, &dev);
	device_destroy(dev_class, dev);
	class_destroy(dev_class);
	cdev_del(&grl_cdev);
	unregister_chrdev_region(dev, 1);
	pr_info(DRIVER_NAME " Device Driver Remove...Done!!!\n");
}

module_init(grl_pwr_ctrl_init);
module_exit(grl_pwr_ctrl_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A simple device driver - Signals");
MODULE_VERSION("1.20");
