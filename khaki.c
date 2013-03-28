#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>



// Contains functions pertaining to the file system
#include <linux/fs.h>

// Contains definitions handling device numbers (major and minor numbers) 
#include <linux/kdev_t.h>

// Contains definitions for registering this driver as a character driver with the system
#include <linux/cdev.h>

// Includes definitions pertaining to registering the driver with the system under a particular driver class
#include <linux/device.h>

// Includes definition of kmalloc
#include <linux/slab.h>

// Includes mmap definition
#include <linux/mman.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#define AUDIO_BUFFER_SIZE 4096 

// Struct containing the major and minor numbers for the driver
dev_t driver_numbers; 

// Struct containing the representation of this character driver
struct cdev fmad_cdev;

// Forward declaration of the drivers write operation
static ssize_t fmad_write(struct file *fp, const char *buff, size_t len, loff_t *off);
// Forward declaration of the drivers open operation
static int fmad_open(struct inode *inode, struct file *file);
// Forward declaration of the drivers release operation
static int fmad_release(struct inode * inode, struct file * file);
// Forward declaration of the major number aquisition function
int acquire_major_and_minor_numbers (void);
// Forward declaration of the character device creation function
int create_character_device (void);
// Forward declaration of the device class registration function
int add_device_to_system(void);
// Forward declaration for the function that will allocate all of the memory used by the driver
int allocate_working_memory(void);
// Forward declaration of the data structure initialization function
int init_data_structures(void);
// Forward declaration of the dma initialization function
int init_dma(void);
// Forward declaration of the status led test function
void test_status_led(void);


// Declares the operations that the character driver will be able to perform
struct file_operations fmad_fops = {
	open: fmad_open,
	write: fmad_write,
	release: fmad_release
};
// Declares a pointer to the struct that will hold the information for the class that this driver resides under
struct class * fmad_class; 
// Declares a pointer to the struct that will hold the information for the device. IE. Information for the the driver 
struct device * fmad_device;
// Declares the name that will be used to reference this driver on the system
char *device_name = "fmad";

// Declares the pointer to the audio buffer that will be used to buffer incoming audio
char * audio_buffer; 

// Declare a pointer to the base of the memory mapped peripherals
uint32_t * peripheral_base;

// Declares a pointer to the resource that this driver will take control of
struct resource * peripheral_resources; 

#define PERIPHERAL_BASE   0x20000000
#define PERIPHERAL_LENGTH 0x00300000 

#define GPIO_BASE_OFFSET 0x00200000

MODULE_LICENSE("GPL");





int init_module(void){
	acquire_major_and_minor_numbers();
	create_character_device();
	add_device_to_system();
	allocate_working_memory();
	init_data_structures();
	//init_dma();
	return 0;
}

void cleanup_module(void){
	iounmap(peripheral_base);
	release_mem_region(PERIPHERAL_BASE, PERIPHERAL_LENGTH);
	
	unregister_chrdev_region(driver_numbers,1);
	cdev_del(&fmad_cdev);
	device_destroy(fmad_class, driver_numbers);
	class_destroy(fmad_class);

	printk(KERN_INFO "Goodbye world 1.\n");
}


static ssize_t fmad_write(struct file *fp, const char *buff, size_t len, loff_t *off){
	


	return 0;
}

static int fmad_open(struct inode *inode, struct file *file){

	return 0;
}

static int fmad_release(struct inode * inode, struct file * file){

	return 0;
}

// Helper functions

// Figure out what major number is available for us to use
int acquire_major_and_minor_numbers (void){
	// This is only ONE very simple device that will only ever have one minor number. The minor number will be some number greater than zero. 
	int first_minor_number = 0;
	// This driver is very simple and will only need one major number.
	int number_of_major_numbers = 1;	
	// Request major and minor numbers for the driver using the parameters explained above
	int result_of_number_allocation = alloc_chrdev_region(&driver_numbers, first_minor_number, number_of_major_numbers, device_name); 
	if(result_of_number_allocation < 0){
		printk(KERN_INFO "FMAD: Failed to acquire major number\n");
		return -1;
	}else{
		printk(KERN_INFO "FMAD: Successfully obtained major and minor numbers.\n");
		return 0; 
	}

}


// Create a character driver
int create_character_device (void){

	// Adds this character driver to the system, the driver has one minor number
	int result_of_driver_number_addition; 	
	// Initialize the system representation of this character driver
	cdev_init(&fmad_cdev, &fmad_fops); 	
	result_of_driver_number_addition =  cdev_add(&fmad_cdev, driver_numbers, 1); 
	if(result_of_driver_number_addition){
		printk(KERN_INFO "FMAD: Failed to register major number %d with error code %d\n", MAJOR(driver_numbers), result_of_driver_number_addition);
		return -1;
	}else{
		printk(KERN_INFO "FMAD: Successfully registered major number of %d and minor numbers\n", MAJOR(driver_numbers));
		return 0;
	}
}


// Adds the device to the system such that it actually shows up under /dev
int add_device_to_system(void){
	// Creates a class that the driver will reside wtihin
	// Drivers on the system are grouped by class	
	fmad_class = class_create(THIS_MODULE,device_name); 
	if(IS_ERR(fmad_class)){
		printk(KERN_INFO "FMAD: Failed to construct a device class for this device drive\n");	
	}else{
		printk(KERN_INFO "FMAD: Successfully constructed a class for this device driver\n");
	}

	
	// Creates a device in the systems eyes so that it may show up in /dev as a device file
	fmad_device = device_create(fmad_class, NULL, driver_numbers, NULL, device_name);
	if(IS_ERR(fmad_device)){
		printk(KERN_INFO "FMAD: Failed to construct the device from the device class\n");
		return -1;
	}else{
		printk(KERN_INFO "FMAD: Successfully constructed the device from the device class\n");
		return 0;
	}

}


// Allocates all of the memory that this driver will need to operate
int allocate_working_memory(void){
	
	audio_buffer = kmalloc(AUDIO_BUFFER_SIZE, GFP_KERNEL);
	memset(audio_buffer, 0, AUDIO_BUFFER_SIZE);
	
	if(audio_buffer == NULL){
		printk(KERN_INFO "FMAD: Failed to allocate working memory for the driver of size %d\n", AUDIO_BUFFER_SIZE);
		return -1;
	}else{	
		printk(KERN_INFO "FMAD: Successfully allocated %d bytes of working memory for the driver\n", AUDIO_BUFFER_SIZE);
		
	}
	
	unsigned volatile * gpio_base;	
	
	peripheral_resources = request_mem_region(PERIPHERAL_BASE, PERIPHERAL_LENGTH, device_name);

	if(peripheral_resources == NULL){
		printk(KERN_INFO "FMAD: FAILED TO ALLOCATE GPIO RESOURCE\n");
		
	}else{
		
		printk(KERN_INFO "FMAD: SUCCESSFULLY ALLOCATED GPIO RESOURCE\n");		
		
		peripheral_base = (uint32_t *)ioremap(PERIPHERAL_BASE, PERIPHERAL_LENGTH);
		
		if(peripheral_base == NULL){
			printk( KERN_INFO "FMAD: FAILED TO MAP MEMORY\n");
		}else{
			printk(KERN_INFO "FMAD: SUCCESSFULLY MAPPED MEMORY to %p\n", peripheral_base);
		}
		
	}

	//*GPFSEL0 |= 1<<14;
	//*GPFSEL0 &= ~(1<<13);
	//*GPFSEL0 &= ~(1<<12);
	return 0;
}


int init_data_structures(void){

	return 0;	
}


int init_dma(void){
	return 0;
}


// This function was created to truly test the success of memory mapping in kernel space
// When called it sets the status light on the board to the ON position
// It turns out that the status light is behind an inversion mechanism that causes it to
// light up only when GPIO16 is set to low
void test_status_led(void){

	unsigned data1 = 1<<18;
	unsigned * location1 = peripheral_base + (GPIO_BASE_OFFSET/4)+1;

	unsigned data2 = 1<<16;
	unsigned * location2 = peripheral_base + (GPIO_BASE_OFFSET/4)+10;


	printk(KERN_INFO "FMAD: WRITING %d to %p\n",data1, location1);
		
	writel(data1, location1);

	printk(KERN_INFO "FMAD: WRITING %d to %p\n", data2, location2);

	writel(data2, location2);

}
