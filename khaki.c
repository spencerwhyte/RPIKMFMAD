/***************************************

This is a kernel space implementation of FM radio transmission for the
raspberry pi that is based on the userspace implememtation by Oliver
Mattos and Oskar Weigl. Transmission is fixed at 107 Mhz. The driver
is configured to appear under /dev/fmad. Writing 16 bit mono PCM data
to the device will cause FM radio transmission of the audio.

By: Hashem Shawqi and Spencer Whyte

[TODO: INSERT GPL HERE]

***************************************/

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

#include <linux/delay.h>

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
int allocate_resources(void);
// Forward declaration of the data structure initialization function
int init_data_structures(void);
// Forward declaration of the dma initialization function
int init_dma(void);
// Forward declaration of the status led test function
void test_status_led(void);
// Forward declaration of the gpio initialization function
void init_gpio(void);
// Forward declaration of the general purpose clock initialization function
void init_general_purpose_clock(void);
// Forward declaration of the function that will start the DMA transfers
void start_DMA(void);
// Forward declaration of the function that will stop the DMA transfers
void stop_DMA(void);


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



// Virtual address of the gpio register
volatile unsigned * gpio_base;
// Virtual address of the dma registers
volatile unsigned * dma_base;
// Virtual address of the clock registers
volatile unsigned * clock_base;
// Virtual address of the pwm registers
volatile unsigned * pwm_base;


// Defines the offset of the gpio registers
#define GPIO_BASE 0x20200000
// Define the number of addresses required to access the various GPIO registers
#define GPIO_LENGTH 0x100

// Defines the offset of the dma controller registers
#define DMA_BASE 0x20007000
// Defines the number of addresses required to access the various DMA registers
#define DMA_LENGTH 0x24

// Defines the offset of the general purpose clock registers
#define CLOCK_BASE 0x20101000
// Defines the number of addresses required to access the general purpose clock registers
#define CLOCK_LENGTH 0xA8

// Defines the offset of the PWM registers
#define PWM_BASE 0x2020C000
// Defines the number of addresses required to acccess the PWM registers
#define PWM_LENGTH 0x28

 // Resource struct used to manage the gpio as a system resource
struct resource * gpio_resource;
// Resource struct used to manage the dma as a system resource
struct resource * dma_resource;
// Resource struct used to manage the clock as a system resource
struct resource * clock_resource;
// Resource struct used to manage the pwm as a system resource
struct resource * pwm_resource;




// Defines the offset of the GPIO function select register zero
// Note that this is the first register among all the GPIO registers which
// is why the offset from the GPIO base is zero
#define GPFSEL0 0

// Defines the offset of the GPIO pin output clear register
#define GPCLR0 0x28

// Defines the offset from the peripheral base at which the general purpose clock register exits
#define CM_GP0CTL 0x00000070

// Struct for modifying the general purpose clock register
struct GPCTL{
	char SRC : 4;	// Source 
	char ENAB : 1;	// Enable/Disable clock generator
	char KILL : 1;	// Kill the clock
	char : 1;	// Reserved
	char BUSY : 1;	// Used to protect the clock
	char FLIP : 1;	// Used to invert the clock generator output
	char MASH : 2;	// Allows specification of the level of Multi-Stage Noise Shaping
	unsigned int : 13;	// Reserved
	char PASSWD : 8;	// Password
};


// Struct used to represent a single page in memory
struct PageInfo{
	void * p; // Physical Address
	void * v; // Virtual Address
};


// Page information about the buffer of frequency dividers
struct PageInfo frequency_divider_buffer;

// Page information about the 8 pages used for holding the DMA control blocks
struct PageInfo instrPage;

// This is used to keep quick references to the 1024 control blocks
struct PageInfo instrs[1024]; 

 
struct page * dma_control_block_pages;
    
struct page * frequency_divider_buffer_page;


struct CB{
	volatile unsigned int TI; 	// Transer information
	volatile unsigned int SOURCE_AD;// Source address
	volatile unsigned int DEST_AD;	// Destination address
	volatile unsigned int TXFR_LEN; // Transfer length in bytes
	volatile unsigned int STRIDE;	// 2D mode stride
	volatile unsigned int NEXTCONBK;// Next control block address
	volatile unsigned int RES1;	// Reserved	
	volatile unsigned int RES2;	// Reserved

};

struct DMAregs{
	volatile unsigned int CS; // Control and Status
	volatile unsigned int CONBLK_AD; // Control block address
 	volatile unsigned int TI; // Transfer information
	volatile unsigned int SOURCE_AD; // Source address
	volatile unsigned int DEST_AD; // Destination address
	volatile unsigned int TXFR_LEN; // Transfer length
	volatile unsigned int STRIDE; // 2D Stride
	volatile unsigned int NEXTCONBK; // Next control block address
	volatile unsigned int DEBUG; // For debugging purposes 
};

#define PWM_BUS_BASE 0x7e20C000

#define CM_GP0DIV 0x7e101074


MODULE_LICENSE("GPL");


int init_module(void){

	
	// Sets up this kernel module to appear as a device driver in /dev/	
	acquire_major_and_minor_numbers();
	create_character_device();
	add_device_to_system();

	// Allocates the resources that will be needed by this driver
	allocate_resources();
	// Setup gpio pin 4 to use the on board clock
	init_gpio();
	// Configure the on board clock
	init_general_purpose_clock();
	// Initializes the data structures used by the driver
	init_data_structures();				
	// Initializes the DMA controller
	init_dma();

	return 0;
}


void cleanup_module(void){
	// Reset the dma controller
	struct DMAregs * DMA0 = (struct DMAregs*)dma_base;
	DMA0->CS = 1 << 31; // Resets the DMA controller	

	// Free the pages allocated for this driver
	__free_pages(dma_control_block_pages,3);
	__free_page(frequency_divider_buffer_page);

	// Unmap the memory to the peripherals
	iounmap(gpio_base);
	iounmap(pwm_base);
	iounmap(clock_base);
	iounmap(dma_base);
	// Release the memory region that was used for accessing the peripherals
	release_mem_region(GPIO_BASE, GPIO_LENGTH);
	release_mem_region(CLOCK_BASE, CLOCK_LENGTH);
	release_mem_region(PWM_BASE, PWM_LENGTH);


	// Destroy the object used to represent this device
	device_destroy(fmad_class, driver_numbers);
	// Destroy the class created for this device
	class_destroy(fmad_class);
	// Remove the character driver from cdev 
	cdev_del(&fmad_cdev);
	// Unregister this character driver with the os
	unregister_chrdev_region(driver_numbers,1);
	
	printk(KERN_INFO "Goodbye world 1.\n");
}

void start_DMA(void){
	struct DMAregs * DMA0;	
	DMA0 = (struct DMAregs *)dma_base;
	// Reset
	DMA0->CS = 1<< 31;
 	DMA0->CONBLK_AD = 0;
	DMA0->TI = 0;
	DMA0->CONBLK_AD = (unsigned int) (instrs[0].p); // Start at the first control block
	DMA0->CS = (1<<0)|(255<<16); // Lowers the end flag and then sets the highest priority and the highest panic priority
}

void stop_DMA(void){
	
	struct DMAregs * DMA0;
	DMA0 = (struct DMAregs *)dma_base;
	// Reset
	DMA0->CS = 1<< 31;

}


static ssize_t fmad_write(struct file *fp, const char *buff, size_t len, loff_t *off){
	
	int chunkSize; 
	int bytesHandledThusFar = 0; 

	int bufPtr = 0;

	long long datanew,dataold = 0;
	short data;
	
	printk(KERN_INFO "FMAD: SOMEONE WROTE %d bytes to our driver\n", len);	
	
	chunkSize = len;
	
	// Handle 1 Mb at a time
	
	int max, min;
	max = 0;
	min = 512;
	
	while(bytesHandledThusFar + 2 < chunkSize){

		data = *((short *)(buff + bytesHandledThusFar));

		int shift = 27;

		long long multiplier, longdata, frac, clocksPerSample, dval, sample;

		int divisor,intval;

		unsigned int fracval;
	
		multiplier = 1 << shift;

		longdata = data;

		datanew = (longdata << (shift - 15));

		sample = datanew + (dataold - datanew)*-8;

		dval = sample * 16;

		intval = (int)(dval >> shift );

		//frac = ((dval - intval * multiplier)/2)+(multiplier>>1);

		clocksPerSample = 1400;


		if(intval > max){
			max = intval;
		}	

		if(intval < min){
			min  = intval;
		}

		if(intval > 512){
			printk(KERN_INFO "BADD APPROX %d\n", intval);
		}

		bufPtr++;	
		while(*(dma_base + (0x4/4)) == (int)instrs[bufPtr].p){
			mdelay(1);
		}
		((struct CB*) (instrs[bufPtr].v))->SOURCE_AD = (int)frequency_divider_buffer.p + 2048 + intval*4 - 4;
	
		bufPtr = (bufPtr+1) % (1024);		

		while(*(dma_base + (0x4/4)) == (int)instrs[bufPtr].p){
			mdelay(1);
		}

		((struct CB*) (instrs[bufPtr].v))->TXFR_LEN = clocksPerSample; // - fracValue


		bytesHandledThusFar += 2;

		dataold = datanew;

	}	
	printk(KERN_INFO  "RANGE ON THAT BLOCK WAS %d - %d\n", min , max);
	
	return len;
}

static int fmad_open(struct inode *inode, struct file *file){
	printk(KERN_INFO "FMAD: FILE OPEN\n");
	start_DMA();
	return 0;
}

static int fmad_release(struct inode * inode, struct file * file){
	printk(KERN_INFO "FMAD: FILE CLOSE\n");
	stop_DMA();
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
int allocate_resources(void){
	int i =0;
	void * current_virtual_address;
	void * current_physical_address;




	// We will construct a buffer containing 1024 different frequency divider values	
	//	int frequency_divider_buffer_size = 4096;
	/*
	
	frequency_divider_buffer.v = kmalloc(frequency_divider_buffer_size, GFP_KERNEL);
	memset(frequency_divider_buffer.v, 0, frequency_divider_buffer_size);
	
	if(frequency_divider_buffer.v == NULL){
		printk(KERN_INFO "FMAD: Failed to allocate memory for the frequency buffer of size %d\n", 1024);
		return -1;
	}else{
		printk(KERN_INFO "FMAD: Successfully allocated %d bytes of working memory for the driver\n", AUDIO_BUFFER_SIZE);
	}
	*/


	printk(KERN_INFO "ABOUT TO ALLOCATE THE PAGE \n");
		
	frequency_divider_buffer_page = NULL;
	
	frequency_divider_buffer_page = alloc_page(GFP_KERNEL);

	if(frequency_divider_buffer_page != NULL){
		printk(KERN_INFO "FMAD: Successfully allocated instruction page\n");

		frequency_divider_buffer.v = page_address(frequency_divider_buffer_page);
		frequency_divider_buffer.p = (void*)virt_to_phys(frequency_divider_buffer.v);

		printk(KERN_INFO "FMAD: Found the virtual and physical address of the instruction page to be %p %p\n", frequency_divider_buffer.v, frequency_divider_buffer.p);
	}else{
		printk(KERN_INFO "FMAD: Failed to allocate the instruction page\n");
	}	

	
	// Allocate 2^3=8 pages that will be used to store the control blocks
	// We need 8 pages because each page is 4096 bytes in size, giving us
	// 32, 768 bytes. With this many bytes we can store 1024 DMA control blocks
	// because each control block is 32 bytes in size. Each of the 1024 control
	// blocks contains the information neccessary to copy one frequency divider
	// from memory to the general purpose clock register.
	dma_control_block_pages = NULL;	

	dma_control_block_pages = alloc_pages(GFP_KERNEL, 3);	

	if(dma_control_block_pages != NULL){
		printk(KERN_INFO "FMAD: Successfully allocated the DMA control block pages\n");
		current_virtual_address = page_address(dma_control_block_pages);
		current_physical_address = (void*)virt_to_phys(current_virtual_address);
		
		for(i=0; i < 1024 ; i++){
			instrs[i].v = current_virtual_address;
			instrs[i].p = current_physical_address;
			current_virtual_address += sizeof(struct CB);
			current_physical_address += sizeof(struct CB);

		
		}
		printk(KERN_INFO "FMAD: Found the virtual and physical address of the first DMA control block page to be %p %p \n", instrs[0].v, instrs[0].p);
	}else{
		printk(KERN_INFO "FMAD: Failed to allocate pages for DMA control blocks\n");
	}		

	
	
	gpio_resource = request_mem_region(GPIO_BASE, GPIO_LENGTH, device_name);

	if(gpio_resource == NULL){
		printk(KERN_INFO "FMAD: FAILED TO ALLOCATE GPIO RESOURCE\n");
		
	}else{
		
		printk(KERN_INFO "FMAD: SUCCESSFULLY ALLOCATED GPIO RESOURCE\n");		
		
		gpio_base = (volatile unsigned *)ioremap(GPIO_BASE, GPIO_LENGTH);
		
		if(gpio_base == NULL){
			printk( KERN_INFO "FMAD: FAILED TO MAP GPIO MEMORY\n");
		}else{
			printk(KERN_INFO "FMAD: SUCCESSFULLY MAPPED GPIO MEMORY to %p\n", gpio_base);
		}
		
	}

	
	//dma_resource = request_mem_region(DMA_BASE, DMA_LENGTH, device_name);

	//if(dma_resource == NULL){
	//	printk(KERN_INFO "FMAD: FAILED TO ALLOCATE DMA RESOURCE\n");
		
	//}else{
		
	//	printk(KERN_INFO "FMAD: SUCCESSFULLY ALLOCATED DMA RESOURCE\n");		
		
	dma_base = (volatile unsigned *)ioremap(DMA_BASE, DMA_LENGTH);
	
	if(dma_base == NULL){
		printk( KERN_INFO "FMAD: FAILED TO MAP DMA MEMORY\n");
	}else{
		printk(KERN_INFO "FMAD: SUCCESSFULLY MAPPED DMA MEMORY to %p\n", dma_base);
	}
		
	//}


	pwm_resource = request_mem_region(PWM_BASE, PWM_LENGTH, device_name);

	if(pwm_resource == NULL){
		printk(KERN_INFO "FMAD: FAILED TO ALLOCATE PWM RESOURCE\n");
		
	}else{
		
		printk(KERN_INFO "FMAD: SUCCESSFULLY ALLOCATED PWM RESOURCE\n");		
		pwm_base = NULL;	
		pwm_base = (volatile unsigned *)ioremap(PWM_BASE, PWM_LENGTH);
		
		if(pwm_base == NULL){
			printk( KERN_INFO "FMAD: FAILED TO MAP PWM MEMORY\n");
		}else{
			printk(KERN_INFO "FMAD: SUCCESSFULLY MAPPED PWM MEMORY to %p\n", pwm_base);
		}
		
	}

	
	clock_resource = request_mem_region(CLOCK_BASE, CLOCK_LENGTH, device_name);

	if(clock_resource == NULL){
		printk(KERN_INFO "FMAD: FAILED TO ALLOCATE CLOCK RESOURCE\n");
		
	}else{
				
		printk(KERN_INFO "FMAD: SUCCESSFULLY ALLOCATED CLOCK RESOURCE\n");		
		clock_base = NULL;	
		clock_base = (volatile unsigned *)ioremap(CLOCK_BASE, CLOCK_LENGTH);
		
		if(clock_base == NULL){
			printk( KERN_INFO "FMAD: FAILED TO MAP CLOCK MEMORY\n");
		}else{
			printk(KERN_INFO "FMAD: SUCCESSFULLY MAPPED CLOCK MEMORY to %p\n", clock_base);
		}
		

	
		
	}

	//test_status_led();
	
	return 0;
}


int init_data_structures(void){
		
	int instrCnt = 0;
	int center_frequency_divider = 5 * (1<<12);

	center_frequency_divider = 18963;
		

	int i;
	struct CB * instr0;
	for(i = 0 ; i < 1024; i++){
		((int *)frequency_divider_buffer.v)[i] = (0x5a << 24) + center_frequency_divider - 512 + i;
	}


	i = 0;			

	instr0 = (struct CB *)instrs[0].v;	
		
	while(instrCnt<1024){
		if(i < 4096/sizeof(struct CB)){
		
		}else{
			i = 0;
		}
		

		printk(KERN_INFO "RUNNING THROUGH DMA CONTROL BLOCKS %d\n", instrCnt);
		instr0->SOURCE_AD = (unsigned int)frequency_divider_buffer.p+2048;
		instr0->DEST_AD = PWM_BUS_BASE + 0x18; // FIF1
		instr0->TXFR_LEN = 4;
		instr0->STRIDE = 0;


	
		// No wide burst because the wide bursts would try and move data that is larger than the data
		// we wish to move ( inefficient) 
		instr0->TI = (1 << 6) | ( 5 << 16) | (1<< 26 );
		instr0->RES1 = 0;
		instr0->RES2 = 0;
	
		if(i%2){	
			printk(KERN_INFO "IN MOD 2 %d\n", i);			
			instr0->DEST_AD = CM_GP0DIV;
			instr0->STRIDE = 4;
			instr0->TI = (1<<26);
		}
			
		

		if(instrCnt!=0){
			((struct CB*)(instrs[instrCnt-1].v))->NEXTCONBK = (int)instrs[instrCnt].p;
		}
		

				
		i++;


		instr0++;
		instrCnt++;
	}

	((struct CB*)(instrs[1023].v))->NEXTCONBK = (int)instrs[0].p;
	

	// Resetting the clock generator and setting the clock source to PLLD
	*(clock_base+(40*4/4)) = 0x5A000026;
	msleep(1);
	// Setting DIVI to 2 and DIVF to 2048
	*(clock_base+(41*4/4)) = 0x5A002800;
	// Enabling the clock and setting the clock source to PLLD
	*(clock_base+(40*4/4)) = 0x5A000016;
	mdelay(1);
	
	*(pwm_base + (0x0/4) ) = 0;
	mdelay(1);
	*(pwm_base + (0x4/4) ) = -1;
	mdelay(1);
	*(pwm_base + (0x0/4) ) = -1;
	mdelay(1);
	*(pwm_base + (0x8/4) ) = (1<<31 ) | 0x0707;

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
	volatile unsigned * location1 = gpio_base + (GPFSEL0/4);

	unsigned data2 = 1<<16;
	volatile unsigned * location2 = gpio_base + (GPCLR0/4);


	printk(KERN_INFO "FMAD: WRITING %d to %p\n",data1, location1);
		
	*location1 = data1;

	printk(KERN_INFO "FMAD: WRITING %d to %p\n", data2, location2);

	*location2 = data2;

}

// Initializes gpio pin 4 such that it now uses an alternate function
// The alternate function selected is the clock function
// The clock function will now reroute the output of PLLD to GPIO4

void init_gpio(void){
	volatile unsigned * gpio_select_function_register = gpio_base + (GPFSEL0/4);
 	// These sequence of operations set the function of GPIO pin 4 to alternate function 0
	// 001 	
	*gpio_select_function_register |= 1<<14;

	*gpio_select_function_register &=~(1<<13);

	*gpio_select_function_register &=~(1<<12);

}

// Initializes the general purpose clock used to produce the FM signal
void init_general_purpose_clock(){
	struct GPCTL setup = {
				6, // PLLD
			   	1, // Enable the clock generator
				0, // Set KILL to zero (don't kill the clock)
				0, // Don't overprotect the clock
				0, // Don't invert the clock generator output
				1, // 1-Stage MASH (Equivalent to non-mash dividers) 
				0x5a // Password
				};
	
	volatile unsigned * general_purpose_clock_register = clock_base + (CM_GP0CTL /4);
	
	printk(KERN_INFO "CLOCK WRITING TO %p \n", clock_base);
	
	*general_purpose_clock_register = *((int*)&setup);
}


