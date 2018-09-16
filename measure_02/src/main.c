

/*	ASSIGNMENT 3- TO MEASURE CONTEXT SWITCH OVERHEAD AND INTERRUPT LARENCY OVERHEAD WITH AND WITHOUT BACKGROUND COMPUTING 
TEAM 2: 
MEMBERS: HENA SHAH-1213348511
	 NIDHI DUBEY-1213031246

References for this program is taken from the sample examples of zephyr.

For implementation of gpio callback function we referred: /zephyr/samples/basic/button
For implementation of shell module we referred: /zephyr/samples/subsys/shell/shell
For implementation for gpio configrations we referred: /zephyr/samples/basic/blinky
For additional support of mutex,threads, pin multiplexing and pwm we referred the online documentation for galileo
*/

#include <zephyr.h>
#include <board.h>
#include <device.h>
#include <gpio.h>
#include <pinmux.h>
#include <misc/printk.h>
#include <misc/util.h>
#include <asm_inline_gcc.h>
#include <pwm.h>
#include <kernel.h>
#include <shell/shell.h>

#define PWM0 CONFIG_PWM_PCA9685_0_DEV_NAME
#define GPIO CONFIG_GPIO_DW_0_NAME
#define MUX CONFIG_PINMUX_NAME 
#define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)

/* size of stack area used by each thread */

#define STACKSIZE 1024


/* scheduling priority used by each thread */
#define PRIORITY 7  //higher priority thread
#define PRIORITY1 14 //lower priority thread

int count;
int TERMINATION;
int BACK_GROUND;
unsigned long long cont_lat[500];
unsigned long long int_lat[500];
unsigned long long int_lat_bc[500];
unsigned long long t1,t2,time;
unsigned long long unlock_time;
//pointer to device structure
struct device *dev1;
struct device *dev2;

//pointer to gpio callback structure
static struct gpio_callback gpio_cb;
struct k_mutex my_mutex;
struct k_msgq my_msgq;

//defining the stack area for threads along side its STACKSIZE
K_THREAD_STACK_DEFINE(stack_area, STACKSIZE);
K_THREAD_STACK_DEFINE(stack_area1, STACKSIZE);
struct k_thread my_thread_data, my_thread_data1;

K_THREAD_STACK_DEFINE(thread_stack_area, STACKSIZE);
K_THREAD_STACK_DEFINE(thread_stack_area1, STACKSIZE);
struct k_thread thread_data, thread_data1;

//structure used for data passing by message queue
struct data_item_type 
{
    char field[12];
};

struct data_item_type data;
//the size of the buffer for message queue is initalized as 10
char my_msgq_buffer[10 * sizeof(data)];
char msg[12] = "SENDING";

/*interrupt handler for IO 0, here time stamp is taken as soon as the callback function is called and then the interrupt latancy of computing without background and with background is found. 500 samples have been taken for each task */
void gpio_handler(struct device *dev2, struct gpio_callback *gpio_cb,u32_t pins)
{
   t2=_tsc_read();
   //calculate time differencxe
   time=t2-t1;
   if(BACK_GROUND==0)
   {
	int_lat[count]=time;
   }
 
if(BACK_GROUND==1)
   {
	int_lat_bc[count]=time;
   }


//   printk("\n %lld ",time);
     count=count+1;
//   printk("\n count %d ", count);

}

/*the shell command for print is implemented where the latency in nanosecond in printed for all 3 tasks-context switch, interrupt latency without background computing and interrupted latency with background computing. Change the value range if you want to see more value after giving the shell command */
static int shell_cmd_params(int argc, char *argv[])
{
	int cnt;
	printk("\n Context Switch \n");
	for (cnt = 5; cnt < 15; cnt++) {
		printk("  %lld ", cont_lat[cnt]*10/4);
	}
	printk("\n Interrupt Latency without BackGround \n");
	for (cnt = 67; cnt < 77; cnt++) {
		printk("  %lld ", int_lat[cnt]*10/4);
	}
	printk("\n Interrupt Latency with BackGround \n");
	for (cnt = 125; cnt < 135; cnt++) {
		printk("  %lld ", int_lat_bc[cnt]*10/4);
	}
	printk("\n");
	
	return 0;
}

#define MY_SHELL_MODULE "print_module"
// registers the shell module commands, here we have registered print command
static struct shell_cmd commands[] = {
	{ "print", shell_cmd_params, "print argc" },
	{ NULL, NULL, NULL }
};



/*thread for background computing which puts the message in the message queue. The task of message put is carried out continuously and after every instuction time stamp is taken because PWM is running in the background and the exact arrival of rising edge to IO0 is not know precisely. We tried to to find the ticks for each PWM cycle(to know the arrival of risng edge) by reading IO 0 for 1, but the ticks didn't come out properly because of the latency of the gpio_read and eventually the interrupt latency was not coming properly.*/
void thread_BC1(void* a ,void* b,void* c)
{ 

//	printk("\n Inside the producer thread\n");
	
	while(1)
	{
	
	//printk("\n Writing to thread 2");
		  t1=_tsc_read();
		  k_msgq_put(&my_msgq, msg, K_FOREVER); 
		  t1=_tsc_read();
		if(count==500)
		{
			break;
		}
		
	}
	gpio_pin_disable_callback(dev2,3);
//	printk("\n Exiting the producer thread\n");
	
}

/*thread for background computing which gets the message from the message queue. The task of message get is carried out continuously and after every instuction time stamp is taken as the PWM is running in the background and the exact arrival of rising edge to IO0 is not know precisely.
*/
void thread_BC2(void* a ,void* b,void* c)
{ 
//	printk("\n Inside the receiver thread \n");
	gpio_pin_enable_callback(dev2,3);
//	struct data_item_type data;
	while(1)
	{
	 	//printk("\n Reading from thread 1");
		t1=_tsc_read();
		k_msgq_get(&my_msgq, msg , K_FOREVER);
		t1=_tsc_read();
		if(count==500)
		{
			break;
		}
		
			
	}
//	printk("\n Exiting the receiver thread");
	gpio_pin_disable_callback(dev2,3);
}

/*higher priority thread for context switch which tries to acquire the mutex but cannot acquire as the lower priority thread has acquired the lock and we find the context switch overhead by taking the time stamp after the higher priority thread acquires the locks and before the lower priority thread releases the lock. Here we are also subtracting the overhead for k_mutex_unlock which is found intially in the main process.The overhead for mutex_unlock varies between 850-1250 so the context switch latency varies a bit(when program is runned multiple times) but the distribution in the histogram graph comes to be same. Here the overhead for k_thread_mutex is not taken into consideration as the higher priority thread is in the waiting state after it tries to acquire the lock and it simply goes to the ready state when the lower priority thread releases the lock. */ 
void thread1(void* a ,void* b,void* c)
{    
	int k,j,z;
	for(z=0;z<500;z++)
	{
//	printk("\n Inside the higher priority thread");
	k_sleep(1);
//	printk("\n After sleeping");
	for(k=0;k<=1000;k++)
        {   
        }
//	printk("\n Trying to acquire the lock for higher priority thread");
	k_mutex_lock(&my_mutex, K_FOREVER);
	t2=_tsc_read();
//	printk("\n Acquired the lock for Higher priority thread");
	for(j=0;j<=10000;j++)
        {
            for(k=0;k<=10000;k++)
            {   
            }
	
        }
	
//	printk("\n Releasing the lock for higher priority thread");
	cont_lat[z]=t2-t1-unlock_time;
   	k_mutex_unlock(&my_mutex);
//	printk("\n %lld",cont_lat[z]);
	k_sleep(5);
	}
	TERMINATION=1;
}

void thread2(void* a ,void* b,void* c)
{
    int i,j,k,z;
    for(z=0;z<500;z++)
    {
 // printk("\n Inside the lower priority thread");
    k_mutex_lock(&my_mutex, K_FOREVER);
 // printk("\n Acquired the lock for lower priority thread");
    for(i=0;i<=1000000;i++)
    {
        for(j=0;j<=500000;j++)
        {
            for(k=0;k<=100000;k++)
            { 
 
            }
        }
    }
	k_sleep(10);
 //     printk("\n Releasing the lock for lower priority thread");
    t1=_tsc_read();
    k_mutex_unlock(&my_mutex);
    k_sleep(10);
    }   
}

/*Here first the configration of IO5 as PWM pin and then setting the PWM cycles is done. The IO0 for Galileo Gen2 is configured as the input pin and the gpio callback function is intialized. Here we measure the context switch latency, interrupt latency without background task and interrupt latency with background is done sequentially by dynamically creating the threads when needed */
void main(void)
{
    
    printk("Hello \n");
    int flag;
 
    k_msgq_init(&my_msgq, my_msgq_buffer,sizeof(data), 10);
    k_mutex_init(&my_mutex);	
    k_mutex_lock(&my_mutex,K_FOREVER);
    t1=_tsc_read();	
    k_mutex_unlock(&my_mutex);
    t2=_tsc_read();
    unlock_time=t2-t1;

    //printk("\n %lld",unlock_time);

     k_msgq_init(&my_msgq, my_msgq_buffer,8, 10);
     struct device *pwm_dev;
     struct device *pinmux;
	
     dev1 = device_get_binding("EXP1");
     if(dev1==NULL)
     	printk("\nError in EXP1");

     dev2 = device_get_binding("GPIO_0");
     if(dev2==NULL)
	printk("\nError in GPIO_0");

      //Set LED pin as output 

      pwm_dev = device_get_binding(PWM0);
      pinmux = device_get_binding(MUX);
      pinmux_pin_set(pinmux, 5, PINMUX_FUNC_C);

      flag=gpio_pin_configure(dev1,0, GPIO_DIR_IN);
      if(flag<0)
           printk("\nError in pin configration of EXP1");
      flag=gpio_pin_write(dev1,0,0);
      if(flag<0)
           printk("\nError in pin write of EXP1");

      flag=gpio_pin_configure(dev1,1, GPIO_DIR_IN);
      if(flag<0)
           printk("\nError in pin configration of EXP1");

      flag=gpio_pin_write(dev1,1,0);
      if(flag<0)
           printk("\nError in pin write of EXP1");

      flag=gpio_pin_configure(dev2,3, GPIO_DIR_IN | GPIO_INT | EDGE);
      if(flag<0)
           printk("\nError in pin configration of GPIO_0");

      flag=gpio_pin_write(dev2,3,0);
      if(flag<0)
          printk("\nError in pin write of GPIO_0"); 

      gpio_init_callback(&gpio_cb,gpio_handler,BIT(3));
      gpio_add_callback(dev2,&gpio_cb);


   printk("\n########################### GOING FOR CONTEXT SWITCH ############################");
   //creating the higher and lower priority thread for context switch overhead

      k_tid_t thread_tid = k_thread_create(&thread_data, thread_stack_area,
                                 K_THREAD_STACK_SIZEOF(thread_stack_area),
                                 thread1,
                                 NULL, NULL, NULL,
                                 PRIORITY, 0, K_NO_WAIT);

      printk("\n thread ID %d",(int)thread_tid); 

      k_tid_t thread_tid1 = k_thread_create(&thread_data1, thread_stack_area1,
                                 K_THREAD_STACK_SIZEOF(thread_stack_area1),
                                 thread2,
                                 NULL, NULL, NULL,
                                 PRIORITY1, 0, K_NO_WAIT);
	printk("\n thread ID %d",(int)thread_tid1); 
	//wait till the measurement for the context switch overhead is over. 	
	while(1)
	{
		k_sleep(1);
		if(TERMINATION==1)
			break;
	}   

  //  k_sleep(1000);
   count=0;
   BACK_GROUND=0;
   printk("\n###########################	GOING INTERRUPT LATENCY WITHOUT BACKGROUND ############################");
  //enable the callback function and set pwm cycles to measure the interrupt latency
   gpio_pin_enable_callback(dev2, 3); 
   pwm_pin_set_cycles(pwm_dev,3,4095,1000);
 /*continuosly reading the timestamp till 500 measurements for interrupt latency is taken. We are continuously reading the timestamp as we don't know the exact instance for arrival for interrupt as PWM   is running in the background */
        t1=_tsc_read();
	while(count<500)
   	{
		t1=_tsc_read();
  	} 
	gpio_pin_disable_callback(dev2, 3);
	BACK_GROUND=1;
	count=0;
	printk("\n########################### GOING INTERRUPT LATENCY WITHOUT BACKGROUND ############################");
	k_sleep(500);
	/*creating the thread for background computing task */
	k_tid_t my_tid = k_thread_create(&my_thread_data, stack_area,
                                 K_THREAD_STACK_SIZEOF(stack_area),
                                 thread_BC1,
                                 NULL, NULL, NULL,
                                 PRIORITY, 0, K_NO_WAIT);
	printk("\n thread ID %d",(int)my_tid);

	k_tid_t my_tid1 = k_thread_create(&my_thread_data1, stack_area1,
                                 K_THREAD_STACK_SIZEOF(stack_area1),
                                 thread_BC2,
                                 NULL, NULL, NULL,
                                 PRIORITY1, 0, K_NO_WAIT);
	printk("\n thread ID %d",(int)my_tid1);
	//sleeping the main thread till the 500 measurement are taken for interrupt latency with background computing
	k_sleep(10000);
	//registering the shell module to print the measurements taken during the program
	printk("\nREGISTRING THE SHELL"); 
	printk("\nSHELL REGISTERED. PRESS ENTER TO ENTER SHELL COMMANDS");

	
	SHELL_REGISTER(MY_SHELL_MODULE, commands);
	
} 


