/*
 * Paulo Pedreiras, 2022/02
 * Zephyr: Simple thread creation example (2)
 * 
 * One of the tasks is periodc, the other two are activated via a semaphore. Data communicated via sharem memory 
 *
 * Base documentation:
 *      https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/kernel/index.html
 * 
 */


#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdlib.h>
#include <stdio.h>

/*ADC definitions and includes*/
#include <hal/nrf_saadc.h>
#define ADC_NID DT_NODELABEL(adc) 
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1 

#define BUFFER_SIZE 1

#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1 

#define TIMER_INTERVAL_MSEC 1000 /* Interval between ADC samples */

/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_In_prio 1
#define thread_Filter_prio 1
#define thread_Out_prio 1

/* Thread periodicity (in ms)*/
#define SAMP_PERIOD_MS 1000

/* ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_ID,
    .input_positive = ADC_CHANNEL_INPUT
};

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_In_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_Filter_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_Out_stack, STACK_SIZE);
  
/* Create variables for thread data */
struct k_thread thread_In_data;
struct k_thread thread_Filter_data;
struct k_thread thread_Out_data;

/* Create task IDs */
k_tid_t thread_In_tid;
k_tid_t thread_Filter_tid;
k_tid_t thread_Out_tid;

/* Global vars (shared memory) */
int sm_1 = 0;
int sm_2 = 0;

/* Semaphores for task synch */
struct k_sem sem1;
struct k_sem sem2;

/* Thread code prototypes */
void thread_In_code(void);
void thread_Filter_code(void);
void thread_Out_code(void);

void config(void){
    int err=0;

    /* ADC setup: bind and initialize */
    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
	if (!adc_dev) {
        printk("ADC device_get_binding() failed\n");
    } 
    err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err) {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }
    
    /* It is recommended to calibrate the SAADC at least once before use, and whenever the ambient temperature has changed by more than 10 �C */
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
   
}

/* Takes one sample */
static int adc_sample(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (adc_dev == NULL) {
            printk("adc_sample(): error, must bind to adc first \n\r");
            return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret) {
            printk("adc_read() failed with code %d\n", ret);
	}	

	return ret;
}

/* Main function */
void main(void) {
    
    config();
    
    /* Create and init semaphores */
    k_sem_init(&sem1, 0, 1);
    k_sem_init(&sem2, 0, 1);
    
    /* Create tasks */
    thread_In_tid = k_thread_create(&thread_In_data, thread_In_stack,
        K_THREAD_STACK_SIZEOF(thread_In_stack), Input,
        NULL, NULL, NULL, thread_In_prio, 0, K_NO_WAIT);

    thread_Filter_tid = k_thread_create(&thread_Filter_data, thread_Filter_stack,
        K_THREAD_STACK_SIZEOF(thread_Filter_stack), Filter,
        NULL, NULL, NULL, thread_Filter_prio, 0, K_NO_WAIT);

    thread_Out_tid = k_thread_create(&thread_Out_data, thread_Out_stack,
        K_THREAD_STACK_SIZEOF(thread_Out_stack), Output,
        NULL, NULL, NULL, thread_Out_prio, 0, K_NO_WAIT);

    
    return;

} 

/* Thread code implementation */
void Input(void)
{
    /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0;

    /* Other variables */
    int16_t input;
    int err=0;
    
    printk("Input thread init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_In_period;

    /* Thread loop */
    while(1) {
        
        /* Do the workload */
        err=adc_sample();
        if(err) {
            printk("adc_sample() failed with error code %d\n\r",err);
        }
        else {
            if(adc_sample_buffer[0] > 1023) {
                printk("adc reading out of range\n\r");
            }
            else {
                input= (uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023));
                /* ADC is set to use gain of 1/4 and reference VDD/4, so input range is 0...VDD (3 V), with 10 bit resolution */
                printk("adc reading: raw:%4u / %4u mV: \n\r",adc_sample_buffer[0],(uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023)));
            }
        }

        sm_1= input;

        k_sem_give(&sem1);

        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_In_period;

        }
    }
}

void Filter(void *argA , void *argB, void *argC)
{
    /* Other variables */
    long int nact = 0;

    printk("Thread B init (sporadic, waits on a semaphore by task A)\n");
    while(1) {
        k_sem_take(&sem1,  K_FOREVER);
         
        k_sem_give(&sem2);    
  }
}

void Output(void *argA , void *argB, void *argC)
{
    /* Other variables */
    long int output;

    printk("Thread C init (sporadic, waits on a semaphore by task A)\n");
    while(1) {
        k_sem_take(&sem2,  K_FOREVER);
        output= sm_2;
  }
}

