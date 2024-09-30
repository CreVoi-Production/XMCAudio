/*******************************************************************************
* File Name:   main.c
*
* Description:A program that receives audio data sent from MiVoi software via
* UART and outputs audio through PWM control from the data.
*
* Related Document:
* None.
********************************************************************************
* Copyright 2019-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include <inttypes.h>
#include <stdio.h>
#include <signal.h>
#include <sys/time.h>


/*******************************************************************************
* Macros
*******************************************************************************/
#define PWM_FREQUENCY (80000u)


/*******************************************************************************
* Global Variables
*******************************************************************************/
/* PWM object */
cyhal_pwm_t pwm_control;
/*Data length*/
int len=0;
/*Input Duty data*/
char freq[999999]={0};
/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
*  User defined error handling function.
*
* Parameters:
*  status - status for evaluation.
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(cy_rslt_t status)
{
    if (CY_RSLT_SUCCESS != status)
    {
        /* Halt the CPU while debugging */
        CY_ASSERT(0);
    }
}


/*******************************************************************************
* Function Name: check_status
********************************************************************************
* Summary:
*  Prints the message and waits forever when an error occurs.
*
* Parameters:
*  message - message to print if status is non-zero.
*  status - status for evaluation.
*
* Return:
*  void
*
*******************************************************************************/
void check_status(char *message, cy_rslt_t status)
{
    if (CY_RSLT_SUCCESS != status)
    {
        printf("\r\n=====================================================\r\n");
        printf("\nFAIL: %s\r\n", message);
        printf("Error Code: 0x%08" PRIX32 "\n", status);
        printf("\r\n=====================================================\r\n");

        while(true);
    }
}

/*******************************************************************************
* Function Name: map
********************************************************************************
* Summary:
*  Converts a number in one range to a number in another range.
*
* Parameters:
*  x - Input values.
*  in_min - Minimum value among input values.
*  in_max - Maximum value among input values.
*  out_min - Minimum value among output values.
*
*
* Return:
*  void
*
*******************************************************************************/
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*******************************************************************************
* Function Name: audio_Output
********************************************************************************
* Summary:
*PWM is generated from the received data and output to the speaker.
*
* Parameters:
*  void
*
*
* Return:
*  void
*
*******************************************************************************/
void audio_Output(void){
	 uint32_t status = cyhal_system_critical_section_enter();

	    for (int i=0;i<len;i++)
	    {
	    	cyhal_pwm_set_duty_cycle(&pwm_control, (float)map(freq[i], 0x00, 0xff, 0, 100),
	PWM_FREQUENCY);
	    	cyhal_pwm_start(&pwm_control);
	    	cyhal_system_delay_us(111);//
	    }
	    cyhal_pwm_stop(&pwm_control);
	    cyhal_system_critical_section_exit(status);
	    for(int i=0;i<len;i++)
	    {
	    	freq[i]=0x00;
	    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for the CPU. It configures the PWM and UART.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{

    /* API return code */
    cy_rslt_t result;
     /* Variable to store the received character through COMPort */
    uint8_t read_data;
    int buffer_f=0;

#if defined(CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;
    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    handle_error(result);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);
    handle_error(result);

    /* Initialize the PWM */
    result = cyhal_pwm_init(&pwm_control,P10_4 , NULL);

    for(;;){
    	cyhal_uart_getc(&cy_retarget_io_uart_obj,
            &read_data, 0);
    	if(read_data==0x53){
    		cyhal_uart_getc(&cy_retarget_io_uart_obj,
    		            &read_data, 0);
    		if(read_data==0x54){
    			cyhal_uart_getc(&cy_retarget_io_uart_obj,
    			            &read_data, 0);
    			if(read_data==0x41){
    			    			cyhal_uart_getc(&cy_retarget_io_uart_obj,
    			    			            &read_data, 0);
    			    			if(read_data==0x52){
    			    			    			cyhal_uart_getc(&cy_retarget_io_uart_obj,
    			    			    			            &read_data, 0);
    			    			    			if(read_data==0x54){
    			    			    			    			for(;;){
    			    			    			    				switch(buffer_f){
    			    			    			    					case 1:
    			    			    			    						freq[len+buffer_f]=0x45;
    			    			    			    						buffer_f=0;
    			    			    			    						break;
    			    			    			    					case 2:
    			    			    			    						freq[len+buffer_f]=0x4e;
    			    			    			    						freq[len+buffer_f-1]=0x45;
    			    			    			    						buffer_f=0;
    			    			    			    						break;
    			    			    			    					case 3:
    			    			    			    						freq[len+buffer_f]=0x44;
    			    			    			    						freq[len+buffer_f-2]=0x45;
    			    			    			    						freq[len+buffer_f-1]=0x4e;
    			    			    			    					    buffer_f=0;
    			    			    			    						break;
    			    			    			    					case 4:
    			    			    			    						freq[len+buffer_f]=0x49;
    			    			    			    						freq[len+buffer_f-3]=0x45;
    			    			    			    						freq[len+buffer_f-2]=0x4e;
    			    			    			    						freq[len+buffer_f-1]=0x44;
    			    			    			    						buffer_f=0;
    			    			    			    						break;
    			    			    			    					case 5:
    			    			    			    						freq[len+buffer_f]=0x4e;
    			    			    			    						freq[len+buffer_f-4]=0x45;
    			    			    			    						freq[len+buffer_f-3]=0x4e;
    			    			    			    						freq[len+buffer_f-2]=0x44;
    			    			    			    						freq[len+buffer_f-1]=0x49;
    			    			    			    						buffer_f=0;
    			    			    			    						break;
    			    			    			    				}

    			    			    			    				cyhal_uart_getc(&cy_retarget_io_uart_obj,
    			    			    			    	    			            &read_data, 0);

    			    			    			    				if(read_data==0x45){
    			    			    			    					buffer_f++;
    			    			    			    				    cyhal_uart_getc(&cy_retarget_io_uart_obj,
    			    			    			    						            &read_data, 0);

    			    			    			    					if(read_data==0x4e){
    			    			    			   		    				buffer_f++;
    			    		 				    			    			cyhal_uart_getc(&cy_retarget_io_uart_obj,
    		    			    				    			    			            &read_data, 0);

    			    		 				    			    			if(read_data==0x44){
    			    			    			    		    				buffer_f++;
    			    			    			     			    			cyhal_uart_getc(&cy_retarget_io_uart_obj,
    			    			    			  	    			    			            &read_data, 0);

    			    			    			    			    			if(read_data==0x49){
    			    			    			    			    				buffer_f++;
    			    			    			    				    			cyhal_uart_getc(&cy_retarget_io_uart_obj,
    			    			    			    		   			    			            &read_data, 0);

    			    			    			    				    			if(read_data==0x4e){
    			    			    			    				    				buffer_f++;
    			    			    			    		  			    			cyhal_uart_getc(&cy_retarget_io_uart_obj,
    			    			    			    		    			    			            &read_data, 0);

    			    			    			    		   			    			if(read_data==0x47){
    			    			    			    					    				buffer_f=0;
    			    			    			    		    			    			break;
    			    			    			    		    			    		}
    			    			    			    		   			    		}
    			    			    			    				    		}
    			    			    			    			    		}
    			    			    			    		    		}
    			    			    			    				}
    			    			    			    				len++;
	    			    			    							freq[len]=read_data;
    			    			    			    			}
    			    		    					}
    			    				}
    				}
    		}
    	}
    	audio_Output();
    }
    return 0;
}


/* [] END OF FILE */
