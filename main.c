/*******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the Watchdog Timer Example for
* ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* WDT time out for reset mode, in milliseconds. */
/* Max limit is given by WDT_MAX_TIMEOUT_MS      */
#define WDT_TIME_OUT_MS            (4000U)

#define RESET_MODE                 (0U)
#define SERVICE_MODE               (1U)
#define WATCHDOG_MODE              (SERVICE_MODE)

/* Each WDT limit */
#define LOWER_LIMIT                (0U)
#define UPPER_LIMIT                (40000U)
#define WARN_LIMIT                 (40000U)

/* Interval to toggle LED */
#define INTERVAL                   (1000U)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void initialize_wdt(void);

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function which demonstrates the WDT reset
*
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
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    Cy_SCB_UART_Init(UART_HW, &UART_config, NULL);
    Cy_SCB_UART_Enable(UART_HW);
    result = cy_retarget_io_init(UART_HW);

    /* retarget-io init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    printf("retarget-io ver1.6 testing \r\n");

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("******************"
           "PDL: Watchdog Timer"
           "****************** \r\n\n");

    /* Check the reason for device restart */
    if(CY_SYSLIB_RESET_HWWDT == Cy_SysLib_GetResetReason())
    {
        printf("Reset event from Watchdog Timer\r\n");
        /* It's WDT reset event - blink LED twice */
        Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, CYBSP_LED_STATE_ON);
        Cy_SysLib_Delay(100);
        Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, CYBSP_LED_STATE_OFF);
        Cy_SysLib_Delay(200);
        Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, CYBSP_LED_STATE_ON);
        Cy_SysLib_Delay(100);
        Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, CYBSP_LED_STATE_OFF);
    }
    else
    {
        printf("Reset event from Power-On or XRES\r\n");
        /* It's Power-On reset or XRES event - blink LED once */
        Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, CYBSP_LED_STATE_ON);
        Cy_SysLib_Delay(100);
        Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, CYBSP_LED_STATE_OFF);
        Cy_SysLib_Delay(100);
    }

    /* Reset the reason for device reset */
    Cy_SysLib_ClearResetReason();

    /* Initialize WDT */
    initialize_wdt();

    /* Enable global interrupt */
    __enable_irq();

    for (;;)
    {
        #if (WATCHDOG_MODE == SERVICE_MODE)
            /* Clear WDT */
            Cy_WDT_ClearWatchdog();

            /* Constant delay of 1000ms */
            Cy_SysLib_Delay(INTERVAL);

            /* Invert the state of LED */
            Cy_GPIO_Inv(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN);
        #elif (WATCHDOG_MODE == RESET_MODE)
            /* Wait until watchdog reset occurs */
            while(1);

        #endif
    }
}

/*******************************************************************************
* Function Name: InitializeWDT
********************************************************************************
* Summary:
* This function initializes the WDT block. WDT register must be configured under
* the condition of WDT register unlocked and disabled.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void initialize_wdt()
{
    /* Unlock WDT register before Initialize WDT */
    Cy_WDT_Unlock();

    /* Disable WDT before Initialize WDT */
    Cy_WDT_Disable();

    /* Set each limit */
    Cy_WDT_SetLowerLimit(LOWER_LIMIT);
    Cy_WDT_SetWarnLimit(WARN_LIMIT);
    Cy_WDT_SetUpperLimit(UPPER_LIMIT);

    /* Set each action when the limit is violated */
    Cy_WDT_SetLowerAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
    Cy_WDT_SetWarnAction(CY_WDT_WARN_ACTION_NONE);
    #if (WATCHDOG_MODE == SERVICE_MODE)
         Cy_WDT_SetUpperAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
    #elif(WATCHDOG_MODE == RESET_MODE)
         Cy_WDT_SetUpperAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_RESET);
    #endif

    Cy_WDT_SetAutoService(CY_WDT_DISABLE);
    Cy_WDT_SetDeepSleepPause(CY_WDT_DISABLE);
    Cy_WDT_SetDebugRun(CY_WDT_ENABLE);

    /* Enable WDT */
    Cy_WDT_Enable();

    /* It takes up to three clk_lf cycles to enable WDT */
    while(Cy_WDT_IsEnabled() == false);

    Cy_WDT_Lock();
    Cy_WDT_ClearInterrupt();
}

/* [] END OF FILE */