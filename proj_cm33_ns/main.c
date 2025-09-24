/*******************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for non-secure
*                    application running on CM33 CPU.
*
* Related Document : See README.md
*
********************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "mtb_hal.h"
#include "retarget_io_init.h"
#include "lfs.h"
#include "lfs_sd_bd.h"
#include "lfs_spi_flash_bd.h"
#include "mtb_serial_memory.h"
#include "cy_smif.h"
#include "cycfg_qspi_memslot.h"
#include "mtb_hal_sdhc.h"
#include "cy_sd_host.h"
#include <inttypes.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"
#include "cy_time.h"

/*******************************************************************************
 * Macros
 ********************************************************************************/
/* When set to 1, SD card will be used instead of the NOR flash */
#define STORAGE_DEVICE_SD_CARD              (0U)
#define FS_STORAGE_START_ADDRESS            (0xA00000U)
#define FS_STORAGE_SIZE                     (0x400000U)
#define FLASH_LFS_ADDRESS_START             (0x80000U)
#define FLASH_LFS_SIZE                      (0x80000U)
#define LITTLEFS_TASK_STACK_SIZE            (configMINIMAL_STACK_SIZE * 8U)
#define LITTLEFS_TASK_PRIORITY              (configMAX_PRIORITIES - 1U)
#define USER_BUTTON_ISR_PRIORITY            (3U)
#define PORT_INTR_MASK                      (0x00000001UL << 8U)

/* Debounce delay for the user button. */
#define DEBOUNCE_DELAY_MS                   (50U)

/* The timeout value in microsecond used to wait for core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC            (10U)

/* Enabling or disabling a MCWDT requires a wait time of upto 2 CLK_LF cycles  
 * to come into effect. This wait time value will depend on the actual CLK_LF  
 * frequency set by the BSP.
 */
#define LPTIMER_0_WAIT_TIME_USEC            (62U)

/* Define the LPTimer interrupt priority number. '1' implies highest priority. 
 */
#define APP_LPTIMER_INTERRUPT_PRIORITY      (1U)

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static TaskHandle_t littlefs_task_handle;
struct lfs_config lfs_cfg;
static mtb_hal_lptimer_t lptimer_obj;

/* RTC HAL object */
static mtb_hal_rtc_t rtc_obj;

/*******************************************************************************
* Function Name: setup_clib_support
********************************************************************************
* Summary:
*    1. This function configures and initializes the Real-Time Clock (RTC).
*    2. It then initializes the RTC HAL object to enable CLIB support library 
*       to work with the provided Real-Time Clock (RTC) module.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void setup_clib_support(void)
{
    /* RTC Initialization */
    Cy_RTC_Init(&CYBSP_RTC_config);
    Cy_RTC_SetDateAndTime(&CYBSP_RTC_config);

    /* Initialize the ModusToolbox CLIB support library */
    mtb_clib_support_init(&rtc_obj);
}

/*******************************************************************************
* Function Name: lptimer_interrupt_handler
********************************************************************************
* Summary:
* Interrupt handler function for LPTimer instance. 
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void lptimer_interrupt_handler(void)
{
    mtb_hal_lptimer_process_interrupt(&lptimer_obj);
}

/*******************************************************************************
* Function Name: setup_tickless_idle_timer
********************************************************************************
* Summary:
*    1. This function first configures and initializes an interrupt for LPTimer.
*    2. Then it initializes the LPTimer HAL object to be used in the RTOS 
*       tickless idle mode implementation to allow the device enter deep sleep 
*       when idle task runs. LPTIMER_0 instance is configured for CM33 CPU.
*    3. It then passes the LPTimer object to abstraction RTOS library that 
*       implements tickless idle mode
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void setup_tickless_idle_timer(void)
{
    /* Interrupt configuration structure for LPTimer */
    cy_stc_sysint_t lptimer_intr_cfg =
    {
        .intrSrc = CYBSP_CM33_LPTIMER_0_IRQ,
        .intrPriority = APP_LPTIMER_INTERRUPT_PRIORITY
    };

    /* Initialize the LPTimer interrupt and specify the interrupt handler. */
    cy_en_sysint_status_t interrupt_init_status = 
                                    Cy_SysInt_Init(&lptimer_intr_cfg, 
                                                    lptimer_interrupt_handler);
    
    /* LPTimer interrupt initialization failed. Stop program execution. */
    if(CY_SYSINT_SUCCESS != interrupt_init_status)
    {
        handle_app_error();
    }

    /* Enable NVIC interrupt. */
    NVIC_EnableIRQ(lptimer_intr_cfg.intrSrc);

    /* Initialize the MCWDT block */
    cy_en_mcwdt_status_t mcwdt_init_status = 
                                    Cy_MCWDT_Init(CYBSP_CM33_LPTIMER_0_HW, 
                                                &CYBSP_CM33_LPTIMER_0_config);

    /* MCWDT initialization failed. Stop program execution. */
    if(CY_MCWDT_SUCCESS != mcwdt_init_status)
    {
        handle_app_error();
    }
  
    /* Enable MCWDT instance */
    Cy_MCWDT_Enable(CYBSP_CM33_LPTIMER_0_HW,
                    CY_MCWDT_CTR_Msk, 
                    LPTIMER_0_WAIT_TIME_USEC);

    /* Setup LPTimer using the HAL object and desired configuration as defined
     * in the device configurator. */
    cy_rslt_t result = mtb_hal_lptimer_setup(&lptimer_obj, 
                                            &CYBSP_CM33_LPTIMER_0_hal_config);
    
    /* LPTimer setup failed. Stop program execution. */
    if(CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Pass the LPTimer object to abstraction RTOS library that implements 
     * tickless idle mode 
     */
    cyabs_rtos_set_lptimer(&lptimer_obj);
}

/*******************************************************************************
 * Function Name: check_status
 ****************************************************************************//**
 * Summary:
 *  Prints the message, indicates the non-zero status (error condition) by
 *  turning the LED on, and asserts the non-zero status.
 *
 * Parameters:
 *  message - message to print if status is non-zero.
 *  status - status for evaluation.
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void check_status(char *message, uint32_t status)
{
    if (status)
    {
        printf("\n=========================================================\n");
        printf("\nFAIL: %s\n", message);
        printf("Error Code: 0x%08"PRIx32"\n", status);
        printf("\n=========================================================\n");

        while(true);
    }
}

/*******************************************************************************
 * Function Name: print_block_device_parameters
 *******************************************************************************
 * Summary:
 *   Prints the block device parameters such as the block count, block size, and
 *   program (page) size to the UART terminal.
 *
 * Parameters:
 *  lfs_cfg - pointer to the lfs_config structure.
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void print_block_device_parameters(struct lfs_config *lfs_cfg)
{
    printf("Number of blocks: %"PRIu32"\n", lfs_cfg->block_count);
    printf("Erase block size: %"PRIu32" bytes\n", lfs_cfg->block_size);
    printf("Prog size: %"PRIu32" bytes\n", lfs_cfg->prog_size);
}

/*******************************************************************************
 * Function Name: user_button_interrupt_handler
 *******************************************************************************
 * Summary:
 *   User button interrupt handler.
 *
 * Parameters:
 *  void *handler_arg (unused)
 *  cyhal_gpio_event_t (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void user_button_interrupt_handler()
{
    /* Get interrupt cause */
    uint32_t intrSrc = Cy_GPIO_GetInterruptCause0();

    /* Check if the interrupt was from the user button's port */
    if(PORT_INTR_MASK == (intrSrc & PORT_INTR_MASK))
    {
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);

        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(littlefs_task_handle, &higher_priority_task_woken);

        /* Yield if xHigherPriorityTaskWoken was set to true */
        portYIELD_FROM_ISR( higher_priority_task_woken );
    }
}

#if(STORAGE_DEVICE_SD_CARD)

static mtb_hal_sdhc_t sdhc_obj;
static cy_stc_sd_host_context_t sdhc_host_context;

/*******************************************************************************
* Function Name: sd_card_isr
********************************************************************************
* Summary:
*   Interrupt routine for SD Card
*
* Parameters:
*  None
*
*******************************************************************************/
void sd_card_isr(void)
{
    mtb_hal_sdhc_process_interrupt(&sdhc_obj);
}

/*******************************************************************************
* Function Name: Cy_SD_Host_IsCardConnected
********************************************************************************
* Summary:
*   Function checks if SD-Card is connected
*
* Parameters:
*  base - Pointer to SDHC HW block
*
* Return:
*     True if Card is connected, False if Card is disconnected
*
*******************************************************************************/
bool Cy_SD_Host_IsCardConnected(SDHC_Type const *base)
{
    (void) base;
    return true;
}

/*******************************************************************************
* Function Name: init_lfs_sd_card
********************************************************************************
* Summary:
*   Initializes SD-Card and littlefs driver
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void init_lfs_sd_card(void)
{
    cy_rslt_t result;
    cy_en_sd_host_status_t pdl_sdhc_status;
    cy_en_sysint_status_t  pdl_sysint_status;

    /* Configuration structure for the SD Host Controller (SDHC) interrupt. */
    cy_stc_sysint_t sdhc_isr_config =
    {
        .intrSrc = CYBSP_SDHC_1_IRQ,
        .intrPriority = 3U
    };

    /* The SD Card should be enabled before calling any other SD Card APIs */
    Cy_SD_Host_Enable(CYBSP_SDHC_1_HW);

    pdl_sdhc_status = Cy_SD_Host_Init(CYBSP_SDHC_1_HW, &CYBSP_SDHC_1_config,
                                                            &sdhc_host_context);

    if (CY_SD_HOST_SUCCESS != pdl_sdhc_status)
    {
        check_status("Cy_SD_Host_Init returns error status",
                                                (uint32_t)pdl_sdhc_status);
    }

    pdl_sdhc_status = Cy_SD_Host_InitCard(CYBSP_SDHC_1_HW,
                                    &CYBSP_SDHC_1_card_cfg, &sdhc_host_context);

    if (CY_SD_HOST_SUCCESS != pdl_sdhc_status)
    {
        check_status("Cy_SD_Host_InitCard returns error status",
                                                    (uint32_t)pdl_sdhc_status);
    }

    result = mtb_hal_sdhc_setup(&sdhc_obj, &CYBSP_SDHC_1_sdhc_hal_config, NULL ,
                                    &sdhc_host_context);

    if (CY_RSLT_SUCCESS != result)
    {
        check_status("mtb_hal_sdhc_setup returns error status",
                                                    (uint32_t)pdl_sdhc_status);
    }


    pdl_sysint_status = Cy_SysInt_Init(&sdhc_isr_config, sd_card_isr);

    if (CY_SYSINT_SUCCESS != pdl_sysint_status)
    {
        check_status("Cy_SysInt_Init returns error status",
                                                    (uint32_t)pdl_sdhc_status);
    }

    NVIC_EnableIRQ((IRQn_Type) sdhc_isr_config.intrSrc);

    result = lfs_sd_bd_create(&lfs_cfg, &sdhc_obj);
    check_status("Creating SD Card block device failed.", result);
}
#else  /* (STORAGE_DEVICE_SD_CARD) */

static mtb_serial_memory_t serial_memory_obj;
static cy_stc_smif_mem_context_t smif_mem_context;
static cy_stc_smif_mem_info_t smif_mem_info;

/*******************************************************************************
* Function Name: init_lfs_flash
********************************************************************************
* Summary:
*   Initialize serial-flash middle-ware and little file system.
*
* Parameters:
*  None
*
*******************************************************************************/
static void init_lfs_flash(void)
{
    cy_rslt_t result;

    /* Set-up serial memory. */
    result = mtb_serial_memory_setup(&serial_memory_obj, 
                                MTB_SERIAL_MEMORY_CHIP_SELECT_1, 
                                CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base,
                                CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.clock,
                                &smif_mem_context, 
                                &smif_mem_info,
                                &smif0BlockConfig);

    if (CY_RSLT_SUCCESS != result)
    {
        check_status("mtb_serial_memory_setup returns error status",
                                                             (uint32_t)result);
    }

    lfs_spi_flash_bd_configure_memory(&lfs_cfg, FS_STORAGE_START_ADDRESS,
                                                               FS_STORAGE_SIZE);

    result = lfs_spi_flash_bd_create(&lfs_cfg, &serial_memory_obj);

    check_status("Creating SPI flash block device failed.", result);
}

#endif

/*******************************************************************************
 * Function Name: increment_boot_count
 ********************************************************************************
 * Summary:
 *   Mounts the filesystem in the memory and performs basic file I/O operations.
 *   And then reads a 32-bit value from a file, increments the value, writes
 *   the value back to the file in the memory, and finally prints the value to the
 *   UART terminal.
 *
 * Parameters:
 *  lfs - pointer to the lfs_t structure.
 *  lfs_cfg - pointer to the lfs_config structure.
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void increment_boot_count(lfs_t *lfs, struct lfs_config *lfs_cfg)
{
    uint32_t boot_count = 0;
    lfs_file_t file;

    /* Mount the filesystem */
    int err = lfs_mount(lfs, lfs_cfg);

    /* Reformat if we cannot mount the filesystem.
     * This should only happen when littlefs is set up on the storage device for
     * the first time.
     */
    if (err) {
        printf("\nError in mounting. This could be the first time littlefs is "
                "used on the storage device.\n");
        printf("Formatting the block device...\n");

        lfs_format(lfs, lfs_cfg);
        lfs_mount(lfs, lfs_cfg);
    }

    /* Read the current boot count. */
    lfs_file_open(lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(lfs, &file, &boot_count, sizeof(boot_count));

    /* Update the boot count. */
    boot_count += 1;
    lfs_file_rewind(lfs, &file);
    lfs_file_write(lfs, &file, &boot_count, sizeof(boot_count));

    /* The storage is not updated until the file_sd is closed successfully. */
    lfs_file_close(lfs, &file);

    /* Release any resources we were using. */
    lfs_unmount(lfs);

    /* Print the boot count. */
    printf("boot_count: %"PRIu32"\r\n\n", boot_count);
}

/*******************************************************************************
 * Function Name: littlefs_task
 *******************************************************************************
 * Summary:
 *   Initializes the block device, prints the block device parameters to the UART
 *   terminal, calls the function that performs simple file I/O operations, and
 *   waits for user button press. When the user button is pressed, formats the
 *   memory and deinitializes the block device.
 *
 * Parameters:
 *  arg - Unused.
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void littlefs_task(void* arg)
{
    lfs_t lfs;

    /* Step 1: Get the default configuration for the block device.
     * Step 2: Initialize the lfs_config structure to zero (not required if it
     *         is a global variable)
     * Step 3: Create the block device
     * Step 4: Print the block device parameters such as erase block size
     * Step 5: Perform file system operations to increment the boot count
     */

    /* Initialize the pointers in lfs_cfg to NULL. */
    memset(&lfs_cfg, 0, sizeof(lfs_cfg));

#if(STORAGE_DEVICE_SD_CARD)
    printf("\nCreating File System block on SD Card device...\n");
    init_lfs_sd_card();
#else
    printf("Creating File System block on SPI flash device...\n");
    init_lfs_flash();
#endif

    print_block_device_parameters(&lfs_cfg);

    printf("\nIncrementing the boot count...\n");

    increment_boot_count(&lfs, &lfs_cfg);

    printf("Press User button (SW2) to format the block device\n "
            "or\nPress Reset button (XRES) to increment the boot count again\n");

    /* Wait until the user button press is notified through the interrupt */
    while (true)
    {
        if (1UL == ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));

            if (!Cy_GPIO_Read(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN))
            {
                break;
            }
        }
    }

    /* User button is pressed. Format the block device. */
    printf("Formatting the block device...\n");
    lfs_format(&lfs, &lfs_cfg);
    printf("Formatting completed...\n");

#if(STORAGE_DEVICE_SD_CARD)
    /* Free the resources associated with the block device. */
    lfs_sd_bd_destroy(&lfs_cfg);
#else
    lfs_spi_flash_bd_destroy(&lfs_cfg);
#endif

    for(;;)
    {

    }
}

/*******************************************************************************
 * Function Name: main
 *******************************************************************************
 * Summary:
 * This is the main function for CM33 CPU. It does...
 *    1. Initializes the UART for redirecting printf output
 *    2. Initializes the user button GPIO
 *    3. Creates a FreeRTOS task to perform file I/O operations
 *    4. Starts the FreeRTOS scheduler
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 ******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /**Configuration structure for the user button 1 interrupt for handling 
     * interrupts from the user button. */
    cy_stc_sysint_t user_btn_int_cfg =
    {
        .intrSrc      = CYBSP_USER_BTN1_IRQ,
        .intrPriority = USER_BUTTON_ISR_PRIORITY,
    };

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;

    CY_ASSERT (result == CY_RSLT_SUCCESS);

    /* Initialize retarget-io middleware */
    init_retarget_io();

    /* Setup CLIB support library. */
    setup_clib_support();

    /* Setup the LPTimer instance for CM33 CPU. */
    setup_tickless_idle_timer();

    /* Clear GPIO interrupt */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN);

    /* Clear interrupt pending bit on user button interrupt */
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN2_IRQ);

    /* Initialize user button interrupt */
    Cy_SysInt_Init(&user_btn_int_cfg, &user_button_interrupt_handler);

    /* Enable user button interrupt */
    NVIC_EnableIRQ(CYBSP_USER_BTN1_IRQ);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("\r\n*************"
            " PSOC Edge MCU: Littlefs File System"
            " ************* \n\n");

    /* Enable CM55. */
    /* CY_CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed.*/
    Cy_SysEnableCM55(MXCM55, CY_CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);

    /* Enable all interrupts. */
    __enable_irq();

    /* Create the user tasks. See the respective task definition for more
     * details of these tasks. */
    xTaskCreate(littlefs_task, "Littlefs Task", LITTLEFS_TASK_STACK_SIZE,
            NULL, LITTLEFS_TASK_PRIORITY, &littlefs_task_handle);

    /* Start the RTOS scheduler. This function should never return */
    vTaskStartScheduler();

    (void) result;

    for (;;)
    {
    }
}

/* [] END OF FILE */
