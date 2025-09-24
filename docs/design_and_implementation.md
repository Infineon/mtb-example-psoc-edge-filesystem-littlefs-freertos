[Click here](../README.md) to view the README.

## Design and implementation

The design of this application is minimalistic to get started with code examples on PSOC&trade; Edge MCU devices. All PSOC&trade; Edge E84 MCU applications have a dual-CPU three-project structure to develop code for the CM33 and CM55 cores. The CM33 core has two separate projects for the secure processing environment (SPE) and non-secure processing environment (NSPE). A project folder consists of various subfolders, each denoting a specific aspect of the project. The three project folders are as follows:

**Table 1. Application projects**

Project | Description
--------|------------------------
*proj_cm33_s* | Project for CM33 secure processing environment (SPE)
*proj_cm33_ns* | Project for CM33 non-secure processing environment (NSPE)
*proj_cm55* | CM55 project

<br>

In this code example, at device reset, the secure boot process starts from the ROM boot with the secure enclave (SE) as the root of trust (RoT). From the secure enclave, the boot flow is passed on to the system CPU subsystem where the secure CM33 application starts. After all necessary secure configurations, the flow is passed on to the non-secure CM33 application. 

Resource initialization for this example is performed by this CM33 non-secure project. It configures the system clocks, pins, clock to peripheral connections, and other platform resources. To conserve power, the CM33 CPU uses a multi-counter watchdog timer (MCWDT) 0 as a low-power timer (LPTIMER). This integration allows the FreeRTOS to enter a tickless idle state, enabling the device to transition into deep sleep when the CPU is idle, minimizing power consumption. It then enables the CM55 core using the `Cy_SysEnableCM55()` function. A FreeRTOS task `cm33_blinky_task` is created, which toggles the 'User LED1' every 1000 milliseconds. 

In this example, CM33 runs a FreeRTOS task that does the following:

1. Mounts the filesystem; if the mounting fails, it formats the storage device and mounts again

2. Opens a file, reads a 32-bit data, increments the value, writes the data into the file, and closes the file

3. Displays the incremented data on the UART terminal

4. Waits for user action

5. If the user button is pressed, it formats the storage device

This example configures `COMPONENTS=RTOS_AWARE` in the *Makefile* to enable RTOS-related features for low-level drivers such as the SDHC HAL. For example, when reading data, the SDHC HAL waits on a semaphore until a read transfer is complete. This example also configures `DEFINES=LFS_THREADSAFE` to enable thread safety in a multi-threaded environment.