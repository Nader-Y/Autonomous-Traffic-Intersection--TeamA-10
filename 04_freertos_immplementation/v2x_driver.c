#include "v2x_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "xparameters.h"
#include "xuartps.h" 

/* =========================================================
   HARDWARE CONFIGURATION CONSTANTS
   ========================================================= */
#define UART_DEVICE_ID     XPAR_XUARTPS_0_DEVICE_ID
#define SENSOR_BAUD_RATE   115200

/* Custom Binary Protocol for Edge AI YOLO Sensor Nodes */
#define FRAME_START_BYTE   0xAA
#define FRAME_END_BYTE     0x55
#define MIN_CONFIDENCE     75    /* Minimum YOLO detection confidence % */

static XUartPs Uart_Ps;

/* =========================================================
   INITIALIZATION
   ========================================================= */
void V2X_Init(void) 
{
    XUartPs_Config *Config;
    
    /* Look up the hardware configuration for the Zynq PS UART */
    Config = XUartPs_LookupConfig(UART_DEVICE_ID);
    if (Config == NULL) {
        return; // Initialization failed
    }

    /* Initialize the hardware driver */
    XUartPs_CfgInitialize(&Uart_Ps, Config, Config->BaseAddress);
    
    /* Set Baud Rate to match the Raspberry Pi / Sensor Node */
    XUartPs_SetBaudRate(&Uart_Ps, SENSOR_BAUD_RATE);
}

/* =========================================================
   SENSOR INGESTION (Replaces Simulated Loop)
   ========================================================= */
int receive_vehicle_id(void) 
{
    uint8_t rx_buffer[5];
    int movement_id = 0;
    
    /* Infinite loop: Blocks until a valid vehicle is detected */
    while (1) 
    {
        /* Check if data is available in the UART hardware FIFO */
        if (XUartPs_IsReceiveData(Uart_Ps.Config.BaseAddress)) 
        {
            rx_buffer[0] = XUartPs_RecvByte(Uart_Ps.Config.BaseAddress);
            
            /* Check for the Start of Frame */
            if (rx_buffer[0] == FRAME_START_BYTE) 
            {
                /* Block and read the remaining 4 bytes of the payload */
                for(int i = 1; i < 5; i++) 
                {
                    while(!XUartPs_IsReceiveData(Uart_Ps.Config.BaseAddress)) {
                        /* Yield to FreeRTOS Scheduler while waiting for UART bits */
                        vTaskDelay(pdMS_TO_TICKS(1)); 
                    }
                    rx_buffer[i] = XUartPs_RecvByte(Uart_Ps.Config.BaseAddress);
                }
                
                /* 
                 * Parse Payload:
                 * rx_buffer[1] = Node ID (Camera 1-4)
                 * rx_buffer[2] = Movement ID (1-12)
                 * rx_buffer[3] = Confidence (0-100)
                 */
                
                /* Validate the frame structure and the AI confidence threshold */
                if (rx_buffer[4] == FRAME_END_BYTE && rx_buffer[3] >= MIN_CONFIDENCE) 
                {
                    movement_id = rx_buffer[2];
                    
                    /* Final safety clamp: Ensure the ID matches the FPGA VHDL requirements */
                    if (movement_id >= 1 && movement_id <= 12) 
                    {
                        return movement_id;
                    }
                }
            }
        }
        
        /* Yield to other FreeRTOS tasks to prevent CPU starvation */
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

/* =========================================================
   EMERGENCY OVERRIDE (Replaces Simulated Loop)
   ========================================================= */
int emergency_detected(void) 
{
    /* 
     * In a physical system, this could read a dedicated GPIO pin 
     * connected to an acoustic siren sensor, or parse a specific 
     * high-priority V2X DSRC packet. 
     */
    
    // Example: return XGpio_DiscreteRead(&Gpio, 1);
    
    return 0; 
}