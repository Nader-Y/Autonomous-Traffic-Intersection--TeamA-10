#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/* =========================================================
   AXI-LITE REGISTER MAPPING (FPGA INTERFACE)
   ========================================================= */

#define FPGA_BASE       0x40000000

#define CONTROL_REG     (*(volatile unsigned int *)(FPGA_BASE + 0x00))
#define REQUEST_ID_REG  (*(volatile unsigned int *)(FPGA_BASE + 0x04))
#define STATUS_REG      (*(volatile unsigned int *)(FPGA_BASE + 0x08))
#define GRANT_REG       (*(volatile unsigned int *)(FPGA_BASE + 0x0C))
#define QUAD_REG        (*(volatile unsigned int *)(FPGA_BASE + 0x10))

/* =========================================================
   SIMPLE QUEUE (FCFS SCHEDULING)
   ========================================================= */

#define MAX_QUEUE 10

int queue[MAX_QUEUE];
int front = 0;
int rear  = 0;

void enqueue(int v)
{
    queue[rear++] = v;
    if (rear >= MAX_QUEUE) rear = 0;
}

int dequeue()
{
    int v = queue[front++];
    if (front >= MAX_QUEUE) front = 0;
    return v;
}

int queue_empty()
{
    return front == rear;
}

/* =========================================================
   SIMULATED INPUT FUNCTIONS
   (replace later with real V2X input)
   ========================================================= */

int receive_vehicle_id()
{
    static int id = 1;
    id++;
    if (id > 12) id = 1;
    return id;
}

int emergency_detected()
{
    return 0; // placeholder (can be replaced with sensor/V2X flag)
}

/* =========================================================
   TASK 1: V2X RECEIVER (INPUT LAYER)
   ========================================================= */

void V2XReceiverTask(void *pvParameters)
{
    while (1)
    {
        int req = receive_vehicle_id();
        enqueue(req);

        printf("Vehicle received: %d\n", req);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* =========================================================
   TASK 2: SCHEDULER (FCFS LOGIC)
   Sends requests to FPGA
   ========================================================= */

void SchedulerTask(void *pvParameters)
{
    while (1)
    {
        if (!queue_empty())
        {
            int req = dequeue();

            REQUEST_ID_REG = req;   // send movement ID to FPGA
            CONTROL_REG = 1;        // start processing

            printf("Sent to FPGA: %d\n", req);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* =========================================================
   TASK 3: FPGA INTERFACE MONITOR (UPDATED)
   Reads FPGA response and waits for execution completion
   ========================================================= */

void FPGAInterfaceTask(void *pvParameters)
{
    while (1)
    {
        if (CONTROL_REG == 1)
        {
            if (GRANT_REG == 1)
            {
                printf("FPGA: GRANTED. Vehicle entering intersection...\n");
                
                /* 
                 * NEW: The VHDL now requires the start signal to be held high.
                 * We must poll STATUS_REG bit 0 (done signal) to know when 
                 * the hardware slot timer has finished.
                 */
                while ((STATUS_REG & 0x01) == 0) 
                {
                    /* Yield to other tasks while the FPGA timer runs */
                    vTaskDelay(pdMS_TO_TICKS(1)); 
                }
                
                printf("FPGA: EXECUTION COMPLETE. Releasing intersection.\n");
            }
            else
            {
                printf("FPGA: REJECTED (conflict)\n");
            }

            /* Reset the cycle only after completion or rejection */
            CONTROL_REG = 0; 
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
/* =========================================================
   TASK 4: EMERGENCY OVERRIDE (HIGHEST PRIORITY)
   ========================================================= */

void EmergencyTask(void *pvParameters)
{
    while (1)
    {
        if (emergency_detected())
        {
            CONTROL_REG = 0xFF; // emergency override signal
            printf("EMERGENCY ACTIVE\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* =========================================================
   TASK 5: MONITOR / DEBUG TASK
   ========================================================= */

void MonitorTask(void *pvParameters)
{
    while (1)
    {
        int qstate = QUAD_REG;
        int status = STATUS_REG;

        printf("QuadState: %d | Status: %d\n", qstate, status);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* =========================================================
   MAIN FUNCTION (RTOS START)
   ========================================================= */

int main(void)
{
    /* Task creation */
    xTaskCreate(EmergencyTask, "EMG", 256, NULL, 5, NULL);
    xTaskCreate(SchedulerTask, "SCH", 512, NULL, 3, NULL);
    xTaskCreate(V2XReceiverTask, "V2X", 512, NULL, 2, NULL);
    xTaskCreate(FPGAInterfaceTask, "FPGA_IF", 512, NULL, 2, NULL);
    xTaskCreate(MonitorTask, "MON", 256, NULL, 1, NULL);

    /* Start scheduler */
    vTaskStartScheduler();

    while (1);
}