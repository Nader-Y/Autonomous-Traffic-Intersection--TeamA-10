#ifndef V2X_DRIVER_H
#define V2X_DRIVER_H

#include <stdint.h>

/* =========================================================
   V2X / SENSOR NETWORK HARDWARE ABSTRACTION LAYER
   ========================================================= */

/**
 * @brief Initializes the hardware UART interface to receive 
 *        data from the multi-node camera/sensor network.
 */
void V2X_Init(void);

/**
 * @brief Blocks until a valid vehicle movement ID (1-12) is 
 *        received and validated from the sensor network.
 * @return integer representing the movement ID.
 */
int receive_vehicle_id(void);

/**
 * @brief Polls the hardware for an emergency vehicle override signal.
 * @return 1 if an emergency vehicle is detected, 0 otherwise.
 */
int emergency_detected(void);

#endif /* V2X_DRIVER_H */