#ifndef _MAIN_H_
#define _MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "variants.h"  // for RCC_ClocksTypeDef
#include "uart.h"      // for _UART_CNT

#define MAX_MENU_DEPTH 10  // max sub menu depth

typedef void (* FP_MENU)(void);

typedef struct
{
  FP_MENU menu[MAX_MENU_DEPTH];  // menu function buffer
  uint8_t cur;                   // current menu index in buffer
} MENU;

typedef enum
{
  HOST_STATUS_IDLE = 0,
  HOST_STATUS_PRINTING,
  HOST_STATUS_RESUMING,
  HOST_STATUS_PAUSED,
  HOST_STATUS_PAUSING
} HOST_STATUS;

typedef enum
{
  HOST_SLOTS_GENERIC_OK = -2,
  HOST_SLOTS_REGULAR_OK,
} HOST_SLOTS;

typedef struct
{
  uint8_t tx_slots;    // keep track of available gcode tx slots (e.g. if ADVANCED_OK feature is enabled on both mainboard and TFT)
  uint8_t tx_count;    // keep track of pending gcode tx count
  bool connected;      // whether have connected to Marlin
  HOST_STATUS status;  // whether the host is busy in printing execution. (USB serial printing and gcode print from onboard)
} HOST;

typedef struct
{
  RCC_ClocksTypeDef rccClocks;
  uint32_t PCLK1_Timer_Frequency;
  uint32_t PCLK2_Timer_Frequency;
} CLOCKS;

extern MENU infoMenu;
extern HOST infoHost;
extern CLOCKS mcuClocks;

void InfoHost_Init(bool isConnected);

// handle OK response:
//   - tx_slots (used/effective only in case "advanced_ok" configuration setting is also enabled in TFT):
//     - HOST_SLOTS_GENERIC_OK: to increase infoHost.tx_slots and decrease infoHost.tx_count by 1 respectively
//     - HOST_SLOTS_REGULAR_OK: to handle static ADVANCED_OK
//     - >= 0: to handle Marlin ADVANCED_OK
void InfoHost_HandleOkAck(int16_t tx_slots);

#ifdef __cplusplus
}
#endif

#endif
