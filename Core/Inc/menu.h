/**
 * @file menu.h
 * @author Bastian de Byl (bastian@bdebyl.net)
 * @brief Enhanced menu system for LCD display management
 * @version 0.2
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __MENU_H
#define __MENU_H
#include "lcd.h"
#include <stdint.h>

typedef enum { 
  MENU_RESET = 0x00,    // Startup/reset display
  MENU_MAIN,            // Main operating display (temperature + status)
  MENU_SETTEMP          // Future: Temperature setting menu
} MenuState;

typedef struct {
  LCD_TypeDef *LCD;
  MenuState    State;
  uint32_t     LastUpdate;  // For refresh timing
} Menu_TypeDef;

// Menu system functions
void Menu_Init(Menu_TypeDef *menu, LCD_TypeDef *lcd);
void Menu_HandleState(Menu_TypeDef *menu);
void Menu_SetState(Menu_TypeDef *menu, MenuState newState);
void Menu_Update(Menu_TypeDef *menu);

// State-specific functions
void Menu_StateReset(Menu_TypeDef *menu);
void Menu_StateMain(Menu_TypeDef *menu);

// Display functions for main operating mode
void Menu_DisplayTemperature(Menu_TypeDef *menu, int16_t temperature);
void Menu_DisplayStatus(Menu_TypeDef *menu, int16_t setpoint, char state, uint8_t phaseIndex);
void Menu_DisplayReflowInfo(Menu_TypeDef *menu, int16_t setpoint, char state, uint8_t phaseIndex);

// Display refresh timing
#define MENU_REFRESH_MS 200

#endif // __MENU_H