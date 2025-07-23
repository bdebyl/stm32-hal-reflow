/**
 * @file menu.c
 * @author Bastian de Byl (bastian@bdebyl.net)
 * @brief Enhanced menu system for LCD display management
 * @version 0.2
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "menu.h"
#include "main.h"  // For HAL_GetTick()
#include <string.h>
#include <stdio.h>

// Static display strings and formatting
static char MenuResetString[] = " (c) 2022 bdebyl";
static char TempFormatStr[] = "%3dC";
static char StatusFormatStr[] = "%d %c%d ";
static char TempLabelStr[] = "Temperature: ";
static char StatusLabelStr[] = "Set: ";

// Static variables for main display state
static LCD_PositionTypeDef TempValuePos;
static LCD_PositionTypeDef StatusValuePos;
static uint8_t MainDisplayInitialized = 0;

/**
 * @brief Initialize the menu system
 * @param menu Menu structure to initialize
 * @param lcd LCD instance to use
 */
void Menu_Init(Menu_TypeDef *menu, LCD_TypeDef *lcd) {
  menu->LCD = lcd;
  menu->State = MENU_RESET;
  menu->LastUpdate = 0;
  MainDisplayInitialized = 0;
}

/**
 * @brief Handle current menu state
 * @param menu Menu structure
 */
void Menu_HandleState(Menu_TypeDef *menu) {
  switch (menu->State) {
  case MENU_RESET:
    Menu_StateReset(menu);
    break;
  case MENU_MAIN:
    Menu_StateMain(menu);
    break;
  default:
    menu->State = MENU_RESET;
    break;
  }
}

/**
 * @brief Set menu to a new state
 * @param menu Menu structure
 * @param newState New state to set
 */
void Menu_SetState(Menu_TypeDef *menu, MenuState newState) {
  if (menu->State != newState) {
    menu->State = newState;
    menu->LastUpdate = 0;
    MainDisplayInitialized = 0;
  }
}

/**
 * @brief Update menu display if needed (called from main loop)
 * @param menu Menu structure
 */
void Menu_Update(Menu_TypeDef *menu) {
  uint32_t currentTime = HAL_GetTick();
  if ((currentTime - menu->LastUpdate) >= MENU_REFRESH_MS) {
    Menu_HandleState(menu);
    menu->LastUpdate = currentTime;
  }
}

/**
 * @brief Handle reset/startup display state
 * @param menu Menu structure
 */
void Menu_StateReset(Menu_TypeDef *menu) {
  LCD_ClearScreen(menu->LCD);
  LCD_PositionTypeDef resetPos = {0, LCD_ROW_1};
  LCD_SetCursorPosition(menu->LCD, resetPos);
  LCD_WriteString(menu->LCD, MenuResetString, strlen(MenuResetString));
}

/**
 * @brief Handle main operating display state
 * @param menu Menu structure
 */
void Menu_StateMain(Menu_TypeDef *menu) {
  if (!MainDisplayInitialized) {
    LCD_ClearScreen(menu->LCD);
    LCD_PositionTypeDef pos = {0, LCD_ROW_1};
    LCD_SetCursorPosition(menu->LCD, pos);
    LCD_WriteString(menu->LCD, TempLabelStr, strlen(TempLabelStr));
    pos.Row = LCD_ROW_2;
    LCD_SetCursorPosition(menu->LCD, pos);
    LCD_WriteString(menu->LCD, StatusLabelStr, strlen(StatusLabelStr));
    TempValuePos.Column = strlen(TempLabelStr);
    TempValuePos.Row = LCD_ROW_1;
    StatusValuePos.Column = strlen(StatusLabelStr);
    StatusValuePos.Row = LCD_ROW_2;
    MainDisplayInitialized = 1;
  }
}

/**
 * @brief Display current temperature value
 * @param menu Menu structure
 * @param temperature Temperature value to display
 */
void Menu_DisplayTemperature(Menu_TypeDef *menu, int16_t temperature) {
  if (menu->State == MENU_MAIN && MainDisplayInitialized) {
    char tempStr[8];
    LCD_SetCursorPosition(menu->LCD, TempValuePos);
    sprintf(tempStr, TempFormatStr, temperature);
    LCD_WriteString(menu->LCD, tempStr, strlen(tempStr));
  }
}

/**
 * @brief Display current status information (setpoint, state, phase)
 * @param menu Menu structure
 * @param setpoint Current temperature setpoint
 * @param state Current reflow state character ('I', 'R', 'H')
 * @param phaseIndex Current reflow phase index
 */
void Menu_DisplayStatus(Menu_TypeDef *menu, int16_t setpoint, char state, uint8_t phaseIndex) {
  if (menu->State == MENU_MAIN && MainDisplayInitialized) {
    char statusStr[16];
    LCD_SetCursorPosition(menu->LCD, StatusValuePos);
    sprintf(statusStr, StatusFormatStr, setpoint, state, phaseIndex);
    LCD_WriteString(menu->LCD, statusStr, strlen(statusStr));
  }
}

/**
 * @brief Display reflow information (alias for Menu_DisplayStatus)
 * @param menu Menu structure
 * @param setpoint Current temperature setpoint
 * @param state Current reflow state character ('I', 'R', 'H')
 * @param phaseIndex Current reflow phase index
 */
void Menu_DisplayReflowInfo(Menu_TypeDef *menu, int16_t setpoint, char state, uint8_t phaseIndex) {
  Menu_DisplayStatus(menu, setpoint, state, phaseIndex);
}