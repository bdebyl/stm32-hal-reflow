/**
 * @file menu.c
 * @author Bastian de Byl (bastian@bdebyl.net)
 * @brief
 * @version 0.1
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "menu.h"
#include <string.h>

static char *MenuResetString = " Copyright (c) 2022\r\n bastian@bdebyl.net";

/**
 * @brief
 *
 * @param menu
 */
void Menu_HandleState(Menu_TypeDef *menu) {
  switch (menu->State) {
  case MENU_RESET:
    Menu_StateReset(menu);
    break;
  case MENU_MAIN:

    break;
  default:
    // We should never be here
    menu->State = MENU_RESET;
    break;
  }
}

void Menu_StateReset(Menu_TypeDef *menu) {
  LCD_ClearScreen(menu->LCD);

  LCD_PositionTypeDef _resetPos = {.Column = 0, .Row = LCD_ROW_1};
  LCD_SetCursorPosition(menu->LCD, _resetPos);

  LCD_WriteString(menu->LCD, MenuResetString, strlen(MenuResetString));
}

void Menu_StateReset(Menu_TypeDef *menu) {
  LCD_ClearScreen(menu->LCD);

  LCD_PositionTypeDef _resetPos = {.Column = 0, .Row = LCD_ROW_1};
  LCD_SetCursorPosition(menu->LCD, _resetPos);

  LCD_WriteString(menu->LCD, MenuResetString, strlen(MenuResetString));
}