/**
 * @file menu.h
 * @author Bastian de Byl (bastian@bdebyl.net)
 * @brief
 * @version 0.1
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __MENU_H
#define __MENU_H
#include "lcd.h"

typedef enum { MENU_RESET = 0x00, MENU_MAIN, MENU_SETTEMP } MenuState;

typedef struct {
  LCD_TypeDef *LCD;
  MenuState    State;
} Menu_TypeDef;

void Menu_HandleState(Menu_TypeDef *menu);

void Menu_StateReset(Menu_TypeDef *menu);
void Menu_StateMain(Menu_TypeDef *menu);
#endif // __MENU_H