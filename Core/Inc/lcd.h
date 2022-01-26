/**
 * @file lcd.h
 * @author Bastian de Byl (bastian@bdebyl.net)
 * @brief
 * @version 0.1
 * @date 2022-01-24
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __LCD_H
#define __LCD_H
#if defined(STM32F051x8)
#include "stm32f0xx_hal.h"
#endif // STM32F0x8

/* Defines */
#define LCD_DELAY          (2)

#define LCD_WRMODE_WRITE   (GPIO_PIN_RESET)
#define LCD_WRMODE_READ    (GPIO_PIN_SET)
#define LCD_RSMODE_INST    (GPIO_PIN_RESET)
#define LCD_RSMODE_DATA    (GPIO_PIN_SET)
#define LCD_DISABLE        (GPIO_PIN_RESET)
#define LCD_ENABLE         (GPIO_PIN_SET)

#define LCD_INST_CLR       (0x01U) // Clear entire display
#define LCD_INST_HOM       (0x02U) // Restore cursor home
#define LCD_INST_EMS       (0x04U) // Entry Mode Set (EMS)
#define LCD_INST_EMS_ID    (0x02U) // (EMS) Increment/Decrement
#define LCD_INST_EMS_S     (0x01U) // (EMS) Shift cursor mode
#define LCD_INST_DISP      (0x08U) // Display (DISP)
#define LCD_INST_DISP_ON   (0x04U) // (DISP) Display On/Off
#define LCD_INST_DISP_CURS (0x02U) // (DISP) Cursor
#define LCD_INST_DISP_BLNK (0x01U) // (DISP) Blink Cursor
#define LCD_INST_DCS       (0x10U) // Display/Cursor Shift (DCS)
#define LCD_INST_DCS_SC    (0x08U) // (DCS) Shift Disp / Cursor
#define LCD_INST_DCS_RL    (0x04U) // (DCS) Right/Left
#define LCD_INST_FSET      (0x20U) // Function Set (FSET)
#define LCD_INST_FSET_DL   (0x10U) // (FSET) Data Length
#define LCD_INST_FSET_N    (0x08U) // (FSET) Num. Display Line(s)
#define LCD_INST_FSET_F    (0x04U) // (FSET) Font
#define LCD_INST_RAM       (0x40U) // RAM Address Set
#define LCD_INST_DDRAM     (0x80U) // DDRAM Address Set

/* Exported Types */
typedef enum { LCD_GPIO_DEFAULT = 0x0U, LCD_GPIO_INVERTED } LCD_GPIOState;
typedef enum { LCD_ROW_1 = 0x00U, LCD_ROW_2 = 0x40U } LCD_Row;

typedef struct {
  LCD_Row Row;
  uint8_t Column;
} LCD_PositionTypeDef;

typedef struct {
  GPIO_TypeDef       *GPIODataPort; // Always assume data port is in sequence
  GPIO_TypeDef       *GPIORWPort;
  uint32_t            GPIORWPin;
  GPIO_TypeDef       *GPIOEnPort;
  uint32_t            GPIOEnPin;
  GPIO_TypeDef       *GPIORSPort;
  uint32_t            GPIORSPin;
  uint8_t             EntryModeSet;
  uint8_t             DisplayMode;
  uint8_t             CursorBehavior;
  uint8_t             FunctionSet;
  LCD_PositionTypeDef InitPosition;
  LCD_GPIOState       GPIOState;
  uint8_t             Columns;
  uint8_t             Rows;
} LCD_InitTypeDef;

typedef struct {
  GPIO_TypeDef       *DataPort; // Always assume data port is in sequence
  GPIO_TypeDef       *RWPort;
  uint32_t            RWPin;
  GPIO_TypeDef       *EnPort;
  uint32_t            EnPin;
  GPIO_TypeDef       *RSPort;
  uint32_t            RSPin;
  LCD_PositionTypeDef CurrentPosition;
  LCD_GPIOState       Inverted;
  uint8_t             _columns;
  uint8_t             _rows;
} LCD_TypeDef;

/* Exported Function Prototypes */
void              LCD_Init(LCD_TypeDef *LCD, LCD_InitTypeDef *LCD_Init);
void              LCD_WritePort(LCD_TypeDef *LCD, char Data);
HAL_StatusTypeDef LCD_WriteString(LCD_TypeDef *LCD, char *String,
                                  size_t StringLen);
HAL_StatusTypeDef LCD_WriteChar(LCD_TypeDef *LCD, char Character);
HAL_StatusTypeDef LCD_SetCursorPosition(LCD_TypeDef        *LCD,
                                        LCD_PositionTypeDef Position);
HAL_StatusTypeDef LCD_GoToNextRow(LCD_TypeDef *LCD, uint8_t Column);
void              LCD_SetEnable(LCD_TypeDef *LCD, GPIO_PinState Enable);
void              LCD_SetReadWrite(LCD_TypeDef *LCD, GPIO_PinState RWMode);
void              LCD_SetRegisterSelect(LCD_TypeDef *LCD, GPIO_PinState RSMode);

#endif // __LCD_H