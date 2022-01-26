/**
 * @file lcd.c
 * @author Bastian de Byl (bastian@bdebyl.net)
 * @brief
 * @version 0.1
 * @date 2022-01-24
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "lcd.h"

void LCD_Init(LCD_TypeDef *LCD, LCD_InitTypeDef *LCD_Init) {
  LCD->DataPort = LCD_Init->GPIODataPort;
  LCD->RWPort   = LCD_Init->GPIORWPort;
  LCD->RWPin    = LCD_Init->GPIORWPin;
  LCD->EnPort   = LCD_Init->GPIOEnPort;
  LCD->EnPin    = LCD_Init->GPIOEnPin;
  LCD->RSPort   = LCD_Init->GPIORSPort;
  LCD->RSPin    = LCD_Init->GPIORSPin;
  LCD->_rows    = LCD_Init->Rows;
  LCD->_columns = LCD_Init->Columns;
  LCD->Inverted = LCD_Init->GPIOState;

  // Initialize Sequence for LCD
  LCD_SetEnable(LCD, LCD_DISABLE);
  LCD_SetReadWrite(LCD, LCD_WRMODE_WRITE);
  LCD_SetRegisterSelect(LCD, LCD_RSMODE_INST);

  LCD_WritePort(LCD, LCD_INST_FSET | LCD_Init->FunctionSet);
  LCD_WritePort(LCD, LCD_INST_DISP | LCD_Init->DisplayMode);
  LCD_WritePort(LCD, LCD_INST_CLR);

  // Set initial cursor position
  LCD->CurrentPosition.Column = LCD_Init->InitPosition.Column;
  LCD->CurrentPosition.Row    = LCD_Init->InitPosition.Row;

  if ((LCD->CurrentPosition.Column == 0) &&
      (LCD->CurrentPosition.Row == LCD_ROW_1)) {
    LCD_WritePort(LCD, LCD_INST_HOM);
  } else {
    LCD_SetCursorPosition(LCD, LCD_Init->InitPosition);
  }
}

void LCD_WritePort(LCD_TypeDef *LCD, char Data) {
  uint8_t _data;

  switch (LCD->Inverted) {
  case LCD_GPIO_INVERTED:
    _data = ~(Data);
    break;
  default:
    _data = Data;
    break;
  }

  LCD->DataPort->ODR = (LCD->DataPort->ODR & 0xFF00) | _data;

  HAL_Delay(LCD_DELAY);
  LCD_SetEnable(LCD, GPIO_PIN_SET);
  HAL_Delay(LCD_DELAY);
  LCD_SetEnable(LCD, GPIO_PIN_RESET);
}

void LCD_SetEnable(LCD_TypeDef *LCD, GPIO_PinState Enable) {
  GPIO_PinState _enable = Enable;

  if (LCD->Inverted == LCD_GPIO_INVERTED) {
    switch (_enable) {
    case GPIO_PIN_SET:
      _enable = GPIO_PIN_RESET;
      break;
    default:
      _enable = GPIO_PIN_SET;
      break;
    }
  }

  HAL_GPIO_WritePin(LCD->EnPort, LCD->EnPin, _enable);
}

void LCD_SetReadWrite(LCD_TypeDef *LCD, GPIO_PinState RWMode) {
  uint8_t _rwMode = RWMode;

  if (LCD->Inverted == LCD_GPIO_INVERTED) {
    switch (_rwMode) {
    case GPIO_PIN_SET:
      _rwMode = GPIO_PIN_RESET;
      break;
    default:
      _rwMode = GPIO_PIN_SET;
      break;
    }
  }

  HAL_GPIO_WritePin(LCD->RWPort, LCD->RWPin, _rwMode);
}
void LCD_SetRegisterSelect(LCD_TypeDef *LCD, GPIO_PinState RSMode) {
  uint8_t _rsMode = RSMode;

  if (LCD->Inverted == LCD_GPIO_INVERTED) {
    switch (_rsMode) {
    case GPIO_PIN_SET:
      _rsMode = GPIO_PIN_RESET;
      break;
    default:
      _rsMode = GPIO_PIN_SET;
      break;
    }
  }

  HAL_GPIO_WritePin(LCD->RSPort, LCD->RSPin, _rsMode);
}

HAL_StatusTypeDef LCD_SetCursorPosition(LCD_TypeDef        *LCD,
                                        LCD_PositionTypeDef Position) {
  if (Position.Column < LCD->_columns) {
    LCD->CurrentPosition.Column = Position.Column;
  } else {
    return HAL_ERROR;
  }
  LCD->CurrentPosition.Row = Position.Row;

  LCD_SetRegisterSelect(LCD, LCD_RSMODE_INST);
  LCD_WritePort(LCD, LCD_INST_DDRAM | (LCD->CurrentPosition.Row +
                                       LCD->CurrentPosition.Column));

  return HAL_OK;
}

HAL_StatusTypeDef LCD_GoToNextRow(LCD_TypeDef *LCD, uint8_t Column) {
  LCD_PositionTypeDef newPos;

  newPos.Column = Column;
  switch (LCD->CurrentPosition.Row) {
  case LCD_ROW_1:
    newPos.Row = LCD_ROW_2;
    break;
  default:
    newPos.Row = LCD_ROW_1;
    break;
  }

  if (LCD_SetCursorPosition(LCD, newPos) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef LCD_WriteChar(LCD_TypeDef *LCD, char Character) {
  // Ensure we set it to write
  LCD_SetReadWrite(LCD, LCD_WRMODE_WRITE);

  // Check if current position would overflow
  if (LCD->CurrentPosition.Column >= LCD->_columns) {
    if (LCD_GoToNextRow(LCD, 0) != HAL_OK) {
      return HAL_ERROR;
    }
  }

  switch (Character) {
  case '\r':
    LCD->CurrentPosition.Column = 0;
    if (LCD_SetCursorPosition(LCD, LCD->CurrentPosition) != HAL_OK) {
      return HAL_ERROR;
    }
    break;
  case '\n':
    if (LCD_GoToNextRow(LCD, LCD->CurrentPosition.Column) != HAL_OK) {
      return HAL_ERROR;
    }
    break;
  default:
    // Ensure re-set to data
    LCD_SetRegisterSelect(LCD, LCD_RSMODE_DATA);
    // Write the character to the screen
    LCD_WritePort(LCD, Character);

    // Increment the column position
    LCD->CurrentPosition.Column++;
    break;
  }

  return HAL_OK;
}

HAL_StatusTypeDef LCD_WriteString(LCD_TypeDef *LCD, char *String,
                                  size_t StringLen) {
  HAL_StatusTypeDef status = HAL_OK;
  size_t            i;
  for (i = 0; i < StringLen; i++) {
    status = LCD_WriteChar(LCD, String[i]);

    if (status != HAL_OK) {
      return status;
    }
  }

  return status;
}