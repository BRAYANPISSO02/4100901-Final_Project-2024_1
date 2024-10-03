#include "keypad.h"
#include "ring_buffer.h"
#include "main.h"


/**
 * @brief: Esta función indica el momento del último tic, es decir,
 * cuando fue rebote (menor a 200ms del anterior), o cuando fue un pulso valido.
 *
 * @param data: nombre de la columna más "_Pin", ejemplo: "COL_1_Pin"
 *
 * @retval: 0 si la presión fué antes de los 200 ms y 1 si fue después.
 */
static uint8_t keypad_debounce(uint16_t GPIO_Pin)
{
	static uint16_t last_pressed = 0xFFFF;
	static uint32_t last_tick = 0;

	if (last_pressed == GPIO_Pin) {
		if (HAL_GetTick() < (last_tick + 300)) {
			return 0;
		}
	}
	last_pressed = GPIO_Pin;
	last_tick = HAL_GetTick();

	return 1;
}


/**
 * @brief: Esta función sirve para devolver el número de la fila presionada (de 1-4)
 *
 * @param data: Primero es el puerto de la columna presionada, el segungo el pin de la columna presionada
 *
 * @retval: Devuelve el número de la fila presionada (de 1-4)
 */
uint8_t keypad_scan_row(GPIO_TypeDef *COLUMNx_GPIO_Port, uint16_t COLUMNx_Pin) //Para el puerto siempre debe de ser tipo "GPIO_TypeDef"
																			   // y el nombre de la variable ser un puntero
{
	uint8_t row_pressed = 0xFF;
	HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_RESET); // "GPIO_PIN_RESET" es una macro que equivale
	if (HAL_GPIO_ReadPin(COLUMNx_GPIO_Port, COLUMNx_Pin) == 0) {   // a "1" (Una macro es una variable global).
		row_pressed =  1;
		goto row_scan_end; // El "goto" lo que nos permite es que el código se siga ejecutando donde esté la etiqueta
	}					   // en este caso "row_scan_end"
	HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(COLUMNx_GPIO_Port, COLUMNx_Pin) == 0) {
		row_pressed =  2;
		goto row_scan_end;
	}
	HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(COLUMNx_GPIO_Port, COLUMNx_Pin) == 0) {
		row_pressed =  3;
		goto row_scan_end;
	}
	HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(COLUMNx_GPIO_Port, COLUMNx_Pin) == 0) {
		row_pressed =  4;
		goto row_scan_end;
	}
row_scan_end:
	HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_SET); // Lo mismo del "GPIO_PIN_RESET" para el "GPIO_PIN_SET"
	HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_SET);
	return row_pressed; // not detected
}

uint8_t keypad_chars[4][4] = { //Creamos una matriz con los valores y el mismo orden que en el teclado
		{'1', '2', '3', 'A'},  //para simplificar código más adelante.
		{'4', '5', '6', 'B'},
		{'7', '8', '9', 'C'},
		{'*', '0', '#', 'D'},
};


/**
 * @brief: Esta función indica el caracter del teclado hexadecimal que se ha presionado
 *
 * @param data: nombre de la columna más "_Pin", ejemplo: "COL_1_Pin"
 *
 * @retval: El caracter presionado si es un pulso valido, o 0xFF si es un rebote
 */
uint8_t keypad_scan(uint16_t GPIO_Pin)
{
	uint8_t key_pressed = 0xFF;

	if (keypad_debounce(GPIO_Pin) == 0) { // Utilizamos la función "keypad_debounce()" para evitar rebotes
		return key_pressed;
	}
	uint8_t row = 0xFF;
	switch (GPIO_Pin) {
	case COLUMN_1_Pin:
		row = keypad_scan_row(COLUMN_1_GPIO_Port, COLUMN_1_Pin); //Usamos la función "keypad_scan_row()"
		key_pressed = keypad_chars[row - 1][1 - 1]; 	   //para saber cuál fue la fila presionada
		break;

	case COLUMN_2_Pin:
		row = keypad_scan_row(COLUMN_2_GPIO_Port, COLUMN_2_Pin);
		key_pressed = keypad_chars[row - 1][2 - 1]; //Llamamos la matríz creada en el valor de la fila y
		break;										//columna que ya calculamos.

	case COLUMN_3_Pin:
			row = keypad_scan_row(COLUMN_3_GPIO_Port, COLUMN_3_Pin);
			key_pressed = keypad_chars[row - 1][3 - 1];
			break;

	case COLUMN_4_Pin:
			row = keypad_scan_row(COLUMN_4_GPIO_Port, COLUMN_4_Pin);
			key_pressed = keypad_chars[row - 1][4 - 1];
			break;
	default:
		break;
	}
	return key_pressed;
}
