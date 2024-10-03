
#include "ring_buffer.h"
#include "main.h"
#include "val_clave.h"

//extern uint8_t pantalla_x;
//extern uint8_t pantalla_y;

/*
 * @brief Esta funcion valida un buffer que se ingrese si coincide con la clave que está predefinida
 *
 * @param data: Recibe el nombre de la variable tipo "ring_buffer_t" (estructura creada).
 *
 * @retval: 1 si la clave NO coincide, 0 si la clave coincide
 */

uint8_t validar_clave(ring_buffer_t *buffer) {
	char clave_correcta[] = "122334";
    uint8_t tamano = ring_buffer_size(buffer);

    if (tamano != (sizeof(clave_correcta)-1)) {
//    	  ssd1306_Fill(Black);
//		  ssd1306_SetCursor(pantalla_x, pantalla_y);
//		  ssd1306_WriteString("CLAVE INCORRECTA", Font_7x10, White);
//		  ssd1306_UpdateScreen();
        return 1; // Tamaño no coincide
    }

    for (uint8_t i = 0; i < tamano; i++) {
        uint8_t clave_val;
        if(ring_buffer_read(buffer, &clave_val) == 0){
//        	  ssd1306_Fill(Black);
//			  ssd1306_SetCursor(pantalla_x, pantalla_y);
//			  ssd1306_WriteString("CLAVE INCORRECTA", Font_7x10, White);
//			  ssd1306_UpdateScreen();
        	return 1;//buffer vacio
        }

        if (clave_val != clave_correcta[i]) {
//            ssd1306_Fill(Black);
//		  ssd1306_SetCursor(pantalla_x, pantalla_y);
//		  ssd1306_WriteString("CLAVE INCORRECTA", Font_7x10, White);
//		  ssd1306_UpdateScreen();
		  return 1; // Clave incorrecta
        }
    }

//    ssd1306_Fill(Black);
//  ssd1306_SetCursor(pantalla_x, pantalla_y);
//  ssd1306_WriteString("CLAVE CORRECTA", Font_7x10, White);
//  ssd1306_UpdateScreen();
    return 0; // Clave correcta
}
