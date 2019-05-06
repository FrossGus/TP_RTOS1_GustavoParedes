/* Copyright 2017-2018, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// sAPI header
#include "sapi.h"
#include "sapi_rtc.h"
#include "sapi_datatypes.h"
#include "sapi_board.h"

//Strings
#include "string.h"
#include "stdlib.h"

//SPI
#include "sd_spi.h"   // <= own header (optional)
#include "ff.h"       // <= Biblioteca FAT FS
#include "fssdc.h"

#include  "ciaaI2C.h"
#include  "funciones.h"

#define UART_PC        UART_USB
#define UART_BLUETOOTH UART_232

//*==================[definiciones y macros]==================================*/
#define BLOQUES 240    //Tamaño del bloque de almacenamiento (ingresan 20 lineas de datos por archivo)
#define INIT 0

/*==================[definiciones de datos internos]=========================*/
xQueueHandle cola;			// Nombre de la cola
static FATFS fs;           // <-- FatFs work area needed for each volume
static FIL fp;             // <-- File object needed for each open file
/*==================[definiciones de datos externos]=========================*/

/* Buffer */
static char uartBuff[10];

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/
void diskTickHook( void *ptr );

// Prototipos de funciones de las tareas
void TareaTXRXBluetooth ( void* taskParmPtr );
void TaskWriteData      ( void* taskParmPtr );
void TaskdiskTickHook   ( void* taskParmPtr );
void TareaRX_RTC   		( void* taskParmPtr );

/*==================[declaraciones de funciones externas]====================*/
 void diskTickHook( void *ptr ){
   disk_timerproc();   // Disk timer process
}

/*==================[funcion principal]======================================*/

	bool_t val_RTC = 0;
	bool_t grabar = FALSE;

int main(void)
{

   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   // Inicializar UART_USB para conectar al monitor serie de la PC con CuteCom
   uartConfig( UART_USB, 115200 );
   printf ("UART_USB configurada en 115200 baudios.\r\n");

   // Inicializar UART_232 para conectar al modulo bluetooth
   uartConfig( UART_BLUETOOTH, 9600 );
   printf("UART_BLUETOOTH configurada en 9600 baudios.\r\n" );

   // Iniciamos bus I2C.
   ciaaI2CInit();

   // Configuro el reloj.
   		rtc.year = 2019;
   		rtc.month = 5;
   		rtc.mday = 5;
   		rtc.wday = 7;
   		rtc.hour = 15;
   		rtc.min = 52;
   		rtc.sec= 0;

	// Inicializar RTC
	val_RTC = rtcConfig( &rtc );
	if (val_RTC == FALSE){
		printf("No se pudo configurar el RTC.\r\n");
	}
	else {
		printf("RTC configurado.\r\n");
	}

	// Establecer fecha y hora
	val_RTC = rtcRead( &rtc );
	if (val_RTC == FALSE){
		printf("No se pudo leer el RTC.\r\n");
	}
	else {
	printf("RTC leido correctamente.\r\n");
	printf("Son las %d hs %d minutos del dia %d/%d/%d.\r\n",
			rtc.hour,
			rtc.min,
			rtc.mday,
			rtc.month,
			rtc.year);
	}

	//Creamos la cola
	cola = xQueueCreate(BLOQUES, sizeof(uint8_t));
	if( cola == NULL )
	   {
		   printf ("Cola no creada.\r\n");
	   }
	else {
	   printf ("Cola creada.\r\n");
	}

	// Crear tarea en freeRTOS recepción de datos de la App por medio de Bluetooth
	printf ("Creacion de tarea TXRXBluetooth\r\n");
	xTaskCreate(
	TareaTXRXBluetooth,                       // Funcion de la tarea
	  (const char *)"TareaTXRXBluetooth",     // Nombre de la tarea
	  configMINIMAL_STACK_SIZE*2,             // Cantidad de stack
	  0,                                      // Parametros
	  tskIDLE_PRIORITY+4,                     // Prioridad
	  0                                       // Puntero
	);

	// Crear tarea en freeRTOS controlador de memoria SD
	printf ("Creacion de tarea TaskWriteData\r\n");
	xTaskCreate(
	 TaskWriteData,                       // Funcion de la tarea
	 (const char *)"TaskWriteData",       // Nombre de la tarea
	 configMINIMAL_STACK_SIZE*2,          // Cantidad de stack
	 0,                                   // Parametros
	 tskIDLE_PRIORITY+3,                  // Prioridad
	 0                                    // Puntero
	);

	// Crear tarea en freeRTOS controlador de escritura en SD
	printf ("Creacion de la tarea TasdiskTickHook\r\n");
	xTaskCreate(
	TaskdiskTickHook,                       // Funcion de la tarea
	  (const char *)"TaskdiskTickHook",    // Nombre de la tarea
	  configMINIMAL_STACK_SIZE*2,          // Cantidad de stack
	  0,                                   // Parametros
	  tskIDLE_PRIORITY+3,                  // Prioridad
	  0                                    // Puntero
	);

	// Crear tarea en freeRTOS lectura reloj RTC
	printf ("Creacion de tarea RX_RTC\r\n");
	xTaskCreate(
	TareaRX_RTC,                       // Funcion de la tarea
	  (const char *)"TareaRX_RTC",     // Nombre de la tarea
	  configMINIMAL_STACK_SIZE*2,             // Cantidad de stack
	  0,                                      // Parametros
	  tskIDLE_PRIORITY+3,                     // Prioridad
	  0                                       // Puntero
	);

	// Iniciar scheduler
	vTaskStartScheduler();
	printf ("Inicio Scheduler.\r\n");

	// ---------- REPETIR POR SIEMPRE --------------------------
	while( TRUE ) {
	  // Si cae en este while 1 significa que no pudo iniciar el scheduler
	  printf ("No pudo inicializar el Scheduler !!!\r\n");
	}
	return 0;
}
/*===========================================================================*/

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/


//***************************** ##################################### ******************
// Implementacion de funcion de la tarea
void TareaTXRXBluetooth( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------

	uint8_t msjLinea[13];// Arreglo con la linea a grabar.
	uint8_t dataBluetooth = 0;
	uint8_t dataBluetooth_X = 84;// Precarga una T, Y y Z como dato para el switch case
	uint8_t dataBluetooth_Y = 85;// Asi comienza automaticamente el intercambio de datos
	uint8_t dataBluetooth_Z = 86;// entre la App y la EDUCIAA.

	// Tarea periodica cada 10 ms
	portTickType xPeriodicity =  10 / portTICK_RATE_MS;
	portTickType xLastWakeTime = xTaskGetTickCount();

	// ---------- REPETIR POR SIEMPRE --------------------------

	while( TRUE ) {

        if( uartReadByte( UART_BLUETOOTH, &dataBluetooth ) ) {

      	  switch (dataBluetooth) {

      	  // El comienzo de la grabacion se visualiza de dos maneras:
      	  // En la App se indica con el cambio de color del boton (rojo a verde) y por la indicacion de las
      	  // letras que se reciben desde la EDUCIAA, las cuales son normalmente T, U y V, pero cuando se
      	  // esta grabando son A, B y C.
      	  // En la EDUCIAA se indica con el led RGB en color AZUL.

      	  case 'G':// Detecta que se presiono el boton de grabar en la App. Comienza grabacion.
      		  grabar = TRUE;
      		  gpioWrite( LEDB, ON );
      		  dataBluetooth_X = 65;// Envia una A como dato.
      		  dataBluetooth_Y = 66;// Envia una B como dato.
      		  dataBluetooth_Z = 67;// Envia una C como dato.

      		  cargaMsj(msjLinea);// Carga el msj en el arreglo.
  				for(uint8_t h=0;h<13;h++){
  					xQueueSendToBack(cola, &msjLinea[h], portMAX_DELAY);// Envia el arreglo por la cola.
  				}

      		  break;

      	  case 'S':// Detecta que se despresiono el boton de grabar en la App. Detiene grabacion.
      		  grabar = FALSE;
      		  gpioWrite( LEDB, OFF );
      		  dataBluetooth_X = 84;// Envia una T como dato.
      		  dataBluetooth_Y = 85;// Envia una U como dato.
      		  dataBluetooth_Z = 86;// Envia una V como dato.
      		  break;

      	  case 'X':// Detecta si la App pide el dato de X y responde con una T.
      		  gpioWrite( LED1, ON );
      		  uartWriteByte( UART_BLUETOOTH, dataBluetooth_X );
      	      break;

      	  case 'Y':// Detecta si la App pide el dato de Y y responde con una U.
      		  gpioWrite( LED2, ON );
      		  uartWriteByte( UART_BLUETOOTH, dataBluetooth_Y );
      		  //printf( "Valor enviado por Y = %d\r\n",dataBluetooth_Y );
      		  break;

      	  case 'Z':// Detecta si la App pide el dato de Z y responde con una V.
      		  gpioWrite( LED3, ON );
      	   	  uartWriteByte( UART_BLUETOOTH, dataBluetooth_Z );
      	   	  break;

      	  default:
      		  gpioWrite( LED1, OFF );// Apaga todos los leds de X, Y y Z.
      		  gpioWrite( LED2, OFF );
      		  gpioWrite( LED3, OFF );
      	  }
        }

        // Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
        vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
     }
}

//***************************** ##################################### ******************
 void TaskWriteData( void* taskParmPtr ){

     // ---------- CONFIGURACIONES ------------------------------
	 // SPI configuration
	 spiConfig( SPI0 );

	 // ------ PROGRAMA QUE ESCRIBE EN LA SD -------
	 UINT nbytes;

	 // Initialize SD card driver
	 FSSDC_InitSPI ();

	 // Give a work area to the default drive
	 if( f_mount( &fs, "SDC:", 0 ) != FR_OK ) { // si entra aqui es por falla SD. Revisar tarjeta
		 printf ("Error de SD Card mal conectada\r\n");
	 }

	 int din,ret,desSize=BLOQUES;
	 uint16_t i=0;
	 uint16_t j=0;
	 uint8_t datoRcv;

	 //asignación de memoria dinámica para la recepción de caracteres x cola.
	 uint8_t * dataIn = malloc(BLOQUES * sizeof(uint8_t));

	 uint8_t msj[40];   //msj para el nombre del archivo

      // Tarea periodica cada 5 ms
      portTickType xPeriodicity =  5 / portTICK_RATE_MS;
      portTickType xLastWakeTime = xTaskGetTickCount();

      // ---------- REPETIR POR SIEMPRE --------------------------
      while(TRUE) {
    	  //Recibe los bytes y los almacena en un array de 1024 valores.
    	  if (xQueueReceive(cola, &datoRcv, portMAX_DELAY) == pdTRUE) {
    		  *(dataIn+i)=datoRcv;
    		  gpioWrite( LEDR, ON );
    		  i++;
    	  }

    	  // Si el buffer interno de recepción se lleno comprimo y copio los datos.
    	  if(i==BLOQUES)                                  // Buffer lleno entonces vuelco a memoria
    		  if (rtcRead( &rtc)){   //Lectura DS1307
    		  	 nombreArchivo(msj);      //guardo archivo con los datos y nombre "XYZ...fecha...txt"
    		     printf("Archivo a guardar en microSD: %s \r\n",msj);

    		    //Creo archivo
    		  if( f_open( &fp, msj, FA_WRITE | FA_OPEN_APPEND ) == FR_OK )
    		     f_write( &fp,dataIn,BLOQUES, &nbytes );
    		  if( nbytes == BLOQUES)
    			  gpioWrite( LEDG, ON );

    		  f_close(&fp);
    		  i=INIT; // preparo para recibir otro bloque de datos
    		  printf("Datos grabados correctamente.\r\n");
    		  }

    	  gpioWrite( LEDB, OFF );
    	  gpioWrite( LEDG, OFF );
    	  // Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
    	  vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
      }
}

 //***************************** ##################################### ******************
 void TaskdiskTickHook( void* taskParmPtr ){

       // ---------- CONFIGURACIONES ------------------------------

       // Tarea periodica cada 10 ms
       portTickType xPeriodicity =  10 / portTICK_RATE_MS;
       portTickType xLastWakeTime = xTaskGetTickCount();

       // ---------- REPETIR POR SIEMPRE --------------------------
       while(TRUE) {
    	   disk_timerproc();   // Disk timer process

    	   // Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
    	   vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
       }
 }

 //***************************** ##################################### ******************
void TareaRX_RTC( void* taskParmPtr ){

	    // ---------- CONFIGURACIONES ------------------------------

	    // Tarea periodica cada 10 ms
	    portTickType xPeriodicity =  5 / portTICK_RATE_MS;
	    portTickType xLastWakeTime = xTaskGetTickCount();

    // ---------- REPETIR POR SIEMPRE --------------------------
    while(TRUE) {

           // Leer fecha y hora
           	val_RTC = rtcRead( &rtc );

			// Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
			vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
    }
}

/*==================[fin del archivo]========================================*/
