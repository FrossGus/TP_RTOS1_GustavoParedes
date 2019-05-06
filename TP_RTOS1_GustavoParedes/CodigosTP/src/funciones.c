

#include "funciones.h"
#include "ciaaI2C.h"
#include "sapi_datatypes.h"
#include "sapi_rtc.h"
#include <stdio.h>
#include <string.h>


	rtc_t rtc;// Creo rtc de tipo rtc_t

void cargaMsj(uint8_t *msj){
	// Carga en el arreglo la linea a grabar
	// X=A Z=B Y=C
	// Deberian haber ido los datos de Pitch, Roll y Azimuth del Giroscopio del TP.

  	msj[0]=88;  //X
  	msj[1]=61;  //=
  	msj[2]=65;   //A
  	msj[3]=32;  //" "
  	msj[4]=89;  //Y
  	msj[5]=61;  //=
  	msj[6]=66;  //B
  	msj[7]=32;  //" "
  	msj[8]=90;  //Z
  	msj[9]=61;  //=
  	msj[10]=67; //C
   	msj[11]=13; //\r
  	msj[12]=10; //\n
  }

 void nombreArchivo(uint8_t *msj){

 	int8_t filename[40]="SDC:/";

 	msj[0]='X';
 	msj[1]='Y';
 	msj[2]='Z';
 	msj[3]='_';
 	msj[4]=50;  //Año 20xx
 	msj[5]=48;
 	msj[6]=((rtc.year-2000)/10)+48;
 	msj[7]=((rtc.year-2000)%10)+48;

 	msj[8]=95;   //guion bajo

 	msj[9]=(rtc.month/10)+48;  //Mes
 	msj[10]=(rtc.month%10)+48;

 	msj[11]=95;   //guion bajo

 	msj[12]=(rtc.mday/10)+48;   //dia
 	msj[13]=(rtc.mday%10)+48;

 	msj[14]=95;   //guion bajo

 	msj[15]=(rtc.hour/10)+48;   //hora
 	msj[16]=(rtc.hour%10)+48;

 	msj[17]=95;   //guion bajo

 	msj[18]=(rtc.min/10)+48;   //minutos
 	msj[19]=(rtc.min%10)+48;

 	msj[20]=95;   //guion bajo

 	msj[21]=(rtc.sec/10)+48;   //segundos
 	msj[22]=(rtc.sec%10)+48;
 	msj[23]='.';
 	msj[24]='t';
 	msj[25]='x';
 	msj[26]='t';
 	msj[27]='\0';
 	strcat(filename,msj );
 	strcpy(msj,filename);
 }
