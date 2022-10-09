/**
* \version 1.0
* \author bazhen.levkovets
* \date 2022-Oct-09
*************************************************************************************
* \copyright	Bazhen Levkovets
* \copyright	Brovary
* \copyright	Ukraine
*************************************************************************************
*/

/*
**************************************************************************
*							INCLUDE FILES
**************************************************************************
*/
	#include	"gnss_nmea_sm.h"
	#include <string.h>
	#include "stdio.h"
/*
**************************************************************************
*							LOCAL DEFINES
**************************************************************************
*/

//	uint8_t		TF_buffer[0xff] = {0} ;

/*
**************************************************************************
*							LOCAL CONSTANTS
**************************************************************************
*/

/*
**************************************************************************
*						    LOCAL DATA TYPES
**************************************************************************
*/


/*
**************************************************************************
*							  LOCAL TABLES
**************************************************************************
*/

/*
**************************************************************************
*								 MACRO'S
**************************************************************************
*/


/*
**************************************************************************
*						 LOCAL GLOBAL VARIABLES
**************************************************************************
*/

/*
**************************************************************************
*                        LOCAL FUNCTION PROTOTYPES
**************************************************************************
*/

/*
**************************************************************************
*                           GLOBAL FUNCTIONS
**************************************************************************
*/

/*
**************************************************************************
*                           LOCAL FUNCTIONS
**************************************************************************
*/

uint32_t RingBuffer_DMA_Count(RingBuffer_DMA * buffer) {
	// get counter returns the number of remaining data units in the current DMA Stream transfer (total size - received count)
	// current head = start + (size - received count)
	uint8_t const * head = buffer->data + buffer->size - __HAL_DMA_GET_COUNTER(buffer->hdma);
	uint8_t const * tail = buffer->tail_ptr;
	if (head >= tail)
		return head - tail;
	else
		return head - tail + buffer->size;
} //***********************************************************************

uint8_t RingBuffer_DMA_GetByte(RingBuffer_DMA * buffer) {
	// get counter returns the number of remaining data units in the current DMA Stream transfer (total size - received count)
	// current head = start + (size - received count)
	uint8_t const * head = buffer->data + buffer->size - __HAL_DMA_GET_COUNTER(buffer->hdma);
	uint8_t const * tail = buffer->tail_ptr;

	if (head != tail) {
		uint8_t c = *buffer->tail_ptr++;
		if (buffer->tail_ptr >= buffer->data + buffer->size) {
			buffer->tail_ptr -= buffer->size;
		}
		return c;
	}

	return 0;
} //***********************************************************************

void RingBuffer_DMA_Init(RingBuffer_DMA * buffer, DMA_HandleTypeDef * hdma, uint8_t * data, uint32_t size){
	buffer->data = data; // set array
	buffer->size = size; // and its size
	buffer->hdma = hdma; // initialized DMA
	buffer->tail_ptr = data; // tail == head == start of array
} //***********************************************************************


void NMEA_Parse(uint8_t *buf, uint8_t len) {
#ifdef OK
	char *p;

	uint32_t tmp;
	float tmp2;
	p = (char *)buf;

	if (	( !strncmp(p, "$GNGGA", 6))
		||	( !strncmp(p, "$GPGGA", 6)) ) {
		p += 7;

		uint8_t hour = (p[0] - '0') * 10 + p[1] - '0';
		uint8_t min = (p[2] - '0') * 10 + p[3] - '0';
		uint8_t sec = (p[4] - '0') * 10 + p[5] - '0';

		//IBUS_Telemetry.alt = (uint32_t) TelGlubina;
		//IBUS_Telemetry.alt = gpsdata.min*100 + gpsdata.sec ;
		if (p[6] == '.')
			gpsdata.hsec = (p[7] - '0') * 10 + p[8] - '0';
		else
			gpsdata.hsec = 0;

		static uint8_t previous_hsec_u8;
		uint8_t NMEA_period ;

		if ( gpsdata.hsec > previous_hsec_u8) {
			NMEA_period = gpsdata.hsec - previous_hsec_u8;
			previous_hsec_u8 = gpsdata.hsec ;
		} else {
			NMEA_period = gpsdata.hsec + 100 - previous_hsec_u8;
			previous_hsec_u8 = gpsdata.hsec ;
		}

		char 		uart_buff_char[0xFF];
		sprintf(uart_buff_char,"p7:%d p8:%d hsec:%d previ:%d period:%d\r\n" ,
									p[7] - '0' , p[8] - '0',
									gpsdata.hsec,
									previous_hsec_u8,
									NMEA_period 				) ;
		HAL_UART_Transmit( &huart3, (uint8_t *)uart_buff_char , strlen(uart_buff_char) , 100 ) ;
		IBUS_Telemetry.alt = NMEA_period ;



		p = strstr(p, ",") + 1;
		// parse lat
		tmp = NMEA_atoi(p);
		tmp2 = latG(tmp);

		p = strstr(p, ",") + 1;
		if (p[0] == 'S') {
			tmp = -tmp;
			tmp2 = -tmp2;
		}
		//gpsdata.lat = 10*tmp;
		gpsdata.lat = tmp;
		//gpsdata.latG = tmp2;

		p = strstr(p, ",") + 1;
		// parse lon
		tmp = NMEA_atoi(p);
		//tmp2 = lonG(p);
		p = strstr(p, ",") + 1;
		if (p[0] == 'W') {
			tmp = -tmp;
			tmp2 = -tmp2;
		}
		//gpsdata.lon = 10*tmp;
		gpsdata.lon = tmp;

		//gpsdata.lonG = tmp2;
		p = strstr(p, ",") + 1;
		gpsdata.valid = (p[0] - '0')?1:0;

		p = strstr(p, ",") + 1;

		//if (!SettingsMode) gpsdata.sats = (p[0] - '0') * 10 + p[1] - '0';
		if (!SettingsMode)	{
			if (p[1] == ',') {
				gpsdata.sats = p[0] - '0' ;
			} else {
				gpsdata.sats = (p[0] - '0') * 10 + p[1] - '0';
			}
		}

//		//char 		uart_buff_char[0xFF];
//		sprintf(uart_buff_char,"sats: %c %c\r\n" , p[0] , p[1] ) ;
//		HAL_UART_Transmit( &huart3, (uint8_t *)uart_buff_char , strlen(uart_buff_char) , 100 ) ;


		if (!SettingsMode) IBUS_Telemetry.satellitesVisible = (uint8_t) gpsdata.sats;

		p = strstr(p, ",") + 1;

		gpsdata.hdop = NMEA_atoi(p);

		p = strstr(p, ",") + 1;
		// alt
		gpsdata.alt = NMEA_atoi(p);


	} else if(!strncmp(p, "$GNRMC", 6)||!strncmp(p, "$GPRMC", 6)) {
		p += 7;
		gpsdata.hour = (p[0] - '0') * 10 + p[1] - '0';
		gpsdata.min = (p[2] - '0') * 10 + p[3] - '0';
		gpsdata.sec = (p[4] - '0') * 10 + p[5] - '0';
		UpdateAutoCorrect=1;
		UpdateAutoCorrect2=1;
		UpdateAutoCorrect3 = 1;

//		if(p[6] == '.')
//			gpsdata.hsec = (p[7]-'0')*10 + p[8]-'0';
//		else
//			gpsdata.hsec=0;

		p = strstr(p, ",") + 1;
		gpsdata.valid = (p[0] == 'A')?1:0;

		p = strstr(p, ",") + 1;
		// parse lat
		tmp = NMEA_atoi(p);
		p = strstr(p, ",") + 1;

		if (p[0] == 'S')
			tmp = -tmp;

		gpsdata.lat = tmp;

		p = strstr(p, ",")+1;
		// parse lon
		tmp = NMEA_atoi(p);
		p=strstr(p, ",") + 1;
		if (p[0] == 'W')
			tmp = -tmp;
		gpsdata.lon = tmp;

		p = strstr(p, ",") + 1;

		gpsdata.speed = NMEA_atoi(p)/2;

		p = strstr(p, ",") + 1;

		gpsdata.heading = NMEA_atoi(p) / 100;



		IBUS_Telemetry.heading = (uint16_t) gpsdata.heading;
		p = strstr(p, ",") + 1;
		// date
		gpsdata.day = (p[0] - '0') * 10 + p[1] - '0';
		gpsdata.month = (p[2] - '0') * 10 + p[3] - '0';
		gpsdata.year = (p[4] - '0') * 10 + p[5] - '0';
	}
#endif
}

uint32_t NMEA_atoi(char *p)
{
	uint32_t out = 0;

	while ((*p >= '0' && *p <= '9') || *p == '.')
	{
		if (*p == '.') {
			p++;
			continue;
		}
		out *= 10;
		out += *p - '0';
		p++;
	}

	return out;
}
