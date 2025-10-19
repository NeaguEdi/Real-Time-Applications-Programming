/* Standard includes. */
#include <stdio.h>
#include <math.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

/* Demo application includes. */
#include "BlockQ.h"
#include "crflash.h"
#include "blocktim.h"
#include "integer.h"
#include "comtest2.h"
#include "partest.h"
//#include "lcd.h"
#include "timertest.h"

// Includes proprii
#include "new_lcd.h"
#include "adcDrv1.h"
#include "new_serial.h"
#include "libq.h"
#include "ds18s20.h"

//#include "serial.h"

/* Demo task priorities. */
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainCOM_TEST_PRIORITY				( 2 )

/* The check task may require a bit more stack as it calls sprintf(). */
#define mainCHECK_TAKS_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )

/* The execution period of the check task. */
#define mainCHECK_TASK_PERIOD				( ( portTickType ) 3000 / portTICK_RATE_MS )

/* The number of flash co-routines to create. */
#define mainNUM_FLASH_COROUTINES			( 5 )

/* Baud rate used by the comtest tasks. */
//#define mainCOM_TEST_BAUD_RATE				( 19200 )
#define mainCOM_TEST_BAUD_RATE				( 9600 )

// Definire lungime coada UART1
#define comBUFFER_LEN						( 10 )

/* We should find that each character can be queued for Tx immediately and we
don't have to block to send. */
#define comNO_BLOCK					( ( portTickType ) 0 )

/* The Rx task will block on the Rx queue for a long period. */
#define comRX_BLOCK_TIME			( ( portTickType ) 0xffff )

/* The LED used by the comtest tasks.  mainCOM_TEST_LED + 1 is also used.
See the comtest.c file for more information. */
#define mainCOM_TEST_LED					( 6 )

/* The frequency at which the "fast interrupt test" interrupt will occur. */
#define mainTEST_INTERRUPT_FREQUENCY		( 20000 )

/* The number of processor clocks we expect to occur between each "fast
interrupt test" interrupt. */
#define mainEXPECTED_CLOCKS_BETWEEN_INTERRUPTS ( configCPU_CLOCK_HZ / mainTEST_INTERRUPT_FREQUENCY )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned short ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Dimension the buffer used to hold the value of the maximum jitter time when
it is converted to a string. */
#define mainMAX_STRING_LENGTH				( 20 )

// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC);
// Enable Clock Switching and Configure
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF);		// FRC + PLL
//_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);		// XT + PLL
_FWDT(FWDTEN_OFF); 		// Watchdog Timer Enabled/disabled by user software

/*
 * Setup the processor ready for the demo.
 */
static void prvSetupHardware( void );

/* The queue used to send messages to the LCD task. */
//static xQueueHandle xUART1_Queue;
	unsigned int stare_aplicatie=0;//aplicatia este initial oprita
	unsigned int mod_lucru=1;//predefinit modul de lucru automat
    char sir1[5]="",sir2[5]="",ultima_comanda;
	xTaskHandle hT_LCD,hT_Seriala,hT_Temp,hT_Tens,hT_PWMAuto,hT_PWMMan;
	static xQueueHandle qH;
	#define QUEUE_SIZE 10
	int ok;
	extern int val;
	typedef struct 
	{
	float temp,adc_read;
	}mesaj;
	
void TaskLCD(void *params) 
{
	for (;;)
		{
				
		if(mod_lucru==1)
		{		
			LCD_line(1);
			LCD_printf("Mod lucru:");
			LCD_printf("auto");	
			LCD_line(2);
			LCD_printf("Temp:");
			LCD_printf(sir1);
		}	
		else
		{
			LCD_line(1);
			LCD_printf("Mod lucru:");
			LCD_printf("man ");
			LCD_line(2);
			LCD_printf("Tens:");
			LCD_printf(sir2);
		}
		LCD_line(4);
		LCD_printf("Ultima com:");
		send_char2LCD(ultima_comanda);
		
		vTaskDelay(5000);
		clear();
		}
		
}

void TaskSeriala(void *params) 
{
	signed char cByteRxed;
	vSerialPutString( NULL, (const signed char * const)"1-interogare mod de lucru\n 2-comutare mod de lucru\n 3-interogare temperatura\n", comNO_BLOCK );
	for (;;)
		{
		if( xSerialGetChar( NULL, &cByteRxed, comRX_BLOCK_TIME ) )
			{
				if(cByteRxed=='1') //interogare mod
				{
					if(mod_lucru==0)
					{
						vSerialPutString( NULL, (const signed char * const)"Mod de lucru manual\n", comNO_BLOCK );
						vTaskSuspend(hT_PWMAuto);
						vTaskSuspend(hT_Temp);
						vTaskResume(hT_PWMMan);
						vTaskResume(hT_Tens);
					}
					else
					{
						vSerialPutString( NULL, (const signed char * const)"Mod de lucru automat\n", comNO_BLOCK );
						vTaskSuspend(hT_PWMMan);
						vTaskSuspend(hT_Tens);
						vTaskResume(hT_PWMAuto);
						vTaskResume(hT_Temp);
					}
					ultima_comanda='1';		
				}
				if(cByteRxed=='2') //comutare mod
				{
					mod_lucru=!mod_lucru;
					if(mod_lucru==1)
					{
						vSerialPutString( NULL, (const signed char * const)"Comutat in mod de lucru automat\n", comNO_BLOCK );
						vTaskSuspend(hT_PWMMan);
						vTaskSuspend(hT_Tens);
						vTaskResume(hT_PWMAuto);
						vTaskResume(hT_Temp);
					}
					else
					{
						vSerialPutString( NULL, (const signed char * const)"Comutat in mod de lucru manual\n", comNO_BLOCK );
						vTaskSuspend(hT_PWMAuto);
						vTaskSuspend(hT_Temp);
						vTaskResume(hT_PWMMan);
						vTaskResume(hT_Tens);
					}
					ultima_comanda='2';		
				}
				if(cByteRxed=='3' && mod_lucru==1) //interogare temperatura
				{
						vSerialPutString( NULL, (const signed char * const)sir1, comNO_BLOCK );
						vSerialPutString( NULL, (const signed char * const)"\n", comNO_BLOCK );
						ultima_comanda='3';
				}			
			}
		}
}

void initINT0()
{
 _TRISB7 = 1; // RB7 este setat ca intrare  
 _INT0IF = 0; // Resetem flagul coresp. intreruperii INT0  
 _INT0IE = 1; // Se permite lucrul cu întreruperea INT0    
 _INT0EP = 1; // Se stabileste frontul negativ pentru apasarea de buton
}

void __attribute__ ((interrupt, no_auto_psv)) _INT0Interrupt(void)
{
 stare_aplicatie++; // la fiecare apasare de buton, starea aplicatiei se schimba(oprita/pornita) 
 _INT0IF = 0;// Resetam flagul corespunzator intreruperii INT0 pentru a nu se reapela rutina de intrerupere
}  

void TaskApp(void *params) 
{
	for (;;)
	{
		if(stare_aplicatie%2 == 0) 
		{	
			_RB11 =~_RB11; //LED aprins intermitent cand aplicatia este oprita
			vTaskDelay(500); //Perioada de 500ms a delay-ului
			vTaskSuspend(hT_LCD);
			vTaskSuspend(hT_Seriala);
			vTaskSuspend(hT_Temp);
			vTaskSuspend(hT_Tens);
			vTaskSuspend(hT_PWMAuto);
			vTaskSuspend(hT_PWMMan);

		}
		else if(stare_aplicatie %2 ==1)
		{
			 _RB11 =0; //LED aprins atunci cand aplciatia este pornita
			vTaskResume(hT_LCD);
			vTaskResume(hT_Seriala);
			vTaskResume(hT_Temp);
			vTaskResume(hT_Tens);
			vTaskResume(hT_PWMAuto);
			vTaskResume(hT_PWMMan);
		}	
	}
}

void TaskTemp(void *params)
{
	mesaj m;
	for(;;)
	{
		
		m.temp = ds1820_read();
		_itoaQ15((int)m.temp,sir1);
		/*LCD_line(1);
		LCD_printf("Temp:");
		LCD_printf(sir1);	*/
		xQueueSend(qH, &m, portMAX_DELAY);
		vTaskDelay(500);
	} 
}

void TaskTens(void *params)
{
	mesaj m;
	for(;;)
	{
		m.adc_read=((double)val/4095)*(3.3);	
		_itoaQ15((int)m.adc_read,sir2);
		/*LCD_line(1);
		LCD_printf("Tens:");
		LCD_printf(sir2);*/
		xQueueSend(qH,&m,portMAX_DELAY);	
		vTaskDelay(500);
	}	
}

void TaskPWM_Manual(void *params)
{
	mesaj receivedMesaj;
	for(;;)
	{
		if(xQueueReceive(qH,&receivedMesaj,portMAX_DELAY))
		{
			if(receivedMesaj.adc_read >=1 && receivedMesaj.adc_read <=3)
			{
				P1DC3=(int)(1250.0+((receivedMesaj.adc_read-1.0)/2.0)*1232.0);
			}	
			vTaskDelay(100);
		}	
	}	
}

void TaskPWM_Automat(void *params)
{
	mesaj receivedMesaj;
	for(;;)
	{
		if(xQueueReceive(qH,&receivedMesaj,portMAX_DELAY))
		{
			if(receivedMesaj.temp >=20 && receivedMesaj.temp <=30)
			{
				P1DC3=(int)(1250.0+((receivedMesaj.temp-20.0)/10.0)*1232.0);
			}	
			vTaskDelay(100);
		}	
	}	
}			


void initPWM1()
{

 P1TCONbits.PTOPS = 0; // Timer base output scale
 P1TCONbits.PTMOD = 0; // Free running
 P1TMRbits.PTDIR = 0; // Numara in sus pana cand timerul = perioada
 P1TMRbits.PTMR = 0; // Baza de timp


 PWM1CON1bits.PMOD3 = 1; // Canalele PWM3H si PWM3L sunt indep

 PWM1CON1bits.PEN3H=1; //Pinul PWM1H setat pe iesire PWM
 PWM1CON1bits.PEN3L=0; //Pinul PWM1L setat pe iesire PWM
 P1DC3 = 0x04E2;
 P1TPER = 0x30D4;
 P1TCONbits.PTCKPS=0b11;//prescaler 64 de biti

 PWM1CON2bits.UDIS = 0; // Disable Updates from duty cycle and period buffers
 P1TCONbits.PTEN = 1; /* Enable the PWM Module */
} 

void initializare_temperatura()
{
	_CN6PUE = 1;
	_TRISB2 = 1; //_RB2 setat ca intrare
	output_float();
	ONE_WIRE_PIN = 1;
}	

int main( void )
{
	prvSetupHardware();
	qH=xQueueCreate(QUEUE_SIZE,sizeof(mesaj));	
	xTaskCreate(TaskLCD, (signed portCHAR *) "TsLCD", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &hT_LCD);
	xTaskCreate(TaskSeriala, (signed portCHAR *) "TsSeriala", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, &hT_Seriala);
	xTaskCreate(TaskApp, (signed portCHAR *) "TsApp", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(TaskTemp, (signed portCHAR *) "TsTemp", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &hT_Temp);
	xTaskCreate(TaskTens, (signed portCHAR *) "TsTens", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &hT_Tens);
	xTaskCreate(TaskPWM_Automat, (signed portCHAR *) "TsPWMAuto", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &hT_PWMAuto);
	xTaskCreate(TaskPWM_Manual, (signed portCHAR *) "TsPWMMan", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &hT_PWMMan);
	
	/* Finally start the scheduler. */
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

void initPLL(void)
{
// Configure PLL prescaler, PLL postscaler, PLL divisor
	PLLFBD = 41; 		// M = 43 FRC
	//PLLFBD = 30; 		// M = 32 XT
	CLKDIVbits.PLLPOST=0; 	// N1 = 2
	CLKDIVbits.PLLPRE=0; 	// N2 = 2

// Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
	__builtin_write_OSCCONH(0x01);	// FRC
	//__builtin_write_OSCCONH(0x03);	// XT
	__builtin_write_OSCCONL(0x01);

// Wait for Clock switch to occur
	while (OSCCONbits.COSC != 0b001);	// FRC
	//while (OSCCONbits.COSC != 0b011);	// XT

// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1) {};
}

static void prvSetupHardware(void)
{
	ADPCFG = 0xFFFF;				//make ADC pins all digital - adaugat
	PORTB=0XF000;
	vParTestInitialise();
	initPLL();
	initializare_temperatura();
	LCD_init();
	initAdc1();
	initTmr3();
	initINT0(); //apelam intreruperea cand apasam butonul S2
	_TRISB11 = 0; //_RB11 setat ca iesire
	initPWM1();
	_TRISB2=1; //_RB2 setat ca intrare
	_TRISB3=1; //_RB3 setat ca intrare
	_TRISB1=0; //_RB1 setat ca iesire
	_TRISB10=0; //_RB10 setat ca iesire
	
	
	// Initializare interfata UART1
	xSerialPortInitMinimal( mainCOM_TEST_BAUD_RATE, comBUFFER_LEN );
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* Schedule the co-routines from within the idle task hook. */
	vCoRoutineSchedule();
}
/*-----------------------------------------------------------*/
