/*
 * File:   SPI_S10.c
 * Author: Cristian Catú
 *
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000
#define FLAG_SPI 0x0F
#define IN_MIN 0           
#define IN_MAX 255        
#define OUT_MIN 40       
#define OUT_MAX 130

/*------------------------------------------------------------------------------
 * VARIABLES
 ------------------------------------------------------------------------------*/
uint8_t pot = 0;
uint8_t cont_slave = 0;
char val_temporal = 0;
unsigned short CCPR = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max,
            unsigned short out_min, unsigned short out_max);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PORTAbits.RA0){                  // ¿Es maestro?
        // interrupciones del maestro
        if(PIR1bits.ADIF){              // Fue interrupci?n del ADC?
            pot = ADRESH;
            PIR1bits.ADIF = 0;          // Limpiamos bandera de interrupci?n
        }
    }
    else {                              // ¿Es esclavo?
        // interrupciones del esclavo
        if(PIR1bits.SSPIF){             // ¿Recibió datos el esclavo?
            val_temporal = SSPBUF;
            if (val_temporal != FLAG_SPI){  // Es envío solo para generar los pulsos de reloj?
                PORTD = val_temporal;       // Mostramos valor recibido en PORTD
                
                SSPBUF = cont_slave;        // Cargamos contador del esclavo al buffer
            }
            else {
                SSPBUF = cont_slave;
            }
            PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción
        }
        else if(INTCONbits.RBIF){            // Fue interrupción del PORTB
            if(!PORTBbits.RB0)                     // Verificamos si fue RB0 quien generó la interrupción
                cont_slave++;                // Incrementar PORTA
            else if (!PORTBbits.RB1)               // Si es RB2
                cont_slave--;                // Decrementar PORTA
            INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción
        }
        
    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if(PORTAbits.RA0){ 
            if(ADCON0bits.GO == 0){ 
                ADCON0bits.GO = 1;       
            }
            
            SSPBUF = pot; //Se envia el valor del potenciometro
            while(!SSPSTATbits.BF){} //Se espera el envio
            
            PORTAbits.RA7 = 1;  //Se habilita y desabilita el esclavo por que es necesario en los PICS
            __delay_ms(10);  
            PORTAbits.RA7 = 0; 
            __delay_ms(10);

            SSPBUF = FLAG_SPI;  //Se envia una señal cualquiera para que el esclavo retorne el valor que desea enviar

            while(!SSPSTATbits.BF){} //Se espera lo que recibr
            PORTD = SSPBUF;    //se muestra lo que se recibio en el PUERTOD
            __delay_ms(10);
        }
        else{
            CCPR = map(PORTD, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
        }
        if(PORTAbits.RA2){
            PORTD = 0;//es lo que no queremos que muestre el esclavo 2
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0x02;         
    ANSELH = 0;            

    OSCCONbits.IRCF = 0b100; 
    OSCCONbits.SCS = 1;     

    TRISA = 0b00100111;  
    PORTA = 0;

    

    TRISD = 0;
    PORTD = 0;

    if(PORTAbits.RA0){

        // Configuraciones del SPI
        TRISC = 0b00010000;      
        PORTC = 0;

        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0000;  
        SSPCONbits.CKP = 0;         
        SSPCONbits.SSPEN = 1;     
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        
        SSPSTATbits.SMP = 1;     
        SSPBUF = pot;            


        // Configuraci?n ADC
        ADCON0bits.ADCS = 0b00;     // Fosc/2
        ADCON1bits.VCFG0 = 0;       // VDD
        ADCON1bits.VCFG1 = 0;       // VSS
        ADCON0bits.CHS = 1;    // Seleccionamos el AN1
        ADCON1bits.ADFM = 0;        // Justificado a la izquierda
        ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
        __delay_us(40);             // Sample time

        // Configuracion interrupciones
        PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
        PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
        INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
        INTCONbits.GIE = 1;         // Habilitamos int. globales
    }
    // Configs del esclavo
    else{
        TRISC = 0b00011000; 
        PORTC = 0;
        TRISB = 0xFF;           


        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0100;  
        SSPCONbits.CKP = 0;     
        SSPCONbits.SSPEN = 1;  
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;   
        SSPSTATbits.SMP = 0;     

        PIR1bits.SSPIF = 0; 
        PIE1bits.SSPIE = 1;     
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;
        
        OPTION_REGbits.nRBPU = 0; 
        WPUB = 0x03;
        INTCONbits.RBIE = 1;   
        IOCB = 0x03;         
        INTCONbits.RBIF = 0;
        
        // Configuración PWM
        TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
        PR2 = 250;                  // periodo de 2ms

        // Configuración CCP
        CCP1CON = 0;                // Apagamos CCP1
        CCP1CONbits.P1M = 0;        // Modo single output
        CCP1CONbits.CCP1M = 0b1100; // PWM

        CCPR1L = 5>>2;
        CCP1CONbits.DC1B = 5 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo

        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
        T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
        T2CONbits.TMR2ON = 1;       // Encendemos TMR2
        while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente

        TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM

    }
}
// función para el mapeo
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}