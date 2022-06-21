/******************************************************************************
 ******************************************************************************
 ***    UNIVERSIDADE DE BRAS�LIA - UNB                                      ***
 ***    DISCIPLINA: FGA0096 - ELETR�NICA EMBARCADA        TURMA: A          ***
 ***    PROFESSOR: Guillermo Alvarez Bestard, Dr. Eng. Mecatr�nica          ***
 ***    ALUNO: Lucas dos Santos Barros de Sousa   MATR�CULA:180022555       ***
 ***    ALUNO: Matheus Oliveira Dias              MATR�CULA:18/0025104      ***
 ***    ALUNO: Victor Hugo Ciurlini               MATR�CULA:11/0021223      ***
 ***               TRABALHO FINAL ELETR�NICA EMBARCADA                      ***
 ******************************************************************************
 ******************************************************************************/

/* DESCRI��O DE FUNCIONAMENTO*/


//DECLARA��O DE BIBLIOTECAS:
#include "mcc_generated_files/mcc.h"

//DEFINI��ES
#define conv_I  0.449                         //O coeficiente de convers�o da corrente � de 0.5, visto que o valor m�x de cprrente � de 1000 mA, no entanto como teremos que transmitir o valor de corrente/5 para evitar mais operacoes pelo uC j� multiplicamos o valor por 1/5=0.2, logo 0.5*0.2=0.1          
#define conv_temp  0.1                      //O coeficiente de convers�o da temperatura � de 0.1, visto que o valor m�x de temperatura assumido � de 100�C, no entanto como teremos que transmitir o valor de temperatura*3 para evitar mais operacoes pelo uC j� multiplicamos aqui logo 0.1*3=0.3
#define conv_second  0.000004
#define distance_1_pulse  0.75        // 15 mm/20 pulsos = 0.75 mm, no entanto usaremos: 0.75*2 = 1.5 para evitar mais uma opera��o a fim de cumprir o preparo de envio

//DECLARA��O DOS PROT�TIPOS DAS FUN��ES: 
void comunicacao ();
void controle();
void movimento();


//DECLARA��O DAS VARI�VEIS:
int and_dst = 0;                    //Vari�vel para armazenas o andar de destino
int pulses = 0;
int sentido = 0;
float I_m = 0;
float temp_mt = 0;
int aux_tempo = 0;
int estado = 0;
int and_atual = 0;
//------------------------------------------------------------------------------

uint8_t state_motor = 0;
uint8_t and_ating = 0;
uint16_t ccp_value = 0;
uint8_t byte[5];
uint8_t IM_B = 0;
uint8_t TM_B = 0;
uint8_t aux = 0;

//FUN��ES DE INTERRUP��O

//INTERRUP��O PARA ENVIO DOS DADOS A CADA 100mS (5 BYTES)
void send_data()
	{
        aux_tempo++;
        if(aux_tempo == 14)
        {
            int i=0;
            while(i<5)
            {
                if(EUSART_is_tx_ready())
                    {   
                        EUSART_Write(byte[i]);
                        i++;
                    }        
            }
        
            aux_tempo =0;
        }
	}

void sensor1()                          //trata interrup��o do primeiro sensor
    {
        and_ating=0;                    //atualiza variavel usada no gerenciamento (de 1 a 4)
        and_atual = 0;
        pulses = 0;
    }

void sensor2()                          //trata interrup��o do segundo sensor
    {
        and_ating=1;                    //atualiza variavel usada no gerenciamento (de 1 a 4)
        and_atual = 1;
    }

void sensor3()                          //trata interrup��o do terceiro sensor
    {
        and_ating=2;                    //atualiza variavel usada no gerenciamento (de 1 a 4)
        and_atual = 2;
    }

void sensor4()                          //trata interrup��o do quarto sensor
    {
        and_ating=3;                    //atualiza variavel usada no gerenciamento (de 1 a 4)
        and_atual = 3;
    }

void get_pulse(uint16_t capturedValue)
    {
        TMR1_WriteTimer(0);
        ccp_value = capturedValue;
        if(Dir_GetValue()==1){
            pulses++;
        }else if(Dir_GetValue() == 0 && pulses >= 0){
            pulses--;
        }
    }

// APLICA��O PROPRIAMENTE DITA
void main(void)
{
    
  //FUN��ES DE INICIALIZA��O
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
   //PASSAGEM DE PAR�METROS PARA FUN��ES DE INTERRUP��O 
    TMR4_SetInterruptHandler(send_data);
    IOCBF0_SetInterruptHandler(sensor1);
    IOCBF3_SetInterruptHandler(sensor2);
    IOCBF4_SetInterruptHandler(sensor3);
    IOCBF5_SetInterruptHandler(sensor4);
    CCP4_SetCallBack(get_pulse);

    while(S1_GetValue()!=0){
        Dir_SetLow();
        PWM3_LoadDutyValue(512);
    }
    PWM3_LoadDutyValue(0);
    
    // LOOP PRINCIPAL
    while (1)
    {
        I_m = ADC_GetConversion(0);          // Realiza a leitura da corrente, converte para mA usando *conv_I, j� esta preparado para envio, veja as defini��es! (Resolu��o da medi��o 0.5 mA )
        IM_B = (uint8_t)((conv_I*I_m)/4);
        temp_mt = ADC_GetConversion(1);   // Realiza a leitura da temperatura, converte para C usando *conv_temp, j� esta preparado para envio, veja as defini��es! (Resolu��o da medi��o 0.1 �C))
        TM_B = (uint8_t)(conv_temp*temp_mt*2);
        
        comunicacao ();    // Chama a fun��o respons�vel por tratar a comunica��o
        controle();   // Chama a fun��o respons�vel pelo controle do motor
    }
}

void comunicacao ()
{
    float speed = (distance_1_pulse/(((float)ccp_value)*conv_second))*4;
    float position = distance_1_pulse*pulses;
    
    if(EUSART_is_rx_ready())                                            
        {
            aux= EUSART_Read();
            and_dst = (int)(aux & 0x02);
        }
    
    byte[0] = ((state_motor<<4)& 0x20)|(and_ating & 0x02); //conferido
    byte[1] = 0x80 |((((uint8_t)position)>>1)& 0x7F); // conferido
    byte[2] = 0x80 |((((uint8_t)speed)>>1)& 0x7F); //conferido
    byte[3] = 0x80 |(((IM_B)>>1)& 0x7F); //conferido
    byte[4] = 0x80 |(((TM_B)>>1)& 0x7F); //conferido          
}

void controle()                                 //rotina respons�vel por controlar o motor
{
    if(and_dst>and_atual){
        if(Dir_GetValue()==0){    
            PWM3_LoadDutyValue(0);
            LedG_SetHigh();
            LedR_SetLow();
            __delay_ms(500);
        }
        Dir_SetHigh();
        estado = 1;
        movimento();
        
    } else if(and_dst<and_atual){
        if(Dir_GetValue()==1){
            PWM3_LoadDutyValue(0);
            LedG_SetHigh();
            LedR_SetLow();
            __delay_ms(500);
        }
        Dir_SetLow();
        estado = 2;
        movimento();
    }

    if(and_dst==and_atual){
        estado = 0;
        LedG_SetHigh();
        LedR_SetLow();
        and_dst=0;
    }
}

void movimento()
{
//    distancia = abs(and_atual-and_dst);
    int mean_dutyValue = 512;
    int destiny = and_dst*80;                 // Vari�vel para receber o andar de destino 
    
    LedR_SetHigh();
    LedG_SetLow();
     
    while(and_atual!=and_dst){
        
        PWM3_LoadDutyValue(mean_dutyValue);
    }
    
    PWM3_LoadDutyValue(0);
    estado = 0;
    LedG_SetHigh();
    LedR_SetLow();
    __delay_ms(2000);
}