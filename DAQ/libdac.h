//inicialização da biblioteca 
#ifndef LIBDAC_H
#define LIBDAC_H
#include "string.h"
#include "stdio.h"

//Variáveis globais
long int B5;
float ax_off, ay_off, az_off, mx_adj, my_adj, mz_adj, US_1, US_2, distance;

//Constantes globais
extern static const uint8_t MPU9250_AD = 0x68 << 1; //Converte o endereço de 7 bits para 8 bits
extern static const uint8_t MAG_AD = 0x0C << 1;
extern static const uint8_t BMP_AD = 0x77 << 1;//Endereço do bmp180; pag.20 do datasheet

extern static const uint8_t ACE_CONFIG_AD = 0x1C ;
extern static const uint8_t GIR_CONFIG_AD = 0x1B ;static const uint8_t USER_CTRL = 0x6A ;
extern static const uint8_t INT_BYPASS_CONFIG = 0x37;
extern static const uint8_t PWR_MGMT_1 = 0x6B ;
extern static const uint8_t CNTL1 = 0x0A ;
extern static const uint8_t ACE_XOUT_H = 0x3B ;
extern static const uint8_t MAG_XOUT_L = 0x03 ;
extern static const uint8_t TEMP_CONTROL_REG_AD = 0x2E ; //Endereço do "control register" da temperatura; pag.21 do datasheet
extern static const uint8_t PRE_CONTROL_REG_AD = 0xB4 ; //Endereço do "control register" da pressão; pag.21 do datasheet
extern static const uint8_t BMP_REG_AD = 0xF4 ; //Endereço do "register"; pag.21 do datasheet
extern static const uint8_t AC1_MSB = 0xAA ; //Endereço do msb do primeiro valor do e2prom; pag.13 do datasheet
extern static const uint8_t XG_OFFSET_H = 0x13 ; //Endereço dos registradores de offset do giroscópio; register map - p. 7
extern static const uint8_t XG_OFFSET_L = 0x14 ;
extern static const uint8_t YG_OFFSET_H = 0x15 ;
extern static const uint8_t YG_OFFSET_L = 0x16 ;
extern static const uint8_t ZG_OFFSET_H = 0x17 ;
extern static const uint8_t ZG_OFFSET_L = 0x18 ;
extern static const uint8_t XA_OFFSET_H = 0x77 ; //Endereço dos registradores de offset do acelerômetro; register map - p. 9
extern static const uint8_t XA_OFFSET_L = 0x78 ;
extern static const uint8_t YA_OFFSET_H = 0x7A ;
extern static const uint8_t YA_OFFSET_L = 0x7B ;
extern static const uint8_t ZA_OFFSET_H = 0x7C ;
extern static const uint8_t ZA_OFFSET_L = 0x7D ;
extern static const uint8_t MAG_ASAX = 0x10 ; //Endereço dos registradores de ajuste de sensibilidade; register map - p. 47
extern static const uint8_t MAG_ASAY = 0x11 ;
extern static const uint8_t MAG_ASAZ = 0x12 ;

extern static const uint8_t GPIO_PIN_ultrassom = 00 ;
/////extern string const GPIO_X = GPIO_A ;
/////extern static const canal_ultrassom = TIM_CHANNEL_2;
//OS PARÂMETROS DA FUNÇÃO DE SENSOR ULTRASSÔNICO DEVEM SER AJUSTADOS SEGUNDO CONFIGURAÇÃO NO CUBE_MX


//Funções

//Configuraçao: Acel, Giro, Mag, Bmp
extern void confgAce (uint8_t * buf, uint8_t A1)
{
	buf[0] = A1; //ACE_CONFIG_AD
	buf[1] = 0x10; // 0x10 configura o range para +-8g
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_AD, buf, 2, 100);
   }

extern void confgGiro (uint8_t * buf, uint8_t G1, uint8_t G2)
{
  	buf[0] = G1; //GIR_CONFIG_AD
	buf[1] = 0x11; // Configura o range para +-1000 dps e Fchoice 01
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_AD, buf, 2, 100);
	buf[0] = G2; //PWR_MGMT_1
	buf[2] = 0x01 ; // Define a referência do clock como o eixo x
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_AD, buf, 2, 100);
  }
  
extern void confMag (uint8_t * buf, uint8_t M1, uint8_t M2, uint8_t M3)
{
  	buf[0] = M1 ; //INT_BYPASS_CONFIG
	buf[1] = 0x02 ; // Liga o bypass multiplex
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_AD, buf, 2, 100);
	buf[0] = M2; //CNTL1
	buf[1] = 0x1F;
	HAL_I2C_Master_Transmit(&hi2c1, MAG_AD, buf, 2, 100);
	buf[0] = M3; //MAG_ASAX
	HAL_I2C_Master_Transmit(&hi2c1, MAG_AD, buf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, MAG_AD, buf, 3, 100); // Lendo a sensibilidade do magnetômetro
/*??*/	magSensi[0] = (float) buf[0];
/*??*/	magSensi[1] = (float) buf[1];
/*??*/	magSensi[2] = (float) buf[2];
  }
  
extern void confgMPU (uint8_t * buf, uint8_t A1, uint8_t G1, uint8_t G2, uint8_t M1, uint8_t M2, uint8_t M3)
{
	//Confg Acel
	buf[0] = A1; //ACE_CONFIG_AD
	buf[1] = 0x10; // 0x10 configura o range para +-8g
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_AD, buf, 2, 100);
	
	//Confg Giro
	buf[0] = G1; //GIR_CONFIG_AD
	buf[1] = 0x11; // Configura o range para +-1000 dps e Fchoice 01
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_AD, buf, 2, 100);
	buf[0] = G2; //PWR_MGMT_1
	buf[2] = 0x01 ; // Define a referência do clock como o eixo x
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_AD, buf, 2, 100);
	
	//Confg Mag
	buf[0] = M1 ; //INT_BYPASS_CONFIG
	buf[1] = 0x02 ; // Liga o bypass multiplex
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_AD, buf, 2, 100);
	buf[0] = M2; //CNTL1
	buf[1] = 0x1F;
	HAL_I2C_Master_Transmit(&hi2c1, MAG_AD, buf, 2, 100);
	buf[0] = M3; //MAG_ASAX
	HAL_I2C_Master_Transmit(&hi2c1, MAG_AD, buf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, MAG_AD, buf, 3, 100); // Lendo a sensibilidade do magnetômetro
}

  void confgBmp (uint8_t * buf, uint8_t B1, uint8_t B2, uint8_t B3, uint8_t B4)
  {
    	int j, k;
	buf[0] = B1; //BMP_REG_AD
	buf[1] = B2; //PRE_CONTROL_REG_AD
	buf[2] = B3; //TEMP_CONTROL_REG_AD
	HAL_I2C_Master_Transmit(&hi2c1, BMP_AD, buf, 3, 100);//Iniciando o sensor
	buf[0] = B4; //AC1_MSB
	HAL_I2C_Master_Receive(&hi2c1, BMP_AD, buf, 22, 100);//Lendo os valores do e2prom e armazenando-os em um vetor chamado e2prom[]
	for (int i=0;i<11;i++)
	{
		//Lógica usada para os valores de j e k -> e[0] = 0 + 1 -> e[1] = 2 + 3 -> e[2] = 4 + 5
		j = i*2;
		k = (i*2)+1;
		e2prom[i] = (int16_t)(((int16_t)buf[j] << 8)  | buf[k]) ;//Turn the MSB and LSB into a signed 16-bit value
	}
    }

//Offset : Accel a Giro
extern void offsetAccel () //atribui a ax_off, ay_off, az_off os valores de offset dos registradores
{
	buf[0] = XA_OFFSET_H;
	ret = HAL_I2C_Master_Transmit(&hi2c1,MPU9250_AD,buf,1,100);
	if (ret != HAL_OK)
	{
		strcpy((char*)msg,"Error Tx\r\n");
	}
	else{
		ret = HAL_I2C_Master_Receive(&hi2c1,MPU9250_AD,buf,6,100);
		if (ret != HAL_OK)
		{
			strcpy((char*)msg,"Error Rx\r\n");
		}
		else
		{
			data[0] = (int16_t)(((int16_t)buf[0] << 8)  | buf[1]);  //concatena dois bytes
			data[1] = (int16_t)(((int16_t)buf[2] << 8)  | buf[3]);
			data[2] = (int16_t)(((int16_t)buf[4] << 8)  | buf[5]);
			
			ax_off = data[0] * aRes;
			ay_off = data[0] * aRes;
			az_off = data[0] * aRes;
			
			sprintf((char*)msg,"ax_off=%f  ay_off=%f  az_off=%f  [g]", ax_off,ay_off,az_off); //imprime no terminal os valores
		}
	}
 }

extern void offsetGyro () //atribui a gx_off, gy_off, gz_off os valores de offset dos registradores
{
	buf[0] = XG_OFFSET_H;
	ret = HAL_I2C_Master_Transmit(&hi2c1,MPU9250_AD,buf,1,100);
	if (ret != HAL_OK)
	{
		strcpy((char*)msg,"Error Tx\r\n");
	}
	else
	{
		ret = HAL_I2C_Master_Receive(&hi2c1,MPU9250_AD,buf,6,100);
		if (ret != HAL_OK)
		{
			strcpy((char*)msg,"Error Rx\r\n");
		}
		else
		{
			data[0] = (int16_t)(((int16_t)buf[0] << 8)  | buf[1]);  //concatena dois bytes
			data[1] = (int16_t)(((int16_t)buf[2] << 8)  | buf[3]);
			data[2] = (int16_t)(((int16_t)buf[4] << 8)  | buf[5]);
			
			gx_off = data[0] * gRes;
			gy_off = data[0] * gRes;
			gz_off = data[0] * gRes;
			
			sprintf((char*)msg,"gx_off=%f  gy_off=%f  gz_off=%f  [degrees/s]", gx_off,gy_off,gz_off); //imprime valores no terminal
		}
	}
 }

extern void adjMagnet () //essa função só atribui a mx_adj, my_adj, mz_adj o fator que multiplicado pela leitura do magnetômetro resulta no adjustment
{
	buf[0] = MAG_ASAX;
	ret = HAL_I2C_Master_Transmit(&hi2c1,MPU9250_AD,buf,1,100);
	if (ret != HAL_OK)
	{
		strcpy((char*)msg,"Error Tx\r\n");
	}
	else
	{
		ret = HAL_I2C_Master_Receive(&hi2c1,MPU9250_AD,buf,6,100);
		if (ret != HAL_OK)
		{
			strcpy((char*)msg,"Error Rx\r\n");
		}
		else
		{
			data[0] = (float) buf[0]; 
			data[1] = (float) buf[2];
			data[2] = (float) buf[4];

			mx_adj = (float) (data[0]-128)/256 + 1.;
			my_adj = (float) (data[1]-128)/256 + 1.;
			mz_adj = (float) (data[2]-128)/256 + 1.;

			sprintf((char*)msg,"mx_adj=%f*Hx   my_adj=%f*Hy  mz_adj=%f*Hz\r\n",mx_adj,my_adj,mz_adj); //impressão dos valores no terminal
		}
	}
 }

//Calculo da temperatura e pressao

extern float getTemp(uint8_t * buf, uint16_t * e2prom) //vetor e2prom calculado na confgBmp
{
	float Temp = 0;
	long int UT, X1, X2;
	B5 = 0; //Variavel global
	
	UT = (int16_t)(((int16_t)buf[0] << 8)  | buf[1]) ; //Turn the MSB and LSB into a signed 16-bit value
	X1 = (UT-e2prom[5])*e2prom[4]/(2^15);//32768
	X2 = e2prom[9]*2^11/(X1+e2prom[10]);
	B5 = X1+X2; 
	Temp = ((B5+8)/(2^4))/10;
	
	return Temp;
}

extern long int getPressure(uint8_t * buf, uint16_t * e2prom)
{
	long int UP, X1, X2, X3, B3, B4, B6, B7, P;
	
	UP = (int16_t)(((int16_t)buf[0] << 16)  | buf[1] << 8 | buf[2]) >> (8-2); //Junta o MSB, o LSB e o XLSB em um único número de 24 bits
	B6 = B5-4000;  //Variavel global - B5
	X1 = (e2prom[7]*(B6*B6/2^12))/(2^11);
	X2 = e2prom[1]*B6/(2^11);
	X3 = X1 + X2;
	B3 = (((e2prom[0]*4+X3)<<2)+2)/4;
	X1 = e2prom[2]*B6/(2^13);
	X2 = (e2prom[6]*(B6*B6/2^12))/(2^16);
	X3 = ((X1+X2)+2)/4;
	B4 = e2prom[3]*(unsigned long)(X3+32768)/(2^15);
	B7 = ((unsigned long)UP-B3)*(50000>>2);
	if(B7<0x80000000){
		P = (B7*2)/B4;
	}else{
		P = (B7/B4)*2;
	}
	X1 = (P/2^8)*(P/2^8);
	X1 = (X1*3038)/(2^16);
	X2 = (-7357*P)/(2^16);
	P = P+(X1+X2+3791)/(2^4);
	
	return P;
}

//Leitura do acelerômetro, termômetro e giroscópio
extern void getAccelTempGyro()
{
	buf[0] = ACE_XOUT_H;
	ret = HAL_I2C_Master_Transmit(&hi2c1,MPU9250_AD,buf,1,100);
	if (ret != HAL_OK)
	{
		strcpy((char*)msg,"Error Tx\r\n");
	}
	else
	{
		ret = HAL_I2C_Master_Receive(&hi2c1,MPU9250_AD,buf,14,100);
		if (ret != HAL_OK)
		{
			strcpy((char*)msg,"Error Rx\r\n");
		}
		else
		{
			data[0] = (int16_t)(((int16_t)buf[0] << 8)  | buf[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
			data[1] = (int16_t)(((int16_t)buf[2] << 8)  | buf[3]) ;
			data[2] = (int16_t)(((int16_t)buf[4] << 8)  | buf[5]) ;
			data[3] = (int16_t)(((int16_t)buf[6] << 8)  | buf[7]) ;
			data[4] = (int16_t)(((int16_t)buf[8] << 8)  | buf[9]) ;
			data[5] = (int16_t)(((int16_t)buf[10] << 8) | buf[11]);
			data[6] = (int16_t)(((int16_t)buf[12] << 8) | buf[13]);

			ax = (float) data[0] * aRes;
			ay = (float) data[1] * aRes;
			az = (float) data[2] * aRes;

			t = ((float) data[3] - 21.0)/333.87 + 21.0;

			gx = (float) data[4] * gRes;
			gy = (float) data[5] * gRes;
			gz = (float) data[6] * gRes;

			sprintf((char*)msg," ax=%f  ay=%f  az=%f  [m/s^2]\n gx=%f  gy=%f  gz=%f  [degrees/s]\n t=%f[ºC]\n",ax,ay,az,gx,gy,gz,t);
		}
	}
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
}

//Leitura Magnetômetro
extern void
{
	buf[0] = MAG_XOUT_L;
	ret = HAL_I2C_Master_Transmit(&hi2c1,MAG_AD,buf,1,100);
	if (ret != HAL_OK){
			strcpy((char*)msg,"Error Mag Tx\r\n");
		}
		else{
			ret = HAL_I2C_Master_Receive(&hi2c1,MAG_AD,buf,6,100);
			if (ret != HAL_OK){
				strcpy((char*)msg,"Error Mag Rx\r\n");
			}
			else{
				data[0] = (int16_t)(((int16_t)buf[1] << 8)  | buf[0]);  // Turn the MSB and LSB into a signed 16-bit value
				data[1] = (int16_t)(((int16_t)buf[3] << 8)  | buf[2]);
				data[2] = (int16_t)(((int16_t)buf[5] << 8)  | buf[4]);

				mx = (float) data[0] *((magSensi[0] - 128)*0.5/128 + 1);
				my = (float) data[1] *((magSensi[1] - 128)*0.5/128 + 1);
				mz = (float) data[2] *((magSensi[2] - 128)*0.5/128 + 1);

				sprintf((char*)msg,"mx=%f   my=%f   mz=%f\r\n",mx,my,mz);
			}
		}
	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	HAL_Delay(100);
}

//Delay microssegundos
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

//Sensor ultrassônico
extern void
{
	HAL_GPIO_WritePin	(GPIOX, GPIO_PIN_ultrassom, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin	(GPIOX, GPIO_PIN_ultrassom, GPIO_PIN_RESET);
	US_1 = HAL_TIM_ReadCapturedValue(htim, canal_ultrassom);
	__HAL_TIM_SET_CAPTUREPOLARITY(htim, canal_ultrassom, TIM_INPUTCHANNELPOLARITY_FALLING);
	US_2 = HAL_TIM_ReadCapturedValue(htim, canal_ultrassom);
	__HAL_TIM_SET_COUNTER(htim, 0);
	__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
	distance = (US_2 - US_1)/58;
	HAL_Delay(80);
}

#endif

