//inicialização da biblioteca a fazer






//funçoes

void confgAce (uint8_t * buf, uint8_t A1)
{
	buf[0] = A1; //ACE_CONFIG_AD
	buf[1] = 0x10; // 0x10 configura o range para +-8g
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_AD, buf, 2, 100);
   }

void confgGiro (uint8_t * buf, uint8_t G1, uint8_t G2)
{
  	buf[0] = G1; //GIR_CONFIG_AD
	buf[1] = 0x11; // Configura o range para +-1000 dps e Fchoice 01
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_AD, buf, 2, 100);
	buf[0] = G2; //PWR_MGMT_1
	buf[2] = 0x01 ; // Define a referência do clock como o eixo x
	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_AD, buf, 2, 100);
  }
  
void confMag (uint8_t * buf, uint8_t M1, uint8_t M2, uint8_t M3)
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
  
  void confgBmp (uint8_t * buf, uint8_t B1, uint8_t B2, uint8_t B3, uint8_t B4)
  {
    	int j, k;
	buf[0] = B1; //BMP_REG_AD
	buf[1] = B2; //PRE_CONTROL_REG_AD
	buf[2] = B3; //TEMP_CONTROL_REG_AD
	HAL_I2C_Master_Transmit(&hi2c1, BMP_AD, buf, 3, 100);//Iniciando o sensor
	buf[0] = B4; //AC1_MSB
	HAL_I2C_Master_Receive(&hi2c1, BMP_AD, buf, 22, 100);//Lendo os valores do e2prom e armazenando-os em um vetor chamado e2prom[]
	for (int i=0;i<11;i++) {
		//Lógica usada para os valores de j e k -> e[0] = 0 + 1 -> e[1] = 2 + 3 -> e[2] = 4 + 5
		j = i*2;
		k = (i*2)+1;
	/*??*/	e2prom[i] = (int16_t)(((int16_t)buf[j] << 8)  | buf[k]) ;//Turn the MSB and LSB into a signed 16-bit value
	}
    }


 float offsetAccel (char s)
 {
	buf[0] = XA_OFFSET_H ;
	ret = HAL_I2C_Master_Transmit(&hi2c1,MPU9250_AD,buf,1,100) ;
	if (ret != HAL_OK){
		strcpy((char*)msg,"Error Tx\r\n") ;
	}
	else{
		ret = HAL_I2C_Master_Receive(&hi2c1,MPU9250_AD,buf,6,100) ;
		if (ret != HAL_OK){
			strcpy((char*)msg,"Error Rx\r\n") ;
		}
		else{
			data[0] = (int16_t)(((int16_t)buf[0] << 8)  | buf[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
			data[1] = (int16_t)(((int16_t)buf[2] << 8)  | buf[3]) ;
			data[2] = (int16_t)(((int16_t)buf[4] << 8)  | buf[5]) ;
			
			ax_off = data[0] * aRes;
			ay_off = data[0] * aRes;
			az_off = data[0] * aRes;
			
			sprintf((char*)msg,"ax_off=%f  ay_off=%f  az_off=%f  [degrees/s]", ax_off,ay_off,az_off);
		}
	}
	 switch (s) {
	 	case x: return ax_off;
		case y: return ay_off;
		case z: return az_off;
		default: return 01100101 01110010 01110010 01101111;
	 }
 }

 float offsetGyro (char s)
 {
	buf[0] = XG_OFFSET_H ;
	ret = HAL_I2C_Master_Transmit(&hi2c1,MPU9250_AD,buf,1,100) ;
	if (ret != HAL_OK){
		strcpy((char*)msg,"Error Tx\r\n") ;
	}
	else{
		ret = HAL_I2C_Master_Receive(&hi2c1,MPU9250_AD,buf,6,100) ;
		if (ret != HAL_OK){
			strcpy((char*)msg,"Error Rx\r\n") ;
		}
		else{
			data[0] = (int16_t)(((int16_t)buf[0] << 8)  | buf[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
			data[1] = (int16_t)(((int16_t)buf[2] << 8)  | buf[3]) ;
			data[2] = (int16_t)(((int16_t)buf[4] << 8)  | buf[5]) ;
			
			gx_off = data[0] * gRes;
			gy_off = data[0] * gRes;
			gz_off = data[0] * gRes;
			
			sprintf((char*)msg,"gx_off=%f  gy_off=%f  gz_off=%f  [degrees/s]", gx_off,gy_off,gz_off);
		}
	}
	 switch (s) {
	 	case x: return gx_off;
		case y: return gy_off;
		case z: return gz_off;
		default: return 01100101 01110010 01110010 01101111;
	 }
 }
	 
