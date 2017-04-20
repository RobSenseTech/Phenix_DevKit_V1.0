#include "iic.h"
#include "driver.h"

int iic_test(void)
{
	int i;
	u8 data;
	u8 datas[6];
	iic_priv_s *mpu6050_priv;

	int16_t Acc_I16_x ;
	int16_t	Acc_I16_y ;
	int16_t	Acc_I16_z ;
	
	mpu6050_priv = iic_get_priv(IIC_ID_0, 0x68, 400000);
	
	
	data = 0x6B;
	Iic_set_address(mpu6050_priv, 0x75);
		

	mpu6050_priv->frequency = 100000;
	
	iic_transfer(mpu6050_priv, NULL, 0, &data, 1, 1);
	pilot_info("%x\r\n",data);	


	datas[1] = 0x80;
	data = 0x11;

	Iic_set_address(mpu6050_priv, 0x6B);
	iic_transfer(mpu6050_priv, &datas[1], 1, &data, 1, 1);	
	pilot_info("%x\r\n",data);

	datas[0] = 0x6B;
	datas[1] = 0x08;
	data = 0x11;
	Iic_set_address(mpu6050_priv, 0x6B);
	iic_transfer(mpu6050_priv, &datas[1], 1, &data, 1, 1);	

	pilot_info("%x\r\n",data);
	

	for(i=0; i<10; i++)
	{
		data = 0x3B;
		Iic_set_address(mpu6050_priv, 0x3B);
		iic_transfer(mpu6050_priv, NULL, 0, datas, 6, 1);	

		Acc_I16_x = ((((int16_t)datas[0]) << 8) | datas[1]);
		Acc_I16_y = ((((int16_t)datas[2]) << 8) | datas[3]);
		Acc_I16_z = ((((int16_t)datas[4]) << 8) | datas[5]);		
		pilot_info("x=:%d  y=:%d  z=:%d\r\n",Acc_I16_x,Acc_I16_y,Acc_I16_z);
		sleep(2);
	}
	
	for(i=0; i<10; i++)
	{
		data = 0x3B;
		Iic_set_address(mpu6050_priv, 0x3B);
		iic_transfer(mpu6050_priv, NULL, 0, datas, 6, 1);	

		Acc_I16_x = ((((int16_t)datas[0]) << 8) | datas[1]);
		Acc_I16_y = ((((int16_t)datas[2]) << 8) | datas[3]);
		Acc_I16_z = ((((int16_t)datas[4]) << 8) | datas[5]);		
		pilot_info("x=:%d  y=:%d  z=:%d\r\n",Acc_I16_x,Acc_I16_y,Acc_I16_z);
		sleep(2);
	}
	
	
	
	data = 0x11;
	Iic_set_address(mpu6050_priv, 0x6B);
	iic_transfer(mpu6050_priv, NULL, 0, &data, 1, 1);
	pilot_info("%x\r\n",data);	
	

	debug_print(mpu6050_priv);

	return XST_SUCCESS;
}

int LED_iic_test(void)
{
	int i;
//	u8 data;
	u8 datas[8] ={0x81,0x06,0x82,0x07,0x83,0x08,0x84,0x03};
	u8 recv[2];
	iic_priv_s *led_priv;

	led_priv = iic_get_priv(IIC_ID_1, 0x55, 100000);

	recv[0] = 0x11;
	iic_transfer(led_priv, NULL, 0, recv, 2, 0);
	for(i=0; i<2; i++)
	{
		pilot_info("%x\r\n",recv[i]);			
	}

	Iic_set_address(led_priv, 0x01);
	iic_transfer(led_priv, datas, 8, NULL, 0, 0);

	iic_transfer(led_priv, NULL, 0, recv, 2, 0);	
	for(i=0; i<2; i++)
	{
		pilot_info("%x\r\n",recv[i]);			
	}	
	
	
	datas[1] = 0x00;
	datas[3] = 0x01;
	datas[5] = 0x02;

	datas[0] = 0x81;
	datas[2] = 0x82;
	datas[4] = 0x83;	
	
	Iic_set_address(led_priv, 0x01);
	iic_transfer(led_priv, datas, 6, NULL, 0, 0);
	
	
	iic_transfer(led_priv, NULL, 0, recv, 2, 0);
	for(i=0; i<2; i++)
	{
		pilot_info("%x\r\n",recv[i]);			
	}	
	debug_print(led_priv);
	return XST_SUCCESS;
}


int hmc_iic_test(void)
{
//	int i;
	u8 data;
//	u8 datas[6];
	iic_priv_s *hmc5883_priv;

	
	hmc5883_priv = iic_get_priv(IIC_ID_0, 0x1e, 400000);
	
	
	data = 0x6B;
	Iic_set_address(hmc5883_priv, 0x0a);
	iic_transfer(hmc5883_priv, NULL, 0, &data, 1, 1);
	pilot_info("WHO AM I 0 = %c\r\n",data);	

	data = 0x6B;
	Iic_set_address(hmc5883_priv, 0x0b);
	iic_transfer(hmc5883_priv, NULL, 0, &data, 1, 1);
	pilot_info("WHO AM I 1 = %c\r\n",data);	
	
	data = 0x6B;
	Iic_set_address(hmc5883_priv, 0x0c);
	iic_transfer(hmc5883_priv, NULL, 0, &data, 1, 1);
	pilot_info("WHO AM I 2 = %c\r\n",data);	

	
	data = 0x0a;
//	Iic_set_address(&hmc5883_priv, 0x0a);
	
	iic_transfer(hmc5883_priv, &data, 1, &data, 1, 0);
	pilot_info("WHO AM I 0 = %c\r\n",data);	

	data = 0x0b;
//	Iic_set_address(&hmc5883_priv, 0x0b);
	iic_transfer(hmc5883_priv, &data, 1, &data, 1, 0);
	pilot_info("WHO AM I 1 = %c\r\n",data);	
	
	data = 0x0c;
//	Iic_set_address(&hmc5883_priv, 0x0c);
	iic_transfer(hmc5883_priv, &data, 1, &data, 1, 0);
	pilot_info("WHO AM I 2 = %c\r\n",data);		
	
	
	
/*	int16_t Acc_I16_x ;
	int16_t	Acc_I16_y ;
	int16_t	Acc_I16_z ;
*/	
	//datas[1] = 0x80;
	//data = 0x11;

	//Iic_set_address(&mpu6050_priv, 0x6B);
	//iic_transfer(&mpu6050_priv, &datas[1], 1, &data, 1, 1);	
	//pilot_info("%x\r\n",data);

	//datas[0] = 0x6B;
	//datas[1] = 0x08;
	//data = 0x11;
	//Iic_set_address(&mpu6050_priv, 0x6B);
	//iic_transfer(&mpu6050_priv, &datas[1], 1, &data, 1, 1);	

	//pilot_info("%x\r\n",data);
	

	//for(i=0; i<10; i++)
	//{
	//	data = 0x3B;
	//	Iic_set_address(&mpu6050_priv, 0x3B);
	//	iic_transfer(&mpu6050_priv, NULL, 0, &datas, 6, 1);	

	//	Acc_I16_x = ((((int16_t)datas[0]) << 8) | datas[1]);
	//	Acc_I16_y = ((((int16_t)datas[2]) << 8) | datas[3]);
	//	Acc_I16_z = ((((int16_t)datas[4]) << 8) | datas[5]);		
	//	pilot_info("x=:%d  y=:%d  z=:%d\r\n",Acc_I16_x,Acc_I16_y,Acc_I16_z);
	//	sleep(2);
	//}
	
	//data = 0x11;
	//Iic_set_address(&mpu6050_priv, 0x6B);
	//iic_transfer(&mpu6050_priv, NULL, 0, &data, 1, 1);
	//pilot_info("%x\r\n",data);	
	

	debug_print(hmc5883_priv);

	return XST_SUCCESS;
}



