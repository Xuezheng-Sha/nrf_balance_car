/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "app_util_platform.h"
#include "twi_master.h"
#include "mpu6050.h"

/*lint ++flb "Enter library region" */

#define ADDRESS_WHO_AM_I          (0x75U) // !< WHO_AM_I register identifies the device. Expected value is 0x68.
#define ADDRESS_SIGNAL_PATH_RESET (0x68U) // !<

static const uint8_t expected_who_am_i = 0x68U; // !< Expected value to get from WHO_AM_I register.
static uint8_t       m_device_address = 0xD0; 


/*lint --flb "Leave library region" */

         // !< Device address in bits [7:1]

bool mpu6050_init(uint8_t device_address)
{
    bool transfer_succeeded = true;

    m_device_address = (uint8_t)(device_address << 1);

    // Do a reset on signal paths
    uint8_t reset_value = 0x04U | 0x02U | 0x01U; // Resets gyro, accelerometer and temperature sensor signal paths.
    transfer_succeeded &= mpu6050_register_write(ADDRESS_SIGNAL_PATH_RESET, reset_value);

    // Read and verify product ID
    transfer_succeeded &= mpu6050_verify_product_id();

    return transfer_succeeded;
}

bool mpu6050_verify_product_id(void)
{
    uint8_t who_am_i;

    if (mpu6050_register_read(ADDRESS_WHO_AM_I, &who_am_i, 1)) {
        if (who_am_i != expected_who_am_i) {
            return false;
        } else {
            return true;
        }
    } else {
        return false;
    }
}

bool mpu6050_write(uint8_t device_address, uint8_t register_address, uint8_t len, uint8_t *data)
{
    uint8_t w2_data[len+1];

    for(int i=0;i<len+1;i++)
    {
        if(i==0)
                w2_data[0] =register_address;
        else
            w2_data[i] = *(data+i-1);
    }

    return twi_master_transfer(m_device_address, w2_data, len+1, TWI_ISSUE_STOP);
}

bool mpu6050_read(uint8_t device_address, uint8_t register_address, uint8_t number_of_bytes, uint8_t *destination)
{
    bool transfer_succeeded;
    transfer_succeeded = twi_master_transfer(m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
    transfer_succeeded &= twi_master_transfer((device_address<<1)|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP);
    return transfer_succeeded;
}

bool mpu6050_register_write(uint8_t register_address, uint8_t value)
{
    uint8_t w2_data[2];

    w2_data[0] = register_address;
    w2_data[1] = value;
    return twi_master_transfer(m_device_address, w2_data, 2, TWI_ISSUE_STOP);
}

bool mpu6050_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
    bool transfer_succeeded;
    transfer_succeeded  = twi_master_transfer(m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
    transfer_succeeded &= twi_master_transfer(m_device_address|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP);
    return transfer_succeeded;
}

/*
  Read the Gyro values from the MPU6050's internal Registers
*/ 
bool MPU6050_ReadGyro(int16_t *pGYRO_X , int16_t *pGYRO_Y , int16_t *pGYRO_Z )
{
  uint8_t buf[6]; 
  
  bool ret = false;	

  if(mpu6050_register_read(MPU6050_GYRO_OUT,  buf, 6) == true) {
        *pGYRO_X = (buf[0] << 8) | buf[1];
        if(*pGYRO_X & 0x8000) *pGYRO_X-=65536;

        *pGYRO_Y= (buf[2] << 8) | buf[3];
        if(*pGYRO_Y & 0x8000) *pGYRO_Y-=65536;

        *pGYRO_Z = (buf[4] << 8) | buf[5];
        if(*pGYRO_Z & 0x8000) *pGYRO_Z-=65536;

        ret = true;
    }

  return ret;
}

/*
  A Function to read accelerometer's values from the internal registers of MPU6050
*/ 
bool MPU6050_ReadAcc( int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z )
{
  uint8_t buf[6];
  bool ret = false;
  
  if(mpu6050_register_read(MPU6050_ACC_OUT, buf, 6) == true)
  {
    mpu6050_register_read(MPU6050_ACC_OUT, buf, 6);
    
    *pACC_X = (buf[0] << 8) | buf[1];
    if(*pACC_X & 0x8000) *pACC_X-=65536;

    *pACC_Y= (buf[2] << 8) | buf[3];
    if(*pACC_Y & 0x8000) *pACC_Y-=65536;

    *pACC_Z = (buf[4] << 8) | buf[5];
    if(*pACC_Z & 0x8000) *pACC_Z-=65536;

    ret = true;
    }
  
  return ret;
}

void mpu_6050_read_temp(double * p_temp)
{
    ret_code_t err_code;

     // Signed 16-bit integer to store the measurements 
    int16_t temp;

    // Raw temperature measurements buffer
    uint8_t temp_data[2];

    //Read the two temperature data registers starting from TEMP_OUT_H
    err_code = mpu6050_register_read(MPU6050_GYRO_OUT, temp_data, sizeof(temp_data));
    APP_ERROR_CHECK(err_code);

    /*  Combine the two 8-bit data registers to a 16-bit value
        for each axis by left shifting TEMP_OUT_H eight times 
        and OR it with TEMP_OUT_L.  */
    temp = (temp_data[0]<<8)|temp_data[1];
    
    /*  Convert raw measurement to degrees C using formula from TEMP_OUT_H register description 
        in MPU-6050 Register Map and Descriptions document  */
    *p_temp = (temp/340.0)+ 36.53;
}

