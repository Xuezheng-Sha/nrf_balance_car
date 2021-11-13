#include <stdbool.h>   
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "sdk_errors.h"
#include "nrf_drv_twi.h"
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "sdk_errors.h"
#include "nrf_drv_twi.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
//Log需要引用的头文件
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "task_service.h"
#include "twi_master.h"

#define TASK_DELAY (2000)
ret_code_t err_code;

TaskHandle_t led_toggle_task_handle;
TaskHandle_t mpu6050_analysis_task_handle;

//static void led_toggle_task_function(void *pvParameter) {

//    while(1) {
//        nrf_gpio_pin_toggle(LED_1);
//        vTaskDelay(TASK_DELAY);
//    }
//}

//日志打印模块初始化
static void log_init(void)
{
    //初始化log程序模块
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    //设置log输出终端（根据sdk_config.h中的配置设置输出终端为UART或者RTT）
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//串口事件回调函数，该函数中判断事件类型并进行处理
void uart_error_handle(app_uart_evt_t * p_event)
{
    uint8_t cr;

    //通讯错误事件
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) {
        
        APP_ERROR_HANDLER(p_event->data.error_communication);
    } else if (p_event->evt_type == APP_UART_FIFO_ERROR) {
            //FIFO错误事件
            APP_ERROR_HANDLER(p_event->data.error_code);
    } else if (p_event->evt_type == APP_UART_DATA_READY) {
        //串口接收事件
        //翻转指示灯D1状态，指示串口接收事件
        nrf_gpio_pin_toggle(LED_1);
        //从FIFO中读取数据
        app_uart_get(&cr);
        //串口输出数据
        printf("%c",cr);
    } else if (p_event->evt_type == APP_UART_TX_EMPTY) {
        //串口发送完成事件
        //翻转指示灯D2状态，指示串口发送完成事件
        nrf_gpio_pin_toggle(LED_2);
    }
}

//串口配置
void uart_config(void)
{
    uint32_t err_code;

    //定义串口通讯参数配置结构体并初始化
  const app_uart_comm_params_t comm_params =
  {
    RX_PIN_NUMBER,//定义uart接收引脚
    TX_PIN_NUMBER,//定义uart发送引脚
    RTS_PIN_NUMBER,//定义uart RTS引脚，流控关闭后虽然定义了RTS和CTS引脚，但是驱动程序会忽略，不会配置这两个引脚，两个引脚仍可作为IO使用
    CTS_PIN_NUMBER,//定义uart CTS引脚
    APP_UART_FLOW_CONTROL_DISABLED,//关闭uart硬件流控
    false,//禁止奇偶检验
    NRF_UART_BAUDRATE_115200//uart波特率设置为115200bps
  };
  //初始化串口，注册串口事件回调函数
  APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

  APP_ERROR_CHECK(err_code);

}


/***********************************************************************************
 * 描  述 : 串口发送数据，数据格式为匿名四轴上位机软件(V2.6版本)数据格式
 * 入  参 : fun:功能码
 *        : dat:数据缓存区地址,最多28字节
 *        : len:数据长度，最大28字节
 * 返回值 : 无
 **********************************************************************************/ 
void Uart_SendDat_ToPC(uint8_t fun,uint8_t *dat,uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;
    if(len>28)return;   //最多28字节数据 
    send_buf[len+3]=0;  //校验数置零
    send_buf[0]=0x88;   //帧头
    send_buf[1]=fun;    //功能码
    send_buf[2]=len;    //数据长度
    for(i=0;i<len;i++)send_buf[3+i]=dat[i]; //复制数据
    for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];   //计算校验和
    for(i=0;i<len+4;i++)app_uart_put(send_buf[i]);  //串口输出数据
}


/***********************************************************************************
 * 描  述 : 发送加速度传感器数据和陀螺仪数据
 * 入  参 : aacx,aacy,aacz:x,y,z三个方向上面的加速度值
 *          gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
 * 返回值 : 无
 **********************************************************************************/
void mpu6050_send_dat(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
    uint8_t tx_buf[12]; 
    tx_buf[0]=(aacx>>8)&0xFF;
    tx_buf[1]=aacx&0xFF;
    tx_buf[2]=(aacy>>8)&0xFF;
    tx_buf[3]=aacy&0xFF;
    tx_buf[4]=(aacz>>8)&0xFF;
    tx_buf[5]=aacz&0xFF; 
    tx_buf[6]=(gyrox>>8)&0xFF;
    tx_buf[7]=gyrox&0xFF;
    tx_buf[8]=(gyroy>>8)&0xFF;
    tx_buf[9]=gyroy&0xFF;
    tx_buf[10]=(gyroz>>8)&0xFF;
    tx_buf[11]=gyroz&0xFF;
    Uart_SendDat_ToPC(0xA1,tx_buf,12);//自定义帧,0XA1
}


/***********************************************************************************
 * 描  述 : 串口上传MPU6050姿态数据
 * 入  参 : aacx,aacy,aacz:x,y,z三个方向上面的加速度值
 *        : gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
 *        : roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
 *        : pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
 *        : yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
 * 返回值 : 无
 **********************************************************************************/ 
void Uart_ReportIMU(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
    uint8_t i,tx_buf[28]; 

    for(i=0;i<28;i++)tx_buf[i]=0;//清0
    tx_buf[0]=(aacx>>8)&0xFF;
    tx_buf[1]=aacx&0xFF;
    tx_buf[2]=(aacy>>8)&0xFF;
    tx_buf[3]=aacy&0xFF;
    tx_buf[4]=(aacz>>8)&0xFF;
    tx_buf[5]=aacz&0xFF; 
    tx_buf[6]=(gyrox>>8)&0xFF;
    tx_buf[7]=gyrox&0xFF;
    tx_buf[8]=(gyroy>>8)&0xFF;
    tx_buf[9]=gyroy&0xFF;
    tx_buf[10]=(gyroz>>8)&0xFF;
    tx_buf[11]=gyroz&0xFF;	
    tx_buf[18]=(roll>>8)&0xFF;
    tx_buf[19]=roll&0xFF;
    tx_buf[20]=(pitch>>8)&0xFF;
    tx_buf[21]=pitch&0xFF;
    tx_buf[22]=(yaw>>8)&0xFF;
    tx_buf[23]=yaw&0xFF;
    Uart_SendDat_ToPC(0xAF,tx_buf,28);//匿名四轴飞控显示帧,0xAF
}

static void mpu6050_analysis_task_function(void *pvParameter)
{
    double Temperature_centigrade;
    float pitch,roll,yaw;       //欧拉角
    int16_t AccValue[3];        //加速度传感器原始数据
    int16_t GyroValue[3];          //陀螺仪原始数据
    nrf_gpio_pin_set(LED_1);  
    while (mpu_dmp_init()) {
        NRF_LOG_INFO("mpu6050 init error %d",mpu_dmp_init());
        NRF_LOG_FLUSH();
        nrf_delay_ms(1000);
    }
    nrf_gpio_cfg_output(LED_1);
    while(true) {
        if(mpu_dmp_get_data(&pitch,&roll,&yaw) == 0)
        {
        
        //加速度传感器原始数据
        // Read acc value from mpu6050 internal registers and save them in the array
        if(MPU6050_ReadAcc(&AccValue[0], &AccValue[1], &AccValue[2]) == true) {
          // display the read values
          NRF_LOG_INFO("ACC Values:  x = %d  y = %d  z = %d",
            AccValue[0], AccValue[1], AccValue[2]); 
          NRF_LOG_FLUSH();
        } else {
          // if reading was unsuccessful then let the user know about it
          NRF_LOG_INFO("Reading ACC values Failed!!!");
          NRF_LOG_FLUSH();
        }
        
        //读取陀螺仪数据
        // read the gyro values from mpu6050's internal registers and save them in another array
        if(MPU6050_ReadGyro(&GyroValue[0], &GyroValue[1], &GyroValue[2]) == true) 
        {
          // display then values
          NRF_LOG_INFO("GYRO Values: x = %d  y = %d  z = %d",
            GyroValue[0], GyroValue[1], GyroValue[2]);
          NRF_LOG_FLUSH();
        } else {
          NRF_LOG_INFO("Reading GYRO values Failed!!!");
          NRF_LOG_FLUSH();
        }

        mpu_6050_read_temp(&Temperature_centigrade);
        mpu6050_send_dat(AccValue[0],AccValue[1],AccValue[2],
        GyroValue[0],GyroValue[1],GyroValue[2]);//用自定义帧发送加速度和陀螺仪原始数据
        Uart_ReportIMU(
        AccValue[0],AccValue[1],AccValue[2],
        GyroValue[0],GyroValue[1],GyroValue[2],
        (int)(roll*100),(int)(pitch*100),(int)(yaw*10));
        nrf_delay_ms(50);
        }
    }
}

void mpu_run(void){

    uint8_t id;
    bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS); // initialize the leds and buttons
    log_init();
    uart_config();
    nrf_drv_clock_init();
    twi_master_init(); // initialize the twi 
    nrf_delay_ms(2000); // give some delay
    while(mpu6050_init(0x68) == false) // wait until MPU6050 sensor is successfully initialized
    {
      NRF_LOG_INFO("MPU_6050 initialization failed!!!"); // if it failed to initialize then print a message
      NRF_LOG_FLUSH();
      nrf_delay_ms(800);
    }
    nrf_delay_ms(1000);
    NRF_LOG_INFO("mpu6050 init ok"); 
    mpu6050_register_read(0x75U, &id, 1);
    NRF_LOG_INFO("mpu6050 id is %d",id);
    NRF_LOG_FLUSH();

//    xTaskCreate(
//        led_toggle_task_function, "LED0", 
//        configMINIMAL_STACK_SIZE*2, NULL, 2, 
//        &led_toggle_task_handle
//    );

    xTaskCreate(
        mpu6050_analysis_task_function, "MPU6050", 
        configMINIMAL_STACK_SIZE*2 + 1, NULL, 3, 
        &mpu6050_analysis_task_handle
    );
    vTaskStartScheduler();
    
}



