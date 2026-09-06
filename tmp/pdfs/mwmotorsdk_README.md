# 电机SDK测试实例说明  
可将[`MWSDK`](MWSDK)中的文件复制到自己的工作环境下即可使用。***[MWMOTOR SDK快速上手教程](https://bcnyljrhe70u.feishu.cn/docx/KRJcdmc3yovKxsxNnXOcNmppnwf?from=from_copylink)***
## 一、测试环境  
本实例使用的控制芯片为STM32G431VB（FDCAN）和大疆C板STM32F407IGH（CAN），由STM32CUBEMX生成HAL库工程，CAN通信波特率为500K BPS，配置使用FDCAN1/CAN1的收发功能，启用CAN中断进行接收，注意CAN输入输出引脚。 
## 二、测试代码说明  
### CANDrive.c  
包含CAN滤波器的初始化和CAN的收发函数，详细函数说明见CANDrive.h文件。  
### MWMotor.c  
MW电机软件开发工具包(SDK)  
**使用前请在`MWMotor.h`自行定义固件版本号（可通过上位机获取到固件版本号，当前版本默认为0.5.13，`FIRMWARE_VERSION_NUMBER`为513）**  
使用本SDK时，用户需要自行创建`MW_MOTOR_ACCESS_INFO`类型的电机信息结构体，并在总线初始化时将此结构体提供给`MWRegisterMotor()`进行总线初始化，用户定义此结构体时，需要提供：   
`busId`：电机总线ID  
`nodeId`：电机节点ID  
`motorData`：电机反馈数据结构体指针  
`sender`：电机发送数据函数指针（**此函数指针的函数需要用户自行定义**）  
`notifier`：电机通知接收函数指针（**此函数指针的函数需要用户自行定义**）  
***注意：使用MWRegisterBus()进行创建总线时，默认busId不可超过8，nodeId不可超过16（如有需求请自行更改SDK中的宏定义`MAX_BUS_NUM`和`MAX_MOTOR_NUM_PER_BUS`）！！！***  
### MWTest.c  
本文件中包含所有测试所需创建的变量和调用的函数，测试所实现的功能有： 
- 使用Endpoint中的参数设置线电流最大值  
- 进行上电校准  
- 当MODE_TEST为1时进行位置滤波控制模式测试  
- 当MODE_TEST为2时进行斜坡速度控制模式测试   
（MODE_TEST在MWTest.h中定义）  
### fdcan.c  
在`MX_FDCAN1_Init()`的用户代码段调用，进行滤波器和CAN接收中断的初始化。  
``` c
    /* FDCAN初始化 */
    CanFilter_Init();
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
```  
覆写HAL库中断回调函数进行数据接收。  
``` c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan->Instance == FDCAN1)
    {
        /* 获取电机can id */
        uint32_t can_id = CAN_Receive_DataFrame(&hfdcan1, FDCAN1_buff);
        /* 电机数据接收 */
        MWReceiver(1, can_id, FDCAN1_buff);
    }
}
```
### main.c
在主函数用户代码段中调用`MWFunctionTest()`，进行测试。 
