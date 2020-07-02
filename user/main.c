/**
******************************************************************************
* @file    Project/main.c 
* @author  MCD Application Team
* @version V2.2.0
* @date    30-September-2014
* @brief   Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/ 
/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_clk.h"
#include "stm8s_i2c.h"
#include "stm8s_spi.h"
#include "stm8s_it.h"
#include "stm8s_gpio.h"
#include "stm8s_uart1.h"
//#include "spi.h"
//#include "i2c.h"

/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




//#if !defined(SERIAL_TX_BUFFER_SIZE)
//#if ((RAMEND - RAMSTART) < 1023)
//#define SERIAL_TX_BUFFER_SIZE 8
//#else
#define SERIAL_TX_BUFFER_SIZE 128        //buffer大小
#define SPI_TX_BUFFER_SIZE    128   
//#endif
//#endif
//#if !defined(SERIAL_RX_BUFFER_SIZE) //用于选择发送和接收的Buffer大小
//#if ((RAMEND - RAMSTART) < 1023)
//#define SERIAL_RX_BUFFER_SIZE 16
//#else
//#define SERIAL_RX_BUFFER_SIZE 64
//#endif
//#endif


//#if (SERIAL_TX_BUFFER_SIZE>256)   //根据buffer的大小，确定头尾指针的类型
//typedef uint16_t tx_buffer_index_t;    
//#else
typedef uint16_t tx_buffer_index_t;
//#endif
//#if  (SERIAL_RX_BUFFER_SIZE>256)
//typedef uint16_t rx_buffer_index_t;  //存储接收数据的buffer大小
//#else
//typedef uint8_t rx_buffer_index_t;
//#endif


#define	I2C_SLAVE_ADDRES (0x10)

volatile uint16_t temp = 0;
volatile uint8_t  data = 0;  
unsigned char a = 0;
unsigned char b = 1;
unsigned char c = 0;
uint8_t regaddr; //在i2c中获得regaddr的值

//unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
uint8_t _tx_buffer[SERIAL_TX_BUFFER_SIZE];  //设定存储数据的数组

uint8_t _spi_buffer[SPI_TX_BUFFER_SIZE];  //设定存储数据的数组

//rx_buffer_index_t _rx_buffer_head; 
//rx_buffer_index_t _rx_buffer_tail;
volatile tx_buffer_index_t _tx_buffer_head = 0 ;  //不需要设定初始值吗？
volatile tx_buffer_index_t _tx_buffer_tail = 0 ;  

volatile tx_buffer_index_t _spi_buffer_head = 0 ;  //不需要设定初始值吗？
volatile tx_buffer_index_t _spi_buffer_tail = 0 ;  


void Delay(u32 nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
} 

void uart_init( uint32_t baudrate)
{
	//Initalize the PD6, it's must to do this
	GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT );
	GPIO_Init(GPIOD,GPIO_PIN_5,GPIO_MODE_IN_PU_NO_IT);
	
	//RX_EN();
	//Reset the UART1
	UART1_DeInit();
	
	//Initalize the UART1
	UART1_Init((uint32_t)(baudrate),                    \
			  UART1_WORDLENGTH_8D,                      \
			  UART1_STOPBITS_1,                         \
			  UART1_PARITY_NO ,                         \
			  UART1_SYNCMODE_CLOCK_DISABLE ,            \
			  UART1_MODE_TXRX_ENABLE);

	UART1_ClearITPendingBit(UART1_IT_RXNE);
	UART1_ITConfig( UART1_IT_RXNE_OR, DISABLE);

	//Open UART1
	UART1_Cmd( ENABLE );
	

	//Enable interrupt of all.
	//__enable_interrupt(); 
}


void uart1_txdata(u8 data){
  
    UART1_SendData8(data);
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
}

void uart1_txstr(u8 *p,u8 datalen){
  while(datalen){
    uart1_txdata(*p);
    p++;
    datalen--;
		
  }
  while (UART1_GetFlagStatus( UART1_FLAG_TC) == RESET);
}

void spi_write_buffer(uint8_t data){

  tx_buffer_index_t i = (_spi_buffer_head + 1) % SPI_TX_BUFFER_SIZE;  
  while(i == _tx_buffer_tail)  //说明buffer满了     
  {
      spi_wr_cmd(_spi_buffer[_spi_buffer_tail]);  
      _spi_buffer_tail = (_spi_buffer_tail + 1) % SPI_TX_BUFFER_SIZE;
      
  }
  _spi_buffer[_spi_buffer_head] = data; //将数据存入了_tx_buffer中，
  _spi_buffer_head = i;  //头指针增加
  
}


void i2c_init(void)
{
	//I2C_DeInit();

	GPIO_Init(GPIOB , GPIO_PIN_4 , GPIO_MODE_OUT_OD_HIZ_FAST);//I2C_SCL
	GPIO_Init(GPIOB , GPIO_PIN_5 , GPIO_MODE_OUT_OD_HIZ_FAST);//I2C_SDA

	I2C_Init((uint32_t)(800000), (I2C_SLAVE_ADDRES << 1), 
			 I2C_DUTYCYCLE_2, I2C_ACK_CURR , 
			 I2C_ADDMODE_7BIT, 16 );

	I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), ENABLE); //I2C中断
}


void spi_init(void)
{

   SPI_DeInit();
   SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER,\
            SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_1EDGE, \
            SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x07); 
  SPI_ITConfig(SPI_IT_RXNE, ENABLE);
   SPI_Cmd(ENABLE);     
}

void SPI_Slave_check_event_ISR(){
  uint8_t  data = 0;
  //uart1_txdata(0X67);
  //if(SPI_GetFlagStatus(SPI_FLAG_RXNE) != 1){
    data = SPI_ReceiveData();//SPI->DR;
    spi_write_buffer(data);
  //}  
}
void spi_wr_cmd(u8 cmd)
{
   GPIO_WriteLow(GPIOC,GPIO_PIN_3); 
//  GPIO_WriteLow(GPIOC,GPIO_PIN_4);      //DC=0
  SPI_SendData(cmd);
  //a = SPI_ReceiveData();
  GPIO_WriteHigh(GPIOC,GPIO_PIN_3);
}



void spi_wr_dat(u8 dat)
{
 GPIO_WriteLow(GPIOC,GPIO_PIN_3);      //CS
  //GPIO_WriteHigh(GPIOC,GPIO_PIN_4);      //DC=1
  SPI_SendData(dat);
  //a = SPI_ReceiveData();
 //GPIO_WriteHigh(GPIOC,GPIO_PIN_3);
}



void write_buffer(u8 data)
{

  
  tx_buffer_index_t i = (_tx_buffer_head + 1) % SERIAL_TX_BUFFER_SIZE;  
  while(i == _tx_buffer_tail)  //说明buffer满了     
  {
      spi_wr_cmd(_tx_buffer[_tx_buffer_tail]);  
      _tx_buffer_tail = (_tx_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;
      
  }
  _tx_buffer[_tx_buffer_head] = data; //将数据存入了_tx_buffer中，
  _tx_buffer_head = i;  //头指针增加
  
}


unsigned char read_buffer()
{
  //将存储的buffer中的数据发送出去
  //检验：将SPI的MOSI与MISO相接，用一个变量接收MISO的值，\
          看这个变量的值是否与MOSI发送的一致  

  if (_tx_buffer_head == _tx_buffer_tail)  //buffer为空，没有数据
  {
   return 0;
  } 
  else
  {
      spi_wr_cmd(_tx_buffer[_tx_buffer_tail] );  
     _tx_buffer_tail = (_tx_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;
  
  } 
  
}


//IIC接收数据和SPI发送数据都在此函数实现。（中断）
void I2C_byte_received(u8 u8_RxData)
{
	
//	regaddr = u8_RxData;
//        SPI_SendData(regaddr);
//        Delay(0xffff);
//        a = SPI_ReceiveData();
//        Delay(0xffff);
  //将收到的数据存在一个buffer 中
   
    write_buffer(u8_RxData);
	
}
void spi_read_buffer(){

  if (_si_buffer_head == _spi_buffer_tail)  //buffer为空，没有数据
  {
   return ;
  } 
  else
  {
      I2C->DR = (_spi_buffer[_spi_buffer_tail] );  
     _spi_buffer_tail = (_spi_buffer_tail + 1) % SPI_TX_BUFFER_SIZE;
  
  } 


}

void I2C_Slave_check_event_ISR(void) {
	//STM8内部寄存器的定义在stm8s.h中
	__IO static uint8_t sr1;					
	__IO static uint8_t sr2;
	__IO static uint8_t sr3;
	
	// save the I2C registers configuration 
	sr1 = I2C->SR1;   //I2C_SR1 ： I2C status register 1
	sr2 = I2C->SR2;   //I2C_SR2 ： I2C status register 2
	sr3 = I2C->SR3;   //I2C_SR3 ： I2C status register 3
	
	/* Communication error? */
	if (sr2 & (I2C_SR2_WUFH | I2C_SR2_OVR |I2C_SR2_ARLO |I2C_SR2_BERR))
	{		
		I2C->CR2|= I2C_CR2_STOP;  // stop communication - release the lines
		I2C->SR2= 0;					    // clear all error flags
	} 
	/* More bytes received ? */
	if ((sr1 & (I2C_SR1_RXNE | I2C_SR1_BTF)) == (I2C_SR1_RXNE | I2C_SR1_BTF))
	{
		//I2C_byte_received(I2C->DR);
	}
	/* Byte received ? */
	if (sr1 & I2C_SR1_RXNE)  //I2C_SR1_RXNE ： Data Register not Empty (receivers)
	{
		I2C_byte_received(I2C->DR); //I2C_DR ： I2C data register 
               // uart1_txdata(I2C->DR);
                
                
	}
	/* NAK? (=end of slave transmit comm) */
	if (sr2 & I2C_SR2_AF)
	{	
		I2C->SR2 &= ~I2C_SR2_AF;	  // clear AF
		//I2C_transaction_end();
	}
	/* Stop bit from Master  (= end of slave receive comm) */
	if (sr1 & I2C_SR1_STOPF) 
	{
		I2C->CR2 |= I2C_CR2_ACK;	  // CR2 write to clear STOPF
		//I2C_transaction_end();
	}
	/* Slave address matched (= Start Comm) */
	if (sr1 & I2C_SR1_ADDR)
	{	 
		//I2C_transaction_begin();
	}
	/* More bytes to transmit ? */
	if ((sr1 & (I2C_SR1_TXE | I2C_SR1_BTF)) == (I2C_SR1_TXE | I2C_SR1_BTF))
	{
	   spi_read_buffer();	
          //I2C->DR = 0x60;
	}
	/* Byte to transmit ? */
	if (sr1 & I2C_SR1_TXE)
	{
	  spi_read_buffer();	
          //I2C->DR = 0x60;
	}	
	//GPIOD->ODR^=1;
}

void main(void)
{
	//GPIO_WriteLow(GPIOC,GPIO_PIN_3);      //CS
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
        GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
	spi_init();
	//i2c_init();
	uart_init(115200);
       // uart1_txdata(0x88);
	__enable_interrupt();
        while(1)
        {
         read_buffer();
           
        }
}















#ifdef USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void assert_failed(u8* file, u32 line)
{ 
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	
	/* Infinite loop */
	while (1)
	{
	}
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
