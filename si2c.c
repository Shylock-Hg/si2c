/*! ******************** i2c protocol low level implement dependent to platform ********************/
//!< dependent to 1.io init 2.io rw 3._delay_us 4.printf_DBG
//!< limitation: 1.only master mode 2.only 7-bit addr

/*  \author Shylock Hg
 *  \date   2018/2/9
 *  \version 0.0.1
 * */

#include "AS569.h"
#include "dbg.h"

#include "si2c.h"

#define  _si2c_io_pin_init    GPIO_Init
#define  _SI2C_IO_OUTPUT      OUTPUT
#define  _SI2C_IO_INPUT       INPUT


#define  _si2c_delay_us       _delay_us
//#define  _si2c_delay_ms  _delay_ms

/*! ******************** i2c master protocol implement ********************/

/*! ******************** i2c master io define ********************/
#define _SI2C_SDA_PORT  GPIOA
#define _SI2C_SDA_PIN   29
#define _SI2C_SDA_IO    GpioRegs.GPADATA.bit.GPA29

#define _SI2C_SCL_PORT  GPIOA
#define _SI2C_SCL_PIN   28
#define _SI2C_SCL_IO    GpioRegs.GPADATA.bit.GPA28



/*! ******************** i2c protocol define ********************/
#define SI2C_R_Msk 0x01
#define SI2C_W_Msk 0x00

//!< timeout for ack/nack
#define SI2C_TIMEOUT_US  10  //!< 10us

/*  *********** si2c dev reg **********/
//!< i2c dev reg , unused in current version
//!< default initialized
//!< | [MASTER_MODE] | [7-BITS_ADDR] | [freq kHz] | [addr] |
static si2c_dev_t dev = {SI2C_MODE_MASTER,SI2C_7_BITS_ADDR,400,0x00};  

/*! \brief init si2c io pin 
 *         sda & scl pin
 * */
static void si2c_io_init(void){
	//!< hardware pull-up 
	_si2c_io_pin_init(_SI2C_SDA_PORT,_SI2C_SDA_PIN,_SI2C_IO_OUTPUT);
	_si2c_io_pin_init(_SI2C_SCL_PORT,_SI2C_SCL_PIN,_SI2C_IO_OUTPUT);
}

/*! \brief send a i2c start signal */
static void si2c_start(void){
	//!< set sda output
	_si2c_io_pin_init(_SI2C_SDA_PORT,_SI2C_SDA_PIN,_SI2C_IO_OUTPUT);

	//!< stop
	_SI2C_SCL_IO = 1;
	_si2c_delay_us(1000/dev.freq_kHz/2);
	_SI2C_SDA_IO = 1;
	_si2c_delay_us(1000/dev.freq_kHz/2);

	//!< start
	_SI2C_SDA_IO = 0;
	_si2c_delay_us(1000/dev.freq_kHz/2);
	_SI2C_SCL_IO = 0;
	_si2c_delay_us(1000/dev.freq_kHz/2);
}

/*! \brief send a i2c end signal */
static void si2c_stop(void){
	//!< set sda output
	_si2c_io_pin_init(_SI2C_SDA_PORT,_SI2C_SDA_PIN,_SI2C_IO_OUTPUT);

	//!< start
	_SI2C_SDA_IO = 0;
	_si2c_delay_us(1000/dev.freq_kHz/2);
	_SI2C_SCL_IO = 0;
	_si2c_delay_us(1000/dev.freq_kHz/2);

	//!< stop
	_SI2C_SCL_IO = 1;
	_si2c_delay_us(1000/dev.freq_kHz/2);
	_SI2C_SDA_IO = 1;
	_si2c_delay_us(1000/dev.freq_kHz/2);
}

/*! \brief send ack to slave*/
static void si2c_send_ack(void){
	//!< set sda output
	_si2c_io_pin_init(_SI2C_SDA_PORT,_SI2C_SDA_PIN,_SI2C_IO_OUTPUT);


	_SI2C_SDA_IO = 0;
	_si2c_delay_us(1000/dev.freq_kHz/2);

	//!< scl pulse
	_SI2C_SCL_IO = 1;
	_si2c_delay_us(1000/dev.freq_kHz/2);
	_SI2C_SCL_IO = 0;
	_si2c_delay_us(1000/dev.freq_kHz/2);
}

/*! \brief send nack to slave */
static void si2c_send_nack(void){
	//!< set sda output
	_si2c_io_pin_init(_SI2C_SDA_PORT,_SI2C_SDA_PIN,_SI2C_IO_OUTPUT);

	_SI2C_SDA_IO = 1;
	_si2c_delay_us(1000/dev.freq_kHz/2);

	_SI2C_SCL_IO = 1;
	_si2c_delay_us(1000/dev.freq_kHz/2);
	_SI2C_SCL_IO = 0;
	_si2c_delay_us(1000/dev.freq_kHz/2);
}

/*! \brief read ACK from slave 
 *  \retval 0--ACK  1--NACK
 *
 * */
static int si2c_read_ack(void){
	int ret = 0;

	_si2c_io_pin_init(_SI2C_SDA_PORT,_SI2C_SDA_PIN,_SI2C_IO_INPUT);

	_si2c_delay_us(1000/dev.freq_kHz/2);
	_SI2C_SCL_IO = 1;
	_si2c_delay_us(1000/dev.freq_kHz/2);
	ret = _SI2C_SDA_IO;
	_SI2C_SCL_IO = 0;
	_si2c_delay_us(1000/dev.freq_kHz/2);

	return ret;
}

/*! \brief read byte from slave 
 *  \retval byte read from slave
 * */
static uint8_t si2c_read_byte(void){
	int i = 0;
	uint8_t byte = 0;

	_si2c_io_pin_init(_SI2C_SDA_PORT,_SI2C_SDA_PIN,_SI2C_IO_INPUT);

	for(i=0; i<8; i++){
		_si2c_delay_us(1000/dev.freq_kHz/2);
		_SI2C_SCL_IO = 1;
		_si2c_delay_us(1000/dev.freq_kHz/2);

		byte<<=1;
		if(1 == _SI2C_SDA_IO){
			byte |= 0x01;
		}

		_SI2C_SCL_IO = 0;
		_si2c_delay_us(1000/dev.freq_kHz/2);
	}

	return byte;
}

/*! \brief wirte byte to slave 
 *  \param byte--write to slave
 * */
static void si2c_write_byte(uint8_t byte){
	int i = 0;

	_si2c_io_pin_init(_SI2C_SDA_PORT,_SI2C_SDA_PIN,_SI2C_IO_OUTPUT);

	for(i=0; i<8; i++){
		_si2c_delay_us(1000/dev.freq_kHz/2);
		if(0x80 & byte){
			_SI2C_SDA_IO = 1;
		}else{
			_SI2C_SDA_IO = 0;
		}
		//!< next bit
		byte <<= 1;

		_si2c_delay_us(1000/dev.freq_kHz/2);
		//!< scl pulse
		_SI2C_SCL_IO = 1;
		_si2c_delay_us(1000/dev.freq_kHz/2);
		_SI2C_SCL_IO = 0;

	}
	_si2c_delay_us(1000/dev.freq_kHz/2);
}

/*! ******************** simulated i2c master init&rw API independent to platform ********************/

/*! \brief read bytes from slave reg 
 *  \param slave -- slave addr
 *  \param reg -- addr to read from slave
 *  \param bytes[out] --  bytes to store data read frome slave reg
 *  \param len -- length of bytes
 *  \retval 0==ok ; -1==err
 * */
int si2c_read_reg(uint8_t slave, uint8_t reg, uint8_t * bytes, size_t len){
	int i = 0;  //!< iterator for bytes receive
	int timeout = 0;  
	//!< start i2c protocol
	si2c_start();
	//!< write slave_addr
	si2c_write_byte((slave<<1)|SI2C_W_Msk);
	//!< wait ACK
	while(0 != si2c_read_ack()){
		timeout++;
		_si2c_delay_us(1000/dev.freq_kHz/2);
		if(SI2C_TIMEOUT_US < timeout){
			printf_DBG("[err]:si2c timeout 0!\r\n");
			return -1;
		}
	}
	//printf_DBG("[info]:ok=%2x\r\n",slave);
	//!< wirte reg addr
	timeout = 0;
	si2c_write_byte(reg);
	//!< wait ACK
	while(0 != si2c_read_ack()){
		timeout++;
		_si2c_delay_us(1000/dev.freq_kHz/2);
		if(SI2C_TIMEOUT_US < timeout){
			printf_DBG("[err]:si2c timeout 1!\r\n");
			return -1;
		}
	}
	si2c_stop();

	//!< restart to read
	si2c_start();
	//!< write slave to R_mode
	timeout = 0;
	si2c_write_byte((slave<<1)|SI2C_R_Msk);
	//!< wait ACK
	while(0 != si2c_read_ack()){
		timeout++;
		_si2c_delay_us(1000/dev.freq_kHz/2);
		if(SI2C_TIMEOUT_US < timeout){
			printf_DBG("[err]:si2c timeout 2!\r\n");
			return -1;
		}
	}

	//!< read bytes from slave reg
	for(; i<len; i++){
		//!< read byte
		*(bytes+i) = si2c_read_byte();
		//!< send ack except last byte
		if(len-1 == i){ 
			si2c_send_nack();  //!< for last byte
		}else{
			si2c_send_ack();  
		}
	}
	//!< terminate i2c protocol
	si2c_stop();
	return 0;
}

/*! \brief write bytes to slave reg 
 *  \param slave -- slave addr
 *  \param reg -- reg addr
 *  \param bytes[in] -- data write to slave reg
 *  \param len -- length of data
 *  \retval 0==ok ; -1==err
 * */
int si2c_write_reg(uint8_t slave, uint8_t reg, const uint8_t * bytes, size_t len){
	int i = 0;  //!< iterator for bytes receive
	int timeout = 0;
	//!< start i2c protocol
	si2c_start();
	//!< write slave addr byte
	si2c_write_byte((slave<<1)|SI2C_W_Msk);
	while(0 != si2c_read_ack()){
		timeout++;
		_si2c_delay_us(1000/dev.freq_kHz/2);
		if(SI2C_TIMEOUT_US < timeout){
			printf_DBG("[err]:si2c timeout!\r\n");
			return -1;
		}
	}
	//!< write reg addr
	timeout = 0;
	si2c_write_byte(reg);
	while(0 != si2c_read_ack()){
		timeout++;
		_si2c_delay_us(1000/dev.freq_kHz/2);
		if(SI2C_TIMEOUT_US < timeout){
			printf_DBG("[err]:si2c timeout!\r\n");
			return -1;
		}
	}

	//!< wirte bytes to reg
	for(; i<len; i++){
		timeout = 0;
		si2c_write_byte(*(bytes+i));
		while(0 != si2c_read_ack()){
			timeout++;
			_si2c_delay_us(1000/dev.freq_kHz/2);
			if(SI2C_TIMEOUT_US < timeout){
				printf_DBG("[err]:si2c timeout!\r\n");
				return -1;
			}
		}
	}

	//!< terminate protocol
	si2c_stop();
	return 0;
}

/*! \brief init si2c for multi mode
 *  \param conf--useless this version
 * */
void si2c_init(si2c_dev_t * conf){
	if(NULL != conf){
		memcpy(&dev,conf,sizeof(dev));  //!< init config to def si2c dev behavior
	}
	si2c_io_init();
}

/*  \brief test addr of slave 
 *  \retval addr of slave
 * */
uint8_t si2c_test_slave(void){
	int timeout = 0;  
	uint8_t slave = 0;
	while(1){
		//!< start i2c protocol
		si2c_start();
		//!< write slave_addr
		si2c_write_byte((slave<<1)|SI2C_W_Msk);
		slave++;
		//!< wait ACK
		while(0 != si2c_read_ack()){
			timeout++;
			_si2c_delay_us(1000/dev.freq_kHz/2);
			if(SI2C_TIMEOUT_US < timeout){
				//printf_DBG("[err]:si2c timeout 0!\r\n");
				break;
			}
		}
		if(SI2C_TIMEOUT_US >= timeout){
			printf_DBG("[info]:ok slave=%2x\r\n",slave);
			return slave;
		}
		timeout = 0;
	}
}

