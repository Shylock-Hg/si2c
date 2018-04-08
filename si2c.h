#ifndef _SI2C_H_
#define _SI2C_H_

/* **********reg for multi-config -- unimplement now********** */

//! \defgroup si2c_config the configuration of i2c protocol implement 
/// @{

typedef enum si2c_mode {
	SI2C_MODE_MASTER,
	SI2C_MODE_SLAVE
} si2c_mode_t;

typedef enum si2c_addr {
	SI2C_7_BITS_ADDR,
	SI2C_10_BITS_ADDR
} si2c_addr_bits_t;

typedef struct si2c_dev {
	si2c_mode_t      mode;  //!< master or slave
	si2c_addr_bits_t bits;  //!< addr bits of slave device
	uint16_t         freq_kHz;  //!< frequency of i2c scl
	uint16_t         addr;  //!< own addr of slave mode
} si2c_dev_t;

///! @}  end of group si2c_config

#ifdef __cplusplus
	extern "C" {
#endif

///! \defgroup si2c_api interface to user 
/// @{

/*! \brief test addr of slave 
 *  \retval addr of slave
 * */
uint8_t si2c_test_slave(void);

/*! \brief init si2c 
 *  \param conf configuration of i2c protocol
 * */
void si2c_init(si2c_dev_t * conf);

/*! \brief write bytes to slave reg 
 *  \param slave -- slave addr
 *  \param reg -- reg addr
 *  \param bytes[in] -- data write to slave reg
 *  \param len -- length of data
 *  \retval 0==ok ; -1==err
 * */
int si2c_write_reg(uint8_t slave, uint8_t reg,const uint8_t * bytes, size_t len);

/*! \brief read bytes from slave reg 
 *  \param slave -- slave addr
 *  \param reg -- addr to read from slave
 *  \param bytes[out] --  bytes to store data read frome slave reg
 *  \param len -- length of bytes
 *  \retval 0==ok ; -1==err
 * */
int si2c_read_reg(uint8_t slave, uint8_t reg, uint8_t * bytes, size_t len);

/// @} group si2c_api

#ifdef __cplusplus
	}
#endif

#endif  //!< _SI2C_H_


