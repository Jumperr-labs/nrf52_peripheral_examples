
/*! @file main.c
@brief firmware for M24C04 EpRom */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define HIGH 1
#define LOW 0
#define DATA_NACK -1
#define OK 1
#define UNSET 0


#define SCL_PIN 27
#define SDA_PIN 26
#define WC_PIN 28
#define M24C04_SIZE             512UL
#define M24C04_ADDRESS          0x50

/* instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);

/* Indicates if operation on TWI has ended. */
static volatile char m_xfer_done = UNSET;


/****************** Global Function Definitions *******************************/

void m24c04_handler(nrf_drv_twi_evt_t const * p_event, void * p_context) {
	switch (p_event->type) {
		case NRF_DRV_TWI_EVT_DONE:
			m_xfer_done = OK;
			break;
		case NRF_DRV_TWI_EVT_ADDRESS_NACK:
		case NRF_DRV_TWI_EVT_DATA_NACK:
			m_xfer_done = DATA_NACK;
			break;
		default:
			break;
	}
}

void twi_init (void) {
	ret_code_t err_code;
   const nrf_drv_twi_config_t m24c04_config = {
	   .scl                = SCL_PIN,
	   .sda                = SDA_PIN,
	   .frequency          = NRF_TWI_FREQ_100K,
	   .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
	   .clear_bus_init     = false
	};

	err_code = nrf_drv_twi_init(&m_twi, &m24c04_config, m24c04_handler, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi);
}

void WaitForDone()  {
	do {
		__WFE();
	} while (m_xfer_done == UNSET);
}


//*****************************************************************************
//
//! \brief Write one byte to M24Cxx.
//!
//! \param pucBuffer specifies the location data which will be written.
//! \param usWriteAddr specifies the address which data will be written.
//!
//!  This function is to write one byte to M24Cxx,one byte will be writen in
//!  appointed address.
//!
//! \return None.
//
//*****************************************************************************
void M24CxxByteWrite(const unsigned char* pucBuffer, unsigned short usWriteAddr) {
	ASSERT((usWriteAddr >= 0) && (usWriteAddr <= M24C04_SIZE));


	uint8_t address_to_send = (usWriteAddr >> 8) | M24C04_ADDRESS;
	uint8_t value[2] = {(unsigned char)((usWriteAddr & 0xff)),
						*pucBuffer};

	m_xfer_done = false;
	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(&m_twi, address_to_send, value, 2, false);
	APP_ERROR_CHECK(err_code);
	WaitForDone();
}

//*****************************************************************************
//
//! \brief Write sequence of bytes to M24Cxx.
//!
//! \param pucBuffer specifies the location data which will be written.
//! \param usWriteAddr specifies the address which data will be written.
//! \param len specifies the bytes length to write.
//!
//!  This function is to write one byte to M24Cxx,one byte will be written in
//!  appointed address.
//!
//! \return None.
//
//*****************************************************************************
void M24CxxWrite(const unsigned char* pucBuffer, unsigned short usWriteAddr, const unsigned short len) {
	ASSERT((usWriteAddr >= 0) && (usWriteAddr <= M24C04_SIZE));


	uint8_t address_to_send = (usWriteAddr >> 8) | M24C04_ADDRESS;
	uint8_t value[1] = {(unsigned char)((usWriteAddr & 0xff))	};

	m_xfer_done = false;
	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(&m_twi, address_to_send, value, 1, true);
	APP_ERROR_CHECK(err_code);
	WaitForDone();

	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, address_to_send, pucBuffer, len, false);
	APP_ERROR_CHECK(err_code);
	WaitForDone();
}


//*****************************************************************************
//
//! \brief Read one byte from M24Cxx.
//!
//! \param pucBuffer specifies the location data which will be read into.
//! \param usWriteAddr specifies the address which data will be read from.
//!
//!  This function is to read one byte from M24Cxx,one byte will be read from
//!  the appointed address.
//!
//! \return None.
//
//*****************************************************************************
void M24CxxByteRead(unsigned char* pucBuffer, unsigned short usReadAddr) {
	ASSERT((usReadAddr >= 0) && (usReadAddr <= M24C04_SIZE));


	uint8_t address_to_send = (usReadAddr >> 8) | M24C04_ADDRESS;
	uint8_t value[1] = {(unsigned char)((usReadAddr & 0xff))};

	m_xfer_done = false;
	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(&m_twi, address_to_send, value, 1, true);
	APP_ERROR_CHECK(err_code);
	WaitForDone();

	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, M24C04_ADDRESS, pucBuffer, 1);
	APP_ERROR_CHECK(err_code);
	WaitForDone();
}

//*****************************************************************************
//
//! \brief Read sequence of bytes from M24Cxx.
//!
//! \param pucBuffer specifies the location data which will be read into.
//! \param usWriteAddr specifies the address which data will be read from.
//! \param len specifies the bytes length to read.
//!
//!  This function is to read one byte from M24Cxx,one byte will be read from
//!  the appointed address.
//!
//! \return None.
//
//*****************************************************************************
void M24CxxRead(unsigned char* pucBuffer, unsigned short usReadAddr, unsigned short len) {
	ASSERT((usReadAddr >= 0) && (usReadAddr <= M24C04_SIZE));


	uint8_t address_to_send = (usReadAddr >> 8) | M24C04_ADDRESS;
	uint8_t value[1] = {(unsigned char)((usReadAddr & 0xff))};

	m_xfer_done = false;
	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(&m_twi, address_to_send, value, 1, true);
	APP_ERROR_CHECK(err_code);
	WaitForDone();

	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, M24C04_ADDRESS, pucBuffer, len);
	APP_ERROR_CHECK(err_code);
	WaitForDone();
}

void WriteReadTest() {
	unsigned char pucBuffer[1] = {0xFF};
	M24CxxByteWrite(pucBuffer, 0x0120);
	NRF_LOG_INFO("\r\nAfter first write\r\n");
	NRF_LOG_FLUSH();
	M24CxxByteRead(pucBuffer, 0x0120);


	NRF_LOG_INFO("\r\nAfter first read\r\n");
	NRF_LOG_FLUSH();
	NRF_LOG_INFO("  read value: %x.\r\n", *pucBuffer);
	NRF_LOG_FLUSH();

	NRF_LOG_INFO("\r\nGoing to sequence of write\r\n");
	NRF_LOG_FLUSH();
	pucBuffer[0] = 0xff;
	for (int i=0; i<512; i++)  {
			M24CxxByteWrite(pucBuffer, (unsigned short)i);
			pucBuffer[0]--;
	}
	NRF_LOG_INFO("\r\nFinished sequence of write\r\n");
	NRF_LOG_FLUSH();

	NRF_LOG_INFO("\r\nGoing to sequence of read\r\n");
	NRF_LOG_FLUSH();
	pucBuffer[0] = 0xFF;
	char tests_success = 1;
	for (int i=0; i<512; i++) {
		unsigned char readBuff[1];
		M24CxxByteRead(readBuff, (unsigned short)i);

		if (readBuff[0] != pucBuffer[0]) {
			tests_success = 0;

			NRF_LOG_INFO(" expected value: %x.\r\n", *pucBuffer);
			NRF_LOG_INFO(" Got value: %x.\r\n", *readBuff);
			NRF_LOG_INFO(" index: %d.\r\n", i);
			NRF_LOG_FLUSH();
		}
		pucBuffer[0]--;
	}
	NRF_LOG_INFO("\r\nFinished sequence of read\r\n");
	NRF_LOG_FLUSH();

	if (1 == tests_success) {
		NRF_LOG_INFO("\r\nM24C04: Read sequence OK\r\n");
		NRF_LOG_FLUSH();
	}
	else {
		NRF_LOG_INFO("\r\nM24C04: Read sequence failed\r\n");
		NRF_LOG_FLUSH();
	}

}

void  SequentialWriteReadTest() {

	NRF_LOG_INFO("\r\nTesting sequential write\r\n");
	NRF_LOG_FLUSH();
	const int page_size = 16;
	unsigned char pucBuffer2[page_size];
	unsigned char test_val =  0xFF;
	for (int i=0; i<page_size; i++) {
		pucBuffer2[i] = test_val;
		test_val--;
	}

	M24CxxWrite(pucBuffer2, 0x00, page_size);

	unsigned char readBuff2[page_size];
	M24CxxRead(readBuff2, 0x00, page_size);
	NRF_LOG_INFO("\r\nFinished sequential read\r\n");
	NRF_LOG_FLUSH();
	char tests_success = 1;
	for (int i=0; i<page_size; i++) {
		if (readBuff2[i] != pucBuffer2[i]) {
			tests_success = 0;
		}
	}

	if (1 == tests_success) {
		NRF_LOG_INFO("\r\nsequential read OK\r\n");
		NRF_LOG_FLUSH();
	}
	else {
		NRF_LOG_INFO("\r\nsequential read failed\r\n");
		NRF_LOG_FLUSH();
	}

}

void WriteProtectionTest() {

	NRF_LOG_INFO("\r\nTesting write protection\r\n");
	NRF_LOG_FLUSH();
	const int page_size = 16;
	unsigned char pucBuffer[page_size];
	unsigned char test_val =  0xFF;
	for (int i=0; i<page_size; i++) {
		pucBuffer[i] = test_val;
		test_val--;
	}

	M24CxxWrite(pucBuffer, 0x00, page_size);
	nrf_gpio_pin_write(WC_PIN, HIGH);
	memset(pucBuffer, 0, page_size);
	M24CxxWrite(pucBuffer, 0x00, page_size);

	unsigned char readBuff[page_size];
	M24CxxRead(readBuff, 0x00, page_size);
	char tests_success = 1;
	test_val = 0xFF;
	for (int i=0; i<page_size; i++) {
		if (readBuff[i] != test_val--) {
			tests_success = 0;
		}
	}

	if (1 == tests_success) {
		NRF_LOG_INFO("\r\nwrite protection OK\r\n");
		NRF_LOG_FLUSH();
	}
	else {
		NRF_LOG_INFO("\r\nwrite protection failed\r\n");
		NRF_LOG_FLUSH();
	}
}

int main(void) {
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

	NRF_LOG_INFO("\r\nM24C04\r\n");
	NRF_LOG_FLUSH();
	twi_init();
	nrf_gpio_cfg_output(WC_PIN);
	nrf_gpio_pin_write(WC_PIN, LOW);

	WriteReadTest();
	SequentialWriteReadTest();
	SequentialWriteReadTest();
	WriteProtectionTest();


	while(1) {}
}
