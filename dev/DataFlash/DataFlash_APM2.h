/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* DataFlash_APM2 Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_APM2_H__
#define __DATAFLASH_APM2_H__

#include <AP_HAL.h>
#include "DataFlash.h"

class DataFlash_APM2 : public DataFlash_Block
{
	public:
		// DataFlash_Class Override
		void Init(const struct LogStructure *structure, uint8_t num_types) override;
		bool CardInserted() override;

		// DataFlash_Block Override
		void ReadManufacturerID(void) override;

	private:

		// DataFlash_Block Overrides
		void BufferToPage (uint8_t BufferNum, uint16_t PageAdr, uint8_t wait) override;
		void PageToBuffer(uint8_t BufferNum, uint16_t PageAdr) override;
		void PageErase(uint16_t PageAdr) override;
		void BlockErase(uint16_t BlockAdr) override;
		void ChipErase(void) override;
		void WaitReady(void) override;

		// write size bytes of data to a page. The caller must ensure that
		// the data fits within the page, otherwise it will wrap to the
		// start of the page
		// If pHeader is not NULL then write the header bytes before the data
		void BlockWrite(uint8_t BufferNum, uint16_t IntPageAdr, 
						   const void *pHeader, uint8_t hdr_size,
						   const void *pBuffer, uint16_t size) override;

		// read size bytes of data to a page. The caller must ensure that
		// the data fits within the page, otherwise it will wrap to the
		// start of the page
		bool BlockRead(uint8_t BufferNum, uint16_t IntPageAdr, void *pBuffer, uint16_t size) override;

	private:
		uint8_t ReadStatusReg(void);
		uint8_t ReadStatus(void);
		uint16_t PageSize(void);

		uint8_t BufferRead (uint8_t BufferNum, uint16_t IntPageAdr);

		// take a semaphore safely
		bool _sem_take(uint8_t timeout);

		AP_HAL::SPIDeviceDriver* _spi;
		AP_HAL::Semaphore* _spi_sem;
};

#endif
