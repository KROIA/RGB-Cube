#ifndef I2C_PARALLELBUS_H
#define I2C_PARALLELBUS_H

#include "Arduino.h"

#define I2C_PARALLEL_MAX_DATA_LINES 31
#define I2C_PARALLEL_MAX_BUFFER_SIZE 254

class I2C_parallel
{
	public:
		I2C_parallel(volatile uint32_t *port, uint8_t dataLines);
		~I2C_parallel();
		
		//void setClockSpeed(
		uint8_t getDataLines();
		
		void setClockOffset(uint8_t offset);
		void setDataOffset(uint8_t dataIndex, uint8_t offset);
		
		void setBufferSize(uint16_t bytes);
		uint16_t getBufferSize();
		
		// Set a 2D array with size: [dataLines][length]
		void write(uint8_t **dataArray , uint16_t length);
		
		// Set a 1D array with size: [length] to port <dataPort>
		//void write(uint8_t dataPort,    uint8_t *dataArray, uint16_t length);
		
		// Set 1 bytes for each dataPort
		void write(uint8_t *portArray);
		
		// Set 1 byte for dataPort <dataPort>
		//void wrire(uint8_t dataPort,    uint8_t data);
		
		void send();
		
		
	
	private:
		void freeBuffer();
		void I2C_ISR();
		enum BusStep
		{
			i2c_startCondition = 0,
			i2c_lowerSCL = 1,
			i2C_setData = 2,
			i2c_riseSCL = 3,
			i2c_setDataLow = 4,
			i2c_lastSCLRise = 5,
			i2c_stopCondition = 6,
			
			i2c_riseSCL_acknowlage = 8,
			i2c_lowerSCL_acknowlage = 9,
			i2c_waitForEqualSpacedSCLSignal = 10,
			i2c_riseSCL_acknowlageWait = 11,
			i2c_acknowlageRead = 12,
			i2c_sendNextByteOrStop = 13,
			i2c_reset = 14
		};
	
	
		volatile uint32_t 	*m_port;
		uint8_t 			 m_dataLines;
		
		uint8_t				*m_dataOffset;
		uint8_t				 m_clockOffset;
		
		uint8_t 			**m_busBuffer;     
		uint8_t				 m_bufferSize;     // Reserved storage size for the Bus
		
		uint8_t				 m_usedBufferSize; // current used Buffer (always smaler or equal to m_bufferSize)
};
#endif
