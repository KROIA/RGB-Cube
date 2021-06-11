#include "I2C_parallelBus.h"

I2C_parallel::I2C_parallel(volatile uint32_t *port, uint8_t dataLines)
{
	m_port 			= port;
	m_dataLines 	= dataLines;
	
	if(m_dataLines > I2C_PARALLEL_MAX_DATA_LINES)
		m_dataLines = I2C_PARALLEL_MAX_DATA_LINES;
		
	m_busBuffer 	= new uint8_t*[m_dataLines];
	m_dataOffset	= new uint8_t[m_dataLines];
	setBufferSize(10);
	m_usedBufferSize = 0; 
}
I2C_parallel::~I2C_parallel()
{
	freeBuffer();
	delete[] m_busBuffer;
	delete[] m_dataOffset;
}
		
//void setClockSpeed(
uint8_t I2C_parallel::getDataLines()
{
	return m_dataLines;
}

void I2C_parallel::setClockOffset(uint8_t offset)
{
	m_clockOffset = offset;
}
void I2C_parallel::setDataOffset(uint8_t dataIndex, uint8_t offset)
{
	if(dataIndex >= m_dataLines)
		return; // out of range error
		
	m_dataOffset[dataIndex] = offset;
}

void I2C_parallel::setBufferSize(uint16_t bytes)
{
	if(m_bufferSize == bytes)
		return;
		
	if(I2C_PARALLEL_MAX_BUFFER_SIZE <= bytes)
		bytes = I2C_PARALLEL_MAX_BUFFER_SIZE;
		
	m_bufferSize = bytes;
	
	freeBuffer();
	
	for(uint8_t i=0; i<m_dataLines; i++)
	{
		m_busBuffer[i] = new uint8_t[m_bufferSize];
		for(uint8_t j=0; j<m_bufferSize; j++)
		{
			m_busBuffer[i][j] = 0;
		}
	}
}
uint16_t I2C_parallel::getBufferSize()
{
	return m_bufferSize;
}

// Set a 2D array with size: [dataLines][length]
void I2C_parallel::write(uint8_t **dataArray , uint16_t length)
{
	if(length > m_bufferSize - m_usedBufferSize)
		length = m_bufferSize - m_usedBufferSize; // Buffer is not large enough, skip the last bytes.
		
	uint8_t *byteBuffer = new uint8_t[m_dataLines];
	
	for(int i=0; i<length; i++)
	{
		for(uint8_t line=0; line<m_dataLines; line++)
		{
			byteBuffer[line] = dataArray[line][i];
			//this->write(line,dataArray[line],length);
		}
		this->write(byteBuffer);
	}
	delete[] byteBuffer; 
}

// Set a 1D array with size: [length] to port <dataPort>
/*void I2C_parallel::write(uint8_t dataPort,    uint8_t *dataArray, uint16_t length)
{
	if(length > m_bufferSize)
		length = m_bufferSize; // Buffer is not large enough, skip the last bytes.
		
	for(uint8_t dataByte=0; dataByte<length; dataByte++)
	{
		
	}
}*/

// Set 1 bytes for each dataPort
void I2C_parallel::write(uint8_t *portArray)
{
	if(m_usedBufferSize >= m_bufferSize)
		return; // Buffer is full, ignore incomming data
		
	for(uint8_t line=0; line<m_dataLines; line++)
	{
		m_busBuffer[line][m_usedBufferSize] = portArray[line];
	}	
	m_usedBufferSize++;
}

// Set 1 byte for dataPort <dataPort>
/*void I2C_parallel::wrire(uint8_t dataPort,    uint8_t data)
{
	
}*/

void I2C_parallel::send()
{
	
}

void I2C_parallel::freeBuffer()
{
	for(uint8_t i=0; i<m_dataLines; i++)
	{
		delete[] m_busBuffer[i];
	}
	
}