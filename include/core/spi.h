#ifndef INC_SPI_H
#define INC_SPI_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include "core/common-defines.h"
#include "core/fifo.h"

#define FIFO_BUFFER_SIZE    (128)    // Needs to be a power of 2 in order for the mask to work

typedef union spi_handle_t   {
    uint32_t spi_base_address;
    uint32_t rcc_address;
    uint32_t rcc_reset_address;
    uint8_t nvic_irq_id;
    fifo_buffer_t fifo;
   
    
    
}spi_handle_t;

void spi_setup(spi_handle_t* spi);
void spi_teardown(spi_handle_t* spi);
void spi_read(spi_handle_t* spi, uint8_t* data, uint32_t length);
uint8_t spi_read_byte(spi_handle_t* spi);
void spi_write(spi_handle_t* spi, uint8_t* data, uint32_t length);
void spi_write_byte(spi_handle_t* spi, uint8_t byte);
bool spi_data_available(spi_handle_t* spi);

#endif/* INC_SPI_H */