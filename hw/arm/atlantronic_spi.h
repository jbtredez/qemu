#ifndef ATLANTRONIC_SPI_H
#define ATLANTRONIC_SPI_H

enum
{
	SPI_IRQ_OUT_HW = 0,        //!< it spi -> stm32
	SPI_IRQ_OUT_DMAR,          //!< it dma rx -> stm32
	SPI_IRQ_OUT_DEVICE0_RX,    //!< reception device : stm32 -> spi -> device0
	SPI_IRQ_OUT_DEVICE1_RX,    //!< reception device : stm32 -> spi -> device1
	SPI_IRQ_OUT_DEVICE2_RX,    //!< reception device : stm32 -> spi -> device2
	SPI_IRQ_OUT_MAX,
};

enum
{
	SPI_IRQ_IN_RX,    //!< reception donnees par le module spi (depuis un device spi connecte sur le bus)
	SPI_IRQ_IN_CS0,   //!< chip select 0
	SPI_IRQ_IN_CS1,   //!< chip select 1
	SPI_IRQ_IN_CS2,   //!< chip select 2
	SPI_IRQ_IN_MAX,
};

#endif
