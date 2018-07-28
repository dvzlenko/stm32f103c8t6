#ifndef AD779X_STM32_H
#define AD779X_STM32_H

#define AD779X_SPI	SPI1

extern int spi_err;

int ad779x_stm32_init();
unsigned long ad779x_stm32_read(int chan);

#endif /*AD779X_STM32_H*/