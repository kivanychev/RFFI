#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ================================================
//                    CONSTANTS
// ================================================

#define CONFIG_EXAMPLE_ENC28J60_SPI_HOST 1
#define CONFIG_EXAMPLE_ENC28J60_SCLK_GPIO 19
#define CONFIG_EXAMPLE_ENC28J60_MOSI_GPIO 23
#define CONFIG_EXAMPLE_ENC28J60_MISO_GPIO 25
#define CONFIG_EXAMPLE_ENC28J60_CS_GPIO 22
#define CONFIG_EXAMPLE_ENC28J60_SPI_CLOCK_MHZ 6
#define CONFIG_EXAMPLE_ENC28J60_INT_GPIO 4
#define CONFIG_EXAMPLE_ENC28J60_DUPLEX_HALF 1

// ================================================
//                FUNCTION PROTOTYPES
// ================================================

void enc28j60_init(void);


#ifdef __cplusplus
}
#endif



