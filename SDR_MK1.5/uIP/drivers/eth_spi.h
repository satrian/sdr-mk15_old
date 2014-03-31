
//#include "compiler.h"
//#include "evk1101/evk1101.h"

//#include "spi.h"
//#include "spi_master.h"


#define SPI_ETH_SPI0				0				// we use SPI0
#define SPI_ETH_BAUDRATE			20000000		//30000000	//		//1000000

#define ETH_SPI						(&AVR32_SPI)
#define ETH_SPI_NPCS				0									// SPARE_SPI_NPCS for CS2 etc.
#define ETH_SPI_SCK_PIN				AVR32_SPI_SCK_0_0_PIN
#define ETH_SPI_SCK_FUNCTION		AVR32_SPI_SCK_0_0_FUNCTION
#define ETH_SPI_MISO_PIN			AVR32_SPI_MISO_0_0_PIN
#define ETH_SPI_MISO_FUNCTION		AVR32_SPI_MISO_0_0_FUNCTION
#define ETH_SPI_MOSI_PIN			AVR32_SPI_MOSI_0_0_PIN
#define ETH_SPI_MOSI_FUNCTION		AVR32_SPI_MOSI_0_0_FUNCTION
#define ETH_SPI_NPCS_PIN			AVR32_SPI_NPCS_0_0_PIN			//AVR32_SPI_NPCS_2_0_PIN for CS2
#define ETH_SPI_NPCS_FUNCTION		AVR32_SPI_NPCS_0_0_FUNCTION		//AVR32_SPI_NPCS_2_0_FUNCTION for CS2

static const gpio_map_t ETH_SPI_GPIO_MAP =
{
	{ETH_SPI_SCK_PIN,          ETH_SPI_SCK_FUNCTION         },  // SPI Clock.
	{ETH_SPI_MISO_PIN,         ETH_SPI_MISO_FUNCTION        },  // MISO.
	{ETH_SPI_MOSI_PIN,         ETH_SPI_MOSI_FUNCTION        },  // MOSI.
	{ETH_SPI_NPCS_PIN,		   ETH_SPI_NPCS_FUNCTION		}	// Chip Select NPCS0
};
