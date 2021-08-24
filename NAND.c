/****
 * REMEMBER, THIS APPENDS DATA - DELETE CONTENTS BEFOREHAND
 */

#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* Driver Header files */
#include <ESLO.h>
#include <SPI_NAND.h>
#include <Serialize.h>

#include <ti/drivers/NVS.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* Driver configuration */
#include <ti_drivers_config.h>

// NAND user defined
uint8_t ret;
uint16_t devId;
uAddrType esloAddr = 0x00000000;
uint8_t readBuf[PAGE_SIZE]; // 2176, always allocate full page size
uint32_t i;

// !! erase memory.dat before running
void* mainThread(void *arg0) {
	uint32_t tempSignature;
	uint32_t tempVersion;
	uAddrType tempAddress;

	GPIO_init();
	SPI_init();

	GPIO_write(_SHDN, GPIO_CFG_OUT_LOW); // ADS129X off
	GPIO_write(LED_0, CONFIG_GPIO_LED_OFF);

	NAND_Init(CONFIG_SPI, _NAND_CS);
	ret = FlashReadDeviceIdentification(&devId); // 0x2C25

	nvsHandle = NVS_open(ESLO_NVS_0, &nvsParams);

	if (nvsHandle != NULL) {
		NVS_getAttrs(nvsHandle, &regionAttrs);
		NVS_read(nvsHandle, 0, (void*) nvsBuffer, sizeof(nvsBuffer));
		// compare eslo sig
		ESLO_decodeNVS(nvsBuffer, &tempSignature, &tempVersion, &tempAddress);
	}

	tempAddress = 0x78000; // !! RMRMRM temp
	// could find last block first, then for loop
	while (1) {
		ret = FlashPageRead(esloAddr, readBuf);

		GPIO_toggle(LED_0); // GEL breakpoint
		esloAddr += 0x00001000; // +1 page

		if (esloAddr > tempAddress | esloAddr > FLASH_SIZE) {
			break;
		}
	}

	GPIO_write(LED_0, CONFIG_GPIO_LED_OFF);
	return (0);
}
