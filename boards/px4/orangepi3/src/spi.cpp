#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(0, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20602, 0),
	}),
	initSPIBus(0, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20689, 1),
	}),
	initSPIBus(1, {
		initSPIDevice(DRV_BARO_DEVTYPE_MS5611,  0),
	}),
	initSPIBus(1, {
		initSPIDevice(DRV_FLOW_DEVTYPE_PMW3901,  1),
	}),
};
