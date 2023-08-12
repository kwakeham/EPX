
#include <stdint.h>
#include <stdbool.h>
#include "boards.h"
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_bootloader_dfu_timers.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"

#include "sdk_config.h"
#include "nrf_power.h"
#include "nrf_dfu_settings.h"
#include "nrf_dfu_utils.h"
#include "nrf_bootloader_wdt.h"
#include "nrf_bootloader_info.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_bootloader_fw_activation.h"
#include "nrf_bootloader_dfu_timers.h"
#include "nrf_dfu_validation.h"
#include "nrf_dfu_ver_validation.h"

static void on_error(void)
{
	NRF_LOG_FINAL_FLUSH();

#if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
	// To allow the buffer to be flushed by the host.
	nrf_delay_ms(100);
#endif
#ifdef NRF_DFU_DEBUG_VERSION
	NRF_BREAKPOINT_COND;
#endif
	NVIC_SystemReset();
}


void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
	NRF_LOG_ERROR("%s:%d", p_file_name, line_num);
	on_error();
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
	NRF_LOG_ERROR("Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x", id, pc, info);
	on_error();
}


void app_error_handler_bare(uint32_t error_code)
{
	NRF_LOG_ERROR("Received an error: 0x%08x!", error_code);
	on_error();
}

#define BUTTON_CHECK_NB         128

static void dfu_enter_check_user(void)
{

	nrf_gpio_cfg_sense_input(BUTTON_1,
			NRF_GPIO_PIN_PULLUP,
			NRF_GPIO_PIN_SENSE_LOW);

	nrf_gpio_cfg_sense_input(BUTTON_2,
			NRF_GPIO_PIN_PULLUP,
			NRF_GPIO_PIN_SENSE_LOW);

	uint32_t res = 0;
	for (int i=0 ; i < BUTTON_CHECK_NB; i++) {

		if (nrf_gpio_pin_read(BUTTON_1) == 0 &&
				nrf_gpio_pin_read(BUTTON_2) == 0) {
			res += 1;
			nrf_delay_ms(1);
		}
	}

	if (res > BUTTON_CHECK_NB / 2)
	{
		NRF_LOG_DEBUG("DFU mode requested via button.");
		/// Tried to use the buttonless DFU, which would be cleaner but is broken on SDK 15.3
		nrf_power_gpregret_set(BOOTLOADER_DFU_START);
	}

}

/**
 * @brief Function notifies certain events in DFU process.
 */
static void dfu_observer(nrf_dfu_evt_type_t evt_type)
{

	switch (evt_type)
	{
	case NRF_DFU_EVT_DFU_FAILED:
		NRF_LOG_INFO("NRF_DFU_EVT_DFU_FAILED");
		break;
	case NRF_DFU_EVT_DFU_ABORTED:
		NRF_LOG_INFO("NRF_DFU_EVT_DFU_ABORTED");
		break;
	case NRF_DFU_EVT_DFU_INITIALIZED:
		NRF_LOG_INFO("NRF_DFU_EVT_DFU_INITIALIZED");
		break;
	case NRF_DFU_EVT_TRANSPORT_ACTIVATED:
		NRF_LOG_INFO("NRF_DFU_EVT_TRANSPORT_ACTIVATED");
		break;
	case NRF_DFU_EVT_DFU_STARTED:
		NRF_LOG_INFO("NRF_DFU_EVT_DFU_STARTED");
		break;
	default:
		NRF_LOG_INFO("dfu_observer %d", evt_type);
		break;
	}
}


/**@brief Function for application main entry. */
int main(void)
{
	uint32_t ret_val;

	// Protect MBR and bootloader code from being overwritten.
	ret_val = nrf_bootloader_flash_protect(0, MBR_SIZE);
	APP_ERROR_CHECK(ret_val);
	ret_val = nrf_bootloader_flash_protect(BOOTLOADER_START_ADDR, BOOTLOADER_SIZE);
	APP_ERROR_CHECK(ret_val);

	(void) NRF_LOG_INIT(nrf_bootloader_dfu_timer_counter_get);
	NRF_LOG_DEFAULT_BACKENDS_INIT();

	NRF_LOG_INFO("Inside main");

	dfu_enter_check_user();

	ret_val = nrf_bootloader_init(dfu_observer);
	APP_ERROR_CHECK(ret_val);

	// Either there was no DFU functionality enabled in this project or the DFU module detected
	// no ongoing DFU operation and found a valid main application.
	// Boot the main application.
	nrf_bootloader_app_start();

	// Should never be reached.
	NRF_LOG_INFO("After main");
}

/**
 * @}
 */
