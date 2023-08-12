#ifndef BLE_CUS_H__
#define BLE_CUS_H__


#ifdef __cplusplus
extern "C" {
#endif

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);

void ble_cus_init(void);

/**
 * @brief Used to send data, likely needs work
 * 
 * @param data_array pointer to the data to be sent
 * @param length how long the transmission is
 */
void nus_data_send(uint8_t *data_array, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif // BLE_CUS_H__
/** @} */

