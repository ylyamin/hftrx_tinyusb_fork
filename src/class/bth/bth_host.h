/*
 * bth_host.h
 *
 *  Created on: Nov 19, 2023
 *      Author: Gena
 */

#ifndef LIB_TINYUSB_SRC_CLASS_BTH_BTH_HOST_H_
#define LIB_TINYUSB_SRC_CLASS_BTH_BTH_HOST_H_


#ifdef __cplusplus
 extern "C" {
#endif


#define TUH_BT_APP_CLASS                    (TUSB_CLASS_WIRELESS_CONTROLLER)
#define TUH_BT_APP_SUBCLASS                 0x01
#define TUH_BT_PROTOCOL_PRIMARY_CONTROLLER  0x01
#define TUH_BT_PROTOCOL_AMP_CONTROLLER      0x02

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
void bthh_init       (void);
bool bthh_open       (uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *itf_desc, uint16_t max_len);
bool bthh_set_config (uint8_t dev_addr, uint8_t itf_num);
bool bthh_xfer_cb    (uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);
void bthh_close      (uint8_t dev_addr);


#ifdef __cplusplus
 }
#endif

#endif /* LIB_TINYUSB_SRC_CLASS_BTH_BTH_HOST_H_ */
