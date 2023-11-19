//

#include "tusb_option.h"

#if (CFG_TUH_ENABLED && CFG_TUH_BTH)

#include "host/usbh.h"
#include "host/usbh_pvt.h"

#include "bth_host.h"

// Level where CFG_TUSB_DEBUG must be at least for this driver is logged
#ifndef CFG_TUH_BTH_LOG_LEVEL
  #define CFG_TUH_BTH_LOG_LEVEL   CFG_TUH_LOG_LEVEL
#endif

#define TU_LOG_DRV(...)   TU_LOG(CFG_TUH_BTH_LOG_LEVEL, __VA_ARGS__)


void bthh_init(void)
{

}

bool bthh_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
	  (void) rhport;

	  //PRINTF("bthh_open: c=%02X s=%02x\n", itf_desc->bInterfaceClass, itf_desc->bInterfaceSubClass);
	  if ( 0xE0 == itf_desc->bInterfaceClass &&
		   0x01 == itf_desc->bInterfaceSubClass &&
	       0x01 == itf_desc->bInterfaceProtocol)
	  {
		  PRINTF("BT RADIO found!\n");
	    return false;//acm_open(daddr, itf_desc, max_len);
	  }
	return false;
}

bool bthh_set_config(uint8_t dev_addr, uint8_t itf_num)
{
	return false;
}

bool bthh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
	return false;
}

void bthh_close(uint8_t dev_addr)
{

}

#endif
