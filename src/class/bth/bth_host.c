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


 //--------------------------------------------------------------------+
 // Host BTH Interface
 //--------------------------------------------------------------------+

typedef struct {
	uint8_t daddr;
	uint8_t bInterfaceNumber;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;

	//uint8_t serial_drid; // Serial Driver ID
	//bth_acm_capability_t acm_capability;
	//uint8_t ep_notif;

	//uint8_t line_state;                               // DTR (bit0), RTS (bit1)
//	TU_ATTR_ALIGNED(4) bth_line_coding_t line_coding; // Baudrate, stop bits, parity, data width

	uint8_t itf_num;
	//tuh_xfer_cb_t user_control_cb;
	uint8_t ep_notif;
	uint8_t acl_in;
	uint8_t acl_out;

	struct {
	 tu_edpt_stream_t ep_notif;
	 tu_edpt_stream_t acl_in;
	 tu_edpt_stream_t acl_out;

//	 uint8_t tx_ff_buf[CFG_TUH_BTH_TX_BUFSIZE];
//	 CFG_TUH_MEM_ALIGN uint8_t tx_ep_buf[CFG_TUH_BTH_TX_EPSIZE];
//
//	 uint8_t rx_ff_buf[CFG_TUH_BTH_TX_BUFSIZE];
//	 CFG_TUH_MEM_ALIGN uint8_t rx_ep_buf[CFG_TUH_BTH_TX_EPSIZE];
	} stream;

} bthh_interface_t;

CFG_TUH_MEM_SECTION
static bthh_interface_t bthh_data[CFG_TUH_BTH];

static bthh_interface_t* make_new_itf(uint8_t daddr, tusb_desc_interface_t const *itf_desc)
{
  for(uint8_t i=0; i<CFG_TUH_CDC; i++)
  {
    if (bthh_data[i].daddr == 0) {
      bthh_interface_t* p_bth = &bthh_data[i];

      p_bth->daddr              = daddr;
      p_bth->bInterfaceNumber   = itf_desc->bInterfaceNumber;
      p_bth->bInterfaceSubClass = itf_desc->bInterfaceSubClass;
      p_bth->bInterfaceProtocol = itf_desc->bInterfaceProtocol;
     // p_bth->line_state         = 0;
      return p_bth;
    }
  }

  return NULL;
}


static inline bthh_interface_t* get_itf(uint8_t idx)
{
	PRINTF("bt get_itf idx=%u\n", idx);
  TU_ASSERT(idx < CFG_TUH_BTH, NULL);
  bthh_interface_t* p_bth = &bthh_data[idx];

  return (p_bth->daddr != 0) ? p_bth : NULL;
}


bool bthh_set_config(uint8_t dev_addr, uint8_t itf_num)
{
	return false;
}

bool bthh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
	return false;
}

void bthh_init(void)
{
	tu_memclr(bthh_data, sizeof(bthh_data));

	for(size_t i=0; i<CFG_TUH_BTH; i++)
	{
	bthh_interface_t* p_bth = &bthh_data[i];

//	tu_edpt_stream_init(&p_bth->stream.acl_out, true, true, false,
//						  p_bth->stream.acl_out_ff_buf, CFG_TUH_TUH_TX_BUFSIZE,
//						  p_bth->stream.acl_out_ep_buf, CFG_TUH_TUH_TX_EPSIZE);
//
//	tu_edpt_stream_init(&p_bth->stream.acl_in, true, false, false,
//						  p_bth->stream.acl_in_ff_buf, CFG_TUH_TUH_RX_BUFSIZE,
//						  p_bth->stream.acl_in_ep_buf, CFG_TUH_TUH_RX_EPSIZE);
//	tu_edpt_stream_init(&p_bth->stream.ep_notif, true, false, false,
//						  p_bth->stream.ep_notif_ff_buf, CFG_TUH_TUH_RX_BUFSIZE,
//						  p_bth->stream.ep_notif_ep_buf, CFG_TUH_TUH_RX_EPSIZE);
	}

}

bool bthh_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
	  (void) rhport;
	  uint8_t const * p_desc_end = ((uint8_t const*) itf_desc) + max_len;

	  bthh_interface_t * p_bth = make_new_itf(dev_addr, itf_desc);
	  TU_VERIFY(p_bth);
	  if (p_bth == NULL)
		  return false;

	  //PRINTF("bthh_open: c=%02X s=%02x\n", itf_desc->bInterfaceClass, itf_desc->bInterfaceSubClass);
	  if ( TUSB_CLASS_WIRELESS_CONTROLLER == itf_desc->bInterfaceClass &&
		  TUH_BT_APP_SUBCLASS == itf_desc->bInterfaceSubClass &&
		  TUH_BT_PROTOCOL_PRIMARY_CONTROLLER == itf_desc->bInterfaceProtocol)
	  {
		  PRINTF("BT RADIO found!\n");

		  tusb_desc_endpoint_t const * desc_ep = (tusb_desc_endpoint_t const *) tu_desc_next(itf_desc);

		  bthh_interface_t* p_bth = get_itf(dev_addr);
		  tusb_desc_endpoint_t const* ep_desc = (tusb_desc_endpoint_t const*) tu_desc_next(itf_desc);

		  for (uint32_t i = 0; i < 3; i++) {
			if (TUSB_DESC_ENDPOINT == ep_desc->bDescriptorType && TUSB_XFER_INTERRUPT == ep_desc->bmAttributes.xfer)
			{
				PRINTF("bt ev: EP %02X\n", ep_desc->bEndpointAddress);
				p_bth->ep_notif = ep_desc->bEndpointAddress;
//				TU_ASSERT(tuh_edpt_open(dev_addr, ep_desc));
			}
			else if (TUSB_DIR_IN == tu_edpt_dir(ep_desc->bEndpointAddress)) {
				PRINTF("bt in: EP %02X\n", ep_desc->bEndpointAddress);
				p_bth->acl_in = ep_desc->bEndpointAddress;
//				TU_ASSERT(tuh_edpt_open(dev_addr, ep_desc));
			} else {
				PRINTF("bt out: EP %02X\n", ep_desc->bEndpointAddress);
				p_bth->acl_out = ep_desc->bEndpointAddress;
//				TU_ASSERT(tuh_edpt_open(dev_addr, ep_desc));
			}

		    ep_desc = (tusb_desc_endpoint_t const*) tu_desc_next(ep_desc);
		  }

		  p_bth->itf_num = itf_desc->bInterfaceNumber;

		  return true;
	  }

	return false;
}

void bthh_close(uint8_t dev_addr)
{
	  for(uint8_t idx=0; idx<CFG_TUH_CDC; idx++)
	  {
	    bthh_interface_t* p_bth = &bthh_data[idx];
	    if (p_bth->daddr == dev_addr)
	    {
	      TU_LOG_DRV("  BTHh close addr = %u index = %u\r\n", dev_addr, idx);

	      // Invoke application callback
	      ////if (tuh_bth_umount_cb) tuh_bth_umount_cb(idx);

	      //tu_memclr(p_bth, sizeof(bthh_interface_t));
	      p_bth->daddr = 0;
	      p_bth->bInterfaceNumber = 0;
	      tu_edpt_stream_close(&p_bth->stream.ep_notif);
	      tu_edpt_stream_close(&p_bth->stream.acl_in);
	      tu_edpt_stream_close(&p_bth->stream.acl_out);
	    }
	  }

}

#endif
