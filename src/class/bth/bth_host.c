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


static enum {
    USBH_OUT_OFF,
    USBH_OUT_IDLE,
    USBH_OUT_CMD,
    USBH_OUT_ACL_SEND,
    USBH_OUT_ACL_POLL,
} usbh_out_state;

static enum {
    USBH_IN_OFF,
    USBH_IN_SUBMIT_REQUEST,
    USBH_IN_POLL,
} usbh_in_state;

// higher-layer callbacks
static void (*usbh_packet_sent)(void);
static void (*usbh_packet_received)(uint8_t packet_type, uint8_t * packet, uint16_t size);

// class state

// outgoing
static const uint8_t * cmd_packet;
static uint16_t        cmd_len;

static const uint8_t * acl_packet;
static uint16_t        acl_len;

// incoming
static uint16_t hci_event_offset;
static uint8_t hci_event[258];

static uint16_t hci_acl_in_offset;
//static uint8_t  hci_acl_in_buffer[HCI_INCOMING_PRE_BUFFER_SIZE + HCI_ACL_BUFFER_SIZE];
//static uint8_t  * hci_acl_in_packet = &hci_acl_in_buffer[HCI_INCOMING_PRE_BUFFER_SIZE];

 //--------------------------------------------------------------------+
 // Host BTH Interface
 //--------------------------------------------------------------------+
#define CFG_TUH_BTH_TX_BUFSIZE 64
#define CFG_TUH_BTH_TX_EPSIZE 64
#define CFG_TUH_BTH_RX_BUFSIZE 64
#define CFG_TUH_BTH_RX_EPSIZE 64

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
	 tu_edpt_stream_t acl_in;
	 tu_edpt_stream_t acl_out;

	 uint8_t acl_out_ff_buf[CFG_TUH_BTH_TX_BUFSIZE];
	 CFG_TUH_MEM_ALIGN uint8_t acl_out_ep_buf[CFG_TUH_BTH_TX_EPSIZE];

	 uint8_t acl_in_ff_buf[CFG_TUH_BTH_TX_BUFSIZE];
	 CFG_TUH_MEM_ALIGN uint8_t acl_in_ep_buf[CFG_TUH_BTH_TX_EPSIZE];
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


uint8_t tuh_bth_itf_get_index(uint8_t daddr, uint8_t itf_num)
{
  for(uint8_t i=0; i<CFG_TUH_CDC; i++)
  {
    const bthh_interface_t* p_bth = &bthh_data[i];

    if (p_bth->daddr == daddr && p_bth->bInterfaceNumber == itf_num) return i;
  }

  return TUSB_INDEX_INVALID_8;
}

bool bthh_set_config(uint8_t dev_addr, uint8_t itf_num)
{
	PRINTF("bthh_set_config: dev_addr=%u,itf_num=%u\n",dev_addr, itf_num);
	  tusb_control_request_t request = { 0 };
	  request.wIndex = tu_htole16((uint16_t) itf_num);

	  // fake transfer to kick-off process
	  tuh_xfer_t xfer;
	  xfer.daddr  = dev_addr;
	  xfer.result = XFER_RESULT_SUCCESS;
	  xfer.setup  = &request;
	  xfer.user_data = 0; // initial state

	  uint8_t const idx = tuh_bth_itf_get_index(dev_addr, itf_num);
	  bthh_interface_t * p_bth = get_itf(idx);
	  //TU_ASSERT(p_bth && p_bth->serial_drid < SERIAL_DRIVER_COUNT);

	  //serial_drivers[p_bth->serial_drid].process_set_config(&xfer);
	  TP();
	  // Prepare for incoming data
	  //tu_edpt_stream_read_xfer(&p_bth->stream.acl_in);
	  TP();

	  // notify usbh that driver enumeration is complete
	  usbh_driver_set_config_complete(p_bth->daddr, itf_num);
	  TP();

	  return true;
}

bool bthh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
	return false;
}

void bthh_init(void)
{
	tu_memclr(bthh_data, sizeof(bthh_data));
	PRINTF("bthh_init:\n");

	for (size_t i=0; i<CFG_TUH_BTH; i++)
	{
		bthh_interface_t* p_bth = &bthh_data[i];

		tu_edpt_stream_init(&p_bth->stream.acl_out, true, true, false,
							  p_bth->stream.acl_out_ff_buf, CFG_TUH_BTH_TX_BUFSIZE,
							  p_bth->stream.acl_out_ep_buf, CFG_TUH_BTH_TX_EPSIZE);

		tu_edpt_stream_init(&p_bth->stream.acl_in, true, false, false,
							  p_bth->stream.acl_in_ff_buf, CFG_TUH_BTH_RX_BUFSIZE,
							  p_bth->stream.acl_in_ep_buf, CFG_TUH_BTH_RX_EPSIZE);
	}

}

bool bthh_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
	  (void) rhport;
	  uint8_t const * p_desc_end = ((uint8_t const*) itf_desc) + max_len;


	  if ( TUSB_CLASS_WIRELESS_CONTROLLER == itf_desc->bInterfaceClass &&
		  TUH_BT_APP_SUBCLASS == itf_desc->bInterfaceSubClass &&
		  TUH_BT_PROTOCOL_PRIMARY_CONTROLLER == itf_desc->bInterfaceProtocol &&
		  0 == itf_desc->bInterfaceNumber &&
		  3 == itf_desc->bNumEndpoints
		  )
	  {
		  PRINTF("BT RADIO found!\n");

		  bthh_interface_t * p_bth = make_new_itf(dev_addr, itf_desc);
		  TU_VERIFY(p_bth);
		  if (p_bth == NULL)
			  return false;

		  tusb_desc_endpoint_t const* ep_desc = (tusb_desc_endpoint_t const *) tu_desc_next(itf_desc);

		  for (uint32_t i = 0; i < 3; i++) {
			if (TUSB_DESC_ENDPOINT == ep_desc->bDescriptorType && TUSB_XFER_INTERRUPT == ep_desc->bmAttributes.xfer)
			{
				p_bth->ep_notif = ep_desc->bEndpointAddress;
				TU_ASSERT(tuh_edpt_open(dev_addr, ep_desc));
			}
			else if (TUSB_DIR_IN == tu_edpt_dir(ep_desc->bEndpointAddress)) {
				p_bth->acl_in = ep_desc->bEndpointAddress;
				TU_ASSERT(tuh_edpt_open(dev_addr, ep_desc));
			} else {
				p_bth->acl_out = ep_desc->bEndpointAddress;
				TU_ASSERT(tuh_edpt_open(dev_addr, ep_desc));
			}

		    ep_desc = (tusb_desc_endpoint_t const*) tu_desc_next(ep_desc);
		  }

		  p_bth->itf_num = itf_desc->bInterfaceNumber;


		  //++++++++++++++


		    // Command
		    usbh_out_state = USBH_OUT_OFF;

		    // Event In
		    usbh_in_state = USBH_IN_OFF;

		    // ACL In
		    hci_acl_in_offset = 0;

		    // ACL Out
		  //--------------

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
	      tu_edpt_stream_close(&p_bth->stream.acl_in);
	      tu_edpt_stream_close(&p_bth->stream.acl_out);
	    }
	  }

}

#endif
