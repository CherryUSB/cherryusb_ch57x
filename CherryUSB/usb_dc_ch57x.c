/**
 * @brief   Ch573 endpoint address description
 * ep0:     in: 0x80  out:0x00
 * ep1:     in: 0x81  out:0x01
 * ep2:     in: 0x82  out:0x02
 * ep3:     in: 0x83  out:0x03
 * ep4:     in: 0x84  out:0x04
 */

#include "usb_dc.h"
#include "usbd_core.h"
#include "./usb_ch57x_reg.h"
#include <stdlib.h>
#include "core_riscv.h"

/**
 * @brief   Related register macro
 */

/*!< ep dir in */
#define EP_DIR_IN 1
/*!< ep dir out */
#define EP_DIR_OUT 0

/*!< 8-bit value of endpoint control register */
#define EPn_CTRL(epid) \
  *(volatile uint8_t *)(0x40008022 + epid * 4 + (epid / 5) * 48)

/*!< The length register value of the endpoint send buffer */
#define EPn_TX_LEN(epid) \
  *(volatile uint8_t *)(0x40008020 + epid * 4 + (epid / 5) * 48)

/*!< get ep id by epadd */
#define GET_EP_ID(ep_add) (uint8_t)(ep_add & 0x7f)
/*!< get ep dir by epadd */
#define GET_EP_DIR(ep_add) (uint8_t)(ep_add & 0x80)
/*!< get interrupt endpoint id */
#define GET_INT_EP_ID \
  (volatile uint8_t)(R8_USB_INT_ST & MASK_UIS_ENDP)
/*!< get usb interrupt state reg */
#define GET_USB_INT_STATE \
  (volatile uint8_t)(R8_USB_INT_ST & MASK_UIS_TOKEN)
/*!< read setup packet to use in ep0 in */
#define GET_SETUP_PACKET(data_add) \
  *(struct usb_setup_packet *)data_add
/*!< set device address // call in set_add state stage */
#define SET_DEVICE_ADDRESS(add) \
  R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | add;
/*!< set epid ep tx valid */
#define EPn_SET_TX_VALID(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_T_RES | UEP_T_RES_ACK;
/*!< set epid ep rx valid */
#define EPn_SET_RX_VALID(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_R_RES | UEP_R_RES_ACK;
/*!< set epid ep tx nak */
#define EPn_SET_TX_NAK(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_T_RES | UEP_T_RES_NAK;
/*!< set epid ep rx nak */
#define EPn_SET_RX_NAK(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_R_RES | UEP_R_RES_NAK;
/*!< set epid ep tx stall */
#define EPn_SET_TX_STALL(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_T_RES | UEP_T_RES_STALL
/*!< set epid ep rx stall */
#define EPn_SET_RX_STALL(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_R_RES | UEP_R_RES_STALL
/*!< set epid ep tx len */
#define EPn_SET_TX_LEN(epid, len) \
  EPn_TX_LEN(epid) = len
/*!< get epid ep rx len */
#define EPn_GET_RX_LEN(epid) \
  R8_USB_RX_LEN

/*!< ep nums */
#define EP_NUMS 5
/*!< ep mps */
#define EP_MPS 64
/*!< set ep4 in mps 64 */
#define EP4_IN_MPS EP_MPS
/*!< set ep4 out mps 64 */
#define EP4_OUT_MPS EP_MPS

/*!< User defined assignment endpoint RAM */
__attribute__((aligned(4))) uint8_t ep0_data_buff[64 + 64 + 64]; /*!< ep0(64)+ep4_out(64)+ep4_in(64) */
__attribute__((aligned(4))) uint8_t ep1_data_buff[64 + 64];      /*!< ep1_out(64)+ep1_in(64) */
__attribute__((aligned(4))) uint8_t ep2_data_buff[64 + 64];      /*!< ep2_out(64)+ep2_in(64) */
__attribute__((aligned(4))) uint8_t ep3_data_buff[64 + 64];      /*!< ep3_out(64)+ep3_in(64) */
__attribute__((aligned(4))) uint8_t ep4_data_buff[64 + 64];      /*!< ep3_out(64)+ep3_in(64) */


uint8_t *EP0_RAM_Addr;
uint8_t *EP1_RAM_Addr;
uint8_t *EP2_RAM_Addr;
uint8_t *EP3_RAM_Addr;
uint8_t *EP4_RAM_Addr;


#define pEP0_DataBuf (EP0_RAM_Addr)
#define pEP1_OUT_DataBuf (EP1_RAM_Addr)
#define pEP1_IN_DataBuf (EP1_RAM_Addr + 64)
#define pEP2_OUT_DataBuf (EP2_RAM_Addr)
#define pEP2_IN_DataBuf (EP2_RAM_Addr + 64)
#define pEP3_OUT_DataBuf (EP3_RAM_Addr)
#define pEP3_IN_DataBuf (EP3_RAM_Addr + 64)
#define pEP4_OUT_DataBuf (EP3_RAM_Addr)
#define pEP4_IN_DataBuf (EP3_RAM_Addr + 64)


/**
 * @brief   Endpoint information structure
 */
typedef struct _usbd_ep_info
{
  uint8_t mps;          /*!< Maximum packet length of endpoint */
  uint8_t eptype;       /*!< Endpoint Type */
  uint8_t *ep_ram_addr; /*!< Endpoint buffer address */
} usbd_ep_info;

/*!< ch582 usb */
static struct _ch573_core_prvi
{
  uint8_t address; /*!< Address */
  usbd_ep_info ep_in[EP_NUMS];
  usbd_ep_info ep_out[EP_NUMS];
  struct usb_setup_packet setup;
} usb_dc_cfg;

/**
 * @brief            Set address
 * @pre              None
 * @param[in]        address 锛�8-bit valid address
 * @retval           >=0 success otherwise failure
 */
int usbd_set_address(const uint8_t address)
{
  if (address == 0)
  {
    R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | address;
  }
  usb_dc_cfg.address = address;
  return 1;
}

/**
 * @brief            Open endpoint
 * @pre              None
 * @param[in]        ep_cfg : Endpoint configuration structure pointer
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_open(const struct usbd_endpoint_cfg *ep_cfg)
{
  /*!< ep id */
  uint8_t epid = GET_EP_ID(ep_cfg->ep_addr);
  /*!< ep dir */
  bool dir = GET_EP_DIR(ep_cfg->ep_addr);
  /*!< ep max packet length */
  uint8_t mps = ep_cfg->ep_mps;
  /*!< update ep max packet length */
  if (dir == EP_DIR_IN)
  {
    /*!< in */
    usb_dc_cfg.ep_in[epid].mps = mps;
  }
  else if (dir == EP_DIR_OUT)
  {
    /*!< out */
    usb_dc_cfg.ep_out[epid].mps = mps;
  }
  return 0;
}

/**
 * @brief            Close endpoint
 * @pre              None
 * @param[in]        ep 锛� Endpoint address
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_close(const uint8_t ep)
{
  return 0;
}

/**
 * @brief            Write send buffer
 * @pre              None
 * @param[in]        ep 锛� Endpoint address
 * @param[in]        data 锛� First address of data buffer to be written
 * @param[in]        data_len 锛� Write total length
 * @param[in]        ret_bytes 锛� Length actually written
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_write(const uint8_t ep, const uint8_t *data, uint32_t data_len, uint32_t *ret_bytes)
{
  /*!< ep id */
  uint8_t epid = GET_EP_ID(ep);
  /*!< real write byte nums */
  uint32_t real_wt_nums = 0;
  /*!< ep mps */
  uint8_t ep_mps = usb_dc_cfg.ep_in[epid].mps;
  /*!< Analyze bytes actually written */
  if (data == NULL && data_len > 0)
  {
    return -1;
  }

  if (data_len == 0)
  {
    /*!< write 0 len data */
    EPn_SET_TX_LEN(epid, 0);
    /*!< enable tx */
    EPn_SET_TX_VALID(epid);
    /*!< return */
    return 0;
  }

  if (data_len > ep_mps)
  {
    /*!< If the data length is greater than the maximum packet length of the endpoint,
         the actual written data length is limited to the maximum packet length of the endpoint */
    real_wt_nums = ep_mps;
  }
  else
  {
    real_wt_nums = data_len;
  }

  /*!< write buff */
  memcpy(usb_dc_cfg.ep_in[epid].ep_ram_addr, data, real_wt_nums);
  /*!< write real_wt_nums len data */
  EPn_SET_TX_LEN(epid, real_wt_nums);
  /*!< enable tx */
  EPn_SET_TX_VALID(epid);

  if (ret_bytes != NULL)
  {
    *ret_bytes = real_wt_nums;
  }

  return 0;
}

/**
 * @brief            Read receive buffer
 * @pre              None
 * @param[in]        ep 锛� Endpoint address
 * @param[in]        data 锛� Read the first address of the buffer where the data is stored
 * @param[in]        max_data_len 锛� Maximum readout length
 * @param[in]        read_bytes 锛� Actual read length
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_read(const uint8_t ep, uint8_t *data, uint32_t max_data_len, uint32_t *read_bytes)
{
  /*!< ep id */
  uint8_t epid = GET_EP_ID(ep);
  /*!< real read byte nums */
  uint32_t real_rd_nums = 0;
  /*!< ep mps */
  uint8_t ep_mps = usb_dc_cfg.ep_out[epid].mps;
  if (data == NULL && max_data_len > 0)
  {
    return -1;
  }

  if (max_data_len == 0)
  {
    /*!< Enable reception */
    /*!< Non endpoint 0 can directly enable reception here,
         and endpoint 0 can enable reception after the end of the interrupt */
    if (epid != 0)
      EPn_SET_RX_VALID(epid);
    return 0;
  }

  if (max_data_len > ep_mps)
  {
    /*!< If the maximum length of the expected readout is greater than the maximum packet length of the endpoint,
         the expected maximum readout length is limited to the maximum packet length */
    max_data_len = ep_mps;
  }

  /*!< Special treatment for ch582 //start */
  if (epid)
  {
    real_rd_nums = EPn_GET_RX_LEN(epid);
    real_rd_nums = MIN(real_rd_nums, max_data_len);
  }
  else
  {
    /*!< ep0 */
    /*!< For ch582, when reading the length of USB received data during idle period, this data is uncertain */
    real_rd_nums = max_data_len;
  }
  /*!< Special treatment for ch582 //end */

  /*!< read buff */
  memcpy(data, usb_dc_cfg.ep_out[epid].ep_ram_addr, real_rd_nums);
  if (read_bytes != NULL)
  {
    *read_bytes = real_rd_nums;
  }

  return 0;
}

/**
 * @brief            Endpoint setting stall
 * @pre              None
 * @param[in]        ep 锛� Endpoint address
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_set_stall(const uint8_t ep)
{
  /*!< ep id */
  uint8_t epid = GET_EP_ID(ep);
  EPn_SET_RX_STALL(epid);
  EPn_SET_TX_STALL(epid);
  return 0;
}

/**
 * @brief            Endpoint clear stall
 * @pre              None
 * @param[in]        ep 锛� Endpoint address
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_clear_stall(const uint8_t ep)
{
  int ret;
  switch (ep)
  {
  case 0x82:
    R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
    ret = 0;
    break;
  case 0x02:
    R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_ACK;
    ret = 0;
    break;
  case 0x81:
    R8_UEP1_CTRL = (R8_UEP1_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
    ret = 0;
    break;
  case 0x01:
    R8_UEP1_CTRL = (R8_UEP1_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_ACK;
    ret = 0;
    break;
  default:
    /*!< Unsupported endpoint */
    ret = -1;
    break;
  }
  return ret;
}

/**
 * @brief            Check endpoint status
 * @pre              None
 * @param[in]        ep 锛� Endpoint address
 * @param[out]       stalled 锛� Outgoing endpoint status
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_is_stalled(const uint8_t ep, uint8_t *stalled)
{
  return 0;
}

/**
 * @brief            USB initialization
 * @pre              None
 * @param[in]        None
 * @retval           >=0 success otherwise failure
 */
int usb_dc_init(void)
{
  EP0_RAM_Addr = ep0_data_buff;
  EP1_RAM_Addr = ep1_data_buff;
  EP2_RAM_Addr = ep2_data_buff;
  EP3_RAM_Addr = ep3_data_buff;

  usb_dc_cfg.ep_in[0].ep_ram_addr = pEP0_DataBuf;
  usb_dc_cfg.ep_out[0].ep_ram_addr = pEP0_DataBuf;

  usb_dc_cfg.ep_in[1].ep_ram_addr = pEP1_IN_DataBuf;
  usb_dc_cfg.ep_out[1].ep_ram_addr = pEP1_OUT_DataBuf;

  usb_dc_cfg.ep_in[2].ep_ram_addr = pEP2_IN_DataBuf;
  usb_dc_cfg.ep_out[2].ep_ram_addr = pEP2_OUT_DataBuf;

  usb_dc_cfg.ep_in[3].ep_ram_addr = pEP3_IN_DataBuf;
  usb_dc_cfg.ep_out[3].ep_ram_addr = pEP3_OUT_DataBuf;

  usb_dc_cfg.ep_in[4].ep_ram_addr = pEP3_IN_DataBuf;
  usb_dc_cfg.ep_out[4].ep_ram_addr = pEP3_OUT_DataBuf;

  R8_USB_CTRL = 0x00; // 先设定模式,取消 RB_UC_CLR_ALL

  R8_UEP4_1_MOD = RB_UEP4_RX_EN | RB_UEP4_TX_EN | RB_UEP1_RX_EN | RB_UEP1_TX_EN; // 端点4 OUT+IN,端点1 OUT+IN
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN; // 端点2 OUT+IN,端点3 OUT+IN

  R16_UEP0_DMA = (uint16_t)(uint32_t)EP0_RAM_Addr;
  R16_UEP1_DMA = (uint16_t)(uint32_t)EP1_RAM_Addr;
  R16_UEP2_DMA = (uint16_t)(uint32_t)EP2_RAM_Addr;
  R16_UEP3_DMA = (uint16_t)(uint32_t)EP3_RAM_Addr;

  R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
  R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
  R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
  R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
  R8_UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  R8_USB_DEV_AD = 0x00;
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN; // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
  R16_PIN_ANALOG_IE |= RB_PIN_USB_IE | RB_PIN_USB_DP_PU;         // 防止USB端口浮空及上拉电阻
  R8_USB_INT_FG = 0xFF;                                          // 清中断标志
  R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;                   // 允许USB端口
  R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;
  //开启中断
  PFIC->IENR[((uint32_t)(USB_IRQn) >> 5)] = (1 << ((uint32_t)(USB_IRQn)&0x1F));
  return 0;
}

/**
 * @brief            USB interrupt processing function
 * @pre              None
 * @param[in]        None
 * @retval           None
 */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void USB_IRQHandler(void)
{
  UINT8 intflag = 0;
  intflag = R8_USB_INT_FG;

  if (intflag & RB_UIF_TRANSFER)
  {
    if ((R8_USB_INT_ST & MASK_UIS_TOKEN) != MASK_UIS_TOKEN) /*!< Non idle */
    {
      switch (R8_USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
      {
      case UIS_TOKEN_IN:
        /*!< PRINT("in \n"); */
        switch (usb_dc_cfg.setup.bmRequestType >> USB_REQUEST_DIR_SHIFT)
        {
        case 1:
          /*!< get */
          R8_UEP0_CTRL ^= RB_UEP_T_TOG;
          usbd_event_notify_handler(USBD_EVENT_EP0_IN_NOTIFY, NULL);
          break;
        case 0:
          /*!< set */
          switch (usb_dc_cfg.setup.bRequest)
          {
          case USB_SET_ADDRESS:
            /*!< Fill in the equipment address */
            R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | usb_dc_cfg.address;
            /*!< No data returned T-NACK */
            R8_UEP0_T_LEN = 0;
            R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
            break;
          default:
            /*!< PRINT("state over \n"); */
            /*!< Normal out state phase */
            R8_UEP0_T_LEN = 0; /*!< The status phase is interrupted or the forced upload of 0-length packet ends the control transmission */
            R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
            break;
          }
          break;
        }
        break;
      case UIS_TOKEN_OUT | 0:
        /*!< ep0 out */
        R8_UEP0_CTRL ^= RB_UEP_R_TOG;
        usbd_event_notify_handler(USBD_EVENT_EP0_OUT_NOTIFY, NULL);
        EPn_SET_RX_VALID(0);
        break;
      case UIS_TOKEN_OUT | 1:
        usbd_event_notify_handler(USBD_EVENT_EP_OUT_NOTIFY, (uint32_t *)(0x01 & 0x7f));
        break;
      case UIS_TOKEN_IN | 1:
        R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        usbd_event_notify_handler(USBD_EVENT_EP_IN_NOTIFY, (uint32_t *)(0x01 | 0x80));
        break;
      case UIS_TOKEN_OUT | 2:
        if (R8_USB_INT_ST & RB_UIS_TOG_OK)
        {
          /*!< Out of sync packets will be discarded */
          usbd_event_notify_handler(USBD_EVENT_EP_OUT_NOTIFY, (uint32_t *)(0x02 & 0x7f));
        }
        break;
      case UIS_TOKEN_IN | 2:
        R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        usbd_event_notify_handler(USBD_EVENT_EP_IN_NOTIFY, (uint32_t *)(0x02 | 0x80));
        break;
      case UIS_TOKEN_OUT | 3:
        if (R8_USB_INT_ST & RB_UIS_TOG_OK)
        {
          /*!< Out of sync packets will be discarded */
          usbd_event_notify_handler(USBD_EVENT_EP_OUT_NOTIFY, (uint32_t *)(0x03 & 0x7f));
        }
        break;
      case UIS_TOKEN_IN | 3:
        R8_UEP3_CTRL = (R8_UEP3_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        usbd_event_notify_handler(USBD_EVENT_EP_IN_NOTIFY, (uint32_t *)(0x03 | 0x80));
        break;
      case UIS_TOKEN_OUT | 4:
        if (R8_USB_INT_ST & RB_UIS_TOG_OK)
        {
          R8_UEP4_CTRL ^= RB_UEP_R_TOG;
          usbd_event_notify_handler(USBD_EVENT_EP_OUT_NOTIFY, (uint32_t *)(0x04 & 0x7f));
        }
        break;
      case UIS_TOKEN_IN | 4:
        R8_UEP4_CTRL ^= RB_UEP_T_TOG;
        R8_UEP4_CTRL = (R8_UEP4_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        usbd_event_notify_handler(USBD_EVENT_EP_IN_NOTIFY, (uint32_t *)(0x04 | 0x80));
        break;
      default:
        break;
      }
      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }

    if (R8_USB_INT_ST & RB_UIS_SETUP_ACT)
    {
      /*!< PRINT("setup \n"); */
      R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
      /*!< get setup packet */
      usb_dc_cfg.setup = GET_SETUP_PACKET(usb_dc_cfg.ep_out[0].ep_ram_addr);
      usbd_event_notify_handler(USBD_EVENT_SETUP_NOTIFY, NULL);
      /*!< enable ep0 rx */
      EPn_SET_RX_VALID(0);

      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
  }
  else if (intflag & RB_UIF_BUS_RST)
  {
    R8_USB_DEV_AD = 0;
    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
	R8_UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;

    R8_USB_INT_FG = RB_UIF_BUS_RST;

    /*!< Call the reset callback in the protocol stack to register the endpoint callback function */
    usbd_event_notify_handler(USBD_EVENT_RESET, NULL);
  }
  else if (intflag & RB_UIF_SUSPEND)
  {
    if (R8_USB_MIS_ST & RB_UMS_SUSPEND)
    {
      /*!< Suspend */
      usbd_event_notify_handler(USBD_EVENT_SUSPEND, NULL);
    }
    else
    {
      /*!< Wake up */
      usbd_event_notify_handler(USBD_EVENT_RESUME, NULL);
      ;
    }

    R8_USB_INT_FG = RB_UIF_SUSPEND;
  }
  else
  {
    R8_USB_INT_FG = intflag;
  }
}
