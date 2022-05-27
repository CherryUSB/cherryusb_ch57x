/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2022/01/25
 * Description        : 模拟兼容HID设备
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "CH57x_common.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

void usb_dc_low_level_init(void)
{
    R8_USB_CTRL = 0x00; // 先设定模式,取消 RB_UC_CLR_ALL

    R8_UEP4_1_MOD = RB_UEP4_RX_EN | RB_UEP4_TX_EN | RB_UEP1_RX_EN | RB_UEP1_TX_EN; // 端点4 OUT+IN,端点1 OUT+IN
    R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN; // 端点2 OUT+IN,端点3 OUT+IN

    //R16_UEP0_DMA = (uint16_t)(uint32_t)pEP0_RAM_Addr;
    //R16_UEP1_DMA = (uint16_t)(uint32_t)pEP1_RAM_Addr;
    //R16_UEP2_DMA = (uint16_t)(uint32_t)pEP2_RAM_Addr;
    //R16_UEP3_DMA = (uint16_t)(uint32_t)pEP3_RAM_Addr;

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

    Delay_Us(100);
    PFIC_EnableIRQ(USB_IRQn);    //启用中断向量

}
/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   调试初始化
 *
 * @return  none
 */
void DebugInit(void)
{
    GPIOA_SetBits(GPIO_Pin_9);
    GPIOA_ModeCfg(GPIO_Pin_8, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();
}

void LedInit(void)
{
    GPIOA_SetBits(GPIO_Pin_12);
    GPIOA_SetBits(GPIO_Pin_13);
    GPIOA_ModeCfg(GPIO_Pin_12, GPIO_ModeOut_PP_5mA);
    GPIOA_ModeCfg(GPIO_Pin_13, GPIO_ModeOut_PP_5mA);
}
/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main()
{
    uint8_t s;
    PWR_DCDCCfg(DISABLE);
    SetSysClock(CLK_SOURCE_PLL_60MHz);

    DebugInit();        //配置串口1用来prinft来debug
    LedInit();
    printf("start\n");
    Delay_Ms(100);
    extern void cdc_acm_init(void);
    cdc_acm_init();
    while (!usb_device_is_configured()) {
    }
    while (1) {
        extern void cdc_acm_data_send_with_dtr_test();
        cdc_acm_data_send_with_dtr_test();
        GPIOA_SetBits(GPIO_Pin_12);
        GPIOA_ResetBits(GPIO_Pin_13);
        Delay_Ms(250);
        GPIOA_SetBits(GPIO_Pin_13);
        GPIOA_ResetBits(GPIO_Pin_12);
        Delay_Ms(250);
    }
}
