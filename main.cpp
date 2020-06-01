/*-
 * $Copyright$
-*/
#include <version.h>

#include <stm32f4/PwrViaSTM32F4.hpp>
#include <stm32f4/FlashViaSTM32F4.hpp>
#include <stm32f4/RccViaSTM32F4.hpp>
#include <stm32f4/ScbViaSTM32F4.hpp>
#include <stm32f4/NvicViaSTM32F4.hpp>

#include <gpio/GpioAccess.hpp>

#include <gpio/GpioEngine.hpp>
#include <gpio/GpioPin.hpp>

#include <uart/UartAccess.hpp>
#include <uart/UartDevice.hpp>

/* For USB Device, includes dependent header files. */
#include <usb/UsbCoreViaSTM32F4.hpp>
#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <usb/OutEndpointViaSTM32F4.hpp>

#include <tasks/Heartbeat.hpp>
#include <tasks/DebounceButton.hpp>
#include <tasks/ButtonHandler.hpp>

/*******************************************************************************
 *
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

extern char stext, etext;
extern char sdata, edata;
extern char sbss, ebss;
extern char bstack, estack;

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

/*******************************************************************************
 * System Devices
 ******************************************************************************/
static devices::RccViaSTM32F4::PllConfiguration pllCfg(336, 8, devices::RccViaSTM32F4::e_PllP_Div2, 7,
                                                  devices::RccViaSTM32F4::e_APBPrescaler_Div4,
                                                  devices::RccViaSTM32F4::e_APBPrescaler_Div2,
                                                  devices::RccViaSTM32F4::e_AHBPrescaler_None,
                                                  devices::RccViaSTM32F4::e_PllSourceHSE,
                                                  devices::RccViaSTM32F4::e_SysclkPLL,
                                                  16000000,
                                                   8000000);

static devices::PwrViaSTM32F4           pwr(PWR);
static devices::FlashViaSTM32F4         flash(FLASH);
static devices::RccViaSTM32F4           rcc(RCC, pllCfg, flash, pwr);
static devices::ScbViaSTM32F4           scb(SCB);
static devices::NvicViaSTM32F4          nvic(NVIC, scb);

/*******************************************************************************
 * GPIO Engine Handlers 
 ******************************************************************************/
static gpio::GpioAccessViaSTM32F4_GpioA gpio_A(rcc);
static gpio::GpioEngine                 gpio_engine_A(&gpio_A);

static gpio::GpioAccessViaSTM32F4_GpioB gpio_B(rcc);
static gpio::GpioEngine                 gpio_engine_B(&gpio_B);

static gpio::GpioAccessViaSTM32F4_GpioC gpio_C(rcc);
static gpio::GpioEngine                 gpio_engine_C(&gpio_C);

static gpio::GpioAccessViaSTM32F4_GpioD gpio_D(rcc);
static gpio::GpioEngine                 gpio_engine_D(&gpio_D);

/*******************************************************************************
 * LEDs
 ******************************************************************************/
static gpio::PinT<decltype(gpio_engine_D)>  g_led_green(&gpio_engine_D, 12);
static gpio::PinT<decltype(gpio_engine_D)>  g_led_orange(&gpio_engine_D, 13);
static gpio::PinT<decltype(gpio_engine_D)>  g_led_red(&gpio_engine_D, 14);
static gpio::PinT<decltype(gpio_engine_D)>  g_led_blue(&gpio_engine_D, 15);

/*******************************************************************************
 * UART
 ******************************************************************************/
static gpio::PinT<decltype(gpio_engine_C)>  uart_tx(&gpio_engine_C, 6);
static gpio::PinT<decltype(gpio_engine_C)>  uart_rx(&gpio_engine_C, 7);
static uart::UartAccessSTM32F4_Uart6        uart_access(rcc, uart_rx, uart_tx);
uart::UartDevice                            g_uart(&uart_access);

/*******************************************************************************
 * USB Device
 ******************************************************************************/
/* GPIO Pins for USB Connection */
static gpio::PinT<decltype(gpio_engine_A)>      usb_pin_dm(&gpio_engine_A, 11);
static gpio::PinT<decltype(gpio_engine_A)>      usb_pin_dp(&gpio_engine_A, 12);
static gpio::PinT<decltype(gpio_engine_A)>      usb_pin_vbus(&gpio_engine_A, 9);
static gpio::PinT<decltype(gpio_engine_A)>      usb_pin_id(&gpio_engine_A, 10);

/* USB Descriptors (defined in UsbDescriptors.cpp) */
extern const ::usb::UsbDeviceDescriptor_t usbDeviceDescriptor;
extern const ::usb::UsbStringDescriptors_t usbStringDescriptors;
extern const ::usb::UsbConfigurationDescriptor_t usbConfigurationDescriptor;

/* USB Stack */
usb::stm32f4::UsbFullSpeedCore                  usbCore(nvic, rcc, usb_pin_dm, usb_pin_dp, usb_pin_vbus, usb_pin_id, /* p_rxFifoSzInWords = */ 256);
static usb::stm32f4::UsbDeviceViaSTM32F4        usbHwDevice(usbCore);
static usb::stm32f4::CtrlInEndpointViaSTM32F4   defaultHwCtrlInEndpoint(usbHwDevice, /* p_fifoSzInWords = */ 0x20);

static usb::stm32f4::IrqInEndpointViaSTM32F4    irqInHwEndp(usbHwDevice, /* p_fifoSzInWords = */ 128, 1);
static usb::UsbIrqInEndpoint                    irqInEndpoint(irqInHwEndp);

static usb::UsbHidInterface                     usbInterface(irqInEndpoint);

static usb::UsbConfiguration                    usbConfiguration(usbInterface, usbConfigurationDescriptor);

static usb::UsbDevice                           genericUsbDevice(usbHwDevice, usbDeviceDescriptor, usbStringDescriptors, { &usbConfiguration });

static usb::UsbCtrlInEndpoint                   ctrlInEndp(defaultHwCtrlInEndpoint);
static usb::UsbControlPipe                      defaultCtrlPipe(genericUsbDevice, ctrlInEndp);

static usb::UsbCtrlOutEndpointT<usb::stm32f4::CtrlOutEndpointViaSTM32F4>
                                                ctrlOutEndp(defaultCtrlPipe);                                               
static usb::stm32f4::CtrlOutEndpointViaSTM32F4  defaultCtrlOutEndpoint(usbHwDevice, ctrlOutEndp);

/*******************************************************************************
 * Button(s)
 ******************************************************************************/
static gpio::PinT<decltype(gpio_engine_A)>  g_btn_blue(&gpio_engine_A, 0);

/*******************************************************************************
 * Tasks
 ******************************************************************************/
static tasks::DebounceButtonT<decltype(g_btn_blue)>                     btn_debounce("btn_debc", /* p_priority */ 1, /* p_periodMs */ 1, g_btn_blue);
static tasks::HeartbeatT<decltype(g_uart), decltype(g_led_green)>       heartbeat_gn("hrtbt_gn", g_uart, g_led_green, /* p_priority */ 2, /* p_periodMs */ 500);
static tasks::ButtonHandler                                             btn_handler("btn_hndl", /* p_priority */ 3,  irqInEndpoint, g_led_blue);

/*******************************************************************************
 * Queues for Task Communication
 ******************************************************************************/
static QueueHandle_t    buttonHandlerQueue;

/*******************************************************************************
 *
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#if defined(HOSTBUILD)
int
#else
void
#endif /* defined(HOSTBUILD) */
main(void) {
    g_led_green.enable(gpio::GpioAccessViaSTM32F4::e_Output, gpio::GpioAccessViaSTM32F4::e_None, gpio::GpioAccessViaSTM32F4::e_Gpio);
    g_led_orange.enable(gpio::GpioAccessViaSTM32F4::e_Output, gpio::GpioAccessViaSTM32F4::e_None, gpio::GpioAccessViaSTM32F4::e_Gpio);
    g_led_red.enable(gpio::GpioAccessViaSTM32F4::e_Output, gpio::GpioAccessViaSTM32F4::e_None, gpio::GpioAccessViaSTM32F4::e_Gpio);
    g_led_blue.enable(gpio::GpioAccessViaSTM32F4::e_Output, gpio::GpioAccessViaSTM32F4::e_None, gpio::GpioAccessViaSTM32F4::e_Gpio);

    g_btn_blue.enable(gpio::GpioAccessViaSTM32F4::e_Input, gpio::GpioAccessViaSTM32F4::e_None, gpio::GpioAccessViaSTM32F4::e_Gpio);

    g_uart.printf("Copyright (c) 2013-2020 Philip Schulz <phs@phisch.org>\r\n");
    g_uart.printf("All rights reserved.\r\n");
    g_uart.printf("\r\n");
    g_uart.printf("SW Version: %s\r\n", gSwVersionId);
    g_uart.printf("SW Build Timestamp: %s\r\n", gSwBuildTime);
    g_uart.printf("\r\n");
    g_uart.printf("Fixed Data: [0x0%x - 0x0%x]\t(%d Bytes total, %d Bytes used)\r\n",
      &gFixedDataBegin, &gFixedDataEnd, &gFixedDataEnd - &gFixedDataBegin, &gFixedDataUsed- &gFixedDataBegin);
    g_uart.printf("      Code: [0x0%x - 0x0%x]\t(%d Bytes)\r\n", &stext, &etext, &etext - &stext);
    g_uart.printf("      Data: [0x%x - 0x%x]\t(%d Bytes)\r\n", &sdata, &edata, &edata - &sdata);
    g_uart.printf("       BSS: [0x%x - 0x%x]\t(%d Bytes)\r\n", &sbss, &ebss, &ebss - &sbss);
    g_uart.printf(" Total RAM: [0x%x - 0x%x]\t(%d Bytes)\r\n", &sdata, &ebss, &ebss - &sdata);
    g_uart.printf("     Stack: [0x%x - 0x%x]\t(%d Bytes)\r\n", &bstack, &estack, &estack - &bstack);
    g_uart.printf("\r\n");

    unsigned sysclk = rcc.getSysclkSpeedInHz() / 1000;
    unsigned ahb    = rcc.getAhbSpeedInHz() / 1000;
    unsigned apb1   = rcc.getApb1SpeedInHz() / 1000;
    unsigned apb2   = rcc.getApb2SpeedInHz() / 1000;

    g_uart.printf("CPU running @ %d kHz\r\n", sysclk);
    g_uart.printf("        AHB @ %d kHz\r\n", ahb);
    g_uart.printf("       APB1 @ %d kHz\r\n", apb1);
    g_uart.printf("       APB2 @ %d kHz\r\n", apb2);
    g_uart.printf("\r\n");

    if (SysTick_Config(rcc.getSysclkSpeedInHz() / 1000)) {
        g_uart.printf("FATAL: Capture Error!\r\n");
        goto bad;
    }

    usbHwDevice.start();

    buttonHandlerQueue = xQueueCreate(2, sizeof(bool));
    if (buttonHandlerQueue == nullptr) {
        g_uart.printf("Failed to create Button Debouncer <-> Buttone Handler Queue\r\n");
        goto bad;
    }
    btn_debounce.setTxQueue(buttonHandlerQueue);
    btn_handler.setRxQueue(buttonHandlerQueue);

    g_uart.printf("Starting FreeRTOS Scheduler...\r\n");
#if !defined(HOSTBUILD)
    vTaskStartScheduler();
#endif /* defined(HOSTBUILD) */

    usbHwDevice.stop();

bad:
    if (buttonHandlerQueue != nullptr) {
        vQueueDelete(buttonHandlerQueue);
    }

    g_led_red.set(gpio::Pin::On);
    g_uart.printf("FATAL ERROR!\r\n");
    while (1) ;

#if defined(HOSTBUILD)
    return (0);
#endif /* defined(HOSTBUILD) */
}

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

/*******************************************************************************
 *
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */
void
halt(const char * const p_file, const unsigned p_line) {
    g_led_red.enable(gpio::GpioAccessViaSTM32F4::e_Output, gpio::GpioAccessViaSTM32F4::e_None, gpio::GpioAccessViaSTM32F4::e_Gpio);
    g_led_red.set(gpio::Pin::On);

    g_uart.printf("%s(): %s : %d\r\n", __func__, p_file, p_line);

    while (1) { };
}

void
assert_failed(uint8_t *p_file, uint32_t p_line) {
    halt(reinterpret_cast<char *>(p_file), p_line);
}

int
usleep(unsigned p_usec) {
    SysTick_Type *sysTick = reinterpret_cast<SysTick_Type *>(SysTick_BASE);

    /*
     * Halt SysTick, if already running. Also, store current SysTick status.
     */
    bool enabled = (sysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0;
    if (enabled) {
        sysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    }

    unsigned safeCtrl = sysTick->CTRL;
    unsigned safeLoad = sysTick->LOAD;
    unsigned safeVal  = sysTick->VAL;

    /*
     * Configure SysTick for 1ms Overflow, then wait for required number of
     * milliseconds.
     */
    const unsigned ticksPerMs = rcc.getSysclkSpeedInHz() / 1000;
    assert((ticksPerMs & 0x00FFFFFF) == ticksPerMs); 
    unsigned waitMs = p_usec / 1000;

    sysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    sysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    sysTick->LOAD = ticksPerMs;
    sysTick->VAL = ticksPerMs;
    sysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    while (waitMs > 0) {
        while (!(sysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) ;
        waitMs--;
    }
    sysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    /*
     * Configure SysTick for 1us Overflow, then wait for required number of
     * microseconds.
     */
    const unsigned ticksPerUs = rcc.getSysclkSpeedInHz() / (1000 * 1000);
    assert((ticksPerUs & 0x00FFFFFF) == ticksPerUs);
    unsigned waitUs = p_usec & 1024; // Assumes 1ms = 1024us. Close enough.

    sysTick->LOAD = ticksPerUs;
    sysTick->VAL = ticksPerUs;
    sysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    while (waitUs > 0) {
        while (!(sysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) ;
        waitUs--;
    }
    sysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    /*
     * Restore SysTick status.
     */
    sysTick->VAL  = safeVal;
    sysTick->LOAD = safeLoad;
    sysTick->CTRL = safeCtrl;
    if (enabled) {
        sysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    }
    
    return 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined (__cplusplus) */

