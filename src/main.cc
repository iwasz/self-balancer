/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include "Debug.h"
#include "Gpio.h"
#include "Hal.h"
#include "Nrf24L01P.h"

static void SystemClock_Config (void);

namespace __gnu_cxx {
void __verbose_terminate_handler ()
{
        while (1)
                ;
}
}

/*****************************************************************************/

int main ()
{
        HAL_Init ();
        SystemClock_Config ();

        Gpio debugGpios (GPIOD, GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF7_USART3);
        Debug *d = Debug::singleton ();
        d->init (115200);
        d->print ("selfbalancer\n");

        /*---------------------------------------------------------------------------*/

        Gpio ceRx (GPIOB, GPIO_PIN_11);
        ceRx.set (false);

        Gpio irqRx (GPIOB, GPIO_PIN_13, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        HAL_NVIC_SetPriority (EXTI15_10_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI15_10_IRQn);

        Gpio spiRxGpiosNss (GPIOB, GPIO_PIN_12, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
        Gpio spiRxGpiosMisoMosiSck (GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI1);
        Spi spiRx (SPI1);
        spiRx.setNssPin (&spiRxGpiosNss);

        Nrf24L01P nrfRx (&spiRx, &ceRx, &irqRx);
        nrfRx.setConfig (Nrf24L01P::MASK_NO_IRQ, true, Nrf24L01P::CRC_LEN_1);
        nrfRx.setAutoAck (Nrf24L01P::ENAA_P1 | Nrf24L01P::ENAA_P0);      // Redundant
        nrfRx.setEnableDataPipe (Nrf24L01P::ERX_P1 | Nrf24L01P::ERX_P0); // Redundant
        nrfRx.setAdressWidth (Nrf24L01P::WIDTH_5);                       // Redundant
        nrfRx.setAutoRetransmit (Nrf24L01P::WAIT_1000, Nrf24L01P::RETRANSMIT_15);
        nrfRx.setChannel (100);
        nrfRx.setDataRate (Nrf24L01P::MBPS_1, Nrf24L01P::DBM_0);
        nrfRx.setPayloadLength (0, 1);
        nrfRx.setPayloadLength (1, 1);

        uint8_t bufRx[4] = {
                0,
        };

        // TODO this works only if we receive manually in main loop
        nrfRx.setOnData ([d, &nrfRx, &bufRx] {
                nrfRx.receive (bufRx, 1);
                d->print (bufRx[0]);
                d->print ("\n");
        });

        nrfRx.powerUp (Nrf24L01P::RX);

        //        uint8_t bufTx[4];
        //        bufTx[0] = 0x66;

        while (1) {
                                nrfRx.receive (bufRx, 1);
                                d->print (bufRx[0]);
                                d->print ("\n");
                HAL_Delay (1000);
//                d->print (".");
        }
}

/*****************************************************************************/

static void SystemClock_Config (void)
{
        RCC_ClkInitTypeDef RCC_ClkInitStruct;
        RCC_OscInitTypeDef RCC_OscInitStruct;

        /* Enable Power Control clock */
        __HAL_RCC_PWR_CLK_ENABLE ();

        /* The voltage scaling allows optimizing the power consumption when the device is
           clocked below the maximum system frequency, to update the voltage scaling value
           regarding system frequency refer to product datasheet.  */
        __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);

        /* Enable HSE Oscillator and activate PLL with HSE as source */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        RCC_OscInitStruct.PLL.PLLM = 8;
        RCC_OscInitStruct.PLL.PLLN = 336;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
        RCC_OscInitStruct.PLL.PLLQ = 7;
        HAL_RCC_OscConfig (&RCC_OscInitStruct);

        /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
           clocks dividers */
        RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
        HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_5);

        /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
        if (HAL_GetREVID () == 0x1001) {
                /* Enable the Flash prefetch */
                __HAL_FLASH_PREFETCH_BUFFER_ENABLE ();
        }
}
