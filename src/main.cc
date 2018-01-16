/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include "Debug.h"
#include "ErrorHandler.h"
#include "Gpio.h"
#include "Hal.h"
#include "HardwareTimer.h"
#include "I2c.h"
#include "MadgwickAHRS.h"
#include "Pwm.h"
#include "Spi.h"
#include "Timer.h"
#include "Usart.h"
#include "actuators/BipolarStepper.h"
#include "actuators/BrushedMotor.h"
#include "imu/mpu6050/Mpu6050.h"
#include "rf/Nrf24L01P.h"
#include "rf/SymaX5HWRxProtocol.h"
#include <cerrno>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const uint8_t ADDRESS_P1[] = { 0xc2, 0xc2, 0xc2, 0xc2, 0xc2 };
const uint8_t CX10_ADDRESS[] = { 0xcc, 0xcc, 0xcc, 0xcc, 0xcc };
#define PACKET_SIZE 5
#define CHANNEL 100

/*****************************************************************************/

static void SystemClock_Config (void);

namespace __gnu_cxx {
void __verbose_terminate_handler ()
{
        while (1)
                ;
}
} // namespace __gnu_cxx

/*****************************************************************************/

int main ()
{
        HAL_Init ();
        SystemClock_Config ();

        Gpio uartGpios (GPIOD, GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, GPIO_AF7_USART3);
        HAL_NVIC_SetPriority (USART3_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ (USART3_IRQn);
        Usart uart (USART3, 115200);

        Debug debug (&uart);
        Debug::singleton () = &debug;
        Debug *d = Debug::singleton ();
        d->print ("Self-balancer here\n");

        /*+-------------------------------------------------------------------------+*/
        /*| NRF24L01+                                                               |*/
        /*+-------------------------------------------------------------------------+*/

        Gpio ceTx (GPIOD, GPIO_PIN_0);
        ceTx.set (false);

        Gpio irqTxNrf (GPIOD, GPIO_PIN_1, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        HAL_NVIC_SetPriority (EXTI1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI1_IRQn);

        Gpio spiTxGpiosNss (GPIOD, GPIO_PIN_2, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
        /// PB3 = SCK, PB4 = MISO, PB5 = MOSI
        Gpio spiTxGpiosMisoMosiSck (GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH,
                                    GPIO_AF5_SPI1);
        Spi spiTx (SPI1);
        spiTx.setNssGpio (&spiTxGpiosNss);

        //#define SYMA_RX

#ifndef SYMA_RX
        Nrf24L01P nrfTx (&spiTx, &ceTx, &irqTxNrf, 50);
        nrfTx.setConfig (Nrf24L01P::MASK_TX_DS, true, Nrf24L01P::CRC_LEN_2);
        nrfTx.setTxAddress (ADDRESS_P1, 5);
        nrfTx.setRxAddress (0, ADDRESS_P1, 5);
        nrfTx.setAutoAck (Nrf24L01P::ENAA_P0 | Nrf24L01P::ENAA_P1);
        nrfTx.setEnableDataPipe (Nrf24L01P::ERX_P0 | Nrf24L01P::ERX_P1);
        nrfTx.setAdressWidth (Nrf24L01P::WIDTH_5);
        nrfTx.setChannel (CHANNEL);
        nrfTx.setAutoRetransmit (Nrf24L01P::WAIT_1000_US, Nrf24L01P::RETRANSMIT_15);
        nrfTx.setDataRate (Nrf24L01P::MBPS_1, Nrf24L01P::DBM_0);
        nrfTx.setEnableDynamicPayload (Nrf24L01P::DPL_P0 | Nrf24L01P::DPL_P0);
        nrfTx.setFeature (Nrf24L01P::EN_DPL | Nrf24L01P::EN_ACK_PAY);
        HAL_Delay (100);
        nrfTx.powerUp (Nrf24L01P::TX);
        HAL_Delay (100);

#if 0
        class TxCallback : public Nrf24L01PCallback {
        public:
                virtual ~TxCallback () {}

                virtual void onRx (uint8_t *data, size_t len)
                {
                        Debug *d = Debug::singleton ();
                        d->print ("nrfTx received : ");
                        d->printArray (data, len);
                        d->print ("\n");
                }

                virtual void onTx () { /*Debug::singleton ()->print ("nrfTx : TX_DS\n");*/}

                virtual void onMaxRt ()
                {
                        Debug *d = Debug::singleton ();
                        d->print ("nrfTx MAX_RT! Unable to send packet!");
                }
        } txCallback;

        nrfTx.setCallback (&txCallback);

        /*---------------------------------------------------------------------------*/
        Gpio ceRx (GPIOE, GPIO_PIN_15);
        ceRx.set (false);

        Gpio irqRxNrf (GPIOB, GPIO_PIN_11, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        HAL_NVIC_SetPriority (EXTI15_10_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI15_10_IRQn);

        Gpio spiRxGpiosNss (GPIOB, GPIO_PIN_12, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
        /// PB13 = SCK, PB14 = MISO, PB15 = MOSI
        Gpio spiRxGpiosMisoMosiSck (GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH,
                                    GPIO_AF5_SPI2);
        Spi spiRx (SPI2);
        spiRx.setNssGpio (&spiRxGpiosNss);

        Nrf24L01P nrfRx (&spiRx, &ceRx, &irqRxNrf, 50);
        nrfRx.setConfig (Nrf24L01P::MASK_TX_DS, true, Nrf24L01P::CRC_LEN_2);
        //        nrfRx.setTxAddress (CX10_ADDRESS, 5);
        //        nrfRx.setRxAddress (0, CX10_ADDRESS, 5);
        nrfRx.setAutoAck (Nrf24L01P::ENAA_P0 | Nrf24L01P::ENAA_P1);
        nrfRx.setEnableDataPipe (Nrf24L01P::ERX_P0 | Nrf24L01P::ERX_P1);
        nrfRx.setAdressWidth (Nrf24L01P::WIDTH_5);
        nrfRx.setChannel (CHANNEL);
        //        nrfRx.setPayloadLength (0, PACKET_SIZE);
        //        nrfRx.setPayloadLength (1, PACKET_SIZE);
        nrfRx.setDataRate (Nrf24L01P::MBPS_1, Nrf24L01P::DBM_0);
        nrfRx.setEnableDynamicPayload (Nrf24L01P::DPL_P0 | Nrf24L01P::DPL_P1);
        nrfRx.setFeature (Nrf24L01P::EN_DPL | Nrf24L01P::EN_ACK_PAY);
        HAL_Delay (100);
        nrfRx.powerUp (Nrf24L01P::RX);
        HAL_Delay (100);

        class RxCallback : public Nrf24L01PCallback {
        public:
                virtual ~RxCallback () {}

                virtual void onRx (uint8_t *data, size_t len)
                {
                        Debug *d = Debug::singleton ();
                        d->print ("nrfRx received : ");
                        d->printArray (data, len);
                        d->print ("\n");
                }

                virtual void onTx () { /*Debug::singleton ()->print ("nrfRx : TX_DS\n");*/}

                virtual void onMaxRt ()
                {
                        Debug *d = Debug::singleton ();
                        d->print ("nrfRx MAX_RT");
                }
        } rxCallback;

        nrfRx.setCallback (&rxCallback);
        uint8_t bufTx[PACKET_SIZE] = { 1, 2, 3, 4, 5 };
        uint8_t ack[PACKET_SIZE] = { 7, 8, 9, 10, 11 };

        Timer timTx;
        Timer timRx;
        while (1) {

//                if (timRx.isExpired ()) {
//                        nrfRx.setAckPayload (1, ack, PACKET_SIZE);
//                        ++(ack[4]);
//                        timRx.start (500);
//                }

                if (timTx.isExpired ()) {
                        nrfTx.transmit (bufTx, 16);
                        ++(bufTx[4]);
                        timTx.start (100);
                }
        }
#endif

#else
        uint8_t bufRx[SymaX5HWRxProtocol::RX_PACKET_SIZE + 1] = {
                0x00,
        };

        /*---------------------------------------------------------------------------*/
        Nrf24L01P nrfRx (&spiRx, &ceRx, &irqRxNrf, 50);

        nrfRx.setConfig (Nrf24L01P::MASK_NO_IRQ, true, Nrf24L01P::CRC_LEN_2);
        nrfRx.setTxAddress (SymaX5HWRxProtocol::BIND_ADDR, 5);
        nrfRx.setRxAddress (0, SymaX5HWRxProtocol::BIND_ADDR, 5);
        nrfRx.setAutoAck (0);
        nrfRx.setEnableDataPipe (Nrf24L01P::ERX_P0);
        nrfRx.setAdressWidth (Nrf24L01P::WIDTH_5);
        nrfRx.setChannel (SymaX5HWRxProtocol::BIND_CHANNELS[0]);
        nrfRx.setAutoRetransmit (Nrf24L01P::WAIT_4000_US, Nrf24L01P::RETRANSMIT_15);
        nrfRx.setPayloadLength (0, SymaX5HWRxProtocol::RX_PACKET_SIZE);
        nrfRx.setDataRate (Nrf24L01P::KBPS_250, Nrf24L01P::DBM_0);

        HAL_Delay (100);
        nrfRx.powerUp (Nrf24L01P::RX);
        HAL_Delay (100);

        SymaX5HWRxProtocol syma (&nrfRx);

        //                nrfRx.setOnData ([&syma, &nrfRx, &bufRx] {
        //                        uint8_t *out = nrfRx.receive (bufRx, SymaX5HWRxProtocol::RX_PACKET_SIZE);
        //                        syma.onPacket (out);
        //                });

        nrfRx.setCallback (&syma);
#endif

#if 1
        /*+-------------------------------------------------------------------------+*/
        /*| MPU6050                                                                 |*/
        /*+-------------------------------------------------------------------------+*/

        // Not used right now
        Gpio irqRxMpu (GPIOB, GPIO_PIN_9, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        HAL_NVIC_SetPriority (EXTI9_5_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI9_5_IRQn);

        Gpio i2cPins (GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, GPIO_AF4_I2C1);
        I2c i2c;
        Mpu6050 mpu6050 (&i2c);
        mpu6050.setFullScaleGyroRange (MPU6050_GYRO_FS_250);
        mpu6050.setFullScaleAccelRange (MPU6050_ACCEL_FS_2);
        // mpu6050.setRate (79);
        mpu6050.setRate (7);

        if (mpu6050.testConnection ()) {
                d->print ("MPU 6050 OK");
        }
        else {
                d->print ("MPU 6050 Fail");
        }

        mpu6050.setTempSensorEnabled (true);

        /*+-------------------------------------------------------------------------+*/
        /*| Motors                                                                  |*/
        /*+-------------------------------------------------------------------------+*/

        const int PWM_PERIOD = 200;

        // TIM1 -> APB2 (84MHz) -> but CK_INT = 168MHz
        Pwm pwmLeft (TIM1, 42 - 1, PWM_PERIOD - 1);
        pwmLeft.enableChannels (Pwm::CHANNEL2);
        Gpio directionLeftPin (GPIOE, GPIO_PIN_9);
        Gpio pwmLeftPin (GPIOE, GPIO_PIN_11, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF1_TIM1);
        BrushedMotor motorLeft (&directionLeftPin, &pwmLeft, Pwm::CHANNEL2, PWM_PERIOD);
        motorLeft.setPwmInvert (true);
        motorLeft.setDirectionInvert (true);

        // TIM3 -> APB1 (42MHz) -> but CK_INT = 84MHz
        Pwm pwmRight (TIM3, 21 - 1, PWM_PERIOD - 1);
        pwmRight.enableChannels (Pwm::CHANNEL3);
        Gpio directionRightPin (GPIOB, GPIO_PIN_1);
        Gpio pwmRightPin (GPIOB, GPIO_PIN_0, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF2_TIM3);
        BrushedMotor motorRight (&directionRightPin, &pwmRight, Pwm::CHANNEL3, PWM_PERIOD);
        motorRight.setPwmInvert (true);

        motorLeft.setSpeed (0);
        motorRight.setSpeed (0);

#if 0
        HardwareTimer tim3 (TIM3, 84 - 1, 100 - 1);
        //        //        tim3.enableChannels (Pwm::CHANNEL1);
        //        OutputCompareChannel oc;
        //        //        oc.setOnIrq ([&] { d->print (".\n"); });
        //        tim3.onUpdate = [&] { motorLeft.step (1); };
        //        tim3.setChannel (0, &oc);
        HAL_NVIC_SetPriority (TIM3_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (TIM3_IRQn);
//        tim3.setDuty (HardwareTimer::CHANNEL1, 2);
//        tim3.setDuty(HardwareTimer::CHANNEL2, 20000);
//        tim3.setDuty(HardwareTimer::CHANNEL3, 20000);
//        tim3.setDuty(HardwareTimer::CHANNEL4, 20000);

        //        Gpio bPhasePinL (GPIOE, GPIO_PIN_9);
        //        Gpio bEnablePinL (GPIOE, GPIO_PIN_11);
        //        Gpio aPhasePinL (GPIOB, GPIO_PIN_1);
        //        Gpio aEnablePinL (GPIOB, GPIO_PIN_0);

        //        BipolarStepper motorLeft (&aPhasePinL, &aEnablePinL, &bPhasePinL, &bEnablePinL, 400);

        //        Gpio bEnablePinR (GPIOA, GPIO_PIN_2);
        //        Gpio bPhasePinR (GPIOA, GPIO_PIN_1);
        //        Gpio aEnablePinR (GPIOC, GPIO_PIN_2);
        //        Gpio aPhasePinR (GPIOC, GPIO_PIN_1);

        //        BipolarStepper motorRight (&aPhasePinR, &aEnablePinR, &bPhasePinR, &bEnablePinR, 400);

        //        tim3.onUpdate = [&motorLeft, &motorRight] {
        //                motorLeft.timeStep ();
        //                motorRight.timeStep ();
        //        };

        //        motorLeft.power (true);
        //        motorRight.power (true);

        // tim3.onUpdate = [&] { d->print ("."); };

        //        motorLeft.power (true);
        //        motorLeft.step (-999999);
        //        motorLeft.power (true);
#endif

        HAL_Delay (100);

        d->print ("Temp : ");
        d->print (int(mpu6050.getTemperature ()));
        d->print ("\n");

        HAL_Delay (100);

        Timer readout;
        Timer timeControl;

        uint32_t n = 0;

        const int readoutDelayMs = 1;
        const float dt = /*1.0 / readoutDelayMs*/ readoutDelayMs;
        const float iScale = dt, dScale = dt / 100.0;
        float error, prevError, integral, derivative;
        integral = derivative = prevError = 0;
        float kp, ki, kd, out;
        kp = 0;
        ki = 0;
        kd = 0;
        float setPoint = 0;

#ifndef SYMA_RX

#if 1
        class TxCallback : public Nrf24L01PCallback {
        public:
                virtual ~TxCallback () {}

                virtual void onRx (uint8_t *data, size_t len)
                {
                        uint8_t command = data[0];
                        int param = *reinterpret_cast<int *> (data + 1);

                        Debug *d = Debug::singleton ();
                        d->print ("Command : ");
                        d->print (&command, 1);
                        d->print (" ");
                        d->print (param);
                        d->print ("\n");

                        switch (command) {
                        case 'p':
                                *kp = param * 10.0;
                                break;

                        case 'i':
                                *ki = param / 10.0;
                                break;

                        case 'd':
                                *kd = param * 10.0;
                                break;

                        case 'c':
                                *sp = param / 10000.0;
                                break;

                        case 'm':
                                motorLeft->setSpeed (param);
                                motorRight->setSpeed (param);
                                break;

                        default:
                                break;
                        }

                        *integral = 0;
                }

                virtual void onTx () {}

                virtual void onMaxRt ()
                {
                        Debug *d = Debug::singleton ();
                        d->print ("nrfTx MAX_RT! Unable to send packet!");
                }

                float *kp, *ki, *kd, *sp, *integral;
                BrushedMotor *motorLeft, *motorRight;
        } txCallback;

        txCallback.kp = &kp;
        txCallback.ki = &ki;
        txCallback.kd = &kd;
        txCallback.sp = &setPoint;
        txCallback.motorLeft = &motorLeft;
        txCallback.motorRight = &motorRight;
        txCallback.integral = &integral;

        nrfTx.setCallback (&txCallback);
#endif

#else

        syma.onRxValues = [&motorLeft, &motorRight, &d](SymaX5HWRxProtocol::RxValues const &v) {
                //                motorLeft.power (true);
                motorLeft.setSpeed (v.throttle);
                motorRight.setSpeed (v.throttle);
                d->print (v.throttle);
                d->print ("\n");

                //                if (v.throttle == 0) {
                //                        motorLeft.power (false);
                //                }
        };
#endif

        //        nrfRx.powerUp (Nrf24L01P::RX);

        timeControl.start (1000);
        int timeCnt = 0;

        int16_t iax, iay, iaz, igx, igy, igz;
        float ax, ay, az, gx, gy, gz;
        float ofx = 0, ofy = 0, ofz = 0; // Gyro offsets

        //                while (true) {
        //                }

        //        FloatQueue integralElements (100);

        while (1) {
                if (readout.isExpired ()) {

                        // mpu6050.getMotion6 (&az, &ay, &ax, &gz, &gy, &gx);
                        mpu6050.getMotion6 (&iaz, &iay, &iax, &igz, &igy, &igx);
                        ax = iax;
                        ay = iay;
                        az = iaz;
                        gx = igx;
                        gy = igy;
                        gz = igz;

#if 1
                        // "Calibration"
                        if (++n < 50) {
                                continue;
                        }
                        else if (n < 250) {
                                ofx += gx;
                                ofy += gy;
                                ofz += gz;
                                // printf ("%d, %d\n", gx, gy);
                                continue;
                        }
                        else if (n == 250) {
                                ofx /= 200.0;
                                ofy /= 200.0;
                                ofz /= 200.0;
                                // printf ("Offsets : %d, %d\n", int(ofx), int(ofy));
                        }
                        else {
                                gx -= ofx;
                                gy -= ofy;
                                gz -= ofz;

                                gx /= (131.0 * 57.2958); // Scale factor from MPU 6050 docs
                                gy /= (131.0 * 57.2958);
                                gz /= (131.0 * 57.2958);

                                ax /= 4096.0 * 4; // Scale factor from MPU 6050 docs
                                ay /= 4096.0 * 4;
                                az /= 4096.0 * 4;
                        }
#endif

                        MadgwickAHRSupdateIMU (gx, gy, -gz, ax, ay, az);
                        float pitch = asinf (-2.0f * (q1 * q3 - q0 * q2));

                        // PID
                        error = setPoint - pitch;

                        integral += error * iScale;

                        // Simple anti integral windup.
                        if (integral > 500) {
                                integral = 500;
                        }
                        else if (integral < -500) {
                                integral = -500;
                        }

                        derivative = (error - prevError) / dScale;
                        out = kp * error + ki * integral + kd * derivative;
                        prevError = error;

                        motorLeft.setSpeed (out);
                        motorRight.setSpeed (out);

                        // End
                        readout.start (readoutDelayMs);
                        ++timeCnt;

#if 1
                        static int i = 0;

                        if (++i % 100 == 0) {
                                uint8_t buf[32];

                                // Sending ints since reveiver runs on STM32F0 without fp, and cant printf floats.
                                int pitchI = pitch * 1000;
                                int errorI = error * 1000;
                                int integralI = integral * 100;
                                int derivativeI = derivative * 100;
                                int outI = out * 100;

                                // printf ("%d,%d,%d,%d\n", pitchI, errorI, integralI, derivativeI);

                                memcpy (buf, &pitchI, 4);
                                memcpy (buf + 4, &errorI, 4);
                                memcpy (buf + 8, &integralI, 4);
                                memcpy (buf + 12, &derivativeI, 4);
                                memcpy (buf + 16, &outI, 4);
                                nrfTx.transmit (buf, 20);
                        }
#endif
                }

#if 1
                if (timeControl.isExpired ()) {
                        if (timeCnt < (1000 / readoutDelayMs) - 2) {
                                printf ("CPU to slow! timeCnt =  %d\n", timeCnt);
                        }

                        timeCnt = 0;
                        timeControl.start (1000);
                }
#endif
        }
#endif
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
