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
//#include <cstring>

// TIM_HandleTypeDef htim;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const uint8_t CX10_ADDRESS[] = { 0xcc, 0xcc, 0xcc, 0xcc, 0xcc };
#define PACKET_SIZE 5
#define CHANNEL 100

/*****************************************************************************/

// extern "C" void TIM3_IRQHandler ()
//{
//        TIM_TypeDef *instance = htim.Instance;

//        if (instance->DIER & TIM_IT_UPDATE && instance->SR & TIM_FLAG_UPDATE) {
//                // Clears the interrupt
//                instance->SR = ~TIM_IT_UPDATE;

//                Debug::singleton ()->print ("%\n");
//        }
//}

static void SystemClock_Config (void);

namespace __gnu_cxx {
void __verbose_terminate_handler ()
{
        while (1)
                ;
}
} // namespace __gnu_cxx

//#define SAMPLEFILTER_TAP_NUM 5

// static double filter_taps[SAMPLEFILTER_TAP_NUM]
//        = { 0.02857983994169657,   -0.07328836181028245, 0.04512928732568175,   0.03422632401030237,  -0.034724262386629436, -0.05343090761376418,
//            0.032914528649623416,  0.09880818246272206,  -0.034135422078843417, -0.3160339484471911,  0.5341936566511765,    -0.3160339484471911,
//            -0.034135422078843417, 0.09880818246272206,  0.032914528649623416,  -0.05343090761376418, -0.034724262386629436, 0.03422632401030237,
//            0.04512928732568175,   -0.07328836181028245, 0.02857983994169657 };

// static double filter_taps[SAMPLEFILTER_TAP_NUM]
//        = { -0.25, -0.25, -0.25, -0.25, 1 };

static constexpr float filter_taps[] = {
        0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2,
        0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2,
};

template <typename FloatType, int num_coeffs, const FloatType *coeffs> class FirFilter {
public:
        FirFilter () : current_index_ (0)
        {
                for (int i = 0; i < num_coeffs; ++i) {
                        history_[i] = 0.0;
                }
        }

        void put (FloatType value)
        {
                history_[current_index_++] = value;

                if (current_index_ == num_coeffs) {
                        current_index_ = 0;
                }
        }

        FloatType get ()
        {
                FloatType output = 0.0;
                int index = current_index_;

                for (int i = 0; i < num_coeffs; ++i) {
                        if (index != 0) {
                                --index;
                        }
                        else {
                                index = num_coeffs - 1;
                        }
                        output += history_[index] * coeffs[i];
                }
                return output;
        }

private:
        FloatType history_[num_coeffs];
        int current_index_;
};

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

        Gpio ceRx (GPIOD, GPIO_PIN_0);
        ceRx.set (false);

        Gpio irqRxNrf (GPIOD, GPIO_PIN_1, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        HAL_NVIC_SetPriority (EXTI1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI1_IRQn);

        Gpio spiRxGpiosNss (GPIOD, GPIO_PIN_2, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
        /// PB3 = SCK, PB4 = MISO, PB5 = MOSI
        Gpio spiRxGpiosMisoMosiSck (GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI1);
        Spi spiRx (SPI1);
        spiRx.setNssPin (&spiRxGpiosNss);

        Nrf24L01P nrfRx (&spiRx, &ceRx, &irqRxNrf);

        // #define SYMA_RX

#ifndef SYMA_RX
        nrfRx.setConfig (Nrf24L01P::MASK_NO_IRQ, true, Nrf24L01P::CRC_LEN_2);
        nrfRx.setTxAddress (CX10_ADDRESS, 5);
        nrfRx.setRxAddress (0, CX10_ADDRESS, 5);
        nrfRx.setAutoAck (Nrf24L01P::ENAA_P0);
        nrfRx.setEnableDataPipe (Nrf24L01P::ERX_P0);
        nrfRx.setAdressWidth (Nrf24L01P::WIDTH_5);
        nrfRx.setChannel (CHANNEL);
        nrfRx.setAutoRetransmit (Nrf24L01P::WAIT_1000, Nrf24L01P::RETRANSMIT_15);
        nrfRx.setPayloadLength (0, PACKET_SIZE);
        nrfRx.setDataRate (Nrf24L01P::MBPS_1, Nrf24L01P::DBM_0);

        uint8_t bufRx[PACKET_SIZE + 1] = {
                1,
        };
#else
        uint8_t bufRx[SymaX5HWRxProtocol::RX_PACKET_SIZE + 1] = {
                0x00,
        };

        /*---------------------------------------------------------------------------*/

        nrfRx.setConfig (Nrf24L01P::MASK_NO_IRQ, true, Nrf24L01P::CRC_LEN_2);
        nrfRx.setTxAddress (SymaX5HWRxProtocol::BIND_ADDR, 5);
        nrfRx.setRxAddress (0, SymaX5HWRxProtocol::BIND_ADDR, 5);
        nrfRx.setAutoAck (0);
        nrfRx.setEnableDataPipe (Nrf24L01P::ERX_P0);
        nrfRx.setAdressWidth (Nrf24L01P::WIDTH_5);
        nrfRx.setChannel (SymaX5HWRxProtocol::BIND_CHANNELS[0]);
        nrfRx.setAutoRetransmit (Nrf24L01P::WAIT_4000, Nrf24L01P::RETRANSMIT_15);
        nrfRx.setPayloadLength (0, SymaX5HWRxProtocol::RX_PACKET_SIZE);
        nrfRx.setDataRate (Nrf24L01P::KBPS_250, Nrf24L01P::DBM_0);

        HAL_Delay (100);
        nrfRx.powerUp (Nrf24L01P::RX);
        HAL_Delay (100);

        SymaX5HWRxProtocol syma (&nrfRx);

        nrfRx.setOnData ([&syma, &nrfRx, &bufRx] {
                uint8_t *out = nrfRx.receive (bufRx, SymaX5HWRxProtocol::RX_PACKET_SIZE);
                syma.onPacket (out);
        });
#endif
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

        // 1kHz if I'm correct
        const int PWM_PERIOD = 200;

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
        d->print (mpu6050.getTemperature ());
        d->print ("\n");

        HAL_Delay (100);

        Timer readout;
        Timer timeControl;

        /*
                const int AVG_LEN = 5;
                int i = 0;
                CircularBuffer<int16_t> gInX (3);
                CircularBuffer<int16_t> gOutX (2);
                CircularBuffer<int16_t> gyy (3);
                CircularBuffer<int16_t> gyz (3);
                float angle = 0;
                float m1, m;
                m1 = m = 0;
                uint32_t n = 0;

                FirFilter<float, sizeof (filter_taps) / sizeof (float), filter_taps> myFilter;

                float out = 0;


                while (1) {
                        int16_t axt[AVG_LEN], ayt[AVG_LEN], azt[AVG_LEN];
                        int ax, ay, az;
                        int16_t gx, gy, gz;

                        //                int16_t gxin[3], gyin[3], gzin[3];
                        //                int gk = 0;

                        float wc = tan (M_PI * (300.0 / 1000.0));
                        float k1 = 2 * wc, k2 = wc * wc;
                        float a0, a1, a2, b1, b2;

                        a0 = a2 = k2 / (1 + k1 + k2);
                        a1 = -2 * a0;
                        b1 = -((1 / k2) - 1) * 2 * a0;
                        b2 = 1 - (a0 + a1 + a2 + b1);

                        if (readout.isExpired ()) {
                                mpu6050.getMotion6 (&axt[i], &ayt[i], &azt[i], &gx, &gy, &gz);

                                m = m1 + ((gx - m1) / ++n);
                                m1 = m;

                                //                        gx -= m;
                                //                        gy -= 113;
                                //                        gz -= 63;

                                //                        output[i] = (input[i] * ALPHA) + (ouptut[i] * (1.0 - ALPHA));
                                out = (gx - m) * ALPHA + (out * (1.0 - ALPHA));

                                angle += out;
                                myFilter.put ((gx - m));

                                // Low pass filter for accelerometer
                                if (i < AVG_LEN - 1) {
                                        ++i;
                                }
                                else {
                                        i = 0;

                                        ax = ay = az = 0;

                                        for (int j = 0; j < AVG_LEN; ++j) {
                                                ax += axt[j];
                                                ay += ayt[j];
                                                az += azt[j];
                                        }

                                        ax /= AVG_LEN;
                                        ay /= AVG_LEN;
                                        az /= AVG_LEN;
                                }

                                // High pass flter for gyroscope
                                //                        gInX.put (gx);

                                //                        if (gInX.isFull ()) {
                                //                                int o = a0 * gInX[0] + a1 * gInX[-1] + a2 * gInX[-2] + b1 * gOutX[0] + b2 * gOutX[-1];
                                //                                gOutX.put (o);
                                //                                // d->print (gOutX.get ());
                                //                                // d->print (", ");
                                //                        }

                                //                nrfRx.receive (bufRx, 1);
                                //                d->print (bufRx[0]);
                                //                d->print ("\n");

                                //                        printf ("%d, %d, %d, %d, %d, %d, %d\n", ax, ay, ax, gx, gy, gz, angle);
                                printf ("%d, %d, %d, angle = %d\n", gx, int(gx - m), int(out), int(angle));

                                float R = sqrt (float(ax * ax) + float(ay * ay) + float(az * az));
                                // float anX = acosf (ax / R);
                                // float anY = acosf (ay / R);
                                float anZ = acosf (az / R);

                                float error = anZ - angleSp;
                                // d->print (-error * 600);
                                // d->print ("\n");

                                //                        motorLeft.setSpeed (-error * 1000);
                                //                        motorRight.setSpeed (-error * 1000);

                                readout.start (20);
                        }
                }
        */
        uint32_t n = 0;
        float angleAccel, angle = 0, angleGyro = 0;
        float ofx = 0, ofy = 0, ofz = 0;

        // TODO why when set to 10, timeCnt reads 91, and when set to 9 timeCnt is 100? When 10 it shoud be 100.
        const int readoutDelayMs = 9;
        const float dt = /*1.0 / readoutDelayMs*/ readoutDelayMs + 1;
        const float GYROSCALE = 6500;
        const int ACCELSCALE = 13;
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        float error, prevError, integral, derivative;
        integral = derivative = prevError = 0;
        float kp, ki, kd, out;
        kp = 0;
        ki = 0;
        kd = 0;
        float correction = 0.12;

#ifndef SYMA_RX
        nrfRx.setOnData ([d, &nrfRx, &bufRx, &kp, &ki, &kd, &correction, &integral, &prevError, &motorLeft, &motorRight] {
                //                d->print ("IRQ: ");
                uint8_t *out = nrfRx.receive (bufRx, PACKET_SIZE);
                //                for (int i = 0; i < PACKET_SIZE; ++i) {
                //                        d->print (out[i]);
                //                        d->print (",");
                //                }
                //                d->print ("\n");

                uint8_t command = out[0];
                int param = *reinterpret_cast<int *> (out + 1);

                d->print ("Command : ");
                d->print (&command, 1);
                d->print (" ");
                d->print (param);
                d->print ("\n");

                switch (command) {
                case 'p':
                        kp = param;
                        break;

                case 'i':
                        ki = param / 1000.0;
                        break;

                case 'd':
                        kd = param / 1000.0;
                        break;

                case 'c':
                        correction = param / 1000.0;
                        break;

                case 'm':
                        motorLeft.setSpeed (param);
                        motorRight.setSpeed (param);
                        //                        motorLeft.power (true);
                        //                        motorLeft.setSpeed (param);

                        //                        if (param == 0) {
                        //                                motorLeft.power (false);
                        //                        }
                        break;

                default:
                        d->print ("Unknown command. Available cmds : p,i,d,c");
                        break;
                }

                integral = 0;
                prevError = 0;
        });
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

        nrfRx.powerUp (Nrf24L01P::RX);

        timeControl.start (1000);
        int timeCnt = 0;

//        while (true) {
//        }

        while (1) {
                if (readout.isExpired ()) {
                        mpu6050.getMotion6 (&az, &ay, &ax, &gz, &gy, &gx);
                        angleAccel = atan (float(ay) / float(ax));
                        angleAccel = (angleAccel > 0) ? ((M_PI / 2) - angleAccel) : (-((M_PI / 2) + angleAccel));

                        // Korekta (ułożenie akcelerometru względem ramy). TODO automatycznie?
                        angleAccel -= correction;

                        // "Calibration"
                        if (++n < 200) {
                                ofx += gx;
                                ofy += gy;
                                ofz += gz;
                                printf ("%d, %d\n", gx, gy);
                                continue;
                        }
                        else if (n == 200) {
                                ofx /= 200.0;
                                ofy /= 200.0;
                                ofz /= 200.0;
                                printf ("Offsets : %d, %d\n", int(ofx), int(ofy));
                        }

                        gx -= ofx;
                        gy -= ofy;
                        gz -= ofz;

                        angle -= (gz / GYROSCALE) / dt;
                        angleGyro = angle;
                        angleAccel *= ACCELSCALE;

                        /*
                         * TODO to 13 skaluje akcelerometr. TODO2 ALE CZEMU!? Nie pamiętam po co to było
                         * ale prawdopodobnie chodziło o to, żeby dopasować jednostki żyroskopu i akcelerometru
                         * do siebie.
                         */
                        angle = 0.98 * (angle) + 0.02 * angleAccel;

                        // PID
                        error = 0 - angle;
                        integral += error * dt;
                        derivative = (error - prevError) / dt;
                        out = kp * error + ki * integral + kd * derivative;
                        prevError = error;

                        motorLeft.setSpeed (out);
                        motorRight.setSpeed (out);

                        // End
                        //                        printf ("%d, %d, %d, %d, %d\n", int(angleAccel * 1000), gy, gz, int(angle * 1000), int(out * 100));
                        readout.start (readoutDelayMs);
                        ++timeCnt;
                        // printf ("gyro : %d, accell : %d, complement : %d\n", int(angleGyro * 1000), int (angleAccel * 1000), int (angle * 1000));
                }

                //                if (timeControl.isExpired ()) {
                //                        printf ("%d, %d\n", timeCnt, int (angle * 1000));
                //                        timeCnt = 0;
                //                        timeControl.start (1000);
                //                }
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
