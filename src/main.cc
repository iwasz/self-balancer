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
#include "I2c.h"
#include "Nrf24L01P.h"
#include "Pwm.h"
#include "Spi.h"
#include "Timer.h"
#include "imu/mpu6050/Mpu6050.h"
#include <cerrno>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void SystemClock_Config (void);

namespace __gnu_cxx {
void __verbose_terminate_handler ()
{
        while (1)
                ;
}
} // namespace __gnu_cxx

/*****************************************************************************/

class Motor {
public:
        Motor (Pwm *p, Pwm::Channel channelFwd, Pwm::Channel channelRev, uint32_t fullScale)
            : pwm (p), channelFwd (channelFwd), channelRev (channelRev), factor (fullScale / 100), fullScale (fullScale)
        {
        }

        /**
         * @brief setSpeed
         * @param speed from -100 to 100
         */
        void setSpeed (int32_t speed);

private:
        Pwm *pwm;
        Pwm::Channel channelFwd;
        Pwm::Channel channelRev;
        uint32_t factor;
        uint32_t fullScale;
};

/*****************************************************************************/

void Motor::setSpeed (int32_t speed)
{
        int newDuty = speed * factor;

        newDuty = std::min<int> (std::abs (newDuty), fullScale);

        if (speed > 0) {
                pwm->setDuty (channelFwd, newDuty);
                pwm->setDuty (channelRev, 1);
        }
        else if (speed < 0) {
                pwm->setDuty (channelFwd, 1);
                pwm->setDuty (channelRev, newDuty);
        }
        else {
                pwm->setDuty (channelFwd, 1);
                pwm->setDuty (channelRev, 1);
        }
}

/*****************************************************************************/

template <class T> class CircularBuffer {
public:
        CircularBuffer (size_t size) : buf (new T[size]), size (size) {}
        ~CircularBuffer () { delete[] buf; }

        void put (T item);
        T get (void);
        T operator[] (int i) const;

        void reset () { head = tail; }
        bool isEmpty () const { return head == tail; }

        bool isFull () const { return ((head + 1) % size) == tail; }
        size_t getSize () const { return size - 1; }

private:
        T *buf;
        size_t head = 0;
        size_t tail = 0;
        size_t size;
};

/*****************************************************************************/

template <class T> void CircularBuffer<T>::put (T item)
{
        buf[head] = item;
        head = (head + 1) % size;

        if (head == tail) {
                tail = (tail + 1) % size;
        }
}

/*****************************************************************************/

template <class T> T CircularBuffer<T>::get ()
{
        if (isEmpty ()) {
                return T ();
        }

        // Read data and advance the tail (we now have a free space)
        auto val = buf[tail];
        tail = (tail + 1) % size;

        return val;
}

/*****************************************************************************/

template <class T> T CircularBuffer<T>::operator[] (int i) const { return buf[tail + (i % size)]; }

/*****************************************************************************/

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

        Gpio debugGpios (GPIOD, GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, GPIO_AF7_USART3);
        Debug *d = Debug::singleton ();
        d->init (115200);
        d->print ("selfbalancer\n");

        /*+-------------------------------------------------------------------------+*/
        /*| NRF24L01+                                                               |*/
        /*+-------------------------------------------------------------------------+*/

        Gpio ceRx (GPIOD, GPIO_PIN_0);
        ceRx.set (false);

        Gpio irqRxNrf (GPIOD, GPIO_PIN_1, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        HAL_NVIC_SetPriority (EXTI1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI1_IRQn);

        Gpio spiRxGpiosNss (GPIOD, GPIO_PIN_2, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
        Gpio spiRxGpiosMisoMosiSck (GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI1);
        Spi spiRx (SPI1);
        spiRx.setNssPin (&spiRxGpiosNss);

        Nrf24L01P nrfRx (&spiRx, &ceRx, &irqRxNrf);
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

        Pwm pwmLeft (TIM1, (uint32_t) (HAL_RCC_GetHCLKFreq () / 2000000) - 1, 10000 - 1);
        pwmLeft.enableChannels (Pwm::CHANNEL1 | Pwm::CHANNEL2);
        Gpio pwmLeftPins (GPIOE, GPIO_PIN_9 | GPIO_PIN_11, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF1_TIM1);
        Motor motorLeft (&pwmLeft, Pwm::CHANNEL1, Pwm::CHANNEL2, 10000);

        Pwm pwmRight (TIM3, (uint32_t) (HAL_RCC_GetHCLKFreq () / 2000000) - 1, 10000 - 1);
        pwmRight.enableChannels (Pwm::CHANNEL3 | Pwm::CHANNEL4);
        Gpio pwmRightPins (GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF2_TIM3);
        Motor motorRight (&pwmRight, Pwm::CHANNEL3, Pwm::CHANNEL4, 10000);

        pwmLeft.setDuty (Pwm::CHANNEL1, 1);
        pwmRight.setDuty (Pwm::CHANNEL3, 1);

        motorLeft.setSpeed (0);
        motorRight.setSpeed (0);

        HAL_Delay (100);

        d->print ("Temp : ");
        d->print (mpu6050.getTemperature ());
        d->print ("\n");

        HAL_Delay (100);


        Timer readout;

#if 0
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

#else
        uint32_t n = 0;
        float angleAccel, angle = 0;
        float ofx = 0, ofy = 0, ofz = 0;

        const int readoutDelayMs = 20;
        const float dt = 1.0 / readoutDelayMs;
        const float GYROSCALE = 1000;
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        float error, prevError, integral, derivative;
        integral = derivative = prevError = 0;
        float kp, ki, kd, out;
        kp = 100;
        ki = 1;
        kd = 1;

        while (1) {
                if (readout.isExpired ()) {
                        mpu6050.getMotion6 (&az, &ay, &ax, &gz, &gy, &gx);
                        angleAccel = atan (float(ay) / float(ax));
                        angleAccel = (angleAccel > 0) ? ((M_PI / 2) - angleAccel) : (-((M_PI / 2) + angleAccel));

                        // Korekta (ułożenie akcelerometru względem ramy). TODO automatycznie?
                        angleAccel -= 0.14;

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

                        angle -= (gz / GYROSCALE) * dt;
                        angle = 0.95 * (angle) + 0.05 * angleAccel * 13; // TODO to 13 skaluje akcelerometr

                        // PID
                        error = 0 - angle;
                        integral += error * dt;
                        derivative = (error - prevError) / dt;
                        out = kp * error + ki * integral + kd * derivative;
                        prevError = error;

//                        motorLeft.setSpeed (out);
//                        motorRight.setSpeed (out);

                        // End
//                        printf ("%d, %d, %d, %d, %d\n", int(angleAccel * 1000), gy, gz, int(angle * 1000), int(out * 100));
                        readout.start (readoutDelayMs);

                        // Nrf
                        nrfRx.receive (bufRx, 1);
                        d->print (bufRx[0]);
                        d->print ("\n");
                }
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



/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
#if 0

/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
//__attribute__ ((interrupt ("IRQ")))
extern "C" void NMI_Handler (void) {}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
//__attribute__ ((interrupt ("IRQ")))
extern "C" void HardFault_Handler (void)
{
        /* Go to infinite loop when Hard Fault exception occurs */
        while (1) {
        }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
//__attribute__ ((interrupt ("IRQ")))
extern "C" void MemManage_Handler (void)
{
        /* Go to infinite loop when Memory Manage exception occurs */
        while (1) {
        }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
//__attribute__ ((interrupt ("IRQ")))
extern "C" void BusFault_Handler (void)
{
        /* Go to infinite loop when Bus Fault exception occurs */
        while (1) {
        }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
//__attribute__ ((interrupt ("IRQ")))
extern "C" void UsageFault_Handler (void)
{
        /* Go to infinite loop when Usage Fault exception occurs */
        while (1) {
        }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
//__attribute__ ((interrupt ("IRQ")))
extern "C" void SVC_Handler (void) {}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
//__attribute__ ((interrupt ("IRQ")))
extern "C" void DebugMon_Handler (void) {}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
//__attribute__ ((interrupt ("IRQ")))
extern "C" void PendSV_Handler (void) {}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
//__attribute__ ((interrupt ("IRQ")))
extern "C" void SysTick_Handler (void)
{
        HAL_IncTick ();

        /* Call user callback */
        HAL_SYSTICK_IRQHandler ();
}
#endif
