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
#include "MadgwickAHRS.h"
#include "MyTelemetry.h"
#include "Pwm.h"
#include "Spi.h"
#include "Timer.h"
#include "Usart.h"
#include "actuators/BrushedMotor.h"
#include "control/PidController.h"
#include "imu/lsm6ds3/Lsm6ds3.h"
#include "imu/lsm6ds3/Lsm6ds3SpiBsp.h"
#include "numeric/IncrementalAverage.h"
#include "numeric/RollingAverage.h"
#include "rf/Nrf24L01P.h"
#include "rf/SymaX5HWRxProtocol.h"
#include <cerrno>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SPEED_READOUT_TIMEOUT_MS 40

#define RADIO 1
#define SYMA_RX 1
#define WITH_IMU 1
#define TELEMETRY 1

/*****************************************************************************/

static void SystemClock_Config (void);

/*****************************************************************************/

int main ()
{
        HAL_Init ();
        SystemClock_Config ();
        HAL_Delay (100);

        Gpio uartGpios (GPIOB, GPIO_PIN_7 | GPIO_PIN_6, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, GPIO_AF7_USART3);
        HAL_NVIC_SetPriority (USART1_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ (USART1_IRQn);
        Usart uart (USART1, 115200);

        Debug debug (&uart);
        Debug::singleton () = &debug;
        Debug *d = Debug::singleton ();
        d->print ("Self-balancer here\n");

        Gpio led (GPIOE, GPIO_PIN_2);

        /*+-------------------------------------------------------------------------+*/
        /*| NRF24L01+ Telemetry                                                     |*/
        /*+-------------------------------------------------------------------------+*/

        Gpio ceTelemetry (GPIOD, GPIO_PIN_0);
        ceTelemetry.set (false);

        Gpio irqTelemetryNrf (GPIOD, GPIO_PIN_1, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        Gpio spiTelemetryGpiosNss (GPIOD, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP);
        spiTelemetryGpiosNss.set (true);

        /// PB3 = SCK, PB4 = MISO, PB5 = MOSI
        Gpio spiTelemetryGpiosMisoMosiSck (GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH,
                                           GPIO_AF5_SPI1);

        Spi spiTelemetry (SPI1);
        spiTelemetry.setNssGpio (&spiTelemetryGpiosNss);

        Nrf24L01P nrfTelemetry (&spiTelemetry, &ceTelemetry, &irqTelemetryNrf, 100);
        HAL_NVIC_SetPriority (EXTI1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI1_IRQn);

        /*+-------------------------------------------------------------------------+*/
        /*| NRF24L01+ RC                                                            |*/
        /*+-------------------------------------------------------------------------+*/

#ifdef SYMA_RX
        Gpio ceRc (GPIOD, GPIO_PIN_8);
        ceRc.set (false);

        Gpio irqRcNrf (GPIOB, GPIO_PIN_12, GPIO_MODE_IT_FALLING, GPIO_PULLUP);

        Gpio spiRcGpiosNss (GPIOD, GPIO_PIN_9, GPIO_MODE_OUTPUT_PP);
        spiRcGpiosNss.set (true);

        /// PB3 = SCK, PB4 = MISO, PB5 = MOSI
        Gpio spiRcGpiosMisoMosiSck (GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH,
                                    GPIO_AF5_SPI2);

        Spi spiRc (SPI2);
        spiRc.setNssGpio (&spiRcGpiosNss);

        Nrf24L01P nrfRc (&spiRc, &ceRc, &irqRcNrf, 100);
        HAL_NVIC_SetPriority (EXTI15_10_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI15_10_IRQn);

        SymaX5HWRxProtocol syma (&nrfRc);
#endif
        /*+-------------------------------------------------------------------------+*/
        /*| IMU                                                                     |*/
        /*+-------------------------------------------------------------------------+*/

#ifdef WITH_IMU
        Spi accelerometerSpi (spiTelemetry);
        Gpio accelerometerNss (GPIOE, GPIO_PIN_0);
        accelerometerSpi.setNssGpio (&accelerometerNss);

        Lsm6ds3SpiBsp bsp (&accelerometerSpi);
        Lsm6ds3 lsm (&bsp);

        lsm.softwareReset ();
        lsm.setI2cEnable (false);

        if (lsm.getWhoAmI () != Lsm6ds3::WHO_AM_I_ADDRESS) {
                Error_Handler ();
        }

        lsm.setBdu (Lsm6ds3::BLOCK_UPDATE);
        lsm.setGyroFullScale (Lsm6ds3::FS_245dps);
        lsm.setGyroOdr (Lsm6ds3::GYRO_ODR_1660Hz);
        lsm.setAccelFullScale (Lsm6ds3::FS_2G);
        lsm.setAccelOdr (Lsm6ds3::ACCEL_ODR_1660Hz);
#endif
        /*+-------------------------------------------------------------------------+*/
        /*| Motors                                                                  |*/
        /*+-------------------------------------------------------------------------+*/

        const int PWM_PERIOD = 200;

        // TIM1 -> APB2 (84MHz) -> but CK_INT = 168MHz
        Pwm pwmLeft (TIM1, 42 - 1, PWM_PERIOD - 1);
        pwmLeft.enableChannels (Pwm::CHANNEL2);
        Gpio directionLeftPin (GPIOC, GPIO_PIN_5);
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

        /*+-------------------------------------------------------------------------+*/
        /*| Encoders                                                                |*/
        /*+-------------------------------------------------------------------------+*/

        // PWM for motors.
        int out = 0;

        // TIM2 (32bit) is APB1 (42MHz) so CK_INT is 84MHz. Prescaler 84 -> counter runs @ 1MHz, period 100 gives us UEV frequency 10kHz
        HardwareTimer tim2 (TIM2, 10 * 84 - 1, 65536 - 1);
        Gpio encoderPins (GPIOA, GPIO_PIN_1 | GPIO_PIN_2, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_TIM2);
        InputCaptureChannel inputCapture2 (&tim2, 1, true);

        uint32_t prevCCR2 = 0;
        int encoderDelayL = 0;
        int distanceL = 0;
        float speedL = 0.0f;
        RollingAverage speedAvgL (4);

        inputCapture2.setOnIrq ([&encoderDelayL, &prevCCR2, &out, &distanceL, &speedAvgL, &speedL] {
                uint32_t tmpCcr2 = TIM2->CCR2;
                encoderDelayL = int(tmpCcr2 - prevCCR2);
                prevCCR2 = tmpCcr2;

                if (encoderDelayL < 0) {
                        encoderDelayL += 65536;
                }

                if (encoderDelayL) {
                        speedL = 100000.0f / encoderDelayL;
                }

                if (out > 0) {
                        speedL = -speedL; // Encoders in this motor cant sense the direction. So we assume the speed sign is ruled by the motors
                                          // movement.
                }

                if (out > 0) {
                        ++distanceL;
                }
                else if (out < 0) {
                        --distanceL;
                }
        });

        InputCaptureChannel inputCapture3 (&tim2, 2, true);

        uint32_t prevCCR3 = 0;
        int encoderDelayR = 0;
        int distanceR = 0;
        float speedR = 0.0f;
        RollingAverage speedAvgR (4);

        inputCapture3.setOnIrq ([&encoderDelayR, &prevCCR3, &out, &distanceR, &speedAvgR, &speedR] {
                uint32_t tmpCcr3 = TIM2->CCR3;
                encoderDelayR = int(tmpCcr3 - prevCCR3);
                prevCCR3 = tmpCcr3;

                if (encoderDelayR < 0) {
                        encoderDelayR += 65536;
                }

                if (encoderDelayR) {
                        speedR = 100000.0f / encoderDelayR;
                }

                if (out > 0) {
                        speedR = -speedR; // Encoders in this motor cant sense the direction. So we assume the speed sign is ruled by the motors
                                          // movement.
                }

                if (out > 0) {
                        ++distanceR;
                }
                else if (out < 0) {
                        --distanceR;
                }
        });

        HAL_NVIC_SetPriority (TIM2_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ (TIM2_IRQn);
        TIM2->CNT = 0;

        /*+-------------------------------------------------------------------------+*/
        /*| Spaghetti code                                                          |*/
        /*+-------------------------------------------------------------------------+*/

        HAL_Delay (100);

#ifdef WITH_IMU
        d->print ("Temp : ");
        d->print (int(lsm.getTemperature ()));
        d->print ("\n");
#endif
        HAL_Delay (100);

        Timer imuReadout;
        Timer speedReadout;
        Timer timeControl;

        uint32_t n = 0;

        const int readoutDelayMs = 1;
        const float dt = /*1.0 / readoutDelayMs*/ readoutDelayMs;
        //                const float iScale = dt, dScale = dt / 100.0f;
        //        const float diScale = dt / 1000.0f, ddScale = dt * 10;

        //                float error, prevError, integral, derivative;
        //                integral = derivative = prevError = 0;
        //                float kp, ki, kd;
        //                kp = 1000;
        //                ki = 15;
        //                kd = 200;
        //                float setPoint = 0;
        //        /*const*/ float VERTICAL = ;

        float sError, sPrevError, sIntegral, sDerivative;
        sIntegral = sDerivative = sPrevError = 0;
        float skp, ski, skd;
        skp = 0; // 20.0f / 100000.0f;
        ski = 0;
        skd = 0;
        float sSetPoint = 0;
        int direction = 0;

#ifdef SYMA_RX
        syma.onRxValues = [&sSetPoint, &direction](SymaX5HWRxProtocol::RxValues const &v) {
                sSetPoint = v.yaw * 10.0f;
                direction = v.roll;
        };
#endif

        timeControl.start (1000);
        int timeCnt = 0;

        float ax, ay, az, gx, gy, gz;
        float ofx = 0, ofy = 0, ofz = 0; // Gyro offsets.
        bool sanityBlockade = false;

        // Delay for Madgwick to heat up.
        Timer startupTimer;
        startupTimer.start (5000);

        Timer blinkTimer;

        speedReadout.start (SPEED_READOUT_TIMEOUT_MS);
        float speed = 0;

        PidController angleController (25, -0.025f, dt, dt / 100.0f);
        angleController.setKp (1000);
        angleController.setKi (15);
        angleController.setKd (200);

        PidController speedController (100, 0.0f, dt, dt);
        speedController.setKp (100.0f / 1000.0f);
        speedController.setKi (80.0f / 1000.0f);
        speedController.setKd (900 / 1000.0f);

#ifdef TELEMETRY
        MyTelemetry telemetry (&nrfTelemetry, &angleController, &speedController);
#endif

        while (true) {
                /*+-------------------------------------------------------------------------+*/
                /*| Speed                                                                   |*/
                /*+-------------------------------------------------------------------------+*/
                //                if (speedReadout.isExpired ()) {

                //                        if (out == 0) {
                //                                /*
                //                                 * Input capture frezes when it doesn't receive impulses from encoders when motors aren't
                //                                 spinning,
                //                                 * so this check (for speed == 0) is moved to main loop, which runs all the time.
                //                                 *
                //                                 */
                //                                speedR = 0;
                //                                speedL = 0;
                //                        }

                //                        speedAvgL.run (speedL);
                //                        speedAvgR.run (speedR);
                //                        speed = (speedAvgL.getResult () + speedAvgR.getResult ()) / 2.0f;

#if 0
                        // int distance = -(distanceL + distanceR) / 2;

                        sError = sSetPoint - speed;
                        sIntegral += sError * diScale;

                        // Simple anti integral windup.
                        if (sIntegral > 25) {
                                sIntegral = 25;
                        }
                        else if (sIntegral < -25) {
                                sIntegral = -25;
                        }

                        sDerivative = (sError - sPrevError) / ddScale;
                        // Output of this PID is used as an input of the next.
                        // setPoint = skp * sError + ski * sIntegral + skd * sDerivative;
                        sPrevError = sError;
                        speedReadout.start (SPEED_READOUT_TIMEOUT_MS);
#endif
                //                }

                /*+-------------------------------------------------------------------------+*/
                /*| Angle                                                                   |*/
                /*+-------------------------------------------------------------------------+*/
                if (imuReadout.isExpired ()) {
                        static int i = 0;

                        IGyroscope::GData gd;
#ifdef WITH_IMU
                        gd = lsm.getGData ();
#endif

                        IAccelerometer::AData ad;
#ifdef WITH_IMU
                        ad = lsm.getAData ();
#endif

                        ax = ad.x;
                        ay = ad.y;
                        az = ad.z;
                        gx = gd.x;
                        gy = gd.y;
                        gz = gd.z;

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

                                // 57.2958 degrees == 1 radian
                                gx /= (131.0 * 57.2958); // Scale factor from MPU 6050 docs
                                gy /= (131.0 * 57.2958);
                                gz /= (131.0 * 57.2958);

                                ax /= 16384; // Scale factor from MPU 6050 docs
                                ay /= 16384;
                                az /= 16384;
                        }

                        MadgwickAHRSupdateIMU (gx, gy, gz, ax, ay, az);

                        if (!startupTimer.isExpired ()) {
                                continue;
                        }

                        float pitch = -asinf (-2.0f * (q1 * q3 - q0 * q2));
                        out = angleController.run (pitch, /*setPoint*/ 0.0f);

                        if (sanityBlockade && (angleController.getError () < 0.002f)) {
                                angleController.setIntegral (0);
                                out = 0;
                                sanityBlockade = false;
                        }

                        if (sanityBlockade || pitch > 0.6f || pitch < -0.6f) {
                                out = 0;
                                sanityBlockade = true;
                        }

                        /*---------------------------------------------------------------------------*/

                        if (out == 0) {
                                /*
                                 * Input capture frezes when it doesn't receive impulses from encoders when motors aren't spinning,
                                 * so this check (for speed == 0) is moved to main loop, which runs all the time.
                                 *
                                 */
                                speedR = 0;
                                speedL = 0;
                        }

                        speedAvgL.run (speedL);
                        speedAvgR.run (speedR);
                        speed = (speedAvgL.getResult () + speedAvgR.getResult ()) / 2.0f;

                        float out2 = speedController.run (speed, out);

                        /*---------------------------------------------------------------------------*/

                        motorLeft.setSpeed (out2 - direction);
                        motorRight.setSpeed (out2 + direction);

                        // End
                        imuReadout.start (readoutDelayMs);
                        ++timeCnt;

#ifdef TELEMETRY
                        telemetry.send (++i, pitch, out, speed, /*out2*/ 0);
#endif
                }

                if (timeControl.isExpired ()) {
                        if (timeCnt < (1000 / readoutDelayMs) - 2) {
                                printf ("CPU to slow! timeCnt =  %d\n", timeCnt);
                        }

                        timeCnt = 0;
                        timeControl.start (1000);
                }

                if (blinkTimer.isExpired ()) {
                        blinkTimer.start (100);
                        static bool b = false;
                        led.set ((b = !b));
                }
        }
}

/*****************************************************************************/

void SystemClock_Config () // 16MHz
{

        RCC_OscInitTypeDef RCC_OscInitStruct;
        RCC_ClkInitTypeDef RCC_ClkInitStruct;

        /**Configure the main internal regulator output voltage
         */
        __HAL_RCC_PWR_CLK_ENABLE ();

        __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);

        /**Initializes the CPU, AHB and APB busses clocks
         */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        RCC_OscInitStruct.PLL.PLLM = 16;
        RCC_OscInitStruct.PLL.PLLN = 336;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
        RCC_OscInitStruct.PLL.PLLQ = 4;
        if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK) {
                Error_Handler ();
        }

        /**Initializes the CPU, AHB and APB busses clocks
         */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

        if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
                Error_Handler ();
        }

        /**Configure the Systick interrupt time
         */
        HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq () / 1000);

        /**Configure the Systick
         */
        HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);

        /* SysTick_IRQn interrupt configuration */
        HAL_NVIC_SetPriority (SysTick_IRQn, 0, 0);
}

/*****************************************************************************/

namespace __gnu_cxx {
void __verbose_terminate_handler ()
{
        while (1)
                ;
}
} // namespace __gnu_cxx
