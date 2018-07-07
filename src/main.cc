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
#include <imu/lsm6ds3/Lsm6ds3.h>
#include <imu/lsm6ds3/Lsm6ds3SpiBsp.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const uint8_t ADDRESS_P1[] = { 0xc2, 0xc2, 0xc2, 0xc2, 0xc2 };
const uint8_t CX10_ADDRESS[] = { 0xcc, 0xcc, 0xcc, 0xcc, 0xcc };
#define PACKET_SIZE 5
#define CHANNEL 100
#define RADIO 1

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
        /*| NRF24L01+                                                               |*/
        /*+-------------------------------------------------------------------------+*/

        Gpio ceTx (GPIOD, GPIO_PIN_0);
        ceTx.set (false);

        Gpio irqTxNrf (GPIOD, GPIO_PIN_1, GPIO_MODE_IT_FALLING, GPIO_PULLUP);
        HAL_NVIC_SetPriority (EXTI1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ (EXTI1_IRQn);

        Gpio spiTxGpiosNss (GPIOD, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP);
        spiTxGpiosNss.set (true);

        /// PB3 = SCK, PB4 = MISO, PB5 = MOSI
        Gpio spiTxGpiosMisoMosiSck (GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH,
                                    GPIO_AF5_SPI1);

        Spi spiTx (SPI1);
        spiTx.setNssGpio (&spiTxGpiosNss);

        //#define SYMA_RX

#ifdef RADIO
#ifndef SYMA_RX
        Nrf24L01P nrfTx (&spiTx, &ceTx, &irqTxNrf, 100);
        nrfTx.setConfig (Nrf24L01P::MASK_TX_DS, true, Nrf24L01P::CRC_LEN_2);
        nrfTx.setTxAddress (ADDRESS_P1, 5);
        nrfTx.readRegister (Nrf24L01P::TX_ADDR);
        nrfTx.setRxAddress (0, ADDRESS_P1, 5);
        nrfTx.readRegister (Nrf24L01P::RX_ADDR_P0);
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
#endif // RADIO

#if 1
        /*+-------------------------------------------------------------------------+*/
        /*| MPU6050                                                                 |*/
        /*+-------------------------------------------------------------------------+*/

        Spi accelerometerSpi (spiTx);
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

        // TIM3 is APB1 (42MHz) so CK_INT is 84MHz. Prescaler 84 -> counter runs @ 1MHz, period 100 gives us UEV frequency 10kHz
        HardwareTimer tim2 (TIM2, 84 - 1, 65536 - 1);
        Gpio encoderPins (GPIOA, GPIO_PIN_1 | GPIO_PIN_2, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_TIM2);
        InputCaptureChannel inputCapture2 (&tim2, 1, true);

        uint32_t prevCCR2 = 0;
        int encoderL = 0;
        int distanceL = 0;

        inputCapture2.setOnIrq ([&encoderL, &prevCCR2, &out, &distanceL] {
                encoderL = TIM2->CCR2 - prevCCR2;
                prevCCR2 = TIM2->CCR2;

                if (encoderL < 0) {
                        encoderL += 65536;
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
        int encoderR = 0;
        int distanceR = 0;

        inputCapture3.setOnIrq ([&encoderR, &prevCCR3, &out, &distanceR] {
                encoderR = TIM2->CCR3 - prevCCR3;
                prevCCR3 = TIM2->CCR3;

                if (encoderR < 0) {
                        encoderR += 65536;
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

#if 0
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
        //        d->print (int(mpu6050.getTemperature ()));
        d->print (int(lsm.getTemperature ()));
        d->print ("\n");

        HAL_Delay (100);

        Timer readout;
        Timer timeControl;

        uint32_t n = 0;

        const int readoutDelayMs = 1;
        const float dt = /*1.0 / readoutDelayMs*/ readoutDelayMs;
        const float iScale = dt, dScale = dt / 100.0;
        const float siScale = dt, sdScale = dt / 100.0;
        const float diScale = dt / 1000.0, ddScale = dt * 10;

        float error, prevError, integral, derivative;
        integral = derivative = prevError = 0;
        float kp, ki, kd;
        kp = 1000;
        ki = 15;
        kd = 200;
        float setPoint = 0;
        const float VERTICAL = 0.03;

        float sError, sPrevError, sIntegral, sDerivative;
        sIntegral = sDerivative = sPrevError = 0;
        float skp, ski, skd;
        skp = 2.0 / 10000.0;
        ski = 0;
        skd = 0;
        float sSetPoint = 0;

#ifndef SYMA_RX

#if 0
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

                        case 'P':
                                *skp = param / 10000.0;
                                break;

                        case 'I':
                                *ski = param / 10000.0;
                                break;

                        case 'D':
                                *skd = param / 10000.0;
                                break;

                        default:
                                break;
                        }

                        *sIntegral = *integral = 0;
                }

                virtual void onTx () {}

                virtual void onMaxRt ()
                {
                        Debug *d = Debug::singleton ();
                        d->print ("nRF MAX_RT!\n");
                }

                float *kp, *ki, *kd, *integral;
                float *skp, *ski, *skd, *sIntegral;
        } txCallback;

        txCallback.kp = &kp;
        txCallback.ki = &ki;
        txCallback.kd = &kd;
        txCallback.integral = &integral;
        txCallback.skp = &skp;
        txCallback.ski = &ski;
        txCallback.skd = &skd;
        txCallback.sIntegral = &sIntegral;

#ifdef RADIO
        nrfTx.setCallback (&txCallback);
#endif
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

        float ax, ay, az, gx, gy, gz;
        float ofx = 0, ofy = 0, ofz = 0; // Gyro offsets.
        bool sanityBlockade = false;

        // Delay for Madgwick to heat up.
        Timer startupTimer;
        startupTimer.start (5000);

        Timer blinkTimer;

        while (true) {
                if (readout.isExpired ()) {
                        static int i = 0;
                        float speed;

                        /*+-------------------------------------------------------------------------+*/
                        /*| Speed                                                                   |*/
                        /*+-------------------------------------------------------------------------+*/

                        int encoderSum = encoderL + encoderR;
                        speed = encoderSum / 2.0;

                        // To prevent comparing floats
                        if (encoderSum) {
                                speed = 100.0 / speed;
                        }

                        // Input capture frezes when it doesn't receive impulses from encoders when motors aren't spinning.
                        if (out == 0) {
                                speed = 0;
                        }

                        // Encoders in this motor cant sense the direction. So we assume the speed sign is ruled by the motors movement.
                        if (out > 0) {
                                speed = -speed;
                        }

                        int distance = -(distanceL + distanceR) / 2;

                        // sError = sSetPoint - speed;
                        sError = sSetPoint - distance;
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
                        setPoint = skp * sError + ski * sIntegral + skd * sDerivative;
                        sPrevError = sError;

                        /*+-------------------------------------------------------------------------+*/
                        /*| Angle                                                                   |*/
                        /*+-------------------------------------------------------------------------+*/

                        IGyroscope::GData gd = lsm.getGData ();
                        IAccelerometer::AData ad = lsm.getAData ();

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

                        //                        if (n % 100 == 0) {
                        //                                printf ("%d, %d, %d, %d, %d, %d, %d\n", int(pitch * 1000), int(gx * 100), int(gy *
                        //                                100), int(gz * 100),
                        //                                        int(ax * 100), int(ay * 100), int(az * 100));
                        //                        }

                        // PID
                        error = VERTICAL + setPoint - pitch;
                        integral += error * iScale;

                        // Simple anti integral windup.
                        if (integral > 25) {
                                integral = 25;
                        }
                        else if (integral < -25) {
                                integral = -25;
                        }

                        derivative = (error - prevError) / dScale;
                        out = kp * error + ki * integral + kd * derivative;
                        prevError = error;

                        if (sanityBlockade && (error < 0.002)) {
                                integral = 0;
                                out = 0;
                                sanityBlockade = false;
                        }

                        if (sanityBlockade || pitch > 0.6 || pitch < -0.6) {
                                out = 0;
                                sanityBlockade = true;
                        }

                        motorLeft.setSpeed (out);
                        motorRight.setSpeed (out);

                        // End
                        readout.start (readoutDelayMs);
                        ++timeCnt;

#if 1
                        if (++i % 50 == 0) {
                                uint8_t buf[32];
                                int inputValueI;
                                int errorI;
                                int integralI;
                                int derivativeI;
                                int outI;

                                // 50, 150, 250 ...
                                if (i % 100 != 0) {
                                        // Sending ints since reveiver runs on STM32F0 without fp, and cant printf floats.
                                        inputValueI = pitch * 1000;
                                        errorI = error * 1000;
                                        integralI = integral * 100;
                                        derivativeI = derivative * 100;
                                        outI = out * 100;
                                }
                                //                                 100, 200, 300
                                else {
                                        inputValueI = distance;
                                        // inputValueI = speed * 1000;
                                        errorI = sError;
                                        integralI = sIntegral * 100;
                                        derivativeI = sDerivative * 100;
                                        outI = setPoint * 10000;
                                }

                                memcpy (buf, &i, 4);
                                memcpy (buf + 4, &inputValueI, 4);
                                memcpy (buf + 8, &errorI, 4);
                                memcpy (buf + 12, &integralI, 4);
                                memcpy (buf + 16, &derivativeI, 4);
                                memcpy (buf + 20, &outI, 4);
#ifdef RADIO
                                nrfTx.transmit (buf, 24);
#endif
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
                if (blinkTimer.isExpired ()) {
                        blinkTimer.start (100);
                        static bool b = false;
                        led.set ((b = !b));
                }
        }
#endif
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
