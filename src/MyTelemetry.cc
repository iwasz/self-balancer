/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include "MyTelemetry.h"
#include "Debug.h"
#include <cstring>

const uint8_t MyTelemetry::ADDRESS[] = { 0xc2, 0xc2, 0xc2, 0xc2, 0xc2 };

MyTelemetry::MyTelemetry (Nrf24L01P *n, PidController *angleController, PidController *speedController)
    : nrf (n), angleController (angleController), speedController (speedController)
{
        nrf->setConfig (Nrf24L01P::MASK_TX_DS, true, Nrf24L01P::CRC_LEN_2);
        nrf->setTxAddress (ADDRESS, 5);
        nrf->readRegister (Nrf24L01P::TX_ADDR);
        nrf->setRxAddress (0, ADDRESS, 5);
        nrf->readRegister (Nrf24L01P::RX_ADDR_P0);
        nrf->setAutoAck (Nrf24L01P::ENAA_P0 | Nrf24L01P::ENAA_P1);
        nrf->setEnableDataPipe (Nrf24L01P::ERX_P0 | Nrf24L01P::ERX_P1);
        nrf->setAdressWidth (Nrf24L01P::WIDTH_5);
        nrf->setChannel (CHANNEL);
        nrf->setAutoRetransmit (Nrf24L01P::WAIT_1000_US, Nrf24L01P::RETRANSMIT_15);
        nrf->setDataRate (Nrf24L01P::MBPS_1, Nrf24L01P::DBM_0);
        nrf->setEnableDynamicPayload (Nrf24L01P::DPL_P0 | Nrf24L01P::DPL_P0);
        nrf->setFeature (Nrf24L01P::EN_DPL | Nrf24L01P::EN_ACK_PAY);
        nrf->powerUp (Nrf24L01P::TX);
        nrf->setCallback (this);
}

/*****************************************************************************/

void MyTelemetry::send (int i, float in1, float out1, float in2, float out2)
{
        if (i % 50 == 0) {

                uint8_t buf[32];
                int inputValueI;
                int errorI;
                int integralI;
                int derivativeI;
                int outI;

                // 50, 150, 250 ...
                if (i % 100 != 0 && angleController) {
                        // Sending ints since reveiver runs on STM32F0 without fp, and cant printf floats.
                        inputValueI = int(in1 * 1000);
                        errorI = int(angleController->getError () * 1000);
                        integralI = int(angleController->getIntegral () * 100);
                        derivativeI = int(angleController->getDerivative () * 100);
                        outI = int(out1);
                }
                // 100, 200, 300
                else if (speedController) {
                        inputValueI = int(in2);
                        errorI = int(speedController->getError ());
                        integralI = int(speedController->getIntegral () * 100);
                        derivativeI = int(speedController->getDerivative () * 100);
                        outI = int(out2 * 10000);
                }

                memcpy (buf, &i, 4);
                memcpy (buf + 4, &inputValueI, 4);
                memcpy (buf + 8, &errorI, 4);
                memcpy (buf + 12, &integralI, 4);
                memcpy (buf + 16, &derivativeI, 4);
                memcpy (buf + 20, &outI, 4);
                nrf->transmit (buf, 24);
        }
}

/*****************************************************************************/

void MyTelemetry::onRx (uint8_t *data, size_t len)
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
                angleController->setKp (param * 10.0f);
                break;

        case 'i':
                angleController->setKi (param / 10.0f);
                break;

        case 'd':
                angleController->setKd (param * 10.0f);
                break;

        case 'v':
                speedController->setConstOffset (param / 1000.0f);
                break;

        case 'P':
                speedController->setKp (param / 100000.0f);
                break;

        case 'I':
                speedController->setKi (param / 100000.0f);
                break;

        case 'D':
                speedController->setKd (param / 1000.0f);
                break;

        default:
                break;
        }

        angleController->setIntegral (0);
        speedController->setIntegral (0);
}

/*****************************************************************************/

void MyTelemetry::onMaxRt ()
{
        // Debug *d = Debug::singleton ();
        // d->print ("nRF MAX_RT!\n");
}
