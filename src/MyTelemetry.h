/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#ifndef MYTELEMETRY_H
#define MYTELEMETRY_H

#include "rf/Nrf24L01P.h"
#include <cstdint>

class MyTelemetry : public Nrf24L01PCallback {
public:
        static const uint8_t ADDRESS[];
        const uint8_t CHANNEL = 100;

        MyTelemetry (Nrf24L01P *n);

        void send (int i, float pitch, float error, float integral, float derivative, int out, int distance, float sError, float sIntegral,
                   float sDerivative, float setPoint);

        virtual void onRx (uint8_t *data, size_t len);
        virtual void onTx () {}
        virtual void onMaxRt ();

        float *kp, *ki, *kd, *integral;
        float *skp, *ski, *skd, *sIntegral;

private:
        Nrf24L01P *nrf;
};

#endif // MYTELEMETRY_H
