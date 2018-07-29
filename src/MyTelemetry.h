/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#ifndef MYTELEMETRY_H
#define MYTELEMETRY_H

#include "control/PidController.h"
#include "rf/Nrf24L01P.h"
#include <cstdint>

class MyTelemetry : public Nrf24L01PCallback {
public:
        static const uint8_t ADDRESS[];
        const uint8_t CHANNEL = 100;

        MyTelemetry (Nrf24L01P *n, PidController *angleController, PidController *speedController);

        void send (int i, float in1, float out1, float in2, float out2);

        virtual void onRx (uint8_t *data, size_t len);
        virtual void onTx () {}
        virtual void onMaxRt ();

private:
        Nrf24L01P *nrf;
        PidController *angleController;
        PidController *speedController;
};

#endif // MYTELEMETRY_H
