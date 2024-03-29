/*

Module: Model4910_cMeasurementLoop_fillBuffer.cpp

Function:
        Class for transmitting accumulated measurements.

Copyright:
        See accompanying LICENSE file for copyright and license information.

Author:
        Pranau R, MCCI Corporation   July 2023

*/

#include <Catena_TxBuffer.h>

#include "Model4910_cMeasurementLoop.h"

#include <arduino_lmic.h>

using namespace McciCatena;
using namespace McciModel4910;

/*

Name:   McciModel4910::cMeasurementLoop::fillTxBuffer()

Function:
        Prepare a messages in a TxBuffer with data from current measurements.

Definition:
        void McciModel4910::cMeasurementLoop::fillTxBuffer(
                cMeasurementLoop::TxBuffer_t& b
                );

Description:
        A format 0x2b message is prepared from the data in the cMeasurementLoop
        object.

*/

void
cMeasurementLoop::fillTxBuffer(
    cMeasurementLoop::TxBuffer_t& b, Measurement const &mData
    )
    {
    gLed.Set(McciCatena::LedPattern::Measuring);

    // initialize the message buffer to an empty state
    b.begin();

    // insert format byte
    b.put(kMessageFormat);

    // the flags in Measurement correspond to the over-the-air flags.
    b.put(std::uint8_t(this->m_data.flags));

    // send Vbat
    if ((this->m_data.flags & Flags::Vbat) !=  Flags(0))
        {
        float Vbat = mData.Vbat;
        gCatena.SafePrintf("Vbat:    %d mV\n", (int) (Vbat * 1000.0f));
        b.putV(Vbat);
        }

    // send Vdd if we can measure it.

    // Vbus is sent as 5000 * v
    if ((this->m_data.flags & Flags::Vcc) !=  Flags(0))
        {
        float Vbus = mData.Vbus;
        gCatena.SafePrintf("Vbus:    %d mV\n", (int) (Vbus * 1000.0f));
        b.putV(Vbus);
        }

    // send boot count
    if ((this->m_data.flags & Flags::Boot) !=  Flags(0))
        {
        b.putBootCountLsb(mData.BootCount);
        }

    if ((this->m_data.flags &  Flags::TPH) !=  Flags(0))
        {
        gCatena.SafePrintf(
                "BME280:  T: %d P: %d RH: %d\n",
                (int) mData.env.Temperature,
                (int) mData.env.Pressure,
                (int) mData.env.Humidity
                );
        b.putT(mData.env.Temperature);
        b.putP(mData.env.Pressure);
        b.putRH(mData.env.Humidity);
        }

    // put light
    if ((this->m_data.flags & Flags::Light) != Flags(0))
        {
        if (this->m_fLtr329)
            {
            gCatena.SafePrintf(
                "Ltr329:  %d Lux\n",
                (int) mData.light.Lux
                );

            b.put3f(mData.light.Lux);
            }
        }

    if ((this->m_data.flags & Flags::WattHours) != Flags(0))
        {
        gCatena.SafePrintf(
            "Pulses:  Pin1: "
            );
        for (size_t i = 0; i < this->pulse1InBufIndex; i++)
            {
            gCatena.SafePrintf(
                "%u, ",
                mData.pulse.Pulse1in[this->pulse1InBufIndex]
                );
            }
        gCatena.SafePrintf(
            "\nPulses:  Pin2: "
            );
        for (size_t i = 0; i < this->pulse1OutBufIndex; i++)
            {
            gCatena.SafePrintf(
                "%u, ",
                mData.pulse.Pulse1out[this->pulse1OutBufIndex]
                );
            }

        gCatena.SafePrintf(
            "\n"
            );

        b.putWH(mData.pulse.Pulse1in[this->pulse1InBufIndex]);
        b.putWH(mData.pulse.Pulse1out[this->pulse1OutBufIndex]);
        }

    if ((this->m_data.flags & Flags::PulsesPerHour) != Flags(0))
        {
        gCatena.SafePrintf(
            "Power:   IN: %u/%u ",
            this->pulse1in_dc,
            this->pulse1in_dt);

        for (size_t i = 0; i < this->fracPulse1InBufIndex; i++)
            {
            gCatena.SafePrintf(
                "%04x, ",
                mData.pulse.FracPulse1In[this->fracPulse1InBufIndex]
                );
            }

        gCatena.SafePrintf(
            "\n"
            );

        gCatena.SafePrintf(
            "Power:   OUT: %u/%u ",
            this->pulse1out_dc,
            this->pulse1out_dt);

        for (size_t i = 0; i < this->fracPulse1OutBufIndex; i++)
            {
            gCatena.SafePrintf(
                "%04x, ",
                mData.pulse.FracPulse1Out[this->fracPulse1OutBufIndex]
                );
            }

         gCatena.SafePrintf(
             "\n"
             );

        b.putPulseFraction(mData.pulse.FracPulse1In[this->fracPulse1InBufIndex]);
        b.putPulseFraction(mData.pulse.FracPulse1Out[this->fracPulse1OutBufIndex]);
        }
    gLed.Set(McciCatena::LedPattern::Off);
    }