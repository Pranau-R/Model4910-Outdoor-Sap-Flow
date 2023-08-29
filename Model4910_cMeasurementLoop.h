/*

Module: Model4910_cMeasurementLoop.h

Function:
        cMeasurementLoop definitions.

Copyright:
        See accompanying LICENSE file for copyright and license information.

Author:
        Pranau R, MCCI Corporation   July 2023

*/

#ifndef _Model4910_cMeasurementLoop_h_
# define _Model4910_cMeasurementLoop_h_

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Catena_FSM.h>
#include <Catena_Led.h>
#include <Catena_Log.h>
#include <Catena_Mx25v8035f.h>
#include <Catena_PollableInterface.h>
#include <Catena_Timer.h>
#include <Catena_TxBuffer.h>
#include <Catena.h>
#include <mcciadk_baselib.h>
#include <stdlib.h>
#include <Adafruit_BME280.h>
#include <mcci_ltr_329als.h>
#include <Catena_Totalizer.h>

#include <cstdint>

using namespace McciCatena;
using namespace Mcci_Ltr_329als;

extern McciCatena::Catena gCatena;
extern McciCatena::Catena::LoRaWAN gLoRaWAN;
extern McciCatena::StatusLed gLed;

constexpr uint8_t kBoosterPowerOn = D14;
constexpr uint8_t kExtVddEnable = D11;
constexpr uint8_t kPinPulse1P1 = A1;

constexpr uint8_t kIntialSize = 10;

// set true is when the interrupt is triggered
// extern volatile bool m_fInterrupt;

static inline void boostPowerOn(void)
    {
    pinMode(kBoosterPowerOn, OUTPUT);
    digitalWrite(kBoosterPowerOn, HIGH);
    }

static inline void boostPowerOff(void)
    {
    pinMode(kBoosterPowerOn, INPUT);
    digitalWrite(kBoosterPowerOn, LOW);
    }

static inline void extVddEnable(void)
    {
    pinMode(kExtVddEnable, OUTPUT);
    digitalWrite(kExtVddEnable, HIGH);
    }

static inline void extVddDisable(void)
    {
    pinMode(kExtVddEnable, INPUT);
    digitalWrite(kExtVddEnable, LOW);
    }

namespace McciModel4910 {

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

class cMeasurementBase
    {
    };

class cMeasurementFormat : public cMeasurementBase
    {
public:
    // buffer size for uplink data
    static constexpr size_t kTxBufferSize = 18;

    // message format
    static constexpr uint8_t kMessageFormat = 0x2b;

    enum class Flags : uint8_t
        {
        Vbat = 1 << 0,      // vBat
        Vcc = 1 << 1,       // vBus
        Boot = 1 << 2,      // boot count
        TPH = 1 << 3,       // BME280 (Env)
        Light = 1 << 4,     // LTR329 (Light)
        WattHours = 1 << 5,
        PulsesPerHour = 1<< 6,
        };

    // the structure of a measurement
    struct Measurement
        {
        //----------------
        // the subtypes:
        //----------------

        // environmental measurements
        struct Env
            {
            // temperature (in degrees C)
            float                   Temperature;
            // pressure (in millibars/hPa)
            float                   Pressure;
            // humidity (in % RH)
            float                   Humidity;
            };

        // ambient light measurements
        struct Light
            {
            float                   Lux;
            };

        // ambient light measurements
        struct Pulse
            {
            uint32_t                *Pulse1in;
            uint32_t                *Pulse1out;
            uint16_t                *FracPulse1In;
            uint16_t                *FracPulse1Out;
            };

        //---------------------------
        // the actual members as POD
        //---------------------------

        // flags of entries that are valid.
        Flags                       flags;
        // measured battery voltage, in volts
        float                       Vbat;
        // measured system Vdd voltage, in volts
        float                       Vsystem;
        // measured USB bus voltage, in volts.
        float                       Vbus;
        // boot count
        uint32_t                    BootCount;
        // environmental data
        Env                         env;
        // ambient light
        Light                       light;
        // Totalizer Pulse
        Pulse                       pulse;
        };
    };

class cMeasurementLoop : public McciCatena::cPollableObject
    {
public:
    // some parameters
    using MeasurementFormat = McciModel4910::cMeasurementFormat;
    using Measurement = MeasurementFormat::Measurement;
    using Flags = MeasurementFormat::Flags;
    static constexpr std::uint8_t kMessageFormat = MeasurementFormat::kMessageFormat;
    static constexpr std::uint8_t kUplinkPort = 1;

    enum OPERATING_FLAGS : uint32_t
        {
        fUnattended = 1 << 0,
        fManufacturingTest = 1 << 1,
        fConfirmedUplink = 1 << 16,
        fDisableDeepSleep = 1 << 17,
        fQuickLightSleep = 1 << 18,
        fDeepSleepTest = 1 << 19,
        };

    enum DebugFlags : std::uint32_t
        {
        kError      = 1 << 0,
        kWarning    = 1 << 1,
        kTrace      = 1 << 2,
        kInfo       = 1 << 3,
        };

    // constructor
    cMeasurementLoop(
            Mcci_Ltr_329als::Ltr_329als& ltr329
            )
        : m_Ltr(ltr329)
        , m_txCycleSec_Permanent(6 * 60)        // default uplink interval
        , m_txCycleSec(30)                      // initial uplink interval
        , m_txCycleCount(10)                    // initial count of fast uplinks
        , m_DebugFlags(DebugFlags(kError | kTrace))
        {};

    // neither copyable nor movable
    cMeasurementLoop(const cMeasurementLoop&) = delete;
    cMeasurementLoop& operator=(const cMeasurementLoop&) = delete;
    cMeasurementLoop(const cMeasurementLoop&&) = delete;
    cMeasurementLoop& operator=(const cMeasurementLoop&&) = delete;

    enum class State : std::uint8_t
        {
        stNoChange = 0, // this name must be present: indicates "no change of state"
        stInitial,      // this name must be present: it's the starting state.
        stInactive,     // parked; not doing anything.
        stSleeping,     // active; sleeping between measurements
        stWarmup,       // transition from inactive to measure, get some data.
        stMeasure,      // take measurents
        stTransmit,     // transmit data
        stFinal,        // this name must be present, it's the terminal state.
        };

    static constexpr const char *getStateName(State s)
        {
        switch (s)
            {
            case State::stNoChange: return "stNoChange";
            case State::stInitial:  return "stInitial";
            case State::stInactive: return "stInactive";
            case State::stSleeping: return "stSleeping";
            case State::stWarmup:   return "stWarmup";
            case State::stMeasure:  return "stMeasure";
            case State::stTransmit: return "stTransmit";
            case State::stFinal:    return "stFinal";
            default:                return "<<unknown>>";
            }
        }

    // concrete type for uplink data buffer
    using TxBuffer_t = McciCatena::AbstractTxBuffer_t<MeasurementFormat::kTxBufferSize>;

    // initialize measurement FSM.
    void begin();
    void end();
    void setTxCycleTime(
        std::uint32_t txCycleSec,
        std::uint32_t txCycleCount
        )
        {
        this->m_txCycleSec = txCycleSec;
        this->m_txCycleCount = txCycleCount;

        this->m_UplinkTimer.setInterval(txCycleSec * 1000);
        if (this->m_UplinkTimer.peekTicks() != 0)
            this->m_fsm.eval();
        }

    std::uint32_t getTxCycleTime()
        {
        return this->m_txCycleSec;
        }

    void setBme280(bool fEnable)
        {
        this->m_fBme280 = fEnable;
        }

    virtual void poll() override;

    void setVbus(float Vbus)
        {
        // set threshold value as 4.0V as there is reverse voltage
        // in vbus(~3.5V) while powered from battery in 4801.
        this->m_fUsbPower = (Vbus > 4.0f) ? true : false;
        }

    // Function prototype to set up interrupts
    void setupInterrupts();

    // Function prototype to disable interrupts
    void disableInterrupts();

    // request that the measurement loop be active/inactive
    void requestActive(bool fEnable);

    // return true if a given debug mask is enabled.
    bool isTraceEnabled(DebugFlags mask) const
        {
        return this->m_DebugFlags & mask;
        }

    // register an additional SPI for sleep/resume
    // can be called before begin().
    void registerSecondSpi(SPIClass *pSpi)
        {
        this->m_pSPI2 = pSpi;
        }
private:
    // sleep handling
    void sleep();
    bool checkDeepSleep();
    void doSleepAlert(bool fDeepSleep);
    void doDeepSleep();
    void deepSleepPrepare();
    void deepSleepRecovery();

    // read data
    void updateSynchronousMeasurements();
    void updateLightMeasurements();
    void resetMeasurements();
    static void measureTotalizer();
    void resizeArray(uint32_t* buf, uint64_t bufSize);
    void resizeArray(uint16_t* buf, uint64_t bufSize);

    // function for scaling pulse data
    static uint16_t dNdT_getFrac(uint32_t deltaC,uint32_t delta_ms);

    // telemetry handling.
    void fillTxBuffer(TxBuffer_t &b, Measurement const & mData);
    void startTransmission(TxBuffer_t &b);
    void sendBufferDone(bool fSuccess);

    bool txComplete()
        {
        return this->m_txcomplete;
        }

    void updateTxCycleTime();

    // set the timer
    void setTimer(std::uint32_t ms);
    // clear the timer
    void clearTimer();
    // test (and clear) the timed-out flag.
    bool timedOut();

    // instance data
private:
    McciCatena::cFSM<cMeasurementLoop, State> m_fsm;
    // evaluate the control FSM.
    State fsmDispatch(State currentState, bool fEntry);

    // Env sensor
    Adafruit_BME280                 m_BME280;

    // Light Sensor
    Mcci_Ltr_329als::Ltr_329als&    m_Ltr;
    Mcci_Ltr_329als_Regs::AlsContr_t    m_AlsCtrl;

    // Totalizer
    cTotalizer                      m_Pulse1P1;
    uint32_t                        pulse1in_dc;
    uint32_t                        pulse1in_dt;
    uint32_t                        pulse1out_dc;
    uint32_t                        pulse1out_dt;

    // Additional Totalizer parameters
    uint64_t                        pulse1InBufSize;
    uint64_t                        pulse1OutBufSize;
    uint64_t                        fracPulse1InBufSize;
    uint64_t                        fracPulse1OutBufSize;
    uint64_t                        pulse1InBufIndex;
    uint64_t                        pulse1OutBufIndex;
    uint64_t                        fracPulse1InBufIndex;
    uint64_t                        fracPulse1OutBufIndex;
    uint32_t                        interrupCounter;

    // second SPI class
    SPIClass                        *m_pSPI2;

    // debug flags
    DebugFlags                      m_DebugFlags;

    // true if object is registered for polling.
    bool                            m_registered : 1;
    // true if object is running.
    bool                            m_running : 1;
    // true to request exit
    bool                            m_exit : 1;
    // true if in active uplink mode, false otehrwise.
    bool                            m_active : 1;

    // set true to request transition to active uplink mode; cleared by FSM
    bool                            m_rqActive : 1;
    // set true to request transition to inactive uplink mode; cleared by FSM
    bool                            m_rqInactive : 1;

    // set true if event timer times out
    bool                            m_fTimerEvent : 1;
    // set true while evenet timer is active.
    bool                            m_fTimerActive : 1;
    // set true if USB power is present.
    bool                            m_fUsbPower : 1;
    // set true if BME280 is present
    bool                            m_fBme280 : 1;
    // set true if LTR329 is present
    bool                            m_fLtr329: 1;
    // set true if SI1133/LTR329 detects low light
    bool                            m_fLowLight: 1;
    // set true if hardware error in LTR329 is present
    bool                            m_fHardError: 1;
    // set true is the contact sensor is present
    bool                            m_fPulse1: 1;


    // set true while a transmit is pending.
    bool                            m_txpending : 1;
    // set true when a transmit completes.
    bool                            m_txcomplete : 1;
    // set true when a transmit complete with an error.
    bool                            m_txerr : 1;
    // set true when we've printed how we plan to sleep
    bool                            m_fPrintedSleeping : 1;
    // set true when SPI2 is active
    bool                            m_fSpi2Active: 1;

    // uplink time control
    McciCatena::cTimer              m_UplinkTimer;
    std::uint32_t                   m_txCycleSec;
    std::uint32_t                   m_txCycleCount;
    std::uint32_t                   m_txCycleSec_Permanent;

    // simple timer for timing-out sensors.
    std::uint32_t                   m_timer_start;
    std::uint32_t                   m_timer_delay;

    // the current measurement
    Measurement                     m_data;

    TxBuffer_t                      m_FileTxBuffer;
    };

//
// operator overloads for ORing structured flags
//
static constexpr cMeasurementLoop::Flags operator| (const cMeasurementLoop::Flags lhs, const cMeasurementLoop::Flags rhs)
        {
        return cMeasurementLoop::Flags(uint8_t(lhs) | uint8_t(rhs));
        };

static constexpr cMeasurementLoop::Flags operator& (const cMeasurementLoop::Flags lhs, const cMeasurementLoop::Flags rhs)
        {
        return cMeasurementLoop::Flags(uint8_t(lhs) & uint8_t(rhs));
        };

static cMeasurementLoop::Flags operator|= (cMeasurementLoop::Flags &lhs, const cMeasurementLoop::Flags &rhs)
        {
        lhs = lhs | rhs;
        return lhs;
        };

} // namespace McciModel4910

#endif /* _Model4910_cMeasurementLoop_h_ */