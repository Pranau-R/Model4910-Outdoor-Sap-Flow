/*

Module: Model4910_cMeasurementLoop.cpp

Function:
        Class for transmitting accumulated measurements.

Copyright:
        See accompanying LICENSE file for copyright and license information.

Author:
        Pranau R, MCCI Corporation   July 2023

*/

#include "Model4910_cMeasurementLoop.h"

#include <arduino_lmic.h>
#include <Model4910-Outdoor-Sap-Flow.h>

using namespace McciModel4910;
using namespace McciCatena;
volatile bool m_fInterrupt;

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

void cMeasurementLoop::begin()
    {
    // register for polling.
    if (! this->m_registered)
        {
        this->m_registered = true;

        gCatena.registerObject(this);

        this->m_UplinkTimer.begin(this->m_txCycleSec * 1000);
        }

    Wire.begin();
    if (this->m_BME280.begin(BME280_ADDRESS, Adafruit_BME280::OPERATING_MODE::Sleep))
        {
        this->m_fBme280 = true;
        gCatena.SafePrintf("BME280 found\n");
        }
    else
        {
        this->m_fBme280 = false;
        gCatena.SafePrintf("No BME280 found: check wiring\n");
        }

    if (this->m_Ltr.begin())
        {
        this->m_fHardError = false;
        this->m_fLtr329 = true;
        gCatena.SafePrintf("LTR329 found\n");
        }
    else
        {
        this->m_fLtr329 = false;
        gCatena.SafePrintf("No LTR329 found: check hardware\n");
        }

    extVddEnable();

    if (m_Pulse1P1.begin(kPinPulse1P1))
        {
        this->pulse1InBufSize = kIntialSize;
        this->pulse1OutBufSize = kIntialSize;
        this->fracPulse1InBufSize = kIntialSize;
        this->fracPulse1OutBufSize = kIntialSize;
        this->pulse1InBufIndex = 0;
        this->pulse1OutBufIndex = 0;
        this->fracPulse1InBufIndex = 0;
        this->fracPulse1OutBufIndex = 0;
        this->interrupCounter = 0;

        this->m_fPulse1 = true;
        gCatena.SafePrintf("Pulse totalization started\n");
        }
    else
        {
        this->m_fPulse1 = false;
        gCatena.SafePrintf("Pulse totalization failed to start: defective board or incorrect platform\n");
        }

    // for gas meter, we need a pull-up
    pinMode(kPinPulse1P1, INPUT_PULLUP);
    // it's a mechanical switch, be generous with debounce.
    m_Pulse1P1.setDebounce(20);

    // start (or restart) the FSM.
    if (! this->m_running)
        {
        this->m_exit = false;
        this->m_fsm.init(*this, &cMeasurementLoop::fsmDispatch);
        }
    }

void cMeasurementLoop::end()
    {
    if (this->m_running)
        {
        this->m_exit = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::requestActive(bool fEnable)
    {
    if (fEnable)
        this->m_rqActive = true;
    else
        this->m_rqInactive = true;

    this->m_fsm.eval();
    }

uint16_t cMeasurementLoop::dNdT_getFrac(uint32_t deltaC,uint32_t delta_ms)
    {
    if (delta_ms == 0 || deltaC == 0)
        return 0;

    // this is a value in [0,1)
    float dNdTperHour = float(deltaC * 250) / float(delta_ms);

    if (dNdTperHour <= 0)
        return 0;
    else if (dNdTperHour >= 1)
        return 0xFFFF;
    else
        {
        int iExp;
        float normalValue;
        normalValue = frexpf(dNdTperHour, &iExp);

        // dNdTperHour is supposed to be in [0..1), so useful exp
        // is [0..-15]
        iExp += 15;
        if (iExp < 0)
            iExp = 0;
        if (iExp > 15)
            return 0xFFFF;

        return (uint16_t)((iExp << 12u) + (unsigned) scalbnf(normalValue, 12));
        }
    }

void cMeasurementLoop::measureTotalizer()
    {
    m_fInterrupt = true;
    }

// Function to set up interrupts
void cMeasurementLoop::setupInterrupts()
    {
    // Attach the ISR to the desired pin and configure the interrupt type
    attachInterrupt(digitalPinToInterrupt(kPinPulse1P1), measureTotalizer, HIGH);
    }

void cMeasurementLoop::disableInterrupts()
    {
    detachInterrupt(digitalPinToInterrupt(kPinPulse1P1));
    }

void cMeasurementLoop::resizeArray(uint16_t* buf, uint64_t bufSize)
    {
    this->resizeArray((uint32_t*)buf, bufSize);
    }

void cMeasurementLoop::resizeArray(uint32_t* buf, uint64_t bufSize)
    {
    size_t new_size = bufSize * 2;
    uint32_t* newDataStorage = new uint32_t[new_size];
    for (size_t i = 0; i < bufSize; i++)
        {
        newDataStorage[i] = buf[i];
        }

    bufSize = new_size;
    delete[] buf;
    buf = newDataStorage;
    }

cMeasurementLoop::State
cMeasurementLoop::fsmDispatch(
    cMeasurementLoop::State currentState,
    bool fEntry
    )
    {
    State newState = State::stNoChange;

    if (fEntry && this->isTraceEnabled(this->DebugFlags::kTrace))
        {
        gCatena.SafePrintf("cMeasurementLoop::fsmDispatch: enter %s\n",
                this->getStateName(currentState)
                );
        }

    switch (currentState)
        {
    case State::stInitial:
        newState = State::stInactive;
        this->resetMeasurements();
        break;

    case State::stInactive:
        if (fEntry)
            {
            // turn off anything that should be off while idling.
            }
        if (this->m_rqActive)
            {
            // when going active manually, start the measurement
            // cycle immediately.
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = true;
            this->m_UplinkTimer.retrigger();
            newState = State::stWarmup;
            }
        break;

    case State::stSleeping:
        if (fEntry)
            {
            // set the LEDs to flash accordingly.
            gLed.Set(McciCatena::LedPattern::Sleeping);
            }

        if (this->m_rqInactive)
            {
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = false;
            newState = State::stInactive;
            }
        else if (this->m_UplinkTimer.isready())
            newState = State::stMeasure;
        else if (this->m_UplinkTimer.getRemaining() > 1500)
            {
            this->pulse1InBufSize = kIntialSize;
            this->pulse1OutBufSize = kIntialSize;
            this->fracPulse1InBufSize = kIntialSize;
            this->fracPulse1OutBufSize = kIntialSize;
            this->pulse1InBufIndex = 0;
            this->pulse1OutBufIndex = 0;
            this->fracPulse1InBufIndex = 0;
            this->fracPulse1OutBufIndex = 0;
            this->interrupCounter = 0;

            this->setupInterrupts();
            this->sleep();
            }
        break;

    // get some data. This is only called while booting up.
    case State::stWarmup:
        if (fEntry)
            {
            //start the timer
            this->setTimer(5 * 1000);
            }
        if (this->timedOut())
            newState = State::stMeasure;
        break;

    // fill in the measurement
    case State::stMeasure:
        if (fEntry)
            {
            this->disableInterrupts();
            this->updateSynchronousMeasurements();
            this->setTimer(1000);
            newState = State::stTransmit;
            }
        if (this->m_fLtr329)
            {
            if (this->m_Ltr.startSingleMeasurement())
                {
                this->updateLightMeasurements();
                newState = State::stTransmit;
                }
            else if (this->timedOut())
                {
                this->m_fHardError = true;
                newState = State::stTransmit;
                }
            else
                this->m_fHardError = true;
            }
        break;

    case State::stTransmit:
        if (fEntry)
            {
            TxBuffer_t b;
            this->fillTxBuffer(b, this->m_data);

            this->m_FileTxBuffer.begin();
            for (auto i = 0; i < b.getn(); ++i)
                this->m_FileTxBuffer.put(b.getbase()[i]);

            this->resetMeasurements();
            this->startTransmission(b);

            while (true)
                {
                std::uint32_t lmicCheckTime;
                os_runloop_once();
                lmicCheckTime = this->m_UplinkTimer.getRemaining();

                // if we can sleep, break out of this loop
                // NOTE: if that the TX is not ready, LMIC is still waiting for interrupt
                if (! os_queryTimeCriticalJobs(ms2osticks(lmicCheckTime)) && LMIC_queryTxReady())
                    {
                    break;
                    }

                gCatena.poll();
                yield();
                }
            }
        if (this->txComplete())
            {
            newState = State::stSleeping;

            // calculate the new sleep interval.
            this->updateTxCycleTime();
            }
        break;

    case State::stFinal:
        break;

    default:
        break;
        }

    return newState;
    }

/****************************************************************************\
|
|   Take a measurement
|
\****************************************************************************/

void cMeasurementLoop::resetMeasurements()
    {
    memset((void *) &this->m_data, 0, sizeof(this->m_data));
    this->m_data.flags = Flags(0);
    }

void cMeasurementLoop::updateSynchronousMeasurements()
    {
    this->m_data.Vbat = gCatena.ReadVbat();
    this->m_data.flags |= Flags::Vbat;

    // enable boost regulator if no USB power and VBat is less than 3.1V
    if (!m_fUsbPower && (this->m_data.Vbat < 3.10f))
        {
        boostPowerOn();
        delay(50);
        }

    this->m_data.Vbus = gCatena.ReadVbus();
    this->m_data.flags |= Flags::Vcc;

    if (gCatena.getBootCount(this->m_data.BootCount))
        {
        this->m_data.flags |= Flags::Boot;
        }

    if (this->m_fBme280)
        {
        auto m = this->m_BME280.readTemperaturePressureHumidity();
        this->m_data.env.Temperature = m.Temperature;
        this->m_data.env.Pressure = m.Pressure;
        this->m_data.env.Humidity = m.Humidity;
        this->m_data.flags |= Flags::TPH;
        }

    boostPowerOff();
    }

void cMeasurementLoop::updateLightMeasurements()
    {
    if (this->m_fLtr329)
        {
        float currentLux;
        bool fHardError;

        static constexpr float kMax_Gain_96 = 640.0f;
        static constexpr float kMax_Gain_48 = 1280.0f;
        static constexpr float kMax_Gain_8 = 7936.0f;
        static constexpr float kMax_Gain_4 = 16128.0f;
        static constexpr float kMax_Gain_2 = 32512.0f;
        static constexpr float kMax_Gain_1 = 65535.0f;

        while (! this->m_Ltr.queryReady(fHardError))
            {
            if (fHardError)
                break;
            }

        if (fHardError)
            {
            this->m_fHardError = true;
            if (gLog.isEnabled(gLog.DebugFlags::kError))
                gLog.printf(
                    gLog.kAlways,
                    "LTR329 queryReady failed: status %s(%u)\n",
                    this->m_Ltr.getLastErrorName(),
                    unsigned(this->m_Ltr.getLastError())
                    );
            }
        else
            {
            currentLux = this->m_Ltr.getLux();

            this->m_data.flags |= Flags::Light;
            this->m_data.light.Lux = currentLux;

            if (currentLux <= kMax_Gain_96)
                m_AlsCtrl.setGain(96);
            else if (currentLux <= kMax_Gain_48)
                m_AlsCtrl.setGain(48);
            else if (currentLux <= kMax_Gain_8)
                m_AlsCtrl.setGain(8);
            else if (currentLux <= kMax_Gain_4)
                m_AlsCtrl.setGain(4);
            else if (currentLux <= kMax_Gain_2)
                m_AlsCtrl.setGain(2);
            else
                m_AlsCtrl.setGain(1);

            if (currentLux <= 100)
                this->m_fLowLight = true;
            else
                this->m_fLowLight = false;
            }
        }
    }

/****************************************************************************\
|
|   Start uplink of data
|
\****************************************************************************/

void cMeasurementLoop::startTransmission(
    cMeasurementLoop::TxBuffer_t &b
    )
    {
    auto const savedLed = gLed.Set(McciCatena::LedPattern::Off);
    gLed.Set(McciCatena::LedPattern::Sending);

    // by using a lambda, we can access the private contents
    auto sendBufferDoneCb =
        [](void *pClientData, bool fSuccess)
            {
            auto const pThis = (cMeasurementLoop *)pClientData;
            pThis->m_txpending = false;
            pThis->m_txcomplete = true;
            pThis->m_txerr = ! fSuccess;
            pThis->m_fsm.eval();
            };

    bool fConfirmed = false;
    if (gCatena.GetOperatingFlags() &
        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fConfirmedUplink))
        {
        gCatena.SafePrintf("requesting confirmed tx\n");
        fConfirmed = true;
        }

    this->m_txpending = true;
    this->m_txcomplete = this->m_txerr = false;

    if (! gLoRaWAN.SendBuffer(b.getbase(), b.getn(), sendBufferDoneCb, (void *)this, fConfirmed, kUplinkPort))
        {
        // uplink wasn't launched.
        this->m_txcomplete = true;
        this->m_txerr = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::sendBufferDone(bool fSuccess)
    {
    this->m_txpending = false;
    this->m_txcomplete = true;
    this->m_txerr = ! fSuccess;
    this->m_fsm.eval();
    }

/****************************************************************************\
|
|   The Polling function --
|
\****************************************************************************/

void cMeasurementLoop::poll()
    {
    bool fEvent;

    // no need to evaluate unless something happens.
    fEvent = false;

    // if we're not active, and no request, nothing to do.
    if (! this->m_active)
        {
        if (! this->m_rqActive)
            return;

        // we're asked to go active. We'll want to eval.
        fEvent = true;
        }

    if (m_fInterrupt == true)
        {
        m_fInterrupt = false;
        gCatena.SafePrintf("Trigger Counter: %d\n", this->interrupCounter);
        this->interrupCounter += 1;

        if (this->pulse1InBufIndex >= this->pulse1InBufSize)
            {
            this->resizeArray(this->m_data.pulse.Pulse1in, this->pulse1InBufSize);
            }

        this->m_data.pulse.Pulse1in[this->pulse1InBufIndex] = this->m_Pulse1P1.getcurrent();
        this->pulse1InBufIndex += 1;
        this->m_Pulse1P1.getDeltaCountAndTime(this->pulse1in_dc, this->pulse1in_dt);
        this->m_Pulse1P1.setReference();

        if (this->pulse1OutBufIndex >= this->pulse1OutBufSize)
            {
            this->resizeArray(this->m_data.pulse.Pulse1out, this->pulse1OutBufSize);
            }

        this->m_data.pulse.Pulse1out[this->pulse1OutBufIndex] = 0;
        this->pulse1OutBufIndex += 1;
        this->pulse1out_dc = 0;
        this->pulse1out_dt = this->pulse1in_dt;

        this->m_data.flags |= Flags::WattHours;

        if (this->fracPulse1InBufIndex >= this->fracPulse1InBufSize)
            {
            this->resizeArray(this->m_data.pulse.FracPulse1In, this->fracPulse1InBufSize);
            }
        m_data.pulse.FracPulse1In[this->fracPulse1InBufIndex] = this->dNdT_getFrac(this->pulse1in_dc, this->pulse1in_dt);
        this->fracPulse1InBufIndex += 1;

        if (this->fracPulse1OutBufIndex >= this->fracPulse1OutBufSize)
            {
            this->resizeArray(this->m_data.pulse.FracPulse1Out, this->fracPulse1OutBufSize);
            }
        m_data.pulse.FracPulse1Out[this->fracPulse1OutBufIndex] = this->dNdT_getFrac(this->pulse1out_dc, this->pulse1out_dt);
        this->fracPulse1OutBufIndex += 1;

        this->m_data.flags |= Flags::PulsesPerHour;
        }

    if (this->m_fTimerActive)
        {
        if ((millis() - this->m_timer_start) >= this->m_timer_delay)
            {
            this->m_fTimerActive = false;
            this->m_fTimerEvent = true;
            fEvent = true;
            }
        }

    // check the transmit time.
    if (this->m_UplinkTimer.peekTicks() != 0)
        {
        fEvent = true;
        }

    if (fEvent)
        this->m_fsm.eval();

    this->m_data.Vbus = gCatena.ReadVbus();
    setVbus(this->m_data.Vbus);
    }

/****************************************************************************\
|
|   Update the TxCycle count.
|
\****************************************************************************/

void cMeasurementLoop::updateTxCycleTime()
    {
    auto txCycleCount = this->m_txCycleCount;

    // update the sleep parameters
    if (txCycleCount > 1)
        {
        // values greater than one are decremented and ultimately reset to default.
        this->m_txCycleCount = txCycleCount - 1;
        }
    else if (txCycleCount == 1)
        {
        // it's now one (otherwise we couldn't be here.)
        gCatena.SafePrintf("resetting tx cycle to default: %u\n", this->m_txCycleSec_Permanent);

        this->setTxCycleTime(this->m_txCycleSec_Permanent, 0);
        }
    else
        {
        // it's zero. Leave it alone.
        }
    }

/****************************************************************************\
|
|   Handle sleep between measurements
|
\****************************************************************************/

void cMeasurementLoop::sleep()
    {
    const bool fDeepSleep = checkDeepSleep();

    if (! this->m_fPrintedSleeping)
        this->doSleepAlert(fDeepSleep);

    if (fDeepSleep)
        this->doDeepSleep();
    }

// for now, we simply don't allow deep sleep. In the future might want to
// use interrupts on activity to wake us up; then go back to sleep when we've
// seen nothing for a while.
bool cMeasurementLoop::checkDeepSleep()
    {
    bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
            static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    bool fDeepSleep;
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (sleepInterval < 2)
        fDeepSleep = false;
    else if (fDeepSleepTest)
        {
        fDeepSleep = true;
        }
#ifdef USBCON
    else if (Serial.dtr())
        {
        fDeepSleep = false;
        }
#endif
    else if (gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDisableDeepSleep))
        {
        fDeepSleep = false;
        }
    else if ((gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)) != 0)
        {
        fDeepSleep = true;
        }
    else
        {
        fDeepSleep = false;
        }

    return fDeepSleep;
    }

void cMeasurementLoop::doSleepAlert(bool fDeepSleep)
    {
    this->m_fPrintedSleeping = true;

    if (fDeepSleep)
        {
        bool const fDeepSleepTest =
                gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
        const uint32_t deepSleepDelay = fDeepSleepTest ? 10 : 30;

        gCatena.SafePrintf("using deep sleep in %u secs"
#ifdef USBCON
                            " (USB will disconnect while asleep)"
#endif
                            ": ",
                            deepSleepDelay
                            );

        // sleep and print
        gLed.Set(McciCatena::LedPattern::TwoShort);

        for (auto n = deepSleepDelay; n > 0; --n)
            {
            uint32_t tNow = millis();

            while (uint32_t(millis() - tNow) < 1000)
                {
                gCatena.poll();
                yield();
                }
            gCatena.SafePrintf(".");
            }
        gCatena.SafePrintf("\nStarting deep sleep.\n");
        uint32_t tNow = millis();
        while (uint32_t(millis() - tNow) < 100)
            {
            gCatena.poll();
            yield();
            }
        }
    else
        gCatena.SafePrintf("using light sleep\n");
    }

void cMeasurementLoop::doDeepSleep()
    {
    // bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
    //             static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (sleepInterval == 0)
        return;

    /* ok... now it's time for a deep sleep */
    gLed.Set(McciCatena::LedPattern::Off);
    this->deepSleepPrepare();

    /* sleep */
    gCatena.Sleep(sleepInterval);

    /* recover from sleep */
    this->deepSleepRecovery();

    /* and now... we're awake again. trigger another measurement */
    this->m_fsm.eval();
    }

//
// call this after waking up from a long (> 15 minute) sleep to correct for LMIC sleep defect
// This should be done after updating micros() and updating LMIC's idea of time based on
// the sleep time.
//
void fixLmicTimeCalculationAfterWakeup(void)
    {
    ostime_t const now = os_getTime();
    // just tell the LMIC that we're available *now*.
    LMIC.globalDutyAvail = now;
    // no need to randomize
    // for EU-like, we need to reset all the channel avail times to "now"
#if CFG_LMIC_EU_like
    for (unsigned i = 0; i < MAX_BANDS; ++i)
        {
        LMIC.bands[i].avail = now;
        }
#endif
    }

void cMeasurementLoop::deepSleepPrepare(void)
    {
    Serial.end();
    Wire.end();
    SPI.end();
    if (this->m_pSPI2 && this->m_fSpi2Active)
        {
        this->m_pSPI2->end();
        this->m_fSpi2Active = false;
        }
    }

void cMeasurementLoop::deepSleepRecovery(void)
    {
    Serial.begin();
    Wire.begin();
    SPI.begin();

    // enable boost regulator if no USB power and VBat is less than 3.1V
    if (!m_fUsbPower && (this->m_data.Vbat < 3.10f))
        {
        boostPowerOn();
        }

    fixLmicTimeCalculationAfterWakeup();
    }

/****************************************************************************\
|
|  Time-out asynchronous measurements.
|
\****************************************************************************/

// set the timer
void cMeasurementLoop::setTimer(std::uint32_t ms)
    {
    this->m_timer_start = millis();
    this->m_timer_delay = ms;
    this->m_fTimerActive = true;
    this->m_fTimerEvent = false;
    }

void cMeasurementLoop::clearTimer()
    {
    this->m_fTimerActive = false;
    this->m_fTimerEvent = false;
    }

bool cMeasurementLoop::timedOut()
    {
    bool result = this->m_fTimerEvent;
    this->m_fTimerEvent = false;
    return result;
    }
