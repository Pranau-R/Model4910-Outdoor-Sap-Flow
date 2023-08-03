/*

Name:   Model4910-Outdoor-Sap-Flow.h

Function:
        Global linkage for Model4910-Outdoor-Sap-Flow.ino

Copyright:
        See accompanying LICENSE file for copyright and license information.

Author:
        Pranau R, MCCI Corporation   July 2023

*/

#ifndef _Model4910-Outdoor-Sap-Flow_h_
# define _Model4910-Outdoor-Sap-Flow_h_

#pragma once

#include <Catena.h>
#include <Catena_Led.h>
#include <Catena_Mx25v8035f.h>
#include <Catena_Timer.h>
#include <SPI.h>
#include "Model4910_cMeasurementLoop.h"

//  The global clock object

extern  McciCatena::Catena                      gCatena;
extern  McciCatena::cTimer                      ledTimer;
extern  McciCatena::Catena::LoRaWAN             gLoRaWAN;
extern  McciCatena::StatusLed                   gLed;

extern  SPIClass                                gSPI2;
extern  McciModel4910::cMeasurementLoop         gMeasurementLoop;

//  The flash
extern  McciCatena::Catena_Mx25v8035f           gFlash;

#endif // !defined(_Model4910-Outdoor-Sap-Flow_h_)