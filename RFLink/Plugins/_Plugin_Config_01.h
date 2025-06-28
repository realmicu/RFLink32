// ************************************* //
// * Arduino Project RFLink32        * //
// * https://github.com/couin3/RFLink  * //
// * 2018..2020 Stormteam - Marc RIVES * //
// * More details in RFLink.ino file   * //
// ************************************* //

// ****************************************************************************************************************************************
// RFLink List of Plugins
// ****************************************************************************************************************************************
// Here are all plugins listed that are supported and used after compilation.
// When needed additional plugins can be added or selected plugins can be enabled/disabled.
//
// BEWARE OF THE PLUGIN_MAX setting!!       TX: 51      RX: 24

#ifndef _RFLINK_PLUGIN_CONFIG_01_H_
#define _RFLINK_PLUGIN_CONFIG_01_H_

#include "2_Signal.h"
#include "1_Radio.h"

// ****************************************************************************************************************************************
// Translation Plugin for oversized packets due to their breaks/pause being too short between packets
// Used for Flamingo FA500R and various others, do NOT exclude this plugin.
#define PLUGIN_001 // DO NOT CHANGE OR EXCLUDE!!
// ------------------------------------------------------------------------
// -- Any of the following protocols can be excluded whenever not needed --
// ------------------------------------------------------------------------
#define PLUGIN_002 // Lacrosse v2 2300/3600
#define PLUGIN_003 // Kaku : Klik-Aan-Klik-Uit (with code wheel) aka ARC
#define PLUGIN_004 // NewKAKU : Klik-Aan-Klik-Uit with automatic coding aka Intertechno.
#define PLUGIN_005 // Eurodomest
#define PLUGIN_006 // Blyss
#define PLUGIN_007 // Conrad RSL2
#define PLUGIN_008 // Kambrook
#define PLUGIN_009 // X10 RF
#define PLUGIN_010 // TRC02 RGB Switch
#define PLUGIN_011 // Home Confort
#define PLUGIN_012 // Flamingo FA500R
#define PLUGIN_013 // Powerfix/Quigg
#define PLUGIN_014 // Ikea Koppla
#define PLUGIN_015 // Home Easy EU
#define PLUGIN_017 // RTS
#define PLUGIN_016 // Silvercrest
#define PLUGIN_018 // Louvolite
// -------------------
// Weather sensors
// -------------------
#define PLUGIN_029 // Alecto V2 (WS3000/DWK2012) 434 MHz.
#define PLUGIN_030 // Alecto V1 (WS3500) 434 MHz.
#define PLUGIN_031 // Alecto V3 (WS1100/WS1200/WSD-19) 433.9 MHz.
#define PLUGIN_032 // Alecto V4
#define PLUGIN_033 // Conrad Pool Thermometer
#define PLUGIN_034 // Cresta
#define PLUGIN_035 // Imagintronix
#define PLUGIN_036 // F007_TH
#define PLUGIN_037 // Accurite 986 Thermometer
#define PLUGIN_040 // Mebus
#define PLUGIN_041 // LaCrosse v3 ws7000
#define PLUGIN_042 // UPM/Esic
#define PLUGIN_043 // LaCrosse v1
#define PLUGIN_044 // Auriol v3
#define PLUGIN_045 // Auriol
#define PLUGIN_046 // Auriol v2 / Xiron
#define PLUGIN_047 // Auriol v4
#define PLUGIN_048 // Oregon Weather
#define PLUGIN_049 // Lacrosse TX141
#define PLUGIN_050 // FineOffset WH2 / ClimeMet CM9088 Temperature/Humidity
#define PLUGIN_051 // Hyundai WS Senzor 77(TH) Temperature/Humidity 433.92 MHz
// -------------------
// Motion Sensors, include when needed
// -------------------
#define PLUGIN_060 // Ajax Chubb Varel 433 MHz. motion sensors
#define PLUGIN_061 // Chinese PIR motion door and window sensors
#define PLUGIN_062 // Chuango Alarm Devices
#define PLUGIN_063 // Oregon PIR/ALARM/LIGHT
#define PLUGIN_064 // Atlantic (and Visonic)
// -------------------
// Doorbells
// -------------------
#define PLUGIN_070 // Select Plus (Action - Quhwa)
#define PLUGIN_071 // Plieger York
#define PLUGIN_072 // Byron SX doorbell
#define PLUGIN_073 // Deltronic doorbell
#define PLUGIN_074 // RL02
#define PLUGIN_075 // Silvercrest
// -------------------
// Smoke detectors / Fire Places
// -------------------
#define PLUGIN_080 // Flamingo FA20 / KD101 smoke detector
#define PLUGIN_081 // Mertik Maxitrol / Dru fireplace
#define PLUGIN_082 // Mertik Maxitrol / Dru fireplace
#define PLUGIN_087 // NOX Alarm
// -------------------
// Misc
// -------------------
#define PLUGIN_076 // CAME-TOP432 Gate/Door opener
// -------------------
// 868 MHZ
// -------------------
// -------------------
// Housekeeping
// -------------------
#define PLUGIN_254 // Debug to show unsupported packets
// ****************************************************************************************************************************************
// RFLink List of Plugins that have TRANSMIT functionality
// ****************************************************************************************************************************************
// Here are all plugins listed that are supported and used after compilation.
// When needed additional plugins can be added or selected plugins can be enabled/disabled.
// ****************************************************************************************************************************************
// ------------------------------------------------------------------------
// -- Any of the following protocols can be excluded whenever not needed --
// ------------------------------------------------------------------------
#define PLUGIN_TX_003           // Kaku : Klik-Aan-Klik-Uit (with code wheel) aka ARC
#define PLUGIN_TX_004 // NewKAKU : Klik-Aan-Klik-Uit with automatic coding aka Intertechno.
// #define PLUGIN_TX_005           // Eurodomest
// #define PLUGIN_TX_006           // Blyss
// #define PLUGIN_TX_007           // Conrad RSL2
// #define PLUGIN_TX_008           // Kambrook
#define PLUGIN_TX_009           // X10 RF
// #define PLUGIN_TX_010		   // TRC02 RGB switch
// #define PLUGIN_TX_011           // Home Confort
// #define PLUGIN_TX_012           // Flamingo FA500R  (11)
// #define PLUGIN_TX_013           // Powerfix/Quigg
// #define PLUGIN_TX_015           // Home Easy EU     (14)
#define PLUGIN_TX_017           // RTS
#define PLUGIN_TX_016           // Silvercrest
#define PLUGIN_TX_018           // Louvolite
// -------------------
// Doorbells
// -------------------
// #define PLUGIN_TX_070           // Select Plus (Action - Quhwa)
// #define PLUGIN_TX_072           // Byron SX doorbell
// #define PLUGIN_TX_073           // Deltronic doorbell
// #define PLUGIN_TX_074           // RL02
// -------------------
// Smoke detectors
// -------------------
// #define PLUGIN_TX_080           // Flamingo FA20 / KD101 smoke detector
// #define PLUGIN_TX_082           // Mertik Maxitrol / Dru fireplace
// -------------------
// -------------------
// Alarm Systems
// -------------------
#define PLUGIN_TX_087
// -------------------
// Misc
// -------------------
#define PLUGIN_TX_076               // CAME TOP-432 Door Opener
// -------------------
//       -=#=-
// -------------------

#endif // _RFLINK_PLUGIN_CONFIG_01_H_
