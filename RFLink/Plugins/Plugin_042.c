//#######################################################################################################
//##                    This Plugin is only for use with the RFLink software package                   ##
//##                                    Plugin-042: UPM/Esic                                           ##
//#######################################################################################################
/*********************************************************************************************\
 * Dit protocol zorgt voor ontvangst van UPM/Esic weerstation buitensensoren 
 * Tevens alle sensoren die het UPM/Esic (W.H. Mandolyn International Ltd) protocol volgen waaronder:
 * UPM, Esic, Emos, DVM, Clas Ohlson, Dickson 
 * WT260,WT260H,WT440H,WT450,WT450H,WDS500,RG700
 *
 * Author  (present)  : StormTeam 2018..2020 - Marc RIVES (aka Couin3)
 * Support (present)  : https://github.com/couin3/RFLink 
 * Author  (original) : StuntTeam 2015..2016
 * Support (original) : http://sourceforge.net/projects/rflink/
 * License            : This code is free for use in any open source project when this header is included.
 *                      Usage of any parts of this code in a commercial application is prohibited!
 *********************************************************************************************
 * Changelog: v1.0 initial release
 *********************************************************************************************
 * Technical information:
 * Decodes signals from a UPM/Esic Weatherstation outdoor unit, (52/54 pulses, 36 bits, 433 MHz).
 * Supports two packet formats
 * --------------------------------------------------------------------------------------------
 * FORMAT 1:
 *
 * ____Byte 0_____  ____Byte 1_____  ____Byte 2_____  ____Byte 3_____  _Nib4__
 * 7 6 5 4 3 2 1 0  7 6 5 4 3 2 1 0  7 6 5 4 3 2 1 0  7 6 5 4 3 2 1 0  3 2 1 0
 * x x x x c c c c  d d y y b S S S  s s s s s P P P  p p p p p p p p  z z C C
 *
 * x = Constant, 1100, probably preamble
 * c = House Code (0 - 15)
 * d = Device Code (1 - 4) ?
 * y = ?
 * b = Low battery indication
 * S = Secondary value - Humidity/Wind direction (high bits)
 * s = Secondary value - Humidity/Wind direction (low bits)
 * P = Primary value - Temperature/Rain/Wind speed value (high bits)
 * p = Primary value - Temperature/Rain/Wind speed value (low bits)
 * z = Sequence number 0 - 2. Messages are sent in bursts of 3. For some senders this is always 0
 * C = Checksum. bit 1 is XOR of odd bits, bit 0 XOR of even bits in message
 * 
 * If HouseCode = 10 and deviceCode = 2, then p and P is Wind speed
 * and h and H is Wind direction
 * 
 * If HouseCode = 10 and deviceCode = 3, then p and P is rain
 *
 * Temp (C) = RawValue / 16 - 50
 * Rain (total mm) = RawValue * 0,7
 * Wind Speed (mph)= RawValue (* 1/3,6 for km/h)
 * Humidity (%) = RawValue / 2
 * Wind direction (deg) = RawValue * 22,5
 * --------------------------------------------------------------------------------------------
 * FORMAT 2:
 *
 * ____Byte 0_____  ____Byte 1_____  ____Byte 2_____  ____Byte 3_____  _Nib4__
 * 7 6 5 4 3 2 1 0  7 6 5 4 3 2 1 0  7 6 5 4 3 2 1 0  7 6 5 4 3 2 1 0  3 2 1 0
 * x x x x c c c c  d d y y b h h h  h h h h T T T T  T T T T t t t t  t t t p
 *
 * x = Constant, 1100, probably preamble
 * c = House Code (0 - 15)
 * d = Device (Channel) Code (1 - 4) ?
 * y = ? 
 * b = ? 
 * h = Humidity (7 bits) (0111011 = 59 %) 
 * T = Temperature (8 bits) (see below) 
 * t = Temperature (7 bits) (see below)       
 * p = Parity (xor of all bits should give 0)
 *
 * The temperature is transmitted as (temp + 50.0) * 128, which equals (temp * 128) + 6400. Adding 50.0 °C makes
 * all values positive, an unsigned 15 bit integer where the first 8 bits correspond to the whole part of the temperature
 * (here 01001001, decimal 73, substract 50 = 23). Remaining 7 bits correspond to the fractional part.
 * Sample:
 * 20;64;DEBUG;Pulses=52;Pulses(uSec)=875,875,825,875,1725,1800,1725,1800,1725,850,825,1800,1725,875,800,850,825,1800,1725,1800,800,875,800,850,1725,1800,825,850,1725,850,825,1800,1725,1800,825,850,1725,875,800,1800,800,875,800,850,825,850,1725,1800,1750,1800,475;
 * 11000001 00110000 11100100 01100000 0010
 *
 *20;C3;DEBUG;Pulses=52;Pulses(uSec)=950,975,850,975,1850,1975,1875,1975,1850,975,850,1975,1850,975,850,975,850,2000,1850,975,875,975,850,975,850,2000,850,975,1850,2000,850,975,1850,2000,850,975,1875,975,850,975,850,975,850,2000,1850,1975,1850,2000,1850,1975,225;
 *20;C4;DEBUG;Pulses=52;Pulses(uSec)=950,975,850,975,1850,2000,1875,2000,1850,975,850,2000,1850,975,850,975,850,2000,1850,975,875,950,875,975,850,2000,850,975,1850,2000,850,975,1850,2000,850,975,1875,975,850,975,850,975,850,2000,1850,2000,1850,2000,1850,2000,225;
 *20;C5;DEBUG;Pulses=52;Pulses(uSec)=950,975,850,975,1850,2000,1875,2000,1850,975,850,2000,1850,975,850,975,850,2000,1850,975,875,975,850,975,850,1975,850,975,1850,1975,850,975,1850,2000,850,975,1875,975,850,975,850,975,850,2000,1850,2000,1850,2000,1850,2000,225;
 *20;32;DEBUG;Pulses=48;Pulses(uSec)=850,900,875,900,1850,1875,1850,1875,1850,900,875,1875,1850,900,875,900,875,1875,1850,900,875,1875,1850,1875,1825,900,875,1875,875,900,1850,1875,875,900,875,900,1850,1875,1825,1875,1850,1875,1850,1875,1825,1875,500;
 *20;33;UPM/Esic;ID=0001;TEMP=0104;HUM=33;BAT=OK;

 925,900,875,900,1825,1875,1850,1875,1850,900,
 875,1875,1850,900,850,900,850,1875,1850,900,
 850,1875,1850,900,850,1875,1850,1875,875,900,
 1850,1875,875,900,875,900,1825,1875,1850,900,
 875,1875,1850,900,875,1875,850,900,875,900,
 500   51
0011111011 0011010111 0110011101 10100
 \*********************************************************************************************/
#define UPM_PLUGIN_ID 042
#define PLUGIN_DESC_042 "UPM/Esic / UPM/Esic F2"

#define UPM_MIN_PULSECOUNT 46
#define UPM_MAX_PULSECOUNT 56

#define UPM_PULSELOHI_D 1100
#define UPM_PULSEHIHI_D 2075
#define UPM_PULSEHILO_D 1600

#ifdef PLUGIN_042
#include "../4_Display.h"

boolean Plugin_042(byte function, const char *string)
{
   if (RawSignal.Number < UPM_MIN_PULSECOUNT || RawSignal.Number > UPM_MAX_PULSECOUNT)
      return false;

   const long UPM_PULSELOHI = UPM_PULSELOHI_D / RawSignal.Multiply;
   const long UPM_PULSEHIHI = UPM_PULSEHIHI_D / RawSignal.Multiply;
   const long UPM_PULSEHILO = UPM_PULSEHILO_D / RawSignal.Multiply;

   unsigned long bitstream1 = 0L; // holds first 10 bits
   unsigned long bitstream2 = 0L; // holds last 26 bits
   byte bitcounter = 0;           // counts number of received bits (converted from pulses)
   byte halfbit = 0;              // high pulse = 1, 2 low pulses = 0, halfbit keeps track of low pulses

   byte rc = 0;
   int temperature = 0;
   byte humidity = 0;
   unsigned int rain = 0;
   unsigned int winds = 0;
   unsigned int windd = 0;
   byte battery = 0;
   byte units = 0;
   byte devicecode = 0;
   byte checksum = 0;
   byte msgformat = 0;
   //==================================================================================
   // Get all 36 bits
   //==================================================================================
   for (byte x = 1; x < RawSignal.Number; x++)
   {
      if ((RawSignal.Pulses[x] > UPM_PULSEHILO) && (RawSignal.Pulses[x] < UPM_PULSEHIHI))
      {
         if (halfbit == 1) // UPM cant receive a 1 bit after a single low value
            return false;  // pulse error, must not be a UPM packet or reception error

         if (bitcounter < 10)
         {
            bitstream1 <<= 1;
            bitcounter++; // only need to count the first 10 bits
         }
         else
            bitstream2 <<= 1;

         halfbit = 0; // wait for next first low or high pulse
      }
      else
      {
         if ((RawSignal.Pulses[x] > UPM_PULSELOHI))
            return false; // Not a valid UPM pulse length

         if (halfbit == 0) // 2 times a low value = 0 bit
            halfbit = 1;   // first half received

         else
         {
            if (bitcounter < 10)
            {
               bitstream1 <<= 1;
               bitstream1 |= 0x1;
               bitcounter++; // only need to count the first 10 bits
            }
            else
            {
               bitstream2 <<= 1;
               bitstream2 |= 0x1;
            }
            halfbit = 0; // wait for next first low or high pulse
         }
      }
      if (bitcounter > 36)
         return false; // too many bits, it cant be the right protocol
   }
   //==================================================================================
   // Perform a quick sanity check
   //==================================================================================
   if ((bitstream1 >> 6) != 0x0C)
      return false; // sanity check, first 4 bits should always be '1100' to be a valid UPM/Esic packet

   // Impossible by design, see test above
   // if (bitstream1 == 0)
   //    return false;

   if (bitstream2 == 0)
      return false;
   //==================================================================================
   // Perform checksum calculations
   //==================================================================================
   // perform a checksum check to make sure the packet is a valid UPM/Esic packet
   // Checksum - xor all odd and all even bits should match the last two bits
   for (byte i = 0; i < 9; i += 2)
      checksum ^= ((bitstream1 >> i) & B11);
   for (byte i = 2; i < 25; i += 2)
      checksum ^= ((bitstream2 >> i) & B11);

   // did the format 1 checksum calculation match?
   if (checksum == (bitstream2 & B11)) // Yes, set it
      msgformat = 1;
   else // else perform a bit parity check to see if we have format 2
   {
      checksum = checksum ^ ((bitstream2)&3); // xor the last 2 bits
      units = (checksum >> 1) & 0x1;          // get the odd bit of the checksum result
      checksum = (checksum & 0x1) ^ units;    // xor the odd with the even bit of the checksum result
      // did the format 2 parity checksum calculation match?
      if (checksum == 0)
         msgformat = 2;
      else
         return false;
   }
   //==================================================================================
   // Prevent repeating signals from showing up
   //==================================================================================
   if ((SignalHash != SignalHashPrevious) || (RepeatingTimer + 500 < millis()) || (SignalCRC != bitstream1))
      SignalCRC = bitstream1; // not seen the RF packet recently
   else
      return true; // already seen the RF packet recently
   //==================================================================================
   // now process the various sensor types
   //==================================================================================
   units = bitstream1 & 0x03;             // housecode format 1&2
   rc = units;                            // housecode format 1&2
   devicecode = (bitstream1 >> 2) & 0x0f; // devicecode format 1&2
   //==================================================================================
   if (msgformat == 1)
   {
      battery = !((bitstream2 >> 23) & 1); // battery state 0=low 1=ok
      if ((rc == 10) && (devicecode == 2))
      { // wind
         units = (bitstream2 >> 4) & 0x0f;
         winds = (bitstream2 >> 8) & 0x7f;
         windd = (bitstream2 >> 15) & 0x0f; //0xff;      // wind direction
         //==================================================================================
         // Output
         //==================================================================================
         display_Header();
         display_Name(PSTR("UPM/Esic"));
         char c_ID[5];
         sprintf(c_ID, "%02X%02X", rc, devicecode);
         display_IDc(c_ID);
         display_WINSP(winds);
         display_WINDIR(windd);
         //==================================================================================
      }
      else if ((rc == 10) && (devicecode == 3))
      { // rain
         units = (bitstream2 >> 4) & 0x0f;
         rain = (bitstream2 >> 8) & 0x7f;
         rain *= 7; // Serial.print( (float)rain * 0.7 );
         //==================================================================================
         // Output
         //==================================================================================
         display_Header();
         display_Name(PSTR("UPM/Esic"));
         char c_ID[5];
         sprintf(c_ID, "%02X%02X", rc, devicecode);
         display_IDc(c_ID);
         display_RAIN(rain);
         //==================================================================================
      }
      else
      {                                    // temperature & Humidity
         units = (bitstream2 >> 4) & 0x0f; // temperature
         temperature = (bitstream2 >> 8) & 0x7f;
         temperature = temperature - 50;
         temperature = (temperature * 10) + units;
         if (temperature > 0x3e8)
            return false;
         humidity = (bitstream2 >> 15) & 0xff; // humidity
         humidity = humidity / 2;
         //if (humidity==0) return false;            // dont accept Bad humidity status
         //if (temperature > 1000) return false;     // dont accept bad temperature
         //==================================================================================
         // Output
         //==================================================================================
         display_Header();
         display_Name(PSTR("UPM/Esic"));
         char c_ID[5];
         sprintf(c_ID, "%02X%02X", rc, devicecode);
         display_IDc(c_ID);
         display_TEMP(temperature);
         display_HUM(humidity); // Humidity 0x15 = 21% decimal
         //==================================================================================
      }
   }
   else
   {
      units = (bitstream2 >> 1) & 0x7f; // temperature
      temperature = (bitstream2 >> 8) & 0xff;
      temperature = temperature - 50;
      temperature = (temperature * 100) + units;
      temperature = temperature / 10;
      if (temperature > 0x3e8)
         return false;
      humidity = (bitstream2 >> 16) & 0x7f; // humidity
      //==================================================================================
      // Output
      //==================================================================================
      display_Header();
      display_Name(PSTR("UPM/Esic F2"));
      char c_ID[5];
      sprintf(c_ID, "%02X%02X", rc, devicecode);
      display_IDc(c_ID);
      display_TEMP(temperature);
      display_HUM(humidity); // Humidity 0x15 = 21% decimal
   }
   //==================================================================================
   // Output (common)
   //==================================================================================
   display_BAT(battery);
   display_Footer();
   //==================================================================================
   RawSignal.Repeats = true; // suppress repeats of the same RF packet
   RawSignal.Number = 0;
   return true;
}
#endif // PLUGIN_042
