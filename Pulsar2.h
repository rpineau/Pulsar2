//
//  Pulsar2.h
//  RoboFocus
//
//  Created by Richard Wright on 3/25/13.
//  modified by Richard Francis, 10 Oct 2013
//  further modified by Richard Francis, 28 Oct 2018
//
//

#ifndef __Pulsar2__Pulsar2__
#define __Pulsar2__Pulsar2__

#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <memory.h>
// #include <string.h>
#include <time.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

// C++ includes
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"  // added by Rodolphe in iEQ30 but not commented
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"

#define VERBOSE_FAILURES            0
#define VERBOSE_ESSENTIAL           1
#define VERBOSE_RESULTS             2
#define VERBOSE_FUNCTION_TRACKING   3
#define VERBOSE_ALL                 4
#define VERBOSE_CRAZY               5

#define PULSAR2_DEBUG VERBOSE_ALL

#define STOPPED     0
#define SIDEREAL    1
#define LUNAR       2
#define SOLAR       3
#define USER1       4
#define USER2       5
#define USER3       6

#define LOG_BUFFER_SIZE 256
#define SERIAL_BUFFER_SIZE 64
#define MAX_TIMEOUT 1000        // 1000 ms
// #define TIMEOUT_READ 1500

#define N_OUT_MAXSIZE 40  // buffer for the UTC timecode in ISO8601 format

#define ACK  0x06
#define NACK 0x15

#define NB_SLEW_SPEEDS      4
#define SLEW_NAME_LENGHT    32

enum PULSAR2_Errors    {OK = 0, NOT_CONNECTED, ND_CANT_CONNECT, BAD_CMD_RESPONSE, COMMAND_FAILED};
enum PULSAR_ParkStatus  {PARKED = 0, UNPARKED};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class CPulsar2Controller
    {
    public:
        CPulsar2Controller(void);
        ~CPulsar2Controller(void);
    
            
        int         connect(const char *szPort);
        void        disconnect(void);
        bool        isConnected(void) { return m_bIsConnected; }
        
        ////////////////////////////////////////////////////////////////
        // Commands to and from device
        
        int         stopSlew(void);                                         // not tested (not called by TSX)
        int         getRADec(double &dRA, double &dDec);                    // Works
        int         toggleFormat();                                         // Works

        int         setRefractionCorrection(bool bEnabled);                 // not tested
        int         getRefractionCorrection(bool &bRefractionNeeded);       // not tested

        int         syncRADec(const double &dRA, const double &dDec);       // not tested
        int         getSideOfPier(int &nPierSide);                          // Works
        int         getFirmware(char *szFirmware, int nMaxStrSize);         // Works

        int         startSlew(const double& dRa, const double& dDec);       // Works
        int         slewStatus(bool &bIsSlewing);                           // Works
        
        int         stopMoving(int iDir);                                   // Works
        int         startMove(int iDir, int iSpeedIndex);                   // Works
        
        int         setTrackingRate(int iRate);                             // not tested
        int         trackingOff(void);                                      // not tested
        int         getTracking(int &iTrackingRate);                        // not tested

        int         park(const double& dAz, const double& dAlt);            // Works
        int         unPark();                                               // Works
        int         parkStatus(bool &isParked);                             // Works

        int         setDateAndTime();                                       // not tested
        int         setLocation();                                          // not tested

        int         setRAdec(const double &dRA, const double &dDec);        // Works

        ////////////////////////////////////////////////////////////////

        int         getNbSlewRates();                                       // Works
        int         getRateName(int nZeroBasedIndex, char *pszOut, unsigned int nOutMaxSize);  // Works

        void        setSerxPointer(SerXInterface *p) { m_pSerx = p; }
        void        setLoggerPointer(LoggerInterface *p) { pLogger = p; }
        void        setSleeper(SleeperInterface *p) { pSleeper = p; }         // added by Rodolphe in iEQ30
        void        setTSX(TheSkyXFacadeForDriversInterface *pTSX) { m_pTsx = pTSX;};

        bool        m_bIsSlewing = false;

        bool        m_bIsParked;
        bool        m_bAtHome;


        int         iVerbosity = VERBOSE_ESSENTIAL;


    protected:

        int         sendCommand(const char *pszCmd, char *pszResult = NULL, int nResultMaxLen = SERIAL_BUFFER_SIZE, int nNbResponses = 1, int bSingleByteResponse = false);
        int         readResponse(char *szRespBuffer, int nBufferLen, int bSingleByteResponse = false);
        
        double      raStringToDouble(char* cRaString);
        double      decStringToDouble(char* cDecString);
        int         parseFields(const char *pszIn, std::vector<std::string> &svFields, char cSeparator);

        bool        m_bIsConnected;
        
        char        m_szFirmware[SERIAL_BUFFER_SIZE];

        LoggerInterface     *pLogger;
        SerXInterface       *m_pSerx;
        SleeperInterface    *pSleeper;        // added by Rodolphe in iEQ30
        TheSkyXFacadeForDriversInterface    *m_pTsx;

        char        szFirmware[9];
        int         iMajorFirmwareVersion; // added by CRF 4 Nov 2018, to be used to distinguish commands in v. 5.xx
        char        m_szLogMessage[LOG_BUFFER_SIZE];

        const char m_aszSlewRateNames[NB_SLEW_SPEEDS][SLEW_NAME_LENGHT] = {"Guide", "Centre", "Find", "Slew"};

#ifdef PULSAR2_DEBUG
        std::string m_sLogfilePath;
        // timestamp for logs
        char *timestamp;
        time_t ltime;
        FILE *Logfile;      // LogFile
#endif

    };


#endif /* defined(__Pulsar2__Pulsar2__) */
