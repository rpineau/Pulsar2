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
#include <string.h>
#include <time.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

// C++ includes
#include <string>


#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"  // added by Rodolphe in iEQ30 but not commented

#define VERBOSE_FAILURES            0
#define VERBOSE_ESSENTIAL           1
#define VERBOSE_RESULTS             2
#define VERBOSE_FUNCTION_TRACKING   3
#define VERBOSE_ALL                 4

#define VERBOSE_DEBUG   VERBOSE_ALL

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

#define ACK  0x06
#define NACK 0x15

#define NB_SLEW_SPEEDS      8
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
    
            
        bool        Connect(const char *szPort);
        void        Disconnect(void);
        bool        IsConnected(void) { return m_bIsConnected; }
        
        ////////////////////////////////////////////////////////////////
        // Commands to and from device
        
        int         Abort(void);                                        // not tested
        int         GetRADec(double &dRA, double &dDec);                // Works

        
        int         SyncRADec(const double &dRA, const double &dDec);   // Works
        int         GetSideOfPier(int &nPierSide);                      // not tested

        int         GetFirmware(char *szFirmware, int nMaxStrSize);     // Works
        
        int         startSlew(const double& dRa, const double& dDec, int &nSlewStatus);   // Works
        int         slewStatus(bool &bIsSlewing);                                   // Works
        
        int         stopMoving(void);                                   // not tested
        int         startMove(int iDir, int iSpeedIndex);               // not tested
        
        int         setTrackingRate(int iRate);                              // not tested
        int         trackingOff(void);                                  // not tested
        int         getTracking(int &iTrackingRate);                    // not tested

        int         Park(const double& dAz, const double& dAlt);                                         // not tested
        int         unPark();                                       // not tested
        int         parkStatus(bool &isParked);                                // not tested


        void        SetSerxPointer(SerXInterface *p) { m_pSerx = p; }
        void        SetLoggerPointer(LoggerInterface *p) { pLogger = p; }
        void        setSleeper(SleeperInterface *p) { pSleeper = p; }         // added by Rodolphe in iEQ30

        bool        m_bIsSlewing = false;

        bool        m_bIsParked;
        bool        m_bAtHome;


        int         iVerbosity = VERBOSE_ESSENTIAL;

        int getNbSlewRates();
        int getRateName(int nZeroBasedIndex, char *pszOut, unsigned int nOutMaxSize);

    protected:

        int         sendCommand(const char *pszCmd, char *pszResult, int nResultMaxLen);
        int         readResponse(char *szRespBuffer, int nBufferLen);
        
        bool        m_bIsConnected;
    
        int         setRAdec(const double &dRA, const double &dDec);
        double      raStringToDouble(char* cRaString);
        double      decStringToDouble(char* cDecString);

        char        m_szFirmware[SERIAL_BUFFER_SIZE];

        LoggerInterface      *pLogger;
        
        SerXInterface        *m_pSerx;
        
        SleeperInterface    *pSleeper;        // added by Rodolphe in iEQ30

        unsigned char        szFirmware[9];
        char                m_szLogMessage[LOG_BUFFER_SIZE];

        const char m_aszSlewRateNames[NB_SLEW_SPEEDS][SLEW_NAME_LENGHT] = { "Sidereal", "2x Sidereal", "8x Sidereal", "16x Sidereal",
                                                                            "64x Sidereal", "128x Sidereal", "256x Sidereal", "512x Sidereal"};

#ifdef VERBOSE_DEBUG
        std::string m_sLogfilePath;
        // timestamp for logs
        char *timestamp;
        time_t ltime;
        FILE *Logfile;      // LogFile
#endif

    };


#endif /* defined(__Pulsar2__Pulsar2__) */
