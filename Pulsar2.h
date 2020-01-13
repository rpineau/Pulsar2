//
//  Pulsar2.h
//  RoboFocus
//
//  Created by Richard Wright on 3/25/13.
//  modified by Richard Francis, 10 Oct 2013
//  further modified by Richard Francis, 28 Oct 2018 -- 23 Nov 2018
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

#define PULSAR2_DEBUG VERBOSE_ESSENTIAL

#define STOPPED     0
#define SIDEREAL    1
#define LUNAR       2
#define SOLAR       3
#define USER1       4
#define USER2       5
#define USER3       6

#define LOG_BUFFER_SIZE 256
#define SERIAL_BUFFER_SIZE 64
#define MAX_TIMEOUT 2000        // in ms

#define N_OUT_MAXSIZE 40  // buffer for the UTC timecode in ISO8601 format

#define ACK  0x06
#define NACK 0x15

#define NB_SLEW_SPEEDS      4
#define SLEW_NAME_LENGTH    32

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
        int         getAltAz(double &dAlt, double &dAz);                    // Works
        int         toggleFormat();                                         // Works

        int         setRefractionCorrection(bool bEnabled);                 // Works
        int         getRefractionCorrection(bool &bRefractionNeeded);       // not tested

        int         syncRADec(const double &dRA, const double &dDec);       // Works
        int         getSideOfPier(int &nPierSide);                          // Works
        int         getFirmware(char *szFirmware, int nMaxStrSize);         // Works

        int         commandTubeSwap(void);                                  // Works

        int         startSlew(const double& dRa, const double& dDec);       // Works
        int         slewStatus(bool &bIsSlewing);                           // Works
        
        int         stopMoving(int iDir);                                   // Works
        int         startMove(int iDir, int iSpeedIndex);                   // Works
        
        int         setTrackingRate(int iRate);                             // Works
        int         trackingOff(void);                                      // Works
        int         getTracking(int &iTrackingRate);                        // Works

        int         park(const double& dAz, const double& dAlt);            // Works
        int         unPark();                                               // Works
        int         parkIsParked(bool &isParked);                           // Works
        int         parkIsParking(bool &isParking);                         // Works
        int         parkIsParkDefined(bool &isParkSet);                     // Works (if answer yes -- don't know  how to undefine park to test opposite case)
        int         parkSetParkPosition();                                  // Works

        int         setDateAndTime();                                       // Works
        int         setLocation();                                          // Works

        int         setRAdec(const double &dRA, const double &dDec);        // Works
        
        int         getGuideRates(int &iRa, int &iDec);                     // Works
        int         getCentreRates(int &iRa, int &iDec);                    // Works
        int         getFindRates(int &iRa, int &iDec);                      // Works
        int         getSlewRates(int &iRa, int &iDec);                      // Works
        int         getGoToRates(int &iRa, int &iDec);                      // not tested and not used

        int         setGuideRates(int iRa, int iDec);                       // Works
        int         setCentreRates(int iRa, int iDec);                      // Works
        int         setFindRates(int iRa, int iDec);                        // Works
        int         setSlewRates(int iRa, int iDec);                        // Works
        int         setGoToRates(int iRa, int iDec);                        // not tested and not used
        
        ////////////////////////////////////////////////////////////////

        int         getNbSlewRates();
        int         getRateName(int nZeroBasedIndex, char *pszOut, unsigned int nOutMaxSize);

        void        setSerxPointer(SerXInterface *p) { m_pSerx = p; }
        void        setLoggerPointer(LoggerInterface *p) { pLogger = p; }
        void        setSleeper(SleeperInterface *p) { pSleeper = p; }
        void        setTSX(TheSkyXFacadeForDriversInterface *pTSX) { m_pTsx = pTSX;};

        bool        m_bIsSlewing = false;

        bool        m_bIsParked;
        bool        m_bAtHome;


        int         iVerbosity = VERBOSE_ESSENTIAL;

        void        logMessage(char* cMethod, char* cMessage);

        // Stored parameters
        
        double                                  dHoursEastStored;
        double                                  dHoursWestStored;
        double                                  dFlipHourStored;
        int                                     iMeridianBehaviourStored;
        double                                  dGuideRateRAStored;
        double                                  dGuideRateDecStored;
        int                                     iCentreRateRAStored;
        int                                     iCentreRateDecStored;
        int                                     iFindRateRAStored;
        int                                     iFindRateDecStored;
        int                                     iSlewRateRAStored;
        int                                     iSlewRateDecStored;
        bool                                    bSyncTimeOnConnectStored;
        bool                                    bSyncLocationOnConnectStored;
        
        
    protected:

        int         sendCommand(const char *pszCmd, char *pszResult = NULL, int nResultMaxLen = SERIAL_BUFFER_SIZE, int nNbResponses = 1, int bSingleByteResponse = false);
        int         readResponse(char *szRespBuffer, int nBufferLen, int bSingleByteResponse = false);
        
        int         handleMeridian(double dRa, double dDec);
        
        double      raStringToDouble(char* cRaString);
        double      decStringToDouble(char* cDecString);
        double      azStringToDouble(char* cAzString);
        
        int         parseFields(const char *pszIn, std::vector<std::string> &svFields, char cSeparator);
        
        bool        m_bIsConnected;
        
        char        m_szFirmware[SERIAL_BUFFER_SIZE];

        LoggerInterface     *pLogger;
        SerXInterface       *m_pSerx;
        SleeperInterface    *pSleeper;
        TheSkyXFacadeForDriversInterface    *m_pTsx;

        char        szFirmware[9];
        int         iMajorFirmwareVersion; // to be used to distinguish commands in v. 5.xx
        char        m_szLogMessage[LOG_BUFFER_SIZE];
        
        bool swapTubeCommandIssued;
        
         const char m_aszSlewRateNames[NB_SLEW_SPEEDS][SLEW_NAME_LENGTH] = {"Guide", "Centre", "Find", "Slew"};

#ifdef PULSAR2_DEBUG
        std::string m_sLogfilePath;
        // timestamp for logs
        char *timestamp;
        time_t ltime;
        FILE *Logfile;      // LogFile
#endif

    };


#endif /* defined(__Pulsar2__Pulsar2__) */
