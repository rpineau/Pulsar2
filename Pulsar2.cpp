//
//  Pulsar2.cpp
//  RoboFocus
//
//  Created by Richard Francis on 6 Oct 2013,
//  Based on RoboFocus code created by Richard Wright on 3/25/13.
//  further modified by Richard Francis, 28 Oct 2018, following
//     changes made by Rodolphe to my iEQ30 code
//
//

#include "Pulsar2.h"

#define TIMEOUT_READ        1500



CPulsar2Controller::CPulsar2Controller(void)
    {
    m_bIsConnected = false;
    m_bIsSlewing = false;  // added by Rodolphe
        
    m_bIsParked = false;  // added by CRF
    m_bAtHome = false;  // added by Rodolphe
        
#ifdef VERBOSE_DEBUG
#if defined(SB_WIN_BUILD)
        m_sLogfilePath = getenv("HOMEDRIVE");
        m_sLogfilePath += getenv("HOMEPATH");
        m_sLogfilePath += "\\Pulsar2Log.txt";
#elif defined(SB_LINUX_BUILD)
        m_sLogfilePath = getenv("HOME");
        m_sLogfilePath += "/Pulsar2Log.txt";
#elif defined(SB_MAC_BUILD)
        m_sLogfilePath = getenv("HOME");
        m_sLogfilePath += "/Pulsar2Log.txt";
#endif
        Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined VERBOSE_DEBUG && VERBOSE_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller] Constructor Called\n", timestamp);
        fflush(Logfile);
#endif
    }



CPulsar2Controller::~CPulsar2Controller(void)
    {

    }


//////////////////////////////////////////////////////////////////////////////
bool CPulsar2Controller::Connect(const char *szPort)
{
    int nErr = SB_OK;

    if(m_pSerx->open(szPort, 38400, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    // Validate that we are talking to an Pulsar2 by
    // getting the firmware version.
    // If this doesn't work
    // then abort and close our connection to the device
    
    nErr = GetFirmware(m_szFirmware, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined VERBOSE_DEBUG && VERBOSE_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Error getting firmware\n", timestamp);
        fflush(Logfile);
#endif
        m_pSerx->close();
        m_bIsConnected = false;
        return nErr;
    }
    
    return m_bIsConnected;
}


void CPulsar2Controller::Disconnect(void)
{
    if(m_bIsConnected)
        {
        m_pSerx->close();
        m_bIsConnected = false;
        }
}


//////////////////////////////////////////////////////////////////////////////
// Send commands, and read results


int CPulsar2Controller::sendCommand(const char *pszCmd, char *pszResult, int nResultMaxLen)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;

    m_pSerx->purgeTxRx();

#if defined VERBOSE_DEBUG && VERBOSE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CNexDome::domeCommand sending : %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;

    if (!pszResult) // we don't expect a response
        return nErr;

    // read response
    nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::domeCommand] ***** ERROR READING RESPONSE **** error = %d , response : %s\n", timestamp, nErr, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
#if defined DEBUG && DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::domeCommand] response : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    if(pszResult)
        strncpy(pszResult, &szResp[1], nResultMaxLen);

    return nErr;

}


int CPulsar2Controller::readResponse(char *szRespBuffer, int nBufferLen)
{
    int nErr = OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;

    memset(szRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = szRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] readFile error\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }

        if (ulBytesRead !=1) {// timeout
#if defined DEBUG && DEBUG >= VERBOSE_ALL
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] Timeout while waiting for response from controller\n", timestamp);
            fflush(Logfile);
#endif

            nErr = BAD_CMD_RESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;

        // special case response
        if(*pszBufPtr == NACK)
            return NACK;
        if(*pszBufPtr == ACK) {
            return OK;
        }

    } while (*pszBufPtr++ != '#' && ulTotalBytesRead < nBufferLen );

    if(ulTotalBytesRead)
        *(pszBufPtr-1) = 0; //remove the #

    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
// Get the firmware version
//
// Response is:
//      PULSAR Vx.xxx, 2008.10.10#
//      PULSAR V4.03a   ,2010.12.01      #
//

int CPulsar2Controller::GetFirmware(char *szFirmware, int nMaxStrSize)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    memset(szFirmware, nMaxStrSize, 0);
    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = sendCommand("#:YV#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // Okay, we got something. Is it a valid looking firmware identifier?
    if (memcmp("PULSAR V", szResp, 8) != 0)
        return false;
   
    memcpy(szFirmware, szResp+7, 6);
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
// Halt current move. False on error.
int CPulsar2Controller::Abort(void)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand(":Q#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // if result isn't 1 we're in trouble. Try again,
    // but only once
    if (szResp[0] != '1') {
        nErr = sendCommand(":Q#", szResp, SERIAL_BUFFER_SIZE);
        if(nErr)
            return nErr;
        if (szResp[0] != '1') {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CPulsar2Controller::Abort] Error in response : %s\n", timestamp, szResp);
            fflush(Logfile);
#endif
        }
    }

#if defined DEBUG && DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::Abort] Abort OK\n", timestamp, );
    fflush(Logfile);
#endif

    return nErr;
}

//////////////////////////////////////////////////////////////////////////////

int CPulsar2Controller::GetRADec(double &dRA, double &dDec)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = sendCommand(":GR#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

#if defined DEBUG && DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Response to GetRA command: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    dRA = raStringToDouble(szResp);

#if defined DEBUG && DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] dRA = %3.2f\n", timestamp, dRA);
    fflush(Logfile);
#endif

    nErr = sendCommand(":GD#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

#if defined DEBUG && DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Response to GetDec command: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    dDec = decStringToDouble(szResp);
#if defined DEBUG && DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] dDec = %3.2f\n", timestamp, dDec);
    fflush(Logfile);
#endif

    // add this call to regularly called functions to keep the status up-to-date
    // bool bResult;  // a dummy to absorb the result
    // bResult = CPulsar2Controller::slewStatus();
    // bResult = CPulsar2Controller::parkStatus();  // added by CRF 28/10/2018

    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
// SyncRaDec
//   three steps are needed:
//      1. define commanded RA
//      2. define commanded Dec
//      3. Calibrate mount (sync): current RA/Dec becomes commanded RA/Dec
//  Important: assumes the mount is on the corect side of the pier

int CPulsar2Controller::SyncRADec(const double &dRa, const double &dDec)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = setRAdec(dRa, dDec);  // function to set the commanded RA and dec
    if(nErr)
        return nErr;

    nErr = sendCommand(":CM#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

#if defined DEBUG && DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec] Response to SyncRADec / Calibrate Mount command: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    // response echoes the synced coordinates back, unlike the LX200 or iEQ30 version
    // so decode these and check against the command to verify success
    
    char raString[SERIAL_BUFFER_SIZE];  // "HH:MM:SS#";
    char decString[SERIAL_BUFFER_SIZE]; // "sDD:MM:SS#";
    strncpy(raString, szResp, 9);
    strncpy(decString, szResp+9, 10);

#if defined DEBUG && DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec]Converted raString : %s\n", timestamp, raStringToDouble(raString));
    fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec]Converted decString : %s\n", timestamp, decStringToDouble(decString));
    fflush(Logfile);
#endif

    if ((fabs(raStringToDouble(raString) - dRa) < 5.0/3600.0) && (fabs(decStringToDouble(decString) - dDec) < 5.0/3600.0)) {  // 5 arcsec
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec] Calibrate Mount command succeeded\n", timestamp);
        fflush(Logfile);
#endif
    }
    else {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec] Calibrate Mount command failed\n", timestamp);
        fflush(Logfile);
#endif
        nErr = ERR_CMDFAILED;
    }
    
    
    return nErr;
    
}

//////////////////////////////////////////////////////////////////////////////
// Get the side of the pier for the OTA
// Returns 0 for East and 1 for West
// Returns 0 on error

int CPulsar2Controller::GetSideOfPier(int &nPierSide)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = sendCommand("#:YGN#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

#if defined DEBUG && DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::GetSideOfPier] Side of pier response: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    if (szResp[0] == '0')
        nPierSide = 0;
    else if (szResp[0] == '1')
        nPierSide = 1;
    else {
        nPierSide = -1;
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
// returns "1" if command accepted, "0" if the object is below the horizon
//   three steps are needed:
//      1. define commanded RA
//      2. define commanded Dec
//      3. command the movement
//
int CPulsar2Controller::startSlew(const double& dRa, const double& dDec, int &nSlewStatus)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = setRAdec(dRa, dDec);  // function to set the commanded RA and dec
    if(nErr)
        return nErr;

    nErr = sendCommand(":MS#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::startSlew] Error startSlew to %3.2f ; %3,2f : %s\n", timestamp, dRa, dDec, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

#if defined DEBUG && DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::startSlew] Response to startSlew : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    if (szResp[0] == '1')
        nSlewStatus = 1; // accepted
    else if (szResp[0] == '0')
        nSlewStatus = 0;  // below horizon
    else {
        nSlewStatus = -1;  // error
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
// Function used to set the commanded RA and dec
//      1. define commanded RA
//      2. define commanded Dec
// Returns true if successful, false if not

int CPulsar2Controller::setRAdec(const double &dRA, const double &dDec)
{
    int nErr = OK;
    char szCommand[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];


    int iRAhh = 00;
    int iRAmm = 00;
    int iRAss = 00;

    double dPosDec = 0.0;
    char cDecs = '+';
    int iDechh = 00;
    int iDecmm = 00;
    int iDecss = 00;

    if(!m_bIsConnected)
        return ERR_NOLINK;


    iRAhh = (int)floor(dRA);
    iRAmm = (int)floor((dRA-(double)iRAhh)*60.0);
    iRAss = (int)floor((((dRA-(double)iRAhh)*60.0)-(double)iRAmm)*60.0);
    

    // first the RA
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRAdec] Error setRAdec to Ra = %3.2f : %s\n", timestamp, dRa, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    if (szResp[0] != '1') {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRAdec] Error setRAdec to Ra = %3.2f : %s\n", timestamp, dRa, szResp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }
    
    // Now the dec
    if (dDec < 0.0) {
        dPosDec = -dDec;
        cDecs ='-';
    }
    else {
        dPosDec = dDec;
        cDecs ='+';
    }
    iDechh = (int)floor(dPosDec);
    iDecmm = (int)floor((dPosDec-(double)iDechh)*60.0);
    iDecss = (int)floor((((dPosDec-(double)iDechh)*60.0)-(double)iDecmm)*60.0);
    
    snprintf(szCommand, SERIAL_BUFFER_SIZE, ":Sd %1c%02i*%02i:%02i#", cDecs, iDechh, iDecmm, iDecss);
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRAdec] Error setRAdec to Dec = %3.2f : %s\n", timestamp, dDec, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    if (szResp[0] != '1') {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRAdec] Error setRAdec to Dec = %3.2f : %s\n", timestamp, dDec, szResp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::slewStatus(bool &bIsSlewing)
{

    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = sendCommand("#:YGi#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::slewStatus] Error getting slew status : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    if (szResp[0] == '1'){
        m_bIsSlewing = true;
    }
    else {
        m_bIsSlewing = false;
    }

    bIsSlewing = m_bIsSlewing;

    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::stopMoving(void)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = sendCommand(":Q#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::stopMoving] Error getting response : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    return nErr;
    
}

//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::startMove(int iDir, int iSpeedIndex)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCommand[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    // enum MountDriverInterface::MoveDir
    //      MD_NORTH = 0
    //      MD_SOUTH = 1
    //      MD_EAST = 2
    //      MD_WEST = 3
    //
    // Rates:
    //      0 = sideral
    //      1 = 2x sidereal
    //      2 = 8x sidereal
    //      3 = 16x sidereal
    //      4 = 64x sidereal
    //      5 = 128x sidereal
    //      6 = 256x sidereal
    //      7 = 512x sidereal
    
    // we will do all this by modifying and selecting the Centring speed

    
    // first set the speed, then invoke the move

    // set the speed for Centring
    switch (iSpeedIndex)
    {
        default:
        case 0: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSB1,1#");       break;    // "Sidereal"
        case 1: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSB2,2#");       break;    // "2x Sidereal"
        case 2: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSB8,8#");       break;    // "8x Sidereal"
        case 3: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSB16,16#");     break;    // "16x Sidereal"
        case 4: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSB64,64#");     break;    // "64x Sidereal"
        case 5: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSB128,128#");   break;    // "128x Sidereal"
        case 6: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSB256,256#");   break;    // "256x Sidereal"
        case 7: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSB512,512#");   break;    // "512x Sidereal"
    }

    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::startMove] Error getting response : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    if (szResp[0] != '1'){
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::startMove] Error incorrect response to set speed : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED; // slewing
    }

    // select centring speed
    nErr = sendCommand(":RC#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::startMove] Error selecting centring speed : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    // start the move
    switch (iDir) {
        default:
        case 0: snprintf(szCommand, SERIAL_BUFFER_SIZE, ":Mn#");   break; // North
        case 1: snprintf(szCommand, SERIAL_BUFFER_SIZE, ":Ms#");   break; // South
        case 2: snprintf(szCommand, SERIAL_BUFFER_SIZE, ":Me#");   break; // East
        case 3: snprintf(szCommand, SERIAL_BUFFER_SIZE, ":Mw#");   break; // West
    }

    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::startMove] Error selecting centring speed : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    // no response expected
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
// Function used to set the tracking rate
// Returns true if successful, false if not

int CPulsar2Controller::setTrackingRate(int nRate)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCommand[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSS%d,0#", nRate);

    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setTrackingRate] Error setting tracking to %d : %s\n", timestamp, nRate, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    if (szResp[0] != '1'){
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setTrackingRate] Error setting tracking to %d : %s\n", timestamp, nRate, szResp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
// Function used to turn tracking off
// Returns true if successful, false if not

int CPulsar2Controller::trackingOff(void)
{
    int nErr = OK;

    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = setTrackingRate(STOPPED); // this stops the tracking

    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
// Function to get tracking rate
// Returns OK if successful, ERR_CMDFAILED if not

int CPulsar2Controller::getTracking(int &iTrackingRate)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = sendCommand("#:YGS#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getTracking] Error getting tracking tracking rate %d : %s\n", timestamp, nErr, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    iTrackingRate = int(szResp[0]);

    if (iTrackingRate >=4) {// user tracking rate, we should get the actual values
        // TOTO : read user rate with #:YGZx#, x=(iTrackingRate-4)
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getTracking] Custorm user tracking rate %d : %s\n", timestamp, iTrackingRate-4, szResp);
        fflush(Logfile);
#endif
    }

    return true;
}


//////////////////////////////////////////////////////////////////////////////
// Function to park the mount
// Returns true if successful, false if not
//
// Not implemented yet -- this is the iEQ30 version

int CPulsar2Controller::Park(const double& dAz, const double& dAlt)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = sendCommand("#:YH#", szResp, SERIAL_BUFFER_SIZE);    // park
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Park] Error parking mount : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if (szResp[0] != '1'){
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Park] Error parking mount : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }
    
    m_bIsParked = false;

    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
// Function to unpark the mount
// Returns true if successful, false if not
int CPulsar2Controller::unPark()
{

    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = sendCommand("#:YL#", szResp, SERIAL_BUFFER_SIZE);    // unpark
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Park] Error parking mount : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if (szResp[0] != '1'){
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Park] Error parking mount : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }

    m_bAtHome = false;
    m_bIsParked = false;

    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
// Function to ask if the mount is parked
// sets the class variable m_bIsParked
// set isPArked to true if parked, false if not
//
int CPulsar2Controller::parkStatus(bool &isParked)
{

    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = sendCommand("#:YGk#", szResp, SERIAL_BUFFER_SIZE);    // parked ?
    if(nErr) {
#if defined DEBUG && DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Park] Error parking mount : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    m_bIsParked = szResp[0]=='1'?true:false;
    isParked = m_bIsParked;
    return nErr;
}




//////////////////////////////////////////////////////////////////////////////

double CPulsar2Controller::raStringToDouble(char* cRaString)
{
    double dRA = -999.0;
    double dHours;
    double dMinutes;
    double dSeconds;

    //  raString is in the form "HH:MM:SS#"
    
    // first get the hours
    sscanf((char *) cRaString, "%2lf", &dHours);
    // minutes
    sscanf((char *) cRaString, "%*3c %2lf", &dMinutes);
    // seconds
    sscanf((char *) cRaString, "%*6c %2lf", &dSeconds);
    
    dRA = dHours + dMinutes/60.0 + dSeconds/3600.0;
    if (dRA < 0.0)
        dRA = dRA + 24.0;
    if (dRA > 24.0)
        dRA = dRA - 24.0;
    
    return dRA;
}


double CPulsar2Controller::decStringToDouble(char* cDecString)
{
    double dDec = -999.0;
    double dDegrees;
    double dMinutes;
    double dSeconds;
    double dSign; // = +1.00 or -1.00
    
//  decString is in the form "sDD*MM:SS#"
    
    // first get the sign
    if (cDecString[0] == 0x2d) // 0x2d = "-"
        dSign = -1.00;
    else
        dSign = +1.00;
    // degrees
    sscanf((char *) cDecString, "%*1c %2lf", &dDegrees);
    // minutes
    sscanf((char *) cDecString, "%*4c %2lf", &dMinutes);
    // seconds
    sscanf((char *) cDecString, "%*7c %2lf", &dSeconds);
    
    dDec = dSign * (dDegrees + dMinutes/60.0 + dSeconds/3600.0);
    
    return dDec;
    
}

int CPulsar2Controller::getNbSlewRates()
{
    return NB_SLEW_SPEEDS;
}

int CPulsar2Controller::getRateName(int nZeroBasedIndex, char *pszOut, unsigned int nOutMaxSize)
{
    if (nZeroBasedIndex > NB_SLEW_SPEEDS)
        return BAD_CMD_RESPONSE;

    strncpy(pszOut, m_aszSlewRateNames[nZeroBasedIndex], nOutMaxSize);

    return OK;
}

    
    


