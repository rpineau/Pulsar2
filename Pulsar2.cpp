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




CPulsar2Controller::CPulsar2Controller(void)
{
    m_bIsConnected = false;
    m_bIsSlewing = false;  // added by Rodolphe
    
    m_bIsParked = false;  // added by CRF
    m_bAtHome = false;  // added by Rodolphe
    
    swapTubeCommandIssued = false;
    
#ifdef PULSAR2_DEBUG
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
    Logfile = fopen(m_sLogfilePath.c_str(), "a");
#endif
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= 2
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


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - Serial Link
////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::connect(const char *szPort)
//
// Firmware Validity: all
//
{
    int nErr = SB_OK;
    
    int iRA, iDec;
    
    m_bIsConnected = true;
    nErr = m_pSerx->open(szPort, 38400, SerXInterface::B_NOPARITY);
    if(nErr != SB_OK) {
        m_bIsConnected = false;
        return nErr;    // return the actual error to TSX so it can display the error dialog.
    }
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Opened serial port OK\n", timestamp);
    fflush(Logfile);
#endif
    
    
    // Validate that we are talking to a Pulsar2 by
    // getting the firmware version.
    // If this doesn't work
    // then abort and close our connection to the device
    
    nErr = getFirmware(m_szFirmware, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Error getting firmware\n", timestamp);
        fflush(Logfile);
#endif
        m_pSerx->close();
        m_bIsConnected = false;
    }
    else {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Firmware check OK.\n", timestamp);
        fflush(Logfile);
#endif
    }
    
    // Now that we are connected, switch the refraction correction off
    nErr = setRefractionCorrection(false);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Error setting refraction correction\n", timestamp);
        fflush(Logfile);
#endif
        return nErr;
    }
    else {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Setting refraction setting OK.\n", timestamp);
        fflush(Logfile);
#endif
    }
    
    // set the Pulsar2 time to TSX time, if required in the user dialog
    if (bSyncTimeOnConnectStored) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Set time.\n", timestamp);
        fflush(Logfile);
#endif
        nErr = setDateAndTime();
        if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Error setting date and time\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }
    }
    
    // set the Pulsar2 location to TSX location, if required in the user dialog
    if (bSyncLocationOnConnectStored) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Now set location.\n", timestamp);
        fflush(Logfile);
#endif
        
        nErr = setLocation();
        if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Error setting location\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }
        else {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Location setting OK.\n", timestamp);
            fflush(Logfile);
#endif
        }
    }
    
    // get the speed settings
    // guide
    getGuideRates(iRA, iDec);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Error getting Guide Rates\n", timestamp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if(iRA != (int)(dGuideRateRAStored*10.0)  ||  iDec != (int)(dGuideRateDecStored*10.0)) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Guide Rate Difference. Stored RA, Dec = %f, %f; Mount RA, Dec = %f, %f\n", timestamp, dGuideRateRAStored, dGuideRateDecStored, (double)(iRA/10.0), (double)(iDec/10.0));
        fflush(Logfile);
#endif
    dGuideRateRAStored = (double)iRA/10.0;
    dGuideRateDecStored = (double)iDec/10.0;
    }

    // centre
    getCentreRates(iRA, iDec);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Error getting Centre Rates\n", timestamp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if(iRA != iCentreRateRAStored  ||  iDec != iCentreRateDecStored) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Centre Rate Difference. Stored RA, Dec = %d, %d; Mount RA, Dec = %d, %d\n", timestamp, iCentreRateRAStored, iCentreRateDecStored, iRA, iDec);
        fflush(Logfile);
#endif
    iCentreRateRAStored = iRA;
    iCentreRateDecStored = iDec;
    }

    // find
    getFindRates(iRA, iDec);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Error getting Find Rates\n", timestamp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if(iRA != iFindRateRAStored  ||  iDec != iFindRateDecStored) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Find Rate Difference. Stored RA, Dec = %d, %d; Mount RA, Dec = %d, %d\n", timestamp, iFindRateRAStored, iFindRateDecStored, iRA, iDec);
        fflush(Logfile);
#endif
    iFindRateRAStored = iRA;
    iFindRateDecStored = iDec;
    }

    // slew
    getSlewRates(iRA, iDec);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Error getting Slew Rates\n", timestamp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if(iRA != iSlewRateRAStored  ||  iDec != iSlewRateDecStored) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Connect] Slew Rate Difference. Stored RA, Dec = %d, %d; Mount RA, Dec = %d, %d\n", timestamp, iSlewRateRAStored, iSlewRateDecStored, iRA, iDec);
        fflush(Logfile);
#endif
    iSlewRateRAStored = iRA;
    iSlewRateDecStored = iDec;
    }


    return nErr;
}


////////////////////////////////////////////////////////////////////////////
void CPulsar2Controller::disconnect(void)
//
// Firmware Validity: all
//
{
    if(m_bIsConnected)
    {
        m_pSerx->close();
        m_bIsConnected = false;
    }
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - Send and Receive
//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::sendCommand(const char *pszCmd, char *pszResult, int nResultMaxLen, int nNbResponses, int bSingleByteResponse)
//
// Firmware Validity: all
//
{
    int nErr = OK;
    int i = 0;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;
    std::string sResult;

    m_pSerx->purgeTxRx();
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CPulsar2Controller::sendCommand sending : %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif
    
    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;
    
    if (!pszResult) // it's NULL so we don't expect a response
        return nErr;

    for(i = 0; i<nNbResponses; i++) {
        // read response
        nErr = readResponse(szResp, SERIAL_BUFFER_SIZE, bSingleByteResponse);
        if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_ALL
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CPulsar2Controller::sendCommand] ***** ERROR READING RESPONSE **** error = %d , response : %s\n", timestamp, nErr, szResp);
            fflush(Logfile);
#endif
            // copy the partial response to pszResult
            strncpy(pszResult, szResp, nResultMaxLen);
            return nErr;
        }
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_ALL
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::sendCommand] response : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        if(i) {  // re-add the # if there is more than one in the response we expect, like RA#dec#
            sResult += "#";
        }
        sResult += szResp;  // concatenate responses
    }

    strncpy(pszResult, sResult.c_str(), nResultMaxLen);

    return nErr;
    
}


//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::readResponse(char *szRespBuffer, int nBufferLen, int bSingleByteResponse)
//
// Firmware Validity: all
//
{
    int nErr = OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;
    
    memset(szRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = szRespBuffer;
    
    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);  // NB: there is also a timeout called TIMEOUT_READ
        if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_ALL
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] readFile error\n", timestamp);
            fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] szRespBuffer = %s\n", timestamp, szRespBuffer);
            fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] ulTotalBytesRead = %lu\n", timestamp, ulTotalBytesRead);
            fflush(Logfile);
#endif
            return nErr;
        }
        
        if (ulBytesRead !=1) {
            // timeout, do not error out in there as some command don't end with # and only return 1 byte
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_ALL
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] Timeout while waiting for response from controller\n", timestamp);
            fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] szRespBuffer = %s\n", timestamp, szRespBuffer);
            fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] ulTotalBytesRead = %lu\n", timestamp, ulTotalBytesRead);
            fflush(Logfile);
#endif
            // nErr = ERR_RXTIMEOUT;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
        if( ulTotalBytesRead>0 && bSingleByteResponse)
        { // we got our single byte respone (0/1 usually).
            break;
        }
// -- this has helped solve comms questions, but now leads to lots of output, so it's disabled pending future use (set PULSAR2_DEBUG to VERBOSE_CRAZY in Pulsar2.h)
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_CRAZY
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] szRespBuffer = %s\n", timestamp, szRespBuffer);
        fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] ulTotalBytesRead = %lu\n", timestamp, ulTotalBytesRead);
        fflush(Logfile);
#endif
        // special case response
        if(*pszBufPtr == NACK)
            return NACK;
        if(*pszBufPtr == ACK) {
            return OK;
        }
    } while (*pszBufPtr++ != '#' && ulTotalBytesRead < nBufferLen );

// This is a summary, unnecessary if the commented-out part above is used
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_ALL && PULSAR2_DEBUG < VERBOSE_CRAZY
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] szRespBuffer = %s\n", timestamp, szRespBuffer);
    fprintf(Logfile, "[%s] [CPulsar2Controller::readResponse] ulTotalBytesRead = %lu\n", timestamp, ulTotalBytesRead);
    fflush(Logfile);
#endif

    
    if(!ulTotalBytesRead)   // we didn't even get 1 byte after MAX_TIMEOUT (250ms)
        nErr = ERR_RXTIMEOUT;

    else if(ulTotalBytesRead>1) // single byte response don't end with #
        *(pszBufPtr-1) = 0; //remove the last # so we don't have to parse it later as we don't need it
    
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::toggleFormat()
// LX200 commands have a so-called long and short format.
//  Short - RA displays and messages HH:MM.T sDD*MM
//  Long  - Dec/Az/El displays and messages HH:MM:SS sDD*MM:S
// In the Pulsar2, Long is the default.
// If we detect the short format, then we toggle to long
// using #:U#. Pulsar2 returns nothing
//
// Firmware Validity: >= 4
//
{
    int nErr = OK;

    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:U#");
    if(nErr)
        return nErr;
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FUNCTION_TRACKING
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::toggleFormat] toggleFormat sent\n", timestamp);
    fflush(Logfile);
#endif
    
    // ignore response as it's always supposed to be "Ok"
    // -- in fact neither the real LX200 nor Pulsar2 return anything
    
    return nErr;
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - Get information
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::getRADec(double &dRA, double &dDec)
//
// This is a combination of 2 of the classic LX200 commands: getRA and getDec
// Note that LX200 commands have a so-called long and short format.
//  Short - Dec returns sDD*MM#, where * here is 0xdf
//  Long  - Dec returns sDD:MM:SS#
// In the Pulsar2, Long is the default (I think).
// It can be toggled with #:U#. We do that if we detect the short format
//
// Firmware Validity: >= 4
//

{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    // first we get the RA
    nErr = sendCommand(":GR#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Response to GetRA command: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    
    dRA = raStringToDouble(szResp);
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] dRA = %5.4f\n", timestamp, dRA);
    fflush(Logfile);
#endif
    
    nErr = sendCommand(":GD#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Response to GetDec command: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    
    // add test here for return string length, to see if "toggle precision" needs to be sent.
    // And if so, re-launch the Get Dec command
    if (strlen(szResp)<= 7) {
        nErr = toggleFormat();
        if(nErr)
            return nErr;
        // send Get Dec again
        nErr = sendCommand(":GD#", szResp, SERIAL_BUFFER_SIZE);
        if(nErr)
            return nErr;
        
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Response to GetDec command: %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        
    }
    
    dDec = decStringToDouble(szResp);
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] dDec = %5.4f\n", timestamp, dDec);
    fflush(Logfile);
#endif
    
    //--------------------------------------------------------------------------------------------------------
    // now, before finishing, check the hour angle and check against the meridian behaviour what to do with it
    double currentHourAngle;
    int currentRate;
    int nPierSide;      // nPierSide is 0 for East and 1 for West, or -1 if the response was not understood
    bool bIsParking;
    bool bIsSlewing;
    
    nErr = getTracking(currentRate);  // first we need to find out if we're stopped already
    if (nErr != OK) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Error getting tracking rate: %i\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }

    if (currentRate == STOPPED)
        return nErr;
    
    nErr = getSideOfPier(nPierSide);      // we also need to check if the tube is already swapped (ie on the east side)
    if (nErr != OK) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Error getting pier side: %i\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    if (nPierSide == 0)  // nPierSide = 1 means that tube is on East side. We don't check against any limit on that side
        return nErr;
    
    nErr = parkIsParking(bIsParking);   // check if the mount is currently parking, and bail out if it is
    if (nErr != OK) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Error getting pier side: %i\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
if (bIsParking)
    return nErr;
    
    nErr = slewStatus(bIsSlewing);      // finally check if the mount is currently slewing: bail out if it is
    if (nErr != OK) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Error getting pier side: %i\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    if (bIsSlewing)
        return nErr;

    currentHourAngle = m_pTsx->hourAngle(dRA);  // sign convention is negative to the east
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Hour Angle: %7.4f, Tracking Rate: %d, Pier Side: %d\n", timestamp, currentHourAngle, currentRate, nPierSide);
    fflush(Logfile);
#endif
    switch (iMeridianBehaviourStored)
    {
        case 0:  // stop at meridian
            if (currentHourAngle >= 0.0) {
                nErr = setTrackingRate(STOPPED);
                if (nErr != OK) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Error stopping tracking: %i\n", timestamp, nErr);
                    fflush(Logfile);
#endif
                    return nErr;
                }
            }
            break;
        case 1:  // flip at meridian, if not already flipped
            if (currentHourAngle >= 0.0  && nPierSide == 1) {  // nPierSide should always be 1 as we bailed if == 0
                if (!swapTubeCommandIssued) {
                    nErr = commandTubeSwap();
                    if (nErr != OK) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
                        ltime = time(NULL);
                        timestamp = asctime(localtime(&ltime));
                        timestamp[strlen(timestamp) - 1] = 0;
                        fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Error swapping tube: %i\n", timestamp, nErr);
                        fflush(Logfile);
#endif
                        return nErr;
                    }
                    swapTubeCommandIssued = true;
                }
            }
            break;
        case 2:  // stop at west limit
            if (currentHourAngle >= dHoursWestStored) {
                nErr = setTrackingRate(STOPPED);
                if (nErr != OK) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Error stopping tracking: %i\n", timestamp, nErr);
                    fflush(Logfile);
#endif
                    return nErr;
                }
            }
            break;
        case 3:  // flip at (west) flip limit
            if (currentHourAngle  >= dFlipHourStored  && nPierSide == 1) {  // nPierSide should always be 1 as we bailed if == 0
                if (!swapTubeCommandIssued) {
                    nErr = commandTubeSwap();
                    if (nErr != OK) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
                        ltime = time(NULL);
                        timestamp = asctime(localtime(&ltime));
                        timestamp[strlen(timestamp) - 1] = 0;
                        fprintf(Logfile, "[%s] [CPulsar2Controller::GetRADec] Error swapping tube: %i\n", timestamp, nErr);
                        fflush(Logfile);
#endif
                        return nErr;
                    }
                    swapTubeCommandIssued = true;
                }
            }
            break;
    }
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::getAltAz(double &dAlt, double &dAz)
//
// This is a combination of 2 of the classic LX200 commands: getAlt and getAz
// Note that LX200 commands have a so-called long and short format.
// This will have been fixed during the periodic getRaDec calls
//
// Firmware Validity: >= 4
//

{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    // first we get the Altitude
    nErr = sendCommand(":GA#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::getAltAz] Response to GetAlt command: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    
    dAlt = decStringToDouble(szResp);  // reuse the method for Dec strings as the format is equivalent
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::getAltAz] dAlt = %5.4f\n", timestamp, dAlt);
    fflush(Logfile);
#endif
    
    nErr = sendCommand(":GZ#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::getAltAz] Response to GetDec command: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    
    dAz = azStringToDouble(szResp);
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::getAltAz] dAz = %5.4f\n", timestamp, dAz);
    fflush(Logfile);
#endif

    return nErr;
}





//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::getTracking(int &iTrackingRate)
// Function to get tracking rate
// Where iTrackingRate is:
//      0 = stop
//      1 = sidereal
//      2 = lunar
//      3 = solar
//      4 = user1
//      5 = user2
//      6 = user3
//
// Firmware Validity: = 4 for named rates
//                    = 5 for custom rate
//
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YGS#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getTracking] Error getting tracking rate %d : %s\n", timestamp, nErr, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    iTrackingRate = atoi(&szResp[0]);
    
    if (iTrackingRate < 4) {
      // standard tracking rate (stopped, sidereal, lunar, solar)
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getTracking] Tracking rate %d : %s\n", timestamp, iTrackingRate, szResp);
        fflush(Logfile);
#endif
   }
   else {
        // user tracking rate, we should get the actual values
        // TODO : read user rate with #:YGZx#, x=(iTrackingRate-4)
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getTracking] Custom user tracking rate %d : %s\n", timestamp, iTrackingRate-4, szResp);
        fflush(Logfile);
#endif
    }
    
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
// Get the firmware version
int CPulsar2Controller::getFirmware(char *szFirmware, int nMaxStrSize)
//
// Response is:
//      PULSAR Vx.xxx, 2008.10.10#
//      PULSAR V4.03a   ,2010.12.01      #
//
// Firmware Validity: all
//
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
     
    memset(szFirmware, 0, nMaxStrSize);
    nErr = sendCommand("#:YV#", szResp, SERIAL_BUFFER_SIZE);
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::GetFirmware] Response: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
   if(nErr != OK)
        return nErr;
    
    // Okay, we got something. Is it a valid looking firmware identifier?
    if (memcmp("PULSAR V", szResp, 8) != 0) {
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::GetFirmware] ERR_CMDFAILED: nErr = : %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    
//    strncpy(szFirmware, szResp+7, nMaxStrSize); // copy the whole string, including the date
    strncpy(szFirmware, szResp+7, 6); szFirmware[6] = 0;  // or without the date
    
    iMajorFirmwareVersion = atoi(&szFirmware[1]);  // expected string is V4.40a, for example

    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
// Get the side of the pier for the OTA
// nPierSide is 0 for East and 1 for West, or -1 if the response was not understood
//
// Firmware Validity: >= 4
//

int CPulsar2Controller::getSideOfPier(int &nPierSide)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YGN#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
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
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::GetSideOfPier] nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
   }
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::getRefractionCorrection(bool &bRefractionNeeded)
// Find out if the Pulsar2 is correcting for refraction or not.
// This can be changed by the user (though not via TSX) so is, in
// principle, unknown
// The parameter bRefractionNeeded is
//      true if it is NOT applied
//      false if it is
//
// Firmware Validity: >= 4
//
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YGR#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::getRefractionCorrection] Refraction Correction response: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
 
    // Response is 0 if the refraction correction is off, and 1 if it is on
    // To complicate matters it can be switched for each axis independently,
    // but we assume here it is always the same for both.
    //
    // Therefore, as baseline, the recommendation must be to switch it off.
    if (szResp[0] == '0')
        bRefractionNeeded = true;
    else if (szResp[0] == '1')
        bRefractionNeeded = false;
    else {
        bRefractionNeeded = true;  // it has to be one or the other ...
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getRefractionCorrection] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
}
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::getNbSlewRates()
//
// Firmware Validity: >= 4
//
{
    return NB_SLEW_SPEEDS;
}

//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::getRateName(int nZeroBasedIndex, char *pszOut, unsigned int nOutMaxSize)
//
// Firmware Validity: >= 4
//
{
    if (nZeroBasedIndex > NB_SLEW_SPEEDS)
        return BAD_CMD_RESPONSE;
    
    strncpy(pszOut, m_aszSlewRateNames[nZeroBasedIndex], nOutMaxSize);
    
    return OK;
}




////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - Set parameters
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::setDateAndTime()
//
// Sets the Pulsar2 time and date to TSX's time (NB: Pulsar2 expects UTC)
//
// Firmware Validity: >= 4
//

{
    int nErr = OK;
    char szCommand[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    
    char pszOut[N_OUT_MAXSIZE];
    
    // get the TSX time as UTC in an ISO8601 string. Note there are several variants:
    //  2018-11-06T19:40:18+00:00
    //  2018-11-06T19:40:18Z
    //  20181106T194018Z
    // It's probably the second of these, but by just using the string after the "T"
    // we can be relatively immune to variations.
    m_pTsx->utInISO8601(pszOut, N_OUT_MAXSIZE);
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setDateAndTime] ISO8601 string: %s\n", timestamp, pszOut);
        fflush(Logfile);
#endif
    
    // extract the actual UTC timestring and datestring
    char cUtcDateString[] = "yyyy-mm-dd";     // yyyy-mm-dd
    char cUtcUsDateString[] = "mm/dd/yy";     // mm/dd/yy
    char cUtcTimeString[] = "hh:mm:ss";       // hh:mm:ss
    
    strncpy(cUtcDateString, pszOut, 10);
    // convert to the US format expected by the set date command
    // do it the hard way by copying bytes, since it's too messy otherwise
    // Note that the '/' characters are already in place
    cUtcUsDateString[0] = cUtcDateString[5];
    cUtcUsDateString[1] = cUtcDateString[6];
    cUtcUsDateString[3] = cUtcDateString[8];
    cUtcUsDateString[4] = cUtcDateString[9];
    cUtcUsDateString[6] = cUtcDateString[2];
    cUtcUsDateString[7] = cUtcDateString[3];
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::setDateAndTime] Date string: %s\n", timestamp, cUtcUsDateString);
    fflush(Logfile);
#endif

    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    // send date
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:SC %s#", cUtcUsDateString);
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true);  // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setDateAndTime] Error: Set date to %s : %s\n", timestamp, cUtcUsDateString, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // Now the time
    strncpy(cUtcTimeString, pszOut+11, 8);  // this should point to the time code which starts 11 bytes in from the start
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::setDateAndTime] Time string: %s\n", timestamp, cUtcTimeString);
    fflush(Logfile);
#endif
    
    // send time
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:SL %s#", cUtcTimeString);
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1 ,true);   // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setDateAndTime] Error: Set time to %s : %s\n", timestamp, cUtcTimeString, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::setLocation()
// set's Pulsar2 latitude and longitude to the values obtained from TSX.
//
// Firmware Validity: >= 4
//
{
    int nErr = OK;
    char szCommand[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    
    double dLongitude;
    double dLatitude;
    //    double dDeg;
    //    double dMin;
    //    double dSec;
    int iDeg;
    int iMin;
    int iSec;
    
    // get longitude
    dLongitude = m_pTsx->longitude();  // TSX has the same inverted logic on longitudes (west = positive) as Pulsar2
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::setLocation] Longitude: %8.4f\n", timestamp, dLongitude);
    fflush(Logfile);
#endif

    if (dLongitude < 0.0) {dLongitude += 360.0;}
    if (dLongitude > 360.0) {dLongitude -= 360.0;}
    // This is one approach
    /*
     dMin = modf(dLongitude, &dDeg);
     dSec = modf(dMin*60.0, &dMin);
     iDeg = (int)dDeg;
     iMin = (int)dMin;
     iSec = (int)dSec*60.0;
     */
    // This is another
    iDeg = (int)floor(dLongitude);
    iMin = (int)floor((dLongitude-(double)iDeg)*60.0);
    iSec = (int)floor((((dLongitude-(double)iDeg)*60.0)-(double)iMin)*60.0);
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::setLocation] Longitude string: %03d*%02d:%02d\n", timestamp, iDeg, iMin, iSec);
    fflush(Logfile);
#endif
    
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:Sg %03d*%02d:%02d#", iDeg, iMin, iSec); // in the form #:Sg 359*29:44#
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setLocation] Error: Set longitude to %03d*%02d:%02d : %s\n", timestamp, iDeg, iMin, iSec, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // get latitude
    dLatitude = m_pTsx->latitude();
    /*
     dMin = modf(dLatitude, &dDeg);
     dSec = modf(dMin*60.0, &dMin);
     iDeg = (int)dDeg;
     iMin = (int)dMin;
     iSec = (int)dSec*60.0;
     */
    iDeg = (int)floor(dLatitude);
    iMin = (int)floor((dLatitude-(double)iDeg)*60.0);
    iSec = (int)floor((((dLatitude-(double)iDeg)*60.0)-(double)iMin)*60.0);
 
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::setLocation] Latitude: %8.4f\n", timestamp, dLatitude);
    fprintf(Logfile, "[%s] [CPulsar2Controller::setLocation] Latitude string: %+02d*%02d:%02d\n", timestamp, iDeg, iMin, iSec);
    fflush(Logfile);
#endif
    
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:St %+02d*%02d:%02d#", iDeg, iMin, iSec); // in the form #:St +43*57:58#
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setLocation] Error: Set latitude to %+02d*%02d:%02d : %s\n", timestamp, iDeg, iMin, iSec, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::setRAdec(const double &dRA, const double &dDec)
// Function used to set the commanded RA and dec
//      1. define commanded RA
//      2. define commanded Dec
//
// Firmware Validity: >= 4
//
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
    
    // first the RA
    
    iRAhh = (int)floor(dRA);
    iRAmm = (int)floor((dRA-(double)iRAhh)*60.0);
    iRAss = (int)floor((((dRA-(double)iRAhh)*60.0)-(double)iRAmm)*60.0);
    
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:Sr %02i:%02i:%02i#", iRAhh, iRAmm, iRAss);
    
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRAdec] Error setRAdec to Ra = %3.2f : %s\n", timestamp, dRA, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    if (szResp[0] != '1') {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRAdec] Error setRAdec to Ra = %3.2f : %s\n", timestamp, dRA, szResp);
        fflush(Logfile);
#endif
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRAdec] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
        
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
    
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:Sd %1c%02i*%02i:%02i#", cDecs, iDechh, iDecmm, iDecss);
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRAdec] Error setRAdec to Dec = %3.2f : %s\n", timestamp, dDec, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    if (szResp[0] != '1') {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRAdec] Error setRAdec to Dec = %3.2f : %s\n", timestamp, dDec, szResp);
        fflush(Logfile);
#endif
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRAdec] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::setTrackingRate(int nRate)
//
// Function used to set the tracking rate
//
// Possible rates are:
//  0,0 = stop
//  1,0 = sidereal
//  2,0 = lunar
//  3,0 = solar
//  4,0 = user1
//  5,0 = user2
//  6,0 = user3
//
// Firmware Validity: >= 4
//

{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCommand[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSS%d,0#", nRate);
    
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setTrackingRate] Error setting tracking to %d : %s\n", timestamp, nRate, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    if (szResp[0] != '1'){
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setTrackingRate] Error setting tracking to %d : %s\n", timestamp, nRate, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::setTrackingRate] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::setRefractionCorrection(bool bEnabled)
//
// Function used to enable or disable the refraction correction
// Command argument 0 = off; 1 = on
// Response is ambiguous. Probably 1 = OK; 0 = NOK
//
// Firmware Validity: >= 4
//

{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    if (bEnabled)
        nErr = sendCommand("#:YSR1,1#", szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    else
        nErr = sendCommand("#:YSR0,0#", szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRefractionCorrection] Error setting correction: %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if(szResp[0] == '0') {
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setRefractionCorrection] ERR_CMDFAILED: szResp[0] = %c: Err = %d\n", timestamp, szResp[0], nErr);
        fflush(Logfile);
#endif
    }
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::commandTubeSwap()
//
// Command a tube swap crossing the meridian
//
// Firmware Validity: >= 4
//

{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;

#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::commandTubeSwap] Send #:YW# Swap Tube\n", timestamp);
    fflush(Logfile);
#endif
        nErr = sendCommand("#:YW#", szResp, SERIAL_BUFFER_SIZE, 1, false);
    
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::commandTubeSwap] Error in command: %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if(szResp[0]!='1')
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::commandTubeSwap] ERR_CMDFAILED: response = %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    return nErr;
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - Movement Rates
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
int         CPulsar2Controller::getGuideRates(int &iRa, int &iDec)
// set Guide Rate in RA and Dec
//
//
// Firmware Validity:  4.xx response has 4 characters for each field
// Firmware Validity:  5.xx response has 3 characters for each field
//

{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    const char comma[] = ",";
    int commaLength;
    char cRaSpeed[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YGA#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getGuideRates] Error getting Guide Rates : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // response is of type nnnn,nnnn (with # removed). Decode the two fields
    commaLength = (int)strcspn(szResp, comma); // commaLength is the length of the string before the comma
    if (commaLength >= strlen(szResp)) {  // ie if no comma found
        nErr = ERR_CMDFAILED;
    #if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getGuideRates] Error getting Guide Rates. Response : %s\n", timestamp, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::getGuideRates] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
    #endif
        return nErr;
    }

    strncpy(cRaSpeed, szResp, commaLength);
    iRa = atoi(cRaSpeed);
    
    iDec = atoi(&szResp[commaLength+1]);
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::getGuideRates] Guide Rates results : RA : %d, Dec : %d\n", timestamp, iRa, iDec);
    fflush(Logfile);
#endif

    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int         CPulsar2Controller::setGuideRates(int iRa, int iDec)
// set Guide Rate in RA and Dec
//
//
// Firmware Validity: >= 4
//

{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCommand[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSA%d,%d#", iRa, iDec);
    
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setGuideRates] Error setting Guide Rates to %d, %d : %s\n", timestamp, iRa, iDec, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    if (szResp[0] != '1'){
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setGuideRates] Error setting Guide Rates to %d, %d : %s\n", timestamp, iRa, iDec, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::setGuideRates] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int         CPulsar2Controller::getCentreRates(int &iRa, int &iDec)
// set Guide Rate in RA and Dec
//
//
// Firmware Validity:  4.xx response has 4 characters for each field
// Firmware Validity:  5.xx response has 3 characters for each field
//

{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    const char comma[] = ",";
    int commaLength;
    char cRaSpeed[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YGB#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getCentreRates] Error getting Centre Rates : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // response is of type nnnn,nnnn (with # removed). Decode the two fields
    commaLength = (int)strcspn(szResp, comma); // commaLength is the length of the string before the comma
    if (commaLength >= strlen(szResp)) {  // ie if no comma found
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getCentreRates] Error getting Centre Rates. Response : %s\n", timestamp, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::getCentreRates] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    strncpy(cRaSpeed, szResp, commaLength);
    iRa = atoi(cRaSpeed);
    
    iDec = atoi(&szResp[commaLength+1]);
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::getCentreRates] Centre Rates results : RA : %d, Dec : %d\n", timestamp, iRa, iDec);
    fflush(Logfile);
#endif
    
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
int         CPulsar2Controller::setCentreRates(int iRa, int iDec)
// set Centre Rate in RA and Dec
//
//
// Firmware Validity: >= 4
//
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCommand[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSB%d,%d#", iRa, iDec);
    
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setCentreRates] Error setting Centre Rates to %d, %d : %s\n", timestamp, iRa, iDec, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    if (szResp[0] != '1'){
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setCentreRates] Error setting Centre Rates to %d, %d : %s\n", timestamp, iRa, iDec, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::setCentreRates] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int         CPulsar2Controller::getFindRates(int &iRa, int &iDec)
// set Guide Rate in RA and Dec
//
//
// Firmware Validity:  4.xx response has 4 characters for each field
// Firmware Validity:  5.xx response has 3 characters for each field
//

{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    const char comma[] = ",";
    int commaLength;
    char cRaSpeed[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YGC#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getFindRates] Error getting Find Rates : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // response is of type nnnn,nnnn (with # removed). Decode the two fields
    commaLength = (int)strcspn(szResp, comma); // commaLength is the length of the string before the comma
    if (commaLength >= strlen(szResp)) {  // ie if no comma found
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getFindRates] Error getting Find Rates. Response : %s\n", timestamp, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::getFindRates] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    strncpy(cRaSpeed, szResp, commaLength);
    iRa = atoi(cRaSpeed);
    
    iDec = atoi(&szResp[commaLength+1]);
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::getFindRates] Find Rates results : RA : %d, Dec : %d\n", timestamp, iRa, iDec);
    fflush(Logfile);
#endif
    
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
int         CPulsar2Controller::setFindRates(int iRa, int iDec)
// set Find Rate in RA and Dec
//
//
// Firmware Validity: >= 4
//
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCommand[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSC%d,%d#", iRa, iDec);
    
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setFindRates] Error setting Find Rates to %d, %d : %s\n", timestamp, iRa, iDec, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    if (szResp[0] != '1'){
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setFindRates] Error setting Find Rates to %d, %d : %s\n", timestamp, iRa, iDec, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::setFindRates] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int         CPulsar2Controller::getSlewRates(int &iRa, int &iDec)
// set Guide Rate in RA and Dec
//
//
// Firmware Validity:  4.xx response has 4 characters for each field
// Firmware Validity:  5.xx response has 3 characters for each field
//

{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    const char comma[] = ",";
    int commaLength;
    char cRaSpeed[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YGD#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getSlewRates] Error getting Slew Rates : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // response is of type nnnn,nnnn (with # removed). Decode the two fields
    commaLength = (int)strcspn(szResp, comma); // commaLength is the length of the string before the comma
    if (commaLength >= strlen(szResp)) {  // ie if no comma found
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getSlewRates] Error getting Slew Rates. Response : %s\n", timestamp, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::getSlewRates] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    strncpy(cRaSpeed, szResp, commaLength);
    iRa = atoi(cRaSpeed);
    
    iDec = atoi(&szResp[commaLength+1]);
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::getSlewRates] Slew Rates results : RA : %d, Dec : %d\n", timestamp, iRa, iDec);
    fflush(Logfile);
#endif
    
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
int         CPulsar2Controller::setSlewRates(int iRa, int iDec)
// set Slew Rate in RA and Dec
//
//
// Firmware Validity: >= 4
//
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCommand[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSD%d,%d#", iRa, iDec);
    
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setSlewRates] Error setting Slew Rates to %d, %d : %s\n", timestamp, iRa, iDec, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    if (szResp[0] != '1'){
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setSlewRates] Error setting Slew Rates to %d, %d : %s\n", timestamp, iRa, iDec, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::setSlewRates] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int         CPulsar2Controller::getGoToRates(int &iRa, int &iDec)
// set Guide Rate in RA and Dec
//
//
// Firmware Validity:  4.xx response has 4 characters for each field
// Firmware Validity:  5.xx response has 3 characters for each field
//

{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    const char comma[] = ",";
    int commaLength;
    char cRaSpeed[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YGE#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getGoToRates] Error getting GoTo Rates : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // response is of type nnnn,nnnn (with # removed). Decode the two fields
    commaLength = (int)strcspn(szResp, comma); // commaLength is the length of the string before the comma
    if (commaLength >= strlen(szResp)) {  // ie if no comma found
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::getGoToRates] Error getting GoTo Rates. Response : %s\n", timestamp, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::getGoToRates] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    strncpy(cRaSpeed, szResp, commaLength);
    iRa = atoi(cRaSpeed);
    
    iDec = atoi(&szResp[commaLength+1]);
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::getSlewRates] GoTo Rates results : RA : %d, Dec : %d\n", timestamp, iRa, iDec);
    fflush(Logfile);
#endif
    
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int         CPulsar2Controller::setGoToRates(int iRa, int iDec)
// set GoTo Rate in RA and Dec
//
//
// Firmware Validity: >= 4
//
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCommand[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSE%d,%d#", iRa, iDec);
    
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setGoToRates] Error setting GoTo Rates to %d, %d : %s\n", timestamp, iRa, iDec, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    if (szResp[0] != '1'){
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::setGoToRates] Error setting GoTo Rates to %d, %d : %s\n", timestamp, iRa, iDec, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::setGoToRates] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    return nErr;
}



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - Synchronisation
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// syncRaDec
//   Uses the Meade LX200 command "Sync Control", which is undocumented in the
//   Pulsar2 docs, but which does work (tested). Scenario:
//      1. define and slew to target using normal commands
//      2. adjust the centring using move or joystick controls
//      3. Calibrate mount (sync): current RA/Dec becomes commanded RA/Dec
//
//  The response to the command is to echo back the RA and Dec values set, but
//  testing has shown they have errors of about 1 arcsec.
//
//  Important: assumes the mount is on the corect side of the pier (not sure
//  this warning came from)
//
//  Note that SyncMountInterface::syncMount(const double & ra, const double & dec)
//  passes an RA, Dec and we assume these are the values from step 1 above. However,
//  to avoid doubt (and this seems to be supported by the description in
//  NeedsRefractionInterface), we will first set the RA and Dec of the target to the
//  passed values.
//
//  Also note that the refraction correction can be toglged on and off, so we will always
//  have to query this.
//
// Firmware Validity: >= 4
//

int CPulsar2Controller::syncRADec(const double &dRa, const double &dDec)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = setRAdec(dRa, dDec);  // function to set the commanded RA and dec
    if(nErr)
        return nErr;
    
    nErr = sendCommand("#:CM#", szResp, SERIAL_BUFFER_SIZE, 2);
    if(nErr)
        return nErr;
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec] Response to SyncRADec / Calibrate Mount command: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    
    // response echoes the synced coordinates back, unlike the LX200 or iEQ30 version
    // so decode these and check against the command to verify success
    // Note that some small error is likely.
    char raString[SERIAL_BUFFER_SIZE];  // "HH:MM:SS#";
    char decString[SERIAL_BUFFER_SIZE]; // "sDD:MM:SS#";
    strncpy(raString, szResp, 8);
    strncpy(decString, szResp+9, 9);
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec]Converted raString : %7.4f\n", timestamp, raStringToDouble(raString));
    fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec]Converted decString : %8.4f\n", timestamp, decStringToDouble(decString));
    fflush(Logfile);
#endif
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    // print out the deltas
    double deltaRA;  // in arcsec
    double deltaDec;  // in arcsec
    deltaRA = fabs(raStringToDouble(raString) - dRa) * 3600.0;
    deltaDec = fabs(decStringToDouble(decString) - dDec) *3600.00;
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec]delta RA : %7.4f\n", timestamp, deltaRA);
    fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec]delta RA : %8.4f\n", timestamp, deltaDec);
    fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec]nErr : %d\n", timestamp, nErr);
    fflush(Logfile);
#endif
    
    // we will allow a tolerance on each axis
#define TOLERANCE_ARCSEC       5.0
    if ((fabs(raStringToDouble(raString) - dRa)*3600.0 < TOLERANCE_ARCSEC) &&
        (fabs(decStringToDouble(decString) - dDec)*3600.0 < TOLERANCE_ARCSEC)) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec] Calibrate Mount command succeeded. nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
    }
    else {
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::SyncRADec] ERR_CMDFAILED. Calibrate Mount command failed. nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
    }
    
    
    return nErr;
    
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - Motion (Slewing)
//////////////////////////////////////////////////////////////////////////////
//
// Note that there is no method in the SlewToInterface to stop an on-going slew.
//

//////////////////////////////////////////////////////////////////////////////
//   three steps are needed:
//      1. define commanded RA
//      2. define commanded Dec
//      3. command the movement
//
// Firmware Validity: >= 4
//
int CPulsar2Controller::startSlew(const double& dRa, const double& dDec)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    // first check if there is a slew in progress
    nErr = sendCommand(":YGi#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::startSlew] Error determining IsSlewing : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // Response is 1 if mount is slewing, 0 if not
    if (szResp[0]=='1') {
        // mount is already slewing
        return ERR_CMD_IN_PROGRESS_MNT;
    }

    // OK, now it's clear to do the slew
    
    nErr = setRAdec(dRa, dDec);  // function to set the commanded RA and dec
    if(nErr)
        return nErr;
    
    nErr = sendCommand(":MS#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::startSlew] Error startSlew to %3.2f ; %3.2f : %s\n", timestamp, dRa, dDec, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // command returns 0# if the slew is OK, and #1 if there's a problem (typically, target below horizon)
    // (note that this was the wrong way around in the previous version)
    if (szResp[0] != '0')
        return ERR_LX200DESTBELOWHORIZ;  // below horizon
    
    // if all is well clear any previous flag preventing a tube swap
    swapTubeCommandIssued = false;

    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::slewStatus(bool &bIsSlewing)
//
// Firmware Validity: >= 4
//
{
    
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YGi#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
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


////////////////////////////////////////////////////////////////////////////
// Halt current slew. It's not called up in the SlewToInterface
int CPulsar2Controller::stopSlew(void)
//
// Firmware Validity: >= 4
//
{
    int nErr = OK;
//    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand(":Q#");
    if(nErr)
        return nErr;
    
/* -- not sure why this was here. No response is expected !
 
    // if result isn't 1 we're in trouble. Try again,
    // but only once
    if (szResp[0] != '1') {
        nErr = sendCommand(":Q#", szResp, SERIAL_BUFFER_SIZE);
        if(nErr)
            return nErr;
        if (szResp[0] != '1') {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_ALL
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CPulsar2Controller::stopSlew] Error in response : %s\n", timestamp, szResp);
            fflush(Logfile);
#endif
        }
    }
*/
    
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_RESULTS
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::stopSlew] Stopped OK\n", timestamp);
    fflush(Logfile);
#endif
    
    return nErr;
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - Motion (Open Loop)
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::startMove(int iDir, int iSpeedIndex)
//
// Unlike the previous version where we allowed 9 separate speeds, all set by
// reprogramming the Centre speed, here we make use of the 4 native speed
// selections. The speed associated with each can be set by command, but
// there is no TSX interface for that.The main way to do it then is by
// using the handbox (under User Parameter Setup)
//
// Firmware Validity: >= 4
//
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
    //      0 = Guide
    //      1 = Centre
    //      2 = Find
    //      3 = Slew
    
/*-----------------------------------------------------------------------------------------------------
    // This was a previous approach
    //-------
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
    
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
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
//-----------------------------------------------------------------------------------------------------
 */
 
    // set the required speed command
    switch (iSpeedIndex)
    {
        default:
        case 0: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:RG#");       break;    // "Guide"
        case 1: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:RC#");       break;    // "Centre"
        case 2: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:RM#");       break;    // "Find"
        case 3: snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:RS#");       break;    // "Slew"
     }

    
    // command the selected speed
    nErr = sendCommand(szCommand);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::startMove] Error selecting speed : %s\n", timestamp, szResp);
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
    
    nErr = sendCommand(szCommand);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::startMove] Error commanding move : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // no response expected
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::stopMoving(int iDir)
//
// Firmware Validity: >= 4
//
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

    // stop the move
    switch (iDir) {
        default:
        case 0: snprintf(szCommand, SERIAL_BUFFER_SIZE, ":Qn#");   break; // North
        case 1: snprintf(szCommand, SERIAL_BUFFER_SIZE, ":Qs#");   break; // South
        case 2: snprintf(szCommand, SERIAL_BUFFER_SIZE, ":Qe#");   break; // East
        case 3: snprintf(szCommand, SERIAL_BUFFER_SIZE, ":Qw#");   break; // West
    }
    
    nErr = sendCommand(szCommand);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::stopMove] Error stopping move : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    return nErr;
    
}


//////////////////////////////////////////////////////////////////////////////
// Function used to turn tracking off
// Returns true if successful, false if not

int CPulsar2Controller::trackingOff(void)
//
// Firmware Validity: >= 4
//
{
    int nErr = OK;
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = setTrackingRate(STOPPED); // this stops the tracking
    
    return nErr;
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - Parking
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Function to park the mount
// Returns true if successful, false if not
//
// The implication of the arguments is that TSX will set the Az and Alt of the park position
// However, by virtue of the DriverSlewsToParkPositionInterface it won't actually
// do it, so we can safely ignore these parameters.
//
// Firmware Validity: >= 4
//

int CPulsar2Controller::park(const double& dAz, const double& dAlt)
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    // first check if a park position has been defined
    bool bIsParkDefined;
    nErr = parkIsParkDefined(bIsParkDefined);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Park] Error getting IsHomeSet : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
if (!bIsParkDefined)
    return ERR_NOPARKPOSITION;
    
    nErr = sendCommand("#:YH#", szResp, SERIAL_BUFFER_SIZE);    // park
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Park] Error parking mount : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if (szResp[0] != '1'){
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::Park] Error parking mount : %s\n", timestamp, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::Park] ERR_CMDFAILED. nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    return nErr;
}


//////////////////////////////////////////////////////////////////////////////
// Function to ask if the mount is parked
// Also used to see if parking is complete
// set isParked to true if parked, false if not
//
// Firmware Validity: >= 4
//
int CPulsar2Controller::parkIsParked(bool &isParked)
{
    
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;

    nErr = sendCommand("#:YGk#", szResp, SERIAL_BUFFER_SIZE);    // IsParked ?
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::parkIsParked] Error determining IsParked : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
   
    // Response is 1 if mount is parked, 0 if not
    m_bIsParked = szResp[0]=='1'?true:false;
    isParked = m_bIsParked;
    
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
// Function to ask if the mount is parking
// set isParking to true if park is on-going, false if not
//
// Firmware Validity: >= 4
//
int CPulsar2Controller::parkIsParking(bool &isParking)
{
    
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YGj#", szResp, SERIAL_BUFFER_SIZE);    // IsParking ?
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::parkIsParking] Error determining IsParking : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // Response is 1 if mount is parking, 0 if not
    isParking = szResp[0]=='1'?true:false;
    
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
// Function to ask if the mount has a park position defined
// set isParkSet to true if park posiiton is defined, false if not
//
// Firmware Validity: >= 4
//
int CPulsar2Controller::parkIsParkDefined(bool &isParkSet)
{
    
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YGh#", szResp, SERIAL_BUFFER_SIZE);    // IsHomeSet ?
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::parkIsParkDefined] Error determining IsHomeSet : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // Response is 1 if park is defined, 0 if not
    isParkSet = szResp[0]=='1'?true:false;
    
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
// Function to set the current posiiton as the park position
// It's based on Alt/Az coordinates
//
// Firmware Validity: >= 4
//
int CPulsar2Controller::parkSetParkPosition()
{
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCommand[SERIAL_BUFFER_SIZE];
    double dAlt, dAz;

    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    // get current alt/az
    nErr = getAltAz(dAlt, dAz);
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::parkSetParkPosition] Error determining alt/az position : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // set park alt/az
    snprintf(szCommand, SERIAL_BUFFER_SIZE, "#:YSX%+08.4f,%8.4f#", dAlt, dAz);
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CPulsar2Controller::parkSetParkPosition] Command sent is : %s\n", timestamp, szCommand);
    fflush(Logfile);
#endif
    nErr = sendCommand(szCommand, szResp, SERIAL_BUFFER_SIZE, 1, true); // not extra # in response so 1 reponse read, 1 byte response.
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::parkSetParkPosition] Error setting position : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    
    return nErr;
}




////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - UnParking
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Function to unpark the mount
// Returns true if successful, false if not
//
// Firmware Validity: >= 4
//
int CPulsar2Controller::unPark()
{
    
    int nErr = OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return ERR_NOLINK;
    
    nErr = sendCommand("#:YL#", szResp, SERIAL_BUFFER_SIZE);    // unpark
    if(nErr) {
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::unPark] Error unparking mount : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if (szResp[0] != '1'){
        nErr = ERR_CMDFAILED;
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_FAILURES
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CPulsar2Controller::unPark] Error unparking mount : %s\n", timestamp, szResp);
        fprintf(Logfile, "[%s] [CPulsar2Controller::unPark] ERR_CMDFAILED: nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    swapTubeCommandIssued = false;

    return nErr;
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - String conversions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
double CPulsar2Controller::raStringToDouble(char* cRaString)
{
    double dRA = -999.0;
    double dHours;
    double dMinutes;
    double dSeconds;
    
    //  raString is in the form "HH:MM:SS"
    
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


//////////////////////////////////////////////////////////////////////////////
double CPulsar2Controller::decStringToDouble(char* cDecString)
{
    double dDec = -999.0;
    double dDegrees;
    double dMinutes;
    double dSeconds;
    double dSign; // = +1.00 or -1.00
    
    //  decString is in the form "sDD*MM:SS"
    //            or in the form "sDD*MM'SS"
    
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

//////////////////////////////////////////////////////////////////////////////
double CPulsar2Controller::azStringToDouble(char* cAzString)
{
    double dAz = -999.0;
    double dDegrees;
    double dMinutes;
    double dSeconds;
//    double dSign; // = +1.00 or -1.00
    
    //  azString is in the form "DDD*MM:SS" eg 196ﬂ26:28#, where the * is 0xdf
    
 // degrees (0 -- 359)
    sscanf((char *) cAzString, "%3lf", &dDegrees);
    // minutes
    sscanf((char *) cAzString, "%*4c %2lf", &dMinutes);
    // seconds
    sscanf((char *) cAzString, "%*7c %2lf", &dSeconds);
    
    dAz = dDegrees + dMinutes/60.0 + dSeconds/3600.0;
    
    return dAz;
    
}


//////////////////////////////////////////////////////////////////////////////
int CPulsar2Controller::parseFields(const char *pszIn, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = OK;
    std::string sSegment;
    std::stringstream ssTmp(pszIn);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_BADFORMAT;
    }
    return nErr;
}

//////////////////////////////////////////////////////////////////////////////
void        CPulsar2Controller::logMessage(char* cMethod, char* cMessage)
{
#if defined PULSAR2_DEBUG && PULSAR2_DEBUG >= VERBOSE_ALL
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [%s] %s\n", timestamp, cMethod, cMessage);
    fflush(Logfile);
#endif

}



