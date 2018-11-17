//  modified by Richard Francis, 28 Oct 2018


#include "x2mount.h"

X2Mount::X2Mount(const char* pszDriverSelection,
                 const int& nInstanceIndex,
                 SerXInterface                    * pSerX,
                 TheSkyXFacadeForDriversInterface    * pTheSkyX,
                 SleeperInterface                    * pSleeper,
                 BasicIniUtilInterface            * pIniUtil,
                 LoggerInterface                    * pLogger,
                 MutexInterface                    * pIOMutex,
                 TickCountInterface                * pTickCount)
{
	m_nPrivateMulitInstanceIndex	            = nInstanceIndex;
	m_pSerX							= pSerX;		
	m_pTheSkyXForMounts				= pTheSkyX;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;	
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;

	m_bLinked = false;
    m_nParity = SerXInterface::B_NOPARITY;

    // set pointers to some of the interfaces inside the Pulsar2 class
    Pulsar2.setSleeper(pSleeper);  // added by Rodolphe in iEQ30
    Pulsar2.setSerxPointer(pSerX);
    Pulsar2.setLoggerPointer(pLogger);
    Pulsar2.setTSX(pTheSkyX);

    // Read in settings
    if (m_pIniUtil)
    {
        Pulsar2.dHoursEastStored = m_pIniUtil->readDouble(PARENT_KEY_STRING, PULSAR2_HOURS_EAST, 0.0);
        Pulsar2.dHoursWestStored = m_pIniUtil->readDouble(PARENT_KEY_STRING, PULSAR2_HOURS_WEST, 0.0);
        Pulsar2.dFlipHourStored = m_pIniUtil->readDouble(PARENT_KEY_STRING, PULSAR2_FLIPHOUR, 0.0);
        Pulsar2.iMeridianBehaviourStored = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_MERIDIAN, 1);

        Pulsar2.dGuideRateRAStored = m_pIniUtil->readDouble(PARENT_KEY_STRING, PULSAR2_GUIDE_RA, 0.3);
        Pulsar2.dGuideRateDecStored = m_pIniUtil->readDouble(PARENT_KEY_STRING, PULSAR2_GUIDE_DEC, 0.3);
        Pulsar2.iCentreRateRAStored = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_CENTRE_RA, 20);
        Pulsar2.iCentreRateDecStored = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_CENTRE_DEC, 20);
        Pulsar2.iFindRateRAStored = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_FIND_RA, 100);
        Pulsar2.iFindRateDecStored = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_FIND_DEC, 100);
        Pulsar2.iSlewRateRAStored = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_SLEW_RA, 750);
        Pulsar2.iSlewRateDecStored = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_SLEW_DEC, 750);
        Pulsar2.bSyncTimeOnConnectStored = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_SYNC_TIME, 0);
        Pulsar2.bSyncLocationOnConnectStored = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_SYNC_LOCATION, 0);
        
    }
    

//    Pulsar2.iVerbosity = iLoggingVerbosity;
    iLoggingVerbosity = Pulsar2.iVerbosity;
    iRateIndex = 0;  // added by Rodolphe in iEQ30

}



X2Mount::~X2Mount()
{
	//Delete objects used through composition
	if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyXForMounts)
		delete m_pTheSkyXForMounts;
	if (m_pSleeper)
		delete m_pSleeper;
	if (m_pIniUtil)
		delete m_pIniUtil;
	if (m_pLogger)
		delete m_pLogger;
	if (m_pIOMutex)
		delete m_pIOMutex;
	if (m_pTickCount)
		delete m_pTickCount;

}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - DriverRootInterface
////////////////////////////////////////////////////////////////////////////
int	X2Mount::queryAbstraction(const char* pszName, void** ppVal)
{
	*ppVal = NULL;

	if (!strcmp(pszName, SyncMountInterface_Name))
		*ppVal = dynamic_cast<SyncMountInterface*>(this);
	else if (!strcmp(pszName, SlewToInterface_Name))
		*ppVal = dynamic_cast<SlewToInterface*>(this);
	else if (!strcmp(pszName, AsymmetricalEquatorialInterface_Name))
		*ppVal = dynamic_cast<AsymmetricalEquatorialInterface*>(this);
	else if (!strcmp(pszName, OpenLoopMoveInterface_Name))
		*ppVal = dynamic_cast<OpenLoopMoveInterface*>(this);
	else if (!strcmp(pszName, NeedsRefractionInterface_Name))
		*ppVal = dynamic_cast<NeedsRefractionInterface*>(this);
	else if (!strcmp(pszName, TrackingRatesInterface_Name))
		*ppVal = dynamic_cast<TrackingRatesInterface*>(this);
	else if (!strcmp(pszName, ParkInterface_Name))
		*ppVal = dynamic_cast<ParkInterface*>(this);
	else if (!strcmp(pszName, UnparkInterface_Name))
		*ppVal = dynamic_cast<UnparkInterface*>(this);
    else if (!strcmp(pszName, DriverSlewsToParkPositionInterface_Name))
        *ppVal = dynamic_cast<DriverSlewsToParkPositionInterface*>(this);  // added by CRF 3 Nov 2018
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
	else if (!strcmp(pszName, LoggerInterface_Name)) //Add support for the optional LoggerInterface
		*ppVal = GetLogger();
    else if (!strcmp(pszName, LinkInterface_Name))
		*ppVal = dynamic_cast<LinkInterface*>(this);
	else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
		*ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
	else if (!strcmp(pszName, X2GUIEventInterface_Name))
		*ppVal = dynamic_cast<X2GUIEventInterface*>(this);


	return SB_OK;
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - LinkInterface
////////////////////////////////////////////////////////////////////////////
int	X2Mount::establishLink(void)
{
    int nErr;
    char szPort[DRIVER_MAX_STRING];

	X2MutexLocker ml(GetMutex());

    if (iLoggingVerbosity >= VERBOSE_ESSENTIAL)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::establishLink");

    
     // get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);
    nErr = Pulsar2.connect(szPort);
    if(!nErr)
        {
        m_bLinked = true;
            if (iLoggingVerbosity >= VERBOSE_ESSENTIAL)
                if (GetLogger())
                    GetLogger()->out((char *) "X2Mount::establishLink - connected OK");
        return nErr;
        }
    else
        {
        m_bLinked = false;
            if (iLoggingVerbosity >= VERBOSE_FAILURES)
                if (GetLogger())
                    GetLogger()->out((char *) "X2Mount::establishLink - failed to connect");
        return nErr;
        }
}

////////////////////////////////////////////////////////////////////////////
int	X2Mount::terminateLink(void)						
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    Pulsar2.disconnect();
	m_bLinked = false;
    if (iLoggingVerbosity >= VERBOSE_ESSENTIAL)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::terminateLink - Link terminated");
	return SB_OK;
}


////////////////////////////////////////////////////////////////////////////
bool X2Mount::isLinked(void) const
{
	return m_bLinked;
}


bool X2Mount::isEstablishLinkAbortable(void) const
{
    return false;
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - DriverInfoInterface
////////////////////////////////////////////////////////////////////////////
//AbstractDriverInfo

void	X2Mount::driverInfoDetailedInfo(BasicStringInterface& str) const
{

    str = DISPLAY_NAME;
}

double	X2Mount::driverInfoVersion(void) const				
{
	return DRIVER_VERSION;
}



////////////////////////////////////////////////////////////////////////////
//AbstractDeviceInfo
void X2Mount::deviceInfoNameShort(BasicStringInterface& str) const				
{
	str = "Pulsar2";
}

////////////////////////////////////////////////////////////////////////////
void X2Mount::deviceInfoNameLong(BasicStringInterface& str) const
{
	str = "Pulsar2 Mount Controller";

}

////////////////////////////////////////////////////////////////////////////
void X2Mount::deviceInfoDetailedDescription(BasicStringInterface& str) const
{
	str = DISPLAY_NAME;
}

////////////////////////////////////////////////////////////////////////////
void X2Mount::deviceInfoFirmwareVersion(BasicStringInterface& str)
{
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::deviceInfoFirmwareVersion");

    if (m_bLinked) {
        X2MutexLocker ml(GetMutex());
        
        char szFirmware[SERIAL_BUFFER_SIZE];
        if(Pulsar2.getFirmware((char *)szFirmware, SERIAL_BUFFER_SIZE) == OK)
            str = szFirmware;
        else
            str = "Unknown";
    }
    
 else
        str = "No Connection";
}

////////////////////////////////////////////////////////////////////////////
void X2Mount::deviceInfoModel(BasicStringInterface& str)
{
	str = "Pulsar2";
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - MountDriverInterface
////////////////////////////////////////////////////////////////////////////
//Common Mount specifics
int	X2Mount::raDec(double& ra, double& dec, const bool& bCached)
{
    int nErr = SB_OK;
    char szLogMessage[LOG_BUFFER_SIZE];

    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::raDec");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = Pulsar2.getRADec(ra, dec);
    
    if (iLoggingVerbosity >= VERBOSE_RESULTS)
        if (GetLogger())
        {
            snprintf(szLogMessage, LOG_BUFFER_SIZE, "X2Mount::raDec: RA = %10f; Dec = %10f", ra, dec);
            GetLogger()->out(szLogMessage);
        }
 
	return nErr;
}

////////////////////////////////////////////////////////////////////////////
// Abort any operation currently in progress.
// The best approximation, in that it does stop motion, is to call stopSlew()
int	X2Mount::abort(void)
{
    
    if (iLoggingVerbosity >= VERBOSE_ESSENTIAL)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::Abort");

    if(!m_bLinked)
        return ERR_NOLINK;
    X2MutexLocker ml(GetMutex());

    Pulsar2.stopSlew();  // not sure if this is sufficient to stop everything
    
    return SB_OK;

}



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - SyncMountInterface
////////////////////////////////////////////////////////////////////////////
int X2Mount::syncMount(const double& ra, const double& dec)
// Set the mount internal RA and declination.
{

    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::syncMount");
    
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    if (Pulsar2.syncRADec(ra, dec) != OK)
        return ERR_CMDFAILED;

 return SB_OK;
}

////////////////////////////////////////////////////////////////////////////

bool X2Mount::isSynced()
// Always return true.
// If possible, return false when appropriate, if and only if the mount hardware
// has the ability to know if it has been synced or not.
// Pulsar2 does not, so always return true
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::isSynced");
 
    return true;
}



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - AsymmetricalEquatorialInterface
////////////////////////////////////////////////////////////////////////////
bool X2Mount::knowsBeyondThePole()
{
    
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::knowsBeyondThePole");
   
    return true;
}


////////////////////////////////////////////////////////////////////////////
int X2Mount::beyondThePole(bool& bYes)
/*!
 If knowsBeyondThePole() returns true,
 then beyondThePole() tells TheSkyX unambiguously
 if the OTA end of the declination axis
 is either east (0) or west of the pier (1).
 Note, the return value must be correct even
 for cases where the OTA end of the Dec axis
 is lower than the counterweights.
 */
{
    int nErr = SB_OK;
    int poleResult;
    
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::beyondThePole");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = Pulsar2.getSideOfPier(poleResult);
    if(nErr)
        return nErr;

    // we will define west (1) as normal, and
    // east (0) as beyond the pole
    //
    // That was wrong: https://www.gralak.com/apdriver/help/pier_side.htm explains
    // that East (here 0) is normal, and West (1) is Beyond the Pole
    bYes = poleResult == 1?true:false;

    return nErr;
 }


////////////////////////////////////////////////////////////////////////////
double X2Mount::flipHourAngle()
{
    // depending on the setting in iMeridianTreatment, the mount may or may not flip
    // automatically. But we will here return the vlaue for the flip anyway.
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::flipHourAngle");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    return Pulsar2.dFlipHourStored;

}


////////////////////////////////////////////////////////////////////////////
int X2Mount::gemLimits(double& dHoursEast, double& dHoursWest)
{

    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::gemLimits");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    dHoursEast = Pulsar2.dHoursEastStored;
    dHoursWest = Pulsar2.dHoursWestStored;
    
    return SB_OK;
}



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - NeedsRefractionInterface
////////////////////////////////////////////////////////////////////////////
//
// only one function
//
// We will ensure in the initialisation that refraction correction is turned off
// So the response will always be true
//

bool X2Mount::needsRefactionAdjustments(void)
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::needsRefractionAdjustments");
    
        return true;

}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - SlewToInterface
////////////////////////////////////////////////////////////////////////////
// NB -- seems to not be called -- Abort is called instead [don't understand this comment]

int X2Mount::startSlewTo(const double& dRa, const double& dDec)
{
    int nErr = SB_OK;
    bool bIsSlewing;
    char cLogMessage[LOG_BUFFER_SIZE];
    
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::startSlewTo");
    
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    if (iLoggingVerbosity >= VERBOSE_ESSENTIAL)
        if (GetLogger())
        {
            snprintf(cLogMessage, LOG_BUFFER_SIZE, "X2Mount::startSlewTo: RA = %10f; Dec = %10f", dRa, dDec);
            GetLogger()->out(cLogMessage);
        }

    // check if there's already a slew going on
    nErr = Pulsar2.slewStatus(bIsSlewing);
    if(nErr)
        return nErr;

    if (!bIsSlewing) {
        nErr = Pulsar2.startSlew(dRa, dDec);
        if(nErr)
            return nErr;
    }
    else
        return ERR_COMMANDINPROGRESS;
    
    return nErr;
}


////////////////////////////////////////////////////////////////////////////
int X2Mount::isCompleteSlewTo(bool& bComplete) const
{
    int nErr = SB_OK;
    bool bIsSlewing;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());

    nErr = pMe->Pulsar2.slewStatus(bIsSlewing);
    if(nErr)
        return nErr;

    bComplete = bIsSlewing?false:true;
    
    return SB_OK;
}


////////////////////////////////////////////////////////////////////////////
// This function is called after isCompleteSlewTo() shows the slew is finished.
// The documentation says:
//  Called once the slew is complete. This is called once for every corresponding
//  startSlewTo() allowing software implementations of gotos.
//
// There dowsn't seem to be any useful function to put in here.
//
int X2Mount::endSlewTo(void)
{
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::endSlewTo");
    
    return SB_OK;
   
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - OpenLoopMoveInterface
////////////////////////////////////////////////////////////////////////////
//
int X2Mount::startOpenLoopMove(const MountDriverInterface::MoveDir& Dir, const int& nRateIndex)
// Start the open-loop move.
{
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::startOpenLoopMove");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    iRateIndex = nRateIndex;  // not sure what this is good for
    
    // note the commanded direction
    currentMoveDir = Dir;

    if (Pulsar2.startMove(int(Dir), nRateIndex) == OK)
        return SB_OK;
    else
        return ERR_CMDFAILED;
    
}


////////////////////////////////////////////////////////////////////////////
//
int	X2Mount::endOpenLoopMove(void)
// End the open-loop move. This function is always called for every
// corresponding startOpenLoopMove(), allowing software implementations of the move.
//
// To do this properly we will need to keep track of the direction of the
// corresponding startOpenLoopMove(), since each stop command is specific
// We do this via the private property currentMoveDir
{
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::endOpenLoopMove");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    return Pulsar2.stopMoving((int)currentMoveDir);
}


////////////////////////////////////////////////////////////////////////////
//
bool X2Mount::allowDiagonalMoves(void)
// Return true if the mount can be commanded to move in more than one
// perpendicular axis at the same time, otherwise return false.
{
    return false;
}


////////////////////////////////////////////////////////////////////////////
//
int X2Mount::rateCountOpenLoopMove(void) const
// Return the number (count) of avaiable moves.
{
    X2Mount* pMe = (X2Mount*)this;
    return pMe->Pulsar2.getNbSlewRates();
}

////////////////////////////////////////////////////////////////////////////
int X2Mount::rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize)
// Return a string along with the amount or size of the corresponding move.
{
    int nErr = SB_OK;
    nErr = Pulsar2.getRateName(nZeroBasedIndex, pszOut, nOutMaxSize);
    if(nErr) {
        return ERR_CMDFAILED;
    }
    return nErr;
}


////////////////////////////////////////////////////////////////////////////
//
int	X2Mount::rateIndexOpenLoopMove(void)
// Return the current index of move selection.
{
    return iRateIndex;
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - TrackingRatesInterface
////////////////////////////////////////////////////////////////////////////
//
int X2Mount::setTrackingRates( const bool& bTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
/*
 - bTrackingOn
 true means turn sidereal tracking on regardless of other parameters
 false means turn it off
 
 In fact requesting Solar tracking had the folowing parameters: bTrackingOn = true, bIgnoreRates = false etc
 
 - bIgnoreRates
 
 true means ignore the content of the rates fields
 false means don't ignore them, but implement them if possible.
 
 - dRaRateArcSecPerSec
 
 for sidereal tracking the mount is maintaining constant RA, so this value is 0.0 for sidereal tracking;
 it is +15.041 068 1 for tracking off, since a positive RA rate means the mount is moving in the opposite direction to sidereal motion;
 it is + 0.549 014 9 for the mean lunar rate (lunar rate varies as the lunar orbit is rather elliptical);
 it is + 0.041 068 1 for solar rate;
 
 - dDecRateArcSecPerSec
 
 for sidereal tracking this is 0.0;
 for tracking off it is also 0.0;
 for both Lunar and Solar tracking this will be non-zero but small, as both the Moon and Earth have orbital planes inclined to the Earth's equator;

 Firmware validity:
 
 rates off, sidereal, lunar and solar: 4.xx
 rates off, sidereal, lunar, solar, user1, user2, user3: 5.xx -- these extensions to be implemented later (probably as set immediate rate)
 
 */
    int nErr;
    char szLogMessage[LOG_BUFFER_SIZE];

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::setTrackingRates");


    if (iLoggingVerbosity >= VERBOSE_RESULTS)
        if (GetLogger()) {
            snprintf(szLogMessage, LOG_BUFFER_SIZE, "X2Mount::setTrackingRates: TrkOn %s, IgnrRates %s, RArate = %3.6f; DecRate = %3.6f",
                                                bTrackingOn?"True":"False",
                                                bIgnoreRates?"True":"False",
                                                dRaRateArcSecPerSec, dDecRateArcSecPerSec);
            GetLogger()->out(szLogMessage);
            }


    if (bTrackingOn) { // set tracking on
        if (bIgnoreRates)
            nErr = Pulsar2.setTrackingRate(SIDEREAL);
        else { // do not ignore rates, but use values in numeric fields
            nErr = ERR_COMMANDNOTSUPPORTED;
            // Sidereal rate
            if (-0.00001 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.00001 && -0.00001 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.00001) {
                nErr = Pulsar2.setTrackingRate(SIDEREAL);
            }
            // Lunar rate (tolerances increased based on JPL ephemeris generator)
            else if (0.30 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.83 && -0.25 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.25) {
            //    dCommandedRAlunarRate = dRaRateArcSecPerSec;
            //    dCommandedDeclunarRate = dDecRateArcSecPerSec;
                nErr = Pulsar2.setTrackingRate(LUNAR);
            }
            // Solar rate (tolerances increased based on JPL ephemeris generator, since TSX demanded a rate outside previous tolerance)
            else if (0.037 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.043 && -0.017 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.017) {
            //    dCommandedRAsolarRate = dRaRateArcSecPerSec;
            //    dCommandedDecsolarRate = dDecRateArcSecPerSec;
                nErr = Pulsar2.setTrackingRate(SOLAR);
            }
        }
    }
    else    // set tracking off
        nErr = Pulsar2.trackingOff();

    return nErr;
}

////////////////////////////////////////////////////////////////////////////
//
int X2Mount::siderealTrackingOn()   // added by CRF 3 Nov 2018
// Turn on sidereal tracking. Provided for convenience, merely calls setTrackingRates() function.
{
    int nErr;
    nErr = SB_OK;

    X2MutexLocker ml(GetMutex());
    nErr = Pulsar2.setTrackingRate(SIDEREAL);

    return nErr;
}

////////////////////////////////////////////////////////////////////////////
//
int X2Mount::trackingOff()   // added by CRF 3 Nov 2018
// Turn off tracking. Provided for convenience, merely calls setTrackingRates() function.
{
    int nErr;
    nErr = SB_OK;

    X2MutexLocker ml(GetMutex());
    nErr = Pulsar2.setTrackingRate(STOPPED);

    return nErr;
}



////////////////////////////////////////////////////////////////////////////
//
int X2Mount::trackingRates( bool& bTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec)
{
/*
 - bTrackingOn
    true means tracking is at sidereal rate regardless of other parameters
    false means tracking rate is as given in the next 2 parameters. It does not mean that tracking is off.
 
    This may no longer be true, with X2 v21
 
 - dRaRateArcSecPerSec
 
    has no real effect: if bTrackingOn = true the rate is always reported to be sidereal;
    if bTrackingOn = false then generally the rate is reported as Custom, but note the special cases below.
 
- dDecRateArcSecPerSec
        
    has no effect, except that the hang reported below is avoided if this value is non-zero.
            
 
 Special cases:
            
    bTrackingOn = false; dRaRateArcSecPerSec = 15.0410681 +- 0.00001; dDecRateArcSecPerSec = 0 +- 0.00001 means "Tracking Off".
 
    If the mount can set tracking rate but not read it, return bTrackingOn=false and return both rates as -1000.0
 
 Nominal rates [arcsec/sec]:
 Stopped:
    RA  = 15.0410681
    Dec = 0.0
 Sidereal:
    RA  = 0.0
    Dec = 0.0
 Lunar:
    RA  = 0.5490149
    Dec = 0.0
 Solar:
    RA  = 0.0410681
    Dec = 0.0

 Firmware validity:
    - V4.xx relies on using cached values for solar and lunar rates
    - V5.xx uses the results from set immediate rate
 
*/
    int nErr = SB_OK;
    int iTrackingRate;
    
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::trackingRates");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = Pulsar2.getTracking(iTrackingRate);
    if(nErr)
        return nErr;
    
    switch (iTrackingRate) {
        case STOPPED:
            bTrackingOn = false;
            dRaRateArcSecPerSec = RA_RATE_STOPPED;
            dDecRateArcSecPerSec = DEC_RATE_STOPPED;
            if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
                if (GetLogger())
                    GetLogger()->out((char *) "X2Mount::trackingRates: response is STOPPED");
            Pulsar2.logMessage((char *) "X2Mount::trackingRates", (char *) "trackingRates: response is STOPPED");
            break;
            
        case SIDEREAL:
            bTrackingOn = true;
            dRaRateArcSecPerSec = RA_RATE_SIDEREAL;
            dDecRateArcSecPerSec = DEC_RATE_SIDEREAL;
            if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
                if (GetLogger())
                    GetLogger()->out((char *) "X2Mount::trackingRates: response is SIDEREAL");
            Pulsar2.logMessage((char *) "X2Mount::trackingRates", (char *) "trackingRates: response is SIDEREAL");
            break;

        case LUNAR:
            bTrackingOn = false;
            dRaRateArcSecPerSec = RA_RATE_LUNAR;
            dDecRateArcSecPerSec = DEC_RATE_LUNAR;
            if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
                if (GetLogger())
                    GetLogger()->out((char *) "X2Mount::trackingRates: response is LUNAR");
            Pulsar2.logMessage((char *) "X2Mount::trackingRates", (char *) "trackingRates: response is LUNAR");
            break;

        case SOLAR:
            bTrackingOn = false;
            dRaRateArcSecPerSec = RA_RATE_SOLAR;
            dDecRateArcSecPerSec = DEC_RATE_SOLAR;
            if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
                if (GetLogger())
                    GetLogger()->out((char *) "X2Mount::trackingRates: response is SOLAR");
            Pulsar2.logMessage((char *) "X2Mount::trackingRates", (char *) "trackingRates: response is SOLAR");
           break;
            
        default:
            bTrackingOn = false;
            dRaRateArcSecPerSec = RA_RATE_STOPPED;
            dDecRateArcSecPerSec = DEC_RATE_STOPPED;
            if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
                if (GetLogger())
                    GetLogger()->out((char *) "X2Mount::trackingRates: response is default (STOPPED)");
            Pulsar2.logMessage((char *) "X2Mount::trackingRates", (char *) "trackingRates: response is STOPPED");
           break;
    }
    
    return nErr;
}



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - ParkInterface
////////////////////////////////////////////////////////////////////////////
//
bool X2Mount::isParked(void)
// Return true if the device is parked.
{
    int nErr = SB_OK;

    bool bIsParked;
    
   if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::isParked");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

   nErr = Pulsar2.parkIsParked(bIsParked);

    return bIsParked;
}

////////////////////////////////////////////////////////////////////////////
int    X2Mount::isCompletePark(bool& bComplete) const
// Called to monitor the park process.
//
// bComplete    Set to true if the park is complete, otherwise set to false.
//
{
    int nErr = SB_OK;
    
    bool bIsParked;
    
    if(!m_bLinked)
        return ERR_NOLINK;
    
    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());
    nErr = pMe->Pulsar2.parkIsParked(bIsParked);

    if (bIsParked) {
        bComplete = true;
        return nErr;
    }
    bComplete = false;
    return nErr;
}


////////////////////////////////////////////////////////////////////////////
int	X2Mount::startPark(const double& dAz, const double& dAlt)
{
// Initiate the park process.
// The implication of the arguments is that TSX will set the Az and Alt of the park position
// However, by virtue of the DriverSlewsToParkPositionInterface it won't actually
// do it, so we can safely ignore these parameters.
    
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::startPark");

	X2MutexLocker ml(GetMutex());
    
    if (Pulsar2.park(dAz, dAlt) == SB_OK)
        return SB_OK;
    else
        return ERR_CMDFAILED;

}



////////////////////////////////////////////////////////////////////////////
int	X2Mount::endPark(void)
// Called once the park is complete. This is called once for every corresponding
// startPark() allowing software implementations of park.
//
// It's not necessary to put anything in here
{
    return SB_OK;
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - UnparkInterface
////////////////////////////////////////////////////////////////////////////
//
int	X2Mount::startUnpark(void)
// Initiate the park process.
{
    int nErr = SB_OK;
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::startUnpark");

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
    
    nErr = Pulsar2.unPark();

    return nErr;
    
}


////////////////////////////////////////////////////////////////////////////
int	X2Mount::isCompleteUnpark(bool& bComplete) const
// Called to monitor the unpark process.
{
    int nErr = SB_OK;
    bool bIsParked;

    // This bit throws an error: related to the use of const in the definition.
    // I don't know why it's there
    /*
     if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
     if (GetLogger())
     GetLogger()->out((char *) "X2Mount::isCompleteUnpark");
     */

    bComplete = false;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());

    // check park status
    nErr = pMe->Pulsar2.parkIsParked(bIsParked);
    if(nErr)
        return nErr;

    if (bIsParked) {
        bComplete = false;
        return nErr;
    }
    bComplete = true;
    return nErr;
    

}


////////////////////////////////////////////////////////////////////////////
int	X2Mount::endUnpark(void)
// Called once the unpark is complete. This is called once for every corresponding
// startUnpark() allowing software implementations of unpark.
//
// It's not necessary to put anything in here
{
    return SB_OK;
}



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - ModalSettingsDialogInterface
// handle dialog box
//
int X2Mount::execModalSettingsDialog(void)
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this,      m_pTheSkyXForMounts);
    X2GUIInterface*					ui = uiutil.X2UI();
    X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
    bool bPressedOK = false;
    
    if (NULL == ui)
        return ERR_POINTER;
    
    if ((nErr = ui->loadUserInterface("Pulsar2.ui", deviceType(), m_nPrivateMulitInstanceIndex)))
        return nErr;
    
    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;
    
    
    dx->setEnabled("groupBox_4", true);
    
    //-----------------------------------------------------
    //  Set the UI dialog in accordance with stored values
    
    //  Meridian Behaviour
    switch (Pulsar2.iMeridianBehaviourStored)
    {
        case 0:
            dx->setChecked("radioButton_Stop", true);
            break;
        case 1:
            dx->setChecked("radioButton_Flip", true);
            break;
        case 2:
            dx->setChecked("radioButton_Continue_Stop", true);
            break;
        case 3:
            dx->setChecked("radioButton_Continue_Flip", true);
            break;
    }

    //  Meridian Values
    dx->setPropertyDouble("doubleSpinBox_HoursEast", "value", Pulsar2.dHoursEastStored);
    dx->setPropertyDouble("doubleSpinBox_HoursWest", "value", Pulsar2.dHoursWestStored);
    dx->setPropertyDouble("doubleSpinBox_HourFlip", "value", Pulsar2.dFlipHourStored);
    
    //  Set Rates
    dx->setPropertyDouble("doubleSpinBox_Guide_RA", "value", Pulsar2.dGuideRateRAStored);
    dx->setPropertyDouble("doubleSpinBox_Guide_Dec", "value", Pulsar2.dGuideRateDecStored);
    dx->setPropertyInt("spinBox_Centre_RA", "value", Pulsar2.iCentreRateRAStored);
    dx->setPropertyInt("spinBox_Centre_Dec", "value", Pulsar2.iCentreRateDecStored);
    dx->setPropertyInt("spinBox_Find_RA", "value", Pulsar2.iFindRateRAStored);
    dx->setPropertyInt("spinBox_Find_Dec", "value", Pulsar2.iFindRateDecStored);
    dx->setPropertyInt("spinBox_Slew_RA", "value", Pulsar2.iSlewRateRAStored);
    dx->setPropertyInt("spinBox_Slew_Dec", "value", Pulsar2.iSlewRateDecStored);

    //   Synchronisation on Connect
    dx->setChecked("checkBox_syncTime", Pulsar2.bSyncTimeOnConnectStored);
    dx->setChecked("checkBox_syncLocation", Pulsar2.bSyncLocationOnConnectStored);
    //-----------------------------------------------------

    
    X2MutexLocker ml(GetMutex());
    
    //Display the user interface
    if((nErr = ui->exec(bPressedOK)))
        return nErr;
    
    //Retrieve values from the user interface
    if (bPressedOK)
    {
        if(dx->isChecked("radioButton_Stop"))
            Pulsar2.iMeridianBehaviourStored = 0;
        if(dx->isChecked("radioButton_Flip"))
            Pulsar2.iMeridianBehaviourStored = 1;
        if(dx->isChecked("radioButton_Continue_Stop"))
            Pulsar2.iMeridianBehaviourStored = 2;
        if(dx->isChecked("radioButton_Continue_Flip"))
            Pulsar2.iMeridianBehaviourStored = 3;

        dx->propertyDouble("doubleSpinBox_HoursEast", "value", Pulsar2.dHoursEastStored);
        dx->propertyDouble("doubleSpinBox_HoursWest", "value", Pulsar2.dHoursWestStored);
        dx->propertyDouble("doubleSpinBox_HourFlip", "value", Pulsar2.dFlipHourStored);
        
        dx->propertyDouble("doubleSpinBox_Guide_RA", "value", Pulsar2.dGuideRateRAStored);
        dx->propertyDouble("doubleSpinBox_Guide_Dec", "value", Pulsar2.dGuideRateDecStored);
        dx->propertyInt("spinBox_Centre_RA", "value", Pulsar2.iCentreRateRAStored);
        dx->propertyInt("spinBox_Centre_Dec", "value", Pulsar2.iCentreRateDecStored);
        dx->propertyInt("spinBox_Find_RA", "value", Pulsar2.iFindRateRAStored);
        dx->propertyInt("spinBox_Find_Dec", "value", Pulsar2.iFindRateDecStored);
        dx->propertyInt("spinBox_Slew_RA", "value", Pulsar2.iSlewRateRAStored);
        dx->propertyInt("spinBox_Slew_Dec", "value", Pulsar2.iSlewRateDecStored);

        Pulsar2.bSyncTimeOnConnectStored = dx->isChecked("checkBox_syncTime");
        Pulsar2.bSyncLocationOnConnectStored = dx->isChecked("checkBox_syncLocation");

        
        // Save configuration to ini file
        if (m_pIniUtil)
        {
            m_pIniUtil->writeDouble(PARENT_KEY_STRING, PULSAR2_HOURS_EAST,  Pulsar2.dHoursEastStored);
            m_pIniUtil->writeDouble(PARENT_KEY_STRING, PULSAR2_HOURS_WEST,  Pulsar2.dHoursWestStored);
            m_pIniUtil->writeDouble(PARENT_KEY_STRING, PULSAR2_FLIPHOUR,  Pulsar2.dFlipHourStored);
            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_MERIDIAN,  Pulsar2.iMeridianBehaviourStored);

            m_pIniUtil->writeDouble(PARENT_KEY_STRING, PULSAR2_GUIDE_RA, Pulsar2.dGuideRateRAStored);
            m_pIniUtil->writeDouble(PARENT_KEY_STRING, PULSAR2_GUIDE_DEC, Pulsar2.dGuideRateDecStored);
            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_CENTRE_RA, Pulsar2.iCentreRateRAStored);
            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_CENTRE_DEC, Pulsar2.iCentreRateDecStored);
            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_FIND_RA, Pulsar2.iFindRateRAStored);
            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_FIND_DEC, Pulsar2.iFindRateDecStored);
            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_SLEW_RA, Pulsar2.iSlewRateRAStored);
            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_SLEW_DEC, Pulsar2.iSlewRateDecStored);
            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_SYNC_TIME, Pulsar2.bSyncTimeOnConnectStored);
            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_SYNC_LOCATION, Pulsar2.bSyncLocationOnConnectStored);
           
        }
    }
    
    return nErr;
    
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - X2GUIEventInterface
////////////////////////////////////////////////////////////////////////////
void X2Mount::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;
    double dRa, dDec;
    int iRa, iDec;
    char szTmpBuf[SERIAL_BUFFER_SIZE];

    if (!strcmp(pszEvent, "on_pushButton_clicked")) { // Guide (0.1 - 0.9 x sidereal)
        uiex->propertyDouble("doubleSpinBox_GuideRA", "value", dRa);
        uiex->propertyDouble("doubleSpinBox_GuideDec", "value", dDec);
        iRa = (int)(dRa*10.0);
        iDec = (int)(dDec*10.0);
        nErr = Pulsar2.setGuideRates(iRa, iDec);
        if(nErr){
            snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "Error setting guide rates to %3.1f : %3.1f", dRa, dDec);
            uiex->messageBox("Error", szTmpBuf);
            return;
        }
        // store the values used
        Pulsar2.dGuideRateRAStored = dRa;
        Pulsar2.dGuideRateDecStored = dDec;
    }
    else if (!strcmp(pszEvent, "on_pushButton_2_clicked")) { // Centre (1 - 999 x sidereal)
        uiex->propertyInt("spinBox_CentreRA", "value", iRa);
        uiex->propertyInt("spinBox_CentreDec", "value", iDec);
        nErr = Pulsar2.setCentreRates(iRa, iDec);
        if(nErr){
            snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "Error setting centre rates to %d : %d", iRa, iDec);
            uiex->messageBox("Error", szTmpBuf);
            return;
        }
        // store the values used
        Pulsar2.iCentreRateRAStored = iRa;
        Pulsar2.iCentreRateDecStored = iDec;
   }
    else if (!strcmp(pszEvent, "on_pushButton_3_clicked")) { // Find (1 - 999 x sidereal)
        uiex->propertyInt("spinBox_FindRA", "value", iRa);
        uiex->propertyInt("spinBox_FindDec", "value", iDec);
        nErr = Pulsar2.setFindRates(iRa, iDec);
        if(nErr){
            snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "Error setting guide rate to %d : %d", iRa, iDec);
            uiex->messageBox("Error", szTmpBuf);
            return;
        }
        // store the values used
        Pulsar2.iFindRateRAStored = iRa;
        Pulsar2.iFindRateDecStored = iDec;
    }
    else if (!strcmp(pszEvent, "on_pushButton_4_clicked")) { // Slew (1 - 999 x sidereal)
        uiex->propertyInt("spinBox_SlewRA", "value", iRa);
        uiex->propertyInt("spinBox_SlewDec", "value", iDec);
        nErr = Pulsar2.setSlewRates(iRa, iDec);
        
        if(nErr){
            snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "Error setting slew rate to %d : %d", iRa, iDec);
            uiex->messageBox("Error", szTmpBuf);
            return;
        }
        // store the values used
        Pulsar2.iSlewRateRAStored = iRa;
        Pulsar2.iSlewRateDecStored = iDec;
    }
    else if (!strcmp(pszEvent, "on_pushButton_5_clicked")) { // Set Park position
        // get current alt/az
        nErr = Pulsar2.parkSetParkPosition();
        if(nErr){
            snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "Error setting Park Position %d", nErr);
            uiex->messageBox("Error", szTmpBuf);
        }
    }
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#pragma mark - SerialPortParams2Interface
//
void X2Mount::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);
    
    str = szPortName;

}

void X2Mount::setPortName(const char* pszPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY_STRING, PULSAR2_SERIAL_NAME, pszPort);

}


void X2Mount::portNameOnToCharPtr(char* pszPort, const unsigned int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize, DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY_STRING, PULSAR2_SERIAL_NAME, pszPort, pszPort, nMaxSize);

}




