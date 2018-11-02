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

    // Read in settings
    if (m_pIniUtil)
    {
        dHoursEastStored = m_pIniUtil->readDouble(PARENT_KEY_STRING, PULSAR2_HOURS_EAST, 0.0);
        dHoursWestStored = m_pIniUtil->readDouble(PARENT_KEY_STRING, PULSAR2_HOURS_WEST, 0.0);
        dFlipHourStored = m_pIniUtil->readDouble(PARENT_KEY_STRING, PULSAR2_FLIPHOUR, 0.0);
        iMeridianBehaviourStored = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_MERIDIAN, 1);
        iProvidesRefraction = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_REFRACTION, 0);
//        iLoggingVerbosity = m_pIniUtil->readInt(PARENT_KEY_STRING, PULSAR2_VERBOSITY, 0);
    }
    
// set pointers to some of the interfaces inside the Pulsar2 class
    Pulsar2.setSleeper(pSleeper);  // added by Rodolphe in iEQ30
    Pulsar2.SetSerxPointer(pSerX);
    Pulsar2.SetLoggerPointer(pLogger);

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
//LinkInterface
int	X2Mount::establishLink(void)					
{
    
	X2MutexLocker ml(GetMutex());

    if (iLoggingVerbosity >= VERBOSE_ESSENTIAL)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::establishLink");

    
    if(Pulsar2.Connect(serialName))
        {
        m_bLinked = true;
            if (iLoggingVerbosity >= VERBOSE_ESSENTIAL)
                if (GetLogger())
                    GetLogger()->out((char *) "X2Mount::establishLink - connected OK");
        return SB_OK;
        }
    else
        {
        m_bLinked = false;
            if (iLoggingVerbosity >= VERBOSE_FAILURES)
                if (GetLogger())
                    GetLogger()->out((char *) "X2Mount::establishLink - failed to connect");
        return ERR_NOLINK;
        }
}


int	X2Mount::terminateLink(void)						
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    Pulsar2.Disconnect();
	m_bLinked = false;
    if (iLoggingVerbosity >= VERBOSE_ESSENTIAL)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::terminateLink - Link terminated");
	return SB_OK;
}


bool X2Mount::isLinked(void) const					
{
	return m_bLinked;
}


bool X2Mount::isEstablishLinkAbortable(void) const
{
    return false;
}




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

void X2Mount::deviceInfoNameLong(BasicStringInterface& str) const
{
	str = "Pulsar2 Mount drive";

}

void X2Mount::deviceInfoDetailedDescription(BasicStringInterface& str) const
{
	str = DISPLAY_NAME;
}

void X2Mount::deviceInfoFirmwareVersion(BasicStringInterface& str)
{
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::deviceInfoFirmwareVersion");

    if (m_bLinked) {
        X2MutexLocker ml(GetMutex());
        char szFirmware[SERIAL_BUFFER_SIZE];
        if(Pulsar2.GetFirmware(szFirmware, SERIAL_BUFFER_SIZE))
            str = szFirmware;
        else
            str = "Device Not Connected.";
    }
    else
        str = "Device Not Connected.";
}


void X2Mount::deviceInfoModel(BasicStringInterface& str)
{
	str = "Pulsar2";
}


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

    nErr = Pulsar2.GetRADec(ra, dec);
    
    if (iLoggingVerbosity >= VERBOSE_RESULTS)
        if (GetLogger())
        {
            snprintf(szLogMessage, LOG_BUFFER_SIZE, "X2Mount::raDec: RA = %10f; Dec = %10f", ra, dec);
            GetLogger()->out(szLogMessage);
        }
 
	return nErr;
}

////////////////////////////////////////////////////////////////////////////
// Abort
int	X2Mount::abort(void)
{
    
    if (iLoggingVerbosity >= VERBOSE_ESSENTIAL)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::Abort");

    if(!m_bLinked)
        return ERR_NOLINK;
    X2MutexLocker ml(GetMutex());

    Pulsar2.Abort();
    
    return SB_OK;

}



int X2Mount::syncMount(const double& ra, const double& dec)
{

    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::syncMount");
    
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    if (!Pulsar2.SyncRADec(ra, dec))
        return ERR_CMDFAILED;

 return SB_OK;
}

////////////////////////////////////////////////////////////////////////////
// Always return true.
// If possible, return false when appropriate, if and only if the mount hardware
// has the ability to know if it has been synced or not.

bool X2Mount::isSynced()
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
// AsymmetricalEquatorialInterface
////////////////////////////////////////////////////////////////////////////


bool X2Mount::knowsBeyondThePole()
{
    
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::knowsBeyondThePole");
   
    return true;
}


////////////////////////////////////////////////////////////////////////////
/*!
 If knowsBeyondThePole() returns true,
 then beyondThePole() tells TheSkyX unambiguously
 if the OTA end of the declination axis
 is either east (0) or west of the pier (1).
 Note, the return value must be correct even
 for cases where the OTA end of the Dec axis
 is lower than the counterweights.
 
 */

int X2Mount::beyondThePole(bool& bYes)
{
    int nErr = SB_OK;
    int poleResult;
    
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::beyondThePole");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = Pulsar2.GetSideOfPier(poleResult);
    if(nErr)
        return nErr;

    // we will define west (1) as normal, and
    // east (0) as beyond the pole
    bYes = poleResult == 1?false:true;

    return nErr;
 }


////////////////////////////////////////////////////////////////////////////
double X2Mount::flipHourAngle()
{
    
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::flipHourAngle");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    // probably need to get this from the mount
    return dFlipHourStored;

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

    dHoursEast = dHoursEastStored;
    dHoursWest = dHoursWestStored;
    
    return SB_OK;
}



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// NeedsRefractionInterface
////////////////////////////////////////////////////////////////////////////
//
// only one function

bool X2Mount::needsRefactionAdjustments(void)
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::needsRefractionAdjustments");
    
    if (iProvidesRefraction == 0)
        return false;
    else
        return true;

}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// SlewToInterfaceInterface
////////////////////////////////////////////////////////////////////////////
// NB -- seems to not be called -- Abort is called instead

int X2Mount::startSlewTo(const double& dRa, const double& dDec)
{
    int nErr = SB_OK;
    bool bIsSlewing;
    int nSlewResult;
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


    nErr = Pulsar2.slewStatus(bIsSlewing);
    if(nErr)
        return nErr;

    if (!bIsSlewing) {
        nErr = Pulsar2.startSlew(dRa, dDec, nSlewResult);
        if(nErr)
            return nErr;
    }
    else
        return ERR_COMMANDINPROGRESS;
    
/*
    if (nSlewResult == 1)
        nErr = SB_OK; // accepted
    else if (nSlewResult == 0)
        nErr = ERR_LX200DESTBELOWHORIZ;  // below horizon (but with LX200 message)
    else
        nErr = ERR_CMDFAILED;  // error
*/

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
// I don't know what to put in this function
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
//OpenLoopMoveInterface
////////////////////////////////////////////////////////////////////////////
//
int X2Mount::startOpenLoopMove(const MountDriverInterface::MoveDir& Dir, const int& nRateIndex)
{

    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::startOpenLoopMove");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    iRateIndex = nRateIndex;

    if (Pulsar2.startMove(int(Dir), nRateIndex))
        return SB_OK;
    else
        return ERR_CMDFAILED;
    
}


////////////////////////////////////////////////////////////////////////////
//
int	X2Mount::endOpenLoopMove(void)
{
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::endOpenLoopMove");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    return Pulsar2.stopMoving();
}


////////////////////////////////////////////////////////////////////////////
//
bool X2Mount::allowDiagonalMoves(void)
{
    return false;
}


////////////////////////////////////////////////////////////////////////////
//
int X2Mount::rateCountOpenLoopMove(void) const
{
    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());
    return pMe->Pulsar2.getNbSlewRates();
}

int X2Mount::rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize)
{
    int nErr = SB_OK;
    nErr = Pulsar2.getRateName(nZeroBasedIndex, pszOut, nOutMaxSize);
    if(nErr) {
#ifdef ATCS_X2_DEBUG
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] rateNameFromIndexOpenLoopMove ERROR %d\n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        return ERR_CMDFAILED;
    }
    return nErr;
}


////////////////////////////////////////////////////////////////////////////
//
int	X2Mount::rateIndexOpenLoopMove(void)
{
    return iRateIndex;
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// TrackingRatesInterface
////////////////////////////////////////////////////////////////////////////
//
int X2Mount::setTrackingRates( const bool& bTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
/*
 - bTrackingOn
 true means turn sidereal tracking on regardless of other parameters
 false means turn it off
 
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
            // Lunar rate
            else if (0.51 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.58 && -0.25 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.25) {
                nErr = Pulsar2.setTrackingRate(LUNAR);
            }
            // Solar rate
            else if (0.041055 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.04108 && -0.034 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.034) {
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
int X2Mount::trackingRates( bool& bTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec)
{
/*
 - bTrackingOn
    true means tracking is at sidereal rate regardless of other parameters
    false means tracking rate is as given in the next 2 parameters. It does not mean that tracking is off.
 
 - dRaRateArcSecPerSec
 
    has no real effect: if bTrackingOn = true the rate is always reported to be sidereal;
    if bTrackingOn = false then generally the rate is reported as Custom, but note the special cases below.
 
- dDecRateArcSecPerSec
        
    has no effect, except that the hang reported below is avoided if this value is non-zero.
            
 
 Special cases:
            
    bTrackingOn = false; dRaRateArcSecPerSec = 15.041...; dDecRateArcSecPerSec = 0.0 ... means "Tracking Off".
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
            dRaRateArcSecPerSec = 15.0410681;
            dDecRateArcSecPerSec = 0.0;
            break;
            
        case SIDEREAL:
            bTrackingOn = true;
            dRaRateArcSecPerSec = 0.0;
            dDecRateArcSecPerSec = 0.0;
            break;

        case LUNAR:
            bTrackingOn = true;
            dRaRateArcSecPerSec = 0.5490149;
            dDecRateArcSecPerSec = 0.0;
            break;

        case SOLAR:
            bTrackingOn = true;
            dRaRateArcSecPerSec = 0.0410681;
            dDecRateArcSecPerSec = 0.0;
            break;
            
        default:
            bTrackingOn = false;
            dRaRateArcSecPerSec = 15.0410681;
            dDecRateArcSecPerSec = 0.0;
            break;
    }
    
    return nErr;
}



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// ParkInterface
////////////////////////////////////////////////////////////////////////////
//
bool X2Mount::isParked(void)
{
    bool bIsParked;
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::isParked");

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    Pulsar2.parkStatus(bIsParked);

    return bIsParked;
}


int	X2Mount::startPark(const double& dAz, const double& dAlt)
{

    return ERR_NOT_IMPL;
/*
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::startPark");

	X2MutexLocker ml(GetMutex());
    
    if (Pulsar2.Park(dAz, dAlt))
        return SB_OK;
    else
        return ERR_CMDFAILED;
*/
}


int	X2Mount::isCompletePark(bool& bComplete) const
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());
    nErr = pMe->Pulsar2.parkStatus(bComplete);

    return nErr;
}


int	X2Mount::endPark(void)
{
    return SB_OK;
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// UnparkInterface
////////////////////////////////////////////////////////////////////////////
//
int	X2Mount::startUnpark(void)
{
    int nErr = SB_OK;
    if (iLoggingVerbosity >= VERBOSE_FUNCTION_TRACKING)
        if (GetLogger())
            GetLogger()->out((char *) "X2Mount::startUnpark");

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
    
    // Just switch on sidereal tracking, like Pulsar2 does
    nErr = Pulsar2.unPark();

    return nErr;
    
}


int	X2Mount::isCompleteUnpark(bool& bComplete) const
{
    int nErr = SB_OK;
    bool bIsParked;

    bComplete = false;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());

    // need to check park status
    nErr = pMe->Pulsar2.parkStatus(bIsParked);
    if(nErr)
        return nErr;

    bComplete = bIsParked?false:true;
    return nErr;
}


int	X2Mount::endUnpark(void)
{
    return SB_OK;
}



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
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
        
//  Set the UI dialog in accordance with stored values
    switch (iMeridianBehaviourStored)
        {
        case 0:
            dx->setChecked("radioButton_Stop", true);
            break;
        case 1:
            dx->setChecked("radioButton_Flip", true);
            break;
        case 2:
            dx->setChecked("radioButton_Continue", true);
            break;
        }
        
    dx->setPropertyDouble("doubleSpinBox_HoursEast", "value", dHoursEastStored);
    dx->setPropertyDouble("doubleSpinBox_HoursWest", "value", dHoursWestStored);

    dx->setPropertyDouble("doubleSpinBox_HourFlip", "value", dFlipHourStored);
            
    if (iProvidesRefraction == 0)
        dx->setChecked("checkBox_Refraction", true);  // checked if mount provides refraction correction
    else
        dx->setChecked("checkBox_Refraction", false);  // checked if mount provides refraction correction
    
//    dx->setPropertyInt("horizontalSlider_Verbosity", "value", iLoggingVerbosity);
    
	//Display the user interface
	if((nErr = ui->exec(bPressedOK)))
		return nErr;
    
	//Retrieve values from the user interface
	if (bPressedOK)
    {
        if(dx->isChecked("radioButton_Stop"))
            iMeridianBehaviourStored = 0;
        if(dx->isChecked("radioButton_Flip"))
            iMeridianBehaviourStored = 1;
        if(dx->isChecked("radioButton_Continue"))
            iMeridianBehaviourStored = 2;

        dx->propertyDouble("doubleSpinBox_HoursEast", "value", dHoursEastStored);
        dx->propertyDouble("doubleSpinBox_HoursWest", "value", dHoursWestStored);

        dx->propertyDouble("doubleSpinBox_HourFlip", "value", dFlipHourStored);
        
        if (dx->isChecked("checkBox_Refraction"))
            iProvidesRefraction = 0;  // Mount does provide refraction correction
        else
            iProvidesRefraction = 1;  // Mount does NOT provide refraction correction

//        dx->propertyInt("horizontalSlider_Verbosity", "value", iLoggingVerbosity);
//        Pulsar2.iVerbosity = iLoggingVerbosity;
		
        // Save configuration to ini file
		if (m_pIniUtil)
        {
            m_pIniUtil->writeDouble(PARENT_KEY_STRING, PULSAR2_HOURS_EAST,  dHoursEastStored);
            m_pIniUtil->writeDouble(PARENT_KEY_STRING, PULSAR2_HOURS_WEST,  dHoursWestStored);
            m_pIniUtil->writeDouble(PARENT_KEY_STRING, PULSAR2_FLIPHOUR,  dFlipHourStored);
            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_MERIDIAN,  iMeridianBehaviourStored);
            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_REFRACTION, iProvidesRefraction);
//            m_pIniUtil->writeInt(PARENT_KEY_STRING, PULSAR2_VERBOSITY, iLoggingVerbosity);
        }
    }
    
	return nErr;
   
}

////////////////////////////////////////////////////////////////////////////
void X2Mount::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
}

#pragma mark - SerialPortParams2Interface

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




