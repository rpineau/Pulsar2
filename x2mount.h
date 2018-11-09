#ifndef __Pulsar2_H_
#define __Pulsar2_H_

#include <stdlib.h>
#include <string.h>

#pragma once
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/basicstringinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/basiciniutilinterface.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/mutexinterface.h"
#include "../../licensedinterfaces/tickcountinterface.h"
#include "../../licensedinterfaces/serialportparams2interface.h"
#include "../../licensedinterfaces/modalsettingsdialoginterface.h"
#include "../../licensedinterfaces/x2guiinterface.h"
#include "../../licensedinterfaces/mountdriverinterface.h"
#include "../../licensedinterfaces/mount/slewtointerface.h"
#include "../../licensedinterfaces/mount/syncmountinterface.h"
#include "../../licensedinterfaces/mount/asymmetricalequatorialinterface.h"
#include "../../licensedinterfaces/mount/openloopmoveinterface.h"
#include "../../licensedinterfaces/mount/needsrefractioninterface.h"
#include "../../licensedinterfaces/mount/trackingratesinterface.h"
#include "../../licensedinterfaces/parkinterface.h"
#include "../../licensedinterfaces/unparkinterface.h"
#include "../../licensedinterfaces/driverslewstoparkpositioninterface.h"


#include "StopWatch.h"
#include "Pulsar2.h"

//#define DRIVER_MAX_STRING   256

#define PARENT_KEY_STRING           "Pulsar2X2"
#define PULSAR2_SERIAL_NAME         "SERIALPORT"
#define PULSAR2_MERIDIAN            "MERIDIAN"
#define PULSAR2_FLIPHOUR            "FLIPHOUR"
#define PULSAR2_HOURS_EAST          "HOURSEAST"
#define PULSAR2_HOURS_WEST          "HOURSWEST"
#define PULSAR2_REFRACTION          "REFRACTION"
//#define PULSAR2_VERBOSITY         "VERBOSITY"

enum DEBUG_LEVEL {NONE = 0, METHOD, VERBOSE};

#define DEBUG VERBOSE

#if defined(SB_WIN_BUILD)
#define DEF_PORT_NAME                    "COM1"
#elif defined(SB_LINUX_BUILD)
#define DEF_PORT_NAME                    "/dev/mount"
#elif defined (SB_MAC_BUILD)
#define DEF_PORT_NAME                    "/dev/cu.KeySerial1"
#endif


#define X2_MOUNT_NAME       "X2 Pulsar2"
#define DRIVER_VERSION      1.0
#define DISPLAY_NAME        "Pulsar2 X2 Plug-In by Richard Francis"

class X2Mount : public MountDriverInterface
                ,public SyncMountInterface
                ,public SlewToInterface
                ,public AsymmetricalEquatorialInterface
                ,public OpenLoopMoveInterface
                ,public TrackingRatesInterface
                ,public ParkInterface
                ,public UnparkInterface
                ,public ModalSettingsDialogInterface
                ,public X2GUIEventInterface
                ,public SerialPortParams2Interface
                ,public DriverSlewsToParkPositionInterface
{
public:
	/*!Standard X2 constructor*/
	X2Mount(const char* pszDriverSelection,
				const int& nInstanceIndex,
				SerXInterface					* pSerX, 
				TheSkyXFacadeForDriversInterface	* pTheSkyX, 
				SleeperInterface					* pSleeper,
				BasicIniUtilInterface			* pIniUtil,
				LoggerInterface					* pLogger,
				MutexInterface					* pIOMutex,
				TickCountInterface				* pTickCount);

	~X2Mount();

// Operations
public:

	/*!\name DriverRootInterface Implementation
	See DriverRootInterface.*/
	//@{ 
	virtual DeviceType							deviceType(void)			{return DriverRootInterface::DT_MOUNT;}
	virtual int									queryAbstraction(const char* pszName, void** ppVal) ;
	//@} 

	/*!\name LinkInterface Implementation
	See LinkInterface.*/
	//@{ 
	virtual int									establishLink(void)						;
	virtual int									terminateLink(void)						;
	virtual bool								isLinked(void) const					;
	virtual bool								isEstablishLinkAbortable(void) const	;
	//@} 

	/*!\name DriverInfoInterface Implementation
	See DriverInfoInterface.*/
	//@{ 
	virtual void								driverInfoDetailedInfo(BasicStringInterface& str) const;
	virtual double								driverInfoVersion(void) const				;
	//@} 

	/*!\name HardwareInfoInterface Implementation
	See HardwareInfoInterface.*/
	//@{ 
	virtual void deviceInfoNameShort(BasicStringInterface& str) const				;
	virtual void deviceInfoNameLong(BasicStringInterface& str) const				;
	virtual void deviceInfoDetailedDescription(BasicStringInterface& str) const	;
	virtual void deviceInfoFirmwareVersion(BasicStringInterface& str)				;
	virtual void deviceInfoModel(BasicStringInterface& str)						;
	//@} 

	virtual int	raDec(double& ra, double& dec, const bool& bCached = false)					;
	virtual int	abort(void)																	;

	//Optional interfaces, uncomment and implement as required.

	//SyncMountInterface
	virtual int syncMount(const double& ra, const double& dec)									;
	virtual bool isSynced()																		;

	//SlewToInterface
	virtual int								startSlewTo(const double& dRa, const double& dDec)	;
	virtual int								isCompleteSlewTo(bool& bComplete) const				;
	virtual int								endSlewTo(void)										;
	
	//AsymmetricalEquatorialInterface
	virtual bool knowsBeyondThePole();
	virtual int beyondThePole(bool& bYes);
	virtual double flipHourAngle();
	virtual int gemLimits(double& dHoursEast, double& dHoursWest);

	//OpenLoopMoveInterface
	virtual int								startOpenLoopMove(const MountDriverInterface::MoveDir& Dir, const int& nRateIndex);
	virtual int								endOpenLoopMove(void)															;
	virtual bool							allowDiagonalMoves()															;
	virtual int								rateCountOpenLoopMove(void) const												;
	virtual int								rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize);
	virtual int								rateIndexOpenLoopMove(void);

	//NeedsRefractionInterface
	virtual bool							needsRefactionAdjustments(void);

	//LinkFromUIThreadInterface

	//TrackingRatesInterface 
	virtual int setTrackingRates( const bool& bTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec);
    virtual int siderealTrackingOn ();  // added by CRF 3 Nov 2018
    virtual int trackingOff();          // added by CRF 3 Nov 2018
    virtual int trackingRates( bool& bTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec);
    

    //ModalSettings etc.
    virtual int								initModalSettingsDialog(void){return 0;}
	virtual int								execModalSettingsDialog(void);
    
	virtual void                            uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent);

    //ParkInterface
	virtual bool							isParked(void);
	virtual int								startPark(const double& dAz, const double& dAlt);
	virtual int								isCompletePark(bool& bComplete) const;
	virtual int								endPark(void);

    
    //UnparkInterface
	virtual int								startUnpark(void);
	virtual int								isCompleteUnpark(bool& bComplete) const;
	virtual int								endUnpark(void);
    
    //SerialPortParams2Interface
    virtual void            portName(BasicStringInterface& str) const            ;
    virtual void            setPortName(const char* szPort)                        ;
    virtual unsigned int    baudRate() const            {return 19200;};
    virtual void            setBaudRate(unsigned int)    {};
    virtual bool            isBaudRateFixed() const        {return true;}

    virtual SerXInterface::Parity    parity() const {return m_nParity;}
    virtual void                    setParity(const SerXInterface::Parity& parity){m_nParity = parity;};
    virtual bool                    isParityFixed() const        {return true;}


// Implementation


private:

	SerXInterface 							*GetSerX() {return m_pSerX; }		
	TheSkyXFacadeForDriversInterface			*GetTheSkyXFacadeForMounts() {return m_pTheSkyXForMounts;}
	SleeperInterface						*GetSleeper() {return m_pSleeper; }
	BasicIniUtilInterface					*GetSimpleIniUtil() {return m_pIniUtil; }
	MutexInterface							*GetMutex()  {return m_pIOMutex;}
	TickCountInterface						*GetTickCountInterface() {return m_pTickCount;}

    LoggerInterface							*GetLogger() {return m_pLogger; }

	int                                     m_nPrivateMulitInstanceIndex;
	SerXInterface*							m_pSerX;		
	TheSkyXFacadeForDriversInterface* 		m_pTheSkyXForMounts;
	SleeperInterface*						m_pSleeper;
	BasicIniUtilInterface*					m_pIniUtil;
	LoggerInterface*						m_pLogger;
	MutexInterface*							m_pIOMutex;
	TickCountInterface*						m_pTickCount;

	void findDevices(X2GUIExchangeInterface *dx);
    
//    int                                     iLastPosition;
    char                                    serialName[256];
    double                                  dHoursEastStored;
    double                                  dHoursWestStored;
    double                                  dFlipHourStored;
    int                                     iMeridianBehaviourStored;
    int                                     iLoggingVerbosity;
    
    MountDriverInterface::MoveDir           currentMoveDir;

    CPulsar2Controller                      Pulsar2;

    void portNameOnToCharPtr(char* pszPort, const unsigned int& nMaxSize) const;
    SerXInterface::Parity m_nParity;

	bool m_bLinked;
    int iRateIndex;     // changed from line below by Rodolphe in iEQ30
//    int iRateIndex = 0;
//	int m_nPosition;


};



#endif //__Pulsar2_H_

