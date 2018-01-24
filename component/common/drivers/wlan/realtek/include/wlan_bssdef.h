/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *                                        
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#ifndef __WLAN_BSSDEF_H__
#define __WLAN_BSSDEF_H__


#define MAX_IE_SZ	768 //384//

#if defined(PLATFORM_LINUX) || defined(PLATFORM_ECOS) || defined(PLATFORM_FREERTOS) || defined(PLATFORM_CMSIS_RTOS)
#define NDIS_802_11_LENGTH_SSID         32
#define NDIS_802_11_LENGTH_RATES        8
#define NDIS_802_11_LENGTH_RATES_EX     16

typedef unsigned char   NDIS_802_11_MAC_ADDRESS[6];
typedef long    		NDIS_802_11_RSSI;           // in dBm
typedef unsigned char   NDIS_802_11_RATES[NDIS_802_11_LENGTH_RATES];        // Set of 8 data rates
typedef unsigned char   NDIS_802_11_RATES_EX[NDIS_802_11_LENGTH_RATES_EX];  // Set of 16 data rates


typedef  uint32_t  NDIS_802_11_KEY_INDEX;
typedef unsigned long long NDIS_802_11_KEY_RSC;


typedef struct _NDIS_802_11_SSID
{
  uint32_t  SsidLength;
  uint8_t  Ssid[NDIS_802_11_LENGTH_SSID+4];
}
#ifdef __CC_ARM	
__attribute__((packed))
#endif
NDIS_802_11_SSID, *PNDIS_802_11_SSID;

typedef enum _NDIS_802_11_NETWORK_TYPE
{
    Ndis802_11FH,
    Ndis802_11DS,
    Ndis802_11OFDM5,
    Ndis802_11OFDM24,
    Ndis802_11NetworkTypeMax    // not a real type, defined as an upper bound
} NDIS_802_11_NETWORK_TYPE, *PNDIS_802_11_NETWORK_TYPE;

typedef struct _NDIS_802_11_CONFIGURATION_FH
{
    uint32_t           Length;             // Length of structure
    uint32_t           HopPattern;         // As defined by 802.11, MSB set
    uint32_t           HopSet;             // to one if non-802.11
    uint32_t           DwellTime;          // units are Kusec
}
#ifdef __CC_ARM	
__attribute__((packed))
#endif
NDIS_802_11_CONFIGURATION_FH, *PNDIS_802_11_CONFIGURATION_FH;
 

/*
	FW will only save the channel number in DSConfig.
	ODI Handler will convert the channel number to freq. number.	
*/
typedef struct _NDIS_802_11_CONFIGURATION
{
    uint32_t           Length;             // Length of structure
    uint32_t           BeaconPeriod;       // units are Kusec
    uint32_t           ATIMWindow;         // units are Kusec
    uint32_t           DSConfig;           // Frequency, units are kHz
    NDIS_802_11_CONFIGURATION_FH    FHConfig;
}
#ifdef __CC_ARM	
__attribute__((packed))
#endif
NDIS_802_11_CONFIGURATION, *PNDIS_802_11_CONFIGURATION;



typedef enum _NDIS_802_11_NETWORK_INFRASTRUCTURE
{
    Ndis802_11IBSS,
    Ndis802_11Infrastructure,
    Ndis802_11AutoUnknown,
    Ndis802_11InfrastructureMax,     // Not a real value, defined as upper bound
    Ndis802_11APMode
} NDIS_802_11_NETWORK_INFRASTRUCTURE, *PNDIS_802_11_NETWORK_INFRASTRUCTURE;





typedef struct _NDIS_802_11_FIXED_IEs
{
  uint8_t  Timestamp[8];
  uint16_t  BeaconInterval;
  uint16_t  Capabilities;
} NDIS_802_11_FIXED_IEs, *PNDIS_802_11_FIXED_IEs;



typedef struct _NDIS_802_11_VARIABLE_IEs
{
  uint8_t  ElementID;
  uint8_t  Length;
  uint8_t  data[1];
} NDIS_802_11_VARIABLE_IEs, *PNDIS_802_11_VARIABLE_IEs;



/*



Length is the 4 bytes multiples of the sume of
	sizeof (NDIS_802_11_MAC_ADDRESS) + 2 + sizeof (NDIS_802_11_SSID) + sizeof (uint32_t)
+   sizeof (NDIS_802_11_RSSI) + sizeof (NDIS_802_11_NETWORK_TYPE) + sizeof (NDIS_802_11_CONFIGURATION)
+   sizeof (NDIS_802_11_RATES_EX) + IELength

Except the IELength, all other fields are fixed length. Therefore, we can define a marco to present the
partial sum.

*/
#if 0
typedef struct _NDIS_WLAN_BSSID_EX
{
  uint32_t  Length;
  NDIS_802_11_MAC_ADDRESS  MacAddress;
  uint8_t  Reserved[2];//[0]: IS beacon frame, [1]:optimum_antenna=>For antenna diversity;
  NDIS_802_11_SSID  Ssid;
  uint32_t  Privacy;
  NDIS_802_11_RSSI  Rssi;
  NDIS_802_11_NETWORK_TYPE  NetworkTypeInUse;
  NDIS_802_11_CONFIGURATION  Configuration;
  NDIS_802_11_NETWORK_INFRASTRUCTURE  InfrastructureMode;
  NDIS_802_11_RATES_EX  SupportedRates;
  uint32_t  IELength;
  uint8_t  IEs[MAX_IE_SZ];	//(timestamp, beacon interval, and capability information)
} NDIS_WLAN_BSSID_EX, *PNDIS_WLAN_BSSID_EX;


typedef struct _NDIS_802_11_BSSID_LIST_EX
{
  uint32_t  NumberOfItems;
  NDIS_WLAN_BSSID_EX  Bssid[1];
} NDIS_802_11_BSSID_LIST_EX, *PNDIS_802_11_BSSID_LIST_EX;
#endif

typedef enum _NDIS_802_11_AUTHENTICATION_MODE
{
    Ndis802_11AuthModeOpen,
    Ndis802_11AuthModeShared,
    Ndis802_11AuthModeAutoSwitch,
    Ndis802_11AuthModeWPA,
    Ndis802_11AuthModeWPAPSK,
    Ndis802_11AuthModeWPANone,
    Ndis802_11AuthModeWAPI,
    Ndis802_11AuthModeMax               // Not a real mode, defined as upper bound
} NDIS_802_11_AUTHENTICATION_MODE, *PNDIS_802_11_AUTHENTICATION_MODE;

typedef enum _NDIS_802_11_WEP_STATUS
{
    Ndis802_11WEPEnabled,
    Ndis802_11Encryption1Enabled = Ndis802_11WEPEnabled,
    Ndis802_11WEPDisabled,
    Ndis802_11EncryptionDisabled = Ndis802_11WEPDisabled,
    Ndis802_11WEPKeyAbsent,
    Ndis802_11Encryption1KeyAbsent = Ndis802_11WEPKeyAbsent,
    Ndis802_11WEPNotSupported,
    Ndis802_11EncryptionNotSupported = Ndis802_11WEPNotSupported,
    Ndis802_11Encryption2Enabled,
    Ndis802_11Encryption2KeyAbsent,
    Ndis802_11Encryption3Enabled,
    Ndis802_11Encryption3KeyAbsent,
    Ndis802_11_EncrypteionWAPI
} NDIS_802_11_WEP_STATUS, *PNDIS_802_11_WEP_STATUS,
  NDIS_802_11_ENCRYPTION_STATUS, *PNDIS_802_11_ENCRYPTION_STATUS;


#define NDIS_802_11_AI_REQFI_CAPABILITIES      1
#define NDIS_802_11_AI_REQFI_LISTENINTERVAL    2
#define NDIS_802_11_AI_REQFI_CURRENTAPADDRESS  4

#define NDIS_802_11_AI_RESFI_CAPABILITIES      1
#define NDIS_802_11_AI_RESFI_STATUSCODE        2
#define NDIS_802_11_AI_RESFI_ASSOCIATIONID     4

typedef struct _NDIS_802_11_AI_REQFI
{
    uint16_t Capabilities;
    uint16_t ListenInterval;
    NDIS_802_11_MAC_ADDRESS  CurrentAPAddress;
} NDIS_802_11_AI_REQFI, *PNDIS_802_11_AI_REQFI;

typedef struct _NDIS_802_11_AI_RESFI
{
    uint16_t Capabilities;
    uint16_t StatusCode;
    uint16_t AssociationId;
} NDIS_802_11_AI_RESFI, *PNDIS_802_11_AI_RESFI;

typedef struct _NDIS_802_11_ASSOCIATION_INFORMATION
{
    uint32_t                   Length;
    uint16_t                  AvailableRequestFixedIEs;
    NDIS_802_11_AI_REQFI    RequestFixedIEs;
    uint32_t                   RequestIELength;
    uint32_t                   OffsetRequestIEs;
    uint16_t                  AvailableResponseFixedIEs;
    NDIS_802_11_AI_RESFI    ResponseFixedIEs;
    uint32_t                   ResponseIELength;
    uint32_t                   OffsetResponseIEs;
} NDIS_802_11_ASSOCIATION_INFORMATION, *PNDIS_802_11_ASSOCIATION_INFORMATION;

typedef enum _NDIS_802_11_RELOAD_DEFAULTS
{
   Ndis802_11ReloadWEPKeys
} NDIS_802_11_RELOAD_DEFAULTS, *PNDIS_802_11_RELOAD_DEFAULTS;


// Key mapping keys require a BSSID
typedef struct _NDIS_802_11_KEY
{
    uint32_t           Length;             // Length of this structure
    uint32_t           KeyIndex;           
    uint32_t           KeyLength;          // length of key in bytes
    NDIS_802_11_MAC_ADDRESS BSSID;
    NDIS_802_11_KEY_RSC KeyRSC;
    uint8_t           KeyMaterial[32];     // variable length depending on above field
} NDIS_802_11_KEY, *PNDIS_802_11_KEY;

typedef struct _NDIS_802_11_REMOVE_KEY
{
    uint32_t                   Length;        // Length of this structure
    uint32_t                   KeyIndex;           
    NDIS_802_11_MAC_ADDRESS BSSID;      
} NDIS_802_11_REMOVE_KEY, *PNDIS_802_11_REMOVE_KEY;

typedef struct _NDIS_802_11_WEP
{
    uint32_t     Length;        // Length of this structure
    uint32_t     KeyIndex;      // 0 is the per-client key, 1-N are the global keys
    uint32_t     KeyLength;     // length of key in bytes
    uint8_t     KeyMaterial[16];// variable length depending on above field
} NDIS_802_11_WEP, *PNDIS_802_11_WEP;

typedef struct _NDIS_802_11_AUTHENTICATION_REQUEST
{
    uint32_t Length;            // Length of structure
    NDIS_802_11_MAC_ADDRESS Bssid;
    uint32_t Flags;
} NDIS_802_11_AUTHENTICATION_REQUEST, *PNDIS_802_11_AUTHENTICATION_REQUEST;

typedef enum _NDIS_802_11_STATUS_TYPE
{
	Ndis802_11StatusType_Authentication,
	Ndis802_11StatusType_MediaStreamMode,
	Ndis802_11StatusType_PMKID_CandidateList,		
	Ndis802_11StatusTypeMax    // not a real type, defined as an upper bound
} NDIS_802_11_STATUS_TYPE, *PNDIS_802_11_STATUS_TYPE;

typedef struct _NDIS_802_11_STATUS_INDICATION
{
    NDIS_802_11_STATUS_TYPE StatusType;
} NDIS_802_11_STATUS_INDICATION, *PNDIS_802_11_STATUS_INDICATION;

// mask for authentication/integrity fields
#define NDIS_802_11_AUTH_REQUEST_AUTH_FIELDS        0x0f
#define NDIS_802_11_AUTH_REQUEST_REAUTH			0x01
#define NDIS_802_11_AUTH_REQUEST_KEYUPDATE		0x02
#define NDIS_802_11_AUTH_REQUEST_PAIRWISE_ERROR		0x06
#define NDIS_802_11_AUTH_REQUEST_GROUP_ERROR		0x0E

// MIC check time, 60 seconds.
#define MIC_CHECK_TIME	60000000

typedef struct _NDIS_802_11_AUTHENTICATION_EVENT
{
    NDIS_802_11_STATUS_INDICATION       Status;
    NDIS_802_11_AUTHENTICATION_REQUEST  Request[1];
} NDIS_802_11_AUTHENTICATION_EVENT, *PNDIS_802_11_AUTHENTICATION_EVENT;
        
typedef struct _NDIS_802_11_TEST
{
    uint32_t Length;
    uint32_t Type;
    union
    {
        NDIS_802_11_AUTHENTICATION_EVENT AuthenticationEvent;
        NDIS_802_11_RSSI RssiTrigger;
    }tt;
} NDIS_802_11_TEST, *PNDIS_802_11_TEST;


#endif //end of #ifdef PLATFORM_LINUX

#ifdef PLATFORM_FREEBSD

#define NDIS_802_11_LENGTH_SSID         32
#define NDIS_802_11_LENGTH_RATES        8
#define NDIS_802_11_LENGTH_RATES_EX     16

typedef unsigned char   NDIS_802_11_MAC_ADDRESS[6];
typedef long    		NDIS_802_11_RSSI;           // in dBm
typedef unsigned char   NDIS_802_11_RATES[NDIS_802_11_LENGTH_RATES];        // Set of 8 data rates
typedef unsigned char   NDIS_802_11_RATES_EX[NDIS_802_11_LENGTH_RATES_EX];  // Set of 16 data rates


typedef  uint32_t  NDIS_802_11_KEY_INDEX;
typedef unsigned long long NDIS_802_11_KEY_RSC;


typedef struct _NDIS_802_11_SSID
{
  uint32_t  SsidLength;
  uint8_t  Ssid[32];
} NDIS_802_11_SSID, *PNDIS_802_11_SSID;

typedef enum _NDIS_802_11_NETWORK_TYPE
{
    Ndis802_11FH,
    Ndis802_11DS,
    Ndis802_11OFDM5,
    Ndis802_11OFDM24,
    Ndis802_11NetworkTypeMax    // not a real type, defined as an upper bound
} NDIS_802_11_NETWORK_TYPE, *PNDIS_802_11_NETWORK_TYPE;

typedef struct _NDIS_802_11_CONFIGURATION_FH
{
    uint32_t           Length;             // Length of structure
    uint32_t           HopPattern;         // As defined by 802.11, MSB set
    uint32_t           HopSet;             // to one if non-802.11
    uint32_t           DwellTime;          // units are Kusec
} NDIS_802_11_CONFIGURATION_FH, *PNDIS_802_11_CONFIGURATION_FH;
 

/*
	FW will only save the channel number in DSConfig.
	ODI Handler will convert the channel number to freq. number.	
*/
typedef struct _NDIS_802_11_CONFIGURATION
{
    uint32_t           Length;             // Length of structure
    uint32_t           BeaconPeriod;       // units are Kusec
    uint32_t           ATIMWindow;         // units are Kusec
    uint32_t           DSConfig;           // Frequency, units are kHz
    NDIS_802_11_CONFIGURATION_FH    FHConfig;
} NDIS_802_11_CONFIGURATION, *PNDIS_802_11_CONFIGURATION;



typedef enum _NDIS_802_11_NETWORK_INFRASTRUCTURE
{
    Ndis802_11IBSS,
    Ndis802_11Infrastructure,
    Ndis802_11AutoUnknown,
    Ndis802_11InfrastructureMax,     // Not a real value, defined as upper bound
    Ndis802_11APMode
} NDIS_802_11_NETWORK_INFRASTRUCTURE, *PNDIS_802_11_NETWORK_INFRASTRUCTURE;





typedef struct _NDIS_802_11_FIXED_IEs
{
  uint8_t  Timestamp[8];
  uint16_t  BeaconInterval;
  uint16_t  Capabilities;
} NDIS_802_11_FIXED_IEs, *PNDIS_802_11_FIXED_IEs;



typedef struct _NDIS_802_11_VARIABLE_IEs
{
  uint8_t  ElementID;
  uint8_t  Length;
  uint8_t  data[1];
} NDIS_802_11_VARIABLE_IEs, *PNDIS_802_11_VARIABLE_IEs;



/*



Length is the 4 bytes multiples of the sume of
	sizeof (NDIS_802_11_MAC_ADDRESS) + 2 + sizeof (NDIS_802_11_SSID) + sizeof (uint32_t)
+   sizeof (NDIS_802_11_RSSI) + sizeof (NDIS_802_11_NETWORK_TYPE) + sizeof (NDIS_802_11_CONFIGURATION)
+   sizeof (NDIS_802_11_RATES_EX) + IELength

Except the IELength, all other fields are fixed length. Therefore, we can define a marco to present the
partial sum.

*/
#if 0
typedef struct _NDIS_WLAN_BSSID_EX
{
  uint32_t  Length;
  NDIS_802_11_MAC_ADDRESS  MacAddress;
  uint8_t  Reserved[2];//[0]: IS beacon frame, [1]:optimum_antenna=>For antenna diversity;
  NDIS_802_11_SSID  Ssid;
  uint32_t  Privacy;
  NDIS_802_11_RSSI  Rssi;
  NDIS_802_11_NETWORK_TYPE  NetworkTypeInUse;
  NDIS_802_11_CONFIGURATION  Configuration;
  NDIS_802_11_NETWORK_INFRASTRUCTURE  InfrastructureMode;
  NDIS_802_11_RATES_EX  SupportedRates;
  uint32_t  IELength;
  uint8_t  IEs[MAX_IE_SZ];	//(timestamp, beacon interval, and capability information)
} NDIS_WLAN_BSSID_EX, *PNDIS_WLAN_BSSID_EX;


typedef struct _NDIS_802_11_BSSID_LIST_EX
{
  uint32_t  NumberOfItems;
  NDIS_WLAN_BSSID_EX  Bssid[1];
} NDIS_802_11_BSSID_LIST_EX, *PNDIS_802_11_BSSID_LIST_EX;
#endif

typedef enum _NDIS_802_11_AUTHENTICATION_MODE
{
    Ndis802_11AuthModeOpen,
    Ndis802_11AuthModeShared,
    Ndis802_11AuthModeAutoSwitch,
    Ndis802_11AuthModeWPA,
    Ndis802_11AuthModeWPAPSK,
    Ndis802_11AuthModeWPANone,
    Ndis802_11AuthModeMax               // Not a real mode, defined as upper bound
} NDIS_802_11_AUTHENTICATION_MODE, *PNDIS_802_11_AUTHENTICATION_MODE;

typedef enum _NDIS_802_11_WEP_STATUS
{
    Ndis802_11WEPEnabled,
    Ndis802_11Encryption1Enabled = Ndis802_11WEPEnabled,
    Ndis802_11WEPDisabled,
    Ndis802_11EncryptionDisabled = Ndis802_11WEPDisabled,
    Ndis802_11WEPKeyAbsent,
    Ndis802_11Encryption1KeyAbsent = Ndis802_11WEPKeyAbsent,
    Ndis802_11WEPNotSupported,
    Ndis802_11EncryptionNotSupported = Ndis802_11WEPNotSupported,
    Ndis802_11Encryption2Enabled,
    Ndis802_11Encryption2KeyAbsent,
    Ndis802_11Encryption3Enabled,
    Ndis802_11Encryption3KeyAbsent
} NDIS_802_11_WEP_STATUS, *PNDIS_802_11_WEP_STATUS,
  NDIS_802_11_ENCRYPTION_STATUS, *PNDIS_802_11_ENCRYPTION_STATUS;


#define NDIS_802_11_AI_REQFI_CAPABILITIES      1
#define NDIS_802_11_AI_REQFI_LISTENINTERVAL    2
#define NDIS_802_11_AI_REQFI_CURRENTAPADDRESS  4

#define NDIS_802_11_AI_RESFI_CAPABILITIES      1
#define NDIS_802_11_AI_RESFI_STATUSCODE        2
#define NDIS_802_11_AI_RESFI_ASSOCIATIONID     4

typedef struct _NDIS_802_11_AI_REQFI
{
    uint16_t Capabilities;
    uint16_t ListenInterval;
    NDIS_802_11_MAC_ADDRESS  CurrentAPAddress;
} NDIS_802_11_AI_REQFI, *PNDIS_802_11_AI_REQFI;

typedef struct _NDIS_802_11_AI_RESFI
{
    uint16_t Capabilities;
    uint16_t StatusCode;
    uint16_t AssociationId;
} NDIS_802_11_AI_RESFI, *PNDIS_802_11_AI_RESFI;

typedef struct _NDIS_802_11_ASSOCIATION_INFORMATION
{
    uint32_t                   Length;
    uint16_t                  AvailableRequestFixedIEs;
    NDIS_802_11_AI_REQFI    RequestFixedIEs;
    uint32_t                   RequestIELength;
    uint32_t                   OffsetRequestIEs;
    uint16_t                  AvailableResponseFixedIEs;
    NDIS_802_11_AI_RESFI    ResponseFixedIEs;
    uint32_t                   ResponseIELength;
    uint32_t                   OffsetResponseIEs;
} NDIS_802_11_ASSOCIATION_INFORMATION, *PNDIS_802_11_ASSOCIATION_INFORMATION;

typedef enum _NDIS_802_11_RELOAD_DEFAULTS
{
   Ndis802_11ReloadWEPKeys
} NDIS_802_11_RELOAD_DEFAULTS, *PNDIS_802_11_RELOAD_DEFAULTS;


// Key mapping keys require a BSSID
typedef struct _NDIS_802_11_KEY
{
    uint32_t           Length;             // Length of this structure
    uint32_t           KeyIndex;           
    uint32_t           KeyLength;          // length of key in bytes
    NDIS_802_11_MAC_ADDRESS BSSID;
    NDIS_802_11_KEY_RSC KeyRSC;
    uint8_t           KeyMaterial[32];     // variable length depending on above field
} NDIS_802_11_KEY, *PNDIS_802_11_KEY;

typedef struct _NDIS_802_11_REMOVE_KEY
{
    uint32_t                   Length;        // Length of this structure
    uint32_t                   KeyIndex;           
    NDIS_802_11_MAC_ADDRESS BSSID;      
} NDIS_802_11_REMOVE_KEY, *PNDIS_802_11_REMOVE_KEY;

typedef struct _NDIS_802_11_WEP
{
    uint32_t     Length;        // Length of this structure
    uint32_t     KeyIndex;      // 0 is the per-client key, 1-N are the global keys
    uint32_t     KeyLength;     // length of key in bytes
    uint8_t     KeyMaterial[16];// variable length depending on above field
} NDIS_802_11_WEP, *PNDIS_802_11_WEP;

typedef struct _NDIS_802_11_AUTHENTICATION_REQUEST
{
    uint32_t Length;            // Length of structure
    NDIS_802_11_MAC_ADDRESS Bssid;
    uint32_t Flags;
} NDIS_802_11_AUTHENTICATION_REQUEST, *PNDIS_802_11_AUTHENTICATION_REQUEST;

typedef enum _NDIS_802_11_STATUS_TYPE
{
	Ndis802_11StatusType_Authentication,
	Ndis802_11StatusType_MediaStreamMode,
	Ndis802_11StatusType_PMKID_CandidateList,		
	Ndis802_11StatusTypeMax    // not a real type, defined as an upper bound
} NDIS_802_11_STATUS_TYPE, *PNDIS_802_11_STATUS_TYPE;

typedef struct _NDIS_802_11_STATUS_INDICATION
{
    NDIS_802_11_STATUS_TYPE StatusType;
} NDIS_802_11_STATUS_INDICATION, *PNDIS_802_11_STATUS_INDICATION;

// mask for authentication/integrity fields
#define NDIS_802_11_AUTH_REQUEST_AUTH_FIELDS        0x0f
#define NDIS_802_11_AUTH_REQUEST_REAUTH			0x01
#define NDIS_802_11_AUTH_REQUEST_KEYUPDATE		0x02
#define NDIS_802_11_AUTH_REQUEST_PAIRWISE_ERROR		0x06
#define NDIS_802_11_AUTH_REQUEST_GROUP_ERROR		0x0E

// MIC check time, 60 seconds.
#define MIC_CHECK_TIME	60000000

typedef struct _NDIS_802_11_AUTHENTICATION_EVENT
{
    NDIS_802_11_STATUS_INDICATION       Status;
    NDIS_802_11_AUTHENTICATION_REQUEST  Request[1];
} NDIS_802_11_AUTHENTICATION_EVENT, *PNDIS_802_11_AUTHENTICATION_EVENT;
        
typedef struct _NDIS_802_11_TEST
{
    uint32_t Length;
    uint32_t Type;
    union
    {
        NDIS_802_11_AUTHENTICATION_EVENT AuthenticationEvent;
        NDIS_802_11_RSSI RssiTrigger;
    }tt;
} NDIS_802_11_TEST, *PNDIS_802_11_TEST;


#endif //PLATFORM_FREEBSD

typedef struct _WLAN_PHY_INFO
{
	uint8_t	SignalStrength;		//(in percentage)
  	uint8_t	SignalQuality;		//(in percentage)
  	uint8_t	Optimum_antenna;  	//for Antenna diversity
  	uint8_t  	Reserved_0;
}
#ifdef __CC_ARM	
__attribute__((packed))
#endif
WLAN_PHY_INFO,*PWLAN_PHY_INFO;

typedef struct _WLAN_BCN_INFO
{
	/* these infor get from rtw_get_encrypt_info when
	 * 	 * translate scan to UI */
	uint8_t	encryp_protocol;	//ENCRYP_PROTOCOL_E: OPEN/WEP/WPA/WPA2/WAPI
	int	group_cipher;		//WPA/WPA2 group cipher
	int	pairwise_cipher;	//WPA/WPA2/WEP pairwise cipher
	int	is_8021x;

	/* bwmode 20/40 and ch_offset UP/LOW */
	unsigned short 	ht_cap_info;
	unsigned char	ht_info_infos_0;
} WLAN_BCN_INFO,*PWLAN_BCN_INFO;

/* temporally add #pragma pack for structure alignment issue of
*   WLAN_BSSID_EX and get_WLAN_BSSID_EX_sz()
*/
#ifdef RTW_PACK_STRUCT_USE_INCLUDES
#  include "pack_begin.h"
#endif
RTW_PACK_STRUCT_BEGIN
typedef struct _WLAN_BSSID_EX
{
  uint32_t					Length;
  NDIS_802_11_MAC_ADDRESS		MacAddress;
 #ifdef CONFIG_P2P_NEW
  uint8_t					Reserved[1];		//[0]: IS beacon frame
  uint8_t					bP2pNetwork;
 #else
  uint8_t					Reserved[2];		//[0]: IS beacon frame (padapter+163)
 #endif
  NDIS_802_11_SSID			Ssid;
  uint32_t					Privacy;
  NDIS_802_11_RSSI			Rssi;			//(in dBM,raw data ,get from PHY)
  NDIS_802_11_NETWORK_TYPE		NetworkTypeInUse;
  NDIS_802_11_CONFIGURATION		Configuration;
  NDIS_802_11_NETWORK_INFRASTRUCTURE	InfrastructureMode;
  NDIS_802_11_RATES_EX  		SupportedRates;
  WLAN_PHY_INFO				PhyInfo;
  uint32_t  				IELength;
  uint8_t  				IEs[MAX_IE_SZ];		//(timestamp, beacon interval, and capability information)
}
RTW_PACK_STRUCT_STRUCT
WLAN_BSSID_EX, *PWLAN_BSSID_EX;
RTW_PACK_STRUCT_END
#ifdef RTW_PACK_STRUCT_USE_INCLUDES
#  include "pack_end.h"
#endif


__inline  static uint get_WLAN_BSSID_EX_sz(WLAN_BSSID_EX *bss)
{
#if 0
	uint t_len;
	
	t_len = sizeof (uint32_t) 
		+ sizeof (NDIS_802_11_MAC_ADDRESS) 
		+ 2 
		+ sizeof (NDIS_802_11_SSID) 
		+ sizeof (uint32_t) 
		+ sizeof (NDIS_802_11_RSSI) 
		+ sizeof (NDIS_802_11_NETWORK_TYPE)
		+ sizeof (NDIS_802_11_CONFIGURATION)
		+ sizeof (NDIS_802_11_NETWORK_INFRASTRUCTURE)
		+ sizeof (NDIS_802_11_RATES_EX)
		//all new member add here
		+ sizeof(WLAN_PHY_INFO)
		//all new member add here
		+ sizeof (uint32_t)
		+ bss->IELength;	
	return t_len;
#else
	return (sizeof(WLAN_BSSID_EX) -MAX_IE_SZ + bss->IELength);
#endif
}

struct	wlan_network {
	struct list_head list;
	int	network_type;	//refer to ieee80211.h for WIRELESS_11A/B/G
	int	fixed;			// set to fixed when not to be removed as site-surveying
	unsigned long	last_scanned; //timestamp for the network
	int	aid;			//will only be valid when a BSS is joinned.
	int	join_res;
	WLAN_BSSID_EX	network; //must be the last item
	WLAN_BCN_INFO	BcnInfo;
#ifdef PLATFORM_WINDOWS	
	unsigned char  iebuf[MAX_IE_SZ];
#endif

};

enum VRTL_CARRIER_SENSE
{
    DISABLE_VCS,	
    ENABLE_VCS,	
    AUTO_VCS
};

enum VCS_TYPE
{
    NONE_VCS,	
    RTS_CTS,
    CTS_TO_SELF 
};




#define PWR_CAM 0
#define PWR_MINPS 1
#define PWR_MAXPS 2
#define PWR_UAPSD 3
#define PWR_VOIP 4


enum UAPSD_MAX_SP
{
	NO_LIMIT,
       TWO_MSDU,
       FOUR_MSDU,
       SIX_MSDU
};


#define NUM_PRE_AUTH_KEY 16
#define NUM_PMKID_CACHE NUM_PRE_AUTH_KEY

/*
* 	WPA2
*/

#ifndef PLATFORM_OS_CE
typedef struct _PMKID_CANDIDATE {
    NDIS_802_11_MAC_ADDRESS BSSID;
    uint32_t Flags;
} PMKID_CANDIDATE, *PPMKID_CANDIDATE;

typedef struct _NDIS_802_11_PMKID_CANDIDATE_LIST
{
    uint32_t Version;       // Version of the structure
    uint32_t NumCandidates; // No. of pmkid candidates
    PMKID_CANDIDATE CandidateList[1];
} NDIS_802_11_PMKID_CANDIDATE_LIST, *PNDIS_802_11_PMKID_CANDIDATE_LIST;


typedef struct _NDIS_802_11_AUTHENTICATION_ENCRYPTION
{
	NDIS_802_11_AUTHENTICATION_MODE AuthModeSupported;
	NDIS_802_11_ENCRYPTION_STATUS EncryptStatusSupported;
	
} NDIS_802_11_AUTHENTICATION_ENCRYPTION, *PNDIS_802_11_AUTHENTICATION_ENCRYPTION;

typedef struct _NDIS_802_11_CAPABILITY 
{
	uint32_t  Length;
	uint32_t  Version;
	uint32_t  NoOfPMKIDs;
	uint32_t  NoOfAuthEncryptPairsSupported;
	NDIS_802_11_AUTHENTICATION_ENCRYPTION AuthenticationEncryptionSupported[1];
	
} NDIS_802_11_CAPABILITY, *PNDIS_802_11_CAPABILITY;
#endif


#endif //#ifndef WLAN_BSSDEF_H_

