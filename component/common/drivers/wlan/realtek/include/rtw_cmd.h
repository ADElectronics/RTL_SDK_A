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
#ifndef __RTW_CMD_H_
#define __RTW_CMD_H_

#include "drv_types.h"
#include <rtw_rf.h>
#include <rtw_led.h>

#define C2H_MEM_SZ (16*1024)
//#define CMD_RSP_BUF	0
//#define CMD_DBG 0

#ifndef CONFIG_RTL8711FW

	#include <ieee80211.h> // <ieee80211/ieee80211.h>
	#include <rom_ieee80211.h> // <ieee80211/ieee80211.h>

	#define FREE_CMDOBJ_SZ	128
	
	#define MAX_CMDSZ	1024
	#define MAX_RSPSZ	512
	#define MAX_EVTSZ	1024

#if defined(PLATFORM_OS_CE) || defined(PLATFORM_ECOS) || defined(PLATFORM_FREERTOS) || defined (PLATFORM_CMSIS_RTOS)
	#define CMDBUFF_ALIGN_SZ 4
#else
	#define CMDBUFF_ALIGN_SZ 512
#endif

	struct cmd_obj {
		_adapter *padapter;
		uint16_t	cmdcode;
		uint8_t	res;
		uint8_t	*parmbuf;
		uint32_t	cmdsz;
		uint8_t	*rsp;
		uint32_t	rspsz;
		//_sema 	cmd_sem;
		_list	list;
	};

	struct cmd_priv {
		//_sema	cmd_done_sema;
		_queue	cmd_queue;
#ifdef CMD_BUF		
		uint8_t	*cmd_buf;	//shall be non-paged, and 4 bytes aligned
		uint8_t	*cmd_allocated_buf;
#endif		
#ifdef CMD_RSP_BUF
		uint8_t	*rsp_buf;	//shall be non-paged, and 4 bytes aligned		
		uint8_t	*rsp_allocated_buf;
		uint32_t	rsp_cnt;
#endif	
#ifdef CMD_DBG
		uint8_t	cmd_seq;
		uint32_t	cmd_issued_cnt;
		uint32_t	cmd_done_cnt;
#endif		
		uint8_t 	cmdthd_running;
		_adapter *padapter;
	};

#ifdef CONFIG_EVENT_THREAD_MODE
	struct evt_obj {
		uint16_t	evtcode;
		uint8_t	res;
		uint8_t	*parmbuf;
		uint32_t	evtsz;		
		_list	list;
	};
#endif

	struct	evt_priv {
#ifdef CONFIG_EVENT_THREAD_MODE
		_sema	evt_notify;
		_sema	terminate_evtthread_sema;
		_queue	evt_queue;
#endif		
		
#ifdef CONFIG_H2CLBK
		_sema	lbkevt_done;
		uint8_t	lbkevt_limit;
		uint8_t	lbkevt_num;
		uint8_t	*cmdevt_parm;		
#endif
		ATOMIC_T event_seq;
		uint8_t	*evt_buf;	//shall be non-paged, and 4 bytes aligned		
		uint8_t	*evt_allocated_buf;
		uint32_t	evt_done_cnt;
#if defined(CONFIG_SDIO_HCI) || defined(CONFIG_GSPI_HCI)
		uint8_t	*c2h_mem;
		uint8_t	*allocated_c2h_mem;
#ifdef PLATFORM_OS_XP
		PMDL	pc2h_mdl;
#endif
#endif

	};

#define init_h2fwcmd_w_parm_no_rsp(pcmd, pparm, code) \
do {\
	rtw_init_listhead(&pcmd->list);\
	pcmd->cmdcode = code;\
	pcmd->parmbuf = (uint8_t *)(pparm);\
	pcmd->cmdsz = sizeof (*pparm);\
	pcmd->rsp = NULL;\
	pcmd->rspsz = 0;\
} while(0)

extern uint32_t rtw_enqueue_cmd(struct cmd_priv *pcmdpriv, struct cmd_obj *obj);
extern struct cmd_obj *rtw_dequeue_cmd(struct cmd_priv *pcmdpriv);
extern void rtw_free_cmd_obj(struct cmd_obj *pcmd);

#ifdef CONFIG_EVENT_THREAD_MODE
extern uint32_t rtw_enqueue_evt(struct evt_priv *pevtpriv, struct evt_obj *obj);
extern struct evt_obj *rtw_dequeue_evt(_queue *queue);
extern void rtw_free_evt_obj(struct evt_obj *pcmd);
#endif

thread_return rtw_cmd_thread(thread_context context);

extern uint32_t rtw_init_cmd_priv (struct cmd_priv *pcmdpriv);
extern void rtw_free_cmd_priv (struct cmd_priv *pcmdpriv);

extern uint32_t rtw_init_evt_priv (struct evt_priv *pevtpriv);
extern void rtw_free_evt_priv (struct evt_priv *pevtpriv);
extern void rtw_cmd_clr_isr(struct cmd_priv *pcmdpriv);
extern void rtw_evt_notify_isr(struct evt_priv *pevtpriv);
#ifdef CONFIG_P2P
uint8_t p2p_protocol_wk_cmd(_adapter*padapter, int intCmdType );
#endif //CONFIG_P2P

#else	/* CONFIG_RTL8711FW */
	#include <ieee80211.h>
#endif	/* CONFIG_RTL8711FW */

enum rtw_drvextra_cmd_id
{	
	NONE_WK_CID,
	DYNAMIC_CHK_WK_CID,
	DM_CTRL_WK_CID,
	PBC_POLLING_WK_CID,
	POWER_SAVING_CTRL_WK_CID,//IPS,AUTOSuspend
	LPS_CTRL_WK_CID,
	ANT_SELECT_WK_CID,
	P2P_PS_WK_CID,
	//P2P_PROTO_WK_CID,
	CHECK_HIQ_WK_CID,//for softap mode, check hi queue if empty
	INTEl_WIDI_WK_CID,
	C2H_WK_CID,
	RTP_TIMER_CFG_WK_CID,
	MAX_WK_CID
};

enum LPS_CTRL_TYPE
{
	LPS_CTRL_SCAN=0,
	LPS_CTRL_JOINBSS=1,
	LPS_CTRL_CONNECT=2,
	LPS_CTRL_DISCONNECT=3,
	LPS_CTRL_SPECIAL_PACKET=4,
	LPS_CTRL_LEAVE=5,
};

enum RFINTFS {
	SWSI,
	HWSI,
	HWPI,
};

/*
Caller Mode: Infra, Ad-HoC(C)

Notes: To enter USB suspend mode

Command Mode

*/
struct usb_suspend_parm {
	uint32_t action;// 1: sleep, 0:resume
};

/*
Caller Mode: Infra, Ad-HoC

Notes: To join a known BSS.

Command-Event Mode

*/

/*
Caller Mode: Infra, Ad-Hoc

Notes: To join the specified bss

Command Event Mode

*/
struct joinbss_parm {
	WLAN_BSSID_EX network;
};

/*
Caller Mode: Infra, Ad-HoC(C)

Notes: To disconnect the current associated BSS

Command Mode

*/
struct disconnect_parm {
	uint32_t rsvd;
};

/*
Caller Mode: AP, Ad-HoC(M)

Notes: To create a BSS

Command Mode
*/
struct createbss_parm {
	WLAN_BSSID_EX network;
};

/*
Caller Mode: AP, Ad-HoC, Infra

Notes: To set the NIC mode of RTL8711

Command Mode

The definition of mode:

#define IW_MODE_AUTO	0	// Let the driver decides which AP to join
#define IW_MODE_ADHOC	1	// Single cell network (Ad-Hoc Clients)
#define IW_MODE_INFRA	2	// Multi cell network, roaming, ..
#define IW_MODE_MASTER	3	// Synchronisation master or Access Point
#define IW_MODE_REPEAT	4	// Wireless Repeater (forwarder)
#define IW_MODE_SECOND	5	// Secondary master/repeater (backup)
#define IW_MODE_MONITOR	6	// Passive monitor (listen only)

*/
struct	setopmode_parm {
	uint8_t	mode;
	uint8_t	rsvd[3];
};

/*
Caller Mode: AP, Ad-HoC, Infra

Notes: To ask RTL8711 performing site-survey

Command-Event Mode 

*/

#if defined(PLATFORM_ECOS) || defined(PLATFORM_FREERTOS) || defined (PLATFORM_CMSIS_RTOS)
#define RTW_SSID_SCAN_AMOUNT 1 //Reduce ssid scan amount due to memory limitation - Alex Fang
#else
#define RTW_SSID_SCAN_AMOUNT 9 // for WEXT_CSCAN_AMOUNT 9
#endif

struct sitesurvey_parm {
	sint scan_mode;	//active: 1, passive: 0 
	sint bsslimit;	// 1 ~ 48
	// for up to 9 probreq with specific ssid
	NDIS_802_11_SSID ssid[RTW_SSID_SCAN_AMOUNT];
};

/*
Caller Mode: Any

Notes: To set the auth type of RTL8711. open/shared/802.1x

Command Mode

*/
struct setauth_parm {
	uint8_t mode;  //0: legacy open, 1: legacy shared 2: 802.1x
	uint8_t _1x;   //0: PSK, 1: TLS
	uint8_t rsvd[2];
};

/*
Caller Mode: Infra

a. algorithm: wep40, wep104, tkip & aes
b. keytype: grp key/unicast key
c. key contents

when shared key ==> keyid is the camid
when 802.1x ==> keyid [0:1] ==> grp key
when 802.1x ==> keyid > 2 ==> unicast key

*/
struct setkey_parm {
	uint8_t	algorithm;	// encryption algorithm, could be none, wep40, TKIP, CCMP, wep104
	uint8_t	keyid;		
	uint8_t 	grpkey;		// 1: this is the grpkey for 802.1x. 0: this is the unicast key for 802.1x
	uint8_t 	set_tx;		// 1: main tx key for wep. 0: other key.
	uint8_t	key[16];	// this could be 40 or 104
};

/*
When in AP or Ad-Hoc mode, this is used to 
allocate an sw/hw entry for a newly associated sta.

Command

when shared key ==> algorithm/keyid 

*/
struct set_stakey_parm {
	uint8_t	addr[ETH_ALEN];
	uint8_t	algorithm;
	uint8_t 	id;// currently for erasing cam entry if algorithm == _NO_PRIVACY_ 
	uint8_t	key[16];
};

struct set_stakey_rsp {
	uint8_t	addr[ETH_ALEN];
	uint8_t	keyid;
	uint8_t	rsvd;
};

/*
Caller Ad-Hoc/AP

Command -Rsp(AID == CAMID) mode

This is to force fw to add an sta_data entry per driver's request.

FW will write an cam entry associated with it.

*/
struct set_assocsta_parm {
	uint8_t	addr[ETH_ALEN];
};

struct set_assocsta_rsp {
	uint8_t	cam_id;
	uint8_t	rsvd[3];
};

/*
	Caller Ad-Hoc/AP
	
	Command mode
	
	This is to force fw to del an sta_data entry per driver's request
	
	FW will invalidate the cam entry associated with it.

*/
struct del_assocsta_parm {
	uint8_t  	addr[ETH_ALEN];
};

/*
Caller Mode: AP/Ad-HoC(M)

Notes: To notify fw that given staid has changed its power state

Command Mode

*/
struct setstapwrstate_parm {
	uint8_t	staid;
	uint8_t	status;
	uint8_t	hwaddr[6];
};

/*
Caller Mode: Any

Notes: To setup the basic rate of RTL8711

Command Mode

*/
struct	setbasicrate_parm {
	uint8_t	basicrates[NumRates];
};

/*
Caller Mode: Any

Notes: To read the current basic rate

Command-Rsp Mode

*/
struct getbasicrate_parm {
	uint32_t rsvd;
};

struct getbasicrate_rsp {
	uint8_t basicrates[NumRates];
};

/*
Caller Mode: Any

Notes: To setup the data rate of RTL8711

Command Mode

*/
struct setdatarate_parm {
#ifdef MP_FIRMWARE_OFFLOAD
	uint32_t	curr_rateidx;
#else
	uint8_t	mac_id;
	uint8_t	datarates[NumRates];
#endif
};

/*
Caller Mode: Any

Notes: To read the current data rate

Command-Rsp Mode

*/
struct getdatarate_parm {
	uint32_t rsvd;
	
};
struct getdatarate_rsp {
	uint8_t datarates[NumRates];
};


/*
Caller Mode: Any
AP: AP can use the info for the contents of beacon frame
Infra: STA can use the info when sitesurveying
Ad-HoC(M): Like AP
Ad-HoC(C): Like STA


Notes: To set the phy capability of the NIC

Command Mode

*/

struct	setphyinfo_parm {
	struct regulatory_class class_sets[NUM_REGULATORYS];
	uint8_t	status;
};

struct	getphyinfo_parm {
	uint32_t rsvd;
};

struct	getphyinfo_rsp {
	struct regulatory_class class_sets[NUM_REGULATORYS];
	uint8_t	status;
};

/*
Caller Mode: Any

Notes: To set the channel/modem/band
This command will be used when channel/modem/band is changed.

Command Mode

*/
struct	setphy_parm {
	uint8_t	rfchannel;
	uint8_t	modem;
};

/*
Caller Mode: Any

Notes: To get the current setting of channel/modem/band

Command-Rsp Mode

*/
struct	getphy_parm {
	uint32_t rsvd;

};
struct	getphy_rsp {
	uint8_t	rfchannel;
	uint8_t	modem;
};

struct readBB_parm {
	uint8_t	offset;
};
struct readBB_rsp {
	uint8_t	value;
};

struct readTSSI_parm {
	uint8_t	offset;
};
struct readTSSI_rsp {
	uint8_t	value;
};

struct writeBB_parm {
	uint8_t	offset;
	uint8_t	value;
};

struct readRF_parm {
	uint8_t	offset;
};
struct readRF_rsp {
	uint32_t	value;
};

struct writeRF_parm {
	uint32_t	offset;
	uint32_t	value;
};

struct getrfintfs_parm {
	uint8_t	rfintfs;
};


struct Tx_Beacon_param
{
	WLAN_BSSID_EX network;
};

/*
	Notes: This command is used for H2C/C2H loopback testing

	mac[0] == 0 
	==> CMD mode, return H2C_SUCCESS.
	The following condition must be ture under CMD mode
		mac[1] == mac[4], mac[2] == mac[3], mac[0]=mac[5]= 0;
		s0 == 0x1234, s1 == 0xabcd, w0 == 0x78563412, w1 == 0x5aa5def7;
		s2 == (b1 << 8 | b0);
	
	mac[0] == 1
	==> CMD_RSP mode, return H2C_SUCCESS_RSP
	
	The rsp layout shall be:
	rsp: 			parm:
		mac[0]  =   mac[5];
		mac[1]  =   mac[4];
		mac[2]  =   mac[3];
		mac[3]  =   mac[2];
		mac[4]  =   mac[1];
		mac[5]  =   mac[0];
		s0		=   s1;
		s1		=   swap16(s0);
		w0		=  	swap32(w1);
		b0		= 	b1
		s2		= 	s0 + s1
		b1		= 	b0
		w1		=	w0
		
	mac[0] == 	2
	==> CMD_EVENT mode, return 	H2C_SUCCESS
	The event layout shall be:
	event:			parm:
		mac[0]  =   mac[5];
		mac[1]  =   mac[4];
		mac[2]  =   event's sequence number, starting from 1 to parm's marc[3]
		mac[3]  =   mac[2];
		mac[4]  =   mac[1];
		mac[5]  =   mac[0];
		s0		=   swap16(s0) - event.mac[2];
		s1		=   s1 + event.mac[2];
		w0		=  	swap32(w0);
		b0		= 	b1
		s2		= 	s0 + event.mac[2]
		b1		= 	b0 
		w1		=	swap32(w1) - event.mac[2];	
	
		parm->mac[3] is the total event counts that host requested.
		
	
	event will be the same with the cmd's param.
		
*/

#ifdef CONFIG_H2CLBK

struct seth2clbk_parm {
	uint8_t mac[6];
	uint16_t	s0;
	uint16_t	s1;
	uint32_t	w0;
	uint8_t	b0;
	uint16_t  s2;
	uint8_t	b1;
	uint32_t	w1;
};

struct geth2clbk_parm {
	uint32_t rsv;	
};

struct geth2clbk_rsp {
	uint8_t	mac[6];
	uint16_t	s0;
	uint16_t	s1;
	uint32_t	w0;
	uint8_t	b0;
	uint16_t	s2;
	uint8_t	b1;
	uint32_t	w1;
};

#endif	/* CONFIG_H2CLBK */

// CMD param Formart for driver extra cmd handler
struct drvextra_cmd_parm {
	int ec_id; //extra cmd id
	int type_size; // Can use this field as the type id or command size
	unsigned char *pbuf;
};

#ifdef CONFIG_P2P_NEW
// CMD param Formart for p2p cmd handler
struct p2p_cmd_parm {
	int id; //p2p cmd id
	int type_size; // Can use this field as the type id or command size
	unsigned char *pbuf;
};
#endif
/*------------------- Below are used for RF/BB tunning ---------------------*/

struct	setantenna_parm {
	uint8_t	tx_antset;		
	uint8_t	rx_antset;
	uint8_t	tx_antenna;		
	uint8_t	rx_antenna;		
};

struct	enrateadaptive_parm {
	uint32_t	en;
};

struct settxagctbl_parm {
	uint32_t	txagc[MAX_RATES_LENGTH];
};

struct gettxagctbl_parm {
	uint32_t rsvd;
};
struct gettxagctbl_rsp {
	uint32_t	txagc[MAX_RATES_LENGTH];
};

struct setagcctrl_parm {
	uint32_t	agcctrl;		// 0: pure hw, 1: fw
};


struct setssup_parm	{
	uint32_t	ss_ForceUp[MAX_RATES_LENGTH];
};

struct getssup_parm	{
	uint32_t rsvd;
};
struct getssup_rsp	{
	uint8_t	ss_ForceUp[MAX_RATES_LENGTH];
};


struct setssdlevel_parm	{
	uint8_t	ss_DLevel[MAX_RATES_LENGTH];
};

struct getssdlevel_parm	{
	uint32_t rsvd;
};
struct getssdlevel_rsp	{
	uint8_t	ss_DLevel[MAX_RATES_LENGTH];
};

struct setssulevel_parm	{
	uint8_t	ss_ULevel[MAX_RATES_LENGTH];
};

struct getssulevel_parm	{
	uint32_t rsvd;
};
struct getssulevel_rsp	{
	uint8_t	ss_ULevel[MAX_RATES_LENGTH];
};


struct	setcountjudge_parm {
	uint8_t	count_judge[MAX_RATES_LENGTH];
};

struct	getcountjudge_parm {
	uint32_t rsvd;
};
struct	getcountjudge_rsp {
	uint8_t	count_judge[MAX_RATES_LENGTH];
};


struct setratable_parm {
	uint8_t ss_ForceUp[NumRates];
	uint8_t ss_ULevel[NumRates];
	uint8_t ss_DLevel[NumRates];
	uint8_t count_judge[NumRates];
};

struct getratable_parm {
                uint rsvd;
};
struct getratable_rsp {
        uint8_t ss_ForceUp[NumRates];
        uint8_t ss_ULevel[NumRates];
        uint8_t ss_DLevel[NumRates];
        uint8_t count_judge[NumRates];
};


//to get TX,RX retry count
struct gettxretrycnt_parm{
	unsigned int rsvd;
};
struct gettxretrycnt_rsp{
	unsigned long tx_retrycnt;
};

struct getrxretrycnt_parm{
	unsigned int rsvd;
};
struct getrxretrycnt_rsp{
	unsigned long rx_retrycnt;
};

//to get BCNOK,BCNERR count
struct getbcnokcnt_parm{
	unsigned int rsvd;
};
struct getbcnokcnt_rsp{
	unsigned long  bcnokcnt;
};

struct getbcnerrcnt_parm{
	unsigned int rsvd;
};
struct getbcnerrcnt_rsp{
	unsigned long bcnerrcnt;
};

// to get current TX power level
struct getcurtxpwrlevel_parm{
	unsigned int rsvd;
};
struct getcurtxpwrlevel_rsp{
	unsigned short tx_power;
};

///TODO
#if 0

struct setprobereqextraie_parm {
	unsigned char e_id;
	unsigned char ie_len;
	unsigned char ie[0];
};

struct setassocreqextraie_parm {
	unsigned char e_id;
	unsigned char ie_len;
	unsigned char ie[0];
};

struct setproberspextraie_parm {
	unsigned char e_id;
	unsigned char ie_len;
	unsigned char ie[0];
};

struct setassocrspextraie_parm {
	unsigned char e_id;
	unsigned char ie_len;
	unsigned char ie[0];
};

#endif	//#if 0

struct addBaReq_parm
{
 	unsigned int tid;
	uint8_t	addr[ETH_ALEN];
};

/*H2C Handler index: 46 */
struct SetChannel_parm
{
	uint32_t curr_ch;	
};

#ifdef MP_FIRMWARE_OFFLOAD
/*H2C Handler index: 47 */
struct SetTxPower_parm
{
	uint8_t TxPower;
};

/*H2C Handler index: 48 */
struct SwitchAntenna_parm
{
	uint16_t antenna_tx;
	uint16_t antenna_rx;
//	R_ANTENNA_SELECT_CCK cck_txrx;
	uint8_t cck_txrx;
};

/*H2C Handler index: 49 */
struct SetCrystalCap_parm
{
	uint32_t curr_crystalcap;
};

/*H2C Handler index: 50 */
struct SetSingleCarrierTx_parm
{
	uint8_t bStart;
};

/*H2C Handler index: 51 */
struct SetSingleToneTx_parm
{
	uint8_t bStart;
	uint8_t curr_rfpath;
};

/*H2C Handler index: 52 */
struct SetCarrierSuppressionTx_parm
{
	uint8_t bStart;
	uint32_t curr_rateidx;
};

/*H2C Handler index: 53 */
struct SetContinuousTx_parm
{
	uint8_t bStart;
	uint8_t CCK_flag; /*1:CCK 2:OFDM*/
	uint32_t curr_rateidx;
};

/*H2C Handler index: 54 */
struct SwitchBandwidth_parm
{
	uint8_t curr_bandwidth;
};

#endif	/* MP_FIRMWARE_OFFLOAD */

/*H2C Handler index: 59 */ 
struct SetChannelPlan_param
{
	uint8_t channel_plan;
};

//TODO
#if 0
/*H2C Handler index: 60 */ 
struct LedBlink_param
{
	PLED_871x	 pLed;
};
#endif	//#if 0

/*H2C Handler index: 61 */ 
struct SetChannelSwitch_param
{
	uint8_t new_ch_no;
};

/*H2C Handler index: 62 */ 
struct TDLSoption_param
{
	uint8_t addr[ETH_ALEN];
	uint8_t option;
};

#define GEN_CMD_CODE(cmd)	cmd ## _CMD_


/*

Result: 
0x00: success
0x01: sucess, and check Response.
0x02: cmd ignored due to duplicated sequcne number
0x03: cmd dropped due to invalid cmd code
0x04: reserved.

*/

#define H2C_RSP_OFFSET			512

#define H2C_SUCCESS			0x00
#define H2C_SUCCESS_RSP			0x01
#define H2C_DUPLICATED			0x02
#define H2C_DROPPED			0x03
#define H2C_PARAMETERS_ERROR		0x04
#define H2C_REJECTED			0x05
#define H2C_CMD_OVERFLOW		0x06
#define H2C_RESERVED			0x07

extern uint8_t rtw_setassocsta_cmd(_adapter  *padapter, uint8_t *mac_addr);
extern uint8_t rtw_setstandby_cmd(_adapter *padapter, uint action);
extern uint8_t rtw_sitesurvey_cmd(_adapter  *padapter, NDIS_802_11_SSID *pssid, int ssid_max_num);
extern uint8_t rtw_createbss_cmd(_adapter  *padapter);
extern uint8_t rtw_createbss_cmd_ex(_adapter  *padapter, unsigned char *pbss, unsigned int sz);
extern uint8_t rtw_setphy_cmd(_adapter  *padapter, uint8_t modem, uint8_t ch);
extern uint8_t rtw_setstakey_cmd(_adapter  *padapter, uint8_t *psta, uint8_t unicast_key);
extern uint8_t rtw_clearstakey_cmd(_adapter *padapter, uint8_t *psta, uint8_t entry, uint8_t enqueue);
extern uint8_t rtw_joinbss_cmd(_adapter  *padapter, struct wlan_network* pnetwork);
extern uint8_t rtw_disassoc_cmd(_adapter  *padapter);
extern uint8_t rtw_setopmode_cmd(_adapter  *padapter, NDIS_802_11_NETWORK_INFRASTRUCTURE networktype);
extern uint8_t rtw_setdatarate_cmd(_adapter  *padapter, uint8_t *rateset);
extern uint8_t rtw_setbasicrate_cmd(_adapter  *padapter, uint8_t *rateset);
extern uint8_t rtw_setbbreg_cmd(_adapter * padapter, uint8_t offset, uint8_t val);
extern uint8_t rtw_setrfreg_cmd(_adapter * padapter, uint8_t offset, uint32_t val);
extern uint8_t rtw_getbbreg_cmd(_adapter * padapter, uint8_t offset, uint8_t * pval);
extern uint8_t rtw_getrfreg_cmd(_adapter * padapter, uint8_t offset, uint8_t * pval);
extern uint8_t rtw_setrfintfs_cmd(_adapter  *padapter, uint8_t mode);
extern uint8_t rtw_setrttbl_cmd(_adapter  *padapter, struct setratable_parm *prate_table);
extern uint8_t rtw_getrttbl_cmd(_adapter  *padapter, struct getratable_rsp *pval);

extern uint8_t rtw_gettssi_cmd(_adapter  *padapter, uint8_t offset,uint8_t *pval);
extern uint8_t rtw_setfwdig_cmd(_adapter*padapter, uint8_t type);
extern uint8_t rtw_setfwra_cmd(_adapter*padapter, uint8_t type);

extern uint8_t rtw_addbareq_cmd(_adapter*padapter, uint8_t tid, uint8_t *addr);

extern uint8_t rtw_dynamic_chk_wk_cmd(_adapter *adapter);
#ifdef CONFIG_P2P_NEW
uint8_t rtw_p2p_cmd(_adapter*padapter, int subid);
#endif
uint8_t rtw_lps_ctrl_wk_cmd(_adapter*padapter, uint8_t lps_ctrl_type, uint8_t enqueue);
#if (RATE_ADAPTIVE_SUPPORT==1)
uint8_t rtw_rpt_timer_cfg_cmd(_adapter*padapter, uint16_t minRptTime);
#endif

#ifdef CONFIG_ANTENNA_DIVERSITY
extern  uint8_t rtw_antenna_select_cmd(_adapter*padapter, uint8_t antenna,uint8_t enqueue);
#endif

extern uint8_t rtw_ps_cmd(_adapter*padapter);

#ifdef CONFIG_AP_MODE
uint8_t rtw_chk_hi_queue_cmd(_adapter*padapter);
#endif

extern uint8_t rtw_set_chplan_cmd(_adapter*padapter, uint8_t chplan, uint8_t enaueue);
//TODO
//extern uint8_t rtw_led_blink_cmd(_adapter*padapter, PLED_871x pLed);
extern uint8_t rtw_set_csa_cmd(_adapter*padapter, uint8_t new_ch_no);
extern uint8_t rtw_tdls_cmd(_adapter*padapter, uint8_t *addr, uint8_t option);

extern uint8_t rtw_c2h_wk_cmd(PADAPTER padapter);

uint8_t rtw_drvextra_cmd_hdl(_adapter *padapter, unsigned char *pbuf);
#ifdef CONFIG_P2P_NEW
uint8_t rtw_p2p_cmd_hdl(_adapter *padapter, unsigned char *pbuf);
#endif
extern void rtw_survey_cmd_callback(_adapter  *padapter, struct cmd_obj *pcmd);
extern void rtw_disassoc_cmd_callback(_adapter  *padapter, struct cmd_obj *pcmd);
extern void rtw_joinbss_cmd_callback(_adapter  *padapter, struct cmd_obj *pcmd);	
extern void rtw_createbss_cmd_callback(_adapter  *padapter, struct cmd_obj *pcmd);
extern void rtw_getbbrfreg_cmdrsp_callback(_adapter  *padapter, struct cmd_obj *pcmd);
extern void rtw_readtssi_cmdrsp_callback(_adapter*	padapter,  struct cmd_obj *pcmd);

extern void rtw_setstaKey_cmdrsp_callback(_adapter  *padapter,  struct cmd_obj *pcmd);
extern void rtw_setassocsta_cmdrsp_callback(_adapter  *padapter,  struct cmd_obj *pcmd);
extern void rtw_getrttbl_cmdrsp_callback(_adapter  *padapter,  struct cmd_obj *pcmd);
extern void rtw_set_channel_plan_cmd_callback(_adapter*	padapter,  struct cmd_obj *pcmd);


struct _cmd_callback {
	uint32_t	cmd_code;
	void (*callback)(_adapter  *padapter, struct cmd_obj *cmd);
};

enum rtw_h2c_cmd
{
	GEN_CMD_CODE(_Read_MACREG) ,	/*0*/
 	GEN_CMD_CODE(_Write_MACREG) ,    
 	GEN_CMD_CODE(_Read_BBREG) ,  
 	GEN_CMD_CODE(_Write_BBREG) ,  
 	GEN_CMD_CODE(_Read_RFREG) ,  
 	GEN_CMD_CODE(_Write_RFREG) , /*5*/
 	GEN_CMD_CODE(_Read_EEPROM) ,  
 	GEN_CMD_CODE(_Write_EEPROM) ,  
 	GEN_CMD_CODE(_Read_EFUSE) ,  
 	GEN_CMD_CODE(_Write_EFUSE) , 
 	
 	GEN_CMD_CODE(_Read_CAM) ,	/*10*/
 	GEN_CMD_CODE(_Write_CAM) ,   
 	GEN_CMD_CODE(_setBCNITV),
 	GEN_CMD_CODE(_setMBIDCFG),
 	GEN_CMD_CODE(_JoinBss),   /*14*/
 	GEN_CMD_CODE(_DisConnect) , /*15*/
 	GEN_CMD_CODE(_CreateBss) ,
	GEN_CMD_CODE(_SetOpMode) , 
	GEN_CMD_CODE(_SiteSurvey),  /*18*/
 	GEN_CMD_CODE(_SetAuth) ,
 	
 	GEN_CMD_CODE(_SetKey) ,	/*20*/
 	GEN_CMD_CODE(_SetStaKey) ,
 	GEN_CMD_CODE(_SetAssocSta) ,
 	GEN_CMD_CODE(_DelAssocSta) ,
 	GEN_CMD_CODE(_SetStaPwrState) , 
 	GEN_CMD_CODE(_SetBasicRate) , /*25*/
 	GEN_CMD_CODE(_GetBasicRate) ,
 	GEN_CMD_CODE(_SetDataRate) ,
 	GEN_CMD_CODE(_GetDataRate) ,
	GEN_CMD_CODE(_SetPhyInfo) ,
	
 	GEN_CMD_CODE(_GetPhyInfo) ,	/*30*/
	GEN_CMD_CODE(_SetPhy) ,
 	GEN_CMD_CODE(_GetPhy) ,
 	GEN_CMD_CODE(_readRssi) ,
 	GEN_CMD_CODE(_readGain) ,
 	GEN_CMD_CODE(_SetAtim) , /*35*/
 	GEN_CMD_CODE(_SetPwrMode) , 
 	GEN_CMD_CODE(_JoinbssRpt),
 	GEN_CMD_CODE(_SetRaTable) ,
 	GEN_CMD_CODE(_GetRaTable) ,  	
 	
 	GEN_CMD_CODE(_GetCCXReport), /*40*/
 	GEN_CMD_CODE(_GetDTMReport),
 	GEN_CMD_CODE(_GetTXRateStatistics),
 	GEN_CMD_CODE(_SetUsbSuspend),
 	GEN_CMD_CODE(_SetH2cLbk),
 	GEN_CMD_CODE(_AddBAReq) , /*45*/
	GEN_CMD_CODE(_SetChannel), /*46*/
	GEN_CMD_CODE(_SetTxPower), 
	GEN_CMD_CODE(_SwitchAntenna),
	GEN_CMD_CODE(_SetCrystalCap),
	GEN_CMD_CODE(_SetSingleCarrierTx), /*50*/
	
	GEN_CMD_CODE(_SetSingleToneTx),/*51*/
	GEN_CMD_CODE(_SetCarrierSuppressionTx),
	GEN_CMD_CODE(_SetContinuousTx),
	GEN_CMD_CODE(_SwitchBandwidth), /*54*/
	GEN_CMD_CODE(_TX_Beacon), /*55*/
	
	GEN_CMD_CODE(_Set_MLME_EVT), /*56*/
	GEN_CMD_CODE(_Set_Drv_Extra), /*57*/
	GEN_CMD_CODE(_Set_H2C_MSG), /*58*/
	
	GEN_CMD_CODE(_SetChannelPlan), /*59*/
	GEN_CMD_CODE(_LedBlink), /*60*/

	GEN_CMD_CODE(_SetChannelSwitch), /*61*/
	GEN_CMD_CODE(_TDLS), /*62*/
	GEN_CMD_CODE(_P2P), /*63*/
	
	MAX_H2CCMD
};

#define _GetBBReg_CMD_		_Read_BBREG_CMD_
#define _SetBBReg_CMD_ 		_Write_BBREG_CMD_
#define _GetRFReg_CMD_ 		_Read_RFREG_CMD_
#define _SetRFReg_CMD_ 		_Write_RFREG_CMD_

#ifdef _RTW_CMD_C_
const struct _cmd_callback 	rtw_cmd_callback[] = 
{
	{GEN_CMD_CODE(_Read_MACREG), NULL}, /*0*/
	{GEN_CMD_CODE(_Write_MACREG), NULL}, 
//TODO
//	{GEN_CMD_CODE(_Read_BBREG), &rtw_getbbrfreg_cmdrsp_callback},
	{GEN_CMD_CODE(_Read_BBREG), NULL},
	{GEN_CMD_CODE(_Write_BBREG), NULL},
//TODO
//	{GEN_CMD_CODE(_Read_RFREG), &rtw_getbbrfreg_cmdrsp_callback},
	{GEN_CMD_CODE(_Read_RFREG), NULL},
	{GEN_CMD_CODE(_Write_RFREG), NULL}, /*5*/
	{GEN_CMD_CODE(_Read_EEPROM), NULL},
	{GEN_CMD_CODE(_Write_EEPROM), NULL},
	{GEN_CMD_CODE(_Read_EFUSE), NULL},
	{GEN_CMD_CODE(_Write_EFUSE), NULL},
	
	{GEN_CMD_CODE(_Read_CAM), NULL},  /*10*/
	{GEN_CMD_CODE(_Write_CAM), NULL},	
	{GEN_CMD_CODE(_setBCNITV), NULL},
 	{GEN_CMD_CODE(_setMBIDCFG), NULL},
	{GEN_CMD_CODE(_JoinBss), &rtw_joinbss_cmd_callback},  /*14*/
	{GEN_CMD_CODE(_DisConnect), &rtw_disassoc_cmd_callback}, /*15*/
//TODO
//	{GEN_CMD_CODE(_CreateBss), &rtw_createbss_cmd_callback},
	{GEN_CMD_CODE(_CreateBss), NULL},
	{GEN_CMD_CODE(_SetOpMode), NULL},
	{GEN_CMD_CODE(_SiteSurvey), &rtw_survey_cmd_callback}, /*18*/
	{GEN_CMD_CODE(_SetAuth), NULL},
	
	{GEN_CMD_CODE(_SetKey), NULL},	/*20*/
	{GEN_CMD_CODE(_SetStaKey), &rtw_setstaKey_cmdrsp_callback},
//TODO
//	{GEN_CMD_CODE(_SetAssocSta), &rtw_setassocsta_cmdrsp_callback},
	{GEN_CMD_CODE(_SetAssocSta), NULL},
	{GEN_CMD_CODE(_DelAssocSta), NULL},	
	{GEN_CMD_CODE(_SetStaPwrState), NULL},	
	{GEN_CMD_CODE(_SetBasicRate), NULL}, /*25*/
	{GEN_CMD_CODE(_GetBasicRate), NULL},
	{GEN_CMD_CODE(_SetDataRate), NULL},
	{GEN_CMD_CODE(_GetDataRate), NULL},
	{GEN_CMD_CODE(_SetPhyInfo), NULL},
	
	{GEN_CMD_CODE(_GetPhyInfo), NULL}, /*30*/
	{GEN_CMD_CODE(_SetPhy), NULL},
	{GEN_CMD_CODE(_GetPhy), NULL},	
	{GEN_CMD_CODE(_readRssi), NULL},
	{GEN_CMD_CODE(_readGain), NULL},
	{GEN_CMD_CODE(_SetAtim), NULL}, /*35*/
	{GEN_CMD_CODE(_SetPwrMode), NULL},
	{GEN_CMD_CODE(_JoinbssRpt), NULL},
	{GEN_CMD_CODE(_SetRaTable), NULL},
	{GEN_CMD_CODE(_GetRaTable) , NULL},
 	
	{GEN_CMD_CODE(_GetCCXReport), NULL}, /*40*/
 	{GEN_CMD_CODE(_GetDTMReport),	NULL},
 	{GEN_CMD_CODE(_GetTXRateStatistics), NULL}, 
 	{GEN_CMD_CODE(_SetUsbSuspend), NULL}, 
 	{GEN_CMD_CODE(_SetH2cLbk), NULL},
 	{GEN_CMD_CODE(_AddBAReq), NULL}, /*45*/
	{GEN_CMD_CODE(_SetChannel), NULL},		/*46*/
	{GEN_CMD_CODE(_SetTxPower), NULL},
	{GEN_CMD_CODE(_SwitchAntenna), NULL},
	{GEN_CMD_CODE(_SetCrystalCap), NULL},
	{GEN_CMD_CODE(_SetSingleCarrierTx), NULL},	/*50*/
	
	{GEN_CMD_CODE(_SetSingleToneTx), NULL}, /*51*/
	{GEN_CMD_CODE(_SetCarrierSuppressionTx), NULL},
	{GEN_CMD_CODE(_SetContinuousTx), NULL},
	{GEN_CMD_CODE(_SwitchBandwidth), NULL},		/*54*/
	{GEN_CMD_CODE(_TX_Beacon), NULL},/*55*/

	{GEN_CMD_CODE(_Set_MLME_EVT), NULL},/*56*/
	{GEN_CMD_CODE(_Set_Drv_Extra), NULL},/*57*/
	{GEN_CMD_CODE(_Set_H2C_MSG), NULL},/*58*/
	{GEN_CMD_CODE(_SetChannelPlan), rtw_set_channel_plan_cmd_callback},/*59*/
	{GEN_CMD_CODE(_LedBlink), NULL},/*60*/
	
	{GEN_CMD_CODE(_SetChannelSwitch), NULL},/*61*/
	{GEN_CMD_CODE(_TDLS), NULL},/*62*/
#ifdef CONFIG_P2P_NEW
	{GEN_CMD_CODE(_P2P), NULL},/*63*/
#endif
};
#endif

#endif // _CMD_H_

