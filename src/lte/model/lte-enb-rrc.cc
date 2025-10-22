/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 * Copyright (c) 2018 Fraunhofer ESK : RLF extensions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Nicola Baldo <nbaldo@cttc.es>
 *          Marco Miozzo <mmiozzo@cttc.es>
 *          Manuel Requena <manuel.requena@cttc.es>
 * Modified by:  Danilo Abrignani <danilo.abrignani@unibo.it> (Carrier Aggregation - GSoC 2015),
 *               Biljana Bojovic <biljana.bojovic@cttc.es> (Carrier Aggregation)
 *               Vignesh Babu <ns3-dev@esk.fraunhofer.de> (RLF extensions)
 */

#include "lte-enb-rrc.h"

#include <ns3/fatal-error.h>
#include <ns3/log.h>
#include <ns3/abort.h>

#include <ns3/pointer.h>
#include <ns3/object-map.h>
#include <ns3/object-factory.h>
#include <ns3/simulator.h>

#include <ns3/lte-radio-bearer-info.h>
#include <ns3/eps-bearer-tag.h>
#include <ns3/packet.h>
#include <algorithm>
#include <ns3/lte-rlc.h>
#include <ns3/lte-rlc-tm.h>
#include <ns3/lte-rlc-um.h>
#include <ns3/lte-rlc-am.h>
#include <ns3/lte-pdcp.h>

#include <ns3/ideal-beamforming-helper.h>

#include <ns3/epc-x2-sap.h>

#include <iomanip>
#include <chrono>

namespace ns3
{

static void
LogOrnsteinUhlenbeckParamsTest(OrnsteinUhlenbeckErrorTraceParams params)
{
  std::cout
        << params.m_theta << "," << params.m_mu << "," << params.m_sigma << std::endl // parameters
        << params.m_deltaT << "," << params.m_deltaW << std::endl    // x,y error applied to this position
        << params.m_Xprev << "," << params.m_Xnext << std::endl    // x,y error applied to this position
        << params.m_restoringTerm << "," << params.m_stochasticTerm << std::endl;    // x,y error applied to this position
}

static void
LogOrnsteinUhlenbeckParamsTestContext(std::string context, OrnsteinUhlenbeckErrorTraceParams params)
{
  std::cout << "context: " << context << std::endl;
  std::cout
        << params.m_theta << "," << params.m_mu << "," << params.m_sigma << std::endl // parameters
        << params.m_deltaT << "," << params.m_deltaW << std::endl    // x,y error applied to this position
        << params.m_Xprev << "," << params.m_Xnext << std::endl    // x,y error applied to this position
        << params.m_restoringTerm << "," << params.m_stochasticTerm << std::endl;    // x,y error applied to this position
}

NS_LOG_COMPONENT_DEFINE ("LteEnbRrc");

///////////////////////////////////////////
// CMAC SAP forwarder
///////////////////////////////////////////

/**
 * \brief Class for forwarding CMAC SAP User functions.
 */
class EnbRrcMemberLteEnbCmacSapUser : public LteEnbCmacSapUser
{
public:
  /**
   * Constructor
   *
   * \param rrc ENB RRC
   * \param componentCarrierId
   */
  EnbRrcMemberLteEnbCmacSapUser (LteEnbRrc* rrc, uint8_t componentCarrierId);

  virtual uint16_t AllocateTemporaryCellRnti ();
  virtual void NotifyLcConfigResult (uint16_t rnti, uint8_t lcid, bool success);
  virtual void RrcConfigurationUpdateInd (UeConfig params);
  virtual bool IsRandomAccessCompleted (uint16_t rnti);
  virtual uint64_t GetImsiFromRnti (uint16_t rnti);
  virtual uint16_t GetRntiFromImsi (uint64_t imsi);

private:
  LteEnbRrc* m_rrc; ///< the RRC
  uint8_t m_componentCarrierId; ///< Component carrier ID
};

EnbRrcMemberLteEnbCmacSapUser::EnbRrcMemberLteEnbCmacSapUser (LteEnbRrc* rrc, uint8_t componentCarrierId)
  : m_rrc (rrc)
  , m_componentCarrierId {componentCarrierId}
{
}

uint16_t
EnbRrcMemberLteEnbCmacSapUser::AllocateTemporaryCellRnti ()
{
  return m_rrc->DoAllocateTemporaryCellRnti (m_componentCarrierId);
}

void
EnbRrcMemberLteEnbCmacSapUser::NotifyLcConfigResult (uint16_t rnti, uint8_t lcid, bool success)
{
  m_rrc->DoNotifyLcConfigResult (rnti, lcid, success);
}

void
EnbRrcMemberLteEnbCmacSapUser::RrcConfigurationUpdateInd (UeConfig params)
{
  m_rrc->DoRrcConfigurationUpdateInd (params);
}

bool
EnbRrcMemberLteEnbCmacSapUser::IsRandomAccessCompleted (uint16_t rnti)
{
  return m_rrc->IsRandomAccessCompleted (rnti);
}

uint64_t
EnbRrcMemberLteEnbCmacSapUser::GetImsiFromRnti (uint16_t rnti)
{
  return m_rrc->DoGetImsiFromRnti (rnti);
}

uint16_t
EnbRrcMemberLteEnbCmacSapUser::GetRntiFromImsi (uint64_t imsi)
{
  return m_rrc->DoGetRntiFromImsi (imsi);
}

///////////////////////////////////////////
// UeManager
///////////////////////////////////////////


/// Map each of UE Manager states to its string representation.
static const std::string g_ueManagerStateName[UeManager::NUM_STATES] =
{
  "INITIAL_RANDOM_ACCESS",
  "CONNECTION_SETUP",
  "CONNECTION_REJECTED",
  "ATTACH_REQUEST",
  "CONNECTED_NORMALLY",
  "CONNECTION_RECONFIGURATION",
  "CONNECTION_REESTABLISHMENT",
  "HANDOVER_PREPARATION",
  "HANDOVER_JOINING",
  "HANDOVER_PATH_SWITCH",
  "HANDOVER_LEAVING",
  "CONNECTION_LTE_COORDINATOR",
};

/**
 * \param s The UE manager state.
 * \return The string representation of the given state.
 */
static const std::string & ToString (UeManager::State s)
{
  return g_ueManagerStateName[s];
}


NS_OBJECT_ENSURE_REGISTERED (UeManager);


UeManager::UeManager ()
{
  NS_FATAL_ERROR ("this constructor is not expected to be used");
}


UeManager::UeManager (Ptr<LteEnbRrc> rrc, uint16_t rnti, State s, uint8_t componentCarrierId)
  : m_lastAllocatedDrbid (0),
    m_rnti (rnti),
    m_imsi (0),
    m_componentCarrierId (componentCarrierId),
    m_lastRrcTransactionIdentifier (0),
    m_rrc (rrc),
    m_state (s),
    m_pendingRrcConnectionReconfiguration (false),
    m_sourceX2apId (0),
    m_sourceCellId (0),
    m_needPhyMacConfiguration (false),
    m_caSupportConfigured (false),
    m_pendingStartDataRadioBearers (false)
{ 
  NS_LOG_FUNCTION (this);
}

void
UeManager::DoInitialize ()
{
  NS_LOG_FUNCTION (this);
  m_drbPdcpSapUser = new LtePdcpSpecificLtePdcpSapUser<UeManager> (this);

  m_physicalConfigDedicated.haveAntennaInfoDedicated = true;
  m_physicalConfigDedicated.antennaInfo.transmissionMode = m_rrc->m_defaultTransmissionMode;
  m_physicalConfigDedicated.haveSoundingRsUlConfigDedicated = true;
  m_physicalConfigDedicated.soundingRsUlConfigDedicated.srsConfigIndex = m_rrc->GetNewSrsConfigurationIndex ();
  m_physicalConfigDedicated.soundingRsUlConfigDedicated.type = LteRrcSap::SoundingRsUlConfigDedicated::SETUP;
  m_physicalConfigDedicated.soundingRsUlConfigDedicated.srsBandwidth = 0;
  m_physicalConfigDedicated.havePdschConfigDedicated = true;
  m_physicalConfigDedicated.pdschConfigDedicated.pa = LteRrcSap::PdschConfigDedicated::dB0;

  
  for (uint8_t i = 0; i < m_rrc->m_numberOfComponentCarriers; i++)
    {
      m_rrc->m_cmacSapProvider.at (i)->AddUe (m_rnti);
      m_rrc->m_cphySapProvider.at (i)->AddUe (m_rnti);
    }

  // setup the eNB side of SRB0
  {
    uint8_t lcid = 0;

    Ptr<LteRlc> rlc = CreateObject<LteRlcTm> ()->GetObject<LteRlc> ();
    rlc->SetLteMacSapProvider (m_rrc->m_macSapProvider);
    rlc->SetRnti (m_rnti);
    rlc->SetLcId (lcid);

    m_srb0 = CreateObject<LteSignalingRadioBearerInfo> ();
    m_srb0->m_rlc = rlc;
    m_srb0->m_srbIdentity = 0;
    // no need to store logicalChannelConfig as SRB0 is pre-configured

    LteEnbCmacSapProvider::LcInfo lcinfo;
    lcinfo.rnti = m_rnti;
    lcinfo.lcId = lcid;
    // Initialise the rest of lcinfo structure even if CCCH (LCID 0) is pre-configured, and only m_rnti and lcid will be used from passed lcinfo structure.
    // See FF LTE MAC Scheduler Iinterface Specification v1.11, 4.3.4 logicalChannelConfigListElement
    lcinfo.lcGroup = 0;
    lcinfo.qci = 0;
    lcinfo.isGbr = false;
    lcinfo.mbrUl = 0;
    lcinfo.mbrDl = 0;
    lcinfo.gbrUl = 0;
    lcinfo.gbrDl = 0;

    // MacSapUserForRlc in the ComponentCarrierManager MacSapUser
    LteMacSapUser* lteMacSapUser = m_rrc->m_ccmRrcSapProvider->ConfigureSignalBearer(lcinfo, rlc->GetLteMacSapUser ()); 
    // Signal Channel are only on Primary Carrier
    m_rrc->m_cmacSapProvider.at (m_componentCarrierId)->AddLc (lcinfo, lteMacSapUser);
    m_rrc->m_ccmRrcSapProvider->AddLc (lcinfo, lteMacSapUser);
  }

  // setup the eNB side of SRB1; the UE side will be set up upon RRC connection establishment
  {
    uint8_t lcid = 1;

    Ptr<LteRlc> rlc = CreateObject<LteRlcAm> ()->GetObject<LteRlc> ();
    rlc->SetLteMacSapProvider (m_rrc->m_macSapProvider);
    rlc->SetRnti (m_rnti);
    rlc->SetLcId (lcid);

    Ptr<LtePdcp> pdcp = CreateObject<LtePdcp> ();
    pdcp->SetRnti (m_rnti);
    pdcp->SetLcId (lcid);
    pdcp->SetLtePdcpSapUser (m_drbPdcpSapUser);
    pdcp->SetLteRlcSapProvider (rlc->GetLteRlcSapProvider ());
    rlc->SetLteRlcSapUser (pdcp->GetLteRlcSapUser ());

    m_srb1 = CreateObject<LteSignalingRadioBearerInfo> ();
    m_srb1->m_rlc = rlc;
    m_srb1->m_pdcp = pdcp;
    m_srb1->m_srbIdentity = 1;
    m_srb1->m_logicalChannelConfig.priority = 1;
    m_srb1->m_logicalChannelConfig.prioritizedBitRateKbps = 100;
    m_srb1->m_logicalChannelConfig.bucketSizeDurationMs = 100;
    m_srb1->m_logicalChannelConfig.logicalChannelGroup = 0;

    LteEnbCmacSapProvider::LcInfo lcinfo;
    lcinfo.rnti = m_rnti;
    lcinfo.lcId = lcid;
    lcinfo.lcGroup = 0; // all SRBs always mapped to LCG 0
    lcinfo.qci = EpsBearer::GBR_CONV_VOICE; // not sure why the FF API requires a CQI even for SRBs...
    lcinfo.isGbr = true;
    lcinfo.mbrUl = 1e6;
    lcinfo.mbrDl = 1e6;
    lcinfo.gbrUl = 1e4;
    lcinfo.gbrDl = 1e4;
    // MacSapUserForRlc in the ComponentCarrierManager MacSapUser
    LteMacSapUser* MacSapUserForRlc = m_rrc->m_ccmRrcSapProvider->ConfigureSignalBearer(lcinfo, rlc->GetLteMacSapUser ()); 
    // Signal Channel are only on Primary Carrier
    m_rrc->m_cmacSapProvider.at (m_componentCarrierId)->AddLc (lcinfo, MacSapUserForRlc);
    m_rrc->m_ccmRrcSapProvider->AddLc (lcinfo, MacSapUserForRlc);
  }

  LteEnbRrcSapUser::SetupUeParameters ueParams;
  ueParams.srb0SapProvider = m_srb0->m_rlc->GetLteRlcSapProvider ();
  ueParams.srb1SapProvider = m_srb1->m_pdcp->GetLtePdcpSapProvider ();
  m_rrc->m_rrcSapUser->SetupUe (m_rnti, ueParams);

  // configure MAC (and scheduler)
  LteEnbCmacSapProvider::UeConfig req;
  req.m_rnti = m_rnti;
  req.m_transmissionMode = m_physicalConfigDedicated.antennaInfo.transmissionMode;

  // configure PHY
  for (uint16_t i = 0; i < m_rrc->m_numberOfComponentCarriers; i++)
    {
      m_rrc->m_cmacSapProvider.at (i)->UeUpdateConfigurationReq (req);
      m_rrc->m_cphySapProvider.at (i)->SetTransmissionMode (m_rnti, m_physicalConfigDedicated.antennaInfo.transmissionMode);
      m_rrc->m_cphySapProvider.at (i)->SetSrsConfigurationIndex (m_rnti, m_physicalConfigDedicated.soundingRsUlConfigDedicated.srsConfigIndex);
    }
  // schedule this UeManager instance to be deleted if the UE does not give any sign of life within a reasonable time
  Time maxConnectionDelay;
  switch (m_state)
    {
    case INITIAL_RANDOM_ACCESS:
      m_connectionRequestTimeout = Simulator::Schedule (m_rrc->m_connectionRequestTimeoutDuration,
                                                        &LteEnbRrc::ConnectionRequestTimeout,
                                                        m_rrc, m_rnti);
      break;

    case HANDOVER_JOINING:
      m_handoverJoiningTimeout = Simulator::Schedule (m_rrc->m_handoverJoiningTimeoutDuration,
                                                      &LteEnbRrc::HandoverJoiningTimeout,
                                                      m_rrc, m_rnti);
      break;

    case CONNECTION_LTE_COORDINATOR:
      NS_LOG_UNCOND ("Performing connection to LTE coordinator");
      break;
      
    default:
      NS_FATAL_ERROR ("unexpected state " << ToString (m_state));
      break;
    }
  m_caSupportConfigured =  false;
}


UeManager::~UeManager (void)
{
}

void
UeManager::DoDispose ()
{
  delete m_drbPdcpSapUser;
  // delete eventual X2-U TEIDs
  for (std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.begin ();
       it != m_drbMap.end ();
       ++it)
    {
      m_rrc->m_x2uTeidInfoMap.erase (it->second->m_gtpTeid);
    }

}

TypeId UeManager::GetTypeId (void)
{
  static TypeId  tid = TypeId ("ns3::UeManager")
    .SetParent<Object> ()
    .AddConstructor<UeManager> ()
    .AddAttribute ("DataRadioBearerMap", "List of UE DataRadioBearerInfo by DRBID.",
                   ObjectMapValue (),
                   MakeObjectMapAccessor (&UeManager::m_drbMap),
                   MakeObjectMapChecker<LteDataRadioBearerInfo> ())
    .AddAttribute ("Srb0", "SignalingRadioBearerInfo for SRB0",
                   PointerValue (),
                   MakePointerAccessor (&UeManager::m_srb0),
                   MakePointerChecker<LteSignalingRadioBearerInfo> ())
    .AddAttribute ("Srb1", "SignalingRadioBearerInfo for SRB1",
                   PointerValue (),
                   MakePointerAccessor (&UeManager::m_srb1),
                   MakePointerChecker<LteSignalingRadioBearerInfo> ())
    .AddAttribute ("C-RNTI",
                   "Cell Radio Network Temporary Identifier",
                   TypeId::ATTR_GET, // read-only attribute
                   UintegerValue (0), // unused, read-only attribute
                   MakeUintegerAccessor (&UeManager::m_rnti),
                   MakeUintegerChecker<uint16_t> ())
    .AddTraceSource ("StateTransition",
                     "fired upon every UE state transition seen by the "
                     "UeManager at the eNB RRC",
                     MakeTraceSourceAccessor (&UeManager::m_stateTransitionTrace),
                     "ns3::UeManager::StateTracedCallback")
    .AddTraceSource ("DrbCreated",
                     "trace fired after DRB is created",
                     MakeTraceSourceAccessor (&UeManager::m_drbCreatedTrace),
                     "ns3::UeManager::ImsiCidRntiLcIdTracedCallback")
  ;
  return tid;
}

void 
UeManager::SetSource (uint16_t sourceCellId, uint16_t sourceX2apId)
{
  m_sourceX2apId = sourceX2apId;
  m_sourceCellId = sourceCellId;
}

void 
UeManager::SetImsi (uint64_t imsi)
{
  m_imsi = imsi;
}

void
UeManager::InitialContextSetupRequest ()
{
  NS_LOG_FUNCTION (this << m_rnti);

  if (m_state == ATTACH_REQUEST)
    {
      SwitchToState (CONNECTED_NORMALLY);
    }
  else
    {
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
    }
}

void
UeManager::SetupDataRadioBearer (EpsBearer bearer, uint8_t bearerId, uint32_t gtpTeid, Ipv4Address transportLayerAddress)
{
  NS_LOG_FUNCTION (this << (uint32_t) m_rnti);

  Ptr<LteDataRadioBearerInfo> drbInfo = CreateObject<LteDataRadioBearerInfo> ();
  uint8_t drbid = AddDataRadioBearerInfo (drbInfo);
  uint8_t lcid = Drbid2Lcid (drbid); 
  uint8_t bid = Drbid2Bid (drbid);
  NS_ASSERT_MSG ( bearerId == 0 || bid == bearerId, "bearer ID mismatch (" << (uint32_t) bid << " != " << (uint32_t) bearerId << ", the assumption that ID are allocated in the same way by MME and RRC is not valid any more");
  drbInfo->m_epsBearer = bearer;
  drbInfo->m_epsBearerIdentity = bid;
  drbInfo->m_drbIdentity = drbid;
  drbInfo->m_logicalChannelIdentity = lcid;
  drbInfo->m_gtpTeid = gtpTeid;
  drbInfo->m_transportLayerAddress = transportLayerAddress;

  if (m_state == HANDOVER_JOINING)
    {
      // setup TEIDs for receiving data eventually forwarded over X2-U 
      LteEnbRrc::X2uTeidInfo x2uTeidInfo;
      x2uTeidInfo.rnti = m_rnti;
      x2uTeidInfo.drbid = drbid;
      std::pair<std::map<uint32_t, LteEnbRrc::X2uTeidInfo>::iterator, bool>
      ret = m_rrc->m_x2uTeidInfoMap.insert (std::pair<uint32_t, LteEnbRrc::X2uTeidInfo> (gtpTeid, x2uTeidInfo));
      NS_ASSERT_MSG (ret.second == true, "overwriting a pre-existing entry in m_x2uTeidInfoMap");
    }

  TypeId rlcTypeId = m_rrc->GetRlcType (bearer);

  ObjectFactory rlcObjectFactory;
  rlcObjectFactory.SetTypeId (rlcTypeId);
  Ptr<LteRlc> rlc = rlcObjectFactory.Create ()->GetObject<LteRlc> ();
  rlc->SetLteMacSapProvider (m_rrc->m_macSapProvider);
  rlc->SetRnti (m_rnti);

  drbInfo->m_rlc = rlc;

  rlc->SetLcId (lcid);

  // we need PDCP only for real RLC, i.e., RLC/UM or RLC/AM
  // if we are using RLC/SM we don't care of anything above RLC
  if (rlcTypeId != LteRlcSm::GetTypeId ())
    {
      Ptr<LtePdcp> pdcp = CreateObject<LtePdcp> ();
      pdcp->SetRnti (m_rnti);
      pdcp->SetLcId (lcid);
      pdcp->SetLtePdcpSapUser (m_drbPdcpSapUser);
      pdcp->SetLteRlcSapProvider (rlc->GetLteRlcSapProvider ());
      rlc->SetLteRlcSapUser (pdcp->GetLteRlcSapUser ());
      drbInfo->m_pdcp = pdcp;
    }

  m_drbCreatedTrace (m_imsi, m_rrc->ComponentCarrierToCellId (m_componentCarrierId), m_rnti, lcid);

  std::vector<LteCcmRrcSapProvider::LcsConfig> lcOnCcMapping = m_rrc->m_ccmRrcSapProvider->SetupDataRadioBearer (bearer, bearerId, m_rnti, lcid, m_rrc->GetLogicalChannelGroup (bearer), rlc->GetLteMacSapUser ());
  // LteEnbCmacSapProvider::LcInfo lcinfo;
  // lcinfo.rnti = m_rnti;
  // lcinfo.lcId = lcid;
  // lcinfo.lcGroup = m_rrc->GetLogicalChannelGroup (bearer);
  // lcinfo.qci = bearer.qci;
  // lcinfo.isGbr = bearer.IsGbr ();
  // lcinfo.mbrUl = bearer.gbrQosInfo.mbrUl;
  // lcinfo.mbrDl = bearer.gbrQosInfo.mbrDl;
  // lcinfo.gbrUl = bearer.gbrQosInfo.gbrUl;
  // lcinfo.gbrDl = bearer.gbrQosInfo.gbrDl;
  // use a for cycle to send the AddLc to the appropriate Mac Sap
  // if the sap is not initialized the appropriated method has to be called
  std::vector<LteCcmRrcSapProvider::LcsConfig>::iterator itLcOnCcMapping = lcOnCcMapping.begin ();
  NS_ASSERT_MSG (itLcOnCcMapping != lcOnCcMapping.end (), "Problem");
  for (itLcOnCcMapping = lcOnCcMapping.begin (); itLcOnCcMapping != lcOnCcMapping.end (); ++itLcOnCcMapping)
    {
      NS_LOG_DEBUG (this << " RNTI " << itLcOnCcMapping->lc.rnti << "Lcid " << (uint16_t) itLcOnCcMapping->lc.lcId << " lcGroup " << (uint16_t) itLcOnCcMapping->lc.lcGroup << " ComponentCarrierId " << itLcOnCcMapping->componentCarrierId);
      uint8_t index = itLcOnCcMapping->componentCarrierId;
      LteEnbCmacSapProvider::LcInfo lcinfo = itLcOnCcMapping->lc;
      LteMacSapUser *msu = itLcOnCcMapping->msu;
      m_rrc->m_cmacSapProvider.at (index)->AddLc (lcinfo, msu);
      m_rrc->m_ccmRrcSapProvider->AddLc (lcinfo, msu);
    }

  if (rlcTypeId == LteRlcAm::GetTypeId ())
    {
      drbInfo->m_rlcConfig.choice =  LteRrcSap::RlcConfig::AM;
    }
  else if (rlcTypeId == LteRlcTm::GetTypeId ())
    {
      drbInfo->m_rlcConfig.choice =  LteRrcSap::RlcConfig::TM;
    }
  else
    {
      drbInfo->m_rlcConfig.choice =  LteRrcSap::RlcConfig::UM_BI_DIRECTIONAL;
    }

  drbInfo->m_logicalChannelIdentity = lcid;
  drbInfo->m_logicalChannelConfig.priority =  m_rrc->GetLogicalChannelPriority (bearer);
  drbInfo->m_logicalChannelConfig.logicalChannelGroup = m_rrc->GetLogicalChannelGroup (bearer);
  if (bearer.IsGbr ())
    {
      drbInfo->m_logicalChannelConfig.prioritizedBitRateKbps = bearer.gbrQosInfo.gbrUl;
    }
  else
    {
      drbInfo->m_logicalChannelConfig.prioritizedBitRateKbps = 0;
    }
  drbInfo->m_logicalChannelConfig.bucketSizeDurationMs = 1000;

  ScheduleRrcConnectionReconfiguration ();
}

void
UeManager::RecordDataRadioBearersToBeStarted ()
{
  NS_LOG_FUNCTION (this << (uint32_t) m_rnti);
  for (std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.begin ();
       it != m_drbMap.end ();
       ++it)
    {
      m_drbsToBeStarted.push_back (it->first);
    }
}

void
UeManager::StartDataRadioBearers ()
{
  NS_LOG_FUNCTION (this << (uint32_t) m_rnti);
  for (std::list <uint8_t>::iterator drbIdIt = m_drbsToBeStarted.begin ();
       drbIdIt != m_drbsToBeStarted.end ();
       ++drbIdIt)
    {
      std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator drbIt = m_drbMap.find (*drbIdIt);
      NS_ASSERT (drbIt != m_drbMap.end ());
      drbIt->second->m_rlc->Initialize ();
      if (drbIt->second->m_pdcp)
        {
          drbIt->second->m_pdcp->Initialize ();
        }
    }
  m_drbsToBeStarted.clear ();
}


void
UeManager::ReleaseDataRadioBearer (uint8_t drbid)
{
  NS_LOG_FUNCTION (this << (uint32_t) m_rnti << (uint32_t) drbid);
  uint8_t lcid = Drbid2Lcid (drbid);
  std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.find (drbid);
  NS_ASSERT_MSG (it != m_drbMap.end (), "request to remove radio bearer with unknown drbid " << drbid);

  // first delete eventual X2-U TEIDs
  m_rrc->m_x2uTeidInfoMap.erase (it->second->m_gtpTeid);

  m_drbMap.erase (it);
  std::vector<uint8_t> ccToRelease = m_rrc->m_ccmRrcSapProvider->ReleaseDataRadioBearer (m_rnti, lcid);
  std::vector<uint8_t>::iterator itCcToRelease = ccToRelease.begin ();
  NS_ASSERT_MSG (itCcToRelease != ccToRelease.end (), "request to remove radio bearer with unknown drbid (ComponentCarrierManager)");
  for (itCcToRelease = ccToRelease.begin (); itCcToRelease != ccToRelease.end (); ++itCcToRelease)
    {
      m_rrc->m_cmacSapProvider.at (*itCcToRelease)->ReleaseLc (m_rnti, lcid);
    }
  LteRrcSap::RadioResourceConfigDedicated rrcd;
  rrcd.havePhysicalConfigDedicated = false;
  rrcd.drbToReleaseList.push_back (drbid);
  //populating RadioResourceConfigDedicated information element as per 3GPP TS 36.331 version 9.2.0
  rrcd.havePhysicalConfigDedicated = true;
  rrcd.physicalConfigDedicated = m_physicalConfigDedicated;
 
  //populating RRCConnectionReconfiguration message as per 3GPP TS 36.331 version 9.2.0 Release 9
  LteRrcSap::RrcConnectionReconfiguration msg;
  msg.haveMeasConfig = false;
  msg.haveMobilityControlInfo = false;
  msg.radioResourceConfigDedicated = rrcd;
  msg.haveRadioResourceConfigDedicated = true;
  // ToDo: Resend in any case this configuration
  // needs to be initialized
  msg.haveNonCriticalExtension = false;
  //RRC Connection Reconfiguration towards UE
  m_rrc->m_rrcSapUser->SendRrcConnectionReconfiguration (m_rnti, msg);
}

void
LteEnbRrc::DoSendReleaseDataRadioBearer (uint64_t imsi, uint16_t rnti, uint8_t bearerId)
{
  NS_LOG_FUNCTION (this << imsi << rnti << (uint16_t) bearerId);
  Ptr<UeManager> ueManager = GetUeManager (rnti);
  // Bearer de-activation towards UE
  ueManager->ReleaseDataRadioBearer (bearerId);
  // Bearer de-activation indication towards epc-enb application
  m_s1SapProvider->DoSendReleaseIndication (imsi,rnti,bearerId);
}

void
UeManager::RecvIdealUeContextRemoveRequest (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << m_rnti);

  //release the bearer info for the UE at SGW/PGW
  if (m_rrc->m_s1SapProvider != 0) //if EPC is enabled
    {
      for (const auto &it:m_drbMap)
        {
          NS_LOG_DEBUG ("Sending release of bearer id : " << (uint16_t) (it.first)
                        << "LCID : "
                        << (uint16_t) (it.second->m_logicalChannelIdentity));
          // Bearer de-activation indication towards epc-enb application
          m_rrc->m_s1SapProvider->DoSendReleaseIndication (GetImsi (), rnti, it.first);
        }
    }
}

void 
UeManager::ScheduleRrcConnectionReconfiguration ()
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case INITIAL_RANDOM_ACCESS:
    case CONNECTION_SETUP:
    case ATTACH_REQUEST:
    case CONNECTION_RECONFIGURATION:
    case CONNECTION_REESTABLISHMENT:
    case HANDOVER_PREPARATION:
    case HANDOVER_JOINING:
    case HANDOVER_LEAVING:
      // a previous reconfiguration still ongoing, we need to wait for it to be finished
      m_pendingRrcConnectionReconfiguration = true;
      break;

    case CONNECTED_NORMALLY:
      {
        m_pendingRrcConnectionReconfiguration = false;
        LteRrcSap::RrcConnectionReconfiguration msg = BuildRrcConnectionReconfiguration ();
        m_rrc->m_rrcSapUser->SendRrcConnectionReconfiguration (m_rnti, msg);
        RecordDataRadioBearersToBeStarted ();
        SwitchToState (CONNECTION_RECONFIGURATION);
      }
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void 
UeManager::PrepareHandover (uint16_t cellId)
{
  std::cout << "  * UeManager::PrepareHandover: to cell ID " << cellId << std::endl; 
  std::cout << "  ** m_state is " << m_state << " - " << ToString(m_state) << std::endl;
  NS_LOG_FUNCTION (this << cellId);
  switch (m_state)
    {
    case CONNECTED_NORMALLY:
      {
        m_targetCellId = cellId;
        EpcX2SapProvider::HandoverRequestParams params;
        params.oldEnbUeX2apId = m_rnti;
        params.cause          = EpcX2SapProvider::HandoverDesirableForRadioReason;
        params.sourceCellId   = m_rrc->ComponentCarrierToCellId (m_componentCarrierId);
        params.targetCellId   = cellId;
        params.mmeUeS1apId    = m_imsi;
        params.ueAggregateMaxBitRateDownlink = 200 * 1000;
        params.ueAggregateMaxBitRateUplink = 100 * 1000;
        params.bearers = GetErabList ();

        LteRrcSap::HandoverPreparationInfo hpi;
        hpi.asConfig.sourceUeIdentity = m_rnti;
        hpi.asConfig.sourceDlCarrierFreq = m_rrc->m_dlEarfcn;
        hpi.asConfig.sourceMeasConfig = m_rrc->m_ueMeasConfig;
        hpi.asConfig.sourceRadioResourceConfig = GetRadioResourceConfigForHandoverPreparationInfo ();
        hpi.asConfig.sourceMasterInformationBlock.dlBandwidth = m_rrc->m_dlBandwidth;
        hpi.asConfig.sourceMasterInformationBlock.systemFrameNumber = 0;
        hpi.asConfig.sourceSystemInformationBlockType1.cellAccessRelatedInfo.plmnIdentityInfo.plmnIdentity = m_rrc->m_sib1.at (m_componentCarrierId).cellAccessRelatedInfo.plmnIdentityInfo.plmnIdentity;
        hpi.asConfig.sourceSystemInformationBlockType1.cellAccessRelatedInfo.cellIdentity = m_rrc->ComponentCarrierToCellId (m_componentCarrierId);
        hpi.asConfig.sourceSystemInformationBlockType1.cellAccessRelatedInfo.csgIndication = m_rrc->m_sib1.at (m_componentCarrierId).cellAccessRelatedInfo.csgIndication;
        hpi.asConfig.sourceSystemInformationBlockType1.cellAccessRelatedInfo.csgIdentity = m_rrc->m_sib1.at (m_componentCarrierId).cellAccessRelatedInfo.csgIdentity;
        LteEnbCmacSapProvider::RachConfig rc = m_rrc->m_cmacSapProvider.at (m_componentCarrierId)->GetRachConfig ();
        hpi.asConfig.sourceSystemInformationBlockType2.radioResourceConfigCommon.rachConfigCommon.preambleInfo.numberOfRaPreambles = rc.numberOfRaPreambles;
        hpi.asConfig.sourceSystemInformationBlockType2.radioResourceConfigCommon.rachConfigCommon.raSupervisionInfo.preambleTransMax = rc.preambleTransMax;
        hpi.asConfig.sourceSystemInformationBlockType2.radioResourceConfigCommon.rachConfigCommon.raSupervisionInfo.raResponseWindowSize = rc.raResponseWindowSize;
        hpi.asConfig.sourceSystemInformationBlockType2.radioResourceConfigCommon.rachConfigCommon.txFailParam.connEstFailCount = rc.connEstFailCount;
        hpi.asConfig.sourceSystemInformationBlockType2.freqInfo.ulCarrierFreq = m_rrc->m_ulEarfcn;
        hpi.asConfig.sourceSystemInformationBlockType2.freqInfo.ulBandwidth = m_rrc->m_ulBandwidth;
        params.rrcContext = m_rrc->m_rrcSapUser->EncodeHandoverPreparationInformation (hpi);

        NS_LOG_LOGIC ("oldEnbUeX2apId = " << params.oldEnbUeX2apId);
        NS_LOG_LOGIC ("sourceCellId = " << params.sourceCellId);
        NS_LOG_LOGIC ("targetCellId = " << params.targetCellId);
        NS_LOG_LOGIC ("mmeUeS1apId = " << params.mmeUeS1apId);
        NS_LOG_LOGIC ("rrcContext   = " << params.rrcContext);

        std::cout << "  * UeManager::PrepareHandover: sending request over x2 sap provider " << std::endl; 
        m_rrc->m_x2SapProvider->SendHandoverRequest (params);
        std::cout << "  * UeManager::PrepareHandover: send request over x2 sap provider" << std::endl; 
        SwitchToState (HANDOVER_PREPARATION);
      }
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }

}

void 
UeManager::RecvHandoverRequestAck (EpcX2SapUser::HandoverRequestAckParams params)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT_MSG (params.notAdmittedBearers.empty (), "not admission of some bearers upon handover is not supported");
  NS_ASSERT_MSG (params.admittedBearers.size () == m_drbMap.size (), "not enough bearers in admittedBearers");

  // note: the Handover command from the target eNB to the source eNB
  // is expected to be sent transparently to the UE; however, here we
  // decode the message and eventually re-encode it. This way we can
  // support both a real RRC protocol implementation and an ideal one
  // without actual RRC protocol encoding. 

  Ptr<Packet> encodedHandoverCommand = params.rrcContext;
  LteRrcSap::RrcConnectionReconfiguration handoverCommand = m_rrc->m_rrcSapUser->DecodeHandoverCommand (encodedHandoverCommand);
  if (handoverCommand.haveNonCriticalExtension)
    {
      //Total number of component carriers = handoverCommand.nonCriticalExtension.sCellsToAddModList.size() + 1 (Primary carrier)
      if (handoverCommand.nonCriticalExtension.sCellsToAddModList.size() + 1 != m_rrc->m_numberOfComponentCarriers)
        {
          //Currently handover is only possible if source and target eNBs have equal number of component carriers
          NS_FATAL_ERROR ("The source and target eNBs have unequal number of component carriers. Target eNB CCs = "
                           << handoverCommand.nonCriticalExtension.sCellsToAddModList.size() + 1
                           << " Source eNB CCs = " << m_rrc->m_numberOfComponentCarriers);
        }
    }
  m_rrc->m_rrcSapUser->SendRrcConnectionReconfiguration (m_rnti, handoverCommand);
  SwitchToState (HANDOVER_LEAVING);
  m_handoverLeavingTimeout = Simulator::Schedule (m_rrc->m_handoverLeavingTimeoutDuration, 
                                                  &LteEnbRrc::HandoverLeavingTimeout, 
                                                  m_rrc, m_rnti);
  NS_ASSERT (handoverCommand.haveMobilityControlInfo);
  m_rrc->m_handoverStartTrace (m_imsi, m_rrc->ComponentCarrierToCellId (m_componentCarrierId), m_rnti, handoverCommand.mobilityControlInfo.targetPhysCellId);

  //Set the target cell ID and the RNTI so that handover cancel message can be sent if required
  m_targetX2apId = params.newEnbUeX2apId;
  m_targetCellId = params.targetCellId;

  EpcX2SapProvider::SnStatusTransferParams sst;
  sst.oldEnbUeX2apId = params.oldEnbUeX2apId;
  sst.newEnbUeX2apId = params.newEnbUeX2apId;
  sst.sourceCellId = params.sourceCellId;
  sst.targetCellId = params.targetCellId;
  for ( std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator drbIt = m_drbMap.begin ();
        drbIt != m_drbMap.end ();
        ++drbIt)
    {
      // SN status transfer is only for AM RLC
      if (0 != drbIt->second->m_rlc->GetObject<LteRlcAm> ())
        {
          LtePdcp::Status status = drbIt->second->m_pdcp->GetStatus ();
          EpcX2Sap::ErabsSubjectToStatusTransferItem i;
          i.dlPdcpSn = status.txSn;
          i.ulPdcpSn = status.rxSn;
          sst.erabsSubjectToStatusTransferList.push_back (i);
        }
    }
  m_rrc->m_x2SapProvider->SendSnStatusTransfer (sst);
}


LteRrcSap::RadioResourceConfigDedicated
UeManager::GetRadioResourceConfigForHandoverPreparationInfo ()
{
  NS_LOG_FUNCTION (this);
  return BuildRadioResourceConfigDedicated ();
}

LteRrcSap::RrcConnectionReconfiguration 
UeManager::GetRrcConnectionReconfigurationForHandover ()
{
  NS_LOG_FUNCTION (this);
  return BuildRrcConnectionReconfiguration ();
}

void
UeManager::SendPacket (uint8_t bid, Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this << p << (uint16_t) bid);
  LtePdcpSapProvider::TransmitPdcpSduParameters params;
  params.pdcpSdu = p;
  params.rnti = m_rnti;
  params.lcid = Bid2Lcid (bid);
  uint8_t drbid = Bid2Drbid (bid);
  // Transmit PDCP sdu only if DRB ID found in drbMap
  std::map<uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.find (drbid);
  if (it != m_drbMap.end ())
    {
      Ptr<LteDataRadioBearerInfo> bearerInfo = GetDataRadioBearerInfo (drbid);
      if (bearerInfo != NULL)
        {
          LtePdcpSapProvider* pdcpSapProvider = bearerInfo->m_pdcp->GetLtePdcpSapProvider ();
          pdcpSapProvider->TransmitPdcpSdu (params);
        }
    }
}

void
UeManager::SendData (uint8_t bid, Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this << p << (uint16_t) bid);
  switch (m_state)
    {
    case INITIAL_RANDOM_ACCESS:
    case CONNECTION_SETUP:
      NS_LOG_WARN ("not connected, discarding packet");
      return;
      break;

    case CONNECTED_NORMALLY:
    case CONNECTION_RECONFIGURATION:
    case CONNECTION_REESTABLISHMENT:
    case HANDOVER_PREPARATION:
    case HANDOVER_JOINING:
    case HANDOVER_PATH_SWITCH:
      {
        NS_LOG_LOGIC ("queueing data on PDCP for transmission over the air");
        SendPacket (bid, p);
      }
      break;

    case HANDOVER_LEAVING:
      {
        NS_LOG_LOGIC ("forwarding data to target eNB over X2-U");
        uint8_t drbid = Bid2Drbid (bid);
        EpcX2Sap::UeDataParams params;
        params.sourceCellId = m_rrc->ComponentCarrierToCellId (m_componentCarrierId);
        params.targetCellId = m_targetCellId;
        params.gtpTeid = GetDataRadioBearerInfo (drbid)->m_gtpTeid;
        params.ueData = p;
        m_rrc->m_x2SapProvider->SendUeData (params);
      }
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

std::vector<EpcX2Sap::ErabToBeSetupItem>
UeManager::GetErabList ()
{
  NS_LOG_FUNCTION (this);
  std::vector<EpcX2Sap::ErabToBeSetupItem> ret;
  for (std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it =  m_drbMap.begin ();
       it != m_drbMap.end ();
       ++it)
    {
      EpcX2Sap::ErabToBeSetupItem etbsi;
      etbsi.erabId = it->second->m_epsBearerIdentity;
      etbsi.erabLevelQosParameters = it->second->m_epsBearer;
      etbsi.dlForwarding = false;
      etbsi.transportLayerAddress = it->second->m_transportLayerAddress;
      etbsi.gtpTeid = it->second->m_gtpTeid;
      ret.push_back (etbsi);
    }
  return ret;
}

void
UeManager::SendUeContextRelease ()
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case HANDOVER_PATH_SWITCH:
      NS_LOG_INFO ("Send UE CONTEXT RELEASE from target eNB to source eNB");
      EpcX2SapProvider::UeContextReleaseParams ueCtxReleaseParams;
      ueCtxReleaseParams.oldEnbUeX2apId = m_sourceX2apId;
      ueCtxReleaseParams.newEnbUeX2apId = m_rnti;
      ueCtxReleaseParams.sourceCellId = m_sourceCellId;
      ueCtxReleaseParams.targetCellId = m_targetCellId;
      m_rrc->m_x2SapProvider->SendUeContextRelease (ueCtxReleaseParams);
      SwitchToState (CONNECTED_NORMALLY);
      m_rrc->m_handoverEndOkTrace (m_imsi, m_rrc->ComponentCarrierToCellId (m_componentCarrierId), m_rnti);
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void 
UeManager::RecvHandoverPreparationFailure (uint16_t cellId)
{
  NS_LOG_FUNCTION (this << cellId);
  switch (m_state)
    {
    case HANDOVER_PREPARATION:
      NS_ASSERT (cellId == m_targetCellId);
      NS_LOG_INFO ("target eNB sent HO preparation failure, aborting HO");
      SwitchToState (CONNECTED_NORMALLY);
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void 
UeManager::RecvSnStatusTransfer (EpcX2SapUser::SnStatusTransferParams params)
{
  NS_LOG_FUNCTION (this);
  for (std::vector<EpcX2Sap::ErabsSubjectToStatusTransferItem>::iterator erabIt 
         = params.erabsSubjectToStatusTransferList.begin ();
       erabIt != params.erabsSubjectToStatusTransferList.end ();
       ++erabIt)
    {
      // LtePdcp::Status status;
      // status.txSn = erabIt->dlPdcpSn;
      // status.rxSn = erabIt->ulPdcpSn;
      // uint8_t drbId = Bid2Drbid (erabIt->erabId);
      // std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator drbIt = m_drbMap.find (drbId);
      // NS_ASSERT_MSG (drbIt != m_drbMap.end (), "could not find DRBID " << (uint32_t) drbId);
      // drbIt->second->m_pdcp->SetStatus (status);
    }
}

void 
UeManager::RecvUeContextRelease (EpcX2SapUser::UeContextReleaseParams params)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (m_state == HANDOVER_LEAVING, "method unexpected in state " << ToString (m_state));
  m_handoverLeavingTimeout.Cancel ();
}


// methods forwarded from RRC SAP

void 
UeManager::CompleteSetupUe (LteEnbRrcSapProvider::CompleteSetupUeParameters params)
{
  NS_LOG_FUNCTION (this);
  m_srb0->m_rlc->SetLteRlcSapUser (params.srb0SapUser);
  m_srb1->m_pdcp->SetLtePdcpSapUser (params.srb1SapUser);
  std::cout << "UeManager::CompleteSetupUe: completed setup for UE" << std::endl;
}

void
UeManager::RecvRrcConnectionRequest (LteRrcSap::RrcConnectionRequest msg)
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case INITIAL_RANDOM_ACCESS:
      {
        m_connectionRequestTimeout.Cancel ();

        if (m_rrc->m_admitRrcConnectionRequest == true)
          {
            m_imsi = msg.ueIdentity;
            //*********************************************************************************
            m_rrc->RegisterImsiToRnti(m_imsi, m_rnti);
            //*********************************************************************************
            // send RRC CONNECTION SETUP to UE
            LteRrcSap::RrcConnectionSetup msg2;
            msg2.rrcTransactionIdentifier = GetNewRrcTransactionIdentifier ();
            msg2.radioResourceConfigDedicated = BuildRadioResourceConfigDedicated ();
            m_rrc->m_rrcSapUser->SendRrcConnectionSetup (m_rnti, msg2);

            RecordDataRadioBearersToBeStarted ();
            m_connectionSetupTimeout = Simulator::Schedule (
                m_rrc->m_connectionSetupTimeoutDuration,
                &LteEnbRrc::ConnectionSetupTimeout, m_rrc, m_rnti);
            SwitchToState (CONNECTION_SETUP);
          }
        else
          {
            NS_LOG_INFO ("rejecting connection request for RNTI " << m_rnti);

            // send RRC CONNECTION REJECT to UE
            LteRrcSap::RrcConnectionReject rejectMsg;
            rejectMsg.waitTime = 3;
            m_rrc->m_rrcSapUser->SendRrcConnectionReject (m_rnti, rejectMsg);

            m_connectionRejectedTimeout = Simulator::Schedule (
                m_rrc->m_connectionRejectedTimeoutDuration,
                &LteEnbRrc::ConnectionRejectedTimeout, m_rrc, m_rnti);
            SwitchToState (CONNECTION_REJECTED);
          }
      }
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void
UeManager::RecvRrcConnectionSetupCompleted (LteRrcSap::RrcConnectionSetupCompleted msg)
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case CONNECTION_SETUP:
//******************************************************************************************
      for (uint8_t i = 0; i < m_rrc->m_numberOfComponentCarriers; i++)
      {
        m_rrc->m_cmacSapProvider.at(i)->SetRAProcessFlag(false);
      }
//******************************************************************************************
      m_connectionSetupTimeout.Cancel ();
      if ( m_caSupportConfigured == false && m_rrc->m_numberOfComponentCarriers > 1)
        {
          m_pendingRrcConnectionReconfiguration = true; // Force Reconfiguration
          m_pendingStartDataRadioBearers = true;
        }
      if (m_rrc->m_s1SapProvider != 0)
        {
          m_rrc->m_s1SapProvider->InitialUeMessage (m_imsi, m_rnti);
          SwitchToState (ATTACH_REQUEST);
        }
      else
        {
          SwitchToState (CONNECTED_NORMALLY);
        }
      m_rrc->m_connectionEstablishedTrace (m_imsi, m_rrc->ComponentCarrierToCellId (m_componentCarrierId), m_rnti);
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void
UeManager::RecvRrcConnectionReconfigurationCompleted (LteRrcSap::RrcConnectionReconfigurationCompleted msg)
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case CONNECTION_RECONFIGURATION:
      StartDataRadioBearers ();
      if (m_needPhyMacConfiguration)
        {
          // configure MAC (and scheduler)
          LteEnbCmacSapProvider::UeConfig req;
          req.m_rnti = m_rnti;
          req.m_transmissionMode = m_physicalConfigDedicated.antennaInfo.transmissionMode;
          for (uint8_t i = 0; i < m_rrc->m_numberOfComponentCarriers; i++)
            {
              m_rrc->m_cmacSapProvider.at (i)->UeUpdateConfigurationReq (req);

              // configure PHY
              m_rrc->m_cphySapProvider.at (i)->SetTransmissionMode (req.m_rnti, req.m_transmissionMode);
              double paDouble = LteRrcSap::ConvertPdschConfigDedicated2Double (m_physicalConfigDedicated.pdschConfigDedicated);
              m_rrc->m_cphySapProvider.at (i)->SetPa (m_rnti, paDouble);
            }

          m_needPhyMacConfiguration = false;
        }
      SwitchToState (CONNECTED_NORMALLY);
//******************************************************************************************
      for (uint8_t i = 0; i < m_rrc->m_numberOfComponentCarriers; i++)
      {
        m_rrc->m_cmacSapProvider.at(i)->SetRAProcessFlag(false);
      }

      m_rrc->UpdateCSIRLMResources(m_imsi);
//******************************************************************************************
      m_rrc->m_connectionReconfigurationTrace (m_imsi, m_rrc->ComponentCarrierToCellId (m_componentCarrierId), m_rnti);
      if(m_rrc->m_lteCellId != 0)
      {
        EpcX2Sap::InformLteCoordinatorParams HOparams;
        HOparams.sourceCellId = m_rrc->GetCellId();
        HOparams.targetCellId = m_rrc->m_lteCellId;
        HOparams.ueImsi = m_imsi;
        HOparams.ueRnti = m_rnti;
        HOparams.hoCompletedFlag = true;
        m_rrc->m_x2SapProvider->InformLteCoordinator(HOparams);
      }      
    // This case is added to NS-3 in order to handle bearer de-activation scenario for CONNECTED state UE
    case CONNECTED_NORMALLY:
      NS_LOG_INFO ("ignoring RecvRrcConnectionReconfigurationCompleted in state " << ToString (m_state));
      break;

    case HANDOVER_LEAVING:
      NS_LOG_INFO ("ignoring RecvRrcConnectionReconfigurationCompleted in state " << ToString (m_state));
      break;

    case HANDOVER_JOINING:
      {
        m_handoverJoiningTimeout.Cancel ();
        
        NS_LOG_INFO ("Send PATH SWITCH REQUEST to the MME");
        if(m_rrc->m_lteCellId != 0)
        {
          EpcX2Sap::InformLteCoordinatorParams HOparams;
          HOparams.sourceCellId = m_rrc->GetCellId();
          HOparams.targetCellId = m_rrc->m_lteCellId; // handover params are sent to CO
          HOparams.ueImsi = m_imsi;
          HOparams.ueRnti = m_rnti;
          HOparams.hoCompletedFlag = false;
          m_rrc->m_x2SapProvider->InformLteCoordinator(HOparams);
        }
        EpcEnbS1SapProvider::PathSwitchRequestParameters params;
        params.rnti = m_rnti;
        params.cellId = m_rrc->ComponentCarrierToCellId (m_componentCarrierId);
        params.mmeUeS1Id = m_imsi;
        SwitchToState (HANDOVER_PATH_SWITCH);
        for (std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it =  m_drbMap.begin ();
             it != m_drbMap.end ();
             ++it)
          {
            EpcEnbS1SapProvider::BearerToBeSwitched b;
            b.epsBearerId = it->second->m_epsBearerIdentity;
            b.teid =  it->second->m_gtpTeid;
            params.bearersToBeSwitched.push_back (b);
          }
        m_rrc->m_s1SapProvider->PathSwitchRequest (params);
      }
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void 
UeManager::RecvRrcConnectionReestablishmentRequest (LteRrcSap::RrcConnectionReestablishmentRequest msg)
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case CONNECTED_NORMALLY:
      break;

    case HANDOVER_LEAVING:
      m_handoverLeavingTimeout.Cancel ();
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }

  LteRrcSap::RrcConnectionReestablishment msg2;
  msg2.rrcTransactionIdentifier = GetNewRrcTransactionIdentifier ();
  msg2.radioResourceConfigDedicated = BuildRadioResourceConfigDedicated ();
  m_rrc->m_rrcSapUser->SendRrcConnectionReestablishment (m_rnti, msg2);
  SwitchToState (CONNECTION_REESTABLISHMENT);
}

void 
UeManager::RecvRrcConnectionReestablishmentComplete (LteRrcSap::RrcConnectionReestablishmentComplete msg)
{
  NS_LOG_FUNCTION (this);
  SwitchToState (CONNECTED_NORMALLY);
}

void 
UeManager::RecvMeasurementReport (LteRrcSap::MeasurementReport msg)
{
  uint8_t measId = msg.measResults.measId;
  NS_LOG_FUNCTION (this << (uint16_t) measId);
  NS_LOG_LOGIC ("measId " << (uint16_t) measId
                          << " haveMeasResultNeighCells " << msg.measResults.haveMeasResultNeighCells
                          << " measResultListEutra " << msg.measResults.measResultListEutra.size ()
                          << " haveScellsMeas " << msg.measResults.haveScellsMeas
                          << " measScellResultList " << msg.measResults.measScellResultList.measResultScell.size ());
  NS_LOG_LOGIC ("serving cellId " << m_rrc->ComponentCarrierToCellId (m_componentCarrierId)
                                  << " RSRP " << (uint16_t) msg.measResults.rsrpResult
                                  << " RSRQ " << (uint16_t) msg.measResults.rsrqResult);

  for (std::list <LteRrcSap::MeasResultEutra>::iterator it = msg.measResults.measResultListEutra.begin ();
       it != msg.measResults.measResultListEutra.end ();
       ++it)
    {
      NS_LOG_LOGIC ("neighbour cellId " << it->physCellId
                                        << " RSRP " << (it->haveRsrpResult ? (uint16_t) it->rsrpResult : 255)
                                        << " RSRQ " << (it->haveRsrqResult ? (uint16_t) it->rsrqResult : 255));
    }

  if ((m_rrc->m_handoverManagementSapProvider != 0)
      && (m_rrc->m_handoverMeasIds.find (measId) != m_rrc->m_handoverMeasIds.end ()))
    {
      // this measurement was requested by the handover algorithm
      m_rrc->m_handoverManagementSapProvider->ReportUeMeas (m_rnti,
                                                            msg.measResults);
    }

  if ((m_rrc->m_ccmRrcSapProvider != 0)
      && (m_rrc->m_componentCarrierMeasIds.find (measId) != m_rrc->m_componentCarrierMeasIds.end ()))
    {
      // this measurement was requested by the handover algorithm
      m_rrc->m_ccmRrcSapProvider->ReportUeMeas (m_rnti,
                                                msg.measResults);
    }

  if ((m_rrc->m_anrSapProvider != 0)
      && (m_rrc->m_anrMeasIds.find (measId) != m_rrc->m_anrMeasIds.end ()))
    {
      // this measurement was requested by the ANR function
      m_rrc->m_anrSapProvider->ReportUeMeas (msg.measResults);
    }

  if ((m_rrc->m_ffrRrcSapProvider.size () > 0)
      && (m_rrc->m_ffrMeasIds.find (measId) != m_rrc->m_ffrMeasIds.end ()))
    {
      // this measurement was requested by the FFR function
      m_rrc->m_ffrRrcSapProvider.at (0)->ReportUeMeas (m_rnti, msg.measResults);
    }
  if (msg.measResults.haveScellsMeas == true)
    {
      for (std::list <LteRrcSap::MeasResultScell>::iterator it = msg.measResults.measScellResultList.measResultScell.begin ();
           it != msg.measResults.measScellResultList.measResultScell.end ();
           ++it)
        {
          m_rrc->m_ffrRrcSapProvider.at (it->servFreqId)->ReportUeMeas (m_rnti, msg.measResults);
          /// ToDo: implement on Ffr algorithm the code to properly parsing the new measResults message format
          /// alternatively it is needed to 'repack' properly the measResults message before sending to Ffr 
        }
    }

  ///Report any measurements to ComponentCarrierManager, so it can react to any change or activate the SCC
  m_rrc->m_ccmRrcSapProvider->ReportUeMeas (m_rnti, msg.measResults);
  // fire a trace source
  m_rrc->m_recvMeasurementReportTrace (m_imsi, m_rrc->ComponentCarrierToCellId (m_componentCarrierId), m_rnti, msg);

} // end of UeManager::RecvMeasurementReport

void
UeManager::RecvPositionReport(LteRrcSap::PositionReport msg)
{
  std::cout << "UeManager::RecvPositionReport: recevied position report. Passing it to the tracing function" << std::endl;
  std::cout << "UEManager at the eNB got the following position:" << std::endl;
  std::cout << "  x: " << msg.uePosition.x << std::endl;
  std::cout << "  y: " << msg.uePosition.y << std::endl;
  //test
  m_rrc->m_recvPositionReportTrace (m_imsi, m_rrc->ComponentCarrierToCellId (m_componentCarrierId), m_rnti, msg);

  // Query REM for the received UE position
  // Instruct the eNB/NR to check if we are using the "expected" beam. If not, then re-select the correct
  // beam without sweeping by directly commanding the correct BPL
  

}


// methods forwarded from CMAC SAP

void
UeManager::CmacUeConfigUpdateInd (LteEnbCmacSapUser::UeConfig cmacParams)
{
  NS_LOG_FUNCTION (this << m_rnti);
  // at this stage used only by the scheduler for updating txMode

  m_physicalConfigDedicated.antennaInfo.transmissionMode = cmacParams.m_transmissionMode;

  m_needPhyMacConfiguration = true;

  // reconfigure the UE RRC
  ScheduleRrcConnectionReconfiguration ();
}


// methods forwarded from PDCP SAP

void
UeManager::DoReceivePdcpSdu (LtePdcpSapUser::ReceivePdcpSduParameters params)
{
  NS_LOG_FUNCTION (this);
  if (params.lcid > 2)
    {
      // data radio bearer
      EpsBearerTag tag;
      tag.SetRnti (params.rnti);
      tag.SetBid (Lcid2Bid (params.lcid));
      params.pdcpSdu->AddPacketTag (tag);
      m_rrc->m_forwardUpCallback (params.pdcpSdu);
    }
}


uint16_t
UeManager::GetRnti (void) const
{
  return m_rnti;
}

uint64_t
UeManager::GetImsi (void) const
{
  return m_imsi;
}

uint8_t
UeManager::GetComponentCarrierId () const
{
  return m_componentCarrierId;
}

uint16_t
UeManager::GetSrsConfigurationIndex (void) const
{
  return m_physicalConfigDedicated.soundingRsUlConfigDedicated.srsConfigIndex;
}

void
UeManager::SetSrsConfigurationIndex (uint16_t srsConfIndex)
{
  NS_LOG_FUNCTION (this);
  m_physicalConfigDedicated.soundingRsUlConfigDedicated.srsConfigIndex = srsConfIndex;
  for (uint16_t i = 0; i < m_rrc->m_numberOfComponentCarriers; i++)
    {
      m_rrc->m_cphySapProvider.at (i)->SetSrsConfigurationIndex (m_rnti, srsConfIndex);
    }
  switch (m_state)
    {
    case INITIAL_RANDOM_ACCESS:
      // do nothing, srs conf index will be correctly enforced upon
      // RRC connection establishment
      break;

    default:
      ScheduleRrcConnectionReconfiguration ();
      break;
    }
}

UeManager::State
UeManager::GetState (void) const
{
  return m_state;
}

void
UeManager::SetPdschConfigDedicated (LteRrcSap::PdschConfigDedicated pdschConfigDedicated)
{
  NS_LOG_FUNCTION (this);
  m_physicalConfigDedicated.pdschConfigDedicated = pdschConfigDedicated;

  m_needPhyMacConfiguration = true;

  // reconfigure the UE RRC
  ScheduleRrcConnectionReconfiguration ();
}

void
UeManager::CancelPendingEvents ()
{
  NS_LOG_FUNCTION (this);
  m_connectionRequestTimeout.Cancel ();
  m_connectionRejectedTimeout.Cancel ();
  m_connectionSetupTimeout.Cancel ();
  m_handoverJoiningTimeout.Cancel ();
  m_handoverLeavingTimeout.Cancel ();
}

uint8_t
UeManager::AddDataRadioBearerInfo (Ptr<LteDataRadioBearerInfo> drbInfo)
{
  NS_LOG_FUNCTION (this);
  const uint8_t MAX_DRB_ID = 32;
  for (int drbid = (m_lastAllocatedDrbid + 1) % MAX_DRB_ID; 
       drbid != m_lastAllocatedDrbid; 
       drbid = (drbid + 1) % MAX_DRB_ID)
    {
      if (drbid != 0) // 0 is not allowed
        {
          if (m_drbMap.find (drbid) == m_drbMap.end ())
            {
              m_drbMap.insert (std::pair<uint8_t, Ptr<LteDataRadioBearerInfo> > (drbid, drbInfo));
              drbInfo->m_drbIdentity = drbid;
              m_lastAllocatedDrbid = drbid;
              return drbid;
            }
        }
    }
  NS_FATAL_ERROR ("no more data radio bearer ids available");
  return 0;
}

Ptr<LteDataRadioBearerInfo>
UeManager::GetDataRadioBearerInfo (uint8_t drbid)
{
  NS_LOG_FUNCTION (this << (uint32_t) drbid);
  NS_ASSERT (0 != drbid);
  std::map<uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.find (drbid);
  NS_ABORT_IF (it == m_drbMap.end ());
  return it->second;
}


void
UeManager::RemoveDataRadioBearerInfo (uint8_t drbid)
{
  NS_LOG_FUNCTION (this << (uint32_t) drbid);
  std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.find (drbid);
  NS_ASSERT_MSG (it != m_drbMap.end (), "request to remove radio bearer with unknown drbid " << drbid);
  m_drbMap.erase (it);
}


LteRrcSap::RrcConnectionReconfiguration
UeManager::BuildRrcConnectionReconfiguration ()
{
  NS_LOG_FUNCTION (this);
  LteRrcSap::RrcConnectionReconfiguration msg;
  msg.rrcTransactionIdentifier = GetNewRrcTransactionIdentifier ();
  msg.haveRadioResourceConfigDedicated = true;
  msg.radioResourceConfigDedicated = BuildRadioResourceConfigDedicated ();
  msg.haveMobilityControlInfo = false;
  msg.haveMeasConfig = true;
  msg.measConfig = m_rrc->m_ueMeasConfig;
  if ( m_caSupportConfigured == false && m_rrc->m_numberOfComponentCarriers > 1)
    {
      m_caSupportConfigured = true;
      NS_LOG_FUNCTION ( this << "CA not configured. Configure now!" );
      msg.haveNonCriticalExtension = true;
      msg.nonCriticalExtension = BuildNonCriticalExtentionConfigurationCa ();
      NS_LOG_FUNCTION ( this << " haveNonCriticalExtension " << msg.haveNonCriticalExtension );
    }
  else
    {
      msg.haveNonCriticalExtension = false;
    }

  return msg;
}

LteRrcSap::RadioResourceConfigDedicated
UeManager::BuildRadioResourceConfigDedicated ()
{
  NS_LOG_FUNCTION (this);
  LteRrcSap::RadioResourceConfigDedicated rrcd;

  if (m_srb1 != 0)
    {
      LteRrcSap::SrbToAddMod stam;
      stam.srbIdentity = m_srb1->m_srbIdentity;
      stam.logicalChannelConfig = m_srb1->m_logicalChannelConfig;
      rrcd.srbToAddModList.push_back (stam);
    }

  for (std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.begin ();
       it != m_drbMap.end ();
       ++it)
    {
      LteRrcSap::DrbToAddMod dtam;
      dtam.epsBearerIdentity = it->second->m_epsBearerIdentity;
      dtam.drbIdentity = it->second->m_drbIdentity;
      dtam.rlcConfig = it->second->m_rlcConfig;
      dtam.logicalChannelIdentity = it->second->m_logicalChannelIdentity;
      dtam.logicalChannelConfig = it->second->m_logicalChannelConfig;
      rrcd.drbToAddModList.push_back (dtam);
    }

  rrcd.havePhysicalConfigDedicated = true;
  rrcd.physicalConfigDedicated = m_physicalConfigDedicated;
  rrcd.csiConfigParams.initialCSIRSResource = m_rrc->GetCSIRSResourceVector (GetImsi ());
  rrcd.csiConfigParams.csiRSOfset = m_rrc->m_csiOffsetInSlots;
  rrcd.csiConfigParams.ciRSPeriodicity = m_rrc->m_csiPeriodicityInSlots;
  uint16_t framesPerSSBUrst = (uint16_t)round((double)m_rrc->m_ssbPeriodicity.GetMilliSeconds () / 10.0);
  rrcd.csiConfigParams.noOfCSIRSResourcesPerSSBurst = framesPerSSBUrst * m_rrc->m_maxCSIperFrame;
  return rrcd;
}

uint8_t 
UeManager::GetNewRrcTransactionIdentifier ()
{
  NS_LOG_FUNCTION (this);
  ++m_lastRrcTransactionIdentifier;
  m_lastRrcTransactionIdentifier %= 4;
  return m_lastRrcTransactionIdentifier;
}

uint8_t 
UeManager::Lcid2Drbid (uint8_t lcid)
{
  NS_ASSERT (lcid > 2);
  return lcid - 2;
}

uint8_t 
UeManager::Drbid2Lcid (uint8_t drbid)
{
  return drbid + 2;
}
uint8_t 
UeManager::Lcid2Bid (uint8_t lcid)
{
  NS_ASSERT (lcid > 2);
  return lcid - 2;
}

uint8_t 
UeManager::Bid2Lcid (uint8_t bid)
{
  return bid + 2;
}

uint8_t 
UeManager::Drbid2Bid (uint8_t drbid)
{
  return drbid;
}

uint8_t 
UeManager::Bid2Drbid (uint8_t bid)
{
  return bid;
}


void 
UeManager::SwitchToState (State newState)
{
  NS_LOG_FUNCTION (this << ToString (newState));
  State oldState = m_state;
  m_state = newState;
  NS_LOG_INFO (this << " IMSI " << m_imsi << " RNTI " << m_rnti << " UeManager "
                    << ToString (oldState) << " --> " << ToString (newState));
  m_stateTransitionTrace (m_imsi, m_rrc->ComponentCarrierToCellId (m_componentCarrierId), m_rnti, oldState, newState);

  switch (newState)
    {
    case INITIAL_RANDOM_ACCESS:
    case HANDOVER_JOINING:
      NS_FATAL_ERROR ("cannot switch to an initial state");
      break;

    case CONNECTION_SETUP:
      break;

    case ATTACH_REQUEST:
      break;

    case CONNECTED_NORMALLY:
      {
        if (m_pendingRrcConnectionReconfiguration == true)
          {
            ScheduleRrcConnectionReconfiguration ();
          }
        if (m_pendingStartDataRadioBearers == true && m_caSupportConfigured == true)
          {
            StartDataRadioBearers ();
          }
      }
      break;

    case CONNECTION_RECONFIGURATION:
      break;

    case CONNECTION_REESTABLISHMENT:
      break;

    case HANDOVER_LEAVING:
      break;

    default:
      break;
    }
}

LteRrcSap::NonCriticalExtensionConfiguration
UeManager::BuildNonCriticalExtentionConfigurationCa ()
{
  NS_LOG_FUNCTION ( this );
  LteRrcSap::NonCriticalExtensionConfiguration ncec;
  
  //  LteRrcSap::SCellToAddMod scell;
  std::list<LteRrcSap::SCellToAddMod> SccCon;

  // sCellToReleaseList is always empty since no Scc is released

  for (auto &it: m_rrc->m_componentCarrierPhyConf)
    {
      uint8_t ccId = it.first;

      if (ccId == m_componentCarrierId)
        {
          // Skip primary CC.
          continue;
        }
      else if (ccId < m_componentCarrierId)
        {
          // Shift all IDs below PCC forward so PCC can use CC ID 1.
          ccId++;
        }

      Ptr<ComponentCarrierBaseStation> eNbCcm = it.second;
      LteRrcSap::SCellToAddMod component;
      component.sCellIndex = ccId;
      component.cellIdentification.physCellId = eNbCcm->GetCellId ();
      component.cellIdentification.dlCarrierFreq = eNbCcm->GetDlEarfcn ();
      component.radioResourceConfigCommonSCell.haveNonUlConfiguration = true;
      component.radioResourceConfigCommonSCell.nonUlConfiguration.dlBandwidth = eNbCcm->GetDlBandwidth ();
      component.radioResourceConfigCommonSCell.nonUlConfiguration.antennaInfoCommon.antennaPortsCount = 0;
      component.radioResourceConfigCommonSCell.nonUlConfiguration.pdschConfigCommon.referenceSignalPower = m_rrc->m_cphySapProvider.at (0)->GetReferenceSignalPower ();
      component.radioResourceConfigCommonSCell.nonUlConfiguration.pdschConfigCommon.pb = 0;
      component.radioResourceConfigCommonSCell.haveUlConfiguration = true;
      component.radioResourceConfigCommonSCell.ulConfiguration.ulFreqInfo.ulCarrierFreq = eNbCcm->GetUlEarfcn ();
      component.radioResourceConfigCommonSCell.ulConfiguration.ulFreqInfo.ulBandwidth = eNbCcm->GetUlBandwidth ();
      component.radioResourceConfigCommonSCell.ulConfiguration.ulPowerControlCommonSCell.alpha = 0;
      //component.radioResourceConfigCommonSCell.ulConfiguration.soundingRsUlConfigCommon.type = LteRrcSap::SoundingRsUlConfigDedicated::SETUP;
      component.radioResourceConfigCommonSCell.ulConfiguration.soundingRsUlConfigCommon.srsBandwidthConfig = 0;
      component.radioResourceConfigCommonSCell.ulConfiguration.soundingRsUlConfigCommon.srsSubframeConfig = 0;
      component.radioResourceConfigCommonSCell.ulConfiguration.prachConfigSCell.index = 0;
    
      if (true)
        {
          component.haveRadioResourceConfigDedicatedSCell = true;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.haveNonUlConfiguration = true;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.haveAntennaInfoDedicated = true;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.antennaInfo.transmissionMode = m_rrc->m_defaultTransmissionMode;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.crossCarrierSchedulingConfig = false;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.havePdschConfigDedicated = true;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.pdschConfigDedicated.pa = LteRrcSap::PdschConfigDedicated::dB0;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.haveUlConfiguration = true;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.haveAntennaInfoUlDedicated = true;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.antennaInfoUl.transmissionMode = m_rrc->m_defaultTransmissionMode;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.pushConfigDedicatedSCell.nPuschIdentity = 0;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.ulPowerControlDedicatedSCell.pSrsOffset = 0;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.haveSoundingRsUlConfigDedicated = true;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.soundingRsUlConfigDedicated.srsConfigIndex = GetSrsConfigurationIndex();
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.soundingRsUlConfigDedicated.type = LteRrcSap::SoundingRsUlConfigDedicated::SETUP;
          component.radioResourceConfigDedicateSCell.physicalConfigDedicatedSCell.soundingRsUlConfigDedicated.srsBandwidth = 0;
        }
      else 
        {
          component.haveRadioResourceConfigDedicatedSCell = false;
        }
      SccCon.push_back (component);
    }
  ncec.sCellsToAddModList = SccCon;

  return ncec;
}

Ptr<LteSignalingRadioBearerInfo>
UeManager::GetSignalingRadioBearerInfo (uint8_t srb)
{
  switch (srb)
  {
  case 0:
    return m_srb0;
  case 1:
    return m_srb1;
  default:
    break;
  }
}


///////////////////////////////////////////
// eNB RRC methods
///////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED (LteEnbRrc);

LteEnbRrc::LteEnbRrc ()
  : m_x2SapProvider (0),
    m_cmacSapProvider (0),
    m_handoverManagementSapProvider (0),
    m_ccmRrcSapProvider (0),
    m_anrSapProvider (0),
    m_ffrRrcSapProvider (0),
    m_rrcSapUser (0),
    m_macSapProvider (0),
    m_s1SapProvider (0),
    m_cphySapProvider (0),
    m_configured (false),
    m_lastAllocatedRnti (0),
    m_srsCurrentPeriodicityId (0),
    m_lastAllocatedConfigurationIndex (0),
    m_reconfigureUes (false),
    m_numberOfComponentCarriers (0),
    m_carriersConfigured (false)
{
  NS_LOG_FUNCTION (this);
  m_cmacSapUser.push_back (new EnbRrcMemberLteEnbCmacSapUser (this, 0));
  m_handoverManagementSapUser = new MemberLteHandoverManagementSapUser<LteEnbRrc> (this);
  m_anrSapUser = new MemberLteAnrSapUser<LteEnbRrc> (this);
  m_ffrRrcSapUser.push_back (new MemberLteFfrRrcSapUser<LteEnbRrc> (this));
  m_rrcSapProvider = new MemberLteEnbRrcSapProvider<LteEnbRrc> (this);
  m_x2SapUser = new EpcX2SpecificEpcX2SapUser<LteEnbRrc> (this);
  m_s1SapUser = new MemberEpcEnbS1SapUser<LteEnbRrc> (this);
  m_cphySapUser.push_back (new MemberLteEnbCphySapUser<LteEnbRrc> (this));
  m_ccmRrcSapUser = new MemberLteCcmRrcSapUser <LteEnbRrc>(this);
  m_beamSweepCompleted = {};
  m_beamSweepStarted = {};

  // REMLAB: for tracing purposes only. This is actually only functionally used in NrGnbPhy and NrUePhy
  m_gnbSectorDegreeMap[SectorDegreeIndex{0, 4.5, 10}] = 10;
  m_gnbSectorDegreeMap[SectorDegreeIndex{355.5, 360, 10}] = 10;

  m_gnbSectorDegreeMap[SectorDegreeIndex{4.5, 13.5, 11}] = 11;
  m_gnbSectorDegreeMap[SectorDegreeIndex{13.5, 22.5, 12}] = 12;
  m_gnbSectorDegreeMap[SectorDegreeIndex{22.5, 31.5, 13}] = 13;
  m_gnbSectorDegreeMap[SectorDegreeIndex{31.5, 40.5, 14}] = 14;
  m_gnbSectorDegreeMap[SectorDegreeIndex{40.5, 49.5, 15}] = 15;
  m_gnbSectorDegreeMap[SectorDegreeIndex{49.5, 58.5, 16}] = 16;
  m_gnbSectorDegreeMap[SectorDegreeIndex{58.5, 67.5, 17}] = 17;
  m_gnbSectorDegreeMap[SectorDegreeIndex{67.5, 76.5, 18}] = 18;
  m_gnbSectorDegreeMap[SectorDegreeIndex{76.5, 85.5, 19}] = 19;
  m_gnbSectorDegreeMap[SectorDegreeIndex{85.5, 94.5, 20}] = 20;
  m_gnbSectorDegreeMap[SectorDegreeIndex{94.5, 103.5, 21}] = 19;
  m_gnbSectorDegreeMap[SectorDegreeIndex{103.5, 112.5, 22}] = 18;
  m_gnbSectorDegreeMap[SectorDegreeIndex{112.5, 121.5, 23}] = 17;
  m_gnbSectorDegreeMap[SectorDegreeIndex{121.5, 130.5, 24}] = 16;
  m_gnbSectorDegreeMap[SectorDegreeIndex{130.5, 139.5, 25}] = 15;
  m_gnbSectorDegreeMap[SectorDegreeIndex{139.5, 148.5, 26}] = 14;
  m_gnbSectorDegreeMap[SectorDegreeIndex{148.5, 157.5, 27}] = 13;
  m_gnbSectorDegreeMap[SectorDegreeIndex{157.5, 166.5, 28}] = 12;
  m_gnbSectorDegreeMap[SectorDegreeIndex{166.5, 175.5, 29}] = 11;
  m_gnbSectorDegreeMap[SectorDegreeIndex{175.5, 184.5, 30}] = 10;
  m_gnbSectorDegreeMap[SectorDegreeIndex{184.5, 193.5, 31}] = 9;
  m_gnbSectorDegreeMap[SectorDegreeIndex{193.5, 202.5, 32}] = 8;
  m_gnbSectorDegreeMap[SectorDegreeIndex{202.5, 211.5, 33}] = 7;
  m_gnbSectorDegreeMap[SectorDegreeIndex{211.5, 220.5, 34}] = 6;
  m_gnbSectorDegreeMap[SectorDegreeIndex{220.5, 229.5, 35}] = 5;
  m_gnbSectorDegreeMap[SectorDegreeIndex{229.5, 238.5, 36}] = 4;
  m_gnbSectorDegreeMap[SectorDegreeIndex{238.5, 247.5, 37}] = 3;
  m_gnbSectorDegreeMap[SectorDegreeIndex{247.5, 256.5, 38}] = 2;
  m_gnbSectorDegreeMap[SectorDegreeIndex{256.5, 265.5, 39}] = 1;
  m_gnbSectorDegreeMap[SectorDegreeIndex{265.5, 274.5, 0}] = 0;
  m_gnbSectorDegreeMap[SectorDegreeIndex{274.5, 283.5, 1}] = 1;
  m_gnbSectorDegreeMap[SectorDegreeIndex{283.5, 292.5, 2}] = 2;
  m_gnbSectorDegreeMap[SectorDegreeIndex{292.5, 301.5, 3}] = 3;
  m_gnbSectorDegreeMap[SectorDegreeIndex{301.5, 310.5, 4}] = 4;
  m_gnbSectorDegreeMap[SectorDegreeIndex{310.5, 319.5, 5}] = 5;
  m_gnbSectorDegreeMap[SectorDegreeIndex{319.5, 328.5, 6}] = 6;
  m_gnbSectorDegreeMap[SectorDegreeIndex{328.5, 337.5, 7}] = 7;
  m_gnbSectorDegreeMap[SectorDegreeIndex{337.5, 346.5, 8}] = 8;
  m_gnbSectorDegreeMap[SectorDegreeIndex{346.5, 355.5, 9}] = 9;


  m_gnbElevationDegreeMap[{0, -14.9999}] = 90;
  m_gnbElevationDegreeMap[{-15, -44.9999}] = 120;
  m_gnbElevationDegreeMap[{-45, -90}] = 150;

  // UE sector and elevation mapping
  m_ueSectorDegreeMap[SectorDegreeIndex{0, 9, 5}] = 5;
  m_ueSectorDegreeMap[SectorDegreeIndex{351, 360, 5}] = 5;

  m_ueSectorDegreeMap[SectorDegreeIndex{9, 27, 6}] = 6;
  m_ueSectorDegreeMap[SectorDegreeIndex{27, 45, 7}] = 7;
  m_ueSectorDegreeMap[SectorDegreeIndex{45, 63, 8}] = 8;
  m_ueSectorDegreeMap[SectorDegreeIndex{63, 81, 9}] = 9;
  m_ueSectorDegreeMap[SectorDegreeIndex{81, 99, 10}] = 10;
  m_ueSectorDegreeMap[SectorDegreeIndex{99, 117, 11}] = 9;
  m_ueSectorDegreeMap[SectorDegreeIndex{117, 135, 12}] = 8;
  m_ueSectorDegreeMap[SectorDegreeIndex{135, 153, 13}] = 7;
  m_ueSectorDegreeMap[SectorDegreeIndex{153, 171, 14}] = 6;
  m_ueSectorDegreeMap[SectorDegreeIndex{171, 189, 15}] = 5;
  m_ueSectorDegreeMap[SectorDegreeIndex{189, 207, 16}] = 4;
  m_ueSectorDegreeMap[SectorDegreeIndex{207, 225, 17}] = 3;
  m_ueSectorDegreeMap[SectorDegreeIndex{225, 243, 18}] = 2;
  m_ueSectorDegreeMap[SectorDegreeIndex{243, 261, 19}] = 1;
  m_ueSectorDegreeMap[SectorDegreeIndex{261, 279, 0}] = 0;
  m_ueSectorDegreeMap[SectorDegreeIndex{279, 297, 1}] = 1;
  m_ueSectorDegreeMap[SectorDegreeIndex{297, 315, 2}] = 2;
  m_ueSectorDegreeMap[SectorDegreeIndex{315, 333, 3}] = 3;
  m_ueSectorDegreeMap[SectorDegreeIndex{333, 351, 4}] = 4;


  m_ueElevationDegreeMap[{0, 14.9999}] = 90;
  m_ueElevationDegreeMap[{15, 44.9999}] = 60;
  m_ueElevationDegreeMap[{45, 90}] = 30;
}

void
LteEnbRrc::ConfigureCarriers (std::map<uint8_t, Ptr<ComponentCarrierBaseStation>> ccPhyConf)
{
  NS_ASSERT_MSG (!m_carriersConfigured, "Secondary carriers can be configured only once.");
  m_componentCarrierPhyConf = ccPhyConf;
  NS_ABORT_MSG_IF (m_numberOfComponentCarriers != m_componentCarrierPhyConf.size (), " Number of component carriers "
                                                  "are not equal to the number of he component carrier configuration provided");

  for (uint8_t i = 1; i < m_numberOfComponentCarriers; i++)
    {
      m_cphySapUser.push_back (new MemberLteEnbCphySapUser<LteEnbRrc> (this));
      m_cmacSapUser.push_back (new EnbRrcMemberLteEnbCmacSapUser (this, i));
      m_ffrRrcSapUser.push_back (new MemberLteFfrRrcSapUser<LteEnbRrc> (this));
    }
  m_carriersConfigured = true;
  Object::DoInitialize ();
}

LteEnbRrc::~LteEnbRrc ()
{
  NS_LOG_FUNCTION (this);
}


void
LteEnbRrc::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  for ( uint8_t i = 0; i < m_numberOfComponentCarriers ; i++)
    {
      delete m_cphySapUser[i];
      delete m_cmacSapUser[i];
      delete m_ffrRrcSapUser[i];
    }
  //delete m_cphySapUser;        
  m_cphySapUser.erase (m_cphySapUser.begin (),m_cphySapUser.end ());
  m_cphySapUser.clear ();  
  //delete m_cmacSapUser;
  m_cmacSapUser.erase (m_cmacSapUser.begin (),m_cmacSapUser.end ());
  m_cmacSapUser.clear ();  
  //delete m_ffrRrcSapUser;
  m_ffrRrcSapUser.erase (m_ffrRrcSapUser.begin (),m_ffrRrcSapUser.end ());
  m_ffrRrcSapUser.clear ();
  m_ueMap.clear ();  
  delete m_handoverManagementSapUser;
  delete m_ccmRrcSapUser;
  delete m_anrSapUser;
  delete m_rrcSapProvider;
  delete m_x2SapUser;
  delete m_s1SapUser;

}

TypeId
LteEnbRrc::GetTypeId (void)
{
  NS_LOG_FUNCTION ("LteEnbRrc::GetTypeId");
  static TypeId tid = TypeId ("ns3::LteEnbRrc")
    .SetParent<Object> ()
    .SetGroupName("Lte")
    .AddConstructor<LteEnbRrc> ()
    .AddAttribute ("UeMap", "List of UeManager by C-RNTI.",
                   ObjectMapValue (),
                   MakeObjectMapAccessor (&LteEnbRrc::m_ueMap),
                   MakeObjectMapChecker<UeManager> ())
    .AddAttribute ("DefaultTransmissionMode",
                   "The default UEs' transmission mode (0: SISO)",
                   UintegerValue (0),  // default tx-mode
                   MakeUintegerAccessor (&LteEnbRrc::m_defaultTransmissionMode),
                   MakeUintegerChecker<uint8_t> ())
    .AddAttribute ("EpsBearerToRlcMapping", 
                   "Specify which type of RLC will be used for each type of EPS bearer. ",
                   EnumValue (RLC_SM_ALWAYS),
                   MakeEnumAccessor (&LteEnbRrc::m_epsBearerToRlcMapping),
                   MakeEnumChecker (RLC_SM_ALWAYS, "RlcSmAlways",
                                    RLC_UM_ALWAYS, "RlcUmAlways",
                                    RLC_AM_ALWAYS, "RlcAmAlways",
                                    RLC_TM_ALWAYS, "RlcTmAlways",
                                    PER_BASED,     "PacketErrorRateBased"))
    .AddAttribute ("SystemInformationPeriodicity",
                   "The interval for sending system information (Time value)",
                   TimeValue (MilliSeconds (80)),
                   MakeTimeAccessor (&LteEnbRrc::m_systemInformationPeriodicity),
                   MakeTimeChecker ())

    // SRS related attributes
    .AddAttribute ("SrsPeriodicity",
                   "The SRS periodicity in milliseconds",
                   UintegerValue (40),
                   MakeUintegerAccessor (&LteEnbRrc::SetSrsPeriodicity, 
                                         &LteEnbRrc::GetSrsPeriodicity),
                   MakeUintegerChecker<uint32_t> ())

    // Timeout related attributes
    .AddAttribute ("ConnectionRequestTimeoutDuration",
                   "After a RA attempt, if no RRC CONNECTION REQUEST is "
                   "received before this time, the UE context is destroyed. "
                   "Must account for reception of RAR and transmission of "
                   "RRC CONNECTION REQUEST over UL GRANT. The value of this"
                   "timer should not be greater than T300 timer at UE RRC",
                   TimeValue (MilliSeconds (15)),
                   MakeTimeAccessor (&LteEnbRrc::m_connectionRequestTimeoutDuration),
                   MakeTimeChecker (MilliSeconds (1), MilliSeconds (15)))
    .AddAttribute ("ConnectionSetupTimeoutDuration",
                   "After accepting connection request, if no RRC CONNECTION "
                   "SETUP COMPLETE is received before this time, the UE "
                   "context is destroyed. Must account for the UE's reception "
                   "of RRC CONNECTION SETUP and transmission of RRC CONNECTION "
                   "SETUP COMPLETE.",
                   TimeValue (MilliSeconds (150)),
                   MakeTimeAccessor (&LteEnbRrc::m_connectionSetupTimeoutDuration),
                   MakeTimeChecker ())
    .AddAttribute ("ConnectionRejectedTimeoutDuration",
                   "Time to wait between sending a RRC CONNECTION REJECT and "
                   "destroying the UE context",
                   TimeValue (MilliSeconds (30)),
                   MakeTimeAccessor (&LteEnbRrc::m_connectionRejectedTimeoutDuration),
                   MakeTimeChecker ())
    .AddAttribute ("HandoverJoiningTimeoutDuration",
                   "After accepting a handover request, if no RRC CONNECTION "
                   "RECONFIGURATION COMPLETE is received before this time, the "
                   "UE context is destroyed. Must account for reception of "
                   "X2 HO REQ ACK by source eNB, transmission of the Handover "
                   "Command, non-contention-based random access and reception "
                   "of the RRC CONNECTION RECONFIGURATION COMPLETE message.",
                   TimeValue (MilliSeconds (200)),
                   MakeTimeAccessor (&LteEnbRrc::m_handoverJoiningTimeoutDuration),
                   MakeTimeChecker ())
    .AddAttribute ("HandoverLeavingTimeoutDuration",
                   "After issuing a Handover Command, if neither RRC "
                   "CONNECTION RE-ESTABLISHMENT nor X2 UE Context Release has "
                   "been previously received, the UE context is destroyed.",
                   TimeValue (MilliSeconds (500)),
                   MakeTimeAccessor (&LteEnbRrc::m_handoverLeavingTimeoutDuration),
                   MakeTimeChecker ())
    .AddAttribute ("BeamSweepTimeoutDuration",
                   "After triggering a beam sweep at the UE's RRC"
                   "this will be called to see whether a beam sweep "
                   "has indeed been conducted",
                   TimeValue (MilliSeconds (500)),
                   MakeTimeAccessor (&LteEnbRrc::m_beamSweepCompleteTimeoutDuration),
                   MakeTimeChecker ())                    
    .AddAttribute ("OutageThreshold",
                    "SNR threshold for outage events [dB]",
                    DoubleValue (-5.0),
                    MakeDoubleAccessor (&LteEnbRrc::m_outageThreshold),
                    MakeDoubleChecker<long double> (-10000.0, 10.0))
    // Cell selection related attribute
    .AddAttribute ("QRxLevMin",
                   "One of information transmitted within the SIB1 message, "
                   "indicating the required minimum RSRP level that any UE must "
                   "receive from this cell before it is allowed to camp to this "
                   "cell. The default value -70 corresponds to -140 dBm and is "
                   "the lowest possible value as defined by Section 6.3.4 of "
                   "3GPP TS 36.133. This restriction, however, only applies to "
                   "initial cell selection and EPC-enabled simulation.",
                   TypeId::ATTR_GET | TypeId::ATTR_CONSTRUCT,
                   IntegerValue (-70),
                   MakeIntegerAccessor (&LteEnbRrc::m_qRxLevMin),
                   MakeIntegerChecker<int8_t> (-70, -22))
    .AddAttribute ("NumberOfComponentCarriers",
                   "Number of Component Carriers ",
                   UintegerValue (1),
                   MakeIntegerAccessor (&LteEnbRrc::m_numberOfComponentCarriers),
                   MakeIntegerChecker<int16_t> (MIN_NO_CC, MAX_NO_CC))

    // Handover related attributes
    .AddAttribute ("AdmitHandoverRequest",
                   "Whether to admit an X2 handover request from another eNB",
                   BooleanValue (true),
                   MakeBooleanAccessor (&LteEnbRrc::m_admitHandoverRequest),
                   MakeBooleanChecker ())
    .AddAttribute ("AdmitRrcConnectionRequest",
                   "Whether to admit a connection request from a UE",
                   BooleanValue (true),
                   MakeBooleanAccessor (&LteEnbRrc::m_admitRrcConnectionRequest),
                   MakeBooleanChecker ())
    .AddAttribute ("MinDynTttValue",
                   "The minimum value of TTT in case of dynamic TTT handover (in ms)",
                   UintegerValue(25),
                   MakeUintegerAccessor(&LteEnbRrc::m_minDynTttValue),
                   MakeUintegerChecker<uint8_t>()) // TODO consider using a TimeValue
    .AddAttribute ("MaxDynTttValue",
                   "The maximum value of TTT in case of dynamic TTT handover (in ms)",
                   UintegerValue(150),
                   MakeUintegerAccessor(&LteEnbRrc::m_maxDynTttValue),
                   MakeUintegerChecker<uint8_t>()) // TODO consider using a TimeValue
    .AddAttribute ("MinDiffValue",
                   "The minimum value of the difference in case of dynamic TTT handover [dB]",
                   DoubleValue(3),
                   MakeDoubleAccessor(&LteEnbRrc::m_minDiffTttValue),
                   MakeDoubleChecker<double>()) // TODO set the proper value
    .AddAttribute ("MaxDiffValue",
                   "The maximum value of the difference in case of dynamic TTT handover [dB]",
                   DoubleValue(20),
                   MakeDoubleAccessor(&LteEnbRrc::m_maxDiffTttValue),
                   MakeDoubleChecker<double>()) // TODO set the proper value
    .AddAttribute ("CrtPeriod",
                   "The periodicity of a CRT (us)",
                   IntegerValue(1600),
                   MakeIntegerAccessor(&LteEnbRrc::m_crtPeriod),
                   MakeIntegerChecker<int>()) // TODO consider using a TimeValue
   .AddAttribute ("HoSinrDifference",
                  "The value for which an handover between MmWave eNB is triggered",
                   DoubleValue (5),
                   MakeDoubleAccessor (&LteEnbRrc::m_sinrThresholdDifference),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MaxRateThreshold",
                    "SNR Threshold for max. rate",
                    DoubleValue (22.7),
                    MakeDoubleAccessor (&LteEnbRrc::m_maxRateThreshold),
                    MakeDoubleChecker <double> (-1000.0, 1000.0))

    // UE measurements related attributes
    .AddAttribute ("RsrpFilterCoefficient",
                   "Determines the strength of smoothing effect induced by "
                   "layer 3 filtering of RSRP in all attached UE; "
                   "if set to 0, no layer 3 filtering is applicable",
                   // i.e. the variable k in 3GPP TS 36.331 section 5.5.3.2
                   UintegerValue (4),
                   MakeUintegerAccessor (&LteEnbRrc::m_rsrpFilterCoefficient),
                   MakeUintegerChecker<uint8_t> (0))
    .AddAttribute ("RsrqFilterCoefficient",
                   "Determines the strength of smoothing effect induced by "
                   "layer 3 filtering of RSRQ in all attached UE; "
                   "if set to 0, no layer 3 filtering is applicable",
                   // i.e. the variable k in 3GPP TS 36.331 section 5.5.3.2
                   UintegerValue (4),
                   MakeUintegerAccessor (&LteEnbRrc::m_rsrqFilterCoefficient),
                   MakeUintegerChecker<uint8_t> (0))
    .AddAttribute ("CSIRSPeriodicity",
                   "Periodicity (in slots) for CSI-RS signals",
                   UintegerValue (6),
                   MakeUintegerAccessor (&LteEnbRrc::m_csiPeriodicityInSlots),
                   MakeUintegerChecker<uint16_t> (0,1000))
    .AddAttribute ("CSIRSOffset",
                   "Offset (in slot) for CSI-RS signal (calculated from the end of a SSB",
                    UintegerValue (11),
                    MakeUintegerAccessor (&LteEnbRrc::m_csiOffsetInSlots),
                    MakeUintegerChecker<uint16_t> (0,1000))
    .AddAttribute ("MaxNumOfCSIRSResource",
                   "Maximum number of time resources allocated for CSI-RS signals",
                    UintegerValue (8),
                    MakeUintegerAccessor (&LteEnbRrc::m_maxCSIperFrame),
                    MakeUintegerChecker<uint16_t> (0,1000))
    .AddAttribute ("SSBPeriodicity",
                   "Periodicity of SS Burst in ms",
                    TimeValue (MilliSeconds (20)),
                    MakeTimeAccessor (&LteEnbRrc::m_ssbPeriodicity),
                    MakeTimeChecker ())

    // REMLAB: Location-Aided Beam Management
    .AddAttribute ("UseREMLAB", "Use REM-based cell and beam selection for UEs and gNBs",
                    BooleanValue (false),
                    MakeBooleanAccessor (&LteEnbRrc::m_useREMLAB),
                    MakeBooleanChecker ())
    .AddAttribute("ApplyErrorModelToUePositions", "Applies Error Model to recevied UE Position Reports",
                    BooleanValue (false),
                    MakeBooleanAccessor(&LteEnbRrc::m_useErroneousUePosition),
                    MakeBooleanChecker())
    .AddAttribute("UseMeasurementREM",
                    "When loading REM, apply Normally distirbuted error for the positions where entries are saved",
                    BooleanValue(false),
                    MakeBooleanAccessor(&LteEnbRrc::m_useErroneousREM),
                    MakeBooleanChecker())
    .AddAttribute("UseREMLABImmediateHandovers",
                    "If the user is not served by the best cell, immediatelly command a HO to it",
                    BooleanValue(false),
                    MakeBooleanAccessor(&LteEnbRrc::m_immediateREMLABHandovers),
                    MakeBooleanChecker())
    .AddAttribute("REMLABHandoverThreshold",
                    "Amount of position reports for which the UE is allowed to be served by a suboptimal gNB",
                    UintegerValue(1),
                    MakeUintegerAccessor(&LteEnbRrc::m_handoverHysterisisMaxCounter),
                    MakeUintegerChecker<uint8_t>())
    .AddAttribute("GnbLocationOffsetX",
                    "Offset for the position of gNB on X axis.",
                    IntegerValue(0),
                    MakeIntegerAccessor(&LteEnbRrc::m_gnbLocationOffsetX),
                    MakeIntegerChecker<int>())
    .AddAttribute("GnbLocationOffsetY",
                    "Offset for the position of gNB on Y axis.",
                    IntegerValue(0),
                    MakeIntegerAccessor(&LteEnbRrc::m_gnbLocationOffsetY),
                    MakeIntegerChecker<int>())
    .AddAttribute("EnableRemRecoveryAlgorithm",
                    "Defines whether Bresenham's line algorithm is used if no REM entry found for the queried position",
                    BooleanValue(false),
                    MakeBooleanAccessor(&LteEnbRrc::m_enableRemRecoveryAlgorithm),
                    MakeBooleanChecker())
    .AddAttribute("LoadMeasurementRemFromFile",
                    "Whether we parse the REM from Inventory file and apply error on top or load directly from file",
                    BooleanValue(false),
                    MakeBooleanAccessor(&LteEnbRrc::m_loadMeasurementRem),
                    MakeBooleanChecker())
    // Trace sources
    .AddTraceSource ("NewUeContext",
                     "Fired upon creation of a new UE context.",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_newUeContextTrace),
                     "ns3::LteEnbRrc::NewUeContextTracedCallback")
    .AddTraceSource ("ConnectionEstablished",
                     "Fired upon successful RRC connection establishment.",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_connectionEstablishedTrace),
                     "ns3::LteEnbRrc::ConnectionHandoverTracedCallback")
    .AddTraceSource ("ConnectionReconfiguration",
                     "trace fired upon RRC connection reconfiguration",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_connectionReconfigurationTrace),
                     "ns3::LteEnbRrc::ConnectionHandoverTracedCallback")
    .AddTraceSource ("HandoverStart",
                     "trace fired upon start of a handover procedure",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_handoverStartTrace),
                     "ns3::LteEnbRrc::HandoverStartTracedCallback")
    .AddTraceSource ("HandoverEndOk",
                     "trace fired upon successful termination of a handover procedure",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_handoverEndOkTrace),
                     "ns3::LteEnbRrc::ConnectionHandoverTracedCallback")
    .AddTraceSource ("RecvMeasurementReport",
                     "trace fired when measurement report is received",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_recvMeasurementReportTrace),
                     "ns3::LteEnbRrc::ReceiveReportTracedCallback")
    .AddTraceSource ("NotifyConnectionRelease",
                     "trace fired when an UE is released",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_connectionReleaseTrace),
                     "ns3::LteEnbRrc::ConnectionHandoverTracedCallback")
    .AddTraceSource ("RrcTimeout",
                     "trace fired when a timer expires",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_rrcTimeoutTrace),
                     "ns3::LteEnbRrc::TimerExpiryTracedCallback")
    .AddTraceSource ("BeamSweepInitialization",
                    "trace fired when a beam sweep has been initialized from the LTE coordinator",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_beamSweepInitiateFromCoordinatorTrace),
                    "ns3::LteEnbRrc::InitiateBeamSweepCallback")
    .AddTraceSource ("RadioLinkMonitoringTrace",
                     "trace fired when handover decisions take place",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_radioLinkMonitoringTrace),
                     "ns3::RadioLinkMonitoringTrace::TracedCallback")
    // REMLAB: Location-Aided Beam Management
    .AddTraceSource ("RecvPositionReport",
                     "trace fired when eNB receives position data from UE",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_recvPositionReportTrace),
                     "ns3::LteEnbRrc::ReceivePositionReportTracedCallback")
    .AddTraceSource ("LocationErrorTrace",
                    "trace fired when UE position arrives. Provides mean, variance for x,y coords",
                    MakeTraceSourceAccessor (&LteEnbRrc::m_locationErrorTrace),
                    "ns3::LteEnbRrc::LocationErrorTrace")
    .AddTraceSource ("UePositionTrace",
                    "trace with a complete info about UE position",
                    MakeTraceSourceAccessor(&LteEnbRrc::m_uePositionTrace),
                    "ns3::LteEnbRrc::UePositionReportTracedCallback")
    .AddTraceSource ("RemBeamSelectionTrace",
                    "trace selected beams for real and erroneous position of UE",
                    MakeTraceSourceAccessor(&LteEnbRrc::m_remBeamSelectionTrace),
                    "ns3::LteEnbRrc::RemBeamSelectionTracedCallback")
    .AddTraceSource("RemQueryErrorTrace",
                    "Map containing position error applied on top of each REM entry",
                    MakeTraceSourceAccessor(&LteEnbRrc::m_remErrorTrace),
                    "ns3::LteEnbRrc::RemMeasurementErrorTracedCallback")
    .AddTraceSource("RemNonEmptyEntriesTrace",
                    "Map containing a boolean for each entry telling whether it contains data",
                    MakeTraceSourceAccessor(&LteEnbRrc::m_measRemNonEmptyEntries),
                    "ns3::LteEnbRrc::RemNonEmptyEntriesTracedCallback")
    .AddTraceSource ("RemQueryRecoveryTrace",
                    "trace the query process of REM and recovery algorithm",
                    MakeTraceSourceAccessor(&LteEnbRrc::m_remRecoveryAlgoTrace),
                    "ns3::LteEnbRrc::RemQueryRecoveryAlgorithmTracedCallback")
  ;
  return tid;
}

void
LteEnbRrc::SetEpcX2SapProvider (EpcX2SapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_x2SapProvider = s;
}

EpcX2SapUser*
LteEnbRrc::GetEpcX2SapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_x2SapUser;
}

void
LteEnbRrc::SetLteEnbCmacSapProvider (LteEnbCmacSapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_cmacSapProvider.at (0) = s;
}

void
LteEnbRrc::SetLteEnbCmacSapProvider (LteEnbCmacSapProvider * s, uint8_t pos)
{
  NS_LOG_FUNCTION (this << s);
  if (m_cmacSapProvider.size () > pos)
    {
      m_cmacSapProvider.at (pos) = s;
    }
  else
    {
      m_cmacSapProvider.push_back (s);
      NS_ABORT_IF (m_cmacSapProvider.size () - 1 != pos);
    }
}

LteEnbCmacSapUser*
LteEnbRrc::GetLteEnbCmacSapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_cmacSapUser.at (0);
}

LteEnbCmacSapUser*
LteEnbRrc::GetLteEnbCmacSapUser (uint8_t pos)
{
  NS_LOG_FUNCTION (this);
  return m_cmacSapUser.at (pos);
}

void
LteEnbRrc::SetLteHandoverManagementSapProvider (LteHandoverManagementSapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_handoverManagementSapProvider = s;
}

LteHandoverManagementSapUser*
LteEnbRrc::GetLteHandoverManagementSapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_handoverManagementSapUser;
}

void
LteEnbRrc::SetLteCcmRrcSapProvider (LteCcmRrcSapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_ccmRrcSapProvider = s;
}

LteCcmRrcSapUser*
LteEnbRrc::GetLteCcmRrcSapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_ccmRrcSapUser;
}

void
LteEnbRrc::SetLteAnrSapProvider (LteAnrSapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_anrSapProvider = s;
}

LteAnrSapUser*
LteEnbRrc::GetLteAnrSapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_anrSapUser;
}

void
LteEnbRrc::SetLteFfrRrcSapProvider (LteFfrRrcSapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  if (m_ffrRrcSapProvider.size () > 0)
    {
      m_ffrRrcSapProvider.at (0) = s;
    }
  else
    {
      m_ffrRrcSapProvider.push_back (s);
    }

}

void
LteEnbRrc::SetLteFfrRrcSapProvider (LteFfrRrcSapProvider * s, uint8_t index)
{
  NS_LOG_FUNCTION (this << s);
  if (m_ffrRrcSapProvider.size () > index)
    {
      m_ffrRrcSapProvider.at (index) = s;
    }
  else
    {
      m_ffrRrcSapProvider.push_back (s);
      NS_ABORT_MSG_IF (m_ffrRrcSapProvider.size () - 1 != index,
                       "You meant to store the pointer at position " <<
                       static_cast<uint32_t> (index) <<
                       " but it went to " << m_ffrRrcSapProvider.size () - 1);
    }
}

LteFfrRrcSapUser*
LteEnbRrc::GetLteFfrRrcSapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_ffrRrcSapUser.at (0);
}

LteFfrRrcSapUser*
LteEnbRrc::GetLteFfrRrcSapUser (uint8_t index)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (index < m_numberOfComponentCarriers, "Invalid component carrier index:"<<index<<" provided in order to obtain FfrRrcSapUser.");
  return m_ffrRrcSapUser.at (index);
}

void
LteEnbRrc::SetLteEnbRrcSapUser (LteEnbRrcSapUser * s)
{
  NS_LOG_FUNCTION (this << s);
  m_rrcSapUser = s;
}

LteEnbRrcSapProvider*
LteEnbRrc::GetLteEnbRrcSapProvider ()
{
  NS_LOG_FUNCTION (this);
  return m_rrcSapProvider;
}

void
LteEnbRrc::SetLteMacSapProvider (LteMacSapProvider * s)
{
  NS_LOG_FUNCTION (this);
  m_macSapProvider = s;
}

void 
LteEnbRrc::SetS1SapProvider (EpcEnbS1SapProvider * s)
{
  m_s1SapProvider = s;
}


EpcEnbS1SapUser* 
LteEnbRrc::GetS1SapUser ()
{
  return m_s1SapUser;
}

void
LteEnbRrc::SetLteEnbCphySapProvider (LteEnbCphySapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  if (m_cphySapProvider.size () > 0)
    {
      m_cphySapProvider.at (0) = s;
    }
  else
    {
      m_cphySapProvider.push_back (s);
    }
}

LteEnbCphySapUser*
LteEnbRrc::GetLteEnbCphySapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_cphySapUser.at(0);
}

void
LteEnbRrc::SetLteEnbCphySapProvider (LteEnbCphySapProvider * s, uint8_t pos)
{
  NS_LOG_FUNCTION (this << s);
  if (m_cphySapProvider.size () > pos)
    {
      m_cphySapProvider.at(pos) = s;
    }
  else
    {
      m_cphySapProvider.push_back (s);
      NS_ABORT_IF (m_cphySapProvider.size () - 1 != pos);
    }
}

LteEnbCphySapUser*
LteEnbRrc::GetLteEnbCphySapUser (uint8_t pos)
{
  NS_LOG_FUNCTION (this);
  return m_cphySapUser.at(pos);
}

bool
LteEnbRrc::HasUeManager (uint16_t rnti) const
{
  NS_LOG_FUNCTION (this << (uint32_t) rnti);
  std::map<uint16_t, Ptr<UeManager> >::const_iterator it = m_ueMap.find (rnti);
  return (it != m_ueMap.end ());
}

Ptr<UeManager>
LteEnbRrc::GetUeManager (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << (uint32_t) rnti);
  NS_ASSERT (0 != rnti);
  std::map<uint16_t, Ptr<UeManager> >::iterator it = m_ueMap.find (rnti);
  NS_ASSERT_MSG (it != m_ueMap.end (), "UE manager for RNTI " << rnti << " not found");
  return it->second;
}

uint8_t
LteEnbRrc::AddUeMeasReportConfig (LteRrcSap::ReportConfigEutra config)
{
  NS_LOG_FUNCTION (this);

  // SANITY CHECK

  NS_ASSERT_MSG (m_ueMeasConfig.measIdToAddModList.size () == m_ueMeasConfig.reportConfigToAddModList.size (),
                 "Measurement identities and reporting configuration should not have different quantity");

  if (Simulator::Now () != Seconds (0))
    {
      NS_FATAL_ERROR ("AddUeMeasReportConfig may not be called after the simulation has run");
    }

  // INPUT VALIDATION

  switch (config.triggerQuantity)
    {
    case LteRrcSap::ReportConfigEutra::RSRP:
      if ((config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A5)
          && (config.threshold2.choice != LteRrcSap::ThresholdEutra::THRESHOLD_RSRP))
        {
          NS_FATAL_ERROR ("The given triggerQuantity (RSRP) does not match with the given threshold2.choice");
        }

      if (((config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A1)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A2)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A4)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A5))
          && (config.threshold1.choice != LteRrcSap::ThresholdEutra::THRESHOLD_RSRP))
        {
          NS_FATAL_ERROR ("The given triggerQuantity (RSRP) does not match with the given threshold1.choice");
        }
      break;

    case LteRrcSap::ReportConfigEutra::RSRQ:
      if ((config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A5)
          && (config.threshold2.choice != LteRrcSap::ThresholdEutra::THRESHOLD_RSRQ))
        {
          NS_FATAL_ERROR ("The given triggerQuantity (RSRQ) does not match with the given threshold2.choice");
        }

      if (((config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A1)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A2)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A4)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A5))
          && (config.threshold1.choice != LteRrcSap::ThresholdEutra::THRESHOLD_RSRQ))
        {
          NS_FATAL_ERROR ("The given triggerQuantity (RSRQ) does not match with the given threshold1.choice");
        }
      break;

    default:
      NS_FATAL_ERROR ("unsupported triggerQuantity");
      break;
    }

  if (config.purpose != LteRrcSap::ReportConfigEutra::REPORT_STRONGEST_CELLS)
    {
      NS_FATAL_ERROR ("Only REPORT_STRONGEST_CELLS purpose is supported");
    }

  if (config.reportQuantity != LteRrcSap::ReportConfigEutra::BOTH)
    {
      NS_LOG_WARN ("reportQuantity = BOTH will be used instead of the given reportQuantity");
    }

  uint8_t nextId = m_ueMeasConfig.reportConfigToAddModList.size () + 1;

  // create the reporting configuration
  LteRrcSap::ReportConfigToAddMod reportConfig;
  reportConfig.reportConfigId = nextId;
  reportConfig.reportConfigEutra = config;

  // create the measurement identity
  LteRrcSap::MeasIdToAddMod measId;
  measId.measId = nextId;
  measId.measObjectId = 1;
  measId.reportConfigId = nextId;

  // add both to the list of UE measurement configuration
  m_ueMeasConfig.reportConfigToAddModList.push_back (reportConfig);
  m_ueMeasConfig.measIdToAddModList.push_back (measId);

  return nextId;
}

void
LteEnbRrc::ConfigureCell (std::map<uint8_t, Ptr<ComponentCarrierBaseStation>> ccPhyConf)
{
  auto it = ccPhyConf.begin ();
  NS_ASSERT (it != ccPhyConf.end ());
  uint16_t ulBandwidth = it->second->GetUlBandwidth ();
  uint16_t dlBandwidth = it->second->GetDlBandwidth ();
  uint32_t ulEarfcn = it->second->GetUlEarfcn ();
  uint32_t dlEarfcn = it->second->GetDlEarfcn ();
  uint16_t cellId = it->second->GetCellId ();
  NS_LOG_FUNCTION (this << ulBandwidth << dlBandwidth
                        << ulEarfcn << dlEarfcn);
  NS_ASSERT (!m_configured);

  for (const auto &it: ccPhyConf)
    {
      m_cphySapProvider.at (it.first)->SetBandwidth (it.second->GetUlBandwidth (), it.second->GetDlBandwidth ());
      m_cphySapProvider.at (it.first)->SetEarfcn (it.second->GetUlEarfcn (), it.second->GetDlEarfcn ());
      m_cphySapProvider.at (it.first)->SetCellId (it.second->GetCellId ());
      m_cmacSapProvider.at (it.first)->ConfigureMac (it.second->GetUlBandwidth (), it.second->GetDlBandwidth ());
      if (m_ffrRrcSapProvider.size () > it.first)
        {
          m_ffrRrcSapProvider.at (it.first)->SetCellId (it.second->GetCellId ());
          m_ffrRrcSapProvider.at (it.first)->SetBandwidth (it.second->GetUlBandwidth (), it.second->GetDlBandwidth ());
        }
    }

  m_dlEarfcn = dlEarfcn;
  m_ulEarfcn = ulEarfcn;
  m_dlBandwidth = dlBandwidth;
  m_ulBandwidth = ulBandwidth;
  m_cellId = cellId;

  /*
   * Initializing the list of UE measurement configuration (m_ueMeasConfig).
   * Only intra-frequency measurements are supported, so only one measurement
   * object is created.
   */

  LteRrcSap::MeasObjectToAddMod measObject;
  measObject.measObjectId = 1;
  measObject.measObjectEutra.carrierFreq = m_dlEarfcn;
  measObject.measObjectEutra.allowedMeasBandwidth = m_dlBandwidth;
  measObject.measObjectEutra.presenceAntennaPort1 = false;
  measObject.measObjectEutra.neighCellConfig = 0;
  measObject.measObjectEutra.offsetFreq = 0;
  measObject.measObjectEutra.haveCellForWhichToReportCGI = false;

  m_ueMeasConfig.measObjectToAddModList.push_back (measObject);
  m_ueMeasConfig.haveQuantityConfig = true;
  m_ueMeasConfig.quantityConfig.filterCoefficientRSRP = m_rsrpFilterCoefficient;
  m_ueMeasConfig.quantityConfig.filterCoefficientRSRQ = m_rsrqFilterCoefficient;
  m_ueMeasConfig.haveMeasGapConfig = false;
  m_ueMeasConfig.haveSmeasure = false;
  m_ueMeasConfig.haveSpeedStatePars = false;

  m_sib1.clear ();
  m_sib1.reserve (ccPhyConf.size ());
  for (const auto &it: ccPhyConf)
    {
      // Enabling MIB transmission
      LteRrcSap::MasterInformationBlock mib;
      mib.dlBandwidth = it.second->GetDlBandwidth ();
      mib.systemFrameNumber = 0;
      m_cphySapProvider.at (it.first)->SetMasterInformationBlock (mib);

      // Enabling SIB1 transmission with default values
      LteRrcSap::SystemInformationBlockType1 sib1;
      sib1.cellAccessRelatedInfo.cellIdentity = it.second->GetCellId ();
      sib1.cellAccessRelatedInfo.csgIndication = false;
      sib1.cellAccessRelatedInfo.csgIdentity = 0;
      sib1.cellAccessRelatedInfo.plmnIdentityInfo.plmnIdentity = 0; // not used
      sib1.cellSelectionInfo.qQualMin = -34; // not used, set as minimum value
      sib1.cellSelectionInfo.qRxLevMin = m_qRxLevMin; // set as minimum value
      m_sib1.push_back (sib1);
      m_cphySapProvider.at (it.first)->SetSystemInformationBlockType1 (sib1);
    }
  /*
   * Enabling transmission of other SIB. The first time System Information is
   * transmitted is arbitrarily assumed to be at +0.016s, and then it will be
   * regularly transmitted every 80 ms by default (set the
   * SystemInformationPeriodicity attribute to configure this).
   */
  Simulator::Schedule (MilliSeconds (16), &LteEnbRrc::SendSystemInformation, this);

  m_configured = true;
  m_firstReport = true;

}

void
LteEnbRrc::SetCellId (uint16_t cellId)
{
  // update SIB1
  m_sib1.at (0).cellAccessRelatedInfo.cellIdentity = cellId;
  m_cphySapProvider.at (0)->SetSystemInformationBlockType1 (m_sib1.at (0));
}

void
LteEnbRrc::SetCellId (uint16_t cellId, uint8_t ccIndex)
{
  // update SIB1
  m_sib1.at (ccIndex).cellAccessRelatedInfo.cellIdentity = cellId;
  m_cphySapProvider.at (ccIndex)->SetSystemInformationBlockType1 (m_sib1.at (ccIndex));
}

uint16_t
LteEnbRrc::GetCellId ()
{
  return m_cellId;
}

// This is called from main_flexRLM to set the coordinator cell id to all gNB's RRC
void
LteEnbRrc::SetClosestLteCellId (uint16_t cellId)
{
  m_lteCellId = cellId;
  NS_LOG_LOGIC("Closest Lte CellId set to " << m_lteCellId);
}
//****************************************************************************************************
    // This goes over X2 to CO
void
LteEnbRrc::DoUpdateUeSinrEstimate(LteEnbCphySapUser::UeAssociatedSinrInfo info)
{
  NS_LOG_FUNCTION(this);

  NS_LOG_INFO ("CC " << (uint16_t)info.componentCarrierId << " reports the ueImsiSinrMap");
  m_ueImsiSinrMap[info.componentCarrierId]=info.ueImsiSinrMap; // store the received report in m_ueImsiSinrMap

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // This is already set by the LteEnbRrc::SetClosestLteCellId
  //m_lteCellId = 11; //Hardcoded until it is decided whether to use SA or NSA

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  // TODO report immediately or with some filtering
  if(m_lteCellId > 0) // i.e., only if a LTE eNB was actually registered in the scenario
                      // (this is done when an X2 interface among mmWave eNBs and LTE eNB is added)
  {
    // send the report to the LTE coordinator
    EpcX2SapProvider::UeImsiSinrParams params;
    params.targetCellId = m_lteCellId; // CO cell id
    params.sourceCellId = m_cellId; // current cell id

    if (m_ueImsiSinrMap.size() == m_numberOfComponentCarriers) // if we received the ueImsiSinrMap report from all the CCs
    {
      // Build the ueImsiSinrMapToSend containing, for each UE, the max SINR among all the CCs
      NS_LOG_INFO ("Number of ueImsiSinrMaps in m_ueImsiSinrMap " << (uint16_t)m_ueImsiSinrMap.size() );

      std::map<uint64_t, double> ueImsiSinrMapToSend; // map which contains the max SINR for each UE among the CCs
      ueImsiSinrMapToSend = m_ueImsiSinrMap.at(0); // initialization

      for(uint8_t cc = 1; cc < m_numberOfComponentCarriers; cc++)
      {
        NS_ASSERT_MSG (m_ueImsiSinrMap.find(cc) != m_ueImsiSinrMap.end(), "CC " << (uint16_t)cc << " didn't report the ueImsiSinrMap");

        for (std::map<uint64_t, double>::iterator ue = ueImsiSinrMapToSend.begin(); ue != ueImsiSinrMapToSend.end(); ue++)
        {
          NS_ASSERT_MSG (m_ueImsiSinrMap.at(cc).find(ue->first) != m_ueImsiSinrMap.at(cc).end(), "CC " << (uint16_t)cc << " didn't report SINR for UE "<< ue->first );

          NS_LOG_DEBUG ("UE " << ue->first << " current SINR " << ue->second << " is higher than " << m_ueImsiSinrMap.at(cc).at(ue->first) << " ?");
          if (ue->second < m_ueImsiSinrMap.at(cc).at(ue->first))
          {
            NS_LOG_DEBUG ("No, update SINR to " << m_ueImsiSinrMap.at(cc).at(ue->first));
            ue->second = m_ueImsiSinrMap.at(cc).at(ue->first); // insert the max SINR for this UE among all the CCs
          }
        }
      }
      params.ueImsiSinrMap = ueImsiSinrMapToSend;
      m_ueImsiSinrMap.clear(); // delete the reports
    }

    NS_LOG_INFO("number of SINR reported " << params.ueImsiSinrMap.size());
    // This goes over X2 to CO
    m_x2SapProvider->SendUeSinrUpdate (params);
  }
}

void
LteEnbRrc::DoRecvUeSinrUpdate(EpcX2SapUser::UeImsiSinrParams params)
{
  NS_LOG_FUNCTION(this);
  NS_LOG_LOGIC("Recv Ue SINR Update from cell " << params.sourceCellId);
  uint16_t mmWaveCellId = params.sourceCellId;

  // cycle on all the Imsi whose SINR is known in cell mmWaveCellId
  for(std::map<uint64_t, double>::iterator imsiIter = params.ueImsiSinrMap.begin(); imsiIter != params.ueImsiSinrMap.end(); ++imsiIter)
  {
    uint64_t imsi = imsiIter->first;
    double sinr = imsiIter->second;

    //m_notifyMmWaveSinrTrace(imsi, mmWaveCellId, sinr);

    NS_LOG_LOGIC("Imsi " << imsi << " sinr " << sinr);

    if(m_imsiCellSinrMap.find(imsi) != m_imsiCellSinrMap.end())
    {
      if(m_imsiCellSinrMap[imsi].find(mmWaveCellId) != m_imsiCellSinrMap[imsi].end())
      {
        // update the SINR measure
        m_imsiCellSinrMap[imsi].find(mmWaveCellId)->second = sinr;
      }
      else // new cell for this Imsi
      {
        // insert a new SINR measure
        m_imsiCellSinrMap[imsi].insert(std::pair<uint16_t, double>(mmWaveCellId, sinr));
      }
    }
    else // new imsi
    {
      CellSinrMap map;
      map.insert(std::pair<uint16_t, double>(mmWaveCellId, sinr));
      m_imsiCellSinrMap.insert(std::pair<uint64_t, CellSinrMap> (imsi, map));
    }
  }

  for(std::map<uint64_t, CellSinrMap>::iterator imsiIter = m_imsiCellSinrMap.begin(); imsiIter != m_imsiCellSinrMap.end(); ++imsiIter)
  {
    double maxSNR = 0;
    uint16_t maxSNRCell = 0;
    NS_LOG_LOGIC("Imsi " << imsiIter->first);
    for(CellSinrMap::iterator cellIter = imsiIter->second.begin(); cellIter != imsiIter->second.end(); ++cellIter)
    {
      //added to reduce console output during simulation
      if(maxSNR < cellIter->second)
      {
        maxSNR = cellIter->second;
        maxSNRCell = cellIter->first;
      }
      //disabled to reduce console output
      //NS_LOG_UNCOND("IMSI: " << imsiIter->first << "\t" << Simulator::Now().GetSeconds() << "\t" << cellIter->first << "\t" <<  cellIter->second);
      //NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "\t" << cellIter->first << "\t" <<  cellIter->second);
    }
    //NS_LOG_UNCOND("IMSI: " << imsiIter->first << "\t max. SNR in Cell: " << maxSNRCell << "\t" <<  maxSNR); disabled to reduce console output
  }
	if(m_firstReport) //Only ran once
	{
		m_firstReport = false;
		NS_LOG_INFO("Handover Mode: Dynamic TTT");
		Simulator::Schedule(MilliSeconds(100), &LteEnbRrc::TriggerUeAssociationUpdate, this);
	}
}

// This method receives the InformLteCoordinatorParams from gNBs and sets the current serving cell for imsi.
// This happens after the IA sweep when UE gets attached to a gNB. gNB sends the params
void
LteEnbRrc::DoRecvInformLteCoordinator (EpcX2SapUser::InformLteCoordinatorParams params)
{
  NS_LOG_FUNCTION(this);
  NS_LOG_LOGIC("Recv Information about UE from cell " << params.sourceCellId);

  uint16_t mmWaveCellId = params.sourceCellId;
  uint64_t ueImsi = params.ueImsi;
  uint16_t ueRnti = params.ueRnti;
  uint16_t oldCellId = 0;

  if (m_lastMmWaveCell.find(ueImsi) != m_lastMmWaveCell.end())
  {
    oldCellId = m_lastMmWaveCell[ueImsi];
  }

  if (oldCellId != 0 && oldCellId != mmWaveCellId && !params.hoCompletedFlag)
  {
    m_numOfUesPerGnb[m_lastMmWaveCell[ueImsi]].first -= 1;
    m_numOfUesPerGnb[m_lastMmWaveCell[ueImsi]].second = false;
    NS_ASSERT(m_numOfUesPerGnb[m_lastMmWaveCell[ueImsi]].first >= 0);
  }

  m_lastMmWaveCell[ueImsi] = mmWaveCellId;
  RegisterImsiToRnti(ueImsi, ueRnti);
  m_mmWaveCellSetupCompleted[ueImsi] = true;

  if (m_rlmMultiGnb && params.hoCompletedFlag)
  {
    if (m_imsiCellSinrMap.size() > 0 && m_imsiCellCSIRSSinrMap.size() > 0)
    {
      for (uint8_t cellIndex = 1; cellIndex <= m_imsiCellSinrMap.at(ueImsi).size(); cellIndex++)
      {
        auto it = std::find_if(m_imsiCellCSIRSSinrMap.at(ueImsi).begin(), m_imsiCellCSIRSSinrMap.at(ueImsi).end(), [&cellIndex](const std::pair<double, uint8_t> &element)
                               { return element.second == cellIndex; });

        if (it != m_imsiCellCSIRSSinrMap.at(ueImsi).end())
        {
          // this is used as a signal to not serving but CSI-RS sending gNBs
          // enables transmission of CSI-RS
          EpcX2SapProvider::SetCSIRSFlagParams params;
          params.imsi = ueImsi;
          params.flag = false;
          params.cellId = cellIndex;
          m_x2SapProvider->SendSetCSIRSFlag(params);
        }
      }
    }

    if (m_numOfUesPerGnb.find(mmWaveCellId) == m_numOfUesPerGnb.end())
    {
      m_numOfUesPerGnb.insert(std::pair<uint8_t, std::pair<uint8_t, bool>>(mmWaveCellId, {1, false}));
    }
    else
    {
      m_numOfUesPerGnb[mmWaveCellId].first += 1;
      m_numOfUesPerGnb[mmWaveCellId].second = false;
    }
  }
}

void
LteEnbRrc::TriggerUeAssociationUpdate()
{
  //m_imsiCellSinrMap.size() will be 1 for simulation with 1 UE
  if(m_imsiCellSinrMap.size() > 0) // there are some entries
  {
    // Iterate over all IMSIs
    for(std::map<uint64_t, CellSinrMap>::iterator imsiIter = m_imsiCellSinrMap.begin(); imsiIter != m_imsiCellSinrMap.end(); ++imsiIter)
    {
      uint64_t imsi = imsiIter->first;
      long double maxSinr = 0;
      long double currentSinr = 0;
      uint16_t maxSinrCellId = 0;
      Ptr<UeManager> ueMan;

      // Iterate over oall received CSI RS signals for the given IMSI
      for(CellSinrMap::iterator cellIter = imsiIter->second.begin(); cellIter != imsiIter->second.end(); ++cellIter)
      {
        NS_LOG_INFO("Cell " << cellIter->first << " reports " << 10*std::log10(cellIter->second));
        if(cellIter->second > maxSinr)
        {
          // max values derived from the CSI RS feedback
          maxSinr = cellIter->second;
          maxSinrCellId = cellIter->first;
        }
        if(m_lastMmWaveCell[imsi] == cellIter->first)
        {
          // SINR from the current serving cell
          currentSinr = cellIter->second;
        }
      }

      long double sinrDifference = std::abs(10*(std::log10((long double)maxSinr) - std::log10((long double)currentSinr)));
      long double maxSinrDb = 10*std::log10((long double)maxSinr);

      if (!m_useREMLAB)
      {
        TttBasedHandover(imsiIter, sinrDifference, maxSinrCellId, maxSinrDb);
      }
    }
  }

  Simulator::Schedule(MicroSeconds(m_crtPeriod), &LteEnbRrc::TriggerUeAssociationUpdate, this);
}

// Still uses sweeps
// This is only executed on the CO
void
LteEnbRrc::TttBasedHandover(std::map<uint64_t, CellSinrMap>::iterator imsiIter, double sinrDifference, uint16_t maxSinrCellId, double maxSinrDb)
{
  
  // std::cout << GetCellId() << ": LteEnbRrc::TttBasedHandover is being executed" << std::endl;
  // std::cout << " - state of m_lastMmWaveCell (imsi and associated gNB):" << std::endl;
  // for (auto imsiCell : m_lastMmWaveCell)
  // {
  //   std::cout << "   - "<< imsiCell.first << " : " << imsiCell.second << std::endl;
  // }
  uint64_t imsi = imsiIter->first;
  bool alreadyAssociatedImsi = false;
  bool onHandoverImsi = true;
  m_outageThreshold = false;
  bool handoverWithoutBeamSweep = false;
  bool isLoadBalancing = false;
  uint8_t busyCount = 3; // hardcoded right now. May be changed when throughput issue is fixed.

  // On RecvRrcConnectionRequesTttBasedt for a new RNTI, the Lte Enb RRC stores the imsi
  // of the UE and insert a new false entry in m_mmWaveCellSetupCompleted.
  // After the first connection to a MmWave eNB, the entry becomes true.
  // When an handover between MmWave cells is triggered, it is set to false.
  if(m_mmWaveCellSetupCompleted.find(imsi) != m_mmWaveCellSetupCompleted.end())
  {
    alreadyAssociatedImsi = true;
    //onHandoverImsi = (!m_switchEnabled) ? true : !m_mmWaveCellSetupCompleted.find(imsi)->second;
    onHandoverImsi = !m_mmWaveCellSetupCompleted.find(imsi)->second;
  }
  else
  {
    alreadyAssociatedImsi = false;
    onHandoverImsi = true;
  }
  NS_LOG_INFO("TttBasedHandover: alreadyAssociatedImsi " << alreadyAssociatedImsi << " onHandoverImsi " << onHandoverImsi);
  bool handoverNeeded = false;

  double currentSinrDb = 0;
  if(m_lastMmWaveCell.find(imsi) != m_lastMmWaveCell.end())
  {
    currentSinrDb = 10*std::log10(m_imsiCellSinrMap.find(imsi)->second[m_lastMmWaveCell[imsi]]);
    // std::cout << " - current SINR dB for imsi " << imsi << " : " << currentSinrDb << std::endl; 
  }

  // If imsi is already connected to a gNB ad it is not currently handing over
  if(alreadyAssociatedImsi && !onHandoverImsi)
  {
    // set vars as if the current cell has best SNR
    bool maxSNRHigherThanCSIRS = true;
    std::pair<uint16_t, bool> bestServingCell = {m_lastMmWaveCell[imsi], false};
    std::pair<uint16_t, bool> bestNonServingCell = {0, false};
    std::pair<uint16_t, bool> bestNonBusyCell = {0, false};

    // if gNB is serving more than 3 UEs
    bool servingCellIsBusy = m_numOfUesPerGnb[m_lastMmWaveCell[imsi]].first > busyCount;

    if (m_rlmOn)
    {
      // If we have CSI RS SINR information for this IMSI
      if (m_imsiCellCSIRSSinrMap.find(imsi) != m_imsiCellCSIRSSinrMap.end())
      {
        for (size_t n = 0; n < m_imsiCellCSIRSSinrMap.at(imsi).size(); n++)
        {
          // If last serving gNB is still not the best one (aka there is one better than the serving one)
          // and CSI RS from the own serving cell is received
          if (!bestServingCell.second && m_imsiCellCSIRSSinrMap.at(imsi).at(n).second == m_lastMmWaveCell[imsi])
          {
            bestServingCell.second = true; // a beam from the currently serving cell is among the monitored beams
            if ((10 * log10(m_imsiCellCSIRSSinrMap.at(imsi).at(n).first)) > m_maxRateThreshold || maxSinrDb < (10 * log10(m_imsiCellCSIRSSinrMap.at(imsi).at(n).first)))
            {
              // one of the monitored beams from the current serving gNB is above m_maxRateThreshold, no handover is required
              // -> might be adapted depending on the following if statements
              maxSNRHigherThanCSIRS = false;
            }
          }
          // if this non serving cell is not the best one
          // and current CSI RS report is not from the last serving cell
          if (!bestNonServingCell.second && (m_imsiCellCSIRSSinrMap.at(imsi).at(n).second != m_lastMmWaveCell[imsi]))
          {
            // check if this CSI RS report is better than max rate threshold
            if ((10 * log10(m_imsiCellCSIRSSinrMap.at(imsi).at(n).first)) > m_maxRateThreshold)
            {
              // only enable switching to that beam if it can provide a good enough beam (above m_maxRateThreshold)
              bestNonServingCell.first = m_imsiCellCSIRSSinrMap.at(imsi).at(n).second;
              bestNonServingCell.second = true; // once this is set, no other of the monitored beams should be better besides the serving one!
              // It was betterm so, we save this info in the bestNonServingCell
            }
          }

          bool possibleTargetCellIsBusy;
          // Check if the cell that delivered the current CSI RS report can already serves max amount of UEs
          if (m_numOfUesPerGnb.find(m_imsiCellCSIRSSinrMap.at(imsi).at(n).second) != m_numOfUesPerGnb.end())
          {
            possibleTargetCellIsBusy = m_numOfUesPerGnb.at(m_imsiCellCSIRSSinrMap.at(imsi).at(n).second).first >= busyCount;
          }
          else
          {
            possibleTargetCellIsBusy = false;
            m_numOfUesPerGnb.insert(std::pair<uint8_t, std::pair<uint8_t, bool>>(m_imsiCellCSIRSSinrMap.at(imsi).at(n).second, {0, false}));
          }

          if (!bestNonBusyCell.second && !possibleTargetCellIsBusy)
          {
            // Why is this block repeated here?
            if ((10 * log10(m_imsiCellCSIRSSinrMap.at(imsi).at(n).first)) > m_maxRateThreshold)
            {
              // check busy cells here
              bestNonBusyCell.first = m_imsiCellCSIRSSinrMap.at(imsi).at(n).second;
              bestNonBusyCell.second = true; // once this is set, no other of the monitored beams from non-busy gNBs should be better!
            }
          }
        }
      }

      // Essentialy tells whether some other monitored cell is better
      if (maxSNRHigherThanCSIRS && bestNonServingCell.second && (maxSinrDb - sinrDifference < m_maxRateThreshold))
      {
        // current SNR to the serving gNB is below m_maxRateThreshold
        // and all beams from the serving gNB are below m_maxRateThreshold
        // we have a good beam from another gNB
        // disable beam sweep and initiate handover later
        maxSNRHigherThanCSIRS = false;
        handoverWithoutBeamSweep = true;
      }
    }

    // the UE is connected to a mmWave eNB which was not in outage
    // check if there are HO events pending
    HandoverEventMap::iterator handoverEvent = m_imsiHandoverEventsMap.find(imsi);
    if(handoverEvent != m_imsiHandoverEventsMap.end())
    {
      // an handover event is already scheduled
      if (!handoverEvent->second.isLoadBalancing)
      {
        // HO event is not for load balancing
        // check if we need to modify that as the target cell can be busy
        if (m_rlmMultiGnb && m_loadBalancing && bestNonBusyCell.second)
        {
          // only allow load balancing for the UE if a non-busy gNB is available
          bool targetCellIsBusy = false;
          if (m_numOfUesPerGnb.find(handoverEvent->second.targetCellId) != m_numOfUesPerGnb.end())
          {
            targetCellIsBusy = m_numOfUesPerGnb[handoverEvent->second.targetCellId].first >= busyCount;
          }
          else
          {
            targetCellIsBusy = false;
            m_numOfUesPerGnb.insert(std::pair<uint8_t, std::pair<uint8_t, bool>>(handoverEvent->second.targetCellId, {0, false}));
          }

          if (targetCellIsBusy && m_numOfUesPerGnb[bestNonBusyCell.first].second == false && m_numOfUesPerGnb[m_lastMmWaveCell[imsi]].second == false)
          {
            // The serving/target gNB is busy. Check if a load-balancing HO can be scheduled.
            // Only allow a handover to the new cell if no other load balancing HO is scheduled to that target cell.
            NS_LOG_UNCOND("----- IMSI: " << imsi << " scheduled handover is modified for load-balancing. Target cell is now: " << bestNonBusyCell.first);

            handoverEvent->second.scheduledHandoverEvent.Cancel();

            if (m_lastMmWaveCell[imsi] == bestNonBusyCell.first)
            {
              NS_LOG_UNCOND("----- IMSI: " << imsi << " scheduled handover was modified. Target cell equals serving cell, delete handover");
              m_imsiHandoverEventsMap.erase(handoverEvent);

              isLoadBalancing = true;
              handoverNeeded = false;

              RadioLinkMonitoringTraceParams rlmParams;
              rlmParams.imsi = imsi;
              rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::HANDOVER_CANCELLED;
              m_radioLinkMonitoringTrace(rlmParams);
            }
            else
            {
              m_numOfUesPerGnb[bestNonBusyCell.first].second = true;
              m_numOfUesPerGnb[m_lastMmWaveCell[imsi]].second = true;
              maxSinrCellId = bestNonBusyCell.first;
              handoverNeeded = true;
              isLoadBalancing = true;

              RadioLinkMonitoringTraceParams rlmParams;
              rlmParams.imsi = imsi;
              rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::HANDOVER_LOAD_BALANCING;
              rlmParams.sourceCellId = m_lastMmWaveCell[imsi];
              rlmParams.targetCellId = maxSinrCellId;
              m_radioLinkMonitoringTrace(rlmParams);
            }
          }
        }
      }
      else
      {
        isLoadBalancing = true;
      }

      if (!isLoadBalancing)
      {
        if (handoverEvent->second.isWithoutSweep)
        {
          // we can alternate without-sweep handovers but only with information from CSI-RS
          maxSinrCellId = bestNonServingCell.first;

          if (maxSinrCellId != 0)
          {
            maxSinrDb = 10 * std::log10(m_imsiCellSinrMap[imsi].at(maxSinrCellId));

          }
          else
          {
            maxSinrDb = 0;
          }
          //maxSinrDb = 10 * std::log10(m_imsiCellSinrMap[imsi].at(maxSinrCellId)); // FIXME: getting out_of_range with REMLAB here?
          // the probelm is taht we try to fetch info about cell 0, which does not exist?
          // (this=0x555555862da0, imsiIter={...}, sinrDifference=32.99748268554918, maxSinrCellId=0, maxSinrDb=46.47065868027147) at ../src/lte/model/lte-enb-rrc.cc:2897 
          // For every imsi a "std::map<uint16_t, double> CellSinrMap". Aka for every imsi a list of SNR from list of cells
          // LteEnbRrc::TttBasedHandover: m_imsiCellSinrMap state is 
          // * imsi: 1
          //   * cell: 1 sinr: 1829.57
          //   * cell: 2 sinr: 6.51679e-12
          //   * cell: 3 sinr: 6.51679e-12
          //   * cell: 4 sinr: 6.51679e-12
          //   * cell: 5 sinr: 6.51679e-12
          //   * cell: 6 sinr: 22.2494
          //   * cell: 7 sinr: 44367.6
          //   * cell: 8 sinr: 6.51679e-12
          //   * cell: 9 sinr: 6.51679e-12
          //   * cell: 10 sinr: 6.51679e-12

          sinrDifference = maxSinrDb - 10 * std::log10(m_imsiCellSinrMap[imsi].at(m_lastMmWaveCell[imsi]));
        }

        // we should not modify load balancing handovers for now
        // check if the cell to which the handover should happen is maxSinrCellId
        if (handoverEvent->second.targetCellId == maxSinrCellId)
        {
          if (currentSinrDb < m_outageThreshold && !m_rlmMultiGnb)
          {
            // handover is pushed out until UE initiates beam sweep. Why is the RRC not initiating a sweep?
            handoverEvent->second.scheduledHandoverEvent.Cancel();
            handoverNeeded = true;
            NS_LOG_INFO("------ Handover was already scheduled, but the current cell is in outage, thus HO to " << maxSinrCellId);
            m_outageAndHandover = true;
          }
          else
          {
            // TODO consider if TTT must be updated or if it can remain as computed before
            // we should compute the new TTT: if Now() + TTT < scheduledTime then update!
            uint8_t newTtt = ComputeTtt(sinrDifference);
            uint64_t handoverHappensAtTime = handoverEvent->second.scheduledHandoverEvent.GetTs(); // in nanoseconds
            NS_LOG_INFO("Scheduled for " << handoverHappensAtTime << " while now the scheduler would give " << Simulator::Now().GetMilliSeconds() + newTtt);
            // If the new TTT will cause a handover earlier that what was scheduled before, cancel it and say that we need a handover
            if (Simulator::Now().GetMilliSeconds() + newTtt < (double)handoverHappensAtTime / 1e6)
            {
              handoverEvent->second.scheduledHandoverEvent.Cancel();
              NS_LOG_INFO("------ Handover remains scheduled for " << maxSinrCellId << " but a new shorter TTT is computed");
              handoverNeeded = true;
            }
          }
        }
        else
        {
          uint16_t targetCellId = handoverEvent->second.targetCellId;
          NS_LOG_INFO("------ Handover was scheduled for " << handoverEvent->second.targetCellId << " but now maxSinrCellId is " << maxSinrCellId);
          //  get the SINR for the scheduled targetCellId: if the diff is smaller than 3 dB handover anyway
          double originalTargetSinrDb = 10 * std::log10(m_imsiCellSinrMap.find(imsi)->second[targetCellId]);
          if (maxSinrDb - originalTargetSinrDb > m_sinrThresholdDifference) // this parameter is the same as the one for ThresholdBasedSecondaryCellHandover
          {
            // delete this event, as it will result in lower SINR than what we have found from CSI RS
            handoverEvent->second.scheduledHandoverEvent.Cancel();
            // we need to re-compute the TTT and schedule a new event
            if (maxSinrCellId != m_lastMmWaveCell[imsi])
            {
              handoverNeeded = true;

              RadioLinkMonitoringTraceParams rlmParams;
              rlmParams.imsi = imsi;
              rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::HANDOVER_ADAPTION;
              rlmParams.sourceCellId = m_lastMmWaveCell[imsi];
              rlmParams.targetCellId = maxSinrCellId;
              m_radioLinkMonitoringTrace(rlmParams);
            }
            else
            {
              m_imsiHandoverEventsMap.erase(handoverEvent);

              RadioLinkMonitoringTraceParams rlmParams;
              rlmParams.imsi = imsi;
              rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::HANDOVER_CANCELLED;
              m_radioLinkMonitoringTrace(rlmParams);
            }
          }
          else
          {
            if (maxSinrCellId == m_lastMmWaveCell[imsi])
            {
              // delete this event
              NS_LOG_INFO("-------------- The difference between the two mmWave SINR is smaller than "
                          << m_sinrThresholdDifference << " dB, but the new max is the current cell, thus cancel the handover");
              handoverEvent->second.scheduledHandoverEvent.Cancel();
              m_imsiHandoverEventsMap.erase(handoverEvent);

              RadioLinkMonitoringTraceParams rlmParams;
              rlmParams.imsi = imsi;
              rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::HANDOVER_CANCELLED;
              m_radioLinkMonitoringTrace(rlmParams);
            }
            else
            {
              NS_LOG_INFO("-------------- The difference between the two mmWave SINR is smaller than "
                          << m_sinrThresholdDifference << " dB, do not cancel the handover");
            }
          }
        }
      }
    }
    else
    {
      uint8_t maxSweepSnrCellId;
      double currSNR = 0.0;
      double maxSNR = 0.0;

      // if the cell is not served by the best SINR cell
      if (maxSinrCellId != m_lastMmWaveCell[imsi] && maxSinrCellId != 0) // my: && maxSinrCellId != 0 added to avoid potential problems
      {
        // if serving cell beams are better than CSI RS
        // and SINR diff not god enough
        if (maxSNRHigherThanCSIRS && (maxSinrDb - sinrDifference < m_maxRateThreshold))
        {
          // If we are still sweeping for IMSI or if if still sweeps
          if (!m_beamSweepCompleted.at(imsi))
          {
            // If the sweep was not started
            if (!m_beamSweepStarted.at(imsi))
            {
              NS_LOG_UNCOND ("IMSI " << imsi << ": SNR below max-rate threshold. No better beam via CSI-RS known. Perform beam sweep");

              BeamSweepTraceParams params;
              params.imsi = imsi;
              params.m_beamSweepOrigin = BeamSweepTraceParams::COORDINATOR_INITIATED_HANDOVER;
              params.currentCell = m_lastMmWaveCell[imsi];
              params.snrBeforeSweep = currentSinrDb;
              params.snrDiffBeforeHO = sinrDifference;
              params.maxCellBeforeHandover = maxSinrCellId;
              m_beamSweepInitiateFromCoordinatorTrace (params);

              // This sends a signal to the UE directly from CO over LTE
              m_beamSweepTriggered = Simulator::ScheduleNow (
                      &LteEnbRrc::InitiateBeamSweepFromRrc, this, currentSinrDb, imsi);
              m_beamSweepStarted.at(imsi) = true;

              // This is only setting m_csiRSFlag in NrGnbMac and no deregistration is performed
              // SNR sufficient to receive data during exhaustive beam search
              LteRrcSap::DeRegisterUeContext deregisterParams;
              deregisterParams.imsi = imsi;
              deregisterParams.cellId = m_lastMmWaveCell[imsi];
              deregisterParams.m_sourceOfCommand = LteRrcSap::DeRegisterUeContext::SourceOfCommand::DeRegisterFromCoordinator;
              DoRecvDeRegisterUeCommand (deregisterParams);
            }
          }
          // We are not sweeping for this IMSI or it does not sweep
          else
          {
            // Seacrh for the max SNR after sweep
            for (auto const& iter : m_imsiBeamSweepCellSinrMap[imsi])
            {
              currSNR = iter.second;
              if (currSNR > maxSNR)
              {
                maxSNR = currSNR;
                maxSweepSnrCellId = iter.first;
              }
              m_imsiCellSinrMap[imsi].at(iter.first) = iter.second; // is that needed?
            }

            NS_LOG_UNCOND ("IMSI " << imsi << ": cell ID that has the maximum SNR after beam sweep" <<
                          maxSweepSnrCellId);

            // If for the current IMSI the SNR at its best cell is larger than 5
            if (m_imsiBeamSweepCellSinrMap[imsi].at(maxSweepSnrCellId) > 5)
            {
              if (maxSweepSnrCellId == maxSinrCellId)
              {
                NS_LOG_UNCOND("----- Handover needed from cell " << m_lastMmWaveCell[imsi] << " to " << maxSinrCellId);
                handoverNeeded = true;
              }
              else if (maxSweepSnrCellId == m_lastMmWaveCell[imsi])
              {
                NS_LOG_UNCOND("----- Handover was needed from cell " << m_lastMmWaveCell[imsi] << " to " << maxSinrCellId 
                  << ", but it is no longer needed due to renewed beam sweep");
                handoverNeeded = false;
              }
              else
              {
                NS_LOG_UNCOND("----- Handover was needed from cell " << m_lastMmWaveCell[imsi] << " to " << maxSinrCellId 
                  << ", but target cell Id was changed to " << maxSweepSnrCellId << "as a result of the renewed beam sweep");
                maxSinrCellId = maxSweepSnrCellId;
                handoverNeeded = true;
              } 
                
              // IMSI is not sweeping 
              SetSweepParameters (imsi, false, false);

              if (handoverNeeded)
              {
                RadioLinkMonitoringTraceParams rlmParams;
                rlmParams.imsi = imsi;
                rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::HANDOVER_AFTER_RRC_SWEEP;
                rlmParams.sourceCellId = m_lastMmWaveCell[imsi];
                rlmParams.targetCellId = maxSinrCellId;
                m_radioLinkMonitoringTrace(rlmParams);
              }
              else
              {
                RadioLinkMonitoringTraceParams rlmParams;
                rlmParams.imsi = imsi;
                rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::NO_HANDOVER_AFTER_RRC_SWEEP;
                m_radioLinkMonitoringTrace(rlmParams);
              }
            }
            else
            {
              // IMSI is not sweeping 
              SetSweepParameters (imsi, false, false);
            }                   
          }
        }
        else if (m_rlmMultiGnb && handoverWithoutBeamSweep)
        {
          NS_LOG_UNCOND("----- IMSI: " << imsi << " SNR below beam sweep threshold for cell " << m_lastMmWaveCell[imsi]
                                       << ". Better cell: " << bestNonServingCell.first << " already known via CSI-RS. Initiate handover!");
          NS_ASSERT(bestNonServingCell.second == true);
          maxSinrCellId = bestNonServingCell.first;
          handoverNeeded = true;

          RadioLinkMonitoringTraceParams rlmParams;
          rlmParams.imsi = imsi;
          rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::HANDOVER_WITHOUT_SWEEP;
          rlmParams.sourceCellId = m_lastMmWaveCell[imsi];
          rlmParams.targetCellId = maxSinrCellId;
          m_radioLinkMonitoringTrace(rlmParams);
        }
        else
        {
          handoverNeeded = false;
        }
      }

      if (m_rlmMultiGnb && m_loadBalancing && !handoverNeeded)
      {
        if (servingCellIsBusy && bestNonBusyCell.second)
        {
          // A handover was not required but the serving cell is busy.
          // Another cell is available via CSI-RS RLM.
          if (m_numOfUesPerGnb[bestNonBusyCell.first].second == false && m_numOfUesPerGnb[m_lastMmWaveCell[imsi]].second == false)
          {
            // Only allow a handover to the new cell if no other load balancing HO is scheduled to that target cell.
            NS_LOG_UNCOND("----- IMSI: " << imsi << " load-balancing handover is scheduled. Target cell is now: " << bestNonBusyCell.first);
            m_numOfUesPerGnb[bestNonBusyCell.first].second = true;
            m_numOfUesPerGnb[m_lastMmWaveCell[imsi]].second = true;
            maxSinrCellId = bestNonBusyCell.first;
            handoverNeeded = true;
            isLoadBalancing = true;

            RadioLinkMonitoringTraceParams rlmParams;
            rlmParams.imsi = imsi;
            rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::HANDOVER_LOAD_BALANCING;
            rlmParams.sourceCellId = m_lastMmWaveCell[imsi];
            rlmParams.targetCellId = maxSinrCellId;
            m_radioLinkMonitoringTrace(rlmParams);
          }
        }
      }
    }
  }

  if(handoverNeeded)
  {
    // compute the TTT
    uint8_t millisecondsToHandover = ComputeTtt(sinrDifference);
    NS_LOG_DEBUG("The sinrDifference is " << sinrDifference << " between CellId "<<m_lastMmWaveCell[imsi]<<" and "<<maxSinrCellId<< " and the TTT computed is " << (uint32_t)millisecondsToHandover
      << " ms, thus the event will happen at time " << Simulator::Now().GetMilliSeconds() + millisecondsToHandover);

    if (isLoadBalancing)
    {
      NS_LOG_DEBUG("For loadbalancing purposes, handover has been modified with the shortest possible TTT (25ms)");
      millisecondsToHandover = m_minDynTttValue;
    }

    EventId scheduledHandoverEvent = Simulator::Schedule(MilliSeconds(millisecondsToHandover), &LteEnbRrc::PerformHandover, this, imsi);
    LteEnbRrc::HandoverEventInfo handoverInfo;
    handoverInfo.sourceCellId = m_lastMmWaveCell[imsi];
    handoverInfo.targetCellId = maxSinrCellId;
    handoverInfo.isLoadBalancing = isLoadBalancing;
    handoverInfo.isWithoutSweep = handoverWithoutBeamSweep;
    handoverInfo.scheduledHandoverEvent = scheduledHandoverEvent;
    HandoverEventMap::iterator handoverEvent = m_imsiHandoverEventsMap.find(imsi);
    if(handoverEvent != m_imsiHandoverEventsMap.end()) // another event was scheduled, but it was already deleted. Replace the entry
    {
      handoverEvent->second = handoverInfo;
    }
    else
    {
      m_imsiHandoverEventsMap.insert(std::pair<uint64_t, HandoverEventInfo> (imsi, handoverInfo));
    }
  }
}

void
LteEnbRrc::PerformHandover(uint64_t imsi)
{
  NS_ASSERT_MSG(m_imsiHandoverEventsMap.find(imsi) != m_imsiHandoverEventsMap.end(), "No handover event for this imsi!");
  LteEnbRrc::HandoverEventInfo handoverInfo = m_imsiHandoverEventsMap.find(imsi)->second;
  NS_ASSERT_MSG(handoverInfo.sourceCellId == m_lastMmWaveCell[imsi], "The secondary cell to which the UE is attached has changed handoverInfo.sourceCellId "
    << handoverInfo.sourceCellId << " m_lastMmWaveCell[imsi] " << m_lastMmWaveCell[imsi] << " imsi " << imsi);

  bool alreadyAssociatedImsi = false;

  if(m_mmWaveCellSetupCompleted.find(imsi) != m_mmWaveCellSetupCompleted.end () )
  {
    alreadyAssociatedImsi = true;
  }

  if (alreadyAssociatedImsi)
  {
    // The new secondary cell HO procedure does not require to switch to LTE
    NS_LOG_INFO("PerformHandover ----- handover from " << m_lastMmWaveCell[imsi] << " to " << handoverInfo.targetCellId << " at time " << Simulator::Now().GetSeconds());

    // trigger ho via X2
    EpcX2SapProvider::SecondaryHandoverParams params;
    params.imsi = imsi;
    params.targetCellId = handoverInfo.targetCellId;
    params.oldCellId = handoverInfo.sourceCellId;
    m_x2SapProvider->SendCoordinatorHandoverRequest(params);

    RadioLinkMonitoringTraceParams rlmParams;
    rlmParams.imsi = imsi;
    rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::HANDOVER_EXECUTED;
    rlmParams.sourceCellId = handoverInfo.sourceCellId;
    rlmParams.targetCellId = handoverInfo.targetCellId;
    m_radioLinkMonitoringTrace(rlmParams);

    m_mmWaveCellSetupCompleted[imsi] = false;

    if (m_rlmMultiGnb)
    {
      for (uint8_t cellIndex = 1; cellIndex <= m_imsiCellSinrMap.at(imsi).size(); cellIndex++)
      {
        if (cellIndex == handoverInfo.sourceCellId)
        {
          // previously serving cell is handled elsewhere
          continue;
        }
        auto it = std::find_if(m_imsiCellCSIRSSinrMap.at(imsi).begin(), m_imsiCellCSIRSSinrMap.at(imsi).end(), [&cellIndex](const std::pair<double, uint8_t> &element)
                               { return element.second == cellIndex; });

        if (it != m_imsiCellCSIRSSinrMap.at(imsi).end())
        {
          // this is used as a signal to not serving but CSI-RS sending gNBs
          // disables transmission of CSI-RS
          EpcX2SapProvider::SetCSIRSFlagParams params;
          params.imsi = imsi;
          params.flag = true;
          params.cellId = cellIndex;
          m_x2SapProvider->SendSetCSIRSFlag(params);
        }
      }
    }
  }
  else
  {
    NS_LOG_INFO("## Warn: handover not triggered because the UE is not associated yet!");
  }

  // remove the HandoverEvent from the map
  m_imsiHandoverEventsMap.erase(m_imsiHandoverEventsMap.find(imsi));
  SetSweepParameters (imsi, false, false);
}

void
LteEnbRrc::DoRecvCoordinatorHandoverRequest(EpcX2SapUser::SecondaryHandoverParams params)
{
  m_cmacSapProvider.at(0)->SetCSIRSFlag (params.imsi, true);
  if (params.targetCellId == 0)
  {
    // this is used for multi-gNB RLM to stop CSI-RS transmission of non-serving gNBs
    NS_FATAL_ERROR ("Is disabled in PerformHandover");
    NS_ASSERT(m_rlmMultiGnb);
    return;
  }
  uint16_t rnti = DoGetRntiFromImsi(params.imsi);
  NS_LOG_LOGIC("Rnti " << rnti);
  SendHandoverRequest(rnti, params.targetCellId);
}

uint16_t
LteEnbRrc::DoGetRntiFromImsi(uint64_t imsi)
{
  if(m_imsiRntiMap.find(imsi) != m_imsiRntiMap.end())
  {
    return m_imsiRntiMap.find(imsi)->second;
  }
  else
  {
    return 0;
  }
}

uint64_t
LteEnbRrc::DoGetImsiFromRnti (uint16_t rnti)
{
  for (std::map<uint64_t, uint16_t>::iterator imsiRntiIterator = m_imsiRntiMap.begin (); 
       imsiRntiIterator != m_imsiRntiMap.end ();
       ++imsiRntiIterator)
  {
    if (imsiRntiIterator->second == rnti)
    {
      return imsiRntiIterator->first;
    }
  }

  return 0;
}

void
LteEnbRrc::RegisterImsiToRnti(uint64_t imsi, uint16_t rnti)
{
  if(m_imsiRntiMap.find(imsi) == m_imsiRntiMap.end())
  {
    m_imsiRntiMap.insert(std::pair<uint64_t, uint16_t> (imsi, rnti));
  }
  else
  {
    m_imsiRntiMap.find(imsi)->second = rnti;
  }
}

//****************************************************************************************************
uint8_t
LteEnbRrc::CellToComponentCarrierId (uint16_t cellId)
{
  NS_LOG_FUNCTION (this << cellId);
  for (auto &it: m_componentCarrierPhyConf)
    {
      if (it.second->GetCellId () == cellId)
        {
          return it.first;
        }
    }
  NS_FATAL_ERROR ("Cell " << cellId << " not found in CC map");
}

uint16_t
LteEnbRrc::ComponentCarrierToCellId (uint8_t componentCarrierId)
{
  NS_LOG_FUNCTION (this << +componentCarrierId);
  return m_componentCarrierPhyConf.at (componentCarrierId)->GetCellId ();
}

bool
LteEnbRrc::SendData (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet);

  EpsBearerTag tag;
  bool found = packet->RemovePacketTag (tag);
  NS_ASSERT_MSG (found, "no EpsBearerTag found in packet to be sent");
  Ptr<UeManager> ueManager = GetUeManager (tag.GetRnti ());
  ueManager->SendData (tag.GetBid (), packet);

  return true;
}

void 
LteEnbRrc::SetForwardUpCallback (Callback <void, Ptr<Packet> > cb)
{
  m_forwardUpCallback = cb;
}

void
LteEnbRrc::ConnectionRequestTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  NS_ASSERT_MSG (GetUeManager (rnti)->GetState () == UeManager::INITIAL_RANDOM_ACCESS,
                 "ConnectionRequestTimeout in unexpected state " << ToString (GetUeManager (rnti)->GetState ()));
  m_rrcTimeoutTrace (GetUeManager (rnti)->GetImsi (), rnti,
                     ComponentCarrierToCellId (GetUeManager (rnti)->GetComponentCarrierId ()), "ConnectionRequestTimeout");
  RemoveUe (rnti);
}

void
LteEnbRrc::ConnectionSetupTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  NS_ASSERT_MSG (GetUeManager (rnti)->GetState () == UeManager::CONNECTION_SETUP,
                 "ConnectionSetupTimeout in unexpected state " << ToString (GetUeManager (rnti)->GetState ()));
  m_rrcTimeoutTrace (GetUeManager (rnti)->GetImsi (), rnti,
                     ComponentCarrierToCellId (GetUeManager (rnti)->GetComponentCarrierId ()), "ConnectionSetupTimeout");
  RemoveUe (rnti);
}

void
LteEnbRrc::ConnectionRejectedTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  NS_ASSERT_MSG (GetUeManager (rnti)->GetState () == UeManager::CONNECTION_REJECTED,
                 "ConnectionRejectedTimeout in unexpected state " << ToString (GetUeManager (rnti)->GetState ()));
  m_rrcTimeoutTrace (GetUeManager (rnti)->GetImsi (), rnti,
                     ComponentCarrierToCellId (GetUeManager (rnti)->GetComponentCarrierId ()), "ConnectionRejectedTimeout");
  RemoveUe (rnti);
}

void
LteEnbRrc::HandoverJoiningTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  NS_ASSERT_MSG (GetUeManager (rnti)->GetState () == UeManager::HANDOVER_JOINING,
                 "HandoverJoiningTimeout in unexpected state " << ToString (GetUeManager (rnti)->GetState ()));
  m_rrcTimeoutTrace (GetUeManager (rnti)->GetImsi (), rnti,
                     ComponentCarrierToCellId (GetUeManager (rnti)->GetComponentCarrierId ()), "HandoverJoiningTimeout");
  RemoveUe (rnti);
}

void
LteEnbRrc::HandoverLeavingTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  NS_ASSERT_MSG (GetUeManager (rnti)->GetState () == UeManager::HANDOVER_LEAVING,
                 "HandoverLeavingTimeout in unexpected state " << ToString (GetUeManager (rnti)->GetState ()));
  m_rrcTimeoutTrace (GetUeManager (rnti)->GetImsi (), rnti,
                     ComponentCarrierToCellId (GetUeManager (rnti)->GetComponentCarrierId ()), "HandoverLeavingTimeout");
  RemoveUe (rnti);
}

void
LteEnbRrc::SendHandoverRequest (uint16_t rnti, uint16_t cellId)
{
  NS_LOG_FUNCTION (this << rnti << cellId);
  NS_LOG_LOGIC ("Request to send HANDOVER REQUEST");
  NS_ASSERT (m_configured);
  std::cout << "  * LteEnbRrc::SendHandoverRequest: rnti " << rnti << " to cell ID: " << cellId << std::endl;

  Ptr<UeManager> ueManager = GetUeManager (rnti);
  std::cout << "  ** UE manager pointer is empty?" << static_cast<bool>(ueManager == nullptr) << std::endl;
  ueManager->PrepareHandover (cellId);
 
}

void 
LteEnbRrc::DoCompleteSetupUe (uint16_t rnti, LteEnbRrcSapProvider::CompleteSetupUeParameters params)
{
  NS_LOG_FUNCTION (this << rnti);
  GetUeManager (rnti)->CompleteSetupUe (params);
}

void
LteEnbRrc::DoRecvRrcConnectionRequest (uint16_t rnti, LteRrcSap::RrcConnectionRequest msg)
{
  NS_LOG_FUNCTION (this << rnti);
  GetUeManager (rnti)->RecvRrcConnectionRequest (msg);
}

void
LteEnbRrc::DoRecvRrcConnectionSetupCompleted (uint16_t rnti, LteRrcSap::RrcConnectionSetupCompleted msg)
{
  NS_LOG_FUNCTION (this << rnti);
  GetUeManager (rnti)->RecvRrcConnectionSetupCompleted (msg);
}

void
LteEnbRrc::DoRecvRrcConnectionReconfigurationCompleted (uint16_t rnti, LteRrcSap::RrcConnectionReconfigurationCompleted msg)
{
  NS_LOG_FUNCTION (this << rnti);
  GetUeManager (rnti)->RecvRrcConnectionReconfigurationCompleted (msg);
}

void 
LteEnbRrc::DoRecvRrcConnectionReestablishmentRequest (uint16_t rnti, LteRrcSap::RrcConnectionReestablishmentRequest msg)
{
  NS_LOG_FUNCTION (this << rnti);
  GetUeManager (rnti)->RecvRrcConnectionReestablishmentRequest (msg);
}
 
void 
LteEnbRrc::DoRecvRrcConnectionReestablishmentComplete (uint16_t rnti, LteRrcSap::RrcConnectionReestablishmentComplete msg)
{
  NS_LOG_FUNCTION (this << rnti);
  GetUeManager (rnti)->RecvRrcConnectionReestablishmentComplete (msg);
}

void 
LteEnbRrc::DoRecvMeasurementReport (uint16_t rnti, LteRrcSap::MeasurementReport msg)
{
  NS_LOG_FUNCTION (this << rnti);
  GetUeManager (rnti)->RecvMeasurementReport (msg);
}

void
LteEnbRrc::DoInitialContextSetupRequest (EpcEnbS1SapUser::InitialContextSetupRequestParameters msg)
{
  NS_LOG_FUNCTION (this);
  Ptr<UeManager> ueManager = GetUeManager (msg.rnti);
  ueManager->InitialContextSetupRequest ();
}

void 
LteEnbRrc::DoRecvIdealUeContextRemoveRequest (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  GetUeManager (rnti)->RecvIdealUeContextRemoveRequest (rnti);
  //delete the UE context at the eNB
  RemoveUe (rnti);
}

void 
LteEnbRrc::DoDataRadioBearerSetupRequest (EpcEnbS1SapUser::DataRadioBearerSetupRequestParameters request)
{
  NS_LOG_FUNCTION (this);
  Ptr<UeManager> ueManager = GetUeManager (request.rnti);
  ueManager->SetupDataRadioBearer (request.bearer, request.bearerId, request.gtpTeid, request.transportLayerAddress);
}

void 
LteEnbRrc::DoPathSwitchRequestAcknowledge (EpcEnbS1SapUser::PathSwitchRequestAcknowledgeParameters params)
{
  NS_LOG_FUNCTION (this);
  Ptr<UeManager> ueManager = GetUeManager (params.rnti);
  ueManager->SendUeContextRelease ();
}

void
LteEnbRrc::DoRecvHandoverRequest (EpcX2SapUser::HandoverRequestParams req)
{
  std::cout << "  * LteEnbRrc::DoRecvHandoverRequest: received a handover request. Processing" << std::endl; 
  NS_LOG_FUNCTION (this);

  NS_LOG_LOGIC ("Recv X2 message: HANDOVER REQUEST");

  NS_LOG_LOGIC ("oldEnbUeX2apId = " << req.oldEnbUeX2apId);
  NS_LOG_LOGIC ("sourceCellId = " << req.sourceCellId);
  NS_LOG_LOGIC ("targetCellId = " << req.targetCellId);
  NS_LOG_LOGIC ("mmeUeS1apId = " << req.mmeUeS1apId);

  if (m_admitHandoverRequest == false)
    {
      NS_LOG_INFO ("rejecting handover request from cellId " << req.sourceCellId);
      EpcX2Sap::HandoverPreparationFailureParams res;
      res.oldEnbUeX2apId =  req.oldEnbUeX2apId;
      res.sourceCellId = req.sourceCellId;
      res.targetCellId = req.targetCellId;
      res.cause = 0;
      res.criticalityDiagnostics = 0;
      m_x2SapProvider->SendHandoverPreparationFailure (res);
      return;
    }

  //*******************(*************************************************************************
  m_cmacSapProvider.at (0)->SetRAProcessFlag (true);
  //*******************(*************************************************************************

  uint16_t rnti = AddUe (UeManager::HANDOVER_JOINING, CellToComponentCarrierId (req.targetCellId));
  LteEnbCmacSapProvider::AllocateNcRaPreambleReturnValue anrcrv = m_cmacSapProvider.at (0)->AllocateNcRaPreamble (rnti);
  if (anrcrv.valid == false)
    {
      NS_LOG_INFO (this << " failed to allocate a preamble for non-contention based RA => cannot accept HO");
      RemoveUe (rnti);
      NS_FATAL_ERROR ("should trigger HO Preparation Failure, but it is not implemented");
      return;
    }

  Ptr<UeManager> ueManager = GetUeManager (rnti);
  ueManager->SetSource (req.sourceCellId, req.oldEnbUeX2apId);
  ueManager->SetImsi (req.mmeUeS1apId);
  //*********************************************************************************************
  RegisterImsiToRnti(req.mmeUeS1apId, rnti);
  //*********************************************************************************************

  EpcX2SapProvider::HandoverRequestAckParams ackParams;
  ackParams.oldEnbUeX2apId = req.oldEnbUeX2apId;
  ackParams.newEnbUeX2apId = rnti;
  ackParams.sourceCellId = req.sourceCellId;
  ackParams.targetCellId = req.targetCellId;

  for (std::vector <EpcX2Sap::ErabToBeSetupItem>::iterator it = req.bearers.begin ();
       it != req.bearers.end ();
       ++it)
    {
      ueManager->SetupDataRadioBearer (it->erabLevelQosParameters, it->erabId, it->gtpTeid, it->transportLayerAddress);
      EpcX2Sap::ErabAdmittedItem i;
      i.erabId = it->erabId;
      ackParams.admittedBearers.push_back (i);
    }

  LteRrcSap::RrcConnectionReconfiguration handoverCommand = ueManager->GetRrcConnectionReconfigurationForHandover ();
  handoverCommand.haveMobilityControlInfo = true;
  handoverCommand.mobilityControlInfo.targetPhysCellId = req.targetCellId;
  handoverCommand.mobilityControlInfo.haveCarrierFreq = true;
  handoverCommand.mobilityControlInfo.carrierFreq.dlCarrierFreq = m_dlEarfcn;
  handoverCommand.mobilityControlInfo.carrierFreq.ulCarrierFreq = m_ulEarfcn;
  handoverCommand.mobilityControlInfo.haveCarrierBandwidth = true;
  handoverCommand.mobilityControlInfo.carrierBandwidth.dlBandwidth = m_dlBandwidth;
  handoverCommand.mobilityControlInfo.carrierBandwidth.ulBandwidth = m_ulBandwidth;
  handoverCommand.mobilityControlInfo.newUeIdentity = rnti;
  handoverCommand.mobilityControlInfo.haveRachConfigDedicated = true;
  handoverCommand.mobilityControlInfo.rachConfigDedicated.raPreambleIndex = anrcrv.raPreambleId;
  handoverCommand.mobilityControlInfo.rachConfigDedicated.raPrachMaskIndex = anrcrv.raPrachMaskIndex;

  LteEnbCmacSapProvider::RachConfig rc = m_cmacSapProvider.at (0)->GetRachConfig ();
  handoverCommand.mobilityControlInfo.radioResourceConfigCommon.rachConfigCommon.preambleInfo.numberOfRaPreambles = rc.numberOfRaPreambles;
  handoverCommand.mobilityControlInfo.radioResourceConfigCommon.rachConfigCommon.raSupervisionInfo.preambleTransMax = rc.preambleTransMax;
  handoverCommand.mobilityControlInfo.radioResourceConfigCommon.rachConfigCommon.raSupervisionInfo.raResponseWindowSize = rc.raResponseWindowSize;
  handoverCommand.mobilityControlInfo.radioResourceConfigCommon.rachConfigCommon.txFailParam.connEstFailCount = rc.connEstFailCount;

  Ptr<Packet> encodedHandoverCommand = m_rrcSapUser->EncodeHandoverCommand (handoverCommand);

  ackParams.rrcContext = encodedHandoverCommand;

  NS_LOG_LOGIC ("Send X2 message: HANDOVER REQUEST ACK");

  NS_LOG_LOGIC ("oldEnbUeX2apId = " << ackParams.oldEnbUeX2apId);
  NS_LOG_LOGIC ("newEnbUeX2apId = " << ackParams.newEnbUeX2apId);
  NS_LOG_LOGIC ("sourceCellId = " << ackParams.sourceCellId);
  NS_LOG_LOGIC ("targetCellId = " << ackParams.targetCellId);

  m_x2SapProvider->SendHandoverRequestAck (ackParams);
}

void
LteEnbRrc::DoRecvHandoverRequestAck (EpcX2SapUser::HandoverRequestAckParams params)
{
  NS_LOG_FUNCTION (this);

  NS_LOG_LOGIC ("Recv X2 message: HANDOVER REQUEST ACK");

  NS_LOG_LOGIC ("oldEnbUeX2apId = " << params.oldEnbUeX2apId);
  NS_LOG_LOGIC ("newEnbUeX2apId = " << params.newEnbUeX2apId);
  NS_LOG_LOGIC ("sourceCellId = " << params.sourceCellId);
  NS_LOG_LOGIC ("targetCellId = " << params.targetCellId);

  uint16_t rnti = params.oldEnbUeX2apId;
  Ptr<UeManager> ueManager = GetUeManager (rnti);
  ueManager->RecvHandoverRequestAck (params);
}

void
LteEnbRrc::DoRecvHandoverPreparationFailure (EpcX2SapUser::HandoverPreparationFailureParams params)
{
  NS_LOG_FUNCTION (this);

  NS_LOG_LOGIC ("Recv X2 message: HANDOVER PREPARATION FAILURE");

  NS_LOG_LOGIC ("oldEnbUeX2apId = " << params.oldEnbUeX2apId);
  NS_LOG_LOGIC ("sourceCellId = " << params.sourceCellId);
  NS_LOG_LOGIC ("targetCellId = " << params.targetCellId);
  NS_LOG_LOGIC ("cause = " << params.cause);
  NS_LOG_LOGIC ("criticalityDiagnostics = " << params.criticalityDiagnostics);

  uint16_t rnti = params.oldEnbUeX2apId;
  Ptr<UeManager> ueManager = GetUeManager (rnti);
  ueManager->RecvHandoverPreparationFailure (params.targetCellId);
}

void
LteEnbRrc::DoRecvSnStatusTransfer (EpcX2SapUser::SnStatusTransferParams params)
{
  NS_LOG_FUNCTION (this);

  NS_LOG_LOGIC ("Recv X2 message: SN STATUS TRANSFER");

  NS_LOG_LOGIC ("oldEnbUeX2apId = " << params.oldEnbUeX2apId);
  NS_LOG_LOGIC ("newEnbUeX2apId = " << params.newEnbUeX2apId);
  NS_LOG_LOGIC ("erabsSubjectToStatusTransferList size = " << params.erabsSubjectToStatusTransferList.size ());

  uint16_t rnti = params.newEnbUeX2apId;
  Ptr<UeManager> ueManager = GetUeManager (rnti);
  ueManager->RecvSnStatusTransfer (params);
}

void
LteEnbRrc::DoRecvUeContextRelease (EpcX2SapUser::UeContextReleaseParams params)
{
  NS_LOG_FUNCTION (this);

  NS_LOG_LOGIC ("Recv X2 message: UE CONTEXT RELEASE");

  NS_LOG_LOGIC ("oldEnbUeX2apId = " << params.oldEnbUeX2apId);
  NS_LOG_LOGIC ("newEnbUeX2apId = " << params.newEnbUeX2apId);

  uint16_t rnti = params.oldEnbUeX2apId;
  GetUeManager (rnti)->RecvUeContextRelease (params);
  RemoveUe(rnti);
}

void
LteEnbRrc::DoRecvLoadInformation (EpcX2SapUser::LoadInformationParams params)
{
  NS_LOG_FUNCTION (this);

  NS_LOG_LOGIC ("Recv X2 message: LOAD INFORMATION");

  NS_LOG_LOGIC ("Number of cellInformationItems = " << params.cellInformationList.size ());

  NS_ABORT_IF (m_ffrRrcSapProvider.size () == 0);
  m_ffrRrcSapProvider.at (0)->RecvLoadInformation (params);
}

void
LteEnbRrc::DoRecvResourceStatusUpdate (EpcX2SapUser::ResourceStatusUpdateParams params)
{
  NS_LOG_FUNCTION (this);

  NS_LOG_LOGIC ("Recv X2 message: RESOURCE STATUS UPDATE");

  NS_LOG_LOGIC ("Number of cellMeasurementResultItems = " << params.cellMeasurementResultList.size ());

  NS_ASSERT ("Processing of RESOURCE STATUS UPDATE X2 message IS NOT IMPLEMENTED");
}

void
LteEnbRrc::DoRecvUeData (EpcX2SapUser::UeDataParams params)
{
  NS_LOG_FUNCTION (this);

  NS_LOG_LOGIC ("Recv UE DATA FORWARDING through X2 interface");
  NS_LOG_LOGIC ("sourceCellId = " << params.sourceCellId);
  NS_LOG_LOGIC ("targetCellId = " << params.targetCellId);
  NS_LOG_LOGIC ("gtpTeid = " << params.gtpTeid);
  NS_LOG_LOGIC ("ueData = " << params.ueData);
  NS_LOG_LOGIC ("ueData size = " << params.ueData->GetSize ());

  std::map<uint32_t, X2uTeidInfo>::iterator 
    teidInfoIt = m_x2uTeidInfoMap.find (params.gtpTeid);
  if (teidInfoIt != m_x2uTeidInfoMap.end ())
    {
      GetUeManager (teidInfoIt->second.rnti)->SendData (teidInfoIt->second.drbid, params.ueData);
    }
  else
    {
      NS_FATAL_ERROR ("X2-U data received but no X2uTeidInfo found");
    }
}


uint16_t 
LteEnbRrc::DoAllocateTemporaryCellRnti (uint8_t componentCarrierId)
{
  NS_LOG_FUNCTION (this << +componentCarrierId);
  return AddUe (UeManager::INITIAL_RANDOM_ACCESS, componentCarrierId);
}

void
LteEnbRrc::DoRrcConfigurationUpdateInd (LteEnbCmacSapUser::UeConfig cmacParams)
{
  Ptr<UeManager> ueManager = GetUeManager (cmacParams.m_rnti);
  ueManager->CmacUeConfigUpdateInd (cmacParams);
}

void
LteEnbRrc::DoNotifyLcConfigResult (uint16_t rnti, uint8_t lcid, bool success)
{
  NS_LOG_FUNCTION (this << (uint32_t) rnti);
  NS_FATAL_ERROR ("not implemented");
}


uint8_t
LteEnbRrc::DoAddUeMeasReportConfigForHandover (LteRrcSap::ReportConfigEutra reportConfig)
{
  NS_LOG_FUNCTION (this);
  uint8_t measId = AddUeMeasReportConfig (reportConfig);
  m_handoverMeasIds.insert (measId);
  return measId;
}

uint8_t
LteEnbRrc::DoAddUeMeasReportConfigForComponentCarrier (LteRrcSap::ReportConfigEutra reportConfig)
{
  NS_LOG_FUNCTION (this);
  uint8_t measId = AddUeMeasReportConfig (reportConfig);
  m_componentCarrierMeasIds.insert (measId);
  return measId;
}

void
LteEnbRrc::DoSetNumberOfComponentCarriers (uint16_t numberOfComponentCarriers)
{
  m_numberOfComponentCarriers = numberOfComponentCarriers;
}

void
LteEnbRrc::DoTriggerHandover (uint16_t rnti, uint16_t targetCellId)
{
  NS_LOG_FUNCTION (this << rnti << targetCellId);

  bool isHandoverAllowed = true;

  Ptr<UeManager> ueManager = GetUeManager (rnti);
  NS_ASSERT_MSG (ueManager != 0, "Cannot find UE context with RNTI " << rnti);

  if (m_anrSapProvider != 0)
    {
      // ensure that proper neighbour relationship exists between source and target cells
      bool noHo = m_anrSapProvider->GetNoHo (targetCellId);
      bool noX2 = m_anrSapProvider->GetNoX2 (targetCellId);
      NS_LOG_DEBUG (this << " cellId=" << ComponentCarrierToCellId (ueManager->GetComponentCarrierId ())
                         << " targetCellId=" << targetCellId
                         << " NRT.NoHo=" << noHo << " NRT.NoX2=" << noX2);

      if (noHo || noX2)
        {
          isHandoverAllowed = false;
          NS_LOG_LOGIC (this << " handover to cell " << targetCellId
                             << " is not allowed by ANR");
        }
    }

  if (ueManager->GetState () != UeManager::CONNECTED_NORMALLY)
    {
      isHandoverAllowed = false;
      NS_LOG_LOGIC (this << " handover is not allowed because the UE"
                         << " rnti=" << rnti << " is in "
                         << ToString (ueManager->GetState ()) << " state");
    }

  if (isHandoverAllowed)
    {
      // initiate handover execution
      ueManager->PrepareHandover (targetCellId);
    }
}

uint8_t
LteEnbRrc::DoAddUeMeasReportConfigForAnr (LteRrcSap::ReportConfigEutra reportConfig)
{
  NS_LOG_FUNCTION (this);
  uint8_t measId = AddUeMeasReportConfig (reportConfig);
  m_anrMeasIds.insert (measId);
  return measId;
}

uint8_t
LteEnbRrc::DoAddUeMeasReportConfigForFfr (LteRrcSap::ReportConfigEutra reportConfig)
{
  NS_LOG_FUNCTION (this);
  uint8_t measId = AddUeMeasReportConfig (reportConfig);
  m_ffrMeasIds.insert (measId);
  return measId;
}

void
LteEnbRrc::DoSetPdschConfigDedicated (uint16_t rnti, LteRrcSap::PdschConfigDedicated pdschConfigDedicated)
{
  NS_LOG_FUNCTION (this);
  Ptr<UeManager> ueManager = GetUeManager (rnti);
  ueManager->SetPdschConfigDedicated (pdschConfigDedicated);
}

void
LteEnbRrc::DoSendLoadInformation (EpcX2Sap::LoadInformationParams params)
{
  NS_LOG_FUNCTION (this);

  m_x2SapProvider->SendLoadInformation(params);
}

uint16_t
LteEnbRrc::AddUe (UeManager::State state, uint8_t componentCarrierId)
{
  NS_LOG_FUNCTION (this);
  bool found = false;
  uint16_t rnti;
  for (rnti = m_lastAllocatedRnti + 1; 
       (rnti != m_lastAllocatedRnti - 1) && (!found);
       ++rnti)
    {
      if ((rnti != 0) && (m_ueMap.find (rnti) == m_ueMap.end ()))
        {
          found = true;
          break;
        }
    }
  NS_ASSERT_MSG (found, "no more RNTIs available (do you have more than 65535 UEs in a cell?)");
  m_lastAllocatedRnti = rnti;
  Ptr<UeManager> ueManager = CreateObject<UeManager> (this, rnti, state, componentCarrierId);
  m_ccmRrcSapProvider-> AddUe (rnti, (uint8_t)state);
  m_ueMap.insert (std::pair<uint16_t, Ptr<UeManager> > (rnti, ueManager));
  ueManager->Initialize ();
  const uint16_t cellId = ComponentCarrierToCellId (componentCarrierId);
  NS_LOG_DEBUG (this << " New UE RNTI " << rnti << " cellId " << cellId << " srs CI " << ueManager->GetSrsConfigurationIndex ());
  std::cout << "LteEnbRrc::AddUe: New UE RNTI " << rnti << " cellId " << cellId << " srs CI " << ueManager->GetSrsConfigurationIndex () << std::endl;
  m_newUeContextTrace (cellId, rnti);
  return rnti;
}

void
LteEnbRrc::RemoveUe (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << (uint32_t) rnti);
  std::map <uint16_t, Ptr<UeManager> >::iterator it = m_ueMap.find (rnti);
  NS_ASSERT_MSG (it != m_ueMap.end (), "request to remove UE info with unknown rnti " << rnti);
  uint64_t imsi = it->second->GetImsi ();
  uint16_t srsCi = (*it).second->GetSrsConfigurationIndex ();
  //cancel pending events
  it->second->CancelPendingEvents ();
  // fire trace upon connection release
  m_connectionReleaseTrace (imsi, ComponentCarrierToCellId (it->second->GetComponentCarrierId ()), rnti);
  m_ueMap.erase (it);
  for (uint8_t i = 0; i < m_numberOfComponentCarriers; i++)
    {
      m_cmacSapProvider.at (i)->RemoveUe (rnti);
      m_cphySapProvider.at (i)->RemoveUe (rnti);
    }
  if (m_s1SapProvider != 0)
    {
      m_s1SapProvider->UeContextRelease (rnti);
    }
  m_ccmRrcSapProvider-> RemoveUe (rnti);
  // need to do this after UeManager has been deleted
  if (srsCi != 0)
    {
  RemoveSrsConfigurationIndex (srsCi);
    }
}

TypeId
LteEnbRrc::GetRlcType (EpsBearer bearer)
{
  switch (m_epsBearerToRlcMapping)
    {
    case RLC_SM_ALWAYS:
      return LteRlcSm::GetTypeId ();
      break;

    case  RLC_UM_ALWAYS:
      return LteRlcUm::GetTypeId ();
      break;

    case RLC_AM_ALWAYS:
      return LteRlcAm::GetTypeId ();
      break;

    case RLC_TM_ALWAYS:
      return LteRlcTm::GetTypeId ();
      break;

    case PER_BASED:
      if (bearer.GetPacketErrorLossRate () > 1.0e-5)
        {
          return LteRlcUm::GetTypeId ();
        }
      else
        {
          return LteRlcAm::GetTypeId ();
        }
      break;

    default:
      return LteRlcSm::GetTypeId ();
      break;
    }
}


void
LteEnbRrc::AddX2Neighbour (uint16_t cellId)
{
  NS_LOG_FUNCTION (this << cellId);

  if (m_anrSapProvider != 0)
    {
      m_anrSapProvider->AddNeighbourRelation (cellId);
    }
}

void
LteEnbRrc::SetCsgId (uint32_t csgId, bool csgIndication)
{
  NS_LOG_FUNCTION (this << csgId << csgIndication);
  for (uint8_t componentCarrierId = 0; componentCarrierId < m_sib1.size (); componentCarrierId++)
    {
      m_sib1.at (componentCarrierId).cellAccessRelatedInfo.csgIdentity = csgId;
      m_sib1.at (componentCarrierId).cellAccessRelatedInfo.csgIndication = csgIndication;
      m_cphySapProvider.at (componentCarrierId)->SetSystemInformationBlockType1 (m_sib1.at (componentCarrierId));
    }
}

/// Number of distinct SRS periodicity plus one.
static const uint8_t SRS_ENTRIES = 9;
/**
 * Sounding Reference Symbol (SRS) periodicity (TSRS) in milliseconds. Taken
 * from 3GPP TS 36.213 Table 8.2-1. Index starts from 1.
 */
static const uint16_t g_srsPeriodicity[SRS_ENTRIES] = {0, 2, 5, 10, 20, 40,  80, 160, 320};
/**
 * The lower bound (inclusive) of the SRS configuration indices (ISRS) which
 * use the corresponding SRS periodicity (TSRS). Taken from 3GPP TS 36.213
 * Table 8.2-1. Index starts from 1.
 */
static const uint16_t g_srsCiLow[SRS_ENTRIES] =       {0, 0, 2,  7, 17, 37,  77, 157, 317};
/**
 * The upper bound (inclusive) of the SRS configuration indices (ISRS) which
 * use the corresponding SRS periodicity (TSRS). Taken from 3GPP TS 36.213
 * Table 8.2-1. Index starts from 1.
 */
static const uint16_t g_srsCiHigh[SRS_ENTRIES] =      {0, 1, 6, 16, 36, 76, 156, 316, 636};

void 
LteEnbRrc::SetSrsPeriodicity (uint32_t p)
{
  NS_LOG_FUNCTION (this << p);
  for (uint32_t id = 1; id < SRS_ENTRIES; ++id)
    {
      if (g_srsPeriodicity[id] == p)
        {
          m_srsCurrentPeriodicityId = id;
          return;
        }
    }
  // no match found
  std::ostringstream allowedValues;
  for (uint32_t id = 1; id < SRS_ENTRIES; ++id)
    {
      allowedValues << g_srsPeriodicity[id] << " ";
    }
  NS_FATAL_ERROR ("illecit SRS periodicity value " << p << ". Allowed values: " << allowedValues.str ());
}

uint32_t 
LteEnbRrc::GetSrsPeriodicity () const
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_srsCurrentPeriodicityId > 0);
  NS_ASSERT (m_srsCurrentPeriodicityId < SRS_ENTRIES);
  return g_srsPeriodicity[m_srsCurrentPeriodicityId];
}


uint16_t
LteEnbRrc::GetNewSrsConfigurationIndex ()
{
  NS_LOG_FUNCTION (this << m_ueSrsConfigurationIndexSet.size ());
  // SRS
  NS_ASSERT (m_srsCurrentPeriodicityId > 0);
  NS_ASSERT (m_srsCurrentPeriodicityId < SRS_ENTRIES);
  NS_LOG_DEBUG (this << " SRS p " << g_srsPeriodicity[m_srsCurrentPeriodicityId] << " set " << m_ueSrsConfigurationIndexSet.size ());
  if (m_ueSrsConfigurationIndexSet.size () >= g_srsPeriodicity[m_srsCurrentPeriodicityId])
    {
      NS_FATAL_ERROR ("too many UEs (" << m_ueSrsConfigurationIndexSet.size () + 1 
                                       << ") for current SRS periodicity "
                                       <<  g_srsPeriodicity[m_srsCurrentPeriodicityId]
                                       << ", consider increasing the value of ns3::LteEnbRrc::SrsPeriodicity");
    }

  if (m_ueSrsConfigurationIndexSet.empty ())
    {
      // first entry
      m_lastAllocatedConfigurationIndex = g_srsCiLow[m_srsCurrentPeriodicityId];
      m_ueSrsConfigurationIndexSet.insert (m_lastAllocatedConfigurationIndex);
    }
  else
    {
      // find a CI from the available ones
      std::set<uint16_t>::reverse_iterator rit = m_ueSrsConfigurationIndexSet.rbegin ();
      NS_ASSERT (rit != m_ueSrsConfigurationIndexSet.rend ());
      NS_LOG_DEBUG (this << " lower bound " << (*rit) << " of " << g_srsCiHigh[m_srsCurrentPeriodicityId]);
      if ((*rit) < g_srsCiHigh[m_srsCurrentPeriodicityId])
        {
          // got it from the upper bound
          m_lastAllocatedConfigurationIndex = (*rit) + 1;
          m_ueSrsConfigurationIndexSet.insert (m_lastAllocatedConfigurationIndex);
        }
      else
        {
          // look for released ones
          for (uint16_t srcCi = g_srsCiLow[m_srsCurrentPeriodicityId]; srcCi < g_srsCiHigh[m_srsCurrentPeriodicityId]; srcCi++) 
            {
              std::set<uint16_t>::iterator it = m_ueSrsConfigurationIndexSet.find (srcCi);
              if (it == m_ueSrsConfigurationIndexSet.end ())
                {
                  m_lastAllocatedConfigurationIndex = srcCi;
                  m_ueSrsConfigurationIndexSet.insert (srcCi);
                  break;
                }
            }
        } 
    }
  return m_lastAllocatedConfigurationIndex;

}


void
LteEnbRrc::RemoveSrsConfigurationIndex (uint16_t srcCi)
{
  NS_LOG_FUNCTION (this << srcCi);
  std::set<uint16_t>::iterator it = m_ueSrsConfigurationIndexSet.find (srcCi);
  NS_ASSERT_MSG (it != m_ueSrsConfigurationIndexSet.end (), "request to remove unkwown SRS CI " << srcCi);
  m_ueSrsConfigurationIndexSet.erase (it);
}

uint8_t 
LteEnbRrc::GetLogicalChannelGroup (EpsBearer bearer)
{
  if (bearer.IsGbr ())
    {
      return 1;
    }
  else
    {
      return 2;
    }
}

uint8_t 
LteEnbRrc::GetLogicalChannelPriority (EpsBearer bearer)
{
  return bearer.qci;
}

void
LteEnbRrc::SendSystemInformation ()
{
  // NS_LOG_FUNCTION (this);

  for (auto &it: m_componentCarrierPhyConf)
    {
      uint8_t ccId = it.first;

      LteRrcSap::SystemInformation si;
      si.haveSib2 = true;
      si.sib2.freqInfo.ulCarrierFreq = it.second->GetUlEarfcn ();
      si.sib2.freqInfo.ulBandwidth = it.second->GetUlBandwidth ();
      si.sib2.radioResourceConfigCommon.pdschConfigCommon.referenceSignalPower = m_cphySapProvider.at (ccId)->GetReferenceSignalPower ();
      si.sib2.radioResourceConfigCommon.pdschConfigCommon.pb = 0;

      LteEnbCmacSapProvider::RachConfig rc = m_cmacSapProvider.at (ccId)->GetRachConfig ();
      LteRrcSap::RachConfigCommon rachConfigCommon;
      rachConfigCommon.preambleInfo.numberOfRaPreambles = rc.numberOfRaPreambles;
      rachConfigCommon.raSupervisionInfo.preambleTransMax = rc.preambleTransMax;
      rachConfigCommon.raSupervisionInfo.raResponseWindowSize = rc.raResponseWindowSize;
      rachConfigCommon.txFailParam.connEstFailCount = rc.connEstFailCount;
      si.sib2.radioResourceConfigCommon.rachConfigCommon = rachConfigCommon;

      m_rrcSapUser->SendSystemInformation (it.second->GetCellId (), si);
    }

  /*
   * For simplicity, we use the same periodicity for all SIBs. Note that in real
   * systems the periodicy of each SIBs could be different.
   */
  Simulator::Schedule (m_systemInformationPeriodicity, &LteEnbRrc::SendSystemInformation, this);
}

//only Dynamic TTT Ho implemented
uint8_t
LteEnbRrc::ComputeTtt(double sinrDifference)
{
  if(sinrDifference < m_minDiffTttValue)
  {
    return m_maxDynTttValue;
  }
  else if(sinrDifference > m_maxDiffTttValue)
  {
    return m_minDynTttValue;
  }
  else // in between
  {
    double ttt = m_maxDynTttValue - (m_maxDynTttValue - m_minDynTttValue)*(sinrDifference - m_minDiffTttValue)/(m_maxDiffTttValue - m_minDiffTttValue);
    NS_ASSERT_MSG(ttt >= 0, "Negative TTT! \n ttt="<<ttt<<" sinrDifference="<<sinrDifference);
    uint8_t truncated_ttt = ttt;
    return truncated_ttt;
  }
}

bool
LteEnbRrc::IsRandomAccessCompleted (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << (uint32_t) rnti);
  Ptr<UeManager> ueManager = GetUeManager (rnti);
  switch (ueManager->GetState ())
    {
    case UeManager::CONNECTED_NORMALLY:
    case UeManager::CONNECTION_RECONFIGURATION:
      return true;
      break;
    default:
      return false;
      break;

    }
}

// This will be run on the Lte Coordinator
// This is reached from the NrUePhy::CheckIfSweepIsComplete after the sweep is finished.
// LAMM: do we need something similar but receiving the struct from X2 from Coordinator?
void 
LteEnbRrc::DoRecvOptimalGnbBeamMap (LteRrcSap::CellOptimalGnbBeamMap msg)
{
  m_imsiBeamSweepCellSinrMap.erase(msg.ueImsi);

  auto currSNR = 0.0;
  auto maxSNR = 0.0;
  auto maxCellId = 0;
  // not connected to any gNB, maxCellId will be chosen

  CellSinrMap map;
  for (auto const &iter : msg.cellOptimalBeamMap)
  {
    currSNR = iter.second.at(0).second.first;
    if (currSNR > maxSNR)
    {
      maxSNR = currSNR;
      maxCellId = iter.first;
    }
    map.insert(std::pair<uint16_t, double>(iter.first, iter.second.at(0).second.first));
  }

  m_imsiCellSinrMap.erase(msg.ueImsi);
  m_imsiCellSinrMap.insert(std::pair<uint64_t, CellSinrMap> (msg.ueImsi, map));
  
  // if an entry for this imsi in the m_mmWaveCellSetupCompleted exists
  if(m_mmWaveCellSetupCompleted.find(msg.ueImsi) != m_mmWaveCellSetupCompleted.end())
  {
    // if the imsi is already connected to a gNB
    if (m_mmWaveCellSetupCompleted[msg.ueImsi] == true)
    {
      // This UE went through a BeamTracking beam sweep. After such, the UE should connect to the best available cell.
      // For that purpose, initiate handover if best cell != currently serving cell.
      
      // This is only setting m_csiRSFlag in NrGnbMac and no deregistration is performed
      LteRrcSap::DeRegisterUeContext deregisterParams;
      deregisterParams.imsi = msg.ueImsi;
      deregisterParams.cellId = m_lastMmWaveCell[msg.ueImsi];
      deregisterParams.m_sourceOfCommand = LteRrcSap::DeRegisterUeContext::SourceOfCommand::DeRegisterFromCoordinator;
      DoRecvDeRegisterUeCommand(deregisterParams);

      NS_LOG_UNCOND("IMSI " << msg.ueImsi << ": cell ID that has the maximum SNR after BeamTracking beam sweep " << maxCellId);
      if (maxSNR > 5)
      {
        if (maxCellId == m_lastMmWaveCell[msg.ueImsi])
        {
          NS_LOG_UNCOND("----- No handover is needed, serving cell provides best connection");
          RadioLinkMonitoringTraceParams rlmParams;
          rlmParams.imsi = msg.ueImsi;
          rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::NO_HANDOVER_AFTER_UE_SWEEP;
          m_radioLinkMonitoringTrace(rlmParams);
        }
        else
        {
          NS_LOG_UNCOND("----- Handover needed from cell " << m_lastMmWaveCell[msg.ueImsi] << " to " << maxCellId);

          RadioLinkMonitoringTraceParams rlmParams;
          rlmParams.imsi = msg.ueImsi;
          rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::HANDOVER_AFTER_UE_SWEEP;
          rlmParams.sourceCellId = m_lastMmWaveCell[msg.ueImsi];
          rlmParams.targetCellId = maxCellId;
          m_radioLinkMonitoringTrace(rlmParams);

          // compute the TTT
          uint8_t millisecondsToHandover = m_minDynTttValue;

          EventId scheduledHandoverEvent = Simulator::Schedule(MilliSeconds(millisecondsToHandover), &LteEnbRrc::PerformHandover, this, msg.ueImsi);
          LteEnbRrc::HandoverEventInfo handoverInfo;
          handoverInfo.sourceCellId = m_lastMmWaveCell[msg.ueImsi];
          handoverInfo.targetCellId = maxCellId;
          handoverInfo.isLoadBalancing = false;
          handoverInfo.isWithoutSweep = false;
          handoverInfo.scheduledHandoverEvent = scheduledHandoverEvent;
          HandoverEventMap::iterator handoverEvent = m_imsiHandoverEventsMap.find(msg.ueImsi);
          if (handoverEvent != m_imsiHandoverEventsMap.end()) // another event was scheduled, but it was already deleted. Replace the entry
          {
            handoverEvent->second = handoverInfo;
          }
          else
          {
            m_imsiHandoverEventsMap.insert(std::pair<uint64_t, HandoverEventInfo>(msg.ueImsi, handoverInfo));
          }
        }
      }
    }
  }

  uint8_t csiCounter = 0;
  std::vector<uint8_t> gnbRlmBeams(msg.cellOptimalBeamMap.size(), 0);
  std::map<uint8_t, std::vector<uint8_t>> mapOfCsiCounter;
  uint8_t noOfBeamsTbRLM = msg.cellOptimalBeamMap.begin()->second.size();

  // if we do rlm, fetch the required info, sort data, save best beams
  // store that info in m_imsiCellCSIRSSinrMap
  // may also execute the SendSpecificGnbOptimalBeamReport
  if (m_rlmMultiGnb) // montior beams from multiple cells
  {
    // identify best beams for CSI-RS RLM
    std::vector<struct OptimalRLMBeamStruct> bestServingBeams;
    std::vector<struct OptimalRLMBeamStruct> bestBeams;

    for (auto const &iter : msg.cellOptimalBeamMap)
    {
      if (iter.first != maxCellId)
      {
        double SNR = iter.second.at(0).second.first;
        if (10 * std::log10(SNR) > m_maxRateThreshold) // only enable monitoring if SNR from that cell is good enough
        {
          // only monitor the best beam from each potential cell
          struct OptimalRLMBeamStruct thisRLMBeam;
          thisRLMBeam.snr = iter.second.at(0).second.first;
          thisRLMBeam.cellId = iter.first;
          thisRLMBeam.beamId = iter.second.at(0).second.second;
          thisRLMBeam.startingSfnSf = iter.second.at(0).first.first;
          thisRLMBeam.optimalBeamIndex = (thisRLMBeam.cellId - 1) * 4 + 0;

          bestBeams.push_back(thisRLMBeam);
        }
      }
    }

    std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>> it = msg.cellOptimalBeamMap.at(maxCellId);
    for (uint8_t i = 0; i < noOfBeamsTbRLM; i++)
    {
      struct OptimalRLMBeamStruct thisRLMBeam;
      thisRLMBeam.snr = it.at(i).second.first;
      thisRLMBeam.cellId = maxCellId;
      thisRLMBeam.beamId = it.at(i).second.second;
      thisRLMBeam.startingSfnSf = it.at(i).first.first;
      thisRLMBeam.optimalBeamIndex = (thisRLMBeam.cellId - 1) * 4 + i;

      bestServingBeams.push_back(thisRLMBeam);
      if (m_csiRSFromServingGnb == 0 && i == 0 && ((uint8_t)bestBeams.size() >= noOfBeamsTbRLM))
      {
        // None of the serving gNB beams is required to be monitored. Insert the best one from this cell into the bestBeams struct.
        // Third if statement:  do not insert best beam from serving cell if bestBeams.size() >= noOfBeamsTbRLM.
        //                      This prevents double insertion of that beam as remaining free slots in bestBeams are filled with beams from bestServingBeams.
        bestBeams.push_back(thisRLMBeam);
      }
    }

    sort(bestBeams.begin(), bestBeams.end(), [](struct OptimalRLMBeamStruct &a, struct OptimalRLMBeamStruct &b){ return a.snr > b.snr; });

    if (m_csiRSFromServingGnb == 0)
    {
      // no beam monitoring required for the serving gNB
      if ((uint8_t)bestBeams.size() < noOfBeamsTbRLM)
      {
        // fill in remaining slots if some are unused
        bestBeams.insert(bestBeams.end(), bestServingBeams.begin(), bestServingBeams.end() - bestBeams.size());
        sort(bestBeams.begin(), bestBeams.end(), [](struct OptimalRLMBeamStruct &a, struct OptimalRLMBeamStruct &b){ return a.snr > b.snr; });
      }
      else if ((uint8_t)bestBeams.size() > noOfBeamsTbRLM)
      {
        bestBeams.erase(bestBeams.begin() + noOfBeamsTbRLM, bestBeams.end());
      }
    }
    else if (m_csiRSFromServingGnb == noOfBeamsTbRLM)
    {
      // all beams are from the serving gNB
      bestBeams = bestServingBeams;
    }
    else
    {
      if ((uint8_t)bestBeams.size() > noOfBeamsTbRLM - m_csiRSFromServingGnb)
      {
        bestBeams.erase(bestBeams.begin() + (noOfBeamsTbRLM - m_csiRSFromServingGnb), bestBeams.end());
      }
      bestBeams.insert(bestBeams.end(), bestServingBeams.begin(), bestServingBeams.end() - bestBeams.size());
      sort(bestBeams.begin(), bestBeams.end(), [](struct OptimalRLMBeamStruct &a, struct OptimalRLMBeamStruct &b){ return a.snr > b.snr; });
    }

    CellCSIRSSinrMap cellCSIRSSinrMap;
    for (uint8_t i = 0; i < noOfBeamsTbRLM; i++)
    {
      uint8_t cellId = bestBeams[i].cellId;
      gnbRlmBeams[cellId-1]++;

      if (mapOfCsiCounter.find(cellId) == mapOfCsiCounter.end())
      {
        mapOfCsiCounter.insert(std::pair<uint8_t, std::vector<uint8_t>>(cellId, {csiCounter}));
        csiCounter += 1;
      }
      else
      {
        mapOfCsiCounter.at(cellId).push_back(csiCounter);
        csiCounter += 1;
      }

      cellCSIRSSinrMap.emplace_back(std::make_pair(bestBeams[i].snr, bestBeams[i].cellId));
    }

    m_imsiCellCSIRSSinrMap.erase(msg.ueImsi);
    m_imsiCellCSIRSSinrMap.insert ({msg.ueImsi, cellCSIRSSinrMap});
  }
  else
  {
    // all RLM beams are from best SNR cell
    CellCSIRSSinrMap cellCSIRSSinrMap;
    gnbRlmBeams[maxCellId - 1] = noOfBeamsTbRLM;
    std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>> it = msg.cellOptimalBeamMap.at(maxCellId);
    for (uint8_t i = 0; i < noOfBeamsTbRLM; i++)
    {
      if (i == 0)
      {
        mapOfCsiCounter.insert(std::pair<uint8_t, std::vector<uint8_t>>(maxCellId, {csiCounter}));
      }
      else
      {
        csiCounter++;
        mapOfCsiCounter.at(maxCellId).push_back(csiCounter);
      }

      double snr = it.at(i).second.first;
      cellCSIRSSinrMap.emplace_back(std::make_pair(snr, maxCellId));
    }
    m_imsiCellCSIRSSinrMap.erase(msg.ueImsi);
    m_imsiCellCSIRSSinrMap.insert({msg.ueImsi, cellCSIRSSinrMap});
  }

  for (auto const& iter : msg.cellOptimalBeamMap)
  {
    if (m_imsiBeamSweepCellSinrMap.find(msg.ueImsi)==m_imsiBeamSweepCellSinrMap.end())
    {
      BeamSweepCellSinrMap map;
      map.insert (std::pair<uint16_t, double> (iter.first, iter.second.at(0).second.first));
      m_imsiBeamSweepCellSinrMap.insert (std::pair<uint64_t, BeamSweepCellSinrMap> (msg.ueImsi, map));
    }
    else
    {
      m_imsiBeamSweepCellSinrMap[msg.ueImsi].insert(std::pair<uint16_t, double> (
          iter.first, iter.second.at(0).second.first));
    }
    if (m_realisticIA)
    {
      EpcX2Sap::OptimalGnbBeamReportParams params;
      params.targetCellId = iter.first;
      params.ueImsi = msg.ueImsi;
      params.startingSfn = iter.second.at(0).first.first;
      std::vector<uint16_t> tmp_optimalBeamIndex;
      for (size_t i = 0; i < 8; i++)
      {
        if(i < iter.second.size())
        {
          tmp_optimalBeamIndex.push_back(iter.second.at(i).first.second);
        }
        else
        {
          // If less than 8 beams will be used for RLM, fill the rest with zeros.
          // Zeros will be ignored later due to knwon number of RLM-beams
          tmp_optimalBeamIndex.push_back(0);
        }
      }
      params.optimalBeamIndex = tmp_optimalBeamIndex;
      
      if (maxCellId-iter.first <= 1)
      {
        params.isServingCell = true;
      }
      else
      {
        params.isServingCell = false;
      }

      params.numOfBeamsTbRlm = gnbRlmBeams[iter.first-1];

      if (mapOfCsiCounter.find(iter.first) != mapOfCsiCounter.end())
      {
        params.csiCounterVector = mapOfCsiCounter.at(iter.first);
      }
      else
      {
        params.csiCounterVector = {};
      }
      m_x2SapProvider->SendSpecificGnbOptimalBeamReport (params);
    }
  }

  if (m_beamSweepStarted.at(msg.ueImsi))
  {
    m_beamSweepCompleted.at(msg.ueImsi) = true;
  }  
}

void 
LteEnbRrc::DoRecvOptimalGnbBeamReport (LteRrcSap::OptimalGnbBeamReport params)
{
  std::cout << "[lte] LteEnbRrc::DoRecvOptimalGnbBeamReport: optimal beam index " << params.optimalBeamIndex.front() << std::endl;
  m_cphySapProvider.at (0)->SetOptimalGnbBeamForImsi (params.ueImsi, params.startingSfn, params.optimalBeamIndex, params.isServingCell, params.numOfBeamsTbRlm);
  if (m_rlmOn)
  {
    m_cmacSapProvider.at (0)->SetCSIRLMResources (m_csiPeriodicityInSlots, m_csiOffsetInSlots, params.numOfBeamsTbRlm, params.ueImsi, params.csiCounterVector, params.isServingCell);
    m_cmacSapProvider.at (0)->SetCSIRSFlag (params.ueImsi, false);
  }  
}

void
LteEnbRrc::DoRecvDeRegisterUeCommand (LteRrcSap::DeRegisterUeContext params)
{
  // If multi-gNB RLM is enabled, check which gNBs send CSI to the IMSI. Signal to them (besides the serving gNB) to disable CIS-RS transmission.
  std::map<uint8_t, bool> gnbToNotifyMap; // second value indicates if cell is serving gNB

  if (params.m_sourceOfCommand == LteRrcSap::DeRegisterUeContext::SourceOfCommand::DeRegisterFromUE)
  {
    m_mmWaveCellSetupCompleted[params.imsi] = false; // This prevents handover decisions after a beam sweep has been started. Should be set to true after handover.
  }

  gnbToNotifyMap.insert(std::pair<uint8_t, bool> (params.cellId, true));

  if (m_rlmMultiGnb)
  {
    for (size_t n = 0; n < m_imsiCellCSIRSSinrMap.at(params.imsi).size(); n++)
    {
      uint8_t cellId = m_imsiCellCSIRSSinrMap.at(params.imsi).at(n).second;
      if (gnbToNotifyMap.find(cellId) == gnbToNotifyMap.end())
      {
        if (cellId != params.cellId)
        {
          // params.cellId is the serving gNB and was already handled before the for loop.
          gnbToNotifyMap.insert(std::pair<uint8_t, bool>(cellId, false));
        }
      }
    }
  }

  for (auto const& iter : gnbToNotifyMap)
  {
    EpcX2Sap::DeRegisterUeParams outgoingParams;
    outgoingParams.imsi = params.imsi;
    outgoingParams.targetCellId = iter.first;
    outgoingParams.isServingGnb = iter.second;
    switch (params.m_sourceOfCommand)
    {
    case LteRrcSap::DeRegisterUeContext::SourceOfCommand::DeRegisterFromCoordinator:
      outgoingParams.m_sourceOfCommand = EpcX2Sap::DeRegisterUeParams::SourceOfCommand::DeRegisterFromCoordinator;
      break;
    case LteRrcSap::DeRegisterUeContext::SourceOfCommand::DeRegisterFromUE:
      outgoingParams.m_sourceOfCommand = EpcX2Sap::DeRegisterUeParams::SourceOfCommand::DeRegisterFromUe;
      break;
    default:
      break;
    }
    m_x2SapProvider->SendDeRegisterUeCommand(outgoingParams);
  }
}

void 
LteEnbRrc::DoRecvTriggerRegisterUe (uint64_t imsi, const Ptr<NetDevice> &netDev)
{
  m_cphySapProvider.at(0)->AttachUeFromRRC (imsi, netDev);
}

void 
LteEnbRrc::DoRecvRegisterUE (uint64_t imsi)
{
  m_cphySapProvider.at(0)->RegisterUeFromRRC (imsi);
}

void 
LteEnbRrc::DoRecvDeregisterUe (LteRrcSap::DeRegisterUeContext params)
{
  switch (params.m_sourceOfCommand)
  {
  case LteRrcSap::DeRegisterUeContext::SourceOfCommand::DeRegisterFromCoordinator:
    m_cmacSapProvider.at(0)->SetCSIRSFlag (params.imsi, true);
    break;
  case LteRrcSap::DeRegisterUeContext::SourceOfCommand::DeRegisterFromUE:
    m_cmacSapProvider.at(0)->SetCSIRSFlag (params.imsi, true);
    if(params.isServingGnb)
    {
      m_cphySapProvider.at(0)->DeregisterUeFromRRC (params.imsi, DoGetRntiFromImsi(params.imsi));
      Simulator::ScheduleNow (&LteEnbRrc::DoSendUeDeRegisterCompletedToLteCo, this, params.imsi);
    }
  default:
    break;
  }
}

void 
LteEnbRrc::DoRecvUpdateBeamsTBRLMFromLteCo (LteRrcSap::UpdateBeamsTbRLM params)
{
  m_cphySapProvider.at (0)->UpdateBeamsTBRLM (params.ueImsi, params.optimalBeamIndexVector, params.startingSfn, params.csiCounterVector);

  if (m_rlmMultiGnb)
  {
    // reconfigure CSI-RS resources
    m_cmacSapProvider.at(0)->SetCSIRLMResources(m_csiPeriodicityInSlots, m_csiOffsetInSlots, params.optimalBeamIndexVector.size(), params.ueImsi, params.csiCounterVector, params.isServingCellId);
    m_cmacSapProvider.at(0)->SetCSIRSFlag(params.ueImsi, false);
  }
}

/// Returns true if we have a handover event for IMSI which ws not scheduled due to outage
bool 
LteEnbRrc::DoCheckForHandoverEvents (uint64_t imsi)
{
  if (m_imsiHandoverEventsMap.find (imsi) != m_imsiHandoverEventsMap.end ())
  {
    if (m_outageAndHandover)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    return false;
  }
  
}

void 
LteEnbRrc::SetupUeFromHelper (uint16_t lteRnti, uint64_t imsi)
{
  m_lteRnti.insert(std::pair<uint64_t, uint16_t> (imsi, lteRnti));
  m_beamSweepCompleted.insert(std::pair<uint64_t, bool> (imsi, false));
  m_beamSweepStarted.insert(std::pair<uint64_t, bool> (imsi, false));
  LteEnbRrcSapUser::SetupUeParameters ueParams;
  ueParams.srb0SapProvider = GetUeManager (lteRnti)->GetSignalingRadioBearerInfo(0)->m_rlc->GetLteRlcSapProvider ();
  ueParams.srb1SapProvider = GetUeManager (lteRnti)->GetSignalingRadioBearerInfo(1)->m_pdcp->GetLtePdcpSapProvider ();
  m_rrcSapUser->SetupUe (lteRnti, ueParams);
}

void
LteEnbRrc::InitiateBeamSweepFromRrc (double perceivedSinr, uint64_t imsi)
{
  NS_ASSERT_MSG(m_lteRnti.find(imsi) != m_lteRnti.end(), "In InitiateBeamSweepFromRrc: lteRnti not found for IMSI "<< imsi);
  uint16_t rnti = m_lteRnti.find(imsi)->second;

  if (perceivedSinr < m_outageThreshold)
  {
    m_rrcSapUser->SendBeamSweepTriggerFromLTECoordinator (rnti);
  }
  else
  {
    // This sends the data directly to UE from CO
    m_rrcSapUser->SendBeamSweepCommandFromLTECoordinator (rnti);
  }
  Simulator::Schedule (m_beamSweepCompleteTimeoutDuration, &LteEnbRrc::BeamSweepTimer, this, imsi);
}

void 
LteEnbRrc::DoRecvUeDeRegisterCompletedFromGnb (EpcX2Sap::InformLteCoordinatorParams params)
{
  NS_ASSERT_MSG(m_lteRnti.find(params.ueImsi) != m_lteRnti.end(), "In DoRecvUeDeRegisterCompletedFromGnb: lteRnti not found for IMSI "<< params.ueImsi);
  uint16_t rnti = m_lteRnti.find(params.ueImsi)->second;
  m_rrcSapUser->SendUeDeRegistrationCompletedUpdate (rnti, params.ueImsi);
}

void 
LteEnbRrc::DoSendUeDeRegisterCompletedToLteCo (uint64_t imsi)
{
  if (m_lteCellId != 0)
  {
    // ueRnti not needed, set to 0 here.
    // in DoRecvUeDeRegisterCompletedFromGnb (see above), RNTI is restored via IMSI and m_lteRnti!
    EpcX2Sap::InformLteCoordinatorParams updateParams;
    updateParams.sourceCellId = GetCellId ();
    updateParams.targetCellId = m_lteCellId;
    updateParams.ueImsi = imsi;
    updateParams.ueRnti = 0; // not needed
    updateParams.hoCompletedFlag = false; // not needed
    m_x2SapProvider->SendDeRegisterUeCompletedUpdate (updateParams);
  }
}

uint16_t
LteEnbRrc::GetLteRnti(uint64_t imsi)
{
  NS_ASSERT_MSG(m_lteRnti.find(imsi) != m_lteRnti.end(), "In GetLteRnti: lteRnti not found for IMSI " << imsi);
  uint16_t rnti = m_lteRnti.find(imsi)->second;
  return rnti;
}

void 
LteEnbRrc::SetSweepParameters (uint64_t imsi, bool sweepStarted, bool sweepCompleted)
{
  m_beamSweepStarted.at(imsi) = sweepStarted;
  m_beamSweepCompleted.at(imsi) = sweepCompleted;
}

void 
LteEnbRrc::DoRecvClearHandoverEvent (uint64_t imsi)
{
  HandoverEventMap::iterator handoverIt = m_imsiHandoverEventsMap.find(imsi);;

  if(handoverIt != m_imsiHandoverEventsMap.end()){
    handoverIt->second.scheduledHandoverEvent.Cancel ();
  }
  
  m_imsiHandoverEventsMap.erase(imsi);
}

void 
LteEnbRrc::DoRecvBeamPerformComplete (uint64_t imsi, bool beamSweepComplete)
{
  if (m_beamSweepStarted.at(imsi))
  {
    m_beamSweepCompleted.at(imsi) = beamSweepComplete;
  }
}

std::pair<uint8_t, uint8_t>
LteEnbRrc::GetCSIRSResourceVector (uint64_t imsi)
{
  return m_cmacSapProvider.at(0)->GetCSIRSResourceFromMac (imsi);
}

void 
LteEnbRrc::BeamSweepTimer (uint64_t imsi)
{
  if (m_beamSweepStarted.at(imsi) && !m_beamSweepCompleted.at(imsi))
  {
    SetSweepParameters (imsi, false, false);
    m_cmacSapProvider.at(0)->SetCSIRSFlag (imsi, false);
  }
}

void 
LteEnbRrc::DoRecvRRCCSIRSReport (uint64_t imsi, uint16_t cellId, std::vector<double> csiRSVector, std::vector<uint8_t> csiRSCellIdVector, std::vector<BeamId> csiRSBeamIdVector)
{
  NS_ASSERT(csiRSVector.size() == csiRSCellIdVector.size());

  std::map<uint8_t, BeamId> suboptimalCellBeamIdMap;

  CellCSIRSSinrMap cellCSIRSSinrMap;
  for (uint8_t i = 0; i < csiRSVector.size(); i++)
  {
    cellCSIRSSinrMap.emplace_back(std::make_pair(csiRSVector[i], csiRSCellIdVector[i]));
    if (csiRSCellIdVector[i] != cellId && (suboptimalCellBeamIdMap.find(csiRSCellIdVector[i]) == suboptimalCellBeamIdMap.end()))
    {
      suboptimalCellBeamIdMap.insert(std::pair<uint8_t, BeamId> (csiRSCellIdVector[i], csiRSBeamIdVector[i]));
    }
  }

  for (auto const& iter : suboptimalCellBeamIdMap)
  {
    EpcX2Sap::SuboptimalCSIRSReportParams params;
    params.imsi = imsi;
    params.targetCellId = iter.first;
    params.sector = iter.second.GetSector();
    params.elevation = iter.second.GetElevation();

    m_x2SapProvider->SendSuboptimalCSIRSReport (params);
  }

  m_imsiCellCSIRSSinrMap.erase(imsi);
  m_imsiCellCSIRSSinrMap.insert ({imsi, cellCSIRSSinrMap});
}

void 
LteEnbRrc::DoRelayRRCCSIRSReport (LteEnbCphySapUser::UeRRCCSIRSReport info) 
{
  EpcX2Sap::UeImsiCSIRSSinrParams params;
  params.ueImsi = DoGetImsiFromRnti (info.rnti);
  params.sourceCellId = info.cellId;
  params.targetCellId = m_lteCellId;

  std::vector<double> tmp_csiRSVector;
  std::vector<uint8_t> tmp_csiRSCellIdVector;
  std::vector<BeamId> tmp_csiRSBeamIdVector;
  for (uint8_t i = 0; i < info.csiRSVector.size(); i++)
  {
    tmp_csiRSCellIdVector.emplace_back(info.csiRSVector[i].first);
    tmp_csiRSVector.emplace_back(info.csiRSVector[i].second.first);
    tmp_csiRSBeamIdVector.emplace_back(info.csiRSVector[i].second.second);
  }

  params.csiRSCellIdVector = tmp_csiRSCellIdVector;
  params.csiRSVector = tmp_csiRSVector;
  params.csiRSBeamIdVector = tmp_csiRSBeamIdVector;

  m_x2SapProvider->SendUeCSIRSSinrReport (params);
}

void 
LteEnbRrc::DoForwardUeSSBRSReport (LteRrcSap::UpdateBeamsTbRLM params)
{
  // This is happening on the coordinator. X2 is used to distribute to gNBs
    std::cout << " !!_!_!__! " <<  GetCellId() << " LteEnbRrc::DoForwardUeSSBRSReport" << std::endl;
  /* 
  Beams for CSI-RS RLM are updated at the respective gNB MAC schedulers.
  LTE RRC only gets fresh information via CSI-RS report in DoRecvRRCCSIRSReport, which is later in time.
  During this interval, the CSI-RS information in m_imsiCellCSIRSSinrMap may be accessed.
  To prevent decisions regarding gNBs which are no longer being monitored, set parameters in m_imsiCellCSIRSSinrMap for the outdated gNBs,
  before m_imsiCellCSIRSSinrMap is eventually updated in DoRecvRRCCSIRSReport.
  */

  // This might be the case in REM-based operation. Might fix later. 
  if (m_imsiCellCSIRSSinrMap.empty())
  {
    return;
  }

  uint8_t noOfBeamsTbRLM = m_imsiCellCSIRSSinrMap.at(params.ueImsi).size();
  if (params.csiCounterVector.size() == 1 && params.csiCounterVector[0] > noOfBeamsTbRLM) // value 99 used as signalling
  {
    // entry for that params.targetCellId is now invalid
    for (size_t n = 0; n < m_imsiCellCSIRSSinrMap.at(params.ueImsi).size(); n++)
    {
      uint8_t cellId = m_imsiCellCSIRSSinrMap.at(params.ueImsi).at(n).second;
      if (params.targetCellId == cellId)
      {
        m_imsiCellCSIRSSinrMap.at(params.ueImsi).at(n).first = 1; // equals 0 dB, which is sufficient to not be considered as alternative link opportunity.
      }
    }
  }

  // CO sends to gNBs
  m_x2SapProvider->SendUeSSBRSReport (params);
}

void 
LteEnbRrc::DoRecvSuboptimalCSIRSReport (LteRrcSap::SuboptimalCSIRSReport params)
{
  m_cphySapProvider.at (0)->UpdateSuboptimalCSIRSBeams (params.ueImsi, params.sector, params.elevation);
}

void 
LteEnbRrc::DoRecvSetCSIRSFlag (LteRrcSap::SetCSIRSFlagRRC params)
{
  m_cmacSapProvider.at(0)->SetCSIRSFlag (params.imsi, params.flag);
}

void 
LteEnbRrc::UpdateCSIRLMResources(uint64_t imsi)
{
  if (m_rlmMultiGnb)
  {
    // Due to a possible handover without beam sweep, update CSI-RS RLM resources!
    m_cmacSapProvider.at(0)->UpdateCSIRLMResources(imsi, m_maxCSIperFrame);
  }
}

void
LteEnbRrc::SetMinCSIRSFromServingGnb(uint8_t minCSIRS)
{
  if (m_useREMLAB)
  {
    m_rlmMultiGnb = false; // No CSI-RS-related logic if we use REM-bsed beam selection
    m_csiRSFromServingGnb = 0;
    return;
  }

  m_csiRSFromServingGnb = minCSIRS;
  if (minCSIRS < 4)
  {
    m_rlmMultiGnb = true;
  }
  else
  {
    m_rlmMultiGnb = false;
  }
}

//--------------------------------------------------------------------------------------------
// Location-aided Beamforming
//--------------------------------------------------------------------------------------------

// Receive report from X2. Do not need this anymore?
void 
LteEnbRrc::DoRecvUePositionReport(LteRrcSap::PositionReport params)
{
  ForwardUePositionReportToREM(params);
}

// Position report from UE to CO
void
LteEnbRrc::DoRecvPositionReport (uint16_t rnti, LteRrcSap::PositionReport msg)
{
  if (!m_useREMLAB)
    return;

  std::cout << GetCellId() << " LteEnbRrc::DoRecvPositionReport: got a report from UE with rnti " << static_cast<int>(rnti) << std::endl;
  NS_LOG_FUNCTION (this << rnti);
  // GetUeManager (rnti)->RecvPositionReport (msg);
  // We might not forward to UeManager, but rather directl call forwarding function to REM
  ForwardUePositionReportToREM(msg);
}

// FIXME: Rename to DoRecvBeamsForGnb and redo  NrGnbPhy::SetREMBeam o avoid manual mapping
// gNB should just receive the BeamId
void
LteEnbRrc::DoRecvREMLinkDataToGnb(LteRrcSap::LinkData params)
{
  if (!m_useREMLAB)
    return;

  std::cout << "LteEnbRrc::DoRecvREMLinkDataToGnb: cell " << GetCellId() << " received LinkData from coordinator" << std::endl;
  // pass the LinkData struct to NrGnbPhy over cphySapProvider
  m_cphySapProvider.at(0)->SetREMBeam(params);
}

void
LteEnbRrc::LoadInventory(std::string inventory_file)
{
  if (m_useErroneousREM)
    std::cout << "LteEnbRrc: applying normally distributed error on top of each saved REM entry" << std::endl;
  
  std::map<Vector, std::pair<double, double>> m_remPositionErrors;
  std::vector<Vector> m_remNonEmtpyEntries;
  m_remNonEmtpyEntries.reserve(500*500);

  NS_LOG_UNCOND ("Loading the inventory from: " << inventory_file);
	std::ifstream singlefile;
	singlefile.open (inventory_file.c_str (), std::ifstream::in);
	NS_ASSERT_MSG (singlefile.good (), "Inventory file not found");

	Ptr<LinkData> linkData = Create<LinkData> ();
	Vector rxPos;

	uint16_t counter = 0;
	std::string line;
	std::string token;
  while (std::getline (singlefile, line)) //Parse each line of the file
  {
  	if (counter == 10)
  	{
      double xErrorValue{0.0};
      double yErrorValue{0.0};

      if(m_useErroneousREM)
      {
        Ptr<NormalRandomVariable> xErr = CreateObject<NormalRandomVariable> ();
        Ptr<NormalRandomVariable> yErr = CreateObject<NormalRandomVariable> ();
        xErr->SetAttribute ("Mean", DoubleValue (0));
        xErr->SetAttribute ("Variance", DoubleValue (10));
        yErr->SetAttribute ("Mean", DoubleValue (0));
        yErr->SetAttribute ("Variance", DoubleValue (10));
        xErrorValue = xErr->GetValue();
        yErrorValue = yErr->GetValue();
        // std::cout << "REM xErr " << xErrorValue << " yErr " << yErrorValue << std::endl;
      }

      // When we use measurement REM or position errors, we want to still keep an ideal REM for tracing and debugging purposes.
      // It can allow us to compare expected path loss of selected beams, ideal cell that would have been used
      // and its sectors.
      m_idealLinkData.insert(std::make_pair(rxPos, linkData));
      
      // Store the error value applied for this REM entry. will be used to draw a heatmap of query errors.
      m_remPositionErrors.insert(std::make_pair(rxPos, std::make_pair(xErrorValue, yErrorValue)));

      rxPos.x += xErrorValue;
      rxPos.y += yErrorValue;
      // std::round is used in ThreeGppSpectrumPropagationLossModel
      rxPos.x = std::round(rxPos.x);
      rxPos.y = std::round(rxPos.y);

      // Store the REM entry corresponding to the parsed or adjusted position.
  		m_allLinkData.insert(std::make_pair (rxPos, linkData));
      // Append only the position for which we inserted some real data,
      // excluding obstacles, as they do not convey any useful information.
      if (linkData->m_path != 0)
        m_remNonEmtpyEntries.push_back(rxPos);

  		linkData = Create<LinkData> ();
  		counter = 0;
  	}

  	doubleVector_t lineElements;
  	std::istringstream stream (line);
      while (getline (stream, token, ',')) //Parse each comma separated string in a line
      {
      	double sigma = 0.00;
      	std::stringstream stream (token);
      	stream >> sigma;
      	lineElements.push_back (sigma);
      }

      switch (counter)
      {
      	case 0:
      	rxPos.x = lineElements.at (0);
      	rxPos.y = lineElements.at (1);
      	rxPos.z = lineElements.at (2);
      	break;
      	case 1:
      	linkData->m_path = lineElements.at(0);
      	break;
      	case 2:
      	linkData->m_los = lineElements;
      	break;
      	case 3:
      	linkData->m_txId = lineElements;
      	break;
      	case 4:
      	linkData->m_aodAzimuth = lineElements;
      	break;
      	case 5:
      	linkData->m_aodElevation = lineElements;
      	break;
      	case 6:
      	linkData->m_aoaAzimuth = lineElements;
      	break;
      	case 7:
      	linkData->m_aoaElevation = lineElements;
      	break;
      	case 8:
      	linkData->m_pathloss = lineElements;
      	break;
      	case 9:
        linkData->m_delay = lineElements;
      	break;
      	default:
      	break;
      }
      counter++;
  }

  // Write the measurement REM for a file for debugging purposes and further reuse.
  WriteRemToFile("MeasurementRem.txt", m_allLinkData);

  std::cout << " Amount of loaded REM entries is " <<  m_remPositionErrors.size() << std::endl;
  m_remErrorTrace(m_remPositionErrors);
  m_measRemNonEmptyEntries(m_remNonEmtpyEntries);
}

void
LteEnbRrc::WriteRemToFile(const std::string& filename, const std::map<Vector, Ptr<LinkData>>& linkData)
{
     std::ofstream outFile(filename, std::ios::binary); // Open file in binary mode
    if (!outFile.is_open())
    {
        throw std::runtime_error("Unable to open file for writing.");
    }

    // Write intVector size and data
    for (auto remEntry : m_allLinkData)
    {
      outFile << remEntry.first.x << "," << remEntry.first.y << "," << remEntry.first.z << std::endl;
      outFile << remEntry.second->m_path << std::endl;

      if (remEntry.second->m_path != 0)
      {
        // LoS
        size_t vectorSize = remEntry.second->m_los.size();
        std::copy(remEntry.second->m_los.begin(), remEntry.second->m_los.end() - 1, std::ostream_iterator<double>(outFile, ",")); // Write all but last with commas
        outFile << remEntry.second->m_los.back() << "\n"; // Write the last element without a trailing comma

        // gNB IDs
        vectorSize = remEntry.second->m_txId.size();
        std::copy(remEntry.second->m_txId.begin(), remEntry.second->m_txId.end() - 1, std::ostream_iterator<double>(outFile, ",")); // Write all but last with commas
        outFile << remEntry.second->m_txId.back() << "\n"; // Write the last element without a trailing comma
        
        // AoD Azimuth
        vectorSize = remEntry.second->m_aodAzimuth.size();
        std::copy(remEntry.second->m_aodAzimuth.begin(), remEntry.second->m_aodAzimuth.end() - 1, std::ostream_iterator<double>(outFile, ",")); // Write all but last with commas
        outFile << remEntry.second->m_aodAzimuth.back() << "\n"; // Write the last element without a trailing comma
        
        // AoD Elevation
        vectorSize = remEntry.second->m_aodElevation.size();
        std::copy(remEntry.second->m_aodElevation.begin(), remEntry.second->m_aodElevation.end() - 1, std::ostream_iterator<double>(outFile, ",")); // Write all but last with commas
        outFile << remEntry.second->m_aodElevation.back() << "\n"; // Write the last element without a trailing comma
                 
        // AoA Azimuth
        vectorSize = remEntry.second->m_aoaAzimuth.size();
        std::copy(remEntry.second->m_aoaAzimuth.begin(), remEntry.second->m_aoaAzimuth.end() - 1, std::ostream_iterator<double>(outFile, ",")); // Write all but last with commas
        outFile << remEntry.second->m_aoaAzimuth.back() << "\n"; // Write the last element without a trailing comma
        
        // AoA Elevation
        vectorSize = remEntry.second->m_aoaElevation.size();
        std::copy(remEntry.second->m_aoaElevation.begin(), remEntry.second->m_aoaElevation.end() - 1, std::ostream_iterator<double>(outFile, ",")); // Write all but last with commas
        outFile << remEntry.second->m_aoaElevation.back() << "\n"; // Write the last element without a trailing comma

        // Pathloss. FIXME: for WiIS this is actually storing RX power at receiver
        vectorSize = remEntry.second->m_pathloss.size();
        std::copy(remEntry.second->m_pathloss.begin(), remEntry.second->m_pathloss.end() - 1, std::ostream_iterator<double>(outFile, ",")); // Write all but last with commas
        outFile << remEntry.second->m_pathloss.back() << "\n"; // Write the last element without a trailing comma
         
        // Delay Spread. FIXME: is this correct?
        vectorSize = remEntry.second->m_delay.size();
        std::copy(remEntry.second->m_delay.begin(), remEntry.second->m_delay.end() - 1, std::ostream_iterator<double>(outFile, ",")); // Write all but last with commas
        outFile << remEntry.second->m_delay.back() << "\n"; // Write the last element without a trailing comma
      }
      else
      {
        // This REM entry is empty, but we still need to write zeros for all fields
        for (uint8_t it=1; it < 9; it++)
          outFile << 0 << std::endl;
      }
    }

    outFile.close();
}

void
LteEnbRrc::LoadRemFromInventory(std::string inventory_file, std::map<Vector, Ptr<LinkData>>& rem)
{
  NS_LOG_UNCOND ("Loading the inventory from: " << inventory_file);
	std::ifstream singlefile;
	singlefile.open (inventory_file.c_str (), std::ifstream::in);
	NS_ASSERT_MSG (singlefile.good (), "Inventory file not found");

	Ptr<LinkData> linkData = Create<LinkData> ();
	Vector rxPos;

	uint16_t counter = 0;
	std::string line;
	std::string token;
  while (std::getline (singlefile, line)) //Parse each line of the file
  {
  	if (counter == 10)
  	{
      rem.insert(std::make_pair(rxPos, linkData));
      
  		linkData = Create<LinkData> ();
  		counter = 0;
  	}

  	doubleVector_t lineElements;
  	std::istringstream stream (line);
      while (getline (stream, token, ',')) //Parse each comma separated string in a line
      {
      	double sigma = 0.00;
      	std::stringstream stream (token);
      	stream >> sigma;
      	lineElements.push_back (sigma);
      }

      switch (counter)
      {
      	case 0:
      	rxPos.x = lineElements.at (0);
      	rxPos.y = lineElements.at (1);
      	rxPos.z = lineElements.at (2);
      	break;
      	case 1:
      	linkData->m_path = lineElements.at(0);
      	break;
      	case 2:
      	linkData->m_los = lineElements;
      	break;
      	case 3:
      	linkData->m_txId = lineElements;
      	break;
      	case 4:
      	linkData->m_aodAzimuth = lineElements;
      	break;
      	case 5:
      	linkData->m_aodElevation = lineElements;
      	break;
      	case 6:
      	linkData->m_aoaAzimuth = lineElements;
      	break;
      	case 7:
      	linkData->m_aoaElevation = lineElements;
      	break;
      	case 8:
      	linkData->m_pathloss = lineElements;
      	break;
      	case 9:
        linkData->m_delay = lineElements;
      	break;
      	default:
      	break;
      }
      counter++;
  }
}

void
LteEnbRrc::LoadRem(std::string input_files_folder)
{
  if (!m_loadMeasurementRem)
  {
    // We want to load the original inventory file and apply measurement error on top (if configured).
    LoadInventory(input_files_folder + "Inventory_28GHz.txt");
  }
  else
  {
    // We want to load an already prepared measurement REM directly from a file.

    // WARNING!!!!
    // Tracing of REM measurement error is not possible when loading the file, as all positions are already 
    // with an applied error. If you want to trace that, set m_loadMeasurementRem to false and re-run the simulation.

    // Load ideal REM for Bema ID offset tracing.
    NS_LOG_DEBUG(" --- Loading Ideal REM for Beam ID offset tracing");
    LoadRemFromInventory(input_files_folder + "Inventory_28GHz.txt", m_idealLinkData);
    // Load actual measurement REM that will be used for the REMLAB.
    NS_LOG_DEBUG(" --- Loading Measurement REM from prepared file for REMLAB");
    LoadRemFromInventory(input_files_folder + "MeasurementRem.txt", m_allLinkData);
  }
}

void
LteEnbRrc::SendBeamsFromCoordinator(uint64_t imsi, size_t optimalEntryIndex, Ptr<LinkData> linkData)
{
  // --------------------------------------
  // Send best beam to the gNB over X2
  // --------------------------------------
  EpcX2Sap::LinkDataToGnb params;
  params.imsi = imsi;
  params.targetCellId = linkData->m_txId.at(optimalEntryIndex);
  params.linkData.imsi = imsi;
  params.linkData.bestTxId = linkData->m_txId.at(optimalEntryIndex);
  params.linkData.bestPathloss = linkData->m_pathloss.at(optimalEntryIndex);
  params.linkData.bestAoDElevation = linkData->m_aodElevation.at(optimalEntryIndex);
  params.linkData.bestAoDAzimuth = linkData->m_aodAzimuth.at(optimalEntryIndex);
  params.linkData.bestAoAElevation = linkData->m_aoaElevation.at(optimalEntryIndex);
  params.linkData.bestAoAAzimuth = linkData->m_aoaAzimuth.at(optimalEntryIndex);
  // For now we do not care about the rest of data struct. Might even reduce it to only have best values
  
  // Send the struct with best AoD, AoA to the best gNB

  // This needs to be executed on each location reception
  m_x2SapProvider->SendREMLinkDataToGnb(params);

  // ---------------------------------------------------
  // Send best beam to the UE over direct LTE connection
  // ---------------------------------------------------
  LteRrcSap::LinkData rrcLinkData;
  rrcLinkData.imsi = imsi;
  rrcLinkData.bestTxId = linkData->m_txId.at(optimalEntryIndex);
  rrcLinkData.bestPathloss = linkData->m_pathloss.at(optimalEntryIndex);
  rrcLinkData.bestAoDElevation = linkData->m_aodElevation.at(optimalEntryIndex);
  rrcLinkData.bestAoDAzimuth = linkData->m_aodAzimuth.at(optimalEntryIndex);
  rrcLinkData.bestAoAElevation = linkData->m_aoaElevation.at(optimalEntryIndex);
  rrcLinkData.bestAoAAzimuth = linkData->m_aoaAzimuth.at(optimalEntryIndex);

  NS_ASSERT_MSG(m_lteRnti.find(imsi) != m_lteRnti.end(), "In ForwardUePositionReportToREM: lteRnti not found for IMSI "<< imsi);
  uint16_t ueRnti = m_lteRnti.find(imsi)->second;

  // Tell UE to use the best beam
  m_rrcSapUser->SendRemBeamFromLTECoordinator(ueRnti, rrcLinkData);
}

void
LteEnbRrc::HandoverToBestCell(uint64_t imsi, uint16_t targetCell)
{
  // Schedule a handover event or modify existing one
  uint8_t millisecondsToHandover = m_minDynTttValue; // FIXME: 25 ms is default. Is it ok?
  EventId scheduledHandoverEvent = Simulator::Schedule(MilliSeconds(millisecondsToHandover), &LteEnbRrc::PerformHandover, this, imsi);
  LteEnbRrc::HandoverEventInfo handoverInfo;
  handoverInfo.sourceCellId = m_lastMmWaveCell[imsi];
  handoverInfo.targetCellId = targetCell;
  handoverInfo.isLoadBalancing = false; // This is not a handover due to a load balancing action
  handoverInfo.isWithoutSweep = true; // We do not want to sweep, as we will pass a beam id
  handoverInfo.scheduledHandoverEvent = scheduledHandoverEvent;
  HandoverEventMap::iterator handoverEvent = m_imsiHandoverEventsMap.find(imsi);
  if(handoverEvent != m_imsiHandoverEventsMap.end()) // another event was scheduled, but it was already deleted. Replace the entry
  {
    handoverEvent->second = handoverInfo;
  }
  else
  {
    m_imsiHandoverEventsMap.insert(std::pair<uint64_t, HandoverEventInfo> (imsi, handoverInfo));
  }
}

size_t
LteEnbRrc::FindIndexForCurrentServingCell(Ptr<LinkData> linkData, uint16_t servingCell)
{
  // Extract best known beam ID
  ns3::LinkData currentSuboptimalGnbLinkData;
  size_t index = 0;
  size_t bestIndex = 0;
  std::vector<size_t> currentCellIndices;

  // Iterate over txId
  for (auto cellId : linkData->m_txId)
  {
    if (cellId == servingCell)
    {
      currentCellIndices.push_back(index);
    }
    ++index;
  }

  double pathLoss = 42; // Dummy value
  if (currentCellIndices.size() != 0)
  {
    // Initialize with PL value of first match and set the best index to this entry.
    pathLoss = linkData->m_pathloss.at(currentCellIndices.front());
    bestIndex = currentCellIndices.front();
  }
  else
  {
    NS_LOG_DEBUG(" Cell " << servingCell << " does not seem to have entries in REM. This should not happen.");
    return bestIndex;
  }
  for (const size_t& index : currentCellIndices)
  {
    if(linkData->m_pathloss.at(index) > pathLoss)
    {
      bestIndex = index;
      pathLoss = linkData->m_pathloss.at(index);
    }
  }
  return bestIndex;
}

void 
LteEnbRrc::ForwardUePositionReportToREM(LteRrcSap::PositionReport msg)
{ 
  // msg contains the real UE position that is not affected by any errrors.
  
  //uint16_t rnti = DoGetRntiFromImsi(msg.imsi); // This would give RNTI for gNB
  uint16_t rnti = m_lteRnti.find(msg.imsi)->second;
  if (rnti == 0)
  {
    // This is the case when UE is in IA
    NS_LOG_UNCOND("LteEnbRrc::ForwardUePositionReportToREM: recevied report from an unknown LTE RNTI 0.");
    return;
  }

  if (m_perImsiPosReportCounter.find(msg.imsi) == m_perImsiPosReportCounter.end())
  {
    // This is the first ever position report from this IMSI.
    // Vectors are 0-indexed, so we insert a 0 instead of 1.
    m_perImsiPosReportCounter.emplace(msg.imsi, 0);
  }

  double xErrorValue{0.0}; 
  double yErrorValue{0.0};

  UeCompletePositionReport posReport;
  RemQueryRecoveryAlgorithmTraceParams remRecoveryTrace;
  remRecoveryTrace.imsi = msg.imsi;
  remRecoveryTrace.servingCellId = msg.cellId;
  remRecoveryTrace.recoveryIsEnabled = m_enableRemRecoveryAlgorithm;

  if (m_useErroneousUePosition)
  {
    xErrorValue = m_latitudeErrorModels.at(msg.imsi)->GetNextValue();
    yErrorValue = m_longitudeErrorModels.at(msg.imsi)->GetNextValue();
    // z-value is left as is, because we have a 2D REM data anyways
  }

  posReport = UeCompletePositionReport{
      msg.imsi, 
      xErrorValue,
      yErrorValue,
      msg.uePosition.x,
      msg.uePosition.y,
      (msg.uePosition.x + xErrorValue),
      (msg.uePosition.y + yErrorValue)};
  m_uePositionTrace(posReport);

  UePositionReportError locationError{msg.imsi, xErrorValue, yErrorValue};
  m_locationErrorTrace(locationError);
  

  Vector3D intPosition;
  // std::round is used in ThreeGppChannelModel
  intPosition.x = std::round(msg.uePosition.x + xErrorValue);
  intPosition.y = std::round(msg.uePosition.y + yErrorValue);
  intPosition.z = 1.5; // FIXME: fetch from argument, do not hardcode

  // If UE is not associated with a gNB cell, we still command a BPL to UE and gNB.
  // NrUePhy will attach to the cell that REM marks as the best one.
  if (m_mmWaveCellSetupCompleted.empty() || (m_mmWaveCellSetupCompleted.find(msg.imsi) == m_mmWaveCellSetupCompleted.end()))
  {
    // UE is in IA
    auto it = m_allLinkData.find(intPosition);
    if (it != m_allLinkData.end() ) // This will fail for measurement REM for some positions
    {
      Ptr<LinkData> linkData = it->second;
      if (linkData->m_path == 0)
      {
        // REM point is inside a building. Skip this iteration, as UE is not connected and we
        // can not execute the currently impelemnted recovery algorithm based on the Bresenham's line
        // algorithm-based cell selection due to lack of serving gNB.
        // IA sweep will be triggered on UE side.
        NS_LOG_UNCOND("LteEnbRrc::ForwardUePositionReportToREM: UE not connected to gNB, can not recover from an empty REM entry. Skipping this report. UE will continue sweeping.");
        return;
      }

      size_t lowestPlIndex = FindLowestPlIndex(linkData->m_pathloss);
      SendBeamsFromCoordinator(msg.imsi, lowestPlIndex, linkData); 
    }

    return;
  }

  bool beamOffsetAlreadyTraced = false;

  // Trace the reported coords that will be used for REM querying.

  remRecoveryTrace.reportedCoords = intPosition;
  // Check if REM contains an entry with the received UE position
  auto it = m_allLinkData.find(intPosition);
  if (it != m_allLinkData.end())
  {
    // Get the actual REM entry
    Ptr<LinkData> linkData = it->second;
    if (linkData->m_path == 0)
    {
      // We could not query the REM at original reported position
      remRecoveryTrace.remPointAvailableForQueriedPosition = false;

      if (!m_enableRemRecoveryAlgorithm)
      {
        NS_LOG_UNCOND("LTE CO: Could not query REM for position of IMSI " << msg.imsi << ". Recovery disabled. Skipping this iteration.");
        remRecoveryTrace.remPointRecovered = false;
        m_remRecoveryAlgoTrace(remRecoveryTrace);
        return;
      }

      std::pair<Vector, Ptr<LinkData>> linkDataPair = FindNonEmptyRemEntry(msg.cellId, intPosition.x, intPosition.y);
      linkData = linkDataPair.second;
      // It is not guaranteed that FindNonEmptyRemEntry will return a non-emtpy entry
      if (linkData == nullptr)
      {
        NS_LOG_UNCOND("Bresenham's algorithm reached the position of gNB without finding a proper REM point."
        " Abort processing of the current UE position report.");
        remRecoveryTrace.remPointRecovered = false;
        m_remRecoveryAlgoTrace(remRecoveryTrace);
        return;
      }
      NS_LOG_UNCOND("Now using the REM entry that is the result of Bresenham's Line Algo-based selection");
      remRecoveryTrace.remPointRecovered = true;
      remRecoveryTrace.recoveredCoords = linkDataPair.first;
      // Pass the position that the recovery algo has found.
      TraceBeamIdSelection(msg.imsi, msg.cellId, msg.uePosition, linkDataPair.first);
      beamOffsetAlreadyTraced = true;
    }
    else
    {
      // The queried position is available in REM
      remRecoveryTrace.remPointAvailableForQueriedPosition = true;
    }

    m_remRecoveryAlgoTrace(remRecoveryTrace);

    if (!beamOffsetAlreadyTraced)
    {
      // Recovery algorithm was not triggered. We will query REM at the intPosition.
      TraceBeamIdSelection(msg.imsi, msg.cellId, msg.uePosition, intPosition);
    }

    std::map<uint16_t, std::pair<uint16_t, double>> estimatedSinr
        = remSinrEstimator.EstimateSinrForUeLocation(linkData, Vector{intPosition.x, intPosition.y, intPosition.z});

    uint16_t lowestPlIndex = 0;
    double bestSnr = -200.;
    for (const auto& [gnbId, indexSnrPair] : estimatedSinr)
    {
      if (indexSnrPair.second >= bestSnr)
      {
        bestSnr = indexSnrPair.second;
        lowestPlIndex = indexSnrPair.first;
      }
    }

    // If REM reports that UE is not served by the best gNB,
    // check SNR thresholds to decide whether to leave things as they are, switch beams or handover

    if (linkData->m_txId.at(lowestPlIndex) != msg.cellId)
    {
      // We want to immediatelly switch to the best cell according to REM
      if (m_immediateREMLABHandovers)
      {
        HandoverToBestCell(msg.imsi, linkData->m_txId.at(lowestPlIndex));
        // FIXME: what if handover fails because the cell is busy? Where to send the beam command?
        SendBeamsFromCoordinator(msg.imsi, lowestPlIndex, linkData); 
        return;
      }
      // Check recently reported SINR for the current UE
      double currentSinrDb = 0;
      if(m_lastMmWaveCell.find(msg.imsi) != m_lastMmWaveCell.end())
      {
        currentSinrDb = 10*std::log10(m_imsiCellSinrMap.find(msg.imsi)->second[m_lastMmWaveCell[msg.imsi]]);
        if (currentSinrDb > m_maxRateThreshold)
        {
          // Estimate SINR, identify the entry from current serving cell and use it.
          size_t bestIndex = std::numeric_limits<size_t>::max();
          for (const auto& [gnbId, indexSnrPair] : estimatedSinr)
          {
            if (gnbId == msg.cellId)
            {
              // This is the best beam for the current serving gNB
              bestIndex = indexSnrPair.first;
              bestSnr = indexSnrPair.second;
            }
          }
          // Only change BPL if we actually had an entry for the serving gNB
          if (bestIndex != std::numeric_limits<size_t>::max())
          {
            // Found the best PL among those entries.
            // Now create a LinkData structure, extract values and send to gNB, UE
            SendBeamsFromCoordinator(msg.imsi, bestIndex, linkData);
          }
          if (m_imsiHandoverHysterisisCounter.find(msg.imsi) != m_imsiHandoverHysterisisCounter.end())
          {
            m_imsiHandoverHysterisisCounter.at(msg.imsi) = 0;
          }
        }
        // If above beamsweepthreshold, might try to switch to second-best beam from this gnb according to REM?
        else if (currentSinrDb > 10) // 10 dB is the beam sweep threshold. NrUePhy::m_beamSweepThreshold
        {
          // Check how long the current IMSI is in this SNR range. If more than 3 psoitions, command a handover.
          if (m_imsiHandoverHysterisisCounter.find(msg.imsi) == m_imsiHandoverHysterisisCounter.end())
          {
            // This is the first time this IMSI ends up in this SNR range
            m_imsiHandoverHysterisisCounter[msg.imsi] = 1;
          }
          else
          {
            ++m_imsiHandoverHysterisisCounter.at(msg.imsi);
          }

          if (m_imsiHandoverHysterisisCounter.at(msg.imsi) > m_handoverHysterisisMaxCounter)
          {
            // User is below max rate and above beam sweep for too long.
            // Better to switch the cell to the best one according to REM
            HandoverToBestCell(msg.imsi, linkData->m_txId.at(lowestPlIndex));

            // FIXME: what if handover fails because the cell is busy? Where to send the beam command?
            SendBeamsFromCoordinator(msg.imsi, lowestPlIndex, linkData); 

            m_imsiHandoverHysterisisCounter.at(msg.imsi) = 0;
            
            return;
          }


          // Extract best known beam for msg.cellId
          size_t bestIndex = FindIndexForCurrentServingCell(linkData, msg.cellId);

          // Found the best PL among those entries.
          // Now create a LinkData structure, extract values and send to gNB, UE
          SendBeamsFromCoordinator(msg.imsi, bestIndex, linkData);
        }
        // If below the beam sweep threshold, definitely need a handover to the best gNB (according to REM) to avoid RLF
        // We might be in RLF already
        else
        {
          // Schedule a handover event or modify existing one
          HandoverToBestCell(msg.imsi, linkData->m_txId.at(lowestPlIndex));

          // FIXME: what if handover fails because the cell is busy? Where to send the beam command?
          SendBeamsFromCoordinator(msg.imsi, lowestPlIndex, linkData); 

          if (m_imsiHandoverHysterisisCounter.find(msg.imsi) != m_imsiHandoverHysterisisCounter.end())
          {
            m_imsiHandoverHysterisisCounter.at(msg.imsi) = 0;
          }        
        }
      }
    }
    else
    {
      // UE is already at correct gNB according to REM
      SendBeamsFromCoordinator(msg.imsi, lowestPlIndex, linkData); 
    }
  }
  else if (m_allLinkData.size() == 0)
  {
    NS_LOG_DEBUG("REM is empty. Is this eNB not a coordinator?");
  }
  else 
  {
    remRecoveryTrace.remPointAvailableForQueriedPosition = false;
    // If we end up here, the walk is either outside of REM boundaries,
    // or the measurement error of REM left a gap in the grid. 
    // In the second case, we might want to try to still use the Bresenham's algorithm.
    // However, we can't really differentiate between those cases.
    if (!m_enableRemRecoveryAlgorithm)
    {
      NS_LOG_UNCOND("LTE CO: Position of IMSI " << msg.imsi
          << " is either ouside of REM boundaries, or REM does not contain an entry for this position. Recovery disabled. Skipping this iteration.");
      remRecoveryTrace.remPointRecovered = false;
      m_remRecoveryAlgoTrace(remRecoveryTrace);
      return;
    }

    NS_LOG_DEBUG("   NO REM point for UE position :( Trying to recover");
    std::pair<Vector, Ptr<LinkData>> linkDataPair = FindNonEmptyRemEntry(msg.cellId, intPosition.x, intPosition.y);
    
    if (linkDataPair.second == nullptr)
    {
      NS_LOG_DEBUG("   Could not recover. Skipping this iteration without tracing and commanding");
      remRecoveryTrace.remPointRecovered = false;
      m_remRecoveryAlgoTrace(remRecoveryTrace);
      return;
    }
    remRecoveryTrace.remPointRecovered = true;
    remRecoveryTrace.recoveredCoords = linkDataPair.first;
    m_remRecoveryAlgoTrace(remRecoveryTrace);

    // We were able to successfully recover. Now need to check whether we are served by the best cell.

    size_t lowestPlIndex = FindLowestPlIndex(linkDataPair.second->m_pathloss);
    std::cout << " Recovered: found lowestPlIndex: " << lowestPlIndex << std::endl;
    // std::map<uint16_t, std::pair<uint16_t, double>> estimatedSinr
    //   = remSinrEstimator.EstimateSinrForUeLocation(linkDataPair.second, Vector{linkDataPair.first.x, linkDataPair.first.y, linkDataPair.first.z});

    // uint16_t lowestPlIndex = 0;
    // double bestSnr = -200.;
    // for (const auto& [gnbId, indexSnrPair] : estimatedSinr)
    // {
    //   std::cout << " + gNB " << gnbId << " SNR " <<  indexSnrPair.second << std::endl;
    //   if (indexSnrPair.second >= bestSnr)
    //   {
    //     bestSnr = indexSnrPair.second;
    //     lowestPlIndex = indexSnrPair.first;
    //   }
    // }
    // std::cout << Simulator::Now().GetSeconds() << ": LteEnbRrc: best SNR estimated to be " << bestSnr
    //     << " from gNB " << linkDataPair.second->m_txId.at(lowestPlIndex)
    //     << " on REM index " << lowestPlIndex << std::endl;
    // std::cout << "   best AoA azimuth: " << linkDataPair.second->m_aoaAzimuth.at(lowestPlIndex) << std::endl;
    // std::cout << "   best AoA elevation: " << linkDataPair.second->m_aoaElevation.at(lowestPlIndex) << std::endl;
    // std::cout << "   best AoD azimuth: " << linkDataPair.second->m_aodAzimuth.at(lowestPlIndex) << std::endl;
    // std::cout << "   best AoD elevation: " << linkDataPair.second->m_aodElevation.at(lowestPlIndex) << std::endl;
    // std::cout << "   best PL : " << linkDataPair.second->m_pathloss.at(lowestPlIndex) << std::endl;

    // Essentially repeat the whole algorithm.
    // linkDataPair.second contains a valid REM entry for the recovered position
    if (linkDataPair.second->m_txId.at(lowestPlIndex) == msg.cellId)
    {
      // Same serving cell, no need for HO related logic
      SendBeamsFromCoordinator(msg.imsi, lowestPlIndex, linkDataPair.second);
      TraceBeamIdSelection(msg.imsi, msg.cellId, msg.uePosition, linkDataPair.first);
    }
    else
    {
      if (m_immediateREMLABHandovers)
      {
        NS_LOG_DEBUG("  Will handover from cell "<< msg.cellId << " to cell " << linkDataPair.second->m_txId.at(lowestPlIndex));
        HandoverToBestCell(msg.imsi, linkDataPair.second->m_txId.at(lowestPlIndex));
        SendBeamsFromCoordinator(msg.imsi, lowestPlIndex, linkDataPair.second); 
        TraceBeamIdSelection(msg.imsi, msg.cellId, msg.uePosition, linkDataPair.first);
        return;
      }
      else
      {
        // Check recently reported SINR for the current UE
        double currentSinrDb = 0;
        if(m_lastMmWaveCell.find(msg.imsi) != m_lastMmWaveCell.end())
        {
          currentSinrDb = 10*std::log10(m_imsiCellSinrMap.find(msg.imsi)->second[m_lastMmWaveCell[msg.imsi]]);

          if (currentSinrDb > m_maxRateThreshold)
          {
            NS_LOG_DEBUG("   SINR is good enough, we do not need handover and also may continue using the current BeamID");
          }
          else if (currentSinrDb > 10)
          {
            // Check how long the current IMSI is in this SNR range. If more than 3 psoitions, command a handover.
            if (m_imsiHandoverHysterisisCounter.find(msg.imsi) == m_imsiHandoverHysterisisCounter.end())
            {
              // This is the first time this IMSI ends up in this SNR range
              m_imsiHandoverHysterisisCounter[msg.imsi] = 1;
            }
            else
            {
              ++m_imsiHandoverHysterisisCounter.at(msg.imsi);
            }

            if (m_imsiHandoverHysterisisCounter.at(msg.imsi) > m_handoverHysterisisMaxCounter)
            {
              // User is below max rate and above beam sweep for too long.
              // Better to switch the cell to the best one according to REM
              HandoverToBestCell(msg.imsi, linkDataPair.second->m_txId.at(lowestPlIndex));
              SendBeamsFromCoordinator(msg.imsi, lowestPlIndex, linkDataPair.second); 
              TraceBeamIdSelection(msg.imsi, msg.cellId, msg.uePosition, linkDataPair.first);
              // Reset the hysterisis counter for this IMSI.
              m_imsiHandoverHysterisisCounter.at(msg.imsi) = 0;
              
              return;
            }

            // Extract best known beam for msg.cellId
            size_t bestIndex = FindIndexForCurrentServingCell(linkDataPair.second, msg.cellId);
            SendBeamsFromCoordinator(msg.imsi, lowestPlIndex, linkDataPair.second); 
            TraceBeamIdSelection(msg.imsi, msg.cellId, msg.uePosition, linkDataPair.first);
          }
          // If below the beam sweep threshold, definitely need a handover to the best gNB (according to REM) to avoid RLF
          else
          {
            // Schedule ahandover event or modify existing one
            HandoverToBestCell(msg.imsi, linkDataPair.second->m_txId.at(lowestPlIndex));
            SendBeamsFromCoordinator(msg.imsi, lowestPlIndex, linkDataPair.second);
            TraceBeamIdSelection(msg.imsi, msg.cellId, msg.uePosition, linkDataPair.first);
          }
        }
      }
    }
  }
}

// For the given vector of values, eturn the index with the lowest Path Loss value
size_t
LteEnbRrc::FindLowestPlIndex(doubleVector_t pathLossData)
{
  double lowestPl = -1000;
  size_t index = 0;
  size_t bestIndex = 0;

  for (auto pl : pathLossData)
  {
    if (pl > lowestPl)
    {
      lowestPl = pl;
      bestIndex = index;
    }
    ++index;
  }

  return bestIndex;
}

// FIXME: try to avoid code duplicaiton and hardcoded files here.
// However, passing ListPositionAllocator does not really help, as we can not iterate over the uderlying container:/
void
LteEnbRrc::StoreGnbPositions(boost::optional<std::vector<uint16_t>> gnbCellIds)
{
  std::string input_folder = "src/nr/model/Raytracing/";
  std::string enbFile = input_folder + "enb_locations.txt";
  std::ifstream file1;
  file1.open (enbFile.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file1.good (), "File containing ENB locations not found");

  std::string line;
  std::string token;
  uint16_t cellId = 1;
  uint16_t lineCounter = 0;
  while (std::getline (file1, line))
  {
    doubleVector_t lineElements;
    std::istringstream stream (line);

    while (getline (stream, token, ','))
    {
      double sigma = 0.00;
      std::stringstream stream (token);
      stream >> sigma;
      lineElements.push_back (sigma);
    }

    if(gnbCellIds)
    {
      // If we passed gNB cell IDs, then we want a "custom" mapping between entries in the container that 
      // stores gNB positions and their Cell IDs.
      m_gNBpositions.emplace(gnbCellIds.get().at(lineCounter), Vector (lineElements.at (0) + m_gnbLocationOffsetX, lineElements.at (1) + m_gnbLocationOffsetY, lineElements.at (2)));
      lineCounter++;
    }
    else
    {
      m_gNBpositions.emplace(cellId, Vector (lineElements.at (0) + m_gnbLocationOffsetX, lineElements.at (1) + m_gnbLocationOffsetY, lineElements.at (2)));
      cellId++;
    }
  }

  remSinrEstimator.SetGnbLocations(m_gNBpositions);
  remSinrEstimator.InitializeAllParameters();
}

std::pair<Vector, Ptr<LinkData>>
LteEnbRrc::FindNonEmptyRemEntry(const uint16_t servingBS, int uePosX, int uePosY)
{
  // Extract the BS position.
  // TODO: need to store that globally or pass to CO somehow.
  int bsPosX = 0;
  int bsPosY = 0;

  Vector gNBpos = m_gNBpositions.at(servingBS);
  bsPosX = gNBpos.x;
  bsPosY = gNBpos.y;

  // https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  // https://zingl.github.io/Bresenham.pdf listing 2
  int dx = std::abs(bsPosX - uePosX);
  int stepX = uePosX < bsPosX ? 1 : -1;
  int dy = -std::abs(bsPosY - uePosY);
  int stepY = uePosY < bsPosY ? 1 : -1;
  int error = dx + dy;
  int error2{0};

  while (true)
  {
    if (uePosX == bsPosX && uePosY == bsPosY)
    {
      break;
    }

    error2 = error * 2;
    if (error2 >= dy)
    {
      error += dy;
      uePosX += stepX;
    }
    if (error2 <= dx)
    {
      error += dx;
      uePosY += stepY;
    }

    // Query REM at the new point
    Vector intPosition;
    intPosition.x = uePosX;
    intPosition.y = uePosY;
    intPosition.z = 1.5; // FIXME: fetch from argument, do not hardcode
    // Query REM for real position
    auto it = m_allLinkData.find(intPosition);
    if (it != m_allLinkData.end() && it->second->m_path != 0)
    {
      return std::make_pair(intPosition, it->second);
    }
  }
  // Return nullptr as we did not find a proper REM entry
  return std::make_pair(Vector(uePosX, uePosY, 1.5), nullptr);
}

// FIXME: need to consider whether we query different gnb for erroneous position
void
LteEnbRrc::TraceBeamIdSelection(
    const uint64_t imsi,
    const uint16_t servingCellId,
    const Vector3D& realPosition,
    const Vector3D& errPosition,
    boost::optional<size_t> suboptimalBeamIndex)
{
  RemBeamSelectionTraceParams traceParams;
  traceParams.imsi = imsi;
  traceParams.servingCellId = servingCellId;

  // Initialize with invalid values so that we can detect whether REM querying failed.
  int gnbSectorOffset{-1};
  int gnbElevationOffset{-1};
  int ueSectorOffset{-1};
  int ueElevationOffset{-1};

  Vector3D intPosition;
  intPosition.x = std::round(realPosition.x);
  intPosition.y = std::round(realPosition.y);
  intPosition.z = 1.5; // FIXME: fetch from argument, do not hardcode

  // Query ideal REM for real position. This is expected to never fail if the walk path was constructed correctly.
  auto it = m_idealLinkData.find(intPosition);
  if (it != m_idealLinkData.end())
  {
    Ptr<LinkData> linkData = it->second;
    // REM point is inside a building
    if (linkData->m_path == 0)
    {
      // This is expected to never happen for ideal REM and ideal position.
      return;
    }

    size_t lowestPlIndex = FindLowestPlIndex(linkData->m_pathloss);
    // std::map<uint16_t, std::pair<uint16_t, double>> estimatedSinr
    //   = remSinrEstimator.EstimateSinrForUeLocation(linkData, Vector{intPosition.x, intPosition.y, intPosition.z});

    // uint16_t lowestPlIndex = 0;
    // double bestSnr = -200.;
    // for (const auto& [gnbId, indexSnrPair] : estimatedSinr)
    // {
    //   std::cout << " + gNB " << gnbId << " SNR " <<  indexSnrPair.second << std::endl;
    //   if (indexSnrPair.second >= bestSnr)
    //   {
    //     bestSnr = indexSnrPair.second;
    //     lowestPlIndex = indexSnrPair.first;
    //   }
    // }

    traceParams.bestCellId = linkData->m_txId.at(lowestPlIndex);
    double bestAoDElevation = linkData->m_aodElevation.at(lowestPlIndex);
    double bestAoDAzimuth = linkData->m_aodAzimuth.at(lowestPlIndex);
    double bestAoAElevation = linkData->m_aoaElevation.at(lowestPlIndex);
    double bestAoAAzimuth = linkData->m_aoaAzimuth.at(lowestPlIndex);
 
    // Map to gNB and UE beams
    uint16_t remSectorGnb{0};
    double remElevationGnb{90.};
    for (const auto& entry : m_gnbSectorDegreeMap)
    {
      if (bestAoDAzimuth >= entry.first.lowerBoundDeg && bestAoDAzimuth <= entry.first.upperBoundDeg)
      {
        remSectorGnb = entry.second;
        gnbSectorOffset = entry.first.sectorIndex;
        break;
      }
    }

    for (const auto& entry : m_gnbElevationDegreeMap)
    {
      // negative angles for gNB
      if (bestAoDElevation <= entry.first.first && bestAoDElevation >= entry.first.second)
      {
        remElevationGnb = entry.second;
        break;
      }
    }
    traceParams.idealPositionGnbRemBeam = BeamId{remSectorGnb, remElevationGnb};
    uint16_t remSectorUe{0};
    double remElevationUe{90.};
    for (const auto& entry : m_ueSectorDegreeMap)
    {
      if (bestAoAAzimuth >= entry.first.lowerBoundDeg && bestAoAAzimuth <= entry.first.upperBoundDeg)
      {
        remSectorUe = entry.second;
        ueSectorOffset = entry.first.sectorIndex;
        break;
      }
    }

    for (const auto& entry : m_ueElevationDegreeMap)
    {
      // positive angles for UE
      if (bestAoAElevation >= entry.first.first && bestAoAElevation <= entry.first.second)
      {
        remElevationUe = entry.second;
        break;
      }
    }
    traceParams.idealPositionUeRemBeam = BeamId{remSectorUe, remElevationUe};
  }

  // Query REM for erroneous position
  // Map to gNB and UE beams
  Vector3D intErrPosition;
  intErrPosition.x = std::round(errPosition.x);
  intErrPosition.y = std::round(errPosition.y);
  intErrPosition.z = 1.5; // FIXME: fetch from argument, do not hardcode
  // Query measurement REM for erroneous position
  it = m_allLinkData.find(intErrPosition);
  if (it != m_allLinkData.end())
  {
    Ptr<LinkData> linkData = it->second;
    if (linkData->m_path == 0) // REM point is inside of an obstacle
    {
      return;
    }
    // If we do not search for the best cell, find the best beam from the serving cell 
    size_t lowestPlIndex;
    if (suboptimalBeamIndex)
    {
      // Find the index for best BPLs for the currently serving gNB
      lowestPlIndex = suboptimalBeamIndex.get();
    }
    else
    {
      // Find the index of best gNB
      lowestPlIndex = FindLowestPlIndex(linkData->m_pathloss);
    //   std::map<uint16_t, std::pair<uint16_t, double>> estimatedSinr
    //   = remSinrEstimator.EstimateSinrForUeLocation(linkData, Vector{intErrPosition.x, intErrPosition.y, intErrPosition.z});

    //   double bestSnr = -200.;
    //   for (const auto& [gnbId, indexSnrPair] : estimatedSinr)
    //   {
    //     std::cout << " + gNB " << gnbId << " SNR " <<  indexSnrPair.second << std::endl;
    //     if (indexSnrPair.second >= bestSnr)
    //     {
    //       bestSnr = indexSnrPair.second;
    //       lowestPlIndex = indexSnrPair.first;
    //     }
    //   }
    }

    traceParams.bestCellId = linkData->m_txId.at(lowestPlIndex);
    double bestAoDElevation = linkData->m_aodElevation.at(lowestPlIndex);
    double bestAoDAzimuth = linkData->m_aodAzimuth.at(lowestPlIndex);
    double bestAoAElevation = linkData->m_aoaElevation.at(lowestPlIndex);
    double bestAoAAzimuth = linkData->m_aoaAzimuth.at(lowestPlIndex);

    // Map to gNB and UE beams
    uint16_t remSectorGnb{0};
    double remElevationGnb{90.};
    for (const auto& entry : m_gnbSectorDegreeMap)
    {
      if (bestAoDAzimuth >= entry.first.lowerBoundDeg && bestAoDAzimuth <= entry.first.upperBoundDeg)
      {
        remSectorGnb = entry.second;
        // Calculate the sector index offset between ideal and erroneous position
        int offset = std::abs(std::round(gnbSectorOffset - entry.first.sectorIndex));

        // Normalize sectors offset to go over "shortest path" to the correct sector.
        if (offset > 20)
          offset = 40 - offset; // we have 40 sectors in total. FIXME: should be configurable

        gnbSectorOffset = offset;

        break;
      }
    }

    for (const auto& entry : m_gnbElevationDegreeMap)
    {
      if (bestAoDElevation <= entry.first.first && bestAoDElevation >= entry.first.second)
      {
        remElevationGnb = entry.second;
        break;
      }
    }
    traceParams.errPositionGnbRemBeam = BeamId{remSectorGnb, remElevationGnb};
    uint16_t remSectorUe{0};
    double remElevationUe{90.};
    for (const auto& entry : m_ueSectorDegreeMap)
    {
      if (bestAoAAzimuth >= entry.first.lowerBoundDeg && bestAoAAzimuth <= entry.first.upperBoundDeg)
      {
        remSectorUe = entry.second;
        // Calculate the sector index offset between ideal and erroneous position
        int offset = std::abs(std::round(ueSectorOffset - entry.first.sectorIndex));

        // Normalize sectors offset to go over "shortest path" to the correct sector.
        if (offset > 10)
          offset = 20 - offset; // we have 20 sectors in total. FIXME: this should be configurable 

        ueSectorOffset = offset;
        break;
      }
    }

    for (const auto& entry : m_ueElevationDegreeMap)
    {
      if (bestAoAElevation >= entry.first.first && bestAoAElevation <= entry.first.second)
      {
        remElevationUe = entry.second;
        break;
      }
    }
    traceParams.errPositionUeRemBeam = BeamId{remSectorUe, remElevationUe};
  }
  else
  {
    // The REM does not contain an entry for the erroneous position.
    // We should not end up here. If the recovery algortihm has found an entry, it will be valid.
    // If not, we would not end up in this function, as the algortihm execution is aborted for that case.
    NS_LOG_DEBUG("LteEnbRrc::TraceBeamIdSelection: we should not end up here!");
  }

  traceParams.gnbSectorOffset = gnbSectorOffset;
  traceParams.ueSectorOffset = ueSectorOffset;

  // Write trace params only if valid offsets were calculated
  if (gnbSectorOffset != -1 && ueSectorOffset != -1)
  {
    m_remBeamSelectionTrace(traceParams);
  }
  else
  {
    NS_LOG_UNCOND("LteEnbRrc::TraceBeamIdSelection: could not query the REM for the actual position. No BeamID offsets can be extracted.");
  }
}

} // namespace ns3