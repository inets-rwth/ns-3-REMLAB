#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include "ns3/nr-helper.h"
#include <ns3/buildings-helper.h>
#include "ns3/log.h"
#include <ns3/buildings-module.h>
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/lte-helper.h"
#include "ns3/nr-module.h"
#include "ns3/three-gpp-http-helper.h"

#include <string>
#include <vector>
#include <dirent.h>
#include <fstream>
#include <regex>
#include <cstdint>

#include <sys/stat.h>

#include "ns3/random-variable-stream.h"

uint32_t runNumber = 1;
/*
**********************************************************************************************************
*   The following ns-3 code is based on the proposed work in the paper:
*   [1] A. Ichkov, A. Schott, P. Mähönen and L. Simić, 'flexRLM: Flexible Radio Link Monitoring
*       for Multi-User Downlink Millimeter-Wave Networks', to be presented in Proc. IEEE INFOCOM 2023.
*
*   This code enables multi-user downlink millimeter-wave end-to-end network simulations utilizing
*   5G-NR beam management downlink operations. It is based on the flexRLM framework, 
*   a coordinator-based flexible radio link monitoring (RLM) framework that enables joint 
*   beam management and low complexity load-balancing, as proposed in [1].
*
*   This code also enables single-user network simulations based on the proposed work in the paper:
*   [2] A. Ichkov, O. Atasoy, P. Mähönen and L. Simić, "Full-Stack ns-3 Framework for the 
*       Evaluation of 5G-NR Beam Management in Non-Standalone Downlink Millimeter-Wave Networks," 
*       in Proc. IEEE WoWMoM 2022, Belfast UK, June 2023. (https://ieeexplore.ieee.org/document/9842765)
*
**********************************************************************************************************
*   The following parameters are the most important for setting up the network simulations:
*
*     - simTime:      Duration of simulation in seconds
*     - numOfUes:     Due to the use of a ray-tracing channel model, for each UE there is
*                     #walkID and #walk-path information, which is stored in 
*                     /src/nr/model/Raytracing_UE_set. Additional UE sets can be found in
*                     /src/nr/model/UE_sets.
*                     The walk information, i.e. (X,Y) coordinates and walking speed, are 
*                     loaded during simulation initialization. We note that the number of UEs 
*                     further impacts the currently static assignment and scheduling of 
*                     Channel State Information-Reference Signals (CSI-RS), as detailed below.
*     - loadBalancing:  enables low complexity load blancing
*
*   A variety of parameters are introduced to configure the beam management operation, 
*   represented by three beam management strategies, as shown in Table I. 
*
*     - Ideal:          Ideal (instantaneous) beamforming/beam scanning and initial access, i.e. 
*                       no exchange on any beam management control messages. Exhaustive beam search 
*                       of candidate beam pair links (BPLs) during initial access is instantaneous 
*                       (completeSSBDuration=0.0) or set to a fixed value (completeSSBDuration=20.0).
*     - Default 5G-NR:  Time-frequency resource scheduling for transmission and reception
*                       of Synchronization Signal Blocks (SSBs) for the purpose of realistic beam
*                       sweep updates for initial access and Radio Link Failure (RLF) recovery.
*                       In addition to SSBs, CSI-RS control signals are scheduled for monitoring
*                       of alternative BPLs from the serving gNB to enable faster beam switching
*                       via Radio Link Monitoring (RLM). The monitored CSI-RSs are sent periodically 
*                       by the serving gNB over different BPLs, which are determined after a 
*                       successful SSB beam sweep and thus can become stale over time.
*                       Using the variable ssbRlmOn, we implement an additional update of the monitored 
*                       CSI-RSs based on the periodic SSB transmissions sent by the serving gNB.  
*                       Each connected UE performs beam sweeps during the periodic SSB transmissions to  
*                       keep track of long-term channel dynamics. The SSB measurement reports are then 
*                       used to update the monitored CSI-RSs for RLM. The CSI-RSs, which are transmitted 
*                       more frequently than the SSBs are thus used to capture short-term channel dynamics.
*     - flexRLM:        Instead of solely monitoring candidate BPLs from the serving gNB, flexRLM
*                       allows flexible configuration of the minimum number of BPLs to be monitored
*                       via CSI-RS control signals from the serving gNB (via minCSIRSFromServiceGnb).
*                       The remaining CSI-RS resources can be utilized to monitor candidate BPLs from 
*                       other candidate gNB to improve link stability and facilitate handover decisions.
*                       Additionally, the CSI-RS BPL information from alternative gNBs can be leveraged
*                       for load-balancing purpsoses via the central coordinator (via loadBalancing).
*
*
*                             Table I. Beam management strategies and configuration parameters
*                              _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
*                             |                  Beam management strategy                     |
*     Associated variables    |   Ideal   |    Default 5G-NR      |         flexRLM           |
*     ----------------------------------------------------------------------------------------|
*     realisticIA             |   false   |         true          |          true             | <- enable SSB
*     rlmOn                   |   false   |         true          |          true             | <- enable sending CSIRS
*     ssbRlmOn                |   false   |      true / false     |          true             | <- enable UE to analyse SSBs for update the CSI RS beam candidates
*     completeSSBDuration (ms)|   0 / 20  |           -           |           -               |
*     minCSIRSFromServiceGnb  |   -       |           4           |          0-3              |
*     loadBalancing           |   -       |           -           |      true / false         |
*
*   flexRLM implements a threshold based operations using two signal-to-noise ratio (SNR)
*   thresholds for Radio Link Failure (γ_RLF = -5 dB, via the variable RLFThreshold) and 
*   maximum achievable data rate based on the highest supported modulation and coding scheme 
*   (γ_MR = 22.7 dB, via the variable MRThreshold), which dictate the beam management events 
*   triggered directly by the UE or facilitated by the coordinator. 
*
*   The static assignment of CSI-RS resources for a given number of users is as follows 
*   (please refer to the reference paper in [1] for further information):
*
*      1 UE:    csiRSPeriodicity = 1
*     20 UEs:   csiRSPeriodicity = 23
*     50 UEs:   csiRSPeriodicity = 55
* 
*       with:   csiRSOffset = 3, 
*               maxCSIRSResourcesPerFrame = 2, 
*               noOfBeamsTbRLM = maxCSIRSResourcesPerFrame*2, 
*               ssbRLMTXDirections = 10; // same as number of gNBs
*
**********************************************************************************************************
*/

using namespace ns3;

// LOAD LOCATION INFORMATION OF ENBS

// The following offsets are relevant only for the case where REM and walk paths are in different
// coordinate systems. If this is not the case, set these offsets to 0.
const int enbXOffset = 0; // 128 from matlab plots of gNB positions on top of building outlines
const int enbYOffset = 0; // 121 from matlab plots of gNB positions on top of building outlines

void
LoadEnbLocations (Ptr<ListPositionAllocator> enbPositionAlloc)
{
  std::string input_folder = "src/nr/model/Raytracing/";
  std::string enbFile = input_folder + "enb_locations.txt";
  std::ifstream file1;
  file1.open (enbFile.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file1.good (), "File containing ENB locations not found");

  std::cout << "Offset for gNB locations: " <<  Vector(enbXOffset, enbYOffset, 0) << std::endl;

  std::string line;
  std::string token;
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
      enbPositionAlloc->Add (
          Vector (lineElements.at (0) + enbXOffset, lineElements.at (1)  + enbYOffset, lineElements.at (2)));
      NS_LOG_UNCOND (Vector (lineElements.at (0) + enbXOffset, lineElements.at (1) + enbYOffset, lineElements.at (2)));
    }
}

// IP THROUGHPUT AND DELAY LOGGING
std::vector<double> totalBytesReceived;
std::vector<double> totalTimeElapsed;
std::vector<int> numPacketsReceived;
std::vector<double> totalDelay;

static void
GetThroughput (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon,
               std::vector<Ptr<OutputStreamWrapper>> vectorOfStream)
{
  flowMon->CheckForLostPackets ();
  std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats ();
  Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier ());

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin ();
       stats != flowStats.end (); ++stats)
    {
      Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);

      if (fiveTuple.sourceAddress != "1.0.0.2")
        {
          continue;
        }
      //change latter IP address to allow for more users
      uint16_t streamIndex; 
      if(Ipv4Address("7.0.0.1") < fiveTuple.destinationAddress && fiveTuple.destinationAddress < Ipv4Address("7.0.0.102"))
      {
        uint8_t buf[4];
        fiveTuple.destinationAddress.Serialize(buf);
        streamIndex = buf[3]-2;
      }else{
        NS_LOG_UNCOND("Stream Index out of range");
      }  

      *(vectorOfStream.at(streamIndex))->GetStream () << Simulator::Now ().GetSeconds () << "\t"
                                            << (stats->second.rxBytes - totalBytesReceived[streamIndex]) * 8.0 /
                                                (stats->second.timeLastRxPacket.GetSeconds () - totalTimeElapsed[streamIndex])
                                            << "\t"
                                            << (stats->second.delaySum.GetSeconds () - totalDelay[streamIndex]) /
                                                (stats->second.rxPackets - numPacketsReceived[streamIndex])
                                            << std::endl;
      totalBytesReceived[streamIndex] = stats->second.rxBytes;
      totalDelay[streamIndex] = stats->second.delaySum.GetSeconds ();
      numPacketsReceived[streamIndex] = stats->second.rxPackets;
      totalTimeElapsed[streamIndex] = stats->second.timeLastRxPacket.GetSeconds ();
    }

  // schedule logging every 1 ms
  Simulator::Schedule (Seconds (0.001), &GetThroughput, fmhelper, flowMon, vectorOfStream);
}

// IP LATENCY LOGGING
static void
GetIpLatency (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p , Ptr<Ipv4> ipv4, uint32_t interface)
{
  Ptr<Packet> packet = p->Copy ();
  Ipv4Header ipHeader;
  packet->PeekHeader (ipHeader);
  // Logging from the IP header: payloadSize, identification, TOS, TTL, protocol, flags, source IP, dst IP, checksum, headerSize
  IpTimestampTag ipTimestampTag;
  packet->RemovePacketTag (ipTimestampTag);
  // get per packet latency in us.
  uint32_t perPacketLatency = (uint32_t)((Simulator::Now().GetSeconds() - ipTimestampTag.GetIpTimestamp())*1e6); 
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " \t" << ipHeader.GetIdentification() << " \t" << perPacketLatency << std::endl;
}

// UDP LATENCY LOGGING
static void
GetUdpLatency (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
  Ptr<Packet> packet = p->Copy ();
  //Logging from the UDP header: payloadSize, source Port, dst Port, source IP, dst IP, protocol, checksum
  UdpTimestampTag udpTimestampTag;
  packet->RemovePacketTag (udpTimestampTag);
  // get per packet latency in us.
  uint32_t perPacketLatency = (uint32_t)((Simulator::Now().GetSeconds() - udpTimestampTag.GetUdpTimestamp())*1e6); 
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " \t" << perPacketLatency << std::endl;
}

// UDP THROUGHPUT LOGGING
std::vector<double> totalUdpPacketsReceived;
std::vector<double> totalTimElapsedUdp;

static void
GetUdpThroughput (ApplicationContainer appContainer, std::vector<Ptr<OutputStreamWrapper>> vectorOfStream)
{
  for (uint32_t n = 0; n < appContainer.GetN(); n++)
  {
    auto tempReceivedPackets = (double)DynamicCast<UdpServer>(appContainer.Get(n))->GetReceivedBytes();
    auto lastPacketReceived = DynamicCast<UdpServer>(appContainer.Get(n))->GetLastReceivedTime().GetSeconds();

    *(vectorOfStream.at(n))->GetStream() << Simulator::Now().GetSeconds() << "\t"
                                         << (tempReceivedPackets - totalUdpPacketsReceived[n]) * 8.0 /
                                                (lastPacketReceived - totalTimElapsedUdp[n])
                                         << std::endl;

    totalUdpPacketsReceived[n] = tempReceivedPackets;
    totalTimElapsedUdp[n] = lastPacketReceived;
  }

  Simulator::Schedule (Seconds (0.04), &GetUdpThroughput, appContainer, vectorOfStream);
}

// Locaiton-aided Beamforming
std::vector<Ptr<OutputStreamWrapper>> streamVector_uePos;
static void
LogUePositionReport(std::string context, UeCompletePositionReport report)
{
  (void) context;
  // vectorOfStream contains entries for each UE. Note that vector entris are 0-indexed

  // Write all infos as CSV
  *(streamVector_uePos.at(report.ueImsi - 1))->GetStream() << Simulator::Now().GetSeconds() << ","
      << report.trueXpos << ","
      << report.trueYpos << ","
      << report.errXpos << ","
      << report.errYpos << ","
      << report.xError << ","
      << report.yError << std::endl;
}

// RemBeamSelectionTrace
// labf: logging of BeamId mismatch for erroneous positions
std::vector<Ptr<OutputStreamWrapper>> streamVector_beamOffsets;

static void
LogRemBeamIdOffets(std::string context, RemBeamSelectionTraceParams report)
{
  std::cout << "LogRemBeamIdOffets for imsi " << report.imsi << std::endl; 
  (void) context;
  // vectorOfStream contains entries for each UE. Note that vector entries are 0-indexed

  // Write all infos as CSV
  *(streamVector_beamOffsets.at(report.imsi - 1))->GetStream() << Simulator::Now().GetSeconds() << ","
      << (int)report.servingCellId << ","
      << (int)report.bestCellId << ","
      << report.idealPositionGnbRemBeam << ","
      << report.errPositionGnbRemBeam << ","
      << report.idealPositionUeRemBeam << ","
      << report.errPositionUeRemBeam << ","
      << (int)report.gnbSectorOffset << ","
      << (int)report.ueSectorOffset << std::endl;
}

// labf: logging of error applied on top of REM entries. For each cell it will store the error
// magnitude applied on top of the position
std::vector<Ptr<OutputStreamWrapper>> remMeasurementErrorOutputStream;
static void 
LogRemMeasurementError(std::string context, std::map<Vector, std::pair<double, double>> remMeasError)
{
  (void) context;
  std::cout << "LogRemMeasurementError received map of size " << remMeasError.size() << std::endl;
  for (const auto& [pos, entry] : remMeasError)
  {
    *(remMeasurementErrorOutputStream.front())->GetStream()
        << pos.x << "," << pos.y << "," << pos.z << std::endl // position
        << entry.first << "," << entry.second << std::endl    // x,y error applied to this position
        << std::sqrt(entry.first*entry.first + entry.second*entry.second) << std::endl; // error magnitude
  }
}

Ptr<OutputStreamWrapper> remNonEmptyEntriesStream;
static void 
LogRemNonEmptyEntries(std::string context, std::vector<Vector> remNonEmptyEntries)
{
  (void) context;
  std::cout << "LogRemNonEmptyEntries received vector of size " << remNonEmptyEntries.size() << std::endl;
  for (const auto& pos : remNonEmptyEntries)
  {
    *(remNonEmptyEntriesStream)->GetStream()
        << pos.x << "," << pos.y << "," << pos.z << std::endl; // position
  }
}

// For debugging purposes only. This is usable only with 1 error process. Does not properly consider all UEs.
std::vector<Ptr<OutputStreamWrapper>> ornsteinOutputTrace;
static void 
LogOrnsteinUhlenbeckParams(std::string context, OrnsteinUhlenbeckErrorTraceParams params)
{
  (void) context;
  *(ornsteinOutputTrace.front())->GetStream()
        << params.m_theta << "," << params.m_mu << "," << params.m_sigma << ","// parameters
        << params.m_deltaT << "," << params.m_deltaW  << ","  // x,y error applied to this position
        << params.m_Xprev << "," << params.m_Xnext  << ","  // x,y error applied to this position
        << params.m_restoringTerm << "," << params.m_stochasticTerm << std::endl;    // x,y error applied to this position
}

std::vector<Ptr<OutputStreamWrapper>> lteDelayStream;
static void
LogLteRrcDelay(std::string context, uint64_t imsi, int delay)
{
  // Vector entries are 0-indexed
  *(lteDelayStream.at(imsi-1))->GetStream()
      << Simulator::Now().GetSeconds() << "," << imsi << "," << delay << std::endl;
}

static void
LogRrcDelay(uint64_t imsi, int delay)
{
  std::cout << "LteDelay: UE IMSI: " << imsi << " delay: " << delay << std::endl;
}

std::vector<Ptr<OutputStreamWrapper>> remQueryRecoveryStream;
static void
LogRemQueryRecovery(std::string context, RemQueryRecoveryAlgorithmTraceParams params)
{
  double distanceBetweenReportedAndRecovered{0.};
  if (params.remPointRecovered)
  {
    distanceBetweenReportedAndRecovered = ns3::CalculateDistance(params.reportedCoords, params.recoveredCoords);
  }
  // Vector entries are 0-indexed
  *(remQueryRecoveryStream.at(params.imsi-1))->GetStream()
      << Simulator::Now().GetSeconds() << "," << params.imsi << "," << static_cast<int>(params.servingCellId)
      << "," << params.recoveryIsEnabled
      << "," << params.remPointAvailableForQueriedPosition
      << "," << params.reportedCoords.x << "," << params.reportedCoords.y << "," << params.reportedCoords.z
      << "," << params.remPointRecovered
      << "," << params.recoveredCoords.x << "," << params.recoveredCoords.y << "," << params.recoveredCoords.z
      << "," << distanceBetweenReportedAndRecovered
      << std::endl;
}

void
NotifyConnectionEstablishedUe (std::string context,
                               uint64_t imsi,
                               uint16_t cellid,
                               uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": connected to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": previously connected to CellId " << cellid
            << " with RNTI " << rnti
            << ", doing handover to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti,
                        uint16_t targetCellId)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

// labf: course change tracing

// Store last reported position per UE.
// Key is the UE Node ID.
// Value is a pair of UE Net Device ID and it corresponding position.
std::map<uint16_t, std::pair<uint16_t, Vector>> ueLastPosition;

std::regex pattern("/NodeList/(\\d+)/\\$ns3::MobilityModel/CourseChange");
std::smatch match;

NetDeviceContainer ueNetDev;

void
CourseChange(std::string context, Ptr<const MobilityModel> model)
{
  uint16_t ueNodeId{0};
  // Extract the UE Node ID.
  if (std::regex_search(context, match, pattern))
  {
    ueNodeId = static_cast<uint16_t>(std::stoi(match[1]));
  }
  else
  {
    NS_LOG_UNCOND("[main] CourseChange: could not extract Node ID of the UE's Mobility Model");
    return;
  }
  Vector position = model->GetPosition();
  // Compare the last position of this UE to the current one. If new, report to the LTE CO.
  if (position != ueLastPosition.at(ueNodeId).second)
  {
    NS_LOG_UNCOND(context <<
		  	  " x = " << position.x << ", y = " << position.y);
    std::cout << context << " x = " << position.x << ", y = " << position.y << std::endl;

    Ptr<LteUeRrc> ueRrc = DynamicCast<NrUeNetDevice>(ueNetDev.Get(ueLastPosition.at(ueNodeId).first))->GetRrc();
    std::cout << "UE " << ueNodeId << " is trying to report the position" << std::endl;
    ueRrc->SendPositionReport(position);
    // Simulator::Schedule(Seconds(2.0), &LteUeRrc::TestSendCtrlMessage, ueRrc);

    ueLastPosition.at(ueNodeId).second = position;
  }
}

void
UePositionReport(std::string context, UeCompletePositionReport report)
{
  NS_LOG_UNCOND(context << "\n imsi: " << report.ueImsi
      << "\n xError: " << report.xError
      << "\n yError: " << report.yError
      << "\n True X pos: " << report.trueXpos
      << "\n True Y pos: " << report.trueXpos
      << "\n Erroneous X pos: " << report.errXpos
      << "\n Erroneous Y pos: " << report.errYpos);
  // TODO: for each IMSI create a file. Write all values into that file as CSV
}

// BeamSweepTraceParams is the type of member TracedCallback <BeamSweepTraceParams> m_beamSweepTrace;
// that is delcared in nr-gnb-phy.h
void
BeamSweepTrace(std::string context, BeamSweepTraceParams params)
{
  NS_LOG_UNCOND(context << "got the BeamSweepTraceParams");

  std::cout << " - imsi: " << static_cast<int>(params.imsi) << std::endl;
  std::cout << " - currentCell: " << static_cast<int>(params.currentCell) << std::endl;
  std::cout << " - foundCell: " <<  static_cast<int>(params.foundCell) << std::endl;
  std::cout << " - snrBeforeSweep: " << params.snrBeforeSweep << std::endl;
  std::cout << " - snrDiffBeforeHO: " << params.snrDiffBeforeHO << std::endl;
}


void SetParameters();

/*
*****************************************    SIMULATION PARAMETER CONFIGURATION    *****************************************
*/

// 5G-NR numerology, bandwidth and txPower
uint32_t num = 3; // 5G-NR numerology mu
double centerFreq = 28e9; // Carrier frequency
                          // Available ray-tracing channel data includes 28 GHz and 60 GHz traces
double bandwidth = 400e6; // System Bandwidth 
double txPower = 15.0;    // txPower = 30 dBm for ray-tracing channel input
                          // (total txPower setting is 15 dBm + 15 dBm (from the ray-tracing simulations)
uint32_t packetSize = 1400; // Packet size in bytes
DataRate dataRate = DataRate("400Mb/s"); // Requested data rate per user in Mbps

/*
 *   The simulation parameters are set according to our paper 'flexRLM: Flexible Radio Link
 *   Monitoring for Multi-User Downlink Millimeter-Wave Networks'. 5G-NR numerology 3 
 *   (120kHz subcarriervspacing) at a central frequency of 28 GHz, with a total system bandwidth 
 *   of 400 MHz and a transmit power of 30 dBm is used. The maximum number of SSBs that can be
 *   transmitted is set to 64. For RLM, we set the number of monitored and reported CSI-RSs to 4.
 *   We consider CBR downlink UE traffic at 400 Mbps, and TDMA round-robin as our scheuduler.
 *   The total duration of the multi-user network simulation is 240 s.
*/
    
double simTime = 240; // in seconds
uint8_t numOfUes = 1;
uint64_t numOfGnbs = 15; // Amount of 5G NR gNBs in the simulation
/*
 *                           Table I. Beam management strategies and configuration parameters
 *                              _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
 *                             |                  Beam management strategy                     |
 *     Associated variables    |   Ideal   |    Default 5G-NR      |         flexRLM           |
 *     ----------------------------------------------------------------------------------------|
 *     realisticIA             |   false   |         true          |          true             | <- SSB
 *     rlmOn                   |   false   |         true          |          true             | <- CSI-RS
 *     ssbRlmOn                |   false   |      true / false     |          true             | <- Feedback SSB reports to coordinator for thresholds?
 *     completeSSBDuration (ms)|   0 / 20  |           -           |           -               |
 *     minCSIRSFromServiceGnb  |   -       |           4           |          0-3              |
 *     loadBalancing           |   -       |           -           |      true / false         |
 *     TODO: disableSweepDuringBeamTracking - after IA, no sweeps. Beam Pairs determined from REM
 *
*/
bool realisticIA = true; // if false, csiRS-related error occurs ans imsi is not found
bool rlmOn = true;
bool ssbRlmOn = false;
Time completeSSBDuration = MilliSeconds (0.0);
uint8_t minCSIRSFromServingGnb = 4; // was 2
bool loadBalancing = false;  // optional for flexRLM, not available in Ideal or Default 5G-NR. was true
bool adaptiveBF = true;     // set this to FALSE for using the Ideal beam management!!! (leave to true for the others)
bool BFdelay = false;       // leave this to FALSE
bool omniFallback = false;  // leave this to FALSE

// Location-aided Beamforming
// If this is set to true, UE and gNB beams and handovers will be managed depending on REM.
// CSI-RS will not be queued by gNBs and RLM logic will be bypassed(?)
// Handovers are still threshold-based, but are now controlled by REM.
bool useLABF = true;
bool lteRrcDelay = false;
bool uePositionError = false;
bool measurementRem = false; // No effect if "loadMeasurementRemFromFile" is true.
bool loadMeasurementRemFromFile = false;
double uePositionReportPeriod = 1;
bool useLabfImmediateHandovers = true;
uint8_t handoverHysterisisMaxCounter = 1;
bool enableLabfRecoveryAlgorithm = false;

// CSI-RS configuration (default settings for numOfUes = 20; please refer to the details above for other cases)
uint16_t csiRSPeriodicity = 23;
uint16_t csiRSOffset = 3;
uint16_t maxCSIRSResourcesPerFrame = 2;
uint8_t noOfBeamsTbRLM = maxCSIRSResourcesPerFrame*2;
// uint16_t ssbRLMTXDirections = 10; // same as number of gNBs
uint16_t ssbRLMTXDirections = numOfGnbs; // same as number of gNBs

// antenna configuraton
std::string antennaConfig = "AntennaConfigInets";
        //AntennaConfigDefault: gNB and UE Elevation range is from 60 to 120 degrees.
        //Angular step in Elevation can be adjusted from variables gNBVerticalBeamStep and ueVerticalBeamStep 
        //gNB and UE Azimuth range is from 0 to 180 degrees.
        //Angular step in Azimuth cannot be adjusted. They depend on the number of rows in antenna arrays of gNB and UE
        //AntennaConfigInets: gNB Elevation: 90, 120, 150; UE Elevation 30, 60, 90
uint8_t ueNumRows = 4;
uint8_t ueNumColumns = 4;
uint8_t gNBNumRows = 8;
uint8_t gNBNumColumns = 8;
double gNBVerticalBeamStep; 
double ueVerticalBeamStep;
double gNBHorizontalBeamStep;
double ueHorizontalBeamStep;

// flexRLM implements a threshold based operations using two signal-to-noise ratio (SNR):
double RLFThreshold = -5.0; // RLF (Outage) threshold in dB
double MRThreshold = 22.7; // Maximum rate threshold in dB

// UE walk parameters
std::vector<uint16_t> walkId;
std::vector<double> ueX;
std::vector<double> ueY;

// Per-packet latency logging
bool ipLatencyLogging  = true;
bool udpLatencyLogging = true;

// other parameters
AsciiTraceHelper asciiTraceHelper;
std::string path = "./out/test/"; // default path for saving simulation output results
bool harqEnabled = true;          // enable/disable hybrid automatic repeat request (HARQ)

void LoadSingleUeWalk(std::string walkNumber)
{
  // UE walk parameters, load UE information from folder:
  std::string input_raytracing_folder = "src/nr/model/Raytracing_UE_set/";

  std::ifstream infile("src/nr/model/Raytracing_UE_set/" + walkNumber + "_cords.txt");
  std::string sLine;
  getline(infile, sLine);

  size_t pos = sLine.find(",");
  ueX.push_back(std::stof(sLine.substr(0, pos)));
  sLine.erase(0, pos + 1);

  pos = sLine.find(",");
  ueY.push_back(std::stof(sLine.substr(0, pos)));

  walkId.push_back(stoi(walkNumber));

  std::cout << "Loaded X,Y coords for Walk ID " << walkNumber << std::endl;
}

/*
*****************************************    START OF MAIN FUNCTION   *****************************************
*/

int
main (int argc, char *argv[])
{

  ns3::RngSeedManager::SetSeed(time(NULL)); // Use system time for randomness
  ns3::RngSeedManager::SetRun(runNumber); // e.g., 1, 2, 3, ...

  if (useLABF)
  {
    // FIXME: re-check whether this makes any sense. I do not think this is needed.
    //ssbRlmOn = true; // We still seem to get measurement reports that are needed for handover algo. That is good.
    ssbRlmOn = false; // We still seem to get measurement reports that are needed for handover algo. That is good.

    // We might actually still want to use that for updating the info about channel
    // while we are waiting for position reports. Eg for adjustment within same gNB. 
    // Need to validate how much time is required to assemble the whole SSB report. If it is longer than
    // 1 second, then there is no point in using that.

    // Handovers probably need to be left for REM
  }

  // antenna configuraton
  if (antennaConfig == "AntennaConfigDefault")
  {
    gNBVerticalBeamStep = 10.0;
    ueVerticalBeamStep = 20.0;
  }
  else if (antennaConfig == "AntennaConfigInets")
  {
    gNBVerticalBeamStep = 30.0;
    ueVerticalBeamStep = 30.0;
    gNBHorizontalBeamStep = 9.0;
    ueHorizontalBeamStep = 18.0;
  }
  else
  {
    NS_ABORT_MSG ("Undefined Antenna Configuration");
  }
  
  // ==================================
  // Block for reading all UE walk IDs
  // ==================================
  // // UE walk parameters, load UE information from folder:
  std::string input_raytracing_folder = "src/nr/model/Raytracing_UE_set/";

  // struct dirent *entry;
  // DIR *dir = opendir("src/nr/model/Raytracing_UE_set/");

  // while((entry = readdir(dir)) != NULL)
  // {
  //   std::string filename = entry->d_name;
  //   if(filename.find("_cords.txt") != std::string::npos)
  //   {
  //     walkId.push_back(stoi(filename));

  //     std::ifstream infile("src/nr/model/Raytracing_UE_set/"+filename);
  //     std::string sLine;
  //     getline(infile, sLine);

  //     size_t pos = sLine.find(",");
  //     ueX.push_back(std::stof(sLine.substr(0, pos)));
  //     sLine.erase(0, pos+1);

  //     pos = sLine.find(",");
  //     ueY.push_back(std::stof(sLine.substr(0, pos)));
  //   }
  // }
  // closedir(dir);

  // ==========================
  // Read only specific walk ID
  // ==========================

  LoadSingleUeWalk("2000");
  //LoadSingleUeWalk("1234"); // Test coords

  // Half circle around gNB6 starting south, east, north. 5 seconds in the middle of each sector with 40m distance to gNB
  //LoadSingleUeWalk("1235"); // expecting all sectors from gNB on the path

  // Same half circle, but UE stands further away at 400,250. Somewhere in the open area between  gnb 9 and 10
  //LoadSingleUeWalk("1236");
  
  // Near gNB 7
  //LoadSingleUeWalk("1237"); // This one works fine gNB set to sector 20. UE to 6
  
  // UE 1 that should always be connected to gNB 6? No, see printout?
  // LoadSingleUeWalk("940");
  //LoadSingleUeWalk("9940"); // only second half of walk where connectivity was lost
  // UE 1 test for assert failed due to unknonw RNTI 2 in lte-rrc-protocol-ideal
  // LoadSingleUeWalk("941");
  // Testing the drop of SNR and crash of 1ue on 940
  // LoadSingleUeWalk("942");
  // Tsting yet anothr out of range crash at 449.18,267.2
  // LoadSingleUeWalk("943");
  // Testing the out of range crash at roughly 414.23 , 344.63
  // LoadSingleUeWalk("944");

  // UE 16 that walks around gNB6 starting from NB9, then gNB7, then ends at gNB3
  //LoadSingleUeWalk("2029");

  // Half circle radius 80
  // LoadSingleUeWalk("1240");
  // Test for multi-user sim wjere each stands on center of the sector
  // LoadSingleUeWalk("1250");
  // LoadSingleUeWalk("1251");
  // LoadSingleUeWalk("1252");
  // LoadSingleUeWalk("1261");
  // LoadSingleUeWalk("1262");
  // LoadSingleUeWalk("1263");
  // LoadSingleUeWalk("1264");

  // Tests with gNB 3 at 305,456. Going up left down.
  // Radius 15 meters. Should not end up in a building anywhere.
  // One point is inside the building and made problems
  // LoadSingleUeWalk("3330");

  // Tests with gNB 3 at 305,456. Going up left down.
  // Radius 10 meters. Should not end up in a building anywhere.
  //LoadSingleUeWalk("3331");

  // Around gNB6 provided by Aron
  // LoadSingleUeWalk("100"); // meant to be run with all gNBs
  // extend cords from aron to move more than 180 degs
  // starts behind the gNB south. gets sector 3
  // LoadSingleUeWalk("101"); // meant to be run with all gNBs

  // 101 but with first coords fixed at same y as aron has.
  // Attempt to avoid handover to gnb 7
  //LoadSingleUeWalk("102"); // connects to gnb9 in the beginning

  // IA on each sector
  // LoadSingleUeWalk("103");
  // LoadSingleUeWalk("104"); // single point in sec15 of gNB8

  // LoadSingleUeWalk("940");

  mkdir(path.c_str(),S_IRWXU);
  SetParameters();
  // Set the LTE CO cell ID for the RRC layer
  Config::SetDefault ("ns3::nrUeRrcProtocolIdeal::LteCoCellId", UintegerValue(numOfGnbs+1));
  // Amount of gNBs in the simulation
  Config::SetDefault ("ns3::ThreeGppSpectrumPropagationLossModel::AmountOfGnbs", UintegerValue(numOfGnbs));
  Config::SetDefault ("ns3::NrUePhy::AmountOfGnbs", UintegerValue(numOfGnbs));

  // Controls whether we apply a delay for transmistion of labf-related RRC messages.
  Config::SetDefault ("ns3::nrUeRrcProtocolIdeal::EnableLteRrcDelay", BooleanValue(lteRrcDelay));
  // In the following, set RaySourceType to "Inventory" if Inventory.txt is going to be used. Else, set it to "EnbTraceData"
  // "InventoryWiIS" configures the model to load files from the "src/nr/model/Raytracing_alternative/" folder
  // that is expected to contain FSPL, AoD, AoA, Delay Spread files.
  // Refer to ThreeGppSpectrumPropagationLossModel::LoadInventoryWirelessInSite for more details.
  // Leaving the attribute at "Inventory" may still be used for loading an already parsed WiIS REM file.
  Config::SetDefault ("ns3::ThreeGppSpectrumPropagationLossModel::RaySourceType", StringValue("Inventory"));
  // Set the TX power of gNBs. This will be used to derive the Path Loss value for WiIS REM construction.
  Config::SetDefault ("ns3::ThreeGppSpectrumPropagationLossModel::GnbTxPower", DoubleValue(txPower));
  // Defines whether we parse the REM from inventory file and apply an error, or load it directly from a separate file.
  Config::SetDefault ("ns3::LteEnbRrc::LoadMeasurementRemFromFile", BooleanValue(loadMeasurementRemFromFile));
  // Does not have any effect in the current state. There are no connections established by the LteHelper.
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (false));
  // Controls whether we apply an error model for UE position reports
  Config::SetDefault ("ns3::LteEnbRrc::ApplyErrorModelToUePositions", BooleanValue(uePositionError));
  // Controls whether we use "measurement" REM, which was affected by position errors during its construction
  Config::SetDefault ("ns3::LteEnbRrc::UseMeasurementREM", BooleanValue(measurementRem));
  // Decide whether to immediatelly HO to the best cell reported by REM
  Config::SetDefault ("ns3::LteEnbRrc::UseLabfImmediateHandovers", BooleanValue(useLabfImmediateHandovers));
  // Controls the amount of UE position reports that the UE may be served by a suboptimal gNB if UseLabfImmediateHandovers is enabled.
  Config::SetDefault ("ns3::LteEnbRrc::LabfHandoverThreshold", UintegerValue(handoverHysterisisMaxCounter));
  // Define whether we try to find closest non-emtpy REM point if the query for reported position is empty.
  Config::SetDefault ("ns3::LteEnbRrc::EnableRemRecoveryAlgorithm", BooleanValue(enableLabfRecoveryAlgorithm));

  // Offset for gNB locations due to different coordinate systems in the walk paths, gNB locations and WiIS REM.
  // This is used for recovery algorithm calculations
  Config::SetDefault ("ns3::LteEnbRrc::GnbLocationOffsetX", IntegerValue(128));
  Config::SetDefault ("ns3::LteEnbRrc::GnbLocationOffsetY", IntegerValue(121));

  // Define the frequency of UE position reports
  Config::SetDefault ("ns3::LteUeRrc::PositionReportPeriodSeconds", DoubleValue(uePositionReportPeriod));

  boost::optional<std::vector<uint16_t>> gnbCellIds
      = boost::none;
      //= boost::make_optional(std::vector<uint16_t>{4,5,7,9,10,13,14,15,16,17,18,19,20,24,26});

  //Good way to display what parameters are tried to be set
  NS_LOG_UNCOND ("Path: " << path);
  NS_LOG_UNCOND ("Simulation parameters:");
  NS_LOG_UNCOND ("sim Time: " << simTime);
  NS_LOG_UNCOND ("dataRate: " << dataRate);
  NS_LOG_UNCOND ("numerology: " << num);
  NS_LOG_UNCOND ("bandwidth: " << bandwidth);
  NS_LOG_UNCOND ("adaptiveBF: " << adaptiveBF);
  NS_LOG_UNCOND ("BFdelay: " << BFdelay);
  NS_LOG_UNCOND ("[not relevant] CSI-RS from serving gNB: " << minCSIRSFromServingGnb);
  NS_LOG_UNCOND ("[not relevant] Load balancing: " << loadBalancing);
  NS_LOG_UNCOND ("Use Location-aided Beamforming: " << useLABF);
  NS_LOG_UNCOND ("Amount of gNBs: " << numOfGnbs);
  NS_LOG_UNCOND ("EnableLteRrcDelay: " << lteRrcDelay);
  NS_LOG_UNCOND ("ApplyErrorModelToUePositions: " << uePositionError);
  NS_LOG_UNCOND ("UseMeasurementREM: " << measurementRem);
  NS_LOG_UNCOND ("PositionReportPeriodSeconds: " << uePositionReportPeriod);
  NS_LOG_UNCOND ("UseLabfImmediateHandovers: " << useLabfImmediateHandovers);
  NS_LOG_UNCOND ("LabfHandoverThreshold: " << static_cast<int>(handoverHysterisisMaxCounter));
  NS_LOG_UNCOND ("EnableLabfRecoveryAlgorithm: " << enableLabfRecoveryAlgorithm);

  // setup the Nr simulation
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper> ();

  nrHelper->SetEpcHelper (epcHelper);
  nrHelper->SetIdealBeamformingHelper (idealBeamformingHelper);
  lteHelper->SetEpcHelper (epcHelper);
  nrHelper->SetAttribute ("ChannelModel", StringValue ("ns3::ThreeGppChannelModel"));
  nrHelper->SetHarqEnabled (harqEnabled);
  nrHelper->SetSchedulerTypeId (NrMacSchedulerTdmaRR::GetTypeId ());

  //nrHelper->Initialize();
  nrHelper->LteChannelModelInitialization ();
  
  // position the base stations
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  LoadEnbLocations (enbPositionAlloc);
  
  NodeContainer gnbNodes;
  NodeContainer lteEnbNodes;
  NodeContainer allEnbNodes;

  gnbNodes.Create (enbPositionAlloc->GetSize());
  std::cout << "Num of gnb nodes: " << gnbNodes.GetN() << std::endl;
  enbPositionAlloc->Add (Vector (300,300,6));
  lteEnbNodes.Create (1);
  allEnbNodes.Add(gnbNodes);
  allEnbNodes.Add(lteEnbNodes);

  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator(enbPositionAlloc);
  enbmobility.Install (allEnbNodes);

  // UE nodes
  NodeContainer ueNodes;
  ueNodes.Create (numOfUes);

  MobilityHelper uemobility;
  uemobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  uemobility.Install (ueNodes);
  // For each UE, set its starting position for the mobility to the coordinates saved into vector ueX ueY
  // The node ID of the single UE is 14.
  for(uint32_t i=0; i<ueNodes.GetN(); i++)
  {
    ueNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (ueX[i], ueY[i], 1.5));
  }

  // labf: Tracing of UE position changes
  for (uint32_t ueId = 0; ueId < ueNodes.GetN(); ueId++)
  {
    std::ostringstream oss;
    oss << "/NodeList/" << ueNodes.Get(ueId)->GetId() << "/$ns3::MobilityModel/CourseChange";
    ueLastPosition.emplace(ueNodes.Get(ueId)->GetId(), std::make_pair<uint16_t, Vector>(ueId, {0,0,1.5}));
    std::cout << "UE ID " << ueId << " has Node ID " << ueNodes.Get(ueId)->GetId() << std::endl;
    Config::Connect(oss.str(), MakeCallback(&CourseChange));
  }

  /*
   * CC band configuration n257F (NR Release 15): one contiguous CC of 400MHz. 
   * In this example, the CC contains a single BWP occupying the whole CC bandwidth.
   *
   * -------------- Band --------------
   * -------------- CC0  --------------
   * -------------- BWP0 --------------
  */
  double centralFrequencyBand = centerFreq;
  const uint8_t numCcPerBand = 1;

  BandwidthPartInfo::Scenario scenario = BandwidthPartInfo::UMi_StreetCanyon;
  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;

  CcBwpCreator::SimpleOperationBandConf bandConf (centralFrequencyBand,
                                                  bandwidth,
                                                  numCcPerBand,
                                                  scenario);
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
  nrHelper->InitializeOperationBand(&band);
  allBwps = CcBwpCreator::GetAllBwps({band});

  // Set true to use cell scanning method, false to use the default power method.
  bool enableCellScan = true;
  if (enableCellScan)
  {
    idealBeamformingHelper->SetAttribute ("IdealBeamformingMethod", TypeIdValue (CellScanBeamforming::GetTypeId ()));
  }

  // Core latency
  epcHelper->SetAttribute ("S1uLinkDelay", TimeValue (MilliSeconds (0)));

  // Antennas for all the UEs
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (ueNumRows));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (ueNumColumns));

  // Antennas for all the gNbs
  nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (gNBNumRows));
  nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (gNBNumColumns));

	// install Nr net devices
  NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice (gnbNodes, allBwps);
  // NetDeviceContainer
  ueNetDev = nrHelper->InstallUeDevice (ueNodes, allBwps); // These are NR UEs??
  // InstallLteEnbDevice creates LTE PHY, MAC, RRC for the eNB. This should in theory create everything we need below RRC/RLC?
  NetDeviceContainer lteNetDev = nrHelper->InstallLteEnbDevice (lteEnbNodes);
  // FIXME: do we need an additional installation of ueNodes in lteHelper for UE<->LTECO link?
  // Might need to comment out some wtuff in LteHelper, as we would then have duplicate conmnections 
  // to EPC
  // auto test = lteHelper->InstallUeDevice(ueNodes); // This seems to work withou crashing for at least 1 UE 

  // create PGW
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // create the internet and install the IP stack on the UEs
  // get SGW/PGW and create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);
  
  // Create the internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (5)));

  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);

  // Setup routing
  Ptr<OutputStreamWrapper> stream =Create<OutputStreamWrapper> (&std::clog);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;

  Ptr<Ipv4StaticRouting> remoteHostStaticRouting;
  remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);
  remoteHostStaticRouting->PrintRoutingTable(stream);

  Ptr<Ipv4StaticRouting> pgwStaticRouting;
  pgwStaticRouting = ipv4RoutingHelper.GetStaticRouting (pgw->GetObject<Ipv4> ());
  pgwStaticRouting->PrintRoutingTable(stream);

  // Install IP stack on UEs
  internet.Install (ueNodes);
  
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (ueNetDev);

  // Assign IP address to UEs
  for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
  {
    Ptr<Node> ueNode = ueNodes.Get(u);
    
    // Set the default gateway for the UE
    Ptr<Ipv4StaticRouting> ueStaticRouting;
    ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    ueStaticRouting->PrintRoutingTable(stream);
  }

  // CBR 400 Mbps Applicationm_ptr
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;
  uint16_t udpCbrPort = 10000;
  
  Time packetInterval (Seconds (packetSize * 8 / static_cast<double> (dataRate.GetBitRate ())));

  for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
  {
    UdpServerHelper udpServer(udpCbrPort);
    serverApps.Add(udpServer.Install(ueNodes.Get(u)));

    UdpClientHelper udpClient(ueIpIface.GetAddress(u), udpCbrPort);
    udpClient.SetAttribute("PacketSize", UintegerValue(packetSize));
    udpClient.SetAttribute("Interval", TimeValue(packetInterval));
    udpClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    clientApps.Add(udpClient.Install(remoteHost));
  }

  // start server and client apps
  serverApps.Start(Seconds(0.));
  clientApps.Start(Seconds(0.));
  serverApps.Stop(Seconds(simTime));
  clientApps.Stop(Seconds(simTime - 0.2));

  for (auto it = enbNetDev.Begin (); it != enbNetDev.End (); ++it)
    {
      DynamicCast<NrGnbNetDevice> (*it)->UpdateConfig ();
    }

  for (auto it = ueNetDev.Begin (); it != ueNetDev.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }


  // Optionally change cell IDs of gNBs
  Ptr<NetDevice> lteEnbNetDevice = lteEnbNodes.Get (0)->GetDevice (0);
  // if (gnbCellIds)
  // {
  //   for(size_t i=0;i<gnbNodes.GetN();++i)
  //   {
  //     Ptr<NrGnbNetDevice> gnbDevice = DynamicCast<NrGnbNetDevice>(gnbNodes.Get(i)->GetDevice(0));
  //     if(gnbDevice)
  //     {
  //       NS_LOG_UNCOND("Changing Cell ID for gNB from " << i << " to " << gnbCellIds.get().at(i));
  //       gnbDevice->SetCellId(gnbCellIds.get().at(i));
  //     }
  //   }
  //   // Need to change the cell ID of the LTE CO as well. But it does not have a method for that?
  //   Ptr<Node> enbNode = lteEnbNodes.Get(0);  // Get the first eNB node
  //   Ptr<LteEnbNetDevice> enbDevice = DynamicCast<LteEnbNetDevice>(enbNode->GetDevice(0));

  //   if (enbDevice)
  //   {
  //       enbDevice->SetCellId(gnbCellIds.get().back() + 1);
  //       NS_LOG_UNCOND("Changed LTE CO Cell ID to " << enbDevice->GetCellId());
  //   }
  // }
  // interconnect all cells

  for(size_t i=0;i<gnbNodes.GetN()-1;++i)
  {
    for(size_t j=i+1;j<gnbNodes.GetN();++j)
    {
      epcHelper->AddX2Interface(gnbNodes.Get(i), gnbNodes.Get(j));
    }
  }
  
  uint16_t lteCellId = lteEnbNodes.Get(0)->GetDevice (0)->GetObject <LteEnbNetDevice> ()->GetCellId();

  // Connect all gNBs to the LTE coordinator
  for(size_t i=0; i<gnbNodes.GetN(); i++)
  {
    epcHelper->AddX2Interface(gnbNodes.Get(i), lteEnbNodes.Get(0));
    gnbNodes.Get(i)->GetDevice (0)->GetObject <NrGnbNetDevice> ()->GetRrc ()->SetClosestLteCellId (lteCellId);

    if (gnbNodes.Get(i)->GetDevice (0)->GetObject <NrGnbNetDevice> ()->GetPhy (0)->GetBeamManager () == nullptr)
    {
        NS_LOG_UNCOND ("Beam Manager is not initialized for gnb Node with Id " << gnbNodes.Get(i)->GetId());
    }
  }

  for (size_t i=0; i<ueNodes.GetN (); i++)
  {
    if (ueNodes.Get(i)->GetDevice (0)->GetObject <NrUeNetDevice> ()->GetPhy (0)->GetBeamManager () == nullptr)
    {
      NS_LOG_UNCOND ("Beam Manager is not initialized for UE node with Id " << ueNodes.Get(i)->GetId());
    }
  }

  // load ray-tracing data
  nrHelper->RayTraceModelLoadData (ueNetDev, &band, walkId, input_raytracing_folder);

  // attach UEs to the closest eNB
  // lteEnbNetDevice is the LTE-only eNB that acts as a coordinator. enbNetDev are gNBs
  nrHelper->AttachToClosestEnb (ueNetDev, enbNetDev);

  for(uint32_t i=0; i<ueNetDev.GetN(); i++)
  {
    std::cout << "UE " << i << " is getting attached to the LTE CO" << std::endl;
    // FIXME? This only creates an RRC-level link that is fine only for Ideal RRC protocol?
    // NrHelper::InstallSingleLteEnbDevice perfroms all PHY, MAC, RLC, RRC connections for eNB, but not for UEs.
    
    nrHelper->AttachToLteCoordinator (ueNetDev.Get(i)->GetObject <NetDevice> (), lteEnbNetDevice);
  }

  // lteEnbNetDevice is the coordinator and the only one who needs REM.
  Ptr<LteEnbRrc> coordinatorEnbRrc;
  if (useLABF)
  {
    Ptr<OutputStreamWrapper> outStream = asciiTraceHelper.CreateFileStream(path + "RemMeasurementError.txt");
    remMeasurementErrorOutputStream.emplace_back(outStream);
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/RemQueryErrorTrace", MakeCallback(&LogRemMeasurementError));
    remNonEmptyEntriesStream = asciiTraceHelper.CreateFileStream(path + "RemNonEmptyEntries.txt");
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/RemNonEmptyEntriesTrace", MakeCallback(&LogRemNonEmptyEntries));

    coordinatorEnbRrc = lteEnbNodes.Get(0)->GetDevice (0)->GetObject <LteEnbNetDevice>()->GetRrc();
    if (coordinatorEnbRrc != nullptr)
    {
      std::cout << "lteEnbNetDevice casted to LteEnbRrc" << std::endl;
      coordinatorEnbRrc->LoadRem("src/nr/model/Raytracing/");
      
      coordinatorEnbRrc->MakeEnbACoordinator(); // FIXME: this should not be required
      coordinatorEnbRrc->StoreGnbPositions(gnbCellIds); // This must be called for the labf recovery algorithm
      std::cout << "LTE coordinator cell ID " << coordinatorEnbRrc->GetCellId() << std::endl;
   }
  }

  Ptr<OutputStreamWrapper> outStream = asciiTraceHelper.CreateFileStream(path + "OrnsteinUhlenbeckParams.txt");
  ornsteinOutputTrace.emplace_back(outStream);

  // Add GNSS error model for each UE at the LTE CO.
  if (coordinatorEnbRrc)
  {
    for(uint32_t i=0; i<ueNetDev.GetN(); i++)
    {
      std::cout << "UE " << i << " is getting an Error model" << std::endl;
      Ptr<OrnsteinUhlenbeckErrorModel> latitudeErrorModel
          = CreateObject<OrnsteinUhlenbeckErrorModel>(6.2, 1.5, 3.8);
      Ptr<OrnsteinUhlenbeckErrorModel> longitudeErrorModel
          = CreateObject<OrnsteinUhlenbeckErrorModel>(3.9, 2.4, 4.2);
      // Assign new random stream for underlying random variable generator
      latitudeErrorModel->AssignStreams(runNumber);
      longitudeErrorModel->AssignStreams(runNumber);
      coordinatorEnbRrc->m_latitudeErrorModels.emplace(i+1, latitudeErrorModel);
      coordinatorEnbRrc->m_longitudeErrorModels.emplace(i+1, longitudeErrorModel);

      // For debugging purposes only. Does not work with multiple UEs.
      // latitudeErrorModel->TraceConnect("InternalStateTrace", "/NodeList/*/DeviceList/*/LteEnbRrc", MakeCallback(&LogOrnsteinUhlenbeckParams));
    }
  }

  if (realisticIA)
  {
    for (auto it = enbNetDev.Begin (); it != enbNetDev.End (); ++it)
    {
      for (uint32_t i = 0; i < DynamicCast<NrGnbNetDevice> (*it)->GetCcMapSize (); ++i)
        {
            for(uint32_t j=0; j<ueNetDev.GetN(); j++)
            {
              ueNetDev.Get(j)->GetObject <NrUeNetDevice> ()->GetPhy (i)->SetNumerology (uint16_t(num));
            }
        }
    }
  }

  // change size of global variables to match # of UEs
  std::vector<double> vector1 (ueNodes.GetN() , 0.0);
  std::vector<int> vector2 (ueNodes.GetN() , 0);
  totalUdpPacketsReceived = vector1;
  totalTimElapsedUdp = vector1;
  totalBytesReceived = vector1;
  totalTimeElapsed = vector1;
  numPacketsReceived = vector2;
  totalDelay = vector1;

  // enable the traces provided by the mmWave module
  nrHelper->EnableTraces();
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
  
  // Configure and schedule IP throughput and IP latency logging
  std::vector<Ptr<OutputStreamWrapper>> streamVector_ip;
  for (uint64_t imsi = 1; imsi <= ueNetDev.GetN(); imsi++)
  {
    Ptr<OutputStreamWrapper> ipStream = asciiTraceHelper.CreateFileStream (path + "DlIpStats" + std::to_string (imsi) + ".txt");
    streamVector_ip.emplace_back(ipStream);
  }
  Simulator::Schedule (Seconds (0.3), &GetThroughput, &flowmon, monitor, streamVector_ip);

  if (ipLatencyLogging)
  {
    // IP latency logging
    for (uint64_t imsi = 1; imsi <= ueNetDev.GetN(); imsi++)
    {
      Ptr<OutputStreamWrapper> ipLatencyStream = asciiTraceHelper.CreateFileStream(path + "DlIpLatency" + std::to_string(imsi) + ".txt");

      Ptr<Ipv4> ipv4 = ueNodes.Get(imsi - 1)->GetObject<Ipv4>();
      Ptr<Ipv4L3Protocol> ipv4L3Protocol = ipv4->GetObject<Ipv4L3Protocol>();
      ipv4L3Protocol->TraceConnectWithoutContext("Rx", MakeBoundCallback(&GetIpLatency, ipLatencyStream));
    }
  }

  if (udpLatencyLogging)
  {
    // UDP latency logging
    for (uint64_t imsi = 1; imsi <= ueNetDev.GetN(); imsi++)
    {
      Ptr<OutputStreamWrapper> udpLatencyStream = asciiTraceHelper.CreateFileStream(path + "DlUdpLatency" + std::to_string(imsi) + ".txt");

      Ptr<UdpServer> udpServer = DynamicCast<UdpServer>(serverApps.Get(imsi - 1));
      udpServer->TraceConnectWithoutContext("Rx", MakeBoundCallback(&GetUdpLatency, udpLatencyStream));
    }
  }

  // Configure and schedule UDP throughput logging
  std::vector<Ptr<OutputStreamWrapper>> streamVector;
  for (uint64_t imsi = 1; imsi <= ueNodes.GetN(); imsi++)
  {
    Ptr<OutputStreamWrapper> udpStream = asciiTraceHelper.CreateFileStream (path + "DlUdpStats" + std::to_string (imsi) + ".txt");
    streamVector.emplace_back (udpStream);
  }

  // 0.00025 is derived from observing cout from within LteEnbRrc::ForwardUePositionReportToREM
  Simulator::Schedule (Seconds(0.00025), &GetUdpThroughput, serverApps, streamVector);

  // connect custom trace sinks for RRC connection establishment and handover notification
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkUe));


  for (uint64_t imsi = 1; imsi <= ueNodes.GetN(); imsi++)
  {
    Ptr<OutputStreamWrapper> posReport = asciiTraceHelper.CreateFileStream (path + "UePositionReport" + std::to_string (imsi) + ".txt");
    streamVector_uePos.emplace_back (posReport);
    *streamVector_uePos.at(imsi-1)->GetStream() << "time,real_x,real_y,err_x,err_y,x_error,y_error" << std::endl;
  }
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/UePositionTrace", MakeCallback(&LogUePositionReport));

  for (uint64_t imsi = 1; imsi <= ueNodes.GetN(); imsi++)
  {
    Ptr<OutputStreamWrapper> beamOffsetTrace = asciiTraceHelper.CreateFileStream (path + "RemBeamOffset" + std::to_string (imsi) + ".txt");
    streamVector_beamOffsets.emplace_back (beamOffsetTrace);
  }
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/RemBeamSelectionTrace", MakeCallback(&LogRemBeamIdOffets));

  // Trace the artificial LTE RRC delay for each UE->LTE_CO connection
  for (uint64_t imsi = 1; imsi <= ueNodes.GetN(); imsi++)
  {
    Ptr<OutputStreamWrapper> lteRrcDelayTrace = asciiTraceHelper.CreateFileStream (path + "LteRrcDelay" + std::to_string (imsi) + ".txt");
    lteDelayStream.emplace_back (lteRrcDelayTrace);
  }

  // Trace the REM query status and recovery algorithm
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/RemQueryRecoveryTrace", MakeCallback(&LogRemQueryRecovery));
  for (uint64_t imsi = 1; imsi <= ueNodes.GetN(); imsi++)
  {
    Ptr<OutputStreamWrapper> remQueryRecoveryTrace = asciiTraceHelper.CreateFileStream (path + "RemQueryRecovery" + std::to_string (imsi) + ".txt");
    remQueryRecoveryStream.emplace_back (remQueryRecoveryTrace);
  }

  // FIXME: this does not work
  Ptr<Node> ueNode = ueNodes.Get(0); // Get the first UE node
  Ptr<NrUeNetDevice> ueDevice = DynamicCast<NrUeNetDevice>(ueNode->GetDevice(0));
  if (ueDevice)
  {
      Ptr<LteUeRrc> ueRrc = ueDevice->GetRrc(); // this works
      Ptr<nrUeRrcProtocolIdeal> rrcProtocol = DynamicCast<nrUeRrcProtocolIdeal>(ueDevice->GetRrc());

      if (rrcProtocol) // This fails
      {
          // Example: Enable tracing on "StateTransition" signal
          rrcProtocol->TraceConnectWithoutContext("LteRrcDelay", MakeCallback(&LogRrcDelay));
      }
      else
      {
        std::cerr << "Failed to get NrUeRrcProtocolIdeal instance!" << std::endl;
      }
  }

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}

void SetParameters()
{
	//This parameter controls the beamforming periodicity. If 0 then no periodic beamforming.
	int BFperiodicity;
	if(adaptiveBF)
	{
		BFperiodicity = 0;
	}
	else
	{
		BFperiodicity = 250; //if adaptive BF is turned off, trigger BF every 250 ms (on each position update)
	}
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::NrGnbPhy::UpdateSinrEstimatePeriod", DoubleValue (1600.0));
	Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingPeriodicity", TimeValue(MilliSeconds(BFperiodicity)));
	Config::SetDefault ("ns3::NrGnbPhy::IADelay", DoubleValue(5250));
	Config::SetDefault ("ns3::NrGnbPhy::BeamTrainingDelay", DoubleValue(5));
	Config::SetDefault ("ns3::NrGnbPhy::OmniNrFallback", BooleanValue(omniFallback));
  Config::SetDefault ("ns3::NrGnbMac::NumberOfBeamsTbRLM", UintegerValue (8));

  if (antennaConfig == "AntennaConfigDefault")
  {
    Config::SetDefault ("ns3::NrGnbPhy::GnbElevationAngleStep", DoubleValue (gNBVerticalBeamStep));
    Config::SetDefault ("ns3::NrGnbPhy::GnbHorizontalAngleStep", DoubleValue (180.0 / (double)gNBNumRows));

    Config::SetDefault ("ns3::NrUePhy::UeElevationAngleStep", DoubleValue (ueVerticalBeamStep));
    Config::SetDefault ("ns3::NrUePhy::UeHorizontalAngleStep", DoubleValue (180.0 / (double)ueNumRows));
  }
  else
  {
    Config::SetDefault("ns3::NrGnbPhy::GnbElevationAngleStep", DoubleValue(gNBVerticalBeamStep));
    Config::SetDefault("ns3::NrGnbPhy::GnbHorizontalAngleStep", DoubleValue(gNBHorizontalBeamStep));

    Config::SetDefault("ns3::NrUePhy::UeHorizontalAngleStep", DoubleValue (ueHorizontalBeamStep));
    Config::SetDefault("ns3::NrUePhy::UeElevationAngleStep", DoubleValue (ueVerticalBeamStep));
  }

  Config::SetDefault("ns3::NrUePhy::GnbSectionNumber", UintegerValue (63));
  Config::SetDefault("ns3::NrUePhy::CellSelectionCriterion", StringValue ("PeakSnr"));
  Config::SetDefault("ns3::NrUePhy::BeamSweepThreshold", DoubleValue (10.0));
  Config::SetDefault("ns3::NrUePhy::MaxRateThreshold", DoubleValue (MRThreshold));
  Config::SetDefault("ns3::NrUePhy::TXSSBScanDirections", UintegerValue (ssbRLMTXDirections));
  Config::SetDefault("ns3::NrPhy::MinCSIRSFromServingGnb", UintegerValue(minCSIRSFromServingGnb));
  Config::SetDefault("ns3::NrHelper::MinCSIRSFromServingGnb", UintegerValue(minCSIRSFromServingGnb));
  Config::SetDefault("ns3::NrGnbMac::MinCSIRSFromServingGnb", UintegerValue(minCSIRSFromServingGnb));
  Config::SetDefault("ns3::NrHelper::LoadBalancing", BooleanValue (loadBalancing));
  Config::SetDefault("ns3::NrPhy::AntennaConfiguration", StringValue(antennaConfig));

  Config::SetDefault("ns3::NrHelper::AdaptiveBeamforming", BooleanValue (adaptiveBF));
  Config::SetDefault("ns3::NrHelper::RealisticIA", BooleanValue (realisticIA));
  Config::SetDefault("ns3::ThreeGppChannelModel::RealisticBeamSweep", BooleanValue (realisticIA));
  Config::SetDefault("ns3::NrHelper::RadioLinkMonitoring", BooleanValue (rlmOn));
  Config::SetDefault("ns3::NrHelper::NumberOfRLMDirections", UintegerValue (noOfBeamsTbRLM));
  Config::SetDefault("ns3::NrGnbMac::NumberOfBeamsTbRLM", UintegerValue (noOfBeamsTbRLM));
  Config::SetDefault("ns3::NrPhy::SSBRLMOn", BooleanValue (ssbRlmOn));
  Config::SetDefault ("ns3::NrUePhy::CompleteTxSweepDuration", TimeValue(completeSSBDuration));
  Config::SetDefault ("ns3::NrUePhy::NoOfCSIRSResourcesPerSSBurst", UintegerValue(maxCSIRSResourcesPerFrame * 2));
  Config::SetDefault ("ns3::LteEnbRrc::CSIRSPeriodicity", UintegerValue (csiRSPeriodicity));
  Config::SetDefault ("ns3::LteEnbRrc::CSIRSOffset", UintegerValue (csiRSOffset));
  Config::SetDefault ("ns3::LteEnbRrc::MaxNumOfCSIRSResource", UintegerValue (maxCSIRSResourcesPerFrame));


	Config::SetDefault("ns3::NrGnbPhy::ApplyBeamformingDelay", BooleanValue(BFdelay)); // This is not used?
  Config::SetDefault ("ns3::NrGnbPhy::TxPower", DoubleValue (txPower));
  Config::SetDefault ("ns3::NrUePhy::TxPower", DoubleValue (txPower));
	Config::SetDefault ("ns3::NrGnbPhy::Numerology", UintegerValue(num));

	Config::SetDefault ("ns3::ThreeGppSpectrumPropagationLossModel::ChannelTypeRaytracing", BooleanValue(true));
  Config::SetDefault ("ns3::ThreeGppPropagationLossModel::ChannelTypeRaytracing", BooleanValue (true));

  Config::SetDefault ("ns3::ThreeGppChannelModel::Scenario", StringValue("UMi-StreetCanyon"));
  // important to set frequency into the 3gpp model
  Config::SetDefault ("ns3::ThreeGppChannelModel::Frequency", DoubleValue(28e9));
  Config::SetDefault ("ns3::ThreeGppSpectrumPropagationLossModel::Frequency", StringValue ("28GHz"));

  Config::SetDefault ("ns3::NrGnbPhy::UpdateUeSinrEstimatePeriod", DoubleValue (0));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (200*1024));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue(MicroSeconds(100.0)));
  Config::SetDefault ("ns3::LteEnbRrc::SystemInformationPeriodicity",TimeValue (MilliSeconds (1.0)));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));
  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue (RLFThreshold));
  Config::SetDefault("ns3::LteEnbRrc::MaxRateThreshold", DoubleValue (MRThreshold));
  Config::SetDefault("ns3::LteUeRrc::MaxRateThreshold", DoubleValue (MRThreshold));
  
	//Realisitc X2 and S1-U links
  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkDelay",
                      TimeValue (MicroSeconds (5)));
  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkDataRate",
                      DataRateValue (DataRate ("1000Gb/s")));
  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkMtu", UintegerValue (10000));
  Config::SetDefault ("ns3::PointToPointEpcHelper::S1uLinkDelay",
                      TimeValue (MicroSeconds (5)));

	Config::SetDefault ("ns3::NrPhyRxTrace::OutputFileName",
                      StringValue (path + "RxPacketTrace.txt"));
  Config::SetDefault ("ns3::NrPhyRxTrace::PhyRxThruFileName",
                      StringValue (path + "PhyRxThruData.txt"));
  Config::SetDefault ("ns3::NrPhyRxTrace::MacRxThruFileName",
                      StringValue (path + "MacRxThruData.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlPdcpOutputFilename",
                      StringValue (path + "DlPdcpStats.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::UlPdcpOutputFilename",
                      StringValue (path + "UlPdcpStats.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlPdcpThruFilename",
                      StringValue (path + "DlPdcpThruStats.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlRlcOutputFilename",
                      StringValue (path + "DlRlcStats.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlRlcThruFilename",
                      StringValue (path + "DlRlcThruStats.txt"));
  Config::SetDefault ("ns3::NrPhyRxTrace::BeamSweepFileName",
                      StringValue (path + "BeamSweepTrace.txt"));
  Config::SetDefault ("ns3::NrPhyRxTrace::RadioLinkMonitoringFileName",
                      StringValue (path + "RadioLinkMonitoringTrace.txt"));
  if (!realisticIA)
  {
    Config::SetDefault ("ns3::NrPhyRxTrace::BeamSweepType",
                      StringValue ("Ideal"));
  }
  else
  {
    Config::SetDefault ("ns3::NrPhyRxTrace::BeamSweepType",
                      StringValue ("Real"));
  }

  // Location-aided Beamforming
  if (useLABF)
  {
    Config::SetDefault("ns3::NrGnbPhy::UseLocationAidedBeamforming", BooleanValue(useLABF));
    Config::SetDefault("ns3::NrUePhy::UseLocationAidedBeamforming", BooleanValue(useLABF));
    Config::SetDefault("ns3::LteEnbRrc::UseLocationAidedBeamforming", BooleanValue(useLABF));
  }
}
