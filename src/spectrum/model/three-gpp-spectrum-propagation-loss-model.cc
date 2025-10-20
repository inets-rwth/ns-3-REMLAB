/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering,
 * New York University
 * Copyright (c) 2019 SIGNET Lab, Department of Information Engineering,
 * University of Padova
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
 */

#include "ns3/log.h"
#include "three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/net-device.h"
#include "ns3/three-gpp-antenna-array-model.h"
#include "ns3/node.h"
#include "ns3/channel-condition-model.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/simulator.h"
#include "ns3/pointer.h"
#include <map>
#include "ns3/node.h"
#include <ns3/nr-gnb-phy.h>

#include <ns3/three-gpp-channel-model.h>
#include <random>

#include <iomanip>

namespace ns3 {

std::random_device rdSM;
std::mt19937 mtSM (rdSM ());

std::uniform_real_distribution<double> m_uniformDistSM (0, 1);
std::uniform_real_distribution<double> m_uniformAziSM (1, 360);
std::uniform_real_distribution<double> m_uniformEleSM (1, 20);
std::uniform_real_distribution<double> m_uniformPowerSM (-80, -120);

NS_LOG_COMPONENT_DEFINE ("ThreeGppSpectrumPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED (ThreeGppSpectrumPropagationLossModel);

ThreeGppSpectrumPropagationLossModel::ThreeGppSpectrumPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

ThreeGppSpectrumPropagationLossModel::~ThreeGppSpectrumPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

void
ThreeGppSpectrumPropagationLossModel::DoDispose ()
{
  m_deviceAntennaMap.clear ();
  m_longTermMap.clear ();
  m_channelModel->Dispose ();
  m_channelModel = nullptr;
}

TypeId
ThreeGppSpectrumPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppSpectrumPropagationLossModel")
    .SetParent<SpectrumPropagationLossModel> ()
    .SetGroupName ("Spectrum")
    .AddConstructor<ThreeGppSpectrumPropagationLossModel> ()
    .AddAttribute("ChannelModel", 
                  "The channel model. It needs to implement the MatrixBasedChannelModel interface",
                  StringValue("ns3::ThreeGppChannelModel"),
                  MakePointerAccessor (&ThreeGppSpectrumPropagationLossModel::SetChannelModel,
                                       &ThreeGppSpectrumPropagationLossModel::GetChannelModel),
      MakePointerChecker<MatrixBasedChannelModel> ())
    .AddAttribute ("ChannelTypeRaytracing",
                   "If true, then raytracing is used; else 3gpp",
                   BooleanValue (true),
                   MakeBooleanAccessor (&ThreeGppSpectrumPropagationLossModel::channelTypeRaytracingSM),
                   MakeBooleanChecker ())
    .AddAttribute ("RaySourceType",
                   "Source of RayData that will be used for the simulation. According to its value either LoadInventory or LoadEnbTraceData will be run. ",
                   StringValue ("Inventory"),
                   MakeStringAccessor (&ThreeGppSpectrumPropagationLossModel::SMSetRaySourceType),
                   MakeStringChecker ())
    .AddAttribute ("Frequency",
                   "Central operation frequency",
                   StringValue ("60GHz"),
                   MakeStringAccessor (&ThreeGppSpectrumPropagationLossModel::m_frequency),
                   MakeStringChecker ())
    .AddAttribute ("AmountOfGnbs",
                   "Amount of gNBs used in the simulation",
                   UintegerValue(0),
                   MakeUintegerAccessor (&ThreeGppSpectrumPropagationLossModel::m_amountOfGnbs),
                   MakeUintegerChecker<uint64_t> (0, std::numeric_limits<uint64_t>::max()))
    .AddAttribute ("GnbTxPower",
                   "Transmission power of gNB in dBm",
                   DoubleValue (15.0),
                   MakeDoubleAccessor (&ThreeGppSpectrumPropagationLossModel::m_txPower),
                   MakeDoubleChecker<double> ())
    // Offsets for the WiIS REM reading
    .AddAttribute ("XposRemOffset",
                   "X position offset for reading a REM entry",
                   UintegerValue(375),
                   MakeUintegerAccessor (&ThreeGppSpectrumPropagationLossModel::m_xPosRemOffset),
                  MakeUintegerChecker<uint64_t> ())
    .AddAttribute ("YposRemOffset",
                   "Y position offset for reading a REM entry",
                   UintegerValue(376),
                   MakeUintegerAccessor (&ThreeGppSpectrumPropagationLossModel::m_yPosRemOffset),
                  MakeUintegerChecker<uint64_t> ())
    .AddAttribute ("ZposRemOffset",
                   "Z position offset for reading a REM entry",
                   UintegerValue(0),
                   MakeUintegerAccessor (&ThreeGppSpectrumPropagationLossModel::m_zPosRemOffset),
                  MakeUintegerChecker<uint64_t> ())
    ;
  return tid;
}

void
ThreeGppSpectrumPropagationLossModel::SetChannelModel (Ptr<MatrixBasedChannelModel> channel)
{
  m_channelModel = channel;
}

Ptr<MatrixBasedChannelModel>
ThreeGppSpectrumPropagationLossModel::GetChannelModel () const
{
  return m_channelModel;
}

void
ThreeGppSpectrumPropagationLossModel::AddDevice (Ptr<NetDevice> n, Ptr<const ThreeGppAntennaArrayModel> a)
{
  NS_ASSERT_MSG (m_deviceAntennaMap.find (n->GetNode ()->GetId ()) == m_deviceAntennaMap.end (), "Device is already present in the map");
  m_deviceAntennaMap.insert (std::make_pair (n->GetNode ()->GetId (), a));
}

double
ThreeGppSpectrumPropagationLossModel::GetFrequency () const
{
  DoubleValue freq;
  m_channelModel->GetAttribute ("Frequency", freq);
  return freq.Get ();
}

void
ThreeGppSpectrumPropagationLossModel::SetChannelModelAttribute (const std::string &name, const AttributeValue &value)
{
  m_channelModel->SetAttribute (name, value);
}

void
ThreeGppSpectrumPropagationLossModel::GetChannelModelAttribute (const std::string &name, AttributeValue &value) const
{
  m_channelModel->GetAttribute (name, value);
}

ThreeGppAntennaArrayModel::ComplexVector
ThreeGppSpectrumPropagationLossModel::CalcLongTerm (Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
                                                    const ThreeGppAntennaArrayModel::ComplexVector &sW,
                                                    const ThreeGppAntennaArrayModel::ComplexVector &uW) const
{
  NS_LOG_FUNCTION (this);

  uint16_t sAntenna = static_cast<uint16_t> (sW.size ());
  uint16_t uAntenna = static_cast<uint16_t> (uW.size ());

  NS_LOG_DEBUG ("CalcLongTerm with sAntenna " << sAntenna << " uAntenna " << uAntenna);
  //store the long term part to reduce computation load
  //only the small scale fading needs to be updated if the large scale parameters and antenna weights remain unchanged.
  ThreeGppAntennaArrayModel::ComplexVector longTerm;
  //uint8_t numCluster = static_cast<uint8_t> (params->m_channel[0][0].size ());
  uint8_t numCluster;
  if (channelTypeRaytracingSM)
  {
    numCluster = params->m_delay.size ();
  }
  else
  {
    numCluster = static_cast<uint8_t> (params->m_channel[0][0].size ());
  }
  

  for (uint8_t cIndex = 0; cIndex < numCluster; cIndex++)
    {
      std::complex<double> txSum (0,0);
      for (uint16_t sIndex = 0; sIndex < sAntenna; sIndex++)
        {
          std::complex<double> rxSum (0,0);
          for (uint16_t uIndex = 0; uIndex < uAntenna; uIndex++)
            {
              if (channelTypeRaytracingSM)
              {
                rxSum = rxSum + std::conj(uW[uIndex]) * params->m_channel[uIndex][sIndex][cIndex];
              }
              else
              {
                rxSum = rxSum + uW[uIndex] * params->m_channel[uIndex][sIndex][cIndex];
              }
              
            }
          txSum = txSum + sW[sIndex] * rxSum;
        }
      longTerm.push_back (txSum);
    }
  return longTerm;
}

Ptr<SpectrumValue>
ThreeGppSpectrumPropagationLossModel::CalcBeamformingGain (Ptr<SpectrumValue> txPsd,
                                                           ThreeGppAntennaArrayModel::ComplexVector longTerm,
                                                           ThreeGppAntennaArrayModel::ComplexVector txW,
                                                           ThreeGppAntennaArrayModel::ComplexVector rxW,
                                                           Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
                                                           double speed) const
{
  NS_LOG_FUNCTION (this);

  NS_LOG_FUNCTION (this);

  /*NS_LOG_INFO ("CalBeamGain: txW " << txW << std::endl << " rxW " << rxW << std::endl <<
                " delay " << params->m_delay << std::endl << " angle " << mara << std::endl <<
                " speed " << speed);*/


  NS_ABORT_MSG_IF (params->m_channel.size()==0, "Channel matrix is empty.");
  //NS_ABORT_MSG_IF (longTerm.size()==0, "Long-term matrix is empty.");
  NS_ABORT_MSG_IF (txW.size()==0, "Tx beamforming vector is emtpy.");
  NS_ABORT_MSG_IF (rxW.size()==0, "Rx beamforming vector is emtpy.");

  double spectrumValueCenterFrequency = txPsd->GetSpectrumModel()->Begin()->fl +
      ((txPsd->GetSpectrumModel()->End()-1)->fh - txPsd->GetSpectrumModel()->Begin()->fl)/2;

  NS_ABORT_MSG_IF (spectrumValueCenterFrequency!= GetFrequency(),
                   "Can't calculate beamforming gain for a spectrum value with that does not have the same central carrier frequency as the channel map");

  Ptr<SpectrumValue> tempPsd = Copy<SpectrumValue> (txPsd);

  NS_ABORT_MSG_UNLESS (params->m_delay.size()==params->m_channel.at(0).at(0).size(), "the cluster number of channel and delay spread should be the same");
  NS_ABORT_MSG_UNLESS (txW.size()==params->m_channel.at(0).size() || txW.size()==params->m_channel.size(), "the tx antenna size of channel and antenna weights should be the same");
  NS_ABORT_MSG_UNLESS (rxW.size()==params->m_channel.size() || rxW.size()==params->m_channel.at(0).size(), "the rx antenna size of channel and antenna weights should be the same");
  NS_ABORT_MSG_UNLESS (params->m_angle.at(0).size()==params->m_channel.at(0).at(0).size(), "the cluster number of channel and AOA should be the same");
  NS_ABORT_MSG_UNLESS (params->m_angle.at(1).size()==params->m_channel.at(0).at(0).size(), "the cluster number of channel and ZOA should be the same");

  //channel[rx][tx][cluster]
  size_t numCluster = params->m_delay.size ();
  //the update of Doppler is simplified by only taking the center angle of each cluster in to consideration.


  Values::iterator vit = tempPsd->ValuesBegin ();
  Bands::const_iterator sbit = tempPsd->ConstBandsBegin(); // sub band iterator

  uint16_t iSubband = 0;
  Time time = Simulator::Now();
  double varTtiTime = time.GetSeconds ();
  std::complex<double> doppler;
  while (vit != tempPsd->ValuesEnd ())
    {
      std::complex<double> subsbandGain (0.0,0.0);
      if ((*vit) != 0.00)
        {
          double fsb = (*sbit).fc; // take the carrier frequency of the band for which we al calculating the gain
          for (size_t cIndex = 0; cIndex < numCluster; cIndex++) // calculate for this subband for all the clusters
            {
              double temp_delay = -2 * M_PI * fsb * (params->m_delay.at (cIndex))*(1e-9);//need to convert ns to s
              std::complex<double> delay(cos(temp_delay),sin(temp_delay));

              double f_d = speed * GetFrequency () / 3e8;
              double temp_doppler = 2 * M_PI * varTtiTime * f_d * params->m_doppler.at(cIndex);
              doppler = std::complex<double> (cos (temp_doppler), sin (temp_doppler));
              std::complex<double> smallScaleFading = /*sqrt (pathPowerLinear) *  doppler */  delay;

              subsbandGain = subsbandGain + smallScaleFading * longTerm.at (cIndex);
            }
          *vit = (*vit) * (norm (subsbandGain));
        }
      vit++;
      sbit++;
      iSubband++;
    }
  return tempPsd;
}

Ptr<SpectrumValue>
ThreeGppSpectrumPropagationLossModel::CalcBeamformingGain (Ptr<SpectrumValue> txPsd,
                                                           ThreeGppAntennaArrayModel::ComplexVector longTerm,
                                                           Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
                                                           const ns3::Vector &sSpeed, const ns3::Vector &uSpeed) const
{
  NS_LOG_FUNCTION (this);

  Ptr<SpectrumValue> tempPsd = Copy<SpectrumValue> (txPsd);

  //channel[rx][tx][cluster]
  uint8_t numCluster = static_cast<uint8_t> (params->m_channel[0][0].size ());

  // compute the doppler term
  // NOTE the update of Doppler is simplified by only taking the center angle of
  // each cluster in to consideration.
  double slotTime = Simulator::Now ().GetSeconds ();
  ThreeGppAntennaArrayModel::ComplexVector doppler;
  for (uint8_t cIndex = 0; cIndex < numCluster; cIndex++)
    {
      //cluster angle angle[direction][n],where, direction = 0(aoa), 1(zoa).
      // TODO should I include the "alfa" term for the Doppler of delayed paths?
      double temp_doppler = 2 * M_PI * ((sin (params->m_angle[MatrixBasedChannelModel::ZOA_INDEX][cIndex] * M_PI / 180) * cos (params->m_angle[MatrixBasedChannelModel::AOA_INDEX][cIndex] * M_PI / 180) * uSpeed.x
                                         + sin (params->m_angle[MatrixBasedChannelModel::ZOA_INDEX][cIndex] * M_PI / 180) * sin (params->m_angle[MatrixBasedChannelModel::AOA_INDEX][cIndex] * M_PI / 180) * uSpeed.y
                                         + cos (params->m_angle[MatrixBasedChannelModel::ZOA_INDEX][cIndex] * M_PI / 180) * uSpeed.z)
                                         + (sin (params->m_angle[MatrixBasedChannelModel::ZOD_INDEX][cIndex] * M_PI / 180) * cos (params->m_angle[MatrixBasedChannelModel::AOD_INDEX][cIndex] * M_PI / 180) * sSpeed.x
                                         + sin (params->m_angle[MatrixBasedChannelModel::ZOD_INDEX][cIndex] * M_PI / 180) * sin (params->m_angle[MatrixBasedChannelModel::AOD_INDEX][cIndex] * M_PI / 180) * sSpeed.y
                                         + cos (params->m_angle[MatrixBasedChannelModel::ZOD_INDEX][cIndex] * M_PI / 180) * sSpeed.z))
        * slotTime * GetFrequency () / 3e8;
      doppler.push_back (exp (std::complex<double> (0, temp_doppler)));
    }

  // apply the doppler term and the propagation delay to the long term component
  // to obtain the beamforming gain
  auto vit = tempPsd->ValuesBegin (); // psd iterator
  auto sbit = tempPsd->ConstBandsBegin(); // band iterator
  while (vit != tempPsd->ValuesEnd ())
    {
      std::complex<double> subsbandGain (0.0,0.0);
      if ((*vit) != 0.00)
        {
          double fsb = (*sbit).fc; // center frequency of the sub-band
          for (uint8_t cIndex = 0; cIndex < numCluster; cIndex++)
            {
              double delay = -2 * M_PI * fsb * (params->m_delay[cIndex]);
              subsbandGain = subsbandGain + longTerm[cIndex] * doppler[cIndex] * exp (std::complex<double> (0, delay));
            }
          *vit = (*vit) * (norm (subsbandGain));
        }
      vit++;
      sbit++;
    }
  return tempPsd;
}

ThreeGppAntennaArrayModel::ComplexVector
ThreeGppSpectrumPropagationLossModel::GetLongTerm (uint32_t aId, uint32_t bId,
                                                   Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
                                                   const ThreeGppAntennaArrayModel::ComplexVector &aW,
                                                   const ThreeGppAntennaArrayModel::ComplexVector &bW) const
{
  ThreeGppAntennaArrayModel::ComplexVector longTerm; // vector containing the long term component for each cluster

  // check if the channel matrix was generated considering a as the s-node and
  // b as the u-node or viceversa
  ThreeGppAntennaArrayModel::ComplexVector sW, uW;
  if (!channelMatrix->IsReverse (aId, bId))
  {
    sW = aW;
    uW = bW;
  }
  else
  {
    sW = bW;
    uW = aW;
  }

  // compute the long term key, the key is unique for each tx-rx pair
  uint32_t x1 = std::min (aId, bId);
  uint32_t x2 = std::max (aId, bId);
  uint32_t longTermId = MatrixBasedChannelModel::GetKey (x1, x2);

  bool update = false; // indicates whether the long term has to be updated
  bool notFound = false; // indicates if the long term has not been computed yet

  // look for the long term in the map and check if it is valid
  if (m_longTermMap.find (longTermId) != m_longTermMap.end ())
  {
    NS_LOG_DEBUG ("found the long term component in the map");
    longTerm = m_longTermMap[longTermId]->m_longTerm;

    // check if the channel matrix has been updated
    // or the s beam has been changed
    // or the u beam has been changed
    update = (m_longTermMap[longTermId]->m_channel->m_generatedTime != channelMatrix->m_generatedTime
              || m_longTermMap[longTermId]->m_sW != sW
              || m_longTermMap[longTermId]->m_uW != uW);

  }
  else
  {
    NS_LOG_DEBUG ("long term component NOT found");
    notFound = true;
  }

  if (update || notFound)
    {
      NS_LOG_DEBUG ("compute the long term");
      // compute the long term component
      longTerm = CalcLongTerm (channelMatrix, sW, uW);

      // store the long term
      Ptr<LongTerm> longTermItem = Create<LongTerm> ();
      longTermItem->m_longTerm = longTerm;
      longTermItem->m_channel = channelMatrix;
      longTermItem->m_sW = sW;
      longTermItem->m_uW = uW;

      m_longTermMap[longTermId] = longTermItem;
    }

  return longTerm;
}

Ptr<SpectrumValue>
ThreeGppSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity (Ptr<const SpectrumValue> txPsd,
                                                                    Ptr<const MobilityModel> a,
                                                                    Ptr<const MobilityModel> b) const
{
  NS_LOG_FUNCTION (this);
  uint32_t aId = a->GetObject<Node> ()->GetId (); // id of the node a
  uint32_t bId = b->GetObject<Node> ()->GetId (); // id of the node b

  uint32_t traceIndex = 0;
  uint16_t imsi;

  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue> (txPsd);

  /*if ("ns3::ThreeGppChannelModel" == typeid(m_channelModel).name())
  {*/
    bool aIsEnb = false;
    bool bIsEnb = false;

    traceIndex = DynamicCast<ThreeGppChannelModel> (m_channelModel)->GetTraceIndex ();

    // With this method a eNB is recognized by matching the poisition from the loaded data
    NS_LOG_INFO ("size=" << enbLocations.size ());
    if (enbLocations.size () != 0)
    {
      for (unsigned int i = 0; i < enbLocations.size (); i++)
      {
        NS_LOG_INFO("enb i="<<i<<": x="<<enbLocations.at(i).x<<" y="<<enbLocations.at(i).y);
        if (a->GetPosition ().x == enbLocations.at (i).x && a->GetPosition ().y == enbLocations.at (i).y)
        {
          aIsEnb = true;
        }
        else if (b->GetPosition ().x == enbLocations.at(i).x && b->GetPosition ().y == enbLocations.at(i).y)
        {
          bIsEnb = true;
        }
      }
    }

    if ((aIsEnb && bIsEnb) || (!aIsEnb && !bIsEnb))
    {
      NS_LOG_INFO ("UE<->UE or gNB<->gNB, returning");
      if (channelTypeRaytracingSM)
      {
        *rxPsd = *rxPsd * 1e-11;
      }
      return rxPsd;
    }
    else if (aIsEnb && !bIsEnb)
    {
      imsi = b->GetObject<Node> ()->GetDevice(0)->GetObject<NrUeNetDevice> ()->GetImsi();
      b->GetObject<MobilityModel> () ->SetPosition(walkCords.at(imsi).at(traceIndex));
      //b->GetObject<MobilityModel> ()->SetPosition (walkCords.at(traceIndex));
    }
    else if (!aIsEnb && bIsEnb)
    {
      imsi = a->GetObject<Node> ()->GetDevice(0)->GetObject<NrUeNetDevice> ()->GetImsi();
      a->GetObject<MobilityModel> () ->SetPosition(walkCords.at(imsi).at(traceIndex));
      //a->GetObject<MobilityModel> ()->SetPosition (walkCords.at(traceIndex));
    }

    NS_ASSERT (aId != bId);
    NS_ASSERT_MSG (a->GetDistanceFrom (b) > 0.0, "The position of a and b devices cannot be the same");
  //}

  // retrieve the antenna of device a
  NS_ASSERT_MSG (m_deviceAntennaMap.find (aId) != m_deviceAntennaMap.end (), "Antenna not found for node " << aId);
  Ptr<const ThreeGppAntennaArrayModel> aAntenna = m_deviceAntennaMap.at (aId);
  NS_LOG_DEBUG ("a node " << a->GetObject<Node> () << " antenna " << aAntenna);

  // retrieve the antenna of the device b
  NS_ASSERT_MSG (m_deviceAntennaMap.find (bId) != m_deviceAntennaMap.end (), "Antenna not found for device " << bId);
  Ptr<const ThreeGppAntennaArrayModel> bAntenna = m_deviceAntennaMap.at (bId);
  NS_LOG_DEBUG ("b node " << bId << " antenna " << bAntenna);
  // if (aAntenna->GetBearingAngleDeg() != 0. || aAntenna->GetBearingAngleRad() != 0.)
  // { 
  //   std::cout << "  ThreeGppSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity:"<< std::endl;
  //   std::cout << "  - device A atenna bearing angle deg: " << aAntenna->GetBearingAngleDeg() << ". Is gNB? " << aIsEnb << std::endl;
  //   std::cout << "  - device A atenna bearing angle rad: " << aAntenna->GetBearingAngleRad() << ". Is gNB? " << aIsEnb << std::endl;
  // }

  if (aAntenna->IsOmniTx () || bAntenna->IsOmniTx () )
    {
      NS_LOG_LOGIC ("Omni transmission, do nothing.");
      if (channelTypeRaytracingSM)
      {
        *rxPsd = *rxPsd * 1e-22;
      }
      return rxPsd;
    }

  Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix;

  if (aIsEnb && !bIsEnb)
  {
    channelMatrix = m_channelModel->GetChannel (a, b, aAntenna, bAntenna);  
  }
  else if (!aIsEnb && bIsEnb) 
  {
    channelMatrix = m_channelModel->GetChannel (a, b, aAntenna, bAntenna);
  }
  else
  {
    NS_ABORT_MSG ("Both node a and b cannot be of the same type");
  }  
  
  if (channelMatrix->m_numCluster == 0)
  {
    NS_LOG_INFO ("No RayTrace data between " << a->GetPosition () << " and " << b->GetPosition ());
    //The ryatracing channel does not have any pathloss, therefore when there is no data we need to supress the ouput signal strength
    if (channelTypeRaytracingSM)
    {
      *rxPsd = *rxPsd * 1e-22;
    }
    return rxPsd;
  }

  // get the precoding and combining vectors

  ThreeGppAntennaArrayModel::ComplexVector aW = aAntenna->GetBeamformingVector ();
  ThreeGppAntennaArrayModel::ComplexVector bW = bAntenna->GetBeamformingVector ();

  // retrieve the long term component
  ThreeGppAntennaArrayModel::ComplexVector longTerm;
  longTerm = GetLongTerm (aId, bId, channelMatrix, aW, bW);
  Ptr<SpectrumValue> bfPsd;
  
  // apply the beamforming gain
  if (channelTypeRaytracingSM)
  {
    bfPsd = CalcBeamformingGain (rxPsd, longTerm, aW, bW, channelMatrix, walkSpeed.at(imsi).at (traceIndex));
    //bfPsd = CalcBeamformingGain (rxPsd, longTerm, aW, bW, channelMatrix, walkSpeed.at (traceIndex));
  }
  else
  {
    bfPsd = CalcBeamformingGain (rxPsd, longTerm, channelMatrix, a->GetVelocity (), b->GetVelocity ());
  }
  
  Ptr<SpectrumValue> bfGain = Create<SpectrumValue>((*bfPsd) / (*rxPsd));

  return bfPsd;
}

void 
ThreeGppSpectrumPropagationLossModel::LoadAllTraceData (NetDeviceContainer ueNetDev, std::vector<uint16_t> walkId, std::string input_raytracing_folder)
{
  if (SMGetRaySourceType () == "InventoryWiIS")
  {
    // Assumes 28GHz frequency being used 
    input_files_folder = "src/nr/model/Raytracing_alternative/";
  }
  else
  {
    input_files_folder = "src/nr/model/Raytracing/";
  }

  std::string inventory_file;
  if (m_frequency == "60GHz")
  {
    inventory_file = input_files_folder + "Inventory.txt";
  }
  else if (m_frequency == "28GHz")
  {
    inventory_file = input_files_folder + "Inventory_28GHz.txt";
  }
  else
  {
    NS_ABORT_MSG ("Undefined central frequency band");
  }
  

  LoadEnbLocations (input_files_folder);

  if (SMGetRaySourceType () == "Inventory")
  {
    NS_LOG_INFO ("Loading the inventory trace data for all ENB");
    LoadInventory (inventory_file);
  }
  else if (SMGetRaySourceType () == "InventoryWiIS")
  {
    LoadInventoryWirelessInSite(input_files_folder);
  }
  else if (SMGetRaySourceType () == "EnbTraceData")
  {
    NS_LOG_INFO ("Loading the trace data for all configured ENB");
    std::vector<Vector>::iterator it;
    for (it = enbLocations.begin (); it != enbLocations.end (); it++)
    {
      LoadEnbTraceData (*it);
    }
  }
  else
  {
    NS_ABORT_MSG ("Unidentified source for RayData");
  }

  NS_LOG_INFO ("Loading the walk and pedestrian blockage data");

  for (uint32_t ue_index = 0; ue_index < ueNetDev.GetN(); ue_index++)
  {
    std::cout << "ThreeGppSpectrumPropagationLossModel::LoadAllTraceData : walkId size " << walkId.size() << std::endl;
    std::cout << "   ue_index: " << static_cast<int>(ue_index) << std::endl;
    uint16_t ueWalkId = walkId[ue_index];
    uint16_t imsi = ueNetDev.Get(ue_index)->GetObject <NrUeNetDevice> ()->GetImsi ();

    std::string walk_file = input_raytracing_folder + std::to_string(ueWalkId) + "_cords.txt";
    std::string speed_file = input_raytracing_folder + std::to_string(ueWalkId) + "_speed.txt";
    std::string los_file = input_raytracing_folder + std::to_string(ueWalkId) + "_ped.txt";

    NS_LOG_INFO("Loading the walk and pedestrian blockage data");
    LoadWalkInfo(walk_file, speed_file, imsi);
    //LoadLosInfo(los_file, imsi);
  }

  if (SMGetRaySourceType () == "Inventory")
  {
    DynamicCast<ThreeGppChannelModel> (m_channelModel)->SetSharedParams (enbLocations,
                                      walkCords, los_data, allLinkData,maxTraceIndexSM, 
                                      channelTypeRaytracingSM, "Inventory");
  }
  else if (SMGetRaySourceType () == "EnbTraceData")
  {
    DynamicCast<ThreeGppChannelModel> (m_channelModel)->SetSharedParams (enbLocations,
                                      walkCords, los_data, all_enbTraceData, maxTraceIndexSM,
                                      channelTypeRaytracingSM, "EnbTraceData");
  }
  else
  {
    NS_ABORT_MSG ("Unidentified source for RayData");
  }
  // FIXME: TODO: add a case for InventoryWiIS?
  
  
}

void
ThreeGppSpectrumPropagationLossModel::LoadEnbLocations (std::string input_folder)
{
  std::string enbFile = input_folder + "enb_locations.txt";
  NS_LOG_UNCOND ("Loading ENB locations from: " << enbFile);
  std::ifstream file1;
  file1.open (enbFile.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file1.good (), "File containing ENB locations not found");

  std::string line;
  std::string token;
  while ( (std::getline (file1, line))) //Parse each line of the file
  {
    doubleVector_t lineElements;
    std::istringstream stream (line);

    while (getline (stream, token, ','))  //Parse each comma separated string in a line
    {
      double sigma = 0.00;
      std::stringstream stream (token);
      stream >> sigma;
      lineElements.push_back (sigma);
    }
    enbLocations.push_back (
            Vector (lineElements.at (0), lineElements.at (1), lineElements.at (2)));
    
  }
  
}

void
ThreeGppSpectrumPropagationLossModel::LoadEnbTraceData (Vector enbLocation)
{
  NS_LOG_UNCOND ("Loading raytracer data for the ENB:" << enbLocation);

  std::string traceFileName = GetTraceFileName (enbLocation);
  std::ifstream singlefile;
  singlefile.open (traceFileName.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (singlefile.good (), "Raytracing file not found");

  std::map<Vector, Ptr<RxRayData>> all_rx_rayData;
  Ptr<RxRayData> rxData = Create<RxRayData> ();
  Vector rxPos;

  uint16_t counter = 0;
  std::string line;
  std::string token;
  while (std::getline (singlefile, line)) //Parse each line of the file
    {
      if (counter == 9)
        {
          all_rx_rayData.insert (std::make_pair (rxPos, rxData));
          rxData = Create<RxRayData> ();
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
          rxData->m_path = lineElements.at (0);
          break;
        case 2:
          rxData->m_delay = lineElements;
          break;
        case 3:
          rxData->m_pathloss = lineElements;
          break;
        case 4:
          rxData->m_los = lineElements;
          break;
        case 5:
          rxData->m_aodElevation = lineElements;
          break;
        case 6:
          rxData->m_aodAzimuth = lineElements;
          break;
        case 7:
          rxData->m_aoaElevation = lineElements;
          break;
        case 8:
          rxData->m_aoaAzimuth = lineElements;
          break;
        default:
          break;
        }
      counter++;
    }

  Ptr<EnbTraceData> enb_trace = Create<EnbTraceData> ();
  enb_trace->m_enbPos = enbLocation;
  enb_trace->m_rxRayData = all_rx_rayData;
  all_enbTraceData.insert (std::make_pair (enbLocation, enb_trace));
}

void
ThreeGppSpectrumPropagationLossModel::LoadWalkInfo (std::string walkInfoFile, std::string speedFile, uint16_t imsi)
{
  NS_LOG_UNCOND ("Loading the walk coordinates from: " << walkInfoFile);
  std::ifstream file1;
  file1.open (walkInfoFile.c_str (), std::ifstream::in);
  //NS_ASSERT_MSG (file1.good (), "Walk coordinates file not found");

  std::vector<Vector> tmp_walkCords;
  std::string line;
  std::string token;
  while (std::getline (file1, line))  //Parse each line of the file
  {
    doubleVector_t lineElements;
    std::istringstream stream (line);

    while (getline (stream, token, ','))  //Parse each comma separated string in a line
    {
      double sigma = 0.00;
      std::stringstream stream (token);
      stream >> sigma;
      lineElements.push_back (sigma);
    }
    tmp_walkCords.push_back (Vector (lineElements.at (0), lineElements.at (1), lineElements.at (2)));
  }
  walkCords[imsi] = tmp_walkCords;
  
  std::ifstream file2;
  file2.open (speedFile.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file2.good (), "Walk speed file not found");

  std::vector<double> tmp_walkSpeed;
  while (std::getline (file2, line)) //Parse each line of the file
  {
    doubleVector_t lineElements;
    std::istringstream stream (line);

    while (getline (stream, token, ','))  //Parse each comma seperated string in a line
    {
      double sigma = 0.00;
      std::stringstream stream (token);
      stream >> sigma;
      lineElements.push_back (sigma);
    }
    
    tmp_walkSpeed.push_back (lineElements.at (0));
  }
  walkSpeed[imsi] = tmp_walkSpeed;

  maxTraceIndexSM = tmp_walkCords.size();

  NS_ASSERT_MSG (walkSpeed.size () == walkCords.size (),
                "The size of the walk coordinates file and the speed file must be the same.");
  
}

void
ThreeGppSpectrumPropagationLossModel::LoadLosInfo (std::string los_file, uint16_t imsi)
{
  NS_LOG_UNCOND ("Loading the LOS data from: " << los_file);
  std::ifstream file1;
  file1.open (los_file.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file1.good (), "LOSS file not found");
  
  std::vector<std::vector<int> >  tmp_los_data;
  std::string line;
  std::string token;
  while (std::getline (file1, line))  //Parse each line of the file
  {
    std::vector<int> lineElements;
    std::istringstream stream (line);

    while (getline (stream, token, ','))  //Parse each comma separated string in a line
    {
      int sigma = 0;
      std::stringstream stream (token);
      stream >> sigma;
      lineElements.push_back (sigma);
    }
    
    tmp_los_data.push_back (lineElements);
  }

  los_data[imsi] = tmp_los_data;
}

// New function to load the inventory trace data for all ENB
void
ThreeGppSpectrumPropagationLossModel::LoadInventory (std::string inventory_file)
{
  NS_LOG_UNCOND ("Loading the inventory from: " << inventory_file);
	std::ifstream singlefile;
	singlefile.open (inventory_file.c_str (), std::ifstream::in);
	NS_ASSERT_MSG (singlefile.good (), "Inventory file not found");

    //std::map<Vector, Ptr<LinkData>> allLinkData;
	Ptr<LinkData> linkData = Create<LinkData> ();
	Vector rxPos;

	uint16_t counter = 0;
	std::string line;
	std::string token;
  while (std::getline (singlefile, line)) //Parse each line of the file
  {
  	if (counter == 10)
  	{
  		allLinkData.insert (std::make_pair (rxPos, linkData));
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
        // std::cout << " delay: " <<  lineElements.front() << std::endl;
      	break;
      	default:
      	break;
      }
      counter++;
  }
}

std::string
ThreeGppSpectrumPropagationLossModel::GenerateFilename(const std::string &inventory_dir, const std::string &prefix, uint32_t gnb)
{
  std::string suffix = "_60.r059.p2m";
  std::ostringstream filename;
  filename << inventory_dir << "/" << prefix << "t" << std::setw(3) << std::setfill('0') << gnb << suffix;
  return {filename.str()};
}

void
ThreeGppSpectrumPropagationLossModel::ParseFspl(const std::string &filePath, uint8_t skipLines, std::function<void(std::istringstream &)> parser)
{
  std::ifstream file(filePath);
  NS_ASSERT_MSG(file.good(), "File not found: " << filePath);

  std::string line;
  for (uint8_t i = 0; i < skipLines; ++i)
    std::getline(file, line); // Skip headers

  while (std::getline(file, line))
  {
    // std::cout << line << std::endl;
    std::istringstream stream(line);
    parser(stream);
  }
}

void
ThreeGppSpectrumPropagationLossModel::ParseDelaySpread(const std::string &filePath, uint8_t skipLines, std::function<void(std::istringstream &)> parser)
{

  std::ifstream file(filePath);
  NS_ASSERT_MSG(file.good(), "File not found: " << filePath);

  std::string line;
  for (uint8_t i = 0; i < skipLines; ++i)
    std::getline(file, line); // Skip headers

  while (std::getline(file, line))
  {
    std::istringstream stream(line);
    parser(stream);
  }
}


void
ThreeGppSpectrumPropagationLossModel::ParseLos(const std::string &filePath, uint8_t skipLines)
{
  // !!!
  // At the moment of writing this code, the LOS data is provided as post-processed .m files which
  // store data in a different format than DoX, Pathloss, etc.
  // Y-axis is inverted, thus, traversal of data is different to other files.
  // If the LOS is provided in a different way now, adjust or provide a new parsing method.
  std::ifstream file(filePath);
  NS_ASSERT_MSG(file.good(), "File not found: " << filePath);

  std::string line;
  uint32_t lineNumber{1};
  uint32_t remIndex{1};
  for (uint8_t i = 0; i < skipLines; ++i)
    std::getline(file, line); // Skip headers

  while (std::getline(file, line))
  {
    std::istringstream stream(line);
    // For the given "x"=="lineNumber", read LOS data for all "y" values; 
    // File contains indices. Need to map them to positions
    Vector rxPos = m_WirelessInSiteIndexPositionMap[remIndex];
    std::vector<short int> losValues;
    std::stringstream ss(line);
    std::string value;

    // Split the line by commas and convert to integers
    while (std::getline(ss, value, ','))
    {
      losValues.push_back(std::stoi(value)); // Convert string to int and store in vector
    }
    // Now we read all columns/y values.
    // Now map each of them to a position and store in m_wiisLos.
    for (const short int& losEntry : losValues)
    {
      // x and y seem to be switched
      // m_WirelessInSiteIndexPositionMap is first iterating over x values, then over y.
      // Here we are reading y first. Need to 
      Vector rxPos = m_WirelessInSiteIndexPositionMap[remIndex];
      m_wiisLos.emplace(rxPos, losEntry);
      remIndex += losValues.size(); // add 500 to iterate over y first
      size_t gridSize = losValues.size() * losValues.size();
      if (remIndex > gridSize)
      {
        // Wrap around
        remIndex -= gridSize;
        remIndex++;
      }
    }

    // Increment "x"
    lineNumber++;
  }
}

void
ThreeGppSpectrumPropagationLossModel::ParseDelaySpread(const std::string &filePath, uint8_t skipLines)
{
  // This is a workaround-ish way to store Delay Spread in seconds for each point per gNB.
  // The thing is that we should have a separate delay spread value for each path, but the
  // provided input files contain only a single entry for each physical location for each gNB.
  // Thus, we use the approach similar to LOS data parsing, where we cache all Delay Spread values
  // and repeat them X times for each gNB, where X is amount of paths for this gNB on a specific
  // geographical position.
  std::ifstream file(filePath);
  NS_ASSERT_MSG(file.good(), "File not found: " << filePath);

  std::string line;
  for (uint8_t i = 0; i < skipLines; ++i)
    std::getline(file, line); // Skip headers

  while (std::getline(file, line))
  {
    std::istringstream stream(line);
    uint64_t index;
    double x, y, z, dist, spread;
    stream >> index >> x >> y >> z >> dist >> spread;

    Vector rxPos = m_WirelessInSiteIndexPositionMap[index];
    m_wiisDelaySpread.emplace(rxPos, spread);
  }
}
  
/// Parse DoA or DoD files
void 
ThreeGppSpectrumPropagationLossModel::ParseDoX(const std::string &filePath, uint8_t skipLines, std::function<void(std::istringstream &, Vector &)> parser)
{
  std::ifstream file(filePath);
  NS_ASSERT_MSG(file.good(), "File not found: " << filePath);

  std::string line;
  for (uint8_t i = 0; i < skipLines; ++i)
  {
    std::getline(file, line); // Skip headers
  }

  Vector rxPos;

  while (std::getline(file, line))
  {
    std::istringstream stream(line);

    uint32_t remIndex, entries;
    stream >> remIndex >> entries;
    rxPos = m_WirelessInSiteIndexPositionMap[remIndex];
    // std::cout << "Accessing rem index " << remIndex << " on position " << rxPos.x << " " << rxPos.y << " with " << entries << " entries" << std::endl;

    for (uint64_t i = 0; i < entries; ++i)
    {
      std::getline(file, line); // Read the next line with doa/dod data
      std::istringstream stream(line);
      parser(stream, rxPos);
    }
  }
}

void ThreeGppSpectrumPropagationLossModel::LoadInventoryWirelessInSite(std::string inventory_dir)
{
  // This methods loads FSPL, DOD and DOA data from output files of the Wireless Insight Ray Tracer. 
  // We try to store data in the same format to how ns3::LinkData expects it to be provided.
  // Key files that are required for each gNB in the simulation: X is the 3-digit index of a gNB
  // - "Frankfurt CF.fspl0.tX_60.r059.p2m" ;
  //      <X(m)> <Y(m)> <Z(m)> <Distance(m)> <FreeSpacePathLossWithoutAntennas(dB)>
  //      example entry:
  //        1 -245 -255 1.5 296.753 250

  // - "Frankfurt CF.doa.tX_60.r059.p2m" ;
  //      <receiver point number> <number of paths for this point>
  //      <path number> <arrival phi(deg)> <arrival theta(deg)> <received power(dBm)>
  //      example entry:
  //        142 4
  //        1 32.4595 88.342 -78.8056
  //        2 32.5206 88.3617 -82.2174
  //        3 53.132 88.8238 -89.021
  //        4 -15.367 88.8967 -89.9611

  // - "Frankfurt CF.dod.tX_60.r059.p2m" ;
  //      <receiver point number> <number of paths for this point>
  //      <path number> <arrival phi(deg)> <arrival theta(deg)> <received power(dBm)>
  //      example entry:
  //        355 4
  //        1 -56.6002 92.2094 -74.0086
  //        2 -91.8739 91.9698 -77.1823
  //        3 123.122 92.1744 -77.4754
  //        4 157.925 91.9469 -79.8499
  NS_LOG_UNCOND("Loading inventory from: " << inventory_dir);
  if (m_amountOfGnbs == 0)
  {
    NS_LOG_UNCOND("Simulation is configured to use 0 gNBs. No REM files will be parsed. Did you forget to set the 'AmountOfGnbs' attribute?");
  }

  for (uint32_t gnb = 1; gnb <= m_amountOfGnbs; ++gnb)
  {
    NS_LOG_UNCOND(" * loading inventory for gNB " << gnb);

    auto fsplFile = GenerateFilename(inventory_dir, "Frankfurt CF.fspl0.", gnb);
    // Skip first 3 lines of headers in FSPL0 files
    ParseFspl(fsplFile, 3, [&](std::istringstream &stream)
              {
                uint64_t index;
                double x, y, z, dist, pathLossNoAntennaGain;
                stream >> index >> x >> y >> z >> dist >> pathLossNoAntennaGain;
                // FIXME: TODO: add configurable offsets for x and y
                // Vector rxPos = {x, y, z};
                Vector rxPos = {x + m_xPosRemOffset, y + m_yPosRemOffset, z + m_zPosRemOffset};
                // TODO: add Error model to the rxPos to create a REM with measurement error?
                // .emplace will make it so that the first ever entry will persist.
                // 
                m_WirelessInSiteIndexPositionMap.emplace(index, rxPos);
                if (firstRemFile)
                {
                  // Create an empty entry.
                  allLinkData[rxPos] = Create<LinkData>();
                }
                // auto data = allLinkData[rxPos];
                // data->m_pathloss.push_back(-pathLossNoAntennaGain); // this needs to per-path, not per-gnb
                m_wiisPathLoss.emplace(rxPos, -pathLossNoAntennaGain);
              });
    
    // Load and cache LOS data, as we will need to use it when loading other data to append to the REM structure
    auto losFile = inventory_dir +  "los";
    losFile += std::to_string(gnb);
    losFile += ".txt";
    // Loads the data from the file to the m_wiisLos struct
    ParseLos(losFile, 1);

    auto delSprFile = GenerateFilename(inventory_dir, "Frankfurt CF.spread.", gnb);
    // Loads the data from the file to the m_wiisDelaySpread struct
    ParseDelaySpread(delSprFile, 3);

    auto doaFile = GenerateFilename(inventory_dir, "Frankfurt CF.doa.", gnb);
    ParseDoX(doaFile, 6, [&](std::istringstream &stream, Vector &rxPos)
              {
                uint16_t subIndex;
                double phi, theta, rxPower;
                stream >> subIndex >> phi >> theta >> rxPower;
                auto data = allLinkData[rxPos];
                data->m_path++;
                data->m_txId.push_back(gnb);
                if (phi < 0)
                  phi += 360; // Azimuth values are expected to be in range [0,360)
                data->m_aoaAzimuth.push_back(phi);
                // convert WiIS elevation into default REM elevation.
                // TODO: provide better explanation
                // def REM provides negative angles when the direction is downwards
                // WiIS REM provides values larger than 90 for downwards
                theta = 90 - theta;
                data->m_aoaElevation.push_back(theta);
                // Fetch the LOS entry here
                // At the moment of writing the code the LOS values are interpreted differently in ns-3 and
                // Wireless InSite ray tracer. The following "switch" statement converts WiIS LOS values 
                // to values that are expected by the ns-3.
                short int los = 0; // no data
                switch (static_cast<int>(m_wiisLos.at(rxPos)))
                {
                  case 0:    // wiis NLOS
                    los = 2; // def REM NLOS
                    break;
                  case 1:    // wiis LOS
                    los = 1; // default REM LOS
                    break;
                  case -1:   // wiis no data
                    los = 2; // default REM no data
                    break;
                  default:
                    break; 
                }
                data->m_los.push_back(los);
                data->m_delay.push_back(m_wiisDelaySpread.at(rxPos));
                //data->m_pathloss.push_back(m_wiisPathLoss.at(rxPos));
                // Trying to compute PL from the constant TX power and parsed RX power
                // FIXME: TODO: is tx power fixed at 15dBm? if so, make it an attribute
                // Total TX power is m_txPower*2.
                data->m_pathloss.push_back(-(m_txPower*2 - rxPower));
              });

    auto dodFile = GenerateFilename(inventory_dir, "Frankfurt CF.dod.", gnb);
    ParseDoX(dodFile, 6, [&](std::istringstream &stream, Vector &rxPos)
              {
                uint16_t index;
                double phi, theta, power;
                stream >> index >> phi >> theta >> power;
                auto data = allLinkData[rxPos];
                if (phi < 0)
                  phi += 360; // Azimuth values are expected to be in range [0,360)
                data->m_aodAzimuth.push_back(phi);
                // convert WiIS elevation into default REM elevation.
                // TODO: provide better explanation
                // def REM provides negative angles when the direction is downwards
                // WiIS REM provides values larger than 90 for downwards
                theta = 90 - theta;
                data->m_aodElevation.push_back(theta);    
              });

    // auto delSprFile = GenerateFilename(inventory_dir, "Frankfurt CF.spread.", gnb);
    // Skip first 3 lines of headers in 'spread' files
    // ParseDelaySpread(delSprFile, 3, [&](std::istringstream &stream)
    //           {
    //             uint64_t index;
    //             double x, y, z, dist, spread;
    //             stream >> index >> x >> y >> z >> dist >> spread;
    //             // FIXME: TODO: add configurable offsets for x and y
    //             // Vector rxPos = {x, y, z};
    //             Vector rxPos = {x + 370, y + 380, z};
    //             auto data = allLinkData[rxPos];
    //             data->m_delay.push_back(spread);
    //           });

    firstRemFile = false;
    // Clear the LOS, Delay Spread nad Path Loss maps to prepare them for next gNB's data.
    m_wiisLos.clear();
    m_wiisDelaySpread.clear();
    m_wiisPathLoss.clear();
  }

  // Verifying some random point
  Vector rxPos = m_WirelessInSiteIndexPositionMap.at(168114);
  std::cout << "Verifying REM entry at x: " << rxPos.x << " y: " << rxPos.y << std::endl;
  std::cout << " m_path: " << allLinkData.at(rxPos)->m_path << std::endl;
  std::cout << " m_txId: " << std::endl << "  ";
  for (auto entry : allLinkData.at(rxPos)->m_txId)
  {
    std::cout << entry << " ";
  }
  std::cout << std::endl << " m_aodAzimuth: " << std::endl << "  ";
  for (auto entry : allLinkData.at(rxPos)->m_aodAzimuth)
  {
    std::cout << entry << " ";
  }
  std::cout << std::endl << " m_aoaAzimuth: " << std::endl << "  ";
  for (auto entry : allLinkData.at(rxPos)->m_aoaAzimuth)
  {
    std::cout << entry << " ";
  }
  std::cout << std::endl;

  std::cout << std::endl << " m_delay: " << std::endl << "  ";
  for (auto entry : allLinkData.at(rxPos)->m_delay)
  {
    std::cout << entry << " ";
  }
  std::cout << std::endl;

  std::cout << "Writing the REM into file" << std::endl;
  WriteRemToFile("Inventory_WiIS.txt", allLinkData);
  std::cout << "Writing finished" << std::endl;

  // Clear the LOS, Delay Spread and Path Loss maps to release some memory.
  m_wiisLos.clear();
  m_wiisDelaySpread.clear();
  m_wiisPathLoss.clear();
}

void
ThreeGppSpectrumPropagationLossModel::WriteRemToFile(const std::string& filename, const std::map<Vector, Ptr<LinkData>>& linkData)
{
    std::ofstream outFile(filename, std::ios::binary); // Open file in binary mode
    if (!outFile.is_open())
    {
        throw std::runtime_error("Unable to open file for writing.");
    }

    // Write intVector size and data
    for (auto remEntry : allLinkData)
    {
      outFile << remEntry.first.x << "," << remEntry.first.y << "," << remEntry.first.z << std::endl;
      outFile << remEntry.second->m_path << std::endl;

      // size_t vectorSize = remEntry.second->m_los.size();
      // std::copy(remEntry.second->m_los.begin(), remEntry.second->m_los.end() - 1, std::ostream_iterator<double>(outFile, ",")); // Write all but last with commas
      // outFile << remEntry.second->m_los.back() << "\n"; // Write the last element without a trailing comma

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
         
        // // Use ostringstream to build the output string
        // std::ostringstream ss;
        // for (const auto& val : remEntry.second->m_txId)
        // {
        //     ss << val << ","; // Append each value followed by a comma
        // }
        // std::string line = ss.str(); // Get the full string
        // line.pop_back(); // Remove the trailing comma

        // outFile << line << "\n"; // Write the full line
      }
      else
      {
        // This REM entry is empty, but we still need to write zeros for all fields
        for (uint8_t it=1; it < 9; it++)
          outFile << 0 << std::endl;
      }
    }

    // // Write doubleVector size and data
    // size_t doubleSize = data.doubleVector.size();
    // outFile.write(reinterpret_cast<const char*>(&doubleSize), sizeof(doubleSize)); // Write size
    // outFile.write(reinterpret_cast<const char*>(data.doubleVector.data()), doubleSize * sizeof(double)); // Write data

    // // Write stringVector size
    // size_t stringSize = data.stringVector.size();
    // outFile.write(reinterpret_cast<const char*>(&stringSize), sizeof(stringSize)); // Write size

    // // Write each string
    // for (const auto& str : data.stringVector)
    // {
    //     size_t len = str.size();
    //     outFile.write(reinterpret_cast<const char*>(&len), sizeof(len)); // Write string length
    //     outFile.write(str.c_str(), len); // Write string content
    // }

    outFile.close();
}

Vector
ThreeGppSpectrumPropagationLossModel::GetCurrentUePosition(uint16_t imsi)
{
  return walkCords.at(imsi).at(DynamicCast<ThreeGppChannelModel> (m_channelModel)->GetTraceIndex ());
}

void
ThreeGppSpectrumPropagationLossModel::SMSetRaySourceType (std::string type)
{
  m_smRaySourceType = type;
}

std::string
ThreeGppSpectrumPropagationLossModel::SMGetRaySourceType ()
{
  return m_smRaySourceType;
}

std::string
ThreeGppSpectrumPropagationLossModel::GetTraceFileName (Vector enbLoc)
{
  std::stringstream ss;
  ss << enbLoc.x;
  ss << enbLoc.y;
  ss << enbLoc.z;
  std::string str = input_files_folder + ss.str () + ".txt";
  return str;
}

void
ThreeGppSpectrumPropagationLossModel::SetBeamSweepState (bool beamSweepState)
{
  DynamicCast<ThreeGppChannelModel> (m_channelModel)->DoSetBeamSweepState (beamSweepState);
}

std::pair<std::pair<double, double>, std::pair<double, double>>
ThreeGppSpectrumPropagationLossModel::GetAodAoaFromCM (Vector enbLoc, uint16_t imsi)
{
  return DynamicCast<ThreeGppChannelModel> (m_channelModel)->DoGetAoaAod (enbLoc, imsi);
}

}  // namespace ns3
