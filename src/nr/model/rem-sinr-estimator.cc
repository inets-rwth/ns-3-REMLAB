#include "rem-sinr-estimator.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <random>

#include <ns3/nr-gnb-phy.h>
#include <ns3/nr-gnb-net-device.h>
#include <ns3/nr-ue-net-device.h>
#include <ns3/nr-spectrum-value-helper.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("RemSinrEstimator");
NS_OBJECT_ENSURE_REGISTERED (RemSinrEstimator);

std::random_device rd;
std::mt19937 mt (rd ());

std::uniform_real_distribution<double> m_uniformDist (0, 1);
std::uniform_real_distribution<double> m_uniformAzi (1, 360);
std::uniform_real_distribution<double> m_uniformEle (1, 20);
std::uniform_real_distribution<double> m_uniformPower (-80, -120);

//Table 7.5-3: Ray offset angles within a cluster, given for rms angle spread normalized to 1.
static const double offSetAlpha[20] = {
  0.0447,-0.0447,0.1413,-0.1413,0.2492,-0.2492,0.3715,-0.3715,0.5129,-0.5129,0.6797,-0.6797,0.8844,-0.8844,1.1481,-1.1481,1.5195,-1.5195,2.1551,-2.1551
};

/*
 * The cross correlation matrix is constructed according to table 7.5-6.
 * All the square root matrix is being generated using the Cholesky decomposition
 * and following the order of [SF,K,DS,ASD,ASA,ZSD,ZSA].
 * The parameter K is ignored in NLOS.
 *
 * The Matlab file to generate the matrices can be found in
 * https://github.com/nyuwireless-unipd/ns3-mmwave/blob/master/src/mmwave/model/BeamFormingMatrix/SqrtMatrix.m
 *
 */
static const double sqrtC_RMa_LOS[7][7] = {
  {1, 0, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0, 0},
  {-0.5, 0, 0.866025, 0, 0, 0, 0},
  {0, 0, 0, 1, 0, 0, 0},
  {0, 0, 0, 0, 1, 0, 0},
  {0.01, 0, -0.0519615, 0.73, -0.2, 0.651383, 0},
  {-0.17, -0.02, 0.21362, -0.14, 0.24, 0.142773, 0.909661},
};

static const double sqrtC_RMa_NLOS[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {-0.5, 0.866025, 0, 0, 0, 0},
  {0.6, -0.11547, 0.791623, 0, 0, 0},
  {0, 0, 0, 1, 0, 0},
  {-0.04, -0.138564, 0.540662, -0.18, 0.809003, 0},
  {-0.25, -0.606218, -0.240013, 0.26, -0.231685, 0.625392},
};

static const double sqrtC_RMa_O2I[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0},
  {0, 0, -0.7, 0.714143, 0, 0},
  {0, 0, 0.66, -0.123225, 0.741091, 0},
  {0, 0, 0.47, 0.152631, -0.393194, 0.775373},
};

static const double sqrtC_UMa_LOS[7][7] = {
  {1, 0, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0, 0},
  {-0.4, -0.4, 0.824621, 0, 0, 0, 0},
  {-0.5, 0, 0.242536, 0.83137, 0, 0, 0},
  {-0.5, -0.2, 0.630593, -0.484671, 0.278293, 0, 0},
  {0, 0, -0.242536, 0.672172, 0.642214, 0.27735, 0},
  {-0.8, 0, -0.388057, -0.367926, 0.238537, -3.58949e-15, 0.130931},
};


static const double sqrtC_UMa_NLOS[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {-0.4, 0.916515, 0, 0, 0, 0},
  {-0.6, 0.174574, 0.78072, 0, 0, 0},
  {0, 0.654654, 0.365963, 0.661438, 0, 0},
  {0, -0.545545, 0.762422, 0.118114, 0.327327, 0},
  {-0.4, -0.174574, -0.396459, 0.392138, 0.49099, 0.507445},
};

static const double sqrtC_UMa_O2I[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {-0.5, 0.866025, 0, 0, 0, 0},
  {0.2, 0.57735, 0.791623, 0, 0, 0},
  {0, 0.46188, -0.336861, 0.820482, 0, 0},
  {0, -0.69282, 0.252646, 0.493742, 0.460857, 0},
  {0, -0.23094, 0.16843, 0.808554, -0.220827, 0.464515},

};

static const double sqrtC_UMi_LOS[7][7] = {
  {1, 0, 0, 0, 0, 0, 0},
  {0.5, 0.866025, 0, 0, 0, 0, 0},
  {-0.4, -0.57735, 0.711805, 0, 0, 0, 0},
  {-0.5, 0.057735, 0.468293, 0.726201, 0, 0, 0},
  {-0.4, -0.11547, 0.805464, -0.23482, 0.350363, 0, 0},
  {0, 0, 0, 0.688514, 0.461454, 0.559471, 0},
  {0, 0, 0.280976, 0.231921, -0.490509, 0.11916, 0.782603},
};

static const double sqrtC_UMi_NLOS[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {-0.7, 0.714143, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0},
  {-0.4, 0.168034, 0, 0.90098, 0, 0},
  {0, -0.70014, 0.5, 0.130577, 0.4927, 0},
  {0, 0, 0.5, 0.221981, -0.566238, 0.616522},
};

static const double sqrtC_UMi_O2I[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {-0.5, 0.866025, 0, 0, 0, 0},
  {0.2, 0.57735, 0.791623, 0, 0, 0},
  {0, 0.46188, -0.336861, 0.820482, 0, 0},
  {0, -0.69282, 0.252646, 0.493742, 0.460857, 0},
  {0, -0.23094, 0.16843, 0.808554, -0.220827, 0.464515},
};

static const double sqrtC_office_LOS[7][7] = {
  {1, 0, 0, 0, 0, 0, 0},
  {0.5, 0.866025, 0, 0, 0, 0, 0},
  {-0.8, -0.11547, 0.588784, 0, 0, 0, 0},
  {-0.4, 0.23094, 0.520847, 0.717903, 0, 0, 0},
  {-0.5, 0.288675, 0.73598, -0.348236, 0.0610847, 0, 0},
  {0.2, -0.11547, 0.418943, 0.541106, 0.219905, 0.655744, 0},
  {0.3, -0.057735, 0.73598, -0.348236, 0.0610847, -0.304997, 0.383375},
};

static const double sqrtC_office_NLOS[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {-0.5, 0.866025, 0, 0, 0, 0},
  {0, 0.46188, 0.886942, 0, 0, 0},
  {-0.4, -0.23094, 0.120263, 0.878751, 0, 0},
  {0, -0.311769, 0.55697, -0.249198, 0.728344, 0},
  {0, -0.069282, 0.295397, 0.430696, 0.468462, 0.709214},
};

static constexpr uint32_t GetKey (uint32_t x1, uint32_t x2)
{
 return (((x1 + x2) * (x1 + x2 + 1)) / 2) + x2;
}

TypeId
RemSinrEstimator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RemSinrEstimator")
    .SetParent<Object> ()
    .SetGroupName ("nr")
    .AddConstructor<RemSinrEstimator> ()

    .AddAttribute ("MaxWalkPathIndex",
      "Maximum Walk Path Index (total amount of steps of a walk path)",
      UintegerValue (1000),
      MakeUintegerAccessor (&RemSinrEstimator::maxTraceIndexCM),
      MakeUintegerChecker<uint32_t> ())

    .AddAttribute ("CarrierFrequency",
      "Carrier Frequency as set for the PHY Layer",
      DoubleValue (28e9),
      MakeDoubleAccessor (&RemSinrEstimator::m_frequency),
      MakeDoubleChecker<double> ())

    .AddAttribute ("WalkSamplingPeriod",
      "Amount of time in seconds between every walk path coordinate change",
      DoubleValue (0.25),
      MakeDoubleAccessor (&RemSinrEstimator::walkSamplingPeriod),
      MakeDoubleChecker<double> ())

    .AddAttribute ("AntennaSeparation",
      "AntennaSeparation",
      DoubleValue (0.5),
      MakeDoubleAccessor (&RemSinrEstimator::m_antennaSeparation),
      MakeDoubleChecker<double> ())

    .AddAttribute ("Scenario",
      "The 3GPP scenario (RMa, UMa, UMi-StreetCanyon, InH-OfficeOpen, InH-OfficeMixed)",
      StringValue ("UMi-StreetCanyon"),
      MakeStringAccessor (&RemSinrEstimator::m_scenario),
      MakeStringChecker ());

  return tid;
}

RemSinrEstimator::RemSinrEstimator()
{
    m_uniformRv = CreateObject<UniformRandomVariable> ();
    m_uniformRvShuffle = CreateObject<UniformRandomVariable> ();

    m_normalRv = CreateObject<NormalRandomVariable> ();
    m_normalRv->SetAttribute ("Mean", DoubleValue (0.0));
    m_normalRv->SetAttribute ("Variance", DoubleValue (1.0));

    walkSamplingPeriod = 0.25;
    m_antennaSeparation = 0.5;
    m_channelMatrixMap.clear ();

    m_gnbNetDeviceFactory.SetTypeId (NrGnbNetDevice::GetTypeId ());
    m_ueNetDeviceFactory.SetTypeId (NrUeNetDevice::GetTypeId ());
    m_gnbAntennaFactory.SetTypeId (ThreeGppAntennaArrayModel::GetTypeId ());
    m_ueAntennaFactory.SetTypeId (ThreeGppAntennaArrayModel::GetTypeId ());

    m_ueAntennaFactory.Set ("NumRows", UintegerValue (ueNumRows));
    m_ueAntennaFactory.Set ("NumColumns", UintegerValue (ueNumColumns));

    m_gnbAntennaFactory.Set ("NumRows", UintegerValue (gNBNumRows));
    m_gnbAntennaFactory.Set ("NumColumns", UintegerValue (gNBNumColumns));

    m_channelConditionModelFactory.SetTypeId (ThreeGppUmiStreetCanyonChannelConditionModel::GetTypeId ());

    // sector 10 wraps around, but we need to be in [0,360) range
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

    // sector 5 wraps around, but we need to be in [0,360) range
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

RemSinrEstimator::~RemSinrEstimator()
{
}

void
RemSinrEstimator::InitializeAllParameters()
{
    if (gnbLocations.size() == 0)
    {
        std::cout << "RemSinrEstimator::InitializeAllParameters: no gNB locations. SetGnbLocations needs to be called." << std::endl;
        return;
    }

    gnbNodes.Create (gnbLocations.size());
    ueNodes.Create (1);

    MobilityHelper enbmobility;
    enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    enbmobility.Install (gnbNodes);

    MobilityHelper uemobility;
    uemobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    uemobility.Install (ueNodes);

    // For each gNB, need to store a dummy NetDevice and create an antenna for it.
    for (const auto& gnbLocation : gnbLocations)
    {
        Ptr<ThreeGppAntennaArrayModel> antennaGnb = m_gnbAntennaFactory.Create <ThreeGppAntennaArrayModel> ();
        // iNETs antenna config
        // horizontal - 9 for gNB, 18 for UE. Vertical 30 for both
        antennaGnb->SetAttribute ("HorizontalBeamStep", DoubleValue (9));
        antennaGnb->SetAttribute ("VerticalBeamStep", DoubleValue (30));

        //AddDevice(devGnb, antennaGnb);
        std::cout << " addidng antenna for gNB at location " <<  gnbLocation << std::endl;
        std::cout << "  - gNB node ID: " << gnbNodes.Get(m_nodeIds)->GetId() << std::endl;
        // AddDevice(m_nodeIds, antennaGnb);
        AddDevice(gnbNodes.Get(m_nodeIds)->GetId(), antennaGnb);

        gnbNodes.Get(m_nodeIds)->GetObject<MobilityModel>()->SetPosition(gnbLocation);
        std::cout << " gnb node ID " << m_nodeIds << " set position " << gnbLocation << std::endl;
        m_nodeIds++;
    }
    // Add UE
    Ptr<ThreeGppAntennaArrayModel> antennaUe = m_ueAntennaFactory.Create <ThreeGppAntennaArrayModel> ();
    antennaUe->SetAttribute ("HorizontalBeamStep", DoubleValue (18));
    DoubleValue test = 0.;
    antennaUe->GetAttribute("HorizontalBeamStep", test);
    std::cout << " UE antenna created with hor beam step: " << test.Get() << std::endl;
    antennaUe->SetAttribute ("VerticalBeamStep", DoubleValue (30));
    // AddDevice(devUe, antennaUe);
    AddDevice(ueNodes.Get(0)->GetId(), antennaUe);
    
    std::cout << "RemSinrEstimator::InitializeAllParameters: antennas installed. m_deviceAntennaMap filled." << std::endl;

    m_channelConditionModel  = m_channelConditionModelFactory.Create<ChannelConditionModel>();

}

std::map<uint16_t, std::pair<uint16_t, double>>
RemSinrEstimator::EstimateSinrForUeLocation(Ptr<const LinkData> remEntry, Vector ueCords)
{
    m_gnbToRemIndexAndSnrMap.clear();
    m_gnbToRemIndexAndSnrMapWithInterf.clear();

    std::cout << "RemSinrEstimator::EstimateSinrForUeLocation: ueCords: " << ueCords << std::endl;
    if(ueNodes.Get(0)->GetObject<MobilityModel>() == nullptr)
    {
        std::cout << "Cant get UE mobility model" << std::endl;
    }
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(ueCords);
    std::cout << " UE node changes position to " << ueNodes.Get(0)->GetObject<MobilityModel>()->GetPosition() << std::endl;
    // count unique gNBs in the REM entry

    std::vector<int> uniqueGnbIds;
    std::vector<int> startIndices;
    std::vector<double> rxPowers; // dBm
    int servingIndex = 0;
    // index in REM entry, gNB ID and RX power
    std::vector<std::pair<int, std::pair<uint16_t, double>>> remIndexAndGnbWRxPower; // REM index for the gNB with RX power from it
    // index in REM entry, gNB ID and AoA azimuth
    std::map<int, std::pair<int, double>> remIndexAndAoa; // AoA value matching the RX power from gNB identified above

    int lastId = remEntry->m_txId[0];
    uniqueGnbIds.push_back(lastId);
    startIndices.push_back(0);
    rxPowers.push_back(2 * 15 + remEntry->m_pathloss.at(0));
    remIndexAndGnbWRxPower.push_back(std::make_pair<int, std::pair<uint16_t, double>>(0, std::make_pair<uint16_t, double>(lastId, 2* 15 + remEntry->m_pathloss.at(0))));
    remIndexAndAoa.emplace(0, std::make_pair(lastId, remEntry->m_aoaAzimuth.at(0)));

    for (size_t i = 1; i < remEntry->m_txId.size(); ++i)
    {
        if (remEntry->m_txId[i] != lastId)
        {
            lastId = remEntry->m_txId[i];
            uniqueGnbIds.push_back(lastId);
            startIndices.push_back(i);
            // PL = - (P_tx * 2 - P_rx)
            // -PL = P_tx * 2 - P_rx
            // P_rx = 2*P_tx + PL
            rxPowers.push_back(2 * 15 + remEntry->m_pathloss.at(i));
            remIndexAndGnbWRxPower.push_back(std::make_pair<int, std::pair<uint16_t, double>>(i, std::make_pair<uint16_t, double>(lastId, 2* 15 + remEntry->m_pathloss.at(i))));
            remIndexAndAoa.emplace(i, std::make_pair(lastId, remEntry->m_aoaAzimuth.at(i)));
        }
    }

    // each entry of the startIndices is the index of next best path of next gNB
    // Iterate over all available gNBs in this REM entry and estimate SINR for the case
    // when each gNB is used as the serving one
    uint16_t j = 0;
    m_candidateGnbToBplMap.clear();
    // m_nodeIds = 15 at this point
    for (auto indexGnb : startIndices)
    {
        uint16_t nodeIndex = remEntry->m_txId.at(indexGnb) - 1;

        double aoaAzimuth = remEntry->m_aoaAzimuth.at(indexGnb);
        double aoaElevation = remEntry->m_aoaElevation.at(indexGnb);
        double aodAzimuth = remEntry->m_aodAzimuth.at(indexGnb);
        double aodElevation = remEntry->m_aodElevation.at(indexGnb);

        std::pair<BeamId, BeamId> bpls = MapAoaAodToSectors(aoaAzimuth, aoaElevation, aodAzimuth, aodElevation);
        m_candidateGnbToBplMap.emplace(remEntry->m_txId.at(indexGnb), bpls);
        // Change antenna configurations of each gNB
        BeamformingVector remBfvGnb = BeamformingVector (
            CreateDirectionalBfv (m_deviceAntennaMap.at(gnbNodes.Get(nodeIndex)->GetId()),
            bpls.first.GetSector (),
            bpls.first.GetElevation (), true),
            bpls.first
          );
        m_deviceAntennaMap.at(gnbNodes.Get(/*j*/nodeIndex)->GetId())->SetBeamformingVector(remBfvGnb.first);
        j++;
    }
    // At this point all available gNBs point beams towards the UE on their best paths.

    // Now for each BFV of the UE from the collected BPLs, calculate the corresponding SNR
    uint16_t gnbRemIndex = 0;
    for (const auto& [gnbId, bpl] : m_candidateGnbToBplMap)
    {
        BeamformingVector remBfvUE = BeamformingVector (
            CreateDirectionalBfv (m_deviceAntennaMap.at(ueNodes.Get(0)->GetId()),
            bpl.second.GetSector (),
            bpl.second.GetElevation (), true),
            bpl.second
          );
        m_deviceAntennaMap.at(ueNodes.Get(0)->GetId())->SetBeamformingVector(remBfvUE.first);

        // Antennas are prepared for the test. Now need to create a dummy signal and measure SNR

        //m_txPsd should be 256 subcarriers with some power.
        Ptr<SpectrumValue> txPsd;

        double m_txPower = 20.; // 20 dB
        // sm params taken from NrPhy::GetSpectrumModel ()
        uint32_t rbNum = 264; // == GetNumBands()
        uint32_t subcarrierSpacing = 120000;
        Ptr<const SpectrumModel> sm = NrSpectrumValueHelper::GetSpectrumModel(
            rbNum,
            m_frequency,
            subcarrierSpacing);
        std::vector<int> rbIndexVector; // filled with values [0,263]
        for (int i = 0; i < 264; i++)
        {
            rbIndexVector.push_back(i); // fetched from NrPhy::GetTxPowerSpectralDensity
        }
        txPsd = NrSpectrumValueHelper::CreateTxPowerSpectralDensity  (m_txPower, rbIndexVector, sm);

        Ptr<SpectrumValue> rxPsd = DoCalcRxPowerSpectralDensity(
                remEntry,
                txPsd,
                gnbNodes.Get(gnbId-1)->GetObject<MobilityModel>(),
                ueNodes.Get(0)->GetObject<MobilityModel>());

        // scale and convert to SINR
        double antennaGain = 9.94; // from NrSpectrumPhy::StartRx
        double antennaGainLinear = std::pow(10, antennaGain / 10);
        double m_noiseFigure= 5; // default value
        Ptr<SpectrumValue> m_noisePsd = NrSpectrumValueHelper::CreateNoisePowerSpectralDensity (m_noiseFigure, sm);
        SpectrumValue sinr = *(rxPsd)*antennaGainLinear / (*m_noisePsd);
        double snr = 10*log10( Sum(sinr) / (rbNum) );
        m_gnbToRemIndexAndSnrMap.emplace(gnbId, std::make_pair(startIndices.at(gnbRemIndex), snr));
        
        gnbRemIndex += 1;
    }
    
    return m_gnbToRemIndexAndSnrMap;
}

std::pair<BeamId, BeamId>
RemSinrEstimator::MapAoaAodToSectors(
    const double aoaAzimuth,
    const double aoaElevation,
    const double aodAzimuth,
    const double aodElevation)
{
    uint16_t remSectorGnb{0};
    double remElevationGnb{90.};
    uint16_t remSectorUe{0};
    double remElevationUe{90.};

    // Map gNB angles to sectors
    for (const auto& entry : m_gnbSectorDegreeMap)
    {
      if (aodAzimuth >= entry.first.lowerBoundDeg && aodAzimuth <= entry.first.upperBoundDeg)
      {
        remSectorGnb = entry.second;
        break;
      }
    }
    for (const auto& entry : m_gnbElevationDegreeMap)
    {
      // negative angles for gNB
      if (aodElevation <= entry.first.first && aodElevation >= entry.first.second)
      {
        remElevationGnb = entry.second;
        break;
      }
    }

    // Map UE angles to sectors
    for (const auto& entry : m_ueSectorDegreeMap)
    {
      if (aoaAzimuth >= entry.first.lowerBoundDeg && aoaAzimuth <= entry.first.upperBoundDeg)
      {
        remSectorUe = entry.second;
        break;
      }
    }
    for (const auto& entry : m_ueElevationDegreeMap)
    {
      // positive angles for UE
      if (aoaElevation >= entry.first.first && aoaElevation <= entry.first.second)
      {
        remElevationUe = entry.second;
        break;
      }
    }

    BeamId gnbBeamId{remSectorGnb, remElevationGnb};
    BeamId ueBeamId{remSectorUe, remElevationUe};
    return std::make_pair(gnbBeamId, ueBeamId);

}

void
RemSinrEstimator::ChangeAntennaConfigurations(Ptr<RxRayData> linkData)
{
    // Change BFV of all gNB antennas according to the linkData
    // FIXME: actually, can' use RxRayData, as it contains no info about gNB IDs

    // Take a look at azimuth and elevation values for the available gNBs of the currently used REM entry. 
}

void
RemSinrEstimator::SetGnbLocations(const std::map<uint16_t, Vector>& gnblocs)
{
    std::cout << "RemSinrEstimator::SetGnbLocations: " << std::endl;
    for (auto gnbLoc : gnblocs)
    {
        Vector location{gnbLoc.second.x, gnbLoc.second.y, gnbLoc.second.z};
        // FIXME: had to remove offset as PHY layer does not have them, but LTE CO uses it for the recovery algorithm
        location.x = location.x - 128;
        location.y = location.y - 121;
        gnbLocations.push_back(location);
        std::cout << " " << location << std::endl;
    }
}

Ptr<RxRayData>
RemSinrEstimator::GetRxRayData(Ptr<const LinkData> linkData, Vector ueCords, Vector gnbLocation)
{
    // rounding off to nearest integer
    ueCords.x = std::round(ueCords.x);
    ueCords.y = std::round(ueCords.y);

    // get current eNB location
    uint16_t enbIndex1;
    for (unsigned int j = 0; j < gnbLocations.size(); j++)
    {
      if (gnbLocations.at(j).x == gnbLocation.x && gnbLocations.at(j).y == gnbLocation.y)
      {
        enbIndex1 = j;
        break;
      }
    }

    Ptr<RxRayData> rxRayData;
    // tempRxRayData will contain only entries for the selected gNB
    Ptr<RxRayData> tempRxRayData = Create<RxRayData>();
    if (linkData->m_path > 0)
    {
      int16_t inventoryIndex = -1;
      int16_t inventoryIndexEnd = -1;
  
      //NS_LOG_UNCOND ("inventoryIndex = "<< inventoryIndex);
      //NS_LOG_UNCOND ("inventoryIndexEnd = "<< inventoryIndexEnd);
  
      for (unsigned int m = 0; m < linkData->m_path; m++)
      {
        if (linkData->m_txId[m] == (enbIndex1 + 1))
        {
          inventoryIndex = m;
          break;
        }
      }
      if (inventoryIndex != -1)
      {
        for (unsigned int n = inventoryIndex; n < linkData->m_path; n++)
        {
          if (linkData->m_txId[n] == (enbIndex1 + 1))
          {
            inventoryIndexEnd = n;
          }
        }
      }
  
      if (inventoryIndex != -1)
      {
        //bool isLosBlocked = false;
        bool isLinkBlocked = false;
  
        //tempRxRayData->m_path = 0;
        //for (int i = 0; i < rxRayData->m_path; i++)
  
        for (int i = 0; i < (inventoryIndexEnd - inventoryIndex + 1); i++)
        {
          if (isLinkBlocked)
          {
            inventoryIndex++;
            continue;
          }
  
          tempRxRayData->m_los.push_back(linkData->m_los[inventoryIndex]);
          tempRxRayData->m_pathloss.push_back(linkData->m_pathloss[inventoryIndex]);
          tempRxRayData->m_aoaAzimuth.push_back(linkData->m_aoaAzimuth[inventoryIndex]);
          tempRxRayData->m_aoaElevation.push_back(linkData->m_aoaElevation[inventoryIndex]);
          tempRxRayData->m_aodAzimuth.push_back(linkData->m_aodAzimuth[inventoryIndex]);
          tempRxRayData->m_aodElevation.push_back(linkData->m_aodElevation[inventoryIndex]);
          tempRxRayData->m_delay.push_back(linkData->m_delay[inventoryIndex]);
  
          inventoryIndex++;
        }
        tempRxRayData->m_path = tempRxRayData->m_pathloss.size();
    
        if (tempRxRayData->m_path > 0)
        {
          // Select the strongst component
          uint8_t idx = GetIndexForStrongestPath(tempRxRayData->m_pathloss);
          // as tempRxRayData contains entries from the selected gNB and entries are already sorted
          // this idx will probably always be 0
  
          doubleVector_t temp;
  
          temp.push_back(tempRxRayData->m_pathloss.at(idx));
          tempRxRayData->m_pathloss = temp;
          temp.at(0) = (tempRxRayData->m_los.at(idx));
          tempRxRayData->m_los = temp;
          temp.at(0) = tempRxRayData->m_delay.at(idx);
          tempRxRayData->m_delay = temp;
          temp.at(0) = tempRxRayData->m_aoaAzimuth.at(idx);
          tempRxRayData->m_aoaAzimuth = temp;
          temp.at(0) = tempRxRayData->m_aoaElevation.at(idx);
          tempRxRayData->m_aoaElevation = temp;
          temp.at(0) = tempRxRayData->m_aodAzimuth.at(idx);
          tempRxRayData->m_aodAzimuth = temp;
          temp.at(0) = tempRxRayData->m_aodElevation.at(idx);
          tempRxRayData->m_aodElevation = temp;
        }
      }
    }
    rxRayData = tempRxRayData;
    rxRayData->m_path = rxRayData->m_pathloss.size();

    return rxRayData;
  
}


uint8_t
RemSinrEstimator::GetIndexForStrongestPath (doubleVector_t pathloss)
{
  double bestPath = pathloss.at (0);
  uint16_t index = 0;

  for (unsigned int pathIndex = 0; pathIndex < pathloss.size (); pathIndex++)
    {
      if (pathloss.at (pathIndex) > bestPath)
        {
          index = pathIndex;
          bestPath = pathloss.at (pathIndex);
        }
    }

  return index;
}

// see ThreeGppChannelModel::GetNewChannel
Ptr<ThreeGppChannelModel::ThreeGppChannelMatrix>
RemSinrEstimator::GetNewChannel (Ptr<const LinkData> linkData, Ptr<const MobilityModel> a, Ptr<const MobilityModel> b,
                                     Vector locUT, bool los, bool o2i,
                                     Ptr<const ThreeGppAntennaArrayModel> sAntenna,
                                     Ptr<const ThreeGppAntennaArrayModel> uAntenna,
                                     Angles &uAngle, Angles &sAngle,
                                     double dis2D, double hBS, double hUT)
{

  NS_LOG_FUNCTION (this);

  Ptr<const ParamsTable> table3gpp = GetThreeGppTable (los, o2i, hBS, hUT, dis2D);

  NS_LOG_INFO("a: x=" << a->GetPosition().x << " y=" << a->GetPosition().y << std::endl << "b: x=" << b->GetPosition().x << " y=" << b->GetPosition().y);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  uint32_t traceIndex = GetTraceIndex();

  Vector enbLoc(0,0,0);
  bool isTargetEnb=false;

  //With this method a eNB is recognized by matching the position from the loaded data
  NS_LOG_INFO("size="<<gnbLocations.size());
  if(gnbLocations.size()!=0)
  {
      for(unsigned int i=0;i<gnbLocations.size();i++)
       {
         NS_LOG_INFO("enb i="<<i<<": x="<<gnbLocations.at(i).x<<" y="<<gnbLocations.at(i).y);
         if(a->GetPosition().x == gnbLocations.at(i).x && a->GetPosition().y == gnbLocations.at(i).y)
          {
             enbLoc.x=a->GetPosition ().x;
             enbLoc.y=a->GetPosition ().y;
             enbLoc.z=a->GetPosition ().z;
             isTargetEnb=false;
             break;
          }
         else if(b->GetPosition().x == gnbLocations.at(i).x && b->GetPosition().y == gnbLocations.at(i).y)
          {
             enbLoc.x=b->GetPosition ().x;
             enbLoc.y=b->GetPosition ().y;
             enbLoc.z=b->GetPosition ().z;
             isTargetEnb=true;
             break;
          }
       }
      NS_ABORT_MSG_IF(enbLoc.x==0 && enbLoc.y==0 && enbLoc.z==0, "Neither a nor b is a valid gNB");
  }
  else
  {
       enbLoc.x=a->GetPosition ().x;
       enbLoc.y=a->GetPosition ().y;
       enbLoc.z=a->GetPosition ().z;
       isTargetEnb=false;
       NS_LOG_INFO("ENB at "<<enbLoc);
  }
  
  //What does bestPath do??
  bool bestPath=true;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  NS_LOG_INFO("Searching for RayData at traceIndex="<<traceIndex<<" enbLoc "<<enbLoc);
  Ptr<RxRayData> rxRayData = Create<RxRayData>();

  // we directly pass UE coords instead of calculating traceIndex, that is essentially an index for ue walk coords
  // a -> gNB
  // b -> UE
  rxRayData=GetRxRayData(linkData, b->GetPosition(), enbLoc);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  Ptr<NetDevice> txDevice;
  Ptr<NetDevice> rxDevice;

  Ptr<const ThreeGppAntennaArrayModel> txAntennaArray = sAntenna;
  Ptr<const ThreeGppAntennaArrayModel> rxAntennaArray = uAntenna;

  Vector txPos = a->GetPosition();
  Vector rxPos = b->GetPosition();

  //Step 1: The parameters are configured in the example code.
  /*make sure txAngle rxAngle exist, i.e., the position of tx and rx cannot be the same*/

  Angles txAngle (rxPos, txPos);
  Angles rxAngle (txPos, rxPos);

  
  locUT = b->GetPosition ();

  NS_ABORT_MSG_IF (txAntennaArray == 0 || rxAntennaArray == 0, "Cannot create a channel if antenna weights are not set.");
  

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//uint16_t numOfPath = rxRayData->m_path;
  NS_LOG_INFO("m_path="<<rxRayData->m_path);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //Here I specify that I have as many clusters as paths from the raytracing data thus each "cluster" consists of one ray
  uint8_t raysPerCluster = 1;
  Ptr<ThreeGppChannelModel::ThreeGppChannelMatrix> channelParams = Create<ThreeGppChannelModel::ThreeGppChannelMatrix> ();
  //for new channel, the previous and current location is the same.

  channelParams->m_preLocUT = locUT;
  channelParams->m_generatedTime = Now ();

  //Step 4: Generate large scale parameters. All LSPS are uncorrelated.

  /* Notice the shadowing is updated much frequently (every transmission),
   * therefore it is generated separately in the 3GPP propagation loss model.*/

  /* since the scaled Los delays are not to be used in cluster power generation,
   * we will generate cluster power first and resume to compute Los cluster delay later.*/

  //Step 6: Generate cluster powers.
  doubleVector_t clusterPower;
  for (uint16_t cIndex = 0; cIndex < rxRayData->m_path; cIndex++)
    {
//Loading path power from rxRayData
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      clusterPower.push_back (rxRayData->m_pathloss[cIndex]);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    }
//Transforming path power to linear scale
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for (uint16_t cIndex = 0; cIndex < rxRayData->m_path; cIndex++)
    {
      clusterPower.at (cIndex) = std::pow ( 10.0, (clusterPower.at(cIndex)) / 10); //(7.5-6)
    }
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//uint8_t numReducedCluster = clusterPower.size ();
//We don't have to reduce the number
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  uint16_t  numReducedCluster = rxRayData->m_path;
  channelParams->m_numCluster = rxRayData->m_path;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  key_t key = std::make_pair(txDevice,rxDevice);

  if(traceIndex>lastTraceIndex.find(key)->second)
    {
      m_channelMatrixMap.clear();
    }
  //key_t key = std::make_pair (txDevice, rxDevice);
  std::map<key_t, Ptr<ThreeGppChannelModel::ThreeGppChannelMatrix>>::iterator it = m_channelMatrixMap.find (key);
  if (it == m_channelMatrixMap.end ())
    {
      key_t reverse_key = std::make_pair (rxDevice,txDevice);
      Ptr<ThreeGppChannelModel::ThreeGppChannelMatrix> channel = Create<ThreeGppChannelModel::ThreeGppChannelMatrix> ();
      doubleVector_t dopplershift;
      for (unsigned int i = 0; i < channelParams->m_numCluster; i++)
      {
        dopplershift.push_back (m_uniformDist (mt));
      }
      channel->m_doppler = dopplershift;
      m_channelMatrixMap.insert(std::make_pair (key,channel));
      m_channelMatrixMap.insert(std::make_pair (reverse_key,channel));
      it=m_channelMatrixMap.find (key);
    }
  channelParams->m_doppler = it->second->m_doppler;

  //step 7: Generate arrival and departure angles for both azimuth and elevation.

  //According to table 7.5-6, only cluster number equals to 8, 10, 11, 12, 19 and 20 is valid.
  //Not sure why the other cases are in Table 7.5-2.
  NS_LOG_INFO("Number of paths="<<rxRayData->m_path);
  NS_LOG_INFO("Cluster number="<<rxRayData->m_path);

  doubleVector_t clusterAoa, clusterAod, clusterZoa, clusterZod;

//Trying to overwrite the probabilistic angles with ray-tracing data
//Possible a size mismatch between reduced number of clusters and real number of rays
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for(size_t iCluster=0;iCluster<rxRayData->m_path;iCluster++)
  {
    clusterAoa.push_back(rxRayData->m_aoaAzimuth.at(iCluster));
    clusterZoa.push_back(rxRayData->m_aoaElevation.at(iCluster));
    clusterAod.push_back(rxRayData->m_aodAzimuth.at(iCluster));
    clusterZod.push_back(rxRayData->m_aodElevation.at(iCluster));
  }
  NS_LOG_INFO("cluster Angles successful");
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  doubleVector_t attenuation_dB;
  attenuation_dB.push_back (0);

  //Step 9: Generate the cross polarization power ratios
  //This step is skipped, only vertical polarization is considered in this version

  //Step 10: Draw initial phases
  
  MatrixBasedChannelModel::Double2DVector crossPolarizationPowerRatios; // vector containing the cross polarization power ratios, as defined by 7.5-21
  MatrixBasedChannelModel::Double3DVector clusterPhase; //rayAoa_radian[n][m], where n is cluster index, m is ray index
  for (uint8_t nInd = 0; nInd < numReducedCluster; nInd++)
    {
        MatrixBasedChannelModel::DoubleVector temp; // used to store the XPR values
        MatrixBasedChannelModel::Double2DVector temp2; // used to store the PHI values for all the possible combination of polarization
      for (uint8_t mInd = 0; mInd < raysPerCluster; mInd++)
        {
          double uXprLinear = pow (10, table3gpp->m_uXpr / 10); // convert to linear
          double sigXprLinear = pow (10, table3gpp->m_sigXpr / 10); // convert to linear

          temp.push_back (std::pow (10, (m_normalRv->GetValue () * sigXprLinear + uXprLinear) / 10));
          MatrixBasedChannelModel::DoubleVector temp3; // used to store the PHI valuse
          for (uint8_t pInd = 0; pInd < 4; pInd++)
            {
              temp3.push_back (m_uniformRv->GetValue (-1 * M_PI, M_PI));
            }
          temp2.push_back (temp3);
        }
      clusterPhase.push_back (temp2);
    }
  channelParams->m_clusterPhase = clusterPhase;

  //Step 11: Generate channel coefficients for each cluster n and each receiver and transmitter element pair u,s.

  MatrixBasedChannelModel::Complex3DVector H_NLOS; // channel coefficients H_NLOS [u][s][n],
  // where u and s are rem_uniformDistceive and transmit antenna element, n is cluster index.
  size_t uSize = uAntenna->GetNumberOfElements ();
  size_t sSize = sAntenna->GetNumberOfElements ();;

  uint8_t cluster1st = 0, cluster2nd = 0; // first and second strongest cluster;
  double maxPower = 0;
  for (uint8_t cIndex = 0; cIndex < numReducedCluster; cIndex++)
    {
      if (maxPower < clusterPower.at (cIndex))
        {
          maxPower = clusterPower.at (cIndex);
          cluster1st = cIndex;
        }
    }
  maxPower = 0;
  for (uint8_t cIndex = 0; cIndex < numReducedCluster; cIndex++)
    {
      if (maxPower < clusterPower.at (cIndex) && cluster1st != cIndex)
        {
          maxPower = clusterPower.at (cIndex);
          cluster2nd = cIndex;
        }
    }

  NS_LOG_INFO ("1st strongest cluster:" << (int)cluster1st << ", 2nd strongest cluster:" << (int)cluster2nd);

  MatrixBasedChannelModel::Complex3DVector H_usn; //channel coffecient H_usn[u][s][n];
  //Since each of the strongest 2 clusters are divided into 3 sub-clusters, the total cluster will be numReducedCLuster + 4.

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    MatrixBasedChannelModel::Complex2DVector txSpatialMatrix;
    MatrixBasedChannelModel::Complex2DVector rxSpatialMatrix;

  uint8_t txAntennaNum[2];
  uint8_t rxAntennaNum[2];

  txAntennaNum[0] = txAntennaArray->GetNumOfRowElements();
  txAntennaNum[1] = txAntennaArray->GetNumOfColumnElements();

  rxAntennaNum[0] = rxAntennaArray->GetNumOfRowElements();
  rxAntennaNum[1] = rxAntennaArray->GetNumOfColumnElements();

  txSpatialMatrix = GenSpatialMatrix (txAntennaNum, rxRayData, !isTargetEnb);
  rxSpatialMatrix = GenSpatialMatrix (rxAntennaNum, rxRayData, isTargetEnb);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  NS_LOG_INFO("Resize H_usn to Number of reduced Clusters="<<numReducedCluster);
  H_usn.resize (uSize);
  for (size_t uIndex = 0; uIndex < uSize; uIndex++)
    {
      H_usn.at (uIndex).resize (sSize);
      for (size_t sIndex = 0; sIndex < sSize; sIndex++)
        {
          H_usn.at (uIndex).at (sIndex).resize (numReducedCluster);
        }
    }
  NS_LOG_INFO("Resized");
  // The following for loops computes the channel coefficients
  for (size_t uIndex = 0; uIndex < uSize; uIndex++)
    {
      //Vector uLoc = rxAntennaArray->GetAntennaLocation (uIndex);

      for (size_t sIndex = 0; sIndex < sSize; sIndex++)
        {

          //Vector sLoc = txAntennaArray->GetAntennaLocation (sIndex);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          for (uint8_t nIndex = 0; nIndex < rxRayData->m_path; nIndex++)
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            {
              std::complex<double> rays (0,0);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
              NS_LOG_INFO("Computing the channel Matrix");
              rays = std::conj (txSpatialMatrix.at(nIndex).at(sIndex)) * rxSpatialMatrix.at(nIndex).at(uIndex);
              rays *= sqrt(clusterPower.at (nIndex));
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
               H_usn.at (uIndex).at (sIndex).at (nIndex) = rays;
            }
        }
    }
  NS_LOG_INFO("Channel calculated");

  NS_LOG_INFO ("size of coefficient matrix =[" << H_usn.size () << "][" << H_usn.at (0).size () << "][" << H_usn.at (0).at (0).size () << "]");
  //NS_LOG_INFO("Channel Matrix= "<<H_usn);

  //I believe that H_usn is the formula H from the paper
  channelParams->m_channel = H_usn;
  //channelParams->m_delay = clusterDelay;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  if(!(H_usn.at(0).at(0).size()==0 && rxRayData->m_delay.size()==1))
  {
    NS_LOG_INFO("Loading Cluster Delays from Raytrace data, size="<<rxRayData->m_delay.size());
    channelParams->m_delay = rxRayData->m_delay;
  }
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  channelParams->m_angle.clear ();
  channelParams->m_angle.push_back (clusterAoa);
  channelParams->m_angle.push_back (clusterZoa);
  channelParams->m_angle.push_back (clusterAod);
  channelParams->m_angle.push_back (clusterZod);

  channelParams->m_clusterPower = clusterPower;

  return channelParams;
}

Ptr<const MatrixBasedChannelModel::ChannelMatrix>
RemSinrEstimator::GetChannel (Ptr<const LinkData> linkData, 
        Ptr<const MobilityModel> aMob,
        Ptr<const MobilityModel> bMob,
        Ptr<const ThreeGppAntennaArrayModel> aAntenna,
        Ptr<const ThreeGppAntennaArrayModel> bAntenna)
{
  NS_LOG_FUNCTION (this);

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  uint32_t traceIndex = GetTraceIndex ();
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  // Compute the channel key. The key is reciprocal, i.e., key (a, b) = key (b, a)
  uint32_t x1 = std::min (aMob->GetObject<Node> ()->GetId (), bMob->GetObject<Node> ()->GetId ());
  uint32_t x2 = std::max (aMob->GetObject<Node> ()->GetId (), bMob->GetObject<Node> ()->GetId ());
  uint32_t channelId = GetKey (x1, x2);

  // retrieve the channel condition
  // according to main_flexRLM -> InitUmi
  Ptr<const ChannelCondition> condition = m_channelConditionModel->GetChannelCondition (aMob, bMob);
  bool los = (condition->GetLosCondition () == ChannelCondition::LosConditionValue::LOS);
  bool o2i = false; // TODO include the o2i condition in the channel condition model

  // Check if the channel is present in the map and return it, otherwise
  // generate a new channel
  bool update = false;
  bool notFound = false;
  Ptr<ThreeGppChannelModel::ThreeGppChannelMatrix> channelMatrix;
  if (m_channelMap.find (channelId) != m_channelMap.end ())
    {
      // channel matrix present in the map
      NS_LOG_DEBUG ("channel matrix present in the map");
      channelMatrix = m_channelMap[channelId];

      // check if it has to be updated
      update = true;//ChannelMatrixNeedsUpdate (channelMatrix, los);
    }
  else
  {
    NS_LOG_DEBUG ("channel matrix not found");
    notFound = true;
  }

  // test: try to always recompute the channel
  notFound = true;
  update = true;

  // If the channel is not present in the map or if it has to be updated
  // generate a new realization
  if (notFound || update)
    {
      std::cout << "  RemSinrEstimator::GetChannel: recomputing channelmatrix " << std::endl;
      // channel matrix not found or has to be updated, generate a new one
      Angles txAngle (bMob->GetPosition (), aMob->GetPosition ());
      Angles rxAngle (aMob->GetPosition (), bMob->GetPosition ());

      double x = aMob->GetPosition ().x - bMob->GetPosition ().x;
      double y = aMob->GetPosition ().y - bMob->GetPosition ().y;
      double distance2D = sqrt (x * x + y * y);
      std::cout << "  RSE: GeChannel: distance2D: " << distance2D << std::endl;

      // NOTE we assume hUT = min (height(a), height(b)) and
      // hBS = max (height (a), height (b))
      double hUt = std::min (aMob->GetPosition ().z, bMob->GetPosition ().z);
      double hBs = std::max (aMob->GetPosition ().z, bMob->GetPosition ().z);

      // TODO this is not currently used, it is needed for the computation of the
      // additional blockage in case of spatial consistent update
      // I do not know who is the UT, I can use the relative distance between
      // tx and rx instead
      Vector locUt = Vector (0.0, 0.0, 0.0);
      
      //if the channel map is empty, we create a new channel.
      // Step 4-11 are performed in function GetNewChannel()
      NS_LOG_INFO ("Create new channel");
      // insert the newly created channel into the map of forward channels

        if (notFound)
        {
          //if the channel map is empty, we create a new channel.
          // Step 4-11 are performed in function GetNewChannel()
          NS_LOG_INFO ("Create new channel");
          // insert the newcly created channel into the map of forward channels
           
            channelMatrix = GetNewChannel(linkData, aMob, bMob, locUt, los, o2i, aAntenna, bAntenna, rxAngle,txAngle, 
                                            distance2D, hBs, hUt);

          UpdateTraceIndex (aMob, bMob);
          channelMatrix->m_nodeIds = std::make_pair (aMob->GetObject<Node> ()->GetId (), bMob->GetObject<Node> ()->GetId ());
        }
        else  //update because the LOS condition changed or because the channel matrix was deleted for the channel update purposes
        {
            channelMatrix = GetNewChannel(linkData, aMob, bMob, locUt, los, o2i, aAntenna, bAntenna, rxAngle,txAngle, 
                                        distance2D, hBs, hUt);

            UpdateTraceIndex (aMob, bMob);
            channelMatrix->m_nodeIds = std::make_pair (aMob->GetObject<Node> ()->GetId (), bMob->GetObject<Node> ()->GetId ());
        }

        // m_speed this does not seem to be used anywhere. Hard-code to some meaningful numbers
        channelMatrix->m_speed = Vector(1,1,0);

        // store or replace the channel matrix in the channel map
        m_channelMap[channelId] = channelMatrix;

      if (m_firstRun)
      {
        if (m_firstCount < 10) // FIXME: TODO: is this somehow related to numbe of gNBs in the sim?
        {
          if (std::find(m_createdChannels.begin (), m_createdChannels.end (), aMob->GetObject<Node>()->GetId()) == 
              m_createdChannels.end ())
            {
              m_createdChannels.emplace_back (aMob->GetObject<Node>()->GetId());
              m_firstCount++;
            }
        }
        else
        {
          m_beamSweepState = true;
          m_firstRun = false;  
        }
      }
  }
  return channelMatrix;
}

void
RemSinrEstimator::AddDevice (uint32_t nodeNum, Ptr<ThreeGppAntennaArrayModel> a)
{
    NS_ASSERT_MSG (m_deviceAntennaMap.find (nodeNum) == m_deviceAntennaMap.end (), "Device is already present in the map");
    m_deviceAntennaMap.insert (std::make_pair (nodeNum, a));
}

Ptr<SpectrumValue>
RemSinrEstimator::DoCalcRxPowerSpectralDensity (
        Ptr<const LinkData> linkData, 
        Ptr<const SpectrumValue> txPsd,
        Ptr<const MobilityModel> a,
        Ptr<const MobilityModel> b) 
{
  NS_LOG_FUNCTION (this);
  uint32_t aId = a->GetObject<Node> ()->GetId (); // id of the node a
  uint32_t bId = b->GetObject<Node> ()->GetId (); // id of the node b

  uint32_t traceIndex = 0;
  uint16_t imsi;

  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue> (txPsd);

    bool aIsEnb = false;
    bool bIsEnb = false;

    traceIndex = GetTraceIndex ();

    // With this method a eNB is recognized by matching the poisition from the loaded data
    NS_LOG_INFO ("size=" << gnbLocations.size ());
    if (gnbLocations.size () != 0)
    {
        for (unsigned int i = 0; i < gnbLocations.size (); i++)
        {
        NS_LOG_INFO("enb i="<<i<<": x="<<gnbLocations.at(i).x<<" y="<<gnbLocations.at(i).y);
        if (a->GetPosition ().x == gnbLocations.at (i).x && a->GetPosition ().y == gnbLocations.at (i).y)
        {
            aIsEnb = true;
        }
        else if (b->GetPosition ().x == gnbLocations.at(i).x && b->GetPosition ().y == gnbLocations.at(i).y)
        {
            bIsEnb = true;
        }
        }
    }

    if ((aIsEnb && bIsEnb) || (!aIsEnb && !bIsEnb))
    {
        NS_LOG_INFO ("UE<->UE or gNB<->gNB, returning");
        *rxPsd = *rxPsd * 1e-11;
        
        return rxPsd;
    }

    NS_ASSERT (aId != bId);
    NS_ASSERT_MSG (a->GetDistanceFrom (b) > 0.0, "The position of a and b devices cannot be the same");

  // retrieve the antenna of device a
  NS_ASSERT_MSG (m_deviceAntennaMap.find (aId) != m_deviceAntennaMap.end (), "Antenna not found for node " << aId);
  Ptr<const ThreeGppAntennaArrayModel> aAntenna = m_deviceAntennaMap.at (aId);
  // !!!! Wrong antenna fetched here
  NS_LOG_DEBUG ("a node " << a->GetObject<Node> () << " antenna " << aAntenna);

  // retrieve the antenna of the device b
  NS_ASSERT_MSG (m_deviceAntennaMap.find (bId) != m_deviceAntennaMap.end (), "Antenna not found for device " << bId);
  Ptr<const ThreeGppAntennaArrayModel> bAntenna = m_deviceAntennaMap.at (bId);
  NS_LOG_DEBUG ("b node " << bId << " antenna " << bAntenna);

  if (aAntenna->IsOmniTx () || bAntenna->IsOmniTx () )
    {
      NS_LOG_LOGIC ("Omni transmission, do nothing.");
      *rxPsd = *rxPsd * 1e-22;

      return rxPsd;
    }

  Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix;

  if (aIsEnb && !bIsEnb)
  {
    channelMatrix = GetChannel (linkData, a, b, aAntenna, bAntenna);  
  }
  else if (!aIsEnb && bIsEnb) 
  {
    channelMatrix = GetChannel (linkData, a, b, aAntenna, bAntenna);
  }
  else
  {
    NS_ABORT_MSG ("Both node a and b cannot be of the same type");
  }  
  
  if (channelMatrix->m_numCluster == 0)
  {
    NS_LOG_INFO ("No RayTrace data between " << a->GetPosition () << " and " << b->GetPosition ());
    //The ryatracing channel does not have any pathloss, therefore when there is no data we need to supress the ouput signal strength
    *rxPsd = *rxPsd * 1e-22;

    return rxPsd;
  }

  // get the precoding and combining vectors for the currently used REM entry

  //ChangeAntennaConfigurations(m_rxRayData);

  ThreeGppAntennaArrayModel::ComplexVector aW = aAntenna->GetBeamformingVector ();
  ThreeGppAntennaArrayModel::ComplexVector bW = bAntenna->GetBeamformingVector ();

  // retrieve the long term component
  ThreeGppAntennaArrayModel::ComplexVector longTerm;
  longTerm = GetLongTerm (aId, bId, channelMatrix, aW, bW);
  Ptr<SpectrumValue> bfPsd;
  
  // apply the beamforming gain
  bfPsd = CalcBeamformingGain (rxPsd, longTerm, aW, bW, channelMatrix, 1); // 1m/s. not used anyway
  
  Ptr<SpectrumValue> bfGain = Create<SpectrumValue>((*bfPsd) / (*rxPsd));

  return bfPsd;
}

Ptr<SpectrumValue>
RemSinrEstimator::CalcBeamformingGain (Ptr<SpectrumValue> txPsd,
                                                           ThreeGppAntennaArrayModel::ComplexVector longTerm,
                                                           ThreeGppAntennaArrayModel::ComplexVector txW,
                                                           ThreeGppAntennaArrayModel::ComplexVector rxW,
                                                           Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
                                                           double speed)
{
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

double
RemSinrEstimator::GetFrequency () const
{
//   DoubleValue freq;
//   m_channelModel->GetAttribute ("Frequency", freq);
  return m_frequency;
}

ThreeGppAntennaArrayModel::ComplexVector
RemSinrEstimator::GetLongTerm (uint32_t aId, uint32_t bId,
                                                   Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
                                                   const ThreeGppAntennaArrayModel::ComplexVector &aW,
                                                   const ThreeGppAntennaArrayModel::ComplexVector &bW)
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

//   bool update = false; // indicates whether the long term has to be updated
  bool update = true; // indicates whether the long term has to be updated

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

ThreeGppAntennaArrayModel::ComplexVector
RemSinrEstimator::CalcLongTerm (Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
                                                    const ThreeGppAntennaArrayModel::ComplexVector &sW,
                                                    const ThreeGppAntennaArrayModel::ComplexVector &uW)
{
  NS_LOG_FUNCTION (this);

  // !!!!!!!!!!!!!!! sAntenna here is 16 but should be 64!!!!!!!!
  uint16_t sAntenna = static_cast<uint16_t> (sW.size ());
  uint16_t uAntenna = static_cast<uint16_t> (uW.size ());

  NS_LOG_DEBUG ("CalcLongTerm with sAntenna " << sAntenna << " uAntenna " << uAntenna);
  //store the long term part to reduce computation load
  //only the small scale fading needs to be updated if the large scale parameters and antenna weights remain unchanged.
  ThreeGppAntennaArrayModel::ComplexVector longTerm;
  //uint8_t numCluster = static_cast<uint8_t> (params->m_channel[0][0].size ());
  uint8_t numCluster;

  numCluster = params->m_delay.size ();
  

  for (uint8_t cIndex = 0; cIndex < numCluster; cIndex++)
    {
      std::complex<double> txSum (0,0);
      for (uint16_t sIndex = 0; sIndex < sAntenna; sIndex++)
        {
          std::complex<double> rxSum (0,0);
          for (uint16_t uIndex = 0; uIndex < uAntenna; uIndex++)
            {
              rxSum = rxSum + std::conj(uW[uIndex]) * params->m_channel[uIndex][sIndex][cIndex];
            }
          txSum = txSum + sW[sIndex] * rxSum;
        }
      longTerm.push_back (txSum);
    }
  return longTerm;
}

bool
RemSinrEstimator::ChannelMatrixNeedsUpdate (Ptr<const ThreeGppChannelModel::ThreeGppChannelMatrix> channelMatrix, bool los) const
{
    return true;
}

void
RemSinrEstimator::SetChannelConditionModel (Ptr<ChannelConditionModel> model)
{
  m_channelConditionModel = model;
}

Ptr<ChannelConditionModel>
RemSinrEstimator::GetChannelConditionModel ()
{
  return m_channelConditionModel;
}

Ptr<const RemSinrEstimator::ParamsTable>
RemSinrEstimator::GetThreeGppTable (bool los, bool o2i, double hBS, double hUT, double distance2D) const
{
  NS_LOG_FUNCTION (this);

  double fcGHz = m_frequency / 1e9;
  Ptr<ParamsTable> table3gpp = Create<ParamsTable> ();
  // table3gpp includes the following parameters:
  // numOfCluster, raysPerCluster, uLgDS, sigLgDS, uLgASD, sigLgASD,
  // uLgASA, sigLgASA, uLgZSA, sigLgZSA, uLgZSD, sigLgZSD, offsetZOD,
  // cDS, cASD, cASA, cZSA, uK, sigK, rTau, uXpr, sigXpr, shadowingStd

  // In NLOS case, parameter uK and sigK are not used and they are set to 0
  if (m_scenario == "RMa")
    {
      if (los && !o2i)
        {
          // 3GPP mentioned that 3.91 ns should be used when the Cluster DS (cDS)
          // entry is N/A.
          table3gpp->m_numOfCluster = 11;
          table3gpp->m_raysPerCluster = 20;
          table3gpp->m_uLgDS = -7.49;
          table3gpp->m_sigLgDS = 0.55;
          table3gpp->m_uLgASD = 0.90;
          table3gpp->m_sigLgASD = 0.38;
          table3gpp->m_uLgASA = 1.52;
          table3gpp->m_sigLgASA = 0.24;
          table3gpp->m_uLgZSA = 0.47;
          table3gpp->m_sigLgZSA = 0.40;
          table3gpp->m_uLgZSD = 0.34;
          table3gpp->m_sigLgZSD = std::max (-1.0, -0.17 * (distance2D / 1000) - 0.01 * (hUT - 1.5) + 0.22);
          table3gpp->m_offsetZOD = 0;
          table3gpp->m_cDS = 3.91e-9;
          table3gpp->m_cASD = 2;
          table3gpp->m_cASA = 3;
          table3gpp->m_cZSA = 3;
          table3gpp->m_uK = 7;
          table3gpp->m_sigK = 4;
          table3gpp->m_rTau = 3.8;
          table3gpp->m_uXpr = 12;
          table3gpp->m_sigXpr = 4;
          table3gpp->m_perClusterShadowingStd = 3;

          for (uint8_t row = 0; row < 7; row++)
            {
              for (uint8_t column = 0; column < 7; column++)
                {
                  table3gpp->m_sqrtC[row][column] = sqrtC_RMa_LOS[row][column];
                }
            }
        }
      else if (!los && !o2i)
        {
          table3gpp->m_numOfCluster = 10;
          table3gpp->m_raysPerCluster = 20;
          table3gpp->m_uLgDS = -7.43;
          table3gpp->m_sigLgDS = 0.48;
          table3gpp->m_uLgASD = 0.95;
          table3gpp->m_sigLgASD = 0.45;
          table3gpp->m_uLgASA = 1.52;
          table3gpp->m_sigLgASA = 0.13;
          table3gpp->m_uLgZSA = 0.58,
          table3gpp->m_sigLgZSA = 0.37;
          table3gpp->m_uLgZSD = std::max (-1.0, -0.19 * (distance2D / 1000) - 0.01 * (hUT - 1.5) + 0.28);
          table3gpp->m_sigLgZSD = 0.30;
          table3gpp->m_offsetZOD = atan ((35 - 3.5) / distance2D) - atan ((35 - 1.5) / distance2D);
          table3gpp->m_cDS = 3.91e-9;
          table3gpp->m_cASD = 2;
          table3gpp->m_cASA = 3;
          table3gpp->m_cZSA = 3;
          table3gpp->m_uK = 0;
          table3gpp->m_sigK = 0;
          table3gpp->m_rTau = 1.7;
          table3gpp->m_uXpr = 7;
          table3gpp->m_sigXpr = 3;
          table3gpp->m_perClusterShadowingStd = 3;

          for (uint8_t row = 0; row < 6; row++)
            {
              for (uint8_t column = 0; column < 6; column++)
                {
                  table3gpp->m_sqrtC[row][column] = sqrtC_RMa_NLOS[row][column];
                }
            }
        }
      else // o2i
        {
          table3gpp->m_numOfCluster = 10;
          table3gpp->m_raysPerCluster = 20;
          table3gpp->m_uLgDS = -7.47;
          table3gpp->m_sigLgDS = 0.24;
          table3gpp->m_uLgASD = 0.67;
          table3gpp->m_sigLgASD = 0.18;
          table3gpp->m_uLgASA = 1.66;
          table3gpp->m_sigLgASA = 0.21;
          table3gpp->m_uLgZSA = 0.93,
          table3gpp->m_sigLgZSA = 0.22;
          table3gpp->m_uLgZSD = std::max (-1.0, -0.19 * (distance2D / 1000) - 0.01 * (hUT - 1.5) + 0.28);
          table3gpp->m_sigLgZSD = 0.30;
          table3gpp->m_offsetZOD = atan ((35 - 3.5) / distance2D) - atan ((35 - 1.5) / distance2D);
          table3gpp->m_cDS = 3.91e-9;
          table3gpp->m_cASD = 2;
          table3gpp->m_cASA = 3;
          table3gpp->m_cZSA = 3;
          table3gpp->m_uK = 0;
          table3gpp->m_sigK = 0;
          table3gpp->m_rTau = 1.7;
          table3gpp->m_uXpr = 7;
          table3gpp->m_sigXpr = 3;
          table3gpp->m_perClusterShadowingStd = 3;

          for (uint8_t row = 0; row < 6; row++)
            {
              for (uint8_t column = 0; column < 6; column++)
                {
                  table3gpp->m_sqrtC[row][column] = sqrtC_RMa_O2I[row][column];
                }
            }
        }
    }
  else if (m_scenario == "UMa")
    {
      if (los && !o2i)
        {
          table3gpp->m_numOfCluster = 12;
          table3gpp->m_raysPerCluster = 20;
          table3gpp->m_uLgDS = -6.955 - 0.0963 * log10 (fcGHz);
          table3gpp->m_sigLgDS = 0.66;
          table3gpp->m_uLgASD = 1.06 + 0.1114 * log10 (fcGHz);
          table3gpp->m_sigLgASD = 0.28;
          table3gpp->m_uLgASA = 1.81;
          table3gpp->m_sigLgASA = 0.20;
          table3gpp->m_uLgZSA = 0.95;
          table3gpp->m_sigLgZSA = 0.16;
          table3gpp->m_uLgZSD = std::max (-0.5, -2.1 * distance2D / 1000 - 0.01 * (hUT - 1.5) + 0.75);
          table3gpp->m_sigLgZSD = 0.40;
          table3gpp->m_offsetZOD = 0;
          table3gpp->m_cDS = std::max (0.25, -3.4084 * log10 (fcGHz) + 6.5622) * 1e-9;
          table3gpp->m_cASD = 5;
          table3gpp->m_cASA = 11;
          table3gpp->m_cZSA = 7;
          table3gpp->m_uK = 9;
          table3gpp->m_sigK = 3.5;
          table3gpp->m_rTau = 2.5;
          table3gpp->m_uXpr = 8;
          table3gpp->m_sigXpr = 4;
          table3gpp->m_perClusterShadowingStd = 3;

          for (uint8_t row = 0; row < 7; row++)
            {
              for (uint8_t column = 0; column < 7; column++)
                {
                  table3gpp->m_sqrtC[row][column] = sqrtC_UMa_LOS[row][column];
                }
            }
        }
      else
        {
          double uLgZSD = std::max (-0.5, -2.1 * distance2D / 1000 - 0.01 * (hUT - 1.5) + 0.9);

          double afc = 0.208 * log10 (fcGHz) - 0.782;
          double bfc = 25;
          double cfc = -0.13 * log10 (fcGHz) + 2.03;
          double efc = 7.66 * log10 (fcGHz) - 5.96;

          double offsetZOD = efc - std::pow (10, afc * log10 (std::max (bfc,distance2D)) + cfc);

          if (!los && !o2i)
            {
              table3gpp->m_numOfCluster = 20;
              table3gpp->m_raysPerCluster = 20;
              table3gpp->m_uLgDS = -6.28 - 0.204 * log10 (fcGHz);
              table3gpp->m_sigLgDS = 0.39;
              table3gpp->m_uLgASD = 1.5 - 0.1144 * log10 (fcGHz);
              table3gpp->m_sigLgASD = 0.28;
              table3gpp->m_uLgASA = 2.08 - 0.27 * log10 (fcGHz);
              table3gpp->m_sigLgASA = 0.11;
              table3gpp->m_uLgZSA = -0.3236 * log10 (fcGHz) + 1.512;
              table3gpp->m_sigLgZSA = 0.16;
              table3gpp->m_uLgZSD = uLgZSD;
              table3gpp->m_sigLgZSD = 0.49;
              table3gpp->m_offsetZOD = offsetZOD;
              table3gpp->m_cDS = std::max (0.25, -3.4084 * log10 (fcGHz) + 6.5622) * 1e-9;;
              table3gpp->m_cASD = 2;
              table3gpp->m_cASA = 15;
              table3gpp->m_cZSA = 7;
              table3gpp->m_uK = 0;
              table3gpp->m_sigK = 0;
              table3gpp->m_rTau = 2.3;
              table3gpp->m_uXpr = 7;
              table3gpp->m_sigXpr = 3;
              table3gpp->m_perClusterShadowingStd = 3;

              for (uint8_t row = 0; row < 6; row++)
                {
                  for (uint8_t column = 0; column < 6; column++)
                    {
                      table3gpp->m_sqrtC[row][column] = sqrtC_UMa_NLOS[row][column];
                    }
                }
            }
          else //(o2i)
            {
              table3gpp->m_numOfCluster = 12;
              table3gpp->m_raysPerCluster = 20;
              table3gpp->m_uLgDS = -6.62;
              table3gpp->m_sigLgDS = 0.32;
              table3gpp->m_uLgASD = 1.25;
              table3gpp->m_sigLgASD = 0.42;
              table3gpp->m_uLgASA = 1.76;
              table3gpp->m_sigLgASA = 0.16;
              table3gpp->m_uLgZSA = 1.01;
              table3gpp->m_sigLgZSA = 0.43;
              table3gpp->m_uLgZSD = uLgZSD;
              table3gpp->m_sigLgZSD = 0.49;
              table3gpp->m_offsetZOD = offsetZOD;
              table3gpp->m_cDS = 11e-9;
              table3gpp->m_cASD = 5;
              table3gpp->m_cASA = 8;
              table3gpp->m_cZSA = 3;
              table3gpp->m_uK = 0;
              table3gpp->m_sigK = 0;
              table3gpp->m_rTau = 2.2;
              table3gpp->m_uXpr = 9;
              table3gpp->m_sigXpr = 5;
              table3gpp->m_perClusterShadowingStd = 4;

              for (uint8_t row = 0; row < 6; row++)
                {
                  for (uint8_t column = 0; column < 6; column++)
                    {
                      table3gpp->m_sqrtC[row][column] = sqrtC_UMa_O2I[row][column];
                    }
                }

            }

        }

    }
  else if (m_scenario == "UMi-StreetCanyon")
    {
      if (los && !o2i)
        {
          table3gpp->m_numOfCluster = 12;
          table3gpp->m_raysPerCluster = 20;
          table3gpp->m_uLgDS = -0.24 * log10 (1 + fcGHz) - 7.14;
          table3gpp->m_sigLgDS = 0.38;
          table3gpp->m_uLgASD = -0.05 * log10 (1 + fcGHz) + 1.21;
          table3gpp->m_sigLgASD = 0.41;
          table3gpp->m_uLgASA = -0.08 * log10 (1 + fcGHz) + 1.73;
          table3gpp->m_sigLgASA = 0.014 * log10 (1 + fcGHz) + 0.28;
          table3gpp->m_uLgZSA = -0.1 * log10 (1 + fcGHz) + 0.73;
          table3gpp->m_sigLgZSA = -0.04 * log10 (1 + fcGHz) + 0.34;
          table3gpp->m_uLgZSD = std::max (-0.21, -14.8 * distance2D / 1000 + 0.01 * std::abs (hUT - hBS) + 0.83);
          table3gpp->m_sigLgZSD = 0.35;
          table3gpp->m_offsetZOD = 0;
          table3gpp->m_cDS = 5e-9;
          table3gpp->m_cASD = 3;
          table3gpp->m_cASA = 17;
          table3gpp->m_cZSA = 7;
          table3gpp->m_uK = 9;
          table3gpp->m_sigK = 5;
          table3gpp->m_rTau = 3;
          table3gpp->m_uXpr = 9;
          table3gpp->m_sigXpr = 3;
          table3gpp->m_perClusterShadowingStd = 3;

          for (uint8_t row = 0; row < 7; row++)
            {
              for (uint8_t column = 0; column < 7; column++)
                {
                  table3gpp->m_sqrtC[row][column] = sqrtC_UMi_LOS[row][column];
                }
            }
        }
      else
        {
          double uLgZSD = std::max (-0.5, -3.1 * distance2D / 1000 + 0.01 * std::max (hUT - hBS,0.0) + 0.2);
          double offsetZOD = -1 * std::pow (10, -1.5 * log10 (std::max (10.0, distance2D)) + 3.3);
          if (!los && !o2i)
            {
              table3gpp->m_numOfCluster = 19;
              table3gpp->m_raysPerCluster = 20;
              table3gpp->m_uLgDS = -0.24 * log10 (1 + fcGHz) - 6.83;
              table3gpp->m_sigLgDS = 0.16 * log10 (1 + fcGHz) + 0.28;
              table3gpp->m_uLgASD = -0.23 * log10 (1 + fcGHz) + 1.53;
              table3gpp->m_sigLgASD = 0.11 * log10 (1 + fcGHz) + 0.33;
              table3gpp->m_uLgASA = -0.08 * log10 (1 + fcGHz) + 1.81;
              table3gpp->m_sigLgASA = 0.05 * log10 (1 + fcGHz) + 0.3;
              table3gpp->m_uLgZSA = -0.04 * log10 (1 + fcGHz) + 0.92;
              table3gpp->m_sigLgZSA = -0.07 * log10 (1 + fcGHz) + 0.41;
              table3gpp->m_uLgZSD = uLgZSD;
              table3gpp->m_sigLgZSD = 0.35;
              table3gpp->m_offsetZOD = offsetZOD;
              table3gpp->m_cDS = 11e-9;
              table3gpp->m_cASD = 10;
              table3gpp->m_cASA = 22;
              table3gpp->m_cZSA = 7;
              table3gpp->m_uK = 0;
              table3gpp->m_sigK = 0;
              table3gpp->m_rTau = 2.1;
              table3gpp->m_uXpr = 8;
              table3gpp->m_sigXpr = 3;
              table3gpp->m_perClusterShadowingStd = 3;

              for (uint8_t row = 0; row < 6; row++)
                {
                  for (uint8_t column = 0; column < 6; column++)
                    {
                      table3gpp->m_sqrtC[row][column] = sqrtC_UMi_NLOS[row][column];
                    }
                }
            }
          else //(o2i)
            {
              table3gpp->m_numOfCluster = 12;
              table3gpp->m_raysPerCluster = 20;
              table3gpp->m_uLgDS = -6.62;
              table3gpp->m_sigLgDS = 0.32;
              table3gpp->m_uLgASD = 1.25;
              table3gpp->m_sigLgASD = 0.42;
              table3gpp->m_uLgASA = 1.76;
              table3gpp->m_sigLgASA = 0.16;
              table3gpp->m_uLgZSA = 1.01;
              table3gpp->m_sigLgZSA = 0.43;
              table3gpp->m_uLgZSD = uLgZSD;
              table3gpp->m_sigLgZSD = 0.35;
              table3gpp->m_offsetZOD = offsetZOD;
              table3gpp->m_cDS = 11e-9;
              table3gpp->m_cASD = 5;
              table3gpp->m_cASA = 8;
              table3gpp->m_cZSA = 3;
              table3gpp->m_uK = 0;
              table3gpp->m_sigK = 0;
              table3gpp->m_rTau = 2.2;
              table3gpp->m_uXpr = 9;
              table3gpp->m_sigXpr = 5;
              table3gpp->m_perClusterShadowingStd = 4;

              for (uint8_t row = 0; row < 6; row++)
                {
                  for (uint8_t column = 0; column < 6; column++)
                    {
                      table3gpp->m_sqrtC[row][column] = sqrtC_UMi_O2I[row][column];
                    }
                }
            }
        }
    }
  else if (m_scenario == "InH-OfficeMixed"||m_scenario == "InH-OfficeOpen")
    {
      NS_ASSERT_MSG (!o2i, "The indoor scenario does out support outdoor to indoor");
      if (los)
        {
          table3gpp->m_numOfCluster = 15;
          table3gpp->m_raysPerCluster = 20;
          table3gpp->m_uLgDS = -0.01 * log10 (1 + fcGHz) - 7.692;
          table3gpp->m_sigLgDS = 0.18;
          table3gpp->m_uLgASD = 1.60;
          table3gpp->m_sigLgASD = 0.18;
          table3gpp->m_uLgASA = -0.19 * log10 (1 + fcGHz) + 1.781;
          table3gpp->m_sigLgASA = 0.12 * log10 (1 + fcGHz) + 0.119;
          table3gpp->m_uLgZSA = -0.26 * log10 (1 + fcGHz) + 1.44;
          table3gpp->m_sigLgZSA = -0.04 * log10 (1 + fcGHz) + 0.264;
          table3gpp->m_uLgZSD = -1.43 * log10 (1 + fcGHz) + 2.228;
          table3gpp->m_sigLgZSD = 0.13 * log10 (1 + fcGHz) + 0.30;
          table3gpp->m_offsetZOD = 0;
          table3gpp->m_cDS = 3.91e-9;
          table3gpp->m_cASD = 5;
          table3gpp->m_cASA = 8;
          table3gpp->m_cZSA = 9;
          table3gpp->m_uK = 7;
          table3gpp->m_sigK = 4;
          table3gpp->m_rTau = 3.6;
          table3gpp->m_uXpr = 11;
          table3gpp->m_sigXpr = 4;
          table3gpp->m_perClusterShadowingStd = 6;

          for (uint8_t row = 0; row < 7; row++)
            {
              for (uint8_t column = 0; column < 7; column++)
                {
                  table3gpp->m_sqrtC[row][column] = sqrtC_office_LOS[row][column];
                }
            }
        }
      else
        {
          table3gpp->m_numOfCluster = 19;
          table3gpp->m_raysPerCluster = 20;
          table3gpp->m_uLgDS = -0.28 * log10 (1 + fcGHz) - 7.173;
          table3gpp->m_sigLgDS = 0.1 * log10 (1 + fcGHz) + 0.055;
          table3gpp->m_uLgASD = 1.62;
          table3gpp->m_sigLgASD = 0.25;
          table3gpp->m_uLgASA = -0.11 * log10 (1 + fcGHz) + 1.863;
          table3gpp->m_sigLgASA = 0.12 * log10 (1 + fcGHz) + 0.059;
          table3gpp->m_uLgZSA = -0.15 * log10 (1 + fcGHz) + 1.387;
          table3gpp->m_sigLgZSA = -0.09 * log10 (1 + fcGHz) + 0.746;
          table3gpp->m_uLgZSD = 1.08;
          table3gpp->m_sigLgZSD = 0.36;
          table3gpp->m_offsetZOD = 0;
          table3gpp->m_cDS = 3.91e-9;
          table3gpp->m_cASD = 5;
          table3gpp->m_cASA = 11;
          table3gpp->m_cZSA = 9;
          table3gpp->m_uK = 0;
          table3gpp->m_sigK = 0;
          table3gpp->m_rTau = 3;
          table3gpp->m_uXpr = 10;
          table3gpp->m_sigXpr = 4;
          table3gpp->m_perClusterShadowingStd = 3;

          for (uint8_t row = 0; row < 6; row++)
            {
              for (uint8_t column = 0; column < 6; column++)
                {
                  table3gpp->m_sqrtC[row][column] = sqrtC_office_NLOS[row][column];
                }
            }
        }
    }
  else
    {
      NS_FATAL_ERROR ("unkonw scenarios");
    }

  return table3gpp;
}

uint32_t
RemSinrEstimator::GetTraceIndex ()
const
{
  uint32_t traceIndex;
  double time = Simulator::Now ().GetSeconds ();
  
  traceIndex = time /  walkSamplingPeriod;
  if (traceIndex > maxTraceIndexCM)
  {
    traceIndex = maxTraceIndexCM;
  }
  return traceIndex;
}

void
RemSinrEstimator::UpdateTraceIndex (Ptr<const MobilityModel> a, Ptr<const MobilityModel> b)
const
{
  uint32_t traceIndex = GetTraceIndex ();

  Ptr<NetDevice> txDevice;
  Ptr<NetDevice> rxDevice;

  key_t key = std::make_pair (txDevice, rxDevice);
  key_t revKey = std::make_pair (rxDevice, txDevice);

  std::map<key_t, uint32_t>::iterator traceIt = lastTraceIndex.find(key);
  std::map<key_t, uint32_t>::iterator revTraceIt = lastTraceIndex.find (revKey);
  if (traceIt != lastTraceIndex.end() && revTraceIt != lastTraceIndex.end())
  {
    traceIt->second = traceIndex;
    revTraceIt->second = traceIndex;
  }
  else
  {
    lastTraceIndex.insert (std::make_pair(key, traceIndex));
    lastTraceIndex.insert (std::make_pair(revKey, traceIndex));
  }
  
}

MatrixBasedChannelModel::Complex2DVector
RemSinrEstimator::GenSpatialMatrix (uint8_t* antennaNum, Ptr<RxRayData> rxRayData,
                                        bool bs) const
{
    MatrixBasedChannelModel::Complex2DVector spatialMatrix;
  uint16_t pathNum = rxRayData->m_path;

  for (unsigned int pathIndex = 0; pathIndex < pathNum; pathIndex++)
    {
      double azimuthAngle;
      double verticalAngle;
      if (bs)
        {
          azimuthAngle = rxRayData->m_aodAzimuth.at (pathIndex);
          verticalAngle = rxRayData->m_aodElevation.at (pathIndex);
        }
      else
        {
          azimuthAngle = rxRayData->m_aoaAzimuth.at (pathIndex);
          verticalAngle = rxRayData->m_aoaElevation.at (pathIndex);
        }

      verticalAngle = 90.0 - verticalAngle;
      complexVector_t singlePath;
      singlePath =
          GenSinglePath (azimuthAngle * M_PI / 180.0, verticalAngle * M_PI / 180.0, antennaNum);
      spatialMatrix.push_back (singlePath);
    }

  return spatialMatrix;
}

complexVector_t
RemSinrEstimator::GenSinglePath (double hAngle, double vAngle, uint8_t *antennaNum) const
{
  complexVector_t singlePath;
  uint16_t vSize = antennaNum[0];
  uint16_t hSize = antennaNum[1];

  for (int vIndex = 0; vIndex < vSize; vIndex++)
    {
      for (int hIndex = 0; hIndex < hSize; hIndex++)
        {
          double w = (-2.0) * M_PI * (hIndex * m_antennaSeparation * sin (hAngle) * sin(vAngle) + 
                                    vIndex * m_antennaSeparation * cos (vAngle));
          singlePath.push_back (std::complex<double> (cos(w), sin(w)));
        }
    }
  return singlePath;
}


}