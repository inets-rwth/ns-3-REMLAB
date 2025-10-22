#ifndef REM_SINR_ESTIMATOR_H
#define REM_SINR_ESTIMATOR_H

#include "nr-phy.h"
#include "nr-control-messages.h"
#include <ns3/lte-enb-phy-sap.h>
#include <ns3/lte-enb-cphy-sap.h>
#include <ns3/nr-harq-phy.h>
#include <functional>
#include "ns3/ideal-beamforming-helper.h"
#include <ns3/nr-mac-scheduler-ns3.h>
#include <ns3/nr-ue-net-device.h>
#include <ns3/nr-helper.h>
#include <ns3/no-backhaul-epc-helper.h>
#include <ns3/epc-ue-nas.h>

#include <ns3/three-gpp-channel-model.h>
#include <ns3/three-gpp-spectrum-propagation-loss-model.h>
#include <ns3/nr-spectrum-phy.h>
#include <ns3/matrix-based-channel-model.h>
#include <ns3/object-factory.h>

namespace ns3
{

/**
 * This class emulates the PHY layer computations for ray-tracing-based setup
 * to estimate SINR for a given REM entry and each candidate gNB in this REM entry.
 * 
 * The class tries to replicate all calculations that would have been done in the real
 * PHY layer to accurately estimate the SINR if a specific antenna configuration on the 
 * UE and gNB side is chosen.
 * 
 * The class implements:
 *  + REM entry loading similar to ThreeGppChannelModel::GetRxRayData
 *     linkData might be passed directly from the coordinator node to avoid a full REM copy
 *  + creation of the ThreeGppChannelMatrix similar to ThreeGppChannelModel::GetNewChannel
 *  + extension of channelMatrix with additional information similar to ThreeGppChannelModel::GetChannel
 *  + calculation of beamforming gain similar to ThreeGppSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity
 *  + final SINR calculation similar to NrSpectrumPhy::StartRx
 * 
 * Required information:
 *  + MobilityModel of RX and TX
 *  + ThreeGppAntennaArrayModel of RX and TX
 */
class RemSinrEstimator : public Object
{
public:
    static TypeId GetTypeId (void);
    RemSinrEstimator();
    virtual ~RemSinrEstimator();

    void SetGnbLocations(const std::map<uint16_t, Vector>& gnbLocations);

    // Get ray data from a specific gNB towards UE location from a given REM entry
    Ptr<RxRayData> GetRxRayData(Ptr<const LinkData> remEntry, Vector ueCords, Vector enbLocation);
    uint8_t GetIndexForStrongestPath (doubleVector_t pathloss);
    void InitializeAllParameters();
    void ChangeAntennaConfigurations(Ptr<RxRayData> linkData);
    std::map<uint16_t, std::pair<uint16_t, double>> EstimateSinrForUeLocation(Ptr<const LinkData> remEntry, Vector ueCords);

    /**
     * Data structure that stores the parameters of 3GPP TR 38.901, Table 7.5-6,
     * for a certain scenario
     */
    struct ParamsTable : public SimpleRefCount<ParamsTable>
    {
        uint8_t m_numOfCluster = 0;
        uint8_t m_raysPerCluster = 0;
        double m_uLgDS = 0;
        double m_sigLgDS = 0;
        double m_uLgASD = 0;
        double m_sigLgASD = 0;
        double m_uLgASA = 0;
        double m_sigLgASA = 0;
        double m_uLgZSA = 0;
        double m_sigLgZSA = 0;
        double m_uLgZSD = 0;
        double m_sigLgZSD = 0;
        double m_offsetZOD = 0;
        double m_cDS = 0;
        double m_cASD = 0;
        double m_cASA = 0;
        double m_cZSA = 0;
        double m_uK = 0;
        double m_sigK = 0;
        double m_rTau = 0;
        double m_uXpr = 0;
        double m_sigXpr = 0;
        double m_perClusterShadowingStd = 0;

        double m_sqrtC[7][7];
    };

    /**
     * Compute the channel matrix between two devices using the procedure
     * described in 3GPP TR 38.901
     * \param linkData REM entry used for channel calculations
     * \param a mobility model of node a 
     * \param b mobility model of node b
     * \param locUT the location of the UT
     * \param los the LOS/NLOS condition
     * \param o2i whether if it is an outdoor to indoor transmission
     * \param sAntenna the s node antenna array
     * \param uAntenna the u node antenna array
     * \param uAngle the u node angle
     * \param sAngle the s node angle
     * \param dis2D the 2D distance between tx and rx
     * \param hBS the height of the BS
     * \param hUT the height of the UT
     * \return the channel realization
     */
    Ptr<ThreeGppChannelModel::ThreeGppChannelMatrix> GetNewChannel (
        Ptr<const LinkData> linkData,
        Ptr<const MobilityModel> a, Ptr<const MobilityModel> b,
        Vector locUT, bool los, bool o2i,
        Ptr<const ThreeGppAntennaArrayModel> sAntenna,
        Ptr<const ThreeGppAntennaArrayModel> uAntenna,
        Angles &uAngle, Angles &sAngle,
        double dis2D, double hBS, double hUT);

    /**
     * Get the parameters needed to apply the channel generation procedure
     * \param los the LOS/NLOS condition
     * \param o2i whether if it is an outdoor to indoor transmission
     * \param hBS the height of the BS
     * \param hUT the height of the UT
     * \param distance2D the 2D distance between tx and rx
     * \return the parameters table
     */
    Ptr<const ParamsTable> GetThreeGppTable (bool los, bool o2i, double hBS, double hUT, double distance2D) const;

    uint32_t GetTraceIndex() const;
    void UpdateTraceIndex (Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
    MatrixBasedChannelModel::Complex2DVector GenSpatialMatrix (uint8_t* antennaNum, Ptr<RxRayData> rxRayData, bool bs) const;
    complexVector_t GenSinglePath (double hAngle, double vAngle, uint8_t* antennaNum) const;

    /**
     * Looks for the channel matrix associated to the aMob and bMob pair in m_channelMap.
     * If found, it checks if it has to be updated. If not found or if it has to
     * be updated, it generates a new uncorrelated channel matrix using the
     * method GetNewChannel and updates m_channelMap.
     *
     * \param aMob mobility model of the a device
     * \param bMob mobility model of the b device
     * \param aAntenna antenna of the a device
     * \param bAntenna antenna of the b device
     * \return the channel matrix
     */
    Ptr<const ThreeGppChannelModel::ChannelMatrix> GetChannel (
        Ptr<const LinkData> linkData,
        Ptr<const MobilityModel> aMob,
        Ptr<const MobilityModel> bMob,
        Ptr<const ThreeGppAntennaArrayModel> aAntenna,
        Ptr<const ThreeGppAntennaArrayModel> bAntenna);

    /**
     * Check if the channel matrix has to be updated
     * \param channelMatrix channel matrix
     * \param isLos the current los condition
     * \return true if the channel matrix has to be updated, false otherwise
     */
    bool ChannelMatrixNeedsUpdate (Ptr<const ThreeGppChannelModel::ThreeGppChannelMatrix> channelMatrix, bool isLos) const;
    
    /**
     * Set the channel condition model
     * \param a pointer to the ChannelConditionModel object
     */
    void SetChannelConditionModel (Ptr<ChannelConditionModel> model);

    /**
     * Get the associated channel condition model
     * \return a pointer to the ChannelConditionModel object
     */
    Ptr<ChannelConditionModel> GetChannelConditionModel ();

    mutable std::vector<Vector> gnbLocations; // locations of all the gNBs
    // From ThreeGppChannelModel
    double m_frequency = 28e9; // TODO: attribute
    std::string m_scenario = "UMi-StreetCanyon"; // TODO: attribute
    double walkSamplingPeriod = 0.25;
    double m_antennaSeparation = 0.5;
    uint32_t maxTraceIndexCM = 1000;     // maxTraceIndexCM == walk coords size
    mutable std::map<key_t, Ptr<ThreeGppChannelModel::ThreeGppChannelMatrix>> m_channelMatrixMap;
    mutable std::map<key_t, uint32_t> lastTraceIndex;
    Ptr<UniformRandomVariable> m_uniformRv; //!< uniform random variable
    Ptr<NormalRandomVariable> m_normalRv; //!< normal random variable
    Ptr<UniformRandomVariable> m_uniformRvShuffle; //!< uniform random variable used to shuffle array in GetNewChannel
    std::unordered_map<uint32_t, Ptr<ThreeGppChannelModel::ThreeGppChannelMatrix> > m_channelMap; //!< map containing the channel realizations
    Ptr<ChannelConditionModel> m_channelConditionModel; //!< the channel condition model
    bool m_beamSweepState {false};
    bool m_firstRun {true};
    std::vector<uint32_t> m_createdChannels;
    uint8_t m_firstCount {0};

    // =========================================
    // From ThreeGppSpectrumPropagationLossModel
    // =========================================
    /**
     * Data structure that stores the long term component for a tx-rx pair
     */
    struct LongTerm : public SimpleRefCount<LongTerm>
    {
        ThreeGppAntennaArrayModel::ComplexVector m_longTerm; //!< vector containing the long term component for each cluster
        Ptr<const MatrixBasedChannelModel::ChannelMatrix> m_channel; //!< pointer to the channel matrix used to compute the long term
        ThreeGppAntennaArrayModel::ComplexVector m_sW; //!< the beamforming vector for the node s used to compute the long term
        ThreeGppAntennaArrayModel::ComplexVector m_uW; //!< the beamforming vector for the node u used to compute the long term
    };

    std::unordered_map <uint32_t, Ptr<ThreeGppAntennaArrayModel> > m_deviceAntennaMap; //!< map containig the <node, antenna> associations
    mutable std::unordered_map < uint32_t, Ptr<const LongTerm> > m_longTermMap; //!< map containing the long term components

    void AddDevice (uint32_t nodeNum, Ptr<ThreeGppAntennaArrayModel> a);

    /**
     * Looks for the long term component in m_longTermMap. If found, checks
     * whether it has to be updated. If not found or if it has to be updated,
     * calls the method CalcLongTerm to compute it.
     * \param aId id of the first node
     * \param bId id of the second node
     * \param channelMatrix the channel matrix
     * \param aW the beamforming vector of the first device
     * \param bW the beamforming vector of the second device
     * \return vector containing the long term compoenent for each cluster
     */
    ThreeGppAntennaArrayModel::ComplexVector GetLongTerm (uint32_t aId, uint32_t bId,
        Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
        const ThreeGppAntennaArrayModel::ComplexVector &aW,
        const ThreeGppAntennaArrayModel::ComplexVector &bW);

    /**
     * Computes the long term component
     * \param channelMatrix the channel matrix H
     * \param sW the beamforming vector of the s device
     * \param uW the beamforming vector of the u device
     * \return the long term component
     */
    ThreeGppAntennaArrayModel::ComplexVector CalcLongTerm (Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
        const ThreeGppAntennaArrayModel::ComplexVector &sW,
        const ThreeGppAntennaArrayModel::ComplexVector &uW);

    /**
     * Computes the beamforming gain and applies it to the tx PSD
     * \param txPsd the tx PSD
     * \param longTerm the long term component
     * \param params The channel matrix
     * \param speed speed of the moving node
     * \return the rx PSD
     */
    Ptr<SpectrumValue> CalcBeamformingGain (Ptr<SpectrumValue> txPsd,
        ThreeGppAntennaArrayModel::ComplexVector longTerm,
        ThreeGppAntennaArrayModel::ComplexVector txW,
        ThreeGppAntennaArrayModel::ComplexVector rxW,
        Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
        double speed);

    /**
     * Get the operating frequency
     * \return the operating frequency in Hz
     */
    double GetFrequency () const;

    /**
     * \brief Computes the received PSD.
     *
     * This function computes the received PSD by applying the 3GPP fast fading
     * model and the beamforming gain.
     * In particular, it retrieves the matrix representing the channel between
     * node a and node b, computes the corresponding long term component, i.e.,
     * the product between the cluster matrices and the TX and RX beamforming
     * vectors (w_rx^T H^n_ab w_tx), and accounts for the Doppler component and
     * the propagation delay.
     * To reduce the computational load, the long term component associated with
     * a certain channel is cached and recomputed only when the channel realization
     * is updated, or when the beamforming vectors change.
     *
     * \param linkData REM entry for which the RX power estimation should happen
     * \param txPsd tx PSD
     * \param a first node mobility model
     * \param b second node mobility model
     *
     * \return the received PSD
     */
    Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity (
        Ptr<const LinkData> linkData, 
        Ptr<const SpectrumValue> txPsd,
        Ptr<const MobilityModel> a,
        Ptr<const MobilityModel> b);

    ObjectFactory m_gnbNetDeviceFactory;  //!< NetDevice factory for gnb
    ObjectFactory m_ueNetDeviceFactory;   //!< NetDevice factory for ue
    ObjectFactory m_gnbAntennaFactory;    //!< gNB antenna factory
    ObjectFactory m_ueAntennaFactory;     //!< UE antenna factory
    ObjectFactory m_channelConditionModelFactory; //!< Channel condition factory

    Ptr<RxRayData> m_rxRayData; // Currently used REM entry with best path for all relevant gNBs
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    NetDeviceContainer m_gnbDevices;
    NetDeviceContainer m_ueDevices;
    int m_nodeIds = 0;


    Ptr<SpectrumValue> m_noisePsd;

    //==========================================
    // Mapping of AoA and AoD values to sectors
    //==========================================
    struct SectorDegreeIndex
    {
      SectorDegreeIndex(double lb, double ub, uint16_t sI)
        : lowerBoundDeg(lb)
        , upperBoundDeg(ub)
        , sectorIndex(sI)
      {};
      SectorDegreeIndex() = default;
  
      bool operator==(const SectorDegreeIndex& r) const
      {
        return lowerBoundDeg == r.lowerBoundDeg
            && upperBoundDeg == r.upperBoundDeg
            && sectorIndex == r.sectorIndex;
      }
  
      bool operator<(const SectorDegreeIndex& r) const
      {
        return sectorIndex < r.sectorIndex;
      }
  
      double lowerBoundDeg{0.};
      double upperBoundDeg{0.};
      /// Actual sector number for BeamId
      // uint16_t beamSector{0};
      /// Sector index for easier tracing of BeamId offsets
      uint16_t sectorIndex{0};
    };
    // keeps the mapping between range of degrees to sectors
    std::map<SectorDegreeIndex, uint16_t> m_gnbSectorDegreeMap;
    std::map<std::pair<double, double>, double> m_gnbElevationDegreeMap;
    std::map<SectorDegreeIndex, uint16_t> m_ueSectorDegreeMap;
    std::map<std::pair<double, double>, double> m_ueElevationDegreeMap;

    std::pair<BeamId, BeamId> MapAoaAodToSectors(
        const double aoaAzimuth,
        const double aoaElevation,
        const double aodAzimuth,
        const double aodElevation);

    std::map<uint16_t, std::pair<BeamId, BeamId>> m_candidateGnbToBplMap;
    // Resulting estimated SINR values for each gNB as map value. REM index is used as a map key.
    std::map<uint16_t, std::pair<uint16_t, double>> m_gnbToRemIndexAndSnrMap;
    std::map<uint16_t, std::pair<uint16_t, double>> m_gnbToRemIndexAndSnrMapWithInterf;

    //==========================================
    // Antenna Parameters
    //==========================================
    uint8_t ueNumRows = 4;
    uint8_t ueNumColumns = 4;
    uint8_t gNBNumRows = 8;
    uint8_t gNBNumColumns = 8;
};

} // namespace ns3

#endif /* REM_SINR_ESTIMATOR_H */
