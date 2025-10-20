#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/applications-module.h"

using namespace ns3;

int main (int argc, char *argv[])
{
    // Command line arguments for configuration
    CommandLine cmd;
    cmd.Parse (argc, argv);

    // Create an LTE Helper
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();

    // Create an EPC Helper (needed for data connectivity)
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
    lteHelper->SetEpcHelper (epcHelper);

    // Create a PGW (Packet Gateway)
    Ptr<Node> pgw = epcHelper->GetPgwNode ();

    // Create a single eNB and UE
    NodeContainer enbNodes;
    NodeContainer ueNodes;
    enbNodes.Create (1);
    ueNodes.Create (1);

    // Position the eNB and UE
    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (enbNodes);
    mobility.Install (ueNodes);

    // Install LTE Devices
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

    // Install IP stack on the UE
    InternetStackHelper internet;
    internet.Install (ueNodes);

    // Assign IP addresses to UEs
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));

    // Attach the UE to the eNB
    lteHelper->Attach (ueLteDevs.Get (0), enbLteDevs.Get (0));

    // Set the default gateway for the UE
    // create the internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
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


    // Create an application to send a packet from the UE to the remote host
    uint16_t dlPort = 1234;
    ApplicationContainer clientApps, serverApps;

    // Install a server on the remote host (connected to PGW)
    UdpServerHelper udpServer (dlPort);
    serverApps = udpServer.Install (pgw);
    serverApps.Start (Seconds (1.0));
    serverApps.Stop (Seconds (15.0));

    // Install a client on the UE
    UdpClientHelper udpClient (epcHelper->GetUeDefaultGatewayAddress (), dlPort);
    udpClient.SetAttribute ("MaxPackets", UintegerValue (1000000000));

    uint32_t packetSize = 1400; // Packet size in bytes
    DataRate dataRate = DataRate("400Mb/s"); // Requested data rate per user in Mbps
    Time packetInterval (Seconds (packetSize * 8 / static_cast<double> (dataRate.GetBitRate ())));
    udpClient.SetAttribute("Interval", TimeValue(packetInterval));

    udpClient.SetAttribute ("PacketSize", UintegerValue (1024));
    clientApps = udpClient.Install (ueNodes.Get (0));
    clientApps.Start (Seconds (2.0));
    clientApps.Stop (Seconds (10.0));

    // Enable output of the simulation results
    lteHelper->EnableTraces ();

    LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
    LogComponentEnable ("UdpServer", LOG_LEVEL_INFO);
    LogComponentEnable ("LteHelper", LOG_LEVEL_INFO);

    // Run the simulation
    Simulator::Stop (Seconds (20.0));
    Simulator::Run ();
    Simulator::Destroy ();

    return 0;
}

