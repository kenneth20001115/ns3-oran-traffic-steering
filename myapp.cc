#include <ns3/core-module.h>
#include <ns3/internet-module.h>
#include <ns3/lte-module.h>
#include <ns3/mobility-module.h>
#include <ns3/applications-module.h>
#include <ns3/network-module.h>
#include <ns3/oran-module.h>
#include <ns3/config-store.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/three-gpp-channel-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/three-gpp-propagation-loss-model.h>
#include "ns3/point-to-point-module.h"
#include <ns3/flow-monitor-module.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <map>
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NewOranHandoverUsingMylm");

static std::string s_trafficTraceFile = "traffic-trace.tr";
static std::string s_positionTraceFile = "position-trace.tr";
static std::string s_handoverTraceFile = "handover-trace.tr";

// Function that will save the traces of RX'd packets
void
RxTrace(Ptr<const Packet> p, const Address& from, const Address& to)
{
    uint16_t ueId = (InetSocketAddress::ConvertFrom(to).GetPort() / 1000);

    std::ofstream rxOutFile(s_trafficTraceFile, std::ios_base::app);
    rxOutFile << Simulator::Now().GetSeconds() << " " << ueId << " RX " << p->GetSize()
              << std::endl;
}
// Function that will save the traces of TX'd packets
void
TxTrace(Ptr<const Packet> p, const Address& from, const Address& to)
{
    uint16_t ueId = (InetSocketAddress::ConvertFrom(to).GetPort() / 1000);

    std::ofstream rxOutFile(s_trafficTraceFile, std::ios_base::app);
    rxOutFile << Simulator::Now().GetSeconds() << " " << ueId << " TX " << p->GetSize()
              << std::endl;
}
// Tracing rsrp, rsrq, and sinr
void LogRsrpRsrqSinr(Ptr<OutputStreamWrapper> stream, uint16_t rnti, uint16_t cellId, double rsrp, double rsrq, uint8_t sinr) {
    *stream->GetStream() << Simulator::Now().GetSeconds() << "\tRNTI: " << rnti
                         << "\tCell ID: " << cellId
                         << "\tRSRP: " << rsrp << " dBm"
                         << "\tRSRQ: " << rsrq << " dB"
                         << "\tSINR: " << static_cast<int>(sinr) << " dB" << std::endl;
}
 
// Callback function to log positions
void LogPosition(Ptr<OutputStreamWrapper> stream, Ptr<Node> node,  Ptr<const MobilityModel> mobility) {
    Vector pos = mobility->GetPosition();
    *stream->GetStream() << Simulator::Now().GetSeconds() << "\t" << node->GetId()
                         << "\t" << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
}


void LogEnbLoad(Ptr<LteEnbNetDevice> enbDevice) {

    Ptr<LteEnbPhy> enbPhy = enbDevice->GetPhy();
    std::vector<int> prbUse = enbPhy->GetDownlinkSubChannels();

    std::cout << "Time: " << Simulator::Now().GetSeconds() << "s, eNodeB "<< enbDevice->GetCellId()<< " Used RBs: " << prbUse.size() << std::endl;
    
    Simulator::Schedule(Seconds(5.0), &LogEnbLoad, enbDevice);
}

void LogUeLoad(Ptr<LteUeNetDevice> ueDevice) {

    Ptr<LteUePhy> uePhy = ueDevice->GetPhy();
    std::vector<int> subChannels = uePhy->GetSubChannelsForTransmission();
    std::cout << "UE"<<ueDevice->GetImsi()<<" Subchannels for Uplink Transmission: ";
    for (int subChannel : subChannels)
    {
        std::cout << subChannel << " ";
    }
    std::cout << std::endl;
    
    Simulator::Schedule(Seconds(5.0), &LogUeLoad, ueDevice);
}


int handoverTime=0;
void
NotifyHandoverEndOkEnb(uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    handoverTime++;
    std::cout << Simulator::Now().As(Time::S) << " eNB CellId " << cellid
              << ": completed handover of UE with IMSI " << imsi << " RNTI " << rnti << std::endl;
    std::cout<<"handover time: "<<handoverTime<<std::endl;
}

void
ReverseVelocity(NodeContainer nodes, Time interval)
{
    for (uint32_t idx = 0; idx < nodes.GetN(); idx++)
    {
        Ptr<ConstantVelocityMobilityModel> mobility =
            nodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        mobility->SetVelocity(Vector(mobility->GetVelocity().x * -1, mobility->GetVelocity().y * -1, 0));
    }

    Simulator::Schedule(interval, &ReverseVelocity, nodes, interval);
}



void
QueryRcSink(std::string query, std::string args, int rc)
{
    std::cout << Simulator::Now().GetSeconds() << " Query "
              << ((rc == SQLITE_OK || rc == SQLITE_DONE) ? "OK" : "ERROR") << "(" << rc << "): \""
              << query << "\"";

    if (!args.empty())
    {
        std::cout << " (" << args << ")";
    }
    std::cout << std::endl;
}

int
main(int argc, char* argv[])
{
    uint16_t numberOfUes = 5;
    uint16_t numberOfEnbs = 4;
    Time simTime = Seconds(100);
    Time maxWaitTime = Seconds(100); 
    std::string processingDelayRv = "ns3::NormalRandomVariable[Mean=0.005|Variance=0.000031]";
    //double distance = 1375-625; // distance between eNBs
    Time interval = Seconds(50);
    double speed = 5; // speed of the ue
    bool dbLog = false;
    Time lmQueryInterval = Seconds(5);
    std::string dbFileName = "oran-repository.db";
    std::string lateCommandPolicy = "DROP";

    // Command line arguments
    CommandLine cmd(__FILE__);
    cmd.AddValue("db-log", "Enable printing SQL queries results", dbLog);
    cmd.AddValue("max-wait-time", "The maximum amount of time an LM has to run", maxWaitTime);
    cmd.AddValue("processing-delay-rv",
                 "The random variable that represents the LMs processing delay",
                 processingDelayRv);
    cmd.AddValue("lm-query-interval",
                 "The interval at which to query the LM for commands",
                 lmQueryInterval);
    cmd.AddValue("late-command-policy",
                 "The policy to use for handling commands received after the maximum wait time "
                 "(\"DROP\" or \"SAVE\")",
                 lateCommandPolicy);
    cmd.AddValue("sim-time", "The amount of time to simulate", simTime);
    cmd.Parse(argc, argv);

    LogComponentEnable("OranNearRtRic", (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));

    Config::SetDefault("ns3::LteUePhy::EnableRlfDetection",BooleanValue(false));
    /*--- lte and epc helper ---*/
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>(); // create lteHelper
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>(); // create epcHelper
    lteHelper->SetEpcHelper(epcHelper); // connect lte to the evolved packet core, which is the core network
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler"); // Round-robin Frequency-first Mac Scheduler for resource distribution
    lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm"); // disable automatic handover
 
    // Getting the PGW node; it acts as a gateway between LTE and external network, such as- internet.
    Ptr<Node> pgw = epcHelper->GetPgwNode(); // PGW: Packet Data Network Gateway
    // Create a single remote host
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // IP configuration
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(65000));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (25));
    /*---- Creating RAN nodes using NodeContainer ----*/
    NodeContainer ueNodes; 
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    // Install Mobility Model
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    
    positionAlloc->Add(Vector(625, 375, 25));
    positionAlloc->Add(Vector(1375, 375, 25));
    positionAlloc->Add(Vector(625, 1125, 25));
    positionAlloc->Add(Vector(1375, 1125, 25));
    
    //srand(time(NULL));
    srand(21);
        // Coordinates of the middle point between the eNBs, minus the distance covered
        // in half of the interval for switching directions
    for(int i=0;i<numberOfUes;i++){
        int x=rand()%(1000-625+1)+625;
        int y=rand()%(500-375+1)+375;
        //int x=rand()%(1375-625)+625;
        //int y=rand()%(1125-375)+375;
        positionAlloc->Add(Vector(x, y, 1.5));
    }


    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(enbNodes);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);

    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        double x =(double) std::rand() / (RAND_MAX + 1.0);
        Ptr<ConstantVelocityMobilityModel> mobility =
            ueNodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        mobility->SetVelocity(Vector(speed*x, speed*(1-x), 0));
        //mobility->SetVelocity(Vector(0, 0, 0));
    }
    std::cout<<"position complete\n";
    //lteHelper->SetAttribute("PathlossModel",StringValue("ns3::ThreeGppUmiStreetCanyonPropagationLossModel"));
    //lteHelper->SetPathlossModelAttribute("Environment",StringValue("Urban"));
    //Config::SetDefault("ns3::RadioBearerStatsCalculator::EpochDuration",TimeValue(Seconds(1.00)));
    //Config::SetDefault("ns3:LteUePhy::TxPower",DoubleValue(24));
    //Config::SetDefault("ns3:LteUePhy::NoiseFigure",DoubleValue(6));
    //lteHelper->SetAttribute("SpectrumChannel",StringValue("ns3::ThreeGppUmiStreetCanyonPropagationLossModel"));
    //lteHelper->SetPathlossModelType(GetTypeId("ns3::ThreeGppUmiStreetCanyonPropagationLossModel"));
    //lteHelper->SetSpectrumChannelType ("ns3::ThreeGppUmiStreetCanyonChannelConditionModel");
    
    
    // Schedule the first direction switch
    Simulator::Schedule(interval, &ReverseVelocity, ueNodes, interval);

    // Install LTE Devices in eNB and UEs
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);
    
    // Setting TxPower to 30dBm for all eNbs 
    // Assuming enbLteDevs is your NetDeviceContainer for eNodeBs
    for (NetDeviceContainer::Iterator it = enbLteDevs.Begin(); it != enbLteDevs.End(); ++it) {
        Ptr<NetDevice> device = *it;
        Ptr<LteEnbNetDevice> enbLteDevice = device->GetObject<LteEnbNetDevice>();
        if (enbLteDevice) {
            Ptr<LteEnbPhy> enbPhy = enbLteDevice->GetPhy();
            enbPhy->SetTxPower(20); // Set the transmission power to 30 dBm
            enbPhy->SetNoiseFigure(6);
            enbLteDevice->SetDlBandwidth(25);
        }
        //Simulator::Schedule(Seconds(1.0), &LogEnbLoad, enbLteDevice);
    }
    for (NetDeviceContainer::Iterator it = ueLteDevs.Begin(); it != ueLteDevs.End(); ++it) {
        Ptr<NetDevice> device = *it;
        Ptr<LteUeNetDevice> ueLteDevice = device->GetObject<LteUeNetDevice>();
        if (ueLteDevice) {
            Ptr<LteUePhy> uePhy = ueLteDevice->GetPhy();
            uePhy->SetTxPower(24);
            uePhy->SetNoiseFigure(6);
            //Simulator::Schedule(Seconds(1.0), &LogUeLoad, ueLteDevice);
        }
    }
    std::cout<<"enb set complete\n";
    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
    

    // Attach all UEs to the first eNodeB
    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(rand() % numberOfEnbs));
        
    }

    
    // Add X2 interface
    lteHelper->AddX2Interface(enbNodes);
    std::cout<<"x2 complete\n";
    
    // Install and start applications on UEs and remote host
    uint16_t basePort = 1000;
    ApplicationContainer remoteApps;
    ApplicationContainer ueApps;

    Ptr<RandomVariableStream> onTimeRv = CreateObject<UniformRandomVariable>();
    onTimeRv->SetAttribute("Min", DoubleValue(1.0));
    onTimeRv->SetAttribute("Max", DoubleValue(5.0));
    Ptr<RandomVariableStream> offTimeRv = CreateObject<UniformRandomVariable>();
    offTimeRv->SetAttribute("Min", DoubleValue(1.0));
    offTimeRv->SetAttribute("Max", DoubleValue(5.0));

    for (uint16_t i = 0; i < ueNodes.GetN(); i++)
    {
        uint16_t port = basePort * (i + 1);

        PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), port));
        ueApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(i)));
        // Enable the tracing of RX packets
        ueApps.Get(i)->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));

        Ptr<OnOffApplication> streamingServer = CreateObject<OnOffApplication>();
        remoteApps.Add(streamingServer);
        // Attributes
        streamingServer->SetAttribute(
            "Remote",
            AddressValue(InetSocketAddress(ueIpIfaces.GetAddress(i), port)));
        streamingServer->SetAttribute("DataRate", DataRateValue(DataRate("3000000bps")));
        streamingServer->SetAttribute("PacketSize", UintegerValue(1500));
        streamingServer->SetAttribute("OnTime", PointerValue(onTimeRv));
        streamingServer->SetAttribute("OffTime", PointerValue(offTimeRv));

        remoteHost->AddApplication(streamingServer);
        streamingServer->TraceConnectWithoutContext("TxWithAddresses", MakeCallback(&TxTrace));
    }

    // Inidcate when to start streaming
    remoteApps.Start(Seconds(2));
    // Indicate when to stop streaming
    remoteApps.Stop(simTime + Seconds(10));

    // UE applications start listening
    ueApps.Start(Seconds(1));
    // UE applications stop listening
    ueApps.Stop(simTime + Seconds(15));
    
    // ORAN Models -- BEGIN
    Ptr<OranNearRtRic> nearRtRic = nullptr;
    OranE2NodeTerminatorContainer e2NodeTerminatorsEnbs;
    OranE2NodeTerminatorContainer e2NodeTerminatorsUes;
    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();
    //Ptr<OranLm> myLm = CreateObject<MyLm>();
    
    oranHelper->SetAttribute("Verbose", BooleanValue(true));
    oranHelper->SetAttribute("LmQueryInterval", TimeValue(lmQueryInterval));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime",
                             TimeValue(maxWaitTime)); // 0 means wait for all LMs to finish
    oranHelper->SetAttribute("LmQueryLateCommandPolicy", StringValue(lateCommandPolicy));

    // RIC setup
    if (!dbFileName.empty())
    {
        std::remove(dbFileName.c_str());
    }

    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                  "DatabaseFile",
                                  StringValue(dbFileName));
    oranHelper->SetDefaultLogicModule("ns3::MyLm",
                                      "ProcessingDelayRv",
                                      StringValue(processingDelayRv));
    oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");

    nearRtRic = oranHelper->CreateNearRtRic();

    // UE Nodes setup
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteUeCellInfo> lteUeCellInfoReporter =
            CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranReporterLteUeRsrpRsrq> rsrpRsrqReporter = CreateObject<OranReporterLteUeRsrpRsrq>();
        Ptr<OranReporterAppLoss> appLossReporter = CreateObject<OranReporterAppLoss>();
        Ptr<OranE2NodeTerminatorLteUe> lteUeTerminator =
            CreateObject<OranE2NodeTerminatorLteUe>();

        locationReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));

        lteUeCellInfoReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));

        rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
        appLossReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
            remoteApps.Get(idx)->TraceConnectWithoutContext(
                "Tx",
                MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
            ueApps.Get(idx)->TraceConnectWithoutContext(
                "Rx",
                MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));
        for (uint32_t netDevIdx = 0; netDevIdx < ueNodes.Get(idx)->GetNDevices(); netDevIdx++)
        {
        
            Ptr<LteUeNetDevice> lteUeDevice = ueNodes.Get(idx)->GetDevice(netDevIdx)->GetObject<LteUeNetDevice>();
            if (lteUeDevice)
            {
                Ptr<LteUePhy> uePhy = lteUeDevice->GetPhy();
                uePhy->TraceConnectWithoutContext("ReportUeMeasurements", MakeCallback(&ns3::OranReporterLteUeRsrpRsrq::ReportRsrpRsrq, rsrpRsrqReporter));
            }
        }

        lteUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        lteUeTerminator->SetAttribute("RegistrationIntervalRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        lteUeTerminator->SetAttribute("SendIntervalRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        
        lteUeTerminator->AddReporter(locationReporter);
        lteUeTerminator->AddReporter(lteUeCellInfoReporter);
        lteUeTerminator->AddReporter(rsrpRsrqReporter);
        lteUeTerminator->AddReporter(appLossReporter);
        lteUeTerminator->Attach(ueNodes.Get(idx));
        
        Simulator::Schedule(Seconds(1), &OranE2NodeTerminatorLteUe::Activate, lteUeTerminator);
    }
    
    // ENb Nodes setup
    for (uint32_t idx = 0; idx < enbNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterDlScheduling> DlReporter = CreateObject<OranReporterDlScheduling>();
        Ptr<OranE2NodeTerminatorLteEnb> lteEnbTerminator =
            CreateObject<OranE2NodeTerminatorLteEnb>();
   
        locationReporter->SetAttribute("Terminator", PointerValue(lteEnbTerminator));
        locationReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));
        DlReporter->SetAttribute("Terminator", PointerValue(lteEnbTerminator));
        DlReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));
        
        for (uint32_t netDevIdx = 0; netDevIdx < enbNodes.Get(idx)->GetNDevices(); netDevIdx++)
        {
        
            Ptr<LteEnbNetDevice> lteEnbDevice = enbNodes.Get(idx)->GetDevice(netDevIdx)->GetObject<LteEnbNetDevice>();
            if (lteEnbDevice)
            {
                Ptr<LteEnbMac> enbMac = lteEnbDevice->GetMac();
                enbMac->TraceConnectWithoutContext("DlScheduling", MakeCallback(&ns3::OranReporterDlScheduling::ReportDlScheduling, DlReporter));
            }
        }
        
        lteEnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        lteEnbTerminator->SetAttribute("RegistrationIntervalRv",
                                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        lteEnbTerminator->SetAttribute("SendIntervalRv",
                                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));


        lteEnbTerminator->AddReporter(locationReporter);
        lteEnbTerminator->AddReporter(DlReporter);
        lteEnbTerminator->Attach(enbNodes.Get(idx));

    
        Simulator::Schedule(Seconds(1.5), &OranE2NodeTerminatorLteEnb::Activate, lteEnbTerminator);
    }


    // DB logging to the terminal
    if (dbLog)
    {
        nearRtRic->Data()->TraceConnectWithoutContext("QueryRc", MakeCallback(&QueryRcSink));
    }

    // Activate and the components
    Simulator::Schedule(Seconds(1),
                        &OranHelper::ActivateAndStartNearRtRic,
                        oranHelper,
                        nearRtRic);
    Simulator::Schedule(Seconds(1.5),
                        &OranHelper::ActivateE2NodeTerminators,
                        oranHelper,
                        e2NodeTerminatorsEnbs);
    Simulator::Schedule(Seconds(2),
                        &OranHelper::ActivateE2NodeTerminators,
                        oranHelper,
                        e2NodeTerminatorsUes);
    // ORAN Models -- END
    std::cout<<"node set complete\n";
    // Trace the end of handovers
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                                  MakeCallback(&NotifyHandoverEndOkEnb));
    std::cout<<"trace first complete\n";
    // Assuming 'ueNodes' and 'enbNodes' are your NodeContainers
    Ptr<OutputStreamWrapper> mobilityTrace = Create<OutputStreamWrapper>("MobilityTrace.tr", std::ios::out);
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mob = ueNodes.Get(i)->GetObject<MobilityModel>();
        mob->TraceConnectWithoutContext("CourseChange", MakeBoundCallback(&LogPosition, mobilityTrace, ueNodes.Get(i)));
    }
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mob = enbNodes.Get(i)->GetObject<MobilityModel>();
        mob->TraceConnectWithoutContext("CourseChange", MakeBoundCallback(&LogPosition, mobilityTrace, enbNodes.Get(i)));
    }
    std::cout<<"trace 2nd complete\n";
    // Tracing rsrp, rsrq, and sinr while setting up uePhy
    Ptr<OutputStreamWrapper> rsrpSinrTrace = Create<OutputStreamWrapper>("RsrpRsrqSinrTrace.tr", std::ios::out);
    for (NetDeviceContainer::Iterator it = ueLteDevs.Begin(); it != ueLteDevs.End(); ++it)
    {
        Ptr<NetDevice> device = *it;
        Ptr<LteUeNetDevice> lteUeDevice = device->GetObject<LteUeNetDevice>();
        if (lteUeDevice)
        {
            Ptr<LteUePhy> uePhy = lteUeDevice->GetPhy();
            //uePhy->TraceConnectWithoutContext("ReportCurrentCellRsrpSinr", MakeBoundCallback(&LogRsrpRsrqSinr, rsrpSinrTrace));
        }
    }
    std::cout<<"trace complete\n";
    /* Enabling Tracing for the simulation scenario */
    lteHelper->EnablePhyTraces();
    lteHelper->EnableMacTraces();
    lteHelper->EnableRlcTraces();
    lteHelper->EnablePdcpTraces();
    Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats();
    rlcStats->SetAttribute("StartTime",TimeValue(Seconds(0)));
    rlcStats->SetAttribute("EpochDuration",TimeValue(simTime));
    
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> flowMonitor = flowmon.InstallAll();
    std::map<Ipv4Address,int> ipue; 
    Simulator::Stop(simTime);
     for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
    	uint64_t cellid=0;
    	for (uint32_t netDevIdx = 0; netDevIdx < ueNodes.Get(idx)->GetNDevices(); netDevIdx++)
        {
        
            Ptr<LteUeNetDevice> lteUeDevice = ueNodes.Get(idx)->GetDevice(netDevIdx)->GetObject<LteUeNetDevice>();
            if (lteUeDevice)
            {
                Ptr<LteUeRrc> ueRrc = lteUeDevice->GetRrc();
                cellid=ueRrc->GetImsi();
            }
         }
    	Ptr<Ipv4> ipv4 = ueNodes.Get(idx)->GetObject<Ipv4>();
	Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1,0);  // LTE interface
	Ipv4Address ipAddr = iaddr.GetLocal();
	std::cout << "UE "<<cellid<<" IP Address: " << ipAddr << std::endl;
	ipue[ipAddr]=cellid;
    }
    Simulator::Run();
     
    flowMonitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats();
    Ipv4Address source("1.0.0.2");
    double total=0;
    int count=0;
    std::ofstream outfile;
    outfile.open("test.csv",std::ios::app);
    
   
    std::vector<double> bitrate(20);
    for (auto iter = stats.begin(); iter != stats.end(); ++iter) {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);
      if(t.sourceAddress==source){
      count++;
		  std::cout << "Flow ID: " << iter->first << " Src Addr " << t.sourceAddress << " --> Dst Addr " << t.destinationAddress << std::endl;
		  std::cout << "  Tx Bytes: " << iter->second.txBytes << std::endl;
		  std::cout << "  Rx Bytes: " << iter->second.rxBytes << std::endl;
		  std::cout << "  Throughput: " << iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds()) / 1e6 << " Mbps" << std::endl;
		  total+= iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds()) / 1e6;
		  bitrate[ipue[t.destinationAddress]-1]=iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds());
      }
   } 
    for(int i=0;i<bitrate.size();i++){
    	std::cout<<"ue "<<i+1<<" throughput "<<bitrate[i]/ 1e6<<std::endl;
    	outfile<<bitrate[i]/ 1e6<<",";
    }
    std::cout<<"total throughput: "<<total/count<<std::endl;
    outfile<<total/count<<"\n";
    outfile.close();
    Simulator::Destroy();
    
    return 0;
}
/*
// ENb Nodes setup
    for (uint32_t idx = 0; idx < enbNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterUlScheduling> UlReporter = CreateObject<OranReporterUlScheduling>();
        Ptr<OranE2NodeTerminatorLteEnb> lteEnbTerminator =
            CreateObject<OranE2NodeTerminatorLteEnb>();
   
        locationReporter->SetAttribute("Terminator", PointerValue(lteEnbTerminator));
        locationReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));
        UlReporter->SetAttribute("Terminator", PointerValue(lteEnbTerminator));
        UlReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));
        
        for (uint32_t netDevIdx = 0; netDevIdx < enbNodes.Get(idx)->GetNDevices(); netDevIdx++)
        {
        
            Ptr<LteEnbNetDevice> lteEnbDevice = enbNodes.Get(idx)->GetDevice(netDevIdx)->GetObject<LteEnbNetDevice>();
            if (lteEnbDevice)
            {
                Ptr<LteEnbMac> enbMac = lteEnbDevice->GetMac();
                enbMac->TraceConnectWithoutContext("UlScheduling", MakeCallback(&ns3::OranReporterUlScheduling::ReportUlScheduling, UlReporter));
            }
        }
        
        lteEnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        lteEnbTerminator->SetAttribute("RegistrationIntervalRv",
                                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        lteEnbTerminator->SetAttribute("SendIntervalRv",
                                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));


        lteEnbTerminator->AddReporter(locationReporter);
        lteEnbTerminator->AddReporter(UlReporter);
        lteEnbTerminator->Attach(enbNodes.Get(idx));

    }
    */
