#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/tcp-westwood.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/propagation-module.h"
#include <fstream>
#include "ns3/internet-apps-module.h"


NS_LOG_COMPONENT_DEFINE ("my-tcp");
std::string dir;

using namespace ns3;

Ptr<PacketSink> sink;                           /* Pointer to the packet sink application */

class MyApp : public Application
{
public:
  MyApp ();
  virtual ~MyApp ();

  /**
   * Register this type.
   * \return The TypeId.
   */
  static TypeId GetTypeId (void);
  void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate);

private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void ScheduleTx (void);
  void SendPacket (void);

  Ptr<Socket>     m_socket;
  Address         m_peer;
  uint32_t        m_packetSize;
  uint32_t        m_nPackets;
  DataRate        m_dataRate;
  EventId         m_sendEvent;
  bool            m_running;
  uint32_t        m_packetsSent;
};

MyApp::MyApp ()
  : m_socket (0),
    m_peer (),
    m_packetSize (0),
    m_nPackets (0),
    m_dataRate (0),
    m_sendEvent (),
    m_running (false),
    m_packetsSent (0)
{
}

MyApp::~MyApp ()
{
  m_socket = 0;
}

/* static */
TypeId MyApp::GetTypeId (void)
{
  static TypeId tid = TypeId ("MyApp")
    .SetParent<Application> ()
    .SetGroupName ("Tutorial")
    .AddConstructor<MyApp> ()
    ;
  return tid;
}

void
MyApp::Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate)
{
  m_socket = socket;
  m_peer = address;
  m_packetSize = packetSize;
  m_nPackets = nPackets;
  m_dataRate = dataRate;
}

void
MyApp::StartApplication (void)
{
  m_running = true;
  m_packetsSent = 0;
  if (InetSocketAddress::IsMatchingType (m_peer))
    {
      m_socket->Bind ();
    }
  else
    {
      m_socket->Bind6 ();
    }
  m_socket->Connect (m_peer);
  SendPacket ();
}

void
MyApp::StopApplication (void)
{
  m_running = false;

  if (m_sendEvent.IsRunning ())
    {
      Simulator::Cancel (m_sendEvent);
    }

  if (m_socket)
    {
      m_socket->Close ();
    }
}

void
MyApp::SendPacket (void)
{
  Ptr<Packet> packet = Create<Packet> (m_packetSize);
  m_socket->Send (packet);

  if (++m_packetsSent < m_nPackets)
    {
      ScheduleTx ();
    }
}

void
MyApp::ScheduleTx (void)
{
  if (m_running)
    {
      Time tNext (Seconds (m_packetSize * 8 / static_cast<double> (m_dataRate.GetBitRate ())));
      m_sendEvent = Simulator::Schedule (tNext, &MyApp::SendPacket, this);
    }
}


uint32_t prev[] = {0,0,0,0,0,0};
Time prevTime[] = {Seconds(0),Seconds(0),Seconds(0),Seconds(0),Seconds(0),Seconds(0)};

static void
TraceThroughput (Ptr<FlowMonitor> monitor)
{
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
  
  Time curTime = Now ();

  int j = 0;
  for (auto iter = stats.begin (); iter != stats.end (); ++iter) { 
          // classifier returns FiveTuple in correspondance to a flowID
    curTime = Now ();

   // std::ofstream thr (dir + "/throughput-" + std::to_string(j+1)+".dat", std::ios::out | std::ios::app);
   // thr << "flow id: " << iter->first << " " << curTime << " " << 8 * (iter->second.rxBytes - prev[j]) / (1024 * (curTime.GetSeconds () - prevTime[j].GetSeconds ())) << std::endl;
    prevTime[j] = curTime;
    prev[j] = iter->second.rxBytes;
    j = j+1;
  }
  Simulator::Schedule (Seconds (1), &TraceThroughput, monitor);
}

static void
CwndChange (Ptr<OutputStreamWrapper> stream, uint32_t oldCwnd, uint32_t newCwnd)
{
 // NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << "\t" << newCwnd);
  //*stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;
}

void TraceCwnd(Ptr<Socket> Socket, int i){
  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream;
 // stream = asciiTraceHelper.CreateFileStream (dir + "congestion-"+std::to_string(i)+".cwnd");
  Socket->TraceConnectWithoutContext ("CongestionWindow", MakeBoundCallback (&CwndChange, stream));
}

static void RxDrop(Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p) {
  //NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << "\tdrop" );
  //*stream->GetStream () << Simulator::Now ().GetSeconds () << std::endl;
}

void TraceDrop(Ptr<NetDevice> device , int i) {
  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream;
 // stream = asciiTraceHelper.CreateFileStream (dir + "drop-"+std::to_string(i)+".dat");
  device->TraceConnectWithoutContext ("PhyRxDrop", MakeBoundCallback (&RxDrop, stream));
}


int main (int argc, char** argv)
{

     // Naming the output directory using local system time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime (buffer, sizeof (buffer), "%d-%m-%Y-%I-%M-%S", timeinfo);
    std::string currentTime (buffer);

    uint32_t payloadSize = 1472;                       /* Transport layer payload size in bytes. */
    std::string dataRate;                  /* Application layer datarate. */
    std::string tcpVariant = "TcpVegas";             /* TCP variant type. */
    std::string phyRate = "HtMcs7";                    /* Physical layer bitrate. */
    double simulationTime = 30;                        /* Simulation time in seconds. */

    tcpVariant = std::string ("ns3::") + tcpVariant;
    Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TypeId::LookupByName (tcpVariant)));

    Packet::EnablePrinting ();
    
    uint32_t nWsnN = 49;
    int nFlow = 8;
    uint32_t nPPS = 10000;
    int coverage = 1;

    CommandLine cmd (__FILE__);
    cmd.AddValue ("nWsnN", "Number of \"extra\" wifi nodes/devices", nWsnN);
    cmd.Parse (argc, argv);

    uint32_t rate = (8*nPPS*payloadSize)/1048576;

    dataRate = std::to_string(rate) + "Mbps";
    
     NodeContainer wsnNodes;
    wsnNodes.Create (nWsnN);

    NodeContainer wiredNodes;
    wiredNodes.Create (1);
    wiredNodes.Add (wsnNodes.Get (0));

    MobilityHelper mobility;
    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                    "MinX", DoubleValue (0.0),
                                    "MinY", DoubleValue (0.0),
                                    "DeltaX", DoubleValue (10*coverage),
                                    "DeltaY", DoubleValue (10*coverage),
                                    "GridWidth", UintegerValue (4),
                                    "LayoutType", StringValue ("RowFirst"));
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (wsnNodes);

    Config::SetDefault ("ns3::RangePropagationLossModel::MaxRange", DoubleValue (40));
    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
    Ptr<RangePropagationLossModel> propModel = CreateObject<RangePropagationLossModel> ();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
    channel->AddPropagationLossModel (propModel);
    channel->SetPropagationDelayModel (delayModel);

    LrWpanHelper lrWpanHelper;
    lrWpanHelper.SetChannel(channel);
    NetDeviceContainer lrwpanDevices = lrWpanHelper.Install (wsnNodes);

    lrWpanHelper.AssociateToPan (lrwpanDevices, 0);

    InternetStackHelper internetv6;
    internetv6.Install (wsnNodes);
    internetv6.Install (wiredNodes.Get (0));

    SixLowPanHelper sixLowPanHelper;
    NetDeviceContainer sixLowPanDevices = sixLowPanHelper.Install (lrwpanDevices);

    CsmaHelper csmaHelper;
    NetDeviceContainer csmaDevices = csmaHelper.Install (wiredNodes);

    Ipv6AddressHelper ipv6;
    ipv6.SetBase (Ipv6Address ("2001:cafe::"), Ipv6Prefix (64));
    Ipv6InterfaceContainer wiredDeviceInterfaces;
    wiredDeviceInterfaces = ipv6.Assign (csmaDevices);
    wiredDeviceInterfaces.SetForwarding (1, true);
    wiredDeviceInterfaces.SetDefaultRouteInAllNodes (1);

    ipv6.SetBase (Ipv6Address ("2001:f00d::"), Ipv6Prefix (64));
    Ipv6InterfaceContainer wsnDeviceInterfaces;
    wsnDeviceInterfaces = ipv6.Assign (sixLowPanDevices);
    wsnDeviceInterfaces.SetForwarding (0, true);
    wsnDeviceInterfaces.SetDefaultRouteInAllNodes (0);

    for (uint32_t i = 0; i < sixLowPanDevices.GetN (); i++) {
        Ptr<NetDevice> dev = sixLowPanDevices.Get (i);
        dev->SetAttribute ("UseMeshUnder", BooleanValue (true));
        dev->SetAttribute ("MeshUnderRadius", UintegerValue (10));
    }

    int i;
    uint16_t sinkPort = 8080;
    for( i = 0; i < (nFlow/2); i++ ){
        std::cout<<" i creating flow "<<i<<std::endl;
        sinkPort -= i;

        /* Install TCP Receiver on the access point */
        PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",Inet6SocketAddress (Ipv6Address::GetAny (), sinkPort));
        sinkHelper.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
        ApplicationContainer sinkApp = sinkHelper.Install (wiredNodes.Get(0));
        sinkApp.Start (Seconds (0.0));
        sinkApp.Stop (Seconds (simulationTime));

        std::cout<<"--------------------1------------------"<<std::endl;
        /* Install TCP/UDP Transmitter on the station */
        Ptr<Socket> ns3TcpSocket = Socket::CreateSocket (wsnNodes.Get(nWsnN-i-1), TcpSocketFactory::GetTypeId ());
        std::cout<<"--------------------2------------------"<<std::endl;

        Ptr<MyApp> app = CreateObject<MyApp> ();
        Address sinkAddress = Inet6SocketAddress (wiredDeviceInterfaces.GetAddress(0,1), sinkPort) ;
        app->Setup (ns3TcpSocket, sinkAddress, payloadSize, 100000, DataRate (dataRate));
        wsnNodes.Get(nWsnN-i-1)->AddApplication (app);
        app->SetStartTime (Seconds (1.0));
        app->SetStopTime (Seconds (simulationTime - 1));
        std::cout << "Server address: "<< wsnDeviceInterfaces.GetAddress(nWsnN-i-1, 1) << std::endl;

        Simulator::Schedule(Seconds(0.01), &TraceCwnd , ns3TcpSocket, i);
        Simulator::Schedule(Seconds(0.01), &TraceDrop , csmaDevices.Get(0), i);
      
    }


    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
    Simulator::Schedule (Seconds (0.01), &TraceThroughput,monitor);

    std::cout<<"--------------------3------------------"<<std::endl;

    /* Start Simulation */
    Simulator::Stop (Seconds (simulationTime + 1));

    // Create a new directory to store the output of the program
    dir = "Task-A-Wireless-results/"+ tcpVariant + "/";
    std::string dirToSave = "mkdir -p " + dir;
    if (system (dirToSave.c_str ()) == -1)
    {
      exit (1);
    }

    std::cout<<"--------------------4------------------"<<std::endl;

    Simulator::Run ();
    std::cout<<"--------------------5------------------"<<std::endl;

    flowmon.SerializeToXmlFile (dir+"my-tcp.flowmonitor", false, false);

    std::cout<<"--------------------5------------------"<<std::endl;

    // variables for output measurement
    int j=0;
    float AvgThroughput = 0;
    float Throughput = 0;
    uint32_t SentPackets = 0;
    uint32_t ReceivedPackets = 0;
    uint32_t LostPackets = 0;
    uint32_t DropPackets = 0;
    uint32_t Sent = 0;
    uint32_t Received = 0;
    uint32_t Lost = 0;
    uint32_t Drop = 0;
    Time AvgDelay;

    std::cout<<"--------------------6------------------"<<std::endl;

    Ptr<Ipv6FlowClassifier> classifier = DynamicCast<Ipv6FlowClassifier> (flowmon.GetClassifier6());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();

    std::cout<<"--------------------7------------------"<<std::endl;

    for (auto iter = stats.begin (); iter != stats.end (); ++iter) {

      Ipv6FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first); 
      
            // classifier returns FiveTuple in correspondance to a flowID
     
      NS_LOG_UNCOND("----Flow ID = " <<iter->first);
      NS_LOG_UNCOND("Src Addr = " <<t.sourceAddress << " -- Dst Addr = "<< t.destinationAddress);
      Sent = iter->second.txPackets;
      NS_LOG_UNCOND("Sent Packets = " <<Sent);
      Received = iter->second.rxPackets;
      NS_LOG_UNCOND("Received Packets = " <<Received);
      Lost = iter->second.lostPackets;
      NS_LOG_UNCOND("Lost Packets = " <<Lost);
      Drop = Lost;
      NS_LOG_UNCOND("Drop Packets = " <<Drop);
      NS_LOG_UNCOND("Packet delivery ratio = " <<Received*100.0/Sent << "%");
      NS_LOG_UNCOND("Packet drop ratio = " << Drop*100.0/Sent << "%");
      if((iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds()) <= 0)
        Throughput = 0;
      else
        Throughput = iter->second.rxBytes * 8.0/((iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds())*1024);
      NS_LOG_UNCOND("Throughput = " <<Throughput<<"Kbps");

      SentPackets = SentPackets + Sent;
      ReceivedPackets = ReceivedPackets + Received;
      LostPackets = LostPackets + Lost;
      DropPackets = DropPackets + Drop;
      AvgThroughput = AvgThroughput + Throughput;
      AvgDelay = AvgDelay + iter->second.delaySum;

      j = j + 1;

      std::cout<<std::endl;

  }

    AvgThroughput = AvgThroughput/nFlow;
    if(ReceivedPackets > 0)
        AvgDelay = AvgDelay/ReceivedPackets;
    NS_LOG_UNCOND("--------Total Results of the simulation----------"<<std::endl);
    NS_LOG_UNCOND("Total Sent Packets  = " << SentPackets);
    NS_LOG_UNCOND("Total Received Packets = " << ReceivedPackets);
    NS_LOG_UNCOND("Total Lost Packets = " << LostPackets);
    NS_LOG_UNCOND("Total Drop Packets = " << DropPackets);
    NS_LOG_UNCOND("Packet Drop Ratio = " << ((DropPackets*100.00)/SentPackets)<< "%");
    NS_LOG_UNCOND("Packet Delivery Ratio = " << ((ReceivedPackets*100.00)/SentPackets)<< "%");
    NS_LOG_UNCOND("Network Throughput = " << AvgThroughput<< " Kbps");
    NS_LOG_UNCOND("End To End Delay =" << AvgDelay);
    NS_LOG_UNCOND("Total Flow id = " << j);
    NS_LOG_UNCOND("Total Flow given = " << nFlow);

    // std::ofstream outfile;
    // outfile.open(dir + tcpVariant +"-coverage.txt", std::ios_base::app); // append instead of overwrite
    // outfile << 4*10*coverage << " "<<AvgThroughput << " "<<" "<<AvgDelay <<  " "<<((DropPackets*100.00)/SentPackets) << " "<<((ReceivedPackets*100.00)/SentPackets) <<"\n"; 

    // std::ofstream outfile;
    // outfile.open(dir + tcpVariant +"-packet.txt", std::ios_base::app); // append instead of overwrite
    // outfile << nPPS << " "<<AvgThroughput << " "<<" "<<AvgDelay <<  " "<<((DropPackets*100.00)/SentPackets) << " "<<((ReceivedPackets*100.00)/SentPackets) <<"\n"; 

    // std::ofstream outfile1;
    // outfile1.open(dir + tcpVariant +"-flow.txt", std::ios_base::app); // append instead of overwrite
    // outfile1 << nFlow << " "<<AvgThroughput << " "<<" "<<AvgDelay <<  " "<<((DropPackets*100.00)/SentPackets) << " "<<((ReceivedPackets*100.00)/SentPackets) <<"\n"; 

    std::ofstream outfile2;
    outfile2.open(dir + tcpVariant +"-node.txt", std::ios_base::app); // append instead of overwrite
    outfile2 << 1 + nWsnN << " "<<AvgThroughput << " "<<" "<<AvgDelay <<  " "<<((DropPackets*100.00)/SentPackets) << " "<<((ReceivedPackets*100.00)/SentPackets) <<"\n"; 

    
    
    Simulator::Destroy ();
    return 0;

}
