/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

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
#include "ns3/flow-monitor-module.h"

// Network Topology
//                                      |------nCsma3
//                                   LAN3 10.1.4.0 
//                                   ================
//                                   |    |    |    |
//                                   n8   n9   n10  n11
//                                   -          r2
//                            point  -
//  nCsma2------|               to   -  10.1.6.0
//   LAN2 10.1.3.0            point  -   
//  ================                 -
//  |    |    |    |    10.1.1.0     -  point-to-point      s1   s0
// n5   n6   n7   n0 -------------- n1 --------------- n2   n3   n4
// r0   r1   s2       point-to-point      10.1.5.0      |    |    |    
//                                                      ===========
//                                                      LAN1 10.1.2.0
//                                                         |------nCsma1

NS_LOG_COMPONENT_DEFINE ("my-tcp");
std::string dir;

using namespace ns3;

Ptr<PacketSink> sink;                           /* Pointer to the packet sink application */
uint64_t lastTotalRx = 0;                     /* The value of the last total received bytes */

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

int
main (int argc, char *argv[])
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
    std::string tcpVariant = "TcpNewReno";             /* TCP variant type. */
    std::string phyRate = "HtMcs7";                    /* Physical layer bitrate. */
    double simulationTime = 30;                        /* Simulation time in seconds. */

    tcpVariant = std::string ("ns3::") + tcpVariant;
    Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TypeId::LookupByName (tcpVariant)));

    uint32_t nCsma1 = 32;
    uint32_t nCsma2 = 32;
    uint32_t nCsma3 = 32;
    int nFlow = 8;
    uint32_t nPPS = 100000;

    CommandLine cmd (__FILE__);
    cmd.AddValue ("nCsma1", "Number of \"extra\" CSMA nodes/devices in LAN 1", nCsma1);
    cmd.AddValue ("nCsma2", "Number of \"extra\" CSMA nodes/devices in LAN 2", nCsma2);
    cmd.AddValue ("nCsma3", "Number of \"extra\" CSMA nodes/devices in LAN 3", nCsma3);
    cmd.AddValue ("nFlow", "Number of flows in the network", nFlow);
    cmd.AddValue ("nPPS", "Number of packets per second", nPPS);

    cmd.Parse (argc,argv);
    nCsma1 = nCsma1 == 0 ? 1 : nCsma1;
    nCsma2 = nCsma2 == 0 ? 1 : nCsma2;
    nCsma3 = nCsma3 == 0 ? 1 : nCsma3;
    nFlow = nFlow == 0 ? 4 : nFlow;
    nPPS = nPPS == 0 ? 10000 : nPPS;

    uint32_t rate = (8*nPPS*payloadSize)/1048576;

    dataRate = std::to_string(rate) + "Mbps";


    Ptr<Node> n0 = CreateObject<Node> ();
    Ptr<Node> n1 = CreateObject<Node> ();
    Ptr<Node> n2 = CreateObject<Node> ();
    Ptr<Node> n8 = CreateObject<Node> ();

    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("1Mbps"));
    pointToPoint.SetChannelAttribute ("Delay", StringValue ("10us"));

    NetDeviceContainer n0n1 = pointToPoint.Install(n0,n1);
    NetDeviceContainer n2n1 = pointToPoint.Install(n2,n1);
    NetDeviceContainer n8n1 = pointToPoint.Install(n8,n1);


    //LAN 1
    NodeContainer csmaNodes1;
    csmaNodes1.Add (n2);
    csmaNodes1.Create (nCsma1);

    CsmaHelper csma1;
    csma1.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
    csma1.SetChannelAttribute ("Delay", StringValue ("10us"));

    NetDeviceContainer csmaDevices1;
    csmaDevices1 = csma1.Install (csmaNodes1);

    //LAN 2
    NodeContainer csmaNodes2;
    csmaNodes2.Add (n0);
    csmaNodes2.Create (nCsma2);

    CsmaHelper csma2;
    csma2.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
    csma2.SetChannelAttribute ("Delay", StringValue ("10us"));

    NetDeviceContainer csmaDevices2;
    csmaDevices2 = csma2.Install (csmaNodes2);

    //LAN 3
    NodeContainer csmaNodes3;
    csmaNodes3.Add (n8);
    csmaNodes3.Create (nCsma3);

    CsmaHelper csma3;
    csma3.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
    csma3.SetChannelAttribute ("Delay", StringValue ("10us"));

    NetDeviceContainer csmaDevices3;
    csmaDevices3 = csma3.Install (csmaNodes3);

    double errorRate = 0.0001;
    uint32_t k;
    Ptr<RateErrorModel> em = CreateObject<RateErrorModel> ();
    em->SetAttribute ("ErrorRate", DoubleValue (errorRate));
    for(k = 0; k < 2; k++){
        n0n1.Get (k)->SetAttribute ("ReceiveErrorModel", PointerValue (em));
        n2n1.Get (k)->SetAttribute ("ReceiveErrorModel", PointerValue (em));
        n8n1.Get (k)->SetAttribute ("ReceiveErrorModel", PointerValue (em));
    }
    
    for(k = 0; k < (nCsma1 + 1); k++){
        csmaDevices1.Get(k)->SetAttribute ("ReceiveErrorModel", PointerValue (em));
    }
    for(k = 0; k < (nCsma2 + 1); k++){
        csmaDevices2.Get(k)->SetAttribute ("ReceiveErrorModel", PointerValue (em));
    }
    for(k = 0; k < (nCsma3 + 1); k++){
        csmaDevices3.Get(k)->SetAttribute ("ReceiveErrorModel", PointerValue (em));
    }

    std::cout<<"--------------------1------------------"<<std::endl;


    // Internet stack
    InternetStackHelper stack;
    stack.InstallAll ();


    Ipv4AddressHelper address;

    address.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ipn0n1;
    ipn0n1 = address.Assign (n0n1);

    address.SetBase ("10.1.5.0", "255.255.255.0");
    Ipv4InterfaceContainer ipn2n1;
    ipn2n1 = address.Assign (n2n1);

    address.SetBase ("10.1.6.0", "255.255.255.0");
    Ipv4InterfaceContainer ipn8n1;
    ipn8n1 = address.Assign (n8n1);

    address.SetBase ("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer csmaInterfaces1;
    csmaInterfaces1 = address.Assign (csmaDevices1);

    address.SetBase ("10.1.3.0", "255.255.255.0");
    Ipv4InterfaceContainer csmaInterfaces2;
    csmaInterfaces2 = address.Assign (csmaDevices2);

    address.SetBase ("10.1.4.0", "255.255.255.0");
    Ipv4InterfaceContainer csmaInterfaces3;
    csmaInterfaces3 = address.Assign (csmaDevices3);

    /* Populate routing table */
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    std::cout<<"flow/4: "<<nFlow/4<< std::endl;
    std::cout<<"Lan 1 " << nCsma1+1<<" Lan 2 "<<nCsma2+1<< std::endl;
    int i;
    uint16_t sinkPort = 8080;
    for( i = 0; i < (nFlow/4); i++ ){
        std::cout<<" i creating flow "<<i<<std::endl;
        sinkPort -= i;

        /* Install TCP Receiver on the access point */
        PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));
        ApplicationContainer sinkApp = sinkHelper.Install (csmaNodes2.Get(nCsma2-i));
        sink = StaticCast<PacketSink> (sinkApp.Get (0));
        std::cout << "Client address: "<< csmaInterfaces2.GetAddress(nCsma2 -i) << std::endl;
        sinkApp.Start (Seconds (0.0));
        sinkApp.Stop (Seconds (simulationTime));


        /* Install TCP/UDP Transmitter on the station */
        Ptr<Socket> ns3TcpSocket = Socket::CreateSocket (csmaNodes1.Get(nCsma1 - i), TcpSocketFactory::GetTypeId ());

        Ptr<MyApp> app = CreateObject<MyApp> ();
        Address sinkAddress = InetSocketAddress (csmaInterfaces2.GetAddress(nCsma2-i), sinkPort);
        app->Setup (ns3TcpSocket, sinkAddress, payloadSize, 100000, DataRate (dataRate));
        csmaNodes1.Get(nCsma1 - i)->AddApplication (app);
        app->SetStartTime (Seconds (1.0));
        app->SetStopTime (Seconds (20.));
        std::cout << "Server address: "<< csmaInterfaces1.GetAddress(nCsma1 - i) << std::endl;

        Simulator::Schedule(Seconds(0.01), &TraceCwnd , ns3TcpSocket, i);
        Simulator::Schedule(Seconds(0.01), &TraceDrop , (csmaDevices2.Get(nCsma2-i)), i);
      
    }
    std::cout<<"--------------------2------------------"<<std::endl;
    std::cout<<"Lan 3 " << nCsma3+1<<" Lan 2 "<<nCsma2+1<< std::endl;
    int x = i;
    sinkPort = 8080;
    for( i = 0; i < (nFlow/4); i++ ){
        std::cout<<" i creating flow "<<i+x<<std::endl;
        sinkPort += i;
        /* Install TCP Receiver on the access point */
        PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));
        ApplicationContainer sinkApp = sinkHelper.Install (csmaNodes3.Get(nCsma3-i));
        std::cout << "Client address: "<< csmaInterfaces3.GetAddress(nCsma3 -i) << std::endl;
        sinkApp.Start (Seconds (0.0));
        sinkApp.Stop (Seconds (simulationTime));


        /* Install TCP/UDP Transmitter on the station */
        Ptr<Socket> ns3TcpSocket = Socket::CreateSocket (csmaNodes2.Get(nCsma2 - i), TcpSocketFactory::GetTypeId ());

        Ptr<MyApp> app = CreateObject<MyApp> ();
        Address sinkAddress = InetSocketAddress (csmaInterfaces3.GetAddress(nCsma3-i), sinkPort);
        app->Setup (ns3TcpSocket, sinkAddress, payloadSize, 100000, DataRate (dataRate));
        csmaNodes2.Get(nCsma2 - i)->AddApplication (app);
        app->SetStartTime (Seconds (1.0));
        app->SetStopTime (Seconds (20.));
        std::cout << "Server address: "<< csmaInterfaces2.GetAddress(nCsma2 - i) << std::endl;

        Simulator::Schedule(Seconds(0.01), &TraceCwnd , ns3TcpSocket, i);
        Simulator::Schedule(Seconds(0.01), &TraceDrop , (csmaDevices3.Get(nCsma3-i)), i);
    }
    std::cout<<"--------------------3------------------"<<std::endl;

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
    Simulator::Schedule (Seconds (0.01), &TraceThroughput,monitor);

    /* Start Simulation */
    Simulator::Stop (Seconds (simulationTime + 1));

    // Create a new directory to store the output of the program
    dir = "Task-A-Wired-results/"+ tcpVariant + "/";
    std::string dirToSave = "mkdir -p " + dir;
    if (system (dirToSave.c_str ()) == -1)
    {
      exit (1);
    }



    Simulator::Run ();
    

    flowmon.SerializeToXmlFile (dir+"my-tcp.flowmonitor", false, false);

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

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();

    for (auto iter = stats.begin (); iter != stats.end (); ++iter) {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first); 
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
      NS_LOG_UNCOND("Packet drop ratio = " << Lost*100.0/Sent << "%");
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

    AvgThroughput = AvgThroughput/j;
    AvgDelay = AvgDelay/ReceivedPackets;
    NS_LOG_UNCOND("--------Total Results of the simulation----------"<<std::endl);
    NS_LOG_UNCOND("Total Sent Packets  = " << SentPackets);
    NS_LOG_UNCOND("Total Received Packets = " << ReceivedPackets);
    NS_LOG_UNCOND("Total Lost Packets = " << LostPackets);
    NS_LOG_UNCOND("Total Drop Packets = " << LostPackets);
    NS_LOG_UNCOND("Packet Drop Ratio = " << ((LostPackets*100.00)/SentPackets)<< "%");
    NS_LOG_UNCOND("Packet Delivery Ratio = " << ((ReceivedPackets*100.00)/SentPackets)<< "%");
    NS_LOG_UNCOND("Network Throughput = " << AvgThroughput<< " Kbps");
    NS_LOG_UNCOND("End To End Delay =" << AvgDelay);
    NS_LOG_UNCOND("Total Flow id = " << j);
    NS_LOG_UNCOND("Error rate = " << errorRate);


    std::ofstream outfile;
    outfile.open(dir + tcpVariant +"-packet-Throughput.txt", std::ios_base::app); // append instead of overwrite
    outfile << nPPS << " "<<AvgThroughput << "\n"; 

    std::ofstream outfile1;
    outfile1.open(dir +  tcpVariant +"-packet-End-to-End-Delay.txt", std::ios_base::app); // append instead of overwrite
    outfile1 << nPPS << " "<<AvgDelay << "\n"; 

    std::ofstream outfile2;
    outfile2.open(dir +  tcpVariant +"-packet-Packet-Drop-Ratio.txt", std::ios_base::app); // append instead of overwrite
    outfile2 << nPPS << " "<<((LostPackets*100.00)/SentPackets) << "\n";

    std::ofstream outfile3;
    outfile3.open(dir +  tcpVariant +"-packet-Packet-Delivery-Ratio.txt", std::ios_base::app); // append instead of overwrite
    outfile3 << nPPS << " "<<((ReceivedPackets*100.00)/SentPackets) << "\n";
    
    
    // std::ofstream outfile;
    // outfile.open(dir + tcpVariant +"-flow-Throughput.txt", std::ios_base::app); // append instead of overwrite
    // outfile << nFlow << " "<<AvgThroughput << "\n"; 

    // std::ofstream outfile1;
    // outfile1.open(dir + tcpVariant +"-flow-End-to-End-Delay.txt", std::ios_base::app); // append instead of overwrite
    // outfile1 << nFlow << " "<<AvgDelay << "\n"; 

    // std::ofstream outfile2;
    // outfile2.open(dir + tcpVariant +"-flow-Packet-Drop-Ratio.txt", std::ios_base::app); // append instead of overwrite
    // outfile2 << nFlow << " "<<((LostPackets*100.00)/SentPackets) << "\n";

    // std::ofstream outfile3;
    // outfile3.open(dir + tcpVariant +"-flow-Packet-Delivery-Ratio.txt", std::ios_base::app); // append instead of overwrite
    // outfile3 << nFlow << " "<<((ReceivedPackets*100.00)/SentPackets) << "\n";

    // std::ofstream outfile;
    // outfile.open(dir +  tcpVariant +"-node-Throughput.txt", std::ios_base::app); // append instead of overwrite
    // outfile << 4 + nCsma1 + nCsma2 + nCsma3 << " "<<AvgThroughput << "\n"; 

    // std::ofstream outfile1;
    // outfile1.open(dir + tcpVariant +"-node-End-to-End-Delay.txt", std::ios_base::app); // append instead of overwrite
    // outfile1 << 4 + nCsma1 + nCsma2 + nCsma3 << " "<<AvgDelay << "\n"; 

    // std::ofstream outfile2;
    // outfile2.open(dir + tcpVariant +"-node-Packet-Drop-Ratio.txt", std::ios_base::app); // append instead of overwrite
    // outfile2 << 4 + nCsma1 + nCsma2 + nCsma3 << " "<<((LostPackets*100.00)/SentPackets) << "\n";

    // std::ofstream outfile3;
    // outfile3.open(dir + tcpVariant +"-node-Packet-Delivery-Ratio.txt", std::ios_base::app); // append instead of overwrite
    // outfile3 << 4 + nCsma1 + nCsma2 + nCsma3 << " "<<((ReceivedPackets*100.00)/SentPackets) << "\n";
    
    Simulator::Destroy ();
    return 0;
}
