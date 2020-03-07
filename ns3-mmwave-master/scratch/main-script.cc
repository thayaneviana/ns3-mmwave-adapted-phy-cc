
/*Author: Thayane Rodrigues Viana <thayaneviana@hotmail.com
* Published paper: https://ieeexplore.ieee.org/abstract/document/8361697 

* This model is a adaptation from a mmwave module for ns-3.
* Original mmwave module for ns-3: https://dl.acm.org/doi/abs/10.1145/2811587.2811619.
* The original model assumes perfect channel conditions for the association mechanism over 
* the control channel, i.e., all control frames during UE connection are always transmitted 
* successfully, no matter the distance between the transmitter and receiver.
* Consequently, because our work focuses on the initial access over the control channel, 
* the ns-3 code was modified to allow the occurrence of errors during node association, using the
* same error models implemented for the data channel.
*/

/* This script contains: 
 * -  01 mmWave cell with 01 UE and 01 eNB
 * -  UE position is static
 * -  control and data packets transmission
 * */
 
 /*Antenna Configuration for Initial Access (PHY-CC):
 * It is necessary to enter on directory /antenna-configuration, 
 * choose the antenna configuration for initial access 
 * (example: Alamouti 2x4), copy the files inside the folder and replace 
 * them on directory src/mmwave/model.
 * */

 /* Information about the results:
  * SINR:
  * UE_0_UL_SINR_CTRL_dB.txt (control packets - uplink)
  * UE_1_DL_SINR_CTRL_dB.txt (control packets - downlink)
  * UE_0_UL_SINR_dB.txt (data packets - uplink)
  * UE_1_DL_SINR_dB.txt (data packets - downlink)
  * 
  * Packet Delivery Ratio (PDR):
  * /results/pdr.txt (control packets)
  * */
  
 

#include "ns3/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
//#include "ns3/gtk-config-store.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/pointer.h"
#include <ns3/buildings-helper.h>
#include "ns3/log.h"
#include <ns3/buildings-module.h>
#include "ns3/radio-environment-map-helper.h"
//#include "ns3/radio-environment-map-helper.h"

#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <string.h>

using namespace ns3;
using namespace std;


NS_LOG_COMPONENT_DEFINE ("mmWave");

double stop = 1;
double warmup = 0.01;
double dist = 70;


void ComputeResults (void);
struct sim_Result
{
  uint64_t sumRxBytesByFlow;
  uint64_t sumRxBytesQuadByFlow;
  uint64_t sumLostPktsByFlow;
  uint64_t sumRxPktsByFlow;
  uint64_t sumTxPktsByFlow;
  uint64_t sumDelayFlow;
  uint64_t nFlows;
  
       /* Throughput Average by Flow (bps) = sumRxBytesByFlow * 8 / (nFlows * time)
       * Throughput Quadratic Average by Flow (bps) = sumRxBytesQuadByFlow * 64 / (nFlows * time * time)
       * Net Aggregated Throughput Average by Node (bps) = sumRxBytesByFlow * 8 / (nodes * time)
       * Fairness = sumRxBytesByFlow^2 / (nFlows * sumRxBytesQuadByFlow)
       * Delay per Packet (seconds/packet) = sumDelayFlow / sumRxPktsByFlow
       * Lost Ratio (%) = 100 * sumLostPktsByFlow / sumTxPktsByFlow
       */ 
  double thrpAvgByFlow;
  double thrpAvgQuadByFlow;
  double thrpVarByFlow;
  double netThrpAvgByNode;
  double fairness;
  double delayByPkt;
  double lostRatio;  
  double pdr;
  
  sim_Result ()
  {
    sumRxBytesByFlow = 0;
    sumRxBytesQuadByFlow = 0;
    sumLostPktsByFlow = 0;
    sumRxPktsByFlow = 0;
    sumTxPktsByFlow = 0;
    sumDelayFlow = 0;
    nFlows = 0;
  }
} data;

int
main (int argc, char *argv[])
{
	//LogComponentEnable("MmWaveChannelMatrix",LOG_LEVEL_DEBUG);
	//LogComponentEnable ("LteUeRrc", LOG_LEVEL_ALL);
	//LogComponentEnable ("LteEnbRrc", LOG_LEVEL_ALL);
	//LogComponentEnable("MmWavePointToPointEpcHelper",LOG_LEVEL_ALL);
	//LogComponentEnable("EpcUeNas",LOG_LEVEL_ALL);
	//LogComponentEnable ("MmWaveSpectrumPhy", LOG_LEVEL_DEBUG);
	//LogComponentEnable ("MmWaveSpectrumPhy", LOG_LEVEL_FUNCTION);
	//LogComponentEnable ("MmWaveUePhy", LOG_LEVEL_DEBUG);
	//LogComponentEnable ("MmWaveUePhy", LOG_LEVEL_DEBUG);
	//LogComponentEnable ("MmWavePhy", LOG_LEVEL_DEBUG);
	//LogComponentEnable ("MmWavePhy", LOG_LEVEL_FUNCTION);
	//LogComponentEnable ("MmWaveEnbPhy", LOG_LEVEL_DEBUG);
	//LogComponentEnable ("MmWaveEnbMac", LOG_LEVEL_DEBUG);
	//LogComponentEnable ("MmWaveRrMacScheduler", LOG_LEVEL_ALL);
	//LogComponentEnable ("MmWaveUeMac", LOG_LEVEL_DEBUG);
	//LogComponentEnable ("MmWaveUeMac", LOG_LEVEL_FUNCTION);
	//LogComponentEnable ("MmWaveChannelMatrix", LOG_LEVEL_FUNCTION);
	//LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
	//LogComponentEnable ("PacketSink", LOG_LEVEL_INFO);
	//LogComponentEnable("PropagationLossModel",LOG_LEVEL_ALL);
	//LogComponentEnable("PropagationLossModel",LOG_LEVEL_DEBUG);
	//LogComponentEnable ("MmWaveBeamforming", LOG_LEVEL_DEBUG);
	//LogComponentEnable ("MmWaveAmc", LOG_LEVEL_LOGIC);
	
	
	//LogComponentEnable ("MmWaveMiErrorModel", LOG_LEVEL_LOGIC);
	//LogComponentEnable ("mmWaveAmc", LOG_LEVEL_DEBUG);
 
   

	uint16_t numEnb = 1;
	uint16_t numUe = 1;
	double simTime = 1;
	double interPacketInterval = 1; //(ms)
	

	// Command line arguments
	CommandLine cmd;
	cmd.AddValue("numEnb", "Number of eNBs", numEnb);
	cmd.AddValue("numUe", "Number of UEs per eNB", numUe);
	cmd.AddValue("simTime", "Total duration of the simulation [s])", simTime);
	cmd.AddValue("interPacketInterval", "Inter packet interval [ms])", interPacketInterval);
	cmd.AddValue("dist", "Distance eNB-node", dist);
	cmd.Parse(argc, argv);


	//The number of TTIs a CQI is valid (default 1000 - 1 sec.)
	//Config::SetDefault ("ns3::MmWaveRrMacScheduler::CqiTimerThreshold", UintegerValue (100));
	

	//The carrier frequency (in Hz) at which propagation occurs  (default is 28 GHz)
     Config::SetDefault ("ns3::MmWavePropagationLossModel::Frequency",  DoubleValue (28e9));
     Config::SetDefault ("ns3::MmWaveEnbPhy::TxPower",  DoubleValue (30));
     Config::SetDefault ("ns3::MmWaveUePhy::TxPower",  DoubleValue (30));
     
     Config::SetDefault ("ns3::MmWaveEnbNetDevice::AntennaNum", UintegerValue(64));//data packets
     Config::SetDefault ("ns3::MmWaveUeNetDevice::AntennaNum", UintegerValue(16));//data packets
     
    //The minimum value (dB) of the total loss, used at short ranges (distance<0)
   // Config::SetDefault ("ns3::MmWavePropagationLossModel::MinLoss",  DoubleValue (20.0));
    
    //Loss (dB) in the Signal-to-Noise-Ratio due to non-idealities in the receiver.
     Config::SetDefault ("ns3::MmWaveEnbPhy::NoiseFigure",  DoubleValue (5.0));

    //The no. of packets received and transmitted by the Base Station
    //Config::SetDefault ("ns3::MmWaveSpectrumPhy::ReportEnbTxRxPacketCount",  MakeTraceSourceAccessor (&MmWaveSpectrumPhy::m_reportEnbPacketCount)); 
   
    //Activate/Deactivate the HARQ [by default is active].
	//Config::SetDefault ("ns3::MmWaveRrMacScheduler::HarqEnabled", BooleanValue(false));
	//A classe MmWavePhyMacCommon tem vários atributos, olhar código da classe
	Config::SetDefault ("ns3::MmWavePhyMacCommon::ResourceBlockNum", UintegerValue(1));
	Config::SetDefault ("ns3::MmWavePhyMacCommon::ChunkPerRB", UintegerValue(72));
	
	Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
	Ptr<MmWavePointToPointEpcHelper>  epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
   
    mmwaveHelper->SetAttribute("PathlossModel", StringValue ("ns3::MmWavePropagationLossModel"));
    // Configure number of antennas
    mmwaveHelper->SetAntenna (16,64);//data packets
	mmwaveHelper->SetEpcHelper (epcHelper);

	ConfigStore inputConfig;
	inputConfig.ConfigureDefaults();

	// parse again so you can override default values from the command line
	cmd.Parse(argc, argv);

	Ptr<Node> pgw = epcHelper->GetPgwNode ();

	// Create a single RemoteHost
	NodeContainer remoteHostContainer;
	remoteHostContainer.Create (1);
	Ptr<Node> remoteHost = remoteHostContainer.Get (0);
	InternetStackHelper internet;
	internet.Install (remoteHostContainer);

	// Create the Internet
	PointToPointHelper p2ph;
	p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
	p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
	p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
	NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
	Ipv4AddressHelper ipv4h;
	ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
	Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
	// interface 0 is localhost, 1 is the p2p device
	//Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

	Ipv4StaticRoutingHelper ipv4RoutingHelper;
	Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
	remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

	NodeContainer ueNodes;
	NodeContainer enbNodes;
	enbNodes.Create(numEnb);
	ueNodes.Create(numUe);
	NodeContainer allNodes;
	
	for (uint16_t i = 0; i < numUe; i++)
  {
	  allNodes.Add(ueNodes.Get(i));
  }

	//allNodes.Add(enbNodes.Get(0));
	allNodes.Add(remoteHostContainer.Get(0));
	

	
	// Install Mobility Model
	Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
	enbPositionAlloc->Add (Vector (0.0, 0.0, 10.0));
	MobilityHelper enbmobility;
	enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	enbmobility.SetPositionAllocator(enbPositionAlloc);
	enbmobility.Install (enbNodes);
	
	

	MobilityHelper uemobility;
	Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator> ();
	
    //Geração de posições aleatórias dentro de um raio máximo
	//Obs: não está funcionando para mais que 4 nós
	/*int n;
	int raio = 100;
	int dist[numUe];
	double x[numUe];
	double y[numUe];
	srand( (unsigned)time(NULL) );
	for (n=0 ; n < numUe; n++) 
	{
		dist[n] = rand() % (raio+1);
		x[n] = rand () % (dist[n]+1);
		y[n] = sqrt (pow(dist[n],2) - pow(x[n],2));
		//std::cout << "Nó=" << n << endl;
		//std::cout << "dist=" << dist[n] << endl;
		//std::cout << "x=" << x[n] << endl;
		//std::cout << "y=" << y[n] << endl;
		uePositionAlloc->Add (Vector (x[n], y[n], 1.5));
		printf("uePositionAlloc->Add (Vector (%f, %f, 1.5)); \n", x[n], y[n]);
		//std::cout << "--------------------------------" << endl;
	}*/
	
	uePositionAlloc->Add (Vector (0.0, dist, 1.5));
	
	//uePositionAlloc->Add (Vector (100.0, 0.0, 1.5));
	//uePositionAlloc->Add (Vector (30.0, 40.0, 1.5));
	//uePositionAlloc->Add (Vector (230.0, 230.0, 1.5));
	//uePositionAlloc->Add (Vector (250.0, 250.0, 1.5));
	//uePositionAlloc->Add (Vector (20.0, 0.0, 1.5));
	//uePositionAlloc->Add (Vector (80.0, 0.0, 1.5));


	
	

	uemobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	uemobility.SetPositionAllocator(uePositionAlloc);
	uemobility.Install (ueNodes);

	
	// Install mmWave Devices to the nodes
	NetDeviceContainer enbmmWaveDevs = mmwaveHelper->InstallEnbDevice (enbNodes);
	NetDeviceContainer uemmWaveDevs = mmwaveHelper->InstallUeDevice (ueNodes);

	// Install the IP stack on the UEs
	internet.Install (ueNodes);
	Ipv4InterfaceContainer ueIpIface;
	ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (uemmWaveDevs));
	// Assign IP address to UEs, and install applications
	for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
	{
		Ptr<Node> ueNode = ueNodes.Get (u);
		// Set the default gateway for the UE
		Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
		ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
	}

	mmwaveHelper->AttachToClosestEnb (uemmWaveDevs, enbmmWaveDevs);
	
    mmwaveHelper->EnableTraces ();
    //mmwaveHelper->EnableDlPhyTrace (); // Uncomment to enable PCAP tracing
    mmwaveHelper->Initialize();
   
	

/*Ptr<RadioEnvironmentMapHelper> remHelper = CreateObject<RadioEnvironmentMapHelper> ();
remHelper->SetAttribute ("ChannelPath", StringValue ("/ChannelList/0"));
remHelper->SetAttribute ("OutputFile", StringValue ("rem.out"));
remHelper->SetAttribute ("XMin", DoubleValue (-400.0));
remHelper->SetAttribute ("XMax", DoubleValue (400.0));
remHelper->SetAttribute ("XRes", UintegerValue (100));
remHelper->SetAttribute ("YMin", DoubleValue (-300.0));
remHelper->SetAttribute ("YMax", DoubleValue (300.0));
remHelper->SetAttribute ("YRes", UintegerValue (75));
remHelper->SetAttribute ("Z", DoubleValue (0.0));
remHelper->SetAttribute ("UseDataChannel", BooleanValue (true));
remHelper->SetAttribute ("RbId", IntegerValue (10));
remHelper->Install ();*/


	// Install and start applications on UEs and remote host
	uint16_t dlPort = 80;
	//uint16_t ulPort = 80;
	ApplicationContainer clientApps;
	ApplicationContainer serverApps;
	for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
	{
		//++ulPort;
		
		PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
		//PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));

		serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get(u)));
		//serverApps.Add (ulPacketSinkHelper.Install (remoteHost));


		UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort);
		dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
		dlClient.SetAttribute ("MaxPackets", UintegerValue(1000000));

		//UdpClientHelper ulClient (remoteHostAddr, ulPort);
		//ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
		//ulClient.SetAttribute ("MaxPackets", UintegerValue(1000000));



		clientApps.Add (dlClient.Install (remoteHost));
		//clientApps.Add (ulClient.Install (ueNodes.Get(u)));
		
	}
	
	
   p2ph.EnablePcapAll("cenario15");
	// FlowMonitor
	
  // Activate a data radio bearer
  /*enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
  EpsBearer bearer (q);
  mmwaveHelper->ActivateDataRadioBearer (uemmWaveDevs, bearer);*/
	
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.Install(allNodes);
  monitor->Start (Seconds (0.0)); // start monitoring after network warm up
  monitor->Stop (Seconds (simTime)); // stop monitoring

  Simulator::Stop(Seconds(simTime));
  Simulator::Run ();

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier());

  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
 

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  {
	  Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
	  if (t.destinationPort == 80) // only http flows
	{
	  std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << "Port:"<< t.destinationPort<<")\n";
	  std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
	  std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
	  std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (simTime - 0.01) / 1024 / 1024  << " Mbps\n";
	  std::cout << "  Packet Delivery Ratio: " << ((double)(i->second.rxPackets)/(double)(i->second.txPackets))*100 << "%\n";
	 
	}
      
     if (t.destinationPort == 80) // only http flows
     {
	data.nFlows++;
	data.sumRxBytesByFlow += i->second.rxBytes; // sum flows
	data.sumRxBytesQuadByFlow += i->second.rxBytes * i->second.rxBytes; // sum flows²
	data.sumDelayFlow += i->second.delaySum.GetInteger (); // sum delays
	data.sumRxPktsByFlow += i->second.rxPackets; // sum rx pkts
	data.sumTxPktsByFlow += i->second.txPackets; // sum tx pkts
	data.sumLostPktsByFlow += i->second.lostPackets; // sum lost pkts
      
		  
	}

	  
	  
  }
	serverApps.Start (Seconds (0.0));
	clientApps.Start (Seconds (0.01));
	
	
	


	



//	Simulator::Stop(Seconds(simTime));
//	Simulator::Run();

	/*GtkConfigStore config;
  config.ConfigureAttributes();*/

	Simulator::Destroy();
	
	ComputeResults ();
	return 0;

}

void 
ComputeResults (void)
{
  double deltaT = (stop - warmup);
    // Throughput Average by Flow (bps)
  data.thrpAvgByFlow = (double) data.sumRxBytesByFlow * 8 / (data.nFlows * deltaT);
  // Throughput Quadratic Average by Flow (bps²)
  data.thrpAvgQuadByFlow = (double) data.sumRxBytesQuadByFlow * 8*8 / (data.nFlows * deltaT*deltaT);
  // Throughput Variance by Flow (bps²)
  data.thrpVarByFlow = data.thrpAvgQuadByFlow - data.thrpAvgByFlow * data.thrpAvgByFlow;
  // Network Aggregated Throughput Average by Node (bps)
  //data.netThrpAvgByNode = (double) data.sumRxBytesByFlow * 8 / (numFluxos * deltaT);
  // Fairness Jain's Index
  data.fairness = (double) data.sumRxBytesByFlow * data.sumRxBytesByFlow / (data.nFlows * data.sumRxBytesQuadByFlow);
  // Delay Mean by Packet (nanoseconds)
  data.delayByPkt = (double) data.sumDelayFlow / data.sumRxPktsByFlow;
  // Lost Ratio (%)
  data.lostRatio = (double) 100 * data.sumLostPktsByFlow / data.sumTxPktsByFlow;
  data.pdr = (double) 100 * data.sumRxPktsByFlow / data.sumTxPktsByFlow;
  

  cout << "======================================================================" << endl
       << "Simulation results:" << endl
       << "Throughput Average by Flow (kbps):\t" << data.thrpAvgByFlow / 1024.0 << endl
       << "Throughput Deviation by Flow (kbps):\t" << sqrt (data.thrpVarByFlow) / 1024.0 << endl
      // << "Network Aggregated Throughput Average by Node (kbps):\t" << data.netThrpAvgByNode / 1024.0 << endl
       << "Fairness Jain's Index:\t" << data.fairness << endl
       << "Delay Mean by Packet (seconds):\t" << data.delayByPkt / 1e9 << endl
       << "Packet Lost Ratio (%):\t" << data.lostRatio << endl 
       << "Packet Delivery Ratio (%):\t" << data.pdr<<endl<< endl<< endl;
       
  cout << "Flows: " << data.nFlows << endl;
  
    
    /*ofstream throughput;
    throughput.open ("Resultados/throughput", ios::app);
    throughput << "\t" << data.nFlows << "\t"<< std::setprecision(4) << data.thrpAvgByFlow / 1024.0 << endl;
    throughput.close ();*/
    
    
    /*ofstream delay;
    delay.open ("Resultados/delay", ios::app);
    delay << "\t" << data.nFlows << "\t" << std::setprecision(4) << data.delayByPkt / 1e6 <<  endl;
    delay.close ();
    
    
    ofstream fairness;
    fairness.open ("Resultados/fairness", ios::app);
    fairness << "\t" << data.nFlows << "\t"  << std::setprecision(4) << data.fairness << endl;
    fairness.close ();*/
    
    ofstream pdr;
    pdr.open ("results/pdr", ios::app);
    pdr << "dist=" << dist << "\t" << std::setprecision(4) << data.pdr << endl;
    pdr.close ();
    
    /*ofstream packets;
    packets.open ("Resultados/packets", ios::app);
    packets << "\t" << data.sumTxPktsByFlow << "\t" << std::setprecision (4) << data.sumRxPktsByFlow << endl;
    packets.close ();*/
    
  
  
}
