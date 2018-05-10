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
 *
 * Author: Nelson Powell (Adapted from wireless-animation.cc)
 */

/*
 * This scenario uses 30 nodes (10 servers, 20 stations) within an Adhoc WiFi
 * network with OLSR routing. 10 clients (out of the 20 stations) will attempt 
 * to transmit UDP Echo data to 10 distinct servers, while 10 additional
 * stations may act as forwarding elements.  The scenario takes place in a 
 * 1km x 1km square, with random walking movement for all nodes.  Speed of nodes 
 * may vary later on to test the affect of movement on the routing convergence
 * as well as bandwidth utilization due to routing information.
 */

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/csma-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/waypoint-mobility-model.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/basic-energy-source.h"
#include "ns3/simple-device-energy-model.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/olsr-helper.h"
#include "ns3/aggregator-helper.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("OlsrBigBox");

int
main (int argc, char *argv[])
{
   uint32_t nWifi = 20;
   int      nRun = 0;
   int      nSeed = 0;
   int      nIter = 1;
   int      strmCnt = 0;
   int      strmTot = 0;
   double   maxSimulatedTime = 60.0;

   CommandLine cmd;
   cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
   cmd.AddValue ("nRun", "Simulation run number", nRun);
   cmd.AddValue ("nSeed", "Seed value that differs from the default", nSeed);
   cmd.AddValue ("nIter", "Number of iterations to execute", nIter);

   cmd.Parse (argc,argv);

   RngSeedManager::SetSeed(nSeed);
   RngSeedManager::SetRun(nRun);
  
   for (int x = 0; x < nIter; ++x ) {
      /*
       * Create NS-3 Nodes to accumulate net devices
       */
      NodeContainer allNodes;
      NodeContainer wifiStaNodes;
      NodeContainer wifiSrvrNode;
      NodeContainer olsrNodes;
      NodeContainer clientNodes;
      
      // Create the server first so it is MAC 00:00:00:00:00:01
      wifiSrvrNode.Create (10);
      allNodes.Add (wifiSrvrNode);
      // Now create the mobile stations that want to send data to the server
      NS_LOG_DEBUG("create " << nWifi << " client nodes ");
      wifiStaNodes.Create (nWifi);
      allNodes.Add (wifiStaNodes);

      olsrNodes.Add (allNodes);

      // Add nodes from the wifiStaNodes to the client container
      uint32_t j = wifiStaNodes.GetN() / 10;
      NS_ASSERT( j > 0 );
      uint32_t clientCount = 0;
      for ( uint32_t i = 0; i < wifiStaNodes.GetN(); i += j ) {
         clientNodes.Add( wifiStaNodes.Get(i) );
         ++clientCount;

         if ( clientCount == wifiSrvrNode.GetN() )
            break;
      }

      // NOTE: Setup the mobility models and randomness before anything else to avoid 
      // variations in the PRNG streams assigned
      
      /*
       * Setup the mobility model for the nodes
       */
      MobilityHelper mobility;
      mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
                                     "X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"),
                                     "Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]") );
      mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                                 "Mode", StringValue ("Time"),
                                 "Time", StringValue ("120s"),
                                 "Direction", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=6.283184]"),
                                 "Speed", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=12.0]"),
                                 "Bounds", StringValue ("0|300|0|300"));
      
//                                 "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=12.0]"),
      
      mobility.InstallAll ();
      
      strmCnt = mobility.AssignStreams( allNodes, 9000000 );
      NS_LOG_DEBUG(" assigned " << strmCnt << " rnd streams to mobility model");
      strmTot += strmCnt;
      
      /*
       * Setup the WiFi Channel, MAC, and MAC Helper for AD HOC Mode
       */
      YansWifiChannelHelper chnlHlpr = YansWifiChannelHelper::Default ();
      Ptr<YansWifiChannel> channel = chnlHlpr.Create ();
      strmCnt = chnlHlpr.AssignStreams( channel, 123456789);
      NS_LOG_DEBUG(" assigned " << strmCnt << " rnd streams to Yans Wifi Channel");
      strmTot += strmCnt;

      YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
      phy.SetChannel ( channel );
      phy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
      WifiHelper wifi;
      wifi.SetRemoteStationManager ("ns3::AarfWifiManager");
      WifiMacHelper mac;
      mac.SetType ("ns3::AdhocWifiMac");
      NetDeviceContainer srvrDevice;
      srvrDevice = wifi.Install (phy, mac, wifiSrvrNode);
      NetDeviceContainer staDevices;
      staDevices = wifi.Install (phy, mac, wifiStaNodes);
      
      strmCnt = wifi.AssignStreams( srvrDevice, 2543210 );
      NS_LOG_DEBUG(" assigned " << strmCnt << " rnd streams to server Wifi devices");
      strmTot += strmCnt;

      strmCnt = wifi.AssignStreams( staDevices, 2643210 );
      NS_LOG_DEBUG(" assigned " << strmCnt << " rnd streams to station Wifi devices");
      strmTot += strmCnt;

      /*
       * Setup the WiFi energy model
       */
      Ptr<BasicEnergySource> energySource = CreateObject<BasicEnergySource>();
      Ptr<SimpleDeviceEnergyModel> energyModel = CreateObject<SimpleDeviceEnergyModel>();
      energySource->SetInitialEnergy (300);
      energyModel->SetEnergySource (energySource);
      energySource->AppendDeviceEnergyModel (energyModel);
      energyModel->SetCurrentA (20);
      // aggregate energy source to node
      //  wifiApNode.Get (0)->AggregateObject (energySource);

      /*
       * Configure the OLSR engine
       */
      OlsrHelper olsr;                          // must assign fixed random streams after installation
      Ipv4StaticRoutingHelper staticRouting;
      Ipv4ListRoutingHelper list;
      list.Add (staticRouting, 0);
      list.Add (olsr, 10);

      InternetStackHelper internet_olsr;
      internet_olsr.SetRoutingHelper (list); // has effect on the next Install ()
      internet_olsr.Install (olsrNodes);
      strmCnt = internet_olsr.AssignStreams( olsrNodes, 2043210 );
      NS_LOG_DEBUG(" assigned " << strmCnt << " rnd streams to Internet Stack Helper");
      strmTot += strmCnt;
      
      strmCnt = olsr.AssignStreams( olsrNodes, 101234 );
      NS_LOG_DEBUG(" assigned " << strmCnt << " rnd streams to Olsr protocol");
      strmTot += strmCnt;

      /*
       * Create the internet stack
       */
      //  InternetStackHelper stack;
      //  stack.Install (allNodes);

      // Install Ipv4 addresses
      Ipv4AddressHelper address;
      address.SetBase ("10.1.1.0", "255.255.255.0");
      Ipv4InterfaceContainer srvrInterface;
      srvrInterface = address.Assign (srvrDevice);
      Ipv4InterfaceContainer staInterfaces;
      staInterfaces = address.Assign (staDevices);

      // Install applications
      UdpEchoServerHelper echoServer(9);
      ApplicationContainer serverApps = echoServer.Install( wifiSrvrNode );;
      serverApps.Start (Seconds (1.0));
      serverApps.Stop (Seconds (maxSimulatedTime));

      ApplicationContainer clientApps;
      for ( uint32_t i = 1; i < clientNodes.GetN(); ++i ) {
         UdpEchoClientHelper echoClient (srvrInterface.GetAddress (i), 9);
         echoClient.SetAttribute ("MaxPackets", UintegerValue (100000));
         /*
          * NOTE: 1200B @ 20ms ~ 480Kbps DR
          */
         echoClient.SetAttribute ("Interval", TimeValue (Seconds (0.02)));
         echoClient.SetAttribute ("PacketSize", UintegerValue (1200));
         ApplicationContainer nxtClient = echoClient.Install( clientNodes.Get(i) );
         clientApps.Add( nxtClient );
      }
      clientApps.Start (Seconds (10.0));
      clientApps.Stop (Seconds (maxSimulatedTime));

      //  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
      Simulator::Stop (Seconds (maxSimulatedTime));

#ifdef RECORD_ANIMATION
      char animName[250];
      sprintf( animName, "olsr-big-box-animation-%d.xml", x);
      AnimationInterface anim ( animName ); // Mandatory
      for (uint32_t i = 0; i < wifiStaNodes.GetN (); ++i)
      {
         anim.UpdateNodeDescription (wifiStaNodes.Get (i), "STA"); // Optional
         anim.UpdateNodeColor (wifiStaNodes.Get (i), 255, 0, 0); // Optional
      }
      for (uint32_t i = 0; i < wifiSrvrNode.GetN (); ++i)
      {
         anim.UpdateNodeDescription (wifiSrvrNode.Get (i), "SRVR"); // Optional
         anim.UpdateNodeColor (wifiSrvrNode.Get (i), 0, 255, 0); // Optional
      }

      anim.EnablePacketMetadata (); // Optional
      anim.EnableIpv4RouteTracking ("routingtable-wireless.xml", Seconds (0), Seconds (45), Seconds (0.25)); //Optional
      anim.EnableWifiMacCounters (Seconds (0), Seconds (10)); //Optional
      anim.EnableWifiPhyCounters (Seconds (0), Seconds (10)); //Optional
      anim.SetMaxPktsPerTraceFile(100000000);
      anim.SetMobilityPollInterval (Seconds (1));

      // Setup tracing in PCAP
      //  phy.EnablePcap ("olsr-big-box", staDevices);
      //  phy.EnablePcap ("olsr-big-box", srvrDevice);
#endif
      
      // Setup the data aggregator to collect packet and frame statistics
      AggregatorHelper aggHelper;
      Ptr<Aggregator>  aggregator = aggHelper.GetAggregator();

      // Add all nodes to the packet aggregator
      aggHelper.Install( allNodes );
      // Now add the Udp Echo client server applications
      aggHelper.InstallUdpClient( clientApps );
      aggHelper.InstallUdpServer( serverApps );

      // Kick off the simulator
      Simulator::Run ();
      Simulator::Destroy ();

      // Now generate the output CSV file for plotting
      // NOTE: Adjust the third argument for scalling factor - timeline ticks in seconds
      char filename[250];
      sprintf(filename, "olsr-bigbox-data_%d.csv", x);
      aggregator->PlotData( filename, maxSimulatedTime, 1.0 );
      aggregator->ClearData();

      ++nSeed;
      RngSeedManager::SetSeed(nSeed);
   }

   return 0;
}
