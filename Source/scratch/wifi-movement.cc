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
 *
 * This module tests the movement of nodes around an area, where the moving
 * node and center node attempt to communicate and various stationary nodes
 * reside in locations around the center node to act as a forwarding 
 * device in the network.
 *
 *       0,0                         110,0
 *        2(D)                         2
 *
 *       0,25
 *         3                                130,40
 *                      70,60                  2
 *                        4
 *
 *
 *                              100,90            145,90
 *                                 5                 2
 *       0,125
 *         1(S)
 *
 *                              100,150
 *                                 6              145,160
 *                                                   2
 *
 *                      70,190
 *                        7                130,200
 *                                            2
 *       0,225
 *        8                           110,230
 *                                       2
 *
 *       0,250         70,250
 *         2             2
 *
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
#include "ns3/batmand-routing-protocol.h"
#include "ns3/batmand-helper.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/olsr-helper.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/aggregator-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiMvmnt");

void PktValue( Ptr<Packet const> initpkt ) {
  Mac48Address addr;
  
  //  printf(" pkt id %lu has size %u at Time %f\n", initpkt->GetUid(), initpkt->GetSize(), (Simulator::Now().GetDouble() / 1000000000.0) );
}


int
main (int argc, char *argv[])
{
  uint32_t nWifi = 2;
  int  nRun = 0;
  int  nSeed = 5;
  int  nCount = 1;
  double   maxSimulatedTime = 350.0;
  
  CommandLine cmd;
  cmd.AddValue ("nTime", "Maximum simulation time", maxSimulatedTime );
  cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue ("nRun", "Simulation run number", nRun);
  cmd.AddValue ("nSeed", "Seed value that differs from the default", nSeed);
  cmd.AddValue ("nCount", "Number of iterations to run this scenario with differing PRNG", nCount);
  cmd.Parse (argc,argv);
  
  RngSeedManager::SetSeed(nSeed);
  RngSeedManager::SetRun(nRun);
  
  for ( int j = 0; j < nCount; ++j )
  {
    for ( uint8_t itr = 0; itr < 2; ++itr )
    {
      /*
       * Create NS-3 Nodes to accumulate net devices
       */
      NodeContainer allNodes;
      NodeContainer wifiStaNodes;
      NodeContainer wifiSrvrNode;
      NodeContainer manetNodes;
      NodeContainer clientNodes;
      
      // Create the server first so it is MAC 00:00:00:00:00:01
      wifiSrvrNode.Create( 1 );
      // Now create the stationary nodes that will act as forwarders
      // Note that Node 1 in the STA nodes is the mobile node
      wifiStaNodes.Create( 7 );
      
      allNodes.Add( wifiSrvrNode );
      allNodes.Add( wifiStaNodes );
      manetNodes.Add( wifiStaNodes );
      manetNodes.Add( wifiSrvrNode );
      clientNodes.Add( wifiStaNodes.Get(0) );
      
      /*
       * Setup the WiFi Channel, MAC, and MAC Helper for AD HOC Mode
       */
      NetDeviceContainer srvrDevice;
      NetDeviceContainer staDevices;
      YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
      YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
      WifiHelper wifi;
      WifiMacHelper mac;
      {
        int count = 0;
        phy.SetChannel (channel.Create ());
        phy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
        
        wifi.SetRemoteStationManager ("ns3::AarfWifiManager");
        mac.SetType ("ns3::AdhocWifiMac");
        
        srvrDevice = wifi.Install (phy, mac, wifiSrvrNode);
        staDevices = wifi.Install (phy, mac, wifiStaNodes);
        
        count = wifi.AssignStreams( srvrDevice, 43534 );
        count = wifi.AssignStreams( staDevices, 43534 + count );
      }
      
      /*
       * Setup the mobility model for the nodes
       */
      MobilityHelper mobility;
      {
        // The first mobility model is a constant position for the UDP Echo Server - i.e. sink
        // Node is at X = 0, Y = 50
        Ptr<ListPositionAllocator> positionAlloc1 = CreateObject<ListPositionAllocator> ();
        positionAlloc1->Add (Vector (0,125,0));
        mobility.SetPositionAllocator (positionAlloc1);
        mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobility.Install (wifiSrvrNode.Get(0));
        
        // Now lay out the fixed positions for nodes 2-7 within the STA container
        Ptr<ListPositionAllocator> positionAlloc2 = CreateObject<ListPositionAllocator> ();
        positionAlloc2->Add (Vector (0,25,0));
        mobility.SetPositionAllocator (positionAlloc2);
        mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobility.Install (wifiStaNodes.Get(1));
        
        Ptr<ListPositionAllocator> positionAlloc3 = CreateObject<ListPositionAllocator> ();
        positionAlloc3->Add (Vector (70,60,0));
        mobility.SetPositionAllocator (positionAlloc3);
        mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobility.Install (wifiStaNodes.Get(2));
        
        Ptr<ListPositionAllocator> positionAlloc4 = CreateObject<ListPositionAllocator> ();
        positionAlloc4->Add (Vector (100,90,0));
        mobility.SetPositionAllocator (positionAlloc4);
        mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobility.Install (wifiStaNodes.Get(3));
        
        Ptr<ListPositionAllocator> positionAlloc5 = CreateObject<ListPositionAllocator> ();
        positionAlloc5->Add (Vector (100,150,0));
        mobility.SetPositionAllocator (positionAlloc5);
        mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobility.Install (wifiStaNodes.Get(4));
        
        Ptr<ListPositionAllocator> positionAlloc6 = CreateObject<ListPositionAllocator> ();
        positionAlloc6->Add (Vector (70,190,0));
        mobility.SetPositionAllocator (positionAlloc6);
        mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobility.Install (wifiStaNodes.Get(5));
        
        Ptr<ListPositionAllocator> positionAlloc7 = CreateObject<ListPositionAllocator> ();
        positionAlloc7->Add (Vector (0,225,0));
        mobility.SetPositionAllocator (positionAlloc7);
        mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobility.Install (wifiStaNodes.Get(6));
      }

      /*
       * Now add the waypoint based movement around the network forwarding stations
       * This is the STA node 0 i.e. client
       * Move at 1.4m/s
       */
      Ptr<WaypointMobilityModel> mob;
      MobilityHelper waymobility;
      {
        waymobility.SetMobilityModel("ns3::WaypointMobilityModel");
        waymobility.Install(wifiStaNodes.Get(0));
        mob = wifiStaNodes.Get(0)->GetObject<WaypointMobilityModel>();
        
        Waypoint clnt_p0 = Waypoint(Seconds(0),Vector(0,0,0));
        mob->AddWaypoint(clnt_p0);
        
        Waypoint clnt_p1 = Waypoint(Seconds(4),Vector(0,0,0));
        mob->AddWaypoint(clnt_p1);
        
        Waypoint clnt_p2 = Waypoint(Seconds(75.4),Vector(100,0,0));
        mob->AddWaypoint(clnt_p2);
        
        Waypoint clnt_p3 = Waypoint(Seconds(107.4),Vector(130,40,0));
        mob->AddWaypoint(clnt_p3);
        
        Waypoint clnt_p4 = Waypoint(Seconds(150.25),Vector(145,90,0));
        mob->AddWaypoint(clnt_p4);
        
        Waypoint clnt_p5 = Waypoint(Seconds(202.25),Vector(145,160,0));
        mob->AddWaypoint(clnt_p5);
        
        Waypoint clnt_p6 = Waypoint(Seconds(287.96),Vector(130,200,0));
        mob->AddWaypoint(clnt_p6);
        
        Waypoint clnt_p7 = Waypoint(Seconds(308.78),Vector(110,230,0));
        mob->AddWaypoint(clnt_p7);
        
        Waypoint clnt_p8 = Waypoint(Seconds(333.78),Vector(70,225,0));
        mob->AddWaypoint(clnt_p8);

        Waypoint clnt_p9 = Waypoint(Seconds(350.0),Vector(0,250,0));
        mob->AddWaypoint(clnt_p9);
      }
      
      /*
       * Setup the WiFi energy model
       */
      Ptr<BasicEnergySource> energySource = CreateObject<BasicEnergySource>();
      Ptr<SimpleDeviceEnergyModel> energyModel = CreateObject<SimpleDeviceEnergyModel>();
      energySource->SetInitialEnergy (300);
      energyModel->SetEnergySource (energySource);
      energySource->AppendDeviceEnergyModel (energyModel);
      energyModel->SetCurrentA (20);
      
      /*
       * Configure the BATMAN or OLSR engine
       */
      BatmandHelper batman;
      OlsrHelper olsr;
      Ipv4StaticRoutingHelper staticRouting;
      Ipv4ListRoutingHelper list;
      InternetStackHelper internet_helper;
      list.Add (staticRouting, 0);

      if ( itr == 0 )
        list.Add (batman, 10);
      else
        list.Add (olsr, 10);
      
      internet_helper.SetRoutingHelper (list); // has effect on the next Install ()
      internet_helper.Install (manetNodes);
      
      /*
       * Create the internet stack
       */
      // Install Ipv4 addresses
      Ipv4AddressHelper address;
      address.SetBase ("10.1.1.0", "255.255.255.0");
      Ipv4InterfaceContainer srvrInterface;
      srvrInterface = address.Assign (srvrDevice);
      Ipv4InterfaceContainer staInterfaces;
      staInterfaces = address.Assign (staDevices);
      
      // Install applications
      UdpEchoServerHelper echoServer (9);
      ApplicationContainer serverApps = echoServer.Install (wifiSrvrNode.Get (0));
      serverApps.Start (Seconds (1.0));
      serverApps.Stop (Seconds ( maxSimulatedTime ));
      UdpEchoClientHelper echoClient (srvrInterface.GetAddress (0), 9);
      echoClient.SetAttribute ("MaxPackets", UintegerValue (10000));
      echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
      echoClient.SetAttribute ("PacketSize", UintegerValue (1024));
      ApplicationContainer clientApps = echoClient.Install (clientNodes);
      clientApps.Start (Seconds (3.0));
      clientApps.Stop (Seconds ( maxSimulatedTime ));
      
      //  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
      Simulator::Stop (Seconds ( maxSimulatedTime ));
      
      std::string animation_filename;
      
      if ( itr == 0 )
        animation_filename = "movement/batman-animation.xml";
      else
        animation_filename = "movement/olsr-animation.xml";
      
      AnimationInterface anim( animation_filename );
      anim.UpdateNodeDescription (wifiStaNodes.Get (0), "CLI"); // Optional
      anim.UpdateNodeColor (wifiStaNodes.Get (0), 0, 255, 255); // Optional

      for (uint32_t i = 1; i < wifiStaNodes.GetN (); ++i)
      {
        anim.UpdateNodeDescription (wifiStaNodes.Get (i), "FWD"); // Optional
        anim.UpdateNodeColor (wifiStaNodes.Get (i), 255, 0, 0); // Optional
      }
      for (uint32_t i = 0; i < wifiSrvrNode.GetN (); ++i)
      {
        anim.UpdateNodeDescription (wifiSrvrNode.Get (i), "SRV"); // Optional
        anim.UpdateNodeColor (wifiSrvrNode.Get (i), 0, 255, 0); // Optional
      }
      
      anim.EnablePacketMetadata (); // Optional

      if ( itr == 0 )
        anim.EnableIpv4RouteTracking ("movement/batman-routingtable-wireless.xml", Seconds (0), Seconds (maxSimulatedTime), Seconds (1.0)); //Optional
      else
        anim.EnableIpv4RouteTracking ("movement/olsr-routingtable-wireless.xml", Seconds (0), Seconds (maxSimulatedTime), Seconds (1.0)); //Optional

      anim.EnableWifiMacCounters (Seconds (0), Seconds (maxSimulatedTime)); //Optional
      anim.EnableWifiPhyCounters (Seconds (0), Seconds (maxSimulatedTime)); //Optional
      anim.SetMaxPktsPerTraceFile(1000000000);

      // Setup tracing in PCAP
      if ( itr == 0 )
      {
        phy.EnablePcap ("movement/batman-hna", staDevices, true);
        phy.EnablePcap ("movement/batman-hna", srvrDevice, true);
      }
      else
      {
        phy.EnablePcap ("movement/olsr-hna", staDevices, true);
        phy.EnablePcap ("olsr-hna", srvrDevice, true);
      }
      
      AggregatorHelper aggHelper;
      Ptr<Aggregator>  aggregator = aggHelper.GetAggregator();
      
      // Add all nodes to the packet aggregator
      aggHelper.Install( allNodes );
      // Now add the Udp Echo client server applications
      aggHelper.InstallUdpClient( clientApps );
      aggHelper.InstallUdpServer( serverApps );
      
      // Just for debug
      Ptr<UdpEchoServer> serverptr = DynamicCast<UdpEchoServer> ( serverApps.Get( 0 ) );
      Ptr<UdpEchoClient> clientptr = DynamicCast<UdpEchoClient> ( clientApps.Get( 0 ) );
      serverptr->TraceConnectWithoutContext( "Rx" , MakeCallback( &PktValue ) ) ;
      clientptr->TraceConnectWithoutContext( "Rx" , MakeCallback( &PktValue ) );
      
      printf("Run the simulator...\n");
      Simulator::Run ();
      Simulator::Destroy ();
      printf("Simulator finished...\n");
      
      // Now generate the output CSV file for plotting
      // NOTE: Adjust the third argument for scalling factor - timeline ticks in seconds
      char filename[200];
      if ( itr == 0 )
      {
        sprintf( filename, "movement/batman-movement-%02d.csv", j );
        aggregator->PlotData( filename, maxSimulatedTime, 1.0 );
      }
      else
      {
        sprintf( filename, "movement/olsr-movement-%02d.csv", j );
        aggregator->PlotData( filename, maxSimulatedTime, 1.0 );
      }
      
      printf("Done with a test round\n");
    }
    
    nSeed *= 13;
    nRun = ( (nRun * nSeed) / 7 ) + 5;
    RngSeedManager::SetSeed(nSeed);
    RngSeedManager::SetRun(nRun);
  }
}

