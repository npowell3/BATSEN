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
#include "ns3/batmand-routing-protocol.h"
#include "ns3/batmand-helper.h"
#include "ns3/aggregator-helper.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("BatmanBigBox");


//#define  PLOT_ITEMS      1
#undef   PLOT_ITEMS

#ifndef  PLOT_ITEMS
   #define  PLOT_DATA_NORM    1
#else
   #undef   PLOT_DATA_NORM
#endif

//#define DIR_PLOT_NORM   1
#undef DIR_PLOT_NORM

int
main (int argc, char *argv[])
{
   uint32_t nWifi = 20;
   int      nRun = 0;
   int      nSeed = 0;
   int      nIter = 1;
   int      strmCnt = 0;
   int      strmTot = 0;
   double   nInterval = 1.0;
   double   maxSimulatedTime = 60.0;
   double   nMultiplier = 5.0;
   double   nJitter = 9.0;

   CommandLine cmd;
   cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
   cmd.AddValue ("nRun", "Simulation run number", nRun);
   cmd.AddValue ("nSeed", "Seed value that differs from the default", nSeed);
   cmd.AddValue ("nIter", "Number of iterations to execute", nIter);
   cmd.AddValue ("nInterval", "OGM Interval length", nInterval);
   cmd.AddValue ("nJitter", "OGM Interval divisor to spread jitter", nJitter);
   cmd.AddValue ("nMultiplier", "Multiplier used to define OGM aggregation period", nMultiplier);

   cmd.Parse (argc,argv);

   RngSeedManager::SetSeed(nSeed);
   RngSeedManager::SetRun(nRun);

   char wrkndir[PATH_MAX];

   if ( NULL == getcwd( wrkndir, PATH_MAX ) )
     printf("error (%d): %s\n", errno, strerror( errno ) );
   else
     printf("starting from dir <%s>\n", wrkndir);

#ifdef PLOT_ITEMS
   char filename[200];
   
   sprintf( filename, "batman-optimize_%f.csv", nInterval );

   int plotFd = open( filename, O_RDWR | O_CREAT, S_IRWXU );
   
   if ( plotFd == -1 ) {
      printf(" Error opening plot file (%u) <%s>\n", errno, strerror(errno) );
      exit(1);
   }
   else {
      const char *str = "Interval,Jitter,Multiplier,Agg Total BW,Agg User BW,Agg Router BW,Avg Total BW,Avg User BW,Avg Router BW\n\0";
      if ( write( plotFd, str, strlen( str ) ) == -1 ) {
         printf("ERROR: writing to FD %d: (%d) <%s>\n", plotFd, errno, strerror( errno ) );
         exit(1);
      }
   }
#endif
   
#ifdef   PLOT_DATA_NORM
  //   Config::SetDefault("ns3::batmand::RoutingProtocol::OGMInterval", TimeValue( Seconds( nInterval )) );
  //   Config::SetDefault("ns3::batmand::RoutingProtocol::OGMDivisor", DoubleValue( nJitter ) );
  //   Config::SetDefault("ns3::batmand::RoutingProtocol::AGGMult", DoubleValue( nMultiplier ) );
#endif
   
   for (int x = 0; x < nIter; ++x ) {

#ifdef PLOT_ITEMS
     //      Config::SetDefault("ns3::batmand::RoutingProtocol::OGMInterval", TimeValue (Seconds (nInterval)) );
#endif
      
#ifdef PLOT_ITEMS
      for ( double jitterbug = 1.0; jitterbug < 10; ++jitterbug) {
         // 1.0 to 10.0   => OGM Interval / OGM Divsior -- JITTER = 2 to 0.2 seconds
         //         Config::SetDefault("ns3::batmand::RoutingProtocol::OGMDivisor", DoubleValue(jitterbug) );

         for ( double multiplier = 0.1; multiplier < 55.0; ) {
#endif
            
            printf("Start a simulation\n");
#ifdef   DIR_PLOT_NORM
            // Create a directory for the current run
            char foldername[PATH_MAX];
            sprintf( foldername, "%s/jit%0.2fmul%0.2f", wrkndir, jitterbug, multiplier);

            if ( 0 != mkdir( foldername, S_IRWXU | S_IRWXG | S_IRWXO ) ) {
               printf("err creating [%s] (%d) <%s>\n", foldername, errno, strerror( errno ) );
               exit(1);
            }

            if ( 0 != chdir( foldername ) ) {
               printf("err changing to [%s] (%d) <%s>\n", foldername, errno, strerror( errno ) );
               exit(1);
            }
            else
               printf("\tchange dir to <%s>\n", foldername );
#endif
        
#ifdef PLOT_ITEMS            
            // 0.1 to 1000.0 = 100us * Aggregate Multiplier -- T between OGMS aggregated = 10us to 100ms
            //            Config::SetDefault("ns3::batmand::RoutingProtocol::AGGMult", DoubleValue(multiplier) );
#endif
            
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
            BatmandHelper olsr;                       // must assign fixed random streams after installation
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

           //strmCnt = olsr.AssignStreams( olsrNodes, 101234 );
            NS_LOG_DEBUG(" assigned " << strmCnt << " rnd streams to Batman protocol");
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
            sprintf( animName, "batmand-big-box-animation-%d.xml", x);
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

            // Setup tracing in PCAP
            //  phy.EnablePcap ("batmand-big-box", staDevices);
            //  phy.EnablePcap ("batmand-big-box", srvrDevice);
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
#ifdef PLOT_DATA_NORM
            char filename[250];
            sprintf(filename, "batman-int_%f_jit_%f_mul_%f-data_%d.csv", nInterval, nJitter, nMultiplier, x);
            aggregator->PlotData( filename, maxSimulatedTime, 1.0 );
            aggregator->ClearData();
#elif defined(DIR_PLOT_NORM) 
            char filename[250];
            sprintf(filename, "batman-jit_%f_mul_%f.csv", nJitter, nMultiplier);
            aggregator->PlotData( filename, maxSimulatedTime, 1.0 );
            aggregator->ClearData();
#endif
            
#ifdef PLOT_ITEMS
            char str[250] = "\0";
            sprintf( str, "%0.2f,%0.2f,%0.2f", nInterval, jitterbug, multiplier );
            aggregator->ExtractData( maxSimulatedTime, 1.0 );
            aggregator->PlotItems( 9, plotFd, ROW_HEADER_STR, str, AGG_TOTAL_BW, AGG_USER_BW, AGG_ROUTER_BW, AVG_TOTAL_BW, AVG_USER_BW, AVG_ROUTER_BW );

            aggregator->ClearData();

            RngSeedManager::SetSeed(nSeed);

            // This is the end of loop increamentor for the MULTIPLIER for loop
            if ( multiplier < 0.9 )
               multiplier += 0.1;
            else if ( multiplier < 10 )
               multiplier += 1.0;
            else 
               multiplier += 10;
#endif
            
#ifdef DIR_PLOT_NORM               
            if ( 0 != chdir( wrkndir ) ) {
               printf("\t err ret to wrk dir (%d) <%s>\n", errno, strerror( errno ) );
               exit(1);
            }
#endif
            printf("\tEND of a simulation\n");
            
#ifdef PLOT_ITEMS
         }
      }
#endif

      ++nSeed;
      RngSeedManager::SetSeed(nSeed);
   }

#ifdef PLOT_ITEMS
   close( plotFd );
#endif
   
   return 0;
}
