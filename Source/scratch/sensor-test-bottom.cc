/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
 *
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
 * Author:  Tom Henderson <thomas.r.henderson@boeing.com>
 *
 * Modified: Nelson Powell <nhp8080@rit.edu>
 *  Modified for Sensor MAC testing.
 */

/*
 * Try to send data end-to-end through a LrWpanMac <-> LrWpanPhy <->
 * SpectrumChannel <-> LrWpanPhy <-> LeachMac chain
 *
 * Trace Phy state changes, and Mac DataIndication and DataConfirm events
 * to stdout
 */
#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/sensor-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/waypoint-mobility-model.h"
#include "ns3/netanim-module.h"
#include "ns3/basic-energy-source.h"
#include "ns3/simple-device-energy-model.h"
#include <ns3/position-allocator.h>

#include <iostream>
#include <string>     // std::string, std::to_string
#include <stdio.h>
#include <stdlib.h>

#define MACOSX  1
//#undef MACOSX

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LeachData");

static void DataIndication (SensorDataIndicationParams params, Ptr<Packet> p)
{
  NS_LOG_DEBUG ("Received packet of size " << p->GetSize ());
}

static void DataConfirm (SensorDataConfirmParams params)
{
  NS_LOG_DEBUG ("DataConfirmStatus = " << params.m_status);
}

static void StateChangeNotification (std::string context, Time now, LrWpanPhyEnumeration oldState, LrWpanPhyEnumeration newState)
{
  //  NS_LOG_DEBUG (context << " state change at " << now.GetSeconds ()
  //                 << " from " << SensorHelper::LrWpanPhyEnumerationPrinter (oldState)
  //                 << " to " << SensorHelper::LrWpanPhyEnumerationPrinter (newState));
}

int main (int argc, char *argv[])
{
  bool verbose = false;
  bool uniform = false;
  int  nNodes = 3;
  int  nClusters = 1;
  int  nRun = 0;
  int  nSeed = 5;
  int  nMode = LEACH_O;
  double nPower = 100.0;
  double nTime = 5.0;
  double nDatInt = 0.5;
  char fname[200];
  char aname[200];
  char dname[200] = ".";

  CommandLine cmd;
  
  cmd.AddValue ("verbose", "turn on all log components", verbose);
  cmd.AddValue ("nNodes", "Number of member nodes for simulation", nNodes);
  cmd.AddValue ("nClusters", "Number of CHs required per LEACH Round", nClusters);
  cmd.AddValue ("nRun", "Simulation run number", nRun);
  cmd.AddValue ("nSeed", "Seed value that differs from the default", nSeed);
  cmd.AddValue ("nPower", "Starting system power", nPower);
  cmd.AddValue ("nTime", "Time (sec) of execution of the sim", nTime );
  cmd.AddValue ("nMode", "0 = LEACH, 1 = BATSEN", nMode );
  cmd.AddValue ("nDatInt", "Frequency at which 1 data pkt/node is queued", nDatInt );
  cmd.AddValue ("nUniform", "TRUE if uniform distributions, FALSE if random", uniform );
  cmd.AddValue ("dirname", "Directory to push the data files", dname);

  cmd.Parse (argc, argv);
  
  Packet::EnablePrinting ();
  
  RngSeedManager::SetSeed(nSeed);
  RngSeedManager::SetRun(nRun);
  
  SensorHelper sensorHelper;
  if (verbose)
    sensorHelper.EnableLogComponents ();

  switch ( nMode )
  {
    case BATSEN:
      NS_LOG_UNCOND(" Setting network to BATSEN " );
      sensorHelper.SetNetworkType( BATSEN );
      Config::SetDefault("ns3::BatsenMac::NumNodes", IntegerValue( nNodes - 1 ) );
      Config::SetDefault("ns3::BatsenMac::NumClusters", IntegerValue( nClusters ) );
      sprintf( fname, "%s/batsen_nodes_%d", dname, nNodes );
      sprintf( aname, "%s/batsen_anim_%d.xml", dname, nNodes );
      break;
      
    default:
    case LEACH_O:
      NS_LOG_UNCOND(" Setting network to LEACH " );
      sensorHelper.SetNetworkType( LEACH_O );
      Config::SetDefault("ns3::LeachMac::NumNodes", IntegerValue( nNodes - 1 ) );
      Config::SetDefault("ns3::LeachMac::NumClusters", IntegerValue( nClusters ) );
      sprintf( fname, "%s/leach_nodes_%d_chs_%02d", dname, nNodes, nClusters );
      sprintf( aname, "%s/leach_anim_%d_chs_%02d.xml", dname, nNodes, nClusters );
      break;
  }
  
  sensorHelper.SetDefaultPower( nPower );
  
  // Enable calculation of FCS in the trailers. Only necessary when interacting with real devices or wireshark.
  // GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
  
  /*
   * Create 3 nodes by default, and a NetDevice for each one - we're
   * using the default SensorNet which is LEACH Original.  Don't forget
   * to assign the random number generator streams.
   */
  NodeContainer net;
  NodeContainer sinkNode;
  NodeContainer senseNodes;
  Ptr<UniformRandomVariable> m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
  
  uint64_t rndstreams = 0;
  
  NetDeviceContainer devices;
  net.Create( nNodes );
  
  sinkNode.Add( net.Get(0) );
  
  for ( unsigned j = 1; j < net.GetN(); j++ )
    senseNodes.Add( net.Get(j) );
  
  
  // Mobility - do this right away to use the same PRNG stream ?
  MobilityHelper mobility;
  
  // Set Random positions for the Non-SINK Nodes
  if ( uniform )
    mobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                  "rho", DoubleValue (46.5),
                                  "X", DoubleValue (50.0),
                                  "Y", DoubleValue (50.0) );
  else
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                  "Rho", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=46.5]"),
                                  "X", DoubleValue (50.0),
                                  "Y", DoubleValue (50.0) );
  
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(senseNodes);
  
  
  // Set fixed position for the SINK node
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (50,95,0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install(sinkNode);
  
  
  
  AnimationInterface anim( aname ); // Mandatory
  anim.UpdateNodeDescription (net.Get(0), "SINK"); // Optional
  anim.UpdateNodeColor (net.Get(0), 255, 0, 0); // Optional
  
  for (uint32_t i = 1; i < net.GetN(); ++i)
  {
    //    anim.UpdateNodeDescription (net.Get(i), "NODE"); // Optional
    anim.UpdateNodeColor (net.Get(i), 0, 255, 0); // Optional
  }
  
  
  devices = sensorHelper.Install( net );
  rndstreams = sensorHelper.AssignStreams( devices, 43534 );
  //sensorHelper.SetRandomPositions( devices );
  sensorHelper.OpenFile(fname);

  m_uniformRandomVariable->SetStream( rndstreams + 10 );
  
  /*
   * Create a net device container pointing to all the SensorNetDevices
   * in the sensor node container.  This way, we can add callbacks
   * and stuff post creation.
   * */
  for ( uint32_t i = 0; i < net.GetN(); ++i )
  {
    Ptr<SensorNetDevice>dev = devices.Get(i)->GetObject<SensorNetDevice>();
    
#ifdef MACOSX
    std::stringstream ss;
    ss << (i + 1);
    std::string str = ss.str();
    
    std::string phyname = "phy" + str;
#else
    std::string phyname = "phy" + std::to_string( i + 1 );
#endif
    
    NS_LOG_DEBUG("setting the phy name for State Change Callback to " << phyname << " for node " << i );
    
    dev->GetPhy()->TraceConnect ("TrxState", phyname, MakeCallback (&StateChangeNotification));
    
    dev->GetMac()->SetSensorDataConfirmCallback( MakeCallback( &DataConfirm ) );
    dev->GetMac()->SetSensorDataIndicationCallback( MakeCallback( &DataIndication ) );
  }
  
  // More animation setup stuff
  anim.EnablePacketMetadata(); // Optional
  anim.EnableWifiMacCounters(Seconds (0), Seconds (10)); //Optional
  anim.EnableWifiPhyCounters(Seconds (0), Seconds (10)); //Optional
  anim.SetMaxPktsPerTraceFile(100000000);
  
  // Tracing
  //  sensorHelper.EnablePcapAll (std::string ("leach-data"), true);
  //  AsciiTraceHelper ascii;
  //  Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("leach-data.tr");
  //  sensorHelper.EnableAsciiAll (stream);
  
  // Setup the data delivery rate of 1pkt/500ms = 2 pkt/sec
  sensorHelper.SetMaxSequence( 10000000 );
  
  /*
   * Send 1 data packet per node every interval, where the interval is 
   * 1 sec / rate (i.e. nDatInt). Therefore,
   */
  sensorHelper.ConfigureDataRate( nDatInt, 100, devices);
  
  Simulator::Stop( Seconds( nTime ) );
  
  Simulator::Run();
  
  //sensorHelper.WhoisAlive();
  
  Simulator::Destroy();
  return 0;
}
