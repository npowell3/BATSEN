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

#include <iostream>
#include <string>     // std::string, std::to_string
#include <stdio.h>
#include <stdlib.h>

#define MACOSX  1
//#undef MACOSX

//#define BOTH_MODES    1
#undef BOTH_MODES

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
  int  nNodes = 3;
  int  nClusters = 1;
  int  nRun = 0;
  int  nSeed = 5;
  int  nMode = LEACH_O;
  double nPower = 100.0;
  double nTime = 5.0;
  double nDatInt = 0.5;
  unsigned nLoops = 1;
  char fname[200];

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
  cmd.AddValue ("nLoops", "Number of loops to run through this test", nLoops );

  cmd.Parse (argc, argv);
  
  Packet::EnablePrinting ();
    
  for ( unsigned y = 0; y < nLoops; ++y )
  {
#ifdef BOTH_MODES
    // Perform the same test twice, once for LEACH and once for BATSEN
    for ( unsigned x = 0; x < 2; ++x )
    {
      // Assert the sensor mode
      nMode = x;
#endif
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
            sprintf( fname, "batsen_nodes_%d", nNodes );
            break;
            
        default:
        case LEACH_O:
            NS_LOG_UNCOND(" Setting network to LEACH " );
            sensorHelper.SetNetworkType( LEACH_O );
            Config::SetDefault("ns3::LeachMac::NumNodes", IntegerValue( nNodes - 1 ) );
            Config::SetDefault("ns3::LeachMac::NumClusters", IntegerValue( nClusters ) );
            sprintf( fname, "leach_nodes_%d_chs_%02d", nNodes, nClusters );
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
      NetDeviceContainer devices;
      net.Create( nNodes );

      devices = sensorHelper.Install( net );
      sensorHelper.AssignStreams( devices, 43534 );
      sensorHelper.SetRandomPositions( devices );
      sensorHelper.OpenFile(fname);

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
        
        /*
        * Now set the mobility profile for the Sink at a specific location
        * and random locations for the rest of the network members.
        */
        if ( i == 0 )
        {
            Ptr<ConstantPositionMobilityModel> senderMobility = CreateObject<ConstantPositionMobilityModel> ();
            
            senderMobility->SetPosition (Vector (0,0,0));
            dev->GetPhy ()->SetMobility (senderMobility);
        }
        /*
        else
        {
            Ptr<ConstantPositionMobilityModel> senderMobility = CreateObject<ConstantPositionMobilityModel> ();
            
            // TODO: Add random position assignments
            if ( ( i % 2 ) == 0 )
              senderMobility->SetPosition (Vector (0,50,0));
            else
              senderMobility->SetPosition (Vector (0,0,0));
            
            dev->GetPhy ()->SetMobility (senderMobility);
        }
        */
      }

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

      Simulator::Destroy();
   
#ifdef BOTH_MODES
    }   // End for loop on modes
#endif

    nSeed = ((nSeed * nRun) + 7) / 6;
    ++nRun;
    
  }     // Loop through the test for nLoops times

  return 0;
}
