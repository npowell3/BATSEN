/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "sensor-helper.h"
#include <ns3/sensor-csma.h>
#include <ns3/sensor-mac.h>
#include <ns3/leach-mac.h>
#include <ns3/batsen2-mac.h>
#include <ns3/batsen-mac.h>
#include <ns3/lr-wpan-error-model.h>
//#include <ns3/sensor.h>
#include <ns3/mobility-model.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/log.h>
#include "ns3/names.h"

#include <iostream>
#include <string>
#include <fstream>

#include <unistd.h>
#include <arpa/inet.h>

#define MACOSX  1
//#undef MACOSX

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SensorHelper");

/**
 * @brief Output an ascii line representing the Transmit event (with context)
 * @param stream the output stream
 * @param context the context
 * @param p the packet
 */
static void
AsciiSensorMacTransmitSinkWithContext (Ptr<OutputStreamWrapper> stream,
                                       std::string context,
                                       Ptr<const Packet> p)
{
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " " << context << " " << *p << std::endl;
}

/**
 * @brief Output an ascii line representing the Transmit event (without context)
 * @param stream the output stream
 * @param p the packet
 */
static void
AsciiSensorMacTransmitSinkWithoutContext (Ptr<OutputStreamWrapper> stream,
                                          Ptr<const Packet> p)
{
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " " << *p << std::endl;
}

SensorHelper::SensorHelper (void) :
  m_power( 1.0 ),
  m_netType( LEACH_O ),
  m_nodeCount( 0 ),
  m_maxSeq( 2 ),
  m_netAlive( true ),
  m_firstFail( Seconds(0.0) ),
  m_fiftyFail( Seconds(0.0) ),
  m_lastFail( Seconds(0.0) )
{
  // Need the PRNG for the Threshold detection algorithm
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
  
  m_trackMap.clear();
  m_roundMap.clear();
  m_round = 0;

  m_channel = CreateObject<SingleModelSpectrumChannel> ();
  
  Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel> ();
  
  m_channel->AddPropagationLossModel (lossModel);
  
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  
  m_channel->SetPropagationDelayModel (delayModel);
  
  m_discAllocator = CreateObject<UniformDiscPositionAllocator> ();
}

SensorHelper::SensorHelper (bool useMultiModelSpectrumChannel) :
  m_power( 1.0 ),
  m_netType( LEACH_O ),
  m_nodeCount( 0 ),
  m_maxSeq( 2 ),
  m_netAlive( true ),
  m_firstFail( Seconds(0.0) ),
  m_fiftyFail( Seconds(0.0) ),
  m_lastFail( Seconds(0.0) )
{
  m_trackMap.clear();
  m_roundMap.clear();
  m_round = 0;

  if (useMultiModelSpectrumChannel)
  {
    m_channel = CreateObject<MultiModelSpectrumChannel> ();
  }
  else
  {
    m_channel = CreateObject<SingleModelSpectrumChannel> ();
  }
  Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel> ();
  m_channel->AddPropagationLossModel (lossModel);
  
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  m_channel->SetPropagationDelayModel (delayModel);
}

SensorHelper::~SensorHelper (void)
{
  m_channel->Dispose ();
  m_channel = 0;
  m_trackMap.clear();
  m_roundMap.clear();
  m_lifeFile.close();
  m_avgChFile.close();
  m_avgFileLen.close();
}

bool
SensorHelper::OpenFile( std::string fname )
{
  char avgName[255];
  char lifeName[255];
  char drName[255];
  char rndName[255];
  char durName[255];
  char dcName[255];
  char rxpktName[255];
  char txpktName[255];
  char perpktName[255];
  
  sprintf( avgName, "%s_%s", fname.c_str(), "avgChs" );
  sprintf( lifeName, "%s_%s", fname.c_str(), "nodeLife" );
  sprintf( drName, "%s_%s", fname.c_str(), "dataRate" );
  sprintf( rndName, "%s_%s", fname.c_str(), "rndStats" );
  sprintf( durName, "%s_%s", fname.c_str(), "durations" );
  sprintf( dcName, "%s_%s", fname.c_str(), "dircons" );
  sprintf( rxpktName, "%s_%s", fname.c_str(), "rxpackets" );
  sprintf( txpktName, "%s_%s", fname.c_str(), "txpackets" );
  sprintf( perpktName, "%s_%s", fname.c_str(), "perpackets" );
  
  NS_LOG_UNCOND( " avgName: " << avgName );
  NS_LOG_UNCOND( " lifeName: " << lifeName );
  NS_LOG_UNCOND( " drateName: " << drName );
  NS_LOG_UNCOND( " rndName: " << rndName );
  NS_LOG_UNCOND( " durName: " << durName );
  NS_LOG_UNCOND( " dcName: " << dcName );
  NS_LOG_UNCOND( " rxpktName: " << rxpktName );
  NS_LOG_UNCOND( " txpktName: " << txpktName );
  NS_LOG_UNCOND( " perpktName: " << perpktName );
  
  m_avgFileLen.open( avgName, std::ios::in );
  m_avgChFile.open ( avgName, std::ios::out | std::ios::app );
  
  m_lifeFile.open ( lifeName, std::ios::out | std::ios::app );
  
  m_rateFileLen.open( drName, std::ios::in );
  m_rateFile.open ( drName, std::ios::out | std::ios::app );

  m_roundFile.open ( rndName, std::ios::out | std::ios::app );
  m_duratFile.open ( durName, std::ios::out | std::ios::app );
  m_duratFileLen.open( durName, std::ios::in );
  
  m_dcFile.open ( dcName, std::ios::out | std::ios::app );
  m_dcFileLen.open( dcName, std::ios::in );

  m_pktFile.open ( rxpktName, std::ios::out | std::ios::app );
  m_pktFileLen.open( rxpktName, std::ios::in );

  m_txpktFile.open ( txpktName, std::ios::out | std::ios::app );
  m_txpktFileLen.open( txpktName, std::ios::in );

  m_pktPerFile.open ( perpktName, std::ios::out | std::ios::app );
  m_pktPerFileLen.open( perpktName, std::ios::in );
  
  m_avgFileLen.seekg (0, std::ios_base::beg );
  m_avgChFile.seekp (0, std::ios_base::end );
  m_lifeFile.seekp (0, std::ios_base::end );
  m_rateFile.seekp (0, std::ios_base::end );
  m_roundFile.seekg (0, std::ios_base::end );  
  m_duratFile.seekg (0, std::ios_base::end );
  m_duratFileLen.seekg (0, std::ios_base::beg );
  m_dcFile.seekg (0, std::ios_base::end );
  m_dcFileLen.seekg (0, std::ios_base::beg );
  m_pktFile.seekg (0, std::ios_base::end );
  m_pktFileLen.seekg (0, std::ios_base::beg );
  m_txpktFile.seekg (0, std::ios_base::end );
  m_txpktFileLen.seekg (0, std::ios_base::beg );
  m_pktPerFile.seekg (0, std::ios_base::end );
  m_pktPerFileLen.seekg (0, std::ios_base::beg );

  
  NS_LOG_UNCOND( " avg open " << m_avgChFile.is_open() << " " << m_avgFileLen.is_open() << " life open " << m_lifeFile.is_open() << " pkt open " << m_rateFile.is_open() );
  
  return ( m_lifeFile.is_open() && m_avgChFile.is_open() && m_rateFile.is_open() &&
           m_roundFile.is_open() && m_duratFile.is_open() && m_dcFile.is_open() && 
           m_pktFile.is_open() && m_txpktFile.is_open()  && m_pktPerFile.is_open());
}
  
void
SensorHelper::EnableLogComponents (void)
{
  LogComponentEnableAll (LOG_PREFIX_TIME);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnable ("SensorCsma", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanErrorModel", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanInterferenceHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("SensorMac", LOG_LEVEL_ALL);
  LogComponentEnable ("SensorNetDevice", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanPhy", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanSpectrumSignalParameters", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanSpectrumValueHelper", LOG_LEVEL_ALL);
}

std::string
SensorHelper::LrWpanPhyEnumerationPrinter (LrWpanPhyEnumeration e)
{
  switch (e)
  {
    case IEEE_802_15_4_PHY_BUSY:
      return std::string ("BUSY");
    case IEEE_802_15_4_PHY_BUSY_RX:
      return std::string ("BUSY_RX");
    case IEEE_802_15_4_PHY_BUSY_TX:
      return std::string ("BUSY_TX");
    case IEEE_802_15_4_PHY_FORCE_TRX_OFF:
      return std::string ("FORCE_TRX_OFF");
    case IEEE_802_15_4_PHY_IDLE:
      return std::string ("IDLE");
    case IEEE_802_15_4_PHY_INVALID_PARAMETER:
      return std::string ("INVALID_PARAMETER");
    case IEEE_802_15_4_PHY_RX_ON:
      return std::string ("RX_ON");
    case IEEE_802_15_4_PHY_SUCCESS:
      return std::string ("SUCCESS");
    case IEEE_802_15_4_PHY_TRX_OFF:
      return std::string ("TRX_OFF");
    case IEEE_802_15_4_PHY_TX_ON:
      return std::string ("TX_ON");
    case IEEE_802_15_4_PHY_UNSUPPORTED_ATTRIBUTE:
      return std::string ("UNSUPPORTED_ATTRIBUTE");
    case IEEE_802_15_4_PHY_READ_ONLY:
      return std::string ("READ_ONLY");
    case IEEE_802_15_4_PHY_UNSPECIFIED:
      return std::string ("UNSPECIFIED");
    default:
      return std::string ("INVALID");
  }
}

std::string
SensorHelper::SensorHelperMacChnStatePrinter (SensorChnState e)
{
  switch (e)
  {
    case SENSOR_IDLE:
      return std::string ("MAC_IDLE");
    case SENSOR_CHN_ACS_FAIL:
      return std::string ("CHANNEL_ACCESS_FAILURE");
    case SENSOR_CHN_IDLE:
      return std::string ("CHANNEL_IDLE");
    case SENSOR_SET_PHY_TX_ON:
      return std::string ("SET_PHY_TX_ON");
    default:
      return std::string ("INVALID");
  }
}

void
SensorHelper::AddMobility (Ptr<LrWpanPhy> phy, Ptr<MobilityModel> m)
{
  phy->SetMobility (m);
}

NetDeviceContainer
SensorHelper::Install (NodeContainer c)
{
  NetDeviceContainer devices;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); i++)
  {
    Ptr<Node> node = *i;
    Ptr<SensorNetDevice> netDevice;
    Ptr<SensorMac> mac;
    
    switch ( m_netType )
    {
      case BATSEN:
        netDevice = CreateObject<SensorNetDevice>( CreateObject<BatsenMac> () );
        break;
        
      case LEACH_O:
      default:
        netDevice = CreateObject<SensorNetDevice>( CreateObject<LeachMac> () );
        break;
    }
    
    netDevice->SetChannel (m_channel);
    node->AddDevice (netDevice);
    netDevice->SetNode (node);
    // \todo add the capability to change short address, extended
    // address and panId. Right now they are hardcoded in SensorMac::SensorMac ()
    devices.Add (netDevice);
    
    mac = netDevice->GetMac();
    
    NS_LOG_DEBUG(mac->GetAddress() << " setting callbacks " );
    mac->SetPower( m_power );
    mac->SetSensorDeadCallback( MakeCallback(&SensorHelper::SensorDeadCallback, this) );
    mac->SetSensorDataTransmitCallback( MakeCallback(&SensorHelper::DataTransmitCallback, this) );
    
    /*
     * Don't use the Mac16Address.Allocate() method, as it will kill
     * any attempt to reuse Mac16Address addresses for a second sensor
     * network in the same simulation.
     * 
     * i.e. Manually allocate Mac16Address addresses per Helper instance
     */
    if ( m_nodeCount == 0 )
    {
      NS_LOG_DEBUG( "SensorHelper: adding sink with address 00:01" );
      
      netDevice->SetSinkStatus( true );
      netDevice->SetAddress( Mac16Address ("00:01") );
      mac->SetSinkDataRcvdCallback( MakeCallback(&SensorHelper::SinkReceptionCallback, this) );
      mac->SetSinkDCRcvdCallback( MakeCallback(&SensorHelper::SinkDCReceptionCallback, this) );
      mac->SetSensorNextRoundCallback( MakeCallback(&SensorHelper::NextRoundCallback, this) );
      
      // We assume the position of the SINK is assigned by the caller
    }
    else
    {
#ifdef MACOSX
      std::stringstream ss;
      ss << (m_nodeCount + 1);
      std::string str = ss.str();
     
      std::string address = "00:" + ((m_nodeCount < 9) ? std::string("0") : std::string("")) + str;
#else
      std::string address = "00:" + ((m_nodeCount < 9) ? std::string("0") : std::string("")) + std::to_string( m_nodeCount + 1 );
#endif
      //NS_LOG_DEBUG( "SensorHelper: adding node with address " << address );
      uint16_t addr = htons( m_nodeCount + 1 );
      Mac16Address macaddr;
      macaddr.CopyFrom( (uint8_t *) &addr );
      //NS_LOG_DEBUG( "Setting node to counter " << m_nodeCount << " 16-bit: " << addr << " MAc16: " << macaddr );
      
      netDevice->SetSinkStatus( false );
      netDevice->SetAddress( macaddr );
      mac->SetCHDataRcvdCallback( MakeCallback(&SensorHelper::ClusterHeadReceptionCallback, this) );
      mac->SetSensorForwarderSelectedCallback( MakeCallback(&SensorHelper::ForwarderSelectedCallback, this) );
      mac->SetSensorDirectSelectCallback( MakeCallback(&SensorHelper::DirectSelectCallback, this) );
    }
    
    ++m_nodeCount;
  }
  
  // The number of live member nodes is m_nodeCount - 1 to discount the sink
  m_liveNodes = m_nodeCount - 1;
  
  // Save number of member nodes for later
  --m_nodeCount;
  
  return devices;
}


Ptr<SpectrumChannel>
SensorHelper::GetChannel (void)
{
  return m_channel;
}

void
SensorHelper::SetChannel (Ptr<SpectrumChannel> channel)
{
  m_channel = channel;
}

void
SensorHelper::SetChannel (std::string channelName)
{
  Ptr<SpectrumChannel> channel = Names::Find<SpectrumChannel> (channelName);
  m_channel = channel;
}

int64_t
SensorHelper::AssignStreams (NetDeviceContainer c, int64_t stream)
{
  int64_t currentStream = stream;
  Ptr<NetDevice> netDevice;
  for (NetDeviceContainer::Iterator i = c.Begin (); i != c.End (); ++i)
  {
    netDevice = (*i);
    Ptr<SensorNetDevice> sensor = DynamicCast<SensorNetDevice> (netDevice);
    if (sensor)
    {
      currentStream += sensor->AssignStreams (currentStream);
    }
  }
  
  NS_LOG_DEBUG("SensorHelper->AssignStreams to local PRNG");
  ++currentStream;
  m_uniformRandomVariable->SetStream( currentStream++ );
  
  m_discAllocator->AssignStreams( currentStream );
  
  return (currentStream - stream);
}

void 
SensorHelper::ConfigureDataRate( double rate, uint16_t size, NetDeviceContainer cntr )
{
  m_pktSize = size;
  m_sensors.Add( cntr );
  m_alive.Add( cntr );
  
  /*
   * Rate = Pkts / sec :: 1 sec / rate = interval
   */
  m_interval = Seconds ( 1.0 / rate );
  
  Simulator::Schedule ( Seconds(0.001), &SensorHelper::SendData, this );
}

void 
SensorHelper::SetRandomPositions(NetDeviceContainer c)
{
  m_discAllocator->SetX( 50.0 );
  m_discAllocator->SetY( 50.0 );
  m_discAllocator->SetRho( 46.5 );
  
  for ( uint32_t i = 1; i < c.GetN(); ++i )
  {
    Ptr<SensorNetDevice> netDevice = c.Get(i)->GetObject<SensorNetDevice>();
    Ptr<SensorMac> mac = netDevice->GetMac();
    
    if ( mac->GetSinkStatus() )
      continue;
    
    // Setup the position of the node randomly within the RF range
    Ptr<ConstantPositionMobilityModel> senderMobility = CreateObject<ConstantPositionMobilityModel> ();

#if 0
    uint32_t xval = m_uniformRandomVariable->GetInteger( 0, 90 );
    uint32_t yval = m_uniformRandomVariable->GetInteger( 0, 90 );
    int x,y;
    
    x = (int)xval - 45;
    y = (int)yval - 45;
    
    NS_LOG_DEBUG( "Assign position X: " << x << " Y: " << y );
    senderMobility->SetPosition( Vector( x, y, 0 ) );
#endif
    
    Vector tmpPos = m_discAllocator->GetNext();
    senderMobility->SetPosition( tmpPos );
    netDevice->GetPhy()->SetMobility( senderMobility );
  }
}

void
SensorHelper::ClusterHeadReceptionCallback( Mac16Address source, uint32_t sequence )
{
  //NS_LOG_DEBUG( " \n\n --- We got CH received data here! ---");
  //NS_LOG_DEBUG( "      SRC " << source << " seq " << sequence << " \n" );
}
  
void
SensorHelper::SinkReceptionCallback( std::list<uint16_t> nodes, std::list<uint32_t> sequence )
{
  //NS_LOG_UNCOND( "\t+++ We got SINK received data here! +++");
  
  while ( nodes.size() > 0 )
  {
    uint16_t addr = nodes.front();
    uint32_t seq  = sequence.front();
    nodes.pop_front();
    sequence.pop_front();
    
    NS_LOG_DEBUG( "\t  node " << addr << " seq " << seq );
    
    uint16_t netAddr = ntohs( addr );
    //NS_LOG_UNCOND( "\t      htons " << netAddr );
    
    Mac16Address tmpAddr;
    tmpAddr.CopyFrom( (uint8_t *)&(netAddr) );
    PktMap_t::iterator pktItr = m_rndPtr->pktList.find( tmpAddr );
    
    if ( pktItr != m_rndPtr->pktList.end() )
    {
      NS_LOG_DEBUG( "\t  set pkt from " << pktItr->first << " to TRUE" );
      pktItr->second = true;
    }
    else
      NS_LOG_DEBUG( "\t\t  no pkt found for addr " << tmpAddr );
    
    TrackIterator_t itr = m_trackMap.begin();
    itr = m_trackMap.find( seq );
    
    if (itr != m_trackMap.end())
    {
      NS_LOG_DEBUG( "\tfound seq " << seq << " in m_trackMap map from " << addr );
      itr->second->received = true;
      itr->second->stop = Now();
    }
  }
}
  
void
SensorHelper::SinkDCReceptionCallback( Mac16Address dcAddr, uint32_t sequence )
{
  PktMap_t::iterator pktItr = m_rndPtr->pktList.find( dcAddr );
  
  if ( pktItr != m_rndPtr->pktList.end() )
  {
    NS_LOG_DEBUG( "\t  set pkt from " << pktItr->first << " to TRUE" );
    pktItr->second = true;
  }
  else
    NS_LOG_DEBUG( "\t\t  no pkt found for addr " << dcAddr );
  
  TrackIterator_t itr = m_trackMap.begin();
  itr = m_trackMap.find( sequence );
  
  if (itr != m_trackMap.end())
  {
    NS_LOG_DEBUG( "\tfound seq " << sequence << " in m_trackMap map from " << dcAddr );
    itr->second->received = true;
    itr->second->stop = Now();
  }
  
  // increment the number of DC nodes that we received from
  m_rndPtr->direct++;
}

void
SensorHelper::SensorDeadCallback( Mac16Address addr )
{
  static bool hitFirst = false;
  static bool hitFifty = false;
  
  m_liveNodes--;
  
  NS_LOG_DEBUG( "Node " << addr << " reported that it died at " << Now() );
  NS_LOG_DEBUG( "   " << m_liveNodes << " nodes remain " );
  
  if ( !hitFirst )
  {
    m_firstFail = Now();
    hitFirst = true;
  }
  else if ( !hitFifty && (m_liveNodes <= ( m_nodeCount / 2)) )
  {
    m_fiftyFail = Now();
    hitFifty = true;
  }
   
  if ( m_liveNodes < 1 )
  {
    NS_LOG_UNCOND( " Network Dead at " << Now() );
    //sleep(1);
    // Kill the SINK to stop all other processing
    Ptr<SensorNetDevice> dev = m_sensors.Get(0)->GetObject<SensorNetDevice>();
    Ptr<SensorMac> mac = dev->GetMac();
    mac->KillNode();
    
    // Signify the network is fully dead
    m_netAlive = false;
    
    // Assign the death of the net in seconds
    m_lastFail = Now();
    
    AnalyzeData();
  }
}

void 
SensorHelper::WhoisAlive()
{
  NetDeviceContainer::Iterator itr = m_sensors.Begin();
  while( itr != m_sensors.End() )
  {
    Ptr<SensorNetDevice> dev = (*itr)->GetObject<SensorNetDevice>();
    Ptr<SensorMac> mac = dev->GetMac();
    
    if ( mac->NodeAlive() )
      NS_LOG_UNCOND("Node " << mac->GetAddress() << " still alive");
    
    itr++;
  }
}

void 
SensorHelper::DataTransmitCallback( uint32_t seq, Mac16Address address )
{
  NS_LOG_DEBUG( "\tDataTransmitCallback for seq " << seq << " sz " << m_trackMap.size() << " @ " << Now() );
  TrackIterator_t itr = m_trackMap.begin();
  
  itr = m_trackMap.find( seq );
  
  // If we found a sequence number in this ap list already, then there
  // is an error in the system.  There can be only one!
  if (itr != m_trackMap.end())
  {
    NS_LOG_DEBUG( "\n\tERROR: seq " << seq << " already in the m_trackMap map\n" );
  }
  else
  {
    NS_LOG_DEBUG( "\t  inserting seq " << seq );
    SensorTracker_s *temp = new SensorTracker_s;
    temp->start = Now();
    
    TrackIterator_t it = m_trackMap.end();
    m_trackMap.insert( it, std::pair<uint32_t, SensorTracker_s *>( seq,temp ) );
  }
  
  if ( m_rndPtr )
    m_rndPtr->pktList.insert( m_rndPtr->pktList.begin(), std::pair<const ns3::Mac16Address, bool>(address, false) );
  else
    NS_LOG_DEBUG( "\n\n ==== CAN'T FIND ROUND " << m_round << " in ROUND MAP ==== \n\n" );

}
  
void 
SensorHelper::ForwarderSelectedCallback( Mac16Address address )
{
  NS_LOG_DEBUG( "ForwarderSelectedCallback from " << address );
  if ( m_rndPtr )
    m_rndPtr->clusters++;
  else
    NS_LOG_ERROR( "\n\n ==== CAN'T FIND ROUND " << m_round << " in ROUND MAP ==== \n\n" );  
}
  
void
SensorHelper::DirectSelectCallback( Mac16Address address )
{
  NS_LOG_DEBUG( "DirectSelectCallback from " << address );
  if ( m_rndPtr )
    m_rndPtr->direct++;
  else
    NS_LOG_ERROR( "\n\n ==== CAN'T FIND ROUND " << m_round << " in ROUND MAP ==== \n\n" );
}

void 
SensorHelper::NextRoundCallback( int round )
{
  NS_LOG_DEBUG( "\n+++ Next Round Callback - Epoch " << round << " +++\n");

  RoundIterator_t itr = m_roundMap.find( round - 1 );
  
  if ( itr != m_roundMap.end() )
  {
    Time temp;
    RoundTracker_s *l_rndPtr = itr->second;
    
    // Determine the number of cluster members for the round
    // NOTE: We're not analyzing the avg/stdev members/CH since we're not
    // evaluating LEACH, just testing it
    l_rndPtr->clusterMembers = m_liveNodes - (l_rndPtr->clusters + l_rndPtr->direct);
    
    l_rndPtr->stop = Now();
    temp = l_rndPtr->stop - l_rndPtr->start;
    NS_LOG_UNCOND("Rnd " << round << ": start " << l_rndPtr->start << " stop " << l_rndPtr->stop << " diff " << temp.GetSeconds() );
    l_rndPtr->duration = temp.GetSeconds();
    NS_LOG_DEBUG( "\tRound " << itr->first << " lasted " << l_rndPtr->duration << " w/ " << l_rndPtr->clusters << " CHs & " << l_rndPtr->direct << " DCs" );
    
    PktMap_t::iterator pktItr = l_rndPtr->pktList.begin();
    while ( pktItr != l_rndPtr->pktList.end() )
    {
      if ( pktItr->second )
        l_rndPtr->rxFrames++;
      
      l_rndPtr->txFrames++;
      
      pktItr++;
    }
    
    NS_LOG_DEBUG( "\t" << l_rndPtr->rxFrames << " rcvd " );
  }
  
  itr = m_roundMap.find( round );
  if ( itr != m_roundMap.end() )
    NS_LOG_ERROR( "\n\n ==== ALREADY HAVE ROUND " << round << " in ROUND MAP ==== \n\n" );
  else
  {
    RoundTracker_s *temp = new RoundTracker_s;
    
    RoundIterator_t it = m_roundMap.end();
    m_roundMap.insert( it, std::pair<uint32_t, RoundTracker_s *>( round,temp ) );
    m_round = round;
    m_rndPtr = temp;
    m_rndPtr->start = Now();
  }
}

/**
 * @brief Write a packet in a PCAP file
 * @param file the output file
 * @param packet the packet
 */
static void
PcapSniffSensor (Ptr<PcapFileWrapper> file, Ptr<const Packet> packet)
{
  file->Write (Simulator::Now (), packet);
}

void
SensorHelper::EnablePcapInternal (std::string prefix, Ptr<NetDevice> nd, bool promiscuous, bool explicitFilename)
{
  //NS_LOG_FUNCTION (this << prefix << nd << promiscuous << explicitFilename);
  //
  // All of the Pcap enable functions vector through here including the ones
  // that are wandering through all of devices on perhaps all of the nodes in
  // the system.
  //
  
  // In the future, if we create different NetDevice types, we will
  // have to switch on each type below and insert into the right
  // NetDevice type
  //
  Ptr<SensorNetDevice> device = nd->GetObject<SensorNetDevice> ();
  if (device == 0)
  {
    //NS_LOG_INFO ("SensorHelper::EnablePcapInternal(): Device " << device << " not of type ns3::Sensor");
    return;
  }
  
  PcapHelper pcapHelper;
  
  std::string filename;
  if (explicitFilename)
  {
    filename = prefix;
  }
  else
  {
    filename = pcapHelper.GetFilenameFromDevice (prefix, device);
  }
  
  Ptr<PcapFileWrapper> file = pcapHelper.CreateFile (filename, std::ios::out,
                                                     PcapHelper::DLT_IEEE802_15_4);
  
  if (promiscuous == true)
  {
    device->GetMac ()->TraceConnectWithoutContext ("PromiscSniffer", MakeBoundCallback (&PcapSniffSensor, file));
    
  }
  else
  {
    device->GetMac ()->TraceConnectWithoutContext ("Sniffer", MakeBoundCallback (&PcapSniffSensor, file));
  }
}

void
SensorHelper::EnableAsciiInternal ( Ptr<OutputStreamWrapper> stream,
                                    std::string prefix,
                                    Ptr<NetDevice> nd,
                                    bool explicitFilename)
{
  uint32_t nodeid = nd->GetNode ()->GetId ();
  uint32_t deviceid = nd->GetIfIndex ();
  std::ostringstream oss;
  
  Ptr<SensorNetDevice> device = nd->GetObject<SensorNetDevice> ();
  if (device == 0)
  {
    NS_LOG_INFO ("SensorHelper::EnableAsciiInternal(): Device " << device << " not of type ns3::Sensor");
    return;
  }
  
  //
  // Our default trace sinks are going to use packet printing, so we have to
  // make sure that is turned on.
  //
  Packet::EnablePrinting ();
  
  //
  // If we are not provided an OutputStreamWrapper, we are expected to create
  // one using the usual trace filename conventions and do a Hook*WithoutContext
  // since there will be one file per context and therefore the context would
  // be redundant.
  //
  if (stream == 0)
  {
    //
    // Set up an output stream object to deal with private ofstream copy
    // constructor and lifetime issues.  Let the helper decide the actual
    // name of the file given the prefix.
    //
    AsciiTraceHelper asciiTraceHelper;
    
    std::string filename;
    if (explicitFilename)
    {
      filename = prefix;
    }
    else
    {
      filename = asciiTraceHelper.GetFilenameFromDevice (prefix, device);
    }
    
    Ptr<OutputStreamWrapper> theStream = asciiTraceHelper.CreateFileStream (filename);
    
    // Ascii traces typically have "+", '-", "d", "r", and sometimes "t"
    // The Mac and Phy objects have the trace sources for these
    //
    
    asciiTraceHelper.HookDefaultReceiveSinkWithoutContext<SensorMac> (device->GetMac (), "MacRx", theStream);
    
    device->GetMac ()->TraceConnectWithoutContext ("MacTx", MakeBoundCallback (&AsciiSensorMacTransmitSinkWithoutContext, theStream));
    
    asciiTraceHelper.HookDefaultEnqueueSinkWithoutContext<SensorMac> (device->GetMac (), "MacTxEnqueue", theStream);
    asciiTraceHelper.HookDefaultDequeueSinkWithoutContext<SensorMac> (device->GetMac (), "MacTxDequeue", theStream);
    asciiTraceHelper.HookDefaultDropSinkWithoutContext<SensorMac> (device->GetMac (), "MacTxDrop", theStream);
    
    return;
  }
  
  //
  // If we are provided an OutputStreamWrapper, we are expected to use it, and
  // to provide a context.  We are free to come up with our own context if we
  // want, and use the AsciiTraceHelper Hook*WithContext functions, but for
  // compatibility and simplicity, we just use Config::Connect and let it deal
  // with the context.
  //
  // Note that we are going to use the default trace sinks provided by the
  // ascii trace helper.  There is actually no AsciiTraceHelper in sight here,
  // but the default trace sinks are actually publicly available static
  // functions that are always there waiting for just such a case.
  //
  
  
  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::Sensor/Mac/MacRx";
  device->GetMac ()->TraceConnect ("MacRx", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultReceiveSinkWithContext, stream));
  
  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::Sensor/Mac/MacTx";
  device->GetMac ()->TraceConnect ("MacTx", oss.str (), MakeBoundCallback (&AsciiSensorMacTransmitSinkWithContext, stream));
  
  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::Sensor/Mac/MacTxEnqueue";
  device->GetMac ()->TraceConnect ("MacTxEnqueue", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultEnqueueSinkWithContext, stream));
  
  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::Sensor/Mac/MacTxDequeue";
  device->GetMac ()->TraceConnect ("MacTxDequeue", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultDequeueSinkWithContext, stream));
  
  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::Sensor/Mac/MacTxDrop";
  device->GetMac ()->TraceConnect ("MacTxDrop", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultDropSinkWithContext, stream));
  
}
  
  
void 
SensorHelper::SendData()
{
  static uint32_t sequence = 0;
  static uint8_t round = 0;
  
  // Push a packet to each node that's not a sink
  for ( uint32_t i = 0; i < m_sensors.GetN(); ++i )
  {
    Ptr<SensorNetDevice>dev = m_sensors.Get(i)->GetObject<SensorNetDevice>();
    
    // If the node is the Sink for the net, or the node is dead, skip the node
    if ( dev->GetMac()->GetSinkStatus() || !(dev->GetMac()->NodeAlive()) )
      continue;
    else
    {
      //      NS_LOG_DEBUG("Node " << (i+1) << " seq " << sequence << " pushing data @ " << Now() );
      
      SensorDataRequestParams params;
      params.m_dstAddr = Mac16Address ("00:01");
      params.m_seqNumber = sequence++;
      params.m_msduHandle = round;
      params.m_pktSize = m_pktSize;
      
      dev->GetMac()->SensorDataRequest( params );
    }
  }
  
  // Now kick off another data interval event
  if ( m_netAlive && ( m_maxSeq > sequence ) )
  {
    //    NS_LOG_DEBUG( "\tseq at " << sequence << " max " << m_maxSeq << " delay " << m_interval );
    Simulator::Schedule ( m_interval, &SensorHelper::SendData, this );
  }
  else
  {
    if ( m_netAlive )
      NS_LOG_DEBUG( "\tseq at " << sequence << " max " << m_maxSeq );
    else
      NS_LOG_DEBUG( "\tseq at " << sequence << " ignored - net dead ");
  }
}

void 
SensorHelper::AnalyzeData()
{
  NS_LOG_UNCOND( " First dead @ " << m_firstFail << "\nMidP dead @ " << m_fiftyFail << "\nLast dead @ " << m_lastFail );
  
  std::streampos begin,end;
  
  RoundIterator_t itr = m_roundMap.begin();
  
  double max_round = 0.0;
  int printRounds = 0;
  
  double chs_per_round = 0.0;
  double mbr_per_round = 0.0;
  double dcs_per_round = 0.0;
  
  double total_trns_pkts = 0.0;
  double total_rcvd_pkts = 0.0;
  double pkt_trn_rate = 0.0;
  double pkt_rcv_rate = 0.0;
  
  double first_dead_time = m_firstFail.GetDouble();
  double fifty_per_time = m_fiftyFail.GetDouble();
  double last_dead_time = m_lastFail.GetDouble();
  
  //  double start_offset = 0.5;    // The networks kick off at 500ms to allow for startup processing success
  
  double total_time = 0.0;
  
  char roundStr[255];
  
  /*****************************************************************
   * Adjust The Average File
   *****************************************************************/
  begin = m_avgFileLen.tellg();
  m_avgFileLen.seekg (0, std::ios::end);
  end = m_avgFileLen.tellg();
  
  m_avgFileLen.seekg( 0, std::ios::beg );

  if ( (end-begin) == 0 )
  {
    NS_LOG_UNCOND(" m_avgChFile file size == 0" );
    // If the size of the file is 0 bytes - it's a new file
    printRounds = ( m_roundMap.size() * 1.25 );
    m_avgChFile << printRounds << std::endl;
    NS_LOG_UNCOND("\tprint round = " << printRounds );
  }
  else
  {
    m_avgFileLen.getline( roundStr, 10, '\n' );
    printRounds = atoi( roundStr );
    NS_LOG_UNCOND(" m_avgChFile file size != 0 and print round = " << printRounds << " read " << roundStr );
  }
  
  /*****************************************************************
   * Adjust The Rate File
   *****************************************************************/
  begin = m_rateFileLen.tellg();
  m_rateFileLen.seekg (0, std::ios::end);
  end = m_rateFileLen.tellg();
  
  m_rateFileLen.seekg( 0, std::ios::beg );
  
  if ( (end-begin) == 0 )
  {
    NS_LOG_UNCOND(" m_rateFile file size == 0" );
    // If the size of the file is 0 bytes - it's a new file
    printRounds = ( m_roundMap.size() * 1.25 );
    m_rateFile << printRounds << std::endl;
    NS_LOG_UNCOND("\tprint round = " << printRounds );
  }
  else
  {
    m_rateFileLen.getline( roundStr, 10, '\n' );
    printRounds = atoi( roundStr );
    NS_LOG_UNCOND(" m_rateFile file size != 0 and print round = " << printRounds << " read " << roundStr );
  }

  /*****************************************************************
   * Adjust The Round Duration Statistics File
   *****************************************************************/
  begin = m_duratFileLen.tellg();
  m_duratFileLen.seekg (0, std::ios::end);
  end = m_duratFileLen.tellg();
  
  m_duratFileLen.seekg( 0, std::ios::beg );

  if ( (end-begin) == 0 )
  {
    NS_LOG_UNCOND(" m_duratFile file size == 0" );
    // If the size of the file is 0 bytes - it's a new file
    printRounds = ( m_roundMap.size() * 1.25 );
    m_duratFile << printRounds << std::endl;
    NS_LOG_UNCOND("\tprint round = " << printRounds );
  }
  else
  {
    m_duratFileLen.getline( roundStr, 10, '\n' );
    printRounds = atoi( roundStr );
    NS_LOG_UNCOND(" m_duratFile file size != 0 and print round = " << printRounds << " read " << roundStr );
  }

  /*****************************************************************
   * Adjust The Direct Connections per Round Statistics File
   *****************************************************************/
  begin = m_dcFileLen.tellg();
  m_dcFileLen.seekg (0, std::ios::end);
  end = m_dcFileLen.tellg();
  
  m_dcFileLen.seekg( 0, std::ios::beg );

  if ( (end-begin) == 0 )
  {
    NS_LOG_UNCOND(" m_dcFile file size == 0" );
    // If the size of the file is 0 bytes - it's a new file
    printRounds = ( m_roundMap.size() * 1.25 );
    m_dcFile << printRounds << std::endl;
    NS_LOG_UNCOND("\tprint round = " << printRounds );
  }
  else
  {
    m_dcFileLen.getline( roundStr, 10, '\n' );
    printRounds = atoi( roundStr );
    NS_LOG_UNCOND(" m_dcFile file size != 0 and print round = " << printRounds << " read " << roundStr );
  }

  /*****************************************************************
   * Adjust The Packets Received per Round Statistics File
   *****************************************************************/
  begin = m_pktFileLen.tellg();
  m_pktFileLen.seekg (0, std::ios::end);
  end = m_pktFileLen.tellg();
  
  m_pktFileLen.seekg( 0, std::ios::beg );

  if ( (end-begin) == 0 )
  {
    NS_LOG_UNCOND(" m_pktFile file size == 0" );
    // If the size of the file is 0 bytes - it's a new file
    printRounds = ( m_roundMap.size() * 1.25 );
    m_pktFile << printRounds << std::endl;
    NS_LOG_UNCOND("\tprint round = " << printRounds );
  }
  else
  {
    m_pktFileLen.getline( roundStr, 10, '\n' );
    printRounds = atoi( roundStr );
    NS_LOG_UNCOND(" m_pktFile file size != 0 and print round = " << printRounds << " read " << roundStr );
  }

  /*****************************************************************
   * Adjust The Transmitted Packets per Round Statistics File
   *****************************************************************/
  begin = m_txpktFileLen.tellg();
  m_txpktFileLen.seekg (0, std::ios::end);
  end = m_txpktFileLen.tellg();
  
  m_txpktFileLen.seekg( 0, std::ios::beg );

  if ( (end-begin) == 0 )
  {
    NS_LOG_UNCOND(" m_txpktFile file size == 0" );
    // If the size of the file is 0 bytes - it's a new file
    printRounds = ( m_roundMap.size() * 1.25 );
    m_txpktFile << printRounds << std::endl;
    NS_LOG_UNCOND("\tprint round = " << printRounds );
  }
  else
  {
    m_txpktFileLen.getline( roundStr, 10, '\n' );
    printRounds = atoi( roundStr );
    NS_LOG_UNCOND(" m_txpktFile file size != 0 and print round = " << printRounds << " read " << roundStr );
  }

  /*****************************************************************
   * Adjust The % Packets Received per Round Statistics File
   *****************************************************************/
  begin = m_pktPerFileLen.tellg();
  m_pktPerFileLen.seekg (0, std::ios::end);
  end = m_pktPerFileLen.tellg();
  
  m_pktPerFileLen.seekg( 0, std::ios::beg );

  if ( (end-begin) == 0 )
  {
    NS_LOG_UNCOND(" m_pktPerFile file size == 0" );
    // If the size of the file is 0 bytes - it's a new file
    printRounds = ( m_roundMap.size() * 1.25 );
    m_pktPerFile << printRounds << std::endl;
    NS_LOG_UNCOND("\tprint round = " << printRounds );
  }
  else
  {
    m_pktPerFileLen.getline( roundStr, 10, '\n' );
    printRounds = atoi( roundStr );
    NS_LOG_UNCOND(" m_pktPerFile file size != 0 and print round = " << printRounds << " read " << roundStr );
  }

  /*****************************************************************
   * Dump the data
   *****************************************************************/
  
  // Jump to the end of the file to start writing the CH count array
  m_avgChFile.seekp(0, std::ios::end);
  m_rateFile.seekp(0, std::ios::end);
  m_duratFile.seekp(0, std::ios::end);
  m_dcFile.seekp(0, std::ios::end);
  m_pktFile.seekp(0, std::ios::end);
  m_txpktFile.seekp(0, std::ios::end);
  m_pktPerFile.seekp(0, std::ios::end);
  m_roundFile.seekp(0, std::ios::end);
  unsigned rndCnt = 1;
  for ( ; itr != m_roundMap.end(); ++itr )
  {
    RoundTracker_s *ptr = itr->second;
    
    if ( ptr->duration > 0 ) {
      if ( itr->first > max_round )
        max_round = itr->first;
      
      NS_LOG_UNCOND("\tCurrent Max Round " << max_round << " RX Pkts " << ptr->rxFrames << " TX Pkts " << ptr->pktList.size() );
      
      dcs_per_round += ptr->direct;
      chs_per_round += ptr->clusters;
      mbr_per_round += ptr->clusterMembers;
      
      //    if ( ptr->direct + ptr->clusters + ptr->clusterMembers != m_nodeCount )
      //      NS_LOG_UNCOND( "*** ERROR: round " << itr->first << " CHs " << ptr->clusters << " CH-Ms " << ptr->clusterMembers << " and DCs " << ptr->direct << " == " << ( ptr->direct + ptr->clusters + ptr->clusterMembers ) << " nodes " );
      
      total_trns_pkts += ptr->rxFrames;
      total_rcvd_pkts += ptr->pktList.size();
      
      total_time += ptr->duration;
      
      NS_LOG_UNCOND( " Round: " << rndCnt << " Pkt rate: \n\t frames " << ptr->rxFrames << "\n\t duration " << ptr->duration );
      double pktrate = (ptr->rxFrames / ptr->duration);
      NS_LOG_UNCOND( "\t rate: " << pktrate << "\n\t Time_t: " << total_time );
      
      m_rateFile << pktrate << ", ";
      m_avgChFile << ptr->clusters << ", ";
      m_duratFile << ptr->duration << ", ";
      m_dcFile << ptr->direct << ", ";
      m_pktFile << ptr->rxFrames << ", ";
      m_txpktFile << ptr->txFrames << ", ";
      m_pktPerFile << (ptr->txFrames > 0 ? (ptr->rxFrames/ptr->txFrames) : 0) << ", ";
      
      ++rndCnt;
    }
    else
      NS_LOG_UNCOND("Invalid slot - 0 duration");
    
    --printRounds;
  }
  
  NS_LOG_UNCOND(" Avg rnd duration: " << ( total_time / rndCnt) );
  
  m_roundFile << rndCnt << ", " << ( total_time / rndCnt) << ", " << (last_dead_time - total_time) << "\n";
  
  for ( ;printRounds > 0; --printRounds )
  {
    if ( printRounds > 1 )
    {
      m_avgChFile << "0, ";
      m_rateFile << "0, ";
      m_duratFile << "0, ";
      m_dcFile << "0, ";
      m_pktFile << "0, ";
      m_txpktFile << "0, ";
      m_pktPerFile << "0.0, ";
    }
  }
  
  // Push an EOL with the last 0 just in case we pushed the wrong amount of data
  m_avgChFile << "0" << std::endl;
  m_rateFile << "0" << std::endl;
  m_duratFile << "0" << std::endl;
  m_dcFile << "0" << std::endl;
  m_pktFile << "0" << std::endl;
  m_txpktFile << "0" << std::endl;
  m_pktPerFile << "0" << std::endl;
  
  pkt_trn_rate = total_trns_pkts / total_time;
  pkt_rcv_rate = total_rcvd_pkts / total_time;

  m_lifeFile.seekp(0, std::ios::end);
  m_lifeFile << first_dead_time << ", " << fifty_per_time << ", " << last_dead_time << ", " << total_time << std::endl;
  

  NS_LOG_UNCOND( "\n\ttotal time " << total_time << "sec\n\tTX Rate " << pkt_trn_rate << "pkts/sec\n\tRX Rate " << pkt_rcv_rate << "pkts/sec" );
  
  m_avgChFile.close();
  m_lifeFile.close();
  m_rateFile.close();
  m_duratFile.close();
  m_dcFile.close();
  m_pktFile.close();
  m_txpktFile.close();
  m_pktPerFile.close();
  m_roundFile.close();
}


} // namespace ns3

