/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Rochester Institute of Technology
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
 *  Author: Nelson Powell <nhp8080@rit.edu>
 */

#include "batsen-mac.h"
#include "batsen-mac-header.h"
#include "mac-trailer.h"
#include "sensor-csma.h"
#include <ns3/output-stream-wrapper.h>
#include <ns3/lr-wpan-sinr-tag.h>
#include <ns3/simulator.h>
#include <ns3/log.h>
#include <ns3/uinteger.h>
#include <ns3/node.h>
#include <ns3/packet.h>
#include <ns3/random-variable-stream.h>
#include <ns3/double.h>
#include <ns3/integer.h>
#include <arpa/inet.h>
#include <iostream>
#include <math.h>

#include <string>
#include <iomanip>
#include <sstream>
#include <stdint.h>

namespace ns3 {
  
NS_LOG_COMPONENT_DEFINE ("BatsenMac");

NS_OBJECT_ENSURE_REGISTERED (BatsenMac);

//  Minimum overhead of BATSEN MAC Frame
//  DST     2
//  SRC     2
//  Type    1
//  Len     1
//  Seq#    2
const uint32_t BatsenMac::aMinMPDUOverhead = 10;
  
const Mac16Address SinkAddr("00:01");

  //#define DUMP_PEERS  1
#undef DUMP_PEERS
  
  //#define MST_LIST_DBG 1
#undef MST_LIST_DBG
  
#define POWER_REMAINING ((uint8_t)((m_totalSystemPower / m_maxSystemPower) * 255))

/*****************************************************************
 *  BATSEN MAC Methods
 *****************************************************************/
  
  // TODO: Multiple the OGM periods by Nx the number of frames required
  // to transmit all created OGMs
#define SLOT_LENGTH           0.007

#define NUM_FRAMES            ceil((2 * m_maxNumNodes)/MAX_PAYLD_LEN)
  
#define NULL_OGM_PERIOD       ( 1.25 * m_maxNumNodes * SLOT_LENGTH )

#define RXPWR_OGM_PERIOD      ( 1.5 * m_maxNumNodes * SLOT_LENGTH * m_multiplier )

#define OGM_ANNOUNCE_PERIOD   ( 1.5 * m_maxNumNodes * SLOT_LENGTH * m_multiplier )
  
#define MULTI_SLOT_LEN        ( SLOT_LENGTH * m_multiplier )
  
#define PERCENT_RTRS          0.05
  
TypeId
BatsenMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::BatsenMac")
  .SetParent<Object> ()
  .SetGroupName ("Sensor")
  .AddConstructor<BatsenMac> ()
  .AddAttribute ("NumNodes", "Number of nodes set for this sensor network.",
                 IntegerValue (100),
                 MakeIntegerAccessor (&BatsenMac::m_maxNumNodes),
                 MakeIntegerChecker <int>())
  .AddAttribute ("NumClusters", "Target number of clusters for network.",
                 IntegerValue (5),
                 MakeIntegerAccessor (&BatsenMac::m_numClusters),
                 MakeIntegerChecker <int>())
  ;
  return tid;
}

BatsenMac::BatsenMac () :
  SensorMac(),
  m_multiplier( 1.0 ),
  m_ogmTimer( Timer::CANCEL_ON_DESTROY ),
  m_startSleep( Timer::CANCEL_ON_DESTROY ),
  m_iAmForwarder( false ),
  m_maxSystemPower( 0.01 ),
  m_currentRound( 0 ),
  m_sequence( 1 ),
  m_pktHandle( 0 ),
  m_curNodeCount( 0 ),
  m_pwrToSink( HIGH_POWER ),
  m_pwrToFwrd( HIGH_POWER ),
  m_resendPwr( true ),
  m_justSentData( false ),
  m_nextHop( "00:01" ),
  m_lastSFrcvd( 0x00 )
{
  NS_LOG_DEBUG ("Creating BatsenMac");
  
  m_Lpeers.clear();
  m_Mpeers.clear();
  m_Hpeers.clear();
  m_masterList.clear();
  m_txQueue.clear();
  m_rxPwrList.clear();
  
  m_electList.Clear();
  
  // First set the state to a known value, call ChangeMacState to fire trace source.
  m_sensorMacState = SENSOR_IDLE;
  
  ChangeMacState (SENSOR_IDLE);
  ChangeFsmState (BATSEN_INIT);
  
  m_Address = Mac16Address( "00:00" );
  
  // Need the PRNG for the Threshold detection algorithm
  m_random = CreateObject<UniformRandomVariable>();

  // Convert the base power level (in dBm) to mW power
  m_mWpwr = pow(10., (0.0 - 30) / 10) / 1000.0;
  NS_LOG_DEBUG( " POWER: " << m_mWpwr << "mW == 0.0 dBm" );
}

BatsenMac::~BatsenMac ()
{
  m_ogmTimer.Cancel();
  m_startSleep.Cancel();
}

void
BatsenMac::DoDispose ()
{
  m_systemEnabled = false;
  
  if (m_csmaCa != 0)
  {
    m_csmaCa->Dispose ();
    m_csmaCa = 0;
  }
  m_txPkt = 0;
  for (uint32_t i = 0; i < m_txQueue.size (); i++)
  {
    m_txQueue[i]->txQPkt = 0;
    delete m_txQueue[i];
  }
  m_txQueue.clear ();
  
//  NS_LOG_UNCOND( m_Address << " DoDispose erasing pkt and txQ" );
  
  m_phy = 0;
  
  m_sensorDataIndicationCallback = MakeNullCallback< void, SensorDataIndicationParams, Ptr<Packet> > ();
  m_sensorDataConfirmCallback = MakeNullCallback< void, SensorDataConfirmParams > ();
  
//  NS_LOG_UNCOND( m_Address << " DoDispose for parent" );
  
  SensorMac::DoDispose();
}

int64_t
BatsenMac::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  SensorMac::AssignStreams (stream);
  
  return 1;
}
  
void
BatsenMac::SetAddress (Mac16Address address)
{
  m_Address = address;

  // If the address is 0x0001, then setup a SINK FSM
  if ( m_Address == Mac16Address ("00:01") )
    m_iAmSink = true;
  else
    m_iAmSink = false;
}

void
BatsenMac::KickOffFSM()
{
  NS_LOG_FUNCTION (this << m_iAmSink );
  
  m_txQueue.clear();
  NS_LOG_DEBUG("TXQ sz = " << m_txQueue.size() << " isEmpty " << m_txQueue.empty() );
  
  SetPower( HIGH_POWER );
  SetChannel( CENTER_FREQ );
  m_phy->SetAddr( m_Address );
  m_nodeSelf.SetAddress( m_Address );
  m_rcvPwrList.SetAddress( m_Address );
  
  m_multiplier = ceil( ( 2.0 * m_maxNumNodes ) / MAX_PAYLD_LEN );
  
  NS_LOG_DEBUG( "\t Starting sys pwr " << m_totalSystemPower << " nodes " << m_maxNumNodes << " multiplier " << m_multiplier );
  m_maxSystemPower = m_totalSystemPower;
  
  // Update the % power remaining
  double tmpPower = m_totalSystemPower / m_maxSystemPower;
  m_nodeSelf.SetPrcntPwrRemain( tmpPower );
  
  Time waitTime = Seconds( 0.5 );
  m_ogmTimeout = Simulator::Schedule (waitTime, &BatsenMac::StartNullOgmCollection, this);
}
 
void
BatsenMac::NodeJustDied()
{
  ChangeFsmState(BATSEN_NODE_DEAD);
  // Disable timers and FSM
  m_phy->Disable();
}

void
BatsenMac::ChangeFsmState(BatsenFsmState state)
{
  m_batsenFsmState = state;
}
  
void
BatsenMac::SensorDataRequest (SensorDataRequestParams params, Ptr<Packet> p)
{
  //NS_LOG_DEBUG ("Node " << m_Address << " queing existing data packet " << p);
  
  SensorDataConfirmParams confirmParams;
  confirmParams.m_msduHandle = params.m_msduHandle;
  
  /*
   * We need to convert the blank packet into a payload within the
   * standard DataPayload_s data structure, to include the sequence
   * number from the params.
   */
  DataMessage msg;
  p->CopyData( (uint8_t *)&msg, p->GetSize() );
  
  msg.numberBytes = p->GetSize();
  msg.sequence = params.m_seqNumber;
  
  // Clear out old data
  p->RemoveAtStart( p->GetSize() );
  
  BatsenMacHeader macHdr (DAT_FRAME, m_sequence);
  ++m_sequence;
  
  if (p->GetSize () > LrWpanPhy::aMaxPhyPacketSize - aMinMPDUOverhead)
  {
    // Note, this is just testing maximum theoretical frame size per the spec
    // The frame could still be too large once headers are put on
    // in which case the phy will reject it instead
    NS_LOG_ERROR (this << " packet too big: " << p->GetSize ());
    confirmParams.m_status = SENSOR_FRAME_TOO_LONG;
    if (!m_sensorDataConfirmCallback.IsNull ())
    {
      m_sensorDataConfirmCallback (confirmParams);
    }
    return;
  }
  
  macHdr.SetSrcAddr( m_Address );
  macHdr.SetDstAddr( m_nextHop );

  p->AddHeader (macHdr);
  
  MacFrameTrailer macTrailer;
  // Calculate FCS if the global attribute ChecksumEnable is set.
  if (Node::ChecksumEnabled ())
  {
    macTrailer.EnableFcs (true);
    macTrailer.SetFcs (p);
  }
  p->AddTrailer (macTrailer);
  
  // Trace callback that MAC received a frame for transmission
  m_macTxEnqueueTrace(p);
  
  // Queue the packet into the TX Queue
  TxQueueElement *txQElement = new TxQueueElement;
  txQElement->txQMsduHandle = params.m_msduHandle;
  txQElement->txQPkt = p;
  m_dataQueue.push_back (txQElement);
}
  
void
BatsenMac::SensorDataRequest (SensorDataRequestParams params )
{
  if ( m_systemEnabled )
  {
    //NS_LOG_DEBUG ("Node " << m_Address << " queing new packet " << params.m_seqNumber << " @ " << Now() );
    
    SensorDataConfirmParams confirmParams;
    confirmParams.m_msduHandle = params.m_msduHandle;
    
    /*
     * We need to convert the blank packet into a payload within the
     * standard DataPayload_s data structure, to include the sequence
     * number from the params.
     */
    DataMessage msg;
    msg.numberBytes = params.m_pktSize;
    msg.sequence = params.m_seqNumber;
    
    // Create the packet
    Ptr<Packet> p = CreatePacketWithData( DAT_FRAME,
                                          m_Address,
                                          m_nextHop,
                                          params.m_seqNumber,
                                          (uint8_t const*)&msg,
                                          msg.numberBytes );
    
    if (p->GetSize () > LrWpanPhy::aMaxPhyPacketSize - aMinMPDUOverhead)
    {
      // Note, this is just testing maximum theoretical frame size per the spec
      // The frame could still be too large once headers are put on
      // in which case the phy will reject it instead
      NS_LOG_ERROR (this << " packet too big: " << p->GetSize ());
      confirmParams.m_status = SENSOR_FRAME_TOO_LONG;
      if (!m_sensorDataConfirmCallback.IsNull ())
      {
        m_sensorDataConfirmCallback (confirmParams);
      }
      return;
    }
    
    // Trace callback that MAC received a frame for transmission
    m_macTxEnqueueTrace(p);
    
    // Queue the packet into the TX Queue
    TxQueueElement *txQElement = new TxQueueElement;
    txQElement->txQMsduHandle = params.m_msduHandle;
    txQElement->txQPkt = p;
    m_dataQueue.push_back (txQElement);
  }
  else
    NS_LOG_DEBUG ("Dead Node " << m_Address << " ignoring packet @ " << Now() );
}
  
void
BatsenMac::PdDataIndication( uint32_t psduLength, Ptr<Packet> p, uint8_t lqi )
{
  NS_ASSERT (m_sensorMacState == SENSOR_IDLE || m_sensorMacState == SENSOR_ACK_PENDING || m_sensorMacState == SENSOR_CSMA);
  
  LrWpanSinrTag stag( 0.0 );
  p->PeekPacketTag( stag );
  
  NS_LOG_DEBUG ( m_Address << " RECEIVE FRAME: @ " << Now() << " Len " << psduLength << " LQI " << (int)lqi << " SINR " << (double)stag.Get() );
  
  Ptr<Packet> originalPkt = p->Copy (); // because we will strip headers
  
  // Promiscuous mode receiver
  m_promiscSnifferTrace (originalPkt);
  
  // Check the MAC Trailer for a valid FCS
  MacFrameTrailer receivedMacTrailer;
  p->RemoveTrailer (receivedMacTrailer);
  if (Node::ChecksumEnabled ())
  {
    receivedMacTrailer.EnableFcs (true);
  }
  
  // level 1 filtering
  if (!receivedMacTrailer.CheckFcs(p))
  {
    m_macRxDropTrace( originalPkt );
  }
  else
  {
    //level 2 frame filtering
    m_macRxTrace (originalPkt);
    
    if ( m_iAmSink )
      ProcessSinkReception( psduLength, p, lqi );
    else
      ProcessNodeReception( psduLength, p, lqi );
  }
  
  ChangeMacState (SENSOR_IDLE);
}

void
BatsenMac::PlmeCcaConfirm (LrWpanPhyEnumeration status)
{
  NS_LOG_FUNCTION (this << status);
  // Direct this call through the csmaCa object
  m_csmaCa->PlmeCcaConfirm (status);
}

void
BatsenMac::PlmeEdConfirm (LrWpanPhyEnumeration status, uint8_t energyLevel)
{
  NS_LOG_FUNCTION (this << status << energyLevel);
}

void
BatsenMac::PdDataConfirm (LrWpanPhyEnumeration status)
{
  NS_LOG_DEBUG ( m_Address << " TRANSMISSION status " << g_phy_string[status] << " with state " << g_chan_state[m_sensorMacState] << " m_txPkt " << m_txPkt << " txq sz " << m_txQueue.size() );
  
  if ( m_txPkt )
  {
    BatsenMacHeader macHdr;
    m_txPkt->PeekHeader (macHdr);
    
    //NS_ASSERT (m_sensorMacState == SENSOR_SENDING);
    
    if (status == IEEE_802_15_4_PHY_SUCCESS)
    {
      m_macTxOkTrace (m_txPkt);
      // remove the copy of the packet that was just sent
      if (!m_sensorDataConfirmCallback.IsNull () ) //&& (m_txQueue.size () > 0) )
      {
        SensorDataConfirmParams confirmParams;
        // NS_ASSERT_MSG (m_txQueue.size () > 0, "TxQsize = 0");
        //TxQueueElement *txQElement = m_txQueue.front ();
        //confirmParams.m_msduHandle = txQElement->txQMsduHandle;
        confirmParams.m_status = SENSOR_SUCCESS;
        m_sensorDataConfirmCallback (confirmParams);
      }
      
      //      NS_LOG_DEBUG( " Xcvr - SUCCESS: popping packet " );
      // JUST DONE:  m_txPkt = 0;
      RemoveFirstTxQElement ();
      
      if ( m_justSentData ) {
        NS_ASSERT_MSG( m_txPkt == 0, "m_txPkt != 0 but going to sleep");
        BackToSleep();
      }
      else
      {
        if ( m_txPkt == 0 )
          NS_LOG_DEBUG( m_Address << " No pkt after TX complete");
      }
    }
    else if (status == IEEE_802_15_4_PHY_UNSPECIFIED)
    {
      NS_ASSERT_MSG (m_txQueue.size () > 0, "TxQsize = 0");
      //if ( m_txQueue.size() > 0 )
      {
        //TxQueueElement *txQElement = m_txQueue.front ();
        //m_macTxDropTrace (txQElement->txQPkt);
        m_macTxDropTrace ( m_txPkt );
        if (!m_sensorDataConfirmCallback.IsNull ())
        {
          SensorDataConfirmParams confirmParams;
          //confirmParams.m_msduHandle = txQElement->txQMsduHandle;
          confirmParams.m_status = SENSOR_FRAME_TOO_LONG;
          m_sensorDataConfirmCallback (confirmParams);
        }
        
        NS_LOG_UNCOND( " Xcvr - UNSPECIFIED: popping packet " );
        RemoveFirstTxQElement ();
      }
      //else
      //  NS_LOG_DEBUG("UNKNOWN loss of frame @ PdDataConfirm callback: node " << m_Address << " lstPkt " << m_txPkt );
    }
    else if ( status == IEEE_802_15_4_PHY_TX_FAIL )
    {
      NS_LOG_DEBUG( "***************************\n" << "\nTX FAIL\n" << "***************************\n" );
    }
    else
    {
      // Something went really wrong. The PHY is not in the correct state for
      // data transmission.
      NS_FATAL_ERROR (" Node " << m_Address << " transmission attempt failed with PHY status " << status);
      NS_LOG_DEBUG (" Node " << m_Address << " transmission attempt failed with PHY status " << status);
    }
  }
  
  Time spawn = Now() + Seconds( 0.000005 );
  NS_LOG_DEBUG( "\twait to go Idle till Now( " << (Now() + Seconds( 0.000010 )) << ") " );
  m_setMacState.Cancel ();
  m_setMacState = Simulator::ScheduleNow( &BatsenMac::SetSensorMacState, this, SENSOR_IDLE);
}
  
void
BatsenMac::StartNullOgmCollection(void)
{
  if ( m_systemEnabled && m_iAmSink )
  {
    SuperFrameMessage msg;
    msg.m_roundNum = m_currentRound;
    msg.m_ogmFlags = NULL_OGM_COLLECT;

    Ptr<Packet> p = CreatePacketWithData(SUPER_FRM,
                                         "00:01",
                                         "FF:FF",
                                         m_sequence,
                                         (uint8_t *)&msg,
                                         sizeof(SuperFrameMessage) );
    ++m_sequence;
    EnqueTxPacket(p);
    
    NS_LOG_DEBUG( "\t Sink Transmitting Super Frame @ " << Now() );
	m_forceTxEvent.Cancel();
    m_forceTxEvent = Simulator::ScheduleNow ( &BatsenMac::SetSensorMacState, this, SENSOR_CSMA);
    
    // Let everyone exchange OGMs, and then we can tell them to elect
    Time waitTime = Seconds( NULL_OGM_PERIOD + SLOT_LENGTH );
    m_ogmTimeout = Simulator::Schedule (waitTime, &BatsenMac::SinkSpawnPeriodicOgm, this);
    
    NS_LOG_DEBUG("\t\tCalling SinkSpawnPeriodicOgm @ " << Now() + waitTime );
    
    NS_LOG_DEBUG("\n\t\t COLLECTING NULL OGMs\n");
  }
}
  
void
BatsenMac::SinkSpawnPeriodicOgm(void)
{
  if ( m_batsenFsmState == BATSEN_INIT )
  {
    NS_LOG_DEBUG( m_Address << " SINK RX PWR Kick @ " << Now() << " in state " << batsenFsmString[m_batsenFsmState] );
    
    RxPwrOgmXmitStage();
    
    SuperFrameMessage msg;
    msg.m_roundNum = m_currentRound;
    msg.m_numNodes = m_masterList.size();
    msg.m_forwarders = ceil( (double)m_masterList.size() * 0.2 );
    msg.m_ogmFlags = RXPWR_OGM_COLLECT;
    
    // Update the current node count in case a new node is added late
    m_curNodeCount = m_masterList.size();
    
    NS_LOG_DEBUG("\t Super: ML sz " << m_masterList.size() << " @ 20% = " << ( (double)m_masterList.size() * 0.2 ) << " or " << ceil( (double)m_masterList.size() * 0.2 ) << " round " << m_currentRound << " flags " << msg.m_ogmFlags );
    NS_LOG_DEBUG("\t Message: " << BufferToString( (uint8_t *)&msg, sizeof(SuperFrameMessage) ) );
    
    Ptr<Packet> p = CreatePacketWithData(SUPER_FRM,
                                         "00:01",
                                         "FF:FF",
                                         m_sequence,
                                         (uint8_t *)&msg,
                                         sizeof(SuperFrameMessage) );
    ++m_sequence;
    EnqueTxPacket(p);
    
    NS_LOG_DEBUG( "\t Sink Transmitting Super Frame @ " << Now() );
	m_forceTxEvent.Cancel();
    m_forceTxEvent = Simulator::ScheduleNow ( &BatsenMac::SetSensorMacState, this, SENSOR_CSMA);
    Time waitTime = Seconds( 0.0 );
    
    NS_LOG_DEBUG("\n\t\tCOLLECTING RX PWR OGMs\n");
    NS_LOG_DEBUG("\t Sink in INIT: wait " << Seconds((RXPWR_OGM_PERIOD) + 0.010) );
    ChangeFsmState( BATSEN_OGM );
    waitTime = Seconds( RXPWR_OGM_PERIOD + MULTI_SLOT_LEN + 0.014 );
    
    // Periodically tell nodes to re-elect forwarders - Add 10ms slop
    // to ensure that the Sink transmits well after all nodes wake up
    m_ogmTimeout = Simulator::Schedule (waitTime, &BatsenMac::SinkSpawnPeriodicOgm, this);
    
    NS_LOG_DEBUG("\t   SINK Calling SinkSpawnPeriodicOgm again at " << (waitTime + Now()) );
  }
  else if ( m_batsenFsmState == BATSEN_OGM )
  {
    NS_LOG_DEBUG( m_Address << " SINK RTR and DATA @ " << Now() << " in state " << batsenFsmString[m_batsenFsmState] );

    SuperFrameMessage msg;
    msg.m_roundNum = m_currentRound;
    msg.m_numNodes = m_masterList.size();
    msg.m_forwarders = ceil( (double)m_masterList.size() * 0.2 );
    msg.m_ogmFlags = PERIODIC_OGM;
    
    // Update the current node count in case a new node is added late
    m_curNodeCount = m_masterList.size();
    
    NS_LOG_DEBUG("\t Super: ML sz " << m_masterList.size() << " @ 20% = " << ( (double)m_masterList.size() * 0.2 ) << " or " << ceil( (double)m_masterList.size() * 0.2 ) << " round " << m_currentRound << " flags " << msg.m_ogmFlags );
    NS_LOG_DEBUG("\t Message: " << BufferToString( (uint8_t *)&msg, sizeof(SuperFrameMessage) ) );
    
    Ptr<Packet> p = CreatePacketWithData(SUPER_FRM,
                                    "00:01",
                                    "FF:FF",
                                    m_sequence,
                                    (uint8_t *)&msg,
                                    sizeof(SuperFrameMessage) );
    ++m_sequence;
    EnqueTxPacket(p);
    
    NS_LOG_DEBUG( "\t Sink Transmitting Super Frame @ " << Now() );
	m_forceTxEvent.Cancel();
    m_forceTxEvent = Simulator::ScheduleNow ( &BatsenMac::SetSensorMacState, this, SENSOR_CSMA);
    Time waitTime = Seconds( 0.0 );
    
    NS_LOG_DEBUG("\n\t\tCOLLECTING RTR SEL OGMs and DATA\n");
    NS_LOG_DEBUG("\t Sink in PERIODIC: wait " << Seconds( OGM_ANNOUNCE_PERIOD + NULL_OGM_PERIOD + 0.010 ) );
    // Callback the start of the next round to the SensorHelper
    if ( !m_sensorNextRoundCallback.IsNull() )
      m_sensorNextRoundCallback( m_currentRound );

    ++m_currentRound;
    NS_LOG_DEBUG("\t\tNext round " << m_currentRound << " in " << waitTime << " or @ " << (Now() + waitTime) );
    waitTime = Seconds( OGM_ANNOUNCE_PERIOD + NULL_OGM_PERIOD + 0.010 );
    
    // Periodically tell nodes to re-elect forwarders - Add 10ms slop
    // to ensure that the Sink transmits well after all nodes wake up
    m_ogmTimeout = Simulator::Schedule (waitTime, &BatsenMac::SinkSpawnPeriodicOgm, this);
    
    NS_LOG_DEBUG("\t   SINK Calling SinkSpawnPeriodicOgm again at " << (waitTime + Now()) );
  }
  else
    NS_LOG_DEBUG( "\n" << m_Address << " SINK INVALID STATE @ " << Now() << " in state " << batsenFsmString[m_batsenFsmState] << "\n" );
}

void
BatsenMac::NullOgmXmitStage(void)
{
  if ( m_systemEnabled && !m_iAmSink )
  {
    NS_LOG_DEBUG( "\t" << m_Address << " NullOgmXmitStage @ " << Now() );
    double msTillTx = NULL_OGM_PERIOD;
    m_ogmTimer.Cancel();
    m_ogmTimer.SetFunction( &BatsenMac::NullOgmPeriodComplete, this );
    m_ogmTimer.Schedule( Seconds( msTillTx ) );
    
    NS_LOG_DEBUG("\t\twait for NullOgmPeriodComplete for " << (msTillTx) << "s or till " << (Now() + Seconds( msTillTx )));
    
    // Now we schedule the TX of a NodeOGM
    OgmMessage msg;
    msg.txPwrLvl = HPWR;
    msg.pwrPrcnt = POWER_REMAINING;
    
    Ptr<Packet> p = CreatePacketWithData(NULL_OGM,
                                         m_Address,
                                         "FF:FF",
                                         m_sequence,
                                         (uint8_t *)&msg,
                                         sizeof(OgmMessage) );
    
    NS_LOG_DEBUG( "\t" << m_Address << " create Init OGM frame " << p << " with seq " << m_sequence << " sz " << sizeof(OgmMessage) );
    
    ++m_sequence;
    EnqueTxPacket(p);
  }
}

void
BatsenMac::RxPwrOgmXmitStage(void)
{
//  if ( ( m_systemEnabled && m_resendPwr ) || m_iAmSink )
  if ( m_systemEnabled && m_resendPwr )
  {
    NS_LOG_DEBUG( "\t" << m_Address << " RxPwrOgmXmitStage @ " << Now() );

    // Move to the power level required to reach the sink
    SetPower( HIGH_POWER );

    OgmBuilder OGM;
    BNodeList::iterator itr;
    BNodeList tmpList;
    
    OGM.SetHeaderData( HPWR, POWER_REMAINING );

    NS_LOG_DEBUG("\t  " << m_Address << " creating Rx Pwr OGM lists");

    // Collect up the LoW power peers
    NS_LOG_DEBUG("\t\t\tLpwr rcvrs sz " << m_rcvPwrList.GetPwrList( LPWR ).size() );
    if ( m_rcvPwrList.GetPwrList( LPWR ).size() > 0 ) {
      tmpList = m_rcvPwrList.GetPwrList( LPWR );
      itr = tmpList.begin();

      OGM.AddAddressTlv( LPWR_LIST );
      while( itr != tmpList.end() )
      {
        // If it doesn't fit, queue a packet and restart
        if ( !OGM.AddAddress( (*itr) ) )
        {
          Ptr<Packet> p = CreatePacketWithData( RXPWR_OGM,
                                                m_Address,
                                                "FF:FF",
                                                m_sequence,
                                                OGM.GetDataPtr(),
                                                OGM.GetDataSize() );
          ++m_sequence;
          EnqueTxPacket(p);
          
          OGM.Clear();
          OGM.AddAddressTlv( LPWR_LIST );

          NS_LOG_DEBUG( " \t\t\t " << m_Address << " CREATE ANOTHER FRAME pkt " << p  );
          // This cannot fail !!!
          NS_ASSERT( OGM.AddAddress( (*itr) ) );
        }
        
        //NS_LOG_DEBUG("\t\t\t  add " << (*itr) << " from LPWR peers");
        
        itr++;
      }
    }
    
    // Collect up the MEDIUM power peers
    NS_LOG_DEBUG("\t\t\tMpwr rcvrs sz " << m_rcvPwrList.GetPwrList( MPWR ).size() );
    if ( m_rcvPwrList.GetPwrList( MPWR ).size() > 0 ) {
      tmpList = m_rcvPwrList.GetPwrList( MPWR );
      itr = tmpList.begin();
      
      OGM.AddAddressTlv( MPWR_LIST );
      while( itr != tmpList.end() )
      {
        // If it doesn't fit, queue a packet and restart
        if ( !OGM.AddAddress( (*itr) ) )
        {
          Ptr<Packet> p = CreatePacketWithData( RXPWR_OGM,
                                                m_Address,
                                                "FF:FF",
                                                m_sequence,
                                                OGM.GetDataPtr(),
                                                OGM.GetDataSize() );
          ++m_sequence;
          EnqueTxPacket(p);
          
          OGM.Clear();
          OGM.AddAddressTlv( MPWR_LIST );

          NS_LOG_DEBUG( " \t\t\t " << m_Address << " CREATE ANOTHER FRAME pkt " << p  );
          // This cannot fail !!!
          NS_ASSERT( OGM.AddAddress( (*itr) ) );
        }
        
        //NS_LOG_DEBUG("\t\t\t  add " << (*itr) << " from MPWR peers");
        
        itr++;
      }
    }
      
    // Collect up the medium power peers
    NS_LOG_DEBUG("\t\t\tHpwr rcvrs sz " << m_rcvPwrList.GetPwrList( HPWR ).size() );
    
    if ( m_rcvPwrList.GetPwrList( HPWR ).size() > 0 ) {
      tmpList = m_rcvPwrList.GetPwrList( HPWR );
      itr = tmpList.begin();

      OGM.AddAddressTlv( HPWR_LIST );
      while( itr != tmpList.end() )
      {
        // If it doesn't fit, queue a packet and restart
        if ( !OGM.AddAddress( (*itr) ) )
        {
          Ptr<Packet> p = CreatePacketWithData( RXPWR_OGM,
                                                m_Address,
                                                "FF:FF",
                                                m_sequence,
                                                OGM.GetDataPtr(),
                                                OGM.GetDataSize() );
          ++m_sequence;
          EnqueTxPacket(p);
          
          OGM.Clear();
          OGM.AddAddressTlv( HPWR_LIST );

          NS_LOG_DEBUG( " \t\t\t " << m_Address << " CREATE ANOTHER FRAME pkt " << p );
          // This cannot fail !!!
          NS_ASSERT( OGM.AddAddress( (*itr) ) );
        }
        
        //NS_LOG_DEBUG("\t\t\t  add " << (*itr) << " from HPWR peers");
        
        itr++;
      }
    }
    
    // If there is a potential packet remaining, add it to the queue
    if ( OGM.GetDataSize() > sizeof(OgmMessage) )
    {
      Ptr<Packet> p = CreatePacketWithData( RXPWR_OGM,
                                            m_Address,
                                            "FF:FF",
                                            m_sequence,
                                            OGM.GetDataPtr(),
                                            OGM.GetDataSize() );
      
      NS_LOG_DEBUG("\t  " << m_Address << " Create Pkt (sz " << (unsigned)OGM.GetDataSize() << ") -> " << BufferToString(OGM.GetDataPtr(), OGM.GetDataSize()) << " pkt " << p );


      ++m_sequence;
      EnqueTxPacket(p);
    }
  
    m_resendPwr = false;
    
    NS_LOG_DEBUG(" \t\tRX PWR OGMs in txq sz " << m_txQueue.size() );
  }
}

void 
BatsenMac::CreateRouterOgm(void)
{
  if ( m_systemEnabled )
  {
    NS_LOG_DEBUG( "\t" << m_Address << " CreateRouterOgm @ " << Now() );

    // This half is for regular nodes that finished the announce
    // period and are now transmitting data to their selected router.
    NS_LOG_DEBUG( "\t" << m_Address << " sorting the peers @ " << Now() );
    // First select best router
    m_Lpeers.sort( PeerCompare );
    m_Mpeers.sort( PeerCompare );
    m_Hpeers.sort( PeerCompare );
    
#ifdef DUMP_PEERS
    DumpPeerLists();
#endif
    PeerNodeInfo best;
    bool bestSet = false;
      
    m_nextHop = SinkAddr;
    
    if ( m_Lpeers.size() > 0 )
    {
      best = *(m_Lpeers.front());
      bestSet = true;
      NS_LOG_DEBUG("Lf addr " << best.GetAddress() << " score " << best.GetScore());
      m_pwrToFwrd = LOW_POWER;
    }
    
    if ( m_Mpeers.size() > 0 )
    {
      if ( ( ( bestSet ) && ( *(m_Mpeers.front()) > best ) ) || ( !bestSet ) ) {
        best = *(m_Mpeers.front());
        bestSet = true;
        NS_LOG_DEBUG("Mf addr better " << best.GetAddress() << " score " << best.GetScore());
        m_pwrToFwrd = MEDIUM_POWER;
      }
    }
    
    if ( m_Hpeers.size() > 0 )
    {
      if ( ( ( bestSet ) && ( *(m_Hpeers.front()) > best ) ) || ( !bestSet ) ) {
        best = *(m_Hpeers.front());
        bestSet = true;
        NS_LOG_DEBUG("Hf addr better " << best.GetAddress() << " score " << best.GetScore());
        m_pwrToFwrd = HIGH_POWER;
      }
    }
    
    if ( ( ( bestSet ) && ( m_nodeSelf > best ) ) || ( !bestSet ) )
    {
      best = m_nodeSelf;
      bestSet = true;
      m_iAmForwarder = true;
      NS_LOG_DEBUG("MY addr better " << best.GetAddress() << " score " << best.GetScore());
    }
    else
      m_iAmForwarder = false;
    
    if ( bestSet )
      m_nextHop = best.GetAddress();
    else
      NS_LOG_DEBUG("   Best was never set...");
    
    NS_LOG_DEBUG("\tchoose " << m_nextHop << " as router" );
    
    // Add our selection to the Election List
    m_electList.Increment( m_nextHop );
    
    // Announce who I believe to be the best router
    {
      Ptr<Packet> p;
      OgmBuilder OGM;
      
      OGM.SetHeaderData( HPWR, POWER_REMAINING );
      OGM.AddAddressTlv( RTR_SELCT );
      OGM.AddAddress( m_nextHop );
              
      // Check to see if there are any nodes we missed their RXPWR OGM
      PeerMap::iterator citr = m_masterList.begin();
      bool fndAtrgt = false;
      
      NS_LOG_DEBUG("\t check to see if we need someone to report RXPWR");
      while ( citr != m_masterList.end() )
      {
        //NS_LOG_DEBUG("\t\tcheck address " << (*citr)->GetAddress());
        if ( (*citr)->NeedRxOgm() ) {
          // Setup the TLV
          if ( !fndAtrgt ) {
            NS_LOG_DEBUG( "\t\t  Adding NOT FOUND TLV to OGM" );
            OGM.AddAddressTlv( NOT_FOUND );
            // Don't set another TLV
            fndAtrgt = true;
          }
          
          NS_LOG_DEBUG( "\t\t  add " << (*citr)->GetAddress() );
          
          // Add the address to the NOT_FOUND TLV
          if ( !OGM.AddAddress( (*citr)->GetAddress() ) )
            break;
        }
        
        // If we still have extra space, check the next entry
        citr++;
      }
      
      // Now we schedule the TX of a NodeOGM
      p = CreatePacketWithData(RTRSEL_FRAME,
                                m_Address,
                                "FF:FF",
                                m_sequence,
                                OGM.GetDataPtr(),
                                OGM.GetDataSize() );
      
      NS_LOG_DEBUG( "\t" << m_Address << " create RTR OGM frame " << p << " with seq " << m_sequence << " sz " << (int)(OGM.GetDataSize()) << " pkt " << p );
      
      ++m_sequence;
      
      if ( m_txPkt ) {
        NS_LOG_DEBUG("\t    m_txPkt NOT empty: " << m_txPkt );

        while ( m_txQueue.size() > 0 )
          RemoveFirstTxQElement ();
        
        EnqueTxPacket(p);
      }
      else {
        EnqueTxPacket(p);
        NS_LOG_DEBUG("\t    m_txPkt empty: " << m_txPkt );
      }
      
      int64_t tNow = Now().GetMicroSeconds();
      tNow /= 10;
        
      double start = ((double)(tNow * 10)) / 1000000.0;
      double atNow = Now().GetSeconds();
      
      NS_LOG_DEBUG("\t  Now " << Now() << " in us " << Now().GetMicroSeconds() << " shift " << start << " N+s " << atNow - start );
      NS_LOG_DEBUG("\t  Now + shift " << Now() - Seconds(atNow - start));

      /*
       * Once we transmit our ROUTER SELECT OGM and collect others, we
       * can start the DATA TX/SLEEP operations
       */
      double msTillTx = NULL_OGM_PERIOD - (atNow - start);
      NS_LOG_DEBUG("\t\t" << m_Address << " scheduling SleepPlan in " << (msTillTx * 1000.0) << "ms or @ " << (Now() + Seconds( msTillTx )) );
      m_startSleep.Cancel();
      m_startSleep.SetFunction( &BatsenMac::SleepPlan, this );
      m_startSleep.Schedule( Seconds( msTillTx ) );
    }
  }
}
  
void
BatsenMac::ConfigureForwarderPower(void)
{
  PeerMap::iterator itr;
  if ( FindExistingPeer(m_nextHop, itr) )
  {
    PowerLvl lvl = (*itr)->GetFrwdPowerLvl();
    
    switch ( lvl )
    {
      case LPWR:
        m_pwrToFwrd = LOW_POWER;
        break;
        
      case MPWR:
        m_pwrToFwrd = MEDIUM_POWER;
        break;
        
      case HPWR:
      default:
        m_pwrToFwrd = HIGH_POWER;
        break;
    }
  }
  else
    NS_FATAL_ERROR("Forwarder without membership");
}
  
void
BatsenMac::SleepPlan(void)
{
  NS_LOG_DEBUG( m_Address << " SleepPlan" );
  
  double forwarders = ceil((double)m_curNodeCount * PERCENT_RTRS );
  
  // Determine which node is considered the Forwarder Candidate
  BNodeList tmp = m_electList.GetTopNodes( forwarders );
  
  if ( forwarders == 1 )
  {
    m_nextHop = tmp.front();
    
    NS_LOG_DEBUG("\t Elect List returned DST " << m_nextHop );
    
    if ( m_nextHop != m_Address )
    {
      ConfigureForwarderPower();
      
      NS_LOG_DEBUG("\n\t Pwr2Fwd " << m_pwrToFwrd << " Pwr2Sink " << m_pwrToSink << "\n");
      
      if ( m_pwrToFwrd >= m_pwrToSink )
      {
        NS_LOG_DEBUG( "\tsend to sink direct\n" );
        m_nextHop = SinkAddr;
        m_pwrToFwrd = m_pwrToSink;
      }
    }
  }
  else
  {
    NS_LOG_DEBUG("\t Elect List must pick a nxt hop of " << forwarders << " nodes in list" );
    
    PeerMap::iterator bstitr = m_masterList.end();
    
    Mac16Address bestAddr = SinkAddr;
    BNodeList::iterator itr = tmp.begin();
    while ( itr != tmp.end() )
    {
      NS_LOG_DEBUG("\t\tcheck " << (*itr));
      // If I'm listed in the top X%, then I must act as a forwarder
      if ((*itr) == m_Address ) {
        m_nextHop = m_Address;
        bestAddr = m_Address;
        NS_LOG_DEBUG("\t\t\tI'm in list - break\n");
        break;
      }
      
      // If our nextHop is in the list, then we will fix the bestAddr
      // as nextHop... but we must continue to check to see if our
      // address is in the list
      if ( ((*itr) == m_nextHop ) || ( bestAddr == m_nextHop ) )
      {
        bestAddr = m_nextHop;
        itr++;
        NS_LOG_DEBUG("\t\titr " << (*itr) << ": best " << bestAddr << " same as nxtHop " << m_nextHop );
        // We skip the next set of tests since best == next already
        // now we just look for ourselves in the list
        continue;
      }
      
      // We're not in the list and our nextHop is not so far.. so
      // look for the best of what we have
      if ( (*itr) != m_nextHop )
      {
        PeerMap::iterator tstitr = m_masterList.end();
        if ( FindExistingPeer((*itr), tstitr) )
        {
          // Test the TestIter against the currnet BestIter
          if ( bstitr != m_masterList.end() )
          {
            NS_LOG_DEBUG("\t\t  BstItr " << (*bstitr)->GetAddress() << " TstItr " << (*tstitr)->GetAddress() );
            NS_LOG_DEBUG("\t\t\t bstitr " << (*bstitr)->GetScore() << "\t" << " tstitr " << (*tstitr)->GetScore() );
            NS_LOG_DEBUG("\t\t\t bstitr pwr " << (*bstitr)->GetFrwdPowerLvl() << "\t" << " tstitr pwr " << (*tstitr)->GetFrwdPowerLvl() );
            
            if ( (*tstitr)->GetFrwdPowerLvl() < (*bstitr)->GetFrwdPowerLvl() )
            {
              NS_LOG_DEBUG("\t\t\t tst pwr is less");
              bstitr = tstitr;
            }
            else if ( ( (*tstitr)->GetFrwdPowerLvl() == (*bstitr)->GetFrwdPowerLvl() ) && ( (*tstitr)->GetScore() > (*bstitr)->GetScore() ) )
            {
              NS_LOG_DEBUG("\t\t\t tst pwr is == but tst score better");
              bstitr = tstitr;
            }
            
            NS_LOG_DEBUG("\t\t\t BstItr now " << (*bstitr)->GetAddress() );
            
            bestAddr = (*bstitr)->GetAddress();
          }
          else
          {
            NS_LOG_DEBUG( "\t\tset " << (*tstitr)->GetAddress() << " to bstitr" );
            // BestIter wasnt set, so set it now
            bstitr = tstitr;
          }
        }
        else
          NS_LOG_DEBUG("\t\t  couldn't find " << (*itr) << " in master list");

      }
      
      itr++;
    }
    
    if ( bestAddr != m_nextHop )
    {
      NS_LOG_DEBUG("\t Setting nxtHop to " << bestAddr );
      m_nextHop = bestAddr;
    }
    
    if ( m_nextHop != m_Address )
    {
      ConfigureForwarderPower();
      
      NS_LOG_DEBUG("\n\t Pwr2Fwd " << m_pwrToFwrd << " Pwr2Sink " << m_pwrToSink << "\n");
      
      if ( m_pwrToFwrd >= m_pwrToSink )
      {
        NS_LOG_DEBUG( "\tsend to sink direct\n" );
        m_nextHop = SinkAddr;
        m_pwrToFwrd = m_pwrToSink;
      }
    }
  }
  
  // If I'm not the forwarder, then I can setup a sleep plan
  if( m_Address != m_nextHop )
  {
    SetPower( m_pwrToFwrd );
    
    m_startSleep.Cancel();
    double msTillTx = (OGM_ANNOUNCE_PERIOD) * 0.8;
    
    NS_LOG_DEBUG( m_Address << " SleepPlan kick off - 1/2 period " << (msTillTx * 1000.0) << "ms  NOW == " << Now() );
    
    double intervals = ceil(msTillTx / SLOT_LENGTH);
    NS_LOG_DEBUG("\t intervals " << (int) intervals );
    
    //    double rndVal = m_random->GetValue(0.1, 1.0);
    double rndVal = (double)(m_random->GetInteger( 1, (int)intervals ) );
    double rndSft = (m_random->GetInteger(0, 20) * 0.00025) - 0.0025;
    double sleepTime = rndVal * 0.007;
    
    NS_LOG_DEBUG("\trndVal " << rndVal << " msTillTx " << msTillTx << " Tsleep " << sleepTime << " shift " << rndSft );
    
    m_sleepOne = sleepTime - rndSft;
    m_sleepTwo = Seconds( OGM_ANNOUNCE_PERIOD ) + Now();
    Time waitTime = Seconds( m_sleepOne );
    
    NS_LOG_DEBUG("\tWake at " << (waitTime + Now()) << " End by " << m_sleepTwo );
    
    m_startSleep.SetFunction( &BatsenMac::WakeToSendData, this );
    m_startSleep.Schedule( waitTime );
    
    // Set the PHY to XCVR OFF until we're ready to transmit
    m_macRxOnWhenIdle = false;
    m_justSentData = false;
    m_forceTxEvent = Simulator::ScheduleNow ( &BatsenMac::SetSensorMacState, this, SENSOR_IDLE );
  }
  else
  {
    // Tell the SensorHelper I've self elected as a forwarder
    if ( !m_sensorForwaderCallback.IsNull() )
      m_sensorForwaderCallback ( m_Address );
    
    // Otherwise, I'm the forwarder and should change my power level
    // to that required to reach the Sink
    SetPower( m_pwrToSink );
    
    // TODO: Setup the time to wait till we forward all received
    // DATA FRAME Addresses and Sequence numbers
    m_startSleep.Cancel();
    
	double msTillTx = (OGM_ANNOUNCE_PERIOD) * 0.2;
	double intervals = ceil(msTillTx / SLOT_LENGTH) / m_multiplier;
	double rndVal = (double)(m_random->GetInteger( 1, (int)intervals ) );
	
//    double rndVal = ((double)(m_random->GetInteger( 1, 24 ) * 4)) / 100.0;
//    double offset = rndVal * ((OGM_ANNOUNCE_PERIOD) * 0.2);
    double offset = rndVal * SLOT_LENGTH * m_multiplier;
    m_sleepOne = ((OGM_ANNOUNCE_PERIOD) * 0.8) + offset;
    m_sleepTwo = Seconds( OGM_ANNOUNCE_PERIOD ) + Now();
    
    // Break up the send time based on 'forwarders' value
  
    Time waitTime = Seconds( m_sleepOne );
    
    NS_LOG_DEBUG("\n\t FORWARDER send at " << (waitTime + Now()) << "\t end T " << m_sleepTwo );
    
    m_startSleep.SetFunction( &BatsenMac::RelayDataToSink, this );
    m_startSleep.Schedule( waitTime );
  }
}

void
BatsenMac::WakeToSendData(void)
{
  NS_LOG_DEBUG( m_Address << " WakeToSendData @ " << Now() );
  
  // Pull a DATA packet from data queue and move it to the tx packet
  if ( !m_dataQueue.empty() )
  {
    // We only wake the PHY if we have data to send - otherwise, we're going back to sleep
    m_macRxOnWhenIdle = true;
    m_forceTxEvent = Simulator::ScheduleNow ( &BatsenMac::SetSensorMacState, this, SENSOR_IDLE );
    
    if ( m_txPkt != 0 )
      NS_LOG_UNCOND( m_Address << " has extra pkt " << m_txPkt << " for DATA Frame @ " << Now() );
    
    Ptr<Packet> p = m_dataQueue.front()->txQPkt->Copy();
    m_dataQueue.pop_front();
  
    // Replace the Dest Address with the CH Address
    BatsenMacHeader tmpHdr;
    p->RemoveHeader (tmpHdr);
    
    uint32_t seq = tmpHdr.GetSeqNum();
    NS_LOG_DEBUG ( "\t" << m_Address << " transmit data seq " << seq << " pkt " << p << " src " << tmpHdr.GetSrcAddr() << " dst " << m_nextHop );
    
    tmpHdr.SetDstAddr( m_nextHop );
    p->AddHeader (tmpHdr);
    EnqueTxPacket(p);

    // Notify the Helper we're transmitting data frame
    if ( !m_sensorDataTransmitCallback.IsNull() )
      m_sensorDataTransmitCallback( seq, m_Address );

	m_forceTxEvent.Cancel();
    m_forceTxEvent = Simulator::ScheduleNow ( &BatsenMac::SetSensorMacState, this, SENSOR_CSMA );
    m_justSentData = true;
  }
  else
  {
    m_justSentData = false;
    
    Time waitTime = m_sleepTwo - Now();
   
    m_startSleep.Cancel();
    m_startSleep.SetFunction( &BatsenMac::WakePostSleep, this );
    m_startSleep.Schedule( waitTime );
  }
}
  
void
BatsenMac::BackToSleep(void)
{
  NS_LOG_DEBUG( m_Address << " BackToSleep " );
  
  // Set the PHY to XCVR OFF until we need to get the next OGM round
  m_macRxOnWhenIdle = false;
  m_forceTxEvent = Simulator::ScheduleNow ( &BatsenMac::SetSensorMacState, this, SENSOR_IDLE );
  
  m_justSentData = false;
  
  NS_LOG_DEBUG("\twake @ " << m_sleepTwo << " now " << Now() );
  if ( m_sleepTwo > Now() ) {
    Time waitTime = m_sleepTwo - Now();
    
    m_startSleep.Cancel();
    m_startSleep.SetFunction( &BatsenMac::WakePostSleep, this );
    m_startSleep.Schedule( waitTime );
  }
  else {
    NS_LOG_DEBUG("\tCan't go back to sleep");
    WakePostSleep();
  }
}
  
void
BatsenMac::WakePostSleep(void)
{
  // Return the PHY to RX IDLE
  m_macRxOnWhenIdle = true;
  m_forceTxEvent = Simulator::ScheduleNow ( &BatsenMac::SetSensorMacState, this, SENSOR_IDLE );
}

void
BatsenMac::RelayDataToSink(void)
{
  // Create a FWD_FRAME compsed of our Seq number and then
  // each neighbor along with their seq number
  uint8_t buffer[MAX_PAYLD_LEN];
  unsigned idx = 0;
  uint32_t seq = 0;
  
  bool createdPacket = false;
  
  if ( !m_dataQueue.empty() )
  {
    BatsenMacHeader tmpHdr;
    DataMessage msg;

    /*
     * Remove the header from the next packet in the data queue and get the
     * sequence number so we can insert it into the CH to SINK packet
     */
    Ptr<Packet> tpkt = m_dataQueue.front()->txQPkt->Copy();
    m_dataQueue.pop_front();
    
    tpkt->RemoveHeader (tmpHdr);
    tpkt->CopyData( (uint8_t *)&msg, sizeof(DataMessage) );
    tpkt = 0;
    
    uint32_t seq = msg.sequence;
    
    NS_LOG_DEBUG("\t FWRDR " << m_Address << " is xmit'ing data seq " << seq );
    
    if ( !m_sensorDataTransmitCallback.IsNull() )
      m_sensorDataTransmitCallback( seq, m_Address );
    
    m_Address.CopyTo( (uint8_t *) &buffer[ idx ] );
    idx += 2;
    
    buffer[ idx++ ] = seq & 0x00FF;
    buffer[ idx++ ] = ( (seq >> 8) & 0x00FF);
    buffer[ idx++ ] = ( (seq >> 16) & 0x00FF);
    buffer[ idx++ ] = ( (seq >> 24) & 0x00FF);
    
    createdPacket = true;
  }
  
  NS_LOG_DEBUG(" rcv mbr list " << m_rcvdMembers.size() << " seq list " << m_rcvdSequences.size() );
  
  // Now copy the address and seq # from every received frame into my packet
  while ( m_rcvdMembers.size() > 0 )
  {
    seq = m_rcvdSequences.front();
    m_rcvdMembers.front().CopyTo( (uint8_t *) &buffer[ idx ] );
    idx += 2;
    buffer[ idx++ ] = seq & 0x00FF;
    buffer[ idx++ ] = ( (seq >> 8) & 0x00FF);
    buffer[ idx++ ] = ( (seq >> 16) & 0x00FF);
    buffer[ idx++ ] = ( (seq >> 24) & 0x00FF);
    
    NS_LOG_DEBUG( "\t\t   front " << m_rcvdMembers.front() << " seq " << seq );
    m_rcvdMembers.pop_front();
    m_rcvdSequences.pop_front();
    
    if ( idx >= (MAX_PAYLD_LEN - 5) )
    {
      // Now we schedule the TX of the Forwarder Frame
      Ptr<Packet> p = CreatePacketWithData(FWRD_FRAME,
                                           m_Address,
                                           SinkAddr,
                                           m_sequence,
                                           (uint8_t *)buffer,
                                           idx );
      ++m_sequence;
      EnqueTxPacket(p);
      
      NS_LOG_DEBUG("\t\t" << m_Address << "  next DATA TX Pkt (sz " << idx << ") -> " << BufferToString(buffer, idx) << " pkt " << p );

      // restart by adding new nodes and seq #'a for another packet
      idx = 0;
      
      createdPacket = true;
    }
  }
  
  NS_LOG_DEBUG("");
  
  // Now we schedule the last Forwarder Frame for Xmission
  if( idx > 0 ) {
    Ptr<Packet> p = CreatePacketWithData(FWRD_FRAME,
                                         m_Address,
                                         SinkAddr,
                                         m_sequence,
                                         (uint8_t *)buffer,
                                         idx );
    
    NS_LOG_DEBUG("\t\t" << m_Address << "     last DATA TX Pkt (sz " << idx << ") -> " << BufferToString(buffer, idx) << " pkt " << p << " \n");

    ++m_sequence;
    EnqueTxPacket(p);
    
    createdPacket = true;
  }
  
  if ( createdPacket ) {
    m_justSentData = true;
	NS_LOG_DEBUG("\t" << m_Address << " FWDR sending data now" );
	m_forceTxEvent.Cancel();
    m_forceTxEvent = Simulator::ScheduleNow ( &BatsenMac::SetSensorMacState, this, SENSOR_CSMA);
  }
  else
    m_justSentData = false;
}
  
void
BatsenMac::ProcessOgmPresence(Mac16Address src, OgmMessage *msg, LrWpanSinrTag stag)
{
  // We need to use SINR (SNR without interference) to determine the best source
  // The higher the SINR, the closer the source
  PowerLvl powerlevel = LPWR;
  if ( msg->txPwrLvl == HPWR )
  {
    //    NS_LOG_DEBUG("\t Adding node " << src << " to rcvPwrList ");
    
    if( stag.Get() < 6.5 ) {
      powerlevel = HPWR;
      //NS_LOG_DEBUG("\t\t peer " << src << " can reach " << m_Address << " w/ HIGH Power" );
    }
    else if ( stag.Get() < 55.5 ) {
      powerlevel = MPWR;
      //NS_LOG_DEBUG("\t\t peer " << src << " can reach " << m_Address << " w/ MEDIUM Power" );
    }
    //else
    //NS_LOG_DEBUG("\t\t peer " << src << " can reach " << m_Address << " w/ LOW Power" );
    
    m_rcvPwrList.AddNode( powerlevel, src );
  }
  
  // Don't bother checking the source since it has infinite power
  if ( src != SinkAddr )
  {
     /*
    // Check to see if we have node in one of our Power lists
    NS_LOG_DEBUG("\t  Check if in a power list");
    PeerMap::iterator itr;
    if ( !FindPeerInLists( src, itr ) )
    {
      PeerNodeInfo *node = new PeerNodeInfo( src );
      node->SetPrcntPwrRemain( msg->pwrPrcnt );
      node->HeardFrom();
      NS_LOG_DEBUG(m_Address << " NOTE \t\taddress " << src << " not in master list - creating" );
      
      if ( msg->txPwrLvl == HPWR )
      {
        node->SetFrwdPowerLvl( powerlevel );
        node->DontNeedRxOgm();
        switch ( powerlevel )
        {
          case LPWR:
            m_Lpeers.push_back( node );
            break;
          case MPWR:
            m_Mpeers.push_back( node );
            break;
          case HPWR:
          default:
            m_Hpeers.push_back( node );
            break;
        }
      }
      */
     
    //NS_LOG_DEBUG("\t  Check if in MasterList");
    PeerMap::iterator itr;
    if ( !FindExistingPeer( src, itr ) )
    {
      PeerNodeInfo *node = new PeerNodeInfo( src );
      node->SetPrcntPwrRemain( msg->pwrPrcnt );
      node->HeardFrom();

      NS_LOG_DEBUG(m_Address << " NOTE \t\taddress " << src << " not in master list - creating" );
      m_masterList.push_back( node );
    }
    else {
      (*itr)->SetPrcntPwrRemain( msg->pwrPrcnt );
      //NS_LOG_DEBUG("\t\t" << src << " already in MstList - update pwr remain " << (*itr)->GetPrcntPwrRemain() );
    }
//    }
  }
}

void
BatsenMac::ProcessPeerOgm(Mac16Address src, char *buffer, int len, LrWpanSinrTag stag)
{
  OgmMessage *msg = (OgmMessage *)buffer;
  char *ptr = &(buffer[ sizeof(OgmMessage) ]);
  
  PeerMap::iterator check;
  FindExistingPeer( SinkAddr, check );
  
  //NS_LOG_DEBUG("\t  ProcessPeerOgm from " << src );

  // We already called the ProcessOgmPresence(), so the node is at least
  // in the master list - now we can check to see if we're in it's
  // received list - if we are, we can add it to one of the power lists
  PeerMap::iterator itr;

  if ( !FindExistingPeer( src, itr ) )
    NS_FATAL_ERROR( src << " somehow not in the list" );
  else
  {
    (*itr)->SetPrcntPwrRemain( ((double)msg->pwrPrcnt / 255.0) );

    // Now parse the TLVs
    uint8_t tlv = *ptr++;
    uint8_t num = *ptr++;
    
    while ( tlv != NULL_TLV )
    {
      //NS_LOG_DEBUG("\tTLV " << (unsigned)tlv << " num " << (unsigned)num );
      
      switch ( tlv )
      {
        case LPWR_LIST:
        case MPWR_LIST:
        case HPWR_LIST: {
          PowerLvl lvl = ((tlv == LPWR_LIST) ? LPWR : ( (tlv == MPWR_LIST) ? MPWR : HPWR ));
          
          for (uint8_t cnt = 0; cnt < num; cnt++)
          {
            char *address = ptr;
            Mac16Address mac;
            mac.CopyFrom( (uint8_t *)address );
            
            //NS_LOG_DEBUG("\t\tfound address " << mac );
            
            if ( mac == m_Address )
            {
              PeerMap::iterator test;
              if ( !FindPeerInLists( src, test ) )
              {
                //NS_LOG_DEBUG("\t\t  moving " << src << " into m_Xpeer lists");
                
                FindExistingPeer( SinkAddr, test );
                
                // If my address is in his list, then move him to one
                // of my power lists based on his perception of my TX power
                if ( tlv == LPWR_LIST )
                  m_Lpeers.push_back( *itr );
                else if ( tlv == MPWR_LIST )
                  m_Mpeers.push_back( *itr );
                else
                  m_Hpeers.push_back( *itr );
                
                (*itr)->DontNeedRxOgm();
                (*itr)->SetFrwdPowerLvl( lvl );
                
                FindExistingPeer( SinkAddr, test );
              }
              //              else
              //                NS_LOG_DEBUG("\t\t  do nothin - " << src << " already in m_Xpeer lists");
            }
            else
            {
              // Find the Node in the pwr list, and add the source to its
              // m_Xpower list
              PeerMap::iterator target;
              if ( FindExistingPeer( mac, target ) )
              {
                //                NS_LOG_DEBUG( "\t\t\t " << mac << " can reach " << src << "  with pwr " << lvl );
                (*target)->AddNode( lvl, src );
              }
              else
              {
                //                NS_LOG_DEBUG(m_Address << " NOTE \t\taddress " << mac << " not in master list - creating" );
                PeerNodeInfo *node = new PeerNodeInfo( mac );
                node->AddNode( lvl, src );
                
                m_masterList.push_back( node );
              }
            }
            
            ptr += 2;
          }
          break;
        }
        case NOT_FOUND: {
          NS_LOG_DEBUG("\t Process NOT_FOUND");
          for (uint8_t cnt = 0; cnt < num; cnt++)
          {
            char *address = ptr;
            Mac16Address mac;
            mac.CopyFrom( (uint8_t *)address );
            
            if ( mac == m_Address ) {
              cnt = num - cnt;
              ptr += (cnt * 2);
              m_resendPwr = true;
              NS_LOG_DEBUG("\t\t" << m_Address << " ... I need to resend RxPwr list");
              break;
            }
            ptr += 2;
          }
          break;
        }
        case RTR_SELCT: {
          // Read the prospective Router
          char *address = ptr;
          Mac16Address mac;
          mac.CopyFrom( (uint8_t *)address );
          ptr += 2;
          
          //NS_LOG_DEBUG("\t\t" << src << " wants " << mac << " to be the forwarder");
          
          m_electList.Increment( mac );
          m_electList.DumpList();
          break;
        }
        default: {
          ptr += num;
          break;
        }
      }

      // Check if we exceed the length of the packet
      if ( (ptr + 2) > &(buffer[len-1]) ) {
        //        printf(" \t     ptr %p beyond buffer end %p\n", (ptr + 2), &(buffer[len-1]));
        break;
      }

      tlv = *ptr++;
      num = *ptr++;
    }
    
    //NS_LOG_DEBUG(" \t     End of list");
  }
  
  // Force a calculation of the scores for this node
  (*itr)->CalculateScore(m_maxNumNodes);
  
  m_nodeSelf.CalculateScore(m_maxNumNodes);
}
  
void
BatsenMac::ProcessSinkOgm(Mac16Address src, char *buffer, int bytes)
{
  char *ptr = &(buffer[ sizeof(OgmMessage) ]);
  
  NS_LOG_DEBUG("\t  ProcessSinkOgm from " << src );

  // Update each peer with their ability to reach the Sink
  PeerMap::iterator itr;
  PowerLvl level = LPWR;
  Mac16Address taddr;

  // Now parse the TLVs
  uint8_t tlv = *ptr++;
  uint8_t len = (uint8_t)(*ptr++);
  while ( tlv != NULL_TLV )
  {
    NS_LOG_DEBUG("\t\tTLV: " << (unsigned)tlv << " w/ len: " << (unsigned)len );
    switch ( tlv )
    {
      case LPWR_LIST:
        level = LPWR;
        break;
        
      case MPWR_LIST:
        level = MPWR;
        break;
        
      case HPWR_LIST:
        level = HPWR;
        break;
        
      default:
      {
        NS_LOG_DEBUG("TLV is NOT a Power Level List - jump " << len << " bytes");
        ptr += len;
        continue;
      }
    }
    
    for ( int i = 0; i < len; i++)
    {
      // Get the MAC address from the list in the frame
      uint16_t addr = *((uint16_t *)ptr);
      taddr.CopyFrom( (uint8_t *)&addr );
      //NS_LOG_DEBUG( "\t\t\t  " << taddr << " in frame list ");
      
      // Find the node in the master list, and record its pwr lvl to sink
      PeerMap::iterator itr;
      if ( FindExistingPeer( taddr, itr ) ) {
        //NS_LOG_DEBUG( "\t\t\t     set pwr 2 sink lvl to " << ((level == LPWR) ? "LOW" : ((level == MPWR) ? "MED" : "HIGH") ) << " for " << taddr );
        (*itr)->SetSinkPowerLvl( level );
      }
      else {
        
        if ( taddr == m_Address ) {
          //NS_LOG_DEBUG("\t\t\t    my pwr to SINK is " << ((level == LPWR) ? "LOW" : ((level == MPWR) ? "MED" : "HIGH")) );
          m_pwrToSink = ((level == HPWR) ? HIGH_POWER : ((level == MPWR) ? MEDIUM_POWER : LOW_POWER ));
          m_nodeSelf.SetSinkPowerLvl( level );
          m_nodeSelf.SetFrwdPowerLvl( level );
        }
        else {
          // Peer was not in our master list
          NS_LOG_DEBUG("\t\t\tpeer " << taddr << " not in our master list?");

          PeerNodeInfo *node = new PeerNodeInfo( taddr );
          node->SetSinkPowerLvl( level );
          //NS_LOG_DEBUG( "\t\t\t     set pwr 2 sink lvl to " << ((level == LPWR) ? "LOW" : ((level == MPWR) ? "MED" : "HIGH") ) << " for " << taddr );
          
          NS_LOG_DEBUG(m_Address << " NOTE \t\taddress " << taddr << " not in master list - creating" );
          m_masterList.push_back( node );
        }
      }
      
      ptr += 2;
    }
    
    // Check if we exceed the length of the packet
    if ( (ptr + 2) > (buffer + bytes) )
      break;
    
    tlv = *ptr++;
    len = *ptr++;
  }
  
  NS_LOG_DEBUG("\t\tTLV is NULL");
}
  
void
BatsenMac::SinkScansNodeOgm(Mac16Address src, char *buffer, int bytes)
{
  char *ptr = &(buffer[ sizeof(OgmMessage) ]);
  
  NS_LOG_DEBUG("\t  SinkScansNodeOgm from " << src );
  
  // Now parse the TLVs
  uint8_t tlv = *ptr++;
  uint8_t len = (uint8_t)(*ptr++);
  while ( tlv != NULL_TLV )
  {
    NS_LOG_DEBUG("\t\tTLV: " << (unsigned)tlv << " w/ len: " << (unsigned)len );
    switch ( tlv )
    {
      case NOT_FOUND:
        NS_LOG_DEBUG("\t\t  Changing SINK state to INIT");
        ChangeFsmState( BATSEN_INIT );
        return;
        break;
        
      case LPWR_LIST:
      case MPWR_LIST:
      case HPWR_LIST:
      case RTR_SELCT:
      default:
        break;
    }
    
    ptr += (len * 2);
    
    
    // Check if we exceed the length of the packet
    if ( (ptr + 2) > (buffer + bytes) )
      break;
    
    tlv = *ptr++;
    len = *ptr++;
  }
}
  
/*****************************************************************
 *  Private Methods
 *****************************************************************/

Ptr<Packet>
BatsenMac::CreatePacket(BatsenMacFrameType type, Mac16Address src, Mac16Address dst, uint32_t seqNum )
{
  Ptr<Packet> p = new Packet();
  
  BatsenMacHeader macHdr (type, seqNum );
  macHdr.SetSrcAddr( src );
  macHdr.SetDstAddr( dst );
  
  p->AddHeader( macHdr );
  
  // Calculate FCS if the global attribute ChecksumEnable is set.
  MacFrameTrailer macTrailer;
  if (Node::ChecksumEnabled ())
  {
    macTrailer.EnableFcs (true);
    macTrailer.SetFcs (p);
  }
  p->AddTrailer (macTrailer);
  
  //NS_LOG_DEBUG( "\t\tCreatePkt w/o Data " << p );
  return p;
}

Ptr<Packet>
BatsenMac::CreatePacketWithData(BatsenMacFrameType type, Mac16Address src, Mac16Address dst, uint32_t seqNum, uint8_t const *data, int length )
{
  Ptr<Packet> p = new Packet( data, length );
  
  BatsenMacHeader macHdr (type, seqNum );
  macHdr.SetSrcAddr( src );
  macHdr.SetDstAddr( dst );
  macHdr.SetLength( (uint8_t)length );
  
  p->AddHeader( macHdr );
  
  // Calculate FCS if the global attribute ChecksumEnable is set.
  MacFrameTrailer macTrailer;
  if (Node::ChecksumEnabled ())
  {
    macTrailer.EnableFcs (true);
    macTrailer.SetFcs (p);
  }
  p->AddTrailer (macTrailer);
  
  //NS_LOG_DEBUG( "\t\t" << m_Address << " CreatePkt (seq: " << seqNum << ") w/ Data " << p << " type " << g_batsen_string[type] );
  return p;
}
  
void
BatsenMac::ProcessSinkReception( uint32_t psduLength, Ptr<Packet> p, uint8_t lqi )
{
  NS_LOG_DEBUG ( "\tSINK Reception" );
  
  BatsenMacHeader receivedMacHdr;
  p->RemoveHeader (receivedMacHdr);
  
  SensorDataIndicationParams params;
  params.m_dsn = receivedMacHdr.GetSeqNum();
  params.m_mpduLinkQuality = lqi;
  params.m_srcAddr = receivedMacHdr.GetSrcAddr();
  params.m_dstAddr = receivedMacHdr.GetDstAddr();
  
  bool forMe = receivedMacHdr.GetDstAddr () == m_Address;
  bool bcast = receivedMacHdr.GetDstAddr () == Mac16Address ("ff:ff");
  BatsenMacFrameType type = receivedMacHdr.GetFrameType();
  
  NS_LOG_DEBUG( "\t    RX " << g_batsen_string[type] << " in state " << batsenFsmString[m_batsenFsmState] );
  NS_LOG_DEBUG ("\t    Packet from " << params.m_srcAddr << " to " << params.m_dstAddr);
  
  // Check for a valid destination address - it's me or a broadcast
  // Otherwise I toss it
  if ( forMe || bcast )
  {
    switch ( type )
    {
      case FWRD_FRAME:
      {
        // Copy at least
        uint8_t buffer[MAX_PAYLD_LEN];
        unsigned idx = 0;
        unsigned endx = (unsigned)receivedMacHdr.GetLength();
        
        std::list<uint16_t> nodes;
        std::list<uint32_t> sequence;
        
        p->CopyData( (uint8_t *) &buffer, endx);
        
        NS_LOG_DEBUG("\t\tData Frame to ME from " << params.m_srcAddr << " with seq " << receivedMacHdr.GetSeqNum() << " end offset " << endx );
        
        while ( idx < endx ) {
          Mac16Address taddr;
          uint16_t addr = 0;
          uint32_t seq = 0;
          
          NS_LOG_DEBUG("\t\t rd buf (@ " << idx << ") -> " << BufferToString(&buffer[idx], 6) << " \n");
          
          taddr.CopyFrom( (uint8_t *)&buffer[idx] );
          taddr.CopyTo( (uint8_t *)&addr );
          addr = ntohs(addr);
          idx += 2;
          
          seq = buffer[ idx++ ];
          seq |= ( ( ((uint32_t)buffer[ idx++ ]) <<  8) & 0x0000FF00 );
          seq |= ( ( ((uint32_t)buffer[ idx++ ]) << 16) & 0x00FF0000 );
          seq |= ( ( ((uint32_t)buffer[ idx++ ]) << 24) & 0xFF000000 );

          NS_LOG_DEBUG("\t\t   addr: " << addr << " seq " << seq );
          nodes.push_back( addr );
          sequence.push_back( seq );
        }
        
        if ( !m_sensorSinkDataRcvdCallback.IsNull() )
          m_sensorSinkDataRcvdCallback( nodes, sequence );
        else
          NS_LOG_ERROR("No Sink MASS CallBack for Forwarder reception");
        
        break;
      }
      case DAT_FRAME:
      {
        // Copy at least
        DataMessage msg;
        p->CopyData( (uint8_t *) &msg, 6);
        
        NS_LOG_DEBUG("\t\tData Frame to ME from " << params.m_srcAddr << " with seq " << msg.sequence );

        if ( !m_sensorSinkDCRcvdCallback.IsNull() )
          m_sensorSinkDCRcvdCallback( params.m_srcAddr, msg.sequence );
        break;
      }
      case NULL_OGM:
      {
        char buffer[ p->GetSize() ];
        LrWpanSinrTag stag( 0.0 );
        p->PeekPacketTag( stag );
        p->CopyData( (uint8_t *)&buffer, p->GetSize() );
        OgmMessage *msg = (OgmMessage *)buffer;
        
        // Just check for presence
        ProcessOgmPresence( params.m_srcAddr, msg, stag );
        break;
      }
      case RTRSEL_FRAME:
      {
        char buffer[ p->GetSize() ];
        LrWpanSinrTag stag( 0.0 );
        p->PeekPacketTag( stag );
        p->CopyData( (uint8_t *)&buffer, p->GetSize() );
        OgmMessage *msg = (OgmMessage *)buffer;
        
        ProcessOgmPresence( params.m_srcAddr, msg, stag );
        SinkScansNodeOgm( params.m_srcAddr, buffer, p->GetSize() );
        break;
      }
      default:
        break;
    }
  }
}
  
void
BatsenMac::ProcessNodeReception( uint32_t psduLength, Ptr<Packet> p, uint8_t lqi )
{
  //NS_LOG_DEBUG ( "\tNODE Reception" );
  
  BatsenMacHeader receivedMacHdr;
  
  uint32_t psz = p->GetSize();
  uint8_t buffer[psz];
  
  p->CopyData( buffer, psz );
  //  NS_LOG_DEBUG("\t RX Pkt (sz " << psz << ") -> " << BufferToString(buffer, psz));

  p->RemoveHeader (receivedMacHdr);

  SensorDataIndicationParams params;
  params.m_dsn = receivedMacHdr.GetSeqNum();
  params.m_mpduLinkQuality = lqi;
  params.m_srcAddr = receivedMacHdr.GetSrcAddr();
  params.m_dstAddr = receivedMacHdr.GetDstAddr();
  
  bool forMe = receivedMacHdr.GetDstAddr () == m_Address;
  bool bcast = receivedMacHdr.GetDstAddr () == Mac16Address ("ff:ff");
  BatsenMacFrameType type = receivedMacHdr.GetFrameType();
  
  NS_LOG_DEBUG( "\t    RX " << g_batsen_string[type] << " packet from " << params.m_srcAddr );
  //NS_LOG_DEBUG ("\t    Packet from " << params.m_srcAddr << " to " << params.m_dstAddr << " seq " << receivedMacHdr.GetSeqNum() << " flen " << receivedMacHdr.GetLength() );
  
  // Check for a valid destination address - it's me or a broadcast
  // Otherwise I toss it
  if ( forMe || bcast )
  {
    switch ( type )
    {
      case SUPER_FRM:
      {
        Time waitTime = Seconds( 0.0 );
        SuperFrameMessage msg;
        p->CopyData( (uint8_t *)&msg, sizeof(SuperFrameMessage) );
        
        m_currentRound = msg.m_roundNum;
        
        //        NS_LOG_DEBUG("\t\tmsg round " << msg.m_roundNum);
        //        NS_LOG_DEBUG("\t\tmsg flags " << msg.m_ogmFlags);
        //        NS_LOG_DEBUG("\t\tmsg nodes " << msg.m_numNodes);
        //        NS_LOG_DEBUG("\t\tmsg fwrdr " << msg.m_forwarders);
        
        /* 
         * This is the only time we should set the minimum power
         * level required to reach the sink, as the network is 
         * not setup for mobility.
         */
        // We need to use SINR (SNR without interference) to determine the
        // best source. The higher the SINR, the closer the source
        LrWpanSinrTag stag( 0.0 );
        p->PeekPacketTag( stag );

        double divisor = 0.0;
        double period = 0.0;
        
        // record the last SF received for future debug purposes
        m_lastSFrcvd = msg.m_ogmFlags;
        
        switch ( msg.m_ogmFlags ) {
          case NULL_OGM_COLLECT: {
            ChangeFsmState( BATSEN_INIT );
            
            NS_LOG_DEBUG( "\tNULL OGM Collect " );
            // The Start OGM Flag kicks off OGMs transmissions form
            // nodes. No peer info is available yet though
            NullOgmXmitStage();
            
            // Set power level needed to reach the Sink
            if( stag.Get() < 6.5 ) {
              m_pwrToSink = HIGH_POWER;
              NS_LOG_DEBUG("\t\t " << m_Address << " can reach sink w/ HIGH Power" );
            }
            else if ( stag.Get() < 55.5 ) {
              m_pwrToSink = MEDIUM_POWER;
              NS_LOG_DEBUG("\t\t " << m_Address << " can reach sink w/ MEDIUM Power" );
            }
            else {
              m_pwrToSink = LOW_POWER;
              NS_LOG_DEBUG("\t\t " << m_Address << " can reach sink w/ LOW Power" );
            }
            
            divisor = SLOT_LENGTH;
            period = NULL_OGM_PERIOD;
            break;
          }
          case RXPWR_OGM_COLLECT: {
            NS_LOG_DEBUG( "\tRXPWR OGM Announce " );
            // The OGM announce is the regular OGM transmit with peer
            // selection. This is done throughout the life of the network
            RxPwrOgmXmitStage();
            ChangeFsmState( BATSEN_INIT );
            
            divisor = MULTI_SLOT_LEN;
            period = RXPWR_OGM_PERIOD;
            break;
          }
          case PERIODIC_OGM: {
            NS_LOG_DEBUG( "\tSink Periodic OGM " );
            
            m_curNodeCount = msg.m_numNodes;
            
            m_electList.Clear();
            ChangeFsmState( BATSEN_INIT );
            CreateRouterOgm();
            
            divisor = SLOT_LENGTH;
            period = NULL_OGM_PERIOD;
            break;
          }
          default: {
            NS_FATAL_ERROR("Unkown Rcvd SUPER FRAME");
            break;
          }
        }
        
        if ( m_txPkt || (m_txQueue.size() > 0) )
        {
          NS_LOG_DEBUG("\t  sending txPkt " << m_txPkt << " txq sz " << m_txQueue.size());
          
          int64_t tNow = Now().GetMicroSeconds();
          tNow /= 10;
          
          // Round the current ToD down to nearest us, so we all start around same time
          double start = ((double)(tNow * 10)) / 1000000.0;
          double atNow = Now().GetSeconds();
          
          //          NS_LOG_DEBUG("\t  Now " << Now() << " in us " << Now().GetMicroSeconds() << " shift " << start << " N+s " << atNow - start );
          //          NS_LOG_DEBUG("\t  Trun(Now) " << Now() - Seconds(atNow - start));
          NS_LOG_DEBUG("\t  OGM Collect Period = " << Seconds(period) );
          NS_LOG_DEBUG("\t  Phase ends @ " << Seconds(((atNow * 2) + (period)) - start) );
          
          int maxInterval = (int) floor((period) / divisor);
          double rndVal = (double)(m_random->GetInteger( 1, (maxInterval - 1)));
          int offset = m_random->GetInteger(0,10);
          
          double jitter = 0.003 - (0.0006 * offset);
          NS_LOG_DEBUG("\t  Jitter by " << jitter );
          
          waitTime = Seconds( rndVal * divisor ) - Seconds(atNow - start) - Seconds( jitter );
          
          NS_LOG_DEBUG( "\t" << m_Address << "  send Node OGM in " << waitTime << " CSMA pkt " << m_txPkt << "\n");
          
          m_csmaEvent.Cancel();
          m_csmaEvent = Simulator::Schedule(waitTime, &BatsenMac::SetSensorMacState, this, SENSOR_CSMA);
        }
        else
          NS_LOG_DEBUG("\tNO OGM to send this OGM frame");
        break;
      }
        
      case NULL_OGM:
      {
        if ( params.m_srcAddr != SinkAddr ) {
          char buffer[ p->GetSize() ];
          LrWpanSinrTag stag( 0.0 );
          p->PeekPacketTag( stag );
          p->CopyData( (uint8_t *)&buffer, p->GetSize() );
          OgmMessage *msg = (OgmMessage *)buffer;
          
          // Just check for presence
          ProcessOgmPresence( params.m_srcAddr, msg, stag );
        }
        break;
      }
        
      case RXPWR_OGM:
      {
        char buffer[ p->GetSize() ];
        LrWpanSinrTag stag( 0.0 );
        p->PeekPacketTag( stag );
        p->CopyData( (uint8_t *)&buffer, p->GetSize() );
        OgmMessage *msg = (OgmMessage *)buffer;
        
        if ( params.m_srcAddr != SinkAddr ) {
          // First check for presence
          ProcessOgmPresence( params.m_srcAddr, msg, stag );
          ProcessPeerOgm( params.m_srcAddr, buffer, p->GetSize(), stag );
        }
        else
          ProcessSinkOgm( params.m_srcAddr, buffer, p->GetSize() );

        break;
      }
        
      case RTRSEL_FRAME:
      {
        char buffer[ p->GetSize() ];
        LrWpanSinrTag stag( 0.0 );
        p->PeekPacketTag( stag );
        p->CopyData( (uint8_t *)&buffer, p->GetSize() );
        OgmMessage *msg = (OgmMessage *)buffer;
        
        if ( params.m_srcAddr != SinkAddr ) {
          // First check for presence
          ProcessOgmPresence( params.m_srcAddr, msg, stag );
          ProcessPeerOgm( params.m_srcAddr, buffer, p->GetSize(), stag );
        }
        else
          NS_LOG_ERROR("\t*************\n\tRTSEL from SINK\n\t*************\n");
        
        break;
      }
        
      case DAT_FRAME:
      {
        // Add the source address to the list of received msg's for future transmit
        m_rcvdMembers.push_back( params.m_srcAddr );
        
        // Copy at least
        DataMessage msg;
        p->CopyData( (uint8_t *) &msg, 6);
        
        NS_LOG_DEBUG("\t\tData Frame to ME from " << params.m_srcAddr << " with seq " << msg.sequence );
        
        m_rcvdSequences.push_back( msg.sequence );
        
        if ( !m_sensorCHDataRcvdCallback.IsNull() )
          m_sensorCHDataRcvdCallback( params.m_srcAddr, msg.sequence );
        break;
      }
      
      default:
        NS_LOG_ERROR("Invalid frame received while in INIT state");
        break;
    }
  }

#ifdef DUMP_PEERS
  DumpPeerLists();
#endif
}

void
BatsenMac::NullOgmPeriodComplete()
{
  // We've completed our attempts to send blank OGMs to announce our presence
  // Now its time to annouce our list of desirable forwarders or if we want to
  // forward based on our surroundings
  m_ogmTimer.Cancel();
  
  NS_LOG_DEBUG( m_Address << " NullOgmPeriodComplete @ " << Now() << "\t\tTxQ sz " << m_txQueue.size() );
  
  // We need to cancel any existing TX Packet if not yet transmitted
  if ( m_txPkt &&
       ( m_sensorMacState != SENSOR_SENDING ) &&
       ( m_sensorMacState != SENSOR_FORCE_TX ) )
  {
    NS_LOG_UNCOND("\t" << m_Address << " Erasing residual TX packet " << m_txPkt );
    m_txPkt = 0;
  }
}
  
void
BatsenMac::PlmeSetTRXStateConfirm (LrWpanPhyEnumeration status)
{
  NS_LOG_DEBUG ( m_Address << " PlmeSetTRXStateConfirm " << g_phy_string[status] << " pkt " << m_txPkt << " txq sz " << m_txQueue.size());
  
  // Check to see if we need to record a significant change in state
  // such that power drainage must be applied to the system power
  // NOTE: We only check this if we're not the SINK node
  if ( !m_iAmSink )
  {
    if ( m_previousState != status )
    {
      // We only apply aggregate power drain on significant state changes
      if ( m_systemEnabled &&
          (
           ( status == IEEE_802_15_4_PHY_IDLE ) ||
           ( status == IEEE_802_15_4_PHY_RX_ON ) ||
           ( status == IEEE_802_15_4_PHY_TRX_OFF ) ||
           ( status == IEEE_802_15_4_PHY_TX_ON )
           )
          )
      {
        // Calculate the time difference for teh previous Transceiver state
        Time tcurrent = Now();
        Time tDiff = tcurrent - m_timePrevState;
        
        //        NS_LOG_DEBUG( "\t T diff = " << tDiff );
        //        NS_LOG_DEBUG( "\t prev st " << g_phy_string[m_previousState] << " new st " << g_phy_string[status] );
        //        NS_LOG_DEBUG( "\t cur mW drain rate " << m_mWpwr );
        
        // Now we can update the previous state and the previous time to the
        // status we just received from the PHY
        m_timePrevState = tcurrent;
        m_previousState = status;
        
        //        NS_LOG_DEBUG( "\t cur sys pwr " << m_totalSystemPower );
        
        // Now reduce the power level of the system as a function of mJ/s
        m_totalSystemPower -= ( m_mWpwr * tDiff.GetSeconds() );
        //        NS_LOG_DEBUG( "\t new sys pwr " << m_totalSystemPower );
        
        // Update the % power remaining
        double tmpPower = m_totalSystemPower / m_maxSystemPower;
        m_nodeSelf.SetPrcntPwrRemain( tmpPower );
        
        // Shut down this sensor if the system power drops below the minimum
        // power required to participate in the sensor network
        if ( m_totalSystemPower <= 0.0 )
        {
          NS_LOG_DEBUG( "\t NODE DEAD " );
          m_systemEnabled = false;
          ChangeFsmState( BATSEN_NODE_DEAD );
          
          // Callback the death of the node to the SensorHelper
          if ( !m_sensorDeadCallback.IsNull() )
          {
            NodeJustDied();
            m_sensorDeadCallback( m_Address );
          }
          
          // Disable the RF since this node is dead
          if ( status != IEEE_802_15_4_PHY_TRX_OFF )
            m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TRX_OFF);
        }
        //        else
        //          NS_LOG_DEBUG( "\t Still alive" );
      }
      //      else
      //        NS_LOG_DEBUG( m_Address << " enabled " << m_systemEnabled );
    }
    //    else
    //      NS_LOG_DEBUG( m_Address << " no state change - ignore power calc ");
  }
  
  if ( m_systemEnabled )
  {
    if (m_sensorMacState == SENSOR_SENDING && (status == IEEE_802_15_4_PHY_TX_ON || status == IEEE_802_15_4_PHY_SUCCESS))
    {
      if ( !m_txPkt )
      {
        if ( m_txQueue.size() > 0 ) {
          TxQueueElement *txQElement = m_txQueue.front ();
          m_txPkt = txQElement->txQPkt;
          
          if ( m_txPkt == 0 )
            NS_LOG_UNCOND( m_Address << " TRXStateCnfrm failed to get pkt");
        }
        else
          NS_LOG_UNCOND( m_Address << " No packet in TxQueue to send");
      }
      
      if ( m_txPkt ) {
      
        //      NS_LOG_INFO( " Node " << m_Address << " pkt " << m_txPkt << " sz " << m_txPkt->GetSize () << " is calling Phy->PdDataRequest @ " << Now() );
        
        // Start sending if we are in state SENDING and the PHY transmitter was enabled.
        m_promiscSnifferTrace (m_txPkt);
        m_snifferTrace (m_txPkt);
        m_macTxTrace (m_txPkt);
        m_phy->PdDataRequest (m_txPkt->GetSize (), m_txPkt);
      }
      else if ( !m_txQueue.empty() ) {
        TxQueueElement *txQElement = m_txQueue.front ();
        m_txPkt = txQElement->txQPkt;
        
        m_promiscSnifferTrace (m_txPkt);
        m_snifferTrace (m_txPkt);
        m_macTxTrace (m_txPkt);
        m_phy->PdDataRequest (m_txPkt->GetSize (), m_txPkt);
      }
      else {
        NS_LOG_UNCOND( m_Address << " no pkt available - will abort @ " << Now() );
        NS_LOG_UNCOND ( m_Address << " TRANSMISSION status " << g_phy_string[status] << " with state " << g_chan_state[m_sensorMacState] << " m_txPkt " << m_txPkt << " txq sz " << m_txQueue.size() << " last SF rcvd " << (int)m_lastSFrcvd );
		
		// Need to reset the RF to RX MODE ?
		m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
        ChangeMacState (SENSOR_IDLE);
        //NS_FATAL_ERROR("no pkt to send @ " << Now() );
      }
    }
    else if (m_sensorMacState == SENSOR_CSMA && (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS))
    {
      //      NS_LOG_DEBUG("\tPlmeSetTRXStateConfirm CSMA start @ " << Now() );
      // Start the CSMA algorithm as soon as the receiver is enabled.
      m_csmaCa->Start ();
    }
    else if (m_sensorMacState == SENSOR_IDLE)
    {
      NS_ASSERT (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS || status == IEEE_802_15_4_PHY_TRX_OFF);
      
      //      NS_LOG_DEBUG("\tPlmeSetTRXStateConfirm Sensor Idle @ " << Now() << " myState " << GetStateString() );
      // Do nothing special when going idle.

      // Allow the TRX to go to sleep
      //      m_macRxOnWhenIdle = false;
      //      m_setMacState = Simulator::ScheduleNow ( &LeachMac::SetSensorMacState, this, SENSOR_IDLE );
    }
    else if (m_sensorMacState == SENSOR_ACK_PENDING)
    {
      //      NS_LOG_DEBUG( "\t\t\t SENSOR_ACK_PENDING ???" );
      NS_ASSERT (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS);
    }
    else
    {
      // TODO: What to do when we receive an error?
      // If we want to transmit a packet, but switching the transceiver on results
      // in an error, we have to recover somehow (and start sending again).
      NS_FATAL_ERROR ("Error changing transceiver state");
    }
  }
}
  
void
BatsenMac::CheckQueue ()
{
  if ( m_systemEnabled )
  {
    NS_LOG_DEBUG( "\t CheckQueue: Bstate " << batsenFsmString[m_batsenFsmState] << " pkt " << m_txPkt << " MAC is run " << m_setMacState.IsRunning () );
    
    // Pull a packet from the queue and start sending, if we are not already sending.
    if ( m_csmaOk && (m_txPkt != 0) )
    {
      NS_LOG_DEBUG( "\t\t" << m_Address << " +++   CSMA kick   +++ " << m_txPkt );
      m_setMacState = Simulator::ScheduleNow (&BatsenMac::SetSensorMacState, this, SENSOR_CSMA);
    }
    else if (m_sensorMacState == SENSOR_IDLE && !m_txQueue.empty () && m_txPkt == 0 && !m_setMacState.IsRunning ())
    {
      NS_LOG_DEBUG( "\t\t" << m_Address << " TxQ sz " << m_txQueue.size() );
      TxQueueElement *txQElement = m_txQueue.front ();
      m_txPkt = txQElement->txQPkt;
      m_setMacState = Simulator::ScheduleNow (&BatsenMac::SetSensorMacState, this, SENSOR_CSMA);
    }
    else
      NS_LOG_DEBUG( "\t\tNo CSMA kick off");
  }
  else
    NS_LOG_DEBUG ("Dead Node " << m_Address << " in CheckQueue " );
}
  
void
BatsenMac::SetSensorMacState (SensorChnState macState)
{
  if ( m_systemEnabled )
  {
    NS_LOG_DEBUG ( m_Address << " SetSensorMacState set mac state to " << g_chan_state[macState] << " @ " << Now() << " \t pkt " << m_txPkt << " FSMState " << batsenFsmString[m_batsenFsmState] << " SensorState " << g_chan_state[m_sensorMacState] );
    
    SensorDataConfirmParams confirmParams;
    
    if (macState == SENSOR_IDLE)
    {
      if ( m_sensorMacState == SENSOR_CSMA )
        m_csmaOk = false;
      else
        m_csmaCa->Cancel();
      
      ChangeMacState (SENSOR_IDLE);
      
      if (m_macRxOnWhenIdle)
      {
        m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
        CheckQueue();
      }
      else
      {
        NS_LOG_DEBUG("\tTRX OFF - no checking the queue");
        m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TRX_OFF);
      }
    }
    else if (macState == SENSOR_ACK_PENDING)
    {
      ChangeMacState (SENSOR_ACK_PENDING);
      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
    }
    else if (macState == SENSOR_CSMA)
    {
	  m_csmaEvent.Cancel();
	  m_forceTxEvent.Cancel();
      if ( m_sensorMacState == SENSOR_FORCE_TX )
      {
        NS_LOG_DEBUG( "  Trying to set CSMA while in state " << batsenFsmString[m_batsenFsmState] << " pkt " << m_txPkt << " FAIL CSMA START");
        m_csmaOk = false;
      }
      else
      {
        m_csmaOk = true;
        NS_LOG_DEBUG( "\t" << m_Address << " setting up CSMA @ " << Now() << " state " << g_chan_state[m_sensorMacState]);
        NS_ASSERT (m_sensorMacState == SENSOR_IDLE || m_sensorMacState == SENSOR_ACK_PENDING || m_sensorMacState == SENSOR_CSMA );
        
        ChangeMacState (SENSOR_CSMA);
        NS_LOG_DEBUG( "\tcalling phy " << m_phy << " using csma " << m_csmaCa );
        m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
      }
    }
    else if (m_sensorMacState == SENSOR_CSMA && macState == SENSOR_CHN_ACS_FAIL) {
      m_csmaOk = false;
      // cannot find a clear channel, drop the current packet.
      NS_LOG_DEBUG ( "\tcannot find clear channel w/ pkt " << m_txPkt << " reCSMA " << (int)m_numCsmacaRetry );
      m_csmaCa->Cancel();
      
      //confirmParams.m_msduHandle = m_txQueue.front ()->txQMsduHandle;
      confirmParams.m_status = SENSOR_CHANNEL_ACCESS_FAILURE;
      
      m_macTxDropTrace (m_txPkt);
      if ( !m_sensorDataConfirmCallback.IsNull() )
        m_sensorDataConfirmCallback (confirmParams);
      
      // remove the copy of the packet that was just sent
      NS_LOG_UNCOND( m_Address << " Xcvr - CHN_ACS_FAIL: popping packet " );
      RemoveFirstTxQElement ();
      
      ChangeMacState (SENSOR_IDLE);
    }
    else if (m_sensorMacState == SENSOR_CSMA && macState == SENSOR_CHN_IDLE) {
      m_csmaOk = false;
      NS_LOG_DEBUG( "\t" << m_Address << " --- switching from CSMA to SEND ---" );
      // Channel is idle, set transmitter to TX_ON
      ChangeMacState (SENSOR_SENDING);
      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TX_ON);
    }
    else if ( macState == SENSOR_FORCE_TX ) {
      m_csmaCa->Cancel();
      m_csmaOk = false;
      // Since there is no delay in the system, we may have achieved BUSY_RX between
      // callbacks - so check one last time to avoid going to TX while we're actually
      // in an active RX
      if ( m_phy->PhyIsBusy() ) {
        NS_LOG_DEBUG( m_Address << " can't Force TX, as we're in another state - staying in " << g_chan_state[m_sensorMacState] );
      
        NS_LOG_DEBUG( m_Address << " reschedule CSMA backoff");
        NS_LOG_DEBUG( "\t " << m_Address << " *** CSMA @ " << Now()+Seconds(SLOT_LENGTH) );
        m_csmaEvent.Cancel();
        m_csmaEvent = Simulator::Schedule ( Seconds(SLOT_LENGTH), &BatsenMac::SetSensorMacState, this, SENSOR_CSMA );
      }
      else {
        if ( m_txPkt ) {
          // Channel is idle, set transmitter to TX_ON
          ChangeMacState (SENSOR_SENDING);
          m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TX_ON);
        }
        else
          NS_LOG_DEBUG( " !!! ABORTING FORCE TX as pkt is " << m_txPkt );
      }
    }
    else if (m_sensorMacState == SENSOR_IDLE && macState == SENSOR_CHN_IDLE) {
      if ( m_csmaOk && (m_txPkt != 0) ) {
        m_csmaOk = false;
        NS_LOG_DEBUG("\t" << m_Address << " --- switching from CSMA to SEND SPECIAL ---" );
        // Channel is idle, set transmitter to TX_ON
        ChangeMacState (SENSOR_SENDING);
        m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TX_ON);
      }
      else {
        NS_LOG_DEBUG( "\t" << m_Address << " +++ Unknown Transition 1 to " << g_chan_state[macState] << " csma ok " << m_csmaOk << " txPkt " << m_txPkt << " txQ sz " << m_txQueue.size() );
        
        // Did we just miss our CSMA slot due to RX?
        CheckQueue();
      }
    }
    else
      NS_LOG_DEBUG( "\t" << m_Address << " +++ Unknown Transition 2 to " << g_chan_state[macState] );
  }
  else
    NS_LOG_DEBUG( "\t" << m_Address << " I'm dead and setting the TRCVR state?" );
}

bool
BatsenMac::FindExistingPeer(Mac16Address target, PeerMap::iterator &itr)
{
  bool found = false;
  itr = m_masterList.begin();

  //  NS_LOG_DEBUG("\tFindExistingPeer");

  while ( itr != m_masterList.end() )
  {
    //    NS_LOG_DEBUG("\t\t\tchk " << (*itr)->GetAddress());
    if ( (*itr)->GetAddress() == target )
    {
      found = true;
      break;
    }
    
    ++itr;
  }
  
  return found;
}
  
bool
BatsenMac::FindPeerInLists(Mac16Address target, PeerMap::iterator &itr)
{
  bool found = false;
  
  //  NS_LOG_DEBUG("\tFindPeersInLists");
  
  itr = m_Lpeers.begin();
  //  NS_LOG_DEBUG("\t\tLow Pwr List");
  while ( itr != m_Lpeers.end() )
  {
    //    NS_LOG_DEBUG("\t\t\tchk " << (*itr)->GetAddress());
    if ( (*itr)->GetAddress() == target )
    {
      found = true;
      break;
    }
    
    ++itr;
  }
  
  if ( !found )
  {
    itr = m_Mpeers.begin();
    //    NS_LOG_DEBUG("\t\tMed Pwr List");
    while ( itr != m_Mpeers.end() )
    {
      //      NS_LOG_DEBUG("\t\t\tchk " << (*itr)->GetAddress());
      if ( (*itr)->GetAddress() == target )
      {
        found = true;
        break;
      }
      
      ++itr;
    }
    
    if ( !found )
    {
      itr = m_Hpeers.begin();
      //      NS_LOG_DEBUG("\t\tHi Pwr List");
      while ( itr != m_Hpeers.end() )
      {
        //        NS_LOG_DEBUG("\t\t\tchk " << (*itr)->GetAddress());
        if ( (*itr)->GetAddress() == target )
        {
          found = true;
          break;
        }
        
        ++itr;
      }
    }
  }
  
  return found;
}
  
void
BatsenMac::EnqueTxPacket( Ptr<Packet> p )
{
  m_macTxEnqueueTrace (p);
  TxQueueElement *txQElement = new TxQueueElement;
  txQElement->txQMsduHandle = m_pktHandle++;
  txQElement->txQPkt = p;
  m_txQueue.push_back (txQElement);
  
  NS_LOG_DEBUG("\t\tEnqueueTxPacket: new TxQ sz " << m_txQueue.size() );
}
  
void
BatsenMac::RemoveFirstTxQElement ()
{
  NS_LOG_DEBUG( m_Address << "\t\t\tRemoveFirstTxQElement: TxQ sz " << m_txQueue.size() );
  
  if ( m_txQueue.size() > 0 )
  {
    TxQueueElement *txQElement = m_txQueue.front ();
    Ptr<const Packet> p = txQElement->txQPkt;
    
    NS_LOG_DEBUG( m_Address << "\t\t\t\ttxPkt " << m_txPkt << " Q " << p << " TxQ sz " << m_txQueue.size() );
    txQElement->txQPkt = 0;
    delete txQElement;
    m_numCsmacaRetry += m_csmaCa->GetNB () + 1;
    m_macTxDequeueTrace (p);
    m_txQueue.pop_front ();
    
    if ( m_txQueue.size() > 0 ) {
      txQElement = m_txQueue.front ();
      p = txQElement->txQPkt;
      NS_LOG_DEBUG( m_Address << "\t\t\t\tnew TxQ sz " << m_txQueue.size() << " nxt pkt " << p );
    }
    else
      NS_LOG_DEBUG( m_Address << "\t\t\t\tnew TxQ sz " << m_txQueue.size() );
  }
  
  m_txPkt = 0;
  m_retransmission = 0;
  m_numCsmacaRetry = 0;
}

void
BatsenMac::DumpPeerLists()
{
  PeerMap::iterator itr = m_Lpeers.begin();
  while( itr != m_Lpeers.end() )
  {
    (*itr)->GetScore();
    NS_LOG_DEBUG( "   L-peer " << (*itr)->GetAddress() << " score " << (*itr)->GetScore() );
    ++itr;
  }
  
  itr = m_Mpeers.begin();
  while( itr != m_Mpeers.end() )
  {
    (*itr)->GetScore();
    NS_LOG_DEBUG( "   M-peer " << (*itr)->GetAddress() << " score " << (*itr)->GetScore() );
    ++itr;
  }
  
  itr = m_Hpeers.begin();
  while( itr != m_Hpeers.end() )
  {
    (*itr)->GetScore();
    NS_LOG_DEBUG( "   H-peer " << (*itr)->GetAddress() << " score " << (*itr)->GetScore() );
    ++itr;
  }
  
#ifdef MST_LIST_DBG
  itr = m_masterList.begin();
  while( itr != m_masterList.end() )
  {
    (*itr)->GetScore();
    NS_LOG_DEBUG( "   MSTLst " << (*itr)->GetAddress() << " score " << (*itr)->GetScore() );
    ++itr;
  }
#endif
  
  m_nodeSelf.GetScore();
  NS_LOG_DEBUG( "   SELF:  " << m_Address << " score " << m_nodeSelf.GetScore() );
  
  PwrList::iterator pitr = m_rxPwrList.begin();
  while( pitr != m_rxPwrList.end() )
  {
    NS_LOG_DEBUG( "   MSTLst " << pitr->first << " pwr lvl " << ((pitr->second == LPWR) ? "LOW" : ((pitr->second == MPWR) ? "MED" : "HI" )) );
    ++pitr;
  }
}
  
} // namespace ns3

