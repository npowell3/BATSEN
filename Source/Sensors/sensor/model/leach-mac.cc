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

#include "leach-mac.h"
#include "leach-mac-header.h"
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

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT             std::clog << "[address " << m_Address << "] ";

#define CH_ADV_TIMEOUT                    ( ( m_numClusters * 3 ) * 5000 )
#define SINK_CHANNEL                      11

 char defaultMessage[] = " \n";
 char csmaCompleteMessage[] = " CSMA complete - Force TX";
 char joinRequestMessage[] = " join request message\n";
 char advertiseScheduleMessage[] = " advertise sched message\n";
 char nodeZeroData[] = " node 0 data to CH message\n";
 char sinkInitMessage[] = " sink sending INIT super frame\n";
 char nodeXdataMessage[] = " node X data to CH message after slot 0\n";
 char sinkAdvertiseMessage[] = " sink sending Advertise Schedule\n";
 char sinkKickOffMessage[] = " sink sending Kick of TDMA\n";
 char sinkDataMessage[] = " sink data phase start message\n";
 char chDataSendMessage[] = " CH sending data to sink\n";

namespace ns3 {
  
NS_LOG_COMPONENT_DEFINE ("LeachMac");

NS_OBJECT_ENSURE_REGISTERED (LeachMac);

  //  Minimum overhead of LEACH MACH Frame
  //  DST     2
  //  SRC     2
  //  MType   1
  //  FType   1
  //  Seq#    1
  //  Cks     2
const uint32_t LeachMac::aMinMPDUOverhead = 9;

//const double CSMASLOT = 0.007;  // 7ms for CSMA/CA
const double TDMASLOT = 0.0052; // 5.1ms for Forced TX in TDMA

#define TDMA_TIMEOUT(x)   ((m_maxNumNodes + m_numClusters + x) * TDMASLOT)

#define MAX_CH_FRAMES     ceil((m_maxNumNodes * 6.0) / MAX_PAYLOAD_LENGTH )
#define SINK_DAT_TIMEOUT  (m_numClusters * TDMASLOT * MAX_CH_FRAMES)
#define SINK_DAT_CH_SLOT  (ceil( (m_maxNumNodes * 6.0) / MAX_PAYLOAD_LENGTH ) * TDMASLOT)
  
TypeId
LeachMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LeachMac")
  .SetParent<Object> ()
  .SetGroupName ("Sensor")
  .AddConstructor<LeachMac> ()
  .AddAttribute ("NumNodes", "Number of nodes set for this sensor network.",
                 IntegerValue (100),
                 MakeIntegerAccessor (&LeachMac::m_maxNumNodes),
                 MakeIntegerChecker <int>())
  .AddAttribute ("NumClusters", "Number of clusters required per round for optimal power.",
                 IntegerValue (5),
                 MakeIntegerAccessor (&LeachMac::m_numClusters),
                 MakeIntegerChecker <int>())
  //.AddTraceSource ("MacTxEnqueue",
  //                 "Trace source indicating a packet has been "
  //                 "enqueued in the transaction queue",
  //                 MakeTraceSourceAccessor (&SensorMac::m_macTxEnqueueTrace),
  //                 "ns3::Packet::TracedCallback")
  //.AddTraceSource ("MacTxDequeue",
  //                 "Trace source indicating a packet has was "
  //                 "dequeued from the transaction queue",
  //                 MakeTraceSourceAccessor (&SensorMac::m_macTxDequeueTrace),
  //                 "ns3::Packet::TracedCallback")
  //.AddTraceSource ("MacTx",
  //                 "Trace source indicating a packet has "
  //                 "arrived for transmission by this device",
  //                 MakeTraceSourceAccessor (&SensorMac::m_macTxTrace),
  //                 "ns3::Packet::TracedCallback")
  //.AddTraceSource ("MacTxOk",
  //                 "Trace source indicating a packet has been "
  //                 "successfully sent",
  //                 MakeTraceSourceAccessor (&SensorMac::m_macTxOkTrace),
  //                 "ns3::Packet::TracedCallback")
  //.AddTraceSource ("MacTxDrop",
  //                 "Trace source indicating a packet has been "
  //                 "dropped during transmission",
  //                 MakeTraceSourceAccessor (&SensorMac::m_macTxDropTrace),
  //                 "ns3::Packet::TracedCallback")
  //.AddTraceSource ("MacRx",
  //                 "A packet has been received by this device, "
  //                 "has been passed up from the physical layer "
  //                 "and is being forwarded up the local protocol stack.  "
  //                 "This is a non-promiscuous trace,",
  //                 MakeTraceSourceAccessor (&SensorMac::m_macRxTrace),
  //                 "ns3::Packet::TracedCallback")
  
  
  //  .AddTraceSource ("MacStateValue",
  //                 "The state of LrWpan Mac",
  //                 MakeTraceSourceAccessor (&SensorMac::m_sensorMacState),
  //                 "ns3::TracedValueCallback::SensorMacState")
  //.AddTraceSource ("MacState",
  //                 "The state of LrWpan Mac",
  //                 MakeTraceSourceAccessor (&SensorMac::m_macStateLogger),
  //                 "ns3::SensorMac::StateTracedCallback")
  //.AddTraceSource ("MacSentPkt",
  //                 "Trace source reporting some information about "
  //                 "the sent packet",
  //                 MakeTraceSourceAccessor (&SensorMac::m_sentPktTrace),
  //                 "ns3::SensorMac::SentTracedCallback")
  ;
  return tid;
}

LeachMac::LeachMac () :
  SensorMac(),
  m_forceTxTimer (Timer::CANCEL_ON_DESTROY),
  m_waitTimer (Timer::CANCEL_ON_DESTROY),
  m_joinTimer (Timer::CANCEL_ON_DESTROY),
  m_chIsDeadTimer( Timer::CANCEL_ON_DESTROY)
{
  NS_LOG_FUNCTION (this);
  
  // Need the PRNG for the Threshold detection algorithm
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
  
  // First set the state to a known value, call ChangeMacState to fire trace source.
  m_sensorMacState = SENSOR_IDLE;
  ChangeMacState (SENSOR_IDLE);
  
  // LEACH FSM starts in the initialization state for the first round
  m_leachMacState = LEACH_INIT;
  ChangeLeachState (LEACH_INIT);

  m_retransmission = 0;
  m_numCsmacaRetry = 2;
  m_txPkt = 0;
  
  Ptr<UniformRandomVariable> uniformVar = CreateObject<UniformRandomVariable> ();
  uniformVar->SetAttribute ("Min", DoubleValue (0.0));
  uniformVar->SetAttribute ("Max", DoubleValue (255.0));

  m_macDsn = SequenceNumber8 (0);
  m_Address = Mac16Address ("00:00");
  
  m_ClusterHeads.clear();
  m_MemberNodes.clear();
  m_rcvdMembers.clear();
  m_rcvdSequences.clear();
  
  m_iAmSink = false;
  iAmClusterHead = false;
  hasBeenClusterHead = false;
  m_pauseCsma = false;
  
  m_waitForAck = false;
  m_joinMsgSent = false;
  
  m_numChs = 0;
  m_currentRound = -1;
  m_myCHchan = 0;
  
  m_sleepStart = Seconds( 0.0 );
  m_sleepFinish = Seconds( 0.0 );
  
  // Convert the base power level (in dBm) to mW power
  m_mWpwr = pow(10., (0.0 - 30) / 10) / 1000.0;
  NS_LOG_DEBUG( " POWER: " << m_mWpwr << "mW == 0.0 dBm" );
  
  // This epoch counter only matter for the SINK node, bue we set it here
  m_epoch = -1;
  
  NS_LOG_DEBUG("In the LEACH MAC");
  
  checkCounter = 0;
  
  debugForceTx = NULL;
}

LeachMac::~LeachMac ()
{
  m_ClusterHeads.clear();
  m_MemberNodes.clear();
  m_rcvdMembers.clear();
  m_rcvdSequences.clear();
}
  
void
LeachMac::KickOffFSM()
{
  NS_LOG_FUNCTION (this << m_iAmSink );

  SensorMac::SetChannel( SINK_CHANNEL );
  SensorMac::SetPower( HIGH_POWER );

  m_phy->SetAddr( m_Address );
  
  // Max number of rounds is the # node / # desired clusters
  m_totalRounds = m_maxNumNodes / m_numClusters;
  
  NS_LOG_DEBUG( "\t Starting sys pwr " << m_totalSystemPower << " # CHs " << m_numClusters << " nodes " << m_maxNumNodes );
  
  Time waitTime = Seconds( 0.5 );
  m_chAdvTimeout = Simulator::Schedule (waitTime, &LeachMac::InitPhaseTimeout, this);
}

void LeachMac::NodeJustDied()
{
  m_forceTxTimer.Cancel();
  m_waitTimer.Cancel();
  m_joinTimer.Cancel();
  m_chIsDeadTimer.Cancel();
  m_phy->Disable();
}

void
LeachMac::SensorDataRequest (SensorDataRequestParams params, Ptr<Packet> p)
{
  //NS_LOG_DEBUG ("Node " << m_Address << " queing existing data packet " << p);
  
  SensorDataConfirmParams confirmParams;
  confirmParams.m_msduHandle = params.m_msduHandle;
  
  /*
   * We need to convert the blank packet into a payload within the 
   * standard DataPayload_s data structure, to include the sequence 
   * number from the params.
   */
  DataPayload_s msg;
  p->CopyData( (uint8_t *)&msg, p->GetSize() );
  
  msg.numberBytes = p->GetSize();
  msg.sequence = params.m_seqNumber;
  
  // Clear out old data
  p->RemoveAtStart( p->GetSize() );

  LeachMacHeader macHdr (LEACH_ORIGINAL,
                         DATA_FRAME,
                         m_macDsn.GetValue());
  m_macDsn++;

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

  macHdr.SetSrcAddr( GetAddress() );
  
  /*
   *  Destination should always be the CH or the Sink.
   *  If I am the CH, I need to queue the data with prev receptions
   *  If I am not the CH, I need to send it to my CH
   */
  if ( !iAmClusterHead )
    macHdr.SetDstAddr( m_currentCh );
  else
    macHdr.SetDstAddr( params.m_dstAddr ); 
    // TODO: q the data with bulk data
  
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
LeachMac::SensorDataRequest (SensorDataRequestParams params )
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
    DataPayload_s msg;
    msg.numberBytes = params.m_pktSize;
    msg.sequence = params.m_seqNumber;
    
    //NS_LOG_DEBUG( "\tbytes " << msg.numberBytes << " seq " << msg.sequence );
    
    /*
     *  Destination should always be the CH or the Sink.
     *  If I am the CH, I need to queue the data with prev receptions
     *  If I am not the CH, I need to send it to my CH
     */
    Mac16Address dest;
    if ( !iAmClusterHead )
      dest = m_currentCh;
    else
      dest = params.m_dstAddr;
    
    // Create the packet
    Ptr<Packet> p = CreatePacketWithData( DATA_FRAME, m_Address,
                                         dest, params.m_seqNumber,
                                         (uint8_t const*)&msg, msg.numberBytes );
    
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
    m_macDsn++;

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
LeachMac::ForceAnotherTx()
{
  m_forceTxEvent.Cancel();
  m_forceTxEvent = Simulator::ScheduleNow( &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX );
}
  
void
LeachMac::CheckQueue ()
{
  if ( m_systemEnabled )
  {
    //  NS_LOG_FUNCTION (this);
    
    NS_LOG_DEBUG( "\t CheckQueue: Lstate " << leachString[m_leachMacState] << " Qempty " << m_txQueue.empty () << " pkt " << m_txPkt << " MAC is run " << m_setMacState.IsRunning () );

  //        m_leachMacState == LEACH_ADV_CH ||
  //        m_leachMacState == LEACH_CH_SCHED ||
    
    // Pull a packet from the queue and start sending, if we are not already sending.
    if ( (m_leachMacState == LEACH_ADVERTISE ||
          m_leachMacState == LEACH_DIRECT_CONN ||
          ( iAmClusterHead && m_leachMacState == LEACH_SINK_DATA ) ) &&
          
         !m_txQueue.empty () && m_txPkt == 0 && !m_setMacState.IsRunning ())
    {
      TxQueueElement *txQElement = m_txQueue.front ();
      m_txPkt = txQElement->txQPkt;
      NS_LOG_DEBUG( "\tCSMA kicked off " << m_txPkt );
      NS_LOG_DEBUG( "\t *** CSMA @ " << Now() );
      m_csmaEvent.Cancel();
      m_csmaEvent = Simulator::ScheduleNow ( &LeachMac::SetSensorMacState, this, SENSOR_CSMA );
    }
    else if ( ( m_csmaOk && (m_txPkt != 0) && (m_leachMacState > LEACH_INIT) ) ||
              ( (m_iAmSink || iAmClusterHead) && (m_txPkt != 0) ) )
    {
      if ( m_iAmSink || (iAmClusterHead && (m_leachMacState == LEACH_DATA_PHASE)) )
      {
        NS_LOG_DEBUG( "\t SINK/CH Calling SENSOR_FORCE_TX LATER @ " << Now() + Seconds(0.0005) );
        //m_idleEvent.Cancel();
        //m_idleEvent = Simulator::ScheduleNow ( &LeachMac::SetSensorMacState, this, SENSOR_IDLE );
        //        m_forceTxEvent.Cancel();
        //        m_forceTxEvent = Simulator::Schedule( Now()+Seconds(0.001), &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX );
        
        // Setup a timer for me to transmit in my TDMA slot
        m_forceTxTimer.Cancel();
        m_forceTxTimer.SetFunction( &LeachMac::ForceAnotherTx, this );
        m_forceTxTimer.Schedule( Seconds( 0.0005 ) );
      }
      else
      {
        if ( m_pauseCsma )
        {
          NS_LOG_DEBUG( "\tCSMA complete - but need to back off " << m_txPkt );
          NS_LOG_DEBUG( "\t Calling SENSOR_FORCE_TX 1 @ " << ( Now() + Seconds(0.0005) ) );
          // CSMA/CA reported we're clear to transmi - so send it
          m_csmaEvent.Cancel();
          m_forceTxEvent.Cancel();
          m_forceTxEvent = Simulator::Schedule ( Now() + Seconds(0.0005), &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX );
        }
        else
        {
          NS_LOG_DEBUG( "\tCSMA complete - TX frame " << m_txPkt );
          
          // If we're sending a JOIN request, then config the control variables
          // to check for a collision with another potential CH-M
          if ( m_waitForAck )
            m_joinMsgSent = true;
          
          if ( debugForceTx == NULL )
            debugForceTx = csmaCompleteMessage;
          
          NS_LOG_DEBUG( "\t Calling SENSOR_FORCE_TX 2 @ " << Now() );
          // CSMA/CA reported we're clear to transmi - so send it
          m_csmaEvent.Cancel();
          m_forceTxEvent.Cancel();
          m_forceTxEvent = Simulator::ScheduleNow ( &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX );
        }
      }
    }
    else
    {
      NS_LOG_DEBUG( "\tNo CSMA kick off");
      
      // Clear the current packet, as we're aborting the transmission
      m_txPkt = 0;
    }
  }
  else
    NS_LOG_DEBUG ("Dead Node " << m_Address << " in CheckQueue " );
}
  
void
LeachMac::PdDataIndication (uint32_t psduLength, Ptr<Packet> p, uint8_t lqi)
{
  NS_ASSERT (m_sensorMacState == SENSOR_IDLE || m_sensorMacState == SENSOR_ACK_PENDING || m_sensorMacState == SENSOR_CSMA);
  
  LrWpanSinrTag stag( 0.0 );
  p->PeekPacketTag( stag );
  
  NS_LOG_DEBUG ("RECEIVE FRAME: @ " << Now() << " Len " << psduLength << " LQI " << (int)lqi << " SINR " << (double)stag.Get() );

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
  if (!receivedMacTrailer.CheckFcs (p))
  {
    m_macRxDropTrace (originalPkt);
  }
  else
  {
    //level 2 frame filtering
    m_macRxTrace (originalPkt);
    
    // Process the reception if I AM the SINK NODE
    if ( m_iAmSink )
      ProcessSinkReception( originalPkt );
    else
    {
      // Now process the packet based on the current state of a non-sink node
      // If not in a CH mode yet, use the generic receiver function
      if ( ( m_leachMacState < LEACH_WAIT_TDMA ) && ( !iAmClusterHead ) )
      {
        ProcessNodeReception( originalPkt, lqi );
      }
      else
      {
        // Otherwise, use the CH or Non-CH receiver
        if ( iAmClusterHead )
          ProcessClusterHeadReception( originalPkt, lqi );
        else
          ProcessClusterMemberReception( originalPkt );
      }
    }
  }
}
  
void
LeachMac::SetSensorMacState (SensorChnState macState)
{
  if ( m_systemEnabled )
  {
    NS_LOG_DEBUG ( "SetSensorMacState set mac state to " << g_chan_state[macState] << " @ " << Now() << " \t pkt " << m_txPkt << " LeachState " << leachString[m_leachMacState] << " SensorState " << g_chan_state[m_sensorMacState] );

    char *oldPtr = debugForceTx;
    if ( macState == SENSOR_FORCE_TX )
    {
      if ( debugForceTx != NULL )
        NS_LOG_DEBUG( "\tFORCE TX kicked off for pkt " << m_txPkt << " reason: " << debugForceTx );
      else
        NS_LOG_DEBUG( "\tFORCE TX kicked off for pkt " << m_txPkt << " reason: EMPTY\n" );
      
      debugForceTx = NULL;
    }
    
    SensorDataConfirmParams confirmParams;
    
    if (macState == SENSOR_CHN_IDLE || macState == SENSOR_IDLE)
    {
      if ( m_sensorMacState == SENSOR_CSMA )
        m_csmaOk = true;
      else
        m_csmaCa->Cancel();
        
      ChangeMacState (SENSOR_IDLE);
      
      if (m_macRxOnWhenIdle)
      {
        m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
        CheckQueue ();
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
      if ( ( m_sensorMacState == SENSOR_CSMA ) &&
           ( ( m_leachMacState == LEACH_SCHEDULE ) ||
             ( m_leachMacState == LEACH_DIRECT_CONN ) ) )
      {
        NS_LOG_DEBUG( " Reseting CSMA due to CSMA Interrupted - state " << leachString[m_leachMacState] );
      }
      else if ( m_sensorMacState == SENSOR_FORCE_TX )
      {
        NS_LOG_DEBUG( " Trying to set CSMA while in state " << leachString[m_leachMacState] << " pkt " << m_txPkt );
      }
      else
      {
        //NS_ASSERT (m_sensorMacState == SENSOR_IDLE || m_sensorMacState == SENSOR_ACK_PENDING );
        
        ChangeMacState (SENSOR_CSMA);
        m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
      }
    }
    else if (m_sensorMacState == SENSOR_CSMA && macState == SENSOR_CHN_ACS_FAIL)
    {
      // cannot find a clear channel, drop the current packet.
      NS_LOG_DEBUG ( "\tcannot find clear channel w/ pkt " << m_txPkt << " reCSMA " << (int)m_numCsmacaRetry );
      m_csmaCa->Cancel();
      
      //confirmParams.m_msduHandle = m_txQueue.front ()->txQMsduHandle;
      confirmParams.m_status = SENSOR_CHANNEL_ACCESS_FAILURE;
      
      m_macTxDropTrace (m_txPkt);
      if ( !m_sensorDataConfirmCallback.IsNull() )
        m_sensorDataConfirmCallback (confirmParams);

      // remove the copy of the packet that was just sent
      NS_LOG_DEBUG( " Xcvr - CHN_ACS_FAIL: not popping packet " );
      RemoveFirstTxQElement ();
      
      ChangeMacState (SENSOR_IDLE);
    }
    else if ( macState == SENSOR_FORCE_TX )
    {
      m_csmaCa->Cancel();
      
      // Since there is no delay in the system, we may have achieved BUSY_RX between
      // callbacks - so check one last time to avoid going to TX while we're actually
      // in an active RX
      if ( m_phy->PhyIsBusy() )
      {
        NS_LOG_DEBUG( " can't Force TX, as we're in another state - staying in " << g_chan_state[m_sensorMacState] );
        
        if ( m_leachMacState == LEACH_DIRECT_CONN )
        {
          debugForceTx = oldPtr;
          NS_LOG_DEBUG( " reschedule CSMA backoff due to DIR CONN ");
          NS_LOG_DEBUG( "\t *** CSMA @ " << Now()+Seconds(0.007) );
          m_csmaEvent.Cancel();
          m_csmaEvent = Simulator::Schedule ( Seconds(0.007), &LeachMac::SetSensorMacState, this, SENSOR_CSMA );
        }
      }
      else
      {
        if ( m_txPkt )
        {
          if ( ( m_leachMacState == LEACH_SCHEDULE ) && 
               ( m_schdDoneTime - Now() < Time( Seconds(0.005)) ) )
          {
            NS_LOG_DEBUG(" ABORTING TRANSMISSION - Too close to Check for CH Membership" );
          }
          else
          {
            // Channel is idle, set transmitter to TX_ON
            ChangeMacState (SENSOR_SENDING);
            m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TX_ON);
          }
        }
        else
          NS_LOG_DEBUG( " !!! ABORTING FORCE TX as pkt is " << m_txPkt );
      }
    }
    else
      NS_LOG_DEBUG( "\t+++ Unknown Transition to " << g_chan_state[macState] );
  }
  else
    NS_LOG_DEBUG( "\tI'm dead and setting the TRCVR state?" );
}
  
void
LeachMac::RemoveFirstTxQElement ()
{
  if ( m_systemEnabled )
  {
    NS_LOG_FUNCTION( this << m_txPkt);
    
    if ( m_txQueue.size () > 0 )
    {
      TxQueueElement *txQElement = m_txQueue.front ();
      Ptr<const Packet> p = txQElement->txQPkt;
      
      NS_LOG_DEBUG( "\tCOPYING Pkt to m_txPkt " << m_txPkt << " TxQ remain " << m_txQueue.size () );
      m_txPkt = p->Copy ();
      m_numCsmacaRetry += m_csmaCa->GetNB () + 1;
      
      Ptr<Packet> pkt = p->Copy ();
      LeachMacHeader hdr;
      pkt->RemoveHeader (hdr);
      
      if (hdr.GetDstAddr () != Mac16Address ("ff:ff"))
        m_sentPktTrace (p, m_retransmission + 1, m_numCsmacaRetry);
      
      txQElement->txQPkt = 0;
      delete txQElement;
      m_txQueue.pop_front ();
      m_macTxDequeueTrace (p);
    }
    else
    {
      m_txPkt = 0;
      m_retransmission = 0;
      //m_numCsmacaRetry = 0;
      //m_txQueue.clear();
      NS_LOG_DEBUG("\tClearing m_txPkt");
    }
  }
  else
    NS_LOG_DEBUG ("Dead Node " << m_Address << " RemoveFirstTxQElem " );
}
  
void
LeachMac::PdDataConfirm (LrWpanPhyEnumeration status)
{
  NS_LOG_DEBUG ( "TRANSMISSION status " << g_phy_string[status] << " with state " << g_chan_state[m_sensorMacState] << " m_txPkt " << m_txPkt );
  
  if ( m_txPkt )
  {
    LeachMacHeader macHdr;
    m_txPkt->PeekHeader (macHdr);
    
    //NS_ASSERT (m_sensorMacState == SENSOR_SENDING);

    if (status == IEEE_802_15_4_PHY_SUCCESS)
    {
      m_macTxOkTrace (m_txPkt);
      // remove the copy of the packet that was just sent
      if (!m_sensorDataConfirmCallback.IsNull () && (m_txQueue.size () > 0) )
      {
        SensorDataConfirmParams confirmParams;
        // NS_ASSERT_MSG (m_txQueue.size () > 0, "TxQsize = 0");
        TxQueueElement *txQElement = m_txQueue.front ();
        confirmParams.m_msduHandle = txQElement->txQMsduHandle;
        confirmParams.m_status = SENSOR_SUCCESS;
        m_sensorDataConfirmCallback (confirmParams);
      }
      
      NS_LOG_DEBUG( " Xcvr - SUCCESS: popping packet " );
      RemoveFirstTxQElement ();
    }
    else if (status == IEEE_802_15_4_PHY_UNSPECIFIED)
    {
      NS_ASSERT_MSG (m_txQueue.size () > 0, "TxQsize = 0");
      if ( m_txQueue.size() > 0 )
      {
        TxQueueElement *txQElement = m_txQueue.front ();
        m_macTxDropTrace (txQElement->txQPkt);
        if (!m_sensorDataConfirmCallback.IsNull ())
        {
          SensorDataConfirmParams confirmParams;
          confirmParams.m_msduHandle = txQElement->txQMsduHandle;
          confirmParams.m_status = SENSOR_FRAME_TOO_LONG;
          m_sensorDataConfirmCallback (confirmParams);
        }
      
        NS_LOG_DEBUG( " Xcvr - UNSPECIFIED: not popping packet " );
        RemoveFirstTxQElement ();
      }
      else
        NS_LOG_DEBUG("UNKNOWN loss of frame @ PdDataConfirm callback: node " << m_Address << " lstPkt " << m_txPkt );
    }
    else if ( status == IEEE_802_15_4_PHY_TX_FAIL )
    {
      NS_LOG_DEBUG( "***************************\n" << "\nTX FAIL\n" << "***************************\n" );
    }
    else
    {
      // Something went really wrong. The PHY is not in the correct state for
      // data transmission.
      //NS_FATAL_ERROR (" Node " << m_Address << " transmission attempt failed with PHY status " << status);
      NS_LOG_DEBUG (" Node " << m_Address << " transmission attempt failed with PHY status " << status);
    }
  }
  
  m_delayAdvSch += Seconds( 0.000010 );
  Time spawn = Now() + m_delayAdvSch;
  NS_LOG_DEBUG( "\twait to go Idle till Now( " << Now() << ") + " << m_delayAdvSch << " = " << spawn );
  m_setMacState.Cancel ();
  m_setMacState = Simulator::ScheduleNow( &LeachMac::SetSensorMacState, this, SENSOR_IDLE);
}

void
LeachMac::PlmeCcaConfirm (LrWpanPhyEnumeration status)
{
  NS_LOG_FUNCTION (this << status);
  // Direct this call through the csmaCa object
  m_csmaCa->PlmeCcaConfirm (status);
}

void
LeachMac::PlmeEdConfirm (LrWpanPhyEnumeration status, uint8_t energyLevel)
{
  NS_LOG_FUNCTION (this << status << energyLevel);
}
  
void
LeachMac::PlmeGetAttributeConfirm (LrWpanPhyEnumeration status,
                                   LrWpanPibAttributeIdentifier id,
                                   LrWpanPhyPibAttributes* attribute)
{
  NS_LOG_FUNCTION (this << status << id << attribute);
  
  /*
  NS_LOG_DEBUG( " Cur chan " << (int)attribute->phyCurrentChannel );
  NS_LOG_DEBUG( " TX power " << (int)attribute->phyTransmitPower );
  NS_LOG_DEBUG( " CCA mode " << (int)attribute->phyCCAMode );
  NS_LOG_DEBUG( " Cur page " << attribute->phyCurrentPage );
  NS_LOG_DEBUG( " MAX frame " << attribute->phyMaxFrameDuration );
  NS_LOG_DEBUG( " SHR dura " << attribute->phySHRDuration );
  NS_LOG_DEBUG( " Symb Oct " << attribute->phySymbolsPerOctet );
   */
}

void
LeachMac::PlmeSetTRXStateConfirm (LrWpanPhyEnumeration status)
{
  NS_LOG_FUNCTION ( this << g_phy_string[status] << " pkt " << m_txPkt );
  
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
        
        //NS_LOG_DEBUG( "\t T diff = " << tDiff );
        //NS_LOG_DEBUG( "\t prev st " << g_phy_string[m_previousState] << " new st " << g_phy_string[status] );
        //NS_LOG_DEBUG( "\t cur mW drain rate " << m_mWpwr );
        
        // Now we can update the previous state and the previous time to the
        // status we just received from the PHY
        m_timePrevState = tcurrent;
        m_previousState = status;
        
        //NS_LOG_DEBUG( "\t cur sys pwr " << m_totalSystemPower );

        // Now reduce the power level of the system as a function of mJ/s
        m_totalSystemPower -= ( m_mWpwr * tDiff.GetSeconds() );
        //NS_LOG_DEBUG( "\t new sys pwr " << m_totalSystemPower );
        
        // Shut down this sensor if the system power drops below the minimum
        // power required to participate in the sensor network
        if ( m_totalSystemPower <= 0.0 )
        {
          NS_LOG_DEBUG( "\t NODE DEAD " );
          m_systemEnabled = false;
          ChangeLeachState( LEACH_NODE_DEAD );
          
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
        else
          NS_LOG_DEBUG( "\t Still alive" );
      }
      else
        NS_LOG_DEBUG( m_Address << " enabled " << m_systemEnabled );
    }
    //    else
    //      NS_LOG_DEBUG( m_Address << " no state change - ignore power calc ");
  }

  if ( m_systemEnabled )
  {
    if (m_sensorMacState == SENSOR_SENDING && (status == IEEE_802_15_4_PHY_TX_ON || status == IEEE_802_15_4_PHY_SUCCESS))
    {
      NS_ASSERT (m_txPkt);
      NS_LOG_INFO( " Node " << m_Address << " pkt " << m_txPkt << " is calling Phy->PdDataRequest ");
      
      // Start sending if we are in state SENDING and the PHY transmitter was enabled.
      m_promiscSnifferTrace (m_txPkt);
      m_snifferTrace (m_txPkt);
      m_macTxTrace (m_txPkt);
      m_phy->PdDataRequest (m_txPkt->GetSize (), m_txPkt);
    }
    else if (m_sensorMacState == SENSOR_CSMA && (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS))
    {
      NS_LOG_DEBUG("\tPlmeSetTRXStateConfirm CSMA start");
      // Start the CSMA algorithm as soon as the receiver is enabled.
      m_csmaCa->Start ();
    }
    else if (m_sensorMacState == SENSOR_IDLE)
    {
      NS_ASSERT (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS || status == IEEE_802_15_4_PHY_TRX_OFF);
      
      NS_LOG_DEBUG("\tPlmeSetTRXStateConfirm Sensor Idle @ " << Now() << " amCH " << iAmClusterHead << " myState " << leachString[m_leachMacState] );
      // Do nothing special when going idle.
      
      // Check to see if we have to go to sleep when we're a cluster member and done
      // transmitting our data
      if ( !iAmClusterHead && !m_iAmSink )
      {
        switch( m_leachMacState )
        {
          case LEACH_DATA_PHASE:
          {
            NS_LOG_DEBUG( "\tCH-M return to SINK Channel " );
            SensorMac::SetChannel( SINK_CHANNEL );
            SensorMac::SetPower( HIGH_POWER );
            break;
          }
            
          case LEACH_POST_SLEEP:
          {
            //m_sleepFinish += Seconds( 2.0 * m_numClusters * 0.005 );
            NS_LOG_DEBUG( "\tCH-M in POST DATA wait - init @ " << (Now() + m_sleepFinish) );
            ChangeLeachState( LEACH_INIT );
            
            m_joinTimer.Cancel();
            m_joinTimer.SetFunction( &LeachMac::NodeWaitingForInit, this );
            m_joinTimer.Schedule( m_sleepFinish );
            
            // Allow the TRX to go to sleep
            m_macRxOnWhenIdle = false;
            m_setMacState = Simulator::ScheduleNow ( &LeachMac::SetSensorMacState, this, SENSOR_IDLE );
            break;
          }
          
          default:
          {
            NS_LOG_DEBUG( "Not CH and Not SINK and state is " << leachString[m_leachMacState] << " RX IDLE ON " << m_macRxOnWhenIdle << " SenMAC state " << g_chan_state[m_sensorMacState] );
            break;
          }
        }
      }
      // A CH node must return to the primary channel to deliver data to the Sink
      else if ( iAmClusterHead )
      {
        if ( m_leachMacState == LEACH_SINK_DATA )
        {
          NS_LOG_DEBUG( "\t @@@ CH returning to SINK Channel @@@" );
          SensorMac::SetChannel( SINK_CHANNEL );
        }
        else
          NS_LOG_DEBUG( "\tam CH but not in SINK DATA STATE" );
      }
    }
    else if (m_sensorMacState == SENSOR_ACK_PENDING)
    {
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
LeachMac::PlmeSetAttributeConfirm (LrWpanPhyEnumeration status,
                                   LrWpanPibAttributeIdentifier id)
{
  //NS_LOG_DEBUG ("PlmeSetAttributeConfirm status " << status << " id " << id << " @ " << Now() );
  
  switch ( m_leachMacState )
  {
    case LEACH_DATA_PHASE:
    {
      // Once a transmitting CH Member is done transmitting its data, and it's
      // switched the channel back to the SINK channel (11), we can put the
      // transceiver to sleep
      
      NS_LOG_DEBUG( "\t am CH " << iAmClusterHead << " would have gone TRX OFF but not... " );
      
      // Allow the TRX to go to sleep
      //m_macRxOnWhenIdle = false;
      
      m_leachMacState = LEACH_POST_SLEEP;
      m_setMacState = Simulator::ScheduleNow ( &LeachMac::SetSensorMacState, this, SENSOR_IDLE );
      break;
    }
      
    default:
      break;
  }
}

void
LeachMac::ProcessSinkReception( Ptr<Packet> p )
{
  NS_LOG_DEBUG ( "\tSINK Reception" );
  
  LeachMacHeader receivedMacHdr;
  p->RemoveHeader( receivedMacHdr );
  
  SensorDataIndicationParams params;
  params.m_dsn = receivedMacHdr.GetSeqNum();
  params.m_srcAddr = receivedMacHdr.GetSrcAddr();
  params.m_dstAddr = receivedMacHdr.GetDstAddr();
  
  bool forMe = receivedMacHdr.GetDstAddr () == m_Address;
  bool bcast = receivedMacHdr.GetDstAddr () == Mac16Address ("ff:ff");
  LeachMacFrameType type = receivedMacHdr.GetFrameType();
  
  NS_LOG_DEBUG ("Packet from " << receivedMacHdr.GetSrcAddr() << " to " << receivedMacHdr.GetDstAddr() );
  
  // Check for a valid destination address - it's me or a broadcast
  // Otherwise I toss it
  if ( forMe || bcast )
  {
    switch ( m_leachMacState )
    {
      case LEACH_ADVERTISE:
      {
        // We should only ever receive CH advertisements during the ADV CH
        // phase. Anything else is garbage.
        if ( receivedMacHdr.GetFrameType() == ADVERTISE_CH )
        {
          NS_LOG_DEBUG( "\t    RX ADVER CH from " << receivedMacHdr.GetSrcAddr() );
          m_ClusterHeads.push_back( receivedMacHdr.GetSrcAddr() );
        }
        else
          NS_LOG_DEBUG("Invalid frm type " << g_leach_string[receivedMacHdr.GetFrameType()] << " during " << leachString[m_leachMacState] );
        break;
      }
        
      case LEACH_SINK_DATA:
      {
        switch ( type )
        {
          case CH_PYLOD:
          {
            SensorDataIndicationParams params;
            params.m_dsn = receivedMacHdr.GetSeqNum();
            params.m_mpduLinkQuality = 0;
            params.m_srcAddr = receivedMacHdr.GetSrcAddr();
            params.m_dstAddr = receivedMacHdr.GetDstAddr();
            
            NS_LOG_DEBUG( "CH_PAYLOAD from CH " << params.m_srcAddr );
            //NS_LOG_DEBUG ("\tDataIndication():  Packet is for SINK (me); forwarding up");
            m_sensorDataIndicationCallback(params, p);
            
            //
            // Now extract the Addresses and Sequences
            //
            uint16_t addr;
            uint32_t seq;
            std::list<uint16_t> nodes;
            std::list<uint32_t> sequence;
            
            uint16_t length = receivedMacHdr.GetLength();
            
            uint8_t msg[ length ];
            
            NS_LOG_DEBUG( "\t  payload " << length );
            
            p->CopyData( msg, length );
            
            unsigned int numNodes = ( length / 6 );
            unsigned int idx = 0;
            
            for ( unsigned int i = 0; i < numNodes; ++i )
            {
              addr = msg[ idx++ ];
              addr |= ( ( ((uint16_t)msg[ idx++ ]) << 8) & 0xFF00 );
              
              addr = ntohs( addr );
              
              seq = msg[ idx++ ];
              seq |= ( ( ((uint32_t)msg[ idx++ ]) <<  8) & 0x0000FF00 );
              seq |= ( ( ((uint32_t)msg[ idx++ ]) << 16) & 0x00FF0000 );
              seq |= ( ( ((uint32_t)msg[ idx++ ]) << 24) & 0xFF000000 );
              
              NS_LOG_DEBUG("\t   addr: " << addr << " seq " << seq );
              nodes.push_back( addr );
              sequence.push_back( seq );
            }
            
            m_sensorSinkDataRcvdCallback( nodes, sequence );
            break;
          }
            
          case DATA_FRAME:
          {
            SensorDataIndicationParams params;
            params.m_dsn = receivedMacHdr.GetSeqNum();
            params.m_mpduLinkQuality = 0;
            params.m_srcAddr = receivedMacHdr.GetSrcAddr();
            params.m_dstAddr = receivedMacHdr.GetDstAddr();

            NS_LOG_DEBUG( "DATA FRAME from DC" );
            //NS_LOG_DEBUG ("\tDataIndication():  Packet is for SINK (me); forwarding up");
            m_sensorDataIndicationCallback(params, p);
            
            NS_LOG_DEBUG("\t   addr: " << params.m_srcAddr << " seq " << receivedMacHdr.GetSeqNum() );
            
            m_sensorSinkDCRcvdCallback( params.m_srcAddr, receivedMacHdr.GetSeqNum() );
            break;
          }
            
          default:
          {
            NS_LOG_DEBUG( "\t\tInvalid type while in SINK DATA " );
            m_macRxDropTrace ( p );
            break;
          }
        }
        break;
      }

      default:
      {
        NS_LOG_DEBUG("Invalid frm type " << receivedMacHdr.GetFrameType() << " during " << leachString[m_leachMacState] );
        m_macRxDropTrace ( p );
        break;
      }
    }
  }
}

void
LeachMac::ProcessClusterHeadReception( Ptr<Packet> p, uint8_t lqi )
{
  NS_LOG_DEBUG ( "\tCH Reception" );
  
  LeachMacHeader receivedMacHdr;
  p->RemoveHeader (receivedMacHdr);
  
  SensorDataIndicationParams params;
  params.m_dsn = receivedMacHdr.GetSeqNum();
  params.m_mpduLinkQuality = lqi;
  params.m_srcAddr = receivedMacHdr.GetSrcAddr();
  params.m_dstAddr = receivedMacHdr.GetDstAddr();
  
  bool forMe = receivedMacHdr.GetDstAddr () == m_Address;
  bool bcast = receivedMacHdr.GetDstAddr () == Mac16Address ("ff:ff");
  LeachMacFrameType type = receivedMacHdr.GetFrameType();
  
  NS_LOG_DEBUG( "\t    RX " << g_leach_string[type] << " in state " << leachString[m_leachMacState] );
  NS_LOG_DEBUG ("\t    Packet from " << params.m_srcAddr << " to " << params.m_dstAddr);
  
  // Check for a valid destination address - it's me or a broadcast
  // Otherwise I toss it
  if ( forMe || bcast )
  {
    switch ( m_leachMacState )
    {
      case LEACH_ADVERTISE:
      {
        switch ( type )
        {
          case ADVERTISE_CH:
          {
            // Parse the packet as if we were not a potential CH
            // just in case the SINK does not select us
            ParseChAdvertisement( p, lqi, params.m_srcAddr );
            break;
          }
            
          case ADD_MORE_CHS:
          {
            NS_LOG_DEBUG( "\tDO NOTHING" );
            // Do Nothing - we wait for Final CH list
            break;
          }
            
          case RX_FINAL_CH_LIST:
          {
            struct NodeList_s addresses;
            int len = p->CopyData( (uint8_t *)&addresses, receivedMacHdr.GetLength() );
            
            NS_LOG_DEBUG( "\tcopied data from packet: " << addresses.addresses[0] );
            
            if ( len != receivedMacHdr.GetLength() )
              NS_LOG_ERROR(" ***** CH: Invalid data length for RX_FINAL_CH_LIST packet ***** ");
            else
            {
              bool imPresent = false;
        
              // Convert length to the number of addresses
              len /= 2;
              
              for (int i = 0; i < len; ++i )
              {
                Mac16Address temp;
                temp.CopyFrom( (const uint8_t *) &(addresses.addresses[i]) );
                
                if ( temp == m_Address )
                {
                  imPresent = true;
                  m_myChPos = i;
                  m_myCHchan = SINK_CHANNEL + m_myChPos + 1;
                  NS_LOG_DEBUG( " \t mh chan " << m_myCHchan );
                  NS_LOG_DEBUG( " \t save " << m_myChPos << " for sink data later on " );
                  break;
                }
              }

              // Generic timer value used for CHJoinDoneCheck or DC Transmission
              double msTillTx = ( 2.5 * (m_maxNumNodes - m_numClusters) * 0.006 );
              m_schdDoneTime = Seconds( msTillTx ) + Now();
              
              // Disable any previous CSMA if we did not complete our TX as a potential CH
              if ( m_setMacState.IsRunning() )
              {
                NS_LOG_DEBUG( " Canceling previous state change event " << g_chan_state[m_sensorMacState] );
                m_setMacState.Cancel ();
                m_setMacState = Simulator::ScheduleNow (&LeachMac::SetSensorMacState, this, SENSOR_IDLE);
              }

              if ( imPresent )
              {
                // Wait for the JOIN REQ messages
                // TODO: Add a timeout for the JOIN REQuests - so we know
                //       when to send our final list
                NS_LOG_DEBUG( "I was selected as CH by SINK" );
                
                // Move to my channel to get JOIN requests
                SensorMac::SetChannel( m_myCHchan );
                
                ChangeLeachState( LEACH_SCHEDULE );
                
                // Setup a timeout for JOIN REQs to complete - return to SINK chan to get kickoff
                NS_LOG_DEBUG( " CH will wait for JOINs to complete @ " << Now() + Seconds( msTillTx ) );
                m_joinTimer.Cancel();
                m_joinTimer.SetFunction( &LeachMac::CHJoinDoneCheck, this );
                m_joinTimer.Schedule( Seconds( msTillTx ) );
              }
              else
              {
                NS_LOG_DEBUG( "I was NOT selected as CH by SINK - switching to normal node" );
                // Switch to CH Member status, as SINK did not want us as CH
                iAmClusterHead = false;
                hasBeenClusterHead = false;
                m_prospectCh.addr = ns3::Mac16Address( "00:00" );
                
                // We change state to the TDMA Scheduling phase - if we have at least
                // one CH in our list, we need to attempt a join request.  If we have no
                // CH's, then we need to move into a DIRECT CONNECT mode
                if ( m_bestCh.size() > 0 )
                {
                  if ( ExtractViableClusterHeads( p, receivedMacHdr.GetLength() ) )
                  {
                    ChangeLeachState( LEACH_SCHEDULE );
                    
                    checkCounter++;
                    
                    // Setup a timer to determine if we've joined with a CH -
                    // if not, we need to go DC to the SINK
                    NS_LOG_DEBUG( "\t  check if CH-M achieved in " << msTillTx << "ms - @ " << m_schdDoneTime << " cntr " << checkCounter << "  : SensorMAC state " << g_chan_state[m_sensorMacState] );
                    //                Simulator::Schedule( m_schdDoneTime, &LeachMac::CheckIfClusterMember, this );
                    
                    m_joinTimer.Cancel();
                    m_joinTimer.SetFunction( &LeachMac::CheckIfClusterMember, this );
                    m_joinTimer.Schedule( Seconds( msTillTx ) );
                  }
                  else
                    LaunchDirectConnect( m_schdDoneTime );
                }
                else
                {
                  LaunchDirectConnect( m_schdDoneTime );
                }
              }
            }
            break;
          }
            
          default:
          {
            NS_LOG_DEBUG( "\t\tInvalid type while in ADVERTISE " );
            m_macRxDropTrace ( p );
            break;
          }
        }
        break;
      }
        
      case LEACH_SCHEDULE:
      {
        switch ( type )
        {
          case JOIN_REQUEST:
          {
            // Add the node to the list of members for the CH
            Ptr<Packet> pkt;
            
            if ( (int)m_MemberNodes.size() < (m_maxNumNodes / m_numClusters) )
            {
              // Add them to our cluster and NACK them
              m_MemberNodes.push_back( receivedMacHdr.GetSrcAddr() );
              pkt = CreatePacket( JOIN_ACK, GetAddress(), params.m_srcAddr, m_macDsn.GetValue() );
              NS_LOG_DEBUG( " Reply with an ACK " << pkt );
            }
            else
            {
              // JOIN NACK - too many nodes in our group
              pkt = CreatePacket( JOIN_NACK, GetAddress(), params.m_srcAddr, m_macDsn.GetValue() );
              NS_LOG_DEBUG( " Reply with a NACK " << pkt );
            }
            
            m_macDsn++;
            
            // Trace the fact that we're queued up a ADV_CH frame
            m_macTxEnqueueTrace( pkt );
            m_txPkt = pkt;
            
            debugForceTx = joinRequestMessage;
            
            // Kick off the ACK transmission 10us
            Time holdTime = Seconds (0.0001);
            NS_LOG_DEBUG( "\t Calling SENSOR_FORCE_TX 3 @ " << ( Now() + holdTime ) );
            m_forceTxEvent = Simulator::Schedule ( holdTime, &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX);
            break;
          }
            
          case TDMA_KICK_OFF:
          {
            NS_LOG_DEBUG( "\t\tCH needs to change channels");
            SensorMac::SetChannel( m_myCHchan );
            
            // If other nodes joined my group, I'm a CH - otherwise, I'm a direct connect
            if ( !m_MemberNodes.empty() )
            {
              NodeList_s msg;
              
              // 116 / 2 = 58 addresses in a single ADVERTISE_SCHEDULE message
              const int maxAddresses = ((MAX_PAYLOAD_LENGTH - 1) / 2) - 1;
              int qdframes = 0;
              int maxframes = ceil((double)m_MemberNodes.size() / (double)maxAddresses);
              
              NS_LOG_DEBUG( "\t  # nodes " << m_MemberNodes.size() << " maxframes " << maxframes << " max addrs " << maxAddresses );
              
              do
              {
                // Add all members to msg (Max 58)
                int idx = 0;
                int maxcnt = 0;
                while ( !m_MemberNodes.empty() && (maxcnt < maxAddresses) )
                {
                  Mac16Address addr = m_MemberNodes.front();
                  NS_LOG_DEBUG("\t   copied " << addr << " member address val " << m_MemberNodes.front() );
                  
                  addr.CopyTo( (uint8_t *) &(msg.addresses[idx]) );
                  NS_LOG_DEBUG( "\t\tidx " << idx << " maxcnt " << maxcnt << " max " << maxAddresses );
                  
                  m_MemberNodes.pop_front();
                  ++idx;
                  ++maxcnt;
                }
                
                ++qdframes;
                
                // The sequence number is used for an alternate purpose in the ADVERTISE_SCHEDULE
                // message.  Since a CH may have to transmit more than one ADVERTISE_SCHEDULE message,
                // we can't let the members kick off with the first packet. We have to provide a
                // Number of Messages ratio.  The upper 16-bit word is the current sequence number
                // where the lower 16-bit word is the max number of frames the CH intends to transmit
                uint32_t sequence = 0;
                sequence |= (((uint32_t)qdframes & 0x0FFFFlu) << 16);
                sequence |= ((uint32_t)maxframes & 0x0FFFFlu);
                NS_LOG_DEBUG( "Sequence we're sending is " << sequence );
                
                Ptr<Packet> pkt = CreatePacketWithData( ADVERTISE_SCHEDULE,
                                                        m_Address,
                                                        Mac16Address ("ff:ff"),
                                                        sequence,
                                                        (uint8_t const*)&msg,
                                                        (2 * idx) );
                m_macDsn++;

                NS_LOG_DEBUG( "\t\t  add frame " << qdframes << " to TXQ w/ pkt " << pkt );
                // Trace the fact that we're queued up a ADV_CH frame
                m_macTxEnqueueTrace( pkt );
                TxQueueElement *txQElement = new TxQueueElement;
                txQElement->txQPkt = pkt;
                m_txQueue.push_back( txQElement );
              
              } while ( qdframes < maxframes );
              
              if ( maxframes > 1 )
                m_delayAdvSch = Seconds(0.001);
              else
                m_delayAdvSch = Seconds(0.0);
              
              // Set the next TX packet once the transciever is enabled
              m_txPkt = m_txQueue.front()->txQPkt->Copy();
              m_txQueue.pop_front();
              
              NS_LOG_DEBUG( "transmitting ADVERTISE_SCHEDULE " << m_txPkt );
              
              debugForceTx = advertiseScheduleMessage;
              
              // Kick off the CH TDMA SCH transmission 10us after the end of CH Membership selection
              Time holdTime = Seconds (0.00001);
              NS_LOG_DEBUG( "\t Calling SENSOR_FORCE_TX 4 @ " << ( Now() + holdTime ) );
              m_forceTxEvent = Simulator::Schedule ( holdTime, &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX);
              
              ChangeLeachState( LEACH_DATA_PHASE );
              
              // Setup a timer for SINK to wait for DATA PHASE / DC's to complete
              //double msTillTx = ( idx * 0.006 );
              //double msTillTx = ( (m_maxNumNodes + m_numClusters) * 0.005 );
              double msTillTx = TDMA_TIMEOUT(0);
              NS_LOG_DEBUG( " \n\t****\t\t****\n" );
              NS_LOG_DEBUG( " CH will wait for DATA PHASE to complete @ " << Now() + Seconds( msTillTx ) );
              NS_LOG_DEBUG( " \n\t****\t\t****\n" );
              m_joinTimer.Cancel();
              m_joinTimer.SetFunction( &LeachMac::CHdataPhaseComplete, this );
              m_joinTimer.Schedule( Seconds( msTillTx ) );
              
              // Inform Helper that I'm a CH
              if ( !m_sensorForwaderCallback.IsNull() )
                m_sensorForwaderCallback ( m_Address );
            }
            else
            {
              // There's no one in my group, change to a direct connect node
              iAmClusterHead = false;
              ChangeLeachState( LEACH_DIRECT_CONN );
              
              // Use CSMA to transmit my data directly to the SINK during the DATA PHASE
              DirectConnectTransmission();
            }
            break;
          }
            
          default:
          {
            NS_LOG_DEBUG( "\t\tInvalid type while in SCHEDULE " );
            m_macRxDropTrace ( p );
            break;
          }
        }
        
        break;
      }
        
      case LEACH_DATA_PHASE:
      {
        switch ( type )
        {
          case DATA_FRAME:
          {
            NS_LOG_DEBUG("\tRX packet in DATA RX Mode from " << params.m_srcAddr);
            
            // Add the source address to the list of received msg's for future transmit
            m_rcvdMembers.push_back( params.m_srcAddr );
            
            // Copy at least
            DataPayload_s msg;
            p->CopyData( (uint8_t *) &msg, 6);
            
            NS_LOG_DEBUG( "\t\tmsg seq " << msg.sequence );
            m_rcvdSequences.push_back( msg.sequence );
            
            if ( !m_sensorCHDataRcvdCallback.IsNull() )
              m_sensorCHDataRcvdCallback( params.m_srcAddr, msg.sequence );
            break;
          }
            
          default:
          {
            NS_LOG_DEBUG( "\t\tInvalid type while in DATA_PHASE " );
            m_macRxDropTrace ( p );
            break;
          }
        }
        break;
      }
        
      case LEACH_POST_SLEEP:
      case LEACH_SINK_DATA:
      {
        switch ( type )
        {
          case SINK_DATA:
          {
            NS_LOG_DEBUG( "\t****** CH " << m_Address << " start SINK_DATA ******" );
            
            NS_LOG_DEBUG( "\t\tCH needs to change channels back to SINK");
            SensorMac::SetChannel( SINK_CHANNEL );
            
            /*
             * We need to allow for any CH to transmit at least 75% of the members' 
             * payloads.  There is asmall probability that a CH has to transmit all,
             * but best guess would be that it's extremely low.
             *
             * T_slots = 0.0051             ( 5.1ms )
             * Bytes_tot = (# nodes * 6)
             * Frames_max = ceil( Bytes_tot / max_len )    where: max len is 116 bytes
             * 
             * CH slots = ceil( Frames_max * 0.75 )
             * # slost = CH * CH_slots
             *
             * Each CH gets at least one slot, and they're consecutive
             */
            
            /*
            double T_slot = 0.0051;
            double Bytes_tot = m_maxNumNodes * 6.0;
            double frames = ceil( Bytes_tot / MAX_PAYLOAD_LENGTH );
            double ch_slots = ceil( frames * 0.75 );
            double tot_slots = m_numClusters * ch_slots;
            
            // Setup a timer for CH to wait for SINK DATA PHASE to complete and return to INIT
            double msTillTx = tot_slots * T_slot;        // ( m_numClusters * 0.006 ) + 0.005;
            */
            double msTillTx = SINK_DAT_TIMEOUT - (2*TDMASLOT);
            NS_LOG_DEBUG( " CH will return to InitPhaseTimeout @ " << Now() + Seconds( msTillTx ) );
            m_joinTimer.Cancel();
            m_joinTimer.SetFunction( &LeachMac::InitPhaseTimeout, this );
            m_joinTimer.Schedule( Seconds( msTillTx ) );
            
            SinkDelivery( m_myChPos );
            break;
          }
            
          default:
          {
            NS_LOG_DEBUG( "\t\tInvalid type while in POST_SLEEP " );
            m_macRxDropTrace ( p );
            break;
          }
        }
        break;
      }
        
      default:
      {
        NS_LOG_DEBUG("Invalid frm type " << receivedMacHdr.GetFrameType() << " during " << leachString[m_leachMacState] );
        m_macRxDropTrace ( p );
        break;
      }
    }
  }
}

void
LeachMac::ProcessClusterMemberReception( Ptr<Packet> p )
{
  NS_LOG_DEBUG ( "\tCluster Member Reception" );
  
  LeachMacHeader receivedMacHdr;
  p->RemoveHeader (receivedMacHdr);
  
  SensorDataIndicationParams params;
  params.m_dsn = receivedMacHdr.GetSeqNum();
  params.m_mpduLinkQuality = 0;
  params.m_srcAddr = receivedMacHdr.GetSrcAddr();
  params.m_dstAddr = receivedMacHdr.GetDstAddr();
  
  bool forMe = receivedMacHdr.GetDstAddr () == m_Address;
  bool bcast = receivedMacHdr.GetDstAddr () == Mac16Address ("ff:ff");
  LeachMacFrameType type = receivedMacHdr.GetFrameType();
  
  NS_LOG_DEBUG( "\t    RX " << g_leach_string[type] << " in state " << leachString[m_leachMacState] );
  NS_LOG_DEBUG ("\t    Packet from " << params.m_srcAddr << " to " << params.m_dstAddr);
  
  // Check for a valid destination address - it's me or a broadcast
  // Otherwise I toss it
  if ( forMe || bcast )
  {
    switch ( m_leachMacState )
    {
      case LEACH_WAIT_TDMA:
      {
        switch ( type )
        {
          case ADVERTISE_SCHEDULE:
          {
            // Make sure we acknowledge the CH is alive now that there is a TDMA schedule
            m_chIsAlive = true;
            
            int mypos = -1;
            bool lastFrame = false;
            int length = receivedMacHdr.GetLength();
            
            uint16_t frame, maxframes = 0;

            NS_LOG_DEBUG( "\tMember " << m_Address << " Rcvd ADV SCH -- rxframes " << m_rxAdvSchFrames << " posOffset " << m_advPosOffset << " foundSpot " << m_advFoundSpot );
            
            uint32_t sequence = receivedMacHdr.GetSeqNum();
            NS_LOG_DEBUG( "\t   seq from header " << sequence );
            frame = (uint16_t)((sequence & 0xFFFF0000lu) >> 16);
            maxframes = (uint16_t)(sequence & 0x00000FFFFlu);
                                   
            lastFrame = (frame == maxframes);
            
            NS_LOG_DEBUG( "\t  packet is " << length << " bytes" );
            NS_LOG_DEBUG( "\t  frame " << frame << " of " << maxframes );
            
            uint8_t buffer[length];
            uint16_t count = length / 2;
            
            p->CopyData( buffer, length );
            
            // Don't bother parsing the list if we already found our TDMA slot
            if ( !m_advFoundSpot )
            {
              for ( int i = 0; i < count; ++i )
              {
                Mac16Address addr;
                addr.CopyFrom( &(buffer[i * 2]) );
                
                NS_LOG_DEBUG("\t   copy addr " << addr << " from " << (uint16_t)(buffer[count * 2]) );
                // Check to see if our address is at index i
                if ( addr == m_Address )
                {
                  mypos = i + m_advPosOffset;
                  m_advFoundSpot = true;
                  NS_LOG_DEBUG("found mypos " << mypos << " in TDMA schedule " );
                  break;
                }
              }
            }
            else
            {
              // If we already found our spot - and we're here, then this is a secondary
              // frame.  Thus we need to set the mypos == m_advPosOffset from the last frame
              mypos = m_advPosOffset;
            }
            
            if ( !lastFrame )
            {
              if ( !m_advFoundSpot )
              {
                // hold over number of addresses in this frame for the next reception to get correct TDMA slot
                m_advPosOffset += count;
                mypos = -1;
                NS_LOG_DEBUG( "\tpos yet to be found" );
              }
              else
              {
                NS_LOG_DEBUG( "\t!lstFrm and Have Pos..." );
                m_advPosOffset = mypos;
              }
              
              // NOTE: Don't add offset to pos here, least it keep getting shifted
            }
            else
            {
              m_chIsDeadTimer.Cancel();

              // We reached the last frame - now assign slot time and reset static variables
              if ( mypos < 0 )
              {
                NS_LOG_DEBUG("\tFailed to acquire TDMA slot");
                m_rcvdTdmaList = false;
                m_advFoundSpot = false;
                // Switch to a DC node
                DirectConnectTransmission();
              }
              else
              {
                m_rcvdTdmaList = true;
                
                /*
                 * If I'm listed as the first transmitter, go ahead and start
                 * the transmission of my data payload. Otherwise, I have to
                 * wait in a powered down state to save energy.
                 */
                if ( mypos == 0 )
                {
                  // Pop the data from the m_dataQueue if a packet is available
                  if ( !m_dataQueue.empty() )
                  {
                    m_txPkt = m_dataQueue.front()->txQPkt->Copy();
                    m_dataQueue.pop_front();
                    
                    if ( m_txPkt == 0 )
                      NS_FATAL_ERROR( " CH-M has no data to send " << m_txPkt );
                    
                    // Replace the Dest Address with the CH Address
                    LeachMacHeader tmpHdr;
                    m_txPkt->RemoveHeader (tmpHdr);
                    
                    uint32_t seq = tmpHdr.GetSeqNum();
                    NS_LOG_DEBUG ("\t transmit seq " << seq << " pkt " << m_txPkt << " src " << tmpHdr.GetSrcAddr() );
                    
                    tmpHdr.SetDstAddr( m_prospectCh.addr );
                    m_txPkt->AddHeader (tmpHdr);
                    
                    ChangeLeachState( LEACH_DATA_PHASE );
                    
                    if ( !m_sensorDataTransmitCallback.IsNull() )
                      m_sensorDataTransmitCallback( seq, m_Address );
                    
                    // The first sleep cycle for the first transmitter is skipped
                    // as it must turn around a transmission right away
                    m_sleepStart = Seconds( 0.0 );
                    
                    // Sleep timer is 4ms / TDMA slot - wait until all nodes are
                    // finished before we wake up the RF, assuming the number of
                    // nodes is correct - too many nodes will force us to wake
                    // up early due to DATA PHASE timeout
                    //double msTillTx = ( (m_maxNumNodes + m_numClusters - 1) * 0.005 );
                    double msTillTx = TDMA_TIMEOUT(-1) + SINK_DAT_TIMEOUT;
                    m_sleepFinish = Seconds( msTillTx );
                    //m_sleepFinish = Seconds( (count - 1) * 0.005 );
                    
                    NS_LOG_DEBUG( " Slot 0 Node " << m_Address << " sleep start " << m_sleepStart << " stop " << m_sleepFinish );
                    NS_LOG_DEBUG( "   Transmitting data to CH " << m_txPkt );
                    
                    debugForceTx = nodeZeroData;
                    // Kick off the DATA transmission 100us
                    Time holdTime = Seconds (0.0001);
                    NS_LOG_DEBUG( "\t Calling SENSOR_FORCE_TX 5 @ " << ( Now() + holdTime ) );
                    m_forceTxEvent = Simulator::Schedule ( holdTime, &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX);
                  }
                  else
                    NS_FATAL_ERROR("No Data PDU ready for LEACH DATA slot");
                }
                else
                {
                  ChangeLeachState( LEACH_PRE_SLEEP );
                  
                  // calculate the Sleep time remaining after we transmit
                  //double msTillTx = ( (m_maxNumNodes + m_numClusters) * 0.005 ) - ((mypos + 1) * 0.005);
                  double msTillTx = TDMA_TIMEOUT(0) - ((mypos + 1) * TDMASLOT) + SINK_DAT_TIMEOUT;
                  //m_sleepFinish = Seconds( (count - mypos) * 0.005 );
                  m_sleepFinish = Seconds( msTillTx );

                  NS_LOG_DEBUG( " Slot " << mypos << " Node " << m_Address << " sleep start " << (mypos * 0.005) << " stop " << m_sleepFinish );
                  NS_LOG_DEBUG( "   Transmitting data to CH " << m_txPkt );

                  // Allow the TRX to go to sleep
                  m_macRxOnWhenIdle = false;
                  m_setMacState = Simulator::ScheduleNow ( &LeachMac::SetSensorMacState, this, SENSOR_IDLE );
                  
                  msTillTx = mypos * TDMASLOT;
                  
                  // Setup a timer for me to transmit in my TDMA slot
                  m_waitTimer.Cancel();
                  m_waitTimer.SetFunction( &LeachMac::WaitTimerExpired, this );
                  m_waitTimer.Schedule( Seconds( msTillTx ) );
                }
              }
              
              m_rxAdvSchFrames = 0;
              m_advPosOffset = 0;
              m_advFoundSpot = false;
            }
            break;
          }
            
          default:
          {
            NS_LOG_DEBUG( "\t\tInvalid type while in WAIT TDMA " );
            m_macRxDropTrace ( p );
            break;
          }
        }
        
        break;
      }
        
      case LEACH_DIRECT_CONN:
      {
        switch( type )
        {
          case TDMA_KICK_OFF:
          {
            NS_LOG_DEBUG( "\t Direct Call to TxDirectConnect with " << m_txQueue.size() << " pkts in TxQ " );
            TxDirectConnect();
            break;
          }
            
          default:
          {
            NS_LOG_DEBUG( "\t\tInvalid type while in DIRECT CONN" );
            m_macRxDropTrace ( p );
            break;
          }
        }
        
        break;
      }
      
      default:
      {
        NS_LOG_DEBUG("Invalid frm type " << receivedMacHdr.GetFrameType() << " during " << leachString[m_leachMacState] );
        m_macRxDropTrace ( p );
        break;
      }
    }
  }
}

void
LeachMac::ProcessNodeReception( Ptr<Packet> p, uint8_t lqi  )
{
  NS_LOG_DEBUG ( "\tNODE Reception" );
  
  LeachMacHeader receivedMacHdr;
  p->RemoveHeader (receivedMacHdr);
  
  SensorDataIndicationParams params;
  params.m_dsn = receivedMacHdr.GetSeqNum();
  params.m_mpduLinkQuality = lqi;
  params.m_srcAddr = receivedMacHdr.GetSrcAddr();
  params.m_dstAddr = receivedMacHdr.GetDstAddr();
  
  bool forMe = receivedMacHdr.GetDstAddr () == m_Address;
  bool bcast = receivedMacHdr.GetDstAddr () == Mac16Address ("ff:ff");
  LeachMacFrameType type = receivedMacHdr.GetFrameType();
  
  NS_LOG_DEBUG( "\t    RX " << g_leach_string[type] << " in state " << leachString[m_leachMacState] );
  NS_LOG_DEBUG ("\t    Packet from " << params.m_srcAddr << " to " << params.m_dstAddr);
  
  // Check for a valid destination address - it's me or a broadcast
  // Otherwise I toss it
  if ( forMe || bcast )
  {
    switch ( m_leachMacState )
    {
      case LEACH_INIT:
      {
        // We need to get the superframe from the SINK and no one else
        // We do nothing until we hear the superframe
        if ( ( type == SUPER_FRM_ANNOUNCE ) &&
             ( receivedMacHdr.GetSrcAddr() == Mac16Address ("00:01") ) )
        {
          // We change state to the ADV CH regardless of our self election
          ChangeLeachState( LEACH_ADVERTISE );
          
          // Determine if I need to advertise myself as a CH for this round
          NextRoundDecision();
        }
        else
        {
          NS_LOG_DEBUG( "\t\tInvalid frm type " << receivedMacHdr.GetFrameType() << " during " << leachString[m_leachMacState] );
          m_macRxDropTrace ( p );
        }
        break;
      }
        
      case LEACH_ADVERTISE:
      {
        switch ( type )
        {
          case ADVERTISE_CH:
          {
            ParseChAdvertisement( p, lqi, params.m_srcAddr );
            break;
          }
            
          case ADD_MORE_CHS:
          {
            // Repeat the process of checking if I need to advertise myself as a CH for this round
            CheckThisRoundAgain();
            break;
          }
            
          case RX_FINAL_CH_LIST:
          {
            // Generic timer value used for CHJoinDoneCheck or DC Transmission
            double msTillTx = ( 2.5 * (m_maxNumNodes - m_numClusters) * 0.006 );
            m_schdDoneTime = Seconds( msTillTx ) + Now();
            
            // We change state to the TDMA Scheduling phase - if we have at least
            // one CH in our list, we need to attempt a join request.  If we have no
            // CH's, then we need to move into a DIRECT CONNECT mode
            if ( m_bestCh.size() > 0 )
            {
              if ( ExtractViableClusterHeads( p, receivedMacHdr.GetLength() ) )
              {
                ChangeLeachState( LEACH_SCHEDULE );
                
                checkCounter++;
                
                // Setup a timer to determine if we've joined with a CH -
                // if not, we need to go DC to the SINK
                NS_LOG_DEBUG( "\t  check if CH-M achieved in " << msTillTx << "ms - @ " << m_schdDoneTime << " cntr " << checkCounter );
                //                Simulator::Schedule( m_schdDoneTime, &LeachMac::CheckIfClusterMember, this );
                
                m_joinTimer.Cancel();
                m_joinTimer.SetFunction( &LeachMac::CheckIfClusterMember, this );
                m_joinTimer.Schedule( Seconds( msTillTx ) );
              }
              else
                LaunchDirectConnect( m_schdDoneTime );
            }
            else
            {
              LaunchDirectConnect( m_schdDoneTime );
            }
            
            break;
          }
            
          default:
          {
            NS_LOG_DEBUG( "\t\tInvalid type while in ADVERTISE " );
            m_macRxDropTrace ( p );
            break;
          }
        }
        break;
      }

      case LEACH_SCHEDULE:
      {
        switch ( type )
        {
          case JOIN_ACK:
          {
            m_currentCh = m_prospectCh.addr;
            
            NS_LOG_DEBUG( "  " << m_Address << " switching to channel " << m_prospectCh.chan << " pwr @ " << m_prospectCh.pwr );
            
            SensorMac::SetChannel( m_prospectCh.chan );
            SensorMac::SetPower( m_prospectCh.pwr );
            
            // We received our ACK, so we can record we're no longer looking
            m_waitForAck = false;
            m_joinMsgSent = false;
            m_amClusterMember = true;
            NS_LOG_DEBUG( "\t%%%%%%%%%% CH-M status " << m_amClusterMember << " wait4Ack " << m_waitForAck << " joinMsgSnt " << m_joinMsgSent << " %%%%%\n ");
            
            ChangeLeachState( LEACH_WAIT_TDMA );
            break;
          }
            
          case JOIN_NACK:
          {
            // If we can't find a new CH, then we need to move into a Direct Connect Mode
            if ( !GetNextViableClusterHead() )
            {
              NS_LOG_DEBUG("\t\t no potential CH - moving to DC" );
              LaunchDirectConnect( m_schdDoneTime );
            }
            break;
          }
            
          default:
          {
            NS_LOG_DEBUG( "\t\tInvalid type while in SCHEDULE " );
            m_macRxDropTrace ( p );
            break;
          }
        }
        break;
      }
        
      default:
      {
        NS_LOG_DEBUG("Invalid frm type " << receivedMacHdr.GetFrameType() << " during " << leachString[m_leachMacState] );
        m_macRxDropTrace ( p );
        break;
      }
    }
  }
  else if ( !forMe && ( m_leachMacState == LEACH_SCHEDULE ) )
  {
    switch ( type )
    {
      case JOIN_REQUEST:
        NS_LOG_DEBUG( "\treceived JOIN REQ for someone else - pause CSMA ? " );
        m_pauseCsma = true;
        break;
        
      case JOIN_ACK:
      case JOIN_NACK:
        NS_LOG_DEBUG( "\treceived " << g_leach_string[type] << " for someone else - don't pause CSMA ? " );
        m_pauseCsma = false;
        
        // We need to restart our CSMA process to JOIN with a CH
        NS_LOG_DEBUG( "\tjoinMsgSent " << m_joinMsgSent << " wait4ACK " << m_waitForAck );
        
        if ( m_joinMsgSent && m_waitForAck )
        {
          NS_LOG_DEBUG( "RX'ed an " << g_leach_string[type] << " for " << receivedMacHdr.GetDstAddr() << " while waiting for ACK " );
          
          JoingRequest_s msg;
          msg.chAddress = m_prospectCh.addr;
          Ptr<Packet> p = CreatePacketWithData( JOIN_REQUEST, GetAddress(),
                                               m_prospectCh.addr, m_macDsn.GetValue(),
                                               (uint8_t const*)&msg, sizeof(msg) );
          m_macDsn++;
          m_waitForAck = true;
          m_joinMsgSent = false;
          
          // Trace the fact that we're queued up a JOIN_REQ frame
          m_macTxEnqueueTrace (p);
          m_txPkt = p;
          
          // Each node needs approximately 5ms to transmit, so we need to
          // use CSMA to avoid collisions... so try a random jitter
          double rndVal = m_uniformRandomVariable->GetValue(0.0, 0.5);
          Time waitTime = ( m_maxJoinTime - Now() ) * rndVal;
          
          NS_LOG_DEBUG( "\t  restart JOIN in " << waitTime << " JOIN REQ pkt " << m_txPkt );
          NS_LOG_DEBUG( "\t *** CSMA @ " << waitTime );
          m_csmaEvent.Cancel();
          m_csmaEvent = Simulator::Schedule(waitTime, &LeachMac::SetSensorMacState, this, SENSOR_CSMA);
        }
        else
          NS_LOG_DEBUG( "\t  NO CALL TO SCH CSMA : SensorMAC state " << g_chan_state[m_sensorMacState] );
        break;
        
      default:
        break;
    }
  }
}
  
void
LeachMac::NextRoundDecision( bool calc )
{
  double threshHold = 0.0;

  // Perform the start of round PRNG calculation
  if ( m_currentRound >= m_totalRounds )
  {
    m_currentRound = 0;
    hasBeenClusterHead = false;
  }
  
  if ( calc )
  {
    /*
     * Pi(t) = k / (N - k mod(r,N/k))
     * where k is the expected number of clusters per round
     * N is the total number of sensor nodes in the network
     * and r is the number of rounds that have already passed.
     */
    if ( (m_maxNumNodes - (m_numClusters * m_currentRound)) < 1 )
    {
      threshHold = 1.0;
    }
    else
    {
      threshHold = ((double)m_numClusters) / (m_maxNumNodes - (m_numClusters * m_currentRound));
      
      // Whenever round_ is 0, all nodes are eligible to be cluster-head.
      if ( m_currentRound == 0 )
        hasBeenClusterHead = false;
    }
    
    // If node has been cluster-head in this group of rounds, it will not
    // act as a cluster-head for this round.
    if ( hasBeenClusterHead ) {
      NS_LOG_DEBUG( m_Address << " already CH previously ");
      threshHold = 0.0;
    }
    
    //    NS_LOG_DEBUG( m_Address << " round " << m_currentRound << " threshold " << threshHold );

    double rndVal = m_uniformRandomVariable->GetValue(0.0, 1.0);
    // Compare the PRNG against the threshold to determin if I am a CH this round
    if ( rndVal < threshHold )
    {
      NS_LOG_DEBUG( m_Address << " Is a cluster head for round " << m_currentRound << " at time @ " << Simulator::Now() );
      hasBeenClusterHead = true;
      iAmClusterHead = true;
      
      //set random_access [$self getRandomNumber 0 $opt(ra_adv)]
      //$ns_ at [expr $now_ + $random_access] "$self advertiseClusterHead"
    }
    else
    {
      NS_LOG_DEBUG( m_Address << " not CH for round " << m_currentRound << " rnd val " << rndVal );
      iAmClusterHead = false;
    }
  }
  else
    iAmClusterHead = false;

  // If I determined that I should be a cluster head, then schedule the
  // Cluster Head Advertisement frame
  if ( iAmClusterHead )
  {
    AdvertiseCh_s msg;
    msg.spreadCode = (uint16_t)m_random->GetValue (12, 26);
    m_myCHchan = msg.spreadCode;
    
    Ptr<Packet> p = CreatePacketWithData( ADVERTISE_CH, GetAddress(),
                                         Mac16Address ("ff:ff"), m_macDsn.GetValue(),
                                         (uint8_t const*)&msg, sizeof(msg) );
    m_macDsn++;
    
    // Trace the fact that we're queued up a ADV_CH frame
    m_macTxEnqueueTrace (p);
    m_txPkt = p;
    
    //TxQueueElement *txQElement = new TxQueueElement;
    //txQElement->txQPkt = p;
    //m_txQueue.push_back (txQElement);
    
    // Kick off the first round 1ms into the simulation
    //    Time holdTime = Seconds (0.0001);
    //    Simulator::Schedule ( holdTime, &LeachMac::CheckQueue, this );
    double rndVal = m_uniformRandomVariable->GetValue(0.0, 0.5);
    Time waitTime = MicroSeconds ( rndVal * CH_ADV_TIMEOUT );
          
    NS_LOG_DEBUG( "\tCH ADV kick off in " << waitTime << " ADV CH pkt " << m_txPkt );
    NS_LOG_DEBUG( "\t *** CSMA @ " << waitTime );
    m_csmaEvent.Cancel();
    m_csmaEvent = Simulator::Schedule(waitTime, &LeachMac::SetSensorMacState, this, SENSOR_CSMA);

  }
  else
  {
    if ( calc )
    {
      // Else, I need to listen for Cluster Heads, and pick out the best
      // CH, then transmit my intention to join one
      m_prospectCh.lqi = 0;
      m_prospectCh.addr = Mac16Address ("00:00");
      m_prospectCh.sinr = 0.0;
      m_myCHchan = 0;
    }
  }
}

void
LeachMac::CheckThisRoundAgain()
{
  double threshHold = 0.0;
  
  /*
   * Pi(t) = k / (N - k mod(r,N/k))
   * where k is the expected number of clusters per round
   * N is the total number of sensor nodes in the network
   * and r is the number of rounds that have already passed.
   */
  if ( (m_maxNumNodes - (m_numClusters * m_currentRound)) < 1 )
  {
    threshHold = 1.0;
  }
  else
  {
    threshHold = ((double)m_numClusters) / (m_maxNumNodes - (m_numClusters * m_currentRound));
    
    // Whenever round_ is 0, all nodes are eligible to be cluster-head.
    if ( m_currentRound == 0 )
      hasBeenClusterHead = false;
  }
  
  // If node has been cluster-head in this group of rounds, it will not
  // act as a cluster-head for this round.
  if ( hasBeenClusterHead ) {
    NS_LOG_DEBUG( m_Address << " already CH previously ");
    threshHold = 0.0;
  }
  
  double topend = threshHold * ((m_numClusters > 1) ? (m_numClusters * 0.9) : m_numClusters);
  NS_LOG_DEBUG( m_Address << " round " << m_currentRound << " threshold " << threshHold << " top rnd " << topend );
  
  double rndVal = m_uniformRandomVariable->GetValue(0.0, topend);
  // Compare the PRNG against the threshold to determin if I am a CH this round
  if ( rndVal < threshHold )
  {
    NS_LOG_DEBUG( m_Address << " Is a cluster head for round " << m_currentRound << " at time @ " << Simulator::Now() );
    hasBeenClusterHead = true;
    iAmClusterHead = true;
  }
  else
  {
    NS_LOG_DEBUG( m_Address << " not CH for round " << m_currentRound << " rnd val " << rndVal );
    iAmClusterHead = false;
  }
  
  // If I determined that I should be a cluster head, then schedule the
  // Cluster Head Advertisement frame
  if ( iAmClusterHead )
  {
    AdvertiseCh_s msg;
    msg.spreadCode = (uint16_t)m_random->GetValue (12, 26);
    m_myCHchan = msg.spreadCode;
    
    Ptr<Packet> p = CreatePacketWithData( ADVERTISE_CH, GetAddress(),
                                          Mac16Address ("ff:ff"), m_macDsn.GetValue(),
                                          (uint8_t const*)&msg, sizeof(msg) );
    m_macDsn++;
    
    // Trace the fact that we're queued up a ADV_CH frame
    m_macTxEnqueueTrace (p);
    m_txPkt = p;
    
    // Kick off the first round 1ms into the simulation
    //    Time holdTime = Seconds (0.0001);
    //    Simulator::Schedule ( holdTime, &LeachMac::CheckQueue, this );
    double rndVal = m_uniformRandomVariable->GetValue(0.0, 0.5);
    Time waitTime = MicroSeconds ( rndVal * CH_ADV_TIMEOUT );
    
    NS_LOG_DEBUG( "\tCH ADV kick off in " << waitTime << " ADV CH " << m_txPkt );
    NS_LOG_DEBUG( "\t *** CSMA @ " << waitTime );
    m_csmaEvent.Cancel();
    m_csmaEvent = Simulator::Schedule(waitTime, &LeachMac::SetSensorMacState, this, SENSOR_CSMA);
  }
}
  
void
LeachMac::InitPhaseTimeout (void)
{
  if ( m_systemEnabled )
  {
    NS_LOG_FUNCTION (this << m_iAmSink );
    
    if( m_iAmSink )
      NS_LOG_UNCOND("Next round: " << m_epoch << " time " << Now() );
    
    m_macRxOnWhenIdle = true;
    
    debugForceTx = NULL;
    
    // Wipe out residual messages that never got transmitted
    m_txQueue.clear();
    m_csmaCa->Cancel();
    m_txPkt = 0;
    
    // Bump the round counter
    m_currentRound++;
    
    // Transition to the LEACH_INIT to kick off the next round
    ChangeLeachState( LEACH_INIT );
    
    // Reset additional FSM variables
    m_bestCh.clear();
    m_prospectCh.lqi = 0;
    m_prospectCh.addr = Mac16Address ("00:00");
    m_prospectCh.chan = 0;
    m_prospectCh.sinr = 0.0;
    
    m_numChs = 0;
    m_myCHchan = 0;
    m_myChPos = -1;
    m_rcvdTdmaList = false;
    m_ClusterHeads.clear();
    m_MemberNodes.clear();
    m_rcvdMembers.clear();
    m_rcvdSequences.clear();
    m_sinkQueried = false;
    iAmClusterHead = false;
    m_amClusterMember = false;
    m_pauseCsma = false;
    m_waitForAck = false;
    m_joinMsgSent = false;
    m_chIsAlive = false;
    
    m_rxAdvSchFrames = 0;
    m_advPosOffset = 0;
    m_advFoundSpot = false;

    m_delayAdvSch = Seconds(0.0);
    
    m_numCsmacaRetry = 2;
    
    m_schdDoneTime = Seconds ( 0.0 );

    // Max number of rounds is the # node / # desired clusters
    // m_totalRounds = m_maxNumNodes / m_numClusters;
    
    if ( m_iAmSink ) {
      NS_LOG_DEBUG( "I am Sink starting " );
      
      m_epoch++;
      
      // Callback the start of the next round to the SensorHelper
      if ( !m_sensorNextRoundCallback.IsNull() )
        m_sensorNextRoundCallback( m_epoch );
      
      Ptr<Packet> p = CreatePacket( SUPER_FRM_ANNOUNCE, GetAddress(), Mac16Address ("ff:ff"), m_macDsn.GetValue() );
      
      // Trace the fact that we're queued up a ADV_CH frame
      m_macTxEnqueueTrace (p);
      
      // Set the next TX packet once the transciever is enabled
      m_txPkt = p;
      
      debugForceTx = sinkInitMessage;
      NS_LOG_DEBUG( "Sink transmitting SUPER FRAME " << m_txPkt );
      // Kick off the first round 1ms into the simulation
      Time holdTime = Seconds (0.001);
      NS_LOG_DEBUG( "\t Calling SENSOR_FORCE_TX 6 @ " << ( Now() + holdTime ) );
      m_forceTxEvent = Simulator::Schedule ( holdTime, &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX);
      
      // Transition to the ADV CH state - we'll auto transition to CH SCHED
      ChangeLeachState( LEACH_ADVERTISE );
      
      // Wait for at least 3x CHs to advertise before we announce the list or ask for more
      double msTillTx = ( 3.0 * m_numClusters * 0.007 );
      NS_LOG_UNCOND("\ttime till Adv: " << msTillTx << "ms" );
      NS_LOG_DEBUG( " SINK will check ADV Done @ " << Now() + Seconds( msTillTx ) );
      m_waitTimer.Cancel();
      m_waitTimer.SetFunction( &LeachMac::SinkAdvertiseDoneCheck, this );
      m_waitTimer.Schedule( Seconds( msTillTx ) );
    }
    else
    {
      m_macRxOnWhenIdle = true;
      
      SensorMac::SetPower( HIGH_POWER );
      SensorMac::SetChannel( SINK_CHANNEL );
      
      m_setMacState = Simulator::ScheduleNow (&LeachMac::SetSensorMacState, this, SENSOR_IDLE);
    }
  }
  else
    NS_LOG_DEBUG(" InitPhase Timeout on DEAD node ");
}
  
int64_t
LeachMac::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

void
LeachMac::ChangeLeachState(LeachMacState state)
{
  if ( m_systemEnabled )
  {
    NS_LOG_DEBUG( "LEACH FSM os " << leachString[m_leachMacState] << " ns " << leachString[state] << " at " << Now() );
    m_leachMacState = state;
  }
  else
    NS_LOG_DEBUG( "ChangeLEachState: I'm DEAD?" );
};
  
void
LeachMac::WaitTimerExpired( void )
{
  if ( m_systemEnabled )
  {
    NS_LOG_FUNCTION( this << m_dataQueue.size() );
    
    ChangeLeachState( LEACH_DATA_PHASE );
    
    m_txPkt = m_dataQueue.front()->txQPkt->Copy();
    m_dataQueue.pop_front();

    if ( m_txPkt == 0 )
      NS_FATAL_ERROR( " pkt pulled from queue is EMPTY!!! " << m_txPkt );
    
    // Replace the Dest Address with the CH Address
    LeachMacHeader tmpHdr;
    m_txPkt->RemoveHeader (tmpHdr);
    
    uint32_t seq = tmpHdr.GetSeqNum();
    NS_LOG_DEBUG ("\t transmit seq " << seq << " on chan " << m_prospectCh.chan << " pkt " << m_txPkt << " src " << tmpHdr.GetSrcAddr() );
    
    tmpHdr.SetDstAddr( m_prospectCh.addr );
    m_txPkt->AddHeader (tmpHdr);
    
    if ( !m_sensorDataTransmitCallback.IsNull() )
      m_sensorDataTransmitCallback( seq, m_Address );
    
    debugForceTx = nodeXdataMessage;
    NS_LOG_DEBUG( "CH-M transmitting data to CH in data phase " << m_txPkt );
    // Kick off the DATA transmission 10us
    Time holdTime = Seconds (0.0001);
    NS_LOG_DEBUG( "\t Calling SENSOR_FORCE_TX 7 @ " << ( Now() + holdTime ) );
    m_forceTxEvent = Simulator::Schedule ( holdTime, &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX);
  }
  else
    NS_LOG_DEBUG( " WaitTimer: TDMA TX when node is dead " );
}

void
LeachMac::JoinCheckTimerExpired( void )
{
  NS_LOG_DEBUG( "Join Check Timer Expired - enabled " << m_systemEnabled );
  
  if ( m_systemEnabled )
  {
    // If the node didn't receive a TDMA schedule with the local MAC Address 
    // in the list, then switch to a Direct Connect node.
    if ( !m_rcvdTdmaList )
    {
      NS_LOG_FUNCTION( this );
      ChangeLeachState( LEACH_DIRECT_CONN );
      DirectConnectTransmission();
    }
    else
      NS_LOG_DEBUG( "\trcvd TDMA list" );
  }
  else
    NS_LOG_DEBUG( "JoinCheckTimer: I'm DEAD?" );
}


void 
LeachMac::DirectConnectTransmission()
{
  if ( m_systemEnabled )
  {
    // Inform the SensorHelper that the node is officially a DC node
    if ( !m_sensorDirectCallback.IsNull() )
      m_sensorDirectCallback( m_Address );
    
    m_macRxOnWhenIdle = true;
    SensorMac::SetChannel( SINK_CHANNEL );
    SensorMac::SetPower( HIGH_POWER );
    
    NS_LOG_DEBUG("\t Back on chan 11 - and PHY is IDLE");
    NS_LOG_DEBUG("\t DIRECT CONNECT transmitting pkt " << m_txPkt );
    TxDirectConnect();
  }
  else
    NS_LOG_DEBUG ("DirConn: I'm DEAD?" );
}


Ptr<Packet>
LeachMac::CreatePacket(LeachMacFrameType type, Mac16Address src, Mac16Address dst, uint32_t seqNum )
{
  
  Ptr<Packet> p = new Packet();
  
  LeachMacHeader macHdr (LEACH_ORIGINAL, type, seqNum );
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
LeachMac::CreatePacketWithData(LeachMacFrameType type, Mac16Address src, Mac16Address dst, uint32_t seqNum, uint8_t const *data, int length )
{
  Ptr<Packet> p = new Packet( data, length );
  
  LeachMacHeader macHdr (LEACH_ORIGINAL, type, seqNum );
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
  
  //NS_LOG_DEBUG( "\t\tCreatePkt w/ Data " << p );
  return p;
}


bool
LeachMac::ExtractViableClusterHeads( Ptr<Packet> p, int size )
{
  bool bestFound = false;
  struct NodeList_s addresses;
  int len = p->CopyData( (uint8_t *)&addresses, size );
  
  if ( len != size )
    NS_FATAL_ERROR(" ***** Invalid data length for RX_FINAL_CH_LIST packet ***** ");
  else
  {
    NS_LOG_DEBUG( "\t  Extracted " << (len/2) << " addresses " );
    len /= 2;
    
    Mac16Address addr;
    std::list<BestCH_s>::iterator it = m_bestCh.begin();
    
    // First clean up our list of CHs, as we may have more than authorized
    while( it != m_bestCh.end() )
    {
      bool found = false;
      NS_LOG_DEBUG( "\t\tcheck if " << it->addr << " is in list" );
      
      for ( int i = 0; i < len; ++i )
      {
        addr.CopyFrom( (const uint8_t *) &(addresses.addresses[i]) );
        
        if ( addr == it->addr )
        {
          it->chan = SINK_CHANNEL + i + 1;
          found = true;
          NS_LOG_DEBUG( " \t\t  CH " << it->addr << " on chan " << it->chan );
          break;
        }
      }
      
      if ( found )
      {
        NS_LOG_DEBUG( "\t\t  found - keeping pwr " << it->pwr << " sinr " << it->sinr << " map sz " << m_bestCh.size() );
        ++it;
      }
      else
      {
        NS_LOG_DEBUG( "\t\t  not found - delete " );
        it = m_bestCh.erase( it );
        NS_LOG_DEBUG( "\t\t     map sz " << m_bestCh.size() );
      }
    }
    
    // Restore the pointer
    it = m_bestCh.begin();
    
    // Find the best CH in our list that is considered a CH by the SINK
    while ( it != m_bestCh.end() )
    {
      for ( int i = 0; i < len; ++i )
      {
        addr.CopyFrom( (const uint8_t *) &(addresses.addresses[i]) );
        
        if ( it->addr == addr )
        {
          bestFound = true;
          m_prospectCh = *it;
          break;
        }
      }
      
      if ( bestFound )
        break;
      else
        ++it;
    }
    
    // If we found the best CH, we need to attempt to join the CH's TDMA
    // list.  Otherwise, we need to revert to the DIRECT CONNNECT state
    if ( bestFound )
    {
      NS_LOG_DEBUG(" ***** RDY TO TX the JOIN REQ ***** ");
      
      NS_LOG_DEBUG( "Best CH " << m_prospectCh.addr << " chan " << m_prospectCh.chan << " pwr " << m_prospectCh.pwr );
      // Switch to the CH's channel and appropriate power level
      SensorMac::SetChannel( m_prospectCh.chan );
      SensorMac::SetPower( m_prospectCh.pwr );
      
      JoingRequest_s msg;
      msg.chAddress = m_prospectCh.addr;
      Ptr<Packet> p = CreatePacketWithData( JOIN_REQUEST, GetAddress(),
                                            m_prospectCh.addr, m_macDsn.GetValue(),
                                            (uint8_t const*)&msg, sizeof(msg) );
      m_macDsn++;
      m_waitForAck = true;
      m_joinMsgSent = false;
      
      // Trace the fact that we're queued up a JOIN_REQ frame
      m_macTxEnqueueTrace (p);
      m_txPkt = p;
      
      // Each node needs approximately 5ms to transmit, so we need to
      // use CSMA to avoid collisions... so try a random jitter
      double rndVal = m_uniformRandomVariable->GetValue(0.0, 1.0);
      Time waitTime = MilliSeconds ( rndVal * 1.2 * ( m_maxNumNodes - m_numClusters ) * 6.0 );
      
      m_maxJoinTime = MilliSeconds ( 1.2 * ( m_maxNumNodes - m_numClusters ) * 6.0 ) + Now();

      NS_LOG_DEBUG( "\t  kick off JOIN in " << waitTime << " pkt " << m_txPkt );
      NS_LOG_DEBUG( "\t *** CSMA @ " << waitTime );
      m_csmaEvent.Cancel();
      m_csmaEvent = Simulator::Schedule(waitTime, &LeachMac::SetSensorMacState, this, SENSOR_CSMA);
    }
  }
  
  return bestFound;
}

bool
LeachMac::GetNextViableClusterHead()
{
  bool found = false;
  Mac16Address addr;
  std::list<BestCH_s>::iterator it = m_bestCh.begin();
  
  // First clean up our list of CHs, as we may have more than authorized
  while( it != m_bestCh.end() )
  {
    if ( m_prospectCh.addr == it->addr )
    {
      NS_LOG_DEBUG( "\t\t\t removing prospect CH " << m_prospectCh.addr );
      found = true;
      m_bestCh.erase( it );
      break;
    }
    
    ++it;
  }
  
  // We redefine "FOUND" here to mean we found a new prospective CH to join
  it = m_bestCh.begin();
  if ( it != m_bestCh.end() )
  {
    found = true;
    m_prospectCh = *it;
    
    // Switch to the CH's channel and appropriate power level
    SensorMac::SetChannel( m_prospectCh.chan );
    SensorMac::SetPower( m_prospectCh.pwr );
    
    NS_LOG_DEBUG( "\t\t\t new prospect CH " << m_prospectCh.addr );
    
    JoingRequest_s msg;
    msg.chAddress = m_prospectCh.addr;
    Ptr<Packet> p = CreatePacketWithData( JOIN_REQUEST, GetAddress(),
                                         m_prospectCh.addr, m_macDsn.GetValue(),
                                         (uint8_t const*)&msg, sizeof(msg) );
    m_macDsn++;
    m_waitForAck = true;
    m_joinMsgSent = false;
    
    // Trace the fact that we're queued up a JOIN_REQ frame
    m_macTxEnqueueTrace (p);
    m_txPkt = p;
    
    // Each node needs approximately 5ms to transmit, so we need to
    // use CSMA to avoid collisions... so try a random jitter
    double rndVal = m_uniformRandomVariable->GetValue(0.0, 0.9);
    Time waitTime = ( m_maxJoinTime - Now() ) * rndVal;
    
    NS_LOG_DEBUG( "\t  JOIN new CH in " << waitTime << " pkt " << m_txPkt );
    NS_LOG_DEBUG( "\t *** CSMA @ " << waitTime );
    m_csmaEvent.Cancel();
    m_csmaEvent = Simulator::Schedule(waitTime, &LeachMac::SetSensorMacState, this, SENSOR_CSMA);
  }
  else
    found = false;
  
  return found;
}

void
LeachMac::LaunchDirectConnect( Time t )
{
  if ( m_systemEnabled )
  {
    NS_LOG_FUNCTION( " launch TxDirectConnect @ " << t );
    m_pauseCsma = false;
    m_macRxOnWhenIdle = true;

    // Return to SINK channel to wait for the TDMA PHASE
    SensorMac::SetChannel( SINK_CHANNEL );
    SensorMac::SetPower( HIGH_POWER );
    ChangeLeachState(LEACH_DIRECT_CONN);
    m_setMacState = Simulator::ScheduleNow (&LeachMac::SetSensorMacState, this, SENSOR_IDLE);
  }
  else
    NS_LOG_DEBUG("\t Can't launch DC... I'm dead" );
}
  
void
LeachMac::TxDirectConnect()
{
  if ( m_systemEnabled )
  {
    NS_LOG_FUNCTION( " start TxDirectConnect @ " << Now() );
    m_pauseCsma = false;

    // Use period for DATA PHASE to complete DC transmissions
    // 1 less period to ensure the last slot is successful
    //double msTillTx = ( (m_maxNumNodes + m_numClusters - 1.0) * 0.005 );
    double msTillTx = TDMA_TIMEOUT(-1);
    double rndVal = m_uniformRandomVariable->GetValue(0.0, 0.9);
    
    // Pop the data from the m_dataQueue if a packet is available
    if ( !m_dataQueue.empty() )
    {
      m_txPkt = m_dataQueue.front()->txQPkt->Copy();
      m_dataQueue.pop_front();
      
      // Replace the Dest Address with the CH Address
      LeachMacHeader tmpHdr;
      m_txPkt->RemoveHeader( tmpHdr );
      
      uint32_t seq = tmpHdr.GetSeqNum();
      NS_LOG_DEBUG("\t transmit seq " << seq );
      
      tmpHdr.SetDstAddr( "00:01" );
      m_txPkt->AddHeader (tmpHdr);
      
      if ( !m_sensorDataTransmitCallback.IsNull() )
        m_sensorDataTransmitCallback( seq, m_Address );
      
      NS_LOG_DEBUG( " Node " << m_Address << " sending data to SINK in " << Seconds( msTillTx * rndVal ) << " pkt " << m_txPkt );
      
      // Kick off a DC tranmission to the SINK
      Time waitTime = Seconds( msTillTx * rndVal );
      NS_LOG_DEBUG( "\t *** Kick off CSMA @ " << waitTime << " m_pkt " << m_txPkt << " and TxQ sz " << m_txQueue.size() );
      m_csmaEvent.Cancel();
      m_csmaEvent = Simulator::Schedule( waitTime, &LeachMac::SetSensorMacState, this, SENSOR_CSMA );
    }
    
    NS_LOG_DEBUG( "\treturn to InitPhaseTimeout in " << msTillTx << "s" );
    m_joinTimer.Cancel();
    m_joinTimer.SetFunction( &LeachMac::InitPhaseTimeout, this );
    m_joinTimer.Schedule( Seconds( msTillTx ) );
  }
  else
    NS_LOG_DEBUG( "TxDirectConnect: I'm DEAD?" );
}
  
void
LeachMac::ParseChAdvertisement( Ptr<Packet> p, uint8_t lqi, Mac16Address srcAddr )
{
  BestCH_s tempCh;
  
  // We need to use SINR (SNR without interference) to determine the best source
  // The higher the SINR, the closer the source
  LrWpanSinrTag stag( 0.0 );
  p->PeekPacketTag( stag );
  
  // Now process the message
  {
    AdvertiseCh_s ptr;
    p->CopyData( (uint8_t *) &ptr, sizeof(uint16_t));
    
    if( stag.Get() < 6.5 )
      tempCh.pwr = HIGH_POWER;
    else if ( stag.Get() < 55.5 )
      tempCh.pwr = MEDIUM_POWER;
    else
      tempCh.pwr = LOW_POWER;
    
    NS_LOG_DEBUG("  CH " << srcAddr << " req-pwr: " << (int)tempCh.pwr << "dBm  SNR " << stag.Get() );
    
    tempCh.lqi = lqi;
    tempCh.addr = srcAddr;
    tempCh.chan = ptr.spreadCode;
    tempCh.sinr = stag.Get();
  }
  
  std::list<BestCH_s>::iterator it = m_bestCh.begin();
  
  // We have to parse the list of CH's if there's already a node in the
  // list. We need to put this CH after the next best CH.
  if ( m_bestCh.size() > 0 )
  {
    while ( ( it != m_bestCh.end() ) &&
            ( tempCh.sinr < it->sinr ) )
    {
      NS_LOG_DEBUG( "\tchk addr " << it->addr << " sinr " << it->sinr );
      ++it;
    }
    
    if ( it == m_bestCh.end() )
    {
      m_bestCh.push_back( tempCh );
    }
    else
    {
      m_bestCh.insert( it, tempCh );
      ++it;
    }
  }
  else
  {
    // If the list is empty, push this CH into the list
    m_bestCh.push_front( tempCh );
    NS_LOG_DEBUG( "\tpshed " << tempCh.addr << " to frnt " );
  }

  if ( iAmClusterHead )
    NS_LOG_DEBUG( "\tcheck if need cancel #rx" << m_bestCh.size() << " IsRun(" << m_setMacState.IsRunning() << ") pkt " << m_txPkt );
  
  if ( iAmClusterHead && ( m_bestCh.size() > (unsigned)m_numClusters ) && m_setMacState.IsRunning() && m_txPkt )
  {
    NS_LOG_DEBUG( "\tCanceling the CSMA operation " );
    m_setMacState.Cancel();
    m_txPkt = 0;
  }
}

void
LeachMac::SinkAdvertiseDoneCheck()
{
  NS_LOG_DEBUG( "  *** SinkAdvertiseDoneCheck *** " << Now() );
  Ptr<Packet> pkt;
  if ( ( !m_sinkQueried ) && ( (int)m_ClusterHeads.size() < m_numClusters ) )
  {
    NS_LOG_DEBUG("SINK is sending the ADD_MORE_CHS pkt");
    pkt = CreatePacket( ADD_MORE_CHS, m_Address, Mac16Address("ff:ff"), m_macDsn.GetValue() );
    m_sinkQueried = true;
    
    // Wait for at least 3x CHs to advertise before we announce the list or ask for more
    double msTillTx = ( 3.0 * m_numClusters * 0.007 );
    NS_LOG_UNCOND("\ttime till Adv: " << msTillTx << "ms" );
    NS_LOG_DEBUG( " SINK will check ADV Done AGAIN @ " << Now() + Seconds( msTillTx ) );
    m_waitTimer.Cancel();
    m_waitTimer.SetFunction( &LeachMac::SinkAdvertiseDoneCheck, this );
    m_waitTimer.Schedule( Seconds( msTillTx ) );
  }
  else
  {
    NodeList_s data;
    int count = ( (int)m_ClusterHeads.size() < m_numClusters ) ? m_ClusterHeads.size() : m_numClusters;
    
    NS_LOG_DEBUG( " Sink attaching " << count << " CHs to the Final List message - #CHs " << m_numClusters );
    
    for ( int i = 0; i < count; ++i )
    {
      Mac16Address addr = m_ClusterHeads.front();

      addr.CopyTo( (uint8_t *) &(data.addresses[i]) );
      NS_LOG_DEBUG( "\t   adding address " << m_ClusterHeads.front() );
      m_ClusterHeads.pop_front();
    }
    
    NS_LOG_DEBUG("SINK is sending the RX_FINAL_CH_LIST pkt with " << count << " CHs " );
    pkt = CreatePacketWithData( RX_FINAL_CH_LIST, m_Address, Mac16Address("ff:ff"), m_macDsn.GetValue(), (uint8_t const *)&data, (count * 2) );
    
    // Setup a timer for SINK to wait on JOIN REQs to complete
    double msTillTx = ( 2.7 * (m_maxNumNodes - m_numClusters) * 0.007 );
    NS_LOG_UNCOND("\ttime till Join check: " << msTillTx << "ms" );
    NS_LOG_DEBUG( " SINK will wait for JOINs to complete @ " << Now() + Seconds( msTillTx ) );
    m_joinTimer.Cancel();
    m_joinTimer.SetFunction( &LeachMac::SinkJoinDoneCheck, this );
    m_joinTimer.Schedule( Seconds( msTillTx ) );
  }
  
  m_macTxEnqueueTrace (pkt);
  m_txPkt = pkt;
  
  debugForceTx = sinkAdvertiseMessage;
  // Kick off the first round 1ms into the simulation
  Time holdTime = Seconds (0.0001);
  NS_LOG_DEBUG( "\t Calling SENSOR_FORCE_TX 8 @ " << ( Now() + holdTime ) );
  m_forceTxEvent = Simulator::Schedule ( holdTime, &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX);
}
  
void
LeachMac::SinkJoinDoneCheck()
{
  NS_LOG_DEBUG( " *** SinkJoinDoneCheck *** " << Now() );
  
  ChangeLeachState( LEACH_SINK_DATA );
  
  // Setup a timer for SINK to wait for DATA PHASE / DC's to complete
  //double msTillTx = ( (m_maxNumNodes + m_numClusters + 1) * 0.005 );
  double msTillTx = TDMA_TIMEOUT(1);
  NS_LOG_UNCOND("\ttime till DataPhase check: " << msTillTx << "ms" );
  NS_LOG_UNCOND("\t\tCHs: " << m_numClusters << " TDMA slot " << TDMASLOT << " Max Frames " << MAX_CH_FRAMES);
  NS_LOG_DEBUG( " SINK will wait for DATA PHASE to complete @ " << Now() + Seconds( msTillTx ) );
  m_joinTimer.Cancel();
  m_joinTimer.SetFunction( &LeachMac::SinkDataPhaseDoneCheck, this );
  m_joinTimer.Schedule( Seconds( msTillTx ) );

  Ptr<Packet> pkt;
  pkt = CreatePacket( TDMA_KICK_OFF, m_Address, Mac16Address("ff:ff"), m_macDsn.GetValue() );
  m_macDsn++;
  m_macTxEnqueueTrace (pkt);
  m_txPkt = pkt;
  
  debugForceTx = sinkKickOffMessage;
  // Kick off the first round 1ms into the simulation
  Time holdTime = Seconds (0.001);
  NS_LOG_DEBUG( "\t Calling SENSOR_FORCE_TX 9 @ " << ( Now() + holdTime ) );
  m_forceTxEvent = Simulator::Schedule ( holdTime, &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX);
}
  
void
LeachMac::SinkDataPhaseDoneCheck()
{
  NS_LOG_DEBUG( " *** SinkDataPhaseDoneCheck *** " << Now() );
  
  /*
   * We need to allow for any CH to transmit at least 75% of the members'
   * payloads.  There is asmall probability that a CH has to transmit all,
   * but best guess would be that it's extremely low.
   *
   * T_slots = 0.0051             ( 5.1ms )
   * Bytes_tot = (# nodes * 6)
   * Frames_max = ceil( Bytes_tot / max_len )    where: max len is 116 bytes
   *
   * CH slots = ceil( Frames_max * 0.75 )
   * # slost = CH * CH_slots
   *
   * Each CH gets at least one slot, and they're consecutive
   */
  
//  double Bytes_tot = m_maxNumNodes * 6.0;
//  double frames = ceil( Bytes_tot / MAX_PAYLOAD_LENGTH );
//  double ch_slots = ceil( frames * 0.75 );
//  double tot_slots = m_numClusters * ch_slots;
//  double tot_slots = m_numClusters * frames;
  
  // Setup a timer for SINK to wait for SINK DATA PHASE to complete and return to INIT
//  double msTillTx = tot_slots * TDMASLOT; // * 1.5;    // ( 2 * m_numClusters * 0.0065 ) + 0.010;
  double msTillTx = SINK_DAT_TIMEOUT;
  NS_LOG_DEBUG( "\t SINK will return to InitPhaseTimeout @ " << Now() + Seconds( msTillTx ) );
  NS_LOG_UNCOND("\t\t max frames: " << MAX_CH_FRAMES );
  NS_LOG_UNCOND("\ttime till nxt round check: " << msTillTx << "ms" << " i.e. " << Seconds(msTillTx) );
  m_joinTimer.Cancel();
  m_joinTimer.SetFunction( &LeachMac::InitPhaseTimeout, this );
  m_joinTimer.Schedule( Seconds( msTillTx ) );
  
  Ptr<Packet> pkt = CreatePacket( SINK_DATA, m_Address, Mac16Address ("ff:ff"), m_macDsn.GetValue() );
  m_macDsn++;
  m_macTxEnqueueTrace (pkt);
  m_txPkt = pkt;
  
  debugForceTx = sinkDataMessage;
  NS_LOG_DEBUG( "\t Calling SENSOR_FORCE_TX 10 @ " << Now() );
  // Kick off the first round 1ms into the simulation
  //Time holdTime = Seconds (0.0001);
  m_forceTxEvent = Simulator::ScheduleNow ( &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX);
}
 
void
LeachMac::CHJoinDoneCheck()
{
  if ( m_systemEnabled )
  {
    NS_LOG_DEBUG( " *** CHJoinDoneCheck *** " << Now() );
    
    // Return to SINK channel to wait for the TDMA kickoff
    SensorMac::SetChannel( SINK_CHANNEL );
    m_macRxOnWhenIdle = true;
    m_setMacState = Simulator::ScheduleNow ( &LeachMac::SetSensorMacState, this, SENSOR_IDLE );
  }
  else
    NS_LOG_DEBUG( "CHJoinDoneCheck: I'm DEAD?" );
}

void
LeachMac::CHdataPhaseComplete()
{
  if ( m_systemEnabled )
  {
    NS_LOG_DEBUG( "\t CH Node " << m_Address << " hit the CHdataPhaseComplete function @ " << Now() );
    SensorMac::SetChannel( SINK_CHANNEL );
    ChangeLeachState( LEACH_SINK_DATA );
  }
  else
    NS_LOG_DEBUG( "CH-DataPhase: I'm DEAD?" );
}
  
void
LeachMac::CheckIfClusterMember()
{
  NS_LOG_DEBUG( "CheckIfClusterMember " << m_amClusterMember << " @ " << Now() << " cntr " << checkCounter << " sysEnabled " << m_systemEnabled << " m_txPkt " << m_txPkt );

  if ( m_systemEnabled )
  {
    if ( !m_amClusterMember )
    {
      m_txPkt = 0;
      m_txQueue.clear();
      m_csmaCa->Cancel();
      
      NS_LOG_DEBUG("   Not a CH-M ... m_txPkt " << m_txPkt << " TxQ sz " << m_txQueue.size() );
      // Return to SINK channel to wait for the TDMA PHASE
      SensorMac::SetChannel( SINK_CHANNEL );
      SensorMac::SetPower( HIGH_POWER );
      ChangeLeachState(LEACH_DIRECT_CONN);
      m_pauseCsma = false;
      m_macRxOnWhenIdle = true;
      m_setMacState.Cancel();
      m_setMacState = Simulator::ScheduleNow (&LeachMac::SetSensorMacState, this, SENSOR_IDLE);
    }
    else
    {
      // Now verify that the CH is still alive
      double msTillTx = 0.013 + ( 0.2 * (m_maxNumNodes - m_numClusters) * 0.05 );
      
      NS_LOG_DEBUG( "\tCheck is CH is alive @ " << (Now() + Seconds( msTillTx )) );
      m_chIsDeadTimer.Cancel();
      m_chIsDeadTimer.SetFunction( &LeachMac::CheckIfCHAlive, this );
      m_chIsDeadTimer.Schedule( Seconds( msTillTx ) );

    }
  }
  else
    NS_LOG_DEBUG( "I'm DEAD? \tnot a CH-M and the radio is dead" );
}
  
void
LeachMac::CheckIfCHAlive()
{
  if ( m_systemEnabled )
  {  
    NS_LOG_DEBUG( " *** checking CH is Alive(" << m_chIsAlive << ") for node " << m_Address << " @ " << Now() );
    
    if ( !m_chIsAlive )
    {
      //LaunchDirectConnect( Time( Seconds(0.0) ) );
      //TxDirectConnect();
      NodeWaitingForInit();
    }
  }
  else
    NS_LOG_DEBUG ( "CheckIfCHAlive - I'm DEAD" );
}

void
LeachMac::NodeWaitingForInit()
{
  NS_LOG_DEBUG( "\t Node " << m_Address << " hit the NodeWaitingForInit function @ " << Now() );
  
  NS_LOG_DEBUG( "\t\tNode needs to change channels - calling InitPhaseTimeout");
  SensorMac::SetChannel( SINK_CHANNEL );
  
  ChangeLeachState( LEACH_INIT );
  InitPhaseTimeout();
}
  
void
LeachMac::SinkDelivery(int offset)
{
  // # Members + CH * 4 bytes - 2B for addr, 4B for seq, all + 2 bytes for length
  int maxlength = MAX_PAYLOAD_LENGTH;
  int payloadSize = (6 * (m_rcvdMembers.size() + 1));
  int numFrames = ceil( (double)payloadSize / (double)maxlength );
  uint8_t msg[ maxlength ];
  
  NS_LOG_DEBUG( "\t\t CH required payload buffer " << payloadSize << " from " << m_Address << " maxlen " << maxlength );
  
  // Push the payload size after we populate - in case we're short
  int idx = 0;
  uint16_t addr;
  uint16_t seq;
  
  // Push my address and the sequence # first - if available
  m_Address.CopyTo( (uint8_t *) &addr );
  
  if ( !m_dataQueue.empty() )
  {
    LeachMacHeader tmpHdr;
    
    /*
     * Remove the header from the next packet in the data queue and get the
     * sequence number so we can insert it into the CH to SINK packet
     */
    
    m_txPkt = m_dataQueue.front()->txQPkt->Copy();
    m_dataQueue.pop_front();
    
    m_txPkt->RemoveHeader (tmpHdr);
    
    uint32_t seq = tmpHdr.GetSeqNum();
    NS_LOG_DEBUG("\t CH " << m_Address << " 16b: " << addr << " is transmiting seq " << seq << " pkt " << m_txPkt );
    
    if ( !m_sensorDataTransmitCallback.IsNull() )
      m_sensorDataTransmitCallback( seq, m_Address );
    
    msg[ idx++ ] = addr & 0x00FF;
    msg[ idx++ ] = ( (addr >> 8) & 0x00FF);
    msg[ idx++ ] = seq & 0x00FF;
    msg[ idx++ ] = ( (seq >> 8) & 0x00FF);
    msg[ idx++ ] = ( (seq >> 16) & 0x00FF);
    msg[ idx++ ] = ( (seq >> 24) & 0x00FF);
  }
  else
    NS_LOG_DEBUG("\t\t   don't copy CH - no pkt to send" );

  // Now create 1 or more CH Payload frames to send to the SINK
  for ( int j = 0; j < numFrames; ++j )
  {
    // Now copy the address and seq # from every received frame into my packet
    while ( m_rcvdMembers.size() > 0 )
    {
      seq = m_rcvdSequences.front();
      m_rcvdMembers.front().CopyTo( (uint8_t *) &addr );
      
      NS_LOG_DEBUG( "\t\t   front " << m_rcvdMembers.front() << " addr " << htons(addr) << " seq " << seq );
      m_rcvdMembers.pop_front();
      m_rcvdSequences.pop_front();
      
      msg[ idx++ ] = addr & 0x00FF;
      msg[ idx++ ] = ( (addr >> 8) & 0x00FF);
      msg[ idx++ ] = seq & 0x00FF;
      msg[ idx++ ] = ( (seq >> 8) & 0x00FF);
      msg[ idx++ ] = ( (seq >> 16) & 0x00FF);
      msg[ idx++ ] = ( (seq >> 24) & 0x00FF);
      
      if ( idx >= (maxlength - 5) )
        break;
    }

    NS_LOG_DEBUG( "\tfinal len of msg " << idx );
    Ptr<Packet> p = CreatePacketWithData( CH_PYLOD, m_Address,
                                          Mac16Address("00:01"), m_macDsn.GetValue(),
                                          (uint8_t const*)&msg, idx );
    m_macDsn++;
    m_macTxEnqueueTrace (p);
    
    TxQueueElement *txQElement = new TxQueueElement;
    txQElement->txQPkt = p;
    m_txQueue.push_back (txQElement);
    
    // reset the index pointerto the start of the msg buffer
    idx = 0;
  }
  
  // Now grab the first packet on the queue for transmission
  m_txPkt = m_txQueue.front()->txQPkt->Copy();
  m_txQueue.pop_front();
  
  // We need to send our data in a 5ms TDMA slot, but give it 10us offset from
  // the beginning of the slot to ensure clear to send
//  double msTillTx = ( m_myChPos * TDMASLOT ) + 0.001;
  double msTillTx = ( m_myChPos * SINK_DAT_CH_SLOT ) + 0.00001;
  
  debugForceTx = chDataSendMessage;
  // Kick off the CH Data Transmissions via CSMA check
  NS_LOG_DEBUG( "\tCH prep to send data m_txPkt = " << m_txPkt << " myPos " << m_myChPos );
  NS_LOG_DEBUG( "\t Calling SENSOR_FORCE_TX 11 in " << msTillTx << "ms\t now " << Now() << " TX @ " << (Seconds(msTillTx) + Now()) );
  m_forceTxEvent = Simulator::Schedule( Seconds(msTillTx), &LeachMac::SetSensorMacState, this, SENSOR_FORCE_TX );
  
  // Clear out the CH list of rcvd member addresses
  // non-intrussive for non-CH nodes
  m_rcvdMembers.clear();
  m_rcvdSequences.clear();
}
 
  
} // namespace ns3

