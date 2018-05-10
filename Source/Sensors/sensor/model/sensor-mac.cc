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

#include "sensor-mac.h"
#include "sensor-csma.h"
#include "leach-mac-header.h"
#include "batsen-mac-header.h"
#include "mac-trailer.h"
#include <ns3/simulator.h>
#include <ns3/log.h>
#include <ns3/uinteger.h>
#include <ns3/node.h>
#include <ns3/packet.h>
#include <ns3/random-variable-stream.h>
#include <ns3/double.h>

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT     std::clog << "[address " << m_Address << "] ";

namespace ns3 {
  
NS_LOG_COMPONENT_DEFINE ("SensorMac");
  
NS_OBJECT_ENSURE_REGISTERED (SensorMac);


const uint32_t SensorMac::aMinMPDUOverhead = 9; // Table 85

TypeId
SensorMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SensorMac")
  .SetParent<Object> ()
  .SetGroupName ("Sensor")
  .AddConstructor<SensorMac> ()
  .AddAttribute ("SystemPower", "Starting system power for the Sensor Node (in Joules).",
                 DoubleValue ( 100.0 ),
                 MakeDoubleAccessor (&SensorMac::m_totalSystemPower),
                 MakeDoubleChecker <double>())
  ;
  return tid;
}

SensorMac::SensorMac() :
  m_macRxOnWhenIdle( true ),
  m_iAmSink( false ),
  m_csmaOk( false ),
  m_txPkt( 0 ),
  m_Address( Mac16Address ("00:00") ),
  m_retransmission( 0 ),
  m_numCsmacaRetry( 0 ),
  m_mWpwr( 0.01 ),
  m_previousState( IEEE_802_15_4_PHY_RX_ON ),
  m_timePrevState( Seconds( 0.0 ) ),
  m_systemEnabled( true )
{
  m_iAmSink = false;
  m_csmaOk = false;
  m_retransmission = 0;
  m_numCsmacaRetry = 0;
  m_txPkt = 0;
  m_macRxOnWhenIdle = true;
  
  Ptr<UniformRandomVariable> uniformVar = CreateObject<UniformRandomVariable> ();
  uniformVar->SetAttribute ("Min", DoubleValue (0.0));
  uniformVar->SetAttribute ("Max", DoubleValue (255.0));
  m_Address = Mac16Address ("00:00");
  
  m_previousState = IEEE_802_15_4_PHY_RX_ON;
  m_timePrevState = Seconds( 0.0 );
  m_systemEnabled = true;
  
  m_random = CreateObject<UniformRandomVariable> ();
}

SensorMac::~SensorMac ()
{
}

void
SensorMac::SetAddress (Mac16Address address)
{
  m_Address = address;
}

Mac16Address
SensorMac::GetAddress () const
{
  return m_Address;
}

void
SensorMac::ChangeMacState (SensorChnState newState)
{
  NS_LOG_LOGIC ("\t  change chn access state from "
                << g_chan_state[m_sensorMacState] << " to "
                << g_chan_state[newState] );
  m_macStateLogger (m_sensorMacState, newState);
  m_sensorMacState = newState;
}
  
void
SensorMac::SetCsmaCa (Ptr<SensorCsmaCa> csmaCa)
{
  m_csmaCa = csmaCa;
}

void
SensorMac::SetPhy (Ptr<LrWpanPhy> phy)
{
  m_phy = phy;
}

Ptr<LrWpanPhy>
SensorMac::GetPhy (void)
{
  return m_phy;
}
  
void
SensorMac::SetSensorDataIndicationCallback (SensorDataIndicationCallback c)
{
  m_sensorDataIndicationCallback = c;
}

void
SensorMac::SetSensorDataConfirmCallback (SensorDataConfirmCallback c)
{
  m_sensorDataConfirmCallback = c;
}

void
SensorMac::SetSinkStatus( bool amSink )
{
  m_iAmSink = amSink;
}
  
void
SensorMac::DoInitialize ()
{
  m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
  Object::DoInitialize ();
}
  
void
SensorMac::DoDispose ()
{
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
  
  for (uint32_t i = 0; i < m_dataQueue.size (); i++)
  {
    m_dataQueue[i]->txQPkt = 0;
    delete m_dataQueue[i];
  }
  m_dataQueue.clear ();
  
  m_phy = 0;
  m_sensorDataIndicationCallback = MakeNullCallback< void, SensorDataIndicationParams, Ptr<Packet> > ();
  m_sensorDataConfirmCallback = MakeNullCallback< void, SensorDataConfirmParams > ();
  m_sensorSinkDataRcvdCallback = MakeNullCallback< void, std::list<uint16_t>, std::list<uint32_t>  > ();
  m_sensorCHDataRcvdCallback = MakeNullCallback< void, Mac16Address, uint32_t > ();
  m_sensorForwaderCallback = MakeNullCallback< void, Mac16Address > ();
  m_sensorDataTransmitCallback = MakeNullCallback< void, uint32_t, Mac16Address > ();
  
  Object::DoDispose ();
}

int64_t
SensorMac::AssignStreams (int64_t stream)
{
  m_random->SetStream (stream);
  return 1;
}
  
void
SensorMac::SetChannel( int chan )
{
  m_PhyParams.phyCurrentChannel = chan;
  //  NS_LOG_DEBUG( "SetChannel ( phy " << m_phy << " )" << "( Addr " << m_Address << " )  to channel " << chan );
  m_phy->PlmeSetAttributeRequest( phyCurrentChannel, &m_PhyParams );
}

void
SensorMac::SetPower( int pwr )
{
  NS_LOG_FUNCTION( this << m_Address << pwr);
  m_PhyParams.phyTransmitPower = pwr;
  
  m_mWpwr = pow(10.0, (m_PhyParams.phyTransmitPower - 30) / 10) / 1000.0;
  //NS_LOG_DEBUG( " POWER: " << m_mWpwr << "mW == " << m_PhyParams.phyTransmitPower << "dBm" );
  m_phy->PlmeSetAttributeRequest( phyTransmitPower, &m_PhyParams );
}
  

  
} // namespace ns3