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
 * Author:
 *  kwong yin <kwong-sang.yin@boeing.com>
 *  Sascha Alexander Jopen <jopen@cs.uni-bonn.de>
 *
 * Modified for Sensor Networks:
 *  Nelson Powell <nhp8080@rit.edu>
 */

#include "sensor-csma.h"
#include <ns3/random-variable-stream.h>
#include <ns3/simulator.h>
#include <ns3/log.h>
#include <algorithm>

namespace ns3 {
  
NS_LOG_COMPONENT_DEFINE ("SensorCsmaCa");

NS_OBJECT_ENSURE_REGISTERED (SensorCsmaCa);

TypeId
SensorCsmaCa::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SensorCsmaCa")
  .SetParent<Object> ()
  .SetGroupName ("Sensor")
  .AddConstructor<SensorCsmaCa> ()
  ;
  return tid;
}

SensorCsmaCa::SensorCsmaCa ()
{
  // TODO-- make these into ns-3 attributes
  m_NB = 0;
  m_CW = 2;
  m_BLE = false;
  m_macMinBE = 3;
  m_macMaxBE = 5;
  m_macMaxCSMABackoffs = 4;
  m_aUnitBackoffPeriod = 20; //20 symbols
  m_random = CreateObject<UniformRandomVariable> ();
  m_BE = m_macMinBE;
  m_ccaRequestRunning = false;
}

SensorCsmaCa::~SensorCsmaCa ()
{
  m_mac = 0;
}

void
SensorCsmaCa::DoDispose ()
{
  m_sensorMacStateCallback = MakeNullCallback< void, SensorChnState> ();
  Cancel ();
  m_mac = 0;
}

void
SensorCsmaCa::SetMac (Ptr<SensorMac> mac)
{
  m_mac = mac;
}

Ptr<SensorMac>
SensorCsmaCa::GetMac (void) const
{
  return m_mac;
}

void
SensorCsmaCa::SetMacMinBE (uint8_t macMinBE)
{
  NS_LOG_FUNCTION (this << macMinBE);
  m_macMinBE = macMinBE;
}

uint8_t
SensorCsmaCa::GetMacMinBE (void) const
{
  NS_LOG_FUNCTION (this);
  return m_macMinBE;
}

void
SensorCsmaCa::SetMacMaxBE (uint8_t macMaxBE)
{
  NS_LOG_FUNCTION (this << macMaxBE);
  m_macMinBE = macMaxBE;
}

uint8_t
SensorCsmaCa::GetMacMaxBE (void) const
{
  NS_LOG_FUNCTION (this);
  return m_macMaxBE;
}

void
SensorCsmaCa::SetMacMaxCSMABackoffs (uint8_t macMaxCSMABackoffs)
{
  NS_LOG_FUNCTION (this << macMaxCSMABackoffs);
  m_macMaxCSMABackoffs = macMaxCSMABackoffs;
}

uint8_t
SensorCsmaCa::GetMacMaxCSMABackoffs (void) const
{
  NS_LOG_FUNCTION (this);
  return m_macMaxCSMABackoffs;
}

void
SensorCsmaCa::SetUnitBackoffPeriod (uint64_t unitBackoffPeriod)
{
  NS_LOG_FUNCTION (this << unitBackoffPeriod);
  m_aUnitBackoffPeriod = unitBackoffPeriod;
}

uint64_t
SensorCsmaCa::GetUnitBackoffPeriod (void) const
{
  NS_LOG_FUNCTION (this);
  return m_aUnitBackoffPeriod;
}

Time
SensorCsmaCa::GetTimeToNextSlot (void) const
{
  NS_LOG_FUNCTION (this);
  
  // TODO: Calculate the offset to the next slot.
  
  return Seconds (0);
  
}
void
SensorCsmaCa::Start ()

{
  NS_LOG_FUNCTION (this);
  m_NB = 0;
  m_BE = m_macMinBE;
  m_randomBackoffEvent = Simulator::ScheduleNow (&SensorCsmaCa::RandomBackoffDelay, this);
}

void
SensorCsmaCa::Cancel ()
{
  m_randomBackoffEvent.Cancel ();
  m_requestCcaEvent.Cancel ();
  m_canProceedEvent.Cancel ();
}

/*
 * Delay for backoff period in the range 0 to 2^BE -1 units
 * TODO: If using Backoff.cc (Backoff::GetBackoffTime) will need to be slightly modified
 */
void
SensorCsmaCa::RandomBackoffDelay ()
{
  NS_LOG_FUNCTION (this);
  
  uint64_t upperBound = (uint64_t) pow (2, m_BE) - 1;
  uint64_t backoffPeriod;
  Time randomBackoff;
  uint64_t symbolRate;
  bool isData = false;
  
  symbolRate = (uint64_t) m_mac->GetPhy ()->GetDataOrSymbolRate (isData); //symbols per second
  backoffPeriod = (uint64_t)m_random->GetValue (0, upperBound+1); // num backoff periods
  randomBackoff = MicroSeconds (backoffPeriod * GetUnitBackoffPeriod () * 1000 * 1000 / symbolRate);
  
  NS_LOG_DEBUG ("\t\t\t--- requesting CCA after backoff off " << randomBackoff.GetMicroSeconds () << " us");
  m_requestCcaEvent = Simulator::Schedule (randomBackoff, &SensorCsmaCa::RequestCCA, this);

}

void
SensorCsmaCa::CanProceed ()
{
  NS_LOG_FUNCTION (this);
  
  bool canProceed = true;
  
  if (canProceed)
  {
    Time backoffBoundary = GetTimeToNextSlot ();
    m_requestCcaEvent = Simulator::Schedule (backoffBoundary, &SensorCsmaCa::RequestCCA, this);
  }
  else
  {
    Time nextCap = Seconds (0);
    m_randomBackoffEvent = Simulator::Schedule (nextCap, &SensorCsmaCa::RandomBackoffDelay, this);
  }
}

void
SensorCsmaCa::RequestCCA ()
{
  NS_LOG_FUNCTION (this);
  m_ccaRequestRunning = true;
  m_mac->GetPhy ()->PlmeCcaRequest ();
}

/*
 * This function is called when the phy calls back after completing a PlmeCcaRequest
 */
void
SensorCsmaCa::PlmeCcaConfirm (LrWpanPhyEnumeration status)
{
  NS_LOG_FUNCTION (this << status);
  
  // Only react on this event, if we are actually waiting for a CCA.
  // If the CSMA algorithm was canceled, we could still receive this event from
  // the PHY. In this case we ignore the event.
  if (m_ccaRequestRunning)
  {
    m_ccaRequestRunning = false;
    if (status == IEEE_802_15_4_PHY_IDLE)
    {
      // inform MAC, channel is idle
      if (!m_sensorMacStateCallback.IsNull ())
      {
        NS_LOG_DEBUG ("Notifying MAC of idle channel");
        //m_sensorMacStateCallback (SENSOR_IDLE);
        m_sensorMacStateCallback (SENSOR_CHN_IDLE);
      }
    }
    else
    {
      m_BE = std::min (static_cast<uint16_t> (m_BE + 1), static_cast<uint16_t> (m_macMaxBE));
      m_NB++;
      if (m_NB > m_macMaxCSMABackoffs)
      {
        // no channel found so cannot send pkt
        NS_LOG_DEBUG ("Channel access failure");
        if (!m_sensorMacStateCallback.IsNull ())
        {
          NS_LOG_LOGIC ("Notifying MAC of Channel access failure");
          m_sensorMacStateCallback (SENSOR_CHN_ACS_FAIL);
        }
        return;
      }
      else
      {
        NS_LOG_DEBUG ("Perform another backoff; m_NB = " << static_cast<uint16_t> (m_NB));
        m_randomBackoffEvent = Simulator::ScheduleNow (&SensorCsmaCa::RandomBackoffDelay, this); //Perform another backoff (step 2)
      }
    }
  }
}

void
SensorCsmaCa::SetSensorMacStateCallback (SensorMacStateCallback c)
{
  NS_LOG_FUNCTION (this);
  m_sensorMacStateCallback = c;
}

int64_t
SensorCsmaCa::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this);
  m_random->SetStream (stream);
  return 1;
}

uint8_t
SensorCsmaCa::GetNB (void)
{
  return m_NB;
}
  
} //namespace ns3
