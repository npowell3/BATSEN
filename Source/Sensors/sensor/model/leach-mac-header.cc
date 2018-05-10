/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
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

#include "leach-mac-header.h"
#include <ns3/log.h>
#include <ns3/address-utils.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LeachMacHdr");

NS_OBJECT_ENSURE_REGISTERED (LeachMacHeader);

LeachMacHeader::LeachMacHeader ()
{
  SetMacType(LEACH_ORIGINAL);   // Assume original LEACH
  SetFrameType(DATA_FRAME);     // Assume Data frame
}


LeachMacHeader::LeachMacHeader (enum LeachMacType leachMacType,
                                enum LeachMacFrameType leachMacFrmType,
                                uint32_t seqNum)
{
  SetMacType (leachMacType);
  SetFrameType (leachMacFrmType);
  SetSeqNum (seqNum);
  NS_LOG_DEBUG( "\t\t%%%% creating Leach Mac Header with seq " << m_seqNum << " %%%% " );
}


LeachMacHeader::~LeachMacHeader ()
{
}

enum LeachMacType
LeachMacHeader::GetMacType (void) const
{
  return (LeachMacType)m_macType;
}

enum LeachMacFrameType
LeachMacHeader::GetFrameType (void) const
{
  return (LeachMacFrameType)m_frmType;
}

Mac16Address
LeachMacHeader::GetDstAddr (void) const
{
  return(m_dstAddr);
}

Mac16Address
LeachMacHeader::GetSrcAddr (void) const
{
  return(m_srcAddr);
}
  
uint32_t
LeachMacHeader::GetSeqNum (void) const
{
  return(m_seqNum);
}

void
LeachMacHeader::SetMacType (enum LeachMacType macType)
{
  m_macType = macType;
}

void
LeachMacHeader::SetFrameType (enum LeachMacFrameType frameType)
{
  m_frmType = frameType;
}
  
void
LeachMacHeader::SetSrcAddr (Mac16Address addr)
{
  m_srcAddr = addr;
}

void
LeachMacHeader::SetDstAddr (Mac16Address addr)
{
  m_dstAddr = addr;
}

void
LeachMacHeader::SetSeqNum (uint32_t seqNum)
{
  m_seqNum = seqNum;
}

TypeId
LeachMacHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LeachMacHeader")
  .SetParent<Header> ()
  .SetGroupName ("Sensor")
  .AddConstructor<LeachMacHeader> ();
  return tid;
}

TypeId
LeachMacHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void
LeachMacHeader::Print (std::ostream &os) const
{
  os << "  Frame Type = " << (uint32_t) m_frmType << ", MAC Type = " << (uint32_t) m_macType
  << ", Seq Num = " << (uint32_t) m_seqNum << ", Data Len = " << (uint32_t) m_dataLen
  << ", Dst Addrs = " << m_dstAddr << ", Src Addr = " << m_srcAddr;
}

uint32_t
LeachMacHeader::GetSerializedSize (void) const
{
  /*
   * Each mac header will have 11 bytes of header
   * Dst Address        : 2 octet
   * Src Address        : 2 octet
   * Frame Type         : 1 octet
   * MAC Type           : 1 octet
   * Sequence Number    : 4 Octet
   * Data Length        : 1 Octet
   *
   * NOTE: Max packet length is 127 - 11 = 116
   */
  return ((uint32_t)SENSOR_HEADER_LENGTH);
}


void
LeachMacHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  WriteTo (i, m_dstAddr);
  WriteTo (i, m_srcAddr);
  
  i.WriteU8 ( m_macType );
  i.WriteU8 ( m_frmType );
  i.WriteU32 ( m_seqNum );
  i.WriteU8 ( m_dataLen );
}


uint32_t
LeachMacHeader::Deserialize (Buffer::Iterator start)
{
  
  Buffer::Iterator i = start;

  ReadFrom (i, m_dstAddr);
  ReadFrom (i, m_srcAddr);
  
  m_macType = i.ReadU8 ();
  m_frmType = i.ReadU8 ();
  m_seqNum  = i.ReadU32 ();
  m_dataLen = i.ReadU8 ();

  return GetSerializedSize();
}


}   // namespace