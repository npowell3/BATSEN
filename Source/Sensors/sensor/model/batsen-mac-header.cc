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

#include "batsen-mac-header.h"
#include <ns3/log.h>
#include <ns3/address-utils.h>
#include <ns3/mac16-address.h>

#include <arpa/inet.h>
#include <iostream>
#include <math.h>

#include <string>
#include <iomanip>
#include <sstream>
#include <stdint.h>

namespace ns3 {
  
NS_LOG_COMPONENT_DEFINE ("BatsenMacHdr");

NS_OBJECT_ENSURE_REGISTERED (BatsenMacHeader);
  
const Mac16Address SinkAddr("00:01");

std::string
BufferToString (uint8_t *buffer, uint32_t len)
{
  std::ostringstream oss;
  //
  // Tell the stream to make hex characters, zero-filled
  //
  oss.setf (std::ios::hex, std::ios::basefield);
  oss.fill ('0');
  
  //
  // Loop through the buffer, separating the two-digit-wide hex bytes
  // with a colon.
  //
  for (uint32_t i = 0; i < len; i++)
    oss << ":" << std::setw (2) << (uint16_t)buffer[i];
  
  return oss.str ();
}

bool PeerCompare( PeerNodeInfo *lhs, PeerNodeInfo *rhs )
{
  bool retval = false;
  
  //  printf("\tPEER COMPARE LHS score %f < RHS score %f\n", lhs->GetScore(), rhs->GetScore() );
  
  if ( lhs->GetScore() > rhs->GetScore() )
    retval = true;
  else if ( lhs->GetScore() == rhs->GetScore() )
  {
    if ( lhs->GetAddress() < rhs->GetAddress() )
      retval = true;
  }
  
  //  printf("\t\tLHS %s RHS\n", (retval ? "<" : ">") );
  return retval;
}

/*****************************************************************
 *  PeerNodeInfo helper class Methods
 *****************************************************************/
PeerNodeInfo::PeerNodeInfo() :
address( SinkAddr ),
m_power( HPWR ),
m_fwdPwr( INVLD ),
m_percent( 100.0 ),
m_score( 1.0 ),
needPeerNotice( true )
{
  ClearNodeList();
}

PeerNodeInfo::PeerNodeInfo( Mac16Address addr ) :
address( addr ),
m_power( HPWR ),
m_fwdPwr( INVLD ),
m_percent( 100.0 ),
m_score( 1.0 ),
needPeerNotice( true )
{
  ClearNodeList();
}

void
PeerNodeInfo::AddNodeList( PowerLvl pwrlvl, BNodeList maclist )
{
  BNodeList::iterator itr = maclist.begin();
  
  NS_LOG_DEBUG("\tPeerNodeInfo::AddNodeList for " << address );
  
  while ( itr != maclist.end() )
  {
    AddNode( pwrlvl, (*itr) );
    ++itr;
  }
}

PowerLvl
PeerNodeInfo::DoesNodeExist( Mac16Address mac )
{
  BNodeList::iterator litr = hiPwrList.begin();
  while ( litr != hiPwrList.end() )
  {
    if ( *litr == mac )
      return HPWR;

    ++litr;
  }
  
  litr = mdPwrList.begin();
  while ( litr != mdPwrList.end() )
  {
    if ( *litr == mac )
      return MPWR;

    ++litr;
  }
  
  litr = lwPwrList.begin();
  while ( litr != lwPwrList.end() )
  {
    if ( *litr == mac )
      return LPWR;

    ++litr;
  }
  
  return INVLD;
}
  
void
PeerNodeInfo::AddNode( PowerLvl pwrlvl, Mac16Address addr )
{
  //  NS_LOG_DEBUG("\t\t\tPeerNodeInfo::AddNode for " << address );
  
  PowerLvl existing = DoesNodeExist( addr );
  if ( existing != INVLD ) {
    /*
     * If the node is already in one of the lists, and the power
     * level of the most recent received frame is less than the
     * previously recorded power level, remove it from the current 
     * list and add it based on the new (lower) power level.
     */
    if ( pwrlvl < existing )
      RemoveNodeFromList( existing, addr );
    else
      return;
  }
  
  switch ( pwrlvl )
  {
    case HPWR:
    {
      //NS_LOG_DEBUG( "\t\t\t  self add node " << addr << " to HPWR list");
      hiPwrList.push_back( addr );
      break;
    }
      
    case MPWR:
    {
      //NS_LOG_DEBUG( "\t\t\t  self add node " << addr << " to MPWR list");
      mdPwrList.push_back( addr );
      break;
    }
      
    case LPWR:
    {
      //NS_LOG_DEBUG( "\t\t\t  self add node " << addr << " to LPWR list");
      lwPwrList.push_back( addr );
      break;
    }
      
    default:
      NS_LOG_DEBUG("Invalid power level list to add to");
      break;
  }
}

void
PeerNodeInfo::RemoveNode( Mac16Address addr )
{
  BNodeList::iterator litr = hiPwrList.begin();
  while ( litr != hiPwrList.end() )
  {
    if ( *litr == addr )
    {
      hiPwrList.erase(litr);
      return;
    }
    ++litr;
  }
  
  litr = mdPwrList.begin();
  while ( litr != mdPwrList.end() )
  {
    if ( *litr == addr )
    {
      mdPwrList.erase(litr);
      return;
    }
    ++litr;
  }
  
  litr = lwPwrList.begin();
  while ( litr != lwPwrList.end() )
  {
    if ( *litr == addr )
    {
      lwPwrList.erase(litr);
      return;
    }
    ++litr;
  }

  NS_LOG_DEBUG("\t\t\t\tRemove Node Failed - not in lists");
}
  
void
PeerNodeInfo::RemoveNodeFromList( PowerLvl pwrlvl, Mac16Address addr )
{
  BNodeList::iterator litr;
  
  switch( pwrlvl )
  {
    case HPWR: {
      litr = hiPwrList.begin();
      while ( litr != hiPwrList.end() )
      {
        if ( *litr == addr )
        {
          hiPwrList.erase(litr);
          return;
        }
        ++litr;
      }
      break;
    }
    case MPWR: {
      litr = mdPwrList.begin();
      while ( litr != mdPwrList.end() )
      {
        if ( *litr == addr )
        {
          mdPwrList.erase(litr);
          break;
        }
        ++litr;
      }
      break;
    }
    case LPWR: {
      litr = lwPwrList.begin();
      while ( litr != lwPwrList.end() )
      {
        if ( *litr == addr )
        {
          lwPwrList.erase(litr);
          break;
        }
        ++litr;
      }
      break;
    }
    case INVLD:
    default:
      NS_LOG_DEBUG("Invalid power level list to delete from");
      break;
  }
}

void
PeerNodeInfo::ClearNodeList()
{
  lwPwrList.clear();
  mdPwrList.clear();
  hiPwrList.clear();
}

void
PeerNodeInfo::CalculateScore(int numNodes)
{
  double totcnt = lwPwrList.size() + mdPwrList.size() + hiPwrList.size();
  double lowcnt = lwPwrList.size() * 100.0;
  double medcnt = mdPwrList.size() * 75.0;
  double hicnt  = hiPwrList.size() * 60.0;
  double cntagg = (totcnt > 0) ? (((lowcnt + medcnt + hicnt) / (100.0 * totcnt)) * (totcnt / numNodes)) : 0;
  
  double pwrtosink = 0.22;
  
  if ( m_power < m_fwdPwr )
    pwrtosink -= 0.11;
  
  if ( m_fwdPwr <= m_power )
    pwrtosink += 0.11;
  
  switch ( m_power )
  {
    case MPWR:
      pwrtosink *= 2.0;
      break;
      
    case LPWR:
      pwrtosink *= 3.0;
      break;
      
    case HPWR:
    default:
      break;
  }
  
  m_score = (cntagg + pwrtosink + m_percent) / 3.0;
  NS_LOG_DEBUG("low " << lowcnt << " med " << medcnt << " hi " << hicnt << " cntagg " << cntagg << " pwr " << pwrtosink );
}

void
PeerNodeInfo::HeardFrom()
{
  bitHistory <<= 1;
  bitHistory |= 0x01;   // Set bit 0 - present for current round
}

void
PeerNodeInfo::Missing()
{
  bitHistory <<= 1;
  bitHistory &= 0xFFFE; // Erase bit 0 - absent for current round
}

/*
 * This method is used for std::list sorting. The LHS node is checked
 * if it is LT the RHS node. If true, the std::list::sort will place
 * the LHS before the RHS in the list.
 *
 * The algorithm to detect "<" is as follows:
 *
 */
bool PeerNodeInfo::operator<(const PeerNodeInfo &rhs)
{
  bool retval = false;
  
  //  NS_LOG_DEBUG("\t\t\toperator<");
  //  NS_LOG_DEBUG("\t\t\tLHS score " << m_score << " < RHS score " << rhs.m_score );
  if ( m_score > rhs.m_score )
    retval = true;
  else if ( m_score == rhs.m_score )
  {
    if ( address < rhs.address )
      retval = true;
  }
  
  return retval;
}

bool PeerNodeInfo::operator>(const PeerNodeInfo &rhs)
{
  bool retval = false;
  
  //  NS_LOG_DEBUG("\tLHS score " << m_score << " > RHS score " << rhs.m_score );
  if ( m_score > rhs.m_score )
    retval = true;
  else if ( m_score == rhs.m_score )
  {
    if ( address < rhs.address )
      retval = true;
  }
  
  return retval;
}

BatsenMacHeader::BatsenMacHeader ()
{
  SetFrameType(DAT_FRAME);     // Assume Data frame
}

BatsenMacHeader::BatsenMacHeader ( enum BatsenMacFrameType macFrmType,
                                   uint32_t seqNum )
{
  SetFrameType (macFrmType);
  SetSeqNum (seqNum);
  //NS_LOG_DEBUG( "\t\t%%%% creating BATSEN Mac Header with seq " << m_seqNum << " %%%% " );
}

BatsenMacHeader::~BatsenMacHeader ()
{
}

enum BatsenMacFrameType
BatsenMacHeader::GetFrameType (void) const
{
  return (BatsenMacFrameType)m_frmType;
}

Mac16Address
BatsenMacHeader::GetDstAddr (void) const
{
  return(m_dstAddr);
}

Mac16Address
BatsenMacHeader::GetSrcAddr (void) const
{
  return(m_srcAddr);
}

uint32_t
BatsenMacHeader::GetSeqNum (void) const
{
  return(m_seqNum);
}

void
BatsenMacHeader::SetFrameType (enum BatsenMacFrameType frameType)
{
  m_frmType = frameType;
}

void
BatsenMacHeader::SetSrcAddr (Mac16Address addr)
{
  m_srcAddr = addr;
}

void
BatsenMacHeader::SetDstAddr (Mac16Address addr)
{
  m_dstAddr = addr;
}

void
BatsenMacHeader::SetSeqNum (uint32_t seqNum)
{
  m_seqNum = seqNum;
}

TypeId
BatsenMacHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::BatsenMacHeader")
  .SetParent<Header> ()
  .SetGroupName ("Sensor")
  .AddConstructor<BatsenMacHeader> ();
  return tid;
}

TypeId
BatsenMacHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void
BatsenMacHeader::Print (std::ostream &os) const
{
  os << "  Frame Type = " << (uint32_t) m_frmType
     << ", Seq Num = "    << (uint32_t) m_seqNum
     << ", Data Len = "   << (uint32_t) m_dataLen
     << ", Dst Addrs = "  << m_dstAddr
     << ", Src Addr = "   << m_srcAddr;
}

uint32_t
BatsenMacHeader::GetSerializedSize (void) const
{
  /*
   * Each mac header will have 11 bytes of header
   * Dst Address        : 2 octet
   * Src Address        : 2 octet
   * Frame Type         : 1 octet
   * Data Length        : 1 Octet
   * Sequence Number    : 4 Octet
   *
   * NOTE: Max packet length is 127 - 10 = 117
   */
  return ((uint32_t)BATSEN_HDR_LEN);
}


void
BatsenMacHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  
  WriteTo (i, m_dstAddr);
  WriteTo (i, m_srcAddr);
  
  i.WriteU32 ( m_seqNum );
  i.WriteU8 ( m_frmType );
  i.WriteU8 ( m_dataLen );
}


uint32_t
BatsenMacHeader::Deserialize (Buffer::Iterator start)
{
  
  Buffer::Iterator i = start;
  
  ReadFrom (i, m_dstAddr);
  ReadFrom (i, m_srcAddr);
  
  m_seqNum  = i.ReadU32 ();
  m_frmType = i.ReadU8 ();
  m_dataLen = i.ReadU8 ();
  
  return GetSerializedSize();
}

/*
 *
 *  OGM Builder Class implementation
 *
 */
bool
OgmBuilder::AddAddressTlv( uint8_t type )
{
  if (( m_len + 2 ) > MAX_PAYLD_LEN )
    return false;
  
  data[m_len++] = type;
  
  // save the TLV Length field offset
  m_tlvLen = m_len;
  data[m_len++] = 0;
  
  // No addresses added yet
  m_addrCnt = 0;
  
  return true;
}
  
bool
OgmBuilder::AddAddress( Mac16Address addr )
{
  if ( ( m_len + 2 ) > MAX_PAYLD_LEN )
    return false;
  
  addr.CopyTo( &(data[m_len]) );
  m_addrCnt++;
  m_len += 2;

  data[m_tlvLen] = m_addrCnt;
  
  return true;
}
  
bool
OgmBuilder::AddData( uint8_t type, uint8_t *input, uint8_t len )
{
  if ( ( m_len + len + 2 ) > MAX_PAYLD_LEN )
    return false;
  
  data[m_len++] = type;
  data[m_len++] = len;
  memcpy( &(data[m_len]), input, len );
  m_len += len;
  
  return true;
}

bool
OgmBuilder::CreateOgm(uint8_t *buffer, uint8_t length)
{
  memcpy( data, buffer, length );
  return true;
}
  
bool
OgmBuilder::GetNextTlv(uint8_t &type, uint8_t &length)
{
  if (( m_len + 2 ) > MAX_PAYLD_LEN )
    return false;
  
  type = data[m_len++];
  length = data[m_len++];
  m_addrCnt = length;
  
  // Make sure the next TLV is valid type
  switch ( type )
  {
    case RX_PWR_LIST:
    case LPWR_LIST:
    case MPWR_LIST:
    case HPWR_LIST:
    case RTR_SELCT:
    case NOT_FOUND:
    case DATA_WORD:
      break;
      
    default:
    {
      type = NULL_TLV;
      length = 0;
      m_addrCnt = 0;
      return false;
    }
  }
  
  return true;
}
  
bool
OgmBuilder::GetAddressList( BNodeList &alist )
{
  uint8_t *ptr = &data[m_len];
  
  for ( uint8_t i = 0; i < m_addrCnt; i++ )
  {
    Mac16Address taddr;
    
    taddr.CopyFrom( &(ptr[i]) );
    alist.push_back( taddr );
  }
  
  // Move the m_len forward for each address in the list
  m_len += (m_addrCnt * 2);
  
  return true;
}
  
  
SortedList::SortedList() :
  listHead( NULL ),
  listTail( NULL )
{
  
}

SortedList::~SortedList()
{
  Clear();
  listHead = NULL;
  listTail = NULL;
}

void
SortedList::Clear()
{
  // If there's anything to delete, go ahead and wipe it out
  NS_LOG_DEBUG( "\t\t######### Clear Sorted List #########" );
  if ( listHead ) {
    DataStruct *ptr = listHead;
    DataStruct *tmp = NULL;
    
    while ( ptr != NULL ) {
      NS_LOG_DEBUG( "\t\t\tDelete ptr for " << ptr->addr );
      tmp = ptr->next;
      ptr->next = NULL;
      ptr->prev = NULL;
      
      if ( listTail == ptr )
        listTail = NULL;
      
      delete ptr;
      
      ptr = tmp;
    }
    
    if ( listTail )
      delete listTail;
    
    listTail = NULL;
    listHead = NULL;
  }
  else if ( listTail )
  {
    NS_LOG_DEBUG( "\t\t\tTAIL NOT NULL even though Head is" );
  }
  else
    NS_LOG_DEBUG( "\t\t\tNOTHING TO CLEAR" );
}

void
SortedList::DumpList()
{
  // If there's anything to dump, go ahead and dump it
  NS_LOG_DEBUG( "\t\t/\\/\\/\\/\\/\\ DUMP Sorted List /\\/\\/\\/\\/\\" );
  if ( listHead ) {
    DataStruct *ptr = listHead;
    
    while ( ptr != NULL ) {
      NS_LOG_DEBUG( "\t\t\tPtr for " << ptr->addr << " cnt " << ptr->count );
      ptr = ptr->next;
    }
    
    /*
    if ( listHead )
      NS_LOG_DEBUG("\t\t\t\t head " << listHead->addr );
    
    if ( listTail )
      NS_LOG_DEBUG("\t\t\t\t tail " << listTail->addr );
     */
  }
}

void
SortedList::AddNode( Mac16Address mac )
{
  DataStruct *ptr = new DataStruct;
  
  NS_LOG_DEBUG("\t\t\t\tAdding node to sorted list " << mac << " hd " << listHead << " tail " << listTail );
  ptr->addr = mac;
  ptr->count = 1;
  ptr->next = NULL;
  ptr->prev = NULL;
  
  // Insert from the tail since it is the lowest count possible
  InsertTail( ptr );
}

void
SortedList::Increment( Mac16Address mac )
{
  DataStruct *ptr = FindAddr( mac );
  
  if ( ptr ) {
    DataStruct *srch = ptr->prev;

    // Bump the counter
    ptr->count++;
        
    if ( ptr != listHead ) {
      
      if ( ( listTail == ptr ) && ( listTail->prev ) ) {
        listTail = listTail->prev;
        NS_LOG_DEBUG( "\t\t\t\t  tail shifted to " << listTail->addr );
      }
      
      // Point both previous and next pointers at each other to 
      // effectively remove this node from the list
      if ( ptr->prev )
        ptr->prev->next = ptr->next;
      if ( ptr->next )
        ptr->next->prev = ptr->prev;
      
      NS_LOG_DEBUG("\t\t\t\trelocate node " << mac << " count bumped " << ptr->count );
      InsertFrom( ptr, srch );
    }
    else {
      // Already the head node, so just update the counter
      //      NS_LOG_DEBUG("\t\t\t\thead node " << mac << " count bumped to " << ptr->count );
    }
  }
  else {
    NS_LOG_DEBUG("\t\t\t\t add new node for " << mac);
    AddNode( mac );
  }
}

BNodeList
SortedList::GetTopNodes( unsigned count )
{
  BNodeList tmp;
  DataStruct *ptr = listHead;
  
  if ( listHead ) {
    unsigned i = 0;
    
    for ( ; i < count; i++ ) {
      tmp.push_back( ptr->addr );
      
      if ( ptr->next )
        ptr = ptr->next;
      else
        break;
    }
    
    NS_LOG_DEBUG("\t\t\t\treturning " << i << " router list");
  }
  else
    NS_FATAL_ERROR("No nodes in list");
  
  return tmp;
}

  
SortedList::DataStruct *
SortedList::FindAddr( Mac16Address mac )
{
  DataStruct *ptr = listHead;
  
  if ( listHead ) {
    while ( ptr ) {
      
      if ( mac == ptr->addr )
        break;
      
      ptr = ptr->next;
    }
  }
  
  return ptr;
}

void
SortedList::InsertTail( DataStruct *node )
{
  if ( listTail == NULL ) { 
    if ( listHead == NULL ) {
      listHead = node;
      node->next = NULL;
      node->prev = NULL;
      listTail = node;
    }
    else {
      // Check if the head is lesser of a node by virtue of address
      InsertFrom( node, listHead );
    }
  }
  else {
    InsertFrom( node, listTail );
  }
}

void
SortedList::InsertFrom( DataStruct *node, DataStruct *search )
{
  bool inserted = false;
  DataStruct *tmp = search;
  
  NS_LOG_DEBUG("\t\t\t\t  srch " << search->addr << " " << search->count );

  // Now search all predicessors for the next highest counter with 
  while ( !inserted ) {
    
    /*
    NS_LOG_DEBUG("\t\t\t\t\ts-cnt " << search->count << " n-cnt " << node->count );
    NS_LOG_DEBUG("\t\t\t\t\ts-add " << search->addr << " n-add " << node->addr );
    
    if ( listHead )
      NS_LOG_DEBUG("\t\t\t\t\thead-add " << listHead->addr << " head-cnt " << listHead->count );
    
    if ( listTail )
      NS_LOG_DEBUG("\t\t\t\t\ttail-add " << listTail->addr << " tail-cnt " << listTail->count );
    */
    
    // Check if the current position has a lesser count
    // or equal count, yet lesser MAC
    if ( ( search->count < node->count ) ||
         ( ( search->addr > node->addr ) && ( search->count == node->count ) ) )
    {
      if ( search == listHead ) {
        //        NS_LOG_DEBUG("\t\t\t\ts is hd: emplace " << node->addr << " at head" );
        node->next = listHead;
        listHead->prev = node;
        listHead = node;
        break;
      }
      else if ( search->prev == NULL ) {
        NS_FATAL_ERROR("search isn't head, but prev == NULL");
      }
      else {
        tmp = search;
        search = search->prev;
      }
    }
    else if ( ( search->count > node->count ) ||
         ( ( search->addr < node->addr ) && ( search->count == node->count ) ) ) {
      // We should stop searching here all together as the Search element 
      // is superior or is NULL
      
      // First check failed and we go back to where we were
      if ( tmp == search ) {
        if ( search->next ) {
          search->next->prev = node;
          node->next = search->next;
        }
        
        // Replace the Tail if needed
        if ( search == listTail )
          listTail = node;
        
        search->next = node;
        node->prev = search;
        break;
      }
      else {
        // Search and Tmp are different, they should point to each other
        NS_ASSERT ( search->next == tmp );
        
        // Unless of course Tmp is the head and Search is NULL
        if ( search == NULL ) {
          NS_ASSERT ( tmp == listHead );
          
          node->next = listHead;
          listHead->prev = node;
          listHead = node;
        }
        else {
          tmp->prev = node;
          node->next = tmp;
          
          search->next = node;
          node->prev = search;
        }
        
        break;
      }
    }
    else {
      NS_FATAL_ERROR( " Unknown movement ? " );
    }
  }
}


  
}   // namespace


