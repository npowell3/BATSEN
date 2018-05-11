/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Rochester Institute of Technology
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
 * Author: Nelson Powell <nhp8080@rit.edu>
 *
 * NOTE:
 *
 * Code based on the OLSR module and modified for BATMAND-0.3.2
 * implementation.  OLSR was the predecessor for BATMAN and has many
 * similar features.  Plus, modifying the OLSR module reduces the
 * effort required to write a module from scratch.
 *
 * The BATMAN module is based on the IETF draft found at
 * https://tools.ietf.org/html/draft-openmesh-b-a-t-m-a-n-00 and the
 * BATMAN-0.3.2 code base downloadable from
 * https://www.open-mesh.org/projects/open-mesh/wiki/Download
 *
 *
 */

#include <cmath>

#include "ns3/assert.h"
#include "ns3/log.h"

#include "batmand-header.h"

#define IPV4_ADDRESS_SIZE 4
#define OGM_PKT_HEADER_SIZE 	sizeof(struct bat_packet)

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("BatmanHeader");

namespace batmand {

const int local_win_size = TQ_LOCAL_WINDOW_SIZE;
const int global_win_size = TQ_GLOBAL_WINDOW_SIZE;

/// Scaling factor used in RFC 3626.
#define OLSR_C 0.0625

///
/// \brief Converts a decimal number of seconds to the mantissa/exponent format.
///
/// \param seconds decimal number of seconds we want to convert.
/// \return the number of seconds in mantissa/exponent format.
///
uint8_t
SecondsToEmf (double seconds)
{
  int a, b = 0;

  // find the largest integer 'b' such that: T/C >= 2^b
  for (b = 0; (seconds / OLSR_C) >= (1 << b); ++b)
    {
    }
  NS_ASSERT ((seconds / OLSR_C) < (1 << b));
  b--;
  NS_ASSERT ((seconds / OLSR_C) >= (1 << b));

  // compute the expression 16*(T/(C*(2^b))-1), which may not be a integer
  double tmp = 16 * (seconds / (OLSR_C * (1 << b)) - 1);

  // round it up.  This results in the value for 'a'
  a = (int) std::ceil (tmp - 0.5);

  // if 'a' is equal to 16: increment 'b' by one, and set 'a' to 0
  if (a == 16)
    {
      b += 1;
      a = 0;
    }

  // now, 'a' and 'b' should be integers between 0 and 15,
  NS_ASSERT (a >= 0 && a < 16);
  NS_ASSERT (b >= 0 && b < 16);

  // the field will be a byte holding the value a*16+b
  return (uint8_t)((a << 4) | b);
}

///
/// \brief Converts a number of seconds in the mantissa/exponent format to a decimal number.
///
/// \param olsr_format number of seconds in mantissa/exponent format.
/// \return the decimal number of seconds.
///
double
EmfToSeconds (uint8_t olsrFormat)
{
  int a = (olsrFormat >> 4);
  int b = (olsrFormat & 0xf);
  // value = C*(1+a/16)*2^b [in seconds]
  return OLSR_C * (1 + a / 16.0) * (1 << b);
}



// ---------------- OLSR Packet -------------------------------

NS_OBJECT_ENSURE_REGISTERED (OGMHeader);

OGMHeader::OGMHeader () :
    m_array( NULL )
{
}

OGMHeader::~OGMHeader () {
}

OGMHeader::OGMHeader(const OGMHeader& o)
{
  NS_LOG_DEBUG(" in OGMHeader copy-constructor ");

  this->m_version              = o.m_version;
  this->m_flags                = o.m_flags;
  this->m_ttl                  = o.m_ttl;
  this->m_gwflags              = o.m_gwflags;
  this->m_packetSequenceNumber = o.m_packetSequenceNumber;
  this->m_gatewayPortNumber    = o.m_gatewayPortNumber;
  this->m_originatorAddress    = o.m_originatorAddress;
  this->m_previousAddress      = o.m_previousAddress;
  this->m_tq                   = o.m_tq;  
  this->m_hnaLength            = o.m_hnaLength;
  this->m_array = NULL;   
  this->m_hnaList.clear();

  NS_LOG_DEBUG("  cpy cnstrctr hnaLen " << (int)this->m_hnaLength );

  for ( uint8_t i = 0; i < o.m_hnaList.size(); ++i ) {
    this->m_hnaList.push_back( o.m_hnaList[i] );
  }
}

TypeId OGMHeader::GetTypeId (void) {
  static TypeId tid = TypeId ("ns3::batmand::OGMHeader")
    .SetParent<Header> ()
    .SetGroupName ("Batmand")
    .AddConstructor<OGMHeader> ()
  ;
  return tid;
}

TypeId OGMHeader::GetInstanceTypeId (void) const {
  return GetTypeId ();
}

uint32_t OGMHeader::GetSerializedSize (void) const {
  // TODO: add in the dynamic length of HNAs appended
  return OGM_PKT_HEADER_SIZE + ( m_hnaList.size() * (IPV4_ADDRESS_SIZE + 1) );
}

void OGMHeader::Print (std::ostream &os) const {
  os << "\n  Version:   " << (int)m_version << "\n" ;
  os << "  Flags:     " << (int)m_flags << "\n" ;
  os << "  TTL:       " << (int)m_ttl << "\n" ;
  os << "  GW Flags:  " << (int)m_gwflags << "\n" ;
  os << "  Seq No:    " << (int)m_packetSequenceNumber << "\n" ;
  os << "  GW Port #: " << (int)m_gatewayPortNumber << "\n" ;
  os << "  Origin:    " << m_originatorAddress << "\n" ;
  os << "  Prev Addr: " << m_previousAddress << "\n" ;
  os << "  TQ Val:    " << (int)m_tq << "\n" ;
  os << "  HNA Len:   " << (int)m_hnaLength << "\n" ;
}

void OGMHeader::Serialize (Buffer::Iterator start) const {
  Buffer::Iterator i = start;
  i.WriteU8 (m_version);
  i.WriteU8 (m_flags);
  i.WriteU8 (m_ttl);
  i.WriteU8 (m_gwflags);
  i.WriteHtonU16 (m_packetSequenceNumber);
  i.WriteHtonU16 (m_gatewayPortNumber);
  i.WriteHtonU32 (m_originatorAddress.Get ());
  i.WriteHtonU32 (m_previousAddress.Get ());
  i.WriteU8 (m_tq);
//  i.WriteU8 (m_hnaLength);
  // Replaces the serialization of the m_hnaLength field
  i.WriteU8 ( (uint8_t) m_hnaList.size () );
  
  for (size_t n = 0; n < m_hnaList.size(); ++n)
  {
    i.WriteHtonU32( m_hnaList[n].addr );
    i.WriteU8 ( m_hnaList[n].netmask );
  }
}

uint32_t OGMHeader::Deserialize (Buffer::Iterator start) {
  Buffer::Iterator i = start;
  m_version 		         = i.ReadU8 ();
  m_flags 		           = i.ReadU8 ();
  m_ttl 		             = i.ReadU8 ();
  m_gwflags        		   = i.ReadU8 ();
  m_packetSequenceNumber = i.ReadNtohU16 ();
  m_gatewayPortNumber    = i.ReadNtohU16 ();
  m_originatorAddress    = Ipv4Address (i.ReadNtohU32 ());
  m_previousAddress 	   = Ipv4Address (i.ReadNtohU32 ());
  m_tq 			             = i.ReadU8 ();
  m_hnaLength 		       = i.ReadU8 ();

  NS_LOG_DEBUG("Deserialize hnaLen " << (int)m_hnaLength );
  m_hnaList.clear ();

  for (uint32_t n = 0; n < m_hnaLength; ++n)
  {
    struct hna_local_entry temp = { i.ReadNtohU32(), i.ReadU8() };
    m_hnaList.push_back( temp );
  }
  
  return GetSerializedSize();
}

uint32_t OGMHeader::Deserialize (unsigned char *start, uint32_t messageSize) {
  uint32_t bytesLeft = messageSize;
  struct bat_packet *ptr = (struct bat_packet *)start;

  m_version              = ptr->version;
  m_flags                = ptr->flags;
  m_ttl                  = ptr->ttl;
  m_gwflags              = ptr->gwflags;
  m_packetSequenceNumber = ntohs( ptr->seqno );
  m_gatewayPortNumber    = ntohs( ptr->gwport );
  m_originatorAddress    = Ipv4Address( ntohl( ptr->orig ) );
  m_previousAddress      = Ipv4Address( ntohl( ptr->prev_sender ) );
  m_tq                   = ptr->tq;
  m_hnaLength            = ptr->hna_len;

  uint8_t *bytep = (uint8_t *)ptr + OGM_PKT_HEADER_SIZE;
  bytesLeft -= OGM_PKT_HEADER_SIZE;

  NS_ASSERT (bytesLeft % (IPV4_ADDRESS_SIZE + 1) == 0);
  int numAddresses = bytesLeft / (IPV4_ADDRESS_SIZE + 1);
  
  NS_LOG_DEBUG(" deserialize byte array - numAddrs " << numAddresses << " bytes left " << 
               bytesLeft << " HNA Len Field " << (int)m_hnaLength );
  
  m_hnaList.clear();

  // This calculation should be the same
  NS_ASSERT( numAddresses == m_hnaLength );
  
  for (int n = 0; n < m_hnaLength; ++n)
  {
    struct hna_local_entry temp;
    temp.addr = ntohl( *( (uint32_t *) bytep ) );
    bytep += 4;

    temp.netmask = *bytep;
    ++bytep;

    m_hnaList.push_back( temp );
  }

  return messageSize;
}

unsigned char *OGMHeader::SerializeHnaList(int16_t &length) {
   
  NS_LOG_DEBUG(" hnaList sz " << m_hnaList.size() << " hnaLen " << (int)m_hnaLength );
    // This calculation should be the same
  NS_ASSERT( m_hnaList.size() == m_hnaLength );
  
  length = m_hnaLength * (IPV4_ADDRESS_SIZE + 1);
  unsigned char *data = new unsigned char[length];
  unsigned char *ptr = data;
  
  for( uint8_t n = 0; n < m_hnaList.size(); ++n ) {
    uint32_t *val = (uint32_t *)ptr;
    *val = ntohl( m_hnaList[n].addr );
    ptr += IPV4_ADDRESS_SIZE;
    
    
    *ptr = m_hnaList[n].netmask;
    ++ptr;
  }
  
  return data;
}

struct bat_packet *OGMHeader::GetBatmanPacket() {
  struct bat_packet *pkt = new struct bat_packet;

  pkt->version   = m_version;
  pkt->flags	   = m_flags;
  pkt->ttl	     = m_ttl;
  pkt->gwflags	 = m_gwflags;
  pkt->seqno 	   = htons( m_packetSequenceNumber );
  pkt->gwport 	 = htons( m_gatewayPortNumber );
  pkt->orig	     = htonl( m_originatorAddress.Get() );
  pkt->prev_sender  = htonl( m_previousAddress.Get() );
  pkt->tq	          = m_tq;
  pkt->hna_len	    = m_hnaLength;
  
  return pkt;
}

unsigned char *OGMHeader::Serialize () {
  if ( m_array != NULL ) {
    delete m_array;
    m_array = NULL;
  }

  m_array = new uint8_t[ GetPacketLength() ];
  struct bat_packet *pkt = (struct bat_packet *)m_array;
  pkt->version      = m_version;
  pkt->flags        = m_flags;
  pkt->ttl          = m_ttl;
  pkt->gwflags      = m_gwflags;
  pkt->seqno        = htons( m_packetSequenceNumber );
  pkt->gwport       = htons( m_gatewayPortNumber );
  pkt->orig         = htonl( m_originatorAddress.Get() );
  pkt->prev_sender  = htonl( m_previousAddress.Get() );
  pkt->tq           = m_tq;
  pkt->hna_len      = m_hnaLength;

  uint8_t *ptr = (uint8_t *)&(m_array[BATMAN_PACKET_SIZE]);
  
  for ( size_t n = 0; n < m_hnaList.size(); ++n ) {
    uint32_t addr = htonl( m_hnaList[n].addr );
    memcpy( ptr, (void *) &addr, 4 );
    ptr += 4;
    *ptr = m_hnaList[n].netmask;
    ++ptr;
  }

  return m_array;
}

/*
OGMHeader& OGMHeader::operator=(const OGMHeader& rhs)
{
  NS_LOG_DEBUG(" in OGMHeader operator= overloading ");

  if (this == &rhs)
    return *this;

  this->m_hnaList.clear();

  this->m_version              = rhs.m_version;
  this->m_flags                = rhs.m_flags;
  this->m_ttl                  = rhs.m_ttl;
  this->m_gwflags              = rhs.m_gwflags;
  this->m_packetSequenceNumber = rhs.m_packetSequenceNumber;
  this->m_gatewayPortNumber    = rhs.m_gatewayPortNumber;
  this->m_originatorAddress    = rhs.m_originatorAddress;
  this->m_previousAddress      = rhs.m_previousAddress;
  this->m_tq                   = rhs.m_tq;
  this->m_hnaLength            = rhs.m_hnaLength;
  this->m_array                = NULL;
  this->m_hnaList              = rhs.m_hnaList;
  
  return *this;
}
*/

OGMHeader& OGMHeader::operator=(OGMHeader const& rhs)
{
  //NS_LOG_DEBUG(" in OGMHeader operator= overloading ");

  if (this == &rhs)
    return *this;
  
  if ( this->m_array != NULL ) {
     delete this->m_array;
     this->m_array = NULL;
  }

  this->m_version              = rhs.m_version;
  this->m_flags                = rhs.m_flags;
  this->m_ttl                  = rhs.m_ttl;
  this->m_gwflags              = rhs.m_gwflags;
  this->m_packetSequenceNumber = rhs.m_packetSequenceNumber;
  this->m_gatewayPortNumber    = rhs.m_gatewayPortNumber;
  this->m_originatorAddress    = rhs.m_originatorAddress;
  this->m_previousAddress      = rhs.m_previousAddress;
  this->m_tq                   = rhs.m_tq;
  this->m_hnaLength            = rhs.m_hnaLength;
  this->m_array                = NULL;
  this->m_hnaList.clear();
  
  for ( uint8_t i = 0; i < rhs.m_hnaList.size(); ++i ) {
    this->m_hnaList.push_back( rhs.m_hnaList[i] );
  }
  
  return *this;
}

}
}  // namespace olsr, ns3

