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

#ifndef BATMAND_HEADER_H
#define BATMAND_HEADER_H

#include <stdint.h>
#include <vector>
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"

#include <arpa/inet.h>

namespace ns3 {
namespace batmand {

double EmfToSeconds (uint8_t emf);
uint8_t SecondsToEmf (double seconds);

#define COMPAT_VERSION 5

#define PORT 4305
#define GW_PORT 4306

/* Flags Fields */
#define UNIDIRECTIONAL  0x80
#define DIRECTLINK      0x40

#define ADDR_STR_LEN 16
#define TQ_MAX_VALUE 255

#define TTL 50                /* Time To Live of broadcast messages */
#define PURGE_TIMEOUT 200000u  /* purge originators after time in ms if no valid packet comes in -> TODO: check influence on TQ_LOCAL_WINDOW_SIZE */
#define TQ_LOCAL_WINDOW_SIZE 64     /* sliding packet range of received originator messages in squence numbers (should be a multiple of our word size) */
#define TQ_GLOBAL_WINDOW_SIZE 10
#define TQ_LOCAL_BIDRECT_SEND_MINIMUM 1
#define TQ_LOCAL_BIDRECT_RECV_MINIMUM 1
#define TQ_TOTAL_BIDRECT_LIMIT 	      1

#define MAX_AGGREGATION_MS 	100
#define MAX_AGGREGATION_NS    100000
#define MAX_AGGREGATION_BYTES	512

extern const int local_win_size;
extern const int global_win_size;

/*
 *  NOTE: the bat_packet is moved from teh batmand-structures.h file
 * as it is needed for deserialization of unsigned char arrays to the
 * OGMHeader structure.  Eventually, the header files should be 
 * consolidated rather than reflect the old headers from the Linux variant.
 */

/// \ingroup batmand
/// Batman Protocol format (Packed byte array)
struct bat_packet
{
   uint8_t  version;  /* batman version field */
   uint8_t  flags;    /* 0x80: UNIDIRECTIONAL link, 0x40: DIRECTLINK flag, ... */
   uint8_t  ttl;
   uint8_t  gwflags;  /* flags related to gateway functions: gateway class */
   uint16_t seqno;
   uint16_t gwport;
   uint32_t orig;
   uint32_t prev_sender;
   uint8_t tq;
   uint8_t hna_len;
} __attribute__((packed));

#define BATMAN_PACKET_SIZE  (sizeof(struct bat_packet))

struct hna_local_entry
{
  uint32_t addr;
  uint8_t netmask;
};


typedef std::vector<hna_local_entry>  HnaLocalList;    //!< Vector of net devices supporting BATMAN protocol.

/**
 * \ingroup batmand
 *
 * The basic layout of any packet in BATMAND is as follows (omitting IP and
 * UDP headers):
  \verbatim
                 Originator Message (OGM) format.

    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |    Version    |U|D|           |      TTL      |    GWFlags    |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |        Sequence Number        |             GW Port           |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                      Originator Address                       |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                   Previous Sender Address                     |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |       TQ      |   HNA Length  |                               :
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+                               :
   :                                                               :
            (etc.)
  
            HNA (Host Network Association) Data Fields
 
    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                         Network Address                       |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |  Mask (CIDR)  |                                               :
   +-+-+-+-+-+-+-+-+                                               :
   :                                                               :
  \endverbatim
 *
 *
 * This is the fundamental BATMAND protocol packet format.
 */
class OGMHeader : public Header
{
public:
  OGMHeader ();
  virtual ~OGMHeader ();
  OGMHeader(const OGMHeader& o);

  //OGMHeader& operator=(const OGMHeader& o);

  OGMHeader& operator=(OGMHeader const& rhs);
  
  /**
   * Get the packet total length.
   * \return The packet length.
   */
  uint16_t GetPacketLength () const
  {
    return (BATMAN_PACKET_SIZE + m_hnaLength);
  }

  /**
   * Set the OGM packet version.
   * \param version The BATMAN protocol version employed.
   */
  void SetVersion (uint8_t version)
  {
    m_version = version;
  }

  /**
   * Get the OGM packet version.
   * \return The BATMAN protocol version employed.
   */
  uint8_t GetVersion () const
  {
    return m_version;
  }

  /**
   * Set the packet flags as a uint8_t (byte) field.
   * \param flags The packet flags as a byte.
   */
  void SetFlags (uint8_t flags)
  {
    m_flags = flags;
  }

  /**
   * Get the packet flags as a uint8_t (byte) field.
   * \return The packet flags as a byte.
   */
  uint8_t GetFlags () const
  {
    return m_flags;
  }

  /**
   * Set the DIRECT LINK flag bit in the flags field.
   * \param bool  Value of the DIRECT LINK bit.  TRUE by default.
   */
  void SetDirectLinkFlag ( bool val=true )
  {
    if ( val )
      m_flags |= DIRECTLINK;
    else
      m_flags &= ~DIRECTLINK;
  }

  /**
   * Check the packet flags for the DIRECT LINK bit being set.
   * \return TRUE if the direct link flag is asserted.
   */
  bool HasDirectLinkFlag () const
  {
    return m_flags & DIRECTLINK;
  }

  /**
   * Set the UNIDIRECTIONAL LINK flag bit in the flags field.
   * \param bool  Value of the UNIDIRECTIONAL LINK bit.  TRUE by default.
   */
  void SetUnidirectionalLinkFlag ( bool val=true )
  {
    if ( val )
      m_flags |= UNIDIRECTIONAL;
    else
      m_flags &= ~UNIDIRECTIONAL;
  }

  /**
   * Check the packet flags for the UNIDIRECTIONAL LINK bit being set.
   * \return TRUE if the unidirectional link flag is asserted.
   */
  bool HasUnidirectionalLinkFlag () const
  {
    return m_flags & UNIDIRECTIONAL;
  }

  /**
   * Set the packet TTL as a uint8_t (byte) field.
   * \param flags The packet TTL as a byte.
   */
  void SetTtl (uint8_t ttl)
  {
    m_ttl = ttl;
  }

  /**
   * Get the packet TTL as a uint8_t (byte) field.
   * \return The packet TTL as a byte.
   */
  uint8_t GetTtl () const
  {
    return m_ttl;
  }

  /**
   * Set the packet Gateway flags as a uint8_t (byte) field.
   * \param flags The packet Gateway flags as a byte.
   */
  void SetGWFlags (uint8_t flags)
  {
    m_gwflags = flags;
  }

  /**
   * Get the packet Gateway flags as a uint8_t (byte) field.
   * \return The packet Gateway flags as a byte.
   */
  uint8_t GetGWFlags () const
  {
    return m_gwflags;
  }

  /**
   * Set the packet sequence number.
   * \param seqnum The packet sequence number.
   */
  void SetPacketSequenceNumber (uint16_t seqnum)
  {
    m_packetSequenceNumber = seqnum;
  }

  /**
   * Get the packet sequence number.
   * \returns The packet sequence number.
   */
  uint16_t GetPacketSequenceNumber () const
  {
    return m_packetSequenceNumber;
  }

  void IncrementSequenceNumber() {
    ++m_packetSequenceNumber;
  }

  void DecrementSequenceNumber() {
    --m_packetSequenceNumber;
  }

  /**
   * Set the advertised gateway port number.
   * \param port The advertised gateway port number.
   */
  void SetGatewayPortNumber (uint16_t port)
  {
    m_gatewayPortNumber = port;
  }

  /**
   * Get the advertised gateway port number.
   * \returns The advertised gateway port number.
   */
  uint16_t GetGatewayPortNumber () const
  {
    return m_gatewayPortNumber;
  }

  /**
   * Set the IPv4 address of the OGM origin station.
   * \param address The IPv4 address of the OGM origin station.
   */
  void SetOriginatorAddress (Ipv4Address address)
  {
     m_originatorAddress = address;
  }

  /**
   * Get the IPv4 address of the OGM origin station.
   * \returns The IPv4 address of the OGM origin station.
   */
  Ipv4Address GetOriginatorAddress () const
  {
    return m_originatorAddress;
  }

  /**
   * Set the IPv4 address of the forwarding station.
   * \param address The IPv4 address of the forwarding station.
   */
  void SetForwarderAddress (Ipv4Address address)
  {
     m_previousAddress = address;
  }

  /**
   * Get the IPv4 address of the forwarding station.
   * \returns The IPv4 address of the forwarding station.
   */
  Ipv4Address GetForwarderAddress () const
  {
    return m_previousAddress;
  }

  /**
   * Set the packet TQ value as a uint8_t (byte) field.
   * \param TQ  The packet TQ value as a byte.
   */
  void SetTQvalue (uint8_t TQ)
  {
    m_tq = TQ;
  }

  /**
   * Get the packet TQ value as a uint8_t (byte) field.
   * \return The packet TQ value as a byte.
   */
  uint8_t GetTQvalue () const
  {
    return m_tq;
  }

  /**
   * Set the packet HNA field length as a uint8_t (byte) field.
   * This method must be called post HNA addition to set the HNA
   * Length field.  Otherwise, we'd be copying a 0 into the packet
   * buffer.
   */
  void SetHnaLength ()
  {
    m_hnaLength = m_hnaList.size();
  }

  /**
   * Get the packet HNA field length as a uint8_t (byte) field.
   *
   * NOTE: This could change by adding or removing HNA entries
   * form the structure.  This is a temporary place holder until
   * the HNA add/remove methods are added.
   *
   * \return The packet Gateway flags as a byte.
   */
  uint8_t GetHnaLength () const
  {
    return m_hnaLength;
  }

  void ClearHnaEntries() {
    m_hnaList.clear();
    m_hnaLength = 0;
  }

  uint8_t AddHnaEntry(struct hna_local_entry data) {
    m_hnaList.push_back( data );
    return m_hnaList.size();
  }
  
  hna_local_entry GetAssociation( unsigned int offset ) {
    hna_local_entry retval = {0,0};
    
    if ( offset < m_hnaLength ) 
      retval = m_hnaList[offset];
    
    return retval;
  }

  /**
   * This method is used by the BATMAN daemon in NS-3 to
   * convert an unsigned char[] to the OGMHeader class.
   *
   * \param start a pointer to an unsigned char array.
   * \param messageSize the message size.
   * \returns the number of bytes read.
   */
  uint32_t Deserialize (unsigned char *start, uint32_t messageSize);

  /*
   * This method is the public facing method to called to get the
   * serialized byte array of HNA entries. The unsigned int parameter
   * is passed by reference so that the caller's variable is updated
   * along with the returned byte array pointer on the heap.
   *
   * \param uint16_t            Reference to a uint16_t variable to return the length
   *                            of the new'ed heap byte array
   * \return unsigned char *    new heap byte array
   */
  unsigned char *SerializeHnaList(int16_t &length);
  
  struct bat_packet *GetBatmanPacket();

  unsigned char *Serialize();


//private:
  uint8_t       m_version;               //!< BATMAN OGM versions
  uint8_t       m_flags;                 //!< OGM Flags field
  uint8_t       m_ttl;                   //!< OGM TTL count
  uint8_t       m_gwflags;               //!< OGM Gateway Flags
  uint16_t      m_packetSequenceNumber;  //!< OGM Sequence Number
  uint16_t      m_gatewayPortNumber;     //!< Gateway port supported by the BATMAN node
  Ipv4Address   m_originatorAddress;     //!< IPv4 Address of the originator of the packet
  Ipv4Address   m_previousAddress;       //!< IPv4 Address of the current sending station
  uint8_t       m_tq;                    //!< TQ value ??? original code provides no explanation
  uint8_t       m_hnaLength;             //!< Length of the succeeding HNA field
  HnaLocalList  m_hnaList;               //!< HNA addresses

  uint8_t  *m_array;                //!< byte array of OGM message

//public:
    
  /**
   * \brief Get the type ID.
   * \return The object TypeId.
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
};


static inline std::ostream& operator<< (std::ostream& os, const OGMHeader & packet)
{
  packet.Print (os);
  return os;
}

}
}  // namespace batmand, ns3

#endif /* OLSR_HEADER_H */

