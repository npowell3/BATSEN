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
 * Code based on the BATMAND module and modified for BATMAND-0.3.2
 * implementation.  BATMAND was the predecessor for BATMAN and has many
 * similar features.  Plus, modifying the BATMAND module reduces the
 * effort required to write a module from scratch.
 *
 * The BATMAN module is based on the IETF draft found at
 * https://tools.ietf.org/html/draft-openmesh-b-a-t-m-a-n-00 and the
 * BATMAN-0.3.2 code base downloadable from
 * https://www.open-mesh.org/projects/open-mesh/wiki/Download
 *
 *
 */

///
/// \brief Implementation of BATMAND agent and related classes.
///
/// This is the main file of this software because %BATMAND's behavior is
/// implemented here.
///

#ifndef BATMAND_STRUCTURES_H
#define BATMAND_STRUCTURES_H

#include <set>
#include <vector>
#include <list>

#include "batmand-header.h"
#include "ns3/log.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"

namespace ns3 {
namespace batmand {

/* sliding packet range of received originator messages in sequence numbers
 * (should be a multiple of our word size)
 */
#define TQ_LOCAL_WINDOW_SIZE  64
#define TYPE_OF_WORD          uint64_t

#define TQ_HOP_PENALTY		5

static inline bool
operator == (const bat_packet &a, const bat_packet &b)
{
  return ( a.orig == b.orig &&
           a.seqno == b.seqno &&
           a.ttl == b.ttl );
}

static inline std::ostream&
operator << (std::ostream &os, const bat_packet &pkt)
{
  os << "BATMAN Pkt(OrigAddr=" << pkt.orig
     << ", PrevAddr=" << pkt.prev_sender
     << ", TTL=" << pkt.ttl
     << ", SeqNo=" << pkt.seqno
     << ", Flags=" << pkt.flags
     << ", TQ=" << pkt.tq
     << ", HNALen=" << pkt.hna_len
     << ")";
  return os;
}

/// \ingroup batmand
/// Data structure for a BATMAND enabled interface
struct batman_if
{
   int32_t udp_send_sock;
   int32_t udp_recv_sock;
   int32_t udp_tunnel_sock;
   uint8_t if_num;
   uint8_t if_active;
   int32_t if_index;
   int8_t if_rp_filter_old;
   int8_t if_send_redirects_old;
   pthread_t listen_thread_id;
   Ipv4Address addr;
   Ipv4Address broad;
   Ipv4Address netaddr;
   uint8_t netmask;
   uint8_t wifi_if;
   OGMHeader out;
};

static inline bool
operator == (const batman_if &a, const batman_if &b)
{
  return ( a.addr == b.addr &&
           a.if_num == b.if_num &&
           a.out.GetPacketSequenceNumber() == b.out.GetPacketSequenceNumber());
}

static inline std::ostream&
operator << (std::ostream &os, const batman_if &batmanif)
{
  os << "BATMAN Iface(Addr=" << batmanif.addr
     << ", bcastAddr=" << batmanif.broad
     << ", ifaceNumber=" << (int) batmanif.if_num
     << ")";
  return os;
}

// Forward declare the orig_node
struct orig_node;


struct neigh_node
{
   Ipv4Address addr;
   uint8_t real_packet_count;
   uint8_t tq_recv[TQ_GLOBAL_WINDOW_SIZE];
   uint8_t tq_index;
   uint8_t tq_avg;
   uint8_t last_ttl;
   int64_t last_valid;            /* when last packet via this neighbor was received */
   uint64_t real_bits;
   struct orig_node *orig_node;
   uint16_t ifnum;                /* replaced the pointer to the BATMAN interface with an IF number */
   batman_if *if_incoming;
};

typedef std::map<uint8_t, uint64_t>		BcastWindowMap;
typedef std::map<uint8_t, uint8_t>		BcastSumMap;

typedef std::vector<neigh_node>          NeighborList;   //!< Vector of known neighbors.

/// \ingroup batmand
/// Data structure for orig_list maintaining nodes of mesh
struct orig_node
{
  orig_node() :
    router( NULL ),
    batman_if( NULL ),
    tq_own( 0 ),
    tq_asym_penalty( 0 ),
    last_valid( 0 ),
    gwflags( 0 ),
    hna_buff( NULL ),
    hna_buff_len( 0 ),
    last_real_seqno( 0 ),
    last_ttl( 0 )
  {
    orig.Set( (uint32_t)0 );
    neigh_list.clear();
    bcast_own[0] = 0lu;
    bcast_own_sum[0] = 0;
  };

   Ipv4Address orig;
   struct neigh_node *router;
   struct batman_if *batman_if;
   OGMHeader in;
   BcastWindowMap bcast_own;
   BcastSumMap bcast_own_sum;
   uint8_t tq_own;
   int tq_asym_penalty;
   uint32_t last_valid;          /* when last packet from this node was received */
   uint8_t  gwflags;             /* flags related to gateway functions: gateway class */
   unsigned char *hna_buff;
   int16_t  hna_buff_len;
   uint16_t last_real_seqno;     /* last and best known sequence number */
   uint8_t last_ttl;             /* ttl of last received packet */
   NeighborList neigh_list;
};

static inline bool
operator == (const orig_node &a, const orig_node &b)
{
  return ( a.orig == b.orig &&
           a.batman_if == b.batman_if &&
           a.router == b.router);
}

static inline std::ostream&
operator << (std::ostream &os, const orig_node &neigh)
{
  os << "Neighbor Node(Addr=" << neigh.orig
     << ", ifaceNumber=" << neigh.batman_if->if_num
     << ", TTL=" << (int) neigh.last_ttl
     << ", TQ=" << (int) neigh.tq_own
     << ")";
  return os;
}

typedef std::vector<OGMHeader>           MessageList;    //!< Vector of OGM messages to be transmitted

struct forw_node                 /* structure for forw_list maintaining packets to be send/forwarded */
{
   forw_node() {
     pack_buff = NULL;
     if_incoming = NULL;
     msgList.clear();
   };
   
   ~forw_node() {
     delete pack_buff;
     if_incoming = NULL;
     msgList.clear();
   };
   
   ns3::Time send_time;
   uint8_t  own;
   unsigned char *pack_buff;
   uint16_t  pack_buff_len;
   uint32_t direct_link_flags;
   uint8_t num_packets;
   Ipv4Address srcAddr;
   MessageList msgList;
   struct batman_if *if_incoming;
};


struct gw_node
{
  struct orig_node *orig_node;
  uint16_t gw_port;
  uint16_t gw_failure;
  uint32_t last_failure;
  Time deleted;
};

typedef std::vector<batman_if *>         BatmanIfList;   //!< Vector of net devices supporting BATMAN protocol.
typedef std::vector<orig_node *>         OriginList;     //!< Vector of known neighbors.
typedef std::list<forw_node *>           ForwardingList; //!< Vector of packets to be forwarded for other nodes.
typedef std::vector<gw_node *>           GatewayList;    //!< Vector of gateway nodes

static inline std::ostream& operator<< (std::ostream& os, const MessageList & messages)
{
  os << "[";
  for (MessageList::const_iterator messageIter = messages.begin ();
       messageIter != messages.end (); )
    {
      messageIter->Print (os);
      messageIter++;
      
      if (messageIter != messages.end ())
        {
          os << ", ";
        }
    }
  os << "]";
  return os;
}


/********** Miscellaneous constants **********/
/// Maximum allowed jitter.
#define BATMAND_MAXJITTER          (m_ogmInterval.GetSeconds () / 6)
/// Maximum allowed sequence number.
#define BATMAND_MAX_SEQ_NUM        65534
/// Random number between [0-BATMAND_MAXJITTER] used to jitter BATMAND packet transmission.
#define JITTER (Seconds (m_uniformRandomVariable->GetValue (0, BATMAND_MAXJITTER)))

/// Maximum number of messages per packet.
#define BATMAND_MAX_MSGS           64
/// Maximum number of hellos per message (4 possible link types * 3 possible nb types).
#define BATMAND_MAX_HELLOS         12
/// Maximum number of addresses advertised on a message.
#define BATMAND_MAX_ADDRS          64
/// purge originators after time in ms if no valid packet comes in -> TODO: check influence on
#define BATMAN_PURGE_TIMEOUT 	200000u

}
}  // namespace ns3, batmand

#endif /* BATMAND_STRUCTURES_H */
