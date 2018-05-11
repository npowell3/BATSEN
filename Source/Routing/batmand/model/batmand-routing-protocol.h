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

#ifndef BATMAND_H
#define BATMAND_H

#include "ns3/test.h"
#include "batmand-structures.h"

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/node.h"
#include "ns3/socket.h"
#include "ns3/event-garbage-collector.h"
#include "ns3/random-variable-stream.h"
#include "ns3/timer.h"
#include "ns3/traced-callback.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-static-routing.h"

#include <vector>
#include <map>
#include <iostream>
#include <fstream>

/// Testcase for MPR computation mechanism
class BatmandMprTestCase;

namespace ns3 {
namespace batmand {

///
/// \defgroup batmand BATMAND Routing
/// This section documents the API of the ns-3 BATMAND module. For a generic
/// functional description, please refer to the ns-3 manual.

#define BATMAN_RT_TABLE_NETWORKS  65
#define BATMAN_RT_TABLE_HOSTS     66
#define BATMAN_RT_TABLE_UNREACH   67
#define BATMAN_RT_TABLE_TUNNEL    68

#define BATMAN_RT_PRIO_DEFAULT 6600
#define BATMAN_RT_PRIO_UNREACH BATMAN_RT_PRIO_DEFAULT + 100
#define BATMAN_RT_PRIO_TUNNEL BATMAN_RT_PRIO_UNREACH + 100

/// \ingroup batmand
/// An %BATMAND's routing table entry.
struct RoutingTableEntry
{
  Ipv4Address destAddr; //!< Address of the destination node.
  Ipv4Address nextAddr; //!< Address of the next hop.
  uint32_t interface; //!< Interface index
  uint32_t distance; //!< Distance in hops to the destination.

  RoutingTableEntry () : // default values
    destAddr ( (uint32_t)0 ), 
    nextAddr ( (uint32_t)0 ),
    interface (0), 
    distance (0)
  {
  }
  
  ~RoutingTableEntry (){};
};


typedef int (*hashdata_compare_cb)(void *, void *);
typedef int (*hashdata_choose_cb)(void *, int);
typedef void (*hashdata_free_cb)(void *);

struct element_t {
  void *data;					/* pointer to the data */
  struct element_t *next;			/* overflow bucket pointer */
};

struct hash_it_t {
  int index;
  struct element_t *bucket;
  struct element_t *prev_bucket;
  struct element_t **first_bucket;
};

struct hashtable_t {
  struct element_t **table;		/* the hashtable itself, with the buckets */
  int elements;				/* number of elements registered */
  int size;				/* size of hashtable */
  hashdata_compare_cb compare;		/* callback to a compare function.
 					 * should compare 2 element datas for their keys,
					 * return 0 if same and not 0 if not same */
  hashdata_choose_cb choose;		/* the hashfunction, should return an index based
					 * on the key in the data of the first argument
					 * and the size the second */
};

struct hna_element
{
  uint32_t addr;
  uint8_t  netmask;
} __attribute__((packed));

struct hna_task
{
  uint32_t addr;
  uint8_t netmask;
  uint8_t route_action;
};



struct hna_global_entry
{
  uint32_t addr;
  uint8_t netmask;
  struct orig_node *curr_orig_node;
  OriginList originList;
}; // __attribute__((packed));

struct hna_orig_ptr
{
  struct orig_node *orig_node;
};

typedef std::vector<hna_global_entry> HnaGlobalList;   //!< Vector of net devices supporting BATMAN protocol.
typedef std::vector<hna_task>         HnaTaskList;     //!< Vector of net devices supporting BATMAN protocol.

typedef std::map<Ipv4Address, RoutingTableEntry> RouteTableMap;


class RoutingProtocol;

///
/// \ingroup batmand
///
/// \brief BATMAND routing protocol for IPv4
///
class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  friend class ::BatmandMprTestCase;

  /**
   * \brief Get the type ID.
   * \return The object TypeId.
   */
  static TypeId GetTypeId (void);

  RoutingProtocol ();
  virtual ~RoutingProtocol ();

  /**
   * \brief Set the BATMAND main address to the first address on the indicated interface.
   *
   * \param interface IPv4 interface index
   */
  void SetMainInterface (uint32_t interface);

  /**
   * Dump the neighbor table, two-hop neighbor table, and routing table
   * to logging output (NS_LOG_DEBUG log level).  If logging is disabled,
   * this function does nothing.
   */
  void Dump (void);

  /**
   * Return the list of routing table entries discovered by BATMAND
   */
  std::vector<RoutingTableEntry> GetRoutingTableEntries () const;

  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  int64_t AssignStreams (int64_t stream);

  /**
   * TracedCallback signature for Packet transmit and receive events.
   *
   * \param [in] header
   * \param [in] messages
   */
  typedef void (* PacketTxRxTracedCallback)(const OGMHeader & header, const MessageList & messages);

  /**
   * TracedCallback signature for routing table computation.
   *
   * \param [in] size Final routing table size.
   */
  typedef void (* TableChangeTracedCallback) (uint32_t size);

private:
  std::set<uint32_t> m_interfaceExclusions; //!< Set of interfaces excluded by OSLR.
  Ptr<Ipv4StaticRouting> m_routingTableAssociation; //!< Associations from an Ipv4StaticRouting instance

public:
  /**
   * Get the excluded interfaces.
   * \returns Container of excluded interfaces.
   */
  std::set<uint32_t> GetInterfaceExclusions () const
  {
    return m_interfaceExclusions;
  }

  /**
   * Set the interfaces to be excluded.
   * \param exceptions Container of excluded interfaces.
   */
  void SetInterfaceExclusions (std::set<uint32_t> exceptions);

  /**
   *  \brief Injects the specified (networkAddr, netmask) tuple in the list of
   *  local HNA associations to be sent by the node via HNA messages.
   *  If this tuple already exists, nothing is done.
   *
   * \param networkAddr The network address.
   * \param netmask The network mask.
   */
  void AddHostNetworkAssociation (Ipv4Address networkAddr, Ipv4Mask netmask);

  /**
   * \brief Removes the specified (networkAddr, netmask) tuple from the list of
   * local HNA associations to be sent by the node via HNA messages.
   * If this tuple does not exist, nothing is done (see "BatmandState::EraseAssociation()").
   *
   * \param networkAddr The network address.
   * \param netmask The network mask.
   */
  void RemoveHostNetworkAssociation (Ipv4Address networkAddr, Ipv4Mask netmask);

  /**
   * \brief Associates the specified Ipv4StaticRouting routing table
   *         to the BATMAND routing protocol. Entries from this associated
   *         routing table that use non-batmand outgoing interfaces are added
   *         to the list of local HNA associations so that they are included
   *         in HNA messages sent by the node.
   *         If this method is called more than once, entries from the old
   *         association are deleted before entries from the new one are added.
   *  \param routingTable the Ipv4StaticRouting routing table to be associated.
   */
  void SetRoutingTableAssociation (Ptr<Ipv4StaticRouting> routingTable);

  bool UsesNonOlsrOutgoingInterface (const Ipv4RoutingTableEntry &route);
  
  void RoutingTableComputation ();

  /**
   * \brief Returns the internal HNA table
   * \returns the internal HNA table
   */
  Ptr<const Ipv4StaticRouting> GetRoutingTableAssociation () const;

protected:
  virtual void DoInitialize (void);
private:
  RouteTableMap m_table; //!< Data structure for the routing table.

  Ptr<Ipv4StaticRouting> m_hnaRoutingTable; //!< Routing table for HNA routes

  EventGarbageCollector m_events; //!< Running events.

  uint16_t m_packetSequenceNumber;    //!< Packets sequence number counter.

  Ptr<Ipv4> m_ipv4;     //!< IPv4 object the routing is linked to.

  /**
   * \brief Clears the routing table and frees the memory assigned to each one of its entries.
   */
  void Clear ();

  /**
   * Returns the routing table size.
   * \return The routing table size.
   */
  uint32_t GetSize () const
  {
    return m_table.size ();
  }

  /**
   * \brief Deletes the entry whose destination address is given.
   * \param dest address of the destination node.
   */
  void RemoveEntry (const Ipv4Address &dest);
  /**
   * \brief Adds a new entry into the routing table.
   *
   * If an entry for the given destination existed, it is deleted and freed.
   *
   * \param dest address of the destination node.
   * \param next address of the next hop node.
   * \param interface address of the local interface.
   * \param distance distance to the destination node.
   */
  void AddEntry (const Ipv4Address &dest,
                 const Ipv4Address &next,
                 uint32_t interface,
                 uint32_t distance);
  /**
   * \brief Adds a new entry into the routing table.
   *
   * If an entry for the given destination existed, an error is thrown.
   *
   * \param dest address of the destination node.
   * \param next address of the next hop node.
   * \param interfaceAddress address of the local interface.
   * \param distance distance to the destination node.
   */
  void AddEntry (const Ipv4Address &dest,
                 const Ipv4Address &next,
                 const Ipv4Address &interfaceAddress,
                 uint32_t distance);

  /**
   * \brief Changes the values in an existing entry into the routing table.
   *
   * If an entry for the given destination existed, the data is updated;
   * otherwise, nothing is done.
   *
   * \param dest address of the destination node.
   * \param next address of the next hop node.
   * \param interfaceAddress address of the local interface.
   * \param distance distance to the destination node.
   */
  void UpdateEntry (Ipv4Address const &dest,
                    Ipv4Address const &next,
                    uint32_t interface,
                    uint32_t distance);
  
  /**
   * \brief Changes the values in an existing entry into the routing table.
   *
   * If an entry for the given destination existed, the data is updated;
   * otherwise, nothing is done.
   *
   * \param dest address of the destination node.
   * \param next address of the next hop node.
   * \param interfaceAddress address of the local interface.
   * \param distance distance to the destination node.
   */
  void UpdateEntry (Ipv4Address const &dest,
                    Ipv4Address const &next,
                    Ipv4Address const &interfaceAddress,
                    uint32_t distance);
  
  /**
   * \brief Looks up an entry for the specified destination address.
   * \param [in] dest Destination address.
   * \param [out] outEntry Holds the routing entry result, if found.
   * \return true if found, false if not found.
   */
  bool Lookup (const Ipv4Address &dest,
               RoutingTableEntry &outEntry) const;

  /**
   * \brief Finds the appropriate entry which must be used in order to forward
   * a data packet to a next hop (given a destination).
   *
   * Imagine a routing table like this: [A,B] [B,C] [C,C]; being each pair of the
   * form [dest addr, next-hop addr]. In this case, if this function is invoked
   * with [A,B] then pair [C,C] is returned because C is the next hop that must be used
   * to forward a data packet destined to A. That is, C is a neighbor of this node,
   * but B isn't. This function finds the appropriate neighbor for forwarding a packet.
   *
   * \param[in] entry The routing table entry which indicates the destination node
   * we are interested in.
   *
   * \param[out] outEntry The appropriate routing table entry which indicates the next
   * hop which must be used for forwarding a data packet, or NULL if there is no such entry.
   *
   * \return True if an entry was found, false otherwise.
   */
  bool FindSendEntry (const RoutingTableEntry &entry,
                      RoutingTableEntry &outEntry) const;

  // From Ipv4RoutingProtocol
  virtual Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p,
                                      const Ipv4Header &header,
                                      Ptr<NetDevice> oif,
                                      Socket::SocketErrno &sockerr);
  virtual bool RouteInput (Ptr<const Packet> p,
                           const Ipv4Header &header,
                           Ptr<const NetDevice> idev,
                           UnicastForwardCallback ucb,
                           MulticastForwardCallback mcb,
                           LocalDeliverCallback lcb,
                           ErrorCallback ecb);
  virtual void NotifyInterfaceUp (uint32_t interface);
  virtual void NotifyInterfaceDown (uint32_t interface);
  virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void SetIpv4 (Ptr<Ipv4> ipv4);
  virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S) const;

  void DoDispose ();

  /**
   * Send an BATMAND message.
   * \param packet The packet to be sent.
   * \param containedMessages The messages contained in the packet.
   */
  void SendPacket (Ptr<Packet> packet, const MessageList &containedMessages);

  /**
   * Increments packet sequence number and returns the new value.
   * \return The packet sequence number.
   */
  inline uint16_t GetPacketSequenceNumber ();

  /**
   * Receive an BATMAND message.
   * \param socket The receiving socket.
   */
  void RecvBatmanOgm (Ptr<Socket> socket);

  int isBidirectionalNeigh(struct orig_node *orig_node,
                           struct orig_node *orig_neigh_node,
                           OGMHeader &in,
                           int64_t recv_time,
                           uint8_t ifnum);

  /**
   * \brief Gets the main address associated with a given interface address.
   * \param iface_addr the interface address.
   * \return the corresponding main address.
   */
//  Ipv4Address GetMainAddress (Ipv4Address iface_addr) const;

  /**
   *  \brief Tests whether or not the specified route uses a non-BATMAND outgoing interface.
   *  \param route The route to be tested.
   *  \returns True if the outgoing interface of the specified route is a non-BATMAND interface, false otherwise.
   */
  bool UsesNonBatmandOutgoingInterface (const Ipv4RoutingTableEntry &route);

  // Timer handlers
  Timer m_ogmTimer; //!< Timer for the OGM message.
  /**
   * \brief Sends a OGM message and reschedules the OGM timer.
   */
  void OgmTimerExpire ();

  /**
   * Increments the ANSN counter.
   */
  void IncrementAnsn ();

  /// A list of pending messages which are buffered awaiting for being sent.
  MessageList m_queuedMessages;
  Timer m_queuedMessagesTimer; //!< timer for throttling outgoing messages

  /**
   * \brief Enques an %BATMAND message which will be sent with a delay of (0, delay].
   *
   * This buffering system is used in order to piggyback several %BATMAND messages in
   * a same %BATMAND packet.
   *
   * \param message the %BATMAND message which must be sent.
   * \param delay maximum delay the %BATMAND message is going to be buffered.
   */
  void QueueMessage (const batmand::OGMHeader &message, Time delay);

  /**
   * \brief Creates as many %BATMAND packets as needed in order to send all buffered
   * %BATMAND messages.
   *
   * Maximum number of messages which can be contained in an %BATMAND packet is
   * dictated by BATMAND_MAX_MSGS constant.
   */
  void SendQueuedMessages ();

  /// Check that address is one of my interfaces
  bool IsMyOwnAddress (const Ipv4Address & a) const;

  Ipv4Address m_mainAddress; //!< the node main address.

  // One socket per interface, each bound to that interface's address
  // (reason: for BATMAND Link Sensing we need to know on which interface
  // HELLO messages arrive)
  std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_socketAddresses; //!< Container of sockets and the interfaces they are opened onto.

  /// Rx packet trace.
  TracedCallback <const OGMHeader &, const MessageList &> m_rxPacketTrace;

  /// Tx packet trace.
  TracedCallback <const OGMHeader &, const MessageList &> m_txPacketTrace;

  /// Routing table chanes challback
  TracedCallback <uint32_t> m_routingTableChanged;
  
  /// Callbacks for tracing the packet Tx events
  TracedCallback<Ptr<const Packet> > m_txTrace;


protected:
  BatmanIfList    m_batmanif;
  OriginList      m_OriginSet;
  NeighborList    m_neighborSet;
  ForwardingList  m_forwardingSet;

public:

  void AddBatmanIface(batman_if *iface);

  struct batman_if *FindBatmanIf( Ipv4Address addr );
  
  struct batman_if *FindBatmanIf( uint8_t ifnum );

  struct orig_node *GetOrCreateOrigNode( Ipv4Address addr, int ifnum=-1 );

  struct orig_node *FindOrigNode( Ipv4Address addr );

  void purge_orig(ns3::Time curr_time);
  
  void debug_orig(void);
  
  struct neigh_node *FindNeighborNode( struct orig_node *orig_node, Ipv4Address addr );

  /*
   * Mark a bit in the 64 bit sequence field - a bit vector marking received
   * OGM messages based on sequence number.
   *
   * @param uint64_t      Reference to the sequence bits field
   * @param int32_t       Bit number to be set
   */
  void MarkSequenceBit( uint64_t &seq_bits, int32_t n );

  /*
   * Mark a bit in the 64 bit sequence field - a bit vector marking received
   * OGM messages based on sequence number.
   *
   * @param uint64_t      Reference to the sequence bits field
   * @param int           Number of bits set in the 64 bit word
   */
  int Uint64BitCount( uint64_t &seq_bits );

  /*
   *
   * \return TRUE if the packet is a duplicate
   */
  bool CountRealPackets( OGMHeader &pkt, Ipv4Address neigh, uint8_t ifnum );

  /*
   * \brief This method checks the OGM bit for a given sequence number to determine
   * if the origin node was heard from at a given point in time.  Notice that
   * all arguments are pass by reference as pass by value is slower.
   *
   * REVERSE ENGINEERING NOTES:
   * - I think this is attempting to check the sequence bit from an OGM in the past
   * As CURR > LAST seqno, DIFF goes negative, so this reception could not
   * be a duplicate reception.
   *
   * - If the LAST == CURR, then this is a duplicate and the bit should theoretically
   * be set as we've heard from it.
   *
   * - If LAST > CURR, then we've received an old OGM from someone.
   *
   * \param uint64_t    Local window bit vector
   * \param uint16_t    Last sequence number
   * returns true if corresponding bit in given seq_bits indicates so and curr_seqno is within range of last_seqno
   */
  bool GetHistoricalSequenceBitStatus( uint64_t &seq_bits, int32_t &last_seqno, int32_t &curr_seqno );

  /* receive and process one packet, returns 1 if received seq_num is considered new, 0 if old  */
  bool bit_get_packet( uint64_t &seq_bits, int16_t seq_num_diff, int8_t set_mark );

  /* shift the packet array p by n places. */
  void bit_shift( uint64_t &seq_bits, int32_t n );

  struct neigh_node *CreateNeighbor( struct orig_node *orig_node,
                                     struct orig_node *orig_neigh_node,
                                     Ipv4Address neigh,
                                     uint8_t ifnum );

  void ScheduleForwardPacket( struct orig_node *orig_node, OGMHeader &in,
                              Ipv4Address neigh, uint8_t directlink, int16_t hna_buff_len,
                              uint8_t ifnum, ns3::Time curr_time);

  void UpdateOrig( struct orig_node *orig_node, OGMHeader &in,
                   Ipv4Address neigh, uint16_t ifnum,
                   unsigned char *hna_recv_buff, int16_t hna_buff_len,
                   bool is_duplicate, ns3::Time curr_time );

  void ring_buffer_set(uint8_t tq_recv[], uint8_t *tq_index, uint8_t value);

  uint8_t ring_buffer_avg(uint8_t tq_recv[]);

  void UpdateRoutes(struct orig_node *orig_node,
                    struct neigh_node *neigh_node,
                    unsigned char *hna_recv_buff,
                    int16_t hna_buff_len);

  void UpdateGatewayList(struct orig_node *orig_node, uint8_t new_gwflags, uint16_t gw_port);
  
  void ChooseGateway(void);

  void get_gw_speeds(unsigned char gw_class, int *down, int *up);

  unsigned char get_gw_class(int down, int up);

  void send_outstanding_packets(ns3::Time curr_time);
  
  void schedule_own_packet(struct batman_if *batman_if);
  
  int compare_hna(void *data1, void *data2);
  
  int choose_hna(void *data, int32_t size);
  
  void hna_init(void);
  
  void HNAGlobalAdd(struct orig_node *orig_node, unsigned char *new_hna, int16_t new_hna_len);

  void _hna_global_add(struct orig_node *orig_node, struct hna_element *hna_element);
  
  void HNAGlobalDel(struct orig_node *orig_node);
  
  void _hna_global_del(struct orig_node *orig_node, struct hna_element *hna_element);
  
  void HNAGlobalUpdate(struct orig_node *orig_node, unsigned char *new_hna,
                       int16_t new_hna_len, struct neigh_node *old_router);

  int HNABuffDelete(struct hna_element *buf, int *buf_len, struct hna_element *e);

    /* clears the hash */
  void  hash_init(struct hashtable_t *hash);

  /* allocates and clears the hash */
  struct hashtable_t *hash_new( int size );

  /* remove bucket (this might be used in hash_iterate() if you already found the bucket
  * you want to delete and don't need the overhead to find it again with hash_remove().
  * But usually, you don't want to use this function, as it fiddles with hash-internals. 
  */
  void *hash_remove_bucket(struct hashtable_t *hash, struct hash_it_t *hash_it_t);

  /* remove the hash structure. if hashdata_free_cb != NULL,
  * this function will be called to remove the elements inside of the hash.
  * if you don't remove the elements, memory might be leaked. 
  */
  void hash_delete(struct hashtable_t *hash, hashdata_free_cb free_cb);

  /* free only the hashtable and the hash itself. */
  void hash_destroy(struct hashtable_t *hash);

  /* adds data to the hashtable. returns 0 on success, -1 on error */
  int hash_add(struct hashtable_t *hash, void *data);

  /* removes data from hash, if found. returns pointer do data on success,
  * so you can remove the used structure yourself, or NULL on error .
  * data could be the structure you use with just the key filled,
  * we just need the key for comparing. 
  */
  void *hash_remove(struct hashtable_t *hash, void *data);

  /* finds data, based on the key in keydata. returns the found data on success, or NULL on error */
  void *hash_find(struct hashtable_t *hash, void *keydata);


  /* print the hash table for debugging */
  void hash_debug( struct hashtable_t *hash);

  /* resize the hash, returns the pointer to the new hash or NULL on error. removes the old hash on success */
  struct hashtable_t *hash_resize(struct hashtable_t *hash, int size);

  /* iterate though the hash. first element is selected with iter_in NULL.
  * use the returned iterator to access the elements until hash_it_t returns NULL. 
  */
  struct hash_it_t *hash_iterate(struct hashtable_t *hash, struct hash_it_t *iter_in);
    
  
  bool m_aggregation;

  uint8_t m_hopPenalty;

    /// Provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;
  Time m_ogmInterval;     //!< OGM messages' emission interval.
  Time m_purgeTimeout;    //!< Timeout to purge a  neighbor if we haven't received an OGM in a while

  uint8_t m_routingClass;
  uint8_t m_numberHnalocal;

  struct gw_node *m_currGateway;
  Ipv4Address m_prefGateway;

  GatewayList m_GatewayList;
  
  HnaGlobalList m_GlobalList;
  HnaLocalList  m_LocalList;
  HnaTaskList	m_TaskList;
  
  struct hashtable_t *m_hnaGlobalHash;
    
  ns3::Time debug_timeout, vis_timeout;
  
  std::ofstream m_outfile;
};

}

}

#endif /* BATMAND_H */

