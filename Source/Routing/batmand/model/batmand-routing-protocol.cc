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


#define NS_LOG_APPEND_CONTEXT                                   \
  if (GetObject<Node> ()) { std::clog << "[node " << GetObject<Node> ()->GetId () << "] "; }


#include "batmand-routing-protocol.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/nstime.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-route.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/ipv4-header.h"
#include "ns3/internet-module.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

template <typename T>
std::string ToString(T val)
{
  std::stringstream stream;
  stream << val;
  return stream.str();
}

// std::ostringstream os;

/********** Useful macros **********/

///
/// \brief Gets the delay between a given time and the current time.
///
/// If given time is previous to the current one, then this macro returns
/// a number close to 0. This is used for scheduling events at a certain moment.
///
#define DELAY(time) (((time) < (Simulator::Now ())) ? Seconds (0.000001) : \
                     (time - Simulator::Now () + Seconds (0.000001)))

///
/// \brief Period at which a node must cite every link and every neighbor.
///
/// We only use this value in order to define BATMAND_NEIGHB_HOLD_TIME.
///
#define BATMAND_REFRESH_INTERVAL   m_ogmInterval


/********** Holding times **********/
/// Neighbor holding time.
#define BATMAND_NEIGHB_HOLD_TIME   Time (3 * BATMAND_REFRESH_INTERVAL)
/// Top holding time.
#define BATMAND_TOP_HOLD_TIME      Time (3 * m_tcInterval)
/// Dup holding time.
#define BATMAND_DUP_HOLD_TIME      Seconds (30)
/// MID holding time.
#define BATMAND_MID_HOLD_TIME      Time (3 * m_midInterval)
/// HNA holding time.
#define BATMAND_HNA_HOLD_TIME      Time (3 * m_hnaInterval)

/********** Link types **********/
/// Unspecified link type.
#define BATMAND_UNSPEC_LINK        0
/// Asymmetric link type.
#define BATMAND_ASYM_LINK          1
/// Symmetric link type.
#define BATMAND_SYM_LINK           2
/// Lost link type.
#define BATMAND_LOST_LINK          3

/********** Neighbor types **********/
/// Not neighbor type.
#define BATMAND_NOT_NEIGH          0
/// Symmetric neighbor type.
#define BATMAND_SYM_NEIGH          1
/// Asymmetric neighbor type.
#define BATMAND_MPR_NEIGH          2


/********** Willingness **********/
/// Willingness for forwarding packets from other nodes: never.
#define BATMAND_WILL_NEVER         0
/// Willingness for forwarding packets from other nodes: low.
#define BATMAND_WILL_LOW           1
/// Willingness for forwarding packets from other nodes: medium.
#define BATMAND_WILL_DEFAULT       3
/// Willingness for forwarding packets from other nodes: high.
#define BATMAND_WILL_HIGH          6
/// Willingness for forwarding packets from other nodes: always.
#define BATMAND_WILL_ALWAYS        7


#define BATMAND_PORT_NUMBER      4305

#define AGGREGATED_FRAMES          1
//#undef AGGREGATED_FRAMES

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("BatmanProtocol");

namespace batmand {

/********** BATMAND class **********/

NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

TypeId
RoutingProtocol::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::batmand::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .SetGroupName ("Batmand")
    .AddConstructor<RoutingProtocol> ()
    .AddAttribute ("OGMInterval", "OGM messages emission interval.",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&RoutingProtocol::m_ogmInterval),
                   MakeTimeChecker ())
    .AddAttribute ("Aggregation", "OGM messages aggregation allowed.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RoutingProtocol::m_aggregation),
                   MakeBooleanChecker ())
    .AddTraceSource ("Rx", "Receive BATMAND packet.",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_rxPacketTrace),
                     "ns3::batmand::RoutingProtocol::PacketTxRxTracedCallback")
    .AddTraceSource ("Tx", "Send BATMAND packet.",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_txPacketTrace),
                     "ns3::batmand::RoutingProtocol::PacketTxRxTracedCallback")
    .AddTraceSource ("RoutingTableChanged", "The BATMAND routing table has changed.",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_routingTableChanged),
                     "ns3::batmand::RoutingProtocol::TableChangeTracedCallback")
    .AddTraceSource ("TxPkt", "An aggregated OGM packet is sent",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_txTrace),
                     "ns3::batmand::RoutingProtocol::TracedCallback")
  ;
  return tid;
}


RoutingProtocol::RoutingProtocol () : 
  m_routingTableAssociation (0),
  m_packetSequenceNumber(0),
  m_ipv4 (0),
  m_ogmTimer (Timer::CANCEL_ON_DESTROY),
  m_queuedMessagesTimer (Timer::CANCEL_ON_DESTROY),
  m_numberHnalocal(0)
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
  m_hnaRoutingTable = Create<Ipv4StaticRouting> ();

  m_purgeTimeout = MilliSeconds( BATMAN_PURGE_TIMEOUT );

  m_aggregation = true;
//  m_ogmInterval = Time( "2s" );
  m_hopPenalty = TQ_HOP_PENALTY;

  /* "-r" is the command line switch for the routing class,
   * 0 set no default route
   * 1 use fast internet connection
   * 2 use stable internet connection
   * 3 use use best statistic (olsr style)
   * this option is used to set the routing behaviour
   */
  m_routingClass = 0;

  m_currGateway = NULL;
  m_prefGateway.Set( (uint32_t)0 );
  m_hnaGlobalHash = NULL;
  
  m_forwardingSet.clear();
  
  hna_init();

  OGMHeader h1;
  OGMHeader H2 = h1;
}

RoutingProtocol::~RoutingProtocol ()
{
}

void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);
  NS_LOG_DEBUG ("Created Batman::RoutingProtocol");
  m_ogmTimer.SetFunction (&RoutingProtocol::OgmTimerExpire, this);
  m_queuedMessagesTimer.SetFunction (&RoutingProtocol::SendQueuedMessages, this);

  m_packetSequenceNumber = BATMAND_MAX_SEQ_NUM;
  m_ipv4 = ipv4;
  m_hnaRoutingTable->SetIpv4 (ipv4);
}

void RoutingProtocol::DoDispose ()
{
  m_ipv4 = 0;
  m_hnaRoutingTable = 0;
  m_routingTableAssociation = 0;

  for (std::map< Ptr<Socket>, Ipv4InterfaceAddress >::iterator iter = m_socketAddresses.begin ();
       iter != m_socketAddresses.end (); iter++)
  {
      iter->first->Close ();
  }
  
  m_socketAddresses.clear ();
  //m_outfile.close();
  
  Ipv4RoutingProtocol::DoDispose ();
}

void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit ) const
{
  std::ostream* os = stream->GetStream ();

  *os << "Node: " << m_ipv4->GetObject<Node> ()->GetId ()
      << ", Time: " << Now ().As (Time::S)
      << ", Local time: " << GetObject<Node> ()->GetLocalTime ().As (Time::S)
      << ", BATMAN Routing table" << std::endl;

  *os << "Destination\t\tNextHop\t\tInterface\tDistance\n";

  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin ();
       iter != m_table.end (); iter++)
    {
      *os << iter->first << "\t\t";
      *os << iter->second.nextAddr << "\t\t";
      if (Names::FindName (m_ipv4->GetNetDevice (iter->second.interface)) != "")
        {
          *os << Names::FindName (m_ipv4->GetNetDevice (iter->second.interface)) << "\t\t";
        }
      else
        {
          *os << iter->second.interface << "\t\t";
        }
      *os << iter->second.distance << "\t";
      *os << "\n";
    }

  // Also print the HNA routing table
  if (m_hnaRoutingTable->GetNRoutes () > 0)
    {
      *os << " HNA Routing Table: ";
      m_hnaRoutingTable->PrintRoutingTable (stream);
    }
  else
    {
      *os << " HNA Routing Table: empty" << std::endl;
    }
}

void RoutingProtocol::DoInitialize ()
{
  if ( m_mainAddress == Ipv4Address() ) 
  {
    Ipv4Address loopback ("127.0.0.1");
    for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
    {
      // Use primary address, if multiple
      Ipv4Address addr = m_ipv4->GetAddress(i, 0).GetLocal();
      if (addr != loopback)
      {
        m_mainAddress = addr;
        break;
      }
    }
    NS_ASSERT (m_mainAddress != Ipv4Address ());
  }
  
  uint32_t addr = m_mainAddress.Get();
  
  std::string fname = "batman_node_" + ToString((unsigned int)(addr & 0x00FF)) + ".log";
  //m_outfile.open(fname.c_str());


//  NS_LOG_DEBUG ("Starting BATMAND on node " << m_mainAddress <<
//                " having " << m_ipv4->GetNInterfaces() << " devices");

  Ipv4Address loopback ("127.0.0.1");

  bool canRunBatman = false;
  for (uint8_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
  {
    Ipv4Address addr = m_ipv4->GetAddress (i, 0).GetLocal ();
    if (addr == loopback) {
//      NS_LOG_DEBUG("IF " << (int)i << " is a loopback");
      continue;
    }

    if (addr != m_mainAddress)
    {
//      NS_LOG_DEBUG("IF " << (int)i << " address " << addr << " NOT main address");
//      NS_ASSERT ( GetMainAddress (addr) == m_mainAddress );
    }
//    else {
//      NS_LOG_DEBUG("IF " << (int)i << " address " << addr << " is main address");
//    }

    // Create never expiring interface association tuple entries for our
    // own network interfaces, so that GetMainAddress () works to
    // translate the node's own interface addresses into the main address.
    batman_if *iface = new batman_if;
    iface->addr = addr;
    iface->netaddr = m_mainAddress;
    iface->if_num = i;
    iface->out.SetVersion( COMPAT_VERSION );
    iface->out.SetFlags( 0x00 );
//    iface->out.ttl = (i > 0 ? 2 : TTL);
    iface->out.SetTtl( TTL );
    iface->out.SetGWFlags( 0 );   // (batman_if->if_num > 0 ? 0 : gateway_class)
    iface->out.SetPacketSequenceNumber( 65535 );
    iface->out.SetGatewayPortNumber( GW_PORT );
    iface->out.SetOriginatorAddress( m_mainAddress );
//    iface->out.prev_sender = 0;
    iface->out.SetForwarderAddress( m_mainAddress );
    iface->out.SetTQvalue( TQ_MAX_VALUE );
    iface->out.ClearHnaEntries();
    AddBatmanIface ( iface );

    // Skip this interface as BATMAN was not enabled on it
    if ( m_interfaceExclusions.find(i) != m_interfaceExclusions.end() ) {
      NS_LOG_DEBUG("IF " << i << " in exclusion list");
      continue;
    }

    // Create a socket to listen only on this interface
    Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),
				    UdpSocketFactory::GetTypeId ());
    socket->SetAllowBroadcast (true);
    InetSocketAddress inetAddr (m_ipv4->GetAddress (i, 0).GetLocal (), BATMAND_PORT_NUMBER);
    socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvBatmanOgm,  this));
    
    if (socket->Bind (inetAddr))
      NS_FATAL_ERROR ("Failed to bind() BATMAND socket");

    socket->BindToNetDevice (m_ipv4->GetNetDevice (i));
    m_socketAddresses[socket] = m_ipv4->GetAddress(i, 0);

//    NS_LOG_DEBUG("\tIF num " << i << "\n\tSckt " << socket << "\n\tAddr " << m_socketAddresses[socket] );
    canRunBatman = true;
  }

  if (canRunBatman)
  {
//    m_ogmInterval = Time( "1s" );
    m_ogmTimer.Schedule ( Time( "0.1s" ) );
//    OgmTimerExpire ();
//    NS_LOG_DEBUG ("BATMAND on node " << m_mainAddress << " started");
//    m_ogmInterval = Time( "2s" );
  }
}

void RoutingProtocol::SetMainInterface (uint32_t interface)
{
  m_mainAddress = m_ipv4->GetAddress (interface, 0).GetLocal ();
}

void RoutingProtocol::SetInterfaceExclusions (std::set<uint32_t> exceptions)
{
  m_interfaceExclusions = exceptions;
}

//
// \brief Processes an incoming %BATMAND packet following the
//        https://tools.ietf.org/html/draft-openmesh-b-a-t-m-a-n-00 specification.
void
RoutingProtocol::RecvBatmanOgm (Ptr<Socket> socket)
{
  std::ostringstream os;

  bool is_my_addr = false;
  bool is_my_orig = false;
  bool is_broadcast = false;
  bool is_my_oldorig = false;
  bool is_duplicate = false;
  bool is_bidirectional = false;
//  bool update_origin_called = false;
//  bool was_duplicate = false;
//  bool was_bidirectional = false;
  
  unsigned char *hna_recv_buff = NULL;
  int16_t hna_buff_len = 0;

  Time curr_time = Simulator::Now ();
  
  debug_timeout = Simulator::Now ();
  vis_timeout = Simulator::Now ();
  
  uint8_t ifnum = 0;
  struct orig_node *origNeighNode = NULL, *origNode = NULL;

  Ptr<Packet> receivedPacket;
  Address sourceAddress;
//  Ipv4Address srcAddr;
  
  receivedPacket = socket->RecvFrom (sourceAddress);
  
//  srcAddr = Ipv4Address::ConvertFrom( sourceAddress );

  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
  Ipv4Address senderIfaceAddr = inetSourceAddr.GetIpv4 ();
  Ipv4Address receiverIfaceAddr = m_socketAddresses[socket].GetLocal ();
  NS_ASSERT (receiverIfaceAddr != Ipv4Address ());
  NS_LOG_DEBUG ("BATMAND node " << m_mainAddress << " received a BATMAND packet from "
                << senderIfaceAddr << " to " << receiverIfaceAddr <<
		" packet length " << receivedPacket->GetSize() << " src " << inetSourceAddr );
  //m_outfile << "BATMAND node " << m_mainAddress << " received a BATMAND packet from "
  //          << senderIfaceAddr << " to " << receiverIfaceAddr
  //		        << " packet length " << receivedPacket->GetSize() << " src " << inetSourceAddr << "\n";
  
  // All routing messages are sent from and to port RT_PORT,
  // so we check it.
  NS_ASSERT (inetSourceAddr.GetPort () == BATMAND_PORT_NUMBER);

  Ptr<Packet> packet = receivedPacket;

  packet->Print(os);
  NS_LOG_DEBUG("\tmsg received PKT: \n" << os.str() );
  //m_outfile << "\tmsg received PKT: \n" << os.str() << "\n";
  os.clear();
  os.seekp(0);
    
  OGMHeader batmanOgmHeader;
  
  MessageList rcvMsgs;
  
  while ( packet->GetSize() > 0 ) {
     packet->RemoveHeader (batmanOgmHeader);
     
     if ( batmanOgmHeader.GetOriginatorAddress() == receiverIfaceAddr ) 
        rcvMsgs.insert( rcvMsgs.begin(), batmanOgmHeader );
     else
        rcvMsgs.push_back( batmanOgmHeader );
  }
  
     
  // This is where the WHILE LOOP should start
  while ( rcvMsgs.size() > 0 )
  {
    is_my_addr = false;
    is_my_orig = false;
    is_broadcast = false;
    is_my_oldorig = false;
    is_duplicate = false;
    is_bidirectional = false;
    hna_buff_len = 0;
  
    std::ostringstream loc_os, nxt_os;
//    packet->RemoveHeader (batmanOgmHeader);
    batmanOgmHeader = rcvMsgs.front();
    rcvMsgs.erase( rcvMsgs.begin() );
    
    NS_ASSERT (batmanOgmHeader.GetPacketLength () >= batmanOgmHeader.GetSerializedSize ());

    batmanOgmHeader.Print (loc_os);
    NS_LOG_DEBUG( "[ Reading RX OGM Header: " << Simulator::Now().GetSeconds() << " ] \n" << loc_os.str () );
    //m_outfile << "[ Reading RX OGM Header: " << Simulator::Now().GetSeconds() << " ] \n" << loc_os.str () << "\n";
//    NS_LOG_DEBUG( "   Bytes remaining in packet: " << packet->GetSize() );
    
    hna_recv_buff = batmanOgmHeader.SerializeHnaList( hna_buff_len );

    /*
     * Perform per-interface evaluation as in BATMAN 0.3.2 source code
     */
    Ptr<Node> node = m_ipv4->GetObject<Node> ();
    //uint32_t devmax = node->GetNDevices ();

    Ipv4Address loopback ("127.0.0.1");
    for (uint32_t device = 0; device < m_ipv4->GetNInterfaces (); device++)
    {
      // Use primary address, if multiple
      Ipv4Address deviceAddr = m_ipv4->GetAddress(device, 0).GetLocal();
      if (deviceAddr == loopback)
        continue;

//    NS_LOG_DEBUG (" max num devices " << devmax );
//    for ( uint32_t device = 0; device < devmax; ++device ) {
      Ptr<NetDevice> dev = node->GetDevice(device);
//      NS_LOG_DEBUG ("  NetDevice: " << dev << "\n  address: " << dev->GetAddress() );
//      InetSocketAddress inetDevAddr = InetSocketAddress::ConvertFrom( dev->GetAddress () );
//      NS_LOG_DEBUG ( "  Ipv4Addr: " << deviceAddr );
      //Ipv4Address deviceAddr = inetDevAddr.GetIpv4 ();
      //Ipv4Address deviceAddr = Ipv4Address::ConvertFrom( dev->GetAddress () );
      
      if ( senderIfaceAddr == deviceAddr ) is_my_addr = true;
      if ( batmanOgmHeader.GetOriginatorAddress () == deviceAddr ) is_my_orig = true;
      
      // Check to see that the interface supports broadcast before we run the GetBroadcast() method
      if ( dev->IsBroadcast() && ( senderIfaceAddr == dev->GetBroadcast () ) ) is_broadcast = true;
      if ( batmanOgmHeader.GetForwarderAddress () == deviceAddr ) is_my_oldorig = true;
    }

    // Find the batman_if structure for the given incoming device
    batman_if *iface = FindBatmanIf( receiverIfaceAddr );
    if ( iface == NULL )
      NS_LOG_DEBUG ("iface not in list\n");
//    else
//      NS_LOG_DEBUG ("Iface " << iface << " num " << (int)iface->if_num << " received this packet ");

    ifnum = iface->if_num;

    // If the source is me - I received my own packet ? - so drop
    // guess it could happen on a broadcast medium like CSMA wired
    if ( is_my_addr )
      return;

    // If the source address is a bcast addr - it's not possible so drop
    if ( is_broadcast )
      return;

    // If the Previous Sender is my address, this is my old OGM forwarded by another node
    if ( is_my_orig ) {
      NS_LOG_DEBUG( "\t  *** OGM from me " << batmanOgmHeader.GetOriginatorAddress() << 
                    " fwd'd by node " << senderIfaceAddr );
      //m_outfile << "\t  *** OGM from me " << batmanOgmHeader.GetOriginatorAddress()
      //          << " fwd'd by node " << senderIfaceAddr << "\n";
      
      // Get or create an ORIG node entry for the given source
      origNeighNode = GetOrCreateOrigNode( senderIfaceAddr, ifnum );

      // We had to have received the OGM form a station via one of our Batman enabled
      // interfaces, or we received an unknown/invalid OGM
      if ( iface ) {
        if ( ( batmanOgmHeader.HasDirectLinkFlag() ) &&
             ( receiverIfaceAddr == batmanOgmHeader.GetOriginatorAddress () ) &&
             ( batmanOgmHeader.GetPacketSequenceNumber() - iface->out.GetPacketSequenceNumber() + 1 == 0 ) )
//             ( batmanOgmHeader.GetPacketSequenceNumber() - iface->out.GetPacketSequenceNumber() + 2 == 0 ) )
        {
          MarkSequenceBit( origNeighNode->bcast_own[ifnum], 0);
          origNeighNode->bcast_own_sum[ifnum] = Uint64BitCount( origNeighNode->bcast_own[ifnum] );
          NS_LOG_DEBUG ("count own bcast (is_my_orig): now = " << (int)(origNeighNode->bcast_own_sum[ifnum]) );
          //m_outfile << "count own bcast (is_my_orig): now = " << (int)(origNeighNode->bcast_own_sum[ifnum]) << "\n";
          // We want to recalc the update origin for ourself if we already tried 
          // to update the sender, but hadn't accounted for this returned OGM
//          if ( update_origin_called || !was_bidirectional ) {
  //           struct orig_node *sender = GetOrCreateOrigNode( senderIfaceAddr, ifnum );
  //           int16_t bufLen = 0;
  //           NS_LOG_DEBUG(" bufLen " << bufLen );

  //           unsigned char *hnaBuff = sender->in.SerializeHnaList( bufLen );
             
  //           UpdateOrig( sender, sender->in, senderIfaceAddr, ifnum, hnaBuff, bufLen, was_duplicate, curr_time);
//          }
        }
        else
        {
          NS_LOG_DEBUG ("\tDirLink " << batmanOgmHeader.HasDirectLinkFlag() <<
                        "\n\t  (rcvaddr == orig) " << ( receiverIfaceAddr == batmanOgmHeader.GetOriginatorAddress () ) <<
                        "\n\t  OGM seq " << batmanOgmHeader.GetPacketSequenceNumber() << 
                        " iface->out seq # " << iface->out.GetPacketSequenceNumber() );
          //m_outfile << "\tDirLink " << batmanOgmHeader.HasDirectLinkFlag() <<
          //"\n\t  (rcvaddr == orig) " << ( receiverIfaceAddr == batmanOgmHeader.GetOriginatorAddress () ) <<
          //"\n\t  OGM seq " << batmanOgmHeader.GetPacketSequenceNumber() <<
          //" iface->out seq # " << iface->out.GetPacketSequenceNumber() << "\n";
        }
      }
      else
        NS_LOG_DEBUG ("receive iface not found in list\n");

      NS_LOG_DEBUG ("Drop packet " << receivedPacket << ": orig pkt from myself (via neighbor " << senderIfaceAddr << ") \n");
      //m_outfile << "Drop packet " << receivedPacket << ": orig pkt from myself (via neighbor " << senderIfaceAddr << ") \n";
#ifdef  AGGREGATED_FRAMES
      continue;
#else
      return;
#endif
    }

    if (batmanOgmHeader.GetTQvalue() == 0) {
      CountRealPackets(batmanOgmHeader, senderIfaceAddr, ifnum);

      NS_LOG_DEBUG("Drop packet: originator packet with tq is 0 \n");
      //m_outfile << "Drop packet: originator packet with tq is 0 \n";
      return;
    }
    else
    {
      NS_LOG_DEBUG("  packet TQ val: " << (int)batmanOgmHeader.GetTQvalue() );
      //m_outfile << "  packet TQ val: " << (int)batmanOgmHeader.GetTQvalue() << "\n";
    }
    
    if (is_my_oldorig) {
      NS_LOG_DEBUG("Drop packet: ignoring all rebroadcast echos (sender: " << senderIfaceAddr << ") \n");
      //m_outfile << "Drop packet: ignoring all rebroadcast echos (sender: " << senderIfaceAddr << ") \n";
      return;
    }

    is_duplicate = CountRealPackets(batmanOgmHeader, senderIfaceAddr, ifnum);

    origNode =  GetOrCreateOrigNode( batmanOgmHeader.GetOriginatorAddress(), ifnum );

    /* if sender is a direct neighbor the sender ip equals originator ip */
    NS_LOG_DEBUG("OGM orig " << batmanOgmHeader.GetOriginatorAddress() << ((batmanOgmHeader.GetOriginatorAddress() == senderIfaceAddr) ? " is" : " is not") << " equal to sender " << senderIfaceAddr );
    //m_outfile << "OGM orig " << batmanOgmHeader.GetOriginatorAddress() << ((batmanOgmHeader.GetOriginatorAddress() == senderIfaceAddr) ? " is" : " is not") << " equal to sender " << senderIfaceAddr << "\n";
    
    origNeighNode = (batmanOgmHeader.GetOriginatorAddress() == senderIfaceAddr) ? origNode : GetOrCreateOrigNode(senderIfaceAddr, ifnum);

    /*
     * I put this in to store the last received OGM from the sender
     * I will use it later to redo the update origin if I receive my 
     * OGM from this origin later in the same packet
     */
    if ( origNode == origNeighNode )
       origNode->in = batmanOgmHeader;
       
    /* drop packet if sender is not a direct neighbor and if we no route towards it */
    if ((batmanOgmHeader.GetOriginatorAddress() != senderIfaceAddr) && (origNeighNode->router == NULL)) {
      NS_LOG_DEBUG("Drop packet: OGM via unknown neighbor! \n");
      //m_outfile << "Drop packet: OGM via unknown neighbor! \n";
      return;
    }

    is_bidirectional = isBidirectionalNeigh( origNode, origNeighNode, batmanOgmHeader, curr_time.GetInteger() , ifnum);
    
//    if ( origNode->orig == senderIfaceAddr )
//       was_bidirectional = is_bidirectional;

//    batmanOgmHeader.Print (nxt_os);
//    NS_LOG_DEBUG( "[ Header Bidir Check: ] \n" << nxt_os.str () );

    NS_LOG_DEBUG(" is bidir " << (int)is_bidirectional << " is dup " << (int)is_duplicate << " lst_real seq# " <<
       (int)origNode->last_real_seqno << " OGM seq # " << (int)batmanOgmHeader.GetPacketSequenceNumber() << " lst TTL " <<
       (int)origNode->last_ttl << " OGM TTL " << (int)batmanOgmHeader.GetTtl() );
    //m_outfile << " is bidir " << (int)is_bidirectional << " is dup " << (int)is_duplicate << " lst_real seq# " <<
    //(int)origNode->last_real_seqno << " OGM seq # " << (int)batmanOgmHeader.GetPacketSequenceNumber() << " lst TTL " <<
    //(int)origNode->last_ttl << " OGM TTL " << (int)batmanOgmHeader.GetTtl() << "\n";

    /* update ranking if it is not a duplicate or has the same seqno and similar ttl as the non-duplicate */
    if ( (is_bidirectional) &&
         ( (!is_duplicate) ||
	         ( (origNode->last_real_seqno == batmanOgmHeader.GetPacketSequenceNumber() ) &&
	           (origNode->last_ttl - 3 <= batmanOgmHeader.GetTtl() ) ) ) ) 
    {
      NS_LOG_DEBUG( " Calling Update Origin" );
      //m_outfile << " Calling Update Origin" << "\n";
//      update_origin_called = true;
      //was_duplicate = is_duplicate;
      UpdateOrig( origNode, batmanOgmHeader, senderIfaceAddr, ifnum, hna_recv_buff, hna_buff_len, is_duplicate, curr_time);
    }
    else {
       NS_LOG_DEBUG(" Skip update_origin ");
      //m_outfile << " Skip update_origin " << "\n";
    }

    /* is single hop (direct) neighbour */
    if (batmanOgmHeader.GetOriginatorAddress() == senderIfaceAddr) {

      /* mark direct link on incoming interface */
      NS_LOG_DEBUG("Forward packet: rebroadcast neighbour packet with direct link flag");
      //m_outfile << "Forward packet: rebroadcast neighbour packet with direct link flag" << "\n";
      NS_LOG_DEBUG("\t OGM origin " << origNode->orig << " tx'ing node " << senderIfaceAddr << "\n");
      //m_outfile << "\t OGM origin " << origNode->orig << " tx'ing node " << senderIfaceAddr << "\n";
      ScheduleForwardPacket( origNode, batmanOgmHeader, senderIfaceAddr, 1, hna_buff_len, ifnum, curr_time);
#ifdef  AGGREGATED_FRAMES
      continue;
#else
      return;
#endif
    }

    /* multihop originator */
    if (!is_bidirectional) {
      NS_LOG_DEBUG("Drop packet: not received via bidirectional link\n");
      //m_outfile << "Drop packet: not received via bidirectional link\n";
      return;
    }

    if (is_duplicate) {
      NS_LOG_DEBUG("Drop packet: duplicate packet received\n");
      //m_outfile << "Drop packet: duplicate packet received\n";
      return;
    }

    NS_LOG_DEBUG("Forward packet: rebroadcast originator packet INDIRECT\n");
    //m_outfile << "Forward packet: rebroadcast originator packet INDIRECT\n";
    NS_LOG_DEBUG("\t OGM origin " << origNode->orig << " tx'ing node " << senderIfaceAddr << "\n");
    //m_outfile << "\t OGM origin " << origNode->orig << " tx'ing node " << senderIfaceAddr << "\n";
    ScheduleForwardPacket( origNode, batmanOgmHeader, senderIfaceAddr, 0, hna_buff_len, ifnum, curr_time);

    hna_buff_len = 0;
    
    if ( hna_recv_buff != NULL )
      delete hna_recv_buff;
    
    hna_recv_buff = NULL;
    
//    NS_LOG_DEBUG( "BYTE REMAINING " << (int)packet->GetSize() );
  }

  /*
   * This is the final step after performing a RCV operation.. scheduling/sending packets
   * We need a loop above to separate forwarded OGMs from the first OGM in the received packet
   * then this section makes sense
   */
  NS_LOG_DEBUG("   now send outstanding pkts post RX @ " << curr_time );
  //m_outfile << "   now send outstanding pkts post RX @ " << curr_time << "\n";
  send_outstanding_packets( curr_time );

  if ((int)(curr_time.GetInteger() - (debug_timeout.GetInteger() + 1000)) > 0) {

    debug_timeout = curr_time;
    purge_orig( curr_time );
    debug_orig();
    // Don't need to check for interface activity in NS-3 for now
//    check_inactive_interfaces();

    if ( ( m_routingClass != 0 ) && ( m_currGateway == NULL ) ) {
      NS_LOG_DEBUG("  Choose gateway" );
      //m_outfile << "  Choose gateway" << "\n";
      ChooseGateway();
    }

    // Not supporting VIS in NS-3 for now
//    if ((vis_if.sock) && ((int)(curr_time - (vis_timeout + 10000)) > 0)) {
//      vis_timeout = curr_time;
//      send_vis_packet();
//    }

    // Not performing HNA tasks in NS-3 
//    hna_local_task_exec();
  }
  
  // Recalculate the routing table after receiving a new OGM - based on OLSR behavior
  RoutingTableComputation ();

  NS_LOG_DEBUG("recv processing complete\n\n");
  //m_outfile << "recv processing complete\n\n";
}


int RoutingProtocol::isBidirectionalNeigh(struct orig_node *origNode,
                                          struct orig_node *origNeighNode,
                                          OGMHeader &in,
                                          int64_t recv_time,
                                          uint8_t ifnum)
{
   struct neigh_node *neigh_node = NULL, *tmpNeighNode = NULL;;
   uint8_t total_count = 0;

   if ( origNode == origNeighNode ) {
      NS_LOG_DEBUG("  BiDir orig Node " << origNode->orig << " is equal to " << origNeighNode->orig );
     //m_outfile << "  BiDir orig Node " << origNode->orig << " is equal to " << origNeighNode->orig << "\n";
     
      for ( unsigned int i = 0; i < origNode->neigh_list.size(); ++i ) {
        tmpNeighNode = &(origNode->neigh_list[i]);

        if ( ( tmpNeighNode->addr == origNeighNode->orig ) && ( tmpNeighNode->ifnum == ifnum ) )
           neigh_node = tmpNeighNode;
      }

      if ( neigh_node == NULL ) {
         NS_LOG_DEBUG("\tneigh node NULL - creating");
        //m_outfile << "\tneigh node NULL - creating" << "\n";
         neigh_node = CreateNeighbor(origNode, origNeighNode, origNeighNode->orig, ifnum);
      }
      else
      {
        NS_LOG_DEBUG("\tfound neigh " << neigh_node->addr );
        //m_outfile << "\tfound neigh " << neigh_node->addr << "\n";
      }

      neigh_node->last_valid = recv_time;
   }
   else {
      NS_LOG_DEBUG("  BiDir orig Node " << origNode->orig << " is " << ((1 + TTL) - (int)in.GetTtl())<< " hop neighbor " << origNeighNode->orig );
     //m_outfile << "  BiDir orig Node " << origNode->orig << " is " << ((1 + TTL) - (int)in.GetTtl())<< " hop neighbor " << origNeighNode->orig << "\n";
      /* find packet count of corresponding one hop neighbor */
      for ( unsigned int i = 0; i < origNeighNode->neigh_list.size(); ++i ) {
        tmpNeighNode = &(origNeighNode->neigh_list[i]);

        if ( ( tmpNeighNode->addr == origNeighNode->orig ) && ( tmpNeighNode->ifnum == ifnum ) )
          neigh_node = tmpNeighNode;
      }

      if ( neigh_node == NULL ) {
         NS_LOG_DEBUG("\tneigh node NULL - creating");
        //m_outfile << "\tneigh node NULL - creating" << "\n";
         neigh_node = CreateNeighbor( origNeighNode, origNeighNode, origNeighNode->orig, ifnum);
      }
      else
      {
        NS_LOG_DEBUG("\tfound neigh " << neigh_node->addr );
        //m_outfile << "\tfound neigh " << neigh_node->addr << "\n";
      }
   }

   origNode->last_valid = recv_time;

   /* pay attention to not get a value bigger than 100% */
   NS_LOG_DEBUG("  orig bcast own sum: " << (int)origNeighNode->bcast_own_sum[ifnum] << " on if " << (int)ifnum <<
                "\n\t\tneigh real pkt cnt: " << (int)neigh_node->real_packet_count << "\n\t\torig neigh bcast own sum:" << (int)origNeighNode->bcast_own_sum[ifnum] );
  //m_outfile << "  orig bcast own sum: " << (int)origNeighNode->bcast_own_sum[ifnum] << " on if " << (int)ifnum << "\n\t\tneigh real pkt cnt: " << (int)neigh_node->real_packet_count << "\n\t\torig neigh bcast own sum:" << (int)origNeighNode->bcast_own_sum[ifnum] << "\n";
   total_count = ( origNeighNode->bcast_own_sum[ifnum] > neigh_node->real_packet_count ) ?
                   neigh_node->real_packet_count : origNeighNode->bcast_own_sum[ifnum];

   /* if we have too few packets (too less data) we set tq_own to zero */
   /* if we receive too few packets it is not considered bidirectional */
   if ( ( total_count < TQ_LOCAL_BIDRECT_SEND_MINIMUM ) ||
        ( neigh_node->real_packet_count < TQ_LOCAL_BIDRECT_RECV_MINIMUM ) ) {
      NS_LOG_DEBUG("\t  setting orgNeig TQ_OWN to 0");
     //m_outfile << "\t  setting orgNeig TQ_OWN to 0" << "\n";
      origNeighNode->tq_own = 0;
   }
   else {
      /* neigh_node->real_packet_count is never zero as we only purge old information when getting new information */
      origNeighNode->tq_own = (TQ_MAX_VALUE * total_count) / neigh_node->real_packet_count;
      NS_LOG_DEBUG("\t  setting orgNeig TQ_OWN to " << (int)origNeighNode->tq_own << " as tot_cnt " << (int)total_count << " and real pkt cnt " << (int)neigh_node->real_packet_count );
     //m_outfile << "\t  setting orgNeig TQ_OWN to " << (int)origNeighNode->tq_own << " as tot_cnt " << (int)total_count << " and real pkt cnt " << (int)neigh_node->real_packet_count << "\n";
   }

   NS_LOG_DEBUG(" local_win_size = " << local_win_size );
  //m_outfile << " local_win_size = " << local_win_size << "\n";
  
   double loc_cube = ((double)local_win_size) * ((double)local_win_size) * ((double)local_win_size);
   double win_min_cnt = (double)(local_win_size - neigh_node->real_packet_count);
   double win_cubed = win_min_cnt * win_min_cnt * win_min_cnt;
   
   NS_LOG_DEBUG(" loc_win ^ 3: " << loc_cube << "\t win  cnt: " << win_min_cnt << "\t (win - cnt)^3: " << win_cubed );
  //m_outfile << " loc_win ^ 3: " << loc_cube << "\t win  cnt: " << win_min_cnt << "\t (win - cnt)^3: " << win_cubed << "\n";
   /* 1 - ((1-x)** 3), normalized to TQ_MAX_VALUE */
   /* this does affect the nearly-symmetric links only a little,
    * but punishes asymmetric links more. */
   /* this will give a value between 0 and TQ_MAX_VALUE */
   origNeighNode->tq_asym_penalty = TQ_MAX_VALUE - (int)( ((double)TQ_MAX_VALUE) * (win_cubed / loc_cube ) );

   NS_LOG_DEBUG(" pkt TQ: " << (int)in.GetTQvalue() << "\t neigh TQ: " << (int)origNeighNode->tq_own << 
                "\t neigh penalty: " << origNeighNode->tq_asym_penalty );
  //m_outfile << " pkt TQ: " << (int)in.GetTQvalue() << "\t neigh TQ: " << (int)origNeighNode->tq_own <<
  //"\t neigh penalty: " << origNeighNode->tq_asym_penalty << "\n";
  
   int product = (in.GetTQvalue() * origNeighNode->tq_own * origNeighNode->tq_asym_penalty);
   int divisor = (TQ_MAX_VALUE * TQ_MAX_VALUE);
   NS_LOG_DEBUG(" product: " << product << "\t divisor: " << divisor );
  //m_outfile << " product: " << product << "\t divisor: " << divisor << "\n";

   // Account for initial connections
   if ( ( product < divisor ) && ( product > 0 ) )
      in.SetTQvalue(1);
   else
      in.SetTQvalue(product / divisor);
   
   NS_LOG_DEBUG("  set pkt TQ for fwd'ing to " << (int)in.GetTQvalue() );
  //m_outfile << "  set pkt TQ for fwd'ing to " << (int)in.GetTQvalue() << "\n";

   /*debug_output( 3, "bidirectional: orig = %-15s neigh = %-15s => own_bcast = %2i, real recv = %2i, local tq: %3i, asym_penalty: %3i, total tq: %3i \n",
   orig_str, neigh_str, total_count, neigh_node->real_packet_count, origNeighNode->tq_own, origNeighNode->tq_asym_penalty, in->tq );*/
   NS_LOG_DEBUG("bidirectional: orig = " << origNode->orig << " orig tq " << (int)origNode->tq_own << " neigh = " << 
                origNeighNode->orig << " => own_bcast = " << (int)total_count << ", real recv = " << 
                (int)(neigh_node->real_packet_count) << ", neigh tq: " << (int)(origNeighNode->tq_own) << 
                ", asym_penalty: " << (int)(origNeighNode->tq_asym_penalty) << ", rcv pkt tq: " << (int)(in.GetTQvalue()) );
  //m_outfile << "bidirectional: orig = " << origNode->orig << " orig tq " << (int)origNode->tq_own << " neigh = " <<
  //origNeighNode->orig << " => own_bcast = " << (int)total_count << ", real recv = " <<
  //(int)(neigh_node->real_packet_count) << ", neigh tq: " << (int)(origNeighNode->tq_own) <<
  //", asym_penalty: " << (int)(origNeighNode->tq_asym_penalty) << ", rcv pkt tq: " << (int)(in.GetTQvalue()) << "\n";

   /* if link has the minimum required transmission quality consider it bidirectional */
   return (in.GetTQvalue() >= TQ_TOTAL_BIDRECT_LIMIT);
}



void
RoutingProtocol::QueueMessage (const OGMHeader &message, Time delay)
{
//  std::ostringstream os;
  
  NS_LOG_FUNCTION (this);
  m_queuedMessages.push_back (message);
  
//  message.Print (os);
//  std::cout << "[ Q'd OGM Header: " << Simulator::Now() << " ] \n" << os.str () << std::endl;

  
  if (not m_queuedMessagesTimer.IsRunning ())
    {
      m_queuedMessagesTimer.SetDelay (delay);
      m_queuedMessagesTimer.Schedule ();
    }
}

void
RoutingProtocol::SendPacket (Ptr<Packet> packet,
                             const MessageList &containedMessages)
{
  NS_LOG_DEBUG ("BATMAND node " << m_mainAddress << " sending a BATMAND packet");
  //m_outfile << "BATMAND node " << m_mainAddress << " sending a BATMAND packet" << "\n";
  
//  NS_LOG_DEBUG( "PACKET SEND SIZE " << (int)packet->GetSize() );
  
  // Peek at the header so we can trace it
  OGMHeader header;
  packet->PeekHeader( header );
  
  // Trace it
  NS_LOG_DEBUG ("BATMAND SendPkt trace tx pkt (seq " << header.GetPacketSequenceNumber() << ")" );
  //m_outfile << "BATMAND SendPkt trace tx pkt (seq " << header.GetPacketSequenceNumber() << ")" << "\n";
  
  m_txPacketTrace (header, containedMessages);
  m_txTrace (packet);
  
  // Send it
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator i =
         m_socketAddresses.begin (); i != m_socketAddresses.end (); i++)
  {
//    NS_LOG_DEBUG ("\t SendPkt ifaddr " << i->second.GetLocal() );
    Ipv4Address bcast = i->second.GetLocal ().GetSubnetDirectedBroadcast (i->second.GetMask ());
//    NS_LOG_DEBUG ("\t SendPkt bcast " << bcast );
    NS_LOG_WARN("BATMAN: TX pkt " << packet->GetUid() << " at " << Simulator::Now() );
    //m_outfile << "BATMAN: TX pkt " << packet->GetUid() << " at " << Simulator::Now() << "\n";

    i->first->SendTo (packet, 0, InetSocketAddress (bcast, BATMAND_PORT_NUMBER));
  }
}

void
RoutingProtocol::SendQueuedMessages ()
{
//  NS_LOG_FUNCTION(this);

  NS_LOG_DEBUG( "Batman node " << m_mainAddress << ": SendQueuedMessages" << " at " << Simulator::Now () );
  //m_outfile << "Batman node " << m_mainAddress << ": SendQueuedMessages" << " at " << Simulator::Now () << "\n";
  send_outstanding_packets( Simulator::Now () );
}

void
RoutingProtocol::AddHostNetworkAssociation (Ipv4Address networkAddr, Ipv4Mask netmask)
{
  /*
  // Check if the (networkAddr, netmask) tuple already exist
  // in the list of local HNA associations
  const Associations &localHnaAssociations = GetAssociations ();
  for (Associations::const_iterator assocIterator = localHnaAssociations.begin ();
       assocIterator != localHnaAssociations.end (); assocIterator++)
    {
      Association const &localHnaAssoc = *assocIterator;
      if (localHnaAssoc.networkAddr == networkAddr && localHnaAssoc.netmask == netmask)
        {
          NS_LOG_INFO ("HNA association for network " << networkAddr << "/" << netmask << " already exists.");
          return;
        }
    }
  */
  // If the tuple does not already exist, add it to the list of local HNA associations.
  NS_LOG_INFO ("Adding HNA association for network " << networkAddr << "/" << netmask << ".");
  //m_outfile << "Adding HNA association for network " << networkAddr << "/" << netmask << "." << "\n";
  //InsertAssociation ( (Association) { networkAddr, netmask} );
}

void
RoutingProtocol::RemoveHostNetworkAssociation (Ipv4Address networkAddr, Ipv4Mask netmask)
{
  NS_LOG_INFO ("Removing HNA association for network " << networkAddr << "/" << netmask << ".");
  //m_outfile << "Removing HNA association for network " << networkAddr << "/" << netmask << "." << "\n";
  //EraseAssociation ( (Association) { networkAddr, netmask} );
}

void
RoutingProtocol::SetRoutingTableAssociation (Ptr<Ipv4StaticRouting> routingTable)
{
  // If a routing table has already been associated, remove
  // corresponding entries from the list of local HNA associations
  if (m_routingTableAssociation != 0)
    {
      NS_LOG_INFO ("Removing HNA entries coming from the old routing table association.");
      //m_outfile << "Removing HNA entries coming from the old routing table association." << "\n";
      for (uint32_t i = 0; i < m_routingTableAssociation->GetNRoutes (); i++)
        {
          Ipv4RoutingTableEntry route = m_routingTableAssociation->GetRoute (i);
          // If the outgoing interface for this route is a non-olsr interface
          if (UsesNonOlsrOutgoingInterface (route))
            {
              // remove the corresponding entry
              RemoveHostNetworkAssociation (route.GetDestNetwork (), route.GetDestNetworkMask ());
            }
        }
    }

  // Sets the routingTableAssociation to its new value
  m_routingTableAssociation = routingTable;

  // Iterate over entries of the associated routing table and
  // add the routes using non-olsr outgoing interfaces to the list
  // of local HNA associations
//  NS_LOG_DEBUG ("Nb local associations before adding some entries from"
//                " the associated routing table: " << GetAssociations ().size ());
  for (uint32_t i = 0; i < m_routingTableAssociation->GetNRoutes (); i++)
    {
      Ipv4RoutingTableEntry route = m_routingTableAssociation->GetRoute (i);
      Ipv4Address destNetworkAddress = route.GetDestNetwork ();
      Ipv4Mask destNetmask = route.GetDestNetworkMask ();

      // If the outgoing interface for this route is a non-olsr interface,
      if (UsesNonOlsrOutgoingInterface (route))
        {
          // Add this entry's network address and netmask to the list of local HNA entries
          AddHostNetworkAssociation (destNetworkAddress, destNetmask);
        }
    }
//  NS_LOG_DEBUG ("Nb local associations after having added some entries from "
//                "the associated routing table: " << GetAssociations ().size ());
}

bool
RoutingProtocol::UsesNonOlsrOutgoingInterface (const Ipv4RoutingTableEntry &route)
{
  std::set<uint32_t>::const_iterator ci = m_interfaceExclusions.find (route.GetInterface ());
  // The outgoing interface is a non-BATMAND interface if a match is found
  // before reaching the end of the list of excluded interfaces
  return ci != m_interfaceExclusions.end ();
}

uint16_t RoutingProtocol::GetPacketSequenceNumber ()
{
  NS_LOG_FUNCTION( this << " seq no " << m_packetSequenceNumber );
  //m_outfile << "GetPacketSequenceNumber seq no " << m_packetSequenceNumber << "\n";
  m_packetSequenceNumber = (m_packetSequenceNumber + 1) % (BATMAND_MAX_SEQ_NUM + 1);
  NS_LOG_DEBUG( "\t new seq no: " << m_packetSequenceNumber );
  //m_outfile << "\t new seq no: " << m_packetSequenceNumber << "\n";
  return m_packetSequenceNumber;
}

void
RoutingProtocol::OgmTimerExpire ()
{
  struct batman_if *batman_if;

  /*
   * Do this in lieu of the SendOGM() call typical of OLSR. Batman uses
   * the schedule_own_packet
   */
  BatmanIfList::iterator ifitr = m_batmanif.begin();
  while( ifitr != m_batmanif.end() ) {
    batman_if = *ifitr;
    NS_LOG_DEBUG ("\tOGM Timer kickoff for IF " << (int)(batman_if->if_num) << " at " << Simulator::Now () );
    //m_outfile << "\tOGM Timer kickoff for IF " << (int)(batman_if->if_num) << " at " << Simulator::Now () << "\n";
    schedule_own_packet( batman_if );
    ++ifitr;
  }

  NS_LOG_DEBUG("OgmTimerExpired - calling send_otstanding_pkts");
  //m_outfile << "OgmTimerExpired - calling send_otstanding_pkts" << "\n";
  send_outstanding_packets( Simulator::Now () );
  m_ogmTimer.Schedule (m_ogmInterval);
}

void
RoutingProtocol::Clear ()
{
  NS_LOG_FUNCTION_NOARGS ();
  m_table.clear ();
}

void
RoutingProtocol::RemoveEntry (Ipv4Address const &dest)
{
  RouteTableMap::iterator it = m_table.find( dest );
  
  if ( it != m_table.end() )
    m_table.erase ( it );
}


bool
RoutingProtocol::Lookup (Ipv4Address const &dest,
                         RoutingTableEntry &outEntry) const
{
  // Get the iterator at "dest" position
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator it = m_table.find (dest);
  
  // If there is no route to "dest", return NULL
  if (it == m_table.end ())
    {
      return false;
    }
  outEntry = it->second;
  return true;
}

bool
RoutingProtocol::FindSendEntry (RoutingTableEntry const &entry,
                                RoutingTableEntry &outEntry) const
{
  outEntry = entry;
  while (outEntry.destAddr != outEntry.nextAddr)
    {
      if (not Lookup (outEntry.nextAddr, outEntry))
        {
          return false;
        }
    }
  return true;
}

Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr)
{
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination () << " " << oif);
  //m_outfile << "RouteOutput " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination () << " " << oif << "\n";
  Ptr<Ipv4Route> rtentry;
  RoutingTableEntry entry1, entry2;
  bool found = false;

  NS_LOG_DEBUG("  Time is now: " << Simulator::Now() );
  //m_outfile << "  Time is now: " << Simulator::Now() << "\n";
  if (Lookup (header.GetDestination (), entry1) != 0)
  {
    bool foundSendEntry = FindSendEntry (entry1, entry2);
    if (!foundSendEntry)
      NS_FATAL_ERROR ("FindSendEntry failure");
    
    uint32_t interfaceIdx = entry2.interface;
    if (oif && m_ipv4->GetInterfaceForDevice (oif) != static_cast<int> (interfaceIdx))
    {
      // We do not attempt to perform a constrained routing search
      // if the caller specifies the oif; we just enforce that
      // that the found route matches the requested outbound interface
      NS_LOG_DEBUG ("Batman node " << m_mainAddress
		    << ": RouteOutput for dest=" << header.GetDestination ()
		    << " Route interface " << interfaceIdx
		    << " does not match requested output interface "
		    << m_ipv4->GetInterfaceForDevice (oif));
      //m_outfile << "Batman node " << m_mainAddress
      //  << ": RouteOutput for dest=" << header.GetDestination ()
      //  << " Route interface " << interfaceIdx
      //  << " does not match requested output interface "
      //  << m_ipv4->GetInterfaceForDevice (oif) << "\n";
      sockerr = Socket::ERROR_NOROUTETOHOST;
      return rtentry;
    }
    
    rtentry = Create<Ipv4Route> ();
    rtentry->SetDestination (header.GetDestination ());
    // the source address is the interface address that matches
    // the destination address (when multiple are present on the
    // outgoing interface, one is selected via scoping rules)
    NS_ASSERT (m_ipv4);
    uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
    NS_ASSERT (numOifAddresses > 0);
    Ipv4InterfaceAddress ifAddr;
    
    if (numOifAddresses == 1)
      ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
    else
    {
      /// \todo Implment IP aliasing and BATMAND
      NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and BATMAND");
    }
    
    rtentry->SetSource (ifAddr.GetLocal ());
    rtentry->SetGateway (entry2.nextAddr);
    rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
    sockerr = Socket::ERROR_NOTERROR;
    NS_LOG_DEBUG ("Batman node " << m_mainAddress
		  << ": RouteOutput for dest=" << header.GetDestination ()
		  << " --> nextHop=" << entry2.nextAddr
		  << " interface=" << entry2.interface);
    //m_outfile << "Batman node " << m_mainAddress
    //  << ": RouteOutput for dest=" << header.GetDestination ()
    //  << " --> nextHop=" << entry2.nextAddr
    //  << " interface=" << entry2.interface << "\n";
    
    NS_LOG_DEBUG ("Found route to " << rtentry->GetDestination () << " via nh " 
		  << rtentry->GetGateway () << " with source addr " 
		  << rtentry->GetSource () << " and output dev " 
		  << rtentry->GetOutputDevice ());
    //m_outfile << "Found route to " << rtentry->GetDestination () << " via nh "
    //  << rtentry->GetGateway () << " with source addr "
    //  << rtentry->GetSource () << " and output dev "
    //  << rtentry->GetOutputDevice () << "\n";
    found = true;
  }
  else
  {
    rtentry = m_hnaRoutingTable->RouteOutput (p, header, oif, sockerr);

    if (rtentry)
    {
      found = true;
      NS_LOG_DEBUG ("Found route to " << rtentry->GetDestination () << " via nh " 
		    << rtentry->GetGateway () << " with source addr " 
		    << rtentry->GetSource () << " and output dev " 
		    << rtentry->GetOutputDevice ());
      //m_outfile << "Found route to " << rtentry->GetDestination () << " via nh "
      //  << rtentry->GetGateway () << " with source addr "
      //  << rtentry->GetSource () << " and output dev "
      //  << rtentry->GetOutputDevice () << "\n";
    }
  }

  if (!found)
  {
    NS_LOG_DEBUG ("Olsr node " << m_mainAddress
				<< ": RouteOutput for dest=" << header.GetDestination ()
				<< " No route to host");
    //m_outfile << "Olsr node " << m_mainAddress
    //		<< ": RouteOutput for dest=" << header.GetDestination ()
    //		<< " No route to host" << "\n";
    sockerr = Socket::ERROR_NOROUTETOHOST;
  }
  
  return rtentry;
}

bool RoutingProtocol::RouteInput  (Ptr<const Packet> p,
                                   const Ipv4Header &header, Ptr<const NetDevice> idev,
                                   UnicastForwardCallback ucb, MulticastForwardCallback mcb,
                                   LocalDeliverCallback lcb, ErrorCallback ecb)
{
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination ());

  Ipv4Address dst = header.GetDestination ();
  Ipv4Address origin = header.GetSource ();

  // Consume self-originated packets
  if ( IsMyOwnAddress(origin) == true )
      return true;

  // Local delivery
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  uint32_t iif = m_ipv4->GetInterfaceForDevice (idev);
  if ( m_ipv4->IsDestinationAddress(dst, iif) )
  {
    if (!lcb.IsNull ())
    {
      NS_LOG_LOGIC ("Local delivery to " << dst);
      //m_outfile << "Local delivery to " << dst << "\n";
      lcb (p, header, iif);
      return true;
    }
    else
    {
      // The local delivery callback is null.  This may be a multicast
      // or broadcast packet, so return false so that another
      // multicast routing protocol can handle it.  It should be possible
      // to extend this to explicitly check whether it is a unicast
      // packet, and invoke the error callback if so
      return false;
    }
  }

  // Forwarding
  Ptr<Ipv4Route> rtentry;
  RoutingTableEntry entry1, entry2;
  if ( Lookup(header.GetDestination (), entry1) )
  {
    bool foundSendEntry = FindSendEntry (entry1, entry2);
    if (!foundSendEntry)
        NS_FATAL_ERROR ("FindSendEntry failure");

    rtentry = Create<Ipv4Route> ();
    rtentry->SetDestination (header.GetDestination ());
    uint32_t interfaceIdx = entry2.interface;

    // the source address is the interface address that matches the destination address
    // (when multiple are present on the outgoing interface, one is selected via scoping rules)
    NS_ASSERT (m_ipv4);
    uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
    NS_ASSERT (numOifAddresses > 0);
    Ipv4InterfaceAddress ifAddr;
    if (numOifAddresses == 1)
        ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
    else
        /// \todo Implment IP aliasing and BATMAND
        NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and BATMAND");

    rtentry->SetSource (ifAddr.GetLocal ());
    rtentry->SetGateway (entry2.nextAddr);
    rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));

    NS_LOG_DEBUG ("Batman node " << m_mainAddress
                  << ": RouteInput for dest=" << header.GetDestination ()
                  << " --> nextHop=" << entry2.nextAddr
                  << " interface=" << entry2.interface);
    //m_outfile << "Batman node " << m_mainAddress
    //<< ": RouteInput for dest=" << header.GetDestination ()
    //<< " --> nextHop=" << entry2.nextAddr
    //<< " interface=" << entry2.interface << "\n";

    ucb (rtentry, p, header);
    return true;
  }
  else
  {
    if (m_hnaRoutingTable->RouteInput (p, header, idev, ucb, mcb, lcb, ecb))
      return true;
    else
    {

#ifdef NS3_LOG_ENABLE
      NS_LOG_DEBUG ("Batman node " << m_mainAddress
                                 << ": RouteInput for dest=" << header.GetDestination ()
                                 << " --> NOT FOUND; ** Dumping routing table...");
      //m_outfile << "Batman node " << m_mainAddress
      //<< ": RouteInput for dest=" << header.GetDestination ()
      //<< " --> NOT FOUND; ** Dumping routing table..." << "\n";
      
      for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin ();
           iter != m_table.end (); iter++)
      {
        NS_LOG_DEBUG ("dest=" << iter->first << " --> next=" << iter->second.nextAddr
                              << " via interface " << iter->second.interface);
        //m_outfile << "dest=" << iter->first << " --> next=" << iter->second.nextAddr
        //<< " via interface " << iter->second.interface << "\n";
      }

      NS_LOG_DEBUG ("** Routing table dump end.");
      //m_outfile << "** Routing table dump end." << "\n";
#endif // NS3_LOG_ENABLE

      return false;
    }
  }
}

void
RoutingProtocol::NotifyInterfaceUp (uint32_t i)
{
}

void
RoutingProtocol::NotifyInterfaceDown (uint32_t i)
{
}

void
RoutingProtocol::NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address)
{
}

void
RoutingProtocol::NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address)
{
}


void RoutingProtocol::UpdateEntry (Ipv4Address const &dest,
                                   Ipv4Address const &next,
                                   Ipv4Address const &interfaceAddress,
                                   uint32_t distance)
{
  NS_LOG_FUNCTION (this << dest << next << interfaceAddress << distance << m_mainAddress);

  NS_ASSERT (distance > 0);
  NS_ASSERT (m_ipv4);

  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
  {
    for (uint32_t j = 0; j < m_ipv4->GetNAddresses (i); j++)
    {
      if (m_ipv4->GetAddress (i,j).GetLocal () == interfaceAddress)
      {
        UpdateEntry (dest, next, i, distance);
        break;
      }
    }
  }
}


void
RoutingProtocol::UpdateEntry (Ipv4Address const &dest,
                              Ipv4Address const &next,
                              uint32_t interface,
                              uint32_t distance)
{
  RouteTableMap::iterator it = m_table.find( dest );
  
  if ( it != m_table.end() ) {
    RoutingTableEntry &entry = m_table[dest];
   
    entry.destAddr = dest;
    entry.nextAddr = next;
    entry.interface = interface;
    entry.distance = distance;
   
    NS_LOG_DEBUG("  updated rt for " << m_table[dest].destAddr << " nh " << m_table[dest].nextAddr << " dist " << 
                 m_table[dest].distance << " now " << (double)(Simulator::Now().GetSeconds() / 1000000000.0) );
    //m_outfile << "  updated rt for " << m_table[dest].destAddr << " nh " << m_table[dest].nextAddr << " dist " <<
    //             m_table[dest].distance << " now " << (double)(Simulator::Now().GetSeconds() / 1000000000.0)  << "\n";
  }
}


void RoutingProtocol::AddEntry (Ipv4Address const &dest,
                                Ipv4Address const &next,
                                uint32_t interface,
                                uint32_t distance)
{
  NS_LOG_FUNCTION (this << dest << next << interface << distance << m_mainAddress);

  NS_ASSERT (distance > 0);

//  RouteTableMap::iterator itr = m_table.begin();
//  if ( ( itr = m_table.find( dest ) ) != m_table.end() ) {
//    NS_LOG_DEBUG("  entry for " << m_table[dest].destAddr << " old nh " << m_table[dest].nextAddr << " old dist " << 
//               m_table[dest].distance << " now " << (Simulator::Now() / 1000000000.0) );
//  }
  
  // Creates a new rt entry with specified values
  RoutingTableEntry &entry = m_table[dest];
  
  entry.destAddr = dest;
  entry.nextAddr = next;
  entry.interface = interface;
  entry.distance = distance;
  
  NS_LOG_DEBUG("  add rt for " << m_table[dest].destAddr << " nh " << m_table[dest].nextAddr << " dist " << 
               m_table[dest].distance << " now " << (double)(Simulator::Now().GetSeconds() / 1000000000.0) );
  //m_outfile << "  add rt for " << m_table[dest].destAddr << " nh " << m_table[dest].nextAddr << " dist " <<
  //                m_table[dest].distance << " now " << (double)(Simulator::Now().GetSeconds() / 1000000000.0) << "\n";

}

void RoutingProtocol::AddEntry (Ipv4Address const &dest,
                                Ipv4Address const &next,
                                Ipv4Address const &interfaceAddress,
                                uint32_t distance)
{
  NS_LOG_FUNCTION (this << dest << next << interfaceAddress << distance << m_mainAddress);

  NS_ASSERT (distance > 0);
  NS_ASSERT (m_ipv4);

  RoutingTableEntry entry;
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
  {
    for (uint32_t j = 0; j < m_ipv4->GetNAddresses (i); j++)
    {
      if (m_ipv4->GetAddress (i,j).GetLocal () == interfaceAddress)
      {
        AddEntry (dest, next, i, distance);
        return;
      }
    }
  }
  NS_ASSERT (false); // should not be reached
  AddEntry (dest, next, 0, distance);
}

std::vector<RoutingTableEntry>
RoutingProtocol::GetRoutingTableEntries () const
{
  std::vector<RoutingTableEntry> retval;
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin ();
       iter != m_table.end (); iter++)
    {
      retval.push_back (iter->second);
    }
  return retval;
}

void RoutingProtocol::RoutingTableComputation ()
{
  NS_LOG_DEBUG (Simulator::Now ().GetSeconds () << " s: Node " << m_mainAddress
                      << ": RoutingTableComputation begin...");
  //m_outfile << Simulator::Now ().GetSeconds () << " s: Node " << m_mainAddress
  //<< ": RoutingTableComputation begin..." << "\n";
 /*
  *
  // 1. All the entries from the routing table are removed.
  Clear();

  // 2. The new routing entries are added starting with the
  // symmetric neighbors (h=1) as the destination nodes.
  const NeighborSet &neighborSet = m_state.GetNeighbors ();
  for (NeighborSet::const_iterator it = neighborSet.begin (); it != neighborSet.end (); it++)
  {
    NeighborTuple const &nb_tuple = *it;
    NS_LOG_DEBUG ("Looking at neighbor tuple: " << nb_tuple);
    if (nb_tuple.status == NeighborTuple::STATUS_SYM)
    {
      bool nb_main_addr = false;
      const LinkTuple *lt = NULL;
      const LinkSet &linkSet = m_state.GetLinks ();
      for (LinkSet::const_iterator it2 = linkSet.begin (); it2 != linkSet.end (); it2++)
      {
        LinkTuple const &link_tuple = *it2;
        NS_LOG_DEBUG ("Looking at link tuple: " << link_tuple
                                  << (link_tuple.time >= Simulator::Now () ? "" : " (expired)"));
        if ((GetMainAddress (link_tuple.neighborIfaceAddr) == nb_tuple.neighborMainAddr) && link_tuple.time >= Simulator::Now ())
        {
          NS_LOG_LOGIC ("Link tuple matches neighbor " << nb_tuple.neighborMainAddr
                        << " => adding routing table entry to neighbor");

          lt = &link_tuple;
          AddEntry (link_tuple.neighborIfaceAddr, link_tuple.neighborIfaceAddr, link_tuple.localIfaceAddr, 1);
          if (link_tuple.neighborIfaceAddr == nb_tuple.neighborMainAddr)
            nb_main_addr = true;
        }
        else
        {
          NS_LOG_LOGIC ("Link tuple: linkMainAddress= " << GetMainAddress (link_tuple.neighborIfaceAddr)
                        << "; neighborMainAddr =  " << nb_tuple.neighborMainAddr
                        << "; expired=" << int (link_tuple.time < Simulator::Now ())
                        << " => IGNORE");
        }
      }

      // If, in the above, no R_dest_addr is equal to the main
      // address of the neighbor, then another new routing entry
      // with MUST be added, with:
      //      R_dest_addr  = main address of the neighbor;
      //      R_next_addr  = L_neighbor_iface_addr of one of the
      //                     associated link tuple with L_time >= current time;
      //      R_dist       = 1;
      //      R_iface_addr = L_local_iface_addr of the
      //                     associated link tuple.
      if (!nb_main_addr && lt != NULL)
      {
        NS_LOG_LOGIC ("no R_dest_addr is equal to the main address of the neighbor "
                      "=> adding additional routing entry");
        AddEntry (nb_tuple.neighborMainAddr, lt->neighborIfaceAddr, lt->localIfaceAddr, 1);
      }
    }
  }

  //  3. for each node in N2, i.e., a 2-hop neighbor which is not a
  //  neighbor node or the node itself, and such that there exist at
  //  least one entry in the 2-hop neighbor set where
  //  N_neighbor_main_addr correspond to a neighbor node with
  //  willingness different of WILL_NEVER,
  const TwoHopNeighborSet &twoHopNeighbors = m_state.GetTwoHopNeighbors ();
  for (TwoHopNeighborSet::const_iterator it = twoHopNeighbors.begin (); it != twoHopNeighbors.end (); it++)
  {
    TwoHopNeighborTuple const &nb2hop_tuple = *it;

    NS_LOG_LOGIC ("Looking at two-hop neighbor tuple: " << nb2hop_tuple);

    // a 2-hop neighbor which is not a neighbor node or the node itself
    if (m_state.FindSymNeighborTuple (nb2hop_tuple.twoHopNeighborAddr))
    {
      NS_LOG_LOGIC ("Two-hop neighbor tuple is also neighbor; skipped.");
      continue;
    }

    if (nb2hop_tuple.twoHopNeighborAddr == m_mainAddress)
    {
      NS_LOG_LOGIC ("Two-hop neighbor is self; skipped.");
      continue;
    }

    // ...and such that there exist at least one entry in the 2-hop
    // neighbor set where N_neighbor_main_addr correspond to a
    // neighbor node with willingness different of WILL_NEVER...
    bool nb2hopOk = false;
    for (NeighborSet::const_iterator neighbor = neighborSet.begin (); neighbor != neighborSet.end (); neighbor++)
    {
      if (neighbor->neighborMainAddr == nb2hop_tuple.neighborMainAddr && neighbor->willingness != OLSR_WILL_NEVER)
      {
        nb2hopOk = true;
        break;
      }
    }

    if (!nb2hopOk)
    {
      NS_LOG_LOGIC ("Two-hop neighbor tuple skipped: 2-hop neighbor "
                    << nb2hop_tuple.twoHopNeighborAddr
                    << " is attached to neighbor " << nb2hop_tuple.neighborMainAddr
                    << ", which was not found in the Neighbor Set.");
      continue;
    }

    // one selects one 2-hop tuple and creates one entry in the routing table with:
    //                R_dest_addr  =  the main address of the 2-hop neighbor;
    //                R_next_addr  = the R_next_addr of the entry in the
    //                               routing table with:
    //                                   R_dest_addr == N_neighbor_main_addr
    //                                                  of the 2-hop tuple;
    //                R_dist       = 2;
    //                R_iface_addr = the R_iface_addr of the entry in the
    //                               routing table with:
    //                                   R_dest_addr == N_neighbor_main_addr
    //                                                  of the 2-hop tuple;
    RoutingTableEntry entry;
    bool foundEntry = Lookup (nb2hop_tuple.neighborMainAddr, entry);
    if (foundEntry)
    {
      NS_LOG_LOGIC ("Adding routing entry for two-hop neighbor.");
      AddEntry (nb2hop_tuple.twoHopNeighborAddr, entry.nextAddr, entry.interface, 2);
    }
    else
    {
      NS_LOG_LOGIC ("NOT adding routing entry for two-hop neighbor ("
                    << nb2hop_tuple.twoHopNeighborAddr
                    << " not found in the routing table)");
    }
  }

  for (uint32_t h = 2;; h++)
  {
    bool added = false;

    // 3.1. For each topology entry in the topology table, if its
    // T_dest_addr does not correspond to R_dest_addr of any
    // route entry in the routing table AND its T_last_addr
    // corresponds to R_dest_addr of a route entry whose R_dist
    // is equal to h, then a new route entry MUST be recorded in
    // the routing table (if it does not already exist)
    const TopologySet &topology = m_state.GetTopologySet ();
    for (TopologySet::const_iterator it = topology.begin (); it != topology.end (); it++)
    {
      const TopologyTuple &topology_tuple = *it;
      NS_LOG_LOGIC ("Looking at topology tuple: " << topology_tuple);

      RoutingTableEntry destAddrEntry, lastAddrEntry;
      bool have_destAddrEntry = Lookup (topology_tuple.destAddr, destAddrEntry);
      bool have_lastAddrEntry = Lookup (topology_tuple.lastAddr, lastAddrEntry);
      if (!have_destAddrEntry && have_lastAddrEntry && lastAddrEntry.distance == h)
      {
        NS_LOG_LOGIC ("Adding routing table entry based on the topology tuple.");
        // then a new route entry MUST be recorded in
        //                the routing table (if it does not already exist) where:
        //                     R_dest_addr  = T_dest_addr;
        //                     R_next_addr  = R_next_addr of the recorded
        //                                    route entry where:
        //                                    R_dest_addr == T_last_addr
        //                     R_dist       = h+1; and
        //                     R_iface_addr = R_iface_addr of the recorded
        //                                    route entry where:
        //                                       R_dest_addr == T_last_addr.
        AddEntry (topology_tuple.destAddr, lastAddrEntry.nextAddr, lastAddrEntry.interface, h + 1);
        added = true;
      }
      else
      {
        NS_LOG_LOGIC ("NOT adding routing table entry based on the topology tuple: "
                      "have_destAddrEntry=" << have_destAddrEntry
                      << " have_lastAddrEntry=" << have_lastAddrEntry
                      << " lastAddrEntry.distance=" << (int) lastAddrEntry.distance
                      << " (h=" << h << ")");
      }
    }

    if (!added)
      break;
  }

  // 4. For each entry in the multiple interface association base
  // where there exists a routing entry such that:
  // R_dest_addr == I_main_addr (of the multiple interface association entry)
  // AND there is no routing entry such that:
  // R_dest_addr == I_iface_addr
  const IfaceAssocSet &ifaceAssocSet = m_state.GetIfaceAssocSet ();
  for (IfaceAssocSet::const_iterator it = ifaceAssocSet.begin ();
  it != ifaceAssocSet.end (); it++)
  {
  IfaceAssocTuple const &tuple = *it;
  RoutingTableEntry entry1, entry2;
  bool have_entry1 = Lookup (tuple.mainAddr, entry1);
  bool have_entry2 = Lookup (tuple.ifaceAddr, entry2);
  if (have_entry1 && !have_entry2)
  {
  // then a route entry is created in the routing table with:
  //       R_dest_addr  =  I_iface_addr (of the multiple interface
  //                                     association entry)
  //       R_next_addr  =  R_next_addr  (of the recorded route entry)
  //       R_dist       =  R_dist       (of the recorded route entry)
  //       R_iface_addr =  R_iface_addr (of the recorded route entry).
  AddEntry (tuple.ifaceAddr,
  entry1.nextAddr,
  entry1.interface,
  entry1.distance);
  }
  }

  // 5. For each tuple in the association set,
  //    If there is no entry in the routing table with:
  //        R_dest_addr     == A_network_addr/A_netmask
  //   and if the announced network is not announced by the node itself,
  //   then a new routing entry is created.
  const AssociationSet &associationSet = m_state.GetAssociationSet ();

  // Clear HNA routing table
  for (uint32_t i = 0; i < m_hnaRoutingTable->GetNRoutes (); i++)
  {
  m_hnaRoutingTable->RemoveRoute (0);
  }

  for (AssociationSet::const_iterator it = associationSet.begin ();
  it != associationSet.end (); it++)
  {
  AssociationTuple const &tuple = *it;

  // Test if HNA associations received from other gateways
  // are also announced by this node. In such a case, no route
  // is created for this association tuple (go to the next one).
  bool goToNextAssociationTuple = false;
  const Associations &localHnaAssociations = m_state.GetAssociations ();
  NS_LOG_DEBUG ("Nb local associations: " << localHnaAssociations.size ());
  for (Associations::const_iterator assocIterator = localHnaAssociations.begin ();
  assocIterator != localHnaAssociations.end (); assocIterator++)
  {
  Association const &localHnaAssoc = *assocIterator;
  if (localHnaAssoc.networkAddr == tuple.networkAddr && localHnaAssoc.netmask == tuple.netmask)
  {
  NS_LOG_DEBUG ("HNA association received from another GW is part of local HNA associations: no route added for network "
  << tuple.networkAddr << "/" << tuple.netmask);
  goToNextAssociationTuple = true;
  }
  }
  if (goToNextAssociationTuple)
  {
  continue;
  }

  RoutingTableEntry gatewayEntry;

  bool gatewayEntryExists = Lookup (tuple.gatewayAddr, gatewayEntry);
  bool addRoute = false;

  uint32_t routeIndex = 0;

  for (routeIndex = 0; routeIndex < m_hnaRoutingTable->GetNRoutes (); routeIndex++)
  {
  Ipv4RoutingTableEntry route = m_hnaRoutingTable->GetRoute (routeIndex);
  if (route.GetDestNetwork () == tuple.networkAddr
  && route.GetDestNetworkMask () == tuple.netmask)
  {
  break;
  }
  }

  if (routeIndex == m_hnaRoutingTable->GetNRoutes ())
  {
  addRoute = true;
  }
  else if (gatewayEntryExists && m_hnaRoutingTable->GetMetric (routeIndex) > gatewayEntry.distance)
  {
  m_hnaRoutingTable->RemoveRoute (routeIndex);
  addRoute = true;
  }

  if (addRoute && gatewayEntryExists)
  {
  m_hnaRoutingTable->AddNetworkRouteTo (tuple.networkAddr,
                      tuple.netmask,
                      gatewayEntry.nextAddr,
                      gatewayEntry.interface,
                      gatewayEntry.distance);

  }
  }

  NS_LOG_DEBUG ("Node " << m_mainAddress << ": RoutingTableComputation end."); */
  m_routingTableChanged (GetSize ());
}

int64_t
RoutingProtocol::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

bool
RoutingProtocol::IsMyOwnAddress (const Ipv4Address & a) const
{
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
  {
    Ipv4InterfaceAddress iface = j->second;
    
    if (a == iface.GetLocal ())
      return true;
  }
  
  return false;
}

void
RoutingProtocol::Dump (void)
{
#ifdef NS3_LOG_ENABLE
  Time now = Simulator::Now ();
  NS_LOG_DEBUG ("Dumping for node with main address " << m_mainAddress);
  //m_outfile << "Dumping for node with main address " << m_mainAddress << "\n";
  NS_LOG_DEBUG (" Neighbor set");
  //m_outfile << " Neighbor set" << "\n";
  /*
  for (NeighborSet::const_iterator iter = GetNeighbors ().begin ();
       iter != GetNeighbors ().end (); iter++)
    {
      NS_LOG_DEBUG ("  " << *iter);
    }
   */
  NS_LOG_DEBUG (" Two-hop neighbor set");
  //m_outfile << " Two-hop neighbor set" << "\n";
  /*
  for (TwoHopNeighborSet::const_iterator iter = GetTwoHopNeighbors ().begin ();
       iter != GetTwoHopNeighbors ().end (); iter++)
    {
      if (now < iter->expirationTime)
        {
          NS_LOG_DEBUG ("  " << *iter);
        }
    }
   */
  NS_LOG_DEBUG (" Routing table");
  //m_outfile << " Routing table" << "\n";
  /*
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin (); iter != m_table.end (); iter++)
    {
      NS_LOG_DEBUG ("  dest=" << iter->first << " --> next=" << iter->second.nextAddr << " via interface " << iter->second.interface);
    }
   */
  NS_LOG_DEBUG ("");
#endif  //NS3_LOG_ENABLE
}

Ptr<const Ipv4StaticRouting>
RoutingProtocol::GetRoutingTableAssociation () const
{
  return m_hnaRoutingTable;
}

} // namespace olsr

}

