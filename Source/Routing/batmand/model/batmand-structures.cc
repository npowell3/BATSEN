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

#include "batmand-routing-protocol.h"
#include "ns3/ipv4-address.h"
#include "ns3/log.h"
#include "ns3/names.h"

#include <arpa/inet.h>

#include <iostream>
#include <iomanip>

namespace ns3 {
  
NS_LOG_COMPONENT_DEFINE ("BatmanStructs");

namespace batmand {

static const unsigned int BitCount[16] = {
    0,    // 0000
    1,    // 0001
    1,    // 0010
    2,    // 0011
    1,    // 0100
    2,    // 0101
    2,    // 0110
    3,    // 0111
    1,    // 1000
    2,    // 1001
    2,    // 1010
    3,    // 1011
    2,    // 1100
    3,    // 1101
    3,    // 1110
    4     // 1111
};

void RoutingProtocol::AddBatmanIface(batman_if *iface) {
  NS_LOG_DEBUG("Adding new interface to m_batmanif: " << iface << " with ifnum " << (int)(iface->if_num) );
  //  m_outfile << "Adding new interface to m_batmanif: " << iface << " with ifnum " << (int)(iface->if_num) << "\n";
  m_batmanif.push_back (iface);
  NS_LOG_DEBUG("m_batmanif.sz " << m_batmanif.size() );
  //  m_outfile << "m_batmanif.sz " << m_batmanif.size() << "\n";
}

struct batman_if *RoutingProtocol::FindBatmanIf( Ipv4Address addr ) {
   int size = m_batmanif.size();
   batman_if *iface = NULL;

   for ( int i = 0; i < size; ++i ) {
      if ( m_batmanif[i]->addr == addr ) {
         iface = m_batmanif[i];
         break;
      }
   }

   return iface;
}

struct batman_if *RoutingProtocol::FindBatmanIf( uint8_t ifnum ) {
   int size = m_batmanif.size();
   batman_if *iface = NULL;

   for ( int i = 0; i < size; ++i ) {
      if ( m_batmanif[i]->if_num == ifnum ) {
         iface = m_batmanif[i];
         break;
      }
   }

   return iface;
}

struct orig_node *RoutingProtocol::GetOrCreateOrigNode( Ipv4Address addr, int ifnum ) {
   struct orig_node *orig_node = FindOrigNode( addr );

   if ( NULL == orig_node ) {
      struct orig_node *tmp = new struct orig_node;

      tmp->orig = addr;
      
      if ( ifnum > -1 )
         tmp->batman_if = FindBatmanIf(ifnum);

      for (uint8_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
      {
        tmp->bcast_own[i] = 0lu;
        tmp->bcast_own_sum[i] = (uint8_t)0;
      }

      m_OriginSet.push_back( tmp );
      orig_node = FindOrigNode( addr );
      NS_LOG_DEBUG( "\t   Creating new originator for " << addr << " location " << (long int)orig_node );
     //m_outfile << "\t   Creating new originator for " << addr << " location " << (long int)orig_node << "\n";
      NS_ASSERT( NULL != orig_node );
   }
//   else
//     NS_LOG_DEBUG( "Existing originator for " << addr << " found " << (long int)orig_node );

   return orig_node;
}


struct orig_node *RoutingProtocol::FindOrigNode( Ipv4Address addr ) {
   int size = m_OriginSet.size();
   orig_node *originnode = NULL;

   for ( int i = 0; i < size; ++i ) {
      if ( m_OriginSet[i]->orig == addr ) {
         originnode = m_OriginSet[i];
         break;
      }
   }

   return originnode;
}


void RoutingProtocol::purge_orig(ns3::Time curr_time) {
  struct hash_it_t *hashit = NULL;
  struct orig_node *orig_node;
  struct neigh_node *neigh_node, *best_neigh_node;
  struct gw_node *gw_node;
  uint8_t gw_purged = 0, neigh_purged, max_tq;

  /* for all origins... */
  while ( NULL != ( hashit = hash_iterate( m_hnaGlobalHash, hashit ) ) ) {
    orig_node = (struct orig_node *)(hashit->bucket->data);

    if ((int)(curr_time.GetInteger() - (orig_node->last_valid + (2 * m_purgeTimeout.GetInteger()))) > 0) {
      NS_LOG_DEBUG( "Originator timeout: originator " << orig_node->orig << ", last_valid " << orig_node->last_valid );
      //m_outfile << "Originator timeout: originator " << orig_node->orig << ", last_valid " << orig_node->last_valid << "\n";
      
      hash_remove_bucket( m_hnaGlobalHash, hashit );

      /* for all neighbours towards this originator ... */
      orig_node->neigh_list.clear();

      for (unsigned int i = 0; i < m_GatewayList.size(); ++ i) {
        gw_node = m_GatewayList[i];

        if ( gw_node->deleted.GetInteger() > 0 )
          continue;

        if ( gw_node->orig_node == orig_node ) {
          NS_LOG_DEBUG( "Removing gateway " << gw_node->orig_node->orig << " from gateway list" );
          //m_outfile << "Removing gateway " << gw_node->orig_node->orig << " from gateway list" << "\n";
          
          gw_node->deleted = Simulator::Now();
          gw_purged = 1;
          break;
        }
      }

      UpdateRoutes( orig_node, NULL, NULL, 0 );

      orig_node->bcast_own.clear();
      orig_node->bcast_own_sum.clear();
      delete orig_node;
    } 
    else {
      best_neigh_node = NULL;
      max_tq = neigh_purged = 0;

      /* for all neighbours towards this originator ... */
      NeighborList::iterator itr = orig_node->neigh_list.begin();
      while ( itr != orig_node->neigh_list.end() ) {
        neigh_node = &(*itr);

        if ((int)(curr_time.GetInteger() - (neigh_node->last_valid + m_purgeTimeout.GetInteger())) > 0) {
          NS_LOG_DEBUG( "Neighbour timeout: originator " << orig_node->orig <<
            ", neighbour: " << neigh_node->addr <<
            ", last_valid " << neigh_node->last_valid );
          //m_outfile << "Neighbour timeout: originator " << orig_node->orig <<
          //          ", neighbour: " << neigh_node->addr <<
          //          ", last_valid " << neigh_node->last_valid << "\n";
          
          if (orig_node->router == neigh_node) {
            /* we have to delete the route towards this node before it gets purged */
            NS_LOG_DEBUG( "Deleting previous route" );
            //m_outfile << "Deleting previous route" << "\n";

            /* remove old announced network(s) */
            HNAGlobalDel(orig_node);
            RemoveEntry( orig_node->orig );

            /* if the neighbour is the route towards our gateway */
            if ((m_currGateway != NULL) && (m_currGateway->orig_node == orig_node))
              m_currGateway = NULL;

            orig_node->router = NULL;
          }

          neigh_purged = 1;

          orig_node->neigh_list.erase( itr );
          delete neigh_node;

          // Don't increment the iterator, as the return from the erase places
          // the iterator at the element immediately following the previously
          // erased element
          continue;
        }
        else {
          if ((best_neigh_node == NULL) || (neigh_node->tq_avg > max_tq)) {
            best_neigh_node = neigh_node;
            max_tq = neigh_node->tq_avg;
          }
        }

        ++itr;
      }

      if ((neigh_purged) && ((best_neigh_node == NULL) || 
          (orig_node->router == NULL) ||
          (max_tq > orig_node->router->tq_avg)))
        UpdateRoutes( orig_node, best_neigh_node, orig_node->hna_buff, orig_node->hna_buff_len );
    }
  }

  GatewayList::iterator itr = m_GatewayList.begin();
  while( itr != m_GatewayList.end() ) {
    gw_node = *itr;

    if ((gw_node->deleted.GetInteger() > 0 ) && 
        ((int)(curr_time.GetInteger() - (gw_node->deleted.GetInteger() + (2 * m_purgeTimeout.GetInteger() ))) > 0)) {
      m_GatewayList.erase( itr );
      delete gw_node;
      continue;
    } 
    
    ++itr;
  }

  if ( gw_purged )
    ChooseGateway();
}


void RoutingProtocol::debug_orig(void) {
  /*
  struct hash_it_t *hashit = NULL;
  struct list_head *forw_pos, *orig_pos, *neigh_pos;
  struct forw_node *forw_node;
  struct orig_node *orig_node;
  struct neigh_node *neigh_node;
  struct gw_node *gw_node;
  uint16_t batman_count = 0;
  uint64_t uptime_sec;
  int download_speed, upload_speed, debug_out_size;
  char str[ADDR_STR_LEN], str2[ADDR_STR_LEN], orig_str[ADDR_STR_LEN], debug_out_str[1001];


  if ( debug_clients.clients_num[1] > 0 ) {

    addr_to_string( ((struct batman_if *)if_list.next)->addr.sin_addr.s_addr, orig_str, sizeof(orig_str) );
    uptime_sec = (uint64_t)(get_time_msec64() / 1000);

    NS_LOG_DEBUG( "BOD");
    NS_LOG_DEBUG( "%''12s     (%s/%i) %''15s [%10s], gw_class ... [B.A.T.M.A.N. %s%s, MainIF/IP: %s/%s, UT: %id%2ih%2im] \n", "Gateway", "#", TQ_MAX_VALUE, "Nexthop", "outgoingIF", SOURCE_VERSION, (strlen(REVISION_VERSION) > 3 ? REVISION_VERSION : ""), ((struct batman_if *)if_list.next)->dev, orig_str, (uint32_t)(uptime_sec/86400), (uint32_t)((uptime_sec%86400)/3600), (uint32_t)((uptime_sec%3600)/60));

    if ( list_empty( &gw_list ) ) {
      NS_LOG_DEBUG( "No gateways in range ... " );
    } 
    else {
      list_for_each( orig_pos, &gw_list ) {

	gw_node = list_entry( orig_pos, struct gw_node, list );

	if ( gw_node->deleted )
	  continue;

	if (gw_node->orig_node->router == NULL)
	  continue;

	addr_to_string( gw_node->orig_node->orig, str, sizeof (str) );
	addr_to_string( gw_node->orig_node->router->addr, str2, sizeof (str2) );

	get_gw_speeds( gw_node->orig_node->gwflags, &download_speed, &upload_speed );

	NS_LOG_DEBUG( "%s %-15s (%3i) %''15s [%10s], gw_class %3i - %i%s/%i%s, gateway failures: %i \n", ( m_currGateway == gw_node ? "=>" : "  " ), str, gw_node->orig_node->router->tq_avg, str2, gw_node->orig_node->router->if_incoming->dev, gw_node->orig_node->gwflags, (download_speed > 2048 ? download_speed / 1024 : download_speed), (download_speed > 2048 ? "MBit" : "KBit"), (upload_speed > 2048 ? upload_speed / 1024 : upload_speed), (upload_speed > 2048 ? "MBit" : "KBit"), gw_node->gw_failure);

	batman_count++;
      }

      if ( batman_count == 0 )
	NS_LOG_DEBUG( "No gateways in range ... " );
    }

    NS_LOG_DEBUG( "EOD" );
  }

  if ( ( debug_clients.clients_num[0] > 0 ) || ( debug_clients.clients_num[3] > 0 ) ) {

    addr_to_string( ((struct batman_if *)if_list.next)->addr.sin_addr.s_addr, orig_str, sizeof(orig_str) );
    uptime_sec = (uint64_t)(get_time_msec64() / 1000);

    NS_LOG_DEBUG( "BOD ");
    NS_LOG_DEBUG( "  %-11s (%s/%i) %''15s [%10s]: %''20s ... [B.A.T.M.A.N. %s%s, MainIF/IP: %s/%s, UT: %id%2ih%2im] \n", "Originator", "#", TQ_MAX_VALUE, "Nexthop", "outgoingIF", "Potential nexthops", SOURCE_VERSION, (strlen(REVISION_VERSION) > 3 ? REVISION_VERSION : ""), ((struct batman_if *)if_list.next)->dev, orig_str, (uint32_t)(uptime_sec/86400), (uint32_t)((uptime_sec%86400)/3600), (uint32_t)((uptime_sec%3600)/60));

    if ( debug_clients.clients_num[3] > 0 ) {
      NS_LOG_DEBUG( "------------------ DEBUG ------------------ " );
      NS_LOG_DEBUG( "Forward list " );

      list_for_each( forw_pos, &forw_list ) {
	forw_node = list_entry( forw_pos, struct forw_node, list );
	addr_to_string( ((struct bat_packet *)forw_node->pack_buff)->orig, str, sizeof(str) );
	NS_LOG_DEBUG( "    %s at %u \n", str, forw_node->send_time );
      }

      NS_LOG_DEBUG( "Originator list " );
      NS_LOG_DEBUG( "  %-11s (%s/%i) %''15s [%10s]: %''20s\n", "Originator", "#", TQ_MAX_VALUE, "Nexthop", "outgoingIF", "Potential nexthops" );
    }

    while ( NULL != ( hashit = hash_iterate( m_hnaGlobalHash, hashit ) ) ) {
      orig_node = hashit->bucket->data;

      if ( orig_node->router == NULL )
	continue;

      batman_count++;

      addr_to_string( orig_node->orig, str, sizeof (str) );
      addr_to_string( orig_node->router->addr, str2, sizeof (str2) );

      NS_LOG_DEBUG( "%-15s (%3i) %''15s [%10s]:", str, orig_node->router->tq_avg, str2, orig_node->router->if_incoming->dev );
      NS_LOG_DEBUG( "%''15s (%3i) %''15s [%10s], last_valid: %u: \n", str, orig_node->router->tq_avg, str2, orig_node->router->if_incoming->dev, orig_node->last_valid );

      debug_out_size = 0;

      list_for_each( neigh_pos, &orig_node->neigh_list ) {
	neigh_node = list_entry( neigh_pos, struct neigh_node, list );

	addr_to_string( neigh_node->addr, str, sizeof (str) );

	debug_out_size = debug_out_size + snprintf( ( debug_out_str + debug_out_size ), ( sizeof(debug_out_str) - 1 - debug_out_size ), " %15s (%3i)", str, neigh_node->tq_avg);

	if ( (unsigned int)(debug_out_size + 30) > sizeof(debug_out_str) - 1 ) {
	  NS_LOG_DEBUG( "%s \n", debug_out_str );
	  NS_LOG_DEBUG( "%s \n", debug_out_str );

	  debug_out_size = 0;
	}
      }

      if (debug_out_size > 0) {
	NS_LOG_DEBUG( "%s \n", debug_out_str );
	NS_LOG_DEBUG( "%s \n", debug_out_str );
      }
    }

    if ( batman_count == 0 ) {
      NS_LOG_DEBUG( "No batman nodes in range ... " );
      NS_LOG_DEBUG( "No batman nodes in range ... " );
    }

    NS_LOG_DEBUG( "EOD" );
    NS_LOG_DEBUG( "---------------------------------------------- END DEBUG " );
  }
  */
}

struct neigh_node *RoutingProtocol::FindNeighborNode( struct orig_node *orig_node, Ipv4Address addr ) {
   int size = orig_node->neigh_list.size();
   struct neigh_node *retval = NULL;

   for ( int i = 0; i < size; ++i ) {
      if ( orig_node->neigh_list[i].addr == addr ) {
         retval = &( orig_node->neigh_list[i] );
         break;
      }
   }

   return retval;
}

/* turn corresponding bit on, so we can remember that we got the packet */
void RoutingProtocol::MarkSequenceBit( uint64_t &seq_bits, int32_t n ) {
  /* if too old, just drop it, otherwise mark it */
//  NS_LOG_DEBUG( "\tMrkSeqBit input seq bits " << seq_bits );
  if ( !( n < 0 || n >= local_win_size) ) {
    seq_bits |= (1 << n);  /* turn the position on */
//    NS_LOG_DEBUG( "\tmark bit " << n << " result seq bits " << seq_bits );
  }
}


int RoutingProtocol::Uint64BitCount( uint64_t &seq_bits ) {

  int i, hamming = 0;
  uint8_t *word = (uint8_t *)&seq_bits;

  for (i = 0; i < 8; i++) {
    uint8_t high = ( (*word) & 0xF0 ) >> 4;
    uint8_t low  = ( (*word) & 0x0F );

    hamming += BitCount[high] + BitCount[low];
    ++word;
  }

  return(hamming);
}


bool RoutingProtocol::CountRealPackets( OGMHeader &pkt, Ipv4Address neigh, uint8_t ifnum )
{
  struct orig_node *orig_node;
  struct neigh_node *tmpNeighNode;
  unsigned int i = 0;
  bool is_duplicate = false;
  bool is_new_seqno = false;
  
  // I added this since we don't count the current reception for some reason !?
  bool foundSource = false;

  orig_node = GetOrCreateOrigNode( pkt.GetOriginatorAddress(), ifnum );

  int32_t pktSeqNumber = pkt.GetPacketSequenceNumber();
  int32_t lastSeqNumber = orig_node->last_real_seqno;

  NS_LOG_DEBUG("\tcount_real_packets: orig = " << orig_node->orig << ", neigh = "<< neigh <<", seq = " << pktSeqNumber << ", last seq = " << orig_node->last_real_seqno << " on interface " << (int)ifnum );
  //m_outfile << "\tcount_real_packets: orig = " << orig_node->orig << ", neigh = "<< neigh <<", seq = " << pktSeqNumber << ", last seq = " << orig_node->last_real_seqno << " on interface " << (int)ifnum  << "\n";
  NS_LOG_DEBUG("\t  orig node " << orig_node->orig << " neigh list sz " << orig_node->neigh_list.size() );
  //m_outfile << "\t  orig node " << orig_node->orig << " neigh list sz " << orig_node->neigh_list.size()  << "\n";
  
  // Traverse the neighbor list for the given origin node
  for ( i = 0; i < orig_node->neigh_list.size(); ++i ) {
    tmpNeighNode = &(orig_node->neigh_list[i]);
    
    NS_LOG_DEBUG( "\t    check tmpNbr addr " << tmpNeighNode->addr << " vs. neigh " << neigh );
    //m_outfile << "\t    check tmpNbr addr " << tmpNeighNode->addr << " vs. neigh " << neigh << "\n";
    if ( tmpNeighNode->addr == neigh ) {
       NS_LOG_DEBUG("\t     ADDR FOUND in nbr list\n");
      //m_outfile << "\t     ADDR FOUND in nbr list\n" << "\n";
       foundSource = true;
    }

    NS_LOG_DEBUG("\t    nbr list itr: " << tmpNeighNode->addr );
    //m_outfile << "\t    nbr list itr: " << tmpNeighNode->addr << "\n";

    if ( !is_duplicate )
      is_duplicate = GetHistoricalSequenceBitStatus( tmpNeighNode->real_bits, lastSeqNumber, pktSeqNumber );

    if ( ( tmpNeighNode->addr == neigh ) && ( tmpNeighNode->ifnum == ifnum ) ) {
      is_new_seqno = bit_get_packet( tmpNeighNode->real_bits, pktSeqNumber - lastSeqNumber, 1 );
      NS_LOG_DEBUG("\t    count_real_packets (yes): neigh = " << neigh << ", is_new = " << (is_new_seqno ? "YES" : "NO") << ", seq = " << pktSeqNumber << ", last seq = " << lastSeqNumber );
      //m_outfile << "\t    count_real_packets (yes): neigh = " << neigh << ", is_new = " << (is_new_seqno ? "YES" : "NO") << ", seq = " << pktSeqNumber << ", last seq = " << lastSeqNumber << "\n";
    }
    else {
      is_new_seqno = bit_get_packet( tmpNeighNode->real_bits, pktSeqNumber - lastSeqNumber, 0 );
      NS_LOG_DEBUG("\t    count_real_packets (no): neigh = " << neigh << ", is_new = " << (is_new_seqno ? "YES" : "NO") << ", seq = " << pktSeqNumber << ", last seq = " << lastSeqNumber );
      //m_outfile <<  "\t    count_real_packets (no): neigh = " << neigh << ", is_new = " << (is_new_seqno ? "YES" : "NO") << ", seq = " << pktSeqNumber << ", last seq = " << lastSeqNumber << "\n";
    }

    NS_LOG_DEBUG("\t    Counting 64 bit field for tmpNeighbor " << tmpNeighNode->addr << " with real bits " << std::hex << tmpNeighNode->real_bits << " and old count " << std::dec << (int)(tmpNeighNode->real_packet_count) );
    //m_outfile << "\t    Counting 64 bit field for tmpNeighbor " << tmpNeighNode->addr << " with real bits " << std::hex << tmpNeighNode->real_bits << " and old count " << std::dec << (int)(tmpNeighNode->real_packet_count) << "\n";
    tmpNeighNode->real_packet_count = Uint64BitCount( tmpNeighNode->real_bits );
    NS_LOG_DEBUG("\t      new real packet count " << (int)(tmpNeighNode->real_packet_count) );
    //m_outfile << "\t      new real packet count " << (int)(tmpNeighNode->real_packet_count) << "\n";
  }

  if ( !is_duplicate ) {
    NS_LOG_DEBUG("\t    updating last_seqno: old " << lastSeqNumber << ", new " << pktSeqNumber );
    //m_outfile << "\t    updating last_seqno: old " << lastSeqNumber << ", new " << pktSeqNumber  << "\n";
    orig_node->last_real_seqno = pktSeqNumber;
  }
  
  // Add a neigh node to the origin's neighbor list, and perform the count on that
  if ( !foundSource ) {
     NS_LOG_DEBUG("\t  Source " << neigh << " not found in nbr list - so created it" );
    //m_outfile << "\t  Source " << neigh << " not found in nbr list - so created it" << "\n";
     tmpNeighNode = CreateNeighbor(orig_node, GetOrCreateOrigNode( neigh, ifnum ), neigh, ifnum);
     tmpNeighNode->real_packet_count = Uint64BitCount( tmpNeighNode->real_bits );
     NS_LOG_DEBUG("\t    Neighbor real pkt cnt " << (int)tmpNeighNode->real_packet_count );
    //m_outfile << "\t    Neighbor real pkt cnt " << (int)tmpNeighNode->real_packet_count << "\n";
  }

  return is_duplicate;
}


bool RoutingProtocol::GetHistoricalSequenceBitStatus( uint64_t &seq_bits, int32_t &last_seqno, int32_t &curr_seqno ) {
  bool retval = false;
  int16_t diff;

//  NS_LOG_FUNCTION( this );

  diff = last_seqno - curr_seqno;

//  NS_LOG_DEBUG( "\t  lst seq " << last_seqno << " - cur seq " << curr_seqno << " = diff " << diff << " ... loc win sz " << local_win_size );

  if ( !((diff < 0) || (diff >= local_win_size)) )
    retval = seq_bits & (1 << diff);

  //  NS_LOG_DEBUG( "\t    is duplicate? : " << ( seq_bits & (1 << diff) ) );
  return retval;
}


bool RoutingProtocol::bit_get_packet( uint64_t &seq_bits, int16_t seq_num_diff, int8_t set_mark ) {
  bool isnew = true;

  //NS_LOG_DEBUG( "\t  bit_get_packet: seq_bits " << (long int)seq_bits << " seq num diff " << (int)seq_num_diff << " set mark " << (int)set_mark );
  /* We already got a sequence number higher than this one, so we
   * just mark it. This should wrap around the integer just fine.
   */
  if ((seq_num_diff < 0) && (seq_num_diff >= -local_win_size)) {
    NS_LOG_DEBUG("\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\nHOW THE FUCK IS THIS POSSIBLE!!!!???\n\n");
    //m_outfile << "\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\nHOW THE FUCK IS THIS POSSIBLE!!!!???\n\n" << "\n";
    if ( set_mark )
      MarkSequenceBit( seq_bits, -seq_num_diff );
    isnew = false;
  }
  else if ( (seq_num_diff > local_win_size) || (seq_num_diff < -local_win_size) ) {
    NS_LOG_DEBUG("\n$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\nHOW THE FUCK IS THIS POSSIBLE!!!!???\n\n");
    //m_outfile << "\n$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\nHOW THE FUCK IS THIS POSSIBLE!!!!???\n\n" << "\n";
    /* it seems we missed a lot of packets or the other host restarted */
    if (seq_num_diff > local_win_size) {
      NS_LOG_DEBUG("\t  It seems we missed a lot of packets (" << (seq_num_diff - 1) <<") !");
      //m_outfile << "\t  It seems we missed a lot of packets (" << (seq_num_diff - 1) <<") !" << "\n";
    }

    if (-seq_num_diff > local_win_size) {
      NS_LOG_DEBUG("\t  Other host probably restarted !");
      //m_outfile << "\t  Other host probably restarted !" << "\n";
    }

    seq_bits = 0;

    /* we only have the latest packet */
    if ( set_mark )
      seq_bits = 1;
  }
  else {
//    NS_LOG_DEBUG( "\t  calling bit shift seq " << seq_bits );
    bit_shift(seq_bits, seq_num_diff);

    NS_LOG_DEBUG( "\t    bit shift result seq " << std::hex << seq_bits << std::dec );
    //m_outfile << "\t    bit shift result seq " << std::hex << seq_bits << std::dec << "\n";
    if ( set_mark )
      MarkSequenceBit(seq_bits, 0);
  }

  NS_LOG_DEBUG("\t ret seq bits " << std::hex << seq_bits << " is new " << std::dec << isnew );
  //m_outfile << "\t ret seq bits " << std::hex << seq_bits << " is new " << std::dec << isnew << "\n";
  return isnew;
}


void RoutingProtocol::bit_shift( uint64_t &seq_bits, int32_t n ) {
  if( n > 0 ) {
//     NS_LOG_DEBUG( "\t bit_shift\t original seq bits: " << seq_bits );
     seq_bits <<= n;
//     NS_LOG_DEBUG( "\t   Final seq bits: " << seq_bits );
  }
}


struct neigh_node *RoutingProtocol::CreateNeighbor( struct orig_node *orig_node,
                                                    struct orig_node *orig_neigh_node,
                                                    Ipv4Address neigh,
                                                    uint8_t ifnum ) {
   struct neigh_node neigh_node;

   NS_LOG_DEBUG( "\t  Creating new last-hop neighbor of originator" );
  //m_outfile << "\t  Creating new last-hop neighbor of originator" << "\n";
   memset( &neigh_node, 0, sizeof(struct neigh_node) );

   neigh_node.addr = neigh;
   neigh_node.orig_node = orig_neigh_node;
   neigh_node.ifnum = ifnum;
   neigh_node.if_incoming = FindBatmanIf(ifnum);
   
   // We obviously received from this neighbor, so set it's first real_bit
   neigh_node.real_bits = 1lu;
   
   NS_LOG_DEBUG( "\t    New Neighbor " << neigh );
  //m_outfile << "\t    New Neighbor " << neigh << "\n";
//   neigh_node.tq_recv = debugMalloc(sizeof(uint16_t) * global_win_size, 406);
//   memset(neigh_node.tq_recv, 0, sizeof(uint16_t) * global_win_size);
//
//   neigh_node->real_bits = debugMalloc(sizeof(TYPE_OF_WORD) * num_words, 407);
//   memset(neigh_node.real_bits, 0, sizeof(TYPE_OF_WORD) * num_words);

   orig_node->neigh_list.push_back(neigh_node);
   NS_LOG_DEBUG("\t    Origin   " << orig_node->orig << " orig ptr " << orig_node );
  //m_outfile << "\t    Origin   " << orig_node->orig << " orig ptr " << orig_node << "\n";
   return FindNeighborNode( orig_node, neigh );
}


void RoutingProtocol::ScheduleForwardPacket(struct orig_node *orig_node, OGMHeader &in,
                                       Ipv4Address neigh, uint8_t directlink, int16_t hna_buff_len,
                                       uint8_t ifnum, ns3::Time curr_time)
{
  struct forw_node *forw_node_new = NULL, *forw_node_aggregate = NULL, *forw_node_pos = NULL;
  struct bat_packet *bat_packet;
  uint8_t tq_avg = 0;
  ns3::Time send_time;

  NS_LOG_DEBUG( "schedule_forward_packet( hna len " << hna_buff_len << "):\n\t\tAddr " << in.GetOriginatorAddress() <<
		" TTL " << (int)(in.GetTtl()) << " SeqNo " << (int)(in.GetPacketSequenceNumber()) <<
		" HNA Len " << (int)(in.GetHnaLength()) << " iface " << (int)ifnum );
  //m_outfile <<  "schedule_forward_packet( hna len " << hna_buff_len << "):\n\t\tAddr " << in.GetOriginatorAddress() <<
		//" TTL " << (int)(in.GetTtl()) << " SeqNo " << (int)(in.GetPacketSequenceNumber()) <<
    //" HNA Len " << (int)(in.GetHnaLength()) << " iface " << (int)ifnum << "\n";
  if (in.GetTtl() <= 1) {
    NS_LOG_DEBUG( "\tttl exceeded " << in.GetTtl() );
    //m_outfile << "\tttl exceeded " << in.GetTtl() << "\n";
    return;
  }

  if (m_aggregation)
    send_time = curr_time + ns3::Time(MAX_AGGREGATION_MS) - (JITTER/2) + JITTER;
  else
    send_time = curr_time + (JITTER/2);


  /* find position for the packet in the forward queue */
  ForwardingList::iterator itr = m_forwardingSet.begin();
  ForwardingList::iterator prev_list_head = m_forwardingSet.begin();
  uint8_t loopCount = 0;
  NS_LOG_DEBUG("\tCheck all fwd msgs - sz " <<  m_forwardingSet.size() << " aggregate " << m_aggregation );
  //m_outfile << "\tCheck all fwd msgs - sz " <<  m_forwardingSet.size() << " aggregate " << m_aggregation << "\n";
  
  while ( itr != m_forwardingSet.end() ) {
    NS_LOG_DEBUG("\t\tnext itr " << *itr);
    //m_outfile << "\t\tnext itr " << *itr << "\n";
    forw_node_pos = *itr;

    bat_packet = (struct bat_packet *)forw_node_pos->pack_buff;
    Ipv4Address hisaddr;
    hisaddr.Set( htonl( bat_packet->orig ) );
    
    NS_LOG_DEBUG("\t\tcnt " << (int)loopCount << " Addr " << hisaddr << " TTL " << (int)(bat_packet->ttl) <<
		 " SeqNo " << (int)ntohs(bat_packet->seqno) << " HNA Len " << (int)(bat_packet->hna_len) );
    //m_outfile << "\t\tcnt " << (int)loopCount << " Addr " << hisaddr << " TTL " << (int)(bat_packet->ttl) <<
    //" SeqNo " << (int)ntohs(bat_packet->seqno) << " HNA Len " << (int)(bat_packet->hna_len) << "\n";
    
//    NS_LOG_DEBUG( "\t     aggregate: " << m_aggregation );
    
    if (m_aggregation) {
        // NOTE: Moved to 661 as we were overflowing the MAX_AGG_BYTES per packet
//      /* don't save aggregation position if aggregation is disabled */
//      forw_node_aggregate = forw_node_pos;

      NS_LOG_DEBUG( "\t\tfwd nd sndT " << forw_node_pos->send_time << " rxPkt sndT " << send_time );
      //m_outfile << "\t\tfwd nd sndT " << forw_node_pos->send_time << " rxPkt sndT " << send_time << "\n";
      /**
       * we can aggregate the current packet to this packet if:
       * - the send time is within our MAX_AGGREGATION_MS time
       * - the resulting packet wont be bigger than MAX_AGGREGATION_BYTES
       */
      /*
       * NOTE: [NHP] - changed this to allow anything scheduled up to 100ms 
       * after the current entry to get pulled in... MAX_AGGREGATION_MS was 0ns
       */
      if (((forw_node_pos->send_time - send_time) < MAX_AGGREGATION_NS ) &&
          (forw_node_pos->pack_buff_len + sizeof(struct bat_packet) + hna_buff_len <= MAX_AGGREGATION_BYTES)) {

        /* don't save aggregation position if aggregation is disabled */
        forw_node_aggregate = forw_node_pos;
      
        bat_packet = (struct bat_packet *)forw_node_pos->pack_buff;

        /**
         * check aggregation compatibility
         * -> direct link packets are broadcasted on their interface only
         * -> aggregate packet if the current packet is a "global" packet
         *    as well as the base packet
         */
        /* packets without direct link flag and high TTL are flooded through the net  */
        if ((!directlink) && (!(bat_packet->flags & DIRECTLINK)) && (bat_packet->ttl != 1) &&
        /* own packets originating non-primary interfaces leave only that interface */
            ((!forw_node_pos->own) || (forw_node_pos->if_incoming->if_num == 1))) {
          NS_LOG_DEBUG("\t   flood high TTL ? ");
          //m_outfile << "\t   flood high TTL ? " << "\n";
          break;
        }

        /* if the incoming packet is sent via this one interface only - we still can aggregate */
        if ((directlink) && (in.GetTtl() == TTL) && (forw_node_pos->if_incoming->if_num == ifnum)) {
          NS_LOG_DEBUG("\t   direct link aggregation ");
          //m_outfile << "\t   direct link aggregation " << "\n";
          break;
        }
      }
    }

    if ((forw_node_pos->send_time - send_time) > 0) {
      NS_LOG_DEBUG("\t  break from loop at cnt " << (int)loopCount << " fwd snd T " << 
		   forw_node_pos->send_time << " desire snd T " << send_time);
      //m_outfile << "\t  break from loop at cnt " << (int)loopCount << " fwd snd T " <<
      //forw_node_pos->send_time << " desire snd T " << send_time << "\n";
      break;
    }

    prev_list_head = itr;
    forw_node_pos = NULL;

    NS_LOG_DEBUG("\t  prev list head @ " << (int)loopCount );
    //m_outfile << "\t  prev list head @ " << (int)loopCount << "\n";
    
    ++itr;
    ++loopCount;
  }

  
  // Clean up the TTL before we add the OGM to the forward set
  {
     in.SetTtl( in.GetTtl() - 1 );
     in.SetDirectLinkFlag( directlink );
  }
  
  unsigned char *data = in.Serialize();
  unsigned char *ptr = NULL;
  
  /* nothing to aggregate with - either aggregation disabled or no suitable aggregation packet found */
  if (forw_node_aggregate == NULL) {
    NS_LOG_DEBUG("\tCreating new fwd node for iface " << (int)ifnum );
    //m_outfile << "\tCreating new fwd node for iface " << (int)ifnum << "\n";
    ForwardingList::iterator itr = m_forwardingSet.begin();
    forw_node_new = new struct forw_node;
    forw_node_new->pack_buff = new unsigned char[MAX_AGGREGATION_BYTES];
    forw_node_new->own = 0;
    forw_node_new->if_incoming = FindBatmanIf( ifnum );
    forw_node_new->num_packets = 1;
    forw_node_new->direct_link_flags = (directlink ? 1 : 0);
    forw_node_new->send_time = send_time;
    forw_node_new->srcAddr = in.GetOriginatorAddress();
    forw_node_new->pack_buff_len = in.GetPacketLength();
    memcpy(forw_node_new->pack_buff, (void *)data, forw_node_new->pack_buff_len );
    
    NS_LOG_DEBUG("Pushing the first pkt into the list");
    //m_outfile << "Pushing the first pkt into the list" << "\n";
    NS_ASSERT( forw_node_new->if_incoming != NULL );
        
      /* if the packet was not aggregated */
      /*
       * NOTE: [NHP] this was at the bottom of the function, but only made sense
       * for the C style queuing from original code. C++ <lists> fixed up the 
       * issue - thus leaving it in caused me issues by adding extra transmissions
       */
//  if (forw_node_aggregate == NULL) {
    NS_LOG_DEBUG("\tAdding pkt to fwd list - not aggregated");
    //m_outfile << "\tAdding pkt to fwd list - not aggregated" << "\n";
//    ForwardingList::iterator itr = m_forwardingSet.begin();
    
    /* if the packet should go somewhere in the queue */
    if (forw_node_pos != NULL) {
      NS_LOG_DEBUG("\t  inserting pkt ");
      //m_outfile << "\t  inserting pkt " << "\n";
      itr = m_forwardingSet.insert ( prev_list_head, forw_node_new );
    }
    else {
      /* if the packet is the last packet in the queue */
      NS_LOG_DEBUG("\t  push back pkt ");
      //m_outfile << "\t  push back pkt " << "\n";
      m_forwardingSet.push_back( forw_node_new );
      itr = (m_forwardingSet.end())--; 
    }
//  }
  
    // Resize the m_forwardingSet and put this packet up front ??
//    m_forwardingSet.push_front( forw_node_new );
//    itr = m_forwardingSet.begin();

    NS_ASSERT( itr != m_forwardingSet.end() );
    bat_packet = (struct bat_packet *)(*itr)->pack_buff;
    
    NS_LOG_DEBUG("\tFORWD SET size now " << m_forwardingSet.size() );
    //m_outfile << "\tFORWD SET size now " << m_forwardingSet.size() << "\n";
  } 
  else {
    NS_LOG_DEBUG("\tcopying pkt into fwd node: cur pkt buf len " << (int)forw_node_aggregate->pack_buff_len );
    //m_outfile << "\tcopying pkt into fwd node: cur pkt buf len " << (int)forw_node_aggregate->pack_buff_len << "\n";
    ptr = forw_node_aggregate->pack_buff + forw_node_aggregate->pack_buff_len;
    bat_packet = (struct bat_packet *)(ptr);
    memcpy( ptr, (void *)data, in.GetPacketLength() );

    forw_node_aggregate->pack_buff_len += in.GetPacketLength();

    NS_LOG_DEBUG("Pushing additional pkt to back of list");
    //m_outfile << "Pushing additional pkt to back of list" << "\n";
    
    // This is for NS-3's tx trace callback
    forw_node_aggregate->num_packets++;
    forw_node_new = forw_node_aggregate;
  }

  /* save packet direct link flag status */
  if (directlink)
    forw_node_new->direct_link_flags = forw_node_new->direct_link_flags | (1 << forw_node_new->num_packets);

//  bat_packet->ttl--;
  bat_packet->prev_sender = htonl( neigh.Get() );
  // This is for NS-3's tx trace callback
  forw_node_new->msgList.push_back( in );

  /* rebroadcast tq of our best ranking neighbor to ensure the rebroadcast of our best tq value */
  if ((orig_node->router != NULL) && (orig_node->router->tq_avg != 0)) {

    /* rebroadcast ogm of best ranking neighbor as is */
    if (orig_node->router->addr != neigh) {
      bat_packet->tq = orig_node->router->tq_avg;
      bat_packet->ttl = orig_node->router->last_ttl - 1;
    }

    tq_avg = orig_node->router->tq_avg;
  }

  /*  
   * apply hop penalty
   * NOTE: I removeed the scaling factor - original method requires floating point
   * operations to be effective. If floating point is used, the TQ allows for up to
   * 116 hop, whereas a straight -5 adjustment allows for only 51 hops
   */
  bat_packet->tq = (uint8_t)(((double)(bat_packet->tq * (TQ_MAX_VALUE - m_hopPenalty))) / ((double)TQ_MAX_VALUE));
  
  NS_LOG_DEBUG( "forwarding: tq_orig: " << (int)in.GetTQvalue() << ", tq_avg: " <<
		(int)tq_avg << ", tq_forw: " << (int)bat_packet->tq <<
                ", ttl_orig: " << (int)(in.GetTtl() + 1) << ", ttl_forw: " << (int)bat_packet->ttl );
  //m_outfile << "forwarding: tq_orig: " << (int)in.GetTQvalue() << ", tq_avg: " <<
  //	(int)tq_avg << ", tq_forw: " << (int)bat_packet->tq <<
  //", ttl_orig: " << (int)(in.GetTtl() + 1) << ", ttl_forw: " << (int)bat_packet->ttl << "\n";

//  /* change sequence number to network order */
//  bat_packet->seqno = htons(bat_packet->seqno);

  if (directlink)
    bat_packet->flags |= DIRECTLINK;
  else
    bat_packet->flags &= ~DIRECTLINK;

}


void RoutingProtocol::UpdateOrig( struct orig_node *origNode,
                                  OGMHeader &in,
                                  Ipv4Address neigh,
                                  uint16_t ifnum,
                                  unsigned char *hna_recv_buff,
                                  int16_t hna_buff_len,
                                  bool is_duplicate,
                                  ns3::Time curr_time ) {
  struct gw_node *gw_node;
  struct neigh_node *neighNode = NULL, *tmpNeighNode = NULL, *best_neigh_node = NULL;
  uint8_t max_bcast_own = 0, max_tq = 0;

  NS_LOG_DEBUG( "###############\nupdate_originator(): Searching and updating originator entry of received packet\n\t num neighbors " <<
                origNode->neigh_list.size() << " orig " << origNode->orig << " neigh " << neigh );
  //m_outfile << "###############\nupdate_originator(): Searching and updating originator entry of received packet\n\t num neighbors " <<
  //origNode->neigh_list.size() << " orig " << origNode->orig << " neigh " << neigh << "\n";

  for ( unsigned int i = 0; i < origNode->neigh_list.size(); ++i ) {

    tmpNeighNode = &( origNode->neigh_list[i] );

    NS_LOG_DEBUG("\t  tmpNeigh addr " << tmpNeighNode->addr );
    //m_outfile << "\t  tmpNeigh addr " << tmpNeighNode->addr << "\n";
    if ( ( tmpNeighNode->addr == neigh ) && ( tmpNeighNode->ifnum == ifnum ) ) {
      neighNode = tmpNeighNode;
      NS_LOG_DEBUG("\t    neighNode set to tmpNeigh addr " << tmpNeighNode->addr );
      //m_outfile << "\t    neighNode set to tmpNeigh addr " << tmpNeighNode->addr << "\n";
    } 
    else {
      if ( !is_duplicate ) {
        NS_LOG_DEBUG("\t    not dup for tmp nbr " << tmpNeighNode->addr << " tq_avg " << (int)tmpNeighNode->tq_avg );
        //m_outfile << "\t    not dup for tmp nbr " << tmpNeighNode->addr << " tq_avg " << (int)tmpNeighNode->tq_avg << "\n";
        //ring_buffer_set(tmpNeighNode->tq_recv, &tmpNeighNode->tq_index, 0);
        //tmpNeighNode->tq_avg = ring_buffer_avg(tmpNeighNode->tq_recv);
      }

      /* if we got have a better TQ value via
       * this neighbor or same TQ value if it is
       * currently our best neighbour (to avoid route flipping) */
      NS_LOG_DEBUG("\t    tq_avg " << (int)tmpNeighNode->tq_avg << " max_tq " << (int)max_tq );
      //m_outfile << "\t    tq_avg " << (int)tmpNeighNode->tq_avg << " max_tq " << (int)max_tq << "\n";
      NS_LOG_DEBUG("\t    bcast own sum " << (int)(tmpNeighNode->orig_node->bcast_own_sum[ifnum]) << " max_bcast own " << (int)max_bcast_own );
      //m_outfile << "\t    bcast own sum " << (int)(tmpNeighNode->orig_node->bcast_own_sum[ifnum]) << " max_bcast own " << (int)max_bcast_own << "\n";
      NS_LOG_DEBUG("\t    router " << origNode->router );
      //m_outfile << "\t    router " << origNode->router << "\n";
      
      if ( (tmpNeighNode->tq_avg > max_tq) ||
	         ((tmpNeighNode->tq_avg == max_tq) && (tmpNeighNode->orig_node->bcast_own_sum[ifnum] > max_bcast_own)) ||
	         ((origNode->router == tmpNeighNode) && (tmpNeighNode->tq_avg == max_tq)) ) {
        max_tq = tmpNeighNode->tq_avg;
        max_bcast_own = tmpNeighNode->orig_node->bcast_own_sum[ifnum];
        best_neigh_node = tmpNeighNode;
        NS_LOG_DEBUG("\t    bestNeigh set to tmpNeigh " << best_neigh_node->addr );
        //m_outfile << "\t    bestNeigh set to tmpNeigh " << best_neigh_node->addr << "\n";
      }
    }
  }

  if ( neighNode == NULL ) {
    neighNode = CreateNeighbor(origNode, GetOrCreateOrigNode(neigh, ifnum), neigh, ifnum);
  } 
  else {
    NS_LOG_DEBUG( "\t Updating existing last-hop neighbor of originator" );
    //m_outfile << "\t Updating existing last-hop neighbor of originator" << "\n";
  }

  neighNode->last_valid = curr_time.GetInteger();

  ring_buffer_set(neighNode->tq_recv, &neighNode->tq_index, in.GetTQvalue() );
  neighNode->tq_avg = ring_buffer_avg(neighNode->tq_recv);

  /*  is_new_seqno = bit_get_packet( neigh_node->seq_bits, in->seqno - orig_node->last_seqno, 1 );
      is_new_seqno = ! get_bit_status( neigh_node->real_bits, orig_node->last_real_seqno, in->seqno ); */


  if ( !is_duplicate ) {
    origNode->last_ttl = in.GetTtl();
    neighNode->last_ttl = in.GetTtl();
  }

  if ((neighNode->tq_avg > max_tq) ||
      ((neighNode->tq_avg == max_tq) && (neighNode->orig_node->bcast_own_sum[ifnum] > max_bcast_own)) ||
      ((origNode->router == neighNode) && (neighNode->tq_avg == max_tq)) ) {
    best_neigh_node = neighNode;
    NS_LOG_DEBUG("\t  bestNeigh set to " << best_neigh_node->addr );
    //m_outfile << "\t  bestNeigh set to " << best_neigh_node->addr << "\n";
  }

  /* update routing table and check for changed hna announcements */
  UpdateRoutes( origNode, best_neigh_node, hna_recv_buff, hna_buff_len );

  if ( origNode->gwflags != in.GetGWFlags() )
    UpdateGatewayList( origNode, in.GetGWFlags(), in.GetGatewayPortNumber() );

  origNode->gwflags = in.GetGWFlags();
//TODO:  hna_global_check_tq(orig_node);

  /* restart gateway selection if we have more packets and fast or late switching enabled */
  if ((m_routingClass > 2) && (origNode->gwflags != 0) && (m_currGateway != NULL)) {

    /* if the node is not our current gateway and
     * we have preferred gateway disabled and a better
     * TQ value or we found our preferred gateway
     */
    if ((m_currGateway->orig_node != origNode) &&
        (((m_prefGateway == 0) && (origNode->router->tq_avg > m_currGateway->orig_node->router->tq_avg)) ||
        (m_prefGateway == origNode->orig)) ) {

      /* it is our preferred gateway or we have fast switching 
       * or the tq is $routing_class better than our old tq 
       */
      if ((m_prefGateway == origNode->orig) ||
          (m_routingClass == 3) ||
          (origNode->router->tq_avg - m_currGateway->orig_node->router->tq_avg >= m_routingClass)) {
        gw_node = NULL;

        for ( unsigned int j = 0; j < m_GatewayList.size(); ++j ) {
          gw_node = m_GatewayList[j];

          if (gw_node->orig_node == origNode)
            break;

          gw_node = NULL;
        }

        /* if this gateway had not a gateway failure within the last 30 seconds */
        if ((gw_node != NULL) && ((int)(curr_time.GetInteger() - (gw_node->last_failure + 30000)) > 0)) {
          NS_LOG_DEBUG("\t Gateway client - restart gateway selection: better gateway found (tq curr: " <<
              m_currGateway->orig_node->router->tq_avg <<
            ", tq new: " << origNode->router->tq_avg << ")");
          //m_outfile << "\t Gateway client - restart gateway selection: better gateway found (tq curr: " <<
          //m_currGateway->orig_node->router->tq_avg <<
          //", tq new: " << origNode->router->tq_avg << ")" << "\n";

          m_currGateway = NULL;
        }
      }
    }
  }
}

void RoutingProtocol::ring_buffer_set(uint8_t tq_recv[], uint8_t *tq_index, uint8_t value)
{
  NS_LOG_DEBUG("\t  Ring buf set:\tindx " << (int)(*tq_index) << " trgt val " << (int)value << " glb win sz " << global_win_size );
  //m_outfile << "\t  Ring buf set:\tindx " << (int)(*tq_index) << " trgt val " << (int)value << " glb win sz " << global_win_size << "\n";
  tq_recv[*tq_index] = value;
  *tq_index = (*tq_index + 1) % global_win_size;
  NS_LOG_DEBUG("\t     new index " << (int)(*tq_index) );
  //m_outfile << "\t     new index " << (int)(*tq_index) << "\n";
}

uint8_t RoutingProtocol::ring_buffer_avg(uint8_t tq_recv[])
{
  uint8_t *ptr;
  uint16_t count = 0, i = 0;
  uint32_t sum = 0;

  ptr = tq_recv;

  while (i < global_win_size) {
    if (*ptr != 0) {
      count++;
      sum += *ptr;
      NS_LOG_DEBUG("\t     ringbuf avg i " << (int)i << " val " << (int)(*ptr) << " sum " << (int)sum << " cnt " << (int)count );
      //m_outfile << "\t     ringbuf avg i " << (int)i << " val " << (int)(*ptr) << " sum " << (int)sum << " cnt " << (int)count << "\n";
    }

    i++;
    ptr++;
  }

  NS_LOG_DEBUG("\t        ringbuf avg " << (int)((count == 0) ? 0 : (uint8_t)(sum / count)) );
  //m_outfile << "\t        ringbuf avg " << (int)((count == 0) ? 0 : (uint8_t)(sum / count)) << "\n";
  return ((count == 0) ? 0 : (uint8_t)(sum / count));
}


void RoutingProtocol::UpdateRoutes(struct orig_node *orig_node,
                              struct neigh_node *neigh_node,
                              unsigned char *hna_recv_buff,
                              int16_t hna_buff_len)
{
  struct neigh_node *old_router;

  NS_LOG_DEBUG( "  update_routes() " );
  //m_outfile << "  %%%%%%    update_routes()   %%%%%%" << "\n";

  old_router = orig_node->router;

  uint32_t distance = 0;
  
  /* also handles orig_node->router == NULL and neigh_node == NULL */
  if ((orig_node != NULL) && (orig_node->router != neigh_node)) {

    if ( ( orig_node != NULL ) && ( neigh_node != NULL ) )
    {
      NS_LOG_DEBUG( "\tRoute to " << orig_node->orig << " via " << neigh_node->addr << " with neigh ptr " << neigh_node );
      //m_outfile << "\tRoute to " << orig_node->orig << " via " << neigh_node->addr << " with neigh ptr " << neigh_node << "\n";
    }

    /* adds duplicated code but makes it more readable */

    /* new route added */
    if ((orig_node->router == NULL) && (neigh_node != NULL)) {

      distance = 51 - neigh_node->last_ttl;
      NS_LOG_DEBUG( "\tAdding new route TO " << orig_node->orig << " THRU " << neigh_node->addr << " DIST " << distance );
      //m_outfile << "\tAdding new route TO " << orig_node->orig << " THRU " << neigh_node->addr << " DIST " << distance << "\n";
      // TODO: last argument is the distance
      AddEntry( orig_node->orig, neigh_node->addr, (uint32_t)neigh_node->if_incoming->if_num, distance );

      orig_node->batman_if = neigh_node->if_incoming;
      orig_node->router = neigh_node;

      /* add new announced network(s) */
      HNAGlobalAdd(orig_node, hna_recv_buff, hna_buff_len);

    /* route deleted */
    } else if ((orig_node->router != NULL) && (neigh_node == NULL)) {

      NS_LOG_DEBUG( "\tDeleting previous route TO " << orig_node->orig );
      //m_outfile << "\tDeleting previous route TO " << orig_node->orig << "\n";

      /* remove old announced network(s) */
      HNAGlobalDel(orig_node);
      RemoveEntry( orig_node->orig );

    /* route changed */
    }
    else {

      NS_LOG_DEBUG( "\tRoute changed" );
      //m_outfile << "\tRoute changed" << "\n";

      distance = 51 - neigh_node->last_ttl;

      // Original code deleted the old route after adding the new one
      // but the original code relied on NET_LINK sockets... so not sure

      /* delete old route */
//      NS_LOG_DEBUG( "\tRemoving previous route TO " << orig_node->orig );
//      RemoveEntry( orig_node->orig );
      /* add new route */
      NS_LOG_DEBUG( "\tReplacing new route TO " << orig_node->orig << " THRU " << neigh_node->addr << " DIST " << distance );
      //m_outfile << "\tReplacing new route TO " << orig_node->orig << " THRU " << neigh_node->addr << " DIST " << distance << "\n";
      UpdateEntry( orig_node->orig, neigh_node->addr, (uint32_t)neigh_node->if_incoming->if_num, distance );

      orig_node->batman_if = neigh_node->if_incoming;
      orig_node->router = neigh_node;

      /* update announced network(s) */
      HNAGlobalUpdate(orig_node, hna_recv_buff, hna_buff_len, old_router);
    }

    orig_node->router = neigh_node;

  }
  else if (orig_node != NULL) {
     
    if ( orig_node->router == neigh_node ){
       NS_LOG_DEBUG("  rtr for " << orig_node->orig << " already set to " << neigh_node->addr );
      //m_outfile << "  rtr for " << orig_node->orig << " already set to " << neigh_node->addr << "\n";
    }
    HNAGlobalUpdate(orig_node, hna_recv_buff, hna_buff_len, old_router);
  }
}


void RoutingProtocol::UpdateGatewayList(struct orig_node *orig_node, uint8_t new_gwflags, uint16_t gw_port)
{
  struct gw_node *gw_node;
  int download_speed, upload_speed;

  for ( unsigned int x = 0; x < m_GatewayList.size(); ++x ) {

    gw_node = m_GatewayList[x];

    if ( gw_node->orig_node == orig_node ) {
      NS_LOG_DEBUG( "Gateway class of originator " << gw_node->orig_node->orig <<
                    " changed from " << gw_node->orig_node->gwflags <<
                    " to " << new_gwflags );
      //m_outfile << "Gateway class of originator " << gw_node->orig_node->orig <<
      //          " changed from " << gw_node->orig_node->gwflags <<
      //          " to " << new_gwflags << "\n";

      if ( new_gwflags == 0 ) {
        gw_node->deleted = Simulator::Now ();
        gw_node->orig_node->gwflags = new_gwflags;
        NS_LOG_DEBUG( "Gateway " << orig_node->orig << " removed from gateway list" );
        //m_outfile << "Gateway " << orig_node->orig << " removed from gateway list" << "\n";

        if (gw_node == m_currGateway)
          ChooseGateway();
      }
      else {
        gw_node->deleted = Seconds( 0.0 );
        gw_node->orig_node->gwflags = new_gwflags;
      }

      return;
    }
  }

  get_gw_speeds( new_gwflags, &download_speed, &upload_speed );

  NS_LOG_DEBUG( "Found new gateway " << orig_node->orig << " -> class: " << new_gwflags <<
                " - " << ( download_speed > 2048 ? download_speed / 1024 : download_speed ) <<
                ( download_speed > 2048 ? "MBit" : "KBit" ) <<
                "/" << ( upload_speed > 2048 ? upload_speed / 1024 : upload_speed ) <<
                ( upload_speed > 2048 ? "MBit" : "KBit" ) );
  
  //m_outfile << "Found new gateway " << orig_node->orig << " -> class: " << new_gwflags <<
  //            " - " << ( download_speed > 2048 ? download_speed / 1024 : download_speed ) <<
  //            ( download_speed > 2048 ? "MBit" : "KBit" ) <<
  //            "/" << ( upload_speed > 2048 ? upload_speed / 1024 : upload_speed ) <<
  //            ( upload_speed > 2048 ? "MBit" : "KBit" ) << "\n";

  gw_node = new struct gw_node;
  memset(gw_node, 0, sizeof(struct gw_node));
  gw_node->orig_node = orig_node;
  gw_node->gw_port = gw_port;
  gw_node->last_failure = Simulator::Now().GetInteger();

  m_GatewayList.push_back( gw_node );
}


void RoutingProtocol::ChooseGateway(void)
{
  struct gw_node *gw_node, *tmp_curr_gw = NULL;
  uint8_t max_gw_class = 0, max_tq = 0;
  uint32_t current_time, max_gw_factor = 0, tmp_gw_factor = 0;
  int download_speed, upload_speed;

  current_time = Simulator::Now().GetInteger();

  if ((m_routingClass == 0) ||
      ((m_routingClass < 4) && ((int64_t)((Simulator::Now().GetInteger()) - (m_ogmInterval.GetInteger() * local_win_size)) < 0))) {
    return;
  }

  if ( m_GatewayList.empty() ) {
    if ( m_currGateway != NULL ) {
      NS_LOG_DEBUG( "Removing default route - no gateway in range" );
      //m_outfile << "Removing default route - no gateway in range" << "\n";
      m_currGateway = NULL;
    }
    return;
  }

  for ( unsigned int x = 0; x < m_GatewayList.size(); ++x ) {
    gw_node = m_GatewayList[x];

    /* ignore this gateway if recent connection attempts were unsuccessful */
    /* if it is our only gateway retry immediately */
    if ( m_GatewayList.size() > 1 ) {
      if ((int)(current_time - (gw_node->last_failure + 30000)) < 0)
        continue;
    }

    if ( gw_node->orig_node->router == NULL )
      continue;

    if ( gw_node->deleted != Seconds( 0.0 ) )
      continue;

    switch ( m_routingClass ) {

      case 1: /* fast connection */
        get_gw_speeds( gw_node->orig_node->gwflags, &download_speed, &upload_speed );

        if (((tmp_gw_factor = (((gw_node->orig_node->router->tq_avg * 100 ) / local_win_size) *
                  ((gw_node->orig_node->router->tq_avg * 100) / local_win_size) *
                     (download_speed / 64))) > max_gw_factor) ||
                  ((tmp_gw_factor == max_gw_factor) && (gw_node->orig_node->router->tq_avg > max_tq)))
          tmp_curr_gw = gw_node;
        break;

      default: /* stable connection (use best statistic) */
         /* fast-switch (use best statistic but change as soon as a better gateway appears) */
         /* late-switch (use best statistic but change as soon as a better gateway appears
          * which has $m_routingClass more tq points) */
        if (gw_node->orig_node->router->tq_avg > max_tq)
          tmp_curr_gw = gw_node;
        break;
    }

    if ( gw_node->orig_node->gwflags > max_gw_class )
      max_gw_class = gw_node->orig_node->gwflags;

    if (gw_node->orig_node->router->tq_avg > max_tq)
      max_tq = gw_node->orig_node->router->tq_avg;

    if ( tmp_gw_factor > max_gw_factor )
      max_gw_factor = tmp_gw_factor;

    if ( ( m_prefGateway != 0 ) && ( m_prefGateway == gw_node->orig_node->orig ) ) {
      tmp_curr_gw = gw_node;
      NS_LOG_DEBUG( "Preferred gateway found: " << tmp_curr_gw->orig_node->orig <<
                    " (gw_flags: " << gw_node->orig_node->gwflags <<
                    ", tq: " << gw_node->orig_node->router->tq_avg <<
                    ", gw_product: " << tmp_gw_factor << ")" );
      //m_outfile << "Preferred gateway found: " << tmp_curr_gw->orig_node->orig <<
      //            " (gw_flags: " << gw_node->orig_node->gwflags <<
      //            ", tq: " << gw_node->orig_node->router->tq_avg <<
      //            ", gw_product: " << tmp_gw_factor << ")" << "\n";
      break;
    }
  }

  if ( m_currGateway != tmp_curr_gw ) {
    if ( m_currGateway != NULL ) {
      if ( tmp_curr_gw != NULL ) {
        NS_LOG_DEBUG( "Removing default route - better gateway found" );
        //m_outfile << "Removing default route - better gateway found" << "\n";
      }
      else {
        NS_LOG_DEBUG( "Removing default route - no gateway in range" );
        //m_outfile << "Removing default route - no gateway in range" << "\n";
      }

      m_currGateway = NULL;
    }

    m_currGateway = tmp_curr_gw;

    /* may be the last gateway is now gone */
    if ( m_currGateway != NULL ) {
      NS_LOG_DEBUG( "Adding default route to " << m_currGateway->orig_node->orig <<
                    " (gw_flags: " << max_gw_class << ", tq: " << max_tq <<
                    ", gw_product: " << max_gw_factor << ")" );
      //m_outfile << "Adding default route to " << m_currGateway->orig_node->orig <<
      //            " (gw_flags: " << max_gw_class << ", tq: " << max_tq <<
      //            ", gw_product: " << max_gw_factor << ")" << "\n";
// TODO: Add a default route in NS-3
//      add_default_route();
    }
  }
}


/* returns the up and downspeeds in kbit, calculated from the class */
void RoutingProtocol::get_gw_speeds(unsigned char gw_class, int *down, int *up)
{
  char sbit = (gw_class & 0x80) >> 7;
  char dpart = (gw_class & 0x7C) >> 3;
  char upart = (gw_class & 0x07);

  *down = 32 * (sbit + 2) * (1 << dpart);
  *up = ((upart + 1) * (*down)) / 8;
}


/* calculates the gateway class from kbit */
unsigned char RoutingProtocol::get_gw_class(int down, int up)
{
  int mdown = 0, tdown, tup, difference = 0x0FFFFFFF;
  unsigned char gw_class = 0, sbit, part;

  /* test all downspeeds */
  for (sbit = 0; sbit < 2; sbit++) {
    for (part = 0; part < 16; part++) {
      tdown = 32 * (sbit + 2) * (1 << part);

      if ( abs(tdown - down) < difference) {
        gw_class = (sbit << 7) + (part << 3);
        difference = abs(tdown - down);
        mdown = tdown;
      }
    }
  }

  /* test all upspeeds */
  difference = 0x0FFFFFFF;
  for (part = 0; part < 8; part++) {
    tup = ((part + 1) * (mdown)) / 8;
    if (abs(tup - up) < difference) {
      gw_class = (gw_class & 0xF8) | part;
      difference = abs(tup - up);
    }
  }

  return gw_class;
}


void RoutingProtocol::send_outstanding_packets(ns3::Time curr_time)
{
  std::ostringstream os;

  struct forw_node *forw_node;
  struct batman_if *batman_if;
  struct bat_packet *bat_packet;
  uint8_t directlink, curr_packet_num;
  int16_t curr_packet_len;

  NS_LOG_DEBUG( "send_outstanding_packets " << curr_time );
  //m_outfile << "send_outstanding_packets " << curr_time << "\n";
  
  ForwardingList::iterator itr = m_forwardingSet.begin();
  while ( itr != m_forwardingSet.end() ) {
    forw_node = *itr;

    NS_LOG_DEBUG( "  TOP OF send_outstanding_packets  LOOP " << (int)m_forwardingSet.size() );
    //m_outfile << "  TOP OF send_outstanding_packets  LOOP " << (int)m_forwardingSet.size() << "\n";
    
    if ((curr_time.GetInteger() - forw_node->send_time.GetInteger()) < 0) {
      NS_LOG_DEBUG("  Invalid time: curr " << curr_time << " fwd-time " << forw_node->send_time );
      //m_outfile << "  Invalid time: curr " << curr_time << " fwd-time " << forw_node->send_time << "\n";
      
      if ( !m_queuedMessagesTimer.IsRunning() ) {
//        NS_LOG_DEBUG("     Cancelling QMsg Timer: remain " << m_queuedMessagesTimer.GetDelayLeft() );
//        m_queuedMessagesTimer.Cancel();
//      }
      
         // Why cancel a timer just to reset it to original value?
         NS_LOG_DEBUG("     Set QMsg Timer with delay " << ( forw_node->send_time - curr_time ) << " from " << curr_time );
        //m_outfile << "     Set QMsg Timer with delay " << ( forw_node->send_time - curr_time ) << " from " << curr_time << "\n";
         m_queuedMessagesTimer.Schedule( forw_node->send_time - curr_time );
      }
      
      break;
    }

    bat_packet = (struct bat_packet *)forw_node->pack_buff;
    directlink = (bat_packet->flags & DIRECTLINK ? 1 : 0);

    Ipv4Address hisaddr;
    hisaddr.Set( ntohl(bat_packet->orig) );

    if (forw_node->if_incoming == NULL) {
      NS_LOG_DEBUG( "Error - can't forward packet: incoming iface not specified for fwdNode " << forw_node->srcAddr << " my main Addr " << m_mainAddress );
      
      NS_ASSERT( 0 );
    }
    else if ( ((directlink) && (bat_packet->ttl == 1)) ||
              ((forw_node->own) && (forw_node->if_incoming->if_num > 0)) ) {
      NS_LOG_DEBUG(" ************ MULTIHOMED PEER !!! ??? ************ " );
      /* multi-homed peer assumed */
      /* non-primary interfaces are only broadcasted on their interface */
      NS_LOG_DEBUG( (forw_node->own ? "Sending own" : "Forwarding") << " packet (originator " <<
		    hisaddr << ", seqno " << (unsigned int)ntohs(bat_packet->seqno) << ", TTL " <<
		    (unsigned int)(bat_packet->ttl) << ") on interface " << (unsigned int)(forw_node->if_incoming->if_num) );

      struct bat_packet *ptr = (struct bat_packet *)forw_node->pack_buff;
      NS_LOG_DEBUG("A fwd nd buf len " << (int)forw_node->pack_buff_len << " hna len " << (int)ptr->hna_len );
    
      OGMHeader batmanOgmHeader;
      Ptr<Packet> packet = Create<Packet> ( forw_node->pack_buff, forw_node->pack_buff_len );

      packet->PeekHeader (batmanOgmHeader);
      std::ostringstream loc_os;
      batmanOgmHeader.Print( loc_os );

//      NS_LOG_DEBUG("Sending packet with first OGM:" << loc_os.str() );

      for (MessageList::iterator itr = forw_node->msgList.begin(); itr != forw_node->msgList.end(); ++itr ) {
         (*itr).Print( loc_os );
      }
      
      NS_LOG_DEBUG("  Sending packet with headers: " << loc_os.str() );
      
      SendPacket( packet, forw_node->msgList );
    }
    else {
      BatmanIfList::iterator ifitr = m_batmanif.begin();
      while( ifitr != m_batmanif.end() ) {
        batman_if = *ifitr;

        curr_packet_num = curr_packet_len = 0;
        bat_packet = (struct bat_packet *)forw_node->pack_buff;

        while ((curr_packet_len + sizeof(struct bat_packet) <= forw_node->pack_buff_len) &&
               (curr_packet_len + sizeof(struct bat_packet) + (bat_packet->hna_len * 5) <= forw_node->pack_buff_len) &&
               (curr_packet_len + sizeof(struct bat_packet) + (bat_packet->hna_len * 5) <= MAX_AGGREGATION_BYTES)) {

          NS_LOG_DEBUG("\t\tCheck DIRECT LINK for " << bat_packet->orig << " dlFlags " << forw_node->direct_link_flags << " curPkt # " << (int)(1 << curr_packet_num) <<
                       "\n\t\t\t in-if " << forw_node->if_incoming << " bat-if " << batman_if );
          if ((forw_node->direct_link_flags & (1 << curr_packet_num)) && (forw_node->if_incoming == batman_if))
            bat_packet->flags |= DIRECTLINK;
          else
            bat_packet->flags &= ~DIRECTLINK;

          NS_LOG_DEBUG("\t\t   Resulting flags " << (int)(bat_packet->flags) );
          
          /**
           * if the outgoing interface is a wifi interface and equal to the incoming interface
           * add extra penalty (own packets are to be ignored)
           */
          if ((batman_if->wifi_if) && (!forw_node->own) && (forw_node->if_incoming == batman_if)) {
            NS_LOG_DEBUG("        adding weird WIFI only penalty to TQ " );
            bat_packet->tq = (bat_packet->tq * (TQ_MAX_VALUE - (2 * m_hopPenalty))) / (TQ_MAX_VALUE);
          }

          NS_LOG_DEBUG( (curr_packet_num > 0 ? "Forwarding" : (forw_node->own ? "Sending own" : "Forwarding")) <<
                  " " << (curr_packet_num > 0 ? "aggregated " : "") << " packet (originator " << hisaddr <<
                  ", seqno " << (unsigned int)(bat_packet->seqno) << ", TQ " << (unsigned int)(bat_packet->tq) << ", TTL " <<
                  (unsigned int)(bat_packet->ttl) << ", IDF " << (bat_packet->flags & DIRECTLINK ? "on" : "off") << 
                  ") on interface " << (unsigned int)(batman_if->if_num) );

          curr_packet_len += sizeof(struct bat_packet) + (bat_packet->hna_len * 5);
          curr_packet_num++;
          bat_packet = (struct bat_packet *)(forw_node->pack_buff + curr_packet_len);
        }

        struct bat_packet *ptr = (struct bat_packet *)forw_node->pack_buff;
        NS_LOG_DEBUG("B fwd nd buf len " << (int)forw_node->pack_buff_len << " hna len " << (int)ptr->hna_len );

        Ptr<Packet> packet = Create<Packet> ( forw_node->pack_buff, forw_node->pack_buff_len );
        SendPacket( packet, forw_node->msgList );
        ++ifitr;
      }
    }

    bool isMyOgm = forw_node->own;
    struct batman_if *myIf = forw_node->if_incoming;

    // .erase() returns the iterator immediately following the element deleted
    // so there's no need for a temp iterator to record the next location
    itr = m_forwardingSet.erase( itr );
    NS_LOG_DEBUG( "  PRE SCHD OWN send_outstanding_packets  LOOP " << (int)m_forwardingSet.size() );
    //m_outfile << "  PRE SCHD OWN send_outstanding_packets  LOOP " << (int)m_forwardingSet.size() << "\n";
    
    if ( isMyOgm )
      schedule_own_packet( myIf );
    
    NS_LOG_DEBUG( "  BOTTOM OF send_outstanding_packets  LOOP " << (int)m_forwardingSet.size() );
    //m_outfile << "  BOTTOM OF send_outstanding_packets  LOOP " << (int)m_forwardingSet.size() << "\n";
  }
}


void RoutingProtocol::schedule_own_packet(struct batman_if *batman_if)
{
  struct forw_node *forw_node_new = NULL;

  NS_LOG_DEBUG( "\tschedule_own_packet(): if " << (int)batman_if->if_num << " addr " << batman_if->addr << 
		" TTL " << (int)batman_if->out.GetTtl() << " currT " << Simulator::Now() );
  //m_outfile << "\tschedule_own_packet(): if " << (int)batman_if->if_num << " addr " << batman_if->addr <<
		//" TTL " << (int)batman_if->out.GetTtl() << " currT " << Simulator::Now() << "\n";

  forw_node_new = new struct forw_node;

  forw_node_new->send_time = ns3::Simulator::Now() + m_ogmInterval - JITTER +
                             (m_uniformRandomVariable->GetValue(0.0, 1.0) * JITTER );
//                             (m_uniformRandomVariable->GetValue(0.0, 1.0) * (2 * JITTER) );
  forw_node_new->if_incoming = batman_if;
  forw_node_new->own = 1;
  forw_node_new->num_packets = 1;
  forw_node_new->direct_link_flags = 0;
  forw_node_new->srcAddr = batman_if->addr;

  NS_ASSERT( forw_node_new->if_incoming != NULL );
  
//  NS_LOG_DEBUG("  IF out ptr seq pre inc " << batman_if->out.GetPacketSequenceNumber() );
  batman_if->out.IncrementSequenceNumber();

  /* non-primary interfaces do not send hna information */
  // NOTE: primary IF is 1 in ns-3 (changed from == 0)
  if ((m_LocalList.size() > 0) && (batman_if->if_num == 1)) {
//    NS_LOG_DEBUG( "   copying with HNAs(" << (unsigned int)m_LocalList.size() << ") for iface " << (int)batman_if->if_num  );

    forw_node_new->pack_buff = new unsigned char[MAX_AGGREGATION_BYTES];
    HnaLocalList::iterator itr = m_LocalList.begin();
    while ( itr != m_LocalList.end() ) {
      batman_if->out.AddHnaEntry( *itr );
      ++itr;
    }
  } 
  else {
    forw_node_new->pack_buff = new unsigned char[MAX_AGGREGATION_BYTES];
//    NS_LOG_DEBUG( "   copying with NO HNAs for iface " << (int)batman_if->if_num  );
    batman_if->out.ClearHnaEntries();
  }

  memcpy(forw_node_new->pack_buff, batman_if->out.Serialize(), batman_if->out.GetPacketLength());
  forw_node_new->pack_buff_len = batman_if->out.GetPacketLength();

  // Check to see if I already queued one for myself earlier
  bool alreadyInsertedSelf = false;
  ForwardingList::iterator itr = m_forwardingSet.begin();
  while ( itr != m_forwardingSet.end() ) {
    if ( ( (*itr)->srcAddr == forw_node_new->srcAddr ) && 
         ( (*itr)->if_incoming == forw_node_new->if_incoming ) ) 
    {
      struct bat_packet *pkt = (struct bat_packet *) ( (*itr)->pack_buff );
      alreadyInsertedSelf = true;
      NS_LOG_DEBUG("\t   found fwd node for self in fwd list " );
      //m_outfile << "\t   found fwd node for self in fwd list " << "\n";
      NS_LOG_DEBUG("\t   old ttl " << (int)ntohs(pkt->ttl) << " old seqN " << ntohs(pkt->seqno) );
      //m_outfile << "\t   old ttl " << (int)ntohs(pkt->ttl) << " old seqN " << ntohs(pkt->seqno) << "\n";
      NS_LOG_DEBUG("\t   new ttl " << (int)(batman_if->out.GetTtl()) << " new seqN " << 
                   (int)(batman_if->out.GetPacketSequenceNumber()) );
      //m_outfile << "\t   new ttl " << (int)(batman_if->out.GetTtl()) << " new seqN " <<
      //              (int)(batman_if->out.GetPacketSequenceNumber()) << "\n";
      // Don't forget to back up the seq #, as we moved it forward and then dropped the frame
      batman_if->out.DecrementSequenceNumber();
      break;
    }
    
    ++itr;
  }

  if ( !alreadyInsertedSelf ) {
    if ( m_forwardingSet.size() > 0 ) {
      ForwardingList::iterator itr = m_forwardingSet.begin();
      while ( itr != m_forwardingSet.end() ) {
        if ( ((*itr)->send_time - forw_node_new->send_time) >= 0 ) {
          NS_LOG_DEBUG("\tinserting forw_new_node own(" << (int)(forw_node_new->own) << ") seqN (" << (int)(batman_if->out.GetPacketSequenceNumber()) << ")" );
          //m_outfile << "\tinserting forw_new_node own(" << (int)(forw_node_new->own) << ") seqN (" << (int)(batman_if->out.GetPacketSequenceNumber()) << ")" << "\n";
          m_forwardingSet.insert( itr, forw_node_new );
          break;
        }
        ++itr;

        if ( itr == m_forwardingSet.end() ) {
          NS_LOG_DEBUG("\tpush back forw_new_node own(" << (int)(forw_node_new->own) << ") seqN (" << (int)(batman_if->out.GetPacketSequenceNumber()) << ")");
          //m_outfile << "\tpush back forw_new_node own(" << (int)(forw_node_new->own) << ") seqN (" << (int)(batman_if->out.GetPacketSequenceNumber()) << ")" << "\n";
          m_forwardingSet.push_back( forw_node_new );
        }
      }
    }
    else {
      NS_LOG_DEBUG("\tpush front forw_new_node own(" << (int)(forw_node_new->own) << ") seqN (" << (int)(batman_if->out.GetPacketSequenceNumber()) << ")");
      //m_outfile << "\tpush front forw_new_node own(" << (int)(forw_node_new->own) << ") seqN (" << (int)(batman_if->out.GetPacketSequenceNumber()) << ")" << "\n";
      m_forwardingSet.push_front( forw_node_new );
    }

    NS_LOG_DEBUG("\t checking hash_iter");
    //m_outfile << "\t checking hash_iter" << "\n";
    OriginList::iterator itr = m_OriginSet.begin();
    while ( itr != m_OriginSet.end() ) {
      int ifnum = (int)( batman_if->if_num );
      NS_LOG_DEBUG( "\tcount own bcast (in schedule_own_packet): for " << (*itr)->orig << " old = " << (int)((*itr)->bcast_own_sum[ifnum]) << " iface " << ifnum );
      //m_outfile << "\tcount own bcast (in schedule_own_packet): for " << (*itr)->orig << " old = " << (int)((*itr)->bcast_own_sum[ifnum]) << " iface " << ifnum << "\n";
      bit_get_packet( (*itr)->bcast_own[ifnum], 1, 0 );
      (*itr)->bcast_own_sum[ifnum] = Uint64BitCount( (*itr)->bcast_own[ifnum] );
      NS_LOG_DEBUG( "\t  new = " << (int)((*itr)->bcast_own_sum[ifnum]) );
      //m_outfile << "\t  new = " << (int)((*itr)->bcast_own_sum[ifnum]) << "\n";
      itr++;
    }
  }

  NS_LOG_DEBUG("END schedule_own_packet");
  //m_outfile << "END schedule_own_packet" << "\n";
}


}
} // namespace batmand, ns3
