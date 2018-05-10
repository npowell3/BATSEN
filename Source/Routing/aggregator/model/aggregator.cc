//******************************************************************************
//******************************************************************************
//
// FILE:        packet-aggregator.cc
//
// DESCRIPTION: See Class description below
//
//******************************************************************************
//
//                       CONFIDENTIALITY NOTICE:
//
// THIS FILE CONTAINS MATERIAL THAT IS "HARRIS PROPRIETARY INFORMATION"  ANY
// REVIEW, RELIANCE, DISTRIBUTION, DISCLOSURE, OR FORWARDING WITHOUT EXPRESSED
// PERMISSION IS STRICTLY PROHIBITED.  PLEASE BE SURE TO PROPERLY DISPOSE ANY
// HARDCOPIES OF THIS DOCUMENT.
//
//******************************************************************************
//
// Government Use Rights:
//
//           (Applicable only for source code delivered under U. S.
//           Government contracts)
//
//                           RESTRICTED RIGHTS LEGEND
//           Use, duplication, or disclosure is subject to restrictions
//           stated in the Government's contract with Harris Corporation,
//           RF Communications Division. The applicable contract number is
//           indicated on the media containing this software. As a minimum,
//           the Government has restricted rights in the software as
//           defined in DFARS 252.227-7013.
//
// Commercial Use Rights:
//
//           (Applicable only for source code procured under contracts other
//           than with the U. S. Government)
//
//                           TRADE SECRET
//           Contains proprietary information of Harris Corporation.
//
// Copyright:
//           Protected as an unpublished copyright work,
//                    (c) Harris Corporation
//           First fixed in 2004, all rights reserved.
//
//******************************************************************************
//
// HISTORY: Created 12/15/2016 by Nelson Powell
// $Id:$
//
//******************************************************************************
//******************************************************************************

#include "ns3/log.h"
#include "ns3/nstime.h"

#include "aggregator.h"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-mac-header.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <errno.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Aggregator");


Aggregator::Aggregator () {
   
}
   
Aggregator::~Aggregator () {
   ClearData();
}
   
void Aggregator::ClearData() {
   NS_LOG_DEBUG("clearing data packets and router packets");
   m_dataPackets.clear();
   m_routerPackets.clear();
   
   NS_LOG_DEBUG("clearing phy node event");
   NodePhyMap::iterator phy = m_nodePhyEvents.begin();
   while ( phy != m_nodePhyEvents.end() ) {
      phy->second.clear();
      m_nodePhyEvents.erase( phy );
      phy = m_nodePhyEvents.begin();
   }
   
   NS_LOG_DEBUG("clearing event frames");
   TxFrameMap::iterator frm = m_txMacFrames.begin();
   while ( frm != m_txMacFrames.end() ) {
      frm->second.clear();
      m_txMacFrames.erase( frm );
      frm = m_txMacFrames.begin();
   }

   NS_LOG_DEBUG("clearing plot data");
   m_plotData.clear();
}

/******************************************************************************
 * Application Payload Processing Methods
 ******************************************************************************/

void Aggregator::PacketTxDataCallback( Ptr<Packet const> pkt ) {
   TransmitPacket nxtPkt( pkt->GetSize(), pkt->GetUid(), Simulator::Now() );
   
   NS_LOG_DEBUG("\t @ " << Simulator::Now() << " new TX Data Pkt " << pkt->GetUid() );
   // Add the new entry to the Application Packet map based on GUID
   m_dataPackets.push_back( nxtPkt );
}

void Aggregator::PacketRxDataCallback( Ptr<Packet const> pkt ) {
   bool found = false;
   TxPacketList::iterator itr = m_dataPackets.begin();
   
   for( ; itr != m_dataPackets.end(); ++itr ) {
      if ( (*itr).m_guid == pkt->GetUid() ) {
         (*itr).m_stop = Simulator::Now();
         (*itr).m_received = true;
	 NS_LOG_DEBUG("\t @ " << Simulator::Now() << " RX Data Pkt " << pkt->GetUid() );
	 found = true;
         break;
      }
   }
   
   if (!found)
     NS_LOG_DEBUG("\t @ " << Simulator::Now() << " RX UNKNOWN Pkt " << pkt->GetUid() );
}

/******************************************************************************
 * Router Payload Processing Methods
 ******************************************************************************/

void Aggregator::PacketRouterCallback( Ptr<Packet const> pkt ) {
   TransmitPacket nxtPkt( pkt->GetSize(), pkt->GetUid(), Simulator::Now() );
   
   NS_LOG_DEBUG("\t @ " << Simulator::Now() << " ROUTER PKT TX Start id " << pkt->GetUid() << " sz " << pkt->GetSize() );
   
   // Add the new entry to the Router Packet map based on GUID
   m_routerPackets.push_back( nxtPkt );  
}

/******************************************************************************
 * Frame Processing Methods
 ******************************************************************************/

void Aggregator::FrameTxStartCallback( uint32_t nodeid, Ptr<Packet const> pkt ) {
   // First record the frame metadata for future processing
   TransmitFrame nxtFrame( pkt->GetSize(), pkt->GetUid(), Simulator::Now() );
   
   NS_LOG_DEBUG("\t @ " << Simulator::Now() << " frame TX Start at node " << nodeid << " id " << pkt->GetUid() << " len " << pkt->GetSize() );
   // Add the new entry to the Application Packet map based on GUID
   m_txMacFrames[nodeid].push_back( nxtFrame );
   
   /*
    * If there already exists an event in the queue, then we need to 
    * end it before we add the transmit event.
    */
   NodePhyMap::iterator itr = m_nodePhyEvents.find( nodeid );
   if ( ( itr != m_nodePhyEvents.end() ) && ( itr->second.size() > 0 ) ) {
      // TODO: Make sure the previous state was LISTENING or SLEEPING 
      // as we aren't supposed to cut off an active reception
      m_nodePhyEvents[nodeid].back().StopEvent( Simulator::Now() );
   }
   else if ( itr == m_nodePhyEvents.end() ) {
      // This is the first event for said node - so add a listening state from time 0
      PhyStateEvent listenEvnt = { PhyStateEvent::PhyState_e::LISTENING, ns3::Time( 0.0 ) };
      listenEvnt.StopEvent( Simulator::Now() );
      m_nodePhyEvents[nodeid].push_back( listenEvnt );
   }
   
   // Now add a PHY event to the list of events
   PhyStateEvent nxtEvnt( PhyStateEvent::PhyState_e::TRANSMITTING, Simulator::Now() );
   m_nodePhyEvents[nodeid].push_back( nxtEvnt );
}

void Aggregator::FrameTxStopCallback( uint32_t nodeid, Ptr<Packet const> pkt ) {

   NS_LOG_DEBUG("\t @ " << Simulator::Now() << " frame TX stop on node " << nodeid << " id " << pkt->GetUid() << " len " << pkt->GetSize() );
   
   // First find the frame transmitted and set its end of time value
   TxFrameList::iterator itr = m_txMacFrames[nodeid].begin();
   
   for( ; itr != m_txMacFrames[nodeid].end(); ++itr ) {
      if ( (*itr).m_guid == pkt->GetUid() ) {
         (*itr).m_stop = Simulator::Now();
         (*itr).m_received = true;
         break;
      }
   }
   
   // Next find the PHY state event for the transmission, and end it
   m_nodePhyEvents[nodeid].back().StopEvent( Simulator::Now() );
   
   // Now start a new LISTENING state event
   // TODO: make this optional with SLEEPING for other MACs
   PhyStateEvent listenEvnt( PhyStateEvent::PhyState_e::LISTENING, Simulator::Now() );
   m_nodePhyEvents[nodeid].push_back( listenEvnt );
   
}

void Aggregator::FrameRxStartCallback( uint32_t nodeid, Ptr<Packet const> pkt ) {
   /*
    * If there already exists an event in the queue, then we need to 
    * end it before we add the active receiver event.
    */
   NodePhyMap::iterator itr = m_nodePhyEvents.find( nodeid );
   if ( ( itr != m_nodePhyEvents.end() ) && ( itr->second.size() > 0 ) ) {
      // TODO: Make sure the previous state was LISTENING or SLEEPING 
      // as we aren't supposed to cut off an active reception
      m_nodePhyEvents[nodeid].back().StopEvent( Simulator::Now() );
   }
   else if ( itr == m_nodePhyEvents.end() ) {
      // This is the first event for said node - so add a listening state from time 0
      PhyStateEvent listenEvnt( PhyStateEvent::PhyState_e::LISTENING, ns3::Time( 0.0 ) );
      listenEvnt.StopEvent( Simulator::Now() );
      m_nodePhyEvents[nodeid].push_back( listenEvnt );
   }
   
   // Now add a PHY event to the list of events
   PhyStateEvent nxtEvnt( PhyStateEvent::PhyState_e::RECEIVING, Simulator::Now() );
   m_nodePhyEvents[nodeid].push_back( nxtEvnt );
}

void Aggregator::FrameRxStopCallback( uint32_t nodeid, Ptr<Packet const> pkt ) {
   // Next find the PHY state event for the transmission, and end it
   m_nodePhyEvents[nodeid].back().StopEvent( Simulator::Now() );
   
   // Now start a new LISTENING state event
   // TODO: make this optional with SLEEPING for other MACs
   PhyStateEvent listenEvnt( PhyStateEvent::PhyState_e::LISTENING, Simulator::Now() );
   m_nodePhyEvents[nodeid].push_back( listenEvnt );
}

/******************************************************************************
 * Data Plot Processing Method
 ******************************************************************************/

void Aggregator::ExtractData( double maxTime, double scale ) {
   
   NS_LOG_DEBUG("\n\n Extracting data from " << m_txMacFrames.size() << " nodes in system with scale = " << scale << " sec" );
   
   m_currentScale = scale;
   
   // Generate the data from the various lists
   for ( double i = 0.0; i < maxTime; i += scale ) {
      OutputRow tmpRow = { 0 };
      tmpRow.time = i;

      double endpt = ( i + scale );

      NS_LOG_DEBUG("  Time slice: " << (double)i << "s to " << (double)endpt << "s ");
      // First collect the total number of bytes transmitted on the PHY of all nodes
      for ( TxFrameMap::iterator mapItr = m_txMacFrames.begin(); 
            mapItr != m_txMacFrames.end(); 
            ++mapItr ) {
         
         NS_LOG_DEBUG("\teval node " << mapItr->first );
      
         for( TxFrameList::iterator fitr = mapItr->second.begin(); 
               fitr != mapItr->second.end(); 
               ++fitr ) {
            double start = (*fitr).m_start.GetSeconds();
            double stop = (*fitr).m_stop.GetSeconds();
                        
            NS_LOG_DEBUG("\t  check: start " << start << " stop " << stop );
         
            // If the current frame is no where near the desired interval
            // move on to the next immediately
            if ( stop < i ) 
               continue;

            NS_LOG_DEBUG("\t\t  frm: start " << start << " stop " << stop );
            if ( ( start >= i ) && ( start < endpt ) ) {
            
               // First calc based on entire frame within the interval
               if ( stop <= endpt ) {
                  tmpRow.totalBytes += (double)((*fitr).m_bytes);
               }
               else {
                  // now calc a partial frame
                  double percent = (endpt - start) / (stop - start);
                  tmpRow.totalBytes += ((double)((*fitr).m_bytes) * percent);
               }
            }
            else if ( ( stop > i ) && ( start < i ) ) {
               if ( stop > endpt )
                  NS_LOG_DEBUG("packet larger than plot interval!\n");
               else {
                  // now calc a partial frame
                  double percent = (stop - i) / (stop - start);
                  tmpRow.totalBytes += ((double)((*fitr).m_bytes) * percent);
               }
            }
            else if ( start > endpt )
               // We reached a TX Frame beyond our current interval, so exit the loop
               break;
         }
      }
      
      // Now calculate the bandwidth for said scale
      tmpRow.totalBw = ( tmpRow.totalBytes > 0.0 ) ? (tmpRow.totalBytes / scale) : 0.0;
      
      // Now we calculate the total user bytes transmitted - as well as packet loss
      TxPacketList::iterator ditr = m_dataPackets.begin();
      for( ; ditr != m_dataPackets.end(); ++ditr ) {
         double start = (*ditr).m_start.GetSeconds();
         double stop = (*ditr).m_stop.GetSeconds();
         bool   lost = !((*ditr).m_received);
                     
         // If the current frame is no where near the desired interval
         // move on to the next immediately
         if ( stop < i ) 
            continue;

         if ( ( start >= i ) && ( start < endpt ) ) {
         
            if ( lost ) {
//                  printf("packet %u, sz %u was lost\n", (*ditr).m_guid, (*ditr).m_bytes );
               NS_LOG_DEBUG("packet " << (*ditr).m_guid << ", sz " << (*ditr).m_bytes << " was lost" );
            }
            
            // First calc based on entire frame within the interval
            if ( stop <= endpt ) {
               tmpRow.userBytes += (double)((*ditr).m_bytes);
               
               if (lost)
                  tmpRow.lostData += (double)((*ditr).m_bytes);
            }
            else {
               // now calc a partial frame
               double percent = (endpt - start) / (stop - start);
               tmpRow.userBytes += ((double)((*ditr).m_bytes) * percent);
               
               if (lost)
                  tmpRow.lostData += ((double)((*ditr).m_bytes) * percent);
            }
         }
         else if ( ( stop > i ) && ( start < i ) ) {
            if ( stop > endpt )
               NS_LOG_DEBUG("packet larger than plot interval!");
            else {
               // now calc a partial frame
               double percent = (stop - i) / (stop - start);
               tmpRow.userBytes += ((double)((*ditr).m_bytes) * percent);
               
               if (lost)
                  tmpRow.lostData += ((double)((*ditr).m_bytes) * percent);
            }
         }
         else if ( start > endpt )
            // We reached a TX Frame beyond our current interval, so exit the loop
            break;
      }
      
      // Now calculate the bandwidth for said scale
      tmpRow.userBw = (tmpRow.userBytes > 0.0) ? (tmpRow.userBytes / scale) : 0.0;
      tmpRow.lostBw = (tmpRow.lostData > 0.0) ? (tmpRow.lostData / scale) : 0.0;
      
      // Now we calculate the total routing bytes transmitted
      TxPacketList::iterator ritr = m_routerPackets.begin();
      for( ; ritr != m_routerPackets.end(); ++ritr ) {
         double start = (*ritr).m_start.GetSeconds();
                     
         // If the current packet is no where near the desired interval
         // move on to the next immediately
         if ( start < i ) 
            continue;

         if ( ( start >= i ) && ( start < endpt ) ) {
         
            tmpRow.routingBytes += (double)((*ritr).m_bytes);
         }
         else if ( start > endpt )
            // We reached a TX Frame beyond our current interval, so exit the loop
            break;
      }
      
      
      // Now calculate the bandwidth for said scale
      tmpRow.routerBw = ( tmpRow.routingBytes > 0.0 ) ? (tmpRow.routingBytes / scale) : 0.0;
      
      // Now calculate percentages of BW
      tmpRow.percentData     = ( tmpRow.userBw > 0.0 ) ? (tmpRow.userBw / tmpRow.totalBw) : 0.0;
      tmpRow.percentRouting  = ( tmpRow.routerBw > 0.0 ) ? (tmpRow.routerBw / tmpRow.totalBw) : 0.0;
      tmpRow.percentLost     = ( tmpRow.lostBw > 0.0 ) ? (tmpRow.lostBw / tmpRow.totalBw) : 0.0;
      tmpRow.percentOverhead = ((tmpRow.totalBw - (tmpRow.lostBw + tmpRow.routerBw + tmpRow.userBw)) > 0.0 ) ? 
                                 (( tmpRow.totalBw - (tmpRow.lostBw + tmpRow.routerBw + tmpRow.userBw) ) / tmpRow.totalBw) : 0.0;
      
      m_plotData.push_back( tmpRow );
      
      tmpRow = { 0 };
   }
   
   /*
    * Now derive the summary data -
    * 
    * NOTE: Do not include the pre-data intervals in calculations.  Therefore, 
    * wee need to detect when the first non-0 user period begins, then start 
    * aggregating the data.
    */
   int entries = 0;
   bool noStartFound = true;

   // Clear out summary data from any previous extraction
   m_summaryData.agg_total_bw  = 0.0;
   m_summaryData.agg_user_bw   = 0.0;
   m_summaryData.agg_router_bw = 0.0;
   
   for ( PlotList::iterator itr = m_plotData.begin(); itr != m_plotData.end(); ++itr ) {
      if ( noStartFound ) {
         if ( itr->userBytes > 0.0 )
            noStartFound = false;
         else
            continue;
      }
      
      // Start was found, aggregate data
      ++entries;
      m_summaryData.agg_total_bw  += itr->totalBw;
      m_summaryData.agg_user_bw   += itr->userBw;
      m_summaryData.agg_router_bw += itr->routerBw;
   }
   
   // derive the averages
   m_summaryData.avg_total_bw  = ( m_summaryData.agg_total_bw  / (double)entries );
   m_summaryData.avg_user_bw   = ( m_summaryData.agg_user_bw   / (double)entries );
   m_summaryData.avg_router_bw = ( m_summaryData.agg_router_bw / (double)entries );
}
  
void Aggregator::PlotData( const char *filename ) {

   int fd = open( filename, O_RDWR | O_CREAT, S_IRWXU );
   
   if ( fd == -1 ) {
      printf(" Error opening plot file (%u) <%s>\n", errno, strerror(errno) );
   }
   else {
      
      // Now write the data to a file
      char buf[1024];
      const char *title = "Time,Total Bytes,Routing Bytes,User Bytes,Lost Bytes,Total BW,Routing BW,User BW,Lost BW,% User Data,% Routing,% Lost,% Overhead\n\0";
      
      write( fd, title, strlen( title ) );
      
      while ( m_plotData.size() > 0 ) {
         sprintf( buf, "%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.6f,%0.6f,%0.6f,%0.6f\n", 
               m_plotData.front().time,
               m_plotData.front().totalBytes,
               m_plotData.front().routingBytes,
               m_plotData.front().userBytes,
               m_plotData.front().lostData,
               m_plotData.front().totalBw,
               m_plotData.front().routerBw,
               m_plotData.front().userBw,
               m_plotData.front().lostBw,
               m_plotData.front().percentData,
               m_plotData.front().percentRouting,
               m_plotData.front().percentLost,
               m_plotData.front().percentOverhead
         );
         
         write( fd, buf, strlen( buf ) );
         m_plotData.pop_front();
      }
      
   }
   
   close( fd );
   printf(" CSV file written\n");
   
}
  
void Aggregator::PlotData( const char *filename, double maxTime, double scale ) {
   ExtractData( maxTime, scale );
   PlotData( filename );
}

void Aggregator::PlotItems( int num, ... ) {
   
   /* Initialize a variable arguments list */
   va_list valist;
   va_start( valist, num );
   
   int i = 0;
   const char *comma = ", \0";
   const char *eol = "\n\0";
   
   // The first argument should always be a file pointer
   int fd = va_arg( valist, int );
   ++i;
   
   if ( fd == -1 )
      printf( "ERROR: invalid file descriptor for output\n" );
   else {
      
      // File Descriptor is okay - so print the line submitted
      while( i < num ) {
         printf(" num %d i %d\n", num, i);
         PlotDataType_e type = (PlotDataType_e) va_arg( valist, int );
         ++i;
         
         switch ( type ) {
            case ROW_HEADER_STR:
            {
               char *str = (char *) (va_arg( valist, long ));
               ++i;
               if ( write( fd, str, strlen( str ) ) == -1 )
                  printf("ERROR: writing to FD %d: (%d) <%s>\n", fd, errno, strerror( errno ) );
               break;
            }
               
            case AGG_TOTAL_BW:
            {
               char data[250];
               sprintf( data, "%f", m_summaryData.agg_total_bw );
               
               if ( write( fd, data, strlen( data ) ) == -1 )
                  printf("ERROR: writing AGG TOT BW to FD %d: (%d) <%s>\n", fd, errno, strerror( errno ) );
               break;
            }
               
            case AGG_ROUTER_BW:
            {
               char data[250];
               sprintf( data, "%f", m_summaryData.agg_router_bw );

               if ( write( fd, data, strlen( data ) ) == -1 )
                  printf("ERROR: writing AGG RTR BW to FD %d: (%d) <%s>\n", fd, errno, strerror( errno ) );
               break;
            }
               
            case AGG_USER_BW:
            {
               char data[250];
               sprintf( data, "%f", m_summaryData.agg_user_bw );

               if ( write( fd, data, strlen( data ) ) == -1 )
                  printf("ERROR: writing AGG USR BW to FD %d: (%d) <%s>\n", fd, errno, strerror( errno ) );
               break;
            }
               
            case AVG_TOTAL_BW:
            {
               char data[250];
               sprintf( data, "%f", m_summaryData.avg_total_bw );

               if ( write( fd, data, strlen( data ) ) == -1 )
                  printf("ERROR: writing AVG TOT BW to FD %d: (%d) <%s>\n", fd, errno, strerror( errno ) );
               break;
            }
               
            case AVG_ROUTER_BW:
            {
               char data[250];
               sprintf( data, "%f", m_summaryData.avg_router_bw );

               if ( write( fd, data, strlen( data ) ) == -1 )
                  printf("ERROR: writing AVG RTR BW to FD %d: (%d) <%s>\n", fd, errno, strerror( errno ) );
               break;
            }
               
            case AVG_USER_BW:
            {
               char data[250];
               sprintf( data, "%f", m_summaryData.avg_user_bw );

               if ( write( fd, data, strlen( data ) ) == -1 )
                  printf("ERROR: writing AVG USR BW to FD %d: (%d) <%s>\n", fd, errno, strerror( errno ) );
               break;
            }
                              
            default:
               printf( "ERROR: Invalid PlotDataType_e type = %d\n", type );
               break;
         }
         
         // Write a comma to the output if there are more line items to print
         if ( i < num ) {
            if ( write( fd, comma, strlen( comma ) ) == -1 )
               printf("ERROR: writing COMMON to FD %d: (%d) <%s>\n", fd, errno, strerror( errno ) );
         }
         else
            if ( write( fd, eol, strlen( eol ) ) == -1 )
               printf("ERROR: writing COMMON to FD %d: (%d) <%s>\n", fd, errno, strerror( errno ) );
      }
   }
   
   /* clean up the reserved memory for valist */
   va_end( valist );
}


