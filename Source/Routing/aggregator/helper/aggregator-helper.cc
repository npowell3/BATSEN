/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include "aggregator-helper.h"
#include "ns3/core-module.h"
#include "ns3/aggregator.h"
#include "ns3/object.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/application-container.h"
#include "ns3/udp-echo-server.h"
#include "ns3/udp-echo-client.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-net-device.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/batmand-routing-protocol.h"
#include "ns3/olsr-routing-protocol.h"

namespace ns3 {
   
NS_LOG_COMPONENT_DEFINE ("AggregatorHelper");
   
void AggregatorHelper::Install (NodeContainer c) 
{
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      Install (*i);
    }
}

void AggregatorHelper::Install (Ptr<Node> node) 
{
  NodeCallback tmp = { node->GetId(), &m_aggregator };
  m_nodeList.push_back( tmp );
  
  Ptr<WifiNetDevice> wfnd = node->GetObject<WifiNetDevice> ();
  
  
  if ( wfnd != NULL ) {
    Ptr<WifiMac> themac = wfnd->GetMac();
    Ptr<WifiPhy> thephy = themac->GetWifiPhy();
   
    thephy->TraceConnectWithoutContext( "PhyRxBegin" , MakeCallback( &NodeCallback::FrameRxStartEvent , &( m_nodeList.back() ) ) );
    thephy->TraceConnectWithoutContext( "PhyRxEnd" , MakeCallback( &NodeCallback::FrameRxStopEvent , &( m_nodeList.back() ) ) );
    thephy->TraceConnectWithoutContext( "PhyTxBegin" , MakeCallback( &NodeCallback::FrameTxStartEvent , &( m_nodeList.back() ) ) );
    thephy->TraceConnectWithoutContext( "PhyTxEnd" , MakeCallback( &NodeCallback::FrameTxStopEvent , &( m_nodeList.back() ) ) );
  }
  else {
     NS_LOG_DEBUG("no wifi net device on node " << (int)(node->GetId()) );
     
     for ( uint32_t i = 0; i < node->GetNDevices(); ++i ) {
        Ptr<WifiNetDevice> wfnd = DynamicCast<WifiNetDevice> ( node->GetDevice(i) );
        
        if ( wfnd != NULL ) {
           NS_LOG_DEBUG("device " << (int)i << " is a wifi net device" );
           Ptr<WifiMac> themac = wfnd->GetMac();
           Ptr<WifiPhy> thephy = themac->GetWifiPhy();
            
           if ( thephy != NULL ) {
             thephy->TraceConnectWithoutContext( "PhyRxBegin" , MakeCallback( &NodeCallback::FrameRxStartEvent , &( m_nodeList.back() ) ) );
             thephy->TraceConnectWithoutContext( "PhyRxEnd" , MakeCallback( &NodeCallback::FrameRxStopEvent , &( m_nodeList.back() ) ) );
             thephy->TraceConnectWithoutContext( "PhyTxBegin" , MakeCallback( &NodeCallback::FrameTxStartEvent , &( m_nodeList.back() ) ) );
             thephy->TraceConnectWithoutContext( "PhyTxEnd" , MakeCallback( &NodeCallback::FrameTxStopEvent , &( m_nodeList.back() ) ) );
           }
           break;
        }
        else
           NS_LOG_DEBUG("device " << (int)i << " is not a wifi net device" );
     }
  }

  Ptr<Ipv4RoutingProtocol> proto = node->GetObject<Ipv4RoutingProtocol> ();
  if ( proto == NULL ) 
     NS_LOG_DEBUG(" no protocol attached to node");
  else {
     NS_LOG_DEBUG(" install routing protocol on node " << node->GetId() );
     proto->TraceConnectWithoutContext( "TxPkt" , MakeCallback( &NodeCallback::RouterPacket , &( m_nodeList.back() ) ) );
  }
}

/*
void AggregatorHelper::Install (std::string nodeName) 
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  Install (node);
}
*/

void AggregatorHelper::InstallUdpClient ( ApplicationContainer c )  
{
  for (ApplicationContainer::Iterator i = c.Begin (); i != c.End (); ++i)
   {
      InstallUdpClient ( DynamicCast<UdpEchoClient> (*i) );
   }
}


void AggregatorHelper::InstallUdpClient ( Ptr<UdpEchoClient> appptr )  
{
  Ptr<Node> ndptr = appptr->GetNode();
  NodeCallback *cb = FindNode( ndptr->GetId() );
  
  NS_LOG_DEBUG("Install UDP Client from node " << ndptr->GetId() << " node " << cb );
  NS_ASSERT( cb != NULL );
  
  if ( cb != NULL ) {
    appptr->TraceConnectWithoutContext( "Tx" , MakeCallback( &NodeCallback::PktTxEvent , cb ) );
    appptr->TraceConnectWithoutContext( "Rx" , MakeCallback( &NodeCallback::PktRxEvent , cb ) );  
  }
}


void AggregatorHelper::InstallUdpServer ( ApplicationContainer c )  
{
  for (ApplicationContainer::Iterator i = c.Begin (); i != c.End (); ++i)
   {
      InstallUdpServer ( DynamicCast<UdpEchoServer> (*i) );
   }
}


void AggregatorHelper::InstallUdpServer ( Ptr<UdpEchoServer> appptr )  
{
  Ptr<Node> ndptr = appptr->GetNode();
  NodeCallback *cb = FindNode( ndptr->GetId() );
  
  NS_LOG_DEBUG("Install UDP Server from node " << ndptr->GetId() << " node " << cb );
  NS_ASSERT( cb != NULL );
  
  if ( cb != NULL ) {
    appptr->TraceConnectWithoutContext( "Tx" , MakeCallback( &NodeCallback::PktTxEvent , cb ) );
    appptr->TraceConnectWithoutContext( "Rx" , MakeCallback( &NodeCallback::PktRxEvent , cb ) );  
  }
}


NodeCallback *AggregatorHelper::FindNode( uint32_t id ) 
{
   NodeCallback *cb = NULL;
   NdCbList::iterator itr;
   for ( itr = m_nodeList.begin(); itr != m_nodeList.end(); ++itr ) 
   {
      if ( itr->GetId() == id ) 
      {
         cb = &(*itr);
         break;
      }
   }
   
   return cb;
}

} // namespace ns3

