/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef AGGREGATOR_HELPER_H
#define AGGREGATOR_HELPER_H


#include "ns3/aggregator.h"
#include "ns3/node-container.h"
#include "ns3/application-container.h"
#include "ns3/udp-echo-server.h"
#include "ns3/udp-echo-client.h"

namespace ns3 {

/**
 * \brief Attach callbacks to node devices and applications for Packet/Frame analysis.
 */
class AggregatorHelper
{
public:
  
  /**
   * For each node in the provided container, create a NodeCallback entry
   * for the node's GUID in the m_nodeList. 
   *
   * \param c NodeContainer of the set of nodes to aggregate the 
   * ns3::PacketSocketFactory on.
   */
  void Install (NodeContainer c);
  
  /**
   * Add an individual node to the m_nodeList. 
   *
   * \param node Node on which to aggregate the ns3::PacketSocketFactory.
   */
  void Install (Ptr<Node> node);

  /**
   * Aggregate an instance of a ns3::PacketSocketFactory onto the provided
   * node.
   *
   * \param nodeName The name of the node on which to aggregate the ns3::PacketSocketFactory.
   */
//  void Install (std::string nodeName);

  void InstallUdpClient ( ApplicationContainer c );

  void InstallUdpClient ( Ptr<UdpEchoClient> appptr );
  
  void InstallUdpServer ( ApplicationContainer c );

  void InstallUdpServer ( Ptr<UdpEchoServer> appptr );

  Ptr<Aggregator> GetAggregator(void) { return &m_aggregator; };
  
  
private:
   
  NodeCallback *FindNode( uint32_t id );
  
  
  
  Aggregator        m_aggregator;
  
  NdCbList          m_nodeList;
  
};

}

#endif /* AGGREGATOR_HELPER_H */

