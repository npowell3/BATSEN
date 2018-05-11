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
 * Author: Nelson Powell <nhp8080@rit.edu>
 *
 * NOTE: Code based on OLSR library and modified for BATMAND-0.3.2
 * implementation.
 */

#include "batmand-helper.h"
#include "ns3/batmand-routing-protocol.h"
#include "ns3/node-list.h"
#include "ns3/names.h"
#include "ns3/ptr.h"
#include "ns3/ipv4-list-routing.h"

namespace ns3 {


BatmandHelper::BatmandHelper ()
{
  m_agentFactory.SetTypeId ("ns3::batmand::RoutingProtocol");
}

BatmandHelper::BatmandHelper (const BatmandHelper &o)
  : m_agentFactory (o.m_agentFactory)
{
  m_interfaceExclusions = o.m_interfaceExclusions;
}

BatmandHelper*
BatmandHelper::Copy (void) const
{
  return new BatmandHelper (*this);
}

void
BatmandHelper::ExcludeInterface (Ptr<Node> node, uint32_t interface)
{
  std::map< Ptr<Node>, std::set<uint32_t> >::iterator it = m_interfaceExclusions.find (node);

  if (it == m_interfaceExclusions.end ())
    {
      std::set<uint32_t> interfaces;
      interfaces.insert (interface);

      m_interfaceExclusions.insert (std::make_pair (node, std::set<uint32_t> (interfaces) ));
    }
  else
    {
      it->second.insert (interface);
    }
}

Ptr<Ipv4RoutingProtocol>
BatmandHelper::Create (Ptr<Node> node) const
{
  Ptr<batmand::RoutingProtocol> agent = m_agentFactory.Create<batmand::RoutingProtocol> ();

  std::map<Ptr<Node>, std::set<uint32_t> >::const_iterator it = m_interfaceExclusions.find (node);

  if (it != m_interfaceExclusions.end ())
    {
      agent->SetInterfaceExclusions (it->second);
    }

  node->AggregateObject (agent);
  return agent;
}

void
BatmandHelper::Set (std::string name, const AttributeValue &value)
{
  m_agentFactory.Set (name, value);
}

int64_t
BatmandHelper::AssignStreams (NodeContainer c, int64_t stream)
{
  int64_t currentStream = stream;
  Ptr<Node> node;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      node = (*i);
      Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
      NS_ASSERT_MSG (ipv4, "Ipv4 not installed on node");
      Ptr<Ipv4RoutingProtocol> proto = ipv4->GetRoutingProtocol ();
      NS_ASSERT_MSG (proto, "Ipv4 routing not installed on node");
      Ptr<batmand::RoutingProtocol> batmand = DynamicCast<batmand::RoutingProtocol> (proto);
      if (batmand)
        {
          currentStream += batmand->AssignStreams (currentStream);
          continue;
        }
      // Batmand may also be in a list
      Ptr<Ipv4ListRouting> list = DynamicCast<Ipv4ListRouting> (proto);
      if (list)
        {
          int16_t priority;
          Ptr<Ipv4RoutingProtocol> listProto;
          Ptr<batmand::RoutingProtocol> listBatmand;
          for (uint32_t i = 0; i < list->GetNRoutingProtocols (); i++)
            {
              listProto = list->GetRoutingProtocol (i, priority);
              listBatmand = DynamicCast<batmand::RoutingProtocol> (listProto);
              if (listBatmand)
                {
                  currentStream += listBatmand->AssignStreams (currentStream);
                  break;
                }
            }
        }
    }
  return (currentStream - stream);

}

}

