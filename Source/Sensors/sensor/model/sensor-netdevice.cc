/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Rochester Institute of Technology
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
 *  Author: Nelson Powell <nhp8080@rit.edu>
 */

#include "sensor-netdevice.h"
#include "sensor-mac.h"
#include <ns3/lr-wpan-phy.h>
#include "sensor-csma.h"
#include <ns3/lr-wpan-error-model.h>
#include <ns3/abort.h>
#include <ns3/node.h>
#include <ns3/log.h>
#include <ns3/spectrum-channel.h>
#include <ns3/pointer.h>
#include <ns3/boolean.h>
#include <ns3/mobility-model.h>
#include <ns3/packet.h>

namespace ns3 {

    
NS_LOG_COMPONENT_DEFINE ("SensorNetDevice");
    
NS_OBJECT_ENSURE_REGISTERED (SensorNetDevice);
    
TypeId
SensorNetDevice::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SensorNetDevice")
  .SetParent<Object> ()
  .SetGroupName ("Sensor")
  .AddConstructor<SensorNetDevice> ()
  .AddAttribute ("Channel", "The channel attached to this device",
                 PointerValue (),
                 MakePointerAccessor (&SensorNetDevice::DoGetChannel),
                 MakePointerChecker<SpectrumChannel> ())
  .AddAttribute ("Phy", "The PHY layer attached to this device.",
                 PointerValue (),
                 MakePointerAccessor (&SensorNetDevice::GetPhy, &SensorNetDevice::SetPhy),
                 MakePointerChecker<LrWpanPhy> ())
  .AddAttribute ("Mac", "The MAC layer attached to this device.",
                 PointerValue (),
                 MakePointerAccessor (&SensorNetDevice::GetMac, &SensorNetDevice::SetMac),
                 MakePointerChecker<SensorMac> ())
  ;
  return tid;
}
    
SensorNetDevice::SensorNetDevice ()
    : m_configComplete (false)
{
  NS_LOG_FUNCTION (this);
  m_mac = CreateObject<SensorMac> ();
  m_phy = CreateObject<LrWpanPhy> ();
  m_csmaca = CreateObject<SensorCsmaCa> ();
  CompleteConfig ();
}
  
SensorNetDevice::SensorNetDevice (Ptr<SensorMac> p)
: m_configComplete (false)
{
  NS_LOG_FUNCTION (this);
  m_mac = p;
  m_phy = CreateObject<LrWpanPhy> ();
  m_csmaca = CreateObject<SensorCsmaCa> ();
  
  CompleteConfig ();
}
    
SensorNetDevice::~SensorNetDevice ()
{
  NS_LOG_FUNCTION (this);
}
    
void
SensorNetDevice::DoInitialize ()
{
  NS_LOG_FUNCTION (this);
  m_phy->Initialize ();
  m_mac->Initialize ();
  NetDevice::DoInitialize ();
  
  NS_LOG_DEBUG( "Kicking off the KickOffFSM" );
  m_mac->KickOffFSM();
}
  

void
SensorNetDevice::CompleteConfig (void)
{
  NS_LOG_FUNCTION (this);
  if (m_mac == 0 || m_phy == 0 || m_csmaca == 0 || m_node == 0 || m_configComplete)
  {
    return;
  }
  m_mac->SetPhy (m_phy);
  m_mac->SetCsmaCa (m_csmaca);
  m_mac->SetSensorDataIndicationCallback (MakeCallback (&SensorNetDevice::SensorDataIndication, this));
  m_csmaca->SetMac (m_mac);
  
  Ptr<MobilityModel> mobility = m_node->GetObject<MobilityModel> ();
  if (!mobility)
  {
    NS_LOG_WARN ("SensorNetDevice: no Mobility found on the node, probably it's not a good idea.");
  }
  m_phy->SetMobility (mobility);
  Ptr<LrWpanErrorModel> model = CreateObject<LrWpanErrorModel> ();
  m_phy->SetErrorModel (model);
  m_phy->SetDevice (this);
  
  m_phy->SetPdDataIndicationCallback (MakeCallback (&SensorMac::PdDataIndication, m_mac));
  m_phy->SetPdDataConfirmCallback (MakeCallback (&SensorMac::PdDataConfirm, m_mac));
  m_phy->SetPlmeEdConfirmCallback (MakeCallback (&SensorMac::PlmeEdConfirm, m_mac));
  m_phy->SetPlmeGetAttributeConfirmCallback (MakeCallback (&SensorMac::PlmeGetAttributeConfirm, m_mac));
  m_phy->SetPlmeSetTRXStateConfirmCallback (MakeCallback (&SensorMac::PlmeSetTRXStateConfirm, m_mac));
  m_phy->SetPlmeSetAttributeConfirmCallback (MakeCallback (&SensorMac::PlmeSetAttributeConfirm, m_mac));
  
  m_csmaca->SetSensorMacStateCallback (MakeCallback (&SensorMac::SetSensorMacState, m_mac));
  m_phy->SetPlmeCcaConfirmCallback (MakeCallback (&SensorCsmaCa::PlmeCcaConfirm, m_csmaca));
  m_configComplete = true;
  
  NS_LOG_DEBUG("new Net Device with PHY " << m_phy << " MAC " << m_mac << " addr " << m_mac->GetAddress() );

}
    
void
SensorNetDevice::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_mac->Dispose ();
  m_phy->Dispose ();
  m_csmaca->Dispose ();
  m_phy = 0;
  m_mac = 0;
  m_csmaca = 0;
  m_node = 0;
  // chain up.
  NetDevice::DoDispose ();
}
    
void
SensorNetDevice::SetMac (Ptr<SensorMac> mac)
{
  m_mac = mac;
}
    
void
SensorNetDevice::SetPhy (Ptr<LrWpanPhy> phy)
{
  NS_LOG_FUNCTION (this);
  m_phy = phy;
  CompleteConfig ();
}
    
void
SensorNetDevice::SetCsmaCa (Ptr<SensorCsmaCa> csmaca)
{
  NS_LOG_FUNCTION (this);
  m_csmaca = csmaca;
  CompleteConfig ();
}

void
SensorNetDevice::SetChannel (Ptr<SpectrumChannel> channel)
{
  NS_LOG_FUNCTION (this << channel);
  m_phy->SetChannel (channel);
  channel->AddRx (m_phy);
  CompleteConfig ();
}

Ptr<SensorMac>
SensorNetDevice::GetMac (void) const
{
  // NS_LOG_FUNCTION (this);
  return m_mac;
}
    
Ptr<LrWpanPhy>
SensorNetDevice::GetPhy (void) const
{
  NS_LOG_FUNCTION (this);
  return m_phy;
}
    
Ptr<SensorCsmaCa>
SensorNetDevice::GetCsmaCa (void) const
{
  NS_LOG_FUNCTION (this);
  return m_csmaca;
}
  
void
SensorNetDevice::SetSinkStatus( bool amSink )
{
  m_mac->SetSinkStatus( amSink );
}
    
void
SensorNetDevice::SetIfIndex (const uint32_t index)
{
  NS_LOG_FUNCTION (this << index);
  m_ifIndex = index;
}
    
uint32_t
SensorNetDevice::GetIfIndex (void) const
{
  NS_LOG_FUNCTION (this);
  return m_ifIndex;
}
    
Ptr<Channel>
SensorNetDevice::GetChannel (void) const
{
  NS_LOG_FUNCTION (this);
  return m_phy->GetChannel ();
}
    
void
SensorNetDevice::LinkUp (void)
{
  NS_LOG_FUNCTION (this);
  m_linkUp = true;
  m_linkChanges ();
}
    
void
SensorNetDevice::LinkDown (void)
{
  NS_LOG_FUNCTION (this);
  m_linkUp = false;
  m_linkChanges ();
}
    
Ptr<SpectrumChannel>
SensorNetDevice::DoGetChannel (void) const
{
  NS_LOG_FUNCTION (this);
  return m_phy->GetChannel ();
}
    
void
SensorNetDevice::SetAddress (Address address)
{
  NS_LOG_FUNCTION (this);
  m_mac->SetAddress (Mac16Address::ConvertFrom (address));
}
    
Address
SensorNetDevice::GetAddress (void) const
{
  NS_LOG_FUNCTION (this);
  return m_mac->GetAddress ();
}
    
bool
SensorNetDevice::SetMtu (const uint16_t mtu)
{
  NS_ABORT_MSG ("Unsupported");
  return false;
}
    
uint16_t
SensorNetDevice::GetMtu (void) const
{
  NS_LOG_FUNCTION (this);
  // Maximum payload size is: max psdu - frame control - seqno - addressing - security - fcs
  //                        = 127      - 2             - 1     - (2+2+2+2)  - 0        - 2
  //                        = 119
  // assuming no security and addressing with only 16 bit addresses without pan id compression.
  return 114;
}
    
bool
SensorNetDevice::IsLinkUp (void) const
{
  NS_LOG_FUNCTION (this);
  return m_phy != 0 && m_linkUp;
}

void
SensorNetDevice::AddLinkChangeCallback (Callback<void> callback)
{
  NS_LOG_FUNCTION (this);
  m_linkChanges.ConnectWithoutContext (callback);
}

bool
SensorNetDevice::IsBroadcast (void) const
{
  NS_LOG_FUNCTION (this);
  return true;
}

Address
SensorNetDevice::GetBroadcast (void) const
{
  NS_LOG_FUNCTION (this);
  return Mac16Address ("ff:ff");
}

bool
SensorNetDevice::IsMulticast (void) const
{
  NS_LOG_FUNCTION (this);
  return true;
}

Address
SensorNetDevice::GetMulticast (Ipv4Address multicastGroup) const
{
  NS_ABORT_MSG ("Unsupported");
  return Address ();
}

Address
SensorNetDevice::GetMulticast (Ipv6Address addr) const
{
  NS_LOG_FUNCTION (this);
  /* Implementation based on RFC 4944 Section 9.
   * An IPv6 packet with a multicast destination address (DST),
   * consisting of the sixteen octets DST[1] through DST[16], is
   * transmitted to the following 802.15.4 16-bit multicast address:
   *           0                   1
   *           0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
   *          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   *          |1 0 0|DST[15]* |   DST[16]     |
   *          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   * Here, DST[15]* refers to the last 5 bits in octet DST[15], that is,
   * bits 3-7 within DST[15].  The initial 3-bit pattern of "100" follows
   * the 16-bit address format for multicast addresses (Section 12). */
  
  // \todo re-add this once Lr-Wpan will be able to accept these multicast addresses
  //  uint8_t buf[16];
  //  uint8_t buf2[2];
  //
  //  addr.GetBytes(buf);
  //
  //  buf2[0] = 0x80 | (buf[14] & 0x1F);
  //  buf2[1] = buf[15];
  //
  //  Mac16Address newaddr = Mac16Address();
  //  newaddr.CopyFrom(buf2);
  //  return newaddr;
  
  return Mac16Address ("ff:ff");
}

bool
SensorNetDevice::IsBridge (void) const
{
  NS_LOG_FUNCTION (this);
  return false;
}

bool
SensorNetDevice::IsPointToPoint (void) const
{
  NS_LOG_FUNCTION (this);
  return false;
}
  
bool
SensorNetDevice::Send (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
  // This method basically assumes an 802.3-compliant device, but a raw
  // 802.15.4 device does not have an ethertype, and requires specific
  // McpsDataRequest parameters.
  // For further study:  how to support these methods somehow, such as
  // inventing a fake ethertype and packet tag for McpsDataRequest
  NS_LOG_FUNCTION (this << packet << dest << protocolNumber);
  
  if (packet->GetSize () > GetMtu ())
  {
    NS_LOG_ERROR ("Fragmentation is needed for this packet, drop the packet ");
    return false;
  }
  
  // TODO: NHP
  // Need to modify this to send to only one destination - the Sink MAC address of 0x01
  SensorDataRequestParams m_sensorDataRequestParams;
  m_sensorDataRequestParams.m_dstAddr = Mac16Address::ConvertFrom (dest);
  //m_sensorDataRequestParams.m_dstAddrMode = SENS_SHORT_ADDR;
  //m_sensorDataRequestParams.m_dstPanId = m_mac->GetPanId ();
  //m_sensorDataRequestParams.m_srcAddrMode = SENS_SHORT_ADDR;
  

  m_sensorDataRequestParams.m_msduHandle = 0;
  m_mac->SensorDataRequest (m_sensorDataRequestParams, packet);
  return true;
}
  
bool
SensorNetDevice::SendFrom (Ptr<Packet> packet, const Address& source, const Address& dest, uint16_t protocolNumber)
{
  NS_ABORT_MSG ("Unsupported");
  // TODO: To support SendFrom, the MACs McpsDataRequest has to use the provided source address, instead of to local one.
  return false;
}
  
Ptr<Node>
SensorNetDevice::GetNode (void) const
{
  NS_LOG_FUNCTION (this);
  return m_node;
}

void
SensorNetDevice::SetNode (Ptr<Node> node)
{
  NS_LOG_FUNCTION (this);
  m_node = node;
  CompleteConfig ();
}
  
bool
SensorNetDevice::NeedsArp (void) const
{
  NS_LOG_FUNCTION (this);
  return false;
}
  
void
SensorNetDevice::SetReceiveCallback (ReceiveCallback cb)
{
  NS_LOG_FUNCTION (this);
  m_receiveCallback = cb;
}
  
void
SensorNetDevice::SetPromiscReceiveCallback (PromiscReceiveCallback cb)
{
  // This method basically assumes an 802.3-compliant device, but a raw
  // 802.15.4 device does not have an ethertype, and requires specific
  // SensorDataIndication parameters.
  // For further study:  how to support these methods somehow, such as
  // inventing a fake ethertype and packet tag for McpsDataRequest
  NS_LOG_WARN ("Unsupported; use LrWpan MAC APIs instead");
}

void
SensorNetDevice::SensorDataIndication (SensorDataIndicationParams params, Ptr<Packet> pkt)
{
  NS_LOG_FUNCTION (this);
  // TODO: Use the PromiscReceiveCallback if the MAC is in promiscuous mode.
  m_receiveCallback (this, pkt, 0, params.m_srcAddr);
}

bool
SensorNetDevice::SupportsSendFrom (void) const
{
  NS_LOG_FUNCTION_NOARGS ();
  return false;
}

int64_t
SensorNetDevice::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (stream);
  int64_t streamIndex = stream;
  streamIndex += m_csmaca->AssignStreams (streamIndex);
  streamIndex += m_phy->AssignStreams (streamIndex);
  NS_LOG_DEBUG ("Number of assigned RV streams:  " << (streamIndex - stream));
  return (streamIndex - stream);
}

}

