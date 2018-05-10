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

#ifndef SENSOR_H
#define SENSOR_H

#include <ns3/net-device.h>
#include <ns3/traced-callback.h>
#include <ns3/sensor-mac.h>

namespace ns3 {
  
class LrWpanPhy;
class SensorCsmaCa;
class SpectrumChannel;
class Node;

/**
 * \defgroup sensor Sensro Network models
 *
 * This section documents the API of the Sensor Network device models.  For a 
 * generic functional description, please refer to the ns-3 manual.
 */
  
/**
 *
 * \ingroup sensor
 *
 * Class that implements the top level API of the Sensor device. The sensor 
 * contains the MAC-PHY specified during initialization.  The MAC is variable 
 * while the PHY is IEEE 802.15.4-2006 from the lr-wpan module.
 */
class SensorNetDevice : public NetDevice {
    
public:
  
  /**
   * Get the type ID.
   *
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  
  /**
   * Default constructor.
   */
  SensorNetDevice(void);
  virtual ~SensorNetDevice(void);
  
  /**
   * Special constructor wherein the MAC is 
   * defined by the calling script.
   */
  SensorNetDevice(Ptr<SensorMac> p);
  
  /**
   * Set the underlying MAC for the Sensor Node.
   *
   * \param mac the MAC
   */
  void SetMac (Ptr<SensorMac> phy);

  /**
   * Set the PHY to be used by the MAC and this NetDevice.
   *
   * \param phy the PHY to be used
   */
  void SetPhy (Ptr<LrWpanPhy> phy);
  
  /**
   * Set the CSMA/CA implementation to be used by the MAC and this NetDevice.
   *
   * \param csmaca the CSMA/CA implementation to be used
   */
  void SetCsmaCa (Ptr<SensorCsmaCa> csmaca);
  
  /**
   * Set the channel to which the NetDevice, and therefore the PHY, should be
   * attached to.
   *
   * \param channel the channel to be used
   */
  void SetChannel (Ptr<SpectrumChannel> channel);
  
  /**
   * Get the underlying MAC of the Sensor Node.
   *
   * \return the MAC
   */
  Ptr<SensorMac> GetMac (void) const;
  
  /**
   * Get the type of the underlying MAC of the Sensor Node.
   *
   * \return the MacType_e
   */
  MacType_e GetMacType(void);

  /**
   * Get the PHY used by this NetDevice.
   *
   * \return the PHY object
   */
  Ptr<LrWpanPhy> GetPhy (void) const;
  
  /**
   * Get the CSMA/CA implementation used by this NetDevice.
   *
   * \return the CSMA/CA implementation object
   */
  Ptr<SensorCsmaCa> GetCsmaCa (void) const;
  
  /**
   * Sets the Sink state of the device.  There should
   * only be one sink in a wireless sensor net.
   *
   */
  void SetSinkStatus( bool amSink = false );
  
  // From class NetDevice
  virtual void SetIfIndex (const uint32_t index);
  virtual uint32_t GetIfIndex (void) const;
  virtual Ptr<Channel> GetChannel (void) const;
  /**
   * This method indirects to LrWpanMac::SetShortAddress ()
   */
  virtual void SetAddress (Address address);
  /**
   * This method indirects to LrWpanMac::SetShortAddress ()
   */
  virtual Address GetAddress (void) const;
  virtual bool SetMtu (const uint16_t mtu);
  virtual uint16_t GetMtu (void) const;
  virtual bool IsLinkUp (void) const;
  virtual void AddLinkChangeCallback (Callback<void> callback);
  virtual bool IsBroadcast (void) const;
  virtual Address GetBroadcast (void) const;
  virtual bool IsMulticast (void) const;
  virtual Address GetMulticast (Ipv4Address multicastGroup) const;
  virtual Address GetMulticast (Ipv6Address addr) const;
  virtual bool IsBridge (void) const;
  virtual bool IsPointToPoint (void) const;
  virtual bool Send (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber);
  virtual bool SendFrom (Ptr<Packet> packet, const Address& source, const Address& dest, uint16_t protocolNumber);
  virtual Ptr<Node> GetNode (void) const;
  virtual void SetNode (Ptr<Node> node);
  virtual bool NeedsArp (void) const;
  
  virtual void SetReceiveCallback (NetDevice::ReceiveCallback cb);
  virtual void SetPromiscReceiveCallback (PromiscReceiveCallback cb);
  virtual bool SupportsSendFrom (void) const;
  
  /**
   * The callback used by the MAC to hand over incoming packets to the
   * NetDevice. This callback will in turn use the ReceiveCallback set by
   * SetReceiveCallback() to notify upper layers.
   *
   * \param params 802.15.4 specific parameters, including source and destination addresses
   * \param pkt the packet do be delivered
   */
  void SensorDataIndication (SensorDataIndicationParams params, Ptr<Packet> pkt);

  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams that have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  int64_t AssignStreams (int64_t stream);
    
private:
  
  // Inherited from NetDevice/Object
  virtual void DoDispose (void);
  virtual void DoInitialize (void);
  
  /**
   * Mark NetDevice link as up.
   */
  void LinkUp (void);
  
  /**
   * Mark NetDevice link as down.
   */
  void LinkDown (void);
  
  /**
   * Attribute accessor method for the "Channel" attribute.
   *
   * \return the channel to which this NetDevice is attached
   */
  Ptr<SpectrumChannel> DoGetChannel (void) const;
  
  /**
   * Configure PHY, MAC and CSMA/CA.
   */
  void CompleteConfig (void);
  
  /**
   * The trace source fired when a packet is generated by the Sensor.
   * Data is generated by the pseudo application (internal representation)
   * and is added to a FIFO.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_dataEnqueueTrace;

  /**
   * The trace source fired when a packet is generated by the Sensor.
   * This occurs when a packet is pulled from the FIFO and pushed 
   * to the MAC for transmission.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_dataSentTrace;
  
  /**
   * The trace source fired when a packet is dropped by the Sensor.
   * If a drop queue or single packet queue is used, the drop
   * trace is used to track dropped transmit packets due to timeout.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_dataDropQueueTrace;
  
  /**
   * The trace source fired when a packet is received from the MAC.
   * The reception is recorded regardless if the data is store-and-
   * forwarded, or if it has reached the sink node.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_dataReceiveTrace;
  
  /**
   * The trace source fired when a packet is received by the Sink and
   * consumed.  This should only be called by a node defined as 
   * the sink for the wireless network.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_sinkDataTrace;
  
  /**
   * The MAC associated with this Sensor.
   */
  Ptr<SensorMac> m_mac;
  
  /**
   * The PHY for this NetDevice.
   */
  Ptr<LrWpanPhy> m_phy;
  
  /**
   * The CSMA/CA implementation for this NetDevice.
   */
  Ptr<SensorCsmaCa> m_csmaca;
  
  /**
   * The node associated with this NetDevice.
   */
  Ptr<Node> m_node;
  
  /**
   * True if MAC, PHY and CSMA/CA where successfully configured and the
   * NetDevice is ready for being used.
   */
  bool m_configComplete;
  
  /**
   * Configure the NetDevice to request MAC layer acknowledgments when sending
   * packets using the Send() API.
   */
  bool m_useAcks;

  /**
   * Is the link/device currently up and running?
   */
  bool m_linkUp;
  
  /**
   * The interface index of this NetDevice.
   */
  uint32_t m_ifIndex;
  
  /**
   * Trace source for link up/down changes.
   */
  TracedCallback<> m_linkChanges;
  
  /**
   * Upper layer callback used for notification of new data packet arrivals.
   */
  ReceiveCallback m_receiveCallback;
};
    
}

#endif /* SENSOR_H */

