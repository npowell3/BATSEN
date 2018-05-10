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

#ifndef SENSOR_MAC_H
#define SENSOR_MAC_H

#include <ns3/log.h>
#include <ns3/object.h>
#include <ns3/traced-callback.h>
#include <ns3/traced-value.h>
#include <ns3/mac16-address.h>
#include <ns3/mac64-address.h>
#include <ns3/sequence-number.h>
#include <ns3/lr-wpan-phy.h>
#include <ns3/event-id.h>
#include <deque>


namespace ns3 {
    
class Packet;
class SensorCsmaCa;
  
#define HIGH_POWER    -1
#define MEDIUM_POWER  -10
#define LOW_POWER     -19
  
/**
 * \defgroup sensor Sensor MAC models
 *
 * This section documents the API of the IEEE 802.15.4-related models.  For a generic functional description, please refer to the ns-3 manual.
 */

/**
 *
 * \ingroup sensor
 *
 * MAC protocol types
 */
typedef enum {
  LEACH_O   = 0,      // Original LEACH protocol
  //    LEACH_E,      // Energy efficient LEACH
  //    LEACH_M,      // Mobility LEACH
  BATSEN    = 1      // BATMAN Sensor network protocol
} MacType_e;

/**
 * \ingroup sensor
 *
 * MAC states
 */
typedef enum
{
  SENSOR_IDLE,              //!< MAC_IDLE
  SENSOR_CSMA,              //!< MAC_CSMA
  SENSOR_FORCE_TX,          //!< Force a TX without CSMA/CA prior
  SENSOR_SENDING,           //!< MAC_SENDING
  SENSOR_ACK_PENDING,       //!< MAC_ACK_PENDING
  SENSOR_CHN_ACS_FAIL,      //!< CHANNEL_ACCESS_FAILURE
  SENSOR_CHN_IDLE,          //!< CHANNEL_IDLE
  SENSOR_SET_PHY_TX_ON,     //!< SET_PHY_TX_ON
  SENSOR_LAST_ENUM
} SensorChnState;
  
const char g_chan_state[SENSOR_LAST_ENUM][20] = {
  {"MAC IDLE"},
  {"MAC CSMA"},
  {"MAC FORCE TX"},
  {"MAC SENDING"},
  {"MAC ACK PND"},
  {"CHN ACS FAIL"},
  {"CHN IDLE"},
  {"SET PHY TX ON"}
};

/**
 * \ingroup sensor
 *
 * table 80 of 802.15.4
 *
 * NOTE: the bare bones Sensor does not require Bluetooth info, so only 
 *       the short address or extended address will be used.
 */
typedef enum
{
  BT_NO_PANID_ADDR = 0,
  BT_ADDR_RESERVED = 1,
  SENS_SHORT_ADDR = 2,
  SENS_EXT_ADDR = 3
} SensorAddressMode;

  
/**
 * \ingroup sensor
 *
 * Table 42 of 802.15.4-2006
 */
typedef enum
{
  SENSOR_SUCCESS                = 0,
  SENSOR_TRANSACTION_OVERFLOW   = 1,
  SENSOR_TRANSACTION_EXPIRED    = 2,
  SENSOR_CHANNEL_ACCESS_FAILURE = 3,
  SENSOR_INVALID_ADDRESS        = 4,
  SENSOR_INVALID_GTS            = 5,
  SENSOR_NO_ACK                 = 6,
  SENSOR_COUNTER_ERROR          = 7,
  SENSOR_FRAME_TOO_LONG         = 8,
  SENSOR_UNAVAILABLE_KEY        = 9,
  SENSOR_UNSUPPORTED_SECURITY   = 10,
  SENSOR_INVALID_PARAMETER      = 11
} SensorDataConfirmStatus;
  
/**
 * \ingroup Sensor
 *
 * Replaces: MCPS-DATA.request params. See 7.1.1.1
 *
 * New data request format for generic sensors using home brewed
 * MACs but 802.15.4 PHY. For Sensor networks, there is no destination
 * other than the Sink node (MAC ID 0x0001 short address).  All
 * other transmissions are Layer 2 LLC frames from the child class.
 */
struct SensorDataRequestParams
{
  SensorDataRequestParams () : 
    m_dstAddr( ),
    m_msduHandle( 0 ),
    m_pktSize( 50 ),
    m_seqNumber( 0 )
  {
  }
  Mac16Address m_dstAddr;          //!< Destination address
  uint8_t m_msduHandle;            //!< MSDU handle
  uint16_t m_pktSize;              //!< Size of packet to generate
  uint32_t m_seqNumber;            //!< Sequence number for the next Packet
};
  
/**
 * \ingroup sensor
 *
 * MCPS-DATA.confirm params. See 7.1.1.2
 */
struct SensorDataConfirmParams
{
  uint8_t m_msduHandle; //!< MSDU handle
  SensorDataConfirmStatus m_status; //!< The status of the last MSDU transmission
};

/**
 * \ingroup sensor
 *
 * MCPS-DATA.indication params. See 7.1.1.3
 */
struct SensorDataIndicationParams
{
  Mac16Address m_srcAddr; //!< Source address
  Mac16Address m_dstAddr; //!< Destination address
  uint8_t m_mpduLinkQuality;  //!< LQI value measured during reception of the MPDU
  uint8_t m_dsn;          //!< The DSN of the received data frame
};

/**
 * \ingroup sensor
 *
 * This callback is called after a SensorDataRequestParams has been called from
 * the higher layer.  It returns a status of the outcome of the
 * transmission request
 */
typedef Callback<void, SensorDataConfirmParams> SensorDataConfirmCallback;
  
/**
 * \ingroup sensor
 *
 * This callback is called after a Mcps has successfully received a
 *  frame and wants to deliver it to the higher layer.
 *
 *  \todo for now, we do not deliver all of the parameters in section
 *  7.1.1.3.1 but just send up the packet.
 */
typedef Callback<void, SensorDataIndicationParams, Ptr<Packet> > SensorDataIndicationCallback;

/**
 * \ingroup sensor
 *
 * This callback is called after a Sink has successfully received a
 *  frame needs to report the results to the SensorHelper for analysis.
 *
 * @param status the status of ED
 * @param energyLevel the energy level of ED
 */
typedef Callback< void, std::list<uint16_t>, std::list<uint32_t> > SinkDataRcvdCallback;
  
/**
 * \ingroup sensor
 *
 * This callback is called after a Sink has successfully received a
 *  frame needs to report the results to the SensorHelper for analysis.
 *
 * @param status the status of ED
 * @param energyLevel the energy level of ED
 */
typedef Callback< void, Mac16Address, uint32_t > SinkDCRcvdCallback;
  
/**
 * \ingroup sensor
 *
 * This callback is called after a forwarding node has successfully
 *  received a frame and must deliver it to the SensorHelper for 
 *  statical analysis.
 *
 * @param MAC Address the CH received from
 * @param Sequence number received
 */
typedef Callback< void, Mac16Address, uint32_t > CHDataRcvdCallback;

/**
 * \ingroup sensor
 *
 * This callback is called after a node dies due to lack of power.
 *
 * @param status the status of ED
 * @param energyLevel the energy level of ED
 */
typedef Callback< void, Mac16Address > SensorDeadCallback;

/**
 * \ingroup sensor
 *
 * This callback is called when a node starts to transmit its data frame.
 *
 * @param uint32_t  sequence number of the data frame being transmitted
 */
typedef Callback< void, uint32_t, Mac16Address > SensorDataTransmitCallback;

/**
 * \ingroup sensor
 *
 * This callback is called when a node officially begins operation as a
 * Forwarder (a CH in LEACH). On a per-MAC basis, sometimes, the status
 * is delayed until forwader selection is confirmed.
 *
 * @param Mac16Address  Address of the node acting as a CH / Forwarder
 */
typedef Callback< void, Mac16Address > SensorForwarderSelectedCallback;

/**
 * \ingroup sensor
 *
 * This callback is called when a node decides to operate in a Direct 
 * Connection mode with the SINK.  This is normally due to a lack of
 * forwarder in between the node and the SINK for a variety of reasons.
 *
 * @param Mac16Address  Address of the node operating in direct transmission
 */
typedef Callback< void, Mac16Address > SensorDirectSelectCallback;

/**
 * \ingroup sensor
 *
 * This callback is called when when the SINK determines it is time for 
 * the next epohc (round) to begin. A 32 bit value must be used to support
 * MACs that execute nearly indefinitely.
 *
 * @param uint32_t  Epoch number starting at 0 
 */
typedef Callback< void, int > SensorNextRoundCallback;

/**
 * \ingroup sensor
 *
 * Class that implements the base class SensorMac state machine
 */
class SensorMac : public Object
{
public:
  /**
   * Get the type ID.
   *
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
    
  /**
   * The minimum number of octets added by the MAC sublayer to the PSDU.
   * See IEEE 802.15.4-2006, section 7.4.1, Table 85
   */
  static const uint32_t aMinMPDUOverhead;
    
  /**
   * Default constructor.
   */
  SensorMac (void);
  virtual ~SensorMac (void);
  
  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  virtual int64_t AssignStreams (int64_t stream);
  
  // XXX these setters will become obsolete if we use the attribute system
  /**
   * Set the short address of this MAC.
   *
   * \param address the new address
   */
  virtual void SetAddress (Mac16Address address);
  
  /**
   * Get the short address of this MAC.
   *
   * \return the short address
   */
  Mac16Address GetAddress (void) const;
  
  /**
   *  IEEE 802.15.4-2006, section 7.1.1.1
   *  MCPS-DATA.request
   *  Request to transfer a MSDU.
   *
   *  \param params the request parameters
   *  \param p the packet to be transmitted
   */
  virtual void SensorDataRequest (SensorDataRequestParams params, Ptr<Packet> p) {};

  /**
   *  Request to transfer a Data Unit - not compatible with IEEE.
   *
   *  \param params the request parameters
   */
  virtual void SensorDataRequest (SensorDataRequestParams params ) {};
  
  /**
   * Set the CSMA/CA implementation to be used by the MAC.
   *
   * \param csmaCa the CSMA/CA implementation
   */
  void SetCsmaCa (Ptr<SensorCsmaCa> csmaCa);
  
  /**
   * Set the underlying PHY for the MAC.
   *
   * \param phy the PHY
   */
  void SetPhy (Ptr<LrWpanPhy> phy);
  
  /**
   * Set the starting power level for this node
   * 
   * \param double  Starting power level in joules
   */
  void SetPower( double pwr ) { m_totalSystemPower = pwr; };
  
  /**
   * Get the underlying PHY of the MAC.
   *
   * \return the PHY
   */
  Ptr<LrWpanPhy> GetPhy (void);
  
  /**
   * Set the callback for the indication of an incoming data packet.
   * The callback implements MCPS-DATA.indication SAP of IEEE 802.15.4-2006,
   * section 7.1.1.3.
   *
   * \param c the callback
   */
  void SetSensorDataIndicationCallback (SensorDataIndicationCallback c);
  
  /**
   * Set the callback for the confirmation of a data transmission request.
   * The callback implements MCPS-DATA.confirm SAP of IEEE 802.15.4-2006,
   * section 7.1.1.2.
   *
   * \param c the callback
   */
  void SetSensorDataConfirmCallback (SensorDataConfirmCallback c);
  
  /**
   * Set the callback for a MAC to provide a Sensor Data reception event.
   * This event is triggered when a forwarding node (or in the case of 
   * LEACH, a CH node) receives a data payload.  This is not used when 
   * a sink receives the data.
   *
   * \param c the callback
   */
  void SetCHDataRcvdCallback( CHDataRcvdCallback c )
  {
    m_sensorCHDataRcvdCallback = c;
  };

  /**
   * Set the callback for a MAC to provide a Sensor Data reception event.
   * This event is triggered only when a SINK receives a data payload. 
   * In accordance with the LEACH protocol paper, it is assumed that the 
   * sink receives a single payload for multiple sources in a single 
   * packet due to in network processing.  So the callback must have a 
   * list of source nodes from which the data is forwarded.
   *
   * \param c the callback
   */
  void SetSinkDataRcvdCallback( SinkDataRcvdCallback c )
  {
    m_sensorSinkDataRcvdCallback = c;
  };
  
  /**
   * Set the callback for a MAC to provide a Sensor Data reception event.
   * This event is triggered only when a SINK receives a data payload.
   * In accordance with the LEACH protocol paper, it is assumed that the
   * sink receives a single payload for multiple sources in a single
   * packet due to in network processing.  So the callback must have a
   * list of source nodes from which the data is forwarded.
   *
   * \param c the callback
   */
  void SetSinkDCRcvdCallback( SinkDCRcvdCallback c )
  {
    m_sensorSinkDCRcvdCallback = c;
  };
  
  /**
   * Set the callback for a sensor goign dark due to power outage. 
   * The SensorHelper or calling script may need to track when a 
   * node dies due to a lack of power.  We use this event to detect
   * when a net "fails" to track performance between different MACs.
   */
  void SetSensorDeadCallback( SensorDeadCallback c )
  {
    m_sensorDeadCallback = c;
  };
  
  /**
   * Set the callback for a sensor going dark due to power outage. 
   * The SensorHelper or calling script may need to track when a 
   * node dies due to a lack of power.  We use this event to detect
   * when a net "fails" to track performance between different MACs.
   */
  void SetSensorDataTransmitCallback( SensorDataTransmitCallback c )
  {
    m_sensorDataTransmitCallback = c;
  };  
  
  /**
   * Set the callback for a sensor becoming a forwarder (CH). 
   * The SensorHelper collects the number of node operating as forwarders 
   * or Cluster Heads per Epoch (LEACH round), so that we can evaluate
   * the number of nodes per round selected as forwarders. LEACH
   * says that 5% is the optimal.
   */
  void SetSensorForwarderSelectedCallback( SensorForwarderSelectedCallback c )
  {
    m_sensorForwaderCallback = c;
  }; 
  
  /**
   * Set the callback for a sensor going dark due to power outage.
   * The SensorHelper collects statistics on the number of nodes that 
   * fail to participate in a forwarding algorithm. In this case, the
   * node must resort to sending its data to the SINK directly, which
   * is inefficient. The measure of DT nodes also measures the 
   * effectiveness of the LEACH MAC or other MAC protocol defined.
   */
  void SetSensorDirectSelectCallback( SensorDirectSelectCallback c )
  {
    m_sensorDirectCallback = c;
  }; 
  
  /**
   * Set the callback for a sensor for the start of the next round. 
   * The SensorHelper needs to know the start of the next routing round.
   * This information is used to discretely track the number of 
   * forwarders and DT nodes per round.
   */
  void SetSensorNextRoundCallback( SensorNextRoundCallback c )
  {
    m_sensorNextRoundCallback = c;
  }; 
  
  /**
   * This method sets the sink status of the node. This is normally not
   * called, as all but one node should be regular member nodes. The 
   * default method asserts the m_iAmSink as FALSE to avoid excessive 
   * unintentional calls.
   */
  void SetSinkStatus( bool amSink = false );
  
  /**
   * Get the Sink status for the node.  Return TRUE if the node is the
   * Sink for the network.  Return FALSE otherwise.
   * 
   * \returns bool  TRUE if Sink, FALSE otherwise
   */
  bool GetSinkStatus() { return m_iAmSink; };
  
  /**
   * Get the Power status of the sensor.  Return TRUE if the node
   * still has power avaialble to operate in the network.
   *
   * \returns bool  TRUE if power availabe, FALSE if node dead
   */
  bool NodeAlive() { return m_systemEnabled; };
  
  /**
   * Use this method to kill a node - inparticular, this method is
   * used to shutdown the SINK after all other nodes are dead. This 
   * method should not be called on a non-SINK node. I put protections
   * in just in case it is done by accident,
   */
  void KillNode() 
  { 
    if ( m_iAmSink )
      m_systemEnabled = false;
    
    NodeJustDied();
  };
  
  /*
   * Kick off the FSM enabling communications on the MAC-PHY. Each MAC will
   * need to perform its own channel access methods before a user-app
   * can proceed with data transmissions.
   *
   * This method is pure virtual.  It is specific to the MAC implemented.
   *
   */
  virtual void KickOffFSM( void ) {};
  
  /*
   * This method will force the implementation class to cancel all 
   * timers. This should be called from the dead node callback 
   * function so that we abort any future processes.
   * 
   * NOTE: Calling it from within for now.
   */
  virtual void NodeJustDied( void ) {};
  
  // interfaces between MAC and PHY
  /**
   *  IEEE 802.15.4-2006 section 6.2.1.3
   *  PD-DATA.indication
   *  Indicates the transfer of an MPDU from PHY to MAC (receiving)
   *  @param psduLength number of bytes in the PSDU
   *  @param p the packet to be transmitted
   *  @param lqi Link quality (LQI) value measured during reception of the PPDU
   */
  virtual void PdDataIndication (uint32_t psduLength, Ptr<Packet> p, uint8_t lqi) {};
  
  /**
   *  IEEE 802.15.4-2006 section 6.2.2.2
   *  PLME-CCA.confirm status
   *  @param status TRX_OFF, BUSY or IDLE
   */
  virtual void PlmeCcaConfirm (LrWpanPhyEnumeration status) {};
  
  /**
   *  IEEE 802.15.4-2006 section 6.2.2.4
   *  PLME-ED.confirm status and energy level
   *  @param status SUCCESS, TRX_OFF or TX_ON
   *  @param energyLevel 0x00-0xff ED level for the channel
   */
  virtual void PlmeEdConfirm (LrWpanPhyEnumeration status, uint8_t energyLevel) {};
  
  /**
   *  IEEE 802.15.4-2006 section 6.2.2.6
   *  PLME-GET.confirm
   *  Get attributes per definition from Table 23 in section 6.4.2
   *  @param status SUCCESS or UNSUPPORTED_ATTRIBUTE
   *  @param id the attributed identifier
   *  @param attribute the attribute value
   */
  virtual void PlmeGetAttributeConfirm (LrWpanPhyEnumeration status,
                                        LrWpanPibAttributeIdentifier id,
                                        LrWpanPhyPibAttributes* attribute) {};
  
  /**
   *  IEEE 802.15.4-2006 section 6.2.2.8
   *  PLME-SET-TRX-STATE.confirm
   *  Set PHY state
   *  @param status in RX_ON,TRX_OFF,FORCE_TRX_OFF,TX_ON
   */
  virtual void PlmeSetTRXStateConfirm (LrWpanPhyEnumeration status) {};
  
  /**
   *  IEEE 802.15.4-2006 section 6.2.2.10
   *  PLME-SET.confirm
   *  Set attributes per definition from Table 23 in section 6.4.2
   *  @param status SUCCESS, UNSUPPORTED_ATTRIBUTE, INVALID_PARAMETER, or READ_ONLY
   *  @param id the attributed identifier
   */
  virtual void PlmeSetAttributeConfirm (LrWpanPhyEnumeration status,
                                        LrWpanPibAttributeIdentifier id) {};
  
  /**
   *  IEEE 802.15.4-2006 section 6.2.1.2
   *  Confirm the end of transmission of an MPDU to MAC
   *  @param status to report to MAC
   *  PHY PD-DATA.confirm status
   */
  virtual void PdDataConfirm (LrWpanPhyEnumeration status) {};
  
  /**
   * CSMA-CA algorithm calls back the MAC after executing channel assessment.
   *
   * \param macState indicate BUSY oder IDLE channel condition
   */
  virtual void SetSensorMacState (SensorChnState macState) {};
  
  void SetChannel( int chan );
  
  void SetPower( int pwr );
  
  /**
   * Indication of whether the MAC sublayer is to enable its receiver during
   * idle periods.
   * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
   */
  bool m_macRxOnWhenIdle;
  
  /**
   * Sink status for a given node. A node designated as a sink
   * must have this variable set.  It is set to FALSE by default
   * to avoid possibility of multiple sinks.
   */
  bool m_iAmSink;
  bool m_csmaOk;
  LrWpanPhyPibAttributes m_PhyParams;

protected:
  // Inherited from Object.
  virtual void DoInitialize (void);
  virtual void DoDispose (void);
  
  //private:
  /**
   * Helper structure for managing transmission queue elements.
   */
  struct TxQueueElement
  {
    uint8_t txQMsduHandle; //!< MSDU Handle
    Ptr<Packet> txQPkt;    //!< Queued packet
  };
  
  /**
   * Remove the tip of the transmission queue, including clean up related to the
   * last packet transmission.
   */
  virtual void RemoveFirstTxQElement () {};
  
  /**
   * Check the transmission queue. If there are packets in the transmission
   * queue and the MAC is idle, pick the first one and initiate a packet
   * transmission.
   */
  virtual void CheckQueue (void) {};
  
  /**
   * Change the current MAC-PHY state to the given new state. 
   *
   * NOTE: This state is relative to the channel, not the FSM of the 
   *       sensor's access protocol.
   *
   * \param newState the new state
   */
  void ChangeMacState (SensorChnState newState);
  
  /**
   * The trace source fired when packets are considered as successfully sent
   * or the transmission has been given up.
   * Only non-broadcast packets are traced.
   *
   * The data should represent:
   * packet, number of retries, total number of csma backoffs
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet>, uint8_t, uint8_t > m_sentPktTrace;
  
  /**
   * The trace source fired when packets come into the "top" of the device
   * at the L3/L2 transition, when being queued for transmission.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxEnqueueTrace;
  
  /**
   * The trace source fired when packets are dequeued from the
   * L3/l2 transmission queue.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxDequeueTrace;
  
  /**
   * The trace source fired when packets are being sent down to L1.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxTrace;
  
  /**
   * The trace source fired when packets where successfully transmitted, that is
   * an acknowledgment was received, if requested, or the packet was
   * successfully sent by L1, if no ACK was requested.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxOkTrace;
  
  /**
   * The trace source fired when packets are dropped due to missing ACKs or
   * because of transmission failures in L1.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxDropTrace;
  
  /**
   * The trace source fired for packets successfully received by the device
   * immediately before being forwarded up to higher layers (at the L2/L3
   * transition).  This is a non-promiscuous trace.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macRxTrace;
  
  /**
   * The trace source fired for packets successfully received by the device
   * but dropped before being forwarded up to higher layers (at the L2/L3
   * transition).
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macRxDropTrace;
  
  /**
   * A trace source that emulates a non-promiscuous protocol sniffer connected
   * to the device.  Unlike your average everyday sniffer, this trace source
   * will not fire on PACKET_OTHERHOST events.
   *
   * On the transmit size, this trace hook will fire after a packet is dequeued
   * from the device queue for transmission.  In Linux, for example, this would
   * correspond to the point just before a device hard_start_xmit where
   * dev_queue_xmit_nit is called to dispatch the packet to the PF_PACKET
   * ETH_P_ALL handlers.
   *
   * On the receive side, this trace hook will fire when a packet is received,
   * just before the receive callback is executed.  In Linux, for example,
   * this would correspond to the point at which the packet is dispatched to
   * packet sniffers in netif_receive_skb.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_snifferTrace;
  
  /**
   * A trace source that emulates a promiscuous mode protocol sniffer connected
   * to the device.  This trace source fire on packets destined for any host
   * just like your average everyday packet sniffer.
   *
   * On the transmit size, this trace hook will fire after a packet is dequeued
   * from the device queue for transmission.  In Linux, for example, this would
   * correspond to the point just before a device hard_start_xmit where
   * dev_queue_xmit_nit is called to dispatch the packet to the PF_PACKET
   * ETH_P_ALL handlers.
   *
   * On the receive side, this trace hook will fire when a packet is received,
   * just before the receive callback is executed.  In Linux, for example,
   * this would correspond to the point at which the packet is dispatched to
   * packet sniffers in netif_receive_skb.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_promiscSnifferTrace;
  
  /**
   * A trace source that fires when the Sensor's Mac channel access
   * changes states.  This is different from the FSM for the 
   * protocol running the LLC of the MAC.
   *
   * Parameters are the old mac state and the new mac state.
   *
   * \deprecated This TracedCallback is deprecated and will be
   * removed in a future release,  Instead, use the \c MacStateValue
   * TracedValue.
   */
  TracedCallback<SensorChnState, SensorChnState> m_macStateLogger;
  
  /**
   * The PHY associated with this MAC.
   */
  Ptr<LrWpanPhy> m_phy;
  
  /**
   * The CSMA/CA implementation used by this MAC.
   */
  Ptr<SensorCsmaCa> m_csmaCa;
  
  /**
   * This callback is used to notify incoming packets to the upper layers.
   * See IEEE 802.15.4-2006, section 7.1.1.3.
   */
  SensorDataIndicationCallback m_sensorDataIndicationCallback;
  
  /**
   * This callback is used to report data transmission request status to the
   * upper layers.
   * See IEEE 802.15.4-2006, section 7.1.1.2.
   */
  SensorDataConfirmCallback m_sensorDataConfirmCallback;
  
  /**
   * This callback is used to report data receptions status to the upper
   * layer.  For this project, it is expected to be the SensorHelper.  This
   * callback is used to report data reception by a forwarder node. In the
   * case of LEACH, it is a ClusterHead.
   */
  CHDataRcvdCallback m_sensorCHDataRcvdCallback;
  
  /**
   * This callback is used to report data receptions status to the upper
   * layer.  For this project, it is expected to be the SensorHelper.  This
   * callback is used to report data received with a list of Mac16Addresses
   * as this data reception is from the SINK node.
   */
  SinkDataRcvdCallback m_sensorSinkDataRcvdCallback;

  /**
   * This callback is used to report data receptions status to the upper
   * layer.  For this project, it is expected to be the SensorHelper.  This
   * callback is used to report data received with a list of Mac16Addresses
   * as this data reception is from the SINK node.
   */
  SinkDCRcvdCallback m_sensorSinkDCRcvdCallback;

  /**
   * This is the callback used to notify the script owner that a node died
   * due to a lack of power.  If this goes to the SensorHelper, it should
   * stop all Layer 3 packets being sent to this node as well as collect
   * statistics on dead times.
   */
  SensorDeadCallback m_sensorDeadCallback;
  
  /**
   * This is the callback used to notify the script owner that a node is
   * transmitting the next data frame in its data queue. A frame is tracked
   * only when the frame is actually getting pushed to the PHY, as the 
   * frame may get queued a long time in advanced of the channel beign ready.
   */
  SensorDataTransmitCallback m_sensorDataTransmitCallback;
  
  /**
   * This callback is called when a node officially begins operation as a
   * Forwarder (a CH in LEACH). On a per-MAC basis, sometimes, the status
   * is delayed until forwader selection is confirmed.
   */
  SensorForwarderSelectedCallback m_sensorForwaderCallback;
  
  /**
   * This callback is called when a node decides to operate in a Direct 
   * Connection mode with the SINK.  This is normally due to a lack of
   * forwarder in between the node and the SINK for a variety of reasons.
   */
  SensorDirectSelectCallback m_sensorDirectCallback;
  
  /**
   * This callback is called when when the SINK determines it is time for 
   * the next epohc (round) to begin. A 32 bit value must be used to support
   * MACs that execute nearly indefinitely.
   */
  SensorNextRoundCallback m_sensorNextRoundCallback;
  
  /**
   * The current state of the MAC-PHY layer.
   *
   * NOTE: This is different than the protocol's channel access FSM.
   */
  TracedValue<SensorChnState> m_sensorMacState;
  
  /**
   * Uniform random variable stream.
   */
  Ptr<UniformRandomVariable> m_random;
  
  /**
   * The packet which is currently being sent by the MAC layer.
   */
  Ptr<Packet> m_txPkt;  // XXX need packet buffer instead of single packet
  
  /**
   * The short address used by this MAC. Currently we do not have complete
   * extended address support in the MAC, nor do we have the association
   * primitives, so this address has to be configured manually.
   */
  Mac16Address m_Address;

  /**
   * The transmit queue used by the MAC.
   */
  std::deque<TxQueueElement*> m_txQueue;

  /**
   * The App Data queue used by the MAC. This queue preloads data packets 
   * pending the MAC gaining access to the channel or needing to transition
   * to a data state.  Mostly used for LEACH.
   */
  std::deque<TxQueueElement*> m_dataQueue;
  
  /**
   * The number of already used retransmission for the currently transmitted
   * packet.
   */
  uint8_t m_retransmission;
  
  /**
   * The number of CSMA/CA retries used for sending the current packet.
   */
  uint8_t m_numCsmacaRetry;
  
  /**
   * Current power level in mW
   */
  double m_mWpwr;
  
  /**
   * Scheduler event for the ACK timeout of the currently transmitted data
   * packet.
   */
  EventId m_ackWaitTimeout;
  
  /**
   * Scheduler event for a deferred MAC state change.
   */
  EventId m_setMacState;
  
  /**
   * Collection of variables used to determine the amount of power drained
   * from teh sensor as the PHY changes state.
   */
  LrWpanPhyEnumeration m_previousState;
  Time m_timePrevState;
  double m_totalSystemPower;

  bool m_systemEnabled;
};
  
  
} // namespace ns3

#endif      // SENSOR_MAC_H
