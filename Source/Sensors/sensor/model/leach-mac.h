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

#ifndef LEACH_MAC_H
#define LEACH_MAC_H

#include "sensor-mac.h"
#include "leach-mac-header.h"
#include <ns3/object.h>
#include <ns3/traced-callback.h>
#include <ns3/traced-value.h>
#include <ns3/mac16-address.h>
#include <ns3/mac64-address.h>
#include <ns3/sequence-number.h>
#include <ns3/lr-wpan-phy.h>
#include <ns3/event-id.h>
#include <ns3/timer.h>
#include <deque>


namespace ns3 {
  
class Packet;
class SensorCsmaCa;

/**
 * \ingroup lr-wpan
 *
 * MAC states
 */
typedef enum
{
  LEACH_INIT,            //!< MAC is Idle on initialization
  LEACH_ADVERTISE,
  LEACH_SCHEDULE,
  LEACH_WAIT_TDMA,
  LEACH_PRE_SLEEP,
  LEACH_DATA_PHASE,
  LEACH_POST_SLEEP,
  LEACH_DIRECT_CONN,
  LEACH_SINK_DATA,
  LEACH_NODE_DEAD,       //!< Sensor node died due to power loss
  LEACH_INVALID_STATE    //!< invalid state - should never reach this
} LeachMacState;

const char leachString[LEACH_INVALID_STATE+1][20] =
{
  "INIT\0",             //!< MAC is Idle on initialization
  "ADVERTISE\0",        //!< CHs send out a schedule, members determine start time
  "SCHEDULE\0",         //!< CHs actively listen
  "WAIT_TDMA\0",        //!< CH members waiting for thier TDMA slot
  "PRE_SLEEP\0",        //!< CH TX members send data
  "DATA_PHASE\0",       //!< CH TX member sleep post transmit
  "POST_SLEEP\0",       //!< CHs forward data to Sink
  "DIRECT_CONN\0",      //!< node is a direct connect to SINK
  "SINK_DATA\0",
  "NODE_DEAD\0",        //!< Sensor node died due to power loss
  "INVALID_STATE\0"     //!< invalid state - should never reach this
};
  
namespace TracedValueCallback {
  
  /**
   * \ingroup sensor
   * TracedValue callback signature for LeachMacState.
   *
   * \param [in] oldValue original value of the traced variable
   * \param [in] newValue new value of the traced variable
   */
  typedef void (* LeachMacState)(LeachMacState oldValue,
                                 LeachMacState newValue);
  
}  // namespace TracedValueCallback
  
/**
 * \ingroup sensor
 *
 * This class implements the LEACH MAC state machine on top of the standard
 * sensor MAC. LEACH has a particular process to stand up channel diverse 
 * self-elected Cluster Heads. 
 */
class LeachMac : public SensorMac
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
   * No longer compatible with 802.15.4
   */
  static const uint32_t aMinMPDUOverhead;
  
  /**
   * Default constructor.
   */
  LeachMac (void);
  virtual ~LeachMac (void);
  
  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  virtual int64_t AssignStreams (int64_t stream);
  
  /*
   * Kick off the FSM enabling communications on the MAC-PHY. Each MAC will
   * need to perform its own channel access methods before a user-app
   * can proceed with data transmissions.
   *
   * This method is pure virtual.  It is specific to the MAC implemented.
   *
   */
  virtual void KickOffFSM( void );
  
  /*
   * This method will force the implementation class to cancel all 
   * timers. This should be called from the dead node callback 
   * function so that we abort any future processes.
   * 
   * NOTE: Calling it from within for now.
   */
  virtual void NodeJustDied( void );
  
  /**
   *  IEEE 802.15.4-2006, section 7.1.1.1
   *  MCPS-DATA.request
   *  Request to transfer a MSDU.
   *
   *  \param params the request parameters
   *  \param p the packet to be transmitted
   */
  void SensorDataRequest (SensorDataRequestParams params, Ptr<Packet> p);
  
  /**
   *  Request to transfer a Data Unit - not compatible with IEEE.
   *
   *  \param params the request parameters
   */
  void SensorDataRequest (SensorDataRequestParams params );

  /**
   * CSMA-CA algorithm calls back the MAC after executing channel assessment.
   *
   * \param macState indicate BUSY oder IDLE channel condition
   */
  virtual void SetSensorMacState (SensorChnState macState);
  
  
  // interfaces between MAC and PHY
  /**
   *  IEEE 802.15.4-2006 section 6.2.1.3
   *  PD-DATA.indication
   *  Indicates the transfer of an MPDU from PHY to MAC (receiving)
   *  @param psduLength number of bytes in the PSDU
   *  @param p the packet to be transmitted
   *  @param lqi Link quality (LQI) value measured during reception of the PPDU
   */
  virtual void PdDataIndication (uint32_t psduLength, Ptr<Packet> p, uint8_t lqi);
  
  /**
   *  IEEE 802.15.4-2006 section 6.2.2.2
   *  PLME-CCA.confirm status
   *  @param status TRX_OFF, BUSY or IDLE
   */
  virtual void PlmeCcaConfirm (LrWpanPhyEnumeration status);
  
  /**
   *  IEEE 802.15.4-2006 section 6.2.2.4
   *  PLME-ED.confirm status and energy level
   *  @param status SUCCESS, TRX_OFF or TX_ON
   *  @param energyLevel 0x00-0xff ED level for the channel
   */
  virtual void PlmeEdConfirm (LrWpanPhyEnumeration status, uint8_t energyLevel);
  
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
                                        LrWpanPhyPibAttributes* attribute);
  
  /**
   *  IEEE 802.15.4-2006 section 6.2.2.8
   *  PLME-SET-TRX-STATE.confirm
   *  Set PHY state
   *  @param status in RX_ON,TRX_OFF,FORCE_TRX_OFF,TX_ON
   */
  virtual void PlmeSetTRXStateConfirm (LrWpanPhyEnumeration status);
  
  /**
   *  IEEE 802.15.4-2006 section 6.2.2.10
   *  PLME-SET.confirm
   *  Set attributes per definition from Table 23 in section 6.4.2
   *  @param status SUCCESS, UNSUPPORTED_ATTRIBUTE, INVALID_PARAMETER, or READ_ONLY
   *  @param id the attributed identifier
   */
  virtual void PlmeSetAttributeConfirm (LrWpanPhyEnumeration status,
                                        LrWpanPibAttributeIdentifier id);
  
  /**
   *  IEEE 802.15.4-2006 section 6.2.1.2
   *  Confirm the end of transmission of an MPDU to MAC
   *  @param status to report to MAC
   *  PHY PD-DATA.confirm status
   */
  virtual void PdDataConfirm (LrWpanPhyEnumeration status);
  
  /**
   *  Function used to kick off the TDMA transmission post WAIT period.
   *  This is a static callback, so the handle to the specific LeachMac
   *  object must be passed so we know which object methods to call.
   */
  void WaitTimerExpired( void );
  
  /**
   *  Function used to check if the node must become a Direct Connect
   *  node based on the TDMA list message from associated CH.
   */
  void JoinCheckTimerExpired( void );
  
  /**
   * This method is a timer callback used by the CH to check to see if
   * we have enough CH's for this round.
   */
  void SinkAdvertiseDoneCheck( void );
  void SinkJoinDoneCheck( void );
  void SinkDataPhaseDoneCheck( void );
  
  /**
   * Sequence number added to transmitted data or MAC command frame, 00-ff.
   * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
   */
  SequenceNumber8 m_macDsn;
  
  int m_maxNumNodes;
  int m_numClusters;
  
protected:
  
  /**
   * Check the transmission queue. If there are packets in the transmission
   * queue and the MAC is idle, pick the first one and initiate a packet
   * transmission.
   */
  void CheckQueue (void);
  
private:
  
  struct BestCH_s {
    BestCH_s() {
      lqi = 0;
      addr = Mac16Address ("00:00");
      chan = 0;
      pwr = HIGH_POWER;
      sinr = 0.0;
    };
    
    int lqi;
    int chan;
    int pwr;
    double sinr;
    ns3::Mac16Address addr;
  };
  
  /*
   * This method is the main FSM for the Sink node in the LEACH 
   * protocol.  This method should be modified based on the form 
   * of LEACH being used.
   *
   * @param frame   The most recent frame received from the PHY
   */
  void ProcessSinkReception( Ptr<Packet> frame );

  /*
   * This method is the FSM for a node operating as a Cluster Head.
   * This method is used when a node self elects to operate as a 
   * Cluster Head, and a node stops using this method when the 
   * end of the cycle has been reached.
   *
   * @param frame   The most recent frame received from the PHY
   */
  void ProcessClusterHeadReception( Ptr<Packet> frame, uint8_t lqi );
  
  /*
   * This method is the FSM for a node operating as a Cluster Member.
   * A node uses this method when they are operating in the Data
   * phase as a member of a cluster. Technically, they should not
   * receive any traffic as a member of a cluster, as they should only
   * be transmitting their data during their assigned TDMA slot.
   *
   * @param frame   The most recent frame received from the PHY
   */
  void ProcessClusterMemberReception( Ptr<Packet> frame );
  
  /*
   * This method is the Basic FSM for all nodes.  This method is 
   * used for nodes when attempting to elect Cluster Heads,
   * or when preparing for the next round of operation.
   *
   * @param frame   The most recent frame received from the PHY
   * @param lqi     Rcvd LQI assessment for best TX'er identification
   */
  void ProcessNodeReception( Ptr<Packet> frame, uint8_t lqi );
  
  /**
   * Remove the tip of the transmission queue, including clean up related to the
   * last packet transmission.
   */
  virtual void RemoveFirstTxQElement ();
  
  /*
   * This method is used to perform the start of rounds decision 
   * processing.  A node must run the PRNG to determine if it should
   * self elect as a Cluster Head, or act as a member node.
   *
   * @param bool  Default is TRUE, FALSE if we short circuit due to 
   *              missing the Super Frame signal
   */
  void NextRoundDecision( bool calc = true );
  
  void CheckThisRoundAgain(void);
  
  /*
   * The Init Phase Timeout - used to restart the LEACH
   * process.  Similar to the KickOffFSM method, but avoids
   * reseting all values, as we need to continue, not reset.
   */
  void InitPhaseTimeout(void);
  
  /*
   * Track the LEACH FSM state.
   */
  void ChangeLeachState(LeachMacState state);
  
  /*
   * Method used to kick off a Direct Connect DATA payload 
   * to the SINK.
   */
  void DirectConnectTransmission( void );
  
  /*
   * Creates a packet without data based on the provided LeachMacFrameType.
   * Caller must supply the Source, Destination, and sequence number.
   */
  Ptr<Packet> CreatePacket( LeachMacFrameType type,
                            ns3::Mac16Address src,
                            ns3::Mac16Address dst,
                            uint32_t seqNum );
  
  /*
   * Creates a packet with data based on the provided LeachMacFrameType.
   * Caller must supply the Source, Destination, and sequence number.
   * The data is expected to be a byte pointer based on a LeachMacHeader type.
   */
  Ptr<Packet> CreatePacketWithData( LeachMacFrameType type,
                                    ns3::Mac16Address src,
                                    ns3::Mac16Address dst,
                                    uint32_t seqNum, uint8_t const *data, int length );
  
  /*
   * This method extracts the final list of cluster heads for the
   * current round. The packet passed in is expected to have the header
   * removed already, and the packet must be of the RX_FINAL_CH_LIST
   * type to maintain consistency. 
   *
   * @param Ptr<Packet>   packet received from the sink
   * @param int           # of bytes in payload
   * @return  bool        TRUE if the CH list is valid
   */
  bool ExtractViableClusterHeads( Ptr<Packet> p, int size );
  
  bool GetNextViableClusterHead();
  
  /*
   * This method is used to launch the Direct Connected nodes'
   * data delivery to the sink node. This is done when a node 
   * does not have an associated CH.
   */
  void LaunchDirectConnect( Time t = Seconds( 0.0 ) );
  
  void TxDirectConnect( void );
  
  /**
   * Method used to parse the CH Advertisement packet - adds 
   * potential CHs to the internal list for future processing.
   *
   * @param Packet  The Packet from a CH node
   */
  void ParseChAdvertisement( Ptr<Packet> p, uint8_t lqi, Mac16Address srcAddr );
  
  void CHJoinDoneCheck( void );
  void CHdataPhaseComplete(void);
  
  void CheckIfClusterMember( void );
  void NodeWaitingForInit(void);
  
  void CheckIfCHAlive(void);
  
  void SinkDelivery( int offset = 0 );
  
  void SetChannel( int chan );
  
  void SetPower( int pwr );
  
  void ForceAnotherTx(void);
  
  /**
   * The current state of the MAC layer.
   */
  TracedValue<LeachMacState> m_leachMacState;
  
  /**
   * Scheduler event for timeout of the Cluster Head Advertisement
   * phase of operation. When we receive the Super Frame, we kick 
   * off the CH Advertisement, and that has a fixed end point.
   */
  EventId m_chAdvTimeout;

  EventId m_csmaEvent;

  EventId m_forceTxEvent;
  
  EventId m_idleEvent;

  Timer m_forceTxTimer;
  
  /**
   * Wait timer.
   * A node waits for its TDMA slot with the transceiver turned
   * off. When this timer expires, the node is allowed to wake
   * up and kick off its transmission.
   */
  Timer m_waitTimer;
  
  /**
   * Join Timer
   * A node waits for its CH to advertise the TDMA slot list. If
   * the node does not hear a slot list, then the node must switch 
   * over to a Direct Connection to the SINK to deliver its data.
   */
  Timer m_joinTimer;
  
  /**
   * CH Is Dead Timer
   * A node waits for to hear from its CH after calling the 
   * CheckIfClusterMember() method.  If it doesn't hear from
   * the CH within XXms, then the node must transition to a 
   * Direct Connect node, but has less time to TX Data
   */
  Timer m_chIsDeadTimer;
  
  /// Provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;
  
  bool iAmClusterHead;
  bool m_amClusterMember;
  bool hasBeenClusterHead;
  bool m_sinkQueried;
  bool m_pauseCsma;
  
  bool m_waitForAck;
  bool m_joinMsgSent;
  
  bool m_rcvdTdmaList;
  
  bool m_chIsAlive;
  
  uint16_t m_myCHchan;
  
  std::list<Mac16Address> m_ClusterHeads;
  std::list<Mac16Address> m_MemberNodes;
  std::list<Mac16Address> m_rcvdMembers;
  std::list<uint32_t>     m_rcvdSequences;
  
  std::list<BestCH_s> m_bestCh;
  struct BestCH_s     m_prospectCh;
  Mac16Address        m_currentCh;
  
  int m_numChs;
  int m_currentRound;
  int m_totalRounds;
  int m_myChPos;
  int m_epoch;
    
  Time m_sleepStart;
  Time m_sleepFinish;
  
  Time m_maxJoinTime;
  
  Time m_schdDoneTime;
  
  Time m_delayAdvSch;
  
  int checkCounter;
  
  /*
   * These are variable use in the CH-Member receiving the ADV SCHED.
   * They need to be global per instance.
   */
  int m_rxAdvSchFrames;
  int m_advPosOffset;
  bool m_advFoundSpot;

  char *debugForceTx;
};
  

  
}   // namespace ns3

#endif    // LEACH_MAC_H