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

#ifndef BATSEN_MAC_H
#define BATSEN_MAC_H

#include "sensor-mac.h"
#include "batsen-mac-header.h"
#include <ns3/lr-wpan-sinr-tag.h>
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
#include <list>
#include <map>


namespace ns3 {
  
class Packet;
class SensorCsmaCa;

/**
 * \ingroup sensor
 *
 * This class implements the BATSEN MAC state machine on top of the standard
 * sensor MAC. BATSEN has a particular process to stand up channel diverse
 * self-elected Cluster Heads.
 */
class BatsenMac : public SensorMac
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
  BatsenMac (void);
  virtual ~BatsenMac (void);
  
  void DoDispose();
  
  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  int64_t AssignStreams (int64_t stream);
  
  /**
   * Set the short address of this MAC.
   *
   * \param address the new address
   */
  void SetAddress (Mac16Address address);
  
  /*
   * Kick off the FSM enabling communications on the MAC-PHY. Each MAC will
   * need to perform its own channel access methods before a user-app
   * can proceed with data transmissions.
   *
   * This method is pure virtual.  It is specific to the MAC implemented.
   *
   */
  void KickOffFSM( void );
  
  /*
   * This method will force the implementation class to cancel all
   * timers. This should be called from the dead node callback
   * function so that we abort any future processes.
   *
   * NOTE: Calling it from within for now.
   */
  void NodeJustDied( void );
  
  /*
   * Track the BATSEN FSM state.
   */
  void ChangeFsmState(BatsenFsmState state);
  
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
   *  IEEE 802.15.4-2006 section 6.2.1.3
   *  PD-DATA.indication
   *  Indicates the transfer of an MPDU from PHY to MAC (receiving)
   *  @param psduLength number of bytes in the PSDU
   *  @param p the packet to be transmitted
   *  @param lqi Link quality (LQI) value measured during reception of the PPDU
   */
  void PdDataIndication (uint32_t psduLength, Ptr<Packet> p, uint8_t lqi);

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
   *  IEEE 802.15.4-2006 section 6.2.1.2
   *  Confirm the end of transmission of an MPDU to MAC
   *  @param status to report to MAC
   *  PHY PD-DATA.confirm status
   */
  virtual void PdDataConfirm (LrWpanPhyEnumeration status);
  
  /*
   * The OGM Phase Timeout - used to restart the BATSEN
   * process.
   */
  void StartNullOgmCollection(void);
  void SinkSpawnPeriodicOgm(void);
  void NullOgmXmitStage(void);
  void RxPwrOgmXmitStage(void);
  void CreateRouterOgm(void);
  void SleepPlan(void);
  
  void ConfigureForwarderPower(void);
  void WakeToSendData(void);
  void BackToSleep(void);
  void WakePostSleep(void);
  
  void RelayDataToSink(void);
  
  /*
   * Basic checks for Rcv Pwr list and Master node lists
   * common for all OGM types
   */
  void ProcessOgmPresence(Mac16Address src, OgmMessage *msg, LrWpanSinrTag stag);
  
  /*
   * Processing of a non-NULL OGM list
   */
  void ProcessPeerOgm(Mac16Address src, char *buffer, int len, LrWpanSinrTag stag);
  
  void ProcessSinkOgm(Mac16Address src, char *buffer, int bytes);
  
  void SinkScansNodeOgm(Mac16Address src, char *buffer, int bytes);
  
  const char *GetStateString() {
    return (const char *) batsenFsmString[m_batsenFsmState];
  };
  
  int m_maxNumNodes;
  int m_numClusters;
  double m_multiplier;
  
protected:
  /**
   * The current state of the MAC layer.
   */
  TracedValue<BatsenFsmState> m_batsenFsmState;
  
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
   * Creates a packet without data based on the provided LeachMacFrameType.
   * Caller must supply the Source, Destination, and sequence number.
   */
  Ptr<Packet> CreatePacket( BatsenMacFrameType type,
                           ns3::Mac16Address src,
                           ns3::Mac16Address dst,
                           uint32_t seqNum );
  
  /*
   * Creates a packet with data based on the provided BatsenMacFrameType.
   * Caller must supply the Source, Destination, and sequence number.
   * The data is expected to be a byte pointer based on a BatsenMacHeader type.
   */
  Ptr<Packet> CreatePacketWithData( BatsenMacFrameType type,
                                   ns3::Mac16Address src,
                                   ns3::Mac16Address dst,
                                   uint32_t seqNum,
                                   uint8_t const *data,
                                   int length );

  /*
   * This method is the main FSM for the Sink node in the BATSEN
   * protocol.  This method should be modified based on the form
   * of BATSEN being used.
   *
   * @param frame   The most recent frame received from the PHY
   */
  void ProcessSinkReception( uint32_t psduLength, Ptr<Packet> p, uint8_t lqi );
  
  /*
   * This method is the Basic FSM for all nodes.  This method is
   * used for nodes when attempting to elect Cluster Heads,
   * or when preparing for the next round of operation.
   *
   * @param frame   The most recent frame received from the PHY
   * @param lqi     Rcvd LQI assessment for best TX'er identification
   */
  void ProcessNodeReception( uint32_t psduLength, Ptr<Packet> p, uint8_t lqi );
  
  void NullOgmPeriodComplete(void);
  
  void PlmeSetTRXStateConfirm (LrWpanPhyEnumeration status);

  void CheckQueue (void);
  
  void SetSensorMacState (SensorChnState macState);
  
  /*
   * Find a PeerNodeInfo * to a node in the Master List
   */
  bool FindExistingPeer(Mac16Address target, PeerMap::iterator &itr);
  
  /*
   * Find a PeerNodeInfo * to a node in one of the 
   * PeerMap lists for Low, Med, or Hi power lists
   */
  bool FindPeerInLists (Mac16Address target, PeerMap::iterator &itr);

  /*
   * Check to see if the node is in the master rcvd power
   * list for future NULL transmission
   */
  //  void CheckPowerList( PowerLvl pwr, Mac16Address src );
  
  void EnqueTxPacket( Ptr<Packet> p );
  
  void RemoveFirstTxQElement(void);
  
  void DumpPeerLists(void);
  
  EventId m_ogmTimeout;
  EventId m_forceTxEvent;
  EventId m_csmaEvent;
  
  /**
   * OGM Timer
   * Timer used by the SINK to spawn the OGM processing phase.
   */
  Timer m_ogmTimer;
  Timer m_startSleep;
  
  bool     m_iAmForwarder;
  double   m_maxSystemPower;
  uint16_t m_currentRound;
  uint32_t m_sequence;
  uint8_t  m_pktHandle;
  int      m_curNodeCount;
  
  int m_pwrToSink;
  int m_pwrToFwrd;
  bool m_resendPwr;
  bool m_justSentData;
  
  // Perspective of nodes based on pwr level to me
  PeerMap m_Lpeers;
  PeerMap m_Mpeers;
  PeerMap m_Hpeers;
  
  // Perspective of self
  PeerNodeInfo m_nodeSelf;

  // Contains a list of PeerNodeInfo* for any nodes present
  PeerMap m_masterList;

  // Map of nodes and power interval we can hear them on
  PwrList m_rxPwrList;
  PeerNodeInfo m_rcvPwrList;
  
  // Next Hop address for sending data to sink
  Mac16Address m_nextHop;
  
  // List of MAC addresses selected by peers as a forwarder
  SortedList m_electList;
  
  double m_sleepOne;
  Time m_sleepTwo;
  
  // Received sources and sequences
  std::list<Mac16Address> m_rcvdMembers;
  std::list<uint32_t>     m_rcvdSequences;
  
  Ptr<UniformRandomVariable> m_random;
  
  uint8_t m_lastSFrcvd;
  
};
  
}

#endif    // BATSEN_MAC_H

