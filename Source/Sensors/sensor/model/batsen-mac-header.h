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

/*
 * the following classes implements the LEACH Mac Header, which is not
 * compatible with the 802.15.4 header.
 *
 * For basic LEACH, there are 6 primitive messages:
 *
 *    Beacon
 *      Super Frame Announcement  - Sink
 *      Cluster Head Self Elect   - Nodes (CH)
 *      Membership Request        - Nodes (non-CH)
 *      Cluster Head Schedule     - Nodes (CH)
 *    Data
 *      Member Node Data          - Nodes (non-CH)
 *      Cluster Head Dump         - Nodes (CH)
 *
 * For a tightly controlled LEACH protocol, there are XXXX primitives:
 *
 *
 * There are XXX types of LEACH Mac Headers Frames, and they have these common fields
 *
 *    The common header fields occur in the first 16 bits to allow for reuse of the LEACH
 *    MAC header class regardless of message type.
 *
 *    //!< Frame Control field Bit 0-1    = 0 - Original LEACH
 *                                          1 - Tight LEACH
 *                                          2 - Energy Efficient LEACH
 *
 *    //!< Frame Control field Bit 2-5    = 0 - Beacon
 *                                          1 - Beacon ACK
 *                                          2 - Data
 *                                          3 - Data ACK
 *                                          4 -
 *
 * NOTES:
 *  1. Destination addresses are a CH address, the Sink, or a broadcast address.
 *  2. Acknowledgements are not normally necessary
 *  3.
 *
 */

#ifndef BATSEN_MAC_HEADER_H
#define BATSEN_MAC_HEADER_H

#include <ns3/header.h>
#include <ns3/mac16-address.h>
#include <map>
#include <list>

namespace ns3 {

#define CENTER_FREQ   11
  
typedef enum  {
  INVLD = 0xFF,
  HPWR = 0x99,
  MPWR = 0x77,
  LPWR = 0x55,
  LOCAL = 0x00
} PowerLvl;

/**
 * \ingroup sensor
 *
 * FSM states
 */
typedef enum
{
  BATSEN_INIT,         //!< MAC is Idle on initialization
  BATSEN_OGM,
  BATSEN_DATA,
  BATSEN_NODE_DEAD,    //!< Sensor node died due to power loss
  BATSEN_FSM_INVALID   //!< invalid state - should never reach this
} BatsenFsmState;

const char batsenFsmString[BATSEN_FSM_INVALID+1][20] =
{
  "INIT\0",          //!< MAC is Idle on initialization
  "OGM\0",           //!< Network is Bcast'ing OGMs
  "DATA\0",          //!< Nodes are transmitting data
  "NODE_DEAD\0",     //!< Sensor node died due to power loss
  "INVALID_STATE\0"  //!< invalid state - should never reach this
};

typedef std::map<Mac16Address, PowerLvl> PwrList;

  
typedef std::list<Mac16Address> BNodeList;
  
#define BATSEN_HDR_LEN    10
#define MAX_FRAME_LEN     127
#define MAC_TRAILER_LEN   2
#define MAX_PAYLD_LEN     (MAX_FRAME_LEN - (BATSEN_HDR_LEN + MAC_TRAILER_LEN))    // 127 - (10 + 2) = 115

/**
 * The possible MAC Frame types.
 */
enum BatsenMacFrameType
{
  SUPER_FRM,
  NULL_OGM,
  RXPWR_OGM,
  RTRSEL_FRAME,
  DAT_FRAME,
  FWRD_FRAME,
  INVLD_FRAME
};

const char g_batsen_string[INVLD_FRAME][10] = {
  {"SUPR FRME"},
  {"NULL OGM"},
  {"RXPWR OGM"},
  {"RTR SELCT"},
  {"DAT FRAME"},
  {"FWD FRAME"}
};
  
/*
 * Data structure used to store neighbor node information
 * used later to determine if a neighbor should act as a
 * forwarder.
 */
class PeerNodeInfo
{
public:
  PeerNodeInfo();
  PeerNodeInfo( Mac16Address addr );
  ~PeerNodeInfo() {};
  
  void SetAddress( Mac16Address addr ) { address = addr; };
  
  Mac16Address GetAddress() { return address; };
  
  double GetScore() { return m_score; };
  
  /*
   * Sets the known power level required to reach this node based on
   * the received SINR from the node's OGM.
   */
  void SetSinkPowerLvl( PowerLvl rxpwr ) { m_power = rxpwr; };
  
  /*
   * This method sets the power level the local node requires
   * to reach this node for routing communications
   */
  void SetFrwdPowerLvl( PowerLvl rxpwr ) { m_fwdPwr = rxpwr; };
  
  /*
   * Get the power level needed to reach this node from the local node.
   */
  PowerLvl GetFrwdPowerLvl(void) { return m_fwdPwr; };
  
  /*
   * Sets the most recently annouced power percentage remaining
   * for the peer node.
   */
  void SetPrcntPwrRemain( double percent ) { m_percent = percent; };
  
    /*
   * Sets the most recently annouced power percentage remaining
   * for the peer node.
   */
  void SetPrcntPwrRemain( uint8_t value ) { m_percent = ((double)value) / 255.0; };
  
  double GetPrcntPwrRemain() { return m_percent; };
  
  /*
   * Check to see if the node already exists in one of the power 
   * lists, and if so, report the power level at which the node
   * resides.
   */
  PowerLvl DoesNodeExist( Mac16Address mac );
  
  /*
   * Method safely adds a list of nodes to one of the power level
   * lists. It avoids duplicates by searching the target list for
   * the MAC's presence before adding the node to the internal list.
   */
  void AddNodeList( PowerLvl pwrlvl, BNodeList maclist );
  
  /*
   * This method adds an individual node to the internal lists.
   * This supports the single PeerNodeInfo structure for the
   * local node in order to compare itself against remote nodes.
   */
  void AddNode( PowerLvl pwrlvl, Mac16Address addr );
  
  /*
   * Method used to eliminate a node when the bitHistory detects the
   * node is most likely dead.
   */
  void RemoveNode( Mac16Address addr );
  
  /*
   * Optimized removal process - removes a node from a specific
   * power list rather than traversing all the lists in order.
   */
  void RemoveNodeFromList( PowerLvl pwrlvl, Mac16Address addr );
  
  /*
   * Method used to erase the lists of addresses to avoid memory
   * leaks when we're shutting down the simulation.
   */
  void ClearNodeList();
  
  BNodeList GetPwrList( PowerLvl pwrlvl ) {
    switch ( pwrlvl )
    {
      case HPWR:
        return hiPwrList;
        
      case MPWR:
        return mdPwrList;
        
      case LPWR:
      default:
        return lwPwrList;
    }
  };
  
  void CalculateScore(int numNodes);
  
  void HeardFrom();
  
  void Missing();
  
  void DontNeedRxOgm() { needPeerNotice = false; };
  
  bool NeedRxOgm() { return needPeerNotice; };
  
  // Used to determine if another node is "worse"
  bool operator<(const PeerNodeInfo *rhs) { return (*this) < (*rhs); };
  bool operator>(const PeerNodeInfo *rhs) { return (*this) > (*rhs); };
  bool operator<(const PeerNodeInfo &rhs);
  bool operator>(const PeerNodeInfo &rhs);
  
public:
  Mac16Address address;
  PowerLvl m_power;     /* Power req to TX to Sink */
  PowerLvl m_fwdPwr;    /* Power level to reach this node */
  double m_percent;
  
  uint16_t bitHistory;  /* bit shifted history of successful OGM rcvs */
  double m_quality;     /* quality is a function of bitHistory */
  
  double m_score;
  
  bool needPeerNotice;
  
  BNodeList lwPwrList;
  BNodeList mdPwrList;
  BNodeList hiPwrList;
};

typedef std::list<PeerNodeInfo *> PeerMap;
  
bool PeerCompare( PeerNodeInfo *lhs, PeerNodeInfo *rhs );

  std::string BufferToString (uint8_t *buffer, uint32_t len);
  
/*
 * The Originator Message (OGM) inherited from BATMAN describes a node's condition
 * i.e. the power remaining from teh original max power level and the current 
 * power level being used to transmit the OGM (as it may not be a high power transmission).
 * The message also relays the number of low and medium power nodes that the 
 * transmitter can hear.  This is because all nodes are garaunteed to be within 
 * high power range.
 *
 * The Low Power Count describes the number of peer nodes the transmitter has heard
 * from wherein the local node can reach using Low Power.  Likewise, the Med Power
 * count describes the number of nodes the local node can hear using medium power.
 * 
 * When filling in the m_addresses field, the field should be front loaded with 
 * Low Power nodes followed by Med Power nodes.  It is assumed that the remaining
 * unlisted nodes are all high power nodes.
 *
 * Structure byte layout breakdown:
 * 117 bytes - (4 + (2 * 56)) = 1
 *
 */
#define NULL_OGM_COLLECT    0x01
#define RXPWR_OGM_COLLECT   0x02
#define PERIODIC_OGM        0x03

struct SuperFrameMessage {
  uint16_t  m_roundNum;   //!< round number for current OGM
  uint16_t  m_ogmFlags;   //!< 0x01 - Start OGM collection
                          //!< 0x02 - OGM Announcement
                          //!< 0x03 - Periodic OGMs (normal op)
  uint16_t  m_numNodes;   //!< number of regsitered nodes
  uint16_t  m_forwarders; //!< number of forwarders for this round
};
// 4 bytes
  

/*
 * Use TLVs to create payload.
 *
 *  +----+----+--------+
 *  | Fl | Tp |   Len  |
 *  +----+----+--------+
 *
 *  Flags - nibble based on Type field
 *
 *  Type      Name          Purpose
 *    0       NULL          End a char array
 *    1       RXPwrLevels   List of nodes
 *    2       TXRcvrs       List of rcvrs @ a given power level
 */
struct OgmMessage {
  uint8_t   txPwrLvl;   //!< current TX power from transmitter
  uint8_t   pwrPrcnt;   //!< power remaining in sending node
};
  
  // OGM Type Field - Type of data that follows
#define NULL_TLV    0x00
#define RX_PWR_LIST 0x80
  
  // OGM Type Field - power level associated with following list
#define LPWR_LIST   0x01
#define MPWR_LIST   0x02
#define HPWR_LIST   0x04
  
#define RTR_SELCT   0x08
#define NOT_FOUND   0x10
#define DATA_WORD   0x20

  
  
struct RouterSelect {
  uint8_t   txPwrLvl;       //!< power level transmitter is using to send this frame
  uint8_t   pwrPrcnt;       //!< power remaining in sending node
  uint16_t  rtrSelect;      //!< the node selected to be the router for this node
};
// 4 bytes
  
struct DataMessage {
  uint16_t numberBytes;
  uint32_t sequence;
  uint16_t data[50];
};
  
/**
 * \ingroup sensor
 * Represent the Mac Header with the Frame Control and Sequence Number fields
 */
class BatsenMacHeader : public Header
{
public:
  
  BatsenMacHeader (void);
  
  /**
   * Constructor
   * \param leachMacType    - the header type for the version of LEACH
   * \param leachMacFrmType - the Frame Primitive
   * \param seqNum          - the sequence number
   */
  BatsenMacHeader (enum BatsenMacFrameType macFrmType,
                   uint32_t seqNum );
  
  ~BatsenMacHeader (void);
  
  /**
   * Get the primitive type
   * \return the header type
   */
  enum BatsenMacFrameType GetFrameType (void) const;
  
  /**
   * Get the Destination Short address
   * \return the Destination Short address
   */
  Mac16Address GetDstAddr (void) const;
  
  /**
   * Get the Source Short address
   * \return the Source Short address
   */
  Mac16Address GetSrcAddr (void) const;
  
  /**
   * Get the frame Sequence number
   * \return the sequence number
   */
  uint32_t GetSeqNum (void) const;
  
  /**
   * Get the length (in bytes) of the payload
   * \return number of bytes in the payload
   */
  int GetLength(void) { return (int)m_dataLen; };
  
  /**
   * Set the Frame primitive type
   * \param wpanMacType the frame type
   */
  void SetFrameType (enum BatsenMacFrameType frameType);
  
  /**
   * Set Source address
   * \param addr source address (16 bit)
   */
  void SetSrcAddr (Mac16Address addr);
  
  /**
   * Set Destination address fields
   * \param addr destination address (16 bit)
   */
  void SetDstAddr (Mac16Address addr);
  
  /**
   * Set the Sequence number
   * \param seqNum sequence number
   */
  void SetSeqNum (uint32_t seqNum);
  
  void SetLength( uint8_t len ) { m_dataLen = len; };
  
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  
  void Print (std::ostream &os) const;
  uint32_t GetSerializedSize (void) const;
  void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
  
private:
  /* Addressing fields */
  Mac16Address m_dstAddr;   //!< Dst Short addr (2 Octets)
  Mac16Address m_srcAddr;   //!< Src Short addr (2 Octets)
  /* Frame Control 2 Octets */

  uint32_t m_seqNum;        //!< Sequence number

  uint8_t m_frmType;        //!< Frame Type
  uint8_t m_dataLen;        //!< Max data length is 127 - 10 = 117 bytes
  
  //uint8_t m_buffer[ 127 ];  //!< Actual frame
};        // BatsenMacHeader
  
  
/*
 * \ingroup sensor
 * This Class is used to construct an OGM and reduce the redundant
 * code in the BATSEN MAC pacakge. The OGM header (not the frame 
 * header) is buit as part of the internal data in this structure.
 * When the data is filled, the owning object can use the data pointer
 * for building a packet in NS-3.
 */
class OgmBuilder {
  
public:
  OgmBuilder() :
    m_hdr( (OgmMessage *)data ),
    m_addrCnt( 0 ),
    m_tlvLen( 0 ),
    m_len( sizeof(OgmMessage) )
  {
    memset( data, 0, MAX_PAYLD_LEN );
    
    // We assume all OGMs are transmitted with High power and at 100% battery
    m_hdr->txPwrLvl = HPWR;
    m_hdr->pwrPrcnt = 0xFF;
  };
  
  ~OgmBuilder() {};
  
  void Clear(void) {
    m_addrCnt = 0;
    m_tlvLen = 0;
    m_len = sizeof(OgmMessage);
    
    // Don't destroy the header already configured
    memset( data, 0, MAX_PAYLD_LEN );
    m_hdr->txPwrLvl = txpower;
    m_hdr->pwrPrcnt = reaminder;
  };
  
  /*
   * Configures the fundamental header values for all OGMs.
   */
  void SetHeaderData( uint8_t pwrlvl, uint8_t remain ) {
    m_hdr->txPwrLvl = pwrlvl;
    m_hdr->pwrPrcnt = remain;
    
    // Store in case we call Clear()
    txpower = pwrlvl;
    reaminder = remain;
  };
  
  /*
   * Start adding a new TLV set to this OGM 
   * Only use for TLVs that will add MAC address
   * based data members
   */
  bool AddAddressTlv( uint8_t type );
  
  /*
   * Add an address to the current TLV set
   */
  bool AddAddress( Mac16Address addr );
  
  /*
   * All addresses are added and we can store the final
   * length for the TLV set
   */
  void CloseAddressOgm(void) {
    data[m_tlvLen] = m_addrCnt;
    m_addrCnt = 0;
  };
  
  /*
   * Adds a byte array of data to the payload
   */
  bool AddData( uint8_t type, uint8_t *input, uint8_t len );
  
  /*
   * Returns a pointer to the internal data for transmission
   */
  uint8_t *GetDataPtr(void) { return data; };
  
  /*
   * Get the overall data length for the current OGM
   */
  uint8_t GetDataSize(void) { return m_len; };
  
  bool CreateOgm(uint8_t *buffer, uint8_t length);
  
  bool GetNextTlv(uint8_t &type, uint8_t &length);
  
  bool GetAddressList( BNodeList &alist );
  
private:
  
  OgmMessage *m_hdr;
  
  uint8_t txpower;
  uint8_t reaminder;
  
  uint8_t m_addrCnt;
  uint8_t m_tlvLen;
  uint8_t m_len;
  uint8_t data[MAX_PAYLD_LEN];
};
  

/**
 * \ingroup sensor
 *
 * Sorted List class. This class allows us to use traditional
 * pointers with data structures to create ordered-dual key lists
 * with all pointer operations abstracted. This is needed as the
 * std::map and std::list are deficient in this area.
 * 
 * TODO: Convert this to a template
 */
class SortedList 
{
public:
  SortedList();
  
  ~SortedList();
  
  void Clear();
  
  void DumpList();
  
  /*
   * Creates a new DataStruct heap variable for connection to the 
   * linked list. It is always assumed that this method is called 
   * for the first time a node is added to the list, and thus 
   * should auto increment the count to a 1 value.
   */
  void AddNode( Mac16Address mac );
  
  /*
   * Finds the node for the given MAC address and increments the 
   * counter value and repositions the node in the list (if it 
   * existed already). If the node does not already exist, then a
   * new node is created and added to the tail.
   */
  void Increment( Mac16Address mac );
  
  /*
   * Return a std::list<Mac16Address> list of the top contenders 
   * for router selection. A count is provided in the event the 
   * network is large and requires more than one router per 
   * cluster.
   */
  BNodeList GetTopNodes( unsigned count );
  
private:
  
  struct DataStruct {
    Mac16Address  addr;
    unsigned      count;
    DataStruct    *prev;
    DataStruct    *next;
  };
  
  /*
   * Determine if a node in the linked list already exists for the given
   * MAC address from the MAC software.
   */
  DataStruct *FindAddr( Mac16Address mac );
  
  /*
   * Attempt to add a node with count 1 starting from the tail. Assuming
   * the list grows from left to right, we start at the rightmost end of
   * the list and search for a better node to the left for an insertion
   * point.
   */
  void InsertTail( DataStruct *node );

  /*
   * Attempt to add a node with a variable count value from a known 
   * point within the linked list. This is typically used when a node
   * has its count value updated an needs to be shifted to the left
   * due to the increase in votes.
   */
  void InsertFrom( DataStruct *node, DataStruct *search );
  
  DataStruct *listHead;
  DataStruct *listTail;
  
};


};        // namespace ns3
#endif    // BATSEN_MAC_HEADER_H


