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

#ifndef LR_WPAN_MAC_HEADER_H
#define LR_WPAN_MAC_HEADER_H

#include <ns3/header.h>
#include <ns3/mac16-address.h>


namespace ns3 {
  
#define SENSOR_HEADER_LENGTH  11
#define MAX_PACKET_LENGTH     127
#define MAC_TRAILER_LEN       2
#define MAX_PAYLOAD_LENGTH    (MAX_PACKET_LENGTH - (SENSOR_HEADER_LENGTH + MAC_TRAILER_LEN))    // 127 - 13 = 114
  
/*
 *
 *
 */
  
  
/**
 * The possible MAC types.
 */
enum LeachMacType
{
  LEACH_ORIGINAL    = 0,      //!< Original LEACH protocol
  LEACH_TIGHT_ORIG  = 1,      //!< LEACH with modifications to garauntee performance
  LEACH_ENERGY      = 2,      //!< Energy efficient LEACH
  LEACH_MAC_RESERVED          //!< Invalid LEACH Type
};

/**
 * The possible MAC Frame types.
 */
enum LeachMacFrameType
{
  SUPER_FRM_ANNOUNCE,
  ADVERTISE_CH,
  ADD_MORE_CHS,
  RX_FINAL_CH_LIST,
  JOIN_REQUEST,
  JOIN_ACK,
  JOIN_NACK,
  TDMA_KICK_OFF,
  ADVERTISE_SCHEDULE,
  DATA_FRAME,
  CH_PYLOD,
  SINK_DATA,
  INVALID_FRAME
};

const char g_leach_string[INVALID_FRAME][10] = {
  {"SUPR FRM"},
  {"ADVER CH"},
  {"ADD MORE"},
  {"NXT LIST"},
  {"JOIN REQ"},
  {"JOIN ACK"},
  {"JOIN NAK"},
  {"TDMA KIK"},
  {"ADV SCHD"},
  {"DATA FRM"},
  {"CH PYLOD"},
  {"SINK DAT"}
};

/*
 * From original LEACH TCL code
 *  $mac_dst $link_dst $ADV_CH $msg $datasize $opt(max_dist) $code_
 */
struct AdvertiseCh_s {
  uint16_t spreadCode;  // Two bytes for even number of bytes
};
  
/*
 * From original LEACH TCL code
 *  $self send $mac_dst $link_dst $JOIN_REQ $msg $datasize $opt(max_dist) $code_
 */
struct JoingRequest_s {
  Mac16Address  chAddress;  // Address of desired cluster head
};
  
/*
 * Using a simple list of 16 bit addresses. The data length
 * from the header is 2x # nodes. Avoids adding another byte 
 * to the frame for number of nodes.
 *
 * From original LEACH TCL code
 *  $self send $mac_dst $link_dst $ADV_SCH $msg $datasize $dist_ $code_
 *    set numNodes [llength $clusterNodes_]
 *    set xmitOrder $clusterNodes_
 *    set msg [list $xmitOrder]
 */
struct NodeList_s {
  uint16_t addresses[200];
};

/*
 * From original LEACH TCL code
 *  set nodeID [$self nodeID]
 *  set msg [list [list $nodeID , [$ns_ now]]]
 *  # Use DS-SS to send data messages to avoid inter-cluster interference.
 *  set spreading_factor $opt(spreading)
 *  set datasize [expr $spreading_factor * \
 *  [expr [expr $BYTES_ID * [llength $msg]] + $opt(sig_size)]]
 *  pp "$nodeID sending data $msg to $currentCH_ at [$ns_ now] (dist = $dist_)"
 *  set mac_dst $MAC_BROADCAST
 *  set link_dst $currentCH_
 *  $self send $mac_dst $link_dst $DATA $msg $datasize $dist_ $code_
 * 
 * NOTE:
 * - We need a sequence number to track success
 * - We need a length (# bytes) to read on receiver
 * - We need data array
 * - other stuff from original code is useless
 * 
 */
struct DataPayload_s {
  uint16_t numberBytes;
  uint32_t sequence;
  uint16_t data[50];
};
  
/**
 * \ingroup sensor
 * Represent the Mac Header with the Frame Control and Sequence Number fields
 */
class LeachMacHeader : public Header
{
public:
  
  LeachMacHeader (void);
  
  /**
   * Constructor
   * \param leachMacType    - the header type for the version of LEACH
   * \param leachMacFrmType - the Frame Primitive
   * \param seqNum          - the sequence number
   */
  LeachMacHeader (enum LeachMacType leachMacType,         // LEACH MAC Type
                  enum LeachMacFrameType leachMacFrmType, // Data, ACK, Control MAC Header must have
                  uint32_t seqNum);                       // frame control and sequence number.
                                                          // Beacon MAC Header must have frame control,
                                                          // sequence number, and source address.
  
  ~LeachMacHeader (void);
  
  /**
   * Get the MAC FSM type
   * \return the header type
   */
  enum LeachMacType GetMacType (void) const;
  
  /**
   * Get the primitive type
   * \return the header type
   */
  enum LeachMacFrameType GetFrameType (void) const;
  
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
   * Set the MAC FSM type
   * \param wpanMacType the frame type
   */
  void SetMacType (enum LeachMacType macType);
  
  /**
   * Set the Frame primitive type
   * \param wpanMacType the frame type
   */
  void SetFrameType (enum LeachMacFrameType frameType);
  
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
  
  uint8_t m_macType;        //!< Frame Control field bit 0-1  = 0 Orig, 1 Tight, 2 Energy
  uint8_t m_frmType;        //!< Frame Control field Bit 2-4  = 0 - Beacon, 1 - Data, 2 - Ack, 3 - Command
  
  uint32_t m_seqNum;        //!< Sequence number

  uint8_t m_dataLen;        //!< Max data length is 127 - 11 = 116 bytes
                            //uint8_t m_buffer[ 127 ];  //!< Actual frame

}; // LeachMacHeader
  
}; // namespace ns-3

#endif /* LR_WPAN_MAC_HEADER_H */
