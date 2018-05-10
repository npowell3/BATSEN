/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef AGGREGATOR_H
#define AGGREGATOR_H


#include <map>
#include <string>
#include "ns3/internet-module.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"


namespace ns3 {
   
typedef enum {
   // The first element is not a data type, but allows for 
   // addition of a char array that is added to the column
   // for future pivot table usage
   ROW_HEADER_STR = 0,
   AGG_TOTAL_BW,
   AGG_ROUTER_BW, 
   AGG_USER_BW,
   AVG_TOTAL_BW,
   AVG_ROUTER_BW, 
   AVG_USER_BW,
   
   // There are none beyond this
   MAX_PLOT_DATA_TYPE
} PlotDataType_e;

/*
 * This call stores the frame activity times for a MAC/PHY. The object 
 * contains the state of the PHY and the start/stop times for said
 * state.  This will allow us to go back and determine the power used 
 * over time per state transition.
 */
class PhyStateEvent {
   
public:

   /// The state of the hardware for the given event.
   enum PhyState_e
   {
      SLEEPING,
      TRANSMITTING,
      LISTENING,
      RECEIVING
   };

   PhyStateEvent( PhyState_e st, Time tm ) : 
      m_state( st ),
      m_start(tm ),
      m_stop( 0 ),
      m_power( 0.0 )
   {};
   
   PhyStateEvent() : 
      m_state( PhyState_e::LISTENING ),
      m_start( 0 ),
      m_stop( 0 ),
      m_power( 0.0 )
   {};
   
   ~PhyStateEvent() {};

   void StartState( PhyState_e st, Time tm, double pwr )
   {
      m_state = st;
      m_start = tm;
      m_power = pwr;
      m_stop  = Time( 0 );
   };
   
   void StartState( PhyState_e st, Time tm )
   {
      m_state = st;
      m_start = tm;
      m_power = 0.0;
      m_stop  = Time( 0 );
   };
   
   void StopEvent( Time tm ) {
      m_stop = tm;
   };
   
   void SetPowerLevel( double pwr ) { m_power = pwr; };
  
private:
   PhyState_e  m_state;
   Time        m_start;
   Time        m_stop;
   double      m_power;
};

typedef std::list<PhyStateEvent>    PhyStateList;

typedef std::map<uint32_t, PhyStateList>  NodePhyMap;

   
/*
 * This class is used to store information about when an application
 * transmits data payloads, and if the said payloads are received.
 * This data correlates to the data at layer 5 and above, not layer 
 * 3 and below, though it could be used for lower layers.
 */
class TransmitPacket {

public:
   TransmitPacket() : 
      m_start( 0 ),
      m_stop( 0 ),
      m_bytes( 0 ),
      m_guid( 0 ),
      m_received( false )
   {};

   TransmitPacket( uint32_t size, uint32_t id, Time time ) : 
      m_start( time ),
      m_stop( 0 ),
      m_bytes( size ),
      m_guid( id ),
      m_received( false )
   {};
   
   ~TransmitPacket() {};
   
   Time     m_start;
   Time     m_stop;
   uint32_t m_bytes;
   uint32_t m_guid;
   bool     m_received;
   
};

/*
 * Mapping of Packet GUID to TransmitPacket object
 */
typedef std::list<TransmitPacket>   TxPacketList;

/*
 * This class is used to store information about when a phy
 * transmits frames, and if the said frames are received.
 * This data correlates to the data at layer 2 and below.
 * NOTE: The same frame may hop multiple times, so they must 
 * be recorded separately per node.
 */
class TransmitFrame {

public:
   TransmitFrame() : 
      m_start( 0 ),
      m_stop( 0 ),
      m_bytes( 0 ),
      m_guid( 0 ),
      m_received( false )
   {};

   TransmitFrame( uint32_t size, uint32_t id, Time time ) : 
      m_start( time ),
      m_stop( 0 ),
      m_bytes( size ),
      m_guid( id ),
      m_received( false )
   {};
   
   ~TransmitFrame() {};
   
   Time     m_start;
   Time     m_stop;
   uint32_t m_bytes;
   uint32_t m_guid;
   bool     m_received;
   
};

typedef std::list<TransmitFrame>            TxFrameList;
/*
 * Mapping of Packet GUID to TransmitPacket object
 */
typedef std::map<uint32_t, TxFrameList>     TxFrameMap;

struct OutputRow {
   double time;
   double totalBytes;
   double routingBytes;
   double userBytes;
   double lostData;
      
   double powerExpended;
   
   double totalBw;
   double routerBw;
   double userBw;
   double lostBw;
   
   double percentData;
   double percentRouting;
   double percentLost;
   double percentOverhead;

};

typedef std::list<OutputRow>        PlotList;
 
struct SummaryData {
   double agg_total_bw;
   double avg_total_bw;
   double agg_user_bw;
   double avg_user_bw;
   double agg_router_bw;
   double avg_router_bw;
};
 
/**
 * \ingroup aggregator
 * This aggregator produces output used to make excel sheet plots.
 * This module is specific to the testing for routing protocols.
 **/

class Aggregator : public SimpleRefCount<Aggregator>
{
   
public:

   Aggregator ();
   
   ~Aggregator ();
   
  void ClearData();
  
  /*
   * This method is used as a callback for when packets are transmitted from 
   * an application.  This allows us to track all transmissions to determine
   * the maximum bandwidth transmitted by all applications as well as the 
   * loss rate based on packets that were not received by their sink.
   * 
   * @param Ptr<Packet>    Smart Pointer to the packet data structure
   */
  void PacketTxDataCallback( Ptr<Packet const> pkt );

  /*
   * This method is used as a callback for when packets are received by
   * an application.  This method finds the associated TX entry in the 
   * m_dataPackets and asserts the received flag and the stop time for 
   * said packet.  
   * 
   * NOTE: This method is only useful for Unicast traffic. Do not try 
   * to collect data on multicast or broadcast traffic.
   * 
   * @param Ptr<Packet>    Smart Pointer to the packet data structure
   */
  void PacketRxDataCallback( Ptr<Packet const> pkt );

  /*
   * This method is used as a callback for when a router transmits its
   * routing primitive PDU. This method asserts the transmission start
   * time in the m_routerPackets list, which is of type TxPacketList. 
   * The stop time is not used, as it is not required for the data 
   * collection.  This is because we are only interested in when the 
   * packet was sent and the amount of data included in the packet.
   * 
   * @param Ptr<Packet>    Smart Pointer to the packet data structure
   */
  void PacketRouterCallback( Ptr<Packet const> pkt );

  /*
   * This method is used as a callback for when frames are transmitted by 
   * MAC/PHY (i.e. NetDevice).  This allows us to track the total number
   * of bytes transmitted into the channel regardless of success.  This also
   * will be used to determine the maximum bandwidth used over time such
   * that we can calculate the percentage of BW used for user data and that
   * used for routing data.  This gives us a performance metric.
   * 
   * This has a second purpose of tracking the TX state for the PHY which 
   * can then be used to calculate the total power expended during the 
   * transmission.
   * 
   * @param Ptr<Packet>    Smart Pointer to the packet data structure
   */
  void FrameTxStartCallback( uint32_t nodeid, Ptr<Packet const> pkt );

  /*
   * This method is used as a callback for when a transmission frame is
   * complete.  This fills in the stop time for the frame identified by
   * the GUID.  This allows us to perform similar calculations as was 
   * done for the User Data Packets.
   * 
   * @param Ptr<Packet>    Smart Pointer to the packet data structure
   */
  void FrameTxStopCallback( uint32_t nodeid, Ptr<Packet const> pkt );

  /*
   * This method is used as a callback for when frames are received by 
   * a netdevice. This allows us to track the power level used during 
   * an active receive,
   * 
   * @param Ptr<Packet>    Smart Pointer to the packet data structure
   */
  void FrameRxStartCallback( uint32_t nodeid, Ptr<Packet const> pkt );

  /*
   * This method is used as a callback for when actively received frames
   * are complete.  This allows us to track when the radio is actively 
   * receiving and passively receiving, which may be considered two 
   * different power levels.
   * 
   * @param Ptr<Packet>    Smart Pointer to the packet data structure
   */
  void FrameRxStopCallback( uint32_t nodeid, Ptr<Packet const> pkt );
  
  void ExtractData( double maxTime, double scale );
  
  void PlotData( const char *filename );

  void PlotData( const char *filename, double maxTime, double scale );
  
  void PlotItems( int num, ... );
  
private:
  
  TxPacketList    m_dataPackets;
  TxPacketList    m_routerPackets;
  TxFrameMap      m_txMacFrames;

  NodePhyMap      m_nodePhyEvents;
  
  PlotList        m_plotData;
  SummaryData     m_summaryData;
  double          m_currentScale;
  
}; // class Aggregator

/*
 * This class is used to setup a particular node call back interface
 * so that the node specific data can be provided in lieu of a 
 * generic callback that only addresses a single piece of data
 */
class NodeCallback {
   
public:
   NodeCallback( uint32_t id, Aggregator *aggregator ) :
      m_nodeId( id ),
      m_callAggregator( aggregator )
   {};
   
   ~NodeCallback() {};
   
   void PktTxEvent( Ptr<Packet const> initpkt ) {
      m_callAggregator->PacketTxDataCallback( initpkt);
   };

   void PktRxEvent( Ptr<Packet const> initpkt ) {
      m_callAggregator->PacketRxDataCallback( initpkt);
   };

   void RouterPacket( Ptr<Packet const> initpkt ) {
      m_callAggregator->PacketRouterCallback( initpkt);
   };
   
   void FrameTxStartEvent( Ptr<Packet const> initpkt ) {
      m_callAggregator->FrameTxStartCallback( m_nodeId, initpkt);
   };

   void FrameTxStopEvent( Ptr<Packet const> initpkt ) {
      m_callAggregator->FrameTxStopCallback( m_nodeId, initpkt);
   };

   void FrameRxStartEvent( Ptr<Packet const> initpkt ) {
      m_callAggregator->FrameRxStartCallback( m_nodeId, initpkt);
   };

   void FrameRxStopEvent( Ptr<Packet const> initpkt ) {
      m_callAggregator->FrameRxStopCallback( m_nodeId, initpkt);
   };
   
   uint32_t GetId() { return m_nodeId; };
   
private:
   uint32_t          m_nodeId;
   Aggregator  *m_callAggregator;
};

typedef std::list<NodeCallback>  NdCbList;

}

#endif /* AGGREGATOR_H */

