/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef SENSOR_HELPER_H
#define SENSOR_HELPER_H

#include "ns3/sensor-netdevice.h"
#include "ns3/batsen-mac.h"
#include <ns3/node-container.h>
#include <ns3/lr-wpan-phy.h>
#include <ns3/sensor-mac.h>
#include <ns3/leach-mac.h>
#include <ns3/mac16-address.h>
#include <ns3/trace-helper.h>
#include <ns3/timer.h>
#include <ns3/position-allocator.h>
#include <map>

// basic file operations
#include <iostream>
#include <fstream>

namespace ns3 {

class SpectrumChannel;
class MobilityModel;

struct SensorTracker_s {
  public:
  SensorTracker_s() {
    received = false;
    start = Seconds( 0.0 );
    stop = Seconds( 0.0 );
  };
  
  bool  received;
  Time  start;
  Time  stop;
};

typedef std::map<uint32_t, SensorTracker_s *>   TrackMap_t;
typedef TrackMap_t::iterator                    TrackIterator_t;


/*
 * MAC Address && Success -
 *
 * Track the packets sent, and if received by the sink,
 * update the success status to TRUE
 */
typedef std::map< Mac16Address, bool > PktMap_t;
  
struct RoundTracker_s {
  public:
  RoundTracker_s() {
    clusters = 0.0;
    clusterMembers = 0.0;
    direct = 0.0;
    rxFrames = 0.0;
    txFrames = 0.0;
    duration = 0.0;
    pktList.clear();
  };
  
  // Track the number of forwarders per round
  double  clusters;
  double  clusterMembers;
  
  // Track the number of direct to sink transmitters in the round
  double  direct;
  double  rxFrames;
  double  txFrames;
  
  // Track the time per round to profile BW or Pkts / round
  Time    start;
  Time    stop;
  double  duration;
  
  // List of packets transmitted and possibly received in the round
  PktMap_t pktList;
};

typedef std::map<uint32_t, RoundTracker_s *>    RoundMap_t;
typedef RoundMap_t::iterator                    RoundIterator_t;
  
class PacketTracer
{
  PacketTracer();
  ~PacketTracer();
  
private:
  PktMap_t m_DataMap;
};

/**
 * \ingroup sensor
 *
 * \brief helps to manage and create IEEE 802.15.4 NetDevice objects
 *
 * This class can help to create IEEE 802.15.4 NetDevice objects
 * and to configure their attributes during creation.  It also contains
 * additional helper functions used by client code.
 *
 * Only one channel is created, and all devices attached to it.  If
 * multiple channels are needed, multiple helper objects must be used,
 * or else the channel object must be replaced.
 */

class SensorHelper : public PcapHelperForDevice, public AsciiTraceHelperForDevice
{
public:
  /**
   * \brief Create a Sensor helper in an empty state.  By default, a
   * SingleModelSpectrumChannel is created, with a
   * LogDistancePropagationLossModel and a ConstantSpeedPropagationDelayModel.
   *
   * To change the channel type, loss model, or delay model, the Get/Set
   * Channel methods may be used.
   */
  SensorHelper (void);
  
  /**
   * \brief Create a Sensor helper in an empty state with either a
   * SingleModelSpectrumChannel or a MultiModelSpectrumChannel.
   * \param useMultiModelSpectrumChannel use a MultiModelSpectrumChannel if true, a SingleModelSpectrumChannel otherwise
   *
   * A LogDistancePropagationLossModel and a
   * ConstantSpeedPropagationDelayModel are added to the channel.
   */
  SensorHelper (bool useMultiModelSpectrumChannel);
  
  virtual ~SensorHelper (void);
  
  bool OpenFile( std::string fname );
  
  /**
   * \brief Get the channel associated to this helper
   * \returns the channel
   */
  Ptr<SpectrumChannel> GetChannel (void);
  
  /**
   * \brief Set the channel associated to this helper
   * \param channel the channel
   */
  void SetChannel (Ptr<SpectrumChannel> channel);
  
  /**
   * \brief Set the channel associated to this helper
   * \param channelName the channel name
   */
  void SetChannel (std::string channelName);
  
  /**
   * \brief Set the default system power level for all nodes generated
   * \param double Power level in Joules
   */
  void SetDefaultPower( double pwr ) { m_power = pwr; };
  
  /**
   * \brief Set the deafult sensor MAC mode for the network
   * \param MacType_e
   */
  void SetNetworkType( MacType_e nettype ) { m_netType = nettype; };
  
  /**
   * \brief Add mobility model to a physical device
   * \param phy the physical device
   * \param m the mobility model
   */
  void AddMobility (Ptr<LrWpanPhy> phy, Ptr<MobilityModel> m);
  
  /**
   * \brief Install a SensorNetDevice and the associated structures (e.g., channel) in the nodes.
   * \param c a set of nodes
   * \returns A container holding the added net devices.
   */
  NetDeviceContainer Install (NodeContainer c);
  
  /**
   * Helper to enable all Sensor log components with one statement
   */
  void EnableLogComponents (void);
  
  /**
   * \brief Transform the LrWpanPhyEnumeration enumeration into a printable string.
   * \param e the LrWpanPhyEnumeration
   * \return a string
   */
  static std::string LrWpanPhyEnumerationPrinter (LrWpanPhyEnumeration e);
  
  /**
   * \brief Transform the SensorChnState enumeration into a printable string.
   * \param e the SensorHelperMacState
   * \return a string
   */
  static std::string SensorHelperMacChnStatePrinter (SensorChnState e);
  
  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model. Return the number of streams that have been
   * assigned. The Install() method should have previously been
   * called by the user.
   *
   * \param c NetDeviceContainer of the set of net devices for which the
   *          CsmaNetDevice should be modified to use a fixed stream
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this helper
   */
  int64_t AssignStreams (NetDeviceContainer c, int64_t stream);

  /**
   * Sets the packet rate for data delivery into the sensor network
   * once the simulator starts.  This packet rate is independant of 
   * the sensor's MAC establishing data transfer capability.  It is 
   * up to the MAC to determine if the Packet is queued or dropped.
   * 
   * It is assumed that the NetDeviceContainer passed into this 
   * method was created by the Install() method, wherein the first 
   * device is assigned to the Sink, and thus does not send data.
   * 
   * \param double packets per second rate, maybe be less than one
   * \param uint16_t basic PDU size of the packets
   * \param NetDeviceContainer container of all sensor devices to send from
   */
  void ConfigureDataRate( double rate, uint16_t size, NetDeviceContainer cntr );
  
  /**
   * Sets the maximum number of packets this simulation is allowed 
   * to execute. This avoid infinit packets which would keep the 
   * simulator running regardless of sensor net operation.
   * 
   * \param uint32_t maximum sequence number allowed.
   */
  void SetMaxSequence( uint32_t seq ) { m_maxSeq = seq; };
  
  /**
   * This method assigns random positions to the nodes
   * using the uniform random number generator. This 
   * should be called after the assign streams operation is 
   * complete; otherwise, we would not produce the same
   * arrangement per seed.
   * 
   * \param NetDeviceContainer Container of net devices we need to modify
   */
  void SetRandomPositions(NetDeviceContainer c);
  
  /**
   * Sets the maximum number of packets this simulation is allowed
   * to execute. This avoid infinit packets which would keep the
   * simulator running regardless of sensor net operation.
   *
   * \param uint32_t maximum sequence number allowed.
   * \param uint32_t maximum sequence number allowed.
   */
  void ClusterHeadReceptionCallback( Mac16Address source, uint32_t sequence );
  
  /**
   * Sets the maximum number of packets this simulation is allowed
   * to execute. This avoid infinit packets which would keep the
   * simulator running regardless of sensor net operation.
   *
   * \param uint32_t maximum sequence number allowed.
   * \param uint32_t maximum sequence number allowed.
   */
  void SinkReceptionCallback( std::list<uint16_t> nodes, std::list<uint32_t> sequence );

  /**
   * Sink callback method used to update the sink when a DC packet is received.
   *
   * \param Mac16Address Direct Connect node sending the data.
   * \param uint32_t     Sequence number of the packet.
   */
  void SinkDCReceptionCallback( Mac16Address dcAddr, uint32_t sequence );

  
  /**
   * Sets the maximum number of packets this simulation is allowed
   * to execute. This avoid infinit packets which would keep the
   * simulator running regardless of sensor net operation.
   *
   * \param uint32_t maximum sequence number allowed.
   * \param uint32_t maximum sequence number allowed.
   */
  void SensorDeadCallback( Mac16Address addr );
  
  /**
   * This method is called back when a frame is ready for transmit by
   * a sensor. The sequence number must be sent in order to add the 
   * SensorTracker_s * referenced by a valid sequence number in the 
   * std::map.
   * 
   * \param uint32_t sequence number of the data frame being transmitted
   */
  void DataTransmitCallback( uint32_t seq, Mac16Address address );
  
  /**
   * This is a callback method signifying that a node is considered a
   * forwarder (or CH for LEACH) in the next notional Epoch (round).
   * By calling this method, a node informs the SensorHelper that the 
   * node self-elected (LEACH) or was elected (OTHER) to be a forwarder
   * to the sink node for the given epoch.  The SensorHelper uses 
   * this to track the number of forwarders per round to check to see 
   * if the MAC is holding to the prescribed number of forwarders per epoch.
   * 
   * \param Mac16Address Address of the node acting as a forwarder
   */
  void ForwarderSelectedCallback( Mac16Address address );
  
  void DirectSelectCallback( Mac16Address address );
  
  /**
   * This callback method is used by the SINK to signal a new epoch, or
   * as in LEACH, a new round. This aligns all DT and CH selection
   * math to the epohc rather than to a floating point time value.
   * 
   * \param uint32_t Round number starting at 0
   */
  void NextRoundCallback( int round );
  
  void WhoisAlive();

private:
  
  // Disable implicit constructors
  /**
   * \brief Copy constructor - defined and not implemented.
   */
  SensorHelper (SensorHelper const &);
  
  /**
   * \brief Copy constructor - defined and not implemented.
   * \returns
   */
  SensorHelper& operator= (SensorHelper const &);
  
  /**
   * \brief Enable pcap output on the indicated net device.
   *
   * NetDevice-specific implementation mechanism for hooking the trace and
   * writing to the trace file.
   *
   * \param prefix Filename prefix to use for pcap files.
   * \param nd Net device for which you want to enable tracing.
   * \param promiscuous If true capture all possible packets available at the device.
   * \param explicitFilename Treat the prefix as an explicit filename if true
   */
  virtual void EnablePcapInternal (std::string prefix, Ptr<NetDevice> nd, bool promiscuous, bool explicitFilename);
  
  /**
   * \brief Enable ascii trace output on the indicated net device.
   *
   * NetDevice-specific implementation mechanism for hooking the trace and
   * writing to the trace file.
   *
   * \param stream The output stream object to use when logging ascii traces.
   * \param prefix Filename prefix to use for ascii trace files.
   * \param nd Net device for which you want to enable tracing.
   * \param explicitFilename Treat the prefix as an explicit filename if true
   */
  virtual void EnableAsciiInternal (Ptr<OutputStreamWrapper> stream,
                                    std::string prefix,
                                    Ptr<NetDevice> nd,
                                    bool explicitFilename);
    
  void SendData();

  void AnalyzeData();

  Ptr<SpectrumChannel> m_channel; //!< channel to be used for the devices
  
  double    m_power;
  MacType_e m_netType;
  uint16_t  m_nodeCount;
  
  uint16_t  m_liveNodes;
  
  uint32_t  m_maxSeq;
  uint16_t  m_pktSize;
  Time      m_interval;
  NetDeviceContainer m_sensors;
  NetDeviceContainer m_alive;
  
  TrackMap_t m_trackMap;
  RoundMap_t m_roundMap;
  RoundTracker_s *m_rndPtr;
  int   m_round;
  
  bool m_netAlive;
  
  /// Provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;
  
  Time m_firstFail, m_fiftyFail, m_lastFail;
  
  /*
   * _avg_ch - Collects the CH's / round on each row
   *         - graph AVG CH vs. round, where AVG across multiple runs for given 
   *           Max CH config. STDDEV nevers goes above MaxCH value
   *
   *           Store R1 R2 R3 ... RX <- where Rx = count of CHs at round X
   *
   * _life_ch - Collects the 3 pts: 1st death, mid pt death, last death
   *          - stored as three columns for plotting
   *
   */
  std::fstream m_avgFileLen;
  std::fstream m_avgChFile;
  std::fstream m_lifeFile;
  std::fstream m_rateFileLen;
  std::fstream m_rateFile;
  std::fstream m_roundFile;
  std::fstream m_duratFile;
  std::fstream m_duratFileLen;
  std::fstream m_dcFile;
  std::fstream m_dcFileLen;
  std::fstream m_pktFile;
  std::fstream m_pktFileLen;
  std::fstream m_txpktFile;
  std::fstream m_txpktFileLen;
  std::fstream m_pktPerFile;
  std::fstream m_pktPerFileLen;
  
  Ptr<UniformDiscPositionAllocator> m_discAllocator;
  
};

}


#endif /* SENSOR_HELPER_H */


