/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Università di Padova
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
 * Authors: Marco Romanelli <marco.romanelli.1@studenti.unipd.it>
 */

#ifndef FBNODE_H
#define FBNODE_H

#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/socket.h"
#include "ns3/vector.h"


namespace ns3
{

/**
 * \ingroup network
 * \brief A special node used in Fast Broadcast protocol.
 */
class FBNode : public Object
{
public:
  static TypeId GetTypeId (void);

  FBNode ();
  virtual ~FBNode ();

  /**
   * \returns the FBNode identifier
   */
  uint32_t GetId (void) const;

  /**
   * \returns the node included in the FBNode
   */
  Ptr<Node> GetNode (void) const;

  /**
   * \returns the value of the CMFR field
   */
  double GetCMFR (void) const;

  /**
   * \returns the value of the LMFR field
   */
  double GetLMFR (void) const;

  /**
   * \returns the value of the CMBR field
   */
  double GetCMBR (void) const;

  /**
   * \returns the value of the LMBR field
   */
  double GetLMBR (void) const;

  /**
   * \returns the position of the node
   */
  Vector GetPosition (void) const;

  /**
   * \returns the num of the node
   */
  uint32_t GetHop (void) const;

  /**
   * \returns the phase of the node
   */
  int32_t GetPhase (void) const;

  /**
   * \returns the cumulative backoff delay in milliseconds.
   */
  uint32_t GetWaitingTimeUs (void) const;

  /**
   * \returns true if the node has received an alert
   */
  bool GetReceived (void) const;

  /**
   * \returns true if the node has sent an alert
   */
  bool GetSent (void) const;

  /**
   * \returns true if the node is set as vehicle
   */
  bool AmIaVehicle (void) const;

  /**
   * \returns true if the node is inside an junction
   */
  bool AmIInJunction (void) const;

  /**
   * \returns the id of the junction the node is in
   */
  uint64_t GetJunctionId (void) const;


  bool GetStopSending (void) const;

  /**
   * \brief Get the propagation time in microseconds for this node
   * \returns propagation time in microseconds
   */
  int64_t GetPropagationTime () const;

  /**
   * \brief Get the timestamp when the alert message was sent
   * \returns time value when the message was sent
   */
  Time GetSendTimestamp () const;

  /**
   * \brief Get the timestamp when the alert message was received
   * \returns time value when the message was received
   */
  Time GetReceiveTimestamp () const;

  /**
   * \brief Check if the send timestamp has been set
   * \returns true if the send timestamp has been set
   */
  bool IsSendTimestampSet () const;

  /**
   * \brief Check if the receive timestamp has been set
   * \returns true if the receive timestamp has been set
   */
  bool IsReceiveTimestampSet () const;

  /**
   * \brief Set the timestamp when the alert message was received
   * \param t time value when the message was received
   */
  void SetReceiveTimestamp (Time t);

  /**
   * \brief set the node id
   * \param value id
   */
  void SetId (uint32_t value);

  /**
   * \brief set the node
   * \param node ns-3 node
   */
  void SetNode (Ptr<Node> node);

  /**
   * \brief set the socket of the node
   * \param socket internet socket
   */
  void SetSocket (Ptr<Socket> socket);

  /**
   * \brief set the value of the CMFR field
   * \param value new value of CMFR
   */
  void SetCMFR (double value);

  /**
   * \brief set the value of the LMFR field
   * \param value new value of LMFR
   */
  void SetLMFR (double value);

  /**
   * \brief set the value of the CMBR field
   * \param value new value of CMBR
   */
  void SetCMBR (double value);

  /**
   * \brief set the value of the LMBR field
   * \param value new value of LMBR
   */
  void SetLMBR (double value);

  /**
   * \brief update it's (node) current position
   * \returns the new position of the node
   */
  Vector UpdatePosition (void);

  /**
   * \brief set the num
   * \param value new value of num
   */
  void SetHop (uint32_t n);

  /**
   * \brief set the phase
   * \param value new value of phase
   */
  void SetPhase (int32_t value);

  /**
   * \brief set the cumulative backoff delay in milliseconds.
   * \param value new value of slot
   */
  void SetWaitingTimeUs (uint32_t value);

  /**
   * \brief set the received field
   * \param value boolean value
   */
  void SetReceived (bool value);

  /**
   * \brief set the sent field
   * \param value boolean value
   */
  void SetSent (bool value);

  /**
   * \brief send a packet
   * \param packet packet to send
   */
  void Send (Ptr<Packet> packet);

  /**
   * \brief set the node as a vehicle
   * \param value true if the node has to be set as a vehicles
   */
  void SetMeAsVehicle (bool value);

  /**
   * \brief set the node in junction
   * \param value true if the node is in an junction
   */
  void SetMeInJunction (bool value);

  /**
   * \brief set the id of the junction the node is in
   * \param JunctionId id of the junction the node is in
   */
  void SetJunctionId (uint64_t junctionId);

  void SetStopSending (bool stopSending);

  /**
   * \brief Set the propagation time in microseconds for this node
   * \param timeUs propagation time in microseconds
   */
  void SetPropagationTime (int64_t timeUs);

  /**
   * \brief Set the timestamp when the alert message was sent
   * \param t time value when the message was sent
   */
  void SetSendTimestamp (Time t);

  /**
   * \brief Manually set the send timestamp flag
   * \param value true if the send timestamp has been set
   */
  void SetSendTimestampSet (bool value);

  /**
   * \brief Manually set the receive timestamp flag
   * \param value true if the receive timestamp has been set
   */
  void SetReceiveTimestampSet (bool value);

  uint32_t    m_id;       // node id
  Ptr<Node>   m_node;     // ns-3 node
  Ptr<Socket> m_socket;   // ns-3 socket
  double      m_CMFR;     // Current Maximum Front Range
  double      m_LMFR;     // Last Maximum Front Range
  double      m_CMBR;     // Current Maximum Back Range
  double      m_LMBR;     // Last Maximum Back Range
  Vector      m_position; // node current position
  uint32_t    m_hop;      // number of hops before the alert message reached this node
  /**
   * \brief Logical phase of the message at this node.
   *
   * Used to control forwarding order in the alert message flooding protocol.
   * When a node receives a message, it compares its own phase with the
   * incoming message's phase to decide whether to defer forwarding.
   */
  int32_t     m_phase;

  /**
   * \brief Cumulative backoff delay (in microseconds) for this message.
   *
   * Each time the node schedules a message forward, the waiting time
   * is added to this value. Used to calculate slots average for statistics purposes.
   */
  uint32_t m_waitingTimeUs;

  /**
   * \brief Flag indicating whether the node has already received the message.
   *
   * Prevents duplicate processing and redundant forwarding.
   */
  bool m_received;

  /**
   * \brief Flag indicating whether the node has already forwarded the message.
   *
   * Prevents multiple forwards of the same message.
   */
  bool     m_sent;
  Time     m_sendTimestamp;    // Timestamp when the node sends the message
  Time     m_receiveTimestamp; // Timestamp when the node receives the message
  bool     m_receiveTimestampSet;
  bool     m_sendTimestampSet;
  int64_t  m_propTimeUs;    // propagation time in microseconds
  bool     m_amIaVehicle;   // used for statistics
  bool     m_amIInJunction; // whether the node is inside a junction
  uint64_t m_junctionId;    // id of the junction where the node is
  bool     m_stopSending;
};

} // namespace ns3

#endif /* FBNODE_H */
