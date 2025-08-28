/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 University of Padova
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
 * Author: Marco Romanelli <marco.romanelli.1@studenti.unipd.it>
 *
 */

#include "FBApplication.h"
#include "FBHeader.h"
#include "FBNode.h"

#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "src/core/model/log.h"
#include <math.h>


using namespace std;


namespace ns3
{

NS_LOG_COMPONENT_DEFINE ("FBApplication");

NS_OBJECT_ENSURE_REGISTERED (FBApplication);

TypeId
FBApplication::GetTypeId (void)
{
  static TypeId tid =
    TypeId ("ns3::FBApplication").SetParent<Application> ().SetGroupName ("Network");

  return tid;
}

FBApplication::FBApplication () :
  m_messageSentTimes (),
  m_nNodes (0),
  m_startingNode (0),
  m_staticProtocol (false),
  m_broadcastPhaseStart (0),
  m_cwMin (32),
  m_cwMax (1024),
  m_flooding (true),
  m_actualRange (300),
  m_estimatedRange (0),
  m_aoi (m_actualRange * 2),
  m_aoi_error (0),
  m_packetPayload (100),
  m_received (0),
  m_sent (0),
  m_cwndSum (0),
  m_cwndCount (0),
  m_errorRate (0),
  m_forgedCoordRate (0),
  m_droneTest (0),
  m_collisions (0),
  m_printCoords (0),
  m_vehicleDistance (25),
  m_transmissionList (),
  m_transmissionVector ()
{
  NS_LOG_FUNCTION (this);

  // Todo: add a parameter for Seed and print it in the csv file
  RngSeedManager::SetSeed (12345);
}

FBApplication::~FBApplication ()
{
  NS_LOG_FUNCTION (this);
}

void
FBApplication::Install (uint32_t protocol,
                        uint32_t broadcastPhaseStart,
                        uint32_t actualRange,
                        uint32_t aoi,
                        uint32_t aoi_error,
                        bool     flooding,
                        uint32_t cwMin,
                        uint32_t cwMax,
                        uint32_t printCoords,
                        uint32_t vehicleDistance,
                        uint32_t errorRate,
                        uint32_t forgedCoordRate,
                        uint32_t droneTest)
{

  if (protocol == PROTOCOL_FB)
    {
      m_estimatedRange = PROTOCOL_FB;
      m_staticProtocol = false;
    }
  else if (protocol == PROTOCOL_STATIC_100)
    {
      m_estimatedRange = PROTOCOL_STATIC_100;
      m_staticProtocol = true;
    }
  else if (protocol == PROTOCOL_STATIC_300)
    {
      m_estimatedRange = PROTOCOL_STATIC_300;
      m_staticProtocol = true;
    }
  else if (protocol == PROTOCOL_STATIC_500)
    {
      m_estimatedRange = PROTOCOL_STATIC_500;
      m_staticProtocol = true;
    }
  else if (protocol == PROTOCOL_STATIC_700)
    {
      m_estimatedRange = PROTOCOL_STATIC_700;
      m_staticProtocol = true;
    }
  else
    {
      NS_LOG_ERROR ("Protocol not found.");
    }

  m_broadcastPhaseStart = broadcastPhaseStart;
  m_aoi                 = aoi;
  m_aoi_error           = aoi_error;
  m_actualRange         = actualRange;
  m_flooding            = flooding;
  m_cwMin               = cwMin;
  m_cwMax               = cwMax;
  m_printCoords         = printCoords;
  m_vehicleDistance     = vehicleDistance;
  m_errorRate           = errorRate;
  m_forgedCoordRate     = forgedCoordRate;
  m_droneTest           = droneTest;
  m_randomVariable      = CreateObject<UniformRandomVariable> ();
  // cout << "connect drop" << endl;
  // Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop",
  //                  MakeCallback (&FBApplication::LogCollision, this));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",
                   MakeCallback (&FBApplication::LogCollision, this));

  // Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiMac/MacRxDrop",
  //                  MakeCallback (&FBApplication::LogCollision, this));
  // Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiPhy/PhyRxDrop",
  //                  MakeCallback (&FBApplication::LogCollision, this));
}

void
FBApplication::LogCollision (std::string context, Ptr<const Packet> p)
{
  m_collisions++;
}

void
FBApplication::AddNode (Ptr<Node>   node,
                        Ptr<Socket> source,
                        Ptr<Socket> sink,
                        bool        onstats,
                        bool        isNodeInJunction,
                        uint64_t    junctionId)
{
  NS_LOG_FUNCTION (this << node);

  Ptr<FBNode> fbNode = CreateObject<FBNode> ();
  fbNode->SetNode (node);
  fbNode->SetId (node->GetId ());
  fbNode->SetSocket (source);
  sink->SetRecvCallback (MakeCallback (&FBApplication::ReceivePacket, this));
  fbNode->SetCMFR (m_estimatedRange);
  fbNode->SetLMFR (m_estimatedRange);
  fbNode->SetCMBR (m_estimatedRange);
  fbNode->SetLMBR (m_estimatedRange);
  fbNode->UpdatePosition ();
  fbNode->SetHop (0);
  fbNode->SetPhase (-1);
  fbNode->SetSlot (0);
  fbNode->SetReceived (false);
  fbNode->SetSent (false);
  fbNode->SetMeAsVehicle (onstats);
  // cout << "AddNode id= " << node->GetId () << " onstats= " << onstats << endl;

  fbNode->SetMeInJunction (isNodeInJunction);
  fbNode->SetJunctionId (junctionId);
  // if (fbNode->AmIInIntersection ())
  //   {
  //     cout << "node " << fbNode->GetId () << " is inside intersection "
  //          << fbNode->GetIntersectionId () << endl;
  //   }

  // misc stuff
  m_nodes.push_back (fbNode);
  m_id2id[fbNode->GetId ()] = m_nodes.size () - 1;
  m_nNodes++;
}


void
FBApplication::PrintStats (std::stringstream& dataStream)
{

  NS_LOG_FUNCTION (this);
  // cout << "cwndAvg " << (m_cwndSum / m_cwndCount) << endl;
  cout << "collisions= " << m_collisions << endl;
  uint32_t totalCoverage = 1; // All nodes reached by the alert, included the source node
  uint32_t coverVehicles = 1; // All vehicles reached bu the alert message, included the
                              // one that originated the alert

  /**
   * Number of nodes on the circumference that received the alert message
   */
  uint32_t coverageOnCirc = 0;
  /**
   * Number of nodes on the circumference
   */
  uint32_t nodesOnCirc = 0;
  // cout << "PrintStats area " << m_aoi << endl;
  double   radiusMin = m_aoi - m_aoi_error;
  double   radiusMax = m_aoi + m_aoi_error;

  long double timeSum                = 0;
  long double hopsSum                = 0;
  long double slotsSum               = 0;
  int         validTimeSamples       = 0;
  long double globalTimeSum          = 0;
  int         validGlobalTimeSamples = 0;

  stringstream receivedOnCircIds;

  // Time when the first alert message was sent
  Time firstMessageSentTime = this->GetFBNode (m_startingNode)->GetSendTimestamp ();

  for (uint32_t i = 0; i < m_nNodes; i++)
    {
      Ptr<FBNode> current = m_nodes.at (i);
      uint32_t    nodeId  = current->GetId ();

      // Skip the starting node
      if (nodeId == m_startingNode)
        {
          continue;
        }

      // Update the total coverage value
      if (current->GetReceived ())
        {
          // cout << "cover++" << endl;
          totalCoverage++;
          if (current->AmIaVehicle ())
            {
              coverVehicles++;
            }
        }

      // Compute cover on circumference of radius m_aoi
      Ptr<FBNode> startingNode = this->GetFBNode (m_startingNode);

      Vector currentPosition      = current->GetPosition ();
      Vector startingNodePosition = startingNode->GetPosition ();

      double distance = ns3::CalculateDistance (currentPosition, startingNodePosition);

      // Check if the current vehicle is in the circumference and within the range
      if ((distance >= radiusMin) && (distance <= radiusMax))
        {
          // Update the number of vehicles in the circumference
          nodesOnCirc++;

          // Update the cover value
          if (current->GetReceived ())
            {
              coverageOnCirc++;
              receivedOnCircIds << current->GetId () << "_";
              // Per-hop propagation
              int64_t propTime = current->GetPropagationTime ();
              if (propTime > 0) // Only count valid times
                {
                  timeSum += propTime;
                  validTimeSamples++;
                }
              // Global delay tracks in us the time that the alert msg took to reach the
              // node from source to circumference
              int64_t globalDelay = current->GetReceiveTimestamp ().GetMicroSeconds () -
                                    firstMessageSentTime.GetMicroSeconds ();
              if (globalDelay >= 0)
                {
                  globalTimeSum += globalDelay;
                  validGlobalTimeSamples++;
                }
              else
                {
                  // NS_LOG_WARN ("Negative global delay for node " << current->GetId ());
                  cout << "Negative global delay for node " << current->GetId () << endl;
                }

              // Update mean time, nums and slots
              // cout << "current get hop= " << current->GetHop () << endl;
              hopsSum  += current->GetHop ();
              slotsSum += current->GetSlot ();
            }
        }
    }

  string       receivedNodes = StringifyVector (m_receivedNodes);
  stringstream nodeIds;

  for (auto i = m_nodes.begin (); i != m_nodes.end (); ++i)
    {
      uint32_t id = (*i)->GetId ();
      nodeIds << id << "_";
      // Vector pos = (*i)->GetPosition ();
      // if (pos.x > 1050.0 && pos.x < 1250.0 && pos.y > 1800.0 && pos.y < 1900.0)
      //   {
      //     cout << id << endl;
      //   }
    }

  //	Alert received mean time: average time the alert took to reach the node on the
  // circumference
  double avgGlobalDelay =
    (validGlobalTimeSamples > 0) ? (globalTimeSum / validGlobalTimeSamples) : 0;
  //  The time the node took to traverse the last hop
  double avgPropTime = (validTimeSamples > 0) ? (timeSum / validTimeSamples) : 0;
  double avgHops     = hopsSum / (double)coverageOnCirc;
  double avgSlots    = slotsSum / (double)coverageOnCirc;
  // These will go into csv file
  dataStream << nodesOnCirc << "," << totalCoverage << "," << coverageOnCirc << ","
             << avgGlobalDelay << "," << avgHops << "," << avgSlots << "," << m_sent
             << "," << m_received << "," << m_collisions;

  NS_LOG_DEBUG ("totalCoverage = " << totalCoverage << "/" << m_nNodes);
  cout << "totalCoverage = " << totalCoverage << "/" << m_nNodes << endl;
  cout << "coverageOnCirc = " << coverageOnCirc << "/" << nodesOnCirc << endl;
  cout << "m_sent = " << m_sent << endl;
  cout << "hops= " << avgHops << endl;
  cout << "slots= " << avgSlots << endl;
  cout << "Alert received mean time = " << avgGlobalDelay << endl;
  cout << "Per-hop propagation time = " << avgPropTime << endl;
  cout << "First message sent time (firstMessageSentTime): "
       << firstMessageSentTime.GetMicroSeconds () << " µs" << std::endl;
  cout << "Starting node timestamp (GetSendTimestamp): "
       << GetFBNode (m_startingNode)->GetSendTimestamp ().GetMicroSeconds () << " µs"
       << std::endl;
  cout << "m_startingNode = " << m_startingNode << endl;


  // cout << "hops sum= " << hopsSum << " circ= " << circ
  //      << " hops= " << (hopsSum / (double)circ) << endl;

  if (m_printCoords)
    {
      Ptr<FBNode> startingNode       = GetFBNode (m_startingNode);
      string      transmissionVector = StringifyVector (m_transmissionVector);
      // cout << "FBApplication::PrintStats coords" << endl;
      // cout << "receivedNodes" << endl;
      // cout << receivedNodes << endl;
      // cout << "nodeIds" << endl;
      // cout << nodeIds.str () << endl;
      // cout << "transmissionMap" << endl;
      // cout << StringifyTransmissionMap () << endl;
      // cout << "receivedOnCircIds" << endl;
      // cout << receivedOnCircIds.str () << endl;
      // cout << "transmissionVector" << endl;
      // cout << transmissionVector << endl;

      dataStream << "," << startingNode->GetPosition ().x << ","
                 << startingNode->GetPosition ().y << "," << m_startingNode << ","
                 << m_vehicleDistance << "," << receivedNodes << "," << nodeIds.str ()
                 << "," << StringifyTransmissionMap () << "," << receivedOnCircIds.str ()
                 << "," << transmissionVector;
    }
  if (m_droneTest)
    {
      double   maxDistance            = 0;
      uint32_t maxDistanceNodeReached = IsMaxDistNodeReached (maxDistance);
      dataStream << "," << maxDistance << "," << maxDistanceNodeReached << ","
                 << coverVehicles;
    }


  // NS_LOG_UNCOND ("aoi = " << m_aoi << "aoi error " << m_aoi_error);
}

void
FBApplication::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  // Get startingNode as node and as fbNode
  m_startingNode = this->GetNode ()->GetId ();

  if (m_id2id.count (m_startingNode) == 0)
    {
      NS_LOG_ERROR ("Starting node is not a fb node!");
    }

  if (!m_staticProtocol)
    {
      // Start Estimation Phase
      NS_LOG_INFO ("Start Estimation Phase.");
      if (m_forgedCoordRate > 0)
        {
          NS_LOG_INFO ("Start Forged Messages Generation Phase.");
          GenerateForgedHelloTraffic ();
        }
      // GenerateHelloTraffic (1);
      GenerateHelloTraffic (5);
    }
  // Schedule Broadcast Phase
  Simulator::Schedule (
    Seconds (m_broadcastPhaseStart), &FBApplication::StartBroadcastPhase, this);
}

void
FBApplication::StopApplication (void)
{
  NS_LOG_FUNCTION (this);
}

void
FBApplication::GenerateForgedHelloTraffic ()
{
  cout << m_nNodes << endl;
  uint32_t nAffectedNodes = m_nNodes * ((double)m_forgedCoordRate / 100);
  NS_LOG_INFO ("GeneratedForgedHelloTraffic affected " << nAffectedNodes << " nodes");

  uint32_t      forgedSenderNodeId = m_nNodes;
  set<uint32_t> affectedNodes;
  while (affectedNodes.size () < nAffectedNodes)
    {
      uint32_t nodeId = m_randomVariable->GetInteger (0, m_nNodes - 1);
      affectedNodes.insert (nodeId);
    }

  for (auto id : affectedNodes)
    {
      double startingX = m_nodes[id]->UpdatePosition ().x + m_actualRange; // low sev
      double startingY = m_nodes[id]->UpdatePosition ().y;
      // double startingX = 10000; // high sev
      for (uint32_t i = 1; i < 151; i++)
        {
          Vector position = Vector (startingX + i, startingY, 0);

          FBHeader fbHeader;
          fbHeader.SetType (HELLO_MESSAGE);
          fbHeader.SetMaxRange (m_actualRange + i);
          fbHeader.SetStarterPosition (position);
          fbHeader.SetPosition (position);
          fbHeader.SetSenderId (forgedSenderNodeId + i); // added
          fbHeader.SetSenderInJunction (false);
          fbHeader.SetJunctionId (0);
          HandleHelloMessage (m_nodes.at (id), fbHeader);
        }
    }
}

void
FBApplication::GenerateHelloTraffic (uint32_t count)
{

  // NS_LOG_INFO (this << count);
  NS_LOG_INFO ("GenerateHelloTraffic" << count);
  NS_LOG_DEBUG ("GenerateHelloTraffic " << count);
  std::vector<int> he;
  uint32_t         hel = (int)m_nNodes / 100 * 50; // 50% of total nodes

  // uint32_t hel = (int)m_nNodes; // 100% of total nodes
  uint32_t time_factor = 10;
  //	cout << "hel= " << hel << endl;
  if (count > 0)
    {
      for (uint32_t i = 0; i < hel; i++)
        {
          // int pos = rand () % m_nNodes;
          int pos = m_randomVariable->GetInteger (0, m_nNodes - 1);
          he.push_back (pos);
          Ptr<FBNode> fbNode = m_nodes.at (pos);
          Simulator::ScheduleWithContext (fbNode->GetNode ()->GetId (),
                                          MicroSeconds (i * time_factor),
                                          &FBApplication::GenerateHelloMessage,
                                          this,
                                          fbNode);
          // Ptr<FBNode> fbNode = m_nodes.at (i);
          // Simulator::ScheduleWithContext (fbNode->GetNode ()->GetId (),
          //                                 MicroSeconds (i * time_factor),
          //                                 &FBApplication::GenerateHelloMessage,
          //                                 this,
          //                                 fbNode);
        }

      // Other nodes must send Hello messages
      double s = ceil ((hel * time_factor) / 1000000.0);
      // auto start = std::chrono::system_clock::now ();
      // std::time_t start_time = std::chrono::system_clock::to_time_t (start);
      Simulator::Schedule (
        Seconds (s), &FBApplication::GenerateHelloTraffic, this, count - 1);
    }
}

void
FBApplication::StartBroadcastPhase (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO ("Start Broadcast Phase.");

  Ptr<FBNode> fbNode = this->GetFBNode (m_startingNode);

  // Generate the first alert message
  GenerateAlertMessage (fbNode);
  if (m_errorRate > 0)
    {
      Simulator::Schedule (MilliSeconds (1),
                           &FBApplication::GenerateAlertMessage,
                           this,
                           m_nodes.at (m_startingNode));
    }
}

void
FBApplication::GenerateHelloMessage (Ptr<FBNode> fbNode)
{
  NS_LOG_FUNCTION (this << fbNode);
  NS_LOG_DEBUG ("GenerateHelloMessage (" << fbNode->GetId () << ") "
                                         << "at time= " << Simulator::Now ().GetSeconds ()
                                         << "s");
  // NS_LOG_DEBUG ("Generate Hello Message (" << fbNode->GetNode ()->GetId () << ").");

  // Create a packet with the correct parameters taken from the node
  Vector   position = fbNode->UpdatePosition ();
  FBHeader fbHeader;
  fbHeader.SetType (HELLO_MESSAGE);
  fbHeader.SetMaxRange (fbNode->GetCMBR ());
  fbHeader.SetStarterPosition (position);
  fbHeader.SetPosition (position);

  fbHeader.SetSenderId (fbNode->GetId ()); // added

  fbHeader.SetSenderInJunction (false);
  fbHeader.SetJunctionId (0);


  Ptr<Packet> packet = Create<Packet> (m_packetPayload);
  packet->AddHeader (fbHeader);

  fbNode->Send (packet);
}

void
FBApplication::GenerateAlertMessage (Ptr<FBNode> fbNode)
{
  NS_LOG_FUNCTION (this << fbNode);
  NS_LOG_DEBUG ("Generate Alert Message (" << fbNode->GetNode ()->GetId () << ").");

  // Create a packet with the correct parameters taken from the node
  uint32_t LMBR, CMBR, maxi;
  LMBR            = fbNode->GetLMBR ();
  CMBR            = fbNode->GetCMBR ();
  maxi            = std::max (LMBR, CMBR);
  Vector position = fbNode->UpdatePosition ();

  FBHeader fbHeader;
  fbHeader.SetType (ALERT_MESSAGE);
  fbHeader.SetMaxRange (maxi);
  fbHeader.SetStarterPosition (position);
  fbHeader.SetPosition (position);
  fbHeader.SetPhase (0);
  fbHeader.SetSlot (0);

  fbHeader.SetSenderId (fbNode->GetId ());
  fbHeader.SetSenderInJunction (fbNode->AmIInJunction ());
  fbHeader.SetJunctionId (fbNode->GetJunctionId ());

  Ptr<Packet> packet = Create<Packet> (m_packetPayload);
  packet->AddHeader (fbHeader);

  fbNode->Send (packet);
  fbNode->SetSent (true);
  m_sent++;

  // Store current time for this sender
  Time currentTime                     = Simulator::Now ();
  m_messageSentTimes[fbNode->GetId ()] = currentTime;

  // Only the origin node sets the send timestamp
  if (!fbNode->IsSendTimestampSet ())
    {
      fbNode->SetSendTimestamp (currentTime);
    }
  fbNode->SetReceived (true);
}

void
FBApplication::ReceivePacket (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);

  // Get the node who received this message and the corresponding FBNode
  Ptr<Node>   node   = socket->GetNode ();
  Ptr<FBNode> fbNode = GetFBNode (node);

  Ptr<Packet> packet;
  Address     senderAddress;

  while ((packet = socket->RecvFrom (senderAddress)))
    {
      FBHeader fbHeader;
      packet->RemoveHeader (fbHeader);
      Vector currentPosition = fbNode->UpdatePosition ();
      // if (fbHeader.GetType () == ALERT_MESSAGE && currentPosition.z > 0)
      //   {
      //     cout << "ricevuto alert da drone in pos " << currentPosition << endl;
      //   }
      // Get the position of the sender node
      Vector senderPosition = fbHeader.GetPosition ();
      double distance       = ns3::CalculateDistance (currentPosition, senderPosition);
      if (m_forgedCoordRate == 0 && distance > m_actualRange)
        {
          continue;
        }
      // cout << "received packet at distance= "
      //      << ns3::CalculateDistance (currentPosition, senderPosition) << endl;
      // NS_LOG_DEBUG ("Packet received by node " << node->GetId () << " from node "
      //                                          << fbHeader.GetSenderId () << ".");

      // Get the type of the message (Hello or Alert)
      uint32_t messageType = fbHeader.GetType ();
      //
      if (messageType == HELLO_MESSAGE)
        {
          HandleHelloMessage (fbNode, fbHeader);
        }
      else if (messageType == ALERT_MESSAGE)
        {
          HandleAlertMessage (fbNode, fbHeader);

          // m_received++;
          // // Get the phase
          // int32_t phase = fbHeader.GetPhase ();

          // // Get the position of the node who start the broadcast
          // Vector starterPosition = fbHeader.GetStarterPosition ();

          // // Compute the two distances
          // double distanceSenderToStarter =
          //   ns3::CalculateDistance (senderPosition, starterPosition);
          // double distanceCurrentToStarter =
          //   ns3::CalculateDistance (currentPosition, starterPosition);

          // // If starter - to - sender distance is less than starter - to - current
          // // distance,
          // //   then the message is coming from the front and it needs to be managed,
          // //   otherwise do nothing
          // if (distanceCurrentToStarter > distanceSenderToStarter &&
          //     !fbNode->GetReceived ())
          //   {

          //     // Store when the current has received the first packet
          //     fbNode->SetTimestamp (Simulator::Now ());

          //     uint32_t sl = fbHeader.GetSlot ();
          //     fbNode->SetSlot (fbNode->GetSlot () + sl);
          //     fbNode->SetReceived (true);

          //     uint32_t senderId   = fbHeader.GetSenderId ();
          //     uint32_t receiverId = fbNode->GetId ();

          //     m_receivedNodes.push_back (fbNode->GetId ());
          //     auto it = m_transmissionList.find (senderId);
          //     if (it == m_transmissionList.end ())
          //       {
          //         //	cout << "senderId = " << senderId << endl;
          //         m_transmissionList[senderId] = vector<uint32_t> ();
          //       }
          //     m_transmissionList[senderId].push_back (receiverId);
          //     m_transmissionVector.push_back (Edge (senderId, receiverId, phase));

          //     if (fbNode->GetNum () == 0)
          //       {
          //         fbNode->SetNum (phase);
          //       }

          //     // check if the message is coming from the front
          //     if (phase > fbNode->GetPhase ())
          //       {
          //         fbNode->SetPhase (phase);
          //         //
          //         m_transmissionList.AddEdge (KeyableVector (fbHeader.GetPosition ()),
          //                                     / KeyableVector (fbNode->GetPosition
          //                                     ()));
          //         HandleAlertMessage (fbNode, fbHeader, distanceSenderToCurrent_uint);
          //       }
          //   }
          // else
          //   {
          //   }
        }
    }
}

void
FBApplication::HandleHelloMessage (Ptr<FBNode> fbNode, FBHeader fbHeader)
{
  NS_LOG_FUNCTION (this << fbNode << fbHeader);

  uint32_t nodeId = fbNode->GetNode ()->GetId ();
  NS_LOG_DEBUG ("Handle a Hello Message (" << nodeId << ").");

  // Retrieve CMFR from the packet received and CMBR from the current node
  double otherCMFR = fbHeader.GetMaxRange ();
  double myCMBR    = fbNode->GetCMBR ();

  // Retrieve the position of the current node
  Vector currentPosition = fbNode->UpdatePosition ();

  // Retrieve the position of the sender node
  Vector senderPosition = fbHeader.GetPosition ();

  // Compute distance
  double distance = ns3::CalculateDistance (senderPosition, currentPosition);
  NS_LOG_DEBUG ("Detected distance: " << distance);

  // Update new values
  double maxi = std::max (std::max (myCMBR, otherCMFR), distance);
  NS_LOG_DEBUG ("myCMBR " << myCMBR << "m");
  NS_LOG_DEBUG ("otherCMFR " << otherCMFR << "m");
  NS_LOG_DEBUG ("Setting CMBR to " << maxi << "m");
  fbNode->SetCMBR (maxi);

  // Override the old values
  fbNode->SetLMBR (myCMBR);
}

void
FBApplication::HandleAlertMessage (Ptr<FBNode> fbNode, FBHeader fbHeader)
{
  int32_t  phase           = fbHeader.GetPhase ();
  Vector   currentPosition = fbNode->UpdatePosition ();
  // Get the position of the sender node
  Vector   senderPosition = fbHeader.GetPosition ();
  uint32_t senderId       = fbHeader.GetSenderId ();
  uint32_t receiverId     = fbNode->GetId ();
  NS_LOG_DEBUG ("Packet received by node " << fbNode->GetId () << " from node "
                                           << senderId << ".");

  // Compute the distance between the sender and me (the node who received the message)
  double distanceSenderToCurrent =
    ns3::CalculateDistance (senderPosition, currentPosition);
  if (distanceSenderToCurrent > m_actualRange + 100)
    {
      return;
    }
  // Get the position of the node who start the broadcast
  // Vector starterPosition = fbHeader.GetStarterPosition ();

  // Compute the two distances
  // double distanceSenderToStarter =
  //   ns3::CalculateDistance (senderPosition, starterPosition);
  // double distanceCurrentToStarter =
  //   ns3::CalculateDistance (currentPosition, starterPosition);

  // Message coming from the back

  if (fbNode->AmIInJunction ())
    {
      NS_LOG_LOGIC ("node " << fbNode->GetId ()
                            << "is inside a junction and has received an alert message");
      // I am in a junction and I receive a message from the same junction -> I have to
      // defer transmission
      if (fbHeader.IsSenderInJunction () &&
          fbNode->GetJunctionId () == fbHeader.GetJunctionId ())
        {
          if (phase > fbNode->GetPhase ())
            {
              fbNode->SetPhase (phase);
              // NS_LOG_LOGIC ("node " << node->GetId ()
              //                       << "is inside a junction: updates phase from "
              //                       << node->GetPhase () << " to " << phase);
            }
        }
    }
  else
    {
      // I am not in a junction and I receive a message from a node farther than me -> I
      // have to defer tranmission
      // if ((phase > fbNode->GetPhase ()) &&
      //     (distanceSenderToStarter > distanceCurrentToStarter))
      //   {
      if (phase > fbNode->GetPhase ()) // todo abilitare per urbano
        {
          fbNode->SetPhase (phase);
          // NS_LOG_LOGIC ("node " << node->GetId ()
          //                       << "is not inside a junction: updates phase from "
          //                       << node->GetPhase () << " to " << phase);
        }
    }
  if (fbNode->GetReceived ())
    {
      return;
    }

  if (!fbNode->GetReceived ())
    {
      fbNode->SetReceived (true);
      Time receiveTime = Simulator::Now ();
      fbNode->SetReceiveTimestamp (receiveTime);

      // Calculate propagation time if sender time is available
      uint32_t senderId = fbHeader.GetSenderId ();
      if (m_messageSentTimes.find (senderId) != m_messageSentTimes.end ())
        {
          Time sentTime = m_messageSentTimes[senderId];
          Time propTime = receiveTime - sentTime;
          fbNode->SetPropagationTime (propTime.GetMicroSeconds ());

          NS_LOG_DEBUG ("Message from " << senderId << " to " << fbNode->GetId ()
                                        << " took " << propTime.GetMicroSeconds ()
                                        << " microseconds");
        }
      fbNode->SetSlot (fbHeader.GetSlot ());
      fbNode->SetHop (phase + 1);
      fbNode->SetPhase (phase);
      m_received++;
      // save transmission for stats and metrics
      m_receivedNodes.push_back (receiverId);
      auto it = m_transmissionList.find (senderId);
      if (it == m_transmissionList.end ())
        {
          m_transmissionList[senderId] = vector<uint32_t> ();
        }
      m_transmissionList[senderId].push_back (receiverId);
      m_transmissionVector.push_back (Edge (senderId, receiverId, phase));
    }

  // If starter-to-sender distance is less than starter-to-current distance,
  // then the message is coming from the front and it needs to be managed,
  // otherwise do nothing

  // if (distanceCurrentToStarter <= distanceSenderToStarter)
  //   { // todo togliere
  //     return;
  //     NS_LOG_DEBUG ("Alert message received by "
  //                   << fbNode->GetId () << " from node " << fbHeader.GetSenderId ()
  //                   << " is being considered for forwarding since "
  //                      "distanceCurrentToStarter > distanceSenderToStarter "
  //                   << distanceCurrentToStarter << " > " << distanceSenderToStarter);
  //   }

  // Compute the size of the contention window
  double   bmr    = fbNode->GetCMBR ();
  uint32_t cwnd   = ComputeContentionWindow (bmr, distanceSenderToCurrent);
  uint32_t cwndUs = cwnd * 1000; // convert cwnd from ms to µs

  // We randomize cwnd with a spread so that the waiting time is loyal to the wanted value
  uint32_t spreadUs = 960;

  // Pick random waiting time in microseconds
  uint32_t maxCwndUs     = cwndUs + spreadUs;
  uint32_t waitingTimeUs = m_randomVariable->GetInteger (cwndUs, maxCwndUs);
  NS_LOG_LOGIC ("Cwnd: " << cwndUs << " maxCwndUs: " << maxCwndUs
                         << " waitingTimeUs: " << waitingTimeUs);
  cout << "(Node: " << receiverId << ") "
       << " Waiting time in microseconds: " << waitingTimeUs << "µs" << endl;

  int32_t errorDelay = ComputeErrorDelay ();
  //		cout << "errorDelay= " << errorDelay << endl;
  if (!m_flooding)
    {
      if (errorDelay == 0)
        {
          Simulator::Schedule (MicroSeconds (waitingTimeUs),
                               &FBApplication::ForwardAlertMessage,
                               this,
                               fbNode,
                               fbHeader,
                               waitingTimeUs / 1000,
                               false);
        }
      else // Todo: convert scheduler calls to MicroSeconds and possibly use only
           // waitingTimeUs and convert slots to milliseconds when printing stats
        {
          uint32_t firstTransmissionTime;
          uint32_t secondTransmissionTime;
          firstTransmissionTime =
            errorDelay > 0 ? waitingTimeUs : waitingTimeUs + errorDelay;
          secondTransmissionTime =
            errorDelay < 0 ? waitingTimeUs : waitingTimeUs + errorDelay;
          Simulator::Schedule (MicroSeconds (firstTransmissionTime),
                               &FBApplication::ForwardAlertMessage,
                               this,
                               fbNode,
                               fbHeader,
                               firstTransmissionTime / 1000,
                               false);
          Simulator::Schedule (MicroSeconds (secondTransmissionTime),
                               &FBApplication::ForwardAlertMessage,
                               this,
                               fbNode,
                               fbHeader,
                               secondTransmissionTime / 1000,
                               true);
        }
    }
  else // flooding
    {
      Simulator::Schedule (MilliSeconds (0),
                           &FBApplication::ForwardAlertMessage,
                           this,
                           fbNode,
                           fbHeader,
                           waitingTimeUs / 1000, // Todo: Why? Shouldn't it be 0?
                           false);
    }
  //	}
}


void
FBApplication::WaitAgain (Ptr<FBNode> fbNode, FBHeader fbHeader, uint32_t waitingTime)
{
  // Todo: waitingTime should be microseconds
  NS_LOG_FUNCTION (this);

  // // Get the phase
  // int32_t phase = fbHeader.GetPhase ();

  // if (phase >= fbNode->GetPhase ())
  //   {
  //     uint32_t rnd = (rand () % 20) + 1;
  //     uint32_t rnd1 = (rand () % 20) + 1;
  //     uint32_t rnd2 = (rand () % 20) + 1;
  //     uint32_t rnd3 = (rand () % 20) + 1;
  //     Simulator::Schedule (
  //       MilliSeconds (10 * (waitingTime + rnd + rnd1 + rnd2 + rnd3) * 200 * 3),
  //       / &FBApplication::ForwardAlertMessage,
  //       this,
  //       fbNode,
  //       fbHeader,
  //       waitingTime);
  //   }
}

void
FBApplication::ForwardAlertMessage (
  Ptr<FBNode> fbNode,
  FBHeader    oldFBHeader,
  uint32_t    waitingTime, // Todo: waitingTime should be in microseconds all the way and
                           // then converted in millisecond for statistics.
  bool        forceSend)
{
  NS_LOG_FUNCTION (this << fbNode << oldFBHeader);
  // Get the phase
  int32_t phase    = oldFBHeader.GetPhase ();
  Vector  oldPos   = oldFBHeader.GetPosition ();
  Vector  position = fbNode->UpdatePosition ();
  double  distance = ns3::CalculateDistance (position, oldPos);
  if (fbNode->GetStopSending ())
    {
      NS_LOG_DEBUG ("node " << fbNode->GetId () << " defers because of StopSending");
      return;
    }
  if (!(fbNode->GetSent () && forceSend))
    {
      if (fbNode->GetSent ())
        {
          NS_LOG_DEBUG ("node " << fbNode->GetId () << " defers because of GetSent");
          return;
        }
      // If I'm not the first to wake up, I must not forward the message
      if (!m_flooding && fbNode->GetPhase () > phase)
        {
          NS_LOG_DEBUG ("node " << fbNode->GetId () << " defers because of phase");
          return;
        }
    }
  if (forceSend)
    {
      fbNode->SetStopSending (true);
    }
  NS_LOG_DEBUG ("Forwarding Alert Message (Node: "
                << fbNode->GetNode ()->GetId () << "at pos " << fbNode->GetPosition ()
                << ") after " << waitingTime << "ms at distance= " << distance << ".");

  // Create a packet with the correct parameters taken from the node

  uint32_t LMBR, CMBR, maxi;
  LMBR = fbNode->GetLMBR ();
  CMBR = fbNode->GetCMBR ();
  maxi = std::max (LMBR, CMBR);

  // Vector position = fbNode->UpdatePosition ();
  Vector starterPosition = oldFBHeader.GetStarterPosition ();

  FBHeader fbHeader;
  fbHeader.SetType (ALERT_MESSAGE);
  fbHeader.SetMaxRange (maxi);
  fbHeader.SetStarterPosition (starterPosition);
  fbHeader.SetPosition (position);
  fbHeader.SetPhase (phase + 1);
  fbHeader.SetSlot (fbNode->GetSlot () + waitingTime);
  fbHeader.SetSenderId (fbNode->GetId ());
  fbHeader.SetSenderInJunction (fbNode->AmIInJunction ());
  fbHeader.SetJunctionId (fbNode->GetJunctionId ());

  Ptr<Packet> packet = Create<Packet> (m_packetPayload);
  packet->AddHeader (fbHeader);

  // Forward
  fbNode->Send (packet);
  fbNode->SetSent (true);

  // Store sent time for this forwarded message
  m_messageSentTimes[fbNode->GetId ()] = Simulator::Now ();

  m_sent++;
  // else
  // {
  //   cout << "de Ferro distance= " << distance << " waitingTime= " << waitingTime <<
  //   endl;
  // }
}

void
FBApplication::StopNode (Ptr<FBNode> fbNode)
{
  NS_LOG_FUNCTION (this);

  Ptr<Node> node = fbNode->GetNode ();

  Ptr<ConstantVelocityMobilityModel> mob =
    node->GetObject<ConstantVelocityMobilityModel> ();
  mob->SetVelocity (Vector (0, 0, 0));
}

Ptr<FBNode>
FBApplication::GetFBNode (Ptr<Node> node)
{
  NS_LOG_FUNCTION (this);

  if (m_id2id.count (node->GetId ()) == 0)
    {
      // We got a problem: key not found
      NS_LOG_ERROR ("Error: key for node " << node->GetId ()
                                           << " not found in fb application.");
    }

  return this->GetFBNode (node->GetId ());
}

Ptr<FBNode>
FBApplication::GetFBNode (uint32_t id)
{
  NS_LOG_FUNCTION (this);

  if (m_id2id.count (id) == 0)
    {
      // We got a problem: key not found
      NS_LOG_ERROR ("Error: key for node " << id << " not found in fb application.");
    }

  uint32_t idin = m_id2id[id];
  return m_nodes.at (idin);
}

uint32_t
FBApplication::ComputeContentionWindow (double maxRange, double distance)
{
  NS_LOG_FUNCTION (this << maxRange << distance);
  double cwnd            = 0.0;
  double proximityFactor = 0.0;

  if (maxRange != 0)
    {
      proximityFactor = (maxRange - distance) / maxRange;
    }
  else
    {
      proximityFactor = 0;
    }
  NS_LOG_LOGIC ("proximityFactor pre: " << proximityFactor);
  proximityFactor = (proximityFactor < 0) ? 0 : proximityFactor;
  NS_LOG_LOGIC ("proximityFactor post= " << proximityFactor);

  cwnd = (proximityFactor * (m_cwMax - m_cwMin)) + m_cwMin;
  NS_LOG_DEBUG ("cwnd before rounding: " << cwnd);
  NS_LOG_WARN ("cwnd after rounding: " << std::round (cwnd) << endl
                                       << "with maxRange: " << maxRange
                                       << " and  DISTANCE: " << distance);


  return static_cast<uint32_t> (std::floor (cwnd));
}

int32_t
FBApplication::ComputeErrorDelay ()
{
  if (m_errorRate == 0)
    {
      return 0;
    }
  int32_t  delay   = 0;
  uint32_t percent = m_randomVariable->GetInteger (0, 100);
  if (percent <= m_errorRate)
    {
      uint32_t plusOrMinusOne = m_randomVariable->GetInteger (0, 1);
      if (plusOrMinusOne == 0)
        {
          delay = 1000;
        }
      else
        {
          delay = -1000;
        }
    }
  return delay;
}

template <typename T>
string
FBApplication::StringifyVector (const vector<T>& v)
{
  stringstream ss;
  // cout << "FbApplication::PrintStuff" << m_receivedCoords.size () << " " << m_received
  //      << endl;
  for (auto i = v.begin (); i != v.end (); ++i)
    {
      ss << *i << "_";
    }
  // cout << "FBApplication::StringifyVector" + ss.str() << endl;
  return ss.str ();
}

string
FBApplication::StringifyTransmissionMap () const
{
  stringstream ss;
  for (auto it = m_transmissionList.begin (); it != m_transmissionList.end (); ++it)
    {
      ss << it->first << ":{";
      for (auto el : it->second)
        {
          ss << el << ";";
        }
      ss << "}";
    }
  //	cout << "FBApplication::StringifyTransmissionMap" + ss.str() << endl;
  return ss.str ();
}

uint32_t
FBApplication::IsMaxDistNodeReached (double& maxDist) const
{
  cout << "FBApplication::IsMaxDistNodeReached" << endl;
  Ptr<FBNode> startingNode    = m_nodes.at (m_startingNode);
  Vector      startingNodePos = startingNode->GetPosition ();
  uint32_t    nodeId          = 0;
  for (auto node : m_nodes)
    {
      double dist = ns3::CalculateDistance (node->GetPosition (), startingNodePos);
      if (dist > maxDist)
        {
          maxDist = dist;
          nodeId  = node->GetId ();
        }
    }
  return m_nodes.at (nodeId)->GetReceived ();
}

} // namespace ns3
