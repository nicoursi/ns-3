#ifndef TRANSMISSIONLIST_H
#define TRANSMISSIONLIST_H

#include <vector>
#include <map>
#include "KeyableVector.h"

using namespace std;

namespace ns3
{

class TransmissionList
{

public:
  TransmissionList ()
  {
  }

  void AddEdge (KeyableVector source, KeyableVector destination);

  string ToString () const;


private:
  // Map to identify outgoing transmission from node (ley) to all other nodes
  std::map<KeyableVector, vector<KeyableVector>> transmissions;
};
} // namespace ns3

#endif
