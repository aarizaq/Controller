//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef SRC_CONTROLLER_H_
#define SRC_CONTROLLER_H_

#include <omnetpp.h>
#include <map>
#include <vector>
#include <deque>
#include "inet/networklayer/common/L3Address.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/geometry/common/Coord.h"


using namespace omnetpp;
using namespace inet;

class Controller : public cSimpleModule {

    struct NodeData {
        std::deque<cPacket *> queue;
        cMessage *queueTimer = nullptr;
        cModule * module = nullptr;
        bool isSenderModule = false;
        Coord position;
        Coord speed;
        double energy;
    };
    std::map<L3Address, NodeData> listNodes;
    simtime_t delay;
    double bitRate = 0;

    cMessage * computeTimer = nullptr;


public:
    Controller();
    virtual ~Controller();
    virtual void registerModule(cModule *, const Coord &pos, const Coord &speed, const double &energy);
    virtual simtime_t getDelay() {return delay;}
    virtual simtime_t getDuration(cPacket * pkt) {return pkt->getBitLength()/bitRate;}

protected:
    virtual void changeSenderNode(const L3Address &oldNode, const L3Address & newNode);
    virtual void activatedSenderNode(const L3Address & newNode) {changeSenderNode(L3Address(), newNode);}
    virtual void deactivatedSenderNode(const L3Address & oldNode) {changeSenderNode(oldNode, L3Address());}

    virtual bool sendData(const L3Address &, cPacket *);
    virtual int numInitStages() const  override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage)  override;
    virtual void handleMessage(cMessage *) override;

};

#endif /* SRC_CONTROLLER_H_ */
