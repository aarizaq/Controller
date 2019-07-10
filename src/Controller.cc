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
#include "inet/common/ModuleAccess.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/common/ModuleIdAddress.h"
#include "Controller.h"
#include "ControlPackets_m.h"
#include "inet/networklayer/contract/IL3AddressType.h"
#include "inet/applications/udpapp/VideoPacket_m.h"

Define_Module(Controller);

Controller::Controller() {
    // TODO Auto-generated constructor stub
    for (auto elem : listNodes)
        cancelAndDelete(elem.second.queueTimer);
    listNodes.clear();
}

Controller::~Controller() {
    // TODO Auto-generated destructor stub
}

void Controller::registerModule(cModule *mod, const Coord &pos, const Coord &speed, const double &energy ) {
   auto node = findContainingNode(mod);
   auto addr = L3AddressResolver().addressOf(node);
   NodeData nodeData;
   nodeData.module = mod;
   std::string name = "Timer-" + addr.str();

   nodeData.queueTimer = new cMessage(name.c_str());
   listNodes[addr] = nodeData;
   auto it = listNodes.find(addr);
   nodeData.queueTimer->setContextPointer(&(it->second));
   nodeData.queueTimer->setKind(20);
   nodeData.position = pos;
   nodeData.speed = speed;
   nodeData.energy = energy;
}

void Controller::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL)
    {

        delay = par("delay");
        bitRate = par("bitRate");

        computeTimer = new cMessage("computeTimer");
        scheduleAt(simTime() + par("FirstComputation"), computeTimer);
    }
}


bool Controller::sendData(const L3Address &addr, cPacket * pkt) {
    simtime_t duration = getDuration(pkt);
    auto it = listNodes.find(addr);
    if (it == listNodes.end()) {
        delete pkt;
        return false;
    }
    // check if enqueue
    if (!it->second.queueTimer->isScheduled()) {
        this->sendDirect(pkt, delay, duration, it->second.module, "directGate");
        scheduleAt(simTime()+duration, it->second.queueTimer);
    }
    else
        it->second.queue.push_back(pkt);
    return true;
}

void Controller::changeSenderNode(const L3Address &oldNode, const L3Address & newNode) {
    if (!oldNode.isUnspecified()) {
        auto it = listNodes.find(oldNode);
        if (it->second.isSenderModule) {
            // send stop message;
            it->second.isSenderModule = false;
            auto control = makeShared<ControlPacket>();
            control->setTypePacket(STOPVIDEO);
            control->setDestination(oldNode);
            control->setChunkLength(B(20));
            auto addressType = oldNode.getAddressType();
            control->setOrigin(addressType->getBroadcastAddress());
            auto pkt = new Packet();
            pkt->insertAtFront(control);
            sendData(oldNode, pkt);
        }
    }
    if (!newNode.isUnspecified()) {
        auto it = listNodes.find(newNode);
        if (!it->second.isSenderModule) {
            // send stop message;
            it->second.isSenderModule = false;
            auto control = makeShared<ControlPacket>();
            control->setTypePacket(STARTVIDEO);
            control->setDestination(newNode);
            control->setChunkLength(B(20));
            auto addressType = newNode.getAddressType();
            control->setOrigin(addressType->getBroadcastAddress());
            auto pkt = new Packet();
            pkt->insertAtFront(control);
            sendData(newNode, pkt);
        }
    }
}

void Controller::handleMessage(cMessage *msg) {

    if (msg->isSelfMessage()) {
        if (msg->getKind() == 20) {
            // queue timer, send next packet in the queue if exist
            NodeData *nodeData = static_cast<NodeData *>(msg->getContextPointer());
            if (!nodeData->queue.empty()) {
                cPacket *pkt = nodeData->queue.front();
                nodeData->queue.pop_front();
                simtime_t duration = getDuration(pkt);
                this->sendDirect(pkt, delay, duration, nodeData->module, "directGate");
                scheduleAt(simTime()+duration, nodeData->queueTimer);
            }
        }
        if (computeTimer == msg) {
            // compute the new sources here

            // Schedule now the new timer
            scheduleAt(simTime() + par("computeInterval"), computeTimer);
        }
        // here others timers
        return;
    }
    // Here packets of nodes
    auto packet = check_and_cast<Packet *>(msg);
    auto controlMsg = packet->peekAtFront<ControlPacket>();
    if (controlMsg->getTypePacket() == VIDEODATA) {
        auto sender = controlMsg->getOrigin();
        packet->popAtFront();
        auto videoData = packet->peekAtFront<VideoPacket>();
        // Now we have the video data,
    }
    else if (controlMsg->getTypePacket() == CONTROLDATA) {

        auto controlData = packet->peekAtFront<ActualizeData>();
        auto it = listNodes.find(controlData->getOrigin());
        if (it == listNodes.end())
            throw cRuntimeError("Node is not in the listNode map");

        it->second.energy = controlData->getEnergy();
        it->second.position = controlData->getPosition();
        it->second.speed = controlData->getSpeed();

        // remember that you need the address of the owner of this control information
        // Now we have the video data,
        // Extract here the data you need for the algorithms that computes the video senders.
    }
    delete msg;
}

