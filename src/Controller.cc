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
#include "AhpTopsis.h"
#include "inet/networklayer/contract/IL3AddressType.h"
#include "inet/applications/udpapp/VideoPacket_m.h"
#include "UdpVideoSend.h"
#include <algorithm>
#include "GSO.h"

Define_Module(Controller);
//#define MINDISTANCECH 170

simsignal_t Controller::areaRec = registerSignal("TotalAreaRecorded");
simsignal_t Controller::numNodes = registerSignal("NumDronesSelected");

simsignal_t Controller::rcvdPkSignal = registerSignal("rcvdPk");


ComputeOverlappingArea::StopType parseStopMode(const char *stopModeString)
{
    if (!strcmp(stopModeString, "absolute"))
        return ComputeOverlappingArea::ABSOLUTE;
    else if (!strcmp(stopModeString, "percentage"))
        return ComputeOverlappingArea::PERCENTAGE;
    else if (!strcmp(stopModeString, "distance"))
        return ComputeOverlappingArea::DISTANCE;
    else
        throw cRuntimeError("Unknown mode: '%s'", stopModeString);
}

Controller::Controller() {


}

Controller::~Controller() {
    // TODO Auto-generated destructor stub
    // TODO Auto-generated constructor stub
    for (auto elem : listNodesAux) {
        cancelAndDelete(elem.second->queueTimer);
        while (!elem.second->queue.empty()) {
            delete elem.second->queue.back();
            elem.second->queue.pop_back();
        }
        delete elem.second;
    }

    listNodesAux.clear();
    listNodesArea.clear();
    while (!gsoVectors.empty()) {
        delete gsoVectors.back();
        gsoVectors.pop_back();
    }

#ifdef debugCode
    if (wirelessGetNeig)
        delete wirelessGetNeig;
#endif
}

void Controller::registerModule(cModule *mod, IMobility *mobility, const Coord &pos, const Coord &speed, const double &energy, const Coord &constraintAreaMax, const Coord &constraintAreaMin, int zone ) {
   Enter_Method_Silent();
   auto node = findContainingNode(mod);
   auto addr = L3AddressResolver().addressOf(node);
   NodeData *nodeData = new NodeData;
   nodeData->module = mod;
   nodeData->mobility = mobility;
   int id = mod->getId();
   listNodesId[id] = nodeData;

   std::string name = "Timer-" + addr.str();

   nodeData->queueTimer = new cMessage(name.c_str());
   if (listNodesArea.size() < zone + 1) {
       listNodesArea.resize(zone+1);
       listNodesArea[zone].areaMax = constraintAreaMax;
       listNodesArea[zone].areaMin = constraintAreaMin;
   }
   else {
       //check that the area is the same.
       if (listNodesArea[zone].areaMin != constraintAreaMin || listNodesArea[zone].areaMax != constraintAreaMax)
           throw cRuntimeError("Limit area is not the same in the nodes of this zone/controller");
   }

   nodeData->queueTimer->setKind(20);
   nodeData->position = pos;
   nodeData->speed = speed;
   nodeData->energy = energy;
   nodeData->constraintAreaMax = constraintAreaMax;
   nodeData->constraintAreaMin = constraintAreaMin;
   nodeData->aperture = rad((45.0/180.0)*M_PI );

   listNodesArea[zone].listNodes[addr] = nodeData;

   auto it = listNodesArea[zone].listNodes.find(addr);
   listNodesAux[addr] = nodeData;
   it->second->queueTimer->setContextPointer(nodeData);
}


void Controller::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL)
    {

        delay = par("delay");
        bitRate = par("bitRate");
        MINDISTANCECH = par("MINDISTANCECH");

        computeTimer = new cMessage("computeTimer");
        scheduleAt(simTime() + par("FirstComputation"), computeTimer);
        const char *typeString = par("stopType");
        type = parseStopMode(typeString);
        stopVal = par("stopValue");

        totalAreaVec.setName("Total aggregate area");

        area1Stat.setName("aggregate area 1 Absolute overlapping");
        area1Vec.setName("aggregate area 1 Absolute overlapping");
        numCh1.setName("Num Cluster head 1 Absolute overlapping");
        nunCh1.setName("Cluster head");
        areaPlusNeig1.setName("Area+Neighbor 1 Absolute overlapping");
        totalDivSelected1.setName("Total area/selected 1 Absolute overlapping");
        totalDivSelectedPlusNeig1.setName("Total area/(selected+neighbors) 1 Absolute overlapping");

#ifdef debugCode
        area2Stat.setName("aggregate area 2 Relative overlapping");
        area2Vec.setName("aggregate area 2 Relative overlapping");
        numCh2.setName("Cluster head 2 Relative overlapping");
        areaPlusNeig2.setName("Area+Neighbor 2 Relative overlapping");
        totalDivSelected2.setName("Total area/selected 2 relative overlapping");
        totalDivSelectedPlusNeig2.setName("Total area/(selected+neighbors) 2 relative overlapping");

        area3Stat.setName("aggregate area 3 Minimum distance between CH 120");
        area3Vec.setName("aggregate area 3 Minimum distance between CH 120");
        numCh3.setName("Cluster head 3 Minimum distance between CH 120");
        areaPlusNeig3.setName("Area+Neighbor 3 Minimum distance between CH 120");
        totalDivSelected3.setName("Total area/selected 3 distance between CH 120");
        totalDivSelectedPlusNeig3.setName("Total area/(selected+neighbors) 3 distance between CH 120");

        stdArea1.setName("Area 1 Absolute overlapping");
        stdAreaPlusN1.setName("Area+Neighbor 1 ");
        nunCh1.setName("Cluster head 1");

        stdArea2.setName("Area 2 Absolute overlapping");
        stdAreaPlusN2.setName("Area+Neighbor 2 Relative overlapping");
        nunCh2.setName("Cluster head 2 Relative overlapping");

        stdArea3.setName("Area 3 Minimum distance between to CH 120");
        stdAreaPlusN3.setName("Area+Neighbor 3 Minimum distance between to CH 120");
        nunCh3.setName("Cluster head 3 Minimum distance between to CH 120");
#endif
        totalAreaTotalOverlaping.setName("total overlapping / Total Area");
    }
#ifdef debugCode
    else if (stage == INITSTAGE_NETWORK_LAYER)
        wirelessGetNeig = new WirelessGetNeig();
#endif
}


bool Controller::sendData(const L3Address &addr, cPacket * pkt) {
    simtime_t duration = getDuration(pkt);
    auto it = listNodesAux.find(addr);
    if (it == listNodesAux.end()) {
        delete pkt;
        return false;
    }
    // check if enqueue
    if (!it->second->queueTimer->isScheduled()) {
        this->sendDirect(pkt, delay, duration, it->second->module, "directGate");
        scheduleAt(simTime()+duration, it->second->queueTimer);
    }
    else
        it->second->queue.push_back(pkt);
    return true;
}

void Controller::requestStatus(const L3Address &node) {
    if (!node.isUnspecified()) {
        auto it = listNodesAux.find(node);
        if (it == listNodesAux.end())
            throw cRuntimeError("Address doesn't exist");
        auto control = makeShared<ControlPacket>();
        control->setTypePacket(REQUESTSTATUS);
        control->setDestination(node);
        control->setChunkLength(B(20));
        auto addressType = node.getAddressType();
        control->setOrigin(addressType->getBroadcastAddress());
        auto pkt = new Packet("Request Status");
        pkt->insertAtFront(control);
        sendData(node, pkt);
    }
}

void Controller::changeSenderNode(const L3Address &oldNode, const L3Address & newNode) {
    if (!oldNode.isUnspecified()) {
        auto it = listNodesAux.find(oldNode);
        if (it == listNodesAux.end())
            throw cRuntimeError("Address doesn't exist");
        if (it->second->isClusterHead) {
            // send stop message;
            it->second->isClusterHead = false;
            it->second->totalTimeCh += (simTime() - it->second->startCh);
            auto control = makeShared<ControlPacket>();
            control->setTypePacket(STOPCLUSTERHEAD);
            control->setDestination(oldNode);
            control->setChunkLength(B(20));
            auto addressType = oldNode.getAddressType();
            control->setOrigin(addressType->getBroadcastAddress());
            auto pkt = new Packet("Stop video");
            pkt->insertAtFront(control);
            sendData(oldNode, pkt);
        }
    }
    if (!newNode.isUnspecified()) {
        auto it = listNodesAux.find(newNode);
        if (it == listNodesAux.end())
            throw cRuntimeError("Address doesn't exist");
        if (!it->second->isClusterHead) {
            // send stop message;
            it->second->isClusterHead = true;
            it->second->startCh = simTime();

            auto control = makeShared<ControlPacket>();
            control->setTypePacket(STARTCLUSTERHEAD);
            control->setDestination(newNode);
            control->setChunkLength(B(20));
            auto addressType = newNode.getAddressType();
            control->setOrigin(addressType->getBroadcastAddress());
            auto pkt = new Packet("Start video");
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
            for (auto elem : listNodesAux) {
                if (simTime() - elem.second->lastInfo > par("computeInterval")) {
                    requestStatus(elem.first);
                }
            }

            if (par("UseGso").boolValue() && gsoVectors.empty()) {

                for (auto &elem : listNodesArea) {
                    GSO * gso = new GSO(&(elem.listNodes), MINDISTANCECH);
                    gsoVectors.push_back(gso);
                }
            }

            // compute the new sources here
            auto aperture = rad((45.0/180.0)*M_PI );
            std::vector<CoverageRegion> rect;
            std::vector<CoverageRegion> rect2;
            std::vector<CoverageRegion> rect3, rect4;
            double area;

            std::vector<L3Address> activeNodesTotal;
            std::set<L3Address> nodes;
#ifdef debugCode
            std::vector<L3Address> activeNodesTotal1;
            std::vector<L3Address> activeNodesTotal2;
            std::vector<L3Address> activeNodesTotal3;
            std::set<L3Address> nodes3, nodes4;
#endif

            ComputeOverlappingArea computeOverlappingArea(stopVal, type);
            std::list<L3Address> activeNodes;
            //for (auto elem : listNodesArea) {
            for (int i = 0 ; i < listNodesArea.size(); i++) {
                auto &elem = listNodesArea[i];

                std::vector<CoverageRegion> tempRect;
                std::vector<std::pair<int, double>> sortedlist;
                std::vector<std::map<L3Address, NodeData*>::iterator > listIterators;
                std::vector<CoverageRegion> rect2Temp, rect3Temp, rect4Temp;

                listIterators.clear();
                for (auto it = elem.listNodes.begin(); it != elem.listNodes.end(); ++it) {
#ifdef debugCode
                    it->second->speed = it->second->mobility->getCurrentVelocity();
                    it->second->position = it->second->mobility->getCurrentPosition();
#endif
                    int num = 0;
                    for (auto it2 = elem.listNodes.begin(); it2 != elem.listNodes.end(); ++it2) {
                        if (it->second == it2->second)
                            continue;
                        if (it->second->position.distance(it2->second->position) <= MINDISTANCECH)
                            num++;
                    }
                    it->second->numNeigh = num;
                    listIterators.push_back(it);
                    if (it->second->isClusterHead)
                        activeNodes.push_back(it->first);
                }

                if (par("UseGso").boolValue()) {
                    gsoVectors[i]->run();
                    activeNodesTotal= gsoVectors[i]->getLeaders();

                    for (auto elem2 : elem.listNodes) {
                        auto val = getCoverageRegion(elem2.second->position, aperture);
                        tempRect.push_back(val);
                    }
                }
                else {
                    Topsis(elem.listNodes,  sortedlist);
                    for (auto elem2 : sortedlist) {
                        // incredible inefficient. Doesn't have sense but I don't understand how to use Topsis
                        auto it = listIterators[elem2.first];
                        auto val = getCoverageRegion(it->second->position, aperture);
                        tempRect.push_back(val);
                        rect.push_back(val);
                    }
                }
                if (!par("UseGso").boolValue()) {
                    computeOverlappingArea.setCoverageArea(tempRect);
                    std::vector<int> positions;
#ifndef debugCode
                    computeOverlappingArea.iterateMinDistance(MINDISTANCECH,  tempRect, area, positions);
                    std::set<L3Address> chplusNeigh;
                    for (unsigned int i = 0; i < positions.size(); i++) {
                        auto it = listIterators[sortedlist[positions[i]].first];
                        nodes.insert(it->first);
                        activeNodesTotal.push_back(it->first);
                        auto val = getCoverageRegion(it->second->position, aperture);
                        auto it2 = std::find(tempRect.begin(), tempRect.end(), val);
                        if (it2 == rect4.end())
                            throw cRuntimeError("");
                        else
                            rect4.push_back(*it2);
                        chplusNeigh.insert(it->first);
                        /*                    wirelessGetNeig->getNeighbours(it->first, listNodes , MINDISTANCECH);
                         for (auto elem : listNodes)
                         chplusNeigh.insert(elem);*/
                    }
                    /*
                     std::vector<CoverageRegion> coveragePlusN;
                     for (auto elem2 : chplusNeigh) {
                     auto it = listNodesAux.find(elem2);
                     auto val = getCoverageRegion(it->second->position, aperture);
                     coveragePlusN.push_back(val);
                     }
                     */
                    stdArea1.collect(computeOverlappingArea.getTotalArea(rect4));
//                stdAreaPlusN1.collect(computeOverlappingArea.getTotalArea(coveragePlusN));

#else
                double area3,area4;
                computeOverlappingArea.runUntil(rect2Temp, area);

                for (unsigned int i = 0 ; i < rect2Temp.size(); i++) {
                    rect2.push_back(rect2Temp[i]);
                    auto it = listIterators[sortedlist[i].first];
                    std::vector<L3Address> listNodes;
                    activeNodesTotal1.push_back(it->first);
                    nodes.insert(it->first);
                    wirelessGetNeig->getNeighbours(it->first, listNodes , MINDISTANCECH);
                    for (auto elem : listNodes)
                        nodes.insert(elem);
                }

                computeOverlappingArea.iterateAll(rect3Temp, area3, positions);
                for (unsigned int i = 0 ; i < positions.size(); i++) {

                    rect3.push_back(rect3Temp[i]);
                    auto it = listIterators[sortedlist[positions[i]].first];
                    std::vector<L3Address> listNodes;
                    nodes3.insert(it->first);
                    activeNodesTotal2.push_back(it->first);
                    wirelessGetNeig->getNeighbours(it->first, listNodes , MINDISTANCECH);
                    for (auto elem : listNodes)
                        nodes3.insert(elem);
                }
                std::vector<int> positions2;
                computeOverlappingArea.iterateMinDistance(MINDISTANCECH, rect4Temp, area4, positions2);

                for (unsigned int i = 0 ; i < positions2.size(); i++) {
                    auto it = listIterators[sortedlist[positions2[i]].first];
                    std::vector<L3Address> listNodes;
                    nodes4.insert(it->first);
                    activeNodesTotal3.push_back(it->first);
                    auto val = getCoverageRegion(it->second->position, aperture);
                    auto it2 = std::find(rect4Temp.begin(), rect4Temp.end(), val);
                    if (it2 == rect4.end())
                        throw cRuntimeError("");
                    else
                        rect4.push_back(*it2);
                    wirelessGetNeig->getNeighbours(it->first, listNodes , MINDISTANCECH);
                    for (auto elem : listNodes)
                        nodes4.insert(elem);
                }
#endif
                }
                else {
                    std::set<L3Address> chplusNeigh;
                    for (unsigned int i = 0; i < activeNodesTotal.size(); i++) {
                        auto it = elem.listNodes.find(activeNodesTotal[i]);
                        if (it == elem.listNodes.end())
                            throw cRuntimeError("Node not found");
                        nodes.insert(it->first);
                        auto val = getCoverageRegion(it->second->position, aperture);
                        auto it2 = std::find(tempRect.begin(), tempRect.end(), val);
                        if (it2 == rect4.end())
                            throw cRuntimeError("");
                        else
                            rect4.push_back(*it2);
                        chplusNeigh.insert(it->first);
                        /*                    wirelessGetNeig->getNeighbours(it->first, listNodes , MINDISTANCECH);
                         for (auto elem : listNodes)
                         chplusNeigh.insert(elem);*/
                    }
                    stdArea1.collect(computeOverlappingArea.getTotalArea(rect4));
                }
            }

            rect.clear();
            for (auto elem : listNodesAux) {
                auto val = getCoverageRegion(elem.second->position, aperture);
                rect.push_back(val);
            }

            computeOverlappingArea.setCoverageArea(rect);
            double totalArea = computeOverlappingArea.getTotalArea();
            nunCh1.collect(activeNodesTotal.size());


            //emit(areaRec, area);
            //emit(numNodes, rect2.size());
            totalAreaVec.record(totalArea);
            stdAreaPlusN1.collect(totalArea);

#ifdef debugCode
            double area3, area4;
            rect2.clear();
            rect3.clear();
            rect4.clear();

            for (auto elem : activeNodesTotal1) {
                auto it = listNodesAux.find(elem);
                auto val = getCoverageRegion(it->second->position, aperture);
                rect2.push_back(val);
            }

            for (auto elem : activeNodesTotal2) {
                auto it = listNodesAux.find(elem);
                auto val = getCoverageRegion(it->second->position, aperture);
                rect3.push_back(val);
            }

            for (auto elem : activeNodesTotal3) {
                auto it = listNodesAux.find(elem);
                auto val = getCoverageRegion(it->second->position, aperture);
                rect4.push_back(val);
            }


            area = computeOverlappingArea.getTotalArea(rect2);
            area3 = computeOverlappingArea.getTotalArea(rect3);
            area4 = computeOverlappingArea.getTotalArea(rect4);

            area1Stat.collect(area);
            area1Vec.record(area);
            numCh1.record(activeNodesTotal1.size());
            nunCh1.collect(activeNodesTotal1.size());

            area2Stat.collect(area3);
            area2Vec.record(area3);
            numCh2.record(activeNodesTotal2.size());
            nunCh2.collect(activeNodesTotal2.size());

            area3Stat.collect(area4);
            area3Vec.record(area4);
            numCh3.record(activeNodesTotal3.size());
            nunCh3.collect(activeNodesTotal3.size());

            totalDivSelected1.record(totalArea/area);
            totalDivSelected2.record(totalArea/area3);
            totalDivSelected3.record(totalArea/area4);

            std::vector<CoverageRegion> coverage1, coverage2, coverage3;
            double areaCov1,areaCov2,areaCov3;

            for (auto elem2 : nodes) {
                auto it = listNodesAux.find(elem2);
                auto val = getCoverageRegion(it->second->position, aperture);
                coverage1.push_back(val);
            }

            for (auto elem2 : nodes3) {
                auto it = listNodesAux.find(elem2);
                auto val = getCoverageRegion(it->second->position, aperture);
                coverage2.push_back(val);
            }

            for (auto elem2 : nodes4) {
                auto it = listNodesAux.find(elem2);
                auto val = getCoverageRegion(it->second->position, aperture);
                coverage3.push_back(val);
            }

            areaCov1 = computeOverlappingArea.getOverlappingArea(coverage1);
            areaCov2 = computeOverlappingArea.getOverlappingArea(coverage2);
            areaCov3 = computeOverlappingArea.getOverlappingArea(coverage3);

            totalDivSelectedPlusNeig1.record(totalArea/areaCov1);
            totalDivSelectedPlusNeig2.record(totalArea/areaCov2);
            totalDivSelectedPlusNeig3.record(totalArea/areaCov3);

            areaPlusNeig1.record(areaCov1);
            areaPlusNeig2.record(areaCov2);
            areaPlusNeig3.record(areaCov3);

            stdArea1.collect(area);
            stdAreaPlusN1.collect(areaCov1);

            stdArea2.collect(area3);
            stdAreaPlusN2.collect(areaCov2);

            stdArea3.collect(area4);
            nunCh3.collect(rect4.size());
            stdAreaPlusN3.collect(areaCov3);

            totalAreaTotalOverlaping.collect(totalArea/(2000.0*2000.0));
            computeOverlappingArea.runUntil(rect2, area);
#else

            emit(areaRec, area);
            emit(numNodes, rect.size());

            // now notify the nodes
            for (const auto &elem : activeNodesTotal) {
                auto itAux = std::find(activeNodes.begin(), activeNodes.end(), elem);
                if (itAux == activeNodes.end()) {
                    // new active node
                    activatedSenderNode(elem);
                }
                else {
                    // erase from the list
                    activeNodes.erase(itAux);
                }
            }
            // deactivate the nodes if it is necessary
            for (auto elem : activeNodes) {
                deactivatedSenderNode(elem);
            }
#endif
            // Schedule now the new timer
            scheduleAt(simTime() + par("computeInterval"), computeTimer);
        }
        // here others timers
        return;
    }

    // Here packets of nodes

    auto itSender = listNodesId.find(msg->getSenderModuleId());

    if (itSender == listNodesId.end())
        throw cRuntimeError("Module not found");

    auto packet = check_and_cast<Packet *>(msg);

    itSender->second->totalBytesRec += packet->getByteLength();

    auto controlMsg = packet->peekAtFront<ControlPacket>();
    if (controlMsg->getTypePacket() == VIDEODATA) {
        numRecPackets++;
        emit(rcvdPkSignal, packet);

        auto sender = controlMsg->getOrigin();



        if (sender.isBroadcast() || sender.isUnspecified())
            throw cRuntimeError("Invalid sender address");

        auto it = nodesData.find(sender);
        if (it == nodesData.end()) {
            NodesData data;
            nodesData[sender] = data;
            it = nodesData.find(sender);
        }

        packet->popAtFront<ControlPacket>();
        auto videoData = packet->peekAtFront<VideoPacket>();
        if (videoData) {
            do {
                auto vpktAux =  packet->popAtFront<VideoPacket>();
                if (vpktAux->getSeqNum() > it->second.lastSeqNum)
                    it->second.lastSeqNum = vpktAux->getSeqNum();
                else  {
                    delete packet;
                    return;
                }
                switch(vpktAux->getType()) {
                    case 'P':
                        it->second.numPframes++;
                        it->second.totalBytesP += vpktAux->getFrameSize().get();
                        break;
                    case 'B':
                        it->second.numBframes++;
                        it->second.totalBytesB += vpktAux->getFrameSize().get();
                        break;
                    case 'I':
                        it->second.numIframes++;
                        it->second.totalBytesI += vpktAux->getFrameSize().get();
                        break;
                }
            } while(packet->getBitLength() > 0);
        }
        // Now we have the video data,
    }
    else if (controlMsg->getTypePacket() == CONTROLDATA) {

        auto controlData = packet->peekAtFront<ActualizeData>();
        auto it = listNodesAux.find(controlData->getOrigin());
        if (it == listNodesAux.end())
            throw cRuntimeError("Node is not in the listNode map");

        it->second->energy = controlData->getEnergy();
        it->second->position = controlData->getPosition();
        it->second->speed = controlData->getSpeed();
        it->second->lastInfo = simTime();

        // remember that you need the address of the owner of this control information
        // Now we have the video data,
        // Extract here the data you need for the algorithms that computes the video senders.
    }
    delete msg;
}


void Controller::finish()
{
    stdArea1.record();
    stdAreaPlusN1.record();
    //totalAreaVec.record();
    nunCh1.record();
#ifdef debugCode
    stdArea2.record();
    stdAreaPlusN2.record();
    nunCh2.record();

    stdArea3.record();
    stdAreaPlusN3.record();
    nunCh3.record();
#endif
    totalAreaTotalOverlaping.record();
    uint64_t totalP = 0;
    uint64_t totalI = 0;
    uint64_t totalB = 0;
    uint64_t totalBytesB = 0;
    uint64_t totalBytesP = 0;
    uint64_t totalBytesI = 0;

    uint64_t totalPs = 0;
    uint64_t totalIs = 0;
    uint64_t totalBs = 0;
    uint64_t totalBytesBs = 0;
    uint64_t totalBytesPs = 0;
    uint64_t totalBytesIs = 0;

    uint64_t totalBytesBCh = 0;
    uint64_t totalBytesPCh = 0;
    uint64_t totalBytesICh = 0;

    uint64_t totalBytesBCl = 0;
    uint64_t totalBytesPCl = 0;
    uint64_t totalBytesICl = 0;

    double totalBw = 0;
    int cont = 0;
    double consumedEnergy = 0;

    for (const auto &elem : listNodesAux) {
        auto source = check_and_cast<UdpVideoSend *> (elem.second->module);
        totalPs += source->getTotalSendP();
        totalIs += source->getTotalSendI();
        totalBs += source->getTotalSendB();
        totalBytesBs += source->getTotalBytesSendB();
        totalBytesIs += source->getTotalBytesSendI();
        totalBytesPs += source->getTotalBytesSendP();

        totalBytesPCh += source->getTotalBytesSendPCh();
        totalBytesICh += source->getTotalBytesSendICh();
        totalBytesBCh += source->getTotalBytesSendBCh();

        totalBytesPCl += source->getTotalBytesSendPCl();
        totalBytesICl += source->getTotalBytesSendICl();
        totalBytesBCl += source->getTotalBytesSendBCl();
        consumedEnergy += source->getConsumedEnergy();
    }

    for (const auto &elem : nodesData) {
        std::string infoNode(elem.first.str());
        std::string framesP = infoNode + " Frames P :";
        totalP += elem.second.numPframes;
        if (!par("recordOnlyTotal"))
            recordScalar(framesP.c_str(), elem.second.numPframes);

        std::string framesI = infoNode + " Frames I :";
        totalI += elem.second.numIframes;
        if (!par("recordOnlyTotal"))
            recordScalar(framesI.c_str(), elem.second.numIframes);

        std::string framesB = infoNode + " Frames B :";
        totalB += elem.second.numBframes;
        if (!par("recordOnlyTotal"))
            recordScalar(framesB.c_str(), elem.second.numBframes);

        std::string bytesP = infoNode + " Bytes P :";
        totalBytesP += elem.second.totalBytesP;
        if (!par("recordOnlyTotal"))
            recordScalar(bytesP.c_str(), elem.second.totalBytesP);

        std::string bytesI = infoNode + " Bytes I :";
        totalBytesI += elem.second.totalBytesI;
        if (!par("recordOnlyTotal"))
            recordScalar(bytesI.c_str(), elem.second.totalBytesI);

        std::string bytesB = infoNode + " Bytes B :";
        totalBytesB += elem.second.totalBytesB;
        if (!par("recordOnlyTotal"))
            recordScalar(bytesB.c_str(), elem.second.totalBytesB);
    }


    for (const auto &elem : listNodesAux) {

        if (elem.second->isClusterHead) {
            elem.second->totalTimeCh += (simTime() - elem.second->startCh);
        }
        if (elem.second->totalTimeCh != SimTime::ZERO) {
            double bw = (double)elem.second->totalBytesRec/elem.second->totalTimeCh.dbl();
            std::string infoNode(elem.first.str());
            std::string data = infoNode + " Bandwidth (Byte/s) :";
            if (!par("recordOnlyTotal"))
                recordScalar(data.c_str(), bw);
            totalBw += bw;
            cont++;
        }

    }

    recordScalar("Total P frames rec", totalP);
    recordScalar("Total I frames rec", totalI);
    recordScalar("Total B frames rec", totalB);

    recordScalar("Total P bytes rec", totalBytesP);
    recordScalar("Total I bytes rec", totalBytesI);
    recordScalar("Total B bytes rec", totalBytesB);

    uint64_t totalVideoRec = totalBytesB+totalBytesP+totalBytesI;
    recordScalar("Total Video rec", totalVideoRec);


    recordScalar("Total P frames send", totalPs);
    recordScalar("Total I frames send", totalIs);
    recordScalar("Total B frames send", totalBs);

    recordScalar("Total P bytes send", totalBytesPs);
    recordScalar("Total I bytes send", totalBytesIs);
    recordScalar("Total B bytes send", totalBytesBs);

    uint64_t totalVideoSend = totalBytesBs+totalBytesPs+totalBytesIs;

    recordScalar("Total Video send", totalVideoSend);

    recordScalar("Total P bytes send Cluster heads", totalBytesPCh);
    recordScalar("Total I bytes send Cluster heads", totalBytesICh);
    recordScalar("Total B bytes send Cluster heads", totalBytesBCh);
    uint64_t totalVideoSendCh = totalBytesPCh + totalBytesICh + totalBytesBCh;
    recordScalar("Total Video send Cluster heads", totalVideoSendCh);


    recordScalar("Total P bytes send Clients", totalBytesPCl);
    recordScalar("Total I bytes send Clients", totalBytesICl);
    recordScalar("Total B bytes send Clients", totalBytesBCl);
    uint64_t totalVideoSendCl = totalBytesPCl + totalBytesICl + totalBytesBCl;
    recordScalar("Total Video send Clients", totalVideoSendCl);

    double vid =  ((double)totalVideoRec / (double)totalVideoSend) * 100;

    recordScalar("video percentage", vid);

    recordScalar("Total consumed energy", consumedEnergy);

    recordScalar("Mean Bandwidth B/s", totalBw/cont);
}

CoverageRegion Controller::getCoverageRegion(const Coord &pos, const rad &aperture)
{
    double h = pos.z;
    if (aperture.get() > 170)
        throw cRuntimeError("Aperture too high");
    double l = h*std::tan(aperture.get());
    CoverageRegion reg;

    Coord min = pos;
    Coord max = pos;
    min.z = 0;
    max.z = 0;
    min.x -= l;
    min.y -= l;
    max.x += l;
    max.y += l;
    reg.first = min;
    reg.second = max;
    reg.center = pos;
    return reg;
}



bool compareFunc (CoverageRegion i,CoverageRegion j) { return (i.second.x < j.second.x); }
bool operator <(const CoverageRegion& rect1, const CoverageRegion& rect2) {
    return (rect1.second.x < rect2.second.x);
}

double ComputeOverlappingArea::getOverlappingArea(std::vector<CoverageRegion> & vect) {
    return getArea(vect);
}

void ComputeOverlappingArea::setCoverageArea(std::vector<CoverageRegion> &r) {
    rectangles = r;
}

ComputeOverlappingArea::ComputeOverlappingArea(std::vector<CoverageRegion> rect, double stopValue, StopType t):
rectangles(rect),
stopVal(stopValue),
type(t)
{
}

ComputeOverlappingArea::ComputeOverlappingArea(double stopValue, StopType t):
stopVal(stopValue),
type(t)
{
    rectangles.clear();
}

double ComputeOverlappingArea::getTotalArea() {
    double val = getArea(rectangles);
    return val;
}

double ComputeOverlappingArea::getTotalArea(std::vector<CoverageRegion> &r) {
    double val = getArea(r);
    return val;
}

void ComputeOverlappingArea::runUntil(std::vector<CoverageRegion> &returnRect, double &totalArea) {
    double initialArea = 0;
    returnRect.clear();

    if (type == DISTANCE) {
        std::vector<int> pos;
        iterateMinDistance(stopVal, returnRect, totalArea, pos);
        return;
    }

    for (const auto & elem : rectangles) {
        returnRect.push_back(elem);

        double areaRec = std::abs((elem.first.x - elem.second.x)) * std::abs((elem.first.y - elem.second.y));
        double area = getArea(returnRect);
        double incArea = std::abs(area - initialArea);

        if (type == ABSOLUTE) {
            if (incArea < stopVal) {
                returnRect.pop_back();
                totalArea = initialArea;
                return;
            }
        }
        else if (type == PERCENTAGE){
            if (100*(areaRec - incArea)/areaRec > stopVal) {
                returnRect.pop_back();
                totalArea = initialArea;
                return;
            }
        }
        else {
            throw cRuntimeError("Type error");
        }
        initialArea = area;
    }
    totalArea = initialArea;
}

void ComputeOverlappingArea::iterateAll(std::vector<CoverageRegion> &returnRect, double &totalArea, std::vector<int> &pos){
    double initialArea = 0;
    returnRect.clear();


    for (unsigned int i = 0; i < rectangles.size(); i++) {
        returnRect.push_back(rectangles[i]);
        pos.push_back(i);
        double areaRec = std::abs((rectangles[i].first.x - rectangles[i].second.x)) * std::abs((rectangles[i].first.y - rectangles[i].second.y));
        double area = getArea(returnRect);
        double incArea = std::abs(area - initialArea);

        if (type == ABSOLUTE) {
            if (incArea < stopVal) {
                returnRect.pop_back();
                pos.pop_back();
                continue;
            }
        }
        else {
            if (100*(areaRec - incArea)/areaRec > stopVal) {
                returnRect.pop_back();
                pos.pop_back();
                continue;
            }
        }
        initialArea = area;
    }
    totalArea = initialArea;
}


void ComputeOverlappingArea::iterateMinDistance(const double &minDist, std::vector<CoverageRegion> &returnRect, double &totalArea, std::vector<int> &pos){
    returnRect.clear();
    std::vector<Coord> selected;
    for (unsigned int i = 0; i < rectangles.size(); i++) {
        if (!returnRect.empty()) {
            bool close = false;
            for (auto elem : returnRect) {
                if (elem.center.distance(rectangles[i].center) < minDist) {
                    close = true;
                    break;
                }
            }
            if (close)
                continue;
        }
        returnRect.push_back(rectangles[i]);
        pos.push_back(i);
    }
    totalArea = getArea(returnRect);
}

void ComputeOverlappingArea::getAllX(const std::vector<CoverageRegion>& rects, std::vector<double>& xes) {

    xes.clear();
    for (auto elem : rects) {
        xes.push_back(elem.first.x);
        xes.push_back(elem.second.x);
    }
}

double ComputeOverlappingArea::getArea(std::vector<CoverageRegion> rects) {
    // sort rectangles according to x-value of right edges
    std::sort(rects.begin(), rects.end(), compareFunc);

    std::vector<double> xes;
    getAllX(rects, xes);
    std::sort(xes.begin(), xes.end());

    double area = 0;
    auto iterX1 = xes.begin();
    auto iterRect = rects.begin();
    for (; iterX1 != xes.end() && iterX1 != xes.end() - 1; ++iterX1) {
        auto iterX2 = iterX1 + 1;

        // filter out duplicated X-es
        if (*iterX1 < *iterX2) {
            Range rangeX(*iterX1, *iterX2);

            while (iterRect->second.x < *iterX1)
                ++iterRect;

            std::list<Range> rangesOfY;
            getRangesOfY(rects, iterRect, rangeX, rangesOfY);
            area += getRectArea(rangeX, rangesOfY);
        }
    }

    return area;
}

double ComputeOverlappingArea::getRectArea(const Range& rangeX, const std::list<Range>& rangesOfY)
{
    double width = rangeX.greater - rangeX.less;
    auto iter = rangesOfY.begin();
    double area = 0;
    for (; iter != rangesOfY.end(); ++iter) {
        auto height = iter->greater - iter->less;
        area += width * height;
    }

    return area;
}

void ComputeOverlappingArea::getRangesOfY(const std::vector<CoverageRegion>& rects, std::vector<CoverageRegion>::const_iterator iterRect,
        const Range& rangeX, std::list<Range>& rangesOfY)
{
    for (; iterRect != rects.end(); ++iterRect) {
        if (rangeX.less < iterRect->second.x && rangeX.greater > iterRect->first.x) {
            Range range(iterRect->first.y, iterRect->second.y);
            insertRangeY(rangesOfY, range);
        }
    }
}

void ComputeOverlappingArea::insertRangeY(std::list<Range>& rangesOfY, Range& rangeY)
{
    auto iter = rangesOfY.begin();
    while (iter != rangesOfY.end()) {
        if (rangeY.IsOverlapping(*iter)) {
            rangeY.Merge(*iter);

            auto iterCopy = iter;
            ++iter;
            rangesOfY.erase(iterCopy);
        } else
            ++iter;
    }
    rangesOfY.push_back(rangeY);
}
