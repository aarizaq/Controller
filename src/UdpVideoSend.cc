//
// Copyright (C) 2005 Andras Varga
// Copyright (C) 2015 A. Ariza (Malaga University)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//


//
// based on the video streaming app of the similar name by Johnny Lai
//


#include <iostream>
#include <fstream>
#include "inet/networklayer/common/InterfaceTable.h"

#include "inet/applications/udpapp/VideoPacket_m.h"
#include "UdpVideoSend.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/TimeTag_m.h"
#include "inet/common/packet/chunk/ByteCountChunk.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/transportlayer/common/L4PortTag_m.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "ControlPackets_m.h"
#include "inet/networklayer/contract/IL3AddressType.h"


Define_Module(UdpVideoSend);

inline std::ostream& operator<<(std::ostream& os, UdpVideoSend::ChData obj)
{
    os << "Dist :" << obj.distance << " Next hop << " << obj.nextHop << " Last t " << obj.lasTime;
    return os;
}


simsignal_t UdpVideoSend::reqStreamBytesSignal = registerSignal("reqStreamBytes");
simsignal_t UdpVideoSend::sentPkSignal = registerSignal("sentPk");

void UdpVideoSend::fileParser(const char *fileName)
{
    std::string fi(fileName);
    fi = "./"+fi;
    std::ifstream inFile(fi.c_str());

    if (!inFile)
    {
        error("Error while opening input file (File not found or incorrect type)\n");
    }

    trace.clear();
    simtime_t timedata;
    while (!inFile.eof())
    {
        int seqNum = 0;
        float time = 0;
        float YPSNR = 0;
        float UPSNR = 0;
        float VPSNR = 0;
        int len = 0;
        char frameType = 0;
        std::string line;
        std::getline (inFile,line);
        size_t pos = line.find("#");
        if (pos != std::string::npos)
        {
            if (pos == 0)
                line.clear();
            else
                line = line.substr(0,pos-1);
        }
        if (line.empty())
            continue;
        std::istringstream(line) >> seqNum >>  time >> frameType >> len >> YPSNR >> UPSNR >> VPSNR;
        // inFile >> seqNum >>  time >> frameType >> len >> YPSNR >> UPSNR >> VPSNR;
        VideoInfo info;
        info.seqNum = seqNum;
        info.type = frameType;
        info.size = len;
        info.timeFrame = time;
        // now insert in time order
        if (trace.empty() || trace.back().timeFrame < info.timeFrame)
            trace.push_back(info);
        else
        {
            // search the place
            for (int i = trace.size()-1 ; i >= 0; i--) {
                if (trace[i].timeFrame < info.timeFrame) {
                    trace.insert(trace.begin()+i+1,info);
                    break;
                }
            }
        }
    }
    inFile.close();
}


UdpVideoSend::UdpVideoSend()
{

}

UdpVideoSend::~UdpVideoSend()
{
    cancelAndDelete(selfMsg);
    if (energyTimer)
        cancelAndDelete(energyTimer);
    clearStreams();
    trace.clear();
    cancelAndDelete(senderControlTimer);
    for (auto &elem: queueToControl)
        delete elem;
    for (auto &elem : streamVector) {
        cancelAndDelete(elem.timer);
    }
    cancelAndDelete(actualizeStatus);
    while (!pendingSend.empty()) {
        auto it =  pendingSend.begin();
        auto timer = *it;
        pendingSend.erase(it);
        delete timer;
    }
    if (advertCluster)
        cancelAndDelete(advertCluster);
}

void UdpVideoSend::initialize(int stage)
{
    ApplicationBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL)
    {
        sendInterval = &par("sendInterval");
        packetLen = &par("packetLen");
        stopTime = &par("stopTime");
        localPort = par("localPort");

        macroPackets = par("macroPackets");
        maxSizeMacro = par("maxSizeMacro").intValue();

        // statistics
        numStreams = 0;
        numPkSent = 0;

        trace.clear();
        std::string fileName(par("traceFileName").stringValue());
        if (!fileName.empty())
            fileParser(fileName.c_str());
        // read energy before
        energyTimer = new cMessage("Energy timer");

        senderControlTimer = new cMessage("senderControlTimer");

        remainingEnergy = par("energy");
        initialEnergy = remainingEnergy;
        consumptionPerSec =  par("consumptionPerTimeUnits");
        percetajeIncreaseEnergy = par("percetajeIncreaseEnergyClusterHeads").doubleValue()/100.0;

        scheduleAt(simTime() + par("energyEvalInterval"), energyTimer);

        actualizeStatus = new cMessage("ActualizeStatus");
        scheduleAt(simTime() + par("startTimer") + par("actualizeTimer"), actualizeStatus);

        jitterPar = &par("jitter");
        periodicJitter = &par("periodicJitter");
        advertCluster = new cMessage("Avert Cluster Timer");
        advertInterval = par("advertInterval");
        selfMsg = new cMessage("startEvent");
        WATCH(iamClusterHead);
        WATCH_MAP(clusterHeads);
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {

        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort);
        socket.setBroadcast(true);
        socket.setCallback(this);

        controllerId = par("controllerNumber");
        zoneId = par("zone");

        interfaceTable = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);

        auto mod = cSimulation::getActiveSimulation()->getSystemModule()->getSubmodule("controller", controllerId);
        controller = check_and_cast<Controller *>(mod);
        auto node = getContainingNode(this);
        myAddress = L3AddressResolver().addressOf(node);
        mobility = check_and_cast<IMobility *>(node->getSubmodule("mobility"));
        controller->registerModule(this, mobility, mobility->getCurrentPosition(), mobility->getCurrentVelocity(), remainingEnergy, mobility->getConstraintAreaMax(), mobility->getConstraintAreaMin(), zoneId);
    }
}


void UdpVideoSend::deleteClusterHead(const L3Address & addr) {
    auto it = clusterHeads.find(addr);
    if (it != clusterHeads.end())
        clusterHeads.erase(it);
}

void UdpVideoSend::addClusterHead(const L3Address & addr, const int & distance, const L3Address &next, const Coord &position, const Coord &speed) {
    auto it = clusterHeads.find(addr);
    if (it == clusterHeads.end()) {
        ChData data;
        data.distance = distance;
        data.lasTime = simTime();
        data.nextHop = next;
        data.position = position;
        data.speed = speed;
        clusterHeads[addr] = data;
    }
    else {
        it->second.distance = distance;
        it->second.lasTime = simTime();
        it->second.nextHop = next;
        it->second.position = position;
        it->second.speed = speed;
    }
}


void UdpVideoSend::notifyEndClusterHead() {
    auto header = makeShared<AdvertClusterHead>();
    header->setOrigin(myAddress);
    header->setDestination(myAddress.getAddressType()->getBroadcastAddress());
    header->setSeqNumber(seqNumber++);
    header->setNumHops(par("maxNumHops"));
    header->setDistance(0);
    header->setTypePacket(CHENDADVERT);
    header->setChunkLength(B(60)); // check the size it is important
    header->setControllerId(controllerId); // at least it will use controller 0
    header->setPosition(mobility->getCurrentPosition());
    header->setSpeed(mobility->getCurrentVelocity());
    header->setZone(zoneId);
    auto pkt = new Packet("CHENDADVERT");
    pkt->insertAtFront(header);
    auto *timer = new PacketHolderMessage("LoadNg-send-jitter", KIND_DELAYEDSEND);
    timer->setOwnedPacket(pkt);
    pendingSend.insert(timer);
    scheduleAt(simTime()+*jitterPar, timer);
    //socket.sendTo(pkt, myAddress.getAddressType()->getBroadcastAddress(), localPort);
}

void UdpVideoSend::notifyNewClusterHead(const simtime_t &t) {
    if (!iamClusterHead)
        return;

    auto header = makeShared<AdvertClusterHead>();
    header->setOrigin(myAddress);
    header->setDestination(myAddress.getAddressType()->getBroadcastAddress());
    header->setSeqNumber(seqNumber++);
    header->setNumHops(par("maxNumHops"));
    header->setPosition(mobility->getCurrentPosition());
    header->setSpeed(mobility->getCurrentVelocity());
    header->setDistance(0);
    header->setTypePacket(CHADVERT);
    header->setChunkLength(B(30)); // check the size it is important
    auto pkt = new Packet("CHADVERT");
    pkt->insertAtFront(header);
    auto *timer = new PacketHolderMessage("LoadNg-send-jitter", KIND_DELAYEDSEND);
    timer->setOwnedPacket(pkt);
    pendingSend.insert(timer);

    if (t != SimTime::ZERO)
        scheduleAt(simTime() + *jitterPar, timer);
    else
        scheduleAt(simTime() + t, timer);

    scheduleAt(simTime() + advertInterval - *periodicJitter, advertCluster);
    // socket.sendTo(pkt, myAddress.getAddressType()->getBroadcastAddress(), localPort);
}

void UdpVideoSend::refreshClusterHead() {
    if (!iamClusterHead)
        return;

    auto header = makeShared<AdvertClusterHead>();
    header->setOrigin(myAddress);
    header->setDestination(myAddress.getAddressType()->getBroadcastAddress());
    header->setSeqNumber(seqNumber++);
    header->setNumHops(par("maxNumHops"));
    header->setPosition(mobility->getCurrentPosition());
    header->setSpeed(mobility->getCurrentVelocity());
    header->setDistance(0);
    header->setTypePacket(CHADVERT);
    header->setChunkLength(B(30)); // check the size it is important
    auto pkt = new Packet("CHADVERT");
    pkt->insertAtFront(header);
    int interfaceId = CHK(interfaceTable->findInterfaceByName(par("broadcastInterface")))->getInterfaceId(); // TODO: Implement: support for multiple interfaces
    pkt->addTag<InterfaceReq>()->setInterfaceId(interfaceId);
    socket.sendTo(pkt, myAddress.getAddressType()->getBroadcastAddress(), localPort);
    scheduleAt(simTime() + advertInterval - *periodicJitter, advertCluster);
}



bool UdpVideoSend::getBestClusterHead(L3Address &add) {
    add = L3Address();
    if (clusterHeads.empty()) {
        return false; // return false if there isn't a cluster head
    }
    int min = 100000;
    for (auto it = clusterHeads.begin(); it != clusterHeads.end();) {
        if ((simTime() - it->second.lasTime) > (par("allowedAdvertLoss").intValue() * advertInterval)) {
            clusterHeads.erase(it++);
            continue;
        }
        if (it->second.distance < min) {
            min = it->second.distance;
            add = it->first;
        }
        ++it;
    }
    if (add.isUnspecified() || add.isBroadcast())
        return false;
    return true;
}

L3Address UdpVideoSend::getNextHopBestClusterHead()
{
    if (clusterHeads.empty()) {
        return L3Address(); // return false if there isn't a cluster head
    }
    int min = 100000;
    double disMin = 10000000;

    L3Address add;
    for (auto it = clusterHeads.begin(); it != clusterHeads.end();) {
        if ((simTime() - it->second.lasTime) > (par("allowedAdvertLoss").intValue() * advertInterval)) {
            clusterHeads.erase(it++);
            continue;
        }
        double dis = mobility->getCurrentPosition().distance(it->second.position);
        if (it->second.distance < min) {
            min = it->second.distance;
            add = it->second.nextHop;
            disMin = dis;
        }
        else if (it->second.distance == min && dis < disMin) {
            min = it->second.distance;
            add = it->second.nextHop;
            disMin = dis;
        }
        ++it;
    }
    return add;
}

int UdpVideoSend::getDistanceClusterHead(const L3Address &add) {
    if (clusterHeads.empty()) return -1; // return -1 if doesn't exist
    auto it = clusterHeads.find(add);
    if (it == clusterHeads.end())
        return -1;
    if ((simTime() - it->second.lasTime) > (par("allowedAdvertLoss").intValue() * advertInterval)) {
        clusterHeads.erase(it++);
        return -1;
    }
    return it->second.distance;
}


void UdpVideoSend::finish()
{
    if (!par("recordOnlyTotal")) {
        recordScalar("Frames P sent: ", totalSendP);
        recordScalar("Frames I sent: ", totalSendI);
        recordScalar("Frames B sent: ", totalSendB);
        recordScalar("Bytes P sent: ", totalBytesSendP);
        recordScalar("Bytes I sent: ", totalBytesSendI);
        recordScalar("Bytes B sent: ", totalBytesSendB);
        recordScalar("Initial Energy", par("energy").doubleValue());
        recordScalar("Remaining Energy", remainingEnergy);
    }


}

void UdpVideoSend::sendToControl(cPacket *pkt)
{
    if (senderControlTimer->isScheduled()) {
        queueToControl.push_back(pkt);
        return;
    }
    simtime_t duration = controller->getDuration(pkt);
    simtime_t delay = controller->getDelay();
    emit(sentPkSignal, pkt);
    scheduleAt(simTime()+duration, senderControlTimer);
    sendDirect(pkt, delay, duration, controller, "directGate");
}


void UdpVideoSend::handleTimer(cMessage *msg) {
   // handle other timers, for example periodic status information

    if (senderControlTimer == msg) {
        if (queueToControl.empty())
            return;
        cPacket *pkt = queueToControl.front();
        queueToControl.pop_front();

        simtime_t duration = controller->getDuration(pkt);
        simtime_t delay = controller->getDelay();
        emit(sentPkSignal, pkt);
        scheduleAt(simTime()+duration, senderControlTimer);
        sendDirect(pkt, delay, duration, controller, "directGate");
        return;
    }
    else if (energyTimer == msg) {
        if (remainingEnergy != -1) {
            if (this->iamClusterHead)
                remainingEnergy -= ((consumptionPerSec + (percetajeIncreaseEnergy * consumptionPerSec)) * par("energyEvalInterval").doubleValue());
            else
                remainingEnergy -= (consumptionPerSec * par("energyEvalInterval").doubleValue());
            if (remainingEnergy < 0)
                remainingEnergy = 0;
            if (remainingEnergy > 0)
                scheduleAt(simTime() + par("energyEvalInterval"), energyTimer);
        }
        return;
    }
    if (msg == actualizeStatus) {
        // Send new status message.
        L3Address ch;
        L3Address nextHop;
        if (!iamClusterHead) {
            if (getBestClusterHead(ch))
                nextHop = getNextHopBestClusterHead();
        }
        if (iamClusterHead || (!ch.isUnspecified() && !nextHop.isUnspecified())) {
            auto status = makeShared<ActualizeData>();
            status->setEnergy(remainingEnergy);
            status->setPosition(mobility->getCurrentPosition());
            status->setSpeed(mobility->getCurrentVelocity());
            status->setOrigin(myAddress);
            status->setChunkLength(B(20));
            status->setConstraintAreaMax(mobility->getConstraintAreaMax());
            status->setConstraintAreaMin(mobility->getConstraintAreaMin());
            status->setControllerId(controllerId);
            status->setZone(zoneId);
            auto packet = new Packet("ActualizePacket");
            packet->insertAtFront(status);
            if (iamClusterHead) {
                sendToControl(packet);
            }
            else {
                int interfaceId = CHK(interfaceTable->findInterfaceByName(par("broadcastInterface")))->getInterfaceId(); // TODO: Implement: support for multiple interfaces
                packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(interfaceId);
                socket.sendTo(packet, nextHop, localPort);
            }
        }
        scheduleAt(simTime() + par("actualizeTimer") - *jitterPar, actualizeStatus);
    }
    else if (msg->getKind() == KIND_DELAYEDSEND) {
        auto timer = check_and_cast<PacketHolderMessage*>(msg);
        auto pkt = timer->removeOwnedPacket();
        pkt->clearTags();
        int interfaceId = CHK(interfaceTable->findInterfaceByName(par("broadcastInterface")))->getInterfaceId(); // TODO: Implement: support for multiple interfaces
        pkt->addTagIfAbsent<InterfaceReq>()->setInterfaceId(interfaceId);
        socket.sendTo(pkt, myAddress.getAddressType()->getBroadcastAddress(), localPort);
        auto it = pendingSend.find(timer);
        if (it != pendingSend.end())
            pendingSend.erase(it);
        delete timer;
    }
    else if (msg == advertCluster) {
        refreshClusterHead();
    }
    else
        throw cRuntimeError("Timer unknown %s", msg->getName());
    // Other timers

}

void UdpVideoSend::handleMessageWhenUp(cMessage *msg)
{
    if (remainingEnergy <= 0 && remainingEnergy != -1) {
        if (!msg->isSelfMessage()) {
            delete msg;
            return;
        }
        // check this timer if it is necessary to delete.
        if (streamVector.empty() && streamVector.front().timer == msg) {
            delete msg;
            streamVector.clear();
        }
        return;
    }

    if (msg->isSelfMessage()) {
        // timer for a particular video stream expired, send packet
        if (!streamVector.empty() && streamVector.front().timer == msg)
            sendStreamData(msg);
        else
            handleTimer(msg);
    }
    else if (msg->getSenderModule() == controller) {
        handleControllerMessges(PK(msg));
    }
    else
        socket.processMessage(msg);
}


void UdpVideoSend::handleControllerMessges(cPacket *pkt)
{
    auto packet = check_and_cast<Packet *>(pkt);

    // process incoming packet
    auto control = packet->peekAtFront<ControlPacket>();
    if (control->getTypePacket() == STARTCLUSTERHEAD) {
        if (!iamClusterHead) {
            iamClusterHead = true;
            // Start here to send the information that this node is now a cluster head
            notifyNewClusterHead(*periodicJitter); // Should be sending this notification periodically?
        }
        processStreamRequest(packet);
    }
    else if (control->getTypePacket() == STOPCLUSTERHEAD) {
        if (iamClusterHead) {
            notifyEndClusterHead();
            iamClusterHead = false;
            // Notify here to the neighbor that this node is no more cluster head
        }
        processStopRequest(packet);
    }
    else if (control->getTypePacket() == REQUESTSTATUS) {
        auto status = makeShared<ActualizeData>();
        status->setEnergy(remainingEnergy);
        status->setPosition(mobility->getCurrentPosition());
        status->setSpeed(mobility->getCurrentVelocity());
        status->setOrigin(myAddress);
        status->setChunkLength(B(20));
        status->setConstraintAreaMax(mobility->getConstraintAreaMax());
        status->setConstraintAreaMin(mobility->getConstraintAreaMin());
        status->setControllerId(controllerId);
        status->setZone(zoneId);
        auto packet = new Packet("ActualizePacket");
        packet->insertAtFront(status);
        sendToControl(packet);
        delete pkt;
    }
}

#if 0
void UdpVideoSend::socketDataArrived(UdpSocket *sock, Packet *packet)
{
    // arrive messages of other nodes
    const auto chuck = packet->peekAtFront<FieldsChunk>();
            // check the type
    if (iamClusterHead && dynamicPtrCast<const ActualizeData> (chuck)) {
        // Connection with the controller active, assume cluster head
        sendToControl(packet);
        return;
    }
    else {
        // redirect to the cluster head
        // destAddr must be the next address to the cluster head or the cluster head
        // check the type.
        // check the type
        if (dynamicPtrCast<const AdvertClusterHead> (chuck)) {
           auto clusterInfo = packet->removeAtFront<AdvertClusterHead>();
           if (clusterInfo->getControllerId() != controllerId) {
               delete packet;
               return; // no correct controller
           }
           // check if old packet
           auto sender = clusterInfo->getOrigin();
           auto sq = clusterInfo->getSeqNumber();
           auto it = seqNumbers.find(sender);
           if (it != seqNumbers.end() && it->second >= sq) {
               delete packet;
               return;
           }


           // This only process the first packet, but it is possible to receive other packets with better routes to the CH
           // This should be take into account
           clusterInfo->setDistance(clusterInfo->getDistance()+1);
           clusterInfo->setNumHops(clusterInfo->getNumHops()-1);
           packet->insertAtFront(clusterInfo);

           if (clusterInfo->getTypePacket() == CHADVERT) {
               L3Address next = packet->getTag<L3AddressInd>()->getSrcAddress();
               addClusterHead(sender, clusterInfo->getDistance(), next);
           }
           else if (clusterInfo->getTypePacket() == CHENDADVERT) {
               deleteClusterHead(sender);
           }
           seqNumbers[sender] = sq;
           packet->insertAtFront(clusterInfo);
           if (clusterInfo->getNumHops() > 0) {
               // re-broadcast
               auto *timer = new PacketHolderMessage("LoadNg-send-jitter", KIND_DELAYEDSEND);
               timer->setOwnedPacket(packet);
               pendingSend.insert(timer);
               scheduleAt(simTime()+*jitterPar, timer);
//               socket.sendTo(packet, myAddress.getAddressType()->getBroadcastAddress(), localPort);
           }
           else
               delete packet;

        }
        else if (dynamicPtrCast<const ActualizeData> (chuck)) {
            // send to the CH
            auto destAddr = getNextHopBestClusterHead();
            if (destAddr.isUnspecified())
                delete packet; // Should send a error?
            else
                socket.sendTo(packet, destAddr, localPort);
        }
        else
            delete packet;
    }
}

#else
void UdpVideoSend::socketDataArrived(UdpSocket *sock, Packet *packet)
{
    // arrive messages of other nodes
    const auto chuck = packet->peekAtFront<FieldsChunk>();
            // check the type
    //if (iamClusterHead && dynamicPtrCast<const ActualizeData> (chuck)) {
    if (iamClusterHead) {
        // Connection with the controller active, assume cluster head
        if (dynamicPtrCast<const AdvertClusterHead> (chuck) == nullptr)
            sendToControl(packet);
        else
            delete packet; // ignore information packets of other CH
        return;
    }
    else {
        // redirect to the cluster head
        // destAddr must be the next address to the cluster head or the cluster head
        // check the type.
        // check the type
        if (dynamicPtrCast<const AdvertClusterHead> (chuck)) {
            auto clusterInfo = packet->peekAtFront<AdvertClusterHead>();
            if (clusterInfo->getControllerId() != controllerId) {
                delete packet;
                return; // no correct controller
            }
            // check if old packet

            auto sender = clusterInfo->getOrigin();
            auto sq = clusterInfo->getSeqNumber();
            auto it = seqNumbers.find(sender);
            if (it != seqNumbers.end() && it->second >= sq) {
                delete packet;
                return;
            }
            // This only process the first packet, but it is possible to receive other packets with better routes to the CH
            // This should be take into account

            if (clusterInfo->getTypePacket() == CHADVERT) {
                L3Address next = packet->getTag<L3AddressInd>()->getSrcAddress();
                addClusterHead(sender, clusterInfo->getDistance()+1, next, clusterInfo->getPosition(), clusterInfo->getSpeed());
            }
            else if (clusterInfo->getTypePacket() == CHENDADVERT) {
                deleteClusterHead(sender);
            }
            // check if cluster head present
            L3Address chAddr;
            if (getBestClusterHead(chAddr)) {
                if (getDistanceClusterHead(chAddr) == 1) {
                    // start video transmitting to the cluster head.
                    if (streamVector.empty()) {
                        // start video sequence
                        processStreamRequest(nullptr);
                    }
                }
            }
            seqNumbers[sender] = sq;
            if (clusterInfo->getNumHops() > 1) {
                auto clusterInfoAux = dynamicPtrCast<AdvertClusterHead>(clusterInfo->dupShared());
                clusterInfoAux->setDistance(clusterInfo->getDistance()+1);
                clusterInfoAux->setNumHops(clusterInfo->getNumHops()-1);
                auto pktAux = new Packet(packet->getName());
                pktAux->insertAtFront(clusterInfoAux);

                // re-broadcast
                auto *timer = new PacketHolderMessage("UdpVideoSend-send-jitter", KIND_DELAYEDSEND);
                timer->setOwnedPacket(pktAux);
                pendingSend.insert(timer);
                scheduleAt(simTime()+*jitterPar, timer);
                //socket.sendTo(packet, myAddress.getAddressType()->getBroadcastAddress(), localPort);
            }

            delete packet;
        }
        else if (dynamicPtrCast<const ActualizeData> (chuck)) {
            // send to the CH
            auto destAddr = getNextHopBestClusterHead();
            if (destAddr.isUnspecified())
                delete packet; // Should send a error?
            else {
                packet->trim();
                packet->clearTags();
                int interfaceId = CHK(interfaceTable->findInterfaceByName(par("broadcastInterface")))->getInterfaceId(); // TODO: Implement: support for multiple interfaces
                packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(interfaceId);
                socket.sendTo(packet, destAddr, localPort);
            }
        }
        else
            delete packet;
    }
}
#endif


void UdpVideoSend::socketErrorArrived(UdpSocket *socket, Indication *indication)
{
    EV_WARN << "Ignoring UDP error report " << indication->getName() << endl;
    delete indication;
}

void UdpVideoSend::socketClosed(UdpSocket *socket)
{
    if (operationalState == State::STOPPING_OPERATION)
        startActiveOperationExtraTimeOrFinish(par("stopOperationExtraTime"));
}


void UdpVideoSend::processStopRequest(Packet *msg)
{
    // check if the sender is the controller, in other case delete the packet
    if (controller != msg->getSenderModule()) {
        delete msg;
        return;
    }
    // check if this node is sending the video already
    if (streamVector.empty()) {
        delete msg;
        return;
    }
    if (streamVector.size() > 1)
        throw cRuntimeError("streamVector.size() > 1");
    // check if there is a valid cluster Head

    if (getNextHopBestClusterHead().isUnspecified()) {
        cancelAndDelete(streamVector.front().timer);
        lastSeqNum += (trace[streamVector.front().traceIndex].seqNum+1);
        streamVector.clear();
    }
    delete msg;
}




void UdpVideoSend::processStreamRequest(Packet *msg)
{
    // check if the sender is the controller, in other case delete the packet
    if (nullptr != msg && msg->getSenderModule() != controller) {
        delete msg;
        return;
    }
    // register video stream...

    // check if this node is sending the video already
    if (!streamVector.empty()) {
        if (nullptr != msg)
            delete msg;
        return;
    }

    // check cluster heads
    if (!iamClusterHead && getNextHopBestClusterHead().isUnspecified()) {
        if (nullptr != msg)
            delete msg;
        return;
    }

    cMessage *timer = new cMessage("VideoStreamTmr");
    streamVector.resize(1);
    VideoStreamData *d = &streamVector.front();
    d->timer = timer;
    // d->videoSize = (*videoSize);
    // d->bytesLeft = d->videoSize;
    d->bytesLeft = -1; // no ending until the order arrives

    d->traceIndex = 0;
    d->timeInit = simTime();
    d->fileTrace = false;
    double stop = (*stopTime);
    if (stop > 0)
        d->stopTime = simTime() + stop;
    else
        d->stopTime = 0;
    d->numPkSent = 0;

    if (!trace.empty())
        d->fileTrace = true;
    // ... then transmit first packet right away
    sendStreamData(timer);

    numStreams++;

    if (msg != nullptr)
        delete msg;
    //emit(reqStreamBytesSignal, d->videoSize);
}

void UdpVideoSend::sendStreamData(cMessage *timer)
{
    bool deleteTimer = false;


    VideoStreamData *d = nullptr;
    auto it = streamVector.begin();
    for (it = streamVector.begin(); it != streamVector.end(); ++it) {
        if (it->timer == timer) {
            d = &(*it);
            break;
        }
    }

    if (it == streamVector.end())
        throw cRuntimeError("Model error: Stream not found for timer");

    // check cluster heads
    if (!iamClusterHead && getNextHopBestClusterHead().isUnspecified()) {
        // cancel
        if (!trace.empty())
            lastSeqNum += (trace[streamVector.front().traceIndex].seqNum+1);
        streamVector.erase(it);
        cancelAndDelete(timer);
        return;
    }

    // if stop time, cancel
    if (d->stopTime > 0 && d->stopTime < simTime()) {
        if (!trace.empty())
            lastSeqNum += (trace[streamVector.front().traceIndex].seqNum+1);
        streamVector.erase(it);
        cancelAndDelete(timer);
        return;
    }

    // if not cluster head no to send video
    if (!iamClusterHead) {
        int dist = 100;
        auto destAddr = getNextHopBestClusterHead();
        if (!destAddr.isUnspecified())
            dist = getDistanceClusterHead(destAddr);
        if (dist > 1 || dist == -1) {
            if (!trace.empty())
                lastSeqNum += (trace[streamVector.front().traceIndex].seqNum+1);
            streamVector.erase(it);
            cancelAndDelete(timer);
            return;
        }
    }

    // send
    Packet *pkt = new Packet("VideoStrmPk");
    if (!d->fileTrace) {
        long pktLen = packetLen->intValue();

        if (d->bytesLeft != -1 && pktLen > d->bytesLeft)
            pktLen = d->bytesLeft;

        const auto& payload = makeShared<VideoPacket>();
        payload->setChunkLength(B(pktLen));
        payload->setFrameSize(B(pktLen));
        payload->setSeqNum(d->numPkSent);

        payload->addTag<CreationTimeTag>()->setCreationTime(simTime());


        pkt->insertAtBack(payload);

        if (d->bytesLeft != -1)
            d->bytesLeft -= pktLen;

        d->numPkSent++;
        numPkSent++;
        // reschedule timer if there's bytes left to send
        if (d->bytesLeft != 0) {
            simtime_t interval = (*sendInterval);
            scheduleAt(simTime()+interval, timer);
        }
        else {
            deleteTimer = true;
        }
    }
    else {
        if (macroPackets) {
            simtime_t tm;
            uint64_t size = 0;

            do{
                const auto& videopk = makeShared<VideoPacket>();
                videopk->setChunkLength(b(trace[d->traceIndex].size));
                videopk->setType(trace[d->traceIndex].type);

                auto len = B(videopk->getChunkLength());
                videopk->setFrameSize(len);

                if (trace[d->traceIndex].type == 'P' || trace[d->traceIndex].type == 'p') {
                    totalSendP++;
                    totalBytesSendP += len.get();
                    if (iamClusterHead)
                        totalBytesSendPCh += len.get();
                    else
                        totalBytesSendPCl += len.get();
                }
                if (trace[d->traceIndex].type == 'I' || trace[d->traceIndex].type == 'i') {
                    totalSendI++;
                    totalBytesSendI += len.get();
                    if (iamClusterHead)
                        totalBytesSendICh += len.get();
                    else
                        totalBytesSendICl += len.get();
                }
                if (trace[d->traceIndex].type == 'B' || trace[d->traceIndex].type == 'b') {
                    totalSendB++;
                    totalBytesSendB += len.get();
                    if (iamClusterHead)
                        totalBytesSendBCh += len.get();
                    else
                        totalBytesSendBCl += len.get();
                }


                videopk->setSeqNum(trace[d->traceIndex].seqNum + lastSeqNum);
                videopk->addTag<CreationTimeTag>()->setCreationTime(simTime());

                pkt->insertAtBack(videopk);
                size = pkt->getByteLength();
                d->traceIndex++;
                if (d->traceIndex >= trace.size()) {
                    lastSeqNum += (trace.back().seqNum+1);
                    d->traceIndex = 0;
                    d->timeInit = simTime();
                }
            } while((size + trace[d->traceIndex].size/8 < maxSizeMacro) && (d->traceIndex < trace.size()));

        }
        else {
            const auto& videopk = makeShared<VideoPacket>();

            videopk->setChunkLength(b(trace[d->traceIndex].size));

            videopk->setType(trace[d->traceIndex].type);
            if (trace[d->traceIndex].type == 'P' || trace[d->traceIndex].type == 'p')
                totalSendP++;
            if (trace[d->traceIndex].type == 'I' || trace[d->traceIndex].type == 'i')
                totalSendI++;
            if (trace[d->traceIndex].type == 'B' || trace[d->traceIndex].type == 'b')
                totalSendB++;

            videopk->setSeqNum(trace[d->traceIndex].seqNum + lastSeqNum);
            auto len = B(videopk->getChunkLength());
            videopk->addTag<CreationTimeTag>()->setCreationTime(simTime());
            videopk->setFrameSize(len);
            pkt->insertAtBack(videopk);
            d->traceIndex++;
            if (d->traceIndex >= trace.size()) {
                lastSeqNum += (trace.back().seqNum+1);
                d->traceIndex = 0;
            }
        }
        if (d->traceIndex >= trace.size())
            deleteTimer = true;
        else
            scheduleAt(d->timeInit + trace[d->traceIndex].timeFrame, timer);

    }
    auto control = makeShared<ControlPacket>();
    control->setOrigin(myAddress);
    control->setDestination(myAddress.getAddressType()->getBroadcastAddress());
    control->setChunkLength(B(20));
    control->setTypePacket(VIDEODATA);
    pkt->insertAtFront(control);

    if (iamClusterHead)
        sendToControl(pkt);
    else {
        auto destAddr = getNextHopBestClusterHead();
        if (destAddr.isUnspecified())
            delete pkt; // Should send a error?
        else {
            int interfaceId = CHK(interfaceTable->findInterfaceByName(par("broadcastInterface")))->getInterfaceId(); // TODO: Implement: support for multiple interfaces
            pkt->addTagIfAbsent<InterfaceReq>()->setInterfaceId(interfaceId);
            socket.sendTo(pkt, destAddr, localPort);
        }
    }

    emit(sentPkSignal, pkt);
    if (deleteTimer) {
        if (!trace.empty())
            lastSeqNum += (trace[streamVector.front().traceIndex].seqNum+1);
        streamVector.erase(it);
        delete timer;
    }
}

void UdpVideoSend::clearStreams()
{
    for(auto it = streamVector.begin(); it  != streamVector.end(); ++it)
        cancelAndDelete(it->timer);
    streamVector.clear();
}

/*
void UdpVideoSend::handleStartOperation(LifecycleOperation *operation)
{
    simtime_t start = std::max(startTime, simTime());
    if ((stopTime < SIMTIME_ZERO) || (start < stopTime) || (start == stopTime && startTime == stopTime)) {
        selfMsg->setKind(START);
        scheduleAt(start, selfMsg);
    }
}
*/

void UdpVideoSend::handleStartOperation(LifecycleOperation *operation)
{

}

void UdpVideoSend::handleStopOperation(LifecycleOperation *operation)
{
    cancelEvent(selfMsg);
    clearStreams();
    socket.setCallback(nullptr);
    socket.close();
    delayActiveOperationFinish(par("stopOperationTimeout"));
}


void UdpVideoSend::handleCrashOperation(LifecycleOperation *operation)
{
    cancelEvent(selfMsg);
    clearStreams();
    if (operation->getRootModule() != getContainingNode(this))     // closes socket when the application crashed only
        socket.destroy();    //TODO  in real operating systems, program crash detected by OS and OS closes sockets of crashed programs.
    socket.setCallback(nullptr);
}

