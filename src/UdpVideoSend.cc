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
#include "inet/common/ModuleAccess.h"
#include "inet/common/TimeTag_m.h"
#include "inet/common/packet/chunk/ByteCountChunk.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/transportlayer/common/L4PortTag_m.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "ControlPackets_m.h"
#include "inet/networklayer/contract/IL3AddressType.h"


Define_Module(UdpVideoSend);

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
    if (energyTimer)
        cancelAndDelete(energyTimer);
    clearStreams();
    trace.clear();
    cancelAndDelete(senderControlTimer);
}

void UdpVideoSend::initialize(int stage)
{
    ApplicationBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL)
    {
        sendInterval = &par("sendInterval");
        packetLen = &par("packetLen");
        videoSize = &par("videoSize");
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
        if (remainingEnergy != -1) {
            energyTimer = new cMessage("Energy timer");
            scheduleAt(simTime()+par("energyTimerEvaluation"),energyTimer);
        }
        senderControlTimer = new cMessage("senderControlTimer");
        scheduleAt(simTime() + par("sendingStatusInterval"), senderControlTimer);

        energyTimer = new cMessage("energyTimer");
        remainingEnergy = par("energy");
        consumptionPerSec =  par("consumptionPerTimeUnits");

        scheduleAt(simTime() + par("energyEvalInterval"), energyTimer);

        actualizeStatus = new cMessage("ActualizeStatus");
        scheduleAt(simTime() + par("startTimer") + par("actualizeTimer"), actualizeStatus);
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {


        auto mod = cSimulation::getActiveSimulation()->getSystemModule()->getModuleByPath("controller");
        controller = check_and_cast<Controller *>(mod);
        auto node = getContainingNode(this);
        myAddress = L3AddressResolver().addressOf(node);
        mobility = check_and_cast<IMobility *>(node->getSubmodule("mobility"));
        controller->registerModule(this, mobility->getCurrentPosition(), mobility->getCurrentVelocity(), remainingEnergy);
    }
}


void UdpVideoSend::deleteClusterHead(const L3Address & addr) {
    auto it = clusterHeads.find(addr);
    if (it != clusterHeads.end())
        clusterHeads.erase(it);
}

void UdpVideoSend::addClusterHead(const L3Address & addr, const int & distance, const L3Address &next) {
    auto it = clusterHeads.find(addr);
    if (it == clusterHeads.end()) {
        ChData data;
        data.distance = distance;
        data.lasTime = simTime();
        data.nextHop = next;
        clusterHeads[addr] = data;
    }
    else {
        it->second.distance = distance;
        it->second.lasTime = simTime();
        it->second.nextHop = next;
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
    header->setChunkLength(B(30)); // check the size it is important
    auto pkt = new Packet("CHENDADVERT");
    pkt->insertAtFront(header);
    socket.sendTo(pkt, myAddress.getAddressType()->getBroadcastAddress(), localPort);
}

void UdpVideoSend::notifyNewClusterHead() {
    auto header = makeShared<AdvertClusterHead>();
    header->setOrigin(myAddress);
    header->setDestination(myAddress.getAddressType()->getBroadcastAddress());
    header->setSeqNumber(seqNumber++);
    header->setNumHops(par("maxNumHops"));
    header->setDistance(0);
    header->setTypePacket(CHADVERT);
    header->setChunkLength(B(30)); // check the size it is important
    auto pkt = new Packet("CHADVERT");
    pkt->insertAtFront(header);
    socket.sendTo(pkt, myAddress.getAddressType()->getBroadcastAddress(), localPort);
}


bool UdpVideoSend::getBestClusterHead(L3Address &add) {
    if (clusterHeads.empty()) {
        add = L3Address();
        return false; // return false if there isn't a cluster head
    }
    int min = 100000;
    for (auto elem : clusterHeads) {
        if (elem.second.distance < min) {
            min = elem.second.distance;
            add = elem.first;
        }
    }
    return true;
}

L3Address UdpVideoSend::getNextHopBestClusterHead()
{
    if (clusterHeads.empty()) {
        return L3Address(); // return false if there isn't a cluster head
    }
    int min = 100000;
    L3Address add;
    for (auto elem : clusterHeads) {
        if (elem.second.distance < min) {
            min = elem.second.distance;
            add = elem.second.nextHop;
        }
    }
    return add;
}

int UdpVideoSend::getDistanceClusterHead(const L3Address &add) {
    if (clusterHeads.empty()) return -1; // return -1 if doesn't exist
    auto it = clusterHeads.find(add);
    if (it == clusterHeads.end())
        return -1;
    return it->second.distance;
}


void UdpVideoSend::finish()
{
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
    }
    else if (energyTimer == msg) {
        if (remainingEnergy != -1) {
            remainingEnergy -= (consumptionPerSec * par("energyEvalInterval").doubleValue());
            if (remainingEnergy < 0)
                remainingEnergy = 0;
            if (remainingEnergy > 0)
                scheduleAt(simTime() + par("energyEvalInterval"), energyTimer);
        }
    }
    if (msg == actualizeStatus) {
        // Send new status message.

        L3Address ch;
        L3Address nextHop;
        if (!iamClusterHead) {
            if (getBestClusterHead(ch))
                nextHop = getNextHopBestClusterHead();
        }
        if (iamClusterHead || (!ch.isUnspecified() && nextHop.isUnspecified())) {
            auto status = makeShared<ActualizeData>();
            status->setEnergy(remainingEnergy);
            status->setPosition(mobility->getCurrentPosition());
            status->setSpeed(mobility->getCurrentVelocity());
            status->setOrigin(myAddress);
            status->setChunkLength(B(20));
            auto packet = new Packet("ActualizePacket");
            packet->insertAtFront(status);
            if (iamClusterHead)
                sendToControl(packet);
            else
                socket.sendTo(packet, nextHop, localPort);
        }
        scheduleAt(simTime() + par("actualizeTimer"), actualizeStatus);

    }
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
    if (control->getTypePacket() == STARTVIDEO) {
        if (!iamClusterHead) {
            iamClusterHead = true;
            // Start here to send the information that this node is now a cluster head
            notifyNewClusterHead(); // Should be sending this notification periodically?
        }
        processStreamRequest(packet);
    }
    else if (control->getTypePacket() == STOPVIDEO) {
        if (iamClusterHead) {
            iamClusterHead = false;
            // Notify here to the neighbor that this node is no more cluster head
            notifyEndClusterHead();
        }
        processStopRequest(packet);
    }
}

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
               socket.sendTo(packet, myAddress.getAddressType()->getBroadcastAddress(), localPort);
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
    // register video stream...

    // check if this node is sending the video alredy
    if (streamVector.empty()) {
        delete msg;
        return;
    }
    if (streamVector.size() > 1)
        throw cRuntimeError("streamVector.size() > 1");
    cancelAndDelete(streamVector.front().timer);
    streamVector.clear();
}

void UdpVideoSend::processStreamRequest(Packet *msg)
{
    // check if the sender is the controller, in other case delete the packet
    if (controller != msg->getSenderModule()) {
        delete msg;
        return;
    }
    // register video stream...

    // check if this node is sending the video alredy
    if (!streamVector.empty()) {
        delete msg;
        return;
    }

    cMessage *timer = new cMessage("VideoStreamTmr");
    streamVector.resize(1);
    VideoStreamData *d = &streamVector.front();
    d->timer = timer;
    // d->videoSize = (*videoSize);
    // d->bytesLeft = d->videoSize;
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
    delete msg;
    // ... then transmit first packet right away
    sendStreamData(timer);

    numStreams++;
    emit(reqStreamBytesSignal, d->videoSize);
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


    // if stop time, cancel
    if (d->stopTime > 0 && d->stopTime < simTime()) {
        streamVector.erase(it);
        cancelAndDelete(timer);
        return;
    }

    // if not cluster head no to send video
    if (!iamClusterHead) {
        streamVector.erase(it);
        cancelAndDelete(timer);
        return;
    }

    // send

    Packet *pkt = new Packet("VideoStrmPk");
    if (!d->fileTrace) {
        long pktLen = packetLen->intValue();

        if (pktLen > d->bytesLeft)
            pktLen = d->bytesLeft;

        const auto& payload = makeShared<ByteCountChunk>(B(pktLen));
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
                videopk->setSeqNum(trace[d->traceIndex].seqNum);
                videopk->addTag<CreationTimeTag>()->setCreationTime(simTime());
                auto len = B(videopk->getChunkLength());
                videopk->setFrameSize(len);
                pkt->insertAtBack(videopk);
                d->traceIndex++;
            } while((size + trace[d->traceIndex].size/8 < maxSizeMacro) && (d->traceIndex < trace.size()));

        }
        else {
            const auto& videopk = makeShared<VideoPacket>();

            videopk->setChunkLength(b(trace[d->traceIndex].size));

            videopk->setType(trace[d->traceIndex].type);
            videopk->setSeqNum(trace[d->traceIndex].seqNum);
            auto len = B(videopk->getChunkLength());
            videopk->addTag<CreationTimeTag>()->setCreationTime(simTime());
            videopk->setFrameSize(len);
            pkt->insertAtBack(videopk);
            d->traceIndex++;
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

    sendToControl(pkt);
    emit(sentPkSignal, pkt);
    if (deleteTimer) {
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

void UdpVideoSend::handleStartOperation(LifecycleOperation *operation)
{
    socket.setOutputGate(gate("socketOut"));
    socket.bind(localPort);
    socket.setCallback(this);
}

void UdpVideoSend::handleStopOperation(LifecycleOperation *operation)
{
    clearStreams();
    socket.setCallback(nullptr);
    socket.close();
    delayActiveOperationFinish(par("stopOperationTimeout"));
}

void UdpVideoSend::handleCrashOperation(LifecycleOperation *operation)
{
    clearStreams();
    if (operation->getRootModule() != getContainingNode(this))     // closes socket when the application crashed only
        socket.destroy();    //TODO  in real operating systems, program crash detected by OS and OS closes sockets of crashed programs.
    socket.setCallback(nullptr);
}

