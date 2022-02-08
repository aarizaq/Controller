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

#ifndef __INET_UDPVIDEOSTREAMSVR2_H
#define __INET_UDPVIDEOSTREAMSVR2_H


#include <vector>
#include <deque>
#include "inet/applications/base/ApplicationBase.h"
#include "inet/common/packet/Packet.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/networklayer/contract/IRoutingTable.h"
#include "Controller.h"
#include "ControlPackets_m.h"
#include "inet/mobility/contract/IMobility.h"
#include <deque>

/**
 * Stream VBR video streams to clients.
 *
 * Cooperates with UDPVideoStreamCli. UDPVideoStreamCli requests a stream
 * and UDPVideoStreamSvr starts streaming to them. Capable of handling
 * streaming to multiple clients.
 */
using namespace inet;

class UdpVideoSend :  public ApplicationBase, public UdpSocket::ICallback
{
public:
    struct ChData {
        L3Address nextHop;
        int distance;
        Coord position;
        Coord speed;
        simtime_t lasTime;
    };

private:
    Controller *controller = nullptr;
    IMobility *mobility = nullptr;
    double remainingEnergy = -1;
    double initialEnergy = -1;
    double consumptionPerSec = 0;
    double percetajeIncreaseEnergy = 0;
    cMessage *energyTimer = nullptr;
    L3Address myAddress;

    cMessage *selfMsg = nullptr;

    cMessage *actualizeStatus = nullptr;

    bool iamClusterHead = false;

    int controllerId = -1;
    int zoneId = -1;

    std::map<L3Address, ChData > clusterHeads;
    std::set<PacketHolderMessage *> pendingSend;

    cMessage *senderControlTimer;
    std::deque<cPacket *> queueToControl;
    uint64_t seqNumber = 0;
    std::map<L3Address,uint64_t> seqNumbers;

    cPar *jitterPar = nullptr;
    cPar *periodicJitter = nullptr;
    simtime_t advertInterval;

    cMessage *advertCluster = nullptr;

    uint64_t totalSendP = 0;
    uint64_t totalSendI = 0;
    uint64_t totalSendB = 0;

    uint64_t totalBytesSendP = 0;
    uint64_t totalBytesSendI = 0;
    uint64_t totalBytesSendB = 0;

    uint64_t totalBytesSendPCh = 0;
    uint64_t totalBytesSendICh = 0;
    uint64_t totalBytesSendBCh = 0;

    uint64_t totalBytesSendPCl = 0;
    uint64_t totalBytesSendICl = 0;
    uint64_t totalBytesSendBCl = 0;

  public:
    /**
     * Stores information on a video stream
     */
    struct VideoStreamData
    {
        cMessage *timer;          ///< self timer msg
        long videoSize = -1;           ///< total size of video
        long bytesLeft = -1;           ///< bytes left to transmit
        long numPkSent;           ///< number of packets sent
        simtime_t stopTime;       ///< stop connection
        bool fileTrace;
        unsigned int traceIndex;
        simtime_t timeInit;
        VideoStreamData() { timer = nullptr; videoSize = bytesLeft = 0; numPkSent = 0; }
    };

    uint64_t lastSeqNum = 0;

    struct VideoInfo
    {
            simtime_t timeFrame;
            uint32_t seqNum;
            char type;
            uint32_t size;
    };

  protected:
    typedef std::deque<VideoStreamData> VideoStreamMap;
    typedef std::deque<VideoInfo> VideoTrace;
    VideoTrace trace;
    bool macroPackets;
    uint64_t maxSizeMacro;
    simtime_t initTime;

    VideoStreamMap streamVector;
    UdpSocket socket;

    // module parameters
    int localPort;
    cPar *sendInterval;
    cPar *packetLen;
    cPar *videoSize;
    cPar *stopTime;

    IRoutingTable *routingTable = nullptr;
    IInterfaceTable *interfaceTable = nullptr;

    // statistics
    unsigned int numStreams;  // number of video streams served
    unsigned long numPkSent;  // total number of packets sent
    static simsignal_t reqStreamBytesSignal;  // length of video streams served
    static simsignal_t sentPkSignal;

  public:
    double getInitalEnergy() {return initialEnergy;}
    double getRemainEnergy() { return remainingEnergy;}
    double getConsumedEnergy() {return (initialEnergy - remainingEnergy);}

    uint64_t getTotalBytesSendP() {return totalBytesSendP;}
    uint64_t getTotalBytesSendI() {return totalBytesSendI;}
    uint64_t getTotalBytesSendB() {return totalBytesSendB;}

    uint64_t getTotalSendP() {return totalSendP;}
    uint64_t getTotalSendI() {return totalSendI;}
    uint64_t getTotalSendB() {return totalSendB;}

    uint64_t getTotalBytesSendPCh() {return totalBytesSendPCh;}
    uint64_t getTotalBytesSendICh() {return totalBytesSendICh;}
    uint64_t getTotalBytesSendBCh() {return totalBytesSendBCh;}

    uint64_t getTotalBytesSendPCl() {return totalBytesSendPCl;}
    uint64_t getTotalBytesSendICl() {return totalBytesSendICl;}
    uint64_t getTotalBytesSendBCl() {return totalBytesSendBCl;}
  protected:
    virtual void deleteClusterHead(const L3Address &);
    virtual void addClusterHead(const L3Address &, const int & distance, const L3Address &, const Coord &, const Coord &);
    virtual bool getBestClusterHead(L3Address &); // return false if there isn't a cluster head
    virtual L3Address getNextHopBestClusterHead(); // return false if there isn't a cluster head

    virtual int getDistanceClusterHead(const L3Address &); // return -1 if doesn't exist


    virtual void notifyEndClusterHead();
    virtual void notifyNewClusterHead(const simtime_t &);
    virtual void refreshClusterHead();

    virtual void sendToControl(cPacket *);

    virtual void handleTimer(cMessage *timer);

    virtual void handleControllerMessges(cPacket *pkt);
    // process stream request from client
    virtual void processStreamRequest(Packet *msg);

    virtual void processStopRequest(Packet *msg);

    // send a packet of the given video stream
    virtual void sendStreamData(cMessage *timer);

    // parse utexas video traces
    virtual void fileParser(const char *fileName);

    ///@name Overridden cSimpleModule functions
    //@{

    virtual int numInitStages() const  override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage)  override;
    virtual void finish() override;
    virtual void handleMessageWhenUp(cMessage* msg) override;
    void clearStreams();
    //@}

    //ApplicationBase:

    virtual void handleStartOperation(LifecycleOperation *operation) override;
    virtual void handleStopOperation(LifecycleOperation *operation) override;
    virtual void handleCrashOperation(LifecycleOperation *operation) override;

    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override;
    virtual void socketClosed(UdpSocket *socket) override;

  public:
    UdpVideoSend();
    virtual ~UdpVideoSend();

};

#endif

