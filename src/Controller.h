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

//#define debugCode

#include <omnetpp.h>
#include <map>
#include <vector>
#include <list>
#include <deque>
#include "inet/networklayer/common/L3Address.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/geometry/common/Coord.h"
#include "inet/common/Units.h"
#include "inet/mobility/contract/IMobility.h"
#ifdef debugCode
#include "inet/common/WirelessGetNeig.h"
#endif



using namespace omnetpp;
using namespace inet;

struct CoverageRegion {
    Coord first; // upper corner
    Coord second; // lower corner
    Coord center;
    bool operator == (const CoverageRegion other) const {
        if (first == other.first && second == other.second && center == other.center)
            return true;
        return false;
    }
};

class ComputeOverlappingArea {
public:
    enum StopType {ABSOLUTE, PERCENTAGE, DISTANCE};
private:
    struct Range {
        double less;
        double greater;

        Range(double l, double g) {
            less = (l < g) ? l : g;
            greater = (l + g) - less;
        }

        bool IsOverlapping(const Range& other) const {
            return !(less > other.greater || other.less > greater);
        }

        void Merge(const Range& other) {
            if (IsOverlapping(other)) {
                less = (less < other.less) ? less : other.less;
                greater = (greater > other.greater) ? greater : other.greater;
            }
        }
    };
    std::vector<CoverageRegion> rectangles;
    double stopVal = 0;
    StopType type;

protected:

    virtual void getAllX(const std::vector<CoverageRegion>& rects, std::vector<double>& xes);
    virtual double getArea(std::vector<CoverageRegion> rects);
    virtual void getRangesOfY(const std::vector<CoverageRegion>& rects, std::vector<CoverageRegion>::const_iterator iterRect, const Range& rangeX, std::list<Range>& rangesOfY);
    virtual void insertRangeY(std::list<Range>& rangesOfY, Range& rangeY);
    virtual double getRectArea(const Range& rangeX, const std::list<Range>& rangesOfY);
public:

    double getOverlappingArea(std::vector<CoverageRegion> &);
    ComputeOverlappingArea(std::vector<CoverageRegion> , double stopValue, StopType = ABSOLUTE);
    ComputeOverlappingArea(double stopValue, StopType = ABSOLUTE);
    virtual void setCoverageArea(std::vector<CoverageRegion> &);


    virtual double getTotalArea();
    virtual double getTotalArea(std::vector<CoverageRegion> &);
    virtual void runUntil(std::vector<CoverageRegion> &, double &); // iterate until the gains in the new rectangle is lower than the stop value set in the constructor
    virtual void iterateAll(std::vector<CoverageRegion> &, double &, std::vector<int> &); // iterate over all rectangles avoiding the rectangles that don't increase the coverage area bigger than the stop value set in the constructor
    virtual void iterateMinDistance(const double &minDist, std::vector<CoverageRegion> &returnRect, double &totalArea, std::vector<int> &pos);
 };

class GSO;

class Controller : public cSimpleModule {

public:
    struct NodeData {
        std::deque<cPacket *> queue;
        cMessage *queueTimer = nullptr;
        cModule * module = nullptr;
        IMobility *mobility = nullptr;
        bool isClusterHead = false;
        Coord position;
        Coord speed;
        Coord constraintAreaMin;
        Coord constraintAreaMax;
        simtime_t lastInfo;
        double energy = 0;
        int numNeigh = 0;
        double bandwith;
        uint64_t totalBytesRec;
        simtime_t startCh;
        simtime_t totalTimeCh;
        rad aperture;
        double getCoverageRegion()
        {
            double h = position.z;
            if (aperture.get() > 170)
                throw cRuntimeError("Aperture too high");
            double lmed = h*std::tan(aperture.get());
            CoverageRegion reg;
            double l = 2*lmed;
            return (l*l);
        }

// GSO Data.
        double Lc = 0;
        double Rd = 0;
        double Fitness;
        L3Address Ld;
        std::vector<L3Address> Neigh;

    };

    typedef std::map<L3Address, NodeData *> ListNodes;

    struct Area {
        Coord areaMin;
        Coord areaMax;
        ListNodes listNodes;
    };
private:

    std::vector<GSO *> gsoVectors;
    std::vector<Area> listNodesArea;
    ListNodes listNodesAux;

    std::map<int, L3Address> listNodesIdAddr;
    std::map<int, NodeData *> listNodesId;

    simtime_t delay;
    double bitRate = 0;
    double MINDISTANCECH = 0;

    cMessage * computeTimer = nullptr;

    double stopVal = 0;
    ComputeOverlappingArea::StopType type;

    static simsignal_t areaRec;
    static simsignal_t numNodes;

// statistics.
    cOutVector totalAreaVec;

    cHistogram area1Stat;
    cOutVector area1Vec;
    cOutVector numCh1;
    cOutVector areaPlusNeig1;
    cOutVector totalDivSelected1;
    cOutVector totalDivSelectedPlusNeig1;

    cStdDev stdArea1;
    cStdDev stdAreaPlusN1;
    cStdDev nunCh1;

#ifdef debugCode
    cHistogram area2Stat;
    cOutVector area2Vec;
    cOutVector numCh2;
    cOutVector areaPlusNeig2;
    cOutVector totalDivSelected2;
    cOutVector totalDivSelectedPlusNeig2;

    cStdDev stdArea2;
    cStdDev stdAreaPlusN2;
    cStdDev nunCh2;


    cHistogram area3Stat;
    cOutVector area3Vec;
    cOutVector numCh3;
    cOutVector areaPlusNeig3;
    cOutVector totalDivSelected3;
    cOutVector totalDivSelectedPlusNeig3;
    cStdDev stdArea3;
    cStdDev stdAreaPlusN3;
    cStdDev nunCh3;
#endif
    cHistogram totalAreaTotalOverlaping;



    uint64_t numRecPackets = 0;

    struct NodesData {
        uint64_t numPframes = 0;
        uint64_t numBframes = 0;
        uint64_t numIframes = 0;

        uint64_t totalBytesP = 0;
        uint64_t totalBytesI = 0;
        uint64_t totalBytesB = 0;
        uint64_t lastSeqNum = 0;
    };

    std::map<L3Address,  NodesData> nodesData;

    static simsignal_t rcvdPkSignal;

#ifdef debugCode
    WirelessGetNeig *wirelessGetNeig = nullptr;
#endif

public:
    Controller();
    virtual ~Controller();
    virtual void registerModule(cModule *, IMobility *, const Coord &pos, const Coord &speed, const double &energy,  const Coord & constraintAreaMax,  const Coord & constraintAreaMin, int zone);
    virtual simtime_t getDelay() {return delay;}
    virtual simtime_t getDuration(cPacket * pkt) {return pkt->getBitLength()/bitRate;}

protected:
    virtual void changeSenderNode(const L3Address &oldNode, const L3Address & newNode);
    virtual void activatedSenderNode(const L3Address & newNode) {changeSenderNode(L3Address(), newNode);}
    virtual void deactivatedSenderNode(const L3Address & oldNode) {changeSenderNode(oldNode, L3Address());}

    virtual bool sendData(const L3Address &, cPacket *);
    virtual void requestStatus(const L3Address &node);
    virtual int numInitStages() const  override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage)  override;
    virtual void handleMessage(cMessage *) override;
    virtual void finish() override;

    virtual CoverageRegion getCoverageRegion(const Coord &, const rad &);

    friend void Topsis(const std::map<L3Address, NodeData>& listNodes,  std::vector<std::pair<int, double>>& sortedlist);


};

#endif /* SRC_CONTROLLER_H_ */
