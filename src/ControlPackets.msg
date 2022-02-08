//
// Copyright (C) 2008
// DSDV simple example for INET (add-on)
// Version 1.0
// Diogo Antio & Pedro Menezes
// Instituto Superior Tecnico
// Lisboa - Portugal
// This version and newer version can be found at http://dsdv.has.it
// This code was written while assisting the course "Redes moveis e sem fios" http://comp.ist.utl.pt/ec-cm
// Autorization to use and modify this code not needed :P
// The authors hope it will be useful to help understand how
// INET and OMNET++ works(more specifically INET 20061020 and omnet++ 3.3).
// Also we hope it will help in the developing of routing protocols using INET.
//

import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;
import inet.common.packet.Packet;
import inet.networklayer.common.L3Address;
import inet.common.geometry.Geometry;

enum KindDelayEnum {
     KIND_DELAYEDSEND = 100;
     KIND_SMALLDELAY = 101;
     }


enum PacketType
{    
    STARTCLUSTERHEAD = 1;
    STOPCLUSTERHEAD = 2;
    CONTROLDATA = 3;
    VIDEODATA = 4;
    CHADVERT = 5;
    CHENDADVERT = 6;
    REQUESTSTATUS = 7;
};


class ControlPacket extends inet::FieldsChunk
{
    int typePacket;
    inet::L3Address destination;
    inet::L3Address origin;
}


class AdvertClusterHead extends ControlPacket
{
    int numHops;
    uint64_t seqNumber;
    int distance;
    inet::Coord position;
    inet::Coord speed;
    int controllerId = 0; // at least it will use controller 0
    int zone = 0;
}

class ActualizeData extends ControlPacket
{
    typePacket = CONTROLDATA;
    inet::Coord position;
    inet::Coord speed;
    inet::Coord constraintAreaMax;
    inet::Coord constraintAreaMin;
    int controllerId = 0; // at least it will use controller 0
    int zone = 0;
    double energy;
}

message PacketHolderMessage
{
    kind = KIND_DELAYEDSEND;
    inet::Packet *ownedPacket @owned;
}

