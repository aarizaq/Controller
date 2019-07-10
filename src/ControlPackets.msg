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
import inet.networklayer.common.L3Address;
import inet.common.geometry.Geometry;

enum PacketType
{    
    STARTVIDEO = 1;
    STOPVIDEO = 2;
    CONTROLDATA = 3;
    VIDEODATA = 4;
    CHADVERT = 5;
    CHENDADVERT = 6;
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
}

class ActualizeData extends ControlPacket
{
    typePacket = CONTROLDATA;
    inet::Coord position;
    inet::Coord speed;
    double energy;
}



