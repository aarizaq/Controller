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

package src;

//
// TODO auto-generated module
//
simple Controller
{
     volatile double delay @unit(s) = default(1ms);
     volatile double bitRate @unit(bps) = default(10Mbps);
     volatile double FirstComputation @unit(s) = default(0s);
     volatile double computeInterval @unit(s) = default(5s);
     double MINDISTANCECH @unit(m);
     bool recordOnlyTotal = true;
     bool UseGso = default(false);
     
     double stopValue = default(20);
     string stopType @enum("absolute", "percentage","distance") = default("absolute");

     @signal[TotalAreaRecorded];
     @signal[NumDronesSelected];
      
      @signal[rcvdPk](type=cPacket);
      @statistic[rcvdPk](title="packets received"; source=rcvdPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
      @statistic[areaRec](title="total Area recorded"; source=TotalAreaRecorded;  record=histogram,vector; interpolationmode=none);
      @statistic[numNodes](title="nodes selected"; source=NumDronesSelected;  record=histogram,vector; interpolationmode=none);
      @statistic[endToEndDelay](title="delay"; source="dataAge(rcvdPk)"; unit=s; record=histogram,vector; interpolationmode=none);
   
  gates:
    input directGate @directIn;
}
