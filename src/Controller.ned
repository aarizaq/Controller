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
   
  gates:
    input directGate @directIn;
}
