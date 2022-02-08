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

package simulations;

// numOfHosts: 5

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.inet.WirelessHost;
import inet.node.wireless.AccessPoint;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.physicallayer.wireless.unitdisk.UnitDiskRadioMedium;
import src.Controller;
import src.UdpVideoSend;
import inet.node.inet.AdhocHost;

network ControllerOneArea
{
    parameters:
        int numOfHosts;

    submodules:
        radioMedium: UnitDiskRadioMedium {
            parameters:
                @display("p=69.324,189.66;is=s");
        }
        //radioMedium: Ieee80211ScalarRadioMedium {
        //    parameters:
        //        @display("p=100,200;is=s");
        //}
        configurator: Ipv4NetworkConfigurator {
            @display("p=140,50");
        }
        controller: Controller {
            @display("p=35,34;r=,,#707070");
        }
        
        host[numOfHosts]: AdhocHost {
            @display("r=,,#707070");
        }

}


network ControllerFourArea
{
    parameters:
        int numOfHosts;
        int numSubArea = int(numOfHosts/4);

    submodules:
        radioMedium: UnitDiskRadioMedium {
            parameters:
                @display("p=69.324,189.66;is=s");
        }
        //radioMedium: Ieee80211ScalarRadioMedium {
        //    parameters:
        //        @display("p=100,200;is=s");
        //}
        configurator: Ipv4NetworkConfigurator {
            @display("p=140,50");
        }
        controller: Controller {
            @display("p=35,34;r=,,#707070");
        }
        host1[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host2[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host3[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host4[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
}




network ControllerSixteenArea
{
    parameters:
        int numOfHosts;
        int numSubArea = int(numOfHosts/16);
    submodules:
        radioMedium: UnitDiskRadioMedium {
            parameters:
                @display("p=69.324,189.66;is=s");
        }
        //radioMedium: Ieee80211ScalarRadioMedium {
        //    parameters:
        //        @display("p=100,200;is=s");
        //}
        configurator: Ipv4NetworkConfigurator {
            @display("p=140,50");
        }
        controller: Controller {
            @display("p=35,34;r=,,#707070");
        }        
        host1[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host2[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host3[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host4[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host5[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host6[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host7[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host8[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host9[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        
        host10[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host11[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host12[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host13[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host14[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }

        host15[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        host16[numSubArea]: AdhocHost {
            @display("r=,,#707070");
        }
        


}

