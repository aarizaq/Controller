/*
 * GSO.h
 *
 *  Created on: Mar 3, 2020
 *      Author: alfonso
 */

#ifndef GSO_H_
#define GSO_H_

#include <vector>
#include <algorithm>
#include <math.h>
#include "inet/common/geometry/common/Coord.h"
#include "Controller.h"

using namespace omnetpp;
using namespace inet;

class GSO {

//    #define r 125.0
    #define rho 0.4
    #define gama 0.6
    #define beta 0.08
    #define s 1
    #define d 2
    #define nd 5
    #define W 3

    Controller::ListNodes *listNodes;

    typedef std::vector<int> IntVector;
    typedef std::vector<double> DoubleVector;
    IntVector Ld;

    std::vector<IntVector> N;
    IntVector Na;
    std::vector<Coord> X;
    DoubleVector Lc;
    DoubleVector Rd;
    DoubleVector Fitness;
    double r = 125;
  //  const double Sol[2] = { -M_PI / 2, M_PI / 2 };



//static float s = 0.03;
//static double X[n][d], Lc[n], Rd[n], P[n][n], Sol[] = { -PI / 2, PI / 2 }, Fitness[n];
//{-0.5268, 0.5268}; //{-10, 5.24};
    void UpdateLuciferin();
    void FindNeighbors();
    void SelectLeader();

/*
void Move() {
    for (int i = 0; i < n; i++) {
        if (Ld[i] != i) {
            int flag = 0;
            double temp[d];

            double dis = Distance(i, Ld[i]);
            for (int j = 0; j < d; j++) {
                temp[j] = X[i][j] + s * (X[Ld[i]][j] - X[i][j]) / dis;
                if (fabs(temp[j]) > W) {
                    flag = 1;
                    break;
                }
            }
            if (flag == 0) for (int j = 0; j < d; j++) X[i][j] = temp[j];
        }
    }
}
*/
    void UpdateNeighborhood();


public:
    GSO(Controller::ListNodes *, double);
    virtual ~GSO();
    virtual void run(){
        UpdateLuciferin();
        FindNeighbors();
        SelectLeader();
    }
    std::vector <L3Address> getLeaders();
};

#endif /* GSO_H_ */
