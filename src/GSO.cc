/*
 * GSO.cpp
 *
 *  Created on: Mar 3, 2020
 *      Author: alfonso
 */

#include "GSO.h"

GSO::GSO(Controller::ListNodes *l, double rd) : listNodes(l), r(rd) {
    // TODO Auto-generated constructor stub
    for (auto & elem : *listNodes){
        elem.second->Ld = elem.first;
        elem.second->Rd = r;
    }
}

GSO::~GSO() {
    // TODO Auto-generated destructor stub
}


void GSO::UpdateLuciferin() {
    for (auto &elem : *listNodes) {
        double x = elem.second->position.x;
        double y = elem.second->position.y;

        double J = 3 * std::pow((1 - x), 2) * std::exp(-pow(x, 2) - std::pow((y + 1), 2)) - 10 * (x / 5 - std::pow(x, 3) -
                std::pow(y, 5)) * std::exp(-std::pow(x, 2) - std::pow(y, 2)) - 1 / 3 * std::exp(-std::pow((x + 1), 2) - std::pow(y, 2));

        J = J + std::pow(std::sin(x), 2);
        J = J + 10 + pow(x, 2) - std::cos(2 * M_PI * x);
        J = J + 418.9829 + x * std::sin(sqrt(fabs(x)));

        J = J + std::pow(std::sin(y), 2);
        J = J + 10 + pow(y, 2) - std::cos(2 * M_PI * y);
        J = J + 418.9829 + y *  std::sin(sqrt(fabs(y)));
        elem.second->Lc = (1 - rho) * elem.second->Lc + gama * J;
    }
}

void GSO::FindNeighbors() {
    for (auto elem : *listNodes) {
        elem.second->Neigh.clear();
        for (auto elem2 : *listNodes) {
            if (elem.first == elem2.first)
                continue;
            double dist = elem.second->position.distance(elem2.second->position);
            if ((elem.second->Lc < elem2.second->Lc) && dist < elem.second->Rd) {
                elem.second->Neigh.push_back(elem2.first);
            }
        }
    }
}


void GSO::SelectLeader() {
    for (auto &elem : *listNodes) {
        elem.second->Ld = elem.first;
        elem.second->Fitness = 0;
        for (auto &elem2 : *listNodes) {
            if (elem2 == elem)
                continue;
            auto it = std::find(elem.second->Neigh.begin(), elem.second->Neigh.end(), elem2.first);
            if (it != elem.second->Neigh.end()) {
                if (elem.second->Fitness < elem2.second->Lc + elem2.second->energy) {
                    elem.second->Fitness = elem2.second->Lc + elem2.second->energy;
                    elem.second->Ld = elem2.first;
                }
            }
        }
    }
}


std::vector <L3Address> GSO::getLeaders() {
    std::vector <L3Address> leaders;
    for (const auto &elem : *listNodes) {
        auto it = std::find(leaders.begin(), leaders.end(), elem.second->Ld);
        if (it == leaders.end())
            leaders.push_back(elem.second->Ld);
    }
    return leaders;
}

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

void GSO::UpdateNeighborhood() {
    for (auto &elem : *listNodes)
        elem.second->Rd = std::max(0.0, std::min(r, elem.second->Rd + beta * (nd - elem.second->Neigh.size())));
}



