#ifndef __AhpTopsis_H
#define __AhpTopsis_H

#include <algorithm>
#include <map>
#include <vector>
#include <cmath>
#include "Controller.h"

using namespace std;

//static const int wlength = 4*3+1; // 13
static const int wlength = 4; // 13

enum AhpWeightComputeType
{
	Satrii,
	Sotooni,
	Hesabi,
	Hendesi 
};

class AhpWeightCompute
{
private:
	double w[wlength][wlength];

	void runHendesi(double weight[], int length) {
		for (int i = 0; i < length; i++) 
			weight[i] = 1;
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < length; j++) {
				weight[i]*=w[i][j];
			}
		}
		for (int i = 0; i < length; i++) {
			weight[i]=pow(weight[i],(double)1/3);
		}
		double normalizer=0;
		for (int i = 0; i < length; i++) {
			normalizer+=weight[i];
		}
		for (int i = 0; i < length; i++) {
			weight[i]=(double)weight[i]/normalizer;
		}
	}
	void runSotooni(double weight[], int length) {
		for (int i = 0; i < length; i++) 
			weight[i] = 0;
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < length; j++) {
				weight[i]+=w[j][i];
			}
		}
		for (int i = 0; i < length; i++) {
			weight[i]=(double)1/weight[i];
		}
		double normalizer=0;
		for (int i = 0; i < length; i++) {
			normalizer+=weight[i];
		}
		for (int i = 0; i < length; i++) {
			weight[i]=(double)weight[i]/normalizer;
		}
	}
	void runSatrii(double weight[], int length) {
		for (int i = 0; i < length; i++) 
			weight[i] = 0;
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < length; j++) {
				weight[i]+=w[i][j];
			}
		}
		double normalizer=0;
		for (int i = 0; i < length; i++) {
			normalizer+=weight[i];
		}
		for (int i = 0; i < length; i++) {
			weight[i]=(double)weight[i]/normalizer;
		}
	}
	void runHesabi(double weight[], int length) {
		double c[wlength];
		for (int i = 0; i < length; i++) {
			double k = 0;
			for (int j = 0; j < length; j++) {
				k += w[j][i];
			}
			c[i] = k;
		}
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < length; j++) {
				w[i][j] = (double) w[i][j] / c[j];
			}
		}
		for (int i = 0; i < length; i++) {
			double k = 0;
			for (int j = 0; j < length; j++) {
				k += w[i][j];
			}
			weight[i] = (double) k / wlength;
		}
	}
public:	
	AhpWeightCompute(double q[wlength][wlength]) {
		for (int i = 0; i < wlength; i++) {
			for (int j = 0; j < wlength; j++) {
				w[i][j]=q[i][j];
			}
		}
	}
	void run(double weight[], int length, 
		AhpWeightComputeType type = AhpWeightComputeType::Hendesi) {
		switch(type){
			case AhpWeightComputeType::Sotooni:
				return runSotooni(weight, length);
			case AhpWeightComputeType::Satrii:
				return runSatrii(weight, length);
			case AhpWeightComputeType::Hesabi:
				return runHesabi(weight, length);
			default:
				return runHendesi(weight, length);
		}
	}
};

void Topsis(double q[wlength][wlength], 
	vector<vector<double>> list,
	vector<pair<int, double>>& sortedlist) 
{
	int size = list.size();
	AhpWeightCompute ahp(q);
	double weight[wlength];
	ahp.run(weight,wlength,AhpWeightComputeType::Hendesi);
	double normalize[wlength];
	for (int i = 0; i < wlength; i++) {
		normalize[i] = 0;
		for (int k = 0; k < size; k++) {
			normalize[i] += pow(list[k][i], 2);
		}
		normalize[i] = sqrt(normalize[i]);
	}
	double best[wlength];
	double worse[wlength];
	for (int i = 0; i < wlength; i++) {
		best[i] = DBL_MAX;
		worse[i] = 0;
	}
	for (int i = 0; i < size; i++) {
		for (int k = 0; k < wlength; k++) {
		    if (normalize[k] != 0)
		        list[i][k] = (double) list[i][k] * weight[k] / normalize[k];
            else if (list[i][k] == 0)
                list[i][k] = 0;
            else
                throw cRuntimeError("Topsis tries to normalize by 0");
			if (list[i][k] > worse[k])
				worse[k] = list[i][k];
			if (list[i][k] < best[k])
				best[k] = list[i][k];
		}
	}

    //double *sPlus=new double[size];
    //double *sMinus=new double[size];
	std::vector<double> sPlus;
	std::vector<double> sMinus;
	sPlus.resize(size);
	sMinus.resize(size);
	for (int i = 0; i < size; i++) {
		sPlus[i] = sMinus[i] = 0;
		for (int k = 0; k < wlength; k++) {
			sPlus[i] += pow(list[i][k]- best[k], 2);
			sMinus[i] += pow(list[i][k]- worse[k], 2);
		}
		sPlus[i] = sqrt(sPlus[i]);
		sMinus[i] = sqrt(sMinus[i]);
	}
	for (int i = 0; i < size; i++) {
		sortedlist.push_back(make_pair(i, (double)sMinus[i]/(sPlus[i]+sMinus[i])));
	}
	sort(sortedlist.begin(), sortedlist.end(), 
		[=](std::pair<int, double>& a, std::pair<int, double>& b)
		{
			return a.second < b.second;
		}
	);
//	delete [] sPlus;
//	delete [] sMinus;
}

void Topsis(const std::map<L3Address, Controller::NodeData *>& listNodes,
	vector<pair<int, double>>& sortedlist) 
{
/*		double q[wlength][wlength]={
			//Energy Pos.X Pos.Y Pos.Z Speed.X Speed.Y Speed.Z ...
			{1,6.0/7.0,3.0/2.0,4,5,6,7,8,9,10,11,12,13},
			{1,1,3.0/2.0,4,5,6,7,8,9,10,11,12,13},
			{1,6.0/7.0,1,4,5,6,7,8,9,10,11,12,13},
			{1,6.0/7.0,3.0/2.0,1,5,6,7,8,9,10,11,12,13},
			{1,6.0/7.0,3.0/2.0,4,1,6,7,8,9,10,11,12,13},
			{1,6.0/7.0,3.0/2.0,4,5,1,7,8,9,10,11,12,13},
			{1,6.0/7.0,3.0/2.0,4,5,6,1,8,9,10,11,12,13},
			{1,6.0/7.0,3.0/2.0,4,5,6,7,1,9,10,11,12,13},
			{1,6.0/7.0,3.0/2.0,4,5,6,7,8,1,10,11,12,13},
			{1,6.0/7.0,3.0/2.0,4,5,6,7,8,9,1,11,12,13},
			{1,6.0/7.0,3.0/2.0,4,5,6,7,8,9,10,1,12,13},
			{1,6.0/7.0,3.0/2.0,4,5,6,7,8,9,10,11,1,13},
			{1,6.0/7.0,3.0/2.0,4,5,6,7,8,9,10,11,12,1},
		};
		*/
        double q[wlength][wlength]={
            //Energy Pos.X Pos.Y Pos.Z Speed.X Speed.Y Speed.Z ...
            {1,3,5,7},
            {1.0/3.0,1,2,4},
            {1.0/5.0,1.0/2.0,1,5},
            {1.0/7.0,1.0/4.0,1.0/5.0,1}
        };

		vector<vector<double>> list;

/*		FILE * f = fopen("Val", "w");
		int index = 0;
*/
		for (auto i=listNodes.begin();i!=listNodes.end();++i) {

			vector<double> v;
			v.push_back(i->second->energy);
			v.push_back(i->second->getCoverageRegion());
			v.push_back(i->second->numNeigh);
			v.push_back(1/(i->second->speed.length()+1e-10));

/*
			v.push_back(i->second->position.getX());
			v.push_back(i->second->position.getY());
			v.push_back(i->second->position.getZ());
			v.push_back(i->second->speed.getX());
			v.push_back(i->second->speed.getY());
			v.push_back(i->second->speed.getZ());
			v.push_back(i->second->constraintAreaMin.getX());
			v.push_back(i->second->constraintAreaMin.getY());
			v.push_back(i->second->constraintAreaMin.getZ());
			v.push_back(i->second->constraintAreaMax.getX());
			v.push_back(i->second->constraintAreaMax.getY());
			v.push_back(i->second->constraintAreaMax.getZ());
			*/
/*            v.push_back(1.0);
            v.push_back(1.0);
            v.push_back(1.0);
            v.push_back(1.0);
            v.push_back(1.0);
            v.push_back(1.0);
            v.push_back(1.0);
            v.push_back(1.0);
            v.push_back(1.0);
            v.push_back(1.0);
            v.push_back(1.0);
            v.push_back(1.0);*/
			list.push_back(v);
/*			fprintf(f, "Index %i -  Remaining E = %f Pos x = %f Pos Y = %f Pos Z = %f", index, v[0], v[1],v[2], v[3]);
			fprintf(f, "  Speed X = %f Speed Y = %f Speed =  Z %f", v[4],v[5], v[6]);
			fprintf(f, "  Area Max X = %f Area Max Y = %f Area Max Z = %f", v[7],v[8], v[9]);
			fprintf(f, "  Area Min X = %f Area Min Y = %f Area Min Z = %f \n", v[10],v[11], v[12]);
			index++;
*/
		}
		Topsis(q,list,sortedlist);
	/*	fprintf(f, "  Topsis Results \n") ;
		for (auto elem : sortedlist) {
		    fprintf(f, "Index %i  Order = %f \n", elem.first, elem.second);
		}
		fclose(f);*/

}
#endif // ifndef __AhpTopsis_H
