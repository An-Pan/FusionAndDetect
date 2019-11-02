#include "Cluster.h"



		// DBSCAN Cluster Algorithm [7/31/2017 panan]
		void dbscan(std::vector<DbscanPoint>& dataset, float Eps, int MinPts)
		{
			int len = dataset.size();
			//calculate pts

			for (int i = 0; i < len; i++) {
				for (int j = i + 1; j < len; j++) {
					if (NYNU::math::euclidean_dis(dataset[i].x, dataset[i].y, dataset[j].x, dataset[j].y) < Eps)
						dataset[i].pts++;
					dataset[j].pts++;
				}
			}
			//core point 

			std::vector<DbscanPoint> corePoint;
			for (int i = 0; i < len; i++) {
				if (dataset[i].pts >= MinPts) {
					dataset[i].ptType = PT_Core;
					corePoint.push_back(dataset[i]);
				}
			}

			//joint core point  
			for (int i = 0; i < corePoint.size(); i++) {
				for (int j = i + 1; j < corePoint.size(); j++) {
					if (NYNU::math::euclidean_dis(corePoint[i].x, corePoint[i].y, corePoint[j].x, corePoint[j].y) < Eps) {
						corePoint[i].corepts.push_back(j);
						corePoint[j].corepts.push_back(i);
					}
				}
			}
			for (int i = 0; i < corePoint.size(); i++) {
				std::stack<DbscanPoint*> ps;
				if (corePoint[i].visited == 1) continue;
				ps.push(&corePoint[i]);
				DbscanPoint *v;
				while (!ps.empty()) {
					v = ps.top();
					v->visited = 1;
					ps.pop();
					for (int j = 0; j < v->corepts.size(); j++) {
						if (corePoint[v->corepts[j]].visited == 1) continue;
						corePoint[v->corepts[j]].cluster = corePoint[i].cluster;
						corePoint[v->corepts[j]].visited = 1;
						ps.push(&corePoint[v->corepts[j]]);
					}
				}
			}

			//border point,joint border point to core point  
			for (int i = 0; i < len; i++) {
				if (dataset[i].ptType == PT_Core) continue;
				for (int j = 0; j < corePoint.size(); j++) {
					if (NYNU::math::euclidean_dis(dataset[i].x, dataset[i].y, corePoint[j].x, corePoint[j].y) < Eps) {
						dataset[i].ptType = PT_Border;
						dataset[i].cluster = corePoint[j].cluster;
						break;
					}
				}
			}
			//cout << "output" << endl;
			dataset = corePoint;
			//output  
			// 		fstream clustering;
			// 		clustering.open("clustering.txt", ios::out);
			// 		for (int i = 0; i < len; i++) {
			// 			if (dataset[i].pointType == 2)
			// 				clustering << dataset[i].x << "," << dataset[i].y << "," << dataset[i].cluster << "\n";
			// 		}
			// 		for (int i = 0; i < corePoint.size(); i++) {
			// 			clustering << corePoint[i].x << "," << corePoint[i].y << "," << corePoint[i].cluster << "\n";
			// 		}
			// 		clustering.close();
		}



	
