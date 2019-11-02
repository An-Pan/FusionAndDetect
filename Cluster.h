#ifndef OBJECT_DETECT_CLUSTER_H_HO
#define OBJECT_DETECT_CLUSTER_H_HO
#include <vector>
#include <stack>
#include <algorithm>
#include "utils.h"

using namespace std;


		enum PointType
		{
			PT_Unknown = 0,
			PT_Noise,
			PT_Border,
			PT_Core
		};

		class DbscanPoint {
		public:
			int		index;
			int		cluster;
			int		ptType;
			int		pts;			//points in MinPts   
			int		visited;
			float	x;
			float	y;
			std::vector<int> corepts;
            DbscanPoint(float a, float b, int c) :index(c), cluster(c), ptType(PT_Noise), visited(0), pts(0), x(a), y(b) {};
		};

		void dbscan(std::vector<DbscanPoint>& dataset, float Eps, int MinPts);






#endif