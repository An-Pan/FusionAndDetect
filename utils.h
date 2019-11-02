//
// Created by panan on 18-4-18.
//

#ifndef NYNU_COMMON_UTILS_H__
#define NYNU_COMMON_UTILS_H__

// STL
#include <iostream>
#include <vector>

// Boost
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

// OpenCV
#include <opencv2/opencv.hpp>


using namespace std;
using namespace boost;


namespace NYNU{

	namespace file
	{
		static const vector<string>& scan_files(const string& rootPath, vector<string>& container = *(new vector<string>())) {
			namespace fs = boost::filesystem;
			fs::path fullpath(rootPath);
			vector<string> &ret = container;

			if (!fs::exists(fullpath)) { return ret; }
			fs::directory_iterator end_iter;
			for (fs::directory_iterator iter(fullpath); iter != end_iter; iter++) {
				try {
					if (fs::is_directory(*iter)) {
						//ret.push_back(iter->path().string());
						//scan_files(iter->path().string(), ret);
						continue;
					}
					else {
						ret.push_back(iter->path().string());
					}
				}
				catch (const std::exception & ex) {
					std::cerr << ex.what() << std::endl;
					continue;
				}
			}
			return ret;
		}

		static const vector<string>& scan_files_recursion(const string& rootPath, vector<string>& container = *(new vector<string>())) {
			namespace fs = boost::filesystem;
			fs::path fullpath(rootPath);
			vector<string> &ret = container;

			if (!fs::exists(fullpath)) { return ret; }
			fs::directory_iterator end_iter;
			for (fs::directory_iterator iter(fullpath); iter != end_iter; iter++) {
				try {
					if (fs::is_directory(*iter)) {
						ret.push_back(iter->path().string());
						scan_files_recursion(iter->path().string(), ret);
					}
					else {
						ret.push_back(iter->path().string());
					}
				}
				catch (const std::exception & ex) {
					std::cerr << ex.what() << std::endl;
					continue;
				}
			}
			return ret;
		}

		static const vector<string>& scan_directory_regex(const string& rootPath, boost::regex name_regx, vector<string>& container = *(new vector<string>())) {
			namespace fs = boost::filesystem;
			fs::path fullpath(rootPath);
			vector<string> &ret = container;

			if (!fs::exists(fullpath)) { return ret; }
			fs::directory_iterator end_iter;
			for (fs::directory_iterator iter(fullpath); iter != end_iter; iter++) {
				try {
					if (fs::is_directory(*iter)) {
						if (boost::regex_match(iter->path().stem().string(), name_regx)) {
							ret.push_back(iter->path().string());
						}
						else {
							scan_directory_regex(iter->path().string(), name_regx, ret);
						}
						continue;
					}
				}
				catch (const std::exception & ex) {
					std::cerr << ex.what() << std::endl;
					continue;
				}
			}
			return ret;
		}

		static bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat)
		{
			if (!ofs.is_open())
				return false;

			if (out_mat.empty()) {
				int s = 0;
				ofs.write((const char*)(&s), sizeof(int));
				return true;
			}

			int type = out_mat.type();
			ofs.write((const char*)(&out_mat.rows), sizeof(int));
			ofs.write((const char*)(&out_mat.cols), sizeof(int));
			ofs.write((const char*)(&type), sizeof(int));
			ofs.write((const char*)(out_mat.data), out_mat.elemSize()*out_mat.total());


			return true;
		}

		static bool saveMatBinary(const char* filename, const cv::Mat& output)
		{
			std::ofstream ofs(filename, std::ios::binary);
			return writeMatBinary(ofs, output);
		}

		static bool readMatBinary(std::ifstream& ifs, cv::Mat& in_mat)
		{
			if (!ifs.is_open())
				return false;

			int rows = 0;
			int cols = 0;
			int type = -1;

			ifs.read((char*)(&rows), sizeof(int));
			ifs.read((char*)(&cols), sizeof(int));
			ifs.read((char*)(&type), sizeof(int));
			if (rows == 0 || cols == 0)
				return false;

			in_mat.release();
			in_mat.create(rows, cols, type);
			ifs.read((char*)(in_mat.data), in_mat.elemSize()*in_mat.total());



			return true;
		}

		static bool loadMatBinary(const char*  filename, cv::Mat& output)
		{
			std::ifstream ifs(filename, std::ios::binary);
			return readMatBinary(ifs, output);
		}



	}

	namespace math
	{

		static void matrix_vector(double coord[3], double R[3][3])
		{
			double b[3];
			b[0] = coord[0];
			b[1] = coord[1];
			b[2] = coord[2];
			coord[0] = R[0][0] * b[0] + R[0][1] * b[1] + R[0][2] * b[2];
			coord[1] = R[1][0] * b[0] + R[1][1] * b[1] + R[1][2] * b[2];
			coord[2] = R[2][0] * b[0] + R[2][1] * b[1] + R[2][2] * b[2];
		}

		static void matrix_mul_1(double coord[3], double Rx[3][3], double Ry[3][3], double Rz[3][3], double move[3])
		{
			double b[3];
			b[0] = coord[0] - move[0];
			b[1] = coord[1] - move[1];
			b[2] = coord[2] - move[2];

			matrix_vector(b, Rx);
			matrix_vector(b, Ry);
			matrix_vector(b, Rz);

			coord[0] = b[0];
			coord[1] = b[1];
			coord[2] = b[2];

		}



		static bool pnpoly(std::vector<cv::Point2f>& edge, float x, float y)
		{
			int i, j;
			j = edge.size() - 1;
			int res = 0;
			for (i = 0; i < edge.size(); i++)
			{
				if ((edge[i].y < y && edge[j].y >= y || edge[j].y < y && edge[i].y >= y) && (edge[i].x <= x || edge[j].x <= x))
				{
		
					res ^= ((edge[i].x + (y - edge[i].y) / (edge[j].y - edge[i].y)*(edge[j].x - edge[i].x)) < x);
				}
				j = i;
			}
			return res;
		}

		static bool pnpoly(std::vector<cv::Point2d>& edge, double x, double y)
		{
			int i, j;
			j = edge.size() - 1;
			int res = 0;
			for (i = 0; i < edge.size(); i++)
			{
				if ((edge[i].y < y && edge[j].y >= y || edge[j].y < y && edge[i].y >= y) && (edge[i].x <= x || edge[j].x <= x))
				{
					res ^= ((edge[i].x + (y - edge[i].y) / (edge[j].y - edge[i].y)*(edge[j].x - edge[i].x)) < x);
				}
				j = i;
			}
			return res;
		}


		static bool point_in_rect(const cv::Point& pt, cv::Rect& rt)
		{
			return (pt.x > rt.x) && (pt.x<rt.x + rt.width) && (pt.y>rt.y) && (pt.y < rt.y + rt.height);
		}

		template<typename T>
		bool iszero(T num)
		{
			return abs(num) < 0.0001;
		}

		template<typename T>
		T manhattan_dis(T x1, T y1, T x2, T y2)
		{
			return abs(x1 - x2) + abs(y1 - y2);
		}

		template<typename T>
		T euclidean_dis(T x1, T y1, T x2, T y2)
		{
			return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
		}

		template<typename T>
		T cosine_dis(T x1, T y1, T x2, T y2)
		{
			/*
			TODO: calc cosine distance
			*/
			return 0;
		}


		template<typename T>
		T pt2LineDis(T linePtX1, T linePtY1, T linePtX2, T linePtY2, T distPtX, T distPtY)
		{
			T rhomusArea = (linePtX2 - linePtX1)*(distPtY - linePtY1) - (linePtY2 - linePtY1)*(distPtX - linePtX1);
			rhomusArea = abs(rhomusArea);
			T lineLength = sqrt((linePtX2 - linePtX1)*(linePtX2 - linePtX1) + (linePtY2 - linePtY1)*(linePtY2 - linePtY1));
			return rhomusArea / lineLength;
		}


		static int calc_overlap(cv::Rect rt1, cv::Rect rt2)
		{
			int xl1 = rt1.x;
			int xr1 = rt1.x + rt1.width;
			int yt1 = rt1.y;
			int yb1 = rt1.y + rt1.height;

			int xl2 = rt2.x;
			int xr2 = rt2.x + rt2.width;
			int yt2 = rt2.y;
			int yb2 = rt2.y + rt2.height;

			int xmin = max(xl1, xl2);
			int ymin = max(yt1, yt2);
			int xmax = min(xr1, xr2);
			int ymax = min(yb1, yb2);
			int width = xmax - xmin;
			int height = ymax - ymin;
			if (width < 0 || height < 0) {
				return 0;
			}
			else {
				return width*height;
			}

		}

		template<typename T>
		T calc_overlap(T x_min_1, T y_min_1, T x_max_1, T y_max_1, T x_min_2, T y_min_2, T x_max_2, T y_max_2)
		{
			T xl1 = x_min_1;
			T xr1 = x_max_1;
			T yt1 = y_min_1;
			T yb1 = y_max_1;

			T xl2 = x_min_2;
			T xr2 = x_max_2;
			T yt2 = y_min_2;
			T yb2 = y_max_2;

			T xmin = max(xl1, xl2);
			T ymin = max(yb1, yb2);
			T xmax = min(xr1, xr2);
			T ymax = min(yt1, yt2);
			T width = xmax - xmin;
			T height = ymax - ymin;
			if (width < 0 || height < 0) {
				return static_cast<T>(0);
			}
			else {
				return width*height;
			}


		}

		template<typename T>
		bool is_rect_intersect(T x01, T x02, T y01, T y02,
			T x11, T x12, T y11, T y12)
		{
			T zx = abs(x01 + x02 - x11 - x12);
			T x = abs(x01 - x02) + abs(x11 - x12);
			T zy = abs(y01 + y02 - y11 - y12);
			T y = abs(y01 - y02) + abs(y11 - y12);
			if (zx <= x && zy <= y)
				return true;
			else
				return false;
		}

	}

	namespace tools
	{
		static cv::Vec3b GreyToPseudocolor(int val)
		{
			int r, g, b;

			//red
			if (val < 128)
			{
				r = 0;
			}
			else if (val < 192)
			{
				r = 255 / 64 * (val - 128);
			}
			else
			{
				r = 255;
			}

			//green
			if (val < 64)
			{
				g = 255 / 64 * val;
			}
			else if (val < 192)
			{
				g = 255;
			}
			else
			{
				g = -255 / 63 * (val - 192) + 255;
			}

			//blue
			if (val < 64)
			{
				b = 255;
			}
			else if (val < 128)
			{
				b = -255 / 63 * (val - 192) + 255;
			}
			else
			{
				b = 0;
			}
			cv::Vec3b rgb;
			rgb[0] = b;
			rgb[1] = g;
			rgb[2] = r;
			return rgb;
		};


		template <typename T>
		std::vector<size_t> sort_indexes(const std::vector<T> &v,bool ascending = false) {

			// initialize original index locations
			std::vector< size_t>  idx(v.size());
			for (size_t i = 0; i < idx.size(); ++i) idx[i] = i;

			// sort indexes based on comparing values in v
			if (ascending) {
				sort(idx.begin(), idx.end(),
					[&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });
			}
			else {
				sort(idx.begin(), idx.end(),
					[&v](size_t i1, size_t i2) {return v[i1] > v[i2]; });
			}


			return idx;
		}


		static void drawDashRectangle(cv::Mat& img, cv::Rect rt, cv::Scalar color, int thinkness)
		{

			// 
			// 			for (int i = rt.x; i < rt.x + rt.width - 1; i += 4) {
			// 				cv::line(img, cv::Point(i, rt.y), cv::Point(i + 1, rt.y), color, thinkness);
			// 				cv::line(img, cv::Point(i, rt.y + rt.height), cv::Point(i + 1, rt.y), color, thinkness);
			// 			}
			// 
			// 
			// 			for (int i = rt.y; i < rt.y + rt.height - 1; i += 4) {
			// 
			// 				cv::line(img, cv::Point(rt.x, i), cv::Point(rt.x, i + 1), color, thinkness);
			// 				cv::line(img, cv::Point(rt.x + rt.width, i), cv::Point(rt.x + rt.width, i + 1), color, thinkness);
			// 
			// 
			// 
			// 			}

			cv::line(img, cv::Point(rt.x, rt.y), cv::Point(rt.x + rt.width, rt.y + rt.height), color, thinkness);

			cv::line(img, cv::Point(rt.x + rt.width, rt.y), cv::Point(rt.x, rt.y + rt.height), color, thinkness);


		}

	}


	namespace stringutils
	{
		static void removeCharacters( std::string & str, const string& charsToRemove )
		{
			for ( const auto& charToRemove : charsToRemove )
				str.erase( remove( str.begin(), str.end(), charToRemove ), str.end() );
		}
	}

	namespace image
	{
		template <typename S, typename R>
		bool ProtectBox( const S & imgSize, R & rect )
		{
			bool wasTampered = false;
			
			if ( rect.x < 0 )
			{
				rect.width += rect.x;
				rect.x = 0;
				wasTampered = true;
			}

			if ( rect.x > imgSize.width )
			{
				rect.x = imgSize.width;
				wasTampered = true;
			}

			if ( rect.y < 0 )
			{
				rect.height += rect.y;
				rect.y = 0;
				wasTampered = true;
			}

			if ( rect.y > imgSize.height )
			{
				rect.y = imgSize.height;
				wasTampered = true;
			}

			if ( rect.x + rect.width > imgSize.width )
			{
				rect.width -= ( rect.x + rect.width ) - imgSize.width;
				wasTampered = true;
			}

			if ( rect.y + rect.height > imgSize.height )
			{
				rect.height -= ( rect.y + rect.height ) - imgSize.height;
				wasTampered = true;
			}

			if ( rect.width < 0 )
			{
				rect.width = 0;
				wasTampered = true;
			}
			
			if ( rect.height < 0 )
			{
				rect.height = 0;
				wasTampered = true;
			}
			
			return wasTampered;
		};
	}



}


#endif //ERLANG_UTILS_H_H
