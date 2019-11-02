#ifndef SYSU_COMMON_VISUAL_OBJECT_H
#define SYSU_COMMON_VISUAL_OBJECT_H

#include <memory>
#include <vector>

#include "opencv2/opencv.hpp"


#include "Eigen/Core"


struct alignas(16) VisualObject 
{
	// Per-frame object id, assigned from detection
	int id = 0;
	// Confidence of objectness, ranging as [0, 1]
	float score = 0.0f;

	float area = 0.0f;

	float area_rate = 0.0;

	std::string sign_label;

	// [pixel] 2D bounding box
	// upper-left corner: x1, y1
	Eigen::Vector2f upper_left;
	// lower-right corner: x2, y2
	Eigen::Vector2f lower_right;

	int track_id = 0;
	// [second] age of the tracked object
	double track_age = 0.0;
	// [second] the last observed timestamp
	double last_track_timestamp = 0.0;

	double first_detect_timestamp = 0.0;

};


#endif