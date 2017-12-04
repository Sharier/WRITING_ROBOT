#ifndef PAPER_DETECTION_CONVERSION_H
#define PAPER_DETECTION_CONVERSION_H

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <paper_detection/point.h>
#include <paper_detection/point_set.h>
#include <paper_detection/point_sets.h>

// function to convert msg format point to point
void decodePoint(const paper_detection::point& input, cv::Point& output);

// function to convert from point to msg format point
void codePoint(const cv::Point& input, paper_detection::point& output);

// function to convert msg format vector_of_points to vector <point>
void decodePointSet(const paper_detection::point_set& input, std::vector<cv::Point>& output);

// function to convert vector <point> to msg format vector_of_points
void codePointSet(const std::vector<cv::Point>& input, paper_detection::point_set& output);

// function to convert msg format vector_of_vector_of_points to vector <vector <point>>
void decodePointSets(const paper_detection::point_sets& input, std::vector<cv::Point>& output);

// function to convert vector <vector <point>> to msg-format vector_of_vector_of_points
void codePointSets(const std::vector<cv::Point>& input, paper_detection::point_sets& output);

#endif // PAPER_DETECTION_CONVERSION_H
