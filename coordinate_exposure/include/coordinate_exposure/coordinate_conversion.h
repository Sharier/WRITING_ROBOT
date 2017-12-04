#ifndef COORDINATE_EXPOSURE_CONVERSION_H
#define COORDINATE_EXPOSURE_CONVERSION_H

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <coordinate_exposure/coordinate.h>
#include <coordinate_exposure/coordinate_set.h>
#include <coordinate_exposure/coordinate_sets.h>

// function to convert msg format coordinate to point_ <double>
void decodeCoordinate(const coordinate_exposure::coordinate& input, cv::Point2d& output);

// function to convert from point_ <double> to msg_format coordinate
void codeCoordinate(const cv::Point2d& input, coordinate_exposure::coordinate& output);

// function to convert msg format vector_of_coordinates to vector <point_ <double>>
void decodeCoordinateSet(const coordinate_exposure::coordinate_set& input, std::vector<cv::Point2d>& output);

// function to convert vector <point_ <double>> to msg format vector_of_coordinates
void codeCoordinateSet(const std::vector<cv::Point2d>& input, coordinate_exposure::coordinate_set& output);

// function to convert msg format vector_of_vector_of_coordinates to vector <vector <point_ <double>>>
void decodeCoordinateSets(const coordinate_exposure::coordinate_sets& input, std::vector<std::vector<cv::Point2d> >& output);

// function to convert vector <vector <point_ <double>>> to msg-format vector_of_vector_of_coordinates
void codeCoordinateSets(const std::vector<std::vector<cv::Point2d> >& input, coordinate_exposure::coordinate_sets& output);

#endif // COORDINATE_EXPOSURE_CONVERSION_H
