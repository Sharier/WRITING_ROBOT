#ifndef PAPER_DETECTION_CONVERSION_H
#define PAPER_DETECTION_CONVERSION_H

#include <ros/ros.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <paper_detection/point.h>
#include <paper_detection/point_set.h>
#include <paper_detection/point_sets.h>

#include <paper_detection/point_conversion.h>


//Function to convert msg format point to point


void decodePoint(const paper_detection::point& input, cv::Point& output)
{
  output = cv::Point(input.x, input.y);
}


//Function for converting point to msg format point




void codePoint(const cv::Point& input, paper_detection::point& output)
{
  output.x = input.x;
  output.y = input.y;
  output.header.stamp = ros::Time::now();
}


//Function to convert msg format vector_of_points to vector <point>



void decodePointSet(const paper_detection::point_set& input, std::vector<cv::Point>& output)
{
  cv::Point current_pt;

  for(int p = 0; p < input.data.size(); p++)
  {
    decodePoint(input.data.at(p), current_pt);
    output.push_back(current_pt);
  }
}


//Function to convert vector <point> to msg format vector_of_points


void codePointSet(const std::vector<cv::Point>& input, paper_detection::point_set& output)
{
  paper_detection::point current_pt;

  for(int p = 0; p < input.size(); p++)
  {
    codePoint(input.at(p), current_pt);
    output.data.push_back(current_pt);
    output.header.stamp = ros::Time::now();
  }
}


//Function to convert msg format vector_of_vector_of_points to vector <vector <point>>



void decodePointSets(const paper_detection::point_sets& input, std::vector<std::vector<cv::Point> >& output)
{
  std::vector<cv::Point> current_pts;

  for(int c = 0; c < input.data.size(); c++)
  {
    decodePointSet(input.data.at(c), current_pts);
    output.push_back(current_pts);
    current_pts.clear();
  }
}


//Function to convert vector <vector> to msg format vector_of_vector_of_points



void codePointSets(const std::vector<std::vector<cv::Point> >& input, paper_detection::point_sets& output)
{
  output.data.resize(input.size());

  for(int c = 1; c < input.size(); c++)
  {
    codePointSet(input.at(c), output.data.at(c));
    output.header.stamp = ros::Time::now();
  }
}

#endif // PAPER_DETECTION_CONVERSION_H
