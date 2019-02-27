#ifndef OCTREE_H
#define OCTREE_H


#include <ros/ros.h>
#include <toposens_msgs/TsPoint.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



struct Node {
  float x;                  // x-value of middle of grid cell
  float y;                  // y-value of middle of grid cell
  float z;                  // z-value of middle of grid cell
  float max_vol;            // maximum volume of all points found in that cell
  float v;                  // volume of grid cell
  bool leaf;                // is node a leaf?
  bool occupied;            // is node occupied?
  char fully_occupied;      // binary code for occupation of children nodes
  struct Node *children[8]; // pointers to all 8 child nodes
};


struct NewCoords {
  float x_neg;
  float x_pos;
  float y_neg;
  float y_pos;
  float z_neg;
  float z_pos;
  float v;
};


class Octree {

protected:
  struct Node *top_node;
  bool isInCell(toposens_msgs::TsPoint point, struct Node *node);
  void createChildren(struct Node *current_node);
  void deleteChildren(struct Node *current_node);
  std_msgs::ColorRGBA colorRainbow(float i);

  float size;
  float divisions;

public:
  Octree(float max_size, float divs);
  void addPoint(toposens_msgs::TsPoint new_point, struct Node *current_node);
  int getMarkers(visualization_msgs::MarkerArray *marker_array, int id, struct Node *current_node, std::string frame_id);
};


#endif
