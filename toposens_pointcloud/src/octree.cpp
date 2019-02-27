#include "toposens_pointcloud/octree.h"


Octree::Octree(float max_size, float divs) {
  size = max_size;
  divisions = divs;
  top_node = new Node;
  top_node->x = 0.0;
  top_node->y = 0.0;
  top_node->z = 0.0;

  top_node->max_vol = 0.0;

  top_node->v = size;
  top_node->leaf = false;
  top_node->occupied = true;
  for (int i = 0; i < 8; i++) {
    top_node->children[i] = NULL;
  }
}


void Octree::addPoint(toposens_msgs::TsPoint new_point, struct Node *current_node) {
  // set top_node to current_node for first recursive function call
  if (current_node == NULL) {
    current_node = top_node;
  }
  current_node->occupied = true; // current_node is occupied

  if (new_point.intensity > current_node->max_vol) {
    current_node->max_vol = new_point.intensity;
  }

  // if current_node is not a leaf of the octree:
  if (!current_node->leaf) {
    // add children to current_node if they have not been added yet
    if (current_node->children[0] == NULL) {
      createChildren(current_node);
    }
    else {
      //ROS_INFO("%f",current_node->children[0]->x);
    }
    // for all children of current_node
    for (int i = 0; i < 8; i++) {
      // recursive call to this function to all of the nodes children which are affected by the new point
      if (isInCell(new_point, current_node->children[i])) {
        addPoint(new_point, current_node->children[i]);
      }
      if (current_node->children[i]->fully_occupied == 0b11111111) {
        current_node->fully_occupied |= 1 << i;
      }
    }

    // if all children are occupied: delelte all child nodes and make current node a leaf
    /*if (current_node->fully_occupied == 0b11111111) {
      deleteChildren(current_node);
      current_node->leaf = true;
    }*/

  }
  // if current node is a leaf: set it to fully occupied
  else {
    current_node->fully_occupied = 0b11111111;
  }
}

void Octree::deleteChildren(struct Node *current_node) {
  for (int i = 0; i < 8; i++) {
    delete(current_node->children[i]);
    // evtl. müssen pointer auf NULL gesetzt werden???
  }
}

void Octree::createChildren(struct Node *current_node) {
  struct NewCoords new_coords;
  new_coords.x_neg = current_node->x - (current_node->v/2);
  new_coords.x_pos = current_node->x + (current_node->v/2);
  new_coords.y_neg = current_node->y - (current_node->v/2);
  new_coords.y_pos = current_node->y + (current_node->v/2);
  new_coords.z_neg = current_node->z - (current_node->v/2);
  new_coords.z_pos = current_node->z + (current_node->v/2);
  new_coords.v = current_node->v/2;

  for (int i = 0; i < 8; i++) {
    current_node->children[i] = new Node;
    current_node->children[i]->leaf = false;
    for (int j = 0; j < 8; j++) {
      current_node->children[i]->children[j] = NULL;
    }
    current_node->children[i]->v = new_coords.v;
    current_node->children[i]->max_vol = 0.0;
  }

  current_node->children[0]->x = new_coords.x_neg;
  current_node->children[0]->y = new_coords.y_neg;
  current_node->children[0]->z = new_coords.z_neg;

  current_node->children[1]->x = new_coords.x_neg;
  current_node->children[1]->y = new_coords.y_neg;
  current_node->children[1]->z = new_coords.z_pos;

  current_node->children[2]->x = new_coords.x_neg;
  current_node->children[2]->y = new_coords.y_pos;
  current_node->children[2]->z = new_coords.z_neg;

  current_node->children[3]->x = new_coords.x_neg;
  current_node->children[3]->y = new_coords.y_pos;
  current_node->children[3]->z = new_coords.z_pos;

  current_node->children[4]->x = new_coords.x_pos;
  current_node->children[4]->y = new_coords.y_neg;
  current_node->children[4]->z = new_coords.z_neg;

  current_node->children[5]->x = new_coords.x_pos;
  current_node->children[5]->y = new_coords.y_neg;
  current_node->children[5]->z = new_coords.z_pos;

  current_node->children[6]->x = new_coords.x_pos;
  current_node->children[6]->y = new_coords.y_pos;
  current_node->children[6]->z = new_coords.z_neg;

  current_node->children[7]->x = new_coords.x_pos;
  current_node->children[7]->y = new_coords.y_pos;
  current_node->children[7]->z = new_coords.z_pos;

  if (new_coords.v <= size/pow(2,divisions)) {
    for (int i = 0; i < 8; i++) {
      current_node->children[i]->leaf = true;
    }
  }
}

bool Octree::isInCell(toposens_msgs::TsPoint point, struct Node *node) {
  if (((point.location.x) < (node->x + (node->v))) &&  ((point.location.x) > (node->x - (node->v)))
    && ((point.location.y) < (node->y + (node->v))) &&  ((point.location.y) > (node->y - (node->v)))
    && ((point.location.z) < (node->z + (node->v))) &&  ((point.location.z) > (node->z - (node->v)))) {
      return true;
  }
  else {
    return false;
  }
}

int Octree::getMarkers(visualization_msgs::MarkerArray *marker_array, int id, struct Node *current_node, std::string frame_id) {

  // set top_node to current_node for first recursive function call
  if (current_node == NULL) {
    current_node = top_node;
  }
  // if current_node is not a leaf of the octree:
  if (!current_node->leaf) {
    if (current_node->children[0] != NULL) {
      for (int i = 0; i < 8; i++) {
        if (current_node->children[i]->occupied) {
          id = getMarkers(marker_array, id, current_node->children[i], frame_id);
        }
      }
    }
  }
  else {
  //  ROS_INFO_STREAM(current_node->max_vol);

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = current_node->x;
    marker.pose.position.y = current_node->y;
    marker.pose.position.z = current_node->z;
    marker.scale.x = current_node->v*2;
    marker.scale.y = current_node->v*2;
    marker.scale.z = current_node->v*2;
  //  marker.color = colorRainbow(((1.0f/(zMax-zMin))*(current_node->z)) - (zMin/(zMax-zMin)));
  //  marker.color.a = ((1.0f/(vMax-vMin))*(current_node->max_vol)) - (vMin/(vMax-vMin));
  //  marker.lifetime = ros::Duration();
    if (current_node->max_vol > 0) marker_array->markers.push_back(marker);
  }
  return id;
}

std_msgs::ColorRGBA Octree::colorRainbow(float i) {
  std_msgs::ColorRGBA color;
  color.a = 1.0f;
  if (i <= 0.25) {
    color.r = 0.0f;
    color.g = i*4.0f;
    color.b = 1.0f;
  }
  else if (i <= 0.5) {
    color.r = 0.0f;
    color.g = 1.0f;
    color.b = 1.0f - ((i-0.25f) * 4.0f);
  }
  else if (i <= 0.75) {
    color.r = (i-0.5f) * 4;
    color.g = 1.0f;
    color.b = 0.0f;
  }
  else {
    color.r = 1.0f;
    color.g = 1.0f - ((i-0.75f) * 4.0f);
    color.b = 0.0f;
  }
  return color;
}
