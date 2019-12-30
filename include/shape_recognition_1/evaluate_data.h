#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "shape_recognition_1/weingarten_utils_1.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <fstream>
#include <stdio.h>
#include "shape_recognition_1/quadric_utils.h"

class Evaluate_Data
{
public:

Evaluate_Data();
~Evaluate_Data();
int getdir (std::string dir, std::vector<std::string> &files);
void analyse_two_folders_quadric(void);
void analyse_two_folders_weingarten(void);
void euclidean_segmentation(void);
//void view_pcd_files(void);
void view_pcd_file_name(void);
void view_pcd_enter_dir(void);
};
