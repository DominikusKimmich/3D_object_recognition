#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
//#include "shape_recognition_1/weingarten_utils_1.h"
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
#include "shape_recognition_1/evaluate_data.h"

using namespace std;

int main (int argc, char** argv)
{
Evaluate_Data evaluate_data;
evaluate_data.analyse_two_folders_weingarten();
//evaluate_data.euclidean_segmentation();
//evaluate_data.view_pcd_enter_dir();
//evaluate_data.view_pcd_file_name();
return (0);
}
