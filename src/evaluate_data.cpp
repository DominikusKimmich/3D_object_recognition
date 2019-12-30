#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
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

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


Evaluate_Data::Evaluate_Data()
{
}

Evaluate_Data::~Evaluate_Data()
{
}

int Evaluate_Data::getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(std::string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}

void Evaluate_Data::analyse_two_folders_quadric(void)
{
Quadric_Utils quadric_utils;
quadric_utils.set_shell_descriptor_parameters();

std::ofstream myfile ("features.txt");
if (myfile.is_open())
{

	std::string dir;
	dir = std::string("bunny");
	std::vector<std::string> files;
	files = std::vector<std::string>();
	std::string path;

	getdir(dir,files);
	double num_samples_class_1 = 0;
	double num_samples_class_2 = 0;

	for (unsigned int i = 0;i < files.size();i++)
	{
		std::string ein_punkt = std::string(".");
		std::string zwei_punkt = std::string("..");

		if (ein_punkt.compare(files[i]) == 0)
		{
			i = i + 1;
		}

		if (zwei_punkt.compare(files[i]) == 0)
		{
			i = i + 1;
		}

		std::cout << files[i] << std::endl;
		path = dir + '/' + files[i];
		std::cout << "path for first file is: " << path << std::endl;
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *quadric_utils.cloud_) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file \n");
		}
		std::cout << "Loading of pcd file is complete now." << std::endl;
		//std::cout << "size of the loaded point cloud is: " << cloud_->points.size () << std::endl;
		size_t size_of_pc = quadric_utils.cloud_->points.size ();

		//remove NAN points from the cloud
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*quadric_utils.cloud_,*quadric_utils.cloud_, indices);

		quadric_utils.analyse_surface();
		std::cout << " " << quadric_utils.feature_.size() << std::endl;
	}
	num_samples_class_1 = quadric_utils.feature_.size()/((quadric_utils.added_shells_+1)*8 + 2);

		dir = std::string("horse");
		files = std::vector<std::string>();

		getdir(dir,files);

		for (unsigned int i = 0;i < files.size();i++)
				{
					std::string ein_punkt = std::string(".");
					std::string zwei_punkt = std::string("..");

					if (ein_punkt.compare(files[i]) == 0)
					{
						i = i + 1;
					}

					if (zwei_punkt.compare(files[i]) == 0)
					{
						i = i + 1;
					}
					std::cout << files[i] << std::endl;
					path = dir + '/' + files[i];

					std::cout << "path name for second file is: " << path << std::endl;
				}

		for (unsigned int i = 0;i < files.size();i++)
		{
			std::string ein_punkt = std::string(".");
			std::string zwei_punkt = std::string("..");

			if (ein_punkt.compare(files[i]) == 0)
			{
				i = i + 1;
			}

			if (zwei_punkt.compare(files[i]) == 0)
			{
				i = i + 1;
			}
			std::cout << files[i] << std::endl;
			path = dir + '/' + files[i];

			std::cout << "Loading of pcd file begins. Please wait ..." << std::endl;
			std::cout << "path name for second file is: " << path << std::endl;
			if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *quadric_utils.cloud_) == -1) //* load the file
			{
				PCL_ERROR ("Couldn't read file \n");
			}
			std::cout << "Loading of pcd file is complete now." << std::endl;
			//std::cout << "size of the loaded point cloud is: " << cloud_->points.size () << std::endl;
			size_t size_of_pc = quadric_utils.cloud_->points.size ();

			//remove NAN points from the cloud
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*quadric_utils.cloud_,*quadric_utils.cloud_, indices);

			/*
			for(size_t search_point_index = quadric_utils.index_offset_; search_point_index < quadric_utils.cloud_->points.size();)
			{
				quadric_utils.analyse_surface();
				search_point_index = search_point_index + quadric_utils.index_stepsize_;
			}
			*/
			quadric_utils.analyse_surface();
			std::cout << "current length of the features vector is: " << quadric_utils.feature_.size() << std::endl;
		}

		num_samples_class_2 = (quadric_utils.feature_.size()/((quadric_utils.added_shells_+1)*8 + 2))-num_samples_class_1;

		std::cout << "writing results to file begins now. Please wait..." << std::endl;
		// feature values fill rows
		// as many columns as samples taken
		for(double l = 0; l < quadric_utils.feature_.size();)
			{
			for(double k = 0; k < ((quadric_utils.added_shells_+1)*8 + 2);k++)
			{
				myfile << quadric_utils.feature_[l + k] << " ";
			}
			myfile << "\n";
			l = l + ((quadric_utils.added_shells_+1)*8 + 2);
		}
		std::cout << "writing results to file finished now." << std::endl;
		std::cout << "Number of samples taken in class 1 is: " << num_samples_class_1 << std::endl;
		std::cout << "Number of samples taken in class 2 is: " << num_samples_class_2 << std::endl;
}
else
{
	std::cout << "Unable to open file" << std::endl;
}
myfile.close();
}

void Evaluate_Data::analyse_two_folders_weingarten(void)
{
Weingarten_Utils weingarten_utils;
weingarten_utils.set_shell_descriptor_parameters();

std::ofstream myfile ("features.txt");
if (myfile.is_open())
{

	std::string dir;
	dir = std::string("Kanne_blau_predict");
	std::vector<std::string> files;
	files = std::vector<std::string>();
	std::string path;

	getdir(dir,files);
	double num_samples_class_1 = 0;
	double num_samples_class_2 = 0;

	for (unsigned int i = 0;i < files.size();i++)
	{
		std::string ein_punkt = std::string(".");
		std::string zwei_punkt = std::string("..");

		if (ein_punkt.compare(files[i]) == 0)
		{
			i = i + 1;
		}

		if (zwei_punkt.compare(files[i]) == 0)
		{
			i = i + 1;
		}

		std::cout << files[i] << std::endl;
		path = dir + '/' + files[i];
		std::cout << "path for first file is: " << path << std::endl;
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *weingarten_utils.cloud_) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file \n");
		}
		std::cout << "Loading of pcd file is complete now." << std::endl;
		//std::cout << "size of the loaded point cloud is: " << cloud_->points.size () << std::endl;
		size_t size_of_pc = weingarten_utils.cloud_->points.size ();

		//remove NAN points from the cloud
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*weingarten_utils.cloud_,*weingarten_utils.cloud_, indices);

		// assign cloud_ to kdtree
		//weingarten_utils.kdtree_.setInputCloud(weingarten_utils.cloud_);

		weingarten_utils.analyse_surface();
		std::cout << " " << weingarten_utils.feature_.size() << std::endl;
	}
	num_samples_class_1 = weingarten_utils.feature_.size()/((weingarten_utils.added_shells_+1)*8 + 2);

		dir = std::string("kanne_rosa_predict");
		files = std::vector<std::string>();

		getdir(dir,files);

		for (unsigned int i = 0;i < files.size();i++)
				{
					std::string ein_punkt = std::string(".");
					std::string zwei_punkt = std::string("..");

					if (ein_punkt.compare(files[i]) == 0)
					{
						i = i + 1;
					}

					if (zwei_punkt.compare(files[i]) == 0)
					{
						i = i + 1;
					}
					std::cout << files[i] << std::endl;
					path = dir + '/' + files[i];

					std::cout << "path name for second file is: " << path << std::endl;
				}

		for (unsigned int i = 0;i < files.size();i++)
		{
			std::string ein_punkt = std::string(".");
			std::string zwei_punkt = std::string("..");

			if (ein_punkt.compare(files[i]) == 0)
			{
				i = i + 1;
			}

			if (zwei_punkt.compare(files[i]) == 0)
			{
				i = i + 1;
			}
			std::cout << files[i] << std::endl;
			path = dir + '/' + files[i];

			std::cout << "Loading of pcd file begins. Please wait ..." << std::endl;
			std::cout << "path name for second file is: " << path << std::endl;
			if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *weingarten_utils.cloud_) == -1) //* load the file
			{
				PCL_ERROR ("Couldn't read file \n");
			}
			std::cout << "Loading of pcd file is complete now." << std::endl;
			//std::cout << "size of the loaded point cloud is: " << cloud_->points.size () << std::endl;
			size_t size_of_pc = weingarten_utils.cloud_->points.size ();

			//remove NAN points from the cloud
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*weingarten_utils.cloud_,*weingarten_utils.cloud_, indices);

			// assign cloud_ to kdtree
			//weingarten_utils.kdtree_.setInputCloud(weingarten_utils.cloud_);
			/*
			for(size_t search_point_index = quadric_utils.index_offset_; search_point_index < quadric_utils.cloud_->points.size();)
			{
				quadric_utils.analyse_surface();
				search_point_index = search_point_index + quadric_utils.index_stepsize_;
			}
			*/
			weingarten_utils.analyse_surface();
			std::cout << "current length of the features vector is: " << weingarten_utils.feature_.size() << std::endl;
		}

		num_samples_class_2 = (weingarten_utils.feature_.size()/((weingarten_utils.added_shells_+1)*8 + 2))-num_samples_class_1;

		std::cout << "writing results to file begins now. Please wait..." << std::endl;
		// feature values fill rows
		// as many columns as samples taken
		for(double l = 0; l < weingarten_utils.feature_.size();)
			{
			for(double k = 0; k < ((weingarten_utils.added_shells_+1)*8 + 2);k++)
			{
				myfile << weingarten_utils.feature_[l + k] << " ";
			}
			myfile << "\n";
			l = l + ((weingarten_utils.added_shells_+1)*8 + 2);
		}
		std::cout << "writing results to file finished now." << std::endl;
		std::cout << "Number of samples taken in class 1 is: " << num_samples_class_1 << std::endl;
		std::cout << "Number of samples taken in class 2 is: " << num_samples_class_2 << std::endl;
}
else
{
	std::cout << "Unable to open file" << std::endl;
}
myfile.close();
}


void Evaluate_Data::euclidean_segmentation(void)
{
	Weingarten_Utils weingarten_utils;

	std::string dir;
	dir = std::string("blaukrautschuessel_predict");
	std::vector<std::string> files;
	files = std::vector<std::string>();
	std::string read_path;

	getdir(dir,files);
	double num_samples_class_1 = 0;
	double num_samples_class_2 = 0;

	for (unsigned int k = 0;k < files.size();k++)
	{
		std::string ein_punkt = std::string(".");
		std::string zwei_punkt = std::string("..");

		if (ein_punkt.compare(files[k]) == 0)
		{
			k = k + 1;
		}

		if (zwei_punkt.compare(files[k]) == 0)
		{
			k = k + 1;
		}

		std::cout << files[k] << std::endl;
		read_path = dir + '/' + files[k];
		std::cout << "path for first file is: " << read_path << std::endl;

		if (pcl::io::loadPCDFile<pcl::PointXYZ> (read_path, *weingarten_utils.cloud_) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file \n");
		}
		std::cout << "Loading of pcd file is complete now." << std::endl;
		//std::cout << "size of the loaded point cloud is: " << cloud_->points.size () << std::endl;
		size_t size_of_pc = weingarten_utils.cloud_->points.size ();

		// assign cloud_ to kdtree
		//quadric_utils.kdtree_.setInputCloud(quadric_utils.cloud_);

		//euclidean segmentation begin

		// Read in the cloud data
		  pcl::PCDReader reader;
		  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>)
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
		  reader.read (read_path, *weingarten_utils.cloud_);
		  std::cout << "PointCloud before filtering has: " << weingarten_utils.cloud_->points.size () << " data points." << std::endl; //*

		  // Create the filtering object: downsample the dataset using a leaf size of 1cm
		  pcl::VoxelGrid<pcl::PointXYZ> vg;
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		  vg.setInputCloud (weingarten_utils.cloud_);
		  vg.setLeafSize (0.01f, 0.01f, 0.01f);
		  vg.filter (*cloud_filtered);
		  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

		  // Create the segmentation object for the planar model and set all the parameters
		  pcl::SACSegmentation<pcl::PointXYZ> seg;
		  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		  pcl::PCDWriter writer;
		  seg.setOptimizeCoefficients (true);
		  seg.setModelType (pcl::SACMODEL_PLANE);
		  seg.setMethodType (pcl::SAC_RANSAC);
		  seg.setMaxIterations (100);
		  seg.setDistanceThreshold (0.02);

		  int i=0, nr_points = (int) cloud_filtered->points.size ();
		  while (cloud_filtered->points.size () > 0.3 * nr_points)
		  {
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloud_filtered);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
			  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			  break;
			}

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers);
			extract.setNegative (false);

			// Get the points associated with the planar surface
			extract.filter (*cloud_plane);
			std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

			// Remove the planar inliers, extract the rest
			extract.setNegative (true);
			extract.filter (*cloud_f);
			*cloud_filtered = *cloud_f;
		  }

		  // Creating the KdTree object for the search method of the extraction
		  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		  tree->setInputCloud (cloud_filtered);

		  std::vector<pcl::PointIndices> cluster_indices;
		  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		  ec.setClusterTolerance (0.02); // 2cm
		  ec.setMinClusterSize (100);
		  ec.setMaxClusterSize (25000);
		  ec.setSearchMethod (tree);
		  ec.setInputCloud (cloud_filtered);
		  ec.extract (cluster_indices);

		  int j = 0;
		  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		  {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			  cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			std::stringstream ss;
			ss << dir << "/" << k <<"euclidean_segmentation" << j << ".pcd";
			writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
			j++;
		  }

		//euclidean segmentation end
	}


}

void Evaluate_Data::view_pcd_file_name(void)
{
Weingarten_Utils quadric_utils;
std::string dir;
std::string pcd_file_number;
std::string segmentation_number;
dir = std::string("kanne_rosa_training/");
int continue_view = 1;
do{
	continue_view = 0;
	std::cout << "Please enter number of pcd file to be viewed" << std::endl;
	std::cin >> pcd_file_number;
	std::cout << "Please enter segmentation number of pcd file to be viewed" << std::endl;
	std::cin >> segmentation_number;

	std::string path;
	path = dir + pcd_file_number + "euclidean_segmentation" + segmentation_number + ".pcd";

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *quadric_utils.cloud_) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file \n");
		}
		std::cout << "Loading of pcd file is complete now." << std::endl;
		size_t size_of_pc = quadric_utils.cloud_->points.size ();
		std::cout << "size of the loaded point cloud " << path << "is: " << size_of_pc << std::endl;
		pcl::visualization::CloudViewer viewer (path);
	   viewer.showCloud (quadric_utils.cloud_);
	   while (!viewer.wasStopped ())
	      {
	      }
	   std::cout << "Enter 1 to continue. Enter 0 to quit" << std::endl;
	   std::cin >> continue_view;
}while(continue_view == 1);

}

void Evaluate_Data::view_pcd_enter_dir(void)
{
Quadric_Utils quadric_utils;
std::string dir_filename;
int continue_view = 1;
do{
	continue_view = 0;
	std::cout << "Please enter dir/filename" << std::endl;
	std::cin >> dir_filename;


	if (pcl::io::loadPCDFile<pcl::PointXYZ> (dir_filename, *quadric_utils.cloud_) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file \n");
		}
		std::cout << "Loading of pcd file is complete now." << std::endl;
		size_t size_of_pc = quadric_utils.cloud_->points.size ();
		std::cout << "size of the loaded point cloud " << dir_filename << "is: " << size_of_pc << std::endl;
		pcl::visualization::CloudViewer viewer (dir_filename);
	   viewer.showCloud (quadric_utils.cloud_);
	   while (!viewer.wasStopped ())
	      {
	      }
	   std::cout << "Enter 1 to continue. Enter 0 to quit" << std::endl;
	   std::cin >> continue_view;
}while(continue_view == 1);

}
