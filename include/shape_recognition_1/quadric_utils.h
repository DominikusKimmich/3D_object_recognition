#include <pcl/kdtree/kdtree_flann.h>
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>


class Quadric_Utils
{
	public :
	//member variables of Weingarten_Utils
	//member always get _at the end
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
	// declare kdtree for nearestKSearch
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
	// Initialize member variable with default of 100 neighbours in point cloud
	size_t num_neighbours_;
	// Coefficients of the model function which is a quadratic polynomial
	Eigen::MatrixXd quadratic_pol_coeff_;
	double a_,b_,c_,d_,e_,f_;
	//Maximum value of normal curvature
	double kappa_max_;
	//Minimum value of normal curvature
	double kappa_min_;
	//Vector contains kappa_max_ and kappa_min_ of the central point and analysis points in all shells of all features collected
	//The last two entries of the curvature_descriptor_ tell how many shells and how many features have been used
	// The last entry of curvature_descriptor_ denotes the number of features
	//The entry before the last entry of curvature_descriptor_ denotes the number of shells
	std::vector<double>feature_;
	// principal curvature directions computed in member function void tangent_axes()
	Eigen::Vector3d tangent_x_axis_;
	Eigen::Vector3d tangent_y_axis_;
	//Central point to be passed from member function develop_feature to member function std::vector<size_t>develop_next_shell(int first_shell)
	size_t index_central_point_;
	//Vector for storage of the indices of the last shell. Its being used to develop the new shell
	std::vector<size_t>last_shell_;
	// user defined variable for the calibration of the shell descriptor
	double shell_variable_;
	//user defined input about the number of shells that will be added to the first standard shell
	double added_shells_;

	//Store principal axes of the shell descriptor in the first two entries. Following eight entries
	//store tangent_x_axis_ and tangent_y_axis_ for all analysis points of the last shell
	std::vector<Eigen::Vector3d>last_tangent_axes_;
	//Determine index to start from in member function analyse_surface
	double index_offset_;
	//Select in which steps to walk through the point cloud
	double index_stepsize_;



	//member functions of Quadric_Utils
	//constructor
	Quadric_Utils();
	//destructor
	~Quadric_Utils();
	// Walk through point cloud in in equal sized steps of point cloud
	void analyse_surface();
	// find coefficients of quadratic polynomial that is fitted locally to surface of the point cloud
	void determine_modelfunction_coefficients(size_t point_index);
	void set_shell_descriptor_parameters();
	void kappa_max_min();
	void develop_feature(double point_index);
	void tangent_axes();
	std::vector<size_t>develop_next_shell(int first_shell);
};
