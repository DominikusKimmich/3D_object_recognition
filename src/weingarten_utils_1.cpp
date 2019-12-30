#include <iostream>
#include <shape_recognition_1/weingarten_utils_1.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <pcl/features/normal_3d.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <cmath>
#include <map>
#include <limits>
#include <fstream>

Weingarten_Utils::Weingarten_Utils() : cloud_(new pcl::PointCloud<pcl::PointXYZ>), num_neighbours_(20), kappa_max_(0), kappa_min_(0), a_(0), b_(0), c_(0), d_(0), e_(0), f_(0), index_central_point_(0), shell_variable_(0.2), added_shells_(4), index_offset_(0), index_stepsize_(50)
{
}

Weingarten_Utils::~Weingarten_Utils()
{
	cloud_.reset();
}

void Weingarten_Utils::analyse_surface()
{
	// assign cloud_ to kdtree
	kdtree_.setInputCloud(cloud_);
	for(size_t search_point_index = index_offset_; search_point_index < cloud_->points.size();)
	{
		develop_feature(search_point_index);
		std::cout << "search_point_index " << search_point_index << std::endl;
		for (double j = 0; j < (added_shells_+1)*8 + 2; ++j)
		{
		std::cout << feature_[feature_.size()-((added_shells_+1)*8 + 2) + j] << std::endl;
		}
		search_point_index = search_point_index + index_stepsize_;
	}
	std::cout << "Point cloud has " << cloud_->points.size() << " coordinates " << std::endl;
}

void Weingarten_Utils::kappa_max_min()
{
	//Obtain principal curvatures from coefficients of the model function
	kappa_max_ = (-a_-c_+ sqrt(pow(a_, 2) + 4*pow(b_, 2) - 2* a_*c_ + pow(c_, 2)))/(sqrt(1 + pow(d_, 2) + pow(e_, 2)));
	kappa_min_ = (-a_-c_- sqrt(pow(a_, 2) + 4*pow(b_, 2) - 2* a_*c_ + pow(c_, 2)))/(sqrt(1 + pow(d_, 2) + pow(e_, 2)));
}

void Weingarten_Utils::tangent_axes()
{
	Eigen::Vector3d parametrisation_x;
	Eigen::Vector3d parametrisation_y;

	parametrisation_x(0)= 1;
	parametrisation_x(1)= 0;
	parametrisation_x(2)= d_;

	parametrisation_y(0)= 1;
	parametrisation_y(1)= 0;
	parametrisation_y(2)= e_;

	tangent_x_axis_ = (-a_-c_+ sqrt(pow(a_, 2) + 4*pow(b_, 2) - 2* a_*c_ + pow(c_, 2)))/(2*b_) * parametrisation_x + parametrisation_y;
	tangent_y_axis_ = (-a_-c_- sqrt(pow(a_, 2) + 4*pow(b_, 2) - 2* a_*c_ + pow(c_, 2)))/(2*b_) * parametrisation_x + parametrisation_y;
}

void Weingarten_Utils::determine_modelfunction_coefficients(size_t point_index)
{
		/*
		#########################################
		important section: subcloud of neighbours
		#########################################
		 */

		std::vector<int> pointIdxNKNSearch(num_neighbours_);
		std::vector<float> pointNKNSquaredDistance(num_neighbours_);

		// Placeholder for the 3x3 covariance matrix at each surface patch
		Eigen::Matrix3f covariance_matrix;

		//Initialize a sub cloud of the cloud that contains only the neighbours of the point adresse by point_index
		pcl::PointCloud<pcl::PointXYZ> sub_cloud;

		if ( kdtree_.nearestKSearch (cloud_->points[point_index],
			num_neighbours_,
			pointIdxNKNSearch,
			pointNKNSquaredDistance) > 0 )
		{

			sub_cloud.width    = pointIdxNKNSearch.size ();
			sub_cloud.height   = 1;
			sub_cloud.points.resize (sub_cloud.width * sub_cloud.height);

			//Compute sub cloud initialized above
			for (size_t k_neighbour_iteration = 0; k_neighbour_iteration < pointIdxNKNSearch.size (); ++k_neighbour_iteration)
			{
				sub_cloud.points[k_neighbour_iteration].x = cloud_->points[ pointIdxNKNSearch[k_neighbour_iteration] ].x;
				sub_cloud.points[k_neighbour_iteration].y = cloud_->points[ pointIdxNKNSearch[k_neighbour_iteration] ].y;
				sub_cloud.points[k_neighbour_iteration].z = cloud_->points[ pointIdxNKNSearch[k_neighbour_iteration] ].z;
			}
		}
		/*
		###############################################
		important section: principle component analysis
		###############################################
		*/
		//Declare center of gravity of local principle component analysis
		Eigen::Vector4f xyz_centroid;
		// Estimate the XYZ centroid
		pcl::compute3DCentroid (sub_cloud, xyz_centroid);

		// Compute the 3x3 covariance matrix of local principle component analysis
		pcl::computeCovarianceMatrix (sub_cloud, xyz_centroid, covariance_matrix);

		// compute eigenvectors and eigenvalues of the covariance matrix
		Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);

		// compute eigenvectors as complex valued vectors imaginary part is always zero
		Eigen::Vector3cf first_eigenvector = es.eigenvectors().col(0);
		Eigen::Vector3cf second_eigenvector = es.eigenvectors().col(1);
		Eigen::Vector3cf third_eigenvector = es.eigenvectors().col(2);

		// cast format of complex vectors to real vectors
		Eigen::Vector3f first_eigenvector_real = first_eigenvector.real();
		Eigen::Vector3f second_eigenvector_real = second_eigenvector.real();
		Eigen::Vector3f third_eigenvector_real = third_eigenvector.real();

		// Eigenvalues of the covariance matrix yield the variances of principal component analysis of sub_cloud
		Eigen::Vector3cf sub_cloud_variances = es.eigenvalues();
		Eigen::Vector3f sub_cloud_variances_real = sub_cloud_variances.real();
		float sub_cloud_first_variance = sub_cloud_variances_real[0];
		sub_cloud_first_variance = std::abs(sub_cloud_first_variance);
		float sub_cloud_second_variance = sub_cloud_variances_real[1];
		sub_cloud_second_variance = std::abs(sub_cloud_second_variance);
		float sub_cloud_third_variance = sub_cloud_variances_real[2];
		sub_cloud_third_variance = std::abs(sub_cloud_third_variance);

		//Sort eigenvectors of principle component analysis by their eigenvalues in descending order
		// Initialize map container of eigenvalues associated with their corresponding eigenvector
		//This makes sure that eigenvectors with the two greatest eigenvalues form the parametrisation of the function graph, whereas height of the functiongraph corresponds to the eigenvetor with smallest eigenvalue
		std::map<float, Eigen::Vector3f> eigenpair;
		eigenpair[sub_cloud_first_variance] = first_eigenvector_real;
		eigenpair[sub_cloud_second_variance] = second_eigenvector_real;
		eigenpair[sub_cloud_third_variance] = third_eigenvector_real;

		//Assign eigenvectors of covariance matrix to local components
		std::map<float, Eigen::Vector3f>::iterator it=eigenpair.begin();

		//Assign eigenvector with smallest eigenvalue to pca_z_component_
		Eigen::Vector3f pca_z_component = it->second;
		++it;

		//Assign eigenvector with intermediate eigenvalue to pca_y_component_
		Eigen::Vector3f pca_y_component = it->second;
		++it;

		//Assign eigenvector with greatest eigenvalue to pca_x_component_
		Eigen::Vector3f pca_x_component = it->second;
		//Use greatest Eigenvalue of pca for length limitation of next_shell-Vectors in member function develop_next_shell
		/*
		#############################################################
		important section: least squares fit for quadratic polynomial
		#############################################################
		*/
		// Initialize vector that hold positions of points in the sub cloud in camera coordinate system
		Eigen::Vector3f sub_cloud_vector_camera_coordinates;
		// Initialize vector that hold positions of points in eigensytem obtained from principle component analysis
		Eigen::Vector3f sub_cloud_vector_local;

		// Initialize Matrices and Vectors to solve least squares problem using a Cholesky decomposition before solving normal equations
		Eigen::MatrixXd A(pointIdxNKNSearch.size (),6);
		Eigen::VectorXd b(pointIdxNKNSearch.size ());
		Eigen::MatrixXd B(6,6);

		// A*quadratic_pol_coeff yields the quadratic polynomial
		// Vector b contains height values or z values of the sub_cloud realtive to the coordinate system formed by xyz_centroid and a basis formed by  the eigenvectors of principle component analysis above
		for (size_t k_neighbour_iteration = 0; k_neighbour_iteration < pointIdxNKNSearch.size (); ++k_neighbour_iteration)
		{
			sub_cloud_vector_camera_coordinates[0] = sub_cloud.points[k_neighbour_iteration].x - xyz_centroid[0];
			sub_cloud_vector_camera_coordinates[1] = sub_cloud.points[k_neighbour_iteration].y - xyz_centroid[1];
			sub_cloud_vector_camera_coordinates[2] = sub_cloud.points[k_neighbour_iteration].z - xyz_centroid[2];

			sub_cloud_vector_local[0] = sub_cloud_vector_camera_coordinates.dot(pca_x_component);
			sub_cloud_vector_local[1] = sub_cloud_vector_camera_coordinates.dot(pca_y_component);
			sub_cloud_vector_local[2] = sub_cloud_vector_camera_coordinates.dot(pca_z_component);

			A(k_neighbour_iteration, 0) = sub_cloud_vector_local[0] * sub_cloud_vector_local[0];
			A(k_neighbour_iteration, 1) = 2* sub_cloud_vector_local[0] * sub_cloud_vector_local[1];
			A(k_neighbour_iteration, 2) = sub_cloud_vector_local[1] * sub_cloud_vector_local[1];
			A(k_neighbour_iteration, 3) = sub_cloud_vector_local[0];
			A(k_neighbour_iteration, 4) = sub_cloud_vector_local[1];
			A(k_neighbour_iteration, 5) = 1;

			//Build Vector b as height of the sub_cloud coordinates over the local coordinate system
			//This is the z-component of sub_cloud_vector_local
			b(k_neighbour_iteration) = sub_cloud_vector_local[2];
		}
	B = A.transpose() * A;
	// Perform solution of least squares linear system using normal equations
	quadratic_pol_coeff_ = (B).ldlt().solve(A.transpose() * b);

	//catch coefficients of model function that have been computed above
	a_ = quadratic_pol_coeff_(0);
	b_ = quadratic_pol_coeff_(1);
	c_ = quadratic_pol_coeff_(2);
	d_ = quadratic_pol_coeff_(3);
	e_ = quadratic_pol_coeff_(4);
	f_ = quadratic_pol_coeff_(5);
}

void Weingarten_Utils::set_shell_descriptor_parameters()
{
//user input for shell variable
do {
	std::cout << "Please enter shell variable not zero: " << std::endl;
	std::cin >> shell_variable_;
	} while (shell_variable_ == 0);
shell_variable_ = abs(shell_variable_);

//user input for the number of shells
std::cout << "How many shells do you want to add to first shell ? " << std::endl;

//std::cin.ignore(std::numeric_limits<std::streamsize>::max());
std::cin >> added_shells_;
added_shells_ = static_cast<int>(std::abs(added_shells_));

// std::cin.ignore(std::numeric_limits<std::streamsize>::max());
std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
std::cin.clear();

//Determine index to start from in member function analyse_surface
std::cout << "Determine index to start from in member function analyse_surface " << std::endl;
std::cin >> index_offset_;
index_offset_ = abs(index_offset_);

//Select in which steps to walk through the point cloud
do {
	std::cout << "Select in which steps to walk through the point cloud " << std::endl;
	std::cin >> index_stepsize_;
	} while (index_stepsize_ == 0);
index_stepsize_ = abs(index_stepsize_);

//user input for the least squares fit
size_t input_num_neighbours;
std::cout << "Please enter number of neighbours to be considered least squares fit" << std::endl;
std::cin >> input_num_neighbours;
num_neighbours_ = input_num_neighbours;
}

std::vector<size_t> Weingarten_Utils::develop_next_shell(int first_shell)
{

std::vector<size_t>new_shell;

Eigen::Vector3d central_point_vec;
Eigen::Vector3d analysis_point_pos_x_vec;
Eigen::Vector3d analysis_point_pos_y_vec;
Eigen::Vector3d analysis_point_neg_x_vec;
Eigen::Vector3d analysis_point_neg_y_vec;

Eigen::Vector3d pos_x_next_shell_vec;
Eigen::Vector3d pos_y_next_shell_vec;
Eigen::Vector3d neg_x_next_shell_vec;
Eigen::Vector3d neg_y_next_shell_vec;

/*########################################################
section: Develop first shell
##########################################################*/
if(first_shell == 1)
{
	// Compute principal axes of the feature
	tangent_axes();
	// Store principal axes computed above for development of further shells later
	last_tangent_axes_.push_back(tangent_x_axis_);
	last_tangent_axes_.push_back(tangent_y_axis_);

	//Get vector of the central point in camera coordinates
	central_point_vec[0] = cloud_->points[index_central_point_].x;
	central_point_vec[1] = cloud_->points[index_central_point_].y;
	central_point_vec[2] = cloud_->points[index_central_point_].z;

	//Add connection vectors to the vector of the central point
	pos_x_next_shell_vec = central_point_vec + (shell_variable_/std::abs(kappa_max_)) * tangent_x_axis_;
	pos_y_next_shell_vec = central_point_vec + (shell_variable_/std::abs(kappa_min_)) * tangent_y_axis_;
	neg_x_next_shell_vec = central_point_vec + (-shell_variable_/std::abs(kappa_max_)) * tangent_x_axis_;
	neg_y_next_shell_vec = central_point_vec + (-shell_variable_/std::abs(kappa_min_)) * tangent_y_axis_;

//Store indices of the new shell computed above for use at next call of this member function
last_shell_.push_back(index_central_point_);
last_shell_.push_back(index_central_point_);
last_shell_.push_back(index_central_point_);
last_shell_.push_back(index_central_point_);
}
/*########################################################
section: Develop further shells
##########################################################*/

else
{
	//Get vector of analysis points of the last shell in camera coordinates
	analysis_point_pos_x_vec[0] = cloud_->points[last_shell_[0]].x;
	analysis_point_pos_x_vec[1] = cloud_->points[last_shell_[0]].y;
	analysis_point_pos_x_vec[2] = cloud_->points[last_shell_[0]].z;

	analysis_point_pos_y_vec[0] = cloud_->points[last_shell_[1]].x;
	analysis_point_pos_y_vec[1] = cloud_->points[last_shell_[1]].y;
	analysis_point_pos_y_vec[2] = cloud_->points[last_shell_[1]].z;

	analysis_point_neg_x_vec[0] = cloud_->points[last_shell_[2]].x;
	analysis_point_neg_x_vec[1] = cloud_->points[last_shell_[2]].y;
	analysis_point_neg_x_vec[2] = cloud_->points[last_shell_[2]].z;

	analysis_point_neg_y_vec[0] = cloud_->points[last_shell_[3]].x;
	analysis_point_neg_y_vec[1] = cloud_->points[last_shell_[3]].y;
	analysis_point_neg_y_vec[2] = cloud_->points[last_shell_[3]].z;

	/*########################################################
	section: Compute connection vector in positive x direction
	##########################################################*/

	//Compute direction of local connection vector for analysis point in positive x direction in local tangent basis in order to obtain kappa_normal
	Eigen::Vector2d pos_x_kappa_normal_direction_2d;
	//Project principal x axis to first local principal curvature direction
	pos_x_kappa_normal_direction_2d(0) = last_tangent_axes_[0].dot(last_tangent_axes_[2]);
	//Project principal x axis to second local principal curvature direction
	pos_x_kappa_normal_direction_2d(1) = last_tangent_axes_[0].dot(last_tangent_axes_[3]);
	//normalize pos_x_kappa_normal_direction for use in euler equation further down
	pos_x_kappa_normal_direction_2d = pos_x_kappa_normal_direction_2d.normalized();

	double pos_x_kappa_normal_direction_fc = pos_x_kappa_normal_direction_2d(0);
	double pos_x_kappa_normal_direction_sc = pos_x_kappa_normal_direction_2d(1);
	//Apply Euler Equation for kappa_normal
	double pos_x_kappa_normal = pow(pos_x_kappa_normal_direction_fc, 2) * feature_[feature_.size()-8] + pow(pos_x_kappa_normal_direction_sc, 2) * feature_[feature_.size()-7];

	Eigen::Vector3d pos_x_kappa_normal_direction_3d = last_tangent_axes_[0].dot(last_tangent_axes_[2])*last_tangent_axes_[2] + last_tangent_axes_[0].dot(last_tangent_axes_[3])* last_tangent_axes_[3];
	pos_x_kappa_normal_direction_3d = pos_x_kappa_normal_direction_3d.normalized();

	pos_x_next_shell_vec = analysis_point_pos_x_vec + abs((shell_variable_/pos_x_kappa_normal)) * pos_x_kappa_normal_direction_3d;

	/*########################################################
	section: Compute connection vector in positive y direction
	##########################################################*/

	//Compute direction of local connection vector for analysis point in positive y direction in local tangent basis in order to obtain kappa_normal
	Eigen::Vector2d pos_y_kappa_normal_direction_2d;
	//Project principal y axis to first local principal curvature direction
	pos_y_kappa_normal_direction_2d(0) = last_tangent_axes_[1].dot(last_tangent_axes_[4]);
	//Project principal y axis to second local principal curvature direction
	pos_y_kappa_normal_direction_2d(1) = last_tangent_axes_[1].dot(last_tangent_axes_[5]);
	//normalize pos_x_kappa_normal_direction for use in euler equation further down
	pos_y_kappa_normal_direction_2d = pos_y_kappa_normal_direction_2d.normalized();

	double pos_y_kappa_normal_direction_fc = pos_y_kappa_normal_direction_2d(0);
	double pos_y_kappa_normal_direction_sc = pos_y_kappa_normal_direction_2d(1);
	//Apply Euler Equation for kappa_normal
	double pos_y_kappa_normal = pow(pos_y_kappa_normal_direction_fc, 2) * feature_[feature_.size()-6] + pow(pos_y_kappa_normal_direction_sc, 2) * feature_[feature_.size()-5];

	Eigen::Vector3d pos_y_kappa_normal_direction_3d = last_tangent_axes_[1].dot(last_tangent_axes_[4])*last_tangent_axes_[4] + last_tangent_axes_[1].dot(last_tangent_axes_[5])* last_tangent_axes_[5];
	pos_y_kappa_normal_direction_3d = pos_y_kappa_normal_direction_3d.normalized();

	pos_y_next_shell_vec = analysis_point_pos_y_vec + abs((shell_variable_/pos_y_kappa_normal)) * pos_y_kappa_normal_direction_3d;

	/*########################################################
	section: Compute connection vector in negative x direction
	##########################################################*/

	//Compute direction of local connection vector for analysis point in negative x direction in local tangent basis in order to obtain kappa_normal
	Eigen::Vector2d neg_x_kappa_normal_direction_2d;
	//Project inverse principal x axis to first local principal curvature direction
	neg_x_kappa_normal_direction_2d(0) = (-1) * last_tangent_axes_[0].dot(last_tangent_axes_[6]);
	//Project principal y axis to second local principal curvature direction
	neg_x_kappa_normal_direction_2d(1) = (-1) * last_tangent_axes_[0].dot(last_tangent_axes_[7]);
	//normalize neg_x_kappa_normal_direction for use in euler equation further down
	neg_x_kappa_normal_direction_2d = neg_x_kappa_normal_direction_2d.normalized();

	double neg_x_kappa_normal_direction_fc = neg_x_kappa_normal_direction_2d(0);
	double neg_x_kappa_normal_direction_sc = neg_x_kappa_normal_direction_2d(1);
	//Apply Euler Equation for kappa_normal
	double neg_x_kappa_normal = pow(neg_x_kappa_normal_direction_fc, 2) * feature_[feature_.size()-4] + pow(pos_y_kappa_normal_direction_sc, 2) * feature_[feature_.size()-3];

	Eigen::Vector3d neg_x_kappa_normal_direction_3d = (-1) * last_tangent_axes_[0].dot(last_tangent_axes_[6])*last_tangent_axes_[6] + (-1) * last_tangent_axes_[0].dot(last_tangent_axes_[7])* last_tangent_axes_[7];
	neg_x_kappa_normal_direction_3d = neg_x_kappa_normal_direction_3d.normalized();

	neg_x_next_shell_vec = analysis_point_neg_x_vec + abs(shell_variable_/neg_x_kappa_normal) * neg_x_kappa_normal_direction_3d;


	/*########################################################
	section: Compute connection vector in negative y direction
	##########################################################*/

	//Compute direction of local connection vector for analysis point in negative y direction in local tangent basis in order to obtain kappa_normal
	Eigen::Vector2d neg_y_kappa_normal_direction_2d;
	//Project inverse principal y axis to first local principal curvature direction
	neg_y_kappa_normal_direction_2d(0) = (-1) * last_tangent_axes_[1].dot(last_tangent_axes_[8]);
	//Project principal y axis to second local principal curvature direction
	neg_y_kappa_normal_direction_2d(1) = (-1) * last_tangent_axes_[1].dot(last_tangent_axes_[9]);
	//normalize neg_y_kappa_normal_direction for use in euler equation further down
	neg_y_kappa_normal_direction_2d = neg_y_kappa_normal_direction_2d.normalized();

	double neg_y_kappa_normal_direction_fc = neg_y_kappa_normal_direction_2d(0);
	double neg_y_kappa_normal_direction_sc = neg_y_kappa_normal_direction_2d(1);
	//Apply Euler Equation for kappa_normal
	double neg_y_kappa_normal = pow(neg_y_kappa_normal_direction_fc, 2) * feature_[feature_.size()-2] + pow(pos_y_kappa_normal_direction_sc, 2) * feature_[feature_.size()-1];

	Eigen::Vector3d neg_y_kappa_normal_direction_3d = (-1) * last_tangent_axes_[1].dot(last_tangent_axes_[8])*last_tangent_axes_[8] + (-1) * last_tangent_axes_[1].dot(last_tangent_axes_[9])* last_tangent_axes_[9];
	neg_y_kappa_normal_direction_3d = neg_y_kappa_normal_direction_3d.normalized();

	neg_y_next_shell_vec = analysis_point_neg_y_vec + abs(shell_variable_/neg_y_kappa_normal) * neg_y_kappa_normal_direction_3d;

	//Delete the last 8 elements that belong to local principal curvature directions of the last shell
	//local principal curvature directions will be built for the next shell in member function develop_feature when evaluating the new shell
	last_tangent_axes_.pop_back();
	last_tangent_axes_.pop_back();
	last_tangent_axes_.pop_back();
	last_tangent_axes_.pop_back();
	last_tangent_axes_.pop_back();
	last_tangent_axes_.pop_back();
	last_tangent_axes_.pop_back();
	last_tangent_axes_.pop_back();
}

/*##############################################
section: Obtain analysis points of the new shell
################################################*/
//Apply nearestKSearch to the positions pointed out by of connection vectors
//Find point in point cloud that has closest distance to "top" of the pos_x_next_shell_vec using nearestKsearch
pcl::PointXYZ pos_x_next_shell;
pos_x_next_shell.x = pos_x_next_shell_vec[0];
pos_x_next_shell.y = pos_x_next_shell_vec[1];
pos_x_next_shell.z = pos_x_next_shell_vec[2];

std::vector<int> NN_point_indices(1);
std::vector<float> NNpi_SquaredDistance(1);

if(isnan(pos_x_next_shell.x)||isinf(pos_x_next_shell.x))
{
	pos_x_next_shell.x = cloud_->points[last_shell_[0]].x;;
	std::cout << "pos_x_next_shell.x was nan; corrected to coordinate of analysis point" << std::endl;
}
if(isnan(pos_x_next_shell.y)||isinf(pos_x_next_shell.y))
{
	pos_x_next_shell.y = cloud_->points[last_shell_[0]].y;;
	std::cout << "pos_x_next_shell.y was nan; corrected to coordinate of analysis point" << std::endl;
}
if(isnan(pos_x_next_shell.z)||isinf(pos_x_next_shell.z))
{
	pos_x_next_shell.z = cloud_->points[last_shell_[0]].z;;
	std::cout << "pos_x_next_shell.z was nan; corrected to coordinate of analysis point" << std::endl;
}
kdtree_.nearestKSearch (pos_x_next_shell, 1, NN_point_indices, NNpi_SquaredDistance);
new_shell.push_back(NN_point_indices[0]);

//Find point in point cloud that has closest distance to "top" of the pos_y_next_shell_vec using nearestKsearch
pcl::PointXYZ pos_y_next_shell;
pos_y_next_shell.x = pos_y_next_shell_vec[0];
pos_y_next_shell.y = pos_y_next_shell_vec[1];
pos_y_next_shell.z = pos_y_next_shell_vec[2];

if(isnan(pos_y_next_shell.x)||isinf(pos_y_next_shell.x))
{
	pos_y_next_shell.x = cloud_->points[last_shell_[1]].x;;
	std::cout << "pos_y_next_shell.x was nan; corrected to coordinate of analysis point" << std::endl;
}
if(isnan(pos_y_next_shell.y)||isinf(pos_y_next_shell.y))
{
	pos_y_next_shell.y = cloud_->points[last_shell_[1]].y;;
	std::cout << "pos_y_next_shell.y was nan; corrected to coordinate of analysis point" << std::endl;
}
if(isnan(pos_y_next_shell.z)||isinf(pos_y_next_shell.z))
{
	pos_y_next_shell.z = cloud_->points[last_shell_[1]].z;;
	std::cout << "pos_y_next_shell.z was nan; corrected to coordinate of analysis point" << std::endl;
}
kdtree_.nearestKSearch (pos_y_next_shell, 1, NN_point_indices, NNpi_SquaredDistance);
new_shell.push_back(NN_point_indices[0]);

//Find point in point cloud that has closest distance to "top" of the neg_x_next_shell_vec using nearestKsearch
pcl::PointXYZ neg_x_next_shell;
neg_x_next_shell.x = neg_x_next_shell_vec[0];
neg_x_next_shell.y = neg_x_next_shell_vec[1];
neg_x_next_shell.z = neg_x_next_shell_vec[2];

if(isnan(neg_x_next_shell.x)||isinf(neg_x_next_shell.x))
{
	neg_x_next_shell.x = cloud_->points[last_shell_[2]].x;;
	std::cout << "neg_x_next_shell.x was nan; corrected to coordinate of analysis point" << std::endl;
}
if(isnan(neg_x_next_shell.y)||isinf(neg_x_next_shell.y))
{
	neg_x_next_shell.y = cloud_->points[last_shell_[2]].y;;
	std::cout << "neg_x_next_shell.y was nan; corrected to coordinate of analysis point" << std::endl;
}
if(isnan(neg_x_next_shell.z)||isinf(pos_x_next_shell.z))
{
	neg_x_next_shell.z = cloud_->points[last_shell_[2]].z;;
	std::cout << "neg_x_next_shell.z was nan; corrected to coordinate of analysis point" << std::endl;
}
kdtree_.nearestKSearch (neg_x_next_shell, 1, NN_point_indices, NNpi_SquaredDistance);
new_shell.push_back(NN_point_indices[0]);

//Find point in point cloud that has closest distance to "top" of the neg_y_next_shell_vec using nearestKsearch
pcl::PointXYZ neg_y_next_shell;
neg_y_next_shell.x = neg_y_next_shell_vec[0];
neg_y_next_shell.y = neg_y_next_shell_vec[1];
neg_y_next_shell.z = neg_y_next_shell_vec[2];

if(isnan(neg_y_next_shell.x)||isinf(neg_y_next_shell.x))
{
	neg_y_next_shell.x = cloud_->points[last_shell_[3]].x;;
	std::cout << "neg_y_next_shell.x was nan; corrected to coordinate of analysis point" << std::endl;
}
if(isnan(neg_y_next_shell.y)||isinf(neg_y_next_shell.y))
{
	neg_y_next_shell.y = cloud_->points[last_shell_[3]].y;;
	std::cout << "neg_y_next_shell.y was nan; corrected to coordinate of analysis point" << std::endl;
}
if(isnan(neg_y_next_shell.z)||isinf(neg_y_next_shell.z))
{
	neg_y_next_shell.z = cloud_->points[last_shell_[3]].z;;
	std::cout << "neg_y_next_shell.z was nan; corrected to coordinate of analysis point" << std::endl;
}
kdtree_.nearestKSearch (neg_y_next_shell, 1, NN_point_indices, NNpi_SquaredDistance);
new_shell.push_back(NN_point_indices[0]);

// Delete indices of analysis points above after they have been used above
// last_shell_ will be built up containing the indices of the new shell. This is done by the end of this member function
last_shell_.pop_back();
last_shell_.pop_back();
last_shell_.pop_back();
last_shell_.pop_back();

//Store indices of the new shell computed above for use at next call of this member function
last_shell_.push_back(new_shell[0]);
last_shell_.push_back(new_shell[1]);
last_shell_.push_back(new_shell[2]);
last_shell_.push_back(new_shell[3]);
//Pass indices of new shell computed above to member function develop_feature in order to evaluate the analysis points computed above

return new_shell;
}

void Weingarten_Utils::develop_feature(double point_index)
{
	//Evaluation of central point
	determine_modelfunction_coefficients(point_index);
	kappa_max_min();
	feature_.push_back(kappa_max_);
	feature_.push_back(kappa_min_);
	index_central_point_ = point_index;

	//Tell member function develop_next_shell if the first shell is developed or if further shells are developed
	int first_shell = 1;
	std::vector<size_t>new_shell = develop_next_shell(first_shell);
	first_shell = 0;

	//Evaluate first shell and prepare developement of second shell by storing principal curvature directions at every analysis point
	for (int i = 0; i < 4; ++i)
	{
	determine_modelfunction_coefficients(new_shell[i]);
	kappa_max_min();
	feature_.push_back(kappa_max_);
	feature_.push_back(kappa_min_);
	//Compute principal curvature directions at every analysis point
	tangent_axes();
	//Store the directions obtained above
	last_tangent_axes_.push_back(tangent_x_axis_);
	last_tangent_axes_.push_back(tangent_y_axis_);
	}
	//Development and evaluation of further shells
	for (double j = 0; j < added_shells_; ++j)
	{
		//Develop new shell
		new_shell = develop_next_shell(first_shell);
		//Evaluate new shell and prepare developement of next shell by storing principal curvature directions at every analysis point of the new shell
		for (int i = 0; i < 4; ++i)
			{
			determine_modelfunction_coefficients(new_shell[i]);
			kappa_max_min();
			feature_.push_back(kappa_max_);
			feature_.push_back(kappa_min_);
			//Compute principal curvature directions at every analysis point of the new shell
			tangent_axes();
			//Store the directions obtained above
			last_tangent_axes_.push_back(tangent_x_axis_);
			last_tangent_axes_.push_back(tangent_y_axis_);
			}
	}
}
