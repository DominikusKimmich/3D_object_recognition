
void Evaluate_Data::view_pcd_files(void)
{
Weingarten_Utils weingarten_utils;
std::string dir;
dir = std::string("kanne_rosa_predict");
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
	size_t size_of_pc = quadric_utils.cloud_->points.size ();
	std::cout << "size of the loaded point cloud " << files[i] << "is: " << size_of_pc << std::endl;
	pcl::visualization::CloudViewer viewer (files[i]);
   viewer.showCloud (quadric_utils.cloud_);

   while (!viewer.wasStopped ())
   {
   }


   std::cout << "hit a key" << std::endl;
   getchar();
   std::cout << "next pcd file" << std::endl;

}

getchar();
std::cout << "next pcd file" << std::endl;
getchar();
std::cout << "next pcd file" << std::endl;
getchar();

}
