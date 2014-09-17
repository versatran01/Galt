#include "omnimapper_ros/voxel_grid.h"

#define MIN(a,b) (a<b)?a:b
#define MAX(a,b) (a>b)?a:b

VoxelGrid::VoxelGrid()
{
}

VoxelGrid::~VoxelGrid()
{
}

void VoxelGrid::initParams(ros::NodeHandle& nh, int mode)
{
  double length;
  double width;
  double height;
  double origin_x;
  double origin_y;
  double origin_z;
  double _rgbd_dis;
  std::string _name;
 
	nh.param("r_xy", res_xy_, 0.05);
	nh.param("r_z", res_z_, 0.05);
  nh.param("inc_mag", inc_mag_, 1);
  nh.param("dec_mag", dec_mag_, 1);
  nh.param("rgbd_max_range", rgbd_max_range_, 1000.0);
  nh.param("rgbd_thr/dis", _rgbd_dis, 0.3);


  if(mode != 3){
    _name = std::string("VoxelGrid");
    nh.param("length", length, 1.0);
    nh.param("width", width, 1.0);
    nh.param("height", height, 1.0);

    nh.param("origin_x", origin_x, 0.0);
    nh.param("origin_y", origin_y, 0.0);
    nh.param("origin_z", origin_z, 0.0);
  }
  else{
    _name = std::string("VoxelPatch");
    length = 2 * rgbd_max_range_ + 2* _rgbd_dis;
    width = 2 * rgbd_max_range_ + 2 * _rgbd_dis;
    nh.param("height", height, 1.0);
    origin_x = -rgbd_max_range_ - _rgbd_dis;
    origin_y = -rgbd_max_range_ - _rgbd_dis;
    nh.param("origin_z", origin_z, 0.0);
  }
	nh.param("use_ray_casting", use_ray_casting_, true);
	nh.param("use_rgbd", use_rgbd_, true);
	nh.param("occ_thr", occ_thr_, 50);

  RGBDSensorTransform st;
  st.initParams(nh);

  Rcb_ = st.sensor_to_body;
  Rlb_ = Eigen::Translation3d(0, 0, 0)
		*Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
		*Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
		*Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

  if(mode == 1)
  {
    VAL_MAX = 127;
    VAL_EVEN = 64;
    VAL_MIN = 0;
    VAL_UNKNOWN = -1;
  }
  else if(mode != 1)
  {
    VAL_MAX = 64;
    VAL_EVEN = 0;
    VAL_MIN = -63;
    VAL_UNKNOWN = 0;
  }
  SetSpace(length, width, height, origin_x, origin_y, origin_z);
  if(mode != 2){
    printf("%s: initialized!!!!!!!\n", _name.c_str());
    printf("%s =================================================\n", _name.c_str());
    printf("Set shift as [%.2f, %.2f, %.2f]\n", st.shift_x, st.shift_y, st.shift_z);
    printf("Set resolution as res_xy: [%.2f], res_z: [%.2f]\n", res_xy_, res_z_);
    printf("Use use_ray_casting = %u\n", use_ray_casting_);
    printf("Use use_rgbd = %u\n", use_rgbd_);
    printf("Occ_thr = %d\n", occ_thr_);
    printf("Inc_mag = %d\n", inc_mag_);
    printf("Dec_mag = %d\n", dec_mag_);
    printf("Pre allocate space %.2f x %.2f x %.2f at origin [%.2f, %.2f, %.2f]\n", 
        length, width, height, origin_x, origin_y, origin_z);
    printf("===========================================================\n\n\n");
  }

 pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

std_msgs::Float64MultiArray VoxelGrid::RetrieveSpace()
{
  std_msgs::Float64MultiArray space;
  space.data.push_back(dim_.x * res_xy_);
  space.data.push_back(dim_.y * res_xy_);
  space.data.push_back(dim_.z * res_z_);
  space.data.push_back(origin_.x * res_xy_);
  space.data.push_back(origin_.y * res_xy_);
  space.data.push_back(origin_.z * res_z_);
  return space;
}

void VoxelGrid::SetSpace(double length, double width, double height, double origin_x, double origin_y, double origin_z)
{
  unsigned int l = length / res_xy_; 
  unsigned int w = width / res_xy_; 
  unsigned int h = height / res_z_;
  int o_x = origin_x / res_xy_;
  int o_y = origin_y / res_xy_;
  int o_z = origin_z / res_z_;
  uint3 new_dim;
  int3 new_o;
  new_dim.x = l;
  new_dim.y = w;
  new_dim.z = h;
  new_o.x = o_x;
  new_o.y = o_y;
  new_o.z = o_z;
  Allocate(new_dim, new_o);
}

void VoxelGrid::SubClear(int n, int n_threads)
{
	unsigned int min_z = dim_.z * n / n_threads;
	unsigned int max_z = dim_.z * (n + 1) / n_threads;
  for (unsigned int l = 0; l < dim_.x; l++)
    for (unsigned int w = 0; w < dim_.y; w++)
      for (unsigned int h = min_z; h < max_z; h++)
      {
        map[l][w][h] = VAL_UNKNOWN;
        if(use_rgbd_)
          map_rgb[l][w][h] = RGB_UNKNOWN;
      }
}

void VoxelGrid::Clear(int n_threads) 
{
   for(int i = 0; i < n_threads; i++)
    clearMapThreads.create_thread(boost::bind(&VoxelGrid::SubClear, this, i, n_threads));
  clearMapThreads.join_all();
}

void VoxelGrid::MergeSubGrid(const array_type& grid)
{
  for(int l = 0; l < (int) dim_.x; l++)
    for(int w = 0; w < (int) dim_.y; w++)
      for(int h = 0; h <  (int) dim_.z; h++)
      {
        if(grid[l][w][h] == 0)
          continue;
        else if(map[l][w][h] == VAL_UNKNOWN && grid[l][w][h] != 0)
          map[l][w][h] = VAL_EVEN;

        int sum = map[l][w][h] + grid[l][w][h];
        if(sum > VAL_MAX)
          sum = VAL_MAX;
        else if(sum < VAL_MIN)
          sum = VAL_MIN;
        map[l][w][h] = sum;
      }
}


void VoxelGrid::MergeSubGrid(const array_type& grid, const array_rgb& grid_rgb)
{
  for(int l = 0; l < (int) dim_.x; l++)
    for(int w = 0; w < (int) dim_.y; w++)
      for(int h = 0; h <  (int) dim_.z; h++)
      {
        AddRGB(map_rgb[l][w][h], grid_rgb[l][w][h]);
        int sum = map[l][w][h] + grid[l][w][h];
        if(sum > VAL_MAX)
          sum = VAL_MAX;
        else if(sum < VAL_MIN)
          sum = VAL_MIN;
        map[l][w][h] = sum;
      }
}

bool VoxelGrid::GetVoxel(int3 pose, char*& voxel, rgb3*& rgb)
{
	bool onmap = true;
	if(pose.x < origin_.x || pose.y < origin_.y || pose.z < origin_.z ||
     pose.x >= (int)dim_.x + origin_.x || pose.y >= (int)dim_.y + origin_.y || pose.z >= (int)dim_.z + origin_.z)
	{
		onmap = false;
	}
	if(onmap){
		voxel = &(map[pose.x-origin_.x][pose.y-origin_.y][pose.z-origin_.z]);
		if(use_rgbd_)
			rgb = &(map_rgb[pose.x-origin_.x][pose.y-origin_.y][pose.z-origin_.z]);
	}
  return onmap;
}

void VoxelGrid::AddRGB(rgb3& rgb, rgb3 color)
{
	if(rgb.n == 1)
		rgb = color;
	else
		rgb = rgb + color; 
}

void VoxelGrid::Inc(char& val)
{
  if( val == VAL_UNKNOWN)
    val = std::min(VAL_EVEN + inc_mag_, VAL_MAX);
  else if(val + inc_mag_ < VAL_MAX) 
    val += inc_mag_;
  else 
    val = VAL_MAX;

}

void VoxelGrid::Dec(char& val)
{
  if( val == VAL_UNKNOWN)
    val = std::max(VAL_EVEN - dec_mag_, VAL_MIN);
  else if(val - dec_mag_ > VAL_MIN) 
    val -= dec_mag_;
  else 
    val = VAL_MIN;
}


bool VoxelGrid::Allocate(uint3 new_dim, int3 o)
{
	if( new_dim.x == dim_.x && new_dim.y == dim_.y && new_dim.z == dim_.z
     && o.x == origin_.x && o.y == origin_.y && o.z == origin_.z)
		return false;
	else{
		array_type new_map(boost::extents[new_dim.x][new_dim.y][new_dim.z]);
		array_rgb new_map_rgb;		 
		if(use_rgbd_)
			new_map_rgb.resize(boost::extents[new_dim.x][new_dim.y][new_dim.z]);
		for(int l = o.x; l < o.x + (int)new_dim.x; l++ )
			for(int w = o.y; w < o.y + (int)new_dim.y; w++ )
				for(int h = o.z; h < o.z + (int)new_dim.z; h++ ){
					if( l < origin_.x || w < origin_.y || h < origin_.z ||
							l >= origin_.x + (int)dim_.x || w >= origin_.y + (int)dim_.y || h >= origin_.z + (int)dim_.z)
					{	
						new_map[l - o.x][w - o.y][h - o.z] = VAL_UNKNOWN;
						if(use_rgbd_)
							new_map_rgb[l - o.x][w - o.y][h - o.z] = RGB_UNKNOWN;
					}
					else
					{
						new_map[l - o.x][w - o.y][h - o.z] = map[l - origin_.x][w - origin_.y][h - origin_.z];
						if(use_rgbd_)
							new_map_rgb[l - o.x][w - o.y][h - o.z] = map_rgb[l - origin_.x][w - origin_.y][h - origin_.z];
					}
				}
		map.resize(boost::extents[new_dim.x][new_dim.y][new_dim.z]);
		map = new_map;
		if(use_rgbd_){
			map_rgb.resize(boost::extents[new_dim.x][new_dim.y][new_dim.z]);
			map_rgb = new_map_rgb;
		}
		dim_.x = new_dim.x;
		dim_.y = new_dim.y;
		dim_.z = new_dim.z;
		origin_.x = o.x;
		origin_.y = o.y;
		origin_.z = o.z;
		return true;
	}
}

sensor_msgs::PointCloud2 VoxelGrid::TransformCloud(const sensor_msgs::PointCloud2& cloud, const Eigen::Affine3d& TF)
{
  PCLPointCloud transformed_cloud; 
  PCLPointCloud cloud_xyzrgb;
  pcl::fromROSMsg(cloud, cloud_xyzrgb);
  pcl::transformPointCloud (cloud_xyzrgb, transformed_cloud,  TF);
  sensor_msgs::PointCloud2 cloud_output;
  pcl::toROSMsg(transformed_cloud, cloud_output);
  return cloud_output; 
}


sensor_msgs::PointCloud2 VoxelGrid::TransformCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud, const Eigen::Affine3d& TF)
{
  PCLPointCloud transformed_cloud; 
  PCLPointCloud cloud_xyzrgb;
  pcl::fromROSMsg(*cloud, cloud_xyzrgb);
  pcl::transformPointCloud (cloud_xyzrgb, transformed_cloud,  TF);
  sensor_msgs::PointCloud2 cloud_output;
  pcl::toROSMsg(transformed_cloud, cloud_output);
  return cloud_output; 
}

void VoxelGrid::AddBeam(int3 start, int3 end, rgb3 color, boost::unordered_set<int> &ignore)
{
	char* voxel;
	rgb3* rgb;
	if(GetVoxel(end, voxel, rgb)){
    int3 diff = end - start;
    float end_dist_sq = (diff.x * diff.x + diff.y * diff.y) * res_xy_ * res_xy_ + diff.z * diff.z * res_z_ * res_z_;
    if (end_dist_sq < rgbd_max_range_ * rgbd_max_range_) {
      Inc(*voxel);
      if(use_rgbd_)
        AddRGB(*rgb, color);
    }

		if(use_ray_casting_){
			int max_diff = std::abs(diff.x);
			double3 step;

			if(std::abs(diff.y) >= max_diff)
				max_diff = std::abs(diff.y);
			if(std::abs(diff.z) >= max_diff)
				max_diff = std::abs(diff.z);

			step.x = (double) diff.x / max_diff;
			step.y = (double) diff.y / max_diff;
			step.z = (double) diff.z / max_diff;

      float step_size_x = step.x * res_xy_;
      float step_size_y = step.y * res_xy_;
      float step_size_z = step.z * res_z_;
      float step_size = sqrt(step_size_x * step_size_x +
                             step_size_y * step_size_y +
                             step_size_z * step_size_z);
      float max_range_step = rgbd_max_range_ / step_size;
      max_diff = std::min(max_diff, (int)max_range_step);
			for(int n = 1; n < max_diff; n++)
			{
				int3 curr = start + int3(step.x*n, step.y*n, step.z*n);
        int index = curr.x  + curr.y * dim_.x + curr.z * dim_.x * dim_.y;
				if(GetVoxel(curr, voxel, rgb)) {
          if (ignore.find(index) == ignore.end()) {
            Dec(*voxel);
            ignore.insert(index);
          }
        }
			}
		}
	}
}

bool VoxelGrid::AddPointCloud(const nav_msgs::Odometry& odom, const sensor_msgs::PointCloud2::ConstPtr& cloud, int mode)
{
	int3 robot;
	robot.x = odom.pose.pose.position.x / res_xy_;
	robot.y = odom.pose.pose.position.y / res_xy_;
	robot.z = odom.pose.pose.position.z / res_z_;

  Eigen::Affine3d TF;
	tf::poseMsgToEigen (odom.pose.pose, TF);
	sensor_msgs::PointCloud2 cloud_w;
	if(mode == 0)
		cloud_w = TransformCloud(cloud, TF * Rcb_);
	else if(mode == 1)
		cloud_w = TransformCloud(cloud, TF * Rlb_);

	sensor_msgs::PointCloud cloud1;
	sensor_msgs::convertPointCloud2ToPointCloud(cloud_w, cloud1);
  occupied_voxels.clear();

  std::vector<bool> use_beam(cloud1.points.size(), true);
  ros::WallTime start = ros::WallTime::now();
	for(int n = 0; n < (int) cloud1.points.size(); n++) {
		int3 end;
		end.x = cloud1.points[n].x / res_xy_;
		end.y = cloud1.points[n].y / res_xy_;
    end.z = cloud1.points[n].z / res_z_;
    int index = end.x  + end.y * dim_.x + end.z * dim_.x * dim_.y;
    std::pair<boost::unordered_set<int>::iterator, bool> result;
    result = occupied_voxels.insert(index);
    if (!result.second) {
      use_beam.at(n) = false;
    }
	}
	for(int n = 0; n < (int) cloud1.points.size(); n++)
	{
    if (!use_beam.at(n)) {
      continue;
    }
		int3 end;
		end.x = cloud1.points[n].x / res_xy_;
		end.y = cloud1.points[n].y / res_xy_;
    end.z = cloud1.points[n].z / res_z_;
		rgb3 color = rgb3(cloud1.channels[0].values[n]); 
		AddBeam(robot, end, color, occupied_voxels);
	}

	return true;
}

void VoxelGrid::MergeMap(const nav_msgs::Odometry& odom, const array_type& grid, const uint3& dim, const int3 o)
{
  Eigen::Affine3d TF;      
  geometry_msgs::Pose pose;
	pose.position.x = odom.pose.pose.position.x / res_xy_;
	pose.position.y = odom.pose.pose.position.y / res_xy_;
	pose.position.z = odom.pose.pose.position.z / res_z_;
  pose.orientation.w = odom.pose.pose.orientation.w;
  pose.orientation.x = odom.pose.pose.orientation.x;
  pose.orientation.y = odom.pose.pose.orientation.y;
  pose.orientation.z = odom.pose.pose.orientation.z;

  tf::poseMsgToEigen (pose, TF);
  for(int l = 0; l < (int) dim.x; l++)
    for(int w = 0; w < (int) dim.y; w++)
      for(int h = 0; h < (int) dim.z; h++)
      {
        if(grid[l][w][h] == 0)
          continue;
        else{
          int3 p;
          Eigen::Vector3d _o(l + o.x , w + o.y, h + o.z);
          Eigen::Vector3d _p = TF*_o;
          p.x = _p[0] - origin_.x;
          p.y = _p[1] - origin_.y;
          p.z = _p[2] - origin_.z;
          if(p.x >= 0 && p.x < (int) dim_.x &&
              p.y >= 0 && p.y < (int) dim_.y &&
              p.z >= 0 && p.z < (int) dim_.z)
          {
            int sum = map[p.x][p.y][p.z] + grid[l][w][h];
            if(sum > VAL_MAX)
              sum = VAL_MAX;
            else if(sum < VAL_MIN)
              sum = VAL_MIN;

            map[p.x][p.y][p.z] = sum;
          }
        }
      }

}



bool VoxelGrid::AddPointCloud(const Eigen::Affine3d& TF, const sensor_msgs::PointCloud2& cloud, int mode)
{
	int3 robot;
	robot.x = TF.translation()[0] / res_xy_;
	robot.y = TF.translation()[1] / res_xy_;
	robot.z = TF.translation()[2] / res_z_;

	sensor_msgs::PointCloud2 cloud_w;
	if(mode == 0)
		cloud_w = TransformCloud(cloud, TF * Rcb_);
	else if(mode == 1)
		cloud_w = TransformCloud(cloud, TF * Rlb_);

	sensor_msgs::PointCloud cloud1;
	sensor_msgs::convertPointCloud2ToPointCloud(cloud_w, cloud1);
  occupied_voxels.clear();

  std::vector<bool> use_beam(cloud1.points.size(), true);
  ros::WallTime start = ros::WallTime::now();
	for(int n = 0; n < (int) cloud1.points.size(); n++) {
		int3 end;
		end.x = cloud1.points[n].x / res_xy_;
		end.y = cloud1.points[n].y / res_xy_;
    end.z = cloud1.points[n].z / res_z_;
    int index = end.x  + end.y * dim_.x + end.z * dim_.x * dim_.y;
    std::pair<boost::unordered_set<int>::iterator, bool> result;
    result = occupied_voxels.insert(index);
    if (!result.second) {
      use_beam.at(n) = false;
    }
	}
	for(int n = 0; n < (int) cloud1.points.size(); n++)
	{
    if (!use_beam.at(n)) {
      continue;
    }
		int3 end;
		end.x = cloud1.points[n].x / res_xy_;
		end.y = cloud1.points[n].y / res_xy_;
    end.z = cloud1.points[n].z / res_z_;
		rgb3 color = rgb3(cloud1.channels[0].values[n]); 
		AddBeam(robot, end, color, occupied_voxels);
	}

	return true;
}

explore3d::VoxelGrid VoxelGrid::ConvertToExplore3d()
{
  if(VAL_UNKNOWN != -1)
  {
    ROS_ERROR("Use the wrong mode for map!!");
  }
 
	explore3d::VoxelGrid explore_map;
	explore_map.header.stamp = ros::Time::now();
	explore_map.header.frame_id = std::string("map");
  explore_map.resolution = res_xy_;
	explore_map.origin.x = origin_.x * res_xy_; 
	explore_map.origin.y = origin_.y * res_xy_; 
	explore_map.origin.z = origin_.z * res_z_;
  explore_map.num_x = dim_.x;
  explore_map.num_y = dim_.y;
  explore_map.num_z = dim_.z;

	for(unsigned int h = 0; h < dim_.z; h++)
		for(unsigned int w = 0; w < dim_.y; w++)
			for(unsigned int l = 0; l < dim_.x; l++)
			{
				if(map[l][w][h] == VAL_UNKNOWN)
					explore_map.probability.push_back(VAL_UNKNOWN);
				else
					explore_map.probability.push_back((float) map[l][w][h] / VAL_MAX);
			}
  return explore_map;
}

sensor_msgs::PointCloud VoxelGrid::GetMap()
{
	sensor_msgs::PointCloud cloud;
	cloud.header.stamp = ros::Time::now();
	cloud.header.frame_id = std::string("/map");
	cloud.channels.resize(3);
	cloud.channels[0].name = std::string("r");
	cloud.channels[1].name = std::string("g");
	cloud.channels[2].name = std::string("b");

	for(int l = origin_.x; l < origin_.x + (int)dim_.x; l++)
		for(int w = origin_.y; w < origin_.y + (int)dim_.y; w++)
			for(int h = origin_.z; h < origin_.z + (int)dim_.z; h++)
				if(map[l - origin_.x][w - origin_.y][h - origin_.z] >= occ_thr_)
				{
					geometry_msgs::Point32 p;
					p.x = l * res_xy_;
					p.y = w * res_xy_;
					p.z = h * res_z_;
					cloud.points.push_back(p);
					if(use_rgbd_){
						cloud.channels[0].values.push_back(map_rgb[l - origin_.x][w - origin_.y][h - origin_.z].r / 255.0);
						cloud.channels[1].values.push_back(map_rgb[l - origin_.x][w - origin_.y][h - origin_.z].g / 255.0);
						cloud.channels[2].values.push_back(map_rgb[l - origin_.x][w - origin_.y][h - origin_.z].b / 255.0);
					}
				}
  return cloud;
}
