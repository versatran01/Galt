#pragma once
#include <ros/ros.h>
#include <boost/multi_array.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/unordered_set.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <stdio.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/transforms.h>
#include <explore3d/VoxelGrid.h>
#include <scarab_omnimapper/sensor_transform.h>

template <class T>
class Type3 {
	public:
		T x;
		T y;
		T z;

		Type3 operator+(const Type3& b)
		{
			Type3 c;
			c.x = this->x + b.x;
			c.y = this->y + b.y;
			c.z = this->z + b.z;
			return c;
		} 
		Type3 operator-(const Type3& b)
		{
			Type3 c;
			c.x = this->x - b.x;
			c.y = this->y - b.y;
			c.z = this->z - b.z;
			return c;
		} 
		Type3 operator*(const T& b)
		{
			Type3 c;
			c.x = this->x * b;
			c.y = this->y * b;
			c.z = this->z * b;
			return c;
		} 
		Type3 operator/(const T& b)
		{
			Type3 c;
			c.x = this->x / b;
			c.y = this->y / b;
			c.z = this->z / b;
			return c;
		} 

    void print() {printf("x = %.2f, y = %.2f, z = %.2f\n", x, y, z);}
    void printi() {printf("x = %d, y = %d, z = %d\n", x, y, z);}

		Type3 (T a=0, T b=0, T c=0) : 
			x(a), y(b), z(c) {}
};


struct rgb3{
	int r;
	int g;
	int b;
	int n;
	void print() {printf("r = %d, g = %d, b = %d, n=%d\n", r, g, b, n);}

	rgb3(int x0=0, int x1=0, int x2=0, int x3=1) :
		r(x0), g(x1), b(x2), n(x3) {}

	rgb3(const rgb3& c) : 
  r(c.r), g(c.g), b(c.b), n(c.n) {}

	rgb3(float rgb){
		r = *reinterpret_cast<int*>(&rgb)>>16 & 0xff;
		g = *reinterpret_cast<int*>(&rgb)>>8 & 0xff;
		b = *reinterpret_cast<int*>(&rgb) & 0xff; 
		n = 1; 
	}
	rgb3 operator+(const rgb3& c){
		rgb3 new_c;
		new_c.r = (this->n*this->r + c.n*c.r)/(this->n + c.n);
		new_c.g = (this->n*this->g + c.n*c.g)/(this->n + c.n);
		new_c.b = (this->n*this->b + c.n*c.b)/(this->n + c.n);
		new_c.n = this->n + c.n;
		return new_c;
	}
}; 


class VoxelGrid {
	public:
		typedef boost::multi_array<char,3> array_type;
		typedef boost::multi_array<rgb3,3> array_rgb;
		typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;
		typedef Type3<double> double3;
		typedef Type3<int> int3;
		typedef Type3<unsigned int> uint3;

		Eigen::Affine3d Rcb_;
		Eigen::Affine3d Rlb_;
		array_type map;
		array_rgb map_rgb;
    int3 origin_;
    uint3 dim_;

  protected:
    int3 offset_o_;
    uint3 offset_dim_;
    int occ_thr_;
    int inc_mag_, dec_mag_;
    double rgbd_max_range_;
    bool use_rgbd_;
		bool use_ray_casting_;
		double res_xy_;
		double res_z_;
    int VAL_MAX;
    int VAL_MIN;
    int VAL_EVEN;
    int VAL_UNKNOWN;

		rgb3 RGB_UNKNOWN;
    boost::thread_group clearMapThreads;
    boost::unordered_set<int> occupied_voxels;
	public:
		VoxelGrid();
		~VoxelGrid();
		void initParams(ros::NodeHandle& nh, int mode = 1);
		void SetSpace(double length, double width, double height, double origin_x, double origin_y, double origin_z);
		void SubClear(int n, int n_threads);
		void Clear(int n_threads = 1);
    void MergeSubGrid(const array_type& grid);
    void MergeSubGrid(const array_type& grid, const array_rgb& grid_rgb);

    void MergeMap(const nav_msgs::Odometry& odom, const array_type& grid, const uint3& dim, const int3 o);
    bool GetVoxel(int3 pose, char*& voxel, rgb3*& rgb);
		bool Allocate(uint3 new_dim, int3 o);
		void Inc(char& val);
		void Dec(char& val);
		sensor_msgs::PointCloud2 TransformCloud(const sensor_msgs::PointCloud2& cloud, const Eigen::Affine3d& TF);
		sensor_msgs::PointCloud2 TransformCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud, const Eigen::Affine3d& TF);
		void AddRGB(rgb3& rgb, rgb3 color);
    void AddBeam(int3 start, int3 end, rgb3 color, boost::unordered_set<int> &ignore);
		bool AddPointCloud(const Eigen::Affine3d& odom_raw, const sensor_msgs::PointCloud2& cloud, int mode = 0);
		bool AddPointCloud(const nav_msgs::Odometry& odom_raw, const sensor_msgs::PointCloud2::ConstPtr& cloud, int mode = 0);
		sensor_msgs::PointCloud GetMap();
		std_msgs::Float64MultiArray RetrieveSpace();

    explore3d::VoxelGrid ConvertToExplore3d();
};


