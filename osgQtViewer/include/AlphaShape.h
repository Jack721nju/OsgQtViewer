/* Copyright© 2022 Jack721 */
#pragma once
#include <Windows.h>
#include <osg/Array>

#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <bitset>
#include <climits>
#include <set>
#include <mutex>
#include <thread>

#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/concave_hull.h>

#include "./struct.h"

// 检测边界线的信息
struct Edge {
	osg::Vec2 point_A;
	osg::Vec2 point_B;

	Edge() = default;
	Edge(const osg::Vec2 & pA, const osg::Vec2 & pB) {
		this->point_A = pA;
		this->point_B = pB;
	}

	bool operator <(const Edge & other) const{
		if ((((*this).point_A - other.point_A).length() < 0.000001) &&
			(((*this).point_B - other.point_B).length() < 0.000001)) {
			return false;
		} else {
			if (((*this).point_A - other.point_A).length() > 0.000001) {
				return (*this).point_A.length() < other.point_A.length();
			} else {
				return (*this).point_B.length() < other.point_B.length();
			}
		}
	}
};

// 检测圆的信息
struct Circle {
	osg::Vec2 m_center;
	float m_radius;
	int m_size;

	Circle() = default;

	Circle(const osg::Vec2 & center, float radius) {
		this->m_center = center;
		this->m_radius = radius;
	}

	Circle(const osg::Vec2 & center, float radius, int size) {
		this->m_center = center;
		this->m_radius = radius;
		this->m_size = size;
	}

	bool operator <(const Circle & other) const {
		if ((((*this).m_center - other.m_center).length() < 0.000001) &&
			(((*this).m_radius - other.m_radius) < 0.000001)) {
			return false;
		} else {
			if (((*this).m_center - other.m_center).length() > 0.000001) {
				return ((*this).m_center.length() < other.m_center.length());
			} else {
				return ((*this).m_radius < other.m_radius);
			}
       }
	}
};

// 单一网格的信息
struct GridInfo {
	float Min_X;
	float Min_Y;
	float Max_X;
	float Max_Y;

	float Size_X;
	float Size_Y;

	int m_ID;
	int m_Col;
	int m_Row;

	GridInfo() = default;
};

// 单个二维网格类
class SingleGrid2D {
 public:
	// 根据网格长宽生成
	SingleGrid2D(float Grid_X, float Grid_Y);

	// 根据网格信息生成
	explicit SingleGrid2D(const GridInfo &curGrid);

 public:
	// 网格内含有的点列表
	std::vector<osg::Vec2> PointList;

	// 网格内的点数量
	unsigned int cur_PointNum;

	// 当前网格与邻域网格的平均中心点的向量列表
	std::vector<osg::Vec2> VectorList;

	// 是否含有点
	bool hasPoint;

	// 当前网格内点的高程差值
	float heightDifference;

	// 当前网格的周围八邻域网格是否均含有点，即表示当前网格是否为内部网格
	bool nearByGridAllWithpoint;

	// 是否被检测过
	bool hasDetected;

	// 是否为平滑网格
	bool isSmoothGrid;

	// 当前网格的平滑度，越平滑值越小
	int SmoothDegree;

	// 当前网格内点的平均几何中心点
	osg::Vec2 CenterPoint;

	// 当前网格与邻域网格的合向量
	osg::Vec2 curVectorGrid;

	// 邻域网格的ID值列表
	std::vector<int> connectGridID_List;

	// 当前网格的网格信息
	GridInfo curGridInfo;
};

// 二维格网类
class GridNet {
 public:
	// 根据点云生成当前的二维网格
	explicit GridNet(const std::vector<osg::Vec2> & pList);

 public:
	// 所有点的列表
	std::vector<osg::Vec2> Points_List;

	// 所有二维网格列表
	std::vector<SingleGrid2D*> Grid_list;

	// 原始点云的范围信息
	point_MAXMIN* pointMMM;

	//网格的长宽
	float Grid_X;
	float Grid_Y;

	// 含有点的网格数量
	int GridWithPoint_Num;

	// 边界网格数量，即非内部网格
	int GridOutside_Num;

	// 网格数量以及行列数
	unsigned int Grid_Num;
	unsigned int Col_Num;
	unsigned int Row_Num;

	// 单一网格内含有点的最大、最小值
	int MaxPointNum_InOneGrid;
	int MinPointNum_InOneGrid;

	// 网格的最大、最小合向量距离
	float MaxVector_Grid;
	float MinVector_Grid;

 public:
	// 根据设定的网格长宽生成二维格网
	void buildNetBySize(float SizeX, float SizeY);

	// 根据设定的网格行列数生成二维格网
	void buildNetByNumOld(int RowNum, int ColNum);

	// 根据设定的网格行列数生成二维格网,快速
	void buildNetByNum(int RowNum, int ColNum);

	// 判断当前某点是否处于某一网格中
	bool isPointInGrid(const osg::Vec2 &curPoint, SingleGrid2D *test_Grid);

	// 获取网格的点的平均中心点
	[[deprecated]] void getCenterPoint();

	// 获取外部网格的合向量
	void getVectorOfOutSideGrid();

	// 计算外部网格的平滑度
	void DetectSmoothForOutSideGrid();

	// 根据网格的行列号获取该网格指针
	SingleGrid2D* getGridByRowAndCol(int RowID, int ColID);

	// 检测二维格网中网格的连通性，从而区分外部和内部网格
	void detectGridWithConnection();
};

// Alpha Shap算法
class AlphaShape {
 public:
	explicit AlphaShape(const std::vector<osg::Vec2> &point_list);
	explicit AlphaShape(GridNet * curGridNet);
	explicit AlphaShape(pcl::PointCloud<pcl::PointXYZ>::Ptr project2DPoints);
	~AlphaShape();

 public:
	// 根据设置的半径，检测点云边界线，默认的常规算法，将判断所有点，效率较慢
	void Detect_Shape_line(float radius);

	//通过PCL构建kd树，加速搜索半径方位内的最近邻点
	void Detect_Alpah_Shape_FLANN(float radius);

	//通过PCL构建kd树，FLANN加速搜索半径方位内的最近邻点，利用多线程并行处理进一步提升速度，后续可考虑结合二维格网，先筛选出必要检测的边界点，剔除内部点集
	void Detect_Alpah_Shape_FLANN_Multi_Thread(float radius, int threadNum = 4);

	void setPclPointPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr project2DPoints) {
		m_projectPcl2DPoints = project2DPoints;
	}

	// 多线程检测
	void Detect_Alpha_Shape_by_Grid_Multi_Thread(float radius, int threadNum = 4);

	// pcl concave hull 方法获取点云凹包边界
	void Detect_Shape_by_PCl_Concave_Hull(float radius);

	// 根据生成的网格对默认算法进行优化，检测邻域域网格内的点，并不判断所有的点
	void Detect_Alpha_Shape_by_Grid(float radius);

	// 根据生成的网格和半径，以及邻域点云检测边界线，传递值为网格列表
	void Detect_Shape_line_by_Grid_New(float radius, const std::vector<SingleGrid2D*> & allGridList);

	// 根据生成的网格和半径检测边界(新)
	void Detect_Shape_By_GridNet_New(float radius);

	// 根据包裹圆，通过落点数量和点的分布角度，检测边界点
	void Detect_Shape_By_SingleCirlce(GridNet* curGridNet, float radius, int pointNum);

	// 根据包裹圆，仅仅通过落点数量，检测边界点
	void Detect_Shape_By_PackCirlce(GridNet* curGridNet, float radius, int pointMaxNum);

 public:
	// 用于检测的点列表
	std::vector<osg::Vec2> m_points;

	// 用于检测的PCL点列表
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_projectPcl2DPoints;

	// 用于检测的格网指针
	GridNet * m_gridNet{nullptr};

	// 轮廓点
	std::vector<osg::Vec2> m_shape_points;

	// 轮廓点对应的点ID
	std::vector<int> m_shape_id;

	// 轮廓线列表
	std::vector<Edge> m_edges;

	// 检测圆列表
	std::vector<Circle> m_circles;

	// 初始的检测半径
	float m_radius;

	// 符合检测半径的点对比例
	float point_pair_scale;

	size_t m_point_pair_N;
	
	size_t m_detectAllNum;
};
