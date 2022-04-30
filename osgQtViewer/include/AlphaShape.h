#pragma once
#include <Windows.h>
#include <math.h>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <osg/Array>
#include <bitset>

#include "struct.h"

using namespace std;
typedef std::vector<osg::Vec3> PointV3List;

//检测边界线的信息
struct Edge {
	osg::Vec2 point_A;
	osg::Vec2 point_B;

	Edge() = default;
	Edge(const osg::Vec2 & pA, const osg::Vec2 & pB) {
		this->point_A = pA;
		this->point_B = pB;
	}
};

//检测圆的信息
struct Circle {
	osg::Vec2 m_center;
	float m_radius;
	int size;

	Circle() = default;

	Circle(const osg::Vec2 & center, float radius) {
		this->m_center = center;
		this->m_radius = radius;
	}
};

//单一网格的信息
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
};

//单个二维网格类
class SingleGrid2D {
public:
	//根据网格长宽生成
	SingleGrid2D(float Grid_X, float Grid_Y);

	//根据网格信息生成
	SingleGrid2D(GridInfo curGrid);

public:
	//网格内含有的点列表
	PointV3List PointList;

	//网格内的点数量
	unsigned int cur_PointNum;

	//当前网格与邻域网格的平均中心点的向量列表
	std::vector<osg::Vec2> VectorList;

	//是否含有点
	bool hasPoint;

	//当前网格内点的高程差值
	float heightDifference;

	//当前网格的周围八邻域网格是否均含有点，即表示当前网格是否为内部网格
	bool nearByGridAllWithpoint;

	//是否被检测过
	bool hasDetected;

	//是否为平滑网格
	bool isSmoothGrid;

	//当前网格的平滑度，越平滑值越小
	int SmoothDegree;

	//当前网格内点的平均几何中心点
	osg::Vec2 CenterPoint;

	//当前网格与邻域网格的合向量
	osg::Vec2 curVectorGrid;

	//邻域网格的ID值列表
	std::vector<int> connectGridID_List;

	//当前网格的网格信息
	GridInfo curGridInfo;
};

//二维格网类
class GridNet {
public:
	//根据点云生成当前的二维网格
	GridNet(PointV3List Point_List);

public:
	//所有点的列表
	PointV3List Points_List;

	//所有二维网格列表
	std::vector<SingleGrid2D*> Grid_list;

	//原始点云的范围信息
	point_MAXMIN* pointMMM;

	//网格的长宽
	float Grid_X;
	float Grid_Y;

	//含有点的网格数量
	int GridWithPoint_Num;

	//边界网格数量，即非内部网格
	int GridOutside_Num;

	//网格数量以及行列数
	unsigned int Grid_Num;
	unsigned int Col_Num;
	unsigned int Row_Num;

	//单一网格内含有点的最大、最小值
	int MaxPointNum_InOneGrid;
	int MinPointNum_InOneGrid;

	//网格的最大、最小合向量距离
	float MaxVector_Grid;
	float MinVector_Grid;

public:
	//根据设定的网格长宽生成二维格网
	void buildNetBySize(float SizeX, float SizeY);

	//根据设定的网格行列数生成二维格网
	void buildNetByNum(int RowNum, int ColNum);

	//判断当前某点是否处于某一网格中
	bool isPointInGrid(osg::Vec3 curPoint, SingleGrid2D *test_Grid);

	//获取网格的点的平均中心点
	void getCenterPoint();

	//获取外部网格的合向量
	void getVectorOfOutSideGrid();

	//计算外部网格的平滑度
	void DetectSmoothForOutSideGrid();

	//根据网格的行列号获取该网格指针
	SingleGrid2D* getGridByRowAndCol(int RowID, int ColID);

	//检测二维格网中网格的连通性，从而区分外部和内部网格
	void detectGridWithConnection();
};

//Alpha Shap算法
class AlphaShape
{
public:
	AlphaShape(std::vector<osg::Vec2> point_list);
	AlphaShape();

public:
	//根据设置的半径，检测点云边界线，默认的常规算法，将判断所有点，效率较慢
	void Detect_Shape_line(float radius);

	//根据生成的网格和半径，以及邻域点云检测边界线，传递值为点列表
	void Detect_Shape_line_by_Grid(std::vector<osg::Vec2> near_point_list, std::vector<osg::Vec2> detect_point_list, float radius);

	//根据生成的网格和半径，以及邻域点云检测边界线，传递值为网格列表
	void Detect_Shape_line_by_Grid_New(SingleGrid2D* centerGrid, std::vector<SingleGrid2D*> nearGrid_Lis, float radius);

	//根据两点计算集合距离
	float Distance_point(osg::Vec2 pointA, osg::Vec2 pointB);

	//根据生成的网格和半径检测边界
	void Detect_Shape_By_GridNet(GridNet* curGridNet, float radius);

	//根据生成的网格和半径检测边界(新)
	void Detect_Shape_By_GridNet_New(GridNet* curGridNet, float radius);

	//根据包裹圆，通过落点数量和点的分布角度，检测边界点
	void Detect_Shape_By_SingleCirlce(GridNet* curGridNet, float radius, int pointNum);

	//根据包裹圆，仅仅通过落点数量，检测边界点
	void Detect_Shape_By_PackCirlce(GridNet* curGridNet, float radius, int pointMaxNum);


public:
	//用于检测的点列表
	std::vector<osg::Vec2> m_points;

	//轮廓点
	std::vector<osg::Vec2> m_shape_points;

	//轮廓点对应的点ID
	std::vector<int> m_shape_id;

	//轮廓线列表
	std::vector<Edge> m_edges;

	//检测圆列表
	std::vector<Circle> m_circles;

	//初始的检测半径
	float m_radius;
	
	//符合检测半径的点对比例
	float point_pair_scale;
};
