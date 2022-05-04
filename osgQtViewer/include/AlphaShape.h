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
#include <climits>
#include <set>
#include <mutex>
#include <thread>

#include "struct.h"

using namespace std;
typedef std::vector<osg::Vec2> PointV2List;

//���߽��ߵ���Ϣ
struct Edge {
	osg::Vec2 point_A;
	osg::Vec2 point_B;

	Edge() = default;
	Edge(const osg::Vec2 & pA, const osg::Vec2 & pB) {
		this->point_A = pA;
		this->point_B = pB;
	}

	bool operator == (const Edge & other) const {
		return ((((*this).point_A - other.point_A).length() < 0.000001) &&
			(((*this).point_B - other.point_B).length() < 0.000001));
	}

	bool operator <(const Edge & other) const{
		return (((*this).point_A - (*this).point_B).length() < (other.point_A - other.point_B).length());
	}

	bool operator >(const Edge & other) const {
		return (((*this).point_A - (*this).point_B).length() > (other.point_A - other.point_B).length());
	}
};

//���Բ����Ϣ
struct Circle {
	osg::Vec2 m_center;
	float m_radius;
	int size;

	Circle() = default;

	Circle(const osg::Vec2 & center, float radius) {
		this->m_center = center;
		this->m_radius = radius;
	}

	bool operator == (const Circle & other) const {
		return ((((*this).m_center - other.m_center).length() < 0.000001) &&
			(((*this).m_radius - other.m_radius) < 0.000001));
	}

	bool operator <(const Circle & other) const {
		return ((*this).m_center.length() < other.m_center.length());
	}

	bool operator >(const Circle & other) const {
		return ((*this).m_center.length() > other.m_center.length());
	}
};

//��һ�������Ϣ
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

//������ά������
class SingleGrid2D {
public:
	//�������񳤿�����
	SingleGrid2D(float Grid_X, float Grid_Y);

	//����������Ϣ����
	SingleGrid2D(const GridInfo &curGrid);

public:
	//�����ں��еĵ��б�
	PointV2List PointList;

	//�����ڵĵ�����
	unsigned int cur_PointNum;

	//��ǰ���������������ƽ�����ĵ�������б�
	std::vector<osg::Vec2> VectorList;

	//�Ƿ��е�
	bool hasPoint;

	//��ǰ�����ڵ�ĸ̲߳�ֵ
	float heightDifference;

	//��ǰ�������Χ�����������Ƿ�����е㣬����ʾ��ǰ�����Ƿ�Ϊ�ڲ�����
	bool nearByGridAllWithpoint;

	//�Ƿ񱻼���
	bool hasDetected;

	//�Ƿ�Ϊƽ������
	bool isSmoothGrid;

	//��ǰ�����ƽ���ȣ�Խƽ��ֵԽС
	int SmoothDegree;

	//��ǰ�����ڵ��ƽ���������ĵ�
	osg::Vec2 CenterPoint;

	//��ǰ��������������ĺ�����
	osg::Vec2 curVectorGrid;

	//���������IDֵ�б�
	std::vector<int> connectGridID_List;

	//��ǰ�����������Ϣ
	GridInfo curGridInfo;
};

//��ά������
class GridNet {
public:
	//���ݵ������ɵ�ǰ�Ķ�ά����
	GridNet(const PointV2List & pList);

public:
	//���е���б�
	PointV2List Points_List;

	//���ж�ά�����б�
	std::vector<SingleGrid2D*> Grid_list;

	//ԭʼ���Ƶķ�Χ��Ϣ
	point_MAXMIN* pointMMM;

	//����ĳ���
	float Grid_X;
	float Grid_Y;

	//���е����������
	int GridWithPoint_Num;

	//�߽����������������ڲ�����
	int GridOutside_Num;

	//���������Լ�������
	unsigned int Grid_Num;
	unsigned int Col_Num;
	unsigned int Row_Num;

	//��һ�����ں��е�������Сֵ
	int MaxPointNum_InOneGrid;
	int MinPointNum_InOneGrid;

	//����������С����������
	float MaxVector_Grid;
	float MinVector_Grid;

public:
	//�����趨�����񳤿����ɶ�ά����
	void buildNetBySize(float SizeX, float SizeY);

	//�����趨���������������ɶ�ά����
	void buildNetByNum(int RowNum, int ColNum);

	//�жϵ�ǰĳ���Ƿ���ĳһ������
	bool isPointInGrid(const osg::Vec2 &curPoint, SingleGrid2D *test_Grid);

	//��ȡ����ĵ��ƽ�����ĵ�
	[[deprecated]] void getCenterPoint();

	//��ȡ�ⲿ����ĺ�����
	void getVectorOfOutSideGrid();

	//�����ⲿ�����ƽ����
	void DetectSmoothForOutSideGrid();

	//������������кŻ�ȡ������ָ��
	SingleGrid2D* getGridByRowAndCol(int RowID, int ColID);

	//����ά�������������ͨ�ԣ��Ӷ������ⲿ���ڲ�����
	void detectGridWithConnection();
};

//Alpha Shap�㷨
class AlphaShape {
public:
	AlphaShape(const std::vector<osg::Vec2> &point_list);
	AlphaShape(GridNet * curGridNet);
	~AlphaShape();

public:
	//�������õİ뾶�������Ʊ߽��ߣ�Ĭ�ϵĳ����㷨�����ж����е㣬Ч�ʽ���
	void Detect_Shape_line(float radius);

	//���̼߳��
	void Detect_Alpha_Shape_by_Grid_Multi_Thread(float radius, int threadNum = 2);

	//�������ɵ������Ĭ���㷨�����Ż�����������������ڵĵ㣬�����ж����еĵ�
	void Detect_Alpha_Shape_by_Grid(float radius);

	//�������ɵ�����Ͱ뾶���Լ�������Ƽ��߽��ߣ�����ֵΪ���б�
	void Detect_Shape_line_by_Grid(std::vector<osg::Vec2> near_point_list, std::vector<osg::Vec2> detect_point_list, float radius);

	//�������ɵ�����Ͱ뾶���Լ�������Ƽ��߽��ߣ�����ֵΪ�����б�
	void Detect_Shape_line_by_Grid_New(SingleGrid2D* centerGrid, std::vector<SingleGrid2D*> nearGrid_List, float radius);

	//����������㼯�Ͼ���
	//float Distance_point(osg::Vec2 pointA, osg::Vec2 pointB);

	//�������ɵ�����Ͱ뾶���߽�
	void Detect_Shape_By_GridNet(GridNet* curGridNet, float radius);

	//�������ɵ�����Ͱ뾶���߽�(��)
	void Detect_Shape_By_GridNet_New(GridNet* curGridNet, float radius);

	//���ݰ���Բ��ͨ����������͵�ķֲ��Ƕȣ����߽��
	void Detect_Shape_By_SingleCirlce(GridNet* curGridNet, float radius, int pointNum);

	//���ݰ���Բ������ͨ��������������߽��
	void Detect_Shape_By_PackCirlce(GridNet* curGridNet, float radius, int pointMaxNum);
	
public:
	//���ڼ��ĵ��б�
	std::vector<osg::Vec2> m_points;

	//���ڼ��ĸ���ָ��
	GridNet * m_gridNet;

	//������
	std::vector<osg::Vec2> m_shape_points;

	//�������Ӧ�ĵ�ID
	std::vector<int> m_shape_id;

	//�������б�
	std::vector<Edge> m_edges;

	//���Բ�б�
	std::vector<Circle> m_circles;

	//��ʼ�ļ��뾶
	float m_radius;
	
	//���ϼ��뾶�ĵ�Ա���
	float point_pair_scale;

	int m_point_pair_N;
};
