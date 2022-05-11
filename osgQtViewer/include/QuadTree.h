#pragma once

#include <QPoint>
#include <vector>

#include "struct.h"

//����˲����ڵ���
class QuadTreeNode {
public:
	QuadTreeNode(
		int depth,
		point2D_MAXMIN curSize,
		QuadTreeNode * top_left_Node = nullptr,
		QuadTreeNode * top_right_Node = nullptr,
		QuadTreeNode * bottom_left_Node = nullptr,
		QuadTreeNode * bottom_right_Node = nullptr
	) : m_depth(depth), m_XY_Size(curSize),
		m_top_left(top_left_Node), m_top_right(top_right_Node), m_bottom_left(bottom_left_Node), m_bottom_right(bottom_right_Node), m_point_num(0) {}

	~QuadTreeNode() = default;

private:
	//�ڵ����
	int m_depth;

	//�ڵ�����
	point2D_MAXMIN m_XY_Size;

	int m_tree_Index;
	int m_point_num;

	std::vector<QPoint> point_list;

	QuadTreeNode * m_top_left;
	QuadTreeNode * m_top_right;
	QuadTreeNode * m_bottom_left;
	QuadTreeNode * m_bottom_right;

public:
	void createQuadTree(QuadTreeNode* rootNode, int treeDepth, const std::vector<QPoint> &point_list, const point2D_MAXMIN &curSize);

};