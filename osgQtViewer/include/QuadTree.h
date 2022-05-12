#pragma once

#include <QPoint>
#include <vector>

#include "struct.h"

//定义八叉树节点类
class QuadTreeNode {
public:
	QuadTreeNode(
		point2D_MAXMIN curSize,
		int depth = 0,
		QuadTreeNode * top_left_Node = nullptr,
		QuadTreeNode * top_right_Node = nullptr,
		QuadTreeNode * bottom_left_Node = nullptr,
		QuadTreeNode * bottom_right_Node = nullptr
	) : m_depth(depth), m_XY_Size(curSize),
		m_top_left(top_left_Node), m_top_right(top_right_Node), m_bottom_left(bottom_left_Node), m_bottom_right(bottom_right_Node), m_point_num(0) {}

	~QuadTreeNode() = default;

public:
	//节点深度
	int m_depth;

	//节点坐标
	point2D_MAXMIN m_XY_Size;

	int m_tree_Index;
	int m_point_num;

	bool m_isSonNode{false};

	std::vector<QPointF> point_list;

	QuadTreeNode * m_top_left;
	QuadTreeNode * m_top_right;
	QuadTreeNode * m_bottom_left;
	QuadTreeNode * m_bottom_right;

public:
	static void createQuadTree(QuadTreeNode* rootNode, int treeDepth, const std::vector<QPointF> &point_list, const point2D_MAXMIN &curSize);
	
	static void getMaxDepQuadNode(QuadTreeNode* curNode, std::vector<QuadTreeNode*> &node_list);

};
