#pragma once

#include <QPoint>
#include <vector>

#include "struct.h"

//定义四叉树节点类
class QuadTreeNode {
public:
	QuadTreeNode() : m_depth(0), m_point_num(0) {}

	~QuadTreeNode() = default;

public:
	//节点深度
	int m_depth;

	//节点坐标
	point2D_MAXMIN m_XY_Size;

	//网格中心坐标
	float m_CenterX;
	float m_CenterY;

	//网格尺寸
	float m_SizeX;
	float m_SizeY;

	int m_tree_Index;
	int m_point_num;

	bool m_isSonNode{false};

	std::vector<QPointF> point_list;

	QuadTreeNode * m_top_left{ nullptr };
	QuadTreeNode * m_top_right{ nullptr };
	QuadTreeNode * m_bottom_left{ nullptr };
	QuadTreeNode * m_bottom_right{ nullptr };

public:
	static void createQuadTree(QuadTreeNode* &rootNode, int treeDepth, const std::vector<QPointF> &point_list, float m_CenterX, float m_CenterY, float m_SizeX, float m_SizeY);
	
	static void getMaxDepQuadNode(QuadTreeNode* curNode, std::vector<QuadTreeNode*> &node_list);

	static void getAllQuadNode(QuadTreeNode* curNode, std::vector<QuadTreeNode*> &node_list);

	static point2D_MAXMIN getMinMaxXY(const std::vector<QPointF> & all_list);
};
