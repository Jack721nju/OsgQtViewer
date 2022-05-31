/* Copyright© 2022 Jack721 */
#include "QuadTree.h"
#include <math.h>
#include <vector>
#include <queue>
#include <stack>

int QuadTreeNode::minPointNumPerGrid = 10;
int QuadTreeNode::maxTreeDepth = 6;

// 获取给定点云数据的XY最大最小范围
point2D_MAXMIN QuadTreeNode::getMinMaxXY(const std::vector<QPointF> & all_list) {
	point2D_MAXMIN Max_area;
	std::vector<float> x_list, y_list, z_list;
	for (int i = 0; i < all_list.size(); ++i) {
		x_list.push_back(all_list[i].x());
		y_list.push_back(all_list[i].y());
	}
	auto xmax = max_element(begin(x_list), end(x_list));
	auto ymax = max_element(begin(y_list), end(y_list));

	auto xmin = min_element(begin(x_list), end(x_list));
	auto ymin = min_element(begin(y_list), end(y_list));

	Max_area.xmax = *xmax;
	Max_area.ymax = *ymax;
	Max_area.xmin = *xmin;
	Max_area.ymin = *ymin;

	return std::move(Max_area);
}

void QuadTreeNode::createQuadTreeBFS(QuadTreeNode* &rootNode, int treeDepth, const std::vector<QPointF> &point_list, float m_CenterX, float m_CenterY, float m_SizeX, float m_SizeY) {
	std::queue<QuadTreeNode*> nodeList;
	nodeList.emplace(rootNode);

	float halfX = m_SizeX * 0.5;
	float halfY = m_SizeY * 0.5;
	rootNode->m_XY_Size = point2D_MAXMIN(m_CenterX - halfX, m_CenterY - halfY, m_CenterX + halfX, m_CenterY + halfY);
	rootNode->m_depth = 0;
	rootNode->point_list.assign(point_list.begin(), point_list.end());

	while (!nodeList.empty()) {
		auto tempNode = nodeList.front();
		nodeList.pop();

		if (nullptr == tempNode) {
			continue;
		}

		const auto &xySize = tempNode->m_XY_Size;
		const auto & parentNode = tempNode->parentNode;
		if (parentNode) {
			for (const auto & curP : parentNode->point_list) {
				float xx = curP.x();
				float yy = curP.y();

				//该点坐标超出了当前网格的坐标范围
				if (xx < (xySize.xmin) || xx >(xySize.xmax) || yy < (xySize.ymin) || yy >(xySize.ymax)) {
					continue;
				}
				//将点存入此节点的数据列表中
				tempNode->point_list.emplace_back(curP);
			}
		}

		tempNode->m_point_num = static_cast<int>(tempNode->point_list.size());
		tempNode->m_isSonNode = false;

		//四叉树划分中止条件，网格内点达到最小值
		if (tempNode->m_point_num < minPointNumPerGrid) {
			tempNode->m_isSonNode = true;
		}

		//四叉树划分中止条件，网格达到最大深度
		auto curTreeDepth = tempNode->m_depth;
		if (curTreeDepth > maxTreeDepth) {
			tempNode->m_isSonNode = true;
		}		

		if (parentNode) {
			if (parentNode->m_point_num == tempNode->m_point_num) {
				tempNode->m_isSonNode = false;
			}
		}

		if (tempNode->m_isSonNode) {
			continue;
		}

		tempNode->m_bottom_left = new QuadTreeNode();
		tempNode->m_bottom_right = new QuadTreeNode();
		tempNode->m_top_left = new QuadTreeNode();
		tempNode->m_top_right = new QuadTreeNode();

		tempNode->m_bottom_left->parentNode = tempNode;
		tempNode->m_bottom_right->parentNode = tempNode;
		tempNode->m_top_left->parentNode = tempNode;
		tempNode->m_top_right->parentNode = tempNode;

		float soncenterX = (xySize.xmax + xySize.xmin) * 0.5;
		float soncenterY = (xySize.ymax + xySize.ymin) * 0.5;

		float sonhalfX = (xySize.xmax - xySize.xmin) * 0.5;
		float sonhalfY = (xySize.ymax - xySize.ymin) * 0.5;

		tempNode->m_bottom_left->m_XY_Size = point2D_MAXMIN(soncenterX - sonhalfX, soncenterY - sonhalfY, soncenterX, soncenterY);
		tempNode->m_bottom_right->m_XY_Size = point2D_MAXMIN(soncenterX, soncenterY - sonhalfY, soncenterX + sonhalfX, soncenterY);
		tempNode->m_top_left->m_XY_Size = point2D_MAXMIN(soncenterX - sonhalfX, soncenterY, soncenterX, soncenterY + sonhalfY);
		tempNode->m_top_right->m_XY_Size = point2D_MAXMIN(soncenterX, soncenterY, soncenterX + sonhalfX, soncenterY + sonhalfY);

		tempNode->m_bottom_left->m_depth = curTreeDepth + 1;
		tempNode->m_bottom_right->m_depth = curTreeDepth + 1;
		tempNode->m_top_left->m_depth = curTreeDepth + 1;
		tempNode->m_top_right->m_depth = curTreeDepth + 1;

		nodeList.emplace(tempNode->m_bottom_left);
		nodeList.emplace(tempNode->m_bottom_right);
		nodeList.emplace(tempNode->m_top_left);
		nodeList.emplace(tempNode->m_top_right);
	}
}

void QuadTreeNode::createQuadTreeDFS(QuadTreeNode* &curNode, int treeDepth, const std::vector<QPointF> & point_list, float m_CenterX, float m_CenterY, float m_SizeX, float m_SizeY) {
	if (nullptr == curNode) {
		curNode = new QuadTreeNode();
	}
	float halfX = m_SizeX * 0.5;
	float halfY = m_SizeY * 0.5;
	curNode->m_tree_Index = treeDepth++;

	if (curNode->m_tree_Index > 0) {
		for (const auto & curP : point_list) {
			float xx = curP.x();
			float yy = curP.y();

			//该点坐标超出了当前网格的坐标范围
			if (xx < (m_CenterX - halfX) || xx >(m_CenterX + halfX) || yy < (m_CenterY - halfY) || yy > (m_CenterY + halfY)) {
				continue;
			}
			//将点存入此节点的数据列表中
			curNode->point_list.emplace_back(curP);
		}
	} else {
		curNode->point_list.assign(point_list.begin(), point_list.end());
	}

	curNode->m_point_num = static_cast<int>(curNode->point_list.size());
	curNode->m_XY_Size = point2D_MAXMIN(m_CenterX - halfX, m_CenterY - halfY, m_CenterX + halfX, m_CenterY + halfY);

	curNode->m_isSonNode = false;

	//四叉树划分中止条件，网格内点达到最小值
	if (curNode->m_point_num < minPointNumPerGrid) {
		curNode->m_isSonNode = true;
	}

	//四叉树划分中止条件，网格达到最大深度
	if (treeDepth > maxTreeDepth) {
		curNode->m_isSonNode = true;
	}

	if (curNode->m_isSonNode) {
		return;
    }

	float sonhalfX = halfX * 0.5;
	float sonhalfY = halfY * 0.5;

	//递归创建子树，根据节点的编号决定其子节点的坐标
	createQuadTreeDFS(curNode->m_bottom_left, treeDepth, curNode->point_list, m_CenterX - sonhalfX, m_CenterY - sonhalfY, halfX, halfY);
	createQuadTreeDFS(curNode->m_bottom_right, treeDepth, curNode->point_list, m_CenterX + sonhalfX, m_CenterY - sonhalfY, halfX, halfY);
	createQuadTreeDFS(curNode->m_top_left, treeDepth, curNode->point_list, m_CenterX - sonhalfX, m_CenterY + sonhalfY, halfX, halfY);
	createQuadTreeDFS(curNode->m_top_right, treeDepth, curNode->point_list, m_CenterX + sonhalfX, m_CenterY + sonhalfY, halfX, halfY);
}

void QuadTreeNode::getAllQuadNode(QuadTreeNode* curNode, std::vector<QuadTreeNode*> &node_list) {
	if (nullptr == curNode) {
		return;
	}

	node_list.emplace_back(curNode);

	getMaxDepQuadNode(curNode->m_bottom_left, node_list);
	getMaxDepQuadNode(curNode->m_bottom_right, node_list);
	getMaxDepQuadNode(curNode->m_top_left, node_list);
	getMaxDepQuadNode(curNode->m_top_right, node_list);
}

void QuadTreeNode::getMaxDepQuadNode(QuadTreeNode* curNode, std::vector<QuadTreeNode*> &node_list) {
	if (nullptr == curNode) {
		return;
	}

	if (curNode->m_point_num <= 0) {
		return;
	}

	if (curNode->m_isSonNode) {
		node_list.emplace_back(curNode);
		return;
	}

	getMaxDepQuadNode(curNode->m_bottom_left, node_list);
	getMaxDepQuadNode(curNode->m_bottom_right, node_list);
	getMaxDepQuadNode(curNode->m_top_left, node_list);
	getMaxDepQuadNode(curNode->m_top_right, node_list);
}
