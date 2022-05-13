#include "QuadTree.h"
#include <math.h>
#include <vector>

const int minPointNumPerGrid = 10;
const int maxTreeDepth = 6;

using namespace std;

//��ȡ�����������ݵ�XY�����С��Χ
point2D_MAXMIN QuadTreeNode::getMinMaxXY(const std::vector<QPointF> & all_list) {
	point2D_MAXMIN Max_area;
	vector<float> x_list, y_list, z_list;
	for (int i = 0; i < all_list.size(); ++i) {
		x_list.push_back(all_list[i].x());
		y_list.push_back(all_list[i].y());
	}
	vector<float>::iterator xmax = max_element(begin(x_list), end(x_list));
	vector<float>::iterator ymax = max_element(begin(y_list), end(y_list));

	vector<float>::iterator xmin = min_element(begin(x_list), end(x_list));
	vector<float>::iterator ymin = min_element(begin(y_list), end(y_list));

	Max_area.xmax = *xmax;
	Max_area.ymax = *ymax;
	Max_area.xmin = *xmin;
	Max_area.ymin = *ymin;

	return Max_area;
}

void QuadTreeNode::createQuadTree(QuadTreeNode* &curNode, int treeDepth, const std::vector<QPointF> & point_list, float m_CenterX, float m_CenterY, float m_SizeX, float m_SizeY) {
	if (treeDepth > 0 || nullptr == curNode) {
		curNode = new QuadTreeNode();
	}
	float halfX = m_SizeX * 0.5;
	float halfY = m_SizeY * 0.5;
	curNode->m_tree_Index = treeDepth++;

	if (curNode->m_tree_Index > 0) {
		for (const auto & curP : point_list) {
			float xx = curP.x();
			float yy = curP.y();

			//�õ����곬���˵�ǰ��������귶Χ
			if (xx < (m_CenterX - halfX) || xx >(m_CenterX + halfX) || yy < (m_CenterY - halfY) || yy > (m_CenterY + halfY)) {
				continue;
			}
			//�������˽ڵ�������б���
			curNode->point_list.emplace_back(curP);
		}
	}
	else {
		curNode->point_list.assign(point_list.begin(), point_list.end());
	}

	curNode->m_point_num = curNode->point_list.size();
	curNode->m_XY_Size = point2D_MAXMIN(m_CenterX - halfX, m_CenterY - halfY, m_CenterX + halfX, m_CenterY + halfY);

	curNode->m_isSonNode = false;

	//�Ĳ���������ֹ�����������ڵ�ﵽ��Сֵ
	if (curNode->m_point_num < minPointNumPerGrid) {
		curNode->m_isSonNode = true;
	}

	//�Ĳ���������ֹ����������ﵽ������
	if (treeDepth > maxTreeDepth) {
		curNode->m_isSonNode = true;
	}

	if (curNode->m_isSonNode) {
		return;
	}	

	float sonhalfX = halfX * 0.5;
	float sonhalfY = halfY * 0.5;

	//�ݹ鴴�����������ݽڵ�ı�ž������ӽڵ������
	createQuadTree(curNode->m_bottom_left, treeDepth, curNode->point_list, m_CenterX - sonhalfX, m_CenterY - sonhalfY, halfX, halfY);
	createQuadTree(curNode->m_bottom_right, treeDepth, curNode->point_list, m_CenterX + sonhalfX, m_CenterY - sonhalfY, halfX, halfY);
	createQuadTree(curNode->m_top_left, treeDepth, curNode->point_list, m_CenterX - sonhalfX, m_CenterY + sonhalfY, halfX, halfY);
	createQuadTree(curNode->m_top_right, treeDepth, curNode->point_list, m_CenterX + sonhalfX, m_CenterY + sonhalfY, halfX, halfY);
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
	}

	getMaxDepQuadNode(curNode->m_bottom_left, node_list);
	getMaxDepQuadNode(curNode->m_bottom_right, node_list);
	getMaxDepQuadNode(curNode->m_top_left, node_list);
	getMaxDepQuadNode(curNode->m_top_right, node_list);	
}
