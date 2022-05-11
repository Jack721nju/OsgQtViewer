#include "QuadTree.h"

void createQuadTree(QuadTreeNode* curNode, int treeDepth, const std::vector<QPoint> & point_list, const point2D_MAXMIN &curSize) {
	if (nullptr == curNode) {
		curNode = new QuadTreeNode(curSize);
		curNode->m_tree_Index = treeDepth++;		
		for (const auto & curP : point_list){
			float xx = curP.x();
			float yy = curP.y();

			//该点坐标超出了当前网格的坐标范围
			if (xx > curSize.xmax || xx < curSize.xmin || yy > curSize.ymax || yy < curSize.ymin)	{
				continue;
			}
			//将点存入此节点的数据列表中
			curNode->point_list.emplace_back(curP);
		}
		curNode->m_point_num = point_list.size();

		//四叉树划分中止条件
		if (curNode->m_point_num < 5) {
			return;
		}

		float xm = (curSize.xmax - curSize.xmin) / 2;
		float ym = (curSize.ymax - curSize.ymin) / 2;

		//递归创建子树，根据节点的编号决定其子节点的坐标
		createQuadTree(curNode->m_top_left,  treeDepth, curNode->point_list, point2D_MAXMIN(curSize.xmin, curSize.ymax - ym, curSize.xmax, curSize.ymax));
		createQuadTree(curNode->m_top_right, treeDepth, curNode->point_list, point2D_MAXMIN(curSize.xmax - xm, curSize.ymax - ym, curSize.xmax, curSize.ymax));
		createQuadTree(curNode->m_bottom_left, treeDepth, curNode->point_list, point2D_MAXMIN(curSize.xmin, curSize.ymin, curSize.xmax - xm, curSize.ymax - ym));
		createQuadTree(curNode->m_bottom_right, treeDepth, curNode->point_list, point2D_MAXMIN(curSize.xmax - xm, curSize.ymin, curSize.xmax, curSize.ymax - ym));
	}
}