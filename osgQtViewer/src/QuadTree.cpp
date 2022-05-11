#include "QuadTree.h"

void createQuadTree(QuadTreeNode* curNode, int treeDepth, const std::vector<QPoint> & point_list, const point2D_MAXMIN &curSize) {
	if (nullptr == curNode) {
		curNode = new QuadTreeNode(curSize);
		curNode->m_tree_Index = treeDepth++;		
		for (const auto & curP : point_list){
			float xx = curP.x();
			float yy = curP.y();

			//�õ����곬���˵�ǰ��������귶Χ
			if (xx > curSize.xmax || xx < curSize.xmin || yy > curSize.ymax || yy < curSize.ymin)	{
				continue;
			}
			//�������˽ڵ�������б���
			curNode->point_list.emplace_back(curP);
		}
		curNode->m_point_num = point_list.size();

		//�Ĳ���������ֹ����
		if (curNode->m_point_num < 5) {
			return;
		}

		float xm = (curSize.xmax - curSize.xmin) / 2;
		float ym = (curSize.ymax - curSize.ymin) / 2;

		//�ݹ鴴�����������ݽڵ�ı�ž������ӽڵ������
		createQuadTree(curNode->m_top_left,  treeDepth, curNode->point_list, point2D_MAXMIN(curSize.xmin, curSize.ymax - ym, curSize.xmax, curSize.ymax));
		createQuadTree(curNode->m_top_right, treeDepth, curNode->point_list, point2D_MAXMIN(curSize.xmax - xm, curSize.ymax - ym, curSize.xmax, curSize.ymax));
		createQuadTree(curNode->m_bottom_left, treeDepth, curNode->point_list, point2D_MAXMIN(curSize.xmin, curSize.ymin, curSize.xmax - xm, curSize.ymax - ym));
		createQuadTree(curNode->m_bottom_right, treeDepth, curNode->point_list, point2D_MAXMIN(curSize.xmax - xm, curSize.ymin, curSize.xmax, curSize.ymax - ym));
	}
}