/* Copyright© 2022 Jack721 */
#include "AlphaShape.h"
#include <math.h>

// 用于多线程处理
static std::mutex all_mutex;
static std::vector<osg::Vec2> all_shape_points;
static std::vector<Edge> all_edges;
static std::vector<Circle> all_circles;
static size_t all_point_pair_N;

static bool checkPointSame(const osg::Vec2 &pointA, const osg::Vec2 &pointB) {
	if ((pointA - pointB).length() < 0.000001) {
		return true;
	}
	return false;
}

// 获取给定点云数据的XY最大最小范围
point_MAXMIN* getMinMaxXYZ(const std::vector<osg::Vec2> & all_list) {
	point_MAXMIN * Max_area = new point_MAXMIN;
	std::vector<float> x_list, y_list, z_list;
	for (int i = 0; i < all_list.size(); ++i) {
		x_list.push_back(all_list[i].x());
		y_list.push_back(all_list[i].y());
	}
	std::vector<float>::iterator xmax = std::max_element(begin(x_list), end(x_list));
	std::vector<float>::iterator ymax = std::max_element(begin(y_list), end(y_list));

	std::vector<float>::iterator xmin = std::min_element(begin(x_list), end(x_list));
	std::vector<float>::iterator ymin = std::min_element(begin(y_list), end(y_list));

	Max_area->xmax = *xmax;
	Max_area->ymax = *ymax;
	Max_area->xmin = *xmin;
	Max_area->ymin = *ymin;

	return Max_area;
}

SingleGrid2D::SingleGrid2D(float Grid_X, float Grid_Y) {
	cur_PointNum = 0;

	hasPoint = false;

	heightDifference = 0;

	nearByGridAllWithpoint = false;

	hasDetected = false;

	isSmoothGrid = true;

	SmoothDegree = 0;
}

SingleGrid2D::SingleGrid2D(const GridInfo & curGrid) {
	cur_PointNum = 0;

	hasPoint = false;

	heightDifference = 0;

	nearByGridAllWithpoint = false;

	hasDetected = false;

	isSmoothGrid = true;

	SmoothDegree = 0;

	curGridInfo = curGrid;
}

GridNet::GridNet(const std::vector<osg::Vec2> &pList) {
	Points_List.insert(Points_List.end(), pList.begin(), pList.end());
	Grid_list.clear();
	pointMMM = getMinMaxXYZ(Points_List);

	Grid_X = 0;
	Grid_Y = 0;

	Grid_Num = 0;
	Col_Num = 0;
	Row_Num = 0;

	GridOutside_Num = 0;
	GridWithPoint_Num = 0;
}

SingleGrid2D* GridNet::getGridByRowAndCol(int RowID, int ColID) {
	this->Grid_Num = this->Grid_list.size();
	for (size_t i = 0; i < this->Grid_Num; ++i) {
		SingleGrid2D* curGrid = this->Grid_list[i];
		if (curGrid->curGridInfo.m_Row == RowID) {
			if (curGrid->curGridInfo.m_Col == ColID) {
				return curGrid;
			}
		}
	}
	return nullptr;
}

bool GridNet::isPointInGrid(const osg::Vec2 & curPoint, SingleGrid2D *test_Grid) {
	float cur_P_X = curPoint.x();
	float cur_P_Y = curPoint.y();

	float grid_max_x = test_Grid->curGridInfo.Max_X;
	float grid_max_y = test_Grid->curGridInfo.Max_Y;
	float grid_min_x = test_Grid->curGridInfo.Min_X;
	float grid_min_y = test_Grid->curGridInfo.Min_Y;

	if ((cur_P_X > grid_min_x)
		&& (cur_P_X < grid_max_x)
		&& (cur_P_Y > grid_min_y)
		&& (cur_P_Y < grid_max_y)) {
		return true;
	} else {
		return false;
	}
}

// 检测每个二维网格的八邻域连通的网格，并逐一判断网格内是否有点
void GridNet::detectGridWithConnection() {
	for (const auto & curGrid2D : this->Grid_list) {
		if (false == curGrid2D->hasPoint) {
			continue;
		}

		int curGridID = curGrid2D->curGridInfo.m_ID;
		int curLeftGridID = curGridID - 1;
		int curRightGridID = curGridID + 1;

		int buttomGridID = curGridID - (this->Col_Num + 2);
		int buttomLeftGridID = buttomGridID - 1;
		int buttomRightGridID = buttomGridID + 1;

		int topGridID = curGridID + (this->Col_Num + 2);
		int topLeftGridID = topGridID - 1;
		int topRightGridID = topGridID + 1;

		std::vector<int> idList{ buttomLeftGridID, buttomGridID, buttomRightGridID, curLeftGridID, curRightGridID, topLeftGridID, topGridID, topRightGridID };
		int countNum = 0;
		SingleGrid2D *nearGrid2D = nullptr;
		for (const auto curID : idList) {
			if (curID < 0 || curID > this->Grid_list.size() - 1) {
				continue;
			}

			nearGrid2D = this->Grid_list[curID];
			if (nullptr == nearGrid2D) {
				continue;
			}
			if (nearGrid2D->hasPoint == true) {
				++countNum;
				curGrid2D->connectGridID_List.push_back(nearGrid2D->curGridInfo.m_ID);
			}
		}

		if (8 == countNum) {
			curGrid2D->nearByGridAllWithpoint = true;
		} else {
			++this->GridOutside_Num;
		}
	}
}

void GridNet::buildNetByNum(int RowNum, int ColNum) {
	this->Row_Num = (unsigned int)(RowNum);
	this->Col_Num = (unsigned int)(ColNum);

	float allPointsHeight = pointMMM->ymax - pointMMM->ymin;
	float allPointsWidth = pointMMM->xmax - pointMMM->xmin;

	this->Grid_X = static_cast<float>(allPointsWidth / Col_Num);
	this->Grid_Y = static_cast<float>(allPointsHeight / Row_Num);

	GridInfo cur_grid;
	cur_grid.Size_X = Grid_X;
	cur_grid.Size_Y = Grid_Y;

	int gridNum = -1;
	// 向外部扩张一层空的网格，便于后续的邻域搜索
	for (size_t i = 0; i < (Row_Num + 2); ++i) {
		for (size_t j = 0; j < (Col_Num + 2); ++j) {
			cur_grid.Min_X = pointMMM->xmin + Grid_X*(i - 1);
			cur_grid.Max_X = cur_grid.Min_X + Grid_X;
			cur_grid.Min_Y = pointMMM->ymin + Grid_Y*(j - 1);
			cur_grid.Max_Y = cur_grid.Min_Y + Grid_Y;

			cur_grid.m_Row = i;// 行号
			cur_grid.m_Col = j;// 列号

			SingleGrid2D * curGrid2D = new SingleGrid2D(cur_grid);

			if (nullptr == curGrid2D) {
				continue;
			}
			curGrid2D->curGridInfo.m_ID = ++gridNum;
			Grid_list.push_back(curGrid2D);
		}
	}

	this->Grid_Num = gridNum;

	for (const auto & curP : this->Points_List) {
		int row_ID = (int)((curP.x() - pointMMM->xmin) / Grid_X) + 1;
		int col_ID = (int)((curP.y() - pointMMM->ymin) / Grid_Y) + 1;

		auto curGridID = row_ID  * (Col_Num + 2) + col_ID;
		if (curGridID > gridNum) {
			continue;
		}

		// SingleGrid2D *locateGrid = getGridByRowAndCol(row_ID + 1, col_ID + 1);
		SingleGrid2D *locateGrid = Grid_list[curGridID];
		if (locateGrid) {
			locateGrid->PointList.emplace_back(curP);
		}
	}

	float CenterX = 0.0, CenterY = 0.0;
	for (const auto & eachGrid2D : Grid_list) {
		auto cur_PointNum = eachGrid2D->PointList.size();
		if (cur_PointNum < 1) {
			continue;
		}
		++this->GridWithPoint_Num;
		eachGrid2D->hasPoint = true;

		for (const auto & curP : eachGrid2D->PointList) {
			CenterX += curP.x();
			CenterY += curP.y();
		}
		eachGrid2D->CenterPoint.set(CenterX / cur_PointNum, CenterY / cur_PointNum);
	}

}

// 根据网格数量构建格网
void GridNet::buildNetByNumOld(int RowNum, int ColNum) {
	this->Row_Num = (unsigned int)(RowNum);
	this->Col_Num = (unsigned int)(ColNum);

	float allPointsHeight = pointMMM->ymax - pointMMM->ymin;
	float allPointsWidth = pointMMM->xmax - pointMMM->xmin;

	this->Grid_X = static_cast<float>(allPointsWidth / Col_Num);
	this->Grid_Y = static_cast<float>(allPointsHeight / Row_Num);

	GridInfo cur_grid;
	cur_grid.Size_X = Grid_X;
	cur_grid.Size_Y = Grid_Y;

	SingleGrid2D *curGrid2D = nullptr;
	int gridNum = -1;
	// 向外部扩张一层空的网格，便于后续的邻域搜索
	for (size_t i = 0; i < (Row_Num + 2); ++i) {
		for (size_t j = 0; j < (Col_Num + 2); ++j) {
			cur_grid.Min_X = pointMMM->xmin + Grid_X*(i - 1);
			cur_grid.Max_X = cur_grid.Min_X + Grid_X;
			cur_grid.Min_Y = pointMMM->ymin + Grid_Y*(j - 1);
			cur_grid.Max_Y = cur_grid.Min_Y + Grid_Y;

			cur_grid.m_Row = i;// 行号
			cur_grid.m_Col = j;// 列号

			curGrid2D = new SingleGrid2D(cur_grid);

			if (nullptr == curGrid2D) {
				continue;
			}

			float CenterX = 0.0, CenterY = 0.0;
			for (const auto & curP : this->Points_List) {
				if (isPointInGrid(curP, curGrid2D)) {
					curGrid2D->PointList.emplace_back(curP);
					CenterX += curP.x();
					CenterY += curP.y();
				}
			}

			curGrid2D->curGridInfo.m_ID = ++gridNum;
			curGrid2D->cur_PointNum = curGrid2D->PointList.size();

			if (curGrid2D->cur_PointNum > 0) {
				++this->GridWithPoint_Num;
				curGrid2D->hasPoint = true;
				curGrid2D->CenterPoint.set(CenterX / curGrid2D->cur_PointNum, CenterY / curGrid2D->cur_PointNum);
			}

			Grid_list.emplace_back(curGrid2D);
		}
	}
	this->Grid_Num = gridNum;
}

// 根据网格大小构建二维格网
void GridNet::buildNetBySize(float SizeX, float SizeY) {
	Grid_X = SizeX;
	Grid_Y = SizeY;

	int id = 0;

	float allPointsHeight = pointMMM->ymax - pointMMM->ymin;
	float allPointsWidth = pointMMM->xmax - pointMMM->xmin;

	this->Row_Num = (unsigned int)(ceil(allPointsHeight / Grid_Y));
	this->Col_Num = (unsigned int)(ceil(allPointsHeight / Grid_X));

	this->buildNetByNum(Row_Num, Col_Num);
}

// 获取网格内离散点的中心点
void GridNet::getCenterPoint() {
	int haspointGridNum = 0, GridOutsideNum = 0;
	for (const auto & curGrid2D : this->Grid_list) {
		if (curGrid2D->hasPoint) {
			++haspointGridNum;
			if (false == curGrid2D->nearByGridAllWithpoint) {
				++GridOutsideNum;
			}
		}
	}
	this->GridWithPoint_Num = haspointGridNum;
	this->GridOutside_Num = GridOutsideNum;
}

// 获取网格内离散点的中心点与邻域网格中心点的连接向量
void GridNet::getVectorOfOutSideGrid() {
	this->MaxVector_Grid = 0.0;
	this->MinVector_Grid = 1 << 31;

	for (const auto & curGrid : this->Grid_list) {
		// 当前网格内无点
		if (false == curGrid->hasPoint) {
			continue;
		}

		// 当前网格的八邻域网格均含有点
		if (curGrid->nearByGridAllWithpoint) {
			continue;
		}

		int curRowID = curGrid->curGridInfo.m_Row;
		int curColID = curGrid->curGridInfo.m_Col;

		const osg::Vec2 & curGridCenterPoint = curGrid->CenterPoint;

		curGrid->curVectorGrid.set(0.0, 0.0);
		SingleGrid2D* nearGrid = nullptr;

		// 统计当前网格的上下左右的邻域网格，而不是八网格
		for (size_t k = curRowID - 1; k <= curRowID + 1; ++k) {
			for (size_t j = curColID - 1; j <= curColID + 1; ++j) {
				if ((k == curRowID) && (j == curColID)) {
					continue;
				}

				if (k < 0 || j < 0) {
					continue;
				}

				if (k >= (this->Row_Num + 2) || j >= (this->Col_Num + 2)) {
					continue;
				}

				nearGrid = this->getGridByRowAndCol(k, j);

				if (nullptr == nearGrid) {
					continue;
				}

				if (nearGrid->hasPoint) {
					// 邻域网格也属于边界网格
					if (false == nearGrid->nearByGridAllWithpoint) {
						const osg::Vec2 &nearGridCenterPoint = nearGrid->CenterPoint;
						curGrid->curVectorGrid += (nearGridCenterPoint - curGridCenterPoint);
						curGrid->VectorList.emplace_back(nearGridCenterPoint - curGridCenterPoint);
					}
				}
			}
		}

		float curVectorDis = curGrid->curVectorGrid.length();

		if (curVectorDis > this->MaxVector_Grid) {
			this->MaxVector_Grid = curVectorDis;
		}

		if (curVectorDis < this->MinVector_Grid) {
			this->MinVector_Grid = curVectorDis;
		}
	}
}

static float AngleBetweenVector(osg::Vec2 vector1, osg::Vec2 vector2) {
	double sin = vector1.x() * vector2.y() - vector2.x() * vector1.y();
	double cos = vector1.x() * vector2.x() + vector1.y() * vector2.y();
	return std::atan2(sin, cos) * (180.0 / 3.1415926);
}

// 检测外部边界网格的平滑度，筛选出不平滑的网格，用于单独处理
void GridNet::DetectSmoothForOutSideGrid() {
	for (const auto & curGrid : this->Grid_list) {
		// 当前网格内无点
		if (false == curGrid->hasPoint) {
			continue;
		}

		// 当前网格的八邻域网格均含有点
		if (curGrid->nearByGridAllWithpoint) {
			continue;
		}

		// 根据邻域网格的数量进行判断
		int FullgridNum = curGrid->connectGridID_List.size();
		int needCalculateGridNum = 0;

		std::vector<SingleGrid2D*> needCalculateGrid_list;
		SingleGrid2D* nearGrid = nullptr;
		for (int k = 0; k < FullgridNum; ++k) {
			nearGrid = this->Grid_list[curGrid->connectGridID_List[k]];
			if (nullptr == nearGrid) {
				continue;
			}

			if (nearGrid->hasPoint) {
				if (false == nearGrid->nearByGridAllWithpoint) {
					++needCalculateGridNum;
					needCalculateGrid_list.emplace_back(nearGrid);
				}
			}
		}

		// 若邻域网格数量小于设定阈值，则认为粗糙度较大
		if (needCalculateGridNum < 2) {
			curGrid->isSmoothGrid = false;
			curGrid->SmoothDegree = 2;
			continue;
		}

		if (needCalculateGridNum == 7) {
			curGrid->isSmoothGrid = true;
			curGrid->SmoothDegree = 0;
			continue;
		}

		if (needCalculateGridNum == 2) {
			SingleGrid2D* nearGrid_1 = needCalculateGrid_list[0];
			SingleGrid2D* nearGrid_2 = needCalculateGrid_list[1];

			int col1 = nearGrid_1->curGridInfo.m_Col;
			int col2 = nearGrid_2->curGridInfo.m_Col;
			int row1 = nearGrid_1->curGridInfo.m_Row;
			int row2 = nearGrid_2->curGridInfo.m_Row;

			int curCol = curGrid->curGridInfo.m_Col;
			int curRow = curGrid->curGridInfo.m_Row;

			if ((col1 == curCol) && (col2 == curCol)) {
				curGrid->isSmoothGrid = true;
				curGrid->SmoothDegree = 0;
				continue;
			}

			if ((row1 == curRow) && (row2 == curRow)) {
				curGrid->isSmoothGrid = true;
				curGrid->SmoothDegree = 0;
				continue;
			}

			if (((col1 + col2) == (curCol * 2)) && ((row1 + row2) == (curRow * 2))) {
				curGrid->isSmoothGrid = true;
				curGrid->SmoothDegree = 0;
				continue;
			}

			curGrid->isSmoothGrid = false;
			curGrid->SmoothDegree = 2;
			continue;
		}

		if ((needCalculateGridNum == 3) || (needCalculateGridNum == 4)) {
			curGrid->isSmoothGrid = false;
			curGrid->SmoothDegree = 1;
		}

		//根据邻域网格的向量夹角和合向量距离进行判断
		int cur_VectorDis = curGrid->curVectorGrid.length();
		int baseValue = (this->MaxVector_Grid - this->MinVector_Grid) / 2.5 + this->MinVector_Grid;

		int vectorNum = curGrid->VectorList.size();
		float AngleValue = 45.0;
		bool isBeyondAngle = false;

		for (int j = 0; j < vectorNum; ++j) {
			const osg::Vec2 &curVector = curGrid->VectorList[j];
			for (int k = j + 1; k < vectorNum; ++k) {
				const osg::Vec2 &nextVector = curGrid->VectorList[k];
				if (AngleBetweenVector(curVector, nextVector) > AngleValue) {
					isBeyondAngle = true;
				}
			}
		}

		bool islittleSmooth = false;

		if (cur_VectorDis > baseValue) {
			if (isBeyondAngle) {
				islittleSmooth = true;
			}
		}

		if (islittleSmooth) {
			curGrid->isSmoothGrid = false;
			curGrid->SmoothDegree = 1;
		}
	}
}


AlphaShape::AlphaShape(const std::vector<osg::Vec2> & point_list) {
	m_radius = 0.0;
	m_points.insert(m_points.end(), point_list.begin(), point_list.end());
}

AlphaShape::AlphaShape(GridNet * curGridNet) {
	m_radius = 0.0;
	m_gridNet = curGridNet;
	if (curGridNet) {
		m_points.insert(m_points.end(), curGridNet->Points_List.begin(), curGridNet->Points_List.end());
	}
}

AlphaShape::AlphaShape(pcl::PointCloud<pcl::PointXYZ>::Ptr project2DPoints) {
	m_radius = 0.0;
	m_projectPcl2DPoints = project2DPoints;
}

AlphaShape::~AlphaShape() {
	m_radius = 0.0;

	m_points.clear();
	m_edges.clear();
	m_circles.clear();

	m_shape_id.clear();
	m_shape_points.clear();
}

static float Distance_point(const osg::Vec2 &pointA, const osg::Vec2 &pointB) {
	// return std::sqrt(std::pow(pointA.x() - pointB.x(), 2) + std::pow(pointA.y() - pointB.y(), 2));
	return (pointA - pointB).length();
}

bool sortFun(const float & angle1, const float & angle2) {
	return angle1 < angle2;
}

// 计算每个点为中心的包裹圆，统计落在圆形内部的点的数量,通过数量对中心点是否属于边界点进行判断
void AlphaShape::Detect_Shape_By_PackCirlce(GridNet* curGridNet, float radius, int pointMaxNum) {
	this->m_shape_points.clear();

	// 遍历所有网格
	for (size_t i = 0; i < curGridNet->Grid_Num; i++) {
		// 当前网格
		SingleGrid2D* curGrid = curGridNet->Grid_list[i];

		if (curGrid) {
			curGrid->hasDetected = true;// 网格被检测过了
		}

		int curRowID = curGrid->curGridInfo.m_Row;
		int curColID = curGrid->curGridInfo.m_Col;

		// 当前网格内无点，为空网格，直接跳过
		if (false == curGrid->hasPoint) {
			continue;
		}

		// 当前网格的八邻域网格均含有点，说明不是边界网格，直接跳过
		if (curGrid->nearByGridAllWithpoint) {
			continue;
		}

	    // 当前边界网格内点数量
		int pointListNum = curGrid->PointList.size();

		// 逐一判断当前网格内点，以此点为圆心，以设置的半径画圆，判断落在圆内点的数量
		for (int k = 0; k < pointListNum; k++) {
			float curPointX = curGrid->PointList[k].x();
			float curPointY = curGrid->PointList[k].y();

			osg::Vec2 curPoint(curPointX, curPointY);

			int allpointNum = curGridNet->Points_List.size();

			int PointInCircleNum = 0;

			for (int m = 0; m < allpointNum; m++) {
				float nerPointX = curGridNet->Points_List[m].x();
				float nerPointY = curGridNet->Points_List[m].y();

				osg::Vec2 nerPoint(nerPointX, nerPointY);

				float distance = Distance_point(curPoint, nerPoint);

				if (distance < radius && distance > 0.000001) {
					PointInCircleNum++;
				}
			}

			if (PointInCircleNum < pointMaxNum) {
				this->m_shape_points.push_back(curPoint);
			}
		}
	}
}

// 计算每个点为中心的包裹圆，统计落在圆形内部的点的数量以及方位角，通过数量以及方位夹角对中心点是否属于边界点进行判断
void AlphaShape::Detect_Shape_By_SingleCirlce(GridNet* curGridNet, float radius, int pointNum) {
	// 用于判断检测圆是否含有点的判断点云
	std::vector<osg::Vec2> detect_point_list;

	for (size_t i = 0; i < curGridNet->Grid_Num; i++) {
		SingleGrid2D* curGrid = curGridNet->Grid_list[i];

		if (curGrid) {
			curGrid->hasDetected = true;
		}

		int curRowID = curGrid->curGridInfo.m_Row;
		int curColID = curGrid->curGridInfo.m_Col;

		// m_points.clear();

		// 当前网格内无点
		if (false == curGrid->hasPoint) {
			continue;
		}

		// 当前网格的八邻域网格均含有点
		if (curGrid->nearByGridAllWithpoint) {
			continue;
		}

		detect_point_list.clear();

		for (size_t k = curRowID - 1; k <= curRowID + 1; k++) {
			for (size_t j = curColID - 1; j <= curColID + 1; j++) {
				if ((k == curRowID) && (j == curColID)) {
					// continue;
				}

				if (k < 0 || j < 0) {
					continue;
				}

				if (k >= (curGridNet->Row_Num + 2) || j >= (curGridNet->Col_Num + 2)) {
					continue;
				}

				SingleGrid2D* nearGrid = curGridNet->getGridByRowAndCol(k, j);

				if (nullptr == nearGrid) {
					continue;
				}

				if (nearGrid->hasPoint) {
					for (size_t m = 0; m < nearGrid->cur_PointNum; m++) {
						float nearPointX = nearGrid->PointList[m].x();
						float nearPointY = nearGrid->PointList[m].y();

						osg::Vec2 nearPoint(nearPointX, nearPointY);
						detect_point_list.emplace_back(nearPoint);
					}
				}
			}
		}

		int pointListNum = curGrid->PointList.size();

		for (int k = 0; k < pointListNum; k++) {
			float curPointX = curGrid->PointList[k].x();
			float curPointY = curGrid->PointList[k].y();

			osg::Vec2 curPoint(curPointX, curPointY);

			int circlePointNum = 0;

			std::vector<float> angle_List;
			angle_List.clear();

			//计算落在指定检测圆内的点，并计算方位角度
			for (int m = 0; m < detect_point_list.size(); m++) {
				float nerPointX = detect_point_list[m].x();
				float nerPointY = detect_point_list[m].y();

				osg::Vec2 nerPoint(nerPointX, nerPointY);

				float distance = Distance_point(curPoint, nerPoint);

				if (distance< radius && distance>0.000001) {
					circlePointNum++;

					float deltX = nerPointX - curPointX;
					float deltY = nerPointY - curPointY;

					// atan2(y,x)所表达的意思是坐标原点为起点，指向(x,y)的射线在坐标平面上与x轴正方向之间的角的角度。
					float auizumAngle = std::atan2(deltY, deltX) * 180 / 3.1415926;
					// 结果为正表示从 X 轴逆时针旋转的角度，结果为负表示从 X 轴顺时针旋转的角度。

					if (auizumAngle < 0) {
						// 结果表示从 X 轴逆时针旋转的角度
						auizumAngle += 360.0;
					}
					angle_List.push_back(auizumAngle);
				}
			}

			std::sort(angle_List.begin(), angle_List.end(), sortFun);

			float deltAngle = 0.0;

			bool isBeyondAngle = false;

            if (angle_List.size() > 0) {
				for (int x = 0; x < angle_List.size() - 1; x++) {
					int y = x + 1;

					if (y < (angle_List.size() - 1)) {
						deltAngle = abs(angle_List[y] - angle_List[x]);
					}

					if (y == (angle_List.size() - 1)) {
						deltAngle = (360 - angle_List[y]) + angle_List[0];
					}

					if (deltAngle > 90.0) {
						isBeyondAngle = true;
					}
				}
			}
            if (circlePointNum < pointNum) {
				m_shape_points.emplace_back(curPoint);
			}
		}
	}
}

void AlphaShape::Detect_Shape_By_GridNet_New(float radius) {
	if (nullptr == m_gridNet) {
		return;
	}
	this->Detect_Shape_line_by_Grid_New(radius, m_gridNet->Grid_list);
}

// 方法五：
void AlphaShape::Detect_Shape_by_PCl_Concave_Hull(float radius) {
	if (m_projectPcl2DPoints.get() == nullptr) {
		return;
	}
	
	pcl::ConcaveHull<pcl::PointXYZ> conHull;
	conHull.setInputCloud(m_projectPcl2DPoints);
	conHull.setAlpha(radius);

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	conHull.reconstruct(*point_boundary);

	auto pointNum = point_boundary->points.size();
	m_shape_points.clear();
	for (int i = 0; i < pointNum; ++i) {
		m_shape_points.emplace_back(osg::Vec2(point_boundary->at(i).x, point_boundary->at(i).y));
	}
}

//方法四：基于方法二，不同之处在于滚动圆的检测半径可变
void AlphaShape::Detect_Shape_line_by_Grid_New(float radius, const std::vector<SingleGrid2D*> & allGridList) {
	all_edges.clear();
	all_circles.clear();
	all_shape_points.clear();
	all_point_pair_N = 0;
	float scaleRate = 0.8f;
	int cur_point_pair_N = 0;

	std::vector<osg::Vec2> cur_shape_points;
	std::vector<Circle> cur_circles;
	std::vector<Edge> cur_edges;

	std::vector<osg::Vec2> detectAreaAllPointList;
	int circleSize = 0;

	for (const auto & centerGrid : allGridList) {
		if (false == centerGrid->hasPoint) {
			continue;
		}

		//若邻域网格内点数量较少且较为离散，可能会出现漏检情况，所以最好别跳过
		if (centerGrid->nearByGridAllWithpoint) {
			continue;
		}

		// 当前网格不平滑，需要缩小检测半径，检测更为细致
		if (false == centerGrid->isSmoothGrid) {
			circleSize = centerGrid->SmoothDegree;

			if (circleSize == 1) {
				m_radius = radius * scaleRate;
			}

			if (circleSize == 2) {
				m_radius = radius * scaleRate * scaleRate;
			}
		} else {
			m_radius = radius;
			circleSize = 0;
		}

		detectAreaAllPointList.clear();
		detectAreaAllPointList.insert(detectAreaAllPointList.end(), centerGrid->PointList.begin(), centerGrid->PointList.end());
		for (const auto nearGirdID : centerGrid->connectGridID_List) {
			const auto & curNearGrid = allGridList[nearGirdID];
			if (nullptr == curNearGrid || !curNearGrid->hasPoint) {
				continue;
			}
			detectAreaAllPointList.insert(detectAreaAllPointList.end(), curNearGrid->PointList.begin(), curNearGrid->PointList.end());
		}

		for (const auto & centerPoint : centerGrid->PointList) {
			bool isAddToShape = false;
			for (const auto & outPoint : detectAreaAllPointList) {
				if (Distance_point(centerPoint, outPoint) > 2 * m_radius) {
					continue;
				}

				if (checkPointSame(centerPoint, outPoint)) {
					continue;
				}

				++cur_point_pair_N;

				const osg::Vec2 &mid_point = (centerPoint + outPoint) / 2;//线段中点
				const osg::Vec2 &vector_line = centerPoint - outPoint;//线段的方向向量

				float a = 1.0, b = 1.0;

				if (abs(vector_line.x()) < 0.001) {
					b = 0.0;
				} else {
					a = (-b * vector_line.y()) / vector_line.x();
				}

				//线段的垂直向量
				osg::Vec2 normal(a, b);
				normal.normalize();//单位向量化

				float line_length = vector_line.length() / 2.0;
				float length = sqrt(std::pow(radius, 2) - std::pow(line_length, 2));

				//两外接圆圆心
				const osg::Vec2 &center1 = mid_point + normal * length;
				const osg::Vec2 &center2 = mid_point - normal * length;

				bool hasPointInCircle1 = false, hasPointInCircle2 = false;

				for (const auto & checkPoint : detectAreaAllPointList) {
					if (checkPointSame(centerPoint, checkPoint) ||
						checkPointSame(outPoint, checkPoint)) {
						continue;
					}

					if (hasPointInCircle1 && hasPointInCircle2) {
						break;
					}

					if (!hasPointInCircle1 && Distance_point(checkPoint, center1) < m_radius) {
						hasPointInCircle1 = true;
					}

					if (!hasPointInCircle2 && Distance_point(checkPoint, center2) < m_radius) {
						hasPointInCircle2 = true;
					}
				}

				if (!hasPointInCircle1 || !hasPointInCircle2) {
					cur_edges.emplace_back(Edge(centerPoint, outPoint));

					if (false == hasPointInCircle1) {
						cur_circles.emplace_back(Circle(center1, m_radius, circleSize));
					}

					if (false == hasPointInCircle2) {
						cur_circles.emplace_back(Circle(center2, m_radius, circleSize));
					}

					if (false == isAddToShape) {
						cur_shape_points.emplace_back(centerPoint);
						isAddToShape = true;
					}
				}
			}
		}
	}

	all_edges.insert(all_edges.end(), cur_edges.begin(), cur_edges.end());
	all_circles.insert(all_circles.end(), cur_circles.begin(), cur_circles.end());
	m_shape_points.insert(m_shape_points.end(), cur_shape_points.begin(), cur_shape_points.end());
	all_point_pair_N += cur_point_pair_N;

	std::set<Edge> edge_set(all_edges.begin(), all_edges.end());
	m_edges.assign(edge_set.begin(), edge_set.end());

	std::set<Circle> circle_set(all_circles.begin(), all_circles.end());
	m_circles.assign(circle_set.begin(), circle_set.end());
}

// 方法三的子函数：基于网格筛选的alpha shape处理函数，供多线程调用
void thread_detect_By_GridList(float radius, const std::vector<SingleGrid2D*> & centerGridList, const std::vector<SingleGrid2D*> & allGridList) {
	thread_local int cur_point_pair_N = 0;
	thread_local std::vector<osg::Vec2> cur_shape_points;
	thread_local std::vector<Circle> cur_circles;
	thread_local std::vector<Edge> cur_edges;
	thread_local std::vector<osg::Vec2> detectAreaAllPointList;
	float dixMax = 2 * radius;

	for (const auto & centerGrid : centerGridList) {
		if (false == centerGrid->hasPoint) {
			continue;
		}

		//若邻域网格内点数量较少且较为离散，可能会出现漏检情况，所以最好别跳过
		if (centerGrid->nearByGridAllWithpoint) {
			continue;
		}

		detectAreaAllPointList.clear();
		detectAreaAllPointList.insert(detectAreaAllPointList.end(), centerGrid->PointList.begin(), centerGrid->PointList.end());
		for (const auto nearGirdID : centerGrid->connectGridID_List) {
			const auto & curNearGrid = allGridList[nearGirdID];
			if (nullptr == curNearGrid || !curNearGrid->hasPoint) {
				continue;
			}
			detectAreaAllPointList.insert(detectAreaAllPointList.end(), curNearGrid->PointList.begin(), curNearGrid->PointList.end());
		}

		for (const auto & centerPoint : centerGrid->PointList) {
			bool isAddToShape = false;
			for (const auto & outPoint : detectAreaAllPointList) {
				if (Distance_point(centerPoint, outPoint) > dixMax) {
					continue;
				}

				if (checkPointSame(centerPoint, outPoint)) {
					continue;
				}

				++cur_point_pair_N;

				const osg::Vec2 &mid_point = (centerPoint + outPoint) * 0.5;//线段中点
				const osg::Vec2 &vector_line = centerPoint - outPoint;//线段的方向向量

				float a = 1.0, b = 1.0;

				if (abs(vector_line.x()) < 0.001) {
					b = 0.0;
				} else {
					a = (-b * vector_line.y()) / vector_line.x();
				}

				//线段的垂直向量
				osg::Vec2 normal(a, b);
				normal.normalize();//单位向量化

				float line_length = vector_line.length() * 0.5;
				float length = sqrt(std::pow(radius, 2) - std::pow(line_length, 2));

				//两外接圆圆心
				const osg::Vec2 &normal_length = normal * length;
				const osg::Vec2 &center1 = mid_point + normal_length;
				const osg::Vec2 &center2 = mid_point - normal_length;

				bool hasPointInCircle1 = false, hasPointInCircle2 = false;

				for (const auto & checkPoint : detectAreaAllPointList) {

					if (checkPointSame(centerPoint, checkPoint) ||
						checkPointSame(outPoint, checkPoint)) {
						continue;
					}

					if (hasPointInCircle1 && hasPointInCircle2) {
						break;
					}

					if (!hasPointInCircle1 && Distance_point(checkPoint, center1) < radius) {
						hasPointInCircle1 = true;
					}

					if (!hasPointInCircle2 && Distance_point(checkPoint, center2) < radius) {
						hasPointInCircle2 = true;
					}
				}

				if (!hasPointInCircle1 || !hasPointInCircle2) {
					cur_edges.emplace_back(Edge(centerPoint, outPoint));

					if (false == hasPointInCircle1) {
						cur_circles.emplace_back(Circle(center1, radius));
					}

					if (false == hasPointInCircle2) {
						cur_circles.emplace_back(Circle(center2, radius));
					}

					if (false == isAddToShape) {
						cur_shape_points.emplace_back(centerPoint);
						isAddToShape = true;
					}
				}
			}
		}
	}

	// 加锁，避免多线程资源冲突
    {
		std::lock_guard<std::mutex> lock(all_mutex);
		all_edges.insert(all_edges.end(), cur_edges.begin(), cur_edges.end());
		all_circles.insert(all_circles.end(), cur_circles.begin(), cur_circles.end());
		all_shape_points.insert(all_shape_points.end(), cur_shape_points.begin(), cur_shape_points.end());
		all_point_pair_N += cur_point_pair_N;
	}
}

// 方法三：基于方法二，利用多线程进行加速
void AlphaShape::Detect_Alpha_Shape_by_Grid_Multi_Thread(float radius, int threadNum) {
	if (nullptr == m_gridNet) {
		return;
	}

	this->m_radius = radius;
	this->m_point_pair_N = 0;
	this->point_pair_scale = 0.0;

	int point_num = m_points.size();

	m_shape_points.clear();
	m_shape_points.resize(point_num);
	m_circles.clear();
	m_edges.clear();

	const auto & gridList = m_gridNet->Grid_list;
	int step = static_cast<int>(gridList.size() / threadNum);

	std::vector<std::thread> threadList;
	std::vector<SingleGrid2D*> curList;
	for (int i = 0; i < threadNum; ++i) {
		curList.clear();
		if (i == (threadNum - 1)) {
			curList.assign(gridList.begin() + step * i, gridList.end());
		} else {
			curList.assign(gridList.begin() + step * i, gridList.begin() + step * (i + 1));
		}
		threadList.emplace_back(std::thread(thread_detect_By_GridList, m_radius, curList, std::ref(gridList)));
	}

	for (auto & curThread : threadList) {
		curThread.join();
	}

	std::set<Edge> edge_set(all_edges.begin(), all_edges.end());
	m_edges.assign(edge_set.begin(), edge_set.end());

	std::set<Circle> circle_set(all_circles.begin(), all_circles.end());
	m_circles.assign(circle_set.begin(), circle_set.end());

	m_shape_points.assign(all_shape_points.begin(), all_shape_points.end());

	this->point_pair_scale = static_cast<float>((all_point_pair_N * 2) / (point_num*(point_num - 1)));
}

// 方法二：构造二维格网，仅对3x3窗口网格内的点进行alpha shape检测，从而提升检测效率
void AlphaShape::Detect_Alpha_Shape_by_Grid(float radius) {
	if (nullptr == m_gridNet) {
		return;
	}

	m_radius = radius;
	int point_pair_N = 0;
	this->point_pair_scale = 0.0;
	int point_num = m_points.size();

	m_shape_points.clear();
	m_shape_points.reserve(point_num);
	m_circles.clear();
	m_edges.clear();

	const auto & gridList = m_gridNet->Grid_list;

	std::vector<osg::Vec2> detectAreaAllPointList;
	float disMax = 2 * m_radius;

	for (const auto & centerGrid : gridList) {
		if (nullptr == centerGrid || false == centerGrid->hasPoint) {
			continue;
		}

		//若邻域网格内点数量较少且较为离散，可能会出现漏检情况
		if (centerGrid->nearByGridAllWithpoint) {
			continue;
		}

		detectAreaAllPointList.clear();
		detectAreaAllPointList.insert(detectAreaAllPointList.end(), centerGrid->PointList.begin(), centerGrid->PointList.end());
		for (const auto nearGirdID : centerGrid->connectGridID_List) {
			const auto & curNearGrid = gridList[nearGirdID];
			if (nullptr == curNearGrid || !curNearGrid->hasPoint) {
				continue;
			}
			detectAreaAllPointList.insert(detectAreaAllPointList.end(), curNearGrid->PointList.begin(), curNearGrid->PointList.end());
		}

		for (const auto & centerPoint : centerGrid->PointList) {
			bool isAddToShape = false;
			for (const auto & outPoint : detectAreaAllPointList) {
				if (Distance_point(centerPoint, outPoint) > disMax) {
					continue;
				}

				if (checkPointSame(centerPoint, outPoint)) {
					continue;
				}

				++point_pair_N;

				const osg::Vec2 &mid_point = (centerPoint + outPoint) * 0.5;//线段中点
				const osg::Vec2 &vector_line = centerPoint - outPoint;//线段的方向向量

				float a = 1.0, b = 1.0;
				float vector_lineX = vector_line.x();
				float vector_lineY = vector_line.y();

				if (abs(vector_lineX) < 0.001) {
					b = 0.0;
				} else {
					a = (-b * vector_lineY) / vector_lineX;
				}

				//线段的垂直向量
				osg::Vec2 normal(a, b);
				normal.normalize();//单位向量化

				float line_length = vector_line.length() * 0.5;
				float length = sqrt(std::pow(m_radius, 2) - std::pow(line_length, 2));

				//两外接圆圆心
				const osg::Vec2 &normal_length = normal * length;
				const osg::Vec2 &center1 = mid_point + normal_length;
				const osg::Vec2 &center2 = mid_point - normal_length;

				bool hasPointInCircle1 = false, hasPointInCircle2 = false;

				for (const auto & checkPoint : detectAreaAllPointList) {
					if (checkPointSame(centerPoint, checkPoint) ||
						checkPointSame(outPoint, checkPoint)) {
						continue;
					}

					if (hasPointInCircle1 && hasPointInCircle2) {
						break;
					}

					if (!hasPointInCircle1 && Distance_point(checkPoint, center1) < m_radius) {
						hasPointInCircle1 = true;
					}

					if (!hasPointInCircle2 && Distance_point(checkPoint, center2) < m_radius) {
						hasPointInCircle2 = true;
					}
				}

				if (!hasPointInCircle1 || !hasPointInCircle2) {
					m_edges.emplace_back(Edge(centerPoint, outPoint));

					if (false == hasPointInCircle1) {
						m_circles.emplace_back(Circle(center1, m_radius));
					}

					if (false == hasPointInCircle2) {
						m_circles.emplace_back(Circle(center2, m_radius));
					}

					if (false == isAddToShape) {
						m_shape_points.emplace_back(centerPoint);
						isAddToShape = true;
					}
				}
			}
		}
	}
	std::set<Edge> edge_set(m_edges.begin(), m_edges.end());
	m_edges.assign(edge_set.begin(), edge_set.end());

	std::set<Circle> circle_set(m_circles.begin(), m_circles.end());
	m_circles.assign(circle_set.begin(), circle_set.end());

	this->point_pair_scale = static_cast<float>((point_pair_N * 2) / (point_num*(point_num - 1)));
}


void Detect_Alpah_Shape_FLANN_single_thread(float radius, const std::vector<int> & targetPointIndexList, pcl::PointCloud<pcl::PointXYZ>::Ptr m_projectPcl2DPoints, 
	                                                                                                    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree) {
	float m_radius = radius;
	int cur_point_pair_N = 0;
	int point_num = targetPointIndexList.size();

	thread_local std::vector<osg::Vec2> cur_shape_points;
	thread_local std::vector<Circle> cur_circles;
	thread_local std::vector<Edge> cur_edges;
	thread_local std::unordered_set<int> m_shape_id_set;
	m_shape_id_set.reserve(point_num);

	float distanceMax = 2 * m_radius;

	// pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	if (kdTree.get() == nullptr) {
		return;
	}
	// kdTree.setInputCloud(m_projectPcl2DPoints);

	std::vector<float> disList;
	disList.reserve(point_num);
	std::vector<int> indexList;
	indexList.reserve(point_num);

	const auto &m_PclPoints = m_projectPcl2DPoints->points;
	uint32_t detectAllNum = 0;

	for (int i = 0; i < point_num; ++i) {
		int indexI = targetPointIndexList[i];
		const auto &curP = m_PclPoints[indexI];
		// 获取与当前点距离小于滚动圆直径的其他点
		auto curNearPointNum = kdTree->radiusSearch(curP, distanceMax, indexList, disList);
		osg::Vec2 point_i(curP.x, curP.y);

		for (int k = 0; k < curNearPointNum; ++k) {
			int indexK = indexList[k];
			if (indexK == indexI) {
				continue;
			}

			osg::Vec2 point_k(m_PclPoints[indexK].x, m_PclPoints[indexK].y);
			++cur_point_pair_N;

			const osg::Vec2 & mid_point = (point_i + point_k) * 0.5;  //线段中点
			const osg::Vec2 & vector_line = point_i - point_k;      //线段的方向向量

			float a = 1.0, b = 1.0;

			const auto& vector_lineX = vector_line.x();
			const auto& vector_lineY = vector_line.y();

			if (abs(vector_lineX) < 0.001) {
				b = 0.0;
			}
			else {
				a = (-b * vector_lineY) / vector_lineX;
			}

			//线段的垂直向量
			osg::Vec2 normal(a, b);
			normal.normalize();//单位向量化

			const auto& line_length_2 = disList[k];
			float length = sqrt(std::pow(m_radius, 2) - line_length_2 * 0.25);

			//两外接圆圆心
			const osg::Vec2 &length_normal = normal * length;
			const osg::Vec2 &center1 = mid_point + length_normal;
			const osg::Vec2 &center2 = mid_point - length_normal;

			bool hasPointInCircle1 = false, hasPointInCircle2 = false;

			for (int m = 0; m < curNearPointNum; ++m) {
				int indexM = indexList[m];
				if (indexM == indexK || indexM == indexI) {
					continue;
				}
				osg::Vec2 point_m(m_PclPoints[indexM].x, m_PclPoints[indexM].y);

				if (hasPointInCircle1 && hasPointInCircle2) {
					break;
				}

				if (!hasPointInCircle1 && Distance_point(point_m, center1) < m_radius) {
					hasPointInCircle1 = true;
				}

				if (!hasPointInCircle2 && Distance_point(point_m, center2) < m_radius) {
					hasPointInCircle2 = true;
				}
			}

			if (!hasPointInCircle1 || !hasPointInCircle2) {
				cur_edges.emplace_back(Edge(point_i, point_k));

				if (false == hasPointInCircle1) {
					cur_circles.emplace_back(Circle(center1, m_radius));
				}

				if (false == hasPointInCircle2) {
					cur_circles.emplace_back(Circle(center2, m_radius));
				}

				if (m_shape_id_set.find(indexI) == m_shape_id_set.end()) {
					m_shape_id_set.emplace(indexI);
					cur_shape_points.emplace_back(point_i);
				}

				if (m_shape_id_set.find(indexK) == m_shape_id_set.end()) {
					m_shape_id_set.emplace(indexK);
					cur_shape_points.emplace_back(point_k);
				}
			}
		}
	}

	// 加锁，避免多线程资源冲突
	{
		std::lock_guard<std::mutex> lock(all_mutex);
		all_edges.insert(all_edges.end(), cur_edges.begin(), cur_edges.end());
		all_circles.insert(all_circles.end(), cur_circles.begin(), cur_circles.end());
		all_shape_points.insert(all_shape_points.end(), cur_shape_points.begin(), cur_shape_points.end());
		all_point_pair_N += cur_point_pair_N;
	}
}


// Flaan，利用多线程进行加速，考虑将边界网格内点的indexList作为传参传入
void AlphaShape::Detect_Alpah_Shape_FLANN_Multi_Thread(float radius, int threadNum) {
	if (m_projectPcl2DPoints.get() == nullptr) {
		return;
	}

	this->m_radius = radius;
	this->m_point_pair_N = 0;
	this->point_pair_scale = 0.0;

	int point_num = m_points.size();

	all_edges.clear();
	all_circles.clear();
	all_shape_points.clear();
	all_point_pair_N = 0;

	m_shape_points.clear();
	m_circles.clear();
	m_edges.clear();

	std::vector<int> allList;
	for (int i = 0; i < point_num; ++i) {
		allList.push_back(i);
	}

	int step = static_cast<int>(point_num / threadNum);

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdTree->setInputCloud(m_projectPcl2DPoints);
		
	std::vector<std::thread> threadList;
	std::vector<int> curList;
	for (int i = 0; i < threadNum; ++i) {
		curList.clear();
		if (i == (threadNum - 1)) {
			curList.assign(allList.begin() + step * i, allList.end());
		}
		else {
			curList.assign(allList.begin() + step * i, allList.begin() + step * (i + 1));
		}
		threadList.emplace_back(std::thread(Detect_Alpah_Shape_FLANN_single_thread, m_radius, curList, std::ref(m_projectPcl2DPoints), std::ref(kdTree)));
	}

	for (auto & curThread : threadList) {
		curThread.join();
	}

	std::set<Edge> edge_set(all_edges.begin(), all_edges.end());
	m_edges.assign(edge_set.begin(), edge_set.end());

	std::set<Circle> circle_set(all_circles.begin(), all_circles.end());
	m_circles.assign(circle_set.begin(), circle_set.end());

	m_shape_points.assign(all_shape_points.begin(), all_shape_points.end());
	
	m_point_pair_N = all_point_pair_N;

	this->point_pair_scale = static_cast<float>((all_point_pair_N * 2) / (point_num*(point_num - 1)));
}


// 利用PCL的FLANN加速常规Alpha-shapes算法，使得快速获取目标点的邻近点，大幅减少需要检测点的数量
void AlphaShape::Detect_Alpah_Shape_FLANN(float radius) {
	m_radius = radius;
	int point_pair_N = 0;
	this->point_pair_scale = 0.0;
	int point_num = m_points.size();

	std::unordered_set<int> m_shape_id_set;
	m_shape_id_set.reserve(point_num);
	m_shape_points.clear();
	m_shape_points.reserve(point_num);
	m_edges.clear();
	m_circles.clear();
	float distanceMax = 2 * m_radius;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	if (m_projectPcl2DPoints.get() == nullptr) {
		return;
	}
	kdTree.setInputCloud(m_projectPcl2DPoints);

	std::vector<float> disList;
	disList.reserve(point_num);
	std::vector<int> indexList;
	indexList.reserve(point_num);

	const auto &m_PclPoints = m_projectPcl2DPoints->points;
	uint32_t detectAllNum = 0;

	for (int indexI = 0; indexI < point_num;  ++indexI) {
		const auto &curP = m_PclPoints[indexI];
		// 获取与当前点距离小于滚动圆直径的其他点
		auto curNearPointNum = kdTree.radiusSearch(curP, distanceMax, indexList, disList);
		osg::Vec2 point_i(curP.x, curP.y);

		bool isCurPointDensity = true;

		if (true) {
			std::vector<pcl::PointXYZ> nearPointList;

			float step = 60.0;
			for (float angle = 0; angle < 360; angle += step) {
				float xx = curP.x + m_radius * std::cosf(osg::DegreesToRadians(angle));
				float yy = curP.y + m_radius * std::sinf(osg::DegreesToRadians(angle));
				pcl::PointXYZ nerP(xx, yy, 0.0);
				nearPointList.emplace_back(nerP);
			}

			std::vector<float> nearDisList;
			std::vector<int> nearIndexList;

			for (const auto & nearP : nearPointList) {
				if (kdTree.radiusSearch(nearP, m_radius * 0.5, nearIndexList, nearDisList) < 1) {
					isCurPointDensity = false;
					break;
				}
			}
		}

		// no need check current point, because it is high density
		if (isCurPointDensity) {
			continue;
		}

		for (int k = 0; k < curNearPointNum; ++k) {
			int indexK = indexList[k];
			if (indexK == indexI) {
				continue;
			}

			osg::Vec2 point_k(m_PclPoints[indexK].x, m_PclPoints[indexK].y);
			++point_pair_N;

			const osg::Vec2 & mid_point = (point_i + point_k) * 0.5;  //线段中点
			const osg::Vec2 & vector_line = point_i - point_k;      //线段的方向向量

			float a = 1.0, b = 1.0;

			const auto& vector_lineX = vector_line.x();
			const auto& vector_lineY = vector_line.y();

			if (abs(vector_lineX) < 0.001) {
				b = 0.0;
			}
			else {
				a = (-b * vector_lineY) / vector_lineX;
			}

			//线段的垂直向量
			osg::Vec2 normal(a, b);
			normal.normalize();//单位向量化

			// float line_length = vector_line.length() * 0.5;
			// float length = sqrt(std::pow(m_radius, 2) - std::pow(line_length, 2));

			const auto& line_length_2 = disList[k];
			float length = sqrt(std::pow(m_radius, 2) - line_length_2 * 0.25);

			//两外接圆圆心
			const osg::Vec2 &length_normal = normal * length;
			const osg::Vec2 &center1 = mid_point + length_normal;
			const osg::Vec2 &center2 = mid_point - length_normal;
			
			bool hasPointInCircle1 = false, hasPointInCircle2 = false;
			
			bool isUseKd = false;
			if (isUseKd) {
				// 效率较低，且提取效果不理想，此处不建议使用kd树再次判断
				pcl::PointXYZ pclCenter1(center1.x(), center1.y(), 0.0);
				pcl::PointXYZ pclCenter2(center2.x(), center2.y(), 0.0);
				std::vector<float> innerDisList;
				std::vector<int> innerIndexList;

				float serchDis = m_radius + 0.00001;

				if (kdTree.radiusSearch(pclCenter1, serchDis, innerIndexList, innerDisList) > 2) {
					hasPointInCircle1 = true;
				}

				if (kdTree.radiusSearch(pclCenter2, serchDis, innerIndexList, innerDisList) > 2) {
					hasPointInCircle2 = true;
				}
			} else {				
				for (int m = 0; m < curNearPointNum; ++m) {
					int indexM = indexList[m];
					if (indexM == indexK || indexM == indexI) {
						continue;
					}
					osg::Vec2 point_m(m_PclPoints[indexM].x, m_PclPoints[indexM].y);

					if (hasPointInCircle1 && hasPointInCircle2) {
						break;
					}

					if (!hasPointInCircle1 && Distance_point(point_m, center1) < m_radius) {
						hasPointInCircle1 = true;
					}

					if (!hasPointInCircle2 && Distance_point(point_m, center2) < m_radius) {
						hasPointInCircle2 = true;
					}
				}
			}

			if (!hasPointInCircle1 || !hasPointInCircle2) {
				m_edges.emplace_back(Edge(point_i, point_k));

				if (false == hasPointInCircle1) {
					m_circles.emplace_back(Circle(center1, m_radius));
				}

				if (false == hasPointInCircle2) {
					m_circles.emplace_back(Circle(center2, m_radius));
				}

				if (m_shape_id_set.find(indexI) == m_shape_id_set.end()) {
					m_shape_id_set.emplace(indexI);
					m_shape_points.emplace_back(point_i);
				}

				if (m_shape_id_set.find(indexK) == m_shape_id_set.end()) {
					m_shape_id_set.emplace(indexK);
					m_shape_points.emplace_back(point_k);
				}
			}
		}
	}

	m_point_pair_N = point_pair_N;
	this->point_pair_scale = static_cast<float>((point_pair_N * 2) / (point_num*(point_num - 1)));
}

// 方法一：常规的Alpha Shapes算法，未优化，效率较低
void AlphaShape::Detect_Shape_line(float radius) {
	m_radius = radius;
	int point_pair_N = 0;
	this->point_pair_scale = 0.0;
	int point_num = m_points.size();

	std::unordered_set<int> m_shape_id_set;
	m_shape_id_set.reserve(point_num);
	m_shape_points.clear();
	m_shape_points.reserve(point_num);
	m_edges.clear();
	m_circles.clear();
	float distanceMax = 2 * m_radius;

	for (int i = 0; i < point_num; ++i) {
		for (int k = i + 1; k < point_num; ++k) {
			// 判断任意两点的点对距离是否大于滚动圆的直径大小			
			if (Distance_point(m_points[i], m_points[k]) > distanceMax) {
				continue;
			}

			++point_pair_N;
			const osg::Vec2 & point_i = m_points[i];
			const osg::Vec2 & point_k = m_points[k];

			const osg::Vec2 & mid_point = (point_i + point_k) * 0.5;  //线段中点
			const osg::Vec2 & vector_line = point_i - point_k;      //线段的方向向量

     		float a = 1.0, b = 1.0;

			float vector_lineX = vector_line.x();
			float vector_lineY = vector_line.y();

			if (abs(vector_lineX) < 0.001) {
				b = 0.0;
			} else {
				a = (-b * vector_lineY) / vector_lineX;
			}

			//线段的垂直向量
			osg::Vec2 normal(a, b);
			normal.normalize();//单位向量化

			float line_length = vector_line.length() * 0.5;
			float length = sqrt(std::pow(m_radius, 2) - std::pow(line_length, 2));

			//两外接圆圆心
			const osg::Vec2 &length_normal = normal * length;
			const osg::Vec2 &center1 = mid_point + length_normal;
			const osg::Vec2 &center2 = mid_point - length_normal;

			bool hasPointInCircle1 = false, hasPointInCircle2 = false;

			for (int m = 0; m < point_num; ++m) {
				if (m == i || m == k) {
					continue;
				}

				const osg::Vec2 & point_m = m_points[m];

				if (hasPointInCircle1 && hasPointInCircle2) {
					break;
				}

				if (!hasPointInCircle1 && Distance_point(point_m, center1) < m_radius) {
					hasPointInCircle1 = true;
				}

				if (!hasPointInCircle2 && Distance_point(point_m, center2) < m_radius) {
					hasPointInCircle2 = true;
				}
			}

			if (!hasPointInCircle1 || !hasPointInCircle2) {
				m_edges.emplace_back(Edge(point_i, point_k));

				if (false == hasPointInCircle1) {
					m_circles.emplace_back(Circle(center1, m_radius));
				}

				if (false == hasPointInCircle2) {
					m_circles.emplace_back(Circle(center2, m_radius));
				}

				if (m_shape_id_set.find(i) == m_shape_id_set.end()) {
					m_shape_id_set.emplace(i);
					m_shape_points.emplace_back(point_i);
				}

				if (m_shape_id_set.find(k) == m_shape_id_set.end()) {
					m_shape_id_set.emplace(k);
					m_shape_points.emplace_back(point_k);
				}
			}
		}
	}

	m_point_pair_N = point_pair_N;
	this->point_pair_scale = static_cast<float>((point_pair_N * 2) / (point_num*(point_num - 1)));
}
