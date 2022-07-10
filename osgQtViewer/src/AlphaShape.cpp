/* Copyright© 2022 Jack721 */
#include "AlphaShape.h"
#include <math.h>

// 声明全局变量，用于多线程处理
static std::mutex all_mutex;
static std::vector<osg::Vec2> all_shape_points;
static std::vector<int> all_index_points;
static std::vector<Edge> all_edges;
static std::vector<Circle> all_circles;
static size_t all_point_pair_N;
static size_t all_detect_num;
static std::vector<char> all_index_set;
static GridNet* all_grid_Net;

static bool checkPointSame(const osg::Vec2 &pointA, const osg::Vec2 &pointB) {
	if ((pointA - pointB).length() < 0.000001) {
		return true;
	}
	return false;
}

// 获取给定点云数据的XY最大最小范围
point_MAXMIN* GridNet::getMinMaxXYZ(const std::vector<osg::Vec2> & all_list) {
	point_MAXMIN * Max_area = new point_MAXMIN;
	std::vector<float> x_list, y_list;
	auto allNum = all_list.size();
	x_list.resize(allNum);
	y_list.resize(allNum);

	for (int i = 0; i < allNum; ++i) {
		const auto & curP = all_list[i];
		x_list[i] = curP.x();
		y_list[i] = curP.y();
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
	Points_List.assign(pList.begin(), pList.end());

	Grid_list.clear();
	pointMMM = getMinMaxXYZ(Points_List);

	grid_all_Point_Num = Points_List.size();
	grid_aver_Point_Num = 0;

	Grid_X = 0;
	Grid_Y = 0;

	Grid_Num = 0;
	Col_Num = 0;
	Row_Num = 0;

	GridOutside_Num = 0;
	GridWithPoint_Num = 0;
}

// 根据行列号获取网格
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

// 判断某点是否处于特定网格内
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

// 获取所有边界网格，并返回边界网格所有点的Index列表		
void GridNet::getAllOutSideGridPointIDList(std::vector<int> & pointIndexList) {
	int checkCenterGridNum = this->grid_aver_Point_Num * 0.75;
	int checkOutGridNum = this->grid_aver_Point_Num * 0.5;

	int eachColNum = this->Col_Num + 2;

	for (const auto & curGrid2D : this->Grid_list) {
		if (false == curGrid2D->hasPoint) {
			continue;
		}

		if (curGrid2D->cur_PointNum < checkCenterGridNum) {
			curGrid2D->isGridMayBeOutSide = true;
			pointIndexList.insert(pointIndexList.end(), curGrid2D->indexList.begin(), curGrid2D->indexList.end());
			continue;
		}

		int curGridID = curGrid2D->curGridInfo.m_ID;
		int curLeftGridID = curGridID - 1;
		int curRightGridID = curGridID + 1;

		int buttomGridID = curGridID - eachColNum;
		int buttomLeftGridID = buttomGridID - 1;
		int buttomRightGridID = buttomGridID + 1;

		int topGridID = curGridID + eachColNum;
		int topLeftGridID = topGridID - 1;
		int topRightGridID = topGridID + 1;

		std::vector<int> idList{ buttomLeftGridID, buttomGridID, buttomRightGridID, curLeftGridID, curRightGridID, topLeftGridID, topGridID, topRightGridID };

		SingleGrid2D *nearGrid2D = nullptr;
		auto girdNum = this->Grid_list.size();

		for (const auto curID : idList) {
			if (curID < 0 || curID > (girdNum - 1)) {
				continue;
			}

			nearGrid2D = this->Grid_list[curID];
			if (nullptr == nearGrid2D) {
				continue;
			}

            if (nearGrid2D->cur_PointNum < checkOutGridNum) {
				curGrid2D->isGridMayBeOutSide = true;
				pointIndexList.insert(pointIndexList.end(), curGrid2D->indexList.begin(), curGrid2D->indexList.end());
				break;
			}
		}
	}
}

// 检测并获取所有边界网格，依赖八领域网格以及中心网格的点数量作为判断依据
void GridNet::detectOutSideGrid(float radius) {
	float detectRatio = 1.0 - (radius * radius) / (Grid_X * Grid_Y);
	auto detectPointNum = this->grid_aver_Point_Num * detectRatio;
	
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

		bool isDetect = false;
		if (curGrid2D->cur_PointNum < detectPointNum) {
			curGrid2D->isGridMayBeOutSide = true;
			isDetect = true;
		}

		SingleGrid2D *nearGrid2D = nullptr;
		for (const auto curID : idList) {
			if (curID < 0 || curID > this->Grid_list.size() - 1) {
				continue;
			}

			nearGrid2D = this->Grid_list[curID];
			if (nullptr == nearGrid2D) {
				continue;
			}	

			// 重要，需要获取邻域网格的ID，因此需要遍历检测所有八领域网格
			curGrid2D->connectGridID_List.push_back(nearGrid2D->curGridInfo.m_ID);

			if (!isDetect) {
				if (nearGrid2D->cur_PointNum < detectPointNum * 0.3) {
					curGrid2D->isGridMayBeOutSide = true;
				}				
			}
		}
	}
}

// 检测每个二维网格的八邻域连通的网格，并逐一判断网格内是否有点，获取边界网格
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
		int countDensityNum = 0;

		if (curGrid2D->cur_PointNum < (this->grid_aver_Point_Num * 0.5)) {
			curGrid2D->isGridMayBeOutSide = true;
		}

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

// 根据网格大小构建二维格网，内部转为行列数调用接口
void GridNet::buildNetBySize(float SizeX, float SizeY) {
	Grid_X = SizeX;
	Grid_Y = SizeY;

	isUsingInputSize = true;

	float allPointsHeight = pointMMM->ymax - pointMMM->ymin;
	float allPointsWidth = pointMMM->xmax - pointMMM->xmin;

	this->Row_Num = (unsigned int)(ceil(allPointsHeight / Grid_Y));
	this->Col_Num = (unsigned int)(ceil(allPointsHeight / Grid_X));

	this->buildNetByNum(Row_Num, Col_Num);
}


// rowNum 表示行数，与网格的Y轴范围相关，colNum表示列数，与网格的X轴范围相关, 网格内填充为点的Index值
void GridNet::buildNetByNum(int RowNum, int ColNum) {
	this->Row_Num = (unsigned int)(RowNum);
	this->Col_Num = (unsigned int)(ColNum);

	auto minX = pointMMM->xmin;
	auto minY = pointMMM->ymin;

	auto maxX = pointMMM->xmax;
	auto maxY = pointMMM->ymax;

	if (false == isUsingInputSize) {
		float allPointsHeight = maxY - minY;
		float allPointsWidth = maxX - minX;

		this->Grid_X = static_cast<float>(allPointsWidth / Col_Num);
		this->Grid_Y = static_cast<float>(allPointsHeight / Row_Num);
	}

	GridInfo cur_grid;

	int gridNum = -1;
	// 向外部扩张一层空的网格，便于后续的邻域搜索
	for (int i = 0; i < (Row_Num + 2); ++i) {
		cur_grid.Min_Y = minY + Grid_Y * (i - 1);
		cur_grid.Max_Y = cur_grid.Min_Y + Grid_Y;
		cur_grid.m_Row = i;// 行号

		for (int j = 0; j < (Col_Num + 2); ++j) {
			cur_grid.Min_X = minX + Grid_X * (j - 1);
			cur_grid.Max_X = cur_grid.Min_X + Grid_X;
			cur_grid.m_Col = j;// 列号

			SingleGrid2D * curGrid2D = new SingleGrid2D(cur_grid);

			curGrid2D->curGridInfo.m_ID = ++gridNum;
			Grid_list.push_back(curGrid2D);
		}
	}

	this->Grid_Num = gridNum;
	int id = -1;

	auto col_Dev = (Col_Num + 2);

	for (const auto & curP : this->Points_List) {
		int col_ID = (int)((curP.x() - minX) / Grid_X) + 1;
		int row_ID = (int)((curP.y() - minY) / Grid_Y) + 1;

		auto curGridID = row_ID  * col_Dev + col_ID;
		SingleGrid2D *locateGrid = Grid_list[curGridID];

		if (locateGrid) {
			locateGrid->indexList.push_back(++id);
		}
	}

	for (const auto & eachGrid2D : Grid_list) {
		auto cur_Pnum = eachGrid2D->indexList.size();
		eachGrid2D->cur_PointNum = cur_Pnum;

		if (cur_Pnum < 1) {
			continue;
		}
		++this->GridWithPoint_Num;
		eachGrid2D->hasPoint = true;
	}

	this->grid_aver_Point_Num = (int)(grid_all_Point_Num / GridWithPoint_Num);
}

// rowNum 表示行数，colNum表示列数, 网格内填充为点的坐标值
void GridNet::buildNetByNumToPoints(int RowNum, int ColNum) {
	this->Row_Num = (unsigned int)(RowNum);
	this->Col_Num = (unsigned int)(ColNum);

	auto minX = pointMMM->xmin;
	auto minY = pointMMM->ymin;

	auto maxX = pointMMM->xmax;
	auto maxY = pointMMM->ymax;

	if (false == isUsingInputSize) {
		float allPointsHeight = maxY - minY;
		float allPointsWidth = maxX - minX;

		this->Grid_X = static_cast<float>(allPointsWidth / Col_Num);
		this->Grid_Y = static_cast<float>(allPointsHeight / Row_Num);
	}

	GridInfo cur_grid;

	int gridNum = -1;
	// 向外部扩张一层空的网格，便于后续的邻域搜索
	for (int i = 0; i < (Row_Num + 2); ++i) {
		for (int j = 0; j < (Col_Num + 2); ++j) {
			cur_grid.Min_X = minX + Grid_X * (j - 1);
			cur_grid.Max_X = cur_grid.Min_X + Grid_X;
			cur_grid.Min_Y = minY + Grid_Y * (i - 1);
			cur_grid.Max_Y = cur_grid.Min_Y + Grid_Y;

			cur_grid.m_Row = i; // 行号
			cur_grid.m_Col = j; // 列号

			SingleGrid2D * curGrid2D = new SingleGrid2D(cur_grid);

			curGrid2D->curGridInfo.m_ID = ++gridNum;
			Grid_list.push_back(curGrid2D);
		}
	}

	this->Grid_Num = gridNum;
	int id = -1;
	auto colNum = Col_Num + 2;

	for (const auto & curP : this->Points_List) {
		int col_ID = (int)((curP.x() - minX) / Grid_X) + 1;
		int row_ID = (int)((curP.y() - minY) / Grid_Y) + 1;

		auto curGridID = row_ID  * colNum + col_ID;
		if (curGridID > gridNum) {
			continue;
		}

		SingleGrid2D *locateGrid = Grid_list[curGridID];
		if (locateGrid) {
			locateGrid->PointList.emplace_back(curP);
		}
	}
	
	for (const auto & eachGrid2D : Grid_list) {
		auto cur_Pnum = eachGrid2D->PointList.size();
		eachGrid2D->cur_PointNum = cur_Pnum;

		if (cur_Pnum < 1) {
			continue;
		}
		++this->GridWithPoint_Num;
		eachGrid2D->hasPoint = true;
	}

	this->grid_aver_Point_Num = (int)(grid_all_Point_Num / GridWithPoint_Num);
}

// 根据网格数量构建格网，旧版本低效，不使用
void GridNet::buildNetByNumOld(int RowNum, int ColNum) {
	this->Row_Num = (unsigned int)(RowNum);
	this->Col_Num = (unsigned int)(ColNum);

	float allPointsHeight = pointMMM->ymax - pointMMM->ymin;
	float allPointsWidth = pointMMM->xmax - pointMMM->xmin;

	this->Grid_X = static_cast<float>(allPointsWidth / Col_Num);
	this->Grid_Y = static_cast<float>(allPointsHeight / Row_Num);

	GridInfo cur_grid;

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
	m_points.assign(point_list.begin(), point_list.end());
}

AlphaShape::AlphaShape(GridNet * curGridNet) {
	m_radius = 0.0;
	m_gridNet = curGridNet;
	if (curGridNet) {
		m_points.assign(curGridNet->Points_List.begin(), curGridNet->Points_List.end());
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

void AlphaShape::Detect_Shape_By_GridNet(float radius) {
	if (nullptr == m_gridNet) {
		return;
	}
	this->Detect_Shape_line_By_Grid(radius, m_gridNet->Grid_list);
}

void AlphaShape::Detect_Shape_line_By_Grid(float radius, const std::vector<SingleGrid2D*> & allGridList) {
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

void detectDesity() {
	bool isCurPointDensity = false;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	float m_radius = 0.0;
	pcl::PointXYZ curP;

	if (false) {
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

		bool isEmpty = false;
		for (const auto & nearP : nearPointList) {
			if (kdTree.radiusSearch(nearP, m_radius * 0.5, nearIndexList, nearDisList) < 1) {
				isEmpty = false;
				break;
			}
		}

		isCurPointDensity = !isEmpty;
	}

	// no need check current point, because it is high density
	if (isCurPointDensity) {
		return;
	}
}

/**************************************************************************************************************************************************************************/

// 单线程子函数，供方法三、五调用，多线程调用
void Detect_Alpah_Shape_FLANN_Grid_single_thread(float radius, const std::vector<int> & targetPointIndexList, pcl::PointCloud<pcl::PointXYZ>::Ptr m_projectPcl2DPoints, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree) {
	float m_radius = radius;
	int cur_point_pair_N = 0;
	int point_num = targetPointIndexList.size();

	const auto &m_PclPoints = m_projectPcl2DPoints->points;

	thread_local std::vector<osg::Vec2> cur_shape_points;
	thread_local std::vector<Circle> cur_circles;
	thread_local std::vector<Edge> cur_edges;
	thread_local std::vector<int> cur_indexs;
	thread_local std::vector<char> m_shape_id_set(m_PclPoints.size(), 0);

	float distanceMax = 2 * m_radius;

	if (kdTree.get() == nullptr) {
		return;
	}

	std::vector<float> disList;
	disList.reserve(point_num);
	std::vector<int> indexList;
	indexList.reserve(point_num);

	size_t detectAllNum = 0;

	for (int i = 0; i < point_num; ++i) {
		int indexI = targetPointIndexList[i];
		const auto &curP = m_PclPoints[indexI];
		// 获取与当前点距离小于滚动圆直径的其他点
		auto curNearPointNum = kdTree->radiusSearch(curP, distanceMax, indexList, disList);
		osg::Vec2 point_i(curP.x, curP.y);

		for (int k = 0; k < curNearPointNum; ++k) {
			int indexK = indexList[k];
			if (indexK <= indexI) {
				continue;
			}

			const auto &nearP = m_PclPoints[indexK];
			osg::Vec2 point_k(nearP.x, nearP.y);
			++cur_point_pair_N;

			const osg::Vec2 & mid_point = (point_i + point_k) * 0.5;  //线段中点
			const osg::Vec2 & vector_line = point_i - point_k;       //线段的方向向量

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
				const auto &detectP = m_PclPoints[indexM];
				osg::Vec2 point_m(detectP.x, detectP.y);

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

			detectAllNum += curNearPointNum;

			if (!hasPointInCircle1 || !hasPointInCircle2) {
				cur_edges.emplace_back(Edge(point_i, point_k));

				if (false == hasPointInCircle1) {
					cur_circles.emplace_back(Circle(center1, m_radius));
				}

				if (false == hasPointInCircle2) {
					cur_circles.emplace_back(Circle(center2, m_radius));
				}

				if (m_shape_id_set[indexI] == 0) {
					m_shape_id_set[indexI] = 1;
					cur_indexs.push_back(indexI);
					// cur_shape_points.emplace_back(point_i);
				}

				if (m_shape_id_set[indexK] == 0) {
					m_shape_id_set[indexK] = 1;
					cur_indexs.push_back(indexK);
					// cur_shape_points.emplace_back(point_k);
				}
			}
		}
	}

	// 加锁，避免多线程资源冲突
	{
		std::lock_guard<std::mutex> lock(all_mutex);
		all_edges.insert(all_edges.end(), cur_edges.begin(), cur_edges.end());
		all_circles.insert(all_circles.end(), cur_circles.begin(), cur_circles.end());
		all_index_points.insert(all_index_points.end(), cur_indexs.begin(), cur_indexs.end());
		// all_shape_points.insert(all_shape_points.end(), cur_shape_points.begin(), cur_shape_points.end());
		all_point_pair_N += cur_point_pair_N;
		all_detect_num += detectAllNum;
	}
}

// 单线程子函数，供方法七调用，多线程调用
void thread_detect_By_GridList(float radius, const std::vector<SingleGrid2D*> & outGridList, const std::vector<SingleGrid2D*> & allGridList) {
	thread_local int cur_point_pair_N = 0;
	thread_local std::vector<osg::Vec2> cur_shape_points;
	thread_local std::vector<int> cur_index_points;
	thread_local std::vector<Circle> cur_circles;
	thread_local std::vector<Edge> cur_edges;
	thread_local std::vector<int> detectAreaAllPointIndexList;
	thread_local int detectAllNum = 0;
	float disMax = 2 * radius;

	const auto & allPoints = all_grid_Net->Points_List;

	for (const auto & centerGrid : outGridList) {
		if (nullptr == centerGrid || false == centerGrid->hasPoint) {
			continue;
		}

		// 判断当前网格是否可能为边界网格
		if (!centerGrid->isGridMayBeOutSide) {
			continue;
		}

		detectAreaAllPointIndexList.clear();
		detectAreaAllPointIndexList.insert(detectAreaAllPointIndexList.end(), centerGrid->indexList.begin(), centerGrid->indexList.end());
		for (const auto nearGirdID : centerGrid->connectGridID_List) {
			const auto & curNearGrid = allGridList[nearGirdID];
			if (nullptr == curNearGrid || !curNearGrid->hasPoint) {
				continue;
			}
			detectAreaAllPointIndexList.insert(detectAreaAllPointIndexList.end(), curNearGrid->indexList.begin(), curNearGrid->indexList.end());
		}

		for (const auto & centerPointIndex : centerGrid->indexList) {
			const auto & centerPoint = allPoints[centerPointIndex];

			for (const auto & outPointIndex : detectAreaAllPointIndexList) {
				if (centerPointIndex == outPointIndex) {
					continue;
				}

				const auto & outPoint = allPoints[outPointIndex];
				if (Distance_point(centerPoint, outPoint) > disMax) {
					continue;
				}

				++cur_point_pair_N;

				const osg::Vec2 &mid_point = (centerPoint + outPoint) * 0.5;//线段中点
				const osg::Vec2 &vector_line = centerPoint - outPoint;//线段的方向向量

				float a = 1.0, b = 1.0;
				float vector_lineX = vector_line.x();
				float vector_lineY = vector_line.y();

				if (abs(vector_lineX) < 0.001) {
					b = 0.0;
				}
				else {
					a = (-b * vector_lineY) / vector_lineX;
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

				for (const auto & checkPointIndex : detectAreaAllPointIndexList) {

					if (centerPointIndex == checkPointIndex ||
						outPointIndex == checkPointIndex) {
						continue;
					}
					const auto & checkPoint = allPoints[checkPointIndex];

					++detectAllNum;

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
					bool isCenterOverlay = false;
					bool isOutOverlay = false;
					{
						std::lock_guard<std::mutex> lock(all_mutex);
						if (all_index_set[centerPointIndex] == 1) {
							isCenterOverlay = true;
						}
						if (all_index_set[outPointIndex] == 1) {
							isOutOverlay = true;
						}
					}

					if (isCenterOverlay && isOutOverlay) {
						continue;
					}

					if (!isCenterOverlay || !isOutOverlay) {
						cur_edges.emplace_back(Edge(centerPoint, outPoint));
					}

					if (false == hasPointInCircle1) {
						cur_circles.emplace_back(Circle(center1, radius));
					}

					if (false == hasPointInCircle2) {
						cur_circles.emplace_back(Circle(center2, radius));
					}

					if (!isCenterOverlay) {
						cur_index_points.push_back(centerPointIndex);
						std::lock_guard<std::mutex> lock(all_mutex);
						all_index_set[centerPointIndex] = 1;
					}

					if (!isOutOverlay) {
						cur_index_points.push_back(outPointIndex);
						std::lock_guard<std::mutex> lock(all_mutex);
						all_index_set[outPointIndex] = 1;
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
		// all_shape_points.insert(all_shape_points.end(), cur_shape_points.begin(), cur_shape_points.end());
		all_index_points.insert(all_index_points.end(), cur_index_points.begin(), cur_index_points.end());
		all_point_pair_N += cur_point_pair_N;
		all_detect_num += detectAllNum;
	}
}


// 方法八，根据pcl库的concave hull方法获取点云凹包边界，主要是对全局点云构建三角网，对三角网边界进行检测是否大于2*radius，计算量和点数量相关，与检测半径无关
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

// 方法七，对方法六进行优化，多线程检测，以点序号为索引
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
	all_index_set.resize(point_num, 0);

	all_grid_Net = m_gridNet;

	const auto & gridList = m_gridNet->Grid_list;
	int step = static_cast<int>(gridList.size() / threadNum);

	std::vector<std::thread> threadList;
	std::vector<SingleGrid2D*> curList;
	for (int i = 0; i < threadNum; ++i) {
		curList.clear();
		if (i == (threadNum - 1)) {
			curList.assign(gridList.begin() + step * i, gridList.end());
		}
		else {
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

	// m_shape_points.assign(all_shape_points.begin(), all_shape_points.end());
	std::set<int> id_set(all_index_points.begin(), all_index_points.end());
	m_shape_id.assign(id_set.begin(), id_set.end());

	m_detectAllNum = all_detect_num;
	m_point_pair_N = all_point_pair_N;
}

// 方法六，对方法一进行优化，构造二维格网，对3x3窗口网格内的点进行alpha shape检测，并不判断所有的点，以点序号为索引,从而提升检测效率
void AlphaShape::Detect_Alpha_Shape_by_Grid(float radius) {
	if (nullptr == m_gridNet) {
		return;
	}

	m_radius = radius;
	int point_num = m_points.size();

	m_shape_points.clear();
	m_circles.clear();
	m_edges.clear();
	m_detectAllNum = 0;
	m_point_pair_N = 0;

	if (point_num < 1) {
		return;
	}

	std::vector<char> m_index_set(point_num, 0);
	const auto & gridList = m_gridNet->Grid_list;

	std::vector<int> detectAreaAllPointIndexList;
	float disMax = 2 * m_radius;

	for (const auto & centerGrid : gridList) {
		if (nullptr == centerGrid || false == centerGrid->hasPoint) {
			continue;
		}

		// 判断当前网格是否可能为边界网格
		if (!centerGrid->isGridMayBeOutSide) {
			continue;
		}

		detectAreaAllPointIndexList.clear();
		detectAreaAllPointIndexList.insert(detectAreaAllPointIndexList.end(), centerGrid->indexList.begin(), centerGrid->indexList.end());
		for (const auto nearGirdID : centerGrid->connectGridID_List) {
			const auto & curNearGrid = gridList[nearGirdID];
			if (nullptr == curNearGrid || !curNearGrid->hasPoint) {
				continue;
			}
			detectAreaAllPointIndexList.insert(detectAreaAllPointIndexList.end(), curNearGrid->indexList.begin(), curNearGrid->indexList.end());
		}

		for (const auto & centerPointIndex : centerGrid->indexList) {
			const auto & centerPoint = m_points[centerPointIndex];

			for (const auto & outPointIndex : detectAreaAllPointIndexList) {
				if (centerPointIndex == outPointIndex) {
					continue;
				}

				const auto & outPoint = m_points[outPointIndex];
				if (Distance_point(centerPoint, outPoint) > disMax) {
					continue;
				}

				++m_point_pair_N;

				const osg::Vec2 &mid_point = (centerPoint + outPoint) * 0.5;//线段中点
				const osg::Vec2 &vector_line = centerPoint - outPoint;//线段的方向向量

				float a = 1.0, b = 1.0;
				float vector_lineX = vector_line.x();
				float vector_lineY = vector_line.y();

				if (abs(vector_lineX) < 0.001) {
					b = 0.0;
				}
				else {
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

				for (const auto & checkPointIndex : detectAreaAllPointIndexList) {

					if (centerPointIndex == checkPointIndex ||
						outPointIndex == checkPointIndex) {
						continue;
					}
					const auto & checkPoint = m_points[checkPointIndex];

					++m_detectAllNum;

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
					bool isCenterOverlay = false;
					bool isOutOverlay = false;
					if (m_index_set[centerPointIndex] == 1) {
						isCenterOverlay = true;
					}

					if (m_index_set[outPointIndex] == 1) {
						isOutOverlay = true;
					}

					if (isCenterOverlay && isOutOverlay) {
						continue;
					}

					if (!isCenterOverlay || !isOutOverlay) {
						m_edges.emplace_back(Edge(centerPoint, outPoint));
					}

					if (false == hasPointInCircle1) {
						m_circles.emplace_back(Circle(center1, m_radius));
					}

					if (false == hasPointInCircle2) {
						m_circles.emplace_back(Circle(center2, m_radius));
					}

					if (!isCenterOverlay) {
						m_shape_id.push_back(centerPointIndex);
						m_index_set[centerPointIndex] = 1;
					}

					if (!isOutOverlay) {
						m_shape_id.push_back(outPointIndex);
						m_index_set[outPointIndex] = 1;
					}
				}
			}
		}
	}

	//std::set<Edge> edge_set(m_edges.begin(), m_edges.end());
	//m_edges.assign(edge_set.begin(), edge_set.end());

	//std::set<Circle> circle_set(m_circles.begin(), m_circles.end());
	//m_circles.assign(circle_set.begin(), circle_set.end());
}

// 方法五，对方法二进行优化，利用多线程并行处理进一步提升速度，threadNum为线程数
void AlphaShape::Detect_Alpah_Shape_FLANN_Grid_Multi_Thread(float radius, const std::vector<int> & pointIDList, int threadNum) {
	if (m_projectPcl2DPoints.get() == nullptr) {
		return;
	}

	this->m_radius = radius;
	this->m_point_pair_N = 0;
	this->point_pair_scale = 0.0;
	this->m_detectAllNum = 0;

	int point_num = pointIDList.size();

	all_edges.clear();
	all_circles.clear();
	all_shape_points.clear();
	all_index_points.clear();
	all_point_pair_N = 0;
	all_detect_num = 0;

	m_shape_points.clear();
	m_circles.clear();
	m_edges.clear();

	int step = static_cast<int>(point_num / threadNum);

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdTree->setInputCloud(m_projectPcl2DPoints);

	std::vector<std::thread> threadList;

	for (int i = 0; i < threadNum; ++i) {
		std::vector<int> curList;
		if (i == (threadNum - 1)) {
			curList.assign(pointIDList.begin() + step * i, pointIDList.end());
		}
		else {
			curList.assign(pointIDList.begin() + step * i, pointIDList.begin() + step * (i + 1));
		}
		threadList.emplace_back(std::thread(Detect_Alpah_Shape_FLANN_Grid_single_thread, m_radius, curList, std::ref(m_projectPcl2DPoints), std::ref(kdTree)));
	}

	for (auto & curThread : threadList) {
		curThread.join();
	}

	std::set<Edge> edge_set(all_edges.begin(), all_edges.end());
	m_edges.assign(edge_set.begin(), edge_set.end());
	//m_edges.assign(all_edges.begin(), all_edges.end());

	std::set<Circle> circle_set(all_circles.begin(), all_circles.end());
	m_circles.assign(circle_set.begin(), circle_set.end());
	//m_circles.assign(all_circles.begin(), all_circles.end());

	// 强制去除若有重复index的点
	std::set<int> index_set(all_index_points.begin(), all_index_points.end());
	m_shape_id.assign(index_set.begin(), index_set.end());
	// m_shape_points.assign(all_shape_points.begin(), all_shape_points.end());

	m_point_pair_N = all_point_pair_N;
	m_detectAllNum = all_detect_num;
}

// 方法四，对方法二进行优化，构建二维格网，先筛选出边界网格和边界点，对边界点集逐一遍历，kd树加速搜索半径方位内的最近邻点
void AlphaShape::Detect_Alpah_Shape_FLANN_Grid(float radius, const std::vector<int> & pointIDList) {
	m_radius = radius;
	m_point_pair_N = 0;
	this->point_pair_scale = 0.0;
	int point_num = m_points.size();

	std::vector<char> m_shape_id_set(point_num, 0);

	m_shape_points.clear();
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
	m_detectAllNum = 0;

	for (const auto indexI : pointIDList) {
		const auto &curP = m_PclPoints[indexI];
		// 获取与当前点距离小于滚动圆直径的其他点
		auto curNearPointNum = kdTree.radiusSearch(curP, distanceMax, indexList, disList);
		osg::Vec2 point_i(curP.x, curP.y);

		for (int k = 0; k < curNearPointNum; ++k) {
			int indexK = indexList[k];

			// same point or the point was the detect point before
			if (indexK <= indexI) {
				continue;
			}

			const auto &curPK = m_PclPoints[indexK];
			osg::Vec2 point_k(curPK.x, curPK.y);
			++m_point_pair_N;

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
			int indexM = 0;
			osg::Vec2 point_m;

			for (int m = 0; m < curNearPointNum; ++m) {
				indexM = indexList[m];
				if (indexM == indexK || indexM == indexI) {
					continue;
				}
				const auto & curMP = m_PclPoints[indexM];
				point_m.set(curMP.x, curMP.y);
				++m_detectAllNum;

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

				if (m_shape_id_set[indexI] == 0) {
					m_shape_id_set[indexI] = 1;
					m_shape_id.push_back(indexI);
					// m_shape_points.emplace_back(point_i);
				}

				if (m_shape_id_set[indexK] == 0) {
					m_shape_id_set[indexK] = 1;
					m_shape_id.push_back(indexK);
					// m_shape_points.emplace_back(point_k);
				}
			}
		}
	}
}

// 方法三，对方法二进行优化，利用多线程并行处理进一步提升速度，threadNum为线程数
void AlphaShape::Detect_Alpah_Shape_FLANN_Multi_Thread(float radius, int threadNum) {
	if (m_projectPcl2DPoints.get() == nullptr) {
		return;
	}

	this->m_radius = radius;
	this->m_point_pair_N = 0;
	this->point_pair_scale = 0.0;
	m_detectAllNum = 0;

	int point_num = m_points.size();

	all_edges.clear();
	all_circles.clear();
	all_shape_points.clear();
	all_index_points.clear();
	all_point_pair_N = 0;
	all_detect_num = 0;

	m_shape_points.clear();
	m_circles.clear();
	m_edges.clear();

	std::vector<int> allList(point_num, 0);
	for (int i = 0; i < point_num; ++i) {
		allList[i] = i;
	}

	int step = static_cast<int>(point_num / threadNum);

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdTree->setInputCloud(m_projectPcl2DPoints);

	std::vector<std::thread> threadList;
	for (int i = 0; i < threadNum; ++i) {
		std::vector<int> curList;
		if (i == (threadNum - 1)) {
			curList.assign(allList.begin() + step * i, allList.end());
		}
		else {
			curList.assign(allList.begin() + step * i, allList.begin() + step * (i + 1));
		}
		threadList.emplace_back(std::thread(Detect_Alpah_Shape_FLANN_Grid_single_thread, m_radius, curList, std::ref(m_projectPcl2DPoints), std::ref(kdTree)));
	}

	for (auto & curThread : threadList) {
		curThread.join();
	}

	// std::set<Edge> edge_set(all_edges.begin(), all_edges.end());
	// m_edges.assign(edge_set.begin(), edge_set.end());
	m_edges.assign(all_edges.begin(), all_edges.end());

	// std::set<Circle> circle_set(all_circles.begin(), all_circles.end());
	// m_circles.assign(circle_set.begin(), circle_set.end());
	m_circles.assign(all_circles.begin(), all_circles.end());

	// 强制去除若有重复index的点
	std::set<int> index_set(all_index_points.begin(), all_index_points.end());
	m_shape_id.assign(index_set.begin(), index_set.end());
	// m_shape_points.assign(all_shape_points.begin(), all_shape_points.end());

	m_point_pair_N = all_point_pair_N;
	m_detectAllNum = all_detect_num;
}

// 方法二，对方法一进行优化，利用PCL的FLANN加速常规Alpha-shapes算法，使得快速获取目标点的邻近点，大幅减少需要检测点的数量
void AlphaShape::Detect_Alpah_Shape_FLANN(float radius) {
	m_radius = radius;
	m_point_pair_N = 0;
	this->point_pair_scale = 0.0;
	int point_num = m_points.size();

	std::vector<char> m_shape_id_set(point_num, 0);

	m_shape_points.clear();
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
	m_detectAllNum = 0;

	for (int indexI = 0; indexI < point_num;  ++indexI) {
		const auto &curP = m_PclPoints[indexI];
		// 获取与当前点距离小于滚动圆直径的其他点
		auto curNearPointNum = kdTree.radiusSearch(curP, distanceMax, indexList, disList);
		osg::Vec2 point_i(curP.x, curP.y);

		for (int k = 0; k < curNearPointNum; ++k) {
			int indexK = indexList[k];

			// same point or the point was the detect point before
			if (indexK <= indexI) {
				continue;
			}

			const auto &curPK = m_PclPoints[indexK];
			osg::Vec2 point_k(curPK.x, curPK.y);
			++m_point_pair_N;

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
			int indexM = 0;
			osg::Vec2 point_m;

			for (int m = 0; m < curNearPointNum; ++m) {
				indexM = indexList[m];
				if (indexM == indexK || indexM == indexI) {
					continue;
				}
				const auto & curMP = m_PclPoints[indexM];
				point_m.set(curMP.x, curMP.y);
				++m_detectAllNum;

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

				if (m_shape_id_set[indexI] == 0) {
					m_shape_id_set[indexI] = 1;
					m_shape_id.push_back(indexI);
					// m_shape_points.emplace_back(point_i);
				}

				if (m_shape_id_set[indexK] == 0) {
					m_shape_id_set[indexK] = 1;
					m_shape_id.push_back(indexK);
					// m_shape_points.emplace_back(point_k);
				}
			}
		}
	}
}

// 方法一：常规的Alpha Shapes算法，未优化，效率较低
void AlphaShape::Detect_Alpha_Shape_Default(float radius) {
	m_radius = radius;
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

			++m_point_pair_N;
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
				++m_detectAllNum;

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
}
