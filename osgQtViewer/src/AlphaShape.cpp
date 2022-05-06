#include "AlphaShape.h"

using namespace std;

//获取给定点云数据的XY最大最小范围
point_MAXMIN* getMinMaxXYZ(const PointV2List & all_list) {
	point_MAXMIN * Max_area = new point_MAXMIN;
	vector<float> x_list, y_list, z_list;
	for (int i = 0; i < all_list.size(); ++i) {
		x_list.push_back(all_list[i].x());
		y_list.push_back(all_list[i].y());
	}
	vector<float>::iterator xmax = max_element(begin(x_list), end(x_list));
	vector<float>::iterator ymax = max_element(begin(y_list), end(y_list));

	vector<float>::iterator xmin = min_element(begin(x_list), end(x_list));
	vector<float>::iterator ymin = min_element(begin(y_list), end(y_list));

	Max_area->xmax = *xmax;
	Max_area->ymax = *ymax;
	Max_area->xmin = *xmin;
	Max_area->ymin = *ymin;

	return Max_area;
}


SingleGrid2D::SingleGrid2D(float Grid_X, float Grid_Y){
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

GridNet::GridNet(const PointV2List &pList) {
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
	for (int i = 0; i < this->Grid_Num; ++i){
		SingleGrid2D* curGrid = this->Grid_list[i];
		if (curGrid->curGridInfo.m_Row == RowID){
			if (curGrid->curGridInfo.m_Col == ColID){
				return curGrid;
			}
		}
	}
	return nullptr;
}

bool GridNet::isPointInGrid(const osg::Vec2 & curPoint, SingleGrid2D *test_Grid){
	float cur_P_X = curPoint.x();
	float cur_P_Y = curPoint.y();

	float grid_max_x = test_Grid->curGridInfo.Max_X;
	float grid_max_y = test_Grid->curGridInfo.Max_Y;
	float grid_min_x = test_Grid->curGridInfo.Min_X;
	float grid_min_y = test_Grid->curGridInfo.Min_Y;

	if ((cur_P_X > grid_min_x)
		&& (cur_P_X < grid_max_x)
		&& (cur_P_Y > grid_min_y)
		&& (cur_P_Y < grid_max_y))	{
		return true;
	}
	else {
		return false;
	}
}

//检测每个二维网格的八邻域连通的网格，并逐一判断网格内是否有点
void GridNet::detectGridWithConnection(){
	for (const auto & curGrid2D : this->Grid_list)	{
		if (curGrid2D->hasPoint == false) {
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

		vector<int> idList{ buttomLeftGridID , buttomGridID, buttomRightGridID, curLeftGridID, curRightGridID, topLeftGridID, topGridID, topRightGridID };
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

		if (countNum == 8) {
			curGrid2D->nearByGridAllWithpoint = true;
		}
		else {
			++this->GridOutside_Num;
		}
	}
}

//根据网格数量构建格网
void GridNet::buildNetByNum(int RowNum, int ColNum){
	this->Row_Num = (unsigned int)(RowNum);
	this->Col_Num = (unsigned int)(ColNum);

	float allPointsHeight = pointMMM->ymax - pointMMM->ymin;
	float allPointsWidth = pointMMM->xmax - pointMMM->xmin;

	this->Grid_X = (float)(allPointsWidth / Col_Num);
	this->Grid_Y = (float)(allPointsHeight / Row_Num);

	GridInfo cur_grid;
	cur_grid.Size_X = Grid_X;
	cur_grid.Size_Y = Grid_Y;

	SingleGrid2D *curGrid2D = nullptr;
	int gridNum = -1;
	//向外部扩张一层空的网格，便于后续的邻域搜索
	for (int i = 0; i < (Row_Num + 2); ++i)	{
		for (int j = 0; j < (Col_Num + 2); ++j)	{
			cur_grid.Min_X = pointMMM->xmin + Grid_X*(i - 1);
			cur_grid.Max_X = cur_grid.Min_X + Grid_X;
			cur_grid.Min_Y = pointMMM->ymin + Grid_Y*(j - 1);
			cur_grid.Max_Y = cur_grid.Min_Y + Grid_Y;
			
			cur_grid.m_Row = i;//行号
			cur_grid.m_Col = j;//列号

			curGrid2D = new SingleGrid2D(cur_grid);

			if (nullptr == curGrid2D) {
				continue;
			}

			float CenterX = 0.0, CenterY = 0.0;
			for (const auto & curP : this->Points_List){
				if (isPointInGrid(curP, curGrid2D)) {
					curGrid2D->PointList.emplace_back(curP);
					CenterX += curP.x();
					CenterY += curP.y();
				}
			}

			curGrid2D->curGridInfo.m_ID = ++gridNum;
			curGrid2D->cur_PointNum = curGrid2D->PointList.size();

			if (curGrid2D->cur_PointNum > 0){
				++this->GridWithPoint_Num;
				curGrid2D->hasPoint = true;
				curGrid2D->CenterPoint.set(CenterX / curGrid2D->cur_PointNum, CenterY / curGrid2D->cur_PointNum);
			}

			Grid_list.emplace_back(curGrid2D);
		}
	}

	this->Grid_Num = gridNum;
}

//根据网格大小构建二维格网
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

//获取网格内离散点的中心点
void GridNet::getCenterPoint(){
	int haspointGridNum = 0, GridOutsideNum = 0;
	for (const auto & curGrid2D : this->Grid_list) {
		if (curGrid2D->hasPoint){
			++haspointGridNum;

			if (curGrid2D->nearByGridAllWithpoint == false){
				++GridOutsideNum;
			}		
		}
	}	
	this->GridWithPoint_Num = haspointGridNum;
	this->GridOutside_Num = GridOutsideNum;
}


//获取网格内离散点的中心点与邻域网格中心点的连接向量
void GridNet::getVectorOfOutSideGrid(){
	this->MaxVector_Grid = 0.0;
	this->MinVector_Grid = 1 << 31;

	for (const auto & curGrid : this->Grid_list){
		//当前网格内无点
		if (curGrid->hasPoint == false) {
			continue;
		}

		//当前网格的八邻域网格均含有点
		if (curGrid->nearByGridAllWithpoint == true) {
			continue;
		}

		int curRowID = curGrid->curGridInfo.m_Row;
		int curColID = curGrid->curGridInfo.m_Col;

		const osg::Vec2 & curGridCenterPoint = curGrid->CenterPoint;

		curGrid->curVectorGrid.set(0.0, 0.0);
		SingleGrid2D* nearGrid = nullptr;

		//统计当前网格的上下左右的邻域网格，而不是八网格
		for (int k = curRowID - 1; k <= curRowID + 1; ++k){
			for (int j = curColID - 1; j <= curColID + 1; ++j){
				if ((k == curRowID) && (j == curColID))	{
					continue;
				}

				if (k < 0 || j < 0)	{
					continue;
				}

				if (k >= (this->Row_Num + 2) || j >= (this->Col_Num + 2)){
					continue;
				}

				nearGrid = this->getGridByRowAndCol(k, j);

				if (nullptr == nearGrid){
					continue;
				}

				if (nearGrid->hasPoint == true)	{
					//邻域网格也属于边界网格
					if (nearGrid->nearByGridAllWithpoint == false){
						const osg::Vec2 &nearGridCenterPoint = nearGrid->CenterPoint;
						curGrid->curVectorGrid += (nearGridCenterPoint - curGridCenterPoint);
						curGrid->VectorList.emplace_back(nearGridCenterPoint - curGridCenterPoint);
					}
				}
			}
		}
		
		float curVectorDis = curGrid->curVectorGrid.length();

		if (curVectorDis > this->MaxVector_Grid){
			this->MaxVector_Grid = curVectorDis;
		}

		if (curVectorDis < this->MinVector_Grid){
			this->MinVector_Grid = curVectorDis;
		}
	}
}


static float AngleBetweenVector(osg::Vec2 vector1, osg::Vec2 vector2){
	double sin = vector1.x() * vector2.y() - vector2.x() * vector1.y();
	double cos = vector1.x() * vector2.x() + vector1.y() * vector2.y();

	return std::atan2(sin, cos) * (180.0 / 3.1415926);
}

//检测外部边界网格的平滑度，筛选出不平滑的网格，用于单独处理
void GridNet::DetectSmoothForOutSideGrid(){
	for (const auto & curGrid : this->Grid_list) {
		//当前网格内无点
		if (curGrid->hasPoint == false){
			continue;
		}

		//当前网格的八邻域网格均含有点
		if (curGrid->nearByGridAllWithpoint == true){
			continue;
		}

		//根据邻域网格的数量进行判断
		int FullgridNum = curGrid->connectGridID_List.size();
		int needCalculateGridNum = 0;

		vector<SingleGrid2D*> needCalculateGrid_list;
		SingleGrid2D* nearGrid = nullptr;
		for (int k = 0; k < FullgridNum; ++k) {
			nearGrid = this->Grid_list[curGrid->connectGridID_List[k]];
			if (nullptr == nearGrid) {
				continue;
			}

			if (nearGrid->hasPoint)	{
				if (nearGrid->nearByGridAllWithpoint == false){
					++needCalculateGridNum;
					needCalculateGrid_list.emplace_back(nearGrid);
				}
			}
		}

		//若邻域网格数量小于设定阈值，则认为粗糙度较大
		if (needCalculateGridNum < 2) {
			curGrid->isSmoothGrid = false;
			curGrid->SmoothDegree = 2;
			continue;
		}

		if (needCalculateGridNum == 7){
			curGrid->isSmoothGrid = true;
			curGrid->SmoothDegree = 0;
			continue;
		}

		if (needCalculateGridNum == 2){
			SingleGrid2D* nearGrid_1 = needCalculateGrid_list[0];
			SingleGrid2D* nearGrid_2 = needCalculateGrid_list[1];

			int col1 = nearGrid_1->curGridInfo.m_Col;
			int col2 = nearGrid_2->curGridInfo.m_Col;
			int row1 = nearGrid_1->curGridInfo.m_Row;
			int row2 = nearGrid_2->curGridInfo.m_Row;

			int curCol = curGrid->curGridInfo.m_Col;
			int curRow = curGrid->curGridInfo.m_Row;

			if ((col1 == curCol) && (col2 == curCol)){
				curGrid->isSmoothGrid = true;
				curGrid->SmoothDegree = 0;
				continue;
			}

			if ((row1 == curRow) && (row2 == curRow)){
				curGrid->isSmoothGrid = true;
				curGrid->SmoothDegree = 0;
				continue;
			}

			if (((col1 + col2) == (curCol * 2)) && ((row1 + row2) == (curRow * 2))){
				curGrid->isSmoothGrid = true;
				curGrid->SmoothDegree = 0;
				continue;
			}

			curGrid->isSmoothGrid = false;
			curGrid->SmoothDegree = 2;
			continue;
		}
		
		if ((needCalculateGridNum == 3) || (needCalculateGridNum == 4))	{
			curGrid->isSmoothGrid = false;
			curGrid->SmoothDegree = 1;
		}

		//根据邻域网格的向量夹角和合向量距离进行判断
		int cur_VectorDis = curGrid->curVectorGrid.length();
		int baseValue = (this->MaxVector_Grid - this->MinVector_Grid) / 2.5 + this->MinVector_Grid;

		int vectorNum = curGrid->VectorList.size();
		float AngleValue = 45.0;
		bool isBeyondAngle = false;

		for (int j = 0; j < vectorNum; ++j)	{
			osg::Vec2 curVector = curGrid->VectorList[j];
			for (int k = j + 1; k < vectorNum; ++k)	{
				osg::Vec2 nextVector = curGrid->VectorList[k];
				if (AngleBetweenVector(curVector, nextVector) > AngleValue) {
					isBeyondAngle = true;
				}
			}
		}

		bool islittleSmooth = false;

		if (cur_VectorDis > baseValue){
			if (isBeyondAngle){
				islittleSmooth = true;
			}
		}

		if (islittleSmooth)	{
			curGrid->isSmoothGrid = false;
			curGrid->SmoothDegree = 1;
		}
	}
}


AlphaShape::AlphaShape(const vector<osg::Vec2> & point_list){
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

AlphaShape::~AlphaShape(){
	m_radius = 0.0;

	m_points.clear();
	m_edges.clear();
	m_circles.clear();

	m_shape_id.clear();
	m_shape_points.clear();
}

static float Distance_point(osg::Vec2 pointA, osg::Vec2 pointB) {
	return std::sqrt(std::pow(pointA.x() - pointB.x(), 2) + std::pow(pointA.y() - pointB.y(), 2));
}


bool sortFun(const float & angle1, const float & angle2) {
	return angle1 < angle2;
}


//计算每个点为中心的包裹圆，统计落在圆形内部的点的数量,通过数量对中心点是否属于边界点进行判断
void AlphaShape::Detect_Shape_By_PackCirlce(GridNet* curGridNet, float radius, int pointMaxNum) {
	this->m_shape_points.clear();

	//遍历所有网格
	for (int i = 0; i < curGridNet->Grid_Num; i++)
	{
		//当前网格
		SingleGrid2D* curGrid = curGridNet->Grid_list[i];

		if (curGrid)
		{
			curGrid->hasDetected = true;//网格被检测过了
		}

		int curRowID = curGrid->curGridInfo.m_Row;
		int curColID = curGrid->curGridInfo.m_Col;

		//当前网格内无点，为空网格，直接跳过
		if (curGrid->hasPoint == false)
		{
			continue;
		}

		//当前网格的八邻域网格均含有点，说明不是边界网格，直接跳过
		if (curGrid->nearByGridAllWithpoint == true)
		{
			continue;
		}

	    //当前边界网格内点数量
		int pointListNum = curGrid->PointList.size();

		//逐一判断当前网格内点，以此点为圆心，以设置的半径画圆，判断落在圆内点的数量
		for (int k = 0; k < pointListNum; k++)
		{
			float curPointX = curGrid->PointList[k].x();
			float curPointY = curGrid->PointList[k].y();

			osg::Vec2 curPoint(curPointX, curPointY);

			int allpointNum = curGridNet->Points_List.size();

			int PointInCircleNum = 0;

			for (int m = 0; m < allpointNum; m++)
			{
				float nerPointX = curGridNet->Points_List[m].x();
				float nerPointY = curGridNet->Points_List[m].y();

				osg::Vec2 nerPoint(nerPointX, nerPointY);

				float distance = Distance_point(curPoint, nerPoint);

				if (distance< radius && distance>0.000001)
				{
					PointInCircleNum++;
				}
			}

			if (PointInCircleNum < pointMaxNum)
			{
				this->m_shape_points.push_back(curPoint);
			}
		}
	}
}

//计算每个点为中心的包裹圆，统计落在圆形内部的点的数量以及方位角，通过数量以及方位夹角对中心点是否属于边界点进行判断
void AlphaShape::Detect_Shape_By_SingleCirlce(GridNet* curGridNet, float radius, int pointNum)
{
	//用于判断检测圆是否含有点的判断点云
	vector<osg::Vec2> detect_point_list;

	for (int i = 0; i < curGridNet->Grid_Num; i++)
	{
		SingleGrid2D* curGrid = curGridNet->Grid_list[i];

		if (curGrid)
		{
			curGrid->hasDetected = true;
		}

		int curRowID = curGrid->curGridInfo.m_Row;
		int curColID = curGrid->curGridInfo.m_Col;

		//m_points.clear();

		//当前网格内无点
		if (curGrid->hasPoint == false)
		{
			continue;
		}

		//当前网格的八邻域网格均含有点
		if (curGrid->nearByGridAllWithpoint == true)
		{
			continue;
		}
		
		detect_point_list.clear();

		for (int k = curRowID - 1; k <= curRowID + 1; k++)
		{
			for (int j = curColID - 1; j <= curColID + 1; j++)
			{
				if ((k == curRowID) && (j == curColID))
				{
					//continue;
				}

				if (k < 0 || j < 0)
				{
					continue;
				}

				if (k >= (curGridNet->Row_Num + 2) || j >= (curGridNet->Col_Num + 2))
				{
					continue;
				}

				SingleGrid2D* nearGrid = curGridNet->getGridByRowAndCol(k, j);

				if (nearGrid == NULL)
				{
					continue;
				}

				if (nearGrid->hasPoint == true)
				{
					for (int m = 0; m < nearGrid->cur_PointNum; m++)
					{
						float nearPointX = nearGrid->PointList[m].x();
						float nearPointY = nearGrid->PointList[m].y();

						osg::Vec2 nearPoint(nearPointX, nearPointY);
						detect_point_list.push_back(nearPoint);
					}					
				}

			}
		}

		int pointListNum = curGrid->PointList.size();

		for (int k = 0; k < pointListNum; k++)
		{
			float curPointX = curGrid->PointList[k].x();
			float curPointY = curGrid->PointList[k].y();

			osg::Vec2 curPoint(curPointX, curPointY);

			int circlePointNum = 0;

			vector<float> angle_List;
			angle_List.clear();

			//计算落在指定检测圆内的点，并计算方位角度
			for (int m = 0; m < detect_point_list.size(); m++)
			{
				float nerPointX = detect_point_list[m].x();
				float nerPointY = detect_point_list[m].y();

				osg::Vec2 nerPoint(nerPointX, nerPointY);

				float distance = Distance_point(curPoint, nerPoint);

				if (distance< radius && distance>0.000001)
				{
					circlePointNum++;

					float deltX = nerPointX - curPointX;
					float deltY = nerPointY - curPointY;

					//atan2(y,x)所表达的意思是坐标原点为起点，指向(x,y)的射线在坐标平面上与x轴正方向之间的角的角度。
					float auizumAngle = std::atan2(deltY, deltX) * 180 / 3.1415926;
					//结果为正表示从 X 轴逆时针旋转的角度，结果为负表示从 X 轴顺时针旋转的角度。

					if (auizumAngle < 0)
					{
						//结果表示从 X 轴逆时针旋转的角度
						auizumAngle += 360.0;
					}

					angle_List.push_back(auizumAngle);
				}
			}

			std::sort(angle_List.begin(), angle_List.end(), sortFun);

			float deltAngle = 0.0;

			bool isBeyondAngle = false;

			if (angle_List.size()>0)
			{
				for (int x = 0; x < angle_List.size() - 1; x++)
				{
					int y = x + 1;

					if (y < (angle_List.size() - 1))
					{
						deltAngle = abs(angle_List[y] - angle_List[x]);
					}

					if (y == (angle_List.size() - 1))
					{
						deltAngle = (360 - angle_List[y]) + angle_List[0];
					}

					if (deltAngle > 90.0)
					{
						isBeyondAngle = true;
					}
				}
			}

			//if ((circlePointNum < pointNum)||(isBeyondAngle == true))

			if (circlePointNum < pointNum)
			{
				m_shape_points.push_back(curPoint);
			}
		}
	}
}

void AlphaShape::Detect_Shape_By_GridNet_New(float radius) {
	if (m_gridNet == nullptr)	{
		return;
	}

	vector<SingleGrid2D*> nearGrid_List;
		
	for (const auto & CenterGrid : m_gridNet->Grid_list) {
		int curRowID = CenterGrid->curGridInfo.m_Row;
		int curColID = CenterGrid->curGridInfo.m_Col;

		//当前网格内无点
		if (CenterGrid->hasPoint == false){
			continue;
		}

		//当前网格的八邻域网格均含有点
		if (CenterGrid->nearByGridAllWithpoint == true)	{
			//continue;
		}

		//清空邻域网格列表
		nearGrid_List.clear();

		//获取当前网格的八邻域网以及中心网格
		for (int k = curRowID - 1; k <= curRowID + 1; ++k){
			for (int j = curColID - 1; j <= curColID + 1; ++j){
				if (k < 0 || j < 0)	{
					continue;
				}

				if (k >= (m_gridNet->Row_Num + 2) || j >= (m_gridNet->Col_Num + 2)){
					continue;
				}

				SingleGrid2D* nearGrid = m_gridNet->getGridByRowAndCol(k, j);

				if (nearGrid == nullptr || !nearGrid->hasPoint){
					continue;
				}

				nearGrid_List.emplace_back(nearGrid);
			}
		}		

		this->Detect_Shape_line_by_Grid_New(CenterGrid, nearGrid_List, radius);
	}
}

void AlphaShape::Detect_Shape_line_by_Grid_New(SingleGrid2D* centerGrid, vector<SingleGrid2D*> nearGrid_List, float radius){
	m_radius = radius;

	int circleSize = 0;

	vector<osg::Vec2> near_point_list;
	vector<osg::Vec2> detect_point_list;

	if (centerGrid)
	{
		//表示当前中心网格已处于被检测过的状态
		centerGrid->hasDetected = true;
	}
	else
	{
		return;
	}

	//当前网格不平滑，需要缩小检测半径，检测更为细致
	if (centerGrid->isSmoothGrid == false)
	{
		circleSize = centerGrid->SmoothDegree;

		if (circleSize == 1)
		{
			m_radius = radius * 0.6;
		}

		if (circleSize == 2)
		{
			m_radius = radius * 0.6 * 0.6;
		}
	}
	else
	{
		m_radius = radius;
		circleSize = 0;
	}

	printf("The distace of curGrid and nearGrid is =========== %f \n", m_radius);
	
	//主点列表清空
	m_points.clear();

	int pointNum = centerGrid->PointList.size();

	//当前中心窗口内所有的点都作为检测判定主点
	for (int n = 0; n < pointNum; n++)
	{
		float curPointX = centerGrid->PointList[n].x();
		float curPointY = centerGrid->PointList[n].y();

		osg::Vec2 curPoint(curPointX, curPointY);

		m_points.push_back(curPoint);
	}

	//中心网格内点数量为0
	if (m_points.size() < 1)
	{
		return;
	}

	//当前所有的点都作为检测判定副点
	for (int t = 0; t < nearGrid_List.size(); t++)
	{
		SingleGrid2D* curGrid = nearGrid_List[t];

		if (curGrid->hasPoint == true)
		{
			for (int m = 0; m < curGrid->cur_PointNum; m++)
			{
				float curPointX = curGrid->PointList[m].x();
				float curPointY = curGrid->PointList[m].y();

				osg::Vec2 detectPoint(curPointX, curPointY);
				detect_point_list.push_back(detectPoint);
			}
		}
	}

	//逐一遍历所有邻域网格
	for (int t = 0; t < nearGrid_List.size(); t++)
	{
		SingleGrid2D* nearGrid = nearGrid_List[t];

		near_point_list.clear();//邻域网格的副点列表需要清空

		//当前邻域网格点数量为0
		if (nearGrid->hasPoint == false)
		{
			continue;
		}

		//当前邻域网格的八邻域网格均含有点
		if (nearGrid->nearByGridAllWithpoint == true)
		{
			//continue;
		}

		//当前网格不平滑，需要缩小检测半径，检测更为细致
		if (nearGrid->isSmoothGrid == false)
		{
			//int NearCircleSize = nearGrid->SmoothDegree;

			//if (NearCircleSize == 1)
			//{
			//	m_radius = radius * 0.8;
			//}

			//if (NearCircleSize == 2)
			//{
			//	m_radius = radius * 0.8 * 0.8;
			//}
		}

		if (nearGrid->hasPoint == true)
		{
			for (int m = 0; m < nearGrid->cur_PointNum; m++)
			{
				float nearPointX = nearGrid->PointList[m].x();
				float nearPointY = nearGrid->PointList[m].y();

				osg::Vec2 nearPoint(nearPointX, nearPointY);
				near_point_list.push_back(nearPoint);
			}
		}
		
		//逐一判断邻域网格与中心网格的离散点的主副点
		for (int i = 0; i < m_points.size(); i++){
			for (int k = 0; k < near_point_list.size(); k++)
			{
				float m_distance = Distance_point(m_points[i], near_point_list[k]);

				//两点距离过大
				if (m_distance > 2 * m_radius)
				{
					continue;
				}

				//距离过小，视同一个点
				if (m_distance < 0.000001)
				{
					continue;
				}

				osg::Vec2 center1, center2;//两外接圆圆心

				osg::Vec2 mid_point = (m_points[i] + near_point_list[k]) / 2;//线段中点

				osg::Vec2 vector_line = m_points[i] - near_point_list[k];//线段的方向向量

				float a = 1.0, b = 1.0;
				osg::Vec2 normal;

				if (abs(vector_line.x() - 0) < 0.01)
				{
					b = 0.0;
				}
				else
				{
					a = (-b * vector_line.y()) / vector_line.x();
				}

				normal.set(a, b);//线段的垂直向量
				normal.normalize();//单位向量化

				float line_length = vector_line.length() / 2.0;

				float length = sqrt(m_radius*m_radius - line_length*line_length);
				center1 = mid_point + normal*length;
				center2 = mid_point - normal*length;

				bool hasPointInCircle1 = false, hasPointInCircle2 = false;

				//判断是否有检测点落在检测圆内
				for (int m = 0; m < detect_point_list.size(); m++)
				{
					float m_distance1 = Distance_point(m_points[i], detect_point_list[m]);
					float m_distance2 = Distance_point(near_point_list[k], detect_point_list[m]);

					if (m_distance1 < 0.000001 || m_distance2 < 0.000001)
					{
						continue;
					}

					if (hasPointInCircle1&&hasPointInCircle2)
					{
						break;
					}

					if (!hasPointInCircle1 && Distance_point(detect_point_list[m], center1) < m_radius)
					{
						hasPointInCircle1 = true;
					}

					if (!hasPointInCircle2 && Distance_point(detect_point_list[m], center2) < m_radius)
					{
						hasPointInCircle2 = true;
					}
				}

				osg::Vec2 vector_to_center1 = center1 - mid_point;
				osg::Vec2 vector_to_center2 = center2 - mid_point;

				bool addCircle1 = true;
				bool addCircle2 = true;

				//若内、外侧圆均含有落点，则直接跳过
				if (hasPointInCircle1 && hasPointInCircle2)	{
					continue;
				}

				//若没有落点在圆内，则继续操作
				if (hasPointInCircle1 != true || hasPointInCircle2 != true)	{
					bool hasPointA = false;
					bool hasPointB = false;
				
					//判断是否有重复边界点
					for (int j = 0; j < m_shape_points.size(); j++)
					{
						float disA = Distance_point(m_shape_points[j], m_points[i]);
						float disB = Distance_point(m_shape_points[j], near_point_list[k]);

						if (disA < 0.000001)
						{
							hasPointA = true;
						}

						if (disB < 0.000001)
						{
							hasPointB = true;
						}
					}

					if (hasPointA == false)
					{
						m_shape_points.push_back(m_points[i]);
					}

					if (hasPointB == false)
					{
						m_shape_points.push_back(near_point_list[k]);
					}

					Edge each_edge;

					each_edge.point_A = m_points[i];
					each_edge.point_B = near_point_list[k];
					
					//录入边界线和检测圆
					{
						m_edges.push_back(each_edge);

						if (hasPointInCircle1 != true)
						{
							Circle each_circle;

							each_circle.m_center = center1;
							each_circle.m_radius = m_radius;
							each_circle.size = circleSize;

							m_circles.push_back(each_circle);
						}

						if (hasPointInCircle2 != true)
						{
							Circle each_circle;

							each_circle.m_center = center2;
							each_circle.m_radius = m_radius;
							each_circle.size = circleSize;

							m_circles.push_back(each_circle);
						}
					}

				}
			}
		}

	}
}

static bool checkPointSame(osg::Vec2 pointA, osg::Vec2 pointB) {
	if ((pointA - pointB).length() < 0.000001) {
		return true;
	}
	return false;
}


//用于多线程处理
static std::mutex all_mutex;
static std::vector<osg::Vec2> all_shape_points;
static std::vector<Edge> all_edges;
static std::vector<Circle> all_circles;
static int all_point_pair_N;

//基于网格筛选的alpha shape处理函数，供多线程调用
void thread_detect_By_GridList(float radius, const std::vector<SingleGrid2D*> & centerGridList, const std::vector<SingleGrid2D*> & allGridList){
	thread_local int cur_point_pair_N = 0;

	thread_local PointV2List cur_shape_points;
	thread_local std::vector<Circle> cur_circles;
	thread_local std::vector<Edge> cur_edges;

	thread_local std::vector<osg::Vec2> detectAreaAllPointList;

	for (const auto & centerGrid : centerGridList) {
		if (centerGrid->hasPoint == false) {
			continue;
		}

		//若邻域网格内点数量较少且较为离散，可能会出现漏检情况
		if (centerGrid->nearByGridAllWithpoint) {
			//continue;
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
				if (Distance_point(centerPoint, outPoint) > 2 * radius) {
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
				}
				else {
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

	//thread_local std::set<Edge> edge_set(cur_edges.begin(), cur_edges.end());
	//cur_edges.assign(edge_set.begin(), edge_set.end());

	//thread_local std::set<Circle> circle_set(cur_circles.begin(), cur_circles.end());
	//cur_circles.assign(circle_set.begin(), circle_set.end());

	{
		std::lock_guard<std::mutex> lock(all_mutex);
		all_edges.insert(all_edges.end(), cur_edges.begin(), cur_edges.end());
		all_circles.insert(all_circles.end(), cur_circles.begin(), cur_circles.end());
		all_shape_points.insert(all_shape_points.end(), cur_shape_points.begin(), cur_shape_points.end());
		all_point_pair_N += cur_point_pair_N;
	}
}


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
	int step = (int)(gridList.size() / threadNum);
	
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

	this->point_pair_scale = (float)(all_point_pair_N * 2) / (point_num*(point_num - 1));
}

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

	for (const auto & centerGrid : gridList) {
		if (nullptr == centerGrid || centerGrid->hasPoint == false) {
			continue;
		}

		//若邻域网格内点数量较少且较为离散，可能会出现漏检情况
		if (centerGrid->nearByGridAllWithpoint) {
			//continue;
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
				if (Distance_point(centerPoint, outPoint) > 2 * m_radius) {
					continue;
				}

				if (checkPointSame(centerPoint, outPoint)) {
					continue;
				}

				++point_pair_N;

				const osg::Vec2 &mid_point = (centerPoint + outPoint) / 2;//线段中点
				const osg::Vec2 &vector_line = centerPoint - outPoint;//线段的方向向量

				float a = 1.0, b = 1.0;

				if (abs(vector_line.x()) < 0.001) {
					b = 0.0;
				}
				else {
					a = (-b * vector_line.y()) / vector_line.x();
				}

				//线段的垂直向量
				osg::Vec2 normal(a, b);
				normal.normalize();//单位向量化

				float line_length = vector_line.length() / 2.0;
				float length = sqrt(std::pow(m_radius, 2) - std::pow(line_length, 2));

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

	this->point_pair_scale = (float)(point_pair_N * 2) / (point_num*(point_num - 1));
}

//常规的Alpha Shapes算法
void AlphaShape::Detect_Shape_line(float radius) {
	m_radius = radius;
	int point_pair_N = 0;
	this->point_pair_scale = 0.0;
	int point_num = m_points.size();

	std::unordered_set<int> m_shape_id_set;
	m_shape_id_set.reserve(point_num);
	m_shape_points.clear();
	m_shape_points.reserve(point_num);

	m_circles.clear();

	for (int i = 0; i < point_num; ++i){
		for (int k = i + 1; k < point_num; ++k) {
			//判断任意两点的点对距离是否大于滚动圆的直径大小
			if (Distance_point(m_points[i], m_points[k]) > 2 * m_radius){
				continue;
			}

			++point_pair_N;
			
			const osg::Vec2 &mid_point = (m_points[i] + m_points[k]) / 2;//线段中点
			const osg::Vec2 &vector_line = m_points[i] - m_points[k];//线段的方向向量

			float a = 1.0, b = 1.0;			

			if (abs(vector_line.x()) < 0.001) {
				b = 0.0;
			}else {
				a = (-b * vector_line.y()) / vector_line.x();
			}

			//线段的垂直向量
			osg::Vec2 normal(a, b);
			normal.normalize();//单位向量化

			float line_length = vector_line.length() / 2.0;
			float length = sqrt(std::pow(m_radius, 2) - std::pow(line_length, 2));

			//两外接圆圆心
			const osg::Vec2 &center1 = mid_point + normal*length;
			const osg::Vec2 &center2 = mid_point - normal*length;

			bool hasPointInCircle1 = false, hasPointInCircle2 = false;

			for (int m = 0; m < point_num; ++m) {
				if (m == i || m == k) {
					continue;
				}

				if (hasPointInCircle1 && hasPointInCircle2){
					break;
				}

				if (!hasPointInCircle1 && Distance_point(m_points[m], center1) < m_radius){
					hasPointInCircle1 = true;
				}

				if (!hasPointInCircle2 && Distance_point(m_points[m], center2) < m_radius){
					hasPointInCircle2 = true;
				}
			}

			if (!hasPointInCircle1 || !hasPointInCircle2){
				m_edges.emplace_back(Edge(m_points[i], m_points[k]));

				if (false == hasPointInCircle1) {
					m_circles.emplace_back(Circle(center1, m_radius));
				}

				if (false == hasPointInCircle2) {
					m_circles.emplace_back(Circle(center2, m_radius));
				}
			
				if (m_shape_id_set.find(i) == m_shape_id_set.end()) {
					m_shape_id_set.emplace(i);
					m_shape_points.emplace_back(m_points[i]);
				}
				
				if (m_shape_id_set.find(k) == m_shape_id_set.end()) {
					m_shape_id_set.emplace(k);
					m_shape_points.emplace_back(m_points[k]);
				}
			}
		}
	}

	this->point_pair_scale = (float)(point_pair_N * 2)/ (point_num*(point_num - 1));
}
