#pragma once
#include <vector>

//单一点的结构体定义
struct Point {
	//坐标
	float p_x;
	float p_y;
	float p_z;

	//颜色
	float C_r;
	float C_g;
	float C_b;
	float C_w;

	std::vector<int> octreeIndex;
	int voxel_index[3];
	int color_type;
	//vector<int> p_depth;
};

//范围的六至边界结构体
struct point_MAXMIN {
	float xmin, ymin, zmin;
	float xmax, ymax, zmax;
};
