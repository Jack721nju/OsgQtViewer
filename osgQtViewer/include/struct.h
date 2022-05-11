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

//三维网格顶点坐标结构体
struct point_MAXMIN {
	float xmin, ymin, zmin;
	float xmax, ymax, zmax;
};

//二维网格顶点坐标结构体
struct point2D_MAXMIN {
	float xmin, ymin;
	float xmax, ymax;

	point2D_MAXMIN() = default;
	point2D_MAXMIN(const point2D_MAXMIN & other) {
		this->xmin = other.xmin;
		this->ymin = other.ymin;
		this->xmax = other.xmax;
		this->ymax = other.ymax;
	}
};
