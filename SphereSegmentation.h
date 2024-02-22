#pragma once
#include<vector>
#include <functional>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

#define M_PI 3.1415926535

class MeshDataMgr;
/*
* 对SpanList找寻临近点用
*/
enum edgeNeighborDirect
{
	NegX,
	X,
	NegZ,
	Z
};

class Tile
{
public:

	glm::vec3 CenterPos = { 0,0,0 }; // Tile 的中心位置

	glm::vec3 axis_u = { 0,0,0 };

	glm::vec3 axis_v = { 0,0,0 };
	//两个基向量

	int TileIndex; //在本球中，这个Tile是第几个
};

class SphereMgr
{
public:
	glm::vec3 CenterPos = { 0,0,0 };
	float Radius = 0.0f;
	std::vector<std::vector<Tile>>Tiles;
	int total_tiles_num = 0;
	int TileSize = 0;
	float Stride = 0.0f;//每个SpanList的边长；
	int SphereId = 0;
public:
	/*
	* 初始化一个球，生成Tile盒SpanList，并将空的SpanList数据加入SpanData单例中
	*/
	void Build(glm::vec3& Center, float radius, int Size, float Stride,int SphereIndex);
	/*
	* 给予世界空间下的x,y,z点，获取Tile的索引(本球中)
	*/
	int getTileIndexFromWorldPos(float x, float y, float z);
	/*
	* 给予世界空间下的x,y,z点，获取Tile二维数组的索引(本球中)
	*/
	std::tuple<int,int> get2TileIndexFromWorldPos(float x, float y, float z);
	int getSpanListIndexFromWorldPos(float x, float y, float z);
	const Tile& GetTileByIndex(int index);
private:
	float unitRadianSize = 0.05f;
private:
	int getLongitudeIndex(float x, float y,float z);
	int offsetGetEdgeSpanListNeighborIndex(const struct SpanList& sl, edgeNeighborDirect);
};



