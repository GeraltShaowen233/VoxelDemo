#pragma once
#include<vector>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include <memory>

/*
* 每个高度场中的一个Span
* bottom，top：距离高度场底面的距离
* ListIndex：从属于哪一个SpanList，此Index用于整个SpanListData中
*/
struct Span
{
	float bottom;
	float top;
	int ListIndex;
	Span(float b, float t, int index)
		:bottom(b), top(t), ListIndex(index)
	{
	}
	Span() = default;
};
/*
* 每个高度场中的一个SpanList
* Spans：保存Span的数组
* CenteralWorldPos：底中心点在世界坐标系下的位置
* neighborsIndex：邻居List在整个SpanListData中的索引
* TileIndex：从属于的Tile编号，需要先知道是哪一个球，再使用此编号
* SphereIndex：从属于的球编号
*/
struct SpanList
{
	std::vector<Span> Spans;
	glm::vec3 CenteralWorldPos;
	std::vector<int> neighborsIndex;
	int TileIndex;
	int SphereIndex;
	SpanList(const glm::vec3& Pos, int TI, int SI)
		:CenteralWorldPos(Pos), TileIndex(TI), SphereIndex(SI)
	{

	}
};


/*
* 所有的数据所组成的结构，单例
* Data：保存所有的数据
* Dictionary：保存有每个球的起始index
*			  eg：Dictionary[0] = [0,10000] 代码编号从0到10000的SpanList都属于球0
*/
class SpanData
{
private:
	SpanData() {};
	~SpanData() {};
	SpanData(const SpanData&);
	SpanData& operator=(const SpanData&) = delete;
public:
	static SpanData& getInstance()
	{
		static SpanData Instance;
		return Instance;
	}
	
public:
	std::vector<SpanList> Data;
	std::vector<std::pair<int, int>> Dictionary;
};
