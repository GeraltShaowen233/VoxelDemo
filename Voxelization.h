#pragma once
#include "SphereSegmentation.h"
#include"ReCast/Recast.h"
#include <memory>

class SceneMgr;
struct Span;

namespace voxelFuncs
{
	/*
	* 用于A*测试 g和h为两个距离函数的值，f = g + h；
	* parnet：路线的上一步节点
	* sp：本wayNode代表哪个sp的上表面
	*/
	struct wayNode
	{
		float f;
		float g;
		float h;
		std::weak_ptr<wayNode> parnet;
		const Span* sp;
		wayNode(float f, float g, float h)
			:f(f), g(g), h(h)
		{

		}
	};
	/*
	* 判断两个AABB盒是否相交
	*/
	template<typename T>
	inline bool wetherAABBIntersect(const T& Amin, const T& Amax, const T& Bmin, const T& Bmax)
	{
		return Amax.x > Bmin.x && Amin.x < Bmax.x&& Amax.y > Bmin.y && Amin.y < Bmax.y&& Amax.z > Bmin.z && Amin.z < Bmax.z;
	}
	/*
	* 计算一个Tile的AABB盒，用于判断模型的AABB盒是否与这个Tile相交
	*/
	inline std::tuple<glm::vec3, glm::vec3> GetTileWorldAABB(const Tile& t, float Height,int SIZE, float stride)
	{
		glm::vec3 center = t.CenterPos;
		glm::vec3 v000 = t.CenterPos + (t.axis_u * stride * float(SIZE) / 2.0f) + (t.axis_v * stride * float(SIZE) / 2.0f);
		glm::vec3 v001 = t.CenterPos + (t.axis_u * stride * float(SIZE) / 2.0f) - (t.axis_v * stride * float(SIZE) / 2.0f);
		glm::vec3 v010 = t.CenterPos - (t.axis_u * stride * float(SIZE) / 2.0f) + (t.axis_v * stride * float(SIZE) / 2.0f);
		glm::vec3 v011 = t.CenterPos - (t.axis_u * stride * float(SIZE) / 2.0f) - (t.axis_v * stride * float(SIZE) / 2.0f);

		glm::vec3 axis_y = glm::normalize(glm::cross(t.axis_u, t.axis_v));
		glm::vec3 v100 = v000 + (axis_y * Height);
		glm::vec3 v101 = v001 + (axis_y * Height);
		glm::vec3 v110 = v010 + (axis_y * Height);
		glm::vec3 v111 = v011 + (axis_y * Height);


		glm::vec3 MinPoint = glm::vec3(std::min({ v000.x, v001.x, v010.x, v011.x , v100.x,v101.x,v110.x,v111.x }),
			std::min({ v000.y, v001.y, v010.y, v011.y , v100.y,v101.y,v110.y,v111.y }),
			std::min({ v000.z, v001.z, v010.z, v011.z , v100.z,v101.z,v110.z,v111.z }));

		glm::vec3 MaxPoint = glm::vec3(std::max({ v000.x, v001.x, v010.x, v011.x , v100.x,v101.x,v110.x,v111.x }),
			std::max({ v000.y, v001.y, v010.y, v011.y , v100.y,v101.y,v110.y,v111.y }),
			std::max({ v000.z, v001.z, v010.z, v011.z , v100.z,v101.z,v110.z,v111.z }));
		return std::make_tuple(MinPoint, MaxPoint);
	}
	
	void ReCastSphereVoxelization(const std::unique_ptr<SceneMgr>& dataPtr, const SphereMgr& Sphere, int TileSize, float cellStride, float cellHeight, float minHeight, float maxHeight);
	void ReCastSingleTileReCast(const Tile& tile, const std::unique_ptr<SceneMgr>& dataPtr, const SphereMgr& Sphere, int TileSize, float cellStride, float cellHeight, float minHeight, float maxHeight);
	void ReCastHeightFieldToSpanData(const Tile& t, rcHeightfield& hf, const SphereMgr& Sphere, float cellHeight, float minHeight);

	//Test Func
	float getSpanDistance(const Span& s1, const Span& s2, SphereMgr& sphere);
	std::vector<std::shared_ptr<wayNode>> findWays(const Span& sp1, const Span& sp2, SphereMgr& sphere);
	const Span& getRandomSpan(SphereMgr& Sphere);

}
