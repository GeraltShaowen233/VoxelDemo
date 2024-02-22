#pragma once
#include "nlohmann/json.hpp"
#include"glm/glm.hpp"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

/*
* 三角形汤，暂时不用
*/
struct GeometryData
{
	std::vector<glm::vec3> worldVertices;
	std::vector<int>           indices;
	GeometryData() = default;
	glm::vec3 WorldMin = glm::vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	glm::vec3 WorldMax = glm::vec3(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
};
/*
* 模型坐标系下的模型数据
*/
struct MeshData
{
	std::string MeshPathName;
	std::vector<glm::vec3> worldVertices;
	std::vector<int>           indices;
	MeshData() = default;
	glm::vec3 ModelSpaceMin = glm::vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	glm::vec3 ModelSpaceMax = glm::vec3(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
};

/*
* 世界坐标系下的模型数据，只包括平移，旋转的缩放。顺便已计算好了世界坐标系下的AABB盒
*/
struct MeshObject
{
	std::string MeshPathName;
	glm::vec3 WorldPos;
	glm::quat rotation;
	glm::vec3 scale;
	glm::vec3 WorldMin;
	glm::vec3 WorldMax;
	MeshObject() = default;
	MeshObject(const std::string& str,const glm::vec3& w,const glm::quat& q,const glm::vec3& scale)
		:MeshPathName(str),WorldPos(w),rotation(q),scale(scale)
	{

	}
	MeshObject(const std::string&& str, const glm::vec3&& w, const glm::quat&& q, const glm::vec3&& scale)
		:MeshPathName(str), WorldPos(w), rotation(q), scale(scale)
	{

	}

};

class SceneMgr
{
public:
	void LoadMeshData(const std::string& s = "D:\\master\\bin\\asset\\Test2\\596.json");
	void LoadJsonData(const std::string& s = "D:\\master\\bin\\asset\\Test2\\JsonData2");
	std::vector<GeometryData> GeometryDatas;//三角形汤
	std::vector<MeshObject> MeshObjects;
	std::unordered_map<std::string, std::shared_ptr<MeshData>> MeshMap;//根据Mesh的路径名获取Mesh的指针
	int TriangleNum = 0;
private:
	void LoadMesh(const std::string& s);
};
