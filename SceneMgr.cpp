#include "SceneMgr.h"
#include <fstream>
inline void GetMeshObjectWorldAABB(MeshObject& object, std::shared_ptr<MeshData> Mesh)
{
	glm::mat4 m;
	m = glm::translate(glm::mat4(1.0f), object.WorldPos) * glm::toMat4(object.rotation) * glm::scale(glm::mat4(1.0f), object.scale);
	
	

	float lengthx = Mesh->ModelSpaceMax.x - Mesh->ModelSpaceMin.x;
	float lengthy = Mesh->ModelSpaceMax.y - Mesh->ModelSpaceMin.y;
	float lengthz = Mesh->ModelSpaceMax.z - Mesh->ModelSpaceMin.z;

	glm::vec3 v0 = Mesh->ModelSpaceMin;
	glm::vec3 v7 = Mesh->ModelSpaceMax;
	glm::vec3 v1 = Mesh->ModelSpaceMin + (glm::vec3(1, 0, 0) * lengthx);
	glm::vec3 v2 = Mesh->ModelSpaceMin + (glm::vec3(0, 0, 1) * lengthz);
	glm::vec3 v3 = Mesh->ModelSpaceMax + (glm::vec3(0, -1, 0) * lengthy);
	glm::vec3 v4 = Mesh->ModelSpaceMin + (glm::vec3(0, 1, 0) * lengthy);
	glm::vec3 v5 = Mesh->ModelSpaceMax + (glm::vec3(0, 0, -1) * lengthz);
	glm::vec3 v6 = Mesh->ModelSpaceMax + (glm::vec3(-1, 0, 0) * lengthx);
	v0 = m * glm::vec4(v0, 1.0f);
	v1 = m * glm::vec4(v1, 1.0f);
	v2 = m * glm::vec4(v2, 1.0f);
	v3 = m * glm::vec4(v3, 1.0f);
	v4 = m * glm::vec4(v4, 1.0f);
	v5 = m * glm::vec4(v5, 1.0f);
	v6 = m * glm::vec4(v6, 1.0f);
	v7 = m * glm::vec4(v7, 1.0f);
	object.WorldMin = glm::vec3(std::min({ v0.x, v1.x, v2.x, v3.x , v4.x, v5.x, v6.x, v7.x }),
								std::min({ v0.y, v1.y, v2.y, v3.y , v4.y, v5.y, v6.y, v7.y }),
								std::min({ v0.z, v1.z, v2.z, v3.z , v4.z, v5.z, v6.z, v7.z }));

	object.WorldMax = glm::vec3(std::max({ v0.x, v1.x, v2.x, v3.x , v4.x, v5.x, v6.x, v7.x }),
								std::max({ v0.y, v1.y, v2.y, v3.y , v4.y, v5.y, v6.y, v7.y }),
								std::max({ v0.z, v1.z, v2.z, v3.z , v4.z, v5.z, v6.z, v7.z }));

}
void SceneMgr::LoadMeshData(const std::string& path)
{
	using json = nlohmann::json;
	json   ij;
	std::ifstream jfile(path);
	jfile >> ij;

	size_t MeshSize = ij.at("MeshList").size();

	GeometryDatas.resize(MeshSize);
	for (int i = 0; i < MeshSize; i++)
	{
		for (int k = 0; k < ij.at("MeshList")[i].at("indices").size(); k++)
		{
			GeometryDatas[i].indices.emplace_back(ij.at("MeshList")[i].at("indices")[k]);
		}
		for (size_t j = 0; j < ij.at("MeshList")[i].at("vertices").size(); j += 3)
		{
			GeometryDatas[i].worldVertices.emplace_back(ij.at("MeshList")[i].at("vertices")[j], ij.at("MeshList")[i].at("vertices")[j + 1], ij.at("MeshList")[i].at("vertices")[j + 2]);
		}
		GeometryDatas[i].WorldMin = glm::vec3(ij.at("MeshList")[i].at("aabbMin")[0], ij.at("MeshList")[i].at("aabbMin")[1], ij.at("MeshList")[i].at("aabbMin")[2]);
		GeometryDatas[i].WorldMax = glm::vec3(ij.at("MeshList")[i].at("aabbMax")[0], ij.at("MeshList")[i].at("aabbMax")[1], ij.at("MeshList")[i].at("aabbMax")[2]);
	}
}

void SceneMgr::LoadJsonData(const std::string& path)
{
	using json = nlohmann::json;
	json senceJson;
	std::ifstream sceneFile(path + "\\" + "scene_596.json");
	sceneFile >> senceJson;
	size_t MeshObjectSize = senceJson.at("mesh").size();
	MeshObjects.reserve(MeshObjectSize);
	for (int i = 0; i < MeshObjectSize; i++)
	{
		std::string&& MeshPath = senceJson.at("mesh")[i].at("meshPath");
		auto&& worldPos = senceJson.at("mesh")[i].at("worldPos");
		auto&& worldQuat = senceJson.at("mesh")[i].at("worldQuat");
		auto&& scale = senceJson.at("mesh")[i].at("scale");

		std::replace(MeshPath.begin(), MeshPath.end(), '/', '\\');
		MeshPath = path + "\\" + MeshPath;
		MeshObjects.emplace_back(MeshPath, glm::vec3(worldPos[0], worldPos[1], worldPos[2]), glm::quat(worldQuat[3], worldQuat[0], worldQuat[1], worldQuat[2]), glm::vec3(scale[0], scale[1], scale[2]));
		if (MeshMap.count(MeshPath) < 1)
		{
			LoadMesh(MeshPath);
		}
		GetMeshObjectWorldAABB(MeshObjects[MeshObjects.size() - 1], MeshMap[MeshPath]);
		TriangleNum += MeshMap[MeshPath]->indices.size() / 3.0f;
	}
}

void SceneMgr::LoadMesh(const std::string& s)
{
	std::shared_ptr<MeshData> Mesh = std::make_shared<MeshData>();
	Mesh->MeshPathName = s;
	MeshMap[s] = Mesh;
	using json = nlohmann::json;
	json MeshJson;
	std::ifstream MeshFile(s);
	MeshFile >> MeshJson;
	for (int i = 0; i < MeshJson.at("vertices").size(); i++)
	{
		glm::vec3 Pos(MeshJson.at("vertices")[i][0], MeshJson.at("vertices")[i][1], MeshJson.at("vertices")[i][2]);
		Mesh->ModelSpaceMin = glm::vec3(std::min(Mesh->ModelSpaceMin.x,Pos.x), std::min(Mesh->ModelSpaceMin.y,Pos.y), std::min(Mesh->ModelSpaceMin.z,Pos.z));
		Mesh->ModelSpaceMax = glm::vec3(std::max(Mesh->ModelSpaceMax.x,Pos.x), std::max(Mesh->ModelSpaceMax.y,Pos.y), std::max(Mesh->ModelSpaceMax.z,Pos.z));
		Mesh->worldVertices.emplace_back(Pos);
	}
	for (int i = 0; i < MeshJson.at("indices").size(); i++)
	{
		Mesh->indices.emplace_back(MeshJson.at("indices")[i]);
	}

}
