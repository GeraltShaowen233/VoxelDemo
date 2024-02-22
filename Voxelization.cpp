#include"Voxelization.h"

#include <array>
#include"SpanData.h"
#include "SceneMgr.h"
#define MaxDepth 20000

namespace voxelFuncs
{

	std::vector<std::shared_ptr<wayNode>> findWays(const Span& sp1, const Span& sp2, SphereMgr& sphere)
	{
		std::shared_ptr<wayNode> start = std::make_shared<wayNode>(0, 0, 0);
		start->sp = &sp1;

		std::shared_ptr<wayNode> end = std::make_shared<wayNode>(0, 0, 0);
		end->sp = &sp2;

		int SearchCount = 0;
		std::vector<std::shared_ptr<wayNode>> open_list;
		std::vector<std::shared_ptr<wayNode>> closed_list;

		open_list.emplace_back(start);

		while (!open_list.empty())
		{
			std::shared_ptr<wayNode> current_node = open_list[0];
			int current_index = 0;

			for (int i = 0; i < open_list.size(); ++i)
			{
				if (open_list[i]->f < current_node->f)
				{
					current_node = open_list[i];
					current_index = i;
				}
			}
			closed_list.push_back(current_node);
			std::swap(open_list[current_index], open_list[open_list.size() - 1]);
			open_list.pop_back();
			SearchCount++;
			if (SearchCount > MaxDepth)
			{
				break;
			}

			if (current_node->sp == end->sp)
			{
				std::vector<std::shared_ptr<wayNode>> path;
				std::shared_ptr<wayNode> current = current_node;
				while (current != nullptr)
				{
					path.push_back(current);
					current = current->parnet.lock();
				}
				std::reverse(path.begin(), path.end());
				return path;
			}

			std::vector<std::shared_ptr<wayNode>> neighbors;
			auto&& instance = SpanData::getInstance();
			for (int i = 0; i < instance.Data[current_node->sp->ListIndex].neighborsIndex.size(); i++)
			{
				const SpanList& List = instance.Data[current_node->sp->ListIndex];
				const SpanList& neighborsList = instance.Data[List.neighborsIndex[i]];
				for (int j = 0; j < neighborsList.Spans.size(); j++)
				{
					auto&& searchSpan = neighborsList.Spans[j];
					if (std::abs(searchSpan.top - current_node->sp->top) > 2* sphere.Stride)
					{
						continue;
					}
					std::shared_ptr<wayNode> newNode = std::make_shared<wayNode>(0, 0, 0);
					newNode->sp = &searchSpan;
					neighbors.emplace_back(newNode);
				}
			}

			for (std::shared_ptr<wayNode> n : neighbors)
			{
				bool inClosedList = false;
				for (int i = 0; i < closed_list.size(); i++)
				{
					if (closed_list[i]->sp == n->sp)
					{
						inClosedList = true;
					}
				}
				if (inClosedList)
					continue;

				n->g = current_node->g + sphere.Stride;
				n->h = getSpanDistance(*n->sp, *end->sp, sphere);
				n->f = n->g + n->h;
				n->parnet = current_node;

				open_list.push_back(n);
			}
		}

		return std::vector<std::shared_ptr<wayNode>>();

	}

	const Span& getRandomSpan(SphereMgr& Sphere)
	{
		int beginIndex = SpanData::getInstance().Dictionary[Sphere.SphereId].first;
		int endIndex = SpanData::getInstance().Dictionary[Sphere.SphereId].second;
		int range = endIndex - beginIndex + 1;
		int RandomListIndex;
		int RandomSpanIndex;
		while (true)
		{
			RandomListIndex = rand() % range + beginIndex;
			
			if (SpanData::getInstance().Data[RandomListIndex].Spans.size() > 0)
			{
				RandomSpanIndex = rand() % (SpanData::getInstance().Data[RandomListIndex].Spans.size());
				break;
			}
		}
		return  SpanData::getInstance().Data[RandomListIndex].Spans[RandomSpanIndex];
	}

	void ReCastSphereVoxelization(const std::unique_ptr<SceneMgr>& dataPtr, const SphereMgr& Sphere, int TileSize, float cellStride, float cellHeight, float minHeight, float maxHeight)
	{
		for (int i = 0; i < Sphere.Tiles.size(); i++)
		{
			for (int j = 0; j < Sphere.Tiles[i].size(); j++)
			{
				ReCastSingleTileReCast(Sphere.Tiles[i][j], dataPtr, Sphere,TileSize, cellStride, cellHeight, minHeight, maxHeight);
			}
		}
	}
	
	void ReCastSingleTileReCast(const Tile& tile, const std::unique_ptr<SceneMgr>& dataPtr, const SphereMgr& Sphere, int TileSize, float cellStride, float cellHeight, float minHeight, float maxHeight)
	{
		auto&& [MinPoint, MaxPoint] = GetTileWorldAABB(tile, float(maxHeight),TileSize,cellStride);
		std::vector<MeshObject> GeosInthisTile;
		rcHeightfield* HeightField = nullptr;
		HeightField = rcAllocHeightfield();
		
		float HeightFiledMin[3] = { -TileSize * cellStride / 2.0f, minHeight,-TileSize * cellStride / 2.0f };
		float HeightFiledMax[3] = { TileSize * cellStride / 2.0f, maxHeight ,TileSize * cellStride / 2.0f };
		rcCreateHeightfield(nullptr, *HeightField, TileSize, TileSize, HeightFiledMin, HeightFiledMax, cellStride, cellHeight);

		for (int i = 0; i < dataPtr->MeshObjects.size(); i++)
		{
			const MeshObject& GD = dataPtr->MeshObjects[i];
			if (wetherAABBIntersect(GD.WorldMin, GD.WorldMax, MinPoint, MaxPoint))
			{
				GeosInthisTile.emplace_back(GD);
			}
		}

		for (int i = 0; i < GeosInthisTile.size(); i++)
		{
			glm::vec3 axis_y = glm::normalize(tile.CenterPos);
			glm::mat4 matrix = glm::mat4(glm::vec4(tile.axis_u, 0), glm::vec4(axis_y, 0), glm::vec4(tile.axis_v, 0), glm::vec4(0, 0, 0, 1));
			matrix = glm::transpose(matrix);
			glm::mat4 m;
			m = glm::translate(glm::mat4(1.0f), GeosInthisTile[i].WorldPos) * glm::toMat4(GeosInthisTile[i].rotation) * glm::scale(glm::mat4(1.0f), GeosInthisTile[i].scale);
			std::shared_ptr<MeshData> MeshD = dataPtr->MeshMap[GeosInthisTile[i].MeshPathName];
			std::vector<float> vecs;
			vecs.reserve(MeshD->worldVertices.size() * 3);
			for (int index = 0; index < MeshD->worldVertices.size(); index++)
			{
				glm::vec3 worldPos = m * glm::vec4(MeshD->worldVertices[index], 1.0f);
				glm::vec3 newPos = matrix * glm::vec4(worldPos - tile.CenterPos, 1.0f);
				vecs.emplace_back(newPos.x);
				vecs.emplace_back(newPos.y);
				vecs.emplace_back(newPos.z);
			}
			static thread_local rcContext ctx;
			auto triareas = std::make_unique<unsigned char[]>(MeshD->indices.size() / 3);
			memset(triareas.get(), RC_WALKABLE_AREA, MeshD->indices.size() / 3 * sizeof(unsigned char));
			rcRasterizeTriangles(&ctx, vecs.data(), MeshD->worldVertices.size(), MeshD->indices.data(), triareas.get(), MeshD->indices.size() / 3, *HeightField, 10000);
			vecs.swap(std::vector<float>());
		}

		ReCastHeightFieldToSpanData(tile, *HeightField, Sphere, cellHeight, minHeight);
		rcFreeHeightField(HeightField);
		HeightField = nullptr;
	}
	
	void ReCastHeightFieldToSpanData(const Tile& t, rcHeightfield& hf, const SphereMgr& Sphere,float cellHeight,float minHeight)
	{
		auto&& instance = SpanData::getInstance();
		int beginIndex = instance.Dictionary[Sphere.SphereId].first;
		int TileSpanListBeginIndex = beginIndex + t.TileIndex * Sphere.TileSize * Sphere.TileSize;

		for (int x = 0; x < Sphere.TileSize; x++)
		{
			for (int z = 0; z < Sphere.TileSize; z++)
			{
				int Index = x + (uint64_t)z * hf.width;
				auto&& List = instance.Data[TileSpanListBeginIndex + Index];
				rcSpan* s = hf.spans[Index];
				if (s)
				{
					while (s)
					{
						List.Spans.emplace_back(s->smin * cellHeight + minHeight, s->smax * cellHeight + minHeight, TileSpanListBeginIndex + Index);
						s = s->next;
					}
				}
			}
		}
	}

	float getSpanDistance(const Span& s1, const Span& s2, SphereMgr& sphere)
	{
		auto&& instance = SpanData::getInstance();
		auto&& Tile1 = sphere.GetTileByIndex(instance.Data[s1.ListIndex].TileIndex);
		auto&& Tile2 = sphere.GetTileByIndex(instance.Data[s2.ListIndex].TileIndex);

		glm::vec3 axis_y1 = glm::normalize(glm::cross(Tile1.axis_u, Tile1.axis_v));
		glm::vec3 axis_y2 = glm::normalize(glm::cross(Tile2.axis_u, Tile2.axis_v));

		glm::vec3 p1 = instance.Data[s1.ListIndex].CenteralWorldPos + (axis_y1 * s1.top);
		glm::vec3 p2 = instance.Data[s2.ListIndex].CenteralWorldPos + (axis_y2 * s2.top);
		return glm::distance(p1, p2);
	}

}
