#include "SphereSegmentation.h"
#include "SpanData.h"
#include <algorithm>

void SphereMgr::Build(glm::vec3& Center, float radius, int Size, float s, int SphereIndex)
{
	this->CenterPos = Center;
	this->Radius = radius;
	this->TileSize = Size;
	this->Stride = s;
	this->SphereId = SphereIndex;

	unitRadianSize = float(M_PI) / (float(M_PI) * Radius / (Stride * TileSize));//计算每个Tile的弧度

	glm::mat4 m(1.0f);
	m = glm::translate(glm::mat4(1.0f), CenterPos) * glm::scale(glm::mat4(1.0f), glm::vec3(radius, radius, radius));
	int N = int(float(M_PI) / unitRadianSize + 2.0f);
	Tiles.resize(N + 1);
	float dl = 180.0f / N;
	for (int i = 0; i <= N; ++i)
	{
		float degree = -90.0f + i * dl;
		float rad = degree / 360.0f * float(M_PI) * 2.0f;
		float y = sinf(rad);
		float r = cosf(rad);
		float perimeter = r * float(M_PI) * 2;
		if (perimeter <= unitRadianSize)
		{
			Tiles[i].resize(1);
			Tile& t = Tiles[i][0];
			if (degree > 0) {
				t.CenterPos = { 0,1,0 };
				t.axis_u = { 1,0,0 };
				t.axis_v = { 0,0,-1 };
			}
			else {
				t.CenterPos = { 0,-1,0 };
				t.axis_u = { 1,0,0 };
				t.axis_v = { 0,0,1 };
			}
			t.CenterPos = m * glm::vec4(t.CenterPos, 1.0f);
			t.TileIndex = total_tiles_num;
			total_tiles_num++;
		}
		else {
			int NUM = int(perimeter / unitRadianSize + 4.0f);
			Tiles[i].resize(NUM);
			float da = float(M_PI) * 2.0f / NUM;
			for (int j = 0; j < NUM; ++j)
			{
				float lat = da * j;
				float x = r * cosf(lat);
				float z = r * sinf(lat);
				Tile& t = Tiles[i][j];
				t.CenterPos = { x,y,z };
				t.axis_u = glm::normalize(glm::cross(t.CenterPos, glm::vec3(0, 1, 0)));
				t.axis_v = glm::normalize(glm::cross(t.CenterPos, t.axis_u));
				t.CenterPos = m * glm::vec4(t.CenterPos, 1.0f);
				t.TileIndex = total_tiles_num;
				total_tiles_num++;
			}
		}
	}

	//初始化一个临时的SpanList数组
	std::vector<SpanList> tempLists;
	tempLists.reserve(total_tiles_num * TileSize * TileSize);
	SpanData::getInstance().Dictionary.emplace_back(std::make_pair(SpanData::getInstance().Data.size(), SpanData::getInstance().Data.size() + total_tiles_num * TileSize * TileSize - 1));
	for (auto&& i : Tiles)
	{
		for (auto&& j : i)
		{
			glm::vec3 centerPoint = j.CenterPos;
			glm::vec3 u = j.axis_u;
			glm::vec3 v = j.axis_v;
			glm::vec3 negu = -u;
			glm::vec3 negv = -v;
			glm::vec3 toMin = (negu * Stride * float(TileSize) / 2.0f) + (negv * Stride * float(TileSize) / 2.0f);
			glm::vec3 MinP = centerPoint + toMin;
			for (int x = 0; x < TileSize; x++)
			{
				for (int z = 0; z < TileSize; z++)
				{
					glm::vec3 ListMinPoint = MinP + (u * float(x) * Stride) + (v * float(z) * Stride);
					glm::vec3 ListCenterPoint = ListMinPoint + (u * Stride / 2.0f) + v * float(Stride / 2.0f);
					tempLists.emplace_back(ListCenterPoint, j.TileIndex, SphereIndex);

					auto&& List = tempLists[tempLists.size() - 1];
					int ListIndexNow = SpanData::getInstance().Data.size() + j.TileIndex*TileSize*TileSize + x*TileSize + z;
					if (x != 0)
					{
						List.neighborsIndex.emplace_back(ListIndexNow - 1);
					}
					else
					{
						List.neighborsIndex.emplace_back(offsetGetEdgeSpanListNeighborIndex(List, edgeNeighborDirect::NegX) + SpanData::getInstance().Data.size());
					}

					if (x != TileSize - 1)
					{
						List.neighborsIndex.emplace_back(ListIndexNow + 1);
					}
					else
					{
						List.neighborsIndex.emplace_back(offsetGetEdgeSpanListNeighborIndex(List, edgeNeighborDirect::X) + SpanData::getInstance().Data.size());
					}
					if (z != 0)
					{
						List.neighborsIndex.emplace_back(ListIndexNow - TileSize);
					}
					else
					{
						List.neighborsIndex.emplace_back(offsetGetEdgeSpanListNeighborIndex(List, edgeNeighborDirect::NegZ) + SpanData::getInstance().Data.size());
					}
					if (z != TileSize - 1)
					{
						List.neighborsIndex.emplace_back(ListIndexNow + TileSize);
					}
					else
					{
						List.neighborsIndex.emplace_back(offsetGetEdgeSpanListNeighborIndex(List, edgeNeighborDirect::Z) + SpanData::getInstance().Data.size());
					}
				}
			}
		}
	}
	SpanData::getInstance().Data.insert(SpanData::getInstance().Data.end(), tempLists.begin(), tempLists.end());
	std::vector<SpanList>().swap(tempLists);
}

int SphereMgr::getTileIndexFromWorldPos(float x, float y, float z)
{
	auto&& [longitudeIndex, patchIndex] = get2TileIndexFromWorldPos( x,y,z);
	return Tiles[longitudeIndex][patchIndex].TileIndex;
}

std::tuple<int, int> SphereMgr::get2TileIndexFromWorldPos(float x, float y, float z)
{
	glm::vec3 PositionVector = { x - CenterPos.x,y - CenterPos.y,z - CenterPos.z };
	int longitudeIndex = getLongitudeIndex(x, y, z);
	if (longitudeIndex == 0|| longitudeIndex == Tiles.size() - 1)
	{
		return std::make_tuple(longitudeIndex, 0);
	}
	int patchIndex = 0;
	float dot2 = glm::dot(glm::normalize(glm::vec3(PositionVector.x, 0, PositionVector.z)), glm::vec3(1, 0, 0));
	float radians = PositionVector.z >= 0 ? acos(dot2) : 2 * M_PI - acos(dot2);
	float HorizontalStride = 2.0f * float(M_PI) / float(Tiles[longitudeIndex].size());

	if (radians < HorizontalStride / 2.0f || (2 * M_PI - radians) < HorizontalStride / 2.0)
	{
		patchIndex = 0;
	}
	else
	{
		int tempIndex = radians / HorizontalStride;
		float left = float(tempIndex) * HorizontalStride;
		patchIndex = (radians - left) > (HorizontalStride / 2.0f) ? tempIndex + 1 : tempIndex;
	}
	return std::make_tuple(longitudeIndex, patchIndex);
}

int SphereMgr::getSpanListIndexFromWorldPos(float x, float y, float z)
{
	auto&&[longitudeIndex, patchIndex] = get2TileIndexFromWorldPos(x, y, z);
	const Tile& tile = Tiles[longitudeIndex][patchIndex];
	glm::vec3 axis_y = glm::normalize(glm::cross(tile.axis_u, tile.axis_v));
	glm::mat4 matrix = glm::mat4(glm::vec4(tile.axis_u, 0), glm::vec4(axis_y, 0), glm::vec4(tile.axis_v, 0), glm::vec4(0, 0, 0, 1));
	matrix = glm::transpose(matrix);

	glm::vec3 worldPos(x, y, z);
	worldPos = worldPos - CenterPos;
	glm::vec3 Porj2SphereWolrdPos = glm::normalize(worldPos) * Radius;

	glm::vec3 SpanListPos = matrix * glm::vec4(Porj2SphereWolrdPos, 1.0f);

	glm::vec3 centerPoint = tile.CenterPos;
	glm::vec3 u = tile.axis_u;
	glm::vec3 v = tile.axis_v;
	glm::vec3 negu = -u;
	glm::vec3 negv = -v;
	glm::vec3 toMin = (negu * Stride * float(TileSize) / 2.0f) + (negv * Stride * float(TileSize) / 2.0f);
	glm::vec3 MinP = centerPoint + toMin;
	glm::vec3 temp = (MinP - tile.CenterPos); 
	MinP = matrix * glm::vec4(temp, 1.0f);

	int xIndex = (SpanListPos.x - MinP.x) / Stride;
	int zIndex = (SpanListPos.z - MinP.z) / Stride;
	xIndex = std::clamp(xIndex, 0, TileSize - 1);

	zIndex = std::clamp(zIndex, 0, TileSize - 1);
	return tile.TileIndex*TileSize*TileSize + xIndex * TileSize + zIndex;
}

const Tile& SphereMgr::GetTileByIndex(int index)
{
	int RowbeginIndex = 0;

	for (int longitudeIndex = Tiles.size() - 1; longitudeIndex >= 0; longitudeIndex--)
	{
		if (Tiles[longitudeIndex][0].TileIndex <= index)
		{
			for (int i = 0; i < Tiles[longitudeIndex].size(); i++)
			{
				if (Tiles[longitudeIndex][i].TileIndex == index)
					return Tiles[longitudeIndex][i];
			}
		}
	}
	return Tiles[0][0];
}

int SphereMgr::getLongitudeIndex(float x, float y, float z)
{
	glm::vec3 PositionVector = { x - CenterPos.x,y - CenterPos.y,z - CenterPos.z };

	float LongitudeAngleStride = 180.0f / float(Tiles.size() - 1);
	float dot = glm::dot(glm::normalize(PositionVector), glm::vec3(0, -1, 0));
	float angle = acos(dot) * (180.0 / M_PI);

	int longitudeIndex = 0;
	int patchIndex = 0;
	if (angle < LongitudeAngleStride / 2.0)
	{
		longitudeIndex = 0;
	}
	else if ((180.0 - angle) < LongitudeAngleStride / 2.0)
	{
		longitudeIndex = Tiles.size() - 1;
	}
	else
	{
		int tempIndex = angle / LongitudeAngleStride;

		float left = LongitudeAngleStride * float(tempIndex);

		longitudeIndex = (angle - left) > (LongitudeAngleStride / 2.0f) ? tempIndex + 1 : tempIndex;
	}

	return longitudeIndex;
}

int SphereMgr::offsetGetEdgeSpanListNeighborIndex(const SpanList& sl, edgeNeighborDirect e)
{
	glm::vec3 Center = sl.CenteralWorldPos;
	glm::mat4 rotate(1.0f);
	if (e == edgeNeighborDirect::NegX)
	{

		int longitudeIndex = getLongitudeIndex(Center.x, Center.y, Center.z);
		float HorizontalRadiansStride = 360.0f / float(Tiles[longitudeIndex].size() * TileSize) / 180 * float(M_PI);
		rotate = glm::rotate(rotate, HorizontalRadiansStride, glm::vec3(0, 1, 0));
		glm::vec3 pos = rotate * glm::vec4(Center, 1.0f);
		return getSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
	}

	if (e == edgeNeighborDirect::X)
	{
		int longitudeIndex = getLongitudeIndex(Center.x, Center.y, Center.z);
		float HorizontalRadiansStride = 360.0f / float(Tiles[longitudeIndex].size() * TileSize) / 180 * float(M_PI);
		rotate = glm::rotate(rotate, HorizontalRadiansStride, glm::vec3(0, -1, 0));
		glm::vec3 pos = rotate * glm::vec4(Center, 1.0f);
		return getSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
	}


	if (e == edgeNeighborDirect::NegZ)
	{
		float LongitudeAngleStride = float(M_PI) / (float(Tiles.size() - 1) * float(TileSize));
		glm::vec3 rotateAxis = glm::normalize(glm::cross((Center - CenterPos), glm::vec3(0, 1, 0)));
		rotate = glm::rotate(rotate, LongitudeAngleStride, rotateAxis);
		glm::vec3 pos = rotate * glm::vec4(Center, 1.0f);
		return getSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
	}

	if (e == edgeNeighborDirect::Z)
	{

		float LongitudeAngleStride = float(M_PI) / (float(Tiles.size() - 1) * float(TileSize));
		glm::vec3 rotateAxis = glm::normalize(glm::cross((Center - CenterPos), glm::vec3(0, -1, 0)));
		rotate = glm::rotate(rotate, LongitudeAngleStride, rotateAxis);
		glm::vec3 pos = rotate * glm::vec4(Center, 1.0f);
		return getSpanListIndexFromWorldPos(pos.x, pos.y, pos.z);
	}
}
