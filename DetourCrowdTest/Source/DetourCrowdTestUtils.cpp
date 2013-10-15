//
// Copyright (c) 2013 MASA Group recastdetour@masagroup.net
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define CATCH_CONFIG_MAIN // Generate automatically the main (one occurrence only)

#include "DetourCrowdTestUtils.h"

TestScene::TestScene()
	: m_crowd(0)
{
	m_crowd = new dtCrowd;
	m_cs.m_maxRadius = 0.2f;
	m_cs.m_context = &m_bc;
}

TestScene::~TestScene()
{
	delete m_crowd;
	m_crowd = 0;
}

dtCrowd* TestScene::createSquareScene(unsigned nbMaxAgents, float maxRadius)
{
	// Creation of a square
	float* vert = new float[12];
	int* tri = new int[6];

	vert[0] = 20.0; vert[1] = 0.0; vert[2] = 20.0;
	vert[3] = 20.0; vert[4] = 0.0; vert[5] = -20.0;
	vert[6] = -20.0; vert[7] = 0.0; vert[8] = -20.0;
	vert[9] = -20.0; vert[10] = 0.0; vert[11] = 20.0;

	tri[0] = 0; tri[1] = 1; tri[2] = 2;
	tri[3] = 2; tri[4] = 3; tri[5] = 0;

	if (!m_cs.initializeScene(&m_scene, vert, 4, tri, 2)) 
		return 0;

	if (!m_cs.initializeNavmesh(m_scene, &m_navMesh)) 
		return 0;

	if (!m_crowd->init(nbMaxAgents, maxRadius, &m_navMesh)) 
		return 0;

	return m_crowd;
}

bool TestScene::defaultInitializeAgent(dtCrowd& crowd, int index) const
{
	if (index == -1)
		return false;

	dtCrowdAgent ag;
	crowd.fetchAgent(ag, index);

	ag.radius = 0.2;
	ag.height = 1.7;
	ag.maxSpeed = 2.0;
	ag.maxAcceleration = 10.0;
	ag.behavior = 0;
	ag.perceptionDistance = 4.f;

	crowd.pushAgent(ag);

	return true;
}

OffMeshConnectionCreator* TestScene::getOffMeshCreator()
{
	return &m_cs.m_creator.m_offMeshConnectionCreator;
}

namespace
{
	static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
	static const int NAVMESHSET_VERSION = 1;
	
	struct NavMeshSetHeader
	{
		int magic;
		int version;
		int numTiles;
		dtNavMeshParams params;
	};
	
	struct NavMeshTileHeader
	{
		dtTileRef tileRef;
		int dataSize;
	};
}

bool loadTiledNavMesh(dtNavMesh* navmesh, const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp)
	{
		// can't open the file
		return false;
	}
	
	// Read header.
	NavMeshSetHeader header;
	fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (header.magic != NAVMESHSET_MAGIC)
	{
		// invalid binary file header
		fclose(fp);
		return false;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		// invalid version
		fclose(fp);
		return false;
	}
	
	dtStatus status = navmesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return false;
	}
	
	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		fread(&tileHeader, sizeof(tileHeader), 1, fp);
		
		if (!tileHeader.tileRef || !tileHeader.dataSize)
		{
			// unable to read tile header #i
			fclose(fp);
			return false;
		}
		
		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data)
		{
			// unable to allocate navigation mesh data for tile #i
			fclose(fp);
			return false;
		}
		memset(data, 0, tileHeader.dataSize);
		fread(data, tileHeader.dataSize, 1, fp);
		
		navmesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}
	
	fclose(fp);
	return true;
}

bool saveTileNavmesh(const char* path, const dtNavMesh* navmesh, unsigned* tileIndices, unsigned tileCount)
{
	if (!navmesh)
	{
		// no navmesh to save.
		return false;
	}
	
	FILE* fp = fopen(path, "wb");
	if (!fp)
	{
		// can't open the file
		return false;
	}
	
	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	memcpy(&header.params, navmesh->getParams(), sizeof(dtNavMeshParams));
	if (tileIndices)
	{
		// Saving some tiles.
		for (unsigned i = 0; i < tileCount; ++i)
		{
			const dtMeshTile* tile = navmesh->getTile(tileIndices[i]);
			if (!tile || !tile->header || !tile->dataSize) continue;
			header.numTiles++;
		}
	}
	else
	{
		// Saving all tiles.
		for (unsigned i = 0; i < navmesh->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = navmesh->getTile(i);
			if (!tile || !tile->header || !tile->dataSize) continue;
			header.numTiles++;
		}
	}
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);
	
	// Store tiles.
	if (tileIndices)
	{
		// Saving some tiles.
		for (unsigned i = 0; i < tileCount; ++i)
		{
			const dtMeshTile* tile = navmesh->getTile(tileIndices[i]);
			if (!tile || !tile->header || !tile->dataSize) continue;
			
			NavMeshTileHeader tileHeader;
			tileHeader.tileRef = navmesh->getTileRef(tile);
			tileHeader.dataSize = tile->dataSize;
			fwrite(&tileHeader, sizeof(tileHeader), 1, fp);
			
			fwrite(tile->data, tile->dataSize, 1, fp);
		}
	}
	else
	{
		// Saving all tiles.
		for (unsigned i = 0; i < navmesh->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = navmesh->getTile(i);
			if (!tile || !tile->header || !tile->dataSize) continue;
			NavMeshTileHeader tileHeader;
			tileHeader.tileRef = navmesh->getTileRef(tile);
			tileHeader.dataSize = tile->dataSize;
			fwrite(&tileHeader, sizeof(tileHeader), 1, fp);
			
			fwrite(tile->data, tile->dataSize, 1, fp);
		}
	}
	
	fclose(fp);
	return true;
}
