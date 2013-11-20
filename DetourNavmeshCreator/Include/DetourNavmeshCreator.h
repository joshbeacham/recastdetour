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

#ifndef DETOURNAVMESHCREATOR_H
#define DETOURNAVMESHCREATOR_H

#include "DetourMesh.h"
#include "DetourChunkyTriMesh.h"
#include "DetourNavmeshCfg.h"

class dtNavMesh;

struct rcCompactHeightfield;
class rcContext;
struct rcContourSet;
struct rcHeightfield;
struct rcPolyMesh;
struct rcPolyMeshDetail;

struct dtTiledNavmeshCreatorIntermediateResults
{
	dtTiledNavmeshCreatorIntermediateResults();
	~dtTiledNavmeshCreatorIntermediateResults();

		rcHeightfield* heightfield;
		unsigned char* triAreas;
		rcCompactHeightfield* compactHeightfield;
		rcContourSet* contourSet;
		rcPolyMesh* mesh;
		rcPolyMeshDetail* detailedMesh;

		void reset();
};

struct dtNavmeshInputGeometry
{
		// Default constructor
	//
	// - mesh = dtMesh()
	// - maximumFacesPerTreeNode = 256
		// - bmin = {0.f, 0.f, 0.f}
		// - bmax = {0.f, 0.f, 0.f}
		// - tree = dtChunkyTriMesh()
		dtNavmeshInputGeometry();

		dtMesh mesh;
		unsigned maximumFacesPerTreeNode;

		float bmin[3];
		float bmax[3];
		dtChunkyTriMesh tree;

		// Compute the bounding box and the tree from current mesh.
		bool initialize();
};

bool dtCreateTiledNavmesh(const dtNavmeshInputGeometry& geometry,
													const dtTiledNavmeshCfg& configuration,
													dtTiledNavmeshCreatorIntermediateResults& intermediateResults,
													dtNavMesh& navmesh,
													rcContext* context);

bool dtCreateTiledNavmesh(const dtNavmeshInputGeometry& geometry,
													const dtTiledNavmeshCfg& configuration,
													dtNavMesh& navmesh,
													rcContext* context);

bool dtCreateTiledNavmeshTile(const dtNavmeshInputGeometry& geometry,
															const dtTiledNavmeshCfg& configuration,
															dtTiledNavmeshCreatorIntermediateResults& intermediateResults,
															dtNavMesh& navmesh,
															unsigned row,
															unsigned column,
															rcContext* context);

#endif
