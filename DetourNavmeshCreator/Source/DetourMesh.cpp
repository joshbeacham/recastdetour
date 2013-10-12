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

#include "DetourMesh.h"

#include <DetourAlloc.h>
#include <DetourAssert.h>
#include <DetourCommon.h>

#include <cstring>

dtMesh::dtMesh()
: m_vertices(0)
, m_verticesCount(0)
, m_verticesCapacity(0)
, m_faces(0)
, m_normals(0)
, m_facesCount(0)
, m_facesCapacity(0)
{
	// NOTHING
}

dtMesh::~dtMesh()
{
	dtFree(m_vertices);
	dtFree(m_faces);
	dtFree(m_normals);
}

void dtMesh::addVertex(float x, float y, float z, unsigned& index)
{
	if (m_verticesCount + 1> m_verticesCapacity)
		reserve(m_verticesCount + 1, m_facesCount);

	dtVset(&m_vertices[m_verticesCount*3], x, y, z);
	index = m_verticesCount;
	++m_verticesCount;
}

void dtMesh::addFace(unsigned a, unsigned b, unsigned c, unsigned& index)
{
	if (m_facesCount + 1> m_facesCapacity)
		reserve(m_verticesCount, m_facesCount + 1);

	unsigned* it = &m_faces[m_facesCount*3];
	*it++ = a;
	*it++ = b;
	*it++ = c;
	index = m_facesCount;
	++m_facesCount;

	computeNormals(index, index + 1);
}

void dtMesh::addFace(unsigned a, unsigned b, unsigned c, float nx, float ny, float nz, unsigned& index)
{
	if (m_facesCount + 1> m_facesCapacity)
		reserve(m_verticesCount, m_facesCount + 1);

	dtVset(&m_normals[m_facesCount*3], nx, ny, nz);
	unsigned* it = &m_faces[m_facesCount*3];
	*it++ = a;
	*it++ = b;
	*it++ = c;
	index = m_facesCount;
	++m_facesCount;
}

namespace
{
	unsigned nextPow2(unsigned n)
	{
		n--;
		n |= n >> 1;
		n |= n >> 2;
		n |= n >> 4;
		n |= n >> 8;
		n |= n >> 16;
		n++;
		return n;
	}
}
void dtMesh::reserve(unsigned verticesCount, unsigned facesCount)
{
	unsigned verticesCapacity = nextPow2(dtMax(verticesCount, m_verticesCapacity));
	verticesCapacity = verticesCapacity > 0 ? dtMax<unsigned>(8, verticesCapacity) : 0;
	if (verticesCapacity != m_verticesCapacity)
	{
		float* nv = (float*)dtAlloc(3*verticesCapacity*sizeof(float), DT_ALLOC_PERM);
		if (m_verticesCount)
			memcpy(nv, m_vertices, 3*m_verticesCount*sizeof(float));
		dtFree(m_vertices);
		m_vertices = nv;
		m_verticesCapacity = verticesCapacity;
	}

	unsigned facesCapacity = nextPow2(dtMax(facesCount, m_facesCapacity));
	facesCapacity = facesCapacity > 0 ? dtMax<unsigned>(8, facesCapacity) : 0;
	if (facesCapacity != m_facesCapacity)
	{
		unsigned* nf = (unsigned*)dtAlloc(3*facesCapacity*sizeof(unsigned), DT_ALLOC_PERM);
		float* nn = (float*)dtAlloc(3*facesCapacity*sizeof(float), DT_ALLOC_PERM);
		if (m_facesCount)
		{
			memcpy(nf, m_faces, 3*m_facesCount*sizeof(unsigned));
			memcpy(nn, m_normals, 3*m_facesCount*sizeof(float));
		}
		dtFree(m_faces);
		m_faces = nf;
		dtFree(m_normals);
		m_normals = nn;
		m_facesCapacity = facesCapacity;
	}
}

void dtMesh::set(const float* vertices, unsigned verticesCount, const unsigned* faces, const float* normals, unsigned facesCount)
{
	dtAssert(vertices);
	dtAssert(faces);

	// Clearing the data structure
	m_verticesCount = m_facesCount = 0;

	reserve(verticesCount, facesCount);

	memcpy(m_vertices, vertices, 3*verticesCount*sizeof(float));
	m_verticesCount = verticesCount;

	memcpy(m_faces, faces, 3*facesCount*sizeof(unsigned));
	m_facesCount = facesCount;

	if (normals)
	{
		memcpy(m_normals, normals, 3*facesCount*sizeof(float));
	}
	else
	{
		computeNormals(0, m_facesCount);
	}
}

void dtMesh::computeNormals(unsigned from, unsigned to)
{
	unsigned a, b, c;
	float ab[3], ac[3];
	for (unsigned i(from) ; i < to ; ++i)
	{
		a = m_faces[i*3];
		b = m_faces[i*3+1];
		c = m_faces[i*3+2];

		dtAssert(a < m_verticesCount);
		dtAssert(b < m_verticesCount);
		dtAssert(c < m_verticesCount);

		dtVsub(ab, &m_vertices[3*b], &m_vertices[3*a]);
		dtVsub(ac, &m_vertices[3*c], &m_vertices[3*a]);

		dtVcross(&m_normals[3*i], ab, ac);
		dtVnormalize(&m_normals[3*i]);
	}
}
