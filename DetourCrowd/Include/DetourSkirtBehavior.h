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

#ifndef DETOURSKIRTBEHAVIOR_H
#define DETOURSKIRTBEHAVIOR_H

#include "DetourSteeringBehavior.h"

struct dtCrowdAgent;

/// Neighbor agent represented as an obstacle
struct dtAgentObstacle
{
	float position[3];
	float direction[3];         ///< Normalized vector from the obstacle to the agent
	float radius;
	unsigned int id;
};
/// Navmesh sgment represented as an obstacle
struct dtSegmentObstacle
{
	float closest[2];
};

/// Parameters for the skirting behavior
/// @ingroup behavior
struct dtSkirtBehaviorParams
{
	float targetPos[3];         ///< Target position
	bool init(const float* position);
};


/// Defines the skirting behavior.
///
/// An agent using this behavior will try to skirt around (circumvent) still targets.
/// @ingroup behavior
class dtSkirtBehavior : public dtSteeringBehavior<dtSkirtBehaviorParams>
{
public:
	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	dtSkirtBehavior(unsigned nbMaxAgents);
	virtual ~dtSkirtBehavior();

	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	///
	/// @return		A pointer on a newly allocated behavior
	static dtSkirtBehavior* allocate(unsigned nbMaxAgents);

	/// Frees the given behavior
	///
	/// @param[in]	ptr	A pointer to the behavior we want to free
	static void free(dtSkirtBehavior* ptr);

	float distance;             ///< The distance in which other entities are considered

	// should not be used
	virtual void computeForce(const dtCrowdQuery& query, const dtCrowdAgent& ag, float* force, const dtSkirtBehaviorParams& currentParams, dtSkirtBehaviorParams& newParams);

	void computeVelocity(const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent);

public:
	virtual void doUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent,
						  const dtSkirtBehaviorParams& currentParams, dtSkirtBehaviorParams& newParams, float dt);

private:
	bool updateObtacles(const dtCrowdAgent& ag, const dtCrowdQuery& query, const float* targetPos);
	/// Adds a agent obstacle to the obstacles list.
	///
	/// @param[in]		agent	The current agent.
	/// @param[in]		obtacle	The agent to avoid.
	bool addAgentObstacle(const dtCrowdAgent& agent, const dtCrowdAgent& obtacle, const float* targetPos);

	/// Adds a segment to the obstacles list.
	///
	/// @param[in]	p	The start position of the segment.
	/// @param[in]	q	The end position of the segment.
	bool addSegment(const float* p, const float* q);

private:
	dtAgentObstacle m_agentObstacle;    ///< The closest agent obstacle.
	float m_agentObstacleDistance;
	dtSegmentObstacle m_segmentObstacle;
	float m_segmentObstacleDistance;
	float currentDt;
};

#endif
