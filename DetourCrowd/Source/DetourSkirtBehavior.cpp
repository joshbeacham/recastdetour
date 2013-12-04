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

#include "DetourSkirtBehavior.h"

#include "DetourCrowd.h"
#include "DetourCommon.h"

#include <new>
#include <limits>

bool dtSkirtBehaviorParams::init(const float* position)
{
	dtVcopy(targetPos, position);
	return true;
}

dtSkirtBehavior::dtSkirtBehavior(unsigned nbMaxAgents)
	: dtSteeringBehavior<dtSkirtBehaviorParams>(nbMaxAgents)
	, distance(1.0f)
	, maximumForce(2.0f)
	//, m_obstacles(0)
	//, m_obstaclesCount(0)
	, m_agentObstacleDistance(std::numeric_limits<float>::max())
	, m_segmentObstacleDistance(std::numeric_limits<float>::max())
	, currentDt(0)
{
}

dtSkirtBehavior::~dtSkirtBehavior()
{
}

dtSkirtBehavior* dtSkirtBehavior::allocate(unsigned nbMaxAgents)
{
	void* mem = dtAlloc(sizeof(dtSkirtBehavior), DT_ALLOC_PERM);

	if (mem)
		return new(mem) dtSkirtBehavior(nbMaxAgents);

	return 0;
}

void dtSkirtBehavior::free(dtSkirtBehavior* ptr)
{
	if (!ptr)
		return;

	ptr->~dtSkirtBehavior();
	dtFree(ptr);
}

void dtSkirtBehavior::computeForce(const dtCrowdQuery&, const dtCrowdAgent& ag, float* force,
								   const dtSkirtBehaviorParams&, dtSkirtBehaviorParams&)
{
	float currentToAgent[2];
	currentToAgent[0] = m_agentObstacle.position[0];
	currentToAgent[1] = m_agentObstacle.position[2];
	currentToAgent[0] -= ag.position[0];
	currentToAgent[1] -= ag.position[2];
	float relativeSide = 0;
	// check if there is an obstacle segment close to the agent obstacle that is close enough to make moves difficult
	if (m_segmentObstacleDistance < ag.radius * 2 + m_agentObstacle.radius)
	{
		// choose to go at the opposite of the segment obstacle
		float agentToSegment[2];
		agentToSegment[0] = m_segmentObstacle.closest[0];
		agentToSegment[1] = m_segmentObstacle.closest[1];
		agentToSegment[0] -= m_agentObstacle.position[0];
		agentToSegment[1] -= m_agentObstacle.position[2];
		// if positive, the angle from currentToAgent to agentToSegment is positive, agentToSegment is on the left side otherwise agentToSegment is on the right side.
		relativeSide = currentToAgent[1]*agentToSegment[0] - currentToAgent[0]*agentToSegment[1];
	}
	else
	{
		// use the shorter path
		// if positive, the angle from ag.desiredVelocity to currentToAgent is positive, currentToAgent is on the left side otherwise currentToAgent is on the right side.
		relativeSide = ag.desiredVelocity[2]*currentToAgent[0] - ag.desiredVelocity[0]*currentToAgent[1];
	}
	if (relativeSide < 0)
	{
		// perpendicular vector pointing to the right
		force[0] = currentToAgent[1];
		force[2] = -currentToAgent[0];
	}
	else
	{
		// perpendicular vector pointing to the left
		force[0] = -currentToAgent[1];
		force[2] = currentToAgent[0];
	}

	// force direction has been computed, now compute its strength based on the normalized distance from agent to obstacle
	float desiredForce = (this->distance - m_agentObstacleDistance) / this->distance * this->maximumForce;
	float currentForce = dtSqrt(dtSqr(force[0]) + dtSqr(force[2]));
	// this can happen when the agent obstacle is precisely on the segment, in such case normal avoidance will be enough
	if (currentForce > EPSILON)
	{
		force[0] *= desiredForce/currentForce;
		force[2] *= desiredForce/currentForce;
	}
	float previousBehaviorForce[3];
	dtVsub(previousBehaviorForce, ag.desiredVelocity, ag.velocity);
	dtVscale(previousBehaviorForce, previousBehaviorForce, 1.f/currentDt);
	dtVadd(force, force, previousBehaviorForce);
}

void dtSkirtBehavior::doUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent,
                               const dtSkirtBehaviorParams& currentParams, dtSkirtBehaviorParams& newParams, float dt)
{
	// only apply the behavior if the agent is trying to move
	if (dt > EPSILON && dtVlen(oldAgent.desiredVelocity) > EPSILON)
	{
		currentDt = dt;
		// check that we have obstacles
		if (updateObtacles(oldAgent, query, currentParams.targetPos))
		{
			// only apply behavior if there is a reason to
			dtSteeringBehavior<dtSkirtBehaviorParams>::doUpdate(query, oldAgent, newAgent, currentParams, newParams, dt);
		}
	}
}

bool dtSkirtBehavior::updateObtacles(const dtCrowdAgent& ag, const dtCrowdQuery& query, const float* targetPos)
{
	m_agentObstacleDistance = m_segmentObstacleDistance = std::numeric_limits<float>::max();
	const dtCrowdAgentEnvironment* agEnv = query.getAgentEnvironment(ag.id);
	bool hasObstacles = false;
	// Add neighbours as obstacles.
	for (unsigned j = 0; j < agEnv->nbNeighbors; ++j)
	{
		const dtCrowdAgent& neighbor = *query.getAgent(agEnv->neighbors[j].idx);
		hasObstacles = addAgentObstacle(ag, neighbor, targetPos) || hasObstacles;
	}

	if (m_agentObstacleDistance != std::numeric_limits<float>::max())
	{
		// Append neighbour segments as obstacles.
		for (int j = 0; j < agEnv->boundary.getSegmentCount(); ++j)
		{
			const float* s = agEnv->boundary.getSegment(j);

			// check that the segment is from an outer bound
			// allows to cross this segment if the current position is inside the obstacle
			if (dtTriArea2D(ag.position, s, s+3) < 0.f)
				continue;
			// do not consider segments that the agent obstacle is within
			if (dtTriArea2D(m_agentObstacle.position, s, s+3) < 0.f)
				continue;

			hasObstacles = addSegment(s, s+3) || hasObstacles;
		}
	}
	return hasObstacles;
}

bool dtSkirtBehavior::addAgentObstacle(const dtCrowdAgent& agent, const dtCrowdAgent& obtacle, const float* targetPos)
{
	float diff[3];
	dtVsub(diff, obtacle.position, agent.position);
	float dist = dtVlen(diff) - agent.radius - obtacle.radius;

	if (dist < agent.perceptionDistance && dist <= this->distance && dist <= m_agentObstacleDistance)
	{
		float diffToTarget[3];
		dtVsub(diffToTarget, targetPos, agent.position);
		// do not avoid obstacles that are after the target pos
		if (dist < dtVlen(diffToTarget) - agent.radius)
		{
			// check if the obtacle can be considered still compared to current agent
			float obstacleVelocity = dtVlenSqr(obtacle.desiredVelocity);
			if (obstacleVelocity < EPSILON || dtVlenSqr(agent.desiredVelocity) >= obstacleVelocity * 100) // |agent.dvelocity| >= |obstacle.dvelocity| * 10
			{
				bool isInTheWay = true;
				float normalizedDiff[3];
				dtVcopy(normalizedDiff, diff);
				normalizedDiff[1] = 0;
				float normalizedDiffNorm = dtVlen(normalizedDiff);
				if (normalizedDiffNorm > EPSILON)
				{
					dtVscale(normalizedDiff, normalizedDiff, 1.f/dtVlen(normalizedDiff));
					float normalizedVelocity[3];
					dtVscale(normalizedVelocity, agent.desiredVelocity, 1.f/dtVlen(agent.desiredVelocity));
					float cosAngle = dtVdot2D(normalizedDiff, normalizedVelocity);
					isInTheWay = acos(cosAngle) <= 90.f/360.f*3.14159265f;
				}
				if (isInTheWay)
				{
					m_agentObstacleDistance = dist;
					dtVcopy(m_agentObstacle.position, obtacle.position);
					dtVcopy(m_agentObstacle.direction, normalizedDiff);
					m_agentObstacle.radius = obtacle.radius;
					return true;
				}
			}
		}
	}
	return false;
}

bool dtSkirtBehavior::addSegment(const float* p, const float* q)
{
	float projectionFator;
	float distanceToSegment = dtDistancePtSegSqr2D(m_agentObstacle.position, p, q, projectionFator);
	if (distanceToSegment < m_segmentObstacleDistance)
	{
		m_segmentObstacleDistance = distanceToSegment;
		m_segmentObstacle.closest[0] = p[0] + (q[0] - p[0]) * projectionFator;
		m_segmentObstacle.closest[1] = p[2] + (q[2] - p[2]) * projectionFator;
		return true;
	}
	return false;
}
