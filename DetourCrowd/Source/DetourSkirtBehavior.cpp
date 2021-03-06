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

#include <cmath>
#include <limits>
#include <new>
#include <utility>

bool dtSkirtBehaviorParams::init(const float* position)
{
	dtVcopy(targetPos, position);
	return true;
}

dtSkirtBehavior::dtSkirtBehavior(unsigned nbMaxAgents)
	: dtSteeringBehavior<dtSkirtBehaviorParams>(nbMaxAgents)
	, distance(1.0f)
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

void dtSkirtBehavior::computeForce(const dtCrowdQuery&, const dtCrowdAgent&, float*, const dtSkirtBehaviorParams&, dtSkirtBehaviorParams&)
{
	// NOTHING
}

namespace
{
	// solve equation ax + by + c = 0
	std::pair<float, float> solveQuadratic(float a, float b, float c)
	{
		// 
		float p = b/a;
		float q = c/a;
		float squareRootPart = dtSqr(p/2) - q;
		if (squareRootPart >= 0)
		{
			float solution1 = -p/2 + dtSqrt(squareRootPart);
			float solution2 = -p/2 - dtSqrt(squareRootPart);
			return std::make_pair(solution1, solution2);
		}
		return std::make_pair(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
	}
	bool computeCircleIntersectionPoints(float center1X, float center1Y, float radius1, float center2X, float center2Y, float radius2, float* intersection1, float* intersection2)
	{
		// consider that the first circle (center1, radius1) is at the origin to compute tangent point intersection then translate result back to the circle's origin
		float newCenter[2];
		newCenter[0] = center2X - center1X;
		newCenter[1] = center2Y - center1Y;
		float r1Square = radius1 * radius1;
		float r2Square = radius2 * radius2;
		if (dtAbs(newCenter[0]) > EPSILON)
		{
			// resolving both circle equations leads to x = ay + b with following a and b values
			float a = -newCenter[1]/newCenter[0];
			float b = (newCenter[0]*newCenter[0] + newCenter[1]*newCenter[1] - r2Square + r1Square) / (2*newCenter[0]);
			// injecting x = ay + b into X^2 + Y^2 = radius1^2 leads to an equation aY^2+bY+c=0
			std::pair<float, float> yResults = solveQuadratic(1+a*a, 2*a*b, b*b-r1Square);
			// inject result back into x = ay + b
			std::pair<float, float> xResults(
				a*yResults.first+b,
				a*yResults.second+b
				);
			// translate back into original frame
			intersection1[0] = xResults.first + center1X;
			intersection1[1] = yResults.first + center1Y;
			intersection2[0] = xResults.second + center1X;
			intersection2[1] = yResults.second + center1Y;
			return true;
		}
		else if (dtAbs(newCenter[1]) > EPSILON)
		{
			// resolving both circle equations leads to y = ax + b with following a and b values
			float a = -newCenter[0]/newCenter[1];
			float b = (newCenter[0]*newCenter[0] + newCenter[1]*newCenter[1] - r2Square + r1Square) / (2*newCenter[1]);
			// injecting y = ax + b into X^2 + Y^2 = radius1^2 leads to a quadratic equation aX^2+bX+c=0
			std::pair<float, float> xResults = solveQuadratic(1+a*a, 2*a*b, b*b-r1Square);
			std::pair<float, float> yResults(
				a*xResults.first+b,
				a*xResults.second+b
				);
			// translate back into original frame
			intersection1[0] = xResults.first + center1X;
			intersection1[1] = yResults.first + center1Y;
			intersection2[0] = xResults.second + center1X;
			intersection2[1] = yResults.second + center1Y;
			return true;
		}
		// it means that both centers are at the same coordinates, either there is no intersection or they intersect perfectly
		return false;
	}
	float computeDistanceBetweenPointsSquare(const float* point1, const float* point2)
	{
		return dtSqr(point2[0]-point1[0]) + dtSqr(point2[1]-point1[1]);
	}
}
void dtSkirtBehavior::computeVelocity(const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent)
{
	float currentToAgent[2];
	currentToAgent[0] = m_agentObstacle.position[0];
	currentToAgent[1] = m_agentObstacle.position[2];
	currentToAgent[0] -= oldAgent.position[0];
	currentToAgent[1] -= oldAgent.position[2];
	float distanceToAgent = dtSqrt(currentToAgent[0]*currentToAgent[0] + currentToAgent[1]*currentToAgent[1]);
	float sinToObstacleBorder = m_agentObstacle.radius / distanceToAgent;
	float cosToObstacleBorder = dtSqrt(1 - sinToObstacleBorder*sinToObstacleBorder);
	float distanceToTangent = cosToObstacleBorder * distanceToAgent;
	// we now have the data for two circles, finding their intersection will give the tangent points from oldAgent.position to the m_agentObstacle circle
	float tangent1[2], tangent2[2];
	if (computeCircleIntersectionPoints(
		m_agentObstacle.position[0], m_agentObstacle.position[2], m_agentObstacle.radius,
		oldAgent.position[0], oldAgent.position[2], distanceToTangent,
		tangent1, tangent2))
	{
		float tangentToUse[2];
		// check if there is an obstacle segment close to the agent obstacle that is close enough to make moves difficult
		if (m_segmentObstacleDistance != std::numeric_limits<float>::max() && m_segmentObstacleDistance < oldAgent.radius * 2 + m_agentObstacle.radius)
		{
			float distanceFromTangent1 = computeDistanceBetweenPointsSquare(tangent1, m_segmentObstacle.closest);
			float distanceFromTangent2 = computeDistanceBetweenPointsSquare(tangent2, m_segmentObstacle.closest);
			float segmentToAgent[2];
			segmentToAgent[0] = m_segmentObstacle.closest[0];
			segmentToAgent[1] = m_segmentObstacle.closest[1];
			segmentToAgent[0] -= m_agentObstacle.position[0];
			segmentToAgent[1] -= m_agentObstacle.position[2];
			float currentToSegment[2];
			currentToSegment[0] = m_segmentObstacle.closest[0];
			currentToSegment[1] = m_segmentObstacle.closest[1];
			currentToSegment[0] -= oldAgent.position[0];
			currentToSegment[1] -= oldAgent.position[2];
			// if positive, the angle from segmentToAgent to currentToSegment is positive, currentToSegment is on the left side otherwise currentToSegment is on the right side.
			float currentPositionRelativeSide = segmentToAgent[1]*currentToSegment[0] - segmentToAgent[0]*currentToSegment[1];
			// if positive, the angle from segmentToAgent to segmentToCurrent is positive, segmentToCurrent is on the left side otherwise segmentToCurrent is on the right side.
			float currentVelocityRelativeSide = segmentToAgent[1]*oldAgent.desiredVelocity[0] - segmentToAgent[0]*oldAgent.desiredVelocity[2];
			// check if the agent is tyring to pass by the agent obstacle or is going away from it
			if ((currentPositionRelativeSide < 0 && currentVelocityRelativeSide < 0) || (currentPositionRelativeSide > 0 && currentVelocityRelativeSide > 0))
			{
				// trying to pass by the agent obstacle: select tangent that is further from obstacle
				if (distanceFromTangent1 > distanceFromTangent2)
					dtVcopy(tangentToUse, tangent1);
				else
					dtVcopy(tangentToUse, tangent2);
			}
			else
			{
				// tyring to go away from the agent obtacle: select tangent that is closer to obstacle
				if (distanceFromTangent1 > distanceFromTangent2)
					dtVcopy(tangentToUse, tangent2);
				else
					dtVcopy(tangentToUse, tangent1);
			}
		}
		else
		{
			// use the path with less deviation
			float currentToTangent[2];
			currentToTangent[0] = tangent1[0];
			currentToTangent[1] = tangent1[1];
			currentToTangent[0] -= m_agentObstacle.position[0];
			currentToTangent[1] -= m_agentObstacle.position[2];
			float deviationTangent1FromVelocity = currentToTangent[1]*oldAgent.desiredVelocity[0] - currentToTangent[0]*oldAgent.desiredVelocity[2];
			currentToTangent[0] = tangent2[0];
			currentToTangent[1] = tangent2[1];
			currentToTangent[0] -= m_agentObstacle.position[0];
			currentToTangent[1] -= m_agentObstacle.position[2];
			float deviationTangent2FromVelocity = currentToTangent[1]*oldAgent.desiredVelocity[0] - currentToTangent[0]*oldAgent.desiredVelocity[2];
			dtVcopy(tangentToUse, dtAbs(deviationTangent1FromVelocity) > dtAbs(deviationTangent2FromVelocity) ? tangent1 : tangent2);
		}
		// compute the direction vector
		float obstacleCenterToTangent[2];
		obstacleCenterToTangent[0] = tangentToUse[0] - m_agentObstacle.position[0];
		obstacleCenterToTangent[1] = tangentToUse[1] - m_agentObstacle.position[2];
		float obstacleCenterToTangentLength = m_agentObstacle.radius;//sqrt(obstacleCenterToTangent[0]*obstacleCenterToTangent[0] + obstacleCenterToTangent[1]*obstacleCenterToTangent[1]);
		// make sure there is room for the agent to move through
		obstacleCenterToTangent[0] *= (m_agentObstacle.radius+oldAgent.radius)/obstacleCenterToTangentLength;
		obstacleCenterToTangent[1] *= (m_agentObstacle.radius+oldAgent.radius)/obstacleCenterToTangentLength;
		float targetPoint[3];
		dtVcopy(targetPoint, m_agentObstacle.position);
		targetPoint[0] += obstacleCenterToTangent[0];
		targetPoint[2] += obstacleCenterToTangent[1];
		float previousVelocityLength = dtVlen(oldAgent.desiredVelocity);
		dtVsub(newAgent.desiredVelocity, targetPoint, oldAgent.position);
		float newVelocityLength = dtVlen(newAgent.desiredVelocity);
		dtVscale(newAgent.desiredVelocity, newAgent.desiredVelocity, previousVelocityLength/newVelocityLength);
		{
			float currentToSegment[2];
			currentToSegment[0] = m_segmentObstacle.closest[0];
			currentToSegment[1] = m_segmentObstacle.closest[1];
			currentToSegment[0] -= oldAgent.position[0];
			currentToSegment[1] -= oldAgent.position[2];
			float approximateAngleBetweenOldVelocityAndOBstacle = currentToSegment[1]*oldAgent.desiredVelocity[0] - currentToSegment[0]*oldAgent.desiredVelocity[2];
			float approximateAngleBetweenNewVelocityAndOBstacle = currentToSegment[1]*newAgent.desiredVelocity[0] - currentToSegment[0]*newAgent.desiredVelocity[2];
			// if applying the new velocity makes the current agent go closer to the obstacle use the old one
			if(dtAbs(approximateAngleBetweenNewVelocityAndOBstacle) < dtAbs(approximateAngleBetweenOldVelocityAndOBstacle))
				dtVcopy(newAgent.desiredVelocity, oldAgent.desiredVelocity);
		}
	}
	else
	{
		// should mean the two agents are one in the other, not much we can do...
		dtVcopy(newAgent.desiredVelocity, oldAgent.desiredVelocity);
	}

}

void dtSkirtBehavior::doUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent,
                               const dtSkirtBehaviorParams& currentParams, dtSkirtBehaviorParams&, float dt)
{
	// only apply the behavior if the agent is trying to move
	if (dt > EPSILON && dtVlen(oldAgent.desiredVelocity) > EPSILON)
	{
		currentDt = dt;
		// check that we have obstacles
		if (updateObtacles(oldAgent, query, currentParams.targetPos))
		{
			computeVelocity(oldAgent, newAgent);
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
		// get the environment based on the agent obstacle to avoid
		agEnv = query.getAgentEnvironment(m_agentObstacle.id);
		// Append neighbour segments as obstacles.
		for (int j = 0; j < agEnv->boundary.getSegmentCount(); ++j)
		{
			const float* s = agEnv->boundary.getSegment(j);

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

	if (dist < agent.detectionRange && dist <= this->distance && dist <= m_agentObstacleDistance)
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
					m_agentObstacle.id = obtacle.id;
					return true;
				}
			}
		}
	}
	return false;
}

bool dtSkirtBehavior::addSegment(const float* p, const float* q)
{
	float projectionFactor;
	float distanceToSegment = dtDistancePtSegSqr2D(m_agentObstacle.position, p, q, projectionFactor);
	if (distanceToSegment < m_segmentObstacleDistance)
	{
		m_segmentObstacleDistance = distanceToSegment;
		m_segmentObstacle.closest[0] = p[0] + (q[0] - p[0]) * projectionFactor;
		m_segmentObstacle.closest[1] = p[2] + (q[2] - p[2]) * projectionFactor;
		return true;
	}
	return false;
}
