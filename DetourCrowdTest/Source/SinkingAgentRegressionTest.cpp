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

#include "DetourCrowdTestUtils.h"

#include <DetourCollisionAvoidance.h>
#include <DetourCrowd.h>
#include <DetourPathFollowing.h>
#include <DetourPipelineBehavior.h>

SCENARIO("Regression/SinkingAgent", "[regression]")
{
	GIVEN("The computed navmesh for the map 'generation bug #1'")
	{
		dtNavMesh navmesh;
		CHECK(loadTiledNavMesh(&navmesh, "../Data/sinking_agent.dtNavmesh"));

		WHEN("An agent is created at (13912.80, 6.25, -12548.90) with a path following target at (13888.73, 12.60, -12586.23)")
		{
			dtPipelineBehavior* pipeline = dtPipelineBehavior::allocate();
			dtPathFollowing* pathFollowing = dtPathFollowing::allocate(5);
			dtCollisionAvoidance* collisionAvoidance = dtCollisionAvoidance::allocate(5);
			dtBehavior* behaviors[] = {pathFollowing, collisionAvoidance};
			pipeline->setBehaviors(behaviors, 2);
			
			const float from[] = {13912.80, 6.25, -12548.90};
			const float to[] = {13888.73, 12.60, -12586.23};
			
			dtCrowd crowd;
			crowd.init(5, 1.f, &navmesh);
			
			dtCrowdAgent ag;
			CHECK(crowd.addAgent(ag, from));
			CHECK(crowd.pushAgentBehavior(ag.id, pipeline));
			
			pathFollowing->init(*crowd.getCrowdQuery());
			pathFollowing->getBehaviorParams(ag.id)->submitTarget(to);
			
			THEN("It goes smoothly to its target")
			{
				const float dt = 0.1f;
				float t = 0.f;
				unsigned i = 0;
				
				while (t < 16.f)
				{
					crowd.update(dt);
					t += dt;
					CAPTURE(t);
					
					++i;
					CAPTURE(i);
					
					float previousPosition[3];
					dtVcopy(previousPosition, ag.position);
					CAPTURE(previousPosition[0]);
					CAPTURE(previousPosition[1]);
					CAPTURE(previousPosition[2]);
					
					crowd.fetchAgent(ag, ag.id);
					CAPTURE(ag.position[0]);
					CAPTURE(ag.position[1]);
					CAPTURE(ag.position[2]);
					
					CAPTURE(ag.velocity[0]);
					CAPTURE(ag.velocity[1]);
					CAPTURE(ag.velocity[2]);
										
					CHECK(dtVdist(previousPosition, ag.position) < (ag.maxSpeed + 1.f) * dt);
				}
			}
			
			dtPipelineBehavior::free(pipeline);
			dtPathFollowing::free(pathFollowing);
			dtCollisionAvoidance::free(collisionAvoidance);
		}
		
		WHEN("An agent is created at (13896.3, 11.2105, -12574.5) with a velocity of (-1.08234, 0, -1.68183)")
		{
			const float from[] = {13896.3, 11.2105, -12574.5};
			const float velocity[] = {-1.08234, 0, -1.68183};
			
			dtCrowd crowd;
			crowd.init(5, 1.f, &navmesh);
			
			dtCrowdAgent ag;
			CHECK(crowd.addAgent(ag, from));
			
			dtVcopy(ag.velocity, velocity);
			crowd.pushAgent(ag);
			
			THEN("The position is well updated")
			{
				const float dt = 0.1f;
				
				crowd.updatePosition(dt);
				
				float previousPosition[3];
				dtVcopy(previousPosition, ag.position);
				CAPTURE(previousPosition[0]);
				CAPTURE(previousPosition[1]);
				CAPTURE(previousPosition[2]);
				
				float expectedPosition[3];
				dtVmad(expectedPosition, previousPosition, velocity, dt);
				CAPTURE(expectedPosition[0]);
				CAPTURE(expectedPosition[1]);
				CAPTURE(expectedPosition[2]);
				
				crowd.fetchAgent(ag, ag.id);
				CAPTURE(ag.position[0]);
				CAPTURE(ag.position[1]);
				CAPTURE(ag.position[2]);
				
				CHECK(dtVdist2D(expectedPosition, ag.position) < 0.001f);
				CHECK(fabs(expectedPosition[1] - ag.position[1]) < 0.01f);
			}
		}
	}
}
