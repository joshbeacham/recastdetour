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

#include "DetourPathFollowing.h"
#include "DetourCollisionAvoidance.h"

#ifdef _MSC_VER
#pragma warning(push, 0)
#include <catch.hpp>
#pragma warning(pop)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#include <catch.hpp>
#pragma GCC diagnostic pop
#endif

#include <cstring>

SCENARIO("DetourPathFollowingTest/PathFollowingOnly", "[detourPathFollowing]")
{
    const float posAgt1[] = {0, 0, 0};
    const float posAgt2[] = {0, 0, 1};
    
	TestScene ts;
	dtCrowd* crowd = ts.createSquareScene(20, 0.5f);
	REQUIRE(crowd != 0);
    
    dtCrowdAgent ag1, ag2;
    REQUIRE(crowd->addAgent(ag1, posAgt1));
    REQUIRE(crowd->addAgent(ag2, posAgt2));
    
    ag1.maxSpeed = 2.f;
    REQUIRE(crowd->applyAgent(ag1));
    
    ag2.maxSpeed = 3.f;
    REQUIRE(crowd->applyAgent(ag2));
    
    dtPathFollowing* pf1 = dtPathFollowing::allocate(2);
    dtPathFollowingParams* pfParams = pf1->getBehaviorParams(crowd->getAgent(ag1.id)->id);
    dtPathFollowingParams* pfParams2 = pf1->getBehaviorParams(crowd->getAgent(ag2.id)->id);
    
    crowd->setAgentBehavior(ag1.id, pf1);
    crowd->setAgentBehavior(ag2.id, pf1);
    
    pf1->init(*crowd->getCrowdQuery());
    
    GIVEN("Two valid destinations")
	{
        const float destAgt1[] = {-18, 0, 0};
        const float destAgt2[] = {18, 0, 1};
        
		// Set the destinations
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt1, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&pfParams->targetRef, 0);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt2, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&pfParams2->targetRef, 0);
		REQUIRE(pf1->requestMoveTarget(ag1.id, pfParams->targetRef, destAgt1));
		REQUIRE(pf1->requestMoveTarget(ag2.id, pfParams2->targetRef, destAgt2));
        
        WHEN("Updated for 3s at 10 Hz")
        {
            for (int i = 0; i < 30; ++i)
                crowd->update(0.1f);
            
            THEN("Agents have moved at (roughly) their maximum speed")
            {
                CHECK(fabs(dtVdist2D(crowd->getAgent(ag1.id)->position, posAgt1) - ag1.maxSpeed * 3.f) < 0.05f * ag1.maxSpeed * 3.f);
                CHECK(fabs(dtVdist2D(crowd->getAgent(ag2.id)->position, posAgt1) - ag2.maxSpeed * 3.f) < 0.05f * ag2.maxSpeed * 3.f);
            }
            
            THEN("Agents have moved in straight line toward their destination")
            {
                CHECK(fabs(crowd->getAgent(ag1.id)->position[0]-destAgt1[0]) < fabs(posAgt1[0]-destAgt1[0]));
                CHECK(fabs(crowd->getAgent(ag1.id)->position[2]-destAgt1[2]) < 0.1f);
                
                CHECK(fabs(crowd->getAgent(ag2.id)->position[0]-destAgt2[0]) < fabs(posAgt2[0]-destAgt2[0]));
                CHECK(fabs(crowd->getAgent(ag2.id)->position[2]-destAgt2[2]) < 0.1f);
            }
        }
        
        WHEN("Updated for 3s at 10000 Hz")
        {
            for (int i = 0; i < 30000; ++i)
                crowd->update(0.0001f);
            
            THEN("Agents have moved at (roughly) their maximum speed")
            {
                CHECK(fabs(dtVdist2D(crowd->getAgent(ag1.id)->position, posAgt1) - ag1.maxSpeed * 3.f) < 0.05f * ag1.maxSpeed * 3.f);
                CHECK(fabs(dtVdist2D(crowd->getAgent(ag2.id)->position, posAgt1) - ag2.maxSpeed * 3.f) < 0.05f * ag2.maxSpeed * 3.f);
            }
        }
	}
    
    dtPathFollowing::free(pf1);
}

SCENARIO("DetourPathFollowingTest/PathFollowingAndCollisionAvoidance", "[detourPathFollowing][debug]")
{
    const float a1Position[] = {0, 0, 0};
    
	TestScene ts;
	dtCrowd* crowd = ts.createSquareScene(20, 0.5f);
	REQUIRE(crowd != 0);
    
    dtCrowdAgent a1;
    REQUIRE(crowd->addAgent(a1, a1Position));
    
    a1.maxSpeed = 2.f;
    REQUIRE(crowd->applyAgent(a1));
    
    dtPipelineBehavior* pipeline = dtPipelineBehavior::allocate();
    dtPathFollowing* pathFollowing = dtPathFollowing::allocate(12);
    dtCollisionAvoidance* collisionAvoidance = dtCollisionAvoidance::allocate(12);
    dtBehavior* pipelineBehaviors[] = {pathFollowing, collisionAvoidance};
    pipeline->setBehaviors(pipelineBehaviors, 2);
    
    dtPathFollowingParams* pathFollowingParams = pathFollowing->getBehaviorParams(crowd->getAgent(a1.id)->id);
    
    crowd->setAgentBehavior(a1.id, pipeline);
    
    pathFollowing->init(*crowd->getCrowdQuery());
    collisionAvoidance->init();
    
    GIVEN("A valid destination")
	{
        const float a1Destination[] = {-18, 0, 0};
        
		// Set the destination
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a1Destination, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &pathFollowingParams->targetRef, 0);

		REQUIRE(pathFollowing->requestMoveTarget(a1.id, pathFollowingParams->targetRef, a1Destination));
        
        WHEN("Updated for 3s at 10 Hz")
        {
            for (int i = 0; i < 30; ++i)
                crowd->update(0.1f);
            
            THEN("Agent have moved at (roughly) its maximum speed")
            {
                CHECK(fabs(dtVdist2D(crowd->getAgent(a1.id)->position, a1Position) - a1.maxSpeed * 3.f) < 0.05f * a1.maxSpeed * 3.f);
            }
        }
        
        WHEN("Updated for 3s at 10000 Hz")
        {
            for (int i = 0; i < 30000; ++i)
                crowd->update(0.0001f);
            
            THEN("Agent have moved at (roughly) its maximum speed")
            {
                CHECK(fabs(dtVdist2D(crowd->getAgent(a1.id)->position, a1Position) - a1.maxSpeed * 3.f) < 0.05f * a1.maxSpeed * 3.f);
            }
        }
	}
    
    dtPipelineBehavior::free(pipeline);
    dtPathFollowing::free(pathFollowing);
    dtCollisionAvoidance::free(collisionAvoidance);
}