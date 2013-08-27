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

SCENARIO("DetourPathFollowingTest/TwoAgents", "[detourPathFollowing]")
{
    const float posAgt1[] = {0, 0, 0};
    const float posAgt2[] = {0, 0, 1};
    
	TestScene ts;
	dtCrowd* crowd = ts.createSquareScene(20, 0.5f);
	REQUIRE(crowd != 0);
    
    dtCrowdAgent ag1, ag2;
    REQUIRE(crowd->addAgent(ag1, posAgt1));
    REQUIRE(crowd->addAgent(ag2, posAgt2));
    
    ts.defaultInitializeAgent(*crowd, ag1.id);
    ts.defaultInitializeAgent(*crowd, ag2.id);
    
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
        
        WHEN("Updated for 1s at 100 Hz")
        {
            for (int i = 0; i < 100; ++i)
                crowd->update(1.0);
            
            THEN("Agents have moved in straight line toward there destination")
            {
                CHECK(crowd->getAgent(ag1.id)->position[0] < -10.f);
                CHECK(crowd->getAgent(ag2.id)->position[0] > 10.f);
            }
        }
	}
}