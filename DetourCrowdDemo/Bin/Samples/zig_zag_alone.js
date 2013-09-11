{
    "scene":
    {
        "file": "Meshes/zig_zag_200.obj"
    },
    "agents":
    [
        {
            "position": [5, 0, -1],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pipeline":
                [
                    {
                        "behavior":
                        {
                            "type": "pathFollowing",
                            "destination": [1, 0, -25],
                            "visibilityPathOptimizationRange": 6,
                            "initialPathfindIterCount": 0,
                            "anticipateTurns": true,
                            "localPathReplanningInterval": 0.5
                        }
                    },
                    {                       
                        "behavior":
                        {
                            "type": "collisionAvoidance"
                        }
                    }
                ]
            }
        }
    ]
}