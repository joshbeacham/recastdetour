{
    "scene":
    {
        "file": "Meshes/corridor_120.obj"
    },
    "agents":
    [
        {
            "position": [-4, 0, 0],
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
                            "destination": [4, 0, 0],
                            "visibilityPathOptimizationRange": 6
                        }
                    },
                    {                       
                        "behavior":
                        {
                            "type": "collisionAvoidance",
                            "weightDesiredVelocity": 2.0,
                            "weightCurrentVelocity": 0.75,
                            "weightCurrentAvoidanceSide": 0.75,
                            "weightTimeToCollision": 2.5
                        }
                    }
                ]
            }
        },
        {
            "position": [4, 0, 0],
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
                            "destination": [-4, 0, 0],
                            "visibilityPathOptimizationRange": 6
                        }
                    },
                    {                       
                        "behavior":
                        {
                            "type": "collisionAvoidance",
                            "weightDesiredVelocity": 2.0,
                            "weightCurrentVelocity": 0.75,
                            "weightCurrentAvoidanceSide": 0.75,
                            "weightTimeToCollision": 2.5
                        }
                    }
                ]
            }
        }
    ]
}