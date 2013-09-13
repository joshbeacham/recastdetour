{
    "scene":
    {
        "file": "Meshes/square_20.obj"
    },
    "agents":
    [
        {
            "position": [-10, 0, -10],
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
                            "destination": [10, 0, 10],
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
            "position": [10, 0, 10],
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
                            "destination": [-10, 0, -10],
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
            "position": [-10, 0, 10],
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
                            "destination": [10, 0, -10],
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
            "position": [10, 0, -10],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4,
                "pipeline":
                [
                    {
                        "behavior":
                        {
                            "type": "pathFollowing",
                            "destination": [-10, 0, 10],
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