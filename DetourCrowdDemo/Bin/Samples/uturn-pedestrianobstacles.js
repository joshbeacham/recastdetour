{
    "scene":
    {
        "file": "Meshes/uturn.obj"
    },
    "agents":
    [
        {
            "position": [6.5, 0, 9],
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
                            "destination": [-6.5, 0, 9],
                            "visibilityPathOptimizationRange": 6
                        }
                    },
                    {
                        "behavior":
                        {
                            "type": "collisionAvoidance",
                            "weightDesiredVelocity": 2.0,
                            "weightCurrentVelocity": 0.75,
                            "weightCurrentAvoidanceSide": 0,
                            "weightTimeToCollision": 2.5,
                            "sampleOriginScale": 0.4
                        }
                    }
                ]
            }
        },
        {
            "position": [5.3, 0, 0],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pipeline":
                [
                ]
            }
        },
        {
            "position": [4, 0, -3.5],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pipeline":
                [
                ]
            }
        },
        {
            "position": [0, 0, -5.3],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pipeline":
                [
                ]
            }
        },
        {
            "position": [0.2, 0, -6.3],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pipeline":
                [
                ]
            }
        }
        ,
        {
            "position": [-5, 0, -5],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pipeline":
                [
                ]
            }
        }
    ]
}