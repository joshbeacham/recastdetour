{
    "scene":
    {
        "file": "Meshes/mazyhall.obj"
    },
    "agents":
    [
        {
            "position": [9.5, 0, 0],
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
                            "destination": [-9.5, 0, 0],
                            "visibilityPathOptimizationRange": 6,
                            "initialPathfindIterCount": 0,
                            "anticipateTurns": true,
                            "localPathReplanningInterval": 1
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
        },
        {
            "position": [-9.5, 0, 0],
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
                            "destination": [9.5, 0, 0],
                            "visibilityPathOptimizationRange": 6,
                            "initialPathfindIterCount": 0,
                            "anticipateTurns": true,
                            "localPathReplanningInterval": 1
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
        },
        {
            "position": [9.5, 0, 2.5],
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
                            "destination": [-9.5, 0, -2.5],
                            "visibilityPathOptimizationRange": 6,
                            "initialPathfindIterCount": 0,
                            "anticipateTurns": true,
                            "localPathReplanningInterval": 1
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
        },
        {
            "position": [-9.5, 0, 2.5],
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
                            "destination": [9.5, 0, -2.5],
                            "visibilityPathOptimizationRange": 6,
                            "initialPathfindIterCount": 0,
                            "anticipateTurns": true,
                            "localPathReplanningInterval": 1
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
        },
        {
            "position": [9.5, 0, -2.5],
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
                            "destination": [-9.5, 0, 2.5],
                            "visibilityPathOptimizationRange": 6,
                            "initialPathfindIterCount": 0,
                            "anticipateTurns": true,
                            "localPathReplanningInterval": 1
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
        },
        {
            "position": [-9.5, 0, -2.5],
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
                            "destination": [9.5, 0, 2.5],
                            "visibilityPathOptimizationRange": 6,
                            "initialPathfindIterCount": 0,
                            "anticipateTurns": true,
                            "localPathReplanningInterval": 1
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
        },
        {
            "position": [9.5, 0, 5],
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
                            "destination": [-9.5, 0, -5],
                            "visibilityPathOptimizationRange": 6,
                            "initialPathfindIterCount": 0,
                            "anticipateTurns": true,
                            "localPathReplanningInterval": 1
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
        },
        {
            "position": [-9.5, 0, 5],
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
                            "destination": [9.5, 0, -5],
                            "visibilityPathOptimizationRange": 6,
                            "initialPathfindIterCount": 0,
                            "anticipateTurns": true,
                            "localPathReplanningInterval": 1
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
        },
        {
            "position": [9.5, 0, -5],
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
                            "destination": [-9.5, 0, 5],
                            "visibilityPathOptimizationRange": 6,
                            "initialPathfindIterCount": 0,
                            "anticipateTurns": true,
                            "localPathReplanningInterval": 1
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
        },
        {
            "position": [-9.5, 0, -5],
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
                            "destination": [9.5, 0, 5],
                            "visibilityPathOptimizationRange": 6,
                            "initialPathfindIterCount": 0,
                            "anticipateTurns": true,
                            "localPathReplanningInterval": 1
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