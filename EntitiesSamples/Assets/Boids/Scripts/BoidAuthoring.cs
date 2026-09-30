using System;
using Unity.Entities;
using Unity.Transforms;
using Unity.VisualScripting.YamlDotNet.Core.Tokens;
using Unity.Mathematics;
using UnityEngine;

namespace Boids
{
    [System.Flags]
    public enum BoidGroups
    {
        None = 0,
        Group1 = 1 << 0,
        Group2 = 1 << 1,
        Group3 = 1 << 2,
        Group4 = 1 << 3,
        All = ~0
    }

    public class BoidAuthoring : MonoBehaviour
    {
        public float CellRadius = 8.0f;
        public float SeparationWeight = 1.0f;
        public float AlignmentWeight = 1.0f;
        public float CohesionWeight = 1.0f;
        public float TargetWeight = 2.0f;
        public float ObstacleWeight = 2.0f;
        public float ObstacleAversionDistance = 30.0f;
        public BoidGroups Group = BoidGroups.Group1;
        public float MoveSpeed = 25.0f;

        class Baker : Baker<BoidAuthoring>
        {
            public override void Bake(BoidAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Renderable | TransformUsageFlags.WorldSpace);

                AddSharedComponent(entity, new Boid
                {
                    CellRadius = authoring.CellRadius,
                    SeparationWeight = authoring.SeparationWeight,
                    AlignmentWeight = authoring.AlignmentWeight,
                    CohesionWeight = authoring.CohesionWeight,
                    TargetWeight = authoring.TargetWeight,
                    ObstacleWeight = authoring.ObstacleWeight,
                    ObstacleAversionDistance = authoring.ObstacleAversionDistance,
                    Group = (int)authoring.Group,
                    MoveSpeed = authoring.MoveSpeed,
                    Scale = new float3(authoring.transform.localScale),
                });
            }
        }
    }

    [Serializable]
    public struct Boid : ISharedComponentData
    {
        public float CellRadius;
        public float SeparationWeight;
        public float AlignmentWeight;
        public float CohesionWeight;
        public float TargetWeight;
        public float ObstacleWeight;
        public float ObstacleAversionDistance;
        public int Group;
        public float MoveSpeed;
        public float3 Scale;
    }
}
