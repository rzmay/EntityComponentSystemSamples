using System;
using Boids;
using Unity.Entities;
using UnityEngine;


namespace Boids
{
    public class BoidObstacleAuthoring : MonoBehaviour
    {
        public BoidGroups Group = BoidGroups.All;
        public float Weight = 1f;
        public float AvoidanceDistance = 0f;

        class Baker : Baker<BoidObstacleAuthoring>
        {
            public override void Bake(BoidObstacleAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Renderable);
                AddComponent(entity, new BoidObstacle
                {
                    Group = (int)authoring.Group,
                    AvoidanceDistance = authoring.AvoidanceDistance,
                    Weight = authoring.Weight,
                });
            }
        }
    }

    [Serializable]
    public struct BoidObstacle : IComponentData
    {
        public int Group;
        public float Weight;
        public float AvoidanceDistance;
    }
}
