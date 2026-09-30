using Unity.Entities;
using UnityEngine;

namespace Boids
{
    public class BoidTargetAuthoring : MonoBehaviour
    {
        public BoidGroups Group = BoidGroups.All;
        public float Weight = 1f;
        public float TargetDistance = 0f;
        class Baker : Baker<BoidTargetAuthoring>
        {
            public override void Bake(BoidTargetAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Renderable);
                AddComponent(entity, new BoidTarget
                {
                    Group = (int)authoring.Group,
                    Weight = authoring.Weight,
                    TargetDistance = authoring.TargetDistance,
                });
            }
        }
    }

    public struct BoidTarget : IComponentData
    {
        public int Group;
        public float Weight;
        public float TargetDistance;

    }
}
