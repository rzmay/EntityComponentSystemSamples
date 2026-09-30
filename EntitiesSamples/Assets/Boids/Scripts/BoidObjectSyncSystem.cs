using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Boids
{
  [UpdateInGroup(typeof(InitializationSystemGroup))]
  public partial class BoidObjectSyncSystem : SystemBase
  {
    private EndInitializationEntityCommandBufferSystem _ecbSystem;
    private Dictionary<GameObject, Entity> _syncedEntities;

    protected override void OnCreate()
    {
      _syncedEntities = new Dictionary<GameObject, Entity>();
    }

    protected override void OnStartRunning()
    {
      _ecbSystem = World.GetOrCreateSystemManaged<EndInitializationEntityCommandBufferSystem>();
    }

    protected override void OnUpdate()
    {
      var ecb = _ecbSystem.CreateCommandBuffer();
      var allBoidObjects = Object.FindObjectsByType<BoidObject>(FindObjectsSortMode.None);

      // Track which GameObjects are still present this frame
      HashSet<GameObject> seen = new HashSet<GameObject>();

      foreach (var boidObject in allBoidObjects)
      {
        var go = boidObject.gameObject;
        RegisterOrUpdate(go, boidObject.Type, ref ecb, seen);

        // Check all child colliders as sub-objects
        var colliders = go.GetComponentsInChildren<Collider>(includeInactive: false);
        foreach (var collider in colliders)
        {
          if (!collider.enabled || collider.isTrigger)
            continue;

          RegisterOrUpdate(collider.gameObject, boidObject.Type, ref ecb, seen, collider);
        }
      }

      // Destroy any entities whose GameObjects no longer exist or were removed
      var toRemove = new List<GameObject>();
      foreach (var kvp in _syncedEntities)
      {
        if (!seen.Contains(kvp.Key))
        {
          if (EntityManager.Exists(kvp.Value))
            ecb.DestroyEntity(kvp.Value);

          toRemove.Add(kvp.Key);
        }
      }

      foreach (var go in toRemove)
        _syncedEntities.Remove(go);
    }

    private void RegisterOrUpdate(
        GameObject go,
        BoidObjectType type,
        ref EntityCommandBuffer ecb,
        HashSet<GameObject> seen,
        Collider optionalCollider = null)
    {
      seen.Add(go);

      bool exists = _syncedEntities.TryGetValue(go, out Entity entity);
      if (exists && !EntityManager.Exists(entity))
      {
        _syncedEntities.Remove(go);
        exists = false;
        entity = Entity.Null;
      }

      if (!exists)
      {
        entity = ecb.CreateEntity();
        _syncedEntities[go] = entity;

        switch (type)
        {
          case BoidObjectType.Obstacle:
            ecb.AddComponent<BoidObstacle>(entity);
            break;
          case BoidObjectType.Target:
            ecb.AddComponent<BoidTarget>(entity);
            break;
        }
      }

      // Always update transform
      var transform = go.transform;
      var localTransform = new LocalTransform
      {
        Position = transform.position,
        Rotation = transform.rotation,
        Scale = math.cmax(transform.lossyScale) // conservative scaling
      };

      if (!EntityManager.HasComponent<LocalTransform>(entity))
        ecb.AddComponent(entity, localTransform);
      else
        ecb.SetComponent(entity, localTransform);

      // Conditionally bake collider
      if (optionalCollider != null)
      {
        var colliderData = BoidColliderUtility.GetBoidColliderData(optionalCollider);

        if (!EntityManager.HasComponent<BoidCollider>(entity))
          ecb.AddComponent(entity, colliderData);
        else
          ecb.SetComponent(entity, colliderData);
      }
    }
  }
}
