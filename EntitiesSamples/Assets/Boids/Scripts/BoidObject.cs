using UnityEngine;

namespace Boids
{
  public enum BoidObjectType : byte
  {
    Obstacle,
    Target
  }

  public class BoidObject : MonoBehaviour
  {
    public BoidObjectType Type = BoidObjectType.Obstacle;

    public BoidGroups Group = BoidGroups.All;
    public float Weight = 1f;
    public float TargetDistance = 0f;
  }
}
