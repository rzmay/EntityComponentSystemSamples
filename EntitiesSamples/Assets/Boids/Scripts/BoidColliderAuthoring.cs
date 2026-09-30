using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Collections;


namespace Boids
{
    public enum BoidColliderType : byte
    {
        None = 0,
        Sphere,
        Box,
        Capsule,
        Mesh,
    }

    public struct BoidCollider : IComponentData
    {
        public BoidColliderType Type;

        // Is this a container, as opposed to an obstacle?
        public bool Container;

        public float3 Center;

        // Sphere
        public float Radius;

        // Box
        public float3 HalfExtents;
        public float3x3 Rotation;

        // Capsule
        public float3 PointA;
        public float3 PointB;

        // Mesh
        public BlobAssetReference<MeshColliderBlob> Mesh;
    }

    public struct MeshColliderBlob
    {
        public BlobArray<float3> Vertices;
        public BlobArray<int> Indices;
    }

    public class BoidColliderAuthoring : MonoBehaviour
    {
        public bool Container;
    }

    public class BoidColliderBaker : Baker<BoidColliderAuthoring>
    {
        public override void Bake(BoidColliderAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Renderable | TransformUsageFlags.Dynamic);


            var collider = authoring.GetComponent<Collider>();

            if (!collider.enabled || collider.isTrigger)
                return;

            AddComponent(entity, BoidColliderUtility.GetBoidColliderData(collider, authoring.Container));


            // var type = BoidColliderType.None;
            // var boidCollider = new BoidCollider();

            // var t = collider.transform;
            // var center = collider.bounds.center;

            // boidCollider.Center = center;
            // boidCollider.Container = authoring.Container;

            // switch (collider)
            // {
            //     case SphereCollider sphere:
            //         type = BoidColliderType.Sphere;
            //         boidCollider.Radius = sphere.radius * math.cmax(t.lossyScale);
            //         break;

            //     case BoxCollider box:
            //         type = BoidColliderType.Box;
            //         boidCollider.HalfExtents = Vector3.Scale(box.size * 0.5f, t.lossyScale);
            //         boidCollider.Rotation = new float3x3(t.rotation);
            //         break;

            //     case CapsuleCollider capsule:
            //         type = BoidColliderType.Capsule;
            //         GetCapsulePoints(capsule, t, out boidCollider.PointA, out boidCollider.PointB);
            //         boidCollider.Radius = capsule.radius * math.max(t.lossyScale.x, t.lossyScale.z);
            //         break;

            //     case MeshCollider meshCollider:
            //         if (meshCollider.sharedMesh == null)
            //             break;

            //         type = BoidColliderType.Mesh;
            //         var mesh = meshCollider.sharedMesh;
            //         var verts = mesh.vertices;
            //         var tris = mesh.triangles;

            //         var vertsNative = new NativeArray<float3>(verts.Length, Allocator.Temp);
            //         var trisNative = new NativeArray<int>(tris, Allocator.Temp);

            //         for (int i = 0; i < verts.Length; i++)
            //             vertsNative[i] = t.TransformPoint(verts[i]);

            //         var builder = new BlobBuilder(Allocator.Temp);
            //         ref var meshRoot = ref builder.ConstructRoot<MeshColliderBlob>();
            //         var vertsBlob = builder.Allocate(ref meshRoot.Vertices, verts.Length);
            //         var trisBlob = builder.Allocate(ref meshRoot.Indices, tris.Length);

            //         for (int i = 0; i < vertsNative.Length; i++)
            //             vertsBlob[i] = vertsNative[i];
            //         for (int i = 0; i < trisNative.Length; i++)
            //             trisBlob[i] = trisNative[i];

            //         boidCollider.Mesh = builder.CreateBlobAssetReference<MeshColliderBlob>(Allocator.Persistent);
            //         break;
            // }

            // boidCollider.Type = type;
            // AddComponent(entity, boidCollider);

        }
    }

    public static class BoidColliderUtility
    {
        public static BoidCollider GetBoidColliderData(Collider collider, bool container = false)
        {
            var type = BoidColliderType.None;
            var boidCollider = new BoidCollider();

            var t = collider.transform;
            var center = collider.bounds.center;

            boidCollider.Center = center;
            boidCollider.Container = container;

            switch (collider)
            {
                case SphereCollider sphere:
                    type = BoidColliderType.Sphere;
                    boidCollider.Radius = sphere.radius * math.cmax(t.lossyScale);
                    break;

                case BoxCollider box:
                    type = BoidColliderType.Box;
                    boidCollider.HalfExtents = Vector3.Scale(box.size * 0.5f, t.lossyScale);
                    boidCollider.Rotation = new float3x3(t.rotation);
                    break;

                case CapsuleCollider capsule:
                    type = BoidColliderType.Capsule;
                    GetCapsulePoints(capsule, t, out boidCollider.PointA, out boidCollider.PointB);
                    boidCollider.Radius = capsule.radius * math.max(t.lossyScale.x, t.lossyScale.z);
                    break;

                case MeshCollider meshCollider:
                    if (meshCollider.sharedMesh == null)
                        break;

                    type = BoidColliderType.Mesh;
                    var mesh = meshCollider.sharedMesh;
                    var verts = mesh.vertices;
                    var tris = mesh.triangles;

                    var vertsNative = new NativeArray<float3>(verts.Length, Allocator.Temp);
                    var trisNative = new NativeArray<int>(tris, Allocator.Temp);

                    for (int i = 0; i < verts.Length; i++)
                        vertsNative[i] = t.TransformPoint(verts[i]);

                    var builder = new BlobBuilder(Allocator.Temp);
                    ref var meshRoot = ref builder.ConstructRoot<MeshColliderBlob>();
                    var vertsBlob = builder.Allocate(ref meshRoot.Vertices, verts.Length);
                    var trisBlob = builder.Allocate(ref meshRoot.Indices, tris.Length);

                    for (int i = 0; i < vertsNative.Length; i++)
                        vertsBlob[i] = vertsNative[i];
                    for (int i = 0; i < trisNative.Length; i++)
                        trisBlob[i] = trisNative[i];

                    boidCollider.Mesh = builder.CreateBlobAssetReference<MeshColliderBlob>(Allocator.Persistent);
                    break;
            }

            boidCollider.Type = type;
            return boidCollider;
        }

        public static void GetCapsulePoints(CapsuleCollider capsule, Transform transform, out float3 a, out float3 b)
        {
            var center = capsule.center;
            var height = capsule.height * 0.5f;
            var radius = capsule.radius;

            Vector3 dir = Vector3.up;
            switch (capsule.direction)
            {
                case 0: dir = Vector3.right; break;
                case 1: dir = Vector3.up; break;
                case 2: dir = Vector3.forward; break;
            }

            a = transform.TransformPoint(center + dir * (height - radius));
            b = transform.TransformPoint(center - dir * (height - radius));
        }

        public static float3 SampleNearestPoint(in BoidCollider collider, in float3 position)
        {
            // float4 containing closest point, along with boolean w denoting whether or not the boid is inside the collider
            float4 surfaceResult = collider.Type switch
            {
                BoidColliderType.Sphere => ClosestPointOnSphere(collider.Center, collider.Radius, position),
                BoidColliderType.Box => ClosestPointOnBox(collider.Center, collider.HalfExtents, collider.Rotation, position),
                BoidColliderType.Capsule => ClosestPointOnCapsule(collider.PointA, collider.PointB, collider.Radius, position),
                BoidColliderType.Mesh when collider.Mesh.IsCreated => ClosestPointOnMesh(position, ref collider.Mesh.Value.Vertices, ref collider.Mesh.Value.Indices),
                _ => new float4(collider.Center, 0f)
            };

            float3 surfacePoint = new float3(surfaceResult.x, surfaceResult.y, surfaceResult.z);

            // If we're outside the mesh, just return the closest point
            if ((collider.Container ? 1f : 0f) - surfaceResult.w == 0) return surfacePoint;

            // Otherwise, we've crossed the bounds and must use repulsion logic
            float3 direction = math.normalizesafe(position - surfacePoint);

            return position + direction * 1e-2f;
        }

        public static float4 ClosestPointOnSphere(float3 center, float radius, float3 point)
        {
            var dir = math.normalizesafe(point - center);

            return new float4(center + dir * radius, math.length(point - center) < radius ? 1f : 0f);
        }

        public static float4 ClosestPointOnBox(float3 center, float3 halfExtents, float3x3 worldRotation, float3 point)
        {
            // Convert world-space point to local box space
            float3 localPoint = math.mul(math.transpose(worldRotation), point - center);

            // Clamp to box bounds
            float3 clamped = math.clamp(localPoint, -halfExtents, halfExtents);

            // If inside box, project onto nearest face
            bool inside = math.all(localPoint >= -halfExtents) && math.all(localPoint <= halfExtents);
            if (inside)
            {
                float3 d = halfExtents - math.abs(localPoint);
                if (d.x < d.y && d.x < d.z)
                    clamped.x = halfExtents.x * math.sign(localPoint.x);
                else if (d.y < d.z)
                    clamped.y = halfExtents.y * math.sign(localPoint.y);
                else
                    clamped.z = halfExtents.z * math.sign(localPoint.z);
            }

            // Convert back to world space
            return new float4(center + math.mul(worldRotation, clamped), inside ? 1f : 0f);
        }

        public static float4 ClosestPointOnCapsule(float3 a, float3 b, float radius, float3 point)
        {
            var ab = b - a;
            var t = math.clamp(math.dot(point - a, ab) / math.lengthsq(ab), 0f, 1f);
            var closest = a + t * ab;

            return new float4(closest + math.normalizesafe(point - closest) * radius, math.length(point - closest) < radius ? 1f : 0f);
        }

        public static float4 ClosestPointOnMesh(float3 point, ref BlobArray<float3> verts, ref BlobArray<int> tris)
        {
            float3 closest = float3.zero;
            float closestNormalDot = 0f;
            float closestDistSq = float.MaxValue;

            for (int i = 0; i < tris.Length; i += 3)
            {
                var p0 = verts[tris[i]];
                var p1 = verts[tris[i + 1]];
                var p2 = verts[tris[i + 2]];

                var triNormal = math.normalize(math.cross(p1 - p0, p2 - p0));
                var q = ClosestPointOnTriangle(point, p0, p1, p2);
                var dirToPoint = math.normalize(point - q);
                float dot = math.dot(dirToPoint, triNormal);

                var distSq = math.distancesq(point, q);
                if (distSq < closestDistSq)
                {
                    closestDistSq = distSq;
                    closestNormalDot = dot;
                    closest = q;
                }
            }

            return new float4(closest, closestNormalDot < 0f ? 1f : 0f);
        }

        public static float3 ClosestPointOnTriangle(float3 p, float3 a, float3 b, float3 c)
        {
            var ab = b - a;
            var ac = c - a;
            var ap = p - a;

            float d1 = math.dot(ab, ap);
            float d2 = math.dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f) return a;

            var bp = p - b;
            float d3 = math.dot(ab, bp);
            float d4 = math.dot(ac, bp);
            if (d3 >= 0f && d4 <= d3) return b;

            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float v = d1 / (d1 - d3);
                return a + v * ab;
            }

            var cp = p - c;
            float d5 = math.dot(ab, cp);
            float d6 = math.dot(ac, cp);
            if (d6 >= 0f && d5 <= d6) return c;

            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w = d2 / (d2 - d6);
                return a + w * ac;
            }

            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return b + w * (c - b);
            }

            float denom = 1.0f / (va + vb + vc);
            float vInterp = vb * denom;
            float wInterp = vc * denom;
            return a + ab * vInterp + ac * wInterp;
        }
    }
}
