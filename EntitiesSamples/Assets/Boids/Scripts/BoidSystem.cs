using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Burst;
using UnityEngine;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Transforms;
using System.Diagnostics;

// Mike's GDC Talk on 'A Data Oriented Approach to Using Component Systems'
// is a great reference for dissecting the Boids sample code:
// https://youtu.be/p65Yt20pw0g?t=1446
// It explains a slightly older implementation of this sample but almost all the
// information is still relevant.

// The targets (2 red fish) and obstacle (1 shark) move based on the ActorAnimation tab
// in the Unity UI, so that they are moving based on key-framed animation.

[assembly: RegisterGenericComponentType(typeof(Boids.BoidCollider))]

namespace Boids
{
    [RequireMatchingQueriesForUpdate]
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateBefore(typeof(TransformSystemGroup))]
    public partial struct BoidSystem : ISystem
    {
        private ComponentLookup<BoidCollider> _colliderLookup;
        private BufferLookup<LinkedEntityGroup> _linkedGroupLookup;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            _colliderLookup = state.GetComponentLookup<BoidCollider>(true);
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            _colliderLookup.Update(ref state);

            var boidQuery = SystemAPI.QueryBuilder().WithAll<Boid>().WithAllRW<LocalToWorld>().Build();
            var targetQuery = SystemAPI.QueryBuilder().WithAll<BoidTarget, LocalToWorld>().Build();
            var obstacleQuery = SystemAPI.QueryBuilder().WithAll<BoidObstacle, LocalToWorld>().Build();

            var obstacleCount = obstacleQuery.CalculateEntityCount();
            var targetCount = targetQuery.CalculateEntityCount();

            var world = state.WorldUnmanaged;
            state.EntityManager.GetAllUniqueSharedComponents(out NativeList<Boid> uniqueBoidTypes, world.UpdateAllocator.ToAllocator);
            float dt = math.min(0.05f, SystemAPI.Time.DeltaTime);

            var targetEntities = targetQuery.ToEntityArray(world.UpdateAllocator.ToAllocator);
            var obstacleEntities = obstacleQuery.ToEntityArray(world.UpdateAllocator.ToAllocator);

            // Each variant of the Boid represents a different value of the SharedComponentData and is self-contained,
            // meaning Boids of the same variant only interact with one another. Thus, this loop processes each
            // variant type individually.
            foreach (var boidSettings in uniqueBoidTypes)
            {
                boidQuery.AddSharedComponentFilter(boidSettings);

                var boidCount = boidQuery.CalculateEntityCount();
                if (boidCount == 0)
                {
                    // Early out. If the given variant includes no Boids, move on to the next loop.
                    // For example, variant 0 will always exit early bc it's it represents a default, uninitialized
                    // Boid struct, which does not appear in this sample.
                    boidQuery.ResetFilter();
                    continue;
                }

                // The following calculates spatial cells of neighboring Boids
                // note: working with a sparse grid and not a dense bounded grid so there
                // are no predefined borders of the space.

                var hashMap = new NativeParallelMultiHashMap<int, int>(boidCount, world.UpdateAllocator.ToAllocator);
                var cellIndices = CollectionHelper.CreateNativeArray<int, RewindableAllocator>(boidCount, ref world.UpdateAllocator);
                var cellObstaclePositions = CollectionHelper.CreateNativeArray<float3, RewindableAllocator>(boidCount, ref world.UpdateAllocator);
                var cellTargetPositions = CollectionHelper.CreateNativeArray<float3, RewindableAllocator>(boidCount, ref world.UpdateAllocator);
                var cellCount = CollectionHelper.CreateNativeArray<int, RewindableAllocator>(boidCount, ref world.UpdateAllocator);
                var cellAlignment = CollectionHelper.CreateNativeArray<float3, RewindableAllocator>(boidCount, ref world.UpdateAllocator);
                var cellSeparation = CollectionHelper.CreateNativeArray<float3, RewindableAllocator>(boidCount, ref world.UpdateAllocator);

                var copyTargetPositions = CollectionHelper.CreateNativeArray<float3, RewindableAllocator>(targetCount, ref world.UpdateAllocator);
                var copyObstaclePositions = CollectionHelper.CreateNativeArray<float3, RewindableAllocator>(obstacleCount, ref world.UpdateAllocator);

                var copyTargetGroups = CollectionHelper.CreateNativeArray<int, RewindableAllocator>(targetCount, ref world.UpdateAllocator);
                var copyObstacleGroups = CollectionHelper.CreateNativeArray<int, RewindableAllocator>(obstacleCount, ref world.UpdateAllocator);

                var copyTargetWeights = CollectionHelper.CreateNativeArray<float, RewindableAllocator>(targetCount, ref world.UpdateAllocator);
                var copyObstacleWeights = CollectionHelper.CreateNativeArray<float, RewindableAllocator>(obstacleCount, ref world.UpdateAllocator);

                var copyTargetAttractionDistances = CollectionHelper.CreateNativeArray<float, RewindableAllocator>(targetCount, ref world.UpdateAllocator);
                var copyObstacleAvoidanceDistances = CollectionHelper.CreateNativeArray<float, RewindableAllocator>(obstacleCount, ref world.UpdateAllocator);



                // These jobs extract the relevant position, heading component
                // to NativeArrays so that they can be randomly accessed by the `MergeCells` and `Steer` jobs.
                // These jobs are defined using the IJobEntity syntax.
                var initialBoidJob = new InitialPerBoidJob
                {
                    CellAlignment = cellAlignment,
                    CellSeparation = cellSeparation,
                    ParallelHashMap = hashMap.AsParallelWriter(),
                    InverseBoidCellRadius = 1.0f / boidSettings.CellRadius,
                };
                var initialBoidJobHandle = initialBoidJob.ScheduleParallel(boidQuery, state.Dependency);

                var initialTargetJob = new InitialPerTargetJob
                {
                    TargetPositions = copyTargetPositions,
                    TargetGroups = copyTargetGroups,
                    TargetWeights = copyTargetWeights,
                    TargetDistances = copyTargetAttractionDistances,
                };
                var initialTargetJobHandle = initialTargetJob.ScheduleParallel(targetQuery, state.Dependency);

                var initialObstacleJob = new InitialPerObstacleJob
                {
                    ObstaclePositions = copyObstaclePositions,
                    ObstacleGroups = copyObstacleGroups,
                    ObstacleWeights = copyObstacleWeights,
                    ObstacleAvoidanceDistances = copyObstacleAvoidanceDistances,
                };
                var initialObstacleJobHandle = initialObstacleJob.ScheduleParallel(obstacleQuery, state.Dependency);

                var initialCellCountJob = new MemsetNativeArray<int>
                {
                    Source = cellCount,
                    Value = 1
                };
                var initialCellCountJobHandle = initialCellCountJob.Schedule(boidCount, 64, state.Dependency);

                var initialCellBarrierJobHandle = JobHandle.CombineDependencies(initialBoidJobHandle, initialCellCountJobHandle);
                var copyTargetObstacleBarrierJobHandle = JobHandle.CombineDependencies(initialTargetJobHandle, initialObstacleJobHandle);
                var mergeCellsBarrierJobHandle = JobHandle.CombineDependencies(initialCellBarrierJobHandle, copyTargetObstacleBarrierJobHandle);

                var mergeCellsJob = new MergeCells
                {
                    currentGroup = boidSettings.Group,
                    cellIndices = cellIndices,
                    cellAlignment = cellAlignment,
                    cellSeparation = cellSeparation,
                    cellObstaclePositions = cellObstaclePositions,
                    cellTargetPositions = cellTargetPositions,
                    cellCount = cellCount,
                    targetPositions = copyTargetPositions,
                    obstaclePositions = copyObstaclePositions,
                    targetGroups = copyTargetGroups,
                    obstacleGroups = copyObstacleGroups,
                    targetWeights = copyTargetWeights,
                    obstacleWeights = copyObstacleWeights,
                    targetDistances = copyTargetAttractionDistances,
                    obstacleDistances = copyObstacleAvoidanceDistances,
                    colliderLookup = _colliderLookup,
                    obstacleEntities = obstacleEntities,
                    targetEntities = targetEntities,
                };
                var mergeCellsJobHandle = mergeCellsJob.Schedule(hashMap, 64, mergeCellsBarrierJobHandle);

                // This reads the previously calculated boid information for all the boids of each cell to update
                // the `localToWorld` of each of the boids based on their newly calculated headings using
                // the standard boid flocking algorithm.
                var steerBoidJob = new SteerBoidJob
                {
                    CellIndices = cellIndices,
                    CellCount = cellCount,
                    CellAlignment = cellAlignment,
                    CellSeparation = cellSeparation,
                    CellObstaclePositions = cellObstaclePositions,
                    CellTargetPositions = cellTargetPositions,
                    ObstaclePositions = copyObstaclePositions,
                    TargetPositions = copyTargetPositions,
                    CurrentBoidVariant = boidSettings,
                    DeltaTime = dt,
                    MoveDistance = boidSettings.MoveSpeed * dt,
                };
                var steerBoidJobHandle = steerBoidJob.ScheduleParallel(boidQuery, mergeCellsJobHandle);

                // Dispose allocated containers with dispose jobs.
                state.Dependency = steerBoidJobHandle;

                // We pass the job handle and add the dependency so that we keep the proper ordering between the jobs
                // as the looping iterates. For our purposes of execution, this ordering isn't necessary; however, without
                // the add dependency call here, the safety system will throw an error, because we're accessing multiple
                // pieces of boid data and it would think there could possibly be a race condition.

                boidQuery.AddDependency(state.Dependency);
                boidQuery.ResetFilter();
            }
            uniqueBoidTypes.Dispose();
        }

        // In this sample there are 3 total unique boid variants, one for each unique value of the
        // Boid SharedComponent (note: this includes the default uninitialized value at
        // index 0, which isnt actually used in the sample).

        // This accumulates the `positions` (separations) and `headings` (alignments) of all the boids in each cell to:
        // 1) count the number of boids in each cell
        // 2) find the nearest obstacle and target to each boid cell
        // 3) track which array entry contains the accumulated values for each boid's cell
        // In this context, the cell represents the hashed bucket of boids that are near one another within cellRadius
        // floored to the nearest int3.
        // Note: `IJobNativeParallelMultiHashMapMergedSharedKeyIndices` is a custom job to iterate safely/efficiently over the
        // NativeContainer used in this sample (`NativeParallelMultiHashMap`). Currently these kinds of changes or additions of
        // custom jobs generally require access to data/fields that aren't available through the `public` API of the
        // containers. This is why the custom job type `IJobNativeParallelMultiHashMapMergedSharedKeyIndicies` is declared in
        // the DOTS package (which can see the `internal` container fields) and not in the Boids sample.
        [BurstCompile]
        struct MergeCells : IJobNativeParallelMultiHashMapMergedSharedKeyIndices
        {
            public int currentGroup;
            public NativeArray<int> cellIndices;
            public NativeArray<float3> cellAlignment;
            public NativeArray<float3> cellSeparation;
            public NativeArray<float3> cellObstaclePositions;

            public NativeArray<float3> cellTargetPositions;

            public NativeArray<int> cellCount;
            [ReadOnly] public NativeArray<float3> targetPositions;
            [ReadOnly] public NativeArray<float3> obstaclePositions;
            [ReadOnly] public NativeArray<int> targetGroups;
            [ReadOnly] public NativeArray<int> obstacleGroups;
            [ReadOnly] public NativeArray<float> targetWeights;
            [ReadOnly] public NativeArray<float> obstacleWeights;
            [ReadOnly] public NativeArray<float> targetDistances;
            [ReadOnly] public NativeArray<float> obstacleDistances;

            [ReadOnly] public NativeArray<Entity> targetEntities;
            [ReadOnly] public NativeArray<Entity> obstacleEntities;
            [ReadOnly] public ComponentLookup<BoidCollider> colliderLookup;

            // Alternative to nearest, get weighted average of targets with weights based on difference
            void AveragePosition(NativeArray<float3> targets, NativeArray<float> weights, NativeArray<float> distances, NativeArray<Entity> entities, NativeArray<int> groups, float3 position, int currentGroup, out float3 averagePosition)
            {
                averagePosition = float3.zero;
                float totalWeight = 0f;

                for (int i = 0; i < targets.Length; i++)
                {
                    if ((groups[i] & currentGroup) == 0)
                        continue;

                    // Default target position
                    var targetPosition = targets[i];


                    if (colliderLookup.HasComponent(entities[i]))
                    {
                        BoidCollider collider = colliderLookup[entities[i]];
                        targetPosition = BoidColliderUtility.SampleNearestPoint(collider, in position);
                    }

                    // Actual distance minus target distance
                    var distance = math.max(math.square(math.length(position - targetPosition) - distances[i]), 1e-3f); // Avoid div by zero
                    var weight = weights[i] / distance;

                    totalWeight += weight;
                    averagePosition += targetPosition * weight;
                }

                averagePosition = totalWeight > 0 ? averagePosition / totalWeight : 0f;
            }

            // Resolves the distance of the nearest obstacle and target and stores the cell index.
            public void ExecuteFirst(int index)
            {
                var position = cellSeparation[index] / cellCount[index];

                float3 obstaclePosition;
                AveragePosition(obstaclePositions, obstacleWeights, obstacleDistances, obstacleEntities, obstacleGroups, position, currentGroup, out obstaclePosition);
                cellObstaclePositions[index] = obstaclePosition;

                float3 targetPosition;
                AveragePosition(targetPositions, targetWeights, targetDistances, targetEntities, targetGroups, position, currentGroup, out targetPosition);
                cellTargetPositions[index] = targetPosition;

                cellIndices[index] = index;
            }

            // Sums the alignment and separation of the actual index being considered and stores
            // the index of this first value where we're storing the cells.
            // note: these items are summed so that in `Steer` their average for the cell can be resolved.
            public void ExecuteNext(int cellIndex, int index)
            {
                cellCount[cellIndex] += 1;
                cellAlignment[cellIndex] += cellAlignment[cellIndex];
                cellSeparation[cellIndex] += cellSeparation[cellIndex];
                cellIndices[index] = cellIndex;
            }
        }

        [BurstCompile]
        partial struct InitialPerBoidJob : IJobEntity
        {
            public NativeArray<float3> CellAlignment;
            public NativeArray<float3> CellSeparation;
            public NativeParallelMultiHashMap<int, int>.ParallelWriter ParallelHashMap;
            public float InverseBoidCellRadius;
            void Execute([EntityIndexInQuery] int entityIndexInQuery, in LocalToWorld localToWorld)
            {
                CellAlignment[entityIndexInQuery] = localToWorld.Forward;
                CellSeparation[entityIndexInQuery] = localToWorld.Position;
                // Populates a hash map, where each bucket contains the indices of all Boids whose positions quantize
                // to the same value for a given cell radius so that the information can be randomly accessed by
                // the `MergeCells` and `Steer` jobs.
                // This is useful in terms of the algorithm because it limits the number of comparisons that will
                // actually occur between the different boids. Instead of for each boid, searching through all
                // boids for those within a certain radius, this limits those by the hash-to-bucket simplification.
                var hash = (int)math.hash(new int3(math.floor(localToWorld.Position * InverseBoidCellRadius)));
                ParallelHashMap.Add(hash, entityIndexInQuery);
            }
        }

        [BurstCompile]
        partial struct InitialPerTargetJob : IJobEntity
        {
            public NativeArray<float3> TargetPositions;
            public NativeArray<int> TargetGroups;
            public NativeArray<float> TargetWeights;
            public NativeArray<float> TargetDistances;
            void Execute([EntityIndexInQuery] int entityIndexInQuery, in BoidTarget boidTarget, in LocalToWorld localToWorld)
            {
                TargetPositions[entityIndexInQuery] = localToWorld.Position;
                TargetGroups[entityIndexInQuery] = boidTarget.Group;
                TargetWeights[entityIndexInQuery] = boidTarget.Weight;
                TargetDistances[entityIndexInQuery] = boidTarget.TargetDistance;
            }
        }

        [BurstCompile]
        partial struct InitialPerObstacleJob : IJobEntity
        {
            public NativeArray<float3> ObstaclePositions;
            public NativeArray<int> ObstacleGroups;
            public NativeArray<float> ObstacleWeights;
            public NativeArray<float> ObstacleAvoidanceDistances;
            void Execute([EntityIndexInQuery] int entityIndexInQuery, in BoidObstacle boidObstacle, in LocalToWorld localToWorld)
            {
                ObstaclePositions[entityIndexInQuery] = localToWorld.Position;
                ObstacleGroups[entityIndexInQuery] = boidObstacle.Group;
                ObstacleWeights[entityIndexInQuery] = boidObstacle.Weight;
                ObstacleAvoidanceDistances[entityIndexInQuery] = boidObstacle.AvoidanceDistance;
            }
        }

        [BurstCompile]
        partial struct SteerBoidJob : IJobEntity
        {
            [ReadOnly] public NativeArray<int> CellIndices;
            [ReadOnly] public NativeArray<int> CellCount;
            [ReadOnly] public NativeArray<float3> CellAlignment;
            [ReadOnly] public NativeArray<float3> CellSeparation;
            [ReadOnly] public NativeArray<float3> CellObstaclePositions;
            [ReadOnly] public NativeArray<float3> CellTargetPositions;
            [ReadOnly] public NativeArray<float3> ObstaclePositions;
            [ReadOnly] public NativeArray<float3> TargetPositions;
            public Boid CurrentBoidVariant;
            public float DeltaTime;
            public float MoveDistance;
            void Execute([EntityIndexInQuery] int entityIndexInQuery, ref LocalToWorld localToWorld)
            {
                // temporarily storing the values for code readability
                var forward = localToWorld.Forward;
                var currentPosition = localToWorld.Position;
                var cellIndex = CellIndices[entityIndexInQuery];
                var neighborCount = CellCount[cellIndex];
                var alignment = CellAlignment[cellIndex];
                var separation = CellSeparation[cellIndex];
                var obstaclePosition = CellObstaclePositions[cellIndex];
                var targetPosition = CellTargetPositions[cellIndex];

                // Setting up the directions for the three main biocrowds influencing directions adjusted based
                // on the predefined weights:
                // 1) alignment - how much should it move in a direction similar to those around it?
                // note: we use `alignment/neighborCount`, because we need the average alignment in this case; however
                // alignment is currently the summation of all those of the boids within the cellIndex being considered.
                var alignmentResult = CurrentBoidVariant.AlignmentWeight
                                          * math.normalizesafe((alignment / neighborCount) - forward);
                // 2) separation - how close is it to other boids and are there too many or too few for comfort?
                // note: here separation represents the summed possible center of the cell. We perform the multiplication
                // so that both `currentPosition` and `separation` are weighted to represent the cell as a whole and not
                // the current individual boid.
                var separationResult = CurrentBoidVariant.SeparationWeight
                                          * math.normalizesafe((currentPosition * neighborCount) - separation);

                // 3) Unity devs forgot about cohesion! Let's calculate that as well -- very similar to separationResult
                var cohesionResult = CurrentBoidVariant.CohesionWeight
                                            * math.normalizesafe((separation / neighborCount) - currentPosition);

                // 4) target - is it still towards its destination?
                var targetResult = TargetPositions.Length > 0 ? CurrentBoidVariant.TargetWeight
                                          * math.normalizesafe(targetPosition - currentPosition)
                                          : float3.zero;    // Zero if no targets

                // 5) obstacles - "weight" has to be different here. In order to ensure the boid never
                // reaches the obstacle, the weight is the exposed weight times inverse of the distance to
                // the specified threshold.
                //
                // The original logic calculated an avoidance vector that was
                // towards the obstacle, but within the specified range, inverted when the entity is
                // too close to the obstacle.
                // var obstacleAvoidanceDirection = (obstaclePosition + math.normalizesafe(currentPosition - obstaclePosition))
                //                                 * CurrentBoidVariant.ObstacleAversionDistance
                //                                 - currentPosition;

                // In this version, we follow the more standard setup and
                // choose the vector away from the obstacle as the direction
                var obstacleAvoidanceDirection = currentPosition - obstaclePosition;

                var obstacleResult = ObstaclePositions.Length > 0 ? math.square(CurrentBoidVariant.ObstacleWeight)
                                            * (1 / (
                                                math.max(                                           // Avoid div by 0 or negative
                                                    math.length(currentPosition - obstaclePosition)
                                                    - CurrentBoidVariant.ObstacleAversionDistance,  // Distance from aversion threshold
                                                    1e-3f
                                                )
                                            ))
                                            * math.normalizesafe(obstacleAvoidanceDirection)  // Vector away from obstacle
                                            : float3.zero;  // Zero if no obstacles

                // Combine all vectors to one forward vector, normalize to find direction
                var targetForward = math.normalizesafe(alignmentResult + separationResult + cohesionResult + targetResult + obstacleResult);

                // Updates using the newly calculated heading direction -- must allow entities to
                // change direction as fast as is necessary to avoid obstacles
                var nextHeading = math.normalizesafe(forward + DeltaTime * math.clamp(math.length(obstacleResult), 1f, 1f / DeltaTime) * (targetForward - forward));
                localToWorld = new LocalToWorld
                {
                    Value = float4x4.TRS(
                        // TODO: precalc speed*dt
                        new float3(localToWorld.Position + (nextHeading * MoveDistance)),
                        quaternion.LookRotationSafe(nextHeading, math.up()),
                        CurrentBoidVariant.Scale
                    )
                };
            }
        }
    }
}
