using Latios.Psyshock;
using Latios.Transforms;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

using static Unity.Entities.SystemAPI;

namespace Latios.Anna.Systems
{
    [DisableAutoCreation]
    [BurstCompile]
    public partial struct CollectRigidBodiesSystem : ISystem, ISystemNewScene
    {
        LatiosWorldUnmanaged latiosWorld;
        EntityQuery          m_query;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            latiosWorld = state.GetLatiosWorldUnmanaged();

            m_query = state.Fluent().With<WorldTransform>(true).With<RigidBody>(false).Without<KinematicCollisionTag>().Build();
        }

        public void OnNewScene(ref SystemState state)
        {
            latiosWorld.sceneBlackboardEntity.AddOrSetCollectionComponentAndDisposeOld<CapturedRigidBodies>( default);
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var physicsSettings = latiosWorld.GetPhysicsSettings();
            var count           = m_query.CalculateEntityCountWithoutFiltering();
            if (count == 0)
            {
                latiosWorld.sceneBlackboardEntity.SetCollectionComponentAndDisposeOld(new CapturedRigidBodies
                {
                    entityToSrcIndexMap = new NativeParallelHashMap<Entity, int>(1, state.WorldUpdateAllocator),
                    states              = CollectionHelper.CreateNativeArray<CapturedRigidBodyState>(0, state.WorldUpdateAllocator)
                });
                return;
            }

            var startIndices     = m_query.CalculateBaseEntityIndexArrayAsync(state.WorldUpdateAllocator, default, out var jh);
            var states           = CollectionHelper.CreateNativeArray<CapturedRigidBodyState>(count, state.WorldUpdateAllocator, NativeArrayOptions.UninitializedMemory);
            var entityToIndexMap = new NativeParallelHashMap<Entity, int>(count, state.WorldUpdateAllocator);

            jh = new BuildJob
            {
                aabbHandle            = GetComponentTypeHandle<CollisionWorldAabb>(false),
                addImpulseHandle      = GetBufferTypeHandle<AddImpulse>(false),
                bucketCalculator      = new CollisionLayerBucketIndexCalculator(in physicsSettings.collisionLayerSettings),
                centerOverrideHandle  = GetComponentTypeHandle<LocalCenterOfMassOverride>(true),
                colliderHandle        = GetComponentTypeHandle<Collider>(true),
                dt                    = Time.DeltaTime,
                entityHandle          = GetEntityTypeHandle(),
                entityToIndexMap      = entityToIndexMap.AsParallelWriter(),
                inertiaOverrideHandle = GetComponentTypeHandle<LocalInertiaOverride>(true),
                physicsSettings       = physicsSettings,
                rigidBodyHandle       = GetComponentTypeHandle<RigidBody>(false),
                gravityOverrideHandle = GetComponentTypeHandle<GravityOverride>(true),
                timeScaleHandle       = GetComponentTypeHandle<TimeScale>(true),
                startIndices          = startIndices,
                states                = states,
                transformHandle       = GetComponentTypeHandle<WorldTransform>(true)
            }.ScheduleParallel(m_query, JobHandle.CombineDependencies(state.Dependency, jh));

            latiosWorld.sceneBlackboardEntity.SetCollectionComponentAndDisposeOld(new CapturedRigidBodies
            {
                entityToSrcIndexMap = entityToIndexMap,
                states              = states
            });
            state.Dependency = jh;
        }

        [BurstCompile]
        partial struct BuildJob : IJobChunk
        {
            [ReadOnly] public EntityTypeHandle                               entityHandle;
            [ReadOnly] public ComponentTypeHandle<WorldTransform>            transformHandle;
            [ReadOnly] public ComponentTypeHandle<Collider>                  colliderHandle;
            [ReadOnly] public ComponentTypeHandle<LocalCenterOfMassOverride> centerOverrideHandle;
            [ReadOnly] public ComponentTypeHandle<LocalInertiaOverride>      inertiaOverrideHandle;
            [ReadOnly] public ComponentTypeHandle<GravityOverride>           gravityOverrideHandle;
            [ReadOnly] public ComponentTypeHandle<TimeScale>                 timeScaleHandle;
            [ReadOnly] public NativeArray<int>                               startIndices;

            public ComponentTypeHandle<RigidBody>          rigidBodyHandle;
            public BufferTypeHandle<AddImpulse>            addImpulseHandle;
            public ComponentTypeHandle<CollisionWorldAabb> aabbHandle;

            [NativeDisableParallelForRestriction] public NativeArray<CapturedRigidBodyState> states;
            public NativeParallelHashMap<Entity, int>.ParallelWriter                         entityToIndexMap;

            public PhysicsSettings                     physicsSettings;
            public CollisionLayerBucketIndexCalculator bucketCalculator;
            public float                               dt;

            public unsafe void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                var entities         = chunk.GetEntityDataPtrRO(entityHandle);
                var transforms       = (WorldTransform*)chunk.GetRequiredComponentDataPtrRO(ref transformHandle);
                var colliders        = chunk.GetComponentDataPtrRO(ref colliderHandle);
                var centerOverrides  = chunk.GetComponentDataPtrRO(ref centerOverrideHandle);
                var inertiaOverrides = chunk.GetComponentDataPtrRO(ref inertiaOverrideHandle);
                var gravityOverrides = chunk.GetComponentDataPtrRO(ref gravityOverrideHandle);
                var timeScales       = chunk.GetComponentDataPtrRO(ref timeScaleHandle);
                var rigidBodies      = (RigidBody*)chunk.GetRequiredComponentDataPtrRW(ref rigidBodyHandle);
                var impulses         = chunk.GetBufferAccessor(ref addImpulseHandle);
                var aabbs            = (Aabb*)chunk.GetComponentDataPtrRW(ref aabbHandle);

                for (int i = 0, index = startIndices[unfilteredChunkIndex]; i < chunk.Count; i++, index++)
                {
                    entityToIndexMap.TryAdd(entities[i], index);

                    var     entity    = entities[i];
                    ref var transform = ref transforms[i];
                    ref var rigidBody = ref rigidBodies[i];

                    Collider collider          = colliders == null ? default(SphereCollider) : colliders[i];
                    var      aabb              = Physics.AabbFrom(in collider, in transform.worldTransform);
                    var      angularExpansion  = UnitySim.AngularExpansionFactorFrom(in collider);
                    var      localCenterOfMass = centerOverrides == null ? UnitySim.LocalCenterOfMassFrom(in collider) : centerOverrides[i].centerOfMass;
                    var      localInertia      = inertiaOverrides == null ? UnitySim.LocalInertiaTensorFrom(in collider, transform.stretch) : inertiaOverrides[i].inertiaDiagonal;
                    UnitySim.ConvertToWorldMassInertia(in transform.worldTransform,
                                                       in localInertia,
                                                       localCenterOfMass,
                                                       rigidBody.inverseMass,
                                                       out var mass,
                                                       out var inertialPoseWorldTransform);

                    
                    float timeScale = timeScales == null ? 1.0f : math.max(timeScales[i].timescale, 1e-6f);
                    float deltaTime = timeScale * dt;

                    float3 gravity = gravityOverrides == null ? physicsSettings.gravity : gravityOverrides[i].gravity;
                    
                    rigidBody.velocity.linear += gravity * deltaTime;

                    if (impulses.Length > 0)
                    {
                        

                        foreach (var impulse in impulses[i])
                        {
                            //var scaledImpulse = impulse.impulse * timeScale;

                            if (math.all(math.isnan(impulse.pointOrAxis)))
                                UnitySim.ApplyFieldImpulse(ref rigidBody.velocity, in mass, impulse.impulse);
                            else if (math.all(math.isnan(impulse.impulse.yz)))
                                UnitySim.ApplyAngularImpulse(ref rigidBody.velocity, in mass, in inertialPoseWorldTransform, impulse.pointOrAxis, impulse.impulse.x);
                            else
                                UnitySim.ApplyImpulseAtWorldPoint(ref rigidBody.velocity, in mass, in inertialPoseWorldTransform, impulse.pointOrAxis, impulse.impulse);
                        }
                        impulses[i].Clear();
                    }

                    var motionExpansion = new UnitySim.MotionExpansion(in rigidBody.velocity, deltaTime, angularExpansion);
                    aabb                = motionExpansion.ExpandAabb(aabb);
                    var bucketIndex     = bucketCalculator.BucketIndexFrom(in aabb);

                    if (aabbs != null)
                        aabbs[i] = aabb;

                    states[index] = new CapturedRigidBodyState
                    {
                        angularDamping                     = physicsSettings.angularDamping,
                        angularExpansion                   = angularExpansion,
                        bucketIndex                        = bucketIndex,
                        coefficientOfFriction              = rigidBody.coefficientOfFriction,
                        coefficientOfRestitution           = rigidBody.coefficientOfRestitution,
                        gravity                            = gravity,
                        inertialPoseWorldTransform         = inertialPoseWorldTransform,
                        linearDamping                      = physicsSettings.linearDamping,
                        mass                               = mass,
                        motionExpansion                    = motionExpansion,
                        motionStabilizer                   = new UnitySim.MotionStabilizer(in rigidBody.velocity),
                        numOtherSignificantBodiesInContact = 0,
                        velocity                           = rigidBody.velocity,
                        timeScale                          = timeScale,
                    };
                }
            }
        }
    }
}
