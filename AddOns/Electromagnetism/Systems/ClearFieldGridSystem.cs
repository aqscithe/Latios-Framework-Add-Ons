using Latios;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Latios.Anna.Electromagnetism.Systems
{
    /// <summary>
    /// Zeros the dynamic-overlay B grid at the start of each substep so source
    /// writes accumulate from a clean baseline.
    ///
    /// Tier 3 will replace the zero with a copy from the offline-baked static
    /// field; until then, "clean baseline" is literal zero.
    /// </summary>
    [DisableAutoCreation]
    [BurstCompile]
    public partial struct ClearFieldGridSystem : ISystem
    {
        LatiosWorldUnmanaged latiosWorld;

        public void OnCreate(ref SystemState state)
        {
            latiosWorld = state.GetLatiosWorldUnmanaged();
        }

        public void OnUpdate(ref SystemState state)
        {
            if (!latiosWorld.HasElectromagnetismSettings())
                return;

            var field = latiosWorld.sceneBlackboardEntity.GetCollectionComponent<ElectromagneticField>(false);
            if (!field.B.IsCreated || field.B.Length == 0)
                return;

            state.Dependency = new ClearJob
            {
                B = field.B,
            }.Schedule(field.B.Length, 4096, state.Dependency);
        }

        [BurstCompile]
        struct ClearJob : IJobParallelFor
        {
            public NativeArray<float3> B;

            public void Execute(int index)
            {
                B[index] = float3.zero;
            }
        }
    }
}
