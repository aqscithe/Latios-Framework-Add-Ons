using Latios;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Latios.Anna.Electromagnetism.Systems
{
    /// <summary>
    /// Allocates (and re-allocates on size change) the <see cref="ElectromagneticField"/>
    /// collection component on the scene blackboard, based on the authored
    /// <see cref="ElectromagnetismSettings"/>.
    ///
    /// Tier 1 allocates with <see cref="Allocator.Persistent"/> so the grid
    /// survives across frames without reallocation. <see cref="ElectromagneticField.TryDispose"/>
    /// handles cleanup when the collection component is replaced or the scene
    /// transitions.
    /// </summary>
    [DisableAutoCreation]
    [BurstCompile]
    public partial struct FieldGridLifecycleSystem : ISystem, ISystemNewScene
    {
        LatiosWorldUnmanaged latiosWorld;

        public void OnCreate(ref SystemState state)
        {
            latiosWorld = state.GetLatiosWorldUnmanaged();
        }

        public void OnNewScene(ref SystemState state)
        {
            // Seed the collection component with an empty grid. The first
            // OnUpdate after the settings entity bakes in will resize it.
            latiosWorld.sceneBlackboardEntity.AddOrSetCollectionComponentAndDisposeOld(default(ElectromagneticField));
        }

        public void OnUpdate(ref SystemState state)
        {
            if (!latiosWorld.HasElectromagnetismSettings())
                return;

            var settings = latiosWorld.GetElectromagnetismSettings();
            if (math.any(settings.gridResolution <= 0) || settings.cellSize <= 0f)
                return;

            int cellCount = settings.CellCount;
            var existing  = latiosWorld.sceneBlackboardEntity.GetCollectionComponent<ElectromagneticField>(false);

            // Already allocated at the right shape — leave it alone.
            if (existing.B.IsCreated
                && existing.B.Length == cellCount
                && math.all(existing.resolution == settings.gridResolution)
                && math.all(existing.origin == settings.gridOrigin)
                && existing.cellSize == settings.cellSize)
            {
                return;
            }

            // Allocate a fresh grid. The dispose-old path inside
            // SetCollectionComponentAndDisposeOld will fire TryDispose on the
            // previous instance (releasing its Persistent arrays).
            var fresh = new ElectromagneticField
            {
                B          = new NativeArray<float3>(cellCount, Allocator.Persistent, NativeArrayOptions.ClearMemory),
                muR        = new NativeArray<half>(cellCount,  Allocator.Persistent, NativeArrayOptions.UninitializedMemory),
                sigma      = new NativeArray<half>(cellCount,  Allocator.Persistent, NativeArrayOptions.ClearMemory),
                resolution = settings.gridResolution,
                origin     = settings.gridOrigin,
                cellSize   = settings.cellSize,
            };

            // Initialize μ_r to 1.0 (vacuum). Tier 1 keeps σ at 0 — induction
            // is Tier 3 and will populate σ when conductors get authored.
            half one = (half)1f;
            for (int i = 0; i < cellCount; i++)
                fresh.muR[i] = one;

            latiosWorld.sceneBlackboardEntity.SetCollectionComponentAndDisposeOld(fresh);
        }
    }
}
