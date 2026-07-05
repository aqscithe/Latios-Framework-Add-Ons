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
    ///
    /// <para><b>TODO — player-centred grid origin.</b> The grid is currently
    /// authored at a fixed world-space <c>gridOrigin</c>, which means the
    /// player can walk out of the field's AABB and lose all magnetic
    /// interaction. Because gameplay magnetism is inherently local (an
    /// InfluenceRadius of a few metres for typical magnets, and the dipole
    /// force falls off as 1/r⁴), the field only ever needs to cover the
    /// volume immediately around the player. Move the grid to track the
    /// player each frame:</para>
    /// <list type="bullet">
    ///   <item><description><b>Where:</b> add a "recentre" step to this
    ///     system's <see cref="OnUpdate"/>, after the resize check. Read the
    ///     player's world position, compute
    ///     <c>desiredOrigin = playerPos - 0.5 · resolution · cellSize</c>,
    ///     and write it to <c>field.origin</c>. Do <i>not</i> reallocate the
    ///     arrays — resolution and cellSize haven't changed, only the
    ///     origin. The receiver-force system re-samples every frame using
    ///     the current origin, so a bare origin write is sufficient.</description></item>
    ///   <item><description><b>Snap to cell grid.</b> Snap
    ///     <c>desiredOrigin</c> to integer multiples of <c>cellSize</c>
    ///     (<c>math.floor(desiredOrigin / cellSize) · cellSize</c>) so
    ///     small player motion doesn't slide the grid continuously — that
    ///     would make every cell's "meaning" wobble sub-cell each frame
    ///     and add temporal noise to the sampled B. Snapping means the
    ///     grid only shifts when the player crosses a cell boundary.</description></item>
    ///   <item><description><b>Advection is free here.</b> When the origin
    ///     shifts by one or more cells, the cells that "enter" the new AABB
    ///     hold stale data from the previous frame's write pattern. Because
    ///     <see cref="ClearFieldGridSystem"/> zeros <c>B</c> at the start of
    ///     every substep and <see cref="Systems.WriteSourceContributionsSystem"/>
    ///     re-stamps sources from scratch, that stale data is overwritten
    ///     before any receiver samples it — no explicit advection step is
    ///     needed. (This changes if a Tier 3 baked-static-field or Faraday
    ///     induction pass is added, since those carry temporal state that
    ///     would need to shift with the origin.)</description></item>
    ///   <item><description><b>Follow target.</b> "The player" is
    ///     <c>PlayerComponent</c>-tagged entity's <c>WorldTransform</c>
    ///     today; in split-screen / multi-player this becomes "the closest
    ///     player" or "AABB union of active players" — decide at the time.
    ///     Split into a separate <c>RecenterFieldGridSystem</c> if the
    ///     follow logic grows non-trivial.</description></item>
    ///   <item><description><b>Debug viz coupling.</b>
    ///     <c>SVORPElectromagnetismGridOverlayPass</c> and the field-lines
    ///     overlay both read <c>field.origin</c> to place their world-space
    ///     geometry, so they'll follow automatically. Nothing to update on
    ///     that side.</description></item>
    /// </list>
    /// <para>Result once wired: 64³ (or even lower) is enough for the whole
    /// game because the field only ever covers ~<c>cellSize · resolution</c>
    /// metres around the player — no cost scaling with world size.</para>
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
                Btemp      = new NativeArray<float3>(cellCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory),
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
