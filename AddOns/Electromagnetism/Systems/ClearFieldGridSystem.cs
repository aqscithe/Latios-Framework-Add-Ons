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
    ///
    /// <para><b>Idle-frame skip:</b> when no sources exist and the grid is
    /// already zero from the previous frame's clear, this system early-outs
    /// entirely — saves the whole clear pass in the common gameplay state
    /// where no magnet is active. The <c>_fieldIsDirty</c> flag tracks
    /// whether last frame's source-write pass wrote anything into B; the
    /// clear then runs on the *transition* frame after sources disappear
    /// (to zero out the stale field) and on every frame that has sources.</para>
    /// </summary>
    [DisableAutoCreation]
    [BurstCompile]
    public partial struct ClearFieldGridSystem : ISystem
    {
        LatiosWorldUnmanaged latiosWorld;
        EntityQuery          _permanentQuery;
        EntityQuery          _electromagnetQuery;
        EntityQuery          _wireSegmentQuery;
        bool                 _fieldIsDirty;

        public void OnCreate(ref SystemState state)
        {
            latiosWorld = state.GetLatiosWorldUnmanaged();

            // Mirror WriteSourceContributionsSystem's source classification —
            // any of these three component types on an entity means "a source
            // will write into B this frame".
            _permanentQuery     = state.Fluent().With<PermanentMagnet>(true).Build();
            _electromagnetQuery = state.Fluent().With<Electromagnet>(true).Build();
            _wireSegmentQuery   = state.Fluent().With<WireSegment>(true).Build();

            _fieldIsDirty = false;
        }

        public void OnUpdate(ref SystemState state)
        {
            if (!latiosWorld.HasElectromagnetismSettings())
                return;

            var field = latiosWorld.sceneBlackboardEntity.GetCollectionComponent<ElectromagneticField>(false);
            if (!field.B.IsCreated || field.B.Length == 0)
                return;

            bool hasSources = !_permanentQuery.IsEmpty
                           || !_electromagnetQuery.IsEmpty
                           || !_wireSegmentQuery.IsEmpty;

            // Skip when there's nothing to zero. Two conditions must both
            // hold: no sources this frame (nothing will write into B) AND
            // last frame also had no sources (so B is already zero from the
            // previous clear or never written to). Otherwise stale field
            // values would linger and receivers would feel force from a
            // magnet that's no longer there.
            if (!hasSources && !_fieldIsDirty)
                return;

            state.Dependency = new ClearJob
            {
                B = field.B,
            }.Schedule(field.B.Length, 4096, state.Dependency);

            // B will be written to non-zero this frame iff sources ran. Next
            // frame's early-out reads this to decide whether a transition
            // clear is still needed.
            _fieldIsDirty = hasSources;
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
