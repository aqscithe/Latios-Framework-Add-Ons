using Latios;
using Latios.Anna.Electromagnetism.Internal;
using Latios.Transforms;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Latios.Anna.Electromagnetism.Systems
{
    /// <summary>
    /// Phase B: For every <see cref="PermanentMagnet"/> source, compute its
    /// world-space dipole moment from the body's rotation, store it on
    /// <see cref="MagneticDipoleMoment"/>, then accumulate its analytical
    /// dipole field into every grid cell within the source's influence radius.
    ///
    /// Tier 1 runs single-threaded over sources because multiple sources may
    /// write to the same cells (e.g., two close magnets) and we want to avoid
    /// the atomic-write complexity. With ~few sources and ~thousands of cells
    /// per source this is fine. Tier 2 parallelizes either over sources (with
    /// atomic accumulation) or over cells (each cell sums contributions from
    /// all in-range sources, no atomics needed).
    ///
    /// Self-contribution note: a source writes its own field to nearby cells,
    /// which means a permanent magnet sampling the grid at its own position
    /// will see its own contribution. The matching subtraction lives in
    /// <c>ComputeReceiverForcesSystem.PermanentReceiverJob</c>, which uses
    /// <see cref="EMMath.SampleSelfDipoleContribution"/> /
    /// <see cref="EMMath.GradientSelfDipoleContribution"/> to remove exactly
    /// what was deposited here before computing F/τ.
    /// </summary>
    [DisableAutoCreation]
    [BurstCompile]
    public partial struct WriteSourceContributionsSystem : ISystem
    {
        LatiosWorldUnmanaged latiosWorld;
        EntityQuery          _sourceQuery;

        public void OnCreate(ref SystemState state)
        {
            latiosWorld  = state.GetLatiosWorldUnmanaged();
            _sourceQuery = state.Fluent()
                .With<PermanentMagnet>(true)
                .With<WorldTransform>(true)
                .With<MagneticDipoleMoment>(false)
                .Without<EMEffectImmuneTag>()
                .Build();
        }

        public void OnUpdate(ref SystemState state)
        {
            if (!latiosWorld.HasElectromagnetismSettings())
                return;
            if (_sourceQuery.IsEmpty)
                return;

            var field = latiosWorld.sceneBlackboardEntity.GetCollectionComponent<ElectromagneticField>(false);
            if (!field.B.IsCreated || field.B.Length == 0)
                return;

            state.Dependency = new WritePermanentMagnetsJob
            {
                B          = field.B,
                resolution = field.resolution,
                origin     = field.origin,
                cellSize   = field.cellSize,
            }.Schedule(_sourceQuery, state.Dependency);
        }

        /// <summary>
        /// Single-threaded iterate-over-sources job. Updates each source's
        /// world dipole moment as a side effect, then writes the analytical
        /// dipole field into the grid over the source's influence sphere.
        /// </summary>
        [BurstCompile]
        partial struct WritePermanentMagnetsJob : IJobEntity
        {
            [NativeDisableContainerSafetyRestriction] public NativeArray<float3> B;

            public int3   resolution;
            public float3 origin;
            public float  cellSize;

            void Execute(in PermanentMagnet permanent,
                         in WorldTransform  transform,
                         ref MagneticDipoleMoment moment)
            {
                // Permanent moment in world space = rotated body-local moment,
                // scaled by Curie state. WorldTransform.worldTransform.rotation
                // is the QVVS V2 rotation accessor.
                var rotation = transform.worldTransform.rotation;
                float3 worldMoment = math.mul(rotation, permanent.localMoment) * (float)permanent.curieScale;

                moment.worldMoment = worldMoment;

                // Bail out cheaply if the source has zero magnitude (e.g.
                // fully demagnetized via Curie) — no contribution to write.
                if (math.lengthsq(worldMoment) < 1e-12f)
                    return;

                float3 sourcePos = transform.worldTransform.position;
                float  radius    = permanent.influenceRadius;
                if (radius <= 0f)
                    return;

                EMMath.SphereCellRange(sourcePos, radius, origin, cellSize, resolution, out var minCell, out var maxCell);

                float radiusSq = radius * radius;

                for (int z = minCell.z; z <= maxCell.z; z++)
                for (int y = minCell.y; y <= maxCell.y; y++)
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    int3   cell       = new int3(x, y, z);
                    float3 cellCenter = EMMath.CellCenter(cell, origin, cellSize);
                    float3 r          = cellCenter - sourcePos;
                    if (math.lengthsq(r) > radiusSq)
                        continue;

                    float3 contribution = EMMath.DipoleField(worldMoment, sourcePos, cellCenter);
                    int    linear       = EMMath.CellLinearIndex(cell, resolution);
                    B[linear] += contribution;
                }
            }
        }
    }
}
