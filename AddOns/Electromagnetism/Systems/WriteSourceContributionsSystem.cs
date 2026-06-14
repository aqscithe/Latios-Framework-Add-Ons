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
    /// Phase B: For every dipole source (permanent magnet or electromagnet),
    /// compute its world-space dipole moment, store it on
    /// <see cref="MagneticDipoleMoment"/>, then accumulate its analytical
    /// dipole field into every grid cell within the source's
    /// <see cref="InfluenceRadius"/>.
    ///
    /// Tier 1 runs single-threaded over sources because multiple sources may
    /// write to the same cells (e.g., two close magnets) and we want to avoid
    /// the atomic-write complexity. With ~few sources and ~thousands of cells
    /// per source this is fine. Tier 2 parallelizes either over sources (with
    /// atomic accumulation) or over cells (each cell sums contributions from
    /// all in-range sources, no atomics needed). Two source kinds are handled
    /// by two jobs chained on <c>state.Dependency</c>; they can't run in
    /// parallel because they both write the same <c>NativeArray&lt;float3&gt; B</c>.
    ///
    /// Self-contribution note: a source writes its own field to nearby cells,
    /// which means a magnet sampling the grid at its own position will see its
    /// own contribution. The matching subtraction lives in
    /// <c>ComputeReceiverForcesSystem.DipoleSourceReceiverJob</c>, which uses
    /// <see cref="EMMath.SampleSelfDipoleContribution"/> /
    /// <see cref="EMMath.GradientSelfDipoleContribution"/> to remove exactly
    /// what was deposited here before computing F/τ.
    /// </summary>
    [DisableAutoCreation]
    [BurstCompile]
    public partial struct WriteSourceContributionsSystem : ISystem
    {
        LatiosWorldUnmanaged latiosWorld;
        EntityQuery          _permanentQuery;
        EntityQuery          _electromagnetQuery;

        public void OnCreate(ref SystemState state)
        {
            latiosWorld  = state.GetLatiosWorldUnmanaged();

            _permanentQuery = state.Fluent()
                .With<PermanentMagnet>(true)
                .With<InfluenceRadius>(true)
                .With<WorldTransform>(true)
                .With<MagneticDipoleMoment>(false)
                .Without<EMEffectImmuneTag>()
                .Build();

            _electromagnetQuery = state.Fluent()
                .With<Electromagnet>(true)
                .With<InfluenceRadius>(true)
                .With<WorldTransform>(true)
                .With<MagneticDipoleMoment>(false)
                .Without<EMEffectImmuneTag>()
                .Build();
        }

        public void OnUpdate(ref SystemState state)
        {
            if (!latiosWorld.HasElectromagnetismSettings())
                return;
            if (_permanentQuery.IsEmpty && _electromagnetQuery.IsEmpty)
                return;

            var field = latiosWorld.sceneBlackboardEntity.GetCollectionComponent<ElectromagneticField>(false);
            if (!field.B.IsCreated || field.B.Length == 0)
                return;

            // Both jobs write the same NativeArray<float3> B with
            // [NativeDisableContainerSafetyRestriction], so they can't run in
            // parallel — chain on state.Dependency.
            var jh = state.Dependency;

            if (!_permanentQuery.IsEmpty)
            {
                jh = new WritePermanentMagnetsJob
                {
                    B          = field.B,
                    resolution = field.resolution,
                    origin     = field.origin,
                    cellSize   = field.cellSize,
                }.Schedule(_permanentQuery, jh);
            }

            if (!_electromagnetQuery.IsEmpty)
            {
                jh = new WriteElectromagnetsJob
                {
                    B          = field.B,
                    resolution = field.resolution,
                    origin     = field.origin,
                    cellSize   = field.cellSize,
                }.Schedule(_electromagnetQuery, jh);
            }

            state.Dependency = jh;
        }

        // ────────────────────────────────────────────────────────────────────
        // Shared write helper: given a world-space dipole moment, source
        // position, and influence radius, accumulate the analytical dipole
        // field into every grid cell inside the influence sphere.
        // ────────────────────────────────────────────────────────────────────

        static void AccumulateDipoleIntoGrid(float3              worldMoment,
                                             float3              sourcePos,
                                             float               radius,
                                             int3                resolution,
                                             float3              origin,
                                             float               cellSize,
                                             NativeArray<float3> B)
        {
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

        /// <summary>
        /// Permanent magnets: world moment = rotated body-local moment × Curie scale.
        /// </summary>
        [BurstCompile]
        partial struct WritePermanentMagnetsJob : IJobEntity
        {
            [NativeDisableContainerSafetyRestriction] public NativeArray<float3> B;

            public int3   resolution;
            public float3 origin;
            public float  cellSize;

            void Execute(in PermanentMagnet permanent,
                         in InfluenceRadius influence,
                         in WorldTransform  transform,
                         ref MagneticDipoleMoment moment)
            {
                var rotation = transform.worldTransform.rotation;
                float3 worldMoment = math.mul(rotation, permanent.localMoment) * (float)permanent.curieScale;
                moment.worldMoment = worldMoment;

                if (math.lengthsq(worldMoment) < 1e-12f)
                    return;
                if (influence.radius <= 0f)
                    return;

                float3 sourcePos = transform.worldTransform.position;
                AccumulateDipoleIntoGrid(worldMoment, sourcePos, influence.radius,
                                         resolution, origin, cellSize, B);
            }
        }

        /// <summary>
        /// Electromagnets: world moment = rotated coil normal × (N · I · A).
        /// </summary>
        [BurstCompile]
        partial struct WriteElectromagnetsJob : IJobEntity
        {
            [NativeDisableContainerSafetyRestriction] public NativeArray<float3> B;

            public int3   resolution;
            public float3 origin;
            public float  cellSize;

            void Execute(in Electromagnet   coil,
                         in InfluenceRadius influence,
                         in WorldTransform  transform,
                         ref MagneticDipoleMoment moment)
            {
                var rotation = transform.worldTransform.rotation;
                float3 normalWorld = math.mul(rotation, coil.coilNormalLocal);
                // m = N · I · A · n̂. Sign of currentAmps flips the pole.
                float  magnitude   = coil.turns * coil.currentAmps * coil.crossSectionArea;
                float3 worldMoment = normalWorld * magnitude;
                moment.worldMoment = worldMoment;

                if (math.lengthsq(worldMoment) < 1e-12f)
                    return;
                if (influence.radius <= 0f)
                    return;

                float3 sourcePos = transform.worldTransform.position;
                AccumulateDipoleIntoGrid(worldMoment, sourcePos, influence.radius,
                                         resolution, origin, cellSize, B);
            }
        }
    }
}
