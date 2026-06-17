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
    /// Phase B: For every source in the scene, accumulate its analytical field
    /// contribution into the grid over the cells inside the source's
    /// <see cref="InfluenceRadius"/>. Three source kinds today:
    /// <see cref="PermanentMagnet"/> and <see cref="Electromagnet"/> deposit a
    /// dipole field and stamp world-space <see cref="MagneticDipoleMoment"/>
    /// for the receiver pass; <see cref="WireSegment"/> deposits a
    /// closed-form Biot-Savart field and has no dipole moment (wires aren't
    /// receivers in Tier 2).
    ///
    /// Each source kind runs as its own Burst job, single-threaded over
    /// entities, chained on <c>state.Dependency</c> because all three write
    /// the same <c>NativeArray&lt;float3&gt; B</c>. Single-threaded keeps
    /// multiple sources writing to the same cells (two close magnets) free of
    /// atomic-write complexity; with few sources × thousands of cells per
    /// source this is fine. Tier 2+ parallelization options are noted in the
    /// implementation plan §4.
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
        EntityQuery          _wireSegmentQuery;

        public void OnCreate(ref SystemState state)
        {
            latiosWorld  = state.GetLatiosWorldUnmanaged();

            _permanentQuery = state.Fluent()
                .With<PermanentMagnet>(true)
                .With<InfluenceRadius>(true)
                .With<WorldTransform>(true)
                .With<MagneticDipoleMoment>(false)
                .Without<EMEffectImmuneTag>()
                .Without<ElectromagneticBypassTag>()
                .Build();

            _electromagnetQuery = state.Fluent()
                .With<Electromagnet>(true)
                .With<InfluenceRadius>(true)
                .With<WorldTransform>(true)
                .With<MagneticDipoleMoment>(false)
                .Without<EMEffectImmuneTag>()
                .Without<ElectromagneticBypassTag>()
                .Build();

            _wireSegmentQuery = state.Fluent()
                .With<WireSegment>(true)
                .With<InfluenceRadius>(true)
                .With<WorldTransform>(true)
                .Without<EMEffectImmuneTag>()
                .Without<ElectromagneticBypassTag>()
                .Build();
        }

        public void OnUpdate(ref SystemState state)
        {
            if (!latiosWorld.HasElectromagnetismSettings())
                return;
            if (_permanentQuery.IsEmpty && _electromagnetQuery.IsEmpty && _wireSegmentQuery.IsEmpty)
                return;

            var field = latiosWorld.sceneBlackboardEntity.GetCollectionComponent<ElectromagneticField>(false);
            if (!field.B.IsCreated || field.B.Length == 0)
                return;

            // All three source jobs write the same NativeArray<float3> B with
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

            if (!_wireSegmentQuery.IsEmpty)
            {
                jh = new WriteWireSegmentsJob
                {
                    B          = field.B,
                    resolution = field.resolution,
                    origin     = field.origin,
                    cellSize   = field.cellSize,
                }.Schedule(_wireSegmentQuery, jh);
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

        /// <summary>
        /// Wire segments: closed-form Biot-Savart over an AABB(segment, radius)
        /// bounding region, gated per-cell by distance-to-segment ≤ radius.
        /// Endpoints are body-local; the entity's <c>WorldTransform</c> applies
        /// position + rotation + stretch each substep so dynamic wires (mobile
        /// railgun barrels) and static wires (wall-mounted conduits) share one
        /// code path. Wires don't write <c>MagneticDipoleMoment</c> — they
        /// aren't dipoles and aren't receivers.
        /// </summary>
        [BurstCompile]
        partial struct WriteWireSegmentsJob : IJobEntity
        {
            [NativeDisableContainerSafetyRestriction] public NativeArray<float3> B;

            public int3   resolution;
            public float3 origin;
            public float  cellSize;

            void Execute(in WireSegment     wire,
                         in InfluenceRadius influence,
                         in WorldTransform  transform)
            {
                if (math.abs(wire.currentAmps) < 1e-6f)
                    return;
                if (influence.radius <= 0f)
                    return;

                // Body-local → world via QVVS: world = position + rotate(local · stretch).
                var    t          = transform.worldTransform;
                float3 startWorld = t.position + math.mul(t.rotation, wire.startLocal * t.stretch);
                float3 endWorld   = t.position + math.mul(t.rotation, wire.endLocal   * t.stretch);

                float3 L     = endWorld - startWorld;
                float  lenSq = math.lengthsq(L);
                if (lenSq < 1e-12f)
                    return;

                float radius   = influence.radius;
                float radiusSq = radius * radius;

                // Cell AABB enclosing the segment expanded by the influence
                // radius. Conservative — the per-cell distance check below
                // discards corner cells outside the swept-capsule region.
                float3 boundsMin = math.min(startWorld, endWorld) - radius;
                float3 boundsMax = math.max(startWorld, endWorld) + radius;
                int3   minCell   = math.clamp((int3)math.floor((boundsMin - origin) / cellSize), 0, resolution - 1);
                int3   maxCell   = math.clamp((int3)math.floor((boundsMax - origin) / cellSize), 0, resolution - 1);

                for (int z = minCell.z; z <= maxCell.z; z++)
                for (int y = minCell.y; y <= maxCell.y; y++)
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    int3   cell    = new int3(x, y, z);
                    float3 cellPos = EMMath.CellCenter(cell, origin, cellSize);

                    // Squared distance from cell center to the segment.
                    float3 toStart = cellPos - startWorld;
                    float  param   = math.saturate(math.dot(toStart, L) / lenSq);
                    float3 foot    = startWorld + param * L;
                    float3 toFoot  = cellPos - foot;
                    if (math.lengthsq(toFoot) > radiusSq)
                        continue;

                    float3 contribution = EMMath.WireSegmentField(startWorld, endWorld, wire.currentAmps, cellPos);
                    int    linear       = EMMath.CellLinearIndex(cell, resolution);
                    B[linear] += contribution;
                }
            }
        }
    }
}
