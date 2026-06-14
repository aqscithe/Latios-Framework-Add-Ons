using Latios;
using Latios.Anna.Electromagnetism.Internal;
using Latios.Transforms;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Latios.Anna.Electromagnetism.Systems
{
    /// <summary>
    /// Phase D: Sample the field grid at every receiver's position, compute the
    /// force `F = ∇(m·B)` and torque `τ = m × B`, convert both to impulses
    /// over the substep dt, and append to the body's <c>AddImpulse</c> buffer.
    ///
    /// Two receiver categories handled in separate parallel jobs:
    ///
    /// <list type="number">
    /// <item><b>Permanent magnet.</b> Any entity carrying
    /// <see cref="PermanentMagnet"/> + <see cref="MagneticDipoleMoment"/> and
    /// Anna's <see cref="AddImpulse"/> buffer. The world moment was set by
    /// <see cref="WriteSourceContributionsSystem"/> earlier in the substep
    /// (Phase B). The job subtracts the magnet's own deposited contribution
    /// from B and ∇B before computing F/τ so the source-also-receiver doesn't
    /// feel its own field. <see cref="PermanentMagnet.influenceRadius"/> is
    /// read to mirror the source-write cutoff exactly.</item>
    ///
    /// <item><b>Ferromagnetic / induced m.</b> Entities carrying
    /// <see cref="Ferromagnet"/>. The induced moment is computed from the
    /// local B per spec §4.4 (`M = χ_m·H = (μ_r−1)·B / (μ_0·μ_r)`, dipole
    /// moment `m = M·volume`), clamped at saturation. The computed moment is
    /// stored on the body's <see cref="MagneticDipoleMoment"/> so debug viz
    /// and later passes (Tier 2 permeability) can read it.</item>
    /// </list>
    /// </summary>
    [DisableAutoCreation]
    [BurstCompile]
    public partial struct ComputeReceiverForcesSystem : ISystem
    {
        LatiosWorldUnmanaged latiosWorld;
        EntityQuery          _permanentReceiverQuery;
        EntityQuery          _ferromagnetReceiverQuery;

        public void OnCreate(ref SystemState state)
        {
            latiosWorld = state.GetLatiosWorldUnmanaged();

            _permanentReceiverQuery = state.Fluent()
                .With<PermanentMagnet>(true)
                .With<MagneticDipoleMoment>(true)
                .With<WorldTransform>(true)
                .With<RigidBody>(true)
                .With<AddImpulse>(false)
                .Without<Ferromagnet>()
                .Without<EMEffectImmuneTag>()
                .Build();

            _ferromagnetReceiverQuery = state.Fluent()
                .With<Ferromagnet>(true)
                .With<MagneticDipoleMoment>(false)
                .With<WorldTransform>(true)
                .With<RigidBody>(true)
                .With<AddImpulse>(false)
                .Without<EMEffectImmuneTag>()
                .Build();
        }

        public void OnUpdate(ref SystemState state)
        {
            if (!latiosWorld.HasElectromagnetismSettings())
                return;

            var settings = latiosWorld.GetElectromagnetismSettings();
            var field    = latiosWorld.sceneBlackboardEntity.GetCollectionComponent<ElectromagneticField>(true);
            if (!field.B.IsCreated || field.B.Length == 0)
                return;

            float dt          = state.WorldUnmanaged.Time.DeltaTime;
            float forceScale  = settings.globalForceScale;

            // Chain order matters: the ferromagnet job writes MagneticDipoleMoment
            // (the induced moment for each ferro receiver), and the permanent
            // job reads MagneticDipoleMoment. Safety system tracks this by
            // ComponentTypeHandle regardless of chunk partitioning, so the two
            // can't run in parallel — schedule permanent first, ferro after.
            var jh = state.Dependency;
            if (!_permanentReceiverQuery.IsEmpty)
            {
                jh = new PermanentReceiverJob
                {
                    B          = field.B,
                    resolution = field.resolution,
                    origin     = field.origin,
                    cellSize   = field.cellSize,
                    dt         = dt,
                    forceScale = forceScale,
                }.ScheduleParallel(_permanentReceiverQuery, jh);
            }

            if (!_ferromagnetReceiverQuery.IsEmpty)
            {
                jh = new FerromagnetReceiverJob
                {
                    B          = field.B,
                    resolution = field.resolution,
                    origin     = field.origin,
                    cellSize   = field.cellSize,
                    dt         = dt,
                    forceScale = forceScale,
                }.ScheduleParallel(_ferromagnetReceiverQuery, jh);
            }

            state.Dependency = jh;
        }

        // ────────────────────────────────────────────────────────────────────
        // Shared force-application logic (inlined so both jobs share the same
        // F/τ formulation and the same impulse-encoding for AddImpulse).
        // ────────────────────────────────────────────────────────────────────

        static void AppendForceAndTorque(float3 moment,
                                         float3 B_sample,
                                         float3x3 gradB,
                                         float dt,
                                         float forceScale,
                                         ref DynamicBuffer<AddImpulse> impulses)
        {
            float3 force   = EMMath.DipoleForce(moment,   gradB);
            float3 torque  = EMMath.DipoleTorque(moment, B_sample);

            float3 linearImpulse  = force  * dt * forceScale;
            float3 angularImpulse = torque * dt * forceScale;

            // Linear: AddImpulse(fieldImpulse) — applied at COM, no torque.
            if (math.lengthsq(linearImpulse) > 1e-20f)
            {
                impulses.Add(new AddImpulse(linearImpulse));
            }

            // Angular: AddImpulse(worldAxis, scalarMagnitude). Sign is in the
            // axis direction; magnitude is non-negative.
            float angularMagnitude = math.length(angularImpulse);
            if (angularMagnitude > 1e-20f)
            {
                float3 axis = angularImpulse / angularMagnitude;
                impulses.Add(new AddImpulse(axis, angularMagnitude));
            }
        }

        // ────────────────────────────────────────────────────────────────────
        // Job 1: Permanent receivers (m already set in MagneticDipoleMoment).
        // ────────────────────────────────────────────────────────────────────

        [BurstCompile]
        partial struct PermanentReceiverJob : IJobEntity
        {
            [ReadOnly] public NativeArray<float3> B;
            public int3   resolution;
            public float3 origin;
            public float  cellSize;
            public float  dt;
            public float  forceScale;

            void Execute(in PermanentMagnet       permanent,
                         in WorldTransform        transform,
                         in MagneticDipoleMoment  moment,
                         ref DynamicBuffer<AddImpulse> impulses)
            {
                if (math.lengthsq(moment.worldMoment) < 1e-12f)
                    return;

                float3   pos    = transform.worldTransform.position;
                float3   Bhere  = EMMath.SampleB(in B, resolution, origin, cellSize, pos);
                float3x3 gradB  = EMMath.GradientB(in B, resolution, origin, cellSize, pos);

                // Subtract this magnet's own contribution to the grid before
                // computing F/τ. Without this, trilinear-sampling asymmetry
                // near the source produces a spurious self-force in isolation.
                // WriteSourceContributionsSystem deposits the same analytical
                // dipole field these helpers reconstruct, so the cancellation
                // is exact for any cell the source actually touched.
                Bhere -= EMMath.SampleSelfDipoleContribution(
                    moment.worldMoment, pos, permanent.influenceRadius,
                    resolution, origin, cellSize, pos);
                gradB -= EMMath.GradientSelfDipoleContribution(
                    moment.worldMoment, pos, permanent.influenceRadius,
                    resolution, origin, cellSize, pos);

                AppendForceAndTorque(moment.worldMoment, Bhere, gradB, dt, forceScale, ref impulses);
            }
        }

        // ────────────────────────────────────────────────────────────────────
        // Job 2: Ferromagnetic receivers (m induced from local field).
        // ────────────────────────────────────────────────────────────────────

        [BurstCompile]
        partial struct FerromagnetReceiverJob : IJobEntity
        {
            [ReadOnly] public NativeArray<float3> B;
            public int3   resolution;
            public float3 origin;
            public float  cellSize;
            public float  dt;
            public float  forceScale;

            void Execute(in Ferromagnet           ferro,
                         in WorldTransform        transform,
                         ref MagneticDipoleMoment moment,
                         ref DynamicBuffer<AddImpulse> impulses)
            {
                float3 pos   = transform.worldTransform.position;
                float3 Bhere = EMMath.SampleB(in B, resolution, origin, cellSize, pos);

                // No field → no induced moment → no force. Saves the gradient
                // work for ferromagnets far from any source.
                float Bmag = math.length(Bhere);
                if (Bmag < 1e-9f)
                {
                    moment.worldMoment = float3.zero;
                    return;
                }

                // Induced magnetization per spec §4.4:
                //   M = χ_m · H,  H = B / (μ_0 · μ_r)
                //   ⇒ M = (μ_r − 1) · B / (μ_0 · μ_r)
                // Then dipole moment m = M · volume. Direction follows B
                // (paramagnetic / soft-ferromagnetic approximation; hysteresis
                // would deviate but is Tier 3+).
                float mur = (float)ferro.relativePermeability;
                float scaleMagnetization = (mur - 1f) / (EMMath.Mu0 * math.max(mur, 1e-3f));
                float3 M = Bhere * scaleMagnetization;

                // Saturation clamp on |M| per spec §4.4.
                float Msat = (float)ferro.saturationMagnetization;
                float Mmag = math.length(M);
                if (Mmag > Msat && Msat > 0f)
                    M = M * (Msat / Mmag);

                float3 inducedMoment = M * ferro.volume;
                moment.worldMoment = inducedMoment;

                float3x3 gradB = EMMath.GradientB(in B, resolution, origin, cellSize, pos);
                AppendForceAndTorque(inducedMoment, Bhere, gradB, dt, forceScale, ref impulses);
            }
        }
    }
}
