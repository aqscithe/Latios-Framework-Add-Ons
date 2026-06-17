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
    /// <item><b>Dipole source receiver.</b> Any entity carrying
    /// <see cref="MagneticDipoleMoment"/> + <see cref="InfluenceRadius"/> and
    /// Anna's <see cref="AddImpulse"/> buffer — i.e. anything that wrote a
    /// dipole field into the grid this substep (permanent magnet, electromagnet,
    /// future wire segment). The world moment was set by
    /// <see cref="WriteSourceContributionsSystem"/> earlier in the substep
    /// (Phase B). The job subtracts the body's own deposited contribution from
    /// B and ∇B before computing F/τ so the source-also-receiver doesn't feel
    /// its own field. <see cref="InfluenceRadius.radius"/> mirrors the
    /// source-write cutoff exactly.</item>
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
        EntityQuery          _dipoleSourceReceiverQuery;
        EntityQuery          _ferromagnetReceiverQuery;

        public void OnCreate(ref SystemState state)
        {
            latiosWorld = state.GetLatiosWorldUnmanaged();

            _dipoleSourceReceiverQuery = state.Fluent()
                .With<InfluenceRadius>(true)
                .With<MagneticDipoleMoment>(true)
                .With<WorldTransform>(true)
                .With<RigidBody>(true)
                .With<AddImpulse>(false)
                .With<MagneticFeedback>(false)
                .Without<Ferromagnet>()
                .Without<EMEffectImmuneTag>()
                .Without<ElectromagneticBypassTag>()
                .Build();

            _ferromagnetReceiverQuery = state.Fluent()
                .With<Ferromagnet>(true)
                .With<MagneticDipoleMoment>(false)
                .With<WorldTransform>(true)
                .With<RigidBody>(true)
                .With<AddImpulse>(false)
                .With<MagneticFeedback>(false)
                .Without<EMEffectImmuneTag>()
                .Without<ElectromagneticBypassTag>()
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

            float dt             = state.WorldUnmanaged.Time.DeltaTime;
            float forceScale     = settings.globalForceScale;
            float torqueScale    = settings.globalTorqueScale;
            float maxLinearAccel = settings.maxLinearAcceleration;

            // Chain order matters: the ferromagnet job writes MagneticDipoleMoment
            // (the induced moment for each ferro receiver), and the dipole-source
            // job reads MagneticDipoleMoment. Safety system tracks this by
            // ComponentTypeHandle regardless of chunk partitioning, so the two
            // can't run in parallel — schedule the source-receiver pass first,
            // ferro after.
            var jh = state.Dependency;
            if (!_dipoleSourceReceiverQuery.IsEmpty)
            {
                jh = new DipoleSourceReceiverJob
                {
                    B              = field.B,
                    resolution     = field.resolution,
                    origin         = field.origin,
                    cellSize       = field.cellSize,
                    dt             = dt,
                    forceScale     = forceScale,
                    torqueScale    = torqueScale,
                    maxLinearAccel = maxLinearAccel,
                }.ScheduleParallel(_dipoleSourceReceiverQuery, jh);
            }

            if (!_ferromagnetReceiverQuery.IsEmpty)
            {
                jh = new FerromagnetReceiverJob
                {
                    B              = field.B,
                    resolution     = field.resolution,
                    origin         = field.origin,
                    cellSize       = field.cellSize,
                    dt             = dt,
                    forceScale     = forceScale,
                    torqueScale    = torqueScale,
                    maxLinearAccel = maxLinearAccel,
                }.ScheduleParallel(_ferromagnetReceiverQuery, jh);
            }

            state.Dependency = jh;
        }

        // ────────────────────────────────────────────────────────────────────
        // Shared impulse-encoding helper. Callers compute the pre-impulse F
        // and τ themselves (and write them to MagneticFeedback for debug viz)
        // before invoking this — keeping the impulse encoding here means both
        // receiver jobs share one source of truth for the AddImpulse encoding
        // convention.
        // ────────────────────────────────────────────────────────────────────

        static void AppendImpulses(float3 force, float3 torque,
                                   float dt, float forceScale, float torqueScale,
                                   float inverseMass, float maxLinearAccel,
                                   ref DynamicBuffer<AddImpulse> impulses)
        {
            float3 scaledForce = force * forceScale;

            // Acceleration ceiling regularizes the 1/r⁴ gradient singularity
            // near contact. Without this, the final substep before two bodies
            // touch deposits an unbounded impulse, Anna's collision flip turns
            // the overshoot into separation velocity, and the pair flies apart
            // instead of sticking. Skip for infinite-mass bodies (the clamp
            // would be meaningless) and when disabled (maxLinearAccel <= 0).
            if (maxLinearAccel > 0f && inverseMass > 0f)
            {
                float accelMag = math.length(scaledForce) * inverseMass;
                if (accelMag > maxLinearAccel)
                    scaledForce *= maxLinearAccel / accelMag;
            }

            float3 linearImpulse  = scaledForce * dt;
            float3 angularImpulse = torque * dt * torqueScale;

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
        // Job 1: Dipole-source receivers (permanent magnets, electromagnets,
        // anything else that wrote a dipole field into the grid this substep).
        // m was set on MagneticDipoleMoment by WriteSourceContributionsSystem.
        // ────────────────────────────────────────────────────────────────────

        [BurstCompile]
        partial struct DipoleSourceReceiverJob : IJobEntity
        {
            [ReadOnly] public NativeArray<float3> B;
            public int3   resolution;
            public float3 origin;
            public float  cellSize;
            public float  dt;
            public float  forceScale;
            public float  torqueScale;
            public float  maxLinearAccel;

            void Execute(in InfluenceRadius       influence,
                         in WorldTransform        transform,
                         in MagneticDipoleMoment  moment,
                         in RigidBody             body,
                         ref MagneticFeedback     feedback,
                         ref DynamicBuffer<AddImpulse> impulses)
            {
                if (math.lengthsq(moment.worldMoment) < 1e-12f)
                {
                    feedback.force  = float3.zero;
                    feedback.torque = float3.zero;
                    return;
                }

                float3   pos    = transform.worldTransform.position;
                float3   Bhere  = EMMath.SampleB(in B, resolution, origin, cellSize, pos);
                float3x3 gradB  = EMMath.GradientB(in B, resolution, origin, cellSize, pos);

                // Subtract this body's own contribution to the grid before
                // computing F/τ. Without this, trilinear-sampling asymmetry
                // near the source produces a spurious self-force in isolation.
                // WriteSourceContributionsSystem deposits the same analytical
                // dipole field these helpers reconstruct, so the cancellation
                // is exact for any cell the source actually touched.
                Bhere -= EMMath.SampleSelfDipoleContribution(
                    moment.worldMoment, pos, influence.radius,
                    resolution, origin, cellSize, pos);
                gradB -= EMMath.GradientSelfDipoleContribution(
                    moment.worldMoment, pos, influence.radius,
                    resolution, origin, cellSize, pos);

                float3 force  = EMMath.DipoleForce(moment.worldMoment,  gradB);
                float3 torque = EMMath.DipoleTorque(moment.worldMoment, Bhere);
                feedback.force  = force;
                feedback.torque = torque;

                AppendImpulses(force, torque, dt, forceScale, torqueScale,
                               body.inverseMass, maxLinearAccel, ref impulses);
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
            public float  torqueScale;
            public float  maxLinearAccel;

            void Execute(in Ferromagnet           ferro,
                         in WorldTransform        transform,
                         in RigidBody             body,
                         ref MagneticDipoleMoment moment,
                         ref MagneticFeedback     feedback,
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
                    feedback.force     = float3.zero;
                    feedback.torque    = float3.zero;
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

                float3 force  = EMMath.DipoleForce(inducedMoment,  gradB);
                float3 torque = EMMath.DipoleTorque(inducedMoment, Bhere);
                feedback.force  = force;
                feedback.torque = torque;

                AppendImpulses(force, torque, dt, forceScale, torqueScale,
                               body.inverseMass, maxLinearAccel, ref impulses);
            }
        }
    }
}
