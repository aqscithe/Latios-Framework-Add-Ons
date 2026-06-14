using Latios;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Latios.Anna.Electromagnetism
{
    // =========================================================================
    // Settings (lives on sceneBlackboardEntity, IComponentData)
    // =========================================================================

    /// <summary>
    /// Per-scene electromagnetism settings. Must exist on the sceneBlackboardEntity
    /// for any EM system to run. Baked from <see cref="Authoring.ElectromagnetismSettingsAuthoring"/>.
    /// </summary>
    public struct ElectromagnetismSettings : IComponentData
    {
        /// <summary>World position of the grid's (0,0,0) corner.</summary>
        public float3 gridOrigin;

        /// <summary>Cell counts along x, y, z.</summary>
        public int3 gridResolution;

        /// <summary>Meters per cell. Spec default 0.1m, but Tier 1 allows tuning.</summary>
        public float cellSize;

        /// <summary>
        /// Global multiplier applied to all magnetic forces and torques at the
        /// final-write step. Use this to dial up gameplay impact without
        /// authoring physically-unrealistic dipole moments. 1.0 = realistic.
        /// </summary>
        public float globalForceScale;

        public float3 GridSizeMeters => (float3)gridResolution * cellSize;
        public int    CellCount      => gridResolution.x * gridResolution.y * gridResolution.z;
    }

    // =========================================================================
    // Field grid (ICollectionComponent on sceneBlackboardEntity)
    // =========================================================================

    /// <summary>
    /// The voxel field grid. Cell-centered B (Tier 1 deviation from canonical
    /// Yee; documented in implementation plan §1). Tier 2/3 will add E, dirty
    /// flag, and migrate to staggered Yee storage when induction lands.
    ///
    /// Allocated with <see cref="Allocator.Persistent"/> in
    /// <see cref="Systems.FieldGridLifecycleSystem"/> and disposed when the
    /// collection component is replaced (scene transition).
    /// </summary>
    public partial struct ElectromagneticField : ICollectionComponent
    {
        /// <summary>Magnetic flux density per cell, Tesla. Length = cellCount.</summary>
        public NativeArray<float3> B;

        /// <summary>
        /// Relative permeability per cell. Length = cellCount. Tier 2+: ferromagnetic
        /// geometry stamps μ_r &gt; 1 into the cells they occupy, which Phase C
        /// propagates as flux concentration. Tier 1 leaves this as all 1.0
        /// (vacuum).
        /// </summary>
        public NativeArray<half> muR;

        /// <summary>
        /// Conductivity per cell, S/m. Length = cellCount. Used by Tier 3
        /// Faraday induction to drive `J = σE` in conductive voxels. Tier 1
        /// leaves this as all 0.
        /// </summary>
        public NativeArray<half> sigma;

        /// <summary>Cell counts along x, y, z.</summary>
        public int3 resolution;

        /// <summary>World position of the grid's (0,0,0) corner.</summary>
        public float3 origin;

        /// <summary>Meters per cell.</summary>
        public float cellSize;

        public JobHandle TryDispose(JobHandle inputDeps)
        {
            if (B.IsCreated)     inputDeps = B.Dispose(inputDeps);
            if (muR.IsCreated)   inputDeps = muR.Dispose(inputDeps);
            if (sigma.IsCreated) inputDeps = sigma.Dispose(inputDeps);
            return inputDeps;
        }
    }

    // =========================================================================
    // Sources (write to the grid)
    // =========================================================================

    /// <summary>
    /// A permanent magnet — a fixed dipole moment in the body-local frame that
    /// is constant unless heated above the Curie temperature. Combined with the
    /// entity's WorldTransform rotation each frame to produce the world-space
    /// moment stored in <see cref="MagneticDipoleMoment"/>.
    /// </summary>
    public struct PermanentMagnet : IComponentData
    {
        /// <summary>
        /// Dipole moment vector in body-local space, units A·m². Direction =
        /// north-pole face direction; magnitude = strength.
        /// </summary>
        public float3 localMoment;

        /// <summary>
        /// Curie demagnetization scale, 1.0 = full strength. Tier 2 will drive
        /// this from a thermal state.
        /// </summary>
        public half curieScale;

        /// <summary>
        /// World-space radius beyond which this source's contribution is
        /// considered negligible and is not written to the grid. Tier 1: set
        /// in authoring. Tier 2 will auto-compute from |m| and a |B| threshold.
        /// </summary>
        public float influenceRadius;
    }

    // =========================================================================
    // Receivers (read from the grid)
    // =========================================================================

    /// <summary>
    /// A ferromagnetic body. Develops an induced dipole moment proportional
    /// to the local field intensity H, then experiences `F = ∇(m·B)` and
    /// `τ = m × B`. Tier 1 uses the linear susceptibility model
    /// `M = χ_m · H` with saturation clamp; full hysteresis is Tier 3.
    /// </summary>
    public struct Ferromagnet : IComponentData
    {
        /// <summary>
        /// Relative permeability. Iron ~200, silicon steel ~7000, neodymium
        /// magnet body ~1.05. Spec §4.4 / §10.
        /// </summary>
        public half relativePermeability;

        /// <summary>
        /// Saturation magnetization |M|_max in A/m. Caps domain alignment per
        /// spec §4.4. A few × 10⁵ is typical for iron.
        /// </summary>
        public half saturationMagnetization;

        /// <summary>
        /// Body volume in m³. Used to convert magnetization M (A/m) into a
        /// dipole moment (A·m²). For Tier 1 the designer authors this
        /// approximately; Tier 2 can derive from the body's collider.
        /// </summary>
        public float volume;
    }

    // =========================================================================
    // Per-body state (updated each substep)
    // =========================================================================

    /// <summary>
    /// The body's current effective magnetic dipole moment in world space.
    /// Updated each substep by <see cref="Systems.WriteSourceContributionsSystem"/>
    /// for permanents and by <see cref="Systems.ComputeReceiverForcesSystem"/>
    /// for ferromagnets (induced m depends on the field they're standing in).
    /// </summary>
    public struct MagneticDipoleMoment : IComponentData
    {
        /// <summary>World-space dipole moment, units A·m².</summary>
        public float3 worldMoment;
    }

    /// <summary>
    /// Opt-out tag. A body carrying this tag is invisible to the EM systems:
    /// it does not write to or read from the field grid. Mirrors the pattern
    /// of <c>GravityEffectImmuneTag</c> from the project's core components.
    /// </summary>
    public struct EMEffectImmuneTag : IComponentData { }
}
