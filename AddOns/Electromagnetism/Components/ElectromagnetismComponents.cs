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
        /// Global multiplier applied to magnetic linear forces at the final
        /// impulse-write step. Use this to dial up gameplay impact without
        /// authoring physically-unrealistic dipole moments. 1.0 = realistic.
        /// </summary>
        public float globalForceScale;

        /// <summary>
        /// Global multiplier applied to magnetic torques (τ = m × B) at the
        /// final impulse-write step, decoupled from <see cref="globalForceScale"/>
        /// so translational and rotational response can be tuned independently.
        /// At realistic scale, dipole bodies receive enough torque to align with
        /// the field over many substeps, but the per-substep angular impulses
        /// stack when the force scale is raised for game-feel — separating the
        /// two lets a designer pull translation up without bodies whipping into
        /// uncontrolled spin. 1.0 = use the same scale as force.
        /// </summary>
        public float globalTorqueScale;

        /// <summary>
        /// Per-body ceiling on the linear acceleration magnetic force can
        /// produce per substep, in m/s². 0 (default) disables the clamp.
        /// The dipole field falls as <c>1/r³</c> and its gradient as
        /// <c>1/r⁴</c>; without a clamp, two bodies touching deposit an
        /// unbounded impulse at the moment of contact, Anna's collision flip
        /// converts that overshoot into separation velocity, and the pair
        /// flies apart instead of sticking. Capping acceleration is the
        /// standard regularization for stiff-attractive-force-into-contact
        /// — the body still feels the full pull at meaningful distances but
        /// can't accumulate impulse faster than it can integrate. Skipped for
        /// infinite-mass bodies (<c>inverseMass == 0</c>) and disabled at 0.
        /// Suggested starting value: 50–200 (~5–20g) for typical magnets.
        /// </summary>
        public float maxLinearAcceleration;

        /// <summary>
        /// Number of Jacobi iterations for the Phase C permeability propagation
        /// pass. 0 disables propagation entirely — the source-write output is
        /// what receivers sample (the Tier 1 behaviour). Higher values let
        /// high-μ_r geometry channel flux further from its surfaces, at a
        /// linear cost per iteration. 2–4 is a good range for most scenes;
        /// going higher mostly smooths the source positions without changing
        /// the qualitative concentration around iron.
        /// </summary>
        public int propagationIterations;

        /// <summary>
        /// Diffusion blend per Jacobi iteration in [0,1]. 0 = no diffusion
        /// (B unchanged); 1 = pure Jacobi neighbour-average (no preservation
        /// of pre-iteration cell value). 0.7 default lets each iteration step
        /// roughly one cell of propagation while keeping near-source values
        /// honest.
        /// </summary>
        public float propagationBlend;

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
        /// Scratch buffer used by Phase C propagation (Jacobi iterations
        /// between this and <see cref="B"/>). Length = cellCount. Persistent
        /// — same lifecycle as <see cref="B"/>. Contents are scratch between
        /// substeps; never read outside the propagation system.
        /// </summary>
        public NativeArray<float3> Btemp;

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
            if (Btemp.IsCreated) inputDeps = Btemp.Dispose(inputDeps);
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
    }

    /// <summary>
    /// An electromagnet — a coil whose dipole moment is derived from its current
    /// each substep: <c>m = N · I · A · n̂_world</c>. The current
    /// <see cref="currentAmps"/> is gameplay-writeable at runtime: any system can
    /// just <c>SetComponentData</c> a new value and the next substep's source
    /// pass picks it up. N · A is treated as a constant (coil construction
    /// doesn't change), so designers author "this is what 1 A produces" via
    /// turns × cross-section area, and gameplay dials the strength via current.
    /// </summary>
    public struct Electromagnet : IComponentData
    {
        /// <summary>Current through the coil in amperes. Runtime-writeable.</summary>
        public float currentAmps;

        /// <summary>Number of turns N. Treated as a coil construction constant.</summary>
        public float turns;

        /// <summary>Cross-section area A in m². Coil construction constant.</summary>
        public float crossSectionArea;

        /// <summary>
        /// Coil-normal direction in body-local space (right-hand rule with
        /// positive current). Will be normalized by the source-write pass.
        /// </summary>
        public float3 coilNormalLocal;
    }

    /// <summary>
    /// World-space cutoff radius beyond which a source's contribution to the
    /// grid is skipped. Shared by every source type (permanent magnet,
    /// electromagnet, wire segment) so the source-write pass and the receiver
    /// self-subtraction can use one component regardless of the underlying
    /// source kind. Cell-count cost per source scales as radius³ for point-like
    /// dipoles and as (segmentLength + 2·radius)·radius² for wires.
    /// </summary>
    public struct InfluenceRadius : IComponentData
    {
        public float radius;
    }

    /// <summary>
    /// A straight finite current-carrying wire segment. Field contribution is
    /// computed via closed-form Biot-Savart (no numerical integration) on every
    /// grid cell whose distance to the segment is &lt; <see cref="InfluenceRadius.radius"/>.
    /// Both endpoints are authored in body-local space and rotated/translated
    /// to world each substep via the entity's <c>WorldTransform</c>, so a
    /// dynamic body (e.g. a railgun barrel) can carry a moving wire and a
    /// static body just keeps its endpoints fixed.
    ///
    /// Direction of current is <c>startLocal → endLocal</c> when
    /// <see cref="currentAmps"/> &gt; 0 (right-hand rule for B); negating the
    /// current flips the field. Wires are emit-only — they don't carry a
    /// <c>MagneticDipoleMoment</c> and are skipped by the receiver pass (we
    /// don't model the F = I L × B Lorentz force on the wire itself, spec §5.3).
    /// </summary>
    public struct WireSegment : IComponentData
    {
        /// <summary>Start endpoint in body-local space, meters.</summary>
        public float3 startLocal;

        /// <summary>End endpoint in body-local space, meters.</summary>
        public float3 endLocal;

        /// <summary>
        /// Current through the wire in amperes. Runtime-writeable, same pattern
        /// as <see cref="Electromagnet.currentAmps"/>: gameplay code calls
        /// <c>SetComponentData</c> and the next substep's source pass picks it
        /// up. Sign determines the direction of B via the right-hand rule.
        /// </summary>
        public float currentAmps;
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

    /// <summary>
    /// Full opt-out tag. A source entity (<see cref="PermanentMagnet"/>,
    /// <see cref="Electromagnet"/>, or <see cref="WireSegment"/>) carrying
    /// this tag is skipped by both <c>WriteSourceContributionsSystem</c> AND
    /// <c>ComputeReceiverForcesSystem</c> — for this substep, the entity
    /// neither writes its field contribution into the grid nor samples force
    /// from the field on the receiver side. The source's
    /// <c>MagneticDipoleMoment.worldMoment</c> is left untouched; it resumes
    /// from its last known value as soon as the tag is removed.
    ///
    /// <para>Differs from <see cref="EMEffectImmuneTag"/> in intent —
    /// <c>EMEffectImmuneTag</c> is "permanently outside the EM simulation"
    /// (authoring choice). <c>ElectromagneticBypassTag</c> is "temporarily
    /// driven by some other system" (runtime toggle).</para>
    ///
    /// <para>Intended consumer: gameplay systems that take over a source's
    /// behaviour for the duration of an ability — e.g. a magnet carried by a
    /// grav-gun ability driving its own gameplay-curve force application
    /// directly, with the underlying physics emit and the carrier's own EM
    /// response suspended. Adding the tag at ability start and removing it
    /// on ability end gives a clean hand-off: the source rejoins the
    /// simulation the next substep with no teardown / re-init.</para>
    /// </summary>
    public struct ElectromagneticBypassTag : IComponentData { }

    /// <summary>
    /// Per-receiver telemetry — the last-substep magnetic force and torque
    /// computed by <c>ComputeReceiverForcesSystem</c>, in pre-impulse units
    /// (Newtons / Newton-metres). Populated by every receiver body each
    /// substep so debug visualization can draw force/torque arrows without
    /// re-computing F = ∇(m·B) and τ = m × B on the CPU side.
    ///
    /// Force and torque are stored *before* the per-substep dt multiplier and
    /// the <c>ElectromagnetismSettings.globalForceScale</c> / <c>globalTorqueScale</c>
    /// are applied — so arrow lengths reflect the physical Newtons / Newton-metres
    /// a designer would reason about, not the impulse magnitudes actually fed
    /// to Anna.
    /// </summary>
    public struct MagneticFeedback : IComponentData
    {
        /// <summary>Newtons. <c>F = ∇(m·B)</c> at the body's centre, before the dt × globalForceScale conversion to impulse.</summary>
        public float3 force;

        /// <summary>Newton-metres. <c>τ = m × B</c> at the body's centre, before the dt × globalTorqueScale conversion to angular impulse.</summary>
        public float3 torque;
    }
}
