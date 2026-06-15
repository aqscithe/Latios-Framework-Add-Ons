using Latios;
using Latios.Anna.Systems;
using Unity.Burst;
using Unity.Entities;

namespace Latios.Anna.Electromagnetism.Systems
{
    /// <summary>
    /// Orchestrates the per-substep electromagnetism update.
    ///
    /// Runs before <see cref="AnnaSuperSystem"/> so that <c>AddImpulse</c>
    /// entries written here land in the same-frame buffer that
    /// <c>CollectRigidBodiesSystem</c> consumes immediately after.
    ///
    /// Phase order:
    /// 1. <see cref="FieldGridLifecycleSystem"/> — ensure the grid exists at the
    ///    settings-specified resolution.
    /// 2. <see cref="ClearFieldGridSystem"/> — zero B for the new substep.
    /// 3. <see cref="WriteSourceContributionsSystem"/> — accumulate dipole
    ///    contributions from every <see cref="PermanentMagnet"/>; also writes
    ///    <see cref="MagneticDipoleMoment"/> on those bodies.
    /// 4. <see cref="PropagatePermeabilitySystem"/> — μ_r-weighted Jacobi
    ///    diffusion to channel B through high-μ_r geometry (skipped when
    ///    propagationIterations == 0).
    /// 5. <see cref="ComputeReceiverForcesSystem"/> — sample grid for every
    ///    <see cref="Ferromagnet"/> and every body that carries
    ///    <see cref="MagneticDipoleMoment"/>; compute F = ∇(m·B), τ = m × B;
    ///    convert to impulses; append to <c>AddImpulse</c>.
    ///
    /// Tier 3 will slot Phase E (Faraday induction) before D.
    /// </summary>
    [DisableAutoCreation]
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateBefore(typeof(AnnaSuperSystem))]
    public partial class ElectromagnetismSuperSystem : RootSuperSystem
    {
        protected override void CreateSystems()
        {
            EnableSystemSorting = false;

            GetOrCreateAndAddUnmanagedSystem<FieldGridLifecycleSystem>();
            GetOrCreateAndAddUnmanagedSystem<ClearFieldGridSystem>();
            GetOrCreateAndAddUnmanagedSystem<WriteSourceContributionsSystem>();
            GetOrCreateAndAddUnmanagedSystem<PropagatePermeabilitySystem>();
            GetOrCreateAndAddUnmanagedSystem<ComputeReceiverForcesSystem>();
        }
    }
}
