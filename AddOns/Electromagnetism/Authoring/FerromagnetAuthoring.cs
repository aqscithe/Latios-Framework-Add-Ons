using Latios.Anna.Authoring;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Latios.Anna.Electromagnetism.Authoring
{
    /// <summary>
    /// Marks a body as ferromagnetic — develops an induced dipole in any
    /// external field and experiences the corresponding force. Requires
    /// <see cref="AnnaRigidBodyAuthoring"/> so the body has the <c>RigidBody</c>
    /// + <c>AddImpulse</c> components needed to receive the resulting impulse.
    ///
    /// Tier 1 uses the simple linear-susceptibility model with saturation
    /// clamp from spec §4.4. Hysteresis is Tier 3+.
    /// </summary>
    [AddComponentMenu("Latios/Anna/Ferromagnet")]
    [RequireComponent(typeof(AnnaRigidBodyAuthoring))]
    public class FerromagnetAuthoring : MonoBehaviour
    {
        [Header("Material")]
        [Tooltip("Relative permeability μ_r. Spec §10 reference: soft iron " +
                 "~200, silicon steel ~7000, neodymium magnet body ~1.05.")]
        [Min(1f)]
        public float relativePermeability = 200f;

        [Tooltip("Saturation magnetization |M|_max in A/m. Caps the induced " +
                 "moment per spec §4.4 — once domain alignment is complete, " +
                 "increasing the external field no longer increases M. Typical " +
                 "iron: ~1.7e6 A/m. Set to 0 to disable saturation clamping.")]
        [Min(0f)]
        public float saturationMagnetization = 1_700_000f;

        [Header("Body")]
        [Tooltip("Approximate volume of the magnetic body in m³. Converts " +
                 "magnetization (A/m) into a dipole moment (A·m²) via m = M·V. " +
                 "Tier 2 may derive this from the body's collider; for now it " +
                 "is designer-authored.")]
        [Min(1e-6f)]
        public float volume = 0.001f;

        void OnDrawGizmosSelected()
        {
            // No directionality to draw — ferromagnets are isotropic in Tier 1.
            // Indicate ferromagnetic presence with a small grey wire sphere.
            Gizmos.color = new Color(0.6f, 0.6f, 0.7f, 0.8f);
            float r = Mathf.Pow(Mathf.Max(volume, 1e-6f), 1f / 3f) * 0.5f;
            Gizmos.DrawWireSphere(transform.position, r);
        }
    }

    public class FerromagnetBaker : Baker<FerromagnetAuthoring>
    {
        public override void Bake(FerromagnetAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(entity, new Ferromagnet
            {
                relativePermeability    = (half)authoring.relativePermeability,
                saturationMagnetization = (half)authoring.saturationMagnetization,
                volume                  = authoring.volume,
            });

            AddComponent(entity, new MagneticDipoleMoment
            {
                worldMoment = float3.zero,
            });
            // Telemetry channel for debug viz arrows.
            AddComponent(entity, default(MagneticFeedback));
        }
    }
}
