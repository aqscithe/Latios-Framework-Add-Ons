using Latios.Anna.Authoring;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Latios.Anna.Electromagnetism.Authoring
{
    /// <summary>
    /// Marks a body as an electromagnet — a coil whose dipole moment is derived
    /// each substep from <c>m = N · I · A · n̂_world</c>.
    ///
    /// Pair with <see cref="AnnaRigidBodyAuthoring"/> for a *dynamic* coil
    /// (handheld magnet, magnetic crane arm, anything that needs to feel
    /// reaction force from other sources). Omit it for a *static* coil
    /// (wall-mounted holding magnet, door slam-shut electromagnet, conveyor
    /// pad) — the field-emit path doesn't need a rigid body, only the
    /// reaction-force path does. Static coils still drive every other
    /// receiver in the scene normally.
    ///
    /// Designers author the coil geometry (turns N + cross-section area A) and
    /// the body-local coil direction; gameplay scripts dial up/down the strength
    /// at runtime by writing <c>currentAmps</c> via
    /// <c>SystemAPI.GetComponent</c> / <c>SetComponentData</c>. Setting current
    /// to zero produces no field (the source-write pass early-outs on zero
    /// moment, same as a fully demagnetized permanent magnet).
    /// </summary>
    [AddComponentMenu("Latios/Anna/Electromagnet")]
    public class ElectromagnetAuthoring : MonoBehaviour
    {
        [Header("Coil geometry")]
        [Tooltip("Coil-normal direction in body-local space (right-hand rule " +
                 "with positive current). Will be normalized at bake time.")]
        public Vector3 coilNormalLocal = new Vector3(0f, 1f, 0f);

        [Tooltip("Number of turns N. Coil construction constant — gameplay " +
                 "typically doesn't change this at runtime.")]
        [Min(1f)]
        public float turns = 100f;

        [Tooltip("Coil cross-section area A in m². For a 5 cm radius solenoid " +
                 "this is ~0.0079 m². Combined with turns and current to " +
                 "produce the dipole moment.")]
        [Min(1e-6f)]
        public float crossSectionArea = 0.001f;

        [Header("Drive")]
        [Tooltip("Initial current in amperes at scene load. Gameplay code can " +
                 "rewrite this at runtime via SetComponentData<Electromagnet>. " +
                 "Set to zero for an off-by-default coil.")]
        public float initialCurrentAmps = 1f;

        [Header("Influence")]
        [Tooltip("World-space radius beyond which this coil's contribution to " +
                 "the field grid is skipped. Cell-count cost per source scales " +
                 "as radius³, so keep this tight. Sized to the maximum |m| the " +
                 "coil can produce at full current — leaving headroom is fine " +
                 "(outside cells contribute nothing).")]
        [Min(0.05f)]
        public float influenceRadius = 1.5f;

        void OnDrawGizmosSelected()
        {
            // Visualize the coil normal (north → red arrow tip; south → blue),
            // length scaled by the current-driven dipole magnitude so designers
            // get a sense of relative strength vs. permanent magnets.
            Vector3 worldNormal = transform.TransformDirection(coilNormalLocal.normalized);
            Vector3 origin      = transform.position;
            float   m_mag       = Mathf.Abs(turns * initialCurrentAmps * crossSectionArea);
            float   length      = Mathf.Max(0.1f, Mathf.Pow(Mathf.Max(m_mag, 1e-6f), 1f / 3f) * 0.2f);

            // Sign of current flips the displayed pole direction.
            float poleSign = Mathf.Sign(initialCurrentAmps);
            if (poleSign == 0f) poleSign = 1f;

            Gizmos.color = Color.red;
            Gizmos.DrawLine(origin, origin + worldNormal * length * poleSign);
            Gizmos.DrawWireSphere(origin + worldNormal * length * poleSign, length * 0.1f);

            Gizmos.color = Color.blue;
            Gizmos.DrawLine(origin, origin - worldNormal * length * poleSign);

            Gizmos.color = new Color(0.4f, 0.7f, 1f, 0.3f);
            Gizmos.DrawWireSphere(origin, influenceRadius);
        }
    }

    public class ElectromagnetBaker : Baker<ElectromagnetAuthoring>
    {
        public override void Bake(ElectromagnetAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            Vector3 axis = authoring.coilNormalLocal;
            if (axis.sqrMagnitude < 1e-6f)
                axis = Vector3.up;
            float3 normal = math.normalize(new float3(axis.x, axis.y, axis.z));

            AddComponent(entity, new Electromagnet
            {
                currentAmps      = authoring.initialCurrentAmps,
                turns            = authoring.turns,
                crossSectionArea = authoring.crossSectionArea,
                coilNormalLocal  = normal,
            });
            AddComponent(entity, new InfluenceRadius
            {
                radius = authoring.influenceRadius,
            });

            // Electromagnets are also receivers (they feel external fields)
            // and downstream debug viz expects every magnetic body to carry
            // MagneticDipoleMoment so the world moment is queryable.
            AddComponent(entity, new MagneticDipoleMoment
            {
                worldMoment = float3.zero,
            });
            // Telemetry channel for debug viz arrows.
            AddComponent(entity, default(MagneticFeedback));
        }
    }
}
