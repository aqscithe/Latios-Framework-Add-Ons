using Latios;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Latios.Anna.Electromagnetism.Systems
{
    /// <summary>
    /// Phase C: μ_r-weighted Jacobi diffusion of the B field. After the
    /// source-write pass has stamped the analytical dipole / Biot-Savart
    /// contributions, this system runs <c>N</c> iterations of a bilateral
    /// neighbour-average where each pair's weight is the geometric mean of
    /// the two cells' μ_r values. The effect is "flux concentrates along the
    /// high-μ_r path" — channels of iron / steel draw B in from their
    /// vacuum-adjacent neighbours, exactly the way field lines bend into a
    /// ferromagnetic body in textbook diagrams.
    ///
    /// <para>The iteration ping-pongs between <c>field.B</c> and
    /// <c>field.Btemp</c>. After an odd number of iterations the latest
    /// result lives in <c>Btemp</c>; we copy it back to <c>B</c> so
    /// downstream receivers always read the same buffer.</para>
    ///
    /// <para>Skips entirely when <c>settings.propagationIterations &lt;= 0</c>
    /// — the default. Designers turn it on only when their scene has
    /// authored high-μ_r geometry that should bend the field.</para>
    /// </summary>
    [DisableAutoCreation]
    [BurstCompile]
    public partial struct PropagatePermeabilitySystem : ISystem
    {
        LatiosWorldUnmanaged latiosWorld;

        public void OnCreate(ref SystemState state)
        {
            latiosWorld = state.GetLatiosWorldUnmanaged();
        }

        public void OnUpdate(ref SystemState state)
        {
            if (!latiosWorld.HasElectromagnetismSettings())
                return;

            var settings = latiosWorld.GetElectromagnetismSettings();
            int iterations = settings.propagationIterations;
            if (iterations <= 0)
                return;

            var field = latiosWorld.sceneBlackboardEntity
                .GetCollectionComponent<ElectromagneticField>(false);
            if (!field.B.IsCreated || !field.Btemp.IsCreated || !field.muR.IsCreated)
                return;
            if (field.B.Length == 0 || field.B.Length != field.Btemp.Length)
                return;

            float blend = math.saturate(settings.propagationBlend);
            int3  res   = field.resolution;
            int   total = field.B.Length;

            var jh        = state.Dependency;
            var srcIsB    = true;  // tracks which array holds the "current" iteration's input

            for (int i = 0; i < iterations; i++)
            {
                jh = new JacobiIterationJob
                {
                    src   = srcIsB ? field.B     : field.Btemp,
                    dst   = srcIsB ? field.Btemp : field.B,
                    muR   = field.muR,
                    res   = res,
                    blend = blend,
                }.Schedule(total, 256, jh);

                srcIsB = !srcIsB;
            }

            // After N iterations, the result lives in whichever buffer was
            // "src" for the *next* (un-run) iteration. If that's Btemp, copy
            // back so the receiver pass always reads B.
            if (!srcIsB)
            {
                jh = new CopyToBJob
                {
                    src = field.Btemp,
                    dst = field.B,
                }.Schedule(total, 1024, jh);
            }

            state.Dependency = jh;
        }

        /// <summary>
        /// One Jacobi sweep. Per cell: weighted neighbour average where each
        /// face-neighbour's weight is <c>sqrt(μ_r[c] · μ_r[n])</c> — pairs
        /// where both cells are high-μ_r couple most strongly, which is what
        /// drives flux concentration into iron geometry.
        /// </summary>
        [BurstCompile]
        struct JacobiIterationJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> src;
            [NativeDisableParallelForRestriction] public NativeArray<float3> dst;
            [ReadOnly] public NativeArray<half>   muR;

            public int3   res;
            public float  blend;

            public void Execute(int idx)
            {
                int strideY = res.x;
                int strideZ = res.x * res.y;

                int x = idx % res.x;
                int y = (idx / res.x) % res.y;
                int z = idx / strideZ;

                float3 bHere = src[idx];

                // Bail out on grid edges — clamping the neighbour fetch would
                // bias the smoothing toward the centre. Edges retain their
                // pre-iteration value, which is fine for a thick-enough grid.
                if (x == 0 || y == 0 || z == 0 ||
                    x == res.x - 1 || y == res.y - 1 || z == res.z - 1)
                {
                    dst[idx] = bHere;
                    return;
                }

                float mc = math.max(1f, (float)muR[idx]);

                float3 sumB = float3.zero;
                float  sumW = 0f;

                int nx0 = idx - 1;
                int nx1 = idx + 1;
                int ny0 = idx - strideY;
                int ny1 = idx + strideY;
                int nz0 = idx - strideZ;
                int nz1 = idx + strideZ;

                AccumulateNeighbour(mc, src[nx0], muR[nx0], ref sumB, ref sumW);
                AccumulateNeighbour(mc, src[nx1], muR[nx1], ref sumB, ref sumW);
                AccumulateNeighbour(mc, src[ny0], muR[ny0], ref sumB, ref sumW);
                AccumulateNeighbour(mc, src[ny1], muR[ny1], ref sumB, ref sumW);
                AccumulateNeighbour(mc, src[nz0], muR[nz0], ref sumB, ref sumW);
                AccumulateNeighbour(mc, src[nz1], muR[nz1], ref sumB, ref sumW);

                float3 diffused = sumW > 1e-6f ? sumB / sumW : bHere;
                dst[idx] = math.lerp(bHere, diffused, blend);
            }

            static void AccumulateNeighbour(float mc, float3 bN, half mNHalf,
                                            ref float3 sumB, ref float sumW)
            {
                float mn = math.max(1f, (float)mNHalf);
                float w  = math.sqrt(mc * mn);
                sumB += bN * w;
                sumW += w;
            }
        }

        /// <summary>Burst-parallel copy of Btemp → B when the iteration count is odd.</summary>
        [BurstCompile]
        struct CopyToBJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> src;
            [NativeDisableParallelForRestriction] public NativeArray<float3> dst;

            public void Execute(int idx)
            {
                dst[idx] = src[idx];
            }
        }
    }
}
