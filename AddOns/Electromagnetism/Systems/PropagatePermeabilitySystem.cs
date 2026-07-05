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
    ///
    /// <para><b>Current status:</b> the system is <i>commented out</i> in
    /// <see cref="ElectromagnetismSuperSystem"/>. Gameplay magnetism is served
    /// entirely by the analytical source stamp in
    /// <see cref="WriteSourceContributionsSystem"/> (grav-gun-carried magnets,
    /// authored magnetized objects, ferrous receivers). Re-enable propagation
    /// only when a scene requires the flux-channeling effect of high-μ_r
    /// geometry.</para>
    ///
    /// <para><b>Performance notes — read before re-enabling.</b> When active
    /// this pass is the single most expensive system in the frame (measured
    /// ~9.7 ms wall-clock per iteration across 14 threads on a 128³ grid,
    /// dropping the app from ~470 fps to ~60 fps at
    /// <c>propagationIterations=1</c>; linear with iteration count). The
    /// bottleneck is DRAM bandwidth, not ALU — 14 Burst threads saturate the
    /// memory controller doing a 7-neighbour stencil over an entire float3
    /// grid, and the Z-axis stride (<c>res.x·res.y·12</c> bytes ≈ 196 KB at
    /// 128³) blows past L1/L2 on every hop. Ordered wins:</para>
    ///
    /// <list type="number">
    ///   <item><description>Lower <c>propagationIterations</c>. Cost is
    ///     linear. Jacobi converges slowly and the last couple of iterations
    ///     are cosmetic. Two is usually enough for visible bending.</description></item>
    ///   <item><description>Precompute <c>sqrt(μ_r)</c> into a companion
    ///     <c>NativeArray&lt;float&gt;</c> at the same time <c>muR</c> is
    ///     populated. Because <c>sqrt(mc·mn) == sqrt(mc)·sqrt(mn)</c>, the
    ///     inner loop's 6 <c>math.sqrt</c> calls collapse to plain
    ///     multiplies, and the neighbour fetch becomes a <c>float</c> read
    ///     instead of <c>half</c>+cast. Same physics, ~15–20% cheaper.</description></item>
    ///   <item><description>Force an even iteration count and delete
    ///     <see cref="CopyToBJob"/>. Track "which buffer holds the current
    ///     result" on <see cref="Latios.Anna.Electromagnetism.ElectromagneticField"/>
    ///     and have <see cref="ComputeReceiverForcesSystem"/> read whichever
    ///     buffer is current instead of always <c>B</c>. Saves ~1.5 ms
    ///     wall-clock per frame unconditionally.</description></item>
    ///   <item><description>Halve field grid resolution. 8× less work and
    ///     8× less bandwidth per iteration. The field is inherently
    ///     low-frequency after diffusion, so a half-res grid is often
    ///     visually indistinguishable; upsample at sample-time in
    ///     <see cref="ComputeReceiverForcesSystem"/> if artifacts show.</description></item>
    ///   <item><description>Early-out uniform-μ_r cells. If <c>muR[idx]</c>
    ///     and all 6 neighbours are ≈ 1 (bit-compare the <c>half</c>s), the
    ///     weighted average degenerates to the unweighted one, and if you
    ///     accept "vacuum cells don't diffuse" you can skip the write
    ///     entirely. Big win in scenes that are mostly empty space around a
    ///     small chunk of iron geometry.</description></item>
    ///   <item><description>Cache-block the traversal. <c>IJobParallelFor</c>
    ///     hands out linear ranges, which is the worst case for the Z
    ///     neighbour stride. A hand-batched job walking 32³ (or 16³) tiles
    ///     keeps the neighbour fetches inside L2. Real engineering effort —
    ///     try 1–5 first.</description></item>
    /// </list>
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
