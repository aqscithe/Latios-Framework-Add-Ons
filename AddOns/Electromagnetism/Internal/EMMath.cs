using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Latios.Anna.Electromagnetism.Internal
{
    /// <summary>
    /// Pure-function helpers for electromagnetism math + grid addressing.
    /// All methods are Burst-friendly and free of allocations / managed refs.
    /// </summary>
    public static class EMMath
    {
        // ────────────────────────────────────────────────────────────────────
        // Physical constants
        // ────────────────────────────────────────────────────────────────────

        /// <summary>Permeability of free space, T·m/A. μ₀ = 4π × 10⁻⁷.</summary>
        public const float Mu0 = 1.25663706143591729e-6f;

        /// <summary>μ₀ / 4π — appears in every dipole / Biot-Savart formula.</summary>
        public const float Mu0Over4Pi = 1e-7f;

        // ────────────────────────────────────────────────────────────────────
        // Magnetic dipole field
        // ────────────────────────────────────────────────────────────────────

        /// <summary>
        /// Magnetic flux density at a field point produced by an ideal dipole.
        /// `B(r) = (μ₀ / 4π) · [(3·(m·r̂)·r̂ − m) / |r|³]`.
        /// Returns zero when the field point is very close to the dipole
        /// (singularity at r = 0).
        /// </summary>
        /// <param name="moment">Dipole moment vector, A·m².</param>
        /// <param name="sourcePos">World position of the dipole center.</param>
        /// <param name="fieldPos">World position of the field point.</param>
        public static float3 DipoleField(float3 moment, float3 sourcePos, float3 fieldPos)
        {
            float3 r = fieldPos - sourcePos;
            float  rSq = math.lengthsq(r);
            // Singularity guard. A more sophisticated treatment would model
            // the body's near-field as the dipole's own volume contribution;
            // for Tier 1 we just zero it.
            if (rSq < 1e-6f)
                return float3.zero;

            float  rLen = math.sqrt(rSq);
            float3 rHat = r / rLen;
            float  mDotRHat = math.dot(moment, rHat);
            return Mu0Over4Pi * (3f * mDotRHat * rHat - moment) / (rSq * rLen);
        }

        // ────────────────────────────────────────────────────────────────────
        // Grid addressing
        // ────────────────────────────────────────────────────────────────────

        /// <summary>
        /// Convert a world position to fractional cell coordinates (no clamp).
        /// </summary>
        public static float3 WorldToCellFloat(float3 worldPos, float3 origin, float cellSize)
        {
            return (worldPos - origin) / cellSize;
        }

        /// <summary>
        /// Convert a world position to integer cell coordinates (no clamp).
        /// Use <see cref="IsCellInBounds"/> to test the result.
        /// </summary>
        public static int3 WorldToCellInt(float3 worldPos, float3 origin, float cellSize)
        {
            return (int3)math.floor((worldPos - origin) / cellSize);
        }

        /// <summary>World position of a cell's center.</summary>
        public static float3 CellCenter(int3 cell, float3 origin, float cellSize)
        {
            return origin + ((float3)cell + 0.5f) * cellSize;
        }

        public static bool IsCellInBounds(int3 cell, int3 resolution)
        {
            return math.all(cell >= 0) && math.all(cell < resolution);
        }

        /// <summary>Flat index for a 3D cell coordinate. No bounds check.</summary>
        public static int CellLinearIndex(int3 cell, int3 resolution)
        {
            return cell.x + cell.y * resolution.x + cell.z * resolution.x * resolution.y;
        }

        // ────────────────────────────────────────────────────────────────────
        // Field sampling
        // ────────────────────────────────────────────────────────────────────

        /// <summary>
        /// Trilinear-interpolated B at a world position. Returns zero if the
        /// position is outside the grid (no extrapolation — receivers outside
        /// the grid simply experience no field).
        /// </summary>
        public static float3 SampleB(in NativeArray<float3> B,
                                     int3                   resolution,
                                     float3                 origin,
                                     float                  cellSize,
                                     float3                 worldPos)
        {
            // Continuous cell coords with cell centers at integer positions.
            // i.e. cell (i,j,k) center is at origin + (i+0.5, j+0.5, k+0.5) * cellSize.
            // Continuous coord at worldPos is therefore (worldPos - origin)/cellSize - 0.5.
            float3 c   = (worldPos - origin) / cellSize - 0.5f;
            int3   c0  = (int3)math.floor(c);
            float3 t   = c - (float3)c0;

            // 8 surrounding cell coordinates, each clamped to grid bounds.
            int3 c000 = math.clamp(c0,                    0, resolution - 1);
            int3 c100 = math.clamp(c0 + new int3(1,0,0),  0, resolution - 1);
            int3 c010 = math.clamp(c0 + new int3(0,1,0),  0, resolution - 1);
            int3 c110 = math.clamp(c0 + new int3(1,1,0),  0, resolution - 1);
            int3 c001 = math.clamp(c0 + new int3(0,0,1),  0, resolution - 1);
            int3 c101 = math.clamp(c0 + new int3(1,0,1),  0, resolution - 1);
            int3 c011 = math.clamp(c0 + new int3(0,1,1),  0, resolution - 1);
            int3 c111 = math.clamp(c0 + new int3(1,1,1),  0, resolution - 1);

            // If the position is significantly outside the grid, return zero.
            // (The clamp above would otherwise extrapolate edge values, which
            // is wrong for an isolated dipole field — outside should be ~0.)
            if (math.any(c0 < -1) || math.any(c0 + 1 > resolution))
                return float3.zero;

            float3 b000 = B[CellLinearIndex(c000, resolution)];
            float3 b100 = B[CellLinearIndex(c100, resolution)];
            float3 b010 = B[CellLinearIndex(c010, resolution)];
            float3 b110 = B[CellLinearIndex(c110, resolution)];
            float3 b001 = B[CellLinearIndex(c001, resolution)];
            float3 b101 = B[CellLinearIndex(c101, resolution)];
            float3 b011 = B[CellLinearIndex(c011, resolution)];
            float3 b111 = B[CellLinearIndex(c111, resolution)];

            // Trilinear lerp.
            float3 b00 = math.lerp(b000, b100, t.x);
            float3 b01 = math.lerp(b001, b101, t.x);
            float3 b10 = math.lerp(b010, b110, t.x);
            float3 b11 = math.lerp(b011, b111, t.x);
            float3 b0  = math.lerp(b00,  b10,  t.y);
            float3 b1  = math.lerp(b01,  b11,  t.y);
            return math.lerp(b0, b1, t.z);
        }

        /// <summary>
        /// Central-difference gradient of B at a world position. Returns a
        /// 3x3 matrix where row i is ∂B/∂xᵢ. Sample stride is half a cell
        /// (smaller risks aliasing the analytical field's high-curvature
        /// regions; larger over-smooths gradients near a magnet).
        /// </summary>
        public static float3x3 GradientB(in NativeArray<float3> B,
                                         int3                   resolution,
                                         float3                 origin,
                                         float                  cellSize,
                                         float3                 worldPos)
        {
            float h = cellSize * 0.5f;
            float invDenominator = 1f / (2f * h);

            float3 bxp = SampleB(in B, resolution, origin, cellSize, worldPos + new float3(h, 0, 0));
            float3 bxm = SampleB(in B, resolution, origin, cellSize, worldPos - new float3(h, 0, 0));
            float3 byp = SampleB(in B, resolution, origin, cellSize, worldPos + new float3(0, h, 0));
            float3 bym = SampleB(in B, resolution, origin, cellSize, worldPos - new float3(0, h, 0));
            float3 bzp = SampleB(in B, resolution, origin, cellSize, worldPos + new float3(0, 0, h));
            float3 bzm = SampleB(in B, resolution, origin, cellSize, worldPos - new float3(0, 0, h));

            // Row i = ∂B/∂xᵢ. float3x3 ctor takes column vectors, so transpose:
            // we want
            //   [ ∂Bx/∂x  ∂Bx/∂y  ∂Bx/∂z ]
            //   [ ∂By/∂x  ∂By/∂y  ∂By/∂z ]
            //   [ ∂Bz/∂x  ∂Bz/∂y  ∂Bz/∂z ]
            // i.e. column j = ∂B/∂xⱼ which is exactly (bjp - bjm) * invDenominator.
            float3 dBdx = (bxp - bxm) * invDenominator;
            float3 dBdy = (byp - bym) * invDenominator;
            float3 dBdz = (bzp - bzm) * invDenominator;
            return new float3x3(dBdx, dBdy, dBdz);
        }

        /// <summary>
        /// Force on a magnetic dipole `F = ∇(m·B)`. Since m is independent of
        /// position, `∂(m·B)/∂xⱼ = m · (∂B/∂xⱼ)`, i.e. each component of F is
        /// m dotted with the corresponding column of ∇B.
        /// </summary>
        public static float3 DipoleForce(float3 moment, float3x3 gradB)
        {
            return new float3(
                math.dot(moment, gradB.c0),
                math.dot(moment, gradB.c1),
                math.dot(moment, gradB.c2));
        }

        /// <summary>Torque on a magnetic dipole `τ = m × B`.</summary>
        public static float3 DipoleTorque(float3 moment, float3 B)
        {
            return math.cross(moment, B);
        }

        // ────────────────────────────────────────────────────────────────────
        // Self-contribution subtraction (for sources that also act as receivers)
        // ────────────────────────────────────────────────────────────────────

        /// <summary>
        /// Trilinear-interpolated B contribution from a single dipole source,
        /// computed analytically at the same 8 cell centers <see cref="SampleB"/>
        /// would touch at <paramref name="samplePos"/>. Subtracting this from
        /// the grid sample removes the source's own contribution — necessary
        /// when a magnet samples a grid that it itself wrote into, since
        /// trilinear-sampling asymmetry near the source produces a small fake
        /// self-force. Honors the same influence-radius cutoff and the same
        /// near-source singularity guard as the source-write pass, so the
        /// subtraction is exactly the contribution that was deposited.
        /// </summary>
        public static float3 SampleSelfDipoleContribution(float3 sourceMoment,
                                                          float3 sourcePos,
                                                          float  sourceRadius,
                                                          int3   resolution,
                                                          float3 origin,
                                                          float  cellSize,
                                                          float3 samplePos)
        {
            float3 c   = (samplePos - origin) / cellSize - 0.5f;
            int3   c0  = (int3)math.floor(c);
            float3 t   = c - (float3)c0;

            if (math.any(c0 < -1) || math.any(c0 + 1 > resolution))
                return float3.zero;

            float radiusSq = sourceRadius * sourceRadius;

            float3 b000 = SelfAt(c0 + new int3(0,0,0), resolution, origin, cellSize, sourceMoment, sourcePos, radiusSq);
            float3 b100 = SelfAt(c0 + new int3(1,0,0), resolution, origin, cellSize, sourceMoment, sourcePos, radiusSq);
            float3 b010 = SelfAt(c0 + new int3(0,1,0), resolution, origin, cellSize, sourceMoment, sourcePos, radiusSq);
            float3 b110 = SelfAt(c0 + new int3(1,1,0), resolution, origin, cellSize, sourceMoment, sourcePos, radiusSq);
            float3 b001 = SelfAt(c0 + new int3(0,0,1), resolution, origin, cellSize, sourceMoment, sourcePos, radiusSq);
            float3 b101 = SelfAt(c0 + new int3(1,0,1), resolution, origin, cellSize, sourceMoment, sourcePos, radiusSq);
            float3 b011 = SelfAt(c0 + new int3(0,1,1), resolution, origin, cellSize, sourceMoment, sourcePos, radiusSq);
            float3 b111 = SelfAt(c0 + new int3(1,1,1), resolution, origin, cellSize, sourceMoment, sourcePos, radiusSq);

            float3 b00 = math.lerp(b000, b100, t.x);
            float3 b01 = math.lerp(b001, b101, t.x);
            float3 b10 = math.lerp(b010, b110, t.x);
            float3 b11 = math.lerp(b011, b111, t.x);
            float3 b0  = math.lerp(b00,  b10,  t.y);
            float3 b1  = math.lerp(b01,  b11,  t.y);
            return math.lerp(b0, b1, t.z);
        }

        static float3 SelfAt(int3   cell,
                             int3   resolution,
                             float3 origin,
                             float  cellSize,
                             float3 sourceMoment,
                             float3 sourcePos,
                             float  radiusSq)
        {
            int3   clamped    = math.clamp(cell, 0, resolution - 1);
            float3 cellCenter = CellCenter(clamped, origin, cellSize);
            if (math.lengthsq(cellCenter - sourcePos) > radiusSq)
                return float3.zero;
            return DipoleField(sourceMoment, sourcePos, cellCenter);
        }

        /// <summary>
        /// Central-difference gradient of a single dipole source's own grid
        /// contribution, mirroring <see cref="GradientB"/>'s stencil. Subtract
        /// from <see cref="GradientB"/> to recover the external-field gradient
        /// at the source's position.
        /// </summary>
        public static float3x3 GradientSelfDipoleContribution(float3 sourceMoment,
                                                              float3 sourcePos,
                                                              float  sourceRadius,
                                                              int3   resolution,
                                                              float3 origin,
                                                              float  cellSize,
                                                              float3 samplePos)
        {
            float h = cellSize * 0.5f;
            float invDenominator = 1f / (2f * h);

            float3 bxp = SampleSelfDipoleContribution(sourceMoment, sourcePos, sourceRadius, resolution, origin, cellSize, samplePos + new float3(h, 0, 0));
            float3 bxm = SampleSelfDipoleContribution(sourceMoment, sourcePos, sourceRadius, resolution, origin, cellSize, samplePos - new float3(h, 0, 0));
            float3 byp = SampleSelfDipoleContribution(sourceMoment, sourcePos, sourceRadius, resolution, origin, cellSize, samplePos + new float3(0, h, 0));
            float3 bym = SampleSelfDipoleContribution(sourceMoment, sourcePos, sourceRadius, resolution, origin, cellSize, samplePos - new float3(0, h, 0));
            float3 bzp = SampleSelfDipoleContribution(sourceMoment, sourcePos, sourceRadius, resolution, origin, cellSize, samplePos + new float3(0, 0, h));
            float3 bzm = SampleSelfDipoleContribution(sourceMoment, sourcePos, sourceRadius, resolution, origin, cellSize, samplePos - new float3(0, 0, h));

            float3 dBdx = (bxp - bxm) * invDenominator;
            float3 dBdy = (byp - bym) * invDenominator;
            float3 dBdz = (bzp - bzm) * invDenominator;
            return new float3x3(dBdx, dBdy, dBdz);
        }

        // ────────────────────────────────────────────────────────────────────
        // Bounding-box cell-range iteration helper
        // ────────────────────────────────────────────────────────────────────

        /// <summary>
        /// Compute the inclusive cell-coordinate range corresponding to a
        /// world-space sphere of the given radius around <paramref name="center"/>,
        /// clamped to the grid. Useful for "iterate every cell in this source's
        /// influence radius" loops.
        /// </summary>
        public static void SphereCellRange(float3 center,
                                           float  radius,
                                           float3 origin,
                                           float  cellSize,
                                           int3   resolution,
                                           out int3 minCell,
                                           out int3 maxCell)
        {
            float3 minWorld = center - radius;
            float3 maxWorld = center + radius;
            minCell = math.clamp((int3)math.floor((minWorld - origin) / cellSize), 0, resolution - 1);
            maxCell = math.clamp((int3)math.floor((maxWorld - origin) / cellSize), 0, resolution - 1);
        }
    }
}
