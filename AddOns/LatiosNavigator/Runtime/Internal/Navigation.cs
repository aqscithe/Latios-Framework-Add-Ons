using System;
using Unity.Mathematics;

namespace Latios.Navigator.Internal
{
    internal struct Triangle
    {
        public int VertexA;
        public int VertexB;
        public int VertexC;
    }

    internal struct Edge : IEquatable<Edge>
    {
        public int VertexA;
        public int VertexB;

        public bool Equals(Edge other) => (VertexA == other.VertexA && VertexB == other.VertexB) ||
                                          (VertexA == other.VertexB && VertexB == other.VertexA);

        public override bool Equals(object obj) => obj is Edge other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                var min = math.min(VertexA, VertexB);
                var max = math.max(VertexA, VertexB);
                return (min * 397) ^ max;
            }
        }
    }
}