using System;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    [Serializable]
    public struct FieldSettings
    {
        public int2 FieldSize;
        public float2 CellSize;
        public float2 Pivot;

        public static FieldSettings Default => new()
        {
            FieldSize = 100, CellSize = 1, Pivot = 0.5f
        };
    }
}