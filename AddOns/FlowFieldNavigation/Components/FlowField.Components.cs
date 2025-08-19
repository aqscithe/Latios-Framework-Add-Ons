using Unity.Entities;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    public static partial class FlowField
    {
        public struct Goal : IComponentData
        {
            public int FootprintSize;
        }

        public struct AgentDirection : IComponentData
        {
            public float2 Value;
        }

        public struct AgentFootprint : IComponentData
        {
            public int Size;
        }

        public struct PrevPosition : IComponentData
        {
            public float2 Value;
        }

        public struct Velocity : IComponentData
        {
            public float2 Value;
        }
    }
}