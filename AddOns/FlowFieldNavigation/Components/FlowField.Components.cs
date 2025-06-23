using Unity.Entities;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    public static partial class FlowField
    {
        public struct Goal : IComponentData { }

        public struct AgentDirection : IComponentData
        {
            public float2 Value;
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