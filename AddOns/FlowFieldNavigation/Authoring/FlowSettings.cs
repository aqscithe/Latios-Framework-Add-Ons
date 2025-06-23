using System;
using UnityEngine;

namespace Latios.FlowFieldNavigation
{
    [Serializable]
    public struct FlowSettings
    {
        internal const float PassabilityLimit = 500000;

        [Range(0, 10)]
        public float DensityInfluence;

        public static FlowSettings Default => new()
        {
            DensityInfluence = 1f,
        };
    }
}