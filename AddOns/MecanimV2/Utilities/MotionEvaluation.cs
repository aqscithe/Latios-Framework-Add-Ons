using System;
using Latios.Kinemation;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Latios.MecanimV2
{
    public static class MotionEvaluation
    {
        public static float GetBlendedMotionDuration(ref MecanimControllerBlob controller,
                                                     ref SkeletonClipSetBlob clips,
                                                     ReadOnlySpan<MecanimParameter>    parameters,
                                                     MecanimControllerBlob.MotionIndex motion)
        {
            if (motion.invalid)
                return 0f;
            if (!motion.isBlendTree)
            {
                return clips.clips[motion.index].duration;
            }

            ref var tree = ref controller.blendTrees[motion.index];
            if (tree.blendTreeType == MecanimControllerBlob.BlendTree.BlendTreeType.Simple1D)
            {
                // Find the last child before or at our parameter
                int beforeIndex = -1;
                var parameter   = parameters[tree.parameterIndices[0]].floatParam;
                for (int i = 0; i < tree.children.Length; i++)
                {
                    if (tree.children[i].position.x <= parameter)
                        beforeIndex = i;
                    else
                        break;
                }

                // Find the first child at or after our parameter
                int afterIndex;
                if (beforeIndex >= 0 && tree.children[beforeIndex].position.x == parameter)
                    afterIndex = beforeIndex;
                else
                    afterIndex = beforeIndex + 1;

                // Try to get the child before's duration. If invalid, walk backwards.
                float beforeDuration = 0f;
                if (beforeIndex >= 0)
                {
                    var timeScale  = math.abs(tree.children[beforeIndex].timeScale);
                    beforeDuration = timeScale * GetBlendedMotionDuration(ref controller, ref clips, parameters, tree.children[beforeIndex].motionIndex);
                    while (beforeDuration == 0f)
                    {
                        beforeIndex--;
                        if (beforeIndex < 0)
                            break;
                        beforeDuration = timeScale * GetBlendedMotionDuration(ref controller, ref clips, parameters, tree.children[beforeIndex].motionIndex);
                    }
                }

                // Try to get the child after's duration. If invalid, walk backwards.
                float afterDuration = 0f;
                if (afterIndex < tree.children.Length)
                {
                    var timeScale = math.abs(tree.children[afterIndex].timeScale);
                    afterDuration = timeScale * GetBlendedMotionDuration(ref controller, ref clips, parameters, tree.children[afterIndex].motionIndex);
                    while (afterDuration == 0f)
                    {
                        afterIndex++;
                        if (afterIndex == tree.children.Length)
                            break;
                        afterDuration = timeScale * GetBlendedMotionDuration(ref controller, ref clips, parameters, tree.children[afterIndex].motionIndex);
                    }
                }

                // Process results
                if (beforeIndex < 0 && afterIndex == tree.children.Length)
                    return 0f; // Tree is totally invalid
                if (beforeIndex < 0)
                    return afterDuration;
                if (afterIndex == tree.children.Length)
                    return beforeDuration;
                return math.remap(tree.children[beforeIndex].position.x, tree.children[afterIndex].position.x, beforeDuration, afterDuration, parameter);
            }
            if (tree.blendTreeType == MecanimControllerBlob.BlendTree.BlendTreeType.SimpleDirectional2D)
            {
                // Todo: Mecanim V1 doesn't support the center clip, which means it is the wrong algorithm.
                throw new NotImplementedException();
            }
            if (tree.blendTreeType == MecanimControllerBlob.BlendTree.BlendTreeType.Direct)
            {
                var totalWeight   = 0f;
                var totalDuration = 0f;
                for (int i = 0; i < tree.children.Length; i++)
                {
                    var weight = parameters[tree.parameterIndices[i]].floatParam;
                    if (weight > 0f)
                    {
                        totalWeight   += weight;
                        totalDuration += weight * math.abs(tree.children[i].timeScale) * GetBlendedMotionDuration(ref controller,
                                                                                                                  ref clips,
                                                                                                                  parameters,
                                                                                                                  tree.children[i].motionIndex);
                    }
                }
                if (totalWeight <= 0f)
                    return 0f;
                return totalDuration / totalWeight;
            }
            // Freeform (directional or cartesian)
            {
                // See https://runevision.com/thesis/rune_skovbo_johansen_thesis.pdf at 6.3 (p58) for details.
                // Freeform cartesian uses cartesian gradient bands, while freeform directional uses gradient
                // bands in polar space.
                var          childCount = tree.children.Length;
                Span<float>  weights    = stackalloc float[childCount];
                Span<float2> pips       = stackalloc float2[childCount];
                Span<bool>   invalids   = stackalloc bool[childCount];
                invalids.Clear();
                Span<float> durations = stackalloc float[childCount];
                durations.Fill(-1f);

                var blendParameters = new float2(parameters[tree.parameterIndices[0]].floatParam, parameters[tree.parameterIndices[1]].floatParam);

                // Precompute the pip vectors, because in freeform directional, the atan2 is pricy.
                for (int i = 0; i < childCount; i++)
                {
                    var    p     = blendParameters;
                    var    pi    = tree.children[i].position;
                    var    pmag  = 0f;
                    var    pimag = 0f;
                    float2 pip;
                    if (tree.blendTreeType == MecanimControllerBlob.BlendTree.BlendTreeType.FreeformDirectional2D)
                    {
                        pmag          = math.length(p);
                        pimag         = math.length(pi);
                        pip.x         = (pmag - pimag) / (0.5f * (pmag + pimag));
                        var direction = LatiosMath.ComplexMul(pi, new float2(p.x, -p.y));
                        pip.y         = MecanimControllerBlob.BlendTree.kFreeformDirectionalBias * math.atan2(direction.y, direction.x);
                    }
                    else
                    {
                        pip = p - pi;
                    }
                    pips[i] = pip;
                }

                // We try to only sample the child nodes if they have nonzero weights. However, invalid nodes could potentially
                // result in zero-weight nodes turning into nonzero weight nodes. So we need to restart whenever we detect invalid
                // nodes, which should be rare.
                bool  foundInvalid      = false;
                float accumulatedWeight = 0f;
                while (!foundInvalid)
                {
                    foundInvalid = false;
                    // Reset the weights
                    weights.Fill(float.MaxValue);
                    accumulatedWeight = 0f;

                    for (int i = 0; i < childCount; i++)
                    {
                        if (invalids[i])
                            continue;

                        var pip = pips[i];

                        for (int j = 0; j < childCount; j++)
                        {
                            if (invalids[j] || j == i)
                                continue;

                            var pipj   = tree.pipjs[MecanimControllerBlob.BlendTree.PipjIndex(i, j, childCount)];
                            var h      = math.max(0, 1 - math.dot(pip, pipj.xy) * pipj.z);
                            weights[i] = math.min(weights[i], h);
                        }

                        accumulatedWeight += weights[i];

                        // Populate child node durations we haven't acquired yet
                        if (weights[i] > 0f && durations[i] < -0.5f)
                        {
                            durations[i] = math.abs(tree.children[i].timeScale) * GetBlendedMotionDuration(ref controller, ref clips, parameters, tree.children[i].motionIndex);
                            if (durations[i] <= 0f)
                            {
                                foundInvalid = true;
                                invalids[i]  = true;
                                break;
                            }
                        }
                    }
                }

                float inverseTotalWeight = 1f / accumulatedWeight;
                float result             = 0f;
                for (int i = 0; i < childCount; i++)
                {
                    if (invalids[i])
                        continue;
                    result += inverseTotalWeight * weights[i] * durations[i];
                }
                return result;
            }
        }

        // Todo: Root motion and normal sampling
    }
}

