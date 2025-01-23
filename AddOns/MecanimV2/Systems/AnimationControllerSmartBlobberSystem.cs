#if UNITY_EDITOR
using System.Collections.Generic;
using System.Linq;
using Latios.Authoring;
using Latios.Authoring.Systems;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Entities.LowLevel.Unsafe;
using UnityEditor.Animations;
using UnityEngine;

// Todo: This blob building system is very heavy on allocations with the use of Linq.
// It may be worth optimizing at some point.

namespace Latios.MecanimV2.Authoring.Systems
{
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    [UpdateInGroup(typeof(SmartBlobberBakingGroup))]
    [RequireMatchingQueriesForUpdate]
    [DisableAutoCreation]
    public partial class AnimationControllerSmartBlobberSystem : SystemBase
    {
        protected override void OnCreate()
        {
            new SmartBlobberTools<MecanimControllerBlob>().Register(World);
        }

        protected override void OnUpdate()
        {
            int count = SystemAPI.QueryBuilder().WithAll<MecanimControllerBlobRequest>().WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities)
                        .Build().CalculateEntityCountWithoutFiltering();
            var hashmap = new NativeParallelHashMap<UnityObjectRef<AnimatorController>, BlobAssetReference<MecanimControllerBlob> >(count * 2, WorldUpdateAllocator);

            new GatherJob { hashmap = hashmap.AsParallelWriter() }.ScheduleParallel();
            CompleteDependency();

            foreach (var pair in hashmap)
            {
                pair.Value = BakeAnimatorController(pair.Key.Value);
            }

            Entities.WithReadOnly(hashmap).ForEach((ref SmartBlobberResult result, in MecanimControllerBlobRequest request) =>
            {
                var controllerBlob = hashmap[request.animatorController];
                result.blob        = UnsafeUntypedBlobAssetReference.Create(controllerBlob);
            }).WithEntityQueryOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities).ScheduleParallel();
        }

        [WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities)]
        [BurstCompile]
        partial struct GatherJob : IJobEntity
        {
            public NativeParallelHashMap<UnityObjectRef<AnimatorController>, BlobAssetReference<MecanimControllerBlob> >.ParallelWriter hashmap;

            public void Execute(in MecanimControllerBlobRequest request)
            {
                hashmap.TryAdd(request.animatorController, default);
            }
        }

        private void BakeAnimatorCondition(ref MecanimControllerBlob.Condition blobAnimatorCondition, AnimatorCondition condition,
                                           AnimatorControllerParameter[] parameters)
        {
            blobAnimatorCondition.mode      = (MecanimControllerBlob.Condition.ConditionType) condition.mode;

            parameters.TryGetParameter(condition.parameter, out short conditionParameterIndex);
            blobAnimatorCondition.parameterIndex = conditionParameterIndex;
            blobAnimatorCondition.compareValue = new MecanimParameter { floatParam = condition.threshold };
        }

        private void BakeAnimatorStateTransition(ref BlobBuilder builder, ref MecanimControllerBlob.Transition blobTransition, AnimatorStateTransition transition,
                                                 AnimatorState[] states, AnimatorControllerParameter[] parameters)
        {
            blobTransition.hasExitTime         = transition.hasExitTime;
            blobTransition.normalizedExitTime  = transition.exitTime / transition.duration;
            blobTransition.normalizedOffset    = transition.offset / transition.duration;
            blobTransition.duration            = transition.duration;
            blobTransition.interruptionSource  = (MecanimControllerBlob.Transition.InterruptionSource) transition.interruptionSource;
            blobTransition.usesOrderedInterruptions = transition.orderedInterruption;

            BlobBuilderArray<MecanimControllerBlob.Condition> conditionsBuilder =
                builder.Allocate(ref blobTransition.conditions, transition.conditions.Length);
            
            for (int i = 0; i < transition.conditions.Length; i++)
            {
                var conditionBlob = conditionsBuilder[i];
                BakeAnimatorCondition(ref conditionBlob, transition.conditions[i], parameters);
                conditionsBuilder[i] = conditionBlob;
            }

            var destinationStateIndex = -1;
            for (int i = 0; i < states.Length; i++)
            {
                if (states[i] == transition.destinationState)
                {
                    destinationStateIndex = i;
                    break;
                }
            }

            blobTransition.destinationStateIndex = (short)destinationStateIndex;
        }

        

        private void BakeAnimatorStateMachineState(ref BlobBuilder builder,
                                        ref MecanimControllerBlob.State blobState,
                                        Motion motion,
                                        AnimatorState parentState,
                                        List<ChildMotion>             motions,
                                        AnimatorControllerParameter[] parameters,
                                        AnimationClip[]               clips)
        {
            blobState.baseStateSpeed                = parentState.speed;
            blobState.motionCycleOffset             = parentState.cycleOffset;
            
            blobState.useMirror                     = parentState.mirror;
            blobState.useFootIK                     = parentState.iKOnFeet;

            blobState.stateSpeedMultiplierParameterIndex = -1;
            if (parentState.speedParameterActive)
            {
                for (int i = 0; i < parameters.Length; i++)
                {
                    if (parameters[i].name == parentState.speedParameter)
                    {
                        blobState.stateSpeedMultiplierParameterIndex = (short)i;
                        break;
                    }
                }
            }

            blobState.motionCycleOffsetParameterIndex = -1;
            if (parentState.cycleOffsetParameterActive)
            {
                for (int i = 0; i < parameters.Length; i++)
                {
                    if (parameters[i].name == parentState.cycleOffsetParameter)
                    {
                        blobState.motionCycleOffsetParameterIndex = (short)i;
                        break;
                    }
                }
            }

            blobState.mirrorParameterIndex = -1;
            if (parentState.mirrorParameterActive)
            {
                for (int i = 0; i < parameters.Length; i++)
                {
                    if (parameters[i].name == parentState.mirrorParameter)
                    {
                        blobState.mirrorParameterIndex = (short)i;
                        break;
                    }
                }
            }

            blobState.motionTimeOverrideParameterIndex = -1;
            if (parentState.timeParameterActive)
            {
                for (int i = 0; i < parameters.Length; i++)
                {
                    if (parameters[i].name == parentState.timeParameter)
                    {
                        blobState.motionTimeOverrideParameterIndex = (short)i;
                        break;
                    }
                }
            }

            
            //Get the clip index
            blobState.motionIndexInLayer = -1;
            for (int i = 0; i < clips.Length; i++)
            {
                if (clips[i] == motion)
                {
                    blobState.motionIndexInLayer = (short)i;
                    break;
                }
            }
        }

        List<ChildMotion> m_motionCache = new List<ChildMotion>();

        private MecanimControllerBlob.StateMachine BakeAnimatorControllerStateMachine(ref BlobBuilder builder,
                                                                       ref MecanimControllerBlob.StateMachine stateMachineBlob,
                                                                       AnimatorControllerLayer layer,
                                                                       AnimationClip[]                clips,
                                                                       AnimatorControllerParameter[]  parameters)
        {
            // TODO: bake initializationEntryStateTransitions
            
            //Gather states motions for reference
            var states = layer.stateMachine.states.Select(x => x.state).ToArray();
            m_motionCache.Clear();
            var childMotions = m_motionCache;
            foreach (var state in states)
            {
                PopulateChildMotions(ref childMotions, state.motion, state);
            }

            //States
            BlobBuilderArray<MecanimControllerBlob.State> statesBuilder =
                builder.Allocate(ref stateMachineBlob.states, states.Length);
            
            BlobBuilderArray<int> stateNameHashesBuilder =
                builder.Allocate(ref stateMachineBlob.stateNameHashes, states.Length);
            BlobBuilderArray<int> stateNameEditorHashesBuilder =
                builder.Allocate(ref stateMachineBlob.stateNameEditorHashes, states.Length);
            BlobBuilderArray<FixedString128Bytes> stateNamesBuilder =
                builder.Allocate(ref stateMachineBlob.stateNames, states.Length);
            BlobBuilderArray<FixedString128Bytes> stateTagsBuilder =
                builder.Allocate(ref stateMachineBlob.stateTags, states.Length);
            
            for (int i = 0; i < states.Length; i++)
            {
                ref var stateBlob = ref statesBuilder[i];
                BakeAnimatorStateMachineState(ref builder, ref stateBlob, states[i].motion, states[i], childMotions, parameters, clips);

                stateNameHashesBuilder[i] = states[i].name.GetHashCode();
                stateNameEditorHashesBuilder[i] = states[i].name.GetHashCode();
                stateNamesBuilder[i] = states[i].name;
                stateTagsBuilder[i] = states[i].tag;
                
                statesBuilder[i] = stateBlob;
            }
            
            //Transitions
            BlobBuilderArray<MecanimControllerBlob.Transition> anyStateTransitionsBuilder =
                builder.Allocate(ref stateMachineBlob.anyStateTransitions, layer.stateMachine.anyStateTransitions.Length);
            for (int i = 0; i < layer.stateMachine.anyStateTransitions.Length; i++)
            {
                ref var anyStateTransitionBlob = ref anyStateTransitionsBuilder[i];
                BakeAnimatorStateTransition(ref builder, ref anyStateTransitionBlob, layer.stateMachine.anyStateTransitions[i], states, parameters);
                anyStateTransitionsBuilder[i] = anyStateTransitionBlob;
            }

            return stateMachineBlob;
        }
        
        private MecanimControllerBlob.Layer BakeAnimatorControllerLayer(ref BlobBuilder builder,
                                                                       ref MecanimControllerBlob.Layer layerBlob,
                                                                       AnimatorControllerLayer layer,
                                                                       short stateMachineIndex,
                                                                       AnimationClip[]                clips,
                                                                       AnimatorControllerParameter[]  parameters)
        {
            layerBlob.name                        = layer.name;
            layerBlob.originalLayerWeight         = layer.defaultWeight;
            layerBlob.performIKPass               = layer.iKPass;
            layerBlob.useAdditiveBlending         = layer.blendingMode == AnimatorLayerBlendingMode.Additive;
            layerBlob.syncLayerUsesBlendedTimings = layer.syncedLayerAffectsTiming;

            layerBlob.stateMachineIndex           = stateMachineIndex;
            layerBlob.isSyncLayer                 = layer.syncedLayerIndex != -1;
            layerBlob.syncLayerIndex              = (short) layer.syncedLayerIndex;
            
            //TODO: layerBlob.boneMaskIndex
            //TODO: layerBlob.motionIndices
            
            return layerBlob;
        }

        private BlobAssetReference<MecanimControllerBlob> BakeAnimatorController(AnimatorController animatorController)
        {
            var builder                    = new BlobBuilder(Allocator.Temp);
            ref var blobAnimatorController = ref builder.ConstructRoot<MecanimControllerBlob>();
            blobAnimatorController.name    = animatorController.name;
            
            BlobBuilderArray<MecanimControllerBlob.StateMachine> stateMachinesBuilder =
                builder.Allocate(ref blobAnimatorController.stateMachines, GetStateMachinesCount(animatorController.layers));
            BlobBuilderArray<MecanimControllerBlob.Layer> layersBuilder =
                builder.Allocate(ref blobAnimatorController.layers, animatorController.layers.Length);
            
            NativeHashMap<short, short> owningLayerToStateMachine = new NativeHashMap<short, short>(1, Allocator.Temp);
            
            builder = BakeStateMachines(animatorController, ref owningLayerToStateMachine, ref stateMachinesBuilder, ref builder);

            builder = BakeLayers(animatorController, ref owningLayerToStateMachine, ref layersBuilder, ref builder);

            BakeParameters(animatorController, ref builder, ref blobAnimatorController);

            var result = builder.CreateBlobAssetReference<MecanimControllerBlob>(Allocator.Persistent);

            return result;
        }

        private BlobBuilder BakeLayers(
            AnimatorController animatorController,
            ref NativeHashMap<short, short> owningLayerToStateMachine,
            ref BlobBuilderArray<MecanimControllerBlob.Layer> layersBuilder,
            ref BlobBuilder builder)
        {
            for (short i = 0; i < animatorController.layers.Length; i++)
            {
                var layer = animatorController.layers[i];

                // Get the state machine index using the current layer index, or the layer index we are syncing with
                short stateMachineIndex = owningLayerToStateMachine[(short) (layer.syncedLayerIndex == -1 ? i : layer.syncedLayerIndex)]; 

                ref var layerBlob = ref layersBuilder[i];
                BakeAnimatorControllerLayer(ref builder, ref layerBlob, layer, stateMachineIndex, animatorController.animationClips, animatorController.parameters);
                layersBuilder[i] = layerBlob;
            }

            return builder;
        }

        private int GetStateMachinesCount(AnimatorControllerLayer[] animatorControllerLayers)
        {
            int stateMachinesCount = 0;
            foreach (var animatorControllerLayer in animatorControllerLayers)
            {
                if (animatorControllerLayer.syncedLayerIndex == -1)
                {
                    stateMachinesCount++;
                }
            }

            return stateMachinesCount;
        }
        
        private BlobBuilder BakeStateMachines(
            AnimatorController animatorController,
            ref NativeHashMap<short, short> owningLayerToStateMachine,
            ref BlobBuilderArray<MecanimControllerBlob.StateMachine> stateMachinesBuilder,
            ref BlobBuilder builder)
        {
            NativeParallelMultiHashMap<short, short> layersInfluencingTimingsByAffectedLayer = new NativeParallelMultiHashMap<short, short>(1, Allocator.Temp);
            
            short stateMachinesAdded = 0;
            for (short i = 0; i < animatorController.layers.Length; i++)
            {
                var layer = animatorController.layers[i];

                if (layer.syncedLayerIndex == -1)
                {
                    // Not a synced layer, add new state machine
                    ref var stateMachineBlob = ref stateMachinesBuilder[stateMachinesAdded];
                    BakeAnimatorControllerStateMachine(ref builder, ref stateMachineBlob, layer, animatorController.animationClips, animatorController.parameters);
                    stateMachinesBuilder[stateMachinesAdded] = stateMachineBlob;
                    
                    // Associate the index of this layer with its state machine index for easier lookups
                    owningLayerToStateMachine.Add(i, stateMachinesAdded);

                    
                    
                    stateMachinesAdded++;
                } else if (layer.syncedLayerAffectsTiming)
                {
                    // This is a synced layer that affects timings. Save it as an influencing layer for the layer it's syncing with.
                    layersInfluencingTimingsByAffectedLayer.Add((short) layer.syncedLayerIndex, i);
                }
            }
            
            PopulateLayersAffectingStateMachinesTimings(animatorController, owningLayerToStateMachine, ref stateMachinesBuilder, builder, layersInfluencingTimingsByAffectedLayer);
            
            return builder;
        }

        private static void PopulateLayersAffectingStateMachinesTimings(
            AnimatorController animatorController,
            NativeHashMap<short, short> owningLayerToStateMachine,
            ref BlobBuilderArray<MecanimControllerBlob.StateMachine> stateMachinesBuilder,
            BlobBuilder builder,
            NativeParallelMultiHashMap<short, short> layersInfluencingTimingsByAffectedLayer)
        {
            // Populate list of layers that affects each state machine timings.
            for (short i = 0; i < animatorController.layers.Length; i++)
            {
                if (!owningLayerToStateMachine.ContainsKey(i)) continue;

                int influencingLayersCount = layersInfluencingTimingsByAffectedLayer.CountValuesForKey(i);
                if (influencingLayersCount == 0) continue;

                // Get associated state machine blob
                ref var stateMachineBlob = ref stateMachinesBuilder[owningLayerToStateMachine[i]];

                // initialize the blob array for layers affecting timings on the state machine blob
                BlobBuilderArray<short> influencingLayersBuilder = builder.Allocate(ref stateMachineBlob.influencingSyncLayers, influencingLayersCount);

                var indexInArrayOfInfluencingLayers = 0;
                if (layersInfluencingTimingsByAffectedLayer.TryGetFirstValue(i, out short influencingLayer, out NativeParallelMultiHashMapIterator<short> it))
                {
                    do
                    {
                        influencingLayersBuilder[indexInArrayOfInfluencingLayers] = influencingLayer;
                        indexInArrayOfInfluencingLayers++;
                    } while (layersInfluencingTimingsByAffectedLayer.TryGetNextValue(out influencingLayer, ref it));
                }

                stateMachinesBuilder[owningLayerToStateMachine[i]] = stateMachineBlob;
            }
        }

        private static void BakeParameters(AnimatorController animatorController, ref BlobBuilder builder, ref MecanimControllerBlob blobAnimatorController)
        {
            var parametersCount = animatorController.parameters.Length;
            
            blobAnimatorController.parameterTypes = new MecanimControllerBlob.ParameterTypes();
            BlobBuilderArray<int> packedTypes = 
                builder.Allocate(
                    ref blobAnimatorController.parameterTypes.packedTypes, 
                    MecanimControllerBlob.ParameterTypes.PackedTypesArrayLength(parametersCount));

            BlobBuilderArray<int> parameterNameHashes = builder.Allocate(ref blobAnimatorController.parameterNameHashes, parametersCount);
            BlobBuilderArray<int> parameterEditorNameHashes = builder.Allocate(ref blobAnimatorController.parameterEditorNameHashes, parametersCount);
            BlobBuilderArray<FixedString64Bytes> parameterNames = builder.Allocate(ref blobAnimatorController.parameterNames, parametersCount);
            
            // Bake parameter types names and hashes
            for (int i = 0; i < parametersCount; i++)
            {
                var parameter = animatorController.parameters[i];

                int nameHash = parameter.name.GetHashCode();

                MecanimControllerBlob.ParameterTypes.PackTypeIntoBlobBuilder(ref packedTypes, i, parameter.type);
                
                parameterNameHashes[i] = nameHash;
                parameterEditorNameHashes[i] = nameHash;
                parameterNames[i] = parameter.name;
            }
        }

        private void PopulateChildMotions(ref List<ChildMotion> motions, Motion motion, AnimatorState parentState)
        {
            if (motion is BlendTree blendTree)
            {
                foreach (var childMotion in blendTree.children)
                {
                    motions.Add(new ChildMotion { Motion = childMotion.motion, ParentState = parentState });

                    if (childMotion.motion is BlendTree childBlendTree)
                    {
                        PopulateChildMotions(ref motions, childBlendTree, parentState);
                    }
                }
            }
        }

        private struct ChildMotion
        {
            public Motion Motion;
            public AnimatorState ParentState;
        }
    }
}
#endif