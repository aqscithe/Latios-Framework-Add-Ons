using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Latios.MecanimV2.Authoring
{
    public struct BaseAnimatorControllerRef
    {
        internal UnityObjectRef<Object> baseControllerAsObject;

        public AnimatorControllerParameter[] parameters
        {
            get
            {
#if UNITY_EDITOR
                return controller.parameters;
#else
                return null;
#endif
            }
        }

        // Todo: What does the API look like for getting state machine state indices for code-driven crossfades?
        // Most likely, we'll implement crossfades purely by jumping to the target state and then starting an
        // inertial blend.

#if UNITY_EDITOR
        internal UnityEditor.Animations.AnimatorController controller
        {
            get => baseControllerAsObject.Value as UnityEditor.Animations.AnimatorController;
            set => baseControllerAsObject = value;
        }
#endif
    }

    public static partial class AnimatorControllerBakingExtensions
    {
        /// <summary>
        /// Finds a parameter index in an array of parameters which can be retrieved from an Animator
        /// </summary>
        /// <param name="parameters">The array of parameters</param>
        /// <param name="parameterName">The name of the parameter to find</param>
        /// <param name="parameterIndex">The found index of the parameter if found, otherwise -1</param>
        /// <returns>True if a parameter with the specified name was found</returns>
        public static bool TryGetParameter(this AnimatorControllerParameter[] parameters, string parameterName, out short parameterIndex)
        {
            parameterIndex = -1;
            for (short i = 0; i < parameters.Length; i++)
            {
                if (parameters[i].name == parameterName)
                {
                    parameterIndex = i;
                    return true;
                }
            }
            return false;
        }

        public static BaseAnimatorControllerRef GetBaseControllerOf(this IBaker baker, RuntimeAnimatorController runtimeController)
        {
            BaseAnimatorControllerRef result = default;
#if UNITY_EDITOR
            baker.DependsOn(runtimeController);
            result.baseControllerAsObject = FindBaseController(baker, runtimeController);
#endif
            return result;
        }

        /// <summary>
        /// Registers DependsOn for all state machines, states, transitions, and blend trees in the controller.
        /// Does not touch clips.
        /// </summary>
        public static void DependsOnRecursively(this IBaker baker, BaseAnimatorControllerRef baseController)
        {
#if UNITY_EDITOR
            var controller = baseController.controller;
            baker.DependsOn(controller);
            int layerCounter = 0;
            foreach (var layer in controller.layers)
            {
                var rootStateMachine = layer.stateMachine;
                var syncLayer        = layer.syncedLayerIndex < 0 ? -1 : layerCounter;
                DependsOnStateMachineRecursively(baker, controller, rootStateMachine, syncLayer);
                layerCounter++;
            }

            static void DependsOnStateMachineRecursively(IBaker baker,
                                                         UnityEditor.Animations.AnimatorController controller,
                                                         UnityEditor.Animations.AnimatorStateMachine stateMachine,
                                                         int syncLayerIndex)
            {
                baker.DependsOn(stateMachine);

                foreach (var visualState in stateMachine.states)
                {
                    var state = visualState.state;
                    if (syncLayerIndex < 0)
                    {
                        baker.DependsOn(state);
                        foreach (var transition in state.transitions)
                        {
                            baker.DependsOn(transition);
                        }
                        DependsOnBlendTreeRecursively(baker, state.motion);
                    }
                    else
                    {
                        var motion = controller.GetStateEffectiveMotion(state, syncLayerIndex);
                        DependsOnBlendTreeRecursively(baker, motion);
                    }
                }

                var sms = stateMachine.stateMachines;
                if (sms == null)
                    return;
                foreach (var sm in stateMachine.stateMachines)
                {
                    DependsOnStateMachineRecursively(baker, controller, sm.stateMachine, syncLayerIndex);
                }
            }

            static void DependsOnBlendTreeRecursively(IBaker baker, Motion motion)
            {
                if (motion is UnityEditor.Animations.BlendTree blendTree)
                {
                    baker.DependsOn(blendTree);
                    foreach (var child in blendTree.children)
                    {
                        DependsOnBlendTreeRecursively(baker, child.motion);
                    }
                }
            }
#endif
        }

#if UNITY_EDITOR
        static UnityEditor.Animations.AnimatorController FindBaseController(this IBaker baker, RuntimeAnimatorController runtimeAnimatorController)
        {
            if (runtimeAnimatorController is UnityEditor.Animations.AnimatorController animatorController)
            {
                baker.DependsOn(animatorController);
                return animatorController;
            }
            else if (runtimeAnimatorController is AnimatorOverrideController animatorOverrideController)
            {
                baker.DependsOn(animatorOverrideController);
                return FindBaseController(baker, animatorOverrideController.runtimeAnimatorController);
            }
            else
            {
                throw new System.Exception(
                    $"Encountered unknown animator controller type {runtimeAnimatorController.GetType()}. If you see this, please report a bug to the Latios Framework developers.");
            }
        }
#endif
    }
}

