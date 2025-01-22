#if UNITY_EDITOR
using Latios.Authoring;
using Latios.Kinemation;
using Latios.Kinemation.Authoring;
using Unity.Collections;
using Unity.Entities;
using UnityEngine;

namespace Latios.MecanimV2.Authoring
{
    [TemporaryBakingType]
    internal struct MecanimSmartBakeItem : ISmartBakeItem<Animator>
    {
        private SmartBlobberHandle<SkeletonClipSetBlob> m_clipSetBlobHandle;
        private SmartBlobberHandle<MecanimControllerBlob> m_controllerBlobHandle;

        public bool Bake(Animator authoring, IBaker baker)
        {
            var entity = baker.GetEntity(TransformUsageFlags.Dynamic);

            var runtimeAnimatorController = authoring.runtimeAnimatorController;
            if (runtimeAnimatorController == null)
            {
                return false;
            }

            // Bake clips
            var sourceClips         = runtimeAnimatorController.animationClips;
            var skeletonClipConfigs = new NativeArray<SkeletonClipConfig>(sourceClips.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            

            for (int i = 0; i < sourceClips.Length; i++)
            {
                var sourceClip = sourceClips[i];

                skeletonClipConfigs[i] = new SkeletonClipConfig
                {
                    clip     = sourceClip,
                    events   = sourceClip.ExtractKinemationClipEvents(Allocator.Temp),
                    settings = SkeletonClipCompressionSettings.kDefaultSettings
                };
            }
            
            // Bake controller
            baker.AddComponent(entity, new MecanimController { speed = authoring.speed });
            BaseAnimatorControllerRef baseAnimatorControllerRef = baker.GetBaseControllerOf(runtimeAnimatorController);
            
            // Bake parameters
            var parameters       = baseAnimatorControllerRef.parameters;
            var parametersBuffer = baker.AddBuffer<MecanimParameter>(entity);
            foreach (var parameter in parameters)
            {
                var parameterData = new MecanimParameter();
                switch (parameter.type)
                {
                    case AnimatorControllerParameterType.Bool:
                    case AnimatorControllerParameterType.Trigger:
                    {
                        parameterData.boolParam = parameter.defaultBool;
                        break;
                    }
                    case AnimatorControllerParameterType.Float:
                    {
                        parameterData.floatParam = parameter.defaultFloat;
                        break;
                    }
                    case AnimatorControllerParameterType.Int:
                    {
                        parameterData.intParam = parameter.defaultInt;
                        break;
                    }
                }
                parametersBuffer.Add(parameterData);
            }

            // Bake layers
            var maskCount = 0;

            m_clipSetBlobHandle    = baker.RequestCreateBlobAsset(authoring, skeletonClipConfigs);
            m_controllerBlobHandle = baker.RequestCreateBlobAsset(baseAnimatorControllerRef.controller);
            // TODO: request Bone Masks Blob
            
            return true;
        }

        public void PostProcessBlobRequests(EntityManager entityManager, Entity entity)
        {
            var animatorController = entityManager.GetComponentData<MecanimController>(entity);
            animatorController.skeletonClipsBlob = m_clipSetBlobHandle.Resolve(entityManager);
            animatorController.controllerBlob = m_controllerBlobHandle.Resolve(entityManager);
            // TODO: bake animatorController.boneMasksBlob = .Resolve(entityManager);
            entityManager.SetComponentData(entity, animatorController);
        }
    }
    
    [DisableAutoCreation]
    internal class AnimatorSmartBaker : SmartBaker<Animator, MecanimSmartBakeItem>
    {
    }
}
#endif

