#if UNITY_EDITOR
using Latios.Authoring;
using Unity.Entities;
using UnityEditor.Animations;

namespace Latios.MecanimV2.Authoring
{
    public static class MecanimBlobberAPIExtensions
    {
        /// <summary>
        /// Requests the creation of an MecanimControllerBlob Blob Asset
        /// </summary>
        /// <param name="animatorController">An animatorController whose layer to bake.</param>
        /// <param name="layerIndex">The index of the layer to bake.</param>
        public static SmartBlobberHandle<MecanimControllerBlob> RequestCreateBlobAsset(this IBaker baker, AnimatorController animatorController)
        {
            return baker.RequestCreateBlobAsset<MecanimControllerBlob, AnimatorControllerBakeData>(new AnimatorControllerBakeData
            {
                animatorController = animatorController
            });
        }
    }

    /// <summary>
    /// Input for the AnimatorController Smart Blobber
    /// </summary>
    public struct AnimatorControllerBakeData : ISmartBlobberRequestFilter<MecanimControllerBlob>
    {
        /// <summary>
        /// The UnityEngine.Animator to bake into a blob asset reference.
        /// </summary>
        public AnimatorController animatorController;

        public bool Filter(IBaker baker, Entity blobBakingEntity)
        {
            baker.AddComponent(blobBakingEntity, new MecanimControllerBlobRequest
            {
                animatorController = new UnityObjectRef<AnimatorController> { Value = animatorController },
            });

            return true;
        }
    }

    [TemporaryBakingType]
    internal struct MecanimControllerBlobRequest : IComponentData
    {
        public UnityObjectRef<AnimatorController> animatorController;
    }
}
#endif

