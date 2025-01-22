using Latios.Authoring;
using Unity.Entities;

namespace Latios.MecanimV2.Authoring
{
    public static class MecanimV2BakingBootstrap
    {
        /// <summary>
        /// Adds Mecanim bakers and baking systems into baking world
        /// </summary>
        public static void InstallMecanimV2Addon(ref CustomBakingBootstrapContext context)
        {
#if UNITY_EDITOR
            context.filteredBakerTypes.Add(typeof(AnimatorSmartBaker));
            context.bakingSystemTypesToInject.Add(TypeManager.GetSystemTypeIndex<Systems.AnimationControllerSmartBlobberSystem>());
#endif
        }
    }
}

