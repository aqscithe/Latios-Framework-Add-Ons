using Latios.Unsafe;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    public static partial class FlowField
    {
        public static JobHandle ScheduleParallelByChunk(this BuildFlowConfig config, out Flow flow, AllocatorManager.AllocatorHandle allocator, JobHandle inputDeps = default)
        {
            config.ValidateSettings();

            flow = new Flow(config.Field, config.FlowSettings, allocator);

            var dependency = inputDeps;

            var width = config.Field.Width;
            var height = config.Field.Height;
            var clusterSize = config.FlowSettings.ClusterSize;

            dependency = new FlowFieldInternal.CollectGoalsJob
            {
                Field = config.Field,
                GoalCells = flow.GoalCells,
                TypeHandles = config.TypeHandles
            }.Schedule(config.GoalsQuery, dependency);

            dependency = new FlowFieldInternal.ResetJob { Costs = flow.Costs, GoalCells = flow.GoalCells, Width = config.Field.Width}.Schedule(dependency);

            var chunks = new NativeList<int4>( 4,Allocator.TempJob);
            var disposeDependencies = new NativeList<JobHandle>(config.FlowSettings.Iterations,Allocator.TempJob);
            
            dependency = new FlowFieldInternal.CalculateCostsForGoalChunksJob
            {
                PassabilityMap = config.Field.PassabilityMap,
                Width = config.Field.Width, Height = config.Field.Height,
                Costs = flow.Costs,
                GoalCells = flow.GoalCells, 
                ClusterSize = clusterSize,
                Chunks = chunks,
            }.Schedule(dependency);

            for (int i = 0; i < config.FlowSettings.Iterations; i++)
            {
                var upbl = new UnsafeParallelBlockList(UnsafeUtility.SizeOf<int4>(), 256, Allocator.TempJob);
                dependency = new FlowFieldInternal.CalculateCostsForChunkJob
                {
                    PassabilityMap = config.Field.PassabilityMap,
                    Width = config.Field.Width, Height = config.Field.Height,
                    Costs = flow.Costs,
                    CurrentChunks = chunks, NextChunksStream = upbl,
                    ClusterSize = clusterSize
                }.Schedule(chunks, 1, dependency);

                dependency = new FlowFieldInternal.CollectChunksJob
                {
                    Chunks = chunks, NextChunksStream = upbl
                }.Schedule(dependency);
                
                disposeDependencies.Add(upbl.Dispose(dependency));
            }
            
            dependency = new FlowFieldInternal.CalculateDirectionJob
            {
                Settings = config.FlowSettings,
                DirectionMap = flow.DirectionMap,
                CostField = flow.Costs,
                DensityField = config.Field.DensityMap,
                Field = config.Field,
                Width = config.Field.Width,
                Height = config.Field.Height,
            }.ScheduleParallel(flow.DirectionMap.Length, 32, dependency);

            disposeDependencies.Add(chunks.Dispose(dependency));
            dependency = JobHandle.CombineDependencies(disposeDependencies.AsArray());
            dependency = disposeDependencies.Dispose(dependency);
            
            return dependency;
        }
    }
}