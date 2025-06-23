using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    [BurstCompile]
    public static class FieldExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetLocalPosition(this Field field, float3 worldPosition)
        {
            var gridOffset = field.GetGridOffset();
            var localPos = worldPosition - field.Transform.Value.position;
            var invRotation = math.inverse(field.Transform.Value.rotation);
            var unrotatedPos = math.rotate(invRotation, localPos);
            var adjustedPos = unrotatedPos - gridOffset;
            return adjustedPos;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float GetSpeedFactor(this Field field, int index)
        {
            var density = field.DensityMap[index];
            var velocity = field.MeanVelocityMap[index];
            var length = math.length(velocity);
            return math.lerp(1, length, math.saturate(density));
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetGridOffset(this Field field)
        {
            return new float3(-field.Width * field.CellSize.x * field.Pivot.x, 0, -field.Height * field.CellSize.y * field.Pivot.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 IndexToCell(this Field field, int index)
        {
            return Grid.IndexToCell(index, field.Width);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int CellToIndex(this Field field, int2 cell)
        {
            return Grid.CellToIndex(field.Width, cell);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 CellToWorld(this Field field, int2 cell)
        {
            var index = Grid.CellToIndex(field.Width, cell);
            return field.CellColliders[index].transform.position;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 IndexToWorld(this Field field, int index)
        {
            var cell = Grid.IndexToCell(index, field.Width);
            return CellToWorld(cell, field.GetGridOffset(), field.CellSize, field.Transform.Value.position, field.Transform.Value.rotation);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool TryWorldToCell(this Field field, float3 worldPosition, out int2 cell)
        {
            cell = WorldToCell(worldPosition, field.GetGridOffset(), field.CellSize, field.Transform.Value.position, field.Transform.Value.rotation);
            return Grid.IsValidCell(cell, field.Width, field.Height);
        }
        
        public static bool TryWorldToFootprint(this Field field, float3 worldPosition, out int4 footprint, out float2 interpolation)
        {
            var localPos = worldPosition - field.Transform.Value.position;
            var invRotation = math.inverse(field.Transform.Value.rotation);
            var unrotatedPos = math.rotate(invRotation, localPos);
            var adjustedPos = unrotatedPos - field.GetGridOffset();

            var gridCoords = new float2(adjustedPos.x / field.CellSize.x, adjustedPos.z / field.CellSize.y);
            var relativeCoords = gridCoords - 0.5f;
            var cell00 = new int2((int)math.floor(relativeCoords.x), (int)math.floor(relativeCoords.y));
            interpolation = relativeCoords - cell00;

            var minX = int.MaxValue;
            var minY = int.MaxValue;
            var maxX = int.MinValue;
            var maxY = int.MinValue;
            var foundValid = false;
    
            for (var dx = 0; dx < 2; dx++)
            {
                for (var dy = 0; dy < 2; dy++)
                {
                    var cell = cell00 + new int2(dx, dy);
                    if (!Grid.IsValidCell(cell, field.Width, field.Height)) continue;
                    foundValid = true;
                    minX = math.min(minX, cell.x);
                    minY = math.min(minY, cell.y);
                    maxX = math.max(maxX, cell.x);
                    maxY = math.max(maxY, cell.y);
                }
            }

            if (!foundValid)
            {
                footprint = default;
                interpolation = default;
                return false;
            }

            footprint = new int4(minX, minY, maxX, maxY);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool TryWorldToIndex(this Field field, float3 worldPosition, out int index)
        {
            index = WorldToIndex(worldPosition, field.GetGridOffset(), field.Width, field.CellSize, field.Transform.Value.position, field.Transform.Value.rotation);
            return Grid.IsValidIndex(index, field.Width, field.Height);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 CellToWorld(int2 cell, float3 gridOffset, float2 cellSize, float3 gridPosition, quaternion gridRotation)
        {
            var cellPosition = new float3(cell.x * cellSize.x, 0, cell.y * cellSize.y);
            cellPosition += new float3(cellSize.x * 0.5f, 0, cellSize.y * 0.5f);
            var localPosition = gridOffset + cellPosition;
            var rotatedPosition = math.rotate(gridRotation, localPosition);
            return gridPosition + rotatedPosition;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 WorldToCell(float3 worldPosition, float3 gridOffset, float2 cellSize, float3 gridPosition, quaternion gridRotation)
        {
            var localPos = worldPosition - gridPosition;
            var invRotation = math.inverse(gridRotation);
            var unrotatedPos = math.rotate(invRotation, localPos);
            var adjustedPos = unrotatedPos - gridOffset;
            return new int2((int)(adjustedPos.x / cellSize.x), (int)(adjustedPos.z / cellSize.y));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int WorldToIndex(float3 worldPosition, float3 gridOffset, int width, float2 cellSize, float3 gridPosition, quaternion gridRotation)
        {
            var cell = WorldToCell(worldPosition, gridOffset, cellSize, gridPosition, gridRotation);
            return Grid.CellToIndex(width, cell);
        }
    }
}