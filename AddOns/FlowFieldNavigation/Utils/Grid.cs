using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    [BurstCompile]
    public static class Grid
    {
        const float InvSqrt2 = 0.70710678118f; // 1/√2 ≈ 0.7071
        public enum Direction : byte
        {
            Up,
            Down,
            Left,
            Right,
            UpLeft,
            UpRight,
            DownLeft,
            DownRight
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 ToVector(this Direction direction)
        {
            return direction switch
            {
                Direction.Up => new float2(0, 1),
                Direction.Down => new float2(0, -1),
                Direction.Left => new float2(-1, 0),
                Direction.Right => new float2(1, 0),
                Direction.UpLeft => new float2(-InvSqrt2, InvSqrt2),
                Direction.UpRight => new float2(InvSqrt2, InvSqrt2),
                Direction.DownLeft => new float2(-InvSqrt2, -InvSqrt2),
                Direction.DownRight => new float2(InvSqrt2, -InvSqrt2),
                _ => float2.zero
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool TryGetNeighborCell(int width, int height, int2 cell, Direction direction, out int2 neighborCell)
        {
            neighborCell = cell;
            switch (direction)
            {
                case Direction.Up:
                    neighborCell.y++;
                    break;
                case Direction.Down:
                    neighborCell.y--;
                    break;
                case Direction.Left:
                    neighborCell.x--;
                    break;
                case Direction.Right:
                    neighborCell.x++;
                    break;
                case Direction.UpLeft:
                    neighborCell.x--;
                    neighborCell.y++;
                    break;
                case Direction.UpRight:
                    neighborCell.x++;
                    neighborCell.y++;
                    break;
                case Direction.DownLeft:
                    neighborCell.x--;
                    neighborCell.y--;
                    break;
                case Direction.DownRight:
                    neighborCell.x++;
                    neighborCell.y--;
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(direction), direction, null);
            }

            return IsValidCell(neighborCell, width, height);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsValidCell(int2 cell, int width, int height)
        {
            return cell.x >= 0 && cell.x < width && cell.y >= 0 && cell.y < height;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsValidIndex(int index, int width, int height)
        {
            return index >= 0 && index < width * height;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 IndexToCell(int index, int width)
        {
            return new int2(index % width, index / width);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int CellToIndex(int width, in int2 cell)
        {
            return cell.y * width + cell.x;
        }
    }
}