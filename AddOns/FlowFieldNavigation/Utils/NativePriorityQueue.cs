using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    [BurstCompile]
    public struct NativePriorityQueue<TElement, TComparer> : IDisposable where TElement : unmanaged where TComparer : struct, IComparer<TElement>
    {
        NativeArray<TElement> data;
        int count;
        TComparer comparer;
        Allocator allocator;

        public bool IsCreated => data.IsCreated;
        public int Count => count;
        public bool IsEmpty => count == 0;

        public NativePriorityQueue(int initialCapacity, Allocator allocator, TComparer comparer = default)
        {
            initialCapacity = math.max(4, initialCapacity);
            data = new NativeArray<TElement>(initialCapacity, allocator, NativeArrayOptions.UninitializedMemory);
            count = 0;
            this.comparer = comparer;
            this.allocator = allocator;
        }

        public void Dispose()
        {
            if (data.IsCreated) data.Dispose();
            count = 0;
        }

        public JobHandle Dispose(JobHandle inputDeps)
        {
            return data.IsCreated ? data.Dispose(inputDeps) : inputDeps;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Enqueue(TElement element)
        {
            if (count == data.Length)
            {
                Resize(count * 2);
            }

            data[count] = element;
            SiftUp(count);
            count++;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public TElement Dequeue()
        {
            CheckEmpty();
            TElement min = data[0];
            count--;

            if (count > 0)
            {
                data[0] = data[count];
                SiftDown(0);
            }

            return min;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryDequeue(out TElement element)
        {
            if (IsEmpty)
            {
                element = default;
                return false;
            }

            element = Dequeue();
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryPeek(out TElement element)
        {
            if (IsEmpty)
            {
                element = default;
                return false;
            }

            element = data[0];
            return true;
        }

        void Resize(int newCapacity)
        {
            newCapacity = math.max(4, newCapacity);
            var newData = new NativeArray<TElement>(newCapacity, allocator, NativeArrayOptions.UninitializedMemory);

            if (count > 0)
            {
                NativeArray<TElement>.Copy(data, newData, count);
            }

            data.Dispose();
            data = newData;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void SiftUp(int index)
        {
            while (index > 0)
            {
                int parentIndex = (index - 1) >> 1;
                if (comparer.Compare(data[index], data[parentIndex]) >= 0) break;

                Swap(index, parentIndex);
                index = parentIndex;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void SiftDown(int index)
        {
            int minIndex = index;
            int leftChild = (index << 1) + 1;
            int rightChild = leftChild + 1;

            while (leftChild < count)
            {
                if (comparer.Compare(data[leftChild], data[minIndex]) < 0)
                {
                    minIndex = leftChild;
                }

                if (rightChild < count && comparer.Compare(data[rightChild], data[minIndex]) < 0)
                {
                    minIndex = rightChild;
                }

                if (minIndex == index) return;

                Swap(index, minIndex);
                index = minIndex;
                leftChild = (index << 1) + 1;
                rightChild = leftChild + 1;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Swap(int a, int b)
        {
            (data[a], data[b]) = (data[b], data[a]);
        }

        void CheckEmpty()
        {
            if (count == 0)
                throw new InvalidOperationException("Heap is empty");
        }
    }
}