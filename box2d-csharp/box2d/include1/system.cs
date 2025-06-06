﻿using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

// Resharper disable ALL

namespace Box2d
{
    public unsafe partial class Box2d
    {
        public static void* malloc(nuint size)
        {
#if NET6_0_OR_GREATER
            return NativeMemory.Alloc((nuint)size);
#else
            return (void*)Marshal.AllocHGlobal((nint)size);
#endif
        }

        public static void free(void* memory)
        {
#if NET6_0_OR_GREATER
            NativeMemory.Free(memory);
#else
            Marshal.FreeHGlobal((nint)memory);
#endif
        }

        public static void memcpy(void* dst, void* src, nuint size) => Unsafe.CopyBlockUnaligned(dst, src, (uint)size);

        public static void memset(void* dst, byte val, nuint size) => Unsafe.InitBlockUnaligned(dst, val, (uint)size);
    }
}