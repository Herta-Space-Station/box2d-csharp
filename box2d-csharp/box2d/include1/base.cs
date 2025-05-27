using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2d
{
    // @defgroup base Base
    // Base functionality

    /// Prototype for user allocation function
    /// @param size the allocation size in bytes
    /// @param alignment the required alignment, guaranteed to be a power of 2
    public unsafe delegate void* b2AllocFcn(uint size, int alignment);

    /// Prototype for user free function
    /// @param mem the memory previously allocated through `b2AllocFcn`
    public unsafe delegate void b2FreeFcn(void* mem);

    /// Prototype for the user assert callback. Return 0 to skip the debugger break.
    public unsafe delegate int b2AssertFcn(byte* condition, byte* fileName, int lineNumber);

    public unsafe partial class Box2d
    {
        /// This allows the user to override the allocation functions. These should be
        /// set during application startup.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial void b2SetAllocator([NativeType(typeof(b2AllocFcn))] delegate* managed<uint, int, void*> allocFcn, [NativeType(typeof(b2FreeFcn))] delegate* managed<void*, void> freeFcn);

        /// @return the total bytes allocated by Box2D
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial int b2GetByteCount();

        /// Override the default assert callback
        /// @param assertFcn a non-null assert callback
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial void b2SetAssertFcn([NativeType(typeof(b2AssertFcn))] delegate* managed<byte*, byte*, int, int> assertFcn);
    }

    /// Version numbering scheme.
    /// See https://semver.org/
    public struct b2Version
    {
        /// Significant changes
        public int major;

        /// Incremental changes
        public int minor;

        /// Bug fixes
        public int revision;
    }

    public unsafe partial class Box2d
    {
        /// Get the current version of Box2D
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Version b2GetVersion();
    }

    public unsafe partial class Box2d
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_COMPARE<TLeft, TRight>(in TLeft left, in TRight right) where TLeft : unmanaged where TRight : unmanaged => MemoryMarshal.CreateSpan(ref Unsafe.As<TLeft, byte>(ref Unsafe.AsRef(in left)), sizeof(TLeft)).SequenceEqual(MemoryMarshal.CreateSpan(ref Unsafe.As<TRight, byte>(ref Unsafe.AsRef(in right)), sizeof(TRight)));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial int b2InternalAssertFcn(byte* condition, byte* fileName, int lineNumber);

        public static byte* __FILE__;
        public static int __LINE__;

        [Conditional("DEBUG")]
        public static void B2_ASSERT(bool condition)
        {
#if !NDEBUG || B2_ENABLE_ASSERT
            if (!condition && b2InternalAssertFcn((byte*)&condition, __FILE__, (int)__LINE__) != 0)
                Debug.Assert(condition, new StackTrace(1).ToString());
#else
            Debug.Assert(condition);
#endif
        }

        /// Get the absolute number of system ticks. The value is platform specific.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial ulong b2GetTicks();

        /// Get the milliseconds passed from an initial tick value.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial float b2GetMilliseconds(ulong ticks);

        /// Get the milliseconds passed from an initial tick value. Resets the passed in
        /// value to the current tick value.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial float b2GetMillisecondsAndReset(ulong* ticks);

        /// Yield to be used in a busy loop.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial void b2Yield();

        /// Simple djb2 hash function for determinism testing
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial uint b2Hash(uint hash, byte* data, int count);
    }
}