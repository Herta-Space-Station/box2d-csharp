using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2d
{
    public unsafe partial class Box2d
    {
        public static bool B2_COMPARE<TLeft, TRight>(in TLeft left, in TRight right) where TLeft : unmanaged where TRight : unmanaged => MemoryMarshal.CreateSpan(ref Unsafe.As<TLeft, byte>(ref Unsafe.AsRef(in left)), sizeof(TLeft)).SequenceEqual(MemoryMarshal.CreateSpan(ref Unsafe.As<TRight, byte>(ref Unsafe.AsRef(in right)), sizeof(TRight)));

        [Conditional("DEBUG")]
        public static void B2_ASSERT(bool condition) => Debug.Assert(condition);
    }
}