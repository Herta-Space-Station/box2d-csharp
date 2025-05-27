using System.Runtime.CompilerServices;

// Resharper disable ALL

namespace Box2d
{
    // @defgroup id Ids
    // These ids serve as handles to internal Box2D objects.
    // These should be considered opaque data and passed by value.
    // Include this header if you need the id types and not the whole Box2D API.
    // All ids are considered null if initialized to zero.
    //
    // For example in C++:
    //
    // @code{.cxx}
    // b2WorldId worldId = {};
    // @endcode
    //
    // Or in C:
    //
    // @code{.c}
    // b2WorldId worldId = {0};
    // @endcode
    //
    // These are both considered null.
    //
    // @warning Do not use the internals of these ids. They are subject to change. Ids should be treated as opaque objects.
    // @warning You should use ids to access objects in Box2D. Do not access files within the src folder. Such usage is unsupported.

    /// World id references a world instance. This should be treated as an opaque handle.
    public struct b2WorldId
    {
        public ushort index1;
        public ushort generation;
    }

    /// Body id references a body instance. This should be treated as an opaque handle.
    public struct b2BodyId
    {
        public int index1;
        public ushort world0;
        public ushort generation;
    }

    /// Shape id references a shape instance. This should be treated as an opaque handle.
    public struct b2ShapeId
    {
        public int index1;
        public ushort world0;
        public ushort generation;
    }

    /// Chain id references a chain instances. This should be treated as an opaque handle.
    public struct b2ChainId
    {
        public int index1;
        public ushort world0;
        public ushort generation;
    }

    /// Joint id references a joint instance. This should be treated as an opaque handle.
    public struct b2JointId
    {
        public int index1;
        public ushort world0;
        public ushort generation;
    }

    public unsafe partial class Box2d
    {
        // Use these to make your identifiers null.
        // You may also use zero initialization to get null.

        public static readonly b2WorldId b2_nullWorldId = new b2WorldId();
        public static readonly b2BodyId b2_nullBodyId = new b2BodyId();
        public static readonly b2ShapeId b2_nullShapeId = new b2ShapeId();
        public static readonly b2ChainId b2_nullChainId = new b2ChainId();
        public static readonly b2JointId b2_nullJointId = new b2JointId();

        /// Macro to determine if any id is null.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_IS_NULL(b2WorldId id) => id.index1 == 0;

        /// Macro to determine if any id is null.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_IS_NULL(b2BodyId id) => id.index1 == 0;

        /// Macro to determine if any id is null.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_IS_NULL(b2ShapeId id) => id.index1 == 0;

        /// Macro to determine if any id is null.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_IS_NULL(b2ChainId id) => id.index1 == 0;

        /// Macro to determine if any id is null.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_IS_NULL(b2JointId id) => id.index1 == 0;


        /// Macro to determine if any id is non-null.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_IS_NON_NULL(b2WorldId id) => id.index1 != 0;

        /// Macro to determine if any id is non-null.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_IS_NON_NULL(b2BodyId id) => id.index1 != 0;

        /// Macro to determine if any id is non-null.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_IS_NON_NULL(b2ShapeId id) => id.index1 != 0;

        /// Macro to determine if any id is non-null.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_IS_NON_NULL(b2ChainId id) => id.index1 != 0;

        /// Macro to determine if any id is non-null.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_IS_NON_NULL(b2JointId id) => id.index1 != 0;


        /// Compare two ids for equality. Doesn't work for b2WorldId.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_ID_EQUALS(b2BodyId id1, b2BodyId id2) => B2_COMPARE(id1, id2);

        /// Compare two ids for equality. Doesn't work for b2WorldId.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_ID_EQUALS(b2ShapeId id1, b2ShapeId id2) => B2_COMPARE(id1, id2);

        /// Compare two ids for equality. Doesn't work for b2WorldId.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_ID_EQUALS(b2ChainId id1, b2ChainId id2) => B2_COMPARE(id1, id2);

        /// Compare two ids for equality. Doesn't work for b2WorldId.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool B2_ID_EQUALS(b2JointId id1, b2JointId id2) => B2_COMPARE(id1, id2);

        /// Store a body id into a ulong.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong b2StoreBodyId(b2BodyId id)
        {
            return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
        }

        /// Load a ulong into a body id.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2BodyId b2LoadBodyId(ulong x)
        {
            b2BodyId id = new b2BodyId { index1 = (int)(x >> 32), world0 = (ushort)(x >> 16), generation = (ushort)(x) };
            return id;
        }

        /// Store a shape id into a ulong.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong b2StoreShapeId(b2ShapeId id)
        {
            return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
        }

        /// Load a ulong into a shape id.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2ShapeId b2LoadShapeId(ulong x)
        {
            b2ShapeId id = new b2ShapeId { index1 = (int)(x >> 32), world0 = (ushort)(x >> 16), generation = (ushort)(x) };
            return id;
        }

        /// Store a chain id into a ulong.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong b2StoreChainId(b2ChainId id)
        {
            return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
        }

        /// Load a ulong into a chain id.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2ChainId b2LoadChainId(ulong x)
        {
            b2ChainId id = new b2ChainId { index1 = (int)(x >> 32), world0 = (ushort)(x >> 16), generation = (ushort)(x) };
            return id;
        }

        /// Store a joint id into a ulong.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong b2StoreJointId(b2JointId id)
        {
            return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
        }

        /// Load a ulong into a joint id.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2JointId b2LoadJointId(ulong x)
        {
            b2JointId id = new b2JointId { index1 = (int)(x >> 32), world0 = (ushort)(x >> 16), generation = (ushort)(x) };
            return id;
        }
    }
}