using System;

namespace Box2d
{
    [AttributeUsage(AttributeTargets.All)]
    public sealed class FixedArrayAttribute: Attribute
    {
        public readonly int Length;

        public FixedArrayAttribute(int length) => Length = length;
    }
}