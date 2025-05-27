using System;

namespace Box2d
{
    [AttributeUsage(AttributeTargets.All)]
    public sealed class NativeTypeAttribute : Attribute
    {
        public readonly Type Type;

        public NativeTypeAttribute(Type type) => Type = type;
    }
}