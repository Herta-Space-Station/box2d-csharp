using System.Runtime.CompilerServices;

namespace Box2d
{
    public static unsafe partial class Box2d
    {
        public static partial b2WorldDef b2DefaultWorldDef()
        {
            b2WorldDef def = new b2WorldDef();
            def.gravity.x = 0.0f;
            def.gravity.y = -10.0f;
            def.hitEventThreshold = 1.0f * b2_lengthUnitsPerMeter;
            def.restitutionThreshold = 1.0f * b2_lengthUnitsPerMeter;
            def.maxContactPushSpeed = 3.0f * b2_lengthUnitsPerMeter;
            def.contactHertz = 30.0f;
            def.contactDampingRatio = 10.0f;
            def.jointHertz = 60.0f;
            def.jointDampingRatio = 2.0f;
            // 400 meters per second, faster than the speed of sound
            def.maximumLinearSpeed = 400.0f * b2_lengthUnitsPerMeter;
            def.enableSleep = true;
            def.enableContinuous = true;
            def.internalValue = B2_SECRET_COOKIE;
            return def;
        }

        public static partial b2BodyDef b2DefaultBodyDef()
        {
            b2BodyDef def = new b2BodyDef();
            def.type = b2_staticBody;
            def.rotation = b2Rot_identity;
            def.sleepThreshold = 0.05f * b2_lengthUnitsPerMeter;
            def.gravityScale = 1.0f;
            def.enableSleep = true;
            def.isAwake = true;
            def.isEnabled = true;
            def.internalValue = B2_SECRET_COOKIE;
            return def;
        }

        public static partial b2Filter b2DefaultFilter()
        {
            b2Filter filter = new b2Filter { categoryBits = B2_DEFAULT_CATEGORY_BITS, maskBits = B2_DEFAULT_MASK_BITS, groupIndex = 0 };
            return filter;
        }

        public static partial b2QueryFilter b2DefaultQueryFilter()
        {
            b2QueryFilter filter = new b2QueryFilter { categoryBits = B2_DEFAULT_CATEGORY_BITS, maskBits = B2_DEFAULT_MASK_BITS };
            return filter;
        }

        public static partial b2ShapeDef b2DefaultShapeDef()
        {
            b2ShapeDef def = new b2ShapeDef();
            def.material.friction = 0.6f;
            def.density = 1.0f;
            def.filter = b2DefaultFilter();
            def.updateBodyMass = true;
            def.invokeContactCreation = true;
            def.internalValue = B2_SECRET_COOKIE;
            return def;
        }

        public static partial b2SurfaceMaterial b2DefaultSurfaceMaterial()
        {
            b2SurfaceMaterial material = new b2SurfaceMaterial { friction = 0.6f };

            return material;
        }

        private static b2SurfaceMaterial b2DefaultChainDef_defaultMaterial = new b2SurfaceMaterial { friction = 0.6f };

        public static partial b2ChainDef b2DefaultChainDef()
        {
            b2ChainDef def = new b2ChainDef();
            def.materials = (b2SurfaceMaterial*)Unsafe.AsPointer(ref b2DefaultChainDef_defaultMaterial);
            def.materialCount = 1;
            def.filter = b2DefaultFilter();
            def.internalValue = B2_SECRET_COOKIE;
            return def;
        }

        public static void b2EmptyDrawPolygon(b2Vec2* vertices, int vertexCount, b2HexColor color, void* context)
        {
        }

        public static void b2EmptyDrawSolidPolygon(b2Transform transform, b2Vec2* vertices, int vertexCount, float radius, b2HexColor color, void* context)
        {
        }

        public static void b2EmptyDrawCircle(b2Vec2 center, float radius, b2HexColor color, void* context)
        {
        }

        public static void b2EmptyDrawSolidCircle(b2Transform transform, float radius, b2HexColor color, void* context)
        {
        }

        public static void b2EmptyDrawSolidCapsule(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context)
        {
        }

        public static void b2EmptyDrawSegment(b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context)
        {
        }

        public static void b2EmptyDrawTransform(b2Transform transform, void* context)
        {
        }

        public static void b2EmptyDrawPoint(b2Vec2 p, float size, b2HexColor color, void* context)
        {
        }

        public static void b2EmptyDrawString(b2Vec2 p, byte* s, b2HexColor color, void* context)
        {
        }

        public static partial b2DebugDraw b2DefaultDebugDraw()
        {
            b2DebugDraw draw = new b2DebugDraw();

            // These allow the user to skip some implementations and not hit null exceptions.
            draw.DrawPolygonFcn = &b2EmptyDrawPolygon;
            draw.DrawSolidPolygonFcn = &b2EmptyDrawSolidPolygon;
            draw.DrawCircleFcn = &b2EmptyDrawCircle;
            draw.DrawSolidCircleFcn = &b2EmptyDrawSolidCircle;
            draw.DrawSolidCapsuleFcn = &b2EmptyDrawSolidCapsule;
            draw.DrawSegmentFcn = &b2EmptyDrawSegment;
            draw.DrawTransformFcn = &b2EmptyDrawTransform;
            draw.DrawPointFcn = &b2EmptyDrawPoint;
            draw.DrawStringFcn = &b2EmptyDrawString;
            return draw;
        }
    }
}