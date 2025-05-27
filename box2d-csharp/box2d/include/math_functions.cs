using System.Runtime.CompilerServices;
using static Box2d.Box2d;

namespace Box2d
{
    // @defgroup math Math
    // @brief Vector math types and functions

    /// 2D vector
    /// This can be used to represent a point or free vector
    public struct b2Vec2
    {
        /// coordinates
        public float x, y;

        // @brief Math operator overloads for C++
        // See math_functions.h for details.

        /// Unary negate a vector
        public static b2Vec2 operator -(b2Vec2 a)
        {
            return new b2Vec2 { x = -a.x, y = -a.y };
        }

        /// Binary vector addition
        public static b2Vec2 operator +(b2Vec2 a, b2Vec2 b)
        {
            return new b2Vec2 { x = a.x + b.x, y = a.y + b.y };
        }

        /// Binary vector subtraction
        public static b2Vec2 operator -(b2Vec2 a, b2Vec2 b)
        {
            return new b2Vec2 { x = a.x - b.x, y = a.y - b.y };
        }

        /// Binary scalar and vector multiplication
        public static b2Vec2 operator *(float a, b2Vec2 b)
        {
            return new b2Vec2 { x = a * b.x, y = a * b.y };
        }

        /// Binary scalar and vector multiplication
        public static b2Vec2 operator *(b2Vec2 a, float b)
        {
            return new b2Vec2 { x = a.x * b, y = a.y * b };
        }

        /// Binary vector equality
        public static bool operator ==(b2Vec2 a, b2Vec2 b)
        {
            return B2_COMPARE(a, b);
        }

        /// Binary vector inequality
        public static bool operator !=(b2Vec2 a, b2Vec2 b)
        {
            return !B2_COMPARE(a, b);
        }
    }

    /// Cosine and sine pair
    /// This uses a custom implementation designed for cross-platform determinism
    public struct b2CosSin
    {
        /// cosine and sine
        public float cosine;

        public float sine;
    }

    /// 2D rotation
    /// This is similar to using a complex number for rotation
    public struct b2Rot
    {
        /// cosine and sine
        public float c, s;
    }

    /// A 2D rigid transform
    public struct b2Transform
    {
        public b2Vec2 p;
        public b2Rot q;
    }

    /// A 2-by-2 Matrix
    public struct b2Mat22
    {
        /// columns
        public b2Vec2 cx, cy;
    }

    /// Axis-aligned bounding box
    public struct b2AABB
    {
        public b2Vec2 lowerBound;
        public b2Vec2 upperBound;
    }

    /// separation = dot(normal, point) - offset
    public struct b2Plane
    {
        public b2Vec2 normal;
        public float offset;
    }

    // @addtogroup math

    public static unsafe partial class Box2d
    {
        public static readonly float B2_PI = 3.14159265359f;
        public static readonly b2Vec2 b2Vec2_zero = new b2Vec2 { x = 0.0f, y = 0.0f };
        public static readonly b2Rot b2Rot_identity = new b2Rot { c = 1.0f, s = 0.0f };
        public static readonly b2Transform b2Transform_identity = new b2Transform { p = new b2Vec2 { x = 0.0f, y = 0.0f }, q = new b2Rot { c = 1.0f, s = 0.0f } };
        public static readonly b2Mat22 b2Mat22_zero = new b2Mat22 { cx = new b2Vec2 { x = 0.0f, y = 0.0f }, cy = new b2Vec2 { x = 0.0f, y = 0.0f } };

        /// @return the minimum of two integers
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int b2MinInt(int a, int b)
        {
            return a < b ? a : b;
        }

        /// @return an integer clamped between a lower and upper bound
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int b2ClampInt(int a, int lower, int upper)
        {
            return a < lower ? lower : (a > upper ? upper : a);
        }

        /// @return the minimum of two floats
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2MinFloat(float a, float b)
        {
            return a < b ? a : b;
        }

        /// @return the maximum of two floats
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2MaxFloat(float a, float b)
        {
            return a > b ? a : b;
        }

        /// @return the absolute value of a float
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2AbsFloat(float a)
        {
            return a < 0 ? -a : a;
        }

        /// @return a float clamped between a lower and upper bound
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2ClampFloat(float a, float lower, float upper)
        {
            return a < lower ? lower : (a > upper ? upper : a);
        }

        /// Compute an approximate arctangent in the range [-pi, pi]
        /// This is hand coded for cross-platform determinism. The atan2f
        /// function in the standard library is not cross-platform deterministic.
        /// Accurate to around 0.0023 degrees
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial float b2Atan2(float y, float x);

        /// Compute the cosine and sine of an angle in radians. Implemented
        /// for cross-platform determinism.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2CosSin b2ComputeCosSin(float radians);

        /// Vector dot product
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2Dot(b2Vec2 a, b2Vec2 b)
        {
            return a.x * b.x + a.y * b.y;
        }

        /// Vector cross product. In 2D this yields a scalar.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2Cross(b2Vec2 a, b2Vec2 b)
        {
            return a.x * b.y - a.y * b.x;
        }

        /// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2CrossVS(b2Vec2 v, float s)
        {
            return new b2Vec2 { x = s * v.y, y = -s * v.x };
        }

        /// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2CrossSV(float s, b2Vec2 v)
        {
            return new b2Vec2 { x = -s * v.y, y = s * v.x };
        }

        /// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2LeftPerp(b2Vec2 v)
        {
            return new b2Vec2 { x = -v.y, y = v.x };
        }

        /// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2RightPerp(b2Vec2 v)
        {
            return new b2Vec2 { x = v.y, y = -v.x };
        }

        /// Vector addition
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Add(b2Vec2 a, b2Vec2 b)
        {
            return new b2Vec2 { x = a.x + b.x, y = a.y + b.y };
        }

        /// Vector subtraction
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Sub(b2Vec2 a, b2Vec2 b)
        {
            return new b2Vec2 { x = a.x - b.x, y = a.y - b.y };
        }

        /// Vector negation
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Neg(b2Vec2 a)
        {
            return new b2Vec2 { x = -a.x, y = -a.y };
        }

        /// Vector linear interpolation
        /// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Lerp(b2Vec2 a, b2Vec2 b, float t)
        {
            return new b2Vec2 { x = (1.0f - t) * a.x + t * b.x, y = (1.0f - t) * a.y + t * b.y };
        }

        /// Component-wise multiplication
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Mul(b2Vec2 a, b2Vec2 b)
        {
            return new b2Vec2 { x = a.x * b.x, y = a.y * b.y };
        }

        /// Multiply a scalar and vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2MulSV(float s, b2Vec2 v)
        {
            return new b2Vec2 { x = s * v.x, y = s * v.y };
        }

        /// a + s * b
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2MulAdd(b2Vec2 a, float s, b2Vec2 b)
        {
            return new b2Vec2 { x = a.x + s * b.x, y = a.y + s * b.y };
        }

        /// a - s * b
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2MulSub(b2Vec2 a, float s, b2Vec2 b)
        {
            return new b2Vec2 { x = a.x - s * b.x, y = a.y - s * b.y };
        }

        /// Component-wise absolute vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Abs(b2Vec2 a)
        {
            b2Vec2 b;
            b.x = b2AbsFloat(a.x);
            b.y = b2AbsFloat(a.y);
            return b;
        }

        /// Component-wise minimum vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Min(b2Vec2 a, b2Vec2 b)
        {
            b2Vec2 c;
            c.x = b2MinFloat(a.x, b.x);
            c.y = b2MinFloat(a.y, b.y);
            return c;
        }

        /// Component-wise maximum vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Max(b2Vec2 a, b2Vec2 b)
        {
            b2Vec2 c;
            c.x = b2MaxFloat(a.x, b.x);
            c.y = b2MaxFloat(a.y, b.y);
            return c;
        }

        /// Component-wise clamp vector v into the range [a, b]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Clamp(b2Vec2 v, b2Vec2 a, b2Vec2 b)
        {
            b2Vec2 c;
            c.x = b2ClampFloat(v.x, a.x, b.x);
            c.y = b2ClampFloat(v.y, a.y, b.y);
            return c;
        }

        /// Get the length of this vector (the norm)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2Length(b2Vec2 v)
        {
            return sqrtf(v.x * v.x + v.y * v.y);
        }

        /// Get the distance between two points
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2Distance(b2Vec2 a, b2Vec2 b)
        {
            float dx = b.x - a.x;
            float dy = b.y - a.y;
            return sqrtf(dx * dx + dy * dy);
        }

        /// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
        /// todo MSVC is not inlining this function in several places per warning 4710
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Normalize(b2Vec2 v)
        {
            float length = sqrtf(v.x * v.x + v.y * v.y);
            if (length < FLT_EPSILON)
            {
                return new b2Vec2 { x = 0.0f, y = 0.0f };
            }

            float invLength = 1.0f / length;
            b2Vec2 n = new b2Vec2 { x = invLength * v.x, y = invLength * v.y };
            return n;
        }

        /// Determines if the provided vector is normalized (norm(a) == 1).
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool b2IsNormalized(b2Vec2 a)
        {
            float aa = b2Dot(a, a);
            return b2AbsFloat(1.0f - aa) < 10.0f * FLT_EPSILON;
        }

        /// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also
        /// outputs the length.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2GetLengthAndNormalize(float* length, b2Vec2 v)
        {
            *length = sqrtf(v.x * v.x + v.y * v.y);
            if (*length < FLT_EPSILON)
            {
                return new b2Vec2 { x = 0.0f, y = 0.0f };
            }

            float invLength = 1.0f / *length;
            b2Vec2 n = new b2Vec2 { x = invLength * v.x, y = invLength * v.y };
            return n;
        }

        /// Normalize rotation
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Rot b2NormalizeRot(b2Rot q)
        {
            float mag = sqrtf(q.s * q.s + q.c * q.c);
            float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
            b2Rot qn = new b2Rot { c = q.c * invMag, s = q.s * invMag };
            return qn;
        }

        /// Integrate rotation from angular velocity
        /// @param q1 initial rotation
        /// @param deltaAngle the angular displacement in radians
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Rot b2IntegrateRotation(b2Rot q1, float deltaAngle)
        {
            // dc/dt = -omega * sin(t)
            // ds/dt = omega * cos(t)
            // c2 = c1 - omega * h * s1
            // s2 = s1 + omega * h * c1
            b2Rot q2 = new b2Rot { c = q1.c - deltaAngle * q1.s, s = q1.s + deltaAngle * q1.c };
            float mag = sqrtf(q2.s * q2.s + q2.c * q2.c);
            float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
            b2Rot qn = new b2Rot { c = q2.c * invMag, s = q2.s * invMag };
            return qn;
        }

        /// Get the length squared of this vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2LengthSquared(b2Vec2 v)
        {
            return v.x * v.x + v.y * v.y;
        }

        /// Get the distance squared between points
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2DistanceSquared(b2Vec2 a, b2Vec2 b)
        {
            b2Vec2 c = new b2Vec2 { x = b.x - a.x, y = b.y - a.y };
            return c.x * c.x + c.y * c.y;
        }

        /// Make a rotation using an angle in radians
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Rot b2MakeRot(float radians)
        {
            b2CosSin cs = b2ComputeCosSin(radians);
            return new b2Rot { c = cs.cosine, s = cs.sine };
        }

        /// Compute the rotation between two unit vectors
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Rot b2ComputeRotationBetweenUnitVectors(b2Vec2 v1, b2Vec2 v2);

        /// Is this rotation normalized?
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool b2IsNormalizedRot(b2Rot q)
        {
            // larger tolerance due to failure on mingw 32-bit
            float qq = q.s * q.s + q.c * q.c;
            return 1.0f - 0.0006f < qq && qq < 1.0f + 0.0006f;
        }

        /// Normalized linear interpolation
        /// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
        /// https://web.archive.org/web/20170825184056/http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Rot b2NLerp(b2Rot q1, b2Rot q2, float t)
        {
            float omt = 1.0f - t;
            b2Rot q = new b2Rot { c = omt * q1.c + t * q2.c, s = omt * q1.s + t * q2.s, };
            float mag = sqrtf(q.s * q.s + q.c * q.c);
            float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
            b2Rot qn = new b2Rot { c = q.c * invMag, s = q.s * invMag };
            return qn;
        }

        /// Compute the angular velocity necessary to rotate between two rotations over a give time
        /// @param q1 initial rotation
        /// @param q2 final rotation
        /// @param inv_h inverse time step
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2ComputeAngularVelocity(b2Rot q1, b2Rot q2, float inv_h)
        {
            // ds/dt = omega * cos(t)
            // dc/dt = -omega * sin(t)
            // s2 = s1 + omega * h * c1
            // c2 = c1 - omega * h * s1

            // omega * h * s1 = c1 - c2
            // omega * h * c1 = s2 - s1
            // omega * h = (c1 - c2) * s1 + (s2 - s1) * c1;
            // omega * h = s1 * c1 - c2 * s1 + s2 * c1 - s1 * c1
            // omega * h = s2 * c1 - c2 * s1 = sin(a2 - a1) ~= a2 - a1 for small delta
            float omega = inv_h * (q2.s * q1.c - q2.c * q1.s);
            return omega;
        }

        /// Get the angle in radians in the range [-pi, pi]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2Rot_GetAngle(b2Rot q)
        {
            return b2Atan2(q.s, q.c);
        }

        /// Get the x-axis
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Rot_GetXAxis(b2Rot q)
        {
            b2Vec2 v = new b2Vec2 { x = q.c, y = q.s };
            return v;
        }

        /// Get the y-axis
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Rot_GetYAxis(b2Rot q)
        {
            b2Vec2 v = new b2Vec2 { x = -q.s, y = q.c };
            return v;
        }

        /// Multiply two rotations: q * r
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Rot b2MulRot(b2Rot q, b2Rot r)
        {
            // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
            // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
            // s(q + r) = qs * rc + qc * rs
            // c(q + r) = qc * rc - qs * rs
            b2Rot qr;
            qr.s = q.s * r.c + q.c * r.s;
            qr.c = q.c * r.c - q.s * r.s;
            return qr;
        }

        /// Transpose multiply two rotations: qT * r
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Rot b2InvMulRot(b2Rot q, b2Rot r)
        {
            // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
            // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
            // s(q - r) = qc * rs - qs * rc
            // c(q - r) = qc * rc + qs * rs
            b2Rot qr;
            qr.s = q.c * r.s - q.s * r.c;
            qr.c = q.c * r.c + q.s * r.s;
            return qr;
        }

        /// relative angle between b and a (rot_b * inv(rot_a))
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2RelativeAngle(b2Rot b, b2Rot a)
        {
            // sin(b - a) = bs * ac - bc * as
            // cos(b - a) = bc * ac + bs * as
            float s = b.s * a.c - b.c * a.s;
            float c = b.c * a.c + b.s * a.s;
            return b2Atan2(s, c);
        }

        /// Convert an angle in the range [-2*pi, 2*pi] into the range [-pi, pi]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2UnwindAngle(float radians)
        {
            if (radians < -B2_PI)
            {
                return radians + 2.0f * B2_PI;
            }
            else if (radians > B2_PI)
            {
                return radians - 2.0f * B2_PI;
            }

            return radians;
        }

        /// Convert any into the range [-pi, pi] (slow)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2UnwindLargeAngle(float radians)
        {
            while (radians > B2_PI)
            {
                radians -= 2.0f * B2_PI;
            }

            while (radians < -B2_PI)
            {
                radians += 2.0f * B2_PI;
            }

            return radians;
        }

        /// Rotate a vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2RotateVector(b2Rot q, b2Vec2 v)
        {
            return new b2Vec2 { x = q.c * v.x - q.s * v.y, y = q.s * v.x + q.c * v.y };
        }

        /// Inverse rotate a vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2InvRotateVector(b2Rot q, b2Vec2 v)
        {
            return new b2Vec2 { x = q.c * v.x + q.s * v.y, y = -q.s * v.x + q.c * v.y };
        }

        /// Transform a point (e.g. local space to world space)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2TransformPoint(b2Transform t, b2Vec2 p)
        {
            float x = (t.q.c * p.x - t.q.s * p.y) + t.p.x;
            float y = (t.q.s * p.x + t.q.c * p.y) + t.p.y;

            return new b2Vec2 { x = x, y = y };
        }

        /// Inverse transform a point (e.g. world space to local space)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2InvTransformPoint(b2Transform t, b2Vec2 p)
        {
            float vx = p.x - t.p.x;
            float vy = p.y - t.p.y;
            return new b2Vec2 { x = t.q.c * vx + t.q.s * vy, y = -t.q.s * vx + t.q.c * vy };
        }

        /// Multiply two transforms. If the result is applied to a point p local to frame B,
        /// the transform would first convert p to a point local to frame A, then into a point
        /// in the world frame.
        /// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
        /// = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Transform b2MulTransforms(b2Transform A, b2Transform B)
        {
            b2Transform C;
            C.q = b2MulRot(A.q, B.q);
            C.p = b2Add(b2RotateVector(A.q, B.p), A.p);
            return C;
        }

        /// Creates a transform that converts a local point in frame B to a local point in frame A.
        /// v2 = A.q' * (B.q * v1 + B.p - A.p)
        /// = A.q' * B.q * v1 + A.q' * (B.p - A.p)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Transform b2InvMulTransforms(b2Transform A, b2Transform B)
        {
            b2Transform C;
            C.q = b2InvMulRot(A.q, B.q);
            C.p = b2InvRotateVector(A.q, b2Sub(B.p, A.p));
            return C;
        }

        /// Multiply a 2-by-2 matrix times a 2D vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2MulMV(b2Mat22 A, b2Vec2 v)
        {
            b2Vec2 u = new b2Vec2
            {
                x = A.cx.x * v.x + A.cy.x * v.y, y = A.cx.y * v.x + A.cy.y * v.y,
            };
            return u;
        }

        /// Get the inverse of a 2-by-2 matrix
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Mat22 b2GetInverse22(b2Mat22 A)
        {
            float a = A.cx.x, b = A.cy.x, c = A.cx.y, d = A.cy.y;
            float det = a * d - b * c;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            b2Mat22 B = new b2Mat22
            {
                cx = new b2Vec2 { x = det * d, y = -det * c },
                cy = new b2Vec2 { x = -det * b, y = det * a },
            };
            return B;
        }

        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2Solve22(b2Mat22 A, b2Vec2 b)
        {
            float a11 = A.cx.x, a12 = A.cy.x, a21 = A.cx.y, a22 = A.cy.y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            b2Vec2 x = new b2Vec2 { x = det * (a22 * b.x - a12 * b.y), y = det * (a11 * b.y - a21 * b.x) };
            return x;
        }

        /// Does a fully contain b
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool b2AABB_Contains(b2AABB a, b2AABB b)
        {
            bool s = true;
            s = s && a.lowerBound.x <= b.lowerBound.x;
            s = s && a.lowerBound.y <= b.lowerBound.y;
            s = s && b.upperBound.x <= a.upperBound.x;
            s = s && b.upperBound.y <= a.upperBound.y;
            return s;
        }

        /// Get the center of the AABB.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2AABB_Center(b2AABB a)
        {
            b2Vec2 b = new b2Vec2 { x = 0.5f * (a.lowerBound.x + a.upperBound.x), y = 0.5f * (a.lowerBound.y + a.upperBound.y) };
            return b;
        }

        /// Get the extents of the AABB (half-widths).
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2Vec2 b2AABB_Extents(b2AABB a)
        {
            b2Vec2 b = new b2Vec2 { x = 0.5f * (a.upperBound.x - a.lowerBound.x), y = 0.5f * (a.upperBound.y - a.lowerBound.y) };
            return b;
        }

        /// Union of two AABBs
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2AABB b2AABB_Union(b2AABB a, b2AABB b)
        {
            b2AABB c;
            c.lowerBound.x = b2MinFloat(a.lowerBound.x, b.lowerBound.x);
            c.lowerBound.y = b2MinFloat(a.lowerBound.y, b.lowerBound.y);
            c.upperBound.x = b2MaxFloat(a.upperBound.x, b.upperBound.x);
            c.upperBound.y = b2MaxFloat(a.upperBound.y, b.upperBound.y);
            return c;
        }

        /// Compute the bounding box of an array of circles
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static b2AABB b2MakeAABB(b2Vec2* points, int count, float radius)
        {
            B2_ASSERT(count > 0);
            b2AABB a = new b2AABB { lowerBound = points[0], upperBound = points[0] };
            for (int i = 1; i < count; ++i)
            {
                a.lowerBound = b2Min(a.lowerBound, points[i]);
                a.upperBound = b2Max(a.upperBound, points[i]);
            }

            b2Vec2 r = new b2Vec2 { x = radius, y = radius };
            a.lowerBound = b2Sub(a.lowerBound, r);
            a.upperBound = b2Add(a.upperBound, r);

            return a;
        }

        /// Signed separation of a point from a plane
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float b2PlaneSeparation(b2Plane plane, b2Vec2 point)
        {
            return b2Dot(plane.normal, point) - plane.offset;
        }

        /// Is this a valid number? Not NaN or infinity.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial bool b2IsValidFloat(float a);

        /// Is this a valid vector? Not NaN or infinity.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial bool b2IsValidVec2(b2Vec2 v);

        /// Is this a valid rotation? Not NaN or infinity. Is normalized.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial bool b2IsValidRotation(b2Rot q);

        /// Is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower bound.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial bool b2IsValidAABB(b2AABB aabb);

        /// Is this a valid plane? Normal is a unit vector. Not Nan or infinity.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial bool b2IsValidPlane(b2Plane a);

        /// Box2D bases all length units on meters, but you may need different units for your game.
        /// You can set this value to use different units. This should be done at application startup
        /// and only modified once. Default value is 1.
        /// For example, if your game uses pixels for units you can use pixels for all length values
        /// sent to Box2D. There should be no extra cost. However, Box2D has some internal tolerances
        /// and thresholds that have been tuned for meters. By calling this function, Box2D is able
        /// to adjust those tolerances and thresholds to improve accuracy.
        /// A good rule of thumb is to pass the height of your player character to this function. So
        /// if your player character is 32 pixels high, then pass 32 to this function. Then you may
        /// confidently use pixels for all the length values sent to Box2D. All length values returned
        /// from Box2D will also be pixels because Box2D does not do any scaling internally.
        /// However, you are now on the hook for coming up with good values for gravity, density, and
        /// forces.
        /// @warning This must be modified before any calls to Box2D
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial void b2SetLengthUnitsPerMeter(float lengthUnits);

        /// Get the current length units per meter.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial float b2GetLengthUnitsPerMeter();
    }
}