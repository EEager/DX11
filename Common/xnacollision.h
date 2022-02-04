//-------------------------------------------------------------------------------------
// XNACollision.h
//  
// An opimtized collision library based on XNAMath
//  
// Microsoft XNA Developer Connection
// Copyright (c) Microsoft Corporation. All rights reserved.
//-------------------------------------------------------------------------------------

#pragma once

#ifndef _XNA_COLLISION_H_
#define _XNA_COLLISION_H_

#include <DirectXMath.h>
#include <DirectXPackedVector.h>

using namespace DirectX;
using namespace DirectX::PackedVector;


// ----------------------------------------------------------------------------------------
// Matrix type: Sixteen 32 bit floating point components aligned on a
// 16 byte boundary and mapped to four hardware vector registers
#if !defined(XM_NO_ALIGNMENT)
#define _DECLSPEC_ALIGN_16_   __declspec(align(16))
#else
#define _DECLSPEC_ALIGN_16_
#endif

// JJLEE : V1, V2 
inline XMVECTOR XMVectorPermute_JJLEE(FXMVECTOR V1, FXMVECTOR V2, XMVECTORI32 Permute)
{
    return XMVectorPermute(V1, V2,
        Permute.i[0],
        Permute.i[1],
        Permute.i[2],
        Permute.i[3]);
}

// JJLEE : XMASSERT 없어서 추가함
#if defined(_PREFAST_)
#define XMASSERT(Expression) __analysis_assume((Expression))
#elif defined(XMDEBUG) // !_PREFAST_
#define XMASSERT(Expression) ((VOID)((Expression) || (XMAssert(#Expression, __FILE__, __LINE__), 0)))
#else // !XMDEBUG
#define XMASSERT(Expression) ((VOID)0)
#endif // !XMDEBUG
// -------------------------------------------------------------------------------------------

namespace XNA
{

//-----------------------------------------------------------------------------
// Bounding volumes structures.
//
// The bounding volume structures are setup for near minimum size because there
// are likely to be many of them, and memory bandwidth and space will be at a
// premium relative to CPU cycles on Xbox 360.
//-----------------------------------------------------------------------------

#pragma warning(push)
#pragma warning(disable: 4324)

_DECLSPEC_ALIGN_16_ struct Sphere
{
    XMFLOAT3 Center;            // Center of the sphere.
    FLOAT Radius;               // Radius of the sphere.
};

_DECLSPEC_ALIGN_16_ struct AxisAlignedBox
{
    XMFLOAT3 Center;            // Center of the box.
    XMFLOAT3 Extents;           // Distance from the center to each side.
};

_DECLSPEC_ALIGN_16_ struct OrientedBox
{
    XMFLOAT3 Center;            // Center of the box.
    XMFLOAT3 Extents;           // Distance from the center to each side.
    XMFLOAT4 Orientation;       // Unit quaternion representing rotation (box -> world).
};

_DECLSPEC_ALIGN_16_ struct Frustum
{
    XMFLOAT3 Origin;            // Origin of the frustum (and projection).
    XMFLOAT4 Orientation;       // Unit quaternion representing rotation.

    FLOAT RightSlope;           // Positive X slope (X/Z).
    FLOAT LeftSlope;            // Negative X slope.
    FLOAT TopSlope;             // Positive Y slope (Y/Z).
    FLOAT BottomSlope;          // Negative Y slope.
    FLOAT Near, Far;            // Z of the near plane and far plane.
};

#pragma warning(pop)

//-----------------------------------------------------------------------------
// Bounding volume construction.
//-----------------------------------------------------------------------------
VOID ComputeBoundingSphereFromPoints( Sphere* pOut, UINT Count, const XMFLOAT3* pPoints, UINT Stride );
VOID ComputeBoundingAxisAlignedBoxFromPoints( AxisAlignedBox* pOut, UINT Count, const XMFLOAT3* pPoints, UINT Stride );
VOID ComputeBoundingOrientedBoxFromPoints( OrientedBox* pOut, UINT Count, const XMFLOAT3* pPoints, UINT Stride );
VOID ComputeFrustumFromProjection( Frustum* pOut, XMMATRIX* pProjection );
VOID ComputePlanesFromFrustum( const Frustum* pVolume, XMVECTOR* pPlane0, XMVECTOR* pPlane1, XMVECTOR* pPlane2,
                               XMVECTOR* pPlane3, XMVECTOR* pPlane4, XMVECTOR* pPlane5 );



//-----------------------------------------------------------------------------
// Bounding volume transforms.
//-----------------------------------------------------------------------------
VOID TransformSphere( Sphere* pOut, const Sphere* pIn, FLOAT Scale, FXMVECTOR Rotation, FXMVECTOR Translation );
VOID TransformAxisAlignedBox( AxisAlignedBox* pOut, const AxisAlignedBox* pIn, FLOAT Scale, FXMVECTOR Rotation,
                              FXMVECTOR Translation );
VOID TransformOrientedBox( OrientedBox* pOut, const OrientedBox* pIn, FLOAT Scale, FXMVECTOR Rotation,
                           FXMVECTOR Translation );
VOID TransformFrustum( Frustum* pOut, const Frustum* pIn, FLOAT Scale, FXMVECTOR Rotation, FXMVECTOR Translation );



//-----------------------------------------------------------------------------
// Intersection testing routines.
//-----------------------------------------------------------------------------
BOOL IntersectPointSphere( FXMVECTOR Point, const Sphere* pVolume );
BOOL IntersectPointAxisAlignedBox( FXMVECTOR Point, const AxisAlignedBox* pVolume );
BOOL IntersectPointOrientedBox( FXMVECTOR Point, const OrientedBox* pVolume );
BOOL IntersectPointFrustum( FXMVECTOR Point, const Frustum* pVolume );
//-----------------------------------------------------------------------------
// Compute the intersection of a ray (Origin, Direction) with a triangle 
// (V0, V1, V2).  Return TRUE if there is an intersection and also set *pDist 
// to the distance along the ray to the intersection.
// 
// The algorithm is based on Moller, Tomas and Trumbore, "Fast, Minimum Storage 
// Ray-Triangle Intersection", Journal of Graphics Tools, vol. 2, no. 1, 
// pp 21-28, 1997.
//-----------------------------------------------------------------------------
BOOL IntersectRayTriangle(FXMVECTOR Origin, FXMVECTOR Direction, FXMVECTOR V0, CXMVECTOR V1, CXMVECTOR V2,
    FLOAT* pDist)
{
    XMASSERT(pDist);
    XMASSERT(XMVector3IsUnit(Direction));

    static const XMVECTOR Epsilon =
    {
        1e-20f, 1e-20f, 1e-20f, 1e-20f
    };

    XMVECTOR Zero = XMVectorZero();

    XMVECTOR e1 = V1 - V0;
    XMVECTOR e2 = V2 - V0;

    // p = Direction ^ e2;
    XMVECTOR p = XMVector3Cross(Direction, e2);

    // det = e1 * p;
    XMVECTOR det = XMVector3Dot(e1, p);

    XMVECTOR u, v, t;

    if (XMVector3GreaterOrEqual(det, Epsilon))
    {
        // Determinate is positive (front side of the triangle).
        XMVECTOR s = Origin - V0;

        // u = s * p;
        u = XMVector3Dot(s, p);

        XMVECTOR NoIntersection = XMVectorLess(u, Zero);
        NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(u, det));

        // q = s ^ e1;
        XMVECTOR q = XMVector3Cross(s, e1);

        // v = Direction * q;
        v = XMVector3Dot(Direction, q);

        NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(v, Zero));
        NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(u + v, det));

        // t = e2 * q;
        t = XMVector3Dot(e2, q);

        NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(t, Zero));

        if (XMVector4EqualInt(NoIntersection, XMVectorTrueInt()))
            return FALSE;
    }
    else if (XMVector3LessOrEqual(det, -Epsilon))
    {
        // Determinate is negative (back side of the triangle).
        XMVECTOR s = Origin - V0;

        // u = s * p;
        u = XMVector3Dot(s, p);

        XMVECTOR NoIntersection = XMVectorGreater(u, Zero);
        NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(u, det));

        // q = s ^ e1;
        XMVECTOR q = XMVector3Cross(s, e1);

        // v = Direction * q;
        v = XMVector3Dot(Direction, q);

        NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(v, Zero));
        NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(u + v, det));

        // t = e2 * q;
        t = XMVector3Dot(e2, q);

        NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(t, Zero));

        if (XMVector4EqualInt(NoIntersection, XMVectorTrueInt()))
            return FALSE;
    }
    else
    {
        // Parallel ray.
        return FALSE;
    }

    XMVECTOR inv_det = XMVectorReciprocal(det);

    t *= inv_det;

    // u * inv_det and v * inv_det are the barycentric cooridinates of the intersection.

    // Store the x-component to *pDist
    XMStoreFloat(pDist, t);

    return TRUE;
}

BOOL IntersectRaySphere( FXMVECTOR Origin, FXMVECTOR Direction, const Sphere* pVolume, FLOAT* pDist );



//-----------------------------------------------------------------------------
// Return TRUE if any of the elements of a 3 vector are equal to 0xffffffff.
// Slightly more efficient than using XMVector3EqualInt.
//-----------------------------------------------------------------------------
static inline BOOL XMVector3AnyTrue(FXMVECTOR V)
{
    XMVECTOR C;

    // Duplicate the fourth element from the first element.
    C = XMVectorSwizzle(V, 0, 1, 2, 0);

    return XMComparisonAnyTrue(XMVector4EqualIntR(C, XMVectorTrueInt()));


}
//-----------------------------------------------------------------------------
// Compute the intersection of a ray (Origin, Direction) with an axis aligned 
// box using the slabs method.
//-----------------------------------------------------------------------------
static BOOL IntersectRayAxisAlignedBox(FXMVECTOR Origin, FXMVECTOR Direction, const AxisAlignedBox* pVolume, FLOAT* pDist)
{
    XMASSERT(pVolume);
    XMASSERT(pDist);
    XMASSERT(XMVector3IsUnit(Direction));

    static const XMVECTOR Epsilon =
    {
        1e-20f, 1e-20f, 1e-20f, 1e-20f
    };
    static const XMVECTOR FltMin =
    {
        -FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX
    };
    static const XMVECTOR FltMax =
    {
        FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX
    };

    // Load the box.
    XMVECTOR Center = XMLoadFloat3(&pVolume->Center);
    XMVECTOR Extents = XMLoadFloat3(&pVolume->Extents);

    // Adjust ray origin to be relative to center of the box.
    XMVECTOR TOrigin = Center - Origin;

    // Compute the dot product againt each axis of the box.
    // Since the axii are (1,0,0), (0,1,0), (0,0,1) no computation is necessary.
    XMVECTOR AxisDotOrigin = TOrigin;
    XMVECTOR AxisDotDirection = Direction;

    // if (fabs(AxisDotDirection) <= Epsilon) the ray is nearly parallel to the slab.
    XMVECTOR IsParallel = XMVectorLessOrEqual(XMVectorAbs(AxisDotDirection), Epsilon);

    // Test against all three axii simultaneously.
    XMVECTOR InverseAxisDotDirection = XMVectorReciprocal(AxisDotDirection);
    XMVECTOR t1 = (AxisDotOrigin - Extents) * InverseAxisDotDirection;
    XMVECTOR t2 = (AxisDotOrigin + Extents) * InverseAxisDotDirection;

    // Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
    // use the results from any directions parallel to the slab.
    XMVECTOR t_min = XMVectorSelect(XMVectorMin(t1, t2), FltMin, IsParallel);
    XMVECTOR t_max = XMVectorSelect(XMVectorMax(t1, t2), FltMax, IsParallel);

    // t_min.x = maximum( t_min.x, t_min.y, t_min.z );
    // t_max.x = minimum( t_max.x, t_max.y, t_max.z );
    t_min = XMVectorMax(t_min, XMVectorSplatY(t_min));  // x = max(x,y)
    t_min = XMVectorMax(t_min, XMVectorSplatZ(t_min));  // x = max(max(x,y),z)
    t_max = XMVectorMin(t_max, XMVectorSplatY(t_max));  // x = min(x,y)
    t_max = XMVectorMin(t_max, XMVectorSplatZ(t_max));  // x = min(min(x,y),z)

    // if ( t_min > t_max ) return FALSE;
    XMVECTOR NoIntersection = XMVectorGreater(XMVectorSplatX(t_min), XMVectorSplatX(t_max));

    // if ( t_max < 0.0f ) return FALSE;
    NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(XMVectorSplatX(t_max), XMVectorZero()));

    // if (IsParallel && (-Extents > AxisDotOrigin || Extents < AxisDotOrigin)) return FALSE;
    XMVECTOR ParallelOverlap = XMVectorInBounds(AxisDotOrigin, Extents);
    NoIntersection = XMVectorOrInt(NoIntersection, XMVectorAndCInt(IsParallel, ParallelOverlap));

    if (!XMVector3AnyTrue(NoIntersection))
    {
        // Store the x-component to *pDist
        XMStoreFloat(pDist, t_min);
        return TRUE;
    }

    return FALSE;
}

BOOL IntersectRayOrientedBox( FXMVECTOR Origin, FXMVECTOR Direction, const OrientedBox* pVolume, FLOAT* pDist );
BOOL IntersectTriangleTriangle( FXMVECTOR A0, FXMVECTOR A1, FXMVECTOR A2, CXMVECTOR B0, CXMVECTOR B1, CXMVECTOR B2 );
BOOL IntersectTriangleSphere( FXMVECTOR V0, FXMVECTOR V1, FXMVECTOR V2, const Sphere* pVolume );
BOOL IntersectTriangleAxisAlignedBox( FXMVECTOR V0, FXMVECTOR V1, FXMVECTOR V2, const AxisAlignedBox* pVolume );
BOOL IntersectTriangleOrientedBox( FXMVECTOR V0, FXMVECTOR V1, FXMVECTOR V2, const OrientedBox* pVolume );
BOOL IntersectSphereSphere( const Sphere* pVolumeA, const Sphere* pVolumeB );
BOOL IntersectSphereAxisAlignedBox( const Sphere* pVolumeA, const AxisAlignedBox* pVolumeB );
BOOL IntersectSphereOrientedBox( const Sphere* pVolumeA, const OrientedBox* pVolumeB );
BOOL IntersectAxisAlignedBoxAxisAlignedBox( const AxisAlignedBox* pVolumeA, const AxisAlignedBox* pVolumeB );
BOOL IntersectAxisAlignedBoxOrientedBox( const AxisAlignedBox* pVolumeA, const OrientedBox* pVolumeB );
BOOL IntersectOrientedBoxOrientedBox( const OrientedBox* pVolumeA, const OrientedBox* pVolumeB );



//-----------------------------------------------------------------------------
// Frustum intersection testing routines.
// Return values: 0 = no intersection, 
//                1 = intersection, 
//                2 = A is completely inside B
//-----------------------------------------------------------------------------
INT IntersectTriangleFrustum( FXMVECTOR V0, FXMVECTOR V1, FXMVECTOR V2, const Frustum* pVolume );
INT IntersectSphereFrustum( const Sphere* pVolumeA, const Frustum* pVolumeB );
INT IntersectAxisAlignedBoxFrustum( const AxisAlignedBox* pVolumeA, const Frustum* pVolumeB );
INT IntersectOrientedBoxFrustum( const OrientedBox* pVolumeA, const Frustum* pVolumeB );
INT IntersectFrustumFrustum( const Frustum* pVolumeA, const Frustum* pVolumeB );




//-----------------------------------------------------------------------------
// Test vs six planes (usually forming a frustum) intersection routines.
// The intended use for these routines is for fast culling to a view frustum.  
// When the volume being tested against a view frustum is small relative to the
// view frustum it is usually either inside all six planes of the frustum or 
// outside one of the planes of the frustum. If neither of these cases is true
// then it may or may not be intersecting the frustum. Outside a plane is 
// defined as being on the positive side of the plane (and inside negative).
// Return values: 0 = volume is outside one of the planes (no intersection),
//                1 = not completely inside or completely outside (intersecting),
//                2 = volume is inside all the planes (completely inside)
//-----------------------------------------------------------------------------
INT IntersectTriangle6Planes( FXMVECTOR V0, FXMVECTOR V1, FXMVECTOR V2, CXMVECTOR Plane0, CXMVECTOR Plane1,
                              CXMVECTOR Plane2, CXMVECTOR Plane3, CXMVECTOR Plane4, CXMVECTOR Plane5 );
INT IntersectSphere6Planes( const Sphere* pVolume, FXMVECTOR Plane0, FXMVECTOR Plane1, FXMVECTOR Plane2,
                            CXMVECTOR Plane3, CXMVECTOR Plane4, CXMVECTOR Plane5 );
INT IntersectAxisAlignedBox6Planes( const AxisAlignedBox* pVolume, FXMVECTOR Plane0, FXMVECTOR Plane1,
                                    FXMVECTOR Plane2, CXMVECTOR Plane3, CXMVECTOR Plane4, CXMVECTOR Plane5 );
INT IntersectOrientedBox6Planes( const OrientedBox* pVolume, FXMVECTOR Plane0, FXMVECTOR Plane1, FXMVECTOR Plane2,
                                 CXMVECTOR Plane3, CXMVECTOR Plane4, CXMVECTOR Plane5 );
INT IntersectFrustum6Planes( const Frustum* pVolume, FXMVECTOR Plane0, FXMVECTOR Plane1, FXMVECTOR Plane2,
                             CXMVECTOR Plane3, CXMVECTOR Plane4, CXMVECTOR Plane5 );


//-----------------------------------------------------------------------------
// Volume vs plane intersection testing routines.
// Return values: 0 = volume is outside the plane (on the positive sideof the plane),
//                1 = volume intersects the plane,
//                2 = volume is inside the plane (on the negative side of the plane) 
//-----------------------------------------------------------------------------
INT IntersectTrianglePlane( FXMVECTOR V0, FXMVECTOR V1, FXMVECTOR V2, CXMVECTOR Plane );
INT IntersectSpherePlane( const Sphere* pVolume, FXMVECTOR Plane );
INT IntersectAxisAlignedBoxPlane( const AxisAlignedBox* pVolume, FXMVECTOR Plane );
INT IntersectOrientedBoxPlane( const OrientedBox* pVolume, FXMVECTOR Plane );
INT IntersectFrustumPlane( const Frustum* pVolume, FXMVECTOR Plane );

}; // namespace

#endif