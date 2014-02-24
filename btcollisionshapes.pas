unit btCollisionShapes;

// Bullet Source Code Translation / DI Helmut Hartl / FirmOS Business Solutions GmbH
// Original Code Licence
{*
Copyright (c) 2003-2009 Erwin Coumans  http://bullet.googlecode.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

---
*}

// FirmOS Porting Licence
// The code written here is a derivative work of the above mentioned "original" code.
// The code was translated to Freepascal and changed/enhanced or stripped where necessary to fit the needs of the FirmOS System
// If you want to use this code you need to comply to the "original" licence and the following 4 Clause BSD Style Licence.
//
// Copyright (c) 2000-2010
//
// FirmOS Business Solutions GmbH. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
//
//   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
//      in the documentation and/or other materials provided with the distribution.
//   3. All advertising materials mentioning features or use of this software must display the following acknowledgement:
//      “This product includes software developed by FirmOS Business Solutions,  and its contributors - www.firmos.com”
//   4. Neither the name of FirmOS nor the names of its contributors may be used to endorse or promote
//      products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///Collision Shapes - Bullet Common

///Common headerfile includes for Bullet Collision Detection

///Bullet's btCollisionWorld and btCollisionObject definitions
//#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"

/////Narrowphase Collision Detector

// FOS -- btScaledBvhTriangleMeshShape no demo found
// FOS -- btMultimaterialTriangleMeshShape demo found, delayed
// FOS -- btHeightfieldTerrainShape demo found, delayed
//xxx #include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h" (deprecated?)

// Files:

//  +btCollisionMargin.h
//  +btConcaveShape.h
//  +btConcaveShape.cpp
//  +btCollisionShape.h
//  +btCollisionShape.cpp
//  +btCompoundShape.h
//  +btCompoundShape.cpp
//  +btEmptyShape.h
//  +btEmptyShape.cpp
//  +btTriangleCallback.h
//  +btStridingMeshInterface.h
//  +btStridingMeshInterface.cpp
//  +btTriangleIndexVertexArray.h
//  +btTriangleIndexVertexArray.cpp
//  +btConvexShape.h
//  +btConvexShape.cpp
//  +btUniformScalingShape.h
//  +btUniformScalingShape.cpp
//  +btConvexInternalShape.h
//  +btConvexInternalShape.cpp
//  +btMultiSphereShape.h
//  +btMultiSphereShape.cpp
//  +btMinkowskiSumShape.h
//  +btMinkowskiSumShape.cpp
//  +btPolyhedralConvexShape.h
//  +btPolyhedralConvexShape.cpp
//  +btTetrahedronShape.h
//  +btTetrahedronShape.cpp
//  +btSphereShape.h
//  +btSphereShape.cpp
//  +btBoxShape.h
//  +btBoxShape.cpp
//  +btTriangleShape.h
//  +btCapsuleShape.h
//  +btCapsuleShape.cpp
//  +btCylinderShape.h
//  +btCylinderShape.cpp
//  +btConeShape.h
//  +btConeShape.cpp
//  +btStaticPlaneShape.h
//  +btStaticPlaneShape.cpp
//  +btConvexHullShape.h
//  +btConvexHullShape.cpp
//  +btTriangleMesh.h
//  +btTriangleMesh.cpp
//  +btTriangleMeshShape.h
//  +btTriangleMeshShape.cpp
//  +btoptimizedBvh.h
//  +btoptimizedBvh.cpp
//  +btTriangleInfo.h
//  +btBvhTriangleMeshShape.h
//  +btBvhTriangleMeshShape.cpp
//  +btConvexTriangleMeshShape.h
//  +btConvexTriangleMeshShape.cpp (w/o calculatePrincipalAxisTransform ) The btConvexTriangleMeshShape is a convex hull of a triangle mesh, but the performance is not as good as btConvexHullShape.
//  +btShapeHull.h   START
//  +btShapeHull.cpp START

interface

{$i fos_bullet.inc}

uses
  sysutils,btLinearMath,btBroadphase,FOS_AlignedArray;


type
    PHY_ScalarType= (PHY_FLOAT,PHY_DOUBLE,PHY_INTEGER,PHY_SHORT,PHY_FIXEDPOINT8,PHY_UCHAR);

    btCollisionShape=class
    protected
      m_shapeType          : TbtBroadphaseNativeTypes;
      m_userPointer        : pointer;
    public
      constructor Create;
      procedure  getAabb                       (const t:btTransform;out aabbMin,aabbMax:btVector3); virtual; abstract; //getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
      procedure  getBoundingSphere             (var center:btVector3;var radius:btScalar); virtual;
      function   getAngularMotionDisc          :btScalar;virtual;                                                      //getAngularMotionDisc returns the maximus radius needed for Conservative Advancement to handle time-of-impact with rotations.
      function   getContactBreakingThreshold   (const defaultContactThresholdFactor:btScalar):btScalar;virtual;
      procedure  calculateTemporalAabb         (const curTrans: btTransform;const linvel,angvel:btVector3;timeStep:btScalar;out  temporalAabbMin,temporalAabbMax:btVector3); //calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep) result is conservative
      function   isPolyhedral                  :boolean;FOS_INLINE;
      function   isConvex2d                    :boolean;FOS_INLINE;
      function   isConvex                      :boolean;FOS_INLINE;
      function   isNonMoving                   :boolean;FOS_INLINE;
      function   isConcave                     :boolean;FOS_INLINE;
      function   isCompound                    :boolean;FOS_INLINE;
      function   isSoftBody                    :boolean;FOS_INLINE;
      function   isInfinite                    :boolean;FOS_INLINE; //isInfinite is used to catch simulation error (aabb check)
      function   getShapeType                  :TbtBroadphaseNativeTypes;
      procedure  setMargin                     (const margin:btScalar);virtual;abstract;
      function   getMargin                     :btScalar;virtual;abstract;
      procedure  setUserPointer                (const ptr:Pointer);
      function   getUserPointer                :Pointer;
      procedure  setLocalScaling               (const scaling:btVector3);virtual;abstract;
      function   getLocalScaling               :btVector3;virtual;abstract;
      procedure  calculateLocalInertia         (const mass:btScalar;var inertia:btVector3);virtual;abstract;
    end;

    { btTriangleCallback }

    btTriangleCallback=class
      procedure processTriangle(const triangle:PbtVector3;const  partId,triangleIndex:integer);virtual;abstract;
    end;

    { btInternalTriangleIndexCallback }

    btInternalTriangleIndexCallback=class(btTriangleCallback) // FOS: changed to be a subclass of btTriangleCallback (multiple inheritance)
      procedure processTriangle(const triangle:PbtVector3;const  partId,triangleIndex:integer);override;
      procedure internalProcessTriangleIndex(const triangle:PbtVector3;const  partId,triangleIndex:integer);virtual;abstract;
    end;


    btCompoundShapeChild=object
      m_transform      : btTransform;
      m_childShape     : btCollisionShape;
      m_childShapeType : TbtBroadphaseNativeTypes;
      m_childMargin    : btScalar;
      m_node           : PbtDbvtNode;
    end;
    PbtCompoundShapeChild = ^btCompoundShapeChild;

    operator= (const c1,c2:btCompoundShapeChild) r:Boolean;FOS_INLINE;

    /// The btCompoundShape allows to store multiple other btCollisionShapes
    /// This allows for moving concave collision objects. This is more general then the static concave btBvhTriangleMeshShape.
    /// It has an (optional) dynamic aabb tree to accelerate early rejection tests.
    /// @todo: This aabb tree can also be use to speed up ray tests on btCompoundShape, see http://code.google.com/p/bullet/issues/detail?id=25
    /// Currently, removal of child shapes is only supported when disabling the aabb tree (pass 'false' in the constructor of btCompoundShape)
type
    btCompoundShapeChildArray=specialize FOS_GenericAlignedArray<btCompoundShapeChild>;

    { btCompoundShape }

    btCompoundShape = class(btCollisionShape)
      m_children        : btCompoundShapeChildArray;
      m_localAabbMin    : btVector3;
      m_localAabbMax    : btVector3;
      m_dynamicAabbTree : btDbvt;
    ///increment m_updateRevision when adding/removing/replacing child shapes, so that some caches can be updated
      m_updateRevision  : integer;
      m_collisionMargin : btScalar;
    protected
      m_localScaling    : btVector3;
    public
      constructor Create                        (enableDynamicAabbTree:boolean=true);
      destructor  Destroy                       ;override;
      procedure   addChildShape                 (const localTransform:btTransform;const shape:btCollisionShape);
     /// Remove all children shapes that contain the specified shape
      procedure  removeChildShape               (const shape:btCollisionShape);virtual;
      procedure  removeChildShapeByIndex        (const childShapeindex:integer);
      function   getNumChildShapes              :integer;
      function   getChildShape                  (const index:integer):btCollisionShape;
      function   getChildTransform              (const index:integer):btTransform;
      function   getChildTransformP             (const index:integer):PbtTransform;
    ///set a new transform for a child, and update internal data structures (local aabb and dynamic tree)
      procedure  updateChildTransform           (const childIndex:integer;const newChildTransform:btTransform);
      function   getChildList                   :PbtCompoundShapeChild;
    ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
      procedure  getAabb                        (const t: btTransform; out aabbMin, aabbMax: btVector3); override;
    // Re-calculate the local Aabb. Is called at the end of removeChildShapes.
    // Use this yourself if you modify the children or their transforms.
      procedure  recalculateLocalAabb           ;virtual;
      procedure  setLocalScaling                (const scaling: btVector3); override;
      function   getLocalScaling                : btVector3; override;
      procedure  calculateLocalInertia          (const mass: btScalar; var inertia: btVector3); override;
      procedure  setMargin                      (const margin: btScalar); override;
      function   getMargin                      : btScalar; override;
      function   getName                        :String;
      function   getDynamicAabbTree             :btDbvt;
      procedure  createAabbTreeFromChildren     ;
    ///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
    ///and the center of mass to the current coordinate system. "masses" points to an array of masses of the children. The resulting transform
    ///"principal" has to be applied inversely to all children transforms in order for the local coordinate system of the compound
    ///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
    ///of the collision object by the principal transform.
      procedure   calculatePrincipalAxisTransform(const masses:PbtScalar;const  principal:btTransform;var inertia:btVector3);
      function    getUpdateRevision              :integer;
    end;

    btConcaveShape=class(btCollisionShape)
    protected
      m_collisionMargin:btScalar;
    public
      procedure   processAllTriangles(const callback:btTriangleCallback;const  aabbMin,aabbMax:btVector3);virtual;abstract;
      function    getMargin:btScalar;override;
      procedure   setMargin(const collisionMargin:btScalar);override;
    end;

    { btEmptyShape }

    btEmptyShape=class(btConcaveShape)
    protected
      m_localScaling : btVector3;
    public
      constructor Create;
      procedure   getAabb          (const t: btTransform; out aabbMin, aabbMax: btVector3); override;
      procedure   setLocalScaling  (const scaling:btVector3);override;
      function    getLocalScaling  :btVector3;override;
    end;


    ///	The btStridingMeshInterface is the interface class for high performance generic access to triangle meshes, used in combination with btBvhTriangleMeshShape and some other collision shapes.
    /// Using index striding of 3*sizeof(integer) it can use triangle arrays, using index striding of 1*sizeof(integer) it can handle triangle strips.
    /// It allows for sharing graphics and collision meshes. Also it provides locking/unlocking of graphics meshes that are in gpu memory.

    { btStridingMeshInterface }

    btStridingMeshInterface=class
    protected
      m_scaling : btVector3;
    public
      constructor Create;
      {$HINTS OFF}
      procedure   InternalProcessAllTriangles (const callback:btInternalTriangleIndexCallback;const aabbMin,aabbMax:btVector3);virtual;
      {$HINTS ON}
      procedure   calculateAabbBruteForce     (var aabbMin,aabbMax:btVector3);                ///brute force method to calculate aabb
                /// get read and write access to a subpart of a triangle mesh
                /// this subpart has a continuous array of vertices and indices
                /// in this way the mesh can be handled as chunks of memory with striding
                /// very similar to OpenGL vertexarray support
                /// make a call to unLockVertexBase when the read and write access is finished
      procedure getLockedVertexIndexBase(var vertexbase:Pointer;var numverts:integer;var typ:PHY_ScalarType; var stride:integer;var indexbase:Pointer;
                                         var indexstride,numfaces:integer;var indicestype:PHY_ScalarType;const subpart:integer=0);virtual;abstract;
      procedure getLockedReadOnlyVertexIndexBase(var vertexbase:Pointer;var numverts:integer;var typ:PHY_ScalarType; var stride:integer;var indexbase:Pointer;
                                         var indexstride,numfaces:integer;var indicestype:PHY_ScalarType;const subpart:integer=0);virtual;abstract;

                /// unLockVertexBase finishes the access to a subpart of the triangle mesh
                /// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
      procedure unLockVertexBase(const subpart:integer);virtual;abstract;
      procedure unLockReadOnlyVertexBase(const subpart:integer);virtual;abstract;
                /// getNumSubParts returns the number of seperate subparts
                /// each subpart has a continuous array of vertices and indices
      function  getNumSubParts:integer;virtual;abstract;
      procedure preallocateVertices(const numverts:integer);virtual;abstract;
      procedure preallocateIndices (const numindices:integer);virtual;abstract;
      function  hasPremadeAabb:boolean;virtual;
      procedure setPremadeAabb(const aabbMin,aabbMax:btVector3);virtual;
      procedure getPremadeAabb(var  aabbMin,aabbMax :btVector3);virtual;
      function  getScaling:btVector3;
      function  getScalingP:PbtVector3;
      procedure setScaling(const scaling:btVector3);
    end;

    btIndexedMesh=record
      m_numTriangles        : integer;
      m_triangleIndexBase   : Pointer;
      m_triangleIndexStride : integer;
      m_numVertices         : integer;
      m_vertexBase          : pointer;
      m_vertexStride        : Integer;
      m_indexType           : PHY_ScalarType;
      m_vertexType          : PHY_ScalarType;
    end;

    PbtIndexedMesh=^btIndexedMesh;

    btIndexedMeshArray=specialize FOS_GenericAlignedArray<btIndexedMesh>;

   //TODO Inner Object ALIGNMENT !! 16 / check and test

    { btTriangleIndexVertexArray }

    btTriangleIndexVertexArray =class(btStridingMeshInterface)
    protected
    	m_indexedMeshes      : btIndexedMeshArray;
    	m_hasAabb            : Boolean; // using int instead of bool to maintain alignment (FOS wants bool)
        m_aabbMin            : btVector3;
        m_aabbMax            : btVector3;
    public
        //just to be backwards compatible
        constructor   Create                   (const numTriangles:Integer;const triangleIndexBase:PInteger;const triangleIndexStride:Integer;const numVertices:Integer;const vertexBase:PbtScalar;const vertexStride:integer);
        destructor    Destroy                  ;override;
        procedure     addIndexedMesh           (const mesh:btIndexedMesh;const indexType:PHY_ScalarType=PHY_INTEGER);
        procedure     getLockedVertexIndexBase(var vertexbase:Pointer;var numverts:integer;var typ:PHY_ScalarType; var vertexstride:integer;var indexbase:Pointer;
                                         var indexstride,numfaces:integer;var indicestype:PHY_ScalarType;const subpart:integer=0);override;
        procedure     getLockedReadOnlyVertexIndexBase(var vertexbase:Pointer;var numverts:integer;var typ:PHY_ScalarType; var stride:integer;var indexbase:Pointer;
                                         var indexstride,numfaces:integer;var indicestype:PHY_ScalarType;const subpart:integer=0);override;

        procedure     unLockVertexBase         (const subpart:integer);override;
        procedure     unLockReadOnlyVertexBase (const subpart:integer);override;
        function      getNumSubParts           : integer; override;
        function      getIndexedMeshArray      : btIndexedMeshArray;
        procedure     preallocateVertices      (const numverts: integer); override;
        procedure     preallocateIndices       (const numindices: integer); override;
        function      hasPremadeAabb           : boolean; override;
        procedure     setPremadeAabb           (const aabbMin, aabbMax: btVector3); override;
        procedure     getPremadeAabb           (var aabbMin, aabbMax: btVector3); override;
    end;


    /// The btConvexShape is an abstract shape interface, implemented by all convex shapes such as btBoxShape, btConvexHullShape etc.
    /// It describes general convex shapes using the localGetSupportingVertex interface, used by collision detectors such as btGjkPairDetector.
    //notice that the vectors should be unit length
    ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version

    { btConvexShape }
    btConvexGetSupportingVertexFunc = function(const vec:btVector3):btVector3 of object;

    btConvexShape =class(btCollisionShape)
    public
        function  localGetSupportingVertex                          (const vec:btVector3):btVector3; virtual;abstract;
        function  localGetSupportingVertexWithoutMargin             (const vec:btVector3):btVector3; virtual;abstract;
        function  localGetSupportVertexNonVirtual                   (const localdir:btVector3):btVector3; // FOS HH: UURGS?
        function  localGetSupportVertexWithoutMarginNonVirtual      (const localdir:btVector3):btVector3; // FOS HH: UURGS?
        function  getMarginNonVirtual                               :btScalar;
        procedure getAabbNonVirtual                                 (const t:btTransform;var aabbMin,aabbMax:btVector3);
        procedure batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors:PbtVector3;const supportVerticesOut:PbtVector3;const numVectors:Integer);virtual;abstract;
        procedure getAabbSlow                                       (const t:btTransform;out aabbMin,aabbMax:btVector3);virtual;abstract;
        function  getNumPreferredPenetrationDirections              :integer;virtual;abstract;
        procedure getPreferredPenetrationDirection                  (const index:integer;var penetrationVector:btVector3);virtual;abstract;
    end;

    ///The btShapeHull class takes a btConvexShape, builds a simplified convex hull using btConvexHull and provides triangle indices and vertices.
    ///It can be useful for to simplify a complex convex object and for visualization of a non-polyhedral convex object.
    ///It approximates the convex hull using the supporting vertex of 42 directions.

    { btShapeHull }

    btShapeHull = class
    protected
      sUnitSpherePoints : Array [0..NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2] of btVector3;//static;
      m_vertices   : btFOSAlignedVectorArray;
      m_indices    : btFOSAlignedCardinals;
      m_numIndices : Cardinal;
      m_shape      : btConvexShape;
      function     getUnitSpherePoints : PbtVector3;
      procedure    _StaticInit;
    public
      constructor Create          (const shape : btConvexShape);
      destructor  Destroy         ;override;
      function  buildHull         (const margin : btScalar):boolean;
      function  numTriangles      : integer;
      function  numVertices       : integer;
      function  numIndices        : integer;
      function  getVertexPointer  : PbtVector3;
      function  getVertexPtrIDX   (const n:cardinal): PbtVector3;
      function  getIndexPointer   : PbtCardinal16;
    end;
//    PbtShapeHull = ^btShapeHull;


    ///The btUniformScalingShape allows to re-use uniform scaled instances of btConvexShape in a memory efficient way.
    ///Istead of using btUniformScalingShape, it is better to use the non-uniform setLocalScaling method on convex shapes that implement it.

    { btUniformScalingShape }

    btUniformScalingShape=class(btConvexShape)
      m_childConvexShape     : btConvexShape;
      m_uniformScalingFactor : btScalar;
    public
      constructor Create                                            (const convexChildShape:btConvexShape;const uniformScalingFactor:btScalar);
      function    localGetSupportingVertexWithoutMargin             (const vec: btVector3): btVector3; override;
      function    localGetSupportingVertex                          (const vec: btVector3): btVector3; override;
      procedure   batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors: PbtVector3; const supportVerticesOut: PbtVector3;const numVectors: Integer); override;
      procedure   calculateLocalInertia                             (const mass:btScalar;var inertia:btVector3);override;
      function    getUniformScalingFactor                           :btScalar;
      function    getChildShape                                     :btConvexShape;
      function    getName                                           :string;virtual;
      ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
      procedure   getAabb(const t: btTransform; out aabbMin, aabbMax: btVector3); override;
      procedure   getAabbSlow(const t: btTransform; out aabbMin, aabbMax: btVector3);override;
      procedure   setLocalScaling(const scaling: btVector3); override;
      function    getLocalScaling: btVector3; override;
      procedure   setMargin(const margin: btScalar); override;
      function    getMargin: btScalar; override;
      function    getNumPreferredPenetrationDirections: integer; override;
      procedure   getPreferredPenetrationDirection(const index: integer; var penetrationVector: btVector3); override;
    end;




    { btConvexInternalShape }

    btConvexInternalShape=class(btConvexShape)
    protected
        //local scaling. collisionMargin is not scaled !
        m_localScaling            : btVector3;
        m_implicitShapeDimensions : btVector3;
        m_collisionMargin         : btScalar;
        m_padding                 : btScalar;
    public
        constructor  Create;virtual;
        function     localGetSupportingVertex    (const vec:btVector3):btVector3;override;
        function     getImplicitShapeDimensions  :btVector3;
        ///warning: use setImplicitShapeDimensions with care
        ///changing a collision shape while the body is in the world is not recommended,
        ///it is best to remove the body from the world, then make the change, and re-add it
        ///alternatively flush the contact points, see documentation for 'cleanProxyFromPairs'
        procedure    setImplicitShapeDimensions (const dimensions:btVector3);
        ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
        procedure    getAabb                    (const t:btTransform;out aabbMin,aabbMax:btVector3);override;
        procedure    getAabbSlow                (const t:btTransform;out aabbMin,aabbMax:btVector3);override;
        procedure    setLocalScaling            (const scaling:btVector3);override;
        function     getLocalScaling            :btVector3;override;
        function     getLocalScalingNV          :btVector3;
        procedure    setMargin                  (const margin:btScalar);override;
        function     getMargin                  :btScalar;override;
        function     getMarginNV                :btScalar;
        function     getNumPreferredPenetrationDirections  :integer;override;
        procedure    getPreferredPenetrationDirection      (const index:integer;var penetrationVector:btVector3);override;
    end;


    /// The btMinkowskiSumShape is only for advanced users. This shape represents implicit based minkowski sum of two convex implicit shapes.

    { btMinkowskiSumShape }

    btMinkowskiSumShape =class(btConvexInternalShape)
      m_transA : btTransform;
      m_transB : btTransform;
      m_shapeA : btConvexShape;
      m_shapeB : btConvexShape;
    public
      constructor create   (const shapeA,shapeB : btConvexShape);reintroduce;
      function    localGetSupportingVertexWithoutMargin(const vec: btVector3): btVector3;  override;
      procedure   batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3;const numVectors: Integer); override;
      procedure   calculateLocalInertia(const mass: btScalar; var inertia: btVector3); override;
      procedure   setTransformA        (const trans:btTransform);
      procedure   setTransformB        (const trans:btTransform);
      function    getTransformA        :btTransform;
      function    getTransformB        :btTransform;
      function    getMargin            :btScalar; override;
      function    getShapeA            :btConvexShape;
      function    getShapeB            :btConvexShape;
      function    getName              :string;
    end;


    ///btConvexInternalAabbCachingShape adds local aabb caching for convex shapes, to avoid expensive bounding box calculations

    { btConvexInternalAabbCachingShape }

    btConvexInternalAabbCachingShape = class (btConvexInternalShape)
    private
        m_localAabbMin     : btVector3;
        m_localAabbMax     : btVector3;
        m_isLocalAabbValid : boolean;
    protected
        procedure setCachedLocalAabb (const aabbMin,aabbMax : btVector3);
        procedure getCachedLocalAabb (var   aabbMin,aabbMax : btVector3);
        procedure getNonvirtualAabb  (const trans:btTransform;out   aabbMin,aabbMax : btVector3;const margin:btScalar);FOS_INLINE;
    public
        constructor Create              ;override;
        procedure   setLocalScaling       (const scaling:btVector3);override;
        procedure   getAabb               (const t:btTransform;out aabbMin,aabbMax:btVector3);override;
        procedure   recalcLocalAabb       ;
    end;

    ///The btMultiSphereShape represents the convex hull of a collection of spheres. You can create special capsules or other smooth volumes.
    ///It is possible to animate the spheres for deformation, but call 'recalcLocalAabb' after changing any sphere position/radius

    { btMultiSphereShape }

    btMultiSphereShape = class(btConvexInternalAabbCachingShape)
    private
      m_localPositionArray : btFOSAlignedVectorArray;
      m_radiArray          : btFOSAlignedScalars;
    public
      constructor Create  (const positions:PbtVector3;const radi:PbtScalar;const numSpheres:integer);reintroduce;
      procedure   calculateLocalInertia(const mass: btScalar; var inertia: btVector3); override;
      /// btConvexShape Interface
      function    localGetSupportingVertexWithoutMargin(const vec0: btVector3): btVector3; override;
      procedure   batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer); override;
      function    getSphereCount : integer;
      function    getSpherePosition (const index:integer):btVector3;
      function    getSphereRadius   (const index:integer):btScalar;
      function    getName           :string;
    end;



    ///The btPolyhedralConvexShape is an internal interface class for polyhedral convex shapes.

    { btPolyhedralConvexShape }

    btPolyhedralConvexShape=class(btConvexInternalShape)
    public
        constructor Create;override;
        //brute force implementations
        function    localGetSupportingVertexWithoutMargin             (const vec0:btVector3):btVector3; override;
        procedure   batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors:PbtVector3;const supportVerticesOut:PbtVector3;const numVectors:Integer);override;
        procedure   calculateLocalInertia                             (const mass:btScalar;var inertia:btVector3);override;
        function    getNumVertices                                    :integer;virtual;abstract;
        function    getNumEdges                                       :integer;virtual;abstract;
        procedure   getEdge                                           (const i:integer;out pa,pb:btVector3);virtual;abstract;
        procedure   getVertex                                         (const i:integer;out vtx:btVector3);virtual;abstract;
        function    getNumPlanes                                      :integer;virtual;abstract;
        procedure   getPlane                                          (var planeNormal,planeSupport:btVector3;const i:integer);virtual;abstract;
        function    getIndex                                          (const i:integer):integer;virtual;abstract;
        function    isInside                                          (const pt:btVector3;const tolerance:btScalar):boolean;virtual;abstract;
   end;
    ///The btPolyhedralConvexAabbCachingShape adds aabb caching to the btPolyhedralConvexShape

   { btPolyhedralConvexAabbCachingShape }

    btPolyhedralConvexAabbCachingShape=class(btPolyhedralConvexShape)
    private
      m_localAabbMin     : btVector3;
      m_localAabbMax     : btVector3;
      m_isLocalAabbValid : boolean;
    protected
      procedure   setCachedLocalAabb (const aabbMin,aabbMax : btVector3);
      procedure   getCachedLocalAabb (var   aabbMin,aabbMax : btVector3);
    public
      constructor Create              ;override;
      procedure   getNonvirtualAabb   (const trans:btTransform;out   aabbMin,aabbMax : btVector3;const margin:btScalar);FOS_INLINE;
      procedure   setLocalScaling     (const scaling:btVector3);override;
      procedure   getAabb             (const t:btTransform;out aabbMin,aabbMax:btVector3);override;
      procedure   recalcLocalAabb     ;
    end;


    /// The btConvexTriangleMeshShape is a convex hull of a triangle mesh, but the performance is not as good as btConvexHullShape.
    /// A small benefit of this class is that it uses the btStridingMeshInterface, so you can avoid the duplication of the triangle mesh data. Nevertheless, most users should use the much better performing btConvexHullShape instead.

     { btConvexTriangleMeshShape }

     btConvexTriangleMeshShape = class(btPolyhedralConvexAabbCachingShape)
       m_stridingMesh : btStridingMeshInterface;
     public
       constructor Create                                            (const meshInterface : btStridingMeshInterface; const calcAabb : boolean = true);overload;
       function    getMeshInterface                                  : btStridingMeshInterface;
       function    localGetSupportingVertex                          (const vec: btVector3): btVector3; override;
       function    localGetSupportingVertexWithoutMargin             (const vec0: btVector3): btVector3; override;
       procedure   batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer); override;
       function    getName                                           : String;
       function    getNumVertices                                    : integer; override;
       function    getNumEdges                                       :integer;override;
       procedure   getEdge                                           (const i:integer;out pa,pb:btVector3);override;
       procedure   getVertex                                         (const i:integer;out vtx:btVector3);override;
       function    getNumPlanes                                      :integer;override;
       procedure   getPlane                                          (var planeNormal,planeSupport:btVector3;const i:integer);override;
       function    isInside                                          (const pt:btVector3;const tolerance:btScalar):boolean;override;
       procedure   setLocalScaling                                   (const scaling: btVector3); override;
       function    getLocalScaling                                   : btVector3; override;
        /////computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
        /////and the center of mass to the current coordinate system. A mass of 1 is assumed, for other masses just multiply the computed "inertia"
        /////by the mass. The resulting transform "principal" has to be applied inversely to the mesh in order for the local coordinate system of the
        /////shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
        /////of the collision object by the principal transform. This method also computes the volume of the convex mesh.
//        procedure  calculatePrincipalAxisTransform                   (const principal : btTransform ; var inertia : btVector3 ; var volume : btScalar);
    end;


    ///The btBU_Simplex1to4 implements tetrahedron, triangle, line, vertex collision shapes. In most cases it is better to use btConvexHullShape instead.

    { btBU_Simplex1to4 }

    btBU_Simplex1to4 =class(btPolyhedralConvexAabbCachingShape)
    protected
      m_numVertices : integer;
      m_vertices    : array [0..3] of btVector3;
    public
      constructor Create;override;
      constructor Create (const pt0 : btVector3);
      constructor Create (const pt0,pt1 : btVector3);
      constructor Create (const pt0,pt1,pt2 : btVector3);
      constructor Create (const pt0,pt1,pt2,pt3 : btVector3);
      procedure   Reset;
      procedure   getAabb   (const t: btTransform; out aabbMin, aabbMax: btVector3); override;
      procedure   addVertex (const pt:btVector3);
      //PolyhedralConvexShape interface
      function    getNumVertices: integer; override;
      function    getNumEdges: integer; override;
      procedure   getEdge(const i: integer; out pa, pb: btVector3); override;
      procedure   getVertex(const i: integer; out vtx: btVector3); override;
      function    getNumPlanes: integer; override;
      procedure   getPlane(var planeNormal, planeSupport: btVector3; const i: integer); override;
      function    getIndex(const i: integer): integer; override;
      function    isInside(const pt: btVector3; const tolerance: btScalar): boolean; override;
      function    getName:string;
    end;

    { btSphereShape }

    btSphereShape=class(btConvexInternalShape)
    public
      constructor Create (const radius:btScalar)                    ;reintroduce;
      function    localGetSupportingVertex                          (const vec:btVector3):btVector3; override;
      function    localGetSupportingVertexWithoutMargin             (const vec:btVector3):btVector3; override;
      ////notice that the vectors should be unit length
      procedure   batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors:PbtVector3;const supportVerticesOut:PbtVector3;const numVectors:Integer);override;
      procedure   getAabb                                           (const t:btTransform;out aabbMin,aabbMax:btVector3);override;
      procedure   calculateLocalInertia                             (const mass:btScalar;var inertia:btVector3);override;
      function    getRadius                                         :btscalar;
      procedure   setUnscalesRadius                                 (const radius:btScalar);
      function    getName                                           :String;
      procedure   setMargin                                         (const margin:btScalar);override;
      function    getMargin                                         :btScalar;override;
    end;

    ///The btBoxShape is a box primitive around the origin, its sides axis aligned with length specified by half extents, in local shape coordinates. When used as part of a btCollisionObject or btRigidBody it will be an oriented box in world space.

    { btBoxShape }

    btBoxShape=class(btPolyhedralConvexShape)
    private
     // m_boxHalfExtents1                    :btVector3; //use m_implicitShapeDimensions instead
    public
      constructor Create                                            (const boxHalfExtents:btVector3);reintroduce;
      function    getHalfExtentsWithMargin                          :btVector3;
      function    getHalfExtentsWithoutMargin                       :btVector3;
      function    localGetSupportingVertex                          (const vec:btVector3):btVector3; override;
      function    localGetSupportingVertexWithoutMargin             (const vec:btVector3):btVector3; override;
      procedure   batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors:PbtVector3;const supportVerticesOut:PbtVector3;const numVectors:Integer);override;
      procedure   setMargin                                         (const collisionmargin:btScalar);override;
      procedure   setLocalScaling                                   (const scaling:btVector3);override;
      procedure   getAabb                                           (const t:btTransform;out aabbMin,aabbMax:btVector3);override;
      procedure   calculateLocalInertia                             (const mass:btScalar;var inertia:btVector3);override;
      procedure   getPlane                                          (var planeNormal,planeSupport:btVector3;const i:integer);override;
      function    getNumPlanes                                      :integer;override;
      function    getNumVertices                                    :integer;override;
      function    getNumEdges                                       :integer;override;
      procedure   getVertex                                         (const i:integer;out vtx:btVector3);override;
      procedure   getPlaneEquation                                  (var plane:btVector4;const i:integer);virtual;
      procedure   getEdge                                           (const i:integer;out pa,pb:btVector3);override;
      function    isInside                                          (const pt:btVector3;const tolerance:btScalar):boolean;override;
      function    getName                                           :String;virtual;
      function    getNumPreferredPenetrationDirections              :integer;override;
      procedure   getPreferredPenetrationDirection                  (const index:integer;var penetrationVector:btVector3);override;
      function    getIndex                                          (const i:integer):integer;override;
    end;

    { btCylinderShape }

    btCylinderShape=class(btConvexInternalShape)
    protected
      m_upAxis   : integer;
    public
      function    getHalfExtentsWithMargin                          :btVector3;
      function    getHalfExtentsWithoutMargin                       :btVector3;
      constructor Create                                            (const HalfExtents:btVector3);reintroduce;
      procedure   getAabb                                           (const t:btTransform;out aabbMin,aabbMax:btVector3);override;
      function    localGetSupportingVertexWithoutMargin             (const vec:btVector3):btVector3; override;
      procedure   batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors:PbtVector3;const supportVerticesOut:PbtVector3;const numVectors:Integer);override;
      procedure   setMargin                                         (const collisionmargin:btScalar);override;
      function    localGetSupportingVertex                          (const vec:btVector3):btVector3; override;
      //use box inertia
      procedure   calculateLocalInertia                             (const mass:btScalar;var inertia:btVector3);override;
      function    getUpAxis                                         :integer;
      function    getRadius                                         :btScalar;
      //debugging
      function    getName                                           :string;virtual;
    end;

    { btCylinderShapeX }

    btCylinderShapeX=class(btCylinderShape)
    public
      constructor Create                                            (const HalfExtents:btVector3);reintroduce;
      function    localGetSupportingVertexWithoutMargin             (const vec:btVector3):btVector3; override;
      procedure   batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors:PbtVector3;const supportVerticesOut:PbtVector3;const numVectors:Integer);override;
      function    getName                                           :string;override;
      function    getRadius                                         :btScalar;
    end;

    { btCylinderShapeZ }

    btCylinderShapeZ=class(btCylinderShape)
    public
      constructor Create                                            (const HalfExtents:btVector3);reintroduce;
      function    localGetSupportingVertexWithoutMargin             (const vec:btVector3):btVector3; override;
      procedure   batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors:PbtVector3;const supportVerticesOut:PbtVector3;const numVectors:Integer);override;
      function    getName                                           :string;override;
      function    getRadius                                         :btScalar;
    end;

    { btTriangleShape }

    btTriangleShape=class(btPolyhedralConvexShape)
    private
       m_vertices1:Array [0..2] of btVector3;
    public
       function    getNumVertices                                    :integer;override;
       function    getVertexPtr                                      (const idx:integer):PbtVector3;
       procedure   getVertex                                         (const i:integer;out vtx:btVector3);override;
       function    getNumEdges                                       :integer;override;
       procedure   getEdge                                           (const i:integer;out pa,pb:btVector3);override;
       procedure   getAabb                                           (const t:btTransform;out aabbMin,aabbMax:btVector3);override;
       function    localGetSupportingVertexWithoutMargin             (const vec:btVector3):btVector3; override;
       procedure   batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors:PbtVector3;const supportVerticesOut:PbtVector3;const numVectors:Integer);override;
       constructor Create                                            ;override;
       constructor Create                                            (const p0,p1,p2:btVector3);
       procedure   getPlane                                          (var planeNormal,planeSupport:btVector3;const i:integer);override;
       function    getNumPlanes                                      :integer;override;
       procedure   calcNormal                                        (var normal:btVector3);
       procedure   getPlaneEquation                                  (const i:integer;var planenormal,planesupport:btVector3);
       procedure   calculateLocalInertia                             (const mass:btScalar;var inertia:btVector3);override;
       function    isInside                                          (const pt:btVector3;const tolerance:btScalar):boolean;override;
       function    getName                                           :String;virtual;
       function    getNumPreferredPenetrationDirections              :integer;override;
       procedure   getPreferredPenetrationDirection                  (const index:integer;var penetrationVector:btVector3);override;
       function    getIndex                                          (const i:integer):integer;override;
    end;

    ///The btCapsuleShape represents a capsule around the Y axis, there is also the btCapsuleShapeX aligned around the X axis and btCapsuleShapeZ around the Z axis.
    ///The total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
    ///The btCapsuleShape is a convex hull of two spheres. The btMultiSphereShape is a more general collision shape that takes the convex hull of multiple sphere, so it can also represent a capsule when just using two spheres.

    { btCapsuleShape }

    btCapsuleShape=class(btConvexInternalShape)
    protected
      m_upAxis    : integer;
      procedure   Create;
    public
      constructor Create                                            (const radius,height : btScalar);
      procedure   calculateLocalInertia                             (const mass:btScalar;var inertia:btVector3);override;
      function    localGetSupportingVertexWithoutMargin             (const vec0: btVector3): btVector3; override;
      procedure   batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);override;
      procedure   setMargin                                         (const collisionmargin:btScalar);override;
      procedure   getAabb                                           (const t: btTransform; out aabbMin, aabbMax: btVector3); override;
      function    getName                                           :String;virtual;
      function    getUpAxis                                         :integer;
      function    getRadius                                         :btScalar;
      function    getHalfHeight                                     :btScalar;
      procedure   setLocalScaling                                   (const scaling: btVector3); override;
    end;
    ///btCapsuleShapeX represents a capsule around the Z axis
    ///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.

    { btCapsuleShapeX }

    btCapsuleShapeX=class(btCapsuleShape)
      constructor Create     (const radius,height : btScalar);reintroduce;
      function    getName    :String;override;
    end;
    ///btCapsuleShapeZ represents a capsule around the Z axis
    ///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.

    { btCapsuleShapeZ }

    btCapsuleShapeZ=class(btCapsuleShape)
      constructor Create     (const radius,height : btScalar);reintroduce;
      function    getName    :String;override;
    end;



    ///The btConeShape implements a cone shape primitive, centered around the origin and aligned with the Y axis. The btConeShapeX is aligned around the X axis and btConeShapeZ around the Z axis.

    { btConeShape }

    btConeShape=class(btConvexInternalShape)
    private
      m_sinAngle,
      m_radius,
      m_height       :btScalar;
      m_coneIndices  : Array [0..2] of integer;
      function    coneLocalSupport   (const v:btVector3):btVector3;
    public
      constructor Create (const radius,height: btScalar)            ;reintroduce;
      function    localGetSupportingVertex                          (const vec:btVector3):btVector3; override;
      function    localGetSupportingVertexWithoutMargin             (const vec0: btVector3): btVector3; override;
      procedure   batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);override;
      function    getRadius                                         :btScalar;
      function    getHeight                                         :btScalar;
      procedure   calculateLocalInertia                             (const mass:btScalar;var inertia:btVector3);override;
      function    getName                                           :String;virtual;
      ///choose upAxis index
      procedure    setConeUpIndex                                   (const upIndex:Integer);
      function     getConeUpIndex                                   :Integer;
    end;

    ///btConeShape implements a Cone shape, around the X axis

    { btConeShapeX }

    btConeShapeX =class(btConeShape)
    public
      constructor Create (const radius,height: btScalar)            ;reintroduce;
    end;

    ///btConeShapeZ implements a Cone shape, around the Z axis

    { btConeShapeZ }

    btConeShapeZ =class(btConeShape)
    public
      constructor Create (const radius,height: btScalar)            ;reintroduce;
    end;

    ///The btStaticPlaneShape simulates an infinite non-moving (static) collision plane.

    { btStaticPlaneShape }

    btStaticPlaneShape = class (btConcaveShape)
    protected
      m_localAabbMin,
      m_localAabbMax,
      m_planeNormal,
      m_localScaling:     btVector3;
      m_planeConstant:    btScalar;
    public
      constructor Create                (const planeNormal:btVector3;const planeConstant:btScalar);
      procedure   getAabb               (const t: btTransform; out aabbMin, aabbMax: btVector3); override;
      procedure   processAllTriangles   (const callback: btTriangleCallback; const aabbMin, aabbMax: btVector3); override;
      procedure   calculateLocalInertia (const mass:btScalar;var inertia:btVector3);override;
      procedure   setLocalScaling       (const scaling:btVector3);override;
      function    getLocalScaling       :btVector3;override;
      function    getPlaneNormal        :btVector3;
      function    getPlaneConstant      :btScalar;
      function    getPlaneNormalP       :PbtVector3;FOS_INLINE;
      function    getPlaneConstantP     :PbtScalar;FOS_INLINE;
      function    getName               :String;
    end;

    ///The btConvexHullShape implements an implicit convex hull of an array of vertices.
    ///Bullet provides a general and fast collision detector for convex shapes based on GJK and EPA using localGetSupportingVertex.

    { btConvexHullShape }

    btConvexHullShape =class(btPolyhedralConvexAabbCachingShape)
    private
      m_unscaledPoints   : btFOSAlignedVectorArray;
    public
    ///this constructor optionally takes in a pointer to points. Each point is assumed to be 3 consecutive btScalar (x,y,z), the striding defines the number of bytes between each point, in memory.
    ///It is easier to not pass any points in the constructor, and just add one point at a time, using addPoint.
    ///btConvexHullShape make an internal copy of the points.
      constructor Create               (const points:PbtScalar=nil;const numPoints:Integer=0;const stride:integer=sizeof(btVector3));reintroduce;
      destructor  Destroy              ;override;
      procedure   addPoint             (const point:btVector3);
      function    getUnscaledPoints    : PbtVector3;
     ///getPoints is obsolete, please use getUnscaledPoints  { return getUnscaledPoints(); }
     function     getScaledPoint      (const i:integer):btVector3;FOS_INLINE;
     function     getNumPoints        :integer;FOS_INLINE;
     function     localGetSupportingVertex(const vec: btVector3): btVector3; override;
     function     localGetSupportingVertexWithoutMargin(const vec0: btVector3): btVector3; override;
     procedure    batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer); override;
    //debugging
     function     getName               :String;
     function     getNumVertices: integer; override;
     function     getNumEdges: integer; override;
     procedure    getEdge(const i: integer; out pa, pb: btVector3); override;
     procedure    getVertex(const i: integer; out vtx: btVector3); override;
     function     getNumPlanes: integer; override;
     procedure    getPlane(var planeNormal, planeSupport: btVector3; const i: integer); override;
     function     isInside(const pt: btVector3; const tolerance: btScalar): boolean; override;
     ///in case we receive negative scaling
     procedure    setLocalScaling(const scaling: btVector3); override;
     function     getIndex                                          (const i:integer):integer;override;
    end;

    ///The btTriangleMesh class is a convenience class derived from btTriangleIndexVertexArray, that provides storage for a concave triangle mesh. It can be used as data for the btBvhTriangleMeshShape.
    ///It allows either 32bit or 16bit indices, and 4 (x-y-z-w) or 3 (x-y-z) component vertices.
    ///If you want to share triangle/index data between graphics mesh and collision mesh (btBvhTriangleMeshShape), you can directly use btTriangleIndexVertexArray or derive your own class from btStridingMeshInterface.
    ///Performance of btTriangleMesh and btTriangleIndexVertexArray used in a btBvhTriangleMeshShape is the same.

    { btTriangleMesh }

    btTriangleMesh = class(btTriangleIndexVertexArray)
    private
      m_4componentVertices    : btFOSAlignedVectorArray;
      m_3componentVertices    : btFOSAlignedScalars;
      m_32bitIndices          : TFOS_AlignedCardinals;
      m_16bitIndices          : TFOS_AlignedWords;
      m_use32bitIndices       : boolean;
      m_use4componentVertices : boolean;
    public
      m_weldingThreshold                   : btScalar;
      constructor Create                   (const use32bitIndices:boolean=true;const use4componentVertices:boolean=true);
      function    getUse32bitIndices       : boolean;
      function    getUse4componentVertices : Boolean;
      ///By default addTriangle won't search for duplicate vertices, because the search is very slow for large triangle meshes.
      ///In general it is better to directly use btTriangleIndexVertexArray instead.
      procedure   addTriangle              (const vertex0,vertex1,vertex2:btVector3;const removeDuplicateVertices:boolean=false);
      function    getNumTriangles          : integer;
      ///findOrAddVertex is an internal method, use addTriangle instead
      function    findOrAddVertex          (const vertex:btVector3;const removeDuplicateVertices:boolean):integer;
      ///addIndex is an internal method, use addTriangle instead
      procedure   addIndex                 (const index:integer);
    end;

    ///The btTriangleMeshShape is an internal concave triangle mesh interface. Don't use this class directly, use btBvhTriangleMeshShape instead.

    { btTriangleMeshShape }

    btTriangleMeshShape = class(btConcaveShape)
    protected
      m_localAabbMin,
      m_localAabbMax      : btVector3;
      m_meshInterface     : btStridingMeshInterface;
    public
      ///btTriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class.
      ///Don't use btTriangleMeshShape but use btBvhTriangleMeshShape instead!
      constructor Create  (const meshInterface:btStridingMeshInterface);
      function    localGetSupportingVertex(const vec: btVector3): btVector3; virtual;
      function    localGetSupportingVertexWithoutMargin(const vec0: btVector3): btVector3; virtual;

      procedure   recalcLocalAabb       ;
      procedure   getAabb               (const t:btTransform;out aabbMin,aabbMax:btVector3);override;
      procedure   processAllTriangles   (const callback: btTriangleCallback; const aabbMin, aabbMax: btVector3); override;
      procedure   calculateLocalInertia (const mass:btScalar;var inertia:btVector3);override;
      procedure   setLocalScaling       (const scaling: btVector3); override;
      function    getLocalScaling       :btVector3;override;
      function    getMeshInterface      :btStridingMeshInterface;
      function    getLocalAabbMin       : btVector3;
      function    getName               :String;virtual;
    end;

    ///The btOptimizedBvh extends the btQuantizedBvh to create AABB tree for triangle meshes, through the btStridingMeshInterface.

    { btOptimizedBvh }

    btOptimizedBvh =class(btQuantizedBvh)
    public
      procedure   build          (const triangles:btStridingMeshInterface;const useQuantizedAabbCompression:Boolean;const bvhAabbMin,bvhAabbMax : btVector3);
      procedure   refit          (const triangles:btStridingMeshInterface;const AabbMin,AabbMax : btVector3);
      procedure   refitPartial   (const triangles:btStridingMeshInterface;const AabbMin,AabbMax : btVector3);
      procedure   updateBvhNodes (const triangles:btStridingMeshInterface;const firstNode,endNode,index : integer);
    end;


    { btTriangleInfo }

    btTriangleInfo=object
      m_flags            : integer;
      m_edgeV0V1Angle,
      m_edgeV1V2Angle,
      m_edgeV2V0Angle    : btScalar;
      procedure Init;
    end;


    btHashTriangleInfoArray   = specialize FOS_GenericAlignedArray<btTriangleInfo>;
    btInternalTriangleInfoMap = specialize FOS_GenericHashMap<btHashInt,btTriangleInfo,btHashIntArray,bthashTriangleInfoArray>;

     ///The btTriangleInfoMap stores edge angle information for some triangles. You can compute this information yourself or using btGenerateInternalEdgeInfo.

    { btTriangleInfoMap }

    btTriangleInfoMap = object(btInternalTriangleInfoMap)
      m_convexEpsilon          : btScalar; ///used to determine if an edge or contact normal is convex, using the dot product
      m_planarEpsilon          : btScalar; ///used to determine if a triangle edge is planar with zero angle
      m_equalVertexThreshold   : btScalar; ///used to compute connectivity: if the distance between two vertices is smaller than m_equalVertexThreshold, they are considered to be 'shared'
      m_edgeDistanceThreshold  : btScalar; ///used to determine edge contacts: if the closest distance between a contact point and an edge is smaller than this distance threshold it is considered to "hit the edge"
      m_zeroAreaThreshold      : btScalar; ///used to determine if a triangle is degenerate (length squared of cross product of 2 triangle edges < threshold)
      procedure                Init;
    end;

    ///The btBvhTriangleMeshShape is a static-triangle mesh shape with several optimizations, such as bounding volume hierarchy and cache friendly traversal for PlayStation 3 Cell SPU. It is recommended to enable useQuantizedAabbCompression for better memory usage.
    ///It takes a triangle mesh as input, for example a btTriangleMesh or btTriangleIndexVertexArray. The btBvhTriangleMeshShape class allows for triangle mesh deformations by a refit or partialRefit method.
    ///Instead of building the bounding volume hierarchy acceleration structure, it is also possible to serialize (save) and deserialize (load) the structure from disk.
    ///See Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.

    { btBvhTriangleMeshShape }

    btBvhTriangleMeshShape = class(btTriangleMeshShape)
      m_bvh                         : btOptimizedBvh;
      m_triangleInfoMap             : btTriangleInfoMap;
      m_useQuantizedAabbCompression : boolean;
      m_ownsBvh                     : boolean;
      m_pad                         : array [0..10] of boolean; ////need padding due to alignment //FOSCHECK
    public
      constructor                     Create;
      constructor Create              (const meshInterface:btStridingMeshInterface;const useQuantizedAabbCompression:boolean; const buildBvh:boolean=true);
      ///optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb
      constructor Create              (const meshInterface:btStridingMeshInterface;const useQuantizedAabbCompression:boolean;const bvhAabbMin,bvhAabbMax:btVector3;const buildBvh:boolean=true);
      destructor  Destroy             ;override;
      function    getOwnsBvh          : boolean;
      procedure   performRaycast      (const  callback:btTriangleCallback; const raySource,rayTarget:btVector3);
      procedure   performConvexcast   (const  callback:btTriangleCallback; const boxSource,boxTarget,boxMin,boxMax:btVector3);
      procedure   processAllTriangles (const callback: btTriangleCallback; const aabbMin, aabbMax: btVector3); override;
      procedure   refitTree           (const aabbMin, aabbMax: btVector3);
      /////for a fast incremental refit of parts of the tree. Note: the entire AABB of the tree will become more conservative, it never shrinks
      procedure   partialRefitTree    (const aabbMin, aabbMax: btVector3);
      function    getName             : String; override;
      procedure   setLocalScaling     (const scaling: btVector3); override;
      function    getOptimizedBvh     :btOptimizedBvh;
      procedure   setOptimizedBvh     (const bvh:btOptimizedBvh; const scaling:btVector3);  //localscaling= btVector3.InitSameS(1)
      procedure   buildOptimizedBvh   ;
      function    usesQuantizedAabbCompression:boolean;
      procedure   setTriangleInfoMap  (const triangleInfoMap:btTriangleInfoMap);
      function    getTriangleInfoMap  :btTriangleInfoMap;
   end;



    btConvexPointCloudShape=class(btConvexInternalShape)
      function getUnscaledPoints:PbtVector3;virtual;abstract; //TODO WORK OVER
      function getnumPoints:integer;virtual;abstract;
    end;


implementation

operator=(const c1, c2: btCompoundShapeChild) r:boolean;
begin
  result :=(c1.m_transform      = c2.m_transform) and
           (c1.m_childShape     = c2.m_childShape) and
           (c1.m_childShapeType = c2.m_childShapeType) and
           (c1.m_childMargin    = c2.m_childMargin);
end;

{ btInternalTriangleIndexCallback }

procedure btInternalTriangleIndexCallback.processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer);
begin
  abort; // FOS _ ONLY HINT REMOVAL - abstract class ?
  internalProcessTriangleIndex(triangle, partId, triangleIndex);
end;

{ btInternalTriangleIndexCallback }


{ btCollisionShape }

constructor btCollisionShape.Create;
begin
   m_shapeType   := INVALID_SHAPE_PROXYTYPE;
   m_userPointer := nil;
end;

procedure btCollisionShape.getBoundingSphere(var center: btVector3;  var radius: btScalar);
var tr:btTransform;
    aabbMin,aabbMax:btVector3;
begin
  tr.setIdentity;
  {$HINTS OFF}
  getAabb(tr,aabbMin,aabbMax);
  {$HINTS ON}
  radius := (aabbMax-aabbMin).length()*btScalar(0.5);
  center := (aabbMin+aabbMax)*btScalar(0.5);
end;

function btCollisionShape.getAngularMotionDisc: btScalar;
var   center:btVector3;
      disc:btScalar;
begin
  ///@todo cache this value, to improve performance
  {$HINTS OFF}
  getBoundingSphere(center,disc);
  {$HINTS ON}
  disc := disc + (center).length;
  result:=disc;
end;

function btCollisionShape.getContactBreakingThreshold(const defaultContactThresholdFactor: btScalar): btScalar;
begin
  Result:=getAngularMotionDisc*defaultContactThresholdFactor;
end;

procedure btCollisionShape.calculateTemporalAabb(const curTrans: btTransform;
                                                 const linvel, angvel: btVector3; timeStep: btScalar; out temporalAabbMin,
                                                 temporalAabbMax: btVector3);
var
    temporalAabbMaxx,temporalAabbMaxy,temporalAabbMaxz,temporalAabbMinx,temporalAabbMiny,temporalAabbMinz,angularMotion:btScalar;
    linMotion,angularMotion3d:btVector3;
begin
  //start with static aabb
  getAabb(curTrans,temporalAabbMin,temporalAabbMax);

  temporalAabbMaxx := temporalAabbMax.getX;
  temporalAabbMaxy := temporalAabbMax.getY;
  temporalAabbMaxz := temporalAabbMax.getZ;
  temporalAabbMinx := temporalAabbMin.getX;
  temporalAabbMiny := temporalAabbMin.getY;
  temporalAabbMinz := temporalAabbMin.getZ;

  // add linear motion
  linMotion := linvel*timeStep;
  //todo: simd would have a vector max/min operation, instead of per-element access
  if (linMotion.getX > 0) then begin
  	temporalAabbMaxx += linMotion.getx;
  end else begin
  	temporalAabbMinx += linMotion.getx;
  end;
  if (linMotion.gety > 0) then begin
  	temporalAabbMaxy += linMotion.gety;
  end else begin
  	temporalAabbMiny += linMotion.gety;
  end;
  if (linMotion.getz > 0) then begin
  	temporalAabbMaxz += linMotion.getz;
  end else begin
  	temporalAabbMinz += linMotion.getz;
  end;

  //add conservative angular motion
  angularMotion   := angvel.length * getAngularMotionDisc * timeStep;
  angularMotion3d.Init(angularMotion,angularMotion,angularMotion);
  temporalAabbMin.Init(temporalAabbMinx,temporalAabbMiny,temporalAabbMinz);
  temporalAabbMax.Init(temporalAabbMaxx,temporalAabbMaxy,temporalAabbMaxz);

  temporalAabbMin -= angularMotion3d;
  temporalAabbMax += angularMotion3d;
end;

function btCollisionShape.isPolyhedral: boolean;FOS_INLINE;
begin
  result:=btBroadphaseProxy.isPolyhedral(m_shapeType);
end;

function btCollisionShape.isConvex2d: boolean;FOS_INLINE;
begin
  result:=btBroadphaseProxy.isConvex2D(m_shapeType);
end;

function btCollisionShape.isConvex: boolean;FOS_INLINE;
begin
  result:=btBroadphaseProxy.isConvex(m_shapeType);
end;

function btCollisionShape.isNonMoving: boolean;FOS_INLINE;
begin
  result:=btBroadphaseProxy.isNonMoving(m_shapeType);
end;

function btCollisionShape.isConcave: boolean;FOS_INLINE;
begin
  result:=btBroadphaseProxy.isConcave(m_shapeType);
end;

function btCollisionShape.isCompound: boolean;FOS_INLINE;
begin
  result:=btBroadphaseProxy.isCompound(m_shapeType);
end;

function btCollisionShape.isSoftBody: boolean;FOS_INLINE;
begin
  result:=btBroadphaseProxy.isSoftBody(m_shapeType);
end;

function btCollisionShape.isInfinite: boolean;FOS_INLINE;
begin
  result:=btBroadphaseProxy.isInfinite(m_shapeType);
end;

function btCollisionShape.getShapeType: TbtBroadphaseNativeTypes;
begin
  result:=m_shapeType;
end;

procedure btCollisionShape.setUserPointer(const ptr: Pointer);
begin
  m_userPointer:=ptr;
end;

function btCollisionShape.getUserPointer: Pointer;
begin
  result:=m_userPointer;
end;

{ btConcaveShape }

function btConcaveShape.getMargin: btScalar;
begin
  Result:=m_collisionMargin;
end;

procedure btConcaveShape.setMargin(const collisionMargin: btScalar);
begin
 m_collisionMargin:=collisionMargin;
end;

{ btStridingMeshInterface }

constructor btStridingMeshInterface.Create;
begin
  m_scaling.InitSame(1);
end;

{$HINTS OFF}
procedure btStridingMeshInterface.InternalProcessAllTriangles(const callback: btInternalTriangleIndexCallback; const aabbMin,aabbMax: btVector3);
var numtotalphysicsverts,part,graphicssubparts,indexstride,stride,numverts,numtriangles,gfxindex:integer;
    vertexbase,indexbase:Pointer;
    typ,gfxindextype:PHY_ScalarType;
    triangle:Array [0..2] of btVector3;
    meshScaling:btVector3;
 {$HINTS OFF} //FOSTODO - find a cleaner hint solution
    procedure float_integer_inner;FOS_INLINE;
    var tri_indices:PCardinal;
        graphicsbase :PSingle;
    begin
      tri_indices  := PCardinal(indexbase)+gfxindex*indexstride;
      graphicsbase := PSingle  (vertexbase)+tri_indices[0]*stride;
      triangle[0].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      graphicsbase := PSingle  (vertexbase)+tri_indices[1]*stride;
      triangle[1].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      graphicsbase := PSingle  (vertexbase)+tri_indices[2]*stride;
      triangle[2].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      callback.internalProcessTriangleIndex(triangle,part,gfxindex);
    end;

    procedure float_smallint_inner;FOS_INLINE;
    var tri_indices:PSmallInt;
        graphicsbase :PSingle;
    begin
      tri_indices  := PSmallInt(indexbase)+gfxindex*indexstride;
      graphicsbase := PSingle  (vertexbase)+tri_indices[0]*stride;
      triangle[0].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      graphicsbase := PSingle  (vertexbase)+tri_indices[1]*stride;
      triangle[1].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      graphicsbase := PSingle  (vertexbase)+tri_indices[2]*stride;
      triangle[2].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      callback.internalProcessTriangleIndex(triangle,part,gfxindex);
    end;

    procedure double_integer_inner;FOS_INLINE;
    var tri_indices:PCardinal;
        graphicsbase:PDouble;
    begin
      tri_indices  := PCardinal(indexbase)+gfxindex*indexstride;
      graphicsbase := PDouble  (vertexbase)+tri_indices[0]*stride;
      triangle[0].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      graphicsbase := PDouble  (vertexbase)+tri_indices[1]*stride;
      triangle[1].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      graphicsbase := PDouble  (vertexbase)+tri_indices[2]*stride;
      triangle[2].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      callback.internalProcessTriangleIndex(triangle,part,gfxindex);
    end;

    procedure double_smallint_inner;FOS_INLINE;
    var tri_indices:PSmallInt;
        graphicsbase:PDouble;
    begin
      tri_indices  := PSmallInt(indexbase)+gfxindex*indexstride;
      graphicsbase := PDouble  (vertexbase)+tri_indices[0]*stride;
      triangle[0].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      graphicsbase := PDouble  (vertexbase)+tri_indices[1]*stride;
      triangle[1].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      graphicsbase := PDouble  (vertexbase)+tri_indices[2]*stride;
      triangle[2].Init(graphicsbase[0]*meshScaling.getX,graphicsbase[1]*meshScaling.getY,graphicsbase[2]*meshScaling.getZ);
      callback.internalProcessTriangleIndex(triangle,part,gfxindex);
    end;
begin
  numtotalphysicsverts := 0;
  graphicssubparts     := getNumSubParts;
  meshScaling          := getScaling;
  ///if the number of parts is big, the performance might drop due to the innerloop switch on indextype
  for part:=0 to graphicssubparts-1 do begin
  	getLockedReadOnlyVertexIndexBase(vertexbase,numverts,typ,stride,indexbase,indexstride,numtriangles,gfxindextype,part);
  	numtotalphysicsverts+=numtriangles*3; //upper bound
  	///unlike that developers want to pass in double-precision meshes in single-precision Bullet build
  	///so disable this feature by default
  	///see patch http://code.google.com/p/bullet/issues/detail?id=213
        case typ of
             PHY_FLOAT: begin
                case gfxindextype of
                  PHY_INTEGER: begin
                                 for gfxindex:=0 to numtriangles-1 do begin
                                    float_integer_inner;
                                 end;
                  end;
                  PHY_SHORT: begin
                                 for gfxindex:=0 to numtriangles-1 do begin
                                    float_smallint_inner;
                                 end;
                  end;
                  else begin
                     btAssert((gfxindextype = PHY_INTEGER) or (gfxindextype = PHY_SHORT));
                  end;
                end;
             end;
             PHY_DOUBLE: begin
                 case gfxindextype of
                   PHY_INTEGER: begin
                                  for gfxindex:=0 to numtriangles-1 do begin
                                     double_integer_inner;
                                  end;
                   end;
                   PHY_SHORT: begin
                                  for gfxindex:=0 to numtriangles-1 do begin
                                     double_smallint_inner;
                                  end;
                   end;
                   else begin
                     btAssert((gfxindextype = PHY_INTEGER) or (gfxindextype = PHY_SHORT));
                   end;
                 end;
             end;
        else begin
          btAssert((typ = PHY_FLOAT) or (typ = PHY_DOUBLE));
        end;
     end;
     unLockReadOnlyVertexBase(part);
  end;
end;
{$HINTS ON}

type

    { AabbCalculationCallback }

    { TAabbCalculationCallback }

    TAabbCalculationCallback=class(btInternalTriangleIndexCallback)
    private
       m_aabbMin,m_aabbMax:btVector3;
    public
       constructor create;
       procedure   internalProcessTriangleIndex(const triangle:PbtVector3;const  partId,triangleIndex:integer);override;
    end;

{ AabbCalculationCallback }

constructor TAabbCalculationCallback.create;
begin
  m_aabbMin.Init(btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT));
  m_aabbMax.Init(btScalar(-BT_LARGE_FLOAT),btScalar(-BT_LARGE_FLOAT),btScalar(-BT_LARGE_FLOAT));
end;


{$HINTS OFF}
procedure TAabbCalculationCallback.internalProcessTriangleIndex(const triangle: PbtVector3; const partId, triangleIndex: integer);
begin
  m_aabbMin.setMin(triangle[0]);
  m_aabbMax.setMax(triangle[0]);
  m_aabbMin.setMin(triangle[1]);
  m_aabbMax.setMax(triangle[1]);
  m_aabbMin.setMin(triangle[2]);
  m_aabbMax.setMax(triangle[2]);
end;
{$HINTS ON}

procedure btStridingMeshInterface.calculateAabbBruteForce(var aabbMin,  aabbMax: btVector3);
var aabbCallback:TAabbCalculationCallback;
begin
  //first calculate the total aabb for all triangles
  aabbCallback := TAabbCalculationCallback.create;
  aabbMin.InitSame(-BT_LARGE_FLOAT);
  aabbMax.InitSame(BT_LARGE_FLOAT);
  {$HINTS OFF}
  InternalProcessAllTriangles(aabbCallback,aabbMin,aabbMax);
  {$HINTS ON}
  aabbMin := aabbCallback.m_aabbMin;
  aabbMax := aabbCallback.m_aabbMax;
  aabbCallback.Free;
end;

function btStridingMeshInterface.hasPremadeAabb: boolean;
begin
  result:=false;
end;
{$HINTS OFF}
procedure btStridingMeshInterface.setPremadeAabb(const aabbMin,aabbMax: btVector3);
begin
  //ignore;
end;

procedure btStridingMeshInterface.getPremadeAabb(var aabbMin, aabbMax: btVector3);
begin
  //ignore
end;
{$HINTS ON}

function btStridingMeshInterface.getScaling: btVector3;
begin
 result:=m_scaling;
end;

function btStridingMeshInterface.getScalingP: PbtVector3;
begin
  result:=@m_scaling;
end;

procedure btStridingMeshInterface.setScaling(const scaling: btVector3);
begin
  m_scaling:=scaling;
end;

{ btTriangleIndexVertexArray }

constructor btTriangleIndexVertexArray.Create(const numTriangles: Integer;
  const triangleIndexBase: PInteger; const triangleIndexStride: Integer;
  const numVertices: Integer; const vertexBase: PbtScalar;
  const vertexStride: integer);
var mesh:btIndexedMesh;
begin
  inherited Create;
  m_indexedMeshes := btIndexedMeshArray.create;
  mesh.m_numTriangles        := numTriangles;
  mesh.m_triangleIndexBase   := triangleIndexBase;
  mesh.m_triangleIndexStride := triangleIndexStride;
  mesh.m_numVertices         := numVertices;
  mesh.m_vertexBase          := vertexBase;
  mesh.m_vertexStride        := vertexStride;
  addIndexedMesh(mesh);
end;

destructor btTriangleIndexVertexArray.Destroy;
begin
  m_indexedMeshes.Free;
  inherited Destroy;
end;

procedure btTriangleIndexVertexArray.addIndexedMesh(const mesh: btIndexedMesh;const indexType: PHY_ScalarType);
var idx:integer;
begin
  idx:=m_indexedMeshes.push_back(mesh);
  m_indexedMeshes.A[idx]^.m_indexType:=indexType;
  m_indexedMeshes.A[idx]^.m_vertexType:=PHY_FLOAT;
end;

procedure btTriangleIndexVertexArray.getLockedVertexIndexBase(
  var vertexbase: Pointer; var numverts: integer; var typ: PHY_ScalarType;
  var vertexstride: integer; var indexbase: Pointer; var indexstride,
  numfaces: integer; var indicestype: PHY_ScalarType; const subpart: integer);

var mesh:PbtIndexedMesh;
begin
  btAssert(subpart<getNumSubParts);
  mesh         := m_indexedMeshes.A[subpart];
  numverts     := mesh^.m_numVertices;
  vertexbase   := mesh^.m_vertexBase;
  typ          := mesh^.m_vertexType;
  vertexStride := mesh^.m_vertexStride;
  numfaces     := mesh^.m_numTriangles;
  indexbase    := mesh^.m_triangleIndexBase;
  indexstride  := mesh^.m_triangleIndexStride;
  indicestype  := mesh^.m_indexType;
end;

procedure btTriangleIndexVertexArray.getLockedReadOnlyVertexIndexBase(
  var vertexbase: Pointer; var numverts: integer; var typ: PHY_ScalarType;
  var stride: integer; var indexbase: Pointer; var indexstride,
  numfaces: integer; var indicestype: PHY_ScalarType; const subpart: integer);
begin
  getLockedVertexIndexBase(vertexbase,numverts,typ,stride,indexbase,indexstride,numfaces,indicestype,subpart);
end;

{$HINTS OFF}
procedure btTriangleIndexVertexArray.unLockVertexBase(const subpart: integer);
begin
  //ignore
end;

procedure btTriangleIndexVertexArray.unLockReadOnlyVertexBase(const subpart: integer);
begin
  //ignore
end;
{$HINTS ON}

function btTriangleIndexVertexArray.getNumSubParts: integer;
begin
  result:=m_indexedMeshes.len;
end;

function btTriangleIndexVertexArray.getIndexedMeshArray: btIndexedMeshArray;
begin
  result:=m_indexedMeshes;
end;

{$HINTS OFF}
procedure btTriangleIndexVertexArray.preallocateVertices(const numverts: integer);
begin
  //ignore
end;

procedure btTriangleIndexVertexArray.preallocateIndices(const numindices: integer);
begin
  //ignore
end;
{$HINTS ON}

function btTriangleIndexVertexArray.hasPremadeAabb: boolean;
begin
  result:=m_hasAabb;
end;

procedure btTriangleIndexVertexArray.setPremadeAabb(const aabbMin, aabbMax: btVector3);
begin
  m_aabbMin := aabbMin;
  m_aabbMax := aabbMax;
  m_hasAabb := true; // this is intentionally an int see notes in header (FOS: ignore comment)
end;

procedure btTriangleIndexVertexArray.getPremadeAabb(var aabbMin, aabbMax: btVector3);
begin
  aabbMin := m_aabbMin;
  aabbMax := m_aabbMax;
end;

{ btConvexShape }

function btConvexShape.localGetSupportVertexNonVirtual(const localdir: btVector3): btVector3;
var localDirNorm : btVector3;
begin
  localDirNorm := localDir;
  localdir.length2NormCheck;
  localDirNorm.normalize;
  result:=localGetSupportVertexWithoutMarginNonVirtual(localDirNorm)+(localDirNorm*getMarginNonVirtual);
end;


function convexHullSupport (const localDirOrg:btVector3; const points:PbtVector3;const numPoints:integer; const localScaling:btVector3):btVector3;
var  vec          : btVector3;
    newDot,maxDot : btScalar;
    ptIndex       : Integer;
    i             : Integer;
begin
  vec     := localDirOrg * localScaling;
  maxDot  := -BT_LARGE_FLOAT;
  ptIndex := -1;
  for i:= 0 to numPoints-1 do begin
    newDot := vec.dot(points[i]);
    if (newDot > maxDot) then begin
      maxDot  := newDot;
      ptIndex := i;
    end;
  end;
  btAssert(ptIndex >= 0);
  result := points[ptIndex] * localScaling;
end;



function btConvexShape.localGetSupportVertexWithoutMarginNonVirtual(const localdir: btVector3): btVector3;
  procedure _box;
  var convexShape:btBoxShape;
      halfExtents:btVector3;
  begin
    convexShape := btBoxShape(self);
    halfExtents := convexShape.getImplicitShapeDimensions;
    result.init(btFsel(localdir._x^, halfExtents._x^, -halfExtents._x^),
                btFsel(localdir._y^, halfExtents._y^, -halfExtents._y^),
                btFsel(localdir._z^, halfExtents._z^, -halfExtents._z^));
  end;
  procedure _triangle;
  var triangleShape : btTriangleShape;
      dir,dots,sup  : btVector3;
      vertices      : PbtVector3;
  begin
    triangleShape := btTriangleShape(self);
    dir.Init(localDir._X^,localDir._Y^,localDir._Z^);
    vertices      := @triangleShape.m_vertices1[0];
    dots.init(dir.dot(vertices[0]), dir.dot(vertices[1]), dir.dot(vertices[2]));
    sup           := vertices[dots.maxAxis];
    result.init(sup._X^,sup._Y^,sup._Z^);
  end;
  procedure _cylinder;
  var cylShape          : btCylinderShape;
      halfExtents,v,tmp : btVector3;
      cylinderUpAxis    : integer;
      XX,YY,ZZ          : integer;
      radius,halfHeight : btScalar;
      d,s               : btScalar;
  begin
    cylShape    := btCylinderShape(self); //mapping of halfextents/dimension onto radius/height depends on how cylinder local orientation is (upAxis)
    halfExtents := cylShape.getImplicitShapeDimensions;
    v.Init(localDir._X^,localDir._Y^,localDir._Z^);
    cylinderUpAxis := cylShape.getUpAxis;
    XX:=1;YY:=0;ZZ:=2;
    case cylinderUpAxis of
      0: begin
            XX := 1; YY := 0; ZZ := 2;
         end;
      1: begin
            XX := 0; YY := 1; ZZ := 2;
         end;
      2: begin
            XX := 0; YY := 2; ZZ := 1;
         end;
      else btAssert(false);
    end;
    radius     := halfExtents[XX];
    halfHeight := halfExtents[cylinderUpAxis];
    s          := btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s <> 0) then begin
      d        := radius / s;
      tmp[XX]  := v[XX] * d;
      if v[YY] < 0.0 then begin
        tmp[YY]  := -halfHeight;
      end else begin
        tmp[YY]  :=  halfHeight;
      end;
      tmp[ZZ]  := v[ZZ] * d;
    end else begin
      tmp[XX] := radius;
      if v[YY] < 0.0 then begin
        tmp[YY] := -halfHeight;
      end else begin
        tmp[YY] :=  halfHeight;
      end;
      tmp[ZZ] := 0;
    end;
    result.init(tmp._X^,tmp._Y^,tmp._Z^);
  end;
  procedure _capsule;
  var capsuleShape    : btCapsuleShape;
      vec0,supVec,
      vec,vtx,pos     : btVector3;
      halfHeight,
      radius,lenSqr,
      rlen,maxDot,
      newDot          : btScalar;
      capsuleUpAxis   : integer;
  begin
    vec0.Init(localDir._X^,localDir._Y^,localDir._Z^);
    capsuleShape  := btCapsuleShape(self);
    halfHeight    := capsuleShape.getHalfHeight;
    capsuleUpAxis := capsuleShape.getUpAxis;
    radius        := capsuleShape.getRadius;
    supVec.init(0,0,0);
    maxDot        := -BT_LARGE_FLOAT;
    vec           := vec0;
    lenSqr        := vec.length2;
    if lenSqr < 0.0001 then begin
      vec.Init(1,0,0);
    end  else begin
      rlen := 1/btSqrt(lenSqr);
      vec *= rlen;
    end;
    pos.init(0,0,0);
    pos[capsuleUpAxis] := halfHeight;
    //vtx = pos +vec*(radius);
    vtx := pos+vec*capsuleShape.getLocalScalingNV*radius - vec*capsuleShape.getMarginNV;
    newDot := vec.dot(vtx);
    if newDot > maxDot then begin
      maxDot := newDot;
      supVec := vtx;
    end;
    pos.init(0,0,0);
    pos[capsuleUpAxis] := -halfHeight;
    //vtx = pos +vec*(radius);
    vtx := pos+vec*capsuleShape.getLocalScalingNV*radius - vec*capsuleShape.getMarginNV;
    newDot := vec.dot(vtx);
    if newDot > maxDot then begin
      maxDot := newDot;
      supVec := vtx;
    end;
    result.init(supVec._X^,supVec._Y^,supVec._Z^);
  end;
  procedure _cloudshape;
  var convexPointCloudShape : btConvexPointCloudShape;
      points                : PbtVector3;
      numPoints             : integer;
  begin
    convexPointCloudShape := btConvexPointCloudShape(self);
    points                := convexPointCloudShape.getUnscaledPoints;
    numPoints             := convexPointCloudShape.getNumPoints;
    result:=convexHullSupport(localDir, points, numPoints,convexPointCloudShape.getLocalScalingNV);
  end;
  procedure _hullshape;
  var convexHullShape : btConvexHullShape;
      points          : PbtVector3;
      numPoints       : integer;
  begin
     convexHullShape := btConvexHullShape(self);
     points          := convexHullShape.getUnscaledPoints;
     numPoints       := convexHullShape.getNumPoints;
    result:=convexHullSupport (localDir, points, numPoints,convexHullShape.getLocalScalingNV);
  end;
begin
  case m_shapeType of
    SPHERE_SHAPE_PROXYTYPE:             result.Init(0,0,0);
    BOX_SHAPE_PROXYTYPE:                _box;
    TRIANGLE_SHAPE_PROXYTYPE:           _triangle;
    CYLINDER_SHAPE_PROXYTYPE:           _cylinder;
    CAPSULE_SHAPE_PROXYTYPE:            _capsule;
    CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE: _cloudshape;
    CONVEX_HULL_SHAPE_PROXYTYPE:        _hullshape;
    else begin
      result:= self.localGetSupportingVertexWithoutMargin(localdir);
    end;
  end;
end;
// /* TODO: This should be bumped up to btCollisionShape () */
function btConvexShape.getMarginNonVirtual: btScalar;
  procedure _sphere;
  var  sphereShape:btSphereShape;
  begin
    sphereShape := btSphereShape(self);
    result:=sphereShape.getRadius;
  end;
  procedure _box;
  var convexShape:btBoxShape;
  begin
    convexShape := btBoxShape(self);
    result      := convexShape.getMarginNV;
  end;
  procedure _triangle;
  var triangleShape : btTriangleShape;
  begin
    triangleShape := btTriangleShape(self);
    result        := triangleShape.getMarginNV;
  end;
  procedure _cylinder;
  var cylShape  : btCylinderShape;
  begin
    cylShape    := btCylinderShape(self);
    result      := cylShape.getMarginNV;
  end;
  procedure _capsule;
  var capsuleShape    : btCapsuleShape;
  begin
    capsuleShape  := btCapsuleShape(self);
    result        := capsuleShape.getMarginNV;
  end;
  procedure _poly;
  var poly:btPolyhedralConvexShape;
  begin
    poly          := btPolyhedralConvexShape(self);
    result        := poly.getMarginNV;
  end;
begin
  case m_shapeType of
    SPHERE_SHAPE_PROXYTYPE:             _sphere;
    BOX_SHAPE_PROXYTYPE:                _box;
    TRIANGLE_SHAPE_PROXYTYPE:           _triangle;
    CYLINDER_SHAPE_PROXYTYPE:           _cylinder;
    CAPSULE_SHAPE_PROXYTYPE:            _capsule;
    CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE,
    CONVEX_HULL_SHAPE_PROXYTYPE:        _poly;
    else begin
      result:= self.getMargin;
    end;
  end;
end;

procedure btConvexShape.getAabbNonVirtual(const t: btTransform; var aabbMin, aabbMax: btVector3);
    procedure _sphere;
    var  sphereShape   : btSphereShape;
         radius,margin : btScalar;
         center        : PbtVector3;
         extent        : btVector3;
    begin
      sphereShape := btSphereShape(self);
      radius      := sphereShape.getImplicitShapeDimensions._X^;// * convexShape->getLocalScaling().getX();
      margin      := radius + sphereShape.getMarginNonVirtual;
      center      := t.getOriginV;
      extent.Init(margin,margin,margin);
      aabbMin := center^ - extent;
      aabbMax := center^ + extent;
    end;
    procedure _boxcyl;
    var convexShape   : btBoxShape;
        margin        : btScalar;
        halfextents,
        center,extent,
        dummy         : btVector3;
        abs_b         : btMatrix3x3;
    begin
      convexShape := btBoxShape(self);
      margin      := convexShape.getMarginNonVirtual;
      halfExtents := convexShape.getImplicitShapeDimensions;
      halfExtents += dummy.init(margin,margin,margin);
      abs_b       := t.GetBasisV^.absolute;
      center      := t.getOriginV^;
      extent.init(abs_b[0].dot(halfExtents),abs_b[1].dot(halfExtents),abs_b[2].dot(halfExtents));
      aabbMin     := center - extent;
      aabbMax     := center + extent;
    end;
    procedure _triangle;
    var triangleShape : btTriangleShape;
        margin        : btScalar;
        vec,sv,tmp    : btVector3;
        i             : Integer;
    begin
      triangleShape := btTriangleShape(self);
      margin        := triangleShape.getMarginNonVirtual;
      for i:=0 to 2 do begin
      	vec.init(0,0,0);
      	vec[i] := 1;
        sv     := localGetSupportVertexWithoutMarginNonVirtual(vec*t.GetBasisV^);
      	tmp    := t.opTrans(sv);
      	aabbMax[i] := tmp[i]+margin;
      	vec[i]     := -1;
      	tmp        := t.opTrans(localGetSupportVertexWithoutMarginNonVirtual(vec*t.GetBasisV^));
      	aabbMin[i] := tmp[i]-margin;
      end;
    end;
    procedure _capsule;
    var capsuleShape  : btCapsuleShape;
        margin,radius : btScalar;
        halfextents,
        center,extent,
        dummy         : btVector3;
        abs_b         : btMatrix3x3;
        m_upAxis      : integer;
    begin
      capsuleShape  := btCapsuleShape(self);
      radius        := capsuleShape.getRadius;
      halfExtents.init(radius,radius,radius);
      m_upAxis      := capsuleShape.getUpAxis;
      halfExtents[m_upAxis] := radius + capsuleShape.getHalfHeight;
      margin      := capsuleShape.getMarginNonVirtual;
      halfExtents += dummy.init(margin,margin,margin);
      abs_b       := t.GetBasisV^.absolute;
      center      := t.getOriginV^;
      extent.init(abs_b[0].dot(halfExtents),abs_b[1].dot(halfExtents),abs_b[2].dot(halfExtents));
      aabbMin := center - extent;
      aabbMax := center + extent;
    end;
    procedure _poly;
    var convexHullShape : btPolyhedralConvexAabbCachingShape;
        margin          : btScalar;
    begin
      convexHullShape := btPolyhedralConvexAabbCachingShape(self);
      margin          := convexHullShape.getMarginNonVirtual;
      convexHullShape.getNonvirtualAabb(t, aabbMin, aabbMax, margin);
    end;
begin
  case m_shapeType of
    SPHERE_SHAPE_PROXYTYPE:             _sphere;
    CYLINDER_SHAPE_PROXYTYPE,           //fallthrough
    BOX_SHAPE_PROXYTYPE:                _boxcyl;
    TRIANGLE_SHAPE_PROXYTYPE:           _triangle;
    CAPSULE_SHAPE_PROXYTYPE:            _capsule;
    CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE,
    CONVEX_HULL_SHAPE_PROXYTYPE:        _poly;
    else begin
      assert(false);
    end;
  end;
end;


{ btConvexInternalShape }

constructor btConvexInternalShape.create;
begin
 inherited Create;
  m_localScaling.InitSame(1);
  m_collisionMargin:=CONVEX_DISTANCE_MARGIN;
end;

function btConvexInternalShape.localGetSupportingVertex(const vec: btVector3): btVector3;
var supVertex : btVector3;
    vecnorm   : btVector3;
begin
  supVertex := localGetSupportingVertexWithoutMargin(vec);
  if getMargin<>0 then begin
    vecnorm := vec;
    vecnorm.length2NormCheck;
    vecnorm.normalize;
    supVertex+=vecnorm*getMargin;
  end;
  result:=supVertex;
end;

function btConvexInternalShape.getImplicitShapeDimensions: btVector3;
begin
  result:=m_implicitShapeDimensions;
end;

procedure btConvexInternalShape.setImplicitShapeDimensions(const dimensions: btVector3);
begin
  m_implicitShapeDimensions := dimensions;
end;

procedure btConvexInternalShape.getAabb(const t: btTransform; out aabbMin,aabbMax: btVector3);
begin
  getAabbSlow(t,aabbMin,aabbMax);
end;

procedure btConvexInternalShape.getAabbSlow(const t: btTransform; out aabbMin,  aabbMax: btVector3);
var margin     : btScalar;
    i          : Integer;
    vec,sv,tmp : btVector3;
begin
  //use localGetSupportingVertexWithoutMargin?
  margin := getMargin;
  for i:=0 to 2 do begin
    vec.init(0,0,0);
    vec[i]     := 1;
    sv         := localGetSupportingVertex(vec*t.GetBasisV^);
    tmp        := t.opTrans(sv);
    aabbMax[i] := tmp[i]+margin;
    vec[i]     := -1;
    tmp        := t.opTrans(localGetSupportingVertex(vec*t.GetBasisV^));
    aabbMin[i] := tmp[i]-margin;
  end;
end;

procedure btConvexInternalShape.setLocalScaling(const scaling: btVector3);
begin
  m_localScaling := scaling.absolute;
end;

function btConvexInternalShape.getLocalScaling: btVector3;
begin
  result:=m_localScaling;
end;

function btConvexInternalShape.getLocalScalingNV: btVector3;
begin
  result:=m_localScaling;
end;

procedure btConvexInternalShape.setMargin(const margin: btScalar);
begin
  m_collisionMargin := margin;
end;

function btConvexInternalShape.getMargin: btScalar;
begin
  result:=m_collisionMargin;
end;

function btConvexInternalShape.getMarginNV: btScalar;
begin
  result:=m_collisionMargin;
end;

function btConvexInternalShape.getNumPreferredPenetrationDirections: integer;
begin
  result:=0;
end;
{$HINTS OFF}
procedure btConvexInternalShape.getPreferredPenetrationDirection(const index: integer; var penetrationVector: btVector3);
begin
  assert(false);
end;
{$HINTS ON}

{ btConvexInternalAabbCachingShape }

procedure btConvexInternalAabbCachingShape.setCachedLocalAabb(const aabbMin,aabbMax: btVector3);
begin
   m_isLocalAabbValid := true;
   m_localAabbMin     := aabbMin;
   m_localAabbMax     := aabbMax;
end;

procedure btConvexInternalAabbCachingShape.getCachedLocalAabb(var aabbMin,aabbMax: btVector3);
begin
  btAssert(m_isLocalAabbValid);
  aabbMin := m_localAabbMin;
  aabbMax := m_localAabbMax;
end;

procedure btConvexInternalAabbCachingShape.getNonvirtualAabb(const trans: btTransform; out aabbMin, aabbMax: btVector3;const margin: btScalar);
begin
  //lazy evaluation of local aabb
  btAssert(m_isLocalAabbValid);
  btTransformAabb(m_localAabbMin,m_localAabbMax,margin,trans,aabbMin,aabbMax);
end;

constructor btConvexInternalAabbCachingShape.Create;
begin
  Inherited;
  m_localAabbMin.Init(1,1,1);
  m_localAabbMax.init(-1,-1,-1);
  m_isLocalAabbValid:=false;
end;

procedure btConvexInternalAabbCachingShape.setLocalScaling(const scaling: btVector3);
begin
  inherited setLocalScaling(scaling);
  recalcLocalAabb;
end;

procedure btConvexInternalAabbCachingShape.getAabb(const t: btTransform;out aabbMin, aabbMax: btVector3);
begin
  getNonvirtualAabb(t,aabbMin,aabbMax,getMargin);
end;

const _directions:array [0..5] of btVector3=
              (
               (v : (m_floats : (1,0,0,0))),
               (v : (m_floats : (0,1,0,0))),
               (v : (m_floats : (0,0,1,0))),
               (v : (m_floats : (-1,0,0,0))),
               (v : (m_floats : (0,-1,0,0))),
               (v : (m_floats : (0,0,-1,0)))
              );

procedure btConvexInternalAabbCachingShape.recalcLocalAabb;
var _supporting : array [0..5] of btVector3;
  i: Integer;
begin
 m_isLocalAabbValid := true;
 batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, @_supporting, 6);
 for i := 0 to 2 do begin
   m_localAabbMax[i] := _supporting[i  ][i] + m_collisionMargin;
   m_localAabbMin[i] := _supporting[i+3][i] - m_collisionMargin;
 end;
 //#else FOS: Safer Code ?
 //for (int i=0;i<3;i++)
 //{
 //	btVector3 vec(btScalar(0.),btScalar(0.),btScalar(0.));
 //	vec[i] = btScalar(1.);
 //	btVector3 tmp = localGetSupportingVertex(vec);
 //	m_localAabbMax[i] = tmp[i]+m_collisionMargin;
 //	vec[i] = btScalar(-1.);
 //	tmp = localGetSupportingVertex(vec);
 //	m_localAabbMin[i] = tmp[i]-m_collisionMargin;
 //}
 //#endif
end;

{ btSphereShape }

constructor btSphereShape.Create(const radius: btScalar);
begin
  Inherited create;
  m_shapeType     := SPHERE_SHAPE_PROXYTYPE;
  m_implicitShapeDimensions.setX(radius);
  m_collisionMargin := radius;
end;

function btSphereShape.localGetSupportingVertex(const vec: btVector3): btVector3;
var vecnorm:btVector3;
begin
 result := localGetSupportingVertexWithoutMargin(vec);
 vecnorm   := vec;
 vecnorm.length2NormCheck;
 vecnorm.normalize;
 result+= (vecnorm*getMargin);
end;
{$HINTS OFF}
function btSphereShape.localGetSupportingVertexWithoutMargin(const vec: btVector3): btVector3;
begin
  result.Init(0,0,0);
end;
{$HINTS ON}
{$HINTS OFF}
procedure btSphereShape.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3;const numVectors: Integer);
var  i: Integer;
begin
  for i:=0 to numVectors-1 do begin
    supportVerticesOut[i].InitSame(0);
  end
end;
{$HINTS ON}

procedure btSphereShape.getAabb(const t: btTransform; out aabbMin,aabbMax: btVector3);
var center,extent:btVector3;
    marg:btScalar;
begin
  center := t.getOriginV^;
  marg   := getMargin;
  extent.init(marg,marg,marg);
  aabbMin := center - extent;
  aabbMax := center + extent;
end;

procedure btSphereShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
var elem:btScalar;
begin
  elem := 0.4 * mass * getMargin*getMargin;
  inertia.Init(elem,elem,elem);
end;

function btSphereShape.getRadius: btscalar;
begin
   Result:=m_implicitShapeDimensions._X^ * m_localScaling._X^;
end;

procedure btSphereShape.setUnscalesRadius(const radius: btScalar);
begin
  m_implicitShapeDimensions._X^:=radius;
  inherited setMargin(radius);
end;

function btSphereShape.getName: String;
begin
  result:='SPHERE';
end;

procedure btSphereShape.setMargin(const margin: btScalar);
begin
  inherited setMargin(margin);
end;

function btSphereShape.getMargin: btScalar;
begin
  Result:=getRadius;
end;


{ btBoxShape }

constructor btBoxShape.Create(const boxHalfExtents: btVector3);
var margin: btVector3;
begin
  inherited create;
  m_shapeType := BOX_SHAPE_PROXYTYPE;
  margin.InitSame(getMargin);
  m_implicitShapeDimensions := (boxHalfExtents * m_localScaling) - margin;
end;

function btBoxShape.getHalfExtentsWithMargin: btVector3;
var margin:btVector3;
begin
  result := getHalfExtentsWithoutMargin;
  margin.InitSame(getMargin);
  result += margin;
end;

function btBoxShape.getHalfExtentsWithoutMargin: btVector3;
begin
  Result := m_implicitShapeDimensions;//scaling is included, margin is not
end;

function btBoxShape.localGetSupportingVertex(const vec: btVector3): btVector3;
var halfExtents,margin:btVector3;
begin
  halfExtents := getHalfExtentsWithoutMargin;
  margin.InitSame(getMargin);
  halfExtents += margin;
  Result.init(btFsel(vec._x^, halfExtents._x^, -halfExtents._x^),
              btFsel(vec._y^, halfExtents._y^, -halfExtents._y^),
              btFsel(vec._z^, halfExtents._z^, -halfExtents._z^));
end;

function btBoxShape.localGetSupportingVertexWithoutMargin(const vec: btVector3): btVector3;
var halfExtents:btVector3;
begin
  halfExtents := getHalfExtentsWithoutMargin;
  Result.init(btFsel(vec._x^, halfExtents._x^, -halfExtents._x^),
              btFsel(vec._y^, halfExtents._y^, -halfExtents._y^),
              btFsel(vec._z^, halfExtents._z^, -halfExtents._z^));
end;

procedure btBoxShape.batchedUnitVectorGetSupportingVertexWithoutMargin( const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);
var halfExtents : btVector3;
    vec         : PbtVector3;
    i           : Integer;
begin
  halfExtents := getHalfExtentsWithoutMargin;
  for i:=0 to numVectors-1 do begin
    vec := @vectors[i];
    supportVerticesOut[i].init(btFsel(vec^._x^, halfExtents._x^, -halfExtents._x^),
                btFsel(vec^._y^, halfExtents._y^, -halfExtents._y^),
                btFsel(vec^._z^, halfExtents._z^, -halfExtents._z^));
  end;
end;

procedure btBoxShape.setMargin(const collisionmargin: btScalar);
var newMargin,oldMargin,implicitShapeDimensionsWithMargin:btVector3;
begin
  //correct the m_implicitShapeDimensions for the margin
  oldMargin.InitSame(getMargin);
  implicitShapeDimensionsWithMargin := m_implicitShapeDimensions+oldMargin;
  inherited setMargin(collisionMargin);
  newMargin.InitSame(getMargin);
  m_implicitShapeDimensions := implicitShapeDimensionsWithMargin - newMargin;
end;

procedure btBoxShape.setLocalScaling(const scaling: btVector3);
var oldMargin,implicitShapeDimensionsWithMargin,unScaledImplicitShapeDimensionsWithMargin:btVector3;
begin
  oldMargin.InitSame(getMargin);
  implicitShapeDimensionsWithMargin         := m_implicitShapeDimensions+oldMargin;
  unScaledImplicitShapeDimensionsWithMargin := implicitShapeDimensionsWithMargin / m_localScaling;
  inherited setLocalScaling(scaling);
  m_implicitShapeDimensions                 := (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
end;

procedure btBoxShape.getAabb(const t: btTransform; out aabbMin, aabbMax: btVector3);
begin
  btTransformAabb(getHalfExtentsWithoutMargin,getMargin,t,aabbMin,aabbMax);
end;

procedure btBoxShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
var halfExtents : btVector3;
    lx,ly,lz    : btScalar;
begin
  halfExtents := getHalfExtentsWithMargin;
  lx          := 2 * halfExtents._x^;
  ly          := 2 * halfExtents._y^;
  lz          := 2 * halfExtents._z^;
  inertia.Init(mass/12 * (ly*ly + lz*lz),mass/12 * (lx*lx + lz*lz),mass/12*(lx*lx + ly*ly));
end;

procedure btBoxShape.getPlane(var planeNormal, planeSupport: btVector3; const i: integer);
var plane:btVector4;
begin
  //this plane might not be aligned...
  {$HINTS OFF}
  getPlaneEquation(plane,i);
  {$HINTS ON}
  planeNormal.init(plane._X^,plane._Y^,plane._Z^);
  planeSupport := localGetSupportingVertex(-planeNormal);
end;

function btBoxShape.getNumPlanes: integer;
begin
  result:=6;
end;

function btBoxShape.getNumVertices: integer;
begin
  result:=8;
end;

function btBoxShape.getNumEdges: integer;
begin
  result:=12;
end;

procedure btBoxShape.getVertex(const i: integer; out vtx: btVector3);
var halfExtents:btVector3;
begin
 halfExtents := getHalfExtentsWithoutMargin;
 vtx.init( halfExtents._x^ * (1-( i and 1))        - halfExtents._x^ * ( i AND 1),
           halfExtents._y^ * (1-(SarLongint((i and 2),1)) - halfExtents._y^ * (SarLongint((i AND 2),1))),
           halfExtents._z^ * (1-(Sarlongint((i and 4),2)) - halfExtents._z^ * (SarLongint((i AND 4),2))));
end;

procedure btBoxShape.getPlaneEquation(var plane: btVector4; const i: integer);
var halfExtents:btVector3;
begin
  halfExtents := getHalfExtentsWithoutMargin;
  case i of
     0:  plane.Init4( 1, 0, 0,-halfExtents._x^);
     1:  plane.Init4(-1, 0, 0,-halfExtents._x^);
     2:  plane.Init4( 0, 1, 0,-halfExtents._y^);
     3:  plane.Init4( 0,-1, 0,-halfExtents._y^);
     4:  plane.Init4( 0, 0, 1,-halfExtents._z^);
     5:  plane.Init4( 0, 0,-1,-halfExtents._z^);
     else btAssert(false);
  end;
end;

procedure btBoxShape.getEdge(const i: integer; out pa, pb: btVector3);
var edgeVert0,edgeVert1:integer;
begin
  edgeVert0:=0;
  edgeVert1:=0;
  case (i) of
     0: begin edgeVert0 := 0; edgeVert1 := 1; end;
     1: begin edgeVert0 := 0; edgeVert1 := 2; end;
     2: begin edgeVert0 := 1; edgeVert1 := 3; end;
     3: begin edgeVert0 := 2; edgeVert1 := 3; end;
     4: begin edgeVert0 := 0; edgeVert1 := 4; end;
     5: begin edgeVert0 := 1; edgeVert1 := 5; end;
     6: begin edgeVert0 := 2; edgeVert1 := 6; end;
     7: begin edgeVert0 := 3; edgeVert1 := 7; end;
     8: begin edgeVert0 := 4; edgeVert1 := 5; end;
     9: begin edgeVert0 := 4; edgeVert1 := 6; end;
    10: begin edgeVert0 := 5; edgeVert1 := 7; end;
    11: begin edgeVert0 := 6; edgeVert1 := 7; end;
    else btAssert(false);
  end;
  getVertex(edgeVert0,pa);
  getVertex(edgeVert1,pb);
end;

function btBoxShape.isInside(const pt: btVector3; const tolerance: btScalar): boolean;
var halfExtents:btVector3;
begin
  halfExtents := getHalfExtentsWithoutMargin;
  //btScalar minDist = 2*tolerance;
  result :=   (pt._x^ <= ( halfExtents._x^+tolerance)) AND
              (pt._x^ >= (-halfExtents._x^-tolerance)) AND
              (pt._y^ <= ( halfExtents._y^+tolerance)) AND
              (pt._y^ >= (-halfExtents._y^-tolerance)) AND
              (pt._z^ <= ( halfExtents._z^+tolerance)) AND
              (pt._z^ >= (-halfExtents._z^-tolerance));
end;

function btBoxShape.getName: String;
begin
  result:='BOX';
end;

function btBoxShape.getNumPreferredPenetrationDirections: integer;
begin
  Result:=6;
end;

procedure btBoxShape.getPreferredPenetrationDirection(const index: integer;var penetrationVector: btVector3);
begin
  case index of
    0: penetrationVector.Init( 1, 0, 0);
    1: penetrationVector.Init(-1, 0, 0);
    2: penetrationVector.Init( 0, 1, 0);
    3: penetrationVector.Init( 0,-1, 0);
    4: penetrationVector.Init( 0, 0, 1);
    5: penetrationVector.Init( 0, 0,-1);
    else btAssert(false);
  end;
end;

function btBoxShape.getIndex(const i: integer): integer;
begin
  result:=i;
  abort; // Warning fix (abstract method)
end;

{ btPolyhedralConvexShape }

constructor btPolyhedralConvexShape.Create;
begin
  inherited Create;
end;

function btPolyhedralConvexShape.localGetSupportingVertexWithoutMargin(const vec0: btVector3): btVector3;
var supVec,vtx,vec        : btVector3;
    maxDot,lenSqr,newDot,rlen : btScalar;
    i                         : Integer;
begin
  supVec.Init(0,0,0);
  maxDot   := -BT_LARGE_FLOAT;
  vec      := vec0;
  lenSqr   := vec.length2;
  if (lenSqr < 0.0001) then begin
    vec.Init(1,0,0);
  end else begin
    rlen := 1/btSqrt(lenSqr);
    vec *= rlen;
  end;
  for i:=0 to getNumVertices-1 do begin
    {$HINTS OFF}
    getVertex(i,vtx);
    {$HINTS ON}
    newDot := vec.dot(vtx);
    if (newDot > maxDot) then begin
      maxDot := newDot;
      supVec := vtx;
    end;
  end;
  result:=supVec;
end;

procedure btPolyhedralConvexShape.batchedUnitVectorGetSupportingVertexWithoutMargin (const vectors: PbtVector3; const supportVerticesOut: PbtVector3;const numVectors: Integer);
var vtx                       : btVector3;
    vec                       : PbtVector3;
    newDot                    : btScalar;
    i,j                       : Integer;
begin
  for i:=0 to numVectors-1 do begin
    supportVerticesOut[i][3] := -BT_LARGE_FLOAT;
  end;
  for j:=0 to numVectors-1 do begin
  	vec := @vectors[j];
  	for i:=0 to getNumVertices-1 do begin
          {$HINTS OFF}
          getVertex(i,vtx);
          {$HINTS ON}
          newDot := vec^.dot(vtx);
          if (newDot > supportVerticesOut[j][3]) then begin
            //WARNING: don't swap next lines, the w component would get overwritten!
            supportVerticesOut[j] := vtx;
            supportVerticesOut[j][3] := newDot;
          end;
        end;
  end;
end;

procedure btPolyhedralConvexShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
var margin,lx,ly,lz,x2,y2,z2,scaledmass   : btScalar;
    ident                                 : btTransform;
    halfExtents,aabbMin,aabbMax,dummy     : btVector3;
begin
  //not yet, return box inertia
  margin := getMargin;
  ident.setIdentity;
  {$HINTS OFF}
  getAabb(ident,aabbMin,aabbMax);
  {$HINTS ON}
  halfExtents := (aabbMax-aabbMin)*btScalar(0.5);
  lx := 2*(halfExtents._x^+margin); ly := 2*(halfExtents._y^+margin); lz :=2*(halfExtents._z^+margin);
  x2 := lx*lx; y2 := ly*ly; z2 := lz*lz;
  scaledmass := mass * btScalar(0.08333333);
  inertia    := (dummy.init(y2+z2,x2+z2,x2+y2)) * scaledmass;
end;

{ btPolyhedralConvexAabbCachingShape }

procedure btPolyhedralConvexAabbCachingShape.setCachedLocalAabb(const aabbMin,aabbMax: btVector3);
begin
  m_isLocalAabbValid := true;
  m_localAabbMin     := aabbMin;
  m_localAabbMax     := aabbMax;
end;

procedure btPolyhedralConvexAabbCachingShape.getCachedLocalAabb(var aabbMin, aabbMax: btVector3);
begin
  btAssert(m_isLocalAabbValid);
  aabbMin := m_localAabbMin;
  aabbMax := m_localAabbMax;
end;

constructor btPolyhedralConvexAabbCachingShape.Create;
begin
  inherited Create;
  m_localAabbMin.init(1,1,1);
  m_localAabbMax.init(-1,-1,-1);
  m_isLocalAabbValid:=false;
end;

procedure btPolyhedralConvexAabbCachingShape.getNonvirtualAabb(const trans: btTransform; out aabbMin, aabbMax: btVector3;const margin: btScalar);
begin
  //lazy evaluation of local aabb
  btAssert(m_isLocalAabbValid);
  btTransformAabb(m_localAabbMin,m_localAabbMax,margin,trans,aabbMin,aabbMax);
end;

procedure btPolyhedralConvexAabbCachingShape.setLocalScaling(const scaling: btVector3);
begin
  inherited setLocalScaling(scaling);
  recalcLocalAabb;
end;

procedure btPolyhedralConvexAabbCachingShape.getAabb(const t: btTransform;out aabbMin, aabbMax: btVector3);
begin
  getNonVirtualAabb(t,aabbMin,aabbMax,getMargin);
end;

procedure btPolyhedralConvexAabbCachingShape.recalcLocalAabb;
var _supporting : array [0..5] of btVector3;
  i: Integer;
begin
 m_isLocalAabbValid := true;
 batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, @_supporting, 6);
 for i := 0 to 2 do begin
   m_localAabbMax[i] := _supporting[i  ][i] + m_collisionMargin;
   m_localAabbMin[i] := _supporting[i+3][i] - m_collisionMargin;
 end;
 //#else FOS: Safer Code ?
 //for (int i=0;i<3;i++)
 //{
 //	btVector3 vec(btScalar(0.),btScalar(0.),btScalar(0.));
 //	vec[i] = btScalar(1.);
 //	btVector3 tmp = localGetSupportingVertex(vec);
 //	m_localAabbMax[i] = tmp[i]+m_collisionMargin;
 //	vec[i] = btScalar(-1.);
 //	tmp = localGetSupportingVertex(vec);
 //	m_localAabbMin[i] = tmp[i]-m_collisionMargin;
 //}
 //#endif
end;

{ btTriangleShape }

constructor btTriangleShape.Create;
begin
  inherited Create;
  m_shapeType := TRIANGLE_SHAPE_PROXYTYPE;
end;

constructor btTriangleShape.Create(const p0, p1, p2: btVector3);
begin
  inherited Create;
  m_shapeType := TRIANGLE_SHAPE_PROXYTYPE;
  m_vertices1[0] := p0;
  m_vertices1[1] := p1;
  m_vertices1[2] := p2;
end;

function btTriangleShape.getNumVertices: integer;
begin
  Result:=3;
end;

function btTriangleShape.getVertexPtr(const idx: integer): PbtVector3;
begin
  Result:=@m_vertices1[idx];
end;

procedure btTriangleShape.getVertex(const i: integer; out vtx: btVector3);
begin
  vtx := m_vertices1[i];
end;

function btTriangleShape.getNumEdges: integer;
begin
  Result:=3;
end;

procedure btTriangleShape.getEdge(const i: integer; out pa, pb: btVector3);
begin
  getVertex(i,pa);
  getVertex((i+1) MOD 3,pb);
end;

procedure btTriangleShape.getAabb(const t: btTransform; out aabbMin,aabbMax: btVector3);
begin
  //          btAssert(0);
  getAabbSlow(t,aabbMin,aabbMax);
end;

function btTriangleShape.localGetSupportingVertexWithoutMargin(const vec: btVector3): btVector3;
var dots:btVector3;
begin
  dots.init(vec.dot(m_vertices1[0]), vec.dot(m_vertices1[1]), vec.dot(m_vertices1[2]));
  result:=m_vertices1[dots.maxAxis];
end;

procedure btTriangleShape.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3;const numVectors: Integer);
var dir  : PbtVector3;
    dots : btVector3;
    i    : Integer;
begin
  for i:=0 to numVectors-1 do begin
    dir := @vectors[i];
    dots.init(dir^.dot(m_vertices1[0]), dir^.dot(m_vertices1[1]), dir^.dot(m_vertices1[2]));
    supportVerticesOut[i] := m_vertices1[dots.maxAxis];
  end;
end;

procedure btTriangleShape.getPlane(var planeNormal, planeSupport: btVector3; const i: integer);
begin
  getPlaneEquation(i,planeNormal,planeSupport);
end;

function btTriangleShape.getNumPlanes: integer;
begin
  result:=1;
end;

procedure btTriangleShape.calcNormal(var normal: btVector3);
begin
  normal := (m_vertices1[1]-m_vertices1[0]).cross(m_vertices1[2]-m_vertices1[0]);
  normal.normalize;
end;
{$HINTS OFF}
procedure btTriangleShape.getPlaneEquation(const i: integer; var planenormal,  planesupport: btVector3);
begin
  calcNormal(planeNormal);
  planeSupport := m_vertices1[0];
end;
{$HINTS ON}

{$HINTS OFF}
procedure btTriangleShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
begin
  //AHELPER
  btAssert(false);
  inertia.InitSame(0);
end;
{$HINTS ON}

function btTriangleShape.isInside(const pt: btVector3; const tolerance: btScalar): boolean;
var  normal,pa,pb,edge,edgenormal : btVector3;
     dist, planeconst,edgeConst   : btScalar;
     i                            : Integer;
begin
  {$HINTS OFF}
  calcNormal(normal);
  {$HINTS ON}
  //distance to plane
  dist       := pt.dot(normal);
  planeconst := m_vertices1[0].dot(normal);
  dist -= planeconst;
  if (dist >= -tolerance) AND (dist <= tolerance) then begin
    //inside check on edge-planes
    for i:=0 to 2 do begin
      {$HINTS OFF}
      getEdge(i,pa,pb);
      {$HINTS ON}
      edge       := pb-pa;
      edgeNormal := edge.cross(normal);
      edgeNormal.normalize;
      dist       := pt.dot(edgeNormal);
      edgeConst  := pa.dot(edgeNormal);
      dist       -= edgeConst;
      if (dist < -tolerance) then exit(false);
    end;
    exit(true);
  end;
  Result:=false;
end;

function btTriangleShape.getName: String;
begin
  result:='Triangle';
end;

function btTriangleShape.getNumPreferredPenetrationDirections: integer;
begin
  Result:=2;
end;

procedure btTriangleShape.getPreferredPenetrationDirection(const index: integer; var penetrationVector: btVector3);
begin
  calcNormal(penetrationVector);
  if index<>0 then begin
    penetrationVector *= -1;
  end;
end;

function btTriangleShape.getIndex(const i: integer): integer;
begin
  Result:=i;
  //WARNING FIX
  abort;
end;

function CylinderLocalSupportX(const halfExtents:btVector3;const v:btVector3):btVector3;FOS_INLINE;
var   radius,halfHeight,d,s:btScalar;
      tmp:btVector3;
const cylinderUpAxis:integer=0; XX:integer=1; YY:integer=0; ZZ:integer=2;
begin
  //cylinderUpAxis := 0;
  //XX := 1;  YY:= 0 ; ZZ := 2;
  //mapping depends on how cylinder local orientation is
  // extents of the cylinder is: X,Y is for radius, and Z for height
  radius     := halfExtents[XX];
  halfHeight := halfExtents[cylinderUpAxis];
  s          := btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
  if s <> 0 then begin
    d := radius / s;
    tmp[XX] := v[XX] * d;
    if v[YY] < 0.0 then begin
      tmp[YY] :=  -halfHeight;;
    end else begin
      tmp[YY] :=   halfHeight;
    end;
    tmp[ZZ] := v[ZZ] * d;
  end else begin
    tmp[XX] := radius;
    if v[YY] < 0.0 then begin
      tmp[YY] :=  -halfHeight;;
    end else begin
      tmp[YY] :=   halfHeight;
    end;
    tmp[ZZ] := 0;
  end;
  result := tmp;
end;
function CylinderLocalSupportY(const halfExtents:btVector3;const v:btVector3):btVector3;FOS_INLINE;
var   radius,halfHeight,d,s:btScalar;
      tmp:btVector3;
const cylinderUpAxis:integer=1; XX:integer=0; YY:integer=1; ZZ:integer=2;
begin
  //cylinderUpAxis := 0;
  //XX := 1;  YY:= 0 ; ZZ := 2;
  //mapping depends on how cylinder local orientation is
  // extents of the cylinder is: X,Y is for radius, and Z for height
  radius     := halfExtents[XX];
  halfHeight := halfExtents[cylinderUpAxis];
  s          := btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
  if s <> 0 then begin
    d := radius / s;
    tmp[XX] := v[XX] * d;
    if v[YY] < 0.0 then begin
      tmp[YY] :=  -halfHeight;;
    end else begin
      tmp[YY] :=   halfHeight;
    end;
    tmp[ZZ] := v[ZZ] * d;
  end else begin
    tmp[XX] := radius;
    if v[YY] < 0.0 then begin
      tmp[YY] :=  -halfHeight;;
    end else begin
      tmp[YY] :=   halfHeight;
    end;
    tmp[ZZ] := 0;
  end;
  result := tmp;
end;
function CylinderLocalSupportZ(const halfExtents:btVector3;const v:btVector3):btVector3;FOS_INLINE;
var   radius,halfHeight,d,s:btScalar;
      tmp:btVector3;
const cylinderUpAxis:integer=2; XX:integer=0; YY:integer=2; ZZ:integer=1;
begin
  //cylinderUpAxis := 0;
  //XX := 1;  YY:= 0 ; ZZ := 2;
  //mapping depends on how cylinder local orientation is
  // extents of the cylinder is: X,Y is for radius, and Z for height
  radius     := halfExtents[XX];
  halfHeight := halfExtents[cylinderUpAxis];
  s          := btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
  if s <> 0 then begin
    d := radius / s;
    tmp[XX] := v[XX] * d;
    if v[YY] < 0.0 then begin
      tmp[YY] :=  -halfHeight;;
    end else begin
      tmp[YY] :=   halfHeight;
    end;
    tmp[ZZ] := v[ZZ] * d;
  end else begin
    tmp[XX] := radius;
    if v[YY] < 0.0 then begin
      tmp[YY] :=  -halfHeight;;
    end else begin
      tmp[YY] :=   halfHeight;
    end;
    tmp[ZZ] := 0;
  end;
  result := tmp;
end;



{ btCylinderShape }

function btCylinderShape.getHalfExtentsWithMargin: btVector3;
var margin:btVector3;
begin
  result := getHalfExtentsWithoutMargin;
  margin.InitSame(getMargin);
  result += margin;
end;

function btCylinderShape.getHalfExtentsWithoutMargin: btVector3;
begin
  result := m_implicitShapeDimensions;
end;

constructor btCylinderShape.Create(const HalfExtents: btVector3);
var margin:btVector3;
begin
  inherited Create;
  m_upAxis:=1;
  margin.InitSame(getMargin);
  m_implicitShapeDimensions := (m_localScaling * halfExtents) - margin;
  m_shapeType := CYLINDER_SHAPE_PROXYTYPE;
end;

procedure btCylinderShape.getAabb(const t: btTransform; out aabbMin, aabbMax: btVector3);
begin
  btTransformAabb(getHalfExtentsWithoutMargin,getMargin,t,aabbMin,aabbMax);
end;

function btCylinderShape.getUpAxis: integer;
begin
  result := m_upAxis;
end;

function btCylinderShape.getRadius: btScalar;
begin
  result:=getHalfExtentsWithMargin._X^;
end;

function btCylinderShape.getName: string;
begin
  Result:= 'CylinderY';
end;

function btCylinderShape.localGetSupportingVertexWithoutMargin(const vec: btVector3): btVector3;
begin
  Result:=CylinderLocalSupportY(getHalfExtentsWithoutMargin,vec);
end;

procedure btCylinderShape.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);
var i: Integer;
begin
  for i:=0 to numVectors-1 do begin
    supportVerticesOut[i] := CylinderLocalSupportY(getHalfExtentsWithoutMargin,vectors[i]);
  end;
end;

procedure btCylinderShape.setMargin(const collisionmargin: btScalar);
var newMargin,oldMargin,implicitShapeDimensionsWithMargin:btVector3;
begin
  //correct the m_implicitShapeDimensions for the margin
  oldMargin.InitSame(getMargin);
  implicitShapeDimensionsWithMargin := m_implicitShapeDimensions+oldMargin;
  inherited setMargin(collisionMargin);
  newMargin.InitSame(getMargin);
  m_implicitShapeDimensions := implicitShapeDimensionsWithMargin - newMargin;
end;

function btCylinderShape.localGetSupportingVertex(const vec: btVector3): btVector3;
var vecnorm:btVector3;
begin
  result := localGetSupportingVertexWithoutMargin(vec);
  if ( getMargin<>0 ) then begin
    vecnorm := vec;
    vecnorm.length2NormCheck;
    vecnorm.normalize;
    result := result + (vecnorm * getMargin);
  end;
end;

procedure btCylinderShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
//approximation of box shape, todo: implement cylinder shape inertia before people notice ;-)
var halfExtents : btVector3;
    lx,ly,lz    : btScalar;
begin
  halfExtents := getHalfExtentsWithMargin;
  lx          := 2 * halfExtents._x^;
  ly          := 2 * halfExtents._y^;
  lz          := 2 * halfExtents._z^;
  inertia.Init(mass/12 * (ly*ly + lz*lz),mass/12 * (lx*lx + lz*lz),mass/12*(lx*lx + ly*ly));
end;


{ btCylinderShapeX }

constructor btCylinderShapeX.Create(const HalfExtents: btVector3);
begin
 Inherited Create(HalfExtents);
 m_upAxis := 0;
end;

function btCylinderShapeX.localGetSupportingVertexWithoutMargin(const vec: btVector3): btVector3;
begin
  Result:=CylinderLocalSupportX(getHalfExtentsWithoutMargin,vec);
end;

procedure btCylinderShapeX.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);
var i: Integer;
begin
  for i:=0 to numVectors-1 do begin
    supportVerticesOut[i] := CylinderLocalSupportX(getHalfExtentsWithoutMargin,vectors[i]);
  end;
end;

function btCylinderShapeX.getName: string;
begin
  result:='CylinderX';
end;

function btCylinderShapeX.getRadius: btScalar;
begin
  Result:=getHalfExtentsWithMargin._Y^;
end;

{ btCylinderShapeZ }

constructor btCylinderShapeZ.Create(const HalfExtents: btVector3);
begin
  Inherited Create(HalfExtents);
  m_upAxis := 2;
end;

function btCylinderShapeZ.localGetSupportingVertexWithoutMargin(const vec: btVector3): btVector3;
begin
  Result:=CylinderLocalSupportZ(getHalfExtentsWithoutMargin,vec);
end;

procedure btCylinderShapeZ.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);
var i: Integer;
begin
  for i:=0 to numVectors-1 do begin
    supportVerticesOut[i] := CylinderLocalSupportZ(getHalfExtentsWithoutMargin,vectors[i]);
  end;
end;

function btCylinderShapeZ.getName: string;
begin
  result:='CylinderZ';
end;

function btCylinderShapeZ.getRadius: btScalar;
begin
  Result:=getHalfExtentsWithMargin.getX;
end;

{ btCapsuleShape }

procedure btCapsuleShape.Create;
begin
  m_shapeType:=CAPSULE_SHAPE_PROXYTYPE;
end;

constructor btCapsuleShape.Create(const radius, height: btScalar);
begin
  m_shapeType := CAPSULE_SHAPE_PROXYTYPE;
  m_upAxis    := 1;
  m_implicitShapeDimensions.Init(radius,0.5*height,radius);
end;

procedure btCapsuleShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
var ident : btTransform;
   halfExtents : btVector3;
   radius,margin,lx,ly,lz,x2,y2,z2,scaledmass : btScalar;
begin
  //as an approximation, take the inertia of the box that bounds the spheres
  ident.setIdentity;
  radius := getRadius;
  halfExtents.InitSame(radius);
  halfExtents[getUpAxis] := halfExtents[getUpAxis] + getHalfHeight;
  margin := CONVEX_DISTANCE_MARGIN;
  lx := 2 * (halfExtents[0]+margin);
  ly := 2 * (halfExtents[1]+margin);
  lz := 2 * (halfExtents[2]+margin);
  x2 := lx*lx;
  y2 := ly*ly;
  z2 := lz*lz;
  scaledmass := mass * 0.08333333;
  inertia[0] := scaledmass * (y2+z2);
  inertia[1] := scaledmass * (x2+z2);
  inertia[2] := scaledmass * (x2+y2);
end;

function btCapsuleShape.localGetSupportingVertexWithoutMargin(const vec0: btVector3): btVector3;
var supVec,vec,vtx,pos:btVector3;
    rlen,maxDot,newdot,lenSqr,radius :btScalar;
begin
  supVec.InitSame(0);
  maxDot := -BT_LARGE_FLOAT;
  vec    := vec0;
  lenSqr := vec.length2;
  if lenSqr < 0.0001 then begin
    vec.Init(1,0,0);
  end else begin
    rlen := 1 / btSqrt(lenSqr);
    vec *= rlen;
  end;
  radius := getRadius;
  pos.InitSame(0);
  pos[getUpAxis] := getHalfHeight;
  vtx    := pos +vec*m_localScaling*(radius) - vec * getMargin;
  newDot := vec.dot(vtx);
  if (newDot > maxDot) then begin
    maxDot := newDot;
    supVec := vtx;
  end;
  pos.InitSame(0);
  pos[getUpAxis()] := -getHalfHeight();
  vtx     := pos +vec*m_localScaling*(radius) - vec * getMargin;
  newDot  := vec.dot(vtx);
  if newDot > maxDot then begin
    maxDot := newDot;
    supVec := vtx;
  end;
  Result:=supVec;
end;

procedure btCapsuleShape.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);
var radius,maxDot,newDot : btScalar;
    vec     : PbtVector3;
    pos,vtx : btVector3;
    j: Integer;
begin
  radius := getRadius();
  for j := 0 to numVectors-1 do begin
    maxDot := -BT_LARGE_FLOAT;
    vec    :=  @vectors[j];
    pos.InitSame(0);
    pos[getUpAxis] := getHalfHeight;
    vtx := pos + vec^*m_localScaling*(radius)-vec^*getMargin;
    newDot := vec^.dot(vtx);
    if (newDot > maxDot) then begin
      maxDot                := newDot;
      supportVerticesOut[j] := vtx;
    end;
    pos.InitSame(0);
    pos[getUpAxis] := -getHalfHeight;
    vtx    := pos +vec^*m_localScaling*(radius) - vec^ * getMargin;
    newDot := vec^.dot(vtx);
    if (newDot > maxDot) then begin
      maxDot                := newDot;
      supportVerticesOut[j] := vtx;
    end;
  end;
end;

procedure btCapsuleShape.setMargin(const collisionmargin: btScalar);
var oldMargin,newMargin,implicitShapeDimensionsWithMargin  : btVector3;
begin
  //correct the m_implicitShapeDimensions for the margin
  oldMargin.InitSame(getMargin);
  implicitShapeDimensionsWithMargin := m_implicitShapeDimensions+oldMargin;
  inherited setMargin(collisionMargin);
  newMargin.InitSame(getMargin);
  m_implicitShapeDimensions := implicitShapeDimensionsWithMargin - newMargin;
end;

procedure btCapsuleShape.getAabb(const t: btTransform; out aabbMin, aabbMax: btVector3);
var halfExtents,
    center,
    extent      : btVector3;
    radius      : btScalar;
    abs_b       : btMatrix3x3;
begin
  radius := getRadius;
  halfExtents.InitSame(radius);
  halfExtents[m_upAxis] := radius + getHalfHeight;
  halfExtents += btVector3.InitSameS(getMargin);
  abs_b       := t.GetBasisV^.absolute;
  center      := t.getOriginV^;
  extent      := btVector3.InitS(abs_b[0].dot(halfExtents),abs_b[1].dot(halfExtents),abs_b[2].dot(halfExtents));
  aabbMin     := center - extent;
  aabbMax     := center + extent;
end;

function btCapsuleShape.getName: String;
begin
  result := 'CapsuleShape';
end;

function btCapsuleShape.getUpAxis: integer;
begin
  result := m_upAxis;
end;

function btCapsuleShape.getRadius: btScalar;
var  radiusAxis:integer;
begin
  radiusAxis := (m_upAxis+2) mod 3;
  result     := m_implicitShapeDimensions[radiusAxis];
end;

function btCapsuleShape.getHalfHeight: btScalar;
begin
  Result := m_implicitShapeDimensions[m_upAxis];
end;

procedure btCapsuleShape.setLocalScaling(const scaling: btVector3);
var oldMargin,implicitShapeDimensionsWithMargin,unScaledImplicitShapeDimensionsWithMargin:btVector3;
begin
  oldMargin.InitSame(getMargin);
  implicitShapeDimensionsWithMargin         := m_implicitShapeDimensions+oldMargin;
  unScaledImplicitShapeDimensionsWithMargin := implicitShapeDimensionsWithMargin / m_localScaling;
  inherited setLocalScaling(scaling);
  m_implicitShapeDimensions                 := (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
end;

{ btCapsuleShapeX }

constructor btCapsuleShapeX.Create(const radius, height: btScalar);
begin
  Inherited Create;
  m_upAxis := 0;
  m_implicitShapeDimensions.Init(0.5*height, radius,radius);
end;

function btCapsuleShapeX.getName: String;
begin
  Result:='CapsuleShapeX'
end;

{ btCapsuleShapeZ }

constructor btCapsuleShapeZ.Create(const radius, height: btScalar);
begin
  Inherited Create;
  m_upAxis := 2;
  m_implicitShapeDimensions.Init(radius,radius,0.5*height);
end;

function btCapsuleShapeZ.getName: String;
begin
  result := 'CapsuleZ';
end;

{ btConeShape }

function btConeShape.coneLocalSupport(const v: btVector3): btVector3;
var halfHeight,s,d : btScalar;
    tmp            : btVector3;
begin
  halfHeight := m_height * btScalar(0.5);
  if (v[m_coneIndices[1]] > (v.length * m_sinAngle)) then begin
    tmp[m_coneIndices[0]] := 0;
    tmp[m_coneIndices[1]] := halfHeight;
    tmp[m_coneIndices[2]] := 0;
    exit(tmp);
  end else begin
    s := btSqrt(v[m_coneIndices[0]] * v[m_coneIndices[0]] + v[m_coneIndices[2]] * v[m_coneIndices[2]]);
    if s > SIMD_EPSILON then begin
      d := m_radius / s;
      tmp[m_coneIndices[0]] := v[m_coneIndices[0]] * d;
      tmp[m_coneIndices[1]] := -halfHeight;
      tmp[m_coneIndices[2]] := v[m_coneIndices[2]] * d;
      exit(tmp);
    end else begin
      tmp[m_coneIndices[0]] := 0;
      tmp[m_coneIndices[1]] := -halfHeight;
      tmp[m_coneIndices[2]] := 0;
      exit(tmp);
     end;
  end;
end;

constructor btConeShape.Create(const radius, height: btScalar);
begin
  Inherited Create;
  m_radius:=radius;
  m_height:=height;
  m_shapeType := CONE_SHAPE_PROXYTYPE;
  setConeUpIndex(1);
  m_sinAngle := (m_radius / btSqrt(m_radius * m_radius + m_height * m_height));
end;

function btConeShape.localGetSupportingVertex(const vec: btVector3): btVector3;
var supVertex,vecnorm : btVector3;
begin
  supVertex := coneLocalSupport(vec);
  if getMargin <> 0 then begin
    vecnorm := vec;
    vecnorm.length2NormCheck;
    vecnorm.normalize;
    supVertex := supVertex + (vecnorm*getMargin);
  end;
  Result := supVertex;
end;

function btConeShape.localGetSupportingVertexWithoutMargin(const vec0: btVector3): btVector3;
begin
  result :=  coneLocalSupport(vec0);
end;

procedure btConeShape.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);
var i   : Integer;
begin
  for i:=0 to numVectors-1 do begin
    supportVerticesOut[i] := coneLocalSupport(vectors[i]);
  end;
end;

function btConeShape.getRadius: btScalar;
begin
  result := m_radius;
end;

function btConeShape.getHeight: btScalar;
begin
  result := m_height;
end;

procedure btConeShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
var identity:btTransform;
    aabbMin,aabbMax,halfExtents:btVector3;
    margin,lx,ly,lz,x2,y2,z2,scaledmass:btScalar;
begin
  identity.setIdentity;
  {$HINTS OFF}
  getAabb(identity,aabbMin,aabbMax);
  {$HINTS ON}
  halfExtents := (aabbMax-aabbMin)*0.5;
  margin := getMargin;

  lx := 2 * (halfExtents._X^+margin);
  ly := 2 * (halfExtents._Y^+margin);
  lz := 2 * (halfExtents._Z^+margin);
  x2 := lx*lx;
  y2 := ly*ly;
  z2 := lz*lz;
  scaledmass := mass * 0.08333333;
  inertia    := btVector3.InitS(y2+z2,x2+z2,x2+y2) * scaledmass ;
end;

function btConeShape.getName: String;
begin
  Result:='Cone';
end;

procedure btConeShape.setConeUpIndex(const upIndex: Integer);
begin
  case (upIndex) of
  0: begin
       m_coneIndices[0] := 1;
       m_coneIndices[1] := 0;
       m_coneIndices[2] := 2;
     end;
  1: begin
       m_coneIndices[0] := 0;
       m_coneIndices[1] := 1;
       m_coneIndices[2] := 2;
     end;
  2: begin
       m_coneIndices[0] := 0;
       m_coneIndices[1] := 2;
       m_coneIndices[2] := 1;
     end;
     else btAssert(false);
  end;
end;

function btConeShape.getConeUpIndex: Integer;
begin
  result := m_coneIndices[1];
end;

{ btConeShapeX }

constructor btConeShapeX.Create(const radius, height: btScalar);
begin
  inherited Create(radius,height);
  setConeUpIndex(0);
end;

{ btConeShapeZ }

constructor btConeShapeZ.Create(const radius, height: btScalar);
begin
  inherited Create(radius,height);
  setConeUpIndex(2);
end;

{ btStaticPlaneShape }

constructor btStaticPlaneShape.Create(const planeNormal: btVector3; const planeConstant: btScalar);
begin
  Inherited create;
  m_planeNormal   := planeNormal.normalized;
  m_planeConstant := planeConstant;
  m_localScaling  .InitSame(0);
  m_shapeType     := STATIC_PLANE_PROXYTYPE;
  //      btAssert( btFuzzyZero(m_planeNormal.length() - btScalar(1.)) );
end;

{$HINTS OFF}
procedure btStaticPlaneShape.getAabb(const t: btTransform; out aabbMin, aabbMax: btVector3);
begin
  aabbMin.InitSame(-BT_LARGE_FLOAT);
  aabbMax.InitSame(BT_LARGE_FLOAT);
end;
{$HINTS ON}

procedure btStaticPlaneShape.processAllTriangles(const callback: btTriangleCallback; const aabbMin, aabbMax: btVector3);
var halfExtents,center,
    tangentDir0,tangentDir1,
    projectedCenter          : btVector3;
    triangle                 : array [0..2] of btVector3;
    radius                   : btScalar;
begin
  halfExtents := (aabbMax - aabbMin) * btScalar(0.5);
  radius      := halfExtents.length();
  center      := (aabbMax + aabbMin) * btScalar(0.5);

  //this is where the triangles are generated, given AABB and plane equation (normal/constant)
  //tangentDir0/tangentDir1 can be precalculated
  btPlaneSpace1(m_planeNormal,tangentDir0,tangentDir1);
  projectedCenter := center - (m_planeNormal.dot(center) - m_planeConstant)*m_planeNormal;

  triangle[0] := projectedCenter + tangentDir0*radius + tangentDir1*radius;
  triangle[1] := projectedCenter + tangentDir0*radius - tangentDir1*radius;
  triangle[2] := projectedCenter - tangentDir0*radius - tangentDir1*radius;

  callback.processTriangle(triangle,0,0);

  triangle[0] := projectedCenter - tangentDir0*radius - tangentDir1*radius;
  triangle[1] := projectedCenter - tangentDir0*radius + tangentDir1*radius;
  triangle[2] := projectedCenter + tangentDir0*radius + tangentDir1*radius;

  callback.processTriangle(triangle,0,1);
end;
{$HINTS OFF}
procedure btStaticPlaneShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
begin
  //moving concave objects not supported
  inertia.InitSame(0);
end;
{$HINTS ON}

procedure btStaticPlaneShape.setLocalScaling(const scaling: btVector3);
begin
  m_localScaling:=scaling;
end;

function btStaticPlaneShape.getLocalScaling: btVector3;
begin
  result:=m_localScaling;
end;

function btStaticPlaneShape.getPlaneNormal: btVector3;
begin
  Result:=m_planeNormal;
end;

function btStaticPlaneShape.getPlaneConstant: btScalar;
begin
 Result:=m_planeConstant;
end;

function btStaticPlaneShape.getPlaneNormalP: PbtVector3;
begin
  result:=@m_planeNormal;
end;

function btStaticPlaneShape.getPlaneConstantP: PbtScalar;
begin
 result:=@m_planeConstant;
end;

function btStaticPlaneShape.getName: String;
begin
  Result := 'STATICPLANE';
end;

{ btConvexHullShape }

constructor btConvexHullShape.Create(const points: PbtScalar; const numPoints: Integer; const stride: integer);
var pointsAddress:Pointer;
    i: Integer;
   // point:PbtScalar; FOSHH Reduced
begin
  Inherited Create;
  m_shapeType := CONVEX_HULL_SHAPE_PROXYTYPE;
  m_unscaledPoints := btFOSAlignedVectorArray.create;
  m_unscaledPoints.Setlength(numPoints);
  pointsAddress := points;
  for i:=0 to numPoints-1 do begin
    //point := pointsAddress;
    //m_unscaledPoints[i] := btVector3.InitS(point[0], point[1], point[2]);
     m_unscaledPoints.A[i]^.InitS(PbtScalar(pointsAddress)[0], PbtScalar(pointsAddress)[1],PbtScalar(pointsAddress)[2]);
    pointsAddress += stride;
  end;
  recalcLocalAabb;
end;

destructor btConvexHullShape.Destroy;
begin
  m_unscaledPoints.Free;
 inherited Destroy;
end;

procedure btConvexHullShape.addPoint(const point: btVector3);
begin
  m_unscaledPoints.push_back(point);
  recalcLocalAabb;
end;

function btConvexHullShape.getUnscaledPoints: PbtVector3;
begin
  Result:=m_unscaledPoints.A[0];
end;

function btConvexHullShape.getScaledPoint(const i: integer):btVector3;
begin
  Result := m_unscaledPoints.A[i]^ * m_localScaling;
end;

function btConvexHullShape.getNumPoints: integer;
begin
  Result := m_unscaledPoints.Length;
end;

function btConvexHullShape.localGetSupportingVertex(const vec: btVector3): btVector3;
var supVertex:btVector3;
  vecnorm: btVector3;
begin
  supVertex := localGetSupportingVertexWithoutMargin(vec);
  if getMargin<>0 then begin
    vecnorm := vec;
    vecnorm.length2NormCheck;
    vecnorm.normalize;
    supVertex+= getMargin * vecnorm;
  end;
  Result := supVertex;
end;

function btConvexHullShape.localGetSupportingVertexWithoutMargin(const vec0: btVector3): btVector3;
var supVec,vtx :btVector3;
    newDot,maxDot:btScalar;
    i: Integer;
begin
  supVec.InitSame(0);
  newDot := -BT_LARGE_FLOAT;
  maxDot := -BT_LARGE_FLOAT;
  for i:=0 to m_unscaledPoints.Length-1 do begin
    vtx    := m_unscaledPoints.A[i]^ * m_localScaling;
    newDot := vec0.dot(vtx);
    if (newDot > maxDot) then begin
      maxDot := newDot;
      supVec := vtx;
    end;
  end;
  Result := supVec;
end;

procedure btConvexHullShape.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);
var newDot  : btScalar;
         i  : Integer;
        vtx : btVector3;
        vec : PbtVector3;
        j: Integer;
begin
  //use 'w' component of supportVerticesOut?
  for i:=0 to numVectors-1 do begin
    supportVerticesOut[i][3] := -BT_LARGE_FLOAT;
  end;
  for i:=0 to m_unscaledPoints.Length-1 do begin
    vtx := getScaledPoint(i);
    for j := 0 to numVectors-1 do begin
      vec  := @vectors[j];
      newDot := vec^.dot(vtx);
      if (newDot > supportVerticesOut[j][3]) then begin
        //WARNING: don't swap next lines, the w component would get overwritten!
        supportVerticesOut[j]    := vtx;
        supportVerticesOut[j][3] := newDot;
      end;
    end;
  end;
end;

function btConvexHullShape.getName: String;
begin
  result:='Convex';
end;


//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you can debug-draw btConvexHullShape with the Raytracer Demo
function btConvexHullShape.getNumVertices: integer;
begin
  Result := m_unscaledPoints.Length;
end;

function btConvexHullShape.getNumEdges: integer;
begin
  Result := m_unscaledPoints.Length;
end;

procedure btConvexHullShape.getEdge(const i: integer; out pa, pb: btVector3);
var index0,index1:integer;
begin
  index0 :=  i    mod m_unscaledPoints.Length;
  index1 := (i+1) mod m_unscaledPoints.Length;
  pa     := getScaledPoint(index0);
  pb     := getScaledPoint(index1);
end;

procedure btConvexHullShape.getVertex(const i: integer; out vtx: btVector3);
begin
  vtx := getScaledPoint(i);
end;

function btConvexHullShape.getNumPlanes: integer;
begin
  result := 0;
end;
{$HINTS OFF}
procedure btConvexHullShape.getPlane(var planeNormal, planeSupport: btVector3; const i: integer);
begin
  //
  btAssert(false);
end;
{$HINTS ON}

{$HINTS OFF}
//not yet
function btConvexHullShape.isInside(const pt: btVector3; const tolerance: btScalar): boolean;
begin
  //
  btAssert(false);
  result:=false;
end;
{$HINTS ON}

procedure btConvexHullShape.setLocalScaling(const scaling: btVector3);
begin
  m_localScaling := scaling;
  recalcLocalAabb;
end;

function btConvexHullShape.getIndex(const i: integer): integer;
begin
  result := i;
  abort;
  //abstract fix
end;

{ btTriangleMesh }

constructor btTriangleMesh.Create(const use32bitIndices: boolean; const use4componentVertices: boolean);
var meshIndex : btIndexedMesh;
begin
  m_use32bitIndices               := use32bitIndices;
  m_use4componentVertices         := use4componentVertices;
  m_weldingThreshold              := 0;
  meshIndex.m_numTriangles        := 0;
  meshIndex.m_numVertices         := 0;
  meshIndex.m_indexType           := PHY_INTEGER;
  meshIndex.m_triangleIndexBase   := nil;
  meshIndex.m_triangleIndexStride := 3*sizeof(Integer);
  meshIndex.m_vertexBase          := nil;
  meshIndex.m_vertexStride        := sizeof(btVector3);
  m_indexedMeshes.push_back(meshIndex);

  if (m_use32bitIndices) then begin
    m_indexedMeshes.A[0]^.m_numTriangles        := m_32bitIndices.Length div 3;
    m_indexedMeshes.A[0]^.m_triangleIndexBase   := nil;
    m_indexedMeshes.A[0]^.m_indexType           := PHY_INTEGER;
    m_indexedMeshes.A[0]^.m_triangleIndexStride := 3*sizeof(Integer);
  end else begin
    m_indexedMeshes.A[0]^.m_numTriangles        := m_16bitIndices.Length div 3;
    m_indexedMeshes.A[0]^.m_triangleIndexBase   := nil;
    m_indexedMeshes.A[0]^.m_indexType           := PHY_SHORT;
    m_indexedMeshes.A[0]^ .m_triangleIndexStride := 3*sizeof(Word);
  end;
  if (m_use4componentVertices) then begin
    m_indexedMeshes.A[0]^.m_numVertices         := m_4componentVertices.Length;
    m_indexedMeshes.A[0]^.m_vertexBase          := nil;
    m_indexedMeshes.A[0]^.m_vertexStride        := sizeof(btVector3);
  end  else begin
    m_indexedMeshes.A[0]^.m_numVertices         := m_3componentVertices.Length div 3;
    m_indexedMeshes.A[0]^.m_vertexBase          := nil;
    m_indexedMeshes.A[0]^.m_vertexStride        := 3*sizeof(btScalar);
  end;
end;

function btTriangleMesh.getUse32bitIndices: boolean;
begin
  result := m_use32bitIndices;
end;

function btTriangleMesh.getUse4componentVertices: Boolean;
begin
  result := m_use4componentVertices;
end;

procedure btTriangleMesh.addTriangle(const vertex0, vertex1, vertex2: btVector3; const removeDuplicateVertices: boolean);
begin
  m_indexedMeshes.A[0]^.m_numTriangles := m_indexedMeshes.A[0]^.m_numTriangles + 1;
  addIndex(findOrAddVertex(vertex0,removeDuplicateVertices));
  addIndex(findOrAddVertex(vertex1,removeDuplicateVertices));
  addIndex(findOrAddVertex(vertex2,removeDuplicateVertices));
end;

function btTriangleMesh.getNumTriangles: integer;
begin
  if m_use32bitIndices then begin
    exit(m_32bitIndices.Length div 3);
  end;
  result := m_16bitIndices.Length div 3;
end;


function btTriangleMesh.findOrAddVertex(const vertex: btVector3; const removeDuplicateVertices: boolean): integer;
var  i   : Integer;
     vtx : btVector3;
begin
  //return index of new/existing vertex
  ///@todo: could use acceleration structure for this
  if m_use4componentVertices then begin
    if removeDuplicateVertices then begin
      for i:=0 to m_4componentVertices.Length-1 do begin
        if (m_4componentVertices.A[i]^-vertex).length2 <= m_weldingThreshold then begin
          exit(i);
        end;
      end;
    end;
    inc(m_indexedMeshes.A[0]^.m_numVertices);
    m_4componentVertices.push_back(vertex);
    m_indexedMeshes.A[0]^.m_vertexBase := m_4componentVertices.A[0];
    exit(m_4componentVertices.Length-1);
  end else begin
    if removeDuplicateVertices then begin
  //    for (int i=0;i< m_3componentVertices.size();i+=3)
      i:=0;
      while(i<m_3componentVertices.Length) do begin
        vtx.Init(m_3componentVertices.A[i]^,m_3componentVertices.A[i+1]^,m_3componentVertices.A[i+2]^);
        if (vtx-vertex).length2 <= m_weldingThreshold then begin
          exit(i div 3);
        end;
        inc(i,3);
      end;
    end;
    m_3componentVertices.push_back(vertex._X^);
    m_3componentVertices.push_back(vertex._Y^);
    m_3componentVertices.push_back(vertex._Z^);
    inc(m_indexedMeshes.A[0]^.m_numVertices);
    m_indexedMeshes.A[0]^.m_vertexBase := m_3componentVertices.A[0];
    exit((m_3componentVertices.Length div 3)-1);
  end;
end;

procedure btTriangleMesh.addIndex(const index: integer);
begin
  if (m_use32bitIndices) then begin
    m_32bitIndices.push_back(index);
    m_indexedMeshes.A[0]^.m_triangleIndexBase :=  m_32bitIndices.A[0];
  end else begin
    m_16bitIndices.push_back(index);
    m_indexedMeshes.A[0]^.m_triangleIndexBase :=  m_16bitIndices.A[0];
  end;
end;

{ btTriangleMeshShape }

constructor btTriangleMeshShape.Create(const meshInterface: btStridingMeshInterface);
begin
  inherited Create;
  m_meshInterface := meshInterface;
  m_shapeType     := TRIANGLE_MESH_SHAPE_PROXYTYPE;
  if meshInterface.hasPremadeAabb then begin
    meshInterface.getPremadeAabb(m_localAabbMin,m_localAabbMax);
  end else begin
    recalcLocalAabb;
  end;
end;

type

  { SupportVertexCallback }

  TSupportVertexCallback=class(btTriangleCallback)
    m_supportVertexLocal:btVector3;
  public
    m_worldTrans : btTransform;
    m_maxDot     : btScalar;
    m_supportVecLocal : btVector3;
    constructor create                      (const supportVecWorld:btVector3;const trans:btTransform);
    procedure   processTriangle             (const triangle: PbtVector3; const partId, triangleIndex: integer);override;
    function    GetSupportVertexWorldSpace  :btVector3;
    function    GetSupportVertexLocal       :btVector3;
  end;

{ SupportVertexCallback }

constructor TSupportVertexCallback.create(const supportVecWorld: btVector3; const trans: btTransform);
begin
  m_supportVertexLocal.InitSame(0);
  m_worldTrans      := trans;
  m_maxDot          := -BT_LARGE_FLOAT;
  m_supportVecLocal := supportVecWorld * m_worldTrans.GetBasisV^;
end;
{$HINTS OFF}
procedure TSupportVertexCallback.processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer);
var
  dot: btScalar;
  i: Integer;
begin
  //(void)partId;
  //(void)triangleIndex;
  for i:=0 to 2 do begin
    dot := m_supportVecLocal.dot(triangle[i]);
    if (dot > m_maxDot) then begin
      m_maxDot := dot;
      m_supportVertexLocal := triangle[i];
    end;
  end;
end;
{$HINTS ON}

function TSupportVertexCallback.GetSupportVertexWorldSpace: btVector3;
begin
  Result := m_worldTrans.opTrans(m_supportVertexLocal);
end;

function TSupportVertexCallback.GetSupportVertexLocal: btVector3;
begin
  Result := m_supportVertexLocal;
end;



function btTriangleMeshShape.localGetSupportingVertex(const vec: btVector3): btVector3;
var supportVertex,aabbMax:btVector3;
    ident:btTransform;
    suppcb:TSupportVertexCallback;
begin
  ident.setIdentity;
  suppcb := TSupportVertexCallback.create(vec,ident);
  aabbMax.InitSame(BT_LARGE_FLOAT);
  processAllTriangles(suppcb,-aabbMax,aabbMax);
  supportVertex := suppcb.GetSupportVertexLocal;
  Result := supportVertex;
  suppcb.Free;
end;

function btTriangleMeshShape.localGetSupportingVertexWithoutMargin(const vec0: btVector3): btVector3;
begin
  btAssert(false);
  Result := localGetSupportingVertex(vec0);
end;

procedure btTriangleMeshShape.recalcLocalAabb;
begin

end;

procedure btTriangleMeshShape.getAabb(const t: btTransform; out aabbMin, aabbMax: btVector3);
var localHalfExtents,localCenter,center,extent:btVector3;
     abs_b : btMatrix3x3;
begin
  localHalfExtents := 0.5*(m_localAabbMax-m_localAabbMin);
  localHalfExtents += btVector3.InitSameS(getMargin);
  localCenter      := 0.5*(m_localAabbMax+m_localAabbMin);
  abs_b            := t.GetBasisV^.absolute;
  center           := t.opTrans(localCenter);
  extent           := btVector3.InitS(abs_b[0].dot(localHalfExtents),abs_b[1].dot(localHalfExtents),abs_b[2].dot(localHalfExtents));
  aabbMin          := center - extent;
  aabbMax          := center + extent;
end;

type

  { FilteredCallback }

  TFilteredCallback =class(btInternalTriangleIndexCallback)
    m_callback : btTriangleCallback;
    m_aabbMin,m_aabbMax : btVector3;
    constructor create                       (const callback:btTriangleCallback;const aabbMin,aabbMax:btVector3);
    procedure   internalProcessTriangleIndex (const triangle: PbtVector3; const partId, triangleIndex: integer);override;
   end;

{ FilteredCallback }

constructor TFilteredCallback.create(const callback: btTriangleCallback; const aabbMin, aabbMax: btVector3);
begin
  m_callback := callback;
  m_aabbMin  := aabbMin;
  m_aabbMax  := aabbMax;
end;

procedure TFilteredCallback.internalProcessTriangleIndex(const triangle: PbtVector3; const partId, triangleIndex: integer);
begin
  if (TestTriangleAgainstAabb2(@triangle[0],m_aabbMin,m_aabbMax)) then begin
    //check aabb in triangle-space, before doing this
    m_callback.processTriangle(triangle,partId,triangleIndex);
  end;
end;




procedure btTriangleMeshShape.processAllTriangles(const callback: btTriangleCallback; const aabbMin, aabbMax: btVector3);
var filtercb : TFilteredCallback;
begin
  filtercb := TFilteredCallback.create(callback,aabbMin,aabbMax);
  m_meshInterface.InternalProcessAllTriangles(filtercb,aabbMin,aabbMax);
  filtercb.free;
end;

{$HINTS OFF}
procedure btTriangleMeshShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
begin
  //(void)mass;
  //moving concave objects not supported
  btAssert(false);
  inertia.InitSame(0);
end;
{$HINTS ON}

procedure btTriangleMeshShape.setLocalScaling(const scaling: btVector3);
begin
  m_meshInterface.setScaling(scaling);
  recalcLocalAabb;
end;

function btTriangleMeshShape.getLocalScaling: btVector3;
begin
  Result := m_meshInterface.getScaling;
end;

function btTriangleMeshShape.getMeshInterface: btStridingMeshInterface;
begin
  result := m_meshInterface;
end;

function btTriangleMeshShape.getLocalAabbMin: btVector3;
begin
  Result := m_localAabbMin;
end;

function btTriangleMeshShape.getName: String;
begin
  result:='TRIANGLEMESH';
end;

{ btOptimizedBvh }

type

    { btNodeTriangleCallback }

    btNodeTriangleCallback =class(btInternalTriangleIndexCallback)
      m_triangleNodes : btNodeArray;
      function    opequals(const other:btNodeTriangleCallback):btNodeTriangleCallback;
      constructor create(const triangleNodes:btNodeArray);
      procedure   internalProcessTriangleIndex(const triangle:PbtVector3;const  partId,triangleIndex:integer);override;
     end;

{ btNodeTriangleCallback }

function btNodeTriangleCallback.opequals(const other: btNodeTriangleCallback): btNodeTriangleCallback;
begin
  m_triangleNodes := other.m_triangleNodes;
  Result := self;
end;

constructor btNodeTriangleCallback.create(const triangleNodes: btNodeArray);
begin
  m_triangleNodes:=triangleNodes;
end;

procedure btNodeTriangleCallback.internalProcessTriangleIndex(const triangle: PbtVector3; const partId, triangleIndex: integer);
var node:btOptimizedBvhNode;
    aabbMin,aabbMax:btVector3;
begin
  aabbMin.InitSame(BT_LARGE_FLOAT);
  aabbMax.InitSame(-BT_LARGE_FLOAT);
  aabbMin.setMin(triangle[0]);
  aabbMax.setMax(triangle[0]);
  aabbMin.setMin(triangle[1]);
  aabbMax.setMax(triangle[1]);
  aabbMin.setMin(triangle[2]);
  aabbMax.setMax(triangle[2]);

  //with quantization?
  node.m_aabbMinOrg  := aabbMin;
  node.m_aabbMaxOrg  := aabbMax;
  node.m_escapeIndex := -1;

  //for child nodes
  node.m_subPart := partId;
  node.m_triangleIndex := triangleIndex;
  m_triangleNodes.push_back(node);
end;

type

   { btQuantizedNodeTriangleCallback }

   btQuantizedNodeTriangleCallback=class(btInternalTriangleIndexCallback)
     m_triangleNodes : btQuantizedNodeArray;
     m_optimizedTree : btQuantizedBvh; // for quantization
     function   opEquals (const other:btQuantizedNodeTriangleCallback):btQuantizedNodeTriangleCallback;
     constructor create  (const triangleNodes:btQuantizedNodeArray;const tree:btQuantizedBvh);
     procedure  internalProcessTriangleIndex(const triangle:PbtVector3;const  partId,triangleIndex:integer);override;
   end;

{ btQuantizedNodeTriangleCallback }

function btQuantizedNodeTriangleCallback.opEquals(const other: btQuantizedNodeTriangleCallback): btQuantizedNodeTriangleCallback;
begin
  m_triangleNodes := other.m_triangleNodes;
  m_optimizedTree := other.m_optimizedTree;
  Result :=self;
end;

constructor btQuantizedNodeTriangleCallback.create(const triangleNodes: btQuantizedNodeArray; const tree: btQuantizedBvh);
begin
  m_triangleNodes := triangleNodes;
  m_optimizedTree := tree;
end;

const MIN_AABB_DIMENSION:btScalar = btScalar(0.002);
const MIN_AABB_HALF_DIMENSION:btScalar = btScalar(0.001);

procedure btQuantizedNodeTriangleCallback.internalProcessTriangleIndex(const triangle: PbtVector3; const partId, triangleIndex: integer);
var node:btQuantizedBvhNode;
    aabbMin,aabbMax:btVector3;
begin
  // The partId and triangle index must fit in the same (positive) integer
  btAssert(partId < (1 SHL MAX_NUM_PARTS_IN_BITS));
  btAssert(triangleIndex < (1 SHL (31-MAX_NUM_PARTS_IN_BITS)));
  //negative indices are reserved for escapeIndex
  btAssert(triangleIndex>=0);

  aabbMin.InitSame(BT_LARGE_FLOAT);
  aabbMax.InitSame(-BT_LARGE_FLOAT);
  aabbMin.setMin(triangle[0]);
  aabbMax.setMax(triangle[0]);
  aabbMin.setMin(triangle[1]);
  aabbMax.setMax(triangle[1]);
  aabbMin.setMin(triangle[2]);
  aabbMax.setMax(triangle[2]);

  //PCK: add these checks for zero dimensions of aabb
  if (aabbMax._x^ - aabbMin._x^ < MIN_AABB_DIMENSION) then begin
    aabbMax._X^ := aabbMax._x^ + MIN_AABB_HALF_DIMENSION;
    aabbMin._X^ := aabbMin._x^ - MIN_AABB_HALF_DIMENSION;
  end;
  if (aabbMax._Y^ - aabbMin._Y^ < MIN_AABB_DIMENSION) then begin
    aabbMax._Y^  := aabbMax._Y^ + MIN_AABB_HALF_DIMENSION;
    aabbMin._Y^  := aabbMin._Y^ - MIN_AABB_HALF_DIMENSION;
  end;
  if (aabbMax._Z^ - aabbMin._Z^ < MIN_AABB_DIMENSION) then begin
    aabbMax._Z^   := aabbMax._Z^ + MIN_AABB_HALF_DIMENSION;
    aabbMin._Z^   := aabbMin._Z^ - MIN_AABB_HALF_DIMENSION;
  end;
  m_optimizedTree.quantize(@node.m_quantizedAabbMin[0],aabbMin,false);
  m_optimizedTree.quantize(@node.m_quantizedAabbMax[0],aabbMax,true);
  node.m_escapeIndexOrTriangleIndex := (partId SHL (31-MAX_NUM_PARTS_IN_BITS)) OR triangleIndex;
  m_triangleNodes.push_back(node);
end;


procedure btOptimizedBvh.build(const triangles: btStridingMeshInterface; const useQuantizedAabbCompression: Boolean; const bvhAabbMin, bvhAabbMax: btVector3);
var
  numLeafNodes  : Integer;
  subtree       : PbtBvhSubtreeInfo;

  procedure _quant;FOS_INLINE;
  var callback      : btQuantizedNodeTriangleCallback;
  begin
    //initialize quantization values
    setQuantizationValues(bvhAabbMin,bvhAabbMax);
    callback:=btQuantizedNodeTriangleCallback.create(m_quantizedLeafNodes,self);
    triangles.InternalProcessAllTriangles(callback,m_bvhAabbMin,m_bvhAabbMax);
    //now we have an array of leafnodes in m_leafNodes
    numLeafNodes := m_quantizedLeafNodes.Length;
    m_quantizedContiguousNodes.SetCapacity(2*numLeafNodes); //FOSHH: was .resize(2*numLeafNodes)
    callback.free;
  end;

  procedure _noquant;
  var  callback      : btNodeTriangleCallback;
       aabbMin,aabbMax:btVector3;
  begin
    callback := btNodeTriangleCallback.create(m_leafNodes);
    aabbMin.InitSame(-BT_LARGE_FLOAT);
    aabbMax.InitSame(BT_LARGE_FLOAT);
    triangles.InternalProcessAllTriangles(callback,aabbMin,aabbMax);
    callback.free;
    //now we have an array of leafnodes in m_leafNodes
    numLeafNodes := m_leafNodes.Length;
    m_contiguousNodes.SetCapacity(2*numLeafNodes);//FOSHH: was .resize(2*numLeafNodes);
  end;

begin
  m_useQuantization := useQuantizedAabbCompression;
  // NodeArray  triangleNodes;
  numLeafNodes := 0;
  if m_useQuantization then begin
    _quant;
  end else begin
    _noquant;
  end;
  m_curNodeIndex := 0;
  buildTree(0,numLeafNodes);
  ///if the entire tree is small then subtree size, we need to create a header info for the tree
  if m_useQuantization and (m_SubtreeHeaders.Length=0) then begin
    subtree := m_SubtreeHeaders.push_new_no_init;
    subtree^.setAabbFromQuantizeNode(m_quantizedContiguousNodes.A[0]^);
    subtree^.m_rootNodeIndex := 0;
    if m_quantizedContiguousNodes.A[0]^.isLeafNode then begin
      subtree^.m_subtreeSize := 1;
    end else begin
      subtree^.m_subtreeSize := m_quantizedContiguousNodes.A[0]^.getEscapeIndex;
    end;
  end;
  //PCK: update the copy of the size
  m_subtreeHeaderCount := m_SubtreeHeaders.Length;
  //PCK: clear m_quantizedLeafNodes and m_leafNodes, they are temporary
  m_quantizedLeafNodes.Free;
  m_leafNodes.Free;
end;

procedure btOptimizedBvh.refit(const triangles: btStridingMeshInterface; const AabbMin, AabbMax: btVector3);
var subtree:PbtBvhSubtreeInfo;
    i: Integer;
begin
  if m_useQuantization then begin
    setQuantizationValues(aabbMin,aabbMax);
    updateBvhNodes(triangles,0,m_curNodeIndex,0);
    ///now update all subtree headers
    for i := 0 to m_SubtreeHeaders.Length-1 do begin
      subtree := m_SubtreeHeaders.A[i];
      subtree^.setAabbFromQuantizeNode(m_quantizedContiguousNodes.A[subtree^.m_rootNodeIndex]^);
    end;
  end else begin
  end;
end;

procedure btOptimizedBvh.refitPartial(const triangles: btStridingMeshInterface; const AabbMin, AabbMax: btVector3);
var   quantizedQueryAabbMin,quantizedQueryAabbMax : array [0..2] of word;
      subtree:PbtBvhSubtreeInfo;
      i: Integer;
      overlap: Boolean;
begin
  //incrementally initialize quantization values
  btAssert(m_useQuantization);
  btAssert(aabbMin.getX() > m_bvhAabbMin.getX());
  btAssert(aabbMin.getY() > m_bvhAabbMin.getY());
  btAssert(aabbMin.getZ() > m_bvhAabbMin.getZ());
  btAssert(aabbMax.getX() < m_bvhAabbMax.getX());
  btAssert(aabbMax.getY() < m_bvhAabbMax.getY());
  btAssert(aabbMax.getZ() < m_bvhAabbMax.getZ());
  ///we should update all quantization values, using updateBvhNodes(meshInterface);
  ///but we only update chunks that overlap the given aabb
  quantize(@quantizedQueryAabbMin[0],aabbMin,false);
  quantize(@quantizedQueryAabbMax[0],aabbMax,true);

  for i := 0 to m_SubtreeHeaders.Length-1 do begin
    subtree := m_SubtreeHeaders.A[i];
    //PCK: unsigned instead of bool
    overlap := testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree^.m_quantizedAabbMin,subtree^.m_quantizedAabbMax);
    if overlap then begin
      updateBvhNodes(triangles,subtree^.m_rootNodeIndex,subtree^.m_rootNodeIndex+subtree^.m_subtreeSize,i);
      subtree^.setAabbFromQuantizeNode(m_quantizedContiguousNodes.A[subtree^.m_rootNodeIndex]^);
    end;
  end;
end;

{$HINTS OFF}
procedure btOptimizedBvh.updateBvhNodes(const triangles: btStridingMeshInterface; const firstNode, endNode, index: integer);
var
  curNodeSubPart,numverts,stride : Integer;
  vertexbase,indexbase           : PByte;
  typ,indicestype                : PHY_ScalarType;
  indexstride,i,numfaces         : Integer;
  aabbMin,aabbMax                : btVector3;
  triangleVerts                  : array [0..2] of btVector3;
  meshscaling                    : PbtVector3;
  curNode,leftChildNode,
  rightChildNode                 : PbtQuantizedBvhNode;
  gfxbase                        : PCardinal;
  nodeTriangleIndex,graphicsindex: Integer;
  nodeSubPart,j                  : Integer;
  graphicsbase_s                 : PSingle;
  graphicsbase_d                 : PDouble;
  k: Integer;
begin
  //laz
  btAssert(m_useQuantization);
  curNodeSubPart:=-1;
//get access info to trianglemesh data
  vertexbase := nil;
  numverts   := 0;
  typ        := PHY_INTEGER;
  stride     := 0;
  indexbase  := nil;
  indexstride:= 0;
  numfaces   := 0;
  indicestype:= PHY_INTEGER;
  meshScaling:= triangles.getScalingP;
  for i := endNode-1 downto firstNode do  begin
    curNode := m_quantizedContiguousNodes.A[i];
    if curNode^.isLeafNode then begin
      //recalc aabb from triangle data
      nodeSubPart       := curNode^.getPartId;
      nodeTriangleIndex := curNode^.getTriangleIndex;
      if nodeSubPart <> curNodeSubPart then begin
        if curNodeSubPart >= 0 then begin
          triangles.unLockReadOnlyVertexBase(curNodeSubPart);
        end;
        triangles.getLockedReadOnlyVertexIndexBase(vertexbase,numverts,typ,stride,indexbase,indexstride,numfaces,indicestype,nodeSubPart);
        curNodeSubPart := nodeSubPart;
        btAssert((indicestype=PHY_INTEGER) OR (indicestype=PHY_SHORT));
      end;
      gfxbase := PCardinal(indexbase)+(nodeTriangleIndex*indexstride);
      for j:=2 downto 0 do begin
        graphicsindex := btDecide(indicestype=PHY_SHORT , PWord(gfxbase)[j] , gfxbase[j]);
        if (typ = PHY_FLOAT) then begin
          graphicsbase_s   := PSingle(vertexbase)+graphicsindex*stride;
          triangleVerts[j] := btVector3.inits(graphicsbase_s[0]*meshScaling^.getX,graphicsbase_s[1]*meshScaling^.getY,graphicsbase_s[2]*meshScaling^.getZ);
        end else begin
          graphicsbase_d   := PDouble(vertexbase)+graphicsindex*stride;
          triangleVerts[j] := btVector3.inits(graphicsbase_d[0]*meshScaling^.getX,graphicsbase_d[1]*meshScaling^.getY,graphicsbase_d[2]*meshScaling^.getZ);
        end;
      end;
      aabbMin.InitSame(BT_LARGE_FLOAT);
      aabbMax.InitSame(-BT_LARGE_FLOAT);
      aabbMin.setMin(triangleVerts[0]);
      aabbMax.setMax(triangleVerts[0]);
      aabbMin.setMin(triangleVerts[1]);
      aabbMax.setMax(triangleVerts[1]);
      aabbMin.setMin(triangleVerts[2]);
      aabbMax.setMax(triangleVerts[2]);
      quantize(@curNode^.m_quantizedAabbMin[0],aabbMin,false);
      quantize(@curNode^.m_quantizedAabbMax[0],aabbMax,true);
    end else begin
      //combine aabb from both children
      leftChildNode  := m_quantizedContiguousNodes.A[i+1];
      if leftChildNode^.isLeafNode then begin
        rightChildNode :=  m_quantizedContiguousNodes.A[i+2];
      end else begin
        rightChildNode :=  m_quantizedContiguousNodes.A[i+1+leftChildNode^.getEscapeIndex];
      end;
      for k:=0 to 2 do begin
        curNode^.m_quantizedAabbMin[k]   := leftChildNode^.m_quantizedAabbMin[k];
        if curNode^.m_quantizedAabbMin[k] > rightChildNode^.m_quantizedAabbMin[k] then begin
          curNode^.m_quantizedAabbMin[k] := rightChildNode^.m_quantizedAabbMin[k];
        end;
        curNode^.m_quantizedAabbMax[k]    := leftChildNode^.m_quantizedAabbMax[k];
        if curNode^.m_quantizedAabbMax[k] < rightChildNode^.m_quantizedAabbMax[k] then begin
          curNode^.m_quantizedAabbMax[k]  := rightChildNode^.m_quantizedAabbMax[k];
        end;
      end;
    end;
  end;
  if (curNodeSubPart >= 0) then begin
    triangles.unLockReadOnlyVertexBase(curNodeSubPart);
  end;
end;
{$HINTS ON}


{ btTriangleInfo }

procedure btTriangleInfo.Init;
begin
  m_edgeV0V1Angle := SIMD_2_PI;
  m_edgeV1V2Angle := SIMD_2_PI;
  m_edgeV2V0Angle := SIMD_2_PI;
  m_flags         := 0;
end;

{ btTriangleInfoMap }

procedure btTriangleInfoMap.Init;
begin
  m_convexEpsilon         := 0.00;
  m_planarEpsilon         := 0.0001;
  m_equalVertexThreshold  := btScalar(0.0001)*btScalar(0.0001);
  m_edgeDistanceThreshold := btScalar(0.1);
  m_zeroAreaThreshold     := btScalar(0.0001)*btScalar(0.0001);
end;

{ btBvhTriangleMeshShape }

constructor btBvhTriangleMeshShape.Create;
begin
  inherited Create(nil);
  m_bvh             := nil;
//  m_triangleInfoMap := nil;
  m_ownsBvh         := false;
  m_shapeType       := TRIANGLE_MESH_SHAPE_PROXYTYPE;
end;

constructor btBvhTriangleMeshShape.Create(const meshInterface: btStridingMeshInterface; const useQuantizedAabbCompression: boolean; const buildBvh: boolean);
begin
  inherited Create(meshInterface);
  m_bvh:=nil;
//  m_triangleInfoMap:=nil;
  m_useQuantizedAabbCompression := useQuantizedAabbCompression;
  m_ownsBvh   := false;
  m_shapeType := TRIANGLE_MESH_SHAPE_PROXYTYPE;
  //construct bvh from meshInterface
  {$ifndef DISABLE_BVH}
  if buildBvh then begin
    buildOptimizedBvh;
  end;
  {$endif} //DISABLE_BVH
end;

constructor btBvhTriangleMeshShape.Create(const meshInterface: btStridingMeshInterface; const useQuantizedAabbCompression: boolean; const bvhAabbMin, bvhAabbMax: btVector3; const buildBvh: boolean);
begin
  inherited Create(meshInterface);
  m_bvh:=nil;
//  m_triangleInfoMap:=nil;
  m_useQuantizedAabbCompression := useQuantizedAabbCompression;
  m_ownsBvh   := false;
  m_shapeType := TRIANGLE_MESH_SHAPE_PROXYTYPE;
  //construct bvh from meshInterface
  {$ifndef DISABLE_BVH}
    if buildBvh then begin
//      void* mem = btAlignedAlloc(sizeof(btOptimizedBvh),16);
      m_bvh := btOptimizedBvh.create;
      m_bvh.build(meshInterface,m_useQuantizedAabbCompression,bvhAabbMin,bvhAabbMax);
      m_ownsBvh := true;
    end;
  {$endif} //DISABLE_BVH
end;

destructor btBvhTriangleMeshShape.Destroy;
begin
  if m_ownsBvh then begin
    m_bvh.Free;
    m_bvh:=nil;
  end;
end;

function btBvhTriangleMeshShape.getOwnsBvh: boolean;
begin
  result := m_ownsBvh;
end;

type

  { TMyNodeOverlapCallback }

  TMyNodeOverlapCallback = class(btNodeOverlapCallback)
    m_meshInterface :btStridingMeshInterface;
    m_callback      :btTriangleCallback;
    constructor  Create(const callback:btTriangleCallback;const meshInterface:btStridingMeshInterface);
    procedure    processNode (const nodeSubPart:integer;const nodeTriangleIndex:integer);override;
  end;

{ TMyNodeOverlapCallback }

constructor TMyNodeOverlapCallback.Create(const callback: btTriangleCallback; const meshInterface: btStridingMeshInterface);
begin
  m_meshInterface := meshInterface;
  m_callback      := callback;
end;

{$HINTS OFF}
procedure TMyNodeOverlapCallback.processNode(const nodeSubPart: integer; const nodeTriangleIndex: integer);
var  m_triangle : array [0..2] of btVector3;
     vertexbase : Pointer;
     indexbase  : Pointer;
     numverts   : integer;
     typ        : PHY_ScalarType;
     stride     : integer;
     indexstride: integer;
     numfaces   : integer;
     indicestyp : PHY_ScalarType;
     meshScaling: PbtVector3;
     gfxbase    : PCardinal;
     j          : integer;
     graphicsindex : integer;

   procedure _Float;FOS_INLINE;
   var graphicsbase:PSingle;
   begin
     graphicsbase  := PSingle(vertexbase)+graphicsindex*stride;
     m_triangle[j] := btVector3.InitS(graphicsbase[0]*meshScaling^._X^,graphicsbase[1]*meshScaling^._Y^,graphicsbase[2]*meshScaling^._Z^);
   end;
   procedure _Double;FOS_INLINE;
   var graphicsbase:PDouble;
   begin
     graphicsbase  := PDouble(vertexbase)+graphicsindex*stride;
     m_triangle[j] := btVector3.InitS(graphicsbase[0]*meshScaling^._X^,graphicsbase[1]*meshScaling^._Y^,graphicsbase[2]*meshScaling^._Z^);
   end;

begin
  m_meshInterface.getLockedReadOnlyVertexIndexBase(vertexbase,numverts,typ,stride,indexbase,indexstride,numfaces,indicestyp,nodeSubPart);
  gfxbase := PCardinal(indexbase)+(nodeTriangleIndex*indexstride);
  btAssert((indicestyp=PHY_INTEGER) or (indicestyp=PHY_SHORT));
  meshScaling := m_meshInterface.getScalingP;
  for j:= 2 downto 0 do begin
    {$ifdef DEBUG_TRIANGLE_MESH}
      printf("%d ,",graphicsindex);
    {$endif} //DEBUG_TRIANGLE_MESH
    if indicestyp=PHY_SHORT then begin
      graphicsindex := PWord(gfxbase)[j];
    end else begin
      graphicsindex := gfxbase[j];
    end;
    if (typ = PHY_FLOAT) then begin
      _Float;
    end else begin
      _Double;
    end;
    {$ifdef DEBUG_TRIANGLE_MESH}
        printf("triangle vertices:%f,%f,%f\n",triangle[j].x(),triangle[j].y(),triangle[j].z());
    {$endif} //DEBUG_TRIANGLE_MESH
  end;
  // Perform ray vs. triangle collision here
  m_callback.processTriangle(m_triangle,nodeSubPart,nodeTriangleIndex);
  m_meshInterface.unLockReadOnlyVertexBase(nodeSubPart);
end;
{$HINTS ON}


procedure btBvhTriangleMeshShape.performRaycast(const callback: btTriangleCallback; const raySource, rayTarget: btVector3);
var myNodeCallback:TMyNodeOverlapCallback;
begin
  myNodeCallback := TMyNodeOverlapCallback.Create(callback,m_meshInterface);
  m_bvh.reportRayOverlappingNodex(myNodeCallback,raySource,rayTarget);
  myNodeCallback.free;
end;

procedure btBvhTriangleMeshShape.performConvexcast(const callback: btTriangleCallback; const boxSource, boxTarget, boxMin, boxMax: btVector3);
var myNodeCallback:TMyNodeOverlapCallback; //FOSHH in bullet this is copied twice ... :-(
begin
  myNodeCallback := TMyNodeOverlapCallback.Create(callback,m_meshInterface);
  m_bvh.reportBoxCastOverlappingNodex(myNodeCallback, boxSource, boxTarget, boxMin, boxMax);
  myNodeCallback.Free;
end;

procedure btBvhTriangleMeshShape.processAllTriangles(const callback: btTriangleCallback; const aabbMin, aabbMax: btVector3);
var myNodeCallback:TMyNodeOverlapCallback; //FOSHH in bullet this is copied trice ... :-(
begin
  {$ifdef DISABLE_BVH}
    //brute force traverse all triangles
    inherited processAllTriangles(callback,aabbMin,aabbMax);
  {$else}
  myNodeCallback := TMyNodeOverlapCallback.Create(callback,m_meshInterface);
  m_bvh.reportAabbOverlappingNodex(myNodeCallback,aabbMin,aabbMax);
  myNodeCallback.free;
  {$endif}
end;

procedure btBvhTriangleMeshShape.refitTree(const aabbMin, aabbMax: btVector3);
begin
  m_bvh.refit( m_meshInterface, aabbMin,aabbMax );
  recalcLocalAabb;
end;

procedure btBvhTriangleMeshShape.partialRefitTree(const aabbMin, aabbMax: btVector3);
begin
  m_bvh.refitPartial( m_meshInterface,aabbMin,aabbMax );
  m_localAabbMin.setMin(aabbMin);
  m_localAabbMax.setMax(aabbMax);
end;

function btBvhTriangleMeshShape.getName: String;
begin
  result:='BVHTRIANGLEMESH';
end;

procedure btBvhTriangleMeshShape.setLocalScaling(const scaling: btVector3);
begin
  if (getLocalScaling-scaling).length2 > SIMD_EPSILON then begin
    inherited setLocalScaling(scaling);
    buildOptimizedBvh;
  end;
end;

function btBvhTriangleMeshShape.getOptimizedBvh: btOptimizedBvh;
begin
  Result := m_bvh;
end;

procedure btBvhTriangleMeshShape.setOptimizedBvh(const bvh: btOptimizedBvh; const scaling: btVector3);
begin
  btAssert(not assigned(m_bvh));
  btAssert(not m_ownsBvh);

  m_bvh := bvh;
  m_ownsBvh := false;
  // update the scaling without rebuilding the bvh
  if (getLocalScaling-scaling).length2 > SIMD_EPSILON then begin
     inherited setLocalScaling(scaling);
  end;
end;

procedure btBvhTriangleMeshShape.buildOptimizedBvh;
begin
  if m_ownsBvh then begin
    m_bvh.Free;
    m_bvh:=nil;
  end;
  ///m_localAabbMin/m_localAabbMax is already re-calculated in btTriangleMeshShape. We could just scale aabb, but this needs some more work
  //void* mem = btAlignedAlloc(sizeof(btOptimizedBvh),16);
  //m_bvh = new(mem) btOptimizedBvh();
  m_bvh:=btOptimizedBvh.create;
  //rebuild the bvh...
  m_bvh.build(m_meshInterface,m_useQuantizedAabbCompression,m_localAabbMin,m_localAabbMax);
  m_ownsBvh := true;
end;

function btBvhTriangleMeshShape.usesQuantizedAabbCompression: boolean;
begin
  result:=m_useQuantizedAabbCompression;
end;

procedure btBvhTriangleMeshShape.setTriangleInfoMap(const triangleInfoMap: btTriangleInfoMap);
begin
  m_triangleInfoMap := triangleInfoMap;
end;

function btBvhTriangleMeshShape.getTriangleInfoMap: btTriangleInfoMap;
begin
  result:=m_triangleInfoMap;
end;



{ btEmptyShape }

constructor btEmptyShape.Create;
begin
  m_shapeType:=EMPTY_SHAPE_PROXYTYPE;
end;

procedure btEmptyShape.getAabb(const t: btTransform; out aabbMin,  aabbMax: btVector3);
var margin:btVector3;
begin
  margin.InitSame(getMargin);
  aabbMin := t.getOriginV^ - margin;
  aabbMax := t.getOriginV^ + margin;
end;

procedure btEmptyShape.setLocalScaling(const scaling: btVector3);
begin
 m_localScaling:=scaling;
end;

function btEmptyShape.getLocalScaling: btVector3;
begin
  Result:=m_localScaling;
end;




{ btUniformScalingShape }

constructor btUniformScalingShape.Create(const convexChildShape: btConvexShape; const uniformScalingFactor: btScalar);
begin
  inherited create;
  m_childConvexShape     := convexChildShape;
  m_uniformScalingFactor := uniformScalingFactor;
  m_shapeType            := UNIFORM_SCALING_SHAPE_PROXYTYPE;
end;

function btUniformScalingShape.localGetSupportingVertexWithoutMargin(const vec: btVector3): btVector3;
begin
  result := m_childConvexShape.localGetSupportingVertexWithoutMargin(vec)*m_uniformScalingFactor;
end;

function btUniformScalingShape.localGetSupportingVertex(const vec: btVector3): btVector3;
begin
  result := m_childConvexShape.localGetSupportingVertex(vec)*m_uniformScalingFactor;
end;

procedure btUniformScalingShape.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3;const numVectors: Integer);
var
  i: Integer;
begin
  m_childConvexShape.batchedUnitVectorGetSupportingVertexWithoutMargin(vectors,supportVerticesOut,numVectors);
  for i:=0 to numVectors-1 do begin
    supportVerticesOut[i] := supportVerticesOut[i] * m_uniformScalingFactor;
  end;
end;

{$HINTS OFF}
procedure btUniformScalingShape.calculateLocalInertia(const mass: btScalar;  var inertia: btVector3);
var   tmpInertia:btVector3;
begin
  ///this linear upscaling is not realistic, but we don't deal with large mass ratios...
  m_childConvexShape.calculateLocalInertia(mass,tmpInertia);
  inertia := tmpInertia * m_uniformScalingFactor;
end;
{$HINTS ON}

function btUniformScalingShape.getUniformScalingFactor: btScalar;
begin
  result:=m_uniformScalingFactor;
end;

function btUniformScalingShape.getChildShape: btConvexShape;
begin
  result:=m_childConvexShape;
end;

function btUniformScalingShape.getName: string;
begin
  result:='UniformScalingShape';
end;

procedure btUniformScalingShape.getAabb(const t: btTransform; out aabbMin,  aabbMax: btVector3);
begin
  getAabbSlow(t,aabbMin,aabbMax);
end;

procedure btUniformScalingShape.getAabbSlow(const t: btTransform; out aabbMin,  aabbMax: btVector3);
var scaledAabbHalfExtends,aabbCenter : btVector3;
begin
  m_childConvexShape.getAabb(t,aabbMin,aabbMax);
  aabbCenter            := (aabbMax+aabbMin)*btScalar(0.5);
  scaledAabbHalfExtends := (aabbMax-aabbMin)*btScalar(0.5)*m_uniformScalingFactor;
  aabbMin := aabbCenter - scaledAabbHalfExtends;
  aabbMax := aabbCenter + scaledAabbHalfExtends;
end;

procedure btUniformScalingShape.setLocalScaling(const scaling: btVector3);
begin
  m_childConvexShape.setLocalScaling(scaling);
end;

function btUniformScalingShape.getLocalScaling: btVector3;
begin
  result := m_childConvexShape.getLocalScaling;
end;

procedure btUniformScalingShape.setMargin(const margin: btScalar);
begin
  m_childConvexShape.setMargin(margin);
end;

function btUniformScalingShape.getMargin: btScalar;
begin
  Result := m_childConvexShape.getMargin * m_uniformScalingFactor;
end;

function btUniformScalingShape.getNumPreferredPenetrationDirections: integer;
begin
  Result := m_childConvexShape.getNumPreferredPenetrationDirections;
end;

procedure btUniformScalingShape.getPreferredPenetrationDirection(const index: integer; var penetrationVector: btVector3);
begin
  m_childConvexShape.getPreferredPenetrationDirection(index,penetrationVector);
end;

{ btMinkowskiSumShape }

constructor btMinkowskiSumShape.create(const shapeA, shapeB: btConvexShape);
begin
  inherited Create;
  m_shapeA    := shapeA;
  m_shapeB    := shapeB;
  m_shapeType := MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE;
  m_transA.setIdentity;
  m_transB.setIdentity;
end;

function btMinkowskiSumShape.localGetSupportingVertexWithoutMargin(const vec: btVector3): btVector3;
var supVertexA,supVertexB: btVector3;
begin
  supVertexA := m_transA.opTrans(m_shapeA.localGetSupportingVertexWithoutMargin(vec*m_transA.GetBasisV^));
  supVertexB := m_transB.opTrans(m_shapeB.localGetSupportingVertexWithoutMargin(-vec*m_transB.GetBasisV^));
  result     := supVertexA - supVertexB;
end;

procedure btMinkowskiSumShape.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);
var i: Integer;
begin
  ///@todo: could make recursive use of batching. probably this shape is not used frequently.
  for i:=0 to numVectors-1 do begin
    supportVerticesOut[i] := localGetSupportingVertexWithoutMargin(vectors[i]);
  end;
end;

{$HINTS OFF}
procedure btMinkowskiSumShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
begin
  //
  btAssert(false);
  inertia.InitSame(0);
end;
{$HINTS ON}

procedure btMinkowskiSumShape.setTransformA(const trans: btTransform);
begin
  m_transA := trans;
end;

procedure btMinkowskiSumShape.setTransformB(const trans: btTransform);
begin
  m_transB := trans;
end;

function btMinkowskiSumShape.getTransformA: btTransform;
begin
  result:=m_transA;
end;

function btMinkowskiSumShape.getTransformB: btTransform;
begin
  result:=m_transB;
end;

function btMinkowskiSumShape.getMargin: btScalar;
begin
  Result := m_shapeA.getMargin + m_shapeB.getMargin;
end;

function btMinkowskiSumShape.getShapeA: btConvexShape;
begin
  result:=m_shapeA;
end;

function btMinkowskiSumShape.getShapeB: btConvexShape;
begin
  result:=m_shapeB;
end;

function btMinkowskiSumShape.getName: string;
begin
  result:='MinkowskiSum';
end;

{ btMultiSphereShape }

constructor btMultiSphereShape.Create(const positions: PbtVector3; const radi: PbtScalar; const numSpheres: integer);
var
  i: Integer;
begin
  inherited create;
  m_shapeType := MULTI_SPHERE_SHAPE_PROXYTYPE;
  //btScalar startMargin = btScalar(BT_LARGE_FLOAT);
  m_localPositionArray.SetCapacity(numSpheres);
  m_radiArray.Setlength(numSpheres);
  for i:=0 to numSpheres-1 do begin
    m_localPositionArray.A[i]^ := positions[i];
    m_radiArray.A[i]^          := radi[i];
  end;
  recalcLocalAabb();
end;

{$HINTS OFF}
procedure btMultiSphereShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
var localAabbMin,localAabbMax,halfExtents:btVector3;
    lx,ly,lz:btScalar;
begin
  //as an approximation, take the inertia of the box that bounds the spheres
  getCachedLocalAabb(localAabbMin,localAabbMax);
  halfExtents := (localAabbMax-localAabbMin)*btScalar(0.5);
  lx:=2*halfExtents._x^;
  ly:=2*halfExtents._y^;
  lz:=2*halfExtents._z^;
  inertia.init(mass/(btScalar(12.0)) * (ly*ly + lz*lz),	mass/(btScalar(12.0)) * (lx*lx + lz*lz),mass/(btScalar(12.0)) * (lx*lx + ly*ly));
end;
{$HINTS ON}

function btMultiSphereShape.localGetSupportingVertexWithoutMargin(const vec0: btVector3): btVector3;
var i,numSpheres:integer;
    supVec,vec,vtx:btVector3;
    maxDot,lenSqr,rlen,newDot:btScalar;
    pos:PbtVector3;
    rad:PbtScalar;

begin
  supVec.InitSame(0);
  maxDot := -BT_LARGE_FLOAT;
  vec    := vec0;
  lenSqr := vec.length2;
  if lenSqr < (SIMD_EPSILON*SIMD_EPSILON) then begin
    vec.Init(1,0,0);
  end else begin
    rlen := 1 / btSqrt(lenSqr);
    vec *= rlen;
  end;

  pos := m_localPositionArray.A[0];
  rad := m_radiArray.A[0];
  numSpheres := m_localPositionArray.size;

  for i := 0  to  numSpheres-1 do begin
    vtx := pos^ +vec*m_localScaling*(rad^) - vec*getMargin;
    inc(pos);inc(rad);
    newDot := vec.dot(vtx);
    if newDot > maxDot then begin
      maxDot := newDot;
      supVec := vtx;
    end;
  end;
  result := supVec;
end;

procedure btMultiSphereShape.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);
var i,j,numSpheres:integer;
    vtx:btVector3;
    newDot,maxDot:btScalar;
    vec,pos:PbtVector3;
    rad:PbtScalar;

begin
  for j := 0 to numVectors-1 do begin
    maxDot := -BT_LARGE_FLOAT;
    vec    := @vectors[j];
    pos    := m_localPositionArray.A[0];
    rad    := m_radiArray.A[0];
    numSpheres := m_localPositionArray.size;
    for i:=0 to numSpheres-1 do begin
      vtx := pos^ +vec^*m_localScaling*(rad^) - vec^*getMargin;
      inc(pos);inc(rad);
      newDot := vec^.dot(vtx);
      if newDot > maxDot then begin
        maxDot := newDot;
        supportVerticesOut[j] := vtx;
      end;
    end;
  end;
end;

function btMultiSphereShape.getSphereCount: integer;
begin
  Result := m_localPositionArray.Size;
end;

function btMultiSphereShape.getSpherePosition(const index: integer): btVector3;
begin
  Result := m_localPositionArray.A[index]^;
end;

function btMultiSphereShape.getSphereRadius(const index: integer): btScalar;
begin
  Result := m_radiArray.A[index]^;
end;

function btMultiSphereShape.getName: string;
begin
  Result := 'MultiSphere';
end;

{ btBU_Simplex1to4 }

constructor btBU_Simplex1to4.Create;
begin
  inherited Create;
  m_numVertices := 0;
  m_shapeType   := TETRAHEDRAL_SHAPE_PROXYTYPE;
end;

constructor btBU_Simplex1to4.Create(const pt0: btVector3);
begin
  m_numVertices := 0;
  m_shapeType   := TETRAHEDRAL_SHAPE_PROXYTYPE;
  addVertex(pt0);
end;

constructor btBU_Simplex1to4.Create(const pt0, pt1: btVector3);
begin
  m_numVertices := 0;
  m_shapeType   := TETRAHEDRAL_SHAPE_PROXYTYPE;
  addVertex(pt0);
  addVertex(pt1);
end;

constructor btBU_Simplex1to4.Create(const pt0, pt1, pt2: btVector3);
begin
  m_numVertices := 0;
  m_shapeType   := TETRAHEDRAL_SHAPE_PROXYTYPE;
  addVertex(pt0);
  addVertex(pt1);
  addVertex(pt2);
end;

constructor btBU_Simplex1to4.Create(const pt0, pt1, pt2, pt3: btVector3);
begin
  m_numVertices := 0;
  m_shapeType   := TETRAHEDRAL_SHAPE_PROXYTYPE;
  addVertex(pt0);
  addVertex(pt1);
  addVertex(pt2);
  addVertex(pt3);
end;

procedure btBU_Simplex1to4.Reset;
begin
  m_numVertices := 0;
end;

procedure btBU_Simplex1to4.getAabb(const t: btTransform; out aabbMin, aabbMax: btVector3);
begin
{$if 1}
	inherited getAabb(t,aabbMin,aabbMax);
{$else}
	aabbMin.setValue(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	aabbMax.setValue(-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT);

	//just transform the vertices in worldspace, and take their AABB
	for (int i=0;i<m_numVertices;i++)
	{
		btVector3 worldVertex = t(m_vertices[i]);
		aabbMin.setMin(worldVertex);
		aabbMax.setMax(worldVertex);
	}
{$endif}
end;

procedure btBU_Simplex1to4.addVertex(const pt: btVector3);
begin
  m_vertices[m_numVertices] := pt;
  inc(m_numVertices);
  recalcLocalAabb;
end;

function btBU_Simplex1to4.getNumVertices: integer;
begin
  result := m_numVertices;
end;

function btBU_Simplex1to4.getNumEdges: integer;
begin
  //euler formula, F-E+V = 2, so E = F+V-2
  case m_numVertices of
     0,1: Result := 0;
     2:   Result := 1;
     3:   Result := 3;
     4:   Result := 6;
     else Result := 0;
  end;
end;

procedure btBU_Simplex1to4.getEdge(const i: integer; out pa, pb: btVector3);
begin
  case m_numVertices of
    2: begin
         pa := m_vertices[0];
         pb := m_vertices[1];
       end;
    3: begin
         case i of
           0: begin
                pa := m_vertices[0];
                pb := m_vertices[1];
              end;
           1: begin
                pa := m_vertices[1];
                pb := m_vertices[2];
              end;
           2: begin
                pa := m_vertices[2];
                pb := m_vertices[0];
           end;
         end;
       end;
    4: begin
         case i of
           0: begin
                pa := m_vertices[0];
                pb := m_vertices[1];
              end;
           1: begin
                pa := m_vertices[1];
                pb := m_vertices[2];
              end;
           2: begin
                pa := m_vertices[2];
                pb := m_vertices[0];
              end;
           3: begin
                pa := m_vertices[0];
                pb := m_vertices[3];
              end;
           4: begin
                pa := m_vertices[1];
                pb := m_vertices[3];
              end;
           5: begin
                pa := m_vertices[2];
                pb := m_vertices[3];
              end;
         end;
       end;
    end;
end;

procedure btBU_Simplex1to4.getVertex(const i: integer; out vtx: btVector3);
begin
  vtx := m_vertices[i];
end;

function btBU_Simplex1to4.getNumPlanes: integer;
begin
  case m_numVertices of
     0,1,2:	Result := 0;
     3: 	Result := 2;
     4: 	Result := 4;
     else       Result := 0;
  end;
end;

{$HINTS OFF}
procedure btBU_Simplex1to4.getPlane(var planeNormal, planeSupport: btVector3; const i: integer);
begin
  //Bullet not implemented
end;
{$HINTS ON}

{$HINTS OFF}
function btBU_Simplex1to4.getIndex(const i: integer): integer;
begin
 Result := 0;
end;
{$HINTS ON}

{$HINTS OFF}
function btBU_Simplex1to4.isInside(const pt: btVector3; const tolerance: btScalar): boolean;
begin
  result:=false;
end;
{$HINTS ON}

function btBU_Simplex1to4.getName: string;
begin
  result:='btBU_Simplex1to4';
end;

{ btCompoundShape }

constructor btCompoundShape.Create(enableDynamicAabbTree: boolean);
begin
  m_localAabbMin.InitSame(BT_LARGE_FLOAT);
  m_localAabbMax.InitSame(-BT_LARGE_FLOAT);
  m_dynamicAabbTree := nil;
  m_updateRevision  := 1;
  m_collisionMargin := 0;
  m_localScaling.InitSame(1);
  m_shapeType       := COMPOUND_SHAPE_PROXYTYPE;
  if enableDynamicAabbTree then begin
//    void* mem = btAlignedAlloc(sizeof(btDbvt),16);
//    btAssert(mem==m_dynamicAabbTree);
    m_dynamicAabbTree := btDbvt.Create;
  end;
end;

destructor btCompoundShape.Destroy;
begin
  if assigned(m_dynamicAabbTree) then begin
    m_dynamicAabbTree.Free;
    m_dynamicAabbTree:=nil;
  end;
  inherited;
end;

{$HINTS OFF}
procedure btCompoundShape.addChildShape(const localTransform: btTransform; const shape: btCollisionShape);
var child:btCompoundShapeChild;
    localAabbMin,localAabbMax:btVector3;
    i: Integer;
    bounds : btDbvtVolume;
    index  : integer;
begin
  inc(m_updateRevision);
  //m_childTransforms.push_back(localTransform);
  //m_childShapes.push_back(shape);
  child.m_node           := nil;
  child.m_transform      := localTransform;
  child.m_childShape     := shape;
  child.m_childShapeType := shape.getShapeType;
  child.m_childMargin    := shape.getMargin;

  //extend the local aabbMin/aabbMax
  shape.getAabb(localTransform,localAabbMin,localAabbMax);
  for i:=0 to 2 do begin
    if m_localAabbMin[i] > localAabbMin[i] then begin
      m_localAabbMin[i] := localAabbMin[i];
    end;
    if (m_localAabbMax[i] < localAabbMax[i]) then begin
      m_localAabbMax[i] := localAabbMax[i];
    end;
  end;
  if assigned(m_dynamicAabbTree) then begin
    bounds.FromMM(localAabbMin,localAabbMax);
    index := m_children.size();
    child.m_node := m_dynamicAabbTree.insert(bounds,pointer(index));
  end;
  m_children.push_back(child);
end;
{$HINTS ON}

procedure btCompoundShape.removeChildShape(const shape: btCollisionShape);
var  i: Integer;
begin
  inc(m_updateRevision);
  // Find the children containing the shape specified, and remove those children.
  // note: there might be multiple children using the same shape !
  for i := m_children.size-1 downto 0 do begin
    if(m_children.A[i]^.m_childShape = shape) then begin
      removeChildShapeByIndex(i);
    end;
  end;
  recalculateLocalAabb;
end;

procedure btCompoundShape.removeChildShapeByIndex(const childShapeindex: integer);
begin
  inc(m_updateRevision);
  btAssert((childShapeIndex>=0) and (childShapeIndex<m_children.size));
  if assigned(m_dynamicAabbTree) then begin
    m_dynamicAabbTree.remove(m_children.A[childShapeIndex]^.m_node);
  end;
  m_children.swap(childShapeIndex,m_children.size-1);
  if assigned(m_dynamicAabbTree) then begin
    m_children.A[childShapeIndex]^.m_node^.int.dataAsInt := childShapeIndex;
  end;
  m_children.pop_back;
end;

function btCompoundShape.getNumChildShapes: integer;
begin
 result:=m_children.size;
end;

function btCompoundShape.getChildShape(const index: integer): btCollisionShape;
begin
  Result:=m_children.A[index]^.m_childShape;
end;

function btCompoundShape.getChildTransform(const index: integer): btTransform;
begin
  result := m_children.A[index]^.m_transform;
end;

function btCompoundShape.getChildTransformP(const index: integer): PbtTransform;
begin
  result := @m_children.A[index]^.m_transform;
end;

{$HINTS OFF}
procedure btCompoundShape.updateChildTransform(const childIndex: integer; const newChildTransform: btTransform);
var localAabbMin,localAabbMax:btVector3;
    bounds : btDbvtVolume;
begin
  m_children.A[childIndex]^.m_transform := newChildTransform;
  if assigned(m_dynamicAabbTree) then begin
    ///update the dynamic aabb tree
    m_children.A[childIndex]^.m_childShape.getAabb(newChildTransform,localAabbMin,localAabbMax);
    bounds.FromMM(localAabbMin,localAabbMax);
    //int index = m_children.size()-1;
    m_dynamicAabbTree.update(m_children.A[childIndex]^.m_node,bounds);
  end;
  recalculateLocalAabb;
end;
{$HINTS ON}

function btCompoundShape.getChildList: PbtCompoundShapeChild;
begin
  Result := m_children.A[0];
end;

procedure btCompoundShape.getAabb(const t: btTransform; out aabbMin, aabbMax: btVector3);
var localHalfExtents,localCenter,center,extent:btVector3;
    abs_b:btMatrix3x3;
begin
  localHalfExtents := btScalar(0.5)*(m_localAabbMax-m_localAabbMin);
  localCenter      := btScalar(0.5)*(m_localAabbMax+m_localAabbMin);
  //avoid an illegal AABB when there are no children
  if (m_children.size=0) then begin
    localHalfExtents.InitSame(0);
    localCenter.InitSame(0);
  end;
  localHalfExtents += btVector3.InitSameS(getMargin);
  abs_b            := t.GetBasisV^.absolute;
  center           := t.opTrans(localCenter);

  extent.init(abs_b[0].dot(localHalfExtents),abs_b[1].dot(localHalfExtents),abs_b[2].dot(localHalfExtents));
  aabbMin := center-extent;
  aabbMax := center+extent;
end;

{$HINTS OFF}
procedure btCompoundShape.recalculateLocalAabb;
var     localAabbMin,localAabbMax : btVector3;
        j: Integer;
        i: Integer;
begin
  // Recalculate the local aabb
  // Brute force, it iterates over all the shapes left.
  m_localAabbMin.InitSame(BT_LARGE_FLOAT);
  m_localAabbMax.InitSame(-BT_LARGE_FLOAT);
  //extend the local aabbMin/aabbMax
  for j := 0 to  m_children.size-1 do begin
    m_children.A[j]^.m_childShape.getAabb(m_children.A[j]^.m_transform, localAabbMin, localAabbMax);
    for i:= 0 to 2 do begin
      if m_localAabbMin[i] > localAabbMin[i]  then begin
        m_localAabbMin[i] := localAabbMin[i];
      end;
      if m_localAabbMax[i] < localAabbMax[i] then begin
        m_localAabbMax[i] := localAabbMax[i];
      end;
    end;
  end;
end;
{$HINTS ON}

procedure btCompoundShape.setLocalScaling(const scaling: btVector3);
var childTrans : btTransform;
    childScale : btVector3;
    i          : integer;
begin
  for i := 0 to m_children.size-1 do begin
    childTrans := getChildTransform(i);
    childScale := m_children.A[i]^.m_childShape.getLocalScaling;
//            childScale = childScale * (childTrans.GetBasisV^() * scaling);
    childScale := childScale * scaling / m_localScaling;
    m_children.A[i]^.m_childShape.setLocalScaling(childScale);
    childTrans.setOrigin((childTrans.getOriginV^)*scaling);
    updateChildTransform(i, childTrans);
    recalculateLocalAabb;
  end;
  m_localScaling := scaling;
end;

function btCompoundShape.getLocalScaling: btVector3;
begin
  result := m_localScaling;
end;

procedure btCompoundShape.calculateLocalInertia(const mass: btScalar; var inertia: btVector3);
var    ident:btTransform;
       halfExtents,aabbMin,aabbMax:btVector3;
       lx,ly,lz : btScalar;
begin
  //approximation: take the inertia from the aabb for now
  ident.setIdentity;
  {$HINTS OFF}
  getAabb(ident,aabbMin,aabbMax);
  {$HINTS ON}
  halfExtents := (aabbMax-aabbMin)*btScalar(0.5);
  lx := btScalar(2)*(halfExtents._x^);
  ly := btScalar(2)*(halfExtents._y^);
  lz := btScalar(2)*(halfExtents._z^);
  inertia[0] := mass/(btScalar(12.0)) * (ly*ly + lz*lz);
  inertia[1] := mass/(btScalar(12.0)) * (lx*lx + lz*lz);
  inertia[2] := mass/(btScalar(12.0)) * (lx*lx + ly*ly);
end;

procedure btCompoundShape.setMargin(const margin: btScalar);
begin
  m_collisionMargin := margin;
end;

function btCompoundShape.getMargin: btScalar;
begin
  Result:=m_collisionMargin;
end;

function btCompoundShape.getName: String;
begin
  result:='Compound';
end;

function btCompoundShape.getDynamicAabbTree: btDbvt;
begin
  Result := m_dynamicAabbTree;
end;

procedure btCompoundShape.createAabbTreeFromChildren;
var child:PbtCompoundShapeChild;
    localAabbMin,localAabbMax : btVector3;
    bounds:btDbvtVolume;
    index: Integer;
begin
  if not assigned(m_dynamicAabbTree) then begin
      //void* mem = btAlignedAlloc(sizeof(btDbvt),16);
      //m_dynamicAabbTree = new(mem) btDbvt();
      //btAssert(mem==m_dynamicAabbTree);
    m_dynamicAabbTree := btDbvt.Create;
    for index := 0 to  m_children.size-1 do begin
      child := m_children.A[index];
      //extend the local aabbMin/aabbMax
      {$HINTS OFF}
      child^.m_childShape.getAabb(child^.m_transform,localAabbMin,localAabbMax);
      {$HINTS ON}
      bounds.FromMM(localAabbMin,localAabbMax);
      child^.m_node := m_dynamicAabbTree.insert(bounds,@index);
    end;
  end;
end;

procedure btCompoundShape.calculatePrincipalAxisTransform(const masses: PbtScalar; const principal: btTransform; var inertia: btVector3);
var n,k:integer;
    totalMass,o2:btScalar;
    i,o,center:btVector3;
    tensor,j:btMatrix3x3;
    t:PbtTransform;
begin
  n := m_children.size;
  totalMass := 0;
  center.InitSame(0);
  for k := 0 to n-1 do begin
    btAssert(masses[k]>0);
    center += m_children.A[k]^.m_transform.getOriginV^ * masses[k];
    totalMass += masses[k];
  end;

  btAssert(totalMass>0);

  center /= totalMass;
  principal.setOrigin(center);
  tensor.init(0, 0, 0, 0, 0, 0, 0, 0, 0);
  for k := 0 to n-1 do begin
    {$HINTS OFF}
    m_children.A[k]^.m_childShape.calculateLocalInertia(masses[k], i);
    {$HINTS ON}
    t := @m_children.A[k]^.m_transform;
    o := t^.getOriginV^-center;
    //compute inertia tensor in coordinate system of compound shape
    j := t^.GetBasisV^.transpose;
    j.RowsP[0]^ *= i[0];
    j.RowsP[1]^ *= i[1];
    j.RowsP[2]^ *= i[2];
    j := t^.GetBasisV^ * j;

    //add inertia tensor
    tensor.RowsP[0]^ += j[0];
    tensor.RowsP[1]^ += j[1];
    tensor.RowsP[2]^ += j[2];

    //compute inertia tensor of pointmass at o
    o2  := o.length2;
    j.RowsP[0]^.Init(o2, 0, 0);
    j.RowsP[1]^.Init(0, o2, 0);
    j.RowsP[2]^.Init(0, 0, o2);
    j.RowsP[0]^ += o * -o._x^;
    j.RowsP[1]^ += o * -o._y^;
    j.RowsP[2]^ += o * -o._z^;

    //add inertia tensor of pointmass
    tensor.RowsP[0]^ += masses[k] * j[0];
    tensor.RowsP[1]^ += masses[k] * j[1];
    tensor.RowsP[2]^ += masses[k] * j[2];
  end;
  tensor.diagonalize(principal.GetBasisV^, btScalar(0.00001), 20);
  inertia.init(tensor[0][0], tensor[1][1], tensor[2][2]);
end;

function btCompoundShape.getUpdateRevision: integer;
begin
  result := m_updateRevision;
end;

{ btConvexTriangleMeshShape }

constructor btConvexTriangleMeshShape.Create(const meshInterface: btStridingMeshInterface; const calcAabb: boolean);
begin
  inherited Create;
  m_stridingMesh := meshInterface;
  m_shapeType    := CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
  if calcAabb then recalcLocalAabb;
end;

function btConvexTriangleMeshShape.getMeshInterface: btStridingMeshInterface;
begin
  result := m_stridingMesh;
end;

function btConvexTriangleMeshShape.localGetSupportingVertex(const vec: btVector3): btVector3;
var vecNorm,supVertex : btVector3;
begin
  supVertex := localGetSupportingVertexWithoutMargin(vec);
  if getMargin<>0 then begin
    vecnorm := vec;
    vecNorm.length2NormCheck;
    vecNorm.normalize;
    supVertex += getMargin * vecnorm;
  end;
  Result := supVertex;
end;


/////It's not nice to have all this virtual function overhead, so perhaps we can also gather the points once
/////but then we are duplicating
{ SupportVertexCallback }
type
    { TLocalSupportVertexCallback }
    TLocalSupportVertexCallback=class(btInternalTriangleIndexCallback)
      m_supportVertexLocal:btVector3;
    public
      m_maxDot          : btScalar;
      m_supportVecLocal : btVector3;
      procedure   Init                         (const supportVecLocal:btVector3);
      procedure   internalProcessTriangleIndex (const triangle: PbtVector3; const partId, triangleIndex: integer);override;
      function    GetSupportVertexLocal        : btVector3;
    end;

  procedure TLocalSupportVertexCallback.Init (const supportVecLocal: btVector3);
  begin
    m_supportVertexLocal.InitSame(0);
    m_maxDot          := -BT_LARGE_FLOAT;
    m_supportVecLocal := supportVecLocal;
  end;

  {$HINTS OFF}
  procedure TLocalSupportVertexCallback.internalProcessTriangleIndex(const triangle: PbtVector3; const partId, triangleIndex: integer);
  var dot : btScalar;
        i : Integer;
  begin
  //  (void)triangleIndex;
  //  (void)partId;
    for i:=0 to 2 do begin
      dot := m_supportVecLocal.dot(triangle[i]);
      if (dot > m_maxDot) then begin
        m_maxDot := dot;
        m_supportVertexLocal := triangle[i];
      end;
    end;
  end;
  {$HINTS ON}

function TLocalSupportVertexCallback.GetSupportVertexLocal: btVector3;
begin
  result := m_supportVertexLocal;
end;


function btConvexTriangleMeshShape.localGetSupportingVertexWithoutMargin(const vec0: btVector3): btVector3;
var vec,supVec,aabbMax : btVector3;
    rlen,lenSqr : btScalar;
    supportCallback : TLocalSupportVertexCallback;
begin
  supVec.zero;
  vec    := vec0;
  lenSqr := vec.length2();
  if lenSqr < btScalar(0.0001) then begin
    vec.Init(1,0,0);
  end else begin
    rlen := 1 / btSqrt(lenSqr);
    vec *= rlen;
  end;
  supportCallback := TLocalSupportVertexCallback.create;
  supportCallback.Init(vec);
  aabbMax.initsame(BT_LARGE_FLOAT);
  m_stridingMesh.InternalProcessAllTriangles(supportCallback,-aabbMax,aabbMax);
  supVec := supportCallback.GetSupportVertexLocal;
  Result := supVec;
  supportCallback.Free;
end;

procedure btConvexTriangleMeshShape.batchedUnitVectorGetSupportingVertexWithoutMargin(const vectors: PbtVector3; const supportVerticesOut: PbtVector3; const numVectors: Integer);
var i,j:integer;
    vec,aabbMax:btVector3;
    supportCallback : TLocalSupportVertexCallback;
begin
  //use 'w' component of supportVerticesOut?
  for i := 0 to numVectors-1 do begin
    supportVerticesOut[i][3] := btScalar(-BT_LARGE_FLOAT);
  end;
  ///@todo: could do the batch inside the callback!
  supportCallback := TLocalSupportVertexCallback.Create;
  for j:=0 to numVectors-1 do begin
    vec := vectors[j];
    supportCallback.Init(vec);
    aabbMax.InitSame(BT_LARGE_FLOAT);
    m_stridingMesh.InternalProcessAllTriangles(supportCallback,-aabbMax,aabbMax);
    supportVerticesOut[j] := supportCallback.GetSupportVertexLocal;
  end;
  supportCallback.Free;
end;

function btConvexTriangleMeshShape.getName: String;
begin
  Result := 'ConvexTrimesh';
end;

//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you can debug-draw btConvexTriangleMeshShape with the Raytracer Demo
function btConvexTriangleMeshShape.getNumVertices: integer;
begin
  result := 0;
end;

function btConvexTriangleMeshShape.getNumEdges: integer;
begin
  result:=0;
end;

{$HINTS OFF}
procedure btConvexTriangleMeshShape.getEdge(const i: integer; out pa, pb: btVector3);
begin
  //.
  btAssert(false);
end;

procedure btConvexTriangleMeshShape.getVertex(const i: integer; out vtx: btVector3);
begin
  //.
  btAssert(false);
end;
{$HINTS ON}


function btConvexTriangleMeshShape.getNumPlanes: integer;
begin
  Result := 0;
end;

{$HINTS OFF}
procedure btConvexTriangleMeshShape.getPlane(var planeNormal, planeSupport: btVector3; const i: integer);
begin
  //.
  btAssert(false);
end;

//not yet
function btConvexTriangleMeshShape.isInside(const pt: btVector3; const tolerance: btScalar): boolean;
begin
  //.
  btAssert(false);
  result:=false;
end;
{$HINTS ON}

procedure btConvexTriangleMeshShape.setLocalScaling(const scaling: btVector3);
begin
  m_stridingMesh.setScaling(scaling);
  recalcLocalAabb;
end;

function btConvexTriangleMeshShape.getLocalScaling: btVector3;
begin
 result := m_stridingMesh.getScaling;
end;


//procedure btConvexTriangleMeshShape.calculatePrincipalAxisTransform(const principal: btTransform; var inertia: btVector3; var volume: btScalar);
//begin
// abort; // implement
// //class CenterCallback: public btInternalTriangleIndexCallback
// //{
// //   bool first;
// //   btVector3 ref;
// //   btVector3 sum;
// //   btScalar volume;
// //
// //public:
// //
// //   CenterCallback() : first(true), ref(0, 0, 0), sum(0, 0, 0), volume(0)
// //   {
// //   }
// //
// //   virtual void internalProcessTriangleIndex(btVector3* triangle, int partId, int triangleIndex)
// //   {
// //      (void) triangleIndex;
// //      (void) partId;
// //      if (first)
// //      {
// //         ref = triangle[0];
// //         first = false;
// //      }
// //      else
// //      {
// //         btScalar vol = btFabs((triangle[0] - ref).triple(triangle[1] - ref, triangle[2] - ref));
// //         sum += (btScalar(0.25) * vol) * ((triangle[0] + triangle[1] + triangle[2] + ref));
// //         volume += vol;
// //      }
// //   }
// //
// //   btVector3 getCenter()
// //   {
// //      return (volume > 0) ? sum / volume : ref;
// //   }
// //
// //   btScalar getVolume()
// //   {
// //      return volume * btScalar(1. / 6);
// //   }
// //
// //};
// //
// //class InertiaCallback: public btInternalTriangleIndexCallback
// //{
// //   btMatrix3x3 sum;
// //   btVector3 center;
// //
// //public:
// //
// //   InertiaCallback(btVector3& center) : sum(0, 0, 0, 0, 0, 0, 0, 0, 0), center(center)
// //   {
// //   }
// //
// //   virtual void internalProcessTriangleIndex(btVector3* triangle, int partId, int triangleIndex)
// //   {
// //      (void) triangleIndex;
// //      (void) partId;
// //      btMatrix3x3 i;
// //      btVector3 a = triangle[0] - center;
// //      btVector3 b = triangle[1] - center;
// //      btVector3 c = triangle[2] - center;
// //      btScalar volNeg = -btFabs(a.triple(b, c)) * btScalar(1. / 6);
// //      for (int j = 0; j < 3; j++)
// //      {
// //         for (int k = 0; k <= j; k++)
// //         {
// //            i[j][k] = i[k][j] = volNeg * (btScalar(0.1) * (a[j] * a[k] + b[j] * b[k] + c[j] * c[k])
// //               + btScalar(0.05) * (a[j] * b[k] + a[k] * b[j] + a[j] * c[k] + a[k] * c[j] + b[j] * c[k] + b[k] * c[j]));
// //         }
// //      }
// //      btScalar i00 = -i[0][0];
// //      btScalar i11 = -i[1][1];
// //      btScalar i22 = -i[2][2];
// //      i[0][0] = i11 + i22;
// //      i[1][1] = i22 + i00;
// //      i[2][2] = i00 + i11;
// //      sum[0] += i[0];
// //      sum[1] += i[1];
// //      sum[2] += i[2];
// //   }
// //
// //   btMatrix3x3& getInertia()
// //   {
// //      return sum;
// //   }
// //
// //};
// //
// //CenterCallback centerCallback;
// //btVector3 aabbMax(btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT));
// //m_stridingMesh->InternalProcessAllTriangles(&centerCallback, -aabbMax, aabbMax);
// //btVector3 center = centerCallback.getCenter();
// //principal.setOrigin(center);
// //volume = centerCallback.getVolume();
// //
// //InertiaCallback inertiaCallback(center);
// //m_stridingMesh->InternalProcessAllTriangles(&inertiaCallback, -aabbMax, aabbMax);
// //
// //btMatrix3x3& i = inertiaCallback.getInertia();
// //i.diagonalize(principal.GetBasisV^(), btScalar(0.00001), 20);
// //inertia.setValue(i[0][0], i[1][1], i[2][2]);
// //inertia /= volume;
//end;

{ btShapeHull }

procedure btShapeHull._StaticInit;
begin
  sUnitSpherePoints[ 0].Init(btScalar(0.000000) , btScalar(-0.000000),btScalar(-1.000000));
  sUnitSpherePoints[ 1].Init(btScalar(0.723608) , btScalar(-0.525725),btScalar(-0.447219));
  sUnitSpherePoints[ 2].Init(btScalar(-0.276388) , btScalar(-0.850649),btScalar(-0.447219));
  sUnitSpherePoints[ 3].Init(btScalar(-0.894426) , btScalar(-0.000000),btScalar(-0.447216));
  sUnitSpherePoints[ 4].Init(btScalar(-0.276388) , btScalar(0.850649),btScalar(-0.447220));
  sUnitSpherePoints[ 5].Init(btScalar(0.723608) , btScalar(0.525725),btScalar(-0.447219));
  sUnitSpherePoints[ 6].Init(btScalar(0.276388) , btScalar(-0.850649),btScalar(0.447220));
  sUnitSpherePoints[ 7].Init(btScalar(-0.723608) , btScalar(-0.525725),btScalar(0.447219));
  sUnitSpherePoints[ 8].Init(btScalar(-0.723608) , btScalar(0.525725),btScalar(0.447219));
  sUnitSpherePoints[ 9].Init(btScalar(0.276388) , btScalar(0.850649),btScalar(0.447219));
  sUnitSpherePoints[10].Init(btScalar(0.894426) , btScalar(0.000000),btScalar(0.447216));
  sUnitSpherePoints[11].Init(btScalar(-0.000000) , btScalar(0.000000),btScalar(1.000000));
  sUnitSpherePoints[12].Init(btScalar(0.425323) , btScalar(-0.309011),btScalar(-0.850654));
  sUnitSpherePoints[13].Init(btScalar(-0.162456) , btScalar(-0.499995),btScalar(-0.850654));
  sUnitSpherePoints[14].Init(btScalar(0.262869) , btScalar(-0.809012),btScalar(-0.525738));
  sUnitSpherePoints[15].Init(btScalar(0.425323) , btScalar(0.309011),btScalar(-0.850654));
  sUnitSpherePoints[16].Init(btScalar(0.850648) , btScalar(-0.000000),btScalar(-0.525736));
  sUnitSpherePoints[17].Init(btScalar(-0.525730) , btScalar(-0.000000),btScalar(-0.850652));
  sUnitSpherePoints[18].Init(btScalar(-0.688190) , btScalar(-0.499997),btScalar(-0.525736));
  sUnitSpherePoints[19].Init(btScalar(-0.162456) , btScalar(0.499995),btScalar(-0.850654));
  sUnitSpherePoints[20].Init(btScalar(-0.688190) , btScalar(0.499997),btScalar(-0.525736));
  sUnitSpherePoints[21].Init(btScalar(0.262869) , btScalar(0.809012),btScalar(-0.525738));
  sUnitSpherePoints[22].Init(btScalar(0.951058) , btScalar(0.309013),btScalar(0.000000));
  sUnitSpherePoints[23].Init(btScalar(0.951058) , btScalar(-0.309013),btScalar(0.000000));
  sUnitSpherePoints[24].Init(btScalar(0.587786) , btScalar(-0.809017),btScalar(0.000000));
  sUnitSpherePoints[25].Init(btScalar(0.000000) , btScalar(-1.000000),btScalar(0.000000));
  sUnitSpherePoints[26].Init(btScalar(-0.587786) , btScalar(-0.809017),btScalar(0.000000));
  sUnitSpherePoints[27].Init(btScalar(-0.951058) , btScalar(-0.309013),btScalar(-0.000000));
  sUnitSpherePoints[28].Init(btScalar(-0.951058) , btScalar(0.309013),btScalar(-0.000000));
  sUnitSpherePoints[29].Init(btScalar(-0.587786) , btScalar(0.809017),btScalar(-0.000000));
  sUnitSpherePoints[30].Init(btScalar(-0.000000) , btScalar(1.000000),btScalar(-0.000000));
  sUnitSpherePoints[31].Init(btScalar(0.587786) , btScalar(0.809017),btScalar(-0.000000));
  sUnitSpherePoints[32].Init(btScalar(0.688190) , btScalar(-0.499997),btScalar(0.525736));
  sUnitSpherePoints[33].Init(btScalar(-0.262869) , btScalar(-0.809012),btScalar(0.525738));
  sUnitSpherePoints[34].Init(btScalar(-0.850648) , btScalar(0.000000),btScalar(0.525736));
  sUnitSpherePoints[35].Init(btScalar(-0.262869) , btScalar(0.809012),btScalar(0.525738));
  sUnitSpherePoints[36].Init(btScalar(0.688190) , btScalar(0.499997),btScalar(0.525736));
  sUnitSpherePoints[37].Init(btScalar(0.525730) , btScalar(0.000000),btScalar(0.850652));
  sUnitSpherePoints[38].Init(btScalar(0.162456) , btScalar(-0.499995),btScalar(0.850654));
  sUnitSpherePoints[39].Init(btScalar(-0.425323) , btScalar(-0.309011),btScalar(0.850654));
  sUnitSpherePoints[40].Init(btScalar(-0.425323) , btScalar(0.309011),btScalar(0.850654));
  sUnitSpherePoints[41].Init(btScalar(0.162456) , btScalar(0.499995),btScalar(0.850654));
end;

constructor btShapeHull.Create(const shape: btConvexShape);
begin
  _StaticInit;
  m_vertices := btFOSAlignedVectorArray.create ; m_vertices.Initialize(nil,16);
  m_indices  := btFOSAlignedCardinals.create   ; m_indices.Initialize(nil,16);
  m_shape    := shape;
  m_numIndices := 0;
end;

destructor btShapeHull.Destroy;
begin
  m_vertices.free;
  m_indices.free;
  inherited Destroy;
end;

function btShapeHull.getUnitSpherePoints: PbtVector3;
begin
  result:=@sUnitSpherePoints[0];
end;


{$HINTS OFF}
function btShapeHull.buildHull(const margin: btScalar): boolean;
var numSampleDirections,numPDA,i : Integer;
    norm                         : btVector3;
    supportPoints                : array [0..(NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2)-1] of btVector3;
    hd                           : btHullDesc;
    hl                           : btHullLibrary;
    hr                           : btHullResult;
begin
 numSampleDirections := NUM_UNITSPHERE_POINTS;
  numPDA := m_shape.getNumPreferredPenetrationDirections;
  if numPDA<>0 then begin
    for i :=0 to numPDA-1 do begin
      norm.zero; //TODO - WARNING REMOVER
      m_shape.getPreferredPenetrationDirection(i,norm);
      getUnitSpherePoints[numSampleDirections] := norm;
      inc(numSampleDirections);
    end;
  end;
  for i := 0 to numSampleDirections-1 do begin
    supportPoints[i] := m_shape.localGetSupportingVertex(getUnitSpherePoints[i]);
  end;
  hd.Init;
  hd.mFlags  := [QF_TRIANGLES];
  hd.mVcount := numSampleDirections;
  hd.mVertices     := supportPoints;
  hd.mVertexStride := sizeof(btVector3);
  hr:=btHullResult.create;
  hl:=btHullLibrary.create;
  if (hl.CreateConvexHull (hd, hr) = QE_FAIL) then begin
    Result := false;
    hl.free;
    exit;
  end;
  m_vertices.resize(hr.mNumOutputVertices);
  for i := 0 to hr.mNumOutputVertices-1 do begin
    m_vertices.A[i]^ := hr.m_OutputVertices.A[i]^;
  end;
  m_numIndices := hr.mNumIndices;
  m_indices.resize(m_numIndices);
  for i := 0 to m_numIndices-1 do begin
    m_indices.A[i]^ := hr.m_Indices.A[i]^;
  end;
   // free temporary hull result that we just copied
  hl.ReleaseResult(hr);
  hl.free;
  Result := true;
end;
{$HINTS ON}


function btShapeHull.numTriangles: integer;
begin
 result := m_numIndices div 3;
end;

function btShapeHull.numVertices: integer;
begin
 result := m_vertices.Size;
end;

function btShapeHull.numIndices: integer;
begin
  result := m_numIndices;
end;

function btShapeHull.getVertexPointer: PbtVector3;
begin
 result := m_vertices.A[0];
end;

function btShapeHull.getVertexPtrIDX(const n: cardinal): PbtVector3;
begin
 result := m_vertices.A[n];
end;

function btShapeHull.getIndexPointer: PbtCardinal16;
begin
 result := PbtCardinal16(m_indices.A[0]);
end;

initialization

end.

