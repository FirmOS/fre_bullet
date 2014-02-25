unit btNarrowphase;

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
// If you want to use this code you need to comply to the "original" licence and the following BSD Style Licence.
//
// Copyright (c) 2000-2014, Helmut Hartl
//
// FirmOS Business Solutions GmbH. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
//
//   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
//      in the documentation and/or other materials provided with the distribution.
//   3. Neither the name of FirmOS nor the names of its contributors may be used to endorse or promote
//      products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

// FILES
// +btManifoldPoint.h
// +btPersistentManifold.h
// +btPersistentManifold.cpp
// +btDiscreteCollisionDetectorInterface.h
// +btGjkPairDetector.h
// +btGjkPairDetector.cpp
// +btSimplexSolverInterface.h -> per #define non virtual mapped to Voronoi
// +btVoronoiSimplexSolver.h
// +btVoronoiSimplexSolver.cpp
// +btConvexPenetrationDepthSolver.h
// +btConvexCast.h
// +btConvexCast.cpp
// +btGjkConvexCast.h
// +btGjkConvexCast.cpp
// +btPointCollector.h
// +btSubSimplexConvexCast.h
// +btSubSimplexConvexCast.cpp
// +btGjkEpaPenetrationDepthSolver.h
// +btGjkEpaPenetrationDepthSolver.cpp
// +btGjkEpa2.h
// +btGjkEpa2.cpp
// +btMinkowskiPenetrationDepthSolver.h DELAYED
// +btMinkowskiPenetrationDepthSolver.cpp DELAYED (code copied)
// +btRaycatCallback.h
// +btRaycatCallback.cpp
// +btContinuosConvexCollision.h
// +btContinuosConvexCollision.cpp

interface

{$i fos_bullet.inc}

uses
  Sysutils,FOS_AlignedArray,btLinearMath,btBroadphase,btCollisionShapes;

type

/// ManifoldContactPoint collects and maintains persistent contactpoints.
/// used to improve stability and performance of rigidbody dynamics response.

    { btOManifoldPoint }

    btOManifoldPoint=object
      m_localPointA                : btVector3;
      m_localPointB                : btVector3;
      m_positionWorldOnB           : btVector3;
      ///m_positionWorldOnA is redundant information, see getPositionWorldOnA(), but for clarity
      m_positionWorldOnA           : btVector3;
      m_normalWorldOnB             : btVector3;
      m_distance1                  : btScalar;
      m_combinedFriction           : btScalar;
      m_combinedRestitution        : btScalar;
      //BP mod, store contact triangles.
      m_partId0                    : integer;
      m_partId1                    : integer;
      m_index0                     : integer;
      m_index1                     : integer;
      m_userPersistentData         : pointer;
      m_appliedImpulse             : btScalar;
      m_lateralFrictionInitialized : boolean;
      m_appliedImpulseLateral1     : btScalar;
      m_appliedImpulseLateral2     : btScalar;
      m_contactMotion1             : btScalar;
      m_contactMotion2             : btScalar;
      m_contactCFM1                : btScalar;
      m_contactCFM2                : btScalar;
      m_lifeTime                   : integer;//lifetime of the contactpoint in frames
      m_lateralFrictionDir1        : btVector3;
      m_lateralFrictionDir2        : btVector3;
      procedure                    Init;
      procedure                    Init(const pointA,pointB,normal:btVector3;const distance:btScalar);
      function                     getDistance:btScalar;
      function                     getLifeTime:integer;
      function                     getPositionWorldOnAP:PbtVector3;
      function                     getPositionWorldOnBP:PbtVector3;
      procedure                    setDistance(const dist:btScalar);
      function                     getAppliedImpulse:btScalar; 	///this returns the most recent applied impulse, to satisfy contact constraints by the constraint solver
    end;
    PbtOManifoldPoint = ^btOManifoldPoint;


    btContactDestroyedCallback = function (const userPersistentData:Pointer):boolean;
    btContactProcessedCallback = function (const cp:btOManifoldPoint;const body0,body:Pointer):boolean;
///maximum contact breaking and merging threshold

var gContactBreakingThreshold:btScalar;
    gContactDestroyedCallback:btContactDestroyedCallback;
    gContactProcessedCallback:btContactProcessedCallback;

type
    btContactManifoldTypes=(BT_PERSISTENT_MANIFOLD_TYPE = 1,MAX_CONTACT_MANIFOLD_TYPE);

const btMANIFOLD_CACHE_SIZE=4;

///btPersistentManifold is a contact point cache, it stays persistent as long as objects are overlapping in the broadphase.
///Those contact points are created by the collision narrow phase.
///The cache can be empty, or hold 1,2,3 or 4 points. Some collision algorithms (GJK) might only add one point at a time.
///updates/refreshes old contact points, and throw them away if necessary (distance becomes too large)
///reduces the cache to 4 points, when more then 4 points are added, using following rules:
///the contact point with deepest penetration is always kept, and it tries to maximuze the area covered by the points
///note that some pairs of objects might have more then one contact manifold.
type

    { btTypedObject }

    btTypedObject=class //TODO FOS think about object/class and pooling
       m_objectType:integer;
    public
       constructor Create(const typ:integer);
       function    getObjectType:integer;FOS_INLINE;
    end;

    { btPersistentManifold }

    btPersistentManifold=class(btTypedObject)
    private
       m_pointCache:array [0..btMANIFOLD_CACHE_SIZE] of btOManifoldPoint;
           /// this two body pointers can point to the physics rigidbody class.
           /// void* will allow any rigidbody class
       m_body0 : Pointer;
       m_body1 : Pointer;
       m_cachedPoints : integer;
       m_contactBreakingThreshold   : btScalar;
       m_contactProcessingThreshold : btScalar;
       /// sort cached points so most isolated points come first
       function    sortCachedPoints(const pt:PbtOManifoldPoint):integer;
       //IMPL NOT FOUND function    findContactPoint(const unUsed:PbtOManifoldPoint; const  numUnused:Integer;const pt:btOManifoldPoint):integer;
    public
       m_companionIdA,
       m_companionIdB,
       m_index1a:integer;
       constructor Create          ;overload;
       procedure   Init            (const body0,body1:pointer;const contactBreakingThreshold,contactProcessingThreshold:btScalar);
       function    getBody0        :Pointer;FOS_INLINE;
       function    getBody1        :Pointer;FOS_INLINE;
       procedure   setBodies       (const body0,body1:pointer);
       procedure   clearUserCache  (const pt:PbtOManifoldPoint);
       function    getNumContacts :integer;FOS_INLINE;
       function    getContactPointP(const index:integer):PbtOManifoldPoint;FOS_INLINE;
    ///@todo: get this margin from the current physics / collision environment
       function    getContactBreakingThreshold:btScalar;
       function    getContactProcessingThreshold:btScalar;
       function    getCacheEntry                (const newPoint:PbtOManifoldPoint):integer;
       function    addManifoldPoint             (const newPoint:PbtOManifoldPoint):integer;
       procedure   removeContactPoint           (const index:integer);
       procedure   replaceContactPoint          (const newPoint:PbtOManifoldPoint;const insertIndex:integer);
       function    validContactDistance         (const pt:PbtOManifoldPoint):boolean;
    /// calculated new worldspace coordinates and depth, and reject points that exceed the collision margin
       procedure   refreshContactPoints         (const trA,trB:btTransform);
       procedure   clearManifold                ;
    end;

    btManifoldArray  = specialize FOS_GenericAlignedArray<btPersistentManifold>;


/// This interface is made to be used by an iterative approach to do TimeOfImpact calculations
/// This interface allows to query for closest points and penetration depth between two (convex) objects
/// the closest point is on the second object (B), and the normal points from the surface on B towards A.
/// distance is between closest points on B and closest point on A. So you can calculate closest point on A
/// by taking closestPointInA = closestPointInB + m_distance * m_normalOnSurfaceB

    btDCDI_Result=class  // Was a nested C class, could be used in fpc 2.5.1 trunk too, but lazarus codetools dont support editing well yet
       ///setShapeIdentifiersA/B provides experimental support for per-triangle material / custom material combiner
      procedure setShapeIdentifiersA (const partId0,index0:integer);virtual;abstract;
      procedure setShapeIdentifiersB (const partId1,index1:integer);virtual;abstract;
      procedure addContactPoint      (const normalOnBInWorld,pointInWorld:btVector3;const depth:btScalar);virtual;abstract;
    end;

    btDCDI_ClosestPointInput=object
       m_transformA,
       m_transformB             : btTransform;
       m_maximumDistanceSquared : btScalar;
       m_stackAlloc             : btStackAllocator;
       procedure  Init;
    end;
    PbtDCDI_ClosestPointInput = ^btDCDI_ClosestPointInput;

    //btDiscreteCollisionDetectorInterface=object // FOS Try with object (Performance)
    //public
    //// give either closest points (distance > 0) or penetration (distance)
    //// the normal always points from B towards A
    //   //procedure  getClosestPoints(const input:btDCDI_ClosestPointInput;var output:btDCDI_Result;const debugDraw:btIDebugDraw;const swapResults:boolean=false);virtual;abstract;
    //end;


    { btStorageResult }
    btStorageResult=class(btDCDI_Result)
      m_normalOnSurfaceB : btVector3;
      m_closestPointInB  : btVector3;
      m_distance         : btScalar; //negative means penetration !
    public
      constructor        Create;
      procedure          addContactPoint(const normalOnBInWorld,pointInWorld:btVector3;const depth:btScalar);override;
    end;

    { btUsageBitfield }
    btUsageBitfield = bitpacked record
     case byte of
      1: (
         usedVertexA      : Boolean;
         usedVertexB      : Boolean;
         usedVertexC      : Boolean;
         usedVertexD      : Boolean;
        );
      2: (field           : Byte);
    end;
    PbtUsageBitfield=^btUsageBitfield;
    procedure btUsageBitfield_reset(var buf:btUsageBitfield);FOS_INLINE;
type

    { btSubSimplexClosestResult }

    btSubSimplexClosestResult = object
      m_closestPointOnSimplex : btVector3;
      //MASK for m_usedVertices
      //stores the simplex vertex-usage, using the MASK,
      // if m_usedVertices & MASK then the related vertex is used
      m_usedVertices          : btUsageBitfield ;
      m_barycentricCoords     : array [0..3] of btScalar;
      m_degenerate            : boolean;
      procedure reset;
      function  isvalid       : boolean;
      procedure setBarycentricCoordinates (const a:btScalar=0;const b:btScalar=0;const c:btScalar=0;const d:btScalar=0);
    end;
    PbtSubSimplexClosestResult=^btSubSimplexClosestResult;

    /// btVoronoiSimplexSolver is an implementation of the closest point distance algorithm from a 1-4 points simplex to the origin.
    /// Can be used with GJK, as an alternative to Johnson distance algorithm.

    { btVoronoiSimplexSolver }

    btVoronoiSimplexSolver = class
    public
      m_numVertices          : integer;
      m_simplexVectorW,
      m_simplexPointsP,
      m_simplexPointsQ       : Array [0..VORONOI_SIMPLEX_MAX_VERTS-1] of btVector3;
      m_cachedP1,
      m_cachedP2,
      m_cachedV,
      m_lastW                : btVector3;
      m_equalVertexThreshold : btScalar;
      m_cachedValidClosest   : boolean;
      m_cachedBC             : btSubSimplexClosestResult;
      m_needsUpdate          : Boolean;
      procedure   removeVertex                 (const index:integer);
      procedure   reduceVertices               (const usedVerts:btUsageBitfield);
      function    updateClosestVectorAndPoints : boolean;
      function    closestPtPointTetrahedron    (const p,a,b,c,d:btVector3;var finalResult:btSubSimplexClosestResult):boolean;
      function    pointOutsideOfPlane          (const p,a,b,c,d:btVector3):integer;
      function    closestPtPointTriangle       (const p,a,b,c  :btVector3;var finalResult:btSubSimplexClosestResult):boolean;
      constructor Create                       ;
      procedure   reset                        ;
      procedure   addVertex                    (const w, p, q : btVector3);
      procedure   setEqualVertexThreshold      (const threshold:btScalar);
      function    getEqualVertexThreshold      :btScalar;
      function    closest                      (out v:btVector3):boolean;
      function    maxVertex                    :btScalar;
      function    fullSimplex                  :Boolean;
      function    getSimplex                   (const pBuf, qBuf, yBuf : PbtVector3):integer;
      function    inSimplex                    (const w:btVector3):boolean;
      procedure   backup_closest               (var   v:btVector3);
      function    emptySimplex                 :boolean;
      procedure   compute_points               (out p1,p2: btVector3);
      function    numVertices                  :integer;
    end;

    btSimplexSolverInterface = btVoronoiSimplexSolver;

    ///ConvexPenetrationDepthSolver provides an interface for penetration depth calculation.

    { btConvexPenetrationDepthSolver }

    btConvexPenetrationDepthSolver = class
    public
      function calcPenDepth( const simplexSolver : btSimplexSolverInterface; const convexA,convexB : btConvexShape;
                             const transA,transB : btTransform ; out v,pa,pb : btVector3;
                             const debugDraw:btIDebugDraw ; const stackAlloc : btStackAllocator) :boolean ; virtual;
    end;



//temp globals, to improve GJK/EPA/penetration calculations
    var
        gNumDeepPenetrationChecks : integer = 0;
        gNumGjkChecks             : integer = 0;

    { btGjkPairDetector }
  type
    btGjkPairDetector=object//(btDiscreteCollisionDetectorInterface)
      m_cachedSeparatingAxis     : btVector3;
      m_penetrationDepthSolver   : btConvexPenetrationDepthSolver;
      m_simplexSolver            : btSimplexSolverInterface;
      m_minkowskiA               : btConvexShape;
      m_minkowskiB               : btConvexShape;
      m_shapeTypeA               : TbtBroadphaseNativeTypes;
      m_shapeTypeB               : TbtBroadphaseNativeTypes;
      m_marginA                  : btScalar;
      m_marginB                  : btScalar;
      m_ignoreMargin             : Boolean;
      m_cachedSeparatingDistance : btScalar;
    public
        //some debugging to fix degeneracy problems
      m_lastUsedMethod           : integer;
      m_curIter                  : integer;
      m_degenerateSimplex        : integer;
      m_catchDegeneracies        : boolean;
      procedure   init                       (const  objectA,objectB : btConvexShape;const simplexSolver:btSimplexSolverInterface;const penetrationDepthSolver:btConvexPenetrationDepthSolver);
      procedure   init                       (const  objectA,objectB : btConvexShape;const shapeTypeA,shapeTypeB:TbtBroadphaseNativeTypes;const marginA, marginB:btScalar;const simplexSolver:btSimplexSolverInterface;const penetrationDepthSolver:btConvexPenetrationDepthSolver);
      procedure   getClosestPoints           (const input: btDCDI_ClosestPointInput; var output: btDCDI_Result; const debugDraw: btIDebugDraw);  //btManifoldresult <> btDCDI_Result ? // const swapResults: boolean=false
      procedure   getClosestPointsNonVirtual (const input: btDCDI_ClosestPointInput; const output: btDCDI_Result; const debugDraw: btIDebugDraw);
      procedure   setMinkowskiA              (const minkA:btConvexShape);
      procedure   setMinkowskib              (const minkB:btConvexShape);
      procedure   setCachedSeperatingAxis    (const seperatingAxis:btVector3);
      function    getCachedSeparatingAxis    :btVector3;
      function    getCachedSeparatingDistance:btScalar;
      procedure   setPenetrationDepthSolver  (const penetrationDepthSolver:btConvexPenetrationDepthSolver);
      ///don't use setIgnoreMargin, it's for Bullet's internal use
      procedure   setIgnoreMargin            (const ignoreMargin:Boolean);
    end;

    /// btConvexCast is an interface for Casting
    btConvexCastResult=class;
    btConvexCast = class
    public
    	///RayResult stores the closest result
    	/// alternatively, add a callback method to decide about closest/all results
    	/// cast a convex against another convex object
        function  calcTimeOfImpact( const fromA,toA,fromB,toB:btTransform;var result:btConvexCastResult) :boolean; virtual;abstract;
    end;


    { btPointCollector }

    btPointCollector = class(btDCDI_Result)
      m_normalOnBInWorld : btVector3;
      m_pointInWorld     : btVector3;
      m_distance         : btScalar;//negative means penetration
      m_hasResult        : Boolean;
    public
      constructor create;
      procedure setShapeIdentifiersA(const partId0, index0: integer); override;
      procedure setShapeIdentifiersB(const partId1, index1: integer); override;
      procedure addContactPoint(const normalOnBInWorld, pointInWorld: btVector3; const depth: btScalar); override;
    end;

    { btConvexCastResult }

    btConvexCastResult = class
      m_hitTransformA,m_hitTransformB : btTransform;
      m_normal,m_hitPoint             : btVector3;
      m_fraction                      : btScalar;
      m_debugDrawer                   : btIDebugDraw;
      m_allowedPenetration            : btScalar;
      //virtual bool  addRayResult(const btVector3& normal,btScalar   fraction) = 0;
      procedure   DebugDraw       (const fraction:btScalar);virtual;
      procedure   drawCoordSystem (const trans:btTransform);virtual;
      constructor create          ;
    end;

    { btGjkConvexCast }

    btGjkConvexCast =class(btConvexCast)
      m_simplexSolver  : btSimplexSolverInterface;
      m_convexA        : btConvexShape;
      m_convexB        : btConvexShape;
    public
       constructor create(const convexA,convexB : btConvexShape;const simplexSolver:btSimplexSolverInterface);
       function    calcTimeOfImpact(const fromA, toA, fromB, toB: btTransform; var resultout: btConvexCastResult): boolean; override;
    end;

    /// btSubsimplexConvexCast implements Gino van den Bergens' paper
    ///"Ray Casting against bteral Convex Objects with Application to Continuous Collision Detection"
    /// GJK based Ray Cast, optimized version
    /// Objects should not start in overlap, otherwise results are not defined.

    { btSubsimplexConvexCast }

    btSubsimplexConvexCast=class(btConvexCast)
        m_simplexSolver  : btSimplexSolverInterface;
        m_convexA        : btConvexShape;
        m_convexB        : btConvexShape;
    public
       constructor create(const shapeA,shapeB : btConvexShape; const simplexSolver:btSimplexSolverInterface);
  	///SimsimplexConvexCast calculateTimeOfImpact calculates the time of impact+normal for the linear cast (sweep) between two moving objects.
	///Precondition is that objects should not penetration/overlap at the start from the interval. Overlap can be tested using btGjkPairDetector.
       function    calcTimeOfImpact(const fromA, toA, fromB, toB: btTransform; var resultout: btConvexCastResult): boolean; override;
    end;


    ///btGjkEpaSolver contributed under zlib by Nathanael Presson

    btGjkEpaSolver2_eStatus=(
                ges2_Separated,              // Shapes doesnt penetrate
                ges2_Penetrating,            // Shapes are penetrating
                ges2_GJK_Failed,             // GJK phase fail, no big issue, shapes are probably just 'touching'
                ges2_EPA_Failed              // EPA phase fail, bigger problem, need to save parameters, and debug
    );

    { btGjkEpaSolver2_sResults }

    btGjkEpaSolver2_sResults=object
      witnesses : array[0..1] of btVector3;
      status    : btGjkEpaSolver2_eStatus;
      normal    : btVector3;
      distance  : btScalar;
      procedure Init;
    end;

    { btGjkEpaSolver2 }

    btGjkEpaSolver2 = object
      function  StackSizeRequirement :integer;static;
      function  Distance             (const shape0,shape1:btConvexShape ; const wtrs0,wtrs1:btTransform ; const guess : btVector3 ; var results:btGjkEpaSolver2_sResults):boolean;static;
      function  Penetration          (const shape0,shape1:btConvexShape ; const wtrs0,wtrs1:btTransform ; const guess : btVector3 ; out results:btGjkEpaSolver2_sResults ; const usemargins:boolean=true):boolean;static;
      function  SignedDistance       (const shape0,shape1:btConvexShape ; const wtrs0,wtrs1:btTransform ; const guess : btVector3 ; var results:btGjkEpaSolver2_sResults):boolean;static;
      function  SignedDistance       (const position:btVector3 ;const margin :btScalar ;const shape0 : btConvexShape; const wtrs0: btTransform; var results : btGjkEpaSolver2_sResults):btScalar;static;
    end;



    ///EpaPenetrationDepthSolver uses the Expanding Polytope Algorithm to
    ///calculate the penetration depth between two convex shapes.

    { btGjkEpaPenetrationDepthSolver }

    btGjkEpaPenetrationDepthSolver=class(btConvexPenetrationDepthSolver)
    public
      function calcPenDepth(const simplexSolver: btSimplexSolverInterface; const convexA, convexB: btConvexShape; const transA, transB: btTransform; out v, wWitnessOnA, wWitnessOnB: btVector3; const debugDraw: btIDebugDraw; const stackAlloc: btStackAllocator): boolean; override;
    end;

    { btMinkowskiPenetrationDepthSolver }
    ///MinkowskiPenetrationDepthSolver implements bruteforce penetration depth estimation.
    ///Implementation is based on sampling the depth using support mapping, and using GJK step to get the witness points.

    btMinkowskiPenetrationDepthSolver=class(btConvexPenetrationDepthSolver)
    protected
      function getPenetrationDirections:PbtVector3;
    public
      function calcPenDepth(const simplexSolver: btSimplexSolverInterface; const convexA, convexB: btConvexShape; const transA, transB: btTransform; out v, pa, pb: btVector3; const debugDraw: btIDebugDraw; const stackAlloc: btStackAllocator): boolean; override;
    end;

    EFlags =
    (
       kF_None                 = 0,
       kF_FilterBackfaces      = 1 SHL 0,
       kF_KeepUnflippedNormal  = 1 SHL 1,   // Prevents returned face normal getting flipped when a ray hits a back-facing triangle
       kF_Terminator           = $FFFFFFFF
    );

    { btTriangleRaycastCallback }

    btTriangleRaycastCallback = class(btTriangleCallback)
    public
      m_from,
      m_to                    : btVector3;
      m_flags                 : Cardinal;
      m_hitFraction           : btScalar;
      constructor create(const from,too : btVector3; const flags : cardinal=0);
      procedure   processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer); override;
      function    reportHit(const hitNormalLocal:btVector3;const hitFraction : btScalar; const partId, triangleIndex :integer ):btScalar;virtual;abstract;
    end;

    { btTriangleConvexcastCallback }

    btTriangleConvexcastCallback = class(btTriangleCallback)
    public
      m_convexShape              : btConvexShape;
      m_convexShapeFrom,
      m_convexShapeTo,
      m_triangleToWorld          : btTransform;
      m_hitFraction,
      m_triangleCollisionMargin  : btScalar;
      constructor create(const convexShape:btConvexShape; const convexShapeFrom, convexShapeTo, triangleToWorld : btTransform; const triangleCollisionMargin:btScalar);
      procedure   processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer); override;
      function    reportHit(const hitNormalLocal,hitPointLocal:btVector3;const hitFraction : btScalar; const partId, triangleIndex :integer ):btScalar;virtual;abstract;
    end;


    /// btContinuousConvexCollision implements angular and linear time of impact for convex objects.
    /// Based on Brian Mirtich's Conservative Advancement idea (PhD thesis).
    /// Algorithm operates in worldspace, in order to keep inbetween motion globally consistent.
    /// It uses GJK at the moment. Future improvement would use minkowski sum / supporting vertex, merging innerloops

    { btContinuousConvexCollision }

    btContinuousConvexCollision = class(btConvexCast)
      m_simplexSolver           : btSimplexSolverInterface;
      m_penetrationDepthSolver  : btConvexPenetrationDepthSolver;
      m_convexA,
      m_convexB                 : btConvexShape;
    public
      constructor create(const shapeA,shapeB : btConvexShape; const simplexSolver : btSimplexSolverInterface ;const penetrationDepthSolver : btConvexPenetrationDepthSolver);
      function    calcTimeOfImpact(const fromA, toA, fromB, toB: btTransform; var lresult: btConvexCastResult): boolean; override;
    end;


implementation

procedure btUsageBitfield_reset(var buf: btUsageBitfield);
begin
  buf.field:=0;
end;

{ btConvexPenetrationDepthSolver }

{$HINTS OFF}
function btConvexPenetrationDepthSolver.calcPenDepth(const simplexSolver: btSimplexSolverInterface; const convexA, convexB: btConvexShape; const transA, transB: btTransform; out v, pa, pb: btVector3;
  const debugDraw: btIDebugDraw; const stackAlloc: btStackAllocator): boolean;
begin
  result:=false;
  abort;
end;
{$HINTS ON}

{ btGjkEpaSolver2_sResults }

procedure btGjkEpaSolver2_sResults.Init;
begin
  distance:=0;
end;

{ btOManifoldPoint }

procedure btOManifoldPoint.Init;
begin
  FillByte(self,SizeOf(btOManifoldPoint),0);
end;

procedure btOManifoldPoint.Init(const pointA, pointB, normal: btVector3;  const distance: btScalar);
begin
  Init;
  m_localPointA                := pointA;
  m_localPointB                := pointB;
  m_normalWorldOnB             := normal;
  m_distance1                  := distance;
  m_lifeTime                   := 0;
  m_userPersistentData         := nil;
end;

function btOManifoldPoint.getDistance: btScalar;
begin
  result:=m_distance1;
end;

function btOManifoldPoint.getLifeTime: integer;
begin
  result:=m_lifeTime;
end;

//function btOManifoldPoint.getPositionWorldOnA: btVector3;
//begin
//  Result:=m_positionWorldOnA;
//  // return m_positionWorldOnB + m_normalWorldOnB * m_distance1;
//end;
//
//function btOManifoldPoint.getPositionWorldOnB: btVector3;
//begin
//  result:=m_positionWorldOnB;
//end;

function btOManifoldPoint.getPositionWorldOnAP: PbtVector3;
begin
  Result:=@m_positionWorldOnA;
end;

function btOManifoldPoint.getPositionWorldOnBP: PbtVector3;
begin
  Result:=@m_positionWorldOnB;
end;

procedure btOManifoldPoint.setDistance(const dist: btScalar);
begin
  m_distance1:=dist;
end;

function btOManifoldPoint.getAppliedImpulse: btScalar;
begin
  Result:=m_appliedImpulse;
end;

{ btTypedObject }

constructor btTypedObject.Create(const typ: integer);
begin
  m_objectType:=typ;
end;

function btTypedObject.getObjectType: integer;
begin
  result:=m_objectType;
end;

{ btPersistentManifold }

function btPersistentManifold.sortCachedPoints(const pt: PbtOManifoldPoint ): integer;
var maxPenetrationIndex,
    biggestarea          : integer;
    maxPenetration       : btScalar;
    res0,res1,res2,res3  : btScalar;
    a0,b0,cross,a1,b1,
    a2,b2,a3,b3          : btVector3;
    maxvec               : btVector4;
    i                    : Integer;
begin
  //calculate 4 possible cases areas, and take biggest area
  //also need to keep 'deepest'
  maxPenetrationIndex := -1;
{$ifdef KEEP_DEEPEST_POINT}
  maxPenetration := pt^.getDistance;
  for i:=0 to 3 do begin
    if (m_pointCache[i].getDistance < maxPenetration) then begin
      maxPenetrationIndex := i;
      maxPenetration      := m_pointCache[i].getDistance;
    end;
  end;
{$endif} //KEEP_DEEPEST_POINT

  res0:=0;res1:=0;res2:=0;res3:=0;
  if (maxPenetrationIndex <> 0) then begin
    a0    := pt^.m_localPointA-m_pointCache[1].m_localPointA;
    b0    := m_pointCache[3].m_localPointA-m_pointCache[2].m_localPointA;
    cross := a0.cross(b0);
    res0  := cross.length2;
  end;
  if (maxPenetrationIndex <> 1) then begin
    a1    := pt^.m_localPointA-m_pointCache[0].m_localPointA;
    b1    := m_pointCache[3].m_localPointA-m_pointCache[2].m_localPointA;
    cross := a1.cross(b1);
    res1  := cross.length2;
  end;
  if (maxPenetrationIndex <> 2) then begin
   a2     := pt^.m_localPointA-m_pointCache[0].m_localPointA;
   b2     := m_pointCache[3].m_localPointA-m_pointCache[1].m_localPointA;
   cross  := a2.cross(b2);
   res2   := cross.length2;
  end;
  if (maxPenetrationIndex <> 3) then begin
    a3    := pt^.m_localPointA-m_pointCache[0].m_localPointA;
    b3    := m_pointCache[2].m_localPointA-m_pointCache[1].m_localPointA;
    cross := a3.cross(b3);
    res3  := cross.length2;
  end;
  maxvec.init4(res0,res1,res2,res3);
  biggestarea := maxvec.closestAxis4;
  Result:=biggestarea;
end;


constructor btPersistentManifold.Create;
begin
  Inherited create(ord(BT_PERSISTENT_MANIFOLD_TYPE));
end;


procedure btPersistentManifold.Init(const body0, body1: pointer; const contactBreakingThreshold, contactProcessingThreshold: btScalar);
begin
  m_pointCache[0].Init;
  m_pointCache[1].Init;
  m_pointCache[2].Init;
  m_pointCache[3].Init;
  m_body0                      := body0;
  m_body1                      := body1;
  m_cachedPoints               := 0;
  m_contactBreakingThreshold   := contactBreakingThreshold;
  m_contactProcessingThreshold := contactProcessingThreshold;
end;

function btPersistentManifold.getBody0: Pointer;
begin
  result:=m_body0;
end;

function btPersistentManifold.getBody1: Pointer;
begin
 result:=m_body1;
end;

procedure btPersistentManifold.setBodies(const body0, body1: pointer);
begin
  m_body0:=body0;
  m_body1:=body1;
end;

procedure btPersistentManifold.clearUserCache(const pt: PbtOManifoldPoint);
var oldPtr:Pointer;
begin
  oldPtr := pt^.m_userPersistentData;
  if assigned(oldPtr) then begin
    if (pt^.m_userPersistentData<>nil) and assigned(gContactDestroyedCallback) then begin
        gContactDestroyedCallback(pt^.m_userPersistentData);
        pt^.m_userPersistentData := nil;
    end;
  end;
end;

function btPersistentManifold.getNumContacts: integer;
begin
  result:=m_cachedPoints;
end;

function btPersistentManifold.getContactPointP(const index: integer): PbtOManifoldPoint;
begin
  //.
  btAssert(index < m_cachedPoints);
  Result := @m_pointCache[index];
end;


function btPersistentManifold.getContactBreakingThreshold: btScalar;
begin
 result:=m_contactBreakingThreshold;
end;

function btPersistentManifold.getContactProcessingThreshold: btScalar;
begin
  result:=m_contactProcessingThreshold;
end;

function btPersistentManifold.getCacheEntry(const newPoint: PbtOManifoldPoint): integer;
var shortestDist      : btScalar;
    size,nearestpoint : integer;
    i                 : Integer;
    mp                : btOManifoldPoint;
    diffA             : btVector3;
    distToManiPoint   : btScalar;
begin
  shortestDist :=  getContactBreakingThreshold* getContactBreakingThreshold;
  size         :=  getNumContacts;
  nearestPoint := -1;
  for i := 0 to size-1 do begin
    mp     := m_pointCache[i];
    diffA :=  mp.m_localPointA- newPoint^.m_localPointA;
    distToManiPoint := diffA.dot(diffA);
    if(distToManiPoint < shortestDist ) then begin
      shortestDist := distToManiPoint;
      nearestPoint := i;
    end;
  end;
  result:=nearestPoint;
end;

function btPersistentManifold.addManifoldPoint(const newPoint: PbtOManifoldPoint): integer;
var  insertIndex: LongInt;
begin
  //ASSJUMP
  btAssert(validContactDistance(newPoint));
  insertIndex := getNumContacts;
  if (insertIndex = btMANIFOLD_CACHE_SIZE) then begin
   {$WARNINGS OFF}
    if btMANIFOLD_CACHE_SIZE >= 4 then begin
      //sort cache so best points come first, based on area
      insertIndex := sortCachedPoints(newPoint);
    end else begin
      insertIndex := 0;
    end;
    {$WARNINGS ON}
    clearUserCache(@m_pointCache[insertIndex]);
  end else begin
    inc(m_cachedPoints);
  end;
  if (insertIndex<0) then insertIndex:=0;
 // btAssert(m_pointCache[insertIndex].m_userPersistentData=nil);
  m_pointCache[insertIndex] := newPoint^;
  Result:=insertIndex;
end;

procedure btPersistentManifold.removeContactPoint(const index: integer);
var lastUsedIndex:Integer;
begin
  clearUserCache(@m_pointCache[index]);
  lastUsedIndex := getNumContacts-1;
//          m_pointCache[index] = m_pointCache[lastUsedIndex];
  if(index <> lastUsedIndex) then begin
    m_pointCache[index] := m_pointCache[lastUsedIndex];
    //get rid of duplicated userPersistentData pointer
    m_pointCache[lastUsedIndex].m_userPersistentData         := nil; //TODO : FOS (fillchar? speedup)
    m_pointCache[lastUsedIndex].m_appliedImpulse             := 0;
    m_pointCache[lastUsedIndex].m_lateralFrictionInitialized := false;
    m_pointCache[lastUsedIndex].m_appliedImpulseLateral1     := 0;
    m_pointCache[lastUsedIndex].m_appliedImpulseLateral2     := 0;
    m_pointCache[lastUsedIndex].m_lifeTime                   := 0;
  end;
  btAssert(m_pointCache[lastUsedIndex].m_userPersistentData=nil);
  dec(m_cachedPoints);
end;

procedure btPersistentManifold.replaceContactPoint(const newPoint: PbtOManifoldPoint; const insertIndex: integer);
var       lifeTime          : integer;
    appliedImpulse,
    appliedLateralImpulse1,
    appliedLateralImpulse2  : btScalar;
    cache                   : Pointer;
begin
  //.
  btAssert(validContactDistance(newPoint));
{$ifdef MAINTAIN_PERSISTENCY}
  lifeTime               := m_pointCache[insertIndex].getLifeTime;
  appliedImpulse         := m_pointCache[insertIndex].m_appliedImpulse;
  appliedLateralImpulse1 := m_pointCache[insertIndex].m_appliedImpulseLateral1;
  appliedLateralImpulse2 := m_pointCache[insertIndex].m_appliedImpulseLateral2;

  btAssert(lifeTime>=0);
  cache := m_pointCache[insertIndex].m_userPersistentData;

  m_pointCache[insertIndex] := newPoint^;

  m_pointCache[insertIndex].m_userPersistentData     := cache;
  m_pointCache[insertIndex].m_appliedImpulse         := appliedImpulse;
  m_pointCache[insertIndex].m_appliedImpulseLateral1 := appliedLateralImpulse1;
  m_pointCache[insertIndex].m_appliedImpulseLateral2 := appliedLateralImpulse2;

  m_pointCache[insertIndex].m_lifeTime := lifeTime;
{$else}
  clearUserCache(m_pointCache[insertIndex]);
  m_pointCache[insertIndex] = newPoint;
{$endif}
end;

function btPersistentManifold.validContactDistance(const pt: PbtOManifoldPoint): boolean;
begin
  Result := pt^.m_distance1 <= getContactBreakingThreshold;
end;

procedure btPersistentManifold.refreshContactPoints(const trA, trB: btTransform);
var
  i                   : Integer;
  manifoldPoint       : PbtOManifoldPoint;
  projectedPoint      : btVector3;
  projectedDifference : btVector3;
  distance2d          : btScalar;
begin
 {$IFDEF FOS_DYNAMICS_DEBUG}
  writeln(format('refreshContactPoints posA = (%6.6f,%6.6f,%06.6f) posB = (%6.6f,%6.6f,%6.6f)',[
          trA.getOriginV^.getX(),
          trA.getOriginV^.getY(),
          trA.getOriginV^.getZ(),
          trB.getOriginV^.getX(),
          trB.getOriginV^.getY(),
          trB.getOriginV^.getZ()]
          ));
  {$ENDIF}
  /// first refresh worldspace positions and distance
  for i:=m_cachedPoints-1 downto 0 do begin
    manifoldPoint                     := @m_pointCache[i];
    manifoldPoint^.m_positionWorldOnA := trA.opTrans( manifoldPoint^.m_localPointA );
    manifoldPoint^.m_positionWorldOnB := trB.opTrans( manifoldPoint^.m_localPointB );
    manifoldPoint^.m_distance1        := (manifoldPoint^.m_positionWorldOnA -  manifoldPoint^.m_positionWorldOnB).dot(manifoldPoint^.m_normalWorldOnB);
    inc(manifoldPoint^.m_lifeTime);
  end;
  for i:=getNumContacts-1 downto 0 do begin
        manifoldPoint := @m_pointCache[i];
        //contact becomes invalid when signed distance exceeds margin (projected on contactnormal direction)
        if (not validContactDistance(manifoldPoint)) then begin
          removeContactPoint(i);
        end else begin
          //contact also becomes invalid when relative movement orthogonal to normal exceeds margin
          projectedPoint      := manifoldPoint^.m_positionWorldOnA - manifoldPoint^.m_normalWorldOnB * manifoldPoint^.m_distance1;
          projectedDifference := manifoldPoint^.m_positionWorldOnB - projectedPoint;
          distance2d          := projectedDifference.dot(projectedDifference);
          if (distance2d  > getContactBreakingThreshold*getContactBreakingThreshold) then begin
            removeContactPoint(i);
          end else begin
            //contact point processed callback
            if assigned(gContactProcessedCallback) then gContactProcessedCallback(manifoldPoint^,m_body0,m_body1);
          end;
        end;
  end;
end;

procedure btPersistentManifold.clearManifold;
var  i: Integer;
begin
  for i:=0 to m_cachedPoints-1 do begin
    clearUserCache(@m_pointCache[i]);
  end;
  m_cachedPoints :=0;
  FillByte(m_pointCache,SizeOf(m_pointCache),0);
end;

procedure btDCDI_ClosestPointInput.Init;
begin
  m_maximumDistanceSquared := BT_LARGE_FLOAT;
  m_stackAlloc             := nil;
end;

{ btStorageResult }

constructor btStorageResult.Create;
begin
  m_distance:=BT_LARGE_FLOAT;
end;

procedure btStorageResult.addContactPoint(const normalOnBInWorld,  pointInWorld: btVector3; const depth: btScalar);
begin
  if (depth < m_distance) then begin
    m_normalOnSurfaceB := normalOnBInWorld;
    m_closestPointInB  := pointInWorld;
    m_distance         := depth;
  end;
end;

{ btUsageBitfield }

//procedure btUsageBitfield.reset;
//begin
//  usedVertexA := false;
//  usedVertexB := false;
//  usedVertexC := false;
//  usedVertexD := false;
//end;

{ btSubSimplexClosestResult }

procedure btSubSimplexClosestResult.reset;
begin
  m_degenerate := false;
  setBarycentricCoordinates;
  btUsageBitfield_reset(m_usedVertices);
end;

function btSubSimplexClosestResult.isvalid: boolean;
begin
  Result := (m_barycentricCoords[0] >= btScalar(0)) AND
            (m_barycentricCoords[1] >= btScalar(0)) AND
            (m_barycentricCoords[2] >= btScalar(0)) AND
            (m_barycentricCoords[3] >= btScalar(0));
end;

procedure btSubSimplexClosestResult.setBarycentricCoordinates(const a: btScalar; const b: btScalar; const c: btScalar; const d: btScalar);
begin
  m_barycentricCoords[0] := a;
  m_barycentricCoords[1] := b;
  m_barycentricCoords[2] := c;
  m_barycentricCoords[3] := d;
end;

{ btVoronoiSimplexSolver }

procedure btVoronoiSimplexSolver.removeVertex(const index: integer);
begin
  //LAB
  btAssert(m_numVertices>0);
  dec(m_numVertices);
  m_simplexVectorW[index] := m_simplexVectorW[m_numVertices];
  m_simplexPointsP[index] := m_simplexPointsP[m_numVertices];
  m_simplexPointsQ[index] := m_simplexPointsQ[m_numVertices];
end;

procedure btVoronoiSimplexSolver.reduceVertices(const usedVerts: btUsageBitfield);
begin
  if (numVertices >= 4) and  (not usedVerts.usedVertexD) then begin
    removeVertex(3);
  end;
  if (numVertices >= 3) and  (not usedVerts.usedVertexC) then begin
    removeVertex(2);
  end;
  if (numVertices >= 2) and  (not usedVerts.usedVertexB) then begin
    removeVertex(1);
  end;
  if (numVertices >= 1) and (not usedVerts.usedVertexA) then begin
    removeVertex(0);
  end;
end;

function btVoronoiSimplexSolver.updateClosestVectorAndPoints: boolean;
var //nearest,
    p,diff,v : btVector3;
    t,dotVV          : btScalar;
begin
  if m_needsUpdate then begin
    m_cachedBC.reset();
    m_needsUpdate := false;
    case numVertices of
      0:  m_cachedValidClosest := false;
      1:  begin
            m_cachedP1 := m_simplexPointsP[0];
            m_cachedP2 := m_simplexPointsQ[0];
            m_cachedV  := m_cachedP1-m_cachedP2; //== m_simplexVectorW[0]
            m_cachedBC.reset;
            m_cachedBC.setBarycentricCoordinates(1,0,0,0);
            m_cachedValidClosest := m_cachedBC.isValid;
          end;
      2:  begin
            //closest point origin from line segment
            p.InitSame(0);
            diff := p                   - m_simplexVectorW[0];
            v    := m_simplexVectorW[1] - m_simplexVectorW[0];
            t    := v.dot(diff);
            if t > 0 then begin
              dotVV := v.dot(v);
              if t < dotVV then begin
                t    :=  t / dotVV;
                diff :=  diff -t*v;
                m_cachedBC.m_usedVertices.usedVertexA := true;
                m_cachedBC.m_usedVertices.usedVertexB := true;
              end else begin
                t := 1;
                diff := diff - v;
                //reduce to 1 point
                m_cachedBC.m_usedVertices.usedVertexB := true;
              end;
            end else begin
              t := 0;
              //reduce to 1 point
              m_cachedBC.m_usedVertices.usedVertexA := true;
            end;
            m_cachedBC.setBarycentricCoordinates(1-t,t);
//            nearest    := m_simplexVectorW[0] + t*v;
            m_cachedP1 := m_simplexPointsP[0] + t * (m_simplexPointsP[1] - m_simplexPointsP[0]);
            m_cachedP2 := m_simplexPointsQ[0] + t * (m_simplexPointsQ[1] - m_simplexPointsQ[0]);
            m_cachedV  := m_cachedP1 - m_cachedP2;
            reduceVertices(m_cachedBC.m_usedVertices);
            m_cachedValidClosest := m_cachedBC.isValid;
          end;
      3:  begin
            //closest point origin from triangle
            p.InitSame(0);
            closestPtPointTriangle(p,m_simplexVectorW[0],m_simplexVectorW[1],m_simplexVectorW[2],m_cachedBC);
            m_cachedP1 := m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords[0] +
                          m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords[1] +
                          m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords[2];
            m_cachedP2 := m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords[0] +
                          m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords[1] +
                          m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords[2];
            m_cachedV  := m_cachedP1-m_cachedP2;
            reduceVertices(m_cachedBC.m_usedVertices);
            m_cachedValidClosest := m_cachedBC.isValid;
          end;
      4:  begin
            p.initsame(0);
            if closestPtPointTetrahedron(p,m_simplexVectorW[0],m_simplexVectorW[1],m_simplexVectorW[2],m_simplexVectorW[3],m_cachedBC) then begin
              m_cachedP1 := m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords[0] +
                            m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords[1] +
                            m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords[2] +
                            m_simplexPointsP[3] * m_cachedBC.m_barycentricCoords[3];
              m_cachedP2 := m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords[0] +
                            m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords[1] +
                            m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords[2] +
                            m_simplexPointsQ[3] * m_cachedBC.m_barycentricCoords[3];
              m_cachedV  := m_cachedP1-m_cachedP2;
              reduceVertices(m_cachedBC.m_usedVertices);
            end else begin
  //                                      printf("sub distance got penetration\n");
              if m_cachedBC.m_degenerate then begin
                m_cachedValidClosest := false;
              end else begin
                m_cachedValidClosest := true;
                //degenerate case == false, penetration = true + zero
                m_cachedV.InitSame(0);
              end;
            end;
            m_cachedValidClosest := m_cachedBC.isValid();
            //closest point origin from tetrahedron
          end;
      else begin
        m_cachedValidClosest := false;
      end;
    end;
  end;
  result := m_cachedValidClosest;
end;

function btVoronoiSimplexSolver.closestPtPointTetrahedron(const p, a, b, c, d: btVector3; var finalResult: btSubSimplexClosestResult): boolean;
var tempResult:btSubSimplexClosestResult;
    pointOutsideABC,pointOutsideACD,pointOutsideADB,pointOutsideBDC:integer;
    bestSqDist,sqDist : btScalar;
    q          : btVector3;
begin
   tempResult.reset;

  // Start out assuming point inside all halfspaces, so closest to itself
    finalResult.m_closestPointOnSimplex := p;
    btUsageBitfield_reset(finalResult.m_usedVertices);
    finalResult.m_usedVertices.usedVertexA := true;
    finalResult.m_usedVertices.usedVertexB := true;
    finalResult.m_usedVertices.usedVertexC := true;
    finalResult.m_usedVertices.usedVertexD := true;

    pointOutsideABC := pointOutsideOfPlane(p, a, b, c, d);
    pointOutsideACD := pointOutsideOfPlane(p, a, c, d, b);
    pointOutsideADB := pointOutsideOfPlane(p, a, d, b, c);
    pointOutsideBDC := pointOutsideOfPlane(p, b, d, c, a);

  if (pointOutsideABC < 0) or (pointOutsideACD < 0) or (pointOutsideADB < 0) or (pointOutsideBDC < 0) then begin
   finalResult.m_degenerate := true;
   exit(false);
  end;

  if (pointOutsideABC=0)  and (pointOutsideACD=0) and (pointOutsideADB=0) and (pointOutsideBDC=0) then begin
   exit(false);
  end;

  bestSqDist := SIMD_INFINITY;
  // If point outside face abc then compute closest point on abc
  if pointOutsideABC<>0 then begin
    closestPtPointTriangle(p, a, b, c,tempResult);
    q      := tempResult.m_closestPointOnSimplex;
    sqDist := (q - p).dot( q - p);
    // Update best closest point if (squared) distance is less than current best
    if sqDist < bestSqDist then begin
      bestSqDist := sqDist;
      finalResult.m_closestPointOnSimplex := q;
      //convert result bitmask!
      btUsageBitfield_reset(finalResult.m_usedVertices);
      finalResult.m_usedVertices.usedVertexA := tempResult.m_usedVertices.usedVertexA;
      finalResult.m_usedVertices.usedVertexB := tempResult.m_usedVertices.usedVertexB;
      finalResult.m_usedVertices.usedVertexC := tempResult.m_usedVertices.usedVertexC;
      finalResult.setBarycentricCoordinates(tempResult.m_barycentricCoords[VERTA],tempResult.m_barycentricCoords[VERTB],tempResult.m_barycentricCoords[VERTC],0);
    end;
  end;
  // Repeat test for face acd
  if pointOutsideACD<>0 then begin
    closestPtPointTriangle(p, a, c, d,tempResult);
    q := tempResult.m_closestPointOnSimplex;
    //convert result bitmask!
    sqDist := (q - p).dot( q - p);
    if sqDist < bestSqDist then begin
      bestSqDist := sqDist;
      finalResult.m_closestPointOnSimplex := q;
      btUsageBitfield_reset(finalResult.m_usedVertices);
      finalResult.m_usedVertices.usedVertexA := tempResult.m_usedVertices.usedVertexA;
      finalResult.m_usedVertices.usedVertexC := tempResult.m_usedVertices.usedVertexB;
      finalResult.m_usedVertices.usedVertexD := tempResult.m_usedVertices.usedVertexC;
      finalResult.setBarycentricCoordinates(tempResult.m_barycentricCoords[VERTA],0,tempResult.m_barycentricCoords[VERTB],tempResult.m_barycentricCoords[VERTC]);
    end;
  end;
  // Repeat test for face adb
  if pointOutsideADB<>0 then begin
    closestPtPointTriangle(p, a, d, b,tempResult);
    q := tempResult.m_closestPointOnSimplex;
    //convert result bitmask!
    sqDist := (q - p).dot( q - p);
    if sqDist < bestSqDist then begin
      bestSqDist := sqDist;
      finalResult.m_closestPointOnSimplex := q;
      btUsageBitfield_reset(finalResult.m_usedVertices);
      finalResult.m_usedVertices.usedVertexA := tempResult.m_usedVertices.usedVertexA;
      finalResult.m_usedVertices.usedVertexB := tempResult.m_usedVertices.usedVertexC;
      finalResult.m_usedVertices.usedVertexD := tempResult.m_usedVertices.usedVertexB;
      finalResult.setBarycentricCoordinates(tempResult.m_barycentricCoords[VERTA],tempResult.m_barycentricCoords[VERTC],0,tempResult.m_barycentricCoords[VERTB]);
    end;
  end;
  // Repeat test for face bdc
  if pointOutsideBDC<>0 then begin
    closestPtPointTriangle(p, b, d, c,tempResult);
    q := tempResult.m_closestPointOnSimplex;
    //convert result bitmask!
    sqDist := (q - p).dot( q - p);
    if sqDist < bestSqDist then begin
      bestSqDist := sqDist;
      finalResult.m_closestPointOnSimplex := q;
      btUsageBitfield_reset(finalResult.m_usedVertices);
      finalResult.m_usedVertices.usedVertexB := tempResult.m_usedVertices.usedVertexA;
      finalResult.m_usedVertices.usedVertexC := tempResult.m_usedVertices.usedVertexC;
      finalResult.m_usedVertices.usedVertexD := tempResult.m_usedVertices.usedVertexB;
      finalResult.setBarycentricCoordinates(0,tempResult.m_barycentricCoords[VERTA],tempResult.m_barycentricCoords[VERTC],tempResult.m_barycentricCoords[VERTB]);
    end;
  end;
  //help! we ended up full !
  //FOSHH : the following 3 lines seem to be worthless ?
  //if (finalResult.m_usedVertices.usedVertexA and finalResult.m_usedVertices.usedVertexB and finalResult.m_usedVertices.usedVertexC and finalResult.m_usedVertices.usedVertexD) then begin
  // exit(true);
  //end;
  Result := true;
end;

function btVoronoiSimplexSolver.pointOutsideOfPlane(const p, a, b, c, d: btVector3): integer;
var normal : btVector3;
    signp,
    signd  : btScalar;
begin
  normal := (b-a).cross(c-a);
  signp  := (p - a).dot(normal); // [AP AB AC]
  signd  := (d - a).dot(normal); // [AD AB AC]
  {$ifdef CATCH_DEGENERATE_TETRAHEDRON}
  {$ifdef BT_USE_DOUBLE_PRECISION}
  if (signd*signd) < (btScalar(1e-8)*btScalar(1e-8)) then begin
    exit(-1);
  end;
  {$else}
  if (signd*signd) < (btScalar(1e-4)*btScalar(1e-4)) then begin
  //            printf("affine dependent/degenerate\n");//
    exit(-1);
  end;
  {$endif}
  {$endif}
   // Points on opposite sides if expression signs are opposite
  if (signp*signd)<0 then begin
    result := 1;
  end else begin
    result := 0;
  end;
end;

function btVoronoiSimplexSolver.closestPtPointTriangle(const p, a, b, c : btVector3; var finalResult: btSubSimplexClosestResult): boolean;
var    ab,ac,ap,bp,cp:btVector3;
       d1,d2,d3,d4,d5,d6,vc,v,vb,va,w,denom:btScalar;
begin
     btUsageBitfield_reset(finalresult.m_usedVertices);
      // Check if P in vertex region outside A
      ab := b - a;
      ac := c - a;
      ap := p - a;
      d1 := ab.dot(ap);
      d2 := ac.dot(ap);
      if (d1 <= 0) and  (d2 <= 0) then begin
        finalresult.m_closestPointOnSimplex    := a;
        finalresult.m_usedVertices.usedVertexA := true;
        finalresult.setBarycentricCoordinates(1,0,0);
        exit(true);// a; // barycentric coordinates (1,0,0)
      end;
      // Check if P in vertex region outside B
      bp := p - b;
      d3 := ab.dot(bp);
      d4 := ac.dot(bp);
      if (d3 >= 0) and (d4 <= d3) then begin
        finalresult.m_closestPointOnSimplex    := b;
        finalresult.m_usedVertices.usedVertexB := true;
        finalresult.setBarycentricCoordinates(0,1,0);
        exit(true); // b; // barycentric coordinates (0,1,0)
      end;
      // Check if P in edge region of AB, if so return projection of P onto AB
      vc := d1*d4 - d3*d2;
      if (vc <= 0) and (d1 >= 0) and (d3 <= 0) then begin
        v := d1 / (d1 - d3);
        finalresult.m_closestPointOnSimplex    := a + v * ab;
        finalresult.m_usedVertices.usedVertexA := true;
        finalresult.m_usedVertices.usedVertexB := true;
        finalresult.setBarycentricCoordinates(1-v,v,0);
        exit(true);
        //return a + v * ab; // barycentric coordinates (1-v,v,0)
      end;

      // Check if P in vertex region outside C
      cp := p - c;
      d5 := ab.dot(cp);
      d6 := ac.dot(cp);
      if (d6 >= 0) and (d5 <= d6) then begin
        finalresult.m_closestPointOnSimplex    := c;
        finalresult.m_usedVertices.usedVertexC := true;
        finalresult.setBarycentricCoordinates(0,0,1);
        exit(true);//c; // barycentric coordinates (0,0,1)
      end;
      // Check if P in edge region of AC, if so return projection of P onto AC
      vb := d5*d2 - d1*d6;
      if (vb <= 0) and (d2 >= 0) and (d6 <= 0) then begin
        w := d2 / (d2 - d6);
        finalresult.m_closestPointOnSimplex    := a + w * ac;
        finalresult.m_usedVertices.usedVertexA := true;
        finalresult.m_usedVertices.usedVertexC := true;
        finalresult.setBarycentricCoordinates(1-w,0,w);
        exit(true);
        //return a + w * ac; // barycentric coordinates (1-w,0,w)
      end;
      // Check if P in edge region of BC, if so return projection of P onto BC
      va := d3*d6 - d5*d4;
      if (va <= 0) and ((d4-d3) >= 0) and ((d5-d6) >= 0)  then begin
        w := (d4 - d3) / ((d4 - d3) + (d5 - d6));
        finalresult.m_closestPointOnSimplex    := b + w * (c - b);
        finalresult.m_usedVertices.usedVertexB := true;
        finalresult.m_usedVertices.usedVertexC := true;
        finalresult.setBarycentricCoordinates(0,1-w,w);
        exit(true)
        // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
      end;
      // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
      denom := 1 / (va + vb + vc);
      v     := vb * denom;
      w     := vc * denom;
      finalresult.m_closestPointOnSimplex    := a + ab * v + ac * w;
      finalresult.m_usedVertices.usedVertexA := true;
      finalresult.m_usedVertices.usedVertexB := true;
      finalresult.m_usedVertices.usedVertexC := true;
      finalresult.setBarycentricCoordinates(1-v-w,v,w);
      Result:=true;
  //    return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = btScalar(1.0) - v - w
end;

constructor btVoronoiSimplexSolver.Create;
begin
  m_equalVertexThreshold := VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD;
end;

procedure btVoronoiSimplexSolver.reset;
begin
  m_cachedValidClosest := false;
  m_numVertices        := 0;
  m_needsUpdate        := true;
  m_lastW.InitSame(BT_LARGE_FLOAT);
  m_cachedBC.reset;
end;

procedure btVoronoiSimplexSolver.addVertex(const w, p, q: btVector3);
begin
  m_lastW       := w;
  m_needsUpdate := true;
  m_simplexVectorW[m_numVertices] := w;
  m_simplexPointsP[m_numVertices] := p;
  m_simplexPointsQ[m_numVertices] := q;
  inc(m_numVertices);
end;

procedure btVoronoiSimplexSolver.setEqualVertexThreshold(const threshold: btScalar);
begin
   m_equalVertexThreshold := threshold;
end;

function btVoronoiSimplexSolver.getEqualVertexThreshold : btScalar;
begin
  result := m_equalVertexThreshold;
end;

function btVoronoiSimplexSolver.closest(out v: btVector3): boolean;
begin
  result := updateClosestVectorAndPoints;
  v      := m_cachedV;
end;

function btVoronoiSimplexSolver.maxVertex: btScalar;
var i,numverts:integer;
    maxV,curLen2:btScalar;
begin
  numverts := numVertices;
  maxV     := btScalar(0);
  for i:=0 to  numverts-1 do begin
    curLen2 := m_simplexVectorW[i].length2;
    if maxV < curLen2 then begin
      maxV := curLen2;
    end;
  end;
  result := maxV;
end;

function btVoronoiSimplexSolver.fullSimplex: Boolean;
begin
  Result := m_numVertices = 4;
end;

function btVoronoiSimplexSolver.getSimplex(const pBuf, qBuf, yBuf: PbtVector3): integer;
var  i: Integer;
begin
  for i:=0 to numVertices-1 do begin
    yBuf[i] := m_simplexVectorW[i];
    pBuf[i] := m_simplexPointsP[i];
    qBuf[i] := m_simplexPointsQ[i];
  end;
  Result := numVertices;
end;

function btVoronoiSimplexSolver.inSimplex(const w: btVector3): boolean;
var found:Boolean;
    i,numverts:integer;
//    maxV:btScalar;
begin
  found    := false;
  numverts := numVertices;
//  maxV     := btScalar(0);
  //w is in the current (reduced) simplex
  for i:=0 to numverts-1 do begin
{$ifdef BT_USE_EQUAL_VERTEX_THRESHOLD}
    if  m_simplexVectorW[i].distance2(w) <= m_equalVertexThreshold then begin
{$else}
    if m_simplexVectorW[i] = w then begin
{$endif}
      found := true;
    end;
  end;
  //check in case lastW is already removed
  if w = m_lastW then begin
    exit(true);
  end;
  Result := found;
end;

procedure btVoronoiSimplexSolver.backup_closest(var v: btVector3);
begin
  v := m_cachedV;
end;

function btVoronoiSimplexSolver.emptySimplex: boolean;
begin
  Result := numVertices = 0;
end;

procedure btVoronoiSimplexSolver.compute_points(out p1, p2: btVector3);
begin
  updateClosestVectorAndPoints;
  p1 := m_cachedP1;
  p2 := m_cachedP2;
end;

function btVoronoiSimplexSolver.numVertices: integer;
begin
  Result := m_numVertices;
end;

{ btGjkPairDetector }

procedure btGjkPairDetector.init(const objectA, objectB: btConvexShape; const simplexSolver: btSimplexSolverInterface; const penetrationDepthSolver: btConvexPenetrationDepthSolver);
begin
  m_cachedSeparatingAxis.Init(0,1,0);
  m_penetrationDepthSolver := penetrationDepthSolver;
  m_simplexSolver          := simplexSolver;
  m_minkowskiA             := objectA;
  m_minkowskiB             := objectB;
  m_shapeTypeA             := objectA.getShapeType;
  m_shapeTypeB             := objectB.getShapeType;
  m_marginA                := objectA.getMargin;
  m_marginB                := objectB.getMargin;
  m_ignoreMargin           := false;
  m_lastUsedMethod         := -1;
  m_catchDegeneracies      := true;
end;

procedure btGjkPairDetector.init(const objectA, objectB: btConvexShape; const shapeTypeA, shapeTypeB: TbtBroadphaseNativeTypes; const marginA, marginB: btScalar; const simplexSolver: btSimplexSolverInterface;  const penetrationDepthSolver: btConvexPenetrationDepthSolver);
begin
  m_cachedSeparatingAxis.Init(0,1,0);
  m_penetrationDepthSolver := penetrationDepthSolver;
  m_simplexSolver          := simplexSolver;
  m_minkowskiA             := objectA;
  m_minkowskiB             := objectB;
  m_shapeTypeA             := shapeTypeA;
  m_shapeTypeB             := shapeTypeB;
  m_marginA                := marginA;
  m_marginB                := marginB;
  m_ignoreMargin           := false;
  m_lastUsedMethod         := -1;
  m_catchDegeneracies      := true;
end;

procedure btGjkPairDetector.getClosestPoints(const input: btDCDI_ClosestPointInput; var output: btDCDI_Result; const debugDraw: btIDebugDraw); //; const swapResults: boolean);
begin
  getClosestPointsNonVirtual(input,output,debugDraw);
end;

procedure btGjkPairDetector.getClosestPointsNonVirtual(const input: btDCDI_ClosestPointInput; const output: btDCDI_Result; const debugDraw: btIDebugDraw);
var  distance         : btScalar;
     normalInB,
     pointOnA,
     pointOnB,
     positionOffset   : btVector3;
     localTransA,
     localTransB      : btTransform;
     check2d          : boolean;
     isValid,
     checkSimplex,
     checkPenetration : boolean;
     marginA,marginB,
     lenSqr,rlen,s    : btScalar;
     squaredDistance,
     delta,
     margin,f0,f1     : btScalar;
     gGjkMaxIter      : integer;
     w,seperatingAxisInA,
     seperatingAxisInB,
     pInA,qInB,pWorld,qWorld,
     newCachedSeparatingAxis  : btVector3;
     previousSquaredDistance  : Single;
     check,isValid2           : Boolean;
     catchDegeneratePenetrationCase : Boolean;
     tmpPointOnA,tmpPointOnB,
     tmpNormalInB              : btVector3;
     distance2                 : btScalar;

begin
  m_cachedSeparatingDistance := 0;
  distance                   := 0;
  normalInB.InitSame(0);
  localTransA    := input.m_transformA;
  localTransB    := input.m_transformB;
  positionOffset := localTransA.getOriginV^ + localTransB.getOriginV^ * 0.5;
  localTransA.getOriginV^ -= positionOffset;
  localTransB.getOriginV^ -= positionOffset;
  check2d := m_minkowskiA.isConvex2d and m_minkowskiB.isConvex2d;
  marginA := m_marginA;
  marginB := m_marginB;
  inc(gNumGjkChecks);
  //for CCD we don't use margins
  if m_ignoreMargin then begin
    marginA := btScalar(0);
    marginB := btScalar(0);
  end;

  m_curIter           := 0;
  gGjkMaxIter         := 1000;//this is to catch invalid input, perhaps check for #NaN?
  m_cachedSeparatingAxis.init(0,1,0);
  isValid             := false;
  checkSimplex        := false;
  checkPenetration    := true;
  m_degenerateSimplex := 0;
  m_lastUsedMethod    := -1;
  squaredDistance     := BT_LARGE_FLOAT;
  delta               := btScalar(0);
  margin              := marginA + marginB;
  m_simplexSolver.reset;

  while(true) do begin
    seperatingAxisInA := (-m_cachedSeparatingAxis)* input.m_transformA.GetBasisV^;
    seperatingAxisInB := m_cachedSeparatingAxis* input.m_transformB.GetBasisV^;
{$if 1}
    pInA := m_minkowskiA.localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
    qInB := m_minkowskiB.localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);
{$else}
    pInA := m_minkowskiA.localGetSupportingVertexWithoutMargin(seperatingAxisInA);
    qInB := m_minkowskiB.localGetSupportingVertexWithoutMargin(seperatingAxisInB);
{$ifdef TEST_NON_VIRTUAL}
    btVector3 pInAv = m_minkowskiA->localGetSupportingVertexWithoutMargin(seperatingAxisInA);
    btVector3 qInBv = m_minkowskiB->localGetSupportingVertexWithoutMargin(seperatingAxisInB);
    btAssert((pInAv-pInA).length() < 0.0001);
    btAssert((qInBv-qInB).length() < 0.0001);
{$endif} //
{$endif}
    pWorld := localTransA.opTrans(pInA);
    qWorld := localTransB.opTrans(qInB);
    if check2d then begin
      pWorld[2] := 0;
      qWorld[2] := 0;
    end;
    w     := pWorld - qWorld;
    delta := m_cachedSeparatingAxis.dot(w);
    // potential exit, they don't overlap
    if (delta > 0) and ((delta*delta) > (squaredDistance*input.m_maximumDistanceSquared)) then begin
      m_degenerateSimplex := 10;
      checkSimplex        :=true;
      //checkPenetration = false;
      break;
    end;
    //exit 0: the new point is already in the simplex, or we didn't come any closer
    if m_simplexSolver.inSimplex(w) then begin
      m_degenerateSimplex := 1;
      checkSimplex        := true;
      break;
    end;
    // are we getting any closer ?
    f0 := squaredDistance - delta;
    f1 := squaredDistance * REL_ERROR2;
    if (f0 <= f1) then begin
      if f0 <= 0 then begin
        m_degenerateSimplex := 2;
      end else begin
        m_degenerateSimplex := 11;
      end;
      checkSimplex := true;
      break;
    end;
    //add current vertex to simplex
    m_simplexSolver.addVertex(w, pWorld, qWorld);
    //calculate the closest point to the origin (update vector v)
    if not m_simplexSolver.closest(newCachedSeparatingAxis) then begin
      m_degenerateSimplex := 3;
      checkSimplex        := true;
      break;
    end;
    if newCachedSeparatingAxis.length2<REL_ERROR2 then begin
      m_cachedSeparatingAxis := newCachedSeparatingAxis;
      m_degenerateSimplex    := 6;
      checkSimplex           := true;
      break;
    end;
    previousSquaredDistance := squaredDistance;
    squaredDistance         := newCachedSeparatingAxis.length2();
{$if 0}
///warning: this termination condition leads to some problems in 2d test case see Bullet/Demos/Box2dDemo
    if squaredDistance>previousSquaredDistance then begin
      m_degenerateSimplex := 7;
      squaredDistance     := previousSquaredDistance;
      checkSimplex        := false;
      break;
    end;
{$endif}
    m_cachedSeparatingAxis := newCachedSeparatingAxis;
    //redundant m_simplexSolver->compute_points(pointOnA, pointOnB);
    //are we getting any closer ?
    if (previousSquaredDistance - squaredDistance) <= (SIMD_EPSILON * previousSquaredDistance) then begin
      m_simplexSolver.backup_closest(m_cachedSeparatingAxis);
      checkSimplex        := true;
      m_degenerateSimplex := 12;
      break;
    end;
    //degeneracy, this is typically due to invalid/uninitialized worldtransforms for a btCollisionObject
    if (m_curIter > gGjkMaxIter) then begin
      inc(m_curIter);
      {$ifdef DEBUG}
        writeln(format('btGjkPairDetector maxIter exceeded:%i\n',[m_curIter]));
        writeln(format('sepAxis=(%f,%f,%f), squaredDistance = %f, shapeTypeA=%i,shapeTypeB=%i\n',[
          m_cachedSeparatingAxis.getX,
          m_cachedSeparatingAxis.getY,
          m_cachedSeparatingAxis.getZ,
          squaredDistance, m_minkowskiA.getShapeType,m_minkowskiB.getShapeType]));
      {$endif}
       break;
    end;
    check := (not m_simplexSolver.fullSimplex);
    //bool check = (!m_simplexSolver->fullSimplex() && squaredDistance > SIMD_EPSILON * m_simplexSolver->maxVertex());
    if (not check) then begin
      //do we need this backup_closest here ?
      m_simplexSolver.backup_closest(m_cachedSeparatingAxis);
      m_degenerateSimplex := 13;
      break;
    end;
  end;

  if checkSimplex then begin
    m_simplexSolver.compute_points(pointOnA, pointOnB);
    normalInB := pointOnA-pointOnB;
    lenSqr    := m_cachedSeparatingAxis.length2;
    //valid normal
    if (lenSqr < 0.0001) then begin
      m_degenerateSimplex := 5;
    end;
    if (lenSqr > SIMD_EPSILON*SIMD_EPSILON) then begin
      rlen      := 1 / btSqrt(lenSqr);
      normalInB *= rlen; //normalize
      s         := btSqrt(squaredDistance);
      btAssert(s > btScalar(0.0));
      pointOnA  -= m_cachedSeparatingAxis * (marginA / s);
      pointOnB  += m_cachedSeparatingAxis * (marginB / s);
      distance  := ((1/rlen) - margin);
      isValid   := true;
      m_lastUsedMethod := 1;
    end else begin
      m_lastUsedMethod := 2;
    end;
  end;
  catchDegeneratePenetrationCase := (m_catchDegeneracies and assigned(m_penetrationDepthSolver) and (m_degenerateSimplex<>0) and ((distance+margin) < 0.01));
  //if (checkPenetration && !isValid)
  if checkPenetration and (not isValid or catchDegeneratePenetrationCase) then begin
    //penetration case
    //if there is no way to handle penetrations, bail out
    if assigned(m_penetrationDepthSolver) then begin
      // Penetration depth case.
      inc(gNumDeepPenetrationChecks);
      m_cachedSeparatingAxis.zero;
      isValid2 := m_penetrationDepthSolver.calcPenDepth(
              m_simplexSolver,m_minkowskiA,m_minkowskiB,localTransA,localTransB,
              m_cachedSeparatingAxis, tmpPointOnA, tmpPointOnB,
              debugDraw,input.m_stackAlloc
              );
      if isValid2 then begin
        tmpNormalInB    := tmpPointOnB-tmpPointOnA;
        lenSqr          := tmpNormalInB.length2;
        if lenSqr <= (SIMD_EPSILON*SIMD_EPSILON) then begin
          tmpNormalInB  := m_cachedSeparatingAxis;
          lenSqr        := m_cachedSeparatingAxis.length2;
        end;
        if lenSqr > (SIMD_EPSILON*SIMD_EPSILON) then begin
          tmpNormalInB /= btSqrt(lenSqr);
          distance2    := -(tmpPointOnA-tmpPointOnB).length;
          //only replace valid penetrations when the result is deeper (check)
          if (not isValid) or (distance2 < distance) then begin
            distance  := distance2;
            pointOnA  := tmpPointOnA;
            pointOnB  := tmpPointOnB;
            normalInB := tmpNormalInB;
            isValid   := true;
            m_lastUsedMethod := 3;
          end else begin
            m_lastUsedMethod := 8;
          end;
        end else begin
          m_lastUsedMethod := 9;
        end;
      end else begin
        ///this is another degenerate case, where the initial GJK calculation reports a degenerate case
        ///EPA reports no penetration, and the second GJK (using the supporting vector without margin)
        ///reports a valid positive distance. Use the results of the second GJK instead of failing.
        ///thanks to Jacob.Langford for the reproduction case
        ///http://code.google.com/p/bullet/issues/detail?id=250
        if m_cachedSeparatingAxis.length2 > 0 then begin
          distance2 := (tmpPointOnA-tmpPointOnB).length-margin;
          //only replace valid distances when the distance is less
          if (not isValid) or (distance2 < distance) then begin
            distance  := distance2;
            pointOnA  := tmpPointOnA;
            pointOnB  := tmpPointOnB;
            pointOnA  -= m_cachedSeparatingAxis * marginA ;
            pointOnB  += m_cachedSeparatingAxis * marginB ;
            normalInB := m_cachedSeparatingAxis;
            normalInB.normalize;
            isValid := true;
            m_lastUsedMethod := 6;
          end else begin
            m_lastUsedMethod := 5;
          end;
        end;
      end;
    end;
  end;
  if isValid and ( (distance < 0) or (distance*distance < input.m_maximumDistanceSquared)) then begin
  {$if 0}
  ///some debugging
    if check2d then begin
      writeln(format('n = %2.3f,%2.3f,%2.3f. ',[normalInB[0],normalInB[1],normalInB[2]]));
      writeln(format('distance = %2.3f exit=%d deg=%d\n',[distance,m_lastUsedMethod,m_degenerateSimplex]));
    end;
  {$endif}
    m_cachedSeparatingAxis     := normalInB;
    m_cachedSeparatingDistance := distance;
    output.addContactPoint(normalInB,pointOnB+positionOffset,distance);
  end;
end;

procedure btGjkPairDetector.setMinkowskiA(const minkA: btConvexShape);
begin
  m_minkowskiA := minkA;
end;

procedure btGjkPairDetector.setMinkowskib(const minkB: btConvexShape);
begin
  m_minkowskiB := minkB;
end;

procedure btGjkPairDetector.setCachedSeperatingAxis(const seperatingAxis: btVector3);
begin
  m_cachedSeparatingAxis := seperatingAxis;
end;

function btGjkPairDetector.getCachedSeparatingAxis: btVector3;
begin
  result := m_cachedSeparatingAxis;
end;

function btGjkPairDetector.getCachedSeparatingDistance: btScalar;
begin
  result := m_cachedSeparatingDistance;
end;

procedure btGjkPairDetector.setPenetrationDepthSolver(const penetrationDepthSolver: btConvexPenetrationDepthSolver);
begin
  m_penetrationDepthSolver := penetrationDepthSolver;
end;

procedure btGjkPairDetector.setIgnoreMargin(const ignoreMargin: Boolean);
begin
  m_ignoreMargin := ignoreMargin;
end;

{ btConvexCastResult }
{$HINTS OFF}
procedure btConvexCastResult.DebugDraw(const fraction: btScalar);
begin

end;

procedure btConvexCastResult.drawCoordSystem(const trans: btTransform);
begin

end;
{$HINTS ON}


constructor btConvexCastResult.create;
begin
  m_fraction           := BT_LARGE_FLOAT;
  m_debugDrawer        := nil;
  m_allowedPenetration := 0;
end;

{ btGjkConvexCast }

constructor btGjkConvexCast.create(const convexA, convexB: btConvexShape; const simplexSolver: btSimplexSolverInterface);
begin
  m_simplexSolver := simplexSolver;
  m_convexA       := convexA;
  m_convexB       := convexB;
end;

function btGjkConvexCast.calcTimeOfImpact(const fromA, toA, fromB, toB: btTransform; var resultout: btConvexCastResult): boolean;
var   linVelA,linVelB : btVector3;
       radius,lambda  : btScalar;
       v,n,r,c        : btVector3;
       maxIter        : integer;
       hasResult      : Boolean;
       numIter        : Integer;
       lastLambda     : btScalar;
       dist           : btScalar;
       identityTrans  : btTransform;
       pointCollector : btPointCollector;
       gjk            : btGjkPairDetector;
       input          : btDCDI_ClosestPointInput;
       projectedLinearVelocity : btScalar;
       dLambda                 : btScalar;

begin
  m_simplexSolver.reset;
  /// compute linear velocity for this interval, to interpolate
  //assume no rotation/angular velocity, assert here?
  linVelA := toA.getOriginV^-fromA.getOriginV^;
  linVelB := toB.getOriginV^-fromB.getOriginV^;
  radius  := btScalar(0.001);
  lambda  := btScalar(0);
  v.init(1,0,0);
  maxIter := MAX_ITERATIONS;
  n.InitSame(0);
  hasResult  := false;
  r          := (linVelA-linVelB);
  lastLambda := lambda;
  //btScalar epsilon = btScalar(0.001);
  numIter    := 0;
  //first solution, using GJK
  identityTrans.setIdentity;
//      result.drawCoordSystem(sphereTr);


  gjk.init(m_convexA,m_convexB,m_simplexSolver,nil);//m_penetrationDepthSolver);
  //we don't use margins during CCD
  //    gjk.setIgnoreMargin(true);
  input.m_transformA := fromA;
  input.m_transformB := fromB;
  pointCollector:=btPointCollector.create;
  try
    gjk.getClosestPoints(input,btDCDI_Result(pointCollector),nil);
    hasResult := pointCollector.m_hasResult;
    c         := pointCollector.m_pointInWorld;
    if hasResult then begin
      dist := pointCollector.m_distance;
      n    := pointCollector.m_normalOnBInWorld;
      //not close enough
      while dist > radius do begin
        inc(numIter);
        if numIter > maxIter then begin
          exit(false); //todo: report a failure
        end;
        dLambda                 := btScalar(0);
        projectedLinearVelocity := r.dot(n);
        dLambda                 := dist / (projectedLinearVelocity);
        lambda                  := lambda - dLambda;
        if lambda > btScalar(1) then begin
          exit(false);
        end;
        if lambda < btScalar(0) then begin
          exit(false);
        end;
        //todo: next check with relative epsilon
        if lambda <= lastLambda then begin
          exit(false);
          //n.setValue(0,0,0);
          break;
        end;
        lastLambda := lambda;
        //interpolate to next lambda
        resultout.DebugDraw(lambda);
        input.m_transformA.getOriginV^.setInterpolate3(fromA.getOriginV^,toA.getOriginV^,lambda);
        input.m_transformB.getOriginV^.setInterpolate3(fromB.getOriginV^,toB.getOriginV^,lambda);
        gjk.getClosestPoints(input,btDCDI_Result(pointCollector),nil);
        if pointCollector.m_hasResult then begin
          if pointCollector.m_distance < btScalar(0) then begin
            resultout.m_fraction := lastLambda;
            n                    := pointCollector.m_normalOnBInWorld;
            resultout.m_normal   := n;
            resultout.m_hitPoint := pointCollector.m_pointInWorld;
            exit(true);
          end;
          c    := pointCollector.m_pointInWorld;
          n    := pointCollector.m_normalOnBInWorld;
          dist := pointCollector.m_distance;
        end else begin
          //??
          exit(false);
        end;
      end;
      //is n normalized?
      //don't report time of impact for motion away from the contact normal (or causes minor penetration)
      if (n.dot(r)>=-resultout.m_allowedPenetration) then begin
        exit(false);
      end;
      resultout.m_fraction := lambda;
      resultout.m_normal   := n;
      resultout.m_hitPoint := c;
      exit(true);
    end;
    Result:=false;
  finally
    pointCollector.Free;
  end;
end;

{ btPointCollector }

constructor btPointCollector.create;
begin
  m_distance  := btScalar(BT_LARGE_FLOAT);
  m_hasResult := false;
end;

{$HINTS OFF}
procedure btPointCollector.setShapeIdentifiersA(const partId0, index0: integer);
begin
  //empty
end;

procedure btPointCollector.setShapeIdentifiersB(const partId1, index1: integer);
begin
  //empty
end;
{$HINTS ON}

procedure btPointCollector.addContactPoint(const normalOnBInWorld, pointInWorld: btVector3; const depth: btScalar);
begin
  if depth< m_distance then begin
    m_hasResult        := true;
    m_normalOnBInWorld := normalOnBInWorld;
    m_pointInWorld     := pointInWorld;
    m_distance         := depth;     //negative means penetration
  end;
end;

{ btSubsimplexConvexCast }

constructor btSubsimplexConvexCast.create(const shapeA, shapeB: btConvexShape; const simplexSolver: btSimplexSolverInterface);
begin
  m_simplexSolver := simplexSolver;
  m_convexA := shapeA;
  m_convexB := shapeB;
end;

function btSubsimplexConvexCast.calcTimeOfImpact(const fromA, toA, fromB, toB: btTransform; var resultout: btConvexCastResult): boolean;
var   linVelA,linVelB,r,v,n,
      w,hitA,hitB,
      supVertexA,supVertexB   : btVector3;
      lambda,
      //lastLambda,
      VdotW,
      dist2,epsilon,VdotR     : btScalar;
      interpolatedTransA,
      interpolatedTransB      : btTransform;
      maxIter                 : Integer;
//      hasResult               : Boolean;

begin
  m_simplexSolver.reset;
  linVelA := toA.getOriginV^-fromA.getOriginV^;
  linVelB := toB.getOriginV^-fromB.getOriginV^;
  lambda  := 0;
  interpolatedTransA := fromA;
  interpolatedTransB := fromB;

  ///take relative motion
  r           := (linVelA-linVelB);
  supVertexA  := fromA.opTrans(m_convexA.localGetSupportingVertex(-r*fromA.GetBasisV^));
  supVertexB  := fromB.opTrans(m_convexB.localGetSupportingVertex( r*fromB.GetBasisV^));
  v           := supVertexA-supVertexB;
  maxIter     := MAX_ITERATIONS;
  n.InitSame(0);
//  hasResult   := false;
//  lastLambda  := lambda;
  dist2       := v.length2;

//#ifdef BT_USE_DOUBLE_PRECISION
//  btScalar epsilon = btScalar(0.0001);
//#else
//  btScalar epsilon = btScalar(0.0001);
//#endif //BT_USE_DOUBLE_PRECISION
  epsilon := 0.0001;

  while ( (dist2 > epsilon) and (maxIter>0)) do begin
    dec(maxIter);
    supVertexA := interpolatedTransA.opTrans(m_convexA.localGetSupportingVertex(-v*interpolatedTransA.GetBasisV^));
    supVertexB := interpolatedTransB.opTrans(m_convexB.localGetSupportingVertex( v*interpolatedTransB.GetBasisV^));
    w          := supVertexA-supVertexB;
    VdotW      := v.dot(w);
    if lambda > btScalar(1) then begin
      exit(false);
    end;
    if ( VdotW > btScalar(0)) then  begin
      VdotR := v.dot(r);
      if VdotR >= -(SIMD_EPSILON*SIMD_EPSILON) then begin
        exit(false);
      end else begin
        lambda := lambda - VdotW / VdotR;
        //interpolate to next lambda
        //      x = s + lambda * r;
        interpolatedTransA.getOriginV^.setInterpolate3(fromA.getOriginV^,toA.getOriginV^,lambda);
        interpolatedTransB.getOriginV^.setInterpolate3(fromB.getOriginV^,toB.getOriginV^,lambda);
        //m_simplexSolver->reset();
        //check next line
        w          := supVertexA-supVertexB;
//        lastLambda := lambda;
        n          := v;
//        hasResult  := true;
      end;
    end;
    ///Just like regular GJK only add the vertex if it isn't already (close) to current vertex, it would lead to divisions by zero and NaN etc.
    if not m_simplexSolver.inSimplex(w) then begin
      m_simplexSolver.addVertex( w, supVertexA , supVertexB);
    end;
    if m_simplexSolver.closest(v) then begin
      dist2     := v.length2();
//      hasResult := true;
      //todo: check this normal for validity
      //n=v;
      //printf("V=%f , %f, %f\n",v[0],v[1],v[2]);
      //printf("DIST2=%f\n",dist2);
      //printf("numverts = %i\n",m_simplexSolver->numVertices());
    end else begin
      dist2 := btScalar(0);
    end;
  end;

  //int numiter = MAX_ITERATIONS - maxIter;
//      printf("number of iterations: %d", numiter);

  //don't report a time of impact when moving 'away' from the hitnormal

  resultout.m_fraction := lambda;
  if (n.length2 >= (SIMD_EPSILON*SIMD_EPSILON)) then begin
    resultout.m_normal := n.normalized;
  end else begin
    resultout.m_normal.InitSame(0);
  end;

  //don't report time of impact for motion away from the contact normal (or causes minor penetration)
  if (resultout.m_normal.dot(r)>=-resultout.m_allowedPenetration) then begin
    exit(false);
  end;
  m_simplexSolver.compute_points(hitA,hitB);
  resultout.m_hitPoint:=hitB;
  exit(true);
end;

{ btGjkEpaPenetrationDepthSolver }
{$HINTS OFF}
function btGjkEpaPenetrationDepthSolver.calcPenDepth(const simplexSolver: btSimplexSolverInterface; const convexA, convexB: btConvexShape; const transA, transB: btTransform; out v, wWitnessOnA, wWitnessOnB: btVector3; const debugDraw: btIDebugDraw; const stackAlloc: btStackAllocator): boolean;
var guessVector : btVector3;
    results     : btGjkEpaSolver2_sResults;
begin
  //(void)debugDraw;
  //(void)v;
  //(void)simplexSolver;
  guessVector := transA.getOriginV^-transB.getOriginV^;
  if btGjkEpaSolver2.Penetration(ConvexA,convexB,transA,transB,guessVector,results) then begin
    // debugDraw->drawLine(results.witnesses[1],results.witnesses[1]+results.normal,btVector3(255,0,0));
    // resultOut->addContactPoint(results.normal,results.witnesses[1],-results.depth);
    wWitnessOnA := results.witnesses[0];
    wWitnessOnB := results.witnesses[1];
    v           := results.normal;
    Result      := true;
    exit;
  end else begin
    if btGjkEpaSolver2.Distance(ConvexA,ConvexB,transA,transB,guessVector,results) then begin
      wWitnessOnA := results.witnesses[0];
      wWitnessOnB := results.witnesses[1];
      v           := results.normal;
      Result      := false;
      exit;
    end;
  end;
  Result := false;
end;
{$HINTS ON}

{ btGjkEpaSolver2 }

type
// MinkowskiDiff

 { gepaMinkowskiDiff }

 gepaMinkowskiDiff=object
   m_shapes              : array [0..1] of btConvexShape;
   m_toshape1            : btMatrix3x3;
   m_toshape0            : btTransform;
   m_enableMArgin        : boolean;
   Ls0,Ls1                : btConvexGetSupportingVertexFunc; // (btConvexShape::*Ls)(const btVector3&) const;
   procedure EnableMargin  (const enable:boolean);
   function  Support0      (const d:btVector3):btVector3;FOS_INLINE;
   function  Support1      (const d:btVector3):btVector3;FOS_INLINE;
   function  Support       (const d:btVector3):btVector3;FOS_INLINE;
   function  Support       (const d:btVector3;const index:Cardinal):btVector3;FOS_INLINE;
   procedure Init;
 end;
 tShape = gepaMinkowskiDiff;

{ gepaMinkowskiDiff }

procedure gepaMinkowskiDiff.EnableMargin(const enable: boolean);
begin
  //
  btAssert(false); //FOSTODO howto ?
  m_enableMargin := enable;
  if enable then begin
    Ls0 := @m_shapes[0].localGetSupportVertexNonVirtual; //Ls = &btConvexShape::localGetSupportVertexNonVirtual;
    Ls1 := @m_shapes[1].localGetSupportVertexNonVirtual;
  end else begin
    //Ls := &btConvexShape::localGetSupportVertexWithoutMarginNonVirtual;
    Ls0 := @m_shapes[0].localGetSupportVertexWithoutMarginNonVirtual;
    Ls1 := @m_shapes[1].localGetSupportVertexWithoutMarginNonVirtual;
  end;
end;

function gepaMinkowskiDiff.Support0(const d: btVector3): btVector3;
begin
  result := Ls0(d);
end;

function gepaMinkowskiDiff.Support1(const d: btVector3): btVector3;
begin
  result := Ls1(d);
end;

function gepaMinkowskiDiff.Support(const d: btVector3): btVector3;
begin
  result := Support0(d)-Support1(-d);
end;

function gepaMinkowskiDiff.Support(const d: btVector3; const index: Cardinal): btVector3;
begin
  if index<>0 then begin
    Result := Support1(d);
  end else begin
    Result := Support0(d);
  end;
end;

procedure gepaMinkowskiDiff.Init;
begin
  m_shapes[0] := nil;
  m_shapes[1] := nil;
end;

type
  // GJK
  sSV=record
    d,w : btVector3;
  end;
  PsSV = ^sSV;

  sSimplex=record
    c    : array [0..3] of PsSV;
    p    : array [0..3] of btScalar;
    rank : cardinal;
  end;
  PsSimplex = ^sSimplex;

  gjkeStatus   = (gjkValid,gjkInside,gjkFailed);

  { gepaGJK }

  gepaGJK=object
    m_shape     : tShape;
    m_ray       : btVector3;
    m_distance  : btScalar;
    m_simplices : array [0..1] of sSimplex;
    m_store     : array [0..3] of sSV;
    m_free      : array [0..3] of PsSV;
    m_nfree     : cardinal;
    m_current   : Cardinal;
    m_simplex   : PsSimplex;
    m_status    : gjkeStatus;
    procedure  getsupport      (const d:btVector3 ; var sv:sSV);
    procedure  appendvertice   (var simplex:sSimplex;const v:btVector3);
    procedure  removevertice   (var simplex:sSimplex);
    procedure  Initialize      ;
    function   Evaluate        (const shapearg : tShape   ; const guess:btVector3):gjkeStatus;
    function   EncloseOrigin   : boolean;
  end;

{ gepaGJK }

procedure gepaGJK.getsupport(const d: btVector3; var sv: sSV);
begin
  sv.d := d/d.length;
  sv.w := m_shape.Support(sv.d);
end;

procedure gepaGJK.appendvertice(var simplex: sSimplex; const v: btVector3);
begin
  simplex.p[simplex.rank] := 0;
  dec(m_nfree);
  simplex.c[simplex.rank] := m_free[m_nfree];
  getsupport(v,simplex.c[simplex.rank]^);
  inc(simplex.rank);
end;

procedure gepaGJK.removevertice(var simplex:sSimplex);
begin
  dec(simplex.rank);
  m_free[m_nfree] := simplex.c[simplex.rank];
  inc(m_nfree);
end;

procedure gepaGJK.Initialize;
begin
  m_ray.InitSame(0);
  m_nfree         :=       0;
  m_status        :=       gjkFailed;
  m_current       :=       0;
  m_distance      :=       0;
end;

function projectorigin(const a, b: btVector3; w: PbtScalar; var m: cardinal): btScalar;
var d    : btVector3;
    l,t  : btScalar;
begin
  d := b-a;
  l := d.length2;
  if l>GJK_SIMPLEX2_EPS then begin
    t := btDecide(l>0,-btDot(a,d)/l,0);
    if t>=1 then begin
      w[0] := 0; w[1] := 1; m:= 2;
      result :=b.length2;
      exit;
    end else
    if t<=0 then begin
      w[0] := 1; w[1] := 0; m:= 1;
      result :=a.length2;
      exit;
    end else begin
      w[1] := t;
      w[0] := 1-w[1] ; m:= 3;
      Result := (a+d*t).length2;
      exit;
    end;
  end;
  Result := -1;
end;

var cimd3 : array [0..2] of cardinal = (1,2,0); // == i1m3
     i2m3 : array [0..2] of cardinal = (2,0,1);

function projectorigin(const a, b, c: btVector3; w: PbtScalar; var m: cardinal): btScalar;
var  vt         : array [0..2] of PbtVector3; //FOSTODO const -> refactor it out (param as array ?)
     dl         : array [0..2] of btVector3;
     n,p        : btVector3;
     l,mindist  : btScalar;
     subw       : array [0..2] of btScalar;
     subm,i,j   : cardinal;
     subd,s     : btScalar; // j = const, subd=const;

begin
  vt[0] :=  @a ; vt[1] :=  @b ; vt[2] :=  @c;
  dl[0] := a-b ; dl[1] := b-c ; dl[2] := c-a;
  n     := btCross(dl[0],dl[1]);
  l     := n.length2;
  if l>GJK_SIMPLEX3_EPS then begin
    mindist := -1;
    subw[0] :=  0; subw[1] := 0;
    subm    :=  0;
    for i := 0 to 2 do begin
      if  btDot(vt[i]^,btCross(dl[i],n)) > 0 then begin
        j    := cimd3[i];
        subd := projectorigin(vt[i]^,vt[j]^,subw,subm);
        if (mindist<0) or  (subd<mindist) then begin
          mindist     := subd;
          m           := btDecide((subm and 1)<>0, 1 SHL i, 0) + btDecide ( (subm and 2)<>0, 1 SHL j, 0) ; // static_cast<U>(((subm&1)?1<<i:0)+((subm&2)?1<<j:0));
          w[i]        := subw[0];
          w[j]        := subw[1];
          w[cimd3[j]] := 0;
        end;
      end;
    end;
    if mindist<0 then begin
      s       := btSqrt(l);
      p       := n*( btDot(a,n) / l);
      mindist := p.length2;
      m       := 7;
      w[0]    :=       (btCross(dl[1],b-p)).length / s;
      w[1]    :=       (btCross(dl[2],c-p)).length / s;
      w[2]    :=       1-(w[0]+w[1]);
    end;
    Result := mindist;
    exit;
  end;
  result := -1;
end;

function det(const a,b,c : btVector3):btScalar;FOS_INLINE;
begin
  result := a.y*b.z*c.x+a.z*b.x*c.y -
            a.x*b.z*c.y-a.y*b.x*c.z+
            a.x*b.y*c.z-a.z*b.y*c.x;
end;


function projectorigin(const a, b, c, d: btVector3; w: PbtScalar; var m: cardinal): btScalar;
var vt         : array [0..3] of PbtVector3; //FOSTODO const -> refactor it out (param as array ?)
    dl         : array [0..2] of btVector3;
    vl         : btScalar;
    ng         : boolean;
    mindist    : btScalar;
    subw       : array [0..3] of btScalar;
    subm,i,j   : cardinal;
    subd,s     : btScalar; // j = const, subd=const;

begin
  vt[0] :=  @a ; vt[1] :=  @b ; vt[2] :=  @c; vt[3] :=  @d;
  dl[0] := a-d ; dl[1] := b-d ; dl[2] := c-d;
  vl    := det(dl[0],dl[1],dl[2]);
  ng    := (vl*btDot(a,btCross(b-c,a-b))) <=0;
  if  ng and (btFabs(vl)>GJK_SIMPLEX4_EPS) then begin
    mindist := -1;
    subw[0] :=  0; subw[1] := 0; subw[2] := 0;
    subm    :=  0;
    for i:=0 to 2 do begin
      j := cimd3[i];
      s :=vl*btDot(d,btCross(dl[i],dl[j]));
      if s>0 then begin
        subd := projectorigin(vt[i]^,vt[j]^,d,subw,subm);
        if (mindist<0) or  (subd<mindist) then begin
          mindist     := subd;
          m           := btDecide( (subm and 1)<>0,1 SHL i,0)+
                         btDecide( (subm and 2)<>0,1 SHL j,0)+
                         btDecide( (subm and 4)<>0,8,0);
          w[i]        := subw[0];
          w[j]        := subw[1];
          w[cimd3[j]] := 0;
          w[3]        := subw[2];
        end;
      end;
    end;
    if mindist<0 then begin
      mindist := 0;
      m       := 15;
      w[0]    := det(c,b,d)/vl;
      w[1]    := det(a,c,d)/vl;
      w[2]    := det(b,a,d)/vl;
      w[3]    := 1-(w[0]+w[1]+w[2]);
    end;
    Result := mindist;
    exit;
  end;
  result := -1;
end;



const cDummy100 : btVector3 = (v:(m_floats : (1,0,0,0)));

function gepaGJK.Evaluate(const shapearg: tShape; const guess: btVector3): gjkeStatus;
var iterations,clastw : cardinal;
    sqdist,alpha,sqrl,
    rl,omega          : btScalar;
    lastw             : array [0..3] of btVector3;
    next,i,ni         : cardinal;
    cs,ns             : PsSimplex;
    w                 : btVector3;
    found             : Boolean;
    weights           : array [0..3] of btScalar;
    mask              : cardinal;


begin
  iterations := 0 ; clastw :=0;
  sqdist     := 0 ; alpha  :=0;
  // /* Initialize solver            */
  m_free[0]                       :=       @m_store[0];
  m_free[1]                       :=       @m_store[1];
  m_free[2]                       :=       @m_store[2];
  m_free[3]                       :=       @m_store[3];
  m_nfree                         :=       4;
  m_current                       :=       0;
  m_status                        :=       gjkValid;
  m_shape                         :=       shapearg;
  m_distance                      :=       0;
  // /* Initialize simplex
  m_simplices[0].rank             :=       0;
  m_ray                           :=       guess;
  sqrl                            :=       m_ray.length2;
  appendvertice (m_simplices[0],btDecide(sqrl>0,-m_ray,cDummy100));
  m_simplices[0].p[0]             :=       1;
  m_ray                           :=       m_simplices[0].c[0]^.w;
  sqdist                          :=       sqrl;
  lastw[0]                        :=       m_ray;
  lastw[1]                        :=       m_ray;
  lastw[2]                        :=       m_ray;
  lastw[3]                        :=       m_ray;
  repeat
    next    := 1-m_current;
    cs      := @m_simplices[m_current];
    ns      := @m_simplices[next];
    ///* Check zero
    rl := m_ray.length;
    if rl<GJK_MIN_DISTANCE then begin
      m_status := gjkInside;
      break;
    end;
    //  Append new vertice in -'v' direction
    appendvertice(cs^,-m_ray);
    w     := cs^.c[cs^.rank-1]^.w;
    found := false;
    for i:=0 to 3 do begin
     if (w-lastw[i]).length2()<GJK_DUPLICATED_EPS then begin
       found :=true;
       break;
     end;
    end;
    if found then begin
    //* Return old simplex
      removevertice(m_simplices[m_current]);
      break;
    end else begin
    //* Update lastw
      clastw        := (clastw+1) mod 3;
      lastw[clastw] := w;
    end;
    //* Check for termination
    omega := btDot(m_ray,w)/rl;
    alpha := btMax(omega,alpha);
    if ((rl-alpha)-(GJK_ACCURARY*rl))<=0 then begin
      //* Return old simplex
      removevertice(m_simplices[m_current]);
      break;
    end;
    // Reduce simplex
    mask :=0;
    case cs^.rank of
      2:      sqdist := projectorigin(cs^.c[0]^.w, cs^.c[1]^.w, weights,mask);
      3:      sqdist := projectorigin(cs^.c[0]^.w, cs^.c[1]^.w, cs^.c[2]^.w, weights, mask);
      4:      sqdist := projectorigin(cs^.c[0]^.w, cs^.c[1]^.w, cs^.c[2]^.w, cs^.c[3]^.w, weights,mask);
    end;
    if  sqdist >= 0 then begin
     // Valid
      ns^.rank   := 0;
      m_ray.InitSame(0);
      m_current := next;
      ni        := cs^.rank;
      for i :=0 to ni-1 do begin
        if (mask and (1 SHL i)) <> 0 then begin
          ns^.c[ns^.rank] := cs^.c[i];
          ns^.p[ns^.rank] := weights[i]; inc(ns^.rank);
          m_ray           += cs^.c[i]^.w*weights[i];
        end else begin
          m_free[m_nfree] := cs^.c[i]; inc(m_nfree);
        end;
      end;
      if mask=15 then begin
        m_status := gjkInside;
      end;
    end else begin
    /// Return old simplex
      removevertice(m_simplices[m_current]);
      break;
    end;
    inc(iterations);
    if iterations >= GJK_MAX_ITERATIONS then begin
      m_status := gjkFailed;
    end;
  until m_status<>gjkValid;
  m_simplex := @m_simplices[m_current];
  case m_status of
    gjkValid   : m_distance := m_ray.length;
    gjkInside  : m_distance := 0;
  end;
  Result := m_status;
end;

function gepaGJK.EncloseOrigin: boolean;
var i          : cardinal;
    axis,d,p,n : btVector3;
begin
  case m_simplex^.rank of
    1:begin
        for i := 0 to 2 do begin
          axis.initsame(0);
          axis[i] := 1;
          appendvertice(m_simplex^, axis);
          if EncloseOrigin then exit(true);
          removevertice(m_simplex^);
          appendvertice(m_simplex^,-axis);
          if EncloseOrigin then exit(true);
          removevertice(m_simplex^);
         end;
      end;
    2:begin
        d := m_simplex^.c[1]^.w-m_simplex^.c[0]^.w;
        for i := 0 to 2 do begin
          axis.initsame(0);
          axis[i] := 1;
          p       := btCross(d,axis);
          if p.length2>0 then begin
            appendvertice(m_simplex^, p);
            if EncloseOrigin then exit(true);
            removevertice(m_simplex^);
            appendvertice(m_simplex^,-p);
            if EncloseOrigin then exit(true);
            removevertice(m_simplex^);
          end;
        end;
      end;
     3:begin
        n := btCross(m_simplex^.c[1]^.w - m_simplex^.c[0]^.w , m_simplex^.c[2]^.w - m_simplex^.c[0]^.w);
        if n.length2>0 then begin
          appendvertice(m_simplex^,n);
          if EncloseOrigin then exit(true);
          removevertice(m_simplex^);
          appendvertice(m_simplex^,-n);
          if EncloseOrigin then exit(true);
          removevertice(m_simplex^);
        end;
       end;
     4:begin
        if(btFabs(det(m_simplex^.c[0]^.w - m_simplex^.c[3]^.w,
                      m_simplex^.c[1]^.w - m_simplex^.c[3]^.w,
                      m_simplex^.c[2]^.w - m_simplex^.c[3]^.w))>0) then exit(true); //FOSTODO Check ABS necessary or simple <> 0 ?
       end;
  end;
  Result := false;
end;


  // EPA
type
  PsFace = ^sFace;
  sFace = record
    n    : btVector3;
    d,p  : btScalar;
    c    : array [0..2] of PsSV;
    f    : array [0..2] of PsFace;
    l    : array [0..1] of PsFace;
    e    : array [0..2] of Cardinal;
    pass : Cardinal;
  end;

  sList = record
    root  : PsFace;
    count : Cardinal;
    //init nil / 0
  end;

  { sHorizon }

  sHorizon = object
    cf : PsFace;
    ff : PsFace;
    nf : Cardinal;
    procedure Init;
  end;

  epaeStatus = ( epaValid,epaTouching,epaDegenerated,epaNonConvex,epaInvalidHull,epaOutOfFaces,epaOutOfVertices,epaAccuraryReached,epaFallBack,epaFailed );

  { gepaEPA }

  gepaEPA = object
    m_status   :  epaeStatus;
    m_result   :  sSimplex;
    m_normal   :  btVector3;
    m_depth    :  btScalar;
    m_sv_store : array [0..EPA_MAX_VERTICES-1] of sSV;
    m_fc_store : array [0..EPA_MAX_FACES-1] of sFace;
    m_nextsv   : Cardinal;
    m_hull     : sList;
    m_stock    : sList;
    procedure  bind       (const fa:PsFace;const ea:cardinal ; const fb : PsFace ; const eb:Cardinal);static;FOS_INLINE;
    procedure  append     (var   list : sList; const face : PsFace);FOS_INLINE;static;
    procedure  remove     (var   list : sList; const face : PsFace);
    procedure  Initialize ;
    function   newface    (const a,b,c :PsSV ; const forced : boolean):PsFace;
    function   findbest   : PsFace;
    function   expand     (const pass:cardinal; const w : PsSV; const f:PsFace;const e:cardinal;var horizon:sHorizon):boolean;
    function   Evaluate   (const gjk:gepaGJK;const guess:btVector3):epaeStatus;
  end;

{ sHorizon }

procedure sHorizon.Init;
begin
  cf := nil;
  ff := nil;
  nf := 0;
end;

{ gepaEPA }

procedure gepaEPA.bind(const fa: PsFace; const ea: cardinal; const fb: PsFace; const eb: Cardinal);
begin
  fa^.e[ea] := byte(eb);
  fa^.f[ea] := fb;
  fb^.e[eb] := byte(ea);
  fb^.f[eb] := fa;
end;

procedure gepaEPA.append(var list: sList; const face: PsFace);
begin
  face^.l[0]  := nil;
  face^.l[1]  := list.root;
  if assigned (list.root) then begin
    list.root^.l[0] := face;
  end;
  list.root  := face;
  inc(list.count);
end;

procedure gepaEPA.remove(var list: sList; const face: PsFace);
begin
  if assigned(face^.l[1]) then begin
    face^.l[1]^.l[0] := face^.l[0];
  end;
  if assigned(face^.l[0]) then begin
    face^.l[0]^.l[1] := face^.l[1];
  end;
  if face=list.root then begin
    list.root := face^.l[1];
  end;
  dec(list.count);
end;

procedure gepaEPA.Initialize;
var i:cardinal;
begin
  m_status  := epaFailed;
  m_normal.InitSame(0);
  m_depth   := 0;
  m_nextsv  := 0;
  for i := 0 to EPA_MAX_FACES-1 do begin
    append(m_stock,@m_fc_store[EPA_MAX_FACES-i-1]);
  end;
end;

function gepaEPA.newface(const a, b, c: PsSV; const forced: boolean): PsFace;
var face : PsFace;
    l    : btScalar;
    v    : Boolean;
begin
  if assigned(m_stock.root) then begin
    face     :=  m_stock.root;
    remove(m_stock,face);
    append(m_hull,face);
    face^.pass := 0;
    face^.c[0] := a;
    face^.c[1] := b;
    face^.c[2] := c;
    face^.n    := btCross(b^.w - a^.w, c^.w - a^.w);
    l          := face^.n.length;
    v          := l>EPA_ACCURACY;
    face^.p    := btMin(btMin(btDot(a^.w,btCross(face^.n, a^.w - b^.w)),
                              btDot(b^.w,btCross(face^.n, b^.w - c^.w))),
                              btDot(c^.w,btCross(face^.n, c^.w - a^.w))) / btDecide (v,l,1);
    face^.p    := btDecide(face^.p >= -EPA_INSIDE_EPS,  0 ,face^.p);
    if v then begin
      face^.d := btDot(a^.w,face^.n)/l;
      face^.n /= l;
      if forced or (face^.d>=-EPA_PLANE_EPS) then begin
        Result   := face;
      end else begin
        m_status := epaNonConvex;
      end;
    end else begin
      m_status := epaDegenerated;
    end;
    remove(m_hull,face);
    append(m_stock,face);
    exit(nil);
  end;
  if assigned(m_stock.root) then begin
    m_status := epaOutOfVertices;
  end else begin
    m_status := epaOutOfFaces;
  end;
  result := nil;
end;

function gepaEPA.findbest: PsFace;
var minf,f        : PsFace;
    mind,maxp,sqd : btScalar;
begin
  minf := m_hull.root;
  mind := minf^.d*minf^.d;
  maxp := minf^.p;
  f:=minf^.l[1];
//  for(sFace* f=minf->l[1];f;f=f->l[1])
  while assigned(f) do begin
    sqd := f^.d * f^.d;
    if (f^.p >= maxp) and (sqd<mind) then begin
      minf := f;
      mind := sqd;
      maxp := f^.p;
    end;
    f:=f^.l[1];
  end;
  Result := minf;
end;

function gepaEPA.expand(const pass:cardinal ; const w: PsSV; const f: PsFace; const e: cardinal; var horizon: sHorizon): boolean;
var e1,e2 : Cardinal;
    nf    : PsFace;
begin
  if f^.pass<>pass then begin
    e1 := cimd3[e];
    if (btDot(f^.n,w^.w) - f^.d) < -EPA_PLANE_EPS then begin
      nf := newface(f^.c[e1],f^.c[e],w,false);
      if assigned(nf) then begin
        bind(nf,0,f,e);
        if assigned(horizon.cf) then begin
          bind(horizon.cf,1,nf,2)
        end else begin
          horizon.ff := nf;
        end;
        horizon.cf:=nf;
        inc(horizon.nf);
        Result := true;
        exit;
      end;
    end else begin
      e2      := i2m3[e];
      f^.pass := byte(pass);
      if(expand(pass,w,f^.f[e1],f^.e[e1],horizon) and expand(pass,w,f^.f[e2],f^.e[e2],horizon))  then begin
        remove(m_hull,f);
        append(m_stock,f);
        Result := true;
        exit;
      end;
    end;
  end;
  Result :=false;
end;

function gepaEPA.Evaluate(const gjk: gepaGJK; const guess: btVector3): epaeStatus;
var simplex      : PsSimplex;
    f,best       : PsFace;
    tetra        : Array [0..3] of PsFace;
    outer        : sFace;
    pass,j,
    iterations   : Cardinal;
    horizon      : sHorizon;
    w            : PsSV;
    valid        : Boolean;
    wdist,sum,nl : btScalar;
    projection   : btVector3;
begin
  horizon.Init;
  simplex := gjk.m_simplex;
  if (simplex^.rank>1) and gjk.EncloseOrigin then begin
    //  Clean up
    while assigned(m_hull.root) do begin
      f := m_hull.root;
      remove(m_hull,f);
      append(m_stock,f);
    end;
    m_status := epaValid;
    m_nextsv := 0;
    // Orient simplex
    if det(simplex^.c[0]^.w - simplex^.c[3]^.w, simplex^.c[1]^.w - simplex^.c[3]^.w, simplex^.c[2]^.w - simplex^.c[3]^.w)<0 then begin
      btSwap(simplex^.c[0],simplex^.c[1]);
      btSwap(simplex^.p[0],simplex^.p[1]);
    end;
    // Build initial hull
    tetra[0] := newface(simplex^.c[0],simplex^.c[1],simplex^.c[2],true);
    tetra[1] := newface(simplex^.c[1],simplex^.c[0],simplex^.c[3],true);
    tetra[2] := newface(simplex^.c[2],simplex^.c[1],simplex^.c[3],true);
    tetra[3] := newface(simplex^.c[0],simplex^.c[2],simplex^.c[3],true);
    if m_hull.count=4 then begin
      best       := findbest;
      outer      := best^;
      pass       := 0;
      iterations := 0;
      bind(tetra[0],0,tetra[1],0);
      bind(tetra[0],1,tetra[2],0);
      bind(tetra[0],2,tetra[3],0);
      bind(tetra[1],1,tetra[3],2);
      bind(tetra[1],2,tetra[2],1);
      bind(tetra[2],2,tetra[3],1);
      m_status := epaValid;
      while(iterations<EPA_MAX_ITERATIONS) do begin
        if m_nextsv<EPA_MAX_VERTICES then begin
          w     := @m_sv_store[m_nextsv]; inc(m_nextsv);
          valid := true;
          inc(pass);
          best^.pass := byte(pass);
          gjk.getsupport(best^.n,w^);
          wdist := btDot(best^.n,w^.w) - best^.d;
          if wdist>EPA_ACCURACY then begin
            j := 0;
            while ( (j<3) and valid ) do begin
             valid := valid and expand(pass,w,best^.f[j],best^.e[j],horizon);
             inc(j);
            end;
            if valid and (horizon.nf >= 3) then begin
              bind(horizon.cf,1,horizon.ff,2);
              remove(m_hull,best);
              append(m_stock,best);
              best:=findbest;
              if best^.p >= outer.p then begin
                outer := best^;
              end;
            end else begin
              m_status := epaInvalidHull;
              break;
            end;
          end else begin
           m_status := epaAccuraryReached;
           break;
          end;
        end else begin
          m_status := epaOutOfVertices;
          break;
        end;
        inc(iterations);
      end;
      projection      := outer.n*outer.d;
      m_normal        := outer.n;
      m_depth         := outer.d;
      m_result.rank   := 3;
      m_result.c[0]   := outer.c[0];
      m_result.c[1]   := outer.c[1];
      m_result.c[2]   := outer.c[2];
      m_result.p[0]   := btCross(outer.c[1]^.w-projection,outer.c[2]^.w-projection).length;
      m_result.p[1]   := btCross(outer.c[2]^.w-projection,outer.c[0]^.w-projection).length;
      m_result.p[2]   := btCross(outer.c[0]^.w-projection,outer.c[1]^.w-projection).length;
      sum             := m_result.p[0]+m_result.p[1]+m_result.p[2];
      m_result.p[0]   /=      sum;
      m_result.p[1]   /=      sum;
      m_result.p[2]   /=      sum;
      Result := m_status;
      exit;
    end;
  end;
  m_status := epaFallBack;
  m_normal := -guess;
  nl       := m_normal.length;
  if nl>0 then begin
    m_normal := m_normal/nl;
  end else begin
    m_normal.init(1,0,0);
  end;
  m_depth       := 0;
  m_result.rank := 1;
  m_result.c[0] := simplex^.c[0];
  m_result.p[0] := 1;
  Result        := m_status;
end;

procedure gepaepa_Initialize(const shape0:btConvexShape;const wtrs0:btTransform;const shape1:btConvexShape;const wtrs1 : btTransform;var results : btGjkEpaSolver2_sResults;var shape : tShape; const withmargins:boolean);
begin
  results.witnesses[0].InitSame(0);
  results.witnesses[1].InitSame(0);
  results.status       := ges2_Separated;
  shape.m_shapes[0]    := shape0;
  shape.m_shapes[1]    := shape1;
  shape.m_toshape1     := wtrs1.GetBasisV^.transposeTimes(wtrs0.getBasisV^);
  shape.m_toshape0     := wtrs0.inverseTimes(wtrs1);
  shape.EnableMargin(withmargins);
end;

function btGjkEpaSolver2.StackSizeRequirement: integer;
begin
  Result := sizeof(gepaEPA)+sizeof(gepaGJK);
end;

function btGjkEpaSolver2.Distance(const shape0, shape1: btConvexShape; const wtrs0, wtrs1: btTransform; const guess: btVector3; var results: btGjkEpaSolver2_sResults): boolean;
var shape      : tShape;
    gjk        : gepaGJK;
    gjk_status : gjkeStatus;
    w0,w1      : btVector3;
    p          : btScalar;
    i          : Cardinal;
begin
  shape.init;
  gepaepa_Initialize(shape0,wtrs0,shape1,wtrs1,results,shape,false);
  gjk_status := gjk.Evaluate(shape,guess);
  if gjk_status=gjkValid then begin
    w0.zero;w1.zero;
    for i:=0 to gjk.m_simplex^.rank-1 do begin
      p  := gjk.m_simplex^.p[i];
      w0 += shape.Support( gjk.m_simplex^.c[i]^.d,0)*p;
      w1 += shape.Support(-gjk.m_simplex^.c[i]^.d,1)*p;
    end;
    results.witnesses[0] := wtrs0*w0;
    results.witnesses[1] := wtrs0*w1;
    results.normal       :=       w0-w1;
    results.distance     :=       results.normal.length();
    if results.distance<GJK_MIN_DISTANCE then begin
      results.normal     /=  results.distance;
    end else begin
      results.normal     /=  1;
    end;
    result := true;
    exit;
  end else begin
    if gjk_status = gjkInside then begin
      results.status := ges2_Penetrating;
    end else begin
      results.status := ges2_GJK_Failed;
    end;
    Result := false;
  end;
end;

function btGjkEpaSolver2.Penetration(const shape0, shape1: btConvexShape; const wtrs0, wtrs1: btTransform; const guess: btVector3; out results: btGjkEpaSolver2_sResults; const usemargins: boolean): boolean;
var shape      : tShape;
    gjk        : gepaGJK;
    epa        : gepaEPA;
    epa_status : epaeStatus;
    gjk_status : gjkeStatus;
    w0         : btVector3;
    i          : Cardinal;
begin
  shape.Init;
  results.init;
  gepaepa_Initialize(shape0,wtrs0,shape1,wtrs1,results,shape,usemargins);
  gjk_status := gjk.Evaluate(shape,-guess);
  case gjk_status of
   gjkInside: begin
     epa_status := epa.Evaluate(gjk,-guess);
     if epa_status <> epaFailed then begin
       w0.zero;
       for i:=0 to epa.m_result.rank-1 do begin
         w0 += shape.Support(epa.m_result.c[i]^.d,0)*epa.m_result.p[i];
       end;
       results.status       := ges2_Penetrating;
       results.witnesses[0] := wtrs0*w0;
       results.witnesses[1] := wtrs0*(w0-epa.m_normal*epa.m_depth);
       results.normal       := -epa.m_normal;
       results.distance     := -epa.m_depth;
       Result               := true;
       exit;
     end else begin
       results.status       := ges2_EPA_Failed;
     end;
   end;
   gjkFailed: begin
     results.status := ges2_GJK_Failed;
   end;
  end;
  Result := false;
end;

function btGjkEpaSolver2.SignedDistance(const shape0, shape1: btConvexShape; const wtrs0, wtrs1: btTransform; const guess: btVector3; var results: btGjkEpaSolver2_sResults): boolean;
begin
  if not Distance(shape0,shape1,wtrs0,wtrs1,guess,results) then begin
    Result := Penetration(shape0,shape1,wtrs0,wtrs1,guess,results,false);
  end else begin
    Result := true;
  end;
end;

function btGjkEpaSolver2.SignedDistance(const position: btVector3; const margin: btScalar; const shape0: btConvexShape; const wtrs0: btTransform; var results: btGjkEpaSolver2_sResults): btScalar;
var shape1  : btSphereShape;
    wtrs1   : btTransform;
    shape   : tShape;
    gjk     : gepaGJK;
    gjk_status : gjkeStatus;
    w0,w1      : btVector3;
    p          : btScalar;
    i          : Cardinal;
    delta      : btVector3;
    marginl,
    length     : btScalar;

begin
  shape.init;
  shape1.Create(margin);
  wtrs1.Init(btQuaternion.Init4S(0,0,0,1),position);
  gepaepa_Initialize(shape0,wtrs0,shape1,wtrs1,results,shape,false);
  gjk_status := gjk.Evaluate(shape,btVector3.InitSameS(1));
  if gjk_status = gjkValid then begin
    w0.zero;w1.zero;
    for i := 0 to gjk.m_simplex^.rank-1 do begin
      p  := gjk.m_simplex^.p[i];
      w0 += shape.Support( gjk.m_simplex^.c[i]^.d,0)*p;
      w1 += shape.Support(-gjk.m_simplex^.c[i]^.d,1)*p;
    end;
    results.witnesses[0] := wtrs0*w0;
    results.witnesses[1] := wtrs0*w1;
    delta   :=  results.witnesses[1] - results.witnesses[0];
    marginl := shape0.getMarginNonVirtual + shape1.getMarginNonVirtual;
    length  := delta.length;
    results.normal := delta/length;
    results.witnesses[0] += results.normal*marginl;
    Result := length-margin;
    exit;
  end else begin
    if gjk_status = gjkInside then begin
      if Penetration(shape0,shape1,wtrs0,wtrs1,gjk.m_ray,results) then begin
        delta  := results.witnesses[0] - results.witnesses[1];
        length := delta.length;
        if length >= SIMD_EPSILON then begin
          results.normal  := delta/length;
        end;
        Result := -length;
        exit;
      end;
    end;
  end;
  Result := SIMD_INFINITY;
end;

{ btMinkowskiPenetrationDepthSolver }

function btMinkowskiPenetrationDepthSolver.getPenetrationDirections: PbtVector3;
begin
  result:=nil;
  abort;
  //static btVector3	sPenetrationDirections[NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2] =
  //{
  //btVector3(btScalar(0.000000) , btScalar(-0.000000),btScalar(-1.000000)),
  //btVector3(btScalar(0.723608) , btScalar(-0.525725),btScalar(-0.447219)),
  //btVector3(btScalar(-0.276388) , btScalar(-0.850649),btScalar(-0.447219)),
  //btVector3(btScalar(-0.894426) , btScalar(-0.000000),btScalar(-0.447216)),
  //btVector3(btScalar(-0.276388) , btScalar(0.850649),btScalar(-0.447220)),
  //btVector3(btScalar(0.723608) , btScalar(0.525725),btScalar(-0.447219)),
  //btVector3(btScalar(0.276388) , btScalar(-0.850649),btScalar(0.447220)),
  //btVector3(btScalar(-0.723608) , btScalar(-0.525725),btScalar(0.447219)),
  //btVector3(btScalar(-0.723608) , btScalar(0.525725),btScalar(0.447219)),
  //btVector3(btScalar(0.276388) , btScalar(0.850649),btScalar(0.447219)),
  //btVector3(btScalar(0.894426) , btScalar(0.000000),btScalar(0.447216)),
  //btVector3(btScalar(-0.000000) , btScalar(0.000000),btScalar(1.000000)),
  //btVector3(btScalar(0.425323) , btScalar(-0.309011),btScalar(-0.850654)),
  //btVector3(btScalar(-0.162456) , btScalar(-0.499995),btScalar(-0.850654)),
  //btVector3(btScalar(0.262869) , btScalar(-0.809012),btScalar(-0.525738)),
  //btVector3(btScalar(0.425323) , btScalar(0.309011),btScalar(-0.850654)),
  //btVector3(btScalar(0.850648) , btScalar(-0.000000),btScalar(-0.525736)),
  //btVector3(btScalar(-0.525730) , btScalar(-0.000000),btScalar(-0.850652)),
  //btVector3(btScalar(-0.688190) , btScalar(-0.499997),btScalar(-0.525736)),
  //btVector3(btScalar(-0.162456) , btScalar(0.499995),btScalar(-0.850654)),
  //btVector3(btScalar(-0.688190) , btScalar(0.499997),btScalar(-0.525736)),
  //btVector3(btScalar(0.262869) , btScalar(0.809012),btScalar(-0.525738)),
  //btVector3(btScalar(0.951058) , btScalar(0.309013),btScalar(0.000000)),
  //btVector3(btScalar(0.951058) , btScalar(-0.309013),btScalar(0.000000)),
  //btVector3(btScalar(0.587786) , btScalar(-0.809017),btScalar(0.000000)),
  //btVector3(btScalar(0.000000) , btScalar(-1.000000),btScalar(0.000000)),
  //btVector3(btScalar(-0.587786) , btScalar(-0.809017),btScalar(0.000000)),
  //btVector3(btScalar(-0.951058) , btScalar(-0.309013),btScalar(-0.000000)),
  //btVector3(btScalar(-0.951058) , btScalar(0.309013),btScalar(-0.000000)),
  //btVector3(btScalar(-0.587786) , btScalar(0.809017),btScalar(-0.000000)),
  //btVector3(btScalar(-0.000000) , btScalar(1.000000),btScalar(-0.000000)),
  //btVector3(btScalar(0.587786) , btScalar(0.809017),btScalar(-0.000000)),
  //btVector3(btScalar(0.688190) , btScalar(-0.499997),btScalar(0.525736)),
  //btVector3(btScalar(-0.262869) , btScalar(-0.809012),btScalar(0.525738)),
  //btVector3(btScalar(-0.850648) , btScalar(0.000000),btScalar(0.525736)),
  //btVector3(btScalar(-0.262869) , btScalar(0.809012),btScalar(0.525738)),
  //btVector3(btScalar(0.688190) , btScalar(0.499997),btScalar(0.525736)),
  //btVector3(btScalar(0.525730) , btScalar(0.000000),btScalar(0.850652)),
  //btVector3(btScalar(0.162456) , btScalar(-0.499995),btScalar(0.850654)),
  //btVector3(btScalar(-0.425323) , btScalar(-0.309011),btScalar(0.850654)),
  //btVector3(btScalar(-0.425323) , btScalar(0.309011),btScalar(0.850654)),
  //btVector3(btScalar(0.162456) , btScalar(0.499995),btScalar(0.850654))
  //};
  //
  //return sPenetrationDirections;
end;

{$HINTS OFF}
function btMinkowskiPenetrationDepthSolver.calcPenDepth(const simplexSolver: btSimplexSolverInterface; const convexA, convexB: btConvexShape; const transA, transB: btTransform; out v, pa, pb: btVector3; const debugDraw: btIDebugDraw; const stackAlloc: btStackAllocator): boolean;
begin
  result := false;
  abort;
end;
{$HINTS ON}

//begin
////
//  result := nil;
//  abort;
////  (void)stackAlloc;
////  (void)v;
////
////  bool check2d= convexA->isConvex2d() && convexB->isConvex2d();
////
////  struct btIntermediateResult : public btDiscreteCollisionDetectorInterface::Result
////  {
////
////        btIntermediateResult():m_hasResult(false)
////        {
////        }
////
////        btVector3 m_normalOnBInWorld;
////        btVector3 m_pointInWorld;
////        btScalar m_depth;
////        bool    m_hasResult;
////
////        virtual void setShapeIdentifiersA(int partId0,int index0)
////        {
////                (void)partId0;
////                (void)index0;
////        }
////        virtual void setShapeIdentifiersB(int partId1,int index1)
////        {
////                (void)partId1;
////                (void)index1;
////        }
////        void addContactPoint(const btVector3& normalOnBInWorld,const btVector3& pointInWorld,btScalar depth)
////        {
////                m_normalOnBInWorld = normalOnBInWorld;
////                m_pointInWorld = pointInWorld;
////                m_depth = depth;
////                m_hasResult = true;
////        }
////  };
////
////  //just take fixed number of orientation, and sample the penetration depth in that direction
////  btScalar minProj = btScalar(BT_LARGE_FLOAT);
////  btVector3 minNorm(btScalar(0.), btScalar(0.), btScalar(0.));
////  btVector3 minA,minB;
////  btVector3 seperatingAxisInA,seperatingAxisInB;
////  btVector3 pInA,qInB,pWorld,qWorld,w;
////
////#ifndef __SPU__
////#define USE_BATCHED_SUPPORT 1
////#endif
////#ifdef USE_BATCHED_SUPPORT
////
////  btVector3     supportVerticesABatch[NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
////  btVector3     supportVerticesBBatch[NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
////  btVector3     seperatingAxisInABatch[NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
////  btVector3     seperatingAxisInBBatch[NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
////  int i;
////
////  int numSampleDirections = NUM_UNITSPHERE_POINTS;
////
////  for (i=0;i<numSampleDirections;i++)
////  {
////        btVector3 norm = getPenetrationDirections()[i];
////        seperatingAxisInABatch[i] =  (-norm) * transA.GetBasisV^() ;
////        seperatingAxisInBBatch[i] =  norm   * transB.GetBasisV^() ;
////  }
////
////  {
////        int numPDA = convexA->getNumPreferredPenetrationDirections();
////        if (numPDA)
////        {
////                for (int i=0;i<numPDA;i++)
////                {
////                        btVector3 norm;
////                        convexA->getPreferredPenetrationDirection(i,norm);
////                        norm  = transA.GetBasisV^() * norm;
////                        getPenetrationDirections()[numSampleDirections] = norm;
////                        seperatingAxisInABatch[numSampleDirections] = (-norm) * transA.GetBasisV^();
////                        seperatingAxisInBBatch[numSampleDirections] = norm * transB.GetBasisV^();
////                        numSampleDirections++;
////                }
////        }
////  }
////
////  {
////        int numPDB = convexB->getNumPreferredPenetrationDirections();
////        if (numPDB)
////        {
////                for (int i=0;i<numPDB;i++)
////                {
////                        btVector3 norm;
////                        convexB->getPreferredPenetrationDirection(i,norm);
////                        norm  = transB.GetBasisV^() * norm;
////                        getPenetrationDirections()[numSampleDirections] = norm;
////                        seperatingAxisInABatch[numSampleDirections] = (-norm) * transA.GetBasisV^();
////                        seperatingAxisInBBatch[numSampleDirections] = norm * transB.GetBasisV^();
////                        numSampleDirections++;
////                }
////        }
////  }
////
////
////
////
////  convexA->batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInABatch,supportVerticesABatch,numSampleDirections);
////  convexB->batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInBBatch,supportVerticesBBatch,numSampleDirections);
////
////  for (i=0;i<numSampleDirections;i++)
////  {
////        btVector3 norm = getPenetrationDirections()[i];
////        if (check2d)
////        {
////                norm[2] = 0.f;
////        }
////        if (norm.length2()>0.01)
////        {
////
////                seperatingAxisInA = seperatingAxisInABatch[i];
////                seperatingAxisInB = seperatingAxisInBBatch[i];
////
////                pInA = supportVerticesABatch[i];
////                qInB = supportVerticesBBatch[i];
////
////                pWorld = transA(pInA);
////                qWorld = transB(qInB);
////                if (check2d)
////                {
////                        pWorld[2] = 0.f;
////                        qWorld[2] = 0.f;
////                }
////
////                w       = qWorld - pWorld;
////                btScalar delta = norm.dot(w);
////                //find smallest delta
////                if (delta < minProj)
////                {
////                        minProj = delta;
////                        minNorm = norm;
////                        minA = pWorld;
////                        minB = qWorld;
////                }
////        }
////  }
////#else
////
////  int numSampleDirections = NUM_UNITSPHERE_POINTS;
////
////#ifndef __SPU__
////  {
////        int numPDA = convexA->getNumPreferredPenetrationDirections();
////        if (numPDA)
////        {
////                for (int i=0;i<numPDA;i++)
////                {
////                        btVector3 norm;
////                        convexA->getPreferredPenetrationDirection(i,norm);
////                        norm  = transA.GetBasisV^() * norm;
////                        getPenetrationDirections()[numSampleDirections] = norm;
////                        numSampleDirections++;
////                }
////        }
////  }
////
////  {
////        int numPDB = convexB->getNumPreferredPenetrationDirections();
////        if (numPDB)
////        {
////                for (int i=0;i<numPDB;i++)
////                {
////                        btVector3 norm;
////                        convexB->getPreferredPenetrationDirection(i,norm);
////                        norm  = transB.GetBasisV^() * norm;
////                        getPenetrationDirections()[numSampleDirections] = norm;
////                        numSampleDirections++;
////                }
////        }
////  }
////#endif // __SPU__
////
////  for (int i=0;i<numSampleDirections;i++)
////  {
////        const btVector3& norm = getPenetrationDirections()[i];
////        seperatingAxisInA = (-norm)* transA.GetBasisV^();
////        seperatingAxisInB = norm* transB.GetBasisV^();
////        pInA = convexA->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
////        qInB = convexB->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);
////        pWorld = transA(pInA);
////        qWorld = transB(qInB);
////        w       = qWorld - pWorld;
////        btScalar delta = norm.dot(w);
////        //find smallest delta
////        if (delta < minProj)
////        {
////                minProj = delta;
////                minNorm = norm;
////                minA = pWorld;
////                minB = qWorld;
////        }
////  }
////#endif //USE_BATCHED_SUPPORT
////
////  //add the margins
////
////  minA += minNorm*convexA->getMarginNonVirtual();
////  minB -= minNorm*convexB->getMarginNonVirtual();
////  //no penetration
////  if (minProj < btScalar(0.))
////        return false;
////
////  btScalar extraSeparation = 0.5f;///scale dependent
////  minProj += extraSeparation+(convexA->getMarginNonVirtual() + convexB->getMarginNonVirtual());
////
////
////
////
////
//////#define DEBUG_DRAW 1
////#ifdef DEBUG_DRAW
////  if (debugDraw)
////  {
////        btVector3 color(0,1,0);
////        debugDraw->drawLine(minA,minB,color);
////        color = btVector3 (1,1,1);
////        btVector3 vec = minB-minA;
////        btScalar prj2 = minNorm.dot(vec);
////        debugDraw->drawLine(minA,minA+(minNorm*minProj),color);
////
////  }
////#endif //DEBUG_DRAW
////
////
////
////  btGjkPairDetector gjkdet(convexA,convexB,&simplexSolver,0);
////
////  btScalar offsetDist = minProj;
////  btVector3 offset = minNorm * offsetDist;
////
////
////
////  btGjkPairDetector::ClosestPointInput input;
////
////  btVector3 newOrg = transA.getOrigin() + offset;
////
////  btTransform displacedTrans = transA;
////  displacedTrans.setOrigin(newOrg);
////
////  input.m_transformA = displacedTrans;
////  input.m_transformB = transB;
////  input.m_maximumDistanceSquared = btScalar(BT_LARGE_FLOAT);//minProj;
////
////  btIntermediateResult res;
////  gjkdet.setCachedSeperatingAxis(-minNorm);
////  gjkdet.getClosestPoints(input,res,debugDraw);
////
////  btScalar correctedMinNorm = minProj - res.m_depth;
////
////
////  //the penetration depth is over-estimated, relax it
////  btScalar penetration_relaxation= btScalar(1.);
////  minNorm*=penetration_relaxation;
////
////
////  if (res.m_hasResult)
////  {
////
////        pa = res.m_pointInWorld - minNorm * correctedMinNorm;
////        pb = res.m_pointInWorld;
////        v = minNorm;
////
////#ifdef DEBUG_DRAW
////        if (debugDraw)
////        {
////                btVector3 color(1,0,0);
////                debugDraw->drawLine(pa,pb,color);
////        }
////#endif//DEBUG_DRAW
////
////
////  }
////  return res.m_hasResult;
//end;

{ btTriangleRaycastCallback }

constructor btTriangleRaycastCallback.create(const from, too: btVector3; const flags: cardinal);
begin
  m_from        := from;
  m_to          := too;
  m_flags       := flags;
  m_hitFraction := 1;
end;

procedure btTriangleRaycastCallback.processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer);
var vert0,vert1,vert2:PbtVector3;
    cp2,cp1,v2p,v0p,v1p,cp0,v10,v20,triangleNormal,point:btVector3;
    edge_tolerance,distance,proj_length,dist,dist_a,dist_b : btScalar;
begin
  vert0 := @triangle[0];
  vert1 := @triangle[1];
  vert2 := @triangle[2];

  v10   := vert1^ - vert0^;
  v20   := vert2^ - vert0^;

  triangleNormal := v10.cross(v20);
  dist   := vert0^.dot(triangleNormal);
  dist_a := triangleNormal.dot(m_from) ;
  dist_a -= dist;
  dist_b := triangleNormal.dot(m_to);
  dist_b -= dist;

  if dist_a * dist_b >= 0  then exit;
  //@BP Mod - Backface filtering
  if ((m_flags AND ord(kF_FilterBackfaces)) <> 0) AND (dist_a > 0) then begin
    // Backface, skip check
    exit;
  end;

  proj_length := dist_a-dist_b;
  distance    := (dist_a)/(proj_length);
  // Now we have the intersection point on the plane, we'll see if it's inside the triangle
  // Add an epsilon as a tolerance for the raycast,
  // in case the ray hits exacly on the edge of the triangle.
  // It must be scaled for the triangle size.
  if distance < m_hitFraction then begin
    edge_tolerance := triangleNormal.length2;
    edge_tolerance *= btScalar(-0.0001);
    point.setInterpolate3( m_from, m_to, distance);
    v0p := vert0^ - point;
    v1p := vert1^ - point;
    cp0 := v0p.cross( v1p );
    if cp0.dot(triangleNormal) >=edge_tolerance then begin
      v2p := vert2^ -  point;
      cp1 := v1p.cross( v2p);
      if cp1.dot(triangleNormal) >=edge_tolerance then begin
        cp2 := v2p.cross(v0p);
        if  cp2.dot(triangleNormal) >=edge_tolerance then begin
          //@BP Mod
          // Triangle normal isn't normalized
         triangleNormal.normalize;
         //@BP Mod - Allow for unflipped normal when raycasting against backfaces
          if (m_flags AND ord(kF_KeepUnflippedNormal) <> 0) or (dist_a <= 0) then begin
            m_hitFraction := reportHit(-triangleNormal,distance,partId,triangleIndex);
          end else begin
            m_hitFraction := reportHit(triangleNormal,distance,partId,triangleIndex);
          end;
        end;
      end;
    end;
  end;
end;

{ btTriangleConvexcastCallback }

constructor btTriangleConvexcastCallback.create(const convexShape: btConvexShape; const convexShapeFrom, convexShapeTo, triangleToWorld: btTransform; const triangleCollisionMargin: btScalar);
begin
  m_convexShape     := convexShape;
  m_convexShapeFrom := convexShapeFrom;
  m_convexShapeTo   := convexShapeTo;
  m_triangleToWorld := triangleToWorld;
  m_hitFraction     := 1;
  m_triangleCollisionMargin := triangleCollisionMargin;
end;

procedure btTriangleConvexcastCallback.processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer);
var triangleShape : btTriangleShape;
    simplexSolver : btVoronoiSimplexSolver;
    convexCaster  : btSubsimplexConvexCast;
    castResult    : btConvexCastResult;
begin
  triangleShape := btTriangleShape.Create(triangle[0], triangle[1], triangle[2]);
  triangleShape.setMargin(m_triangleCollisionMargin);
  simplexSolver   := btVoronoiSimplexSolver.Create;
  convexCaster := btSubsimplexConvexCast.create(m_convexShape, triangleShape, simplexSolver);
  simplexSolver.free;
  castResult.m_fraction := 1;
  if convexCaster.calcTimeOfImpact(m_convexShapeFrom,m_convexShapeTo,m_triangleToWorld, m_triangleToWorld, castResult) then begin
    if castResult.m_normal.length2 > btScalar(0.0001) then begin //add hit
      if castResult.m_fraction < m_hitFraction then begin
        castResult.m_normal.normalize;
        reportHit (castResult.m_normal,castResult.m_hitPoint,castResult.m_fraction,partId,triangleIndex);
      end;
    end;
  end;
  convexCaster.Free;
  triangleShape.free;
end;

{ btContinuousConvexCollision }

constructor btContinuousConvexCollision.create(const shapeA, shapeB: btConvexShape; const simplexSolver: btSimplexSolverInterface; const penetrationDepthSolver: btConvexPenetrationDepthSolver);
begin
  m_simplexSolver          := simplexSolver;
  m_penetrationDepthSolver := penetrationDepthSolver;
  m_convexA                := shapeA;
  m_convexB                := shapeB;
end;

function btContinuousConvexCollision.calcTimeOfImpact(const fromA, toA, fromB, toB: btTransform; var lresult: btConvexCastResult): boolean;
var   c,n,v,relLinVel,linVelA,angVelA,linVelB,angVelB : btVector3;
      lastlambda,lambda,radius,relLinVelocLength,
      maxAngularProjectedVelocity,
      dist,boundingRadiusA,boundingRadiusB,
      projectedLinearVelocity,dLambda               : btScalar;
      maxIter,numIter                               : integer;
      hasResult                                     : boolean;
      raySphere                                     : btSphereShape;
      interpolatedTransA,interpolatedTransB,
      //relativeTrans,
      identityTrans                                 : btTransform;
      pointCollector1,pointCollector                : btPointCollector;
      gjk                                           : btGjkPairDetector;
      input                                         : btDCDI_ClosestPointInput;

begin
  m_simplexSolver.reset;
  /// compute linear and angular velocity for this interval, to interpolate
  btTransformUtil.calculateVelocity(fromA,toA,1,linVelA,angVelA);
  btTransformUtil.calculateVelocity(fromB,toB,1,linVelB,angVelB);
  boundingRadiusA := m_convexA.getAngularMotionDisc;
  boundingRadiusB := m_convexB.getAngularMotionDisc;

  maxAngularProjectedVelocity := angVelA.length * boundingRadiusA + angVelB.length * boundingRadiusB;
  relLinVel                   := (linVelB-linVelA);
  relLinVelocLength           := (linVelB-linVelA).length();

  if (relLinVelocLength+maxAngularProjectedVelocity) = 0 then begin
    exit(false);
  end;
  radius  := 0.001;
  lambda  := 0;
  maxIter := MAX_ITERATIONS;
  v.init(1,0,0);
  n.InitSame(0);
  hasResult  := false;
  lastlambda := lambda;
  //btScalar epsilon = btScalar(0.001);
  numIter    := 0;
  //first solution, using GJK
  identityTrans.setIdentity;
  raySphere := btSphereShape.Create(0);
  raySphere.setMargin(0);
try // FOS
  gjk.init(m_convexA,m_convexB,m_convexA.getShapeType,m_convexB.getShapeType,m_convexA.getMargin,m_convexB.getMargin,m_simplexSolver,m_penetrationDepthSolver);

  input.init;
  input.m_transformA := fromA;
  input.m_transformB := fromB;
  pointCollector1:=btPointCollector.Create; //TRY

  gjk.getClosestPoints(input,btDCDI_Result(pointCollector1),nil);

  hasResult          := pointCollector1.m_hasResult;
  c                  := pointCollector1.m_pointInWorld;
  if hasResult then begin
    dist := pointCollector1.m_distance;
    n    := pointCollector1.m_normalOnBInWorld;
    projectedLinearVelocity := relLinVel.dot(n);
    //not close enough
    while dist > radius do begin
      if assigned(lresult.m_debugDrawer) then begin
        lresult.m_debugDrawer.drawSphere(c,0.2,btVector3.InitSameS(1));
      end;
      inc(numIter);
      if numIter > maxIter then begin
        exit(false); //todo: report a failure
      end;
      dLambda                 := 0;
      projectedLinearVelocity := relLinVel.dot(n);
      //calculate safe moving fraction from distance / (linear+rotational velocity)
      //btScalar clippedDist  = GEN_min(angularConservativeRadius,dist);
      //btScalar clippedDist  = dist;
      //don't report time of impact for motion away from the contact normal (or causes minor penetration)
      if ((projectedLinearVelocity+ maxAngularProjectedVelocity)<=SIMD_EPSILON) then begin
        exit(false);
      end;
      dLambda := dist / (projectedLinearVelocity+ maxAngularProjectedVelocity);
      lambda  := lambda + dLambda;
      if lambda > 1 then begin
        exit(false);
      end;
      if lambda < 0 then begin
        exit(false);
      end;
      //todo: next check with relative epsilon
      if (lambda <= lastLambda) then begin
        exit(false);
      end;
      lastLambda := lambda;
      //interpolate to next lambda
      btTransformUtil.integrateTransform(fromA,linVelA,angVelA,lambda,interpolatedTransA);
      btTransformUtil.integrateTransform(fromB,linVelB,angVelB,lambda,interpolatedTransB);
//      relativeTrans := interpolatedTransB.inverseTimes(interpolatedTransA);
      if assigned(lresult.m_debugDrawer) then begin
        lresult.m_debugDrawer.drawSphere(interpolatedTransA.getOriginV^,0.2,btVector3.InitS(1,0,0));
      end;
      lresult.DebugDraw(lambda);
      gjk.init(m_convexA,m_convexB,m_simplexSolver,m_penetrationDepthSolver);
      input.m_transformA := interpolatedTransA;
      input.m_transformB := interpolatedTransB;
      pointCollector:=btPointCollector.Create;
      gjk.getClosestPoints(input,btDCDI_Result(pointCollector),nil);
      if (pointCollector.m_hasResult) then begin
        if pointCollector.m_distance < 0 then begin
          //degenerate ?!
          lresult.m_fraction := lastLambda;
          n                  := pointCollector.m_normalOnBInWorld;
          lresult.m_normal   := n;//.setValue(1,1,1);// = n;
          lresult.m_hitPoint := pointCollector.m_pointInWorld;
          exit(true);
        end;
        c    := pointCollector.m_pointInWorld;
        n    := pointCollector.m_normalOnBInWorld;
        dist := pointCollector.m_distance;
        pointCollector.Free;
      end else begin
        //??
        pointCollector.Free;
        exit(false);
      end;
    end;
    if ((projectedLinearVelocity+ maxAngularProjectedVelocity)<=lresult.m_allowedPenetration) then begin //SIMD_EPSILON)
     exit(false);
    end;
    lresult.m_fraction := lambda;
    lresult.m_normal   := n;
    lresult.m_hitPoint := c;
    exit(true);
  end;
  exit(false);
finally
  pointCollector1.free;
  raySphere.free;
end;
 //{** todo:
 //       //if movement away from normal, discard result
 //       btVector3 move = transBLocalTo.getOrigin() - transBLocalFrom.getOrigin();
 //       if (result.m_fraction < btScalar(1.))
 //       {
 //           if (move.dot(result.m_normal) <= btScalar(0.))
 //           {
 //           }
 //       }
 //**}
end;

initialization
  gContactBreakingThreshold:=0.02;
  gContactDestroyedCallback:=nil;
  gContactProcessedCallback:=nil;

end.
