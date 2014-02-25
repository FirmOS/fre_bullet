unit btDispatch;

// Bullet Source Code Translation / DI Helmut Hartl / FirmOS Business Solutions GmbH
// Original Code Licence
{*

/*
 * Box-Box collision detection re-distributed under the ZLib license with permission from Russell L. Smith
 * Original version is from Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org

Copyright (c) 2003-2009 Erwin Coumans  http://bullet.googlecode.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

//
// btAxisSweep3
// Copyright (c) 2006 Simon Hobbs
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.

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


// Files:

// +btManifoldResult.h
// +btManifoldResult.cpp
// +btCollisionConfiguration.h
// +btCollisionObject.h
// +btCollisionObject.cpp
// +btCollisionCreateFunc.h
// +btDispatcher.h
// +btDispatcher.cpp            (broadphase)
// +btOverlappingPairCallback.h (broadphase)
// +btOverlappingPairCache.h    (broadphase)
// +btOverlappingPairCache.cpp  (broadphase)
// +btCollisionDispatcher.h
// +btCollisionDispatcher.cpp
// +btCollisionAlgorithm.h
// +btCollisionAlgorithm.cpp
// +btBroadphaseInterface.h
// +btSimpleBroadPhase.h
// +btSimpleBroadPhase.cpp
// +btDefaultCollisionConfiguration.h
// +btDefaultCollisionConfiguration.cpp
// +btEmptyCollisionAlgorithm.h
// +btEmptyCollisionAlgorithm.cpp
// +btActivatingCollisionAlgorithm.h   (BULLET NOT DONE)
// +btActivatingCollisionAlgorithm.cpp (BULLET NOT DONE)
// +btBoxBoxCollisionAlgorithm.h
// +btBoxBoxCollisionAlgorithm.cpp
// +btBoxBoxDetector.h
// +btBoxBoxDetector.cpp
// +btConvexConvexAlgorithm.h
// +btConvexConvexAlgorithm.cpp
// +btConvexConcaveCollisionAlgorithm.h
// +btConvexConcaveCollisionAlgorithm.cpp
// +btCompoundCollisionAlgorithm.h
// +btCompoundCollisionAlgorithm.cpp
// +btConvexPlaneCollisionAlgorithm.h
// +btConvexPlaneCollisionAlgorithm.cpp
// +btSphereSphereCollisionAlgorithm.h
// +btSphereSphereCollisionAlgorithm.cpp
// +btSphereTriangleCollisionAlgorithm.h
// +btSphereTriangleCollisionAlgorithm.cpp
// +SphereTriangleDetector.h
// +SphereTriangleDetector.cpp
// +btDbvtBroadPhase.h
// +btDbvtBroadPhase.cpp   w/o Benchmark
// +btCollisionWorld.h
// +btCollisionWorld.cpp
// btAxisSweep3.h
// btAxisSweep3.cpp

//Delayed
// btMultiSapBroadphase.h
// btMultiSapBroadphase.cpp

interface

{$i fos_bullet.inc}

uses
  Sysutils,btLinearMath,btBroadphase,btNarrowphase,btCollisionShapes,FOS_AlignedArray;

var fosdebug_GLOBAL_STEPPING : boolean=false;
    fosdebug_gcount          : integer=0;
    fosdebug_clock           : btClock;

type

     btCollisionObject=class;
     btContactAddedCallback=function (const cp:PbtOManifoldPoint;const colObj0:btCollisionObject;const partId0,index0:integer;const colObj1:btCollisionObject;const partId1,index1:integer):boolean;


var   gContactAddedCallback:btContactAddedCallback;
var   gAddedPairs,gOverlappingPairs,gRemovePairs,gFindPairs,gNumManifold:integer;
const BT_NULL_PAIR:integer = integer($ffffffff);
      function btCollisionObjectCompare(const a,b:btCollisionObject):nativeint;

type

  btDispatchFunc=(DISPATCH_DISCRETE = 1, DISPATCH_CONTINUOUS);

  { btDispatcherInfo }
  btDispatcherInfo=object
     m_timeStep                            : btScalar;
     m_stepCount                           : integer;
     m_dispatchFunc                        : btDispatchFunc;
     m_timeOfImpact                        : btScalar;
     m_useContinuous                       : boolean;
     m_debugDraw                           : btIDebugDraw;
     m_enableSatConvex                     : boolean;
     m_enableSPU                           : boolean;
     m_useEpa                              : boolean;
     m_allowedCcdPenetration               : btScalar;
     m_useConvexConservativeDistanceUtil   : boolean;
     m_convexConservativeDistanceThreshold : btScalar;
     m_stackAllocator                      : btStackAllocator;
     procedure Init;
  end;
  PbtDispatcherInfo = ^btDispatcherInfo;

  btOverlappingPaircache = class;
  btCollisionAlgorithm   = class;
  btCollisionAlgorithmClass   = class of btCollisionAlgorithm;

  ///The btDispatcher interface class can be used in combination with broadphase to dispatch calculations for overlapping pairs.
  ///For example for pairwise collision detection, calculating contact points stored in btPersistentManifold or user callbacks (game logic).
  btDispatcher=class
  public
     function   findAlgorithm   (const body0,body1:btCollisionObject;const sharedManifold:btPersistentManifold=nil):btCollisionAlgorithm;virtual;abstract;
     function   getNewManifold  (const body0,body1:btCollisionObject):btPersistentManifold;virtual;abstract;
     procedure  releaseManifold (const manifold:btPersistentManifold);virtual;abstract;
     procedure  clearManifold   (const manifold:btPersistentManifold);virtual;abstract;
     function   needsCollision  (const body0,body1:btCollisionObject):Boolean;virtual;abstract;
     function   needsResponse   (const body0,body1:btCollisionObject):Boolean;virtual;abstract;
     procedure  dispatchAllCollisionPairs  (const pairCache:btOverlappingPairCache;const dispatchInfo:btDispatcherInfo;const dispatcher:btDispatcher);virtual;abstract;
     function   getNumManifolds            :integer;virtual;abstract;
     function   getManifoldByIndexInternal (const index:integer):btPersistentManifold;virtual;abstract;
     function   getInternalManifoldPointer :btManifoldArray;virtual;
     function   allocateCollisionAlgorithm (const alg:btCollisionAlgorithmClass):btCollisionAlgorithm;virtual;abstract;
     procedure  freeCollisionAlgorithm     (const alg:btCollisionAlgorithm);virtual;abstract;
  end;

 btCollisionObjectArray=specialize FOS_GenericAlignedArray<btCollisionObject>;

/// btCollisionObject can be used to manage collision detection objects.
/// btCollisionObject maintains all information that is needed for a collision detection: Shape, Transform and AABB proxy.
/// They can be added to the btCollisionWorld.

  { btCollisionObject }
  btCollisionFlags=(
          CF_STATIC_OBJECT= 1,
          CF_KINEMATIC_OBJECT= 2,
          CF_NO_CONTACT_RESPONSE = 4,
          CF_CUSTOM_MATERIAL_CALLBACK = 8,//this allows per-triangle material (friction/restitution)
          CF_CHARACTER_OBJECT = 16,
          CF_DISABLE_VISUALIZE_OBJECT = 32, //disable debug drawing
          CF_DISABLE_SPU_COLLISION_PROCESSING = 64//disable parallel/SPU processing
  );
  btCollisionFlagSet=set of btCollisionFlags;
  btCollisionObjectTypes=
  (
          CO_COLLISION_OBJECT =1,
          CO_RIGID_BODY,
          ///CO_GHOST_OBJECT keeps track of all objects overlapping its AABB and that pass its collision filter
          ///It is useful for collision sensors, explosion objects, character controller etc.
          CO_GHOST_OBJECT,
          CO_SOFT_BODY,
          CO_HF_FLUID
  );

  //island management, m_activationState1
  btActivationState=(btas_NONE,btas_ACTIVE_TAG,btas_ISLAND_SLEEPING,btas_WANTS_DEACTIVATION,btas_DISABLE_DEACTIVATION,btas_DISABLE_SIMULATION);


  btCollisionObject=class
  protected
    m_worldTransform : btTransform;
    ///m_interpolationWorldTransform is used for CCD and interpolation
    ///it can be either previous or future (predicted) transform
    m_interpolationWorldTransform : btTransform;
   //those two are experimental: just added for bullet time effect, so you can still apply impulses (directly modifying velocities)
    //without destroying the continuous interpolated motion (which uses this interpolation velocities)
    m_interpolationLinearVelocity,
    m_interpolationAngularVelocity,
    m_anisotropicFriction            : btVector3;
    m_hasAnisotropicFriction         : boolean;
    m_contactProcessingThreshold     : btScalar;
    m_broadphaseHandle               : btBroadphaseProxy;
    m_collisionShape                 : btCollisionShape;
  ///m_extensionPointer is used by some internal low-level Bullet extensions.
    m_extensionPointer               : Pointer;
  ///m_rootCollisionShape is temporarily used to store the original collision shape
  ///The m_collisionShape might be temporarily replaced by a child collision shape during collision detection purposes
  ///If it is NULL, the m_collisionShape is not temporarily replaced.
    m_rootCollisionShape             : btCollisionShape;
    m_collisionFlags                 : btCollisionFlagSet;
    m_islandTag1,
    m_companionId                    : integer;
    m_activationState1               : btActivationState;
    m_deactivationTime               : btScalar;
    m_friction                       : btScalar;
    m_restitution                    : btScalar;
  ///m_internalType is reserved to distinguish Bullet's btCollisionObject, btRigidBody, btSoftBody, btGhostObject etc.
  ///do not assign your own m_internalType unless you write a new dynamics object class.
    m_internalType                   : btCollisionObjectTypes;
   ///users can point to their objects, m_userPointer is not used by Bullet, see setUserPointer/getUserPointer
    m_userObjectPointer              : Pointer;
  ///time of impact calculation
    m_hitFraction                    : btScalar;
  ///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
    m_ccdSweptSphereRadius           : btScalar;
  /// Don't do continuous collision detection if the motion (in one step) is less then m_ccdMotionThreshold
    m_ccdMotionThreshold             : btScalar;
  /// If some object should have elaborate collision filtering by sub-classes
    m_checkCollideWith               : boolean;
    function                          checkCollideWithOverride(const x:btCollisionObject):boolean;virtual;
  public
    function    mergesSimulationIslands : Boolean  ;FOS_INLINE;
    function    getAnisotropicFriction  : btVector3;
    procedure   setAnisotropicFriction  (const anisotropicFriction:btVector3);
    function    hasAnisotropicFriction  : boolean  ;
  ///the constraint solver can discard solving contacts, if the distance is above this threshold. 0 by default.
  ///Note that using contacts with positive distance can improve stability. It increases, however, the chance of colliding with degerate contacts, such as 'interior' triangle edges
    procedure   setContactProcessingThreshold(const contactProcessingThreshold:btScalar);
    function    getContactProcessingThreshold:btScalar;
    function    isStaticObject            : boolean;FOS_INLINE;
    function    isKinematicObject         : boolean;FOS_INLINE;
    function    isStaticOrKinematicObject : boolean;FOS_INLINE;
    function    hasContactResponse        : boolean;FOS_INLINE;
    constructor Create                    ;
    destructor  Destroy; override;
    procedure   setCollisionShape         (const collisionShape:btCollisionShape);virtual;
    function    getCollisionShape         :btCollisionShape;FOS_INLINE;
    function    getRootCollisionShape     :btCollisionShape;FOS_INLINE;
  ///Avoid using this internal API call
  ///internalSetTemporaryCollisionShape is used to temporary replace the actual collision shape by a child collision shape.
    procedure   internalSetTemporaryCollisionShape    (const collisionShape:btCollisionShape);FOS_INLINE;
  ///Avoid using this internal API call, the extension pointer is used by some Bullet extensions.
  ///If you need to store your own user pointer, use 'setUserPointer/getUserPointer' instead.
    function    internalGetExtensionPointer:Pointer;FOS_INLINE;
  ///Avoid using this internal API call, the extension pointer is used by some Bullet extensions
  ///If you need to store your own user pointer, use 'setUserPointer/getUserPointer' instead.
    procedure   internalSetExtensionPointer      (const pointer:Pointer);
    function    getActivationState               : btActivationState;FOS_INLINE;
    procedure   setActivationState               (const newState:btActivationState);
    procedure   setDeactivationTime              (const time:btScalar);
    function    getDeactivationTime              :btScalar;
    procedure   forceActivationState             (const newState:btActivationState);
    procedure   activate                         (const forceActivation:boolean=false);
    function    isActive                         :boolean;FOS_INLINE;
    procedure   setRestitution                   (const rest:btScalar);
    function    getRestitution                   :btScalar;
    procedure   setFriction                      (const fric:btScalar);
    function    getFriction                      :btScalar;
    ///reserved for Bullet internal usage
    function    getInternalType                  :btCollisionObjectTypes;
    function    getWorldTransformP               :PbtTransform;
    procedure   setWorldTransform                (const worldTrans:btTransform);
    function    getBroadphaseHandle              :btBroadphaseProxy;FOS_INLINE;
    procedure   setBroadphaseHandle              (const handle:btBroadphaseProxy);
    function    getInterpolationWorldTransformP  :PbtTransform;
    procedure   setInterpolationWorldTransform   (const worldTrans:btTransform);
    procedure   setInterpolationLinearVelocity   (const linvel:btVector3);
    procedure   setInterpolationAngularVelocity  (const angvel:btVector3);
    function    getInterpolationLinearVelocityP  :PbtVector3;
    function    getInterpolationAngularVelocityP :PbtVector3;
    function    getIslandTag                     : integer;FOS_INLINE;
    procedure   setIslandTag                     (const tag:integer);
    function    getCompanionId                   : integer;FOS_INLINE;
    procedure   setCompanionId                   (const id:integer);
    function    getHitFraction                  :btScalar;FOS_INLINE;
    procedure   setHitFraction                   (const hitFraction:btScalar);
    function    getCollisionFlags                :btCollisionFlagSet;FOS_INLINE;
    procedure   setCollisionFlags                (const flags:btCollisionFlagSet);
  ///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
    function    getCcdSweptSphereRadius          :btScalar;FOS_INLINE;
    procedure   setCcdSweptSphereRadius          (const radius:btScalar);
    function    getCcdMotionThreshold           :btScalar;FOS_INLINE;
    function    getCcdSquareMotionThreshold     :btScalar;FOS_INLINE;
    procedure   setCcdMotionThreshold           (const ccdMotionThreshold:btScalar);
  ///users can point to their objects, userPointer is not used by Bullet
    function    getUserPointer:Pointer;FOS_INLINE;
    procedure   setUserPointer(const userPointer:Pointer);
    function    checkCollideWith(const co:btCollisionObject):boolean;FOS_INLINE;
  end;

///btManifoldResult is a helper class to manage  contact results.
  btManifoldResult=class(btDCDI_Result)
  protected
    m_manifoldPtr   : btPersistentManifold;
      //we need this for compounds
    m_rootTransA    : btTransform;
    m_rootTransB    : btTransform;
    m_body0,m_body1 : btCollisionObject;
    m_partId0,
    m_partId1,
    m_index0,
    m_index1        : integer;
  public
    constructor Create                    (const body0,body1 : btCollisionObject);
    procedure   setPersistentManifold     (manifoldPtr       : btPersistentManifold);
    function    getPersistentManifold     :btPersistentManifold;
    procedure   setShapeIdentifiersA      (const partId0, index0: integer); override;
    procedure   setShapeIdentifiersB      (const partId1, index1: integer); override;
    procedure   addContactPoint           (const normalOnBInWorld, pointInWorld: btVector3;const depth: btScalar); override;
    procedure   refreshContactPoints      ;
    function    getBody0Internal          :btCollisionObject;FOS_INLINE;
    function    getBody1Internal          :btCollisionObject;FOS_INLINE;
  end;

  ///The btBroadphasePair class contains a pair of aabb-overlapping objects.
  ///A btDispatcher can search a btCollisionAlgorithm that performs exact/narrowphase collision detection on the actual collision shapes.

  { btBroadphasePair }
  TFOS_internal=record
    case byte of
      0: ( m_internalInfo1    : Pointer);
      1: ( m_internalTmpValue : integer);
  end;
  PbtBroadphasePair=^btBroadphasePair;
  btBroadphasePair=object
    m_pProxy0   : btBroadphaseProxy;
    m_pProxy1   : btBroadphaseProxy;
    m_algorithm : btCollisionAlgorithm;
    int         : TFOS_internal;
    procedure   Init;
    procedure   Init(const other:PbtBroadphasePair);overload;
    procedure   Init(const proxy0,proxy1:btBroadphaseProxy);overload;
  end;
/////The btOverlappingPairCallback class is an additional optional broadphase user callback for adding/removing overlapping pairs, similar interface to btOverlappingPairCache.
  btOverlappingPairCallback=class
  public
    function   addOverlappingPair                    (const proxy0,proxy1:btBroadphaseProxy):PbtBroadphasePair;virtual;abstract;
    function   removeOverlappingPair                 (proxy0,proxy1:btBroadphaseProxy;const dispatcher:btDispatcher):pointer;virtual;abstract;
    procedure  removeOverlappingPairsContainingProxy (const proxy0:btBroadphaseProxy;const dispatcher:btDispatcher);virtual;abstract;
  end;

  { btOverlapCallback }

  btOverlapCallback=class
    //return true for deletion of the pair
    function processOverlap(const pair:PbtBroadphasePair):boolean;virtual;abstract;
  end;

  btOverlapFilterCallback=class
    // return true when pairs need collision
    function needBroadphaseCollision(const proxy0,proxy1:btBroadphaseProxy):boolean;virtual;abstract;
  end;

  btBroadphasePairArray=specialize FOS_GenericAlignedArray<btBroadphasePair>;
 // PbtBroadphasePairArray=^btBroadphasePairArray;

/////The btOverlappingPairCache provides an interface for overlapping pair management (add, remove, storage), used by the btBroadphaseInterface broadphases.
/////The btHashedOverlappingPairCache and btSortedOverlappingPairCache classes are two implementations.
  btOverlappingPairCache=class(btOverlappingPairCallback)
    public
    //  function   getOverlappingPairArrayPtr   :PbtBroadphasePair;virtual;abstract;
      function   getOverlappingPairArray      :btBroadphasePairArray;virtual;abstract;
      procedure  cleanOverlappingPair         (const pair:PbtBroadphasePair;const dispatcher:btDispatcher);virtual;abstract;
      function   getNumOverlappingPairs       :integer;virtual;abstract;
      procedure  cleanProxyFromPairs          (const proxy:btBroadphaseProxy;const dispatcher:btDispatcher);virtual;abstract;
      procedure  setOverlapFilterCallback     (const callback:btOverlapFilterCallback);virtual;abstract;
      procedure  processAllOverlappingPairs   (const ocb:btOverlapCallback;const dispatcher:btDispatcher);virtual;abstract;
      function   findPair                     (proxy0,proxy1: btBroadphaseProxy):PbtBroadphasePair;virtual;abstract;
      function   hasDeferredRemoval           :boolean;virtual;abstract;
      procedure  setInternalGhostPairCallback (const ghostPairCallback:btOverlappingPairCallback);virtual;abstract;
      procedure  sortOverlappingPairs         (const dispatcher:btDispatcher);virtual;abstract;
  end;
/// Hash-space based Pair Cache, thanks to Erin Catto, Box2D, http://www.box2d.org, and Pierre Terdiman, Codercorner, http://codercorner.com

  { btHashedOverlappingPairCache }
 // btAlignedIntArray=specialize FOS_GenericAlignedArray<integer>;

  btHashedOverlappingPairCache=class(btOverlappingPairCache)
  private
    m_overlappingPairArray   : btBroadphasePairArray;
    m_overlapFilterCallback  : btOverlapFilterCallback;
    m_blockedForChanges      : boolean;
    function                 internalAddPair    (proxy0,proxy1:btBroadphaseProxy):PbtBroadphasePair;
    procedure                growTables         ;
    function                 equalsPair         (const pair:btBroadphasePair;const proxyId1,proxyId2:integer):boolean;FOS_INLINE;
    function                 getHash            (const proxyId1,proxyId2:integer):integer;
    function                 internalFindPair   (const proxy0,proxy1:btBroadphaseProxy;const hash:integer):integer;
  protected
    m_hashTable              : btFOSAlignedIntegers;
    m_next                   : btFOSAlignedIntegers;
    m_ghostPairCallback      : btOverlappingPairCallback;
  public
    constructor Create                                ;
    destructor  Destroy                               ; override;
    function    hasDeferredRemoval : boolean          ; override;
    procedure   setInternalGhostPairCallback          (const ghostPairCallback: btOverlappingPairCallback); override;
    procedure   sortOverlappingPairs                  (const dispatcher: btDispatcher); override;
    procedure   removeOverlappingPairsContainingProxy (const proxy0: btBroadphaseProxy; const dispatcher: btDispatcher); override;
    function    removeOverlappingPair                 (proxy0, proxy1: btBroadphaseProxy; const dispatcher: btDispatcher):pointer; override;
    function    needsBroadphaseCollision              (const proxy0, proxy1: btBroadphaseProxy):boolean;FOS_INLINE;
    // Add a pair and return the new pair. If the pair already exists,
    // no new pair is created and the old one is returned.
    function    addOverlappingPair                    (const proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair; override;
    procedure   cleanProxyFromPairs                   (const proxy: btBroadphaseProxy;const dispatcher: btDispatcher); override;
    procedure   processAllOverlappingPairs            (const ocb: btOverlapCallback; const dispatcher: btDispatcher); override;
//    function    getOverlappingPairArrayPtr            : PbtBroadphasePair; override;
    function    getOverlappingPairArray               : btBroadphasePairArray; override;
    procedure   cleanOverlappingPair                  (const pair: PbtBroadphasePair; const dispatcher: btDispatcher); override;
    function    findPair                              (proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair; override;
    function    GetCount                              :integer;
    function    getOverlapFilterCallback              :btOverlapFilterCallback;
    procedure   setOverlapFilterCallback              (const oc:btOverlapFilterCallback);override;
    function    getNumOverlappingPairs                : integer; override;
  end;

///btSortedOverlappingPairCache maintains the objects with overlapping AABB
///Typically managed by the Broadphase, Axis3Sweep or btSimpleBroadphase

   { btSortedOverlappingPairCache }

   btSortedOverlappingPairCache=class(btOverlappingPairCache)
   protected
    //avoid brute-force finding all the time
     m_overlappingPairArray   : btBroadphasePairArray;
     //during the dispatch, check that user doesn't destroy/create proxy
     m_blockedForChanges      : boolean;
    ///by default, do the removal during the pair traversal
     m_hasDeferredRemoval     :  boolean;
    //if set, use the callback instead of the built in filter in needBroadphaseCollision
     m_overlapFilterCallback  : btOverlapFilterCallback;
     m_ghostPairCallback      : btOverlappingPairCallback;
   public
     constructor Create                                ;
     destructor  Destroy                               ;override;
     procedure   processAllOverlappingPairs            (const ocb: btOverlapCallback; const dispatcher: btDispatcher); override;
     function    removeOverlappingPair                 (proxy0, proxy1: btBroadphaseProxy; const dispatcher: btDispatcher):pointer; override;
     procedure   cleanOverlappingPair                  (const pair: PbtBroadphasePair; const dispatcher: btDispatcher); override;
     function    addOverlappingPair                    (const proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair; override;
     function    findPair                              (proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair; override;
     procedure   cleanProxyFromPairs                   (const proxy:  btBroadphaseProxy;const dispatcher: btDispatcher); override;
     procedure   removeOverlappingPairsContainingProxy (const proxy0: btBroadphaseProxy; const dispatcher: btDispatcher); override;
     function    needsBroadphaseCollision              (const proxy0, proxy1: btBroadphaseProxy):boolean;FOS_INLINE;
     function    getOverlappingPairArray               : btBroadphasePairArray; override;
     function    getNumOverlappingPairs                : integer; override;
     function    getOverlapFilterCallback              :btOverlapFilterCallback;
     procedure   setOverlapFilterCallback              (const oc:btOverlapFilterCallback);override;
     function    hasDeferredRemoval                    : boolean; override;
     procedure   setInternalGhostPairCallback          (const ghostPairCallback: btOverlappingPairCallback); override;
     procedure   sortOverlappingPairs                  (const dispatcher: btDispatcher); override;
   end;

///btNullPairCache skips add/removal of overlapping pairs. Userful for benchmarking and unit testing.

   { btNullPairCache }

  btNullPairCache=class(btOverlappingPairCache)
     m_overlappingPairArray   : btBroadphasePairArray;
  public
    constructor create                                ;
    destructor  destroy                               ;override;
    //function    getOverlappingPairArrayPtr            : PbtBroadphasePair; override;
    function    getOverlappingPairArray               : btBroadphasePairArray; override;
    procedure   cleanOverlappingPair                  (const pair: PbtBroadphasePair; const dispatcher: btDispatcher); override;
    function    getNumOverlappingPairs                : integer; override;
    procedure   cleanProxyFromPairs                   (const proxy: btBroadphaseProxy;const dispatcher: btDispatcher); override;
    procedure   setOverlapFilterCallback              (const oc:btOverlapFilterCallback);override;
    procedure   processAllOverlappingPairs            (const ocb: btOverlapCallback; const dispatcher: btDispatcher); override;
    function    findPair                              ( proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair; override;
    function    hasDeferredRemoval                    : boolean; override;
    procedure   setInternalGhostPairCallback          (const ghostPairCallback: btOverlappingPairCallback); override;
    function    removeOverlappingPair                 (proxy0, proxy1: btBroadphaseProxy; const dispatcher: btDispatcher):Pointer; override;
    function    addOverlappingPair                    (const proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair; override;
    procedure   removeOverlappingPairsContainingProxy (const proxy0: btBroadphaseProxy; const dispatcher: btDispatcher); override;
    procedure   sortOverlappingPairs                  (const dispatcher:btDispatcher);override;
  end;

   { btCollisionAlgorithmConstructionInfo }
   btCollisionAlgorithmConstructionInfo=object
     m_dispatcher1 : btDispatcher;
     m_manifold    : btPersistentManifold;
    //IMPL NOT FOUND function      getDispatcherId:integer;
   end;

   ///btCollisionAlgorithm is an collision interface that is compatible with the Broadphase and btDispatcher.
   ///It is persistent over frames
   btCollisionAlgorithm = class
   protected
        m_dispatcher       : btDispatcher;
       //IMPL not found function           getDispatcherId:integer;
   public
        //Create / Init Split for pooled Allocations
        procedure       Init                    (const ci:btCollisionAlgorithmConstructionInfo);virtual;
        procedure       processCollision        (const body0, body1 : btCollisionObject;const dispatchInfo:btDispatcherInfo;var resultOut:btManifoldResult); virtual;abstract;
        function        calculateTimeOfImpact   (const body0, body1 : btCollisionObject;const dispatchInfo:btDispatcherInfo;var resultOut:btManifoldResult):btScalar;virtual;abstract;
        procedure       getAllContactManifolds  (const manifoldArray: btManifoldArray);virtual;abstract;
   end;
   PbtCollisionAlgorithm=^btCollisionAlgorithm;


   /////Used by the btCollisionDispatcher to register and create instances for btCollisionAlgorithm

   { btCollisionAlgorithmCreateFunc }

   btCollisionAlgorithmCreateFunc=class
      m_swapped        : boolean;
      constructor  Create(const swapped:boolean=false);
      function     CreateCollisionAlgorithm(const info:btCollisionAlgorithmConstructionInfo; const body0,body1:btCollisionObject):btCollisionAlgorithm;virtual;abstract;
   end;

   ///btCollisionConfiguration allows to configure Bullet collision detection
   ///stack allocator size, default collision algorithms and persistent manifold pool size
   ///@todo: describe the meaning
   btCollisionConfiguration=class
   public
      ///memory pools
      function  getPersistentManifoldPool       :btPoolAllocator  ;virtual;abstract;
      function  getCollisionAlgorithmPool       :btPoolAllocator  ;virtual;abstract;
      function  getStackAllocator               :btStackAllocator ;virtual;abstract;
      function  getCollisionAlgorithmCreateFunc (const proxyType0,proxyType1 :TbtBroadphaseNativeTypes):btCollisionAlgorithmCreateFunc; virtual;abstract;
   end;

  btCollisionDispatcher=class;

   ///user can override this nearcallback for collision filtering and more finegrained control over collision detection
  btNearCallback= procedure(const collisionPair:PbtBroadphasePair;const dispatcher:btCollisionDispatcher; var dispatchInfo:btDispatcherInfo);

  btDispatcherFlags   = (CD_STATIC_STATIC_REPORTED, CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
  btDispatcherFlagSet = set of btDispatcherFlags;

   ///btCollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
   ///Time of Impact, Closest Points and Penetration Depth.

  { btCollisionDispatcher }

  btCollisionDispatcher=class(btDispatcher)
    m_dispatcherFlags                 : btDispatcherFlagSet;
    m_manifoldsPtr                    : btManifoldArray;
    m_defaultManifoldResult           : btManifoldResult;
    m_nearCallback                    : btNearCallback;
    m_collisionAlgorithmPoolAllocator : btPoolAllocator;
    m_persistentManifoldPoolAllocator : btPoolAllocator;
    m_doubleDispatch                  : Array [low(TbtBroadphaseNativeTypes)..high(TbtBroadphaseNativeTypes),low(TbtBroadphaseNativeTypes)..high(TbtBroadphaseNativeTypes)] of btCollisionAlgorithmCreateFunc;
    m_collisionConfiguration          : btCollisionConfiguration;
  public
    function    getDispatherFlags            :btDispatcherFlagSet;
    procedure   setDispatcherFlags           (const flags:btDispatcherFlagSet);
    ///registerCollisionCreateFunc allows registration of custom/alternative collision create functions
    procedure   registerCollisionCreateFunc  (const proxyType0,proxyType1:TbtBroadphaseNativeTypes;const createFunc:btCollisionAlgorithmCreateFunc);
    function    getNumManifolds              : integer; override;
    function    getManifoldByIndexInternal(const index: integer): btPersistentManifold; override;
    constructor Create(const collisionConfiguration:btCollisionConfiguration);
    destructor  Destroy;override;
    function    getNewManifold(const body0, body1: btCollisionObject): btPersistentManifold; override;
    procedure   releaseManifold(const manifold: btPersistentManifold); override;
    procedure   clearManifold(const manifold: btPersistentManifold); override;
    function    findAlgorithm(const body0, body1:  btCollisionObject; const sharedManifold: btPersistentManifold=nil): btCollisionAlgorithm; override;
    function    needsCollision(const body0, body1: btCollisionObject): Boolean; override;
    function    needsResponse(const body0, body1:  btCollisionObject): Boolean; override;
    procedure   dispatchAllCollisionPairs  (const pairCache: btOverlappingPairCache; const dispatchInfo: btDispatcherInfo; const dispatcher: btDispatcher);override;
    procedure   setNearCallback(const nearCallback:btNearCallback);
    function    getNearCallback:btNearCallback;
    function    allocateCollisionAlgorithm(const alg: btCollisionAlgorithmClass): btCollisionAlgorithm; override;
    procedure   freeCollisionAlgorithm(const alg: btCollisionAlgorithm); override;
   //     virtual void* allocateCollisionAlgorithm(int size);
   //     virtual void freeCollisionAlgorithm(void* ptr);
     function   getCollisionConfiguration: btCollisionConfiguration;
     procedure  setCollisionConfiguration(const config: btCollisionConfiguration);
   end;

   btBroadphaseAabbCallback=class
     function process (const proxy:btBroadphaseProxy):boolean;virtual;abstract;
   end;

   btBroadphaseRayCallback=class(btBroadphaseAabbCallback)
      ///added some cached data to accelerate ray-AABB tests
      m_rayDirectionInverse     : btVector3;
      m_signs                   : array [0..2] of Cardinal;
      m_lambda_max              : btScalar;
   end;

   { btBroadphaseInterface }

   btBroadphaseInterface=class
   public
     function   createProxy               (const aabbMin, aabbMax:btVector3;const shapeType:TbtBroadphaseNativeTypes;const userPtr:Pointer; const collisionFilterGroup,collisionFilterMask:btCollisionFilterGroupSet; const dispatcher : btDispatcher; const multiSapProxy:Pointer):btBroadphaseProxy;virtual;abstract;
     procedure  destroyProxy              (const proxy:btBroadphaseProxy;const dispatcher:btDispatcher);virtual;abstract;
     procedure  setAabb                   (const proxy:btBroadphaseProxy;const aabbMin,aabbMax:btVector3; const dispatcher : btDispatcher);virtual;abstract;
     procedure  getAabb                   (const proxy:btBroadphaseProxy;var   aabbMin,aabbMax:btVector3) ;virtual;abstract;
     procedure  rayTest                   (const rayFrom,rayTo:btVector3;const rayCallback: btBroadphaseRayCallback; const aabbMin,aabbMax:btVector3);virtual;abstract; //aabbmin,max = (0,0,0) default ? howto
     procedure  aabbTest                  (const aabbMin,aabbMax:btVector3;const callback:btBroadphaseAabbCallback);virtual;abstract;
     ///calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
     procedure  calculateOverlappingPairs (const dispatcher:btDispatcher);virtual;abstract;
     function   getOverlappingPairCache   :btOverlappingPairCache;virtual;abstract;
     ///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
     ///will add some transform later
     procedure  getBroadphaseAabb        (out aabbMin,aabbMax:btVector3);virtual;abstract;
     ///reset broadphase internal structures, to ensure determinism/reproducability
     procedure  resetPool                (const dispatcher:btDispatcher);virtual;
     procedure  printStats               ;virtual;abstract;
   end;


  { btSimpleBroadphaseProxy }

  btSimpleBroadphaseProxy =class (btBroadphaseProxy)
    m_nextFree: integer;
    m_handleId: integer;
    constructor Create;
    procedure   init              (const minpt, maxpt:btVector3;const shapeType:TbtBroadphaseNativeTypes;const userPtr:Pointer;const collisionFilterGroup,collisionFilterMask:btCollisionFilterGroupSet;const multiSapProxy:pointer;const handleindex:cardinal);reintroduce;
    procedure   SetNextFree       (const next:integer);FOS_INLINE;
    function    GetNextFree       :integer;FOS_INLINE;
  end;

  btAlignedSimpleBroadphaseproxyArray=specialize FOS_GenericAlignedArray<btSimpleBroadphaseProxy>;

  ///The SimpleBroadphase is just a unit-test for btAxisSweep3, bt32BitAxisSweep3, or btDbvtBroadphase, so use those classes instead.
  ///It is a brute force aabb culling broadphase based on O(n^2) aabb checks

  btSimpleBroadphase =class(btBroadphaseInterface)
  protected
    m_numHandles      : integer;  // number of active handles
    m_maxHandles      : integer;  // max number of handles
    m_LastHandleIndex : integer;
    m_pHandles        : btAlignedSimpleBroadphaseproxyArray; // handles pool
    m_firstFreeHandle : integer;              // free handles list
    m_pairCache       : btOverlappingPairCache;
    m_ownsPairCache   : Boolean;
    m_invalidPair     : integer;

    function   allocHandle             : integer;
    procedure  freeHandle              (const  proxy:btSimpleBroadphaseProxy);
    ///reset broadphase internal structures, to ensure determinism/reproducability
  public
    constructor    Create                    (const maxProxies:integer=16384;const overlappingPairCache:btOverlappingPairCache=nil);
    destructor     Destroy                   ;override;
    procedure      resetPool                 (const dispatcher: btDispatcher);override;
    procedure      validate                  ;
    class function aabbOverlap               (const proxy0,proxy1:btSimpleBroadphaseProxy):boolean;FOS_INLINE;
    function       createProxy               (const aabbMin, aabbMax: btVector3; const shapeType: TbtBroadphaseNativeTypes; const userPtr: Pointer; const collisionFilterGroup, collisionFilterMask: btCollisionFilterGroupSet; const dispatcher: btDispatcher; const multiSapProxy: Pointer): btBroadphaseProxy; override;
    procedure      calculateOverlappingPairs (const dispatcher: btDispatcher); override;
    procedure      destroyProxy              (const proxy: btBroadphaseProxy; const dispatcher: btDispatcher); override;
    procedure      setAabb                   (const proxy: btBroadphaseProxy; const aabbMin, aabbMax: btVector3; const dispatcher: btDispatcher); override;
    procedure      getAabb                   (const proxy: btBroadphaseProxy; var aabbMin, aabbMax: btVector3); override;
    procedure      rayTest                   (const rayFrom, rayTo: btVector3; const rayCallback: btBroadphaseRayCallback; const aabbMin, aabbMax: btVector3); override;
    procedure      aabbTest                  (const aabbMin, aabbMax: btVector3; const callback: btBroadphaseAabbCallback); override;
    function       getOverlappingPairCache   : btOverlappingPairCache; override;
    function       testAabbOverlap           (const proxy0,proxy1:btBroadphaseProxy):boolean;FOS_INLINE;
    ///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
    ///will add some transform later
    procedure      getBroadphaseAabb         (out aabbMin, aabbMax: btVector3); override;
    procedure      printStats                ; override;
  end;

  ///The btDbvtBroadphase implements a broadphase using two dynamic AABB bounding volume hierarchies/trees (see btDbvt).
  ///One tree is used for static/non-moving objects, and another tree is used for dynamic objects. Objects can move from one tree to the other.
  ///This is a very fast broadphase, especially for very dynamic worlds where many objects are moving. Its insert/add and remove of objects is generally faster than the sweep and prune broadphases btAxisSweep3 and bt32BitAxisSweep3.

  btDbvtBpCfg=(
    DYNAMIC_SET = 0,      // Dynamic set index
    FIXED_SET   = 1,      // Fixed set index
    STAGECOUNT  = 2       // Number of stages
    );
  {$ifdef DBVT_BP_PROFILE}
        btDbvtBPProfile = record
          m_total     : cardinal;
          m_ddcollide : cardinal;
          m_fdcollide : cardinal;
          m_cleanup   : cardinal;
          m_jobcount  : cardinal;
        end;
  {$endif}

  { btDbvtBroadphase }

  btDbvtBroadphase = class(btBroadphaseInterface)
    m_sets          : array [0..1] of btDbvt;                    // Dbvt sets
    m_stageRoots    : array [0..ord(STAGECOUNT)] of btDbvtProxy; // Stages list
    m_paircache     : btOverlappingPairCache;                    // Pair cache
    m_prediction    : btScalar;                                  // Velocity prediction
    m_stageCurrent,             // Current stage
    m_fupdates,                 // % of fixed updates per frame
    m_dupdates,                 // % of dynamic updates per frame
    m_cupdates,                 // % of cleanup updates per frame
    m_newpairs,                 // Number of pairs created
    m_fixedleft     : integer;  // Fixed optimization left
    m_updates_call,             // Number of updates call
    m_updates_done  : cardinal; // Number of updates done
    m_updates_ratio : btScalar; // m_updates_done/m_updates_call
    m_pid,                      // Parse id
    m_cid,                      // Cleanup index
    m_gid           : integer;  // Gen id
    m_releasepaircache,         // Release pair cache on delete
    m_deferedcollide,           // Defere dynamic/static collision to collide call
    m_needcleanup   : boolean;  // Need to run cleanup?
  {$ifdef DBVT_BP_PROFILE}
    m_clock         : btClock;
    m_profiling     : btDbvtBPProfile;
  {$endif}
    constructor Create       (const paircache:btOverlappingPairCache=nil);
    destructor  Destroy      ;override;
    procedure   collide      (const dispatcher : btDispatcher);
    procedure   optimize     ;
    //    /* btBroadphaseInterface Implementation */
    function    createProxy(const aabbMin, aabbMax: btVector3; const shapeType: TbtBroadphaseNativeTypes; const userPtr: Pointer; const collisionFilterGroup, collisionFilterMask: btCollisionFilterGroupSet; const dispatcher: btDispatcher;const multiSapProxy: Pointer): btBroadphaseProxy; override;
    procedure   destroyProxy(const absproxy: btBroadphaseProxy; const dispatcher: btDispatcher); override;
    procedure   setAabb(const absproxy: btBroadphaseProxy; const aabbMin, aabbMax: btVector3; const dispatcher: btDispatcher); override;
    procedure   rayTest(const rayFrom, rayTo: btVector3; const rayCallback: btBroadphaseRayCallback; const aabbMin, aabbMax: btVector3); override;
    procedure   aabbTest(const aabbMin, aabbMax: btVector3; const aabbcallback: btBroadphaseAabbCallback); override;
    procedure   getAabb(const proxy: btBroadphaseProxy; var aabbMin, aabbMax: btVector3); override;
    procedure   calculateOverlappingPairs(const dispatcher: btDispatcher); override;
    function    getOverlappingPairCache: btOverlappingPairCache; override;
    procedure   getBroadphaseAabb(out aabbMin, aabbMax: btVector3); override;
    procedure   printStats; override;

    ///reset broadphase internal structures, to ensure determinism/reproducability
    procedure   resetPool              (const dispatcher: btDispatcher); override;
    procedure   performDeferredRemoval (const dispatcher: btDispatcher);
    procedure   setVelocityPrediction  (const prediction: btScalar);
    function    getVelocityPrediction  : btScalar;
    /////this setAabbForceUpdate is similar to setAabb but always forces the aabb update.
    /////it is not part of the btBroadphaseInterface but specific to btDbvtBroadphase.
    /////it bypasses certain optimizations that prevent aabb updates (when the aabb shrinks), see
    /////http://code.google.com/p/bullet/issues/detail?id=223
    procedure  setAabbForceUpdate(const absproxy: btBroadphaseProxy; const aabbMin, aabbMax: btVector3; const dispatcher: btDispatcher);
    class procedure benchmark ;
  end;

// AXIS SWEEP 3
/// The internal templace class btAxisSweep3Internal implements the sweep and prune broadphase.
/// It uses quantized integers to represent the begin and end points for each of the 3 axis.
/// Dont use this class directly, use btAxisSweep3 or bt32BitAxisSweep3 instead.

//  { btAxisSweep3Internal_Edge }
//
//  generic btAxisSweep3Internal_Edge<BP_FP_INT_TYPE>=class
//    m_pos    : BP_FP_INT_TYPE; // low bit is min/max
//    m_handle : BP_FP_INT_TYPE;
//    function IsMax:boolean;
//  end;
//
//  { btAxisSweep3Internal_Handle }
//
//  generic btAxisSweep3Internal_Handle<BP_FP_INT_TYPE>=class(btBroadphaseProxy)
//    // indexes into the edge arrays
//    m_minEdges,
//    m_maxEdges  : ARRAY [0..2] of BP_FP_INT_TYPE; // 6 * 2 = 12
//    m_dbvtProxy : btBroadphaseProxy;//for faster raycast
//    procedure    SetNextFree(const next : BP_FP_INT_TYPE);FOS_INLINE;
//    function     GetNextFree:BP_FP_INT_TYPE;FOS_INLINE; // 24 bytes + 24 for Edge structures = 44 bytes total per entry
//  end;
//
//  generic btAxisSweep3Internal<BP_FP_INT_TYPE> = class(btBroadphaseInterface)
//  private type
//     _AS3_Handle  = specialize btAxisSweep3Internal_Handle<BP_FP_INT_TYPE>;
//     _AS3_HandleA = specialize FOS_GenericAlignedArray<_AS3_Handle>;
//     _AS3_Edge    = specialize btAxisSweep3Internal_Edge<BP_FP_INT_TYPE>;
//     _AS3_EdgeA   = specialize FOS_GenericAlignedArray<_AS3_Edge>;
////     P_AS3_Handle = ^_AS3_Handle;
//     P_AS3_Edge   = ^_AS3_Edge;
//     PBP_FP_INT_TYPE = ^BP_FP_INT_TYPE;
//  protected
//     m_bpHandleMask   : BP_FP_INT_TYPE;
//     m_handleSentinel : BP_FP_INT_TYPE;
//     m_worldAabbMin,                    // overall system bounds
//     m_worldAabbMax,                    // overall system bounds
//     m_quantize       : btVector3;      // scaling factor for quantization
//     m_numHandles,                      // number of active handles
//     m_maxHandles     : BP_FP_INT_TYPE; // max number of handles
//     m_pHandles       : _AS3_HandleA;   // handles pool
//     m_firstFreeHandle:BP_FP_INT_TYPE;  // free handles list
//     m_pEdges         : array [0..2] of _AS3_EdgeA; // edge arrays for the 3 axes (each array has m_maxHandles * 2 + 2 sentinel entries)
//     m_pairCache      : btOverlappingPairCache;
//     ///btOverlappingPairCallback is an additional optional user callback for adding/removing overlapping pairs, similar interface to btOverlappingPairCache.
//     m_userPairCallback : btOverlappingPairCallback;
//     m_ownsPairCache    : Boolean;
//     m_invalidPair      : integer;
//     ///additional dynamic aabb structure, used to accelerate ray cast queries.
//     ///can be disabled using a optional argument in the constructor
//     m_raycastAccelerator : btDbvtBroadphase;
//     m_nullPairCache      : btOverlappingPairCache;
//     // allocation/deallocation
//     function  allocHandle : BP_FP_INT_TYPE;
//     procedure freeHandle(const handle : BP_FP_INT_TYPE);
//     function  testOverlap2D(const pHandleA, pHandleB : _AS3_Handle ; const axis0,axis1 : integer) : boolean;
//     {$ifdef DEBUG_BROADPHASE}
//     procedure debugPrintAxis(const axis : integer ;const checkCardinality:Boolean=true);
//     {$endif} //DEBUG_BROADPHASE
//     procedure sortMinDown (const axis:integer; const edge :BP_FP_INT_TYPE ; const dispatcher :btDispatcher ; const updateOverlaps : Boolean);
//     procedure sortMinUp   (const axis:integer; const edge :BP_FP_INT_TYPE ; const dispatcher :btDispatcher ; const updateOverlaps : Boolean);
//     procedure sortMaxDown (const axis:integer; const edge :BP_FP_INT_TYPE ; const dispatcher :btDispatcher ; const updateOverlaps : Boolean);
//     procedure sortMaxUp   (const axis:integer; const edge :BP_FP_INT_TYPE ; const dispatcher :btDispatcher ; const updateOverlaps : Boolean);
//     procedure FOS_Assign  (var dst:_AS3_Edge;const src:_AS3_Edge);FOS_INLINE;
//     procedure _DumpHandles;
//  public
//     constructor create        (const worldAabbMin,worldAabbMax : btVector3 ; const handleMask,handleSentinel: BP_FP_INT_TYPE ; const usermaxHandles : BP_FP_INT_TYPE = 16384 ; const pairCache : btOverlappingPairCache=nil ;const  disableRaycastAccelerator : Boolean=false);
//     destructor  destroy       ; override;
//     function    getNumHandles : BP_FP_INT_TYPE;
//     procedure   calculateOverlappingPairs (const dispatcher : btDispatcher);override;
//     function    addHandle                 (const aabbMin,aabbMax : btVector3 ; const pOwner : pointer ; const collisionFilterGroup,collisionFilterMask : btCollisionFilterGroupSet;const dispatcher : btDispatcher ; const multiSapProxy : Pointer):BP_FP_INT_TYPE;
//     procedure   removeHandle              (const handle : BP_FP_INT_TYPE;const dispatcher:btDispatcher);
//     procedure   updateHandle              (const handle : BP_FP_INT_TYPE;const aabbMin,aabbMax : btVector3 ;const dispatcher : btDispatcher);
//     function    getHandle                 (const index :BP_FP_INT_TYPE):_AS3_Handle;FOS_INLINE;
//     procedure   resetPool                 (const dispatcher: btDispatcher); override;
////     procedure   processAllOverlappingPairs(const callback : btOverlapCallback);
//     function    createProxy(const aabbMin, aabbMax: btVector3; const shapeType: TbtBroadphaseNativeTypes; const userPtr: Pointer; const collisionFilterGroup, collisionFilterMask: btCollisionFilterGroupSet;const dispatcher: btDispatcher; const multiSapProxy: Pointer): btBroadphaseProxy; override;
//     procedure   destroyProxy(const proxy: btBroadphaseProxy; const dispatcher: btDispatcher); override;
//     procedure   setAabb(const proxy: btBroadphaseProxy; const aabbMin, aabbMax: btVector3; const dispatcher: btDispatcher); override;
//     procedure   getAabb(const proxy: btBroadphaseProxy; var aabbMin, aabbMax: btVector3); override;
//     procedure   rayTest(const rayFrom, rayTo: btVector3; const rayCallback: btBroadphaseRayCallback; const aabbMin, aabbMax: btVector3); override;
//     procedure   aabbTest(const aabbMin, aabbMax: btVector3; const callback: btBroadphaseAabbCallback); override;
//     procedure   quantize(const out:PBP_FP_INT_TYPE ; const point : btVector3 ; const isMax : integer);
//     ///unQuantize should be conservative: aabbMin/aabbMax should be larger then 'getAabb' result
//     procedure   unQuantize(const proxy : btBroadphaseProxy;var aabbMin, aabbMax : btVector3);
//     function    testAabbOverlap(const proxy0,proxy1 : btBroadphaseProxy):boolean;
//     function    getOverlappingPairCache : btOverlappingPairCache;override;
//     procedure   setOverlappingPairUserCallback(const  pairCallback : btOverlappingPairCallback);
//     function    getOverlappingPairUserCallback:btOverlappingPairCallback;
//      ///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
//      ///will add some transform later
//    procedure getBroadphaseAabb(var aabbMin, aabbMax: btVector3); override;
//    procedure printStats; override;
//  end;
//
//  _btAxisSweep3      = specialize btAxisSweep3Internal<WORD>;
//  _bt32BitAxisSweep3 = specialize btAxisSweep3Internal<CARDINAL>;
//
//  btAxisSweep3 = class(_btAxisSweep3)
//    constructor create        (const worldAabbMin,worldAabbMax : btVector3 ; maxHandles : word = 16384 ; const pairCache : btOverlappingPairCache=nil ;const  disableRaycastAccelerator : Boolean=false);
//  end;
//
//  { bt32BitAxisSweep3 }
//
//  bt32BitAxisSweep3 = class(_bt32BitAxisSweep3)
//    constructor create        (const worldAabbMin,worldAabbMax : btVector3 ; maxHandles : cardinal = 1500000 ; const pairCache : btOverlappingPairCache=nil ;const  disableRaycastAccelerator : Boolean=false);
//  end;
//

  { btDefaultCollisionConstructionInfo }

  btDefaultCollisionConstructionInfo=object
    m_stackAlloc                    : btStackAllocator;
    m_persistentManifoldPool        : btPoolAllocator;
    m_collisionAlgorithmPool        : btPoolAllocator;
    m_defaultMaxPersistentManifoldPoolSize,
    m_defaultMaxCollisionAlgorithmPoolSize,
    m_customCollisionAlgorithmMaxElementSize,
    m_defaultStackAllocatorSize     : integer;
    m_useEpaPenetrationAlgorithm    : boolean;
    function                        Init:btDefaultCollisionConstructionInfo;static;
  end;

  ///btCollisionConfiguration allows to configure Bullet collision detection
  ///stack allocator, pool memory allocators
  ///@todo: describe the meaning

  { btDefaultCollisionConfiguration }

  btDefaultCollisionConfiguration=class(btCollisionConfiguration)
  protected
    m_persistentManifoldPoolSize          : integer;
    m_stackAlloc                          : btStackAllocator;
    m_ownsStackAllocator                  : boolean;
    m_persistentManifoldPool              : btPoolAllocator;
    m_ownsPersistentManifoldPool          : boolean;
    m_collisionAlgorithmPool              : btPoolAllocator;
    m_ownsCollisionAlgorithmPool          : boolean;
    //default simplex/penetration depth solvers
    m_simplexSolver                       : btVoronoiSimplexSolver;
    m_pdSolver                            : btConvexPenetrationDepthSolver;
    //default CreationFunctions, filling the m_doubleDispatch table
    m_convexConvexCreateFunc,
    m_convexConcaveCreateFunc,
    m_swappedConvexConcaveCreateFunc,
    m_compoundCreateFunc,
    m_swappedCompoundCreateFunc,
    m_emptyCreateFunc,
    m_sphereSphereCF,
  {$ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM}
    m_sphereBoxCF,
    m_boxSphereCF,
  {$endif //USE_BUGGY_SPHERE_BOX_ALGORITHM}
    m_boxBoxCF,
    m_sphereTriangleCF,
    m_triangleSphereCF,
    m_planeConvexCF,
    m_convexPlaneCF                        : btCollisionAlgorithmCreateFunc;
  public
    constructor Create                           (const constructionInfo:btDefaultCollisionConstructionInfo); //  = btDefaultCollisionConstructionInfo()
    destructor  Destroy                          ;override;
    function    getPersistentManifoldPool        : btPoolAllocator; override;
    function    getCollisionAlgorithmPool        : btPoolAllocator; override;
    function    getStackAllocator                : btStackAllocator; override;
    function    getSimplexSolver                 : btVoronoiSimplexSolver;
    function    getCollisionAlgorithmCreateFunc  (const proxyType0, proxyType1: TbtBroadphaseNativeTypes): btCollisionAlgorithmCreateFunc; override;
    ///Use this method to allow to generate multiple contact points between at once, between two objects using the generic convex-convex algorithm.
    ///By default, this feature is disabled for best performance.
    ///@param numPerturbationIterations controls the number of collision queries. Set it to zero to disable the feature.
    ///@param minimumPointsPerturbationThreshold is the minimum number of points in the contact cache, above which the feature is disabled
    ///3 is a good value for both params, if you want to enable the feature. This is because the default contact cache contains a maximum of 4 points, and one collision query at the unperturbed orientation is performed first.
    ///See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
    ///@todo we could add a per-object setting of those parameters, for level-of-detail collision detection.
    procedure   setConvexConvexMultipointIterations(const numPerturbationIterations:integer=3;const minimumPointsPerturbationThreshold:integer = 3);
  end;


  btEmptyAlgorithm=class(btCollisionAlgorithm)
  public
    procedure   processCollision(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult); override;
    function    calculateTimeOfImpact(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar; override;
    procedure   getAllContactManifolds(const manifoldArray: btManifoldArray); override;
  end;

  { btEmptyAlgorithmCreateFunc }

  btEmptyAlgorithmCreateFunc=class(btCollisionAlgorithmCreateFunc)
    function    CreateCollisionAlgorithm (const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;override;
  end;


  ///This class is not enabled yet (work-in-progress) to more aggressively activate objects.
  btActivatingCollisionAlgorithm=class(btCollisionAlgorithm)
    procedure Init                   (const ci:btCollisionAlgorithmConstructionInfo;const body0, body1:btCollisionObject);reintroduce; // USing INIT to split allocation Create from initiaization (init) (FOS)
  end;

  /// btBoxBoxDetector wraps the ODE box-box collision detector
  /// re-distributed under the Zlib license with permission from Russell L. Smith

  { btBoxBoxDetector }

  btBoxBoxDetector=object
    m_box1   : btBoxShape;
    m_box2   : btBoxShape;
  public
    procedure  init             (const box1,box2:btBoxShape);
    procedure  getClosestPoints (const input:btDCDI_ClosestPointInput;var output:btDCDI_Result;const debugDraw:btIDebugDraw;const swapResults:boolean=false);
  end;


  btBoxBoxCollisionAlgorithm=class(btActivatingCollisionAlgorithm)
    m_ownManifold             : boolean;
    m_manifoldPtr             : btPersistentManifold;
  public
    procedure Init                   (const ci:btCollisionAlgorithmConstructionInfo;const body0, body1:btCollisionObject);
    destructor                       Destroy;override;
    procedure processCollision       (const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult); override;
    function  calculateTimeOfImpact  (const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar; override;
    procedure getAllContactManifolds (const manifoldArray: btManifoldArray); override;
  end;

  { btBoxBoxCreateFunc }

  btBoxBoxCollisionAlgorithmCreateFunc=class(btCollisionAlgorithmCreateFunc)
    function  CreateCollisionAlgorithm (const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;override;
  end;


  ///Enabling USE_SEPDISTANCE_UTIL2 requires 100% reliable distance computation. However, when using large size ratios GJK can be imprecise
  ///so the distance is not conservative. In that case, enabling this USE_SEPDISTANCE_UTIL2 would result in failing/missing collisions.
  ///Either improve GJK for large size ratios (testing a 100 units versus a 0.1 unit object) or only enable the util
  ///for certain pairs that have a small size ratio

  //#define USE_SEPDISTANCE_UTIL2 1

  ///The convexConvexAlgorithm collision algorithm implements time of impact, convex closest points and penetration depth calculations between two convex objects.
  ///Multiple contact points are calculated by perturbing the orientation of the smallest object orthogonal to the separating normal.
  ///This idea was described by Gino van den Bergen in this forum topic http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=4&t=288&p=888#p888

  { btConvexConvexAlgorithm }

  btConvexConvexAlgorithm =class(btActivatingCollisionAlgorithm)
  {$ifdef USE_SEPDISTANCE_UTIL2}
    m_sepDistance                        : btConvexSeparatingDistanceUtil;
  {$endif}
    m_simplexSolver                      : btSimplexSolverInterface;
    m_pdSolver                           : btConvexPenetrationDepthSolver;
    m_ownManifold                        : Boolean;
    m_manifoldPtr                        : btPersistentManifold;
    m_lowLevelOfDetail                   : Boolean;
    m_numPerturbationIterations          : integer;
    m_minimumPointsPerturbationThreshold : integer;
    ///cache separating vector to speedup collision detection
  public
    procedure   Init                   (const  mf:btPersistentManifold;const ci:btCollisionAlgorithmConstructionInfo;const body0, body1:btCollisionObject;const simplexSolver:btSimplexSolverInterface;
                                        const pdSolver : btConvexPenetrationDepthSolver; const numPerturbationIterations,minimumPointsPerturbationThreshold : integer);
    destructor  Destroy                ; override;
    procedure   processCollision       (const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult); override;
    function    calculateTimeOfImpact  (const col0, col1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar; override;
    procedure   getAllContactManifolds (const manifoldArray: btManifoldArray); override;
    procedure   setLowLevelOfDetail    (const useLowLevel:boolean);
    function    getManifold            : btPersistentManifold;
  end;

  { btConvexConvexAlgorithmCreateFunc }

  btConvexConvexAlgorithmCreateFunc=class(btCollisionAlgorithmCreateFunc)
    m_pdSolver                           : btConvexPenetrationDepthSolver;
    m_simplexSolver                      : btSimplexSolverInterface;
    m_numPerturbationIterations,
    m_minimumPointsPerturbationThreshold : integer;
    constructor Create                   (const simplexSolver:btSimplexSolverInterface;const pdSolver:btConvexPenetrationDepthSolver);
    function    CreateCollisionAlgorithm (const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;override;
  end;


  ///For each triangle in the concave mesh that overlaps with the AABB of a convex (m_convexProxy), processTriangle is called.

  { btConvexTriangleCallback }

  btConvexTriangleCallback=class(btTriangleCallback)
    m_convexBody              : btCollisionObject;
    m_triBody                 : btCollisionObject;
    m_aabbMin                 : btVector3;
    m_aabbMax                 : btVector3;
    m_resultOut               : btManifoldResult;
    m_dispatcher              : btDispatcher;
    m_dispatchInfoPtr         : PbtDispatcherInfo;
    m_collisionMarginTriangle : btScalar ;
  public
    m_triangleCount           : integer;
    m_manifoldPtr             : btPersistentManifold;
    constructor create                  (const dispatcher:btDispatcher;const body0,body1 :btCollisionObject;const isSwapped:boolean);
    destructor  destroy                 ;override;
    procedure   setTimeStepAndCounters  (const collisionMarginTriangle:btScalar;const dispatchInfo:btDispatcherInfo;var resultOut:btManifoldResult);
    procedure   processTriangle         (const triangle: PbtVector3; const partId, triangleIndex: integer); override;
    procedure   clearcache              ;
    function    getAabbMin              : btVector3;
    function    getAabbMax              : btVector3;
  end;

  /// btConvexConcaveCollisionAlgorithm  supports collision between convex shapes and (concave) trianges meshes.

  { btConvexConcaveCollisionAlgorithm }

  btConvexConcaveCollisionAlgorithm = class(btActivatingCollisionAlgorithm)
    m_isSwapped                : boolean;
    m_btConvexTriangleCallback : btConvexTriangleCallback;
  public
    destructor  Destroy                ;override;
    procedure   Init                   (const ci:btCollisionAlgorithmConstructionInfo;const body0, body1:btCollisionObject;const isSwapped:boolean);
    procedure   processCollision       (const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult); override;
    function    calculateTimeOfImpact  (const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar; override;
    procedure   getAllContactManifolds (const manifoldArray: btManifoldArray); override;
    procedure   clearchache;
  end;

  { btConvexConcaveCollisionAlgorithmCreateFunc }

  btConvexConcaveCollisionAlgorithmCreateFuncNormAndSwapped=class(btCollisionAlgorithmCreateFunc)
    function    CreateCollisionAlgorithm (const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;override;
  end;

  btCollisionAlgorithmArray=specialize FOS_GenericAlignedArray<btCollisionAlgorithm>;

  /// btCompoundCollisionAlgorithm  supports collision between CompoundCollisionShapes and other collision shapes

  { btCompoundCollisionAlgorithm }

  btCompoundCollisionAlgorithm=class(btActivatingCollisionAlgorithm)
    m_childCollisionAlgorithms : btCollisionAlgorithmArray;
    m_sharedManifold           : btPersistentManifold;
    m_ownsManifold             : boolean;
    m_isSwapped                : boolean;
    m_compoundShapeRevision    : integer;//to keep track of changes, so that childAlgorithm array can be updated
    procedure  removeChildAlgorithms      ;
    procedure  preallocateChildAlgorithms (const body0,body1 : btCollisionObject);
  public
    procedure   Init                   (const ci:btCollisionAlgorithmConstructionInfo;const body0, body1:btCollisionObject;const isSwapped:boolean);
    destructor  Destroy                ;override;
    procedure   processCollision       (const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult); override;
    function    calculateTimeOfImpact  (const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar; override;
    procedure   getAllContactManifolds (const manifoldArray: btManifoldArray); override;
  end;

  { btCompoundCollisionAlgorithmCreateFuncNormAndSwapped }

  btCompoundCollisionAlgorithmCreateFuncNormAndSwapped=class(btCollisionAlgorithmCreateFunc)
    function    CreateCollisionAlgorithm (const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;override;
  end;

  { btConvexPlaneCollisionAlgorithm }

  btConvexPlaneCollisionAlgorithm=class(btCollisionAlgorithm)
    m_ownManifold : boolean;
    m_manifoldPtr : btPersistentManifold;
    m_isSwapped   : boolean;
    m_numPerturbationIterations          : integer;
    m_minimumPointsPerturbationThreshold : integer;
  public
    procedure  Init(const mf:btPersistentManifold;const ci:btCollisionAlgorithmConstructionInfo;const body0, body1:btCollisionObject;const isSwapped:boolean; const numPerturbationIterations,minimumPointsPerturbationThreshold : integer);reintroduce;
    destructor Destroy;override;
    procedure  processCollision     (const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult); override;
    procedure  collideSingleContact (const perturbeRot:btQuaternion; const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult);
    function   calculateTimeOfImpact(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar; override;
    procedure  getAllContactManifolds(const manifoldArray: btManifoldArray); override;
  end;

  { btConvexPlaneCollisionAlgorithmCreateFuncNormAndSwapped }

  btConvexPlaneCollisionAlgorithmCreateFuncNormAndSwapped=class(btCollisionAlgorithmCreateFunc)
    m_numPerturbationIterations,m_minimumPointsPerturbationThreshold:integer;
    constructor Create(const swapped: boolean=false);
    function    CreateCollisionAlgorithm (const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;override;
  end;

  ///// btSphereSphereCollisionAlgorithm  provides sphere-sphere collision detection.
  ///// Other features are frame-coherency (persistent data) and collision response.
  ///// Also provides the most basic sample for custom/user btCollisionAlgorithm

  { btSphereSphereCollisionAlgorithm }

  btSphereSphereCollisionAlgorithm = class(btActivatingCollisionAlgorithm)
    m_ownManifold : boolean;
    m_manifoldPtr : btPersistentManifold;
  public
    procedure  Init(const mf:btPersistentManifold;const ci:btCollisionAlgorithmConstructionInfo;const body0, body1:btCollisionObject);
    destructor Destroy;override;
    procedure  processCollision(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult); override;
    function   calculateTimeOfImpact(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar; override;
    procedure  getAllContactManifolds(const manifoldArray: btManifoldArray); override;
  end;

  { btSphereSphereCollisionAlgorithmCreateFunc }

  btSphereSphereCollisionAlgorithmCreateFunc=class(btCollisionAlgorithmCreateFunc)
    function  CreateCollisionAlgorithm (const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;override;
  end;


  // sphere-triangle to match the btDiscreteCollisionDetectorInterface

  { btSphereTriangleDetector }

  btSphereTriangleDetector=object
  private
    m_sphere                   : btSphereShape;
    m_triangle                 : btTriangleShape;
    m_contactBreakingThreshold : btScalar;
    function    pointInTriangle (const vertices :  PbtVector3; const  normal : btVector3 ; const p : PbtVector3) : boolean;
    function    facecontains    (const p        :  btVector3 ; const vertices: PbtVector3; const normal : btVector3):boolean;
  public
    procedure   getClosestPoints (const input: btDCDI_ClosestPointInput ; var output: btDCDI_Result; const debugDraw: btIDebugDraw; const swapResults: boolean=false);
    function    collide          (const sphereCenter: btVector3;  out point,resultNormal:btVector3 ; var depth, timeOfImpact : btScalar; const contactBreakingThreshold : btScalar):boolean;
    procedure   Init             (const  sphere:btSphereShape;const triangle:btTriangleShape;const contactBreakingThreshold:btScalar);
  end;

  /// btSphereSphereCollisionAlgorithm  provides sphere-sphere collision detection.
  /// Other features are frame-coherency (persistent data) and collision response.
  /// Also provides the most basic sample for custom/user btCollisionAlgorithm

  { btSphereTriangleCollisionAlgorithm }

  btSphereTriangleCollisionAlgorithm=class(btActivatingCollisionAlgorithm)
    m_ownManifold : boolean;
    m_manifoldPtr : btPersistentManifold;
    m_swapped   : boolean;
  public
    procedure  Init(const mf:btPersistentManifold;const ci:btCollisionAlgorithmConstructionInfo;const body0, body1:btCollisionObject;const swapped:boolean);
    destructor Destroy;override;
    procedure  processCollision(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult); override;
    function   calculateTimeOfImpact(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar; override;
    procedure  getAllContactManifolds(const manifoldArray: btManifoldArray); override;
  end;

  { btSphereTriangleCollisionAlgorithmCreateFunc }

  btSphereTriangleCollisionAlgorithmCreateFunc=class(btCollisionAlgorithmCreateFunc)
    function  CreateCollisionAlgorithm (const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;override;
  end;


///CollisionWorld is interface and container for the collision detection

  { btCollisionWorld }

  ///LocalShapeInfo gives extra information for complex shapes
  ///Currently, only btTriangleMeshShape is available, so it just contains triangleIndex and subpart
  btCW_LocalShapeInfo=record
    m_shapePart,
    m_triangleIndex : integer;
    //const btCollisionShape*       m_shapeTemp;
    //const btTransform*    m_shapeLocalTransform;
  end;
  PbtCW_LocalShapeInfo = ^btCW_LocalShapeInfo;

  { btCW_LocalRayResult }

  btCW_LocalRayResult=object
    m_collisionObject    : btCollisionObject;
    m_localShapeInfo     : PbtCW_LocalShapeInfo;
    m_hitNormalLocal     : btVector3;
    m_hitFraction        : btScalar;
    procedure Init       (const collisionObject : btCollisionObject;const localShapeInfo:PbtCW_LocalShapeInfo;const hitNormalLocal:btVector3;const hitFraction:btScalar);
  end;

    ///RayResultCallback is used to report new raycast results

  { btCW_RayResultCallback }

  btCW_RayResultCallback=class
    m_closestHitFraction        : btScalar;
    m_collisionObject           : btCollisionObject;
    m_collisionFilterGroup,
    m_collisionFilterMask       : btCollisionFilterGroupSet;
    m_flags                     : Cardinal;  //@BP Mod - Custom flags, currently used to enable backface culling on tri-meshes, see btRaycastCallback
    constructor create          ;
    function    hasHit          : boolean;
    function    needsCollision  (const proxy0:btBroadphaseProxy):boolean;virtual;
    function    addSingleResult (const rayResult : btCW_LocalRayResult;const normalInWorldSpace:Boolean):btScalar;virtual;abstract;
  end;

  { btCW_ClosestRayResultCallback }

  //TODO -> Create Init Split and / per Class init
  btCW_ClosestRayResultCallback=class(btCW_RayResultCallback)
    m_rayFromWorld   : btVector3;//used to calculate hitPointWorld from hitFraction
    m_rayToWorld     : btVector3;
    m_hitNormalWorld : btVector3;
    m_hitPointWorld  : btVector3;
    constructor Create (const rayFromWorld,rayToWorld : btVector3);
    function    addSingleResult(const rayResult : btCW_LocalRayResult; const normalInWorldSpace : Boolean):btScalar;override;
  end;

  { btCW_LocalConvexResult }

  btCW_LocalConvexResult=object
    m_hitCollisionObject : btCollisionObject;
    m_localShapeInfo     : PbtCW_LocalShapeInfo;
    m_hitNormalLocal,
    m_hitPointLocal      : btVector3;
    m_hitFraction        : btScalar;
    procedure Init (const hitCollisionObject : btCollisionObject ; const localShapeInfo : PbtCW_LocalShapeInfo ; const hitNormalLocal,hitPointLocal : btVector3 ; const hitFraction : btScalar);
  end;

  ///RayResultCallback is used to report new raycast results

  { btCW_ConvexResultCallback }

  btCW_ConvexResultCallback=class
    m_closestHitFraction   : btScalar;
    m_collisionFilterGroup : btCollisionFilterGroupSet;
    m_collisionFilterMask  : btCollisionFilterGroupSet;
    constructor Create     ;
    destructor  Destroy    ; override;
    function    hasHit     : boolean;
    function    needsCollision  (const proxy0 : btBroadphaseProxy):boolean;virtual;
    function    addSingleResult (const convexResult : btCW_LocalConvexResult ; const normalInWorldSpace : boolean):btScalar;virtual;abstract;
  end;

  { btCW_ClosestConvexResultCallback }

  btCW_ClosestConvexResultCallback = class(btCW_ConvexResultCallback)
    m_convexFromWorld, //used to calculate hitPointWorld from hitFraction
    m_convexToWorld,
    m_hitNormalWorld,
    m_hitPointWorld        : btVector3;
    m_hitCollisionObject   : btCollisionObject;
    constructor  Create (const convexFromWorld , convexToWorld : btVector3);
    function     addSingleResult(const convexResult: btCW_LocalConvexResult;const normalInWorldSpace: boolean): btScalar; override;
  end;

  ///ContactResultCallback is used to report contact points

  { btCW_ContactResultCallback }

  btCW_ContactResultCallback = class
    m_collisionFilterGroup : btCollisionFilterGroupSet;
    m_collisionFilterMask  : btCollisionFilterGroupSet;
    constructor create;
    procedure   Init;
    function    needsCollision  (const proxy0 : btBroadphaseProxy):boolean;virtual;
    function    addSingleResult (const cp: PbtOManifoldPoint ; const colObj0 : btCollisionObject;const partId0,index0 : integer;const colObj1 : btCollisionObject;const partId1, index1 : integer):btScalar;virtual;abstract;
  end;


  btCollisionWorld=class
  protected
    m_collisionObjects    : btCollisionObjectArray;
    m_dispatcher1         : btDispatcher;
    m_dispatchInfo        : btDispatcherInfo;
    m_stackAlloc          : btStackAllocator;
    m_broadphasePairCache : btBroadphaseInterface;
    m_debugDrawer         : btIDebugDraw;
    ///m_forceUpdateAllAabbs can be set to false as an optimization to only update active object AABBs
    ///it is true by default, because it is error-prone (setting the position of static objects wouldn't update their AABB)
    m_forceUpdateAllAabbs : boolean;
  //        void    serializeCollisionObjects(btSerializer* serializer);
  public
  //this constructor doesn't own the dispatcher and paircache/broadphase
    constructor create           (const dispatcher : btDispatcher;const broadphasePairCache:btBroadphaseInterface;const  collisionConfiguration:btCollisionConfiguration);
    destructor  destroy          ;override;
    procedure   setBroadphase    (const pairCache:btBroadphaseInterface);
    function    getBroadphase    :btBroadphaseInterface;
    function    getPairCache     :btOverlappingPairCache;
    function    getDispatcher    :btDispatcher;
    procedure   updateSingleAabb (const colObj:btCollisionObject);
    procedure   updateAabbs      ;virtual;
    procedure   setDebugDrawer   (const debugDrawer:btIDebugDraw);virtual;
    function    getDebugDrawer   :btIDebugDraw;virtual;
    procedure   debugDrawWorld   ;virtual;
    procedure   debugDrawObject  (const worldTransform:btTransform; const shape:btCollisionShape; const color:btVector3);virtual;
    function    getNumCollisionObjects:integer;
    /// rayTest performs a raycast on all objects in the btCollisionWorld, and calls the resultCallback
    /// This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback.
    procedure   rayTest          (const  rayFromWorld,rayToWorld : btVector3 ; const resultCallback : btCW_RayResultCallback);virtual;
    ///  performs a swept convex cast on all objects in the btCollisionWorld, and calls the resultCallback
    /// This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.
    procedure   convexSweepTest  (const castShape : btConvexShape ; const convexFromWorld, convexToWorld : btTransform ; const resultCallback : btCW_ConvexResultCallback ;  const allowedCcdPenetration:btScalar=0);
    ///contactTest performs a discrete collision test between colObj against all objects in the btCollisionWorld, and calls the resultCallback.
    ///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
    procedure   contactTest      (const colObj : btCollisionObject ;  const resultCallback : btCW_ContactResultCallback);
    ///contactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
    ///it reports one or more contact points (including the one with deepest penetration)
    procedure   contactPairTest  (const colObjA, colObjB : btCollisionObject ; const resultCallback : btCW_ContactResultCallback);
    /// rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
    /// In a future implementation, we consider moving the ray test as a virtual method in btCollisionShape.
    /// This allows more customization.
    class procedure  rayTestSingle(const rayFromTrans,rayToTrans : btTransform ;const  collisionObject : btCollisionObject;const collisionShape : btCollisionShape;const colObjWorldTransform : btTransform ; const resultCallback: btCW_RayResultCallback);
    /// objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally by rayTest.
    class procedure  objectQuerySingle           (const castShape : btConvexShape ; const convexFromTrans,convexToTrans:btTransform ;const collisionObject : btCollisionObject;const collisionShape : btCollisionShape;
                                                  const colObjWorldTransform : btTransform; const resultCallback : btCW_ConvexResultCallback ; const allowedPenetration : btScalar);
    procedure addCollisionObject                 (const collisionObject : btCollisionObject ;const collisionFilterGroup:btCollisionFilterGroupSet=[btcfgDefaultFilter];const collisionFilterMask : btCollisionFilterGroupSet=btcfgAllFilter);
    function  getCollisionObjectArray            : btCollisionObjectArray;
    function  getCollisionObject                 (const i:integer) : btCollisionObject;
    procedure removeCollisionObject              (const collisionObject:btCollisionObject);virtual;
    procedure performDiscreteCollisionDetection  ; virtual;
    function  getDispatchInfo                    : btDispatcherInfo;
    function  getDispatchInfoP                   : PbtDispatcherInfo;
    function  getForceUpdateAllAabbs             : boolean;
    procedure setForceUpdateAllAabbs             (const forceUpdateAllAabbs:boolean);
  end;




//Functions, operators

   { btBroadphasePairSortPredicate }
   function btBroadphasePairSortPredicate(const a,b: btBroadphasePair):NativeInt;

   //by default, Bullet will use this near callback
   procedure defaultNearCallback(const  collisionPair:PbtBroadphasePair; const dispatcher:btCollisionDispatcher; var dispatchInfo:btDispatcherInfo);
   operator= (const a,b:btBroadphasePair) res:boolean;


implementation

{ btDispatcher }

function btDispatcher.getInternalManifoldPointer: btManifoldArray;
begin
  result := nil;
  abort;
  //should be virtual abstract
end;

{ btDispatcherInfo }

procedure btDispatcherInfo.Init;
begin
  m_timeStep        := 0;
  m_stepCount       := 0;
  m_dispatchFunc    := DISPATCH_DISCRETE;
  m_timeOfImpact    := 1;
  m_useContinuous   := false;
  m_debugDraw       := nil;
  m_enableSatConvex := false;
  m_enableSPU       := true;
  m_useEpa          := true;
  m_stackAllocator  := nil;
  m_allowedCcdPenetration               := 0.04;
  m_useConvexConservativeDistanceUtil   := false;
  m_convexConservativeDistanceThreshold := 0;
end;


{ btManifoldResult }

constructor btManifoldResult.Create(const body0, body1: btCollisionObject);
begin
  inherited Create;
  m_body0 := body0;
  m_body1 := body1;
  m_rootTransA := body0.getWorldTransformP^;
  m_rootTransB := body1.getWorldTransformP^;
end;

procedure btManifoldResult.setPersistentManifold(manifoldPtr: btPersistentManifold);
begin
  m_manifoldPtr := manifoldPtr;
end;

function btManifoldResult.getPersistentManifold: btPersistentManifold;
begin
  Result:=m_manifoldPtr;
end;

procedure btManifoldResult.setShapeIdentifiersA(const partId0, index0: integer);
begin
  m_partId0 := partId0;
  m_index0  := index0;
end;

procedure btManifoldResult.setShapeIdentifiersB(const partId1, index1: integer);
begin
  m_partId1 := partId1;
  m_index1  := index1;
end;

function calculateCombinedFriction(const body0,body1:btCollisionObject):btScalar;FOS_INLINE;
var friction,MAX_FRICTION : btScalar;
begin
  friction      := body0.getFriction * body1.getFriction;
  MAX_FRICTION  := 10;
  if (friction < -MAX_FRICTION) then friction := -MAX_FRICTION;
  if (friction > MAX_FRICTION)  then friction := MAX_FRICTION;
  Result:=friction;
end;

function calculateCombinedRestitution(const body0,body1:btCollisionObject):btScalar;FOS_INLINE;
begin
  Result := body0.getRestitution * body1.getRestitution;
end;


procedure btManifoldResult.addContactPoint(const normalOnBInWorld, pointInWorld: btVector3; const depth: btScalar);
var  isSwapped            : Boolean;
     pointA,localA,localB : btVector3;
     newPt                : btOManifoldPoint;
     insertIndex          : integer;
     obj0,obj1            : btCollisionObject;
begin
  //order in manifold needs to match
  btAssert(assigned(m_manifoldPtr));
  if (depth > m_manifoldPtr.getContactBreakingThreshold) then exit;

  isSwapped := btCollisionObject(m_manifoldPtr.getBody0) <> m_body0; //TODO FOSCHECK
  pointA := pointInWorld + normalOnBInWorld * depth;

  if (isSwapped) then begin
    localA := m_rootTransB.invXform(pointA );
    localB := m_rootTransA.invXform(pointInWorld);
  end else begin
    localA := m_rootTransA.invXform(pointA );
    localB := m_rootTransB.invXform(pointInWorld);
  end;


  newPt.Init(localA,localB,normalOnBInWorld,depth);
  newPt.m_positionWorldOnA := pointA;
  newPt.m_positionWorldOnB := pointInWorld;

  insertIndex := m_manifoldPtr.getCacheEntry(@newPt);

  newPt.m_combinedFriction    := calculateCombinedFriction(m_body0,m_body1);
  newPt.m_combinedRestitution := calculateCombinedRestitution(m_body0,m_body1);

  //BP mod, store contact triangles.
  if (isSwapped) then begin
    newPt.m_partId0 := m_partId1;
    newPt.m_partId1 := m_partId0;
    newPt.m_index0  := m_index1;
    newPt.m_index1  := m_index0;
  end else begin
    newPt.m_partId0 := m_partId0;
    newPt.m_partId1 := m_partId1;
    newPt.m_index0  := m_index0;
    newPt.m_index1  := m_index1;
  end;
  ///@todo, check this for any side effects
  if (insertIndex >= 0) then begin
    //const btManifoldPoint& oldPoint = m_manifoldPtr->getContactPoint(insertIndex);
    m_manifoldPtr.replaceContactPoint(@newPt,insertIndex);
  end else begin
    insertIndex := m_manifoldPtr.addManifoldPoint(@newPt);
  end;
  //User can override friction and/or restitution
  if (assigned(gContactAddedCallback) AND
    //and if either of the two bodies requires custom material
      ((CF_CUSTOM_MATERIAL_CALLBACK in m_body0.getCollisionFlags) OR
       (CF_CUSTOM_MATERIAL_CALLBACK in m_body1.getCollisionFlags))) then begin
    //experimental feature info, for per-triangle material etc.
    if isSwapped then begin
      obj0 :=  m_body1;
      obj1 :=  m_body0;
    end else begin
      obj1 :=  m_body1;
      obj0 :=  m_body0;
    end;
    gContactAddedCallback(m_manifoldPtr.getContactPointP(insertIndex),obj0,newPt.m_partId0,newPt.m_index0,obj1,newPt.m_partId1,newPt.m_index1);
  end;
end;

procedure btManifoldResult.refreshContactPoints;
var swapped : boolean;
begin
  //btAssert JUMPER
  btAssert(assigned(m_manifoldPtr));
  if (m_manifoldPtr.getNumContacts=0) then exit;
  swapped := m_manifoldPtr.getBody0 <> pointer(m_body0);
  if swapped then begin // isSwapped //TODO FOSCHECK
    m_manifoldPtr.refreshContactPoints(m_rootTransB,m_rootTransA);
  end else begin
    m_manifoldPtr.refreshContactPoints(m_rootTransA,m_rootTransB);
  end;
end;

function btManifoldResult.getBody0Internal: btCollisionObject;
begin
  result:=m_body0;
end;

function btManifoldResult.getBody1Internal: btCollisionObject;
begin
  result:=m_body1;
end;


{ btCollisionObject }
{$HINTS OFF}
function btCollisionObject.checkCollideWithOverride(const x: btCollisionObject): boolean;
begin
  result:=True;
end;
{$HINTS ON}

function btCollisionObject.mergesSimulationIslands: Boolean;
begin
  ///static objects, kinematic and object without contact response don't merge islands
 result := [CF_STATIC_OBJECT,CF_KINEMATIC_OBJECT,CF_NO_CONTACT_RESPONSE] * m_collisionFlags = [];
 // return  ((m_collisionFlags & (CF_STATIC_OBJECT | CF_KINEMATIC_OBJECT | CF_NO_CONTACT_RESPONSE) )==0);
end;

function btCollisionObject.getAnisotropicFriction: btVector3;
begin
  result:=m_anisotropicFriction;
end;

procedure btCollisionObject.setAnisotropicFriction(const anisotropicFriction: btVector3);
begin
  m_anisotropicFriction    := anisotropicFriction;
  m_hasAnisotropicFriction := (anisotropicFriction[0] <> 1) or (anisotropicFriction[1]<>1) or (anisotropicFriction[2]<>1);
end;

function btCollisionObject.hasAnisotropicFriction: boolean;
begin
  result:=m_hasAnisotropicFriction;
end;

procedure btCollisionObject.setContactProcessingThreshold(const contactProcessingThreshold: btScalar);
begin
   m_contactProcessingThreshold := contactProcessingThreshold;
end;

function btCollisionObject.getContactProcessingThreshold: btScalar;
begin
  Result:=m_contactProcessingThreshold;
end;

function btCollisionObject.isStaticObject: boolean;
begin
  Result:= CF_STATIC_OBJECT in m_collisionFlags;
end;

function btCollisionObject.isKinematicObject: boolean;
begin
  Result:= CF_KINEMATIC_OBJECT in m_collisionFlags;
end;

function btCollisionObject.isStaticOrKinematicObject: boolean;
begin
  Result:= ([CF_KINEMATIC_OBJECT,CF_STATIC_OBJECT] * m_collisionFlags) <> [];
end;

function btCollisionObject.hasContactResponse: boolean;
begin
  Result := not (CF_NO_CONTACT_RESPONSE in m_collisionFlags);
end;

constructor btCollisionObject.Create;
begin
  m_anisotropicFriction.InitSame(1);
  m_hasAnisotropicFriction     :=   false;
  m_contactProcessingThreshold := BT_LARGE_FLOAT;
  m_broadphaseHandle           := nil;
  m_collisionShape             := nil;
  m_extensionPointer           := nil;
  m_rootCollisionShape         := nil;
  m_collisionFlags             := [CF_STATIC_OBJECT];
  m_islandTag1                 := -1;
  m_companionId                := -1;
  m_activationState1           := btas_ACTIVE_TAG;
  m_deactivationTime           := 0;
  m_friction                   := 0.5;
  m_restitution                := 0;
  m_internalType               := CO_COLLISION_OBJECT;
  m_userObjectPointer          := nil;
  m_hitFraction                := 1;
  m_ccdSweptSphereRadius       := 0;
  m_ccdMotionThreshold         := 0;
  m_checkCollideWith           := false;
  m_worldTransform.setIdentity;
end;

destructor btCollisionObject.Destroy;
begin
  inherited Destroy;
end;

procedure btCollisionObject.setCollisionShape(const collisionShape: btCollisionShape);
begin
  m_collisionShape     := collisionShape;
  m_rootCollisionShape := collisionShape;
end;

function btCollisionObject.getCollisionShape: btCollisionShape;
begin
  result:=m_collisionShape;
end;

function btCollisionObject.getRootCollisionShape: btCollisionShape;
begin
  result:=m_rootCollisionShape;
end;

procedure btCollisionObject.internalSetTemporaryCollisionShape(const collisionShape: btCollisionShape);
begin
  m_collisionShape := collisionShape;
end;

function btCollisionObject.internalGetExtensionPointer: Pointer;
begin
  Result:=m_extensionPointer;
end;

procedure btCollisionObject.internalSetExtensionPointer(const pointer: Pointer);
begin
  m_extensionPointer := pointer;
end;

function btCollisionObject.getActivationState: btActivationState;
begin
  result:=m_activationState1;
end;

procedure btCollisionObject.setActivationState(const newState: btActivationState);
begin
  if ((m_activationState1 <> btas_DISABLE_DEACTIVATION) AND (m_activationState1 <> btas_DISABLE_SIMULATION)) then begin
    m_activationState1 := newState;
  end;
end;

procedure btCollisionObject.setDeactivationTime(const time: btScalar);
begin
  m_deactivationTime:=time;
end;

function btCollisionObject.getDeactivationTime: btScalar;
begin
  result:=m_deactivationTime;
end;

procedure btCollisionObject.forceActivationState(const newState: btActivationState);
begin
  m_activationState1 := newState;
end;

procedure btCollisionObject.activate(const forceActivation: boolean);
begin
//  	if (forceActivation || !(m_collisionFlags & (CF_STATIC_OBJECT|CF_KINEMATIC_OBJECT)))
  if (forceActivation OR ((m_collisionFlags * [CF_STATIC_OBJECT,CF_KINEMATIC_OBJECT] = []))) then begin
    setActivationState(btas_ACTIVE_TAG);
    m_deactivationTime := 0;
  end;
end;

function btCollisionObject.isActive: boolean;
begin
  result:= (m_activationState1 <> btas_ISLAND_SLEEPING) AND (m_activationState1 <> btas_DISABLE_SIMULATION);
end;

procedure btCollisionObject.setRestitution(const rest: btScalar);
begin
  m_restitution := rest;
end;

function btCollisionObject.getRestitution: btScalar;
begin
  result:=m_restitution;
end;

procedure btCollisionObject.setFriction(const fric: btScalar);
begin
  m_friction:=fric;
end;

function btCollisionObject.getFriction: btScalar;
begin
  result:=m_friction;
end;

function btCollisionObject.getInternalType: btCollisionObjectTypes;
begin
  result:=m_internalType;
end;

function btCollisionObject.getWorldTransformP: PbtTransform;
begin
  result := @m_worldTransform;
end;

procedure btCollisionObject.setWorldTransform(const worldTrans: btTransform);
begin
  m_worldTransform:=worldTrans;
end;

function btCollisionObject.getBroadphaseHandle: btBroadphaseProxy;
begin
  result := m_broadphaseHandle;
end;

procedure btCollisionObject.setBroadphaseHandle(const handle: btBroadphaseProxy);
begin
  m_broadphaseHandle := handle;
end;

function btCollisionObject.getInterpolationWorldTransformP: PbtTransform;
begin
  result:=@m_interpolationWorldTransform;
end;

procedure btCollisionObject.setInterpolationWorldTransform( const worldTrans: btTransform);
begin
  m_interpolationWorldTransform:=worldTrans;
end;

procedure btCollisionObject.setInterpolationLinearVelocity(const linvel: btVector3);
begin
  m_interpolationLinearVelocity := linvel;
end;

procedure btCollisionObject.setInterpolationAngularVelocity(const angvel: btVector3);
begin
  m_interpolationAngularVelocity:=angvel;
end;

function btCollisionObject.getInterpolationLinearVelocityP: PbtVector3;
begin
  result := @m_interpolationLinearVelocity;
end;

function btCollisionObject.getInterpolationAngularVelocityP: PbtVector3;
begin
  result := @m_interpolationAngularVelocity;
end;

function btCollisionObject.getIslandTag: integer;
begin
  result:=m_islandTag1;
end;

procedure btCollisionObject.setIslandTag(const tag: integer);
begin
  m_islandTag1:=tag;
end;

function btCollisionObject.getCompanionId: integer;
begin
  result := m_companionId;
end;

procedure btCollisionObject.setCompanionId(const id: integer);
begin
  m_companionId:=id;
end;

function btCollisionObject.getHitFraction: btScalar;
begin
  result:=m_hitFraction;
end;

procedure btCollisionObject.setHitFraction(const hitFraction: btScalar);
begin
  m_hitFraction:=hitFraction;
end;

function btCollisionObject.getCollisionFlags: btCollisionFlagSet;
begin
  result:=m_collisionFlags;
end;

procedure btCollisionObject.setCollisionFlags(const flags: btCollisionFlagSet);
begin
  m_collisionFlags:=flags;
end;

function btCollisionObject.getCcdSweptSphereRadius: btScalar;
begin
  result:=m_ccdSweptSphereRadius;
end;

procedure btCollisionObject.setCcdSweptSphereRadius(const radius: btScalar);
begin
  m_ccdSweptSphereRadius:=radius;
end;

function btCollisionObject.getCcdMotionThreshold: btScalar;
begin
  result:=m_ccdMotionThreshold;
end;

function btCollisionObject.getCcdSquareMotionThreshold: btScalar;
begin
  result:=m_ccdMotionThreshold*m_ccdMotionThreshold;
end;

procedure btCollisionObject.setCcdMotionThreshold(const ccdMotionThreshold: btScalar);
begin
   m_ccdMotionThreshold := ccdMotionThreshold;
end;

function btCollisionObject.getUserPointer: Pointer;
begin
  result:=m_userObjectPointer;
end;

procedure btCollisionObject.setUserPointer(const userPointer: Pointer);
begin
  m_userObjectPointer:=userPointer;
end;

function btCollisionObject.checkCollideWith(const co: btCollisionObject): boolean;
begin
  if (m_checkCollideWith) then begin
    result:=checkCollideWithOverride(co);
  end else begin
    Result:=true;
  end;
end;

{ btHashedOverlappingPairCache }
//var gTCount:integer=0;

function btHashedOverlappingPairCache.internalAddPair(proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair;
var
  proxyId1 : longint;
  proxyId2 : longint;
  hash     : integer;
  pair     : PbtBroadphasePair;
  idx      : integer;
  //new_pair : btBroadphasePair;
  count    : LongWord;
  //idx      : longword;
  oldCapacity: LongWord;
  newCapacity: LongWord;
begin
  if(proxy0.m_uniqueId > proxy1.m_uniqueId) then begin
    btSwap(TObject(proxy0),TObject(proxy1));
  end;
  proxyId1 := proxy0.getUid;
  proxyId2 := proxy1.getUid;
  hash := integer(getHash(cardinal(proxyId1), cardinal(proxyId2)) and (m_overlappingPairArray.Capacity-1));
  idx := internalFindPair(proxy0, proxy1, hash);
  if idx<>-1 then begin
    pair := m_overlappingPairArray[idx];
    exit(nil);
  end;
  count        := m_overlappingPairArray.Length;
  oldCapacity  := m_overlappingPairArray.Capacity;

  pair := m_overlappingPairArray.push_new_no_init;

  //this is where we add an actual pair, so also call the 'ghost'
  if assigned(m_ghostPairCallback) then begin
    m_ghostPairCallback.addOverlappingPair(proxy0,proxy1);
  end;
  newCapacity := m_overlappingPairArray.capacity;
  if (oldCapacity < newCapacity) then begin
    growTables;
    hash := integer(getHash(cardinal(proxyId1), cardinal(proxyId2)) and (m_overlappingPairArray.Capacity-1)); //hash with new capacity
  end;
  pair^.init(proxy0,proxy1);
  m_next[count]^     := m_hashTable[hash]^;
  m_hashTable[hash]^ := count;
  result := pair;
end;

procedure btHashedOverlappingPairCache.growTables;
var
  newCapacity: integer;
  curHashtableSize: LongWord;
  i: Integer;
  pair: PbtBroadphasePair;
  proxyId1: longint;
  proxyId2: longint;
  hashValue: LongInt;
begin
  newCapacity := m_overlappingPairArray.capacity;
  if (m_hashTable.Length < newCapacity) then begin
    //grow hashtable and next table
    curHashtableSize := m_hashTable.Length;
    m_hashTable.Setlength(newCapacity);
    m_next.Setlength(newCapacity);
    for i:=0 to newCapacity-1 do begin
      m_hashTable[i]^ := BT_NULL_PAIR;
    end;
    for i:=0 to newCapacity-1 do begin //TODO FOS: two loops faster then one ?
      m_next[i]^ := BT_NULL_PAIR;
    end;
    for i:=0 to curHashtableSize-1 do begin
      pair := m_overlappingPairArray.A[i];
      proxyId1 := pair^.m_pProxy0.getUid;
      proxyId2 := pair^.m_pProxy1.getUid;
//      hashValue = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1),static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity()-1)); // New hash value with new mask
      hashValue := integer(getHash(cardinal(proxyId1),cardinal(proxyId2)) and (m_overlappingPairArray.Capacity-1)); // New hash value with new mask
      m_next[i]^ := m_hashTable[hashValue]^;
      m_hashTable[hashValue]^ := i;
    end;
  end;
end;

function btHashedOverlappingPairCache.equalsPair(const pair: btBroadphasePair; const proxyId1, proxyId2: integer): boolean;
begin
  Result:= (pair.m_pProxy0.getUid = proxyId1) AND (pair.m_pProxy1.getUid = proxyId2);
end;

function btHashedOverlappingPairCache.getHash(const proxyId1, proxyId2: integer): integer;
var key:integer;
begin
  key := integer((proxyId1 or (proxyId2 SHL 16)));
  key := key + not(key SHL 15);
  key := key XOR  SarLongint(key,10);
  key := key +    (key SHL  3);
  key := key XOR  SarLongint(key,6);
  key := key + not(key SHL 11);
  key := key XOR  SarLongint(key,16);
  result := cardinal(key);
  //      SIMD_FORCE_INLINE       unsigned int getHash(unsigned int proxyId1, unsigned int proxyId2)
  //      {
  //              int key = static_cast<int>(((unsigned int)proxyId1) | (((unsigned int)proxyId2) <<16));
  //              Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
  //              This assumes proxyId1 and proxyId2 are 16-bit.  //
  //              key += ~(key << 15);
  //              key ^=  (key >> 10);
  //              key +=  (key << 3);
  //              key ^=  (key >> 6);
  //              key += ~(key << 11);
  //              key ^=  (key >> 16);
  //              return static_cast<unsigned int>(key);
  //      }
end;

function btHashedOverlappingPairCache.internalFindPair(const proxy0, proxy1: btBroadphaseProxy; const hash: integer): integer;
var
  proxyId1 : longint;
  proxyId2 : longint;
  index    : integer;
begin
  proxyId1 := proxy0.getUid;
  proxyId2 := proxy1.getUid;
  //#if 0 // wrong, 'equalsPair' use unsorted uids, copy-past devil striked again. Nat.
  //if (proxyId1 > proxyId2)
  //        btSwap(proxyId1, proxyId2);
  //#endif
  index := m_hashTable[hash]^;
  while( (index<>BT_NULL_PAIR) AND (equalsPair(m_overlappingPairArray[index]^, proxyId1, proxyId2) = false)) do begin
    index := m_next[index]^;
  end;
  if ( index = BT_NULL_PAIR ) then begin
    exit(-1);
  end;
  btAssert(index < m_overlappingPairArray.Length);
  result := index;
end;

function btHashedOverlappingPairCache.hasDeferredRemoval: boolean;
begin
  Result:=false;
end;

procedure btHashedOverlappingPairCache.setInternalGhostPairCallback(const ghostPairCallback: btOverlappingPairCallback);
begin
  m_ghostPairCallback := ghostPairCallback;
end;

procedure btHashedOverlappingPairCache.sortOverlappingPairs(const dispatcher: btDispatcher);
var tmpPairs:btBroadphasePairArray;
    i:integer;
begin
  ///need to keep hashmap in sync with pair address, so rebuild all
abort; //TODO DBG
  tmpPairs:=btBroadphasePairArray.create;
  try
    for i := 0 to m_overlappingPairArray.size-1 do begin
      tmpPairs.push_back(m_overlappingPairArray[i]^);
    end;
    for i := 0 to tmpPairs.size-1 do begin
      removeOverlappingPair(tmpPairs[i]^.m_pProxy0,tmpPairs[i]^.m_pProxy1,dispatcher);
    end;

    for i := 0  to m_next.size-1 do begin
      m_next[i]^ := BT_NULL_PAIR;
    end;

    tmpPairs.quickSort(@btBroadphasePairSortPredicate);
    for i:=0 to tmpPairs.size-1 do begin
      addOverlappingPair(tmpPairs[i]^.m_pProxy0,tmpPairs[i]^.m_pProxy1);
    end;
  finally
    tmpPairs.Free;
  end;
end;

constructor btHashedOverlappingPairCache.Create;
begin
  Inherited Create;
  m_overlapFilterCallback := nil;
  m_blockedForChanges     := false;
  m_ghostPairCallback     := nil;
  m_overlappingPairArray  := btBroadphasePairArray.create;
  m_hashTable             := btFOSAlignedIntegers.create;
  m_next                  := btFOSAlignedIntegers.Create;
  m_overlappingPairArray.Reserve(2); // int initialAllocatedSize = 2; ...
  growTables;
end;

destructor btHashedOverlappingPairCache.Destroy;
begin
  m_hashTable.Clear;
  m_next.Clear;
  m_overlappingPairArray.Clear;
  inherited Destroy;
end;

type

  { RemovePairCallback }

  { btRemovePairCallback }

  btRemovePairCallback=class(btOverlapCallback)
  private
    m_obsoleteProxy : btBroadphaseProxy;
  public
    constructor Create(const obsoleteProxy:btBroadphaseProxy);
    function    processOverlap (const pair: PbtBroadphasePair): boolean;override;
   end;

{ RemovePairCallback }
  constructor btRemovePairCallback.create(const obsoleteProxy: btBroadphaseProxy);
  begin
    m_obsoleteProxy:=obsoleteProxy;
  end;

  function btRemovePairCallback.processOverlap(const pair: PbtBroadphasePair): boolean;
  begin
    Result:= ((pair^.m_pProxy0 = m_obsoleteProxy) or (pair^.m_pProxy1 = m_obsoleteProxy));
  end;

procedure btHashedOverlappingPairCache.removeOverlappingPairsContainingProxy( const proxy0: btBroadphaseProxy; const dispatcher: btDispatcher);
var removeCallback:btRemovePairCallback;
begin
  removeCallback := btRemovePairCallback.Create(proxy0);
  processAllOverlappingPairs(removeCallback,dispatcher);
  removeCallback.Free;
end;

function btHashedOverlappingPairCache.removeOverlappingPair(proxy0, proxy1: btBroadphaseProxy; const dispatcher: btDispatcher):pointer;
var
  proxyId1  : longint;
  proxyId2  : longint;
  hash      : longint;
  pair,last : PbtBroadphasePair;
  userData  : Pointer;
  pairindex : integer;
  index,
  previous,
  lastHash,
  lastpairIndex : integer;
  debug_next    : integer;

begin
  //abort;
//  result:=nil;
 // exit;
  inc(gRemovePairs);
  if(proxy0.m_uniqueId>proxy1.m_uniqueId) then begin
    btSwap(TObject(proxy0),TObject(proxy1));
  end;
  proxyId1 := proxy0.getUid;
  proxyId2 := proxy1.getUid;

  hash := integer(getHash(cardinal(proxyId1), cardinal(proxyId2)) and (m_overlappingPairArray.Capacity-1));

  pairindex := internalFindPair(proxy0, proxy1, hash);
  if pairindex=-1 then begin
    exit(nil);
  end;
  pair := m_overlappingPairArray.A[pairindex];
  userData := pair^.int.m_internalInfo1;
  btAssert(pair^.m_pProxy0.getUid = proxyId1);
  btAssert(pair^.m_pProxy1.getUid = proxyId2);

  cleanOverlappingPair(pair,dispatcher);

  btAssert(pairIndex < m_overlappingPairArray.size);

  // Remove the pair from the hash table.
  index := m_hashTable[hash]^;
  btAssert(index <> BT_NULL_PAIR);

  previous := BT_NULL_PAIR;
  while (index <> pairIndex) do begin
    previous := index;
    index    := m_next[index]^;
  end;
  if (previous <> BT_NULL_PAIR) then begin
    btAssert(m_next[previous]^ = pairIndex);
    m_next[previous]^  := m_next[pairIndex]^;
  end else begin
    debug_next := m_next[pairIndex]^;
    m_hashTable[hash]^ := m_next[pairIndex]^;
  end;

  // We now move the last pair into spot of the
  // pair being removed. We need to fix the hash
  // table indices to support the move.
  lastPairIndex := m_overlappingPairArray.size-1;
  if assigned(m_ghostPairCallback) then begin
    m_ghostPairCallback.removeOverlappingPair(proxy0, proxy1,dispatcher);
  end;

  // If the removed pair is the last pair, we are done.
  if lastPairIndex = pairIndex then begin
    m_overlappingPairArray.pop_back;
    exit(userData);
  end;

  // Remove the last pair from the hash table.
  last := m_overlappingPairArray.A[lastPairIndex];
        // missing swap here too, Nat.
  lastHash := integer(getHash(cardinal(last^.m_pProxy0.getUid), cardinal(last^.m_pProxy1.getUid)) AND (m_overlappingPairArray.capacity-1));

  index := m_hashTable[lastHash]^;
  btAssert(index <> BT_NULL_PAIR);

  previous := BT_NULL_PAIR;
  while (index <> lastPairIndex) do begin
    previous := index;
    index    := m_next[index]^;
  end;

  if previous <> BT_NULL_PAIR then begin
    btAssert(m_next[previous]^ = lastPairIndex);
    m_next[previous]^ := m_next[lastPairIndex]^;
  end else begin
    m_hashTable[lastHash]^ := m_next[lastPairIndex]^;
  end;

  // Copy the last pair into the remove pair's spot.
  m_overlappingPairArray[pairIndex]^ := m_overlappingPairArray[lastPairIndex]^;

  // Insert the last pair into the hash table
  m_next[pairIndex]^     := m_hashTable[lastHash]^;
  m_hashTable[lastHash]^ := pairIndex;

  m_overlappingPairArray.pop_back;
  Result := userData;
end;

function btHashedOverlappingPairCache.needsBroadphaseCollision(const proxy0, proxy1: btBroadphaseProxy): boolean;
begin
  if assigned(m_overlapFilterCallback) then begin
    exit(m_overlapFilterCallback.needBroadphaseCollision(proxy0,proxy1));
  end;
  result := ((proxy0.m_collisionFilterGroup * proxy1.m_collisionFilterMask)<> []) AND ((proxy1.m_collisionFilterGroup * proxy0.m_collisionFilterMask)<>[]);
end;

function btHashedOverlappingPairCache.addOverlappingPair(const proxy0,proxy1: btBroadphaseProxy): PbtBroadphasePair;
begin
  inc(gAddedPairs);
  if (not needsBroadphaseCollision(proxy0,proxy1)) then exit(nil);
  Result:=internalAddPair(proxy0,proxy1);
end;

type

  { CleanPairCallback }

  btCleanPairCallback=class(btOverlapCallback)
  private
    m_cleanProxy : btBroadphaseProxy;
    m_pairCache  : btOverlappingPairCache;
    m_dispatcher : btDispatcher;
  public
    constructor create(const cleanProxy:  btBroadphaseProxy;const pairCache:btOverlappingPairCache;const dispatcher:btDispatcher);
    function    processOverlap (const pair: PbtBroadphasePair): boolean;override;
   end;

  { CleanPairCallback }

  constructor btCleanPairCallback.create(const cleanProxy: btBroadphaseProxy; const pairCache: btOverlappingPairCache; const dispatcher: btDispatcher);
  begin
    m_cleanProxy := cleanProxy;
    m_pairCache  := pairCache;
    m_dispatcher := dispatcher;
  end;

  function btCleanPairCallback.processOverlap(const pair: PbtBroadphasePair): boolean;
  begin
    if ((pair^.m_pProxy0 = m_cleanProxy) OR (pair^.m_pProxy1 = m_cleanProxy)) then begin
      m_pairCache.cleanOverlappingPair(pair,m_dispatcher);
    end;
    result:=false;
  end;


procedure btHashedOverlappingPairCache.cleanProxyFromPairs(const proxy: btBroadphaseProxy; const dispatcher: btDispatcher);
var cleanPairs:btCleanPairCallback;
begin
  cleanPairs:=btCleanPairCallback.Create(proxy,self,dispatcher);
  processAllOverlappingPairs(cleanPairs,dispatcher);
  cleanPairs.Free;
end;

procedure btHashedOverlappingPairCache.processAllOverlappingPairs(const ocb: btOverlapCallback; const dispatcher: btDispatcher);
var i:integer;
  pair:PbtBroadphasePair;
begin
//      printf("m_overlappingPairArray.size()=%d\n",m_overlappingPairArray.size());
  i:=0;
  while (i<m_overlappingPairArray.size) do begin
    pair := m_overlappingPairArray.A[i];
    if (pair^.m_pProxy0.getUid=4) and (pair^.m_pProxy1.getUid=11) then begin
      i:=i;
      gFOS_Debug_Step := true;
    end;
    if ocb.processOverlap(pair) then begin
      removeOverlappingPair(pair^.m_pProxy0,pair^.m_pProxy1,dispatcher);
      dec(gOverlappingPairs);
    end else begin
      inc(i);
    end;
  end;
end;

//
//function btHashedOverlappingPairCache.getOverlappingPairArrayPtr: PbtBroadphasePair;
//begin
//  Result := m_overlappingPairArray.A[0];
//end;

function btHashedOverlappingPairCache.getOverlappingPairArray: btBroadphasePairArray;
begin
  Result:= m_overlappingPairArray;
end;

procedure btHashedOverlappingPairCache.cleanOverlappingPair(const pair: PbtBroadphasePair; const dispatcher: btDispatcher);
begin
  if assigned(pair^.m_algorithm) then begin
    dispatcher.freeCollisionAlgorithm(pair^.m_algorithm); //FOSCHECK
    pair^.m_algorithm:=nil;
  end;
end;

function btHashedOverlappingPairCache.findPair(proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair;
var
  proxyId1 : integer;
  proxyId2 : integer;
  hash     : integer;
  index    : integer;
begin
  inc(gFindPairs);
  if(proxy0.m_uniqueId > proxy1.m_uniqueId) then begin
    btSwap(Tobject(proxy0),TObject(proxy1));
  end;
  proxyId1 := proxy0.getUid;
  proxyId2 := proxy1.getUid;

//  hash = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1), static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity()-1));
  hash := integer(getHash(cardinal(proxyId1), cardinal(proxyId2)) and (m_overlappingPairArray.Capacity-1));

  if (hash >= m_hashTable.Length) then exit(nil);
  index := m_hashTable[hash]^;

  while ((index <> BT_NULL_PAIR)  AND (equalsPair(m_overlappingPairArray[index]^, proxyId1, proxyId2)=false)) do begin
    index := m_next[index]^;
  end;

  if (index = BT_NULL_PAIR) then exit(nil);
  btAssert(index < m_overlappingPairArray.Length);
  result := m_overlappingPairArray.A[index];

end;

function btHashedOverlappingPairCache.GetCount: integer;
begin
  result:=m_overlappingPairArray.Length;
end;

function btHashedOverlappingPairCache.getOverlapFilterCallback: btOverlapFilterCallback;
begin
  Result:=m_overlapFilterCallback;
end;

procedure btHashedOverlappingPairCache.setOverlapFilterCallback(const oc: btOverlapFilterCallback);
begin
  m_overlapFilterCallback:=oc;
end;

function btHashedOverlappingPairCache.getNumOverlappingPairs: integer;
begin
  Result:=m_overlappingPairArray.Length;
end;

function compareBroadphasePairs(const a,b:btBroadphasePair):NativeInt; // DUE To possible generics overloading operator bug in fpc
begin
  if a=b then begin
    result :=  0
  end else begin
    result := -1;
  end;
  //result := a=b;
end;


{ btSortedOverlappingPairCache }

constructor btSortedOverlappingPairCache.Create;
begin
  m_blockedForChanges     := false;
  m_hasDeferredRemoval    := true;
  m_overlapFilterCallback := nil;
  m_ghostPairCallback     := nil;
  m_overlappingPairArray  := btBroadphasePairArray.create;
  m_overlappingPairArray.reserve(2); //   int initialAllocatedSize=2
  m_overlappingPairArray.SetComparefunctionT(@compareBroadphasePairs);
end;

destructor btSortedOverlappingPairCache.Destroy;
begin
  m_overlappingPairArray.Free;
  inherited Destroy;
end;

procedure btSortedOverlappingPairCache.processAllOverlappingPairs(const ocb: btOverlapCallback; const dispatcher: btDispatcher);
var i:integer;
    pair:PbtBroadphasePair;
begin
   i:=0;
   while(i < m_overlappingPairArray.size) do begin
    pair := m_overlappingPairArray.A[i];
    if ocb.processOverlap(pair) then begin
      cleanOverlappingPair(pair,dispatcher);
      pair^.m_pProxy0 := nil;
      pair^.m_pProxy1 := nil;
      m_overlappingPairArray.swap(i,m_overlappingPairArray.size-1);
      m_overlappingPairArray.pop_back;
      dec(gOverlappingPairs);
    end else begin
      inc(i);
    end;
  end;
end;

function btSortedOverlappingPairCache.removeOverlappingPair(proxy0, proxy1: btBroadphaseProxy; const dispatcher: btDispatcher):Pointer;
var fndPair   : btBroadphasePair;
    findIndex : integer;
    pair      : PbtBroadphasePair;
    userData  : Pointer;
begin
abort;
  if not hasDeferredRemoval then begin
    fndPair.init(proxy0,proxy1);
    findIndex := m_overlappingPairArray.findLinearSearch(fndPair);
    if findIndex < m_overlappingPairArray.size then begin
      dec(gOverlappingPairs);
      pair := m_overlappingPairArray.A[findIndex];
      userData := pair^.int.m_internalInfo1;
      cleanOverlappingPair(pair,dispatcher);
      if assigned(m_ghostPairCallback) then begin
        m_ghostPairCallback.removeOverlappingPair(proxy0, proxy1,dispatcher);
      end;
      m_overlappingPairArray.swap(findIndex,m_overlappingPairArray.capacity-1);
      m_overlappingPairArray.pop_back;
      exit(userData);
    end;
  end;
  Result := nil;
end;

procedure btSortedOverlappingPairCache.cleanOverlappingPair(const  pair: PbtBroadphasePair; const dispatcher: btDispatcher);
begin
abort;
  if assigned(pair^.m_algorithm) then begin
    dispatcher.freeCollisionAlgorithm(pair^.m_algorithm);  // FOSCHECK
    pair^.m_algorithm:=nil;
    dec(gRemovePairs);
  end;
end;

function btSortedOverlappingPairCache.addOverlappingPair(const proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair;
var pair:PbtBroadphasePair;
begin
  //don't add overlap with own
  btAssert(proxy0 <> proxy1);
  if (not needsBroadphaseCollision(proxy0,proxy1)) then begin
    exit(nil);
  end;

 // void* mem = &m_overlappingPairArray.expandNonInitializing();
 // btBroadphasePair* pair = new (mem) btBroadphasePair(*proxy0,*proxy1);
  pair:=m_overlappingPairArray.push_new_no_init;
  pair^.init(proxy0,proxy1);

  inc(gOverlappingPairs);
  inc(gAddedPairs);

  if assigned(m_ghostPairCallback) then begin
    m_ghostPairCallback.addOverlappingPair(proxy0, proxy1);
  end;
  Result := pair;
end;

///this findPair becomes really slow. Either sort the list to speedup the query, or
///use a different solution. It is mainly used for Removing overlapping pairs. Removal could be delayed.
///we could keep a linked list in each proxy, and store pair in one of the proxies (with lowest memory address)
///Also we can use a 2D bitmap, which can be useful for a future GPU implementation
function btSortedOverlappingPairCache.findPair( proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair;
var tmpPair:btBroadphasePair;
    findIndex:integer;
begin
  if (not needsBroadphaseCollision(proxy0,proxy1)) then begin
    exit(nil);
  end;

  tmpPair.init(proxy0,proxy1);
  findIndex := m_overlappingPairArray.findLinearSearch(tmpPair);

  if (findIndex < m_overlappingPairArray.size) then begin
    //btAssert(it != m_overlappingPairSet.end());
    //btBroadphasePair* pair = &m_overlappingPairArray[findIndex];
    //return pair;
    result := (m_overlappingPairArray.A[findIndex]);
    exit;
  end;
  result:=nil;
end;

procedure btSortedOverlappingPairCache.cleanProxyFromPairs(const proxy: btBroadphaseProxy; const dispatcher: btDispatcher);
var cleanpairs:btCleanPairCallback;
begin
  cleanpairs := btCleanPairCallback.Create(proxy,self,dispatcher);
  processAllOverlappingPairs(cleanPairs,dispatcher);
  cleanpairs.Free;
end;

procedure btSortedOverlappingPairCache.removeOverlappingPairsContainingProxy(const proxy0: btBroadphaseProxy; const dispatcher: btDispatcher);
var removeCallback:btRemovePairCallback;
begin
  removecallback := btRemovePairCallback.Create(proxy0);
  processAllOverlappingPairs(removeCallback,dispatcher);
  removeCallback.Free;
end;

function btSortedOverlappingPairCache.needsBroadphaseCollision(const proxy0, proxy1: btBroadphaseProxy): boolean;
begin
  if assigned(m_overlapFilterCallback) then begin
    exit(m_overlapFilterCallback.needBroadphaseCollision(proxy0,proxy1));
  end;
  result := ((proxy0.m_collisionFilterGroup * proxy1.m_collisionFilterMask)<> []) AND ((proxy1.m_collisionFilterGroup * proxy0.m_collisionFilterMask)<>[]);
end;

function btSortedOverlappingPairCache.getOverlappingPairArray: btBroadphasePairArray;
begin
  Result := m_overlappingPairArray;
end;


function btSortedOverlappingPairCache.getNumOverlappingPairs: integer;
begin
  Result:=m_overlappingPairArray.Length;
end;

function btSortedOverlappingPairCache.getOverlapFilterCallback: btOverlapFilterCallback;
begin
  Result:=m_overlapFilterCallback;
end;

procedure btSortedOverlappingPairCache.setOverlapFilterCallback(const oc: btOverlapFilterCallback);
begin
   m_overlapFilterCallback:=oc;
end;

function btSortedOverlappingPairCache.hasDeferredRemoval: boolean;
begin
  result := m_hasDeferredRemoval;
end;

procedure btSortedOverlappingPairCache.setInternalGhostPairCallback(const ghostPairCallback: btOverlappingPairCallback);
begin
  m_ghostPairCallback:=ghostPairCallback;
end;

{$HINTS OFF}
procedure btSortedOverlappingPairCache.sortOverlappingPairs(const dispatcher: btDispatcher);
begin
  //should already be sorted
end;
{$HINTS ON}

{ btNullPairCache }

constructor btNullPairCache.create;
begin
  inherited;
  m_overlappingPairArray := btBroadphasePairArray.create;
end;

destructor btNullPairCache.destroy;
begin
  m_overlappingPairArray.Free;
  inherited destroy;
end;


function btNullPairCache.getOverlappingPairArray: btBroadphasePairArray;
begin
  Result := m_overlappingPairArray;
end;

function btNullPairCache.getNumOverlappingPairs: integer;
begin
  Result:=0;
end;

function btNullPairCache.hasDeferredRemoval: boolean;
begin
  Result:=true;
end;

{$HINTS OFF}
procedure btNullPairCache.cleanOverlappingPair(const pair: PbtBroadphasePair; const dispatcher: btDispatcher);
begin
  //empty
end;

procedure btNullPairCache.cleanProxyFromPairs(const proxy: btBroadphaseProxy; const dispatcher: btDispatcher);
begin
  //empty
end;

procedure btNullPairCache.setOverlapFilterCallback(const oc: btOverlapFilterCallback);
begin
  //empty
end;

procedure btNullPairCache.processAllOverlappingPairs(const ocb: btOverlapCallback; const dispatcher: btDispatcher);
begin
  //empty
end;

function btNullPairCache.findPair( proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair;
begin
  Result:=nil;
end;

procedure btNullPairCache.setInternalGhostPairCallback(const ghostPairCallback: btOverlappingPairCallback);
begin
  //empty
end;

function btNullPairCache.removeOverlappingPair( proxy0, proxy1: btBroadphaseProxy; const dispatcher: btDispatcher): Pointer;
begin
  result:=nil;
end;

function btNullPairCache.addOverlappingPair(const proxy0, proxy1: btBroadphaseProxy): PbtBroadphasePair;
begin
  Result:=nil;
end;

procedure btNullPairCache.removeOverlappingPairsContainingProxy(const proxy0: btBroadphaseProxy; const dispatcher: btDispatcher);
begin
  //empty
end;

procedure btNullPairCache.sortOverlappingPairs(const dispatcher: btDispatcher);
begin
 //empty
end;
{$HINTS ON}

{ btCollisionAlgorithmCreateFunc }


{ btCollisionDispatcher }

function btCollisionDispatcher.getDispatherFlags: btDispatcherFlagSet;
begin
  result := m_dispatcherFlags;
end;

procedure btCollisionDispatcher.setDispatcherFlags(const flags: btDispatcherFlagSet);
begin
//  m_dispatcherFlags:=[]; //BULLET m_dispatcherFlags = 0;
  m_dispatcherFlags:=flags;
end;

procedure btCollisionDispatcher.registerCollisionCreateFunc(const proxyType0, proxyType1: TbtBroadphaseNativeTypes; const createFunc: btCollisionAlgorithmCreateFunc);
begin
  m_doubleDispatch[proxyType0][proxyType1] := createFunc;
end;

function btCollisionDispatcher.getNumManifolds: integer;
begin
  Result := m_manifoldsPtr.Length;
end;

function btCollisionDispatcher.getManifoldByIndexInternal(const index: integer): btPersistentManifold;
begin
  Result:=m_manifoldsPtr[index]^;
end;

constructor btCollisionDispatcher.Create(const collisionConfiguration: btCollisionConfiguration);
var
  i: TbtBroadphaseNativeTypes;
  j: TbtBroadphaseNativeTypes;
begin
  m_dispatcherFlags        := [CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD];
  m_manifoldsPtr           := btManifoldArray.create;
  m_collisionConfiguration := collisionConfiguration;
  setNearCallback(@defaultNearCallback);
  m_collisionAlgorithmPoolAllocator := collisionConfiguration.getCollisionAlgorithmPool;
  m_persistentManifoldPoolAllocator := collisionConfiguration.getPersistentManifoldPool;
  for i:=low(TbtBroadphaseNativeTypes) to high(TbtBroadphaseNativeTypes) do begin
    for j:=low(TbtBroadphaseNativeTypes) to high(TbtBroadphaseNativeTypes) do begin
      m_doubleDispatch[i][j] := m_collisionConfiguration.getCollisionAlgorithmCreateFunc(i,j);
      btAssert(assigned(m_doubleDispatch[i][j]));
    end;
  end;
end;

destructor btCollisionDispatcher.Destroy;
begin
  m_manifoldsPtr.Clear;
  inherited;
end;

function btCollisionDispatcher.getNewManifold(const body0, body1: btCollisionObject): btPersistentManifold;
var
  contactBreakingThreshold:btScalar;
  contactProcessingThreshold:btScalar;
  manifold: btPersistentManifold;
begin
  inc(gNumManifold);
  //btAssert(gNumManifold < 65535);

  //optional relative contact breaking threshold, turned on by default (use setDispatcherFlags to switch off feature for improved performance)
  if (CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD in m_dispatcherFlags) then begin
    contactBreakingThreshold := btMin(body0.getCollisionShape.getContactBreakingThreshold(gContactBreakingThreshold) , body1.getCollisionShape.getContactBreakingThreshold(gContactBreakingThreshold))
  end else begin
    contactBreakingThreshold := gContactBreakingThreshold;
  end;
  contactProcessingThreshold := btMin(body0.getContactProcessingThreshold,body1.getContactProcessingThreshold);

  if (m_persistentManifoldPoolAllocator.getFreeCount>0) then begin
    manifold := btPersistentManifold(m_persistentManifoldPoolAllocator.CreatePoolInstance(btPersistentManifold));
  end else begin
     manifold := btPersistentManifold.Create;
  end;
  manifold.init(body0,body1,contactBreakingThreshold,contactProcessingThreshold);
  manifold.m_index1a := m_manifoldsPtr.Length;
  m_manifoldsPtr.push_back(manifold);
  Result:=manifold;
end;

procedure btCollisionDispatcher.releaseManifold(const manifold: btPersistentManifold);
var  findIndex,fix: LongInt;
     oooh:btPersistentManifold;
begin
  dec(gNumManifold);
  clearManifold(manifold);

  findIndex := manifold.m_index1a;
  btAssert(findIndex < m_manifoldsPtr.Length);
  m_manifoldsPtr.swap(findIndex,m_manifoldsPtr.Length-1);
  fix :=  m_manifoldsPtr[findIndex]^.m_index1a;
  btAssert(fix=m_manifoldsPtr.Length-1);
  m_manifoldsPtr[findIndex]^.m_index1a := findIndex;
  oooh := m_manifoldsPtr.pop_back^;

  btAssert(oooh=manifold);

  if m_persistentManifoldPoolAllocator.ValidPointer(manifold) then begin
    m_persistentManifoldPoolAllocator.FreePoolInstance(manifold);
  end else begin
    abort; // Check for non pooled objects / when / hwo ?
    manifold.Free;
  end;
end;

procedure btCollisionDispatcher.clearManifold(const manifold: btPersistentManifold);
begin
  manifold.clearManifold;
end;

function btCollisionDispatcher.findAlgorithm(const body0, body1: btCollisionObject; const sharedManifold: btPersistentManifold): btCollisionAlgorithm;
var ci:btCollisionAlgorithmConstructionInfo;
begin
  ci.m_dispatcher1 := self;
  ci.m_manifold    := sharedManifold;
  result           := m_doubleDispatch[body0.getCollisionShape.getShapeType][body1.getCollisionShape.getShapeType].CreateCollisionAlgorithm(ci,body0,body1);
end;

function btCollisionDispatcher.needsCollision(const body0, body1: btCollisionObject): Boolean;
begin
  btAssert(assigned(body0));
  btAssert(assigned(body1));
  result := true;

{$ifdef BT_DEBUG}
  //if (!(m_dispatcherFlags & btCollisionDispatcher::CD_STATIC_STATIC_REPORTED))
  //{
  //	//broadphase filtering already deals with this
  //	if ((body0->isStaticObject() || body0->isKinematicObject()) &&
  //		(body1->isStaticObject() || body1->isKinematicObject()))
  //	{
  //		m_dispatcherFlags |= btCollisionDispatcher::CD_STATIC_STATIC_REPORTED;
  //		printf("warning btCollisionDispatcher::needsCollision: static-static collision!\n");
  //	}
  //}
{$endif} //BT_DEBUG

  if (not body0.isActive) and (not body1.isActive) then begin
    result := false;
  end else
  if (not body0.checkCollideWith(body1)) then begin
     result :=  false;
  end;
end;

function btCollisionDispatcher.needsResponse(const body0, body1: btCollisionObject): Boolean;
begin
  //here you can do filtering                                             //no response between two static/kinematic bodies
  result:= (body0.hasContactResponse AND body1.hasContactResponse) AND ((NOT body0.isStaticOrKinematicObject) OR (NOT body1.isStaticOrKinematicObject));
end;


///interface for iterating all overlapping collision pairs, no matter how those pairs are stored (array, set, map etc)
///this is useful for the collision dispatcher.
type

  { btCollisionPairCallback }

  btCollisionPairCallback=class(btOverlapCallback)
    m_dispatchInfo : btDispatcherInfo;
    m_dispatcher   : btCollisionDispatcher;
  public
    procedure   init            (const dispatchInfo:btDispatcherInfo;const dispatcher:btCollisionDispatcher);
    function    opequals        (const other:btCollisionPairCallback):btCollisionPairCallback;
    function    processOverlap  (const pair: PbtBroadphasePair): boolean;override;
  end;

{ btCollisionPairCallback }

procedure btCollisionPairCallback.init(const dispatchInfo: btDispatcherInfo; const dispatcher: btCollisionDispatcher);
begin
  m_dispatchInfo := dispatchInfo;
  m_dispatcher   := dispatcher;
end;

function btCollisionPairCallback.opequals(const other: btCollisionPairCallback): btCollisionPairCallback;
begin
  m_dispatchInfo := other.m_dispatchInfo;
  m_dispatcher   := other.m_dispatcher;
  Result         := self;
end;

function btCollisionPairCallback.processOverlap(const pair: PbtBroadphasePair): boolean;
begin
  m_dispatcher.getNearCallback()(pair,m_dispatcher,m_dispatchInfo);
  Result:=false;
end;



procedure btCollisionDispatcher.dispatchAllCollisionPairs(const pairCache: btOverlappingPairCache; const dispatchInfo: btDispatcherInfo; const dispatcher: btDispatcher);
var collisionCallback:btCollisionPairCallback;
begin
  //m_blockedForChanges = true;
   collisionCallback:=btCollisionPairCallback.Create;
  collisionCallback.init(dispatchInfo,self);
  pairCache.processAllOverlappingPairs(collisionCallback,dispatcher);
  collisionCallback.Free;
  //m_blockedForChanges = false;
end;

procedure btCollisionDispatcher.setNearCallback(const nearCallback: btNearCallback);
begin
  m_nearCallback:=nearCallback;
end;

function btCollisionDispatcher.getNearCallback: btNearCallback;
begin
  result:=m_nearCallback;
end;

function btCollisionDispatcher.allocateCollisionAlgorithm(const alg: btCollisionAlgorithmClass): btCollisionAlgorithm;
begin
  result := alg.Create;
end;

procedure btCollisionDispatcher.freeCollisionAlgorithm(const alg: btCollisionAlgorithm);
begin
  alg.Free;
end;

procedure defaultNearCallback(const collisionPair: PbtBroadphasePair; const dispatcher: btCollisionDispatcher; var dispatchInfo: btDispatcherInfo);
var colObj0,colObj1:btCollisionObject;
    contactPointResult:btManifoldResult;
    toi:btScalar;
begin
  colObj0 := btCollisionObject(collisionPair^.m_pProxy0.m_clientObject);
  colObj1 := btCollisionObject(collisionPair^.m_pProxy1.m_clientObject);

  if (dispatcher.needsCollision(colObj0,colObj1)) then begin
  	//dispatcher will keep algorithms persistent in the collision pair
    if (not assigned(collisionPair^.m_algorithm)) then begin
      collisionPair^.m_algorithm := dispatcher.findAlgorithm(colObj0,colObj1);
    end;
    if (assigned(collisionPair^.m_algorithm)) then begin
      contactPointResult:=btManifoldResult.Create(colObj0,colObj1);
      try
        if (dispatchInfo.m_dispatchFunc = DISPATCH_DISCRETE) then begin
          //discrete collision detection query
          collisionPair^.m_algorithm.processCollision(colObj0,colObj1,dispatchInfo,contactPointResult);
        end else begin
          //continuous collision detection query, time of impact (toi)
          toi := collisionPair^.m_algorithm.calculateTimeOfImpact(colObj0,colObj1,dispatchInfo,contactPointResult);
          if (dispatchInfo.m_timeOfImpact > toi) then begin
            dispatchInfo.m_timeOfImpact := toi;
          end;
        end
      finally
        contactPointResult.Free;
      end;
    end
  end;
end;

function btCollisionDispatcher.getCollisionConfiguration: btCollisionConfiguration;
begin
  result:=m_collisionConfiguration;
end;

procedure btCollisionDispatcher.setCollisionConfiguration(const config: btCollisionConfiguration);
begin
  m_collisionConfiguration:=config;
end;

{ btCollisionAlgorithmConstructionInfo }

//procedure btCollisionAlgorithm.Init(const ci: btCollisionAlgorithmConstructionInfo);
//begin
//  m_dispatcher:=ci.m_dispatcher1;
//end;


{ btBroadphasePair }

procedure btBroadphasePair.Init;
begin
  m_pProxy0       := nil;
  m_pProxy1       := nil;
  m_algorithm     := nil;
  int.m_internalInfo1 := nil;
end;

procedure btBroadphasePair.Init(const other: PbtBroadphasePair);
begin
  m_pProxy0       := other^.m_pProxy0;
  m_pProxy1       := other^.m_pProxy1;
  m_algorithm     := other^.m_algorithm;
  int.m_internalInfo1 := other^.int.m_internalInfo1;
end;

procedure btBroadphasePair.Init(const proxy0, proxy1: btBroadphaseProxy);
begin
  //keep them sorted, so the std::set operations work
  if (proxy0.m_uniqueId < proxy1.m_uniqueId) then begin
      m_pProxy0 := proxy0;
      m_pProxy1 := proxy1;
  end else begin
      m_pProxy0 := proxy1;
      m_pProxy1 := proxy0;
  end;
  m_algorithm         := nil;
  int.m_internalInfo1 := nil;
end;

{ btBroadphasePairSortPredicate }

function btCollisionObjectCompare(const a, b: btCollisionObject): nativeint;
begin
  if a=b then exit(0);
  exit(-1);
  //result := a=b;
end;

function btBroadphasePairSortPredicate(const a, b: btBroadphasePair): nativeint;
var uidA0,uidB0,uidA1,uidB1:integer;
    res : boolean;
begin
  abort;
  if assigned(a.m_pProxy0) then uidA0:=a.m_pProxy0.m_uniqueId else uidA0:=-1;
  if assigned(b.m_pProxy0) then uidB0:=b.m_pProxy0.m_uniqueId else uidB0:=-1;
  if assigned(a.m_pProxy1) then uidA1:=a.m_pProxy1.m_uniqueId else uidA1:=-1;
  if assigned(b.m_pProxy1) then uidB1:=b.m_pProxy1.m_uniqueId else uidB1:=-1;
  res  := (uidA0 > uidB0) OR
             ( (a.m_pProxy0 = b.m_pProxy0) AND (uidA1 > uidB1) ) OR
             ( (a.m_pProxy0 = b.m_pProxy0) AND (a.m_pProxy1 = b.m_pProxy1) AND (assigned(a.m_algorithm) and (not assigned(b.m_algorithm))) );

  if res then exit(0);
  exit(-1);
  //return uidA0 > uidB0 ||
 	//(a.m_pProxy0 == b.m_pProxy0 && uidA1 > uidB1) ||
 	//(a.m_pProxy0 == b.m_pProxy0 && a.m_pProxy1 == b.m_pProxy1 && a.m_algorithm > b.m_algorithm);
end;

operator= (const a, b: btBroadphasePair) res:boolean;
begin
  res:=(a.m_pProxy0 = b.m_pProxy0) AND (a.m_pProxy1 = b.m_pProxy1);
end;

{ btSimpleBroadphaseProxy }


procedure btSimpleBroadphaseProxy.SetNextFree(const next: integer);
begin
  m_nextFree := next;
end;

function btSimpleBroadphaseProxy.GetNextFree: integer;
begin
  Result:=m_nextFree;
end;

constructor btSimpleBroadphaseProxy.Create;
begin
  inherited;
end;

{$HINTS OFF}
procedure btSimpleBroadphaseProxy.init(const minpt, maxpt: btVector3; const shapeType: TbtBroadphaseNativeTypes; const userPtr: Pointer; const collisionFilterGroup, collisionFilterMask: btCollisionFilterGroupSet; const multiSapProxy: pointer;const handleindex: cardinal);
begin
  Inherited init(minpt,maxpt,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy);
  m_handleId:=handleindex;
end;


procedure btBroadphaseInterface.resetPool(const dispatcher: btDispatcher);
begin

end;
{$HINTS ON}

{ btSimpleBroadphase }

function btSimpleBroadphase.allocHandle: integer;
var
  freeH: LongInt;
begin
  //.
  btAssert(m_numHandles < m_maxHandles);
  freeH             := m_firstFreeHandle;
  m_firstFreeHandle := m_pHandles[freeH]^.GetNextFree;
  inc(m_numHandles);
  if(freeH > m_LastHandleIndex) then begin
    m_LastHandleIndex := freeH;
  end;
  Result:=freeH;
end;

procedure btSimpleBroadphase.freeHandle(const proxy: btSimpleBroadphaseProxy);
var handle:integer;
begin
//  handle := PtrUInt(@proxy-@m_pHandles);
  handle := proxy.m_handleId;
  btAssert ((handle >= 0) AND (handle < m_maxHandles));
  if(handle = m_LastHandleIndex) then begin
    dec(m_LastHandleIndex);
  end;

  proxy.SetNextFree(m_firstFreeHandle);
  m_firstFreeHandle := handle;

  proxy.m_clientObject := nil;
  dec(m_numHandles);
end;

{$HINTS OFF}
procedure btSimpleBroadphase.resetPool(const dispatcher: btDispatcher);
begin
  // not yet (bullet)
end;
{$HINTS ON}

procedure btSimpleBroadphase.validate;
var
  i: Integer;
  j: Integer;
begin
  for i:=0 to m_numHandles-1 do begin
    for j:=i+1 to m_numHandles-1 do begin
      btAssert(m_pHandles[i] <> m_pHandles[j]);
    end;
  end;
end;

constructor btSimpleBroadphase.Create(const maxProxies: integer; const overlappingPairCache: btOverlappingPairCache);
var
  i: LongInt;
begin
  m_pairCache     := overlappingPairCache;
  m_ownsPairCache := false;
  m_invalidPair   := 0;

  if not assigned (overlappingPairCache) then begin
    m_pairCache     := btHashedOverlappingPairCache.Create;
    m_ownsPairCache := true;
  end;

  // allocate handles buffer and put all handles on free list
//  m_pHandlesRawPtr := btAlignedAlloc(sizeof(btSimpleBroadphaseProxy)*maxProxies,16);
  m_pHandles := btAlignedSimpleBroadphaseproxyArray.create;
  m_pHandles.Initialize(btSimpleBroadphaseProxy);
  m_pHandles.Setlength(maxProxies);
  m_maxHandles      := maxProxies;
  m_numHandles      := 0;
  m_firstFreeHandle := 0;
  m_LastHandleIndex := -1;
  for i := m_firstFreeHandle to maxProxies-1 do begin
    m_pHandles[i]^:=btSimpleBroadphaseProxy.Create;
    m_pHandles[i]^.SetNextFree(i + 1);
    m_pHandles[i]^.m_uniqueId := i+2;//any UID will do, we just avoid too trivial values (0,1) for debugging purposes
  end;
  m_pHandles[maxProxies - 1]^.SetNextFree(0);
end;

destructor btSimpleBroadphase.Destroy;
var
  i: Integer;
begin
  for i:=0 to m_pHandles.Length-1 do begin
    m_pHandles[i]^.free;
    m_pHandles[i]^:=nil;
  end;
  m_pHandles.Free;
  if m_ownsPairCache then begin
    m_pairCache.free;
  end;
end;

class function btSimpleBroadphase.aabbOverlap(const proxy0, proxy1: btSimpleBroadphaseProxy): boolean;
begin
  Result:= (proxy0.m_aabbMin[0] <= proxy1.m_aabbMax[0]) AND (proxy1.m_aabbMin[0] <= proxy0.m_aabbMax[0]) AND
  	   (proxy0.m_aabbMin[1] <= proxy1.m_aabbMax[1]) AND (proxy1.m_aabbMin[1] <= proxy0.m_aabbMax[1]) AND
  	   (proxy0.m_aabbMin[2] <= proxy1.m_aabbMax[2]) AND (proxy1.m_aabbMin[2] <= proxy0.m_aabbMax[2]);
end;

{$HINTS OFF}
function btSimpleBroadphase.createProxy(const aabbMin, aabbMax: btVector3; const shapeType: TbtBroadphaseNativeTypes; const userPtr: Pointer; const collisionFilterGroup, collisionFilterMask: btCollisionFilterGroupSet;  const dispatcher: btDispatcher; const multiSapProxy: Pointer): btBroadphaseProxy;
var newHandleIndex: LongInt;
begin
  if (m_numHandles >= m_maxHandles) then begin
    btAssert(false);
    result:=nil; //should never happen, but don't let the game crash ;-)
  end;
  btAssert((aabbMin[0]<=aabbMax[0]) and (aabbMin[1]<=aabbMax[1]) and (aabbMin[2]<=aabbMax[2]));
  newHandleIndex := allocHandle;
  m_pHandles[newHandleIndex]^.init(aabbMin,aabbMax,shapeType,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy,newHandleIndex);
  result:=m_pHandles[newHandleIndex]^;
end;
{$HINTS ON}

procedure btSimpleBroadphase.calculateOverlappingPairs(const dispatcher: btDispatcher);
var  i,j:integer;
     new_largest_index: Integer;
     proxy0 : btSimpleBroadphaseProxy;
     proxy1 : btSimpleBroadphaseProxy;
     overlappingPairArray : btBroadphasePairArray;
     previousPair :btBroadphasePair;
     pair : PbtBroadphasePair;
     isDuplicate: Boolean;
     needsRemoval: Boolean;
     hasOverlap: Boolean;

begin
  //first check for new overlapping pairs
  //write('<<-COP ');
  if (m_numHandles >= 0) then begin
    new_largest_index := -1;
    for i:=0 to m_LastHandleIndex do begin
      proxy0 := m_pHandles[i]^;
      if(not assigned(proxy0.m_clientObject)) then begin
        //write(format(' %d IC ',[i]));
        continue;
      end;
      new_largest_index := i;
      for j:=i+1 to m_LastHandleIndex do begin
        proxy1 := m_pHandles[j]^;
        btAssert(proxy0 <> proxy1);
        if(not assigned(proxy1.m_clientObject)) then begin
          //write(format(' %d JC ',[j]));
          continue;
        end;
        if aabbOverlap(proxy0,proxy1) then begin
          //write(format(' <AAB OL %d %d',[proxy0.getUid,proxy1.getUid]));
          if not assigned(m_pairCache.findPair(proxy0,proxy1)) then begin
            //write(' NOTASS ADD ');
            m_pairCache.addOverlappingPair(proxy0,proxy1);
          end;
          //write(' AAB>');
         // abort;
        end else begin
          //write(format(' <!AAB OL %d %d',[proxy0.getUid,proxy1.getUid]));
          if not m_pairCache.hasDeferredRemoval then begin
            if assigned(m_pairCache.findPair(proxy0,proxy1)) then begin
              //write(' ASS REM ');
              m_pairCache.removeOverlappingPair(proxy0,proxy1,dispatcher);
            end;
          end;
          //write(' !AAB>');
        end;
      end;
    end;

    m_LastHandleIndex := new_largest_index;
    //write(format(' -LHI %d ',[m_LastHandleIndex]));

    if (m_ownsPairCache and m_pairCache.hasDeferredRemoval) then begin
      //write(' OPDR ');
      overlappingPairArray := m_pairCache.getOverlappingPairArray;
      //perform a sort, to find duplicates and to sort 'invalid' pairs to the end
      overlappingPairArray.quickSort(@btBroadphasePairSortPredicate);

      overlappingPairArray.Setlength(overlappingPairArray.length - m_invalidPair);
      m_invalidPair := 0;

      previousPair.m_pProxy0   := nil;
      previousPair.m_pProxy1   := nil;
      previousPair.m_algorithm := nil;

      for i := 0 to overlappingPairArray.Length do begin
        pair := overlappingPairArray.A[i];
        //write(format(' S(%d)[PROX:%d %d]',[i,pair^.m_pProxy0.getUid,pair^.m_pProxy1.getUid]));
        isDuplicate := (pair^ = previousPair);
        previousPair := pair^;
        needsRemoval := false;
        if (not isDuplicate) then begin
          hasOverlap := testAabbOverlap(btSimpleBroadphaseProxy(pair^.m_pProxy0),btSimpleBroadphaseProxy(pair^.m_pProxy1));
          if (hasOverlap) then begin
            needsRemoval := false;//callback->processOverlap(pair);
          end else begin
            needsRemoval := true;
          end;
        end else begin
          //remove duplicate
          needsRemoval := true;
          //should have no algorithm
          btAssert(not assigned(pair^.m_algorithm));
        end;
        if (needsRemoval) then begin
          m_pairCache.cleanOverlappingPair(pair,dispatcher);
          //              m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
          //              m_overlappingPairArray.pop_back();
          pair^.m_pProxy0 := nil;
          pair^.m_pProxy1 := nil;
          inc(m_invalidPair);
          dec(gOverlappingPairs);
        end;
      end;
      ///if you don't like to skip the invalid pairs in the array, execute following code:
  {$ifdef CLEAN_INVALID_PAIRS}
      //perform a sort, to sort 'invalid' pairs to the end
      overlappingPairArray.quickSort(@btBroadphasePairSortPredicate);
      overlappingPairArray.Setlength(overlappingPairArray.Length - m_invalidPair);
      m_invalidPair := 0;
  {$endif}//CLEAN_INVALID_PAIRS
    end;
  end;
end;


procedure btSimpleBroadphase.destroyProxy(const proxy: btBroadphaseProxy; const dispatcher: btDispatcher);
begin
  freeHandle(btSimpleBroadphaseProxy(proxy));
  m_pairCache.removeOverlappingPairsContainingProxy(proxy,dispatcher);
  //validate();
end;

{$HINTS OFF}
procedure btSimpleBroadphase.setAabb(const proxy: btBroadphaseProxy; const aabbMin, aabbMax: btVector3; const dispatcher: btDispatcher);
begin
  btSimpleBroadphaseProxy(proxy).m_aabbMin := aabbMin;
  btSimpleBroadphaseProxy(proxy).m_aabbMax := aabbMax;
end;
{$HINTS ON}

procedure btSimpleBroadphase.getAabb(const proxy: btBroadphaseProxy; var aabbMin, aabbMax: btVector3);
begin
  aabbMin := btSimpleBroadphaseProxy(proxy).m_aabbMin;
  aabbMax := btSimpleBroadphaseProxy(proxy).m_aabbMax;
end;

{$HINTS OFF}
procedure btSimpleBroadphase.rayTest(const rayFrom, rayTo: btVector3; const rayCallback: btBroadphaseRayCallback; const aabbMin, aabbMax: btVector3);
var  i: Integer;
  proxy: btSimpleBroadphaseProxy;
begin
  for i:=0 to m_LastHandleIndex do begin
    proxy:=m_pHandles[i]^;
    if(not assigned(proxy.m_clientObject)) then begin
            continue;
    end;
    rayCallback.process(proxy);
  end;
end;
{$HINTS ON}

procedure btSimpleBroadphase.aabbTest(const aabbMin, aabbMax: btVector3; const callback: btBroadphaseAabbCallback);
var  i: Integer;
  proxy: btSimpleBroadphaseProxy;
begin
  for i:=0 to m_LastHandleIndex do begin
    proxy:=m_pHandles[i]^;
    if(not assigned(proxy.m_clientObject)) then begin
            continue;
    end;
    if (TestAabbAgainstAabb2(aabbMin,aabbMax,proxy.m_aabbMin,proxy.m_aabbMax)) then begin
      callback.process(proxy);
    end;
  end;
end;

function btSimpleBroadphase.getOverlappingPairCache: btOverlappingPairCache;
begin
  Result := m_pairCache;
end;

function btSimpleBroadphase.testAabbOverlap(const proxy0, proxy1: btBroadphaseProxy): boolean;
begin
  result:=aabbOverlap(btSimpleBroadphaseProxy(proxy0),btSimpleBroadphaseProxy(proxy1));
end;

procedure btSimpleBroadphase.getBroadphaseAabb(out aabbMin, aabbMax: btVector3);
begin
  aabbMin.InitSame(-BT_LARGE_FLOAT);
  aabbMax.InitSame(BT_LARGE_FLOAT);
end;

procedure btSimpleBroadphase.printStats;
begin
   WriteLn('btSimpleBroadphase');
   WriteLn(format('numHandles = %d, maxHandles = %dn',[m_numHandles,m_maxHandles]));
end;

{ btDefaultCollisionConstructionInfo }

function btDefaultCollisionConstructionInfo.Init:btDefaultCollisionConstructionInfo;
begin
  with result do begin
    m_stackAlloc                             := nil;
    m_persistentManifoldPool                 := nil;
    m_collisionAlgorithmPool                 := nil;
    m_defaultMaxPersistentManifoldPoolSize   := 4096;
    m_defaultMaxCollisionAlgorithmPoolSize   := 4096;
    m_customCollisionAlgorithmMaxElementSize := 0;
    m_defaultStackAllocatorSize              := 0;
    m_useEpaPenetrationAlgorithm             := true;
  end;
end;

{ btDefaultCollisionConfiguration }

constructor btDefaultCollisionConfiguration.Create(const constructionInfo: btDefaultCollisionConstructionInfo);
begin
  m_simplexSolver := btVoronoiSimplexSolver.Create;
  if (constructionInfo.m_useEpaPenetrationAlgorithm) then begin
    m_pdSolver := btGjkEpaPenetrationDepthSolver.Create;
  end else begin
    m_pdSolver := btMinkowskiPenetrationDepthSolver.Create;
  end;

  //default CreationFunctions, filling the m_doubleDispatch table
  m_convexConvexCreateFunc         := btConvexConvexAlgorithmCreateFunc.Create(m_simplexSolver,m_pdSolver);
  m_convexConcaveCreateFunc        := btConvexConcaveCollisionAlgorithmCreateFuncNormAndSwapped.Create(false);
  m_swappedConvexConcaveCreateFunc := btConvexConcaveCollisionAlgorithmCreateFuncNormAndSwapped.Create(true);
  m_compoundCreateFunc             := btCompoundCollisionAlgorithmCreateFuncNormAndSwapped.Create(false);
  m_swappedCompoundCreateFunc      := btCompoundCollisionAlgorithmCreateFuncNormAndSwapped.Create(false);
  m_emptyCreateFunc                := btEmptyAlgorithmCreateFunc.Create;
  m_sphereSphereCF                 := btSphereSphereCollisionAlgorithmCreateFunc.Create;
  {$ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM}
    //m_sphereBoxCF := btSphereBoxCollisionAlgorithm.CreateFunc; FOSNOTCHECKED
    //m_boxSphereCF := btSphereBoxCollisionAlgorithm.CreateFunc;
    //m_boxSphereCF.m_swapped := true;
  {$endif} //USE_BUGGY_SPHERE_BOX_ALGORITHM

   m_sphereTriangleCF := btSphereTriangleCollisionAlgorithmCreateFunc.Create(false);
   m_triangleSphereCF := btSphereTriangleCollisionAlgorithmCreateFunc.Create(true);
   m_boxBoxCF         := btBoxBoxCollisionAlgorithmCreateFunc.Create;
  //convex versus plane
  m_convexPlaneCF := btConvexPlaneCollisionAlgorithmCreateFuncNormAndSwapped.Create(false);
  m_planeConvexCF := btConvexPlaneCollisionAlgorithmCreateFuncNormAndSwapped.Create(true);

  ///calculate maximum element size, big enough to fit any collision algorithm in the memory pool
  if assigned(constructionInfo.m_stackAlloc) then begin
    m_ownsStackAllocator         := false;
    self.m_stackAlloc            := constructionInfo.m_stackAlloc;
  end else begin
    m_ownsStackAllocator         := true;
    m_stackAlloc                 := btStackAllocator.Create(constructionInfo.m_defaultStackAllocatorSize);
  end;

  if assigned(constructionInfo.m_persistentManifoldPool) then begin
    m_ownsPersistentManifoldPool := false;
    m_persistentManifoldPool     := constructionInfo.m_persistentManifoldPool;
  end else begin
    m_ownsPersistentManifoldPool := true;
    m_persistentManifoldPool     := btPoolAllocator.Create(constructionInfo.m_defaultMaxPersistentManifoldPoolSize);
  end;

  if assigned(constructionInfo.m_collisionAlgorithmPool) then begin
    m_ownsCollisionAlgorithmPool := false;
     m_collisionAlgorithmPool    := constructionInfo.m_collisionAlgorithmPool;
  end else begin
    m_ownsCollisionAlgorithmPool := true;
    m_collisionAlgorithmPool     := btPoolAllocator.Create(constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize);
  end;
end;

destructor btDefaultCollisionConfiguration.Destroy;
begin
  if m_ownsStackAllocator then begin
    m_stackAlloc.free;
  end;
  if m_ownsCollisionAlgorithmPool then begin
    m_collisionAlgorithmPool.Free;
  end;
  if m_ownsPersistentManifoldPool then begin
    m_persistentManifoldPool.free;
  end;
  m_convexConvexCreateFunc.free;
  m_convexConcaveCreateFunc.free;
  m_swappedConvexConcaveCreateFunc.free;
  m_compoundCreateFunc.free;
  m_swappedCompoundCreateFunc.free;
  m_emptyCreateFunc.free;
  m_sphereSphereCF.free;
{$ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM}
  m_sphereBoxCF.free;
  m_boxSphereCF.free;
{$endif} //USE_BUGGY_SPHERE_BOX_ALGORITHM
  m_sphereTriangleCF.free;
  m_triangleSphereCF.free;
  m_boxBoxCF.free;
  m_convexPlaneCF.free;
  m_planeConvexCF.free;
  m_simplexSolver.free;
  m_pdSolver.free;
end;

function btDefaultCollisionConfiguration.getPersistentManifoldPool: btPoolAllocator;
begin
  result := m_persistentManifoldPool;
end;

function btDefaultCollisionConfiguration.getCollisionAlgorithmPool: btPoolAllocator;
begin
  result := m_collisionAlgorithmPool;
end;

function btDefaultCollisionConfiguration.getStackAllocator: btStackAllocator;
begin
  Result := m_stackAlloc;
end;

function btDefaultCollisionConfiguration.getSimplexSolver: btVoronoiSimplexSolver;
begin
  result := m_simplexSolver;
end;

function btDefaultCollisionConfiguration.getCollisionAlgorithmCreateFunc(const proxyType0, proxyType1: TbtBroadphaseNativeTypes): btCollisionAlgorithmCreateFunc;
begin
  if (proxyType0 = SPHERE_SHAPE_PROXYTYPE) and (proxyType1=SPHERE_SHAPE_PROXYTYPE) then begin
    result :=  m_sphereSphereCF;
  end else
  {$ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM}
  if (proxyType0 = SPHERE_SHAPE_PROXYTYPE) and (proxyType1=BOX_SHAPE_PROXYTYPE) then begin
    result :=  m_sphereBoxCF;
  end else
  if (proxyType0 = BOX_SHAPE_PROXYTYPE ) and (proxyType1=SPHERE_SHAPE_PROXYTYPE) then begin
    result :=  m_boxSphereCF;
  end else
  {$endif} //USE_BUGGY_SPHERE_BOX_ALGORITHM
  if (proxyType0 = SPHERE_SHAPE_PROXYTYPE ) and (proxyType1=TRIANGLE_SHAPE_PROXYTYPE) then begin
    result :=  m_sphereTriangleCF;
  end else
  if (proxyType0 = TRIANGLE_SHAPE_PROXYTYPE  ) and (proxyType1=SPHERE_SHAPE_PROXYTYPE) then begin
    result :=  m_triangleSphereCF;
  end else
  if (proxyType0 = BOX_SHAPE_PROXYTYPE) and (proxyType1 = BOX_SHAPE_PROXYTYPE) then begin
    result := m_boxBoxCF;
  end else
  if (btBroadphaseProxy.isConvex(proxyType0) and (proxyType1 = STATIC_PLANE_PROXYTYPE)) then begin
    result := m_convexPlaneCF;
  end else
  if (btBroadphaseProxy.isConvex(proxyType1) and (proxyType0 = STATIC_PLANE_PROXYTYPE)) then begin
    result := m_planeConvexCF;
  end else
  if (btBroadphaseProxy.isConvex(proxyType0) and btBroadphaseProxy.isConvex(proxyType1)) then begin
    result := m_convexConvexCreateFunc;
  end else
  if (btBroadphaseProxy.isConvex(proxyType0) and btBroadphaseProxy.isConcave(proxyType1)) then begin
    result := m_convexConcaveCreateFunc;
  end else
  if (btBroadphaseProxy.isConvex(proxyType1) and btBroadphaseProxy.isConcave(proxyType0)) then begin
    result := m_swappedConvexConcaveCreateFunc;
  end else
  if (btBroadphaseProxy.isCompound(proxyType0)) then begin
    result := m_compoundCreateFunc;
  end else begin
    if (btBroadphaseProxy.isCompound(proxyType1)) then begin
      result := m_swappedCompoundCreateFunc;
    end else begin
      result := m_emptyCreateFunc;   //failed to find an algorithm
    end;
  end;
end;

procedure btDefaultCollisionConfiguration.setConvexConvexMultipointIterations(const numPerturbationIterations: integer; const minimumPointsPerturbationThreshold: integer);
var convexConvex:btConvexConvexAlgorithmCreateFunc;
begin
  convexConvex := btConvexConvexAlgorithmCreateFunc(m_convexConvexCreateFunc);
  convexConvex.m_numPerturbationIterations          := numPerturbationIterations;
  convexConvex.m_minimumPointsPerturbationThreshold := minimumPointsPerturbationThreshold;
end;

{ btEmptyAlgorithm }

{$HINTS OFF}
procedure btEmptyAlgorithm.processCollision(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult);
begin

end;
{$HINTS ON}
{$HINTS OFF}
function btEmptyAlgorithm.calculateTimeOfImpact(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar;
begin
  result := 1;
end;

procedure btEmptyAlgorithm.getAllContactManifolds(const manifoldArray: btManifoldArray);
begin

end;
{$HINTS ON}

{ btBoxBoxCollisionAlgorithm }

procedure btBoxBoxCollisionAlgorithm.Init(const ci: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject);
begin
  inherited Init(ci,body0,body1);
  m_ownManifold := false;
  m_manifoldPtr := nil;
  if not assigned(m_manifoldPtr) and m_dispatcher.needsCollision(body0,body1) then begin
    m_manifoldPtr := m_dispatcher.getNewManifold(body0,body1);
    m_ownManifold := true;
  end;
end;

destructor btBoxBoxCollisionAlgorithm.Destroy;
begin
  if m_ownManifold then begin
    if assigned(m_manifoldPtr) then begin
      m_dispatcher.releaseManifold(m_manifoldPtr);
    end;
  end;
  inherited Destroy;
end;

procedure btBoxBoxCollisionAlgorithm.processCollision(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult);
var box0,box1:btBoxShape;
    input:btDCDI_ClosestPointInput;
    detector:btBoxBoxDetector;
begin
  if not assigned(m_manifoldPtr) then begin  abort;  exit; end; //fos dbg try

  //btCollisionObject*    col0 = body0;
  //btCollisionObject*    col1 = body1;

  box0 := btBoxShape(body0.getCollisionShape);
  box1 := btBoxShape(body1.getCollisionShape);

 /// report a contact. internally this will be kept persistent, and contact reduction is done
  resultOut.setPersistentManifold(m_manifoldPtr);
{$ifndef USE_PERSISTENT_CONTACTS}
  s m_manifoldPtr.clearManifold;
{$endif} //USE_PERSISTENT_CONTACTS
  input.init;
  input.m_transformA             := body0.getWorldTransformP^;
  input.m_transformB             := body1.getWorldTransformP^;

  detector.init(box0,box1);
  detector.getClosestPoints(input,btDCDI_Result(resultOut),dispatchInfo.m_debugDraw);
{$ifdef USE_PERSISTENT_CONTACTS}
  //  refreshContactPoints is only necessary when using persistent contact points. otherwise all points are newly added
  if (m_ownManifold) then begin
    resultOut.refreshContactPoints;
  end;
{$endif} //USE_PERSISTENT_CONTACTS
end;

{$HINTS OFF}
function btBoxBoxCollisionAlgorithm.calculateTimeOfImpact(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar;
begin
  //not yet
  Result := 1;
end;
{$HINTS ON}

procedure btBoxBoxCollisionAlgorithm.getAllContactManifolds(const manifoldArray: btManifoldArray);
begin
  if assigned(m_manifoldPtr) and m_ownManifold then begin
    manifoldArray.push_back(m_manifoldPtr);
  end;
end;

{ btActivatingCollisionAlgorithm }

{$HINTS OFF}
procedure btActivatingCollisionAlgorithm.Init(const ci:btCollisionAlgorithmConstructionInfo;const body0, body1: btCollisionObject);
begin
  inherited Init(ci);
end;
{$HINTS ON}

{ btBoxBoxDetector }


// given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and
// generate contact points. this returns 0 if there is no contact otherwise
// it returns the number of contacts generated.
// `normal' returns the contact normal.
// `depth' returns the maximum penetration depth along that normal.
// `return_code' returns a number indicating the type of contact that was
// detected:
//        1,2,3 = box 2 intersects with a face of box 1
//        4,5,6 = box 1 intersects with a face of box 2
//        7..15 = edge-edge contact
// `maxc' is the maximum number of contacts allowed to be generated, i.e.
// the size of the `contact' array.
// `contact' and `skip' are the contact array information provided to the
// collision functions. this function only fills in the position and depth
// fields.
//struct dContactGeom;
//#define dDOTpq(a,b,p,q) ((a)[0]*(b)[0] + (a)[p]*(b)[q] + (a)[2*(p)]*(b)[2*(q)])
// #define dInfinity FLT_MAX -> SIMD_INFINITY
//

(*
  PURE_INLINE btScalar dDOT   (const btScalar *a, const btScalar *b) { return dDOTpq(a,b,1,1); }
  PURE_INLINE btScalar dDOT13 (const btScalar *a, const btScalar *b) { return dDOTpq(a,b,1,3); }
  PURE_INLINE btScalar dDOT31 (const btScalar *a, const btScalar *b) { return dDOTpq(a,b,3,1); }
  PURE_INLINE btScalar dDOT33 (const btScalar *a, const btScalar *b) { return dDOTpq(a,b,3,3); }
*)

function dDOTpq (a,b:PbtScalar;const p,q:integer):btScalar;FOS_INLINE;
begin
  result := a[0]*b[0] + a[p]*b[q] + a[2*p]*b[2*q];
end;
function dDOT   (const a,b:PbtScalar): btScalar;FOS_INLINE;
begin
  result := dDOTpq(a,b,1,1);
end;
function dDOT44 (const a,b:PbtScalar): btScalar;FOS_INLINE;
begin
  result := dDOTpq(a,b,4,4);
end;
function dDOT41  (const a,b:PbtScalar): btScalar;FOS_INLINE;
begin
  result := dDOTpq(a,b,4,1);
end;
function dDOT14  (const a,b:PbtScalar): btScalar;FOS_INLINE;
begin
  result := dDOTpq(a,b,1,4);
end;

//#define dMULTIPLYOP1_331(A,op,B,C) \
//{\
//  (A)[0] op dDOT41((B),(C)); \
//  (A)[1] op dDOT41((B+1),(C)); \
//  (A)[2] op dDOT41((B+2),(C)); \
//}
//
//#define dMULTIPLYOP0_331(A,op,B,C) \
//{ \
//  (A)[0] op dDOT((B),(C)); \
//  (A)[1] op dDOT((B+4),(C)); \
//  (A)[2] op dDOT((B+8),(C)); \
//}
//
//#define dMULTIPLY1_331(A,B,C) dMULTIPLYOP1_331(A,=,B,C)
//#define dMULTIPLY0_331(A,B,C) dMULTIPLYOP0_331(A,=,B,C)
//
type dMatrix3= Array [0..11] of btScalar;

procedure dLineClosestApproach (const pa, ua, pb, ub : btVector3; out alpha, beta :btScalar);
var p:btVector3;
    uaub,q1,q2,d:btScalar;
begin
  p[0] := pb[0] - pa[0];
  p[1] := pb[1] - pa[1];
  p[2] := pb[2] - pa[2];
  uaub := dDOT(@ua,@ub);
    q1 := dDOT(@ua,@p);
    q2 := -dDOT(@ub,@p);
     d := 1-uaub*uaub;
  if (d <= 0.0001)  then begin
    // @@@ this needs to be made more robust
    alpha := 0;
    beta  := 0;
  end else begin
    d := 1/d;
    alpha := (q1 + uaub*q2)*d;
    beta  := (uaub*q1 + q2)*d;
  end;
end;

// find all the intersection points between the 2D rectangle with vertices
// at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
// (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
//
// the intersection points are returned as x,y pairs in the 'ret' array.
// the number of intersection points is returned by the function (this will
// be in the range 0 to 8).

type dArray2  = array[0..1] of btScalar;
     dArray8  = array[0..7] of btScalar;
     dArray16 = array[0..15] of btScalar;

function intersectRectQuad2 (const h:dArray2;const p:dArray8;out ret:dArray16):integer;
var
  nq,i      : Integer;
  nr        : Integer;
  buffer    : dArray16;
  q,r,pq,pr,
  nextq     : PbtScalar;
  dir,sign  : Integer;
  label done;

begin
  // q (and r) contain nq (and nr) coordinate points for the current (and chopped) polygons
  nq:=4;nr:=0;
  q := @p;
  r := @ret;
  for dir:=0 to 1 do begin
    // direction notation: xy[0] = x axis, xy[1] = y axis
    sign:=-1;
    while(sign<=1) do begin
      // chop q along the line xy[dir] = sign*h[dir]
      pq := q; pr := r; nr := 0;
      for i := nq downto 1 do begin
        // go through all points in q and all lines between adjacent points
        if sign*pq[dir] < h[dir] then begin
          // this point is inside the chopping line
          pr[0] := pq[0];
          pr[1] := pq[1];
          pr    := pr+2;
          nr    := nr+1;
          if (nr and 8)>0 then begin
            q := r;
            goto done;
          end;
        end;
        if i>1 then begin
          nextq := pq+2;
        end else begin
          nextq := q;
        end;
        if (sign*pq[dir] < h[dir]) XOR (sign*nextq[dir] < h[dir]) then begin
          // this line crosses the chopping line
          pr[1-dir] := pq[1-dir] + (nextq[1-dir]-pq[1-dir]) / (nextq[dir]-pq[dir]) * (sign*h[dir]-pq[dir]);
          pr[dir]   := sign*h[dir];
          pr        := pr+2;
          nr        := nr +1;
          if (nr and 8) >0 then begin
            q := r;
            goto done;
          end
        end;
        pq := pq + 2;
      end;
      q := r;
      if q=@ret then begin
        r:=@buffer[0];
      end else begin
        r:=ret;
      end;
      nq := nr;
      sign := sign+2;
    end;
  end;
 done:
  if (q <> @ret) then begin
    move(q[0],ret[0],nr*2*sizeof(btScalar));
  end;
  Result:=nr;
end;

// given n points in the plane (array p, of size 2*n), generate m points that
// best represent the whole set. the definition of 'best' here is not
// predetermined - the idea is to select points that give good box-box
// collision detection behavior. the chosen point indexes are returned in the
// array  (of size m). 'i0' is always the first entry in the array.
// n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
// in the range [0..n-1].
//
procedure cullPoints2(const n:integer; p:PbtScalar;const m,i0:integer;iret:PInteger);
var i,j:integer;
    a,cx,cy,q:btScalar;
    AA:dArray8;
    avail:Array [0..7] of integer;
    maxdiff,diff:btScalar;
begin
  // compute the centroid of the polygon in cx,cy
  if (n=1) then begin
    cx := p[0];
    cy := p[1];
  end else
  if (n=2) then begin
    cx := 0.5 * (p[0] + p[2]);
    cy := 0.5 * (p[1] + p[3]);
  end else begin
    a := 0;
    cx := 0;
    cy := 0;
    for i:=0 to (n-2) do begin
      q  := p[i*2]*p[i*2+3] - p[i*2+2]*p[i*2+1];
      a  := a+q;
      cx := cx + q*(p[i*2]+p[i*2+2]);
      cy := cy + q*(p[i*2+1]+p[i*2+3]);
    end;
    q := p[n*2-2]*p[1] - p[0]*p[n*2-1];
    if (btFabs(a+q) > SIMD_EPSILON) then begin
      a := 1/(3*(a+q));
    end else begin
      a := BT_LARGE_FLOAT;
    end;
    cx := a*(cx + q*(p[n*2-2]+p[0]));
    cy := a*(cy + q*(p[n*2-1]+p[1]));
  end;

  // compute the angle of each point w.r.t. the centroid
  for i:=0 to n-1 do begin
    AA[i] := btAtan2(p[i*2+1]-cy,p[i*2]-cx);
  end;

  // search for points that have angles closest to A[i0] + i*(2*pi/m).
  for i := 0 to n-1 do begin
    avail[i] := 1;
  end;
  avail[i0] := 0;
  iret[0]   := i0;
  inc(iret);
  for j := 1 to  m-1 do begin
    a := j*(SIMD_2_PI/m) + AA[i0];
    if (a > SIMD_PI) then begin
      a := a -SIMD_2_PI;
    end;
    maxdiff:=1e9;
    iret^ := i0;                 // iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0
    for i := 0 to  n-1 do begin
      if avail[i]>0 then begin
        diff := btFabs(AA[i]-a);
        if (diff > SIMD_PI) then begin
          diff := SIMD_2_PI - diff;
        end;
        if (diff < maxdiff) then begin
          maxdiff := diff;
          iret^    := i;
        end;
      end;
    end;
    btAssert (iret^ <> i0);     // ensure iret got set
    avail[iret^] := 0;
    inc(iret);
  end;
end;

const fudge_factor:btScalar=1.05;

//#define dMULTIPLYOP1_331(A,op,B,C) \
//{\
//  (A)[0] op dDOT41((B),(C)); \
//  (A)[1] op dDOT41((B+1),(C)); \
//  (A)[2] op dDOT41((B+2),(C)); \
//}
//#define dMULTIPLYOP1_331(A,op,B,C) \
//{\
//  (A)[0] op dDOT41((B),(C)); \
//  (A)[1] op dDOT41((B+1),(C)); \
//  (A)[2] op dDOT41((B+2),(C)); \
//}
//
//#define dMULTIPLYOP0_331(A,op,B,C) \
//{ \
//  (A)[0] op dDOT((B),(C)); \
//  (A)[1] op dDOT((B+4),(C)); \
//  (A)[2] op dDOT((B+8),(C)); \
//}

//#define dMULTIPLY1_331(A,B,C) dMULTIPLYOP1_331(A,=,B,C)
//#define dMULTIPLY0_331(A,B,C) dMULTIPLYOP0_331(A,=,B,C)

//#define TST(expr1,expr2,norm,cc) \
//  s2 = btFabs(expr1) - (expr2); \
//  if (s2 > 0) return 0; \
//  if (s2 > s) { \
//    s = s2; \
//    normalR = norm; \
//    invert_normal = ((expr1) < 0); \
//    code = (cc); \
//  }

//const cfudge2:btScalar = 1.0e-5;


function dBoxBox2 (const p1:btVector3     ; const R1:dMatrix3;
                   const side1:btVector3  ; const p2:btVector3;
                   const R2:dMatrix3      ; const side2:btVector3;
                   out   normal:btVector3 ; const depth:PbtScalar;out return_code:integer;
                   var   maxc:integer     ; var output:btDCDI_Result):integer;
var   p,pp,normalC,center                        : btVector3;
      normalR,Ra,Rb,pa,pb,Sa,Sb                  : PbtScalar;
      R11,R12,R13,R21,R22,R23,R31,R32,R33,det1,
      Q11,Q12,Q13,Q21,Q22,Q23,Q31,Q32,Q33,s,s2,l : btScalar;
      A,B                                        : array [0..2] of btScalar;
      rect                                       : array [0..1] of btScalar;
      i,j,code,lanr,a1,a2                        : integer;
      invert_normal                              : boolean;
      normal2,nr,anr                             : btVector3;
      codeN,code1,code2                          : integer;
      quad                                       : array [0..7] of btScalar; // 2D coordinate of incident face (x,y pairs)
      c1,c2,m11,m12,m21,m22,k1,k2,k3,k4          : btScalar;
      ret                                        : array [0..15] of btScalar;
      n                                          : LongInt;
      point                                      : array [0..3*8-1] of btScalar; // penetrating contact points
      dep                                        : array [0..7]  of btScalar; // depths for those points
      cnum                                       : Integer;
      pointInWorld,posInWorld                    : btVector3; //TODO FOS : make same ?
      i1                                         : Integer;
      maxdepth                                   : btScalar;
      iret                                       : array [0..7] of Integer;

      function TST(const expr1,expr2:btScalar;const norm:PbtScalar;const cc:integer):boolean;FOS_INLINE;
      begin
        s2 := btFabs(expr1) - expr2;
        if s2 > 0 then exit(true);
        if s2 > s then begin
          s := s2;
          normalR := norm;
          invert_normal := expr1 < 0;
          code := cc;
        end;
        result:=false;
      end;

      function TST2(const expr1,expr2,n1,n2,n3:btScalar;const cc:integer):boolean;FOS_INLINE;
      begin
        s2 := btFabs(expr1) - (expr2);
        if s2 > SIMD_EPSILON then exit(true);
        l  := btSqrt((n1)*(n1) + (n2)*(n2) + (n3)*(n3));
        if (l > SIMD_EPSILON) then begin
          s2 := s2 / l;
          if (s2*fudge_factor > s) then begin
            s := s2;
            normalR := nil;
            normalC[0] := n1/l; normalC[1] := n2/l; normalC[2] := n3/l;
            invert_normal := expr1 < 0;
            code := cc;
          end;
        end;
        result:=false;
      end;

      procedure _ComputeContactPoints;
      var pa,pb,ua,ub:btVector3;
          sign,alpha,beta:btScalar;
          i,j:integer;
          k : btScalar;
      begin
        // an edge from box 1 touches an edge from box 2.
        // find a point pa on the intersecting edge of box 1
        pa.InitVector(p1);
        for j:=0 to 2 do begin
          if dDOT14(@normal,PbtScalar(R1)+j)>0 then begin
            sign:=1;
          end else begin
            sign:=-1;
          end;
          for i:=0 to 2 do pa[i] := pa[i] +  (sign * A[j] * R1[i*4+j]);
        end;
        // find a point pb on the intersecting edge of box 2
        pb.InitVector(p2);
        for j:=0 to 2 do begin
          k:=dDOT14(@normal,@R2[j]);
          if k>0 then begin
            sign:=-1;
          end else begin
            sign:=1;
          end;
          for i:=0 to 2 do pb[i] := pb[i] +  (sign * B[j] * R2[i*4+j]);
        end;

        for i:=0 to 2 do begin
          ua[i] := R1[(code-7) div 3 + i*4];
        end;
        for i:=0 to 2 do begin
          ub[i] := R2[(code-7) mod 3 + i*4];
        end;

        dLineClosestApproach (pa,ua,pb,ub,alpha,beta);

        for i:=0 to 2 do pa[i] := pa[i] + ua[i]*alpha;
        for i:=0 to 2 do pb[i] := pb[i] + ub[i]*beta;

        //contact[0].pos[i] = btScalar(0.5)*(pa[i]+pb[i]);
        //contact[0].depth = *depth;
        {$ifdef USE_CENTER_POINT}
         for i:=0 to 2 do pointInWorld[i] := (pa[i]+pb[i])*0.5;
          output.addContactPoint(-normal,pointInWorld,-depth^); ss
        {$else}
          output.addContactPoint(-normal,pb,-depth^);
        {$endif} //
        return_code := code;
      end;


begin
//  writeln('ENTER BOXBOX p1=',p1.DumpValues,' side1/2=',side1.DumpValues,'/',side2.DumpValues,' normal=',normal.DumpValues,' maxc=',maxc,' ');
  normalC.InitSame(0);
  normalR := nil;
  // get vector from centers of box 1 to box 2, relative to box 1
  p := p2 - p1;

//dMULTIPLY1_331 (pp,R1,p);             // get pp = p relative to body 1
  pp[0] := dDOT41(@R1[0],@p);
  pp[1] := dDOT41(@R1[1],@p);
  pp[2] := dDOT41(@R1[2],@p);
//END  dMULTIPLY1_331 (pp,R1,p);             // get pp = p relative to body 1

  // get side lengths / 2
  A[0] := side1[0]*(0.5);
  A[1] := side1[1]*(0.5);
  A[2] := side1[2]*(0.5);
  B[0] := side2[0]*(0.5);
  B[1] := side2[1]*(0.5);
  B[2] := side2[2]*(0.5);

  // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
  R11 := dDOT44(@R1[0],@R2[0]); R12 := dDOT44(@R1[0],@R2[1]); R13 := dDOT44(@R1[0],@R2[2]);
  R21 := dDOT44(@R1[1],@R2[0]); R22 := dDOT44(@R1[1],@R2[1]); R23 := dDOT44(@R1[1],@R2[2]);
  R31 := dDOT44(@R1[2],@R2[0]); R32 := dDOT44(@R1[2],@R2[1]); R33 := dDOT44(@R1[2],@R2[2]);

  Q11 := btFabs(R11); Q12 := btFabs(R12); Q13 := btFabs(R13);
  Q21 := btFabs(R21); Q22 := btFabs(R22); Q23 := btFabs(R23);
  Q31 := btFabs(R31); Q32 := btFabs(R32); Q33 := btFabs(R33);

  // for all 15 possible separating axes:
  //   * see if the axis separates the boxes. if so, return 0.
  //   * find the depth of the penetration along the separating axis (s2)
  //   * if this is the largest depth so far, record it.
  // the normal vector will be set to the separating axis with the smallest
  // depth. note: normalR is set to point to a column of R1 or R2 if that is
  // the smallest depth normal so far. otherwise normalR is 0 and normalC is
  // set to a vector relative to body 1. invert_normal is 1 if the sign of
  // the normal should be flipped.

  s             := -SIMD_INFINITY;
  invert_normal := false;
  code          := 0;

  // separating axis = u1,u2,u3
  if TST(pp[0],(A[0] + B[0]*Q11 + B[1]*Q12 + B[2]*Q13),@R1[0],1) then exit(0);
  if TST(pp[1],(A[1] + B[0]*Q21 + B[1]*Q22 + B[2]*Q23),@R1[1],2) then exit(0);
  if TST(pp[2],(A[2] + B[0]*Q31 + B[1]*Q32 + B[2]*Q33),@R1[2],3) then exit(0);

  // separating axis = v1,v2,v3
  if TST(dDOT41(@R2[0],@p),(A[0]*Q11 + A[1]*Q21 + A[2]*Q31 + B[0]),@R2[0],4) then exit(0);
  if TST(dDOT41(@R2[1],@p),(A[0]*Q12 + A[1]*Q22 + A[2]*Q32 + B[1]),@R2[1],5) then exit(0);
  if TST(dDOT41(@R2[2],@p),(A[0]*Q13 + A[1]*Q23 + A[2]*Q33 + B[2]),@R2[2],6) then exit(0);

  // note: cross product axes need to be scaled when s is computed.
  // normal (n1,n2,n3) is relative to box 1.

  //Q11 += cfudge2;
  //Q12 += cfudge2;
  //Q13 += cfudge2;
  //
  //Q21 += cfudge2;
  //Q22 += cfudge2;
  //Q23 += cfudge2;
  //
  //Q31 += cfudge2;
  //Q32 += cfudge2;
  //Q33 += cfudge2;

  // separating axis = u1 x (v1,v2,v3)
  if TST2(pp[2]*R21-pp[1]*R31,(A[1]*Q31+A[2]*Q21+B[1]*Q13+B[2]*Q12),0,-R31,R21,7) then exit(0);
  if TST2(pp[2]*R22-pp[1]*R32,(A[1]*Q32+A[2]*Q22+B[0]*Q13+B[2]*Q11),0,-R32,R22,8) then exit(0);
  if TST2(pp[2]*R23-pp[1]*R33,(A[1]*Q33+A[2]*Q23+B[0]*Q12+B[1]*Q11),0,-R33,R23,9) then exit(0);

  // separating axis = u2 x (v1,v2,v3)
  if TST2(pp[0]*R31-pp[2]*R11,(A[0]*Q31+A[2]*Q11+B[1]*Q23+B[2]*Q22),R31,0,-R11,10) then exit(0);
  if TST2(pp[0]*R32-pp[2]*R12,(A[0]*Q32+A[2]*Q12+B[0]*Q23+B[2]*Q21),R32,0,-R12,11) then exit(0);
  if TST2(pp[0]*R33-pp[2]*R13,(A[0]*Q33+A[2]*Q13+B[0]*Q22+B[1]*Q21),R33,0,-R13,12) then exit(0);

  // separating axis = u3 x (v1,v2,v3)
  if TST2(pp[1]*R11-pp[0]*R21,(A[0]*Q21+A[1]*Q11+B[1]*Q33+B[2]*Q32),-R21,R11,0,13) then exit(0);
  if TST2(pp[1]*R12-pp[0]*R22,(A[0]*Q22+A[1]*Q12+B[0]*Q33+B[2]*Q31),-R22,R12,0,14) then exit(0);
  if TST2(pp[1]*R13-pp[0]*R23,(A[0]*Q23+A[1]*Q13+B[0]*Q32+B[1]*Q31),-R23,R13,0,15) then exit(0);

  if code=0 then exit(0);

  // if we get to this point, the boxes interpenetrate. compute the normal
  // in global coordinates.
  if assigned(normalR) then begin
    normal[0] := normalR[0];
    normal[1] := normalR[4];
    normal[2] := normalR[8];
  end else begin
   // dMULTIPLY0_331 (normal,R1,normalC);
    normal[0] := dDOT(@R1[0],@normalC);
    normal[1] := dDOT(@R1[4],@normalC);
    normal[2] := dDOT(@R1[8],@normalC);
    // END dMULTIPLY0_331 (normal,R1,normalC);
  end;

  if (invert_normal) then begin
    normal[0] := -normal[0];
    normal[1] := -normal[1];
    normal[2] := -normal[2];
  end;
  depth^ := -s;

  // compute contact point(s)
  if fosdebug_gcount=877 then begin
    fosdebug_gcount:=fosdebug_gcount;
  end;

  if (code > 6) then begin
    _ComputeContactPoints;
    exit(1);
  end;
  // okay, we have a face-something intersection (because the separating
  // axis is perpendicular to a face). define face 'a' to be the reference
  // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
  // the incident face (the closest face of the other box).

  if (code <= 3) then begin
    Ra := @R1;
    Rb := @R2;
    pa := @p1;
    pb := @p2;
    Sa := @A;
    Sb := @B;
  end else begin
    Ra := @R2;
    Rb := @R1;
    pa := @p2;
    pb := @p1;
    Sa := @B;
    Sb := @A;
  end;

  // nr = normal vector of reference face dotted with axes of incident box.
  // anr = absolute values of nr.
  if (code <= 3) then begin
    normal2[0] := normal[0];
    normal2[1] := normal[1];
    normal2[2] := normal[2];
  end else  begin
    normal2[0] := -normal[0];
    normal2[1] := -normal[1];
    normal2[2] := -normal[2];
  end;

//  dMULTIPLY1_331 (nr,Rb,normal2);
  nr[0] := dDOT41(@Rb[0],@normal2);
  nr[1] := dDOT41(@Rb[1],@normal2);
  nr[2] := dDOT41(@Rb[2],@normal2);
//END  dMULTIPLY1_331 (nr,Rb,normal2);

  anr[0] := btFabs (nr[0]);
  anr[1] := btFabs (nr[1]);
  anr[2] := btFabs (nr[2]);

  // find the largest compontent of anr: this corresponds to the normal
  // for the indident face. the other axis numbers of the indicent face
  // are stored in a1,a2.
  if (anr[1] > anr[0]) then begin
    if (anr[1] > anr[2]) then begin
      a1   := 0;
      lanr := 1;
      a2   := 2;
    end else begin
      a1   := 0;
      a2   := 1;
      lanr := 2;
    end;
  end else begin
    if (anr[0] > anr[2]) then begin
      lanr := 0;
      a1   := 1;
      a2   := 2;
    end else begin
      a1   := 0;
      a2   := 1;
      lanr := 2;
    end;
  end;

  // compute center point of incident face, in reference-face coordinates
  if (nr[lanr] < 0) then begin
    for i:=0 to 2 do  begin
      center[i] := pb[i] - pa[i] + Sb[lanr] * Rb[i*4+lanr];
    end;
  end else begin
    for i:=0 to 2 do begin
     center[i] := pb[i] - pa[i] - Sb[lanr] * Rb[i*4+lanr];
    end;
  end;

  // find the normal and non-normal axis numbers of the reference box
  if (code <= 3) then begin
    codeN := code-1;
  end else begin
    codeN := code-4;
  end;
  if (codeN = 0) then begin
    code1 := 1;
    code2 := 2;
  end else
  if (codeN=1) then begin
    code1 := 0;
    code2 := 2;
  end else begin
    code1 := 0;
    code2 := 1;
  end;

  // find the four corners of the incident face, in reference-face coordinates
  c1 := dDOT14 (@center,@Ra[code1]);
  c2 := dDOT14 (@center,@Ra[code2]);
  // optimize this? - we have already computed this data above, but it is not
  // stored in an easy-to-index format. for now it's quicker just to recompute
  // the four dot products.
  m11 := dDOT44 (@Ra[code1],@Rb[a1]);
  m12 := dDOT44 (@Ra[code1],@Rb[a2]);
  m21 := dDOT44 (@Ra[code2],@Rb[a1]);
  m22 := dDOT44 (@Ra[code2],@Rb[a2]);

  k1 := m11*Sb[a1];
  k2 := m21*Sb[a1];
  k3 := m12*Sb[a2];
  k4 := m22*Sb[a2];
  quad[0] := c1 - k1 - k3;
  quad[1] := c2 - k2 - k4;
  quad[2] := c1 - k1 + k3;
  quad[3] := c2 - k2 + k4;
  quad[4] := c1 + k1 + k3;
  quad[5] := c2 + k2 + k4;
  quad[6] := c1 + k1 - k3;
  quad[7] := c2 + k2 - k4;


  // find the size of the reference face
  rect[0] := Sa[code1];
  rect[1] := Sa[code2];

  // intersect the incident and reference faces
  n := intersectRectQuad2 (rect,quad,ret);

  if (n < 1) then begin
  //  btAssert(false);
    exit(-1);  // this should never happen
  end;

  // convert the intersection points into reference-face coordinates,
  // and compute the contact position and depth for each point. only keep
  // those points that have a positive (penetrating) depth. delete points in
  // the 'ret' array as necessary so that 'point' and 'ret' correspond.
  //btScalar point[3*8];          // penetrating contact points
  //btScalar dep[8];                      // depths for those points
  det1 := 1/(m11*m22 - m12*m21);
  m11 *= det1;
  m12 *= det1;
  m21 *= det1;
  m22 *= det1;
  cnum := 0;                 // number of penetrating contact points found
  for j:=0 to n-1 do begin
    k1 :=  m22*(ret[j*2]-c1) - m12*(ret[j*2+1]-c2);
    k2 := -m21*(ret[j*2]-c1) + m11*(ret[j*2+1]-c2);
    for i:=0 to 2 do begin
      point[cnum*3+i] := center[i] + k1*Rb[i*4+a1] + k2*Rb[i*4+a2];
    end;
    dep[cnum] := Sa[codeN] - dDOT(@normal2,@point[cnum*3]);
    if dep[cnum] >= 0 then begin
      ret[cnum*2]   := ret[j*2];
      ret[cnum*2+1] := ret[j*2+1];
      inc(cnum);
    end;
  end;
  if cnum < 1 then begin
//    abort;
    exit(-2);       // this should never happen
  end;

  // we can't generate more contacts than we actually have
  if (maxc > cnum) then begin
    maxc := cnum;
  end;
  if (maxc < 1) then begin
    maxc := 1;
  end;

  if (cnum <= maxc) then begin
    if (code<4) then begin
      // we have less contacts than we need, so we use them all
      for j:=0 to cnum-1 do begin
        for i:=0 to 2 do begin
          pointInWorld[i] := point[j*3+i] + pa[i];
        end;
        output.addContactPoint(-normal,pointInWorld,-dep[j]);
      end;
    end else begin
      // we have less contacts than we need, so we use them all
      for j:=0 to cnum-1 do begin
        for i:=0 to 2 do begin
          pointInWorld[i] := point[j*3+i] + pa[i]-normal[i]*dep[j];
        end;
                //pointInWorld[i] = point[j*3+i] + pa[i];
        output.addContactPoint(-normal,pointInWorld,-dep[j]);
      end;
    end;
  end else begin
    // we have more contacts than are wanted, some of them must be culled.
    // find the deepest point, it is always the first contact.
    i1 := 0;
    maxdepth := dep[0];
    for i:=1 to cnum-1 do begin
      if (dep[i] > maxdepth) then begin
        maxdepth := dep[i];
        i1       := i;
      end;
    end;
    cullPoints2 (cnum,ret,maxc,i1,@iret[0]);
    for j:=0 to maxc-1 do begin
//      dContactGeom *con = CONTACT(contact,skip*j);
  //    for (i=0; i<3; i++) con->pos[i] = point[iret[j]*3+i] + pa[i];
    //  con->depth = dep[iret[j]];
      for i:= 0 to 2 do begin
        posInWorld[i] := point[iret[j]*3+i] + pa[i];
      end;
      if (code<4) then begin
        output.addContactPoint(-normal,posInWorld,-dep[iret[j]]);
      end else begin
        output.addContactPoint(-normal,posInWorld-normal*dep[iret[j]],-dep[iret[j]]);
      end;
    end;
    cnum := maxc;
  end;
  return_code := code;
  Result:=cnum;
end;

procedure btBoxBoxDetector.init(const box1, box2: btBoxShape);
begin
  m_box1:=box1;
  m_box2:=box2;
end;

{$HINTS OFF}
procedure btBoxBoxDetector.getClosestPoints(const input: btDCDI_ClosestPointInput; var output: btDCDI_Result; const debugDraw: btIDebugDraw; const swapResults: boolean);
var
    R1,R2:dMatrix3;
    normal : btVector3;
    depth  : btScalar;
    return_code,maxc : integer;
//    n                : integer;
begin
  R1[0]       := input.m_transformA.getBasisV^[0]._x^; R2[0      ] := input.m_transformB.getBasisV^[0]._x^;
  R1[1]       := input.m_transformA.getBasisV^[0]._y^; R2[1      ] := input.m_transformB.getBasisV^[0]._y^;
  R1[2]       := input.m_transformA.getBasisV^[0]._z^; R2[2      ] := input.m_transformB.getBasisV^[0]._z^;
  R1[0+(4*1)] := input.m_transformA.getBasisV^[1]._x^; R2[0+(4*1)] := input.m_transformB.getBasisV^[1]._x^;
  R1[1+(4*1)] := input.m_transformA.getBasisV^[1]._y^; R2[1+(4*1)] := input.m_transformB.getBasisV^[1]._y^;
  R1[2+(4*1)] := input.m_transformA.getBasisV^[1]._z^; R2[2+(4*1)] := input.m_transformB.getBasisV^[1]._z^;
  R1[0+(4*2)] := input.m_transformA.getBasisV^[2]._x^; R2[0+(4*2)] := input.m_transformB.getBasisV^[2]._x^;
  R1[1+(4*2)] := input.m_transformA.getBasisV^[2]._y^; R2[1+(4*2)] := input.m_transformB.getBasisV^[2]._y^;
  R1[2+(4*2)] := input.m_transformA.getBasisV^[2]._z^; R2[2+(4*2)] := input.m_transformB.getBasisV^[2]._z^;

  maxc := 4;
  //writeln(input.m_transformA.dump);
  //writeln(input.m_transformB.dump);
  if gFOS_Debug_SIMUSTEP=143 then begin
    gFOS_Debug_SIMUSTEP:=gFOS_Debug_SIMUSTEP;
  end;
  {$IFDEF FOS_DYNAMICS_DEBUG}
  n:=dBoxBox2 (input.m_transformA.getOriginV^,R1,m_box1.getHalfExtentsWithMargin*2,input.m_transformB.getOriginV^,R2,m_box2.getHalfExtentsWithMargin*2,normal, @depth, return_code, maxc, output);
  write(n);
  if (n<>0) and (n<>4) then begin
    n := n;
  end;
  {$ENDIF}
  dBoxBox2 (input.m_transformA.getOriginV^,R1,m_box1.getHalfExtentsWithMargin*2,input.m_transformB.getOriginV^,R2,m_box2.getHalfExtentsWithMargin*2,normal, @depth, return_code, maxc, output);
end;
{$HINTS ON}

{ btOverlapCallback }

//function btEmptyCreateFuncClass.CreateCollisionAlgorithm(const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;
//begin
//    //void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btEmptyAlgorithm));
//    //return new(mem) btEmptyAlgorithm(ci);
//  result:=info.m_dispatcher1.allocateCollisionAlgorithm(btEmptyAlgorithm);
//  result.Init(info);
//end;

{ btCollisionAlgorithmCreateFunc }

constructor btCollisionAlgorithmCreateFunc.Create(const swapped:boolean=false);
begin
  m_swapped:=swapped;
end;


{ btConvexConvexAlgorithm }

procedure segmentsClosestPoints(out ptsVector,offsetA,offsetB:btVector3;out tA,tB:btScalar;const translation,dirA:btVector3;const hlenA:btScalar;const dirB:btVector3;const hlenB:btScalar);FOS_INLINE;
var dirA_dot_dirB,dirA_dot_trans,dirB_dot_trans,denom:btScalar;
begin
  // compute the parameters of the closest points on each line segment
  dirA_dot_dirB  := btDot(dirA,dirB);
  dirA_dot_trans := btDot(dirA,translation);
  dirB_dot_trans := btDot(dirB,translation);
  denom          := 1 - dirA_dot_dirB * dirA_dot_dirB;
  if denom = 0 then begin
    tA := 0;
  end else begin
    tA := ( dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB ) / denom;
    if tA < -hlenA  then begin
      tA := -hlenA;
    end else
    if ( tA > hlenA ) then begin
      tA := hlenA;
    end;
  end;
  tB := tA * dirA_dot_dirB - dirB_dot_trans;
  if tB < -hlenB then begin
    tB := -hlenB;
    tA := tB * dirA_dot_dirB + dirA_dot_trans;
    if tA < -hlenA then begin
      tA := -hlenA;
    end else
    if ( tA > hlenA ) then begin
      tA := hlenA;
    end;
  end else if ( tB > hlenB ) then begin
    tB := hlenB;
    tA := tB * dirA_dot_dirB + dirA_dot_trans;
    if tA < -hlenA then begin
      tA := -hlenA;
    end else
    if ( tA > hlenA ) then begin
      tA := hlenA;
    end;
  end;
  // compute the closest points relative to segment centers.
  offsetA   := dirA * tA;
  offsetB   := dirB * tB;
  ptsVector := translation - offsetA + offsetB;
end;

function capsuleCapsuleDistance( var   normalOnB,pointOnB        : btVector3;
                                 const capsuleLengthA,capsuleRadiusA,capsuleLengthB,capsuleRadiusB : btScalar;
	                         const capsuleAxisA,capsuleAxisB : integer;
	                         const transformA,transformB     : btTransform;
                                 const distanceThreshold         : btScalar ): btScalar;

var translation,directionA,translationA,directionB,translationB : btVector3;
    ptsVector,q     : btVector3 ;        // the vector between the closest points
    offsetA,offsetB : btVector3;   // offsets from segment centers to their closest points
    tA, tB          : btScalar;                // parameters on line segment
    distance,lenSqr : btScalar;
begin
  directionA   := transformA.GetBasisV^.getColumn(capsuleAxisA);
  translationA := transformA.getOriginV^;
  directionB   := transformB.GetBasisV^.getColumn(capsuleAxisB);
  translationB := transformB.getOriginV^;

  // translation between centers
  translation := translationB - translationA;
  // compute the closest points of the capsule line segments
  segmentsClosestPoints( ptsVector, offsetA, offsetB, tA, tB, translation,directionA, capsuleLengthA, directionB, capsuleLengthB );
  distance := ptsVector.length - capsuleRadiusA - capsuleRadiusB;
  if distance > distanceThreshold then begin
    exit(distance);
  end;
  lenSqr := ptsVector.length2;
  if lenSqr <= (SIMD_EPSILON*SIMD_EPSILON) then begin //degenerate case where 2 capsules are likely at the same location: take a vector tangential to 'directionA'
    btPlaneSpace1(directionA,normalOnB,q);
  end else begin // compute the contact normal
    normalOnB := ptsVector * -btRecipSqrt(lenSqr);
  end;
  pointOnB := transformB.getOriginV^+offsetB + normalOnB * capsuleRadiusB;
  Result := distance;
end;

type

  { btPerturbedContactResult }

  btPerturbedContactResult = class(btManifoldResult)
    m_originalManifoldResult : btManifoldResult;
    m_transformA             : btTransform;
    m_transformB             : btTransform;
    m_unPerturbedTransform   : btTransform;
    m_perturbA               : boolean;
    m_debugDrawer            : btIDebugDraw;
  public
    constructor create(const originalResult:btManifoldResult;const transformA,transformB,unPerturbedTransform : btTransform;const perturbA:boolean;const debugDrawer:btIDebugDraw);
    procedure   addContactPoint(const normalOnBInWorld, pointInWorld: btVector3; const orgDepth: btScalar); override;
  end;

 { btPerturbedContactResult }
  constructor btPerturbedContactResult.create(const originalResult: btManifoldResult; const transformA, transformB, unPerturbedTransform: btTransform; const perturbA: boolean;const debugDrawer: btIDebugDraw);
  begin
    m_originalManifoldResult := originalResult;
    m_transformA             := transformA;
    m_transformB             := transformB;
    m_unPerturbedTransform   := unPerturbedTransform;
    m_perturbA               := perturbA;
    m_debugDrawer            := debugDrawer;
  end;

  procedure btPerturbedContactResult.addContactPoint(const normalOnBInWorld, pointInWorld: btVector3; const orgDepth: btScalar);
  var     endPtOrg,endPt,startPt  : btVector3;
          newDepth                : btScalar;
  begin
    if m_perturbA then begin
      endPtOrg := pointInWorld + normalOnBInWorld*orgDepth;
      endPt    := (m_unPerturbedTransform*m_transformA.inverse).opTrans(endPtOrg);
      newDepth := (endPt -  pointInWorld).dot(normalOnBInWorld);
      startPt  := endPt+normalOnBInWorld*newDepth;
    end else begin
      endPt    := pointInWorld + normalOnBInWorld*orgDepth;
      startPt  := (m_unPerturbedTransform*m_transformB.inverse()).opTrans(pointInWorld);
      newDepth := (endPt -  startPt).dot(normalOnBInWorld);
    end;
  //#define DEBUG_CONTACTS 1
  {$ifdef DEBUG_CONTACTS}
    m_debugDrawer.drawLine(startPt,endPt,btVector3(1,0,0));
    m_debugDrawer.drawSphere(startPt,0.05,btVector3(0,1,0));
    m_debugDrawer.drawSphere(endPt,0.05,btVector3(0,0,1));
  {$endif} //DEBUG_CONTACTS
    m_originalManifoldResult.addContactPoint(normalOnBInWorld,startPt,newDepth);
  end;


//
// Convex-Convex collision algorithm
//

procedure btConvexConvexAlgorithm.Init(const mf: btPersistentManifold; const ci: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject;
                                       const simplexSolver: btSimplexSolverInterface; const pdSolver: btConvexPenetrationDepthSolver; const numPerturbationIterations, minimumPointsPerturbationThreshold: integer);
begin
  inherited init (ci,body0,body1);
  m_simplexSolver      := simplexSolver;
  m_pdSolver           := pdSolver;
  m_ownManifold        := false;
  m_manifoldPtr        := mf;
  m_lowLevelOfDetail   := false;
  {$ifdef USE_SEPDISTANCE_UTIL2}
  m_sepDistance((static_cast<btConvexShape*>(body0->getCollisionShape()))->getAngularMotionDisc(),
  			  (static_cast<btConvexShape*>(body1->getCollisionShape()))->getAngularMotionDisc()),
  {$endif}
  m_numPerturbationIterations           := numPerturbationIterations;
  m_minimumPointsPerturbationThreshold  := minimumPointsPerturbationThreshold;
end;

destructor btConvexConvexAlgorithm.Destroy;
begin
  if m_ownManifold then begin
    if assigned(m_manifoldPtr) then begin
      m_dispatcher.releaseManifold(m_manifoldPtr);
    end;
  end;
  inherited Destroy;
end;

//
// Convex-Convex collision algorithm
//
const cAngleLimit:btScalar= 0.125 * SIMD_PI;

procedure btConvexConvexAlgorithm.processCollision(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult);
var min0,min1                     : btConvexShape;
    normalOnB,pointOnBWorld       : btVector3;
    input                         : btDCDI_ClosestPointInput;//<=> btGjkPairDetector::ClosestPointInput input;
    gjkPairDetector               : btGjkPairDetector;
    v0,v1,sepNormalWorldSpace     : btVector3;
    perturbeA                     : boolean;
    perturbeAngle,radiusA,radiusB : btScalar;
    unPerturbedTransform          : btTransform;
    perturbeRot,rotq              : btQuaternion;
    i                             : Integer;
    iterationAngle                : btScalar;
    perturbedResultOut            : btPerturbedContactResult;

    procedure _capsulecapsule;FOS_INLINE;
    var capsuleA,capsuleB:btCapsuleShape;
      //  localScalingA,localScalingB:btVector3;
        dist,threshold : btScalar;
    begin
      capsuleA           := btCapsuleShape(min0);
      capsuleB           := btCapsuleShape(min1);
      //localScalingA      := capsuleA.getLocalScaling;
      //localScalingB      := capsuleB.getLocalScaling;
      threshold          := m_manifoldPtr.getContactBreakingThreshold;
      dist               := capsuleCapsuleDistance(normalOnB,pointOnBWorld,capsuleA.getHalfHeight,capsuleA.getRadius,
                                                   capsuleB.getHalfHeight,capsuleB.getRadius,capsuleA.getUpAxis,capsuleB.getUpAxis,
                                                   body0.getWorldTransformP^,body1.getWorldTransformP^,threshold);
      if dist<threshold then begin
        btAssert(normalOnB.length2>=(SIMD_EPSILON*SIMD_EPSILON));
        resultOut.addContactPoint(normalOnB,pointOnBWorld,dist);
      end;
      resultOut.refreshContactPoints;
    end;
begin
  if not assigned(m_manifoldPtr) then begin
    //swapped?
    m_manifoldPtr := m_dispatcher.getNewManifold(body0,body1);
    m_ownManifold := true;
  end;
  resultOut.setPersistentManifold(m_manifoldPtr);
  //comment-out next line to test multi-contact generation
  //resultOut->getPersistentManifold()->clearManifold();

  min0 := btConvexShape(body0.getCollisionShape);
  min1 := btConvexShape(body1.getCollisionShape);

  {$ifndef BT_DISABLE_CAPSULE_CAPSULE_COLLIDER}
  if (min0.getShapeType = CAPSULE_SHAPE_PROXYTYPE) and (min1.getShapeType = CAPSULE_SHAPE_PROXYTYPE) then begin
    _CapsuleCapsule;
    exit;
  end;
  {$endif} //BT_DISABLE_CAPSULE_CAPSULE_COLLIDER


  {$ifdef USE_SEPDISTANCE_UTIL2}
  if dispatchInfo.m_useConvexConservativeDistanceUtil then begin
    m_sepDistance.updateSeparatingDistance(body0.getWorldTransform,body1.getWorldTransform);
  end;
  if (not dispatchInfo.m_useConvexConservativeDistanceUtil) or (m_sepDistance.getConservativeSeparatingDistance<=) then
  {$endif} //USE_SEPDISTANCE_UTIL2
  begin
    gjkPairDetector.init(min0,min1,m_simplexSolver,m_pdSolver);
    //TODO: if (dispatchInfo.m_useContinuous)
    gjkPairDetector.setMinkowskiA(min0);
    gjkPairDetector.setMinkowskiB(min1);

    {$ifdef USE_SEPDISTANCE_UTIL2}
    if (dispatchInfo.m_useConvexConservativeDistanceUtil) then begin
      input.m_maximumDistanceSquared := BT_LARGE_FLOAT;
    end else
    {$endif} //USE_SEPDISTANCE_UTIL2
    begin
      input.m_maximumDistanceSquared := min0.getMargin + min1.getMargin + m_manifoldPtr.getContactBreakingThreshold;
      input.m_maximumDistanceSquared *= input.m_maximumDistanceSquared;
    end;
    input.m_stackAlloc := dispatchInfo.m_stackAllocator;
    input.m_transformA := body0.getWorldTransformP^;
    input.m_transformB := body1.getWorldTransformP^;

    gjkPairDetector.getClosestPoints(input,btDCDI_Result(resultOut),dispatchInfo.m_debugDraw);

    {$ifdef USE_SEPDISTANCE_UTIL2}
    sepDist := 0;
    if dispatchInfo.m_useConvexConservativeDistanceUtil then begin
              sepDist = gjkPairDetector.getCachedSeparatingDistance();
            if (sepDist>SIMD_EPSILON)
            {
                    sepDist += dispatchInfo.m_convexConservativeDistanceThreshold;
                    //now perturbe directions to get multiple contact points

            }
    end;
    {$endif} //USE_SEPDISTANCE_UTIL2

    //now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects
    //perform perturbation when more then 'm_minimumPointsPerturbationThreshold' points
    if (m_numPerturbationIterations and resultOut.getPersistentManifold.getNumContacts) < m_minimumPointsPerturbationThreshold then begin
      sepNormalWorldSpace := gjkPairDetector.getCachedSeparatingAxis.normalized;
      btPlaneSpace1(sepNormalWorldSpace,v0,v1);
      perturbeA := true;
      radiusA   := min0.getAngularMotionDisc;
      radiusB   := min1.getAngularMotionDisc;
      if radiusA < radiusB then begin
        perturbeAngle := gContactBreakingThreshold / radiusA;
        perturbeA     := true;
      end else begin
        perturbeAngle := gContactBreakingThreshold / radiusB;
        perturbeA     := false;
      end;
      if (perturbeAngle > cAngleLimit) then begin
        perturbeAngle := cAngleLimit;
      end;
      if perturbeA then begin
        unPerturbedTransform := input.m_transformA;
      end else begin
        unPerturbedTransform := input.m_transformB;
      end;

      for i:=0 to m_numPerturbationIterations-1 do begin
        if v0.length2>SIMD_EPSILON then begin
          perturbeRot.initQ(v0,perturbeAngle);
          iterationAngle := i*(SIMD_2_PI/btScalar(m_numPerturbationIterations));
          rotq.initq(sepNormalWorldSpace,iterationAngle);
        end;
        if (perturbeA) then begin
          input.m_transformA.setBasis(btMatrix3x3.InitS(rotq.inverse*perturbeRot*rotq)*body0.getWorldTransformP^.GetBasisV^);
          input.m_transformB := body1.getWorldTransformP^;
  {$ifdef DEBUG_CONTACTS}
          dispatchInfo.m_debugDraw->drawTransform(input.m_transformA,10.0);
  {$endif} //DEBUG_CONTACTS
        end else begin
          input.m_transformA := body0.getWorldTransformP^;
          input.m_transformB.setBasis(btMatrix3x3.InitS(rotq.inverse*perturbeRot*rotq)*body1.getWorldTransformP^.GetBasisV^);
  {$ifdef DEBUG_CONTACTS}
          dispatchInfo.m_debugDraw->drawTransform(input.m_transformB,10.0);
  {$endif}
        end;
        perturbedResultOut := btPerturbedContactResult.create(resultOut,input.m_transformA,input.m_transformB,unPerturbedTransform,perturbeA,dispatchInfo.m_debugDraw);
        gjkPairDetector.getClosestPoints(input,btDCDI_Result(perturbedResultOut),dispatchInfo.m_debugDraw);
      end
    end;
    {$ifdef USE_SEPDISTANCE_UTIL2}
    if dispatchInfo.m_useConvexConservativeDistanceUtil and (sepDist>SIMD_EPSILON) then begin
            m_sepDistance.initSeparatingDistance(gjkPairDetector.getCachedSeparatingAxis(),sepDist,body0->getWorldTransform(),body1->getWorldTransform());
    end;
    {$endif //USE_SEPDISTANCE_UTIL2}
  end;
  if m_ownManifold then begin
    resultOut.refreshContactPoints;
  end;
end;

const cDisableCCD:boolean=false;

{$HINTS OFF}
function btConvexConvexAlgorithm.calculateTimeOfImpact(const col0, col1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar;
var resultFraction,squareMot0,squareMot1 : btScalar;
    convex0,convex1                      : btConvexShape;
    sphere1,sphere0                      : btSphereShape;
    voronoiSimplex                       : btVoronoiSimplexSolver;
    lresult                              : btConvexCastResult;
    ccd1                                 : btGjkConvexCast;
   procedure _Store;FOS_INLINE;
   begin
     if (col0.getHitFraction > lresult.m_fraction) then begin
       col0.setHitFraction(lresult.m_fraction);
     end;
     if (col1.getHitFraction > lresult.m_fraction) then begin
       col1.setHitFraction(lresult.m_fraction);
     end;
     if (resultFraction > lresult.m_fraction) then begin
       resultFraction := lresult.m_fraction;
     end;
   end;
begin
  ///Rather then checking ALL pairs, only calculate TOI when motion exceeds threshold
  ///Linear motion for one of objects needs to exceed m_ccdSquareMotionThreshold
  ///col0->m_worldTransform,
  resultFraction := 1;
  squareMot0     := (col0.getInterpolationWorldTransformP^.getOriginV^ - col0.getWorldTransformP^.getOriginV^).length2;
  squareMot1     := (col1.getInterpolationWorldTransformP^.getOriginV^ - col1.getWorldTransformP^.getOriginV^).length2;
  if (squareMot0 < col0.getCcdSquareMotionThreshold) and (squareMot1 < col1.getCcdSquareMotionThreshold) then begin
    exit(resultFraction);
  end;
  if cDisableCCD then begin
    exit(1);
  end;
  //An adhoc way of testing the Continuous Collision Detection algorithms
  //One object is approximated as a sphere, to simplify things
  //Starting in penetration should report no time of impact
  //For proper CCD, better accuracy and handling of 'allowed' penetration should be added
  //also the mainloop of the physics should have a kind of toi queue (something like Brian Mirtich's application of Timewarp for Rigidbodies)

  /// Convex0 against sphere for Convex1
  begin
    convex0 := btConvexShape(col0.getCollisionShape);
    sphere1 := btSphereShape.Create(col1.getCcdSweptSphereRadius); //todo: allow non-zero sphere sizes, for better approximation
    lresult := btConvexCastResult.create;
    voronoiSimplex:=btVoronoiSimplexSolver.Create;
    try
      //SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
      ///Simplification, one object is simplified as a sphere
      ccd1 := btGjkConvexCast.create( convex0 ,sphere1,voronoiSimplex);
      //ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
      if ccd1.calcTimeOfImpact(col0.getWorldTransformP^,col0.getInterpolationWorldTransformP^,col1.getWorldTransformP^,col1.getInterpolationWorldTransformP^,lresult) then begin
        //store result.m_fraction in both bodies
        _Store;
      end;
     finally
       sphere1.free;
       lresult.free;
       voronoiSimplex.Free;
       ccd1.free;
     end;
  end;
  /// Sphere (for convex0) against Convex1
  begin
    convex1 := btConvexShape(col1.getCollisionShape);
    sphere0 := btSphereShape.Create(col0.getCcdSweptSphereRadius); //todo: allow non-zero sphere sizes, for better approximation
    lresult := btConvexCastResult.create;
    voronoiSimplex:=btVoronoiSimplexSolver.Create;
    try
      //SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
      ///Simplification, one object is simplified as a sphere
      ccd1 := btGjkConvexCast.create(sphere0,convex1,voronoiSimplex);
      //ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
      if ccd1.calcTimeOfImpact(col0.getWorldTransformP^,col0.getInterpolationWorldTransformP^,col1.getWorldTransformP^,col1.getInterpolationWorldTransformP^,lresult) then begin
        _Store;
      end;
     finally
      sphere0.Free;
      lresult.free;
      voronoiSimplex.free; //FOSTODO Think about allocations
      ccd1.Free;
     end;
  end;
  Result := resultFraction;
end;
{$HINTS ON}

procedure btConvexConvexAlgorithm.getAllContactManifolds(const manifoldArray: btManifoldArray);
begin
  ///should we use m_ownManifold to avoid adding duplicates?
  if assigned(m_manifoldPtr) and m_ownManifold then begin
          manifoldArray.push_back(m_manifoldPtr);
  end;
end;

procedure btConvexConvexAlgorithm.setLowLevelOfDetail(const useLowLevel: boolean);
begin
  m_lowLevelOfDetail := useLowLevel;
end;



function btConvexConvexAlgorithm.getManifold: btPersistentManifold;
begin
  result := m_manifoldPtr;
end;

{ btConvexConvexAlgorithmCreateFunc }

constructor btConvexConvexAlgorithmCreateFunc.Create(const simplexSolver: btSimplexSolverInterface; const pdSolver: btConvexPenetrationDepthSolver);
begin
  m_numPerturbationIterations          := 0;
  m_minimumPointsPerturbationThreshold := 3;
  m_simplexSolver                      := simplexSolver;
  m_pdSolver                           := pdSolver;
end;

function btConvexConvexAlgorithmCreateFunc.CreateCollisionAlgorithm(const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;
begin
  result:=info.m_dispatcher1.allocateCollisionAlgorithm(btConvexConvexAlgorithm);
  btConvexConvexAlgorithm(result).Init(info.m_manifold,info,body0,body1,m_simplexSolver,m_pdSolver,m_numPerturbationIterations,m_minimumPointsPerturbationThreshold);
end;

{ btConvexTriangleCallback }

constructor btConvexTriangleCallback.create(const dispatcher: btDispatcher; const body0, body1: btCollisionObject; const isSwapped: boolean);
begin
  m_dispatcher      := dispatcher;
  m_dispatchInfoPtr := nil; // =0 (but is object by now)
  if isSwapped then begin
    m_convexBody := body1;
  end else begin
    m_convexBody := body0;
  end;
  if isSwapped then begin
    m_triBody := body0;
  end else begin
    m_triBody := body1;
  end;
  // create the manifold from the dispatcher 'manifold pool'
  m_manifoldPtr := m_dispatcher.getNewManifold(m_convexBody,m_triBody);
  clearCache;
end;

destructor btConvexTriangleCallback.destroy;
begin
  clearCache;
  m_dispatcher.releaseManifold(m_manifoldPtr);
  inherited;
end;

procedure btConvexTriangleCallback.setTimeStepAndCounters(const collisionMarginTriangle: btScalar; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult);
var   convexInTriangleSpace : btTransform;
      convexShape           : btCollisionShape;
      extraMargin           : btScalar;
      extra                 : btVector3;
begin
  m_dispatchInfoPtr         := @dispatchInfo;
  m_collisionMarginTriangle := collisionMarginTriangle;
  m_resultOut               := resultOut;
  //recalc aabbs
  convexInTriangleSpace := m_triBody.getWorldTransformP^.inverse * m_convexBody.getWorldTransformP^;
  convexShape           := btCollisionShape(m_convexBody.getCollisionShape);
  //CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody->m_collisionShape);
  convexShape.getAabb(convexInTriangleSpace,m_aabbMin,m_aabbMax);
  extraMargin := collisionMarginTriangle;
  extra.InitSame(extraMargin);
  m_aabbMax += extra;
  m_aabbMin -= extra;
end;

procedure btConvexTriangleCallback.processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer);
var ci       : btCollisionAlgorithmConstructionInfo;
    ob       : btCollisionObject;
    color    : btVector3;
    tr       : btTransform;
    tm       : btTriangleShape;
    tmpShape : btCollisionShape;
    colAlgo  : btCollisionAlgorithm;
begin
  //just for debugging purposes
  //printf("triangle %d",m_triangleCount++);

  //aabb filter is already applied!
  ci.m_dispatcher1  := m_dispatcher;
  ob := btCollisionObject(m_triBody);

  ///debug drawing of the overlapping triangles
  if assigned(m_dispatchInfoPtr) and (assigned(m_dispatchInfoPtr^.m_debugDraw)) and (DBG_DrawWireframe in m_dispatchInfoPtr^.m_debugDraw.getDebugMode) then begin
   color.init(1,1,0);
   tr := ob.getWorldTransformP^;
   m_dispatchInfoPtr^.m_debugDraw.drawLine(tr.opTrans(triangle[0]),tr.opTrans(triangle[1]),color);
   m_dispatchInfoPtr^.m_debugDraw.drawLine(tr.opTrans(triangle[1]),tr.opTrans(triangle[2]),color);
   m_dispatchInfoPtr^.m_debugDraw.drawLine(tr.opTrans(triangle[2]),tr.opTrans(triangle[0]),color);
   //btVector3 center = triangle[0] + triangle[1]+triangle[2];
   //center *= btScalar(0.333333);
   //m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]),tr(center),color);
   //m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]),tr(center),color);
   //m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]),tr(center),color);
  end;
  //btCollisionObject* colObj = static_cast<btCollisionObject*>(m_convexProxy->m_clientObject);
  if m_convexBody.getCollisionShape.isConvex then begin
    tm.Create(triangle[0],triangle[1],triangle[2]);
    tm.setMargin(m_collisionMarginTriangle);
    tmpShape := ob.getCollisionShape();
    ob.internalSetTemporaryCollisionShape(tm);
    colAlgo := ci.m_dispatcher1.findAlgorithm(m_convexBody,m_triBody,m_manifoldPtr);
    if m_resultOut.getBody0Internal = m_triBody then begin
      m_resultOut.setShapeIdentifiersA(partId,triangleIndex);
    end else begin
      m_resultOut.setShapeIdentifiersB(partId,triangleIndex);
    end;
    colAlgo.processCollision(m_convexBody,m_triBody,m_dispatchInfoPtr^,m_resultOut);
    //colAlgo->~btCollisionAlgorithm(); FOS
    ci.m_dispatcher1.freeCollisionAlgorithm(colAlgo);
    ob.internalSetTemporaryCollisionShape(tmpShape);
  end;
end;

procedure btConvexTriangleCallback.clearcache;
begin
  m_dispatcher.clearManifold(m_manifoldPtr);
end;

function btConvexTriangleCallback.getAabbMin: btVector3;
begin
  result := m_aabbMin;
end;

function btConvexTriangleCallback.getAabbMax: btVector3;
begin
  result := m_aabbMax;
end;

{ btConvexConcaveCollisionAlgorithm }

destructor btConvexConcaveCollisionAlgorithm.Destroy;
begin
  m_btConvexTriangleCallback.Free;
  inherited;
end;

procedure btConvexConcaveCollisionAlgorithm.Init(const ci: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject; const isSwapped: boolean);
begin
  inherited init(ci,body0,body1);
  m_isSwapped := isSwapped;
  m_btConvexTriangleCallback := btConvexTriangleCallback.create(ci.m_dispatcher1,body0,body1,isSwapped);
end;

procedure btConvexConcaveCollisionAlgorithm.processCollision(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult);
var triOb,triBody,convexBody : btCollisionObject;
    concaveShape             : btConcaveShape;
    collisionMarginTriangle  : btScalar;
begin
  if m_isSwapped then begin
    convexBody := body1;
    triBody    := body0;
  end else begin
    convexBody := body0;
    triBody    := body1;
  end;
  if triBody.getCollisionShape.isConcave then begin
    triOb := triBody;
    concaveShape := btConcaveShape(triOb.getCollisionShape);
    if convexBody.getCollisionShape.isConvex then begin
      collisionMarginTriangle := concaveShape.getMargin;
      resultOut.setPersistentManifold(m_btConvexTriangleCallback.m_manifoldPtr);
      m_btConvexTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle,dispatchInfo,resultOut);
      //Disable persistency. previously, some older algorithm calculated all contacts in one go, so you can clear it here.
      //m_dispatcher->clearManifold(m_btConvexTriangleCallback.m_manifoldPtr);
      m_btConvexTriangleCallback.m_manifoldPtr.setBodies(convexBody,triBody);
      concaveShape.processAllTriangles(m_btConvexTriangleCallback,m_btConvexTriangleCallback.getAabbMin,m_btConvexTriangleCallback.getAabbMax);
      resultOut.refreshContactPoints;
    end;
  end;
end;

type

  { LocalTriangleSphereCastCallback }

  LocalTriangleSphereCastCallback=class(btTriangleCallback)
    m_ccdSphereFromTrans,
    m_ccdSphereToTrans,
    m_meshTransform      : btTransform;
    m_ccdSphereRadius    : btScalar;
    m_hitFraction        : btScalar;
  public
    constructor create(const from,too :btTransform ;const ccdSphereRadius,hitFraction:btScalar);
    procedure   processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer); override;
  end;

{ LocalTriangleSphereCastCallback }

constructor LocalTriangleSphereCastCallback.create(const from, too: btTransform; const ccdSphereRadius, hitFraction: btScalar);
begin
  m_ccdSphereFromTrans   := from;
  m_ccdSphereToTrans     := too;
  m_ccdSphereRadius      := ccdSphereRadius;
  m_hitFraction          := hitFraction;
end;

{$HINTS OFF}
procedure LocalTriangleSphereCastCallback.processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer);
var      ident  : btTransform;
     castResult : btConvexCastResult;
     pointShape : btSphereShape;
     triShape   : btTriangleShape;
  simplexSolver : btVoronoiSimplexSolver;
  convexCaster  : btSubsimplexConvexCast;
begin
  //do a swept sphere for now
  ident.setIdentity();
  castResult := btConvexCastResult.create;
  castResult.m_fraction := m_hitFraction;
  pointShape := btSphereShape.Create(m_ccdSphereRadius);
  triShape   := btTriangleShape.Create(triangle[0],triangle[1],triangle[2]);
  simplexSolver := btVoronoiSimplexSolver.Create;
  convexCaster  := btSubSimplexConvexCast.Create(pointShape,triShape,simplexSolver);
  //GjkConvexCast convexCaster(&pointShape,convexShape,&simplexSolver);
  //ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
  //local space?

  if convexCaster.calcTimeOfImpact(m_ccdSphereFromTrans,m_ccdSphereToTrans,ident,ident,castResult) then begin
    if m_hitFraction > castResult.m_fraction then begin
      m_hitFraction := castResult.m_fraction;
    end;
  end;
end;
{$HINTS ON}

{$HINTS OFF}
function btConvexConcaveCollisionAlgorithm.calculateTimeOfImpact(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar;
var
    triBody,convexBody : btCollisionObject;
//    concaveShape             : btConcaveShape;
//    collisionMarginTriangle  : btScalar;
    squareMot0,ccdRadius0    : btScalar;
    curHitFraction           : btScalar;
    triInv,
    convexFromLocal,
    convexToLocal            : btTransform;
    rayAabbMin,rayAabbMax    : btVector3;
    concavebody              : btCollisionObject;
    raycastCallback          : LocalTriangleSphereCastCallback;
    triangleMesh             : btConcaveShape;
begin
  if m_isSwapped then begin
    convexBody := body1;
    triBody    := body0;
  end else begin
    convexBody := body0;
    triBody    := body1;
  end;
  //quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)
  //only perform CCD above a certain threshold, this prevents blocking on the long run
  //because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
  squareMot0 :=  (convexbody.getInterpolationWorldTransformP^.getOriginV^ - convexbody.getWorldTransformP^.getOriginV^).length2;
  if squareMot0 < convexbody.getCcdSquareMotionThreshold then begin
    exit(1);
  end;
  //const btVector3& from = convexbody->m_worldTransform.getOrigin();
  //btVector3 to = convexbody->m_interpolationWorldTransform.getOrigin();
  //todo: only do if the motion exceeds the 'radius'
  triInv          := triBody.getWorldTransformP^.inverse;
  convexFromLocal := triInv * convexbody.getWorldTransformP^;
  convexToLocal   := triInv * convexbody.getInterpolationWorldTransformP^;

  if (triBody.getCollisionShape.isConcave) then begin
    rayAabbMin := convexFromLocal.getOriginV^;
    rayAabbMin.setMin(convexToLocal.getOriginV^);
    rayAabbMax := convexFromLocal.getOriginV^;
    rayAabbMax.setMax(convexToLocal.getOriginV^);
    ccdRadius0 := convexbody.getCcdSweptSphereRadius;
    rayAabbMin -= btVector3.inits(ccdRadius0,ccdRadius0,ccdRadius0);
    rayAabbMax += btVector3.inits(ccdRadius0,ccdRadius0,ccdRadius0);
    curHitFraction := btScalar(1); //is this available?
    raycastCallback := LocalTriangleSphereCastCallback.create(convexFromLocal,convexToLocal,convexbody.getCcdSweptSphereRadius,curHitFraction);
    try
      raycastCallback.m_hitFraction := convexbody.getHitFraction;
      concavebody                   := triBody;
      triangleMesh                  := btConcaveShape(concavebody.getCollisionShape);
      if assigned(triangleMesh) then begin
        triangleMesh.processAllTriangles(raycastCallback,rayAabbMin,rayAabbMax);
      end;
      if (raycastCallback.m_hitFraction < convexbody.getHitFraction) then begin
        convexbody.setHitFraction( raycastCallback.m_hitFraction);
        result:=raycastCallback.m_hitFraction;
        exit;
      end;
    finally
      raycastCallback.free;
    end;
  end;
  result := btScalar(1);
end;
{$HINTS ON}

procedure btConvexConcaveCollisionAlgorithm.getAllContactManifolds(const manifoldArray: btManifoldArray);
begin
  if assigned(m_btConvexTriangleCallback.m_manifoldPtr) then begin
    manifoldArray.push_back(m_btConvexTriangleCallback.m_manifoldPtr);
  end;
end;

procedure btConvexConcaveCollisionAlgorithm.clearchache;
begin
  m_btConvexTriangleCallback.clearCache;
end;

{ btConvexConcaveCollisionAlgorithmCreateFunc }

function btConvexConcaveCollisionAlgorithmCreateFuncNormAndSwapped.CreateCollisionAlgorithm(const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;
begin
  result:=info.m_dispatcher1.allocateCollisionAlgorithm(btConvexConcaveCollisionAlgorithm);
  btConvexConcaveCollisionAlgorithm(result).Init(info,body0,body1,m_swapped);
end;

{ btEmptyAlgorithmCreateFunc }

{$HINTS OFF}
function btEmptyAlgorithmCreateFunc.CreateCollisionAlgorithm(const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;
begin
  result:=info.m_dispatcher1.allocateCollisionAlgorithm(btEmptyAlgorithm);
end;
{$HINTS ON}

{ btBoxBoxCreateFunc }

function btBoxBoxCollisionAlgorithmCreateFunc.CreateCollisionAlgorithm(const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;
begin
  result:=info.m_dispatcher1.allocateCollisionAlgorithm(btBoxBoxCollisionAlgorithm);
  btBoxBoxCollisionAlgorithm(result).Init(info,body0,body1);
end;

{ btCompoundCollisionAlgorithm }

procedure btCompoundCollisionAlgorithm.removeChildAlgorithms;
var i,numChildren : integer;
begin
  numChildren := m_childCollisionAlgorithms.size;
  for i :=0 to numChildren-1 do begin
    if assigned(m_childCollisionAlgorithms[i]) then begin
     // m_childCollisionAlgorithms[i]->~btCollisionAlgorithm(); FOSCHECK
      m_dispatcher.freeCollisionAlgorithm(m_childCollisionAlgorithms[i]^);
    end;
  end;
end;

procedure btCompoundCollisionAlgorithm.preallocateChildAlgorithms(const body0, body1: btCollisionObject);
var otherObj,colObj : btCollisionObject;
    compoundShape   : btCompoundShape;
    numChildren     : LongInt;
    tmpShape,childShape : btCollisionShape;
    i: Integer;
begin
  if m_isSwapped then begin
    colObj   := body1;
    otherObj := body0;
  end else begin
    colObj   := body0;
    otherObj := body1;
  end;
  btAssert (colObj.getCollisionShape.isCompound);
  compoundShape := btCompoundShape(colObj.getCollisionShape);
  numChildren   := compoundShape.getNumChildShapes;
  m_childCollisionAlgorithms.Setlength(numChildren);
  for i :=0 to numChildren-1 do begin
    if assigned(compoundShape.getDynamicAabbTree) then begin
      m_childCollisionAlgorithms[i]^ := nil;
    end else begin
      tmpShape    := colObj.getCollisionShape;
      childShape  := compoundShape.getChildShape(i);
      colObj.internalSetTemporaryCollisionShape(childShape);
      m_childCollisionAlgorithms[i]^ := m_dispatcher.findAlgorithm(colObj,otherObj,m_sharedManifold);
      colObj.internalSetTemporaryCollisionShape( tmpShape );
    end;
  end;
end;

procedure btCompoundCollisionAlgorithm.Init(const ci: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject; const isSwapped: boolean);
var colObj:btCollisionObject;
    compoundShape:btCompoundShape;
begin
  inherited   init(ci,body0,body1);
  m_isSwapped      := isSwapped;
  m_sharedManifold := ci.m_manifold;
  m_ownsManifold   := false;
  if isSwapped then begin
    colObj := body1;
  end else begin
    colObj := body0;
  end;
  btAssert(colObj.getCollisionShape.isCompound);
  compoundShape := btCompoundShape(colObj.getCollisionShape);
  m_compoundShapeRevision := compoundShape.getUpdateRevision;
  preallocateChildAlgorithms(body0,body1);
end;

destructor btCompoundCollisionAlgorithm.Destroy;
begin
  removeChildAlgorithms;
  inherited;
end;

type

  { btCompoundLeafCallback }

  btCompoundLeafCallback=class(btDbvt_ICollide)
  public
    m_compoundColObj : btCollisionObject;
    m_otherObj       : btCollisionObject;
    m_dispatcher     : btDispatcher;
    m_dispatchInfo   : PbtDispatcherInfo;
    m_resultOut      : btManifoldResult;
    m_childCollisionAlgorithms : PbtCollisionAlgorithm; //FOS - Array ?
    m_sharedManifold : btPersistentManifold;
    constructor      Create             (const compoundObj,otherObj:btCollisionObject;const dispatcher:btDispatcher; const dispatchInfo:PbtDispatcherInfo;var resultOut:btManifoldResult;const childCollisionAlgorithms:PbtCollisionAlgorithm;const sharedManifold:btPersistentManifold);
    procedure        ProcessChildShape  (const childShape:btCollisionShape ; const index:integer);
    procedure        Process            (const leaf: PbtDbvtNode); override;
  end;

{ btCompoundLeafCallback }

constructor btCompoundLeafCallback.Create(const compoundObj, otherObj: btCollisionObject; const dispatcher: btDispatcher; const dispatchInfo: PbtDispatcherInfo; var resultOut: btManifoldResult; const childCollisionAlgorithms: PbtCollisionAlgorithm;
  const sharedManifold: btPersistentManifold);
begin
  m_compoundColObj := compoundObj;
  m_otherObj       := otherObj;
  m_dispatcher     := dispatcher;
  m_dispatchInfo   := dispatchInfo;
  m_resultOut      := resultOut;
  m_childCollisionAlgorithms := childCollisionAlgorithms;
  m_sharedManifold := sharedManifold;
end;

procedure btCompoundLeafCallback.ProcessChildShape(const childShape: btCollisionShape; const index: integer);
var compoundShape         : btCompoundShape;
    orgTrans,newChildWorldTrans,
    orgInterpolationTrans : btTransform;
    childTrans            : PbtTransform;
    aabbMin0,aabbMax0,
    aabbMin1,aabbMax1     : btVector3;
    //worldAabbMin,
    //worldAabbMax          : btVector3;
    tmpShape              : btCollisionShape;
begin
  btAssert(index>=0);
  compoundShape := btCompoundShape(m_compoundColObj.getCollisionShape);
  btAssert(index<compoundShape.getNumChildShapes);
  //backup
  orgTrans              := m_compoundColObj.getWorldTransformP^;
  orgInterpolationTrans := m_compoundColObj.getInterpolationWorldTransformP^;
  childTrans            := compoundShape.getChildTransformP(index);
  newChildWorldTrans    := orgTrans*childTrans^;

  //perform an AABB check first
  childShape.getAabb(newChildWorldTrans,aabbMin0,aabbMax0);
  m_otherObj.getCollisionShape.getAabb(m_otherObj.getWorldTransformP^,aabbMin1,aabbMax1);

  if TestAabbAgainstAabb2(aabbMin0,aabbMax0,aabbMin1,aabbMax1) then begin
    m_compoundColObj.setWorldTransform(newChildWorldTrans);
    m_compoundColObj.setInterpolationWorldTransform(newChildWorldTrans);

    //the contactpoint is still projected back using the original inverted worldtrans
    tmpShape := m_compoundColObj.getCollisionShape;
    m_compoundColObj.internalSetTemporaryCollisionShape(childShape);

    if not assigned(m_childCollisionAlgorithms[index]) then begin
      m_childCollisionAlgorithms[index] := m_dispatcher.findAlgorithm(m_compoundColObj,m_otherObj,m_sharedManifold);
    end;

    ///detect swapping case
    if m_resultOut.getBody0Internal = m_compoundColObj then begin
      m_resultOut.setShapeIdentifiersA(-1,index);
    end else begin
      m_resultOut.setShapeIdentifiersB(-1,index);
    end;
    m_childCollisionAlgorithms[index].processCollision(m_compoundColObj,m_otherObj,m_dispatchInfo^,m_resultOut);
    if assigned(m_dispatchInfo^.m_debugDraw) and (DBG_DrawAabb in  m_dispatchInfo^.m_debugDraw.getDebugMode) then begin
      m_dispatchInfo^.m_debugDraw.drawAabb(aabbMin0,aabbMax0,btVector3.InitSameS(1));
      m_dispatchInfo^.m_debugDraw.drawAabb(aabbMin1,aabbMax1,btVector3.InitSameS(1));
    end;
    //revert back transform
    m_compoundColObj.internalSetTemporaryCollisionShape(tmpShape);
    m_compoundColObj.setWorldTransform(orgTrans);
    m_compoundColObj.setInterpolationWorldTransform(orgInterpolationTrans);
  end;
end;

procedure btCompoundLeafCallback.Process(const leaf: PbtDbvtNode);
var index:integer;
    compoundShape:btCompoundShape;
    childShape:btCollisionShape;
    worldAabbMin,worldAabbMax : btVector3;
    orgTrans : btTransform;

begin
  index := leaf^.int.dataAsInt;
  compoundShape := btCompoundShape(m_compoundColObj.getCollisionShape);
  childShape    := compoundShape.getChildShape(index);
  if assigned(m_dispatchInfo^.m_debugDraw) and (DBG_DrawAabb in m_dispatchInfo^.m_debugDraw.getDebugMode) then begin
    orgTrans := m_compoundColObj.getWorldTransformP^;
    btTransformAabb(leaf^.volume.Mins^,leaf^.volume.Maxs^,0,orgTrans,worldAabbMin,worldAabbMax);
    m_dispatchInfo^.m_debugDraw.drawAabb(worldAabbMin,worldAabbMax,btVector3.inits(1,0,0));
  end;
  ProcessChildShape(childShape,index);
end;

procedure btCompoundCollisionAlgorithm.processCollision(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult);
var colObj,otherObj : btCollisionObject;
    compoundShape   : btCompoundShape;
    childShape      : btCollisionShape;
    tree            : btDbvt;
    callback        : btCompoundLeafCallback;
    i               : integer;
    manifoldArray   : btManifoldArray;
    m               : Integer;
    localAabbMin,
    localAabbMax         : btVector3;
    otherInCompoundSpace : btTransform;
    bounds               : btDbvtVolume;
    numChildren          : LongInt;
    orgTrans,
//    orgInterpolationTrans,
    newChildWorldTrans     : btTransform;
    childTrans             : PbtTransform;
    aabbMin0,aabbMax0,
    aabbMin1,aabbMax1      : btVector3;
begin
  colObj    := btCollisionObject(btDecide(m_isSwapped,body1,body0));
  otherObj  := btCollisionObject(btDecide(m_isSwapped,body0,body1));
  btAssert(colObj.getCollisionShape.isCompound);

  compoundShape := btCompoundShape(colObj.getCollisionShape);

  ///btCompoundShape might have changed:
  ////make sure the internal child collision algorithm caches are still valid
  if (compoundShape.getUpdateRevision <> m_compoundShapeRevision) then begin
    ///clear and update all
    removeChildAlgorithms;
    preallocateChildAlgorithms(body0,body1);
  end;


  tree := compoundShape.getDynamicAabbTree;
  //use a dynamic aabb tree to cull potential child-overlaps
  callback := btCompoundLeafCallback.Create(colObj,otherObj,m_dispatcher,@dispatchInfo,resultOut,m_childCollisionAlgorithms.A[0],m_sharedManifold);

  ///we need to refresh all contact manifolds
  ///note that we should actually recursively traverse all children, btCompoundShape can nested more then 1 level deep
  ///so we should add a 'refreshManifolds' in the btCollisionAlgorithm
  for i := 0 to m_childCollisionAlgorithms.size-1 do begin
    if assigned(m_childCollisionAlgorithms[i]) then begin
      manifoldArray := btManifoldArray.create;
      try
        m_childCollisionAlgorithms[i]^.getAllContactManifolds(manifoldArray);
        for  m :=0 to manifoldArray.size-1 do begin
          if manifoldArray[m]^.getNumContacts>0 then begin
            resultOut.setPersistentManifold(manifoldArray[m]^);
            resultOut.refreshContactPoints;
            resultOut.setPersistentManifold(Nil);//??necessary?
          end;
        end;
        manifoldArray.clear;
      finally
        manifoldArray.free;
      end;
    end;
  end;

  if assigned(tree) then begin
    otherInCompoundSpace := colObj.getWorldTransformP^.inverse * otherObj.getWorldTransformP^;
    otherObj.getCollisionShape.getAabb(otherInCompoundSpace,localAabbMin,localAabbMax);
    bounds.FromMM(localAabbMin,localAabbMax);
    //process all children, that overlap with  the given AABB bounds
    tree.collideTV(tree.m_root,bounds,callback);
  end  else begin
    //iterate over all children, perform an AABB check inside ProcessChildShape
    numChildren := m_childCollisionAlgorithms.size;
    for i:=0 to numChildren-1 do begin
      callback.ProcessChildShape(compoundShape.getChildShape(i),i);
    end;
  end;
  //iterate over all children, perform an AABB check inside ProcessChildShape
  numChildren := m_childCollisionAlgorithms.size;
  childShape  := nil;
  for i := 0 to numChildren-1 do begin
    if assigned(m_childCollisionAlgorithms[i]) then begin
      childShape := compoundShape.getChildShape(i);
      //if not longer overlapping, remove the algorithm
      orgTrans              := colObj.getWorldTransformP^;
//      orgInterpolationTrans := colObj.getInterpolationWorldTransformP^;
      childTrans            := compoundShape.getChildTransformP(i);
      newChildWorldTrans    := orgTrans*childTrans^;
      //perform an AABB check first
      childShape.getAabb(newChildWorldTrans,aabbMin0,aabbMax0);
      otherObj.getCollisionShape.getAabb(otherObj.getWorldTransformP^,aabbMin1,aabbMax1);
      if not (TestAabbAgainstAabb2(aabbMin0,aabbMax0,aabbMin1,aabbMax1)) then begin
//        m_childCollisionAlgorithms[i].fr->~btCollisionAlgorithm();
        m_dispatcher.freeCollisionAlgorithm(m_childCollisionAlgorithms[i]^);
        m_childCollisionAlgorithms[i]^ := nil;
      end;
    end;
  end;
end;

function btCompoundCollisionAlgorithm.calculateTimeOfImpact(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar;
var colObj,otherObj : btCollisionObject;
    compoundShape   : btCompoundShape;
    childShape,
    tmpShape        : btCollisionShape;
    i               : integer;
    hitFraction,frac: btScalar;
    //manifoldArray   : btManifoldArray;
    //m               : Integer;
    //localAabbMin,
    //localAabbMax         : btVector3;
    //otherInCompoundSpace : btTransform;
    //bounds               : btDbvtVolume;
    numChildren          : LongInt;
    orgTrans             : btTransform;
    childTrans           : PbtTransform;
    //aabbMin0,aabbMax0,
    //aabbMin1,aabbMax1      : btVector3;

begin
  hitFraction := 1;
  colObj      := btCollisionObject(btDecide(m_isSwapped,body1,body0));
  otherObj    := btCollisionObject(btDecide(m_isSwapped,body0,body1));
  btAssert(colObj.getCollisionShape.isCompound);

  compoundShape := btCompoundShape(colObj.getCollisionShape);

  //We will use the OptimizedBVH, AABB tree to cull potential child-overlaps
  //If both proxies are Compound, we will deal with that directly, by performing sequential/parallel tree traversals
  //given Proxy0 and Proxy1, if both have a tree, Tree0 and Tree1, this means:
  //determine overlapping nodes of Proxy1 using Proxy0 AABB against Tree1
  //then use each overlapping node AABB against Tree0
  //and vise versa.

  numChildren := m_childCollisionAlgorithms.size;
  for i := 0 to numChildren-1 do begin
    //temporarily exchange parent btCollisionShape with childShape, and recurse
    childShape := compoundShape.getChildShape(i);
    //backup
    orgTrans   := colObj.getWorldTransformP^;
    childTrans := compoundShape.getChildTransformP(i);
    //btTransform   newChildWorldTrans = orgTrans*childTrans ;
    colObj.setWorldTransform( orgTrans * childTrans^ );
    tmpShape   := colObj.getCollisionShape;
    colObj.internalSetTemporaryCollisionShape( childShape );
    frac       := m_childCollisionAlgorithms[i]^.calculateTimeOfImpact(colObj,otherObj,dispatchInfo,resultOut);
    if (frac<hitFraction) then begin
      hitFraction := frac;
    end;
    //revert back
    colObj.internalSetTemporaryCollisionShape( tmpShape);
    colObj.setWorldTransform( orgTrans);
  end;
  result := hitFraction;
end;

procedure btCompoundCollisionAlgorithm.getAllContactManifolds(const manifoldArray: btManifoldArray);
var i:integer;
begin
  for i:=0 to m_childCollisionAlgorithms.size-1 do begin
    if assigned(m_childCollisionAlgorithms[i]) then begin
      m_childCollisionAlgorithms[i]^.getAllContactManifolds(manifoldArray);
    end;
  end;
end;

{ btCompoundCollisionAlgorithmCreateFuncNormAndSwapped }

function btCompoundCollisionAlgorithmCreateFuncNormAndSwapped.CreateCollisionAlgorithm(const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;
begin
  result:=info.m_dispatcher1.allocateCollisionAlgorithm(btCompoundCollisionAlgorithm);
  btCompoundCollisionAlgorithm(result).Init(info,body0,body1,m_swapped);
end;

{ btConvexPlaneCollisionAlgorithm }

procedure btConvexPlaneCollisionAlgorithm.Init(const mf: btPersistentManifold; const ci: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject; const isSwapped: boolean;  const numPerturbationIterations, minimumPointsPerturbationThreshold: integer);
var convexObj,planeObj: btCollisionObject;
begin
  inherited Init(ci);
  m_ownManifold := false;
  m_manifoldPtr := mf;
  m_isSwapped   := isSwapped;
  m_numPerturbationIterations := numPerturbationIterations;
  m_minimumPointsPerturbationThreshold := minimumPointsPerturbationThreshold;

  convexObj := btCollisionObject(btDecide(m_isSwapped,body1,body0));
  planeObj  := btCollisionObject(btDecide(m_isSwapped,body0,body1));

  if not assigned(m_manifoldPtr) and m_dispatcher.needsCollision(convexObj,planeObj) then begin
    m_manifoldPtr := m_dispatcher.getNewManifold(convexObj,planeObj);
    m_ownManifold := true;
  end;
end;

destructor btConvexPlaneCollisionAlgorithm.Destroy;
begin
  if m_ownManifold then begin
    if assigned(m_manifoldPtr) then begin
      m_dispatcher.releaseManifold(m_manifoldPtr);
    end;
  end;
  Inherited;
end;

procedure btConvexPlaneCollisionAlgorithm.processCollision(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult);
var convexObj,planeObj: btCollisionObject;
    convexShape         : btConvexShape;
    planeShape          : btStaticPlaneShape;
    planeNormal         : PbtVector3;
    rotq,
    perturbeRot         : btQuaternion;
    v0,v1               : btVector3;
    angleLimit,radius,
    iterationAngle,
    perturbeAngle          : btScalar;
    i: Integer;
begin
  if not assigned(m_manifoldPtr) then begin
    exit;
  end;

  convexObj   := btCollisionObject(btDecide(m_isSwapped,body1,body0));
  planeObj    := btCollisionObject(btDecide(m_isSwapped,body0,body1));
  convexShape := btConvexShape(convexObj.getCollisionShape);
  planeShape  := btStaticPlaneShape(planeObj.getCollisionShape);
  planeNormal := planeShape.getPlaneNormalP;
  //first perform a collision query with the non-perturbated collision objects
  rotq.init4(0,0,0,1);
  collideSingleContact(rotq,body0,body1,dispatchInfo,resultOut);

  if (resultOut.getPersistentManifold.getNumContacts<m_minimumPointsPerturbationThreshold) then begin
    btPlaneSpace1(planeNormal^,v0,v1);
    //now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects
    angleLimit    := 0.125 * SIMD_PI;
    radius        := convexShape.getAngularMotionDisc;
    perturbeAngle := gContactBreakingThreshold / radius;
    if perturbeAngle > angleLimit then begin
      perturbeAngle := angleLimit;
    end;

    perturbeRot.InitQ(v0,perturbeAngle);
    for i :=0 to m_numPerturbationIterations-1 do begin
      iterationAngle := i*(SIMD_2_PI/btScalar(m_numPerturbationIterations));
      rotq.initq(planeNormal^,iterationAngle);
      collideSingleContact(rotq.inverse*perturbeRot*rotq,body0,body1,dispatchInfo,resultOut);
    end;
  end;
  if m_ownManifold then begin
    if m_manifoldPtr.getNumContacts>0 then begin
      resultOut.refreshContactPoints;
    end;
  end;
end;

{$HINTS OFF}
procedure btConvexPlaneCollisionAlgorithm.collideSingleContact(const perturbeRot: btQuaternion; const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo;  var resultOut: btManifoldResult);
var convexObj,planeObj  : btCollisionObject;
    convexShape         : btConvexShape;
    planeShape          : btStaticPlaneShape;
    hasCollision        : boolean;
    planeNormal         : PbtVector3;
    planeConstant       : PbtScalar;
    convexWorldTransform,
    convexInPlaneTrans,
    planeInConvex       : btTransform;
    vtx,vtxInPlane,
    vtxInPlaneProjected,
    vtxInPlaneWorld,
    normalOnSurfaceB,
    pOnB                : btVector3;
    distance            : btScalar;
begin

    convexObj   := btCollisionObject(btDecide(m_isSwapped,body1,body0));
    planeObj    := btCollisionObject(btDecide(m_isSwapped,body0,body1));
    convexShape := btConvexShape(convexObj.getCollisionShape);
    planeShape  := btStaticPlaneShape(planeObj.getCollisionShape);

    hasCollision := false;
    planeNormal   := planeShape.getPlaneNormalP;
    planeConstant := planeShape.getPlaneConstantP;

    convexWorldTransform := convexObj.getWorldTransformP^;
    convexInPlaneTrans   := planeObj.getWorldTransformP^.inverse * convexWorldTransform;
    //now perturbe the convex-world transform
    convexWorldTransform.GetBasisV^*= btMatrix3x3.InitS(perturbeRot);
    planeInConvex        := convexWorldTransform.inverse * planeObj.getWorldTransformP^;

    vtx                  := convexShape.localGetSupportingVertex(planeInConvex.GetBasisV^*-planeNormal^);
    vtxInPlane           := convexInPlaneTrans.opTrans(vtx);
    distance             := planeNormal^.dot(vtxInPlane) - planeConstant^;

    vtxInPlaneProjected  := vtxInPlane - distance*planeNormal^;
    vtxInPlaneWorld      := planeObj.getWorldTransformP^ * vtxInPlaneProjected;

    hasCollision         := distance < m_manifoldPtr.getContactBreakingThreshold;
    resultOut.setPersistentManifold(m_manifoldPtr);
    if hasCollision then begin
      /// report a contact. internally this will be kept persistent, and contact reduction is done
      normalOnSurfaceB := planeObj.getWorldTransformP^.GetBasisV^ * planeNormal^;
      pOnB             := vtxInPlaneWorld;
      resultOut.addContactPoint(normalOnSurfaceB,pOnB,distance);
    end;
end;
{$HINTS ON}

{$HINTS OFF}
function btConvexPlaneCollisionAlgorithm.calculateTimeOfImpact(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar;
begin
  //(void)resultOut;
  //(void)dispatchInfo;
  //(void)col0;
  //(void)col1;
  //not yet
  Result := 1;
end;
{$HINTS ON}

procedure btConvexPlaneCollisionAlgorithm.getAllContactManifolds(const manifoldArray: btManifoldArray);
begin
  if assigned(m_manifoldPtr) and m_ownManifold then begin
    manifoldArray.push_back(m_manifoldPtr);
  end;
end;

{ btConvexPlaneCollisionAlgorithmCreateFuncNormAndSwapped }

constructor btConvexPlaneCollisionAlgorithmCreateFuncNormAndSwapped.Create(const swapped: boolean);
begin
 inherited Create(swapped);
 m_minimumPointsPerturbationThreshold:=1;
 m_numPerturbationIterations:=1;
end;

function btConvexPlaneCollisionAlgorithmCreateFuncNormAndSwapped.CreateCollisionAlgorithm(const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;
begin
  result:=info.m_dispatcher1.allocateCollisionAlgorithm(btConvexPlaneCollisionAlgorithm);
  btConvexPlaneCollisionAlgorithm(result).Init(nil,info,body0,body1,m_swapped,m_numPerturbationIterations,m_minimumPointsPerturbationThreshold);
end;

{ btCollisionAlgorithm }

procedure btCollisionAlgorithm.Init(const ci: btCollisionAlgorithmConstructionInfo);
begin
  m_dispatcher := ci.m_dispatcher1;
end;

{ btSphereSphereCollisionAlgorithm }

procedure btSphereSphereCollisionAlgorithm.Init(const mf: btPersistentManifold; const ci: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject);
begin
  inherited init(ci,body0,body1);
  m_ownManifold := false;
  m_manifoldPtr := mf;
  if not assigned(m_manifoldPtr) then begin
    m_manifoldPtr := m_dispatcher.getNewManifold(body0,body1);
    m_ownManifold := true;
  end;
end;

destructor btSphereSphereCollisionAlgorithm.Destroy;
begin
  if m_ownManifold then begin
    if assigned(m_manifoldPtr) then begin
      m_dispatcher.releaseManifold(m_manifoldPtr);
    end;
  end;
  inherited Destroy;
end;

{$HINTS OFF}
procedure btSphereSphereCollisionAlgorithm.processCollision(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult);
var sphere0,sphere1  : btSphereShape;
    diff,pos1,
    normalOnSurfaceB : btVector3;
    len,radius0,
    radius1,dist     : btScalar;
begin
    if not assigned(m_manifoldPtr) then begin
      exit;
    end;
    resultOut.setPersistentManifold(m_manifoldPtr);
    sphere0 := btSphereShape(body0.getCollisionShape);
    sphere1 := btSphereShape(body1.getCollisionShape);
    diff    := body0.getWorldTransformP^.getOriginV^ - body1.getWorldTransformP^.getOriginV^;
    len     := diff.length;
    radius0 := sphere0.getRadius;
    radius1 := sphere1.getRadius;

  {$ifdef CLEAR_MANIFOLD}
    m_manifoldPtr.clearManifold; //don't do this, it disables warmstarting
  {$endif}

    ///iff distance positive, don't generate a new contact
    if len > (radius0+radius1) then begin
  {$ifdef CLEAR_MANIFOLD}
      resultOut.refreshContactPoints;
  {$endif}
      exit;
    end;
    ///distance (negative means penetration)
    dist := len - (radius0+radius1);
    normalOnSurfaceB.init(1,0,0);
    if (len > SIMD_EPSILON) then begin
      normalOnSurfaceB := diff / len;
    end;
    ///point on A (worldspace)
    ///btVector3 pos0 = body0->getWorldTransform.getOrigin() - radius0 * normalOnSurfaceB;
    ///point on B (worldspace)
    pos1 := body1.getWorldTransformP^.getOriginV^ + radius1* normalOnSurfaceB;
    /// report a contact. internally this will be kept persistent, and contact reduction is done
    resultOut.addContactPoint(normalOnSurfaceB,pos1,dist);

  {$ifndef CLEAR_MANIFOLD}
    resultOut.refreshContactPoints;
  {$endif} //CLEAR_MANIFOLD
end;
{$HINTS ON}

{$HINTS OFF}
function btSphereSphereCollisionAlgorithm.calculateTimeOfImpact(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar;
begin
  //(void)col0;
  //(void)col1;
  //(void)dispatchInfo;
  //(void)resultOut;
  //not yet
  Result := 1;
end;
{$HINTS ON}

procedure btSphereSphereCollisionAlgorithm.getAllContactManifolds(const manifoldArray: btManifoldArray);
begin
  if assigned(m_manifoldPtr) and m_ownManifold then begin
    manifoldArray.push_back(m_manifoldPtr);
  end;
end;

{ btSphereSphereCollisionAlgorithmCreateFunc }

function btSphereSphereCollisionAlgorithmCreateFunc.CreateCollisionAlgorithm(const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;
begin
  result:=info.m_dispatcher1.allocateCollisionAlgorithm(btSphereSphereCollisionAlgorithm);
  btSphereSphereCollisionAlgorithm(result).Init(nil,info,body0,body1);
end;

{ btSphereTriangleCollisionAlgorithm }

procedure btSphereTriangleCollisionAlgorithm.Init(const mf: btPersistentManifold; const ci: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject; const swapped: boolean);
begin
  inherited Init(ci,body0,body1);
  m_ownManifold := false;
  m_manifoldPtr := mf;
  m_swapped     := swapped;
  if not assigned(m_manifoldPtr) then begin
    m_manifoldPtr := m_dispatcher.getNewManifold(body0,body1);
    m_ownManifold := true;
  end;
end;

destructor btSphereTriangleCollisionAlgorithm.Destroy;
begin
  if m_ownManifold then begin
    if assigned(m_manifoldPtr) then begin
      m_dispatcher.releaseManifold(m_manifoldPtr);
    end;
  end;
  inherited;
end;

procedure btSphereTriangleCollisionAlgorithm.processCollision(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult);
var sphereObj,triObj : btCollisionObject;
    sphere           : btSphereShape;
    triangle         : btTriangleShape;
    detector         : btSphereTriangleDetector;
    input            : btDCDI_ClosestPointInput;

begin
  if not assigned(m_manifoldPtr) then exit;

  sphereObj := btCollisionObject(btDecide(m_Swapped,body1,body0));
  triObj    := btCollisionObject(btDecide(m_Swapped,body0,body1));

  sphere    := btSphereShape(sphereObj.getCollisionShape);
  triangle  := btTriangleShape(triObj.getCollisionShape);

  /// report a contact. internally this will be kept persistent, and contact reduction is done
  resultOut.setPersistentManifold(m_manifoldPtr);
  detector.init(sphere,triangle, m_manifoldPtr.getContactBreakingThreshold);
  input.m_maximumDistanceSquared := btScalar(BT_LARGE_FLOAT);///@todo: tighter bounds
  input.m_transformA             := sphereObj.getWorldTransformP^;
  input.m_transformB             := triObj.getWorldTransformP^;
  detector.getClosestPoints(input,btDCDI_Result(resultOut),dispatchInfo.m_debugDraw,m_swapped);
  if m_ownManifold then begin
    resultOut.refreshContactPoints;
  end;
end;

{$HINTS OFF}
function btSphereTriangleCollisionAlgorithm.calculateTimeOfImpact(const body0, body1: btCollisionObject; const dispatchInfo: btDispatcherInfo; var resultOut: btManifoldResult): btScalar;
begin
  //(void)resultOut;
  //(void)dispatchInfo;
  //(void)col0;
  //(void)col1;
  //
  //not yet
  Result := 1;
end;
{$HINTS ON}


procedure btSphereTriangleCollisionAlgorithm.getAllContactManifolds(const manifoldArray: btManifoldArray);
begin
  if assigned(m_manifoldPtr) and m_ownManifold then begin
    manifoldArray.push_back(m_manifoldPtr);
  end;
end;

{ btSphereTriangleCollisionAlgorithmCreateFunc }

function btSphereTriangleCollisionAlgorithmCreateFunc.CreateCollisionAlgorithm(const info: btCollisionAlgorithmConstructionInfo; const body0, body1: btCollisionObject): btCollisionAlgorithm;
begin
  result:=info.m_dispatcher1.allocateCollisionAlgorithm(btSphereTriangleCollisionAlgorithm);
  btSphereTriangleCollisionAlgorithm(result).Init(nil,info,body0,body1,m_swapped);
end;

{ btSphereTriangleDetector }

function btSphereTriangleDetector.pointInTriangle(const vertices: PbtVector3; const normal:btVector3;const p: PbtVector3): boolean;
var p1,p2,p3                                  : PbtVector3;
    edge1,edge2,edge3,p1_to_p,p2_to_p,p3_to_p,
    edge1_normal,edge2_normal,edge3_normal    : btVector3 ;
    r1, r2, r3                                : btScalar;
begin
  p1           := @vertices[0];
  p2           := @vertices[1];
  p3           := @vertices[2];
  edge1        := (p2^ - p1^);
  edge2        := (p3^ - p2^);
  edge3        := (p1^ - p3^);
  p1_to_p      := (p^ - p1^);
  p2_to_p      := (p^ - p2^);
  p3_to_p      := (p^ - p3^);
  edge1_normal :=  edge1.cross(normal);
  edge2_normal :=  edge2.cross(normal);
  edge3_normal :=  edge3.cross(normal);
  r1           := edge1_normal.dot( p1_to_p );
  r2           := edge2_normal.dot( p2_to_p );
  r3           := edge3_normal.dot( p3_to_p );
  if   ( (r1>0) and (r2>0)  and (r3>0 ) ) or ( (r1<0) and  (r2<=0) and (r3<=0) ) then begin
   result := true;
  end;
  Result := false;
end;

function btSphereTriangleDetector.facecontains(const p: btVector3; const vertices:PbtVector3;const normal: btVector3): boolean;
var lnormal:btVector3;
begin
  lnormal := normal;
  Result := pointInTriangle(vertices, lnormal,@p);
end;

{$HINTS OFF}
procedure btSphereTriangleDetector.getClosestPoints(const input: btDCDI_ClosestPointInput; var output: btDCDI_Result; const debugDraw: btIDebugDraw; const swapResults: boolean);
var transformA,transformB,sphereInTr:btTransform;
    point,normal,normalOnB,normalOnA,pointOnA : btVector3;
    timeOfImpact,depth : btScalar;
begin
  transformA   := input.m_transformA;
  transformB   := input.m_transformB;
  timeOfImpact := 1;
  depth        := 0;
  //    output.m_distance = btScalar(BT_LARGE_FLOAT);
  //move sphere into triangle space
  sphereInTr := transformB.inverseTimes(transformA);

  if collide(sphereInTr.getOriginV^,point,normal,depth,timeOfImpact,m_contactBreakingThreshold) then begin
    if swapResults then begin
      normalOnB := transformB.GetBasisV^*normal;
      normalOnA := -normalOnB;
      pointOnA  := transformB*point+normalOnB*depth;
      output.addContactPoint(normalOnA,pointOnA,depth);
    end else begin
      output.addContactPoint(transformB.GetBasisV^*normal,transformB*point,depth);
    end;
  end;
end;
{$HINTS ON}


// See also geometrictools.com
// Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv
//btScalar SegmentSqrDistance(const btVector3& from, const btVector3& to,const btVector3 &p, btVector3 &nearest);

function SegmentSqrDistance(const from, tov,p:btVector3;out nearest :btVector3) : btScalar;
var diff,v  : btVector3;
    t,dotVV : btScalar;
begin
  diff := p - from;
  v    := tov - from;
  t    := v.dot(diff);
  if (t > 0)  then begin
    dotVV := v.dot(v);
    if (t < dotVV) then begin
      t    /= dotVV;
      diff -= t*v;
    end else begin
      t    := 1;
      diff -= v;
    end;
  end else begin
    t := 0;
  end;
  nearest := from + t*v;
  result := diff.dot(diff);
end;
const cMAX_OVERLAP = 0;
///combined discrete/continuous sphere-triangle
function btSphereTriangleDetector.collide(const sphereCenter : btVector3; out point,resultNormal:btVector3 ; var depth, timeOfImpact: btScalar; const contactBreakingThreshold: btScalar): boolean;
var  vertices              : PbtVector3;
     r,contactMargin,
     deltaDotNormal,
     contactCapsuleRadiusSqr,
     distanceSqr,distance,
     distanceFromPlane     : btScalar;
     delta,c,contactPoint,
     p1ToCentre,pa,pb,
     contactToCentre,
     normal,nearestOnEdge  : btVector3;
     isInsideShellPlane,
     hasContact,
     isInsideContactPlane  : Boolean;
     i: Integer;
begin
  vertices := m_triangle.getVertexPtr(0);
  c        := sphereCenter;
  r        := m_sphere.getRadius;
  delta.InitSame(0);
  normal   := (vertices[1]-vertices[0]).cross(vertices[2]-vertices[0]);
  normal.normalize;
  p1ToCentre        := c - vertices[0];
  distanceFromPlane := p1ToCentre.dot(normal);

  if distanceFromPlane < 0 then begin
    //triangle facing the other way
    distanceFromPlane *= -1;
    normal            *= -1;
  end;
  contactMargin        := contactBreakingThreshold;
  isInsideContactPlane := distanceFromPlane < (r+contactMargin);
  isInsideShellPlane   := distanceFromPlane < r;
  deltaDotNormal       := delta.dot(normal);
  if (not isInsideShellPlane) and (deltaDotNormal >= 0) then begin
    exit(false);
  end;
  // Check for contact / intersection
  hasContact := false;
  if isInsideContactPlane then begin
    if facecontains(c,vertices,normal) then begin // Inside the contact wedge - touches a point on the shell plane
      hasContact   := true;
      contactPoint := c - normal*distanceFromPlane;
    end else begin
      // Could be inside one of the contact capsules
      contactCapsuleRadiusSqr := (r + contactMargin) * (r + contactMargin);
      for i := 0 to m_triangle.getNumEdges-1 do  begin
        m_triangle.getEdge(i,pa,pb);
        distanceSqr     := SegmentSqrDistance(pa,pb,c, nearestOnEdge);
        if distanceSqr < contactCapsuleRadiusSqr then begin
          // Yep, we're inside a capsule
          hasContact   := true;
          contactPoint := nearestOnEdge;
        end;
      end;
    end;
  end;
  if hasContact then begin
    contactToCentre := c - contactPoint;
    distanceSqr     := contactToCentre.length2;
    if (distanceSqr < (r - cMAX_OVERLAP)*(r - cMAX_OVERLAP)) then begin
      distance     := btSqrt(distanceSqr);
      resultNormal := contactToCentre;
      resultNormal.normalize;
      point        := contactPoint;
      depth        := -(r-distance);
      exit(true);
    end;
    if delta.dot(contactToCentre) >= 0 then begin
      result:=false;
      exit;
    end;
    // Moving towards the contact point -> collision
    point        := contactPoint;
    timeOfImpact := 0;
    Result := true;
    exit;
  end;
  Result := false;
end;

procedure btSphereTriangleDetector.Init(const sphere: btSphereShape; const triangle: btTriangleShape; const contactBreakingThreshold: btScalar);
begin
  m_sphere   := sphere;
  m_triangle := triangle;
  m_contactBreakingThreshold := contactBreakingThreshold;
end;




{ btDbvtBroadphase }

//  btCompoundLeafCallback=class(btDbvt_ICollide)

type
  // Tree collider

  { btDbvtTreeCollider }

  //FOSTODO -> Kick this class and do it stack local
  btDbvtTreeCollider = class(btDbvt_ICollide)
   pbp               : btDbvtBroadphase;
   proxy             : btDbvtProxy;
   constructor create  (const p : btDbvtBroadphase);
   procedure   Process (const na, nb: PbtDbvtNode); override;
   procedure   Process (const n: PbtDbvtNode); override;
  end;

  { btBroadphaseRayTester }

  btBroadphaseRayTester = class(btDbvt_ICollide)
    m_rayCallback : btBroadphaseRayCallback;
    constructor create(const orgCallback:btBroadphaseRayCallback);
    procedure   Process(const leaf: PbtDbvtNode); override;
  end;

  { btBroadphaseAabbTester }

  btBroadphaseAabbTester =class(btDbvt_ICollide)
    m_aabbCallback : btBroadphaseAabbCallback;
    constructor create(const orgCallback:btBroadphaseAabbCallback);
    procedure   Process(const leaf: PbtDbvtNode); override;
  end;

{ btBroadphaseAabbTester }

constructor btBroadphaseAabbTester.create(const orgCallback: btBroadphaseAabbCallback);
begin
  m_aabbCallback := orgCallback;
end;

procedure btBroadphaseAabbTester.Process(const leaf: PbtDbvtNode);
begin
  m_aabbCallback.process(btDbvtProxy(leaf^.int.data));
end;

{ btBroadphaseRayTester }

constructor btBroadphaseRayTester.create(const orgCallback: btBroadphaseRayCallback);
begin
  m_rayCallback := orgCallback;
end;

procedure btBroadphaseRayTester.Process(const leaf: PbtDbvtNode);
begin
  m_rayCallback.process(btDbvtProxy(leaf^.int.data));
end;

{ btDbvtTreeCollider }

constructor btDbvtTreeCollider.create(const p: btDbvtBroadphase);
begin
  pbp := p;
end;

procedure btDbvtTreeCollider.Process(const na, nb: PbtDbvtNode);
var pa,pb: btDbvtProxy;
begin
  if na <> nb then begin
    pa := btDbvtProxy(na^.int.data);
    pb := btDbvtProxy(nb^.int.data);
  {$ifdef DBVT_BP_SORTPAIRS}
    if pa.m_uniqueId > pb.m_uniqueId then begin
      btSwap(pa,pb);
    end;
  {$endif}
   pbp.m_paircache.addOverlappingPair(pa,pb);
//  writeln('Add overlapping pair : ',pa.dump,' ',pb.dump);
   inc(pbp.m_newpairs);
  end;
end;

procedure btDbvtTreeCollider.Process(const n: PbtDbvtNode);
begin
  Process(n,proxy.leaf);
end;

constructor btDbvtBroadphase.Create(const paircache: btOverlappingPairCache);
var
  i: Integer;
begin
  Inherited Create;
  m_sets[0] := btDbvt.create;
  m_sets[1] := btDbvt.create;
  m_deferedcollide   := false;
  m_needcleanup      := true;
  m_releasepaircache := not assigned(paircache);
  m_prediction       := 0;
  m_stageCurrent     := 0;
  m_fixedleft        := 0;
  m_fupdates         := 1;
  m_dupdates         := 0;
  m_cupdates         := 10;
  m_newpairs         := 1;
  m_updates_call     := 0;
  m_updates_done     := 0;
  m_updates_ratio    := 0;
  if assigned(paircache) then begin
   m_paircache       := paircache;
  end else begin
   m_paircache      := btHashedOverlappingPairCache.Create;
  end;
  m_gid              := 0;
  m_pid              := 0;
  m_cid              := 0;

  for i:=0 to ord(STAGECOUNT) do begin
    m_stageRoots[i] := nil;
  end;
  {$ifdef DBVT_BP_PROFILE}
    //FOSTODO clear(m_profiling);
  {$endif}
end;

destructor btDbvtBroadphase.Destroy;
begin
  if m_releasepaircache then begin
    m_paircache.Free;
  end;
  m_sets[0].free;
  m_sets[1].free;
  m_sets[0]:=nil;
  m_sets[1]:=nil;
  inherited;
end;

//{
//    printf("---------------------------------------------------------\n");
//    printf("m_sets[0].m_leaves=%d\n",m_sets[0].m_leaves);
//    printf("m_sets[1].m_leaves=%d\n",m_sets[1].m_leaves);
//    printf("numPairs = %d\n",getOverlappingPairCache()->getNumOverlappingPairs());
//    {
//          int i;
//          for (i=0;i<getOverlappingPairCache()->getNumOverlappingPairs();i++)
//          {
//                  printf("pair[%d]=(%d,%d),",i,getOverlappingPairCache()->getOverlappingPairArray()[i].m_pProxy0->getUid(),
//                          getOverlappingPairCache()->getOverlappingPairArray()[i].m_pProxy1->getUid());
//          }
//          printf("\n");
//    }
//}

procedure btDbvtBroadphase.collide(const dispatcher: btDispatcher);
var count,ni,i : integer;
    current,
    next       : btDbvtProxy;
    collider   : btDbvtTreeCollider;
    curAabb    : btDbvtVolume;
    pairs      : btBroadphasePairArray;
    p          : btBroadphasePair;
    pa,pb      : btDbvtProxy;
begin
  //SPC(m_profiling.m_total);
  // optimize
  m_sets[0].optimizeIncremental(1+(m_sets[0].m_leaves*m_dupdates) div 100);
  if m_fixedleft<>0 then begin
    count := 1+(m_sets[1].m_leaves*m_fupdates) div 100;
    m_sets[1].optimizeIncremental(1+(m_sets[1].m_leaves*m_fupdates) div 100);
    m_fixedleft := btMax(0,m_fixedleft-count);
  end;
  // dynamic -> fixed set
  m_stageCurrent := (m_stageCurrent+1) mod (ord(STAGECOUNT));
  current        := m_stageRoots[m_stageCurrent];
  if assigned(current) then begin
    abort;
//    collider := btDbvtTreeCollider.create(self);
    repeat
      next := current.links[1];
      btDbvtProxyList.listremove(current,m_stageRoots[current.stage]);
      btDbvtProxyList.listappend(current,m_stageRoots[ord(STAGECOUNT)]);
      {$ifdef DBVT_BP_ACCURATESLEEPING}
        m_paircache.removeOverlappingPairsContainingProxy(current,dispatcher);
        collider.proxy := current;
        btDbvt.collideTV(m_sets[0].m_root,current->aabb,collider);
        btDbvt.collideTV(m_sets[1].m_root,current->aabb,collider);
      {$endif}
      m_sets[0].remove(current.leaf);
      curAabb.FromMM(current.m_aabbMin,current.m_aabbMax);
      current.leaf  := m_sets[1].insert(curAabb,current);
      current.stage := ord(STAGECOUNT);
      current       := next;
    until current=nil;
//    collider.free;
    m_fixedleft   := m_sets[1].m_leaves;
    m_needcleanup := true;
  end;
  // collide dynamics
  collider := btDbvtTreeCollider.create(self);
  if m_deferedcollide then begin
   // SPC(m_profiling.m_fdcollide);
    m_sets[0].collideTTpersistentStack(m_sets[0].m_root,m_sets[1].m_root,collider);
  end;
  if m_deferedcollide then begin
    //SPC(m_profiling.m_ddcollide);
    m_sets[0].collideTTpersistentStack(m_sets[0].m_root,m_sets[0].m_root,collider);
  end;
  collider.free;
  // clean up
  if m_needcleanup then begin
    //SPC(m_profiling.m_cleanup);
    pairs := m_paircache.getOverlappingPairArray;
    if pairs.size>0 then begin
      ni := btMin(pairs.size,btMax(m_newpairs,(pairs.size*m_cupdates) div 100));
      i:=0;
      while (i<ni) do begin
        p  := pairs[(m_cid+i) mod pairs.size]^;
        pa := btDbvtProxy(p.m_pProxy0);
        pb := btDbvtProxy(p.m_pProxy1);
        if not Intersect(pa.leaf^.volume,pb.leaf^.volume) then begin
        {$ifdef DBVT_BP_SORTPAIRS}
          if(pa.m_uniqueId > pb.m_uniqueId) then btSwap(pa,pb);
        {$endif}
          m_paircache.removeOverlappingPair(pa,pb,dispatcher);
          dec(ni);
          dec(i);
        end;
        inc(i);
      end;
      if pairs.size>0 then begin
        m_cid := (m_cid+ni) mod pairs.size;
      end else begin
        m_cid := 0;
      end;
    end;
  end;
  inc(m_pid);
  m_newpairs    := 1;
  m_needcleanup := false;
  if m_updates_call>0 then begin
    m_updates_ratio := m_updates_done / m_updates_call;
  end else begin
    m_updates_ratio := 0;
  end;
  m_updates_done := m_updates_done div 2;
  m_updates_call := m_updates_call div 2;
end;

procedure btDbvtBroadphase.optimize;
begin
  abort;
  m_sets[0].optimizeTopDown;
  m_sets[1].optimizeTopDown;
end;

{$HINTS OFF}
function btDbvtBroadphase.createProxy(const aabbMin, aabbMax: btVector3; const shapeType: TbtBroadphaseNativeTypes; const userPtr: Pointer; const collisionFilterGroup, collisionFilterMask: btCollisionFilterGroupSet;const dispatcher: btDispatcher; const multiSapProxy: Pointer): btBroadphaseProxy;
var aabb    : btDbvtAabbMm;
   proxy    : btDbvtProxy;
   collider : btDbvtTreeCollider;
begin
  proxy := btDbvtProxy.create;
  proxy.init(aabbMin,aabbMax,userPtr,collisionFilterGroup,collisionFilterMask);
  aabb.FromMM(aabbMin,aabbMax);
  //bproxy->aabb                        =       btDbvtVolume::FromMM(aabbMin,aabbMax);
  proxy.stage      := m_stageCurrent;
  proxy.m_uniqueId := btPlusPlusVar(m_gid);
  proxy.leaf       := m_sets[0].insert(aabb,proxy);
  btDbvtProxyList.listappend(proxy,m_stageRoots[m_stageCurrent]);
  if not m_deferedcollide then begin
    collider       := btDbvtTreeCollider.create(self);
    collider.proxy := proxy;
    m_sets[0].collideTV(m_sets[0].m_root,aabb,collider);
    m_sets[1].collideTV(m_sets[1].m_root,aabb,collider);
    collider.Free;
  end;
  Result := proxy;
end;
{$HINTS ON}

procedure btDbvtBroadphase.destroyProxy(const absproxy: btBroadphaseProxy; const dispatcher: btDispatcher);
var proxy : btDbvtProxy;
begin
  proxy := btDbvtProxy(absproxy);
  if proxy.stage=ord(STAGECOUNT) then begin
    m_sets[1].remove(proxy.leaf);
  end else begin
    m_sets[0].remove(proxy.leaf);
  end;
  btDbvtProxyList.listremove(proxy,m_stageRoots[proxy.stage]);
  m_paircache.removeOverlappingPairsContainingProxy(proxy,dispatcher);
  proxy.free;
  proxy:=nil;
  m_needcleanup := true;
end;

{$HINTS OFF}
procedure btDbvtBroadphase.setAabb(const absproxy: btBroadphaseProxy; const aabbMin, aabbMax: btVector3; const dispatcher: btDispatcher);
var proxy : btDbvtProxy;
    aabb  : btDbvtVolume;
    docollide : boolean;
    delta,
    velocity  : btVector3;
    collider : btDbvtTreeCollider;
begin
  proxy := btDbvtProxy(absproxy);
  aabb.FromMM(aabbMin,aabbMax);
//{$ifdef DBVT_BP_PREVENTFALSEUPDATE}
//  if NotEqual(aabb,proxy->leaf->volume) then
//{$endif}
  //begin
   docollide  := false;
    if proxy.stage = ord(STAGECOUNT) then begin
     //* fixed -> dynamic set        */
      m_sets[1].remove(proxy.leaf);
      proxy.leaf := m_sets[0].insert(aabb,proxy);
      docollide:=true;
    end else begin
     //* dynamic set                         */
      inc(m_updates_call);
      if Intersect(proxy.leaf^.volume,aabb) then begin
      //* Moving*/
        //writeln('MOV ');
        delta := aabbMin-proxy.m_aabbMin;
        velocity := ((proxy.m_aabbMax-proxy.m_aabbMin)/2)*m_prediction;
        if delta[0]<0 then velocity[0] := -velocity[0];
        if delta[1]<0 then velocity[1] := -velocity[1];
        if delta[2]<0 then velocity[2] := -velocity[2];
        if
        {$ifdef DBVT_BP_MARGIN}
           m_sets[0].update(proxy.leaf,aabb,velocity,DBVT_BP_MARGIN)
        {$else}
           m_sets[0].update(proxy.leaf,aabb,velocity)
        {$endif}
        then begin
          inc(m_updates_done);
          docollide := true;
        end;
      end else begin
      //* Teleporting                 */
        m_sets[0].update(proxy.leaf,aabb);
        inc(m_updates_done);
        docollide:=true;
      end;
    end;

    btDbvtProxyList.listremove(proxy,m_stageRoots[proxy.stage]);
    proxy.m_aabbMin := aabbMin;
    proxy.m_aabbMax := aabbMax;
    proxy.stage     := m_stageCurrent;
    btDbvtProxyList.listappend(proxy,m_stageRoots[m_stageCurrent]);

   fosdebug_clock.init;
    if docollide then begin
      m_needcleanup := true;
      if not m_deferedcollide then begin
        collider       := btDbvtTreeCollider.create(self);
        m_sets[1].collideTTpersistentStack(m_sets[1].m_root,proxy.leaf,collider);
        m_sets[0].collideTTpersistentStack(m_sets[0].m_root,proxy.leaf,collider);
        collider.free;
      end;
    end;
    //writeln('CL2 ',fosdebug_clock.getTimeMicroseconds);
  //end;
end;
{$HINTS ON}

{$HINTS OFF}
procedure btDbvtBroadphase.rayTest(const rayFrom, rayTo: btVector3; const rayCallback: btBroadphaseRayCallback; const aabbMin, aabbMax: btVector3);
var callback:btBroadphaseRayTester;
begin
  callback := btBroadphaseRayTester.create(rayCallback);
  m_sets[0].rayTestInternal(m_sets[0].m_root,rayFrom,rayCallback.m_rayDirectionInverse,rayCallback.m_signs,rayCallback.m_lambda_max,aabbMin,aabbMax,callback);
  m_sets[1].rayTestInternal(m_sets[1].m_root,rayFrom,rayCallback.m_rayDirectionInverse,rayCallback.m_signs,rayCallback.m_lambda_max,aabbMin,aabbMax,callback);
  callback.free;
end;
{$HINTS ON}

procedure btDbvtBroadphase.aabbTest(const aabbMin, aabbMax: btVector3; const aabbcallback: btBroadphaseAabbCallback);
var callback : btBroadphaseAabbTester;
    bounds   : btDbvtVolume;
begin
  callback := btBroadphaseAabbTester.Create(aabbCallback);
  bounds.FromMM(aabbMin,aabbMax);
  //process all children, that overlap with  the given AABB bounds
  m_sets[0].collideTV(m_sets[0].m_root,bounds,callback);
  m_sets[1].collideTV(m_sets[1].m_root,bounds,callback);
  callback.free;
end;

procedure btDbvtBroadphase.getAabb(const proxy: btBroadphaseProxy; var aabbMin, aabbMax: btVector3);
begin
  aabbMin := btDbvtProxy(proxy).m_aabbMin;
  aabbMax := btDbvtProxy(proxy).m_aabbMax;
end;

procedure btDbvtBroadphase.calculateOverlappingPairs(const dispatcher: btDispatcher);
begin
  collide(dispatcher);
{$ifdef DBVT_BP_PROFILE}
  if(0==(m_pid%DBVT_BP_PROFILING_RATE))
  {
  	printf("fixed(%u) dynamics(%u) pairs(%u)\r\n",m_sets[1].m_leaves,m_sets[0].m_leaves,m_paircache->getNumOverlappingPairs());
  	unsigned int	total=m_profiling.m_total;
  	if(total<=0) total=1;
  	printf("ddcollide: %u%% (%uus)\r\n",(50+m_profiling.m_ddcollide*100)/total,m_profiling.m_ddcollide/DBVT_BP_PROFILING_RATE);
  	printf("fdcollide: %u%% (%uus)\r\n",(50+m_profiling.m_fdcollide*100)/total,m_profiling.m_fdcollide/DBVT_BP_PROFILING_RATE);
  	printf("cleanup:   %u%% (%uus)\r\n",(50+m_profiling.m_cleanup*100)/total,m_profiling.m_cleanup/DBVT_BP_PROFILING_RATE);
  	printf("total:     %uus\r\n",total/DBVT_BP_PROFILING_RATE);
  	const unsigned long	sum=m_profiling.m_ddcollide+
  		m_profiling.m_fdcollide+
  		m_profiling.m_cleanup;
  	printf("leaked: %u%% (%uus)\r\n",100-((50+sum*100)/total),(total-sum)/DBVT_BP_PROFILING_RATE);
  	printf("job counts: %u%%\r\n",(m_profiling.m_jobcount*100)/((m_sets[0].m_leaves+m_sets[1].m_leaves)*DBVT_BP_PROFILING_RATE));
  	clear(m_profiling);
  	m_clock.reset();
  }
{$endif}
  performDeferredRemoval(dispatcher);
end;

function btDbvtBroadphase.getOverlappingPairCache: btOverlappingPairCache;
begin
  Result := m_paircache;
end;

procedure btDbvtBroadphase.getBroadphaseAabb(out aabbMin, aabbMax: btVector3);
var bounds:btDbvtVolume;
begin
  if not m_sets[0].empty then begin
    if not m_sets[1].empty then begin
      Merge(m_sets[0].m_root^.volume, m_sets[1].m_root^.volume,bounds);
    end else begin
      bounds := m_sets[0].m_root^.volume;
    end;
  end else
  if not m_sets[1].empty then begin
    bounds := m_sets[1].m_root^.volume;
  end else begin
    bounds.FromCR(btVector3.InitSameS(0),0);
  end;
  aabbMin := bounds.Mins^;
  aabbMax := bounds.Maxs^;
end;

procedure btDbvtBroadphase.printStats;
begin
  //nothing
end;

{$HINTS OFF}
procedure btDbvtBroadphase.resetPool(const dispatcher: btDispatcher);
var i,totalObjects:integer;
begin
  totalObjects := m_sets[0].m_leaves + m_sets[1].m_leaves;
  if totalObjects=0 then begin
    //reset internal dynamic tree data structures
    m_sets[0].clear;
    m_sets[1].clear;
    m_deferedcollide := false;
    m_needcleanup    := true;
    m_stageCurrent   := 0;
    m_fixedleft      := 0;
    m_fupdates       := 1;
    m_dupdates       := 0;
    m_cupdates       := 10;
    m_newpairs       := 1;
    m_updates_call   := 0;
    m_updates_done   := 0;
    m_updates_ratio  := 0;
    m_gid            := 0;
    m_pid            := 0;
    m_cid            := 0;
    for i:=0 to ord(STAGECOUNT) do begin
      m_stageRoots[i] := nil;
    end;
  end;
end;
{$HINTS ON}

procedure btDbvtBroadphase.performDeferredRemoval(const dispatcher: btDispatcher);
var overlappingPairArray : btBroadphasePairArray;
    invalidPair,i        : integer;
    previousPair         : btBroadphasePair;
    pair                 : PbtBroadphasePair;
    isDuplicate,
    hasOverlap,
    needsRemoval         : boolean;
    pa,pb                : btDbvtProxy;
begin
  if m_paircache.hasDeferredRemoval then begin
     overlappingPairArray := m_paircache.getOverlappingPairArray;
    //perform a sort, to find duplicates and to sort 'invalid' pairs to the end
    overlappingPairArray.quickSort(@btBroadphasePairSortPredicate);
    invalidPair := 0;
    previousPair.m_pProxy0   := nil;
    previousPair.m_pProxy1   := nil;
    previousPair.m_algorithm := nil;


    for i:=0 to overlappingPairArray.size-1 do begin
      pair         := overlappingPairArray.A[i];
      isDuplicate  := (pair^ = previousPair);
      previousPair := pair^;
      needsRemoval := false;
      if not  isDuplicate then begin
        //important to perform AABB check that is consistent with the broadphase
        pa := btDbvtProxy(pair^.m_pProxy0);
        pb := btDbvtProxy(pair^.m_pProxy1);
        hasOverlap := Intersect(pa.leaf^.volume,pb.leaf^.volume);
        if hasOverlap then begin
          needsRemoval := false;
        end else begin
          needsRemoval := true;
        end;
      end  else begin
        //remove duplicate
        needsRemoval := true;
        //should have no algorithm
        btAssert(not assigned(pair^.m_algorithm));
      end;
      if needsRemoval then begin
        m_paircache.cleanOverlappingPair(pair,dispatcher);
        pair^.m_pProxy0 := nil;
        pair^.m_pProxy1 := nil;
        inc(invalidPair);
      end;
    end;
    //perform a sort, to sort 'invalid' pairs to the end
    overlappingPairArray.quickSort(@btBroadphasePairSortPredicate);
    overlappingPairArray.resize(overlappingPairArray.size-invalidPair);
  end;
end;

procedure btDbvtBroadphase.setVelocityPrediction(const prediction: btScalar);
begin
  m_prediction := prediction;
end;

function btDbvtBroadphase.getVelocityPrediction: btScalar;
begin
  Result := m_prediction;
end;

{$HINTS OFF}
procedure btDbvtBroadphase.setAabbForceUpdate(const absproxy: btBroadphaseProxy; const aabbMin, aabbMax: btVector3; const dispatcher: btDispatcher);
var proxy : btDbvtProxy;
    aabb  : btDbvtVolume;
    docollide : boolean;
    collider : btDbvtTreeCollider;
begin
  proxy     := btDbvtProxy(absproxy);
  aabb.FromMM(aabbMin,aabbMax);
  docollide := false;
  if proxy.stage = ord(STAGECOUNT) then begin
   //* fixed -> dynamic set        */
    m_sets[1].remove(proxy.leaf);
    proxy.leaf := m_sets[0].insert(aabb,proxy);
    docollide:=true;
  end else begin
    inc(m_updates_call);
    m_sets[0].update(proxy.leaf,aabb);
    inc(m_updates_done);
    docollide:=true;
  end;
  btDbvtProxyList.listremove(proxy,m_stageRoots[proxy.stage]);
  proxy.m_aabbMin := aabbMin;
  proxy.m_aabbMax := aabbMax;
  proxy.stage     := m_stageCurrent;
  btDbvtProxyList.listappend(proxy,m_stageRoots[m_stageCurrent]);
  if docollide then begin
    m_needcleanup := true;
    if not m_deferedcollide then begin
      collider       := btDbvtTreeCollider.create(self);
      m_sets[1].collideTTpersistentStack(m_sets[1].m_root,proxy.leaf,collider);
      m_sets[0].collideTTpersistentStack(m_sets[0].m_root,proxy.leaf,collider);
      collider.free;
    end;
  end;
end;
{$HINTS ON}



class procedure btDbvtBroadphase.benchmark;
begin
  //
  //{$ifdef DBVT_BP_ENABLE_BENCHMARK}
  //struct	btBroadphaseBenchmark
  //{
  //	struct	Experiment
  //	{
  //		const char*			name;
  //		int					object_count;
  //		int					update_count;
  //		int					spawn_count;
  //		int					iterations;
  //		btScalar			speed;
  //		btScalar			amplitude;
  //	};
  //	struct	Object
  //	{
  //		btVector3			center;
  //		btVector3			extents;
  //		btBroadphaseProxy*	proxy;
  //		btScalar			time;
  //		void				update(btScalar speed,btScalar amplitude,btBroadphaseInterface* pbi)
  //		{
  //			time		+=	speed;
  //			center[0]	=	btCos(time*(btScalar)2.17)*amplitude+
  //				btSin(time)*amplitude/2;
  //			center[1]	=	btCos(time*(btScalar)1.38)*amplitude+
  //				btSin(time)*amplitude;
  //			center[2]	=	btSin(time*(btScalar)0.777)*amplitude;
  //			pbi->setAabb(proxy,center-extents,center+extents,0);
  //		}
  //	};
  //	static int		UnsignedRand(int range=RAND_MAX-1)	{ return(rand()%(range+1)); }
  //	static btScalar	UnitRand()							{ return(UnsignedRand(16384)/(btScalar)16384); }
  //	static void		OutputTime(const char* name,btClock& c,unsigned count=0)
  //	{
  //		const unsigned long	us=c.getTimeMicroseconds();
  //		const unsigned long	ms=(us+500)/1000;
  //		const btScalar		sec=us/(btScalar)(1000*1000);
  //		if(count>0)
  //			printf("%s : %u us (%u ms), %.2f/s\r\n",name,us,ms,count/sec);
  //		else
  //			printf("%s : %u us (%u ms)\r\n",name,us,ms);
  //	}
  //};
  //
  //void							btDbvtBroadphase::benchmark(btBroadphaseInterface* pbi)
  //{
  //	static const btBroadphaseBenchmark::Experiment		experiments[]=
  //	{
  //		{"1024o.10%",1024,10,0,8192,(btScalar)0.005,(btScalar)100},
  //		/*{"4096o.10%",4096,10,0,8192,(btScalar)0.005,(btScalar)100},
  //		{"8192o.10%",8192,10,0,8192,(btScalar)0.005,(btScalar)100},*/
  //	};
  //	static const int										nexperiments=sizeof(experiments)/sizeof(experiments[0]);
  //	btAlignedObjectArray<btBroadphaseBenchmark::Object*>	objects;
  //	btClock													wallclock;
  //	/* Begin			*/
  //	for(int iexp=0;iexp<nexperiments;++iexp)
  //	{
  //		const btBroadphaseBenchmark::Experiment&	experiment=experiments[iexp];
  //		const int									object_count=experiment.object_count;
  //		const int									update_count=(object_count*experiment.update_count)/100;
  //		const int									spawn_count=(object_count*experiment.spawn_count)/100;
  //		const btScalar								speed=experiment.speed;
  //		const btScalar								amplitude=experiment.amplitude;
  //		printf("Experiment #%u '%s':\r\n",iexp,experiment.name);
  //		printf("\tObjects: %u\r\n",object_count);
  //		printf("\tUpdate: %u\r\n",update_count);
  //		printf("\tSpawn: %u\r\n",spawn_count);
  //		printf("\tSpeed: %f\r\n",speed);
  //		printf("\tAmplitude: %f\r\n",amplitude);
  //		srand(180673);
  //		/* Create objects	*/
  //		wallclock.reset();
  //		objects.reserve(object_count);
  //		for(int i=0;i<object_count;++i)
  //		{
  //			btBroadphaseBenchmark::Object*	po=new btBroadphaseBenchmark::Object();
  //			po->center[0]=btBroadphaseBenchmark::UnitRand()*50;
  //			po->center[1]=btBroadphaseBenchmark::UnitRand()*50;
  //			po->center[2]=btBroadphaseBenchmark::UnitRand()*50;
  //			po->extents[0]=btBroadphaseBenchmark::UnitRand()*2+2;
  //			po->extents[1]=btBroadphaseBenchmark::UnitRand()*2+2;
  //			po->extents[2]=btBroadphaseBenchmark::UnitRand()*2+2;
  //			po->time=btBroadphaseBenchmark::UnitRand()*2000;
  //			po->proxy=pbi->createProxy(po->center-po->extents,po->center+po->extents,0,po,1,1,0,0);
  //			objects.push_back(po);
  //		}
  //		btBroadphaseBenchmark::OutputTime("\tInitialization",wallclock);
  //		/* First update		*/
  //		wallclock.reset();
  //		for(int i=0;i<objects.size();++i)
  //		{
  //			objects[i]->update(speed,amplitude,pbi);
  //		}
  //		btBroadphaseBenchmark::OutputTime("\tFirst update",wallclock);
  //		/* Updates			*/
  //		wallclock.reset();
  //		for(int i=0;i<experiment.iterations;++i)
  //		{
  //			for(int j=0;j<update_count;++j)
  //			{
  //				objects[j]->update(speed,amplitude,pbi);
  //			}
  //			pbi->calculateOverlappingPairs(0);
  //		}
  //		btBroadphaseBenchmark::OutputTime("\tUpdate",wallclock,experiment.iterations);
  //		/* Clean up			*/
  //		wallclock.reset();
  //		for(int i=0;i<objects.size();++i)
  //		{
  //			pbi->destroyProxy(objects[i]->proxy,0);
  //			delete objects[i];
  //		}
  //		objects.resize(0);
  //		btBroadphaseBenchmark::OutputTime("\tRelease",wallclock);
  //	}
  //
  //}
  //{$else}
  ////..
  //{$endif}
end;

{ btCollisionWorld }

constructor btCollisionWorld.create(const dispatcher: btDispatcher; const broadphasePairCache: btBroadphaseInterface; const collisionConfiguration: btCollisionConfiguration);
begin
  m_dispatchInfo.Init;
  m_dispatcher1         := dispatcher;
  m_broadphasePairCache := broadphasePairCache;
  m_debugDrawer         := nil;
  m_forceUpdateAllAabbs := true;
  m_stackAlloc          := collisionConfiguration.getStackAllocator;
  m_dispatchInfo.m_stackAllocator := m_stackAlloc;
  m_collisionObjects := btCollisionObjectArray.create;
  m_collisionObjects.Initialize(btCollisionObject);
  m_collisionObjects.SetComparefunctionT(@btCollisionObjectCompare);
end;

destructor btCollisionWorld.destroy;
var i               : integer;
    collisionObject : btCollisionObject;
    bp              : btBroadphaseProxy;
begin
  //clean up remaining objects
  for i := 0 to m_collisionObjects.size-1 do begin
    collisionObject := m_collisionObjects[i]^;
    bp              := collisionObject.getBroadphaseHandle;
    if assigned(bp) then begin
     //- only clear the cached algorithms
      getBroadphase.getOverlappingPairCache.cleanProxyFromPairs(bp,m_dispatcher1);
     //FOSTODO!!!!!! -> Cleaningup the stuff - wrong !!!
     // comment in and debug ->
     getBroadphase.destroyProxy(bp,m_dispatcher1);
     //FOSTODO!!!!!! -> Cleaningup the stuff
     collisionObject.setBroadphaseHandle(nil);
    end;
  end;
  m_collisionObjects.Clear;
  inherited Destroy;
end;

procedure btCollisionWorld.setBroadphase(const pairCache: btBroadphaseInterface);
begin
  m_broadphasePairCache := pairCache;
end;

function btCollisionWorld.getBroadphase: btBroadphaseInterface;
begin
  Result := m_broadphasePairCache;
end;

function btCollisionWorld.getPairCache: btOverlappingPairCache;
begin
  Result := m_broadphasePairCache.getOverlappingPairCache;
end;

function btCollisionWorld.getDispatcher: btDispatcher;
begin
  result := m_dispatcher1;
end;

var reportMe:boolean = true;

procedure btCollisionWorld.updateSingleAabb(const colObj: btCollisionObject);
var minAabb,maxAabb,contactThreshold:btVector3;
    bp:btBroadphaseInterface;
begin
  colObj.getCollisionShape.getAabb(colObj.getWorldTransformP^, minAabb,maxAabb);
  //need to increase the aabb for contact thresholds
  contactThreshold.InitSame(gContactBreakingThreshold);
  minAabb -= contactThreshold;
  maxAabb += contactThreshold;
  bp := m_broadphasePairCache;
  //moving objects should be moderately sized, probably something wrong if not
  if colObj.isStaticObject OR ((maxAabb-minAabb).length2 < btScalar(1e12)) then begin
    bp.setAabb(colObj.getBroadphaseHandle,minAabb,maxAabb, m_dispatcher1);
  end else begin
    //something went wrong, investigate
    //this assert is unwanted in 3D modelers (danger of loosing work)
    colObj.setActivationState(btas_DISABLE_SIMULATION);
    if (reportMe and  assigned(m_debugDrawer)) then begin
      reportMe := false;
      m_debugDrawer.reportErrorWarning('Overflow in AABB, object removed from simulation');
      m_debugDrawer.reportErrorWarning('If you can reproduce this, please email (FirmOS: this is a port/check with official bullet)\n');
      m_debugDrawer.reportErrorWarning('Please include above information, your Platform, version of OS.\n');
      m_debugDrawer.reportErrorWarning('Thanks.\n');
    end;
  end;
end;

procedure btCollisionWorld.updateAabbs;
var colObj         : btCollisionObject;
    i              : Integer;
begin
  //.
  BT_PROFILE('updateAabbs');
  for i := 0 to m_collisionObjects.size-1 do begin
    colObj := m_collisionObjects[i]^;
    //only update aabb of active objects
    if m_forceUpdateAllAabbs or colObj.isActive then begin
      updateSingleAabb(colObj);
    end;
  end;
end;

procedure btCollisionWorld.setDebugDrawer(const debugDrawer: btIDebugDraw);
begin
  m_debugDrawer := debugDrawer;
end;

function btCollisionWorld.getDebugDrawer: btIDebugDraw;
begin
  Result := m_debugDrawer;
end;

procedure btCollisionWorld.debugDrawWorld;
var numManifolds    : integer;
    color           : btVector3;
    minAabb,maxAabb : btVector3;
    contactManifold : btPersistentManifold;
    i,j,numContacts : integer;
    cp              : PbtOManifoldPoint;
    colObj          : btCollisionObject;
begin
  if assigned(getDebugDrawer) and (DBG_DrawContactPoints in getDebugDrawer.getDebugMode) then begin
    numManifolds := m_dispatcher1.getNumManifolds;
    color.zero;
    for i := 0 to numManifolds-1 do begin
      contactManifold := m_dispatcher1.getManifoldByIndexInternal(i);
      //btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
      //btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
      numContacts  := contactManifold.getNumContacts;
      for j := 0 to numContacts-1 do begin
        cp := contactManifold.getContactPointP(j);
        getDebugDrawer.drawContactPoint(cp^.m_positionWorldOnB,cp^.m_normalWorldOnB,cp^.getDistance,cp^.getLifeTime,color);
      end;
    end;
  end;
  if assigned(getDebugDrawer) and ([DBG_DrawWireframe,DBG_DrawAabb] * getDebugDrawer.getDebugMode <> []) then begin
    for i:=0 to m_collisionObjects.size-1 do begin
      colObj := m_collisionObjects[i]^;
      if not (CF_DISABLE_VISUALIZE_OBJECT in colObj.getCollisionFlags) then begin
        if assigned(getDebugDrawer) and (DBG_DrawWireframe in getDebugDrawer.getDebugMode) then begin
          color.InitSame(1);
          case colObj.getActivationState of
            btas_ACTIVE_TAG           : color.init(1,1,1); //white
            btas_ISLAND_SLEEPING      : color.init(0,1,0); //green
            btas_WANTS_DEACTIVATION   : color.init(0,1,1); //türkis
            btas_DISABLE_DEACTIVATION : color.init(1,0,0); //red
            btas_DISABLE_SIMULATION   : color.init(1,1,0); //gelb
            else color.init(1,0,0);
          end;
          debugDrawObject(colObj.getWorldTransformP^,colObj.getCollisionShape,color);
        end;
        if assigned(m_debugDrawer) and (DBG_DrawAabb in m_debugDrawer.getDebugMode) then begin
          color.init(1,0,0);
          colObj.getCollisionShape.getAabb(colObj.getWorldTransformP^, minAabb,maxAabb);
          m_debugDrawer.drawAabb(minAabb,maxAabb,color);
      end;
      end;
    end;
  end;
end;

type

  { btDebugDrawcallback }

  btDebugDrawcallback =class(btInternalTriangleIndexCallback) // public btTriangleCallback, public btInternalTriangleIndexCallback
    m_debugDrawer : btIDebugDraw;
    m_color       : btVector3;
    m_worldTrans  : btTransform;
  public
    constructor create(const debugDrawer : btIDebugDraw ;const worldTrans : btTransform; const color : btVector3);
    procedure   processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer); override;
    procedure   internalProcessTriangleIndex(const triangle: PbtVector3; const partId, triangleIndex: integer); override;
  end;

{ btDebugDrawcallback }

constructor btDebugDrawcallback.create(const debugDrawer: btIDebugDraw; const worldTrans: btTransform; const color: btVector3);
begin
  m_debugDrawer := debugDrawer;
  m_color       := color;
  m_worldTrans  := worldTrans;
end;

{$HINTS OFF}
procedure btDebugDrawcallback.processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer);
var    wv0,wv1,wv2   : btVector3;
       normalColor,
       center,normal : btVector3;
begin
  wv0 := m_worldTrans*triangle[0];
  wv1 := m_worldTrans*triangle[1];
  wv2 := m_worldTrans*triangle[2];
  center := (wv0+wv1+wv2)*(1/3);
  normal := (wv1-wv0).cross(wv2-wv0);
  normal.normalize;
  normalColor.init (1,1,0);
  m_debugDrawer.drawLine(center,center+normal,normalColor);
  m_debugDrawer.drawLine(wv0,wv1,m_color);
  m_debugDrawer.drawLine(wv1,wv2,m_color);
  m_debugDrawer.drawLine(wv2,wv0,m_color);
end;
{$HINTS ON}

procedure btDebugDrawcallback.internalProcessTriangleIndex(const triangle: PbtVector3; const partId, triangleIndex: integer);
begin
  processTriangle(triangle,partId,triangleIndex);
end;


procedure btCollisionWorld.debugDrawObject(const worldTransform: btTransform; const shape: btCollisionShape; const color: btVector3);
var compoundShape : btCompoundShape;
    childTrans    : btTransform;
    colShape      : btCollisionShape;
    i             : integer;

    procedure _BoxShape;
    var halfExtents   : btVector3;
    begin
      halfExtents   := btBoxShape(shape).getHalfExtentsWithMargin;
      getDebugDrawer.drawBox(-halfExtents,halfExtents,worldTransform,color);
    end;
    procedure _SphereShape;
    var radius : btScalar;
    begin
      radius := btSphereShape(shape).getMargin;//radius doesn't include the margin, so draw with margin
      getDebugDrawer.drawSphere(radius, worldTransform, color);
    end;
    procedure _MultiSphere;
    var childTransform:btTransform;
        i:integer;
    begin
      childTransform.setIdentity;
      for i := btMultiSphereShape(shape).getSphereCount-1 downto 0 do begin
        childTransform.setOrigin(btMultiSphereShape(shape).getSpherePosition(i));
        getDebugDrawer.drawSphere(btMultiSphereShape(shape).getSphereRadius(i), worldTransform*childTransform, color);
      end;
    end;
    procedure _Capsule;
    var radius,halfHeight : btScalar;
        capsuleShape      : btCapsuleShape;
        capStart,capEnd,
        start             : btVector3;
        upAxis            : integer;
        childTransform    : btTransform;
    begin
      capsuleShape := btCapsuleShape(shape);
      radius       := capsuleShape.getRadius;
      halfHeight   := capsuleShape.getHalfHeight;
      upAxis       := capsuleShape.getUpAxis;
      capStart.zero; capEnd.zero;
      capStart[upAxis] := -halfHeight;
      capEnd[upAxis]   := halfHeight;
      // Draw the ends
      childTransform := worldTransform;
      childTransform.getOriginV^ := worldTransform * capStart;
      getDebugDrawer.drawSphere(radius, childTransform, color);

      childTransform := worldTransform;
      childTransform.getOriginV^ := worldTransform * capEnd;
      getDebugDrawer.drawSphere(radius, childTransform, color);
      // Draw some additional lines
      start := worldTransform.getOriginV^;

      capStart[(upAxis+1) mod 3] := radius;
      capEnd  [(upAxis+1) mod 3] := radius;
      getDebugDrawer.drawLine(start+worldTransform.GetBasisV^ * capStart,start+worldTransform.GetBasisV^ * capEnd, color);
      capStart[(upAxis+1) mod 3] := -radius;
      capEnd  [(upAxis+1) mod 3] := -radius;
      getDebugDrawer.drawLine(start+worldTransform.GetBasisV^ * capStart,start+worldTransform.GetBasisV^ * capEnd, color);

      capStart[(upAxis+1) mod 3] := 0;
      capEnd  [(upAxis+1) mod 3] := 0;

      capStart[(upAxis+2) mod 3] := radius;
      capEnd  [(upAxis+2) mod 3] := radius;
      getDebugDrawer.drawLine(start+worldTransform.GetBasisV^ * capStart,start+worldTransform.GetBasisV^ * capEnd, color);
      capStart[(upAxis+2) mod 3] := -radius;
      capEnd  [(upAxis+2) mod 3] := -radius;
      getDebugDrawer.drawLine(start+worldTransform.GetBasisV^ * capStart,start+worldTransform.GetBasisV^ * capEnd, color);
    end;
    procedure _Cone;
    var radius,height       : btScalar;
        start,offsetHeight,
        offsetRadius,yaxis,
        offset2Radius,xaxis : btVector3;
        upAxis              : integer;
    begin
      radius := btConeShape(shape).getRadius;//+btConeShape(shape)->getMargin();
      height := btConeShape(shape).getHeight;//+btConeShape(shape)->getMargin();
      start  := worldTransform.getOriginV^;
      upAxis := btConeShape(shape).getConeUpIndex;
      offsetHeight.zero;offsetRadius.zero;offset2Radius.zero;
      offsetHeight [upAxis]           := height * btScalar(0.5);
      offsetRadius [(upAxis+1) mod 3] := radius;
      offset2Radius[(upAxis+2) mod 3]  := radius;
      getDebugDrawer.drawLine(start+worldTransform.GetBasisV^ * (offsetHeight),start+worldTransform.GetBasisV^ * (-offsetHeight+offsetRadius),color);
      getDebugDrawer.drawLine(start+worldTransform.GetBasisV^ * (offsetHeight),start+worldTransform.GetBasisV^ * (-offsetHeight-offsetRadius),color);
      getDebugDrawer.drawLine(start+worldTransform.GetBasisV^ * (offsetHeight),start+worldTransform.GetBasisV^ * (-offsetHeight+offset2Radius),color);
      getDebugDrawer.drawLine(start+worldTransform.GetBasisV^ * (offsetHeight),start+worldTransform.GetBasisV^ * (-offsetHeight-offset2Radius),color);
      // Drawing the base of the cone
      yaxis.zero ; xaxis.zero;
      yaxis[upAxis] := 1;
      xaxis[(upAxis+1) mod 3] := 1;
      getDebugDrawer.drawArc(start-worldTransform.GetBasisV^*(offsetHeight),worldTransform.GetBasisV^*yaxis,worldTransform.GetBasisV^*xaxis,radius,radius,0,SIMD_2_PI,color,false,10.0);
    end;
    procedure _Cylinder;
    var cylinder   : btCylinderShape;
        upAxis     : integer;
        radius,
        halfHeight    : btScalar;
        offsetHeight,
        yaxis,xaxis,start,
        offsetRadius  : btVector3;
    begin
      cylinder   := btCylinderShape(shape);
      upAxis     := cylinder.getUpAxis;
      radius     := cylinder.getRadius;
      halfHeight := cylinder.getHalfExtentsWithMargin[upAxis];
      start      := worldTransform.getOriginV^;
      offsetHeight.zero;
      offsetHeight[upAxis] := halfHeight;
      offsetRadius.zero;
      offsetRadius[(upAxis+1) mod 3] := radius;
      getDebugDrawer.drawLine(start+worldTransform.GetBasisV^ * (offsetHeight+offsetRadius),start+worldTransform.GetBasisV^ * (-offsetHeight+offsetRadius),color);
      getDebugDrawer.drawLine(start+worldTransform.GetBasisV^ * (offsetHeight-offsetRadius),start+worldTransform.GetBasisV^ * (-offsetHeight-offsetRadius),color);

      // Drawing top and bottom caps of the cylinder
      yaxis.zero;
      yaxis[upAxis] := 1;
      xaxis.zero;
      xaxis[(upAxis+1) mod 3] := 1;
      getDebugDrawer.drawArc(start-worldTransform.GetBasisV^*(offsetHeight),worldTransform.GetBasisV^*yaxis,worldTransform.GetBasisV^*xaxis,radius,radius,0,SIMD_2_PI,color,false,btScalar(10));
      getDebugDrawer.drawArc(start+worldTransform.GetBasisV^*(offsetHeight),worldTransform.GetBasisV^*yaxis,worldTransform.GetBasisV^*xaxis,radius,radius,0,SIMD_2_PI,color,false,btScalar(10));
    end;
    procedure _StaticPlane;
    var planeConst,
        vecLen      : btScalar;
        planeNormal : btVector3;
        planeOrigin,
        vec0,vec1   : btVector3;
        pt0,pt1,pt2,
        pt3         : btVector3;
    begin
      planeConst    := btStaticPlaneShape(shape).getPlaneConstant;
      planeNormal   := btStaticPlaneShape(shape).getPlaneNormal;
      planeOrigin   := planeNormal * planeConst;
      btPlaneSpace1(planeNormal,vec0,vec1);
      vecLen        := 100;
      pt0           := planeOrigin + vec0*vecLen;
      pt1           := planeOrigin - vec0*vecLen;
      pt2           := planeOrigin + vec1*vecLen;
      pt3           := planeOrigin - vec1*vecLen;
      getDebugDrawer.drawLine(worldTransform*pt0,worldTransform*pt1,color);
      getDebugDrawer.drawLine(worldTransform*pt2,worldTransform*pt3,color);
    end;
    procedure _Default;
    var aabbMax,aabbMin : btVector3;
        drawCallback    : btDebugDrawcallback;
        a,b,wa,wb       : btVector3;
        i               : integer;
    begin
      if shape.isConcave then begin
        ///@todo pass camera, for some culling? no -> we are not a graphics lib
        aabbMax.InitSame(BT_LARGE_FLOAT);
        aabbMin.InitSame(-BT_LARGE_FLOAT);
        drawCallback := btDebugDrawcallback.create(getDebugDrawer,worldTransform,color);
        btConcaveShape(shape).processAllTriangles(drawCallback,aabbMin,aabbMax);
        drawCallback.Free;
      end;
      if shape.getShapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE then begin
        //todo: pass camera for some culling
        aabbMax.InitSame(BT_LARGE_FLOAT);
        aabbMin.InitSame(-BT_LARGE_FLOAT);
        drawCallback := btDebugDrawcallback.create(getDebugDrawer,worldTransform,color);
        btConvexTriangleMeshShape(shape).getMeshInterface.InternalProcessAllTriangles(drawCallback,aabbMin,aabbMax);
        drawCallback.Free;
      end;
      // for polyhedral shapes
      if shape.isPolyhedral then begin
        for i := 0 to btPolyhedralConvexShape(shape).getNumEdges-1 do begin
          btPolyhedralConvexShape(shape).getEdge(i,a,b);
          wa := worldTransform * a;
          wb := worldTransform * b;
          getDebugDrawer.drawLine(wa,wb,color);
        end;
      end;
    end;

begin
  // Draw a small simplex at the center of the object
  getDebugDrawer.drawTransform(worldTransform,1);
  if shape.getShapeType = COMPOUND_SHAPE_PROXYTYPE then begin
    compoundShape := btCompoundShape(shape);
    for i:= compoundShape.getNumChildShapes-1 downto 0 do begin
      childTrans := compoundShape.getChildTransform(i);
      colShape   := compoundShape.getChildShape(i);
      debugDrawObject(worldTransform*childTrans,colShape,color);
    end;
  end  else begin
    case shape.getShapeType of
      BOX_SHAPE_PROXYTYPE          : _BoxShape;
      SPHERE_SHAPE_PROXYTYPE       : _Sphereshape;
      MULTI_SPHERE_SHAPE_PROXYTYPE : _MultiSphere;
      CAPSULE_SHAPE_PROXYTYPE      : _Capsule;
      CONE_SHAPE_PROXYTYPE         : _Cone;
      CYLINDER_SHAPE_PROXYTYPE     : _Cylinder;
      STATIC_PLANE_PROXYTYPE       : _StaticPlane;
     else _Default;
    end;
  end;
end;

function btCollisionWorld.getNumCollisionObjects: integer;
begin
  result := m_collisionObjects.size;
end;

type

  { btSingleRayCallback }

  btSingleRayCallback =class(btBroadphaseRayCallback)
    m_rayFromWorld,
    m_rayToWorld    : btVector3;
    m_rayFromTrans  : btTransform;
    m_rayToTrans    : btTransform;
    m_hitNormal     : btVector3;
    m_world         : btCollisionWorld;
    m_resultCallback: btCW_RayResultCallback;
    constructor create(const rayFromWorld,rayToWorld : btVector3 ; const world:btCollisionWorld;const resultCallback : btCW_RayResultCallback);
    function    process(const proxy: btBroadphaseProxy): boolean; override;
  end;

  constructor btSingleRayCallback.create(const rayFromWorld, rayToWorld: btVector3; const world: btCollisionWorld; const resultCallback: btCW_RayResultCallback);
  var rayDir : btVector3;
  begin
    m_rayFromWorld   := rayFromWorld;
    m_rayToWorld     := rayToWorld;
    m_world          := world;
    m_resultCallback := resultCallback;
    m_rayFromTrans.setIdentity;
    m_rayFromTrans.setOrigin(m_rayFromWorld);
    m_rayToTrans.setIdentity;
    m_rayToTrans.setOrigin(m_rayToWorld);
    rayDir := rayToWorld-rayFromWorld;
    rayDir.normalize;

    ///what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT
    m_rayDirectionInverse[0] := btDecide(rayDir[0]= 0 , BT_LARGE_FLOAT , 1/rayDir[0]);
    m_rayDirectionInverse[1] := btDecide(rayDir[1]= 0 , BT_LARGE_FLOAT , 1/rayDir[1]);
    m_rayDirectionInverse[2] := btDecide(rayDir[2]= 0 , BT_LARGE_FLOAT , 1/rayDir[2]);

    m_signs[0]   := btDecide(m_rayDirectionInverse[0] < 0, 1 , 0);
    m_signs[1]   := btDecide(m_rayDirectionInverse[1] < 0, 1 , 0);
    m_signs[2]   := btDecide(m_rayDirectionInverse[2] < 0, 1 , 0);
    m_lambda_max := rayDir.dot(m_rayToWorld-m_rayFromWorld);
  end;

  function btSingleRayCallback.process(const proxy: btBroadphaseProxy): boolean;
  var collisionObject : btCollisionObject;
  begin
    ///terminate further ray tests, once the closestHitFraction reached zero
    if (m_resultCallback.m_closestHitFraction = 0) then begin
      exit(false);
    end;
    collisionObject := btCollisionObject(proxy.m_clientObject);
    //only perform raycast if filterMask matches
    if m_resultCallback.needsCollision(collisionObject.getBroadphaseHandle) then begin
      m_world.rayTestSingle(m_rayFromTrans,m_rayToTrans,collisionObject,collisionObject.getCollisionShape,collisionObject.getWorldTransformP^,m_resultCallback);
    end;
    Result := true;
  end;



procedure btCollisionWorld.rayTest(const rayFromWorld, rayToWorld: btVector3; const resultCallback: btCW_RayResultCallback);
var rayCB:btSingleRayCallback;
begin
  //BT_PROFILE("rayTest");
  /// use the broadphase to accelerate the search for objects, based on their aabb
  /// and for each object with ray-aabb overlap, perform an exact ray test
  rayCB := btSingleRayCallback.create(rayFromWorld,rayToWorld,self,resultCallback);
{$ifndef USE_BRUTEFORCE_RAYBROADPHASE}
  m_broadphasePairCache.rayTest(rayFromWorld,rayToWorld,rayCB,cbtNullVector,cbtNullVector);
{$else}
  for (int i=0;i<this->getNumCollisionObjects();i++)
  {
  	rayCB.process(m_collisionObjects[i]->getBroadphaseHandle());
  }
{$endif} //USE_BRUTEFORCE_RAYBROADPHASE
  rayCB.Free;
end;

type

  { btSingleSweepCallback }

  btSingleSweepCallback = class(btBroadphaseRayCallback)
    m_convexFromTrans   : btTransform;
    m_convexToTrans     : btTransform;
    m_hitNormal         : btVector3;
    m_world             : btCollisionWorld;
    m_resultCallback    : btCW_ConvexResultCallback;
    m_allowedCcdPenetration : btScalar;
    m_castShape             : btConvexShape;
    constructor create(const castShape : btConvexShape; const convexFromTrans,convexToTrans : btTransform;const world : btCollisionWorld ;const resultCallback : btCW_ConvexResultCallback ; const allowedPenetration : btScalar);
    function    process(const proxy: btBroadphaseProxy): boolean; override;
  end;

{ btSingleSweepCallback }

constructor btSingleSweepCallback.create(const castShape: btConvexShape; const convexFromTrans, convexToTrans: btTransform; const world: btCollisionWorld; const resultCallback: btCW_ConvexResultCallback; const allowedPenetration: btScalar);
var unnormalizedRayDir,rayDir : btVector3;
begin
  m_convexFromTrans := convexFromTrans;
  m_convexToTrans   := convexToTrans;
  m_world           := world;
  m_resultCallback  := resultCallback;
  m_allowedCcdPenetration := allowedPenetration;
  m_castShape             := castShape;

  unnormalizedRayDir := convexToTrans.getOriginV^-m_convexFromTrans.getOriginV^;
  rayDir             := unnormalizedRayDir.normalized;
  ///what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT

  m_rayDirectionInverse[0] := btDecide(rayDir[0] = 0 , BT_LARGE_FLOAT , 1 / rayDir[0]);
  m_rayDirectionInverse[1] := btDecide(rayDir[1] = 0 , BT_LARGE_FLOAT , 1 / rayDir[1]);
  m_rayDirectionInverse[2] := btDecide(rayDir[2] = 0 , BT_LARGE_FLOAT , 1 / rayDir[2]);

  m_signs[0] := btDecide (m_rayDirectionInverse[0] < 0, 1 ,0);
  m_signs[1] := btDecide (m_rayDirectionInverse[1] < 0, 1 ,0);
  m_signs[2] := btDecide (m_rayDirectionInverse[2] < 0, 1 ,0);

  m_lambda_max := rayDir.dot(unnormalizedRayDir);
end;

function btSingleSweepCallback.process(const proxy: btBroadphaseProxy): boolean;
var collisionObject:btCollisionObject;
begin
  ///terminate further convex sweep tests, once the closestHitFraction reached zero
  if m_resultCallback.m_closestHitFraction = 0 then begin
    result :=false;
  end;
  collisionObject := btCollisionObject(proxy.m_clientObject);
  //only perform raycast if filterMask matches
  if m_resultCallback.needsCollision(collisionObject.getBroadphaseHandle) then begin
    //RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
    m_world.objectQuerySingle(m_castShape, m_convexFromTrans,m_convexToTrans,collisionObject,collisionObject.getCollisionShape,collisionObject.getWorldTransformP^,m_resultCallback,m_allowedCcdPenetration);
  end;
  Result := true;
end;

procedure btCollisionWorld.convexSweepTest(const castShape: btConvexShape; const convexFromWorld, convexToWorld: btTransform; const resultCallback: btCW_ConvexResultCallback; const allowedCcdPenetration: btScalar);
var   R,convexFromTrans,convexToTrans : btTransform;
      castShapeAabbMin, castShapeAabbMax : btVector3;
      zeroLinVel,linVel, angVel : btVector3;
      convexCB : btSingleSweepCallback;

begin
  BT_PROFILE("convexSweepTest");
  /// use the broadphase to accelerate the search for objects, based on their aabb
  /// and for each object with ray-aabb overlap, perform an exact ray test
  /// unfortunately the implementation for rayTest and convexSweepTest duplicated, albeit practically identical

  convexFromTrans := convexFromWorld;
  convexToTrans   := convexToWorld;
  // Compute AABB that encompasses angular movement
  btTransformUtil.calculateVelocity (convexFromTrans, convexToTrans, 1.0, linVel, angVel);
  zeroLinVel.zero;
  R.setIdentity;
  R.setRotation(convexFromTrans.getRotation);
  castShape.calculateTemporalAabb (R,zeroLinVel, angVel, 1.0, castShapeAabbMin, castShapeAabbMax);

{$ifndef USE_BRUTEFORCE_RAYBROADPHASE}
  convexCB:=btSingleSweepCallback.create(castShape,convexFromWorld,convexToWorld,self,resultCallback,allowedCcdPenetration);
  m_broadphasePairCache.rayTest(convexFromTrans.getOriginV^,convexToTrans.getOriginV^,convexCB,castShapeAabbMin,castShapeAabbMax);
{$else}
  ///// go over all objects, and if the ray intersects their aabb + cast shape aabb,
  //// do a ray-shape query using convexCaster (CCD)
  //int i;
  //for (i=0;i<m_collisionObjects.size();i++)
  //{
  //      btCollisionObject*      collisionObject= m_collisionObjects[i];
  //      //only perform raycast if filterMask matches
  //      if(resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) {
  //              //RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
  //              btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
  //              collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
  //              AabbExpand (collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
  //              btScalar hitLambda = btScalar(1.); //could use resultCallback.m_closestHitFraction, but needs testing
  //              btVector3 hitNormal;
  //              if (btRayAabb(convexFromWorld.getOrigin(),convexToWorld.getOrigin(),collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,hitNormal))
  //              {
  //                      objectQuerySingle(castShape, convexFromTrans,convexToTrans,
  //                              collisionObject,
  //                              collisionObject->getCollisionShape(),
  //                              collisionObject->getWorldTransform(),
  //                              resultCallback,
  //                              allowedCcdPenetration);
  //              }
  //      }
  //}
{$endif} //USE_BRUTEFORCE_RAYBROADPHASE
end;

type

  { btBridgedManifoldResult }

  btBridgedManifoldResult = class(btManifoldResult)
    m_resultCallback : btCW_ContactResultCallback;
    constructor create(const obj0,obj1 : btCollisionObject;const resultCallback : btCW_ContactResultCallback);
    procedure   addContactPoint(const normalOnBInWorld, pointInWorld: btVector3; const depth: btScalar); override;
  end;

  constructor btBridgedManifoldResult.create(const obj0, obj1: btCollisionObject; const resultCallback: btCW_ContactResultCallback);
  begin
    inherited create(obj0,obj1);
    m_resultCallback := resultCallback;
  end;

  procedure btBridgedManifoldResult.addContactPoint(const normalOnBInWorld, pointInWorld: btVector3; const depth: btScalar);
  var isSwapped            : Boolean;
      pointA,localA,localB : btVector3;
      newPt                : btOManifoldPoint;
      obj0,obj1            : btCollisionObject;
  begin
    isSwapped := btCollisionObject(m_manifoldPtr.getBody0) <> m_body0;
    pointA    := pointInWorld + normalOnBInWorld * depth;
    if isSwapped then begin
      localA := m_rootTransB.invXform(pointA );
      localB := m_rootTransA.invXform(pointInWorld);
    end else begin
      localA := m_rootTransA.invXform(pointA );
      localB := m_rootTransB.invXform(pointInWorld);
    end;

    newPt.Init(localA,localB,normalOnBInWorld,depth);
    newPt.m_positionWorldOnA := pointA;
    newPt.m_positionWorldOnB := pointInWorld;
    //BP mod, store contact triangles.
    if isSwapped then begin
      newPt.m_partId0 := m_partId1;
      newPt.m_partId1 := m_partId0;
      newPt.m_index0  := m_index1;
      newPt.m_index1  := m_index0;
    end else begin
      newPt.m_partId0 := m_partId0;
      newPt.m_partId1 := m_partId1;
      newPt.m_index0  := m_index0;
      newPt.m_index1  := m_index1;
    end;
    //experimental feature info, for per-triangle material etc.
    obj0 := btCollisionObject(btDecide(isSwapped,Tobject(m_body1), TObject(m_body0)));
    obj1 := btCollisionObject(btDecide(isSwapped,Tobject(m_body0), TObject(m_body1)));
    m_resultCallback.addSingleResult(@newPt,obj0,newPt.m_partId0,newPt.m_index0,obj1,newPt.m_partId1,newPt.m_index1);
  end;


type

  { btSingleContactCallback }

  btSingleContactCallback = class(btBroadphaseAabbCallback)
    m_collisionObject : btCollisionObject;
    m_world           : btCollisionWorld;
    m_resultCallback  : btCW_ContactResultCallback;
    constructor create(const collisionObject : btCollisionObject; const  world :btCollisionWorld;const resultCallback : btCW_ContactResultCallback);
    function    process(const proxy: btBroadphaseProxy): boolean; override;
  end;

{ btSingleContactCallback }

constructor btSingleContactCallback.create(const collisionObject: btCollisionObject; const world: btCollisionWorld; const resultCallback: btCW_ContactResultCallback);
begin
  m_collisionObject := collisionObject;
  m_world           := world;
  m_resultCallback  := resultCallback;
end;

function btSingleContactCallback.process(const proxy: btBroadphaseProxy): boolean;
var collisionObject : btCollisionObject;
    algorithm       : btCollisionAlgorithm;
    contactPointResult : btBridgedManifoldResult;
begin
  collisionObject := btCollisionObject(proxy.m_clientObject);
  if collisionObject = m_collisionObject then begin
   exit(true);
  end;

  //only perform raycast if filterMask matches
  if m_resultCallback.needsCollision(collisionObject.getBroadphaseHandle) then begin
    algorithm := m_world.m_dispatcher1.findAlgorithm(m_collisionObject,collisionObject);
    if assigned(algorithm) then begin
      contactPointResult := btBridgedManifoldResult.create(m_collisionObject,collisionObject, m_resultCallback);
      //discrete collision detection query
      algorithm.processCollision(m_collisionObject,collisionObject, m_world.getDispatchInfo,btManifoldResult(contactPointResult));
      //algorithm->~btCollisionAlgorithm();
      m_world.m_dispatcher1.freeCollisionAlgorithm(algorithm);
      contactPointResult.Free;
    end;
  end;
  Result:=true;
end;

///contactTest performs a discrete collision test against all objects in the btCollisionWorld, and calls the resultCallback.
///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
procedure btCollisionWorld.contactTest(const colObj: btCollisionObject; const resultCallback: btCW_ContactResultCallback);
var   aabbMin,aabbMax : btVector3;
      contactCB       : btSingleContactCallback;
begin
  colObj.getCollisionShape.getAabb(colObj.getWorldTransformP^,aabbMin,aabbMax);
  contactCB := btSingleContactCallback.create(colObj,self,resultCallback);
  m_broadphasePairCache.aabbTest(aabbMin,aabbMax,contactCB);
  contactCB.Free;
end;

procedure btCollisionWorld.contactPairTest(const colObjA, colObjB: btCollisionObject; const resultCallback: btCW_ContactResultCallback);
var     algorithm : btCollisionAlgorithm;
        contactPointResult : btBridgedManifoldResult;
begin
  algorithm := m_dispatcher1.findAlgorithm(colObjA,colObjB);
  if assigned(algorithm) then begin
    contactPointResult := btBridgedManifoldResult.create(colObjA,colObjB, resultCallback);
    //discrete collision detection query
    algorithm.processCollision(colObjA,colObjB, getDispatchInfo,btManifoldResult(contactPointResult));
    //algorithm->~btCollisionAlgorithm();
    m_dispatcher1.freeCollisionAlgorithm(algorithm);
    contactPointResult.Free;
  end;
end;

type
  //ConvexCast::CastResult

  { btCW_BridgeTriangleRaycastCallback }

  btCW_BridgeTriangleRaycastCallback=class(btTriangleRaycastCallback)
    m_resultCallback       : btCW_RayResultCallback;
    m_collisionObject      : btCollisionObject;
    m_triangleMesh         : btConcaveShape;// btTriangleMeshShape;
    m_colObjWorldTransform : btTransform;
    constructor create(const  from,too : btVector3 ;const resultCallback : btCW_RayResultCallback ; const collisionObject : btCollisionObject;const triangleMesh : btConcaveShape;const colObjWorldTransform: btTransform);
    function    reportHit(const hitNormalLocal : btVector3; const hitFraction : btScalar; const partId, triangleIndex : integer):btScalar;override;
  end;

  { btCW_BridgeTriangleRaycastCallback }

  constructor btCW_BridgeTriangleRaycastCallback.create(const from, too: btVector3; const resultCallback: btCW_RayResultCallback; const collisionObject: btCollisionObject; const triangleMesh: btConcaveShape; const colObjWorldTransform: btTransform);
  begin
    inherited Create(from,too, resultCallback.m_flags);
    m_resultCallback        := resultCallback;
    m_collisionObject       := collisionObject;
    m_triangleMesh          := triangleMesh;
    m_colObjWorldTransform  := colObjWorldTransform;
  end;

  function btCW_BridgeTriangleRaycastCallback.reportHit(const hitNormalLocal: btVector3; const hitFraction: btScalar; const partId, triangleIndex: integer): btScalar;
  var shapeInfo      : btCW_LocalShapeInfo;
      hitNormalWorld : btVector3;
      rayResult      : btCW_LocalRayResult;
  begin
    shapeInfo.m_shapePart     := partId;
    shapeInfo.m_triangleIndex := triangleIndex;
    hitNormalWorld            := m_colObjWorldTransform.GetBasisV^ * hitNormalLocal;
    rayResult.Init(m_collisionObject,@shapeInfo,hitNormalWorld,hitFraction);
    Result := m_resultCallback.addSingleResult(rayResult,true);
  end;

type

  { btCW_BridgeTriangleConvexcastCallback }

  btCW_BridgeTriangleConvexcastCallback=class(btTriangleConvexcastCallback)
    m_resultCallback       : btCW_ConvexResultCallback;
    m_collisionObject      : btCollisionObject;
    m_triangleMesh         : btConcaveShape;// btTriangleMeshShape;
    m_fos_normalinws       : boolean;
    constructor create(const castShape:btConvexShape ;const  from,too : btTransform ;const resultCallback : btCW_ConvexResultCallback ; const collisionObject : btCollisionObject;const triangleMesh : btConcaveShape;const triangleToWorld: btTransform;const normalinws:boolean);
    function    reportHit(const hitNormalLocal,hitPointLocal : btVector3; const hitFraction : btScalar; const partId, triangleIndex : integer):btScalar;override;
  end;

{ btCW_BridgeTriangleConvexcastCallback }

constructor btCW_BridgeTriangleConvexcastCallback.create(const castShape: btConvexShape; const from, too: btTransform; const resultCallback: btCW_ConvexResultCallback;const collisionObject: btCollisionObject; const triangleMesh: btConcaveShape; const triangleToWorld: btTransform;const normalinws:boolean);
begin
  inherited Create(castShape, from,too, triangleToWorld, triangleMesh.getMargin);
  m_resultCallback  := resultCallback;
  m_collisionObject := collisionObject;
  m_triangleMesh    := triangleMesh;
  m_fos_normalinws  := normalinws;
end;

function btCW_BridgeTriangleConvexcastCallback.reportHit(const hitNormalLocal, hitPointLocal: btVector3; const hitFraction: btScalar; const partId, triangleIndex: integer): btScalar;
var shapeInfo    : btCW_LocalShapeInfo;
    convexResult : btCW_LocalConvexResult;
begin
  shapeInfo.m_shapePart     := partId;
  shapeInfo.m_triangleIndex := triangleIndex;
  if hitFraction <= m_resultCallback.m_closestHitFraction then begin
    convexResult.Init(m_collisionObject,@shapeInfo,hitNormalLocal,hitPointLocal,hitFraction);
    result := m_resultCallback.addSingleResult(convexResult,m_fos_normalinws);
    exit;
  end;
  Result := hitFraction;
end;

type

  { btCW_LocalInfoAdder2 }

  btCW_LocalInfoAdder2 = class(btCW_RayResultCallback)
   m_i : integer;
   m_userCallback : btCW_RayResultCallback;
   constructor create(const i:integer ; const user:btCW_RayResultCallback);
   function    addSingleResult(const r: btCW_LocalRayResult; const normalInWorldSpace: Boolean): btScalar; override;
  end;

  btCW_LocalInfoAdder = class(btCW_ConvexResultCallback)
   m_i : integer;
   m_userCallback : btCW_ConvexResultCallback;
   constructor create(const i:integer ; const user:btCW_ConvexResultCallback);
   function    addSingleResult(const r: btCW_LocalConvexResult; const b: Boolean): btScalar; override;
  end;

constructor btCW_LocalInfoAdder.create(const i: integer; const user: btCW_ConvexResultCallback);
begin
  m_i := i;
  m_userCallback := user;
end;

function btCW_LocalInfoAdder.addSingleResult(const r: btCW_LocalConvexResult; const b: Boolean): btScalar;
var shapeInfo : btCW_LocalShapeInfo;
    lr        : btCW_LocalConvexResult;
begin
  shapeInfo.m_shapePart     := -1;
  shapeInfo.m_triangleIndex := m_i;
  if r.m_localShapeInfo = nil then begin
    lr:=r;
    lr.m_localShapeInfo := @shapeInfo;
    Result := m_userCallback.addSingleResult(lr, b);
  end else begin
    Result := m_userCallback.addSingleResult(r, b);
  end;
end;



{ btCW_LocalInfoAdder2 }

constructor btCW_LocalInfoAdder2.create(const i: integer; const user: btCW_RayResultCallback);
begin
   m_i := i;
   m_userCallback := user;
end;

function btCW_LocalInfoAdder2.addSingleResult(const r: btCW_LocalRayResult; const normalInWorldSpace: Boolean): btScalar;
var   shapeInfo : btCW_LocalShapeInfo;
      lr        : btCW_LocalRayResult;
begin
  shapeInfo.m_shapePart     := -1;
  shapeInfo.m_triangleIndex := m_i;
  if (r.m_localShapeInfo = nil) then begin
    lr:=r;
    lr.m_localShapeInfo := @shapeInfo;
    Result := m_userCallback.addSingleResult(lr, normalInWorldSpace);
  end else begin
    Result := m_userCallback.addSingleResult(r, normalInWorldSpace);
  end;
end;




class procedure btCollisionWorld.rayTestSingle(const rayFromTrans, rayToTrans: btTransform; const collisionObject: btCollisionObject; const collisionShape: btCollisionShape; const colObjWorldTransform: btTransform; const resultCallback: btCW_RayResultCallback);
var pointShape         : btSphereShape;
    castshape          : btConvexShape;
    procedure _Convex;
    var convexShape        : btConvexShape;
        castResult         : btConvexCastResult;
        localRayResult     : btCW_LocalRayResult;
        convexCaster       : btSubsimplexConvexCast;
        simplexSolver      : btVoronoiSimplexSolver;
    begin
      //              BT_PROFILE("rayTestConvex");
      castResult            := btConvexCastResult.create;
      castResult.m_fraction := resultCallback.m_closestHitFraction;
      convexShape           := btConvexShape(collisionShape);
      simplexSolver         := btVoronoiSimplexSolver.Create;
      convexCaster          := btSubsimplexConvexCast.create(castShape,convexShape,simplexSolver);
      if convexCaster.calcTimeOfImpact(rayFromTrans,rayToTrans,colObjWorldTransform,colObjWorldTransform,castResult) then begin
        //add hit
        if castResult.m_normal.length2 > 0.0001 then begin
          if castResult.m_fraction < resultCallback.m_closestHitFraction then begin
            //rotate normal into worldspace
            castResult.m_normal := rayFromTrans.GetBasisV^ * castResult.m_normal;
            castResult.m_normal.normalize;
            localRayResult.Init(collisionObject,nil,castResult.m_normal,castResult.m_fraction);
            resultCallback.addSingleResult(localRayResult, true); //normalinworldspace=true ?FOS
          end;
        end;
      end;
      convexcaster.free;
      simplexSolver.Free;
      castResult.Free;
    end;
    procedure _OptTriangleMesh;
    var triangleMesh            : btBvhTriangleMeshShape;
        worldTocollisionObject  : btTransform;
        rayFromLocal,rayToLocal : btVector3;
        rcb                     : btCW_BridgeTriangleRaycastCallback;
    begin
      triangleMesh            := btBvhTriangleMeshShape(collisionShape);
      worldTocollisionObject  := colObjWorldTransform.inverse;
      rayFromLocal            := worldTocollisionObject * rayFromTrans.getOriginV^;
      rayToLocal              := worldTocollisionObject * rayToTrans.getOriginV^;
      rcb                     := btCW_BridgeTriangleRaycastCallback.create(rayFromLocal,rayToLocal,resultCallback,collisionObject,triangleMesh,colObjWorldTransform);
      rcb.m_hitFraction       := resultCallback.m_closestHitFraction;
      triangleMesh.performRaycast(rcb,rayFromLocal,rayToLocal);
      rcb.free;
    end;
    procedure _GenericMesh;
    var concaveShape:btConcaveShape;
        worldTocollisionObject  : btTransform;
        rayFromLocal,rayToLocal,
        rayAabbMinLocal,
        rayAabbMaxLocal         : btVector3;
        rcb                     : btCW_BridgeTriangleRaycastCallback;
    begin
      //generic (slower) case
      concaveShape           := btConcaveShape(collisionShape);
      worldTocollisionObject := colObjWorldTransform.inverse;
      rayFromLocal           := worldTocollisionObject * rayFromTrans.getOriginV^;
      rayToLocal             := worldTocollisionObject * rayToTrans.getOriginV^;
      rcb                    := btCW_BridgeTriangleRaycastCallback.Create (rayFromLocal,rayToLocal,resultCallback,collisionObject,concaveShape, colObjWorldTransform);
      rcb.m_hitFraction      := resultCallback.m_closestHitFraction;
      rayAabbMinLocal        := rayFromLocal;
      rayAabbMinLocal.setMin(rayToLocal);
      rayAabbMaxLocal        := rayFromLocal;
      rayAabbMaxLocal.setMax(rayToLocal);
      concaveShape.processAllTriangles(rcb,rayAabbMinLocal,rayAabbMaxLocal);
      rcb.free;
    end;
    procedure _Compound;
    var compoundShape       : btCompoundShape;
        i                   : integer;
        childTrans,
        childWorldTrans     : btTransform;
        childCollisionShape,
        saveCollisionShape  : btCollisionShape;
        my_cb               : btCW_LocalInfoAdder2;
    begin
      // BT_PROFILE("rayTestCompound");
      ///@todo: use AABB tree or other BVH acceleration structure, see btDbvt
      if collisionShape.isCompound then begin
        compoundShape := btCompoundShape(collisionShape);
        for i:=0 to compoundShape.getNumChildShapes-1 do begin
          childTrans          := compoundShape.getChildTransform(i);
          childCollisionShape := compoundShape.getChildShape(i);
          childWorldTrans     := colObjWorldTransform * childTrans;
          saveCollisionShape  := collisionObject.getCollisionShape;  // replace collision shape so that callback can determine the triangle
          collisionObject.internalSetTemporaryCollisionShape(childCollisionShape);
          my_cb := btCW_LocalInfoAdder2.create(i, resultCallback);
          my_cb.m_closestHitFraction := resultCallback.m_closestHitFraction;
          rayTestSingle(rayFromTrans,rayToTrans,collisionObject,childCollisionShape,childWorldTrans,my_cb);
          // restore
          collisionObject.internalSetTemporaryCollisionShape(saveCollisionShape);
        end;
      end;
    end;
begin
  pointShape := btSphereShape.Create(0);
  pointShape.setMargin(0);
  castShape := pointShape;
  if collisionShape.isConvex then begin
    _Convex;
  end else begin
    if collisionShape.isConcave then begin
      //BT_PROFILE("rayTestConcave");
      if collisionShape.getShapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE then begin
        ///optimized version for btBvhTriangleMeshShape
        _OptTriangleMesh;
      end else begin
        _GenericMesh;
      end;
    end else begin
      _Compound;
    end;
  end;
  pointShape.Free;
end;

class procedure btCollisionWorld.objectQuerySingle(const castShape: btConvexShape; const convexFromTrans, convexToTrans: btTransform; const collisionObject: btCollisionObject; const collisionShape: btCollisionShape; const colObjWorldTransform: btTransform; const resultCallback: btCW_ConvexResultCallback; const allowedPenetration: btScalar);

  procedure _Convex;
  var castResult  : btConvexCastResult;
      convexShape : btConvexShape;
      simplexSolver : btVoronoiSimplexSolver;
      gjkEpaPenetrationSolver : btGjkEpaPenetrationDepthSolver;
      convexCaster1           : btContinuousConvexCollision;
      localConvexResult       : btCW_LocalConvexResult;
  begin
    //BT_PROFILE("convexSweepConvex");
    castResult := btConvexCastResult.create;
    castResult.m_allowedPenetration := allowedPenetration;
    castResult.m_fraction           := resultCallback.m_closestHitFraction;//btScalar(1.);//??
    convexShape                     := btConvexShape(collisionShape);
    simplexSolver                   := btVoronoiSimplexSolver.Create;
    gjkEpaPenetrationSolver         := btGjkEpaPenetrationDepthSolver.Create;
    convexCaster1                   := btContinuousConvexCollision.create(castShape,convexShape,simplexSolver,gjkEpaPenetrationSolver);
    if  convexCaster1.calcTimeOfImpact(convexFromTrans,convexToTrans,colObjWorldTransform,colObjWorldTransform,castResult) then begin
      //add hit
      if castResult.m_normal.length2 > 0.0001 then begin
        if castResult.m_fraction < resultCallback.m_closestHitFraction then begin
          castResult.m_normal.normalize;
          localConvexResult.Init(collisionObject,nil,castResult.m_normal,castResult.m_hitPoint,castResult.m_fraction);
          resultCallback.addSingleResult(localConvexResult, true);
        end;
      end;
    end;
    convexCaster1.Free;
    simplexSolver.Free;
    gjkEpaPenetrationSolver.Free;
    castResult.Free;
  end;
  procedure _Concave;
    procedure _ConvexSweepConcave;
    var concaveShape:btConcaveShape;
        worldTocollisionObject,
        rotationXform            : btTransform;
        convexFromLocal,
        convexToLocal            : btVector3;
        tccb                     : btCW_BridgeTriangleConvexcastCallback;
        boxMinLocal, boxMaxLocal,
        rayAabbMinLocal,rayAabbMaxLocal : btVector3;
    begin
      //BT_PROFILE("convexSweepConcave");
      concaveShape := btConcaveShape(collisionShape);
      worldTocollisionObject := colObjWorldTransform.inverse;
      convexFromLocal        := worldTocollisionObject * convexFromTrans.getOriginV^;
      convexToLocal          := worldTocollisionObject * convexToTrans.getOriginV^;
      // rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
      rotationXform.init(worldTocollisionObject.GetBasisV^ * convexToTrans.GetBasisV^);
      tccb               := btCW_BridgeTriangleConvexcastCallback.create(castShape, convexFromTrans,convexToTrans,resultCallback,collisionObject,concaveShape, colObjWorldTransform,false);
      tccb.m_hitFraction := resultCallback.m_closestHitFraction;
      castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);
      rayAabbMinLocal := convexFromLocal;
      rayAabbMinLocal.setMin(convexToLocal);
      rayAabbMaxLocal := convexFromLocal;
      rayAabbMaxLocal.setMax(convexToLocal);
      rayAabbMinLocal += boxMinLocal;
      rayAabbMaxLocal += boxMaxLocal;
      concaveShape.processAllTriangles(tccb,rayAabbMinLocal, rayAabbMaxLocal);
      tccb.Free;
    end;

    procedure _Triangle;
    var triangleMesh             : btBvhTriangleMeshShape;
        worldTocollisionObject,
        rotationXform            : btTransform;
        convexFromLocal,
        convexToLocal            : btVector3;
        tccb                     : btCW_BridgeTriangleConvexcastCallback;
        boxMinLocal, boxMaxLocal : btVector3;
    begin
      //BT_PROFILE("convexSweepbtBvhTriangleMesh");
      triangleMesh           := btBvhTriangleMeshShape(collisionShape);
      worldTocollisionObject := colObjWorldTransform.inverse;
      convexFromLocal        := worldTocollisionObject * convexFromTrans.getOriginV^;
      convexToLocal          := worldTocollisionObject * convexToTrans.getOriginV^;
      // rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
      rotationXform.init(worldTocollisionObject.GetBasisV^ * convexToTrans.GetBasisV^);
      tccb               := btCW_BridgeTriangleConvexcastCallback.create(castShape, convexFromTrans,convexToTrans,resultCallback,collisionObject,triangleMesh, colObjWorldTransform,true);
      tccb.m_hitFraction := resultCallback.m_closestHitFraction;
      castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);
      triangleMesh.performConvexcast(tccb,convexFromLocal,convexToLocal,boxMinLocal, boxMaxLocal);
      tccb.Free;
    end;

  begin
    if collisionShape.getShapeType=TRIANGLE_MESH_SHAPE_PROXYTYPE then begin
      _Triangle;
    end else begin
      _ConvexSweepConcave;
    end;
  end;
  procedure _Compound;
    var compoundShape       : btCompoundShape;
        i                   : integer;
        childTrans,
        childWorldTrans     : btTransform;
        childCollisionShape,
        saveCollisionShape  : btCollisionShape;
        my_cb               : btCW_LocalInfoAdder;
  begin
    ///@todo : use AABB tree or other BVH acceleration structure!
    if collisionShape.isCompound then begin
      compoundShape := btCompoundShape(collisionShape);
      for i:=0 to compoundShape.getNumChildShapes-1 do begin
        childTrans          := compoundShape.getChildTransform(i);
        childCollisionShape := compoundShape.getChildShape(i);
        childWorldTrans     := colObjWorldTransform * childTrans;
        saveCollisionShape  := collisionObject.getCollisionShape;  // replace collision shape so that callback can determine the triangle
        collisionObject.internalSetTemporaryCollisionShape(childCollisionShape);
        my_cb := btCW_LocalInfoAdder.create(i, resultCallback);
        my_cb.m_closestHitFraction := resultCallback.m_closestHitFraction;
        objectQuerySingle(castShape, convexFromTrans,convexToTrans,collisionObject,childCollisionShape,childWorldTrans,my_cb,allowedPenetration);
        // restore
        collisionObject.internalSetTemporaryCollisionShape(saveCollisionShape);
      end;
    end;
  end;

begin
  if collisionShape.isConvex then begin
    _Convex;
  end else begin
    if collisionShape.isConcave then begin
      _Concave;
    end else begin
      _Compound;
    end;
  end;
end;

procedure btCollisionWorld.addCollisionObject(const collisionObject: btCollisionObject; const collisionFilterGroup: btCollisionFilterGroupSet; const collisionFilterMask: btCollisionFilterGroupSet);
var trans           : btTransform;
    minAabb,maxAabb : btVector3;
    typ             : TbtBroadphaseNativeTypes;
    bp              : btBroadphaseProxy;
begin
 //.
  btAssert(assigned(collisionObject));
  //check that the object isn't already added
  //btAssert(m_collisionObjects.findLinearSearch(collisionObject) = m_collisionObjects.size);
  m_collisionObjects.push_back(collisionObject);
  //calculate new AABB
  trans := collisionObject.getWorldTransformP^;
  collisionObject.getCollisionShape.getAabb(trans,minAabb,maxAabb);
  typ := collisionObject.getCollisionShape.getShapeType;
  bp:=getBroadphase.createProxy(minAabb,maxAabb,typ,collisionObject,collisionFilterGroup,collisionFilterMask,m_dispatcher1,nil);
  collisionObject.setBroadphaseHandle(bp);
end;

function btCollisionWorld.getCollisionObjectArray: btCollisionObjectArray;
begin
  result := m_collisionObjects;
end;

function btCollisionWorld.getCollisionObject(const i: integer): btCollisionObject;
begin
  result := m_collisionObjects[i]^;
end;

procedure btCollisionWorld.removeCollisionObject(const collisionObject: btCollisionObject);
var bp:btBroadphaseProxy;
begin
  //bool removeFromBroadphase = false;
  bp := collisionObject.getBroadphaseHandle;
  if assigned(bp) then begin
    // only clear the cached algorithms
    getBroadphase.getOverlappingPairCache.cleanProxyFromPairs(bp,m_dispatcher1);
    getBroadphase.destroyProxy(bp,m_dispatcher1);
    collisionObject.setBroadphaseHandle(nil);
  end;
  //swapremove
  m_collisionObjects.remove(collisionObject);
end;

procedure btCollisionWorld.performDiscreteCollisionDetection;
begin
  BT_PROFILE('performDiscreteCollisionDetection');
  updateAabbs;
  BT_PROFILE('calculateOverlappingPairs');
  m_broadphasePairCache.calculateOverlappingPairs(m_dispatcher1);

  BT_PROFILE('dispatchAllCollisionPairs');
  if assigned (m_dispatcher1) then begin
    m_dispatcher1.dispatchAllCollisionPairs(m_broadphasePairCache.getOverlappingPairCache,m_dispatchInfo,m_dispatcher1);
  end;
end;

function btCollisionWorld.getDispatchInfo: btDispatcherInfo;
begin
  result := m_dispatchInfo;
end;

function btCollisionWorld.getDispatchInfoP: PbtDispatcherInfo;
begin
  result := @m_dispatchInfo;
end;

function btCollisionWorld.getForceUpdateAllAabbs: boolean;
begin
  result := m_forceUpdateAllAabbs;
end;

procedure btCollisionWorld.setForceUpdateAllAabbs(const forceUpdateAllAabbs: boolean);
begin
  m_forceUpdateAllAabbs := forceUpdateAllAabbs;
end;

{ btCW_LocalRayResult }

procedure btCW_LocalRayResult.Init(const collisionObject: btCollisionObject; const localShapeInfo: PbtCW_LocalShapeInfo; const hitNormalLocal: btVector3; const hitFraction: btScalar);
begin
  m_collisionObject := collisionObject;
  m_localShapeInfo  := localShapeInfo;
  m_hitNormalLocal  := hitNormalLocal;
  m_hitFraction     := hitFraction;
end;

{ btCW_RayResultCallback }

constructor btCW_RayResultCallback.create;
begin
  m_closestHitFraction   := 1;
  m_collisionObject      := nil;
  m_collisionFilterGroup := [btcfgDefaultFilter];
  m_collisionFilterMask  := btcfgAllFilter;
  m_flags                := 0;
end;

function btCW_RayResultCallback.hasHit: boolean;
begin
  Result := m_collisionObject <> nil;
end;

function btCW_RayResultCallback.needsCollision(const proxy0: btBroadphaseProxy): boolean;
begin
  result := (proxy0.m_collisionFilterGroup * m_collisionFilterMask <> []) and (m_collisionFilterGroup * proxy0.m_collisionFilterMask <> []);
end;

{ btCW_ClosestRayResultCallback }

constructor btCW_ClosestRayResultCallback.Create(const rayFromWorld,rayToWorld: btVector3);
begin
  inherited Create;
  m_rayFromWorld := rayFromWorld;
  m_rayToWorld   := rayToWorld;
end;

function btCW_ClosestRayResultCallback.addSingleResult(const rayResult: btCW_LocalRayResult; const normalInWorldSpace: Boolean): btScalar;
begin
  //caller already does the filter on the m_closestHitFraction
  btAssert(rayResult.m_hitFraction <= m_closestHitFraction);

  m_closestHitFraction := rayResult.m_hitFraction;
  m_collisionObject    := rayResult.m_collisionObject;
  if normalInWorldSpace then begin
    m_hitNormalWorld := rayResult.m_hitNormalLocal;
  end else begin
    ///need to transform normal into worldspace
    m_hitNormalWorld := m_collisionObject.getWorldTransformP^.GetBasisV^*rayResult.m_hitNormalLocal;
  end;
  m_hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
  Result := rayResult.m_hitFraction;
end;

{ btCW_LocalConvexResult }

procedure btCW_LocalConvexResult.Init(const hitCollisionObject: btCollisionObject;const localShapeInfo: PbtCW_LocalShapeInfo; const hitNormalLocal,hitPointLocal: btVector3; const hitFraction: btScalar);
begin
  m_hitCollisionObject := hitCollisionObject;
  m_localShapeInfo     := localShapeInfo;
  m_hitNormalLocal     := hitNormalLocal;
  m_hitPointLocal      := hitPointLocal;
  m_hitFraction        := hitFraction;
end;

{ btCW_ConvexResultCallback }

constructor btCW_ConvexResultCallback.Create;
begin
  m_closestHitFraction    := 1;
  m_collisionFilterGroup  := [btcfgDefaultFilter];
  m_collisionFilterMask   := btcfgAllFilter;
end;

destructor btCW_ConvexResultCallback.Destroy;
begin

end;

function btCW_ConvexResultCallback.hasHit: boolean;
begin
  result := m_closestHitFraction < 1;
end;

function btCW_ConvexResultCallback.needsCollision(const proxy0: btBroadphaseProxy): boolean;
var collides:boolean;
begin
  collides := (proxy0.m_collisionFilterGroup * m_collisionFilterMask)<>[];
  collides := collides and ((m_collisionFilterGroup * proxy0.m_collisionFilterMask)<>[]);
  Result   := collides;
end;

{ btCW_ClosestConvexResultCallback }

constructor btCW_ClosestConvexResultCallback.Create(const convexFromWorld,convexToWorld: btVector3);
begin
 m_convexFromWorld    := convexFromWorld;
 m_convexToWorld      := convexToWorld;
 m_hitCollisionObject := nil;
end;

function btCW_ClosestConvexResultCallback.addSingleResult(const convexResult: btCW_LocalConvexResult; const normalInWorldSpace: boolean): btScalar;
begin
  //caller already does the filter on the m_closestHitFraction
  btAssert(convexResult.m_hitFraction <= m_closestHitFraction);
  m_closestHitFraction := convexResult.m_hitFraction;
  m_hitCollisionObject := convexResult.m_hitCollisionObject;
  if normalInWorldSpace then begin
    m_hitNormalWorld := convexResult.m_hitNormalLocal;
  end else begin
    ///need to transform normal into worldspace
    m_hitNormalWorld := m_hitCollisionObject.getWorldTransformP^.GetBasisV^*convexResult.m_hitNormalLocal;
  end;
  m_hitPointWorld := convexResult.m_hitPointLocal;
  result := convexResult.m_hitFraction;
end;

{ btCW_ContactResultCallback }

constructor btCW_ContactResultCallback.create;
begin
  inherited;
  init;
end;

procedure btCW_ContactResultCallback.Init;
begin
  m_collisionFilterGroup := [btcfgDefaultFilter];
  m_collisionFilterMask  := btcfgAllFilter;
end;

function btCW_ContactResultCallback.needsCollision(const proxy0: btBroadphaseProxy): boolean;
var collides:boolean;
begin
  collides := (proxy0.m_collisionFilterGroup * m_collisionFilterMask)<>[];
  collides := collides and ((m_collisionFilterGroup * proxy0.m_collisionFilterMask)<>[]);
  Result   := collides;
end;

//{ btAxisSweep3Internal_Edge }
//
//function btAxisSweep3Internal_Edge.IsMax: boolean;
//begin
//  result := (m_pos and 1)=1;
//end;
//
//{ btAxisSweep3Internal_Handle }
//
//procedure btAxisSweep3Internal_Handle.SetNextFree(const next: BP_FP_INT_TYPE);
//begin
//  m_minEdges[0] := next;
//end;
//
//function btAxisSweep3Internal_Handle.GetNextFree: BP_FP_INT_TYPE;
//begin
//  Result := m_minEdges[0];
//end;
//
//{ btAxisSweep3Internal }
//
//{$ifdef DEBUG_BROADPHASE}
//#include <stdio.h>
//
//template <typename BP_FP_INT_TYPE>
//void btAxisSweep3<BP_FP_INT_TYPE>::debugPrintAxis(int axis, bool checkCardinality)
//{
//	int numEdges = m_pHandles[0].m_maxEdges[axis];
//	printf("SAP Axis %d, numEdges=%d\n",axis,numEdges);
//
//	int i;
//	for (i=0;i<numEdges+1;i++)
//	{
//		Edge* pEdge = m_pEdges[axis] + i;
//		Handle* pHandlePrev = getHandle(pEdge->m_handle);
//		int handleIndex = pEdge->IsMax()? pHandlePrev->m_maxEdges[axis] : pHandlePrev->m_minEdges[axis];
//		char beginOrEnd;
//		beginOrEnd=pEdge->IsMax()?'E':'B';
//		printf("	[%c,h=%d,p=%x,i=%d]\n",beginOrEnd,pEdge->m_handle,pEdge->m_pos,handleIndex);
//	}
//
//	if (checkCardinality)
//		btAssert(numEdges == m_numHandles*2+1);
//}
//{$endif} //DEBUG_BROADPHASE
//
//function btAxisSweep3Internal.allocHandle: BP_FP_INT_TYPE;
//var handle : BP_FP_INT_TYPE;
//begin
//  //.
//  btAssert(m_firstFreeHandle<>0);
//  handle := m_firstFreeHandle;
//  m_firstFreeHandle := getHandle(handle).GetNextFree;
//  m_numHandles := m_numHandles+1;
//  Result := handle;
//end;
//
//procedure btAxisSweep3Internal.freeHandle(const handle: BP_FP_INT_TYPE);
//begin
//  btAssert((handle > 0) and (handle < m_maxHandles));
//  getHandle(handle).SetNextFree(m_firstFreeHandle);
//  m_firstFreeHandle := handle;
//  m_numHandles := m_numHandles-1;
//end;
//
//function btAxisSweep3Internal.testOverlap2D(const pHandleA, pHandleB: _AS3_Handle; const axis0, axis1: integer): boolean;
//begin
//
//end;
//
//// sorting a min edge downwards can only ever *add* overlaps
//procedure btAxisSweep3Internal.sortMinDown(const axis: integer; const edge: BP_FP_INT_TYPE; const dispatcher: btDispatcher; const updateOverlaps: Boolean);
//var pEdge,pPrev             : P_AS3_Edge;
//    pHandleEdge,pHandlePrev : _AS3_Handle;
//    axis1,axis2             : integer;
//    swap                    : _AS3_Edge;
//begin
//  pEdge       := m_pEdges[axis].A[edge];
//  pPrev       := m_pEdges[axis].DecPointerf(pEdge);
//  //pFOSTEst    := m_pEdges[axis].A[edge-1];
//  //btAssert(pPrev=pFOSTEst);
//  pHandleEdge := getHandle(pEdge^.m_handle);
//  while (pEdge^.m_pos < pPrev^.m_pos) do begin
//    pHandlePrev := getHandle(pPrev^.m_handle);
//    if pPrev^.IsMax then begin
//      // if previous edge is a maximum check the bounds and add an overlap if necessary
//      axis1 := (1 SHL axis ) and 3;
//      axis2 := (1 SHL axis1) and 3;
//      if updateOverlaps and testOverlap2D(pHandleEdge, pHandlePrev,axis1,axis2) then begin
//        m_pairCache.addOverlappingPair(pHandleEdge,pHandlePrev);
//        if assigned(m_userPairCallback) then begin
//          m_userPairCallback.addOverlappingPair(pHandleEdge,pHandlePrev);
//        end;
//        //AddOverlap(pEdge->m_handle, pPrev->m_handle);
//      end;
//      // update edge reference in other handle
//      inc(pHandlePrev.m_maxEdges[axis]);
//    end else begin
//      inc(pHandlePrev.m_minEdges[axis]);
//    end;
//    dec(pHandleEdge.m_minEdges[axis]);
//    // swap the edges
//    swap   := pEdge^;
//    pEdge^ := pPrev^;
//    pPrev^ := swap;
//    // decrement
//    dec(pEdge); // FOS Array must be aligned in Pointersize
//    dec(pPrev);
//    //m_pEdges[axis].DecPointer(pEdge);
//    //m_pEdges[axis].DecPointer(pPrev);
//  end;
//  {$ifdef DEBUG_BROADPHASE}
//  debugPrintAxis(axis);
//  {$endif} //DEBUG_BROADPHASE
//end;
//
//// sorting a min edge upwards can only ever *remove* overlaps
//procedure btAxisSweep3Internal.sortMinUp(const axis: integer; const edge: BP_FP_INT_TYPE; const dispatcher: btDispatcher; const updateOverlaps: Boolean);
//var pEdge,pNext             : P_AS3_Edge;
//    pHandleEdge,pHandleNext : _AS3_Handle;
//    axis1,axis2             : integer;
//    swap                    : _AS3_Edge;
//    handle0,handle1         : _AS3_Handle;
//begin
//  pEdge := m_pEdges[axis].A[edge];
//  pNext := pEdge;
//  m_pEdges[axis].IncPointer(pNext,1);
//  pHandleEdge := getHandle(pEdge^.m_handle);
//  while (pNext^.m_handle<>0) and (pEdge^.m_pos >= pNext^.m_pos) do begin
//    pHandleNext := getHandle(pNext^.m_handle);
//    if pNext^.IsMax then begin
//      handle0 := getHandle(pEdge^.m_handle);
//      handle1 := getHandle(pNext^.m_handle);
//      axis1 := (1  SHL axis)  and 3;
//      axis2 := (1  SHL axis1) and 3;
//      // if next edge is maximum remove any overlap between the two handles
//      if (updateOverlaps
//      {$ifdef USE_OVERLAP_TEST_ON_REMOVES}
//          and testOverlap2D(handle0,handle1,axis1,axis2)
//      {$endif} //USE_OVERLAP_TEST_ON_REMOVES
//          ) then begin
//             m_pairCache.removeOverlappingPair(handle0,handle1,dispatcher);
//             if assigned(m_userPairCallback) then begin
//               m_userPairCallback.removeOverlappingPair(handle0,handle1,dispatcher);
//             end;
//      end;
//      // update edge reference in other handle
//      dec(pHandleNext.m_maxEdges[axis]);
//    end else begin
//      dec(pHandleNext.m_minEdges[axis]);
//    end;
//    inc(pHandleEdge.m_minEdges[axis]);
//    // swap the edges
//    swap   := pEdge^;
//    pEdge^ := pNext^;
//    pNext^ := swap;
//    // increment
//    m_pEdges[axis].IncPointer(pEdge);
//    m_pEdges[axis].IncPointer(pNext);
//  end;
//end;
//
//// sorting a max edge downwards can only ever *remove* overlaps
//procedure btAxisSweep3Internal.sortMaxDown(const axis: integer; const edge: BP_FP_INT_TYPE; const dispatcher: btDispatcher; const updateOverlaps: Boolean);
//var pEdge,pPrev             : P_AS3_Edge;
//    pHandleEdge,pHandlePrev : _AS3_Handle;
//    axis1,axis2             : integer;
//    swap                    : _AS3_Edge;
//    handle0,handle1         : _AS3_Handle;
//begin
//  pEdge       := m_pEdges[axis].A[edge];
//  pPrev       := m_pEdges[axis].DecPointerf(pEdge);
//  pHandleEdge := getHandle(pEdge^.m_handle);
//  while (pEdge^.m_pos < pPrev^.m_pos) do begin
//    pHandlePrev := getHandle(pPrev^.m_handle);
//    if not pPrev^.IsMax then begin
//      // if previous edge was a minimum remove any overlap between the two handles
//      handle0 := getHandle(pEdge^.m_handle);
//      handle1 := getHandle(pPrev^.m_handle);
//      axis1   := (1 SHL axis)  and 3;
//      axis2   := (1 SHL axis1) and 3;
//      if (updateOverlaps
//      {$ifdef USE_OVERLAP_TEST_ON_REMOVES}
//              and testOverlap2D(handle0,handle1,axis1,axis2)
//      {$endif} //USE_OVERLAP_TEST_ON_REMOVES
//      ) then begin
//       //this is done during the overlappingpairarray iteration/narrowphase collision
//        m_pairCache.removeOverlappingPair(handle0,handle1,dispatcher);
//        if assigned(m_userPairCallback) then begin
//          m_userPairCallback.removeOverlappingPair(handle0,handle1,dispatcher);
//        end;
//      end;
//      // update edge reference in other handle
//      inc(pHandlePrev.m_minEdges[axis]);
//    end else begin
//      inc(pHandlePrev.m_maxEdges[axis]);
//    end;
//    dec(pHandleEdge.m_maxEdges[axis]);
//    // swap the edges
//    swap   := pEdge^;
//    pEdge^ := pPrev^;
//    pPrev^ := swap;
//    // decrement
//    dec(pEdge);
//    dec(pPrev);
//    //m_pEdges[axis].DecPointer(pEdge);
//    //m_pEdges[axis].DecPointer(pPrev);
//  end;
// {$ifdef DEBUG_BROADPHASE}
//   debugPrintAxis(axis);
// {$endif} //DEBUG_BROADPHASE
//end;
//
//// sorting a max edge upwards can only ever *add* overlaps
//procedure btAxisSweep3Internal.sortMaxUp(const axis: integer; const edge: BP_FP_INT_TYPE; const dispatcher: btDispatcher; const updateOverlaps: Boolean);
//var pEdge,pNext             : P_AS3_Edge;
//    testEdge                : _AS3_Edge;
//    pHandleEdge,pHandleNext : _AS3_Handle;
//    axis1,axis2             : integer;
//    swap                    : _AS3_Edge;
//    handle0,handle1         : _AS3_Handle;
//begin
//  pEdge       := m_pEdges[axis].A[edge];
//  testEdge    := pEdge^;
//  pNext       := m_pEdges[axis].IncPointerf(pEdge);
//  testEdge    := pNext^;
//  pHandleEdge := getHandle(pEdge^.m_handle);
//
//  while (pNext^.m_handle<>0) and (pEdge^.m_pos >= pNext^.m_pos) do begin
//    pHandleNext := getHandle(pNext^.m_handle);
//    axis1       := (1 SHL axis)  and 3;
//    axis2       := (1 SHL axis1) and 3;
//    if not pNext^.IsMax then begin
//      // if next edge is a minimum check the bounds and add an overlap if necessary
//      if updateOverlaps and testOverlap2D(pHandleEdge, pHandleNext,axis1,axis2) then begin
//        handle0 := getHandle(pEdge^.m_handle);
//        handle1 := getHandle(pNext^.m_handle);
//        m_pairCache.addOverlappingPair(handle0,handle1);
//        if assigned(m_userPairCallback) then begin
//          m_userPairCallback.addOverlappingPair(handle0,handle1);
//        end;
//      end;
//      // update edge reference in other handle
//      dec(pHandleNext.m_minEdges[axis]);
//    end else begin
//      dec(pHandleNext.m_maxEdges[axis]);
//    end;
//    inc(pHandleEdge.m_maxEdges[axis]);
//    // swap the edges
//    swap   := pEdge^;
//    pEdge^ := pNext^;
//    pNext^ := swap;
//    // increment
//    //m_pEdges[axis].IncPointerf(pEdge);
//    //m_pEdges[axis].IncPointerf(pNext);
//    inc(pEdge);
//    inc(pNext);
//  end;
//end;
//
//procedure btAxisSweep3Internal.FOS_Assign(var dst: _AS3_Edge; const src: _AS3_Edge);
//begin
//  dst.m_handle := src.m_handle;
//  dst.m_pos    := src.m_pos;
//end;
//
//procedure btAxisSweep3Internal._DumpHandles;
//var   i: Integer;
//begin
//  for i:=0 to 3 do begin
//    write('m_pEdges Axis 0 :',i,': handle=',m_pEdges[0].A[i]^.m_handle,' pos=',m_pEdges[0].A[i]^.m_pos,' ');
//    write('m_pEdges Axis 1 :',i,': handle=',m_pEdges[1].A[i]^.m_handle,' pos=',m_pEdges[1].A[i]^.m_pos,' ');
//    writeln('m_pEdges Axis 2 :',i,': handle=',m_pEdges[2].A[i]^.m_handle,' pos=',m_pEdges[2].A[i]^.m_pos);
//  end;
//end;
//
//constructor btAxisSweep3Internal.create(const worldAabbMin, worldAabbMax: btVector3; const handleMask, handleSentinel: BP_FP_INT_TYPE; const usermaxHandles: BP_FP_INT_TYPE;  const pairCache: btOverlappingPairCache; const disableRaycastAccelerator: Boolean);
//var maxHandles,maxInt : BP_FP_INT_TYPE;
//    aabbSize          : btVector3;
//    i                 : Integer;
//    j                 : Integer;
//    axis              : integer;
//begin
//  m_bpHandleMask       := handleMask;
//  m_handleSentinel     := handleSentinel;
//  m_pairCache          := pairCache;
//  m_userPairCallback   := nil;
//  m_ownsPairCache      := false;
//  m_invalidPair        := 0;
//  m_raycastAccelerator := nil;
//
//  maxHandles := userMaxHandles+1;//need to add one sentinel handle
//  if not assigned(m_pairCache) then begin
//    m_pairCache     := btHashedOverlappingPairCache.Create;
//    m_ownsPairCache := true;
//  end;
//  if not disableRaycastAccelerator then begin
//    m_nullPairCache      := btNullPairCache.Create;
//    m_raycastAccelerator := btDbvtBroadphase.Create (m_nullPairCache);//m_pairCache);
//    m_raycastAccelerator.m_deferedcollide := true;//don't add/remove pairs
//  end;
//
//  //btAssert(bounds.HasVolume());
//
//  // init bounds
//  m_worldAabbMin := worldAabbMin;
//  m_worldAabbMax := worldAabbMax;
//  aabbSize       := m_worldAabbMax - m_worldAabbMin;
//  maxInt         := m_handleSentinel;
//  m_quantize     := btVector3.InitSameS(maxInt) / aabbSize;
//
//  // allocate handles buffer, using btAlignedAlloc, and put all handles on free list
//  m_pHandles.Init(sizeof(_AS3_Handle));
//  m_pHandles.Setlength(maxHandles);
//  for i := 0 to  maxHandles-1 do begin
//    m_pHandles.A[i]^ := _AS3_Handle.Create;
//  end;
//
//  m_maxHandles := maxHandles;
//  m_numHandles := 0;
//  // handle 0 is reserved as the null index, and is also used as the sentinel
//  m_firstFreeHandle := 1;
//  for i := m_firstFreeHandle to  maxHandles-1 do begin
//    m_pHandles.A[i]^.SetNextFree(i+1);
//  end;
//  m_pHandles.A[maxHandles-1]^.SetNextFree(0);
//  // allocate edge buffers
//  for i := 0 to 2 do begin
//    m_pEdges[i].Init(sizeof(_AS3_Edge));
//    m_pEdges[i].Setlength(maxHandles*2);
//    for j:=0 to maxHandles-1 do begin
//     m_pEdges[i].A[j]^ := _AS3_Edge.Create;
//    end;
//  end;
//  // removed overlap management
//  // make boundary sentinels
//
//  m_pHandles.A[0]^.m_clientObject :=nil;
//
//  for axis := 0 to 2 do begin
//    m_pHandles.A[0]^.m_minEdges[axis] := 0;
//    m_pHandles.A[0]^.m_maxEdges[axis] := 1;
//
//    m_pEdges[axis].A[0]^.m_pos    := 0;
//    m_pEdges[axis].A[0]^.m_handle := 0;
//    m_pEdges[axis].A[1]^.m_pos    := m_handleSentinel;
//    m_pEdges[axis].A[1]^.m_handle := 0;
//    {$ifdef DEBUG_BROADPHASE}
//    debugPrintAxis(axis);
//    {$endif} //DEBUG_BROADPHASE
//  end;
//end;
//
//destructor btAxisSweep3Internal.destroy;
//var i,j:integer;
//begin
//  if assigned(m_raycastAccelerator) then begin
//    m_nullPairCache.Free;
//    m_nullPairCache:=nil;
//    m_raycastAccelerator.Free;
//    m_raycastAccelerator:=nil;
//  end;
//  for i := 2 downto 0 do begin
//    for j:=m_pEdges[i].size-1 downto 0 do begin
//      m_pEdges[i].A[j]^.free;
//      m_pEdges[i].A[j]^:=nil;
//    end;
//  end;
//  for i := m_pHandles.Size-1 downto 0 do begin
//    m_pHandles.A[i]^.free;
//    m_pHandles.A[i]^ := nil;
//  end;
//  if m_ownsPairCache then begin
//    m_pairCache.Free;
//  end;
//  m_pEdges[0].Finalize;
//  m_pEdges[1].Finalize;
//  m_pEdges[2].Finalize;
//  m_pHandles.Finalize;
//  inherited destroy;
//end;
//
//function btAxisSweep3Internal.getNumHandles: BP_FP_INT_TYPE;
//begin
//  result := m_numHandles;
//end;
//
//procedure btAxisSweep3Internal.calculateOverlappingPairs(const dispatcher: btDispatcher);
//var overlappingPairArray : PbtBroadphasePairArray;
//    i                    : integer;
//    previousPair         : btBroadphasePair;
//    pair                 : PbtBroadphasePair;
//    isDuplicate          : Boolean;
//    needsRemoval         : Boolean;
//    hasOverlap           : Boolean;
//begin
//  if m_pairCache.hasDeferredRemoval then begin
//        overlappingPairArray := m_pairCache.getOverlappingPairArray;
//        //perform a sort, to find duplicates and to sort 'invalid' pairs to the end
//        overlappingPairArray^.quickSort(@btBroadphasePairSortPredicate);
//        overlappingPairArray^.resize(overlappingPairArray^.size - m_invalidPair);
//        m_invalidPair            := 0;
//        previousPair.Init;
//        //previousPair.m_pProxy0   := nil;
//        //previousPair.m_pProxy1   := nil;
//        //previousPair.m_algorithm := nil;
//        for i :=0 to overlappingPairArray^.size-1 do begin
//          pair := overlappingPairArray^.A[i];
//          isDuplicate  := (pair^ = previousPair);
//          previousPair := pair^;
//          needsRemoval := false;
//          if not isDuplicate then begin
//            ///important to use an AABB test that is consistent with the broadphase
//            hasOverlap := testAabbOverlap(pair^.m_pProxy0,pair^.m_pProxy1);
//            if hasOverlap then begin
//              needsRemoval := false;//callback->processOverlap(pair);
//            end else begin
//              needsRemoval := true;
//            end;
//          end else begin
//            needsRemoval := true; //remove duplicate
//            btAssert(not assigned(pair^.m_algorithm));  //should have no algorithm
//          end;
//          if needsRemoval then begin
//            m_pairCache.cleanOverlappingPair(pair,dispatcher);
//            pair^.m_pProxy0 := nil;
//            pair^.m_pProxy1 := nil;
//            inc(m_invalidPair);
//            dec(gOverlappingPairs);
//          end;
//        end;
//
//  ///if you don't like to skip the invalid pairs in the array, execute following code:
//  {$define CLEAN_INVALID_PAIRS}
//  {$ifdef CLEAN_INVALID_PAIRS}
//        //perform a sort, to sort 'invalid' pairs to the end
//        overlappingPairArray^.quickSort(@btBroadphasePairSortPredicate);
//        overlappingPairArray^.resize(overlappingPairArray^.size - m_invalidPair);
//        m_invalidPair := 0;
//  {$endif}//CLEAN_INVALID_PAIRS
//        //printf("overlappingPairArray.size()=%d\n",overlappingPairArray.size());
//  end;
//end;
//
//function btAxisSweep3Internal.addHandle(const aabbMin, aabbMax: btVector3; const pOwner: pointer; const collisionFilterGroup, collisionFilterMask: btCollisionFilterGroupSet; const dispatcher: btDispatcher; const multiSapProxy: Pointer): BP_FP_INT_TYPE;
//var min,max      : array [0..2] of BP_FP_INT_TYPE;
//    handle,limit : BP_FP_INT_TYPE;
//    pHandle      : _AS3_Handle;
//    axis         : integer;
//    e1,e2        : _AS3_Edge;
//begin
//  // quantize the bounds
//  quantize(min, aabbMin, 0);
//  quantize(max, aabbMax, 1);
//  // allocate a handle
//  handle  := allocHandle;
//  pHandle := getHandle(handle);
//  pHandle.m_uniqueId := handle;
//  //pHandle->m_pOverlaps = 0;
//  pHandle.m_clientObject         := pOwner;
//  pHandle.m_collisionFilterGroup := collisionFilterGroup;
//  pHandle.m_collisionFilterMask  := collisionFilterMask;
//  pHandle.m_multiSapParentProxy  := multiSapProxy;
//
//  // compute current limit of edge arrays
//  limit := m_numHandles * 2;
//  //   insert new edges just inside the max boundary edge
//  for axis := 0 to 2 do begin
//    m_pHandles.A[0]^.m_maxEdges[axis]     += 2;
//    FOS_Assign(m_pEdges[axis].A[limit + 1]^ , m_pEdges[axis].A[limit - 1]^);
//    m_pEdges[axis].A[limit - 1]^.m_pos    := min[axis];
//    m_pEdges[axis].A[limit - 1]^.m_handle := handle;
//    m_pEdges[axis].A[limit]^.m_pos        := max[axis];
//    m_pEdges[axis].A[limit]^.m_handle     := handle;
//    pHandle.m_minEdges[axis]              := limit-1;
//    pHandle.m_maxEdges[axis]              := limit;
//  end;
//  // now sort the new edges to their correct position
//  sortMinDown(0, pHandle.m_minEdges[0], dispatcher,false);
//  sortMaxDown(0, pHandle.m_maxEdges[0], dispatcher,false);
//  sortMinDown(1, pHandle.m_minEdges[1], dispatcher,false);
//  sortMaxDown(1, pHandle.m_maxEdges[1], dispatcher,false);
//  sortMinDown(2, pHandle.m_minEdges[2], dispatcher,true);
//  sortMaxDown(2, pHandle.m_maxEdges[2], dispatcher,true);
//  Result := handle;
//end;
//
//procedure btAxisSweep3Internal.removeHandle(const handle: BP_FP_INT_TYPE; const dispatcher: btDispatcher);
//var pHandle    : _AS3_Handle;
//    limit,axis : integer;
//    pEdges     : _AS3_EdgeA;
//    max,i      : BP_FP_INT_TYPE;
//begin
//  pHandle := getHandle(handle);
//  //explicitly remove the pairs containing the proxy
//  //we could do it also in the sortMinUp (passing true)
//  ///@todo: compare performance
//  if not (m_pairCache.hasDeferredRemoval) then begin
//    m_pairCache.removeOverlappingPairsContainingProxy(pHandle,dispatcher);
//  end;
//
//  // compute current limit of edge arrays
//  limit := m_numHandles * 2;
//  for axis := 0 to 2 do begin
//    m_pHandles.A[0]^.m_maxEdges[axis] -= 2;
//  end;
//
//  // remove the edges by sorting them up to the end of the list
//  for axis := 0 to 2 do begin
//    pEdges               := m_pEdges[axis];
//    max                  := pHandle.m_maxEdges[axis];
//    pEdges.A[max]^.m_pos := m_handleSentinel;
//    sortMaxUp(axis,max,dispatcher,false);
//    i  := pHandle.m_minEdges[axis];
//    pEdges.A[i]^.m_pos := m_handleSentinel;
//    sortMinUp(axis,i,dispatcher,false);
//    pEdges.A[limit-1]^.m_handle := 0;
//    pEdges.A[limit-1]^.m_pos    := m_handleSentinel;
//    {$ifdef DEBUG_BROADPHASE}
//      debugPrintAxis(axis,false);
//    {$endif} //DEBUG_BROADPHASE
//  end;
//  freeHandle(handle); // free the handle
//end;
//
//procedure btAxisSweep3Internal.updateHandle(const handle: BP_FP_INT_TYPE; const aabbMin, aabbMax: btVector3; const dispatcher: btDispatcher);
//var pHandle        : _AS3_Handle;
//    min,max        : array [0..2] of BP_FP_INT_TYPE;
//    axis,dmin,dmax : integer;
//    emin,emax      : BP_FP_INT_TYPE;
//begin
//  pHandle := getHandle(handle);
//  // quantize the new bounds
//  quantize(min, aabbMin, 0);
//  quantize(max, aabbMax, 1);
//  // update changed edges
//  for axis := 0 to 2 do begin
//    emin := pHandle.m_minEdges[axis];
//    emax := pHandle.m_maxEdges[axis];
//    dmin := min[axis] - m_pEdges[axis].A[emin]^.m_pos;
//    dmax := max[axis] - m_pEdges[axis].A[emax]^.m_pos;
//    m_pEdges[axis].A[emin]^.m_pos := min[axis];
//    m_pEdges[axis].A[emax]^.m_pos := max[axis];
//    // expand (only adds overlaps)
//    if dmin < 0 then begin
//      sortMinDown(axis, emin,dispatcher,true);
//    end;
//    if dmax > 0 then begin
//      sortMaxUp(axis, emax,dispatcher,true);
//    end;
//    // shrink (only removes overlaps)
//    if dmin > 0 then begin
//      sortMinUp(axis, emin,dispatcher,true);
//    end;
//    if dmax < 0 then begin
//      sortMaxDown(axis, emax,dispatcher,true);
//    end;
//  {$ifdef DEBUG_BROADPHASE}
//    debugPrintAxis(axis);
//  {$endif} //DEBUG_BROADPHASE
//  end;
//end;
//
//function btAxisSweep3Internal.getHandle(const index: BP_FP_INT_TYPE): _AS3_Handle;
//begin
//  Result := m_pHandles.A[index]^;
//end;
//
//procedure btAxisSweep3Internal.resetPool(const dispatcher: btDispatcher);
//var i : integer;
//begin
//  if m_numHandles = 0 then begin
//    m_firstFreeHandle := 1;
//    for  i := m_firstFreeHandle to m_maxHandles-1 do begin
//      m_pHandles.A[i]^.SetNextFree(i+1);
//    end;
//    m_pHandles.A[m_maxHandles-1]^.SetNextFree(0);
//  end;
//end;
//
//
//function btAxisSweep3Internal.createProxy(const aabbMin, aabbMax: btVector3; const shapeType: TbtBroadphaseNativeTypes; const userPtr: Pointer; const collisionFilterGroup,  collisionFilterMask: btCollisionFilterGroupSet; const dispatcher: btDispatcher; const multiSapProxy: Pointer): btBroadphaseProxy;
//var handleId : BP_FP_INT_TYPE;
//    handle   : _AS3_Handle;
//    rayProxy : btBroadphaseProxy;
//begin
//  handleId := addHandle(aabbMin,aabbMax, userPtr,collisionFilterGroup,collisionFilterMask,dispatcher,multiSapProxy);
//  handle   := getHandle(handleId);
//  if assigned(m_raycastAccelerator) then begin
//    rayProxy := m_raycastAccelerator.createProxy(aabbMin,aabbMax,shapeType,userPtr,collisionFilterGroup,collisionFilterMask,dispatcher,nil);
//    handle.m_dbvtProxy := rayProxy;
//  end;
//  Result := handle;
//end;
//
//
//procedure btAxisSweep3Internal.destroyProxy(const proxy: btBroadphaseProxy; const dispatcher: btDispatcher);
//var handle : _AS3_Handle;
//begin
//  handle:=_AS3_Handle(proxy);
//  if assigned(m_raycastAccelerator) then begin
//    m_raycastAccelerator.destroyProxy(handle.m_dbvtProxy,dispatcher);
//  end;
//  removeHandle(handle.m_uniqueId, dispatcher);
//end;
//
//procedure btAxisSweep3Internal.setAabb(const proxy: btBroadphaseProxy; const aabbMin, aabbMax: btVector3; const dispatcher: btDispatcher);
//var handle : _AS3_Handle;
//begin
//  handle:=_AS3_Handle(proxy);
//  handle.m_aabbMin := aabbMin;
//  handle.m_aabbMax := aabbMax;
//  updateHandle(handle.m_uniqueId, aabbMin, aabbMax,dispatcher);
//  if assigned(m_raycastAccelerator) then begin
//    m_raycastAccelerator.setAabb(handle.m_dbvtProxy,aabbMin,aabbMax,dispatcher);
//  end;
//end;
//
//procedure btAxisSweep3Internal.getAabb(const proxy: btBroadphaseProxy; var aabbMin, aabbMax: btVector3);
//var handle : _AS3_Handle;
//begin
//  handle  := _AS3_Handle(proxy);
//  aabbMin := Handle.m_aabbMin;
//  aabbMax := Handle.m_aabbMax;
//end;
//
//procedure btAxisSweep3Internal.rayTest(const rayFrom, rayTo: btVector3; const rayCallback: btBroadphaseRayCallback; const aabbMin, aabbMax: btVector3);
//var axis : BP_FP_INT_TYPE;
//       i : integer;
//begin
//  if assigned(m_raycastAccelerator) then begin
//    m_raycastAccelerator.rayTest(rayFrom,rayTo,rayCallback,aabbMin,aabbMax);
//  end else begin
//    //choose axis?
//    axis := 0;
//    //for each proxy
//    for i := 1 to m_numHandles*2 do begin
//      if m_pEdges[axis].A[i]^.IsMax then begin
//        rayCallback.process(getHandle(m_pEdges[axis].A[i]^.m_handle));
//      end;
//    end;
//  end;
//end;
//
//procedure btAxisSweep3Internal.aabbTest(const aabbMin, aabbMax: btVector3; const callback: btBroadphaseAabbCallback);
//var axis : BP_FP_INT_TYPE;
//       i : integer;
//    handle : _AS3_Handle;
//begin
//  if assigned(m_raycastAccelerator) then begin
//    m_raycastAccelerator.aabbTest(aabbMin,aabbMax,callback);
//  end else begin
//    axis := 0;
//    //for each proxy
//    for i := 1 to m_numHandles*2 do begin
//      if m_pEdges[axis].A[i]^.IsMax then begin
//        handle := getHandle(m_pEdges[axis].A[i]^.m_handle);
//        if TestAabbAgainstAabb2(aabbMin,aabbMax,handle.m_aabbMin,handle.m_aabbMax) then begin
//          callback.process(handle);
//        end;
//      end;
//    end;
//  end;
//end;
//
//procedure btAxisSweep3Internal.quantize(const out: PBP_FP_INT_TYPE; const point: btVector3; const isMax: integer);
//var v : btVector3;
//begin
//  //#ifdef OLD_CLAMPING_METHOD
//  //      ///problem with this clamping method is that the floating point during quantization might still go outside the range [(0|isMax) .. (m_handleSentinel&m_bpHandleMask]|isMax]
//  //      ///see http://code.google.com/p/bullet/issues/detail?id=87
//  //      btVector3 clampedPoint(point);
//  //      clampedPoint.setMax(m_worldAabbMin);
//  //      clampedPoint.setMin(m_worldAabbMax);
//  //      btVector3 v = (clampedPoint - m_worldAabbMin) * m_quantize;
//  //      out[0] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getX() & m_bpHandleMask) | isMax);
//  //      out[1] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getY() & m_bpHandleMask) | isMax);
//  //      out[2] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getZ() & m_bpHandleMask) | isMax);
//  //#else
//  //out[0]=(v[0]<=0)?(BP_FP_INT_TYPE)isMax:(v[0]>=m_handleSentinel)?(BP_FP_INT_TYPE)((m_handleSentinel&m_bpHandleMask)|isMax):(BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[0]&m_bpHandleMask)|isMax);
//  //out[1]=(v[1]<=0)?(BP_FP_INT_TYPE)isMax:(v[1]>=m_handleSentinel)?(BP_FP_INT_TYPE)((m_handleSentinel&m_bpHandleMask)|isMax):(BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[1]&m_bpHandleMask)|isMax);
//  //out[2]=(v[2]<=0)?(BP_FP_INT_TYPE)isMax:(v[2]>=m_handleSentinel)?(BP_FP_INT_TYPE)((m_handleSentinel&m_bpHandleMask)|isMax):(BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[2]&m_bpHandleMask)|isMax);
//  //#endif //OLD_CLAMPING_METHOD
//  v := (point - m_worldAabbMin) * m_quantize;
//  if v[0]<=0 then begin
//    out[0] := isMax;
//  end else begin
//    if v[0]>=m_handleSentinel then begin
//      out[0] := (m_handleSentinel and m_bpHandleMask) or isMax;
//    end else begin
//      out[0] := (trunc(v[0]) and m_bpHandleMask) or isMax;
//    end;
//  end;
//  if v[1]<=0 then begin
//    out[1] := isMax;
//  end else begin
//    if v[1]>=m_handleSentinel then begin
//      out[1] := (m_handleSentinel and m_bpHandleMask) or isMax;
//    end else begin
//      out[1] := (trunc(v[1]) and m_bpHandleMask) or isMax;
//    end;
//  end;
//  if v[2]<=0 then begin
//    out[2] := isMax;
//  end else begin
//    if v[2]>=m_handleSentinel then begin
//      out[2] := (m_handleSentinel and m_bpHandleMask) or isMax;
//    end else begin
//      out[2] := (trunc(v[2]) and m_bpHandleMask) or isMax;
//    end;
//  end;
//end;
//
//procedure btAxisSweep3Internal.unQuantize(const proxy: btBroadphaseProxy; var aabbMin, aabbMax: btVector3);
//var phandle   : _AS3_Handle;
//    vecInMin,
//    vecInMax  : array [0..2] of Word;
//begin
//  pHandle := _AS3_Handle(proxy);
//  vecInMin[0] := m_pEdges[0].A[pHandle.m_minEdges[0]]^.m_pos ;
//  vecInMax[0] := m_pEdges[0].A[pHandle.m_maxEdges[0]]^.m_pos +1 ;
//  vecInMin[1] := m_pEdges[1].A[pHandle.m_minEdges[1]]^.m_pos ;
//  vecInMax[1] := m_pEdges[1].A[pHandle.m_maxEdges[1]]^.m_pos +1 ;
//  vecInMin[2] := m_pEdges[2].A[pHandle.m_minEdges[2]]^.m_pos ;
//  vecInMax[2] := m_pEdges[2].A[pHandle.m_maxEdges[2]]^.m_pos +1 ;
//  aabbMin.init(vecInMin[0] / m_quantize.getX,vecInMin[1] / m_quantize.getY, vecInMin[2] / m_quantize.getZ);
//  aabbMin += m_worldAabbMin;
//  aabbMax.init(vecInMax[0] / m_quantize.getX,vecInMax[1] / m_quantize.getY, vecInMax[2] / m_quantize.getZ);
//  aabbMax += m_worldAabbMin;
//end;
//
//function btAxisSweep3Internal.testAabbOverlap(const proxy0, proxy1: btBroadphaseProxy): boolean;
//var pHandleA,pHandleB : _AS3_Handle;
//    axis              : integer;
//begin
//  pHandleA := _AS3_Handle(proxy0);
//  pHandleB := _AS3_Handle(proxy1);
//  //optimization 1: check the array index (memory address), instead of the m_pos
//  for axis := 0 to 2 do begin
//    if (pHandleA.m_maxEdges[axis] < pHandleB.m_minEdges[axis]) or  (pHandleB.m_maxEdges[axis] < pHandleA.m_minEdges[axis]) then begin
//      exit(false);
//    end;
//  end;
//  Result := true;
//end;
//
//function btAxisSweep3Internal.getOverlappingPairCache: btOverlappingPairCache;
//begin
//  Result := m_pairCache;
//end;
//
//procedure btAxisSweep3Internal.setOverlappingPairUserCallback(const pairCallback: btOverlappingPairCallback);
//begin
//  m_userPairCallback := pairCallback;
//end;
//
//function btAxisSweep3Internal.getOverlappingPairUserCallback: btOverlappingPairCallback;
//begin
// Result := m_userPairCallback;
//end;
//
//procedure btAxisSweep3Internal.getBroadphaseAabb(var aabbMin, aabbMax: btVector3);
//begin
//  aabbMin := m_worldAabbMin;
//  aabbMax := m_worldAabbMax;
//end;
//
//procedure btAxisSweep3Internal.printStats;
//begin
/////*              printf("btAxisSweep3.h\n");
////                printf("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
////                printf("aabbMin=%f,%f,%f,aabbMax=%f,%f,%f\n",m_worldAabbMin.getX(),m_worldAabbMin.getY(),m_worldAabbMin.getZ(),
////                        m_worldAabbMax.getX(),m_worldAabbMax.getY(),m_worldAabbMax.getZ());
////                        */
//end;
//
//{ btAxisSweep3 }
//
//constructor btAxisSweep3.create(const worldAabbMin, worldAabbMax: btVector3; maxHandles: word; const pairCache: btOverlappingPairCache; const disableRaycastAccelerator: Boolean);
//begin
//  inherited create(worldAabbMin,worldAabbMax,$fffe,$ffff,maxHandles,pairCache,disableRaycastAccelerator);
//  // 1 handle is reserved as sentinel
//  btAssert((maxHandles > 1) and  (maxHandles < 32767));
//end;
//
//{ bt32BitAxisSweep3 }
//
//constructor bt32BitAxisSweep3.create(const worldAabbMin, worldAabbMax: btVector3; maxHandles: cardinal; const pairCache: btOverlappingPairCache; const disableRaycastAccelerator: Boolean);
//begin
//  inherited create(worldAabbMin,worldAabbMax,$fffffffe,$7fffffff,maxHandles,pairCache,disableRaycastAccelerator);
//  // 1 handle is reserved as sentinel
//  btAssert((maxHandles > 1) and  (maxHandles < 2147483647));
//end;
//
end.