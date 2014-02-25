unit btDynamics;
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
// btContactSolverInfo.h
//. btSolverBody.h
// btSolverConstraint.h
// btTypedConstraint.h
// btConstraintSolver.h
// btActionInterface.h
// btSequentialImpulseConstraintSolver.h
// btSequentialImpulseConstraintSolver.cpp
// btRigidBody.h
// btRigidBody.cpp
// btDynamicsWorld.h
// btUnionFind.h
// btUnionFind.cpp
// btSimulationIslandManager.h
// btSimulationIslandManager.cpp
// btMotionState.h
// btDefaultMotionState.h
// btDiscreteDynamicsWorld.h
// S btDiscreteDynamicsWorld.cpp


//#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
//#include "BulletDynamics/Dynamics/btContinuousDynamicsWorld.h"
//
//#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
//#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
//#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
//#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
//#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
//#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"
//#include "BulletDynamics/ConstraintSolver/btUniversalConstraint.h"
//#include "BulletDynamics/ConstraintSolver/btHinge2Constraint.h"
//
//
//
/////Vehicle simulation, with wheel contact simulated by raycasts
//#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

interface

{$i fos_bullet.inc}
{$modeswitch nestedprocvars}

uses
  Sysutils,FOS_AlignedArray,btLinearMath,btBroadphase,btNarrowphase,btDispatch,btCollisionShapes;


var gDeactivationTime    : btScalar=2;
    gDisableDeactivation : boolean;
    gUniqueID            : cardinal=0;
    gNumSplitImpulseRecoveries :integer = 0;
    gNumClampedCcdMotions : integer = 0; ///internal debugging variable. this value shouldn't be too high

type

  btSolverMode=(
    SOLVER_RANDMIZE_ORDER,
    SOLVER_FRICTION_SEPARATE,
    SOLVER_USE_WARMSTARTING,
    SOLVER_USE_FRICTION_WARMSTARTING,
    SOLVER_USE_2_FRICTION_DIRECTIONS,
    SOLVER_ENABLE_FRICTION_DIRECTION_CACHING,
    SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION,
    SOLVER_CACHE_FRIENDLY,
    SOLVER_SIMD,      //enabled for Windows, the solver innerloop is branchless SIMD, 40% faster than FPU/scalar version
    SOLVER_CUDA        //will be open sourced during Game Developers Conference 2009. Much faster.
  );
  btSolverModeSet=set of btSolverMode;

  //btContactSolverInfoData = class
  //  m_tau,
  //  m_damping,//global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
  //  m_friction,
  //  m_timeStep,
  //  m_restitution        : btScalar;
  //  m_numIterations      : integer;
  //  m_maxErrorReduction,
  //  m_sor,
  //  m_erp,               //used as Baumgarte factor
  //  m_erp2,              //used in Split Impulse
  //  m_globalCfm          : btScalar;//constraint force mixing
  //  m_splitImpulse       : Boolean;
  //  m_splitImpulsePenetrationThreshold : btScalar;
  //  m_linearSlop                       : btScalar;
  //  m_warmstartingFactor               : btScalar;
  //  m_solverMode                       : btSolverModeSet;
  //  m_restingContactRestitutionThreshold : integer;
  //  m_minimumSolverBatchSize             : integer;
  //end;

  { btContactSolverInfo }

  btContactSolverInfo = class
  public
    m_tau,
    m_damping,//global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
    m_friction,
    m_timeStep,
    m_restitution        : btScalar;
    m_numIterations      : integer;
    m_maxErrorReduction,
    m_sor,
    m_erp,               //used as Baumgarte factor
    m_erp2,              //used in Split Impulse
    m_globalCfm          : btScalar;//constraint force mixing
    m_splitImpulse       : Boolean;
    m_splitImpulsePenetrationThreshold : btScalar;
    m_linearSlop                       : btScalar;
    m_warmstartingFactor               : btScalar;
    m_solverMode                       : btSolverModeSet;
    m_restingContactRestitutionThreshold : integer;
    m_minimumSolverBatchSize             : integer;
    constructor create;
  end;


  //#ifdef USE_SIMD
  //
  //struct        btSimdScalar
  //{
  //      SIMD_FORCE_INLINE       btSimdScalar()
  //      {
  //
  //      }
  //
  //      SIMD_FORCE_INLINE       btSimdScalar(float      fl)
  //      :m_vec128 (_mm_set1_ps(fl))
  //      {
  //      }
  //
  //      SIMD_FORCE_INLINE       btSimdScalar(__m128 v128)
  //              :m_vec128(v128)
  //      {
  //      }
  //      union
  //      {
  //              __m128          m_vec128;
  //              float           m_floats[4];
  //              int                     m_ints[4];
  //              btScalar        m_unusedPadding;
  //      };
  //      SIMD_FORCE_INLINE       __m128  get128()
  //      {
  //              return m_vec128;
  //      }
  //
  //      SIMD_FORCE_INLINE       const __m128    get128() const
  //      {
  //              return m_vec128;
  //      }
  //
  //      SIMD_FORCE_INLINE       void    set128(__m128 v128)
  //      {
  //              m_vec128 = v128;
  //      }
  //
  //      SIMD_FORCE_INLINE       operator       __m128()
  //      {
  //              return m_vec128;
  //      }
  //      SIMD_FORCE_INLINE       operator const __m128() const
  //      {
  //              return m_vec128;
  //      }
  //
  //      SIMD_FORCE_INLINE       operator float() const
  //      {
  //              return m_floats[0];
  //      }
  //
  //};
  //
  /////@brief Return the elementwise product of two btSimdScalar
  //SIMD_FORCE_INLINE btSimdScalar
  //operator*(const btSimdScalar& v1, const btSimdScalar& v2)
  //{
  //      return btSimdScalar(_mm_mul_ps(v1.get128(),v2.get128()));
  //}
  //
  /////@brief Return the elementwise product of two btSimdScalar
  //SIMD_FORCE_INLINE btSimdScalar
  //operator+(const btSimdScalar& v1, const btSimdScalar& v2)
  //{
  //      return btSimdScalar(_mm_add_ps(v1.get128(),v2.get128()));
  //}
  //
  //
  //#else
  //#define btSimdScalar btScalar
  //#endif


  btTypedConstraintType=(
        POINT2POINT_CONSTRAINT_TYPE=ord(MAX_CONTACT_MANIFOLD_TYPE)+1,
        HINGE_CONSTRAINT_TYPE,
        CONETWIST_CONSTRAINT_TYPE,
        D6_CONSTRAINT_TYPE,
        SLIDER_CONSTRAINT_TYPE,
        CONTACT_CONSTRAINT_TYPE
  );
  btConstraintParams=(
        BT_CONSTRAINT_ERP=1,
        BT_CONSTRAINT_STOP_ERP,
        BT_CONSTRAINT_CFM,
        BT_CONSTRAINT_STOP_CFM
  );
  {$ifdef 1}
    {$define btAssertConstrParams:=btAssert}
  {$else}
    {$define btAssertConstrParams:=//}
  {$endif}

  btSimdScalar      = btScalar;
  btMotionState     = class;
  btTypedConstraint = class;

  btTypedConstraintArray = specialize FOS_GenericAlignedArray<btTypedConstraint>;
  PbtTypedConstraintArray = ^btTypedConstraintArray;

  btRigidBodyFlags = ( BT_DISABLE_WORLD_GRAVITY );
  btRigidBodyFlagSet = set of btRigidBodyFlags;


  ///The btRigidBodyConstructionInfo structure provides information to create a rigid body. Setting mass to zero creates a fixed (non-dynamic) rigid body.
  ///For dynamic objects, you can use the collision shape to approximate the local inertia tensor, otherwise use the zero vector (default argument)
  ///You can use the motion state to synchronize the world transform between physics and graphics objects.
  ///And if the motion state is provided, the rigid body will initialize its initial world transform from the motion state,
  ///m_startWorldTransform is only used when you don't provide a motion state.

  { btRigidBodyConstructionInfo }

  btRigidBodyConstructionInfo = object
    m_mass                    : btScalar;
    ///When a motionState is provided, the rigid body will initialize its world transform from the motion state
    ///In this case, m_startWorldTransform is ignored.
    m_motionState             : btMotionState;
    m_startWorldTransform     : btTransform;
    m_collisionShape          : btCollisionShape;
    m_localInertia            : btVector3;
    m_linearDamping           : btScalar;
    m_angularDamping          : btScalar;
    ///best simulation results when friction is non-zero
    m_friction                : btScalar;
    ///best simulation results using zero restitution.
    m_restitution             : btScalar;
    m_linearSleepingThreshold : btScalar;
    m_angularSleepingThreshold: btScalar;
    //Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
    //Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
    m_additionalDamping       : boolean;
    m_additionalDampingFactor : btScalar;
    m_additionalLinearDampingThresholdSqr  : btScalar;
    m_additionalAngularDampingThresholdSqr : btScalar;
    m_additionalAngularDampingFactor       : btScalar;
    procedure Init(const mass : btScalar; const  motionState : btMotionState ; const collisionShape:btCollisionShape ; const localInertia : btVector3); //default localInertia = cbtNullVector
  end;



  ///The btRigidBody is the main class for rigid body objects. It is derived from btCollisionObject, so it keeps a pointer to a btCollisionShape.
  ///It is recommended for performance and memory use to share btCollisionShape objects whenever possible.
  ///There are 3 types of rigid bodies:
  ///- A) Dynamic rigid bodies, with positive mass. Motion is controlled by rigid body dynamics.
  ///- B) Fixed objects with zero mass. They are not moving (basically collision objects)
  ///- C) Kinematic objects, which are objects without mass, but the user can move them. There is on-way interaction, and Bullet calculates a velocity based on the timestep and previous and current world transform.
  ///Bullet automatically deactivates dynamic rigid bodies, when the velocity is below a threshold for a given time.
  ///Deactivated (sleeping) rigid bodies don't take any processing time, except a minor broadphase collision detection impact (to allow active objects to activate/wake up sleeping objects)

  btRigidBody = class(btCollisionObject)
    m_invInertiaTensorWorld : btMatrix3x3;
    m_linearVelocity        : btVector3;
    m_angularVelocity       : btVector3;
    m_inverseMass           : btScalar;
    m_linearFactor          : btVector3;
    m_gravity               : btVector3;
    m_gravity_acceleration  : btVector3;
    m_invInertiaLocal       : btVector3;
    m_totalForce            : btVector3;
    m_totalTorque           : btVector3;
    m_linearDamping         : btScalar;
    m_angularDamping        : btScalar;
    m_additionalDamping     : boolean;
    m_additionalDampingFactor : btScalar;
    m_additionalLinearDampingThresholdSqr  : btScalar;
    m_additionalAngularDampingThresholdSqr : btScalar;
    m_additionalAngularDampingFactor       : btScalar;
    m_linearSleepingThreshold              : btScalar;
    m_angularSleepingThreshold             : btScalar;
    //m_optionalMotionState allows to automatic synchronize the world transform for active objects
    m_optionalMotionState                  : btMotionState;
    //keep track of typed constraints referencing this rigid body
    m_constraintRefs                       : btTypedConstraintArray;
    m_rigidbodyFlags                       : btRigidBodyFlagSet;
    m_debugBodyId                          : integer;
  protected
    //    ATTRIBUTE_ALIGNED64(btVector3           m_deltaLinearVelocity);
    m_deltaLinearVelocity                  : btVector3;
    m_deltaAngularVelocity                 : btVector3;
    m_angularFactor                        : btVector3;
    m_invMass                              : btVector3;
    m_turnVelocity                         : btVector3;
    m_pushVelocity                         : btVector3;
    //for experimental overriding of friction/contact solver func
    m_contactSolverType                    : integer;
    m_frictionSolverType                   : integer;
  public
    ///btRigidBody constructor using construction info
    constructor create(const  constructionInfo : btRigidBodyConstructionInfo);overload;
    ///btRigidBody constructor for backwards compatibility.
    ///To specify friction (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo)
    constructor create(const mass : btScalar ; const motionState : btMotionState ; const collisionShape : btCollisionShape ;  const localInertia : btVector3);overload; // default : localInertia = btNullvector
    destructor  destroy;override;
  protected
  ///setupRigidBody is only used internally by the constructor
    procedure setupRigidBody(const constructionInfo:btRigidBodyConstructionInfo);
  public
    procedure proceedToTransform(const newTrans : btTransform);
    ///to keep collision detection and dynamics separate we don't store a rigidbody pointer
    ///but a rigidbody is derived from btCollisionObject, so we can safely perform an upcast
    class function  upcast(const colObj : btCollisionObject):btRigidBody;
    /// continuous collision detection needs prediction
    procedure predictIntegratedTransform(const step : btScalar ; out predictedTransform : btTransform);
    procedure saveKinematicState(const step : btScalar);
    procedure applyGravity;
    procedure setGravity(const acceleration : btVector3);
    function  getGravityP : PbtVector3;FOS_INLINE;
    procedure setDamping(const lin_damping,ang_damping : btScalar);
    function  getLinearDamping  : btScalar;
    function  getAngularDamping : btScalar;
    function  getLinearSleepingThreshold : btScalar;
    function  getAngularSleepingThreshold : btScalar;
    procedure applyDamping(const timeStep : btScalar);
    function  getCollisionShape : btCollisionShape;FOS_INLINE;
    procedure setMassProps(const mass : btScalar ; const inertia : btVector3);
    function  getLinearFactorP: PbtVector3 ;FOS_INLINE;
    procedure setLinearFactor(const linearFactor: btVector3);
    function  getInvMass     : btScalar;
    function  getInvInertiaTensorWorldP : PbtMatrix3x3;
    procedure integrateVelocities(const step : btScalar);
    procedure setCenterOfMassTransform(const xform : btTransform);
    procedure applyCentralForce(const force : btVector3);
    function  getTotalForceP:PbtVector3;FOS_INLINE;
    function  getTotalTorqueP:PbtVector3;FOS_INLINE;
    function  getInvInertiaDiagLocalP:PbtVector3;FOS_INLINE;
    procedure setInvInertiaDiagLocal(const diagInvInertia:btVector3);
    procedure setSleepingThresholds(const linear,angular : btScalar);
    procedure applyTorque(const torque:btVector3);
    procedure applyForce(const  force, rel_pos : btVector3);
    procedure applyCentralImpulse(const impulse : btVector3);
    procedure applyTorqueImpulse(const  torque  : btVector3);
    procedure applyImpulse(const impulse, rel_pos : btVector3);
    procedure clearForces;
    procedure updateInertiaTensor;
    function  getCenterOfMassPositionP:PbtVector3;
    function  getOrientation:btQuaternion;
    function  getCenterOfMassTransformP:PbtTransform;
    function  getLinearVelocityP:PbtVector3;FOS_INLINE;
    function  getAngularVelocityP:PbtVector3;FOS_INLINE;
    procedure setLinearVelocity(const lin_vel:btVector3);FOS_INLINE;
    procedure setAngularVelocity(const ang_vel:btVector3);FOS_INLINE;
    function  getVelocityInLocalPoint(const rel_pos:btVector3):btVector3;
    procedure translate(const v:btVector3);
    procedure getAabb(var aabbMin,aabbMax : btVector3);
    function  computeImpulseDenominator(const pos    : btVector3; const normal:btVector3):btScalar;FOS_INLINE;
    function  computeAngularImpulseDenominator(const axis : btVector3):btScalar;FOS_INLINE;
    procedure updateDeactivation(const timeStep : btScalar);//inline;
    function  wantsSleeping:boolean;//inline;
    function  getBroadphaseProxy:btBroadphaseProxy;
    procedure setNewBroadphaseProxy(const broadphaseProxy : btBroadphaseProxy);
    //btMotionState allows to automatic synchronize the world transform for active objects
    function  getMotionState:btMotionState;FOS_INLINE;
    procedure setMotionState(const motionState:btMotionState);
    procedure setAngularFactor(const angFac : btVector3);overload;
    procedure setAngularFactor(const angFac : btScalar);overload;
    function  getAngularFactorP:PbtVector3;FOS_INLINE;
    //is this rigidbody added to a btCollisionWorld/btDynamicsWorld/btBroadphase?
    function  isInWorld:boolean;
    function  checkCollideWithOverride(const co: btCollisionObject): boolean; override;
    procedure addConstraintRef(const  c:btTypedConstraint);
    procedure removeConstraintRef(const c:btTypedConstraint);
    function  getConstraintRef(const index:integer):btTypedConstraint;
    function  getNumConstraintRefs:integer;
    procedure setFlags(const flags:btRigidBodyFlagSet);
    function  getFlags:btRigidBodyFlagSet;
    function  getDeltaLinearVelocityP:PbtVector3;
    function  getDeltaAngularVelocityP:PbtVector3;
    function  getPushVelocityP:PbtVector3;
    function  getTurnVelocityP:PbtVector3;
    ////////////////////////////////////////////////
    ///some internal methods, don't use them
    function  internalGetDeltaLinearVelocityP:PbtVector3;
    function  internalGetDeltaAngularVelocityP:PbtVector3;
    function  internalGetAngularFactorP:PbtVector3;
    function  internalGetInvMassP:PbtVector3;
    function  internalGetPushVelocityP:PbtVector3;
    function  internalGetTurnVelocityP:PbtVector3;
    procedure internalGetVelocityInLocalPointObsolete(const  rel_pos : btVector3 ; var velocity : btVector3 );FOS_INLINE;
    procedure internalGetAngularVelocity(var angVel:btVector3);FOS_INLINE;
    //Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
    procedure internalApplyImpulse(const linearComponent, angularComponent : btVector3 ; const impulseMagnitude : btScalar);FOS_INLINE;
    procedure internalApplyPushImpulse(const linearComponent, angularComponent : btVector3 ; const impulseMagnitude : btScalar);FOS_INLINE;
    procedure internalWritebackVelocity;
    procedure internalWritebackVelocity(const timeStep : btScalar);
  end;

  btRigidBodyArray = specialize FOS_GenericAlignedArray<btRigidBody>;

  ///1D constraint along a normal axis between bodyA and bodyB. It can be combined to solve contact and friction constraints.
//  ATTRIBUTE_ALIGNED64 (struct)  btSolverConstraint

  btRSolverConstraint=record
    m_relpos1CrossNormal,
    m_contactNormal,
    m_relpos2CrossNormal,
        //btVector3             m_contactNormal2;//usually m_contactNormal2 == -m_contactNormal
    m_angularComponentA,
    m_angularComponentB   : btVector3;
    m_appliedPushImpulse  : btSimdScalar;
    m_appliedImpulse      : btSimdScalar;
    m_friction            : btScalar;
    m_jacDiagABInv        : btScalar;
    m_numConsecutiveRowsPerKernel : integer;
    m_unusedPadding0              : integer;
    m_frictionIndex               : integer;
    m_unusedPadding1              : integer;
    m_solverBodyA                 : btRigidBody;
    m_solverBodyB                 : btRigidBody;//  FOS : was -> union { btRigidBody* m_solverBodyB; int m_companionIdA; };
    m_originalContactPoint        : Pointer;
    {$IFNDEF  x86_64}
    m_unusedPadding4              : integer;
    {$ENDIF}
    m_rhs,
    m_cfm,
    m_lowerLimit,
    m_upperLimit,
    m_rhsPenetration              : btScalar;
  end;
  PbtRSolverConstraint = ^btRSolverConstraint;

  btSolverConstraintType = ( BT_SOLVER_CONTACT_1D = 0, BT_SOLVER_FRICTION_1D );
  btConstraintArray = specialize FOS_GenericAlignedArray<btRSolverConstraint>;

  { btConstraintSolver }

  btConstraintSolver = class
  public
    procedure prepareSolve (const numBodies,numManifolds:integer);virtual;
    ///solve a group of constraints
    function solveGroup   (const bodies : btCollisionObjectArray ; const numBodies : integer ; const manifold : btManifoldArray;const startManifold ,numManifolds : integer; const constraints : btTypedConstraintArray;
                           const startConstraint,numConstraints : integer; const info : btContactSolverInfo ; const  debugDrawer : btIDebugDraw ; const stackAlloc : btStackAllocator ; const  dispatcher:btDispatcher):btScalar;virtual;abstract;
    procedure allSolved   (const info : btContactSolverInfo; const debugDrawer : btIDebugDraw ; const stackAlloc : btStackAllocator);virtual;
    /////clear internal cached data and reset random seed
    procedure reset       ;virtual;abstract;
  end;

  btTC_ConstraintInfo1=record
   m_numConstraintRows,nub : integer;
  end;
  btTC_ConstraintInfo2=record
    // integrator parameters: frames per second (1/stepsize), default error
    // reduction parameter (0..1).
    fps,erp : btScalar;
    // for the first and second body, pointers to two (linear and angular)
    // n*3 jacobian sub matrices, stored by rows. these matrices will have
    // been initialized to 0 on entry. if the second body is zero then the
    // J2xx pointers may be 0.
    m_J1linearAxis,m_J1angularAxis,m_J2linearAxis,m_J2angularAxis : PbtScalar;
    // elements to jump from one row to the next in J's
    rowskip : integer;
    // right hand sides of the equation J*v = c + cfm * lambda. cfm is the
    // "constraint force mixing" vector. c is set to zero on entry, cfm is
    // set to a constant value (typically very small or zero) value on entry.
    m_constraintError,cfm : PbtScalar;
    // lo and hi limits for variables (set to -/+ infinity on entry).
    m_lowerLimit,m_upperLimit : PbtScalar;
    // findex vector for variables. see the LCP solver interface for a
    // description of what this does. this is set to -1 on entry.
    // note that the returned indexes are relative to the first index of
    // the constraint.
    findex : PInteger;
    // number of solver iterations
    m_numIterations : integer;
    //damping of the velocity
    m_damping : btScalar;
  end;
  PbtTC_ConstraintInfo1 = ^btTC_ConstraintInfo1;
  PbtTC_ConstraintInfo2 = ^btTC_ConstraintInfo2;


  ///TypedConstraint is the baseclass for Bullet constraints and vehicles

  { btTypedConstraint }
  PbtTypedConstraint = ^btTypedConstraint;
  btTypedConstraint =class(btTypedObject)
  private
    m_userConstraintType : integer;
    m_userConstraintId   : PtrInt;
    m_needsFeedback      : Boolean;
        //btTypedConstraint&      operator=(btTypedConstraint&    other)
        //{
        //        btAssert(0);
        //        (void) other;
        //        return *this;
        //}
  protected
    m_rbA : btRigidBody;
    m_rbB : btRigidBody;
    m_appliedImpulse : btScalar;
    m_dbgDrawSize    : btScalar;
    ///internal method used by the constraint solver, don't use them directly
    function getMotorFactor(const pos, lowLim, uppLim, vel, timeFact : btScalar):btScalar;
    class function getFixedBody : btRigidBody;
  public
    constructor create(const typ : btTypedConstraintType ;const rbA: btRigidBody);
    constructor create(const typ : btTypedConstraintType ;const rbA,rbB: btRigidBody);
    destructor  destroy;override;
    ///internal method used by the constraint solver, don't use them directly
    procedure   buildJacobian;virtual;
    ///internal method used by the constraint solver, don't use them directly
    procedure setupSolverConstraint(const ca : btConstraintArray; const solverBodyA,solverBodyB : integer; const timeStep : btScalar);virtual;
    ///internal method used by the constraint solver, don't use them directly
    procedure getInfo1 (var info : btTC_ConstraintInfo1);virtual;abstract;
    procedure getInfo2 (var info : btTC_ConstraintInfo2);virtual;abstract;
    ///internal method used by the constraint solver, don't use them directly
    procedure internalSetAppliedImpulse(const appliedImpulse:btScalar);
    ///internal method used by the constraint solver, don't use them directly
    function   internalGetAppliedImpulse:btScalar;FOS_INLINE;
    ///internal method used by the constraint solver, don't use them directly
    procedure  solveConstraintObsolete(const bodyA,bodyB : btRigidBody; const timeStep:btScalar);virtual;
    function   getRigidBodyA:btRigidBody;
    function   getRigidBodyB:btRigidBody;
    function   getUserConstraintType:integer;
    procedure  setUserConstraintType(const userConstraintType:Integer);
    procedure  setUserConstraintId(const uid:integer);
    function   getUserConstraintId:integer;
    procedure  setUserConstraintPtr(const ptr:pointer);
    function   getUserConstraintPtr:Pointer;
    function   getUid:Integer;
    function   needsFeedback:Boolean;
    ///enableFeedback will allow to read the applied linear and angular impulse
    ///use getAppliedImpulse, getAppliedLinearImpulse and getAppliedAngularImpulse to read feedback information
    procedure enableFeedback(const nFeedback:Boolean);
    ///getAppliedImpulse is an estimated total applied impulse.
    ///This feedback could be used to determine breaking constraints or playing sounds.
    function  getAppliedImpulse:btScalar;
    function  getConstraintType:btTypedConstraintType;
    procedure setDbgDrawSize(const dbgDrawSize:btScalar);
    function  getDbgDrawSize:btScalar;
    ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5).
    ///If no axis is provided, it uses the default axis for this constraint.
    procedure setParam(const num : integer ; const value : btScalar ; const axis : integer = -1);virtual;abstract;
    ///return the local value of parameter
    function getParam(const num : integer ;const axis : integer = -1):btScalar;virtual;abstract;
  end;

  ///Basic interface to allow actions such as vehicles and characters to be updated inside a btDynamicsWorld

  { btActionInterface }

  btActionInterface = class
  protected
    class function getFixedBody : btRigidBody;
  public
    procedure updateAction(const collisionWorld : btCollisionWorld ; const deltaTimeStep : btScalar);virtual;abstract;
    procedure debugDraw(const debugDrawer : btIDebugDraw);virtual;abstract;
  end;
  btActionInterfaceArray = specialize FOS_GenericAlignedArray<btActionInterface>;


  ///The btMotionState interface class allows the dynamics world to synchronize and interpolate the updated world transforms with graphics
  ///For optimizations, potentially only moving objects get synchronized (using setWorldPosition/setWorldOrientation)
  btMotionState=class
    procedure getWorldTransform(var worldTrans:btTransform);virtual;abstract;
    //Bullet only calls the update of worldtransform for active objects
    procedure setWorldTransform(const worldTrans:btTransform);virtual;abstract;
  end;

  ///The btDefaultMotionState provides a common implementation to synchronize world transforms with offsets.

  { btDefaultMotionState }

  btDefaultMotionState=class(btMotionState)
    m_graphicsWorldTrans  : btTransform;
    m_centerOfMassOffset  : btTransform;
    m_startWorldTrans     : btTransform;
    m_userPointer         : Pointer;
    constructor create    ;overload;
    constructor create    (const startTrans : btTransform);overload;
    constructor create    (const startTrans,centerOfMassOffset : btTransform);overload;
    ///synchronizes world transform from user to physics
    procedure getWorldTransform(var centerOfMassWorldTrans:btTransform);override;
    ///synchronizes world transform from physics to user
    ///Bullet only calls the update of worldtransform for active objects
    procedure setWorldTransform(const centerOfMassWorldTrans : btTransform);override;
  end;


  btDynamicsWorld = class;
  /// Type for the callback for each tick
  btInternalTickCallback = procedure (const world:btDynamicsWorld;const  timeStep : btScalar);
  btDynamicsWorldType    = (BT_SIMPLE_DYNAMICS_WORLD,BT_DISCRETE_DYNAMICS_WORLD,BT_CONTINUOUS_DYNAMICS_WORLD);

  ///The btDynamicsWorld is the interface class for several dynamics implementation, basic, discrete, parallel, and continuous etc.

  { btDynamicsWorld }

  btDynamicsWorld = class(btCollisionWorld)
  protected
    m_internalTickCallback    : btInternalTickCallback;
    m_internalPreTickCallback : btInternalTickCallback;
    m_worldUserInfo           : Pointer;
    m_solverInfo              : btContactSolverInfo;
  public
    constructor create(const dispatcher : btDispatcher;const broadphase : btBroadphaseInterface; const collisionConfiguration :btCollisionConfiguration);
    destructor  destroy;override;
    ///stepSimulation proceeds the simulation over 'timeStep', units in preferably in seconds.
    ///By default, Bullet will subdivide the timestep in constant substeps of each 'fixedTimeStep'.
    ///in order to keep the simulation real-time, the maximum number of substeps can be clamped to 'maxSubSteps'.
    ///You can disable subdividing the timestep/substepping by passing maxSubSteps=0 as second argument to stepSimulation, but in that case you have to keep the timeStep constant.
    function  stepSimulation(const timeStep : btScalar;  maxSubSteps:Integer=1 ; fixedTimeStep:btScalar = 1/60):integer ;virtual;abstract;
//    procedure debugDrawWorld;override
    procedure addConstraint(const constraint    : btTypedConstraint; const disableCollisionsBetweenLinkedBodies : Boolean=false);virtual;
    procedure removeConstraint(const constraint : btTypedConstraint);virtual;
    procedure addAction(const  action : btActionInterface);virtual;abstract;
    procedure removeAction(const action : btActionInterface);virtual;abstract;
    //once a rigidbody is added to the dynamics world, it will get this gravity assigned
    //existing rigidbodies in the world get gravity assigned too, during this method
    procedure setGravity(const gravity:btVector3);virtual;abstract;
    function  getGravity:btVector3;virtual;abstract;
    procedure synchronizeMotionStates;virtual;abstract;
    procedure addRigidBody(const body :btRigidBody);virtual;abstract;
    procedure removeRigidBody(const body:btRigidBody);virtual;abstract;
    procedure setConstraintSolver(const solver : btConstraintSolver);virtual;abstract;
    function  getConstraintSolver : btConstraintSolver;virtual;abstract;
    function  getNumConstraints:integer;
    function  getConstraint(const index:integer):btTypedConstraint;virtual;
    function  getWorldType : btDynamicsWorldType ;virtual;abstract;
    procedure clearForces;virtual;abstract;
    /// Set the callback for when an internal tick (simulation substep) happens, optional user info
    procedure setInternalTickCallback(const cb : btInternalTickCallback ; const worldUserInfo:Pointer=nil;const isPreTick:boolean=false);
    procedure setWorldUserInfo(const worldUserInfo:Pointer);
    function  getWorldUserInfo:pointer;
    function  getSolverInfo:btContactSolverInfo;
    /////obsolete, use addAction instead.
    //virtual void    addVehicle(btActionInterface* vehicle) {(void)vehicle;}
    /////obsolete, use removeAction instead
    //virtual void    removeVehicle(btActionInterface* vehicle) {(void)vehicle;}
    /////obsolete, use addAction instead.
    //virtual void    addCharacter(btActionInterface* character) {(void)character;}
    /////obsolete, use removeAction instead
    //virtual void    removeCharacter(btActionInterface* character) {(void)character;}
  end;

  btTC_ConstraintInfo1Array = specialize FOS_GenericAlignedArray<btTC_ConstraintInfo1>;

  ///The btSequentialImpulseConstraintSolver is a fast SIMD implementation of the Projected Gauss Seidel (iterative LCP) method.

  { btSequentialImpulseConstraintSolver }

  btSequentialImpulseConstraintSolver = class(btConstraintSolver)
  protected
    //FOS Store transient Parameters
    ms_bodies: btCollisionObjectArray;
    ms_numBodies: LongInt;
    ms_manifoldPtr    : btManifoldArray;
    ms_startManifold  : LongInt;
    ms_numManifolds   : LongInt;
    ms_constraints    : btTypedConstraintArray;
    ms_numConstraints : LongInt;
    ms_startConstraint: LongInt;
    ms_info           : btContactSolverInfo;
    ms_debugDrawer    : btIDebugDraw;
    ms_stackAlloc     : btStackAllocator;
    ms_disp           : btDispatcher;
    // FOS Store tranisent Parameters END
    m_btSeed2 : cardinal;
    ///m_btSeed2 is used for re-arranging the constraint rows. improves convergence/quality of friction
    m_tmpSolverContactConstraintPool         : btConstraintArray;
    m_tmpSolverNonContactConstraintPool      : btConstraintArray;
    m_tmpSolverContactFrictionConstraintPool : btConstraintArray;
    m_orderTmpConstraintPool                 : btFOSAlignedIntegers;
    m_orderFrictionConstraintPool            : btFOSAlignedIntegers;
    m_tmpConstraintSizesPool                 : btTC_ConstraintInfo1Array;
    procedure  setupFrictionConstraint( const solverConstraint : PbtRSolverConstraint ; const normalAxis : btVector3 ; const solverBodyA,solverBodyIdB : btRigidBody ;
                                        const cp : PbtOManifoldPoint; const rel_pos1 , rel_pos2 : btVector3 ; const colObj0, colObj1 : btCollisionObject; const relaxation : btScalar ;
                                        const desiredVelocity:btScalar = 0 ; const cfmSlip : btScalar=0);

    function   addFrictionConstraint(const normalAxis : btVector3;const solverBodyA,solverBodyB : btRigidBody ;const frictionIndex : Integer ;const cp : PbtOManifoldPoint ;const rel_pos1,rel_pos2 : btVector3 ;
                                     const colObj0,colObj1 : btCollisionObject ; const relaxation : btScalar ; const desiredVelocity : btScalar=0 ; const cfmSlip : btScalar =0):PbtRSolverConstraint;

    procedure  setupContactConstraint(const solverConstraint : PbtRSolverConstraint ; const  colObj0, colObj1 : btCollisionObject ; const cp : btOManifoldPoint;
                                                            const infoGlobal : btContactSolverInfo ; out  vel : btVector3 ; out rel_vel,relaxation : btScalar ;
                                                            out   rel_pos1, rel_pos2 : btVector3);
    procedure  setFrictionConstraintImpulse( const  solverConstraint : PbtRSolverConstraint ; const rb0, rb1 : btRigidBody ; const  cp : btOManifoldPoint ; const infoGlobal : btContactSolverInfo);
    function   restitutionCurve(const rel_vel, restitution : btScalar) : btScalar;
    procedure  convertContact(const  manifold : btPersistentManifold ; const infoGlobal : btContactSolverInfo);
    procedure  resolveSplitPenetrationSIMD(const body1,body2 : btRigidBody ; const contactConstraint : PbtRSolverConstraint);
    procedure  resolveSplitPenetrationImpulseCacheFriendly(const body1,body2 : btRigidBody ; const contactConstraint : PbtRSolverConstraint);
    //internal method
//    function   getOrInitSolverBody(const body : btCollisionObject) : integer;
    procedure  resolveSingleConstraintRowGeneric             (const body1,body2 : btRigidBody ; const contactConstraint : PbtRSolverConstraint);
    procedure  resolveSingleConstraintRowGenericSIMD         (const body1,body2 : btRigidBody ; const contactConstraint : PbtRSolverConstraint);
    procedure  resolveSingleConstraintRowLowerLimit          (const body1,body2 : btRigidBody ; const contactConstraint : PbtRSolverConstraint);
    procedure  resolveSingleConstraintRowLowerLimitSIMD      (const body1,body2 : btRigidBody ; const contactConstraint : PbtRSolverConstraint);
    class      function getFixedBody:btRigidBody             ;
    procedure  solveGroupCacheFriendlySplitImpulseIterations ;
    procedure  solveGroupCacheFriendlyFinish                 ;
    procedure  solveSingleIteration                          (const iteration:integer);
    procedure  solveGroupCacheFriendlySetup                  ;
    procedure  solveGroupCacheFriendlyIterations             ;
  public
    constructor create;
    destructor  destroy;override;
    function    solveGroup                                   (const bodies : btCollisionObjectArray ; const numBodies : integer ;const manifoldPtr : btManifoldArray; const startManifold,numManifolds : integer ; const constraints : btTypedConstraintArray ; const startConstraint,numConstraints : integer;
                                                              const info   : btContactSolverInfo  ; const debugDrawer : btIDebugDraw ; const stackAlloc : btStackAllocator;const disp:btDispatcher):btScalar;override;

    procedure reset;override;
    function  btRand2:cardinal;
    function  btRandInt2(const n:integer):integer;
    procedure setRandSeed(const seed:cardinal);
    function  getRandSeed:Cardinal;
  end;

  {$ifndef BT_PREFER_SIMD}
    btSequentialImpulseConstraintSolverPrefered=btSequentialImpulseConstraintSolver;
  {$endif}

  btElement = record
    m_id, m_sz : integer;
  end;
  PbtElement = ^btElement;

  btElementArray = specialize FOS_GenericAlignedArray<btElement>;
  ///UnionFind calculates connected subsets
  // Implements weighted Quick Union with path compression
  // optimization: could use short ints instead of ints (halving memory, would limit the number of rigid bodies to 64k, sounds reasonable)

  { btUnionFind }

  btUnionFind = class
  private
    m_elements : btElementArray;
  public
    constructor create;
    destructor  destroy;override;
    //this is a special operation, destroying the content of btUnionFind.
    //it sorts the elements, based on island id, in order to make it easy to iterate over islands
    procedure   sortIslands     ;
    procedure   reset           (const N:integer);
    function    getNumElements  :integer;FOS_INLINE;
    function    isRoot          (const x:integer):boolean;FOS_INLINE;
    function    getElement      (const index:integer):PbtElement;
    procedure   allocate        (const N : integer);
    function    find            (const p,q : integer):integer;
    procedure   unite           (const p,q : integer);
    function    find            (x : integer):integer;
  end;

  ///SimulationIslandManager creates and handles simulation islands, using btUnionFind

  btSM_IslandCallback = procedure (const bodies : btCollisionObjectArray ; const manifolds : btManifoldArray ; const startIdx , numManifolds:integer ; const islandId : integer) is nested; // ProcessIsland

  { btSimulationIslandManager }

  btSimulationIslandManager = class
    m_unionFind      : btUnionFind;
    m_islandmanifold : btManifoldArray;
    m_islandBodies   : btCollisionObjectArray;
    m_splitIslands   : Boolean;
  public
    constructor create;
    destructor  destroy;override;
    procedure   initUnionFind(const n:integer);
    function    getUnionFind:btUnionFind;
    procedure   updateActivationState(const colWorld : btCollisionWorld ; const dispatcher : btDispatcher);virtual;
    procedure   storeIslandActivationState(const  world:btCollisionWorld);virtual;
    procedure   findUnions(const dispatcher : btDispatcher ; const colWorld : btCollisionWorld);
    procedure   buildAndProcessIslands(const dispatcher : btDispatcher ; const collisionWorld : btCollisionWorld ; const callback : btSM_IslandCallback);
    procedure   buildIslands(const dispatcher : btDispatcher ; const colWorld : btCollisionWorld);
    function    getSplitIslands:Boolean;
    procedure   setSplitIslands(const doSplitIslands:boolean);
  end;

  ///btDiscreteDynamicsWorld provides discrete rigid body simulation

  { btDiscreteDynamicsWorld }

  btDiscreteDynamicsWorld = class(btDynamicsWorld)
  protected
    m_constraintSolver     : btConstraintSolver;
    m_islandManager        : btSimulationIslandManager;
    m_constraints          : btTypedConstraintArray;      // FOS: predefined constraints ?
    m_nonStaticRigidBodies : btRigidBodyArray;
    m_gravity              : btVector3;
    //for variable timesteps
    m_localTime            : btScalar;
    m_ownsIslandManager    : Boolean;
    m_ownsConstraintSolver : Boolean;
    m_synchronizeAllMotionStates : boolean;
    m_actions              : btActionInterfaceArray;
    m_profileTimings       : integer;
    procedure              predictUnconstraintMotion(const timeStep : btScalar);overload;
    procedure              integrateTransforms(const timeStep : btScalar);overload;
    procedure              calculateSimulationIslands;overload;
    procedure              solveConstraints(const solverInfo : btContactSolverInfo); virtual;
    procedure              updateActivationState(const timeStep:btScalar);
    procedure              updateActions(const timeStep:btScalar);
    procedure              startProfiling(const timeStep:btScalar);
    procedure              internalSingleStepSimulation(const timeStep:btScalar);virtual;
    procedure              saveKinematicState(const timeStep:btScalar);virtual;
  public
    ///this btDiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those
    constructor create(const dispatcher : btDispatcher ; const pairCache : btBroadphaseInterface ; const constraintSolver : btConstraintSolver ; const collisionConfiguration : btCollisionConfiguration);
    destructor  destroy;override;
    ///if maxSubSteps > 0, it will interpolate motion between fixedTimeStep's
    function    stepSimulation(const timeStep : btScalar ;  maxSubSteps:integer=1 ; fixedTimeStep:btScalar=1/60):integer;override;
    procedure   synchronizeMotionStates;override;
    ///this can be useful to synchronize a single rigid body -> graphics object
    procedure   synchronizeSingleMotionState(const  body:btRigidBody);
    procedure   addConstraint(const constraint : btTypedConstraint ; const disableCollisionsBetweenLinkedBodies:Boolean=false);override;
    procedure   removeConstraint(const constraint : btTypedConstraint);override;
    procedure   addAction   (const ai:btActionInterface);override;
    procedure   removeAction(const ai:btActionInterface);override;
    procedure setGravity(const gravity : btVector3);override;
    function  getGravity:btVector3;override;
    procedure addCollisionObject(const collisionObject : btCollisionObject ; const collisionFilterGroup : btCollisionFilterGroupSet=[btcfgStaticFilter] ; collisionFilterMask : btCollisionFilterGroupSet=btcfgAllButStatic);virtual; //btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter
    procedure addRigidBody(const body : btRigidBody);override;
    procedure addRigidBody(const body : btRigidBody ; const group,mask : btCollisionFilterGroupSet) ;virtual;
    procedure removeRigidBody(const body : btRigidBody);override;
    ///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
    procedure removeCollisionObject(const collisionObject : btCollisionObject);override;
    procedure debugDrawConstraint(const constraint:btTypedConstraint);
    procedure debugDrawWorld; override;
    procedure setConstraintSolver(const solver:btConstraintSolver);override;
    function  getConstraintSolver:btConstraintSolver;override;
    function  getNumConstraints:integer;virtual;
    function  getConstraint(const index:integer):btTypedConstraint;override;
    function  getWorldType:btDynamicsWorldType;override;
    ///the forces on each rigidbody is accumulating together with gravity. clear this after each timestep.
    procedure clearForces;override;
    ///apply gravity, call this once per timestep
    procedure applyGravity;virtual;
    procedure setNumTasks(const numTasks:integer);virtual;
    procedure setSynchronizeAllMotionStates(const synchronizeAll:boolean);
    function  getSynchronizeAllMotionStates:Boolean;
  end;



//Functions
  function btAdjustAngleToLimits(const angleInRadians, angleLowerLimitInRadians, angleUpperLimitInRadians : btScalar):btScalar;FOS_INLINE;
  function btRigidBodyCompare(const a,b:btRigidBody):nativeint;

  procedure fdb_pvec(const v:btVector3);
  procedure fdb_pmanifoldpoint(const mp : btOManifoldPoint ;const i:integer);
  procedure fdb_pmanifold(const m:btPersistentManifold);

implementation

function btAdjustAngleToLimits(const angleInRadians, angleLowerLimitInRadians, angleUpperLimitInRadians: btScalar): btScalar;
var diffHi,diffLo : btScalar;
begin
  if angleLowerLimitInRadians >= angleUpperLimitInRadians then begin
    result := angleInRadians;
  end else
  if angleInRadians < angleLowerLimitInRadians then begin
    diffLo := btFabs(btNormalizeAngle(angleLowerLimitInRadians - angleInRadians));
    diffHi := btFabs(btNormalizeAngle(angleUpperLimitInRadians - angleInRadians));
    Result := btDecide(diffLo < diffHi , angleInRadians , angleInRadians + SIMD_2_PI);
  end else
  if angleInRadians > angleUpperLimitInRadians then begin
    diffHi := btFabs(btNormalizeAngle(angleInRadians - angleUpperLimitInRadians));
    diffLo := btFabs(btNormalizeAngle(angleInRadians - angleLowerLimitInRadians));
    Result := btDecide (diffLo < diffHi , angleInRadians - SIMD_2_PI , angleInRadians);
  end else begin
    Result := angleInRadians;
  end;
end;

function btRigidBodyCompare(const a, b: btRigidBody): nativeint;
begin
  //result := a=b;
  if a=b then exit(0);
  exit(-1);
end;

procedure fdb_pvec(const v: btVector3);
begin
  write('['+v.DumpSimple+'] ');
end;

procedure fdb_pbody(const body: btRigidBody; const i: integer);
begin
//  writeln('  Body('+inttostr(i)+') '+body.getWorldTransformP^.getOriginV^.DumpSimpleFull+'  LV '+body.getLinearVelocityP^.DumpSimpleFull+'  AV '+body.getAngularVelocityP^.DumpSimpleFull);
   //writeln(format(' body id(%d/%d) o(%s) lv[%s] av[%s]) lf[%s] af[%s] '+LineEnding+' dlv[%s] dav[%s] ',[
   //	   i,body.getActivationState,
   //        body.getWorldTransformP^.getOriginV^.DumpSimple,
   //	   //body.getWorldTransformP^.getBasisV^[0].DumpSimple,
   //	   //body.getWorldTransformP^.getBasisV^[1].DumpSimple,
   //	   //body.getWorldTransformP^.getBasisV^[2].DumpSimple,
   //	   body.getLinearVelocityP^.DumpSimple,
   //        //body.getAngularVelocityP^.DumpSimple,
   //        //body.getLinearFactorP^.Dumpsimple,
   //        //body.getAngularFactorP^.DumpSimple,
   //        //body.getDeltaLinearVelocityP^.DumpSimple,
   //        //body.getDeltaAngularVelocityP^.DumpSimple
   //        ])
   //	   );
   //fdb_pvec(body.getInterpolationWorldTransformP^.getOriginV^);
   ////fdb_pvec(body.getInterpolationWorldTransformP^.getBasisV^[0]);
   ////fdb_pvec(body.getInterpolationWorldTransformP^.getBasisV^[1]);
   ////fdb_pvec(body.getInterpolationWorldTransformP^.getBasisV^[2]);
   //writeln;
end;

procedure fdb_pmanifoldpoint(const mp: btOManifoldPoint; const i: integer);
begin

 write(format(' MP (%d) : ',[i]));
 write('lpa ');fdb_pvec(mp.m_localPointA);
 write('lpb ');fdb_pvec(mp.m_localPointB);
 write('pWA ');fdb_pvec(mp.m_positionWorldOnA);
 write('pWB ');fdb_pvec(mp.m_positionWorldOnB);
 writeln(format('d=%3.3f cF= %3.3f cR=%3.3f pid0/1/idx0/1 (%d %d %d %d)',[mp.m_distance1,mp.m_combinedFriction,mp.m_combinedRestitution,
                 mp.m_partId0,mp.m_partId1,mp.m_index0,mp.m_index1]));


 write(Format('   aI/aIL1/2=%3.3f/%3.3f/%3.3f cM1/2=%3.3f/%3.3f CFM1/2=%3.3f/%3.3f lT=%3.3d',[mp.m_appliedImpulse,mp.m_appliedImpulseLateral1,mp.m_appliedImpulseLateral2,
 	   mp.m_contactMotion1,mp.m_contactMotion2,mp.m_contactCFM1,mp.m_contactCFM2,mp.m_lifeTime]));
 writeln;
 write('   lFD1  lFD2"');fdb_pvec(mp.m_lateralFrictionDir1);fdb_pvec(mp.m_lateralFrictionDir2);
 writeln;
end;

procedure fdb_pmanifold(const m: btPersistentManifold);
var
  i: Integer;
begin
  write(format('manifold index1a(%d) compIDa/b (%d/%d) numContacts(%d)',[m.m_index1a,m.m_companionIdA,m.m_companionIdB,m.getNumContacts]));
  if (m.getNumContacts()>0) then begin
    writeln('');
    for i:=0 to m.getNumContacts-1 do begin
      fdb_pmanifoldpoint(m.getContactPointP(i)^,i);
    end;
  end;
  writeln;
end;

{ btContactSolverInfo }

constructor btContactSolverInfo.create;
begin
  m_tau               := btScalar(0.6);
  m_damping           := btScalar(1.0);
  m_friction          := btScalar(0.3);
  m_restitution       := btScalar(0);
  m_maxErrorReduction := btScalar(20);
  m_numIterations     := 10;
  m_erp               := btScalar(0.2);
  m_erp2              := btScalar(0.1);
  m_globalCfm         := btScalar(0);
  m_sor               := btScalar(1);
  m_splitImpulse      := false;
  m_splitImpulsePenetrationThreshold   := -0.02;
  m_linearSlop                         := btScalar(0.0);
  m_warmstartingFactor                 := btScalar(0.85);
  m_solverMode                         := [SOLVER_USE_WARMSTARTING, SOLVER_SIMD];// | SOLVER_RANDMIZE_ORDER;
  m_restingContactRestitutionThreshold := 2;//resting contact lifetime threshold to disable restitution
  m_minimumSolverBatchSize             := 128; //try to combine islands until the amount of constraints reaches this limit
end;


{ btRigidBodyConstructionInfo }

procedure btRigidBodyConstructionInfo.Init(const mass: btScalar; const motionState: btMotionState; const collisionShape: btCollisionShape; const localInertia: btVector3);
begin
  m_mass                        := mass;
  m_motionState                 := motionState;
  m_collisionShape              := collisionShape;
  m_localInertia                := localInertia;
  m_linearDamping               := 0;
  m_angularDamping              := 0;
  m_friction                    := 0.5;
  m_restitution                 := 0;
  m_linearSleepingThreshold     := 0.8;
  m_angularSleepingThreshold    := 1;
  m_additionalDamping           := false;
  m_additionalDampingFactor     := 0.005;
  m_additionalLinearDampingThresholdSqr  := 0.01;
  m_additionalAngularDampingThresholdSqr := 0.01;
  m_additionalAngularDampingFactor       := 0.01;
  m_startWorldTransform.setIdentity;
end;

{ btRigidBody }

constructor btRigidBody.create(const constructionInfo: btRigidBodyConstructionInfo);
begin
  inherited Create;
  setupRigidBody(constructionInfo);
  m_constraintRefs := btTypedConstraintArray.create;
  m_constraintRefs.Initialize(btTypedConstraint);
end;

constructor btRigidBody.create(const mass: btScalar; const motionState: btMotionState; const collisionShape: btCollisionShape; const localInertia: btVector3);
var cinfo:btRigidBodyConstructionInfo;
begin
  inherited Create;
  cinfo.Init(mass,motionState,collisionShape,localInertia);
  setupRigidBody(cinfo);
  m_constraintRefs := btTypedConstraintArray.create;
  m_constraintRefs.Initialize(btTypedConstraint);
end;

destructor btRigidBody.destroy;
begin
  m_constraintRefs.Free;
  //No constraints should point to this rigidbody
  //Remove constraints from the dynamics world before you delete the related rigidbodies.
  btAssert(m_constraintRefs.size=0);
  inherited;
end;

procedure btRigidBody.setupRigidBody(const constructionInfo: btRigidBodyConstructionInfo);
begin
  m_internalType   := CO_RIGID_BODY;
  m_linearVelocity.zero;
  m_angularVelocity.zero;
  m_angularFactor.InitSame(1);
  m_linearFactor.Initsame(1);
  m_gravity.zero;
  m_gravity_acceleration.zero;
  m_totalForce.zero;
  m_totalTorque.zero;
  m_linearDamping  := 0;
  m_angularDamping := 0.5;
  m_linearSleepingThreshold  := constructionInfo.m_linearSleepingThreshold;
  m_angularSleepingThreshold := constructionInfo.m_angularSleepingThreshold;
  m_optionalMotionState      := constructionInfo.m_motionState;
  m_contactSolverType        := 0;
  m_frictionSolverType       := 0;
  m_additionalDamping        := constructionInfo.m_additionalDamping;
  m_additionalDampingFactor  := constructionInfo.m_additionalDampingFactor;
  m_additionalLinearDampingThresholdSqr  := constructionInfo.m_additionalLinearDampingThresholdSqr;
  m_additionalAngularDampingThresholdSqr := constructionInfo.m_additionalAngularDampingThresholdSqr;
  m_additionalAngularDampingFactor       := constructionInfo.m_additionalAngularDampingFactor;

  if assigned(m_optionalMotionState) then begin
    m_optionalMotionState.getWorldTransform(m_worldTransform);
  end else begin
    m_worldTransform := constructionInfo.m_startWorldTransform;
  end;

  m_interpolationWorldTransform := m_worldTransform;
  m_interpolationLinearVelocity.zero;
  m_interpolationAngularVelocity.zero;

  //moved to btCollisionObject
  m_friction    := constructionInfo.m_friction;
  m_restitution := constructionInfo.m_restitution;

  setCollisionShape( constructionInfo.m_collisionShape );
  m_debugBodyId := gUniqueID;
  inc(gUniqueID);

  setMassProps(constructionInfo.m_mass, constructionInfo.m_localInertia);
  setDamping(constructionInfo.m_linearDamping, constructionInfo.m_angularDamping);
  updateInertiaTensor;

  m_rigidbodyFlags := [];

  m_deltaLinearVelocity.zero;
  m_deltaAngularVelocity.zero;
  m_invMass := m_inverseMass*m_linearFactor;
  m_pushVelocity.Zero;
  m_turnVelocity.Zero;
end;

procedure btRigidBody.proceedToTransform(const newTrans: btTransform);
begin
  setCenterOfMassTransform(newTrans);
end;

class function btRigidBody.upcast(const colObj: btCollisionObject): btRigidBody;
begin
  if colObj.getInternalType = CO_RIGID_BODY then begin
    result := btRigidBody(colObj);
  end else begin
    result := nil;
  end;
end;

procedure btRigidBody.predictIntegratedTransform(const step: btScalar; out predictedTransform: btTransform);
begin
  btTransformUtil.integrateTransform(m_worldTransform,m_linearVelocity,m_angularVelocity,step,predictedTransform);
end;

procedure btRigidBody.saveKinematicState(const step: btScalar);
//var  linVel,angVel : btVector3;
begin
  //todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
  if step <> 0 then begin
      //if we use motionstate to synchronize world transforms, get the new kinematic/animated world transform
      if assigned(m_optionalMotionState) then begin
        m_optionalMotionState.getWorldTransform(m_worldTransform);
      end;
      btTransformUtil.calculateVelocity(m_interpolationWorldTransform,m_worldTransform,step,m_linearVelocity,m_angularVelocity);
      m_interpolationLinearVelocity  := m_linearVelocity;
      m_interpolationAngularVelocity := m_angularVelocity;
      m_interpolationWorldTransform  := m_worldTransform;
      //printf("angular = %f %f %f\n",m_angularVelocity.getX(),m_angularVelocity.getY(),m_angularVelocity.getZ());
  end;
end;

procedure btRigidBody.applyGravity;
begin
  if isStaticOrKinematicObject then exit;
  applyCentralForce(m_gravity);
end;

procedure btRigidBody.setGravity(const acceleration: btVector3);
begin
  if m_inverseMass <> 0 then begin
    m_gravity := acceleration * (1/m_inverseMass);
  end;
  m_gravity_acceleration := acceleration;
end;

function btRigidBody.getGravityP: PbtVector3;
begin
  Result := @m_gravity_acceleration;
end;

procedure btRigidBody.setDamping(const lin_damping, ang_damping: btScalar);
begin
  m_linearDamping  := btClamped(lin_damping, 0, 1);
  m_angularDamping := btClamped(ang_damping, 0, 1);
end;

function btRigidBody.getLinearDamping: btScalar;
begin
  Result := m_linearDamping;
end;

function btRigidBody.getAngularDamping: btScalar;
begin
  result := m_angularDamping;
end;

function btRigidBody.getLinearSleepingThreshold: btScalar;
begin
  result := m_linearSleepingThreshold;
end;

function btRigidBody.getAngularSleepingThreshold: btScalar;
begin
  Result := m_angularSleepingThreshold;
end;

///applyDamping damps the velocity, using the given m_linearDamping and m_angularDamping
procedure btRigidBody.applyDamping(const timeStep: btScalar);

  procedure _Additional;
  var speed,dampVel,angSpeed,angDampVel : btScalar;
      dir                    : btVector3;
  begin
    //Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
    //Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
    if (m_angularVelocity.length2 < m_additionalAngularDampingThresholdSqr) and (m_linearVelocity.length2 < m_additionalLinearDampingThresholdSqr) then begin
      m_angularVelocity *= m_additionalDampingFactor;
      m_linearVelocity *= m_additionalDampingFactor;
    end;
    speed := m_linearVelocity.length;
    if speed < m_linearDamping then begin
      dampVel := 0.005;
      if speed > dampVel then begin
        dir := m_linearVelocity.normalized;
        m_linearVelocity -=  dir * dampVel;
      end else begin
        m_linearVelocity.zero;
      end;
    end;
    angSpeed := m_angularVelocity.length;
    if angSpeed < m_angularDamping then begin
      angDampVel := 0.005;
      if angSpeed > angDampVel then begin
        dir := m_angularVelocity.normalized;
        m_angularVelocity -=  dir * angDampVel;
      end else begin
        m_angularVelocity.zero;
      end;
    end;
  end;

begin
  //On new damping: see discussion/issue report here: http://code.google.com/p/bullet/issues/detail?id=74
  //todo: do some performance comparisons (but other parts of the engine are probably bottleneck anyway

//#define USE_OLD_DAMPING_METHOD 1
{$ifdef USE_OLD_DAMPING_METHOD}
  m_linearVelocity *= GEN_clamped((btScalar(1.) - timeStep * m_linearDamping), (btScalar)btScalar(0.0), (btScalar)btScalar(1.0));
  m_angularVelocity *= GEN_clamped((btScalar(1.) - timeStep * m_angularDamping), (btScalar)btScalar(0.0), (btScalar)btScalar(1.0));
{$else}
  m_linearVelocity  *= btPow(btScalar(1)-m_linearDamping, timeStep);
  m_angularVelocity *= btPow(btScalar(1)-m_angularDamping, timeStep);
{$endif}

  if m_additionalDamping then begin
    _Additional;
  end;
end;

function btRigidBody.getCollisionShape: btCollisionShape;
begin
  result := m_collisionShape;
end;

procedure btRigidBody.setMassProps(const mass: btScalar; const inertia: btVector3);
begin
  if mass = 0 then begin
    include(m_collisionFlags,CF_STATIC_OBJECT);
    m_inverseMass := 0;
  end else begin
    exclude(m_collisionFlags,CF_STATIC_OBJECT);
    m_inverseMass := 1 / mass;
  end;
  //Fg = m * a
  m_gravity := mass * m_gravity_acceleration;
  m_invInertiaLocal.init(btDecide(inertia.x <> 0,1/inertia.x,0),btDecide(inertia.y <> 0,1/inertia.y,0),btDecide(inertia.z <> 0,1/inertia.z,0));
  m_invMass := m_linearFactor*m_inverseMass;
end;

function btRigidBody.getLinearFactorP: PbtVector3;
begin
  result := @m_linearFactor;
end;

procedure btRigidBody.setLinearFactor(const linearFactor: btVector3);
begin
  m_linearFactor := linearFactor;
  m_invMass      := m_linearFactor*m_inverseMass;
end;

function btRigidBody.getInvMass: btScalar;
begin
  result := m_inverseMass;
end;

function btRigidBody.getInvInertiaTensorWorldP: PbtMatrix3x3;
begin
  result := @m_invInertiaTensorWorld;
end;

procedure btRigidBody.integrateVelocities(const step: btScalar);
var  angvel:btScalar;
begin
  if isStaticOrKinematicObject then begin
    exit;
  end;
  m_linearVelocity  += m_totalForce * (m_inverseMass * step);
  m_angularVelocity += m_invInertiaTensorWorld * m_totalTorque * step;

{$define MAX_ANGVEL:=SIMD_HALF_PI}
  /// clamp angular velocity. collision calculations will fail on higher angular velocities
  angvel := m_angularVelocity.length;
  if (angvel*step) > MAX_ANGVEL then begin
    m_angularVelocity *= (MAX_ANGVEL/step) / angvel;
  end;
end;

procedure btRigidBody.setCenterOfMassTransform(const xform: btTransform);
begin
  if isStaticOrKinematicObject then begin
    m_interpolationWorldTransform := m_worldTransform;
  end else begin
    m_interpolationWorldTransform := xform;
  end;
  m_interpolationLinearVelocity   := getLinearVelocityP^;
  m_interpolationAngularVelocity  := getAngularVelocityP^;
  m_worldTransform := xform;
  updateInertiaTensor;
end;

procedure btRigidBody.applyCentralForce(const force: btVector3);
begin
  m_totalForce += force*m_linearFactor;
end;

function btRigidBody.getTotalForceP: PbtVector3;
begin
  result := @m_totalForce;
end;

function btRigidBody.getTotalTorqueP: PbtVector3;
begin
  result := @m_totalTorque;
end;

function btRigidBody.getInvInertiaDiagLocalP: PbtVector3;
begin
  result := @m_invInertiaLocal;
end;

procedure btRigidBody.setInvInertiaDiagLocal(const diagInvInertia: btVector3);
begin
  m_invInertiaLocal := diagInvInertia;
end;

procedure btRigidBody.setSleepingThresholds(const linear, angular: btScalar);
begin
  m_linearSleepingThreshold  := linear;
  m_angularSleepingThreshold := angular;
end;

procedure btRigidBody.applyTorque(const torque: btVector3);
begin
  m_totalTorque += torque*m_angularFactor;
end;

procedure btRigidBody.applyForce(const force, rel_pos: btVector3);
begin
  applyCentralForce(force);
  applyTorque(rel_pos.cross(force*m_linearFactor));
end;

procedure btRigidBody.applyCentralImpulse(const impulse: btVector3);
begin
  m_linearVelocity += impulse *m_linearFactor * m_inverseMass;
end;

procedure btRigidBody.applyTorqueImpulse(const torque: btVector3);
begin
  m_angularVelocity += m_invInertiaTensorWorld * torque * m_angularFactor;
end;

procedure btRigidBody.applyImpulse(const impulse, rel_pos: btVector3);
begin
  if m_inverseMass <> 0 then begin
    applyCentralImpulse(impulse);
    //if m_angularFactor then begin
      applyTorqueImpulse(rel_pos.cross(impulse*m_linearFactor));
    //end;
  end;
end;

procedure btRigidBody.clearForces;
begin
  m_totalForce.zero;
  m_totalTorque.zero;
end;

procedure btRigidBody.updateInertiaTensor;
var sm,tm : btMatrix3x3;
begin
  sm := m_worldTransform.getBasisV^.scaled(m_invInertiaLocal);
  tm := m_worldTransform.getBasisV^.transpose;
  m_invInertiaTensorWorld :=  sm * tm;
end;

function btRigidBody.getCenterOfMassPositionP: PbtVector3;
begin
  result := m_worldTransform.getOriginV;
end;

function btRigidBody.getOrientation: btQuaternion;
begin
  m_worldTransform.getBasisV^.getRotation(result);
end;

function btRigidBody.getCenterOfMassTransformP: PbtTransform;
begin
  result := @m_worldTransform;
end;

function btRigidBody.getLinearVelocityP: PbtVector3;
begin
  result := @m_linearVelocity;
end;

function btRigidBody.getAngularVelocityP: PbtVector3;
begin
  result := @m_angularVelocity;
end;

procedure btRigidBody.setLinearVelocity(const lin_vel: btVector3);
begin
  m_linearVelocity := lin_vel;
end;

procedure btRigidBody.setAngularVelocity(const ang_vel: btVector3);
begin
  m_angularVelocity := ang_vel;
end;

function btRigidBody.getVelocityInLocalPoint(const rel_pos: btVector3): btVector3;
begin
  //we also calculate lin/ang velocity for kinematic objects
  Result := m_linearVelocity + m_angularVelocity.cross(rel_pos);
  //for kinematic objects, we could also use use:
  //              return  (m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
end;

procedure btRigidBody.translate(const v: btVector3);
begin
  m_worldTransform.getOriginV^ += v;
end;

procedure btRigidBody.getAabb(var aabbMin, aabbMax: btVector3);
begin
  m_collisionShape.getAabb(m_worldTransform,aabbMin,aabbMax);
end;

function btRigidBody.computeImpulseDenominator(const pos: btVector3; const normal: btVector3): btScalar;
var r0,c0,vec : btVector3;
begin
  r0     := pos - getCenterOfMassPositionP^;
  c0     := (r0).cross(normal);
  vec    := (c0 * getInvInertiaTensorWorldP^).cross(r0);
  Result := m_inverseMass + normal.dot(vec);
end;

function btRigidBody.computeAngularImpulseDenominator(const axis: btVector3): btScalar;
var vec : btVector3;
begin
  vec := axis * getInvInertiaTensorWorldP^;
  Result := axis.dot(vec);
end;

procedure btRigidBody.updateDeactivation(const timeStep: btScalar);
var a,b : btScalar;
begin
  if (getActivationState = btas_ISLAND_SLEEPING) or (getActivationState = btas_DISABLE_DEACTIVATION) then begin
    exit;
  end;
  a:=getLinearVelocityP^.length2;
  b:=getAngularVelocityP^.length2;
  if (a < (m_linearSleepingThreshold*m_linearSleepingThreshold)) and (b < (m_angularSleepingThreshold*m_angularSleepingThreshold)) then begin
    m_deactivationTime += timeStep;
  end else begin
    m_deactivationTime := 0;
    setActivationState(btas_NONE);
  end;
end;

function btRigidBody.wantsSleeping: boolean;
begin
  if m_activationState1 = btas_DISABLE_DEACTIVATION then begin
    exit(false);
  end;
  if gDisableDeactivation or (gDeactivationTime = 0) then begin
    exit(false);
  end;
  if (m_activationState1 = btas_ISLAND_SLEEPING) or (m_activationState1 = btas_WANTS_DEACTIVATION) then begin
    exit(true);
  end;
  if (m_deactivationTime> gDeactivationTime) then begin
    exit(true);
  end;
  Result := false;
end;

function btRigidBody.getBroadphaseProxy: btBroadphaseProxy;
begin
  result := m_broadphaseHandle;
end;

procedure btRigidBody.setNewBroadphaseProxy(const broadphaseProxy: btBroadphaseProxy);
begin
  m_broadphaseHandle := broadphaseProxy;
end;

function btRigidBody.getMotionState: btMotionState;
begin
  Result := m_optionalMotionState;
end;

procedure btRigidBody.setMotionState(const motionState: btMotionState);
begin
  m_optionalMotionState := motionState;
  if assigned(m_optionalMotionState) then begin
    motionState.getWorldTransform(m_worldTransform);
  end;
end;

procedure btRigidBody.setAngularFactor(const angFac: btVector3);
begin
  m_angularFactor := angFac;
end;

procedure btRigidBody.setAngularFactor(const angFac: btScalar);
begin
  m_angularFactor.InitSame(angFac);
end;

function btRigidBody.getAngularFactorP: PbtVector3;
begin
  Result := @m_angularFactor;
end;

function btRigidBody.isInWorld: boolean;
begin
  result := assigned(m_broadphaseHandle);
end;

function btRigidBody.checkCollideWithOverride(const co: btCollisionObject): boolean;
var otherRb : btRigidBody;
    c       : btTypedConstraint;
    i       : Integer;
begin
  otherRb := upcast(co);
  if not assigned(otherRb) then exit;
  for i := 0 to m_constraintRefs.size-1 do begin
    c := m_constraintRefs.A[i]^;
    if (c.getRigidBodyA = otherRb) or (c.getRigidBodyB = otherRb) then exit(false);
  end;
  Result := true;
end;

procedure btRigidBody.addConstraintRef(const c: btTypedConstraint);
var index : integer;
begin
  index := m_constraintRefs.findLinearSearch(c);
  if index = m_constraintRefs.size then begin
    m_constraintRefs.push_back(c);
  end;
  m_checkCollideWith := true;
end;

procedure btRigidBody.removeConstraintRef(const c: btTypedConstraint);
begin
  m_constraintRefs.remove(c);
  m_checkCollideWith := m_constraintRefs.size > 0;
end;

function btRigidBody.getConstraintRef(const index: integer): btTypedConstraint;
begin
  Result := m_constraintRefs.A[index]^;
end;

function btRigidBody.getNumConstraintRefs: integer;
begin
  Result := m_constraintRefs.size;
end;

procedure btRigidBody.setFlags(const flags: btRigidBodyFlagSet);
begin
  m_rigidbodyFlags := flags;
end;

function btRigidBody.getFlags: btRigidBodyFlagSet;
begin
  Result := m_rigidbodyFlags;
end;

function btRigidBody.getDeltaLinearVelocityP: PbtVector3;
begin
  result := @m_deltaLinearVelocity;
end;

function btRigidBody.getDeltaAngularVelocityP: PbtVector3;
begin
  result := @m_deltaAngularVelocity;
end;

function btRigidBody.getPushVelocityP: PbtVector3;
begin
  result := @m_pushVelocity;
end;

function btRigidBody.getTurnVelocityP: PbtVector3;
begin
  result := @m_turnVelocity;
end;

function btRigidBody.internalGetDeltaLinearVelocityP: PbtVector3;
begin
  result := @m_deltaLinearVelocity;
end;

function btRigidBody.internalGetDeltaAngularVelocityP: PbtVector3;
begin
  result := @m_deltaAngularVelocity;
end;

function btRigidBody.internalGetAngularFactorP: PbtVector3;
begin
  result := @m_angularFactor;
end;

function btRigidBody.internalGetInvMassP: PbtVector3;
begin
  result := @m_invMass;
end;

function btRigidBody.internalGetPushVelocityP: PbtVector3;
begin
  result := @m_pushVelocity;
end;

function btRigidBody.internalGetTurnVelocityP: PbtVector3;
begin
  result := @m_turnVelocity;
end;

procedure btRigidBody.internalGetVelocityInLocalPointObsolete(const rel_pos : btVector3 ; var velocity: btVector3);
begin
  velocity := getLinearVelocityP^+m_deltaLinearVelocity + (getAngularVelocityP^+m_deltaAngularVelocity).cross(rel_pos);
end;

procedure btRigidBody.internalGetAngularVelocity(var angVel: btVector3);
begin
  angVel := getAngularVelocityP^+m_deltaAngularVelocity;
end;

procedure btRigidBody.internalApplyImpulse(const linearComponent, angularComponent: btVector3; const impulseMagnitude: btScalar);
begin
  if m_inverseMass<>0 then begin
    m_deltaLinearVelocity  += linearComponent*impulseMagnitude;
    m_deltaAngularVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
  end;
end;

procedure btRigidBody.internalApplyPushImpulse(const linearComponent, angularComponent: btVector3; const impulseMagnitude: btScalar);
begin
  if m_inverseMass<>0 then begin
    m_pushVelocity += linearComponent*impulseMagnitude;
    m_turnVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
  end;
end;

procedure btRigidBody.internalWritebackVelocity;
begin
  if m_inverseMass<>0 then begin
    setLinearVelocity(getLinearVelocityP^+ m_deltaLinearVelocity);
    setAngularVelocity(getAngularVelocityP^+m_deltaAngularVelocity);
    //m_deltaLinearVelocity.setZero();
    //m_deltaAngularVelocity .setZero();
    //m_originalBody->setCompanionId(-1);
  end;
end;

procedure btRigidBody.internalWritebackVelocity(const timeStep: btScalar);
var newTransform : btTransform;
begin
  if m_inverseMass<>0 then begin
    setLinearVelocity(getLinearVelocityP^+ m_deltaLinearVelocity);
    setAngularVelocity(getAngularVelocityP^+m_deltaAngularVelocity);
    //correct the position/orientation based on push/turn recovery
    btTransformUtil.integrateTransform(getWorldTransformP^,m_pushVelocity,m_turnVelocity,timeStep,newTransform);
    setWorldTransform(newTransform);
    //m_originalBody->setCompanionId(-1);
  end;
end;

{ btTypedConstraint }

function btTypedConstraint.getMotorFactor(const pos, lowLim, uppLim, vel, timeFact: btScalar): btScalar;
var lim_fact,delta_max : btScalar;
begin
  if lowLim > uppLim then begin
    Result := 1;
    exit;
  end else
  if lowLim = uppLim then begin
    Result := 0;
    exit;
  end;
  lim_fact  := 1;
  delta_max := vel / timeFact;
  if delta_max < 0 then begin
    if (pos >= lowLim) and (pos < (lowLim - delta_max)) then begin
      lim_fact := (lowLim - pos) / delta_max;
    end else
    if (pos  < lowLim) then begin
      lim_fact := 0;
    end else begin
      lim_fact := 1;
    end;
  end else
  if delta_max > 0 then begin
    if (pos <= uppLim) and (pos > (uppLim - delta_max)) then begin
      lim_fact := (uppLim - pos) / delta_max;
    end else
    if pos  > uppLim then begin
      lim_fact := 0;
    end else begin
      lim_fact := 1;
    end;
  end else begin
    lim_fact := 0;
  end;
  Result := lim_fact;
end;

class function btTypedConstraint.getFixedBody: btRigidBody;
begin
  result := btRigidBody.create(0,nil,nil,cbtNullVector);
  result.setMassProps(0,cbtNullVector);
end;

constructor btTypedConstraint.create(const typ: btTypedConstraintType; const rbA: btRigidBody);
begin
  inherited Create(integer(typ));
  m_userConstraintType := -1;
  m_userConstraintId   := -1;
  m_needsFeedback      := false;
  m_rbA                := rbA;
  m_rbB                := getFixedBody;
  m_appliedImpulse     := 0;
  m_dbgDrawSize        := DEFAULT_DEBUGDRAW_SIZE;
end;

constructor btTypedConstraint.create(const typ: btTypedConstraintType; const rbA, rbB: btRigidBody);
begin
  inherited Create(integer(typ));
  m_userConstraintType := -1;
  m_userConstraintId   := -1;
  m_needsFeedback      := false;
  m_rbA                := rbA;
  m_rbB                := rbB;
  m_appliedImpulse     := 0;
  m_dbgDrawSize        := DEFAULT_DEBUGDRAW_SIZE;
end;

destructor btTypedConstraint.destroy;
begin
  inherited destroy;
end;

procedure btTypedConstraint.buildJacobian;
begin
  ;
end;

{$HINTS OFF}
procedure btTypedConstraint.setupSolverConstraint(const ca: btConstraintArray; const solverBodyA, solverBodyB: integer; const timeStep: btScalar);
begin
  ;
end;
{$HINTS ON}

procedure btTypedConstraint.internalSetAppliedImpulse(const appliedImpulse: btScalar);
begin
  m_appliedImpulse := appliedImpulse;
end;

function btTypedConstraint.internalGetAppliedImpulse: btScalar;
begin
  Result := m_appliedImpulse;
end;

{$HINTS OFF}
procedure btTypedConstraint.solveConstraintObsolete(const bodyA, bodyB: btRigidBody; const timeStep: btScalar);
begin
  //
end;
{$HINTS ON}

function btTypedConstraint.getRigidBodyA: btRigidBody;
begin
  result := m_rbA;
end;

function btTypedConstraint.getRigidBodyB: btRigidBody;
begin
  result := m_rbB;
end;

function btTypedConstraint.getUserConstraintType: integer;
begin
  result := m_userConstraintType;
end;

procedure btTypedConstraint.setUserConstraintType(const userConstraintType: Integer);
begin
  m_userConstraintType := userConstraintType;
end;

procedure btTypedConstraint.setUserConstraintId(const uid: integer);
begin
  m_userConstraintId := uid;
end;

function btTypedConstraint.getUserConstraintId: integer;
begin
  Result := m_userConstraintId;
end;

{$HINTS OFF}
procedure btTypedConstraint.setUserConstraintPtr(const ptr: pointer);
begin
  m_userConstraintId := PtrInt(ptr);
end;

function btTypedConstraint.getUserConstraintPtr: Pointer;
begin
  Result := Pointer(m_userConstraintId);
end;
{$HINTS ON}

function btTypedConstraint.getUid: Integer;
begin
  Result := m_userConstraintId;
end;

function btTypedConstraint.needsFeedback: Boolean;
begin
  result := m_needsFeedback;
end;

procedure btTypedConstraint.enableFeedback(const nFeedback: Boolean);
begin
  m_needsFeedback := nFeedback;
end;

function btTypedConstraint.getAppliedImpulse: btScalar;
begin
  //.
  btAssert(m_needsFeedback);
  Result := m_appliedImpulse;
end;

function btTypedConstraint.getConstraintType: btTypedConstraintType;
begin
  Result := btTypedConstraintType(m_objectType);
end;

procedure btTypedConstraint.setDbgDrawSize(const dbgDrawSize: btScalar);
begin
  m_dbgDrawSize := dbgDrawSize;
end;

function btTypedConstraint.getDbgDrawSize: btScalar;
begin
  result := m_dbgDrawSize;
end;

{ btConstraintSolver }
{$HINTS OFF}
procedure btConstraintSolver.prepareSolve(const numBodies, numManifolds: integer);
begin
  ;
end;
{$HINTS ON}

{$HINTS OFF}
procedure btConstraintSolver.allSolved(const info: btContactSolverInfo; const debugDrawer: btIDebugDraw; const stackAlloc: btStackAllocator);
begin
  ;
end;
{$HINTS ON}

{ btDynamicsWorld }

constructor btDynamicsWorld.create(const dispatcher: btDispatcher; const broadphase: btBroadphaseInterface; const collisionConfiguration: btCollisionConfiguration);
begin
  inherited Create(dispatcher,broadphase,collisionConfiguration);
  m_solverInfo := btContactSolverInfo.create;
end;

destructor btDynamicsWorld.destroy;
begin
  m_solverInfo.free;
  inherited destroy;
end;

{$HINTS OFF}
procedure btDynamicsWorld.addConstraint(const constraint: btTypedConstraint; const disableCollisionsBetweenLinkedBodies: Boolean);
begin
 ;
end;
{$HINTS ON}

{$HINTS OFF}
procedure btDynamicsWorld.removeConstraint(const constraint: btTypedConstraint);
begin

end;
{$HINTS ON}

function btDynamicsWorld.getNumConstraints: integer;
begin
  result:=0;
end;

{$HINTS OFF}
function btDynamicsWorld.getConstraint(const index: integer): btTypedConstraint;
begin
  result := nil;
end;
{$HINTS ON}

procedure btDynamicsWorld.setInternalTickCallback(const cb: btInternalTickCallback; const worldUserInfo: Pointer; const isPreTick: boolean);
begin
  if isPreTick then begin
    m_internalPreTickCallback := cb;
  end else begin
    m_internalTickCallback := cb;
  end;
  m_worldUserInfo := worldUserInfo;
end;

procedure btDynamicsWorld.setWorldUserInfo(const worldUserInfo: Pointer);
begin
 m_worldUserInfo := worldUserInfo;
end;

function btDynamicsWorld.getWorldUserInfo: pointer;
begin
  result := m_worldUserInfo;
end;

function btDynamicsWorld.getSolverInfo: btContactSolverInfo;
begin
 Result := m_solverInfo;
end;

{ btActionInterface }

class function btActionInterface.getFixedBody: btRigidBody;
begin
  result := nil;
end;

{ btSequentialImpulseConstraintSolver }

{$ifdef USE_SIMD}
//#include <emmintrin.h>
//#define vec_splat(x, e) _mm_shuffle_ps(x, x, _MM_SHUFFLE(e,e,e,e))
static inline __m128 _vmathVfDot3( __m128 vec0, __m128 vec1 )
{
	__m128 result = _mm_mul_ps( vec0, vec1);
	return _mm_add_ps( vec_splat( result, 0 ), _mm_add_ps( vec_splat( result, 1 ), vec_splat( result, 2 ) ) );
}
ss
{$endif}//USE_SIMD


procedure  applyAnisotropicFriction(const colObj : btCollisionObject;var frictionDirection : btVector3);
var loc_lateral,friction_scaling : btVector3;
begin
  if assigned(colObj) and colObj.hasAnisotropicFriction then begin
    // transform to local coordinates
    loc_lateral      := frictionDirection * colObj.getWorldTransformP^.getBasisV^;
    friction_scaling := colObj.getAnisotropicFriction;
    //apply anisotropic friction
    loc_lateral *= friction_scaling;
    // ... and transform it back to global coordinates
    frictionDirection := colObj.getWorldTransformP^.getBasisV^ * loc_lateral;
  end;
end;

{$HINTS OFF}
procedure btSequentialImpulseConstraintSolver.setupFrictionConstraint(const solverConstraint: PbtRSolverConstraint; const normalAxis: btVector3; const solverBodyA, solverBodyIdB: btRigidBody;
  const cp: PbtOManifoldPoint; const rel_pos1, rel_pos2: btVector3; const colObj0, colObj1: btCollisionObject; const relaxation: btScalar; const desiredVelocity: btScalar; const cfmSlip: btScalar);

var body0,body1   : btRigidBody;
    ftorqueAxis1  : btVector3;
    vec           : btVector3;
    denom0,denom1 : btScalar;
    rel_vel,vel1Dotn,vel2Dotn : btScalar;
    velocityError,velocityImpulse : btSimdScalar;
begin
  body0 := btRigidBody.upcast(colObj0);
  body1 := btRigidBody.upcast(colObj1);

  solverConstraint^.m_contactNormal := normalAxis;
  solverConstraint^.m_solverBodyA   := body0;
  solverConstraint^.m_solverBodyB   := body1;
  if not assigned(solverConstraint^.m_solverBodyA) then solverConstraint^.m_solverBodyA := getFixedBody;
  if not assigned(solverConstraint^.m_solverBodyB) then solverConstraint^.m_solverBodyB := getFixedBody;


  solverConstraint^.m_friction             := cp^.m_combinedFriction;
  solverConstraint^.m_originalContactPoint := nil;
  solverConstraint^.m_appliedImpulse       := 0;
  solverConstraint^.m_appliedPushImpulse   := 0;

  ftorqueAxis1                          := rel_pos1.cross(solverConstraint^.m_contactNormal);
  solverConstraint^.m_relpos1CrossNormal := ftorqueAxis1;
  if assigned(body0) then begin
    solverConstraint^.m_angularComponentA  := body0.getInvInertiaTensorWorldP^*ftorqueAxis1*body0.getAngularFactorP^;
  end else begin
    solverConstraint^.m_angularComponentA.zero;
  end;
  ftorqueAxis1                          := rel_pos2.cross(-solverConstraint^.m_contactNormal);
  solverConstraint^.m_relpos2CrossNormal := ftorqueAxis1;
  if assigned(body1) then begin
    solverConstraint^.m_angularComponentB  := body1.getInvInertiaTensorWorldP^*ftorqueAxis1*body1.getAngularFactorP^;
  end else begin
    solverConstraint^.m_angularComponentB.zero;
  end;

{$ifdef COMPUTE_IMPULSE_DENOM}
  btScalar denom0 = rb0->computeImpulseDenominator(pos1,solverConstraint^.m_contactNormal);
  btScalar denom1 = rb1->computeImpulseDenominator(pos2,solverConstraint^.m_contactNormal);
{$else}
  denom0 := 0;
  denom1 := 0;
  if assigned (body0) then begin
    vec    := ( solverConstraint^.m_angularComponentA).cross(rel_pos1);
    denom0 := body0.getInvMass + normalAxis.dot(vec);
  end;
  if assigned (body1) then begin
    vec    := ( -solverConstraint^.m_angularComponentB).cross(rel_pos2);
    denom1 := body1.getInvMass + normalAxis.dot(vec);
  end;
{$endif} //COMPUTE_IMPULSE_DENOM
  solverConstraint^.m_jacDiagABInv := relaxation/(denom0+denom1);

{$ifdef _USE_JACOBIAN}
  solverConstraint^.m_jac =  btJacobianEntry (
          rel_pos1,rel_pos2,solverConstraint^.m_contactNormal,
          body0->getInvInertiaDiagLocal(),
          body0->getInvMass(),
          body1->getInvInertiaDiagLocal(),
          body1->getInvMass());
{$endif} //_USE_JACOBIAN
  if assigned(body0) then begin
    vel1Dotn := solverConstraint^.m_contactNormal.dot(body0.getLinearVelocityP^)
               + solverConstraint^.m_relpos1CrossNormal.dot(body0.getAngularVelocityP^);
  end else begin
    vel1Dotn := solverConstraint^.m_contactNormal.dot(cbtNullVector)
               + solverConstraint^.m_relpos1CrossNormal.dot(cbtNullVector);
  end;
  if assigned(body1) then begin
    vel2Dotn := -solverConstraint^.m_contactNormal.dot(body1.getLinearVelocityP^)+ solverConstraint^.m_relpos2CrossNormal.dot(body1.getAngularVelocityP^);
  end else begin
    vel2Dotn := -solverConstraint^.m_contactNormal.dot(cbtNullVector)+ solverConstraint^.m_relpos2CrossNormal.dot(cbtNullVector);
  end;
  rel_vel         := vel1Dotn+vel2Dotn;
  velocityError   :=  desiredVelocity - rel_vel;
  velocityImpulse := velocityError * btSimdScalar(solverConstraint^.m_jacDiagABInv);
  solverConstraint^.m_rhs := velocityImpulse;
  solverConstraint^.m_cfm := cfmSlip;
  solverConstraint^.m_lowerLimit := 0;
  solverConstraint^.m_upperLimit := 1e10;
end;
{$HINTS ON}

function btSequentialImpulseConstraintSolver.addFrictionConstraint(const normalAxis: btVector3; const solverBodyA, solverBodyB: btRigidBody; const frictionIndex: Integer; const cp: PbtOManifoldPoint;
  const rel_pos1, rel_pos2: btVector3; const colObj0, colObj1: btCollisionObject; const relaxation: btScalar; const desiredVelocity: btScalar; const cfmSlip: btScalar): PbtRSolverConstraint;
var solverConstraint : PbtRSolverConstraint;
begin
 solverConstraint := m_tmpSolverContactFrictionConstraintPool.push_new_no_init;// ExpandNonInitializing;
 solverConstraint^.m_frictionIndex := frictionIndex;
 setupFrictionConstraint(solverConstraint, normalAxis, solverBodyA, solverBodyB, cp, rel_pos1, rel_pos2,colObj0, colObj1, relaxation, desiredVelocity, cfmSlip);
 Result := solverConstraint;
end;

procedure btSequentialImpulseConstraintSolver.setupContactConstraint(const solverConstraint: PbtRSolverConstraint; const colObj0, colObj1: btCollisionObject; const cp: btOManifoldPoint;
  const infoGlobal: btContactSolverInfo; out vel: btVector3; out rel_vel, relaxation: btScalar; out rel_pos1, rel_pos2: btVector3);
var rb0,rb1     : btRigidBody;
    pos1,pos2   : PbtVector3;
    torqueAxis0,
    torqueAxis1 : btVector3;
    vec : btVector3;
    denom0,denom1 : btScalar;
    vel1,vel2 : btVector3;
    penetration:btScalar;
    restitution:btScalar;
    vel1Dotn,vel2Dotn : btScalar;
    velocityImpulse,penetrationImpulse,velocityError,positionalError:btScalar;

begin
  rb0 := btRigidBody.upcast(colObj0);
  rb1 := btRigidBody.upcast(colObj1);

  pos1 := cp.getPositionWorldOnAP;
  pos2 := cp.getPositionWorldOnBP;

  rel_pos1   := pos1^ - colObj0.getWorldTransformP^.getOriginV^;
  rel_pos2   := pos2^ - colObj1.getWorldTransformP^.getOriginV^;
  relaxation := 1;

  torqueAxis0 := rel_pos1.cross(cp.m_normalWorldOnB);
  if assigned(rb0) then begin
    solverConstraint^.m_angularComponentA := rb0.getInvInertiaTensorWorldP^*torqueAxis0*rb0.getAngularFactorP^;
  end else begin
    solverConstraint^.m_angularComponentA.zero;
  end;
  torqueAxis1 := rel_pos2.cross(cp.m_normalWorldOnB);
  if assigneD(rb1) then begin
    solverConstraint^.m_angularComponentB := rb1.getInvInertiaTensorWorldP^*-torqueAxis1*rb1.getAngularFactorP^;
  end else begin
    solverConstraint^.m_angularComponentB.zero;
  end;

  {$ifdef COMPUTE_IMPULSE_DENOM}
  btScalar denom0 = rb0->computeImpulseDenominator(pos1,cp.m_normalWorldOnB);
  btScalar denom1 = rb1->computeImpulseDenominator(pos2,cp.m_normalWorldOnB);
  {$else}
  denom0 := 0;
  denom1 := 0;
  if assigned(rb0) then begin
    vec    := (solverConstraint^.m_angularComponentA).cross(rel_pos1);
    denom0 := rb0.getInvMass + cp.m_normalWorldOnB.dot(vec);
  end;
  if assigned(rb1) then begin
    vec    := ( -solverConstraint^.m_angularComponentB).cross(rel_pos2);
    denom1 := rb1.getInvMass + cp.m_normalWorldOnB.dot(vec);
  end;
  {$endif} //COMPUTE_IMPULSE_DENOM

  solverConstraint^.m_jacDiagABInv       := relaxation/(denom0+denom1);
  solverConstraint^.m_contactNormal      := cp.m_normalWorldOnB;
  solverConstraint^.m_relpos1CrossNormal := rel_pos1.cross(cp.m_normalWorldOnB);
  solverConstraint^.m_relpos2CrossNormal := rel_pos2.cross(-cp.m_normalWorldOnB);

  if assigned(rb0) then begin
    vel1 := rb0.getVelocityInLocalPoint(rel_pos1);
  end else begin
    vel1.zero;
  end;
  if assigned(rb1) then begin
    vel2 := rb1.getVelocityInLocalPoint(rel_pos2);
  end else begin
    vel2.zero;
  end;
  vel         := vel1 - vel2;
  rel_vel     := cp.m_normalWorldOnB.dot(vel);
  penetration := cp.getDistance()+infoGlobal.m_linearSlop;
  solverConstraint^.m_friction := cp.m_combinedFriction;
  //FOSCHECK? restitution := 0;
  if (cp.m_lifeTime>infoGlobal.m_restingContactRestitutionThreshold) then begin
    restitution := 0;
  end else begin
    restitution :=  restitutionCurve(rel_vel, cp.m_combinedRestitution);
    if restitution <= 0 then begin
      restitution := 0;
    end;
  end;

  ///warm starting (or zero if disabled)
  if SOLVER_USE_WARMSTARTING in infoGlobal.m_solverMode then begin
    solverConstraint^.m_appliedImpulse := cp.m_appliedImpulse * infoGlobal.m_warmstartingFactor;
    if assigned(rb0) then begin
      rb0.internalApplyImpulse(solverConstraint^.m_contactNormal*rb0.getInvMass*rb0.getLinearFactorP^,solverConstraint^.m_angularComponentA,solverConstraint^.m_appliedImpulse);
    end;
    if assigned(rb1) then begin
      rb1.internalApplyImpulse(solverConstraint^.m_contactNormal*rb1.getInvMass*rb1.getLinearFactorP^,-solverConstraint^.m_angularComponentB,-solverConstraint^.m_appliedImpulse);
    end;
  end else begin
    solverConstraint^.m_appliedImpulse := 0;
  end;
  solverConstraint^.m_appliedPushImpulse := 0;

  if assigned(rb0) then begin
    vel1Dotn := solverConstraint^.m_contactNormal.dot(rb0.getLinearVelocityP^) + solverConstraint^.m_relpos1CrossNormal.dot(rb0.getAngularVelocityP^);
  end else begin
    vel1Dotn := solverConstraint^.m_contactNormal.dot(cbtNullVector) + solverConstraint^.m_relpos1CrossNormal.dot(cbtNullVector);
  end;
  if assigned(rb1) then begin
    vel2Dotn := -solverConstraint^.m_contactNormal.dot(rb1.getLinearVelocityP^)+solverConstraint^.m_relpos2CrossNormal.dot(rb1.getAngularVelocityP^);
  end else begin
    vel2Dotn := -solverConstraint^.m_contactNormal.dot(cbtNullVector)+ solverConstraint^.m_relpos2CrossNormal.dot(cbtNullVector);
  end;
  rel_vel := vel1Dotn+vel2Dotn;

  positionalError    := -penetration * infoGlobal.m_erp/infoGlobal.m_timeStep;
  velocityError      := restitution - rel_vel;// * damping;
  penetrationImpulse := positionalError*solverConstraint^.m_jacDiagABInv;
  velocityImpulse    := velocityError *solverConstraint^.m_jacDiagABInv;
  if (infoGlobal.m_splitImpulse=false) or (penetration > infoGlobal.m_splitImpulsePenetrationThreshold) then begin
    //combine position and velocity into rhs
    solverConstraint^.m_rhs := penetrationImpulse+velocityImpulse;
    solverConstraint^.m_rhsPenetration := 0;
  end else begin
    //split position and velocity into rhs and m_rhsPenetration
    solverConstraint^.m_rhs := velocityImpulse;
    solverConstraint^.m_rhsPenetration := penetrationImpulse;
  end;
  solverConstraint^.m_cfm        := 0;
  solverConstraint^.m_lowerLimit := 0;
  solverConstraint^.m_upperLimit := 1e10;
end;

procedure btSequentialImpulseConstraintSolver.setFrictionConstraintImpulse(const solverConstraint: PbtRSolverConstraint; const rb0, rb1: btRigidBody; const cp: btOManifoldPoint; const infoGlobal: btContactSolverInfo);
var frictionConstraint1,frictionConstraint2 : PbtRSolverConstraint;
begin
  if SOLVER_USE_FRICTION_WARMSTARTING in infoGlobal.m_solverMode  then begin
    frictionConstraint1 := m_tmpSolverContactFrictionConstraintPool.A[solverConstraint^.m_frictionIndex];
    if SOLVER_USE_WARMSTARTING in infoGlobal.m_solverMode then begin
      frictionConstraint1^.m_appliedImpulse := cp.m_appliedImpulseLateral1 * infoGlobal.m_warmstartingFactor;
      if assigned(rb0) then begin
        rb0.internalApplyImpulse(frictionConstraint1^.m_contactNormal*rb0.getInvMass*rb0.getLinearFactorP^,frictionConstraint1^.m_angularComponentA,frictionConstraint1^.m_appliedImpulse);
      end;
      if assigned(rb1) then begin
        rb1.internalApplyImpulse(frictionConstraint1^.m_contactNormal*rb1.getInvMass*rb1.getLinearFactorP^,-frictionConstraint1^.m_angularComponentB,-frictionConstraint1^.m_appliedImpulse);
      end;
    end else begin
      frictionConstraint1^.m_appliedImpulse := 0;
    end;
    if SOLVER_USE_2_FRICTION_DIRECTIONS in infoGlobal.m_solverMode then begin
      frictionConstraint2 := m_tmpSolverContactFrictionConstraintPool.A[solverConstraint^.m_frictionIndex+1];
      if SOLVER_USE_WARMSTARTING in infoGlobal.m_solverMode then begin
        frictionConstraint2^.m_appliedImpulse := cp.m_appliedImpulseLateral2 * infoGlobal.m_warmstartingFactor;
        if assigned(rb0) then begin
          rb0.internalApplyImpulse(frictionConstraint2^.m_contactNormal*rb0.getInvMass,frictionConstraint2^.m_angularComponentA,frictionConstraint2^.m_appliedImpulse);
        end;
        if assigned(rb1) then begin
          rb1.internalApplyImpulse(frictionConstraint2^.m_contactNormal*rb1.getInvMass,-frictionConstraint2^.m_angularComponentB,-frictionConstraint2^.m_appliedImpulse);
        end;
      end else begin
        frictionConstraint2^.m_appliedImpulse := 0;
      end;
    end;
  end else begin
    frictionConstraint1 := m_tmpSolverContactFrictionConstraintPool.A[solverConstraint^.m_frictionIndex];
    frictionConstraint1^.m_appliedImpulse := 0;
    if SOLVER_USE_2_FRICTION_DIRECTIONS in infoGlobal.m_solverMode then begin
      frictionConstraint2 := m_tmpSolverContactFrictionConstraintPool.A[solverConstraint^.m_frictionIndex+1];
      frictionConstraint2^.m_appliedImpulse := 0;
    end;
  end;
end;

function btSequentialImpulseConstraintSolver.restitutionCurve(const rel_vel, restitution: btScalar): btScalar;
begin
 Result := restitution * -rel_vel;
end;

procedure btSequentialImpulseConstraintSolver.convertContact(const manifold: btPersistentManifold; const infoGlobal: btContactSolverInfo);
var  colObj0,colObj1         : btCollisionObject;
     solverBodyA,solverBodyB : btRigidBody;
     j                       : Integer;
     cp                      : PbtOManifoldPoint;
     vel,rel_pos1,rel_pos2   : btVector3;
     relaxation, rel_vel,
     lat_rel_vel             : btScalar;
     frictionIndex           : integer;
     solverConstraint        : PbtRSolverConstraint;
     rb0,rb1                 : btRigidBody;
begin
  colObj0     := btCollisionObject(manifold.getBody0);
  colObj1     := btCollisionObject(manifold.getBody1);
  solverBodyA := btRigidBody.upcast(colObj0);
  solverBodyB := btRigidBody.upcast(colObj1);
  ///avoid collision response between two static objects
  if (not assigned(solverBodyA) or (solverBodyA.getInvMass=0)) and (not assigned(solverBodyB) or (solverBodyB.getInvMass=0)) then exit;
  for j := 0 to manifold.getNumContacts-1 do begin
    cp := manifold.getContactPointP(j);
    if cp^.getDistance <= manifold.getContactProcessingThreshold then begin
      frictionIndex    := m_tmpSolverContactConstraintPool.size;
      solverConstraint := m_tmpSolverContactConstraintPool.push_new_no_init; // expandNonInitializing;
      rb0 := btRigidBody.upcast(colObj0);
      rb1 := btRigidBody.upcast(colObj1);
      if assigned(rb0) then begin
        solverConstraint^.m_solverBodyA := rb0;
      end else begin
        solverConstraint^.m_solverBodyA := getFixedBody;
      end;
      if assigned(rb1) then begin
        solverConstraint^.m_solverBodyB := rb1;
      end else begin
        solverConstraint^.m_solverBodyB := getFixedBody;
      end;
      solverConstraint^.m_originalContactPoint := cp;
      setupContactConstraint(solverConstraint, colObj0, colObj1, cp^, infoGlobal, vel, rel_vel, relaxation, rel_pos1, rel_pos2);
      /////setup the friction constraints
      solverConstraint^.m_frictionIndex := m_tmpSolverContactFrictionConstraintPool.size;
      if not(SOLVER_ENABLE_FRICTION_DIRECTION_CACHING in infoGlobal.m_solverMode) or not(cp^.m_lateralFrictionInitialized) then begin
        cp^.m_lateralFrictionDir1 := vel - cp^.m_normalWorldOnB * rel_vel;
        lat_rel_vel := cp^.m_lateralFrictionDir1.length2;
        if not(SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION in infoGlobal.m_solverMode) and (lat_rel_vel > SIMD_EPSILON) then begin
          cp^.m_lateralFrictionDir1 /= btSqrt(lat_rel_vel);
          if SOLVER_USE_2_FRICTION_DIRECTIONS in infoGlobal.m_solverMode then begin
            cp^.m_lateralFrictionDir2 := cp^.m_lateralFrictionDir1.cross(cp^.m_normalWorldOnB);
            cp^.m_lateralFrictionDir2.normalize;//??
            applyAnisotropicFriction(colObj0,cp^.m_lateralFrictionDir2);
            applyAnisotropicFriction(colObj1,cp^.m_lateralFrictionDir2);
            addFrictionConstraint(cp^.m_lateralFrictionDir2,solverBodyA,solverBodyB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
          end;
          applyAnisotropicFriction(colObj0,cp^.m_lateralFrictionDir1);
          applyAnisotropicFriction(colObj1,cp^.m_lateralFrictionDir1);
          addFrictionConstraint(cp^.m_lateralFrictionDir1,solverBodyA,solverBodyB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
          cp^.m_lateralFrictionInitialized := true;
        end else begin
          //re-calculate friction direction every frame, todo: check if this is really needed
          btPlaneSpace1(cp^.m_normalWorldOnB,cp^.m_lateralFrictionDir1,cp^.m_lateralFrictionDir2);
          if SOLVER_USE_2_FRICTION_DIRECTIONS in infoGlobal.m_solverMode then begin
            applyAnisotropicFriction(colObj0,cp^.m_lateralFrictionDir2);
            applyAnisotropicFriction(colObj1,cp^.m_lateralFrictionDir2);
            addFrictionConstraint(cp^.m_lateralFrictionDir2,solverBodyA,solverBodyB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
          end;
          applyAnisotropicFriction(colObj0,cp^.m_lateralFrictionDir1);
          applyAnisotropicFriction(colObj1,cp^.m_lateralFrictionDir1);
          addFrictionConstraint(cp^.m_lateralFrictionDir1,solverBodyA,solverBodyB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
          cp^.m_lateralFrictionInitialized := true;
        end;
      end else begin
        addFrictionConstraint(cp^.m_lateralFrictionDir1,solverBodyA,solverBodyB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation,cp^.m_contactMotion1, cp^.m_contactCFM1);
        if SOLVER_USE_2_FRICTION_DIRECTIONS in infoGlobal.m_solverMode then begin
          addFrictionConstraint(cp^.m_lateralFrictionDir2,solverBodyA,solverBodyB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation, cp^.m_contactMotion2, cp^.m_contactCFM2);
        end;
      end;
      setFrictionConstraintImpulse( solverConstraint, rb0, rb1, cp^, infoGlobal);
    end;
  end;
end;

procedure btSequentialImpulseConstraintSolver.resolveSplitPenetrationSIMD(const body1, body2: btRigidBody; const contactConstraint: PbtRSolverConstraint);
begin
{$ifdef USE_SIMD}
  if (!c.m_rhsPenetration)
          return;
  gNumSplitImpulseRecoveries++;
  __m128 cpAppliedImp = _mm_set1_ps(c.m_appliedPushImpulse);
  __m128  lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
  __m128  upperLimit1 = _mm_set1_ps(c.m_upperLimit);
  __m128 deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhsPenetration), _mm_mul_ps(_mm_set1_ps(c.m_appliedPushImpulse),_mm_set1_ps(c.m_cfm)));
  __m128 deltaVel1Dotn    =       _mm_add_ps(_vmathVfDot3(c.m_contactNormal.mVec128,body1.internalGetPushVelocity().mVec128), _vmathVfDot3(c.m_relpos1CrossNormal.mVec128,body1.internalGetTurnVelocity().mVec128));
  __m128 deltaVel2Dotn    =       _mm_sub_ps(_vmathVfDot3(c.m_relpos2CrossNormal.mVec128,body2.internalGetTurnVelocity().mVec128),_vmathVfDot3((c.m_contactNormal).mVec128,body2.internalGetPushVelocity().mVec128));
  deltaImpulse    =       _mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel1Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
  deltaImpulse    =       _mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel2Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
  btSimdScalar sum = _mm_add_ps(cpAppliedImp,deltaImpulse);
  btSimdScalar resultLowerLess,resultUpperLess;
  resultLowerLess = _mm_cmplt_ps(sum,lowerLimit1);
  resultUpperLess = _mm_cmplt_ps(sum,upperLimit1);
  __m128 lowMinApplied = _mm_sub_ps(lowerLimit1,cpAppliedImp);
  deltaImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse) );
  c.m_appliedImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum) );
  __m128  linearComponentA = _mm_mul_ps(c.m_contactNormal.mVec128,body1.internalGetInvMass().mVec128);
  __m128  linearComponentB = _mm_mul_ps((c.m_contactNormal).mVec128,body2.internalGetInvMass().mVec128);
  __m128 impulseMagnitude = deltaImpulse;
  body1.internalGetPushVelocity().mVec128 = _mm_add_ps(body1.internalGetPushVelocity().mVec128,_mm_mul_ps(linearComponentA,impulseMagnitude));
  body1.internalGetTurnVelocity().mVec128 = _mm_add_ps(body1.internalGetTurnVelocity().mVec128 ,_mm_mul_ps(c.m_angularComponentA.mVec128,impulseMagnitude));
  body2.internalGetPushVelocity().mVec128 = _mm_sub_ps(body2.internalGetPushVelocity().mVec128,_mm_mul_ps(linearComponentB,impulseMagnitude));
  body2.internalGetTurnVelocity().mVec128 = _mm_add_ps(body2.internalGetTurnVelocity().mVec128 ,_mm_mul_ps(c.m_angularComponentB.mVec128,impulseMagnitude));
{$else}
  resolveSplitPenetrationImpulseCacheFriendly(body1,body2,contactConstraint);
{$endif}
end;

procedure btSequentialImpulseConstraintSolver.resolveSplitPenetrationImpulseCacheFriendly(const body1, body2: btRigidBody; const contactConstraint: PbtRSolverConstraint);
var sum,deltaImpulse,deltaVel2Dotn,deltaVel1Dotn : btScalar;
begin
  if contactConstraint^.m_rhsPenetration<>0 then begin
    inc(gNumSplitImpulseRecoveries);
    deltaImpulse  := contactConstraint^.m_rhsPenetration-btScalar(contactConstraint^.m_appliedPushImpulse)*contactConstraint^.m_cfm;
    deltaVel1Dotn := contactConstraint^.m_contactNormal.dot(body1.internalGetPushVelocityP^)  + contactConstraint^.m_relpos1CrossNormal.dot(body1.internalGetTurnVelocityP^);
    deltaVel2Dotn := -contactConstraint^.m_contactNormal.dot(body2.internalGetPushVelocityP^) + contactConstraint^.m_relpos2CrossNormal.dot(body2.internalGetTurnVelocityP^);

    deltaImpulse    -=      deltaVel1Dotn*contactConstraint^.m_jacDiagABInv;
    deltaImpulse    -=      deltaVel2Dotn*contactConstraint^.m_jacDiagABInv;
    sum             := btScalar(contactConstraint^.m_appliedPushImpulse) + deltaImpulse;
    if sum < contactConstraint^.m_lowerLimit then begin
      deltaImpulse  := contactConstraint^.m_lowerLimit-contactConstraint^.m_appliedPushImpulse;
      contactConstraint^.m_appliedPushImpulse := contactConstraint^.m_lowerLimit;
    end else begin
      contactConstraint^.m_appliedPushImpulse := sum;
    end;
    body1.internalApplyPushImpulse(contactConstraint^.m_contactNormal*body1.internalGetInvMassP^,contactConstraint^.m_angularComponentA,deltaImpulse);
    body2.internalApplyPushImpulse(-contactConstraint^.m_contactNormal*body2.internalGetInvMassP^,contactConstraint^.m_angularComponentB,deltaImpulse);
  end;
end;

//function btSequentialImpulseConstraintSolver.getOrInitSolverBody(const body: btCollisionObject): integer;
//begin
// return 0;
//end;

procedure btSequentialImpulseConstraintSolver.resolveSingleConstraintRowGeneric(const body1, body2: btRigidBody; const contactConstraint: PbtRSolverConstraint);
var deltaImpulse,deltaVel1Dotn,deltaVel2Dotn,sum: btScalar;
begin
  deltaImpulse   := contactConstraint^.m_rhs-btScalar(contactConstraint^.m_appliedImpulse)*contactConstraint^.m_cfm;
  deltaVel1Dotn  := contactConstraint^.m_contactNormal.dot(body1.internalGetDeltaLinearVelocityP^)   + contactConstraint^.m_relpos1CrossNormal.dot(body1.internalGetDeltaAngularVelocityP^);
  deltaVel2Dotn  := -contactConstraint^.m_contactNormal.dot(body2.internalGetDeltaLinearVelocityP^) + contactConstraint^.m_relpos2CrossNormal.dot(body2.internalGetDeltaAngularVelocityP^);

  //      const btScalar delta_rel_vel    =       deltaVel1Dotn-deltaVel2Dotn;
  deltaImpulse   -=      deltaVel1Dotn*contactConstraint^.m_jacDiagABInv;
  deltaImpulse   -=      deltaVel2Dotn*contactConstraint^.m_jacDiagABInv;

  sum            := btScalar(contactConstraint^.m_appliedImpulse) + deltaImpulse;
  if sum < contactConstraint^.m_lowerLimit then begin
    deltaImpulse       := contactConstraint^.m_lowerLimit-contactConstraint^.m_appliedImpulse;
    contactConstraint^.m_appliedImpulse := contactConstraint^.m_lowerLimit;
  end else
  if sum > contactConstraint^.m_upperLimit then begin
    deltaImpulse       := contactConstraint^.m_upperLimit-contactConstraint^.m_appliedImpulse;
    contactConstraint^.m_appliedImpulse := contactConstraint^.m_upperLimit;
  end else begin
    contactConstraint^.m_appliedImpulse := sum;
  end;
  body1.internalApplyImpulse(contactConstraint^.m_contactNormal*body1.internalGetInvMassP^,contactConstraint^.m_angularComponentA,deltaImpulse);
  body2.internalApplyImpulse(-contactConstraint^.m_contactNormal*body2.internalGetInvMassP^,contactConstraint^.m_angularComponentB,deltaImpulse);
end;

procedure btSequentialImpulseConstraintSolver.resolveSingleConstraintRowGenericSIMD(const body1, body2: btRigidBody; const contactConstraint: PbtRSolverConstraint);
begin
  {$ifdef USE_SIMD}
        __m128 cpAppliedImp = _mm_set1_ps(contactConstraint.m_appliedImpulse);
        __m128  lowerLimit1 = _mm_set1_ps(contactConstraint.m_lowerLimit);
        __m128  upperLimit1 = _mm_set1_ps(contactConstraint.m_upperLimit);
        __m128 deltaImpulse = _mm_sub_ps(_mm_set1_ps(contactConstraint.m_rhs), _mm_mul_ps(_mm_set1_ps(contactConstraint.m_appliedImpulse),_mm_set1_ps(contactConstraint.m_cfm)));
        __m128 deltaVel1Dotn    =       _mm_add_ps(_vmathVfDot3(contactConstraint.m_contactNormal.mVec128,body1.internalGetDeltaLinearVelocity().mVec128), _vmathVfDot3(contactConstraint.m_relpos1CrossNormal.mVec128,body1.internalGetDeltaAngularVelocity().mVec128));
        __m128 deltaVel2Dotn    =       _mm_sub_ps(_vmathVfDot3(contactConstraint.m_relpos2CrossNormal.mVec128,body2.internalGetDeltaAngularVelocity().mVec128),_vmathVfDot3((contactConstraint.m_contactNormal).mVec128,body2.internalGetDeltaLinearVelocity().mVec128));
        deltaImpulse    =       _mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel1Dotn,_mm_set1_ps(contactConstraint.m_jacDiagABInv)));
        deltaImpulse    =       _mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel2Dotn,_mm_set1_ps(contactConstraint.m_jacDiagABInv)));
        btSimdScalar sum = _mm_add_ps(cpAppliedImp,deltaImpulse);
        btSimdScalar resultLowerLess,resultUpperLess;
        resultLowerLess = _mm_cmplt_ps(sum,lowerLimit1);
        resultUpperLess = _mm_cmplt_ps(sum,upperLimit1);
        __m128 lowMinApplied = _mm_sub_ps(lowerLimit1,cpAppliedImp);
        deltaImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse) );
        contactConstraint.m_appliedImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum) );
        __m128 upperMinApplied = _mm_sub_ps(upperLimit1,cpAppliedImp);
        deltaImpulse = _mm_or_ps( _mm_and_ps(resultUpperLess, deltaImpulse), _mm_andnot_ps(resultUpperLess, upperMinApplied) );
        contactConstraint.m_appliedImpulse = _mm_or_ps( _mm_and_ps(resultUpperLess, contactConstraint.m_appliedImpulse), _mm_andnot_ps(resultUpperLess, upperLimit1) );
        __m128  linearComponentA = _mm_mul_ps(contactConstraint.m_contactNormal.mVec128,body1.internalGetInvMass().mVec128);
        __m128  linearComponentB = _mm_mul_ps((contactConstraint.m_contactNormal).mVec128,body2.internalGetInvMass().mVec128);
        __m128 impulseMagnitude = deltaImpulse;
        body1.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(body1.internalGetDeltaLinearVelocity().mVec128,_mm_mul_ps(linearComponentA,impulseMagnitude));
        body1.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(body1.internalGetDeltaAngularVelocity().mVec128 ,_mm_mul_ps(contactConstraint.m_angularComponentA.mVec128,impulseMagnitude));
        body2.internalGetDeltaLinearVelocity().mVec128 = _mm_sub_ps(body2.internalGetDeltaLinearVelocity().mVec128,_mm_mul_ps(linearComponentB,impulseMagnitude));
        body2.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(body2.internalGetDeltaAngularVelocity().mVec128 ,_mm_mul_ps(contactConstraint.m_angularComponentB.mVec128,impulseMagnitude));
  {$else}
    resolveSingleConstraintRowGeneric(body1,body2,contactConstraint);
  {$endif}
end;

procedure btSequentialImpulseConstraintSolver.resolveSingleConstraintRowLowerLimit(const body1, body2: btRigidBody; const contactConstraint: PbtRSolverConstraint);
var sum,deltaImpulse,deltaVel1Dotn,deltaVel2Dotn : btScalar;
begin
  deltaImpulse   := contactConstraint^.m_rhs-btScalar(contactConstraint^.m_appliedImpulse)*contactConstraint^.m_cfm;
  deltaVel1Dotn  :=       contactConstraint^.m_contactNormal.dot(body1.internalGetDeltaLinearVelocityP^)   + contactConstraint^.m_relpos1CrossNormal.dot(body1.internalGetDeltaAngularVelocityP^);
  deltaVel2Dotn  :=       -contactConstraint^.m_contactNormal.dot(body2.internalGetDeltaLinearVelocityP^) + contactConstraint^.m_relpos2CrossNormal.dot(body2.internalGetDeltaAngularVelocityP^);

  deltaImpulse   -=      deltaVel1Dotn*contactConstraint^.m_jacDiagABInv;
  deltaImpulse   -=      deltaVel2Dotn*contactConstraint^.m_jacDiagABInv;
  sum            := btScalar(contactConstraint^.m_appliedImpulse) + deltaImpulse;
  if sum < contactConstraint^.m_lowerLimit then begin
    deltaImpulse := contactConstraint^.m_lowerLimit-contactConstraint^.m_appliedImpulse;
    contactConstraint^.m_appliedImpulse := contactConstraint^.m_lowerLimit;
  end else begin
    contactConstraint^.m_appliedImpulse := sum;
  end;
  body1.internalApplyImpulse(contactConstraint^.m_contactNormal*body1.internalGetInvMassP^,contactConstraint^.m_angularComponentA,deltaImpulse);
  body2.internalApplyImpulse(-contactConstraint^.m_contactNormal*body2.internalGetInvMassP^,contactConstraint^.m_angularComponentB,deltaImpulse);
end;

procedure btSequentialImpulseConstraintSolver.resolveSingleConstraintRowLowerLimitSIMD(const body1, body2: btRigidBody; const contactConstraint: PbtRSolverConstraint);
begin
{$ifdef USE_SIMD}
  __m128 cpAppliedImp = _mm_set1_ps(contactConstraint.m_appliedImpulse);
  __m128  lowerLimit1 = _mm_set1_ps(contactConstraint.m_lowerLimit);
  __m128  upperLimit1 = _mm_set1_ps(contactConstraint.m_upperLimit);
  __m128 deltaImpulse = _mm_sub_ps(_mm_set1_ps(contactConstraint.m_rhs), _mm_mul_ps(_mm_set1_ps(contactConstraint.m_appliedImpulse),_mm_set1_ps(contactConstraint.m_cfm)));
  __m128 deltaVel1Dotn    =       _mm_add_ps(_vmathVfDot3(contactConstraint.m_contactNormal.mVec128,body1.internalGetDeltaLinearVelocity().mVec128), _vmathVfDot3(contactConstraint.m_relpos1CrossNormal.mVec128,body1.internalGetDeltaAngularVelocity().mVec128));
  __m128 deltaVel2Dotn    =       _mm_sub_ps(_vmathVfDot3(contactConstraint.m_relpos2CrossNormal.mVec128,body2.internalGetDeltaAngularVelocity().mVec128),_vmathVfDot3((contactConstraint.m_contactNormal).mVec128,body2.internalGetDeltaLinearVelocity().mVec128));
  deltaImpulse    =       _mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel1Dotn,_mm_set1_ps(contactConstraint.m_jacDiagABInv)));
  deltaImpulse    =       _mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel2Dotn,_mm_set1_ps(contactConstraint.m_jacDiagABInv)));
  btSimdScalar sum = _mm_add_ps(cpAppliedImp,deltaImpulse);
  btSimdScalar resultLowerLess,resultUpperLess;
  resultLowerLess = _mm_cmplt_ps(sum,lowerLimit1);
  resultUpperLess = _mm_cmplt_ps(sum,upperLimit1);
  __m128 lowMinApplied = _mm_sub_ps(lowerLimit1,cpAppliedImp);
  deltaImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse) );
  contactConstraint.m_appliedImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum) );
  __m128  linearComponentA = _mm_mul_ps(contactConstraint.m_contactNormal.mVec128,body1.internalGetInvMass().mVec128);
  __m128  linearComponentB = _mm_mul_ps((contactConstraint.m_contactNormal).mVec128,body2.internalGetInvMass().mVec128);
  __m128 impulseMagnitude = deltaImpulse;
  body1.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(body1.internalGetDeltaLinearVelocity().mVec128,_mm_mul_ps(linearComponentA,impulseMagnitude));
  body1.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(body1.internalGetDeltaAngularVelocity().mVec128 ,_mm_mul_ps(contactConstraint.m_angularComponentA.mVec128,impulseMagnitude));
  body2.internalGetDeltaLinearVelocity().mVec128 = _mm_sub_ps(body2.internalGetDeltaLinearVelocity().mVec128,_mm_mul_ps(linearComponentB,impulseMagnitude));
  body2.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(body2.internalGetDeltaAngularVelocity().mVec128 ,_mm_mul_ps(contactConstraint.m_angularComponentB.mVec128,impulseMagnitude));
{$else}
  resolveSingleConstraintRowLowerLimit(body1,body2,contactConstraint);
{$endif}
end;

class function btSequentialImpulseConstraintSolver.getFixedBody: btRigidBody;
var s_fixed:btRigidBody;
begin
 s_fixed := btRigidBody.create(0,nil,nil,cbtNullVector);
 s_fixed.setMassProps(0,cbtNullVector);
 Result := s_fixed;
end;

procedure btSequentialImpulseConstraintSolver.solveGroupCacheFriendlySplitImpulseIterations ;
//(const bodies: btCollisionObjectArr; const numBodies: integer; const manifoldPtr: btPersistentManifoldArr;
//  const numManifolds: integer; const constraints: btTypedConstraintArr; const numConstraints: integer; const infoGlobal: btContactSolverInfo; const debugDrawer: btIDebugDraw;
//  const stackAlloc: btStackAllocator);
var solveManifold       : PbtRSolverConstraint;
     iteration,j,
     numPoolConstraints : integer;

begin
  if ms_info.m_splitImpulse then begin
    if SOLVER_SIMD in ms_info.m_solverMode then begin
      for iteration := 0 to ms_info.m_numIterations-1 do begin
        numPoolConstraints := m_tmpSolverContactConstraintPool.size-1;
        for j:=0 to numPoolConstraints do begin
          solveManifold := m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]^];
          resolveSplitPenetrationSIMD(solveManifold^.m_solverBodyA,solveManifold^.m_solverBodyB,solveManifold);
        end;
      end;
    end else begin
      for iteration := 0 to ms_info.m_numIterations-1 do begin
        numPoolConstraints := m_tmpSolverContactConstraintPool.size-1;
        for j:=0 to numPoolConstraints do begin
          solveManifold := m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]^];
          resolveSplitPenetrationImpulseCacheFriendly(solveManifold^.m_solverBodyA,solveManifold^.m_solverBodyB,solveManifold);
        end;
      end;
    end;
  end;
end;

procedure btSequentialImpulseConstraintSolver.solveGroupCacheFriendlyFinish ;
var i,j,numPoolConstraints: integer;
    solveManifold,
    solverConstr  : PbtRSolverConstraint;
    constr        : btTypedConstraint;
    pt            : PbtOManifoldPoint;
    sum           : btScalar;
    body          : btRigidBody;
begin
  numPoolConstraints := m_tmpSolverContactConstraintPool.size;
  for j:=0 to numPoolConstraints-1 do begin
    solveManifold := m_tmpSolverContactConstraintPool[j];
    pt := PbtOManifoldPoint(solveManifold^.m_originalContactPoint);
    btAssert(assigned(pt));
    pt^.m_appliedImpulse := solveManifold^.m_appliedImpulse;
    if SOLVER_USE_FRICTION_WARMSTARTING in ms_info.m_solverMode then begin
      pt^.m_appliedImpulseLateral1 := m_tmpSolverContactFrictionConstraintPool[solveManifold^.m_frictionIndex]^.m_appliedImpulse;
      pt^.m_appliedImpulseLateral2 := m_tmpSolverContactFrictionConstraintPool[solveManifold^.m_frictionIndex+1]^.m_appliedImpulse;
    end;
    //do a callback here?
  end;
  numPoolConstraints := m_tmpSolverNonContactConstraintPool.size;
  for j:=0 to numPoolConstraints-1 do begin
    solverConstr := m_tmpSolverNonContactConstraintPool[j];
    constr := btTypedConstraint(solverConstr^.m_originalContactPoint);
    sum    := constr.internalGetAppliedImpulse;
    sum    += solverConstr^.m_appliedImpulse;
    constr.internalSetAppliedImpulse(sum);
  end;
  if ms_info.m_splitImpulse then begin
    for i:=0 to ms_numBodies-1 do begin
      body := btRigidBody.upcast(ms_bodies[i]^);
      if assigned(body) then begin
        body.internalWritebackVelocity(ms_info.m_timeStep);
      end;
    end;
  end else begin
    for i:=0 to ms_numBodies-1 do begin
      body := btRigidBody.upcast(ms_bodies[i]^);
      if assigned(body) then begin
        body.internalWritebackVelocity;
      end;
    end;
  end;
  m_tmpSolverContactConstraintPool.resize(0);
  m_tmpSolverNonContactConstraintPool.resize(0);
  m_tmpSolverContactFrictionConstraintPool.resize(0);
end;

procedure btSequentialImpulseConstraintSolver.solveSingleIteration(const iteration:integer);
var tmp,swapi,j,numPool : integer;
    constraint          : PbtRSolverConstraint;
    totalImpulse        : btScalar;
begin
  if SOLVER_RANDMIZE_ORDER in ms_info.m_solverMode then begin
    if (iteration and 7) = 0 then begin
      numPool := m_tmpSolverContactConstraintPool.size-1;
      for j:=0 to numPool do begin
        tmp                                := m_orderTmpConstraintPool.A[j]^;
        swapi                              := btRandInt2(j+1);
        m_orderTmpConstraintPool.A[j]^     := m_orderTmpConstraintPool.A[swapi]^;
        m_orderTmpConstraintPool.A[swapi]^ := tmp;
      end;
      numPool   := m_tmpSolverContactFrictionConstraintPool.size-1;
      for j:=0 to numPool do begin
        tmp                                      := m_orderFrictionConstraintPool.A[j]^;
        swapi                                    := btRandInt2(j+1);
        m_orderFrictionConstraintPool.A[j]^      := m_orderFrictionConstraintPool.A[swapi]^;
        m_orderFrictionConstraintPool.A[swapi]^ := tmp;
      end;
    end;
  end;
  if SOLVER_SIMD in ms_info.m_solverMode then begin
    ///solve all joint constraints, using SIMD, if available
    numPool := m_tmpSolverNonContactConstraintPool.size-1;
    for j:=0 to numPool do begin
      constraint := m_tmpSolverNonContactConstraintPool[j];
      resolveSingleConstraintRowGenericSIMD(constraint^.m_solverBodyA,constraint^.m_solverBodyB,constraint);
    end;
    for j:=ms_startConstraint to ms_numConstraints-1 do begin
      ms_constraints[j]^.solveConstraintObsolete(ms_constraints[j]^.getRigidBodyA,ms_constraints[j]^.getRigidBodyB,ms_info.m_timeStep);
    end;
    ///solve all contact constraints using SIMD, if available
    numPool := m_tmpSolverContactConstraintPool.size-1;
    for j:=0 to numPool do begin
      constraint := m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool.A[j]^];
      resolveSingleConstraintRowLowerLimitSIMD(constraint^.m_solverBodyA,constraint^.m_solverBodyB,constraint);
    end;
    ///solve all friction constraints, using SIMD, if available
    numPool := m_tmpSolverContactFrictionConstraintPool.size-1;
    for j:=0 to numPool do begin
      constraint   := m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool.A[j]^];
      totalImpulse := m_tmpSolverContactConstraintPool.A[constraint^.m_frictionIndex]^.m_appliedImpulse;
      if totalImpulse>0 then begin
        constraint^.m_lowerLimit := -(constraint^.m_friction*totalImpulse);
        constraint^.m_upperLimit := constraint^.m_friction*totalImpulse;
        resolveSingleConstraintRowGenericSIMD(constraint^.m_solverBodyA,constraint^.m_solverBodyB,constraint);
      end;
    end;
  end else begin
    ///solve all joint constraints
    numPool:=m_tmpSolverNonContactConstraintPool.size-1;
    for j:=0 to numPool do begin
      constraint := m_tmpSolverNonContactConstraintPool[j];
      resolveSingleConstraintRowGeneric(constraint^.m_solverBodyA,constraint^.m_solverBodyB,constraint);
    end;
    for j:=ms_startConstraint to ms_numConstraints-1 do begin
      ms_constraints[j]^.solveConstraintObsolete(ms_constraints[j]^.getRigidBodyA,ms_constraints[j]^.getRigidBodyB,ms_info.m_timeStep);
    end;
    ///solve all contact constraints
    numPool := m_tmpSolverContactConstraintPool.size-1;
    for j:=0 to numPool do begin
      constraint := m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]^];
      resolveSingleConstraintRowLowerLimit(constraint^.m_solverBodyA,constraint^.m_solverBodyB,constraint);
    end;
    ///solve all friction constraints
    numPool := m_tmpSolverContactFrictionConstraintPool.size-1;
    for j:=0 to numPool do begin
      constraint := m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[j]^];
      totalImpulse := m_tmpSolverContactConstraintPool[constraint^.m_frictionIndex]^.m_appliedImpulse;
      if totalImpulse>0 then begin
        constraint^.m_lowerLimit := -(constraint^.m_friction*totalImpulse);
        constraint^.m_upperLimit := constraint^.m_friction*totalImpulse;
        resolveSingleConstraintRowGeneric(constraint^.m_solverBodyA,constraint^.m_solverBodyB,constraint);
      end;
    end
  end;
end;

procedure btSequentialImpulseConstraintSolver.solveGroupCacheFriendlySetup ;
//  (const bodies: btCollisionObjectArr; const numBodies: integer; const manifoldPtr: btPersistentManifoldArr;
//  const numManifolds: integer; const constraints: btTypedConstraintArr; const numConstraints: integer; const infoGlobal: btContactSolverInfo; const debugDrawer: btIDebugDraw;
//  const stackAlloc: btStackAllocator): btScalar;
var  i,j          : Integer;
     body,rbA,rbB : btRigidBody;
     constraint   : btTypedConstraint;
     totalNumRows,
     currentRow   : Integer;
     info1        : PbtTC_ConstraintInfo1;
     info2        : btTC_ConstraintInfo2;
     currentConstraintRow,
     solverConstraint          : PbtRSolverConstraint;
     ftorqueAxis1,ftorqueAxis2 : PbtVector3;
     iMJlA,iMJaA,iMJlB,iMJaB   : btVector3;
     sum                       : btScalar;
     rel_vel,vel1Dotn,vel2Dotn : btScalar;
     restitution,positionalError,velocityError,penetrationImpulse,velocityImpulse : btScalar;
     manifold : btPersistentManifold;
//     info     : btContactSolverInfo;
     numConstraintPool,numFrictionPool : integer;
begin
  //.
  BT_PROFILE("solveGroupCacheFriendlySetup");
  if (ms_numConstraints + ms_numManifolds)=0 then begin // DONE IN ASSERTION
    exit;
  end;
  if ms_info.m_splitImpulse then begin
    for i := 0 to ms_numBodies-1 do begin
      body := btRigidBody.upcast(ms_bodies[i]^);
      if assigned(body) then begin
        body.internalGetDeltaLinearVelocityP^.zero;
        body.internalGetDeltaAngularVelocityP^.zero;
        body.internalGetPushVelocityP^.zero;
        body.internalGetTurnVelocityP^.zero;
      end;
    end;
  end else begin
    for i := 0 to ms_numBodies-1 do begin
      body := btRigidBody.upcast(ms_bodies[i]^);
      if assigned(body) then begin
        body.internalGetDeltaLinearVelocityP^.zero;
        body.internalGetDeltaAngularVelocityP^.zero;
      end;
    end;
  end;
//  if true then begin
    for i:=ms_startConstraint  to ms_numConstraints-1 do begin
      ms_constraints[i]^.buildJacobian;
    end;
//  end;
//  if true then
  begin
    totalNumRows := 0;
    m_tmpConstraintSizesPool.resize(ms_numConstraints);
    //calculate the total number of contraint rows
    for i:=ms_startConstraint to ms_numConstraints-1 do begin
      info1 := m_tmpConstraintSizesPool[i];
      ms_constraints[i]^.getInfo1(info1^);
      totalNumRows += info1^.m_numConstraintRows;
    end;
    m_tmpSolverNonContactConstraintPool.resize(totalNumRows);
    ///setup the btSolverConstraints
    currentRow := 0;
    for i := ms_startConstraint to ms_numConstraints-1 do begin
      info1 := m_tmpConstraintSizesPool[i];
      if info1^.m_numConstraintRows<>0 then begin
        btAssert(currentRow<totalNumRows);
        currentConstraintRow := m_tmpSolverNonContactConstraintPool[currentRow];
        constraint := ms_constraints[i]^;
        rbA := constraint.getRigidBodyA;
        rbB := constraint.getRigidBodyB;
        for j := 0 to info1^.m_numConstraintRows-1 do begin
          //memset(&currentConstraintRow[j],0,sizeof(btRSolverConstraint));
          FillByte(currentConstraintRow[j],sizeof(btRSolverConstraint),0);
          currentConstraintRow[j].m_lowerLimit         := -SIMD_INFINITY;
          currentConstraintRow[j].m_upperLimit         := SIMD_INFINITY;
          currentConstraintRow[j].m_appliedImpulse     := 0;
          currentConstraintRow[j].m_appliedPushImpulse := 0;
          currentConstraintRow[j].m_solverBodyA        := rbA;
          currentConstraintRow[j].m_solverBodyB        := rbB;
        end;
        rbA.internalGetDeltaLinearVelocityP^.zero;
        rbA.internalGetDeltaAngularVelocityP^.zero;
        rbB.internalGetDeltaLinearVelocityP^.zero;
        rbB.internalGetDeltaAngularVelocityP^.zero;
        info2.fps             := 1/ms_info.m_timeStep;
        info2.erp             := ms_info.m_erp;
        info2.m_J1linearAxis  := currentConstraintRow^.m_contactNormal._X;     //FOS : First element
        info2.m_J1angularAxis := currentConstraintRow^.m_relpos1CrossNormal._X;
        info2.m_J2linearAxis  := nil;
        info2.m_J2angularAxis := currentConstraintRow^.m_relpos2CrossNormal._X;
        info2.rowskip         := sizeof(btRSolverConstraint) div sizeof(btScalar);//check this
        ///the size of btRSolverConstraint needs be a multiple of btScalar
        btAssert(info2.rowskip*sizeof(btScalar)=sizeof(btRSolverConstraint));
        abort; // FOSTODO ABOVE
        info2.m_constraintError    := @currentConstraintRow^.m_rhs;
        currentConstraintRow^.m_cfm := ms_info.m_globalCfm;
        info2.m_damping            := ms_info.m_damping;
        info2.cfm                  := @currentConstraintRow^.m_cfm;
        info2.m_lowerLimit         := @currentConstraintRow^.m_lowerLimit;
        info2.m_upperLimit         := @currentConstraintRow^.m_upperLimit;
        info2.m_numIterations      := ms_info.m_numIterations;
        ms_constraints.A[i]^.getInfo2(info2);

        ///finalize the constraint setup
        for j := 0 to info1^.m_numConstraintRows-1 do begin
          solverConstraint := @currentConstraintRow[j];
          solverConstraint^.m_originalContactPoint := constraint; //ALERT

          ftorqueAxis1 := @solverConstraint^.m_relpos1CrossNormal;
          solverConstraint^.m_angularComponentA := constraint.getRigidBodyA.getInvInertiaTensorWorldP^*ftorqueAxis1^*constraint.getRigidBodyA.getAngularFactorP^;
          ftorqueAxis2 := @solverConstraint^.m_relpos2CrossNormal;
          solverConstraint^.m_angularComponentB := constraint.getRigidBodyB.getInvInertiaTensorWorldP^*ftorqueAxis2^*constraint.getRigidBodyB.getAngularFactorP^;

          iMJlA := solverConstraint^.m_contactNormal*rbA.getInvMass;
          iMJaA := rbA.getInvInertiaTensorWorldP^*solverConstraint^.m_relpos1CrossNormal;
          iMJlB := solverConstraint^.m_contactNormal*rbB.getInvMass;//sign of normal?
          iMJaB := rbB.getInvInertiaTensorWorldP^*solverConstraint^.m_relpos2CrossNormal;

          sum := iMJlA.dot(solverConstraint^.m_contactNormal);
          sum += iMJaA.dot(solverConstraint^.m_relpos1CrossNormal);
          sum += iMJlB.dot(solverConstraint^.m_contactNormal);
          sum += iMJaB.dot(solverConstraint^.m_relpos2CrossNormal);

          solverConstraint^.m_jacDiagABInv := 1/sum;
          ///fix rhs
          ///todo: add force/torque accelerators
          begin
            vel1Dotn := solverConstraint^.m_contactNormal.dot(rbA.getLinearVelocityP^) + solverConstraint^.m_relpos1CrossNormal.dot(rbA.getAngularVelocityP^);
            vel2Dotn := -solverConstraint^.m_contactNormal.dot(rbB.getLinearVelocityP^) + solverConstraint^.m_relpos2CrossNormal.dot(rbB.getAngularVelocityP^);
            rel_vel  := vel1Dotn+vel2Dotn;
            restitution     := 0;
            positionalError := solverConstraint^.m_rhs;//already filled in by getConstraintInfo2
            velocityError   := restitution - rel_vel * info2.m_damping;
            penetrationImpulse := positionalError*solverConstraint^.m_jacDiagABInv;
            velocityImpulse    := velocityError *solverConstraint^.m_jacDiagABInv;
            solverConstraint^.m_rhs := penetrationImpulse+velocityImpulse;
            solverConstraint^.m_appliedImpulse := 0;
          end;
        end;
      end;
      currentRow := currentRow + m_tmpConstraintSizesPool[i]^.m_numConstraintRows;
    end;
    for i :=ms_startManifold to ms_numManifolds-1 do begin
      manifold := ms_manifoldPtr[i]^;
      convertContact(manifold,ms_info);
    end;
  end;
//  info := infoGlobal;
  numConstraintPool := m_tmpSolverContactConstraintPool.size();
  numFrictionPool   := m_tmpSolverContactFrictionConstraintPool.size();
  ///@todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
  m_orderTmpConstraintPool.resize(numConstraintPool);
  m_orderFrictionConstraintPool.resize(numFrictionPool);
  begin
    for i :=0 to numConstraintPool-1 do begin
      m_orderTmpConstraintPool.A[i]^ := i;
    end;
    for i := 0 to numFrictionPool-1 do begin
      m_orderFrictionConstraintPool.A[i]^ := i;
    end;
  end;
end;

procedure btSequentialImpulseConstraintSolver.solveGroupCacheFriendlyIterations;
 //(const bodies: btCollisionObjectArr; const numBodies: integer; const manifoldPtr: btPersistentManifoldArr;
 // const numManifolds: integer; const constraints: btTypedConstraintArr; const numConstraints: integer; const infoGlobal: btContactSolverInfo; const debugDrawer: btIDebugDraw;
 // const stackAlloc: btStackAllocator): btScalar;
var  iteration : integer;
begin
 //.
  BT_PROFILE("solveGroupCacheFriendlyIterations");
  //should traverse the contacts random order...
  for iteration := 0 to ms_info.m_numIterations-1 do begin
    solveSingleIteration (iteration); // (iteration, bodies ,numBodies,manifoldPtr, numManifolds,constraints,numConstraints,infoGlobal,debugDrawer,stackAlloc);
  end;
  solveGroupCacheFriendlySplitImpulseIterations; // (bodies ,numBodies,manifoldPtr, numManifolds,constraints,numConstraints,infoGlobal,debugDrawer,stackAlloc);
end;

constructor btSequentialImpulseConstraintSolver.create;
begin
  Inherited Create;
  m_btSeed2 := 0;

  m_tmpSolverContactConstraintPool         := btConstraintArray.create;
  m_tmpSolverNonContactConstraintPool      := btConstraintArray.create;
  m_tmpSolverContactFrictionConstraintPool := btConstraintArray.create;
  m_orderTmpConstraintPool                 := btFOSAlignedIntegers.create;
  m_orderFrictionConstraintPool            := btFOSAlignedIntegers.create;
  m_tmpConstraintSizesPool                 := btTC_ConstraintInfo1Array.create;
end;

destructor btSequentialImpulseConstraintSolver.destroy;
begin
 m_tmpSolverContactConstraintPool.Free;
 m_tmpSolverNonContactConstraintPool.Free;
 m_tmpSolverContactFrictionConstraintPool.Free;
 m_orderTmpConstraintPool.Free;
 m_orderFrictionConstraintPool.Free;
 m_tmpConstraintSizesPool.Free;
 inherited destroy;
end;

function btSequentialImpulseConstraintSolver.solveGroup(const bodies: btCollisionObjectArray; const numBodies: integer; const manifoldPtr: btManifoldArray; const startManifold,numManifolds: integer;  const constraints: btTypedConstraintArray; const startConstraint,numConstraints: integer; const info: btContactSolverInfo; const debugDrawer: btIDebugDraw; const stackAlloc: btStackAllocator; const disp: btDispatcher): btScalar;
//var i: Integer;
begin
 //.
  BT_PROFILE("solveGroup");
  //you need to provide at least some bodies
  btAssert(bodies<>nil);
  btAssert(numBodies<>0);

  {$IFDEF FOS_DYNAMICS_DEBUG}
  writeln('-> enter SolveGroup');
  for i:=0 to numBodies-1 do begin
    fdb_pbody(btRigidBody(bodies^[i]^),i);
  end;
  for i:=0 to numManifolds-1 do begin
    fdb_pmanifold(manifoldPtr^[i]^);
  end;
  {$ENDIF}

  ms_bodies          := bodies;
  ms_numBodies       := numBodies;
  ms_manifoldPtr     := manifoldPtr;
  ms_startManifold   := startManifold;
  ms_numManifolds    := numManifolds;
  ms_constraints     := constraints;
  ms_startConstraint := startConstraint;
  ms_numConstraints  := numConstraints;
  ms_info            := info;
  ms_debugDrawer     := debugDrawer;
  ms_stackAlloc      := stackAlloc;
  ms_disp            := disp;

  solveGroupCacheFriendlySetup      ;
  solveGroupCacheFriendlyIterations ;
  solveGroupCacheFriendlyFinish     ;

  {$IFDEF FOS_DYNAMICS_DEBUG}
  writeln('-- after --');
  for i:=0 to numBodies-1 do begin
    fdb_pbody(btRigidBody(bodies^[i]^),i);
  end;
  writeln;
  writeln('leave SolveGroup');
  {$ENDIF}
  Result := 0;
end;

procedure btSequentialImpulseConstraintSolver.reset;
begin
  m_btSeed2:=0;
end;

function btSequentialImpulseConstraintSolver.btRand2: cardinal;
begin
 abort;
 m_btSeed2 := (1664525*m_btSeed2 + 1013904223) and $ffffffff;
 Result    := m_btSeed2;
end;

//See ODE: adam's all-int straightforward(?) dRandInt (0..n-1)
function btSequentialImpulseConstraintSolver.btRandInt2(const n: integer): integer;
var un,r : cardinal;
begin
 // seems good; xor-fold and modulus
 abort;
  un := cardinal(n);
  r  := btRand2;
 // note: probably more aggressive than it needs to be -- might be
 //       able to get away without one or two of the innermost branches.
  if un <= $00010000 then begin
    r := r xor  SarLongint(r,16);
    if un <= $00000100 then begin
      r := r xor SarLongint(r,8);
      if un <= $00000010 then begin
        r := r xor SarLongint(r,4);
        if un <= $00000004 then begin
          r := r xor SarLongint(r,2);
          if un <= $00000002 then begin
            r := r xor SarLongint(r,1);
          end;
        end;
      end;
    end;
  end;
  Result := integer (r mod un);
end;

procedure btSequentialImpulseConstraintSolver.setRandSeed(const seed: cardinal);
begin
 abort;
 m_btSeed2 := seed;
end;

function btSequentialImpulseConstraintSolver.getRandSeed: Cardinal;
begin
 abort;
  result := m_btSeed2;
end;

{ btUnionFind }

constructor btUnionFind.create;
begin
  m_elements := btElementArray.create;
end;

destructor btUnionFind.destroy;
begin
  m_elements.Free;
  inherited destroy;
end;

function btUnionFindElementSortPredicate(const lhs,rhs:btElement):nativeint;
begin
  Result := lhs.m_id - rhs.m_id;
  //Result := lhs.m_id < rhs.m_id;
end;

///this is a special operation, destroying the content of btUnionFind.
///it sorts the elements, based on island id, in order to make it easy to iterate over islands

procedure btUnionFind.sortIslands;
var numElements: LongInt;
  i: Integer;
begin
  //first store the original body index, and islandId
  numElements := m_elements.size;
  for i:=0 to numElements-1 do begin
    m_elements[i]^.m_id := find(i);
    m_elements[i]^.m_sz := i;
  end;
   // Sort the vector using predicate and std::sort
    //std::sort(m_elements.begin(), m_elements.end(), btUnionFindElementSortPredicate);
   m_elements.quickSort(@btUnionFindElementSortPredicate);
end;

procedure btUnionFind.reset(const N: integer);
var
  i: Integer;
begin
 allocate(N);
 for i := 0 to  N-1 do begin
   m_elements[i]^.m_id := i;
   m_elements[i]^.m_sz := 1;
 end;
end;

function btUnionFind.getNumElements: integer;
begin
 Result := m_elements.size;
end;

function btUnionFind.isRoot(const x: integer): boolean;
begin
 Result := x = m_elements[x]^.m_id;
end;

function btUnionFind.getElement(const index: integer): PbtElement;
begin
 result := m_elements[index];
end;

procedure btUnionFind.allocate(const N: integer);
begin
  m_elements.resize(N);
end;


function btUnionFind.find(const p, q: integer): integer;
begin
  if find(p) = find(q) then begin
    exit(1);
  end else begin
    exit(0);
  end;
end;

procedure btUnionFind.unite(const p, q: integer);
var i,j:integer;
begin
  i := find(p);
  j := find(q);
  if i = j then exit;
  {$ifndef USE_PATH_COMPRESSION}
  //weighted quick union, this keeps the 'trees' balanced, and keeps performance of unite O( log(n) )
  if (m_elements[i].m_sz < m_elements[j].m_sz)
  {
         m_elements[i].m_id = j; m_elements[j].m_sz += m_elements[i].m_sz;
  }
  else
  {
         m_elements[j].m_id = i; m_elements[i].m_sz += m_elements[j].m_sz;
  }
  {$else}
  m_elements[i]^.m_id := j;
  m_elements[j]^.m_sz += m_elements[i]^.m_sz;
  {$endif} //USE_PATH_COMPRESSION
end;

function btUnionFind.find(x: integer): integer;
var elementPtr : PbtElement;
    id         : integer;
begin
   //btAssert(x < m_N);
   //btAssert(x >= 0);
   while (x <> m_elements[x]^.m_id) do begin
  //not really a reason not to use path compression, and it flattens the trees/improves find performance dramatically
    {$ifdef USE_PATH_COMPRESSION}
     id := m_elements[x]^.m_id;
     elementPtr          := m_elements[id];
     m_elements[x]^.m_id := elementPtr^.m_id;
     x                   := elementPtr^.m_id;
    {$else}
     x := m_elements[x]^.m_id;
    {$endif}
     //btAssert(x < m_N);
     //btAssert(x >= 0);
   end;
   result := x;
end;

{ btSimulationIslandManager }

constructor btSimulationIslandManager.create;
begin
  m_splitIslands   := true;
  m_islandmanifold := btManifoldArray.create;
  m_islandBodies   := btCollisionObjectArray.create;
  m_unionFind      := btUnionFind.create;
end;

destructor btSimulationIslandManager.destroy;
begin
 m_islandmanifold.Free;
  m_islandBodies.Free;
  m_unionFind.Free;
end;

procedure btSimulationIslandManager.initUnionFind(const n: integer);
begin
  m_unionFind.reset(n);
end;

function btSimulationIslandManager.getUnionFind: btUnionFind;
begin
  result := m_unionFind;
end;

{$ifdef STATIC_SIMULATION_ISLAND_OPTIMIZATION}
procedure btSimulationIslandManager.updateActivationState(const colWorld: btCollisionWorld; const dispatcher: btDispatcher);
var i:integer;
    collisionObject : btCollisionObject;
begin
  initUnionFind(colWorld.getCollisionObjectArray.size);
  // put the index into m_controllers into m_tag
  for i:=0 to colWorld.getCollisionObjectArray.size-1 do begin
    collisionObject := colWorld.getCollisionObjectArray[i]^;
    collisionObject.setIslandTag(i);
    collisionObject.setCompanionId(-1);
    collisionObject.setHitFraction(1);
  end;
  // do the union find
  findUnions(dispatcher,colWorld);
end;
procedure btSimulationIslandManager.storeIslandActivationState(const world: btCollisionWorld);
var i,idx:integer;
    collisionObject : btCollisionObject;
begin
 // put the islandId ('find' value) into m_tag
  for i:=0 to world.getCollisionObjectArray.size-1 do begin
    collisionObject := world.getCollisionObjectArray[i]^;
    if not collisionObject.isStaticOrKinematicObject then begin
      idx := m_unionFind.find(i);
      collisionObject.setIslandTag( idx );
      collisionObject.setCompanionId(-1);
    end else begin
      collisionObject.setIslandTag(-1);
      collisionObject.setCompanionId(-2);
    end;
  end;
end;
{$ELSE}
 ...Port on occasion (FOSHH)
{$ENDIF}

{$HINTS OFF}
procedure btSimulationIslandManager.findUnions(const dispatcher: btDispatcher; const colWorld: btCollisionWorld);
var pairCachePtr        : btOverlappingPairCache;
    numOverlappingPairs : integer;
    pairPtr             : btBroadphasePairArray;
    collisionPair       : PbtBroadphasePair;
    colObj0,colObj1     : btCollisionObject;
    i                   : Integer;
begin
  pairCachePtr        := colWorld.getPairCache;
  numOverlappingPairs := pairCachePtr.getNumOverlappingPairs;
  pairPtr             := pairCachePtr.getOverlappingPairArray;

  for i:=0 to numOverlappingPairs-1 do begin
    collisionPair   := pairPtr.A[i];
    colObj0 := btCollisionObject(collisionPair^.m_pProxy0.m_clientObject);
    colObj1 := btCollisionObject(collisionPair^.m_pProxy1.m_clientObject);
    if (assigned(colObj0) and (colObj0.mergesSimulationIslands)) and (assigned(colObj1) and (colObj1.mergesSimulationIslands)) then begin
      m_unionFind.unite(colObj0.getIslandTag,colObj1.getIslandTag);
    end;
  end;
end;
{$HINTS ON}


function getIslandId(const lhs : btPersistentManifold):integer;FOS_INLINE;
begin
 result := btCollisionObject(lhs.getBody0).getIslandTag;
 if result >= 0 then exit;
 result := btCollisionObject(lhs.getBody1).getIslandTag;
 //islandId := rcolObj0->getIslandTag()>=0?rcolObj0->getIslandTag():rcolObj1->getIslandTag();
end;

function btPersistentManifoldSortPredicate(const lhs,rhs:btPersistentManifold):NativeInt;
begin
  //result := getIslandId(lhs) < getIslandId(rhs);
 result := getIslandId(lhs) - getIslandId(rhs);
end;

///@todo: this is random access, it can be walked 'cache friendly'!
procedure btSimulationIslandManager.buildAndProcessIslands(const dispatcher: btDispatcher; const collisionWorld: btCollisionWorld; const callback: btSM_IslandCallback);
var collisionObjects                        : btCollisionObjectArray;
    endIslandIndex,startIslandIndex,numElem : integer;
    startManifoldIndex,endManifoldIndex,i   : integer;
    numIslandManifolds,curIslandId          : integer;
    manifold                                : btManifoldArray;
    pm                                      : btPersistentManifold;
    NumManifolds,islandId                   : integer;
    islandSleeping                          : boolean;
    colObj0                                 : btCollisionObject;
//    startManifold                           : PbtPersistentManifold;

begin
  collisionObjects := collisionWorld.getCollisionObjectArray;
  buildIslands(dispatcher,collisionWorld);
  endIslandIndex := 1;
  numElem := m_unionFind.getNumElements;
  BT_PROFILE("processIslands");
  if not m_splitIslands then begin
    manifold := dispatcher.getInternalManifoldPointer;
    callback(collisionObjects,manifold,0,manifold.Size,-1);
  end else begin
  // Sort manifolds, based on islands
    numManifolds := m_islandmanifold.size;  //we should do radix sort, it it much faster (O(n) instead of O (n log2(n))
    m_islandmanifold.quickSort(@btPersistentManifoldSortPredicate);
    //writeln('');
    //writeln('+QS MFA');
    //for i:=0 to m_islandmanifold.Size -1 do begin
    //  pm := m_islandmanifold[i]^;
    //  write(' ix1a=',pm.m_index1a,' ',integer(btCollisionObject(pm.getBody0).getUserPointer),':',integer(btCollisionObject(pm.getBody1).getUserPointer));
    //end;
    //writeln('');
    //writeln('-QS MFA');
    //now process all active islands (sets of manifolds for now)
    startManifoldIndex := 0;
    endManifoldIndex   := 1;
    //traverse the simulation islands, and call the solver, unless all objects are sleeping/deactivated
    //for ( startIslandIndex=0;startIslandIndex<numElem;startIslandIndex = endIslandIndex)
    startIslandIndex :=0;
    while(startIslandIndex<numElem) do begin
      islandId       := m_unionFind.getElement(startIslandIndex)^.m_id;
      islandSleeping := false;
      endIslandIndex := startIslandIndex;
      while((endIslandIndex<numElem) and (m_unionFind.getElement(endIslandIndex)^.m_id = islandId)) do begin
        i        := m_unionFind.getElement(endIslandIndex)^.m_sz;
        colObj0  := collisionObjects[i]^;
        m_islandBodies.push_back(colObj0);
        if not colObj0.isActive then begin
          islandSleeping := true;
        end;
        inc(endIslandIndex);
      end;
      //find the accompanying contact manifold for this islandId
      numIslandManifolds := 0;
      if startManifoldIndex<numManifolds then begin
        curIslandId  := getIslandId(m_islandmanifold[startManifoldIndex]^);
        if curIslandId = islandId then begin
          endManifoldIndex := startManifoldIndex+1;
          while ((endManifoldIndex<numManifolds) and (islandId = getIslandId(m_islandmanifold[endManifoldIndex]^))) do begin
            inc(endManifoldIndex);
          end;
          /// Process the actual simulation, only if not sleeping/deactivated
          numIslandManifolds := endManifoldIndex-startManifoldIndex;
        end;
      end;
      if not islandSleeping then begin
      // writeln(format('Island callback of size: ID=%d %d bodies, %d manifolds , %d startmanifoldindex',[islandID,m_islandBodies.size,numIslandManifolds,startManifoldIndex]));
        callback(m_islandBodies,m_islandmanifold,startManifoldIndex,numIslandManifolds,islandId);
      end;
      if numIslandManifolds<>0 then begin
        startManifoldIndex := endManifoldIndex;
      end;
      m_islandBodies.resize(0);
      startIslandIndex := endIslandIndex;
    end;
  end; // else if(!splitIslands)
end;

procedure btSimulationIslandManager.buildIslands(const dispatcher: btDispatcher; const colWorld: btCollisionWorld);
var collisionObjects                : btCollisionObjectArray;
    numElem,islandId,idx            : integer;
    startIslandIndex,endIslandIndex : integer;
    allSleeping                     : boolean;
    colObj0,colObj1                 : btCollisionObject;
    manifold                        : btPersistentManifold;
    i,maxNumManifolds               : integer;
begin
  //.
  BT_PROFILE("islandUnionFindAndQuickSort");
  collisionObjects := colWorld.getCollisionObjectArray;
  m_islandmanifold.resize(0);
  //we are going to sort the unionfind array, and store the element id in the size
  //afterwards, we clean unionfind, to make sure no-one uses it anymore
  m_unionFind.sortIslands;
  numElem        := m_unionFind.getNumElements;

  //update the sleeping state for bodies, if all are sleeping
  startIslandIndex:=0;
  while(startIslandIndex<numElem) do begin
    islandId := m_unionFind.getElement(startIslandIndex)^.m_id;
    endIslandIndex := startIslandIndex+1;
    while((endIslandIndex<numElem) and (m_unionFind.getElement(endIslandIndex)^.m_id = islandId)) do inc(endIslandIndex);
    allSleeping := true;
    for idx := startIslandIndex to endIslandIndex-1 do begin
      i := m_unionFind.getElement(idx)^.m_sz;
      colObj0 := collisionObjects[i]^;
      btAssert((colObj0.getIslandTag = islandId) or (colObj0.getIslandTag = -1)); // error in island management
      if colObj0.getIslandTag = islandId then begin
        if colObj0.getActivationState = btas_ACTIVE_TAG then begin
          allSleeping := false;
        end;
        if colObj0.getActivationState = btas_DISABLE_DEACTIVATION then begin
          allSleeping := false;
        end;
      end;
    end;
    if allSleeping then begin
      for idx := startIslandIndex to endIslandIndex-1 do begin
        i := m_unionFind.getElement(idx)^.m_sz;
        colObj0 := collisionObjects[i]^;
        btAssert((colObj0.getIslandTag = islandId) or (colObj0.getIslandTag = -1)); // error in island management 2
        if colObj0.getIslandTag = islandId then begin
          colObj0.setActivationState(btas_ISLAND_SLEEPING);
        end;
      end;
    end else begin
      for idx := startIslandIndex to endIslandIndex-1 do begin
        i := m_unionFind.getElement(idx)^.m_sz;
        colObj0 := collisionObjects[i]^;
        btAssert((colObj0.getIslandTag = islandId) or (colObj0.getIslandTag = -1)); // error in island management 3
        if colObj0.getIslandTag = islandId then begin
          if colObj0.getActivationState =btas_ISLAND_SLEEPING then begin
            colObj0.setActivationState(btas_WANTS_DEACTIVATION);
            colObj0.setDeactivationTime(0);
          end;
        end;
      end;
    end;
    startIslandIndex := endIslandIndex;
  end;
  maxNumManifolds := dispatcher.getNumManifolds;
  for i:=0 to maxNumManifolds-1 do begin
    manifold := dispatcher.getManifoldByIndexInternal(i);
    colObj0 := btCollisionObject(manifold.getBody0);
    colObj1 := btCollisionObject(manifold.getBody1);
     ///@todo: check sleeping conditions!
    if (assigned(colObj0) and (colObj0.getActivationState <> btas_ISLAND_SLEEPING)) or
       (assigned(colObj1) and (colObj1.getActivationState <> btas_ISLAND_SLEEPING)) then begin
      //kinematic objects don't merge islands, but wake up all connected objects
      if colObj0.isKinematicObject and (colObj0.getActivationState <> btas_ISLAND_SLEEPING) then begin
        colObj1.activate;
      end;
      if colObj1.isKinematicObject and (colObj1.getActivationState <> btas_ISLAND_SLEEPING) then begin
        colObj0.activate;
      end;
      if m_splitIslands then begin
        //filtering for response
        if dispatcher.needsResponse(colObj0,colObj1) then begin
          m_islandmanifold.push_back(manifold);
        end;
      end;
    end;
  end;
end;

function btSimulationIslandManager.getSplitIslands: Boolean;
begin
  result := m_splitIslands;
end;

procedure btSimulationIslandManager.setSplitIslands(const doSplitIslands: boolean);
begin
  m_splitIslands := doSplitIslands;
end;


{ btDiscreteDynamicsWorld }

procedure btDiscreteDynamicsWorld.predictUnconstraintMotion(const timeStep: btScalar);
var body:btRigidBody;
  i: Integer;
begin
 //.
  BT_PROFILE("predictUnconstraintMotion");
  for i:=0 to m_nonStaticRigidBodies.size-1 do begin
    body := m_nonStaticRigidBodies[i]^;
    if not body.isStaticOrKinematicObject then begin
      body.integrateVelocities( timeStep);
      //damping
      body.applyDamping(timeStep);
      body.predictIntegratedTransform(timeStep,body.getInterpolationWorldTransformP^);
    end;
  end;
end;

type

  { btClosestNotMeConvexResultCallback }

  btClosestNotMeConvexResultCallback=class(btCW_ClosestConvexResultCallback)
    m_me                 : btCollisionObject;
    m_allowedPenetration : btScalar;
    m_pairCache          : btOverlappingPairCache;
    m_dispatcher         : btDispatcher;
  public
    constructor Create(const me : btCollisionObject ; const fromA,toA : btVector3; const pairCache : btOverlappingPairCache ; const dispatcher : btDispatcher );
    function    addSingleResult(const convexResult: btCW_LocalConvexResult; const normalInWorldSpace: boolean): btScalar; override;
    function    needsCollision(const proxy0: btBroadphaseProxy): boolean; override;
  end;

{ btClosestNotMeConvexResultCallback }

constructor btClosestNotMeConvexResultCallback.Create(const me: btCollisionObject; const fromA, toA: btVector3; const pairCache: btOverlappingPairCache; const dispatcher: btDispatcher);
begin
  inherited Create(fromA,toA);
  m_me                 := me;
  m_allowedPenetration := 0;
  m_pairCache          := pairCache;
  m_dispatcher         := dispatcher;
end;

function btClosestNotMeConvexResultCallback.addSingleResult(const convexResult: btCW_LocalConvexResult; const normalInWorldSpace: boolean): btScalar;
var   linVelA : btVector3;
begin
  if convexResult.m_hitCollisionObject = m_me then begin
    exit(1);
  end;
  //ignore result if there is no contact response
  if  not (convexResult.m_hitCollisionObject.hasContactResponse) then begin
    exit(1);
  end;
  linVelA := m_convexToWorld-m_convexFromWorld;
//  linVelB = btVector3(0,0,0);//toB.getOrigin()-fromB.getOrigin();
//  btVector3 relativeVelocity = (linVelA-linVelB);
  //don't report time of impact for motion away from the contact normal (or causes minor penetration)
  if (convexResult.m_hitNormalLocal.dot(linVelA)>=-m_allowedPenetration) then begin
    exit(1);
  end;
  Result := inherited addSingleResult(convexResult, normalInWorldSpace);
end;

function btClosestNotMeConvexResultCallback.needsCollision(const proxy0: btBroadphaseProxy): boolean;
var otherObj : btCollisionObject;
    manifoldArray : btManifoldArray;
   collisionPair  : PbtBroadphasePair;
   j: Integer;
begin
  //don't collide with itself
  result := false;
  if btCollisionObject(proxy0.m_clientObject) = m_me then exit;
  ///don't do CCD when the collision filters are not matching
  if not inherited needsCollision(proxy0) then exit;

  otherObj := btCollisionObject(proxy0.m_clientObject);
  //call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
  if m_dispatcher.needsResponse(m_me,otherObj) then begin
    ///don't do CCD when there are already contact points (touching contact/penetration)
    manifoldArray := btManifoldArray.create;
    collisionPair := m_pairCache.findPair(m_me.getBroadphaseHandle,proxy0);
    if assigned(collisionPair) then begin
      if assigned(collisionPair^.m_algorithm) then begin
        manifoldArray.resize(0);
        collisionPair^.m_algorithm.getAllContactManifolds(manifoldArray);
        for j:=0 to manifoldArray.size-1 do begin
          if manifoldArray[j]^.getnumcontacts>0 then begin
            manifoldArray.Free;
            exit;
          end;
        end;
      end;
    end;
    manifoldArray.Free;
  end;
  result := true;
end;


procedure btDiscreteDynamicsWorld.integrateTransforms(const timeStep: btScalar);
var predictedTrans : btTransform;
    i              : Integer;
    body           : btRigidBody;
    squareMotion   : btScalar;
    sweepResults   : btClosestNotMeConvexResultCallback;
    tmpSphere      : btSphereShape;
begin
  //.
  BT_PROFILE("integrateTransforms");
  for i := 0 to m_nonStaticRigidBodies.size-1 do begin
    body := m_nonStaticRigidBodies[i]^;
    body.setHitFraction(1);
    {$IFDEF FOS_DYNAMICS_DEBUG}
    writeln('BEFORE INTEGRATE');
    fdb_pbody(body,integer(body.getUserPointer));
    {$ENDIF}
    if body.isActive and not body.isStaticOrKinematicObject then begin
      body.predictIntegratedTransform(timeStep, predictedTrans);
      squareMotion := (predictedTrans.getOriginV^-body.getWorldTransformP^.getOriginV^).length2;
      if (body.getCcdSquareMotionThreshold <> 0) and (body.getCcdSquareMotionThreshold < squareMotion) then begin
        BT_PROFILE("CCD motion clamping");
        if body.getCollisionShape.isConvex then begin
          inc(gNumClampedCcdMotions);
          sweepResults := btClosestNotMeConvexResultCallback.Create(body,body.getWorldTransformP^.getOriginV^,predictedTrans.getOriginV^,m_broadphasePairCache.getOverlappingPairCache,m_dispatcher1);
          tmpSphere    := btSphereShape.Create(body.getCcdSweptSphereRadius);
          sweepResults.m_collisionFilterGroup := body.getBroadphaseProxy.m_collisionFilterGroup;
          sweepResults.m_collisionFilterMask  := body.getBroadphaseProxy.m_collisionFilterMask;
          convexSweepTest(tmpSphere,body.getWorldTransformP^,predictedTrans,sweepResults);
          if sweepResults.hasHit  and (sweepResults.m_closestHitFraction < 1) then begin
            body.setHitFraction(sweepResults.m_closestHitFraction);
            body.predictIntegratedTransform(timeStep*body.getHitFraction, predictedTrans);
            body.setHitFraction(0);
            // printf("clamped integration to hit fraction = %f\n",fraction);
          end;
          sweepResults.Free;
          tmpSphere.Free;
        end;
      end;
     body.proceedToTransform(predictedTrans);
     {$IFDEF FOS_DYNAMICS_DEBUG}
     writeln('AFTER INTEGRATE');
     fdb_pbody(body,integer(body.getUserPointer));
     {$ENDIF}
    end;
  end;
end;

procedure btDiscreteDynamicsWorld.calculateSimulationIslands;
var i, numConstraints : integer;
    constraint        : btTypedConstraint;
    colObj0,colObj1   : btRigidBody;
begin
  //.
  BT_PROFILE("calculateSimulationIslands");
  m_islandManager.updateActivationState(self,m_dispatcher1);
  numConstraints := m_constraints.size-1;
  for i:=0 to numConstraints do begin
    constraint := m_constraints[i]^;
    colObj0    := constraint.m_rbA;
    colObj1    := constraint.m_rbB;
    if (assigned(colObj0) and not (colObj0.isStaticOrKinematicObject)) and
       (assigned(colObj1) and not (colObj1.isStaticOrKinematicObject)) then begin
      if colObj0.isActive or colObj1.isActive then begin
        m_islandManager.m_unionFind.unite(colObj0.getIslandTag,colObj1.getIslandTag);
      end;
    end;
  end;
  //Store the island id in each body
  m_islandManager.storeIslandActivationState(self);
end;

function btGetConstraintIslandId(const lhs:btTypedConstraint):integer; FOS_INLINE;
var lhs_id:integer;
begin
  lhs_id := lhs.m_rbA.m_islandTag1;
  if lhs_id<0 then begin
    lhs_id := lhs.m_rbB.m_islandTag1;
  end;
  result := lhs_id;
end;

function btSortConstraintOnIslandPredicate(const lhs,rhs:btTypedConstraint):nativeint;
begin
  //Result := btGetConstraintIslandId(lhs) < btGetConstraintIslandId(rhs);
 Result := btGetConstraintIslandId(lhs) - btGetConstraintIslandId(rhs);
end;


procedure btDiscreteDynamicsWorld.solveConstraints(const solverInfo: btContactSolverInfo);
var i                          : Integer;
    m_sortedConstraints        : btTypedConstraintArray;
    m_numSortedConstraints     : integer;
    m_bodies                   : btCollisionObjectArray;
    m_manifolds                : btManifoldArray;
    m_callback_constraints     : btTypedConstraintArray;

  procedure _ProcessConstraints;
  begin
    if (m_manifolds.size + m_callback_constraints.size)>0 then begin
      m_constraintSolver.solveGroup(m_bodies,m_bodies.size,m_manifolds,0,m_manifolds.size,m_callback_constraints,0,m_callback_constraints.size,solverInfo,m_debugDrawer,m_stackAlloc,m_dispatcher1);
    end;
    m_bodies.resize(0);
    m_manifolds.resize(0);
    m_callback_constraints.resize(0);
  end;

  procedure _ProcessIsland (const bodies: btCollisionObjectArray; const manifolds: btManifoldArray; const startIdx, numManifolds: integer; const islandId: integer);
  var  numCurConstraints,i : integer;
       fos_startIndex      : integer;
  begin
    if islandId<0 then begin
      if (numManifolds + m_numSortedConstraints)<>0 then begin
        ///we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
        //FOSTODO -> sortedConstraints start at 0 ??
        m_constraintSolver.solveGroup(bodies,bodies.Size,manifolds, startIdx,numManifolds,m_sortedConstraints,0,m_numSortedConstraints,solverInfo,m_debugDrawer,m_stackAlloc,m_dispatcher1);
      end;
    end else begin //also add all non-contact constraints/joints for this island
      numCurConstraints := 0;
      //find the first constraint for this island
      fos_startIndex:=0;
      for i:=0 to m_numSortedConstraints-1 do begin
        if btGetConstraintIslandId(m_sortedConstraints[i]^) = islandId then begin
          //FOS startConstraint := &m_sortedConstraints[i];
          fos_startIndex := i;
          break;
        end;
      end;
      //count the number of constraints in this island
      for i := fos_startIndex to m_numSortedConstraints-1 do begin
        if btGetConstraintIslandId(m_sortedConstraints[i]^) = islandId then begin
          inc(numCurConstraints);
        end;
      end;
      if solverInfo.m_minimumSolverBatchSize <=1 then begin
       ///only call solveGroup if there is some work: avoid virtual function call, its overhead can be excessive
       if (numManifolds + numCurConstraints)<>0 then begin
         m_constraintSolver.solveGroup(bodies,bodies.Size,manifolds,startIdx, numManifolds,m_sortedConstraints,fos_startIndex,numCurConstraints,solverInfo,m_debugDrawer,m_stackAlloc,m_dispatcher1);
       end;
      end else begin
        for i:=0 to bodies.size-1 do begin
          m_bodies.push_back(bodies[i]^);
        end;
        for i:=startIdx to (startIdx+numManifolds-1) do begin
          m_manifolds.push_back(manifolds[i]^);
        end;
        for i:=0 to numCurConstraints-1 do begin
          m_callback_constraints.push_back(m_sortedConstraints[i+fos_startIndex]^);
        end;
        if (m_callback_constraints.size+m_manifolds.size)>solverInfo.m_minimumSolverBatchSize then begin
          _processConstraints;
        end else begin
          //printf("deferred\n");
        end;
      end;
    end;
  end;



begin
 //.
  BT_PROFILE("solveConstraints");
  // sorted version of all btTypedConstraint, based on islandId
  if m_constraints.Size<>0 then begin
    m_sortedConstraints := btTypedConstraintArray.create;
    m_sortedConstraints.resize( m_constraints.size());
    for i := 0 to m_constraints.Size-1 do begin
      m_sortedConstraints[i]^ := m_constraints[i]^;
    end;
    m_sortedConstraints.quickSort(@btSortConstraintOnIslandPredicate);
    m_numSortedConstraints := m_sortedConstraints.size;
  end else begin
    m_sortedConstraints    := nil;
    m_numSortedConstraints := 0;
  end;

  m_bodies               := btCollisionObjectArray.create;
  m_manifolds            := btManifoldArray.create;
  m_callback_constraints := btTypedConstraintArray.create;

  m_constraintSolver.prepareSolve(m_collisionObjects.Size,m_dispatcher1.getNumManifolds);
  /// solve all the constraints for this island
  m_islandManager.buildAndProcessIslands(m_dispatcher1,self,@_ProcessIsland);
  _ProcessConstraints;

  m_constraintSolver.allSolved(solverInfo, m_debugDrawer, m_stackAlloc);

  m_bodies.Free;
  m_callback_constraints.Free;
  m_manifolds.Free;

  m_sortedConstraints.Free;
end;

procedure btDiscreteDynamicsWorld.updateActivationState(const timeStep: btScalar);
var body : btRigidBody;
  i: Integer;
begin
 //.
  BT_PROFILE("updateActivationState");
  for i := 0 to m_nonStaticRigidBodies.size-1 do begin
    body := m_nonStaticRigidBodies[i]^;
    if assigned(body) then begin
      body.updateDeactivation(timeStep);
      if body.wantsSleeping then begin
        if body.isStaticOrKinematicObject then begin
          body.setActivationState(btas_ISLAND_SLEEPING);
        end else begin
          if body.getActivationState = btas_ACTIVE_TAG then begin
            body.setActivationState(btas_WANTS_DEACTIVATION );
          end;
          if body.getActivationState=btas_ISLAND_SLEEPING then begin
            body.setAngularVelocity(cbtNullVector);
            body.setLinearVelocity(cbtNullVector);
          end;
        end;
      end else begin
        if body.getActivationState<>btas_DISABLE_DEACTIVATION then begin
          body.setActivationState(btas_ACTIVE_TAG );
        end;
      end;
    end;
  end;
end;

procedure btDiscreteDynamicsWorld.updateActions(const timeStep: btScalar);
var
  i: Integer;
begin
 //.
  BT_PROFILE("updateActions");
  for i := 0 to m_actions.size-1 do begin
    m_actions[i]^.updateAction(self, timeStep);
  end;
end;

{$HINTS OFF}
procedure btDiscreteDynamicsWorld.startProfiling(const timeStep: btScalar);
begin
 {$ifndef BT_NO_PROFILE}
 CProfileManager::Reset();
 {$endif} //BT_NO_PROFILE
end;
{$HINTS ON}

procedure btDiscreteDynamicsWorld.internalSingleStepSimulation(const timeStep: btScalar);
begin
 //.
  BT_PROFILE("internalSingleStepSimulation");
  if assigned(m_internalPreTickCallback) then begin
    m_internalPreTickCallback(self, timeStep);
  end;
  ///apply gravity, predict motion
  predictUnconstraintMotion(timeStep);
  m_dispatchInfo.m_timeStep  := timeStep;
  m_dispatchInfo.m_stepCount := 0;
  m_dispatchInfo.m_debugDraw := m_debugDrawer; //TODO - Why set this every time ? (FOS)
  ///perform collision detection
  performDiscreteCollisionDetection;
  calculateSimulationIslands;
  m_solverInfo.m_timeStep := timeStep;
  ///solve contact and other joint constraints
  solveConstraints(m_solverInfo);
  ///CallbackTriggers();
  ///integrate transforms
  integrateTransforms(timeStep);
  ///update vehicle simulation
  updateActions(timeStep);
  updateActivationState( timeStep );
  if assigned(m_internalTickCallback) then begin
    m_internalTickCallback(self, timeStep);
  end;
end;

procedure btDiscreteDynamicsWorld.saveKinematicState(const timeStep: btScalar);
var colObj : btCollisionObject;
    body   : btRigidBody;
    i: Integer;
begin
 ///would like to iterate over m_nonStaticRigidBodies, but unfortunately old API allows
 ///to switch status _after_ adding kinematic objects to the world
 ///fix it for Bullet 3.x release
  for i := 0 to m_collisionObjects.size-1 do begin
    colObj := m_collisionObjects[i]^;
    body   := btRigidBody.upcast(colObj);
    if assigned(body) and (body.getActivationState<>btas_ISLAND_SLEEPING) then begin
      if body.isKinematicObject then begin
        //to calculate velocities next frame
        body.saveKinematicState(timeStep);
      end;
    end;
  end;
end;

constructor btDiscreteDynamicsWorld.create(const dispatcher: btDispatcher; const pairCache: btBroadphaseInterface; const constraintSolver: btConstraintSolver; const collisionConfiguration: btCollisionConfiguration);
begin
  inherited Create(dispatcher,pairCache,collisionConfiguration);
  m_constraintSolver := constraintSolver;
  m_gravity.Init(0,-10,0);
  m_localTime := 1/60;
  m_synchronizeAllMotionStates := false;
  m_nonStaticRigidBodies       := btRigidBodyArray.create;
  m_nonStaticRigidBodies.Initialize(btRigidBody);
  m_nonStaticRigidBodies.SetComparefunctionT(@btRigidBodyCompare);
  m_actions := btActionInterfaceArray.create;
  m_actions.Initialize(btActionInterface);
  m_constraints := btTypedConstraintArray.create;
  m_constraints.Initialize(btTypedConstraint);
  m_profileTimings:=0;
  if not assigned(m_constraintSolver) then begin
    m_constraintSolver     := btSequentialImpulseConstraintSolver.create;
    m_ownsConstraintSolver := true;
  end else begin
    m_ownsConstraintSolver := false;
  end;
  m_islandManager     := btSimulationIslandManager.create;
  m_ownsIslandManager := true;
end;

destructor btDiscreteDynamicsWorld.destroy;
begin
  //only delete it when we created it
  if m_ownsIslandManager then begin
    m_islandManager.Free;
  end;
  if m_ownsConstraintSolver then begin
    m_constraintSolver.Free;
  end;
  m_actions.Free;
  m_nonStaticRigidBodies.Free;
  inherited;
end;

function btDiscreteDynamicsWorld.stepSimulation(const timeStep: btScalar; maxSubSteps: integer; fixedTimeStep: btScalar): integer;
var numSimulationSubSteps,clampedSimulationSteps : integer;
    i                                            : Integer;
begin
  startProfiling(timeStep);
  BT_PROFILE("stepSimulation");
  numSimulationSubSteps := 0;
  if maxSubSteps<>0 then begin
    //fixed timestep with interpolation
    m_localTime := m_localTime + timeStep;
    if m_localTime >= fixedTimeStep then begin
      numSimulationSubSteps := trunc(m_localTime / fixedTimeStep);
      m_localTime := m_localTime - (numSimulationSubSteps * fixedTimeStep);
    end;
  end else begin
    //variable timestep
    fixedTimeStep := timeStep;
    m_localTime   := timeStep;
    if btFuzzyZero(timeStep) then begin
      numSimulationSubSteps := 0;
      maxSubSteps           := 0;
    end else begin
      numSimulationSubSteps := 1;
      maxSubSteps           := 1;
    end;
  end;
  //process some debugging flags
  if assigned(m_debugdrawer) then begin
    gDisableDeactivation := DBG_NoDeactivation in m_debugDrawer.getDebugMode;
  end;
  if numSimulationSubSteps<>0 then begin
    //clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
    if numSimulationSubSteps > maxSubSteps then begin
      clampedSimulationSteps:=maxSubSteps;
    end else begin
      clampedSimulationSteps:=numSimulationSubSteps;
    end;
    saveKinematicState(fixedTimeStep*clampedSimulationSteps);
    applyGravity;
    for i:=0  to clampedSimulationSteps-1 do begin
      internalSingleStepSimulation(fixedTimeStep);
      synchronizeMotionStates;
    end;
  end else begin
    synchronizeMotionStates;
  end;
  clearForces;
  {$ifndef BT_NO_PROFILE}
  CProfileManager::Increment_Frame_Counter();
  {$endif} //BT_NO_PROFILE
  Result := numSimulationSubSteps;
end;

procedure btDiscreteDynamicsWorld.synchronizeMotionStates;
var body : btRigidBody;
  i: Integer;
begin
 //.
  BT_PROFILE("synchronizeMotionStates");
  if m_synchronizeAllMotionStates then begin
    //iterate  over all collision objects
    for i := 0 to m_collisionObjects.size-1 do begin
      body := btRigidBody.upcast(m_collisionObjects[i]^);
      if assigned(body) then begin
        synchronizeSingleMotionState(body);
      end;
    end;
  end else begin
    //iterate over all active rigid bodies
    for i := 0 to m_nonStaticRigidBodies.size-1 do begin
      body := m_nonStaticRigidBodies[i]^;
      if (body.isActive) then begin
        synchronizeSingleMotionState(body);
      end;
      fdb_pbody(body,i);
    end;
  end;
end;

procedure btDiscreteDynamicsWorld.synchronizeSingleMotionState(const body: btRigidBody);
var        interpolatedTransform:btTransform;
begin
  //.
  btAssert(assigned(body));
  if assigned(body.getMotionState) and not body.isStaticOrKinematicObject then begin
    //we need to call the update at least once, even for sleeping objects
    //otherwise the 'graphics' transform never updates properly
    ///@todo: add 'dirty' flag
    //if (body->getActivationState() != ISLAND_SLEEPING)
    //{
      btTransformUtil.integrateTransform(body.getInterpolationWorldTransformP^,body.getInterpolationLinearVelocityP^,body.getInterpolationAngularVelocityP^,m_localTime*body.getHitFraction,interpolatedTransform);
      body.getMotionState.setWorldTransform(interpolatedTransform);
    //}
  end;
end;

procedure btDiscreteDynamicsWorld.addConstraint(const constraint: btTypedConstraint; const disableCollisionsBetweenLinkedBodies: Boolean);
begin
  m_constraints.push_back(constraint);
  if disableCollisionsBetweenLinkedBodies then begin
    constraint.getRigidBodyA.addConstraintRef(constraint);
    constraint.getRigidBodyB.addConstraintRef(constraint);
  end;
end;

procedure btDiscreteDynamicsWorld.removeConstraint(const constraint: btTypedConstraint);
begin
  m_constraints.remove(constraint);
  constraint.getRigidBodyA.removeConstraintRef(constraint);
  constraint.getRigidBodyB.removeConstraintRef(constraint);
end;

procedure btDiscreteDynamicsWorld.addAction(const ai: btActionInterface);
begin
  m_actions.push_back(ai);
end;

procedure btDiscreteDynamicsWorld.removeAction(const ai: btActionInterface);
begin
  m_actions.remove(ai);
end;

procedure btDiscreteDynamicsWorld.setGravity(const gravity: btVector3);
var body : btRigidBody;
    i    : Integer;
begin
  m_gravity := gravity;
  for i :=0 to m_nonStaticRigidBodies.size-1 do begin
    body := m_nonStaticRigidBodies[i]^;
    if body.isActive and not(BT_DISABLE_WORLD_GRAVITY in body.getFlags) then begin
      body.setGravity(gravity);
    end;
  end;
end;

function btDiscreteDynamicsWorld.getGravity: btVector3;
begin
 Result := m_gravity;
end;

procedure btDiscreteDynamicsWorld.addCollisionObject(const collisionObject: btCollisionObject; const collisionFilterGroup: btCollisionFilterGroupSet; collisionFilterMask: btCollisionFilterGroupSet);
begin
  inherited addCollisionObject(collisionObject,collisionFilterGroup,collisionFilterMask);
end;

procedure btDiscreteDynamicsWorld.addRigidBody(const body: btRigidBody);
begin
  if not (body.isStaticOrKinematicObject) and  not(BT_DISABLE_WORLD_GRAVITY in body.getFlags) then begin
    body.setGravity(m_gravity);
  end;
  if assigned(body.getCollisionShape) then begin
    if not body.isStaticObject then begin
      m_nonStaticRigidBodies.push_back(body);
    end else begin
      body.setActivationState(btas_ISLAND_SLEEPING);
    end;
    if not (body.isStaticObject or  body.isKinematicObject) then begin
      addCollisionObject(body,[btcfgDefaultFilter],btcfgAllFilter);
    end else begin
      addCollisionObject(body,[btcfgStaticFilter],btcfgAllButStatic);
    end;
  end;
end;

procedure btDiscreteDynamicsWorld.addRigidBody(const body: btRigidBody; const group, mask: btCollisionFilterGroupSet);
begin
  if not (body.isStaticOrKinematicObject) and  not(BT_DISABLE_WORLD_GRAVITY in body.getFlags) then begin
    body.setGravity(m_gravity);
  end;
  if assigned(body.getCollisionShape) then begin
    if not body.isStaticObject then begin
      m_nonStaticRigidBodies.push_back(body);
    end else begin
      body.setActivationState(btas_ISLAND_SLEEPING);
    end;
    addCollisionObject(body,group,mask);
  end;
end;

procedure btDiscreteDynamicsWorld.removeRigidBody(const body: btRigidBody);
begin
 m_nonStaticRigidBodies.remove(body);
 inherited removeCollisionObject(body);
end;

procedure btDiscreteDynamicsWorld.removeCollisionObject(const collisionObject: btCollisionObject);
var body:btRigidBody;
begin
 body := btRigidBody.upcast(collisionObject);
 if assigned(body) then begin
   removeRigidBody(body);
 end else begin
   inherited removeCollisionObject(collisionObject);
 end;
end;

{$HINTS OFF}
procedure btDiscreteDynamicsWorld.debugDrawConstraint(const constraint: btTypedConstraint);
begin
 //FOSTODO - pulls in constraints

  //bool drawFrames = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraints) != 0;
  //bool drawLimits = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraintLimits) != 0;
  //btScalar dbgDrawSize = constraint->getDbgDrawSize();
  //if(dbgDrawSize <= btScalar(0.f))
  //{
  //      return;
  //}
  //
  //switch(constraint->getConstraintType())
  //{
  //      case POINT2POINT_CONSTRAINT_TYPE:
  //              {
  //                      btPoint2PointConstraint* p2pC = (btPoint2PointConstraint*)constraint;
  //                      btTransform tr;
  //                      tr.setIdentity();
  //                      btVector3 pivot = p2pC->getPivotInA();
  //                      pivot = p2pC->getRigidBodyA().getCenterOfMassTransform() * pivot;
  //                      tr.setOrigin(pivot);
  //                      getDebugDrawer()->drawTransform(tr, dbgDrawSize);
  //                      // that ideally should draw the same frame
  //                      pivot = p2pC->getPivotInB();
  //                      pivot = p2pC->getRigidBodyB().getCenterOfMassTransform() * pivot;
  //                      tr.setOrigin(pivot);
  //                      if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
  //              }
  //              break;
  //      case HINGE_CONSTRAINT_TYPE:
  //              {
  //                      btHingeConstraint* pHinge = (btHingeConstraint*)constraint;
  //                      btTransform tr = pHinge->getRigidBodyA().getCenterOfMassTransform() * pHinge->getAFrame();
  //                      if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
  //                      tr = pHinge->getRigidBodyB().getCenterOfMassTransform() * pHinge->getBFrame();
  //                      if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
  //                      btScalar minAng = pHinge->getLowerLimit();
  //                      btScalar maxAng = pHinge->getUpperLimit();
  //                      if(minAng == maxAng)
  //                      {
  //                              break;
  //                      }
  //                      bool drawSect = true;
  //                      if(minAng > maxAng)
  //                      {
  //                              minAng = btScalar(0.f);
  //                              maxAng = SIMD_2_PI;
  //                              drawSect = false;
  //                      }
  //                      if(drawLimits)
  //                      {
  //                              btVector3& center = tr.getOrigin();
  //                              btVector3 normal = tr.getBasis().getColumn(2);
  //                              btVector3 axis = tr.getBasis().getColumn(0);
  //                              getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, minAng, maxAng, btVector3(0,0,0), drawSect);
  //                      }
  //              }
  //              break;
  //      case CONETWIST_CONSTRAINT_TYPE:
  //              {
  //                      btConeTwistConstraint* pCT = (btConeTwistConstraint*)constraint;
  //                      btTransform tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
  //                      if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
  //                      tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
  //                      if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
  //                      if(drawLimits)
  //                      {
  //                              //const btScalar length = btScalar(5);
  //                              const btScalar length = dbgDrawSize;
  //                              static int nSegments = 8*4;
  //                              btScalar fAngleInRadians = btScalar(2.*3.1415926) * (btScalar)(nSegments-1)/btScalar(nSegments);
  //                              btVector3 pPrev = pCT->GetPointForAngle(fAngleInRadians, length);
  //                              pPrev = tr * pPrev;
  //                              for (int i=0; i<nSegments; i++)
  //                              {
  //                                      fAngleInRadians = btScalar(2.*3.1415926) * (btScalar)i/btScalar(nSegments);
  //                                      btVector3 pCur = pCT->GetPointForAngle(fAngleInRadians, length);
  //                                      pCur = tr * pCur;
  //                                      getDebugDrawer()->drawLine(pPrev, pCur, btVector3(0,0,0));
  //
  //                                      if (i%(nSegments/8) == 0)
  //                                              getDebugDrawer()->drawLine(tr.getOrigin(), pCur, btVector3(0,0,0));
  //
  //                                      pPrev = pCur;
  //                              }
  //                              btScalar tws = pCT->getTwistSpan();
  //                              btScalar twa = pCT->getTwistAngle();
  //                              bool useFrameB = (pCT->getRigidBodyB().getInvMass() > btScalar(0.f));
  //                              if(useFrameB)
  //                              {
  //                                      tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
  //                              }
  //                              else
  //                              {
  //                                      tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
  //                              }
  //                              btVector3 pivot = tr.getOrigin();
  //                              btVector3 normal = tr.getBasis().getColumn(0);
  //                              btVector3 axis1 = tr.getBasis().getColumn(1);
  //                              getDebugDrawer()->drawArc(pivot, normal, axis1, dbgDrawSize, dbgDrawSize, -twa-tws, -twa+tws, btVector3(0,0,0), true);
  //
  //                      }
  //              }
  //              break;
  //      case D6_CONSTRAINT_TYPE:
  //              {
  //                      btGeneric6DofConstraint* p6DOF = (btGeneric6DofConstraint*)constraint;
  //                      btTransform tr = p6DOF->getCalculatedTransformA();
  //                      if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
  //                      tr = p6DOF->getCalculatedTransformB();
  //                      if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
  //                      if(drawLimits)
  //                      {
  //                              tr = p6DOF->getCalculatedTransformA();
  //                              const btVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
  //                              btVector3 up = tr.getBasis().getColumn(2);
  //                              btVector3 axis = tr.getBasis().getColumn(0);
  //                              btScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
  //                              btScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
  //                              btScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
  //                              btScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
  //                              getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * btScalar(.9f), minTh, maxTh, minPs, maxPs, btVector3(0,0,0));
  //                              axis = tr.getBasis().getColumn(1);
  //                              btScalar ay = p6DOF->getAngle(1);
  //                              btScalar az = p6DOF->getAngle(2);
  //                              btScalar cy = btCos(ay);
  //                              btScalar sy = btSin(ay);
  //                              btScalar cz = btCos(az);
  //                              btScalar sz = btSin(az);
  //                              btVector3 ref;
  //                              ref[0] = cy*cz*axis[0] + cy*sz*axis[1] - sy*axis[2];
  //                              ref[1] = -sz*axis[0] + cz*axis[1];
  //                              ref[2] = cz*sy*axis[0] + sz*sy*axis[1] + cy*axis[2];
  //                              tr = p6DOF->getCalculatedTransformB();
  //                              btVector3 normal = -tr.getBasis().getColumn(0);
  //                              btScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
  //                              btScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
  //                              if(minFi > maxFi)
  //                              {
  //                                      getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btVector3(0,0,0), false);
  //                              }
  //                              else if(minFi < maxFi)
  //                              {
  //                                      getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btVector3(0,0,0), true);
  //                              }
  //                              tr = p6DOF->getCalculatedTransformA();
  //                              btVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
  //                              btVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
  //                              getDebugDrawer()->drawBox(bbMin, bbMax, tr, btVector3(0,0,0));
  //                      }
  //              }
  //              break;
  //      case SLIDER_CONSTRAINT_TYPE:
  //              {
  //                      btSliderConstraint* pSlider = (btSliderConstraint*)constraint;
  //                      btTransform tr = pSlider->getCalculatedTransformA();
  //                      if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
  //                      tr = pSlider->getCalculatedTransformB();
  //                      if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
  //                      if(drawLimits)
  //                      {
  //                              btTransform tr = pSlider->getUseLinearReferenceFrameA() ? pSlider->getCalculatedTransformA() : pSlider->getCalculatedTransformB();
  //                              btVector3 li_min = tr * btVector3(pSlider->getLowerLinLimit(), 0.f, 0.f);
  //                              btVector3 li_max = tr * btVector3(pSlider->getUpperLinLimit(), 0.f, 0.f);
  //                              getDebugDrawer()->drawLine(li_min, li_max, btVector3(0, 0, 0));
  //                              btVector3 normal = tr.getBasis().getColumn(0);
  //                              btVector3 axis = tr.getBasis().getColumn(1);
  //                              btScalar a_min = pSlider->getLowerAngLimit();
  //                              btScalar a_max = pSlider->getUpperAngLimit();
  //                              const btVector3& center = pSlider->getCalculatedTransformB().getOrigin();
  //                              getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, a_min, a_max, btVector3(0,0,0), true);
  //                      }
  //              }
  //              break;
  //      default :
  //              break;
  //}
end;
{$HINTS ON}


procedure btDiscreteDynamicsWorld.debugDrawWorld;
var drawConstraints:boolean;
    mode :TbtIDEBUGFlagsSet;
    i: Integer;
begin
  //.
  BT_PROFILE("debugDrawWorld");
  inherited debugDrawWorld;

  mode := m_debugDrawer.getDebugMode;
  drawConstraints := false;
  if assigned(m_debugDrawer) then begin
    drawConstraints := mode  * [DBG_DrawConstraints,DBG_DrawConstraintLimits] <> [];
  end;
  if drawConstraints then begin
    for i := getNumConstraints-1 downto 0 do begin
      debugDrawConstraint(m_constraints[i]^);
    end;
  end;
  if assigned(m_debugDrawer) and (mode * [DBG_DrawWireframe,DBG_DrawAabb] <> []) then begin
    for i:=0 to m_actions.size-1 do begin
      m_actions[i]^.debugDraw(m_debugDrawer);
    end;
  end;
end;

procedure btDiscreteDynamicsWorld.setConstraintSolver(const solver: btConstraintSolver);
begin
  if m_ownsConstraintSolver then begin
    m_constraintSolver.Free;
  end;
  m_ownsConstraintSolver := false;
  m_constraintSolver     := solver;
end;

function btDiscreteDynamicsWorld.getConstraintSolver: btConstraintSolver;
begin
  result := m_constraintSolver;
end;

function btDiscreteDynamicsWorld.getNumConstraints: integer;
begin
  result := m_constraints.Size;
end;

function btDiscreteDynamicsWorld.getConstraint(const index: integer): btTypedConstraint;
begin
 Result := m_constraints[index]^;
end;

function btDiscreteDynamicsWorld.getWorldType: btDynamicsWorldType;
begin
 Result := BT_DISCRETE_DYNAMICS_WORLD;
end;

procedure btDiscreteDynamicsWorld.clearForces;
var
  i: Integer;
begin
 ///@todo: iterate over awake simulation islands!
 for i := 0 to m_nonStaticRigidBodies.size-1 do begin
   //need to check if next line is ok
   //it might break backward compatibility (people applying forces on sleeping objects get never cleared and accumulate on wake-up
   btRigidBody(m_nonStaticRigidBodies[i]^).clearForces;
 end;
end;
///apply gravity, call this once per timestep
procedure btDiscreteDynamicsWorld.applyGravity;
var i: Integer;
begin
 ///@todo: iterate over awake simulation islands!
 for i := 0 to m_nonStaticRigidBodies.size-1 do begin
   if btRigidBody(m_nonStaticRigidBodies[i]^).isActive then begin
     btRigidBody(m_nonStaticRigidBodies[i]^).applyGravity;
   end;
 end;
end;

{$HINTS OFF}
procedure btDiscreteDynamicsWorld.setNumTasks(const numTasks: integer);
begin

end;
{$HINTS ON}

procedure btDiscreteDynamicsWorld.setSynchronizeAllMotionStates(const synchronizeAll: boolean);
begin
 m_synchronizeAllMotionStates := synchronizeAll;
end;

function btDiscreteDynamicsWorld.getSynchronizeAllMotionStates: Boolean;
begin
 Result := m_synchronizeAllMotionStates;
end;

{ btDefaultMotionState }

constructor btDefaultMotionState.create;
begin
 m_graphicsWorldTrans.setIdentity;
 m_centerOfMassOffset.setIdentity;
 m_startWorldTrans.setIdentity;
 m_userPointer        := nil;
end;

constructor btDefaultMotionState.create(const startTrans: btTransform);
begin
 m_graphicsWorldTrans := startTrans;
 m_centerOfMassOffset.setIdentity;
 m_startWorldTrans    := startTrans;
 m_userPointer        := nil;
end;

constructor btDefaultMotionState.create(const startTrans, centerOfMassOffset: btTransform);
begin
  m_graphicsWorldTrans := startTrans;
  m_centerOfMassOffset := centerOfMassOffset;
  m_startWorldTrans    := startTrans;
  m_userPointer        := nil;
end;

procedure btDefaultMotionState.getWorldTransform(var centerOfMassWorldTrans: btTransform);
begin
 centerOfMassWorldTrans :=  m_centerOfMassOffset.inverse * m_graphicsWorldTrans ;
end;

procedure btDefaultMotionState.setWorldTransform(const centerOfMassWorldTrans: btTransform);
begin
  m_graphicsWorldTrans := centerOfMassWorldTrans * m_centerOfMassOffset ;
end;

end.
