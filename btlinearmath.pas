unit btLinearMath;
{$i fos_bullet.inc}
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

/*
Stan Melax Convex Hull Computation
Copyright (c) 2008 Stan Melax http://www.melax.com/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

---
*/

///includes modifications/improvements by John Ratcliff, see BringOutYourDead below.
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

// Bullet SVN Revision 2091 / 07.05.2010

// Files:
// -btQuadWord.h - skipped, using btVector4 as Baseclass, as btVector3 does not seem to use btQuadword as base ...
// +btScalar.h
// +btMinMax.h
// +btVector3.h // skipped serialization and endian swapping
// +btQuaternion.h
// +btMatrix3x3.h
// +btTransform.h
// +btTransformUtil.h
// +btRandom.h
// -btAllignedAllocator.h   / todo using fpc's methods
// -btAllignedAllocator.cpp / todo using fpc's methods
// -btAlignedObjectArray.h  / todo -> replace with fpc aligning methods
// -btPoolAllocator.h       / todo -> replace with dynarray
// +btAabbUtil2.h
// +btConvexHull.h
// +btConvexHull.cpp / Start
// +btGeometryUtil.h
// +btGeometryUtil.cpp


//TODO MAYBE -> Const References Wrong -> CHECK

interface

uses
  Classes, SysUtils,math,FOS_AlignedArray
  {$ifdef MSWINDOWS}
  ,windows
  {$endif}
  {$ifdef UNIX}
  ,Unix
  {$endif}
  ;

type
   {$IFDEF FRE_BULLET_DOUBLE_PRECISION}
     btScalar = double;
   {$ELSE}
     btScalar = single;
   {$ENDIF}
     TFOS_GL_Matrix       = array[0..15] of single;
     TFOS_GL_NormalMatrix = array[0..8] of single;

const
  SIMD_2_PI                  = btScalar(6.283185307179586232);
  SIMD_PI                    = btScalar(SIMD_2_PI * btScalar(0.5));
  SIMD_HALF_PI               = (SIMD_2_PI * btScalar(0.25));
  SIMD_RADS_PER_DEG:btScalar = (SIMD_2_PI / btScalar(360.0));
  SIMD_DEGS_PER_RAD:btScalar = (btScalar(360.0)/SIMD_2_PI);
  SIMDSQRT12:btScalar        = btScalar(0.7071067811865475244008443621048490);
  {$IFDEF FRE_BULLET_DOUBLE_PRECISION}
  SIMD_EPSILON               = 2.2204460492503131e-016 ;
  SIMD_INFINITY              = MaxDouble;
  {$ELSE}
  SIMD_EPSILON               = 1.192092896e-07 ;
  SIMD_INFINITY              = MaxSingle; // FLT_MAX
  {$ENDIF}
  ANGULAR_MOTION_THRESHOLD   = SIMD_HALF_PI * 0.5;

function  btDecide                (const a:boolean;const l1,l2:integer):integer;FOS_INLINE;overload;
function  btDecide                (const a:boolean;const l1,l2:cardinal):integer;FOS_INLINE;overload;
function  btDecide                (const a:boolean;const l1,l2:btScalar):btScalar;FOS_INLINE;overload;
function  btDecide                (const a:boolean;const l1,l2:boolean):boolean;FOS_INLINE;overload;
function  btDecide                (const a:boolean;const l1,l2:TObject):TObject;FOS_INLINE;overload;
function  btDecide                (const a:boolean;const l1,l2:Pointer):Pointer;FOS_INLINE;overload;
function  btGetversion            :integer;FOS_INLINE;
function  btNormalizeAngle        (angleInRadians:btScalar):btScalar;FOS_INLINE;
function  btUnswapEndianDouble    (const src:PByte):double;FOS_INLINE;
procedure btSwapEndianDouble      (const d:double;const dst:PByte);FOS_INLINE;
function  btUnSwapEndianFloat     (const a:cardinal):single;FOS_INLINE;
function  btSwapEndianFloat       (const d:single):cardinal;FOS_INLINE;
function  btSwapEndian            (const val:Cardinal):Cardinal;FOS_INLINE;overload;
function  btSwapEndian            (const val:integer):integer;FOS_INLINE;overload;
function  btSwapEndian            (const val:Word):Word;FOS_INLINE;overload;
function  btSwapEndian            (const val:SmallInt):SmallInt;FOS_INLINE;overload;
procedure btSwap                  (var a,b:btScalar);FOS_INLINE;overload; // define all needed on occasion
procedure btSwap                  (var a,b:Pointer);FOS_INLINE;overload; // define all needed on occasion
procedure btSwap                  (var a,b:Tobject);FOS_INLINE;overload; // define all needed on occasion
procedure btSwap                  (var a,b:integer);FOS_INLINE;overload; // define all needed on occasion
function  btMachineIsLittleEndian :boolean;FOS_INLINE;
function  btFsel                  (const a,b,c:btScalar):btScalar;FOS_INLINE;
function  btSqr                   (const x: btScalar): btScalar;FOS_INLINE;
function  btSqrt                  (const x:btScalar):btScalar;FOS_INLINE; //CHECK APPROXIMATION FROM ORIGINAL ?
function  btFabs                  (const x:btScalar):btscalar;FOS_INLINE;
function  btCos                   (const x:btScalar):btscalar;FOS_INLINE;
function  btSin                   (const x:btScalar):btscalar;FOS_INLINE;
function  btTan                   (const x:btScalar):btscalar;FOS_INLINE;
function  btAcos                  (const x:btScalar):btscalar;FOS_INLINE;
function  btAsin                  (const x:btScalar):btscalar;FOS_INLINE;
function  btAtan                  (const x:btScalar):btscalar;FOS_INLINE;
function  btAtan2                 (const x,y:btScalar):btscalar;FOS_INLINE;
function  btAtan2Fast             (const x,y:btScalar):btScalar;FOS_INLINE; // CHECK
function  btExp                   (const x:btScalar):btscalar;FOS_INLINE;
function  btLog                   (const x:btScalar):btscalar;FOS_INLINE;
function  btPow                   (const x,y:btScalar):btscalar;FOS_INLINE;
function  btFMod                  (const x,y:btScalar):btscalar;FOS_INLINE; //CHECK
function  btRecipSqrt             (const x:btScalar):btScalar;FOS_INLINE;
function  btFuzzyZero             (const x:btScalar):boolean;FOS_INLINE;
function  btEqual                 (const a,eps:btScalar):boolean;FOS_INLINE;
function  btGreaterEqual          (const a,eps:btScalar):boolean;FOS_INLINE;
function  btIsNegative            (const x:btScalar):integer;FOS_INLINE;
function  btRadians               (const x:btScalar):btScalar;FOS_INLINE;
function  btDegrees               (const x:btScalar):btScalar;FOS_INLINE;
function  btCeil                  (const x:btScalar):Integer;FOS_INLINE;
function  btFloor                 (const x:btScalar):Integer;FOS_INLINE;
function  btVarMinusMinus         (var i:integer):integer;FOS_INLINE;
function  btMinusMinusVar         (var i:integer):integer;FOS_INLINE;
function  btVarPlusPlus           (var i:integer):integer;FOS_INLINE;
function  btPlusPlusVar           (var i:integer):integer;FOS_INLINE;

// btMinMax.h (they were class and function templates -> strategy overload types)
function  btMin(const a,b:btScalar):btScalar;overload;FOS_INLINE;
function  btMin(const a,b:integer):integer;overload;FOS_INLINE;
function  btMax(const a,b:btScalar):btScalar;overload;FOS_INLINE;
function  btMax(const a,b:integer) :integer;overload;FOS_INLINE;
function  btClamped(const a,lb,ub:btScalar):btScalar;overload;FOS_INLINE;
procedure btSetMin(var a:btScalar; const b:btScalar);overload;FOS_INLINE;
procedure btSetMax(var a:btScalar; const b:btScalar);overload;FOS_INLINE;
procedure btClamp(var a:btScalar; const lb,ub:btScalar);overload;FOS_INLINE;

//btRandom.h
procedure GEN_srand(const seed:cardinal);
function  GEN_rand:cardinal;

//btVector3.h
type
  TFOS_m128   = array [0..7] of byte;
  {$PACKRECORDS 16}
  TFOS_vecrec = record
    case byte of
     1: (mVec128  : TFOS_m128);
     2: (m_floats : Array [0..3] of btScalar)
  end;

  PbtScalar = ^btScalar;
  PbtMAtrix3x3=^btMatrix3x3;
  PbtVector3=^btVector3;
  PPbtVector3=^PbtVector3;
  PbtVector4=^btVector4;


{$PACKRECORDS 16}

  { btVector2 }

  btVector2=object
     v:TFOS_vecrec;
     function  GetItem(i: integer): btScalar;FOS_INLINE;
     procedure SetItem(i: integer; const AValue: btScalar);FOS_INLINE;
     procedure Set128            (const v128:TFOS_m128);FOS_INLINE; //TODO SSE
     function  Get128            :TFOS_m128;FOS_INLINE;
     function  DumpValues        (const rot:boolean=false):string;
     function  DumpSimple        (const vec4:boolean=false):string;
     function  DumpSimpleFull    (const vec4:boolean=false):string;
     property  Items             [i:integer]:btScalar read GetItem write SetItem; default;
  end;

  btVector3=object(btVector2)
   public
     function  InitS             (const xx,yy,zz:btScalar):btVector3;static;FOS_INLINE;
     function  Init              (const xx,yy,zz:btScalar):btVector3;FOS_INLINE;  //TODO : make procedure, replace callers with static version
     procedure InitVector        (const v1:btVector3);FOS_INLINE;
     procedure InitSame          (const xx:btScalar);FOS_INLINE;
     function  InitSameS         (const xx:btScalar):btVector3;static;FOS_INLINE;
     function  dot               (const v1:btVector3):btScalar;FOS_INLINE;
     function  absDot            (const v1:btVector3):btScalar;FOS_INLINE;
     function  length2           :btScalar;FOS_INLINE;
     procedure length2NormCheck  ;FOS_INLINE; // FOS Shortcut function
     function  length            :btScalar;FOS_INLINE;
     function  normalized        :btVector3;FOS_INLINE;
     procedure normalize         ;FOS_INLINE;
     function  normalizef        :btScalar;FOS_INLINE;
     function  distance          (const v1:btVector3):btScalar;FOS_INLINE;
     function  distance2         (const v1:btVector3):btScalar;FOS_INLINE;
     function  rotate            (const wAxis:btVector3;const angle:btScalar):btVector3;FOS_INLINE;
     function  cross             (const v1:btVector3):btVector3;FOS_INLINE;
     function  angle             (const v1:btVector3):btScalar;FOS_INLINE;
     function  absolute          :btVector3;FOS_INLINE;
     procedure abs_self          ;FOS_INLINE;
     function  triple            (const v1,v2:btVector3):btScalar;FOS_INLINE;
     function  minAxis           :integer;FOS_INLINE; // return values are 0,1,2 for x, y, or z / Return the axis with the smallest value
     function  maxAxis           :integer;FOS_INLINE; // return values are 0,1,2 for x, y, or z / Return the axis with the largest value
     function  furthestAxis      :integer;FOS_INLINE;
     function  closestAxis       :integer;FOS_INLINE;
     procedure setInterpolate3   (const v0,v1:btVector3;const rt:btScalar);FOS_INLINE;
     function  lerp              (const v1: btVector3;const t:btScalar):btVector3;FOS_INLINE; // Return the linear interpolation between two vectors / t=ratio
     function  mulEqals          (const v1: btVector3):btVector3;FOS_INLINE; //   *=   // self := self * b1;
     function  addEqals          (const v1: btVector3):btVector3;FOS_INLINE; //   +=   // self := self + b1;
     function  _X                :PbtScalar;FOS_INLINE;
     function  _Y                :PbtScalar;FOS_INLINE;
     function  _Z                :PbtScalar;FOS_INLINE;
     function  getX              :btScalar;
     function  getY              :btScalar;
     function  getZ              :btScalar;
     function  X                 :btScalar;FOS_INLINE;
     function  Y                 :btScalar;FOS_INLINE;
     function  Z                 :btScalar;FOS_INLINE;
     procedure setX              (const s:btScalar);FOS_INLINE;
     procedure setY              (const s:btScalar);FOS_INLINE;
     procedure setZ              (const s:btScalar);FOS_INLINE;
     procedure setMax            (const v1:btVector3);FOS_INLINE; // Set each element to the max of the current values and the values of another btVector3
     procedure setMin            (const v1:btVector3);FOS_INLINE; // Set each element to the min of the current values and the values of another btVector3
     procedure zero              ;FOS_INLINE;
     procedure getSkewSymmetricMatrix(var v0,v1,v2:btVector3);FOS_INLINE;
     function  isZero            :boolean;FOS_INLINE;
     function  fuzzyZero         :boolean;FOS_INLINE;

   end;

   { btVector4 }

   btVector4=object(btVector3)
     function  Init4S            (const xx,yy,zz,ww:btScalar):btVector4;static;FOS_INLINE;
     function  Init4             (const xx,yy,zz,ww:btScalar):btVector4;FOS_INLINE; //this crashes investigate compiler problem
     function  _W                :PbtScalar;FOS_INLINE;
     function  W                 :btScalar;FOS_INLINE;
     function  getW              :btScalar;FOS_INLINE;
     procedure setW              (const s:btScalar);FOS_INLINE;
     function  absolute4         :btVector4;FOS_INLINE;
     function  maxAxis4          :integer;FOS_INLINE;
     function  minAxis4          :integer;FOS_INLINE;
     function  closestAxis4      :integer;FOS_INLINE;
   end;

   { btQuaternion }

   btQuaternion=object(btVector4)
     procedure setValue          (const xx,yy,zz,ww:btScalar);FOS_INLINE;
     function  Init4S            (const xx,yy,zz,ww:btScalar):btQuaternion;static;FOS_INLINE;
     function  InitQ             (const vector:btVector3 ;const vangle:btScalar):btQuaternion;FOS_INLINE;
     function  InitQS            (const vector:btVector3 ;const vangle:btScalar):btQuaternion;static;FOS_INLINE;
     function  InitYPR           (const yaw,pitch,roll:btScalar):btQuaternion;FOS_INLINE;
     procedure setRotation       (const axis:btVector3   ;const vangle:btScalar);FOS_INLINE;
     procedure setEuler          (const yaw,pitch,roll:btScalar);FOS_INLINE;
     procedure setEulerZYX       (const yaw,pitch,roll:btScalar);FOS_INLINE;
     procedure addEquals         (const q:btQuaternion);FOS_INLINE; // C: +=
     procedure subEquals         (const q:btQuaternion);FOS_INLINE; // C: -=
     procedure mulEquals         (const s:btScalar);FOS_INLINE;     // C: *=
     procedure mulEquals         (const q:btQuaternion);FOS_INLINE; // C: *=
     function  dot               (const q:btQuaternion):btScalar;FOS_INLINE; // dot Product
     function  length2           :btScalar;FOS_INLINE;
     function  length            :btScalar;FOS_INLINE;
     function  normalize         :btQuaternion;FOS_INLINE; //x^2+y^2+z^2+w^2=1 / Set it
     function  normalized        :btQuaternion;FOS_INLINE;
     function  angle             (const q:btQuaternion):btScalar;FOS_INLINE; // angle between quats
     function  getAngle          :btScalar;FOS_INLINE;  // angle this quat represents
     function  getAxis           :btVector3;FOS_INLINE; // axis this quat represents
     function  inverse           :btQuaternion;FOS_INLINE;
     function  sum               (const q:btQuaternion):btQuaternion;FOS_INLINE;
     function  diff              (const q:btQuaternion):btQuaternion;FOS_INLINE;
     function  farthest          (const qd:btQuaternion):btQuaternion;FOS_INLINE;
     function  nearest           (const qd:btQuaternion):btQuaternion;FOS_INLINE;
     function  slerp             (const q:btQuaternion;const t:btScalar):btQuaternion;FOS_INLINE;
     function  getIdentity       :btQuaternion;FOS_INLINE;
   end;
   PbtQuaternion=^btQuaternion;

   { btMatrix }

   { btMatrix3x3 }

   btMatrix3x3=object
   private
     m_el:array [0..2] of btVector3;
     function  GetItems(i: integer): btVector3;FOS_INLINE;
     function  GetItemsP(i: integer): PbtVector3;FOS_INLINE;
     procedure SetItems(i: integer; const AValue: btVector3);FOS_INLINE;
   public
     procedure Init                    (const q:btQuaternion);
     function  InitS                   (const q:btQuaternion):btMatrix3x3;static;FOS_INLINE;
     procedure Init                    (const xx,xy,xz,yx,yy,yz,zx,zy,zz:btScalar);
     function  InitS                   (const xx,xy,xz,yx,yy,yz,zx,zy,zz:btScalar):btMatrix3x3;static;
     procedure setRotation             (const q:btQuaternion);
     function  getColumn               (const i:integer):btVector3;
     function  getRow                  (const i:integer):btVector3;
     function  mulEquals               (const m:btMatrix3x3):btMatrix3x3;  // C: *=
     procedure setfromOpenGLSubMatrix  (const m:PbtScalar);
     procedure getOpenGLSubMatrix      (const m:PbtScalar);
     procedure getOpenGLNormalMatrix   (const m:PbtScalar);
     procedure setEulerYPR             (const yaw,pitch,roll:btScalar);
     procedure setEulerZYX             (const eulerX,eulerY,eulerZ:btScalar);
     procedure setIdentity             ;
     function  getIdentity             :btMatrix3x3;static;
     procedure getRotation             (out  q:btQuaternion);
     procedure getEulerYPR             (var yaw,pitch,roll:btScalar);
     procedure getEulerZYX             (var yaw,pitch,roll:btscalar;var solution_number:cardinal=1);
     function  scaled                  (const s:btVector3):btMatrix3x3;
     function  determinant             :btScalar;
     function  adjoint                 :btMatrix3x3;
     function  absolute                :btMatrix3x3;
     function  transpose               :btMatrix3x3;
     function  inverse                 :btMatrix3x3;
     function  transposeTimes          (const m:btMatrix3x3):btMatrix3x3;
     function  timesTranspose          (const m:btMatrix3x3):btMatrix3x3;
     function  cofac                   (r1,c1,r2,c2:integer):btScalar;
     function  dump                    :string;
     procedure diagonalize             (const rot:btMatrix3x3;threshold:btScalar;const maxSteps:Integer);
     function  tdotx                   (const v:btVector3):btScalar;
     function  tdoty                   (const v:btVector3):btScalar;
     function  tdotz                   (const v:btVector3):btScalar;
     property  Rows                    [i:integer]:btVector3 read GetItems write SetItems;default;
     property  RowsP                   [i:integer]:PbtVector3 read GetItemsP;
   end;

   { btMatrix4x4 }

   btMatrix4x4=object
     m_el      : array [0..3] of btVector4;
     function  GetItems(i: integer): btVector4;FOS_INLINE;
     function  GetItemsP(i: integer): PbtVector4;FOS_INLINE;
     procedure SetItems(i: integer; const AValue: btVector4);FOS_INLINE;
   public
     procedure Init                    (const xx,xy,xz,xw,yx,yy,yz,yw,zx,zy,zz,zw, wx, wy, wz, ww:btScalar);
     function  InitS                   (const xx,xy,xz,xw,yx,yy,yz,yw,zx,zy,zz,zw, wx, wy, wz, ww:btScalar):btMatrix4x4;static;
     function  tdotx                   (const v:btVector4):btScalar;
     function  tdoty                   (const v:btVector4):btScalar;
     function  tdotz                   (const v:btVector4):btScalar;
     function  tdotw                   (const v:btVector4):btScalar;
     function  dump                    :string;
     procedure setfromOpenGLMatrix     (const m:PbtScalar);
     property  Rows                    [i:integer]:btVector4 read GetItems write SetItems;default;
     property  RowsP                   [i:integer]:PbtVector4 read GetItemsP;
   end;

   { btTransform }

   btTransform=object
   private
     m_basis:btMatrix3x3;
     m_origin:btVector3;
   public
     procedure Init                (const q:btQuaternion;const c:btVector3);
     procedure Init                (const b:btMatrix3x3;const c:btVector3);
     procedure Init                (const b:btMatrix3x3);
     procedure mult                (const t1,t2:btTransform);//inline;
     function  getRotation         :btQuaternion;
     function  opTrans             (const x:btVector3):btVector3;//inline;  // => C: operator() is transform of the vector
     function  opMul               (const x:btVector3):btVector3;//inline;
     function  opMul               (const q:btQuaternion):btQuaternion;
     function  opMul               (const t:btTransform):btTransform;
     function  getBasisV           :PbtMatrix3x3;
     function  getOriginV          :PbtVector3;
     function  dump                :string;
     procedure setFromOpenGLMatrix (const m:PbtScalar);
     procedure getOpenGLMatrix     (const m:PbtScalar);
     procedure setOrigin           (const origin:btVector3);
     procedure setBasis            (const m:btMatrix3x3);
     procedure setRotation         (const q:btQuaternion);
     procedure setIdentity         ;
     function  mulEquals           (const t:btTransform):btTransform; // C: operator *=
     function  inverse             :btTransform;
     function  inverseTimes        (const t:btTransform):btTransform;
     function  identityTransform   :btTransform;
     function  invXform            (const inVec:btVector3):btVector3;
     property  Origin              :btVector3 read m_origin write m_origin;
   end;
   PbtTransform=^btTransform;

   { btTransformUtil }

   btTransformUtil=object
     procedure integrateTransform               (const curTrans              :btTransform;  const linvel,angvel :btVector3;    const timeStep      :btScalar; out  predictedTransform :btTransform);static;
     procedure calculateVelocityQuaternion      (const pos0,pos1             :btVector3;    const orn0,orn1     :btQuaternion; const timeStep      :btScalar; out  linVel,angVel      :btVector3);static;
     procedure calculateDiffAxisAngleQuaternion (const orn0,orn1a            :btQuaternion; out   axis          :btVector3;    out   angle         :btScalar);static;
     procedure calculateVelocity                (const transform0,transform1 :btTransform;  const timeStep      :btScalar;     out   linVel,angVel :btVector3);static;
     procedure calculateDiffAxisAngle           (const transform0,transform1 :btTransform;  out   axis          :btVector3;    out   angle         :btScalar); static;
   end;

   { btConvexSeparatingDistanceUtil }

   btConvexSeparatingDistanceUtil=object
   private
     m_ornA,m_ornB        :btQuaternion;
     m_posA,m_posB,
     m_separatingNormal   :btVector3;
     m_boundingRadiusA,
     m_boundingRadiusB,
     m_separatingDistance :btScalar;
   public
     procedure   init                              (const boundingRadiusA,boundingRadiusB:btScalar);
     function  	 getConservativeSeparatingDistance :btScalar;
     procedure   updateSeparatingDistance          (const transA,transB:btTransform);
     procedure   initSeparatingDistance            (const separatingVector:btVector3;const separatingDistance:btScalar;const transA,transB:btTransform);
   end;


  { btHullResult }
  btFOSAlignedVectorArray=specialize FOS_GenericAlignedArray<btVector3>;
  btHullFlag = (
  	        QF_TRIANGLES     =1,             // report results as triangles, not polygons.
	        QF_REVERSE_ORDER =2              // reverse order of the triangle indices.
                );

  btHullFlagSet = set of btHULLFLAG;
  btHullError   = (QE_OK,QE_FAIL);

  btPlaneTestFlag    = (btptCOPLANAR=0,btptUNDER=1,btptOVER=2,btptSPLIT=(btptOVER+btptUNDER));
  btPlaneTestFlagSet = set of btPlaneTestFlag;

  const btcPAPERWIDTH = btScalar(0.001);
        QF_DEFAULT:btHullflag =QF_TRIANGLES;

  var   btvPlanetestEpsilon:btScalar = btcPAPERWIDTH;

type

  { btGeometryUtil }

  btGeometryUtil=object
     procedure  getPlaneEquationsFromVertices  (const vertices       : btFOSAlignedVectorArray ; var   planeEquationsOut : btFOSAlignedVectorArray   );static;
     procedure  getVerticesFromPlaneEquations  (const planeEquations : btFOSAlignedVectorArray ; var   verticesOut       : btFOSAlignedVectorArray   );static;
     //function   isInside                       (const vertices       : btFOSAlignedVectorArray16 ; const planeNormal       : btVector3                 ; const margin:btScalar ):boolean;static; c++ impl not found
     function   isPointInsidePlanes            (const planeEquations : btFOSAlignedVectorArray ; const point             : btVector3                 ; const margin:btScalar ):boolean;static;
     function   areVerticesBehindPlane         (const planeNormal    : btVector3                 ; const vertices          : btFOSAlignedVectorArray ; const margin:btScalar ):boolean;
   end;


type

  btofVertFlag=object
    planetest,
    junk,
    undermap,
    overmap    :byte;
  end;
  btofEdgeFlag=object
    planetest,
    fixes:     byte;
    undermap,
    overmap    :SmallInt;
  end;
  btofPlaneFlag=object
    undermap,
    overmap   :Byte;
  end;
  btofCoplanar=object
    ea:word;
    v0:byte;
    v1:byte;
  end;

  btHullResult=class
    mPolygons           :Boolean;                    // true if indices represents polygons, false indices are triangles
    mNumOutputVertices  :cardinal;                   // number of vertices in the output hull
    m_OutputVertices    :btFOSAlignedVectorArray;    // array of vertices
    mNumFaces           :Cardinal;                   // the number of faces produced
    mNumIndices         :Cardinal;                   // the total number of indices
    m_Indices           :TFOS_AlignedCardinals;      // pointer to indices.
    // If triangles, then indices are array indexes into the vertex list.
    // If polygons, indices are in the form (number of points in face) (p1, p2, p3, ..) etc..
  public
    constructor create;
    destructor  Destroy;override;
  end;

  { btHullDesc }

  btHullDesc=object
  public
      mFlags         :btHullFlagSet; // flags to use when generating the convex hull.
      mVcount        :Cardinal;     // number of vertices in the input point cloud
      mVertices      :PbtVector3;   // the array of vertices.
      mVertexStride  :Cardinal;     // the stride of each vertex, in bytes.
      mNormalEpsilon :btScalar;     // the epsilon for removing duplicates.  This is a normalized value, if normalized bit is on.
      mMaxVertices   :Cardinal;     // maximum number of vertices to be considered for the hull!
      mMaxFaces      :Cardinal;
      procedure      Init          ;
      procedure      Init          (const flags:btHullFlagSet;const vcount:cardinal;const vertices:PbtVector3;const stride:cardinal=sizeof(btVector3));
      function       HasHullFlag   (const flag:btHullFlag):boolean;
      procedure      SetHullFlag   (const flag:btHullFlag);
      procedure      ClearHullFlag (const flag:btHullFlag);
  end;

  { btPlane }

  btPlane=object
    normal   : btVector3;
    dist     : btScalar;   // distance below origin - the D from plane equasion Ax+By+Cz+D=0
    procedure Init (const v:btVector3;const d:btScalar);
  end;

    { btHalfEdge }

  btHalfEdge=object
    ea:SmallInt; // the other half of the edge (index into edges list)
    v :Byte;     // the vertex at the start of this edge (index into vertices list)
    p :Byte;     // the facet on which this edge lies (index into facets list)
    procedure Init(const _ea:SmallInt;const _v,_p:Byte);
  end;

  btFOSAlignedHalfEdges =specialize FOS_GenericAlignedArray<btHalfEdge>;
  btFOSAlignedPlanes    =specialize FOS_GenericAlignedArray<btPlane>;

  { btConvexH }

  btConvexH=object
    vertices : btFOSAlignedVectorArray;
    edges    : btFOSAlignedHalfEdges;
    facets   : btFOSAlignedPlanes;
    procedure Init(const vertices_size,edges_size,facets_size:integer);
  end;
  PbtConvexH=^btConvexH;


  { btPHullResult }

  btPHullResult=class
    mVcount     : cardinal;
    mIndexCount : cardinal;
    mFaceCount  : cardinal;
    mVertices   : PbtVector3;
    m_indices   : TFOS_AlignedCardinals;
    constructor Create;
    destructor  Destroy;override;
  end;

  { btHullTriangle }

  { btInt3 }

  btInt3=class
  private
    function  GetItem (const i: integer): integer;FOS_INLINE;
    function  GetItemP(const i: integer): PInteger;
    procedure SetItem(const i: integer; const AValue: Integer);FOS_INLINE;
  public
    x,y,z:integer;
    procedure Initialize(const _x,_y,_z:integer);inline;
    property  Item  [const i:integer]:Integer read GetItem write SetItem ;default;
    property  ItemV [const i:integer]:PInteger read GetItemP;
  end;

  { btInt4 }

  btInt4=class
  private
    function GetItemP(const i: integer): PInteger;
  public
    x,y,z,w:integer;
    constructor create(const _x,_y,_z,_w:integer);
    property  Item[const i:integer]:PInteger read GetItemP;default;
  end;

  btHullTriangle=class(btInt3)
    n     : btInt3;
    id    : integer;
    vmax  : integer;
    rise  : btScalar;
    constructor create(const a,b,c:integer);
    destructor  Destroy;override;
    function    neib(const a,b:integer):PInteger;
  end;

//  PbtHullTriangle=^btHullTriangle;
  btHullTriangleA=array [0..0] of btHullTriangle;

  btFOSAlignedHullTrianglePointers    = specialize FOS_GenericAlignedArray<btHullTriangle>;
  btFOSAlignedScalars                 = specialize FOS_GenericAlignedArray<btScalar>;
  btFOSAlignedIntegers                = TFOS_AlignedIntegers;
  btFOSAlignedCardinals               = TFOS_AlignedCardinals;
  btCardinal16                        = record
    v          : Cardinal;
    _d1,_d2,d3 : Cardinal;
  end;
  PbtCardinal16                       = ^btCardinal16;

//  PbtFOSAlignedHullTriangles   =^PbtFOSAlignedPlanes;



    ///The HullLibrary class can create a convex hull from a collection of vertices, using the ComputeHull method.
    ///The btShapeHull class uses this HullLibrary to create a approximate convex mesh given a general (non-polyhedral) convex shape.

    { btHullLibrary }

    btHullLibrary=class
    private
      m_tris               : btFOSAlignedHullTrianglePointers;
      m_vertexIndexMapping : TFOS_AlignedIntegers;

      function             ComputeHull         (const vcount:cardinal;const vertices:PbtVector3;var res:btPHullResult;const vlimit:cardinal):boolean;
      function             allocateTriangle    (const a,b,c:integer):btHullTriangle;
      procedure            deAllocateTriangle  (const triangle:btHullTriangle);
      procedure            b2bFix              (const s,t:btHullTriangle);
      procedure            removeb2b           (const s,t:btHullTriangle);
      procedure            checkit             (const t:btHullTriangle);
      function             extrudable          (const epsilon:btScalar):btHullTriangle;
      function             calchull            (const verts:PbtVector3;const verts_count:integer;var tris_out:TFOS_AlignedCardinals;var tris_count:integer;const vlimit:integer):integer;
      function             calchullgen         (const verts:PbtVector3;const verts_count:integer;vlimit:integer):integer;
      function             FindSimplex         (const verts:PbtVector3;const verts_count:integer;var allow:TFOS_AlignedIntegers):btInt4;
//      function             ConvexHCrop         (var   convex:btConvexH;const slice:btPlane):PbtConvexH;
      procedure            extrude             (const t0:btHullTriangle;const v:integer);
//      function             test_cube           :PbtConvexH;
      procedure            BringOutYourDead    (const hr:btPHullResult; const overts:PbtVector3;var ocount:cardinal);
          //BringOutYourDead (John Ratcliff): When you create a convex hull you hand it a large input set of vertices forming a 'point cloud'.
          //After the hull is generated it give you back a set of polygon faces which index the *original* point cloud.
          //The thing is, often times, there are many 'dead vertices' in the point cloud that are on longer referenced by the hull.
          //The routine 'BringOutYourDead' find only the referenced vertices, copies them to an new buffer, and re-indexes the hull so that it is a minimal representation.
      //    void BringOutYourDead(const btVector3* verts,unsigned int vcount, btVector3* overts,unsigned int &ocount,unsigned int* indices,unsigned indexcount);
      function             CleanupVertices     (const svcount:cardinal;const svertices:PbtVector3;const stride:cardinal;var vcount:cardinal;const vertices:PbtVector3;const normalepsilon:btScalar;const scale:PbtVector3):boolean;
      //    bool CleanupVertices(unsigned int svcount,
      //                         const btVector3* svertices,
      //                         unsigned int stride,
      //                         unsigned int &vcount, // output number of vertices
      //                         btVector3* vertices, // location to store the results.
      //                         btScalar  normalepsilon,
      //                         btVector3& scale);
      //};
    public
      constructor  create;
      destructor   destroy;override;
      function     CreateConvexHull(const desc:btHullDesc;var res:btHullResult):btHullError;
      function     ReleaseResult   (var   res:btHullResult):btHullError; //release memory allocated for this result, we are done with it.
    end;

    btClockData = record
    {$ifdef MSWINDOWS}
	mClockFrequency : Int64;
	mStartTick      : Cardinal;
	mPrevElapsedTime: Int64;
	mStartTime      : int64;
     {$else}
	mStartTime      : timeval;
     {$endif}
    end;

    ///The btClock is a portable basic clock that measures accurate time in seconds, use for profiling.

    { btClock }

    btClock = object
    private
      m_data : btClockData;
    public
      procedure init;
      procedure copy(const other:btClock);
      procedure reset;
      function  getTimeMilliseconds : Cardinal;        /// Returns the time in ms since the last call to reset or since the btClock was created.
      function  getTimeMicroseconds : int64;         /// Returns the time in us since the last call to reset or since the Clock was created.
    end;


//
// Operators and Utility Functions
//

 //Vector3 operators
   function  btDot         (const v1,v2:btVector3):btScalar;FOS_INLINE;
   function  btDistance    (const v1,v2:btVector3):btScalar;FOS_INLINE;
   function  btDistance2   (const v1,v2:btVector3):btScalar;FOS_INLINE;
   function  btAngle       (const v1,v2:btVector3):btScalar;FOS_INLINE;
   function  btCross       (const v1,v2:btVector3):btVector3;FOS_INLINE;
   function  btTriple      (const v1,v2,v3:btVector3):btScalar;FOS_INLINE;
   function  btLerp        (const v1,v2:btVector3;const t:btScalar):btVector3;FOS_INLINE;
   procedure btPlaneSpace1 (const n:btVector3; out p,q:btVector3);FOS_INLINE;
   operator + (const v1,v2:btVector3) v3:btVector3;FOS_INLINE;
   operator - (const v1,v2:btVector3) v3:btVector3;FOS_INLINE;
   operator * (const v1:btVector3;const s:btScalar) v3:btVector3;FOS_INLINE;
   operator * (const s:btScalar;  const v1:btVector3) v3:btVector3;FOS_INLINE;
   operator * (const v1:btVector3;const v2:btVector3) v3:btVector3;FOS_INLINE; // Element wise product
   operator / (const v1:btVector3;s:btScalar) v3:btVector3;FOS_INLINE;
   operator / (const v1,v2:btVector3) vres:btVector3;FOS_INLINE;
   operator = (const v1,v2:btVector3) res:boolean;FOS_INLINE;
   operator -(const v1:btVector3) :btVector3;FOS_INLINE;

  //Vector4 operators
   operator + (const v1,v2:btVector4) v3:btVector4;FOS_INLINE;
   operator - (const v1,v2:btVector4) v3:btVector4;FOS_INLINE;

   //Quaternion Operators
   operator + (const v1,v2:btQuaternion) v3:btQuaternion;FOS_INLINE;
   operator - (const v1,v2:btQuaternion) v3:btQuaternion;FOS_INLINE;
   operator - (const v1:btQuaternion) v3:btQuaternion;FOS_INLINE;
   operator * (const v1,v2:btQuaternion) v3:btQuaternion;FOS_INLINE;
   operator * (const v1:btQuaternion;const s:btScalar)  v3:btQuaternion;FOS_INLINE;
   operator * (const q :btQuaternion;const w:btVector3) v3:btQuaternion;FOS_INLINE;
   operator * (const w :btVector3   ;const q:btQuaternion) v3:btQuaternion;FOS_INLINE;
   operator / (const v1:btQuaternion;const s:btScalar)  v3:btQuaternion;FOS_INLINE;

   function  btDot                      (const q1,q2:btQuaternion):btScalar;FOS_INLINE;
   function  btLength                   (const q    :btQuaternion):btScalar;FOS_INLINE;
   function  btAngle                    (const q1,q2:btQuaternion):btScalar;FOS_INLINE;
   function  btInverse                  (const q:btQuaternion):btQuaternion;FOS_INLINE;
   function  btSlerp                    (const q1,q2:btQuaternion;const t:btScalar):btQuaternion;FOS_INLINE;
   function  btQuatRotate               (const rotation:btQuaternion;const v:btVector3):btVector3;FOS_INLINE;
   function  btShortestArcQuat          (const v0,v1:btVector3):btQuaternion;FOS_INLINE; // Game Programming Gems 2.10. make sure v0,v1 are normalized
   function  btShortestArcQuatNormalize (const v0,v1:btVector3):btQuaternion;FOS_INLINE; // Game Programming Gems 2.10. make sure v0,v1 are normalized
   function  calcWindowXY                (const objx,objy,objz: Single;const modelview,projection: TFOS_GL_Matrix;const viewport:PIntegerArray;var wx,wy,wz:Single):Boolean;FOS_INLINE;
   function  calcQuatFromFwdVec          (fwd: btVector3):btQuaternion;
   function  calcProjectionMatrix        (const fovyInDegrees,aspectRatio,znear,zfar:Single; var aabbMin,aabbMax: btVector3):TFOS_GL_Matrix;

   //Matrix3x3 Operators
   operator * (const m     :btMatrix3x3 ;const v :btVector3  ) :btVector3;FOS_INLINE;
   operator * (const v     :btVector3   ;const m :btMatrix3x3) :btVector3;FOS_INLINE;
   operator * (const m1    :btMatrix3x3 ;const m2:btMatrix3x3) :btMatrix3x3;
   operator = (const m1,m2 :btMatrix3x3 ) res:boolean;FOS_INLINE;

   //Matrix4x4 Operators
   operator * (const m     :btMatrix4x4 ;const v :btVector4  ) :btVector4;FOS_INLINE;
   operator * (const v     :btVector4   ;const m :btMatrix4x4) :btVector4;FOS_INLINE;
   operator * (const m1    :btMatrix4x4 ;const m2:btMatrix4x4) :btMatrix4x4;
   operator = (const m1,m2 :btMatrix4x4 ) res:boolean;FOS_INLINE;


   //transform Operators
   operator = (const t1,t2: btTransform):boolean;FOS_INLINE;
   operator * (const t1,t2: btTransform):btTransform;FOS_INLINE;
   operator * (const t1:btTransform ; const v2:btVector3 ):btVector3;FOS_INLINE;

   function  btDecide                (const a:boolean;const l1,l2:btVector3):btVector3;FOS_INLINE;overload;
   function  btDecide                (const a:boolean;const l1,l2:PbtVector3):PbtVector3;FOS_INLINE;overload;



// Utility Functions

 function btAabbSupport(const halfExtents,supportDir:btVector3):btVector3;FOS_INLINE;

 // btAabbutil2.h
 type
   TbtFOSSignArray   = array [0..2] of cardinal;
   TbtFOSboundsArray = array [0..1] of btVector3;

   procedure AabbExpand               (var   aabbMin, aabbMax:btVector3;const expansionMin,expansionMax:btVector3);FOS_INLINE;
   function  TestPointAgainstAabb2    (const aabbMin1,aabbMax1,point:btVector3):boolean;FOS_INLINE;
   function  TestAabbAgainstAabb2     (const aabbMin1,aabbMax1,aabbMin2,aabbMax2:btVector3):boolean;FOS_INLINE;
   function  TestTriangleAgainstAabb2 (const vertices:PbtVector3; const aabbMin,aabbMax:btVector3):boolean;FOS_INLINE;
   function  btOutcode                (const p,halfExtent:btVector3):integer;FOS_INLINE;
   function  btRayAabb2               (const rayFrom,rayInvDirection:btVector3;const raySign:TbtFOSSignArray;const bounds:TbtFOSboundsArray;var tmin:btScalar;const lambda_min,lambda_max:btScalar):boolean;FOS_INLINE;
   function  btRayAabb                (const rayFrom,rayTo,aabbMin,aabbMax:btVector3;var param:btScalar;out normal: btVector3):boolean;FOS_INLINE;
   procedure btTransformAabb          (const halfExtents:btVector3;const margin:btScalar;const t:btTransform;out aabbMinOut,aabbMaxOut:btVector3);FOS_INLINE;
   procedure btTransformAabb          (const localAabbMin,localAabbMax:btVector3;const margin:btScalar;const trans:btTransform;out aabbMinOut,aabbMaxOut :btVector3);FOS_INLINE;
   function  testQuantizedAabbAgainstQuantizedAabb(const aabbMin1, aabbMax1, aabbMin2, aabbMax2: PWord):LongBool;FOS_INLINE;


// btConvexHull Functions
  function btPlaneFlip              (const plane:btPlane):btPlane;FOS_INLINE;
  function btCoPlanar               (const a,b:btPlane):boolean;FOS_INLINE;
  function btPlaneLineIntersection  (const plane:btPlane;const p0,p1:btVector3):btVector3;FOS_INLINE;
  function btPlaneProject           (const plane:btPlane;const point:btVector3):btVector3;FOS_INLINE;
  function btThreePlaneIntersection (const p0,p1,p2:btPlane):btVector3;FOS_INLINE;
  function btDistanceBetweenLines   (const ustart,udir,vstart,vdir:btVector3; const upoint:PbtVector3=nil; const vpoint:PbtVector3=nil):btScalar;FOS_INLINE;
  function btTriNormal              (const v0,v1,v2:btVector3):btVector3;FOS_INLINE;
//  function btNormalOf               (const vert:PbtVector3;const n:integer):btVector3; //TODO defined but not found in source
  function btPlaneTest              (const p:btPlane; const v:btVector3):btPlaneTestFlag;FOS_INLINE;
  function btSplitTest              (const convex:btConvexH;const plane:btPlane):btPlaneTestFlagSet;FOS_INLINE;
  function btOrth                   (const v:btVector3):btVector3;FOS_INLINE;
  function btAbove                  (const vertices:PbtVector3;const t:btInt3; const p:btVector3;const epsilon:btScalar):boolean;FOS_INLINE;
  function btHasEdge                (const t:btInt3;const a,b:integer):boolean;FOS_INLINE;
  function btHasVert                (const t:btInt3;const v:integer):boolean;FOS_INLINE;
  function btShareEdge              (const a,b:btInt3):boolean;FOS_INLINE;

// btConvexHull Operators
  operator =(const a,b:btplane):boolean;FOS_INLINE;
//  operator =(const a,b:btInt3) :boolean;FOS_INLINE;

  const
     cbtNullVector : btVector3 = (v:(m_floats:(0,0,0,0)));
     cbtOneVector  : btVector3 = (v:(m_floats:(1,1,1,1)));

//
//
// Implementation Section
//
//
implementation


function btDecide(const a: boolean; const l1, l2: integer): integer;
begin
  if a then begin
    result := l1;
  end else begin
    result := l2;
  end;
end;

function btDecide(const a: boolean; const l1, l2: cardinal): integer;
begin
  if a then begin
    result := l1;
  end else begin
    result := l2;
  end;
end;

function btDecide(const a: boolean; const l1, l2: btScalar): btScalar;
begin
 if a then begin
   result := l1;
 end else begin
   result := l2;
 end;
end;

function btDecide(const a: boolean; const l1, l2: boolean): boolean;
begin
 if a then begin
   result := l1;
 end else begin
   result := l2;
 end;
end;

function btDecide(const a: boolean; const l1, l2: TObject): TObject;
begin
 if a then begin
   result := l1;
 end else begin
   result := l2;
 end;
end;

function btDecide(const a: boolean; const l1, l2: Pointer): Pointer;
begin
 if a then begin
   result := l1;
 end else begin
   result := l2;
 end;
end;

function btGetversion:integer;FOS_INLINE;
begin
  result:=cbtBULLET_VERSION;
end;

function btSqr(const x: btScalar): btScalar;FOS_INLINE;
begin
  result:=sqr(x);
end;

{.$IFDEF FRE_BULLET_DOUBLE_PRECISION}  // || defined(BT_FORCE_DOUBLE_FUNCTIONS)
function btSqrt(const x:btScalar):btScalar;FOS_INLINE; //CHECK APPROXIMATION FROM ORIGINAL ?
begin
  result:=sqrt(x);
end;
function btFabs(const x:btScalar):btscalar;FOS_INLINE;
begin
  result:=abs(x);
end;
function btCos(const x:btScalar):btscalar;FOS_INLINE;
begin
  result:=cos(x);
end;
function btSin(const x:btScalar):btscalar;FOS_INLINE;
begin
  result:=sin(x);
end;
function btTan(const x:btScalar):btscalar;FOS_INLINE;
begin
  result:=tan(x);
end;
function btAcos(const x:btScalar):btscalar;FOS_INLINE;
begin
  result:=arccos(x);
end;
function btAsin(const x:btScalar):btscalar;FOS_INLINE;
begin
  result:=arcsin(x);
end;
function btAtan(const x:btScalar):btscalar;FOS_INLINE;
begin
  result:=arctan(x);
end;
function btAtan2(const x,y:btScalar):btscalar;FOS_INLINE;
begin
  result:=arctan2(x,y);
end;

const
  coeff_1          = btScalar(SIMD_PI / 4.0);
  coeff_2:btScalar = 3.0 * coeff_1;

function btAtan2Fast(const x,y:btScalar):btScalar;FOS_INLINE; // CHECK
var r,abs_y,angle:btScalar;
begin
  abs_y := btFabs(y);
  if x>=0 then begin
    r     := (x - abs_y) / (x + abs_y);
    angle := coeff_1 - coeff_1 * r;
  end else begin
    r     := (x + abs_y) / (abs_y - x);
    angle := coeff_2 - coeff_1 * r;
  end;
  if y<0 then begin
    result:=-angle;
  end else begin
    result:=angle;
  end;
end;
function btExp(const x:btScalar):btscalar;FOS_INLINE;
begin
  result:=exp(x);
end;
function btLog(const x:btScalar):btscalar;FOS_INLINE;
begin
  result:=log10(x);
end;
function btPow(const x,y:btScalar):btscalar;FOS_INLINE;
begin
  result:=power(x,y);
end;
function btFMod(const x,y:btScalar):btscalar;FOS_INLINE; //CHECK
var Z:btScalar;
    temp:btScalar;
begin
  temp := X/Y;
  Z := trunc(temp);
  if temp < 0 then Z := Z - 1.0;
  Result := X - (Z*Y);
end;


function btRecipSqrt(const x:btScalar):btScalar;FOS_INLINE;
begin
  result:= 1 / btSqrt(x);
end;
function btFuzzyZero(const x:btScalar):boolean;FOS_INLINE;
begin
  result:= btFabs(x)<SIMD_EPSILON;
end;
function btEqual(const a,eps:btScalar):boolean;FOS_INLINE;
begin
  result := (a<=eps) and (not(a<-eps));
end;
function btGreaterEqual(const a,eps:btScalar):boolean;FOS_INLINE;
begin
  result := not(a<=eps);
end;

function btIsNegative(const x:btScalar):integer;FOS_INLINE;
begin
  if x<btScalar(0) then begin
    exit(1);
  end else begin
    exit(0);
  end;
end;
function btRadians(const x:btScalar):btScalar;FOS_INLINE;
begin
  result:=x*SIMD_RADS_PER_DEG;
end;
function btDegrees(const x:btScalar):btScalar;FOS_INLINE;
begin
  result:=x*SIMD_DEGS_PER_RAD;
end;

function btCeil(const x: btScalar): Integer;FOS_INLINE;
begin
  Result:=ceil(x);
end;

function btFloor(const x: btScalar): Integer;FOS_INLINE;
begin
  Result:=floor(x);
end;

function btVarMinusMinus(var i: integer): integer;
begin
 result := i;
 i:=i-1;
end;

function btMinusMinusVar(var i: integer): integer;
begin
 i:=i-1;
 result := i;
end;

function btVarPlusPlus(var i: integer): integer;
begin
 result := i;
 i:=i+1;
end;

function btPlusPlusVar(var i: integer): integer;
begin
 i:=i+1;
 result := i;
end;


//#define BT_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

function btFsel(const a,b,c:btScalar):btScalar;FOS_INLINE;
begin
  if a>=0 then begin
    result:=b;
  end else begin
    result:=c;
  end;
end;

procedure btSwap(var a, b: Pointer);
var tmp:Pointer;
begin
  tmp:=a;a:=b;b:=tmp;
end;

procedure btSwap(var a, b: Tobject);
var tmp:Tobject;
begin
  tmp:=a;a:=b;b:=tmp;
end;

procedure btSwap(var a, b: integer);
var tmp:integer;
begin
  tmp:=a;a:=b;b:=tmp;
end;

function btMachineIsLittleEndian:boolean;FOS_INLINE;
var i:integer=1;
    p:pchar;
begin
  p:=@i;
  if byte(p[0])=1 then begin
    exit(true);
  end else begin
    exit(false);
  end;
end;

//#define btFsels(a,b,c) (btScalar)btFsel(a,b,c)

//SIMD_FORCE_INLINE unsigned btSelect(unsigned condition, unsigned valueIfConditionNonZero, unsigned valueIfConditionZero)
//{
//    // Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
//    // Rely on positive value or'ed with its negative having sign bit on
//    // and zero value or'ed with its negative (which is still zero) having sign bit off
//    // Use arithmetic shift right, shifting the sign bit through all 32 bits
//    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
//    unsigned testEqz = ~testNz;
//    return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
//}
//SIMD_FORCE_INLINE int btSelect(unsigned condition, int valueIfConditionNonZero, int valueIfConditionZero)
//{
//    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
//    unsigned testEqz = ~testNz;
//    return static_cast<int>((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
//}
//SIMD_FORCE_INLINE float btSelect(unsigned condition, float valueIfConditionNonZero, float valueIfConditionZero)
//{
//#ifdef BT_HAVE_NATIVE_FSEL
//    return (float)btFsel((btScalar)condition - btScalar(1.0f), valueIfConditionNonZero, valueIfConditionZero);
//#else
//    return (condition != 0) ? valueIfConditionNonZero : valueIfConditionZero;
//#endif
//}


//template<typename T> SIMD_FORCE_INLINE void btSwap(T& a, T& b)
//{
//	T tmp = a;
//	a = b;
//	b = tmp;
//}

procedure btSwap(var a,b:btScalar);FOS_INLINE;overload; // define all needed on occasion
var tmp:btScalar;
begin
  tmp:=a;a:=b;b:=tmp;
end;

function btSwapEndian(const val:Cardinal):Cardinal;FOS_INLINE;overload;
begin
  result:=SwapEndian(val);
end;
function btSwapEndian(const val:integer):integer;FOS_INLINE;overload;
begin
  result:=SwapEndian(val);
end;
function btSwapEndian(const val:Word):Word;FOS_INLINE;overload;
begin
  result:=SwapEndian(val);
end;
function btSwapEndian(const val:SmallInt):SmallInt;FOS_INLINE;overload;
begin
  result:=SwapEndian(val);
end;
/////btSwapFloat uses using char pointers to swap the endianness
//////btSwapFloat/btSwapDouble will NOT return a float, because the machine might 'correct' invalid floating point values
/////Not all values of sign/exponent/mantissa are valid floating point numbers according to IEEE 754.
/////When a floating point unit is faced with an invalid value, it may actually change the value, or worse, throw an exception.
/////In most systems, running user mode code, you wouldn't get an exception, but instead the hardware/os/runtime will 'fix' the number for you.
/////so instead of returning a float/double, we return integer/long long integer

function btSwapEndianFloat(const d:single):cardinal;FOS_INLINE;
var src,dst:pbyte;
begin
  result:=0;
  dst:=@result;
  src:=@d;
  dst[0]:=src[3];
  dst[1]:=src[2];
  dst[2]:=src[1];
  dst[3]:=src[0];
end;

function btUnSwapEndianFloat(const a:cardinal):single;FOS_INLINE;
var src,dst:pbyte;
begin
  result:=0;
  dst:=@result;
  src:=@a;
  dst[0]:=src[3];
  dst[1]:=src[2];
  dst[2]:=src[1];
  dst[3]:=src[0];
end;

procedure btSwapEndianDouble(const d:double;const dst:PByte);FOS_INLINE;
var src:PByte;
begin
  src:=@d;
  dst[0] := src[7];
  dst[1] := src[6];
  dst[2] := src[5];
  dst[3] := src[4];
  dst[4] := src[3];
  dst[5] := src[2];
  dst[6] := src[1];
  dst[7] := src[0];
end;

function btUnswapEndianDouble(const src:PByte):double;
var dst:PByte;
begin
  result:=0;
  dst:=@result;
  dst[0] := src[7];
  dst[1] := src[6];
  dst[2] := src[5];
  dst[3] := src[4];
  dst[4] := src[3];
  dst[5] := src[2];
  dst[6] := src[1];
  dst[7] := src[0];
end;

function btNormalizeAngle(angleInRadians:btScalar):btScalar;
begin
  angleInRadians:=btFMod(angleInRadians,SIMD_2_PI);
  if angleInRadians < -SIMD_PI then
  begin
    exit(angleInRadians+SIMD_2_PI);
  end else
  if angleInRadians > SIMD_PI then
  begin
    exit(angleInRadians - SIMD_2_PI);
  end else
  begin
    exit(angleInRadians);
  end;
end;

// btMinMax.h (class and function templates -> strategy overload types)
function btMin(const a,b:btScalar):btScalar;overload;FOS_INLINE;
begin
  if a<b then exit(a) else exit(b);
end;

function btMin(const a, b: integer): integer;
begin
  if a<b then exit(a) else exit(b);
end;

function btMax(const a,b:btScalar):btScalar;overload;FOS_INLINE;
begin
  if a>b then exit(a) else exit(b);
end;

function btMax(const a, b: integer): integer;
begin
  if a>b then exit(a) else exit(b);
end;

function btClamped(const a,lb,ub:btScalar):btScalar;overload;FOS_INLINE;
begin
 if a<lb then exit(lb);
 if ub<a then exit(ub) else exit(a);
end;

procedure btSetMin(var a:btScalar; const b:btScalar);overload;FOS_INLINE;
begin
 if b<a then a:=b;
end;
procedure btSetMax(var a:btScalar; const b:btScalar);overload;FOS_INLINE;
begin
 if a<b then a:=b;
end;
procedure btClamp(var a:btScalar; const lb,ub:btScalar);overload;FOS_INLINE;
begin
 if a<lb then
 begin
   a:=lb;
 end else
 if ub<a then
 begin
   a:=ub;
 end;
end;

procedure GEN_srand(const seed: cardinal);
begin
  RandSeed:=seed;
  abort;
end;

function GEN_rand: cardinal;
begin
  result:=Random(high(Cardinal));
  abort;
end;

procedure AabbExpand(var aabbMin, aabbMax: btVector3; const expansionMin,expansionMax: btVector3);
begin
  aabbMin := aabbMin + expansionMin;
  aabbMax := aabbMax + expansionMax;
end;

// conservative test for overlap between two aabbs
function TestPointAgainstAabb2(const aabbMin1, aabbMax1, point: btVector3): boolean;
begin
 if (aabbMin1.getX > point.getX) or (aabbMax1.getX < point.getX) then exit(false);
 if (aabbMin1.getZ > point.getZ) or (aabbMax1.getZ < point.getZ) then exit(false);
 if (aabbMin1.getY > point.getY) or (aabbMax1.getY < point.getY) then exit(false);
 result:=true;
 //bool overlap = true;
 //overlap = (aabbMin1.getX() > point.getX() || aabbMax1.getX() < point.getX()) ? false : overlap;
 //overlap = (aabbMin1.getZ() > point.getZ() || aabbMax1.getZ() < point.getZ()) ? false : overlap;
 //overlap = (aabbMin1.getY() > point.getY() || aabbMax1.getY() < point.getY()) ? false : overlap;
 //return overlap;
end;

/// conservative test for overlap between two aabbs
function TestAabbAgainstAabb2(const aabbMin1, aabbMax1, aabbMin2, aabbMax2: btVector3): boolean;
begin
 if (aabbMin1.getX > aabbMax2.getX) or (aabbMax1.getX < aabbMin2.getX) then exit(false);
 if (aabbMin1.getZ > aabbMax2.getZ) or (aabbMax1.getZ < aabbMin2.getZ) then exit(false);
 if (aabbMin1.getY > aabbMax2.getY) or (aabbMax1.getY < aabbMin2.getY) then exit(false);
 result:=true;
 //bool overlap = true;
 //overlap = (aabbMin1.getX() > aabbMax2.getX() || aabbMax1.getX() < aabbMin2.getX()) ? false : overlap;
 //overlap = (aabbMin1.getZ() > aabbMax2.getZ() || aabbMax1.getZ() < aabbMin2.getZ()) ? false : overlap;
 //overlap = (aabbMin1.getY() > aabbMax2.getY() || aabbMax1.getY() < aabbMin2.getY()) ? false : overlap;
 //return overlap;
end;

function TestTriangleAgainstAabb2(const vertices: PbtVector3; const aabbMin, aabbMax: btVector3): boolean;
var p1,p2,p3:PbtVector3;
begin
 p1 := vertices;   //TODO FOS : check pointer arithmetic
 p2 := vertices+1;
 p3 := vertices+2;

 if (btMin(btMin(p1^[0], p2^[0]), p3^[0]) > aabbMax[0]) then exit(false);
 if (btMax(btMax(p1^[0], p2^[0]), p3^[0]) < aabbMin[0]) then exit(false);

 if (btMin(btMin(p1^[2], p2^[2]), p3^[2]) > aabbMax[2]) then exit(false);
 if (btMax(btMax(p1^[2], p2^[2]), p3^[2]) < aabbMin[2]) then exit(false);

 if (btMin(btMin(p1^[1], p2^[1]), p3^[1]) > aabbMax[1]) then exit(false);
 if (btMax(btMax(p1^[1], p2^[1]), p3^[1]) < aabbMin[1]) then exit(false);
 Result := true;
end;

function btOutcode(const p, halfExtent: btVector3): integer;
begin
 if p.getX < -halfExtent.getX then result:=1 else result:=0;
 if p.getX >  halfExtent.getX then result:=result or $08;
 if p.getY < -halfExtent.getY then result:=result or $02;
 if p.getY >  halfExtent.getY then result:=result or $10;
 if p.getZ < -halfExtent.getZ then result:=result or $04;
 if p.getZ >  halfExtent.getZ then result:=result or $20;
 //return (p.getX()  < -halfExtent.getX() ? 0x01 : 0x0) |
 //	   (p.getX() >  halfExtent.getX() ? 0x08 : 0x0) |
 //	   (p.getY() < -halfExtent.getY() ? 0x02 : 0x0) |
 //	   (p.getY() >  halfExtent.getY() ? 0x10 : 0x0) |
 //	   (p.getZ() < -halfExtent.getZ() ? 0x4 : 0x0) |
 //	   (p.getZ() >  halfExtent.getZ() ? 0x20 : 0x0);
end;

function btRayAabb2(const rayFrom, rayInvDirection: btVector3; const raySign: TbtFOSSignArray; const bounds: TbtFOSboundsArray; var tmin:btScalar;const lambda_min, lambda_max: btScalar): boolean;
var tmax,tymin,tymax,tzmin,tzmax:btScalar;
begin
 tmin  := (bounds[  raySign[0]].getX - rayFrom.getX) * rayInvDirection.getX;
 tmax  := (bounds[1-raySign[0]].getX - rayFrom.getX) * rayInvDirection.getX;
 tymin := (bounds[  raySign[1]].getY - rayFrom.getY) * rayInvDirection.getY;
 tymax := (bounds[1-raySign[1]].getY - rayFrom.getY) * rayInvDirection.getY;

 if ( (tmin > tymax) or (tymin > tmax) ) then exit(false);
 if (tymin > tmin) then tmin := tymin;
 if (tymax < tmax) then	tmax := tymax;

 tzmin := (bounds[  raySign[2]].getZ - rayFrom.getZ) * rayInvDirection.getZ;
 tzmax := (bounds[1-raySign[2]].getZ - rayFrom.getZ) * rayInvDirection.getZ;

 if ( (tmin > tzmax) or (tzmin > tmax) ) then exit(false);
 if ( tzmin > tmin) then tmin := tzmin;
 if (tzmax < tmax)  then tmax := tzmax;

 Result := ((tmin < lambda_max) and (tmax > lambda_min));
end;

function btRayAabb(const rayFrom, rayTo, aabbMin, aabbMax: btVector3; var param:btScalar;out normal: btVector3): boolean;
var  hitNormal,r,aabbHalfExtent,aabbCenter,source,target:btVector3;
     j,bit,i,sourceOutcode,targetOutcode:integer;
     lambda,normsign,lambda_enter,lambda_exit:btScalar;
begin
  aabbHalfExtent := (aabbMax-aabbMin)* btScalar(0.5);
  aabbCenter     := (aabbMax+aabbMin)* btScalar(0.5);
  source         := rayFrom - aabbCenter;
  target         := rayTo - aabbCenter;
  sourceOutcode  := btOutcode(source,aabbHalfExtent);
  targetOutcode  := btOutcode(target,aabbHalfExtent);

  if ((sourceOutcode and targetOutcode) = 0) then begin
    lambda_enter := btScalar(0.0);
    lambda_exit  := param;
    r            := target - source;
    normSign     := 1;
    bit          := 1;
    hitNormal.zero;
    for j:=0 to 1 do begin
      i:=0;
      while(i <> 3) do begin
        if (sourceOutcode and bit)>0 then begin
          lambda := (-source[i] - aabbHalfExtent[i]*normSign) / r[i];
          if (lambda_enter <= lambda) then begin
            lambda_enter := lambda;
            hitNormal.init(0,0,0);
            hitNormal[i] := normSign;
          end;
        end else
        if (targetOutcode and bit)>0 then begin
          lambda := (-source[i] - aabbHalfExtent[i]*normSign) / r[i];
          btSetMin(lambda_exit, lambda);
        end;
        //bit<<=1;
        bit := bit shl 1;
        inc(i);
      end;
      normSign := btScalar(-1);
    end;
    if (lambda_enter <= lambda_exit) then begin
      param := lambda_enter;
      normal := hitNormal;
      exit(true);
    end;
 end;
 result:=false;
end;


procedure btTransformAabb(const halfExtents: btVector3; const margin: btScalar; const t: btTransform; out aabbMinOut, aabbMaxOut: btVector3);
var tv,extent,center,halfExtentsWithMargin:btVector3;
    abs_b:btMatrix3x3;
begin
 tv.Init(margin,margin,margin);
 halfExtentsWithMargin := halfExtents+tv;
 abs_b                 := t.getBasisV^.absolute;
 center                := t.getOriginV^;
 extent.Init(abs_b[0].dot(halfExtentsWithMargin),abs_b[1].dot(halfExtentsWithMargin),abs_b[2].dot(halfExtentsWithMargin));
 aabbMinOut            := center - extent;
 aabbMaxOut            := center + extent;
end;

procedure btTransformAabb(const localAabbMin, localAabbMax: btVector3; const margin: btScalar; const trans: btTransform; out aabbMinOut, aabbMaxOut: btVector3);
var extent,center,localCenter,localHalfExtents:btVector3;
    abs_b:btMatrix3x3;
begin
 localHalfExtents := (localAabbMax-localAabbMin)*btScalar(0.5);
 localHalfExtents := localHalfExtents +  btVector3.InitSameS(margin);
 localCenter      := (localAabbMax+localAabbMin)*btScalar(0.5);
 abs_b            := trans.getBasisV^.absolute;
 center           := trans.opTrans(localCenter);

 extent.init(abs_b[0].dot(localHalfExtents),abs_b[1].dot(localHalfExtents),abs_b[2].dot(localHalfExtents));
 aabbMinOut := center-extent;
 aabbMaxOut := center+extent;
end;


/////btSelect avoids branches, which makes performance much better for consoles like Playstation 3 and XBox 360
/////Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html
//SIMD_FORCE_INLINE unsigned btSelect(unsigned condition, unsigned valueIfConditionNonZero, unsigned valueIfConditionZero)
//{
//    // Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
//    // Rely on positive value or'ed with its negative having sign bit on
//    // and zero value or'ed with its negative (which is still zero) having sign bit off
//    // Use arithmetic shift right, shifting the sign bit through all 32 bits
//    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
//    unsigned testEqz = ~testNz;
//    return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
//}
//SIMD_FORCE_INLINE int btSelect(unsigned condition, int valueIfConditionNonZero, int valueIfConditionZero)
//{
//    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
//    unsigned testEqz = ~testNz;
//    return static_cast<int>((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
//}
//SIMD_FORCE_INLINE float btSelect(unsigned condition, float valueIfConditionNonZero, float valueIfConditionZero)
//{
//#ifdef BT_HAVE_NATIVE_FSEL
//    return (float)btFsel((btScalar)condition - btScalar(1.0f), valueIfConditionNonZero, valueIfConditionZero);
//#else
//    return (condition != 0) ? valueIfConditionNonZero : valueIfConditionZero;
//#endif
//}
function btSelect(const condition,valueIfConditionNonZero,valueIfConditionZero:cardinal):cardinal;FOS_INLINE;
var testNz,testEqz:cardinal;
begin
    // Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
    // Rely on positive value or'ed with its negative having sign bit on
    // and zero value or'ed with its negative (which is still zero) having sign bit off
    // Use arithmetic shift right, shifting the sign bit through all 32 bits
    testNz  := cardinal(  SarLongint(integer(condition) or (-integer(condition)) , 31));
abort;
    testEqz := not testNz;
    result  := ((valueIfConditionNonZero and testNz) or (valueIfConditionZero and testEqz));
end;
function btSelect(const condition:cardinal;const valueIfConditionNonZero,valueIfConditionZero:integer):integer;FOS_INLINE;
var testNz,testEqz:cardinal;
begin
    testNz  := cardinal(  SarLongint((integer(condition) or (-integer(condition))),31));
    testEqz := not testNz;
    result  := integer((valueIfConditionNonZero and testNz) or (valueIfConditionZero and testEqz));
end;
function btSelect(const condition:cardinal;const valueIfConditionNonZero,valueIfConditionZero:float):float;FOS_INLINE;
begin
  if condition <> 0 then result:=valueIfConditionNonZero else result:=valueIfConditionZero;
end;



{.$define USE_BANCHLESS}
{$ifdef USE_BANCHLESS}
//TODO: This block replaces the block below and uses no branches, and replaces the 8 bit return with a 32 bit return for improved performance (~3x on XBox 360) FOS : don't know how FPC performs test the two variations ...
function testQuantizedAabbAgainstQuantizedAabb(const aabbMin1, aabbMax1, aabbMin2, aabbMax2: PWord): LongBool;
var b:boolean;
begin
  result:=btSelect(cardinal((aabbMin1[0] <= aabbMax2[0]) and (aabbMax1[0] >= aabbMin2[0]) and (aabbMin1[2] <= aabbMax2[2]) and (aabbMax1[2] >= aabbMin2[2]) and (aabbMin1[1] <= aabbMax2[1]) and (aabbMax1[1] >= aabbMin2[1])), 1, 0)=1;
end;
//SIMD_FORCE_INLINE unsigned testQuantizedAabbAgainstQuantizedAabb(const unsigned short int* aabbMin1,const unsigned short int* aabbMax1,const unsigned short int* aabbMin2,const unsigned short int* aabbMax2)
//{
//	return static_cast<unsigned int>(btSelect((unsigned)((aabbMin1[0] <= aabbMax2[0]) & (aabbMax1[0] >= aabbMin2[0])
//		& (aabbMin1[2] <= aabbMax2[2]) & (aabbMax1[2] >= aabbMin2[2])
//		& (aabbMin1[1] <= aabbMax2[1]) & (aabbMax1[1] >= aabbMin2[1])),
//		1, 0));
//}
{$else}
function testQuantizedAabbAgainstQuantizedAabb(const aabbMin1, aabbMax1, aabbMin2, aabbMax2: PWord): LongBool;
begin
  if (aabbMin1[0] > aabbMax2[0]) or (aabbMax1[0] < aabbMin2[0]) then exit(false);
  if (aabbMin1[2] > aabbMax2[2]) or (aabbMax1[2] < aabbMin2[2]) then exit(false);
  if (aabbMin1[1] > aabbMax2[1]) or (aabbMax1[1] < aabbMin2[1]) then exit(false);
  result:=true;
end;

function btPlaneFlip(const plane: btPlane): btPlane;FOS_INLINE;
begin
  result.Init(-plane.normal,-plane.dist);
end;

function btCoPlanar(const a, b: btPlane): boolean;FOS_INLINE;
begin
  result:=(a=b) or (a=btPlaneFlip(b));
end;

// returns the point where the line p0-p1 intersects the plane n&d
function btPlaneLineIntersection(const plane: btPlane; const p0, p1: btVector3): btVector3;
var dif:btVector3;
    t,dn:btScalar;
begin
 dif := p1-p0;
 dn  := btDot(plane.normal,dif);
 t   := -(plane.dist+btDot(plane.normal,p0) )/dn;
 result := p0 + (dif*t);
end;

function btPlaneProject(const plane: btPlane; const point: btVector3): btVector3;
begin
 result := point - plane.normal * (btDot(point,plane.normal)+plane.dist);
end;

function btThreePlaneIntersection(const p0, p1, p2: btPlane): btVector3;
var N1,N2,N3:btVector3;
    n2n3,n3n1,n1n2:btVector3;
    quotient:btScalar;
    potentialVertex:btVector3;
begin
 N1 := p0.normal;
 N2 := p1.normal;
 N3 := p2.normal;

 n2n3 := N2.cross(N3);
 n3n1 := N3.cross(N1);
 n1n2 := N1.cross(N2);

 quotient := (N1.dot(n2n3));

 // btAssert(btFabs(quotient) > btScalar(0.000001));

 quotient := -1/quotient;
 n2n3 := n2n3 * p0.dist;
 n3n1 := n3n1 * p1.dist;
 n1n2 := n1n2 * p2.dist;
 potentialVertex := n2n3;
 potentialVertex += n3n1;
 potentialVertex += n1n2;
 potentialVertex *= quotient;
 result := potentialVertex;
end;

function btDistanceBetweenLines(const ustart, udir, vstart, vdir: btVector3;const upoint: PbtVector3; const vpoint: PbtVector3): btScalar;
var cp:btVector3;
    distu,distv,dist:btScalar;
    plane:btPlane;
begin
 cp := btCross(udir,vdir).normalized;

 distu := -btDot(cp,ustart);
 distv := -btDot(cp,vstart);
 dist  := btFabs(distu-distv);
 if assigned(upoint) then begin
   plane.normal := btCross(vdir,cp).normalized();
   plane.dist   := -btDot(plane.normal,vstart);
   upoint^      := btPlaneLineIntersection(plane,ustart,ustart+udir);
 end;
 if assigned(vpoint) then begin
   plane.normal := btCross(udir,cp).normalized();
   plane.dist   := -btDot(plane.normal,ustart);
   vpoint^      := btPlaneLineIntersection(plane,vstart,vstart+vdir);
 end;
 result:=dist;
end;

 // return the normal of the triangle
// inscribed by v0, v1, and v2
function btTriNormal(const v0, v1, v2: btVector3): btVector3;
var cp:btVector3;
     m:btScalar;
begin
   cp := btCross(v1-v0,v2-v1);
   m  := cp.length;
   if (m=0) then begin
     result.Init(1,0,0);
   end else begin
     result:= cp*(1/m);
   end;
end;

function btPlaneTest(const p: btPlane; const v: btVector3): btPlaneTestFlag;
var a:btScalar;
begin
  a  := btDot(v,p.normal)+p.dist;
  if a>btvPlanetestepsilon then begin
    result:=btptOVER;
  end else begin
    if a>btvPlanetestepsilon then begin
      result:=btptUNDER;
    end else begin
      result:=btptCOPLANAR;
    end;
  end;
end;

function btSplitTest(const convex: btConvexH; const plane: btPlane): btPlaneTestFlagSet;
var
  i: Integer;
begin
  result := [];
  for i:=0 to convex.vertices.Length-1 do begin
    include(result,btPlaneTest(plane,convex.vertices.A[i]^))
  end;
end;

function btOrth(const v: btVector3): btVector3;
var a,b,temp:btVector3;
begin
  a:=btCross(v,temp.Init(0,0,1));
  b:=btCross(v,temp.Init(0,1,0));
  if a.length > b.length then begin
    result := a.normalized;
  end else begin
    result := b.normalized;
  end;
end;

function btAbove(const vertices: PbtVector3; const t: btInt3; const p: btVector3; const epsilon: btScalar): boolean;
var n:btVector3;
begin
  n := btTriNormal(vertices[t[0]],vertices[t[1]],vertices[t[2]]);
  result := (btDot(n,p-vertices[t[0]]) > epsilon); // EPSILON???
end;

function btHasEdge(const t: btInt3; const a, b: integer): boolean;
var i,i1:integer;
begin
  for i:=0 to 2 do begin
    i1:=(i+1) mod 3;
    if (t[i]=a) and (t[i1]=b) then exit(true);
  end;
  result:=false;
end;

function btHasVert(const t: btInt3; const v: integer): boolean;
begin
  result:= (t[0]=v) or (t[1]=v) or (t[2]=v);
end;

function btShareEdge(const a, b: btInt3): boolean;
var  i,i1:integer;
begin
  for i:=0 to 2 do begin
    i1 := (i+1) mod 3;
   if (btHasEdge(a,b[i1],b[i])) then exit(true);
  end;
  result:=false;
end;




operator=(const a, b: btplane): boolean;FOS_INLINE;
begin
  result := (a.normal=b.normal) and (a.dist=b.dist);
end;

//operator=(const a, b: btInt3): boolean;FOS_INLINE;
//begin
//  if a.x<>b.x then exit(false);
//  if a.y<>b.y then exit(false);
//  if a.z<>b.z then exit(false);
//  result:=true;
//end;

{$endif} //USE_BANCHLESS




{ btVector3 }
operator + (const v1,v2:btVector3) v3:btVector3;
begin
  v3.v.m_floats[0]:=v1.v.m_floats[0]+v2.v.m_floats[0];
  v3.v.m_floats[1]:=v1.v.m_floats[1]+v2.v.m_floats[1];
  v3.v.m_floats[2]:=v1.v.m_floats[2]+v2.v.m_floats[2];
end;
operator - (const v1,v2:btVector3) v3:btVector3;
begin
  v3.v.m_floats[0]:=v1.v.m_floats[0]-v2.v.m_floats[0];
  v3.v.m_floats[1]:=v1.v.m_floats[1]-v2.v.m_floats[1];
  v3.v.m_floats[2]:=v1.v.m_floats[2]-v2.v.m_floats[2];
end;
operator *(const v1:btVector3;const s:btScalar) v3:btVector3;
begin
  v3.v.m_floats[0]:=v1.v.m_floats[0]*s;
  v3.v.m_floats[1]:=v1.v.m_floats[1]*s;
  v3.v.m_floats[2]:=v1.v.m_floats[2]*s;
end;

operator*(const s: btScalar; const v1: btVector3) v3:btVector3;
begin
  v3 := v1*s;
end;

operator *(const v1:btVector3;const v2:btVector3) v3:btVector3; // Element wise product
begin
  v3.v.m_floats[0]:=v1.v.m_floats[0]*v2.v.m_floats[0];
  v3.v.m_floats[1]:=v1.v.m_floats[1]*v2.v.m_floats[1];
  v3.v.m_floats[2]:=v1.v.m_floats[2]*v2.v.m_floats[2];
end;
operator / (const v1:btVector3;s:btScalar) v3:btVector3;
begin
  v3:=v1*(1/s);
end;

operator / (const v1, v2: btVector3) vres:btVector3;
begin
   vres.init(v1.v.m_floats[0] / v2.v.m_floats[0],v1.v.m_floats[1] / v2.v.m_floats[1],v1.v.m_floats[2] / v2.v.m_floats[2]);
end;

operator = (const v1,v2:btVector3) res:boolean;
begin
 if (v1.getX=v2.getX) and (v1.getY=v2.getY) and (v1.getZ=v2.getZ) then begin
   res:=true;
 end else begin
   res:=false;
 end;
end;

operator -(const v1:btVector3) :btVector3;
begin
 result.Init(-v1.v.m_floats[0],-v1.v.m_floats[1],-v1.v.m_floats[2]);
end;

function init_btVector3(const x, y, z: btScalar): btVector3;
begin
  result.Init(x,y,z);
end;

{ btVector4 }
operator + (const v1,v2:btVector4) v3:btVector4;
begin
  v3.v.m_floats[0]:=v1.v.m_floats[0]+v2.v.m_floats[0];
  v3.v.m_floats[1]:=v1.v.m_floats[1]+v2.v.m_floats[1];
  v3.v.m_floats[2]:=v1.v.m_floats[2]+v2.v.m_floats[2];
  v3.v.m_floats[3]:=v1.v.m_floats[3]+v2.v.m_floats[3];
end;
operator - (const v1,v2:btVector4) v3:btVector4;
begin
  v3.v.m_floats[0]:=v1.v.m_floats[0]-v2.v.m_floats[0];
  v3.v.m_floats[1]:=v1.v.m_floats[1]-v2.v.m_floats[1];
  v3.v.m_floats[2]:=v1.v.m_floats[2]-v2.v.m_floats[2];
  v3.v.m_floats[3]:=v1.v.m_floats[3]-v2.v.m_floats[3];
end;

{ btQuaternion }
operator + (const v1,v2:btQuaternion) v3:btQuaternion;
begin
  v3.v.m_floats[0]:=v1.v.m_floats[0]+v2.v.m_floats[0];
  v3.v.m_floats[1]:=v1.v.m_floats[1]+v2.v.m_floats[1];
  v3.v.m_floats[2]:=v1.v.m_floats[2]+v2.v.m_floats[2];
  v3.v.m_floats[3]:=v1.v.m_floats[3]+v2.v.m_floats[3];
end;
operator - (const v1,v2:btQuaternion) v3:btQuaternion;
begin
  v3.v.m_floats[0]:=v1.v.m_floats[0]-v2.v.m_floats[0];
  v3.v.m_floats[1]:=v1.v.m_floats[1]-v2.v.m_floats[1];
  v3.v.m_floats[2]:=v1.v.m_floats[2]-v2.v.m_floats[2];
  v3.v.m_floats[3]:=v1.v.m_floats[3]-v2.v.m_floats[3];
end;

operator-(const v1: btQuaternion)v3: btQuaternion;
begin
  result.Init4(-v1.getX,-v1.getY,-v1.getZ,-v1.getW);
end;

operator * (const v1, v2: btQuaternion) v3: btQuaternion;
begin
 result.Init4(v1.getW * v2.getX + v1.v.m_floats[0] * v2.getW + v1.v.m_floats[1] * v2.getz - v1.v.m_floats[2] * v2.gety,
              v1.getW * v2.getY + v1.v.m_floats[1] * v2.getW + v1.v.m_floats[2] * v2.getx - v1.v.m_floats[0] * v2.getz,
              v1.getW * v2.getZ + v1.v.m_floats[2] * v2.getW + v1.v.m_floats[0] * v2.gety - v1.v.m_floats[1] * v2.getx,
              v1.getW * v2.getW - v1.v.m_floats[0] * v2.getX - v1.v.m_floats[1] * v2.gety - v1.v.m_floats[2] * v2.getz);
end;

operator*(const v1: btQuaternion; const s: btScalar)v3: btQuaternion;
begin
 result.v.m_floats[0]:=v1.v.m_floats[0]*s;
 result.v.m_floats[1]:=v1.v.m_floats[1]*s;
 result.v.m_floats[2]:=v1.v.m_floats[2]*s;
 result.v.m_floats[3]:=v1.v.m_floats[3]*s;
end;

operator*(const q: btQuaternion; const w: btVector3)v3: btQuaternion;
begin
 result.Init4( q.getW * w.getX + q.getY * w.getz - q.getz * w.gety,
 	       q.getw * w.gety + q.getz * w.getx - q.getx * w.getz,
 	       q.getw * w.getz + q.getx * w.gety - q.gety * w.getx,
              -q.getx * w.getx - q.gety * w.gety - q.getz * w.getz);
end;

operator*(const w: btVector3; const q: btQuaternion)v3: btQuaternion;
begin
  result.Init4 ( w.getx * q.getw + w.gety * q.getz - w.getz * q.gety,
	 	 w.gety * q.getw + w.getz * q.getx - w.getx * q.getz,
		 w.getz * q.getw + w.getx * q.gety - w.gety * q.getx,
		-w.getx * q.getx - w.gety * q.gety - w.getz * q.getz);
end;

operator/(const v1: btQuaternion; const s: btScalar)v3: btQuaternion;
begin
  result:=v1*(1/s);
end;

function btDot(const q1, q2: btQuaternion): btScalar;
begin
  result:=q1.dot(q2);
end;

function btLength(const q: btQuaternion): btScalar;
begin
  result:=q.length;
end;

function btAngle(const q1, q2: btQuaternion): btScalar;
begin
  result:=q1.angle(q2);
end;

function btInverse(const q: btQuaternion): btQuaternion;
begin
  result:=q.inverse;
end;

function btSlerp(const q1, q2: btQuaternion; const t: btScalar): btQuaternion;
begin
  result:=q1.slerp(q2,t);
end;

function btQuatRotate(const rotation: btQuaternion; const v: btVector3): btVector3;
var q:btQuaternion;
begin
 q := rotation * v;
 q.mulEquals(rotation.inverse);
 result.Init(q.getX,q.getY,q.getZ);
end;

function btShortestArcQuat(const v0, v1: btVector3): btQuaternion;
var c,n,unused:btVector3;
    d,s,rs:btScalar;
begin
 c := v0.cross(v1);
 d := v0.dot(v1);
 if (d < -1.0 + SIMD_EPSILON) then begin
   btPlaneSpace1(v0,n,unused);
   result.Init4(n.getX,n.gety,n.getz,0.0); // just pick any vector that is orthogonal to v0
   exit;
 end;
 s  := btSqrt((1.0 + d) * 2.0);
 rs := 1.0 / s;
 result.init4(c.getX*rs,c.getY*rs,c.getZ*rs,s*0.5);
end;

function btShortestArcQuatNormalize(const v0, v1: btVector3): btQuaternion;
begin
 v0.normalize;
 v1.normalize;
 result := btShortestArcQuat(v0,v1);
end;

function calcWindowXY(const objx, objy, objz: Single; const modelview, projection: TFOS_GL_Matrix; const viewport: PIntegerArray; var wx, wy, wz: Single): Boolean;
var
 //Transformation vectors
 fTempo: array [0..7] of Single;
begin
   //Modelview transform
   fTempo[0]:=modelview[0]*objx+modelview[4]*objy+modelview[8]*objz+modelview[12];  //w is always 1
   fTempo[1]:=modelview[1]*objx+modelview[5]*objy+modelview[9]*objz+modelview[13];
   fTempo[2]:=modelview[2]*objx+modelview[6]*objy+modelview[10]*objz+modelview[14];
   fTempo[3]:=modelview[3]*objx+modelview[7]*objy+modelview[11]*objz+modelview[15];
   //Projection transform, the final row of projection matrix is always [0 0 -1 0]
   //so we optimize for that.
   fTempo[4]:=projection[0]*fTempo[0]+projection[4]*fTempo[1]+projection[8]*fTempo[2]+projection[12]*fTempo[3];
   fTempo[5]:=projection[1]*fTempo[0]+projection[5]*fTempo[1]+projection[9]*fTempo[2]+projection[13]*fTempo[3];
   fTempo[6]:=projection[2]*fTempo[0]+projection[6]*fTempo[1]+projection[10]*fTempo[2]+projection[14]*fTempo[3];
   fTempo[7]:=-fTempo[2];
   //The result normalizes between -1 and 1
   if fTempo[7]=0.0 then begin	//The w value
      exit(false);
   end;
   fTempo[7]:=1/fTempo[7];
   //Perspective division
   fTempo[4]:=fTempo[4]*fTempo[7];
   fTempo[5]:=fTempo[5]*fTempo[7];
   fTempo[6]:=fTempo[6]*fTempo[7];
   //Window coordinates
   //Map x, y to range 0-1
   wx:=(fTempo[4]*0.5+0.5)*viewport^[2]+viewport^[0];
   wy:=(fTempo[5]*0.5+0.5)*viewport^[3]+viewport^[1];
   //This is only correct when glDepthRange(0.0, 1.0)
   wz:=(1.0+fTempo[6])*0.5;	//Between 0 and 1
   result:=true;
end;

function calcQuatFromFwdVec(fwd: btVector3): btQuaternion;
var
  up,right: btVector3;
  glm: TFOS_GL_Matrix;
  trans: btTransform;
begin
  fwd.normalize;
  up.Init(0,1,0);

  if 1-btFabs(fwd[1])<SIMD_EPSILON then begin
    up.Init(0,0,1);
  end;
  right:=up.cross(fwd);
  up:=fwd.cross(right);

  right.normalize;
  up.normalize;

  glm[0]:=-right[0]; glm[1]:=-right[1]; glm[2]:=-right[2]; glm[3]:=0;
  glm[4]:=up[0]; glm[5]:=up[1]; glm[6]:=up[2]; glm[7]:=0;
  glm[8]:=-fwd[0]; glm[9]:=-fwd[1]; glm[10]:=-fwd[2]; glm[11]:=0;
  glm[12]:=0; glm[13]:=0; glm[14]:=0; glm[15]:=1;
  trans.setFromOpenGLMatrix(glm);
  Result:=trans.getRotation.normalized;
end;

function calcProjectionMatrix(const fovyInDegrees, aspectRatio, znear, zfar: Single; var aabbMin,aabbMax: btVector3): TFOS_GL_Matrix;
var
  ymax,xmax: Single;

  function generateFrustum(const left,right,bottom,top,znear,zfar:Single):TFOS_GL_Matrix;
  var
    temp,temp2,temp3,temp4:Single;
  begin
      temp:=2*znear;
      temp2:=right-left;
      temp3:=top-bottom;
      temp4:=zfar-znear;
      Result[0]:=temp/temp2;
      Result[1]:=0; Result[2]:=0; Result[3]:=0; Result[4]:=0;
      Result[5]:=temp/temp3;
      Result[6]:=0; Result[7]:=0;
      Result[8]:=(right+left) / temp2;
      Result[9]:=(top+bottom) / temp3;
      Result[10]:=(-zfar-znear) / temp4;
      Result[11]:=-1; Result[12]:=0; Result[13]:=0;
      Result[14]:=(-temp*zfar) / temp4;
      Result[15]:=0;
  end;

begin
    //calc aabb of frustum
    ymax:=zfar* btTan(btRadians(fovyInDegrees));
    xmax:=ymax * aspectRatio;
    aabbMin.Init(-xmax,-ymax,znear);
    aabbMax.Init(xmax,ymax,zfar);
    //calc projection matrix
    ymax:=znear* btTan(btRadians(fovyInDegrees));
    xmax:=ymax * aspectRatio;
    Result:=generateFrustum(-xmax,xmax,-ymax,ymax,znear,zfar);
end;

operator*(const m: btMatrix3x3; const v: btVector3):btVector3;
begin
  result.Init (m[0].dot(v), m[1].dot(v), m[2].dot(v));
end;

operator*(const v: btVector3; const m: btMatrix3x3):btVector3;
begin
  result.Init(m.tdotx(v), m.tdoty(v), m.tdotz(v));
end;

operator*(const m1: btMatrix3x3; const m2: btMatrix3x3):btMatrix3x3;
begin
 result.init(
 	m2.tdotx( m1[0]), m2.tdoty( m1[0]), m2.tdotz( m1[0]),
 	m2.tdotx( m1[1]), m2.tdoty( m1[1]), m2.tdotz( m1[1]),
 	m2.tdotx( m1[2]), m2.tdoty( m1[2]), m2.tdotz( m1[2]));
end;

operator=(const m1, m2: btMatrix3x3): boolean;
begin
 result:= (m1[0][0] = m2[0][0]) and (m1[1][0] = m2[1][0]) and (m1[2][0] = m2[2][0]) and
   	  (m1[0][1] = m2[0][1]) and (m1[1][1] = m2[1][1]) and (m1[2][1] = m2[2][1]) and
 	  (m1[0][2] = m2[0][2]) and (m1[1][2] = m2[1][2]) and (m1[2][2] = m2[2][2]);
end;

operator*(const m: btMatrix4x4; const v: btVector4): btVector4;
begin
  result.Init4(m[0].dot(v), m[1].dot(v), m[2].dot(v),m[3].dot(v));
end;

operator*(const v: btVector4; const m: btMatrix4x4): btVector4;
begin
  result.Init4(m.tdotx(v), m.tdoty(v), m.tdotz(v), m.tdotw(v));
end;

operator*(const m1: btMatrix4x4; const m2: btMatrix4x4): btMatrix4x4;
begin
  result.init(
  	m2.tdotx( m1[0]), m2.tdoty( m1[0]), m2.tdotz( m1[0]), m2.tdotw( m1[0]),
  	m2.tdotx( m1[1]), m2.tdoty( m1[1]), m2.tdotz( m1[1]), m2.tdotw( m1[1]),
  	m2.tdotx( m1[2]), m2.tdoty( m1[2]), m2.tdotz( m1[2]), m2.tdotw( m1[2]),
  	m2.tdotx( m1[3]), m2.tdoty( m1[3]), m2.tdotz( m1[3]), m2.tdotw( m1[3])
        );
end;

operator=(const m1, m2: btMatrix4x4):boolean ;
begin
  result:= (m1[0][0] = m2[0][0]) and (m1[1][0] = m2[1][0]) and (m1[2][0] = m2[2][0]) and (m1[3][0] = m2[3][0]) and
      	   (m1[0][1] = m2[0][1]) and (m1[1][1] = m2[1][1]) and (m1[2][1] = m2[2][1]) and (m1[3][1] = m2[3][1]) and
  	   (m1[0][2] = m2[0][2]) and (m1[1][2] = m2[1][2]) and (m1[2][2] = m2[2][2]) and (m1[3][2] = m2[3][2]) and
           (m1[0][3] = m2[0][3]) and (m1[1][3] = m2[1][3]) and (m1[2][3] = m2[2][3]) and (m1[3][3] = m2[3][3]);
end;

operator=(const t1, t2: btTransform): boolean;
begin
  result := (t1.GetBasisV^ = t2.GetBasisV^) and (t1.getOriginV^ = t2.getOriginV^);
end;

operator*(const t1, t2: btTransform): btTransform;
begin
  result:=t1.opMul(t2);
end;

operator*(const t1: btTransform; const v2: btVector3): btVector3;
begin
  result := t1.opTrans(v2);
end;

function btDecide(const a: boolean; const l1, l2: btVector3): btVector3;
begin
   if a then begin
     result := l1;
   end else begin
     result := l2;
   end;
end;

function btDecide(const a: boolean; const l1, l2: PbtVector3): PbtVector3;
begin
   if a then begin
      result := l1;
    end else begin
      result := l2;
    end;
end;

function btAabbSupport(const halfExtents, supportDir: btVector3): btVector3;
var vx,vy,vz:btScalar;
begin
 if supportDir.getX<0 then vx:= -halfExtents.getX else vx:=halfExtents.getX;
 if supportDir.gety<0 then vy:= -halfExtents.gety else vy:=halfExtents.gety;
 if supportDir.getz<0 then vz:= -halfExtents.getz else vz:=halfExtents.getz;
 Result.Init(vx,vy,vz);
end;


function btDot(const v1,v2:btVector3):btScalar;
begin
  result:=v1.dot(v2);
end;
function btDistance(const v1,v2:btVector3):btScalar;
begin
 result:=v1.distance(v2);
end;
function btDistance2(const v1,v2:btVector3):btScalar;
begin
 result:=v1.distance2(v2);
end;
function btAngle(const v1, v2: btVector3): btScalar;
begin
  result:=v1.angle(v2);
end;

function btCross(const v1, v2: btVector3): btVector3;
begin
  result:=v1.cross(v2);
end;

function btTriple(const v1, v2, v3: btVector3): btScalar;
begin
  result:=v1.triple(v2,v3);
end;

function btLerp(const v1, v2: btVector3; const t: btScalar): btVector3;
begin
  result:=v1.lerp(v2,t);
end;

procedure btPlaneSpace1(const n: btVector3; out p, q: btVector3);
var a,k:btScalar;
begin
  if (btFabs(n.getZ) > SIMDSQRT12)  then begin
  // choose p in y-z plane
    a := n[1]*n[1] + n[2]*n[2];
    k := btRecipSqrt(a);
    p.Init(0,-n[2]*k,n[1]*k);
    // set q = n x p
    q.Init(a*k,-n[0]*p[2],n[0]*p[1]);
  end else begin
    // choose p in x-y plane
    a := n.getX*n.getX+ n.getY*n.getY;
    k := btRecipSqrt(a);
    p.Init(-n.getY*k,n.getX*k,0);
    // set q = n x p
    q.init(-n.getZ*p.getY,n.getZ*p.getX,a*k);
  end;
end;

function btVector2.GetItem(i: integer): btScalar;
begin
  result:=v.m_floats[i]; // beware of 0..3 :-)
end;

procedure btVector2.SetItem(i: integer; const AValue: btScalar);
begin
  v.m_floats[i]:=AValue;  // beware of 0..3 :-)
end;

procedure btVector2.Set128(const v128: TFOS_m128);FOS_INLINE;
begin
  v.mVec128:=v128; //TODO SSE
end;

function btVector2.Get128: TFOS_m128;FOS_INLINE;
begin
  result:=v.mVec128;
end;

function btVector2.DumpValues(const rot:boolean=false): string;
begin
  if rot then begin
    result:=format('%4.4f Grad %4.4f Grad %4.4f Grad %4.4f',[btDegrees(v.m_floats[0]),btDegrees(v.m_floats[1]),btDegrees(v.m_floats[2]),btDegrees(v.m_floats[3])]);
  end else begin
    result:=format('x=%4.4f y=%4.4f z=%4.4f w=%4.4f',[v.m_floats[0],v.m_floats[1],v.m_floats[2],v.m_floats[3]]);
  end;
end;

function btVector2.DumpSimple(const vec4:boolean): string;
begin
  if vec4 then begin
    result:=format('%3.3f %3.3f %3.3f %3.3f',[v.m_floats[0],v.m_floats[1],v.m_floats[2],v.m_floats[3]]);
  end else begin
    result:=format('%3.3f %3.3f %3.3f',[v.m_floats[0],v.m_floats[1],v.m_floats[2]]);
  end;
end;

function btVector2.DumpSimpleFull(const vec4: boolean): string;
begin
  if vec4 then begin
    result:=format('%6.6f %6.6f %6.6f %6.6f',[v.m_floats[0],v.m_floats[1],v.m_floats[2],v.m_floats[3]]);
  end else begin
    result:=format('%6.6f %6.6f %6.6f',[v.m_floats[0],v.m_floats[1],v.m_floats[2]]);
  end;
end;

function btVector3.InitS(const xx, yy, zz: btScalar): btVector3;
begin
  result.Init(xx,yy,zz);
end;

function btVector3.Init(const xx, yy, zz: btScalar):btVector3;
begin
  v.m_floats[0]:=xx;
  v.m_floats[1]:=yy;
  v.m_floats[2]:=zz;
  v.m_floats[3]:=0;
  result:=self;
end;

procedure btVector3.InitVector(const v1: btVector3);
begin
 v.m_floats[0]:=v1.v.m_floats[0];
 v.m_floats[1]:=v1.v.m_floats[1];
 v.m_floats[2]:=v1.v.m_floats[2];
end;

procedure btVector3.InitSame(const xx: btScalar);
begin
  v.m_floats[0]:=xx;
  v.m_floats[1]:=xx;
  v.m_floats[2]:=xx;
end;

function btVector3.InitSameS(const xx: btScalar): btVector3;
begin
  result.InitSame(xx);
end;

function btVector3.dot(const v1: btVector3): btScalar;FOS_INLINE;
begin
  result:=v.m_floats[0]*v1.v.m_floats[0]+v.m_floats[1]*v1.v.m_floats[1]+v.m_floats[2]*v1.v.m_floats[2];
end;

function btVector3.absDot(const v1: btVector3): btScalar;
begin
  result:=btFabs(v.m_floats[0]*v1.v.m_floats[0])+btFabs(v.m_floats[1]*v1.v.m_floats[1])+btFabs(v.m_floats[2]*v1.v.m_floats[2]);
end;

function btVector3.length2: btScalar;FOS_INLINE;
begin
  result:=dot(self);
end;

procedure btVector3.length2NormCheck;
begin
  if length2 < (SIMD_EPSILON*SIMD_EPSILON) then begin
    InitSame(-1);
  end;
end;

function btVector3.length: btScalar;FOS_INLINE;
begin
  result:=sqrt(length2);
end;

function btVector3.normalized: btVector3;FOS_INLINE;
begin
 result:=Self/length;
end;

procedure btVector3.normalize;
begin
 self:=self/length;
end;

function btVector3.normalizef: btScalar;
begin
  Result:=length;
  self:=self/Result;
end;

function btVector3.distance(const v1: btVector3): btScalar;
begin
  result:=(v1-Self).length;
end;

function btVector3.distance2(const v1: btVector3): btScalar;
begin
 result:=(v1-Self).length2;
end;

function btVector3.rotate(const wAxis: btVector3; const angle: btScalar): btVector3;FOS_INLINE;
var o,xx,yy:btVector3;
begin
 // wAxis must be a unit lenght vector
 o:= wAxis*wAxis.dot(self);
 xx:= self-o;
 yy:= wAxis.cross(self);
 result:=(o+xx*btCos(angle)+yy*btSin(angle));
end;

function btVector3.cross(const v1: btVector3): btVector3;FOS_INLINE;
begin
 result.Init(v.m_floats[1] * v1.v.m_floats[2] - v.m_floats[2] * v1.v.m_floats[1],
             v.m_floats[2] * v1.v.m_floats[0] - v.m_floats[0] * v1.v.m_floats[2],
             v.m_floats[0] * v1.v.m_floats[1] - v.m_floats[1] * v1.v.m_floats[0]);
end;

function btVector3.angle(const v1: btVector3): btScalar;FOS_INLINE;
var s:btScalar;
begin
 s := btSqrt(length2 * v1.length2);
 result:=btAcos(dot(v1) / s);
end;

function btVector3.absolute: btVector3;FOS_INLINE;
begin
  result.Init(btFabs(v.m_floats[0]),btFabs(v.m_floats[1]),btFabs(v.m_floats[2]));
end;

procedure btVector3.abs_self;
begin
 self.Init(btFabs(v.m_floats[0]),btFabs(v.m_floats[1]),btFabs(v.m_floats[2]));
end;

function btVector3.triple (const v1,v2:btVector3): btScalar;FOS_INLINE;
begin
 result:= v.m_floats[0] * (v1.v.m_floats[1] * v2.v.m_floats[2] - v1.v.m_floats[2] * v2.v.m_floats[1]) +
 	  v.m_floats[1] * (v1.v.m_floats[2] * v2.v.m_floats[0] - v1.v.m_floats[0] * v2.v.m_floats[2]) +
 	  v.m_floats[2] * (v1.v.m_floats[0] * v2.v.m_floats[1] - v1.v.m_floats[1] * v2.v.m_floats[0]);
end;

function btVector3.minAxis: integer;FOS_INLINE;
begin
 //return m_floats[0] < m_floats[1] ? (m_floats[0] <m_floats[2] ? 0 : 2) : (m_floats[1] <m_floats[2] ? 1 : 2);
 if v.m_floats[0] < v.m_floats[1] then begin
   if v.m_floats[0] < v.m_floats[2] then exit(0) else exit(2);
 end else begin
   if v.m_floats[1] < v.m_floats[2] then exit(1) else exit(2);
 end;
end;

function btVector3.maxAxis: integer;FOS_INLINE;
begin
// return m_floats[0] < m_floats[1] ? (m_floats[1] <m_floats[2] ? 2 : 1) : (m_floats[0] <m_floats[2] ? 2 : 0);
 if v.m_floats[0] < v.m_floats[1] then begin
   if v.m_floats[1] < v.m_floats[2] then exit(2) else exit(1);
 end else begin
   if v.m_floats[0] < v.m_floats[2] then exit(2) else exit(0);
 end;

end;

function btVector3.furthestAxis: integer;FOS_INLINE;
begin
 result:=absolute.minAxis;
end;

function btVector3.closestAxis: integer;FOS_INLINE;
begin
 result:= absolute.maxAxis;
end;

procedure btVector3.setInterpolate3(const v0, v1: btVector3; const rt: btScalar);
var s:btScalar;
begin
 s := btScalar(1.0) - rt;
 v.m_floats[0] := s * v0.v.m_floats[0] + rt * v1.v.m_floats[0];
 v.m_floats[1] := s * v0.v.m_floats[1] + rt * v1.v.m_floats[1];
 v.m_floats[2] := s * v0.v.m_floats[2] + rt * v1.v.m_floats[2];
 //don't do the unused w component
 //		m_co[3] = s * v0[3] + rt * v1[3];
end;

// @brief Return the linear interpolation between this and another vector
// @param v The other vector
// @param t The ration of this to v (t = 0 => return this, t=1 => return other)
function btVector3.lerp(const v1: btVector3;const t:btScalar): btVector3;
begin
 result.Init(v.m_floats[0] + (v1.v.m_floats[0] - v.m_floats[0]) * t,
   	     v.m_floats[1] + (v1.v.m_floats[1] - v.m_floats[1]) * t,
 	     v.m_floats[2] + (v1.v.m_floats[2] - v.m_floats[2]) * t);
end;

function btVector3.mulEqals(const v1: btVector3): btVector3;
begin
  v.m_floats[0]:=v.m_floats[0] * v1.v.m_floats[0];
  v.m_floats[1]:=v.m_floats[1] * v1.v.m_floats[1];
  v.m_floats[2]:=v.m_floats[2] * v1.v.m_floats[2];
  result:=self;
end;

function btVector3.addEqals(const v1: btVector3): btVector3;FOS_INLINE;
begin
  v.m_floats[0]:=v.m_floats[0] + v1.v.m_floats[0];
  v.m_floats[1]:=v.m_floats[1] + v1.v.m_floats[1];
  v.m_floats[2]:=v.m_floats[2] + v1.v.m_floats[2];
  result:=self;
end;

function btVector3._X: PbtScalar;FOS_INLINE;
begin
  result:=@v.m_floats[0];
end;

function btVector3._Y: PbtScalar;FOS_INLINE;
begin
 result:=@v.m_floats[1];
end;

function btVector3._Z: PbtScalar;FOS_INLINE;
begin
 result:=@v.m_floats[2];
end;

function btVector3.getX: btScalar;
begin
  result:=v.m_floats[0];
end;

function btVector3.getY: btScalar;
begin
 result:=v.m_floats[1];
end;

function btVector3.getZ: btScalar;
begin
 result:=v.m_floats[2];
end;

function btVector3.X: btScalar;
begin
  result:=v.m_floats[0];
end;

function btVector3.Y: btScalar;
begin
 result:=v.m_floats[1];
end;

function btVector3.Z: btScalar;
begin
 result:=v.m_floats[2];
end;

procedure btVector3.setX(const s: btScalar);
begin
 v.m_floats[0]:=s;
end;

procedure btVector3.setY(const s: btScalar);
begin
 v.m_floats[1]:=s;
end;

procedure btVector3.setZ(const s: btScalar);
begin
 v.m_floats[2]:=s;
end;

procedure btVector3.setMax(const v1: btVector3);
begin
 btSetMax(v.m_floats[0],v1.v.m_floats[0]);
 btSetMax(v.m_floats[1],v1.v.m_floats[1]);
 btSetMax(v.m_floats[2],v1.v.m_floats[2]);
 btSetMax(v.m_floats[3],v1.v.m_floats[3]);
end;

procedure btVector3.setMin(const v1: btVector3);
begin
 btSetMin(v.m_floats[0],v1.v.m_floats[0]);
 btSetMin(v.m_floats[1],v1.v.m_floats[1]);
 btSetMin(v.m_floats[2],v1.v.m_floats[2]);
 btSetMin(v.m_floats[3],v1.v.m_floats[3]);
end;

procedure btVector3.zero;
begin
  v.m_floats[0]:=0;
  v.m_floats[1]:=0;
  v.m_floats[2]:=0;
  v.m_floats[3]:=0;
end;


procedure btVector3.getSkewSymmetricMatrix(var v0, v1, v2: btVector3);
begin
  v0.Init(    0, -getZ ,   getY );
  v1.Init( getZ,     0 ,  -getX );
  v2.Init(-getY,  getX ,      0 );
end;

function btVector3.isZero: boolean;
begin
  result:=(v.m_floats[0]=0) and (v.m_floats[1]=0) and (v.m_floats[2]=0);
end;

function btVector3.fuzzyZero: boolean;
begin
  result:= length2 < SIMD_EPSILON;
end;


{ btVector4 }

function btVector4.Init4S(const xx, yy, zz, ww: btScalar): btVector4;
begin
  result.Init4(xx,yy,zz,ww);
end;

function btVector4.Init4(const xx, yy, zz, ww: btScalar):btVector4;
begin
  v.m_floats[0]:=xx;
  v.m_floats[1]:=yy;
  v.m_floats[2]:=zz;
  v.m_floats[3]:=ww;
  result:=self;
end;

function btVector4._W: PbtScalar;FOS_INLINE;
begin
 result:=@v.m_floats[3];
end;

function btVector4.W: btScalar;
begin
 result:=v.m_floats[3];
end;

function btVector4.getW: btScalar;
begin
  result:=v.m_floats[3];
end;

procedure btVector4.setW(const s: btScalar);
begin
 v.m_floats[3]:=s;
end;

function btVector4.absolute4: btVector4;
begin
  result.Init4(btFabs(v.m_floats[0]),btFabs(v.m_floats[1]),btFabs(v.m_floats[2]),btFabs(v.m_floats[3]));
end;

function btVector4.maxAxis4: integer;
var maxIndex:integer;
    maxVal:btScalar;
begin
 maxIndex:=-1;
 maxVal:=btScalar(-BT_LARGE_FLOAT);
 if (v.m_floats[0] > maxVal) then
 begin
   maxIndex := 0;
   maxVal   := v.m_floats[0];
 end;
 if (v.m_floats[1] > maxVal) then
 begin
   maxIndex := 1;
   maxVal   := v.m_floats[1];
 end;
 if (v.m_floats[2] > maxVal) then
 begin
   maxIndex := 2;
   maxVal   := v.m_floats[2];
 end;
 if (v.m_floats[3] > maxVal) then
 begin
   maxIndex := 3;
   maxVal   := v.m_floats[3];
 end;
 result := maxIndex;
end;

function btVector4.minAxis4: integer;
var minIndex:integer;
    minVal:btScalar;
begin
 minIndex:=-1;
 minVal:=btScalar(BT_LARGE_FLOAT);
 if (v.m_floats[0] < minVal) then
 begin
   minIndex := 0;
   minVal   := v.m_floats[0];
 end;
 if (v.m_floats[1] < minVal) then
 begin
   minIndex := 1;
   minVal   := v.m_floats[1];
 end;
 if (v.m_floats[2] < minVal) then
 begin
   minIndex := 2;
   minVal   := v.m_floats[2];
 end;
 if (v.m_floats[3] < minVal) then
 begin
   minIndex := 3;
   minVal   := v.m_floats[3];
 end;
 result := minIndex;
end;

function btVector4.closestAxis4: integer;
begin
  result:=absolute4.maxAxis4;
end;


{ btQuaternion }

procedure btQuaternion.setValue(const xx, yy, zz, ww: btScalar);
begin
  init4(xx,yy,zz,ww);
end;

function btQuaternion.Init4S(const xx, yy, zz, ww: btScalar): btQuaternion;
begin
 result.Init4(xx,yy,zz,ww);
end;

function btQuaternion.InitQ(const vector: btVector3; const vangle: btScalar):btQuaternion;
begin
  setRotation(vector,vangle);
  Result:=Self;
end;

function btQuaternion.InitQS(const vector: btVector3; const vangle: btScalar): btQuaternion;
begin
  result.InitQ(vector,vangle);
end;

function btQuaternion.InitYPR(const yaw, pitch, roll: btScalar):btQuaternion;
begin
 {$ifndef BT_EULER_DEFAULT_ZYX}
   setEuler(yaw, pitch, roll);
 {$else}
   setEulerZYX(yaw, pitch, roll);
 {$endif}
 result:=Self;
end;

procedure btQuaternion.setRotation(const axis: btVector3; const vangle: btScalar);
var s,d:btScalar;
begin
 d := axis.length;
 s := btSin(vangle * btScalar(0.5)) / d;
 Init4(axis.getX*s,axis.getY*s,axis.getZ*s,btScalar(btCos(vangle*btScalar(0.5))));
end;

///**@brief Set the quaternion using Euler angles
// * @param yaw Angle around Y
// * @param pitch Angle around X
// * @param roll Angle around Z */

procedure btQuaternion.setEuler(const yaw, pitch, roll: btScalar);
var halfYaw,halfPitch,halfRoll,cosYaw,sinYaw,cosPitch,sinPitch,cosRoll,sinRoll:btScalar;
begin
   halfYaw   := btScalar(yaw) * btScalar(0.5);
   halfPitch := btScalar(pitch) * btScalar(0.5);
   halfRoll  := btScalar(roll) * btScalar(0.5);
   cosYaw    := btCos(halfYaw);
   sinYaw    := btSin(halfYaw);
   cosPitch  := btCos(halfPitch);
   sinPitch  := btSin(halfPitch);
   cosRoll   := btCos(halfRoll);
   sinRoll   := btSin(halfRoll);
   Init4 (cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
   	cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
   	sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
   	cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
end;

procedure btQuaternion.setEulerZYX(const yaw, pitch, roll: btScalar);
var halfYaw,halfPitch,halfRoll,cosYaw,sinYaw,cosPitch,sinPitch,cosRoll,sinRoll:btScalar;
begin
   halfYaw   := btScalar(yaw) * btScalar(0.5);
   halfPitch := btScalar(pitch) * btScalar(0.5);
   halfRoll  := btScalar(roll) * btScalar(0.5);
   cosYaw    := btCos(halfYaw);
   sinYaw    := btSin(halfYaw);
   cosPitch  := btCos(halfPitch);
   sinPitch  := btSin(halfPitch);
   cosRoll   := btCos(halfRoll);
   sinRoll   := btSin(halfRoll);
   Init4   (sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
            cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
            cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
            cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
end;

procedure btQuaternion.addEquals(const q: btQuaternion);
begin
  self:=self+q;
end;

procedure btQuaternion.subEquals(const q: btQuaternion);
begin
  self:=self-q;
end;

procedure btQuaternion.mulEquals(const s: btScalar);
begin
  self:=self*s;
end;

procedure btQuaternion.mulEquals(const q: btQuaternion);
begin
  Self:=self*Q;
end;

function btQuaternion.dot(const q: btQuaternion):btScalar;
begin
  result:=v.m_floats[0] * q.getX + v.m_floats[1] * q.getY + v.m_floats[2] * q.getZ+ v.m_floats[3] * q.getW;
end;

function btQuaternion.length2: btScalar;
begin
  result:=dot(Self);
end;

function btQuaternion.length: btScalar;
begin
  result:=btSqrt(length2);
end;

function btQuaternion.normalize:btQuaternion;
begin
  self:=normalized;
  result:=self;
end;

function btQuaternion.normalized: btQuaternion;
begin
  result:=Self/length;
end;

function btQuaternion.angle(const q: btQuaternion): btScalar;
var s:btScalar;
begin
 s      := btSqrt(length2 * q.length2);
 result := btAcos(dot(q)/s);
end;

function btQuaternion.getAngle: btScalar;
begin
 result := btScalar(2)*btAcos(v.m_floats[3]);
end;

function btQuaternion.getAxis: btVector3;
var s_squared,s:btScalar;
begin
 s_squared := btScalar(1) - btPow(v.m_floats[3], btScalar(2));
 if (s_squared < (btScalar(10) * SIMD_EPSILON)) then begin //Check for divide by zero
   result.Init(1.0, 0.0, 0.0);  // Arbitrary
   exit;
 end;
 s := btSqrt(s_squared);
 result.Init(v.m_floats[0] / s, v.m_floats[1] / s, v.m_floats[2] / s);
end;

function btQuaternion.inverse: btQuaternion;
begin
 result.Init4(-v.m_floats[0], -v.m_floats[1], -v.m_floats[2], v.m_floats[3]);
end;

function btQuaternion.sum(const q: btQuaternion): btQuaternion;
begin
  result:=self+q;
end;

function btQuaternion.diff(const q: btQuaternion): btQuaternion;
begin
  result:=self-q;
end;

function btQuaternion.farthest(const qd: btQuaternion): btQuaternion;
var dif,summ:btQuaternion;
begin
  dif   := self - qd;
  summ  := self + qd;
  if( dif.dot(dif) > summ.dot(summ) ) then begin
    result:=qd;
  end else begin
    result:=(-qd);
  end;
end;

function btQuaternion.nearest(const qd: btQuaternion): btQuaternion;
var dif,summ:btQuaternion;
begin
  dif   := self - qd;
  summ  := self + qd;
  if( dif.dot(dif) < summ.dot(summ) ) then begin
    result:=qd;
  end else begin
    result:=(-qd);
  end;
end;

function btQuaternion.slerp(const q: btQuaternion; const t: btScalar): btQuaternion;
var theta,d,s0,s1:btScalar;
begin
 theta := angle(q);
 if (theta <> btScalar(0.0)) then begin
 	d  := btScalar(1.0) / btSin(theta);
 	s0 := btSin((btScalar(1.0) - t) * theta);
 	s1 := btSin(t*theta);
         if (dot(q) < 0) then begin // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
           result.Init4(       (v.m_floats[0] * s0 + -q.getX * s1) * d,
                               (v.m_floats[1] * s0 + -q.getY * s1) * d,
                               (v.m_floats[2] * s0 + -q.getZ * s1) * d,
                               (v.m_floats[3] * s0 + -q.getW * s1) * d);
         end else begin
           Result.Init4(       (v.m_floats[0] * s0 + q.getX * s1) * d,
                               (v.m_floats[1] * s0 + q.getY * s1) * d,
                               (v.m_floats[2] * s0 + q.getZ * s1) * d,
                               (v.m_floats[3] * s0 + q.getW * s1) * d);
         end;
 end else begin
   result:=self;
 end;
end;

function btQuaternion.getIdentity: btQuaternion;
begin
  result.Init4(0,0,0,1);
end;

{ btMatrix }

function btMatrix3x3.GetItems(i: integer): btVector3;
begin
  result:=m_el[i];
end;

function btMatrix3x3.GetItemsP(i: integer): PbtVector3;
begin
 result:=@m_el[i];
end;

procedure btMatrix3x3.SetItems(i: integer; const AValue: btVector3);
begin
 m_el[i]:=AValue;
end;

procedure btMatrix3x3.Init(const q: btQuaternion);
begin
  setRotation(q);
end;

function btMatrix3x3.InitS(const q: btQuaternion): btMatrix3x3;
begin
  result.Init(q);
end;

///** @brief Set the values of the matrix explicitly (row major)
//*  @param xx Top left
//*  @param xy Top Middle
//*  @param xz Top Right
//*  @param yx Middle Left
//*  @param yy Middle Middle
//*  @param yz Middle Right
//*  @param zx Bottom Left
//*  @param zy Bottom Middle
//*  @param zz Bottom Right*/

procedure btMatrix3x3.Init(const xx, xy, xz, yx, yy, yz, zx, zy, zz: btScalar);
begin
  m_el[0].Init(xx,xy,xz);
  m_el[1].Init(yx,yy,yz);
  m_el[2].Init(zx,zy,zz);
end;

function btMatrix3x3.InitS(const xx, xy, xz, yx, yy, yz, zx, zy, zz: btScalar): btMatrix3x3;
begin
  result.Init(xx,xy,xz,yx,yy,yz,zx,zy,zz);
end;

procedure btMatrix3x3.setRotation(const q: btQuaternion);
var d,s,xs,ys,zs,wx,wy,wz, xx,xy,xz,yy,yz,zz :btScalar;
begin
  d := q.length2();
  //btFullAssert(d != btScalar(0.0));
  s  := btScalar(2.0) / d;
  xs := q.getx * s ;  ys := q.gety * s ;  zs := q.getz * s ;
  wx := q.getw * xs;  wy := q.getw * ys;  wz := q.getw * zs;
  xx := q.getx * xs;  xy := q.getx * ys;  xz := q.getx * zs;
  yy := q.gety * ys;  yz := q.gety * zs;  zz := q.getz * zs;
  Init(btScalar(1.0) - (yy + zz), xy - wz, xz + wy,xy + wz, btScalar(1.0) - (xx + zz), yz - wx,xz - wy, yz + wx, btScalar(1.0) - (xx + yy));
end;


function btMatrix3x3.getColumn(const i: integer): btVector3;
begin
  result.Init(m_el[0][i],m_el[1][i],m_el[2][i]);
end;

function btMatrix3x3.getRow(const i: integer): btVector3;
begin
  result:=m_el[i];
end;

function btMatrix3x3.mulEquals(const m: btMatrix3x3): btMatrix3x3;
begin
 init  (m.tdotx(m_el[0]), m.tdoty(m_el[0]), m.tdotz(m_el[0]),
 	m.tdotx(m_el[1]), m.tdoty(m_el[1]), m.tdotz(m_el[1]),
 	m.tdotx(m_el[2]), m.tdoty(m_el[2]), m.tdotz(m_el[2]));
 result:=self;
end;

procedure btMatrix3x3.setfromOpenGLSubMatrix(const m: PbtScalar);
begin
  m_el[0].Init(m[0],m[4],m[8]); //FOS: Think about this
  m_el[1].Init(m[1],m[5],m[9]);
  m_el[2].Init(m[2],m[6],m[10]);
end;

procedure btMatrix3x3.getOpenGLSubMatrix(const m: PbtScalar);
begin
   m[0]  := btScalar(m_el[0].getX);
   m[1]  := btScalar(m_el[1].getX);
   m[2]  := btScalar(m_el[2].getX);
   m[3]  := btScalar(0.0);
   m[4]  := btScalar(m_el[0].getY);
   m[5]  := btScalar(m_el[1].getY);
   m[6]  := btScalar(m_el[2].getY);
   m[7]  := btScalar(0.0);
   m[8]  := btScalar(m_el[0].getZ);
   m[9]  := btScalar(m_el[1].getZ);
   m[10] := btScalar(m_el[2].getZ);
   m[11] := btScalar(0.0);
end;

procedure btMatrix3x3.getOpenGLNormalMatrix(const m: PbtScalar);
begin
   m[0] := btScalar(m_el[0].getX);
   m[1] := btScalar(m_el[1].getX);
   m[2] := btScalar(m_el[2].getX);
   m[3] := btScalar(m_el[0].getY);
   m[4] := btScalar(m_el[1].getY);
   m[5] := btScalar(m_el[2].getY);
   m[6] := btScalar(m_el[0].getZ);
   m[7] := btScalar(m_el[1].getZ);
   m[8] := btScalar(m_el[2].getZ);
end;

///** @brief Set the matrix from euler angles using YPR around YXZ respectively
//*  @param yaw Yaw about Y axis
//*  @param pitch Pitch about X axis
//*  @param roll Roll about Z axis
//*/
procedure btMatrix3x3.setEulerYPR(const yaw, pitch, roll: btScalar);
begin
 setEulerZYX(roll, pitch, yaw);
end;

///** @brief Set the matrix from euler angles YPR around ZYX axes
//* @param eulerX Roll about X axis
//* @param eulerY Pitch around Y axis
//* @param eulerZ Yaw aboud Z axis
//*
//* These angles are used to produce a rotation matrix. The euler
//* angles are applied in ZYX order. I.e a vector is first rotated
//* about X then Y and then Z
//**/
procedure btMatrix3x3.setEulerZYX(const eulerX, eulerY, eulerZ: btScalar);
var ci,cj,ch,si,sj,sh,cc,cs,sc,ss : btScalar;
begin
 ci := btCos(eulerX);
 cj := btCos(eulerY);
 ch := btCos(eulerZ);
 si := btSin(eulerX);
 sj := btSin(eulerY);
 sh := btSin(eulerZ);
 cc := ci * ch;
 cs := ci * sh;
 sc := si * ch;
 ss := si * sh;
 Init(cj * ch, sj * sc - cs, sj * cc + ss,
   	  cj * sh, sj * ss + cc, sj * cs - sc,
 	      -sj,      cj * si,      cj * ci);
end;

procedure btMatrix3x3.setIdentity;
begin
 Init(1,0,0,
          0,1,0,
          0,0,1);
end;

function btMatrix3x3.getIdentity: btMatrix3x3;
begin
  result.Init(1,0,0,
              0,1,0,
              0,0,1);
end;

// /**@brief Get the matrix represented as a quaternion
// * @param q The quaternion which will be set */
procedure btMatrix3x3.getRotation(out q: btQuaternion);
var trace,s:btScalar;
    temp:array[0..3] of btScalar;
    i,j,k:integer;
begin
 trace := m_el[0].getX + m_el[1].getY + m_el[2].getZ;
 if (trace > btScalar(0.0)) then begin
   s       := btSqrt(trace + btScalar(1.0));
   temp[3] := (s * btScalar(0.5));
   s       := btScalar(0.5) / s;
   temp[0] := ((m_el[2].gety - m_el[1].getz) * s);
   temp[1] := ((m_el[0].getz - m_el[2].getx) * s);
   temp[2] := ((m_el[1].getx - m_el[0].gety) * s);
 end else begin
   if m_el[0].getX < m_el[1].getY then begin
      if m_el[1].getY < m_el[2].getZ then begin
        i:=2;
      end else begin
        i:=1;
      end;
   end else begin
      if m_el[0].getX < m_el[2].getZ then begin
        i:=2;
      end else begin
        i:=0;
      end;
   end;
   j       := (i+1) mod 3;
   k       := (i+2) mod 3;
   s       := btSqrt(m_el[i][i] - m_el[j][j] - m_el[k][k] + btScalar(1.0));
   temp[i] := s * btScalar(0.5);
   s       := btScalar(0.5) / s;
   temp[3] := (m_el[k][j] - m_el[j][k]) * s;
   temp[j] := (m_el[j][i] + m_el[i][j]) * s;
   temp[k] := (m_el[k][i] + m_el[i][k]) * s;
 end;
 q.setValue(temp[0],temp[1],temp[2],temp[3]);
end;

///**@brief Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR
//* @param yaw Yaw around Y axis
//* @param pitch Pitch around X axis
//* @param roll around Z axis */

procedure btMatrix3x3.getEulerYPR(var yaw, pitch, roll: btScalar);
begin
 // first use the normal calculus
 yaw   := btScalar(btAtan2(m_el[1].getx, m_el[0].getx));
 pitch := btScalar(btAsin(-m_el[2].getx));
 roll  := btScalar(btAtan2(m_el[2].gety, m_el[2].getz));
 // on pitch = +/-HalfPI
 if (btFabs(pitch)=SIMD_HALF_PI) then begin
 	if (yaw>0) then begin
 	  yaw:=yaw-SIMD_PI;
        end else begin
  	  yaw:=yaw+SIMD_PI;
        end;
 	if (roll>0) then begin
 	  roll := roll-SIMD_PI;
 	end else begin
 	  roll := roll+SIMD_PI;
        end;
 end;
end;

///**@brief Get the matrix represented as euler angles around ZYX
//* @param yaw Yaw around X axis
//* @param pitch Pitch around Y axis
//* @param roll around X axis
//* @param solution_number Which solution of two possible solutions ( 1 or 2) are possible values*/

procedure btMatrix3x3.getEulerZYX(var yaw, pitch, roll: btscalar; var solution_number: cardinal); //TODO FOS SPEEDUP (calc only one solution)
  type TEuler=record
    yaw,pitch,roll:btScalar;
  end;
  var euler_out,euler_out2:TEuler;
      delta:btScalar;
begin
 // Check that pitch is not at a singularity
 if (btFabs(m_el[2].getx) >= 1) then begin
 	euler_out.yaw  := 0;
 	euler_out2.yaw := 0;
 	// From difference of angles formula
 	delta := btAtan2(m_el[0].getx,m_el[0].getz);
 	if (m_el[2].getx > 0) then begin  //gimbal locked up
 		euler_out.pitch  := SIMD_PI / btScalar(2.0);
 		euler_out2.pitch := SIMD_PI / btScalar(2.0);
 		euler_out.roll   := euler_out.pitch + delta;
 		euler_out2.roll  := euler_out.pitch + delta;
 	end else begin // gimbal locked down
 		euler_out.pitch  := -SIMD_PI / btScalar(2.0);
 		euler_out2.pitch := -SIMD_PI / btScalar(2.0);
 		euler_out.roll   := -euler_out.pitch + delta;
 		euler_out2.roll  := -euler_out.pitch + delta;
 	end;
 end else begin
 	euler_out.pitch  := - btAsin(m_el[2].getx);
 	euler_out2.pitch := SIMD_PI - euler_out.pitch;
 	euler_out.roll   := btAtan2(m_el[2].gety/btCos(euler_out.pitch),m_el[2].getz/btCos(euler_out.pitch));
 	euler_out2.roll  := btAtan2(m_el[2].gety/btCos(euler_out2.pitch),m_el[2].getz/btCos(euler_out2.pitch));
 	euler_out.yaw    := btAtan2(m_el[1].getx/btCos(euler_out.pitch),m_el[0].getx/btCos(euler_out.pitch));
 	euler_out2.yaw   := btAtan2(m_el[1].getx/btCos(euler_out2.pitch),m_el[0].getx/btCos(euler_out2.pitch));
 end;
 if (solution_number = 1) then begin
 	yaw   := euler_out.yaw;
 	pitch := euler_out.pitch;
 	roll  := euler_out.roll;
 end else begin
 	yaw   := euler_out2.yaw;
 	pitch := euler_out2.pitch;
 	roll  := euler_out2.roll;
 end;
end;

function btMatrix3x3.scaled(const s: btVector3): btMatrix3x3;
begin
  result.Init(m_el[0].getx * s.getx, m_el[0].gety * s.gety, m_el[0].getz * s.getz,
              m_el[1].getx * s.getx, m_el[1].gety * s.gety, m_el[1].getz * s.getz,
              m_el[2].getx * s.getx, m_el[2].gety * s.gety, m_el[2].getz * s.getz);
end;

function btMatrix3x3.determinant: btScalar;
begin
  result:=btTriple(self[0],self[1],self[2]);
end;

function btMatrix3x3.adjoint: btMatrix3x3;
begin
 Result.Init(cofac(1, 1, 2, 2), cofac(0, 2, 2, 1), cofac(0, 1, 1, 2),
 	     cofac(1, 2, 2, 0), cofac(0, 0, 2, 2), cofac(0, 2, 1, 0),
 	     cofac(1, 0, 2, 1), cofac(0, 1, 2, 0), cofac(0, 0, 1, 1));
end;

function btMatrix3x3.absolute: btMatrix3x3;
begin
 result.Init(
 	     btFabs(m_el[0].getx), btFabs(m_el[0].gety), btFabs(m_el[0].getz),
 	     btFabs(m_el[1].getx), btFabs(m_el[1].gety), btFabs(m_el[1].getz),
 	     btFabs(m_el[2].getx), btFabs(m_el[2].gety), btFabs(m_el[2].getz)
             );
end;

function btMatrix3x3.transpose: btMatrix3x3;
begin
 Result.Init(m_el[0].getx, m_el[1].getx, m_el[2].getx,
 	     m_el[0].gety, m_el[1].gety, m_el[2].gety,
 	     m_el[0].getz, m_el[1].getz, m_el[2].getz);
end;

function btMatrix3x3.inverse: btMatrix3x3;
var co    :btVector3;
    det,s :btScalar;
begin
 co.init(cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2, 1));
 det    := self[0].dot(co);
 s      := btScalar(1.0) / det;
 Result.Init(co.getx * s, cofac(0, 2, 2, 1) * s, cofac(0, 1, 1, 2) * s,
             co.gety * s, cofac(0, 0, 2, 2) * s, cofac(0, 2, 1, 0) * s,
 	     co.getz * s, cofac(0, 1, 2, 0) * s, cofac(0, 0, 1, 1) * s);
end;

function btMatrix3x3.transposeTimes(const m:btMatrix3x3): btMatrix3x3;
begin
 result.Init(
 	m_el[0].getx * m[0].getx + m_el[1].getx * m[1].getx + m_el[2].getx * m[2].getx,
 	m_el[0].getx * m[0].gety + m_el[1].getx * m[1].gety + m_el[2].getx * m[2].gety,
 	m_el[0].getx * m[0].getz + m_el[1].getx * m[1].getz + m_el[2].getx * m[2].getz,
 	m_el[0].gety * m[0].getx + m_el[1].gety * m[1].getx + m_el[2].gety * m[2].getx,
 	m_el[0].gety * m[0].gety + m_el[1].gety * m[1].gety + m_el[2].gety * m[2].gety,
 	m_el[0].gety * m[0].getz + m_el[1].gety * m[1].getz + m_el[2].gety * m[2].getz,
 	m_el[0].getz * m[0].getx + m_el[1].getz * m[1].getx + m_el[2].getz * m[2].getx,
 	m_el[0].getz * m[0].gety + m_el[1].getz * m[1].gety + m_el[2].getz * m[2].gety,
 	m_el[0].getz * m[0].getz + m_el[1].getz * m[1].getz + m_el[2].getz * m[2].getz);
end;

function btMatrix3x3.timesTranspose(const m:btMatrix3x3): btMatrix3x3;
begin
 result.init(
 	m_el[0].dot(m[0]), m_el[0].dot(m[1]), m_el[0].dot(m[2]),
 	m_el[1].dot(m[0]), m_el[1].dot(m[1]), m_el[1].dot(m[2]),
 	m_el[2].dot(m[0]), m_el[2].dot(m[1]), m_el[2].dot(m[2]));

end;

function btMatrix3x3.cofac(r1, c1, r2, c2: integer): btScalar;
begin
 result:= m_el[r1][c1] * m_el[r2][c2] - m_el[r1][c2] * m_el[r2][c1];
end;

function btMatrix3x3.dump: string;
var
  i: Integer;
begin
  for i:=0 to 2 do begin
   result:=result+inttostr(i)+' : '+m_el[i].DumpValues(false)+LineEnding;
  end;
end;

///**@brief diagonalizes this matrix by the Jacobi method.
//* @param rot stores the rotation from the coordinate system in which the matrix is diagonal to the original
//* coordinate system, i.e., old_this = rot * new_this * rot^T.
//* @param threshold See iteration
//* @param iteration The iteration stops when all off-diagonal elements are less than the threshold multiplied
//* by the sum of the absolute values of the diagonal, or when maxSteps have been executed.
//*
//* Note that this matrix is assumed to be symmetric.
//*/

procedure btMatrix3x3.diagonalize(const rot: btMatrix3x3; threshold: btScalar;const maxSteps: Integer);
var i,p,q,r,step:integer;
    max,v,t,mrp,mrq,mpq,theta,theta2,vcos,vsin:btScalar;
    row: btVector3;
begin
 rot.setIdentity();

 step:=maxSteps;
 while step>0 do begin // for (int step = maxSteps; step > 0; step--)
   step:=step-1;
   // find off-diagonal element [p][q] with largest magnitude
   p := 0;
   q := 1;
   r := 2;
   max := btFabs(m_el[0][1]);
   v   := btFabs(m_el[0][2]);
   if (v > max) then begin
     q   := 2;
     r   := 1;
     max := v;
   end;
   v := btFabs(m_el[1][2]);
   if (v > max) then begin
     p   := 1;
     q   := 2;
     r   := 0;
     max := v;
   end;
   t := threshold * (btFabs(m_el[0][0]) + btFabs(m_el[1][1]) + btFabs(m_el[2][2]));
   if (max <= t) then begin
     if (max <= SIMD_EPSILON * t) then exit;
     step := 1;
   end;
 // compute Jacobi rotation J which leads to a zero for element [p][q]
   mpq    := m_el[p][q];
   theta  := (m_el[q][q] - m_el[p][p]) / (2 * mpq);
   theta2 := theta * theta;
   if (theta2 * theta2 < btScalar(10 / SIMD_EPSILON)) then begin
     if theta>=0 then begin
       t := 1 / (theta + btSqrt(1 + theta2))
     end else begin
       t :=  1 / (theta - btSqrt(1 + theta2));
     end;
     vcos := 1 / btSqrt(1 + t*t);
     vsin := vcos * t;

   end else begin
    // approximation for large theta-value, i.e., a nearly diagonal matrix
    t    := 1 / (theta * (2 + btScalar(0.5) / theta2));
    vcos := 1 - btScalar(0.5) * t * t;
    vsin := vcos * t;
    // apply rotation to matrix (this = J^T * this * J)
    m_el[p][q] := 0;
    m_el[q][p] := 0;
    m_el[p][p] := m_el[p][p] - t*mpq;
    m_el[q][q] := m_el[q][q] + t*mpq;
    mrp        := m_el[r][p];
    mrq        := m_el[r][q];
    m_el[r][p] := vcos * mrp - vsin * mrq;
    m_el[p][r] := m_el[r][p];
    m_el[r][q] := vcos * mrq + vsin * mrp;
    m_el[q][r] := m_el[r][q];
    // apply rotation to rot (rot = rot * J)
    for i:=0 to 2 do begin //     for (int i = 0; i < 3; i++)
      row    := rot[i];
      mrp    := row[p];
      mrq    := row[q];
      row[p] := vcos * mrp - vsin * mrq;
      row[q] := vcos * mrq + vsin * mrp;
    end;
   end;
end;

end;

function btMatrix3x3.tdotx(const v: btVector3): btScalar;
begin
  result := m_el[0].getx * v.getx + m_el[1].getx * v.gety + m_el[2].getx * v.getz;
end;

function btMatrix3x3.tdoty(const v: btVector3): btScalar;
begin
  result := m_el[0].gety * v.getx + m_el[1].gety * v.gety + m_el[2].gety * v.getz;
end;

function btMatrix3x3.tdotz(const v: btVector3): btScalar;
begin
  result := (m_el[0].getz * v.getx) + (m_el[1].getz * v.gety) + (m_el[2].getz * v.getz);
end;

{ btTransform }

procedure btTransform.Init(const q: btQuaternion; const c: btVector3);
begin
  m_basis.Init(q);
  m_origin:=c;
end;

procedure btTransform.Init(const b: btMatrix3x3; const c: btVector3);
begin
  m_basis  := b;
  m_origin := c;
end;

procedure btTransform.Init(const b: btMatrix3x3);
begin
  m_basis :=  b;
  m_origin.zero;
end;

///**@brief Set the current transform as the value of the product of two transforms
// * @param t1 Transform 1
// * @param t2 Transform 2
// * This = Transform1 * Transform2 */
procedure btTransform.mult(const t1, t2: btTransform);
begin
  m_basis  := t1.m_basis * t2.m_basis;
  m_origin := t1.opTrans(t2.m_origin);//TODO RETHINK ? m_origin = t1(t2.m_origin);
end;

function btTransform.getRotation: btQuaternion;
begin
  m_basis.getRotation(result);
end;


function btTransform.opTrans(const x: btVector3): btVector3;
begin
  result.Init(m_basis[0].dot(x) + m_origin.getx,
              m_basis[1].dot(x) + m_origin.gety,
              m_basis[2].dot(x) + m_origin.getz);
end;

function btTransform.opMul(const x: btVector3): btVector3;
begin
  result:=Self.opTrans(x);
end;

function btTransform.opMul(const q: btQuaternion): btQuaternion;
begin
  result := getRotation * q;
end;

function btTransform.opMul(const t: btTransform): btTransform;
begin
 result.Init(m_basis * t.m_basis,self.opTrans(t.m_origin));
end;

//function btTransform.getBasis: btMatrix3x3;
//begin
//  result:=m_basis;
//end;

function btTransform.getBasisV: PbtMatrix3x3;
begin
  result:=@m_basis;
end;

//function btTransform.getOrigin: btVector3;
//begin
//  result:=m_origin;
//end;

function btTransform.getOriginV: PbtVector3;
begin
  result:=@m_origin;
end;

function btTransform.dump: string;
begin
  result:='Basis:'+LineEnding;
  result:=result+m_basis.dump+LineEnding;
  result:=result+'Origin:'+LineEnding;
  result:=Result+m_origin.DumpValues(false)+LineEnding;
end;


///**@brief Set from an array
// * @param m A pointer to a 15 element array (12 rotation(row major padded on the right by 1), and 3 translation */
procedure btTransform.setFromOpenGLMatrix(const m: PbtScalar);
begin
   m_basis.setFromOpenGLSubMatrix(m);
   m_origin.Init(m[12],m[13],m[14]);
end;

procedure btTransform.getOpenGLMatrix(const m: PbtScalar);
begin
  m_basis.getOpenGLSubMatrix(m);
  m[12] := m_origin.getx;
  m[13] := m_origin.gety;
  m[14] := m_origin.getz;
  m[15] := btScalar(1.0);
end;

procedure btTransform.setOrigin(const origin: btVector3);
begin
  m_origin:=origin;
end;

procedure btTransform.setBasis(const m: btMatrix3x3);
begin
  m_basis:=m;
end;

procedure btTransform.setRotation(const q: btQuaternion);
begin
  m_basis.setRotation(q);
end;

procedure btTransform.setIdentity;
begin
  m_basis.setIdentity;
  m_origin.Init(0,0,0);
end;

///**@brief Multiply this Transform by another(this = this * another)
// * @param t The other transform */

function btTransform.mulEquals(const t:btTransform): btTransform;
begin
  m_origin := m_origin + (m_basis*t.m_origin);
  m_basis  := m_basis * t.m_basis;
  result   := self;
end;

///**@brief Return the inverse of this transform */
function btTransform.inverse: btTransform;
var inv:btMatrix3x3;
begin
  inv    := m_basis.transpose;
  result.Init(inv, inv * -m_origin);
end;

function btTransform.inverseTimes(const t: btTransform): btTransform;
var v:btVector3;
begin
 v := t.getOriginV^ - m_origin;
 result.Init(m_basis.transposeTimes(t.m_basis),v * m_basis);
end;

var identityTransform:btTransform;

function btTransform.identityTransform: btTransform;
begin
  result:=identityTransform;
end;

function btTransform.invXform(const inVec: btVector3): btVector3;
var v:btVector3;
begin
 v := inVec - m_origin;
 result:=(m_basis.transpose*v);
end;

{ btTransformUtil }

procedure btTransformUtil.integrateTransform(const curTrans: btTransform; const linvel, angvel: btVector3; const timeStep: btScalar; out predictedTransform: btTransform);
var predictedOrn:btQuaternion;
    axis:btVector3;
    fangle:btScalar;
    dorn,orn0:btQuaternion;
begin
 predictedTransform.setOrigin(curTrans.getOriginV^ + linvel * timeStep);
{.$define QUATERNION_DERIVATIVE}
{$ifdef QUATERNION_DERIVATIVE}
 predictedOrn := curTrans.getRotation;
 predictedOrn := predictedOrn+ ((angvel * predictedOrn) * (timeStep * btScalar(0.5)));
 predictedOrn.normalize;
{$else}
 //Exponential map
 //google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia

 fAngle := angvel.length;
 //limit the angular motion
 if (fAngle*timeStep > ANGULAR_MOTION_THRESHOLD) then
 begin
   fAngle := ANGULAR_MOTION_THRESHOLD / timeStep;
 end;
 if ( fAngle < btScalar(0.001) ) then
 begin
  // use Taylor's expansions of sync function
  axis   := angvel*( btScalar(0.5)*timeStep-(timeStep*timeStep*timeStep)*(btScalar(0.020833333333))*fAngle*fAngle );
 end else begin
  // sync(fAngle) = sin(c*fAngle)/t
  axis   := angvel*( btSin(btScalar(0.5)*fAngle*timeStep)/fAngle );
 end;
 dorn.Init4   (axis.getx,axis.gety,axis.getz,btCos(fAngle*timeStep*btScalar(0.5)));
 orn0         := curTrans.getRotation;
 predictedOrn := dorn * orn0;
 predictedOrn.normalize;
{$endif}
 predictedTransform.setRotation(predictedOrn);
end;

procedure btTransformUtil.calculateVelocityQuaternion(const pos0, pos1: btVector3; const orn0, orn1: btQuaternion; const timeStep: btScalar; out linVel, angVel: btVector3);
var axis:btVector3;
    angle:btScalar;
begin
 linVel := (pos1 - pos0) / timeStep;
 if (orn0 <> orn1) then begin
   calculateDiffAxisAngleQuaternion(orn0,orn1,axis,angle);
   angVel := axis * angle / timeStep;
 end else begin
   angVel.Init(0,0,0);
 end;
end;

procedure btTransformUtil.calculateDiffAxisAngleQuaternion(const orn0, orn1a: btQuaternion; out axis: btVector3; out angle: btScalar);
var orn1,dorn:btQuaternion;
    len:btScalar;
begin
  orn1    := orn0.nearest(orn1a);
  dorn    := orn1 * orn0.inverse;
  angle   := dorn.getAngle;
  axis.init(dorn.getx,dorn.gety,dorn.getz);
  axis[3] := btScalar(0);
  //check for axis length
  len     := axis.length2;
  if (len < SIMD_EPSILON*SIMD_EPSILON) then begin
    axis.init(1,0,0);
  end else begin
    axis  := axis / btSqrt(len);
  end;
end;

procedure btTransformUtil.calculateVelocity(const transform0,transform1: btTransform; const timeStep: btScalar; out linVel,angVel: btVector3);
var axis:btVector3;
    angle:btScalar;
begin
  linVel := (transform1.getOriginV^ - transform0.getOriginV^) / timeStep;
  calculateDiffAxisAngle(transform0,transform1,axis,angle);
  angVel := axis * angle / timeStep;
end;

procedure btTransformUtil.calculateDiffAxisAngle(const transform0,transform1: btTransform; out axis: btVector3; out angle:btScalar);
var dmat:btMatrix3x3;
    dorn:btQuaternion;
    len :btScalar;
begin
  dmat := transform1.GetBasisV^ * transform0.GetBasisV^.inverse();
  dmat.getRotation(dorn);
  ///floating point inaccuracy can lead to w component > 1..., which breaks
  dorn.normalize;
  angle := dorn.getAngle;
  axis.Init(dorn.getx,dorn.gety,dorn.getz);
  axis[3] := btScalar(0);
  //check for axis length
  len := axis.length2;
  if (len < SIMD_EPSILON*SIMD_EPSILON) then begin
    axis.init(1,0,0);
  end else begin
    axis /= btSqrt(len);
  end;
end;

{ btConvexSeparatingDistanceUtil }

procedure btConvexSeparatingDistanceUtil.init(const boundingRadiusA,boundingRadiusB: btScalar);
begin
  m_boundingRadiusA:=boundingRadiusA;
  m_boundingRadiusB:=boundingRadiusB;
  m_separatingDistance:=0;
end;

function btConvexSeparatingDistanceUtil.getConservativeSeparatingDistance: btScalar;
begin
  result:=m_separatingDistance;
end;

var gbtTransformUtil:btTransformUtil;

procedure btConvexSeparatingDistanceUtil.updateSeparatingDistance(const transA, transB: btTransform);
var toPosA,toPosB:btVector3;
    toOrnA,toOrnB:btQuaternion;
    // relLinVel,
    linVelA,angVelA,linVelB,angVelB:btVector3;
    relLinVelocLength,projectedMotion,maxAngularProjectedVelocity:btScalar;
begin
   toPosA := transA.getOriginV^;
   toPosB := transB.getOriginV^;
   toOrnA := transA.getRotation;
   toOrnB := transB.getRotation;

 if (m_separatingDistance>0) then begin
 	gbtTransformUtil.calculateVelocityQuaternion(m_posA,toPosA,m_ornA,toOrnA,btScalar(1),linVelA,angVelA);
 	gbtTransformUtil.calculateVelocityQuaternion(m_posB,toPosB,m_ornB,toOrnB,btScalar(1),linVelB,angVelB);
 	maxAngularProjectedVelocity := (angVelA.length * m_boundingRadiusA) + (angVelB.length*m_boundingRadiusB);
 	//relLinVel := (linVelB-linVelA);
 	relLinVelocLength := (linVelB-linVelA).dot(m_separatingNormal);
 	if (relLinVelocLength<0) then
        begin
       	  relLinVelocLength := 0;
        end;
        projectedMotion := maxAngularProjectedVelocity +relLinVelocLength;
 	m_separatingDistance -= projectedMotion;
 end;
 m_posA := toPosA;
 m_posB := toPosB;
 m_ornA := toOrnA;
 m_ornB := toOrnB;

end;

procedure btConvexSeparatingDistanceUtil.initSeparatingDistance(const separatingVector: btVector3; const separatingDistance: btScalar;const transA, transB: btTransform);
var toPosA,toPosB:btVector3;
    toOrnA,toOrnB:btQuaternion;
begin
  m_separatingDistance := separatingDistance;
  if (m_separatingDistance>0) then begin
        m_separatingNormal := separatingVector;
        toPosA             := transA.getOriginV^;
        toPosB             := transB.getOriginV^;
        toOrnA             := transA.getRotation;
        toOrnB             := transB.getRotation;
        m_posA             := toPosA;
        m_posB             := toPosB;
        m_ornA             := toOrnA;
        m_ornB             := toOrnB;
  end;
end;

{ btHullResult }

destructor btHullResult.Destroy;
begin
  m_OutputVertices.free;
  m_Indices.free;
end;

constructor btHullResult.create;
begin
  mPolygons          := true;
  mNumOutputVertices := 0;
  mNumFaces          := 0;
  mNumIndices        := 0;
  m_OutputVertices   := btFOSAlignedVectorArray.create;
  m_Indices          := btFOSAlignedCardinals.create;
end;



{ btHullDesc }

procedure btHullDesc.Init;
begin
  mFlags          := [QF_DEFAULT];
  mVcount         := 0;
  mVertices       := nil;
  mVertexStride   := sizeof(btVector3);
  mNormalEpsilon  := 0.001;
  mMaxVertices    := 4096; // maximum number of points to be considered for a convex hull.
  mMaxFaces       := 4096;
end;

procedure btHullDesc.Init(const flags: btHullFlagSet; const vcount: cardinal;  const vertices: PbtVector3; const stride: cardinal);
begin
  mFlags          := flags;
  mVcount         := vcount;
  mVertices       := vertices;
  mVertexStride   := stride;
  mNormalEpsilon  := 0.001;
  mMaxVertices    := 4096; // maximum number of points to be considered for a convex hull.
  mMaxFaces       := 4096;
end;

function btHullDesc.HasHullFlag(const flag: btHULLFLAG): boolean;
begin
  result := flag in mFlags;
end;

procedure btHullDesc.SetHullFlag(const flag: btHULLFLAG);
begin
  Include(mFlags,flag);
end;

procedure btHullDesc.ClearHullFlag(const flag: btHULLFLAG);
begin
  Exclude(mFlags,flag);
end;

{ btPlane }

procedure btPlane.Init(const v: btVector3; const d: btScalar);
begin
  normal:=v;
  dist:=d;
end;

{ _HalfEdge }

procedure btHalfEdge.Init(const _ea: SmallInt; const _v, _p: Byte);
begin
  ea:=_ea;
  v:=_v;
  p:=_p;
end;

{ btConvexH }

procedure btConvexH.Init(const vertices_size, edges_size, facets_size: integer);
begin
  vertices.Setlength(vertices_size);
  edges.Setlength(edges_size);
  facets.Setlength(facets_size);
end;

{ btPHullResult }

constructor btPHullResult.create;
begin
  m_indices := btFOSAlignedCardinals.create;
end;

destructor btPHullResult.Destroy;
begin
  m_indices.Free;
  inherited;
end;

{ btHullLibrary }

function maxdirfiltered(const p : PbtVector3 ; const count : integer ;const dir : btVector3; var allow : btFOSAlignedIntegers):integer;
var m : Integer;
  i: Integer;
begin
  btAssert(count<>0);
  m := -1;
  for i:=0 to count-1 do begin
    if allow.A[i]^<>0 then begin
      if (m=-1) or (btDot(p[i],dir)>btDot(p[m],dir)) then begin
        m:=i;
      end;
    end;
  end;
  btAssert(m <> -1);
  Result := m;
end;


function maxdirsterid(const  p : PbtVector3;const count : integer ; const dir : btVector3 ;var allow : btFOSAlignedIntegers):integer;
var m,mc     : integer;
    u,v      : btVector3;
    ma,mb,md : integer;
    x,s,c,xx : btScalar;

begin
  m := -1;
  while (m=-1) do begin
    m := maxdirfiltered(p,count,dir,allow);
    if allow.A[m]^=3 then begin
      Result := m;
    end;
    u  := btorth(dir);
    v  := btCross(u,dir);
    ma := -1;
    x:=0;
//    for(btScalar x = btScalar(0.0) ; x<= btScalar(360.0) ; x+= btScalar(45.0))
    while (x <= 360) do begin
      s  := btSin(SIMD_RADS_PER_DEG*(x));
      c  := btCos(SIMD_RADS_PER_DEG*(x));
      mb := maxdirfiltered(p,count,dir+(u*s+v*c)*0.025,allow);
      if (ma=m) and (mb=m) then begin
        allow.A[m]^ := 3;
        Result := m;
        exit;
      end;
      if (ma<>-1)  and (ma<>mb) then begin  // Yuck - this is really ugly
        mc := ma;
//        for(btScalar xx = x-btScalar(40.0) ; xx <= x ; xx+= btScalar(5.0))
        xx := x - 40;
        while (xx <= x) do  begin
          s := btSin(SIMD_RADS_PER_DEG*(xx));
          c := btCos(SIMD_RADS_PER_DEG*(xx));
          md := maxdirfiltered(p,count,dir+(u*s+v*c)*btScalar(0.025),allow);
          if (mc=m) and (md=m) then begin
            allow.A[m]^ := 3;
            Result := m;
            exit;
          end;
          mc:=md;
          xx := xx + 5;
        end;
      end;
      ma:=mb;
     x := x  + 45;
    end;
    allow.A[m]^:=0;
    m:=-1;
  end;
  btAssert(false);
  Result := m;
end;


function btHullLibrary.ComputeHull(const vcount: cardinal; const vertices: PbtVector3; var res: btPHullResult; const vlimit: cardinal): boolean;
var tris_count : integer;
    ret        : integer;
begin
 tris_count := 0;
 ret := calchull(vertices,vcount, res.m_Indices, tris_count,integer(vlimit));
 if ret=0 then exit(false);

 res.mIndexCount := cardinal(tris_count*3);
 res.mFaceCount  := cardinal(tris_count);
 res.mVertices   := vertices;
 res.mVcount     := cardinal(vcount);
 Result := true;
end;

function btHullLibrary.allocateTriangle(const a, b, c: integer): btHullTriangle;
begin
  result:=btHullTriangle.Create(a,b,c);
  Result.id := m_tris.Length;
  m_tris.push_back(result);
end;

procedure btHullLibrary.deAllocateTriangle(const triangle: btHullTriangle);
begin
  btAssert(m_tris.A[triangle.id]^=triangle);
  m_tris.A[triangle.id]^:=nil;
  triangle.free;
end;

procedure btHullLibrary.b2bFix(const s, t: btHullTriangle);
var i,i1,i2,a,b:integer;
begin
  for i:=0 to 2 do begin
    i1 := (i+1) mod 3;
    i2 := (i+2) mod 3;
    a  := s[i1];
    b  := s[i2];
    btAssert(m_tris.A[s.neib(a,b)^]^.neib(b,a)^ = s.id);
    btAssert(m_tris.A[t.neib(a,b)^]^.neib(b,a)^ = t.id);
    m_tris.A[s.neib(a,b)^]^.neib(b,a)^ := t.neib(b,a)^;
    m_tris.A[t.neib(b,a)^]^.neib(a,b)^ := s.neib(a,b)^;
  end;
end;

procedure btHullLibrary.removeb2b(const s, t: btHullTriangle);
begin
  b2bfix(s,t);
  deAllocateTriangle(s);
  deAllocateTriangle(t);
end;

procedure btHullLibrary.checkit(const t: btHullTriangle);
var i,i1,i2,a,b:integer;
begin
// (void)t;
 btAssert(m_tris.A[t.id]^=t);
 for i:=0 to 2 do begin
    i1 :=(i+1) mod 3;
    i2 :=(i+2) mod 3;
    a  := t[i1];
    b  := t[i2];
    btAssert(a<>b);
    btAssert(m_tris.A[t.n[i]]^.neib(b,a)^ = t.id);
 end;
end;

function btHullLibrary.extrudable(const epsilon: btScalar): btHullTriangle;
var i:integer;
    t:btHullTriangle;
begin
  t := Nil;
  for i:=0 to m_tris.size-1 do begin
    if not assigned(t)  or (assigned(m_tris.A[i]^) and  (t.rise < m_tris.A[i]^.rise)) then begin
      t := m_tris.A[i]^;
    end;
  end;
  Result := btHullTriangle(btDecide(t.rise>epsilon , t , nil));
end;

function btHullLibrary.calchull(const verts: PbtVector3; const verts_count: integer; var tris_out: TFOS_AlignedCardinals; var tris_count: integer; const vlimit: integer): integer;
var rc,i,j : integer;
    ts     : btFOSAlignedIntegers;
begin
  rc := calchullgen(verts,verts_count,  vlimit) ;
  if rc=0 then begin
   exit(0);
  end;
  ts := btFOSAlignedIntegers.create;
  for i:=0 to m_tris.size-1 do begin
    if assigned(m_tris.A[i]^) then begin
      for j:=0 to 2 do begin
        ts.push_back((m_tris.A[i]^)[j]);
      end;
      deAllocateTriangle(m_tris.A[i]^);
    end;
 end;
 tris_count := ts.size div 3;
 tris_out.resize(ts.size);
 for i:=0 to ts.size-1 do begin
  tris_out.A[i]^ := cardinal(ts.A[i]^);
 end;
 m_tris.resize(0);
 ts.Free;
 Result := 1;
end;

function btHullLibrary.calchullgen(const verts: PbtVector3; const verts_count: integer; vlimit: integer): integer;
var j         : Integer;
    bmin,bmax : btVector3;
    isextreme,
    allow     : btFOSAlignedIntegers;
    epsilon   : btScalar;
    p         : btInt4;
    center,n  : btVector3;
    t0,t1,te,
    nb,
    t2,t3,t   : btHullTriangle;
    nt     : btInt3;
    v         : integer;

    function _LoopCond:boolean;
    begin
      te := extrudable(epsilon);
      result := (vlimit>0) and (te<>nil);
    end;

begin
  if verts_count< 4 then begin
    exit(0);
  end;
  if vlimit=0 then begin
    vlimit:=1000000000;
  end;
  bmin := verts^;
  bmax := verts^;
  isextreme := btFOSAlignedIntegers.create;
  allow     := btFOSAlignedIntegers.create;
  isextreme.reserve(verts_count);
  allow.reserve(verts_count);
  try
    for j:=0 to verts_count-1 do begin
      allow.push_back(1);
      isextreme.push_back(0);
      bmin.setMin(verts[j]);
      bmax.setMax(verts[j]);
    end;
    epsilon := (bmax-bmin).length * btScalar(0.001);
    btAssert (epsilon <> 0);

    p := FindSimplex(verts,verts_count,allow);
    try
      if p.x=-1 then begin
        exit(0); // simplex failed
      end;

      center.InitVector(verts[p[0]^]+verts[p[1]^]+verts[p[2]^]+verts[p[3]^]);  // a valid interior point
      center := center / 4;
      t0 := allocateTriangle(p[2]^,p[3]^,p[1]^); t0.n := btint3.Create; t0.n.Initialize(2,3,1);
      t1 := allocateTriangle(p[3]^,p[2]^,p[0]^); t1.n := btint3.Create; t1.n.Initialize(3,2,0);
      t2 := allocateTriangle(p[0]^,p[1]^,p[3]^); t2.n := btint3.Create; t2.n.Initialize(0,1,3);
      t3 := allocateTriangle(p[1]^,p[0]^,p[2]^); t3.n := btint3.Create; t3.n.Initialize(1,0,2);

      isextreme.A[p[3]^]^ := 1;
      isextreme.A[p[2]^]^ := 1;
      isextreme.A[p[1]^]^ := 1;
      isextreme.A[p[0]^]^ := 1;
      checkit(t0);checkit(t1);checkit(t2);checkit(t3);
    finally
      p.free;
      p:=nil;
    end;

    for j:=0 to m_tris.size-1 do begin
      t := m_tris.A[j]^;
      btAssert(assigned(t));
      btAssert(t.vmax < 0);
      n := btTriNormal(verts[t[0]],verts[t[1]],verts[t[2]]);
      t.vmax := maxdirsterid(verts,verts_count,n,allow);
      t.rise := btDot(n,verts[t.vmax]-verts[t[0]]);
    end;

    vlimit -= 4;
    //while(vlimit >0 && ((te=extrudable(epsilon)) != 0))
    while _LoopCond do begin
//      ti := te;
      v  := te.vmax;
      btAssert(v <> -1);
      btAssert(isextreme.A[v]^=0);  // wtf we've already done this vertex
      isextreme.A[v]^ := 1;
      //if(v==p0 || v==p1 || v==p2 || v==p3) continue; // done these already
      j := m_tris.size;
      while btVarMinusMinus(j)<>0 do begin
       if not assigned(m_tris.A[j]^) then continue;
        t := m_tris.A[j]^;
        if btAbove(verts,t,verts[v],btScalar(0.01)*epsilon) then begin
          extrude(m_tris.A[j]^,v);
        end;
      end;
      // now check for those degenerate cases where we have a flipped triangle or a really skinny triangle
      j:=m_tris.size;
      while btVarMinusMinus(j)<>0 do begin
        if not assigned(m_tris.A[j]^) then continue;
        if not btHasVert(m_tris.A[j]^,v) then break;
        nt := m_tris.A[j]^;
        if btAbove(verts,nt,center,btScalar(0.01)*epsilon) or (btCross(verts[nt[1]]-verts[nt[0]],verts[nt[2]]-verts[nt[1]]).length< (epsilon*epsilon*btScalar(0.1))) then begin
          nb := m_tris.A[m_tris.A[j]^.n[0]]^;
          btAssert(assigned(nb));
          btAssert(not bthasvert(nb,v));
          btAssert(nb.id<j);
          extrude(nb,v);
          j := m_tris.size;
        end;
      end;
      j:=m_tris.size;
      while btVarMinusMinus(j)<>0 do begin
        t := m_tris.A[j]^;
        if not assigned(t) then continue;
        if t.vmax>=0 then break;
        n := btTriNormal(verts[t[0]],verts[t[1]],verts[t[2]]);
        t.vmax := maxdirsterid(verts,verts_count,n,allow);
        if isextreme.A[t.vmax]^<>0 then begin
          t.vmax := -1; // already done that vertex - algorithm needs to be able to terminate.
        end else begin
          t.rise := btDot(n,verts[t.vmax]-verts[t[0]]);
        end;
      end;
      dec(vlimit);
    end;
    exit(1);
  finally
    isextreme.free;
    allow.free;
  end;
end;

function btHullLibrary.FindSimplex(const verts: PbtVector3;const verts_count: integer; var allow: TFOS_AlignedIntegers): btInt4;
var  basis       : array [0..2] of btVector3;
     p0,p1,p2,p3 : integer;
begin
 basis[0] := btVector3.Inits(0.01,0.02,1.0);
 p0       := maxdirsterid(verts,verts_count, basis[0],allow);
 p1       := maxdirsterid(verts,verts_count,-basis[0],allow);
 basis[0] := verts[p0]-verts[p1];
 if (p0=p1) or (basis[0].isZero) then begin
   result := btInt4.Create(-1,-1,-1,-1);
   exit;
 end;
 basis[1] := btCross(btVector3.InitS(    1, 0.02, 0),basis[0]);
 basis[2] := btCross(btVector3.InitS(-0.02,    1, 0),basis[0]);
 if basis[1].length > basis[2].length then begin
    basis[1].normalize;
 end else begin
    basis[1] := basis[2];
    basis[1].normalize;
 end;
 p2 := maxdirsterid(verts,verts_count,basis[1],allow);
 if (p2=p0) or (p2=p1) then begin
   p2 := maxdirsterid(verts,verts_count,-basis[1],allow);
 end;
 if (p2=p0) or (p2=p1) then begin
   Result := btint4.create(-1,-1,-1,-1);
   exit;
 end;
 basis[1] := verts[p2] - verts[p0];
 basis[2] := btCross(basis[1],basis[0]).normalized;
 p3 := maxdirsterid(verts,verts_count,basis[2],allow);
 if(p3=p0) or (p3=p1) or (p3=p2) then begin
   p3 := maxdirsterid(verts,verts_count,-basis[2],allow);
 end;
 if (p3=p0) or (p3=p1) or (p3=p2) then begin
   Result := btInt4.create(-1,-1,-1,-1);
   exit;
 end;
 btAssert(not ((p0=p1) or (p0=p2) or (p0=p3) or (p1=p2) or (p1=p3) or (p2=p3)));
 if(btDot(verts[p3]-verts[p0],btCross(verts[p1]-verts[p0],verts[p2]-verts[p0])) <0) then begin
   btSwap(p2,p3);
 end;
 Result := btint4.create(p0,p1,p2,p3);
end;


procedure btHullLibrary.extrude(const t0: btHullTriangle; const v: integer);
var t  : btint3;
    n  : integer;
    ta : btHullTriangle;
    tb : btHullTriangle;
    tc : btHullTriangle;
begin
 t := t0;
 n := m_tris.size;
 ta := allocateTriangle(v,t[1],t[2]);
 ta.n := btInt3.Create; ta.n.Initialize(t0.n[0],n+1,n+2);
 m_tris.A[t0.n[0]]^.neib(t[1],t[2])^ := n+0;

 tb := allocateTriangle(v,t[2],t[0]);
 tb.n := btInt3.Create; tb.n.initialize(t0.n[1],n+2,n+0);
 m_tris.A[t0.n[1]]^.neib(t[2],t[0])^ := n+1;
 tc := allocateTriangle(v,t[0],t[1]);
 tc.n := btint3.Create; tc.n.Initialize(t0.n[2],n+0,n+1);
 m_tris.A[t0.n[2]]^.neib(t[0],t[1])^ := n+2;

 checkit(ta);
 checkit(tb);
 checkit(tc);
 if btHasVert(m_tris.A[ta.n[0]]^,v) then begin
   removeb2b(ta,m_tris.A[ta.n[0]]^);
 end;
 if btHasVert(m_tris.A[tb.n[0]]^,v) then begin
   removeb2b(tb,m_tris.A[tb.n[0]]^);
 end;
 if btHasVert(m_tris.A[tc.n[0]]^,v) then begin
   removeb2b(tc,m_tris.A[tc.n[0]]^);
 end;
 deAllocateTriangle(t0);
end;


procedure btHullLibrary.BringOutYourDead(const hr:btPHullResult ; const overts: PbtVector3; var ocount: cardinal);
var tmpIndices  : TFOS_AlignedIntegers;
    usedIndices : TFOS_AlignedCardinals;
    i,k         : integer;
    v           : cardinal;
begin
 tmpIndices := btFOSAlignedIntegers.create;
 tmpIndices.resize(m_vertexIndexMapping.size);
  for i :=0 to m_vertexIndexMapping.size-1 do begin
    tmpIndices.A[i]^ := m_vertexIndexMapping.A[i]^;
  end;
  usedIndices := btFOSAlignedCardinals.create;
  usedIndices.resize(hr.mVcount);
  usedIndices.InitValues(0);
  //memset(&usedIndices[0],0,sizeof(unsigned int)*vcount); //FOSTODO Check Zero initialisation
  ocount := 0;
  for i :=0 to hr.mIndexCount-1 do begin
    v := hr.m_indices.A[i]^; // original array index
    btAssert( (v>=0) and (v<hr.mVcount) );
    if usedIndices.A[v]^<>0 then begin // if already remapped
      hr.m_indices.A[i]^ := usedIndices.A[v]^-1; // index to new array
    end else begin
      hr.m_indices.A[i]^  := ocount;      // new index mapping
      overts[ocount][0] := hr.mVertices[v][0]; // copy old vert to new vert array
      overts[ocount][1] := hr.mVertices[v][1];
      overts[ocount][2] := hr.mVertices[v][2];
      for k := 0 to m_vertexIndexMapping.size-1 do begin
        if tmpIndices.A[k]^=integer(v) then begin
          m_vertexIndexMapping.A[k]^ := ocount;
        end;
      end;
      inc(ocount); // increment output vert count
      btAssert( (ocount>=0) and (ocount <= hr.mVcount) );
      usedIndices.A[v]^ := ocount; // assign new index remapping
    end;
  end;
  tmpIndices.free;
  usedIndices.free;
end;


function btHullLibrary.CleanupVertices(const svcount: cardinal; const svertices: PbtVector3; const stride: cardinal; var vcount: cardinal; const vertices: PbtVector3; const normalepsilon: btScalar; const scale: PbtVector3): boolean;
var recip,bmin,bmax : array [0..2] of btScalar;
    vtx             : PByte;
    dx,dy,dz,len    : btScalar;
    p               : PbtScalar;
    center          : btVector3;
    x1,x2,y1,
    y2,z1,z2,
    cx,cy,cz,
    px,py,pz        : btScalar;
    pp              : PbtVector3;
    v,dest          : PbtVector3;
    x,y,z,
    dist1,dist2     : btScalar;
    i               : integer;
    j               : cardinal;

    procedure addPoint(var vcount:cardinal ;const p : PbtVector3 ; const  x,y,z : btScalar);
    var dest:PbtVector3;
    begin
    	// XXX, might be broken
      dest := @p[vcount];
      dest^[0] := x; dest^[1] := y; dest^[2] := z;
      inc(vcount);
    end;
    function GetDist(const px,py,pz : btScalar ; const p2 : btVector3) : btScalar;
    var dx,dy,dz : btScalar;
    begin
      dx := px - p2[0];
      dy := py - p2[1];
      dz := pz - p2[2];
      Result := dx*dx+dy*dy+dz*dz;
    end;

begin
  if svcount = 0 then exit(false);
  m_vertexIndexMapping.resize(0);

  vcount   := 0; recip[0] := 0; recip[1] := 0; recip[2] := 0;
  if assigned(scale) then begin
    scale^[0] := 1;
    scale^[1] := 1;
    scale^[2] := 1;
  end;
  bmin[0] :=  SIMD_INFINITY ; bmin[1] :=  SIMD_INFINITY ; bmin[2] :=  SIMD_INFINITY;
  bmax[0] := -SIMD_INFINITY ; bmax[1] := -SIMD_INFINITY ; bmax[2] := -SIMD_INFINITY;
  vtx := PByte(svertices);
  for i := 0 to svcount-1 do begin
    p   := PbtScalar(vtx);
    vtx := vtx + stride;
    for j := 0 to 2 do begin
      if  p[j] < bmin[j] then begin
        bmin[j] := p[j];
      end;
      if  p[j] > bmax[j] then begin
        bmax[j] := p[j];
      end;
    end;
  end;
  dx := bmax[0] - bmin[0];
  dy := bmax[1] - bmin[1];
  dz := bmax[2] - bmin[2];
  center[0] := dx*btScalar(0.5) + bmin[0];
  center[1] := dy*btScalar(0.5) + bmin[1];
  center[2] := dz*btScalar(0.5) + bmin[2];
  if (dx < cu_EPSILON) or (dy < cu_EPSILON)  or  (dz < cu_EPSILON) or (svcount < 3) then begin
    len := SIMD_INFINITY;
    if ( dx > cu_EPSILON) and (dx < len ) then begin
      len := dx;
    end;
    if ( dy > cu_EPSILON) and (dy < len ) then begin
      len := dy;
    end;
    if ( dz > cu_EPSILON) and (dz < len ) then begin
      len := dz;
    end;
    if len = SIMD_INFINITY then begin
      dz := 0.01; // one centimeter
      dy := 0.01;
      dz := 0.01;
    end else begin
      if ( dx < cu_EPSILON ) then begin
        dx := len * btScalar(0.05); // 1/5th the shortest non-zero edge.
      end;
      if ( dy < cu_EPSILON ) then begin
        dy := len * btScalar(0.05);
      end;
      if ( dz < cu_EPSILON ) then begin
        dz := len * btScalar(0.05);
      end;
    end;
    x1 := center[0] - dx;
    x2 := center[0] + dx;
    y1 := center[1] - dy;
    y2 := center[1] + dy;
    z1 := center[2] - dz;
    z2 := center[2] + dz;
    addPoint(vcount,vertices,x1,y1,z1);
    addPoint(vcount,vertices,x2,y1,z1);
    addPoint(vcount,vertices,x2,y2,z1);
    addPoint(vcount,vertices,x1,y2,z1);
    addPoint(vcount,vertices,x1,y1,z2);
    addPoint(vcount,vertices,x2,y1,z2);
    addPoint(vcount,vertices,x2,y2,z2);
    addPoint(vcount,vertices,x1,y2,z2);
    Result := true; // return cube
    exit;
  end else begin
    if assigned(scale) then begin
      scale^[0] := dx;
      scale^[1] := dy;
      scale^[2] := dz;
      recip[0]  := 1 / dx;
      recip[1]  := 1 / dy;
      recip[2]  := 1 / dz;
      center[0] := center[0] * recip[0];
      center[1] := center[1] * recip[1];
      center[2] := center[2] * recip[2];
    end;
  end;

  vtx := PByte(svertices);
  for i := 0 to svcount-1 do begin
    pp  := nil;
    pp  := PbtVector3(vtx);
    vtx := vtx + stride;
    px  := pp^.getX;
    py  := pp^.getY;
    pz  := pp^.getZ;
    if assigned (scale) then begin
      px := px*recip[0]; // normalize
      py := py*recip[1]; // normalize
      pz := pz*recip[2]; // normalize
    end;
    j := 0;
    while (j<vcount) do begin
      /// XXX might be broken
      v := @vertices[j];
      x := v^[0];
      y := v^[1];
      z := v^[2];
      dx := btFabs(x - px );
      dy := btFabs(y - py );
      dz := btFabs(z - pz );
      if ( dx < normalepsilon ) and ( dy < normalepsilon )  and ( dz < normalepsilon ) then begin
        // ok, it is close enough to the old one
        // now let us see if it is further from the center of the point cloud than the one we already recorded.
        // in which case we keep this one instead.
        dist1 := GetDist(px,py,pz,center);
        dist2 := GetDist(v^[0],v^[1],v^[2],center);
        if ( dist1 > dist2 ) then begin
          v^[0] := px;
          v^[1] := py;
          v^[2] := pz;
        end;
        break;
      end;
      inc(j);
    end;
    if (j = vcount) then begin
      dest := @vertices[vcount];
      dest^[0] := px;
      dest^[1] := py;
      dest^[2] := pz;
      inc(vcount);
    end;
    m_vertexIndexMapping.push_back(j);
  end;
  // ok..now make sure we didn't prune so many vertices it is now invalid.
  bmin[0] :=  SIMD_INFINITY ; bmin[1] :=  SIMD_INFINITY ; bmin[2] :=  SIMD_INFINITY;
  bmax[0] := -SIMD_INFINITY ; bmax[1] := -SIMD_INFINITY ; bmax[2] := -SIMD_INFINITY;
  for i := 0 to vcount-1 do begin
    pp := @vertices[i];
    for j:=0 to 2 do begin
      if ( pp^[j] < bmin[j] ) then begin
        bmin[j] := pp^[j];
      end;
      if ( pp^[j] > bmax[j] ) then begin
        bmax[j] := pp^[j];
      end;
    end;
  end;
  dx := bmax[0] - bmin[0];
  dy := bmax[1] - bmin[1];
  dz := bmax[2] - bmin[2];

  if ( dx < cu_EPSILON) or (dy < cu_EPSILON) or (dz < cu_EPSILON) or (vcount < 3) then begin
    cx  := dx*btScalar(0.5) + bmin[0];
    cy  := dy*btScalar(0.5) + bmin[1];
    cz  := dz*btScalar(0.5) + bmin[2];
    len := SIMD_INFINITY;
    if ( dx >= cu_EPSILON) and (dx < len ) then begin
      len := dx;
    end;
    if ( dy >= cu_EPSILON) and (dy < len ) then begin
      len := dy;
    end;
    if ( dz >= cu_EPSILON) and (dz < len ) then begin
      len := dz;
    end;
    if  len = SIMD_INFINITY then begin
      dz := 0.01; // one centimeter
      dy := 0.01;
      dz := 0.01;
    end else begin
      if ( dx < cu_EPSILON ) then begin
        dx := len * btScalar(0.05); // 1/5th the shortest non-zero edge.
      end;
      if ( dy < cu_EPSILON ) then begin
        dy := len * btScalar(0.05);
      end;
      if ( dz < cu_EPSILON ) then begin
        dz := len * btScalar(0.05);
      end;
    end;
    x1 := cx - dx;
    x2 := cx + dx;
    y1 := cy - dy;
    y2 := cy + dy;
    z1 := cz - dz;
    z2 := cz + dz;
    vcount := 0; // add box
    addPoint(vcount,vertices,x1,y1,z1);
    addPoint(vcount,vertices,x2,y1,z1);
    addPoint(vcount,vertices,x2,y2,z1);
    addPoint(vcount,vertices,x1,y2,z1);
    addPoint(vcount,vertices,x1,y1,z2);
    addPoint(vcount,vertices,x2,y1,z2);
    addPoint(vcount,vertices,x2,y2,z2);
    addPoint(vcount,vertices,x1,y2,z2);
    exit(true);
  end;
  Result := true;
end;

constructor btHullLibrary.create;
begin
  m_tris := btFOSAlignedHullTrianglePointers.create;
  m_tris.Initialize(btHullTriangle);
  m_vertexIndexMapping := TFOS_AlignedIntegers.create;
end;

destructor btHullLibrary.destroy;
begin
  m_tris.Free;
  m_vertexIndexMapping.Free;
end;

function btHullLibrary.CreateConvexHull(const desc: btHullDesc; var res: btHullResult): btHullError;
var ret           : btHullError;
    hr            : btPHullResult;
    vcount        : Cardinal;
    vertexSource  : btFOSAlignedVectorArray;
    scale         : btVector3;
    ovcount       : Cardinal;
    ok            : boolean;
    i             : cardinal;
    v             : PbtVector3;
    vertexScratch : btFOSAlignedVectorArray;
    source        : PCardinal;
    dest          : PCardinal;

    procedure ReleaseHull(var res:btPHullResult);
    begin
      if  res.m_Indices.size<>0 then begin
        res.m_Indices.clear;
      end;
      res.mVcount     := 0;
      res.mIndexCount := 0;
      res.mVertices   := nil;
    end;

    procedure _CopyVerts(const from,too : btFOSAlignedVectorArray;const count:integer);
    var i:integer;
    begin
      for i:=0 to count-1 do begin
        too.A[i]^ := from.A[i]^;
      end;
    end;
    procedure _CopyIdx(const from,too : TFOS_AlignedCardinals;const count:integer);
    var i:integer;
    begin
      for i:=0 to count-1 do begin
        too.A[i]^ := from.A[i]^;
      end;
    end;

begin
  hr := btPHullResult.create;
  ret    := QE_FAIL;
  vcount := desc.mVcount;
  if (vcount < 8) then begin
    vcount := 8;
  end;
  vertexSource := btFOSAlignedVectorArray.create;
  vertexSource.resize(vcount);
  ovcount := 0;
  ok := CleanupVertices(desc.mVcount,desc.mVertices, desc.mVertexStride, ovcount, vertexSource.A[0], desc.mNormalEpsilon, @scale); // normalize point cloud, remove duplicates!

  if ok then begin
    for i := 0 to ovcount-1 do begin
      v := vertexSource.A[i];
      v^[0] := v^[0] * scale[0];
      v^[1] := v^[1] * scale[1];
      v^[2] := v^[2] * scale[2];
    end;
    ok := ComputeHull(ovcount,vertexSource.A[0],hr,desc.mMaxVertices);
    if ok then begin
        // re-index triangle mesh so it refers to only used vertices, rebuild a new vertex table.
        vertexScratch := btFOSAlignedVectorArray.create;
        vertexScratch.resize(hr.mVcount);
        BringOutYourDead(hr , vertexScratch.A[0], ovcount );
        //for i:=0 to ovcount-1 do begin
        //  writeln('VS Idx',i,'  : ', vertexScratch[i].dumpvalues);
        //end;
        //abort;

        ret := QE_OK;
        if desc.HasHullFlag(QF_TRIANGLES)  then begin // if he wants the results as triangle!
          res                    := btHullResult.create;
          res.mPolygons          := false;
          res.mNumOutputVertices := ovcount;
          res.mNumFaces          := hr.mFaceCount;
          res.mNumIndices        := hr.mIndexCount;
          res.m_OutputVertices.resize(ovcount);
          res.m_Indices.resize(hr.mIndexCount);
          //memcpy(&result.m_OutputVertices[0], &vertexScratch[0], sizeof(btVector3)*ovcount );
          _CopyVerts(vertexScratch,res.m_OutputVertices,ovcount);
          if desc.HasHullFlag(QF_REVERSE_ORDER) then begin
            source := hr.m_Indices.A[0];
            dest   := res.m_Indices.A[0];
            for i := 0 to hr.mFaceCount-1 do begin
              dest[0] := source[2];
              dest[1] := source[1];
              dest[2] := source[0];
              dest   += 3;
              source += 3;
            end;
          end else begin
            //memcpy(&res.m_Indices[0], &hr.m_Indices[0], sizeof(unsigned int)*hr.mIndexCount);
            _CopyIdx(hr.m_indices,res.m_Indices,hr.mIndexCount);
          end;
        end else begin
          res.mPolygons          := true;
          res.mNumOutputVertices := ovcount;
          res.mNumFaces          := hr.mFaceCount;
          res.mNumIndices        := hr.mIndexCount+hr.mFaceCount;
          res.m_Indices.resize(res.mNumIndices);
          res.m_OutputVertices.resize(ovcount);
          //memcpy(&result.m_OutputVertices[0], &vertexScratch[0], sizeof(btVector3)*ovcount );
          _CopyVerts(vertexScratch,res.m_OutputVertices,ovcount);
          source := hr.m_Indices.A[0];
          dest   := res.m_Indices.A[0];
          for i :=0 to hr.mFaceCount-1 do begin
            dest[0] := 3;
            if desc.HasHullFlag(QF_REVERSE_ORDER) then begin
              dest[1] := source[2];
              dest[2] := source[1];
              dest[3] := source[0];
            end else begin
              dest[1] := source[0];
              dest[2] := source[1];
              dest[3] := source[2];
            end;
            dest   += 4;
            source += 3;
          end;
        end;
        ReleaseHull(hr);
    end;
  end;
  Result := ret;
  vertexScratch.Clear;
  vertexSource.Clear;
end;


function btHullLibrary.ReleaseResult(var res: btHullResult): btHullError;
begin
  if  res.m_OutputVertices.size<>0 then begin
    res.mNumOutputVertices:=0;
    res.m_OutputVertices.clear;
  end;
  if res.m_Indices.size<>0 then begin
    res.mNumIndices:=0;
    res.m_Indices.clear;
  end;
  m_vertexIndexMapping.Clear;
  m_tris.Clear;
  Result := QE_OK;
end;

{ int4 }

function btInt4.GetItemP(const i: integer): PInteger;
begin
 case i of
   0:result:=@x;
   1:result:=@y;
   2:result:=@z;
   3:result:=@w;
   else btAssert(false);
 end;
end;

constructor btInt4.create(const _x, _y, _z, _w: integer);
begin
  x:=_x;y:=_y;z:=_z;w:=_w;
end;

{ btInt3 }

function btInt3.GetItem(const i: integer): integer;
begin
 case i of
   0:result:=x;
   1:result:=y;
   2:result:=z;
 end;
end;

function btInt3.GetItemP(const i: integer): PInteger;
begin
 case i of
   0:result:=@x;
   1:result:=@y;
   2:result:=@z;
 end;
end;

procedure btInt3.SetItem(const i: integer; const AValue: Integer);
begin
 case i of
   0: x:=AValue;
   1: y:=AValue;
   2: z:=AValue;
 end;
end;

procedure btInt3.Initialize(const _x, _y, _z: integer);
begin
  x:=_x;y:=_y;z:=_z;
end;

{ btHullTriangle }

constructor btHullTriangle.create(const a, b, c: integer);
begin
 inherited Create;
 inherited Initialize(a,b,c);
 n:=nil;
 //n := btInt3.Create(-1,-1,-1);
 vmax := -1;
 rise := btScalar(0);
end;

destructor btHullTriangle.Destroy;
begin
  n.free;
  inherited Destroy;
end;

function btHullTriangle.neib(const a, b: integer): PInteger;
var i,i1,i2:integer;
begin
 for i:=0 to 2 do begin
   i1:=(i+1) mod 3;
   i2:=(i+2) mod 3;
   if (self[i]=a) and (self[i1]=b) then exit(n.ItemV[i2]);
   if (self[i]=b) and (self[i1]=a) then exit(n.ItemV[i2]);
 end;
 btAssert(false);
 result:=nil;
end;

{ btGeometryUtil }

function notExist(const planeEquation:btVector3;const planeEquations:btFOSAlignedVectorArray):boolean;
var i,numbrushes:integer;
    n1:PbtVector3;
begin
  numbrushes := planeEquations.Length;
  for i:=0 to numbrushes-1 do begin
    N1 := planeEquations.A[i];
    if (planeEquation.dot(N1^) > 0.999) then exit(false);
  end;
  result:=true;
end;

procedure btGeometryUtil.getPlaneEquationsFromVertices(const vertices: btFOSAlignedVectorArray;var planeEquationsOut: btFOSAlignedVectorArray);
var i,j,k,numvertices:integer;
    n1,n2,n3:PbtVector3;
    planeEquation,edge0,edge1:btVector3;
    normalSign:btScalar;
begin
 numvertices := vertices.Length;
// brute force:
  for i:=0 to numvertices-1 do begin
    n1:=vertices.A[i];
    for j:=i+1 to numvertices-1 do begin
      n2 := vertices.A[j];
      for k:=j+1 to numvertices-1 do begin
        N3 := vertices.A[k];
        edge0 := N2^-N1^;
        edge1 := N3^-N1^;
        normalSign := 1;
        planeEquation := edge0.cross(edge1) * normalSign;
        if (planeEquation.length2 > 0.0001) then begin
          planeEquation.normalize;
          if (notExist(planeEquation,planeEquationsOut)) then begin
            planeEquation[3] := -planeEquation.dot(N1^);
              //check if inside, and replace supportingVertexOut if needed
              if (btGeometryUtil.areVerticesBehindPlane(planeEquation,vertices,0.01)) then begin
                 planeEquationsOut.push_back(planeEquation);
              end;
          end;
        end;
        normalSign := -1; // FOS Normalsign loop(ww) unrolled ...
        planeEquation := edge0.cross(edge1) * normalSign;
        if (planeEquation.length2 > 0.0001) then begin
          planeEquation.normalize;
          if (notExist(planeEquation,planeEquationsOut)) then begin
            planeEquation[3] := -planeEquation.dot(N1^);
              //check if inside, and replace supportingVertexOut if needed
              if (btGeometryUtil.areVerticesBehindPlane(planeEquation,vertices,0.01)) then begin
                 planeEquationsOut.push_back(planeEquation);
              end;
          end;
        end;
      end;
    end;
  end;
end;

procedure btGeometryUtil.getVerticesFromPlaneEquations(const planeEquations: btFOSAlignedVectorArray;var verticesOut: btFOSAlignedVectorArray);
var i,j,k,numbrushes:integer;
    n1,n2,n3:PbtVector3;
    n2n3,n3n1,n1n2,potentialVertex:btVector3;
    quotient:btScalar;
begin
 numbrushes := planeEquations.Length;
 // brute force:
  for i:=0 to numbrushes-1 do begin
    N1 := planeEquations.A[i];
    for j:=i+1 to numbrushes-1 do begin
      N2 := planeEquations.A[j];
      for k:=j+1 to numbrushes-1 do begin
        N3 := planeEquations.A[k];
        n2n3 := N2^.cross(N3^);
        n3n1 := N3^.cross(N1^);
        n1n2 := N1^.cross(N2^);
        if (n2n3.length2 > 0.0001) and (n3n1.length2 > 0.0001) and ( n1n2.length2 > 0.0001) then begin
          //point P out of 3 plane equations:
          //      d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )
          //P =  -------------------------------------------------------------------------
          //   N1 . ( N2 * N3 )
          quotient := (N1^.dot(n2n3));
          if btFabs(quotient) > 0.000001 then begin
            quotient := -1 / quotient;
            n2n3 *= N1^[3];
            n3n1 *= N2^[3];
            n1n2 *= N3^[3];
            potentialVertex := n2n3;
            potentialVertex += n3n1;
            potentialVertex += n1n2;
            potentialVertex *= quotient;
            //check if inside, and replace supportingVertexOut if needed
            if (isPointInsidePlanes(planeEquations,potentialVertex,0.01)) then begin
              verticesOut.push_back(potentialVertex);
            end;
          end;
        end;
      end;
    end;
  end;
end;


function btGeometryUtil.isPointInsidePlanes(const planeEquations: btFOSAlignedVectorArray; const point: btVector3;const margin: btScalar): boolean;
var i,numbrushes:integer;
    n1:PbtVector3;
    dist:btScalar;
begin
 numbrushes := planeEquations.Length;
 for i:=0 to numbrushes-1 do begin
   n1   := planeEquations.A[i];
   dist := n1^.dot(point)+n1^[3]-margin;
   if dist>0 then exit(false);
 end;
 result:=true;
end;

function btGeometryUtil.areVerticesBehindPlane(const planeNormal: btVector3;const vertices: btFOSAlignedVectorArray; const margin: btScalar): boolean;
var i,numvertices:integer;
    n1:PbtVector3;
    dist:btScalar;
begin
 numvertices := vertices.Length;
 for i:= 0 to  numvertices-1 do begin
   n1   := vertices.A[i];
   dist := planeNormal.dot(n1^)+planeNormal[3]-margin;
   if dist>0 then exit(False);
 end;
 result:=true;
end;

{ btClock }

//function TFOS_DEFAULT_BASISTOOLS.Get_Ticks_us: qword;
//{$IFDEF MSWINDOWS}
//var Ticks: UInt64;
//begin
// QueryPerformanceCounter(Int64(Ticks));
// Result := Ticks * 1000000  div _IntFreq;
//end;
//{$ENDIF}
//{$IFDEF UNIX}
//var tz:timeval;
//    intern:qword;
//begin
//  fpgettimeofday(@tz,nil);
//  intern:=qword(tz.tv_sec)*1000*1000;
//  intern:=intern+tz.tv_usec;
//  result:=intern;
//end;
//{$ENDIF}
procedure btClock.init;
begin
{$ifdef MSWINDOWS}
  QueryPerformanceFrequency(m_data.mClockFrequency);
{$endif}
  reset;
end;

procedure btClock.copy(const other: btClock);
begin
  m_data := other.m_data;
end;

procedure btClock.reset;
begin
 {$ifdef MSWINDOWS}
   QueryPerformanceCounter(@m_data.mStartTime);
   m_data.mStartTick       := GetTickCount;
   m_data.mPrevElapsedTime := 0;
 {$else}
 {$ifdef UNIX}
   fpgettimeofday(@m_data.mStartTime, nil);
 {$else}
   abort;//implement
 {$endif}
 {$endif}
end;

function btClock.getTimeMilliseconds: Cardinal;
var
{$ifdef MSWINDOWS}
  currentTime,elapsedTime,msecTicks,elapsedTicks,msecOff: Int64;
  msecAdjustment: Int64;
{$endif}
{$ifdef Unix}
  currentTime : timeval;
{$endif}
begin
 {$ifdef MSWINDOWS}
   QueryPerformanceCounter(currentTime);
   elapsedTime:=currentTime - m_data.mStartTime;
   // Compute the number of millisecond ticks elapsed.
   msecTicks:=trunc(1000 * elapsedTime /  m_data.mClockFrequency);
  // Check for unexpected leaps in the Win32 performance counter.
  // (This is caused by unexpected data across the PCI to ISA
  // bridge, aka south bridge.  See Microsoft KB274323.)
  elapsedTicks:=GetTickCount() - m_data.mStartTick;
  msecOff:=msecTicks - elapsedTicks;
  if (msecOff < -100) or (msecOff > 100) then begin
    // Adjust the starting time forwards.
    //msecAdjustment:=btMin(trunc(msecOff * m_data.mClockFrequency / 1000), elapsedTime - m_data.mPrevElapsedTime);
    msecAdjustment:=0;
    m_data.mStartTime += msecAdjustment;
    elapsedTime -= msecAdjustment;
    // Recompute the number of millisecond ticks elapsed.
    msecTicks:=trunc(1000 * elapsedTime / m_data.mClockFrequency);
  end;
  // Store the current elapsed time for adjustments next time.
  m_data.mPrevElapsedTime:=elapsedTime;
  Result:=msecTicks;
 {$else}
 {$ifdef Unix}
    fpgettimeofday(@currentTime, nil);
    Result := round((currentTime.tv_sec - m_data.mStartTime.tv_sec) * 1000 + (currentTime.tv_usec - m_data.mStartTime.tv_usec) / 1000);
 {$endif}
 {$endif}
end;

function btClock.getTimeMicroseconds: int64;
var
{$ifdef MSWINDOWS}
  currentTime,elapsedTime,msecTicks,elapsedTicks,msecOff: Int64;
  msecAdjustment,usecTicks: Int64;
{$endif}
{$ifdef Unix}
  currentTime : timeval;
{$endif}
begin
 {$ifdef MSWINDOWS}
    QueryPerformanceCounter(currentTime);
    elapsedTime:=currentTime - m_data.mStartTime;
    // Compute the number of millisecond ticks elapsed.
    msecTicks:=trunc(1000 * elapsedTime /  m_data.mClockFrequency);
   // Check for unexpected leaps in the Win32 performance counter.
   // (This is caused by unexpected data across the PCI to ISA
   // bridge, aka south bridge.  See Microsoft KB274323.)
   elapsedTicks:=GetTickCount() - m_data.mStartTick;
   msecOff:=msecTicks - elapsedTicks;
   if (msecOff < -100) or (msecOff > 100) then begin
     // Adjust the starting time forwards.
     //msecAdjustment:=btMin(trunc(msecOff * m_data.mClockFrequency / 1000), elapsedTime - m_data.mPrevElapsedTime);
     msecAdjustment := 0;
     m_data.mStartTime += msecAdjustment;
     elapsedTime -= msecAdjustment;
     // Recompute the number of millisecond ticks elapsed.
   end;
   // Store the current elapsed time for adjustments next time.
   m_data.mPrevElapsedTime:=elapsedTime;
   usecTicks:=trunc(1000000 * elapsedTime / m_data.mClockFrequency);
   Result:=usecTicks;
 {$else}
 {$ifdef Unix}
    fpgettimeofday(@currentTime, nil);
    result := round( (currentTime.tv_sec - m_data.mStartTime.tv_sec) * 1000000 + (currentTime.tv_usec - m_data.mStartTime.tv_usec));
 {$endif}
 {$endif}
end;

{ btMatrix4x4 }


function btMatrix4x4.GetItems(i: integer): btVector4;
begin
  result:=m_el[i];
end;

function btMatrix4x4.GetItemsP(i: integer): PbtVector4;
begin
  result:=@m_el[i];
end;

procedure btMatrix4x4.SetItems(i: integer; const AValue: btVector4);
begin
  m_el[i]:=AValue;
end;

procedure btMatrix4x4.Init(const xx, xy, xz, xw, yx, yy, yz, yw, zx, zy, zz, zw, wx, wy, wz, ww: btScalar);
begin
  m_el[0].Init4(xx,xy,xz,xw);
  m_el[1].Init4(yx,yy,yz,yw);
  m_el[2].Init4(zx,zy,zz,zw);
  m_el[3].Init4(wx,wy,wz,ww);
end;

function btMatrix4x4.InitS(const xx, xy, xz, xw, yx, yy, yz, yw, zx, zy, zz, zw, wx, wy, wz, ww: btScalar): btMatrix4x4;
begin
  result.Init(xx,xy,xz,xw,yx,yy,yz,yw,zx,zy,zz,zw,wx,wy,wz,ww);
end;

function btMatrix4x4.tdotx(const v: btVector4): btScalar;
begin
  result := (m_el[0].getx * v.getx) + (m_el[1].getx * v.gety)+ (m_el[2].getx * v.getz) + (m_el[3].getx * v.getw);
end;

function btMatrix4x4.tdoty(const v: btVector4): btScalar;
begin
  result := (m_el[0].gety * v.getx) + (m_el[1].gety * v.gety)+ (m_el[2].gety * v.getz) + (m_el[3].gety * v.getw);
end;

function btMatrix4x4.tdotz(const v: btVector4): btScalar;
begin
  result := (m_el[0].getz * v.getx) + (m_el[1].getz * v.gety) + (m_el[2].getz * v.getz) + (m_el[3].getz * v.getw);
end;

function btMatrix4x4.tdotw(const v: btVector4): btScalar;
begin
  result := (m_el[0].getw * v.getx) + (m_el[1].getw * v.gety)+ (m_el[2].getw * v.getz) + (m_el[3].getw * v.getw);
end;

function btMatrix4x4.dump: string;
var i: Integer;
begin
   for i:=0 to 3 do begin
    result:=result+inttostr(i)+' : '+m_el[i].DumpValues(false)+LineEnding;
   end;
end;

procedure btMatrix4x4.setfromOpenGLMatrix(const m: PbtScalar);
begin
  m_el[0].Init4(m[0],m[4],m[8],m[12]);
  m_el[1].Init4(m[1],m[5],m[9],m[13]);
  m_el[2].Init4(m[2],m[6],m[10],m[14]);
  m_el[3].Init4(m[3],m[7],m[11],m[15]);
end;

initialization
  identityTransform.m_basis.setIdentity;
finalization

end.
