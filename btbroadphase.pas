unit btBroadphase;

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

//FILES:
// btQuantizedBvh.h
// btQuantizedBvh.cpp
// btDbvt.h
// btDbvt.cpp w/o benchmarks
// btDbvtBroadphase.h START
// btDbvtBroadphase.cpp START

interface

{$i fos_bullet.inc}

uses  btLinearMath,FOS_AlignedArray,sysutils;

type
    TbtBroadphaseNativeTypes=(
	// polyhedral convex shapes
	BOX_SHAPE_PROXYTYPE,
	TRIANGLE_SHAPE_PROXYTYPE,
	TETRAHEDRAL_SHAPE_PROXYTYPE,
	CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE,
	CONVEX_HULL_SHAPE_PROXYTYPE,
	CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE,
	CUSTOM_POLYHEDRAL_SHAPE_TYPE,
        //implicit convex shapes
        IMPLICIT_CONVEX_SHAPES_START_HERE,
	SPHERE_SHAPE_PROXYTYPE,
	MULTI_SPHERE_SHAPE_PROXYTYPE,
	CAPSULE_SHAPE_PROXYTYPE,
	CONE_SHAPE_PROXYTYPE,
	CONVEX_SHAPE_PROXYTYPE,
	CYLINDER_SHAPE_PROXYTYPE,
	UNIFORM_SCALING_SHAPE_PROXYTYPE,
	MINKOWSKI_SUM_SHAPE_PROXYTYPE,
	MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE,
	BOX_2D_SHAPE_PROXYTYPE,
	CONVEX_2D_SHAPE_PROXYTYPE,
	CUSTOM_CONVEX_SHAPE_TYPE,
        //concave shapes
        CONCAVE_SHAPES_START_HERE,
	//keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
	TRIANGLE_MESH_SHAPE_PROXYTYPE,
	SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE,
	///used for demo integration FAST/Swift collision library and Bullet
	FAST_CONCAVE_MESH_PROXYTYPE,
	//terrain
	TERRAIN_SHAPE_PROXYTYPE,
        ///Used for GIMPACT Trimesh integration
	GIMPACT_SHAPE_PROXYTYPE,
        ///Multimaterial mesh
        MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE,
	EMPTY_SHAPE_PROXYTYPE,
	STATIC_PLANE_PROXYTYPE,
	CUSTOM_CONCAVE_SHAPE_TYPE,
        CONCAVE_SHAPES_END_HERE,
	COMPOUND_SHAPE_PROXYTYPE,
	SOFTBODY_SHAPE_PROXYTYPE,
	HFFLUID_SHAPE_PROXYTYPE,
	HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE,
	INVALID_SHAPE_PROXYTYPE,
	MAX_BROADPHASE_COLLISION_TYPES);

///optional filtering to cull potential collisions
 btCollisionFilterGroups   = (btcfgDefaultFilter,btcfgStaticFilter,btcfgKinematicFilter,btcfgDebrisFilter,btcfgSensorTrigger,btcfgCharacterFilter);
 btCollisionFilterGroupSet = set of btCollisionFilterGroups;
 const btcfgAllFilter      = [btcfgDefaultFilter,btcfgStaticFilter,btcfgKinematicFilter,btcfgDebrisFilter,btcfgSensorTrigger,btcfgCharacterFilter];
       btcfgAllButStatic   = btcfgAllFilter - [btcfgStaticFilter]; //all bits sets: DefaultFilter | StaticFilter | KinematicFilter | DebrisFilter | SensorTrigger

 ///The btBroadphaseProxy is the main class that can be used with the Bullet broadphases.
 ///It stores collision shape type information, collision filter information and a client object, typically a btCollisionObject or btRigidBody.

 { btBroadphaseProxy }
type
 btBroadphaseProxy=class
     //Usually the client btCollisionObject or Rigidbody class
     m_clientObject         : Pointer;
     m_collisionFilterGroup : btCollisionFilterGroupSet;
     m_collisionFilterMask  : btCollisionFilterGroupSet;
     m_multiSapParentProxy  : Pointer;
     m_uniqueId             : integer;//m_uniqueId is introduced for paircache. could get rid of this, by calculating the address offset etc.
     m_aabbMin,m_aabbMax    : btVector3;
     function               getUid:integer;FOS_INLINE;
     constructor            create        ;
     procedure              init          (const aabbMin,aabbMax:btVector3;userPtr:Pointer;const collisionFilterGroup,collisionFilterMask:btCollisionFilterGroupSet; const multiSapParentProxy:pointer=nil);virtual; // for placement New
     class function         isPolyhedral  (const proxyType:TbtBroadphaseNativeTypes):boolean;FOS_INLINE;
     class function         isConcave     (const proxyType:TbtBroadphaseNativeTypes):boolean;FOS_INLINE;
     class function         isNonMoving   (const proxyType:TbtBroadphaseNativeTypes):boolean;FOS_INLINE;
     class function         isCompound    (const proxyType:TbtBroadphaseNativeTypes):boolean;FOS_INLINE;
     class function         isSoftBody    (const proxyType:TbtBroadphaseNativeTypes):boolean;FOS_INLINE;
     class function         isInfinite    (const proxyType:TbtBroadphaseNativeTypes):boolean;FOS_INLINE;
     class function         isConvex2D    (const proxyType:TbtBroadphaseNativeTypes):boolean;FOS_INLINE;
     class function         isConvex      (const proxyType:TbtBroadphaseNativeTypes):boolean;FOS_INLINE;
 end;

 btBroadphaseproxyArray=specialize FOS_GenericAlignedArray<btBroadphaseProxy>;


 ///The btIDebugDraw interface class allows hooking up a debug renderer to visually debug simulations.
 ///Typical use case: create a debug drawer object, and assign it to a btCollisionWorld or btDynamicsWorld using setDebugDrawer and call debugDrawWorld.
 ///A class that implements the btIDebugDraw interface has to implement the drawLine method at a minimum.
 ///For color arguments the X,Y,Z components refer to Red, Green and Blue each in the range [0..1]
  TbtIDEBUGFlags=( DBG_NoDebug,DBG_DrawWireframe,DBG_DrawAabb,DBG_DrawFeaturesText,DBG_DrawContactPoints,DBG_NoDeactivation,
                   DBG_NoHelpText,DBG_DrawText,DBG_ProfileTimings,DBG_EnableSatComparison,DBG_DisableBulletLCP,
                   DBG_EnableCCD,DBG_DrawConstraints,DBG_DrawConstraintLimits,DBG_FastWireframe,DBG_MAX_DEBUG_DRAW_MODE );

  TbtIDEBUGFlagsSet= set of TbtIDEBUGFlags;

  { btIDebugDraw }

  btIDebugDraw = class
  public
    procedure drawLine            (const  from, too, color : btVector3);virtual;abstract;
    procedure drawLine            (const  from, too, fromColor,toColor : btVector3); virtual;
    procedure drawSphere          (const  radius :btScalar; const transform : btTransform; const color : btVector3);virtual;
    procedure drawSphere          (const  p : btVector3;   const radius : btScalar; const color : btVector3);virtual;
    procedure drawTriangle        (const v0,v1,v2,n0,n1,n2,color:btVector3; const alpha : btScalar);virtual;
    procedure drawTriangle        (const v0,v1,v2,color:btVector3; const alpha : btScalar);virtual;
    procedure drawContactPoint    (const PointOnB,normalOnB : btVector3;const  distance:btScalar; const lifeTime : integer; const color : btVector3);virtual;abstract;
    procedure reportErrorWarning  (const warningString : string);virtual;abstract;
    procedure draw3dText          (const location : btVector3;const textString : string);virtual;abstract;
    procedure setDebugMode        (const debugMode:TbtIDEBUGFlagsSet);virtual;abstract;
    function  getDebugMode        :TbtIDEBUGFlagsSet;virtual;abstract;
    procedure drawAabb            (const from, too,color : btVector3);virtual;
    procedure drawTransform       (const transform:btTransform;const orthoLen:btScalar);virtual;
    procedure drawArc             (const  center, normal, axis : btVector3;const  radiusA, radiusB, minAngle, maxAngle : btScalar; const color :btVector3; const  drawSect : boolean; const stepDegrees:btScalar=10);virtual;
    procedure drawSpherePatch     (const center, up, axis:btVector3; radius,minTh,maxTh,minPs,maxPs:btScalar;const color:btVector3; const stepDegrees:btScalar=10);virtual;
    procedure drawBox             (const bbMin, bbMax, color : btVector3;const alpha : btScalar);virtual;
    procedure drawBox             (const bbMin, bbMax : btVector3; const trans : btTransform; const color:btVector3);virtual;
 end;

 ///btQuantizedBvhNode is a compressed aabb node, 16 bytes.
 ///Node can be used for leafnode or internal node. Leafnodes can point to 32-bit triangle index (non-negative range).

   { btQuantizedBvhNode }

//TODO: FOS Check Sizes !!! alignment
type
  btQuantizedBvhNode = object
    //12 bytes
    m_quantizedAabbMin : array [0..2] of WORD;
    m_quantizedAabbMax : array [0..2] of WORD;
    //4 bytes
    m_escapeIndexOrTriangleIndex : integer;
    function isLeafNode:boolean;
    function getEscapeIndex:integer;
    function getTriangleIndex:integer;
    function getPartId:integer;
  end;
  PbtQuantizedBvhNode = ^btQuantizedBvhNode;
 /// btOptimizedBvhNode contains both internal and leaf node information.
 // Total node size is 44 bytes / node. You can use the compressed version of 16 bytes.
  btOptimizedBvhNode = object
    //32 bytes
    m_aabbMinOrg,
    m_aabbMaxOrg  : btVector3;
    //4
    m_escapeIndex : integer;
    //8
    //for child nodes
    m_subPart       : integer;
    m_triangleIndex : integer;
    m_padding       : array [0..4] of integer;//bad, due to alignment
  end;
  PbtOptimizedBvhNode = ^btOptimizedBvhNode;
 ///btBvhSubtreeInfo provides info to gather a subtree of limited size

  { btBvhSubtreeInfo }

  btBvhSubtreeInfo=object
    //12 bytes
    m_quantizedAabbMin : array [0..2] of word;
    m_quantizedAabbMax : array [0..2] of word;
    //4 bytes, points to the root of the subtree
    m_rootNodeIndex    : integer;
    //4 bytes
    m_subtreeSize      : integer;
    m_padding          : array [0..2] of integer;
    procedure setAabbFromQuantizeNode(const quantizedNode:btQuantizedBvhNode);
  end;
  PbtBvhSubtreeInfo=^btBvhSubtreeInfo;

 { btNodeOverlapCallback }

 btNodeOverlapCallback = class
   procedure processNode(const subPart,triangleIndex : integer);virtual;abstract;
 end;

 ///for code readability:
 btNodeArray           = specialize FOS_GenericAlignedArray<btOptimizedBvhNode>;
 btQuantizedNodeArray  = specialize FOS_GenericAlignedArray<btQuantizedBvhNode>;
 btBvhSubtreeInfoArray = specialize FOS_GenericAlignedArray<btBvhSubtreeInfo>;

 ///The btQuantizedBvh class stores an AABB tree that can be quickly traversed on CPU and Cell SPU.
 ///It is used by the btBvhTriangleMeshShape as midphase, and by the btMultiSapBroadphase.
 ///It is recommended to use quantization for better performance and lower memory requirements.

    btQuantizedBvh=class
    public
       type
       btTraversalMode = (
         TRAVERSAL_STACKLESS = 0,
         TRAVERSAL_STACKLESS_CACHE_FRIENDLY,
         TRAVERSAL_RECURSIVE
       );
    protected
       m_bvhAabbMin,
       m_bvhAabbMax,
       m_bvhQuantization : btVector3;
       m_bulletVersion   : integer;        //for serialization versioning. It could also be used to detect endianess.
       m_curNodeIndex    : integer;
       //quantization data
       m_useQuantization          : Boolean;
       m_leafNodes                : btNodeArray;
       m_contiguousNodes          : btNodeArray;
       m_quantizedLeafNodes       : btQuantizedNodeArray;
       m_quantizedContiguousNodes : btQuantizedNodeArray;
       m_traversalMode            : btTraversalMode;
       m_SubtreeHeaders           : btBvhSubtreeInfoArray;
       //This is only used for serialization so we don't have to add serialization directly to btAlignedObjectArray
       m_subtreeHeaderCount       : integer;
       ///two versions, one for quantized and normal nodes. This allows code-reuse while maintaining readability (no template/macro!)
       ///this might be refactored into a virtual, it is usually not calculated at run-time
       procedure   setInternalNodeAabbMin       (const nodeIndex:integer;const aabbMin:btVector3);
       procedure   setInternalNodeAabbMax       (const nodeIndex:integer;const aabbMax:btVector3);
       function    getAabbMin                   (const nodeIndex:integer):btVector3;
       function    getAabbMax                   (const nodeIndex:integer):btVector3;
       procedure   setInternalNodeEscapeIndex   (const nodeIndex,escapeIndex:Integer);
       procedure   mergeInternalNodeAabb        (const nodeIndex:integer;const newAabbMin,newAabbMax : btVector3);
       procedure   swapLeafNodes                (const firstIndex,secondIndex:integer);
       procedure   assignInternalNodeFromLeafNode(const internalNode,leafNodeIndex:integer);
    protected
       procedure   buildTree                    (const startIndex,endIndex:integer);
       function    calcSplittingAxis            (const startIndex,endIndex:integer):integer;
       function    sortAndCalcSplittingIndex    (const startIndex,endIndex,splitAxis:integer):integer;
       procedure   walkStacklessTree            (const nodeCallback:btNodeOverlapCallback;const aabbMin,aabbMax:btVector3);
       procedure   walkStacklessQuantizedTreeAgainstRay (const nodeCallback:btNodeOverlapCallback;const raySource,rayTarget,aabbMin,aabbMax:btVector3;const startNodeIndex,endNodeIndex:integer);
       procedure   walkStacklessQuantizedTree           (const nodeCallback:btNodeOverlapCallback;const quantizedQueryAabbMin,quantizedQueryAabbMax:Pword;const startNodeIndex,endNodeIndex:integer);
//       procedure   walkStacklessTreeAgainstRay          (const nodeCallback:btNodeOverlapCallback;const raySource,rayTarget,aabbMin,aabbMax:btVector3;const startNodeIndex,endNodeIndex:integer);
       procedure   walkStacklessTreeAgainstRay          (const nodeCallback:btNodeOverlapCallback;const raySource,rayTarget,aabbMin,aabbMax:btVector3);
       ///tree traversal designed for small-memory processors like PS3 SPU
       procedure   walkStacklessQuantizedTreeCacheFriendly(const nodeCallback:btNodeOverlapCallback;const quantizedQueryAabbMin,quantizedQueryAabbMax:Pword);
       ///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
       procedure   walkRecursiveQuantizedTreeAgainstQueryAabb(const currentNode:PbtQuantizedBvhNode;const nodeCallback:btNodeOverlapCallback;const quantizedQueryAabbMin,quantizedQueryAabbMax:Pword);
       ///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
    //   procedure   walkRecursiveQuantizedTreeAgainstQuantizedTree(const treeNodeA,treeNodeB:btQuantizedBvhNode;const nodeCallback:btNodeOverlapCallback); FOS NO IMPL FOUND
       procedure   updateSubtreeHeaders                          (const leftChildNodexIndex,rightChildNodexIndex : Integer);
    public
       constructor create;
        ///***************************************** expert/internal use only *************************
       procedure   setQuantizationValues (const bvhAabbMin,bvhAabbMax:btVector3;const quantizationMargin:btScalar=1);
       function    getLeafNodeArray      :btQuantizedNodeArray;
       ///buildInternal is expert use only: assumes that setQuantizationValues and LeafNodeArray are initialized
       procedure   buildInternal;
       ///***************************************** expert/internal use only *************************
       procedure   reportAabbOverlappingNodex    (const nodeCallback:btNodeOverlapCallback;const aabbMin,aabbMax:btVector3);
       procedure   reportRayOverlappingNodex     (const nodeCallback:btNodeOverlapCallback;const raySource,rayTarget:btVector3);
       procedure   reportBoxCastOverlappingNodex (const nodeCallback:btNodeOverlapCallback;const raySource,rayTarget,aabbMin,aabbMax:btVector3);
       procedure   quantize                      (const lout:PWord; const point:btVector3; const  isMax:boolean);FOS_INLINE;
       procedure   quantizeWithClamp             (const lout:PWord; const point:btVector3; const  isMax:boolean);FOS_INLINE;
       function    unQuantize                    (const vecIn:Pword):btVector3;FOS_INLINE;
       ///setTraversalMode let's you choose between stackless, recursive or stackless cache friendly tree traversal. Note this is only implemented for quantized trees.
       procedure   setTraversalMode              (const traversalMode:btTraversalMode);
       function    getQuantizedNodeArray         :btQuantizedNodeArray;FOS_INLINE;
       function    getSubtreeInfoArray           :btBvhSubtreeInfoArray;FOS_INLINE;
     ////////////////////////////////////////////////////////////////////
    //        /////Calculate space needed to store BVH for serialization
    //        unsigned calculateSerializeBufferSize() const;
    //
    //        /// Data buffer MUST be 16 byte aligned
    //        virtual bool serialize(void *o_alignedDataBuffer, unsigned i_dataBufferSize, bool i_swapEndian) const;
    //
    //        ///deSerializeInPlace loads and initializes a BVH from a buffer in memory 'in place'
    //        static btQuantizedBvh *deSerializeInPlace(void *i_alignedDataBuffer, unsigned int i_dataBufferSize, bool i_swapEndian);
    //
    //        static unsigned int getAlignmentSerializationPadding();
    // //////////////////////////////////////////////////////////////////////
    //        virtual int     calculateSerializeBufferSizeNew() const;
    //        ///fills the dataBuffer and returns the struct name (and 0 on failure)
    //        virtual const char*     serialize(void* dataBuffer, btSerializer* serializer) const;
    //        virtual void deSerializeFloat(struct btQuantizedBvhFloatData& quantizedBvhFloatData);
    //        virtual void deSerializeDouble(struct btQuantizedBvhDoubleData& quantizedBvhDoubleData);
    // ////////////////////////////////////////////////////////////////////
        function  isQuantized                    :boolean;FOS_INLINE;
    // private:
    //        // Special "copy" constructor that allows for in-place deserialization
    //        // Prevents btVector3's default constructor from being called, but doesn't inialize much else
    //        // ownsMemory should most likely be false if deserializing, and if you are not, don't call this (it also changes the function signature, which we need)
    //        btQuantizedBvh(btQuantizedBvh &other, bool ownsMemory);
    //
    end;


/// btDbvt implementation by Nathanael Presson

////
//// Defaults volumes
////
//
  // btDbvtAabbMm
  btDbvtAabbMm = object
  private
    mi,mx        :btVector3;
    procedure    AddSpan(const  d :btVector3; var smi, smx : btScalar) ;inline;
  public
    function   Center           : btVector3; inline;
    function   Lengths          : btVector3; inline;
    function   Extents          : btVector3; inline;
    function   Mins             : PbtVector3; inline;
    function   Maxs             : PbtVector3; inline;
    procedure  FromCE           (const c,e: btVector3); inline;
    procedure  FromCR           (const c: btVector3 ; const r : btScalar); inline;
    procedure  FromMM           (const imiv,imxv: btVector3); inline;
    procedure  FromPoints       (const pts:PbtVector3;const n:integer); inline ;
    procedure  FromPoints       (const pts:PPbtVector3;const n:integer); inline ;
    procedure  Expand           (const e:btVector3); inline;
    procedure  SignedExpand     (const e:btVector3); inline;
    function   Contain          (const a:btDbvtAabbMm): boolean; inline;
    function   Classify         (const n:btVector3; const o:btScalar; const s:integer):integer; inline;
    function   ProjectMinimum   (const v:btVector3; const signs: cardinal):btScalar; inline;
    //function   Intersect        (const a,b:btDbvtAabbMm) : boolean ; inline;
    //function   Intersect        (const a:btDbvtAabbMm;const b:btVector3) : boolean ; inline;
    //function   Proximity        (const a,b:btDbvtAabbMm) :integer ; inline;
    //function   Select           (const o,a,b:btDbvtAabbMm) :btScalar ; inline;
    //procedure  Merge            (const a,b,r:btDbvtAabbMm) ;FOS_INLINE;
//    function   NotEqual         (const a,b:btDbvtAabbMm) :boolean ;FOS_INLINE;
  end;

  function  Intersect (const a, b: btDbvtAabbMm): boolean;inline;
  function  Intersect (const a: btDbvtAabbMm; const b: btVector3): boolean;inline;
  procedure Merge     (const a,b: btDbvtAabbMm;out r :btDbvtAabbMm);inline;


type
  btDbvtVolume = btDbvtAabbMm;
//  PbtDbvtVolume = ^btDbvtVolume;


  // btDbvtNode

  PbtDbvtNode=^dyn_btDbvtNode;

  btDBvt_nodeint=record
    case byte of
     1 : (childs    : array [0..1]  of PbtDbvtNode);
     2 : (data      : Pointer) ;
     3 : (dataAsInt : integer) ;
  end;

  dyn_btDbvtNode = object
    volume : btDbvtVolume;
    parent : PbtDbvtNode;
    int    : btDBvt_nodeint;
    function  isleaf       : boolean ;FOS_INLINE ;
    function  isinternal   : boolean ;FOS_INLINE ;
  end;


/////The btDbvt class implements a fast dynamic bounding volume tree based on axis aligned bounding boxes (aabb tree).
/////This btDbvt is used for soft body collision detection and for the btDbvtBroadphase. It has a fast insert, remove and update of nodes.
/////Unlike the btQuantizedBvh, nodes can be dynamically moved around, which allows for change in topology of the underlying data structure.


  //  Stack element

  { btDbvt_sStkNN }

  dyn_btDbvt_sStkNN=object
    a,b : PbtDbvtNode;
    procedure Init (const na,nb: PbtDbvtNode);
  end;
  PbtDbvt_sStkNN = ^dyn_btDbvt_sStkNN;

  { btDbvt_sStkNP }

  dyn_btDbvt_sStkNP=object
    node  : PbtDbvtNode;
    mask  : integer;
    procedure Init (const n: PbtDbvtNode;const m:cardinal);
  end;
  PbtDbvt_sStkNP = ^dyn_btDbvt_sStkNP;

  { btDbvt_sStkNPS }

  dyn_btDbvt_sStkNPS=object
    node  : PbtDbvtNode;
    mask  : integer;
    value : btScalar;
    procedure Init (const n: PbtDbvtNode;const m:cardinal;const v:btScalar); inline;
  end;

  PbtDbvt_sStkNPS =  ^dyn_btDbvt_sStkNPS;

  { btDbvt_sStkCLN }

  dyn_btDbvt_sStkCLN=object
    node  : PbtDbvtNode;
    parent: PbtDbvtNode;
    procedure Init (const n,p: PbtDbvtNode); inline;
  end;

  // Interfaces
  //FOSTODO -> Template version faster ?? - Pascal

  { btDbvt_ICollide }

  btDbvt_ICollide=class
    procedure  Process   (const n1,n2:PbtDbvtNode);virtual;
    procedure  Process   (const n:PbtDbvtNode);virtual;
    procedure  Process   (const n:PbtDbvtNode;const s:btScalar);virtual;
    function   Descent   (const n:PbtDbvtNode):boolean;virtual;
    function   AllLeaves (const n:PbtDbvtNode):boolean;virtual;
  end;

  btDbvt_IWriter=class
    procedure Prepare   (const root : PbtDbvtNode ; const numnodes:integer) ;virtual;abstract;
    procedure WriteNode (const n    : PbtDbvtNode ; const index,parent,child0,child1 : integer) ;virtual;abstract;
    procedure WriteLeaf (const n    : PbtDbvtNode ; const index,parent : integer) ;virtual;abstract;
  end;

  { btDbvt_IClone }

  btDbvt_IClone=class
    procedure CloneLeaf (const n:PbtDbvtNode);virtual;abstract;
  end;

const
  cbtDbvt_SIMPLE_STACKSIZE = 64;
  cbtDbvt_DOUBLE_STACKSIZE = 2 * cbtDbvt_SIMPLE_STACKSIZE;

  function btDbvtNodecompare(const a,b:PbtDbvtNode):boolean;

type
  btsStkNNArray     = specialize FOS_GenericAlignedArray<dyn_btDbvt_sStkNN>;
  btsStkNPArray     = specialize FOS_GenericAlignedArray<dyn_btDbvt_sStkNP>;
  btsStkNPSArray    = specialize FOS_GenericAlignedArray<dyn_btDbvt_sStkNPS>;
  btsStkCLNArray    = specialize FOS_GenericAlignedArray<dyn_btDbvt_sStkCLN>;
  btDbvtNodePArray  = specialize FOS_GenericAlignedArray<PbtDbvtNode>; // Needed ?
  btDbvt_SignArray  = array [0..2] of cardinal;

  btDbvt  = class
    // Fields
    m_root        : PbtDbvtNode;
    m_free        : PbtDbvtNode;
    m_lkhd        : integer;
    m_leaves      : Integer;
    m_opath       : cardinal;
    m_stkStack    : btsStkNNArray;
    tmp_stack     : btDbvtNodePArray;

    // Methods
    constructor create              ;
    destructor  Destroy             ;override;
    procedure   clear               ;
    function    empty               :boolean;
    procedure   optimizeBottomUp    ;
    procedure   optimizeTopDown     (const bu_treshold:integer=128);
    procedure   optimizeIncremental (passes:integer);
    function    insert              (const box :btDbvtVolume ; const data:Pointer):PbtDbvtNode;
    procedure   update              (const leaf:PbtDbvtNode; const lookahead:integer=-1);
    procedure   update              (const leaf:PbtDbvtNode; var   volume:btDbvtVolume);
    function    update              (const leaf:PbtDbvtNode; var   volume:btDbvtVolume; const velocity : btVector3; const margin: btScalar):boolean;
    function    update              (const leaf:PbtDbvtNode; var   volume:btDbvtVolume; const velocity : btVector3 ):boolean;
    function    update              (const leaf:PbtDbvtNode; var   volume:btDbvtVolume; const margin: btScalar):boolean;
    procedure   remove              (const leaf:PbtDbvtNode);
    procedure   write               (const iwriter : btDbvt_IWriter);
    procedure   clone               (const dest : btDbvt; const iclone:btDbvt_IClone=nil);
    class function  maxdepth        (const node : PbtDbvtNode):integer;//inline;
    class function  countLeaves     (const node : PbtDbvtNode):integer;//inline;
    class procedure extractLeaves   (const node : PbtDbvtNode;const leaves : btDbvtNodePArray);//inline;
    class procedure benchmark       ;
    class procedure enumNodes       (const root : PbtDbvtNode;const policy : btDbvt_ICollide);//inline;
    class procedure enumLeaves      (const root : PbtDbvtNode;const policy : btDbvt_ICollide);//inline;
    procedure  collideTT            (const root0,root1 : PbtDbvtNode ; const policy : btDbvt_ICollide );//inline;
    procedure  collideTTpersistentStack (const root0,root1 : PbtDbvtNode ; const policy : btDbvt_ICollide );//inline;
    procedure  collideTV            (const root : PbtDbvtNode;const vol:btDbvtVolume ; const policy : btDbvt_ICollide);//inline;
    ///rayTest is a re-entrant ray test, and can be called in parallel as long as the btAlignedAlloc is thread-safe (uses locking etc)
    ///rayTest is slower than rayTestInternal, because it builds a local stack, using memory allocations, and it recomputes signs/rayDirectionInverses each time
    class procedure rayTest         (const root : PbtDbvtNode;const rayFrom,rayTo : btVector3; const policy : btDbvt_ICollide);//inline;
      ///rayTestInternal is faster than rayTest, because it uses a persistent stack (to reduce dynamic memory allocations to a minimum) and it uses precomputed signs/rayInverseDirections
      ///rayTestInternal is used by btDbvtBroadphase to accelerate world ray casts
//    procedure  rayTestInternal      (const root : PbtDbvtNode;const rayFrom,rayTo,rayToInverse : btVector3;const signs:btDbvt_SignArray ; const lambdaMax : btScalar ; const aabbMin,aabbMax : btVector3 ; const policy : btDbvt_ICollide);//inline;
    procedure  rayTestInternal      (const root : PbtDbvtNode;const rayFrom,rayToInverse : btVector3;const signs:btDbvt_SignArray ; const lambdaMax : btScalar ; const aabbMin,aabbMax : btVector3 ; const policy : btDbvt_ICollide);//inline;
    class procedure collideKDOP     (const root : PbtDbvtNode;const normals : PbtVector3; const  offsets : PbtScalar ; const count : integer; const policy : btDbvt_ICollide);//inline;
    class procedure collideTU       (const root : PbtDbvtNode;const policy : btDbvt_ICollide);
    class procedure collideOCL      (const root : PbtDbvtNode;const normals : array of btVector3; const  offsets : array of btScalar ; const sortaxis : btVector3; const count : integer; const policy : btDbvt_ICollide ; const fullsort:boolean=true);//inline;
    // Helpers
    class function   nearest        (const i:btFOSAlignedIntegers; const a : btsStkNPSArray ; const v:btScalar; l,h : integer):integer;//inline;
    class function   allocate       (const ifree:btFOSAlignedIntegers;const stock : btsStkNPSArray ; const value : dyn_btDbvt_sStkNPS):integer;//inline;
  end;

  //
  // btDbvtProxy
  //
  btDbvtProxy = class(btBroadphaseProxy)
    // Fields
    leaf   : PbtDbvtNode;
    links  : array [0..1] of btDbvtProxy;
    stage  : integer;
    // ctor
    procedure init(const aabbMin, aabbMax: btVector3; userPtr: Pointer; const collisionFilterGroup, collisionFilterMask: btCollisionFilterGroupSet ; const multiSapParentProxy:pointer=nil ); override;
  end;

  btDbvtProxyArray = specialize FOS_GenericAlignedArray<btDbvtProxy>;

  //template <typename T>
  //static inline void	clear(T& value)
  //{
  //	static const struct ZeroDummy : T {} zerodummy;
  //	value=zerodummy;
  //}

  { gbt_list }

  generic gbt_list<T>=object
    procedure listappend (const item:T;var list:T);static;FOS_INLINE;
    procedure listremove (var item:T;var list:T);static;FOS_INLINE; // var as warning fix
    function  listcount  (root:T):integer;static;FOS_INLINE;
    procedure clear      (var item:T);
  end;

  btDbvtProxyList = specialize gbt_list<btDbvtProxy>;


implementation

{ btBroadphaseProxy }

function btBroadphaseProxy.getUid: integer;FOS_INLINE;
begin
  result:=m_uniqueId;
end;

constructor btBroadphaseProxy.create;
begin
  m_clientObject         := nil;
  m_multiSapParentProxy  := nil;
end;

procedure btBroadphaseProxy.init(const aabbMin, aabbMax: btVector3; userPtr: Pointer; const collisionFilterGroup, collisionFilterMask: btCollisionFilterGroupSet; const multiSapParentProxy: pointer);
begin
  m_clientObject         := nil;
  m_multiSapParentProxy  := multiSapParentProxy;
  m_clientObject         := userPtr;
  m_collisionFilterGroup := collisionFilterGroup;
  m_collisionFilterMask  := collisionFilterMask;
  m_aabbMin              := aabbMin;
  m_aabbMax              := aabbMax;
end;

//constructor btBroadphaseProxy.Create(const aabbMin, aabbMax: btVector3;userPtr: Pointer; const collisionFilterGroup, collisionFilterMask: btCollisionFilterGroupSet;const multiSapParentProxy: pointer);
//begin
//  init(aabbMin,aabbMax,userPtr,collisionFilterGroup,collisionFilterMask,multiSapParentProxy)
//  //m_clientObject         := nil;
//  //m_multiSapParentProxy  := nil;
//  //m_clientObject         :=userPtr;
//  //m_collisionFilterGroup := collisionFilterGroup;
//  //m_collisionFilterMask  := collisionFilterMask;
//  //m_aabbMin              :=aabbMin;
//  //m_aabbMax              :=aabbMax;
//end;


class function btBroadphaseProxy.isPolyhedral(const proxyType: TbtBroadphaseNativeTypes): boolean;FOS_INLINE;
begin
  result:=proxyType<IMPLICIT_CONVEX_SHAPES_START_HERE;
end;

class function btBroadphaseProxy.isNonMoving(const proxyType: TbtBroadphaseNativeTypes): boolean;FOS_INLINE;
begin
  Result:= btBroadphaseProxy.isConcave(proxyType) AND  (NOT(proxyType=GIMPACT_SHAPE_PROXYTYPE));
end;

class function btBroadphaseProxy.isConcave(const proxyType: TbtBroadphaseNativeTypes): boolean;FOS_INLINE;
begin
  Result:=((proxyType > CONCAVE_SHAPES_START_HERE) AND (proxyType < CONCAVE_SHAPES_END_HERE));
end;

class function btBroadphaseProxy.isCompound(const proxyType: TbtBroadphaseNativeTypes): boolean;FOS_INLINE;
begin
  Result:=proxyType = COMPOUND_SHAPE_PROXYTYPE;
end;

class function btBroadphaseProxy.isSoftBody(const proxyType: TbtBroadphaseNativeTypes): boolean;FOS_INLINE;
begin
  Result:=proxyType = SOFTBODY_SHAPE_PROXYTYPE;
end;

class function btBroadphaseProxy.isInfinite(const proxyType: TbtBroadphaseNativeTypes): boolean;FOS_INLINE;
begin
  Result := (proxyType = STATIC_PLANE_PROXYTYPE);
end;

class function btBroadphaseProxy.isConvex2D(const proxyType: TbtBroadphaseNativeTypes): boolean;FOS_INLINE;
begin
  Result := (proxyType = BOX_2D_SHAPE_PROXYTYPE) or (proxyType = CONVEX_2D_SHAPE_PROXYTYPE);
end;

class function btBroadphaseProxy.isConvex(const proxyType: TbtBroadphaseNativeTypes): boolean;FOS_INLINE;
begin
  result := (proxyType < CONCAVE_SHAPES_START_HERE);
end;




{ btQuantizedBvhNode }

function btQuantizedBvhNode.isLeafNode: boolean;
begin
  //skipindex is negative (internal node), triangleindex >=0 (leafnode)
  Result := m_escapeIndexOrTriangleIndex >= 0;
end;

function btQuantizedBvhNode.getEscapeIndex: integer;
begin
  //laz
  btAssert(not isLeafNode);
  Result:=-m_escapeIndexOrTriangleIndex;
end;

function btQuantizedBvhNode.getTriangleIndex: integer;
begin
  //laz
  btAssert(isLeafNode);
  // Get only the lower bits where the triangle index is stored
  result := (m_escapeIndexOrTriangleIndex AND NOT ((NOT 0) SHL (31-MAX_NUM_PARTS_IN_BITS)));
end;

function btQuantizedBvhNode.getPartId: integer;
begin
  //laz
  btAssert(isLeafNode);
  // Get only the highest bits where the part index is stored
  Result := SarLongint(m_escapeIndexOrTriangleIndex,(31-MAX_NUM_PARTS_IN_BITS));
end;

{ btBvhSubtreeInfo }

procedure btBvhSubtreeInfo.setAabbFromQuantizeNode(const quantizedNode: btQuantizedBvhNode);
begin
  m_quantizedAabbMin[0] := quantizedNode.m_quantizedAabbMin[0];
  m_quantizedAabbMin[1] := quantizedNode.m_quantizedAabbMin[1];
  m_quantizedAabbMin[2] := quantizedNode.m_quantizedAabbMin[2];
  m_quantizedAabbMax[0] := quantizedNode.m_quantizedAabbMax[0];
  m_quantizedAabbMax[1] := quantizedNode.m_quantizedAabbMax[1];
  m_quantizedAabbMax[2] := quantizedNode.m_quantizedAabbMax[2];
end;

{ btQuantizedBvh }

procedure btQuantizedBvh.quantize(const lout: PWord; const point: btVector3; const isMax: boolean); //FOSTODO check if round() or trunc() is appropriate !!!
var v:btVector3;
    {$ifdef DEBUG_CHECK_DEQUANTIZATION}
    newPoint:btVector3;
    {$ENDIF}
begin
  //assert helper (lazbug)
    btAssert(m_useQuantization);
    btAssert(point.getX <= m_bvhAabbMax.getX);
    btAssert(point.getY <= m_bvhAabbMax.getY);
    btAssert(point.getZ <= m_bvhAabbMax.getZ);
    btAssert(point.getX >= m_bvhAabbMin.getX);
    btAssert(point.getY >= m_bvhAabbMin.getY);
    btAssert(point.getZ >= m_bvhAabbMin.getZ);
    v := (point - m_bvhAabbMin) * m_bvhQuantization;
    ///Make sure rounding is done in a way that unQuantize(quantizeWithClamp(...)) is conservative
    ///end-points always set the first bit, so that they are sorted properly (so that neighbouring AABBs overlap properly)
    ///@todo: double-check this
    if isMax then begin
      lout[0] :=  word(round(v.getX+1)) OR 1;
      lout[1] :=  word(round(v.getY+1)) OR 1;
      lout[2] :=  word(round(v.getZ+1)) OR 1;
    end else begin
      lout[0] := round(v.getX) AND $fffe;
      lout[1] := round(v.getY) AND $fffe;
      lout[2] := round(v.getZ) AND $fffe;
    end;
{$ifdef DEBUG_CHECK_DEQUANTIZATION}
    newPoint := unQuantize(lout);
    if isMax then begin
      if (newPoint.getX() < point.getX()) then begin
        writeln(format('unconservative X, diffX = %f, oldX=%f,newX=%f\n',[newPoint.getX()-point.getX(), newPoint.getX(),point.getX()]));
      end;
      if (newPoint.getY() < point.getY()) then begin
        writeln(format('unconservative Y, diffY = %f, oldY=%f,newY=%f\n',[newPoint.getY()-point.getY(), newPoint.getY(),point.getY()]));
      end;
      if (newPoint.getZ() < point.getZ()) then begin
        writeln(format('unconservative Z, diffZ = %f, oldZ=%f,newZ=%f\n',[newPoint.getZ()-point.getZ(), newPoint.getZ(),point.getZ()]));
      end;
    end else begin
      if (newPoint.getX() > point.getX()) then begin
        writeln(format('unconservative X, diffX = %f, oldX=%f,newX=%f\n',[newPoint.getX()-point.getX(), newPoint.getX(),point.getX()]));
      end;
      if (newPoint.getY() > point.getY()) then begin
        writeln(format('unconservative Y, diffY = %f, oldY=%f,newY=%f\n',[newPoint.getY()-point.getY(), newPoint.getY(),point.getY()]));
      end;
      if (newPoint.getZ() > point.getZ()) then begin
        writeln(format('unconservative Z, diffZ = %f, oldZ=%f,newZ=%f\n',[newPoint.getZ()-point.getZ(), newPoint.getZ(),point.getZ()]));
      end;
    end;
{$endif} //DEBUG_CHECK_DEQUANTIZATION
end;

procedure btQuantizedBvh.quantizeWithClamp(const lout: PWord; const point: btVector3; const isMax: boolean);
var clampedPoint:btVector3;
begin
  //assertlaz
  btAssert(m_useQuantization);
  clampedPoint.InitVector(point);
  clampedPoint.setMax(m_bvhAabbMin);
  clampedPoint.setMin(m_bvhAabbMax);
  quantize(lout,clampedPoint,isMax);
end;

function btQuantizedBvh.unQuantize(const vecIn: Pword): btVector3;
var vecOut:btVector3;
begin
  vecOut.Init(btScalar((vecIn[0]) / (m_bvhQuantization.getX)),btScalar(vecIn[1]) / (m_bvhQuantization.getY),btScalar(vecIn[2]) / (m_bvhQuantization.getZ));
  vecOut += m_bvhAabbMin;
  Result := vecOut;
end;

procedure btQuantizedBvh.setTraversalMode(const traversalMode: btTraversalMode);
begin
  m_traversalMode := traversalMode;
end;

function btQuantizedBvh.getQuantizedNodeArray: btQuantizedNodeArray;
begin
  Result:=m_quantizedContiguousNodes;
end;

function btQuantizedBvh.getSubtreeInfoArray: btBvhSubtreeInfoArray;
begin
  Result := m_SubtreeHeaders;
end;

function btQuantizedBvh.isQuantized: boolean;
begin
  Result := m_useQuantization;
end;

procedure btQuantizedBvh.setInternalNodeAabbMin(const nodeIndex: integer; const aabbMin: btVector3);
begin
  if m_useQuantization then begin
    quantize(@m_quantizedContiguousNodes.A[nodeIndex]^.m_quantizedAabbMin[0],aabbMin,false);
  end else begin
    m_contiguousNodes.A[nodeIndex]^.m_aabbMinOrg := aabbMin;
  end;
end;

procedure btQuantizedBvh.setInternalNodeAabbMax(const nodeIndex: integer; const aabbMax: btVector3);
begin
  if m_useQuantization then begin
    quantize(@m_quantizedContiguousNodes.A[nodeIndex]^.m_quantizedAabbMax[0],aabbMax,true);
  end else begin
    m_contiguousNodes.A[nodeIndex]^.m_aabbMaxOrg := aabbMax;
  end;
end;

function btQuantizedBvh.getAabbMin(const nodeIndex: integer): btVector3;
begin
  if m_useQuantization then begin
    Result := unQuantize(@m_quantizedLeafNodes.A[nodeIndex]^.m_quantizedAabbMin[0]);
  end;
  //non-quantized
  result := m_leafNodes.A[nodeIndex]^.m_aabbMinOrg;
end;

function btQuantizedBvh.getAabbMax(const nodeIndex: integer): btVector3;
begin
  if m_useQuantization then begin
    Result := unQuantize(@m_quantizedLeafNodes.A[nodeIndex]^.m_quantizedAabbMax[0]);
  end;
  //non-quantized
  result := m_leafNodes.A[nodeIndex]^.m_aabbMaxOrg;
end;

procedure btQuantizedBvh.setInternalNodeEscapeIndex(const nodeIndex, escapeIndex: Integer);
begin
  if m_useQuantization then begin
    m_quantizedContiguousNodes.A[nodeIndex]^.m_escapeIndexOrTriangleIndex := -escapeIndex;
  end else begin
    m_contiguousNodes.A[nodeIndex]^.m_escapeIndex := escapeIndex;
  end;
end;

procedure btQuantizedBvh.mergeInternalNodeAabb(const nodeIndex: integer; const newAabbMin, newAabbMax: btVector3);
var quantizedAabbMin,quantizedAabbMax : array [0..2] of word;
  i: Integer;
begin
  if m_useQuantization then begin
    quantize(@quantizedAabbMin[0],newAabbMin,false);
    quantize(@quantizedAabbMax[0],newAabbMax,true);
    for i:=0 to 2 do begin
      if (m_quantizedContiguousNodes.A[nodeIndex]^.m_quantizedAabbMin[i] > quantizedAabbMin[i]) then begin
        m_quantizedContiguousNodes.A[nodeIndex]^.m_quantizedAabbMin[i] := quantizedAabbMin[i];
      end;
      if (m_quantizedContiguousNodes.A[nodeIndex]^.m_quantizedAabbMax[i] < quantizedAabbMax[i]) then begin
        m_quantizedContiguousNodes.A[nodeIndex]^.m_quantizedAabbMax[i] := quantizedAabbMax[i];
      end;
    end;
  end else begin
    //non-quantized
    m_contiguousNodes.A[nodeIndex]^.m_aabbMinOrg.setMin(newAabbMin);
    m_contiguousNodes.A[nodeIndex]^.m_aabbMaxOrg.setMax(newAabbMax);
  end;
end;

procedure btQuantizedBvh.swapLeafNodes(const firstIndex, secondIndex: integer);
var tmp:btQuantizedBvhNode;
    tmp2:btOptimizedBvhNode;
begin
  if m_useQuantization then begin
    tmp := m_quantizedLeafNodes.A[firstIndex]^;
    m_quantizedLeafNodes.A[firstIndex]^  := m_quantizedLeafNodes.A[secondIndex]^;
    m_quantizedLeafNodes.A[secondIndex]^ := tmp;
  end else begin
    tmp2 := m_leafNodes.A[firstIndex]^;
    m_leafNodes.A[firstIndex]^  := m_leafNodes.A[secondIndex]^;
    m_leafNodes.A[secondIndex]^ := tmp2;
  end;
end;

procedure btQuantizedBvh.assignInternalNodeFromLeafNode(const internalNode, leafNodeIndex: integer);
begin
  if m_useQuantization then begin
    m_quantizedContiguousNodes.A[internalNode]^ := m_quantizedLeafNodes.A[leafNodeIndex]^;
  end else begin
    m_contiguousNodes.A[internalNode]^ := m_leafNodes.A[leafNodeIndex]^;
  end;
end;

{$ifdef DEBUG_TREE_BUILDING}
var gStackDepth:integer = 0;
    gMaxStackDepth:integer = 0;
{$endif} //DEBUG_TREE_BUILDING

procedure btQuantizedBvh.buildTree(const startIndex, endIndex: integer);
var splitAxis, splitIndex, i,numIndices,curIndex : integer;
    internalNodeIndex: integer;
    leftChildNodexIndex: LongInt;
    rightChildNodexIndex: LongInt;
    escapeIndex: Integer;
    sizeQuantizedNode: Integer;
    treeSizeInBytes: Integer;
begin
{$ifdef DEBUG_TREE_BUILDING}
  inc(gStackDepth);
  if gStackDepth > gMaxStackDepth then gMaxStackDepth := gStackDepth;
{$endif} //DEBUG_TREE_BUILDING

  numIndices := endIndex-startIndex;
  curIndex   := m_curNodeIndex;

  btAssert(numIndices>0);
  if numIndices=1 then begin
   {$ifdef DEBUG_TREE_BUILDING}
    dec(gStackDepth);
   {$endif //DEBUG_TREE_BUILDING}
    assignInternalNodeFromLeafNode(m_curNodeIndex,startIndex);
    inc(m_curNodeIndex);
    exit;
  end;
  //calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.
  splitAxis  := calcSplittingAxis(startIndex,endIndex);
  splitIndex := sortAndCalcSplittingIndex(startIndex,endIndex,splitAxis);
  internalNodeIndex := m_curNodeIndex;
  //set the min aabb to 'inf' or a max value, and set the max aabb to a -inf/minimum value.
  //the aabb will be expanded during buildTree/mergeInternalNodeAabb with actual node values
  setInternalNodeAabbMin(m_curNodeIndex,m_bvhAabbMax);//can't use btVector3(SIMD_INFINITY,SIMD_INFINITY,SIMD_INFINITY)) because of quantization
  setInternalNodeAabbMax(m_curNodeIndex,m_bvhAabbMin);//can't use btVector3(-SIMD_INFINITY,-SIMD_INFINITY,-SIMD_INFINITY)) because of quantization
  for i:=startIndex to endIndex-1 do begin
    mergeInternalNodeAabb(m_curNodeIndex,getAabbMin(i),getAabbMax(i));
  end;
  inc(m_curNodeIndex);
  //internalNode->m_escapeIndex;
  leftChildNodexIndex := m_curNodeIndex;
  //build left child tree
  buildTree(startIndex,splitIndex);
  rightChildNodexIndex := m_curNodeIndex;
  //build right child tree
  buildTree(splitIndex,endIndex);
  {$ifdef DEBUG_TREE_BUILDING}
    dec(gStackDepth);
  {$endif //DEBUG_TREE_BUILDING}
  escapeIndex := m_curNodeIndex - curIndex;
  if m_useQuantization then begin
    //escapeIndex is the number of nodes of this subtree
    sizeQuantizedNode := sizeof(btQuantizedBvhNode);
    treeSizeInBytes   := escapeIndex * sizeQuantizedNode;
    if (treeSizeInBytes > MAX_SUBTREE_SIZE_IN_BYTES) then begin
      updateSubtreeHeaders(leftChildNodexIndex,rightChildNodexIndex);
    end;
  end else begin
  end;
  setInternalNodeEscapeIndex(internalNodeIndex,escapeIndex);
end;

function btQuantizedBvh.calcSplittingAxis(const startIndex, endIndex: integer): integer;
var i: integer;
    means,variance,diff2:btVector3;
    numIndices: Integer;
    center: btVector3;
begin
  means.InitSame(0);
  variance.InitSame(0);
  numIndices := endIndex-startIndex;
  for i:=startIndex to endIndex-1 do begin
    center := btScalar(0.5)*(getAabbMax(i)+getAabbMin(i));
    means+=center;
  end;
  means *= 1/numIndices;

  for i := startIndex to endIndex-1 do begin
   center := btScalar(0.5)*(getAabbMax(i)+getAabbMin(i));
   diff2  := center-means;
   diff2  := diff2 * diff2;
   variance += diff2;
  end;
  variance *= 1/(numIndices-1);
  Result := variance.maxAxis;
end;

function btQuantizedBvh.sortAndCalcSplittingIndex(const startIndex, endIndex, splitAxis: integer): integer;
var i,splitIndex,numIndices,rangeBalancedIndices:integer;
    splitValue:btScalar;
    means,center:btVector3;
    unbalanced: Boolean;
begin
  splitIndex := startIndex;
  numIndices := endIndex - startIndex;
  means.InitSame(0);
  for i:=startIndex to endIndex-1 do begin
    center := 0.5*(getAabbMax(i)+getAabbMin(i));
    means  +=center;
  end;
  means *= 1/numIndices;

  splitValue := means[splitAxis];

  //sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
  for i:=startIndex to endIndex-1 do begin
    center := 0.5*(getAabbMax(i)+getAabbMin(i));
    if (center[splitAxis] > splitValue) then begin
      //swap
      swapLeafNodes(i,splitIndex);
      inc(splitIndex);
    end;
  end;

  //if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
  //otherwise the tree-building might fail due to stack-overflows in certain cases.
  //unbalanced1 is unsafe: it can cause stack overflows
  //bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

  //unbalanced2 should work too: always use center (perfect balanced trees)
  //bool unbalanced2 = true;

  //this should be safe too:
  rangeBalancedIndices := numIndices div 3;
  unbalanced := ((splitIndex<=(startIndex+rangeBalancedIndices)) or (splitIndex >=(endIndex-1-rangeBalancedIndices)));

  if unbalanced then begin
    splitIndex := startIndex + SarLongint(numIndices,1);
  end;

  btAssert(not((splitIndex=startIndex) OR (splitIndex = endIndex)));
  result := splitIndex;
end;

var maxIterations:integer = 0;

procedure btQuantizedBvh.walkStacklessTree(const nodeCallback: btNodeOverlapCallback; const aabbMin, aabbMax: btVector3);
var rootNode:^btOptimizedBvhNode;
    escapeIndex, curIndex, walkIterations:integer;
    isLeafNode:boolean;
    aabbOverlap:boolean;
begin
  //laz
  btAssert(not m_useQuantization);
  rootNode    := m_contiguousNodes.A[0];
  escapeIndex := 0;
  curIndex    := 0;
  walkIterations := 0;
  //PCK: unsigned instead of bool
  while curIndex < m_curNodeIndex do begin
    //catch bugs in tree data
    btAssert (walkIterations < m_curNodeIndex);
    inc(walkIterations);
    aabbOverlap := TestAabbAgainstAabb2(aabbMin,aabbMax,rootNode^.m_aabbMinOrg,rootNode^.m_aabbMaxOrg);
    isLeafNode  := rootNode^.m_escapeIndex = -1;
    //PCK: unsigned instead of bool
    if (isLeafNode and aabbOverlap) then begin
      nodeCallback.processNode(rootNode^.m_subPart,rootNode^.m_triangleIndex);
    end;
    //PCK: unsigned instead of bool
    if (aabbOverlap or isLeafNode) then begin
      inc(rootNode);
      inc(curIndex);
    end else begin
      escapeIndex := rootNode^.m_escapeIndex;
      rootNode    += escapeIndex;
      curIndex    += escapeIndex;
    end;
  end;
  if (maxIterations < walkIterations) then begin
    maxIterations := walkIterations;
  end;
end;

procedure btQuantizedBvh.walkStacklessQuantizedTreeAgainstRay(const nodeCallback: btNodeOverlapCallback; const raySource, rayTarget, aabbMin, aabbMax: btVector3; const startNodeIndex,  endNodeIndex: integer);
var
  curIndex: LongInt;
  walkIterations: Integer;
  subTreeSize: Integer;
  rootNode:PbtQuantizedBvhNode;
  escapeIndex:integer;
  isLeafnode,boxBoxOverlap,rayBoxOverlap:boolean;
  lambda_max:btScalar;
  rayDirection: btVector3;
  rayAabbMax: btVector3;
  rayAabbMin: btVector3;
  quantizedQueryAabbMin,quantizedQueryAabbMax : array [0..2] of word;
  sign : TbtFOSSignArray;
  param: btScalar;
  bounds:TbtFOSboundsArray;
begin
  //laz
  btAssert(m_useQuantization);
  curIndex       := startNodeIndex;
  walkIterations := 0;
  subTreeSize    := endNodeIndex - startNodeIndex;
//  (void)subTreeSize;

  rootNode := m_quantizedContiguousNodes.A[startNodeIndex];
  lambda_max := 1.0;

{$ifdef RAYAABB2}
  rayDirection := (rayTarget-raySource);
  rayDirection.normalize;
  lambda_max := rayDirection.dot(rayTarget-raySource);
  ///what about division by zero? --> just set rayDirection[i] to 1.0
  rayDirection[0] := btDecide (rayDirection[0] = 0 , btScalar(BT_LARGE_FLOAT) , btScalar(1.0) / rayDirection[0]);
  rayDirection[1] := btDecide (rayDirection[1] = 0 , btScalar(BT_LARGE_FLOAT) , btScalar(1.0) / rayDirection[1]);
  rayDirection[2] := btDecide (rayDirection[2] = 0 , btScalar(BT_LARGE_FLOAT) , btScalar(1.0) / rayDirection[2]);
  sign[0] := btDecide(rayDirection[0] < 0 , 1, 0);
  sign[1] := btDecide(rayDirection[1] < 0 , 1, 0);
  sign[2] := btDecide(rayDirection[2] < 0 , 1, 0);
{$endif}

  ///* Quick pruning by quantized box */
  rayAabbMin := raySource;
  rayAabbMax := raySource;
  rayAabbMin.setMin(rayTarget);
  rayAabbMax.setMax(rayTarget);
  ///* Add box cast extents to bounding box */
  rayAabbMin += aabbMin;
  rayAabbMax += aabbMax;

  quantizeWithClamp(quantizedQueryAabbMin,rayAabbMin,false);
  quantizeWithClamp(quantizedQueryAabbMax,rayAabbMax,true);

  while (curIndex < endNodeIndex) do begin
//#define VISUALLY_ANALYZE_BVH 1
{$ifdef VISUALLY_ANALYZE_BVH}
    //some code snippet to debugDraw aabb, to visually analyze bvh structure
    static int drawPatch = 0;
    //need some global access to a debugDrawer
    extern btIDebugDraw* debugDrawerPtr;
    if (curIndex==drawPatch)
    {
            btVector3 aabbMin,aabbMax;
            aabbMin = unQuantize(rootNode->m_quantizedAabbMin);
            aabbMax = unQuantize(rootNode->m_quantizedAabbMax);
            btVector3       color(1,0,0);
            debugDrawerPtr->drawAabb(aabbMin,aabbMax,color);
    }
{$endif}//VISUALLY_ANALYZE_BVH
    //catch bugs in tree data
    btAssert (walkIterations < subTreeSize);
    inc(walkIterations);
    //PCK: unsigned instead of bool
    // only interested if this is closer than any previous hit
    param := 1.0;
    rayBoxOverlap := false;
    boxBoxOverlap := testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,rootNode^.m_quantizedAabbMin,rootNode^.m_quantizedAabbMax);
    isLeafNode    := rootNode^.isLeafNode;
    if boxBoxOverlap then begin
      bounds[0] := unQuantize(rootNode^.m_quantizedAabbMin);
      bounds[1] := unQuantize(rootNode^.m_quantizedAabbMax);
      ///* Add box cast extents */
      bounds[0] -= aabbMax;
      bounds[1] -= aabbMin;
{$if 0}
      ra2:= btRayAabb2 (raySource, rayDirection, sign, bounds, param, 0.0, lambda_max);
      ra := btRayAabb (raySource, rayTarget, bounds[0], bounds[1], param, normal);
      if (ra2 != ra) then begin
        GFRE_BT.criticalabort('functions don''t match');
      end;
{$endif}
{$ifdef RAYAABB2}
      ///careful with this check: need to check division by zero (above) and fix the unQuantize method
      ///thanks Joerg/hiker for the reproduction case!
      ///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858
      //BT_PROFILE("btRayAabb2");
      rayBoxOverlap := btRayAabb2 (raySource, rayDirection, sign, bounds, param, 0, lambda_max);
{$else}
      rayBoxOverlap := true;//btRayAabb(raySource, rayTarget, bounds[0], bounds[1], param, normal);
{$endif}
    end;
    if (isLeafNode and rayBoxOverlap) then begin
      nodeCallback.processNode(rootNode^.getPartId(),rootNode^.getTriangleIndex());
    end;
    //PCK: unsigned instead of bool
    if rayBoxOverlap or isLeafNode then begin
      inc(rootNode);
      inc(curIndex);
    end else begin
      escapeIndex := rootNode^.getEscapeIndex;
      rootNode += escapeIndex;
      curIndex += escapeIndex;
    end;
  end;
  if maxIterations < walkIterations then begin
    maxIterations := walkIterations;
  end;
end;

procedure btQuantizedBvh.walkStacklessQuantizedTree(const nodeCallback: btNodeOverlapCallback; const quantizedQueryAabbMin, quantizedQueryAabbMax: Pword; const startNodeIndex, endNodeIndex: integer);
var
  curIndex: LongInt;
  walkIterations: Integer;
  subTreeSize: Integer;
  rootNode:PbtQuantizedBvhNode;
  escapeIndex:integer;
  isLeafnode,aabbOverlap:boolean;

begin
  btAssert(m_useQuantization);
  curIndex := startNodeIndex;
  walkIterations := 0;
  subTreeSize := endNodeIndex - startNodeIndex;
  rootNode := m_quantizedContiguousNodes.A[startNodeIndex];
  while (curIndex < endNodeIndex) do begin
//#define VISUALLY_ANALYZE_BVH 1
{$ifdef VISUALLY_ANALYZE_BVH}
        //some code snippet to debugDraw aabb, to visually analyze bvh structure
        static int drawPatch = 0;
        //need some global access to a debugDrawer
        extern btIDebugDraw* debugDrawerPtr;
        if (curIndex==drawPatch)
        {
                btVector3 aabbMin,aabbMax;
                aabbMin = unQuantize(rootNode->m_quantizedAabbMin);
                aabbMax = unQuantize(rootNode->m_quantizedAabbMax);
                btVector3       color(1,0,0);
                debugDrawerPtr->drawAabb(aabbMin,aabbMax,color);
        }
{$endif}//VISUALLY_ANALYZE_BVH
    //catch bugs in tree data
    btAssert (walkIterations < subTreeSize);
    inc(walkIterations);
    //PCK: unsigned instead of bool
    aabbOverlap := testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,rootNode^.m_quantizedAabbMin,rootNode^.m_quantizedAabbMax);
    isLeafNode  := rootNode^.isLeafNode;
    if isLeafNode and aabbOverlap then begin
      nodeCallback.processNode(rootNode^.getPartId,rootNode^.getTriangleIndex);
    end;
    //PCK: unsigned instead of bool
    if aabbOverlap or isLeafNode then begin
      inc(rootNode);
      inc(curIndex);
    end else begin
      escapeIndex := rootNode^.getEscapeIndex;
      rootNode    += escapeIndex;
      curIndex    += escapeIndex;
    end;
  end;
  if (maxIterations < walkIterations) then begin
    maxIterations := walkIterations;
  end;
end;

procedure btQuantizedBvh.walkStacklessTreeAgainstRay(const nodeCallback: btNodeOverlapCallback; const raySource, rayTarget, aabbMin, aabbMax: btVector3);
var rootNode:PbtOptimizedBvhNode;
    escapeIndex, curIndex,walkIterations:integer;
    isLeafNode:boolean;
    aabbOverlap,rayBoxOverlap:boolean;
    lambda_max:btScalar;
    rayAabbMin,rayAabbMax:btVector3;
    param:btScalar;
    bounds : TbtFOSboundsArray;
    {$ifdef RAYAABB2}
    rayDir,rayDirectionInverse:btVector3;
    sign   : TbtFOSSignArray;
    {$else}
    normal : btVector3;
    {$endif}
begin
  //laz
  btAssert(not m_useQuantization);
  rootNode := m_contiguousNodes.A[0];
  escapeIndex := 0;
  curIndex    := 0;
  walkIterations := 0;
  //PCK: unsigned instead of bool
  aabbOverlap   := false;
  rayBoxOverlap := false;
  lambda_max    := 1.0;

  ///* Quick pruning by quantized box */
  rayAabbMin := raySource;
  rayAabbMax := raySource;
  rayAabbMin.setMin(rayTarget);
  rayAabbMax.setMax(rayTarget);

  ///* Add box cast extents to bounding box */
  rayAabbMin += aabbMin;
  rayAabbMax += aabbMax;

{$ifdef RAYAABB2}
  rayDir := (rayTarget-raySource);
  rayDir.normalize;
  lambda_max := rayDir.dot(rayTarget-raySource);
  ///what about division by zero? --> just set rayDirection[i] to 1.0
  rayDirectionInverse[0] := btDecide (rayDir[0] = 0 , btScalar(BT_LARGE_FLOAT) , btScalar(1.0) / rayDir[0]);
  rayDirectionInverse[1] := btDecide (rayDir[1] = 0 , btScalar(BT_LARGE_FLOAT) , btScalar(1.0) / rayDir[1]);
  rayDirectionInverse[2] := btDecide (rayDir[2] = 0 , btScalar(BT_LARGE_FLOAT) , btScalar(1.0) / rayDir[2]);
  //unsigned int sign[3] = { rayDirectionInverse[0] < 0.0, rayDirectionInverse[1] < 0.0, rayDirectionInverse[2] < 0.0};
  sign[0] := btDecide(rayDirectionInverse[0] < 0 , 1, 0);
  sign[1] := btDecide(rayDirectionInverse[1] < 0 , 1, 0);
  sign[2] := btDecide(rayDirectionInverse[2] < 0 , 1, 0);
{$endif}

  while curIndex < m_curNodeIndex do begin
    param := 1;
    //catch bugs in tree data
    btAssert (walkIterations < m_curNodeIndex);
    inc(walkIterations);
    bounds[0] := rootNode^.m_aabbMinOrg;
    bounds[1] := rootNode^.m_aabbMaxOrg;
    ///* Add box cast extents */
    bounds[0] -= aabbMax;
    bounds[1] -= aabbMin;

    aabbOverlap := TestAabbAgainstAabb2(rayAabbMin,rayAabbMax,rootNode^.m_aabbMinOrg,rootNode^.m_aabbMaxOrg);
    //perhaps profile if it is worth doing the aabbOverlap test first

  {$ifdef RAYAABB2}
    ///careful with this check: need to check division by zero (above) and fix the unQuantize method
    ///thanks Joerg/hiker for the reproduction case!
    ///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858
    rayBoxOverlap := btDecide (aabbOverlap , btRayAabb2(raySource, rayDirectionInverse, sign, bounds, param, 0, lambda_max) , false);
  {$else}
    rayBoxOverlap := btRayAabb(raySource, rayTarget,bounds[0],bounds[1],param, normal);
  {$endif}

    isLeafNode := rootNode^.m_escapeIndex = -1;

    //PCK: unsigned instead of bool / FOS -> no
    if (isLeafNode and rayBoxOverlap ) then begin
      nodeCallback.processNode(rootNode^.m_subPart,rootNode^.m_triangleIndex);
    end;
    //PCK: unsigned instead of bool / FOS -> no
    if rayBoxOverlap  or isLeafNode then begin
      inc(rootNode);
      inc(curIndex);
    end else begin
      escapeIndex := rootNode^.m_escapeIndex;
      rootNode += escapeIndex;
      curIndex += escapeIndex;
    end;
  end;
  if (maxIterations < walkIterations) then begin
    maxIterations := walkIterations;
  end;
end;

procedure btQuantizedBvh.walkStacklessQuantizedTreeCacheFriendly(const nodeCallback: btNodeOverlapCallback; const quantizedQueryAabbMin, quantizedQueryAabbMax: Pword);
var
  i: Integer;
  subtree:PbtBvhSubtreeInfo;
  overlap: Boolean;
begin
//laz
  btAssert(m_useQuantization);
  for i:=0 to m_SubtreeHeaders.Length-1 do begin
    subtree := m_SubtreeHeaders.A[i];
    //PCK: unsigned instead of bool
    overlap := testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree^.m_quantizedAabbMin,subtree^.m_quantizedAabbMax);
    if overlap then begin
      walkStacklessQuantizedTree(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax,subtree^.m_rootNodeIndex,subtree^.m_rootNodeIndex+subtree^.m_subtreeSize);
    end;
  end;
end;

procedure btQuantizedBvh.walkRecursiveQuantizedTreeAgainstQueryAabb(const currentNode: PbtQuantizedBvhNode; const nodeCallback: btNodeOverlapCallback; const quantizedQueryAabbMin,quantizedQueryAabbMax: Pword);
var aabbOverlap,isLeafNode:boolean;
    leftChildNode,rightChildNode:^btQuantizedBvhNode;
begin
  //laz
  btAssert(m_useQuantization);
  //PCK: unsigned instead of bool
  aabbOverlap := testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,currentNode^.m_quantizedAabbMin,currentNode^.m_quantizedAabbMax);
  isLeafNode  := currentNode^.isLeafNode;
  //PCK: unsigned instead of bool
  if aabbOverlap then begin
    if isLeafNode then begin
      nodeCallback.processNode(currentNode^.getPartId,currentNode^.getTriangleIndex);
    end else begin
      //process left and right children
      leftChildNode := currentNode+1;
      walkRecursiveQuantizedTreeAgainstQueryAabb(leftChildNode,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
      if leftChildNode^.isLeafNode then begin
        rightChildNode := leftChildNode+1;
      end else begin
        rightChildNode := leftChildNode+leftChildNode^.getEscapeIndex;
      end;
      walkRecursiveQuantizedTreeAgainstQueryAabb(rightChildNode,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
    end;
  end;
end;


procedure btQuantizedBvh.updateSubtreeHeaders(const leftChildNodexIndex, rightChildNodexIndex: Integer);
var leftChildNode,rightChildNode:^btQuantizedBvhNode;
    leftSubTreeSize: LongInt;
    leftSubTreeSizeInBytes: Integer;
    subtree : ^btBvhSubtreeInfo;
    rightSubTreeSize: LongInt;
    rightSubTreeSizeInBytes: Integer;
begin
  //lazassert
  btAssert(m_useQuantization);
  leftChildNode           := m_quantizedContiguousNodes.A[leftChildNodexIndex];
  leftSubTreeSize         := btDecide(leftChildNode^.isLeafNode , 1  , leftChildNode^.getEscapeIndex);
  leftSubTreeSizeInBytes  := leftSubTreeSize * sizeof(btQuantizedBvhNode);
  rightChildNode          := m_quantizedContiguousNodes.A[rightChildNodexIndex];
  rightSubTreeSize        := btDecide(rightChildNode^.isLeafNode,  1  ,rightChildNode^.getEscapeIndex);
  rightSubTreeSizeInBytes := rightSubTreeSize *  sizeof(btQuantizedBvhNode);

  if leftSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES then begin
    subtree := m_SubtreeHeaders.push_new_no_init;
    subtree^.setAabbFromQuantizeNode(leftChildNode^);
    subtree^.m_rootNodeIndex := leftChildNodexIndex;
    subtree^.m_subtreeSize   := leftSubTreeSize;
  end;
  if rightSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES then begin
    subtree := m_SubtreeHeaders.push_new_no_init;
    subtree^.setAabbFromQuantizeNode(rightChildNode^);
    subtree^.m_rootNodeIndex := rightChildNodexIndex;
    subtree^.m_subtreeSize   := rightSubTreeSize;
  end;
  //PCK: update the copy of the size
  m_subtreeHeaderCount := m_SubtreeHeaders.Length;
end;

constructor btQuantizedBvh.create;
begin
  m_bulletVersion   := cbtBULLET_VERSION;
  m_useQuantization := false;
  //m_traversalMode(TRAVERSAL_STACKLESS_CACHE_FRIENDLY)
  m_traversalMode   := TRAVERSAL_STACKLESS;
  //m_traversalMode(TRAVERSAL_RECURSIVE)
  m_subtreeHeaderCount := 0; //PCK: add this line
  m_bvhAabbMin.InitSame(-SIMD_INFINITY);
  m_bvhAabbMax.InitSame(SIMD_INFINITY);
end;

procedure btQuantizedBvh.setQuantizationValues(const bvhAabbMin, bvhAabbMax: btVector3; const quantizationMargin: btScalar);
var clampValue,aabbSize:btVector3;
begin
  //enlarge the AABB to avoid division by zero when initializing the quantization values
  clampValue.InitSame(quantizationMargin);
  m_bvhAabbMin      := bvhAabbMin - clampValue;
  m_bvhAabbMax      := bvhAabbMax + clampValue;
  aabbSize          := m_bvhAabbMax - m_bvhAabbMin;
  m_bvhQuantization := btVector3.InitSameS(65533) / aabbSize;
  m_useQuantization := true;
end;

function btQuantizedBvh.getLeafNodeArray: btQuantizedNodeArray;
begin
  result:=m_quantizedLeafNodes;
end;

procedure btQuantizedBvh.buildInternal;
var numLeafNodes:integer;
    subtree:^btBvhSubtreeInfo;
begin
  ///assumes that caller filled in the m_quantizedLeafNodes
  m_useQuantization := true;
  numLeafNodes := 0;

  //if (m_useQuantization) //FOSHMMH ?
  //{
        //now we have an array of leafnodes in m_leafNodes
        numLeafNodes := m_quantizedLeafNodes.Length;
        m_quantizedContiguousNodes.Setlength(2*numLeafNodes);
  //}

  m_curNodeIndex := 0;

  buildTree(0,numLeafNodes);

  ///if the entire tree is small then subtree size, we need to create a header info for the tree
//  if(m_useQuantization && !m_SubtreeHeaders.size()) FOSHMMH ?
  if(m_SubtreeHeaders.Length=0) then begin
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
  m_quantizedLeafNodes.Clear;
  m_leafNodes.Clear;
end;

procedure btQuantizedBvh.reportAabbOverlappingNodex(const nodeCallback: btNodeOverlapCallback; const aabbMin, aabbMax: btVector3);
var quantizedQueryAabbMin,quantizedQueryAabbMax :array [0..2] of WORD;
    rootNode:^btQuantizedBvhNode;
begin
  //either choose recursive traversal (walkTree) or stackless (walkStacklessTree)
  if m_useQuantization then begin
    ///quantize query AABB
    quantizeWithClamp(quantizedQueryAabbMin,aabbMin,false);
    quantizeWithClamp(quantizedQueryAabbMax,aabbMax,true);

    case m_traversalMode of
      TRAVERSAL_STACKLESS:                 walkStacklessQuantizedTree(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax,0,m_curNodeIndex);
      TRAVERSAL_STACKLESS_CACHE_FRIENDLY:  walkStacklessQuantizedTreeCacheFriendly(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
      TRAVERSAL_RECURSIVE: begin
        rootNode := m_quantizedContiguousNodes.A[0];
        walkRecursiveQuantizedTreeAgainstQueryAabb(rootNode,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
      end;
      else  btAssert(false);
    end;
  end else begin
    walkStacklessTree(nodeCallback,aabbMin,aabbMax);
  end;
end;

procedure btQuantizedBvh.reportRayOverlappingNodex(const nodeCallback: btNodeOverlapCallback; const raySource, rayTarget: btVector3);
begin
  reportBoxCastOverlappingNodex(nodeCallback,raySource,rayTarget,btVector3.InitSameS(0),btVector3.InitSameS(0));
end;

procedure btQuantizedBvh.reportBoxCastOverlappingNodex(const nodeCallback: btNodeOverlapCallback; const raySource, rayTarget, aabbMin, aabbMax: btVector3);
begin
  //always use stackless
  if m_useQuantization then begin
    walkStacklessQuantizedTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, m_curNodeIndex);
  end else begin
//    walkStacklessTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, m_curNodeIndex);
    walkStacklessTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax);
  end;
  {
        //recursive traversal
        btVector3 qaabbMin = raySource;
        btVector3 qaabbMax = raySource;
        qaabbMin.setMin(rayTarget);
        qaabbMax.setMax(rayTarget);
        qaabbMin += aabbMin;
        qaabbMax += aabbMax;
        reportAabbOverlappingNodex(nodeCallback,qaabbMin,qaabbMax);
  }
end;

{ btNodeOverlapCallback }



function Intersect(const a, b: btDbvtAabbMm): boolean;
begin
  {$if DBVT_INT0_IMPL=DBVT_IMPL_SSE}
  //    const __m128    rt(_mm_or_ps(   _mm_cmplt_ps(_mm_load_ps(b.mx),_mm_load_ps(a.mi)),
  //            _mm_cmplt_ps(_mm_load_ps(a.mx),_mm_load_ps(b.mi))));
  //    const __int32*  pu((const __int32*)&rt);
  //    return((pu[0]|pu[1]|pu[2])==0);
  {$else}
  result:=  (a.mi._x^<=b.mx._x^) AND
            (a.mx._x^>=b.mi._x^) AND
            (a.mi._y^<=b.mx._y^) AND
            (a.mx._y^>=b.mi._y^) AND
            (a.mi._z^<=b.mx._z^) AND
            (a.mx._z^>=b.mi._z^);
  {$endif}
end;

function Intersect(const a: btDbvtAabbMm; const b: btVector3): boolean;
begin
  abort;
  result := (b._x^>=a.mi._x^) AND
           (b._y^>=a.mi._y^) AND
           (b._z^>=a.mi._z^) AND
           (b._x^<=a.mx._x^) AND
           (b._y^<=a.mx._y^) AND
           (b._z^<=a.mx._z^);
end;


function Proximity(const a,b : btDbvtAabbMm):btScalar;FOS_INLINE;
var d:btVector3;
begin
  d      := (a.mi+a.mx)-(b.mi+b.mx);
  result :=  btFabs(d._x^)+btFabs(d._y^)+btFabs(d._z^);
end;

function Select( const o,a,b:btDbvtAabbMm):integer;FOS_INLINE;
begin
  if Proximity(o,a) < Proximity(o,b) then begin
    result := 0;
  end else begin
    Result := 1;
  end;
end;
//{
//#if  DBVT_SELECT_IMPL == DBVT_IMPL_SSE
//    static ATTRIBUTE_ALIGNED16(const unsigned __int32)      mask[]={0x7fffffff,0x7fffffff,0x7fffffff,0x7fffffff};
//    ///@todo: the intrinsic version is 11% slower
//#if DBVT_USE_INTRINSIC_SSE
//    union btSSEUnion ///NOTE: if we use more intrinsics, move btSSEUnion into the LinearMath directory
//    {
//       __m128               ssereg;
//       float                floats[4];
//       int                  ints[4];
//    };
//    __m128  omi(_mm_load_ps(o.mi));
//    omi=_mm_add_ps(omi,_mm_load_ps(o.mx));
//    __m128  ami(_mm_load_ps(a.mi));
//    ami=_mm_add_ps(ami,_mm_load_ps(a.mx));
//    ami=_mm_sub_ps(ami,omi);
//    ami=_mm_and_ps(ami,_mm_load_ps((const float*)mask));
//    __m128  bmi(_mm_load_ps(b.mi));
//    bmi=_mm_add_ps(bmi,_mm_load_ps(b.mx));
//    bmi=_mm_sub_ps(bmi,omi);
//    bmi=_mm_and_ps(bmi,_mm_load_ps((const float*)mask));
//    __m128  t0(_mm_movehl_ps(ami,ami));
//    ami=_mm_add_ps(ami,t0);
//    ami=_mm_add_ss(ami,_mm_shuffle_ps(ami,ami,1));
//    __m128 t1(_mm_movehl_ps(bmi,bmi));
//    bmi=_mm_add_ps(bmi,t1);
//    bmi=_mm_add_ss(bmi,_mm_shuffle_ps(bmi,bmi,1));
//
//    btSSEUnion tmp;
//    tmp.ssereg = _mm_cmple_ss(bmi,ami);
//    return tmp.ints[0]&1;
//
//#else
//    ATTRIBUTE_ALIGNED16(__int32     r[1]);
//    __asm
//    {
//            mov             eax,o
//                    mov             ecx,a
//                    mov             edx,b
//                    movaps  xmm0,[eax]
//            movaps  xmm5,mask
//                    addps   xmm0,[eax+16]
//            movaps  xmm1,[ecx]
//            movaps  xmm2,[edx]
//            addps   xmm1,[ecx+16]
//            addps   xmm2,[edx+16]
//            subps   xmm1,xmm0
//                    subps   xmm2,xmm0
//                    andps   xmm1,xmm5
//                    andps   xmm2,xmm5
//                    movhlps xmm3,xmm1
//                    movhlps xmm4,xmm2
//                    addps   xmm1,xmm3
//                    addps   xmm2,xmm4
//                    pshufd  xmm3,xmm1,1
//                    pshufd  xmm4,xmm2,1
//                    addss   xmm1,xmm3
//                    addss   xmm2,xmm4
//                    cmpless xmm2,xmm1
//                    movss   r,xmm2
//    }
//    return(r[0]&1);
//#endif
//#else
//    return(Proximity(o,a)<Proximity(o,b)?0:1);
//#endif
//}

procedure Merge(const a,b: btDbvtAabbMm;out r :btDbvtAabbMm);FOS_INLINE;
var i: Integer;
begin
  {$if DBVT_MERGE_IMPL=DBVT_IMPL_SSE}
  //    __m128  ami(_mm_load_ps(a.mi));
  //    __m128  amx(_mm_load_ps(a.mx));
  //    __m128  bmi(_mm_load_ps(b.mi));
  //    __m128  bmx(_mm_load_ps(b.mx));
  //    ami=_mm_min_ps(ami,bmi);
  //    amx=_mm_max_ps(amx,bmx);
  //    _mm_store_ps(r.mi,ami);
  //    _mm_store_ps(r.mx,amx);
  {$else}
  for i:=0 to 2 do begin
    if(a.mi[i]<b.mi[i]) then r.mi[i]:=a.mi[i] else r.mi[i]:=b.mi[i];
    if(a.mx[i]>b.mx[i]) then r.mx[i]:=a.mx[i] else r.mx[i]:=b.mx[i];
  end;
  {$endif}
end;

function btDbvtNodecompare(const a, b: PbtDbvtNode): boolean;
begin
  result := a=b;
end;

function NotEqual(const a, b : btDbvtAabbMm):boolean;
begin
  result:= (a.mi._x^ <> b.mi._x^) OR
           (a.mi._y^ <> b.mi._y^) OR
           (a.mi._z^ <> b.mi._z^) OR
           (a.mx._x^ <> b.mx._x^) OR
           (a.mx._y^ <> b.mx._y^) OR
           (a.mx._z^ <> b.mx._z^);
end;

procedure deletenode(const pdbvt:btDbvt ; const node:PbtDbvtNode);
begin
  Dispose(pdbvt.m_free);
  pdbvt.m_free:=node;
end;

procedure recursedeletenode(const pdbvt : btDbvt ; node : PbtDbvtNode);FOS_INLINE;
begin
  if not (node^.isleaf) then begin
    recursedeletenode(pdbvt,node^.int.childs[0]);
    recursedeletenode(pdbvt,node^.int.childs[1]);
  end;
  if (node=pdbvt.m_root) then begin
    pdbvt.m_root := nil;
  end;
  deletenode(pdbvt,node);
end;


procedure fetchleaves(const pdbvt : btDbvt ; root : PbtDbvtNode ; const leaves : btDbvtNodePArray ; const depth : integer =-1);
begin
  if root^.isinternal  and (depth<>0) then begin
    fetchleaves(pdbvt,root^.int.childs[0],leaves,depth-1);
    fetchleaves(pdbvt,root^.int.childs[1],leaves,depth-1);
    deletenode(pdbvt,root);
  end else begin
    leaves.push_back(root);
  end
end;

// volume+edge lengths
function size(const a: btDbvtVolume) :btScalar;FOS_INLINE;
var edges : btVector3;
begin
  edges  := a.Lengths;
  result := edges.x*edges.y*edges.z + edges.x+edges.y+edges.z;
end;

function merge(const a,b:btDbvtVolume) : btDbvtVolume;FOS_INLINE;
begin
  Merge(a,b,result);
end;

function createnode(const pdbvt : btDbvt ; parent : PbtDbvtNode ; const data : Pointer):PbtDbvtNode;FOS_INLINE;
var node:PbtDbvtNode;
begin
  if assigned(pdbvt.m_free) then begin
    node         := pdbvt.m_free;
    pdbvt.m_free :=nil;
  end else begin
    node  := New(PbtDbvtNode);
  end;
  node^.parent        := parent;
  node^.int.data      := data; //FOSHH attention union
  node^.int.childs[1] := nil;
  result :=  node;
end;

function createnode(const pdbvt : btDbvt ; parent : PbtDbvtNode ; const volume: btDbvtVolume ; const data : Pointer):PbtDbvtNode;FOS_INLINE;
begin
  result         := createnode(pdbvt,parent,data);
  result^.volume := volume;
end;

function createnode(const pdbvt : btDbvt ; parent : PbtDbvtNode ; const volume0,volume1: btDbvtVolume ; const data : Pointer):PbtDbvtNode;FOS_INLINE;
begin
  Result  := createnode(pdbvt,parent,data);
  Merge   (volume0,volume1,result^.volume);
end;



procedure bottomup(const pdbvt : btDbvt ; const leaves : btDbvtNodePArray);
var minsize : btScalar;
    sz      : btScalar;
    minidx  : array [0..1] of integer;
    i       : Integer;
    j       : Integer;
    p       : PbtDbvtNode;
    n       : Array [0..1] of PbtDbvtNode;
begin
  abort;
  while leaves.size>1 do begin
    minsize   := SIMD_INFINITY;
    minidx[0] := -1;
    minidx[1] := -1;
    for i := 0 to leaves.size-1 do begin
      for j:= i+1 to leaves.size-1 do begin
//        sz := size(merge(leaves.A[i]^.volume,leaves.A[j]^.volume));
        sz := size(merge(leaves.A[i]^^.volume,leaves.A[j]^^.volume));
        if (sz<minsize) then begin
          minsize   := sz;
          minidx[0] := i;
          minidx[1] := j;
        end;
      end;
    end;
    n[0]              := leaves.A[minidx[0]]^;
    n[1]              := leaves.A[minidx[1]]^;
    p                 := createnode(pdbvt,nil,n[0]^.volume,n[1]^.volume,nil);
    p^.int.childs[0]   := n[0];
    p^.int.childs[1]   := n[1];
    n[0]^.parent       := p;
    n[1]^.parent       := p;
    leaves.A[minidx[0]]^:= p;
    leaves.swap(minidx[1],leaves.size-1);
    leaves.pop_back;
  end;
end;

function bounds(const leaves: btDbvtNodePArray):btDbvtVolume;
var volume : btDbvtVolume;
    i,ni:integer;
begin
//#if DBVT_MERGE_IMPL==DBVT_IMPL_SSE
//        ATTRIBUTE_ALIGNED16(char        locals[sizeof(btDbvtVolume)]);
//        btDbvtVolume&   volume=*(btDbvtVolume*)locals;
//        volume=leaves[0]->volume;
//#else
//        btDbvtVolume volume=leaves[0]->volume;
//#endif
abort;
  volume := leaves.A[0]^^.Volume;
  i:=1;ni:=leaves.Size;
  while (i<ni) do begin
   merge(volume,leaves.A[i]^^.Volume,volume);
   inc(i);
  end;
  result:=volume;
end;

procedure split(const leaves,left,right : btDbvtNodePArray; const org,axis:btVector3 );
var i,ni:integer;
begin
  left.Resize(0);
  right.resize(0);
  i:=0;ni:=leaves.Size;
  while (i<ni) do begin
    if (btDot(axis,leaves[i]^^.volume.Center-org)<0) then begin
      left.push_back(leaves.A[i]^);
    end else begin
      right.push_back(leaves.A[i]^);
    end;
    inc(i);
  end;
end;

function indexof(const node:PbtDbvtNode):integer;FOS_INLINE;
begin
  Result := btDecide(node^.parent^.int.childs[1]=node,1,0);
end;


function  sort(n:PbtDbvtNode;var r:PbtDbvtNode):PbtDbvtNode;FOS_INLINE; // btDbvtNode* n,btDbvtNode*& r
var p     : PbtDbvtNode;
    i,j   : integer;
    s,q   : PbtDbvtNode;
    t     : btDbvtVolume;

    //procedure _swap(var a,b:btDbvtVolume);
    //begin
    //  t:=b;
    //  b:=a;
    //  a:=t;
    //end;

begin
  p:=n^.parent;
  btAssert(n^.isinternal);
  if p>n then begin //FOSTODO was p>n : !!! Check wht this means , found no overloaded operator
    i := indexof(n);
    j := 1-i;
    s := p^.int.childs[j];
    q := p^.parent;
    btAssert(n=p^.int.childs[i]);
    if assigned(q) then begin
      q^.int.childs[indexof(p)] :=n;
    end else begin
      r := n;
    end;
    s^.parent := n;
    p^.parent := n;
    n^.parent := q;
    p^.int.childs[0] := n^.int.childs[0];
    p^.int.childs[1] := n^.int.childs[1];
    n^.int.childs[0]^.parent := p;
    n^.int.childs[1]^.parent := p;
    n^.int.childs[i] := p;
    n^.int.childs[j] := s;
    //swap
    t         := p^.volume;
    p^.volume := n^.volume;
    n^.volume := t;
    result    :=  p;
    exit;
  end;
  result := n;
end;


const cbtAxisVectors : array [0..2] of btVector3 = (
                                                    (v:(m_floats:(1,0,0,0))),
                                                    (v:(m_floats:(0,1,0,0))),
                                                    (v:(m_floats:(0,0,1,0)))
                                                   );

function topdown(const pdbvt : btDbvt ; const leaves : btDbvtNodePArray ;const  bu_treshold:integer):PbtDbvtNode;
var vol           : btDbvtVolume;
    org,x         : btVector3;
    sets          : array [0..1] of btDbvtNodePArray;
    bestaxis,ni,
    dummy_idx,midp,
    bestmidp,i,j  : integer;
    splitcount    : array [0..2,0..1] of integer = ((0,0),(0,0),(0,0));
    node          : PbtDbvtNode;
begin
  sets[0] := btDbvtNodePArray.create;
  sets[1] := btDbvtNodePArray.create;
  try
    if leaves.size>1 then begin
      if leaves.size>bu_treshold then begin
        vol := bounds(leaves);
        org := vol.Center;
        bestaxis := -1;
        bestmidp := leaves.size;
        splitcount[0,0]:=0;
        for  i:=0 to leaves.size-1 do begin
          x := leaves.A[i]^^.volume.Center-org;
          for j := 0 to 2 do begin
            dummy_idx := btDecide(btDot(x,cbtAxisVectors[j])>0,1,0);
            inc(splitcount[j][dummy_idx]); // ++splitcount[j][btDot(x,axis[j])>0?1:0];
          end;
        end;
        for  i:=0 to 2 do begin
          if (splitcount[i][0]>0) and  (splitcount[i][1]>0) then begin
            midp :=  trunc(btFabs(btScalar(splitcount[i][0]-splitcount[i][1])));
            if midp<bestmidp then begin
              bestaxis := i;
              bestmidp := midp;
            end
          end;
        end;
        if bestaxis>=0 then begin
          sets[0].reserve(splitcount[bestaxis][0]);
          sets[1].reserve(splitcount[bestaxis][1]);
          split(leaves,sets[0],sets[1],org,cbtAxisVectors[bestaxis]);
        end else begin
          sets[0].reserve(leaves.size div 2+1);
          sets[1].reserve(leaves.size div 2);
          i:=0;ni:=leaves.Size;
          while (i<ni) do begin
            sets[i and 1].push_back(leaves.A[i]^);
            inc(i);
          end;
        end;
        node := createnode(pdbvt,nil,vol,nil);
        node^.int.childs[0] := topdown(pdbvt,sets[0],bu_treshold);
        node^.int.childs[1] := topdown(pdbvt,sets[1],bu_treshold);
        node^.int.childs[0]^.parent := node;
        node^.int.childs[1]^.parent := node;
        result:=node;
        exit;
      end else begin
        bottomup(pdbvt,leaves);
        Result := leaves.A[0]^;
      end;
    end;
    Result := leaves.A[0]^;
  finally
    sets[0].Free;
    sets[1].Free;
  end;
end;

procedure insertleaf(const  pdbvt:btDbvt;root, leaf:PbtDbvtNode);
var prev,node : PbtDbvtNode;
begin
  if not assigned(pdbvt.m_root) then begin
    pdbvt.m_root  := leaf;
    leaf^.parent  := nil;
  end else begin
    if not root^.isleaf then begin
      repeat
        root := root^.int.childs[Select(leaf^.volume,root^.int.childs[0]^.volume,root^.int.childs[1]^.volume)];
      until root^.isleaf;
    end;
    prev := root^.parent;
    node := createnode(pdbvt,prev,leaf^.volume,root^.volume,nil);
    if assigned(prev) then begin
      prev^.int.childs[indexof(root)] := node;
      node^.int.childs[0]             := root ; root^.parent := node;
      node^.int.childs[1]             := leaf ; leaf^.parent := node;
      repeat
        if not(prev^.volume.Contain(node^.volume)) then begin
          Merge(prev^.int.childs[0]^.volume,prev^.int.childs[1]^.volume,prev^.volume);
        end else begin
          break;
        end;
        node:=prev;
        prev:=node^.parent;
      until not assigned(prev);
    end else begin
      node^.int.childs[0] := root ; root^.parent := node;
      node^.int.childs[1] := leaf ; leaf^.parent := node;
      pdbvt.m_root       := node;
    end;
  end;
end;
//
function removeleaf(const pdbvt:btDbvt ; const  leaf:PbtDbvtNode):PbtDbvtNode;
var parent,prev,sibling:PbtDbvtNode;
    pb : btDbvtVolume;
begin
  if leaf=pdbvt.m_root then begin
    pdbvt.m_root := nil;
    Result       := nil;
  end  else begin
    parent  := leaf^.parent;
    prev    := parent^.parent;
    sibling := parent^.int.childs[1-indexof(leaf)];
    if assigned(prev) then begin
      prev^.int.childs[indexof(parent)] := sibling;
      sibling^.parent := prev;
      deletenode(pdbvt,parent);
      while assigned(prev) do begin
        pb := prev^.volume;
        Merge(prev^.int.childs[0]^.volume,prev^.int.childs[1]^.volume,prev^.volume);
        if NotEqual(pb,prev^.volume) then begin
          prev := prev^.parent;
        end else begin
          break;
        end;
      end;
      if assigned(prev) then begin
        result := prev;
      end else begin
        result := pdbvt.m_root;
      end;
      //return(prev?prev:pdbvt->m_root);
    end else begin
      pdbvt.m_root   := sibling;
      sibling^.parent := nil;
      deletenode(pdbvt,parent);
      Result := pdbvt.m_root;
    end;
  end;
end;

procedure getmaxdepth(const node:PbtDbvtNode ; const depth:integer; var maxdepth:integer);
begin
  if node^.isinternal then begin
    getmaxdepth(node^.int.childs[0],depth+1,maxdepth);
    getmaxdepth(node^.int.childs[1],depth+1,maxdepth);
  end else begin
    maxdepth := btMax(maxdepth,depth);
  end;
end;


constructor btDbvt.create;
begin
  m_root   := nil;
  m_free   := nil;
  m_lkhd   := -1;
  m_leaves := 0;
  m_opath  := 0;
  m_stkStack := btsStkNNArray.create;
  tmp_stack  := btDbvtNodePArray.create;
end;

destructor btDbvt.Destroy;
begin
  m_stkStack.Free;
  tmp_stack.Free;
  clear;
  inherited;
end;

procedure btDbvt.clear;
begin
  if assigned(m_root) then begin
    recursedeletenode(self,m_root);
  end;
  Dispose(m_Free);
  m_free := nil;
  m_lkhd := -1;
  m_stkStack.clear;
  m_opath := 0;
end;

function btDbvt.empty: boolean;
begin
  Result := nil=m_root;
end;

procedure btDbvt.optimizeBottomUp;
var leaves :btDbvtNodePArray;
begin
  leaves:=nil;abort; //rethink mm
  if assigned(m_root) then begin
    leaves.reserve(m_leaves);
    fetchleaves(self,m_root,leaves);
    bottomup(self,leaves);
    m_root:=leaves.A[0]^;
  end;
end;

procedure btDbvt.optimizeTopDown(const bu_treshold: integer);
var leaves :btDbvtNodePArray;
begin
  leaves:=nil;abort; //rethink mm
  if assigned(m_root) then begin
    leaves.reserve(m_leaves);
    fetchleaves(self,m_root,leaves);
    m_root := topdown(self,leaves,bu_treshold);
  end;
end;

procedure btDbvt.optimizeIncremental(passes: integer);
var node : PbtDbvtNode;
    bit  : cardinal;
begin
  if passes<0 then passes := m_leaves;
  if assigned(m_root) and (passes>0) then begin
    repeat
      node := m_root;
      bit  := 0;
      while node^.isinternal do begin
        node := sort(node,m_root)^.int.childs[(SarLongint(m_opath, bit)) AND 1];
        bit  := (bit+1) and (sizeof(cardinal)*8-1);
      end;
      update(node);
      inc(m_opath);
      dec(passes);
    until passes=0;
  end;
end;

procedure btDbvt.Update(const leaf: PbtDbvtNode; var volume: btDbvtVolume);
var root : PbtDbvtNode;
       i : Integer;
begin
  root := removeleaf(self,leaf);
  if assigned(root) then begin
    if (m_lkhd>=0) then begin
      i:=0;
      while(i<m_lkhd) and assigned (root^.parent) do begin
        root := root^.parent;
        inc(i);
      end;
    end else begin
      root := m_root;
    end;
  end;
  leaf^.volume := volume;
  insertleaf(self,root,leaf);
end;

function btDbvt.update(const leaf: PbtDbvtNode; var volume: btDbvtVolume; const velocity: btVector3; const margin: btScalar):boolean;
begin
  if leaf^.volume.Contain(volume) then begin
    exit(false);
  end;
  volume.Expand(btVector3.InitSameS(margin));
  volume.SignedExpand(velocity);
  update(leaf,volume);
  result:=true;
end;

function btDbvt.update(const leaf: PbtDbvtNode; var volume: btDbvtVolume; const velocity: btVector3):boolean;
begin
  if leaf^.volume.Contain(volume) then begin
    exit(false);
  end;
  volume.SignedExpand(velocity);
  update(leaf,volume);
  result:=true;
end;

function btDbvt.update(const leaf: PbtDbvtNode; var volume: btDbvtVolume; const margin: btScalar):boolean;
begin
  if leaf^.volume.Contain(volume) then begin
    exit(false);
  end;
  volume.Expand(btVector3.InitSameS(margin));
  update(leaf,volume);
  result:=true;
end;

function btDbvt.insert(const box: btDbvtVolume; const data: pointer): PbtDbvtNode;
var leaf:PbtDbvtNode;
begin
  leaf := createnode(self,nil,box,data);
  insertleaf(self,m_root,leaf);
  inc(m_leaves);
  Result := leaf;
end;

procedure btDbvt.update(const leaf: PbtDbvtNode; const lookahead: integer);
var root : PbtDbvtNode;
  i      : Integer;
begin
  root := removeleaf(self,leaf);
  if assigned(root) then begin
    if (lookahead>=0) then begin
      i:=0;
      while(i<lookahead) and assigned (root^.parent) do begin
        root := root^.parent;
        inc(i);
      end;
    end else begin
      root := m_root;
    end;
  end;
  insertleaf(self,root,leaf);
end;

procedure btDbvt.remove(const leaf: PbtDbvtNode);
begin
  removeleaf(self,leaf);
  deletenode(self,leaf);
  dec(m_leaves);
end;

type
    { btDbvtNodeEnumerator }
    btDbvtNodeEnumerator=class(btDbvt_ICollide)
      nodes:btDbvtNodePArray;
      constructor Create;
      procedure Process(const n:PbtDbvtNode);override;
    end;

constructor btDbvtNodeEnumerator.Create;
begin
//  nodes.SetComparefunctionT(@btDbvtNodecompare);
end;

{ btDbvtNodeEnumerator }
procedure btDbvtNodeEnumerator.Process(const n: PbtDbvtNode);
begin
  nodes.push_back(n);
end;


procedure btDbvt.write(const iwriter: btDbvt_IWriter);
var nodes   : btDbvtNodeEnumerator;
    n       : PbtDbvtNode;
    i       : Integer;
    p,c0,c1 : Integer;
begin
  nodes := btDbvtNodeEnumerator.Create;
  nodes.nodes.reserve(m_leaves*2);
  enumNodes(m_root,nodes);
  iwriter.Prepare(m_root,nodes.nodes.size);
  for i:=0 to nodes.nodes.size-1 do begin
    n := nodes.nodes.A[i]^;
    p := -1;
    if assigned(n^.parent) then begin
      p:=nodes.nodes.findLinearSearch(n^.parent);
      abort;
    end;
    if n^.isinternal then begin
      c0 := nodes.nodes.findLinearSearch(n^.int.childs[0]);
      c1 := nodes.nodes.findLinearSearch(n^.int.childs[1]);
      iwriter.WriteNode(n,i,p,c0,c1);
    end else begin
      iwriter.WriteLeaf(n,i,p);
    end;
  end;
end;

procedure btDbvt.clone(const dest: btDbvt; const iclone: btDbvt_IClone);
var stack   : btsStkCLNArray;
    dummy,e : dyn_btDbvt_sStkCLN;
    i       : integer;
    n       : PbtDbvtNode;
begin
  stack:=nil;abort; //rethink mm
  dest.clear;
  if assigned(m_root) then begin
    stack.reserve(m_leaves);
    dummy.Init(m_root,nil);
    stack.push_back(dummy);
    repeat
      i := stack.size-1;
      e := stack.A[i]^;
      n := createnode(dest,e.parent,e.node^.volume,e.node^.int.data);
      stack.pop_back;
      if not assigned(e.parent) then begin
        e.parent^.int.childs[i and 1] := n;
      end else begin
        dest.m_root:=n;
      end;
      if e.node^.isinternal then begin
        dummy.init(e.node^.int.childs[0],n);
        stack.push_back(dummy);
        dummy.init(e.node^.int.childs[1],n);
        stack.push_back(dummy);
      end else begin
        iclone.CloneLeaf(n);
      end;
    until stack.size=0;
  end;
end;

class function btDbvt.maxdepth(const node: pbtDbvtNode): integer;
var depth : integer;
begin
  depth := 0;
  if assigned(node) then begin
    getmaxdepth(node,1,depth);
  end;
  Result := depth;
end;

class function btDbvt.countLeaves(const node: PbtDbvtNode): integer;
begin
  if node^.isinternal then begin
    Result := countLeaves(node^.int.childs[0])+countLeaves(node^.int.childs[1]);
  end else begin
    Result := 1;
  end;
end;

class procedure btDbvt.extractLeaves(const node: PbtDbvtNode; const leaves: btDbvtNodePArray);
begin
  if node^.isinternal then begin
    extractLeaves(node^.int.childs[0],leaves);
    extractLeaves(node^.int.childs[1],leaves);
  end else begin
    leaves.push_back(node);
  end;
end;


//FOSTODO -> Test with benchmark ...
  ////
  //#if DBVT_ENABLE_BENCHMARK
  //
  //#include <stdio.h>
  //#include <stdlib.h>
  //#include "LinearMath/btQuickProf.h"
  //
  ///*
  //q6600,2.4ghz
  //
  ///Ox /Ob2 /Oi /Ot /I "." /I "..\.." /I "..\..\src" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"
  ///GF /FD /MT /GS- /Gy /arch:SSE2 /Zc:wchar_t- /Fp"..\..\out\release8\build\libbulletcollision\libbulletcollision.pch"
  ///Fo"..\..\out\release8\build\libbulletcollision\\"
  ///Fd"..\..\out\release8\build\libbulletcollision\bulletcollision.pdb"
  ///W3 /nologo /c /Wp64 /Zi /errorReport:prompt
  //
  //Benchmarking dbvt...
  //World scale: 100.000000
  //Extents base: 1.000000
  //Extents range: 4.000000
  //Leaves: 8192
  //sizeof(btDbvtVolume): 32 bytes
  //sizeof(btDbvtNode):   44 bytes
  //[1] btDbvtVolume intersections: 3499 ms (-1%)
  //[2] btDbvtVolume merges: 1934 ms (0%)
  //[3] btDbvt::collideTT: 5485 ms (-21%)
  //[4] btDbvt::collideTT self: 2814 ms (-20%)
  //[5] btDbvt::collideTT xform: 7379 ms (-1%)
  //[6] btDbvt::collideTT xform,self: 7270 ms (-2%)
  //[7] btDbvt::rayTest: 6314 ms (0%),(332143 r/s)
  //[8] insert/remove: 2093 ms (0%),(1001983 ir/s)
  //[9] updates (teleport): 1879 ms (-3%),(1116100 u/s)
  //[10] updates (jitter): 1244 ms (-4%),(1685813 u/s)
  //[11] optimize (incremental): 2514 ms (0%),(1668000 o/s)
  //[12] btDbvtVolume notequal: 3659 ms (0%)
  //[13] culling(OCL+fullsort): 2218 ms (0%),(461 t/s)
  //[14] culling(OCL+qsort): 3688 ms (5%),(2221 t/s)
  //[15] culling(KDOP+qsort): 1139 ms (-1%),(7192 t/s)
  //[16] insert/remove batch(256): 5092 ms (0%),(823704 bir/s)
  //[17] btDbvtVolume select: 3419 ms (0%)
  //*/
  //
  //struct btDbvtBenchmark
  //{
  //        struct NilPolicy : btDbvt::ICollide
  //        {
  //                NilPolicy() : m_pcount(0),m_depth(-SIMD_INFINITY),m_checksort(true)             {}
  //                void    Process(const btDbvtNode*,const btDbvtNode*)                            { ++m_pcount; }
  //                void    Process(const btDbvtNode*)                                                                      { ++m_pcount; }
  //                void    Process(const btDbvtNode*,btScalar depth)
  //                {
  //                        ++m_pcount;
  //                        if(m_checksort)
  //                        { if(depth>=m_depth) m_depth=depth; else printf("wrong depth: %f (should be >= %f)\r\n",depth,m_depth); }
  //                }
  //                int                     m_pcount;
  //                btScalar        m_depth;
  //                bool            m_checksort;
  //        };
  //        struct P14 : btDbvt::ICollide
  //        {
  //                struct Node
  //                {
  //                        const btDbvtNode*       leaf;
  //                        btScalar                        depth;
  //                };
  //                void Process(const btDbvtNode* leaf,btScalar depth)
  //                {
  //                        Node    n;
  //                        n.leaf  =       leaf;
  //                        n.depth =       depth;
  //                }
  //                static int sortfnc(const Node& a,const Node& b)
  //                {
  //                        if(a.depth<b.depth) return(+1);
  //                        if(a.depth>b.depth) return(-1);
  //                        return(0);
  //                }
  //                btAlignedObjectArray<Node>              m_nodes;
  //        };
  //        struct P15 : btDbvt::ICollide
  //        {
  //                struct Node
  //                {
  //                        const btDbvtNode*       leaf;
  //                        btScalar                        depth;
  //                };
  //                void Process(const btDbvtNode* leaf)
  //                {
  //                        Node    n;
  //                        n.leaf  =       leaf;
  //                        n.depth =       dot(leaf->volume.Center(),m_axis);
  //                }
  //                static int sortfnc(const Node& a,const Node& b)
  //                {
  //                        if(a.depth<b.depth) return(+1);
  //                        if(a.depth>b.depth) return(-1);
  //                        return(0);
  //                }
  //                btAlignedObjectArray<Node>              m_nodes;
  //                btVector3                                               m_axis;
  //        };
  //        static btScalar                 RandUnit()
  //        {
  //                return(rand()/(btScalar)RAND_MAX);
  //        }
  //        static btVector3                RandVector3()
  //        {
  //                return(btVector3(RandUnit(),RandUnit(),RandUnit()));
  //        }
  //        static btVector3                RandVector3(btScalar cs)
  //        {
  //                return(RandVector3()*cs-btVector3(cs,cs,cs)/2);
  //        }
  //        static btDbvtVolume     RandVolume(btScalar cs,btScalar eb,btScalar es)
  //        {
  //                return(btDbvtVolume::FromCE(RandVector3(cs),btVector3(eb,eb,eb)+RandVector3()*es));
  //        }
  //        static btTransform              RandTransform(btScalar cs)
  //        {
  //                btTransform     t;
  //                t.setOrigin(RandVector3(cs));
  //                t.setRotation(btQuaternion(RandUnit()*SIMD_PI*2,RandUnit()*SIMD_PI*2,RandUnit()*SIMD_PI*2).normalized());
  //                return(t);
  //        }
  //        static void                             RandTree(btScalar cs,btScalar eb,btScalar es,int leaves,btDbvt& dbvt)
  //        {
  //                dbvt.clear();
  //                for(int i=0;i<leaves;++i)
  //                {
  //                        dbvt.insert(RandVolume(cs,eb,es),0);
  //                }
  //        }
  //};


class procedure btDbvt.benchmark;
begin
  //static const btScalar cfgVolumeCenterScale            =       100;
  //static const btScalar cfgVolumeExentsBase                     =       1;
  //static const btScalar cfgVolumeExentsScale            =       4;
  //static const int              cfgLeaves                                       =       8192;
  //static const bool             cfgEnable                                       =       true;
  //
  ////[1] btDbvtVolume intersections
  //bool                                  cfgBenchmark1_Enable            =       cfgEnable;
  //static const int              cfgBenchmark1_Iterations        =       8;
  //static const int              cfgBenchmark1_Reference         =       3499;
  ////[2] btDbvtVolume merges
  //bool                                  cfgBenchmark2_Enable            =       cfgEnable;
  //static const int              cfgBenchmark2_Iterations        =       4;
  //static const int              cfgBenchmark2_Reference         =       1945;
  ////[3] btDbvt::collideTT
  //bool                                  cfgBenchmark3_Enable            =       cfgEnable;
  //static const int              cfgBenchmark3_Iterations        =       512;
  //static const int              cfgBenchmark3_Reference         =       5485;
  ////[4] btDbvt::collideTT self
  //bool                                  cfgBenchmark4_Enable            =       cfgEnable;
  //static const int              cfgBenchmark4_Iterations        =       512;
  //static const int              cfgBenchmark4_Reference         =       2814;
  ////[5] btDbvt::collideTT xform
  //bool                                  cfgBenchmark5_Enable            =       cfgEnable;
  //static const int              cfgBenchmark5_Iterations        =       512;
  //static const btScalar cfgBenchmark5_OffsetScale       =       2;
  //static const int              cfgBenchmark5_Reference         =       7379;
  ////[6] btDbvt::collideTT xform,self
  //bool                                  cfgBenchmark6_Enable            =       cfgEnable;
  //static const int              cfgBenchmark6_Iterations        =       512;
  //static const btScalar cfgBenchmark6_OffsetScale       =       2;
  //static const int              cfgBenchmark6_Reference         =       7270;
  ////[7] btDbvt::rayTest
  //bool                                  cfgBenchmark7_Enable            =       cfgEnable;
  //static const int              cfgBenchmark7_Passes            =       32;
  //static const int              cfgBenchmark7_Iterations        =       65536;
  //static const int              cfgBenchmark7_Reference         =       6307;
  ////[8] insert/remove
  //bool                                  cfgBenchmark8_Enable            =       cfgEnable;
  //static const int              cfgBenchmark8_Passes            =       32;
  //static const int              cfgBenchmark8_Iterations        =       65536;
  //static const int              cfgBenchmark8_Reference         =       2105;
  ////[9] updates (teleport)
  //bool                                  cfgBenchmark9_Enable            =       cfgEnable;
  //static const int              cfgBenchmark9_Passes            =       32;
  //static const int              cfgBenchmark9_Iterations        =       65536;
  //static const int              cfgBenchmark9_Reference         =       1879;
  ////[10] updates (jitter)
  //bool                                  cfgBenchmark10_Enable           =       cfgEnable;
  //static const btScalar cfgBenchmark10_Scale            =       cfgVolumeCenterScale/10000;
  //static const int              cfgBenchmark10_Passes           =       32;
  //static const int              cfgBenchmark10_Iterations       =       65536;
  //static const int              cfgBenchmark10_Reference        =       1244;
  ////[11] optimize (incremental)
  //bool                                  cfgBenchmark11_Enable           =       cfgEnable;
  //static const int              cfgBenchmark11_Passes           =       64;
  //static const int              cfgBenchmark11_Iterations       =       65536;
  //static const int              cfgBenchmark11_Reference        =       2510;
  ////[12] btDbvtVolume notequal
  //bool                                  cfgBenchmark12_Enable           =       cfgEnable;
  //static const int              cfgBenchmark12_Iterations       =       32;
  //static const int              cfgBenchmark12_Reference        =       3677;
  ////[13] culling(OCL+fullsort)
  //bool                                  cfgBenchmark13_Enable           =       cfgEnable;
  //static const int              cfgBenchmark13_Iterations       =       1024;
  //static const int              cfgBenchmark13_Reference        =       2231;
  ////[14] culling(OCL+qsort)
  //bool                                  cfgBenchmark14_Enable           =       cfgEnable;
  //static const int              cfgBenchmark14_Iterations       =       8192;
  //static const int              cfgBenchmark14_Reference        =       3500;
  ////[15] culling(KDOP+qsort)
  //bool                                  cfgBenchmark15_Enable           =       cfgEnable;
  //static const int              cfgBenchmark15_Iterations       =       8192;
  //static const int              cfgBenchmark15_Reference        =       1151;
  ////[16] insert/remove batch
  //bool                                  cfgBenchmark16_Enable           =       cfgEnable;
  //static const int              cfgBenchmark16_BatchCount       =       256;
  //static const int              cfgBenchmark16_Passes           =       16384;
  //static const int              cfgBenchmark16_Reference        =       5138;
  ////[17] select
  //bool                                  cfgBenchmark17_Enable           =       cfgEnable;
  //static const int              cfgBenchmark17_Iterations       =       4;
  //static const int              cfgBenchmark17_Reference        =       3390;
  //
  //btClock                                       wallclock;
  //printf("Benchmarking dbvt...\r\n");
  //printf("\tWorld scale: %f\r\n",cfgVolumeCenterScale);
  //printf("\tExtents base: %f\r\n",cfgVolumeExentsBase);
  //printf("\tExtents range: %f\r\n",cfgVolumeExentsScale);
  //printf("\tLeaves: %u\r\n",cfgLeaves);
  //printf("\tsizeof(btDbvtVolume): %u bytes\r\n",sizeof(btDbvtVolume));
  //printf("\tsizeof(btDbvtNode):   %u bytes\r\n",sizeof(btDbvtNode));
  //if(cfgBenchmark1_Enable)
  //{// Benchmark 1
  //      srand(380843);
  //      btAlignedObjectArray<btDbvtVolume>      volumes;
  //      btAlignedObjectArray<bool>                      results;
  //      volumes.resize(cfgLeaves);
  //      results.resize(cfgLeaves);
  //      for(int i=0;i<cfgLeaves;++i)
  //      {
  //              volumes[i]=btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
  //      }
  //      printf("[1] btDbvtVolume intersections: ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark1_Iterations;++i)
  //      {
  //              for(int j=0;j<cfgLeaves;++j)
  //              {
  //                      for(int k=0;k<cfgLeaves;++k)
  //                      {
  //                              results[k]=Intersect(volumes[j],volumes[k]);
  //                      }
  //              }
  //      }
  //      const int time=(int)wallclock.getTimeMilliseconds();
  //      printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark1_Reference)*100/time);
  //}
  //if(cfgBenchmark2_Enable)
  //{// Benchmark 2
  //      srand(380843);
  //      btAlignedObjectArray<btDbvtVolume>      volumes;
  //      btAlignedObjectArray<btDbvtVolume>      results;
  //      volumes.resize(cfgLeaves);
  //      results.resize(cfgLeaves);
  //      for(int i=0;i<cfgLeaves;++i)
  //      {
  //              volumes[i]=btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
  //      }
  //      printf("[2] btDbvtVolume merges: ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark2_Iterations;++i)
  //      {
  //              for(int j=0;j<cfgLeaves;++j)
  //              {
  //                      for(int k=0;k<cfgLeaves;++k)
  //                      {
  //                              Merge(volumes[j],volumes[k],results[k]);
  //                      }
  //              }
  //      }
  //      const int time=(int)wallclock.getTimeMilliseconds();
  //      printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark2_Reference)*100/time);
  //}
  //if(cfgBenchmark3_Enable)
  //{// Benchmark 3
  //      srand(380843);
  //      btDbvt                                          dbvt[2];
  //      btDbvtBenchmark::NilPolicy      policy;
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[0]);
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[1]);
  //      dbvt[0].optimizeTopDown();
  //      dbvt[1].optimizeTopDown();
  //      printf("[3] btDbvt::collideTT: ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark3_Iterations;++i)
  //      {
  //              btDbvt::collideTT(dbvt[0].m_root,dbvt[1].m_root,policy);
  //      }
  //      const int time=(int)wallclock.getTimeMilliseconds();
  //      printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark3_Reference)*100/time);
  //}
  //if(cfgBenchmark4_Enable)
  //{// Benchmark 4
  //      srand(380843);
  //      btDbvt                                          dbvt;
  //      btDbvtBenchmark::NilPolicy      policy;
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
  //      dbvt.optimizeTopDown();
  //      printf("[4] btDbvt::collideTT self: ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark4_Iterations;++i)
  //      {
  //              btDbvt::collideTT(dbvt.m_root,dbvt.m_root,policy);
  //      }
  //      const int time=(int)wallclock.getTimeMilliseconds();
  //      printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark4_Reference)*100/time);
  //}
  //if(cfgBenchmark5_Enable)
  //{// Benchmark 5
  //      srand(380843);
  //      btDbvt                                                          dbvt[2];
  //      btAlignedObjectArray<btTransform>       transforms;
  //      btDbvtBenchmark::NilPolicy                      policy;
  //      transforms.resize(cfgBenchmark5_Iterations);
  //      for(int i=0;i<transforms.size();++i)
  //      {
  //              transforms[i]=btDbvtBenchmark::RandTransform(cfgVolumeCenterScale*cfgBenchmark5_OffsetScale);
  //      }
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[0]);
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[1]);
  //      dbvt[0].optimizeTopDown();
  //      dbvt[1].optimizeTopDown();
  //      printf("[5] btDbvt::collideTT xform: ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark5_Iterations;++i)
  //      {
  //              btDbvt::collideTT(dbvt[0].m_root,dbvt[1].m_root,transforms[i],policy);
  //      }
  //      const int time=(int)wallclock.getTimeMilliseconds();
  //      printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark5_Reference)*100/time);
  //}
  //if(cfgBenchmark6_Enable)
  //{// Benchmark 6
  //      srand(380843);
  //      btDbvt                                                          dbvt;
  //      btAlignedObjectArray<btTransform>       transforms;
  //      btDbvtBenchmark::NilPolicy                      policy;
  //      transforms.resize(cfgBenchmark6_Iterations);
  //      for(int i=0;i<transforms.size();++i)
  //      {
  //              transforms[i]=btDbvtBenchmark::RandTransform(cfgVolumeCenterScale*cfgBenchmark6_OffsetScale);
  //      }
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
  //      dbvt.optimizeTopDown();
  //      printf("[6] btDbvt::collideTT xform,self: ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark6_Iterations;++i)
  //      {
  //              btDbvt::collideTT(dbvt.m_root,dbvt.m_root,transforms[i],policy);
  //      }
  //      const int time=(int)wallclock.getTimeMilliseconds();
  //      printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark6_Reference)*100/time);
  //}
  //if(cfgBenchmark7_Enable)
  //{// Benchmark 7
  //      srand(380843);
  //      btDbvt                                                          dbvt;
  //      btAlignedObjectArray<btVector3>         rayorg;
  //      btAlignedObjectArray<btVector3>         raydir;
  //      btDbvtBenchmark::NilPolicy                      policy;
  //      rayorg.resize(cfgBenchmark7_Iterations);
  //      raydir.resize(cfgBenchmark7_Iterations);
  //      for(int i=0;i<rayorg.size();++i)
  //      {
  //              rayorg[i]=btDbvtBenchmark::RandVector3(cfgVolumeCenterScale*2);
  //              raydir[i]=btDbvtBenchmark::RandVector3(cfgVolumeCenterScale*2);
  //      }
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
  //      dbvt.optimizeTopDown();
  //      printf("[7] btDbvt::rayTest: ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark7_Passes;++i)
  //      {
  //              for(int j=0;j<cfgBenchmark7_Iterations;++j)
  //              {
  //                      btDbvt::rayTest(dbvt.m_root,rayorg[j],rayorg[j]+raydir[j],policy);
  //              }
  //      }
  //      const int       time=(int)wallclock.getTimeMilliseconds();
  //      unsigned        rays=cfgBenchmark7_Passes*cfgBenchmark7_Iterations;
  //      printf("%u ms (%i%%),(%u r/s)\r\n",time,(time-cfgBenchmark7_Reference)*100/time,(rays*1000)/time);
  //}
  //if(cfgBenchmark8_Enable)
  //{// Benchmark 8
  //      srand(380843);
  //      btDbvt                                                          dbvt;
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
  //      dbvt.optimizeTopDown();
  //      printf("[8] insert/remove: ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark8_Passes;++i)
  //      {
  //              for(int j=0;j<cfgBenchmark8_Iterations;++j)
  //              {
  //                      dbvt.remove(dbvt.insert(btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale),0));
  //              }
  //      }
  //      const int       time=(int)wallclock.getTimeMilliseconds();
  //      const int       ir=cfgBenchmark8_Passes*cfgBenchmark8_Iterations;
  //      printf("%u ms (%i%%),(%u ir/s)\r\n",time,(time-cfgBenchmark8_Reference)*100/time,ir*1000/time);
  //}
  //if(cfgBenchmark9_Enable)
  //{// Benchmark 9
  //      srand(380843);
  //      btDbvt                                                                          dbvt;
  //      btAlignedObjectArray<const btDbvtNode*> leaves;
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
  //      dbvt.optimizeTopDown();
  //      dbvt.extractLeaves(dbvt.m_root,leaves);
  //      printf("[9] updates (teleport): ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark9_Passes;++i)
  //      {
  //              for(int j=0;j<cfgBenchmark9_Iterations;++j)
  //              {
  //                      dbvt.update(const_cast<btDbvtNode*>(leaves[rand()%cfgLeaves]),
  //                              btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale));
  //              }
  //      }
  //      const int       time=(int)wallclock.getTimeMilliseconds();
  //      const int       up=cfgBenchmark9_Passes*cfgBenchmark9_Iterations;
  //      printf("%u ms (%i%%),(%u u/s)\r\n",time,(time-cfgBenchmark9_Reference)*100/time,up*1000/time);
  //}
  //if(cfgBenchmark10_Enable)
  //{// Benchmark 10
  //      srand(380843);
  //      btDbvt                                                                          dbvt;
  //      btAlignedObjectArray<const btDbvtNode*> leaves;
  //      btAlignedObjectArray<btVector3>                         vectors;
  //      vectors.resize(cfgBenchmark10_Iterations);
  //      for(int i=0;i<vectors.size();++i)
  //      {
  //              vectors[i]=(btDbvtBenchmark::RandVector3()*2-btVector3(1,1,1))*cfgBenchmark10_Scale;
  //      }
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
  //      dbvt.optimizeTopDown();
  //      dbvt.extractLeaves(dbvt.m_root,leaves);
  //      printf("[10] updates (jitter): ");
  //      wallclock.reset();
  //
  //      for(int i=0;i<cfgBenchmark10_Passes;++i)
  //      {
  //              for(int j=0;j<cfgBenchmark10_Iterations;++j)
  //              {
  //                      const btVector3&        d=vectors[j];
  //                      btDbvtNode*             l=const_cast<btDbvtNode*>(leaves[rand()%cfgLeaves]);
  //                      btDbvtVolume            v=btDbvtVolume::FromMM(l->volume.Mins()+d,l->volume.Maxs()+d);
  //                      dbvt.update(l,v);
  //              }
  //      }
  //      const int       time=(int)wallclock.getTimeMilliseconds();
  //      const int       up=cfgBenchmark10_Passes*cfgBenchmark10_Iterations;
  //      printf("%u ms (%i%%),(%u u/s)\r\n",time,(time-cfgBenchmark10_Reference)*100/time,up*1000/time);
  //}
  //if(cfgBenchmark11_Enable)
  //{// Benchmark 11
  //      srand(380843);
  //      btDbvt                                                                          dbvt;
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
  //      dbvt.optimizeTopDown();
  //      printf("[11] optimize (incremental): ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark11_Passes;++i)
  //      {
  //              dbvt.optimizeIncremental(cfgBenchmark11_Iterations);
  //      }
  //      const int       time=(int)wallclock.getTimeMilliseconds();
  //      const int       op=cfgBenchmark11_Passes*cfgBenchmark11_Iterations;
  //      printf("%u ms (%i%%),(%u o/s)\r\n",time,(time-cfgBenchmark11_Reference)*100/time,op/time*1000);
  //}
  //if(cfgBenchmark12_Enable)
  //{// Benchmark 12
  //      srand(380843);
  //      btAlignedObjectArray<btDbvtVolume>      volumes;
  //      btAlignedObjectArray<bool>                              results;
  //      volumes.resize(cfgLeaves);
  //      results.resize(cfgLeaves);
  //      for(int i=0;i<cfgLeaves;++i)
  //      {
  //              volumes[i]=btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
  //      }
  //      printf("[12] btDbvtVolume notequal: ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark12_Iterations;++i)
  //      {
  //              for(int j=0;j<cfgLeaves;++j)
  //              {
  //                      for(int k=0;k<cfgLeaves;++k)
  //                      {
  //                              results[k]=NotEqual(volumes[j],volumes[k]);
  //                      }
  //              }
  //      }
  //      const int time=(int)wallclock.getTimeMilliseconds();
  //      printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark12_Reference)*100/time);
  //}
  //if(cfgBenchmark13_Enable)
  //{// Benchmark 13
  //      srand(380843);
  //      btDbvt                                                          dbvt;
  //      btAlignedObjectArray<btVector3>         vectors;
  //      btDbvtBenchmark::NilPolicy                      policy;
  //      vectors.resize(cfgBenchmark13_Iterations);
  //      for(int i=0;i<vectors.size();++i)
  //      {
  //              vectors[i]=(btDbvtBenchmark::RandVector3()*2-btVector3(1,1,1)).normalized();
  //      }
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
  //      dbvt.optimizeTopDown();
  //      printf("[13] culling(OCL+fullsort): ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark13_Iterations;++i)
  //      {
  //              static const btScalar   offset=0;
  //              policy.m_depth=-SIMD_INFINITY;
  //              dbvt.collideOCL(dbvt.m_root,&vectors[i],&offset,vectors[i],1,policy);
  //      }
  //      const int       time=(int)wallclock.getTimeMilliseconds();
  //      const int       t=cfgBenchmark13_Iterations;
  //      printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark13_Reference)*100/time,(t*1000)/time);
  //}
  //if(cfgBenchmark14_Enable)
  //{// Benchmark 14
  //      srand(380843);
  //      btDbvt                                                          dbvt;
  //      btAlignedObjectArray<btVector3>         vectors;
  //      btDbvtBenchmark::P14                            policy;
  //      vectors.resize(cfgBenchmark14_Iterations);
  //      for(int i=0;i<vectors.size();++i)
  //      {
  //              vectors[i]=(btDbvtBenchmark::RandVector3()*2-btVector3(1,1,1)).normalized();
  //      }
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
  //      dbvt.optimizeTopDown();
  //      policy.m_nodes.reserve(cfgLeaves);
  //      printf("[14] culling(OCL+qsort): ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark14_Iterations;++i)
  //      {
  //              static const btScalar   offset=0;
  //              policy.m_nodes.resize(0);
  //              dbvt.collideOCL(dbvt.m_root,&vectors[i],&offset,vectors[i],1,policy,false);
  //              policy.m_nodes.quickSort(btDbvtBenchmark::P14::sortfnc);
  //      }
  //      const int       time=(int)wallclock.getTimeMilliseconds();
  //      const int       t=cfgBenchmark14_Iterations;
  //      printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark14_Reference)*100/time,(t*1000)/time);
  //}
  //if(cfgBenchmark15_Enable)
  //{// Benchmark 15
  //      srand(380843);
  //      btDbvt                                                          dbvt;
  //      btAlignedObjectArray<btVector3>         vectors;
  //      btDbvtBenchmark::P15                            policy;
  //      vectors.resize(cfgBenchmark15_Iterations);
  //      for(int i=0;i<vectors.size();++i)
  //      {
  //              vectors[i]=(btDbvtBenchmark::RandVector3()*2-btVector3(1,1,1)).normalized();
  //      }
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
  //      dbvt.optimizeTopDown();
  //      policy.m_nodes.reserve(cfgLeaves);
  //      printf("[15] culling(KDOP+qsort): ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark15_Iterations;++i)
  //      {
  //              static const btScalar   offset=0;
  //              policy.m_nodes.resize(0);
  //              policy.m_axis=vectors[i];
  //              dbvt.collideKDOP(dbvt.m_root,&vectors[i],&offset,1,policy);
  //              policy.m_nodes.quickSort(btDbvtBenchmark::P15::sortfnc);
  //      }
  //      const int       time=(int)wallclock.getTimeMilliseconds();
  //      const int       t=cfgBenchmark15_Iterations;
  //      printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark15_Reference)*100/time,(t*1000)/time);
  //}
  //if(cfgBenchmark16_Enable)
  //{// Benchmark 16
  //      srand(380843);
  //      btDbvt                                                          dbvt;
  //      btAlignedObjectArray<btDbvtNode*>       batch;
  //      btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
  //      dbvt.optimizeTopDown();
  //      batch.reserve(cfgBenchmark16_BatchCount);
  //      printf("[16] insert/remove batch(%u): ",cfgBenchmark16_BatchCount);
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark16_Passes;++i)
  //      {
  //              for(int j=0;j<cfgBenchmark16_BatchCount;++j)
  //              {
  //                      batch.push_back(dbvt.insert(btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale),0));
  //              }
  //              for(int j=0;j<cfgBenchmark16_BatchCount;++j)
  //              {
  //                      dbvt.remove(batch[j]);
  //              }
  //              batch.resize(0);
  //      }
  //      const int       time=(int)wallclock.getTimeMilliseconds();
  //      const int       ir=cfgBenchmark16_Passes*cfgBenchmark16_BatchCount;
  //      printf("%u ms (%i%%),(%u bir/s)\r\n",time,(time-cfgBenchmark16_Reference)*100/time,int(ir*1000.0/time));
  //}
  //if(cfgBenchmark17_Enable)
  //{// Benchmark 17
  //      srand(380843);
  //      btAlignedObjectArray<btDbvtVolume>      volumes;
  //      btAlignedObjectArray<int>                       results;
  //      btAlignedObjectArray<int>                       indices;
  //      volumes.resize(cfgLeaves);
  //      results.resize(cfgLeaves);
  //      indices.resize(cfgLeaves);
  //      for(int i=0;i<cfgLeaves;++i)
  //      {
  //              indices[i]=i;
  //              volumes[i]=btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
  //      }
  //      for(int i=0;i<cfgLeaves;++i)
  //      {
  //              btSwap(indices[i],indices[rand()%cfgLeaves]);
  //      }
  //      printf("[17] btDbvtVolume select: ");
  //      wallclock.reset();
  //      for(int i=0;i<cfgBenchmark17_Iterations;++i)
  //      {
  //              for(int j=0;j<cfgLeaves;++j)
  //              {
  //                      for(int k=0;k<cfgLeaves;++k)
  //                      {
  //                              const int idx=indices[k];
  //                              results[idx]=Select(volumes[idx],volumes[j],volumes[k]);
  //                      }
  //              }
  //      }
  //      const int time=(int)wallclock.getTimeMilliseconds();
  //      printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark17_Reference)*100/time);
  //}
  //printf("\r\n\r\n");
end;

class procedure btDbvt.enumNodes(const root: PbtDbvtNode; const policy: btDbvt_ICollide);
begin
  policy.Process(root);
  if root^.isinternal then begin
    enumNodes(root^.int.childs[0],policy);
    enumNodes(root^.int.childs[1],policy);
  end;
end;

class procedure btDbvt.enumLeaves(const root: PbtDbvtNode; const policy: btDbvt_ICollide);
begin
  if root^.isinternal then begin
    enumLeaves(root^.int.childs[0],policy);
    enumLeaves(root^.int.childs[1],policy);
  end else begin
    policy.Process(root);
  end;
end;

procedure btDbvt.collideTT(const root0, root1: PbtDbvtNode; const policy: btDbvt_ICollide);
var depth    : integer;
    treshold : integer;
    stkStack : btsStkNNArray;
    p        : PbtDbvt_sStkNN;
begin
  stkStack:=nil;abort; //rethink mm
  if assigned(root0) and assigned(root1) then begin
    depth:=1;
    treshold:=cbtDbvt_DOUBLE_STACKSIZE-4;
    stkStack.Setlength(cbtDbvt_DOUBLE_STACKSIZE);
    stkStack.A[0]^.Init(root0,root1);
    repeat
      dec(depth);
      p := stkStack.A[depth];
      if depth>treshold then begin
        stkStack.Setlength(stkStack.size*2);
        treshold:=stkStack.size-4;
      end;
      if p^.a=p^.b then begin
        if p^.a^.isinternal then begin
          stkStack.A[depth]^.init(p^.a^.int.childs[0],p^.a^.int.childs[0]);inc(depth);
          stkStack.A[depth]^.init(p^.a^.int.childs[1],p^.a^.int.childs[1]);inc(depth);
          stkStack.A[depth]^.init(p^.a^.int.childs[0],p^.a^.int.childs[1]);inc(depth);
        end;
      end else
      if Intersect(p^.a^.volume,p^.b^.volume) then begin
        if p^.a^.isinternal then begin
          if p^.b^.isinternal then begin
            stkStack.A[depth]^.init(p^.a^.int.childs[0],p^.b^.int.childs[0]);inc(depth);
            stkStack.A[depth]^.init(p^.a^.int.childs[1],p^.b^.int.childs[0]);inc(depth);
            stkStack.A[depth]^.init(p^.a^.int.childs[0],p^.b^.int.childs[1]);inc(depth);
            stkStack.A[depth]^.init(p^.a^.int.childs[1],p^.b^.int.childs[1]);inc(depth);
          end else begin
            stkStack.A[depth]^.init(p^.a^.int.childs[0],p^.b);inc(depth);
            stkStack.A[depth]^.init(p^.a^.int.childs[1],p^.b);inc(depth);
          end;
        end else begin
          if p^.b^.isinternal then begin
            stkStack.A[depth]^.init(p^.a,p^.b^.int.childs[0]);inc(depth);
            stkStack.A[depth]^.init(p^.a,p^.b^.int.childs[1]);inc(depth);
          end else begin
            policy.Process(p^.a,p^.b);
          end;
        end;
      end;
    until (depth=0);
  end;
end;

procedure btDbvt.collideTTpersistentStack(const root0, root1: PbtDbvtNode; const policy: btDbvt_ICollide);
var depth    : integer;
    treshold : integer;
    p        : dyn_btDbvt_sStkNN; //PbtDbvt_sStkNN;
begin
  if assigned(root0) and assigned(root1) then begin
    depth:=1;
    treshold:=cbtDbvt_DOUBLE_STACKSIZE-4;
    m_stkStack.Setlength(cbtDbvt_DOUBLE_STACKSIZE);
    m_stkStack.A[0]^.Init(root0,root1);
    repeat
      dec(depth);
      p := m_stkStack.A[depth]^;
      if depth>treshold then begin
        m_stkStack.Setlength(m_stkStack.size*2);
        treshold:=m_stkStack.size-4;
      end;
      if p.a=p.b then begin
        if p.a^.isinternal then begin
          m_stkStack.A[depth]^.init(p.a^.int.childs[0],p.a^.int.childs[0]);inc(depth);
          m_stkStack.A[depth]^.init(p.a^.int.childs[1],p.a^.int.childs[1]);inc(depth);
          m_stkStack.A[depth]^.init(p.a^.int.childs[0],p.a^.int.childs[1]);inc(depth);
        end;
      end else
      if Intersect(p.a^.volume,p.b^.volume) then begin
        if p.a^.isinternal then begin
          if p.b^.isinternal then begin
            m_stkStack.A[depth]^.init(p.a^.int.childs[0],p.b^.int.childs[0]);inc(depth);
            m_stkStack.A[depth]^.init(p.a^.int.childs[1],p.b^.int.childs[0]);inc(depth);
            m_stkStack.A[depth]^.init(p.a^.int.childs[0],p.b^.int.childs[1]);inc(depth);
            m_stkStack.A[depth]^.init(p.a^.int.childs[1],p.b^.int.childs[1]);inc(depth);
          end else begin
            m_stkStack.A[depth]^.init(p.a^.int.childs[0],p.b);inc(depth);
            m_stkStack.A[depth]^.init(p.a^.int.childs[1],p.b);inc(depth);
          end;
        end else begin
          if p.b^.isinternal then begin
            m_stkStack.A[depth]^.init(p.a,p.b^.int.childs[0]);inc(depth);
            m_stkStack.A[depth]^.init(p.a,p.b^.int.childs[1]);inc(depth);
          end else begin
            policy.Process(p.a,p.b);
          end;
        end;
      end;
    until (depth=0);
  end;
end;

procedure btDbvt.collideTV(const root: PbtDbvtNode; const vol: btDbvtVolume; const policy: btDbvt_ICollide);
var volume : btDbvtVolume;
    n      : PbtDbvtNode;
begin
  if assigned(root) then begin
    volume := vol;
    tmp_stack.Resize(0);
    tmp_stack.Reserve(cbtDbvt_SIMPLE_STACKSIZE);
    tmp_stack.push_back(root);
    repeat
      n := tmp_stack.A[tmp_stack.size-1]^;
      tmp_stack.pop_back;
      if Intersect(n^.volume,volume) then begin
//        writeln('2--->> Voulme',volume.Center.DumpSimple());
        if n^.isinternal then begin
          tmp_stack.push_back(n^.int.childs[0]);
          tmp_stack.push_back(n^.int.childs[1]);
        end else begin
          policy.Process(n);
        end;
      end;
//      writeln('1--->> Voulme',volume.Center.DumpSimple());
    until not (tmp_stack.Size>0);
  end; //FOSTODO -> free objects on stack ...
end;

class procedure btDbvt.rayTest(const root: PbtDbvtNode; const rayFrom, rayTo: btVector3; const policy: btDbvt_ICollide);
var rayDir               : btVector3;
    rayDirectionInverse  : btVector3;
    signs                : btDbvt_SignArray;
    lambda_max           : btScalar;
    stack                : btDbvtNodePArray;
    node                 : PbtDbvtNode;
    depth                : integer;
    treshold             : integer;
    bounds               : array [0..1] of btVector3;
    tmin,
    lambda_min           : btScalar;
    result1              : boolean;
    {$ifdef COMPARE_BTRAY_AABB2}
    param                : btScalar;
    result2              : boolean;
    {$endif} //TEST_BTRAY_AABB2

begin
  stack:=nil;abort; //rethink mm
  if assigned(root) then begin
    rayDir     := (rayTo-rayFrom);
    lambda_max := rayDir.dot(rayDir);
    rayDir.normalize;

    ///what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT
    rayDir[0]  := btDecide( rayDir[0] = 0, BT_LARGE_FLOAT, 1 / rayDir[0]);
    rayDir[1]  := btDecide( rayDir[1] = 0, BT_LARGE_FLOAT, 1 / rayDir[1]);
    rayDir[2]  := btDecide( rayDir[2] = 0, BT_LARGE_FLOAT, 1 / rayDir[2]);
    signs[0]   := btDecide(rayDirectionInverse[0] < 0 ,1 , 0);
    signs[1]   := btDecide(rayDirectionInverse[1] < 0 ,1 , 0);
    signs[2]   := btDecide(rayDirectionInverse[2] < 0 ,1 , 0);
    //        lambda_max := rayDir.dot(rayTo-rayFrom); FOS moved UPWARDS ^^ !!
    depth      := 1;
    treshold   := cbtDbvt_DOUBLE_STACKSIZE-2;
    stack.resize(cbtDbvt_DOUBLE_STACKSIZE);
    stack.A[0]^  := root;
    repeat
      dec(depth);
      node      := stack.A[depth]^;
      bounds[0] := node^.volume.Mins^;
      bounds[1] := node^.volume.Maxs^;
      tmin      := 1;
      lambda_min:= 0;
      result1   := btRayAabb2(rayFrom,rayDirectionInverse,signs,bounds,tmin,lambda_min,lambda_max);

  {$ifdef COMPARE_BTRAY_AABB2}
      param     := 1;
      result2   := btRayAabb(rayFrom,rayTo,node^.volume.Mins^,node^.volume.Maxs^,param,resultNormal);
      btAssert(result1 = result2);
  {$endif} //TEST_BTRAY_AABB2
      if (result1) then begin
        if node^.isinternal then begin
          if depth>treshold then begin
            stack.resize(stack.size*2);
            treshold:=stack.size-2;
          end;
          stack.A[depth]^:=node^.int.childs[0];inc(depth);
          stack.A[depth]^:=node^.int.childs[1];inc(depth);
        end else begin
          policy.Process(node);
        end;
      end;
    until depth=0;
  end;
end;

procedure btDbvt.rayTestInternal(const root: PbtDbvtNode; const rayFrom,  rayToInverse: btVector3; const signs: btDbvt_SignArray; const lambdaMax: btScalar; const aabbMin, aabbMax: btVector3;  const policy: btDbvt_ICollide);
var depth         : integer;
    treshold      : integer;
    stack         : btDbvtNodePArray;
    node          : PbtDbvtNode;
    bounds        : array [0..1] of btVector3;
    tmin,
    lambda_min    : btScalar;
    result1       : boolean;

begin
  if assigned(root) then begin
    depth := 1;
    treshold := cbtDbvt_DOUBLE_STACKSIZE-2;
    stack := btDbvtNodePArray.create;
    stack.Setlength(cbtDbvt_DOUBLE_STACKSIZE);
    stack.A[0]^:=root;
    repeat
      dec(depth);
      node       := stack.A[depth]^;
      bounds[0]  := node^.volume.Mins^-aabbMax;
      bounds[1]  := node^.volume.Maxs^-aabbMin;
      tmin       := 1;
      lambda_min := 0;
      result1    := btRayAabb2(rayFrom,rayToInverse,signs,bounds,tmin,lambda_min,lambdaMax);
      if result1 then begin
        if node^.isinternal then begin
          if depth>treshold then begin
            stack.resize(stack.size*2);
            treshold:=stack.size-2;
          end;
          stack.A[depth]^ := node^.int.childs[0];inc(depth);
          stack.A[depth]^ := node^.int.childs[1];inc(depth);
        end else begin
          policy.Process(node);
        end;
      end;
    until depth=0;
    stack.Free;
  end;
end;

class procedure btDbvt.collideKDOP(const root: PbtDbvtNode; const normals: PbtVector3; const offsets: PbtScalar; const count: integer; const policy: btDbvt_ICollide);
var inside    : integer;
    stack     : btsStkNPArray;
    se,sdummy : dyn_btDbvt_sStkNP;
    signs     : Array [0..sizeof(cardinal) * 8] of integer;
    i,j,side  : Integer;
    outb      : boolean;
begin
  if assigned(root) then begin
    stack := btsStkNPArray.create;
    inside := (1 SHL count)-1;
    btAssert (count < round(sizeof(signs)/sizeof(signs[0])));
    for i:=0 to count-1 do begin
      //signs[i]= ((normals[i].x()>=0)?1:0)+
      //          ((normals[i].y()>=0)?2:0)+
      //          ((normals[i].z()>=0)?4:0);
      signs[i] :=  btDecide(normals[i]._x^>=0 , 1, 0)+
                   btDecide(normals[i]._y^>=0 , 2, 0)+
                   btDecide(normals[i]._z^>=0 , 4, 0);
    end;
    stack.reserve  (cbtDbvt_SIMPLE_STACKSIZE);
    se.Init(root,0);
    stack.push_back(se);
    repeat
      se   := stack.A[stack.size-1]^;
      outb := false;
      stack.pop_back;
      //for(int i=0,j=1;(!out)&&(i<count);++i,j<<=1)
      i:=0;j:=1;
      while (not(outb) and (i<count)) do begin
        if 0 = (se.mask and j) then begin
          side := se.node^.volume.Classify(normals[i],offsets[i],signs[i]);
          case  side of
            -1: outb    := true;
            +1: se.mask := se.mask or j;
          end;
        end;
        inc(i);
        j := j SHL 1;
      end;
      if not outb then begin
        if (se.mask <>  inside) and  (se.node^.isinternal) then begin
          sdummy.Init(se.node^.int.childs[0],se.mask);
          stack.push_back(sdummy);
          sdummy.Init(se.node^.int.childs[1],se.mask);
          stack.push_back(sdummy);
        end else begin
          //if se.node^.isinternal then begin
          //  abort;
          //end;
          if policy.AllLeaves(se.node) then begin
            enumLeaves(se.node,policy);
          end;
        end;
      end;
    until stack.size=0;
    stack.free;
  end;
end;

class procedure btDbvt.collideOCL(const root: PbtDbvtNode; const normals: Array of btVector3; const offsets: Array of btScalar; const sortaxis: btVector3; const count: integer; const policy: btDbvt_ICollide;  const fullsort: boolean);
var srtsigns : cardinal;
    inside   : integer;
    stock    : btsStkNPSArray;
    ifree    : btFOSAlignedIntegers;
    stack    : btFOSAlignedIntegers;
    signs    : Array [0..sizeof(cardinal) * 8] of integer;
    se,sdummy: dyn_btDbvt_sStkNPS;
    id,q,k   : integer;
    //depth    : integer;
    //treshold : integer;
    i,j,side : Integer;
    outb     : boolean;
    pns      : array [0..1] of PbtDbvtNode;
    nes      : array [0..1] of dyn_btDbvt_sStkNPS;

begin
  if assigned(root) then begin
   stack := btFOSAlignedIntegers.create;
   stock := btsStkNPSArray.create;
   ifree := btFOSAlignedIntegers.create;
    srtsigns :=  btDecide(sortaxis[0]>=0 , 1, 0)+
                 btDecide(sortaxis[1]>=0 , 2, 0)+
                 btDecide(sortaxis[2]>=0 , 4, 0);

    inside   := (1 SHL count)-1;
    btAssert(count<int (sizeof(signs)/sizeof(signs[0])));

    for i:=0 to count-1 do begin
      //signs[i]= ((normals[i].x()>=0)?1:0)+
      //          ((normals[i].y()>=0)?2:0)+
      //          ((normals[i].z()>=0)?4:0);
      signs[i] :=  btDecide(normals[i]._x^>=0 , 1, 0)+
                   btDecide(normals[i]._y^>=0 , 2, 0)+
                   btDecide(normals[i]._z^>=0 , 4, 0);
    end;
    stock.reserve(cbtDbvt_SIMPLE_STACKSIZE);
    stack.reserve(cbtDbvt_SIMPLE_STACKSIZE);
    ifree.reserve(cbtDbvt_SIMPLE_STACKSIZE);
    sdummy.init(root,0,root^.volume.ProjectMinimum(sortaxis,srtsigns));
    stack.push_back(btDbvt.allocate(ifree,stock,sdummy));
    repeat
      id := stack.A[stack.size-1]^;
      se := stock.A[id]^;
      stack.pop_back;
      ifree.push_back(id);
      if se.mask<>inside then begin
        outb :=false;
        i:=0;j:=1;
        while (not(outb) and (i<count)) do begin
          if 0 = (se.mask and j) then begin
            side := se.node^.volume.Classify(normals[i],offsets[i],signs[i]);
            case  side of
              -1: outb    := true;
              +1: se.mask := se.mask or j;
            end;
          end;
          inc(i);
          j := j SHL 1;
        end;
        if outb then continue;
      end;
      if policy.Descent(se.node) then begin
        if se.node^.isinternal then begin
          pns[0] := se.node^.int.childs[0];
          pns[1] := se.node^.int.childs[1];
          {$WARNINGS OFF}
          nes[0].init(pns[0],se.mask,pns[0]^.volume.ProjectMinimum(sortaxis,srtsigns));
          nes[1].init(pns[1],se.mask,pns[1]^.volume.ProjectMinimum(sortaxis,srtsigns));
          {$WARNINGS ON}
          q      := btDecide(nes[0].value<nes[1].value,1,0);
          j      := stack.size;
          if fullsort and  (j>0) then begin
            //* Insert 0     */
            j := btDbvt.nearest(stack,stock,nes[q].value,0,stack.size);
            stack.push_back(0);
          {$ifdef DBVT_USE_MEMMOVE}
            memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
          {$else}
            //for(int k=stack.size()-1;k>j;--k) stack[k]=stack[k-1];
            for k:=stack.size-1 downto j+1 do begin
              stack.A[k]^ := stack.A[k-1]^;
            end;
          {$endif}
            stack.A[j]^ := btDbvt.allocate(ifree,stock,nes[q]);
            //* Insert 1     */
            j := btDbvt.nearest(stack,stock,nes[1-q].value,j,stack.size);
            stack.push_back(0);
          {$ifdef DBVT_USE_MEMMOVE}
            memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
          {$else}
          //for(int k=stack.size()-1;k>j;--k) stack[k]=stack[k-1];
           for k:=stack.size-1 downto j+1 do begin
             stack.A[k]^ := stack.A[k-1]^;
           end;
          {$endif}
            stack.A[j]^ := btDbvt.allocate(ifree,stock,nes[1-q]);
          end else begin
            stack.push_back(btDbvt.allocate(ifree,stock,nes[q]));
            stack.push_back(btDbvt.allocate(ifree,stock,nes[1-q]));
          end;
        end else begin
          policy.Process(se.node,se.value);
        end;
      end;
    until stack.size=0;
    stack.Free;
    stock.free;
    ifree.Free;
  end;
end;

class procedure btDbvt.collideTU(const root: PbtDbvtNode; const policy: btDbvt_ICollide);
var stack : btDbvtNodePArray;
    n     : PbtDbvtNode;
begin
  stack := nil;abort; //rethink mm
  if assigned(root) then begin
    stack.reserve(cbtDbvt_SIMPLE_STACKSIZE);
    stack.push_back(root);
    repeat
      n := stack.A[stack.size()-1]^;
      stack.pop_back;
      if policy.Descent(n) then begin
        if n^.isinternal then begin
          stack.push_back(n^.int.childs[0]);
          stack.push_back(n^.int.childs[1]);
        end else begin
          policy.Process(n);
        end;
      end;
    until stack.size=0;
  end;
end;

class function btDbvt.nearest(const i: btFOSAlignedIntegers; const a: btsStkNPSArray; const v: btScalar; l, h: integer): integer;
var m:integer;
begin
  m:=0;
  while(l<h) do begin
    m:= (l+h) SHR 1;
    if a[i.Idx[m]]^.value>=v then begin
      l:=m+1;
    end else begin
      h:=m;
    end;
  end;
  result := h;
end;

class function btDbvt.allocate(const ifree: btFOSAlignedIntegers; const stock: btsStkNPSArray; const value: dyn_btDbvt_sStkNPS): integer;
var i:integer;
begin
  if ifree.size>0 then begin
    i:=ifree.A[ifree.size-1]^;
    ifree.pop_back;
    stock.A[i]^:=value;
  end else begin
    i:=stock.size;
    stock.push_back(value);
  end;
  Result := i;
end;


{ btIDebugDraw }


procedure btIDebugDraw.drawLine(const from, too, fromColor, toColor: btVector3);
begin
  toColor.absolute;
  drawLine (from, too, fromColor);
end;

procedure btIDebugDraw.drawSphere(const radius: btScalar; const transform: btTransform; const color: btVector3);
var start,xoffs,yoffs,zoffs : btVector3;
begin
  start := transform.getOriginV^;
  xoffs := transform.GetBasisV^ * btVector3.inits(radius,0,0);
  yoffs := transform.GetBasisV^ * btVector3.inits(0,radius,0);
  zoffs := transform.GetBasisV^ * btVector3.inits(0,0,radius);
  // XY
  drawLine(start-xoffs, start+yoffs, color);
  drawLine(start+yoffs, start+xoffs, color);
  drawLine(start+xoffs, start-yoffs, color);
  drawLine(start-yoffs, start-xoffs, color);
  // XZ
  drawLine(start-xoffs, start+zoffs, color);
  drawLine(start+zoffs, start+xoffs, color);
  drawLine(start+xoffs, start-zoffs, color);
  drawLine(start-zoffs, start-xoffs, color);
  // YZ
  drawLine(start-yoffs, start+zoffs, color);
  drawLine(start+zoffs, start+yoffs, color);
  drawLine(start+yoffs, start-zoffs, color);
  drawLine(start-zoffs, start-yoffs, color);
end;

procedure btIDebugDraw.drawSphere(const p: btVector3; const radius: btScalar; const color: btVector3);
var tr:btTransform;
begin
  tr.setIdentity;
  tr.setOrigin(p);
  drawSphere(radius,tr,color);
end;

{$HINTS OFF}
procedure btIDebugDraw.drawTriangle(const v0, v1, v2, n0, n1, n2, color: btVector3; const alpha: btScalar);
begin
   drawTriangle(v0,v1,v2,color,alpha);
end;

procedure btIDebugDraw.drawTriangle(const v0, v1, v2, color: btVector3; const alpha: btScalar);
begin
   drawLine(v0,v1,color);
   drawLine(v1,v2,color);
   drawLine(v2,v0,color);
end;
{$HINTS ON}

procedure btIDebugDraw.drawAabb(const from, too, color: btVector3);
var pa,pb,halfExtents,center,edgecoord:btVector3;
    i,j,othercoord: integer;
begin
  halfExtents := (too-from)* 0.5;
  center      := (too+from) *0.5;
  edgecoord.InitSame(1);
  for i:=0 to 3 do begin
    for j:=0 to 2 do begin
      pa.Init(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1],edgecoord[2]*halfExtents[2]);
      pa+=center;
      othercoord := j mod 3;
      edgecoord[othercoord] := edgecoord[othercoord] * -1;
      pb.Init(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1],edgecoord[2]*halfExtents[2]);
      pb+=center;
      drawLine(pa,pb,color);
    end;
    edgecoord.InitSame(-1);
    if (i<3) then begin
      edgecoord[i] := -1 * edgecoord[i];
    end;
   end;
end;

procedure btIDebugDraw.drawTransform(const transform: btTransform; const orthoLen: btScalar);
var start:btVector3;
begin
  start := transform.getOriginV^;
  drawLine(start, start+transform.GetBasisV^ * btVector3.inits(orthoLen, 0, 0), btVector3.inits(0.7,0,0));
  drawLine(start, start+transform.GetBasisV^ * btVector3.inits(0, orthoLen, 0), btVector3.inits(0,0.7,0));
  drawLine(start, start+transform.GetBasisV^ * btVector3.inits(0, 0, orthoLen), btVector3.inits(0,0,0.7));
end;

procedure btIDebugDraw.drawArc(const center, normal, axis: btVector3; const radiusA, radiusB, minAngle, maxAngle: btScalar; const color: btVector3; const drawSect: boolean; const stepDegrees: btScalar);
var vy,vx,prev,next:btVector3;
    step,angle : btScalar;
    nSteps : integer;
    i: Integer;
begin
  vx     := axis;
  vy     := normal.cross(axis);
  step   := stepDegrees * SIMD_RADS_PER_DEG;
  nSteps := round((maxAngle - minAngle) / step);
  if nSteps=0 then nSteps := 1;
  prev := center + radiusA * vx * btCos(minAngle) + radiusB * vy * btSin(minAngle);
  if drawSect then begin
    drawLine(center, prev, color);
  end;
  for i := 1 to nSteps do begin
    angle := minAngle + (maxAngle - minAngle) * btScalar(i) / btScalar(nSteps);
    next  := center + radiusA * vx * btCos(angle) + radiusB * vy * btSin(angle);
    drawLine(prev, next, color);
    prev  := next;
  end;
  if drawSect then begin
    drawLine(center, prev, color);
  end;
end;

procedure btIDebugDraw.drawSpherePatch(const center, up, axis: btVector3; radius, minTh, maxTh, minPs, maxPs: btScalar; const color: btVector3; const stepDegrees: btScalar);
var vA,vB                       :array [0..73] of btVector3;
    pT,pvA,pvB                  :PbtVector3;
    jv,kv,iv,arcStart,
    nPole,sPole                 :btVector3;
    step                        :btScalar;
    drawN,drawS,isClosed        :boolean;
    n_hor                       : integer;
    n_vert                      : Integer;
    step_v                      : btScalar;
    i                           : Integer;
    step_h,th,sth,cth,
    psi,sps,cps                 : btScalar;
    j: Integer;
begin
  pvA    := @vA;
  pvB    := @vB;
  npole  := center + up * radius;
  spole  := center - up * radius;
  step   := stepDegrees * SIMD_RADS_PER_DEG;
  kv     := up;
  iv     := axis;
  jv     := kv.cross(iv);
  drawN  := false;
  drawS  := false;
  if minTh <= -SIMD_HALF_PI then begin
    minTh := -SIMD_HALF_PI + step;
    drawN := true;
  end;
  if maxTh >= SIMD_HALF_PI then begin
    maxTh := SIMD_HALF_PI - step;
    drawS := true;
  end;
  if minTh > maxTh then begin
    minTh := -SIMD_HALF_PI + step;
    maxTh :=  SIMD_HALF_PI - step;
    drawN := drawS = true;
  end;
  n_hor := round ((maxTh - minTh) / step) + 1;
  if n_hor < 2 then begin
    n_hor := 2;
  end;
  step_h   := (maxTh - minTh) / btScalar(n_hor - 1);
  isClosed := false;
  if minPs > maxPs then begin
     minPs    := -SIMD_PI + step;
     maxPs    :=  SIMD_PI;
     isClosed := true;
  end else
  if (maxPs - minPs) >= (SIMD_PI*2) then begin
    isClosed := true;
  end else begin
    isClosed := false;
  end;
  n_vert := round ((maxPs - minPs) / step) + 1;
  if n_vert < 2 then begin
    n_vert := 2;
  end;
  step_v := (maxPs - minPs) / btScalar(n_vert - 1);
  for  i := 0 to n_hor-1 do begin
    th  := minTh + btScalar(i) * step_h;
    sth := radius * btSin(th);
    cth := radius * btCos(th);
    for j := 0 to n_vert-1 do begin
      psi := minPs + btScalar(j) * step_v;
      sps := btSin(psi);
      cps := btCos(psi);
      pvB[j] := center + cth * cps * iv + cth * sps * jv + sth * kv;
      if i<>0 then begin
        drawLine(pvA[j], pvB[j], color);
      end else
      if drawS then begin
        drawLine(spole, pvB[j], color);
      end;
      if j<>0 then begin
        drawLine(pvB[j-1], pvB[j], color);
      end else begin
        arcStart := pvB[j];
      end;
      if (i=(n_hor-1)) and drawN then begin
         drawLine(npole, pvB[j], color);
      end;
      if isClosed then begin
        if j = (n_vert-1) then begin
          drawLine(arcStart, pvB[j], color);
        end;
      end else begin
        if ((i=0) or (i = (n_hor-1))) and ((j=0) or (j = (n_vert-1))) then begin
          drawLine(center, pvB[j], color);
        end;
      end;
    end;
    pT := pvA; pvA := pvB; pvB := pT;
  end;
end;

{$HINTS OFF}
procedure btIDebugDraw.drawBox(const bbMin, bbMax, color: btVector3;const alpha : btScalar);
begin
  drawLine(btVector3.inits(bbMin[0], bbMin[1], bbMin[2]), btVector3.inits(bbMax[0], bbMin[1], bbMin[2]), color);
  drawLine(btVector3.inits(bbMax[0], bbMin[1], bbMin[2]), btVector3.inits(bbMax[0], bbMax[1], bbMin[2]), color);
  drawLine(btVector3.inits(bbMax[0], bbMax[1], bbMin[2]), btVector3.inits(bbMin[0], bbMax[1], bbMin[2]), color);
  drawLine(btVector3.inits(bbMin[0], bbMax[1], bbMin[2]), btVector3.inits(bbMin[0], bbMin[1], bbMin[2]), color);
  drawLine(btVector3.inits(bbMin[0], bbMin[1], bbMin[2]), btVector3.inits(bbMin[0], bbMin[1], bbMax[2]), color);
  drawLine(btVector3.inits(bbMax[0], bbMin[1], bbMin[2]), btVector3.inits(bbMax[0], bbMin[1], bbMax[2]), color);
  drawLine(btVector3.inits(bbMax[0], bbMax[1], bbMin[2]), btVector3.inits(bbMax[0], bbMax[1], bbMax[2]), color);
  drawLine(btVector3.inits(bbMin[0], bbMax[1], bbMin[2]), btVector3.inits(bbMin[0], bbMax[1], bbMax[2]), color);
  drawLine(btVector3.inits(bbMin[0], bbMin[1], bbMax[2]), btVector3.inits(bbMax[0], bbMin[1], bbMax[2]), color);
  drawLine(btVector3.inits(bbMax[0], bbMin[1], bbMax[2]), btVector3.inits(bbMax[0], bbMax[1], bbMax[2]), color);
  drawLine(btVector3.inits(bbMax[0], bbMax[1], bbMax[2]), btVector3.inits(bbMin[0], bbMax[1], bbMax[2]), color);
  drawLine(btVector3.inits(bbMin[0], bbMax[1], bbMax[2]), btVector3.inits(bbMin[0], bbMin[1], bbMax[2]), color);
end;
{$HINTS ON}

procedure btIDebugDraw.drawBox(const bbMin, bbMax: btVector3; const trans: btTransform; const color: btVector3);
begin
  drawLine(trans * btVector3.inits(bbMin[0], bbMin[1], bbMin[2]), trans * btVector3.inits(bbMax[0], bbMin[1], bbMin[2]), color);
  drawLine(trans * btVector3.inits(bbMax[0], bbMin[1], bbMin[2]), trans * btVector3.inits(bbMax[0], bbMax[1], bbMin[2]), color);
  drawLine(trans * btVector3.inits(bbMax[0], bbMax[1], bbMin[2]), trans * btVector3.inits(bbMin[0], bbMax[1], bbMin[2]), color);
  drawLine(trans * btVector3.inits(bbMin[0], bbMax[1], bbMin[2]), trans * btVector3.inits(bbMin[0], bbMin[1], bbMin[2]), color);
  drawLine(trans * btVector3.inits(bbMin[0], bbMin[1], bbMin[2]), trans * btVector3.inits(bbMin[0], bbMin[1], bbMax[2]), color);
  drawLine(trans * btVector3.inits(bbMax[0], bbMin[1], bbMin[2]), trans * btVector3.inits(bbMax[0], bbMin[1], bbMax[2]), color);
  drawLine(trans * btVector3.inits(bbMax[0], bbMax[1], bbMin[2]), trans * btVector3.inits(bbMax[0], bbMax[1], bbMax[2]), color);
  drawLine(trans * btVector3.inits(bbMin[0], bbMax[1], bbMin[2]), trans * btVector3.inits(bbMin[0], bbMax[1], bbMax[2]), color);
  drawLine(trans * btVector3.inits(bbMin[0], bbMin[1], bbMax[2]), trans * btVector3.inits(bbMax[0], bbMin[1], bbMax[2]), color);
  drawLine(trans * btVector3.inits(bbMax[0], bbMin[1], bbMax[2]), trans * btVector3.inits(bbMax[0], bbMax[1], bbMax[2]), color);
  drawLine(trans * btVector3.inits(bbMax[0], bbMax[1], bbMax[2]), trans * btVector3.inits(bbMin[0], bbMax[1], bbMax[2]), color);
  drawLine(trans * btVector3.inits(bbMin[0], bbMax[1], bbMax[2]), trans * btVector3.inits(bbMin[0], bbMin[1], bbMax[2]), color);
end;


{ btDbvtAabbMm }

procedure btDbvtAabbMm.AddSpan(const d: btVector3; var smi, smx: btScalar);
var i:integer;
begin
  for i:=0 to 2 do begin
    if d[i]<0 then begin
      smi+=mx[i]*d[i];
      smx+=mi[i]*d[i];
    end else begin
      smi+=mi[i]*d[i];
      smx+=mx[i]*d[i];
    end;
  end;
end;

function btDbvtAabbMm.Center: btVector3;
begin
  result := (mi+mx)/2;
end;

function btDbvtAabbMm.Lengths: btVector3;
begin
  result := mx-mi;
end;

function btDbvtAabbMm.Extents: btVector3;
begin
  result := (mx-mi)/2;
end;

function btDbvtAabbMm.Mins: PbtVector3;
begin
  result := @mi;
end;

function btDbvtAabbMm.Maxs: PbtVector3;
begin
  result := @mx;
end;

procedure btDbvtAabbMm.FromCE(const c, e: btVector3);
begin
  mi := c-e;
  mx := c+e;
end;

procedure btDbvtAabbMm.FromCR(const c: btVector3; const r: btScalar);
begin
  FromCE(c,btVector3.InitSameS(r));
end;

procedure btDbvtAabbMm.FromMM(const imiv, imxv: btVector3);
begin
  mi := imiv;
  mx := imxv;
end;

procedure btDbvtAabbMm.FromPoints(const pts: PbtVector3; const n: integer);
var  // box : btDbvtAabbMm;
      i   : integer;
begin
  mx := pts[0];
  mi := mx;
  for i:=1  to n-1 do begin
    mi.setMin(pts[i]);
    mx.setMax(pts[i]);
  end;
end;

procedure btDbvtAabbMm.FromPoints(const pts: PPbtVector3; const n: integer);
var i: Integer;
begin
  mx := pts[0]^;
  mi := mx;
  for i:=1 to n-1 do begin
    mi.setMin(pts[i]^);
    mx.setMax(pts[i]^);
  end;
end;

procedure btDbvtAabbMm.Expand(const e: btVector3);
begin
  mi-=e;mx+=e;
end;

procedure btDbvtAabbMm.SignedExpand(const e: btVector3);
begin
  if(e._x^>0) then mx._X^:=(mx._x^+e[0]) else mi.setX(mi._X^+e[0]);
  if(e._y^>0) then mx._Y^:=(mx._y^+e[1]) else mi.setY(mi._y^+e[1]);
  if(e._z^>0) then mx._z^:=(mx._z^+e[2]) else mi.setZ(mi._z^+e[2]);
end;

function btDbvtAabbMm.Contain(const a: btDbvtAabbMm): boolean;
begin
  result := (mi._x^<=a.mi._x^) AND (mi._y^<=a.mi._y^) AND (mi._z^<=a.mi._z^) AND (mx._x^>=a.mx._x^) AND (mx._y^>=a.mx._y^) AND (mx._z^>=a.mx._z^);
end;

function btDbvtAabbMm.Classify(const n: btVector3; const o: btScalar; const s: integer): integer;
var pi,px : btVector3;
begin
  case s of
    (0+0+0): begin
               px:=btVector3.InitS(mi._x^,mi._y^,mi._z^);
               pi:=btVector3.InitS(mx._x^,mx._y^,mx._z^);
             end;
    (1+0+0): begin
               px:=btVector3.initS(mx._x^,mi._y^,mi._z^);
               pi:=btVector3.initS(mi._x^,mx._y^,mx._z^);
             end;
    (0+2+0): begin
               px:=btVector3.inits(mi._x^,mx._y^,mi._z^);
               pi:=btVector3.inits(mx._x^,mi._y^,mx._z^);
             end;
    (1+2+0): begin
               px:=btVector3.inits(mx._x^,mx._y^,mi._z^);
               pi:=btVector3.inits(mi._x^,mi._y^,mx._z^);
             end;
    (0+0+4): begin
               px:=btVector3.inits(mi._x^,mi._y^,mx._z^);
               pi:=btVector3.inits(mx._x^,mx._y^,mi._z^);
             end;
    (1+0+4): begin
               px:=btVector3.inits(mx._x^,mi._y^,mx._z^);
               pi:=btVector3.inits(mi._x^,mx._y^,mi._z^);
             end;
    (0+2+4): begin
               px:=btVector3.inits(mi._x^,mx._y^,mx._z^);
               pi:=btVector3.inits(mx._x^,mi._y^,mi._z^);
             end;
    (1+2+4): begin
               px:=btVector3.inits(mx._x^,mx._y^,mx._z^);
               pi:=btVector3.inits(mi._x^,mi._y^,mi._z^);
             end;
  end;
  if (btDot(n,px)+o)<0 then begin
    exit(-1);
  end;
  if (btDot(n,pi)+o)>=0 then begin
    exit(+1);
  end;
  Result := 0;
end;

function btDbvtAabbMm.ProjectMinimum(const v: btVector3; const signs: cardinal): btScalar;
var   p:btVector3;
      b:array [0..1] of PbtVector3;
begin
  b[0]:=@mx ; b[1] := @mi;
  p.init( b[signs AND 1]^._X^ , b[(signs SHR 1) AND 1]^._Y^ , b[(signs SHR 2) AND 1]^._Z^ );
  result := btDot(p,v);
end;


{ btDbvtNode }

function dyn_btDbvtNode.isleaf: boolean;
begin
  result := int.childs[1] = nil;
end;

function dyn_btDbvtNode.isinternal: boolean;
begin
  result := not isleaf;
end;

{ btDbvt_sStkNN }

procedure dyn_btDbvt_sStkNN.Init(const na, nb: PbtDbvtNode);
begin
 a:=na;
 b:=nb;
end;

{ btDbvt_sStkNP }

procedure dyn_btDbvt_sStkNP.Init(const n: PbtDbvtNode; const m: cardinal);
begin
  node:=n;
  mask:=m;
end;

{ btDbvt_sStkNPS }

procedure dyn_btDbvt_sStkNPS.Init(const n: PbtDbvtNode; const m: cardinal; const v: btScalar);
begin
  node:=n;
  mask:=m;
  value:=v;
end;

{ btDbvt_sStkCLN }

procedure dyn_btDbvt_sStkCLN.Init(const n, p: PbtDbvtNode);
begin
  node   := n;
  parent := p;
end;

{ btDbvt_ICollide }

{$HINTS OFF}
procedure btDbvt_ICollide.Process(const n1, n2: PbtDbvtNode);
begin

end;

procedure btDbvt_ICollide.Process(const n: PbtDbvtNode);
begin

end;

procedure btDbvt_ICollide.Process(const n: PbtDbvtNode; const s: btScalar);
begin
  Process(n);
end;

function btDbvt_ICollide.Descent(const n: PbtDbvtNode): boolean;
begin
  result := true;
end;

function btDbvt_ICollide.AllLeaves(const n: PbtDbvtNode): boolean;
begin
  result:=true;
end;
{$HINTS ON}



{ btDbvtProxy }


procedure btDbvtProxy.init(const aabbMin, aabbMax: btVector3; userPtr: Pointer; const collisionFilterGroup, collisionFilterMask: btCollisionFilterGroupSet ; const multiSapParentProxy:pointer=nil );
begin
  inherited init(aabbMin,aabbMax,userPtr,collisionFilterGroup,collisionFilterMask,multiSapParentProxy);
  links[0]:=nil;
  links[1]:=nil;
end;

{ gbt_list }

procedure gbt_list.listappend(const item: T; var list: T);
begin
  item.links[0] := nil;
  item.links[1] := list;
  if assigned(TObject(list)) then begin
    list.links[0] := item;
  end;
  list := item;
end;

//procedure gbt_list.listremove(const item: T; var list: T);
//begin
//
//  //if assigned(TObject(item.links[0])) then begin
//  //  item.links[0].links[1] := item.links[1];
//  //end else begin
//  //  list := item.links[1];
//  //end;
//  //if assigned(TObject(item.links[1])) then begin
//  //  item.links[1].links[0] := item.links[0];
//  //end;
//end;

procedure gbt_list.listremove(var item: T; var list: T);
begin
  if assigned(TObject(item.links[0])) then begin
    item.links[0].links[1] := item.links[1];
  end else begin
    list := item.links[1];
  end;
  if assigned(TObject(item.links[1])) then begin
   item.links[1].links[0] := item.links[0];
  end;
end;

function gbt_list.listcount(root: T): integer;
var n:integer;
begin
  n:=0;
  while assigned(TObject(root)) do begin
    inc(n);
    root:=root.links[1];
  end;
  result:=n;
end;

procedure gbt_list.clear(var item: T);
begin
  TObject(item):=nil;
end;

end.

