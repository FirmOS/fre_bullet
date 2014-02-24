{$mode objfpc}{$H+}

{$MACRO ON}
{.$STACKFRAMES OFF} // Ommit StackFrames
{.$DEFINE FRE_BULLET_DOUBLE_PRECISION}

{$DEFINE FRE_BT_ASSERTIONS}
{$define USE_BT_CLOCK}
{$DEFINE DEBUG}
{$define FOS_DEBUGINIT} // Check initialization of FOS Generic Aligned Arrays Objects
{$define BT_NO_PROFILE}

{$define cbtBULLET_VERSION:=276}

{$IFDEF FRE_BULLET_DOUBLE_PRECISION}
  {$define BT_LARGE_FLOAT:=double(1e30)}
{$ELSE}
  {$define BT_LARGE_FLOAT:=double(1e18)}
{$ENDIF}

{.$DEFINE FOS_INLINE:=inline}
{$DEFINE FOS_INLINE:=//}

{$IFDEF FRE_BT_ASSERTIONS}
  {$DEFINE btAssert:=myAssert}
{$ELSE}
  {$DEFINE btAssert:=//}
{$ENDIF}
{$DEFINE BT_PROFILE:=//}

{$DEFINE CONVEX_DISTANCE_MARGIN:= 0.04} // btScalar(0.1)//;//btScalar(0.01)
{$DEFINE NUM_UNITSPHERE_POINTS:=42}
{$DEFINE MAX_PREFERRED_PENETRATION_DIRECTIONS:=10}

{$define RAYAABB2}
{$define DEBUG_CHECK_DEQUANTIZATION}
{$define DEBUG_TREE_BUILDING}
// //http://msdn.microsoft.com/library/default.asp?url=/library/en-us/vclang/html/vclrf__m128.asp
//Note: currently we have 16 bytes per quantized node
{$DEFINE MAX_SUBTREE_SIZE_IN_BYTES:=  2048}
// 10 gives the potential for 1024 parts, with at most 2^21 (2097152) (minus one
// actually) triangles each (since the sign bit is reserved
{$define MAX_NUM_PARTS_IN_BITS:=10}

//btDbvt
{.$define DBVT_BP_PROFILE}
{.$define DBVT_BP_PREVENTFALSEUPDATE}
{.$define DBVT_BP_ACCURATESLEEPING}
{.$define DBVT_BP_ENABLE_BENCHMARK}
{$define DBVT_BP_MARGIN:=0.05}
{$ifdef DBVT_BP_PROFILE}
  {$define DBVT_BP_PROFILING_RATE:=256}
{$endif}

{$define VORONOI_SIMPLEX_MAX_VERTS:=5}
///disable next define, or use defaultCollisionConfiguration->getSimplexSolver()->setEqualVertexThreshold(0.f) to disable/configure
{$define BT_USE_EQUAL_VERTEX_THRESHOLD}
{$define VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD:=0.0001}
{$define VERTA:=0}
{$define VERTB:=1}
{$define VERTC:=2}
{$define VERTD:=3}
{$define CATCH_DEGENERATE_TETRAHEDRON:=1}

/// btGjkPairDetector uses GJK to implement the btDiscreteCollisionDetectorInterface
  //must be above the machine epsilon
{$define REL_ERROR2:=btScalar(1.0e-6)}
{$define cu_EPSILON:=btScalar(1.0e-6)}

//CollisionInterfaceDemo
{$define TEST_NOT_ADDING_OBJECTS_TO_WORLD}
//CollisionInterfaceDemo End


//-- Internal Defines

{$define USE_PERSISTENT_CONTACTS}

{$define USE_DISPATCH_REGISTRY_ARRAY 1}
{$define CLEAN_INVALID_PAIRS 1}

{$define KEEP_DEEPEST_POINT}
{$define MAINTAIN_PERSISTENCY}

{$ifdef BT_USE_DOUBLE_PRECISION}
  {$define MAX_ITERATIONS:=64}
{$else}
  {$define MAX_ITERATIONS:=32}
{$endif}


// GJK	- Config
{$define GJK_MAX_ITERATIONS:= 128    }
{$define GJK_ACCURARY:=       0.0001 }
{$define GJK_MIN_DISTANCE:=   0.0001 }
{$define GJK_DUPLICATED_EPS:= 0.0001 }
{$define GJK_SIMPLEX2_EPS:=   0.0    }
{$define GJK_SIMPLEX3_EPS:=   0.0    }
{$define GJK_SIMPLEX4_EPS:=   0.0    }

// EPA
{$define EPA_MAX_VERTICES:=   64                   }
{$define EPA_MAX_FACES:=      (EPA_MAX_VERTICES*2) }
{$define EPA_MAX_ITERATIONS:= 255                  }
{$define EPA_ACCURACY:=       0.0001               }
{$define EPA_FALLBACK:=       (10*EPA_ACCURACY)    }
{$define EPA_PLANE_EPS:=      0.00001              }
{$define EPA_INSIDE_EPS:=     0.01                 }

//Axis Sweep 3
{$define USE_OVERLAP_TEST_ON_REMOVES:=1}


//FirmOS Object Init Debugging Macros
//{$IFDEF FOS_DEBUGINIT}
//  {$DEFINE CFOS_INIT_MAGIC:=$DEADBEAF}
//  {$DEFINE FOS_DEBUGINIT_GUARD:=FOS_DBG_DBG_INIT_CheckMagic(fos_dbg_magic);}
//  {$DEFINE FOS_DEBUGINIT_SAVE:=fos_dbg_magic := CFOS_INIT_MAGIC}
//  {$DEFINE FOS_DEBUGINIT_INIT:=fos_dbg_magic:cardinal}
//{$ELSE}
//  {$DEFINE FOS_DEBUGINIT_GUARD:=//}
//  {$DEFINE FOS_DEBUGINIT_SAVE:=//}
//  {$DEFINE FOS_DEBUGINIT_INIT:=//}
//{$ENDIF}

{$IFDEF FOS_PROFILE}
  {$DEFINE FOS_START_PROFILE:=GFRE_PROFILER.Startkey }
  {$DEFINE FOS_END_PROFILE:=GFRE_PROFILER.Endkey }
  {$DEFINE FOS_DUMP_PROFILE:=GFRE_PROFILER.Dump; }
  {$DEFINE FOS_DUMP_PROFILE_US:=GFRE_PROFILER.Dump(false); }
{$ELSE}
  {$DEFINE FOS_START_PROFILE:=// }
  {$DEFINE FOS_END_PROFILE:=//}
  {$DEFINE FOS_DUMP_PROFILE:=//}
  {$DEFINE FOS_DUMP_PROFILE_US:=// }
{$ENDIF}

//TypedConstraint
{$define DEFAULT_DEBUGDRAW_SIZE:=0.3}
//Dynamics
{$define USE_PATH_COMPRESSION:=1}
///see for discussion of static island optimizations by Vroonsh here: http://code.google.com/p/bullet/issues/detail?id=406
{$define STATIC_SIMULATION_ISLAND_OPTIMIZATION:=1}

