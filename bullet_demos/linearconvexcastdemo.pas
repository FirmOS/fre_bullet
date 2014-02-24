unit LinearConvexCastDemo;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils,btCollisionShapes,btDispatch,btNarrowphase,btLinearMath,glutstuff,gl,glu,glut;

const  maxNumObjects = 4;
          numObjects = 2;

var  sVoronoiSimplexSolver : btVoronoiSimplexSolver;
     gGjkSimplexSolver     : btSimplexSolverInterface;
     yaw,pitch,roll        : btScalar;
     objects               : array [0..maxNumObjects-1] of btCollisionObject;
     shapePtr              : array [0..maxNumObjects-1] of btPolyhedralConvexShape;
     tr                    : array [0..NumObjects-1] of btTransform;

type

   { btLinearConvexCastDemo }

   btLinearConvexCastDemo = class(btGlutDemoApplication)
   private
     shapeA,shapeB : btConvexHullShape;
     angle : btScalar;
   public
      procedure initPhysics; override;
      procedure cleanupPhysics;
      procedure clientMoveAndDisplay; override;
      procedure displayCallback; override;
      destructor destroy; override;
   end;

implementation

{ btLinearConvexCastDemo }

procedure btLinearConvexCastDemo.initPhysics;
var r,h           : btScalar;
begin
  yaw:=0;
  pitch:=0;
  roll:=0;
  sVoronoiSimplexSolver:=btVoronoiSimplexSolver.Create;
  gGjkSimplexSolver:=sVoronoiSimplexSolver;

  setCameraDistance(10);

  tr[0].setIdentity;
  tr[0].setOrigin(btVector3.InitS(0,5.5,0.0));
  tr[1].setIdentity;
  tr[1].setOrigin(btVector3.InitSameS(0));

  // Pyramide
  r := 1;
  h := 2;

  shapeA := btConvexHullShape.Create;
  shapeA.addPoint( btVector3.inits(  0,  0.75 * h, 0.0  ) );
  shapeA.addPoint( btVector3.inits( -r, -0.25 * h,    r ) );
  shapeA.addPoint( btVector3.inits(  r, -0.25 * h,    r ) );
  shapeA.addPoint( btVector3.inits(  r, -0.25 * h,   -r ) );
  shapeA.addPoint( btVector3.inits( -r, -0.25 * h,   -r ) );

  // Triangle
  shapeB := btConvexHullShape.Create;
  shapeB.addPoint( btVector3.inits(  0.0,  1.0, 0.0));
  shapeB.addPoint( btVector3.inits(  1.0, -1.0, 0.0));
  shapeB.addPoint( btVector3.inits( -1.0, -1.0, 0.0));

  shapePtr[0] := shapeA;
  shapePtr[1] := shapeB;

  shapePtr[0].setMargin(0.01);
  shapePtr[1].setMargin(0.01);

end;

procedure btLinearConvexCastDemo.cleanupPhysics;
begin
  shapeA.Free;
  shapeB.Free;
  sVoronoiSimplexSolver.Free;
end;

procedure btLinearConvexCastDemo.clientMoveAndDisplay;
begin
  displayCallback;
end;

procedure btLinearConvexCastDemo.displayCallback;
var  toA, toB,tmp,A,B : btTransform;
     worldBoundsMin,
     worldBoundsMax   : btVector3;
     convexCaster     : btSubsimplexConvexCast;
     result           : btConvexCastResult;
     m1,m2,m3         : array [0..15] of btScalar;
     sphere           : btSphereShape;
     originA, originB : btVector3;
begin
  updateCamera;

  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);
  glDisable(GL_LIGHTING);

  btGL_ShapeDrawer.drawCoordSystem;

//  angle += getDeltaTimeMicroseconds/1000000.0;
  angle += 0.01;

  tr[1].setRotation(btQuaternion.InitQS(btVector3.inits(1,0,0),angle));

  toA := tr[0];
  toA.setOrigin( btVector3.InitSameS(0));
  toB := tr[1];
  toB.setOrigin( btVector3.initsames(0));

  gGjkSimplexSolver.reset;

  worldBoundsMin.init(-1000,-1000,-1000);
  worldBoundsMax.init(1000,1000,1000);

  //btGjkConvexCast convexCaster(shapePtr[ 0 ], shapePtr[ 1 ], &gGjkSimplexSolver );
  convexCaster := btSubsimplexConvexCast.Create( shapePtr[ 0 ], shapePtr[ 1 ], gGjkSimplexSolver );

  result:=btConvexCastResult.create;
  result.m_hitPoint.init(0,0,0);

  convexCaster.calcTimeOfImpact( tr[ 0 ], toA, tr[ 1 ], toB, result );

  tr[ 0 ].getOpenGLMatrix( m1 );
  tr[ 1 ].getOpenGLMatrix( m2 );

  sphere := btSphereShape.Create(0.2);

  tmp := tr[0];
  tmp.setOrigin(result.m_hitPoint);
  tmp.getOpenGLMatrix(m3);

  m_shapeDrawer.drawOpenGL( m3, sphere       , btVector3.InitS( 1, 0, 1 ), getDebugMode ,worldBoundsMin,worldBoundsMax);
  m_shapeDrawer.drawOpenGL( m1, shapeA, btVector3.inits( 1, 0, 0 ), getDebugMode ,worldBoundsMin,worldBoundsMax);
  m_shapeDrawer.drawOpenGL( m2, shapeB, btVector3.inits( 1, 0, 0 ), getDebugMode ,worldBoundsMin,worldBoundsMax);

  originA.setInterpolate3( tr[ 0 ].getOriginV^, toA.getOriginV^, result.m_fraction );
  originB.setInterpolate3( tr[ 1 ].getOriginV^, toB.getOriginV^, result.m_fraction );

  A := tr[ 0 ];
  A.setOrigin( originA );

  B := tr[ 1 ];
  B.setOrigin( originB );

  A.getOpenGLMatrix( m1 );
  B.getOpenGLMatrix( m2 );

  m_shapeDrawer.drawOpenGL( m1, shapePtr[ 0 ], btVector3.inits( 1, 1, 0 ), getDebugMode ,worldBoundsMin,worldBoundsMax);
  m_shapeDrawer.drawOpenGL( m2, shapePtr[ 1 ], btVector3.inits( 1, 1, 0 ), getDebugMode ,worldBoundsMin,worldBoundsMax);

  glFlush();
  glutSwapBuffers();
  convexCaster.free;
  sphere.free;
  result.free;
end;

destructor btLinearConvexCastDemo.destroy;
begin
  sVoronoiSimplexSolver.Free;
  shapeA.Free;
  shapeB.Free;
  inherited destroy;
end;

end.

