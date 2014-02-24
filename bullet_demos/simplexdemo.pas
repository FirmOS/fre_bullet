unit SimplexDemo;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils,btCollisionShapes,btDispatch,btNarrowphase,btLinearMath,glutstuff,gl,glu,glut;

const  maxNumObjects = 4;
          numObjects = 1;

var  simplexSolver : btVoronoiSimplexSolver;
     yaw,pitch,roll : btScalar;
     objects       : array [0..maxNumObjects-1] of btCollisionObject;
     collisionWorld : btCollisionWorld;
     simplex        : GL_Simplex1to4;
     debugDrawer    : btGLDebugDrawer;
     shapePtr       : array [0..maxNumObjects-1] of btPolyhedralConvexShape;

type

    { btSimplexDemo }

    btSimplexDemo=class(btGlutDemoApplication)
      procedure initPhysics; override;
      procedure cleanupPhysics;
      procedure clientMoveAndDisplay; override;
      procedure displayCallback; override;
      destructor destroy; override;
    end;

implementation

{ btSimplexDemo }

procedure btSimplexDemo.initPhysics;
begin
  simplexSolver:=btVoronoiSimplexSolver.Create;
  simplex := GL_Simplex1to4.Create;
  shapePtr[0] := simplex;
  simplex.setSimplexSolver(simplexSolver);
  simplex.addVertex(btVector3.inits(-2,0,-2));
  simplex.addVertex(btVector3.inits(2,0,-2));
  simplex.addVertex(btVector3.inits(0,0,2));
  simplex.addVertex(btVector3.inits(0,2,0));
end;

procedure btSimplexDemo.cleanupPhysics;
begin
  simplex.free;
  simplexSolver.free;
end;

procedure btSimplexDemo.clientMoveAndDisplay;
begin
  displayCallback;
end;

procedure btSimplexDemo.displayCallback;
var   m : array [0..15] of btScalar;
      i : integer;
      worldBoundsMin,
      dpos,
      worldBoundsMax  : btVector3;
      transA          : btTransform;
      orn             : btQuaternion;

begin
  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);
  glDisable(GL_LIGHTING);

  btGL_ShapeDrawer.drawCoordSystem;

  worldBoundsMin.InitSame(-1000);
  worldBoundsMax.Initsame(1000);

  for i := 0 to numObjects-1 do begin
    transA.setIdentity();
    dpos.Init(0,5,0);
    transA.setOrigin(dpos);
    orn.setEuler(yaw,pitch,roll);
    transA.setRotation(orn);
    transA.getOpenGLMatrix(m);
    /// draw the simplex
    m_shapeDrawer.drawOpenGL(m,shapePtr[i],btVector3.InitSameS(1),getDebugMode,worldBoundsMin,worldBoundsMax);
    /// calculate closest point from simplex to the origin, and draw this vector
    simplex.calcClosest(m);
  end;
  pitch += 0.005;
  yaw += 0.01;
  glFlush();
  glutSwapBuffers();
end;

destructor btSimplexDemo.destroy;
begin
  cleanupPhysics;
  inherited destroy;
end;

end.

