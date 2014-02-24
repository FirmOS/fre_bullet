unit ContinuosConvexCollisionDemo;

interface

{$i ../fos_bullet.inc}

uses
  Classes, SysUtils,gl,glut,glutstuff,btCollisionShapes,btDispatch,btBroadphase,btNarrowphase,btLinearMath;

const  maxNumObjects = 4;
          numObjects = 2;

  type
    PlatformDemoApplication = btGlutDemoApplication;

    { btCollisionInterfaceDemo }
    btContinuousConvexCollisionDemo =class(PlatformDemoApplication)
    private
      f_initialized          : boolean;
      f_param                : string;
      //renderCallback         : Tobject;
      //boxA,boxB              : btBoxShape;
      yaw,pitch,roll         : btScalar;
      //collisionConfiguration : btDefaultCollisionConfiguration;
      //dispatcher             : btDispatcher;
      //broadphase             : btBroadphaseInterface;

      angVels                : array [0..numObjects-1]    of btVector3;
      linVels                : array [0..numObjects-1]    of btVector3;
      shapePtr               : array [0..maxNumObjects-1] of btPolyhedralConvexShape;
      boxA,boxB              : btBoxShape;

      fromTrans,toTrans      : array [0..maxNumObjects-1] of btTransform;

      //static btVoronoiSimplexSolver sVoronoiSimplexSolver;
      //btSimplexSolverInterface&
      gGjkSimplexSolver       : btSimplexSolverInterface;
      drawLine                : boolean;
      minlines                : integer;
      maxlines                : integer;
    public
      constructor Create(const param:string);
      destructor  Destroy;override;
      procedure   initPhysics;override;
      procedure   cleanPhysics;
      procedure   clientMoveAndDisplay; override;
      procedure   displayCallback; override;
//      procedure   clientResetScene; override;
  end;



implementation

{ btCollisionInterfaceDemo }

type

  { btDrawingResult }

  btDrawingResult = class(btCW_ContactResultCallback)
    function addSingleResult(const cp: PbtOManifoldPoint; const colObj0: btCollisionObject; const partId0, index0: integer; const colObj1: btCollisionObject; const partId1, index1: integer): btScalar; override;
  end;

{ btDrawingResult }

{$HINTS OFF}
function btDrawingResult.addSingleResult(const cp: PbtOManifoldPoint; const colObj0: btCollisionObject; const partId0, index0: integer; const colObj1: btCollisionObject; const partId1, index1: integer): btScalar;
var ptA,ptB:PbtVector3;
begin
  glBegin(GL_LINES);
  glColor3f(0, 0, 0);

  ptA := cp^.getPositionWorldOnAP;
  ptB := cp^.getPositionWorldOnBP;

  glVertex3d(ptA^.x,ptA^.y,ptA^.z);
  glVertex3d(ptB^.x,ptB^.y,ptB^.z);
  glEnd();
  writeln('addSingleResult : PointA = ',ptA^.DumpValues, ' PointB = ',ptB^.DumpValues);
  Result := 0;
end;
{$HINTS ON}


constructor btContinuousConvexCollisionDemo.Create(const param:string);
begin
  inherited Create;
  f_param   := param;
  minlines  := 0;
  maxlines  := 512;
  drawline  := true;
  gGjkSimplexSolver := btVoronoiSimplexSolver.Create;
//  renderCallback:=btDrawingResult.Create;
end;

destructor btContinuousConvexCollisionDemo.Destroy;
begin
//  renderCallback.Free;
  gGjkSimplexSolver.Free;
  if f_initialized then cleanPhysics;
  inherited Destroy;
end;

procedure btContinuousConvexCollisionDemo.initPhysics;
var basisA                    : btMatrix3x3;
    //worldAabbMin,worldAabbMax : btVector3;
    identBasis                : btMatrix3x3;
    boxHalfExtentsA,
    boxHalfExtentsB           : btVector3;
    i: Integer;
begin
  f_initialized:=true;

  fromTrans[0].setOrigin(btVector3.InitS(0,10,20));
    toTrans[0].setOrigin(btVector3.InitS(0,10,-20));
  fromTrans[1].setOrigin(btVector3.InitS(-2,7,0));
    toTrans[1].setOrigin(btVector3.InitS(-2,10,0));

  identBasis.setIdentity;

  basisA.setIdentity;
  basisA.setEulerZYX(0,-SIMD_HALF_PI,0);

  fromTrans[0].setBasis(identBasis);
    toTrans[0].setBasis(basisA);

  fromTrans[1].setBasis(identBasis);
    toTrans[1].setBasis(identBasis);

  toTrans[1].setBasis(identBasis);
  boxHalfExtentsA.init(10,1,1);
  boxHalfExtentsB.init(1.1,1.1,1.1);
  boxA := btBoxShape.Create(boxHalfExtentsA);
  boxB := btBoxShape.Create(boxHalfExtentsB);
  //    btBU_Simplex1to4* boxA = new btBU_Simplex1to4(btVector3(-2,0,-2),btVector3(2,0,-2),btVector3(0,0,2),btVector3(0,2,0));
  //    btBU_Simplex1to4* boxA = new btBU_Simplex1to4(btVector3(-12,0,0),btVector3(12,0,0));

  shapePtr[0] := boxA;
  shapePtr[1] := boxB;

  shapePtr[0].setMargin(0.01);
  shapePtr[1].setMargin(0.01);

  for i:=0 to numObjects-1 do begin
    btTransformUtil.calculateVelocity(fromTrans[i],toTrans[i],1,linVels[i],angVels[i]);
  end;
end;

procedure btContinuousConvexCollisionDemo.cleanPhysics;
begin
  f_initialized:=false;
  boxA.free;
  boxB.free;
  //collisionWorld.free;
  //collisionConfiguration.Free;
  //dispatcher.free;
  //broadphase.free;
end;

procedure btContinuousConvexCollisionDemo.clientMoveAndDisplay;
begin
  displayCallback;
end;

procedure btContinuousConvexCollisionDemo.displayCallback;
var m                             : array [0..15] of btScalar;
    worldBoundsMin,worldBoundsMax : btVector3;
    //numManifolds,numContacts      : integer;
    i                             : integer;
    //contactManifold               : btPersistentManifold;
    //obA,obB                       : btCollisionObject;
//    pt                            : btOManifoldPoint;
    //ptA,ptB                       : btVector3;
    //qA,qB,
    orn                           : btQuaternion;
    //timeInSeconds                 : btScalar;
    numSubSteps                   : integer;
    s                             : integer;
    subStep                       : btScalar;
    interpolatedTrans             : btTransform;
    mat                           : btMatrix3x3;
    rayFromWorld,rayToWorld       : btTransform;
    rayResult1                    : btDebugCastResult;
    rayResultPtr                  : btConvexCastResult;
    penetrationDepthSolver        : btConvexPenetrationDepthSolver;
    convexCaster                  :  btContinuousConvexCollision;
    hitTrans                      : btTransform;

begin
  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);
      glDisable(GL_LIGHTING);
//      GL_ShapeDrawer.drawCoordSystem;
      worldBoundsMin.Init(-1000,-1000,-1000);
      worldBoundsMax.Init(1000,1000,1000);
      include(m_debugMode,DBG_DrawAabb);
      if DBG_DrawAabb in getDebugmode then begin
      	i:=0;//for (i=1;i<numObjects;i++)
          //for each object, subdivide the from/to transform in 10 equal steps
          numSubSteps := 10;
          for s:=0 to 9 do begin
            subStep := s * 1/numSubSteps;
            btTransformUtil.integrateTransform(fromTrans[i],linVels[i],angVels[i],subStep,interpolatedTrans);
            //fromTrans[i].getOpenGLMatrix(m);
            //m_shapeDrawer.drawOpenGL(m,shapePtr[i]);
            //toTrans[i].getOpenGLMatrix(m);
            //m_shapeDrawer.drawOpenGL(m,shapePtr[i]);
            interpolatedTrans.getOpenGLMatrix( m );
            m_shapeDrawer.drawOpenGL(m,shapePtr[i],btVector3.inits(1,0,1),getDebugMode,worldBoundsMin,worldBoundsMax);
      	  end;
      end;

      mat.setEulerZYX(yaw,pitch,roll);
      mat.getRotation(orn);
      orn.setEuler(yaw,pitch,roll);
      fromTrans[1].setRotation(orn);
      toTrans[1].setRotation(orn);


      if m_stepping or m_singleStep then begin
      	m_singleStep := false;
      	pitch += 0.005;
//		yaw += 0.01f;
      end;
//	btVector3 fromA(-25,11,0);
//	btVector3 toA(-15,11,0);

//	btQuaternion ornFromA(0.f,0.f,0.f,1.f);
//	btQuaternion ornToA(0.f,0.f,0.f,1.f);

//	btTransform	rayFromWorld(ornFromA,fromA);
//	btTransform	rayToWorld(ornToA,toA);

      rayFromWorld := fromTrans[0];
      rayToWorld   := toTrans[0];


      if drawLine then begin
      	glBegin(GL_LINES);
      	glColor3f(0, 0, 1);
      	glVertex3d(rayFromWorld.getOriginV^.x, rayFromWorld.getOriginV^.y,rayFromWorld.getOriginV^.z);
      	glVertex3d(rayToWorld.getOriginV^.x,rayToWorld.getOriginV^.y,rayToWorld.getOriginV^.z);
      	glEnd();
      end;

      //now perform a raycast on the shapes, in local (shape) space
      gGjkSimplexSolver.reset();

      //choose one of the following lines


      for i := 0 to numObjects-1 do begin
      	fromTrans[i].getOpenGLMatrix(m);
      	m_shapeDrawer.drawOpenGL(m,shapePtr[i],btVector3.InitS(1,1,1),getDebugMode,worldBoundsMin,worldBoundsMax);
      end;

      rayResult1:=btDebugCastResult.create(fromTrans[0],shapePtr[0],linVels[0],angVels[0],m_shapeDrawer);

      for i:=1 to numObjects-1 do begin
        rayResultPtr := rayResult1;
      	//GjkConvexCast	convexCaster(&gGjkSimplexSolver);
      	//SubsimplexConvexCast convexCaster(&gGjkSimplexSolver);

      	//optional
        penetrationDepthSolver := btConvexPenetrationDepthSolver.Create;
        convexCaster := btContinuousConvexCollision.create(shapePtr[0],shapePtr[i],gGjkSimplexSolver,penetrationDepthSolver);
        gGjkSimplexSolver.reset;
        if (convexCaster.calcTimeOfImpact(fromTrans[0],toTrans[0],fromTrans[i] ,toTrans[i] ,rayResultPtr)) then begin
          glDisable(GL_DEPTH_TEST);
          btTransformUtil.integrateTransform(fromTrans[0],linVels[0],angVels[0],rayResultPtr.m_fraction,hitTrans);
          hitTrans.getOpenGLMatrix(m);
          m_shapeDrawer.drawOpenGL(m,shapePtr[0],btVector3.inits(0,1,0),getDebugMode(),worldBoundsMin,worldBoundsMax);
          btTransformUtil.integrateTransform(fromTrans[i],linVels[i],angVels[i],rayResultPtr.m_fraction,hitTrans);
          hitTrans.getOpenGLMatrix(m);
          m_shapeDrawer.drawOpenGL(m,shapePtr[i],btVector3.inits(0,1,1),getDebugMode(),worldBoundsMin,worldBoundsMax);
        end;
        convexCaster.Free;
        penetrationDepthSolver.FRee;
      end;

      rayResult1.Free;
  swapBuffers();
end;

end.

