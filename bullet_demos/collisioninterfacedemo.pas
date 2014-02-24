unit CollisionInterfaceDemo;

interface

{$i ../fos_bullet.inc}

uses
  Classes, SysUtils,gl,glut,glutstuff,btCollisionShapes,btDispatch,btBroadphase,btNarrowphase,btLinearMath;

const  maxNumObjects = 4;
          numObjects = 2;

  type
    PlatformDemoApplication = btGlutDemoApplication;

    { btCollisionInterfaceDemo }
    btCollisionInterfaceDemo=class(PlatformDemoApplication)
    private
      f_initialized          : boolean;
      f_param                : string;
      renderCallback         : Tobject;
      boxA,boxB              : btBoxShape;
      collisionConfiguration : btDefaultCollisionConfiguration;
      dispatcher             : btDispatcher;
      broadphase             : btBroadphaseInterface;
    public
      constructor Create(const param:string);
      destructor  Destroy;override;
      procedure initPhysics;override;
      procedure cleanPhysics;
      procedure clientMoveAndDisplay; override;
      procedure displayCallback; override;
      procedure clientResetScene; override;
  end;



implementation

var yaw,pitch,roll : btScalar;
     objects       : array [0..maxNumObjects-1] of btCollisionObject;
     collisionWorld : btCollisionWorld;
 //    simplex        : GL_Simplex1to4;
     debugDrawer    : btGLDebugDrawer;

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


constructor btCollisionInterfaceDemo.Create(const param:string);
var i:integer;
begin
  inherited Create;
  for i := 0 to high(objects) do begin
    objects[i] := btCollisionObject.Create;
  end;
  debugDrawer := btGLDebugDrawer.create;
  f_param:=param;
  renderCallback:=btDrawingResult.Create;
end;

destructor btCollisionInterfaceDemo.Destroy;
var
  i: Integer;
begin
  renderCallback.Free;
  if f_initialized then cleanPhysics;
  for i := 0 to high(objects) do begin
    objects[i].free;
  end;
  debugDrawer.Free;
  inherited Destroy;
end;

procedure btCollisionInterfaceDemo.initPhysics;
var basisA,basisB : btMatrix3x3;
    worldAabbMin,worldAabbMax : btVector3;
begin
  worldAabbMin.init(-1000,-1000,-1000);
  worldAabbMax.init(1000,1000,1000);
  if f_param='' then begin
    writeln('no param - using "simple"');
    f_param:='simple';
  end;
  writeln('PARAM = ',f_param);
  //DEBUG
  //  f_param:='axis';
  //DEBUG end

  case lowercase(f_param) of
    'axis' : begin
              // broadphase     := btAxisSweep3.Create(worldAabbMin,worldAabbMax);
              abort; // Blease Check;
             end;
    'simple' : begin
               broadphase     := btSimpleBroadphase.Create;
             end;
    'dbvt'   : begin
               broadphase     := btDbvtBroadphase.Create;
             end;
    else begin
      writeln('you must use "axis", "simple" or "dbvt" parameter for the demo');
      halt;
    end;
  end;

  f_initialized:=true;

  include(m_debugMode,DBG_DrawWireframe);
  basisA.setIdentity;
  basisB.setIdentity;

  objects[0].getWorldTransformP^.setBasis(basisA);
  objects[1].getWorldTransformP^.setBasis(basisB);

  boxA := btBoxShape.Create(btVector3.InitSameS(1));
  boxA.setMargin(0);
  boxB := btBoxShape.Create(btVector3.InitSameS(0.5));
  boxB.setMargin(0);
  //ConvexHullShape	hullA(points0,3);
  //hullA.setLocalScaling(btVector3(3,3,3));
  //ConvexHullShape	hullB(points1,4);
  //hullB.setLocalScaling(btVector3(4,4,4));

  objects[0].setCollisionShape(boxA);//&hullA;
  objects[1].setCollisionShape(boxB);//&hullB;


  collisionConfiguration := btDefaultCollisionConfiguration.Create(btDefaultCollisionConstructionInfo.Init);
  dispatcher             := btCollisionDispatcher.Create(collisionConfiguration);

  collisionWorld := btCollisionWorld.create(dispatcher,broadphase,collisionConfiguration);
  collisionWorld.setDebugDrawer(debugDrawer);

{$ifdef TEST_NOT_ADDING_OBJECTS_TO_WORLD}
//      collisionWorld->addCollisionObject(&objects[0]);
  collisionWorld.addCollisionObject(objects[1]);
{$endif //TEST_NOT_ADDING_OBJECTS_TO_WORLD}
end;

procedure btCollisionInterfaceDemo.cleanPhysics;
begin
  f_initialized:=false;
  boxA.free;
  boxB.free;
  collisionWorld.free;
  collisionConfiguration.Free;
  dispatcher.free;
  broadphase.free;
end;

procedure btCollisionInterfaceDemo.clientMoveAndDisplay;
begin
  displayCallback;
end;

procedure btCollisionInterfaceDemo.displayCallback;
var m                             : array [0..15] of btScalar;
    worldBoundsMin,worldBoundsMax : btVector3;
    i                             : integer;
    //numManifolds,numContacts,j  : integer;
    //contactManifold               : btPersistentManifold;
    //obA,obB                       : btCollisionObject;
    ////pt                            : PbtManifoldPoint;
    //ptA,ptB                       : btVector3;
//    qA,qB,
//      orn                     : btQuaternion;
    timeInSeconds                 : btScalar;

    procedure _Query;
    var algo               : btCollisionAlgorithm;
        contactPointResult : btManifoldResult;
        manifoldArray      : btManifoldArray ;
        i,j,numManifolds   : integer;
        contactManifold    : btPersistentManifold;
        obA                : btCollisionObject;
        numContacts        : integer;
        swap               : boolean;
        pt                 : PbtOManifoldPoint;
        ptA,ptB            : PbtVector3;
    begin
      manifoldArray := btManifoldArray.create;
      manifoldArray.Initialize(btPersistentManifold);
      //another way is to directly query the dispatcher for both objects. The objects don't need to be inserted into the world
     algo := collisionWorld.getDispatcher.findAlgorithm(objects[0],objects[1]);
     contactPointResult := btManifoldResult.Create(objects[0],objects[1]);
     //  contactPointResult(&objects[0],&objects[1]);
     algo.processCollision(objects[0],objects[1],collisionWorld.getDispatchInfo,contactPointResult);
     algo.getAllContactManifolds(manifoldArray);
     numManifolds := manifoldArray.size;
     for i :=0 to numManifolds-1 do begin
        contactManifold := manifoldArray[i]^;
        obA := btCollisionObject(contactManifold.getBody0);
      //        btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
        glDisable(GL_DEPTH_TEST);
        numContacts := contactManifold.getNumContacts;
        swap := obA = objects[0];
        for j := 0 to numContacts-1 do begin;
          pt := contactManifold.getContactPointP(j);
          glBegin(GL_LINES);
          glColor3f(0, 0, 0);
          ptA :=  btDecide(swap,pt^.getPositionWorldOnAP,pt^.getPositionWorldOnBP);
          ptB :=  btDecide(swap,pt^.getPositionWorldOnBP,pt^.getPositionWorldOnAP);
          glVertex3d(ptA^.x,ptA^.y,ptA^.z);
          glVertex3d(ptB^.x,ptB^.y,ptB^.z);
          glEnd();
        end;
        //you can un-comment out this line, and then all points are removed
        //contactManifold->clearManifold();
      end;
     manifoldArray.Clear;
     algo.free;
     contactPointResult.free;
    end;

begin
  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);
      glDisable(GL_LIGHTING);
      collisionWorld.getBroadphase.getBroadphaseAabb(worldBoundsMin,worldBoundsMax);
      for i:=0 to numObjects-1 do begin
        objects[i].getWorldTransformP^.getOpenGLMatrix( m );
        m_shapeDrawer.drawOpenGL(m,objects[i].getCollisionShape,btVector3.InitSameS(1),getDebugMode,worldBoundsMin,worldBoundsMax);
      end;
      collisionWorld.getDispatchInfoP^.m_debugDraw := debugDrawer;

      if assigned(collisionWorld) then begin
        collisionWorld.performDiscreteCollisionDetection;
      end;

{$ifndef TEST_NOT_ADDING_OBJECTS_TO_WORLD}
      collisionWorld.debugDrawWorld();
      ///one way to draw all the contact points is iterating over contact manifolds in the dispatcher:
      numManifolds := collisionWorld.getDispatcher.getNumManifolds;
      for i :=0 to numManifolds-1 do begin
        contactManifold := collisionWorld.getDispatcher.getManifoldByIndexInternal(i);
        obA := btCollisionObject(contactManifold.getBody0);
        obB := btCollisionObject(contactManifold.getBody1);
        numContacts := contactManifold.getNumContacts;
        for j := 0 to numContacts-1 do begin
          pt := contactManifold.getContactPointP(j);
          glBegin(GL_LINES);
          glColor3f(0, 0, 0);
          ptA := pt^.getPositionWorldOnA();
          ptB := pt^.getPositionWorldOnB();
          glVertex3d(ptA.x(),ptA.y(),ptA.z());
          glVertex3d(ptB.x(),ptB.y(),ptB.z());
          glEnd();
        end;
        //you can un-comment out this line, and then all points are removed
        //contactManifold->clearManifold();
      end;
{$else}
      glDisable(GL_TEXTURE_2D);
      for i:=0 to numObjects-1 do begin
        collisionWorld.debugDrawObject(objects[i].getWorldTransformP^,objects[i].getCollisionShape(), btVector3.InitS(1,1,0));
      end;
//      collisionWorld.contactPairTest(objects[0],objects[1],btCW_ContactResultCallback(renderCallback));
      collisionWorld.contactTest(objects[0],btCW_ContactResultCallback(renderCallback));
    {$if 1}
    //_Query;
    {$endif}
{$endif}

      btGL_ShapeDrawer.drawCoordSystem;

      //qA := objects[0].getWorldTransformP^.getRotation;
      //qB := objects[1].getWorldTransformP^.getRotation;

      if true and not m_idle then begin
        timeInSeconds := getDeltaTimeMicroseconds/1000;
        timeInSeconds := 60;
        writeln(fosdebug_gcount);
        inc(fosdebug_gcount);
        objects[0].getWorldTransformP^.GetBasisV^.getEulerYPR(yaw,pitch,roll);
        pitch := pitch + (0.00005 * timeInSeconds);
        yaw   := yaw +   (0.0001  * timeInSeconds);
        //writeln(timeInSeconds:2:2,' y= ',yaw:2:2,' p=',pitch:2:2);
        objects[0].getWorldTransformP^.getBasisV^.setEulerYPR(yaw,pitch,roll);
        objects[1].getWorldTransformP^.setOrigin(objects[1].getWorldTransformP^.getOriginV^+btVector3.InitS(0,-0.0001*timeInSeconds,0));
        //orn.setEuler(yaw,pitch,roll);
        //objects[0].getWorldTransformP^.setRotation(orn);
      end;
      glFlush();
  swapBuffers();
end;

procedure btCollisionInterfaceDemo.clientResetScene;
var rotA:btQuaternion;
begin
  //objects[0].getWorldTransform.setOrigin(btVector3.InitS(0,3,0));
  //rotA.Init4S(0.739,-0.204,0.587,0.257);
  //rotA.normalize;
  //objects[0].getWorldTransform.setRotation(rotA);
  //objects[1].getWorldTransform.setOrigin(btVector3.inits(0.0,4.248,0));
  objects[0].getWorldTransformP^.setOrigin(btVector3.InitS(0,3,0));
  rotA.Init4(0.739,-0.204,0.587,0.257);
  rotA.normalize;
  objects[0].getWorldTransformP^.setRotation(rotA);
  objects[1].getWorldTransformP^.setOrigin(btVector3.inits(0.0,4.248,0));
  fosdebug_gcount:=0;
end;



end.

