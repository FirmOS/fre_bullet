unit BasicDemo;

{$i ../fos_bullet.inc}

interface

uses
  Classes, SysUtils,btCollisionShapes,btDispatch,btNarrowphase,btDynamics,FOS_AlignedArray,btBroadphase,
  btLinearMath,glutstuff,gl,glu,glut;


///create 125 (5x5x5) dynamic object
const ARRAY_SIZE_X = 5  ;
      ARRAY_SIZE_Y = 5 ;
      ARRAY_SIZE_Z = 5  ;
      MAX_PROXIES = (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024); //maximum number of objects (and allow user to shoot additional boxes)
      ///scaling of the objects (0.1 = 20 centimeter boxes )
      SCALING     =   1;
      START_POS_X =  -5; // -5
      START_POS_Y =  -5; // -5
      START_POS_Z =  -3; // -3

type
    btCollisionShapeArray = specialize FOS_GenericAlignedArray<btCollisionShape>;

    { btBasicDemo }

    btBasicDemo=class(btGlutDemoApplication)
        //keep the collision shapes, for deletion/cleanup
      m_collisionShapes : btCollisionShapeArray;
      m_broadphase      : btBroadphaseInterface;
      m_dispatcher      : btCollisionDispatcher;
      m_solver          : btConstraintSolver;
      m_collisionConfiguration : btDefaultCollisionConfiguration;
      pc                : btOverlappingPairCache;
      debugDrawer       : btGLDebugDrawer;
    public
      constructor create;
      destructor  Destroy;override;
      procedure   exitPhysics;
      procedure   initPhysics;
      procedure   clientMoveAndDisplay; override;
      procedure   displayCallback; override;
    end;

implementation


{ btBasicDemo }

constructor btBasicDemo.create;
begin
  inherited create;
  debugDrawer := btGLDebugDrawer.create;
  m_collisionShapes := btCollisionShapeArray.create;
  m_collisionShapes.Initialize(btCollisionShape);
end;

destructor btBasicDemo.Destroy;
begin
  exitPhysics;
  debugDrawer.Free;
  m_collisionShapes.Free;
  inherited Destroy;
end;

procedure btBasicDemo.exitPhysics;
var
  i: Integer;
  obj:btCollisionObject;
  body:btRigidBody;
begin
  //cleanup in the reverse order of creation/initialization
  //remove the rigidbodies from the dynamics world and delete them
  for i  := m_dynamicsWorld.getNumCollisionObjects-1 downto 0 do begin
    obj  := m_dynamicsWorld.getCollisionObjectArray[i]^;
    body := btRigidBody.upcast(obj);
    if assigned(body) and assigned(body.getMotionState) then begin
      body.getMotionState.Free;
    end;
    m_dynamicsWorld.removeCollisionObject(obj);
    obj.Free;
  end;
  //delete collision shapes
  for i:=0 to m_collisionShapes.Size-1 do begin
    m_collisionShapes[i]^.free;
  end;
  m_dynamicsWorld.Free;
  m_solver.Free;
  m_broadphase.Free;
  m_dispatcher.Free;
  m_collisionConfiguration.Free;
//  pc.Free;
end;


procedure btBasicDemo.initPhysics;
var groundShape     : btCollisionShape;
    groundTransform : btTransform;
    mass            : btScalar;
    isDynamic       : boolean;
    localInertia    : btVector3;
    myMotionState   : btDefaultMotionState;
    rbInfo          : btRigidBodyConstructionInfo;
    body            : btRigidBody;
    colShape        : btCollisionShape;
    startTransform  : btTransform;
    start_x,start_y,
    start_z         : btScalar;
    k,i,j           : integer;
    useridx         : integer;

begin
  setTexturing(true);
  setShadows(true);
  setCameraDistance(SCALING*50);

  ///collision configuration contains default setup for memory, collision setup
  m_collisionConfiguration := btDefaultCollisionConfiguration.Create(btDefaultCollisionConstructionInfo.Init);
  //m_collisionConfiguration->setConvexConvexMultipointIterations();

  ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
  m_dispatcher :=  btCollisionDispatcher.Create(m_collisionConfiguration);

//  pc           := btSortedOverlappingPairCache.Create;
//  pc           := btHashedOverlappingPairCache.Create;
//  m_broadphase := btSimpleBroadphase.Create(16384,pc);
  m_broadphase :=  btDbvtBroadphase.Create;

  ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
  m_solver     :=  btSequentialImpulseConstraintSolver.create;
  m_dynamicsWorld := btDiscreteDynamicsWorld.create(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
  //m_dynamicsWorld.getSolverInfo.m_numIterations:=10;
  //  m_dynamicsWorld.setGravity(btVector3.InitS(0,-10,0));
  m_dynamicsWorld.setGravity(btVector3.InitS(0,-10,0));

  ///create a few basic rigid bodies
  groundShape := btBoxShape.Create(btVector3.InitSameS(50));
//      btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
  m_collisionShapes.push_back(groundShape);

  groundTransform.setIdentity;
  groundTransform.setOrigin(btVector3.inits(0,-50,0));
  //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
  mass := 0;
  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  isDynamic := mass <> 0;
  localInertia.zero;
  if isDynamic then begin
    groundShape.calculateLocalInertia(mass,localInertia);
  end;

  //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
  myMotionState := btDefaultMotionState.create(groundTransform);
  rbInfo.init(mass,myMotionState,groundShape,localInertia);
  body := btRigidBody.create(rbInfo);
  body.setUserPointer(pointer(1));

  //add the body to the dynamics world
  m_dynamicsWorld.addRigidBody(body);

  //create a few dynamic rigidbodies
  // Re-using the same collision is better for memory usage and performance

  colShape := btBoxShape.Create(btVector3.InitSameS(SCALING*1));
  //btCollisionShape* colShape = new btSphereShape(btScalar(1.));
  m_collisionShapes.push_back(colShape);

  /// Create Dynamic Objects
  startTransform.setIdentity();

  mass := 1;
  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  isDynamic := mass <> 0;
  localInertia.zero;
  if isDynamic then begin
    colShape.calculateLocalInertia(mass,localInertia);
  end;
  start_x := START_POS_X - ARRAY_SIZE_X div 2;
  start_y := START_POS_Y;
  start_z := START_POS_Z - ARRAY_SIZE_Z div 2;
  useridx := 11;
  for k :=0 to ARRAY_SIZE_Y-1 do begin
    for i:= 0 to ARRAY_SIZE_X-1 do begin
      for j := 0 to ARRAY_SIZE_Z-1 do begin
        startTransform.setOrigin(SCALING*btVector3.InitS(2*i+start_x,20+2.0*k + start_y,2.0*j + start_z));
        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        myMotionState := btDefaultMotionState.create(startTransform);
        rbInfo.init(mass,myMotionState,colShape,localInertia);
        body := btRigidBody.create(rbInfo);
        body.setUserPointer(pointer(useridx));
        inc(useridx);
        m_dynamicsWorld.addRigidBody(body);
        body.setActivationState(btas_ISLAND_SLEEPING);
      end;
    end;
  end;
  clientResetScene;
//  debugDrawer.setDebugMode([DBG_NoDeactivation]);
end;

procedure btBasicDemo.clientMoveAndDisplay;
var ms : btScalar;
begin
  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);

  //simple dynamics world doesn't handle fixed-time-stepping
  ms := getDeltaTimeMicroseconds();
  ms := 1000000;
  ///step the simulation
  gFOS_Debug_Step:=false;
  if assigned(m_dynamicsWorld) then begin
    gFOS_DEbug_SIMUSTEP:=gFOS_DEbug_SIMUSTEP+1;
   // writeln('STEP-',gFOS_DEbug_SIMUSTEP);
    if gFOS_Debug_SIMUSTEP=878 then begin
      //gFOS_Debug_Step     := true;
     // abort;
     // readln;
    end;
    m_dynamicsWorld.stepSimulation(ms / 1000000);
    m_dynamicsWorld.debugDrawWorld;
    //sleep(0);
  end;
  renderme;
  glFlush;
  swapBuffers;
end;

procedure btBasicDemo.displayCallback;
begin
  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);
  renderme;
  //optional but useful: debug drawing to detect problems
  if assigned(m_dynamicsWorld) then begin
    m_dynamicsWorld.debugDrawWorld;
  end;
  glFlush;
  swapBuffers;
end;



end.

