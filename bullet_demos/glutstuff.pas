unit glutstuff;

interface

{$i ../fos_bullet.inc}

uses  Classes, SysUtils, GL, GLU, GLUT,gldebugfont,btCollisionShapes,btNarrowphase,btBroadphase,btDispatch,btDynamics,btLinearMath,FOS_AlignedArray;

{$WARNINGS OFF}
{$HINTS OFF}

  var use6Dof : boolean = false;
{$define SHOW_NUM_DEEP_PENETRATIONS:=1}
  gNumClampedCcdMotions : integer=0;
{$ifdef SHOW_NUM_DEEP_PENETRATIONS}
  gNumDeepPenetrationChecks,
  gNumSplitImpulseRecoveries,
  gNumGjkChecks,
  gNumAlignedAllocs,
  gNumAlignedFree,
  gTotalBytesAlignedAllocs : integer;
{$endif}

const
  BT_KEY_LEFT  = GLUT_KEY_LEFT;
  BT_KEY_RIGHT = GLUT_KEY_RIGHT;
  BT_KEY_UP    = GLUT_KEY_UP;
  BT_KEY_DOWN  = GLUT_KEY_DOWN;
  BT_KEY_F1    = GLUT_KEY_F1;
  BT_KEY_F2    = GLUT_KEY_F2;
  BT_KEY_F3    = GLUT_KEY_F3;
  BT_KEY_F4    = GLUT_KEY_F4;
  BT_KEY_F5    = GLUT_KEY_F5;
  BT_KEY_PAGEUP   = GLUT_KEY_PAGE_UP;
  BT_KEY_PAGEDOWN = GLUT_KEY_PAGE_DOWN;
  BT_KEY_END      = GLUT_KEY_END;
  BT_KEY_HOME     = GLUT_KEY_HOME;
  BT_ACTIVE_ALT   = GLUT_ACTIVE_ALT;
  BT_ACTIVE_CTRL  = GLUT_ACTIVE_ALT;
  BT_ACTIVE_SHIFT = GLUT_ACTIVE_SHIFT;

  //class DemoApplication;
  //int glutmain(int argc, char **argv,int width,int height,const char* title,DemoApplication* demoApp);
type

  /// OpenGL shape drawing
  btGLSC_Edge=record
    n : array [0..1] of btVector3;
    v : array [0..1] of integer;
  end;
  PbtGLSC_Edge=^btGLSC_Edge;

  btGLSC_EdgeArray  = specialize FOS_GenericAlignedArray<btGLSC_Edge>;
  btGLSC_EdgeArrayP = specialize FOS_GenericAlignedArray<PbtGLSC_Edge>;

  { btGL_ShapeCache }

  btGL_ShapeCache=class
    m_shapehull : btShapeHull;
    m_edges     : btGLSC_EdgeArray;
    constructor  Create(const s:btConvexShape);
    destructor   Destroy;override;
  end;
  btGL_ShapeCacheArray = specialize FOS_GenericAlignedArray<btGL_ShapeCache>;


  { btGL_ShapeDrawer }

//  void OGL_displaylist_register_shape(btCollisionShape * shape);
//  void OGL_displaylist_clean();

  btGL_ShapeDrawer = class
  protected
    //clean-up memory of dynamically created shape hulls
    m_shapecaches    : btGL_ShapeCacheArray;
    m_texturehandle  : cardinal;
    m_textureenabled : boolean;
    m_textureinitialized : boolean;
    function  cache (const shape:btConvexShape) : btGL_ShapeCache;
  public
    constructor create;
    destructor  destroy;override;
    ///drawOpenGL might allocate temporary memoty, stores pointer in shape userpointer
    procedure    drawOpenGL (const m : PbtScalar; const shape : btCollisionShape ; const color : btVector3 ; const debugMode : TbtIDEBUGFlagsSet;const worldBoundsMin,worldBoundsMax : btVector3); virtual;
    procedure    drawShadow (const m : PbtScalar; const extrusion : btVector3 ; const  shape : btCollisionShape ;const worldBoundsMin,worldBoundsMax : btVector3); virtual;
    function     enableTexture(const enable : boolean) : boolean;
    function     hasTextureEnabled : boolean;
    class procedure   drawCylinder(const  radius,halfHeight : btScalar ; const  upAxis : integer);
    class procedure   drawSphere  (const  radius:btScalar; const  lats, longs : integer);
    class procedure   drawCoordSystem;
  end;

  const STEPSIZE:btScalar = 5;
type
  btDemoApplication = class
  protected
    //    class CProfileIterator* m_profileIterator;
   {$ifdef USE_BT_CLOCK}
    m_clock          : btClock;
   {$endif} //USE_BT_CLOCK
    ///this is the most important class
    m_dynamicsWorld  : btDynamicsWorld;
    m_pickConstraint : btTypedConstraint;
    m_shootBoxShape  : btCollisionShape;
    m_cameraDistance : btScalar;
    m_debugMode      : TbtIDEBUGFlagsSet;
    m_ele            : btScalar;
    m_azi            : btScalar;
    m_cameraPosition : btVector3 ;
    m_cameraTargetPosition : btVector3 ;//look at
    m_mouseOldX,
    m_mouseOldY,
    m_mouseButtons   : integer;
    procedure displayProfileString(const xOffset,yStart:integer; const message:String);
  public
    m_modifierKeys   : integer;
  protected
    m_scaleBottom    : btScalar;
    m_scaleFactor    : btScalar;
    m_cameraUp       : btVector3;
    m_forwardAxis    : integer;
    m_glutScreenWidth: integer;
    m_glutScreenHeight: integer;
    m_frustumZNear,
    m_frustumZFar    : btScalar;
    m_ortho          : boolean;
    m_ShootBoxInitialSpeed : btScalar;
    m_stepping,
    m_singleStep,
    m_idle           : boolean;
    m_lastKey        : integer;
    m_shapeDrawer    : btGL_ShapeDrawer;
    m_enableshadows  : boolean;
    m_sundirection   : btVector3;
    m_defaultContactProcessingThreshold : btScalar;
    procedure  showProfileInfo(const xOffset,yStart, yIncr : integer);
    procedure  renderscene(const pass : integer);
  public
    constructor create;
    destructor  destroy;override;
    function    getDynamicsWorld           : btDynamicsWorld;
    procedure   initPhysics                ; virtual;abstract;
    procedure   setDrawClusters            (const drawClusters:boolean);virtual;
    procedure   overrideGLShapeDrawer      (const shapeDrawer : btGL_ShapeDrawer);
    procedure   setOrthographicProjection  ;
    procedure   resetPerspectiveProjection ;
    function    setTexturing               (const enable : boolean):boolean;
    function    setShadows                 (const  enable : boolean):boolean;
    function    getTexturing               :boolean;
    function    getShadows                 :boolean;
    function    getDebugmode               :TbtIDEBUGFlagsSet;
    procedure   setdebugmode               (const mode:TbtIDEBUGFlagsSet);
    procedure   setAzi                     (const azi:btScalar);
    procedure   setCameraUp                (const camUp : btVector3);
    procedure   setCameraForwardAxis       (const axis  : integer);
    procedure   myinit                     ; virtual;
    procedure   toggleIdle                 ;
    procedure   updateCamera               ; virtual;
    function    getCameraPosition          : btVector3;
    function    getCameraTargetPosition    : btVector3;
    function    getDeltaTimeMicroseconds   : btScalar;
    procedure   setFrustumZPlanes          (const zNear, zFar : btScalar);
    ///glut callbacks
    function    getCameraDistance          : btScalar;
    procedure   setCameraDistance          (const dist:btScalar);
    procedure   moveAndDisplay             ;
    procedure   clientMoveAndDisplay       ;virtual;abstract;
    procedure   clientResetScene           ;virtual;
    ///Demo functions
    procedure   setShootBoxShape           ;virtual;
    procedure   shootBox                   (const destination : btVector3);virtual;
    function    getRayTo                   (const x,y : integer) : btVector3;
    function    localCreateRigidBody       (const mass : btScalar; const startTransform : btTransform ;const shape : btCollisionShape):btRigidBody;
    ///callback methods by glut
    procedure   keyboardCallback           (key : cardinal; x, y : integer);virtual;
    procedure   keyboardUpCallback         (key : cardinal; x, y : integer);virtual;
    procedure   specialKeyboard            (key : integer ; x, y : integer);virtual;
    procedure   specialKeyboardUp          (key : integer ; x, y : integer);virtual;
    procedure   reshape                    (w,h : integer);virtual;
    procedure   mouseFunc                  (button, state, x, y : integer);virtual;
    procedure   mouseMotionFunc            (x,y : integer); virtual;
    procedure   displayCallback            ;virtual;
    procedure   renderme                   ;virtual;
    procedure   swapBuffers                ;virtual;abstract;
    procedure   updateModifierKeys         ;virtual;abstract;
    procedure   stepLeft                   ;
    procedure   stepRight                  ;
    procedure   stepFront                  ;
    procedure   stepBack                   ;
    procedure   zoomIn                     ;
    procedure   zoomOut                    ;
    function    isIdle                     : boolean;
    procedure   setIdle                    (const idle:boolean);
  end;

  { btGlutDemoApplication }

  btGlutDemoApplication =class(btDemoApplication)
  private
    myID:string;
  public
    constructor Create;
    procedure specialKeyboard(key: integer; x, y: integer); override;
    procedure swapBuffers; override;
    procedure updateModifierKeys; override;
  end;

  { btGLDebugDrawer }

  btGLDebugDrawer = class(btIDebugDraw)
    m_debugMode : TbtIDEBUGFlagsSet;
  public
    constructor create;
    procedure drawLine(const from, too, fromColor, toColor: btVector3); override;
    procedure drawLine(const from, too, color: btVector3); override;
    procedure drawSphere(const p: btVector3; const radius: btScalar; const color: btVector3); override;
    procedure drawBox(const bbMin, bbMax, color: btVector3; const alpha: btScalar); override;
    procedure drawTriangle(const a, b, c, color: btVector3; const alpha: btScalar); override;
    procedure drawContactPoint(const PointOnB, normalOnB: btVector3; const distance: btScalar; const lifeTime: integer; const color: btVector3); override;
    procedure reportErrorWarning(const warningString: string); override;
    procedure draw3dText(const location: btVector3; const textString: string); override;
    procedure setDebugMode(const debugMode: TbtIDEBUGFlagsSet); override;
    function  getDebugMode: TbtIDEBUGFlagsSet; override;
  end;


  ///GL_Simplex1to4 is a class to debug a Simplex Solver with 1 to 4 points.
  ///Can be used by GJK.

  { GL_Simplex1to4 }

  GL_Simplex1to4 = class(btBU_Simplex1to4)
    m_simplexSolver : btSimplexSolverInterface;
  public
    procedure    calcClosest(const m:PbtScalar);
    procedure    setSimplexSolver(const simplexSolver : btSimplexSolverInterface);
  end;

  ///
  ///renderTexture provides a software-render context (setpixel/printf)
  ///

  { btRenderTexture }

  btRenderTexture=class
    m_height,
    m_width     : integer;
    m_buffer    : PByte;
  public
   constructor create(const width,height : integer);
   destructor  Destroy;override;
   ///rgba input is in range [0..1] for each component
   procedure   setPixel(const x,y : integer ; const rgba : btVector4);
   procedure   addPixel(const x,y : integer ; const rgba : btVector4);
   function    getPixel(const x,y : integer): btVector4 ;
   function    getBuffer : PByte;
   function    getWidth  : integer;
   function    getHeight : integer;
   procedure   grapicalPrintf (const  str : string ; const fontData : Pointer ; startx : integer = 0 ; starty : integer =0);
  end;


  { btDebugCastResult }

  btDebugCastResult=class(btConvexCastResult)
    m_fromTrans : btTransform;
    m_shape     : btPolyhedralConvexShape;
    m_linVel    : btVector3;
    m_angVel    : btVector3;
    m_shapeDrawer : btGL_ShapeDrawer;
  public
    constructor create(const fromTrans : btTransform ;const shape : btPolyhedralConvexShape;const linVel,angVel : btVector3 ;const drawer:btGL_ShapeDrawer);
    //
    //virtual void drawCoordSystem(const btTransform& tr)
    //{
    //        btScalar m[16];
    //        tr.getOpenGLMatrix(m);
    //        glPushMatrix();
    //        btglLoadMatrix(m);
    //        glBegin(GL_LINES);
    //        btglColor3(1, 0, 0);
    //        btglVertex3(0, 0, 0);
    //        btglVertex3(1, 0, 0);
    //        btglColor3(0, 1, 0);
    //        btglVertex3(0, 0, 0);
    //        btglVertex3(0, 1, 0);
    //        btglColor3(0, 0, 1);
    //        btglVertex3(0, 0, 0);
    //        btglVertex3(0, 0, 1);
    //        glEnd();
    //        glPopMatrix();
    //}
    //
    //virtual void    DebugDraw(btScalar      fraction)
    //{
    //        btVector3 worldBoundsMin(-1000,-1000,-1000);
    //        btVector3 worldBoundsMax(1000,1000,1000);
    //
    //
    //        btScalar m[16];
    //        btTransform hitTrans;
    //        btTransformUtil::integrateTransform(m_fromTrans,m_linVel,m_angVel,fraction,hitTrans);
    //        hitTrans.getOpenGLMatrix(m);
    //        if (m_shapeDrawer)
    //                m_shapeDrawer->drawOpenGL(m,m_shape,btVector3(1,0,0),btIDebugDraw::DBG_NoDebug,worldBoundsMin,worldBoundsMax);
    //}
  end;



  function glutmain(argcp: PInteger; argv: PPChar; width,height : integer; const title : string ; demoApp : btDemoApplication;const mode:integer=0) : integer;

implementation

{ btDemoApplication }

procedure btDemoApplication.displayProfileString(const xOffset, yStart: integer; const message: String);
begin
  glRasterPos3f(btScalar(xOffset),btScalar(yStart),btScalar(0));
  GLDebugDrawString(xOffset,yStart,message);
end;

procedure btDemoApplication.showProfileInfo(const xOffset, yStart, yIncr: integer);
begin
  //FOSTODO
  {$ifndef BT_NO_PROFILE}
  static double time_since_reset = 0.f;
  if (!m_idle)
  {
          time_since_reset = CProfileManager::Get_Time_Since_Reset();
  }


  {
          //recompute profiling data, and store profile strings

          char blockTime[128];

          double totalTime = 0;

          int frames_since_reset = CProfileManager::Get_Frame_Count_Since_Reset();

          m_profileIterator->First();

          double parent_time = m_profileIterator->Is_Root() ? time_since_reset : m_profileIterator->Get_Current_Parent_Total_Time();

          {
                  sprintf(blockTime,"--- Profiling: %s (total running time: %.3f ms) ---",        m_profileIterator->Get_Current_Parent_Name(), parent_time );
                  displayProfileString(xOffset,yStart,blockTime);
                  yStart += yIncr;
                  sprintf(blockTime,"press number (1,2...) to display child timings, or 0 to go up to parent" );
                  displayProfileString(xOffset,yStart,blockTime);
                  yStart += yIncr;

          }


          double accumulated_time = 0.f;

          for (int i = 0; !m_profileIterator->Is_Done(); m_profileIterator->Next())
          {
                  double current_total_time = m_profileIterator->Get_Current_Total_Time();
                  accumulated_time += current_total_time;
                  double fraction = parent_time > SIMD_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;

                  sprintf(blockTime,"%d -- %s (%.2f %%) :: %.3f ms / frame (%d calls)",
                          ++i, m_profileIterator->Get_Current_Name(), fraction,
                          (current_total_time / (double)frames_since_reset),m_profileIterator->Get_Current_Total_Calls());
                  displayProfileString(xOffset,yStart,blockTime);
                  yStart += yIncr;
                  totalTime += current_total_time;
          }

          sprintf(blockTime,"%s (%.3f %%) :: %.3f ms", "Unaccounted",
                  // (min(0, time_since_reset - totalTime) / time_since_reset) * 100);
                  parent_time > SIMD_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);

          displayProfileString(xOffset,yStart,blockTime);
          yStart += yIncr;



          sprintf(blockTime,"-------------------------------------------------");
          displayProfileString(xOffset,yStart,blockTime);
          yStart += yIncr;

  }
  {$endif}//BT_NO_PROFILE
end;

procedure btDemoApplication.renderscene(const pass: integer);
var    m          : array [0..15] of btScalar;
       rot        : btMatrix3x3;
       numObjects : integer;
       wireColor  : btVector3;
       i          : integer;
       colObj     : btCollisionObject;
       body       : btRigidBody;
       myMotionState : btDefaultMotionState;
       aabbMin,aabbMax : btVector3;
begin
  i:=pass;
  inc(i);
  rot.setIdentity;
  numObjects := m_dynamicsWorld.getNumCollisionObjects;
  wireColor.init(1,0,0);
  for i:=0 to numObjects-1 do begin
    colObj := m_dynamicsWorld.getCollisionObjectArray[i]^;
    body   := btRigidBody.upcast(colObj);
    myMotionState := btDefaultMotionState(body.getMotionState);
    if assigned(body) and assigned(myMotionState) then begin
      myMotionState.m_graphicsWorldTrans.getOpenGLMatrix(m);
      rot := myMotionState.m_graphicsWorldTrans.getBasisV^;
    end else begin
      colObj.getWorldTransformP^.getOpenGLMatrix(m);
      rot  := colObj.getWorldTransformP^.getBasisV^;
    end;
    wireColor.init(1,1,0.5); //wants deactivation
    if (i and 1)<>0 then begin
      wireColor.init(0,0,1);
    end;
    ///color differently for active, sleeping, wantsdeactivation states
    if (colObj.getActivationState = btas_ACTIVE_TAG) then begin//active
      if (i and 1)<>0 then begin
        wireColor += btVector3.Inits(1,0,0);
      end else begin
        wireColor += btVector3.inits(0.5,0,0);
      end;
    end;
    if colObj.getActivationState=btas_ISLAND_SLEEPING then begin //ISLAND_SLEEPING
      if (i and 1)<>0 then begin
        wireColor += btVector3.Inits(0,1,0);
      end else begin
        wireColor += btVector3.inits(0,0.5,0);
      end;
    end;
    m_dynamicsWorld.getBroadphase.getBroadphaseAabb(aabbMin,aabbMax);
    aabbMin -= btVector3.InitSameS(BT_LARGE_FLOAT);
    aabbMax += btVector3.InitSameS(BT_LARGE_FLOAT);
  //              printf("aabbMin=(%f,%f,%f)\n",aabbMin.getX(),aabbMin.getY(),aabbMin.getZ());
  //              printf("aabbMax=(%f,%f,%f)\n",aabbMax.getX(),aabbMax.getY(),aabbMax.getZ());
  //              m_dynamicsWorld->getDebugDrawer()->drawAabb(aabbMin,aabbMax,btVector3(1,1,1));
    if not(DBG_DrawWireframe in getDebugMode) then begin
      case(pass) of
        0: m_shapeDrawer.drawOpenGL(m,colObj.getCollisionShape,wireColor,getDebugMode,aabbMin,aabbMax);
        1: m_shapeDrawer.drawShadow(m,m_sundirection*rot,colObj.getCollisionShape,aabbMin,aabbMax);
        2: m_shapeDrawer.drawOpenGL(m,colObj.getCollisionShape,wireColor*0.3,[],aabbMin,aabbMax);
      end;
    end else begin
      writeln('Wireframe');
    end;
  end;
end;

constructor btDemoApplication.create;
begin
  inherited Create;
  m_clock.init;
  m_dynamicsWorld  := nil;
  m_pickConstraint := nil;
  m_shootBoxShape  := nil;
  m_cameraDistance := 15;
  m_debugMode      := [];
  m_ele            := 20;
  m_azi            := 0;
  m_cameraPosition.InitSame(0);
  m_cameraTargetPosition.initsame(0);
  m_mouseOldX    := 0;
  m_mouseOldY    := 0;
  m_mouseButtons := 0;
  m_modifierKeys := 0;
  m_scaleBottom  := 0.5;
  m_scaleFactor  := 2;
  m_cameraUp.init(0,1,0);
  m_forwardAxis  := 2;
  m_glutScreenWidth   := 0;
  m_glutScreenHeight := 0;
  m_frustumZNear := 1;
  m_frustumZFar  := 10000;
  m_ortho        := false;
  m_ShootBoxInitialSpeed := 40;
  m_stepping             := true;
  m_singleStep           := false;
  m_idle                 := false;

  m_enableshadows        := false;
  m_sundirection.init(1000,-2000,1000);
  m_defaultContactProcessingThreshold := BT_LARGE_FLOAT;

  {$ifndef BT_NO_PROFILE}
   //FOSTODO m_profileIterator := CProfileManager::Get_Iterator();
  {$endif} //BT_NO_PROFILE

  m_shapeDrawer := btGL_ShapeDrawer.create;
  m_shapeDrawer.enableTexture(true);
  m_enableshadows := false;
end;

destructor btDemoApplication.destroy;
begin
  {$ifndef BT_NO_PROFILE}
   //FOSTODO  	CProfileManager::Release_Iterator(m_profileIterator);
  {$endif} //BT_NO_PROFILE
   if assigned(m_shootBoxShape) then m_shootBoxShape.free;
   if assigned(m_shapeDrawer)   then m_shapeDrawer.free;
   writeln('btDemo Destroy');
   Inherited Destroy;
end;

function btDemoApplication.getDynamicsWorld: btDynamicsWorld;
begin
  result := m_dynamicsWorld;
end;

procedure btDemoApplication.setDrawClusters(const drawClusters: boolean);
begin
  if drawClusters then ;
end;

procedure btDemoApplication.overrideGLShapeDrawer(const shapeDrawer: btGL_ShapeDrawer);
begin
  shapeDrawer.enableTexture(m_shapeDrawer.hasTextureEnabled);
  m_shapeDrawer.free;
  m_shapeDrawer := shapeDrawer;
end;

procedure btDemoApplication.setOrthographicProjection;
begin
  // switch to projection mode
  glMatrixMode(GL_PROJECTION);
  // save previous matrix which contains the
  //settings for the perspective projection
  glPushMatrix();
  // reset matrix
  glLoadIdentity();
  // set a 2D orthographic projection
  gluOrtho2D(0, m_glutScreenWidth, 0, m_glutScreenHeight);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // invert the y axis, down is positive
  glScalef(1, -1, 1);
  // mover the origin from the bottom left corner
  // to the upper left corner
  glTranslatef(btScalar(0), btScalar(-m_glutScreenHeight), btScalar(0));
end;

procedure btDemoApplication.resetPerspectiveProjection;
begin
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  updateCamera();
end;

function btDemoApplication.setTexturing(const enable: boolean): boolean;
begin
  Result := m_shapeDrawer.enableTexture(enable);
end;

function btDemoApplication.setShadows(const enable: boolean): boolean;
var p:boolean;
begin
  p := m_enableshadows;
  m_enableshadows:=enable;
  Result := p;
end;

function btDemoApplication.getTexturing: boolean;
begin
  Result := m_shapeDrawer.hasTextureEnabled;
end;

function btDemoApplication.getShadows: boolean;
begin
  result := m_enableshadows;
end;

function btDemoApplication.getDebugmode: TbtIDEBUGFlagsSet;
begin
  result := m_debugMode;
end;

procedure btDemoApplication.setdebugmode(const mode: TbtIDEBUGFlagsSet);
begin
  m_debugMode := mode;
  if assigned(getDynamicsWorld) and assigned(getDynamicsWorld.getDebugDrawer) then begin
    getDynamicsWorld.getDebugDrawer.setDebugMode(mode);
  end;
end;

procedure btDemoApplication.setAzi(const azi: btScalar);
begin
  m_azi := azi;
end;

procedure btDemoApplication.setCameraUp(const camUp: btVector3);
begin
  m_cameraUp := camUp;
end;

procedure btDemoApplication.setCameraForwardAxis(const axis: integer);
begin
  m_forwardAxis := axis;
end;

procedure btDemoApplication.myinit;
var light_ambient,light_diffuse,light_specular,light_position0,light_position1 : array [0..3] of GLfloat;
begin
  light_ambient[0]   := 0.2 ; light_ambient[1] := 0.2 ; light_ambient[2] := 0.2  ; light_ambient[3] := 1;
  light_diffuse[0]   :=   1 ; light_diffuse[1] :=   1 ; light_diffuse[2] :=   1  ; light_diffuse[3] := 1;
  light_specular[0]  :=   1 ; light_specular[1] :=  1  ; light_specular[2] := 1  ; light_specular[3] := 1;
  light_position0[0] :=   1 ; light_position0[1] := 10 ; light_position0[2] := 1 ; light_position0[3] := 0;
  light_position1[0] :=  -1 ; light_position1[1] := -10; light_position1[2] := -1; light_position1[3] := 0;

  glLightfv(GL_LIGHT0, GL_AMBIENT,  @light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  @light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, @light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, @light_position0);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);

  glLightfv(GL_LIGHT1, GL_AMBIENT,  @light_ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE,  @light_diffuse);
  glLightfv(GL_LIGHT1, GL_SPECULAR, @light_specular);
  glLightfv(GL_LIGHT1, GL_POSITION, @light_position1);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);


  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  glClearColor(btScalar(0.7),btScalar(0.7),btScalar(0.7),btScalar(0));
  //  glEnable(GL_CULL_FACE);
  //  glCullFace(GL_BACK);
end;

procedure btDemoApplication.toggleIdle;
begin
  m_idle := not m_idle;
end;

procedure btDemoApplication.updateCamera;
var rele,razi : btScalar;
    rot,roll  : btQuaternion;
    eyePos    : btVector3;
    forward   : btVector3;
    right     : btVector3;
    aspect    : btScalar;
    extents,
    lower,
    upper     : btVector3;

begin
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity;
  rele := m_ele * btScalar(0.01745329251994329547);// rads per deg
  razi := m_azi * btScalar(0.01745329251994329547);// rads per deg

  rot.InitQ(m_cameraUp,razi);

  eyePos.InitSame(0);
  eyePos[m_forwardAxis] := -m_cameraDistance;

  forward.init(eyePos[0],eyePos[1],eyePos[2]);
  if forward.length2 < SIMD_EPSILON then begin
    forward.Init(1,0,0);
  end;

  right := m_cameraUp.cross(forward);
  roll.initQ(right,-rele);

  eyePos := btMatrix3x3.InitS(rot) * btMatrix3x3.initS(roll) * eyePos;

  m_cameraPosition[0] := eyePos.getX;
  m_cameraPosition[1] := eyePos.getY;
  m_cameraPosition[2] := eyePos.getZ;
  m_cameraPosition += m_cameraTargetPosition;

  if (m_glutScreenWidth = 0) and (m_glutScreenHeight = 0) then exit;


  if m_glutScreenWidth > m_glutScreenHeight then begin
    aspect := m_glutScreenWidth / m_glutScreenHeight;
    extents.Init(aspect, 1,0);
  end else begin
    aspect := m_glutScreenHeight / m_glutScreenWidth;
    extents.Init(1, aspect,0);
  end;


  if m_ortho then begin
    // reset matrix
    glLoadIdentity();
    extents *= m_cameraDistance;
    lower := m_cameraTargetPosition - extents;
    upper := m_cameraTargetPosition + extents;
    //gluOrtho2D(lower.x, upper.x, lower.y, upper.y);
    glOrtho(lower.getX(), upper.getX(), lower.getY(), upper.getY(),-1000,1000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    //glTranslatef(100,210,0);
  end else begin
    if m_glutScreenWidth > m_glutScreenHeight then begin
      glFrustum (-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
    end else begin
      glFrustum (-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
    end;
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2],
            m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2],
            m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
  end;
end;

function btDemoApplication.getCameraPosition: btVector3;
begin
  result := m_cameraPosition;
end;

function btDemoApplication.getCameraTargetPosition: btVector3;
begin
  result := m_cameraTargetPosition;
end;

function btDemoApplication.getDeltaTimeMicroseconds: btScalar;
begin
  {$ifdef USE_BT_CLOCK}
    result := m_clock.getTimeMicroseconds;
    m_clock.reset;
  {$else}
    result := 16666;
  {$endif}
end;

procedure btDemoApplication.setFrustumZPlanes(const zNear, zFar: btScalar);
begin
  m_frustumZNear := zNear;
  m_frustumZFar  := zFar;
end;

function btDemoApplication.getCameraDistance: btScalar;
begin

end;

procedure btDemoApplication.setCameraDistance(const dist: btScalar);
begin
  m_cameraDistance := dist;
end;

procedure btDemoApplication.moveAndDisplay;
begin
  if not m_idle then
   clientMoveAndDisplay()
  else
   displayCallback();
end;

procedure btDemoApplication.clientResetScene;
var numObjects,i:integer;
    copyArray : btCollisionObjectArray;
    colObj    : btCollisionObject;
    body      : btRigidBody;
    myMotionState : btDefaultMotionState;
begin
{$ifdef SHOW_NUM_DEEP_PENETRATIONS}
  gNumDeepPenetrationChecks := 0;
  gNumGjkChecks := 0;
{$endif} //SHOW_NUM_DEEP_PENETRATIONS

  gNumClampedCcdMotions := 0;
  numObjects := 0;

  if assigned(m_dynamicsWorld) then begin
    numObjects := m_dynamicsWorld.getNumCollisionObjects;
    ///create a copy of the array, not a reference! // FOS did'nt make a copy ...
    copyArray := m_dynamicsWorld.getCollisionObjectArray;
    for i:=0 to numObjects-1 do begin
      colObj := copyArray[i]^;
      body   := btRigidBody.upcast(colObj);
      if assigned(body) then begin
        if assigned(body.getMotionState) then begin
          myMotionState := btDefaultMotionState(body.getMotionState);
          myMotionState.m_graphicsWorldTrans := myMotionState.m_startWorldTrans;
          body.setCenterOfMassTransform(myMotionState.m_graphicsWorldTrans);
          colObj.setInterpolationWorldTransform(myMotionState.m_startWorldTrans);
          colObj.forceActivationState(btas_ACTIVE_TAG);
          colObj.activate;
          colObj.setDeactivationTime(0);
          //colObj->setActivationState(WANTS_DEACTIVATION);
        end;
        //removed cached contact points (this is not necessary if all objects have been removed from the dynamics world)
        if assigned(m_dynamicsWorld.getBroadphase.getOverlappingPairCache) then begin
        //  m_dynamicsWorld.getBroadphase.getOverlappingPairCache.cleanProxyFromPairs(colObj.getBroadphaseHandle,getDynamicsWorld.getDispatcher);
        end;

        body := btRigidBody.upcast(colObj);
        if assigned(body) and not(body.isStaticObject) then begin
          body.setLinearVelocity(cbtNullVector);
          body.setAngularVelocity(cbtNullVector);
        end;
      end;
    end;
    ///reset some internal cached data in the broadphase
    m_dynamicsWorld.getBroadphase.resetPool(getDynamicsWorld.getDispatcher);
    m_dynamicsWorld.getConstraintSolver.reset;
  end;
end;

{$define NUM_SPHERES_ON_DIAGONAL:=9}

procedure btDemoApplication.setShootBoxShape;
begin
  if not assigned(m_shootBoxShape) then begin
    m_shootBoxShape := btBoxShape.create(btVector3.initsames(0.5));
  end;
end;

procedure btDemoApplication.shootBox(const destination: btVector3);
var mass : btScalar;
    startTransform : btTransform;
    camPos,linVel  : btVector3;
    body           : btRigidBody;
begin
  if assigned(m_dynamicsWorld) then begin
    mass := 1;
    startTransform.setIdentity;
    camPos := getCameraPosition();
    startTransform.setOrigin(camPos);
    setShootBoxShape;
    body := localCreateRigidBody(mass, startTransform,m_shootBoxShape);
    body.setLinearFactor(btVector3.InitSameS(1));
    //body->setRestitution(1);

    linVel.init(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
    linVel.normalize;
    linVel*=m_ShootBoxInitialSpeed;

    body.getWorldTransformP^.setOrigin(camPos);
    body.getWorldTransformP^.setRotation(btQuaternion.Init4S(0,0,0,1));
    body.setLinearVelocity(linVel);
    body.setAngularVelocity(cbtNullVector);
    body.setCcdMotionThreshold(1);
    body.setCcdSweptSphereRadius(0.2);
  end;
end;

//var gPickingConstraintId : integer = 0;
//    gOldPickingPos       : btVector3;
//    gHitPos              : btVector3 = (v:(m_floats:(-1,-1,-1,-1)));
//    gOldPickingDist      : btScalar = 0;
//    pickedBody           : btRigidBody = nil;//for deactivation state


function btDemoApplication.getRayTo(const x, y: integer): btVector3;
var aspect     : btScalar;
    extents,
    lower,
    upper,
    p,
    rayForward,
    rayfrom,
    vertical,
    hor,
    rayto      : btVector3;
    u,v,top,
    bottom,
    nearplane,
    farplane,
//    rightOffset,
    tanfov,fov : btScalar;
    rayToCenter,dHor,dVert : btVector3;
begin
  if m_ortho then begin
    if m_glutScreenWidth > m_glutScreenHeight then begin
      aspect := m_glutScreenWidth / m_glutScreenHeight;
      extents.Init(aspect,1,0);
    end else begin
      aspect := m_glutScreenHeight / m_glutScreenWidth;
      extents.init(1,aspect,0);
    end;
    extents *= m_cameraDistance;
    lower   := m_cameraTargetPosition - extents;
    upper   := m_cameraTargetPosition + extents;
    u       := x / btScalar(m_glutScreenWidth);
    v       := (m_glutScreenHeight - y) / btScalar(m_glutScreenHeight);
    p.initsame(0);
    p.init((1.- u) * lower.getX() + u * upper.getX,(1 - v) * lower.getY + v * upper.getY,m_cameraTargetPosition.getZ);
    Result := p;
    exit;
  end;

  top        :=  1;
  bottom     := -1;
  nearPlane  :=  1;
  tanFov     := (top-bottom)*0.5 / nearPlane;
  fov        := btScalar(2.0) * btAtan(tanFov);
  rayFrom    := getCameraPosition;
  rayForward := getCameraTargetPosition-getCameraPosition;
  rayForward.normalize;
  farPlane   := 10000;
  rayForward *= farPlane;
  vertical   := m_cameraUp;
  hor        := rayForward.cross(vertical);
  hor.normalize;
  vertical   := hor.cross(rayForward);
  vertical.normalize();
  tanfov     := btTan(0.5*fov);
  hor        *= 2 * farPlane * tanfov;
  vertical   *= 2 * farPlane * tanfov;

  if m_glutScreenWidth > m_glutScreenHeight then begin
    aspect  := m_glutScreenWidth / m_glutScreenHeight;
    hor     *= aspect;
  end else begin
    aspect  := m_glutScreenHeight / m_glutScreenWidth;
    vertical*=aspect;
  end;

  rayToCenter := rayFrom + rayForward;
  dHor        := hor * 1 / m_glutScreenWidth;
  dVert       := vertical * 1/ m_glutScreenHeight;
  rayTo       := rayToCenter - 0.5 * hor + 0.5 * vertical;
  rayTo       += btScalar(x) * dHor;
  rayTo       -= btScalar(y) * dVert;
  Result      := rayTo;
end;

function btDemoApplication.localCreateRigidBody(const mass: btScalar; const startTransform: btTransform; const shape: btCollisionShape): btRigidBody;
var isDynamic:boolean;
    localInertia : btVector3;
    myMotionState : btDefaultMotionState;
    cInfo         : btRigidBodyConstructionInfo;
    body          : btRigidBody;
begin
  btAssert(not assigned(shape) or (shape.getShapeType<>INVALID_SHAPE_PROXYTYPE));

  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  isDynamic := (mass <> 0);

  localInertia.zero;
  if isDynamic then begin
    shape.calculateLocalInertia(mass,localInertia);
  //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
  end;

  myMotionState := btDefaultMotionState.create(startTransform);

  cInfo.Init(mass,myMotionState,shape,localInertia);

  body := btRigidBody.create(cInfo);
  body.setContactProcessingThreshold(m_defaultContactProcessingThreshold);
  m_dynamicsWorld.addRigidBody(body);
  Result := body;
end;

procedure btDemoApplication.keyboardCallback(key: cardinal; x, y: integer);
//var child:integer;

    procedure _ToggleDebugModeFlag(const dflag:TbtIDEBUGFlags);
    begin
      if dflag in m_debugMode then begin
        exclude(m_debugMode,dflag);
      end else begin
        include(m_debugMode,dflag);
      end;
    end;
begin
  m_lastKey := 0;

{$ifndef BT_NO_PROFILE}
  if (key >= $31) and (key <= $39) then begin
    child := key-$31;
    //FOSTODO m_profileIterator.Enter_Child(child);
  end;
  if key=$30 then begin
    //FOSTODOD m_profileIterator->Enter_Parent();
  end;
{$endif} //BT_NO_PROFILE

  case  char(key) of
    'q' : halt(0);
    'l' : stepLeft();
    'r' : stepRight();
    'f' : stepFront();
    'b' : stepBack();
    'z' : zoomIn();
    'x' : zoomOut();
    'i' : toggleIdle();
    'g' : m_enableshadows := not m_enableshadows;
    'u' : m_shapeDrawer.enableTexture(not m_shapeDrawer.enableTexture(false));
    'h' : _ToggleDebugModeFlag(DBG_NoHelpText);
    'w' : _ToggleDebugModeFlag(DBG_DrawWireframe);
    'p' : _ToggleDebugModeFlag(DBG_ProfileTimings);
    '=' : writeln('no serialization');
                  //int maxSerializeBufferSize = 1024*1024*5;
                  //btDefaultSerializer*    serializer = new btDefaultSerializer(maxSerializeBufferSize);
                  ////serializer->setSerializationFlags(BT_SERIALIZE_NO_DUPLICATE_ASSERT);
                  //m_dynamicsWorld->serialize(serializer);
                  //FILE* f2 = fopen("testFile.bullet","wb");
                  //fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1,f2);
                  //fclose(f2);
                  //delete serializer;
                  //break;
    'm' : _ToggleDebugModeFlag(DBG_EnableSatComparison);
    'n' : _ToggleDebugModeFlag(DBG_DisableBulletLCP);
    't' : _ToggleDebugModeFlag(DBG_DrawText);
    'y' : _ToggleDebugModeFlag(DBG_DrawFeaturesText);
    'a' : _ToggleDebugModeFlag(DBG_DrawAabb);
    'c' : _ToggleDebugModeFlag(DBG_DrawContactPoints);
    'C' : _ToggleDebugModeFlag(DBG_DrawConstraints);
    'L' : _ToggleDebugModeFlag(DBG_DrawConstraintLimits);
    'd' : begin
            _ToggleDebugModeFlag(DBG_NoDeactivation);
            gDisableDeactivation := DBG_NoDeactivation in m_debugMode;
          end;
    'o' : m_ortho := not m_ortho;
    's' : clientMoveAndDisplay();
    ' ' : clientResetScene();
    '1' : _ToggleDebugModeFlag(DBG_EnableCCD);
    '.' : shootBox(getRayTo(x,y));//getCameraTargetPosition());
    '+' : m_ShootBoxInitialSpeed += 10;
    '-' : m_ShootBoxInitialSpeed -= 10;
    else begin
      writeln('unused key = ',key);
    end;
  end;
 setdebugmode(m_debugMode); // refresh worlddebugdrawer ...
  //FOSTODO if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
  //        getDynamicsWorld()->getDebugDrawer()->setDebugMode(m_debugMode);
end;

procedure btDemoApplication.keyboardUpCallback(key: cardinal; x, y: integer);
begin

end;

procedure btDemoApplication.specialKeyboard(key: integer; x, y: integer);
begin

end;

procedure btDemoApplication.specialKeyboardUp(key: integer; x, y: integer);
begin

end;

procedure btDemoApplication.reshape(w, h: integer);
begin
  GLDebugResetFont(w,h);
  m_glutScreenWidth := w;
  m_glutScreenHeight := h;
  glViewport(0, 0, w, h);
  updateCamera;
end;

//var mousePickClamping : btScalar = 30;

procedure btDemoApplication.mouseFunc(button, state, x, y: integer);
var rayTo : btVector3;
begin
  if state = 0 then begin
    m_mouseButtons := m_mouseButtons or (1 SHL button);
  end else begin
    m_mouseButtons := 0;
  end;
  m_mouseOldX := x;
  m_mouseOldY := y;
  updateModifierKeys;
  if ((m_modifierKeys and BT_ACTIVE_ALT) <> 0) and (state=0) then begin
    exit;
   end;

  //printf("button %i, state %i, x=%i,y=%i\n",button,state,x,y);
  //button 0, state 0 means left mouse down

  rayTo := getRayTo(x,y);

  case button of
    2: if state=0 then shootBox(rayTo);
    1: begin
                if state=0 then begin
//#if 0
//                        //apply an impulse
//                        if (m_dynamicsWorld)
//                        {
//                                btCollisionWorld::ClosestRayResultCallback rayCallback(m_cameraPosition,rayTo);
//                                m_dynamicsWorld->rayTest(m_cameraPosition,rayTo,rayCallback);
//                                if (rayCallback.hasHit())
//                                {
//
//                                        btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
//                                        if (body)
//                                        {
//                                                body->setActivationState(ACTIVE_TAG);
//                                                btVector3 impulse = rayTo;
//                                                impulse.normalize();
//                                                float impulseStrength = 10.f;
//                                                impulse *= impulseStrength;
//                                                btVector3 relPos = rayCallback.m_hitPointWorld - body->getCenterOfMassPosition();
//                                                body->applyImpulse(impulse,relPos);
//                                        }
//                                }
//                        }
//#endif
                  end;
        end;
    0: begin
                if state=0 then begin
                        //add a point to point constraint for picking
                        if assigned(m_dynamicsWorld) then
//
//                                btVector3 rayFrom;
//                                if (m_ortho)
//                                {
//                                        rayFrom = rayTo;
//                                        rayFrom.setZ(-100.f);
//                                } else
//                                {
//                                        rayFrom = m_cameraPosition;
//                                }
//
//                                btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);
//                                m_dynamicsWorld->rayTest(rayFrom,rayTo,rayCallback);
//                                if (rayCallback.hasHit())
//                                {
//
//
//                                        btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
//                                        if (body)
//                                        {
//                                                //other exclusions?
//                                                if (!(body->isStaticObject() || body->isKinematicObject()))
//                                                {
//                                                        pickedBody = body;
//                                                        pickedBody->setActivationState(DISABLE_DEACTIVATION);
//
//
//                                                        btVector3 pickPos = rayCallback.m_hitPointWorld;
//                                                        printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
//
//
//                                                        btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
//
//
//
//
//
//
//                                                        if (use6Dof)
//                                                        {
//                                                                btTransform tr;
//                                                                tr.setIdentity();
//                                                                tr.setOrigin(localPivot);
//                                                                btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*body, tr,false);
//                                                                dof6->setLinearLowerLimit(btVector3(0,0,0));
//                                                                dof6->setLinearUpperLimit(btVector3(0,0,0));
//                                                                dof6->setAngularLowerLimit(btVector3(0,0,0));
//                                                                dof6->setAngularUpperLimit(btVector3(0,0,0));
//
//                                                                m_dynamicsWorld->addConstraint(dof6);
//                                                                m_pickConstraint = dof6;
//
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,0);
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,1);
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,2);
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,3);
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,4);
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,5);
//
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,0);
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,1);
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,2);
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,3);
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,4);
//                                                                dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,5);
//                                                        } else
//                                                        {
//                                                                btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body,localPivot);
//                                                                m_dynamicsWorld->addConstraint(p2p);
//                                                                m_pickConstraint = p2p;
//                                                                p2p->m_setting.m_impulseClamp = mousePickClamping;
//                                                                //very weak constraint for picking
//                                                                p2p->m_setting.m_tau = 0.001f;
///*
//                                                                p2p->setParam(BT_CONSTRAINT_CFM,0.8,0);
//                                                                p2p->setParam(BT_CONSTRAINT_CFM,0.8,1);
//                                                                p2p->setParam(BT_CONSTRAINT_CFM,0.8,2);
//                                                                p2p->setParam(BT_CONSTRAINT_ERP,0.1,0);
//                                                                p2p->setParam(BT_CONSTRAINT_ERP,0.1,1);
//                                                                p2p->setParam(BT_CONSTRAINT_ERP,0.1,2);
//                                                                */
//
//
//                                                        }
//                                                        use6Dof = !use6Dof;
//
//                                                        //save mouse position for dragging
//                                                        gOldPickingPos = rayTo;
//                                                        gHitPos = pickPos;
//
//                                                        gOldPickingDist  = (pickPos-rayFrom).length();
//                                                }
//                                        }
//                                }
//                        }
                end else begin
                //FOSTODO
                        //if (m_pickConstraint && m_dynamicsWorld)
                        //{
                        //        m_dynamicsWorld->removeConstraint(m_pickConstraint);
                        //        delete m_pickConstraint;
                        //        //printf("removed constraint %i",gPickingConstraintId);
                        //        m_pickConstraint = 0;
                        //        pickedBody->forceActivationState(ACTIVE_TAG);
                        //        pickedBody->setDeactivationTime( 0.f );
                        //        pickedBody = 0;
                        //}
                end;

        end;
    else ;
  end;
end;

procedure btDemoApplication.mouseMotionFunc(x, y: integer);
begin
    //    if (m_pickConstraint)
    //    {
    //            //move the constraint pivot
    //
    //            if (m_pickConstraint->getConstraintType() == D6_CONSTRAINT_TYPE)
    //            {
    //                    btGeneric6DofConstraint* pickCon = static_cast<btGeneric6DofConstraint*>(m_pickConstraint);
    //                    if (pickCon)
    //                    {
    //                            //keep it at the same picking distance
    //
    //                            btVector3 newRayTo = getRayTo(x,y);
    //                            btVector3 rayFrom;
    //                            btVector3 oldPivotInB = pickCon->getFrameOffsetA().getOrigin();
    //
    //                            btVector3 newPivotB;
    //                            if (m_ortho)
    //                            {
    //                                    newPivotB = oldPivotInB;
    //                                    newPivotB.setX(newRayTo.getX());
    //                                    newPivotB.setY(newRayTo.getY());
    //                            } else
    //                            {
    //                                    rayFrom = m_cameraPosition;
    //                                    btVector3 dir = newRayTo-rayFrom;
    //                                    dir.normalize();
    //                                    dir *= gOldPickingDist;
    //
    //                                    newPivotB = rayFrom + dir;
    //                            }
    //                            pickCon->getFrameOffsetA().setOrigin(newPivotB);
    //                    }
    //
    //            } else
    //            {
    //                    btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_pickConstraint);
    //                    if (pickCon)
    //                    {
    //                            //keep it at the same picking distance
    //
    //                            btVector3 newRayTo = getRayTo(x,y);
    //                            btVector3 rayFrom;
    //                            btVector3 oldPivotInB = pickCon->getPivotInB();
    //                            btVector3 newPivotB;
    //                            if (m_ortho)
    //                            {
    //                                    newPivotB = oldPivotInB;
    //                                    newPivotB.setX(newRayTo.getX());
    //                                    newPivotB.setY(newRayTo.getY());
    //                            } else
    //                            {
    //                                    rayFrom = m_cameraPosition;
    //                                    btVector3 dir = newRayTo-rayFrom;
    //                                    dir.normalize();
    //                                    dir *= gOldPickingDist;
    //
    //                                    newPivotB = rayFrom + dir;
    //                            }
    //                            pickCon->setPivotB(newPivotB);
    //                    }
    //            }
    //    }
    //
    //    float dx, dy;
    //dx = btScalar(x) - m_mouseOldX;
    //dy = btScalar(y) - m_mouseOldY;
    //
    //
    //    ///only if ALT key is pressed (Maya style)
    //    if (m_modifierKeys& BT_ACTIVE_ALT)
    //    {
    //            if(m_mouseButtons & 2)
    //            {
    //                    btVector3 hor = getRayTo(0,0)-getRayTo(1,0);
    //                    btVector3 vert = getRayTo(0,0)-getRayTo(0,1);
    //                    btScalar multiplierX = btScalar(0.001);
    //                    btScalar multiplierY = btScalar(0.001);
    //                    if (m_ortho)
    //                    {
    //                            multiplierX = 1;
    //                            multiplierY = 1;
    //                    }
    //
    //
    //                    m_cameraTargetPosition += hor* dx * multiplierX;
    //                    m_cameraTargetPosition += vert* dy * multiplierY;
    //            }
    //
    //            if(m_mouseButtons & (2 << 2) && m_mouseButtons & 1)
    //            {
    //            }
    //            else if(m_mouseButtons & 1)
    //            {
    //                    m_azi += dx * btScalar(0.2);
    //                    m_azi = fmodf(m_azi, btScalar(360.f));
    //                    m_ele += dy * btScalar(0.2);
    //                    m_ele = fmodf(m_ele, btScalar(180.f));
    //            }
    //            else if(m_mouseButtons & 4)
    //            {
    //                    m_cameraDistance -= dy * btScalar(0.02f);
    //                    if (m_cameraDistance<btScalar(0.1))
    //                            m_cameraDistance = btScalar(0.1);
    //
    //
    //            }
    //    }
    //
    //
    //    m_mouseOldX = x;
    //m_mouseOldY = y;
    //    updateCamera();
end;

procedure btDemoApplication.displayCallback;
begin

end;

procedure btDemoApplication.renderme;
var  xOffset: Integer;
     yStart: Integer;
     yIncr: Integer;
begin
  myinit();
  updateCamera();
  if assigned(m_dynamicsWorld) then begin
    if m_enableshadows then begin
      glClear(GL_STENCIL_BUFFER_BIT);
      glEnable(GL_CULL_FACE);
      renderscene(0);
      glDisable(GL_LIGHTING);
      glDepthMask(GL_FALSE);
      glDepthFunc(GL_LEQUAL);
      glEnable(GL_STENCIL_TEST);
      glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
      glStencilFunc(GL_ALWAYS,1,$FFFFFFFF);
      glFrontFace(GL_CCW);
      glStencilOp(GL_KEEP,GL_KEEP,GL_INCR);
      renderscene(1);
      glFrontFace(GL_CW);
      glStencilOp(GL_KEEP,GL_KEEP,GL_DECR);
      renderscene(1);
      glFrontFace(GL_CCW);
      glPolygonMode(GL_FRONT,GL_FILL);
      glPolygonMode(GL_BACK,GL_FILL);
      glShadeModel(GL_SMOOTH);
      glEnable(GL_DEPTH_TEST);
      glDepthFunc(GL_LESS);
      glEnable(GL_LIGHTING);
      glDepthMask(GL_TRUE);
      glCullFace(GL_BACK);
      glFrontFace(GL_CCW);
      glEnable(GL_CULL_FACE);
      glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
      glDepthFunc(GL_LEQUAL);
      glStencilFunc( GL_NOTEQUAL, 0,$FFFFFFFF);
      glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );
      glDisable(GL_LIGHTING);
      renderscene(2);
      glEnable(GL_LIGHTING);
      glDepthFunc(GL_LESS);
      glDisable(GL_STENCIL_TEST);
      glDisable(GL_CULL_FACE);
    end else begin
      glDisable(GL_CULL_FACE);
      renderscene(0);
    end;
    xOffset := 10;
    yStart  := 20;
    yIncr   := 20;
    glDisable(GL_LIGHTING);
    glColor3f(0, 0, 0);
    if not (DBG_NoHelpText in m_debugMode) then begin
    setOrthographicProjection();
    showProfileInfo(xOffset,yStart,yIncr);
{$ifdef USE_QUICKPROF}
    if ( getDebugMode() & btIDebugDraw::DBG_ProfileTimings)
    {
            static int counter = 0;
            counter++;
            std::map<std::string, hidden::ProfileBlock*>::iterator iter;
            for (iter = btProfiler::mProfileBlocks.begin(); iter != btProfiler::mProfileBlocks.end(); ++iter)
            {
                    char blockTime[128];
                    sprintf(blockTime, "%s: %lf",&((*iter).first[0]),btProfiler::getBlockTime((*iter).first, btProfiler::BLOCK_CYCLE_SECONDS));//BLOCK_TOTAL_PERCENT));
                    glRasterPos3f(xOffset,yStart,0);
                    GLDebugDrawString(BMF_GetFont(BMF_kHelvetica10),blockTime);
                    yStart += yIncr;

            }

    }
{$endif} //USE_QUICKPROF
    resetPerspectiveProjection();
    end;
    glEnable(GL_LIGHTING);
  end;
  updateCamera();
end;

procedure btDemoApplication.stepLeft;
begin
  m_azi -= STEPSIZE; if (m_azi < 0) then m_azi += 360; updateCamera;
end;

procedure btDemoApplication.stepRight;
begin
  m_azi += STEPSIZE; if (m_azi >= 360) then m_azi -= 360; updateCamera;
end;

procedure btDemoApplication.stepFront;
begin
  m_ele += STEPSIZE; if (m_ele >= 360) then m_ele -= 360; updateCamera;
end;

procedure btDemoApplication.stepBack;
begin
  m_ele -= STEPSIZE; if (m_ele < 0) then m_ele += 360; updateCamera;
end;

procedure btDemoApplication.zoomIn;
begin
  m_cameraDistance -= btScalar(0.4); updateCamera();
  if (m_cameraDistance < btScalar(0.1)) then m_cameraDistance := btScalar(0.1);
end;

procedure btDemoApplication.zoomOut;
begin
  m_cameraDistance += btScalar(0.4); updateCamera();
end;

function btDemoApplication.isIdle: boolean;
begin
  result := m_idle;
end;

procedure btDemoApplication.setIdle(const idle: boolean);
begin
  m_idle := idle;
end;

{ btGL_ShapeCache }

constructor btGL_ShapeCache.create(const s: btConvexShape);
begin
  m_shapehull := btShapeHull.Create(s);
  m_edges     := btGLSC_EdgeArray.create;
end;

destructor btGL_ShapeCache.Destroy;
begin
  m_edges.Free;
  m_shapehull.Free;
  inherited Destroy;
end;

{ btGL_ShapeDrawer }

function btGL_ShapeDrawer.cache(const shape: btConvexShape): btGL_ShapeCache;
var sc    : btGL_ShapeCache;
    ni,nv : integer;
    ti,pi : PbtCardinal16;
    pv    : PbtVector3;
    edges : btGLSC_EdgeArrayP;
    i,j,k : integer;
    nrm   : btVector3;
//    a,b   : cardinal;
    a,b   : integer;
    e     : ^PbtGLSC_Edge;
    de    : btGLSC_Edge;
    eidx  : integer;
begin
  sc := btGL_ShapeCache(shape.getUserPointer);
  if not assigned(sc) then begin
        sc := btGL_ShapeCache.Create(shape);
  	sc.m_shapehull.buildHull(shape.getMargin);
  	m_shapecaches.push_back(sc);
  	shape.setUserPointer(sc);
  	//
  	ni := sc.m_shapehull.numIndices;
  	nv := sc.m_shapehull.numVertices;
  	pi := sc.m_shapehull.getIndexPointer;
  	pv := sc.m_shapehull.getVertexPointer;
  	sc.m_edges.reserve(ni);
        edges := btGLSC_EdgeArrayP.create;
  	edges.resize(nv*nv,nil);
        edges.InitValues(nil);
        i:=0;
        while (i<ni) do begin
          ti  := pi+i;
          nrm := btCross(pv[ti[1].v]-pv[ti[0].v],pv[ti[2].v]-pv[ti[0].v]).normalized;
          j := 2; k:=0;
          while k<3 do begin //          for(int j=2,k=0;k<3;j=k++)
            a := ti[j].v;
            b := ti[k].v;
            eidx := btMin(a,b)*nv+btMax(a,b);
            e := edges.A[eidx];
            if not assigned(e^) then begin
              sc.m_edges.push_back(de);
              e^:=sc.m_edges.A[sc.m_edges.size-1];
              e^^.n[0] := nrm; e^^.n[1]:=-nrm;
              e^^.v[0] := a; e^^.v[1]:=b;
            end else begin
              e^^.n[1] := nrm;
            end;
           j:=k;
           inc(k);
          end;
          i := i +3;
        end;
        edges.free;
  end;
  Result := sc;
end;

constructor btGL_ShapeDrawer.create;
begin
  m_texturehandle      := 0;
  m_textureenabled     := false;
  m_textureinitialized := false;
  m_shapecaches        := btGL_ShapeCacheArray.create;
end;

destructor btGL_ShapeDrawer.destroy;
var i:integer;
begin
  writeln('GLSHAPEDRAWER FREE!!!');
  for i := 0 to m_shapecaches.size-1 do begin
    m_shapecaches[i]^.Free;
    //btAlignedFree(m_shapecaches[i]);
  end;
  m_shapecaches.clear;
  if m_textureinitialized then begin
   glDeleteTextures(1,@m_texturehandle);
  end;
  inherited;
end;

procedure glDrawVector(const v:btVector3);FOS_INLINE;
begin
  glVertex3f(v[0], v[1], v[2]);
end;

type

  { btGlDrawcallback }

  btGlDrawcallback=class(btTriangleCallback)
  private
    m_wireframe : boolean;
  public
    constructor create;
    procedure   processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer); override;
  end;

{ btGlDrawcallback }

constructor btGlDrawcallback.create;
begin
  m_wireframe:=false;
end;

procedure btGlDrawcallback.processTriangle(const triangle: PbtVector3; const partId, triangleIndex: integer);
begin
  if m_wireframe then begin
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
    glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
    glColor3f(0, 1, 0);
    glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
    glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
    glColor3f(0, 0, 1);
    glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
    glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
    glEnd();
  end  else begin
    glBegin(GL_TRIANGLES);
    //glColor3f(1, 1, 1);
    glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
    glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
    glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
    glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
    glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
    glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
    glEnd();
  end;
end;

type

  { btTriangleGlDrawcallback }

  btTriangleGlDrawcallback=class(btInternalTriangleIndexCallback)
  public
    procedure internalProcessTriangleIndex(const triangle: PbtVector3; const partId, triangleIndex: integer); override;
  end;

{ btTriangleGlDrawcallback }

procedure btTriangleGlDrawcallback.internalProcessTriangleIndex(const triangle: PbtVector3; const partId, triangleIndex: integer);
begin
  glBegin(GL_TRIANGLES);//LINES);
  glColor3f(1, 0, 0);
  glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
  glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
  glColor3f(0, 1, 0);
  glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
  glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
  glColor3f(0, 0, 1);
  glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
  glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
  glEnd();
end;



const cbtplanex:array [0..3] of GLfloat = (1,0,0,0);
//      cbtplaney:array [0..3] of GLfloat = (0,1,0,0);
      cbtplanez:array [0..3] of GLfloat = (0,0,1,0);
      cbtboxindices : array [0..35] of Integer = ( 0,1,2, 3,2,1, 4,0,6, 6,0,2, 5,1,4, 4,1,0, 7,3,1, 7,1,5, 5,4,7, 7,4,6, 7,2,3, 7,6,2 );

procedure btGL_ShapeDrawer.drawOpenGL(const m: PbtScalar; const shape: btCollisionShape; const color: btVector3; const debugMode: TbtIDEBUGFlagsSet; const worldBoundsMin, worldBoundsMax: btVector3);
var //useWireframeFallback : boolean;
    shapetype            : TbtBroadphaseNativeTypes;
    concaveMesh          : btConcaveShape;
    drawCallback         : btGLDrawcallback;

  procedure _CustomConvex;
  var org,dx,dy,halfextent: btVector3;
       boxShape           : btBoxShape;
  begin
    org.init(m[12], m[13], m[14]);
    dx.init (m[0] , m[1] , m[2]);
    dy.init (m[4] , m[5] , m[6]);
//              btVector3 dz(m[8], m[9], m[10]);
    boxShape   := btBoxShape(shape);
    halfExtent := boxShape.getHalfExtentsWithMargin;
    dx *= halfExtent[0];
    dy *= halfExtent[1];
//              dz *= halfExtent[2];
    glColor3f(1,1,1);
    glDisable(GL_LIGHTING);
    glLineWidth(2);
    glBegin(GL_LINE_LOOP);
    glDrawVector(org - dx - dy);
    glDrawVector(org - dx + dy);
    glDrawVector(org + dx + dy);
    glDrawVector(org + dx - dy);
    glEnd;
  end;
  procedure _BoxFast;
  var org,dx,dy,dz,halfextent: btVector3;
       boxShape           : btBoxShape;
  begin
    org.init(m[12], m[13], m[14]);
    dx.init(m[0], m[1], m[2]);
    dy.init(m[4], m[5], m[6]);
    dz.init(m[8], m[9], m[10]);
    boxShape   := btBoxShape(shape);
    halfExtent := boxShape.getHalfExtentsWithMargin;
    dx *= halfExtent[0];
    dy *= halfExtent[1];
    dz *= halfExtent[2];
    glBegin(GL_LINE_LOOP);
    glDrawVector(org - dx - dy - dz);
    glDrawVector(org + dx - dy - dz);
    glDrawVector(org + dx + dy - dz);
    glDrawVector(org - dx + dy - dz);
    glDrawVector(org - dx + dy + dz);
    glDrawVector(org + dx + dy + dz);
    glDrawVector(org + dx - dy + dz);
    glDrawVector(org - dx - dy + dz);
    glEnd();
    glBegin(GL_LINES);
    glDrawVector(org + dx - dy - dz);
    glDrawVector(org + dx - dy + dz);
    glDrawVector(org + dx + dy - dz);
    glDrawVector(org + dx + dy + dz);
    glDrawVector(org - dx - dy - dz);
    glDrawVector(org - dx + dy - dz);
    glDrawVector(org - dx - dy + dz);
    glDrawVector(org - dx + dy + dz);
    glEnd();
  end;
  procedure _UniformScaling;
  var scalingShape   : btUniformScalingShape;
       convexShape   : btConvexShape;
       scalingFactor : btScalar;
       tmpScaling    : array [0..3,0..3] of btScalar;
  begin
    scalingShape  := btUniformScalingShape(shape);
    convexShape   := scalingShape.getChildShape;
    scalingFactor := scalingShape.getUniformScalingFactor;
    tmpScaling[0,0] := scalingFactor; tmpScaling[0,1] := 0             ; tmpScaling[0,2] := 0             ; tmpScaling[0,3] := 0;
    tmpScaling[1,0] :=             0; tmpScaling[1,1] := scalingFactor ; tmpScaling[1,2] := 0             ; tmpScaling[1,3] := 0;
    tmpScaling[2,0] :=             0; tmpScaling[2,1] := 0             ; tmpScaling[2,2] := scalingFactor ; tmpScaling[2,3] := 0;
    tmpScaling[3,0] :=             0; tmpScaling[3,1] := 0             ; tmpScaling[3,2] := 0             ; tmpScaling[3,3] := 1;
    drawOpenGL(@tmpScaling,convexShape,color,debugMode,worldBoundsMin,worldBoundsMax);
    glPopMatrix();
  end;
  procedure _Compound;
  var compoundShape : btCompoundShape;
      i   :integer;
      childTrans : btTransform;
      colShape   : btCollisionShape;
      childMat   : array [0..15] of btScalar;
  begin
    compoundShape := btCompoundShape(shape);
    for i:=compoundShape.getNumChildShapes-1 downto 0 do begin
      childTrans := compoundShape.getChildTransform(i);
      colShape   := compoundShape.getChildShape(i);
      childTrans.getOpenGLMatrix(childMat);
      drawOpenGL(childMat,colShape,color,debugMode,worldBoundsMin,worldBoundsMax);
    end;
  end;
  procedure _GenTexture;
  var image,pi  : ^GLubyte;
      x,y,t,s : integer;
      b,c       : GLubyte;
  begin
    image := GetMem(256*256*3);
    for y:=0 to 255 do begin
      t  := SarLongint(y,4);
      pi := image+y*256*3;
      for x :=0 to 255 do begin
        s := sarlongint(x,4);
        b := 180;
        c := b+((s+ (t and 1)) and 1)*(255-b);
        pi[2]:=c;pi[1]:=c;pi[0]:=c; pi+=3;
      end;
    end;
    glGenTextures(1,@m_texturehandle);
    glBindTexture(GL_TEXTURE_2D,m_texturehandle);
    glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
    glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
    glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR_MIPMAP_LINEAR);
    glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
    gluBuild2DMipmaps(GL_TEXTURE_2D,3,256,256,GL_RGB,GL_UNSIGNED_BYTE,image);
    Freemem(image);
  end;

  procedure _Sphere;
  var sphereShape : btSphereShape;
      radius      : btScalar;
  begin
    sphereShape := btSphereShape(shape);
    radius      := sphereShape.getMargin;//radius doesn't include the margin, so draw with margin
    drawSphere(radius,10,10);
//    useWireframeFallback := false;
  end;
  procedure _Box;
  var boxShape   : btBoxShape;
      halfExtent,
      normal     : btVector3;
      v1,v2,v3   : PbtVector3;
      si,i       : integer;
      vertices   : array [0..7] of btVector3;
  begin
    boxShape   := btBoxShape(shape);
    halfExtent := boxShape.getHalfExtentsWithMargin;

    vertices[0].init(halfExtent[0],halfExtent[1],halfExtent[2]);
    vertices[1].init(-halfExtent[0],halfExtent[1],halfExtent[2]);
    vertices[2].init(halfExtent[0],-halfExtent[1],halfExtent[2]);
    vertices[3].init(-halfExtent[0],-halfExtent[1],halfExtent[2]);
    vertices[4].init(halfExtent[0],halfExtent[1],-halfExtent[2]);
    vertices[5].init(-halfExtent[0],halfExtent[1],-halfExtent[2]);
    vertices[6].init(halfExtent[0],-halfExtent[1],-halfExtent[2]);
    vertices[7].init(-halfExtent[0],-halfExtent[1],-halfExtent[2]);
    glBegin (GL_TRIANGLES);
    si := 36;
    i  := 0;
    while (i<si) do begin
      v1 := @vertices[cbtboxindices[i]];
      v2 := @vertices[cbtboxindices[i+1]];
      v3 := @vertices[cbtboxindices[i+2]];
      normal := (v3^-v1^).cross(v2^-v1^);
      normal.normalize;
      glNormal3f(normal.getX,normal.getY,normal.getZ);
      glVertex3f (v1^.x(), v1^.y(), v1^.z());
      glVertex3f (v2^.x(), v2^.y(), v2^.z());
      glVertex3f (v3^.x(), v3^.y(), v3^.z());
      i+=3;
    end;
    glEnd();
//    useWireframeFallback := false;
  end;
  procedure _StaticPlane;
  var staticPlaneShape : btStaticPlaneShape;
      planeConst       : btScalar;
      planeNormal      : PbtVector3;
      planeOrigin,vec0,
      vec1,pt0,pt1,pt2,
      pt3              :btVector3;
      vecLen           : btScalar;
  begin
    staticPlaneShape := btStaticPlaneShape(shape);
    planeConst       := staticPlaneShape.getPlaneConstant;
    planeNormal      := staticPlaneShape.getPlaneNormalP;
    planeOrigin      := planeNormal^ * planeConst;
    btPlaneSpace1(planeNormal^,vec0,vec1);
    vecLen := 100;
    pt0 := planeOrigin + vec0*vecLen;
    pt1 := planeOrigin - vec0*vecLen;
    pt2 := planeOrigin + vec1*vecLen;
    pt3 := planeOrigin - vec1*vecLen;
    glBegin(GL_LINES);
    glVertex3f(pt0.getX(),pt0.getY(),pt0.getZ());
    glVertex3f(pt1.getX(),pt1.getY(),pt1.getZ());
    glVertex3f(pt2.getX(),pt2.getY(),pt2.getZ());
    glVertex3f(pt3.getX(),pt3.getY(),pt3.getZ());
    glEnd();
  end;
  procedure _MultiSphere;
  var multiSphereShape : btMultiSphereShape;
      childTransform : btTransform;
      i              : integer;
      sc             : btSphereShape;
      childMat       : array [0..15] of btScalar;
  begin
    multiSphereShape := btMultiSphereShape(shape);
    childTransform.setIdentity;
    for i := multiSphereShape.getSphereCount-1 downto 0 do begin
      sc := btSphereShape.Create(multiSphereShape.getSphereRadius(i));
      childTransform.setOrigin(multiSphereShape.getSpherePosition(i));
      childTransform.getOpenGLMatrix(childMat);
      drawOpenGL(childMat,sc,color,debugMode,worldBoundsMin,worldBoundsMax);
      sc.free;
    end;
  end;
  procedure _Default;
  var sc      : btGL_ShapeCache;
      hull    : btShapeHull;
      index,i : integer;
      i1,i2,i3: integer;
      index1,
      index2,
      index3  : integer;
      idx     : PbtCardinal16;
      vtx     : PbtVector3;
      v1,v2,v3: PbtVector3;
      normal  : btVector3;
  begin
    if shape.isConvex then begin
      sc := cache(btConvexShape(shape));
      //glutSolidCube(1.0);
      hull := sc.m_shapehull ; //(btShapeHull*)shape->getUserPointer()
      if hull.numTriangles > 0 then begin
        idx   := hull.getIndexPointer;
        vtx   := hull.getVertexPointer;
        index := 0;
        glBegin (GL_TRIANGLES);
        for i := 0  to hull.numTriangles-1 do begin
          i1 := index;inc(index);
          i2 := index;inc(index);
          i3 := index;inc(index);
          btAssert((i1<hull.numIndices) and (i2 < hull.numIndices) and (i3 < hull.numIndices));
          index1 := idx[i1].v; index2 := idx[i2].v; index3 := idx[i3].v;
          btAssert((index1 < hull.numVertices) and (index2<hull.numVertices) and (index3<hull.numVertices));
          v1 := @vtx[index1];
          v2 := @vtx[index2];
          v3 := @vtx[index3];
          normal := (v3^-v1^).cross(v2^-v1^);
          normal.normalize;
          glNormal3f(normal.getX(),normal.getY(),normal.getZ());
          glVertex3f (v1^.x, v1^.y, v1^.z);
          glVertex3f (v2^.x, v2^.y, v2^.z);
          glVertex3f (v3^.x, v3^.y, v3^.z);
        //  writeln(i:2,' : ',v1^.DumpValues,' ',v2^.DumpValues,' ',v3^.DumpValues);
        end;
        glEnd ();
      end;
    end;
  end;
  procedure _Polyhedral;
  var polyshape : btPolyhedralConvexShape;
      i         : integer;
      vtx,normal: btVector3;
  begin
    if (DBG_DrawFeaturesText in debugMode) and (shape.isPolyhedral) then begin
      polyshape := btPolyhedralConvexShape(shape);
      glColor3f(1,1,1);
      //for i:=0 to polyshape.getNumVertices-1 do begin
      //  polyshape.getVertex(i,vtx);
      //  //char buf[12];
      //  //sprintf(buf," %d",i);
      //  //btDrawString(BMF_GetFont(BMF_kHelvetica10),buf);
      //end;
      for i:=0 to polyshape.getNumPlanes-1 do begin
        polyshape.getPlane(normal,vtx,i);
        //btScalar d = vtx.dot(normal);
        //char buf[12];
        //sprintf(buf," plane %d",i);
        //btDrawString(BMF_GetFont(BMF_kHelvetica10),buf);
      end;
    end;
  end;

begin
  if shape.getShapeType = CUSTOM_CONVEX_SHAPE_TYPE then begin
    _CustomConvex;exit;
  end else
  if (shape.getShapeType = BOX_SHAPE_PROXYTYPE) and (DBG_FastWireframe in debugMode) then begin
    _BoxFast; exit;
  end;
  glPushMatrix();
  glMultMatrixf(m);
  if shape.getShapeType = UNIFORM_SCALING_SHAPE_PROXYTYPE then begin
    _UniformScaling; exit;
  end;
  if (shape.getShapeType = COMPOUND_SHAPE_PROXYTYPE) then begin
    _Compound;
  end else begin
    if m_textureenabled and not m_textureinitialized then begin
      _GenTexture;
    end;
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity;
    glScalef(0.025,0.025,0.025);
    glMatrixMode(GL_MODELVIEW);
    glTexGenfv(GL_S,GL_OBJECT_PLANE,cbtplanex);
    glTexGenfv(GL_T,GL_OBJECT_PLANE,cbtplanez);
    glTexGeni(GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
    glTexGeni(GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
    glEnable(GL_TEXTURE_GEN_S);
    glEnable(GL_TEXTURE_GEN_T);
    glEnable(GL_TEXTURE_GEN_R);
    m_textureinitialized:=true;

    //drawCoordSystem();

    //glPushMatrix();
    glEnable(GL_COLOR_MATERIAL);
    if m_textureenabled then begin
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D,m_texturehandle);
    end else begin
      glDisable(GL_TEXTURE_2D);
    end;
    glColor3f(color.x(),color.y(), color.z());
//    useWireframeFallback := true;

    if not (DBG_DrawWireframe in debugMode) then begin
      ///you can comment out any of the specific cases, and use the default
      ///the benefit of 'default' is that it approximates the actual collision shape including collision margin
      //int shapetype=m_textureenabled?MAX_BROADPHASE_COLLISION_TYPES:shape->getShapeType();
      shapetype := shape.getShapeType;
      case shapetype of
              SPHERE_SHAPE_PROXYTYPE       : _Sphere;
              BOX_SHAPE_PROXYTYPE          : _Box;
              STATIC_PLANE_PROXYTYPE       : _StaticPlane;
              MULTI_SPHERE_SHAPE_PROXYTYPE : _MultiSphere;
              else  _Default;
      end;
    end;
    glNormal3f(0,1,0);
    /// for polyhedral shapes
    _Polyhedral;
  end;
{$ifdef USE_DISPLAY_LISTS}
    if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE||shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE) then begin
            GLuint dlist =   OGL_get_displaylist_for_shape((btCollisionShape * )shape);
            if (dlist) then begin
              glCallList(dlist);
            end else begin

{$else}
    if shape.isConcave and not(shape.isInfinite) then begin
      concaveMesh := btConcaveShape(shape);
      drawCallback := btGlDrawcallback.Create;
      drawCallback.m_wireframe := DBG_DrawWireframe in debugMode;
      concaveMesh.processAllTriangles(drawCallback,worldBoundsMin,worldBoundsMax);
      drawcallback.free;
    end;
{$endif}
{$ifdef USE_DISPLAY_LISTS}
     end;
   end;
{$endif}
  glPopMatrix();
end;

procedure btGL_ShapeDrawer.drawShadow(const m: PbtScalar; const extrusion: btVector3; const shape: btCollisionShape; const worldBoundsMin, worldBoundsMax: btVector3);
  procedure _uniformscaling;
  var scalingShape  : btUniformScalingShape;
      convexShape   : btConvexShape;
      scalingFactor : btScalar;
      tmpScaling    : array [0..3,0..3] of btScalar;
  begin
    scalingShape    := btUniformScalingShape(shape);
    convexShape     := scalingShape.getChildShape;
    scalingFactor   := scalingShape.getUniformScalingFactor;
    tmpScaling[0,0] := scalingFactor; tmpScaling[0,1] := 0             ; tmpScaling[0,2] := 0             ; tmpScaling[0,3] := 0;
    tmpScaling[1,0] :=             0; tmpScaling[1,1] := scalingFactor ; tmpScaling[1,2] := 0             ; tmpScaling[1,3] := 0;
    tmpScaling[2,0] :=             0; tmpScaling[2,1] := 0             ; tmpScaling[2,2] := scalingFactor ; tmpScaling[2,3] := 0;
    tmpScaling[3,0] :=             0; tmpScaling[3,1] := 0             ; tmpScaling[3,2] := 0             ; tmpScaling[3,3] := 1;
    drawShadow(@tmpScaling,extrusion,convexShape,worldBoundsMin,worldBoundsMax);
    glPopMatrix();
  end;
  procedure _compound;
  var compoundShape : btCompoundShape;
      i   :integer;
      childTrans : btTransform;
      colShape   : btCollisionShape;
      childMat   : array [0..15] of btScalar;
  begin
    compoundShape := btCompoundShape(shape);
    for i:=compoundShape.getNumChildShapes-1 downto 0 do begin
      childTrans := compoundShape.getChildTransform(i);
      colShape   := compoundShape.getChildShape(i);
      childTrans.getOpenGLMatrix(childMat);
      drawShadow(childMat,extrusion*childTrans.GetBasisV^,colShape,worldBoundsMin,worldBoundsMax);
    end;
  end;
  procedure _isconvex;
  var sc   : btGL_ShapeCache;
      hull : btShapeHull;
      i    : integer;
      d    : btScalar;
      q    : integer;
      a,b  : PbtVector3;
  begin
    //	bool useWireframeFallback = true;
    if shape.isConvex then begin
      sc   := cache(btConvexShape(shape));
      hull := sc.m_shapehull;
      glBegin(GL_QUADS);
      for i := 0 to sc.m_edges.size-1 do begin
        d := btDot(sc.m_edges[i]^.n[0],extrusion);
        if (d*btDot(sc.m_edges[i]^.n[1],extrusion))<0 then begin
          q := btDecide(d<0 , 1 , 0);
          a := hull.getVertexPtrIDX(sc.m_edges[i]^.v[q]);
          b := hull.getVertexPtrIDX(sc.m_edges[i]^.v[1-q]);
          glVertex3f(a^[0],a^[1],a^[2]);
          glVertex3f(b^[0],b^[1],b^[2]);
          glVertex3f(b^[0]+extrusion[0],b^[1]+extrusion[1],b^[2]+extrusion[2]);
          glVertex3f(a^[0]+extrusion[0],a^[1]+extrusion[1],a^[2]+extrusion[2]);
        end;
      end;
      glEnd();
    end;
  end;
  procedure _concave;
  var concaveMesh  : btConcaveShape;
      drawCallback : btGlDrawcallback;
  begin
    concaveMesh  := btConcaveShape(shape);
    drawCallback := btGlDrawcallback.create;
    concaveMesh.processAllTriangles(drawCallback,worldBoundsMin,worldBoundsMax);
    drawCallback.free;;
  end;

begin
  glPushMatrix();
  glMultMatrixf(m);
  if shape.getShapeType = UNIFORM_SCALING_SHAPE_PROXYTYPE then begin
    _uniformscaling;
    exit;
  end else
  if shape.getShapeType=COMPOUND_SHAPE_PROXYTYPE then begin
    _compound;
  end else begin
    _isconvex;
  end;
  if shape.isConcave then begin//>getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE||shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
        //              if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
    _concave;
  end;
  glPopMatrix();
end;

function btGL_ShapeDrawer.enableTexture(const enable: boolean): boolean;
begin
  result           := m_textureenabled;
  m_textureenabled := enable;
end;

function btGL_ShapeDrawer.hasTextureEnabled: boolean;
begin
  result := m_textureenabled;
end;

class procedure btGL_ShapeDrawer.drawCylinder(const radius, halfHeight: btScalar; const upAxis: integer);
var quadObj : PGLUquadric;
begin
  glPushMatrix;
  case upAxis of
    0: begin
  	glRotatef(-90.0, 0.0, 1.0, 0.0);
  	glTranslatef(0.0, 0.0, -halfHeight);
       end;
    1: begin
        glRotatef(-90.0, 1.0, 0.0, 0.0);
        glTranslatef(0.0, 0.0, -halfHeight);
       end;
    2: begin
        glTranslatef(0.0, 0.0, -halfHeight);
       end;
    else btAssert(false);
  end;

  quadObj := gluNewQuadric();

  //The gluCylinder subroutine draws a cylinder that is oriented along the z axis.
  //The base of the cylinder is placed at z = 0; the top of the cylinder is placed at z=height.
  //Like a sphere, the cylinder is subdivided around the z axis into slices and along the z axis into stacks.

  gluQuadricDrawStyle(quadObj, GLU_FILL);
  gluQuadricNormals(quadObj,   GLU_SMOOTH);
  gluDisk(quadObj,0,radius,15, 10);
  gluCylinder(quadObj, radius, radius, 2*halfHeight, 15, 10);
  glTranslatef(0.0, 0.0, 2*halfHeight);
  glRotatef(-180.0, 0.0, 1.0, 0.0);
  gluDisk(quadObj,0,radius,15, 10);
  glPopMatrix();
  gluDeleteQuadric(quadObj);
end;

class procedure btGL_ShapeDrawer.drawSphere(const radius: btScalar; const lats, longs: integer);
var i,j : integer;
    lat0,z0,zr0,lat1,z1,zr1,lng,x,y:btScalar;
begin
  for i := 0 to lats do begin
    lat0 := SIMD_PI * (-btScalar(0.5) + (i - 1) / lats);
    z0   := radius * sin(lat0);
    zr0  :=  radius*cos(lat0);
    lat1 := SIMD_PI * (-btScalar(0.5) +  i / lats);
    z1   := radius*sin(lat1);
    zr1  := radius*cos(lat1);
    glBegin(GL_QUAD_STRIP);
    for j := 0 to longs do begin
      lng := 2 * SIMD_PI *  (j - 1) / longs;
      x   := cos(lng);
      y   := sin(lng);
      glNormal3f(x * zr1, y * zr1, z1);
      glVertex3f(x * zr1, y * zr1, z1);
      glNormal3f(x * zr0, y * zr0, z0);
      glVertex3f(x * zr0, y * zr0, z0);
    end;
    glEnd;
  end;
end;

class procedure btGL_ShapeDrawer.drawCoordSystem;
begin
  glBegin(GL_LINES);
  glColor3f (1, 0, 0);
  glVertex3d(0, 0, 0);
  glVertex3d(1, 0, 0);
  glColor3f (0, 1, 0);
  glVertex3d(0, 0, 0);
  glVertex3d(0, 1, 0);
  glColor3f (0, 0, 1);
  glVertex3d(0, 0, 0);
  glVertex3d(0, 0, 1);
  glEnd;
end;

{ btGlutDemoApplication }

constructor btGlutDemoApplication.Create;
begin
  myID:='GlutDemoApplication';
  inherited create;
end;

procedure btGlutDemoApplication.specialKeyboard(key: integer; x, y: integer);
//var numobj:integer;
begin
  case key of
     GLUT_KEY_F1: ;
     GLUT_KEY_F2: ;
     GLUT_KEY_END: begin
  		//numObj := getDynamicsWorld.getNumCollisionObjects;
  		//if numObj<>0 then begin
  		//	btCollisionObject* obj = getDynamicsWorld()->getCollisionObjectArray()[numObj-1];
  		//	getDynamicsWorld()->removeCollisionObject(obj);
  		//	btRigidBody* body = btRigidBody::upcast(obj);
  		//	if (body && body->getMotionState())
  		//	{
  		//		delete body->getMotionState();
  		//	}
  		//	delete obj;
  		//end;
  	end;
     GLUT_KEY_LEFT      : stepLeft;
     GLUT_KEY_RIGHT     : stepRight;
     GLUT_KEY_UP        : stepFront;
     GLUT_KEY_DOWN      : stepBack;
     GLUT_KEY_PAGE_UP   : zoomIn;
     GLUT_KEY_PAGE_DOWN : zoomOut;
     GLUT_KEY_HOME      : toggleIdle;
    else begin
    //  writeln('unused special key '+key);
    end;
  end;
  glutPostRedisplay();
end;

procedure btGlutDemoApplication.swapBuffers;
begin
  glutSwapBuffers;
end;

procedure btGlutDemoApplication.updateModifierKeys;
begin
  m_modifierKeys := 0;
  if (glutGetModifiers() and GLUT_ACTIVE_ALT)   <> 0 then m_modifierKeys := m_modifierKeys or BT_ACTIVE_ALT;
  if (glutGetModifiers() and GLUT_ACTIVE_CTRL)  <> 0 then m_modifierKeys := m_modifierKeys or BT_ACTIVE_CTRL;
  if (glutGetModifiers() and GLUT_ACTIVE_SHIFT) <> 0 then m_modifierKeys := m_modifierKeys or BT_ACTIVE_SHIFT;
end;

{ btGLDebugDrawer }

constructor btGLDebugDrawer.create;
begin
  m_debugMode:=[];
end;

procedure btGLDebugDrawer.drawLine(const from, too, fromColor, toColor: btVector3);
begin
  glBegin(GL_LINES);
  	glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ());
  	glVertex3f(from.getX(), from.getY(), from.getZ());
  	glColor3f(toColor.getX(), toColor.getY(), toColor.getZ());
  	glVertex3f(too.getX(), too.getY(), too.getZ());
  glEnd();
end;

procedure btGLDebugDrawer.drawLine(const from, too, color: btVector3);
begin
  drawLine(from,too,color,color);
end;

procedure btGLDebugDrawer.drawSphere(const p: btVector3; const radius: btScalar; const color: btVector3);
var i,j,longs,lats:integer;
    lng,x,y,lat0,z0,zr0,lat1,z1,zr1 : btScalar;
begin
  glColor4f (color.getX(), color.getY(), color.getZ(), btScalar(1));
  glPushMatrix ();
  glTranslatef (p.getX(), p.getY(), p.getZ());

  lats  := 5;
  longs := 5;

  for i := 0 to lats do begin
    lat0 := SIMD_PI * (-btScalar(0.5) + (i - 1) / lats);
    z0   := radius*sin(lat0);
    zr0  := radius*cos(lat0);
    lat1 := SIMD_PI * (-btScalar(0.5) + i / lats);
    z1   := radius*sin(lat1);
    zr1  := radius*cos(lat1);
    glBegin(GL_QUAD_STRIP);
    for j := 0 to longs do begin
      lng := 2 * SIMD_PI * (j - 1) / longs;
      x   := cos(lng);
      y   := sin(lng);
      glNormal3f(x * zr0, y * zr0, z0);
      glVertex3f(x * zr0, y * zr0, z0);
      glNormal3f(x * zr1, y * zr1, z1);
      glVertex3f(x * zr1, y * zr1, z1);
    end;
    glEnd();
  end;
  glPopMatrix;
end;

procedure btGLDebugDrawer.drawBox(const bbMin, bbMax, color: btVector3; const alpha: btScalar);
var center,halfExtent : btVector3;
begin
  halfExtent := (bbMax - bbMin) * btScalar(0.5);
  center     := (bbMax + bbMin) * btScalar(0.5);
  //glEnable(GL_BLEND);     // Turn blending On
  //glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  glColor4f(color.getX(), color.getY(), color.getZ(), alpha);
  glPushMatrix();
  glTranslatef(center.getX(), center.getY(), center.getZ());
  glScaled(2*halfExtent[0], 2*halfExtent[1], 2*halfExtent[2]);
//	glutSolidCube(1.0);
  glPopMatrix ();
  //glDisable(GL_BLEND);
end;

procedure btGLDebugDrawer.drawTriangle(const a, b, c, color: btVector3; const alpha: btScalar);
var n:btVector3;
begin
//    if (m_debugMode > 0)
  n := btCross(b-a,c-a).normalized();
  glBegin(GL_TRIANGLES);
  glColor4f(color.getX(), color.getY(), color.getZ(),alpha);
  glNormal3d(n.getX(),n.getY(),n.getZ());
  glVertex3d(a.getX(),a.getY(),a.getZ());
  glVertex3d(b.getX(),b.getY(),b.getZ());
  glVertex3d(c.getX(),c.getY(),c.getZ());
  glEnd();
end;

procedure btGLDebugDrawer.drawContactPoint(const PointOnB, normalOnB: btVector3; const distance: btScalar; const lifeTime: integer; const color: btVector3);
var too,from : btVector3;
begin
  too  := pointOnB+normalOnB*distance;
  from := pointOnB;
  glColor4f(color.getX(), color.getY(), color.getZ(),1);
  //glColor4f(0,0,0,1.f);

  glBegin(GL_LINES);
  glVertex3d(from.getX(), from.getY(), from.getZ());
  glVertex3d(too.getX(), too.getY(), too.getZ());
  glEnd();

  //glRasterPos3f(from.x(),  from.y(),  from.z());
  //char buf[12];
  //sprintf(buf," %d",lifeTime);
  //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
end;

procedure btGLDebugDrawer.reportErrorWarning(const warningString: string);
begin
 writeln('WARN: ',warningString);
end;

procedure btGLDebugDrawer.draw3dText(const location: btVector3; const textString: string);
begin
//  glRasterPos3f(location.x(),  location.y(),  location.z());

end;

procedure btGLDebugDrawer.setDebugMode(const debugMode: TbtIDEBUGFlagsSet);
begin
  m_debugMode := debugMode;
end;

function btGLDebugDrawer.getDebugMode: TbtIDEBUGFlagsSet;
begin
  result := m_debugMode;
end;


var gDemoApplication : btDemoApplication = nil;
    //TGlutVoidCallback = procedure; cdecl;
    //TGlut1IntCallback = procedure(value: Integer); cdecl;
    //TGlut2IntCallback = procedure(v1, v2: Integer); cdecl;
    //TGlut3IntCallback = procedure(v1, v2, v3: Integer); cdecl;
    //TGlut4IntCallback = procedure(v1, v2, v3, v4: Integer); cdecl;
    //TGlut1Char2IntCallback = procedure(c: Byte; v1, v2: Integer); cdecl;
    //TGlut1UInt3IntCallback = procedure(u: Cardinal; v1, v2, v3: Integer); cdecl;


procedure glutKeyboardCallback(key: Byte; x, y: Integer); cdecl;
begin
  gDemoApplication.keyboardCallback(key,x,y);
end;
procedure glutKeyboardUpCallback(key: Byte; x, y: Integer); cdecl;
begin
  gDemoApplication.keyboardUpCallback(key,x,y);
end;

procedure glutSpecialKeyboardCallback(key, x, y: Integer);cdecl;
begin
  gDemoApplication.specialKeyboard(key,x,y);
end;

procedure glutSpecialKeyboardUpCallback(key, x, y: Integer);cdecl;
begin
  gDemoApplication.specialKeyboardUp(key,x,y);
end;

procedure glutReshapeCallback( w, h : Integer);cdecl;
begin
  gDemoApplication.reshape(w,h);
end;

procedure glutMoveAndDisplayCallback;cdecl;
begin
  gDemoApplication.moveAndDisplay;
end;

procedure glutMouseFuncCallback(button, state, x, y : integer);cdecl;
begin
  gDemoApplication.mouseFunc(button,state,x,y);
end;

procedure glutMotionFuncCallback(x, y : integer);cdecl;
begin
  gDemoApplication.mouseMotionFunc(x,y);
end;

procedure glutDisplayCallback;cdecl;
begin
  gDemoApplication.displayCallback;
end;


function glutmain(argcp: PInteger; argv: PPChar; width,height : integer; const title : string ; demoApp : btDemoApplication; const mode:integer=0) : integer;
begin
  gDemoApplication := demoApp;
  glutInit(argcp, argv);
  if mode=0 then begin
    glutInitDisplayMode(GLUT_DOUBLE or GLUT_RGBA or GLUT_DEPTH or GLUT_STENCIL);
  end else begin
    glutInitDisplayMode(GLUT_RGB or GLUT_DOUBLE or GLUT_DEPTH);
  end;
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(width, height);
  glutCreateWindow(pchar(title));

  gDemoApplication.myinit;

  glutKeyboardFunc(@glutKeyboardCallback);
  glutKeyboardUpFunc(@glutKeyboardUpCallback);
  glutSpecialFunc(@glutSpecialKeyboardCallback);
  glutSpecialUpFunc(@glutSpecialKeyboardUpCallback);

  glutReshapeFunc(@glutReshapeCallback);
//createMenu();
  glutIdleFunc(@glutMoveAndDisplayCallback);
  glutMouseFunc(@glutMouseFuncCallback);
  glutPassiveMotionFunc(@glutMotionFuncCallback);
  glutMotionFunc(@glutMotionFuncCallback);
  glutDisplayFunc(@glutDisplayCallback );
  glutMoveAndDisplayCallback();

//enable vsync to avoid tearing on Apple (todo: for Windows)

//#if defined(__APPLE__) && !defined (VMDMESA)
//int swap_interval = 1;
//CGLContextObj cgl_context = CGLGetCurrentContext();
//CGLSetParameter(cgl_context, kCGLCPSwapInterval, &swap_interval);
//#endif


  Result := 0;
end;


{ GL_Simplex1to4 }

procedure GL_Simplex1to4.calcClosest(const m: PbtScalar);
var tr  : btTransform;
//    res : boolean;
    v   : btVector3;
    i   : integer;
begin
  tr.setFromOpenGLMatrix(m);
  btGL_ShapeDrawer.drawCoordSystem;
  if assigned(m_simplexSolver) then begin
    m_simplexSolver.reset;
    for i :=0 to m_numVertices-1 do begin
      v :=  tr.opTrans(m_vertices[i]);
      m_simplexSolver.addVertex(v,v,btVector3.InitSameS(0));
      m_simplexSolver.closest(v);
    end;
    //draw v?
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(v.x,v.y,v.z);
    glEnd();
    glEnable(GL_LIGHTING);
  end;
end;

procedure GL_Simplex1to4.setSimplexSolver(const simplexSolver: btSimplexSolverInterface);
begin
  m_simplexSolver := simplexSolver;
end;

{ btRenderTexture }

constructor btRenderTexture.create(const width, height: integer);
var x,y : integer;
begin
  inherited Create;
  m_width:=width;
  m_height:=height;
  GetMem(m_buffer,m_width*m_height*4);
  //clear screen
  FillByte(m_buffer^,m_width*m_height*4,0);
  //clear screen version 2
  for x := 0 to m_width-1 do begin
    for y := 0 to m_height-1 do begin
      self.setPixel(x,y,btVector4.Init4S(x,y,0,1));
    end;
  end;
end;

destructor btRenderTexture.Destroy;
begin
  Freemem(m_buffer);
  inherited Destroy;
end;

procedure btRenderTexture.setPixel(const x, y: integer; const rgba: btVector4);
var pixel : PByte;
begin
  pixel    := @m_buffer[ (x+y*m_width) * 4];
  pixel[0] := Byte(trunc( 255 * rgba.getX));
  pixel[1] := Byte(trunc( 255 * rgba.getY));
  pixel[2] := Byte(trunc( 255 * rgba.getZ));
  pixel[3] := Byte(trunc( 255 * rgba.getW));
end;

procedure btRenderTexture.addPixel(const x, y: integer; const rgba: btVector4);
var pixel : PByte;
begin
  pixel    := @m_buffer[ (x+y*m_width) * 4];
  pixel[0] := round(btMin( 255, pixel[0] + (255 * rgba.getX)));
  pixel[1] := round(btMin( 255, pixel[1] + (255 * rgba.getY)));
  pixel[2] := round(btMin( 255, pixel[2] + (255 * rgba.getZ)));
//  pixel30] := round(btMin( 255, pixel[3] + (255 * rgba.getW)));
end;

function btRenderTexture.getPixel(const x, y: integer): btVector4;
var pixel : PByte;
begin
  pixel    := @m_buffer[ (x+y*m_width) * 4];
  result.Init4(pixel[0]*1/255,pixel[1]*1/255,pixel[2]*1/255,pixel[3]*1/255);
end;

function btRenderTexture.getBuffer: PByte;
begin
  result := m_buffer;
end;

function btRenderTexture.getWidth: integer;
begin
  result := m_width;
end;

function btRenderTexture.getHeight: integer;
begin
  result := m_height;
end;

procedure btRenderTexture.grapicalPrintf(const str: string; const fontData: Pointer; startx: integer; starty: integer);
var  c           : Byte;
     x,xx,i,y,j  : integer;
     fontPtr     : PByte;
     ch          : Byte;
     sx,sy,k     : integer;
     packedColor : Byte;
     colorf      : btScalar;
     rgba        : btVector4;
begin
  fontPtr := fontData;
  xx:=0;
  for k:=1 to length(str) do begin
    c:=byte(str[k]);
    x  := xx;
    ch := c-32;
    sx := ch mod 16;
    sy := ch div 16;
    for i := sx*16 to (sx*16+16)-1 do begin
      y := 0;
      for j := sy*16 to (sy*16+16)-1 do begin
        packedColor := fontPtr[i*3+255*256*3-(256*j)*3];
        colorf      := packedColor / 255;
        rgba.Init4  (colorf,colorf,colorf,1);
        addPixel(startx+x,starty+y,rgba);
        inc(y);
      end;
      inc(x);
    end;
    xx += 10;
  end;
end;

{ btDebugCastResult }

constructor btDebugCastResult.create(const fromTrans: btTransform; const shape: btPolyhedralConvexShape; const linVel, angVel: btVector3; const drawer: btGL_ShapeDrawer);
begin
  m_fromTrans   := fromTrans;
  m_shape       := shape;
  m_linVel      := linVel;
  m_angVel      := angVel;
  m_shapeDrawer := drawer;
end;

end.

