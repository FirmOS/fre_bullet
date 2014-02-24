unit CDTestFrameworkDemo;

{$mode objfpc}{$H+}

interface

uses  Classes, SysUtils,btCollisionShapes,btDispatch,btBroadphase,btNarrowphase,btDynamics,FOS_AlignedArray,btLinearMath,glutstuff,gl,glu,glut,gldebugfont,gldebugfontmap;

const
   NUM_SAP_BOXES = 1024;

   WINDOW_WIDTH  = 1024;
   WINDOW_HEIGHT = 768;
//   #define ONE_OVER_RAND_MAX		(1.0f / float(RAND_MAX))	//!< Inverse of the max possible value returned by rand()
   ONE_OVER_RAND_MAX = 1 / 2147483647;


var
   percentUpdate:Integer   = 100;
   objectSpeed:Single      = 0.00;//0.01;
   enableDraw:Boolean      = true;
   enableCulling:boolean   = True;
   enableOcclusion:boolean = true;
   showCulling:boolean     = true;
   showOcclusion:boolean   = True;
   cullFarPlane:boolean	   = false;
   predictablerandom_hh:boolean = true;

//Broadphase comparison
//Static case (updating 10% of objects to same position ( -> no swaps)
//number of objects //OPCODE BoxPruning / Bullet SAP / Bullet MultiSAP
//1024                     0.35ms, 0.03ms, 0.15ms
//8192                       21ms, 0.2ms, 5ms
//16384                   92ms , 0.5ms, 28ms

//Dynamic case, 10% objects are moving as fast as this testbed allows (0.01?)
//number of objects //OPCODE BoxPruning / Bullet SAP / Bullet MultiSAP
//1024                                                0.35ms, 0.2ms, 0.25ms
//8192                        21ms , 15ms , 13ms
//16384                    92ms,  80ms,  49ms


  function  UnitRandomFloat:single ; // inline;

type

  { btBulletSAPCompleteBoxPruningTest }

  { bticeAABB }

  { bticePoint }

  bticePoint=btVector3;

  bticeAABB=object
    mCenter,mExtents : bticePoint;
    procedure Init;
    procedure SetCenterExtents (const c,e : bticePoint);
    procedure GetExtents       (var extents:bticePoint);
    procedure GetCenter        (var center:bticePoint);
  end;
  PbtIceAABB = ^ bticeAABB;

  { bticePAIRS }

  bticePAIRS=object
    procedure ResetPairs;
  end;

  { bticeOBB }

  bticeOBB=object
    mCenter,mExtents : bticePoint;
    mRot             : btMatrix3x3;
    procedure init;
  end;

  { btProfiler }

  btProfiler=object
    mFreq,mcounter0,mcounter1:Int64;
    mCycles       : integer;
    mTime,mMsTime : single;
    mNBQueries    : cardinal;
    cl            : btClock;
    procedure Init;
    procedure Start;
    procedure EndP;
    procedure Reset;
    procedure Accum;
  end;

  btBulletSAPCompleteBoxPruningTest=class
  private
    m_broadphase : btBroadphaseInterface;
    m_proxies    : btBroadphaseproxyArray;
    m_isdbvt     : Boolean;
    m_method     : integer;
    m_firstTime  : Boolean;
    m_methodname : string;

    mNbBoxes     : longword;
    mBoxes       : array [0..NUM_SAP_BOXES] of bticeAABB;
    mBoxPtrs     : array [0..NUM_SAP_BOXES] of ^bticeAABB;
    mBoxTime     : array [0..NUM_SAP_BOXES] of single;
    mFlags       : array [0..NUM_SAP_BOXES] of Boolean;

    mPairs       : bticePairs;
    mProfiler    : btProfiler;
    mAmplitude   : single;

    function     UpdateBoxes(numBoxes:integer):Boolean;

  public
    constructor Create(const  numBoxes,method:Integer);
    destructor  Destroy;
    procedure   Init;
    procedure   Release;
    procedure   PerformTest;
    procedure   RenderAll;
    procedure   Render;
    procedure   Select;
    procedure   Deselect;
    procedure   KeyboardCallback(key:byte; x,y : integer);
    procedure   MouseCallback(button, state, x, y : Integer);
    procedure   MotionCallback(x,y : integer);
    procedure   TWEAK_Setup;
    procedure   TWEAK_DeferredCollide(const dc:boolean);
    procedure   _DrawOBB(const obb:bticeOBB);
    procedure   _SetupGLMatrix(const pos: bticePoint;const rot:btMatrix3x3);
  end;


  { btCDTestFrameWorkDemo }
  btCDTestFrameWorkDemo=class(btGlutDemoApplication)
    constructor create;
    destructor  Destroy;override;
    procedure   myInit;override;
    procedure   exitPhysics;
    procedure   initPhysics;
    procedure   clientMoveAndDisplay; override;
    procedure   displayCallback; override;
    procedure   SetupCameraMatrix;
    procedure   keyboardCallback(key: cardinal; x, y: integer); override;
  end;

var
  numParts:integer        = 2;
  visiblecount:integer	  = 0;
  sBulletProfilerToggle:Boolean = false;

type

  { btOcclusionBuffer }

  btOcclusionBuffer=class
  public
    class function WriteOCL_Process(var   q:btScalar;const v:btscalar):boolean;inline;  //TODO : FOS -> result ?
    class function QueryOCL_Process(const q:btScalar;const v:btScalar):boolean;inline;
  private
    initialized    : Boolean;
    buffer         : btFOSAlignedScalars;
    sizes          : array [0..1]  of integer;
    scales,offsets : array [0..1]  of btScalar;
    wtrs           : array [0..15] of btScalar;
    eye,neardist   : btVector3;
    ocarea,qrarea  : btScalar;
    texture        : GLuint;
  public
    constructor create;
    destructor  destroy;override;
    procedure   setup          (const w,h : integer);
    procedure   clear;
    procedure   initialize;
    procedure   drawBuffer     (l,t,r,b : btScalar);
    function    transform      (const x:btVector3):btVector4;
    class function project     (const p:PbtVector4;const n:integer):boolean;
    function    queryOccluder2 (const c,e : btVector3):boolean;
    procedure   appendOccluder2(const c,e : btVector3);
    function    clipDraw4WQOCL  (const p:PbtVector4;const minarea:btScalar;const query:boolean):boolean; //inline;
    function    clip4          (const pi:PbtVector4;const po:PbtVector4):integer;
    function    drawWQOCL      (const a,b,c : btVector4 ; const minarea : btScalar ; const query:boolean):boolean;
  end;

  var ocb : btOcclusionBuffer;
      gCollisionTest : btBulletSAPCompleteBoxPruningTest;

  var gEye : bticePoint = (v:(m_floats:(240,205,205,0)));
      gDir : bticePoint = (v:(m_floats:(-1,-1,-1,0)));
      gN   : bticePoint;
      gFOV : btScalar = 60.0;


  function  ComputeWorldRay(xs, ys : integer):bticePoint;
  function  GetCameraPos : bticePoint;
  function  GetCameraDir : bticePoint;


implementation

var my_rand : single=0;
function UnitRandomFloat: single;
begin
  if not predictablerandom_hh then begin;
   result :=  Random;
  end else begin
    my_rand := my_rand + 0.01;
    if my_rand>1 then begin
      my_rand:=0;
    end;
    result := my_rand;
  end;
end;

function ComputeWorldRay(xs, ys: integer): bticePoint;
var viewPort : array [0..3 ] of GLint;
  modelMatrix,projMatrix : array [0..15] of GLdouble;
  wx0, wy0, wz0, wx1, wy1, wz1 : GLdouble ;
  tmp : bticePoint;

begin
 glGetIntegerv (GL_VIEWPORT,         @viewPort);
 glGetDoublev  (GL_MODELVIEW_MATRIX, @modelMatrix);
 glGetDoublev (GL_PROJECTION_MATRIX, @projMatrix);
 ys := viewPort[3] - ys - 1;
 gluUnProject(GLdouble(xs), GLdouble(ys), 0.0, modelMatrix, projMatrix, viewPort, @wx0, @wy0, @wz0);
 gluUnProject(GLdouble(xs), GLdouble(ys), 1.0, modelMatrix, projMatrix, viewPort, @wx1, @wy1, @wz1);
 tmp.init(single(wx1-wx0), single(wy1-wy0), single(wz1-wz0));
 tmp.Normalize;
 result := tmp;
end;

function GetCameraPos: bticePoint;
begin
  Result := gEye;
end;

function GetCameraDir: bticePoint;
begin
  Result := gDir;
end;

{ btCDTestFrameWorkDemo }

constructor btCDTestFrameWorkDemo.create;
begin
  inherited;
end;

destructor btCDTestFrameWorkDemo.Destroy;
begin
  //      ReleaseTerrain();
  //      for(int i=0;i<MAX_NB_TESTS;i++)
  //      {
  //              gCollisionTests[i]->Release();
  //              DELETESINGLE(gCollisionTests[i]);
  //      }
  //      TwTerminate();
  inherited Destroy;
end;

procedure btCDTestFrameWorkDemo.myInit;
var light_ambient,light_diffuse,light_specular,light_position0,light_position1 : array [0..3] of GLfloat;
begin
 // inherited myInit;
  //      ::SetPriorityClass(::GetCurrentProcess(),REALTIME_PRIORITY_CLASS);

  // glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
  // Setup default render states

  glClearColor(0.3, 0.4, 0.5, 1.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_CULL_FACE);
  glDepthFunc(GL_LEQUAL);


  // Setup lighting
  //glEnable(GL_LIGHTING);
  //float AmbientColor[] = { 0.0f, 0.1f, 0.2f, 0.0f };              glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientColor);
  //float DiffuseColor[] = { 1.0f, 1.0f, 1.0f, 0.0f };              glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseColor);
  //float SpecularColor[] = { 0.0f, 0.0f, 0.0f, 0.0f };             glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularColor);
  //float Position[] = { -10.0f, 1000.0f, -4.0f, 1.0f };    glLightfv(GL_LIGHT0, GL_POSITION, Position);
  //glEnable(GL_LIGHT0);

  light_ambient[0]   :=   0 ; light_ambient[1] :=   0.1 ; light_ambient[2] := 0.2  ; light_ambient[3] := 0;
  light_diffuse[0]   :=   1 ; light_diffuse[1] :=   1 ; light_diffuse[2] :=   1  ; light_diffuse[3] := 1;
  light_specular[0]  :=   0 ; light_specular[1] :=  0  ; light_specular[2] := 0  ; light_specular[3] := 0;
  light_position0[0] :=   -10 ; light_position0[1] := 1000 ; light_position0[2] := -4 ; light_position0[3] := 1;


  glLightfv(GL_LIGHT0, GL_AMBIENT,  @light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  @light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, @light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, @light_position0);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

 // CreateTerrain;
  // Create tests
   gCollisionTest := btBulletSAPCompleteBoxPruningTest.Create(NUM_SAP_BOXES,7);
   gCollisionTest.Init;
   gCollisionTest.Select;
   gCollisionTest.TWEAK_Setup;
// MotionCallback(0,0);
end;

procedure btCDTestFrameWorkDemo.exitPhysics;
begin

end;

procedure btCDTestFrameWorkDemo.initPhysics;
begin

end;

procedure btCDTestFrameWorkDemo.clientMoveAndDisplay;
begin

end;

procedure btCDTestFrameWorkDemo.displayCallback;
begin
  // Clear buffers
  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);
  // Setup camera
  glMatrixMode(GL_PROJECTION);
  SetupCameraMatrix;
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity;
  glEnable(GL_LIGHTING);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  gCollisionTest.PerformTest;

  // Draw tweak bars
  //TwDraw();
  glutSwapBuffers;
  glutPostRedisplay();
end;

procedure btCDTestFrameWorkDemo.SetupCameraMatrix;
begin
 glLoadIdentity();
 gluPerspective(gFOV, (glutGet(GLUT_WINDOW_WIDTH))/(glutGet(GLUT_WINDOW_HEIGHT)), 1.0, 10000.0);
 gluLookAt(gEye.x, gEye.y, gEye.z, gEye.x + gDir.x, gEye.y + gDir.y, gEye.z + gDir.z, 0.0, 1.0, 0.0);
end;

procedure btCDTestFrameWorkDemo.keyboardCallback(key: cardinal; x, y: integer);
begin
  case char(key) of
    'o' : enableOcclusion := not enableOcclusion;
    's' : showOcclusion   := not showOcclusion;
    'f' : cullFarPlane    := not cullFarPlane;
    else inherited keyboardCallback(key, x, y);
  end;
end;

{ btBulletSAPCompleteBoxPruningTest }

function btBulletSAPCompleteBoxPruningTest.UpdateBoxes(numBoxes: integer): Boolean;
var i:integer;
    Center,Extents : bticePoint;

begin
  for i:=0 to numBoxes-1 do begin
    mBoxTime[i] += objectSpeed;
    mBoxes[i].GetExtents(Extents);
    Center._x^ := cos(mBoxTime[i]*2.17)*mAmplitude + sin(mBoxTime[i])*mAmplitude*0.5;
    Center._y^ := cos(mBoxTime[i]*1.38)*mAmplitude + sin(mBoxTime[i]*mAmplitude);
    Center._z^ := sin(mBoxTime[i]*0.777)*mAmplitude;
    mBoxes[i].SetCenterExtents(Center, Extents);
  end;
  Result := true;
end;

constructor btBulletSAPCompleteBoxPruningTest.Create(const numBoxes, method: Integer);
var  aabbMin,aabbMax : btVector3;
     maxNumBoxes     : integer;
     disableRaycastAccelerator : Boolean;
     cbb                       : btDbvtBroadphase;
begin
  mProfiler.Init;
  mNbBoxes := numBoxes;
//  mBoxes   := nil;
//  mBoxPtrs := nil;
//  mBoxTime := nil;
  mAmplitude := 100.0;
  m_method   := method;
  aabbMin.Init(-200,-200,-200);
  aabbMax.init(200,200,200);
  maxNumBoxes               := numBoxes;
  m_isdbvt                  := false;
  disableRaycastAccelerator := true;
  if method<>7 then begin
    writeln('method must be 7');
    abort;
  end;
  //      switch (method)
  //      {
  //      case 1:
  //              m_broadphase = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes,0,disableRaycastAccelerator);
  //              methodname      =       "btAxisSweep3";
  //              break;
  //      case 2:
  //              m_broadphase = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes,new btNullPairCache(),disableRaycastAccelerator);
  //              methodname      =       "btAxisSweep3+btNullPairCache";
  //              break;
  //      case 3:
  //              m_broadphase = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes,new btSortedOverlappingPairCache(),disableRaycastAccelerator);
  //              methodname      =       "btAxisSweep3+btSortedOverlappingPairCache";
  //              break;
  //      case 4:
  //              m_broadphase = new btSimpleBroadphase(maxNumBoxes,new btSortedOverlappingPairCache());
  //              methodname      =       "btSimpleBroadphase+btSortedOverlappingPairCache";
  //              break;
  //      case 5:
  //              m_broadphase = new btSimpleBroadphase(maxNumBoxes,new btNullPairCache());
  //              methodname      =       "btSimpleBroadphase+btNullPairCache";
  //              break;

  cbb := btDbvtBroadphase.Create;
  cbb.m_deferedcollide:=true;
  m_broadphase := cbb;
  m_isdbvt     := true;
  m_MethodName   := 'dynamic AABB tree, btDbvtBroadphase';
  ocb := btOcclusionBuffer.create;
end;

destructor btBulletSAPCompleteBoxPruningTest.Destroy;
begin
  //TODO
  //DELETEARRAY(mBoxTime);
  //DELETEARRAY(mBoxPtrs);
  //DELETEARRAY(mBoxes);
  m_proxies.Free;
  m_broadphase.free;
  ocb.Free;
end;

procedure btBulletSAPCompleteBoxPruningTest.Init;
var clock : btClock;
    i     : Integer;
    Center, Extents : bticePoint;
    aabbMin,
    aabbMax : btVector3;
    proxy   : btBroadphaseProxy;

begin
  m_firstTime := true;
  Randomize;
  for i:=0 to mNbBoxes-1 do begin
    mBoxes[i].Init;
    mBoxPtrs[i] := nil;
    mBoxTime[i] := 0;
  end;
  m_proxies := btBroadphaseproxyArray.create;
  m_proxies.Initialize(btBroadphaseProxy);

  clock.init;
  for i:=0 to mNbBoxes-1 do begin
    Center._x^  := (UnitRandomFloat-0.5) * 100.0;
    Center._y^  := (UnitRandomFloat-0.5) * 10.0;
    Center._z^   := (UnitRandomFloat-0.5) * 100.0;
    Extents._x^ := 2.0 + (UnitRandomFloat * 2.0);
    Extents._y^ := 2.0 + (UnitRandomFloat * 2.0);
    Extents._z^ := 2.0 + (UnitRandomFloat * 2.0);
    mBoxes[i].SetCenterExtents(Center, Extents);
    mBoxPtrs[i] := @mBoxes[i];
    aabbMin.Init(Center.x-Extents.x,Center.y-Extents.y,Center.z-Extents.z);
    aabbMax.init(Center.x+Extents.x,Center.y+Extents.y,Center.z+Extents.z);
    proxy  := m_broadphase.createProxy(aabbMin,aabbMax,BOX_SHAPE_PROXYTYPE,@mBoxes[i],[btcfgDefaultFilter],[btcfgDefaultFilter],nil,nil);
//    writeln('MBOXES ',i,'  ',inttohex(cardinal(@mBoxes[i]),2),' ',PbtIceAABB(@mboxes[i])-PbtIceAABB(@mboxes[0]));
    m_proxies.push_back( proxy );
    mBoxTime[i] := 2000.0*UnitRandomFloat;
  end;
  writeln(format('Initialization of %s with %u boxes: %ums\r\n',[m_methodname,mNbBoxes,clock.getTimeMilliseconds]));
end;

procedure btBulletSAPCompleteBoxPruningTest.Release;
begin
  //DELETEARRAY(mBoxTime); static array
  //DELETEARRAY(mBoxes);
end;


procedure DrawVolume(const volume : btDbvtVolume ; const color : btVector3);
var mins,maxs : btVector3;
begin
  mins := volume.Mins^;
  maxs := volume.Maxs^;
  glColor3f(color.x(),color.y(),color.z());
  glVertex3f(mins.x(),mins.y(),mins.z());
  glVertex3f(maxs.x(),mins.y(),mins.z());

  glVertex3f(maxs.x(),mins.y(),mins.z());
  glVertex3f(maxs.x(),maxs.y(),mins.z());

  glVertex3f(maxs.x(),maxs.y(),mins.z());
  glVertex3f(mins.x(),maxs.y(),mins.z());

  glVertex3f(mins.x(),maxs.y(),mins.z());
  glVertex3f(mins.x(),mins.y(),mins.z());

  glVertex3f(mins.x(),mins.y(),maxs.z());
  glVertex3f(maxs.x(),mins.y(),maxs.z());

  glVertex3f(maxs.x(),mins.y(),maxs.z());
  glVertex3f(maxs.x(),maxs.y(),maxs.z());

  glVertex3f(maxs.x(),maxs.y(),maxs.z());
  glVertex3f(mins.x(),maxs.y(),maxs.z());

  glVertex3f(mins.x(),maxs.y(),maxs.z());
  glVertex3f(mins.x(),mins.y(),maxs.z());

  glVertex3f(mins.x(),mins.y(),mins.z());
  glVertex3f(mins.x(),mins.y(),maxs.z());

  glVertex3f(maxs.x(),mins.y(),mins.z());
  glVertex3f(maxs.x(),mins.y(),maxs.z());

  glVertex3f(maxs.x(),maxs.y(),mins.z());
  glVertex3f(maxs.x(),maxs.y(),maxs.z());

  glVertex3f(mins.x(),maxs.y(),mins.z());
  glVertex3f(mins.x(),maxs.y(),maxs.z());
end;




procedure btBulletSAPCompleteBoxPruningTest.PerformTest;
var numUpdatedBoxes : Integer;
    Center,Extents  : bticePoint;
    aabbMin,aabbMax : btVector3;
    i,j,k           : Integer;
    pairCache       : btOverlappingPairCache;
    pairArray       : btBroadphasePairArray;
    pairPtr         : PbtBroadphasePair;
    tmpAabbMin,tmpAabbMax : btVector3;
    numP                  : single;
    CurrentBox            : bticeOBB;
    mmin,mmax             : btVector3;
    clock                 : btClock;

begin
  numUpdatedBoxes := (mNbBoxes*percentUpdate) div 100;
  if m_firstTime then begin
    numUpdatedBoxes := mNbBoxes;
  end;

  mProfiler.Start;
  UpdateBoxes(numUpdatedBoxes);
 // mPairs.ResetPairs;

  //CompleteBoxPruning(mNbBoxes, mBoxPtrs, mPairs, Axes(AXES_XZY));
  ///add batch query?
  clock.init;
  writeln('START');
  for i := 0 to numUpdatedBoxes-1 do begin
    mBoxPtrs[i]^.GetCenter(Center);
    mBoxPtrs[i]^.GetExtents(Extents);
    aabbMin.init(Center.x-Extents.x,Center.y-Extents.y,Center.z-Extents.z);
    aabbMax.init(Center.x+Extents.x,Center.y+Extents.y,Center.z+Extents.z);
    m_broadphase.setAabb(m_proxies[i]^,aabbMin,aabbMax,nil);//m_dispatcher);
  end;
 // writeln('CLOCK ',clock.getTimeMilliseconds);
 // abort;

//#ifndef BT_NO_PROFILE
//  if(sBulletProfilerToggle)
//  {
//          CProfileManager::Reset();
//  }
//#endif //BT_NO_PROFILE

  m_broadphase.calculateOverlappingPairs(nil);

//#ifndef BT_NO_PROFILE
//  if(sBulletProfilerToggle)
//  {
//          CProfileManager::Increment_Frame_Counter();
//          CProfileManager::dumpAll();
//  }
//#endif //BT_NO_PROFILE


  mProfiler.EndP;
  mProfiler.Accum;

  if (m_firstTime) then begin
    //initialization messes up timings
    m_firstTime := false;
    btDbvtBroadphase(m_broadphase).m_deferedcollide := false;
    mProfiler.Reset();
  end;

  //#if 0
  //        {
  //        int     missedpairs=0;
  //        for(int i=0;i<m_proxies.size();++i)
  //                {
  //                btDbvtProxy*    pa((btDbvtProxy*)m_proxies[i]);
  //                for(int j=i+1;j<m_proxies.size();++j)
  //                        {
  //                        btDbvtProxy*    pb((btDbvtProxy*)m_proxies[j]);
  //                        if(Intersect(pa->aabb,pb->aabb))
  //                                {
  //                                btDbvtProxy*    spa=pa;
  //                                btDbvtProxy*    spb=pb;
  //                                if(spa>spb) btSwap(spa,spb);
  //                                if(!m_broadphase->getOverlappingPairCache()->findPair(spa,spb))
  //                                        {
  //                                        ++missedpairs;
  //                                        printf("Cannot find %i,%i\r\n",i,j);
  //                                        }
  //                                }
  //                        }
  //                }
  //        if(missedpairs>0) printf("Missed pairs: %u\r\n",missedpairs);
  //        }
  //#endif

 //    printf("%d pairs colliding\r     ", mPairs.GetNbPairs());
 // writeln('pairs colliding ',mPairs.GetNbPairs);

  FillByte(mFlags,sizeof(Boolean)*mNbBoxes,0);

  pairCache := m_broadphase.getOverlappingPairCache;
  pairArray := pairCache.getOverlappingPairArray;

  for  i:=0 to pairCache.getNumOverlappingPairs-1 do begin
      j :=  PbtIceAABB(pairArray[i]^.m_pProxy0.m_clientObject) - PbtIceAABB(@mBoxes[0]);
     // writeln('j1= ',j);
      mFlags[j] := true;
      j :=  PbtIceAABB(pairArray[i]^.m_pProxy1.m_clientObject) - PbtIceAABB(@mBoxes[0]);
    //  writeln('j2= ',j);
      mFlags[j] := true;
  end;

  if enableDraw then begin
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    aabbMin.init(-200,-200,-200);
    aabbMax.init(200,200,200);
    glDisable(GL_DEPTH_TEST);
    numP := trunc(numParts);
    for i:=0 to numParts-1 do begin
      tmpAabbMin[0] := aabbMin[0] + i*(aabbMax[0]-aabbMin[0])/numP;
      tmpAabbMax[0] := aabbMin[0] + (i+1)*(aabbMax[0]-aabbMin[0])/numP;
      for j :=0 to numParts-1 do begin
        tmpAabbMin[1] := aabbMin[1] + j*(aabbMax[1]-aabbMin[1])/numP;
        tmpAabbMax[1] := aabbMin[1] + (j+1)*(aabbMax[1]-aabbMin[1])/numP;
        for k:=0 to numParts-1 do begin
          tmpAabbMin[2] := aabbMin[2] + k*(aabbMax[2]-aabbMin[2])/numP;
          tmpAabbMax[2] := aabbMin[2] + (k+1)*(aabbMax[2]-aabbMin[2])/numP;
          CurrentBox.init;
          CurrentBox.mRot.setIdentity;
          mmin.Init(tmpAabbMin[0],tmpAabbMin[1],tmpAabbMin[2]);
          mmax.init(tmpAabbMax[0],tmpAabbMax[1],tmpAabbMax[2]);
          glColor4f(i, j,k,0.2);//1.0f, 0.0f);
          CurrentBox.mCenter  := (mmin+mmax)*0.5;
          CurrentBox.mExtents := (mmax-mmin)*0.5;
          _DrawOBB(CurrentBox);
        end;
       end;
    end;
    glEnable(GL_DEPTH_TEST);
    //glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    Render();
  end;
//  char Buffer[4096];
//  sprintf_s(Buffer, sizeof(Buffer), "Bullet %s: %5.1f us (%d cycles) : %d pairs\n", methodname, mProfiler.mMsTime, mProfiler.mCycles,
//                  m_broadphase->getOverlappingPairCache()->getNumOverlappingPairs());
//
////    m_broadphase)->printStats();
//  GLFontRenderer::print(10.0f, 10.0f, 0.02f, Buffer);
 writeln(format('Bullet %s: %5.1f us (%d cycles) : %d pairs', [m_methodname, mProfiler.mMsTime, mProfiler.mCycles,m_broadphase.getOverlappingPairCache.getNumOverlappingPairs]));
end;

procedure btBulletSAPCompleteBoxPruningTest.RenderAll;
var CurrentBox : bticeOBB;
             i : integer;
begin
  CurrentBox.mRot.setIdentity;
  for i := 0 to mNbBoxes-1 do begin
    if mFlags[i] then begin
      glColor3f(1, 0, 0);
    end else begin
      glColor3f(0, 1, 0);
    end;
    mBoxes[i].GetCenter(CurrentBox.mCenter);
    mBoxes[i].GetExtents(CurrentBox.mExtents);
    _DrawOBB(CurrentBox);
  end;
end;


type

     { bticeSceneRenderer }

     bticeSceneRenderer = class(btDbvt_ICollide)
       drawn : Integer;
       this  : btBulletSAPCompleteBoxPruningTest;
       ocb   : btOcclusionBuffer;
       constructor create;
       procedure init;
       function  Descent(const node:PbtDbvtNode):Boolean;override;
       procedure Process(const n: PbtDbvtNode; const s: btScalar); override;
       procedure Process(const leaf: PbtDbvtNode); override;
     end;

{ bticeSceneRenderer }

constructor bticeSceneRenderer.create;
begin
  init;
end;

procedure bticeSceneRenderer.init;
begin
  drawn  := 0;
end;

//var dbg_cnt:integer=0;

function bticeSceneRenderer.Descent(const node: PbtDbvtNode): Boolean;
begin
  Result := ocb.queryOccluder2(node^.volume.Center,node^.volume.Extents);
end;

procedure bticeSceneRenderer.Process(const n: PbtDbvtNode; const s: btScalar);
begin
  Process(n);
end;

procedure bticeSceneRenderer.Process(const leaf: PbtDbvtNode);
var proxy : btBroadphaseProxy;
        i : integer;
        box   : bticeOBB;

begin
  box.mRot.SetIdentity;
  proxy := btBroadphaseProxy(leaf^.int.data);
// i     := ((AABB*)proxy->m_clientObject)-self->mBoxes;
  i    :=  PbtIceAABB(Proxy.m_clientObject) - PbtIceAABB(@this.mBoxes[0]);

 if(this.mFlags[i])then	 begin
   glColor3f(1.0, 0.0, 0.0);
 end else begin
   glColor3f(0.0, 1.0, 0.0);
 end;
 this.mBoxes[i].GetCenter(box.mCenter);
 this.mBoxes[i].GetExtents(box.mExtents);
 this._DrawOBB(box);
 inc(drawn);
 if assigned(ocb) then begin
   ocb.appendOccluder2(btVector3.inits(box.mCenter.x,box.mCenter.y,box.mCenter.z),btVector3.inits(box.mExtents.x,box.mExtents.y,box.mExtents.z));
 end;
end;

const mm_array_r:array [0..15] of single =   (  1,0,0,0,
                                                0,1,0,0,
                                                0,0,1,0,
                                                0,0,0,1  );
const mm_array_deb:array [0..15] of single = (  1,0,0,0,
                                                0,0,1,0,
                                                0,1,0,0,
                                                0,0,0,1  );

const mm_array_buf:array [0..15] of single = (  1,0,0,0,
                                                0,1,0,0,
                                                0,0,0,1,
                                                0,0,0,1  );


var cnt:integer=0;


 type

  { bticeDebugRenderer }

  bticeDebugRenderer=class(btDbvt_ICollide)
   ocb : btOcclusionBuffer;
   sid : integer;
   function  Descent(const n: PbtDbvtNode): boolean; override;
   procedure Process(const n1, n2: PbtDbvtNode); override;
   procedure Process(const node: PbtDbvtNode); override;
   procedure Process(const n: PbtDbvtNode; const s: btScalar); override;
  end;

{ bticeDebugRenderer }


function bticeDebugRenderer.Descent(const n: PbtDbvtNode): boolean;
begin
  Result :=  ocb.queryOccluder2(n^.volume.Center,n^.volume.Extents);
end;

procedure bticeDebugRenderer.Process(const n1, n2: PbtDbvtNode);
begin
  inherited Process(n1, n2);
end;

procedure bticeDebugRenderer.Process(const node: PbtDbvtNode);
var f : single;
begin
  if assigned(ocb) then begin
    ocb.appendOccluder2(node^.volume.Center,node^.volume.Extents);
  end;
  if sid>=0 then begin
    f := sid / 1023;
    DrawVolume(node^.volume,btVector3.inits(1,f,f));
    sid := (sid+1) mod 1024;
  end else begin
    if node^.isinternal then begin
      DrawVolume(node^.volume,btVector3.inits(0,1,0))
    end else begin
      DrawVolume(node^.volume,btVector3.inits(1,0,1));
    end;
  end;
end;

procedure bticeDebugRenderer.Process(const n: PbtDbvtNode; const s: btScalar);
begin
  Process(n);
end;


procedure btBulletSAPCompleteBoxPruningTest.Render;
var pbp : btDbvtBroadphase;
const margin   = 0;
      lc       = margin;
      farplane = 200;
var   rc,tc,bc : integer;
    c00,c10,c01,c11,
    eye,dir,x00,x01,x10,x11: btVector3;
    planes_n :  array [0..4] of btVector3;
    planes_o :  array [0..4] of btScalar;
//    nplanes  : integer;
    acplanes : integer;
    i: Integer;
    srenderer : bticeSceneRenderer;
    ratio,scale    : single;
    drenderer : bticeDebugRenderer;


begin
  visiblecount := mNbBoxes ;
  inc(cnt);

  if not enableCulling then begin
    RenderAll;
  end else begin
    pbp    := btDbvtBroadphase(m_broadphase);
    rc     := glutGet(GLUT_WINDOW_WIDTH)-(1+margin);
    tc     := margin;
    bc     := glutGet(GLUT_WINDOW_HEIGHT)-(1+margin);
    c00.Init(ComputeWorldRay(lc,tc).x,ComputeWorldRay(lc,tc).y,ComputeWorldRay(lc,tc).z);
    c10.init(ComputeWorldRay(rc,tc).x,ComputeWorldRay(rc,tc).y,ComputeWorldRay(rc,tc).z);
    c01.init(ComputeWorldRay(lc,bc).x,ComputeWorldRay(lc,bc).y,ComputeWorldRay(lc,bc).z);
    c11.init(ComputeWorldRay(rc,bc).x,ComputeWorldRay(rc,bc).y,ComputeWorldRay(rc,bc).z);
    eye.init(GetCameraPos().x,GetCameraPos().y,GetCameraPos().z);
    dir.init(GetCameraDir().x,GetCameraDir().y,GetCameraDir().z);
    x00 := eye+c00*100;
    x10 := eye+c10*100;
    x01 := eye+c01*100;
    x11 := eye+c11*100;

    ocb.initialize;
    ocb.eye := eye;

//    nplanes := sizeof(planes_n)/sizeof(planes_n[0]);
    if cullFarPlane then begin
      acplanes := 5;
    end else begin
      acplanes := 4;
    end;
    planes_n[0] := c01.cross(c00).normalized();
    planes_n[1] := c10.cross(c11).normalized();
    planes_n[2] := c00.cross(c10).normalized();
    planes_n[3] := c11.cross(c01).normalized();
    planes_n[4] := -dir;
    planes_o[4] := -(eye+dir*farplane).dot(planes_n[4]);

    for i := 0 to 3 do begin
      planes_o[i] := -(eye.dot(planes_n[i]));
    end;
    srenderer := bticeSceneRenderer.create;
    srenderer.init;
    srenderer.this:= self;
    if enableOcclusion then begin
      srenderer.ocb := ocb;
      btDbvt.collideOCL(pbp.m_sets[1].m_root,planes_n,planes_o,dir,acplanes,srenderer,true);
      btDbvt.collideOCL(pbp.m_sets[0].m_root,planes_n,planes_o,dir,acplanes,srenderer,true);
    end else begin
      btDbvt.collideKDOP(pbp.m_sets[1].m_root,planes_n,planes_o,acplanes,srenderer);
      btDbvt.collideKDOP(pbp.m_sets[0].m_root,planes_n,planes_o,acplanes,srenderer);
    end;
    visiblecount := srenderer.drawn;
    if showOcclusion and enableOcclusion then begin
      ratio := glutGet(GLUT_WINDOW_HEIGHT) / glutGet(GLUT_WINDOW_WIDTH);
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      glOrtho(-1,1,-1,1,-1,1);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();

      glMultMatrixf(@mm_array_buf);
      glDisable(GL_DEPTH_TEST);
      glDisable(GL_LIGHTING);
      //size   :=0.6;
      //orgx   :=0.3;
      //orgy   :=0.25;
      //left   :=orgx;
      //right  :=orgx+size;
      //top    :=orgy+size;
      //bottom :=orgy;
      ocb.drawBuffer(0.3,0.25,0.25+0.6,0.3+0.6);
    end;
    if showCulling then begin
      ratio := glutGet(GLUT_WINDOW_HEIGHT) / glutGet(GLUT_WINDOW_WIDTH);
      scale := 0.004;

      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      glOrtho(-1,1,-1*ratio,1*ratio,-1,1);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      glMultMatrixf(@mm_array_deb);
      glScalef(scale,scale,scale);

      glDisable(GL_DEPTH_TEST);
      glDisable(GL_LIGHTING);

      glBegin(GL_LINES);
      glColor4f(1,1,1,1);
      drenderer := bticeDebugRenderer.Create;
      if enableOcclusion then begin
        drenderer.ocb := ocb;
        drenderer.sid := 0;
        ocb.clear;
        btDbvt.collideOCL(pbp.m_sets[1].m_root,planes_n,planes_o,dir,acplanes,drenderer);
        btDbvt.collideOCL(pbp.m_sets[0].m_root,planes_n,planes_o,dir,acplanes,drenderer);
      end  else begin
        drenderer.ocb := nil;
        drenderer.sid := -1;
        btDbvt.collideKDOP(pbp.m_sets[1].m_root,planes_n,planes_o,acplanes,drenderer);
        btDbvt.collideKDOP(pbp.m_sets[0].m_root,planes_n,planes_o,acplanes,drenderer);
      end;
      drenderer.free;
      glEnd();
      glBegin(GL_LINES);
      glColor4f(1,1,1,1);
      glVertex3f(eye.x(),eye.y(),eye.z());
      glVertex3f(x00.x(),x00.y(),x00.z());
      glVertex3f(eye.x(),eye.y(),eye.z());
      glVertex3f(x10.x(),x10.y(),x10.z());
      glVertex3f(eye.x(),eye.y(),eye.z());
      glVertex3f(x01.x(),x01.y(),x01.z());
      glVertex3f(eye.x(),eye.y(),eye.z());
      glVertex3f(x11.x(),x11.y(),x11.z());
      glVertex3f(x00.x(),x00.y(),x00.z());
      glVertex3f(x10.x(),x10.y(),x10.z());
      glVertex3f(x10.x(),x10.y(),x10.z());
      glVertex3f(x11.x(),x11.y(),x11.z());
      glVertex3f(x11.x(),x11.y(),x11.z());
      glVertex3f(x01.x(),x01.y(),x01.z());
      glVertex3f(x01.x(),x01.y(),x01.z());
      glVertex3f(x00.x(),x00.y(),x00.z());
      glEnd();
    end;
  end;
  writeln('VISIBLECNT = ',visiblecount);
  //abort;
end;

procedure btBulletSAPCompleteBoxPruningTest.Select;
begin

end;

procedure btBulletSAPCompleteBoxPruningTest.Deselect;
begin

end;

procedure btBulletSAPCompleteBoxPruningTest.KeyboardCallback(key: byte; x,
  y: integer);
begin

end;

procedure btBulletSAPCompleteBoxPruningTest.MouseCallback(button, state, x,
  y: Integer);
begin

end;

procedure btBulletSAPCompleteBoxPruningTest.MotionCallback(x, y: integer);
begin

end;

procedure btBulletSAPCompleteBoxPruningTest.TWEAK_Setup;
var pbp : btDbvtBroadphase;
begin
  if not m_isdbvt then abort; // rethink
//  objectSpeed := 0.01; // TwAddVarRW(mBar, "Speed", TW_TYPE_FLOAT, &objectSpeed, " min=0.0 max=0.01 step=0.0001");

  mAmplitude  := 100;    //  TwAddVarRW(mBar, "Amplitude", TW_TYPE_FLOAT, &mAmplitude, " min=10.0 max=200.0 step=0.1");
  pbp         := btDbvtBroadphase(m_broadphase);
 // enableCulling   := True;   // TwAddVarRW(mBar, "Enable culling",TW_TYPE_BOOLCPP,&enableCulling,"");
 // enableOcclusion := True;   // TwAddVarRW(mBar, "Enable occlusion",TW_TYPE_BOOLCPP,&enableOcclusion,"");
 // showCulling     := True;   // TwAddVarRW(mBar, "Show culling",TW_TYPE_BOOLCPP,&showCulling,"");
 // showOcclusion   := True;   // TwAddVarRW(mBar, "Show occlusion",TW_TYPE_BOOLCPP,&showOcclusion,"");
 // cullFarPlane    := False;   // TwAddVarRW(mBar, "Cull far plane",TW_TYPE_BOOLCPP,&cullFarPlane,"");
  ocb.ocarea      := 0.00;   // TwAddVarRW(mBar, "OC Min area",TW_TYPE_FLOAT,&ocb.ocarea,"min=0.0 max=1.0 step=0.001");
  ocb.qrarea      := 0.00;   // TwAddVarRW(mBar, "QR Min area",TW_TYPE_FLOAT,&ocb.qrarea,"min=0.0 max=1.0 step=0.001");
  pbp.m_sets[0].m_lkhd := -1; // TwAddVarRW(mBar, "Dyn lkhd",TW_TYPE_INT32,&pbp->m_sets[0].m_lkhd,"min=-1 max=32");
  pbp.m_sets[1].m_lkhd := -1; // TwAddVarRW(mBar, "Fix lkhd",TW_TYPE_INT32,&pbp->m_sets[1].m_lkhd,"min=-1 max=32");
  pbp.m_dupdates       := 0; // TwAddVarRW(mBar, "Dyn opt/f(%)",TW_TYPE_INT32,&pbp->m_dupdates,"min=0 max=100");
  pbp.m_fupdates       := 0; // TwAddVarRW(mBar, "Fix opt/f(%)",TW_TYPE_INT32,&pbp->m_fupdates,"min=0 max=100");
  pbp.m_cupdates       := 0; // TwAddVarRW(mBar, "Cln opt/f(%)",TW_TYPE_INT32,&pbp->m_cupdates,"min=0 max=100");
  pbp.m_prediction     := 0;//                      TwAddVarRW(mBar, "Prediction",TW_TYPE_FLOAT,&pbp->m_prediction,"min=0.0 max=2.0 step=0.1");
//  pbp.m_deferedcollide := false;//                      TwAddVarRW(mBar, "Defered collide",TW_TYPE_BOOLCPP,&pbp->m_deferedcollide,"");
//  TWEAK_DeferredCollide(true);
  //                      TwAddVarRO(mBar, "Dyn leafs",TW_TYPE_INT32,&pbp->m_sets[0].m_leaves,"");
  //                      TwAddVarRO(mBar, "Fix leafs",TW_TYPE_INT32,&pbp->m_sets[1].m_leaves,"");
  //                      TwAddVarRO(mBar, "Updates ratio",TW_TYPE_FLOAT,&pbp->m_updates_ratio,"");
  //                      TwAddVarRO(mBar, "Visible",TW_TYPE_INT32,&visiblecount,"");
  //                      TwAddButton(mBar,"Normal mode",&NormalMode,m_broadphase,"");
  //                      TwAddButton(mBar,"Slow speed mode",&SlowSpeedMode,m_broadphase,"");
  //                      }
  //      }
  //      printf("SubMethod: %s\r\n",methodname);
end;

procedure btBulletSAPCompleteBoxPruningTest.TWEAK_DeferredCollide(const dc: boolean);
begin
  btDbvtBroadphase(m_broadphase).m_deferedcollide := dc;
end;

procedure btBulletSAPCompleteBoxPruningTest._DrawOBB(const obb: bticeOBB);
begin
 glPushMatrix();
  _SetupGLMatrix(obb.mCenter, obb.mRot);
  glScalef(obb.mExtents.x, obb.mExtents.y, obb.mExtents.z);
  glutSolidCube(2);
  glPopMatrix();
end;

procedure btBulletSAPCompleteBoxPruningTest._SetupGLMatrix(const pos: bticePoint; const rot: btMatrix3x3);
var glmat:array [0..15] of GLfloat;
    trans:btTransform;
begin
 trans.setBasis(rot);
 trans.setOrigin(pos);
 trans.getOpenGLMatrix(@glmat);
 glMultMatrixf(glmat);
end;

{ btOcclusionBuffer }

class function btOcclusionBuffer.WriteOCL_Process(var q: btScalar; const v: btscalar): boolean;
begin
  if q<v then q:=v;
  result := false;
end;

class function btOcclusionBuffer.QueryOCL_Process(const q: btScalar; const v: btScalar): boolean;
begin
  result := q<=v;
end;

constructor btOcclusionBuffer.create;
begin
  initialized :=false;
  neardist.Init(2,2,2);
  ocarea := 0;
  qrarea := 0;
  buffer := btFOSAlignedScalars.create;
end;

destructor btOcclusionBuffer.destroy;
begin
  buffer.free;
  inherited destroy;
end;

procedure btOcclusionBuffer.setup(const w, h: integer);
begin
  initialized := true;
  sizes[0]    := w;
  sizes[1]    := h;
  scales[0]   := w/2;
  scales[1]   := h/2;
  offsets[0]  := scales[0]+0.5;
  offsets[1]  := scales[1]+0.5;
  glGenTextures(1,@texture);
  Clear;
end;

procedure btOcclusionBuffer.clear;
begin
  buffer.resize(0);
  buffer.resize(sizes[0]*sizes[1],0);
end;

procedure btOcclusionBuffer.initialize;
var v   : array[0..3] of GLint;
    m,p : array [0..15] of GLdouble;
    i   : Integer;
begin
  if(not initialized) then begin
    setup(128,128);
  end;
  glGetIntegerv(GL_VIEWPORT,v);
  glGetDoublev(GL_MODELVIEW_MATRIX,m);
  glGetDoublev(GL_PROJECTION_MATRIX,p);
  for i:=0 to 15 do begin
    wtrs[i] := p[i];
  end;
  clear;
end;

type
  btFOSAlignedGLubyte=specialize FOS_GenericAlignedArray<GLubyte>;

procedure btOcclusionBuffer.drawBuffer(l, t, r, b: btScalar);
var data : array of byte;
    db   : Byte;
        i: Integer;
begin
  SetLength(data,buffer.Size);
  for i:=0 to buffer.size-1 do begin
    db      := trunc(32 / buffer.Idx[i]) mod 255;
    data[i] := db;
  end;
  glBindTexture(GL_TEXTURE_2D,texture);
  glDisable(GL_BLEND);
  glEnable(GL_TEXTURE_2D);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
 // glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE,sizes[0],sizes[1],0,GL_LUMINANCE,GL_UNSIGNED_BYTE,data.A[0]);
  glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE,sizes[0],sizes[1],0,GL_LUMINANCE,GL_UNSIGNED_BYTE,@data[0]);
  glBegin(GL_QUADS);
  glColor4ub(255,255,255,255);
  glTexCoord2f(0,0);glVertex2f(l,t);
  glTexCoord2f(1,0);glVertex2f(r,t);
  glTexCoord2f(1,1);glVertex2f(r,b);
  glTexCoord2f(0,1);glVertex2f(l,b);
  glEnd();
  glDisable(GL_TEXTURE_2D);
end;

function btOcclusionBuffer.transform(const x: btVector3): btVector4;
begin
  result[0] := x[0]*wtrs[0]+x[1]*wtrs[4]+x[2]*wtrs[8]+wtrs[12];
  result[1] := x[0]*wtrs[1]+x[1]*wtrs[5]+x[2]*wtrs[9]+wtrs[13];
  result[2] := x[0]*wtrs[2]+x[1]*wtrs[6]+x[2]*wtrs[10]+wtrs[14];
  result[3] := x[0]*wtrs[3]+x[1]*wtrs[7]+x[2]*wtrs[11]+wtrs[15];
end;

class function btOcclusionBuffer.project(const p: PbtVector4; const n: integer): boolean;
var i  : integer;
    iw : btScalar;
begin
  for i := 0 to n-1 do begin
    iw      := 1/p[i][3];
    p[i][2] := 1/p[i][3];
    p[i][0] := p[i][0] * p[i][2];
    p[i][1] := p[i][1] * p[i][2];
  end;
  result := true;
end;

//template <const int NP,typename POLICY>
//inline bool	clipDraw(	const btVector4* p,
//						btScalar minarea)
//	{
//	btVector4	o[NP*2];
//	const int	n=clip<NP>(p,o);
//	bool		earlyexit=false;
//	project(o,n);
//	for(int i=2;i<n;++i)
//		{
//		earlyexit|=draw<POLICY>(o[0],o[i-1],o[i],minarea);
//		}
//	return(earlyexit);
//	}

//function btOcclusionBuffer.queryOccluder3(const a, b, c: btVector3): boolean;
//var p : array [0..2] of btVector4;
//begin
//  abort;
//  p[0] :=   transform(a);
//  p[1] :=   transform(b);
//  p[2] :=   transform(c);
//  //const btVector4 p[]={transform(a),transform(b),transform(c)};
//  result := clipDraw3QOCL(p,qrarea);
//end;

 const d_array:array [0..23] of integer = ( 1,0,3,2,
                                            4,5,6,7,
                                            4,7,3,0,
                                            6,5,1,2,
                                            7,6,2,3,
                                            5,4,0,1 );

function btOcclusionBuffer.queryOccluder2(const c, e: btVector3): boolean;
var x : array [0..7] of btVector4;
    p : array [0..3] of btVector4;
    i : Integer;
begin

  x[0] := transform(btVector3.InitS(c[0]-e[0],c[1]-e[1],c[2]-e[2]));
  x[1] := transform(btVector3.InitS(c[0]+e[0],c[1]-e[1],c[2]-e[2]));
  x[2] := transform(btVector3.inits(c[0]+e[0],c[1]+e[1],c[2]-e[2]));
  x[3] := transform(btVector3.inits(c[0]-e[0],c[1]+e[1],c[2]-e[2]));
  x[4] := transform(btVector3.inits(c[0]-e[0],c[1]-e[1],c[2]+e[2]));
  x[5] := transform(btVector3.inits(c[0]+e[0],c[1]-e[1],c[2]+e[2]));
  x[6] := transform(btVector3.inits(c[0]+e[0],c[1]+e[1],c[2]+e[2]));
  x[7] := transform(btVector3.inits(c[0]-e[0],c[1]+e[1],c[2]+e[2]));
  for i:=0 to 7 do begin
    if (x[i][2]+x[i][3]) <= 0 then exit(true);
  end;
  i:=0;
  while i<24 do begin
    p[0] := x[d_array[i]];inc(i);
    p[1] := x[d_array[i]];inc(i);
    p[2] := x[d_array[i]];inc(i);
    p[3] := x[d_array[i]];inc(i);
    if clipDraw4WQOCL(p,qrarea,true) then begin
      exit(true);
    end;
  end;
  result := false;
end;

procedure btOcclusionBuffer.appendOccluder2(const c, e: btVector3);
var x : array [0..7] of btVector4;
    p : array [0..3] of btVector4;
    i : Integer;
begin
  x[0] := transform(btVector3.InitS(c[0]-e[0],c[1]-e[1],c[2]-e[2]));
  x[1] := transform(btVector3.InitS(c[0]+e[0],c[1]-e[1],c[2]-e[2]));
  x[2] := transform(btVector3.inits(c[0]+e[0],c[1]+e[1],c[2]-e[2]));
  x[3] := transform(btVector3.inits(c[0]-e[0],c[1]+e[1],c[2]-e[2]));
  x[4] := transform(btVector3.inits(c[0]-e[0],c[1]-e[1],c[2]+e[2]));
  x[5] := transform(btVector3.inits(c[0]+e[0],c[1]-e[1],c[2]+e[2]));
  x[6] := transform(btVector3.inits(c[0]+e[0],c[1]+e[1],c[2]+e[2]));
  x[7] := transform(btVector3.inits(c[0]-e[0],c[1]+e[1],c[2]+e[2]));
  i:=0;
  while i<=23 do begin
    p[0] := x[d_array[i]];inc(i);
    p[1] := x[d_array[i]];inc(i);
    p[2] := x[d_array[i]];inc(i);
    p[3] := x[d_array[i]];inc(i);
    clipDraw4WQOCL(p,ocarea,false);
  end;
end;

//function btOcclusionBuffer.clipDraw3QOCL(const p: PbtVector4; const minarea: btScalar): boolean;
//var o:array [0..2*3-1] of btVector4;
//    n,i:integer;
//    earlyexit:boolean;
//begin
//  n         := clip3(p,o);
//  earlyexit := false;
//  project(@o,n);
//  for i:=2 to n-1 do begin
//    earlyexit := earlyexit or drawWQOCL(o[0],o[i-1],o[i],minarea,true);
//  end;
//  result := earlyexit;
//end;

function btOcclusionBuffer.clipDraw4WQOCL(const p: PbtVector4; const minarea: btScalar;const query:boolean): boolean;
var o:array [0..2*4-1] of btVector4;
    n,i:integer;
    earlyexit,ex:boolean;
begin
  n         := clip4(p,o);
  earlyexit := false;
  project(@o,n);
  for i:=2 to n-1 do begin
   ex        :=  drawWQOCL(o[0],o[i-1],o[i],minarea,query);
   earlyexit :=  earlyexit or ex;
  end;
  result := earlyexit;
end;

//function btOcclusionBuffer.clipDraw3WOCL(const p: PbtVector4; const minarea: btScalar): boolean;
//var o:array [0..2*3-1] of btVector4;
//    n,i:integer;
//    earlyexit:boolean;
//begin
//  n         := clip3(p,o);
//  earlyexit := false;
//  project(@o,n);
//  for i:=2 to n-1 do begin
//    earlyexit := earlyexit or drawWQOCL(o[0],o[i-1],o[i],minarea,false);
//  end;
//  result := earlyexit;
//end;


//function btOcclusionBuffer.clip3(const pi: PbtVector4; const po: PbtVector4): integer;
//var s:array [0..2] of btScalar;
//    m,i,j,n:integer;
//    a,b : PbtVector4;
//    t   : btScalar;
//begin
//  m := 0;
//  for i:=0  to 2 do begin
//    s[i] := pi[i][2]+pi[i][3];
//    if s[i]<0 then begin
//      m := m + (1 SHL i);
//    end;
//  end;
//  if (m = ((1 SHL 3)-1)) then exit(0);
//  if m<>0 then begin
//    n := 0;
//    //for(int i=NP-1,j=0;j<NP;i=j++)
//    j := 0;
//    i := 3-1;
//    while (j<3) do begin
//      a := @pi[i];
//      b := @pi[j];
//      t := s[i] / (a^[3]+a^[2]-b^[3]-b^[2]);
//      if ((t>0) and (t<1)) then begin
//        po[n][0] := a^[0]+(b^[0]-a^[0])*t;
//        po[n][1] := a^[1]+(b^[1]-a^[1])*t;
//        po[n][2] := a^[2]+(b^[2]-a^[2])*t;
//        po[n][3] := a^[3]+(b^[3]-a^[3])*t;
//        inc(n);
//      end;
//      if (s[j]>0) then begin
//        po[n] :=b^;inc(n);
//      end;
//      i:=j;
//      inc(j);
//    end;
//    Result := n;
//  end;
//  for i := 0 to 3-1 do begin
//    po[i] := pi[i];
//  end;
//  Result := 3;
//end;
//
//
function btOcclusionBuffer.clip4(const pi: PbtVector4; const po: PbtVector4): integer;
var s       : array [0..3] of btScalar;
    m,i,j,n : integer;
    a,b     : PbtVector4;
    t       : btScalar;
begin
  m := 0;
  for i:=0  to 3 do begin
    s[i] := pi[i][2]+pi[i][3];
    if s[i]<0 then begin
      m := m + (1 SHL i);
    end;
  end;
  if (m = ((1 SHL 4)-1)) then exit(0);
  if m<>0 then begin
    n := 0;
    //for(int i=NP-1,j=0;j<NP;i=j++)
    j := 0;
    i := 4-1;
    while (j<4) do begin
      a := @pi[i];
      b := @pi[j];
      t := s[i] / (a^[3]+a^[2]-b^[3]-b^[2]);
      if ((t>0) and (t<1)) then begin
        po[n][0] := a^[0]+(b^[0]-a^[0])*t;
        po[n][1] := a^[1]+(b^[1]-a^[1])*t;
        po[n][2] := a^[2]+(b^[2]-a^[2])*t;
        po[n][3] := a^[3]+(b^[3]-a^[3])*t;
        inc(n);
      end;
      if (s[j]>0) then begin
        po[n] :=b^;inc(n);
      end;
      i:=j;
      inc(j);
    end;
    Result := n;
  end;
  for i := 0 to 4-1 do begin
    po[i] := pi[i];
  end;
  Result := 4;
end;


function btOcclusionBuffer.drawWQOCL(const a, b, c: btVector4; const minarea: btScalar ; const query:boolean): boolean;
var a2,ia,dzx,dzy,v : btScalar;
    x,y,dx,dy,cc    : array [0..2] of integer;
    z,dz            : array [0..2] of btScalar;
    mix,mxx,miy,
    mxy,width,
    height,aa,ix,iy : integer;
    scan            : PbtScalar;

begin
  a2 := (b-a).cross(c-a)[2];
  if a2>0 then begin
    if a2<minarea then exit(true);
    x[0] := trunc(a.x()*scales[0]+offsets[0]);
    x[1] := trunc(b.x()*scales[0]+offsets[0]);
    x[2] := trunc(c.x()*scales[0]+offsets[0]);

    y[0] := trunc(a.y()*scales[1]+offsets[1]);
    y[1] := trunc(b.y()*scales[1]+offsets[1]);
    y[2] := trunc(c.y()*scales[1]+offsets[1]);

    z[0]   := a.z;
    z[1]   := b.z;
    z[2]   := c.z;

    mix    := btMax(0,btMin(x[0],btMin(x[1],x[2])));
    mxx    := btMin(sizes[0],1+btMax(x[0],btMax(x[1],x[2])));
    miy    := btMax(0,btMin(y[0],btMin(y[1],y[2])));
    mxy    := btMin(sizes[1],1+btMax(y[0],btMax(y[1],y[2])));

    width  := mxx-mix;
    height := mxy-miy;
    if (width*height)>0 then begin
      dx[0] := y[0]-y[1];
      dx[1] := y[1]-y[2];
      dx[2] := y[2]-y[0];
      dy[0] := x[1]-x[0]-dx[0]*width;
      dy[1] := x[2]-x[1]-dx[1]*width;
      dy[2] := x[0]-x[2]-dx[2]*width;
      aa    := x[2]*y[0]+x[0]*y[1]-x[2]*y[1]-x[0]*y[2]+x[1]*y[2]-x[1]*y[0];
      ia    := 1 / aa;
      dzx   := ia*(y[2]*(z[1]-z[0])+y[1]*(z[0]-z[2])+y[0]*(z[2]-z[1]));
      dzy   := ia*(x[2]*(z[0]-z[1])+x[0]*(z[1]-z[2])+x[1]*(z[2]-z[0]))-(dzx*width);
      cc[0] := miy*x[1]+mix*y[0]-x[1]*y[0]-mix*y[1]+x[0]*y[1]-miy*x[0];
      cc[1] := miy*x[2]+mix*y[1]-x[2]*y[1]-mix*y[2]+x[1]*y[2]-miy*x[1];
      cc[2] := miy*x[0]+mix*y[2]-x[0]*y[2]-mix*y[0]+x[2]*y[0]-miy*x[2];
      v     := ia*((z[2]*cc[0])+(z[0]*cc[1])+(z[1]*cc[2]));
      for iy := miy to mxy-1 do begin
        for ix := mix to mxx-1 do begin
          if (cc[0]>=0) and (cc[1]>=0) and (cc[2]>=0) then begin
            scan  := buffer.A[iy*sizes[1] + ix ];
            if query then begin
              if scan^<=v then exit(true);
            end else begin
              if (scan^ < v) then begin
                scan^ := v;
              end;
            end;
          end;
          cc[0] += dx[0];
          cc[1] += dx[1];
          cc[2] += dx[2];
          v     += dzx;
        end;
        cc[0] += dy[0];
        cc[1] += dy[1];
        cc[2] += dy[2];
        v     += dzy;
      end;
    end;
  end;
  Result := false;
end;



{ bticeAABB }

procedure bticeAABB.Init;
begin
  mCenter.init(0,0,0);
  mExtents.Init(0,0,0);
end;

procedure bticeAABB.SetCenterExtents(const c, e: bticePoint);
begin
  mCenter  := c;
  mExtents := e;
end;

procedure bticeAABB.GetExtents(var extents: bticePoint);
begin
  extents := mExtents;
end;

procedure bticeAABB.GetCenter(var center: bticePoint);
begin
 center := mCenter;
end;

{ bticePoint }


{ bticePAIRS }

procedure bticePAIRS.ResetPairs;
begin
  // reset
end;

{ btProfiler }

procedure btProfiler.Init;
begin
  cl.init;
end;

procedure btProfiler.Start;
begin
  cl.reset;
 // mcounter0 := cl.getTimeMilliseconds;
end;

procedure btProfiler.EndP;
begin
  //mcounter1 := cl.getTimeMilliseconds;
  mcounter0:=cl.getTimeMicroseconds;
end;

procedure btProfiler.Reset;
begin
  mCycles    := 0;
  mTime      := 0;
  mNbQueries := 0;
  mMsTime    := 0.0;
end;

procedure btProfiler.Accum;
begin
 // mms   := mcounter1 - mcounter0;
  mTime := mTime + mcounter0;
  inc(mNbQueries);
  if mNbQueries=100 then begin
    mNbQueries :=1;
    mTime := mcounter0;
  end;
  mMsTime := mTime / mNbQueries;
end;

{ bticeOBB }

procedure bticeOBB.init;
begin
  //mCenter.zero;
  //mExtents.zero;
  //mRot.setIdentity;
end;

end.
