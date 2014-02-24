unit RayTraceDemo;

{$i ../fos_bullet.inc}

interface

uses
  Classes, SysUtils,btCollisionShapes,btDispatch,btNarrowphase,FOS_AlignedArray,btLinearMath,glutstuff,gl,glu,glut,gldebugfontmap;

const  maxNumObjects   =  4;
          numObjects   =  3;
          //screenWidth  = 128;
          //screenHeight = 64;
          screenWidth  = 256;
          screenHeight = 256;

var  yaw,pitch,roll  : btScalar;
     shapePtr        : array [0..maxNumObjects-1] of btConvexShape;
     transforms      : array [0..NumObjects-1] of btTransform;
//     m_collisionWorld : btCollisionWorld;
//     debugDrawer    : btGLDebugDrawer;


///Raytracer shows the inner working of the ray casting, using ray tracing rendering into a texture.
type

  { btRaytracer }

  btRaytracer = class(btGlutDemoApplication)
    m_collisionConfiguration : btDefaultCollisionConfiguration;
    m_dispatcher             : btCollisionDispatcher;
//    m_overlappingPairCache   : btAxisSweep3;
    m_collisionWorld         : btCollisionWorld;
    m_overlappingPairCache   : btBroadphaseInterface;
    m_initialized            : Boolean;

    raytracepicture : btRenderTexture;
    glTextureId     : GLuint;
    myCone          : btConeShape;
    mySphere        : btSphereShape;
    myBox           : btBoxShape;
    simplexSolver   : btVoronoiSimplexSolver;


   public
    constructor Create;
    destructor  Destroy;override;
    procedure   initPhysics; override;
    procedure   clientMoveAndDisplay; override;
    procedure   displayCallback; override;
    ///worldRaytest performs a ray versus all objects in a collision world, returning true is a hit is found (filling in worldNormal and worldHitPoint)
    function worldRaytest(const rayFrom,rayTo : btVector3 ;var worldNormal,worldHitPoint : btVector3):boolean;
    ///singleObjectRaytest performs a ray versus one collision shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
    function singleObjectRaytest(const rayFrom,rayTo : btVector3 ; var worldNormal,worldHitPoint : btVector3;const x,y:integer):boolean;
    ///lowlevelRaytest performs a ray versus convex shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
    function lowlevelRaytest(const rayFrom,rayTo : btVector3 ; var worldNormal,worldHitPoint : btVector3;const x,y:integer):boolean;
  end;





implementation


{ btRaytracer }

constructor btRaytracer.Create;
begin
  inherited Create;
  myCone   := btConeShape.Create(1,1);
  mySphere := btSphereShape.Create(1);
  myBox    := btBoxShape.Create(btVector3.InitSameS(1));
  simplexSolver := btVoronoiSimplexSolver.Create;
end;

destructor btRaytracer.Destroy;
var i   : integer;
    obj : btCollisionObject;
    ar  : btCollisionObjectArray;
begin
  myBox.free;
  mySphere.free;
  myCone.free;
  simplexSolver.Free;

  ar := m_collisionWorld.getCollisionObjectArray;
  for i := m_collisionWorld.getNumCollisionObjects-1 downto 0 do begin
    obj := ar.A[i]^;
    m_collisionWorld.removeCollisionObject( obj );
    obj.free;
  end;

  m_collisionWorld.Free;
  m_overlappingPairCache.Free;
  m_dispatcher.Free;
  m_collisionConfiguration.Free;
  raytracePicture.Free;
  inherited Destroy;
end;

procedure btRaytracer.initPhysics;
var pos:btVector3;
    orn:btQuaternion;
    i: Integer;
    worldMin,worldMax : btVector3;
    obj:btCollisionObject;
begin
  m_ele := 0;
  raytracePicture := btRenderTexture.create(screenWidth,screenHeight);
  myCone.setMargin(0.2);
  //choose shape
  shapePtr[0] := myCone;
  shapePtr[1] := mySphere;
  shapePtr[2] := myBox;

  for i:=0 to numObjects-1 do begin
    transforms[i].setIdentity();
    pos.Init(0,0,-(2.5* numObjects * 0.5)+i*2.5);
    transforms[i].setIdentity;
    transforms[i].setOrigin(pos);
    if i<2 then begin
      orn.setEuler(yaw,pitch,roll);
      transforms[i].setRotation(orn);
    end;
  end;

  m_collisionConfiguration := btDefaultCollisionConfiguration.Create(btDefaultCollisionConstructionInfo.Init);
  m_dispatcher             := btCollisionDispatcher.Create(m_collisionConfiguration);
  worldMin.Init (-1000,-1000,-1000);
  worldMax.init (1000,1000,1000);

//  m_overlappingPairCache := btAxisSweep3.create(worldMin,worldMax);
//  m_overlappingPairCache := btSimpleBroadphase.create;
  m_overlappingPairCache := btDbvtBroadphase.create;

  m_collisionWorld       := btCollisionWorld.create(m_dispatcher,m_overlappingPairCache,m_collisionConfiguration);
  for i:=0 to numObjects-1 do begin
    obj := btCollisionObject.Create;
    obj.setCollisionShape(shapePtr[i]);
    obj.setWorldTransform(transforms[i]);
    m_collisionWorld.addCollisionObject(obj);
  end;
end;

procedure btRaytracer.clientMoveAndDisplay;
begin
  displayCallback;
end;

procedure btRaytracer.displayCallback;
var pos,rayFrom,rayForward : btVector3;
    rgba : btVector4;
    orn : btQuaternion;
    i   : Integer;
    top,bottom,tanFov,fov,farPlane,
    nearPlane : btScalar;
    hor,vertical : btVector3;
    rayToCenter,dHor,dVert : btVector3;
    rayFromTrans:btTransform;
    x: Integer;
    y: Integer;
    buffer:string;
    ptr : pointer;
    rayTo : btVector3;
    colObjWorldTransform : btTransform;
    mode  : integer;
    worldNormal,worldPoint : btVector3;
    hasHit : Boolean;
    lightVec0,lightVec1 : btScalar;
begin
  updateCamera;

  for i := 0 to numObjects-1 do begin
    transforms[i].setIdentity;
    pos.Init(0,0,-(2.5* numObjects * 0.5)+i*2.5);
    transforms[i].setOrigin(pos);
    if i < 2 then begin
      orn.setEuler(yaw,pitch,roll);
      transforms[i].setRotation(orn);
    end;
  end;

  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);
  glDisable(GL_LIGHTING);
  if not m_initialized then begin
    m_initialized := true;
    glGenTextures(1, @glTextureId);
  end;

  glBindTexture(GL_TEXTURE_2D,glTextureId );

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_BLEND);

  rgba.Init4(1,0,0,0.5);
  top       := 1;
  bottom    := -1;
  nearPlane := 1;

  tanFov  := (top-bottom)*0.5 / nearPlane;
  fov     := 2.0 * btAtan(tanFov);
  rayFrom := getCameraPosition();
  rayForward := getCameraTargetPosition-getCameraPosition;
  rayForward.normalize;
  farPlane := 600;
  rayForward*= farPlane;

  vertical.init(0,1,0);
  hor := rayForward.cross(vertical);
  hor.normalize;
  vertical := hor.cross(rayForward);
  vertical.normalize;

  tanfov   := bttan(0.5*fov);
  hor      *= 2 * farPlane * tanfov;
  vertical *= 2 * farPlane * tanfov;

  rayToCenter := rayFrom + rayForward;
  dHor        := hor * 1/screenWidth;
  dVert       := vertical * 1/screenHeight;

  rayFromTrans.setIdentity;
  rayFromTrans.setOrigin(rayFrom);

  ///clear texture
  for x:=0 to screenWidth-1 do begin
    for y:=0 to screenHeight-1 do begin
      rgba.init4(0.2,0.2,0.2,1);
      raytracePicture.setPixel(x,y,rgba);
    end;
  end;
 {$if 1}
  colObjWorldTransform.setIdentity;
  mode := 2;
  for x:=0 to screenWidth-1  do begin
    for y := 0 to screenHeight-1 do begin
      rayTo := rayToCenter - 0.5 * hor + 0.5 * vertical;
      rayTo += x * dHor;
      rayTo -= y * dVert;
      worldNormal.zero;
      worldPoint.zero;
      hasHit := false;
      hashit:=false;
      case  mode of
        0: hasHit := lowlevelRaytest(rayFrom,rayTo,worldNormal,worldPoint,x,y);
        1: hasHit := singleObjectRaytest(rayFrom,rayTo,worldNormal,worldPoint,x,y);
        2: hasHit := worldRaytest(rayFrom,rayTo,worldNormal,worldPoint);
      end;
      if hasHit then begin
        lightVec0 := worldNormal.dot(btVector3.inits(0,-1,-1));//0.4f,-1.f,-0.4f));
        lightVec1 := worldNormal.dot(btVector3.inits(-1,0,-1));//-0.4f,-1.f,-0.4f));
        rgba.init4(lightVec0,lightVec1,0,1);
        rgba.setMin(btVector3.initsames(1));
        rgba.setMax(btVector3.initsames(0.2));
        rgba[3] := 1;
        raytracePicture.setPixel(x,y,rgba);
      end else begin
        rgba := raytracePicture.getPixel(x,y);
      end;
      if rgba.length2=0 then begin
        raytracePicture.setPixel(x,y,btVector4.Init4S(1,1,1,1));
      end;
    end;
  end;
 {$endif}

  buffer:=format('%d rays',[screenWidth*screenHeight*numObjects]);
  raytracePicture.grapicalPrintf(buffer,@sFontData,0,0);//&BMF_font_helv10,0,10);


  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glFrustum(-1.0,1.0,-1.0,1.0,3,2020.0);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();                                                                     // reset The Modelview Matrix
  glTranslatef(0.0,0.0,-3.1);                                                // Move Into The Screen 5 Units

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D,glTextureId );

  ptr := raytracePicture.getBuffer;
  glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,raytracePicture.getWidth,raytracePicture.getHeight,0,GL_RGBA,GL_UNSIGNED_BYTE,ptr);

  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColor4f (1,1,1,1); // alpha=0.5=half visible

  glBegin(GL_QUADS);
  glTexCoord2f(0.0, 0.0);
  glVertex2f(-1,1);
  glTexCoord2f(1.0, 0.0);
  glVertex2f(1,1);
  glTexCoord2f(1.0, 1.0);
  glVertex2f(1,-1);
  glTexCoord2f(0.0, 1.0);
  glVertex2f(-1,-1);
  glEnd();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);


  glDisable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);

  btGL_ShapeDrawer.drawCoordSystem;

  glPushMatrix();
  glPopMatrix();

  pitch += 0.005;
  yaw   += 0.01;
  m_azi += 1;

  glFlush();
  glutSwapBuffers();
end;

type

 { btAllRayResultCallback }

 btAllRayResultCallback=class(btCW_RayResultCallback)
   m_rayFromWorld,   //used to calculate hitPointWorld from hitFraction
   m_rayToWorld,
   m_hitNormalWorld,
   m_hitPointWorld : btVector3;
   procedure   Init(const   rayFromWorld,rayToWorld : btVector3);
   function    addSingleResult(const rayResult: btCW_LocalRayResult; const normalInWorldSpace: Boolean): btScalar; override;
  end;

{ btAllRayResultCallback }

procedure btAllRayResultCallback.Init(const rayFromWorld, rayToWorld: btVector3);
begin
  m_rayFromWorld := rayFromWorld;
  m_rayToWorld   := rayToWorld;
end;

function btAllRayResultCallback.addSingleResult(const rayResult: btCW_LocalRayResult; const normalInWorldSpace: Boolean): btScalar;
begin
  //caller already does the filter on the m_closestHitFraction
  btAssert(rayResult.m_hitFraction <= m_closestHitFraction);
  m_closestHitFraction := rayResult.m_hitFraction;
  m_collisionObject    := rayResult.m_collisionObject;
  if normalInWorldSpace then begin
    m_hitNormalWorld := rayResult.m_hitNormalLocal;
  end else begin
    m_hitNormalWorld := m_collisionObject.getWorldTransformP^.getBasisV^*rayResult.m_hitNormalLocal; ///need to transform normal into worldspace
  end;
  m_hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
  Result := 1;
end;

{$HINTS OFF}
function btRaytracer.worldRaytest(const rayFrom, rayTo: btVector3; var worldNormal, worldHitPoint: btVector3): boolean;
var resultCallback : btAllRayResultCallback;
begin
  resultCallback :=btAllRayResultCallback.create;
  resultCallback.init (rayFrom,rayTo);
  //      btCollisionWorld::ClosestRayResultCallback resultCallback(rayFrom,rayTo);
  m_collisionWorld.rayTest(rayFrom,rayTo,resultCallback);
  if resultCallback.hasHit then begin
    worldNormal := resultCallback.m_hitNormalWorld;
    resultCallback.Free;
    exit(true);
  end;
  resultCallback.Free;
  result := false;
end;
{$HINTS ON}

{$HINTS OFF}
function btRaytracer.singleObjectRaytest(const rayFrom, rayTo: btVector3; var worldNormal, worldHitPoint: btVector3; const x, y: integer): boolean;
var resultCallback    : btCW_ClosestRayResultCallback;
    //closestHitResults : btScalar;
    hasHit: Boolean;
    //rayResult : btConvexCastResult ;
    rayFromTrans,rayToTrans : btTransform;
    pointShape              : btSphereShape;
    aabbMin,aabbMax         : btVector3;
    hitLambda               : btScalar;
    hitNormal               : btVector3;
    tmpObj                  : btCollisionObject;
    s                       : integer;
begin
//  closestHitResults := 1;

  resultCallback := btCW_ClosestRayResultCallback.create(rayFrom,rayTo);
  hasHit         := false;
  pointShape     := btSphereShape.Create(0);

  rayFromTrans.setIdentity;
  rayToTrans.setIdentity;
  rayFromTrans.setOrigin(rayFrom);
  rayToTrans.setOrigin(rayTo);
  for s :=0 to numObjects-1 do begin
    //comment-out next line to get all hits, instead of just the closest hit
    //resultCallback.m_closestHitFraction = 1.f;
    //do some culling, ray versus aabb
    shapePtr[s].getAabb(transforms[s],aabbMin,aabbMax);
    hitLambda := 1;
    tmpObj := btCollisionObject.Create;
    tmpObj.setWorldTransform(transforms[s]);
    if (btRayAabb(rayFrom,rayTo,aabbMin,aabbMax,hitLambda,hitNormal)) then begin
     btCollisionWorld.rayTestSingle(rayFromTrans,rayToTrans, tmpObj, shapePtr[s], transforms[s], resultCallback);
     if resultCallback.hasHit then begin
        //float fog = 1.f - 0.1f * rayResult.m_fraction;
        resultCallback.m_hitNormalWorld.normalize;//.m_normal.normalize();
        worldNormal   := resultCallback.m_hitNormalWorld;
        //worldNormal = transforms[s].getBasis() *rayResult.m_normal;
        worldNormal.normalize;
        hasHit := true;
     end;
    end;
    tmpObj.Free;
  end;
  Result := hasHit;
  pointShape.Free;
  resultCallback.Free;
end;
{$HINTS ON}

{$HINTS OFF}
function btRaytracer.lowlevelRaytest(const rayFrom, rayTo: btVector3; var worldNormal, worldHitPoint: btVector3; const x, y: integer): boolean;
var //resultCallback    : btCW_ClosestRayResultCallback;
    closestHitResults : btScalar;
    hasHit: Boolean;
    rayResult : btConvexCastResult ;
    rayFromTrans,rayToTrans : btTransform;
    pointShape              : btSphereShape;
    aabbMin,aabbMax         : btVector3;
    hitLambda               : btScalar;
    hitNormal               : btVector3;
    tmpObj                  : btCollisionObject;
    s                       : integer;
    convexCaster            : btSubsimplexConvexCast;
begin
  closestHitResults := 1;
  hasHit            := false;
  pointShape        := btSphereShape.Create(0);
  rayFromTrans.setIdentity;
  rayToTrans.setIdentity;
  rayFromTrans.setOrigin(rayFrom);
  rayToTrans.setOrigin(rayTo);
  rayResult := btConvexCastResult.create;
  for s:=0 to numObjects-1 do begin
    //do some culling, ray versus aabb
    shapePtr[s].getAabb(transforms[s],aabbMin,aabbMax);
    hitLambda := 1;
    tmpObj := btCollisionObject.Create;
    tmpObj.setWorldTransform(transforms[s]);
    if btRayAabb(rayFrom,rayTo,aabbMin,aabbMax,hitLambda,hitNormal) then begin
      //reset previous result   //choose the continuous collision detection method
      convexCaster := btSubsimplexConvexCast.Create(pointShape,shapePtr[s],simplexSolver);
      //btGjkConvexCast convexCaster(&pointShape,shapePtr[s],&simplexSolver);
      //btContinuousConvexCollision convexCaster(&pointShape,shapePtr[s],&simplexSolver,0);
      if convexCaster.calcTimeOfImpact(rayFromTrans,rayToTrans,transforms[s],transforms[s],rayResult) then begin
        if rayResult.m_fraction < closestHitResults then begin
          closestHitResults := rayResult.m_fraction;
          worldNormal       := transforms[s].getBasisV^*rayResult.m_normal;
          hasHit            := true;
          worldNormal.normalize;
        end;
      end;
      convexCaster.Free;
      hasHit:=true;
    end;
    tmpObj.Free;
  end;
  pointShape.free;
  rayResult.Free;
  Result := hasHit;
end;
{$HINTS ON}


end.

