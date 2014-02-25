program CollisionDemos;

{$mode objfpc}{$H+}

uses
  //cmem,
  {$IFDEF UNIX}
  cthreads,
  {$ENDIF}
  math,
  Classes, sysutils, GL,  GLUT, glutstuff,btLinearMath,
  CollisionInterfaceDemo, SimplexDemo, LinearConvexCastDemo,ContinuosConvexCollisionDemo,
  RayTraceDemo,BasicDemo,
  {$IFDEF WITH_OPENCL}
  OpenCLDemo,
  {$ENDIF}
  CDTestFrameworkDemo;

 var
   gcollInterfaceDemo : btCollisionInterfaceDemo;
   gsimplexDemo       : btSimplexDemo;
   glinconvexDemo     : btLinearConvexCastDemo;
   graytraceDemo      : btRaytracer;
   {$IFDEF WITH_OPENCL}
   gopenCLDemo        : btOpenCLDemo;
   {$ENDIF}
   gcontDemo          : btContinuousConvexCollisionDemo;
   gBasicDemo         : btBasicDemo;
   gCDFrameworkDemo   : btCDTestFrameWorkDemo;


  procedure _CollisionMain(screenWidth,screenHeight: integer;const param:string);
  begin
    gcollInterfaceDemo := btCollisionInterfaceDemo.create(param);
    gcollInterfaceDemo.initPhysics;
    gcollInterfaceDemo.clientResetScene;
    glutmain(@argc, argv,screenWidth,screenHeight,'Collision Interface Demo',gcollInterfaceDemo);
    glutMainLoop;
  end;

  procedure _SimplexDemo(screenWidth,screenHeight: integer);
  begin
    gsimplexDemo := btSimplexDemo.Create;
    gsimplexDemo .initPhysics;
    glutmain(@argc, argv,screenWidth,screenHeight,'Simplex Demo',gsimplexDemo);
    glutMainLoop;
  end;

  procedure _LinearConvexCastDemo(screenWidth,screenHeight: integer);
  begin
    glinconvexDemo := btLinearConvexCastDemo.Create;
    glinconvexDemo.initPhysics;
    glutmain(@argc, argv,screenWidth,screenHeight,'Linear Convex Cast Demo',glinconvexDemo);
    glutMainLoop;
  end;

  procedure _Raytracer(screenWidth,screenHeight: integer);
  begin
    graytraceDemo := btRaytracer.Create;
    graytraceDemo.initPhysics;
    graytraceDemo.setCameraDistance(6);
    glutmain(@argc, argv,screenWidth,screenHeight,'Raytracer Demo',graytraceDemo);
    glutMainLoop;
  end;

{$IFDEF WITH_OPENCL}
  procedure _OpenCLDemo();
  begin
    gopenCLDemo := btOpenCLDemo.Create;
    glutmain(@argc, argv,screenWidth,screenHeight,'OpenCL Demo',gopenCLDemo);
    glutMainLoop;
  end;
{$ENDIF}

  procedure _ContinuousConvexCollision(screenWidth,screenHeight: integer);
  begin
    gcontDemo := btContinuousConvexCollisionDemo.Create('');
    gcontDemo.setCameraDistance(40);
    gcontDemo.initPhysics;
    glutmain(@argc, argv,screenWidth,screenHeight,'Continuous Convex Collision Demo',gcontDemo);
    glutMainLoop;
  end;

  procedure _BasicDemo(screenWidth,screenHeight: integer);
  begin
    gBasicDemo := btBasicDemo.Create;
    gBasicDemo.setCameraDistance(40);
    gBasicDemo.initPhysics;
    gBasicDemo.getDynamicsWorld.setDebugDrawer(gBasicDemo.debugDrawer);
    glutmain(@argc, argv,screenWidth,screenHeight,'Continuous Convex Collision Demo',gBasicDemo);
    glutMainLoop;
  end;

  procedure _CDTestFrameWork(screenWidth,screenHeight: integer);
  begin
    gCDFrameworkDemo := btCDTestFrameWorkDemo.Create;
    gCDFrameworkDemo.setCameraDistance(40);
    gCDFrameworkDemo.initPhysics;
    //gCDFrameworkDemo.getDynamicsWorld.setDebugDrawer(gCDFrameworkDemo.debugDrawer);
    glutmain(@argc, argv,screenWidth,screenHeight,'CD TestFrameWork Demo',gCDFrameworkDemo,1);
    glutMainLoop;
  end;


  procedure myexit;
  begin
    gcollInterfaceDemo.Free;
    gsimplexDemo.free;
    glinconvexDemo.free;
    graytraceDemo.Free;
    {$IFDEF WITH_OPENCL}
    gopenCLDemo.Free;
    {$ENDIF}
    gcontDemo.Free;
    gBasicDemo.Free;
  end;

 var i:integer;
     x,y:integer;

begin
  writeln('FRE Bullet Demos V0.03');
  SetPrecisionMode(pmSingle);
  SetRoundMode(rmNearest);
  writeln('Roundmode ',GetRoundMode,' PM=',GetPrecisionMode);
  AddExitProc(@myexit);
  x:=1024;
  y:=768;
  i:=StrToIntDef(paramstr(1),-1);
  //i:=0; // DEBUG
  case i of
    0 : _CollisionMain(x,y,paramstr(2));
    1 : _SimplexDemo(x,y);
    2 : _LinearConvexCastDemo(x,y);
    3 : _Raytracer(x,y);
    4 : _ContinuousConvexCollision(x,y);
    5 : _Basicdemo(x,y);
    6 : _CDTestFramework(x,y);
{$IFDEF WITH_OPENCL}
    10 : _OpenCLDemo;
{$ENDIF}
  else
    writeln('Usage : ',paramstr(0)+' demonumber (0-6, 10) param');
  end;
end.