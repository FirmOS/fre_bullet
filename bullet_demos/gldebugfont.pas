unit gldebugfont;

{$mode objfpc}{$H+}

interface

uses  Classes, SysUtils,btLinearMath,gl,glu,gldebugfontmap;

procedure GLDebugDrawStringInternal (const x,y : integer ;const txt:string;const rgb:btVector3; const enableBlend : Boolean ; const spacing : integer);
procedure GLDebugDrawStringInternal (const x,y : integer ;const txt:string;const rgb:btVector3);
procedure GLDebugDrawString(const x,y : integer ;const txt:string);
procedure GLDebugResetFont(const screenWidth,screenHeight : integer);

//extern unsigned char sFontData[];
var sTexturesInitialized : boolean = false;
    sTexture             : Gluint  = GLuint(-1);
    sScreenWidth         : integer = -1;
    sScreenHeight        : integer = -1;

implementation

{$define USE_ARRAYS}

procedure GLDebugDrawStringInternal(const x, y: integer; const txt: string; const rgb: btVector3; const enableBlend: Boolean; const spacing: integer);
var cx,cy : btScalar;
    verts : array [0..11] of btScalar;
    uv_texcoords : array [0..7] of btScalar;
    ch           : Char;
    i            : integer;
begin
  if not sTexturesInitialized then begin
     GLDebugResetFont(sScreenWidth,sScreenHeight);
  end;
  if txt<>'' then begin
    glColor4f(rgb.getX(),rgb.getY(),rgb.getZ(),1);
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glDisable(GL_TEXTURE_GEN_S);
    glDisable(GL_TEXTURE_GEN_T);
    glDisable(GL_TEXTURE_GEN_R);
    glEnable(GL_TEXTURE_2D);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE);
    glDepthFunc (GL_LEQUAL);
    if enableBlend then begin
      glEnable(GL_BLEND);
    end else begin
      glDisable(GL_BLEND);
    end;
    glEnable (GL_DEPTH_TEST);
    glBindTexture(GL_TEXTURE_2D, sTexture);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0,sScreenWidth,0,sScreenHeight,-1,1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(btScalar(x),btScalar(sScreenHeight - y),btScalar(0));

{$ifdef USE_ARRAYS}
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState (GL_TEXTURE_COORD_ARRAY);
{$endif}
    //GLfloat verts[] ={
    //        0.0f, 1.0f, 0.0f,
    //        -1.0f, -1.0f, 0.0f,
    //        1.0f, -1.0f, 0.0f,
    //        0.f,0.f,0.f
    //};
    //GLfloat uv_texcoords[] = {
    //        0,0,
    //        0,0,
    //        0,0,
    //        0,0
    //};
    verts[0] := 0;    verts[1]  := 0;    verts[2]  := 0;
    verts[3] := 16-1; verts[4]  := 0;    verts[5]  := 0;
    verts[6] := 16-1; verts[7]  := 16-1; verts[8]  := 0;
    verts[9] := 0;    verts[10] := 16-1; verts[11] := 0;

    for i:=0 to Length(txt)-1 do begin
      ch := char(ord(txt[i+1])-32);
      if ord(ch)>=0 then begin
      cx := btScalar(ord(ch) mod 16) * btScalar(1/16);
      cy := btScalar(ord(ch) div 16) * btScalar(1/16);
      uv_texcoords[0] := cx                   ; uv_texcoords[1] := btScalar(1-cy-1/16);
      uv_texcoords[2] := btScalar(cx+1/16)    ; uv_texcoords[3] := btScalar(1-cy-1/16);
      uv_texcoords[4] := btScalar(cx+1/16)    ; uv_texcoords[5] := btScalar(1-cy);
      uv_texcoords[6] := cx                   ; uv_texcoords[7] := btScalar(1-cy);
    {$ifdef USE_ARRAYS}
      glTexCoordPointer(2,GL_FLOAT,0,@uv_texcoords);
      glVertexPointer(3, GL_FLOAT, 0,@verts);
      glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    {$else}
      glBegin(GL_QUADS);
      glTexCoord2f(cx,1-cy-1./16.f);
      glVertex2i(0,0);
      glTexCoord2f(cx+1./16.f,1-cy-1./16.f);
      glVertex2i(16 - 1,0);
      glTexCoord2f(cx+1./16.f,1-cy);
      glVertex2i(16 - 1,16 -1);
      glTexCoord2f(cx,1-cy);
      glVertex2i(0,16 -1);
      glEnd();
    {$endif}
      glTranslatef(spacing,0,0);
    end;
    end;

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
{$if 1}
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE);
    glDepthFunc (GL_LEQUAL);
    glDisable(GL_BLEND);
    glDisable(GL_TEXTURE_2D);

    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glScalef(btScalar(0.025),btScalar(0.025),btScalar(0.025));
{$endif}
    glMatrixMode(GL_MODELVIEW);
{$ifdef USE_ARRAYS}
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState (GL_TEXTURE_COORD_ARRAY);
{$endif}
    //glDisable(GL_TEXTURE_2D);
  end;
end;

procedure GLDebugDrawStringInternal(const x, y: integer; const txt: string; const rgb: btVector3);
begin
  GLDebugDrawStringInternal(x,y,txt,rgb,true,10);
end;

procedure GLDebugDrawString(const x, y: integer; const txt: string);
begin
  GLDebugDrawStringInternal(x,y,txt,btVector3.inits(1,1,1));
end;

procedure GLDebugResetFont(const screenWidth, screenHeight: integer);
begin
  if (sScreenWidth = screenWidth) and (sScreenHeight = screenHeight) then exit;

  sScreenWidth  := screenWidth;
  sScreenHeight := screenHeight;

  if not sTexturesInitialized then begin
    sTexturesInitialized := true;
    glGenTextures(1, @sTexture);
    glBindTexture(GL_TEXTURE_2D, sTexture);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 256 , 256 , 0, GL_RGB, GL_UNSIGNED_BYTE, @sFontData[0]);
  end;
  writeln(format('generating font at resolution %d,%d\n',[screenWidth,screenHeight]));
end;


end.

