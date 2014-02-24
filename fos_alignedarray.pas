unit fos_alignedarray;

{
(§LIC)
  (c) Autor,Copyright
      Dipl.Ing.- Helmut Hartl, Dipl.Ing.- Franz Schober, Dipl.Ing.- Christian Koch
      FirmOS Business Solutions GmbH
      www.openfirmos.org
      New Style BSD Licence (OSI)

  Copyright (c) 2001-2013, FirmOS Business Solutions GmbH
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

      * Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.
      * Neither the name of the <FirmOS Business Solutions GmbH> nor the names
        of its contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
(§LIC_END)
} 

//{$i fos_bullet.inc}

{.$DEFINE CHECK_ALIGNMENT} // TODO FIX Variable Alignment

//Non Bullet workaround
{$mode objfpc}
{$MACRO ON}
{$DEFINE FOS_INLINE:=//}

interface

uses  Classes, SysUtils;
//TODO: Move to include file when aligning
//TODO : Check if Alignment Variant is necessary for basistypes / non SSE usage ?
var gFOS_Debug_Step:Boolean=false; // global falg to debug non dependend routines on request
    gFOS_Debug_SIMUSTEP:integer=0;


function FOS_SarNativeInt(Const AValue : NativeInt): NativeInt;

type

  TFOS_PtrCast =packed record
    case byte of
    0 : (ptr :pointer);
    1 : (adr :PtrUint);
    2 : (val :PtrInt);
    3 : (vskk: BYte);
  end;

   { FOS_GenericAlignedArray }
   generic FOS_GenericAlignedArray<T>=class
   private
    type
     PT=^T;
     //TComparefunction  = function(const a,b:PT):boolean;
     TComparefunctionT = function(const a,b:T):Nativeint;
   var private
     FClassmode:boolean;
     data:PByte;
     len:cardinal;
     cap:cardinal;
     type_len,pad_len:integer;
     FComparefuncT : TComparefunctionT;
     function    allocSize           (const size:integer):integer;FOS_INLINE;
     procedure   QuickSortInternal   (const CompareFunc:TComparefunctionT ; const lo,hi:integer);
     function    binary_find         (const x: T; const size: NativeInt): nativeint;
   public
     //procedure   xxx;virtual;abstract;
     constructor create              ;
     destructor  Destroy             ;override;
     procedure   Initialize          (const classref:TClass;const padsz:integer=4);FOS_INLINE;
     procedure   SetComparefunctionT (const cf:TComparefunctionT);
     procedure   SetCapacity         (const n:cardinal);
     procedure   Setlength           (const n:cardinal);
     procedure   Reserve             (const n:cardinal); // make sure that capacity is at least count
     procedure   Setlength           (const n:cardinal;const fill_val:T);  //TODO CHECK THIS IMPL
     procedure   Shrink              ;
     procedure   Clear               ; // = Finalize
     function    Length              :integer;FOS_INLINE; // = size
     function    Size                :integer;inline;
     function    Capacity            :integer;FOS_INLINE; // capacity
     procedure   Resize              (const n:integer);FOS_INLINE;
     procedure   Resize              (const n:integer;const fill:T);FOS_INLINE;
     procedure   InitValues          (const fill:T);FOS_INLINE;
     function    push_back           (const new:T):cardinal;FOS_INLINE; // = add one to array deliver index of newly added
     function    push_new            (const new:T):PT;FOS_INLINE; // = add one to array deliver pointer to it
     function    push_new_no_init    :PT;FOS_INLINE; // = add one to array deliver pointer to it
     function    pop_back            :PT;FOS_INLINE;
     procedure   remove              (const key:T);FOS_INLINE;
     procedure   quicksort           (const cp:TComparefunctionT);
     procedure   DecPointer          (var   ptr:PT;const amount:integer=1);
     procedure   IncPointer          (var   ptr:PT;const amount:integer=1);
     function    IncPointerf         (const ptr:PT;const amount:integer=1):PT;
     function    DecPointerf         (const ptr:PT;const amount:integer=1):PT;
     function    findLinearSearch    (const key:T):integer;
     function    findSorted          (const key:T):integer;
     procedure   swap                (const index0,index1:integer);FOS_INLINE;  // Todo Test SWAP
     procedure   SetIdx              (const index:cardinal;const val:T);inline;
     procedure   GetIdx              (const index:cardinal;var val:T);inline;
     function    GetIdx              (const index:cardinal):T;inline;
     function    GetIdxP             (const index:cardinal):PT;inline;
     property    A                   [const index:cardinal]:PT read GetIdxP;default;
     property    Idx                 [const index:cardinal]:T read GetIdx write SetIdx;
   end;

   TFOS_AlignedCardinals    =specialize FOS_GenericAlignedArray<Cardinal>;
   TFOS_AlignedWords        =specialize FOS_GenericAlignedArray<Word>;
   TFOS_AlignedIntegers     =specialize FOS_GenericAlignedArray<Integer>;


   { btPoolAllocator }
   btPoolAllocator=class
     function    getFreeCount:integer;
     function    CreatePoolInstance  (const typ:TClass):TObject;
     function    FreePoolInstance    (const obj:TObject):boolean;
     function    ValidPointer        (const obj:TObject):boolean;
     constructor Create              (const size:integer);
   end;

   { btStackAllocator }
   btStackAllocator=class
   public
     constructor Create           (const size:integer);
   end;

   ///very basic hashable string implementation, compatible with btHashMap

  { btHashString }

  btHashString = object
    m_string : string; //FOSHH: think about this -> const char* m_string;
    m_hash   : cardinal;
    function   getHash:cardinal;FOS_INLINE;
    procedure  init(const name:string);FOS_INLINE;
    function   portableStringCompare(const src,dst :string):integer;
    function   equals(const other:btHashString):boolean;
   end;

const BT_HASH_NULL:integer=integer($ffffffff);
type

  { btHashInt }

  btHashInt = object
    m_uid:integer;
  public
    procedure Init        (const uid:integer);
    function  getUid1     :integer;
    procedure setUid1     (const uid:integer);
    function  equals      (const other:btHashInt):boolean;
    ////to our success
    function              gethash:cardinal;FOS_INLINE;
  end;

  //class btHashPtr
  //{
  //      union
  //      {
  //              const void*     m_pointer;
  //              int     m_hashValues[2];
  //      };
  //public:
  //      btHashPtr(const void* ptr)
  //              :m_pointer(ptr)
  //      {
  //      }
  //      const void*     getPointer() const
  //      {
  //              return m_pointer;
  //      }
  //      bool equals(const btHashPtr& other) const
  //      {
  //              return getPointer() == other.getPointer();
  //      }
  //      //to our success
  //      SIMD_FORCE_INLINE       unsigned int getHash()const
  //      {
  //              const bool VOID_IS_8 = ((sizeof(void*)==8));
  //
  //              int key = VOID_IS_8? m_hashValues[0]+m_hashValues[1] : m_hashValues[0];
  //              // Thomas Wang's hash
  //              key += ~(key << 15);    key ^=  (key >> 10);    key +=  (key << 3);     key ^=  (key >> 6);     key += ~(key << 11);    key ^=  (key >> 16);
  //              return key;
  //      }
  //};
  //
  //
  //template <class Value>
  //class btHashKeyPtr
  //{
  //        int     m_uid;
  //public:
  //        btHashKeyPtr(int uid)    :m_uid(uid)
  //        {
  //        }
  //        int     getUid1() const
  //        {
  //                return m_uid;
  //        }
  //        bool equals(const btHashKeyPtr<Value>& other) const
  //        {
  //                return getUid1() == other.getUid1();
  //        }
  //        //to our success
  //        SIMD_FORCE_INLINE       unsigned int getHash()const
  //        {
  //                int key = m_uid;
  //                // Thomas Wang's hash
  //                key += ~(key << 15);  key ^=  (key >> 10);    key +=  (key << 3);     key ^=  (key >> 6);     key += ~(key << 11);    key ^=  (key >> 16);
  //                return key;
  //        }
  //};
  //
  //template <class Value>
  //class btHashKey
  //{
  //      int     m_uid;
  //public:
  //      btHashKey(int uid)      :m_uid(uid)
  //      {
  //      }
  //      int     getUid1() const
  //      {
  //              return m_uid;
  //      }
  //      bool equals(const btHashKey<Value>& other) const
  //      {
  //              return getUid1() == other.getUid1();
  //      }
  //      //to our success
  //      SIMD_FORCE_INLINE       unsigned int getHash()const
  //      {
  //              int key = m_uid;
  //              // Thomas Wang's hash
  //              key += ~(key << 15);    key ^=  (key >> 10);    key +=  (key << 3);     key ^=  (key >> 6);     key += ~(key << 11);    key ^=  (key >> 16);
  //              return key;
  //      }
  //};
  //
  ///The btHashMap template class implements a generic and lightweight hashmap.
  ///A basic sample of how to use btHashMap is located in Demos\BasicDemo\main.cpp



  { FOS_GenericHashMap }
  //TFOS_HASHValueArray = specialize FOS_GenericAlignedArray16<Value>;
  //TFOS_HASHKeyArray   = specialize FOS_GenericAlignedArray16<Key>;


  // THE KEY TYPE MUST PROVIDE AN EQUALS METHOD !!
  generic FOS_GenericHashMap<TKey,TValue,TKeyArr,TValArr>=object
  protected
    type
     PKey                = ^TKey;
     PValue              = ^TValue;
  public
     m_hashTable  : TFOS_AlignedIntegers;
     m_next       : TFOS_AlignedIntegers;
     m_valueArray : TValArr;
     m_keyArray   : TKeyArr;
     function  GetIdx(const key: TKey): PValue;
     //procedure SetIdx(const key: TKey; const AValue: PValue);
  public
    procedure genAssert  (const x:boolean);
    procedure growTables ;
    procedure insert     (const key:TKey;const value:TValue);
    procedure remove     (const key:TKey);
    function  size       :integer;
    function  getAtIndex (const index:integer):PValue;
    property  Items      [const key:TKey]:PValue  read GetIdx;  default;
    function  find       (const key:TKey):PValue;
    function  findIndex  (const key:TKey):integer;
    procedure clear      ;
  end;

  btHashIntArray     = specialize FOS_GenericAlignedArray<btHashInt>;
  btHashStringArray  = specialize FOS_GenericAlignedArray<btHashString>;

  //  bValAlArr  =specialize FOS_GenericAlignedArray16<bValt>;

   operator = (const a,b:btHashInt) res:boolean;FOS_INLINE;
   operator = (const a,b:btHashString) res:boolean;FOS_INLINE;

   function FOS_PtrCast(const p:pointer):ptruint;
   procedure myAssert(a:boolean);
   function _DumpCurrentBacktrace: string;

//   procedure Trigger;

implementation

{$HINTS OFF}

procedure myAssert(a: boolean);
begin
   Assert(a,'ATTENTION');
   if not a then begin
    writeln('ASSERTION failed ');
    writeln(_DumpCurrentBacktrace);
    abort;
   end;
end;

function _DumpCurrentBacktrace: string;
  var
    bp: Pointer;
    addr: Pointer;
    oldbp: Pointer;
    CurAddress: Shortstring;
begin
    Result:='';
    bp:=get_caller_frame(get_frame);
    bp:=get_caller_frame(bp);
    while bp<>nil do begin
      addr:=get_caller_addr(bp);
      CurAddress:=BackTraceStrFunc(Addr);;
      Result:=Result+CurAddress+LineEnding;
      oldbp:=bp;
      bp:=get_caller_frame(bp);
      if (bp<=oldbp) or (bp>(StackBottom + StackLength)) then
        bp:=nil;
    end;
end;

function FOS_SarNativeInt(const AValue: NativeInt): NativeInt;
begin
  {$ifdef CPU64}
  result := SarInt64(AValue);
  {$ELSE}
  result := SarLongint(AValue);
  {$ENDIF}
end;

operator=(const a, b: btHashInt) res:boolean;
begin
 res := a.equals(b);
end;

operator=(const a, b: btHashString) res:boolean;
begin
 res := a.equals(b);
end;

function FOS_PtrCast(const p: pointer): ptruint;
var pc:TFOS_PtrCast;
begin
  pc.ptr:=p;
  result:=pc.adr;
end;

{$HINTS ON}
function FOS_GenericAlignedArray.allocSize(const size: integer): integer;
begin
  if size=0 then exit(1);
  exit(size*2);
end;

procedure FOS_GenericAlignedArray.QuickSortInternal(const CompareFunc:TComparefunctionT ; const lo, hi: integer);
var i,j,p : integer;
    x     : T;
begin
  i := lo;
  j := hi;
  p := (lo+hi) div 2;
  x := self[p]^;
  repeat
    while CompareFunc(self[i]^,x)<0 do begin
      inc(i);
    end;
    while CompareFunc(x,self[j]^)<0 do begin
      dec(j);
    end;
    if i<=j then begin
      swap(i,j);
      inc(i);
      dec(j);
    end;
  until (i>j);
  if lo<j then begin
    quickSortInternal(CompareFunc, lo, j);
  end;
  if i<hi then begin
    quickSortInternal(CompareFunc, i, hi);
  end;
end;

function FOS_GenericAlignedArray.binary_find(const x: T; const size: NativeInt): nativeint;
var l,c,r,i,val  : NativeInt;
    lx,cx,rx     : T;
begin
  l  := 0;
  r  := size;
  c  := FOS_SarNativeInt(r);
  lx := GetIdx(l);
  if FComparefuncT(x, lx) < 0 then begin
    //writeln('CASE C');
    exit(size+1);
  end else
  if FComparefuncT(x, lx) = 0 then begin
    i := 1;
    while (FComparefuncT(x, GetIdx(i)) = 0) do inc(i);
    //writeln('CASE D');
    exit(i);
  end;
  rx := GetIdx(r);
  cx := GetIdx(c);
  while true do begin
    val := FComparefuncT(x, cx);
    if val < 0 then begin
      if (c - l) <= 1 then begin
        //writeln('CASE A');
        //exit(c);
        exit(size+1);
      end;
      r  := c;
      rx := cx;
    end else
    if val > 0 then begin
      if (r - c) <= 1 then begin
        //exit(c + 1);
        exit(c + 2);
      end;
      l  := c;
      lx  := cx;
    end else begin
      repeat
        inc(c);
        cx := GetIdx(c);
      until FComparefuncT(x, cx) <> 0;
      exit(c);
    end;
    c  := l + FOS_SarNativeInt(r - l);
    cx := GetIdx(c);
  end;
end;

constructor FOS_GenericAlignedArray.create;
begin
  type_len      := sizeof(T);
  pad_len       := (type_len-1) + (4 - (((type_len-1)) mod 4));
  FComparefuncT := nil;
  FClassmode    := false;
end;

destructor FOS_GenericAlignedArray.Destroy;
begin
  clear;
  Inherited;
end;

//function FOS_GenericAlignedArray16.PadTo(const plen: cardinal; const pad: cardinal): cardinal;
//var npad_len:cardinal;
//begin
//  //.
//  FOS_DEBUGINIT_GUARD;
//  //.
// npad_len:=plen mod pad;
// if npad_len>0 then begin
//  result:=plen+(pad-npad_len);
// end else begin
//  result:=plen;
// end;
//end;

procedure FOS_GenericAlignedArray.Initialize(const classref:TClass;const padsz: integer);
begin
  if not assigned(classref) then begin
    type_len      := sizeof(T);
    pad_len       := (sizeof(T)-1) + (padsz - (((sizeof(T)-1)) mod padsz));
    FComparefuncT := nil;
    FClassmode    := false;
  end else begin
    type_len      := classref.InstanceSize;
//    writeln('Generic Array ',type_len);
    pad_len       := (type_len-1) + (padsz - ((type_len-1)) mod padsz);
    FComparefuncT := nil;
    FClassmode    := true;
  end;
end;

procedure FOS_GenericAlignedArray.SetComparefunctionT(const cf: TComparefunctionT);
begin
  FComparefuncT:=cf;
end;

//function FOS_GenericAlignedArray16._IntCompare(const a, b: T): boolean;
//begin
//  result := a=b;
//end;

procedure FOS_GenericAlignedArray.SetCapacity(const n: cardinal);
begin
  if data=nil then begin
    data:=AllocMem (integer(n)*pad_len);
    {$IFDEF CHECK_ALIGNMENT}
     {$HINTS OFF}
      if (PtrUInt(data) mod 16) <> 0 then abort;
     {$HINTS ON}
    {$ENDIF}
     cap:=n;
  end else begin
    if n>cap then begin
      data:=ReAllocMem(data,integer(n)*pad_len);
      cap:=n;
    end;
  end;
end;

{ FOS_GenericAlignedArray16 }
procedure FOS_GenericAlignedArray.Setlength(const n: cardinal);
begin
  SetCapacity(n);
  len:=n;
end;

procedure FOS_GenericAlignedArray.Reserve(const n: cardinal);
begin
  SetCapacity(n);
end;

procedure FOS_GenericAlignedArray.Setlength(const n: cardinal;const fill_val: T);
var olen:cardinal;
       i:integer;
begin
  olen:=len;
  Setlength(n);
  if n>olen then begin
    for i:=olen to n-1 do begin
      A[i]^:=fill_val;
    end;
  end;
end;

procedure FOS_GenericAlignedArray.Shrink;
begin
  if cap>len then begin
    data:=ReAllocMem(data,integer(len)*pad_len);
    cap:=len;
  end;
end;


procedure FOS_GenericAlignedArray.Clear;
begin
  Freemem(data);
  data:=nil;
end;

function FOS_GenericAlignedArray.Length: integer;
begin
  result:=len;
end;

function FOS_GenericAlignedArray.Size: integer;
begin
  result:=len;
end;

function FOS_GenericAlignedArray.Capacity: integer;
begin
  result:=cap;
end;

procedure FOS_GenericAlignedArray.Resize(const n: integer);
begin
  Setlength(n);
end;

procedure FOS_GenericAlignedArray.Resize(const n: integer; const fill: T);
var i,oldlen:integer;
begin
  oldlen := len;
  Resize(n);
  for i:=oldlen to cap-1 do begin
    A[i]^ := fill;
  end;
//  writeln('VERRY Spezial REsize called ',oldlen,' to ',cap);
//  abort;
end;

procedure FOS_GenericAlignedArray.InitValues(const fill: T);
var i:integer;
begin
  for i:=0 to len-1 do begin
    A[i]^ := fill;
  end;
end;

function FOS_GenericAlignedArray.push_back(const new: T): cardinal;
begin
  if len=cap then reserve(allocSize(len));
  result:=len;
  A[result]^:=new;
  len:=len+1;
end;

function FOS_GenericAlignedArray.push_new(const new: T): PT;
begin
  result:=push_new_no_init;
  result^ := new;
end;

function FOS_GenericAlignedArray.push_new_no_init: PT;
begin
  if len=cap then reserve(allocSize(len));
  result:=A[len];
  len:=len+1;
end;

function FOS_GenericAlignedArray.pop_back:PT;
begin
  if len>0 then begin
    len:=len-1;
  end else begin
    raise Exception.Create('invalid pop_back len='+inttostr(len));
  end;
  result:=self.A[len]; // To allow free (non c++ generic destrucor equivalent)
end;

procedure FOS_GenericAlignedArray.remove(const key: T);
var findIndex:Integer;
begin
  findIndex := findLinearSearch(key);
  if findIndex<size then begin
    swap( findIndex,size-1);
    pop_back;
  end;
end;


{$HINTS OFF}
procedure FOS_GenericAlignedArray.quicksort(const cp: TComparefunctionT);
begin
  if len > 1 then begin
    quickSortInternal( cp,0,len-1);
  end;
end;

procedure FOS_GenericAlignedArray.DecPointer(var ptr: PT; const amount: integer);
begin
  ptr:=PT(PByte(ptr)-(amount*pad_len));
end;

procedure FOS_GenericAlignedArray.IncPointer(var ptr: PT; const amount: integer);
begin
  ptr:=PT(PByte(ptr)+(amount*pad_len));
end;

function FOS_GenericAlignedArray.IncPointerf(const ptr: PT; const amount: integer): PT;
begin
  result:=PT(PByte(ptr)+(amount*pad_len));
end;

function FOS_GenericAlignedArray.DecPointerf(const ptr: PT; const amount: integer): PT;
begin
  result:=PT(PByte(ptr)-(amount*pad_len));
end;

function FOS_GenericAlignedArray.findLinearSearch(const key: T): integer;
var i,index:integer;
begin
  index:=size;
  for i:=0 to size-1 do  begin
    if FComparefuncT(self.A[i]^,key)=0 then begin
     index := i;
     break;
    end;
  end;
  result := index;
end;

function FOS_GenericAlignedArray.findSorted(const key: T): integer;
begin
  result   := binary_find(key,Length)-1;
end;

{$HINTS ON}

{$HINTS OFF}
procedure FOS_GenericAlignedArray.swap(const index0, index1: integer);
var temp :T;
begin
//  writeln('swap ix0 = ',index0,' ix1=',index1);
  temp := A[index0]^;
  A[index0]^ := A[index1]^;
  A[index1]^ := temp;
  {
#ifdef BT_USE_MEMCPY
  	char	temp[sizeof(T)];
  	memcpy(temp,&m_data[index0],sizeof(T));
  	memcpy(&m_data[index0],&m_data[index1],sizeof(T));
  	memcpy(&m_data[index1],temp,sizeof(T));
#else
  	T temp = m_data[index0];
  	m_data[index0] = m_data[index1];
  	m_data[index1] = temp;
#endif //BT_USE_PLACEMENT_NEW

  }
end;
{$HINTS ON}

procedure FOS_GenericAlignedArray.GetIdx(const index: cardinal; var val: T);
begin
  val:=PT(@data[integer(index)*pad_len])^;
  {$IFDEF CHECK_ALIGNMENT}
   {$HINTS OFF}
    if (PtrUInt(@data[index*pad_len]) mod 16) <> 0 then abort;
   {$HINTS ON}
  {$ENDIF}
end;

function FOS_GenericAlignedArray.GetIdx(const index: cardinal):T;
begin
  result:=PT(@data[integer(index)*pad_len])^;
  {$IFDEF CHECK_ALIGNMENT}
  {$HINTS OFF}
    if (PtrUInt(@data[index*pad_len]) mod 16) <> 0 then abort;
  {$HINTS ON}
  {$ENDIF}
end;

function FOS_GenericAlignedArray.GetIdxP(const index: cardinal): PT;
begin
  result:=PT(@data[integer(index)*pad_len]);
  {$IFDEF CHECK_ALIGNMENT}
    if (PtrUInt(@data[index*pad_len]) mod 16) <> 0 then abort;
  {$ENDIF}
end;

procedure FOS_GenericAlignedArray.SetIdx(const index: cardinal; const val: T);
begin
  PT(@data[integer(index)*pad_len])^:=val;
  {$IFDEF CHECK_ALIGNMENT}
  {$HINTS OFF}
    if (PtrUInt(@data[index*pad_len]) mod 16) <> 0 then abort;
  {$HINTS ON}
  {$ENDIF}
end;

{ btPoolAllocator }

function btPoolAllocator.getFreeCount: integer;
begin
  result:=0;
end;

function btPoolAllocator.CreatePoolInstance(const typ: TClass): TObject;
begin
  result:=typ.Create;
end;

function btPoolAllocator.FreePoolInstance(const obj: TObject): boolean;
begin
  obj.Free;
  result:=true;
end;

function btPoolAllocator.ValidPointer(const obj: TObject): boolean;
begin
  result := true;
end;

{$HINTS OFF}
constructor btPoolAllocator.Create(const size: integer);
begin
  inherited CReate;
end;
{$HINTS ON}

{ btStackAllocator }
{$HINTS OFF}
constructor btStackAllocator.Create(const size: integer);
begin

end;
{$HINTS ON}

{ btHashString }

function btHashString.getHash: cardinal;
begin
  result := m_hash;
end;

///* magic numbers from http://www.isthe.com/chongo/tech/comp/fnv/ */
const InitialFNV:cardinal = 2166136261;
      FNVMultiple:cardinal = 16777619;

procedure btHashString.init(const name: string);
var hash:cardinal;
  i: Integer;
begin
//  /* Fowler / Noll / Vo (FNV) Hash */
  m_string := name;
  hash := InitialFNV;
  for i := 1 to length(m_string) do begin
    hash := hash XOR (byte(m_string[i]));    //   /* xor  the low 8 bits */
    hash := hash * FNVMultiple;       //  /* multiply by the magic number */
  end;
  m_hash := hash;
end;

function btHashString.portableStringCompare(const src, dst: string): integer;
begin
  result:=CompareStr(src,dst);
  //int ret = 0 ;
  //while( ! (ret = *(unsigned char *)src - *(unsigned char *)dst) && *dst)
  //                ++src, ++dst;
  //if ( ret < 0 )
  //                ret = -1 ;
  //else if ( ret > 0 )
  //                ret = 1 ;
  //return( ret );
end;

function btHashString.equals(const other: btHashString): boolean;
begin
  result := m_string=other.m_string;
  {
          return (m_string == other.m_string) ||
                  (0==portableStringCompare(m_string,other.m_string));
  }
end;

{ btHashInt }

procedure btHashInt.Init(const uid: integer);
begin
  m_uid := uid;
end;

function btHashInt.getUid1: integer;
begin
  result:=m_uid;
end;

procedure btHashInt.setUid1(const uid: integer);
begin
  m_uid:=uid;
end;

function btHashInt.equals(const other: btHashInt): boolean;
begin
  Result := getUid1 = other.getUid1;
end;

function btHashInt.gethash: cardinal;
var key:integer;
begin
  //TEST compare with one working from btHashedoverlappingPaircache
  key := m_uid;
  // Thomas Wang's hash
  key := m_uid;
  key := key + not(key SHL 15);
  key := key XOR  sarlongint(key,10);
  key := key +    (key SHL  3);
  key := key XOR  SarLongint(key,6);
  key := key + not(key SHL 11);
  key := key XOR  sarlongint(key,16);
  result := cardinal(key);
end;

{ FOS_GenericHashMap }

procedure FOS_GenericHashMap.growTables;
var hashValue,newCapacity,curHashtableSize,i:integer;
begin
  newCapacity := m_valueArray.Capacity;
  if m_hashTable.length < newCapacity then begin
    //grow hashtable and next table
    curHashtableSize := m_hashTable.length;
    m_hashTable.setcapacity(newCapacity);
    m_next.setcapacity(newCapacity);
    for i := 0 to newCapacity-1 do begin
      m_hashTable.A[i]^ := BT_HASH_NULL;
    end;
    for i := 0 to newCapacity-1 do begin
      m_next.A[i]^      := BT_HASH_NULL;
    end;
    for i := 0 to  curHashtableSize-1 do begin
      //const Value& value = m_valueArray[i];
      //const Key& key = m_keyArray[i];
      hashValue := m_keyArray.A[i]^.getHash AND (m_valueArray.capacity-1);      // New hash value with new mask
      m_next.A[i]^ := m_hashTable.A[hashValue]^;
      m_hashTable.A[hashValue]^ := i;
    end;
  end;
end;

procedure FOS_GenericHashMap.insert(const key: TKey;const value:TValue);
var hash:integer;
    index:integer;
    count,oldCapacity,newCapacity:integer;
begin
  hash := key.getHash and (m_valueArray.capacity-1);
  //replace value if the key is already there
  index := findIndex(key);
  if index <> BT_HASH_NULL then begin
    m_valueArray.A[index]^:=value;
    exit;
  end;

  count       := m_valueArray.length;
  oldCapacity := m_valueArray.capacity;

  m_valueArray.push_back(value);
  m_keyArray.push_back(key);

  newCapacity := m_valueArray.capacity;
  if oldCapacity < newCapacity then begin
    growTables; // FOSHH: was growTables(key);
    //hash with new capacity
    hash := key.getHash AND (m_valueArray.capacity-1);
  end;
  m_next.A[count]^     := m_hashTable.A[hash]^;
  m_hashTable.A[hash]^ := count;
end;

procedure FOS_GenericHashMap.remove(const key: TKey);
var hash:integer;
    index,pairindex,lastPairIndex,lastHash:integer;
    previous:integer;
begin
  hash      := key.getHash and (m_valueArray.capacity-1);
  pairIndex := findIndex(key);
  if pairIndex = BT_HASH_NULL then begin
    exit;
  end;
  // Remove the pair from the hash table.
  index := m_hashTable.A[hash]^;
  genAssert(index <> BT_HASH_NULL);

  previous := BT_HASH_NULL;
  while index <> pairIndex do begin
    previous := index;
    index    := m_next.A[index]^;
  end;

  if previous <> BT_HASH_NULL then begin
    genAssert(m_next.A[previous]^ = pairIndex);
    m_next.A[previous]^  := m_next.A[pairIndex]^;
  end else begin
    m_hashTable.A[hash]^ := m_next.A[pairIndex]^;
  end;

  // We now move the last pair into spot of the
  // pair being removed. We need to fix the hash
  // table indices to support the move.

  lastPairIndex := m_valueArray.size - 1;

  // If the removed pair is the last pair, we are done.
  if (lastPairIndex = pairIndex) then begin
    m_valueArray.pop_back;
    m_keyArray.pop_back;
    exit;
  end;

  // Remove the last pair from the hash table.
  lastHash := m_keyArray.A[lastPairIndex]^.getHash and (m_valueArray.capacity-1);

  index := m_hashTable.A[lastHash]^;
  genAssert(index <> BT_HASH_NULL);

  previous := BT_HASH_NULL;
  while (index <> lastPairIndex) do begin
    previous := index;
    index    := m_next.A[index]^;
  end;
  if (previous <> BT_HASH_NULL) then begin
    genAssert(m_next.A[previous]^ = lastPairIndex);
    m_next.A[previous]^ := m_next.A[lastPairIndex]^;
  end else begin
    m_hashTable.A[lastHash]^ := m_next.A[lastPairIndex]^;
  end;

  // Copy the last pair into the remove pair's spot.
  m_valueArray.A[pairIndex]^ := m_valueArray.A[lastPairIndex]^;
  m_keyArray.A[pairIndex]^   := m_keyArray.A[lastPairIndex]^;

  // Insert the last pair into the hash table
  m_next.A[pairIndex]^     := m_hashTable.A[lastHash]^;
  m_hashTable.A[lastHash]^ := pairIndex;

  m_valueArray.pop_back;
  m_keyArray.pop_back;
end;

function FOS_GenericHashMap.size: integer;
begin
  result := m_valueArray.length;
end;

function FOS_GenericHashMap.getAtIndex(const index: integer): PValue;
begin
  genAssert(index < m_valueArray.size);
  result := m_valueArray.A[index];
end;

function FOS_GenericHashMap.find(const keY: TKey): PValue;
var index:integer;
begin
  index := findIndex(key);
  if (index = BT_HASH_NULL) then begin
    exit(nil);
  end;
  result := m_valueArray.A[index];
end;

function FOS_GenericHashMap.GetIdx(const key: TKey): PValue;
begin
  result := find(key);
end;

{$HINTS OFF}
procedure FOS_GenericHashMap.genAssert(const x: boolean);
begin
 if x then abort;
end;
{$HINTS ON}

{$HINTS OFF}
function FOS_GenericHashMap.findIndex(const key: TKey): integer;
var hash:cardinal;
    index:integer;
begin
  hash := key.getHash AND (m_valueArray.capacity-1);
  if hash >= cardinal(m_hashTable.size) then begin
   Result := BT_HASH_NULL;
  end;
  index := m_hashTable.A[hash]^;
  while (index <> BT_HASH_NULL) AND (key.equals(m_keyArray.A[index]^) = false) do begin // THE KEY TYPE MUST PROVIDE AN EQUALS METHOD !!
    index := m_next.A[index]^;
  end;
  result := index;
end;
{$HINTS ON}

procedure FOS_GenericHashMap.clear;
begin
  m_hashTable.clear;
  m_next.clear;
  m_valueArray.clear;
  m_keyArray.clear;
end;

end.

