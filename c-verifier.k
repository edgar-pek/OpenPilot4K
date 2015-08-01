require "../semantics/c11.k"
require "int_set.k"
require "int_list.k"
require "tree_pattern.k"
require "list_pattern.k"

module C-VERIFIER
  imports C11
  imports C-CONFIGURATION
  imports LIST-PATTERN
  imports TREE-PATTERN

  rule cast(t(.Set, T:SimpleType), tv(V:CValue, t(_:Set, T)))
    => tv(V, t(.Set, T:SimpleType))    //[lemma]

  rule (I1:Int &Int I2:Int) &Int I2:Int => I1:Int &Int I2:Int    [lemma]
  rule I:Int |Int 0 => I [lemma]
  rule 0 |Int I:Int => I [lemma]
  rule ((I1:Int >>Int I2:Int) &Int I3:Int) <<Int I2:Int
    => I1 &Int (I3 <<Int I2)
    [lemma]
  rule (I1:Int &Int I2:Int) |Int (I1:Int &Int I3:Int)
    => I1 &Int (I2 |Int I3)
    [lemma]
  rule I:Int &Int 4294967295 => I
    requires (-2147483648 <=Int I) andBool (I <=Int 2147483647)
    [lemma]
  rule (I1:Int +Int I2:Int) -Int I3:Int => I1 +Int (I2 -Int I3)   [lemma]
  rule I:Int +Int 0 => I    [lemma]
  rule 0 +Int I:Int => I    [lemma]


  /* set axioms */
  rule intersectSet(SetItem(K:K) S1:Set, S2:Set)
    => #if K in S2 #then
         SetItem(K) intersectSet(S1, S2)
       #else
         intersectSet(S1, S2)
       #fi 
  rule intersectSet(.Set, _:Set) => .Set

  rule (SetItem(K:K) S1:Set) -Set S2:Set
    => #if K in S2 #then
         S1 -Set S2
       #else
         SetItem(K) (S1 -Set S2)
       #fi 
  rule .Set -Set _:Set => .Set

  rule '_<=Set_(S1:Set,, S2:Set) => (S1 -Set S2) ==K .Set

  rule K1:K in SetItem(K2:K) S:Set => K1 ==K K2 orBool K1 in S
  rule _:K in .Set => false

  /* list axioms */
  rule K1:K in ListItem(K2:K) L:List => K1 ==K K2 orBool K1 in L
  rule _ in .List => false

  syntax KItem ::= "breakpoint"

  syntax Int ::= "ii" "(" Int "," Int "," Int "," Int ")"
  rule ii(I3:Int,I2:Int,I1:Int,I0:Int)
    => ((((((I3:Int <<Int 8) |Int I2:Int) <<Int 8) |Int I1:Int) <<Int 8) |Int I0:Int) [macro]
    
  // simulate Attitude sensors for the OpenPilot
  rule [AttitudeStateGetInit]:
       <k> prepareBuiltin(Identifier("AttitudeStateGetInit"),
            hrItem(tv(loc(ATT0, 0, SetItem(align(16)) SetItem(fromArray(0, 28))) , t(_, pointerType(t(.Set, structType(global(Identifier("AttitudeStateDataPacked"), "imu_airspeed-inlined_calls.c"))))))))
       => tv(0, t(.Set, int)) </k> <attitude-stream> ListItem (ListItem (ATIQ0:Float) ListItem (ATIQ1:Float) ListItem (ATIQ2:Float) ListItem (ATIQ3:Float) ListItem (ATIR:Float) ListItem (ATIP:Float) ListItem (ATIY:Float) ) => .List ...</attitude-stream>
       <mem>... 
       ATT0 |-> object(t(.Set, pointerType(t(.Set, structType(global(Identifier("AttitudeStateDataPacked"), "imu_airspeed-inlined_calls.c"))))), 28,
        0 |-> piece(encodedFloat(ATIQ0), 8)
        1 |-> piece(encodedFloat(ATIQ0), 8)
        2 |-> piece(encodedFloat(ATIQ0), 8)
        3 |-> piece(encodedFloat(ATIQ0), 8)

        4 |-> piece(encodedFloat(ATIQ1), 8)
        5 |-> piece(encodedFloat(ATIQ1), 8)
        6 |-> piece(encodedFloat(ATIQ1), 8)
        7 |-> piece(encodedFloat(ATIQ1), 8)

        8 |-> piece(encodedFloat(ATIQ2), 8)
        9 |-> piece(encodedFloat(ATIQ2), 8)
       10 |-> piece(encodedFloat(ATIQ2), 8)
       11 |-> piece(encodedFloat(ATIQ2), 8)

       12 |-> piece(encodedFloat(ATIQ3), 8)
       13 |-> piece(encodedFloat(ATIQ3), 8)
       14 |-> piece(encodedFloat(ATIQ3), 8)
       15 |-> piece(encodedFloat(ATIQ3), 8)

       16 |-> piece(encodedFloat(ATIR), 8)
       17 |-> piece(encodedFloat(ATIR), 8)
       18 |-> piece(encodedFloat(ATIR), 8)
       19 |-> piece(encodedFloat(ATIR), 8)

       20 |-> piece(encodedFloat(ATIP), 8)
       21 |-> piece(encodedFloat(ATIP), 8)
       22 |-> piece(encodedFloat(ATIP), 8)
       23 |-> piece(encodedFloat(ATIP), 8)

       24 |-> piece(encodedFloat(ATIY), 8)
       25 |-> piece(encodedFloat(ATIY), 8)
       26 |-> piece(encodedFloat(ATIY), 8)
       27 |-> piece(encodedFloat(ATIY), 8), .Map)
       ...</mem>
       [structural]

  rule [AttitudeStateGet]:
       <k> prepareBuiltin(Identifier("AttitudeStateGetInit"),
            hrItem(tv(loc(ATT0, 0, SetItem(align(16)) SetItem(fromArray(0, 28))) , t(_, pointerType(t(.Set, structType(global(Identifier("AttitudeStateDataPacked"), "imu_airspeed-inlined_calls.c"))))))))
       => tv(0, t(.Set, int)) </k> <attitude-stream> ListItem (ListItem (ATQ0:Float) ListItem (ATQ1:Float) ListItem (ATQ2:Float) ListItem (ATQ3:Float) ListItem (ATR:Float) ListItem (ATP:Float) ListItem (ATY:Float) ) => .List ...</attitude-stream>
       <mem>... 
       ATT0 |-> object(t(.Set, pointerType(t(.Set, structType(global(Identifier("AttitudeStateDataPacked"), "imu_airspeed-inlined_calls.c"))))), 28,
        0 |-> piece(encodedFloat(ATQ0), 8)
        1 |-> piece(encodedFloat(ATQ0), 8)
        2 |-> piece(encodedFloat(ATQ0), 8)
        3 |-> piece(encodedFloat(ATQ0), 8)

        4 |-> piece(encodedFloat(ATQ1), 8)
        5 |-> piece(encodedFloat(ATQ1), 8)
        6 |-> piece(encodedFloat(ATQ1), 8)
        7 |-> piece(encodedFloat(ATQ1), 8)

        8 |-> piece(encodedFloat(ATQ2), 8)
        9 |-> piece(encodedFloat(ATQ2), 8)
       10 |-> piece(encodedFloat(ATQ2), 8)
       11 |-> piece(encodedFloat(ATQ2), 8)

       12 |-> piece(encodedFloat(ATQ3), 8)
       13 |-> piece(encodedFloat(ATQ3), 8)
       14 |-> piece(encodedFloat(ATQ3), 8)
       15 |-> piece(encodedFloat(ATQ3), 8)

       16 |-> piece(encodedFloat(ATR), 8)
       17 |-> piece(encodedFloat(ATR), 8)
       18 |-> piece(encodedFloat(ATR), 8)
       19 |-> piece(encodedFloat(ATR), 8)

       20 |-> piece(encodedFloat(ATP), 8)
       21 |-> piece(encodedFloat(ATP), 8)
       22 |-> piece(encodedFloat(ATP), 8)
       23 |-> piece(encodedFloat(ATP), 8)

       24 |-> piece(encodedFloat(ATY), 8)
       25 |-> piece(encodedFloat(ATY), 8)
       26 |-> piece(encodedFloat(ATY), 8)
       27 |-> piece(encodedFloat(ATY), 8), .Map)
       ...</mem>
       [structural]

  rule [VelocityStateGetInit]:
       <k> prepareBuiltin(Identifier("VelocityStateGetInit"),
            hrItem(tv(loc(ATT0, 0, SetItem(align(16)) SetItem(fromArray(0, 12))) , t(_, pointerType(t(.Set, structType(global(Identifier("VelocityStateDataPacked"), "imu_airspeed-inlined_calls.c"))))))))
       => tv(0, t(.Set, int)) </k> <attitude-stream> ListItem (ListItem (ATIQ0:Float) ListItem (ATIQ1:Float) ListItem (ATIQ2:Float)) => .List ...</attitude-stream>
       <mem>... 
       ATT0 |-> object(t(.Set, pointerType(t(.Set, structType(global(Identifier("VelocityStateDataPacked"), "imu_airspeed-inlined_calls.c"))))), 12,
        0 |-> piece(encodedFloat(VSIN), 8)
        1 |-> piece(encodedFloat(VSIN), 8)
        2 |-> piece(encodedFloat(VSIN), 8)
        3 |-> piece(encodedFloat(VSIN), 8)

        4 |-> piece(encodedFloat(VSIE), 8)
        5 |-> piece(encodedFloat(VSIE), 8)
        6 |-> piece(encodedFloat(VSIE), 8)
        7 |-> piece(encodedFloat(VSIE), 8)

        8 |-> piece(encodedFloat(VSID), 8)
        9 |-> piece(encodedFloat(VSID), 8)
       10 |-> piece(encodedFloat(VSID), 8)
       11 |-> piece(encodedFloat(VSID), 8), .Map)
       ...</mem>
       [structural]

  rule [VelocityStateGetInit]:
       <k> prepareBuiltin(Identifier("VelocityStateGetInit"),
            hrItem(tv(loc(ATT0, 0, SetItem(align(16)) SetItem(fromArray(0, 12))) , t(_, pointerType(t(.Set, structType(global(Identifier("VelocityStateDataPacked"), "imu_airspeed-inlined_calls.c"))))))))
       => tv(0, t(.Set, int)) </k> <attitude-stream> ListItem (ListItem (ATIQ0:Float) ListItem (ATIQ1:Float) ListItem (ATIQ2:Float)) => .List ...</attitude-stream>
       <mem>... 
       ATT0 |-> object(t(.Set, pointerType(t(.Set, structType(global(Identifier("VelocityStateDataPacked"), "imu_airspeed-inlined_calls.c"))))), 12,
        0 |-> piece(encodedFloat(VSN), 8)
        1 |-> piece(encodedFloat(VSN), 8)
        2 |-> piece(encodedFloat(VSN), 8)
        3 |-> piece(encodedFloat(VSN), 8)

        4 |-> piece(encodedFloat(VSE), 8)
        5 |-> piece(encodedFloat(VSE), 8)
        6 |-> piece(encodedFloat(VSE), 8)
        7 |-> piece(encodedFloat(VSE), 8)

        8 |-> piece(encodedFloat(VSD), 8)
        9 |-> piece(encodedFloat(VSD), 8)
       10 |-> piece(encodedFloat(VSD), 8)
       11 |-> piece(encodedFloat(VSD), 8), .Map)
       ...</mem>
       [structural]

endmodule
