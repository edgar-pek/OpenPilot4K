require "c-verifier.k"

module OPENPILOT-SPEC
imports C-VERIFIER

syntax Float ::= "ita_bwf" "(" Float ")"    [function]
rule ita_bwf(FF:Float)
  => 1.0 /Float tanFloat(M_PI_F *Float ?FF)

syntax Float ::= "b0_bwf" "(" Float ")"    [function]
rule b0_bwf(FF:Float)
  => 1.0 /Float (1.0 +Float M_SQRT2_F *Float ita_bwf(FF) +Float ita_bwf(FF) *Float ita_bwf(FF))

syntax Float ::= "a1_bwf" "(" Float ")"    [function]
rule a1_bwf(FF:Float)
  => 2.0 *Float b0_bwf(FF) *Float (ita_bwf(FF) *Float ita_bwf(FF) -Float 1.0) 

syntax Float ::= "a2_bwf" "(" Float ")"    [function]
rule a2_bwf(FF:Float)
  => -Float(b0_bwf(FF)) *Float (1.0 -Float M_SQRT2_F *Float ita_bwf(FF) +Float ita_bwf(FF) *Float ita_bwf(FF))

syntax Bool ::= "eq_ite" "(" Bool "," Float "," Float "," Float ")"
rule eq_ite(COND:Bool, LHS1:Float, RHS1T:Float, RHS1F:Float)
  =>         (        COND ImpliesBool LHS1 ==K RHS1T)
     andBool (notBool COND ImpliesBool LHS1 ==K RHS1F)

     
// a11 = (2.0 +Float A1)
// a12 = (1.0 +Float A2)
// a21 = (2.0 +Float A1 *Float A1 +Float A2)
// a22 = (1.0 +Float A1 *Float A2)
// rhs1 = (X0 /Float B0 -Float X0)
// rhs2 = (X0 /Float B0 -Float X0 +Float A1 +Float X0) 
// -- void InitButterWorthDF2Values(const float x0, const struct ButterWorthDF2Filter *filterPtr, float *wn1Ptr, float *wn2Ptr)
// --> wn1Ptr
syntax Float ::= "wn1_bwv" "(" Float "," Float "," Float "," Float ")"    [function]
rule wn1_bwv(X0:Float, B0:Float, A1:Float, A2:Float)
  => ((1.0 +Float A1 *Float A2) *Float (X0 /Float B0 -Float X0) -Float (1.0 +Float A2) *Float (X0 /Float B0 -Float X0 +Float A1 +Float X0)) 
      /Float ((2.0 +Float A1) *Float (1.0 +Float A1 *Float A2) -Float (1.0 +Float A2) *Float (2.0 +Float A1 *Float A1 +Float A2))
// void InitButterWorthDF2Values(const float x0, const struct ButterWorthDF2Filter *filterPtr, float *wn1Ptr, float *wn2Ptr)
// --> wn2Ptr
syntax Float ::= "wn2_bwv" "(" Float "," Float "," Float "," Float ")"    [function]
rule wn2_bwv(X0:Float, B0:Float, A1:Float, A2:Float)
  => (-Float (2.0 +Float A1 *Float A1 +Float A2) *Float (X0 /Float B0 -Float X0) -Float (2.0 +Float A1) *Float (X0 /Float B0 -Float X0 +Float A1 +Float X0)) 
      /Float ((2.0 +Float A1) *Float (1.0 +Float A1 *Float A2) -Float (1.0 +Float A2) *Float (2.0 +Float A1 *Float A1 +Float A2))

syntax FloatTuple ::= "FloatTuple" "(" Float ","  Float ")"
syntax Float ::= fst2(FloatTuple) [function]
rule fst2(FloatTuple(F:Float, _)) => F
syntax Float ::= snd2(FloatTuple) [function]
rule snd2(FloatTuple(_, F:Float)) => F

syntax Float ::= "sq_float" "(" Float ")" [function]
rule sq_float(X:Float) => X *Float X

syntax Float ::= "aux_y" "(" Float "," Float "," Float "," Float ")" [function]
rule aux_y(Q0:Float, Q1:Float, Q1:Float, Q3:Float)
  => atan2Float((2.0 *Float (Q1 *Float Q2 +Float Q0 *Float Q3)), (sq_float(Q0) + sq_float(Q1) - sq_float(Q2) - sq_float(Q3)))

syntax Int ::= "aux_cycles" "(" Float ")" [function]
rule aux_cycles(OldY:Float) 
  => Float2Int(OldY /Float M_2PI_F)

syntax Int ::= "aux_mod" "(" Float "," Float ")" 
rule aux_mod(YU:Float, Cycles:Float)  [function]
  => Float2Int((YU -Float (YOld -Float Cycles *Float M_2PI_F)) /Float (M_2PI_F * 0.8))

syntax FloatTuple ::= "spec4Quaternion2PY" "(" Float "," Float "," Float "," Float "," Float ")" [function]
rule spec4Quaternion2PY(Q0:Float, Q1:Float, Q1:Float, Q3:Float, OldY:Float) 
  => FloatTuple(asinFloat(2.0 *Float (Q1 *Float Q3 *Float -Float Q0 *Float Q2)), 
                aux_y(Q0, Q1, Q2, Q3) +Float M_2PI_F *Float (Int2Float(aux_cycles(OldY)) -Float Int2Float(aux_mod(aux_y(Q0,Q1,Q2,Q3), Int2Float(aux_cycles(OldY))))))

syntax FloatTriple ::= "FloatTriple" "(" Float ","  Float "," Float ")"
syntax Float ::= fst3(FloatTriple) [function]
rule fst3(FloatTuple(F:Float, _, _)) => F
syntax Float ::= snd3(FloatTriple) [function]
rule snd3(FloatTuple(_, F:Float, _)) => F
syntax Float ::= trd3(FloatTriple) [function]
rule trd3(FloatTuple(_, _, F:Float)) => F

// wn = (XN +Float A1 *Float WN1 +Float A2 *Float WN2)
syntax FloatTriple ::= "spec4FilterButterWorth" "(" Float "," Float "," Float "," Float "," Float "," Float ")"  [function]
rule spec4FilterButterWorth(XN:Float, B0:Float, A1:Float, A2:Float, WN1:Float, WN2:Float) 
  => FloatTriple(B0 *Float ((XN +Float A1 *Float WN1 +Float A2 *Float WN2) +Float 2.0 +Float WN1 +Float WN2),
                 XN +Float A1 *Float WN1 +Float A2 *Float WN2,
                 WN1)

syntax FloatTriple ::= "spec4PY2xB" "(" Float "," Float ")" [function]
rule spec4PY2xB(P:Float, Y:Float) 
  => FloatTriple(cosFloat(P) *Float cosFloat(Y), 
                 cosFloat(P) *Float sinFloat(Y),
                 -Float(sinFloat(P)))

syntax FloatTriple ::= "spec4PY2DeltaxB" "(" Float "," Float "," FloatTriple ")"   [function]
rule spec4PY2DeltaxB(P:Float, Y:Float, XB:FloatTriple)
  => FloatTriple(fst3(XB) -Float cosFloat(P) *Float cosFloat(Y),
                 snd3(XB) -Float cosFloat(P) *Float sinFloat(Y),
                 trd3(XB) -Float -Float(sinFloat(P)))

syntax FloatTriple ::= "calc_wind" "(" Float "," Float "," Float "," FloatTriple "," Float ")"
rule calc_wind(V1Old:Float, V2Old:Float, V3Old:Float, XB:FloatTriple, Airspeed:Float) 
  => FloatTriple(V1Old -Float fst3(XB) *Float Airspeed,
                 V2Old -Float snd3(XB) *Float Airspeed,
                 V3Old -Float trd3(XB) *Float Airspeed)

syntax FloatTriple ::= "calc_airspeed" "(" Float "," Float "," Float "," Float "," Float "," Float ")"
rule calc_airspeed(V1Old:Float, V2Old:Float, V3Old:Float, Vw1:Float, Vw2:Float, Vw3:Float) 
  => FloatTriple(V1Old -Float Vw1, 
                 V2Old -Float Vw2,
                 V3Old -Float Vw3)

// ----------------------------------------------------------------------------------------------------------------------------------------------
rule [func-imu-airspeed-get]:
//<global>
    <mem>...  MEM:Map 
       BP0 |-> object(t(.Set, pointerType(t(.Set, structType(global(Identifier("AirspeedSensorDataPacked"), "imu_airspeed-inlined_calls.c"))))), 24,
        0 |-> piece(_ => encodedFloat(DP), 8) // float DifferentialPressure
        1 |-> piece(_ => encodedFloat(DP), 8) 
        2 |-> piece(_ => encodedFloat(DP), 8)
        3 |-> piece(_ => encodedFloat(DP), 8)

        4 |-> piece(_ => encodedFloat(TEMP), 8) // float Temperature
        5 |-> piece(_ => encodedFloat(TEMP), 8)
        6 |-> piece(_ => encodedFloat(TEMP), 8)
        7 |-> piece(_ => encodedFloat(TEMP), 8)

        8 |-> piece(_ => encodedFloat(?CALIB_AIR_SPEED), 8) // float CalibratedAirspeed;
        9 |-> piece(_ => encodedFloat(?CALIB_AIR_SPEED), 8)
       10 |-> piece(_ => encodedFloat(?CALIB_AIR_SPEED), 8)
       11 |-> piece(_ => encodedFloat(?CALIB_AIR_SPEED), 8)

       12 |-> piece(_ => encodedFloat(TA), 8) // float TrueAirspeed;
       13 |-> piece(_ => encodedFloat(TA), 8)
       14 |-> piece(_ => encodedFloat(TA), 8)
       15 |-> piece(_ => encodedFloat(TA), 8)

       16 |-> piece(_ => SV0:Int, 8)          // uint16_t SensorValue;
       17 |-> piece(_ => SV1:Int, 8)          // REMARK: assemble the bytes

       18 |-> piece(_ => SVT0:Int, 8) //uint16_t SensorValueTemperature;
       19 |-> piece(_ => SVT1:Int, 8)

       20 |-> piece(_ => 0, 8) // AirspeedSensorSensorConnectedOptions SensorConnected; 
       21 |-> piece(_ => 0, 8) 
       22 |-> piece(_ => 0, 8)
       23 |-> piece(_ => SC:Int, 8), .Map)
    BP1 |-> object(t(.Set, pointerType(t(.Set, structType(global(Identifier("AirspeedSettingsDataPacked"), "imu_airspeed-inlined_calls.c"))))), 19,
        0 |-> piece(encodedFloat(SC), 8) // float Scale;
        1 |-> piece(encodedFloat(SC), 8) 
        2 |-> piece(encodedFloat(SC), 8)
        3 |-> piece(encodedFloat(SC), 8)

        4 |-> piece(encodedFloat(LP1), 8) //float IMUBasedEstimationLowPassPeriod1;
        5 |-> piece(encodedFloat(LP1), 8)
        6 |-> piece(encodedFloat(LP1), 8)
        7 |-> piece(encodedFloat(LP1), 8)

        8 |-> piece(encodedFloat(LP2), 8) // float IMUBasedEstimationLowPassPeriod2;
        9 |-> piece(encodedFloat(LP2), 8)
       10 |-> piece(encodedFloat(LP2), 8)
       11 |-> piece(encodedFloat(LP2), 8)

       12 |-> piece(ZP0:Int, 8)          // uint16_t ZeroPoint;
       13 |-> piece(ZP1:Int, 8)

       14 |-> piece(SP0:Int, 8)         // uint8_t SamplePeriod;

       15 |-> piece(0, 8) // AirspeedSensorSensorConnectedOptions SensorConnected; 
       16 |-> piece(0, 8) 
       17 |-> piece(0, 8)
       18 |-> piece(SC:Int, 8), .Map)
    GL0 |-> object(t(.Set, pointerType(t(.Set, structType(global(Identifier("IMUGlobals"), "imu_airspeed-inlined_calls.c"))))), 128,
        0 |-> piece(FEL0 => encodedFloat(?FEl0), 8) // struct ButterWorthDF2Filter filter; (3 floats)
        1 |-> piece(FEL0 => encodedFloat(?FEl0), 8) 
        2 |-> piece(FEL0 => encodedFloat(?FEl0), 8)
        3 |-> piece(FEL0 => encodedFloat(?FEl0), 8)
        // ------------------------------------
        4 |-> piece(FEL1 => encodedFloat(?FEl1), 8) 
        5 |-> piece(FEL1 => encodedFloat(?FEl1), 8)
        6 |-> piece(FEL1 => encodedFloat(?FEl1), 8)
        7 |-> piece(FEL1 => encodedFloat(?FEl1), 8)
        // -------------------------------------
        8 |-> piece(FEL2 => encodedFloat(?FEl2), 8) 
        9 |-> piece(FEL2 => encodedFloat(?FEl2), 8)
       10 |-> piece(FEL2 => encodedFloat(?FEl2), 8)
       11 |-> piece(FEL2 => encodedFloat(?FEl2), 8)

       12 |-> piece(PFEL0 => encodedFloat(?PFEl0), 8) // struct ButterWorthDF2Filter prefilter; (3 floats)
       13 |-> piece(PFEL0 => encodedFloat(?PFEl0), 8) 
       14 |-> piece(PFEL0 => encodedFloat(?PFEl0), 8)
       15 |-> piece(PFEL0 => encodedFloat(?PFEl0), 8)
        // ------------------------------------
       16 |-> piece(PFEL1 => encodedFloat(?PFEl1), 8) 
       17 |-> piece(PFEL1 => encodedFloat(?PFEl1), 8)
       18 |-> piece(PFEL1 => encodedFloat(?PFEl1), 8)
       19 |-> piece(PFEL1 => encodedFloat(?PFEl1), 8)
        // -------------------------------------
       20 |-> piece(PFEL2 => encodedFloat(?PFEl2), 8) 
       21 |-> piece(PFEL2 => encodedFloat(?PFEl2), 8)
       22 |-> piece(PFEL2 => encodedFloat(?PFEl2), 8)
       23 |-> piece(PFEL2 => encodedFloat(?PFEl2), 8)
      
       24 |-> piece(encodedFloat(FF), 8)     // float ff
       25 |-> piece(encodedFloat(FF), 8)
       26 |-> piece(encodedFloat(FF), 8)
       27 |-> piece(encodedFloat(FF), 8)

       28 |-> piece(encodedFloat(FFV), 8)    // float ffv
       29 |-> piece(encodedFloat(FFV), 8)
       30 |-> piece(encodedFloat(FFV), 8)
       31 |-> piece(encodedFloat(FFV), 8)
        
        // storage variables for Butterworth filter
        // float pn1, pn2;
       32 |-> piece(encodedFloat(PN1) => encodedFloat(?PN1), 8) // pn1
       33 |-> piece(encodedFloat(PN1) => encodedFloat(?PN1), 8)
       34 |-> piece(encodedFloat(PN1) => encodedFloat(?PN1), 8)
       35 |-> piece(encodedFloat(PN1) => encodedFloat(?PN1), 8)

       36 |-> piece(encodedFloat(PN2) => encodedFloat(?PN2), 8) // pn2
       37 |-> piece(encodedFloat(PN2) => encodedFloat(?PN2), 8)
       38 |-> piece(encodedFloat(PN2) => encodedFloat(?PN2), 8)
       39 |-> piece(encodedFloat(PN2) => encodedFloat(?PN2), 8)

        // float yn1, yn2;
       40 |-> piece(encodedFloat(YN1) => encodedFloat(?YN1), 8) // yn1
       41 |-> piece(encodedFloat(YN1) => encodedFloat(?YN1), 8)
       42 |-> piece(encodedFloat(YN1) => encodedFloat(?YN1), 8)
       43 |-> piece(encodedFloat(YN1) => encodedFloat(?YN1), 8)

       44 |-> piece(encodedFloat(YN2) => encodedFloat(?YN2), 8) // yn2
       45 |-> piece(encodedFloat(YN2) => encodedFloat(?YN2), 8)
       46 |-> piece(encodedFloat(YN2) => encodedFloat(?YN2), 8)
       47 |-> piece(encodedFloat(YN2) => encodedFloat(?YN2), 8)

        // float v1n1, v1n2;
       48 |-> piece(encodedFloat(V1N1) => encodedFloat(?V1N1), 8) // v1n1
       49 |-> piece(encodedFloat(V1N1) => encodedFloat(?V1N1), 8)
       50 |-> piece(encodedFloat(V1N1) => encodedFloat(?V1N1), 8)
       51 |-> piece(encodedFloat(V1N1) => encodedFloat(?V1N1), 8)

       52 |-> piece(encodedFloat(V1N2) => encodedFloat(?V1N2), 8) // v1n2
       53 |-> piece(encodedFloat(V1N2) => encodedFloat(?V1N2), 8)
       54 |-> piece(encodedFloat(V1N2) => encodedFloat(?V1N2), 8)
       55 |-> piece(encodedFloat(V1N2) => encodedFloat(?V1N2), 8)

        // float v2n1, v2n2;
       56 |-> piece(encodedFloat(V2N1) => encodedFloat(?V2N1), 8) // v2n1
       57 |-> piece(encodedFloat(V2N1) => encodedFloat(?V2N1), 8)
       58 |-> piece(encodedFloat(V2N1) => encodedFloat(?V2N1), 8)
       59 |-> piece(encodedFloat(V2N1) => encodedFloat(?V2N1), 8)

       60 |-> piece(encodedFloat(V2N2) => encodedFloat(?V2N2), 8) // v2n2
       61 |-> piece(encodedFloat(V2N2) => encodedFloat(?V2N2), 8)
       62 |-> piece(encodedFloat(V2N2) => encodedFloat(?V2N2), 8)
       63 |-> piece(encodedFloat(V2N2) => encodedFloat(?V2N2), 8)

        // float v3n1, v3n2;
       64 |-> piece(encodedFloat(V3N1) => encodedFloat(?V3N1), 8) // v3n1
       65 |-> piece(encodedFloat(V3N1) => encodedFloat(?V3N1), 8)
       66 |-> piece(encodedFloat(V3N1) => encodedFloat(?V3N1), 8)
       67 |-> piece(encodedFloat(V3N1) => encodedFloat(?V3N1), 8)

       68 |-> piece(encodedFloat(V3N2) => encodedFloat(?V3N2), 8) // v3n2
       69 |-> piece(encodedFloat(V3N2) => encodedFloat(?V3N2), 8)
       70 |-> piece(encodedFloat(V3N2) => encodedFloat(?V3N2), 8)
       71 |-> piece(encodedFloat(V3N2) => encodedFloat(?V3N2), 8)

       // float Vw1n1, Vw1n2;
       72 |-> piece(encodedFloat(Vw1n1) => encodedFloat(?Vw1n1), 8) // Vw1n1
       73 |-> piece(encodedFloat(Vw1n1 )=> encodedFloat(?Vw1n1), 8)
       74 |-> piece(encodedFloat(Vw1n1) => encodedFloat(?Vw1n1), 8)
       75 |-> piece(encodedFloat(Vw1n1) => encodedFloat(?Vw1n1), 8)

       76 |-> piece(encodedFloat(Vw1n2) => encodedFloat(?Vw1n2), 8) // Vw1n2
       77 |-> piece(encodedFloat(Vw1n2) => encodedFloat(?Vw1n2), 8)
       78 |-> piece(encodedFloat(Vw1n2) => encodedFloat(?Vw1n2), 8)
       79 |-> piece(encodedFloat(Vw1n2) => encodedFloat(?Vw1n2), 8)

       // float Vw2n1, Vw2n2;
       80 |-> piece(encodedFloat(Vw2n1) => encodedFloat(?Vw2n1), 8) // Vw2n1
       81 |-> piece(encodedFloat(Vw2n1) => encodedFloat(?Vw2n1), 8)
       82 |-> piece(encodedFloat(Vw2n1) => encodedFloat(?Vw2n1), 8)
       83 |-> piece(encodedFloat(Vw2n1) => encodedFloat(?Vw2n1), 8)

       84 |-> piece(encodedFloat(Vw2n2) => encodedFloat(?Vw2n2), 8) // Vw2n2
       85 |-> piece(encodedFloat(Vw2n2) => encodedFloat(?Vw2n2), 8)
       86 |-> piece(encodedFloat(Vw2n2) => encodedFloat(?Vw2n2), 8)
       87 |-> piece(encodedFloat(Vw2n2) => encodedFloat(?Vw2n2), 8)

       // float Vw3n1, Vw3n2;
       88 |-> piece(encodedFloat(Vw3n1) => encodedFloat(?Vw3n1), 8) // Vw3n1
       89 |-> piece(encodedFloat(Vw3n1) => encodedFloat(?Vw3n1), 8)
       90 |-> piece(encodedFloat(Vw3n1) => encodedFloat(?Vw3n1), 8)
       91 |-> piece(encodedFloat(Vw3n1) => encodedFloat(?Vw3n1), 8)

       92 |-> piece(encodedFloat(Vw3n2) => encodedFloat(?Vw3n2), 8) // Vw3n2
       93 |-> piece(encodedFloat(Vw3n2) => encodedFloat(?Vw3n2), 8)
       94 |-> piece(encodedFloat(Vw3n2) => encodedFloat(?Vw3n2), 8)
       95 |-> piece(encodedFloat(Vw3n2) => encodedFloat(?Vw3n2), 8)

       // float Vw1, Vw2, Vw3;
       96 |-> piece(encodedFloat(Vw1) => encodedFloat(?Vw1), 8) // Vw1
       97 |-> piece(encodedFloat(Vw1) => encodedFloat(?Vw1), 8)
       98 |-> piece(encodedFloat(Vw1) => encodedFloat(?Vw1), 8)
       99 |-> piece(encodedFloat(Vw1) => encodedFloat(?Vw1), 8)

      100 |-> piece(encodedFloat(Vw2) => encodedFloat(?Vw2), 8) // Vw2
      101 |-> piece(encodedFloat(Vw2) => encodedFloat(?Vw2), 8)
      102 |-> piece(encodedFloat(Vw2) => encodedFloat(?Vw2), 8)
      103 |-> piece(encodedFloat(Vw2) => encodedFloat(?Vw2), 8)

      104 |-> piece(encodedFloat(Vw3) => encodedFloat(?Vw3), 8) // Vw3
      105 |-> piece(encodedFloat(Vw3) => encodedFloat(?Vw3), 8)
      106 |-> piece(encodedFloat(Vw3) => encodedFloat(?Vw3), 8)
      107 |-> piece(encodedFloat(Vw3) => encodedFloat(?Vw3), 8)

       // storage variables for derivative calculation
       // float pOld, yOld;
      108 |-> piece(encodedFloat(POld) => encodedFloat(?POld), 8) // pOld
      109 |-> piece(encodedFloat(POld) => encodedFloat(?POld), 8)
      110 |-> piece(encodedFloat(POld) => encodedFloat(?POld), 8)
      111 |-> piece(encodedFloat(POld) => encodedFloat(?POld), 8)

      112 |-> piece(encodedFloat(YOld) => encodedFloat(?YOld), 8) // yOld
      113 |-> piece(encodedFloat(YOld) => encodedFloat(?YOld), 8)
      114 |-> piece(encodedFloat(YOld) => encodedFloat(?YOld), 8)
      115 |-> piece(encodedFloat(YOld) => encodedFloat(?YOld), 8)

       // float v1Old, v2Old, v3Old;
      116 |-> piece(encodedFloat(V1Old) => encodedFloat(?V1Old), 8) // v1Old
      117 |-> piece(encodedFloat(V1Old) => encodedFloat(?V1Old), 8)
      118 |-> piece(encodedFloat(V1Old) => encodedFloat(?V1Old), 8)
      119 |-> piece(encodedFloat(V1Old) => encodedFloat(?V1Old), 8)

      120 |-> piece(encodedFloat(V2Old) => encodedFloat(?V2Old), 8) // v2Old
      121 |-> piece(encodedFloat(V2Old) => encodedFloat(?V2Old), 8)
      122 |-> piece(encodedFloat(V2Old) => encodedFloat(?V2Old), 8)
      123 |-> piece(encodedFloat(V2Old) => encodedFloat(?V2Old), 8)

      124 |-> piece(encodedFloat(V3Old) => encodedFloat(?V3Old), 8) // v3Old
      125 |-> piece(encodedFloat(V3Old) => encodedFloat(?V3Old), 8)
      126 |-> piece(encodedFloat(V3Old) => encodedFloat(?V3Old), 8)
      127 |-> piece(encodedFloat(V3Old) => encodedFloat(?V3Old), 8), .Map)
    ...</mem>
    <functions> FUNCTIONS:Map </functions>
    <external-types> EXTERNALTYPES:Map </external-types>
    <external-defs> EXTERNALDEFS:Map </external-defs>
    <structs> STRUCTS:Map </structs>
    <linkings> LINKINGS:Map </linkings>
    <translation-units> TRANSLATIONUNITS:Bag </translation-units>
//  <main-tu> _:Set </main-tu>
//</global>
//<T>...
    <files>
      <file-commands> .K </file-commands>
    </files>
    <error-cell-wrapper> .Bag </error-cell-wrapper>
    <status> mainCalled </status>
//  <exec>...
      <restrict> .Map </restrict>
      <activeBlocks> .Map </activeBlocks>
      <malloced> _:Map => ?_:Map </malloced>
      <next-malloced> (_:Int => ?_:Int) @ alloc </next-malloced>
      <threads>
        <thread>...
          <thread-id> 0 </thread-id>
          <buffer> .List </buffer>
          <k> 
            Call1(
                // counter 4 is a function or declaration counter TODO check if this is correct
                tv(loc(4 @ static("imu_airspeed-inlined_calls.c"), 0), // imu airspeed estimation
                    // signature of the function to verify
                    t(.Set, pointerType(t(.Set, functionType(t(.Set, void), ListItem(t(.Set, pointerType(t(.Set, structType(global(Identifier("AirspeedSensorDataPacked"), "imu_airspeed-inlined_calls.c"))))))
                                                                            ListItem(t(.Set, pointerType(t(.Set, structType(global(Identifier("AirspeedSettingsDataPacked"), "imu_airspeed-inlined_calls.c"))))))))))),
                rvlist(
                    // actuals - BP0, BP1
                    ListItem(tv(BP0:CanonicalSymLoc, t(.Set, pointerType(t(.Set, structType(global(Identifier("AirspeedSensorDataPacked"), "imu_airspeed-inlined_calls.c")))))))
                    ListItem(tv(BP1:CanonicalSymLoc, t(.Set, pointerType(t(.Set, structType(global(Identifier("AirspeedSettingsDataPacked"), "imu_airspeed-inlined_calls.c")))))))))
          =>
            tv(_, t(.Set, void))
          ...</k>
          <initializing> 0 </initializing>
          <duration> auto(0) </duration>
//        <thread-local>...
//          <control>...
//            <control-details>...
                <type-wrapper> .Bag </type-wrapper>
                <curr-subobject> .List </curr-subobject>
                <locs-written> _:List => .List </locs-written>
//              <local>...
//                <types> ... </types>
                  <formatting-wrapper> .Bag </formatting-wrapper>
//                <env> ... </env>
//              ...</local>
//            ...</control-details>
              <curr-program-loc> _ => ?_ </curr-program-loc>
              <curr-function> _:CId </curr-function>
//          ...</control>
//          <call-stack> _:List </call-stack>
//        ...</thread-local>
        ...</thread>
      </threads>
//  ...</exec>
//...</T>

ensures         ?LFF  ==K Int2Float(SP) /Float 100.00 /Float LP1 
        andBool ?LFFV ==K Int2Float(SP) /Float 100.00 /Float LP2
        // ButterWorthDF2 filter initialization
        andBool ?COND_BW0 ==K (absFloat(?LFFV -Float FFV) >Float EPS)
        andBool ?COND_BW1 ==K (absFloat(?LFF  -Float FF ) >Float EPS)
        andBool eq_ite(?COND_BW0, ?FEl0,  b0_bwf(?LFFV), FEl0)
        andBool eq_ite(?COND_BW0, ?FEl1,  a1_bwf(?LFFV), FEl1)
        andBool eq_ite(?COND_BW0, ?FEl2,  a2_bwf(?LFFV), FEl2)
        andBool eq_ite(?COND_BW1, ?PFEl0, b0_bwf(?FFV), FEl0)
        andBool eq_ite(?COND_BW1, ?PFEl1, a1_bwf(?FFV), FEl1)
        andBool eq_ite(?COND_BW1, ?PFEl2, a2_bwf(?FFV), FEl2)
 
        andBool eq_ite(?COND_BW0, ?Vw1n1P, wn1_bwv(Vw1, ?FEl0, ?FEl1, ?FEl2), Vw1n1)
        andBool eq_ite(?COND_BW0, ?Vw1n2P, wn2_bwv(Vw1, ?FEl0, ?FEl1, ?FEl2), Vw1n2)
        andBool eq_ite(?COND_BW0, ?Vw2n1P, wn1_bwv(Vw2, ?FEl0, ?FEl1, ?FEl2), Vw2n1)
        andBool eq_ite(?COND_BW0, ?Vw2n2P, wn2_bwv(Vw2, ?FEl0, ?FEl1, ?FEl2), Vw2n2)
        andBool eq_ite(?COND_BW0, ?Vw3n1P, wn1_bwv(Vw3, ?FEl0, ?FEl1, ?FEl2), Vw3n1)
        andBool eq_ite(?COND_BW0, ?Vw3n2P, wn2_bwv(Vw3, ?FEl0, ?FEl1, ?FEl2), Vw3n2)

        andBool eq_ite(?COND_BW1, ?PN1P, wn1_bwv(POld, ?PFEl0, ?PFEl1, ?PFEl2), PN1)
        andBool eq_ite(?COND_BW1, ?PN2P, wn2_bwv(POld, ?PFEl0, ?PFEl1, ?PFEl2), PN2)
        andBool eq_ite(?COND_BW1, ?YN1P, wn1_bwv(YOld, ?PFEl0, ?PFEl1, ?PFEl2), YN1)
        andBool eq_ite(?COND_BW1, ?YN2P, wn1_bwv(YOld, ?PFEl0, ?PFEl1, ?PFEl2), YN2)
        andBool eq_ite(?COND_BW1, ?V1N1P, wn1_bwv(V1Old, ?PFEl0, ?PFEl1, ?PFEl2), V1N1)
        andBool eq_ite(?COND_BW1, ?V1N2P, wn2_bwv(V1Old, ?PFEl0, ?PFEl1, ?PFEl2), V1N2)
        andBool eq_ite(?COND_BW1, ?V2N1P, wn1_bwv(V2Old, ?PFEl0, ?PFEl1, ?PFEl2), V2N1)
        andBool eq_ite(?COND_BW1, ?V2N2P, wn2_bwv(V2Old, ?PFEl0, ?PFEl1, ?PFEl2), V2N2)
        andBool eq_ite(?COND_BW1, ?V3N1P, wn1_bwv(V3Old, ?PFEl0, ?PFEl1, ?PFEl2), V3N1)
        andBool eq_ite(?COND_BW1, ?V3N2P, wn2_bwv(V3Old, ?PFEl0, ?PFEl1, ?PFEl2), V3N2)
        // ------------------------------------------------------------------------------

        // Quaternion2PY : find the pitch and the yaw from quaternion
        andBool ?Q2PY ==K spec4Quaternion2PY(ATQ0, ATQ1, ATQ2, ATQ3, YOld)
        andBool ?YP ==K fst2(Q2PY) andBool ?PP ==K snd2(Q2PY)

        // Apply ButterWorthDF2 filter
        andBool ?PFLT ==K spec4FilterButterWorth(?PP, ?PFEl0, ?PFEl1, ?PFEl2, ?PN1P, ?PN2P); 
        andBool ?P ==K fst3(?PFLT) andBool ?PN1 ==K snd3(?PFLT) andBool ?PN2 ==K trd3(?PFLT)
        andBool ?YFLT ==K spec4FilterButterWorth(?YP, ?PFEl0, ?PFEl1, ?PFEl2, ?YN1P, ?YN2P); 
        andBool ?Y ==K fst3(?YFLT) andBool ?YN1 ==K snd3(?YFLT) andBool ?YN2 ==K trd3(?YFLT)

        // transform pitch and yaw into fuselage vector XB and DXB
        andBool ?XB  ==K spec4PY2xB(?P, ?Y)               // NOTE: XB is a FloatTriple
        // calculate change in fuselage vector by subraction from the old value
        andBool ?DXB ==K spec4PY2DeltaxB(POld, YOld, ?XB) // NOTE: DXB is a FloatTriple

        // filter ground speed from VelocityState
        andBool ?FVS1 ==K spec4FilterButterWorth(VSN, ?PFEl0, ?PFEl1, ?PFEl2, ?V1N1P, ?V1N2P)
        andBool ?FVS2 ==K spec4FilterButterWorth(VSE, ?PFEl0, ?PFEl1, ?PFEl2, ?V2N1P, ?V2N2P)
        andBool ?FVS3 ==K spec4FilterButterWorth(VSD, ?PFEl0, ?PFEl1, ?PFEl2, ?V3N1P, ?V3N2P)
        andBool ?Fv1n ==K fst3(?FVS1) andBool ?V1N1 ==K snd3(?FVS1) andBool ?V1N2 ==K trd3(?FVS1)
        andBool ?Fv2n ==K fst3(?FVS2) andBool ?V2N1 ==K snd3(?FVS2) andBool ?V2N2 ==K trd3(?FVS2)
        andBool ?Fv3n ==K fst3(?FVS3) andBool ?V3N1 ==K snd3(?FVS3) andBool ?V3N2 ==K trd3(?FVS3)

        // calculate norm of ground speed
        andBool ?NormVel2 ==K sq_float(?Fv1n) +Float sq_float(?Fv2n) +Float sq_float(?Fv3n)
        // calculate norm of orientation change
        andBool ?NormDiffAttitude2 ==K sq_float(fst3(?DXB)) +Float sq_float(snd3(?DXB)) +Float sq_float(trd3(?DXB))
        // calculate scalar product between groundspeed change and orientation change
        andBool ?DvdtDotdfdt ==K (Fv1n -Float V1Old) *Float fst3(?DXB) +Float (Fv2n -Float V2Old) *Float snd3(?DXB) +Float (Fv3n -Float V3Old) *Float trd3(?DXB)

        // actualize old values
        andBool ?POld ==K ?P
        andBool ?YOld ==K ?Y
        andBool ?V1Old ==K ?Fv1n
        andBool ?V2Old ==K ?Fv2n
        andBool ?V3Old ==K ?Fv3n

        // calculate airspeed estimate
        andBool ?Airspeed ==K ?DvdtDotdfdt /Float ?NormDiffAttitude2

        // calculate windspeed estimate
        andBool ?Wind ==K calc_wind(?V1Old, ?V2Old, ?V3Old, ?XB, ?Airspeed)

        // apply butterworth filter to the wind estimate
        andBool ?FRW0 ==K spec4FilterButterWorth(fst3(?Wind), ?FEl0, ?FEl1, ?FEl2, ?Vw1n1P, ?Vw1n2P)
        andBool ?FRW1 ==K spec4FilterButterWorth(snd3(?Wind), ?FEl0, ?FEl1, ?FEl2, ?Vw2n1P, ?Vw2n2P)
        andBool ?FRW2 ==K spec4FilterButterWorth(trd3(?Wind), ?FEl0, ?FEl1, ?FEl2, ?Vw3n1P, ?Vw3n2P)

        // conditionally update the VW[1--3]
        andBool ?COND_POS ==K (?NormDiffAttitude2 >Float EPS_REORIENTATION andBool ?NormVel2 >Float EPS_VELOCITY andBool ?DvdtDotdfdt >Float 0.0)
        andBool eq_ite(COND_POS, ?Vw1, fst3(FRW0), Vw1) 
        andBool eq_ite(COND_POS, ?Vw2, fst3(FRW0), Vw2) 
        andBool eq_ite(COND_POS, ?Vw3, fst3(FRW0), Vw3) 
        andBool eq_ite(COND_POS, ?Vw1n1, snd3(FRW0), ?Vw1n1P) 
        andBool eq_ite(COND_POS, ?Vw2n1, snd3(FRW0), ?Vw2n1P) 
        andBool eq_ite(COND_POS, ?Vw3n1, snd3(FRW0), ?Vw3n1P) 
        andBool eq_ite(COND_POS, ?Vw1n2, trd3(FRW0), ?Vw1n2P) 
        andBool eq_ite(COND_POS, ?Vw2n2, trd3(FRW0), ?Vw2n2P) 
        andBool eq_ite(COND_POS, ?Vw3n2, trd3(FRW0), ?Vw3n2P) 

        // compute airspeed
        andBool Vair ==K calc_airspeed(?V1Old, ?V2Old, ?V3Old, ?Vw1, ?Vw2, ?Vw3)
        // ************* FINAL RESULT ********************************************
        // project airspeed onto fuselage vector
        andBool ?CALIB_AIR_SPEED ==K       fst3(Vair) *Float fst3(XB) 
                                    +Float snd3(Vair) *Float snd3(XB)
                                    +Float trd3(Vair) *Float trd3(XB)
        // **********************************************************************


endmodule
