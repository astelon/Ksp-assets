// ============================================================================
//  rendezvous.ks  --  Orbit-to-orbit rendezvous with a station or vessel
// ----------------------------------------------------------------------------
//  Target : another vessel orbiting the same body.  The script matches its
//           plane, phases into a transfer, intercepts it, nulls the relative
//           velocity, and parks a few hundred metres off it in an orbit that is
//           almost identical to the target's.
//
//  It does NOT dock.  Docking needs a chosen port on both ships and a different
//  control law; this script's job is to deliver the ship to the doorstep with
//  the relative velocity dead and the nose on the target, which is the state a
//  docking script wants to start from.
//
//  Run with:   RUN rendezvous.                   - whatever is selected on the map
//              RUN rendezvous("Station Alpha").  - or by name (substring is enough)
//              RUN rendezvous("Alpha", 250).     - ...and a park distance, in metres
//
//  Design notes (why it flies the way it does)
//  -------------------------------------------
//  * A RENDEZVOUS IS A TIMING PROBLEM, NOT A STEERING PROBLEM.  Getting into
//    the target's orbit is easy and almost useless: an orbit is a place you are
//    at a time, and two ships in the same orbit half a lap apart are as far from
//    a rendezvous as it is possible to be.  Every phase below exists to control
//    *when* we arrive, and the burns that do it are ordinary Hohmann burns.
//  * THE PHASING ORBIT IS SOLVED, NOT PICKED.  The classic transfer needs the
//    target sitting at a particular lead angle when we leave, and that angle
//    comes round at a rate set by the difference between the two orbital
//    periods.  Which is the whole difficulty: ascent.ks parks at 100 km and a
//    station at 100 km has *no* period difference, so the lead angle we need
//    never arrives and a script that simply waits for it waits forever.  So the
//    script does not wait - it sweeps candidate phasing orbits, prices each one
//    (two burns to get in and out, against the wait it produces), and flies the
//    cheapest one whose total time fits MAX_TOTAL_TIME.  Dropping to a lower
//    orbit to catch up is a decision with a number attached, and the number is
//    printed.
//  * THE CLOSED FORM PLANS, THE SEARCH AIMS.  Vis-viva assumes two circular,
//    coplanar orbits and an impulsive burn, and the real problem is none of
//    those things: the target's orbit is slightly eccentric, our plane match
//    left a residue, and the burn takes half a minute.  So the transfer node is
//    *planted* from the closed form and then *refined* numerically - kOS's
//    POSITIONAT accounts for planned maneuver nodes on the active vessel, so
//    the script can nudge the node and read the closest approach it would
//    actually produce.  The closed form gets it to a few tens of km; the search
//    takes it to the intercept distance.
//      That refinement is guarded, not trusted.  Before using it the script
//    plants a throwaway node and checks that the prediction actually moves.  If
//    it does not, the closed-form plan flies unrefined and the mid-course
//    correction - which is measured off the real post-burn orbit, with no node
//    involved - cleans up behind it.
//  * WE AIM TO MISS.  INTERCEPT_DIST is a couple of kilometres, not zero.  Two
//    conics that both ships are only approximately on will not deliver a
//    zero-mile pass, and a plan that asks for one buys a fast crossing with no
//    time to brake.  Aiming a little wide costs nothing - the arrival burn nulls
//    the relative velocity either way - and it leaves room to be wrong in.
//  * THE APPROACH SPEED IS WHAT THE SHIP CAN STOP FROM.  The terminal phase
//    closes at no more than sqrt(2*a*d), where a is the acceleration the RCS
//    actually produces on the mass actually aboard and d is the distance left
//    to the park point.  A loaded Mk3 with 100 t in the bays has a quarter of
//    the translational authority it has empty, and it gets a quarter of the
//    approach speed without anyone retuning anything.  A fixed closing rate is
//    how a heavy ship arrives at a station it cannot stop before.
//  * IT NEVER BURNS DRY.  Same rule ascent.ks flies by, for the same reason:
//    every burn is policed against a reserve that funds the deorbit and the
//    glide home, and no manoeuvre may breach it.  A ship parked next to a
//    station with no way home has not completed a rendezvous.  The reserve is
//    priced at the mass the deorbit burn will actually be flown at, which -
//    after the cargo is delivered - is not the mass on the gauge now.
//  * A RENDEZVOUS CANNOT BE HALF-DONE.  The reserve above says where a burn
//    stops; this says which burn is allowed to be the one that stops there.
//    Every phase up to the transfer injection ends in a circular orbit the
//    ship can sit in, so flying one on a short budget wastes propellant and
//    nothing else.  The arrival burn is different: everything spent getting to
//    the intercept is spent buying a relative velocity that then has to be
//    paid off, and an arrival burn cut short does not deliver a partial
//    rendezvous - it delivers a flyby, at speed, with the tanks empty and the
//    target receding.  So the arrival is priced into the bill, priced again
//    from the real geometry at the point of no return, and the injection is
//    refused unless it is funded.  Refusing to go is a result.  Going and not
//    arriving is worse than either.
//  * THE PLAN IS SEARCHED UNDER THE BUDGET, NOT PRICED AFTER IT.  Both the
//    phasing sweep and the transfer refinement carry the cost of arriving in
//    their objective.  A hill-climb told only to minimise the miss will buy a
//    steeper, faster intercept every round, because a steeper one aims better
//    - and spend the arrival burn's propellant doing it, before the arrival
//    burn is lit.  When nothing inside MAX_TOTAL_TIME can be paid for, the
//    sweep looks further out rather than flying something it cannot finish:
//    waiting is free and propellant is not.
//  * PAYLOAD FUEL IS NOT OUR FUEL.  Same part-tree walk ascent.ks does, and it
//    matters more here: on a station run the payload usually *is* propellant,
//    and counting the cargo's tanks as budget would fund a rendezvous with fuel
//    that is supposed to be left behind.
//
//  Flight plan
//    0. Pre-flight  : acquire the target, map the fuel network, price the whole
//                     rendezvous, GO/NO-GO, log the resources aboard.
//    1. Plane match : burn normal at the cheaper relative node (skipped if the
//                     relative inclination is already negligible).
//    2. Tidy        : circularise if the ascent left the orbit eccentric.
//    3. Phasing     : solve and fly the phasing orbit, then warp the wait out.
//    4. Transfer    : Hohmann injection, refined against the real geometry, plus
//                     a mid-course correction measured off the actual orbit.
//    5. Arrival     : at closest approach, kill the relative velocity.
//    6. Terminal    : brake-limited RCS approach to the park distance.
//    7. Park        : hold station, report, hand over for docking.
//
//  Tunables are grouped at the top so you can trim them to your build.
// ============================================================================

// The vessel to meet.  Leave it out and the script uses whatever is selected as
// the target on the map, which is usually what you want.  A partial name is
// enough - "alpha" finds "Kerbin Station Alpha" - but an ambiguous one is
// refused rather than guessed at.
//
// (Optional program parameters need kOS 1.0.1 or newer.  On anything older,
// drop the "IS" defaults and always pass both values.)
DECLARE PARAMETER targetName IS "".
DECLARE PARAMETER parkMetres IS 200.

CLEARSCREEN.

// --- Mission ----------------------------------------------------------------
SET PARK_DIST     TO parkMetres.  // where we stop, short of the target (m)
SET PARK_BAND     TO 60.          // station-keeping deadband around it (m)
SET PARK_V_TOL    TO 0.25.        // relative speed that counts as parked (m/s)
SET PARK_HOLD_T   TO 15.          // hold it this long before handing over (s)
SET NEAR_ALREADY  TO 25000.       // start already inside this range and the
                                  // transfer phases are skipped entirely (m)

// --- dV budgeting -----------------------------------------------------------
//  Identical policy to ascent.ks, and deliberately so: the two scripts have to
//  agree about what the ship is allowed to spend, or one of them will spend the
//  other's reserve.
SET DEORBIT_PE          TO 32000. // periapsis deorbit_land.ks burns down to (m)
SET DV_MARGIN           TO 100.   // spare kept on top of the deorbit burn (m/s)
SET DV_GLIDE_RESERVE    TO 120.   // never burn below this, ever (m/s)
SET ROCKET_ISP_FALLBACK TO 305.   // closed-cycle Isp if none can be read (s)
SET RCS_ISP_FALLBACK    TO 240.   // monopropellant thruster Isp (s)
SET RCS_THRUST_BLOCK    TO 1.0.   // thrust per RCS block (kN) - RV-105 is 1 kN
SET LFO_LF_RATIO        TO 9.     // stock LF:Ox mixture ratio, LF part
SET LFO_OX_RATIO        TO 11.    // ... Ox part
SET DEPLOY_BEFORE_DEORBIT TO TRUE.// the cargo comes off at the station, so the
                                  // deorbit burn is flown light and the reserve
                                  // is priced accordingly.  FALSE reserves
                                  // enough to bring the payload home again
SET ISOLATE_PAYLOAD     TO TRUE.  // block crossfeed on the payload separator
SET FORCE_TANK_LOCK     TO FALSE. // also disable flow on payload tanks (only
                                  // needed if no separator takes a crossfeed
                                  // toggle; remember to re-enable on deploy)
SET ABORT_IF_INFEASIBLE TO FALSE. // TRUE = do not leave the parking orbit at
                                  // all once the rendezvous is priced out of
                                  // reach.  FALSE = set off anyway on the
                                  // recoverable half of the plan.  Either way
                                  // the transfer injection is refused unless
                                  // the arrival that follows it is paid for -
                                  // see phase 4c.  A rendezvous is not a thing
                                  // you can do most of
SET GO_MARGIN           TO 0.10.  // dV in hand, as a fraction of the bill,
                                  // below which the answer is MARGINAL, not GO
SET PREFLIGHT_HOLD      TO 12.    // pause this long on a failed check (s)
SET PE_SAFETY           TO 8000.  // no phasing orbit's periapsis may come
                                  // closer than this to the atmosphere (m)

// --- Plane matching ---------------------------------------------------------
SET PLANE_SKIP_DI  TO 0.05.       // relative inclination below this is left
                                  // alone (deg).  ascent.ks launches due east,
                                  // so against an equatorial station this is
                                  // usually already true, and 3 m/s spent
                                  // chasing a hundredth of a degree is worse
                                  // than the hundredth of a degree
SET PLANE_SCAN_STEP TO 30.        // coarse step hunting the node crossing (s)
SET PLANE_SCAN_ORBITS TO 1.2.     // how far ahead to look for one
SET ALIGN_TOL      TO 5.          // steering error accepted before burning (deg)

// --- Phasing ----------------------------------------------------------------
SET MAX_TOTAL_TIME TO 7200.       // longest rendezvous the solver will accept
                                  // (s).  Raise it to be allowed cheaper, more
                                  // patient plans; lower it to be in a hurry
SET MAX_TIME_PATIENT TO 28800.    // ... and the horizon it is allowed to fall
                                  // back to when nothing inside MAX_TOTAL_TIME
                                  // can be paid for (s).  Waiting is free and
                                  // propellant is not, so a plan that costs
                                  // four more hours and fits the tanks beats
                                  // a quick one that runs out at the target
SET PHASE_GRID     TO 40.         // candidate phasing radii tried per side
SET PHASE_R_SPAN   TO 0.35.       // how far from the target's radius the search
                                  // may roam, as a fraction of it
SET ECC_TIDY       TO 0.01.       // eccentricity above which the orbit is
                                  // circularised before the phasing maths -
                                  // which assumes a circular orbit - is trusted
SET PHASE_LEAD     TO 60.         // plan burns at least this far ahead (s)

// --- Transfer ---------------------------------------------------------------
SET INTERCEPT_DIST TO 2000.       // closest approach the plan aims for (m)
SET CORR_DV_ALLOW  TO 25.         // budgeted allowance for corrections (m/s)
SET REFINE_ROUNDS  TO 14.         // hill-climb rounds on the transfer node
SET REFINE_DV0     TO 15.         // its starting step, on the node's dV (m/s)
SET REFINE_DT0     TO 60.         // ... and on the node's time (s)
SET REFINE_FLOOR   TO 0.05.       // stop refining below this dV step (m/s)
SET CA_COARSE      TO 60.         // samples per closest-approach scan
SET CA_REFINE      TO 18.         // ternary-search rounds after the scan
SET MIDCOURSE_FRAC TO 0.35.       // fraction of the way across to correct at
SET MIDCOURSE_MIN  TO 500.        // do not bother correcting a miss under this (m)

// --- Arrival ----------------------------------------------------------------
SET ARRIVE_DV_ALLOW TO 40.        // budgeted allowance for the arrival burn on
                                  // top of the transfer's own closing burn
                                  // (m/s).  The closed-form second burn prices
                                  // a tangential arrival into a circular orbit
                                  // and the ship never arrives quite like that;
                                  // this is what the difference is allowed to be
SET ARRIVE_DV_FLOOR TO 5.         // slack left over the predicted arrival cost
                                  // when deciding whether to commit (m/s).  It
                                  // pays for the mid-course and for the
                                  // prediction being a prediction
SET TERM_V_MARGIN  TO 1.25.       // the terminal phase wants this much more
                                  // budget than the residual it has to null
SET ARRIVE_V_TOL   TO 0.4.        // relative speed the arrival burn stops at (m/s)
SET RCS_HANDOVER   TO 4.          // below this remaining dV the arrival burn is
                                  // flown on RCS instead of the main engines (m/s)
SET ARRIVE_LEAD    TO 5.          // start the arrival burn this long early (s)

// --- Terminal approach ------------------------------------------------------
SET V_CLOSE_MAX    TO 15.         // hard cap on closing speed, whatever the
                                  // braking schedule would allow (m/s)
SET V_CLOSE_MIN    TO 0.8.        // ... and the crawl it closes the last bit at
SET BRAKE_SAFETY   TO 2.5.        // braking distance is padded by this factor
SET TRANS_GAIN     TO 1.2.        // RCS command per m/s of velocity error
SET TRANS_DEAD     TO 0.05.       // ... below which the axis is left alone
SET RCS_ACC_MIN    TO 0.02.       // never believe an RCS accel lower than this
SET MONO_RESERVE   TO 15.         // stop closing with less mono than this (units)
SET TERM_TIMEOUT   TO 900.        // longest the terminal approach may run (s)

// --- Warp / logging ---------------------------------------------------------
SET USE_WARP       TO TRUE.       // time-warp the long waits
SET WARP_LEAD      TO 25.         // drop out of warp this long before a burn (s)
SET LOG_TO_FILE    TO FALSE.      // also append the resource snapshots to a file
SET LOG_PATH       TO "0:/rendezvous_log.csv".

// ---------------------------------------------------------------------------
//  Body / physical constants (cached once)
// ---------------------------------------------------------------------------
SET G0      TO 9.80665.
SET BODY_MU TO SHIP:BODY:MU.
SET BODY_R  TO SHIP:BODY:RADIUS.
SET ATM_TOP TO SHIP:BODY:ATM:HEIGHT.
SET SOI_R   TO SHIP:BODY:SOIRADIUS.

// The lowest radius any plan of ours may pass through: clear of the air, with
// the safety margin on top.
SET R_FLOOR TO BODY_R + ATM_TOP + PE_SAFETY.

// kOS runs a fixed number of instructions per physics tick, and the numerical
// searches below are the most instruction-hungry thing in this repository.
// Restored on exit.
SET IPU_SAVED TO CONFIG:IPU.
SET CONFIG:IPU TO 1000.

// ---------------------------------------------------------------------------
//  Small helpers
// ---------------------------------------------------------------------------
//  NOTE ON NAMING: kOS refuses to compile a script that declares a variable
//  hiding one of its built-in functions (the CLOBBERBUILTINS check), and the
//  list is much longer than the one-letter ones - NODE, LIST, RANGE, QUEUE,
//  STACK, PATH, BODY, TRANSFER and VESSEL are all built-ins, and all of them
//  are words you would reach for while writing a rendezvous.  Run
//  `tools/check_kos.py` after editing; it checks before the game does.

// Clamp `valIn` into [loV, hiV].  Note the argument order differs from
// dlClampVal in deorbit_land.ks - that one takes (lo, hi, value).  Keep them
// straight when copying a line between the two files.
FUNCTION rdvClampVal {
  PARAMETER valIn, loV, hiV.
  RETURN MAX(loV, MIN(hiV, valIn)).
}

FUNCTION rdvNormAng {                  // wrap an angle into (-180, 180]
  PARAMETER angIn.
  LOCAL ang IS angIn.
  UNTIL ang <= 180 { SET ang TO ang - 360. }
  UNTIL ang > -180 { SET ang TO ang + 360. }
  RETURN ang.
}

// ---------------------------------------------------------------------------
//  Fuel-network mapping  --  which tanks are actually ours?
// ---------------------------------------------------------------------------
//  Lifted from ascent.ks, and for the same reason.  A payload rides on a
//  decoupler, and a fuelled payload is not fuel we may spend: counting it would
//  fund a rendezvous with propellant that is meant to be delivered, and letting
//  the engines drink it would strand the cargo.  Rather than trust part names,
//  walk the part tree outward from the engines and refuse to cross anything
//  that can separate.  Whatever we reach is the ship; the rest is payload.
// ---------------------------------------------------------------------------
FUNCTION rdvIsSeparator {
  PARAMETER prt.
  FOR mname IN prt:MODULES {
    LOCAL low IS mname:TOLOWER.
    IF low:CONTAINS("decouple") OR low:CONTAINS("separator") OR
       low:CONTAINS("dockingnode") OR low:CONTAINS("launchclamp") {
      RETURN TRUE.
    }
  }
  RETURN FALSE.
}

FUNCTION rdvMapFeedNetwork {
  LOCAL seen    IS LEXICON().
  LOCAL pending IS LIST().
  LOCAL engs    IS LIST().
  LIST ENGINES IN engs.
  FOR eng IN engs {
    IF NOT seen:HASKEY(eng:UID) { seen:ADD(eng:UID, TRUE). pending:ADD(eng). }
  }
  LOCAL idx IS 0.
  UNTIL idx >= pending:LENGTH {
    LOCAL prt IS pending[idx].
    SET idx TO idx + 1.
    IF NOT rdvIsSeparator(prt) {
      LOCAL nbrs IS LIST().
      IF prt:HASPARENT { nbrs:ADD(prt:PARENT). }
      FOR kid IN prt:CHILDREN { nbrs:ADD(kid). }
      FOR nbr IN nbrs {
        IF NOT seen:HASKEY(nbr:UID) { seen:ADD(nbr:UID, TRUE). pending:ADD(nbr). }
      }
    }
  }
  RETURN seen.
}

SET CORE_UIDS  TO rdvMapFeedNetwork().
SET CORE_TANKS TO LIST().           // our tanks - the only ones counted as dV
SET PAY_TANKS  TO LIST().           // payload tanks - counted, then locked out
SET SEPARATORS TO LIST().
SET RCS_BLOCKS TO 0.
SET payloadMass TO 0.

FOR prt IN SHIP:PARTS {
  LOCAL holds IS FALSE.
  FOR res IN prt:RESOURCES {
    IF res:NAME = "LiquidFuel" OR res:NAME = "Oxidizer" { SET holds TO TRUE. }
  }
  FOR mname IN prt:MODULES {
    IF mname:TOLOWER:CONTAINS("modulercs") { SET RCS_BLOCKS TO RCS_BLOCKS + 1. }
  }
  IF CORE_UIDS:HASKEY(prt:UID) {
    IF holds { CORE_TANKS:ADD(prt). }
    IF rdvIsSeparator(prt) { SEPARATORS:ADD(prt). }
  } ELSE {
    SET payloadMass TO payloadMass + prt:MASS.
    IF holds { PAY_TANKS:ADD(prt). }
  }
}

FUNCTION rdvCoreResAmt {               // amount inside the engines' feed network
  PARAMETER rname.
  LOCAL total IS 0.
  FOR prt IN CORE_TANKS {
    FOR res IN prt:RESOURCES {
      IF res:NAME = rname { SET total TO total + res:AMOUNT. }
    }
  }
  RETURN total.
}

FUNCTION rdvListResAmt {               // same, over an arbitrary part list
  PARAMETER prts, rname.
  LOCAL total IS 0.
  FOR prt IN prts {
    FOR res IN prt:RESOURCES {
      IF res:NAME = rname { SET total TO total + res:AMOUNT. }
    }
  }
  RETURN total.
}

FUNCTION rdvResAmtShip {               // whole-ship total
  PARAMETER rname.
  LOCAL total IS 0.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = rname { SET total TO total + res:AMOUNT. }
  }
  RETURN total.
}

FUNCTION rdvResDensity {               // tonnes per unit
  PARAMETER rname, fallback.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = rname AND res:DENSITY > 0 { RETURN res:DENSITY. }
  }
  RETURN fallback.
}

SET LF_DENS   TO rdvResDensity("LiquidFuel", 0.005).
SET OX_DENS   TO rdvResDensity("Oxidizer", 0.005).
SET MONO_DENS TO rdvResDensity("MonoPropellant", 0.004).

FUNCTION rdvBlockCrossfeed {
  LOCAL blocked IS 0.
  FOR prt IN SEPARATORS {
    FOR mname IN prt:MODULES {
      LOCAL pmod IS prt:GETMODULE(mname).
      IF pmod:HASEVENT("disable crossfeed") {
        pmod:DOEVENT("disable crossfeed").
        SET blocked TO blocked + 1.
      }
    }
  }
  RETURN blocked.
}

FUNCTION rdvLockPayloadTanks {
  LOCAL locked IS 0.
  FOR prt IN PAY_TANKS {
    FOR res IN prt:RESOURCES {
      IF res:TOGGLEABLE AND res:ENABLED {
        SET res:ENABLED TO FALSE.
        SET locked TO locked + 1.
      }
    }
  }
  RETURN locked.
}

// ---------------------------------------------------------------------------
//  Ship measurement
// ---------------------------------------------------------------------------
FUNCTION rdvMeasureRocketIsp {
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  LOCAL wsum IS 0.
  LOCAL isum IS 0.
  FOR eng IN engs {
    IF (NOT eng:MULTIMODE) OR (eng:MODE = "ClosedCycle") {
      IF eng:MAXTHRUST > 0 AND eng:VACUUMISP > 0 {
        SET wsum TO wsum + eng:MAXTHRUST.
        SET isum TO isum + eng:MAXTHRUST * eng:VACUUMISP.
      }
    }
  }
  IF wsum > 0 { RETURN isum / wsum. }
  RETURN ROCKET_ISP_FALLBACK.
}

// Force every multimode engine into closed cycle.  In vacuum the air-breathing
// mode reports no thrust at all, which would poison every burn-time estimate
// below and leave the ship sitting at full throttle going nowhere.
FUNCTION rdvGoClosedCycle {
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  FOR eng IN engs {
    IF eng:MULTIMODE {
      SET eng:AUTOSWITCH TO FALSE.
      IF eng:MODE = "AirBreathing" { eng:TOGGLEMODE(). }
    }
  }
}

// Propellant the closed-cycle engines can actually burn: our LF and Ox only,
// and only in 9:11 pairs, so a lopsided tank state is not counted as dV.
FUNCTION rdvRocketPropMass {
  LOCAL lfAmt IS rdvCoreResAmt("LiquidFuel").
  LOCAL oxAmt IS rdvCoreResAmt("Oxidizer").
  LOCAL lfUse IS MIN(lfAmt, oxAmt * LFO_LF_RATIO / LFO_OX_RATIO).
  LOCAL oxUse IS lfUse * LFO_OX_RATIO / LFO_LF_RATIO.
  RETURN lfUse * LF_DENS + oxUse * OX_DENS.
}

FUNCTION rdvRocketDv {                 // remaining closed-cycle dV (m/s)
  LOCAL m0 IS SHIP:MASS.
  LOCAL m1 IS m0 - rdvRocketPropMass().
  IF m0 <= 0 OR m1 <= 0 OR m1 >= m0 { RETURN 0. }
  RETURN RKT_ISP * G0 * LN(m0 / m1).
}

// The monopropellant aboard, as dV.  This is a second, separate currency: the
// terminal approach is flown on RCS, and running the mono dry a hundred metres
// off a station leaves the ship drifting with no way to stop.
FUNCTION rdvMonoDv {
  LOCAL m0 IS SHIP:MASS.
  LOCAL m1 IS m0 - rdvResAmtShip("MonoPropellant") * MONO_DENS.
  IF m0 <= 0 OR m1 <= 0 OR m1 >= m0 { RETURN 0. }
  RETURN RCS_ISP_FALLBACK * G0 * LN(m0 / m1).
}

// The part of it that is actually ours to spend.  MONO_RESERVE is not a low
// fuel warning, it is propellant already promised to station-keeping, so a
// decision about whether the approach can be flown must be taken on what is
// left above it and not on what the gauge reads.
FUNCTION monoDvUsable {
  LOCAL amt IS MAX(0, rdvResAmtShip("MonoPropellant") - MONO_RESERVE).
  LOCAL m0  IS SHIP:MASS.
  LOCAL m1  IS m0 - amt * MONO_DENS.
  IF m0 <= 0 OR m1 <= 0 OR m1 >= m0 { RETURN 0. }
  RETURN RCS_ISP_FALLBACK * G0 * LN(m0 / m1).
}

// Main-engine dV that may be spent without touching the reserve.  Every "can
// we still do this" question below is asked in these terms rather than in
// gauge readings, because the gauge includes the ride home.
FUNCTION rocketDvUsable {
  PARAMETER floorDv.
  RETURN MAX(0, rdvRocketDv() - floorDv).
}

FUNCTION rdvBurnTimeFor {              // seconds to spend dvNeed at full throttle
  PARAMETER dvNeed.
  LOCAL thr IS SHIP:AVAILABLETHRUST.
  IF thr <= 0 OR dvNeed <= 0 { RETURN 0. }
  LOCAL vEx IS RKT_ISP * G0.
  RETURN SHIP:MASS * vEx * (1 - CONSTANT:E ^ (-dvNeed / vEx)) / thr.
}

// Translational acceleration the RCS can produce, in m/s^2.  Counted off the
// blocks actually installed and the mass actually aboard, so a full cargo bay
// gets a gentler approach without anyone retuning anything.  It is refined in
// flight from what the thrusters really deliver - see rdvApproachAccel().
SET rcsAccEst TO 0.15.

FUNCTION rdvRcsAccelNominal {
  IF SHIP:MASS <= 0 { RETURN RCS_ACC_MIN. }
  RETURN MAX(RCS_ACC_MIN, RCS_BLOCKS * RCS_THRUST_BLOCK / SHIP:MASS).
}

FUNCTION rdvApproachAccel {            // the figure the braking schedule uses
  RETURN MAX(RCS_ACC_MIN, rcsAccEst).
}

// A dV figure quoted for the *light* ship, converted into what it costs on the
// gauge here, with the cargo still aboard.  The deorbit burn happens after the
// payload is delivered, on a ship that can be half this mass, where the same
// tonnes of propellant are worth far more; reserving the post-delivery figure
// against the pre-delivery gauge sets aside roughly twice the propellant the
// burn will use, which is enough to price a reachable rendezvous out of reach.
FUNCTION rdvHeavyEquivalentOf {
  PARAMETER dvLight.
  IF NOT DEPLOY_BEFORE_DEORBIT OR payloadMass <= 1 { RETURN dvLight. }
  LOCAL vEx IS RKT_ISP * G0.
  LOCAL mHeavy IS SHIP:MASS.
  LOCAL mLight IS mHeavy - payloadMass.
  IF mLight <= 0 OR vEx <= 0 { RETURN dvLight. }
  LOCAL prop IS mLight * (1 - CONSTANT:E ^ (-dvLight / vEx)).
  IF prop <= 0 OR mHeavy <= prop { RETURN dvLight. }
  RETURN MIN(dvLight, vEx * LN(mHeavy / (mHeavy - prop))).
}

// ---------------------------------------------------------------------------
//  Orbital mechanics
// ---------------------------------------------------------------------------
FUNCTION vCircAtR {                 // circular speed at a radius from the centre
  PARAMETER rr.
  IF rr <= 0 { RETURN 0. }
  RETURN SQRT(BODY_MU / rr).
}

FUNCTION orbPeriodAtR {             // period of a circular orbit of radius rr
  PARAMETER rr.
  IF rr <= 0 { RETURN 0. }
  RETURN 2 * CONSTANT:PI * SQRT(rr ^ 3 / BODY_MU).
}

// Total dV of a two-burn Hohmann transfer between two circular radii.
FUNCTION hohmannDv {
  PARAMETER rFrom, rTo.
  IF rFrom <= 0 OR rTo <= 0 { RETURN 0. }
  IF ABS(rTo - rFrom) < 1 { RETURN 0. }
  LOCAL smaT IS (rFrom + rTo) / 2.
  LOCAL vA IS SQRT(BODY_MU * (2 / rFrom - 1 / smaT)).
  LOCAL vB IS SQRT(BODY_MU * (2 / rTo - 1 / smaT)).
  RETURN ABS(vA - vCircAtR(rFrom)) + ABS(vCircAtR(rTo) - vB).
}

FUNCTION hohmannTime {              // half the period of the transfer ellipse
  PARAMETER rFrom, rTo.
  LOCAL smaT IS (rFrom + rTo) / 2.
  IF smaT <= 0 { RETURN 0. }
  RETURN CONSTANT:PI * SQRT(smaT ^ 3 / BODY_MU).
}

// dV to drop periapsis to DEORBIT_PE from a circular orbit of radius rr.
FUNCTION deorbitDvFromR {
  PARAMETER rr.
  LOCAL rP IS BODY_R + DEORBIT_PE.
  IF rP >= rr { RETURN 0. }
  LOCAL smaT IS (rr + rP) / 2.
  RETURN MAX(0, vCircAtR(rr) - SQRT(BODY_MU * (2 / rr - 1 / smaT))).
}

// What must still be in the tanks after the rendezvous is over.
FUNCTION reserveDvAtR {
  PARAMETER rr.
  RETURN MAX(DV_GLIDE_RESERVE, rdvHeavyEquivalentOf(deorbitDvFromR(rr) + DV_MARGIN)).
}

// ---------------------------------------------------------------------------
//  Prediction helpers
//
//  kOS reports positions relative to the *current* ship, and the body itself is
//  moving through that frame as it orbits its own parent, so a future position
//  only means something once the body's own future position is taken out of it.
//  Every geometric quantity below is built on this one function.
//
//  POSITIONAT on the active vessel assumes any planned maneuver nodes are
//  executed exactly as planned, which is what makes the node refinement below
//  possible.  For every *other* vessel it assumes pure drift - fine for a
//  station, which is exactly what a station does.
// ---------------------------------------------------------------------------
FUNCTION relPosAt {
  PARAMETER orbtbl, tUT.
  RETURN POSITIONAT(orbtbl, tUT) - POSITIONAT(SHIP:BODY, tUT).
}

FUNCTION orbVelAt {
  PARAMETER orbtbl, tUT.
  RETURN VELOCITYAT(orbtbl, tUT):ORBIT.
}

// Distance between the two ships at a future time.  The body's motion cancels,
// so this one does not need relPosAt.
FUNCTION missAt {
  PARAMETER tUT.
  RETURN (POSITIONAT(SHIP, tUT) - POSITIONAT(tgtVes, tUT)):MAG.
}

// Relative speed at a future time - what the arrival burn will have to pay to
// stop.  Like missAt this reads through any planned node, so it prices the
// intercept a candidate transfer would actually deliver rather than the one
// the closed-form plan imagined.  It is the number the refinement below was
// missing: a hill-climb that only minimises the miss will gladly trade a slow
// tangential arrival for a fast steep one, and the ship pays for that trade at
// the far end, in the burn there is least propellant left for.
FUNCTION relVelAt {
  PARAMETER tUT.
  RETURN (orbVelAt(SHIP, tUT) - orbVelAt(tgtVes, tUT)):MAG.
}

// Closest approach inside a window: a coarse sweep for the basin, then a
// ternary search inside it.  Returned as a lexicon so the caller gets both the
// time and the distance without a global to trip over.
FUNCTION closestApproach {
  PARAMETER tFrom, tTo.
  IF tTo <= tFrom { RETURN LEXICON("t", tFrom, "d", missAt(tFrom)). }
  LOCAL dt    IS (tTo - tFrom) / CA_COARSE.
  LOCAL bestT IS tFrom.
  LOCAL bestD IS missAt(tFrom).
  FROM { LOCAL idx IS 1. } UNTIL idx > CA_COARSE STEP { SET idx TO idx + 1. } DO {
    LOCAL tt IS tFrom + idx * dt.
    LOCAL dd IS missAt(tt).
    IF dd < bestD { SET bestD TO dd. SET bestT TO tt. }
  }
  LOCAL lo IS MAX(tFrom, bestT - dt).
  LOCAL hi IS MIN(tTo, bestT + dt).
  FROM { LOCAL idx IS 0. } UNTIL idx >= CA_REFINE STEP { SET idx TO idx + 1. } DO {
    LOCAL m1 IS lo + (hi - lo) / 3.
    LOCAL m2 IS hi - (hi - lo) / 3.
    IF missAt(m1) < missAt(m2) { SET hi TO m2. } ELSE { SET lo TO m1. }
  }
  LOCAL tBest IS (lo + hi) / 2.
  RETURN LEXICON("t", tBest, "d", missAt(tBest)).
}

// Signed phase angle at a future time: how far the target is *ahead* of us,
// measured in our own orbital plane and in our own direction of travel.  This
// is the one number the whole phasing problem is about.
FUNCTION phaseAngleAt {
  PARAMETER tUT.
  LOCAL rS IS relPosAt(SHIP, tUT).
  LOCAL rT IS relPosAt(tgtVes, tUT).
  IF rS:MAG < 1 OR rT:MAG < 1 { RETURN 0. }
  LOCAL hS  IS VCRS(rS, orbVelAt(SHIP, tUT)).       // our orbit normal
  LOCAL rTp IS VXCL(hS, rT).                        // target, flattened into our plane
  IF rTp:MAG < 1 OR hS:MAG < 1 { RETURN 0. }
  LOCAL ang IS VANG(rS, rTp).
  // r x v points along h, so rotating from us toward the target the same way we
  // are travelling produces a cross product that agrees with h.  That is what
  // "ahead" means, and it is why this needs no knowledge of kOS's frame.
  IF VDOT(VCRS(rS, rTp), hS) < 0 { RETURN -ang. }
  RETURN ang.
}

// Relative inclination: the angle between the two orbit normals.  Frame-free,
// which matters - the alternative is subtracting inclinations and longitudes of
// ascending node, which is only right when both are measured the same way.
FUNCTION relIncDeg {
  LOCAL hS IS VCRS(relPosAt(SHIP, TIME:SECONDS), SHIP:VELOCITY:ORBIT).
  LOCAL hT IS VCRS(relPosAt(tgtVes, TIME:SECONDS), tgtVes:VELOCITY:ORBIT).
  IF hS:MAG < 1 OR hT:MAG < 1 { RETURN 0. }
  RETURN VANG(hS, hT).
}

// The relative node line: where the two orbital planes cross.  Both normals are
// perpendicular to it, so their cross product is it.
FUNCTION relNodeVec {
  LOCAL hS IS VCRS(relPosAt(SHIP, TIME:SECONDS), SHIP:VELOCITY:ORBIT).
  LOCAL hT IS VCRS(relPosAt(tgtVes, TIME:SECONDS), tgtVes:VELOCITY:ORBIT).
  RETURN VCRS(hS, hT).
}

// The velocity change that rotates our orbital plane onto the target's, as a
// vector, computed live from where the ship is right now.
//
// This deliberately does NOT go through a maneuver node's normal component.
// KSP's coordinate system is left-handed and the sign convention for a node's
// normal axis is exactly the kind of thing that is right on one machine and
// mirrored on the next; a plane change that comes out backwards doubles the
// relative inclination instead of removing it, at full price.  Rotating the
// velocity vector about the radius vector is the same manoeuvre expressed in
// terms this script can check for itself: try it both ways round, keep the one
// that actually points our orbit normal at the target's.
//
// Only valid *at* the relative node, where both planes contain our radius
// vector - which is where phase 1 arranges to be.
FUNCTION planeBurnVec {
  LOCAL rVec IS relPosAt(SHIP, TIME:SECONDS).
  LOCAL vVec IS SHIP:VELOCITY:ORBIT.
  IF rVec:MAG < 1 OR vVec:MAG < 1 { RETURN V(0, 0, 0). }
  LOCAL hT IS VCRS(relPosAt(tgtVes, TIME:SECONDS), tgtVes:VELOCITY:ORBIT).
  IF hT:MAG < 1 { RETURN V(0, 0, 0). }
  LOCAL rHat IS rVec:NORMALIZED.
  LOCAL diNow IS relIncDeg().
  LOCAL vA IS ANGLEAXIS(diNow, rHat) * vVec.
  LOCAL vB IS ANGLEAXIS(-diNow, rHat) * vVec.
  IF VANG(VCRS(rVec, vA), hT) <= VANG(VCRS(rVec, vB), hT) { RETURN vA - vVec. }
  RETURN vB - vVec.
}

// Angle from our predicted position to the node line, signed, so a crossing
// shows up as a sign change and can be bisected.
FUNCTION nodeAngleAt {
  PARAMETER tUT, nodeVec.
  LOCAL rS IS relPosAt(SHIP, tUT).
  IF rS:MAG < 1 OR nodeVec:MAG < 1 { RETURN 0. }
  LOCAL hS  IS VCRS(rS, orbVelAt(SHIP, tUT)).
  LOCAL nvP IS VXCL(hS, nodeVec).
  IF nvP:MAG < 1 { RETURN 0. }
  LOCAL ang IS VANG(rS, nvP).
  IF VDOT(VCRS(rS, nvP), hS) < 0 { RETURN -ang. }
  RETURN ang.
}

// First time at or after tFrom when we cross the node line.  Same coarse-scan
// and bisect shape deorbit_land.ks uses to find its deorbit point; the wrap
// guard rejects the +-180 discontinuity, which is a sign change but not a
// crossing.  Returns 0 if the span holds none.
FUNCTION nodeCrossAfter {
  PARAMETER tFrom, nodeVec.
  LOCAL span IS SHIP:OBT:PERIOD * PLANE_SCAN_ORBITS.
  LOCAL tA IS tFrom.
  LOCAL fa IS nodeAngleAt(tA, nodeVec).
  LOCAL tB IS tFrom.
  UNTIL tB >= tFrom + span {
    SET tB TO MIN(tB + PLANE_SCAN_STEP, tFrom + span).
    LOCAL fb IS nodeAngleAt(tB, nodeVec).
    IF fa * fb <= 0 AND ABS(fa - fb) < 90 {
      LOCAL lo IS tA. LOCAL hi IS tB. LOCAL flo IS fa.
      FROM { LOCAL idx IS 0. } UNTIL idx >= 25 STEP { SET idx TO idx + 1. } DO {
        LOCAL mid IS (lo + hi) / 2.
        LOCAL fm  IS nodeAngleAt(mid, nodeVec).
        IF flo * fm <= 0 { SET hi TO mid. } ELSE { SET lo TO mid. SET flo TO fm. }
      }
      RETURN (lo + hi) / 2.
    }
    SET tA TO tB. SET fa TO fb.
  }
  RETURN 0.
}

// How long until a lead angle of phiNow becomes phiWant, drifting at rateDeg
// degrees per second.  Negative means it never will.
FUNCTION phaseWaitFor {
  PARAMETER phiNow, phiWant, rateDeg.
  IF ABS(rateDeg) < 0.0000001 { RETURN -1. }
  LOCAL dAng IS rdvNormAng(phiWant - phiNow).
  IF rateDeg > 0 {
    IF dAng < 0 { SET dAng TO dAng + 360. }
  } ELSE {
    IF dAng > 0 { SET dAng TO dAng - 360. }
  }
  RETURN dAng / rateDeg.
}

// ---------------------------------------------------------------------------
//  Burn execution
// ---------------------------------------------------------------------------
//  Every burn in this script goes through here.  It aligns, warps, burns, and -
//  the part that matters - refuses to spend the reserve.  The node is flown to
//  the point where its own dV vector reverses, which is the only overshoot test
//  that cannot be argued with: a node whose burn vector has flipped is a node
//  that has been overflown.
// ---------------------------------------------------------------------------
FUNCTION execNode {
  PARAMETER ndIn, label, floorDv.
  LOCAL dvWant IS ndIn:DELTAV:MAG.
  IF dvWant < 0.05 {
    PRINT "  " + label + ": nothing to burn (" + ROUND(dvWant, 2) + " m/s).".
    RETURN TRUE.
  }
  rdvGoClosedCycle().
  LOCAL burnDur IS rdvBurnTimeFor(dvWant).
  IF burnDur <= 0 { SET burnDur TO 30. }

  PRINT "  " + label + ": " + ROUND(dvWant, 1) + " m/s, " +
        ROUND(burnDur, 1) + " s burn, T-" +
        ROUND(ndIn:ETA / 60, 1) + " min.".

  // Say so, but fly it.  The pre-flight check has already given its verdict and
  // the pilot has already chosen to go; refusing to start a burn here would
  // contradict that, and the in-burn floor below stops it at exactly the right
  // moment anyway.  ABORT_IF_INFEASIBLE is where "do not even start" lives.
  IF rdvRocketDv() < dvWant + floorDv {
    PRINT "  !! " + label + " cannot be flown in full - " +
          ROUND(rdvRocketDv()) + " m/s aboard, " + ROUND(dvWant) + " wanted with " +
          ROUND(floorDv) + " reserved. It will stop on the reserve.".
  }

  SAS OFF.
  LOCK STEERING TO ndIn:BURNVECTOR.

  LOCAL tStart IS TIME:SECONDS + ndIn:ETA - burnDur / 2.
  IF USE_WARP AND tStart - WARP_LEAD > TIME:SECONDS + 5 {
    // Settle on the burn attitude *before* going on rails: warp freezes the
    // ship's orientation in inertial space, so anything not already pointed the
    // right way arrives pointed the wrong way.
    LOCAL tSettle IS MIN(TIME:SECONDS + 90, tStart - WARP_LEAD).
    WAIT UNTIL VANG(SHIP:FACING:VECTOR, ndIn:BURNVECTOR) < ALIGN_TOL
            OR TIME:SECONDS > tSettle.
    WARPTO(tStart - WARP_LEAD).
    WAIT UNTIL TIME:SECONDS >= tStart - WARP_LEAD.
    SET WARP TO 0.
    WAIT UNTIL KUNIVERSE:TIMEWARP:ISSETTLED.
  }

  LOCAL tGiveUp IS TIME:SECONDS + 120.
  WAIT UNTIL VANG(SHIP:FACING:VECTOR, ndIn:BURNVECTOR) < ALIGN_TOL
          OR TIME:SECONDS > tGiveUp
          OR TIME:SECONDS >= tStart.
  WAIT UNTIL TIME:SECONDS >= tStart.

  LOCAL dv0     IS ndIn:DELTAV.
  LOCAL burning IS TRUE.
  LOCAL stopped IS "done".
  // The throttle is locked once, to a global the loop writes.  Locking it to an
  // expression over a loop-local would leave the lock re-evaluating a variable
  // that has gone out of scope - which is how a throttle ends up frozen at
  // whatever it happened to hold on the last tick of the loop.
  SET thrCmd TO 1.
  LOCK THROTTLE TO thrCmd.
  UNTIL NOT burning {
    LOCAL left IS ndIn:DELTAV:MAG.
    // Taper, so a burn that is nearly finished does not step past the target in
    // a single physics tick at full thrust.
    IF left < 15 { SET thrCmd TO rdvClampVal(left / 15, 0.05, 1). }
    IF left < 0.15 { SET burning TO FALSE. }
    IF VDOT(dv0, ndIn:DELTAV) < 0 { SET burning TO FALSE. }
    IF rdvRocketDv() <= floorDv {
      SET burning TO FALSE.
      SET stopped TO "reserve".
    }
    IF SHIP:AVAILABLETHRUST <= 0 {
      SET burning TO FALSE.
      SET stopped TO "dry".
    }
    WAIT 0.02.
  }
  SET thrCmd TO 0.
  LOCK THROTTLE TO 0.
  UNLOCK STEERING.

  IF stopped = "reserve" {
    PRINT "  !! " + label + " stopped on the reserve with " +
          ROUND(ndIn:DELTAV:MAG, 1) + " m/s unflown.".
    RETURN FALSE.
  }
  IF stopped = "dry" {
    PRINT "  !! " + label + " stopped - no thrust available.".
    RETURN FALSE.
  }
  PRINT "  " + label + " complete (" + ROUND(ndIn:DELTAV:MAG, 2) + " m/s residual).".
  RETURN TRUE.
}

// Where the nose points during the terminal approach.  Normally at the target,
// because that is the attitude a docking script wants to inherit and it keeps
// the engine bell pointed away from the station.  But if the ship is closing
// faster than its RCS can stop it, pointing at the target is exactly wrong:
// the nose has to come round to the braking vector so the main engine can be
// used, and getting there is worth more than looking tidy.
FUNCTION approachSteer {
  IF brakeHard {
    LOCAL vBrake IS tgtVes:VELOCITY:ORBIT - SHIP:VELOCITY:ORBIT.
    IF vBrake:MAG > 0.5 { RETURN vBrake. }
  }
  RETURN tgtVes:POSITION.
}

FUNCTION clearNodes {
  UNTIL NOT HASNODE { REMOVE NEXTNODE. WAIT 0. }
}

// Does this install actually fold planned nodes into its predictions?  The
// refinement below is worthless if it does not, and silently worthless is the
// worst kind.  Plant a throwaway node, see whether the predicted position ten
// minutes out moves, and take the node away again.
FUNCTION nodePredictionWorks {
  LOCAL tProbe IS TIME:SECONDS + 600.
  LOCAL before IS relPosAt(SHIP, tProbe).
  LOCAL ndProbe IS NODE(TIME:SECONDS + 60, 0, 0, 25).
  ADD ndProbe.
  WAIT 0.
  LOCAL after IS relPosAt(SHIP, tProbe).
  REMOVE ndProbe.
  WAIT 0.
  RETURN (after - before):MAG > 100.
}

// ---------------------------------------------------------------------------
//  Reporting
// ---------------------------------------------------------------------------
SET logStarted TO FALSE.

FUNCTION rdvResourceReport {
  PARAMETER label.
  LOCAL lfAmt   IS rdvCoreResAmt("LiquidFuel").
  LOCAL oxAmt   IS rdvCoreResAmt("Oxidizer").
  LOCAL monoAmt IS rdvResAmtShip("MonoPropellant").
  LOCAL ecAmt   IS rdvResAmtShip("ElectricCharge").
  PRINT "------------------------------------------------------".
  PRINT "RESOURCES :: " + label.
  PRINT "  Mass  : " + ROUND(SHIP:MASS, 2) + " t" +
        "   (payload " + ROUND(payloadMass, 1) + " t)".
  PRINT "  LF/Ox : " + ROUND(lfAmt, 1) + " / " + ROUND(oxAmt, 1) +
        " u   ->  " + ROUND(rdvRocketDv()) + " m/s".
  PRINT "  Mono  : " + ROUND(monoAmt, 1) + " u   ->  " + ROUND(rdvMonoDv(), 1) +
        " m/s on RCS".
  PRINT "  EC    : " + ROUND(ecAmt, 1) + " u".
  IF PAY_TANKS:LENGTH > 0 {
    PRINT "  Payload LF/Ox (not counted): " +
          ROUND(rdvListResAmt(PAY_TANKS, "LiquidFuel"), 1) + " / " +
          ROUND(rdvListResAmt(PAY_TANKS, "Oxidizer"), 1) + " u".
  }
  PRINT "------------------------------------------------------".

  IF LOG_TO_FILE {
    IF NOT logStarted {
      LOG "ut,label,mass_t,lf_u,ox_u,mono_u,ec_u,dv_ms,mono_dv_ms" TO LOG_PATH.
      SET logStarted TO TRUE.
    }
    LOG ROUND(TIME:SECONDS, 1) + "," + label + "," + ROUND(SHIP:MASS, 3) + "," +
        ROUND(lfAmt, 2) + "," + ROUND(oxAmt, 2) + "," + ROUND(monoAmt, 2) + "," +
        ROUND(ecAmt, 2) + "," + ROUND(rdvRocketDv(), 1) + "," +
        ROUND(rdvMonoDv(), 1) TO LOG_PATH.
  }
}

FUNCTION rdvHandBack {
  LOCK THROTTLE TO 0.
  UNLOCK THROTTLE.
  UNLOCK STEERING.
  SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
  SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
  SET CONFIG:IPU TO IPU_SAVED.
  SAS ON.
}

// ---------------------------------------------------------------------------
//  0. PRE-FLIGHT  --  acquire the target, measure the ship, price the job
// ---------------------------------------------------------------------------
PRINT "=== RAPIER SSTO :: rendezvous autopilot ===".
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
SET thrCmd TO 0.                    // every burn loop writes this; the throttle
                                    // is LOCKed to it rather than to the loop's
                                    // own locals
SET brakeHard TO FALSE.             // set when the approach needs main-engine
                                    // braking - see approachSteer()
SET abortWhy  TO "".                // why the flight stopped short, if it did,
                                    // so the closing report can say it rather
                                    // than leave the pilot to guess from the
                                    // last line that happened to print
LOCK THROTTLE TO 0.
SAS OFF.

// --- 0a. Which vessel are we meeting? ---------------------------------------
SET tgtVes   TO SHIP.               // replaced below; typed placeholder only
SET tgtFound TO FALSE.

IF targetName = "" {
  IF HASTARGET {
    SET tgtVes TO TARGET.
    SET tgtFound TO TRUE.
    PRINT "Target taken from the map selection: " + tgtVes:NAME.
  } ELSE {
    PRINT "!! No target. Select one on the map, or name it:".
    // kOS has no escape character in string literals: a backslash is just a
    // backslash, so \" ends the string early and the file fails to compile.
    // A literal double quote is written by doubling it.
    PRINT "   RUN rendezvous(""Station Alpha"").".
    rdvHandBack().
    PRINT "Nothing to rendezvous with - stopping.".
  }
} ELSE {
  SET vlist TO LIST().
  LIST TARGETS IN vlist.
  SET want    TO targetName:TOLOWER.
  SET exact   TO LIST().
  SET partial TO LIST().
  FOR ves IN vlist {
    LOCAL nm IS ves:NAME:TOLOWER.
    IF nm = want { exact:ADD(ves). }
    ELSE IF nm:CONTAINS(want) { partial:ADD(ves). }
  }
  IF exact:LENGTH >= 1 {
    SET tgtVes TO exact[0].
    SET tgtFound TO TRUE.
  } ELSE IF partial:LENGTH = 1 {
    SET tgtVes TO partial[0].
    SET tgtFound TO TRUE.
    PRINT "Matched """ + targetName + """ to " + tgtVes:NAME + ".".
  } ELSE IF partial:LENGTH > 1 {
    // Guessing between two stations is how a ship ends up rendezvousing with
    // the wrong one after a two-hour phasing wait.  Say what was found instead.
    PRINT "!! """ + targetName + """ is ambiguous - " + partial:LENGTH + " matches:".
    FOR ves IN partial { PRINT "     " + ves:NAME. }
    PRINT "   Give more of the name, or select it on the map and RUN rendezvous.".
  } ELSE {
    PRINT "!! No vessel matching """ + targetName + """ is in flight.".
  }
  IF NOT tgtFound { rdvHandBack(). }
}

IF tgtFound {
  SET TARGET TO tgtVes.             // so the map, the navball and the docking
                                    // script that follows all agree with us

  // --- 0b. Is this rendezvous even the kind of problem this script solves? ---
  SET tgtOk TO TRUE.
  IF tgtVes:BODY:NAME <> SHIP:BODY:NAME {
    PRINT "!! " + tgtVes:NAME + " orbits " + tgtVes:BODY:NAME + ", not " +
          SHIP:BODY:NAME + ".".
    PRINT "   This script matches orbits around one body; an interplanetary".
    PRINT "   transfer is a different problem. Stopping.".
    SET tgtOk TO FALSE.
  } ELSE IF tgtVes:OBT:ECCENTRICITY >= 1 {
    PRINT "!! " + tgtVes:NAME + " is on an escape trajectory, not an orbit.".
    SET tgtOk TO FALSE.
  } ELSE IF SHIP:PERIAPSIS < ATM_TOP {
    PRINT "!! Our periapsis is " + ROUND(SHIP:PERIAPSIS / 1000, 1) +
          " km, inside the atmosphere.".
    PRINT "   This is a suborbital arc - circularise before rendezvousing.".
    SET tgtOk TO FALSE.
  }

  IF tgtOk {
    SET RKT_ISP TO rdvMeasureRocketIsp().
    SET rcsAccEst TO rdvRcsAccelNominal().
    rdvGoClosedCycle().

    IF ISOLATE_PAYLOAD AND PAY_TANKS:LENGTH > 0 {
      SET blockedN TO rdvBlockCrossfeed().
      IF blockedN > 0 {
        PRINT "Payload isolated: crossfeed blocked on " + blockedN + " separator(s).".
      } ELSE IF FORCE_TANK_LOCK {
        PRINT "Payload isolated: flow disabled on " + rdvLockPayloadTanks() +
              " payload tank(s) - RE-ENABLE THEM ON DEPLOY.".
      }
    }

    SET rShipNow TO BODY_R + SHIP:ALTITUDE.
    SET rTgt     TO tgtVes:OBT:SEMIMAJORAXIS.
    SET rangeNow TO (tgtVes:POSITION):MAG.

    PRINT "------------------------------------------------------".
    PRINT "TARGET :: " + tgtVes:NAME.
    PRINT "  Orbit    : " + ROUND(tgtVes:OBT:APOAPSIS / 1000, 1) + " x " +
          ROUND(tgtVes:OBT:PERIAPSIS / 1000, 1) + " km, inc " +
          ROUND(tgtVes:OBT:INCLINATION, 2) + " deg".
    PRINT "  Ours     : " + ROUND(SHIP:APOAPSIS / 1000, 1) + " x " +
          ROUND(SHIP:PERIAPSIS / 1000, 1) + " km, inc " +
          ROUND(SHIP:ORBIT:INCLINATION, 2) + " deg".
    PRINT "  Range    : " + ROUND(rangeNow / 1000, 1) + " km".
    PRINT "  Rel inc  : " + ROUND(relIncDeg(), 3) + " deg".
    PRINT "  Phase    : " + ROUND(phaseAngleAt(TIME:SECONDS), 1) +
          " deg (positive = target ahead)".

    // The resource log you asked for, taken before a single drop is spent.
    rdvResourceReport("before rendezvous").

    // --- 0c. Plane change ---------------------------------------------------
    SET diRel     TO relIncDeg().
    SET planePlan TO diRel > PLANE_SKIP_DI.
    SET planeDv   TO 0.
    IF diRel > 90 {
      PRINT "!! Relative inclination is " + ROUND(diRel, 1) +
            " deg - the target orbits the other way round.".
      PRINT "   Matching it costs more than a launch. Stopping.".
      SET tgtOk TO FALSE.
    } ELSE IF planePlan {
      // Cheapest at the slowest point, which on a near-circular orbit is the
      // higher of the two node crossings.  2*v*sin(di/2) is exact for a pure
      // rotation of the velocity vector.
      SET planeDv TO 2 * vCircAtR(rShipNow) * SIN(diRel / 2).
    }
  }

  IF tgtOk {
    // --- 0d. Phasing solver -------------------------------------------------
    //  Sweep candidate phasing radii and price each one: two burns to get in
    //  and out, against the wait it produces.  Our current radius is in the
    //  candidate list, so "just wait where we are" is one of the options and
    //  wins whenever it is affordable in time - which, against a station in a
    //  different orbit, it usually is.  Against one in *our* orbit the phase
    //  rate is zero, the wait is infinite, and the sweep is the only thing
    //  standing between the ship and a rendezvous that never happens.
    //
    //  The sweep also asks, of every candidate, whether the tanks can pay for
    //  it - and that is a search constraint, not a verdict to print afterwards.
    //  MAX_TOTAL_TIME is a preference; propellant is a fact.  A plan that fits
    //  the clock and not the tanks is not a plan, so when the cheapest thing
    //  inside the time limit cannot be afforded the sweep looks further out
    //  instead, to MAX_TIME_PATIENT, and takes the soonest one that can be.
    SET omegaT TO 360 / tgtVes:OBT:PERIOD.        // deg/s, target
    SET phiNow TO phaseAngleAt(TIME:SECONDS).

    // Priced here so the sweep can weigh each candidate against them.  The
    // reserve is what must survive the whole rendezvous; the two allowances
    // are the corrections and the arrival burn, neither of which the closed-
    // form transfer dV covers.
    SET reserveDv TO reserveDvAtR(rTgt).
    SET haveDv    TO rdvRocketDv().
    SET fixedDv   TO planeDv + CORR_DV_ALLOW + ARRIVE_DV_ALLOW + reserveDv.

    SET bestR  TO 0.
    SET bestDv TO 0.
    SET bestT  TO 0.
    SET bestWait TO 0.
    SET bestFound TO FALSE.
    SET bestAfford TO FALSE.
    SET patR TO 0.
    SET patDv TO 0.
    SET patT TO 0.
    SET patWait TO 0.
    SET patFound TO FALSE.
    SET anyR TO 0.
    SET anyT TO 0.
    SET anyDv TO 0.
    SET anyWait TO 0.
    SET anyFound TO FALSE.

    SET rLo TO MAX(R_FLOOR, rTgt * (1 - PHASE_R_SPAN)).
    SET rHi TO MIN(SOI_R * 0.8, rTgt * (1 + PHASE_R_SPAN)).
    SET cands TO LIST().
    cands:ADD(rShipNow).
    FROM { LOCAL idx IS 0. } UNTIL idx > PHASE_GRID STEP { SET idx TO idx + 1. } DO {
      cands:ADD(rLo + (rHi - rLo) * idx / PHASE_GRID).
    }

    FOR rP IN cands {
      IF rP > R_FLOOR AND rP < SOI_R * 0.8 {
        LOCAL sameOrbit IS ABS(rP - rShipNow) < 1000.
        LOCAL dvMove IS 0.
        LOCAL tMove  IS 0.
        IF NOT sameOrbit {
          SET dvMove TO hohmannDv(rShipNow, rP).
          SET tMove  TO hohmannTime(rShipNow, rP).
        }
        // Crossing to the phasing orbit sweeps exactly 180 degrees of our own
        // travel while the target keeps turning at its own rate, so the lead
        // angle we arrive with is not the one we have now.
        LOCAL phiArr IS phiNow + omegaT * tMove - 180.
        IF sameOrbit { SET phiArr TO phiNow. }

        LOCAL tTrans IS hohmannTime(rP, rTgt).
        LOCAL phiReq IS rdvNormAng(180 - omegaT * tTrans).
        LOCAL omegaP IS 360 / orbPeriodAtR(rP).
        LOCAL waitT  IS phaseWaitFor(phiArr, phiReq, omegaT - omegaP).

        IF waitT >= 0 {
          LOCAL dvAll IS dvMove + hohmannDv(rP, rTgt).
          LOCAL tAll  IS tMove + waitT + tTrans.
          LOCAL canPay IS (dvAll + fixedDv) <= haveDv.
          IF (NOT anyFound) OR tAll < anyT {
            SET anyFound TO TRUE. SET anyR TO rP. SET anyT TO tAll.
            SET anyDv TO dvAll. SET anyWait TO waitT.
          }
          IF tAll <= MAX_TOTAL_TIME {
            IF (NOT bestFound) OR dvAll < bestDv {
              SET bestFound TO TRUE.
              SET bestR TO rP.
              SET bestDv TO dvAll.
              SET bestT TO tAll.
              SET bestWait TO waitT.
              SET bestAfford TO canPay.
            }
          }
          // The patient pool: anything the tanks can actually pay for, out to
          // the longer horizon, ranked by how soon it parks us.  Waiting costs
          // nothing that cannot be warped through.
          IF canPay AND tAll <= MAX_TIME_PATIENT {
            IF (NOT patFound) OR tAll < patT {
              SET patFound TO TRUE.
              SET patR TO rP.
              SET patDv TO dvAll.
              SET patT TO tAll.
              SET patWait TO waitT.
            }
          }
        }
      }
    }

    // The cheapest plan inside the time limit is the one we want, but only if
    // it can be flown to the end.  If it cannot and a slower one can, take the
    // slower one: this script used to print "raise MAX_TOTAL_TIME and re-run"
    // at exactly this moment, which is advice a pilot cannot act on without
    // throwing away the orbit they are in.  Do it here instead.
    IF rangeNow >= NEAR_ALREADY AND patFound AND
       ((NOT bestFound) OR (NOT bestAfford)) {
      IF bestFound {
        PRINT "!! The quickest plan (" + ROUND(bestT / 60) + " min, " +
              ROUND(bestDv + fixedDv) + " m/s all in) costs more than the " +
              ROUND(haveDv) + " m/s aboard.".
      } ELSE {
        PRINT "!! Nothing fits inside " + ROUND(MAX_TOTAL_TIME / 3600, 1) +
              " h that the tanks can also pay for.".
      }
      PRINT "   Waiting longer instead: " + ROUND(patT / 60) +
            " min, " + ROUND(patDv + fixedDv) + " m/s all in.".
      SET bestFound TO TRUE.
      SET bestAfford TO TRUE.
      SET bestR TO patR.
      SET bestDv TO patDv.
      SET bestT TO patT.
      SET bestWait TO patWait.
    }

    IF NOT bestFound AND anyFound {
      // Nothing fits the time budget.  Say so and fly the quickest thing there
      // is rather than refusing - the pilot can always warp away from a long
      // wait, but cannot conjure a plan the script declined to make.
      PRINT "!! No rendezvous fits inside " + ROUND(MAX_TOTAL_TIME / 3600, 1) +
            " h. The quickest is " + ROUND(anyT / 3600, 1) + " h.".
      SET bestFound TO TRUE.
      SET bestR TO anyR.
      SET bestDv TO anyDv.
      SET bestT TO anyT.
      SET bestWait TO anyWait.
    }

    // Belt and braces.  Every candidate having been rejected means the sweep
    // found no orbit with a phase rate at all, which needs a target orbit the
    // grid could not straddle.  bestR would still be zero, and phase 3 would
    // cheerfully read that as "transfer to the centre of the planet" - so fall
    // back to staying put and let the live phase-angle wait sort it out.
    IF NOT bestFound OR bestR < R_FLOOR {
      PRINT "!! The phasing sweep found no usable orbit - staying in this one".
      PRINT "   and departing on whatever geometry comes round.".
      SET bestR TO rShipNow.
      SET bestDv TO hohmannDv(rShipNow, rTgt).
      SET bestWait TO 0.
      SET bestT TO hohmannTime(rShipNow, rTgt).
    }

    IF rangeNow < NEAR_ALREADY {
      PRINT "Already within " + ROUND(rangeNow / 1000, 1) +
            " km - skipping the transfer and closing directly.".
    }

    // --- 0e. The bill -------------------------------------------------------
    //  ARRIVE_DV_ALLOW is in here because the closed-form transfer dV is not
    //  the whole cost of arriving.  Its second burn prices a tangential
    //  arrival into a circular orbit at the target's radius; the transfer the
    //  ship actually flies has been bent to hit the target where it will be,
    //  and the relative velocity that leaves is larger - sometimes several
    //  times larger.  Leaving that off the bill is how a rendezvous gets a GO
    //  and then runs out of propellant in the one burn that matters.
    SET termDv    TO 2 * V_CLOSE_MAX + 2 * ARRIVE_V_TOL.
    SET reserveDv TO reserveDvAtR(rTgt).
    SET billDv    TO planeDv + bestDv + CORR_DV_ALLOW + ARRIVE_DV_ALLOW.
    IF rangeNow < NEAR_ALREADY { SET billDv TO 0. }
    SET needDv    TO billDv + reserveDv.
    SET haveDv    TO rdvRocketDv().

    PRINT "======================================================".
    PRINT "RENDEZVOUS BUDGET".
    IF planePlan {
      PRINT "  Plane change  : " + ROUND(planeDv, 1) + " m/s  (" +
            ROUND(diRel, 3) + " deg of relative inclination)".
    } ELSE {
      PRINT "  Plane change  : none needed (" + ROUND(diRel, 3) +
            " deg, under the " + ROUND(PLANE_SKIP_DI, 2) + " deg threshold)".
    }
    IF rangeNow >= NEAR_ALREADY {
      IF ABS(bestR - rShipNow) < 1000 {
        PRINT "  Phasing       : none - waiting in this orbit for " +
              ROUND(bestWait / 60) + " min".
      } ELSE {
        PRINT "  Phasing orbit : " + ROUND((bestR - BODY_R) / 1000, 1) +
              " km (we are at " + ROUND((rShipNow - BODY_R) / 1000, 1) +
              " km), wait " + ROUND(bestWait / 60) + " min".
      }
      PRINT "  Transfer      : " + ROUND(bestDv, 1) +
            " m/s  (phasing + Hohmann, both ends)".
      PRINT "  Corrections   : " + ROUND(CORR_DV_ALLOW) + " m/s allowed".
      PRINT "  Arrival       : " + ROUND(ARRIVE_DV_ALLOW) +
            " m/s allowed on top of the transfer's own closing burn".
    }
    PRINT "  Terminal      : " + ROUND(termDv) + " m/s on RCS (" +
          ROUND(termDv * SHIP:MASS / MAX(0.001, RCS_ISP_FALLBACK * G0) /
                MONO_DENS) + " u of mono)".
    PRINT "  Reserve       : " + ROUND(reserveDv) +
          " m/s  (deorbit + margin, priced at the delivered mass)".
    PRINT "  NEED  " + ROUND(needDv) + " m/s     HAVE  " + ROUND(haveDv) + " m/s".
    PRINT "  Time to park  : " + ROUND(bestT / 60) + " min".

    SET feasible TO haveDv >= needDv.
    IF feasible {
      SET spareDv TO haveDv - needDv.
      IF spareDv >= needDv * GO_MARGIN {
        PRINT "  => GO. " + ROUND(spareDv) + " m/s in hand.".
      } ELSE {
        PRINT "  => MARGINAL. Only " + ROUND(spareDv) + " m/s in hand (" +
              ROUND(100 * spareDv / MAX(1, needDv)) + "% of the bill).".
        PRINT "     Corrections and a sloppy intercept spend that. Consider".
        PRINT "     raising MAX_TOTAL_TIME - a more patient plan is a cheaper one.".
      }
    } ELSE {
      PRINT "  => NOT ENOUGH dV - short by " + ROUND(needDv - haveDv) + " m/s.".
      PRINT "     The sweep already looked out to " +
            ROUND(MAX_TIME_PATIENT / 3600, 1) +
            " h for a cheaper plan and found none.".
      PRINT "     * Deliver the payload first if it is going to the station".
      PRINT "       anyway - the reserve is most of this bill at this mass.".
      IF ABS(bestR - rShipNow) >= 1000 {
        PRINT "     * The phasing detour costs " +
              ROUND(hohmannDv(rShipNow, bestR), 1) +
              " m/s of it; a wait fitting in this orbit would avoid it.".
      }
    }
    IF rdvMonoDv() < termDv {
      PRINT "  !! Monopropellant is short for the approach: " +
            ROUND(rdvMonoDv(), 1) + " m/s aboard, " + ROUND(termDv) + " wanted.".
      PRINT "     The terminal phase will stop at MONO_RESERVE and hold.".
    }
    PRINT "======================================================".

    IF NOT feasible AND ABORT_IF_INFEASIBLE {
      PRINT "ABORT_IF_INFEASIBLE is set - staying in this orbit.".
      rdvHandBack().
      SET tgtOk TO FALSE.
    } ELSE IF NOT feasible {
      // Setting off is not the same as committing.  The plane change and the
      // phasing burns all end in a circular orbit the ship can live in, so
      // flying them on a short budget costs propellant and nothing else.  The
      // transfer injection is the one that cannot be taken back, and phase 4c
      // refuses it unless the arrival at the far end is funded - so this is
      // not a licence to fly the whole thing on hope.
      PRINT "Setting off anyway. Every burn stops at the reserve, and the".
      PRINT "transfer will not be committed to unless the arrival is paid for.".
      IF PREFLIGHT_HOLD > 0 { WAIT PREFLIGHT_HOLD. }
    }
  }

  // -------------------------------------------------------------------------
  //  1..7  --  fly it
  // -------------------------------------------------------------------------
  IF tgtOk {
    RCS ON.
    clearNodes().
    SET goOn TO TRUE.
    SET skipTransfer TO rangeNow < NEAR_ALREADY.

    // Ask the one question the numerical targeting depends on, once, while no
    // node is planted and the answer cannot be confused by one.  If this comes
    // back FALSE the closed-form plan still flies; it just flies unrefined.
    SET nodePredOk TO nodePredictionWorks().
    IF NOT nodePredOk {
      PRINT "!! Maneuver nodes do not move this install's predictions.".
      PRINT "   Transfer targeting will be closed-form only.".
    }

    // --- 1. PLANE MATCH -----------------------------------------------------
    IF goOn AND planePlan AND NOT skipTransfer {
      PRINT "1. PLANE MATCH :: " + ROUND(diRel, 3) + " deg to take out.".
      SET nvec TO relNodeVec().
      SET tNode TO nodeCrossAfter(TIME:SECONDS + PHASE_LEAD, nvec).
      IF tNode <= 0 {
        PRINT "  !! No node crossing found in " + PLANE_SCAN_ORBITS +
              " orbits - skipping the plane match.".
        PRINT "     The transfer refinement will absorb what it can.".
      } ELSE {
        //  Flown as a vector burn rather than a node.  The burn is centred on
        //  the node crossing, steered on planeBurnVec() - recomputed every tick,
        //  so it follows the manoeuvre round as the plane rotates under it - and
        //  terminated on the thing it is actually for: the relative inclination
        //  itself, stopped the moment it stops falling.  Burning to a fixed dV
        //  would trust the arithmetic; this checks it.
        rdvGoClosedCycle().
        SET burnDur TO rdvBurnTimeFor(planeDv).
        SET tStart  TO tNode - burnDur / 2.
        PRINT "  " + ROUND(planeDv, 1) + " m/s at the node, " +
              ROUND(burnDur, 1) + " s, T-" +
              ROUND((tNode - TIME:SECONDS) / 60, 1) + " min.".
        SAS OFF.
        LOCK STEERING TO planeBurnVec().
        IF USE_WARP AND tStart - WARP_LEAD > TIME:SECONDS + 5 {
          SET tSettle TO MIN(TIME:SECONDS + 90, tStart - WARP_LEAD).
          WAIT UNTIL VANG(SHIP:FACING:VECTOR, planeBurnVec()) < ALIGN_TOL
                  OR TIME:SECONDS > tSettle.
          WARPTO(tStart - WARP_LEAD).
          WAIT UNTIL TIME:SECONDS >= tStart - WARP_LEAD.
          SET WARP TO 0.
          WAIT UNTIL KUNIVERSE:TIMEWARP:ISSETTLED.
        }
        SET tSettle TO MIN(TIME:SECONDS + 120, tStart).
        WAIT UNTIL VANG(SHIP:FACING:VECTOR, planeBurnVec()) < ALIGN_TOL
                OR TIME:SECONDS > tSettle.
        WAIT UNTIL TIME:SECONDS >= tStart.

        SET diBest TO relIncDeg().
        SET burningPlane TO TRUE.
        SET tPlaneEnd TO TIME:SECONDS + burnDur * 3 + 30.
        SET thrCmd TO 1.
        LOCK THROTTLE TO thrCmd.
        UNTIL NOT burningPlane {
          SET diNow TO relIncDeg().
          IF diNow < diBest { SET diBest TO diNow. }
          IF diNow < PLANE_SKIP_DI { SET burningPlane TO FALSE. }
          // Past the bottom: every further second is spending fuel to build the
          // relative inclination back up on the far side.
          IF diNow > diBest + 0.02 { SET burningPlane TO FALSE. }
          IF diNow < 0.5 { SET thrCmd TO rdvClampVal(diNow / 0.5, 0.05, 1). }
          IF rdvRocketDv() <= reserveDv { SET burningPlane TO FALSE. SET goOn TO FALSE. }
          IF TIME:SECONDS > tPlaneEnd { SET burningPlane TO FALSE. }
          IF SHIP:AVAILABLETHRUST <= 0 { SET burningPlane TO FALSE. SET goOn TO FALSE. }
          WAIT 0.02.
        }
        SET thrCmd TO 0.
        LOCK THROTTLE TO 0.
        UNLOCK STEERING.
        PRINT "  Relative inclination now " + ROUND(relIncDeg(), 3) +
              " deg (was " + ROUND(diRel, 3) + ").".
      }
    }

    // --- 2. TIDY THE ORBIT --------------------------------------------------
    //  The phasing arithmetic treats our orbit as circular.  Ascent leaves it
    //  very nearly so, but "very nearly" is doing real work in a lead angle
    //  computed from a period, so if the ascent settled for something lumpy we
    //  round it off before trusting the numbers.
    IF goOn AND NOT skipTransfer AND SHIP:ORBIT:ECCENTRICITY > ECC_TIDY {
      PRINT "2. TIDY :: eccentricity " + ROUND(SHIP:ORBIT:ECCENTRICITY, 4) +
            " - circularising at apoapsis first.".
      SET tAp  TO TIME:SECONDS + ETA:APOAPSIS.
      SET rAp  TO BODY_R + SHIP:APOAPSIS.
      SET dvC  TO vCircAtR(rAp) - orbVelAt(SHIP, tAp):MAG.
      SET ndTidy TO NODE(tAp, 0, 0, dvC).
      ADD ndTidy.
      IF NOT execNode(ndTidy, "Circularise", reserveDv) { SET goOn TO FALSE. }
      clearNodes().
      SET rShipNow TO BODY_R + SHIP:ALTITUDE.
    }

    // --- 3. PHASING ---------------------------------------------------------
    IF goOn AND NOT skipTransfer {
      IF ABS(bestR - rShipNow) >= 1000 {
        PRINT "3. PHASING :: dropping to " + ROUND((bestR - BODY_R) / 1000, 1) +
              " km to change the phase rate.".
        // Out: raise or lower the far side to the phasing radius...
        SET tB   TO TIME:SECONDS + PHASE_LEAD.
        SET rB   TO BODY_R + SHIP:BODY:ALTITUDEOF(POSITIONAT(SHIP, tB)).
        SET smaT TO (rB + bestR) / 2.
        SET dv1  TO SQRT(BODY_MU * (2 / rB - 1 / smaT)) - orbVelAt(SHIP, tB):MAG.
        SET ndP1 TO NODE(tB, 0, 0, dv1).
        ADD ndP1.
        IF NOT execNode(ndP1, "Phasing transfer", reserveDv) { SET goOn TO FALSE. }
        clearNodes().

        // ...then circularise there, so every point of the phasing orbit is
        // equivalent and the departure can happen the moment the phase is right
        // rather than only at one point per lap.
        IF goOn {
          SET tFar TO TIME:SECONDS + ETA:APOAPSIS.
          IF bestR < rShipNow { SET tFar TO TIME:SECONDS + ETA:PERIAPSIS. }
          SET rFar TO BODY_R + SHIP:BODY:ALTITUDEOF(POSITIONAT(SHIP, tFar)).
          SET dv2  TO vCircAtR(rFar) - orbVelAt(SHIP, tFar):MAG.
          SET ndP2 TO NODE(tFar, 0, 0, dv2).
          ADD ndP2.
          IF NOT execNode(ndP2, "Phasing circularise", reserveDv) { SET goOn TO FALSE. }
          clearNodes().
        }
      } ELSE {
        PRINT "3. PHASING :: waiting in this orbit - no detour needed.".
      }
    }

    // --- 3b. Wait for the departure phase angle -----------------------------
    IF goOn AND NOT skipTransfer {
      SET rPhase TO BODY_R + SHIP:ALTITUDE.
      SET tTrans TO hohmannTime(rPhase, rTgt).
      SET phiReq TO rdvNormAng(180 - omegaT * tTrans).
      SET rateNow TO omegaT - 360 / SHIP:OBT:PERIOD.
      SET waitT TO phaseWaitFor(phaseAngleAt(TIME:SECONDS), phiReq, rateNow).
      IF waitT < 0 {
        PRINT "  !! The phase angle is standing still - the phasing orbit did".
        PRINT "     not take. Closing on what geometry there is.".
        SET waitT TO 0.
      }
      // Stop PHASE_LEAD short of the departure point, because the injection
      // node below is planted that far ahead of wherever the wait ends.  Waiting
      // the full time and *then* adding the lead departs a minute late, which at
      // orbital rates is three or four degrees of phase - a miss the refinement
      // then has to spend dV buying back.
      SET tDepart TO TIME:SECONDS + MAX(0, waitT - PHASE_LEAD).
      PRINT "  Departure phase " + ROUND(phiReq, 1) + " deg, now " +
            ROUND(phaseAngleAt(TIME:SECONDS), 1) + " deg - waiting " +
            ROUND(waitT / 60, 1) + " min.".
      IF USE_WARP AND tDepart - WARP_LEAD > TIME:SECONDS + 5 {
        WARPTO(tDepart - WARP_LEAD).
        WAIT UNTIL TIME:SECONDS >= tDepart - WARP_LEAD OR TIME:SECONDS >= tDepart.
        SET WARP TO 0.
        WAIT UNTIL KUNIVERSE:TIMEWARP:ISSETTLED.
      }
      WAIT UNTIL TIME:SECONDS >= tDepart.
    }

    // --- 4. TRANSFER --------------------------------------------------------
    IF goOn AND NOT skipTransfer {
      PRINT "4. TRANSFER :: injecting.".
      SET tB   TO TIME:SECONDS + PHASE_LEAD.
      SET rB   TO BODY_R + SHIP:BODY:ALTITUDEOF(POSITIONAT(SHIP, tB)).
      SET smaT TO (rB + rTgt) / 2.
      SET dvI  TO SQRT(BODY_MU * (2 / rB - 1 / smaT)) - orbVelAt(SHIP, tB):MAG.
      SET ndX  TO NODE(tB, 0, 0, dvI).
      ADD ndX.
      WAIT 0.

      SET tTrans TO hohmannTime(rB, rTgt).
      SET caNow  TO closestApproach(TIME:SECONDS + PHASE_LEAD,
                                    TIME:SECONDS + PHASE_LEAD + tTrans * 1.6).
      SET arrNow TO relVelAt(caNow["t"]).
      PRINT "  Closed-form plan misses by " + ROUND(caNow["d"] / 1000, 1) +
            " km, arriving at " + ROUND(arrNow) + " m/s.".

      //  What the rest of the rendezvous is allowed to cost.  Everything after
      //  this point is spent out of one purse: the injection itself, the mid-
      //  course, and the arrival burn that stops the ship at the far end.  A
      //  transfer is only worth planning inside it.
      SET xferPurse TO rocketDvUsable(reserveDv) - CORR_DV_ALLOW.
      PRINT "  Budget from here: " + ROUND(xferPurse) +
            " m/s for the injection and the arrival together.".

      // --- 4b. Refine it against the real geometry -------------------------
      //  The node's ETA is a *countdown*, not a timestamp: it shrinks as the
      //  search runs, so restoring a rejected trial by writing the old ETA back
      //  would quietly walk the node later every round.  The wanted burn time
      //  is therefore held as an absolute UT and the countdown is derived from
      //  it each time it is written.
      //
      //  The search is scored on the miss AND on what the intercept costs to
      //  stop at.  Miss alone is the wrong objective and was the bug that put
      //  this ship past its station at seventy-odd metres a second: hitting a
      //  point is easy if you are allowed to arrive at any speed you like, and
      //  a hill-climb given a free hand will steepen the transfer round after
      //  round because a steeper one aims better.  Every one of those rounds
      //  spends the arrival burn's propellant before the arrival burn is lit.
      IF nodePredOk {
        SET stepDv TO REFINE_DV0.
        SET stepDt TO REFINE_DT0.
        SET bestMiss TO caNow["d"].
        SET bestCost TO ndX:DELTAV:MAG + arrNow.
        SET ndAbsT TO TIME:SECONDS + ndX:ETA.
        FROM { LOCAL rnd IS 0. } UNTIL rnd >= REFINE_ROUNDS STEP { SET rnd TO rnd + 1. } DO {
          SET improved TO FALSE.
          // Six neighbours: more and less prograde, more and less normal,
          // earlier and later.  Radial buys almost nothing on a transfer this
          // shape and costs two more evaluations a round, so it is left out.
          SET axes TO LIST("pro", "nrm", "eta").
          FOR ax IN axes {
            FOR sgn IN LIST(1, -1) {
              SET oldPro TO ndX:PROGRADE.
              SET oldNrm TO ndX:NORMAL.
              SET oldAbs TO ndAbsT.
              IF ax = "pro" { SET ndX:PROGRADE TO oldPro + sgn * stepDv. }
              ELSE IF ax = "nrm" { SET ndX:NORMAL TO oldNrm + sgn * stepDv. }
              ELSE {
                SET ndAbsT TO oldAbs + sgn * stepDt.
                SET ndX:ETA TO MAX(20, ndAbsT - TIME:SECONDS).
              }
              WAIT 0.
              SET tryCa TO closestApproach(TIME:SECONDS + ndX:ETA,
                                           TIME:SECONDS + ndX:ETA + tTrans * 1.6).
              SET tryCost TO ndX:DELTAV:MAG + relVelAt(tryCa["t"]).

              //  Two regimes, and which one applies is decided by the best so
              //  far, not by the trial - otherwise a trial that misses by
              //  miles is judged on price and wins for being cheap.
              //    Still short of an intercept: take the smaller miss, but
              //    never one the purse cannot cover.
              //    Already intercepting: keep it that way and get cheaper.
              SET takeIt TO FALSE.
              IF bestMiss > INTERCEPT_DIST {
                IF tryCa["d"] < bestMiss AND
                   (tryCost <= xferPurse OR tryCost < bestCost) {
                  SET takeIt TO TRUE.
                }
              } ELSE IF tryCa["d"] < INTERCEPT_DIST AND tryCost < bestCost - 0.05 {
                SET takeIt TO TRUE.
              }

              IF takeIt {
                SET bestMiss TO tryCa["d"].
                SET bestCost TO tryCost.
                SET caNow TO tryCa.
                SET improved TO TRUE.
              } ELSE {
                SET ndX:PROGRADE TO oldPro.
                SET ndX:NORMAL TO oldNrm.
                SET ndAbsT TO oldAbs.
                SET ndX:ETA TO MAX(20, ndAbsT - TIME:SECONDS).
                WAIT 0.
              }
            }
          }
          IF NOT improved {
            SET stepDv TO stepDv / 2.
            SET stepDt TO stepDt / 2.
          }
          IF stepDv < REFINE_FLOOR { BREAK. }
          // Stop once the plan both hits and can be paid for.  Stopping on the
          // miss alone, as this used to, walks away from a plan that arrives
          // faster than the tanks can stop.
          IF bestMiss < INTERCEPT_DIST AND bestCost <= xferPurse { BREAK. }
        }
        SET arrNow TO relVelAt(caNow["t"]).
        PRINT "  Refined to a " + ROUND(bestMiss / 1000, 2) + " km miss for " +
              ROUND(ndX:DELTAV:MAG, 1) + " m/s, arriving at " +
              ROUND(arrNow) + " m/s.".
      } ELSE {
        PRINT "  !! This install does not fold maneuver nodes into its".
        PRINT "     predictions - flying the closed-form plan unrefined and".
        PRINT "     letting the mid-course correction clean up.".
      }

      // --- 4c. The point of no return --------------------------------------
      //  Everything above this line ends in a circular orbit the ship can sit
      //  in, top up in, or deorbit from.  Below it the ship is on a transfer
      //  that arrives somewhere at a relative velocity somebody has to pay to
      //  stop, and a transfer flown with nothing left for the arrival does not
      //  buy a partial rendezvous - it buys a flyby, and it spends the orbit
      //  the ship had to do it.  The reserve is defended here, by not going,
      //  rather than at the far end by cutting the arrival burn short.
      SET arrCost TO relVelAt(caNow["t"]) + ARRIVE_DV_FLOOR.
      SET dvAfter TO rdvRocketDv() - ndX:DELTAV:MAG - reserveDv.
      IF dvAfter < arrCost {
        PRINT "  !! NOT COMMITTING to this transfer.".
        PRINT "     The injection costs " + ROUND(ndX:DELTAV:MAG) +
              " m/s and would leave " + ROUND(MAX(0, dvAfter)) + " m/s over the".
        PRINT "     reserve; stopping at the target needs " + ROUND(arrCost) +
              " m/s. Short by " + ROUND(arrCost - dvAfter) + " m/s.".
        PRINT "     Holding this orbit with the reserve intact. Refuel, drop the".
        PRINT "     payload, or run again from a lower orbit.".
        SET abortWhy TO "arrival unfunded".
        SET goOn TO FALSE.
        clearNodes().
      }

      IF goOn {
        IF NOT execNode(ndX, "Transfer injection", reserveDv) { SET goOn TO FALSE. }
        clearNodes().
      }
    }

    // --- 5. MID-COURSE CORRECTION ------------------------------------------
    //  Measured off the orbit the ship is actually on, with no node in the way,
    //  which is what makes it a genuine check on everything above rather than a
    //  second opinion from the same model.
    IF goOn AND NOT skipTransfer {
      SET caReal TO closestApproach(TIME:SECONDS + 30,
                                    TIME:SECONDS + SHIP:OBT:PERIOD * 1.1).
      SET tCa TO caReal["t"].
      SET dCa TO caReal["d"].
      PRINT "5. COAST :: closest approach " + ROUND(dCa / 1000, 2) + " km in " +
            ROUND((tCa - TIME:SECONDS) / 60, 1) + " min.".

      IF dCa > MIDCOURSE_MIN AND tCa - TIME:SECONDS > 180 {
        SET tCorr TO TIME:SECONDS + (tCa - TIME:SECONDS) * MIDCOURSE_FRAC.
        PRINT "  Correcting at T-" + ROUND((tCa - tCorr) / 60, 1) + " min.".
        SET ndC TO NODE(tCorr, 0, 0, 0).
        ADD ndC.
        WAIT 0.
        IF nodePredOk {
          SET stepDv TO 3.
          SET bestMiss TO dCa.
          FROM { LOCAL rnd IS 0. } UNTIL rnd >= 10 STEP { SET rnd TO rnd + 1. } DO {
            SET improved TO FALSE.
            SET axes TO LIST("pro", "nrm", "rad").
            FOR ax IN axes {
              FOR sgn IN LIST(1, -1) {
                SET oldPro TO ndC:PROGRADE.
                SET oldNrm TO ndC:NORMAL.
                SET oldRad TO ndC:RADIALOUT.
                IF ax = "pro" { SET ndC:PROGRADE TO oldPro + sgn * stepDv. }
                ELSE IF ax = "nrm" { SET ndC:NORMAL TO oldNrm + sgn * stepDv. }
                ELSE { SET ndC:RADIALOUT TO oldRad + sgn * stepDv. }
                WAIT 0.
                SET tryCa TO closestApproach(tCorr + 30, tCa + 600).
                IF tryCa["d"] < bestMiss {
                  SET bestMiss TO tryCa["d"].
                  SET improved TO TRUE.
                } ELSE {
                  SET ndC:PROGRADE TO oldPro.
                  SET ndC:NORMAL TO oldNrm.
                  SET ndC:RADIALOUT TO oldRad.
                  WAIT 0.
                }
              }
            }
            IF NOT improved { SET stepDv TO stepDv / 2. }
            IF stepDv < 0.05 { BREAK. }
            IF bestMiss < INTERCEPT_DIST { BREAK. }
          }
          PRINT "  Correction " + ROUND(ndC:DELTAV:MAG, 2) + " m/s -> miss " +
                ROUND(bestMiss / 1000, 2) + " km.".
          IF ndC:DELTAV:MAG > 0.1 {
            IF NOT execNode(ndC, "Mid-course", reserveDv) { SET goOn TO FALSE. }
          }
        }
        clearNodes().
      }
    }

    // --- 6. ARRIVAL :: kill the relative velocity ---------------------------
    IF goOn {
      SET caFin TO closestApproach(TIME:SECONDS + 5,
                                   TIME:SECONDS + MAX(600, SHIP:OBT:PERIOD)).
      SET tCa TO caFin["t"].
      SET dCa TO caFin["d"].
      PRINT "6. ARRIVAL :: closest approach " + ROUND(dCa / 1000, 2) +
            " km in " + ROUND((tCa - TIME:SECONDS) / 60, 1) + " min.".

      IF tCa - TIME:SECONDS > ARRIVE_LEAD + 5 {
        // Point the ship where the braking burn will want it before the wait,
        // then warp what is left.
        SAS OFF.
        LOCK STEERING TO LOOKDIRUP(tgtVes:VELOCITY:ORBIT - SHIP:VELOCITY:ORBIT,
                                   SHIP:UP:VECTOR).
        IF USE_WARP AND tCa - TIME:SECONDS > 120 {
          WARPTO(tCa - WARP_LEAD).
          WAIT UNTIL TIME:SECONDS >= tCa - WARP_LEAD OR TIME:SECONDS >= tCa.
          SET WARP TO 0.
          WAIT UNTIL KUNIVERSE:TIMEWARP:ISSETTLED.
        }
        WAIT UNTIL TIME:SECONDS >= tCa - ARRIVE_LEAD.
      }

      // This is the reading you want in the log: everything above is transfer,
      // everything below is rendezvous, and this is the line between them.
      rdvResourceReport("at intercept, before the approach").

      PRINT "  Nulling relative velocity (" +
            ROUND((SHIP:VELOCITY:ORBIT - tgtVes:VELOCITY:ORBIT):MAG, 1) + " m/s).".
      rdvGoClosedCycle().
      LOCK STEERING TO LOOKDIRUP(tgtVes:VELOCITY:ORBIT - SHIP:VELOCITY:ORBIT,
                                 SHIP:UP:VECTOR).
      SET tKill TO TIME:SECONDS + 300.
      // A steering command built from a difference of two nearly equal
      // velocities is a direction made of noise.  If the intercept already
      // arrived matched, there is nothing here for the main engines to do.
      // How slow does this have to get before RCS can take it?  Not a fixed
      // number: the answer is whatever the thrusters can brake away in the
      // distance that will be left, and on a heavy ship at long range that is
      // far less than a nominal handover speed.  Handing a 12 m/s closure to
      // 0.03 m/s^2 of RCS at two kilometres is a collision - the arithmetic
      // needs 2.4 km of braking room and has 1.8.
      SET killing TO TRUE.
      SET thrCmd TO 0.
      LOCK THROTTLE TO thrCmd.
      UNTIL NOT killing {
        SET vRelMag TO (SHIP:VELOCITY:ORBIT - tgtVes:VELOCITY:ORBIT):MAG.
        SET gapKill TO MAX(0, (tgtVes:POSITION):MAG - PARK_DIST).
        SET vHandover TO MIN(RCS_HANDOVER,
                             SQRT(2 * rdvApproachAccel() * gapKill / BRAKE_SAFETY)).
        IF vRelMag <= vHandover { SET killing TO FALSE. SET thrCmd TO 0. }
        ELSE IF TIME:SECONDS > tKill { SET killing TO FALSE. SET thrCmd TO 0. }
        ELSE IF rdvRocketDv() <= reserveDv {
          SET killing TO FALSE.
          SET thrCmd TO 0.
          PRINT "  !! Arrival burn stopped on the reserve with " +
                ROUND(vRelMag, 1) + " m/s still on.".
        } ELSE {
          // Only push while the nose is actually on the braking vector: thrust
          // applied off-axis does not slow the ship down, it steers it into a
          // new miss distance.
          IF VANG(SHIP:FACING:VECTOR,
                  tgtVes:VELOCITY:ORBIT - SHIP:VELOCITY:ORBIT) < ALIGN_TOL {
            SET thrCmd TO rdvClampVal(vRelMag / 20, 0.05, 1).
          } ELSE {
            SET thrCmd TO 0.
          }
        }
        WAIT 0.02.
      }
      SET thrCmd TO 0.
      LOCK THROTTLE TO 0.
      SET vResid TO (SHIP:VELOCITY:ORBIT - tgtVes:VELOCITY:ORBIT):MAG.
      PRINT "  Relative velocity down to " + ROUND(vResid, 2) + " m/s.".

      // --- 6b. Is there an approach left to fly? ----------------------------
      //  The terminal phase assumes the arrival burn did its job: it is a
      //  station-keeping controller with a closing schedule bolted on, and it
      //  is flown on thrusters worth a few m/s.  Handing it a residual it
      //  cannot null does not produce a slower rendezvous, it produces a
      //  flyby with the monopropellant spent - the ship coasts past the
      //  station, the RCS empties into a velocity it was never going to
      //  change, and what is left drifts away with no way to stop.  If the
      //  budget will not cover the residual, do not start.
      //
      //  The test is the residual and nothing else.  Closing is incremental -
      //  a ship that runs out of mono halfway in is closer than it was, and
      //  the loop below stops at MONO_RESERVE and says where it got to.
      //  Nulling is not: it either happens or the ship goes past.  So a budget
      //  that covers the residual but not the whole transit is still worth
      //  spending, and only a budget that cannot null is a reason not to start.
      SET vBudget TO monoDvUsable() + rocketDvUsable(reserveDv).
      SET vWanted TO vResid * TERM_V_MARGIN.
      IF vResid > PARK_V_TOL AND vBudget < vWanted {
        PRINT "  !! NOT CLOSING. " + ROUND(vResid, 1) +
              " m/s of relative velocity left and " + ROUND(vBudget, 1) +
              " m/s to null it with".
        PRINT "     (" + ROUND(rocketDvUsable(reserveDv), 1) +
              " m/s over the reserve, " + ROUND(monoDvUsable(), 1) +
              " m/s of usable mono). This is a flyby, not an approach.".
        PRINT "     Keeping what is left rather than spending it on a pass.".
        SET abortWhy TO "arrival burn ran out - flyby".
        SET goOn TO FALSE.
      }
    }

    // --- 7. TERMINAL APPROACH ----------------------------------------------
    //  RCS only from here.  The nose is held on the target both because it is
    //  the attitude a docking script wants to inherit and because it keeps the
    //  main engine bell pointed away from the station.
    IF goOn {
      PRINT "7. APPROACH :: closing to " + ROUND(PARK_DIST) + " m.".
      RCS ON.
      SET brakeHard TO FALSE.
      SET thrCmd TO 0.
      LOCK THROTTLE TO thrCmd.
      LOCK STEERING TO LOOKDIRUP(approachSteer(), SHIP:UP:VECTOR).

      SET tEnd     TO TIME:SECONDS + TERM_TIMEOUT.
      SET parkedAt TO 0.
      SET closing  TO TRUE.
      SET vLast    TO SHIP:VELOCITY:ORBIT - tgtVes:VELOCITY:ORBIT.
      SET tLast    TO TIME:SECONDS.
      SET lastFull TO FALSE.
      SET whyStop  TO "parked".

      UNTIL NOT closing {
        SET losVec TO tgtVes:POSITION.          // from us to the target
        SET dist   TO losVec:MAG.
        SET losHat TO losVec:NORMALIZED.
        SET vOurs  TO SHIP:VELOCITY:ORBIT - tgtVes:VELOCITY:ORBIT.

        // Measure what the thrusters are really delivering and believe that
        // over the nameplate figure.  A blocked or missing block, or a heavier
        // load than expected, shows up here and slows the approach down.
        //
        // Only sample it while the last interval was spent at *full* command.
        // Acceleration measured through a half-throttled correction is half the
        // authority the ship actually has, and feeding that back into the
        // braking schedule would ratchet the approach slower and slower for no
        // reason - each sluggish sample justifying the next.
        LOCAL dtM IS TIME:SECONDS - tLast.
        IF dtM > 0.5 {
          IF lastFull {
            LOCAL accM IS (vOurs - vLast):MAG / dtM.
            IF accM > RCS_ACC_MIN AND accM < 5 {
              SET rcsAccEst TO rcsAccEst * 0.8 + accM * 0.2.
            }
          }
          SET vLast TO vOurs.
          SET tLast TO TIME:SECONDS.
          SET lastFull TO FALSE.
        }

        // The braking schedule: never carry more speed than can be shed in the
        // distance left, with a safety factor on the stopping distance.  Past
        // the park point the sign flips and the same law backs the ship out
        // again, so a drift inward is station-keeping rather than a slow
        // collision nobody commanded.
        LOCAL gapSig IS dist - PARK_DIST.
        LOCAL vCap   IS SQRT(2 * rdvApproachAccel() * ABS(gapSig) / BRAKE_SAFETY).
        LOCAL vCmd   IS 0.
        IF gapSig > PARK_BAND {
          SET vCmd TO MAX(V_CLOSE_MIN, rdvClampVal(vCap, 0, V_CLOSE_MAX)).
        } ELSE IF gapSig < -PARK_BAND {
          SET vCmd TO -MIN(V_CLOSE_MIN, rdvClampVal(vCap, 0, V_CLOSE_MAX)).
        }

        // Can the ship still stop at all?  Closing speed squared against twice
        // the authority times the room left is the whole question, and if the
        // answer is no then no amount of RCS finesse fixes it - the main engine
        // has to come back on, pointed at the braking vector.  This is the
        // guard that stops a heavy ship that arrived hot from flying through
        // the station while its thrusters do their conscientious best.
        LOCAL vClose IS VDOT(vOurs, losHat).
        IF vClose > 0 AND gapSig > 0 AND
           vClose * vClose > 2 * rdvApproachAccel() * gapSig {
          IF NOT brakeHard {
            PRINT "  !! Closing at " + ROUND(vClose, 1) + " m/s with " +
                  ROUND(gapSig) + " m to stop in - RCS cannot. Main engine.".
          }
          SET brakeHard TO TRUE.
          SET vCmd TO 0.
        } ELSE IF brakeHard AND
                  vClose * vClose < 2 * rdvApproachAccel() * gapSig * 0.5 {
          // Hysteresis: come off the main engine only once there is real room
          // in hand, or the two modes will chatter against each other.
          PRINT "  Back inside the RCS braking envelope.".
          SET brakeHard TO FALSE.
          SET vCmd TO 0.
        }

        IF brakeHard {
          SET vCmd TO 0.
          IF rdvRocketDv() > reserveDv AND
             VANG(SHIP:FACING:VECTOR, approachSteer()) < ALIGN_TOL {
            SET thrCmd TO rdvClampVal(vClose / 20, 0.05, 1).
          } ELSE {
            SET thrCmd TO 0.
          }
          // Called for the main engine and there is none to be had, with more
          // closing speed than the thrusters can pay off: the pass is going to
          // happen whatever this loop does next.  Stop here with the mono still
          // aboard, because after the pass it is the only thing that can hold
          // the ship anywhere at all.
          IF rdvRocketDv() <= reserveDv AND
             monoDvUsable() < ABS(vClose) * TERM_V_MARGIN {
            SET closing TO FALSE.
            SET whyStop TO "unstoppable".
          }
        } ELSE {
          SET thrCmd TO 0.
        }

        // One command covers closing, braking and lateral nulling: fly the
        // velocity we want to have, minus the one we have got, on whichever
        // thrusters point that way.
        LOCAL vWant IS losHat * vCmd.
        LOCAL vErr  IS vWant - vOurs.
        LOCAL fCmd  IS VDOT(vErr, SHIP:FACING:FOREVECTOR) * TRANS_GAIN.
        LOCAL sCmd  IS VDOT(vErr, SHIP:FACING:STARVECTOR) * TRANS_GAIN.
        LOCAL tCmd  IS VDOT(vErr, SHIP:FACING:TOPVECTOR) * TRANS_GAIN.
        IF ABS(fCmd) < TRANS_DEAD { SET fCmd TO 0. }
        IF ABS(sCmd) < TRANS_DEAD { SET sCmd TO 0. }
        IF ABS(tCmd) < TRANS_DEAD { SET tCmd TO 0. }
        SET SHIP:CONTROL:FORE      TO rdvClampVal(fCmd, -1, 1).
        SET SHIP:CONTROL:STARBOARD TO rdvClampVal(sCmd, -1, 1).
        SET SHIP:CONTROL:TOP       TO rdvClampVal(tCmd, -1, 1).
        IF ABS(fCmd) > 0.95 OR ABS(sCmd) > 0.95 OR ABS(tCmd) > 0.95 {
          SET lastFull TO TRUE.
        }

        IF ABS(dist - PARK_DIST) < PARK_BAND AND vOurs:MAG < PARK_V_TOL {
          IF parkedAt = 0 { SET parkedAt TO TIME:SECONDS. }
          IF TIME:SECONDS - parkedAt > PARK_HOLD_T { SET closing TO FALSE. }
        } ELSE {
          SET parkedAt TO 0.
        }

        IF rdvResAmtShip("MonoPropellant") < MONO_RESERVE {
          SET closing TO FALSE.
          SET whyStop TO "mono".
        }
        IF TIME:SECONDS > tEnd {
          SET closing TO FALSE.
          SET whyStop TO "timeout".
        }
        WAIT 0.05.
      }

      SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
      IF whyStop = "mono" {
        PRINT "  !! Stopped at " + ROUND((tgtVes:POSITION):MAG) +
              " m - monopropellant down to MONO_RESERVE.".
        SET abortWhy TO "monopropellant".
      } ELSE IF whyStop = "timeout" {
        PRINT "  !! Approach timed out at " + ROUND((tgtVes:POSITION):MAG) + " m.".
        SET abortWhy TO "approach timed out".
      } ELSE IF whyStop = "unstoppable" {
        PRINT "  !! Stopped at " + ROUND((tgtVes:POSITION):MAG) +
              " m - closing faster than anything left aboard can brake.".
        PRINT "     Expect to pass the target. The mono is being kept for".
        PRINT "     station-keeping afterwards rather than spent on the pass.".
        SET abortWhy TO "closing faster than it can brake".
      }
    }

    // --- 8. REPORT ----------------------------------------------------------
    PRINT "======================================================".
    SET distFin TO (tgtVes:POSITION):MAG.
    SET vRelFin TO (SHIP:VELOCITY:ORBIT - tgtVes:VELOCITY:ORBIT):MAG.
    IF distFin < PARK_DIST + PARK_BAND * 4 AND vRelFin < 1 {
      PRINT "PARKED ALONGSIDE " + tgtVes:NAME.
    } ELSE IF abortWhy <> "" {
      PRINT "RENDEZVOUS INCOMPLETE :: " + abortWhy + ".".
    } ELSE {
      PRINT "RENDEZVOUS INCOMPLETE - handing back where we are.".
    }
    PRINT "  Range      : " + ROUND(distFin, 1) + " m".
    PRINT "  Rel speed  : " + ROUND(vRelFin, 3) + " m/s".
    // A relative velocity that is still on is a range that is still changing.
    // Say which way, because "range 1.3 km" reads like an arrival and this is
    // very often a departure.
    IF vRelFin > 1 {
      IF VDOT(SHIP:VELOCITY:ORBIT - tgtVes:VELOCITY:ORBIT,
              tgtVes:POSITION:NORMALIZED) < 0 {
        PRINT "               (opening - the range is growing)".
      } ELSE {
        PRINT "               (closing)".
      }
    }
    PRINT "  Our orbit  : " + ROUND(SHIP:APOAPSIS / 1000, 2) + " x " +
          ROUND(SHIP:PERIAPSIS / 1000, 2) + " km, inc " +
          ROUND(SHIP:ORBIT:INCLINATION, 3) + " deg".
    PRINT "  Target     : " + ROUND(tgtVes:OBT:APOAPSIS / 1000, 2) + " x " +
          ROUND(tgtVes:OBT:PERIAPSIS / 1000, 2) + " km, inc " +
          ROUND(tgtVes:OBT:INCLINATION, 3) + " deg".
    PRINT "  Rel inc    : " + ROUND(relIncDeg(), 4) + " deg".

    IF distFin < PARK_DIST + PARK_BAND * 4 AND vRelFin < 1 {
      rdvResourceReport("parked, rendezvous complete").
    } ELSE {
      rdvResourceReport("rendezvous incomplete").
    }

    SET dvNow  TO rdvRocketDv().
    SET dvHome TO reserveDvAtR(BODY_R + SHIP:ALTITUDE).
    PRINT "  dV left    : " + ROUND(dvNow) + " m/s".
    PRINT "  Deorbit    : needs " + ROUND(dvHome) +
          " m/s from here, priced at the delivered mass.".
    IF dvNow >= dvHome {
      PRINT "  DEORBIT FUNDED - RUN deorbit_land. when the mission is done.".
    } ELSE {
      PRINT "  !! DEORBIT NOT FUNDED - short by " + ROUND(dvHome - dvNow) +
            " m/s. Refuel at the station before leaving.".
    }
    IF DEPLOY_BEFORE_DEORBIT AND payloadMass > 1 {
      PRINT "  RELEASE THE PAYLOAD FIRST - that is what these numbers assume.".
    }
    PRINT "======================================================".
    IF distFin < PARK_DIST + PARK_BAND * 4 AND vRelFin < 1 {
      PRINT "Docking is not this script's job: select the target port and run".
      PRINT "the docking script. RCS is left on and the nose is on the target.".
    } ELSE {
      PRINT "Not parked, so there is nothing here for a docking script to".
      PRINT "inherit. The orbit is safe and the reserve is intact; fix the".
      PRINT "shortfall above and RUN rendezvous. again from here.".
    }

    rdvHandBack().
    RCS ON.
  }

  // Every path that stopped early - wrong body, retrograde target, suborbital,
  // or a refused budget - lands here.  Leaving the IPU raised and the throttle
  // locked after the program ends is how a pilot finds the stick dead.
  IF NOT tgtOk { rdvHandBack(). }
}
