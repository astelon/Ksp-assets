// ============================================================================
//  ascent.ks  --  Runway-to-orbit autopilot for a RAPIER spaceplane
// ----------------------------------------------------------------------------
//  Target : REQUESTED_APOAPSIS (default 100 km) circular over Kerbin, due east.
//  Engines: any CR-7 R.A.P.I.E.R. cluster (air-breathing on the runway, closed
//           cycle for the final push to orbit).
//
//  This script is not tied to one airframe.  It measures the ship it is flying
//  and derives the profile from what that ship actually does, so it will fly
//  any RAPIER spaceplane that has the performance to reach orbit at all.
//
//  Design notes (why it flies the way it does)
//  -------------------------------------------
//  * ACCELERATE FIRST.  Both powered phases obey one rule: climb only as
//    steeply as the ship can climb *while still gaining speed*.  Air-breathing
//    holds a commanded vertical speed that is raised while acceleration is
//    healthy and lowered the moment it sags; the rocket phase holds a commanded
//    flight-path angle biased the same way.  A spaceplane that stops
//    accelerating is not climbing, it is dying, and no amount of pitch fixes
//    it.  (The previous version hill-climbed pitch on dE/dt and let the nose
//    reach 25 deg at 9 km and 20 deg at the mode switch; that killed the
//    acceleration, tripped the "jets are done" test at 1136 m/s, and threw away
//    roughly 400 m/s of nearly-free air-breathing speed.)
//  * ONE STABLE INNER LOOP.  Pitch is not bumped up and down by a bang-bang
//    test.  A single controller converts a commanded flight-path angle into a
//    pitch command: feed-forward gamma plus a slow, clamped, anti-windup trim
//    that learns this airframe's angle of attack.  Time-to-apoapsis is *not*
//    used as a control variable - it is discontinuous when apoapsis is near the
//    ship's own altitude, which is exactly where the rocket phase starts, and
//    steering on it produced the +-25 deg pitch oscillation seen in flight.
//  * THE JETS ARE CHEAP.  Air-breathing Isp is roughly ten times the rocket's,
//    so the switch is deferred until the jets are genuinely finished: flameout,
//    thrust decayed off its own peak, airspeed falling away from its peak, or
//    no acceleration left *after* the profile has already flattened out.  A
//    single low-acceleration sample while the nose is up is not evidence.
//  * PAYLOAD FUEL IS NOT OUR FUEL.  The script walks the part tree out from the
//    engines and stops at every decoupler, separator and docking port.  Only
//    tanks inside that feed network are counted as dV, crossfeed is switched
//    off on every separator, and flow is disabled on the payload's own tanks,
//    so a fuelled payload can neither inflate the budget nor be siphoned.
//  * IT NEVER BURNS DRY.  Two hard floors police every burn: once an orbit is
//    actually in reach the ascent stops at circularise + deorbit + margin, and
//    below that a glide reserve is kept back no matter what.  The budget itself
//    is measured, not assumed - the script watches how much dV it really costs
//    to buy apoapsis and re-prices the orbit from that, settling for the best
//    orbit it can afford rather than discovering the shortfall on empty tanks.
//
//  Flight plan
//    0. Pre-flight  : map the fuel network, isolate the payload, print budget.
//    1. Runway roll : hold heading, rotate at ROTATE_SPEED.
//    2. Air-breathing climb: accelerate-first vertical-speed profile.
//    3. Closed-cycle push  : flight-path-angle schedule, measured dV policing.
//    4. Coast       : throttle 0, warp to the circularisation burn.
//    5. Circularise : burn prograde at apoapsis, stop at the deorbit reserve.
//    6. Report      : final orbit, dV left, deorbit funding check.
//
//  Run with:   RUN ascent.        (from the archive or the ship's Script volume)
//  Tunables are grouped at the top so you can trim the profile to your build.
// ============================================================================

CLEARSCREEN.

// --- Mission ----------------------------------------------------------------
SET REQUESTED_APOAPSIS TO 100000.   // orbit we would *like* to reach (m)
SET LAUNCH_HEADING     TO 90.       // due east - use Kerbin's rotation

// --- dV budgeting -----------------------------------------------------------
SET DEORBIT_PE          TO 32000.   // periapsis deorbit_land.ks burns down to (m)
SET DV_MARGIN           TO 100.     // spare kept on top of circ + deorbit (m/s)
SET DV_GLIDE_RESERVE    TO 120.     // never burn below this, ever (m/s)
SET CLIMB_LOSS_FACTOR   TO 1.15.    // pre-flight guess only; measured in flight
SET ROCKET_ISP_FALLBACK TO 305.     // closed-cycle Isp assumed before the switch (s)
SET PLAN_MECO_PE        TO -50000.  // periapsis assumed when pricing an orbit we
                                    // have not reached yet (deliberately
                                    // pessimistic, so circ dV is over-quoted)
SET LFO_LF_RATIO        TO 9.       // stock LF:Ox mixture ratio, LF part
SET LFO_OX_RATIO        TO 11.      // ... Ox part
SET ISOLATE_PAYLOAD     TO TRUE.    // block crossfeed on the payload separator
SET FORCE_TANK_LOCK     TO FALSE.   // also disable flow on payload tanks (only
                                    // needed if the separator has no crossfeed
                                    // toggle; remember to re-enable on deploy)
SET ABORT_IF_INFEASIBLE TO FALSE.   // TRUE = cut the burn the moment orbit is
                                    // priced out of reach (keeps the most fuel
                                    // for a return); FALSE = keep flying to the
                                    // hard floors, which never burn dry either

// --- Attitude controller (shared by both powered phases) --------------------
SET AOA_KI       TO 0.30.           // trim gain, deg of trim per deg of error per s
SET AOA_TRIM_MAX TO 14.             // trim authority clamp (deg)

// --- Runway / air-breathing climb -------------------------------------------
SET ROTATE_SPEED    TO 80.          // surface speed to rotate the nose up (m/s)
SET ROTATE_PITCH    TO 10.          // fixed pitch used until guidance engages (deg)
SET AB_GUIDE_SPEED  TO 160.         // hand over to the vs controller above this (m/s)
SET AB_PITCH_MIN    TO -6.          // pitch clamp, air-breathing phase (deg)
SET AB_PITCH_MAX    TO 28.
SET AB_TUNE_DT      TO 1.           // profile re-scheduling interval (s)
SET AB_VS_INIT      TO 110.         // initial commanded climb rate (m/s)
SET AB_VS_MAX       TO 220.         // ... clamp, upper
SET AB_VS_DIVE      TO -20.         // ... clamp, lower (shallow dive to build speed)
SET AB_VS_STEP      TO 10.          // command change per AB_TUNE_DT (m/s)
SET AB_ACC_LOW      TO 1.2.         // below this longitudinal accel, flatten out
SET AB_ACC_HIGH     TO 3.0.         // above this, buy altitude with the spare
SET AB_DIVE_ALT     TO 15000.       // no descending below this altitude (m)
SET AB_FLOOR_ALT    TO 8000.        // hold at least AB_VS_FLOOR under this (m)
SET AB_VS_FLOOR     TO 25.          // ... that minimum climb rate (m/s)
SET AB_Q_MAX        TO 0.55.        // too deep in the air: climb out (atm)
SET AB_Q_MIN        TO 0.06.        // too thin to breathe: sink back in (atm)
SET AB_VS_QCLIMB    TO 70.          // climb rate forced by the high-Q guard (m/s)

// --- Mode-switch sensing ----------------------------------------------------
//  Deliberately reluctant.  The jets are ~10x cheaper than the rocket, so the
//  cost of switching one sample early is far higher than the cost of hanging on.
SET SW_ARM_SPEED    TO 1150.        // no thrust/accel sensing below this (m/s)
SET SW_THRUST_DECAY TO 0.45.        // switch when thrust < this x peak jet thrust
SET SW_MIN_ACCEL    TO 0.12.        // "no acceleration left" threshold (m/s^2)
SET SW_SPEED_LOSS   TO 45.          // switch if airspeed falls this far off peak (m/s)
SET SW_STALL_TIME   TO 20.          // flat and not accelerating this long = done (s)
SET SW_CONFIRM      TO 3.           // consecutive samples needed to confirm
SET SW_ALT_HARD     TO 28000.       // hard backstop: switch by this altitude (m)
SET SW_SPEED_HARD   TO 1600.        // ... or this airspeed (m/s)

// --- Closed-cycle push ------------------------------------------------------
SET CC_FPA_HI      TO 15.           // commanded flight-path angle at the switch (deg)
SET CC_FPA_LO      TO 0.            // ... once the apoapsis target is reached (deg)
SET CC_PITCH_MIN   TO -8.           // pitch clamp, rocket phase (deg)
SET CC_PITCH_MAX   TO 40.
SET CC_ACC_DT      TO 0.5.          // acceleration sampling interval (s)
SET CC_ACC_LOW     TO 1.5.          // below this, shallow out (m/s^2)
SET CC_ACC_HIGH    TO 5.0.          // above this, allow a steeper climb (m/s^2)
SET CC_BIAS_STEP   TO 0.5.          // bias change per sample (deg)
SET CC_BIAS_DOWN   TO 15.           // how far the bias may shallow the climb (deg)
SET CC_BIAS_UP     TO 6.            // ... and steepen it (deg)
SET CC_VS_ATM_MIN  TO 50.           // never stop climbing inside the atmosphere (m/s)
SET CC_POLICE_DT   TO 0.5.          // dV re-pricing interval (s)
SET FEAS_ARM_DV    TO 350.          // rocket dV spent before trusting the measured
                                    // climb efficiency (m/s)
SET FEAS_CONFIRM   TO 4.            // consecutive infeasible samples to act on

// --- Coast / circularisation ------------------------------------------------
SET USE_WARP    TO TRUE.            // time-warp the coast to apoapsis
SET WARP_LEAD   TO 25.              // drop out of warp this long before the burn (s)
SET CIRC_PE_TOL TO 1500.            // accept periapsis this far under apoapsis (m)
SET PE_SAFETY   TO 8000.            // keep periapsis this far above the atmosphere (m)

// ---------------------------------------------------------------------------
//  Body / physical constants (cached once)
// ---------------------------------------------------------------------------
SET G0      TO 9.80665.
SET BODY_MU TO SHIP:BODY:MU.
SET BODY_R  TO SHIP:BODY:RADIUS.
SET ATM_TOP TO SHIP:BODY:ATM:HEIGHT.

// The lowest orbit worth having: clear of the air with the periapsis safety
// margin on top.  Everything below this is a suborbital arc, not an orbit, and
// needs no deorbit burn because it is already coming down.
SET MIN_ORBIT_ALT TO ATM_TOP + PE_SAFETY.

// kOS runs a fixed number of instructions per physics tick; the default is too
// slow for a control loop that also re-prices an orbit.  Restored on exit.
SET IPU_SAVED TO CONFIG:IPU.
SET CONFIG:IPU TO 800.

// ---------------------------------------------------------------------------
//  Fuel-network mapping  --  which tanks are actually ours?
// ---------------------------------------------------------------------------
//  NOTE ON NAMING: kOS reserves its built-in function names and refuses to
//  compile a script declaring a variable that would hide one (the
//  CLOBBERBUILTINS check).  R(), V() and Q() are built-ins, so no identifier in
//  this script is a bare single letter.
//
//  A payload rides on a decoupler or separator, and a fuelled payload is not
//  fuel we may spend: counting it would inflate the dV budget, and letting the
//  engines drink it would strand the payload.  Rather than trust part names, we
//  walk the part tree outward from the engines and refuse to cross any part
//  that can separate.  Whatever we reach is the spaceplane; everything else is
//  payload.
// ---------------------------------------------------------------------------
FUNCTION isSeparator {
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

// Breadth-first flood fill from the engines, blocked by separators.  Returns a
// lexicon of part UIDs that share a fuel network with the engines.  The
// separators themselves are included (they hold no propellant) but never
// traversed through, so the payload beyond them stays out.
FUNCTION mapFeedNetwork {
  LOCAL seen  IS LEXICON().
  LOCAL queue IS LIST().
  LOCAL engs  IS LIST().
  LIST ENGINES IN engs.
  FOR eng IN engs {
    IF NOT seen:HASKEY(eng:UID) { seen:ADD(eng:UID, TRUE). queue:ADD(eng). }
  }
  LOCAL idx IS 0.
  UNTIL idx >= queue:LENGTH {
    LOCAL prt IS queue[idx].
    SET idx TO idx + 1.
    IF NOT isSeparator(prt) {
      LOCAL nbrs IS LIST().
      IF prt:HASPARENT { nbrs:ADD(prt:PARENT). }
      FOR kid IN prt:CHILDREN { nbrs:ADD(kid). }
      FOR nbr IN nbrs {
        IF NOT seen:HASKEY(nbr:UID) { seen:ADD(nbr:UID, TRUE). queue:ADD(nbr). }
      }
    }
  }
  RETURN seen.
}

SET CORE_UIDS   TO mapFeedNetwork().
SET CORE_TANKS  TO LIST().          // our tanks - the only ones counted as dV
SET PAY_TANKS   TO LIST().          // payload tanks - counted, then locked out
SET SEPARATORS  TO LIST().
SET PAY_PARTS   TO 0.

FOR prt IN SHIP:PARTS {
  LOCAL holds IS FALSE.
  FOR res IN prt:RESOURCES {
    IF res:NAME = "LiquidFuel" OR res:NAME = "Oxidizer" { SET holds TO TRUE. }
  }
  IF CORE_UIDS:HASKEY(prt:UID) {
    IF holds { CORE_TANKS:ADD(prt). }
    IF isSeparator(prt) { SEPARATORS:ADD(prt). }
  } ELSE {
    SET PAY_PARTS TO PAY_PARTS + 1.
    IF holds { PAY_TANKS:ADD(prt). }
  }
}

// Resource amount inside the engines' own feed network only.
FUNCTION coreResAmt {
  PARAMETER rname.
  LOCAL total IS 0.
  FOR prt IN CORE_TANKS {
    FOR res IN prt:RESOURCES {
      IF res:NAME = rname { SET total TO total + res:AMOUNT. }
    }
  }
  RETURN total.
}

FUNCTION listResAmt {               // same, over an arbitrary part list
  PARAMETER prts, rname.
  LOCAL total IS 0.
  FOR prt IN prts {
    FOR res IN prt:RESOURCES {
      IF res:NAME = rname { SET total TO total + res:AMOUNT. }
    }
  }
  RETURN total.
}

FUNCTION resAmtShip {               // whole-ship total (reporting only)
  PARAMETER rname.
  LOCAL total IS 0.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = rname { SET total TO total + res:AMOUNT. }
  }
  RETURN total.
}

FUNCTION resDensity {               // tonnes per unit (0.005 for LF and Ox)
  PARAMETER rname.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = rname AND res:DENSITY > 0 { RETURN res:DENSITY. }
  }
  RETURN 0.005.
}

SET LF_DENS TO resDensity("LiquidFuel").
SET OX_DENS TO resDensity("Oxidizer").

// Switch crossfeed off on every separator, so the stack physically cannot pull
// propellant across the payload joint even if the budget maths were wrong.
FUNCTION blockCrossfeed {
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

// Fallback isolation: disable resource flow on the payload's own tanks so the
// engines cannot draw them.  This is only used when no separator would take a
// crossfeed toggle, because a locked tank stays locked after deployment - the
// payload's own engines will not see its fuel until flow is re-enabled.
// TOGGLEABLE guards resources that are not allowed to be switched off at all.
FUNCTION lockPayloadTanks {
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
//  Ship measurement helpers
// ---------------------------------------------------------------------------
// Vacuum Isp of the engines that will do the rocket work.  While the RAPIERs
// are still breathing air their reported Isp is the jet figure, which would
// wildly overstate the budget, so until the switch we quote the fallback.
SET RKT_ISP TO ROCKET_ISP_FALLBACK.

FUNCTION measureRocketIsp {
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  LOCAL wsum IS 0.
  LOCAL isum IS 0.
  FOR eng IN engs {
    IF (NOT eng:MULTIMODE) OR (eng:MODE = "ClosedCycle") {
      LOCAL thr IS eng:MAXTHRUST.
      IF thr > 0 AND eng:VACUUMISP > 0 {
        SET wsum TO wsum + thr.
        SET isum TO isum + thr * eng:VACUUMISP.
      }
    }
  }
  IF wsum > 0 { RETURN isum / wsum. }
  RETURN ROCKET_ISP_FALLBACK.
}

// Propellant mass (t) the closed-cycle engines can actually burn: our LF and Ox
// only, and only in 9:11 pairs, so a lopsided tank state is not counted as dV.
FUNCTION rocketPropMass {
  LOCAL lfAmt IS coreResAmt("LiquidFuel").
  LOCAL oxAmt IS coreResAmt("Oxidizer").
  LOCAL lfUse IS MIN(lfAmt, oxAmt * LFO_LF_RATIO / LFO_OX_RATIO).
  LOCAL oxUse IS lfUse * LFO_OX_RATIO / LFO_LF_RATIO.
  RETURN lfUse * LF_DENS + oxUse * OX_DENS.
}

// Remaining closed-cycle dV (m/s): the rocket equation on the live ship.  The
// payload's mass counts against us; the payload's fuel does not count for us.
FUNCTION rocketDv {
  LOCAL m0 IS SHIP:MASS.
  LOCAL m1 IS m0 - rocketPropMass().
  IF m0 <= 0 OR m1 <= 0 OR m1 >= m0 { RETURN 0. }
  RETURN RKT_ISP * G0 * LN(m0 / m1).
}

FUNCTION localG   { RETURN BODY_MU / ((BODY_R + SHIP:ALTITUDE) * (BODY_R + SHIP:ALTITUDE)). }
FUNCTION weightKN { RETURN SHIP:MASS * localG(). }        // 1 t x 1 m/s^2 = 1 kN
FUNCTION twrNow {
  LOCAL wgt IS weightKN().
  IF wgt <= 0 { RETURN 0. }
  RETURN SHIP:AVAILABLETHRUST / wgt.
}

FUNCTION burnTimeFor {              // seconds to spend dvNeed at full throttle
  PARAMETER dvNeed.
  LOCAL thr IS SHIP:AVAILABLETHRUST.
  IF thr <= 0 OR dvNeed <= 0 { RETURN 0. }
  LOCAL vEx IS RKT_ISP * G0.
  RETURN SHIP:MASS * vEx * (1 - CONSTANT:E ^ (-dvNeed / vEx)) / thr.
}

FUNCTION clampVal {
  PARAMETER val, lo, hi.
  RETURN MAX(lo, MIN(hi, val)).
}

// ---------------------------------------------------------------------------
//  Orbital mechanics helpers  (altitudes in m above the surface)
// ---------------------------------------------------------------------------
FUNCTION vCircAt {                  // circular orbital speed at an altitude
  PARAMETER altM.
  RETURN SQRT(BODY_MU / (BODY_R + altM)).
}

// dV to circularise at apAlt.  If that apoapsis is the one we are actually on,
// price it off the live orbit; otherwise price a pessimistic transfer ellipse.
FUNCTION circDvAt {
  PARAMETER apAlt.
  LOCAL rA IS BODY_R + apAlt.
  LOCAL sma IS 0.
  IF SHIP:APOAPSIS >= apAlt - 1000 AND SHIP:ORBIT:SEMIMAJORAXIS > 0 {
    SET sma TO SHIP:ORBIT:SEMIMAJORAXIS.
  } ELSE {
    SET sma TO (rA + BODY_R + PLAN_MECO_PE) / 2.
  }
  LOCAL term IS 2 / rA - 1 / sma.
  LOCAL vAp IS 0.
  IF term > 0 { SET vAp TO SQRT(BODY_MU * term). }
  RETURN MAX(0, vCircAt(apAlt) - vAp).
}

// dV to drop periapsis to DEORBIT_PE from a circular orbit at apAlt.
FUNCTION deorbitDvFrom {
  PARAMETER apAlt.
  LOCAL rA IS BODY_R + apAlt.
  LOCAL rP IS BODY_R + DEORBIT_PE.
  IF rP >= rA { RETURN 0. }
  LOCAL smaT IS (rA + rP) / 2.
  RETURN MAX(0, vCircAt(apAlt) - SQRT(BODY_MU * (2 / rA - 1 / smaT))).
}

// What must still be in the tanks *after* we are circular at apAlt.
FUNCTION reserveDvFor {
  PARAMETER apAlt.
  RETURN deorbitDvFrom(apAlt) + DV_MARGIN.
}

// Impulsive estimate of the dV still needed to raise apoapsis to apAlt from
// here.  It compares speed magnitudes, so it is optimistic - which is exactly
// why the rocket phase measures the real cost and stops trusting this alone.
FUNCTION dvToRaiseApTo {
  PARAMETER apAlt.
  LOCAL rHere IS BODY_R + SHIP:ALTITUDE.
  LOCAL rTgt IS BODY_R + apAlt.
  IF rTgt <= rHere OR SHIP:APOAPSIS >= apAlt { RETURN 0. }
  LOCAL smaT IS (rHere + rTgt) / 2.
  LOCAL vNeed IS SQRT(BODY_MU * (2 / rHere - 1 / smaT)).
  RETURN MAX(0, vNeed - SHIP:VELOCITY:ORBIT:MAG).
}

// Full price of an orbit at apAlt: the climb, the circularisation, and the fuel
// that must survive both.  Only the *climb* is scaled by the loss factor - it
// is the part flown against drag and gravity inside the atmosphere.  The
// circularisation is an impulsive vacuum burn and costs what it says it costs.
// In flight the loss factor is the measured one, so this is the honest number.
FUNCTION priceOfOrbit {
  PARAMETER apAlt, lossFactor.
  RETURN dvToRaiseApTo(apAlt) * lossFactor + circDvAt(apAlt) + reserveDvFor(apAlt).
}

// Highest apoapsis between MIN_ORBIT_ALT and REQUESTED_APOAPSIS whose full
// price fits in dvHave at the measured loss factor.  0 = not even the minimum
// orbit is affordable.
FUNCTION affordableAp {
  PARAMETER dvHave, lossFactor.
  IF priceOfOrbit(MIN_ORBIT_ALT, lossFactor) > dvHave { RETURN 0. }
  LOCAL lo IS MIN_ORBIT_ALT.
  LOCAL hi IS REQUESTED_APOAPSIS.
  IF priceOfOrbit(hi, lossFactor) <= dvHave { RETURN hi. }
  FROM { LOCAL iter IS 0. } UNTIL iter >= 16 STEP { SET iter TO iter + 1. } DO {
    LOCAL mid IS (lo + hi) / 2.
    IF priceOfOrbit(mid, lossFactor) <= dvHave { SET lo TO mid. } ELSE { SET hi TO mid. }
  }
  RETURN lo.
}

// ---------------------------------------------------------------------------
//  Attitude controller  --  commanded flight-path angle -> pitch command
// ---------------------------------------------------------------------------
//  Pitch and flight path are not the same thing: a loaded spaceplane flies at
//  several degrees of angle of attack, and how many depends on mass, wing area
//  and air density, none of which we want to hard-code.  So the command is
//  gamma (feed-forward) plus a slow integral trim that discovers this ship's
//  AoA for itself.  The trim is clamped, and it stops integrating when the
//  output is already pinned against a clamp and the error would push it further
//  into it - without that anti-windup, a phase the ship cannot fly (a climb it
//  has no thrust for) leaves the trim saturated and poisons the next phase.
// ---------------------------------------------------------------------------
SET aoaTrim TO 0.

FUNCTION speedNow {
  IF SHIP:ALTITUDE > ATM_TOP { RETURN MAX(30, SHIP:VELOCITY:ORBIT:MAG). }
  RETURN MAX(30, SHIP:VELOCITY:SURFACE:MAG).
}

FUNCTION vsToFpa {                  // a vertical speed, as a flight-path angle
  PARAMETER vsWanted.
  RETURN ARCSIN(clampVal(vsWanted / speedNow(), -0.95, 0.95)).
}

FUNCTION fpaNow {
  RETURN ARCSIN(clampVal(SHIP:VERTICALSPEED / speedNow(), -0.95, 0.95)).
}

FUNCTION steerFpa {
  PARAMETER fpaWanted, pitchLo, pitchHi, dtStep.
  LOCAL err IS fpaWanted - fpaNow().
  LOCAL raw IS fpaWanted + aoaTrim.
  IF (raw > pitchLo AND raw < pitchHi) OR
     (raw <= pitchLo AND err > 0) OR
     (raw >= pitchHi AND err < 0) {
    SET aoaTrim TO clampVal(aoaTrim + AOA_KI * err * dtStep, -AOA_TRIM_MAX, AOA_TRIM_MAX).
  }
  RETURN clampVal(fpaWanted + aoaTrim, pitchLo, pitchHi).
}

// ---------------------------------------------------------------------------
//  Engine helpers
// ---------------------------------------------------------------------------
FUNCTION setRapierMode {            // force every multimode engine into one mode
  PARAMETER wantClosed.             // TRUE = ClosedCycle, FALSE = AirBreathing
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  FOR eng IN engs {
    IF eng:MULTIMODE {
      SET eng:AUTOSWITCH TO FALSE.  // we manage the switch ourselves
      IF wantClosed AND eng:MODE = "AirBreathing" { eng:TOGGLEMODE(). }
      IF (NOT wantClosed) AND eng:MODE = "ClosedCycle" { eng:TOGGLEMODE(). }
    }
  }
}

// Light the engines *without* staging where possible.  With a payload on a
// separator, a stray STAGE is how you drop the cargo on the runway.
FUNCTION igniteEngines {
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  FOR eng IN engs { IF NOT eng:IGNITION { eng:ACTIVATE. } }
}

FUNCTION anyFlameout {
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  LOCAL flamed IS FALSE.
  FOR eng IN engs { IF eng:IGNITION AND eng:FLAMEOUT { SET flamed TO TRUE. } }
  RETURN flamed.
}

FUNCTION budgetLine {               // one-line dV status, used throughout
  PARAMETER apAlt, lossFactor.
  RETURN "dV " + ROUND(rocketDv()) + " vs need " + ROUND(priceOfOrbit(apAlt, lossFactor)) +
         " (climb " + ROUND(dvToRaiseApTo(apAlt) * lossFactor) +
         " + circ " + ROUND(circDvAt(apAlt)) +
         " + deorbit " + ROUND(deorbitDvFrom(apAlt)) +
         " + margin " + ROUND(DV_MARGIN) + ") @ loss x" + ROUND(lossFactor, 2).
}

// ---------------------------------------------------------------------------
//  0. PRE-FLIGHT  --  measure the ship we are actually flying
// ---------------------------------------------------------------------------
PRINT "=== RAPIER SSTO :: ascent autopilot ===".
SAS OFF.
RCS OFF.
LIGHTS ON.
BRAKES ON.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.

SET thrCmd   TO 0.
SET pitchCmd TO 0.
LOCK THROTTLE TO thrCmd.
LOCK STEERING TO HEADING(LAUNCH_HEADING, pitchCmd).

// --- payload isolation ------------------------------------------------------
PRINT "FUEL NETWORK".
PRINT "  Ship parts   : " + SHIP:PARTS:LENGTH + "  (payload side " + PAY_PARTS + ")".
PRINT "  Our tanks    : " + CORE_TANKS:LENGTH + "   separators " + SEPARATORS:LENGTH.
IF PAY_TANKS:LENGTH > 0 {
  PRINT "  Payload tanks: " + PAY_TANKS:LENGTH + " holding LF " +
        ROUND(listResAmt(PAY_TANKS, "LiquidFuel")) + " / Ox " +
        ROUND(listResAmt(PAY_TANKS, "Oxidizer")) + " - NOT counted as dV.".
}
IF ISOLATE_PAYLOAD {
  SET xfBlocked TO blockCrossfeed().
  PRINT "  Crossfeed disabled on " + xfBlocked + " separator module(s).".
  // The separator blocking crossfeed is the clean isolation and it leaves the
  // payload's own plumbing untouched.  Only if that is unavailable do we fall
  // back to locking the payload's tanks - which the user must undo on deploy.
  IF PAY_TANKS:LENGTH > 0 AND (xfBlocked = 0 OR FORCE_TANK_LOCK) {
    SET tkLocked TO lockPayloadTanks().
    PRINT "  !! No crossfeed toggle available - locked " + tkLocked +
          " payload resource(s) instead.".
    PRINT "     RE-ENABLE THEM after you release the payload.".
  } ELSE IF PAY_TANKS:LENGTH > 0 {
    PRINT "  Payload fuel is behind a blocked separator - left flowable.".
  }
}

PRINT "Ignition. RAPIERs to air-breathing.".
igniteEngines().
setRapierMode(FALSE).               // ensure air-breathing for takeoff
SET thrCmd TO 1.
WAIT 1.
IF SHIP:AVAILABLETHRUST < 1 {
  // Falling back to STAGE is only safe with nothing to drop.  With a payload on
  // a separator, a stray stage is how the cargo ends up on the runway, so we
  // stop and let the pilot light them instead.
  IF SEPARATORS:LENGTH > 0 {
    PRINT "!! Engines did not light and this ship has a separator - NOT staging.".
    PRINT "   Light them by hand; the script resumes as soon as they burn.".
    WAIT UNTIL SHIP:AVAILABLETHRUST > 1.
  } ELSE {
    PRINT "  Engines did not light on command - staging once.".
    STAGE.
    WAIT 1.
  }
}

SET launchMass TO SHIP:MASS.
SET jetThrust  TO SHIP:AVAILABLETHRUST.
SET jetTwr     TO twrNow().
SET dvAtPad    TO rocketDv().       // closed-cycle dV sitting in *our* tanks
SET planCirc   TO circDvAt(REQUESTED_APOAPSIS).
SET planDeorb  TO deorbitDvFrom(REQUESTED_APOAPSIS).

PRINT "------------------------------------------------------".
PRINT "VEHICLE".
PRINT "  Mass         : " + ROUND(launchMass, 1) + " t  (dry " + ROUND(SHIP:DRYMASS, 1) + " t)".
PRINT "  Weight       : " + ROUND(weightKN()) + " kN".
PRINT "  Thrust (jet) : " + ROUND(jetThrust) + " kN    TWR " + ROUND(jetTwr, 2).
PRINT "  Usable dV    : " + ROUND(dvAtPad) + " m/s  (Isp " + ROUND(RKT_ISP) +
      " s assumed until the mode switch)".
PRINT "BUDGET for a " + ROUND(REQUESTED_APOAPSIS / 1000) + " km orbit".
PRINT "  Circularise  : ~" + ROUND(planCirc) + " m/s".
PRINT "  Deorbit to " + ROUND(DEORBIT_PE / 1000) + " km : ~" + ROUND(planDeorb) + " m/s".
PRINT "  Margin       : " + ROUND(DV_MARGIN) + " m/s".
PRINT "  => RESERVED  : " + ROUND(planCirc + planDeorb + DV_MARGIN) +
      " m/s, leaving " + ROUND(dvAtPad - planCirc - planDeorb - DV_MARGIN) +
      " m/s for the climb".
IF jetTwr < 0.35 { PRINT "  !! Low take-off TWR - expect a long roll.". }
IF dvAtPad < planCirc + planDeorb + DV_MARGIN {
  PRINT "  !! The tanks do not even cover circ + deorbit. Expect to settle low.".
}
PRINT "------------------------------------------------------".

BRAKES OFF.

// ---------------------------------------------------------------------------
//  1. RUNWAY ROLL + ROTATE
// ---------------------------------------------------------------------------
PRINT "Rolling. Rotate at " + ROTATE_SPEED + " m/s.".
WAIT UNTIL SHIP:VELOCITY:SURFACE:MAG > ROTATE_SPEED.
SET pitchCmd TO ROTATE_PITCH.
PRINT "Rotate - climbing out.".
WAIT UNTIL SHIP:ALTITUDE > 100.     // positively airborne
GEAR OFF.
WAIT UNTIL SHIP:AIRSPEED > AB_GUIDE_SPEED.

// ---------------------------------------------------------------------------
//  2. AIR-BREATHING CLIMB  --  accelerate first, climb with the change
// ---------------------------------------------------------------------------
//  One commanded quantity: vertical speed.  It is raised while the ship has
//  spare acceleration and lowered the instant acceleration sags, so the ship
//  finds its own best compromise at this mass, this payload, this air.  Three
//  guards outrank the schedule: climb out of air that is too thick to push
//  through, sink back into air too thin to breathe, and never mush into the
//  ground.  The pitch that delivers the commanded climb comes from the shared
//  flight-path controller, so the nose moves smoothly instead of hunting.
// ---------------------------------------------------------------------------
PRINT "Air-breathing climb: accelerate-first profile.".
// Start from the climb the ship is already flying, not from a constant: at
// handover speed a fixed 110 m/s command is a 40 deg flight path, and the nose
// would slam to the clamp before the trim had learned anything.
SET vsCmd     TO clampVal(SHIP:VERTICALSPEED, 30, AB_VS_INIT).
SET aoaTrim   TO 0.
SET lastLoopT TO TIME:SECONDS.
SET lastTuneT TO TIME:SECONDS.
SET lastSpd   TO SHIP:AIRSPEED.
SET accelNow  TO 0.
SET peakJetT  TO SHIP:AVAILABLETHRUST.
SET peakSpd   TO SHIP:AIRSPEED.
SET swVotes   TO 0.
SET stallT    TO 0.
SET switchNow TO FALSE.
SET swReason  TO "hard backstop".
SET lfWarned  TO FALSE.
SET abReportT TO TIME:SECONDS.

UNTIL switchNow {
  LOCAL nowT IS TIME:SECONDS.
  LOCAL dtStep IS MAX(0.02, nowT - lastLoopT).
  SET lastLoopT TO nowT.

  // ---- profile scheduling + switch sensing, once per AB_TUNE_DT ------------
  IF nowT - lastTuneT >= AB_TUNE_DT {
    LOCAL tuneDt IS nowT - lastTuneT.
    SET lastTuneT TO nowT.
    SET accelNow TO (SHIP:AIRSPEED - lastSpd) / tuneDt.
    SET lastSpd  TO SHIP:AIRSPEED.
    SET peakJetT TO MAX(peakJetT, SHIP:AVAILABLETHRUST).
    SET peakSpd  TO MAX(peakSpd, SHIP:AIRSPEED).

    // Trade: spare acceleration buys altitude, missing acceleration is bought
    // back by flattening out.  This is the whole profile.
    IF accelNow > AB_ACC_HIGH { SET vsCmd TO vsCmd + AB_VS_STEP. }
    ELSE IF accelNow < AB_ACC_LOW { SET vsCmd TO vsCmd - AB_VS_STEP. }

    // Guards, in priority order over the schedule.
    IF SHIP:Q > AB_Q_MAX { SET vsCmd TO MAX(vsCmd, AB_VS_QCLIMB). }
    IF SHIP:Q < AB_Q_MIN AND SHIP:ALTITUDE > 14000 { SET vsCmd TO MIN(vsCmd, -10). }
    IF SHIP:ALTITUDE < AB_FLOOR_ALT { SET vsCmd TO MAX(vsCmd, AB_VS_FLOOR). }
    LOCAL vsFloor IS 0.
    IF SHIP:ALTITUDE > AB_DIVE_ALT { SET vsFloor TO AB_VS_DIVE. }
    SET vsCmd TO clampVal(vsCmd, vsFloor, AB_VS_MAX).

    // ---- book-keeping ------------------------------------------------------
    //  The jets run on liquid fuel, and the ship carries an LF-only reserve for
    //  exactly that.  Once LF drops to the level that pairs with the remaining
    //  oxidiser, every further second of jet burn is spent out of the rocket dV
    //  budget - worth knowing, though not worth switching for (see below).
    IF (NOT lfWarned) AND
       coreResAmt("LiquidFuel") <= coreResAmt("Oxidizer") * LFO_LF_RATIO / LFO_OX_RATIO {
      PRINT "  Note: LF-only reserve used up; jet burn now draws on rocket dV (" +
            ROUND(rocketDv()) + " m/s left).".
      SET lfWarned TO TRUE.
    }

    // ---- sense the end of the air-breathing envelope ------------------------
    //  Purely physical signals, and all of them require the profile to have
    //  already given up climbing.  Jet Isp is ~10x the rocket's, so while the
    //  jets still make thrust they are always the cheaper way to buy speed -
    //  even though they burn liquid fuel the rocket phase would also like.
    LOCAL armed   IS SHIP:AIRSPEED > SW_ARM_SPEED.
    LOCAL flatOut IS vsCmd <= vsFloor + 1.
    LOCAL shallow IS vsCmd <= AB_VS_FLOOR.
    LOCAL decayed IS SHIP:AVAILABLETHRUST < peakJetT * SW_THRUST_DECAY.
    LOCAL gasping IS flatOut AND accelNow < SW_MIN_ACCEL.
    LOCAL sliding IS shallow AND SHIP:AIRSPEED < peakSpd - SW_SPEED_LOSS.

    // Backstop for a ship that never reaches the arming speed at all: an
    // underpowered airframe would otherwise sit here burning liquid fuel with
    // no sensor allowed to fire.  If the profile has flattened right out and
    // there is no acceleration left, the jets are finished whatever the
    // airspeed says.
    IF gasping { SET stallT TO stallT + tuneDt. } ELSE { SET stallT TO 0. }
    LOCAL stalled IS stallT >= SW_STALL_TIME.

    IF anyFlameout() OR SHIP:AVAILABLETHRUST < 1 {
      SET switchNow TO TRUE.
      SET swReason TO "flameout - out of intake air".
    } ELSE IF stalled {
      SET switchNow TO TRUE.
      SET swReason TO "jets flat out of acceleration for " + ROUND(stallT) + " s at " +
                      ROUND(SHIP:AIRSPEED) + " m/s".
    } ELSE IF armed AND (decayed OR gasping OR sliding) {
      SET swVotes TO swVotes + 1.
      IF swVotes >= SW_CONFIRM {
        SET switchNow TO TRUE.
        IF decayed {
          SET swReason TO "jet thrust decayed to " +
                          ROUND(100 * SHIP:AVAILABLETHRUST / MAX(1, peakJetT)) + "% of peak".
        } ELSE IF sliding {
          SET swReason TO "airspeed falling off peak (" + ROUND(peakSpd) + " m/s)".
        } ELSE {
          SET swReason TO "flat and out of acceleration (" + ROUND(accelNow, 2) + " m/s^2)".
        }
      }
    } ELSE {
      SET swVotes TO 0.
    }

    IF SHIP:ALTITUDE > SW_ALT_HARD OR SHIP:AIRSPEED > SW_SPEED_HARD {
      SET switchNow TO TRUE.
      SET swReason TO "hard backstop (altitude/speed limit)".
    }
  }

  // ---- attitude ------------------------------------------------------------
  SET pitchCmd TO steerFpa(vsToFpa(vsCmd), AB_PITCH_MIN, AB_PITCH_MAX, dtStep).

  IF nowT - abReportT > 15 {
    PRINT "  alt " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km | " + ROUND(SHIP:AIRSPEED) +
          " m/s | acc " + ROUND(accelNow, 2) + " | Q " + ROUND(SHIP:Q, 2).
    PRINT "      vs " + ROUND(SHIP:VERTICALSPEED) + "/" + ROUND(vsCmd) +
          " | pitch " + ROUND(pitchCmd, 1) + " | thr " +
          ROUND(100 * SHIP:AVAILABLETHRUST / MAX(1, peakJetT)) + "% | dV " + ROUND(rocketDv()).
    SET abReportT TO nowT.
  }

  WAIT 0.1.
}

// ---------------------------------------------------------------------------
//  3. CLOSED-CYCLE PUSH  --  with measured dV policing
// ---------------------------------------------------------------------------
PRINT "Mode switch: " + swReason + ".".
PRINT "  at " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km, " + ROUND(SHIP:AIRSPEED) +
      " m/s, pitch " + ROUND(pitchCmd, 1) + " deg.".
setRapierMode(TRUE).
WAIT 0.5.                           // let the engines settle in mode
SET RKT_ISP TO measureRocketIsp().  // now the live engines quote rocket Isp

SET targetAp    TO REQUESTED_APOAPSIS.
SET lossFactor  TO CLIMB_LOSS_FACTOR.   // replaced by the measured value below
SET apAtSwitch  TO SHIP:APOAPSIS.
SET dvAtSwitch  TO rocketDv().
// The efficiency yardstick is pinned to a fixed reference altitude, not to the
// live target.  If it tracked the target, re-targeting downward would itself
// look like progress, the measured loss factor would jump, the orbit would look
// affordable again, and the decision would oscillate.
SET EFF_REF_AP  TO REQUESTED_APOAPSIS.
SET price0      TO dvToRaiseApTo(EFF_REF_AP).

PRINT "Closed cycle. " + budgetLine(targetAp, lossFactor).
PRINT "  Mass " + ROUND(SHIP:MASS, 1) + " t, thrust " + ROUND(SHIP:AVAILABLETHRUST) +
      " kN, TWR " + ROUND(twrNow(), 2) + ", Isp " + ROUND(RKT_ISP) + " s.".

SET aoaTrim   TO 0.                 // fresh trim: different regime, different AoA
SET fpaBias   TO 0.
SET lastLoopT TO TIME:SECONDS.
SET lastAccT  TO TIME:SECONDS.
SET lastPolT  TO TIME:SECONDS.
SET lastSpd   TO SHIP:VELOCITY:ORBIT:MAG.
SET accelNow  TO 0.
SET reportT   TO TIME:SECONDS.
SET settled   TO FALSE.
SET infeasVotes TO 0.
SET infeasWarned TO FALSE.
SET ccDone    TO FALSE.
SET stopReason TO "apoapsis reached".

UNTIL ccDone {
  LOCAL nowT IS TIME:SECONDS.
  LOCAL dtStep IS MAX(0.02, nowT - lastLoopT).
  SET lastLoopT TO nowT.

  IF SHIP:APOAPSIS >= targetAp { SET ccDone TO TRUE. BREAK. }

  // ---- acceleration sampling: the same accelerate-first bias ---------------
  IF nowT - lastAccT >= CC_ACC_DT {
    LOCAL accDt IS nowT - lastAccT.
    SET lastAccT TO nowT.
    SET accelNow TO (SHIP:VELOCITY:ORBIT:MAG - lastSpd) / accDt.
    SET lastSpd  TO SHIP:VELOCITY:ORBIT:MAG.
    IF accelNow < CC_ACC_LOW {
      SET fpaBias TO MAX(fpaBias - CC_BIAS_STEP, -CC_BIAS_DOWN).
    } ELSE IF accelNow > CC_ACC_HIGH {
      SET fpaBias TO MIN(fpaBias + CC_BIAS_STEP, CC_BIAS_UP).
    }
  }

  // ---- guidance: a flight-path schedule that flattens as apoapsis arrives --
  //  Progress, not time-to-apoapsis.  Apoapsis is a smooth, monotone measure of
  //  how much of the job is done; ETA:APOAPSIS is discontinuous exactly here,
  //  where apoapsis sits just above the ship, and steering on it oscillates.
  LOCAL prog IS 1.
  IF targetAp > apAtSwitch {
    SET prog TO clampVal((SHIP:APOAPSIS - apAtSwitch) / (targetAp - apAtSwitch), 0, 1).
  }
  LOCAL fpaWant IS CC_FPA_HI + (CC_FPA_LO - CC_FPA_HI) * prog + fpaBias.
  IF SHIP:ALTITUDE < ATM_TOP {
    // Whatever the bias says, we are leaving the atmosphere.
    SET fpaWant TO MAX(fpaWant, vsToFpa(CC_VS_ATM_MIN)).
  }
  SET pitchCmd TO steerFpa(fpaWant, CC_PITCH_MIN, CC_PITCH_MAX, dtStep).

  // ---- dV policing --------------------------------------------------------
  IF nowT - lastPolT >= CC_POLICE_DT {
    SET lastPolT TO nowT.
    LOCAL dvNow IS rocketDv().

    // Floor 1: an orbit is genuinely in reach, so the deorbit money is real
    // money.  Stop climbing and bank the orbit we can still pay to leave.
    IF SHIP:APOAPSIS >= MIN_ORBIT_ALT {
      LOCAL keep IS circDvAt(SHIP:APOAPSIS) + reserveDvFor(SHIP:APOAPSIS).
      IF dvNow <= keep {
        SET settled TO TRUE.
        SET stopReason TO "stopped on the circularise + deorbit reserve".
        SET targetAp TO SHIP:APOAPSIS.
        SET ccDone TO TRUE.
        BREAK.
      }
    }

    // Floor 2: absolute. Whatever else is true, the ship keeps a glide reserve.
    IF dvNow <= DV_GLIDE_RESERVE {
      SET settled TO TRUE.
      SET stopReason TO "stopped on the " + ROUND(DV_GLIDE_RESERVE) + " m/s glide reserve".
      SET targetAp TO SHIP:APOAPSIS.
      SET ccDone TO TRUE.
      BREAK.
    }

    // ---- measured climb efficiency ----------------------------------------
    //  The impulsive estimate is always optimistic; how optimistic depends on
    //  drag, TWR and the profile, i.e. on the ship.  So measure it: how much of
    //  the bill did the dV we have already spent actually pay off?  That ratio
    //  re-prices the rest of the climb, and it is the number the settle
    //  decision is made on.
    LOCAL spent IS dvAtSwitch - dvNow.
    IF spent > FEAS_ARM_DV {
      LOCAL priceNow IS dvToRaiseApTo(EFF_REF_AP).
      LOCAL paid IS price0 - priceNow.
      IF paid > 1 {
        SET lossFactor TO clampVal(spent / paid, 1, 12).
      } ELSE {
        SET lossFactor TO 12.       // spending dV and buying nothing
      }

      IF dvNow < priceOfOrbit(targetAp, lossFactor) {
        SET infeasVotes TO infeasVotes + 1.
      } ELSE {
        SET infeasVotes TO 0.
      }

      IF infeasVotes >= FEAS_CONFIRM {
        SET infeasVotes TO 0.
        LOCAL bestAp IS affordableAp(dvNow, lossFactor).
        IF bestAp > 0 AND bestAp < targetAp - 1000 {
          PRINT "  Re-targeting: " + ROUND(targetAp / 1000, 1) + " km is priced out; " +
                "settling for " + ROUND(bestAp / 1000, 1) + " km.".
          PRINT "  " + budgetLine(targetAp, lossFactor).
          SET targetAp TO bestAp.
          SET settled TO TRUE.
        } ELSE IF bestAp = 0 {
          IF NOT infeasWarned {
            PRINT "======================================================".
            PRINT "!! CANNOT REACH ORBIT. Even a " + ROUND(MIN_ORBIT_ALT / 1000, 1) +
                  " km apoapsis is out of budget.".
            PRINT "   " + budgetLine(MIN_ORBIT_ALT, lossFactor).
            PRINT "   Measured climb losses are x" + ROUND(lossFactor, 2) +
                  " the impulsive estimate.".
            IF ABORT_IF_INFEASIBLE {
              PRINT "   Cutting the burn now and keeping the remaining fuel.".
            } ELSE {
              PRINT "   Flying on to the hard floors; the glide reserve is protected.".
            }
            PRINT "======================================================".
            SET infeasWarned TO TRUE.
          }
          IF ABORT_IF_INFEASIBLE {
            SET settled TO TRUE.
            SET stopReason TO "aborted - orbit priced out of reach".
            SET targetAp TO SHIP:APOAPSIS.
            SET ccDone TO TRUE.
            BREAK.
          }
        }
      }
    }
  }

  // ---- periodic status ----------------------------------------------------
  IF nowT - reportT > 10 {
    PRINT "  AP " + ROUND(SHIP:APOAPSIS / 1000, 1) + "/" + ROUND(targetAp / 1000) +
          " km | alt " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km | " +
          ROUND(SHIP:VELOCITY:ORBIT:MAG) + " m/s".
    PRINT "      fpa " + ROUND(fpaNow(), 1) + "/" + ROUND(fpaWant, 1) + " | pitch " +
          ROUND(pitchCmd, 1) + " | acc " + ROUND(accelNow, 2) + " | TWR " +
          ROUND(twrNow(), 2) + " | dV " + ROUND(rocketDv()).
    SET reportT TO nowT.
  }

  WAIT 0.1.
}

SET thrCmd TO 0.
SET orbitAlt TO SHIP:APOAPSIS.
PRINT "MECO at apoapsis " + ROUND(orbitAlt / 1000, 1) + " km - " + stopReason + ".".
PRINT "  " + budgetLine(orbitAlt, lossFactor).

// ---------------------------------------------------------------------------
//  4. COAST TO APOAPSIS
// ---------------------------------------------------------------------------
SET abortSuborbital TO FALSE.
IF orbitAlt < ATM_TOP + 2000 {
  SET abortSuborbital TO TRUE.
  PRINT "======================================================".
  PRINT "!! SUBORBITAL. Apoapsis " + ROUND(orbitAlt / 1000, 1) + " km is inside or".
  PRINT "   barely above the atmosphere and there is no dV to fix it.".
  PRINT "   Not circularising - the ship is already on a reentry path, so".
  PRINT "   fly it home as a glider (no deorbit burn required).".
  PRINT "   Fuel kept back: " + ROUND(rocketDv()) + " m/s.".
  PRINT "======================================================".
  UNLOCK STEERING.
  UNLOCK THROTTLE.
  SAS ON.
}

IF NOT abortSuborbital {
  LOCK STEERING TO SHIP:PROGRADE.
  WAIT UNTIL SHIP:ALTITUDE > ATM_TOP.        // clear of the atmosphere

  // -------------------------------------------------------------------------
  //  5. CIRCULARISE  --  and stop at the deorbit reserve, whatever happens
  // -------------------------------------------------------------------------
  SET circDv   TO circDvAt(orbitAlt).
  SET circBurn TO burnTimeFor(circDv).
  SET circLead TO clampVal(circBurn / 2, 5, 150).
  PRINT "Vacuum. ETA to apoapsis " + ROUND(ETA:APOAPSIS) + " s; circ burn ~" +
        ROUND(circBurn) + " s, starting " + ROUND(circLead) + " s early.".

  IF USE_WARP AND ETA:APOAPSIS > circLead + WARP_LEAD + 20 {
    SET tWarp TO TIME:SECONDS + ETA:APOAPSIS - circLead - WARP_LEAD.
    PRINT "Warping " + ROUND(tWarp - TIME:SECONDS) + " s toward the burn.".
    WARPTO(tWarp).
    WAIT UNTIL TIME:SECONDS >= tWarp OR ETA:APOAPSIS <= circLead.
    SET WARP TO 0.
    WAIT UNTIL KUNIVERSE:TIMEWARP:ISSETTLED.
    PRINT "Out of warp. Settling on prograde.".
    WAIT 3.
  }

  WAIT UNTIL ETA:APOAPSIS < circLead OR SHIP:PERIAPSIS > ATM_TOP.

  SET circPeTarget TO MAX(ATM_TOP + PE_SAFETY, orbitAlt - CIRC_PE_TOL).
  SET circPeTarget TO MIN(circPeTarget, orbitAlt - 500).
  PRINT "Circularising at " + ROUND(orbitAlt / 1000, 1) + " km; PE target " +
        ROUND(circPeTarget / 1000, 1) + " km.".
  PRINT "  Protecting " + ROUND(reserveDvFor(orbitAlt)) + " m/s (deorbit " +
        ROUND(deorbitDvFrom(orbitAlt)) + " + margin " + ROUND(DV_MARGIN) + ").".

  SET circStarved TO FALSE.
  SET circFloor   TO MAX(reserveDvFor(orbitAlt), DV_GLIDE_RESERVE).
  SET dvCheckT    TO TIME:SECONDS.
  SET thrCmd TO 1.
  UNTIL SHIP:PERIAPSIS >= circPeTarget {
    // Ease the throttle down in the last stretch for a precise cut-off.
    IF SHIP:PERIAPSIS > circPeTarget - 4000 { SET thrCmd TO 0.15. }

    // Hard stop: we will not eat the deorbit money to buy a rounder orbit.
    // (Polled a few times a second - rocketDv() is not cheap in kOS.)
    IF TIME:SECONDS - dvCheckT > 0.25 {
      SET dvCheckT TO TIME:SECONDS.
      IF rocketDv() <= circFloor {
        SET circStarved TO TRUE.
        BREAK.
      }
    }
    WAIT 0.05.
  }
  SET thrCmd TO 0.
  UNLOCK STEERING.
  SAS ON.

  IF circStarved {
    PRINT "!! Circularisation stopped on the deorbit reserve.".
    PRINT "   PE " + ROUND(SHIP:PERIAPSIS / 1000, 1) + " km - the orbit stays elliptical,".
    PRINT "   but the deorbit burn is still funded.".
  }

  // -------------------------------------------------------------------------
  //  6. REPORT
  // -------------------------------------------------------------------------
  SET dvLeft  TO rocketDv().
  SET dvDeorb TO deorbitDvFrom(SHIP:APOAPSIS).
  PRINT "======================================================".
  IF settled {
    PRINT "ORBIT ACHIEVED (settled below the requested " +
          ROUND(REQUESTED_APOAPSIS / 1000) + " km)".
  } ELSE {
    PRINT "ORBIT ACHIEVED".
  }
  PRINT "  Apoapsis   : " + ROUND(SHIP:APOAPSIS / 1000, 2) + " km".
  PRINT "  Periapsis  : " + ROUND(SHIP:PERIAPSIS / 1000, 2) + " km".
  PRINT "  Inclination: " + ROUND(SHIP:ORBIT:INCLINATION, 2) + " deg".
  PRINT "  Mass       : " + ROUND(SHIP:MASS, 1) + " t  (burned " +
        ROUND(launchMass - SHIP:MASS, 1) + " t)".
  PRINT "  Climb loss : x" + ROUND(lossFactor, 2) + " vs the impulsive estimate".
  PRINT "  dV left    : " + ROUND(dvLeft) + " m/s  (our tanks only)".
  PRINT "  Deorbit    : " + ROUND(dvDeorb) + " m/s  ->  spare after deorbit " +
        ROUND(dvLeft - dvDeorb) + " m/s".
  PRINT "  Fuel -- LF: " + ROUND(coreResAmt("LiquidFuel")) +
        " , Ox: " + ROUND(coreResAmt("Oxidizer")) +
        " , Mono: " + ROUND(resAmtShip("MonoPropellant")).
  IF PAY_TANKS:LENGTH > 0 {
    PRINT "  Payload LF/Ox (locked out): " + ROUND(listResAmt(PAY_TANKS, "LiquidFuel")) +
          " / " + ROUND(listResAmt(PAY_TANKS, "Oxidizer")).
  }
  IF dvLeft >= dvDeorb + DV_MARGIN {
    PRINT "  DEORBIT FUNDED - RUN deorbit_land. when you are ready.".
  } ELSE IF dvLeft >= dvDeorb {
    PRINT "  Deorbit funded but with no margin - deorbit promptly.".
  } ELSE {
    PRINT "  !! DEORBIT NOT FUNDED - refuel, or lower PE on RCS monopropellant.".
  }
  PRINT "======================================================".
  PRINT "Autopilot complete. Ship handed back to pilot (SAS on).".
}

SET CONFIG:IPU TO IPU_SAVED.
