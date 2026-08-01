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

// --- Pre-flight feasibility model -------------------------------------------
//  A go/no-go check run on the runway, before the ship has spent anything.  It
//  models the flight as: jets take the ship to a handover state, then the
//  rocket does the rest.  The handover assumptions below are what a healthy
//  RAPIER spaceplane achieves on Kerbin; the script prints the *actual* values
//  at the real mode switch so you can calibrate them for your airframe.
SET PLAN_SWITCH_ALT  TO 20000.      // assumed altitude at the mode switch (m)
SET PLAN_SWITCH_SPD  TO 1450.       // ... and surface speed there (m/s)
SET PLAN_JET_DV      TO 3000.       // jet-phase dV equivalent, incl. drag (m/s)
SET PLAN_LOSS_FACTOR TO 1.50.       // gravity/drag/steering losses, rocket climb
SET PLAN_TWR_MIN     TO 1.05.       // closed-cycle TWR wanted at the handover
SET PLAN_JET_ISP     TO 3200.       // air-breathing Isp if it cannot be read (s)
SET PREFLIGHT_HOLD   TO 12.         // pause this long on a failed check (s)

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
//  NOTE ON NAMING: kOS refuses to compile a script that declares a variable
//  hiding one of its built-in functions (the CLOBBERBUILTINS check).  This is
//  not just about the one-letter ones - R(), V(), Q() - it catches ordinary
//  words too: QUEUE, STACK, LIST, RANGE, NODE, PATH, BODY, CORE, TIME and a
//  long tail of others are all built-ins.  `tools/check_kos.py` checks a script
//  against the list before the game does; run it after editing.
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
  LOCAL pending IS LIST().
  LOCAL engs  IS LIST().
  LIST ENGINES IN engs.
  FOR eng IN engs {
    IF NOT seen:HASKEY(eng:UID) { seen:ADD(eng:UID, TRUE). pending:ADD(eng). }
  }
  LOCAL idx IS 0.
  UNTIL idx >= pending:LENGTH {
    LOCAL prt IS pending[idx].
    SET idx TO idx + 1.
    IF NOT isSeparator(prt) {
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

// Isp of the engines in whatever mode they are in *now*, at the current
// ambient pressure.  Used to read the jet Isp on the runway, and to work out
// how much of the closed-cycle thrust the sea-level pressure is eating.
FUNCTION measureCurrentIsp {
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  LOCAL wsum IS 0.
  LOCAL isum IS 0.
  FOR eng IN engs {
    LOCAL thr IS eng:MAXTHRUST.
    IF thr > 0 AND eng:ISP > 0 {
      SET wsum TO wsum + thr.
      SET isum TO isum + thr * eng:ISP.
    }
  }
  IF wsum > 0 { RETURN isum / wsum. }
  RETURN 0.
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

// The same two questions asked about an *assumed* state rather than the live
// one, so the mission can be priced on the runway.  dvRaiseApFrom is the
// impulsive cost of stretching the orbit out to apAlt from (altM, spd);
// circDvFrom is the burn waiting at the far end of that transfer.
FUNCTION dvRaiseApFrom {
  PARAMETER altM, spd, apAlt.
  LOCAL rHere IS BODY_R + altM.
  LOCAL rTgt  IS BODY_R + apAlt.
  IF rTgt <= rHere { RETURN 0. }
  LOCAL smaT IS (rHere + rTgt) / 2.
  RETURN MAX(0, SQRT(BODY_MU * (2 / rHere - 1 / smaT)) - spd).
}

FUNCTION circDvFrom {
  PARAMETER altM, apAlt.
  LOCAL rA IS BODY_R + apAlt.
  LOCAL sma IS (rA + BODY_R + altM) / 2.
  IF sma <= 0 OR rA <= 0 { RETURN 0. }
  LOCAL term IS 2 / rA - 1 / sma.
  IF term <= 0 { RETURN 0. }
  RETURN MAX(0, vCircAt(apAlt) - SQRT(BODY_MU * term)).
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
//  Pre-flight feasibility  --  can this ship afford the mission at all?
// ---------------------------------------------------------------------------
//  The question a launch autopilot cannot answer once it is airborne, and the
//  one that matters while the ship is still being designed: is there enough dV
//  aboard, and if not, how much more of what?
//
//  The model is deliberately explicit about the two things that make a
//  spaceplane different from a rocket:
//
//   * The jets burn liquid fuel the rocket phase would otherwise have.  Simply
//     counting every paired LF/Ox unit in the tanks - which is what the ΔV
//     figure on the pad reports - badly overstates what will be left when the
//     RAPIERs switch over.  So the model burns the jet phase first, then prices
//     the rocket phase against what survives.
//   * Adding propellant is not free.  Tanks have structure, so x tonnes of
//     propellant costs x(1+k) tonnes on the runway, where k is measured from
//     this ship's own tanks.  That is why the answer to "how much more fuel?"
//     is not linear, and why there is a hard ceiling: as tanks are added the
//     mass ratio tends to (1+k)/k and no amount of fuel gets past it.  A design
//     that needs more than that ceiling cannot be fixed with fuel at all.
// ---------------------------------------------------------------------------
// The LF share of a balanced LF/Ox load.  LF and Ox share a density, so mass
// ratios equal unit ratios.
SET LF_FRAC TO LFO_LF_RATIO / (LFO_LF_RATIO + LFO_OX_RATIO).
SET OX_FRAC TO 1 - LF_FRAC.

// Fraction of launch mass the jets consume as liquid fuel, from the rocket
// equation at air-breathing Isp over the jet phase's dV equivalent.
FUNCTION jetBurnFraction {
  PARAMETER jetIsp.
  RETURN 1 - CONSTANT:E ^ (-PLAN_JET_DV / (jetIsp * G0)).
}

//  All three take the upgrade as a split (addLf, addOx, addDry) rather than a
//  single propellant figure: topping up liquid fuel alone is a different - and
//  often much cheaper - fix from adding balanced LF/Ox, and the jets can only
//  ever drink the LF.
FUNCTION padMassWith {              // launch mass with an upgrade fitted
  PARAMETER addLf, addOx, addDry.
  RETURN launchMass + (addLf + addOx) * (1 + TANK_K) + addDry.
}

FUNCTION jetBurnWith {              // tonnes of LF the jets eat, upgrade fitted
  PARAMETER addLf, addOx, addDry.
  RETURN MIN(padMassWith(addLf, addOx, addDry) * JET_FRAC, lfMass0 + addLf).
}

FUNCTION handoverMass {
  PARAMETER addLf, addOx, addDry.
  RETURN padMassWith(addLf, addOx, addDry) - jetBurnWith(addLf, addOx, addDry).
}

// Rocket dV left at the handover, with addLf / addOx tonnes of extra
// propellant and addDry tonnes of extra dry mass (engines) fitted.  addDry may
// be negative, to ask what shedding payload would buy.
FUNCTION dvHandoverSplit {
  PARAMETER addLf, addOx, addDry.
  LOCAL lfLeft IS lfMass0 + addLf - jetBurnWith(addLf, addOx, addDry).
  LOCAL oxLeft IS oxMass0 + addOx.
  LOCAL lfUse IS MIN(lfLeft, oxLeft * LFO_LF_RATIO / LFO_OX_RATIO).
  IF lfUse <= 0 { RETURN 0. }
  LOCAL prop IS lfUse * (LFO_LF_RATIO + LFO_OX_RATIO) / LFO_LF_RATIO.
  LOCAL mH IS handoverMass(addLf, addOx, addDry).
  IF mH <= prop { RETURN 0. }
  RETURN RKT_ISP * G0 * LN(mH / (mH - prop)).
}

FUNCTION dvAtHandover {             // ... adding a balanced LF/Ox load
  PARAMETER addProp, addDry.
  RETURN dvHandoverSplit(addProp * LF_FRAC, addProp * OX_FRAC, addDry).
}

// How much liquid fuel is left over once the oxidiser is fully paired off -
// i.e. the LF-only reserve the jets are meant to run on.  Negative means the
// air-breathing phase is eating fuel the rocket phase was counting on, which
// is the single most common way an otherwise sound SSTO comes up short.
FUNCTION lfSurplusWith {
  PARAMETER addLf.
  RETURN (lfMass0 + addLf - jetBurnWith(addLf, 0, 0)) -
         oxMass0 * LFO_LF_RATIO / LFO_OX_RATIO.
}

// Smallest LF-only top-up (t) that stops the jets eating paired propellant.
FUNCTION solveLfTopUp {
  IF lfSurplusWith(0) >= 0 { RETURN 0. }
  LOCAL hi IS 1.
  UNTIL lfSurplusWith(hi) >= 0 OR hi > 20000 { SET hi TO hi * 2. }
  IF hi > 20000 { RETURN -1. }
  LOCAL lo IS 0.
  FROM { LOCAL iter IS 0. } UNTIL iter >= 24 STEP { SET iter TO iter + 1. } DO {
    LOCAL mid IS (lo + hi) / 2.
    IF lfSurplusWith(mid) >= 0 { SET hi TO mid. } ELSE { SET lo TO mid. }
  }
  RETURN hi.
}

// Tonnes of propellant that must be added to reach dvNeed at the handover.
// -1 means no amount will do it: the mass ratio ceiling is in the way.
FUNCTION solveProp {
  PARAMETER dvNeed, addDry.
  IF dvAtHandover(0, addDry) >= dvNeed { RETURN 0. }
  LOCAL hi IS 1.
  UNTIL dvAtHandover(hi, addDry) >= dvNeed OR hi > 20000 { SET hi TO hi * 2. }
  IF hi > 20000 { RETURN -1. }
  LOCAL lo IS 0.
  FROM { LOCAL iter IS 0. } UNTIL iter >= 24 STEP { SET iter TO iter + 1. } DO {
    LOCAL mid IS (lo + hi) / 2.
    IF dvAtHandover(mid, addDry) >= dvNeed { SET hi TO mid. } ELSE { SET lo TO mid. }
  }
  RETURN hi.
}

// Tonnes of payload that would have to come off instead.  -1 if even flying
// empty would not close the gap.
FUNCTION solveMassCut {
  PARAMETER dvNeed.
  IF dvAtHandover(0, 0) >= dvNeed { RETURN 0. }
  LOCAL hi IS payloadMass.
  IF hi <= 0 OR dvAtHandover(0, -hi) < dvNeed { RETURN -1. }
  LOCAL lo IS 0.
  FROM { LOCAL iter IS 0. } UNTIL iter >= 24 STEP { SET iter TO iter + 1. } DO {
    LOCAL mid IS (lo + hi) / 2.
    IF dvAtHandover(0, -mid) >= dvNeed { SET hi TO mid. } ELSE { SET lo TO mid. }
  }
  RETURN hi.
}

FUNCTION twrAtHandover {
  PARAMETER addProp, addDry, addEng.
  LOCAL gSw IS BODY_MU / ((BODY_R + PLAN_SWITCH_ALT) * (BODY_R + PLAN_SWITCH_ALT)).
  LOCAL thr IS ccThrustVac * (engCount + addEng) / MAX(1, engCount).
  RETURN thr / (handoverMass(addProp * LF_FRAC, addProp * OX_FRAC, addDry) * gSw).
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

PRINT "Ignition (throttle closed - measuring both engine modes).".
igniteEngines().
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

// --- measure both engine modes, with the throttle shut ----------------------
//  The RAPIER reports the Isp and thrust of whichever mode it is *in*, so the
//  only way to budget a closed-cycle burn from the runway is to look.  At zero
//  throttle on the brakes this costs nothing and moves nothing.
setRapierMode(TRUE).
WAIT 0.6.
SET ccThrustAsl TO SHIP:AVAILABLETHRUST.
SET RKT_ISP     TO measureRocketIsp().
SET RKT_ISP_ASL TO measureCurrentIsp().
setRapierMode(FALSE).               // back to air-breathing for takeoff
WAIT 0.6.
SET jetThrust TO SHIP:AVAILABLETHRUST.
SET JET_ISP   TO measureCurrentIsp().
IF JET_ISP < 500 OR JET_ISP > 20000 { SET JET_ISP TO PLAN_JET_ISP. }

// Thrust quoted at sea level is throttled by ambient pressure; at the handover
// the engines are effectively in vacuum, so scale by the Isp ratio.
SET ccThrustVac TO ccThrustAsl.
IF RKT_ISP_ASL > 0 { SET ccThrustVac TO ccThrustAsl * RKT_ISP / RKT_ISP_ASL. }

SET launchMass TO SHIP:MASS.
SET jetTwr     TO jetThrust / MAX(0.001, weightKN()).
SET ROT_BONUS  TO SHIP:VELOCITY:ORBIT:MAG * SIN(LAUNCH_HEADING).
SET lfMass0    TO coreResAmt("LiquidFuel") * LF_DENS.
SET oxMass0    TO coreResAmt("Oxidizer") * OX_DENS.
SET JET_FRAC   TO jetBurnFraction(JET_ISP).

// Payload mass, and the structural cost of this ship's own tankage: tonnes of
// tank dry mass per tonne of propellant it can hold.  Measured, not assumed -
// wet wings and Mk3 fuselages are not the same deal as a plain tank.
SET payloadMass TO 0.
FOR prt IN SHIP:PARTS {
  IF NOT CORE_UIDS:HASKEY(prt:UID) { SET payloadMass TO payloadMass + prt:MASS. }
}
// Dry mass is derived as (current mass - propellant aboard) rather than read
// from PART:DRYMASS, so this works on older kOS builds too.  A tank also
// holding monopropellant reads very slightly heavy, which biases the sizing
// advice conservative - the safe direction.
SET tankDry TO 0.
SET tankCap TO 0.
FOR prt IN CORE_TANKS {
  LOCAL wet IS 0.
  FOR res IN prt:RESOURCES {
    IF res:NAME = "LiquidFuel" {
      SET tankCap TO tankCap + res:CAPACITY * LF_DENS.
      SET wet TO wet + res:AMOUNT * LF_DENS.
    }
    IF res:NAME = "Oxidizer" {
      SET tankCap TO tankCap + res:CAPACITY * OX_DENS.
      SET wet TO wet + res:AMOUNT * OX_DENS.
    }
  }
  SET tankDry TO tankDry + MAX(0, prt:MASS - wet).
}
SET TANK_K TO 0.125.                // stock LF/Ox tanks: 8 t of fuel per t of dry
IF tankCap > 0 { SET TANK_K TO tankDry / tankCap. }

SET engCount TO 0.
SET engMass  TO 0.
SET allEngs  TO LIST().
LIST ENGINES IN allEngs.
FOR eng IN allEngs {
  IF eng:MULTIMODE { SET engCount TO engCount + 1. SET engMass TO engMass + eng:MASS. }
}
SET engMassEach TO 2.
IF engCount > 0 { SET engMassEach TO engMass / engCount. }

PRINT "------------------------------------------------------".
PRINT "VEHICLE".
PRINT "  Mass         : " + ROUND(launchMass, 1) + " t  (dry " + ROUND(SHIP:DRYMASS, 1) +
      " t, payload " + ROUND(payloadMass, 1) + " t)".
PRINT "  Jet mode     : " + ROUND(jetThrust) + " kN, Isp " + ROUND(JET_ISP) +
      " s, TWR " + ROUND(jetTwr, 2).
PRINT "  Closed cycle : " + ROUND(ccThrustVac) + " kN (vac), Isp " + ROUND(RKT_ISP) +
      " s, " + engCount + " RAPIER(s)".
PRINT "  Tankage      : " + ROUND(1 / MAX(0.001, TANK_K), 1) +
      " t of propellant per t of tank structure".
IF jetTwr < 0.35 { PRINT "  !! Low take-off TWR - expect a long roll.". }

// ---------------------------------------------------------------------------
//  FEASIBILITY CHECK
// ---------------------------------------------------------------------------
SET planSwSpd  TO PLAN_SWITCH_SPD + ROT_BONUS.        // orbital speed at handover
SET planClimb  TO dvRaiseApFrom(PLAN_SWITCH_ALT, planSwSpd, REQUESTED_APOAPSIS).
SET planCirc   TO circDvFrom(PLAN_SWITCH_ALT, REQUESTED_APOAPSIS).
SET planDeorb  TO deorbitDvFrom(REQUESTED_APOAPSIS).
SET dvRequired TO planClimb * PLAN_LOSS_FACTOR + planCirc + planDeorb + DV_MARGIN.
SET dvAtPad    TO rocketDv().                         // every paired unit aboard
SET dvHandover TO dvAtHandover(0, 0).                 // what survives the jets
SET jetLfPlan  TO jetBurnWith(0, 0, 0).
SET lfSpare    TO lfMass0 - oxMass0 * LFO_LF_RATIO / LFO_OX_RATIO.

PRINT "FEASIBILITY for a " + ROUND(REQUESTED_APOAPSIS / 1000) + " km orbit".
PRINT "  Assumes handover at " + ROUND(PLAN_SWITCH_ALT / 1000, 1) + " km / " +
      ROUND(PLAN_SWITCH_SPD) + " m/s after ~" + ROUND(PLAN_JET_DV) + " m/s of jet dV,".
PRINT "  and x" + ROUND(PLAN_LOSS_FACTOR, 2) + " losses on the rocket climb.".
PRINT "  JETS  burn ~" + ROUND(jetLfPlan, 1) + " t of LF (" +
      ROUND(jetLfPlan / LF_DENS) + " units); LF-only reserve is " +
      ROUND(MAX(0, lfSpare) / LF_DENS) + " units.".
IF lfSpare < 0 {
  // More oxidiser aboard than the liquid fuel can ever pair with.  Every one
  // of those units is mass carried to orbit and back for nothing.
  SET deadOx TO -lfSpare * LFO_OX_RATIO / LFO_LF_RATIO.       // tonnes
  PRINT "        !! " + ROUND(deadOx / OX_DENS) + " units of oxidizer (" +
        ROUND(deadOx, 1) + " t) can never be burned - there is not".
  PRINT "           enough LF to pair with it. Dead mass; drain or rebalance it.".
}
IF lfSpare < jetLfPlan {
  PRINT "        !! The jets will eat " + ROUND((jetLfPlan - MAX(0, lfSpare)) / LF_DENS) +
        " units of PAIRED LF - that is rocket dV being spent as jet fuel.".
}
PRINT "  NEED  climb " + ROUND(planClimb * PLAN_LOSS_FACTOR) + " + circ " +
      ROUND(planCirc) + " + deorbit " + ROUND(planDeorb) + " + margin " +
      ROUND(DV_MARGIN) + "  =  " + ROUND(dvRequired) + " m/s".
// Note the handover figure can legitimately come out *above* the pad figure:
// burning an LF-only reserve lightens the ship without touching a single
// paired unit, which is exactly what that reserve is for.
PRINT "  HAVE  " + ROUND(dvHandover) + " m/s at the handover (tanks hold " +
      ROUND(dvAtPad) + " m/s of pairs on the pad)".
PRINT "  TWR   " + ROUND(twrAtHandover(0, 0, 0), 2) + " closed cycle at the handover".

IF dvHandover >= dvRequired {
  PRINT "  => GO. " + ROUND(dvHandover - dvRequired) + " m/s in hand.".
  IF twrAtHandover(0, 0, 0) < PLAN_TWR_MIN {
    PRINT "  !! ...but closed-cycle TWR is below " + ROUND(PLAN_TWR_MIN, 2) +
          ". The climb will be slow and lossy; consider another RAPIER.".
  }
} ELSE {
  SET dvShort TO dvRequired - dvHandover.
  PRINT "======================================================".
  PRINT "!! NOT ENOUGH dV - SHORT BY " + ROUND(dvShort) + " m/s".
  PRINT "   (needs " + ROUND(dvRequired) + ", has " + ROUND(dvHandover) + " at handover)".
  PRINT "   This ship is not expected to make " + ROUND(REQUESTED_APOAPSIS / 1000) +
        " km with deorbit fuel left.".
  PRINT "".
  PRINT "   TO CLOSE THE GAP:".

  // --- cheapest fix first: is the LF/Ox split starving the jets? ------------
  //  Adding liquid fuel alone costs a fraction of what balanced LF/Ox costs,
  //  because every tonne of it goes to the jets at Isp ~3200 instead of being
  //  hauled to orbit.  Worth saying before quoting the big number.
  SET fixLf TO solveLfTopUp().
  IF fixLf > 0.05 {
    SET dvAfterLf TO dvHandoverSplit(fixLf, 0, 0).
    PRINT "   * LIQUID FUEL FIRST. Your jets are eating " +
          ROUND((jetLfPlan - lfSpare) / LF_DENS) + " units of paired LF.".
    PRINT "     +" + ROUND(fixLf, 1) + " t of LF ONLY (" + ROUND(fixLf / LF_DENS) +
          " units, +" + ROUND(fixLf * TANK_K, 1) + " t tank) feeds them from a".
    PRINT "     proper reserve and lifts the handover to " + ROUND(dvAfterLf) +
          " m/s (+" + ROUND(dvAfterLf - dvHandover) + " m/s).".
    PRINT "     Compare that against the balanced figure below before you add".
    PRINT "     any oxidizer - LF burns at jet Isp, oxidizer is just cargo.".
  }

  // --- more propellant, converged against the TWR it costs ------------------
  SET addEngN TO 0.
  SET fixProp TO -1.
  FROM { LOCAL iter IS 0. } UNTIL iter >= 10 STEP { SET iter TO iter + 1. } DO {
    SET fixProp TO solveProp(dvRequired, addEngN * engMassEach).
    IF fixProp < 0 { BREAK. }
    IF twrAtHandover(fixProp, addEngN * engMassEach, addEngN) >= PLAN_TWR_MIN { BREAK. }
    SET addEngN TO addEngN + 1.
  }

  IF fixProp < 0 {
    SET dvCeiling TO dvAtHandover(20000, 0).
    PRINT "   * FUEL ALONE CANNOT DO IT. With tankage this heavy the mass ratio".
    PRINT "     tops out around " + ROUND(dvCeiling) + " m/s however many tanks you bolt on,".
    PRINT "     and you need " + ROUND(dvRequired) + " m/s. Cut dry mass or payload instead:".
    PRINT "     lighter tanks, fewer parts, or less cargo.".
  } ELSE {
    SET fixDry  TO fixProp * TANK_K.
    SET fixMass TO fixProp + fixDry + addEngN * engMassEach.
    PRINT "   * FUEL: +" + ROUND(fixProp, 1) + " t of LF/Ox (" +
          ROUND(fixProp * LF_FRAC / LF_DENS) + " LF + " +
          ROUND(fixProp * OX_FRAC / OX_DENS) + " Ox)".
    PRINT "     plus ~" + ROUND(fixDry, 1) + " t of tank structure to hold it.".
    IF addEngN > 0 {
      PRINT "     ...and +" + addEngN + " RAPIER(s) (" + ROUND(addEngN * engMassEach, 1) +
            " t) to keep TWR at " + ROUND(PLAN_TWR_MIN, 2) + " - the fuel figure".
      PRINT "     above already pays for hauling them.".
    }
    PRINT "     => launch mass " + ROUND(launchMass, 1) + " -> " +
          ROUND(launchMass + fixMass, 1) + " t.".
    IF twrAtHandover(fixProp, addEngN * engMassEach, addEngN) < PLAN_TWR_MIN {
      PRINT "     !! Even then TWR is only " +
            ROUND(twrAtHandover(fixProp, addEngN * engMassEach, addEngN), 2) +
            " - this design is chasing its own tail. Cut dry mass.".
    }
  }

  // --- or carry less ---------------------------------------------------------
  SET fixCut TO solveMassCut(dvRequired).
  IF fixCut > 0 {
    PRINT "   * PAYLOAD: fly " + ROUND(fixCut, 1) + " t lighter (of " +
          ROUND(payloadMass, 1) + " t aboard) and the fuel you have is enough.".
  } ELSE IF payloadMass > 0 {
    PRINT "   * PAYLOAD: even flying empty would not close the gap.".
  }

  // --- or accept a lower orbit ----------------------------------------------
  PRINT "   * ORBIT: lower REQUESTED_APOAPSIS, or raise PLAN_SWITCH_SPD if this".
  PRINT "     airframe really does hand over faster than " + ROUND(PLAN_SWITCH_SPD) + " m/s.".
  PRINT "".
  PRINT "   Launching anyway - the in-flight budget will settle for what it can.".
  PRINT "======================================================".
  IF PREFLIGHT_HOLD > 0 { WAIT PREFLIGHT_HOLD. }
}
PRINT "------------------------------------------------------".

SET thrCmd TO 1.
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
// Calibration feedback for the pre-flight model: every gram the ship has lost
// so far went out of the jets, so the handover state and the jet phase's true
// dV equivalent can both be read straight off the flight.  Trim
// PLAN_SWITCH_ALT / PLAN_SWITCH_SPD / PLAN_JET_DV to these numbers and the
// next pre-flight check will be about this airframe rather than a generic one.
SET jetLfUsed TO launchMass - SHIP:MASS.
SET jetDvReal TO JET_ISP * G0 * LN(launchMass / MAX(0.001, SHIP:MASS)).
PRINT "  Jet phase used " + ROUND(jetLfUsed, 1) + " t of LF = " +
      ROUND(jetDvReal) + " m/s of jet dV (planned " + ROUND(PLAN_JET_DV) + ").".
PRINT "  Handover " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km / " + ROUND(SHIP:AIRSPEED) +
      " m/s (planned " + ROUND(PLAN_SWITCH_ALT / 1000, 1) + " / " +
      ROUND(PLAN_SWITCH_SPD) + ").".

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
