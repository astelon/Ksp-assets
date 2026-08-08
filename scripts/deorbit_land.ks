// ============================================================================
//  deorbit_land.ks  --  Deorbit, reentry, and guided glide back to the KSC
//                       runway for the Mk3 Titan Heavy SSTO.
// ----------------------------------------------------------------------------
//  From a circular Kerbin orbit this script:
//    1. Orients retrograde, *predicts* the moment the ground track reaches the
//       deorbit point, plants a maneuver node there, time-warps to it, and
//       executes the retrograde burn, lowering periapsis to ~32 km.
//    2. Turns the nose *prograde* for entry, solves the time left to the
//       atmospheric interface, and warps the coast away if it is worth warping.
//    3. Flies a high angle-of-attack reentry to bleed orbital energy while
//       steering the nose toward the runway.
//    4. Transitions to an energy-managed glide, homing on a final approach fix
//       on the extended runway centreline.
//    5. Captures the runway heading, flies a glideslope, flares, drops the
//       gear, touches down, and brakes to a stop on the centreline.
//
//  ATTITUDE CONVENTION - every atmospheric command in this script is given
//  relative to the *airstream*, not to the horizon.  `aeroSteer(hdg, aoa, bank)`
//  puts the nose `aoa` degrees above the surface velocity vector, so the angle
//  of attack is what is actually held no matter how steeply the ship is
//  descending.  Commanding an absolute pitch instead is what makes a spaceplane
//  stall: -3 degrees "nose down" while falling at -25 degrees is +22 degrees of
//  AoA, well past the wing.
//
//  ENVELOPE - the convention above is not enough on its own, because a guidance
//  law can still ask for an attitude the ship has no speed to fly to.  Every
//  phase therefore hands its heading and AoA demand to `setNav()`, which folds
//  the heading into the airstream through a speed schedule (25 degrees of lean
//  at Mach 4, opening to 60 once subsonic), limits bank by the same schedule,
//  and takes bank to zero at the stall.  Above that sits one rule that outranks
//  navigation entirely: with less than STALL_Q of dynamic pressure under the
//  wing, or with the nose more than DEPART_AOA off the airstream, the ship is
//  not flying and `stallRecover()` runs until it is again.  The three ways this
//  autopilot used to lose a ship - handing a Mach 3 reentry to a bearing-chasing
//  glide law, S-turning at the stall, and an absolute pitch floor that quietly
//  ate the AoA guard in a steep descent - are all failures of the envelope, not
//  of the navigation.  See docs/REENTRY_REVIEW.md.
//
//  SPEED - "the glide speed" is not a speed.  A wing flies on dynamic pressure,
//  and 160 m/s at 12 km is a third of the air 160 m/s has over the runway: a
//  fixed airspeed target is a stall order everywhere except the altitude it was
//  measured at.  Every phase below the entry therefore aims at GLIDE_Q of
//  dynamic pressure and lets the speed that implies fall out of the air it is
//  actually in - 110 m/s at sea level, 160 at 6 km, 250 at 12 km, 450 at 20 km.
//  The glide that spun, stalled, recovered and spun again all the way down from
//  22 km was flying the number, not the wing.  See docs/GLIDE_REVIEW.md.
//
//  PATH - a speed is not a flight path.  Angle of attack says what the ship is
//  flying at, never where it is going, so a nose trimmed for a speed and nothing
//  else leaves the trajectory entirely free: the same command holds a 45-degree
//  dive and holds level flight, and a ship handed it does both, in turn, forever.
//  The glide therefore commands a *sink rate* - taken from the glide ratio the
//  energy error is asking for, LD_DUMP steep when high and LD_STRETCH shallow
//  when short - and lets the speed error trim it.  The glide that dived from
//  12 km to 8 km at 45 degrees and then cruised level at 8 km for 45 km, while
//  the profile it was tracking sank away underneath it, was flying a speed.
//
//  POWER - a glider holds its speed with the nose and its flight path with
//  drag.  Under thrust that law is backwards, and flying it anyway is what
//  makes an autopilot climb at full throttle until the tanks are dry and then
//  dive into the sea: the speed the nose is holding is *already* the target, so
//  every newton of thrust goes into altitude.  With the jets lit this script
//  swaps the two - the throttle holds the speed, the nose holds the flight path
//  - and thrust is spent only on a real energy deficit, never on a ship that is
//  merely slow and high.  The answer to slow and high is to point the nose down.
//
//  GROUND - every number in the energy plan is referenced to the runway, and
//  height above the runway is not height above the ground.  The approach to the
//  KSC from the west crosses ridges over 2 km high, so a descent that is exactly
//  on profile flies straight through them, and a stall recovery given 600 m of
//  "height to dive in" has none.  Radar altitude is the only thing that knows
//  the ground is there: it bounds the commanded sink (TERRAIN_FLOOR), it floors
//  every stall recovery, and below TERRAIN_ABORT it ends the glide outright.
//  It outranks the plan, because the plan cannot see.
//
//  IMPORTANT - this is a *guidance* script, not a precision lander.  A
//  spaceplane's huge glide range absorbs deorbit-timing error, but the
//  constants below (deorbit lead angle, glide AoA, planned L/D, flare height)
//  are the things you will most likely trim for your exact mass / drag.  They
//  are all grouped at the top.
//
//  Run with:   RUN deorbit_land.
// ============================================================================

CLEARSCREEN.

// The atmospheric autopilot re-derives its steering command from the flight
// path angle on every physics tick, so give the interpreter room to do it.  The
// stock 200 instructions per update is enough to run the loop but not enough to
// keep it crisp on a heavy airframe.
SET CONFIG:IPU TO 500.

// --- KSC runway (09 threshold, west end; land heading ~090) -----------------
SET KSC_RWY        TO LATLNG(-0.0485, -74.726).
SET RUNWAY_HDG     TO 90.

// --- Tunables: deorbit ------------------------------------------------------
SET DEORBIT_PE     TO 32000.     // target periapsis for the deorbit burn (m)

// HOW FAR OUT TO HIT THE AIR.  This, not the burn angle, is the number the
// entry is really about: the ground range from the atmospheric interface to the
// KSC is the distance the ship has to bleed 2 100 m/s across, and whether it is
// enough is a question about the airframe, not about the orbit.
//
// The two reference flights bracket the answer.  Flown clean the ship made
// ~880 km from the interface (it crossed the field from 552 km out still at
// 26 km and 959 m/s - an energy height of 73 km, worth another 329 km of glide).
// Flown at 40 deg of alpha with the boards out the whole way it made 530 km.
// 800 km therefore sits mid-band: about 80 km of surplus for the entry to dump
// if it flies clean, and about 270 km of dumping authority in hand if it does
// not.  Both are inside what the energy loop can steer with, which is the point
// - a schedule the ship cannot correct from either side is not a schedule.
//
// Trim rule: crossing the field with energy to spare means the ship out-flew
// this number, so raise it; landing short means lower it.  100 km is 9.5 deg of
// Kerbin ground track, so the steps are big and one flight tells you which way.
SET ENTRY_RANGE    TO 800000.    // ground range from the interface to the KSC (m)

// Ground-track angle before the KSC at which the burn is started.  Left at 0 it
// is *solved* from ENTRY_RANGE and the coast arc the deorbit ellipse will sweep
// between the burn and the interface (see coastArcDeg) - which is the honest
// form of the number, because the arc depends on the parking altitude and on
// DEORBIT_PE and a hand-trimmed lead angle silently stops being right when
// either of those changes.  Set it non-zero to override the solution.
// For the reference case - 100 km circular, periapsis 32 km - the solver returns
// 149.5 deg for 800 km of entry range, against the 149 deg that was trimmed by
// hand from flight data in docs/REENTRY_REVIEW.md: the same number to within
// half a degree, arrived at from the geometry instead of from a crash.  It is
// also the reason a hand-trimmed constant is dangerous - the coast arc is 63 deg
// from a 90 km parking orbit and 85 from a 120 km one, so a lead angle trimmed
// at one altitude is 230 km wrong at the other.
SET DEORBIT_LEAD   TO 0.         // ground-track angle before KSC to start burn (deg)
SET WARP_LEAD      TO 45.        // come out of warp this long before the burn (s)
SET SCAN_STEP      TO 20.        // coarse step when hunting the deorbit point (s)
SET SCAN_ORBITS    TO 2.         // how many orbits ahead to search for it
SET ALIGN_TOL      TO 5.         // steering error accepted before burning (deg)
SET ISP_FALLBACK   TO 305.       // RAPIER closed-cycle Isp, if none can be read (s)

// --- Tunables: the coast down to the atmosphere -----------------------------
SET ENTRY_ALT      TO 70000.     // atmospheric interface (m) - read from the body below
SET COAST_WARP_MIN TO 60.        // only warp the coast when it is longer than this (s)
SET COAST_LEAD     TO 30.        // come out of warp this long before the interface (s)

// --- Tunables: reentry ------------------------------------------------------
SET REENTRY_AOA    TO 40.        // AoA held while hypersonic (deg above the airstream)
SET ENTRY_AOA_HI   TO 2000.      // airspeed at/above which the full AoA is held (m/s)
SET ENTRY_AOA_LO   TO 500.       // airspeed at/below which AoA has tapered to glide AoA
SET ENTRY_AOA_MIN  TO 12.        // least alpha the entry will fly when it is short (deg)
SET ENTRY_YAW_MAX  TO 25.        // furthest the nose may be yawed off the airstream (deg)
SET ENTRY_BANK_MAX TO 20.        // bank allowed while supersonic (deg)
SET ENTRY_END_SPD  TO 650.       // hand over to the glide at this airspeed (m/s) ...
SET ENTRY_FLOOR    TO 15000.     // ... or at this altitude, as a backstop only (m)
SET ENTRY_HOLD_RNG TO 60000.     // stop chasing the bearing inside this ground range (m)
SET ENTRY_STURN_T  TO 25.        // seconds between entry bank reversals
SET ENTRY_LD       TO 3.0.       // ground range flown per metre of energy height, hypersonic
SET ENERGY_LONG    TO 1.15.      // range capability / range to go above which we are long
SET ENERGY_SHORT   TO 1.00.      // ... and below which we are short and must stretch
SET SKIP_VS        TO 15.        // climbing faster than this = ballooning, unload AoA
SET RCS_Q_OFF      TO 0.02.      // dynamic pressure (atm) at which RCS is no longer needed
SET ENTRY_HOLD_TOL TO 25.        // actual AoA this far past the command = departed (deg)
SET ENTRY_TRIM_TOL TO 12.        // ... and this far past it = stop asking for so much (deg)
SET ENTRY_AOA_STEP TO 2.         // AoA given up each time the ship cannot hold the command
SET LD_SAMPLE      TO 5.         // seconds between measurements of the L/D actually flown
SET LD_FILTER      TO 0.35.      // weight given to each new L/D sample

// --- Tunables: glide --------------------------------------------------------
// The glide flies a dynamic pressure, not an airspeed: see the SPEED note in the
// header.  GLIDE_Q is the pressure the wing wants under it, and the two speed
// limits are only the ends of the range the answer is allowed to come out in.
SET GLIDE_Q        TO 0.07.      // dynamic pressure the glide is flown at (atm)
SET GLIDE_SPD_MIN  TO 110.       // slowest airspeed the glide will ever aim for (m/s)
SET GLIDE_SPD_MAX  TO 450.       // ... and the fastest
SET GLIDE_AOA      TO 6.         // trim AoA at the target glide speed (deg)
SET GLIDE_AOA_MAX  TO 14.        // hard stall guard - never command more than this
SET FAST_AOA_MAX   TO 8.         // tighter AoA ceiling while still supersonic (deg)
SET AOA_CMD_MIN    TO -4.        // the wing may be unloaded below zero lift to regain speed
SET AOA_PER_MS     TO 0.05.      // AoA added per m/s of overspeed
SET BANK_MAX       TO 30.        // bank angle used to turn (deg)
SET BANK_PER_DEG   TO 1.5.       // bank commanded per degree of heading error
SET PLAN_LD        TO 4.5.       // glide ratio the energy plan assumes (m fwd per m down)
SET HIGH_MARGIN    TO 1500.      // metres above the profile before dumping energy
SET LOW_MARGIN     TO 400.       // metres below the profile before calling it short
SET STURN_OFFSET   TO 40.        // heading offset flown when high (deg)
SET STURN_PERIOD   TO 18.        // seconds between S-turn reversals
SET HOLD_RADIUS    TO 8000.      // inside this range, excess height is spiralled off (m)
// The glide ratios the nose is asked to *fly*, either side of the plan.  A
// glider trimmed for a speed and nothing else does not track a profile - it
// phugoids, because AoA alone says nothing about where the ship is going.  The
// flight path is therefore commanded as a sink rate taken from these, and the
// speed error only trims it.  See docs/GLIDE_REVIEW.md.
SET LD_DUMP        TO 1.5.       // glide ratio flown when HIGH_MARGIN above the profile
SET LD_STRETCH     TO 6.0.       // ... and when LOW_MARGIN below it
SET GLIDE_VS_GAIN  TO 0.15.      // AoA per m/s of vertical-speed error in the glide
SET GLIDE_VS_AUTH  TO 6.         // most AoA the vertical-speed term may add or take (deg)
SET BOARD_MARGIN   TO 500.       // energy height above the profile that puts the boards out
SET BOARD_OFF      TO 150.       // ... and below which they come back in

// --- Tunables: powered flight -----------------------------------------------
// With the jets lit the throttle holds the speed; the nose is already holding
// the flight path and goes on holding it, powered or not, so there is no second
// vertical-speed law here and no climb to schedule.  A ship that is short of the
// runway cannot climb its way there, and thrust spent on altitude at 110 m/s
// buys neither range nor speed.
SET THROT_PER_MS   TO 0.02.      // throttle per m/s of airspeed error under power
SET THROT_MIN_PWR  TO 0.25.      // spool-up floor, held only while genuinely short

// --- Tunables: terrain clearance --------------------------------------------
// Height above the *runway* is not height above the ground.  Everything the
// energy plan says is referenced to a sea-level runway, and the approach to the
// KSC from the west crosses ridges over 2 km high: a descent that is perfectly
// on profile flies straight through them.  These are the only numbers in the
// script that know the ground is there, so they outrank the plan.
//
// Two layers, and they answer different questions.  TERRAIN_* is reactive - it
// reads ALT:RADAR, the ground directly underneath, and is the last guard before
// a hillside.  MTN_* is predictive: it scans the ground all the way to the fix
// and raises a *deck* the glide is not allowed to descend through while the high
// ground is still ahead.  Only the second one can save a glider, because a
// glider does not climb over a ridge - it has to arrive already above it, and by
// the time the radar sees the face there is nothing left to do about it.
SET TERRAIN_FLOOR  TO 500.       // radar altitude the glide will not sink below (m)
SET TERRAIN_TAU    TO 12.        // seconds over which the sink is bled off at a floor
SET TERRAIN_CLIMB  TO 8.         // climb commanded when already below a floor (m/s)
SET TERRAIN_ABORT  TO 250.       // radar altitude at which the glide gives up and lands (m)
SET MTN_CLEAR      TO 350.       // clearance held over the highest ground ahead (m)
SET MTN_SAMPLES    TO 20.        // sample points along the track to the fix
SET MTN_SCAN_T     TO 2.         // seconds between terrain scans

// --- Tunables: the flight envelope ------------------------------------------
// How far the nose may be commanded off the airstream, and how hard the ship may
// be banked, as a function of airspeed.  This is the schedule that keeps a
// guidance law from asking a fast ship for an attitude it cannot fly: told to
// point at a bearing well off its own track at Mach 3, a spaceplane does not
// turn, it departs.
//
// This limit is *sideslip*, not turn rate, and the two are easy to confuse.
// dirFor() points the nose at the heading it is given, so a demand `hdgErr`
// degrees off the airstream puts `hdgErr` degrees of air across the fuselage
// until the velocity vector catches up.  An aeroplane turns on its bank, which
// setNav() commands from the same error and which costs no sideslip at all, so
// the yaw limit can be tight without slowing a single turn down: it bounds how
// crossed-up the ship is allowed to get while the bank does the work.
//
// 60 degrees was not a bound.  Told to S-turn 40 degrees off track at 648 m/s,
// the ship was handed 40 degrees of sideslip in one pass and departed at 58
// degrees off the airstream; the same command at 154 m/s put it 83 degrees off.
// Both of those are inside a 60-degree allowance and neither is flight.
SET YAW_SPD_HI     TO 1200.      // at/above this airspeed the nose is pinned to ENTRY_YAW_MAX
SET YAW_SPD_LO     TO 250.       // at/below it may lean YAW_SUB_MAX off the airstream
SET YAW_SUB_MAX    TO 15.        // sideslip allowed once subsonic (deg) - the bank turns
SET FAST_SPD       TO 400.       // above this the ship is flown as a fast ship, not a glider
// Whether the wing is flying is a question about *dynamic pressure*, not about
// airspeed: 76 m/s at 9 km and 76 m/s over the runway are not the same flight
// state, and any airspeed threshold high enough to catch the first is above the
// speed this ship is meant to fly its approach at.  The ship needs ~0.07 atm to
// hold level flight - which is 110 m/s at sea level and 160 m/s at 6 km, the two
// speeds this script already flies - so half of that is the wing letting go.
// Note RCS_Q_OFF, arrived at separately, says the same thing about the control
// surfaces at almost the same number.
SET STALL_Q        TO 0.025.     // below this dynamic pressure (atm) the wing is gone
SET MANEUVER_Q     TO 0.045.     // full bank and energy-dumping authority only above this
SET RECOVER_Q      TO 0.050.     // fly a stall recovery until back above this
SET DEPART_AOA     TO 30.        // nose this far off the airstream in the glide = departed
SET RECOVER_AOA    TO 2.         // AoA held while diving out of a stall (deg)
// A recovery that ends the moment the pressure gauge crosses back over its
// threshold hands the guidance law a ship that is still pointing the wrong way
// at exactly its stall speed, and it departs again on the first turn command.
// Recovery ends when the wing has air *and* the nose is following the airstream,
// and the turn demand is held off for STEADY_TIME afterwards while the ship
// builds a margin.  Six cycles of recover-and-depart is what this costs.
SET RECOVER_NOSE   TO 20.        // nose back within this of the airstream = flying (deg)
SET RECOVER_MAX_T  TO 45.        // longest a single recovery attempt runs before re-thinking
SET STEADY_TIME    TO 6.         // wings-level seconds granted after a recovery
// Absolute attitude envelope, applied to every atmospheric command as a last
// resort.  Both limits must stay clear of the attitudes the AoA law legitimately
// asks for, or they quietly become the very thing this script exists to avoid -
// an absolute pitch command.  PITCH_MIN in particular has to allow a nose-down
// recovery dive along a near-vertical flight path; its only job is to stop the
// nose being commanded straight at the ground or straight at the sky.
SET PITCH_MAX      TO 45.        // absolute nose-up limit (deg)
SET PITCH_MIN      TO -85.       // absolute nose-down limit (deg)

// --- Tunables: approach & landing -------------------------------------------
SET FINAL_DIST     TO 9000.      // final approach fix, this far short of the threshold (m)
SET GLIDESLOPE     TO 5.         // final-approach descent angle (deg)
SET APPR_SPEED     TO 110.       // target airspeed on final (m/s)
SET APPR_AOA       TO 6.         // trim AoA on final (deg)
SET LOC_GAIN       TO 2.5.       // heading correction per degree of localiser error
SET LOC_MAX        TO 25.        // largest intercept angle onto the centreline (deg)
SET FAF_CAPTURE    TO 2500.      // ground range to the fix that counts as captured (m)
SET GLIDE_FLOOR    TO 600.       // AGL at which the glide must hand over, wherever it is (m)
SET DIVERT_RANGE   TO 12000.     // farther from the runway than this at the floor: land here
SET FLARE_ALT      TO 28.        // radar altitude to begin the flare (m)
SET FLARE_PITCH    TO 7.         // absolute nose-up attitude in the flare (deg)
SET TOUCHDOWN_ALT  TO 3.         // radar altitude considered "on the wheels" (m)
SET USE_JETS_SHORT TO TRUE.      // relight the jets rather than land short
SET JET_ARM_ALT    TO 12000.     // only below this altitude is a jet save attempted (m)
SET JET_MIN_LF     TO 5.         // never light the jets on less LiquidFuel than this

// ---------------------------------------------------------------------------
//  Body / physical constants (cached once)
// ---------------------------------------------------------------------------
SET G0            TO 9.80665.
SET BODY_MU       TO SHIP:BODY:MU.
SET BODY_R        TO SHIP:BODY:RADIUS.
SET BODY_ROT_RATE TO 360 / SHIP:BODY:ROTATIONPERIOD.   // deg/s, eastward
// Read the interface height from the body rather than trusting Kerbin's 70 km:
// the deorbit lead is solved from the coast down to it, so it has to be right
// before the burn is planned, not just before the ship arrives there.
IF SHIP:BODY:ATM:EXISTS { SET ENTRY_ALT TO SHIP:BODY:ATM:HEIGHT. }

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------
// NOTE: no identifier here may be a bare single letter - kOS refuses to compile
// a variable that would hide a built-in function such as R(), V() or Q().
FUNCTION resAmt {
  PARAMETER rname.
  LOCAL total IS 0.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = rname { SET total TO total + res:AMOUNT. }
  }
  RETURN total.
}

FUNCTION normAng {                   // wrap an angle into (-180, 180]
  PARAMETER angIn.
  LOCAL ang IS angIn.
  UNTIL ang <= 180 { SET ang TO ang - 360. }
  UNTIL ang > -180 { SET ang TO ang + 360. }
  RETURN ang.
}

FUNCTION clampVal {
  PARAMETER loV, hiV, valIn.
  RETURN MAX(loV, MIN(hiV, valIn)).
}

// Signed ground-track angle (deg) from the sub-point we will be over at
// universal time t, round to the KSC.  POSITIONAT predicts the future position,
// but GEOPOSITIONOF reads it back in the body's *present* rotation frame, so the
// KSC's longitude is advanced by the planet's own spin over the same interval to
// keep the two comparable.  Positive means the KSC is still ahead of us.
FUNCTION trackAngleAt {
  PARAMETER tUT.
  LOCAL dt  IS tUT - TIME:SECONDS.
  LOCAL sub IS SHIP:BODY:GEOPOSITIONOF(POSITIONAT(SHIP, tUT)).
  RETURN normAng(KSC_RWY:LNG + BODY_ROT_RATE * dt - sub:LNG).
}

FUNCTION trackAngleToKSC { RETURN trackAngleAt(TIME:SECONDS). }

// First universal time at or after tFrom at which the ground-track angle passes
// through DEORBIT_LEAD.  Coarse-scans for a sign change of (angle - lead) and
// then bisects it.  The wrap guard rejects the +-180 discontinuity, which is a
// sign change but not a crossing.  Returns 0 if the span holds no crossing.
FUNCTION deorbitTimeAfter {
  PARAMETER tFrom.
  LOCAL span IS SHIP:OBT:PERIOD * SCAN_ORBITS.
  LOCAL tA   IS tFrom.
  LOCAL fa   IS normAng(trackAngleAt(tA) - DEORBIT_LEAD).
  LOCAL tB   IS tFrom.
  UNTIL tB >= tFrom + span {
    SET tB TO MIN(tB + SCAN_STEP, tFrom + span).
    LOCAL fb IS normAng(trackAngleAt(tB) - DEORBIT_LEAD).
    IF fa * fb <= 0 AND ABS(fa - fb) < 90 {
      LOCAL lo IS tA.  LOCAL hi IS tB.  LOCAL flo IS fa.
      FROM { LOCAL iter IS 0. } UNTIL iter >= 25 STEP { SET iter TO iter + 1. } DO {
        LOCAL mid IS (lo + hi) / 2.
        LOCAL fm  IS normAng(trackAngleAt(mid) - DEORBIT_LEAD).
        IF flo * fm <= 0 { SET hi TO mid. }
        ELSE { SET lo TO mid. SET flo TO fm. }
      }
      RETURN (lo + hi) / 2.
    }
    SET tA TO tB.  SET fa TO fb.
  }
  RETURN 0.
}

// Retrograde dV needed at universal time t to drop periapsis to DEORBIT_PE,
// priced off the orbit we will actually be on at that moment.
FUNCTION deorbitDvAt {
  PARAMETER tUT.
  LOCAL rAt IS BODY_R + SHIP:BODY:ALTITUDEOF(POSITIONAT(SHIP, tUT)).
  LOCAL rP  IS BODY_R + DEORBIT_PE.
  IF rP >= rAt { RETURN 0. }
  LOCAL smaT IS (rAt + rP) / 2.
  RETURN MAX(0, VELOCITYAT(SHIP, tUT):ORBIT:MAG - SQRT(BODY_MU * (2 / rAt - 1 / smaT))).
}

// Ground-track angle swept between the deorbit burn and the atmospheric
// interface.  The burn is retrograde and lowers the periapsis, so the burn point
// *is* the apoapsis of the ellipse that results, and the whole geometry is fixed
// by two radii: where the burn happens, and the periapsis it aims at.  Solve the
// eccentric anomaly at the interface radius, turn it into a true anomaly for the
// arc and into a mean anomaly for the time, and give the planet back the
// rotation it performs underneath the ship while it coasts.
//
// This is the half of the deorbit lead that has nothing to do with the entry:
// on a 100 km orbit aiming at a 32 km periapsis it is 73.1 degrees, and it is
// 7.3 minutes of coast.  Add the range you want to hit the air at and you have
// the burn angle - which is how DEORBIT_LEAD is now arrived at instead of being
// trimmed by hand against one flight's mass and drag.
FUNCTION coastArcDeg {
  PARAMETER rBurn.
  LOCAL rPe IS BODY_R + DEORBIT_PE.
  LOCAL rIf IS BODY_R + ENTRY_ALT.
  IF rBurn <= rIf OR rPe >= rIf { RETURN 0. }
  LOCAL sma IS (rBurn + rPe) / 2.
  LOCAL ecc IS (rBurn - rPe) / (rBurn + rPe).
  IF ecc < 0.000001 { RETURN 0. }
  // r = a(1 - e cos E), measured from periapsis: E = 0 at Pe, 180 at Ap.
  LOCAL eAnom IS ARCCOS(clampVal(-1, 1, (sma - rIf) / (sma * ecc))).
  LOCAL nu    IS 2 * ARCTAN2(SQRT(1 + ecc) * SIN(eAnom / 2),
                             SQRT(1 - ecc) * COS(eAnom / 2)).
  LOCAL mAnom IS eAnom - ecc * SIN(eAnom) * CONSTANT:RADTODEG.
  LOCAL nDeg  IS SQRT(BODY_MU / (sma ^ 3)) * CONSTANT:RADTODEG.   // deg/s
  IF nDeg <= 0 { RETURN 0. }
  LOCAL dtCoast IS (180 - mAnom) / nDeg.
  RETURN (180 - nu) - BODY_ROT_RATE * dtCoast.
}

// The burn angle that puts the interface ENTRY_RANGE short of the KSC.
FUNCTION deorbitLeadFor {
  PARAMETER rBurn.
  RETURN ENTRY_RANGE / BODY_R * CONSTANT:RADTODEG + coastArcDeg(rBurn).
}

// Predicted altitude above the datum at universal time t.
FUNCTION altAt {
  PARAMETER tUT.
  RETURN SHIP:BODY:ALTITUDEOF(POSITIONAT(SHIP, tUT)).
}

// Seconds from now until the coast falls through `altWant`, or -1 if it never
// does.  On the post-burn ellipse altitude decreases monotonically from the burn
// point to periapsis, so the crossing is found by bisecting that span - no
// integration, and no assumption about where in the orbit the burn happened
// beyond the one the geometry guarantees.  Returns 0 if we are already below.
FUNCTION coastTimeTo {
  PARAMETER altWant.
  IF SHIP:ALTITUDE <= altWant { RETURN 0. }
  LOCAL tPe IS TIME:SECONDS + ETA:PERIAPSIS.
  IF altAt(tPe) > altWant { RETURN -1. }        // periapsis is above it: no entry
  LOCAL lo IS TIME:SECONDS.
  LOCAL hi IS tPe.
  FROM { LOCAL iter IS 0. } UNTIL iter >= 30 STEP { SET iter TO iter + 1. } DO {
    LOCAL mid IS (lo + hi) / 2.
    IF altAt(mid) > altWant { SET lo TO mid. } ELSE { SET hi TO mid. }
  }
  RETURN (lo + hi) / 2 - TIME:SECONDS.
}

// Thrust-weighted vacuum Isp of the engines that will do the burn.
FUNCTION rocketIsp {
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
  RETURN ISP_FALLBACK.
}

// How long dV takes at full throttle, from the rocket equation run backwards.
FUNCTION burnTimeFor {
  PARAMETER dv.
  LOCAL thr IS SHIP:AVAILABLETHRUST.
  IF dv <= 0 OR thr <= 0 { RETURN 0. }
  LOCAL ve IS rocketIsp() * G0.
  RETURN SHIP:MASS * ve * (1 - CONSTANT:E ^ (-dv / ve)) / thr.
}

// Steering target for the deorbit burn.  A maneuver node's burn vector shrinks
// to nothing as the burn is flown and its *direction* goes to noise with it, so
// once there is almost nothing left to burn we hold plain retrograde instead of
// letting the ship chase a numerically meaningless vector around the sky.
FUNCTION burnSteer {
  IF HASNODE {
    IF NEXTNODE:DELTAV:MAG > 5 { RETURN NEXTNODE:BURNVECTOR. }
  }
  RETURN SHIP:RETROGRADE:VECTOR.
}

// Force every multimode engine into closed cycle: in vacuum the air-breathing
// mode reports no thrust, which would poison the node's burn-time estimate.
FUNCTION goClosedCycle {
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  FOR eng IN engs {
    IF eng:MULTIMODE {
      SET eng:AUTOSWITCH TO FALSE.
      IF eng:MODE = "AirBreathing" { eng:TOGGLEMODE(). }
    }
  }
}

// Put the multimode engines back on air so that any thrust asked for during the
// glide is jet thrust.  goClosedCycle() also cleared AUTOSWITCH, so without this
// the RAPIERs would still be burning oxidiser down at 5 km.
FUNCTION goAirBreathing {
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  FOR eng IN engs {
    IF eng:MULTIMODE {
      SET eng:AUTOSWITCH TO FALSE.
      IF eng:MODE = "ClosedCycle" { eng:TOGGLEMODE(). }
    }
  }
}

// ---------------------------------------------------------------------------
//  Atmospheric guidance geometry
//
//  The local horizontal frame: north lies along SHIP:NORTH flattened against
//  local up, and east is up x north - the same construction the navball uses,
//  so a "compass heading" here means what it means on the navball.
// ---------------------------------------------------------------------------
FUNCTION eastVec { RETURN VCRS(SHIP:UP:VECTOR, VXCL(SHIP:UP:VECTOR, SHIP:NORTH:VECTOR)). }

// Compass heading (0-360) of any vector, its vertical component discarded.
FUNCTION compassOf {
  PARAMETER vecIn.
  LOCAL upv  IS SHIP:UP:VECTOR.
  LOCAL flat IS VXCL(upv, vecIn).
  IF flat:MAG < 0.001 { RETURN 0. }
  LOCAL ang IS ARCTAN2(VDOT(eastVec():NORMALIZED, flat),
                       VDOT(VXCL(upv, SHIP:NORTH:VECTOR):NORMALIZED, flat)).
  IF ang < 0 { RETURN ang + 360. }
  RETURN ang.
}

// Flight path angle: how far the surface velocity vector points above the local
// horizon, in degrees.  Negative while descending.
FUNCTION fpaNow {
  LOCAL svel IS SHIP:VELOCITY:SURFACE.
  IF svel:MAG < 5 { RETURN 0. }
  RETURN 90 - VANG(SHIP:UP:VECTOR, svel).
}

// A steering Direction from compass heading, absolute pitch above the horizon,
// and bank angle (positive = right wing down).  Built vector-by-vector rather
// than with HEADING() so the roll is ours to set: an aeroplane turns by banking,
// and a wings-level HEADING() command makes the steering manager try to skid the
// ship round on the rudder instead.
FUNCTION dirFor {
  PARAMETER hdgDeg, pitchDeg, bankDeg.
  LOCAL upv  IS SHIP:UP:VECTOR.
  LOCAL nrt  IS VXCL(upv, SHIP:NORTH:VECTOR):NORMALIZED.
  LOCAL est  IS eastVec():NORMALIZED.
  LOCAL pCmd IS clampVal(PITCH_MIN, PITCH_MAX, pitchDeg).
  LOCAL flat IS nrt * COS(hdgDeg) + est * SIN(hdgDeg).
  LOCAL fore IS flat * COS(pCmd) + upv * SIN(pCmd).
  LOCAL rgt  IS VCRS(upv, fore):NORMALIZED.       // starboard, same handedness as east
  LOCAL topv IS upv * COS(bankDeg) + rgt * SIN(bankDeg).
  RETURN LOOKDIRUP(fore, VXCL(fore, topv)).
}

// The command every atmospheric phase uses: hold `aoaDeg` of angle of attack on
// the airstream while pointing at `hdgDeg`.  Pitch is derived from the flight
// path angle each time the steering manager reads this, so AoA stays bounded
// however steep the descent gets - this is the stall guard.
FUNCTION aeroSteer {
  PARAMETER hdgDeg, aoaDeg, bankDeg.
  RETURN dirFor(hdgDeg, fpaNow() + clampVal(AOA_CMD_MIN, REENTRY_AOA, aoaDeg), bankDeg).
}

// Compass heading of the airstream, which is what every atmospheric command is
// referenced to.  A ship falling nearly straight down has no horizontal velocity
// to take a bearing from, and compassOf() would report a meaningless 0 - i.e.
// "turn north" - exactly when it is least able to survive being told to turn, so
// the last well-conditioned heading is held instead.
SET gLastHdg TO 90.
FUNCTION flightHdg {
  LOCAL svel IS SHIP:VELOCITY:SURFACE.
  IF VXCL(SHIP:UP:VECTOR, svel):MAG > 20 {
    SET gLastHdg TO compassOf(svel).
  } ELSE IF VXCL(SHIP:UP:VECTOR, SHIP:FACING:VECTOR):MAG > 0.2 {
    SET gLastHdg TO compassOf(SHIP:FACING:VECTOR).
  }
  RETURN gLastHdg.
}

// The angle between the nose and the airstream: the *actual* angle of attack,
// unsigned.  The commanded AoA is only a request; this is what the ship is
// really flying at, and it is how a departure is recognised.
FUNCTION noseOff {
  LOCAL svel IS SHIP:VELOCITY:SURFACE.
  IF svel:MAG < 20 { RETURN 0. }
  RETURN VANG(SHIP:FACING:VECTOR, svel).
}

// How far off the airstream the *command* is pointing, which is not the same as
// the AoA it asks for: a command is an attitude, and an attitude that is 40
// degrees of alpha and 25 degrees of lean is 47 degrees off the relative wind
// even when the ship is flying it perfectly.  Comparing noseOff() against the
// bare alpha therefore reads a correctly flown S-turn as a failure to hold the
// command - which is what walked the entry's alpha cap down 40 -> 38 -> 36 ->
// 34 -> 32 -> 30 on the reference flight, one bank reversal at a time, taking
// the drag the range plan had already spent with it.  Tracking error is the
// difference between where the nose is and where the command points, and this
// is the second half of it.
FUNCTION cmdOff {
  LOCAL svel IS SHIP:VELOCITY:SURFACE.
  IF svel:MAG < 20 { RETURN 0. }
  RETURN VANG(aeroSteer(gHdg, gAoa, gBank):FOREVECTOR, svel).
}

// --- The flight envelope ----------------------------------------------------
// How far off the airstream the nose may be pointed at this airspeed: ENTRY_YAW_MAX
// while hypersonic, opening linearly to YAW_SUB_MAX once subsonic.
FUNCTION yawAuthority {
  PARAMETER spd.
  IF spd >= YAW_SPD_HI { RETURN ENTRY_YAW_MAX. }
  IF spd <= YAW_SPD_LO { RETURN YAW_SUB_MAX. }
  LOCAL frac IS (spd - YAW_SPD_LO) / (YAW_SPD_HI - YAW_SPD_LO).
  RETURN YAW_SUB_MAX + frac * (ENTRY_YAW_MAX - YAW_SUB_MAX).
}

// How hard the ship may be banked.  Two independent limits, and the tighter one
// wins: the wing has to have the air to carry the turn (zero bank at STALL_Q,
// opening to full by MANEUVER_Q), and a fast ship leans rather than banks.
FUNCTION bankAuthority {
  PARAMETER spd.
  LOCAL byAir IS BANK_MAX * clampVal(0, 1,
        (SHIP:DYNAMICPRESSURE - STALL_Q) / (MANEUVER_Q - STALL_Q)).
  IF spd >= FAST_SPD { RETURN MIN(byAir, ENTRY_BANK_MAX). }
  RETURN byAir.
}

// AoA ceiling for the glide.  The stall guard is the binding limit once slow,
// but at supersonic speed a 14 deg pitch-up is not a stall risk, it is a lift
// spike: the ship balloons back up, bleeds what speed it had at altitude where
// there is nothing to fly on, and falls out of the sky.
FUNCTION glideAoaMax {
  PARAMETER spd.
  IF spd <= GLIDE_SPD_MIN { RETURN GLIDE_AOA_MAX. }
  IF spd >= FAST_SPD { RETURN FAST_AOA_MAX. }
  LOCAL frac IS (spd - GLIDE_SPD_MIN) / (FAST_SPD - GLIDE_SPD_MIN).
  RETURN GLIDE_AOA_MAX + frac * (FAST_AOA_MAX - GLIDE_AOA_MAX).
}

// The airspeed that puts GLIDE_Q under the wing *here*, at this altitude, in
// this air.  Dynamic pressure goes as rho*v^2 and both are known - the ship is
// flying in the answer - so the target speed is the speed being flown scaled by
// the square root of the pressure ratio, and no density model is needed.
//
// This is the difference between a glider and a brick.  At 12 km a 160 m/s
// "glide speed" is about 0.03 atm, below the pressure this script itself calls
// the minimum for a banked turn and barely above the one it calls the stall;
// the ship cannot hold a heading, departs, dives, recovers into the same
// command and departs again.  The same GLIDE_Q asks for 250 m/s there and 110
// over the runway, which is the speed schedule the airframe actually has.
FUNCTION glideSpeedTarget {
  LOCAL qNow IS SHIP:DYNAMICPRESSURE.
  LOCAL spd  IS SHIP:AIRSPEED.
  IF qNow < 0.001 OR spd < 40 { RETURN GLIDE_SPD_MAX. }   // no air to measure: dive
  RETURN clampVal(GLIDE_SPD_MIN, GLIDE_SPD_MAX, spd * SQRT(GLIDE_Q / qNow)).
}

// The single place a guidance law is turned into a steering command.  Every
// atmospheric phase asks for a heading and an AoA and gets back what the ship
// can actually fly: the heading demand is folded into the airstream heading
// through the yaw schedule, so the nose is *never* commanded across its own
// track, and the bank that goes with the turn is limited by the same schedule.
// A law that wants a 180 deg turn at Mach 2.5 gets 25 deg of lean and has to
// wait for the ship to slow down - which is the whole point.
FUNCTION setNav {
  PARAMETER hdgWant, aoaWant, aoaMax.
  LOCAL spd     IS SHIP:AIRSPEED.
  LOCAL base    IS flightHdg().
  LOCAL yawLim  IS yawAuthority(spd).
  LOCAL bankLim IS bankAuthority(spd).
  LOCAL hdgErr  IS clampVal(-yawLim, yawLim, normAng(hdgWant - base)).
  SET gHdg  TO base + hdgErr.
  SET gBank TO clampVal(-bankLim, bankLim, hdgErr * BANK_PER_DEG).
  SET gAoa  TO clampVal(AOA_CMD_MIN, aoaMax, aoaWant).
}

// Stall / departure recovery, and the only manoeuvre that outranks navigation:
// unload the wing, point the nose along the airstream, level the wings, stop
// asking for a turn, and trade height for speed until the ship is flying again.
// RCS goes back on because at 40 m/s the control surfaces have nothing to bite.
// Returns as soon as the wing is back, or at `floorAgl` - below that there is no
// height left to recover in and the landing is whatever it is going to be.
//
// `floorAgl` is radar altitude, i.e. height above whatever is actually under the
// ship.  It used to be height above the *runway*, which is a different number
// over anything that is not the runway: 600 m above a sea-level threshold is
// 1 400 m underground over a 2 km ridge, and the recovery dive - which is a
// deliberate, committed, nose-down dive - was being flown to that floor on the
// approach to the KSC over exactly such ridges.
//
// "Flying again" is two conditions, not one.  Dynamic pressure alone is what a
// tumbling ship reads on the way down: the reference flight recovered six times
// between 22 km and the sea, each time on the pressure gauge alone, each time
// handing the glide law a ship still 100-160 degrees off its own airstream, and
// each time departing again inside five seconds.  The nose has to be following
// the airstream too, and gSteady then keeps the turn demand off it for a few
// seconds while the speed builds a margin over the number that just failed.
SET gSteady TO 0.
FUNCTION stallRecover {
  PARAMETER floorAgl.
  PRINT "  ** stalled at " + ROUND(SHIP:AIRSPEED) + " m/s (" +
        ROUND(noseOff()) + " deg off the airstream) - unloading and diving.".
  BRAKES OFF.
  RCS ON.
  // Jets only where jets are worth their fuel.  A RAPIER at full throttle at
  // 20 km in a tumble is the deorbit reserve going out of the back of the ship.
  LOCAL jets IS USE_JETS_SHORT AND (resAmt("LiquidFuel") > JET_MIN_LF)
              AND SHIP:ALTITUDE < JET_ARM_ALT.
  LOCAL tGiveUp IS TIME:SECONDS + RECOVER_MAX_T.
  UNTIL (SHIP:DYNAMICPRESSURE > RECOVER_Q AND noseOff() < RECOVER_NOSE)
        OR ALT:RADAR < floorAgl
        OR TIME:SECONDS > tGiveUp
        OR SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" {
    SET gHdg  TO flightHdg().
    SET gAoa  TO RECOVER_AOA.
    SET gBank TO 0.
    IF jets { SET gThrot TO 1. }
    WAIT 0.05.
  }
  SET gThrot TO 0.
  RCS OFF.
  SET gSteady TO TIME:SECONDS + STEADY_TIME.
  IF SHIP:DYNAMICPRESSURE > RECOVER_Q AND noseOff() < RECOVER_NOSE {
    PRINT "  ** flying again at " + ROUND(SHIP:AIRSPEED) + " m/s, " +
          ROUND(ALT:RADAR) + " m AGL.".
  } ELSE IF ALT:RADAR < floorAgl {
    PRINT "  ** out of height to recover in: " + ROUND(SHIP:AIRSPEED) + " m/s at " +
          ROUND(ALT:RADAR) + " m AGL.".
  } ELSE {
    PRINT "  ** still " + ROUND(noseOff()) + " deg off the airstream after " +
          ROUND(RECOVER_MAX_T) + " s - re-thinking.".
  }
}

// Height the ship's energy is worth: where it is, plus where its speed could
// take it.  Everything the range plan says is a statement about this number.
FUNCTION energyHeight {
  LOCAL spd IS SHIP:AIRSPEED.
  RETURN MAX(0, SHIP:ALTITUDE - RWY_ELEV) + spd * spd / (2 * G0).
}

// The L/D the plan *expects* to average shedding that energy.  The hypersonic
// figure is not the glide ratio: most of the energy at Mach 6 leaves as heat,
// never as distance.
FUNCTION ldNominal {
  LOCAL spd IS SHIP:AIRSPEED.
  IF spd <= ENTRY_AOA_LO { RETURN PLAN_LD. }
  LOCAL frac IS clampVal(0, 1, (spd - ENTRY_AOA_LO) / (ENTRY_AOA_HI - ENTRY_AOA_LO)).
  RETURN PLAN_LD + frac * (ENTRY_LD - PLAN_LD).
}

// ... and the L/D it is actually getting, measured over the last few seconds:
// ground range flown per metre of energy height spent.  The nominal figures are
// a statement about a configuration; this is the bill the air is presenting for
// the configuration being flown right now, boards and all.
//
// This closes the loop that lost the reference flight.  That entry read 1.09 at
// the interface, called itself long, went to 40 degrees of alpha with the boards
// out, and *stayed* long on the gauge all the way down - because the nominal
// model credits a slowing ship with a rising L/D at exactly the rate the extra
// drag was destroying it.  It was in fact buying 1.7 km of range per km of
// energy height, not 3.1, and it crossed from "long" to 292 km short in the last
// 90 seconds of the entry, far too late to do anything about it.  Measured, the
// same flight reads short at 45 km, stops dumping, and stops digging.
SET gLdMeas TO 0.
SET gLdEh   TO 0.
SET gLdRng  TO 0.
SET gLdNext TO 0.
FUNCTION ldSample {
  PARAMETER rngToGo.
  IF TIME:SECONDS < gLdNext { RETURN. }
  LOCAL eh IS energyHeight().
  IF gLdEh > 0 {
    LOCAL dEh  IS gLdEh - eh.
    LOCAL dRng IS gLdRng - rngToGo.
    // Only price a real expenditure, and only one that bought ground toward the
    // fix.  Once the ship is on top of the target the range starts *growing*
    // again, and an L/D measured across that reads zero for a ship that is in
    // fact long - the one state in which shutting the energy dumps down would be
    // exactly the wrong answer.
    IF dEh > 50 AND dRng > 0 {
      LOCAL ldNow IS clampVal(0.3, 10, dRng / dEh).
      IF gLdMeas <= 0 { SET gLdMeas TO ldNow. }
      ELSE { SET gLdMeas TO gLdMeas + LD_FILTER * (ldNow - gLdMeas). }
    }
  }
  SET gLdEh   TO eh.
  SET gLdRng  TO rngToGo.
  SET gLdNext TO TIME:SECONDS + LD_SAMPLE.
}

// Ground range the ship could still cover if it flew for distance from here.
// The measurement only ever *lowers* the estimate: it is taken while flying for
// energy loss, not for range, so it is a floor on what the ship can do and not a
// forecast of what it will.  Believing the optimistic half of it is what put a
// spaceplane in the sea 248 km from the runway.
//
// But it is a floor on what the ship can do *hypersonically*, and applying it to
// the whole energy budget prices the glide at the entry's L/D.  Those are not
// the same aeroplane.  On the reference flight the measured entry L/D settled at
// 1.1, so at 49 km and Mach 6 the old form read 242 km of capability against
// 333 km to fly - `energy 0.56`, badly short - and the entry did what it is told
// to do when short: it gave up alpha, stowed the boards, and stopped dumping.
// It then arrived over the runway 21 km of energy height LONG, because the
// 70 km of altitude in that budget was never going to be spent at 1.1; it was
// going to be spent gliding, at 4.5.  Split the budget where the flight regime
// splits it - the kinetic energy above the handover speed is the entry's, and
// everything under it is the glider's - and the same moment reads 1.52, long,
// which is both true and actionable while there is still air to dump into.
FUNCTION rangeCapability {
  LOCAL spd   IS SHIP:AIRSPEED.
  LOCAL ehTot IS energyHeight().
  LOCAL ehFast IS 0.
  IF spd > ENTRY_END_SPD {
    SET ehFast TO MIN(ehTot,
          (spd * spd - ENTRY_END_SPD * ENTRY_END_SPD) / (2 * G0)).
  }
  LOCAL ldFast IS ldNominal().
  IF gLdMeas > 0 { SET ldFast TO MIN(ldFast, gLdMeas). }
  RETURN ehFast * ldFast + (ehTot - ehFast) * PLAN_LD.
}

// Great-circle distance over the ground to a point, in metres.  Note this is
// NOT geo:DISTANCE, which is the straight-line 3-D range and is dominated by
// altitude when the ship is high - at 30 km overhead the runway is "30 km away".
FUNCTION groundRange {
  PARAMETER geo.
  LOCAL ctr   IS SHIP:BODY:POSITION.             // ship -> body centre
  LOCAL vShip IS V(0, 0, 0) - ctr.               // centre -> ship
  LOCAL vTgt  IS geo:POSITION - ctr.             // centre -> target
  RETURN BODY_R * VANG(vShip, vTgt) * CONSTANT:DEGTORAD.
}

// Signed along-track range to the runway threshold, measured *in the landing
// direction*: positive means the runway is still ahead of us and we are that far
// short of it, negative means we have flown past it.  The cross-track figure is
// the same range resolved across the centreline.
//
// groundRange() cannot tell those two apart, and neither could any report this
// script has ever printed: "248.6 km from the runway" is the same line whether
// the ship stopped 248 km short of the field or sailed 248 km beyond it, and
// those two failures want opposite corrections to ENTRY_RANGE.  KSC_RWY:HEADING
// is the bearing *to* the threshold, so comparing it with the runway heading is
// the whole calculation.
FUNCTION alongTrackToRwy {
  RETURN groundRange(KSC_RWY) * COS(normAng(KSC_RWY:HEADING - RUNWAY_HDG)).
}
FUNCTION crossTrackToRwy {
  RETURN groundRange(KSC_RWY) * SIN(normAng(KSC_RWY:HEADING - RUNWAY_HDG)).
}

// The point `dist` metres from `origin` along compass heading `hdgDeg`.
FUNCTION geoOffset {
  PARAMETER origin, hdgDeg, dist.
  LOCAL dLat IS dist * COS(hdgDeg) / BODY_R * CONSTANT:RADTODEG.
  LOCAL dLng IS dist * SIN(hdgDeg) / (BODY_R * COS(origin:LAT)) * CONSTANT:RADTODEG.
  RETURN LATLNG(origin:LAT + dLat, origin:LNG + dLng).
}

// ---------------------------------------------------------------------------
//  Terrain look-ahead
//
//  The highest ground between here and a point, sampled along the direct track.
//
//  This is the number the whole approach turns on, and until now the script did
//  not have it.  ALT:RADAR answers "how high is the ground *under* me", and that
//  is the wrong question by exactly the time it takes to act on the answer: at
//  190 m/s a ridge face is eight seconds wide, and a glider with 8 m/s of climb
//  in hand cannot outrun rising ground.  A glider does not fly over a ridge, it
//  arrives already above it - so the ridge has to be measured while it is still
//  tens of kilometres ahead, and the energy plan has to be told about it then.
//
//  Two flights have now ended in the mountains west of the KSC, both of them on
//  an energy profile that was, referenced to a sea-level runway 68 m up, exactly
//  on plan the whole way down.  The plan was not wrong about energy.  It simply
//  had no idea the ground was not flat.
FUNCTION terrainPeakTo {
  PARAMETER geo, samples.
  LOCAL peak IS MAX(0, geo:TERRAINHEIGHT).
  LOCAL rng  IS groundRange(geo).
  IF rng < 100 OR samples < 1 { RETURN peak. }
  LOCAL here  IS SHIP:GEOPOSITION.
  LOCAL hdgTo IS geo:HEADING.
  FROM { LOCAL idx IS 0. } UNTIL idx > samples STEP { SET idx TO idx + 1. } DO {
    LOCAL pt IS geoOffset(here, hdgTo, rng * idx / samples).
    SET peak TO MAX(peak, pt:TERRAINHEIGHT).
  }
  RETURN peak.
}

// Cached, because a scan is MTN_SAMPLES geoposition lookups and the guidance
// loop runs at 10 Hz.  The ridge does not move; only our distance to it does,
// and MTN_SCAN_T seconds of that is 2 km at glide speed.
SET gMtnPeak TO 0.
SET gMtnNext TO 0.
FUNCTION scanTerrain {
  PARAMETER geo.
  IF TIME:SECONDS >= gMtnNext {
    SET gMtnPeak TO terrainPeakTo(geo, MTN_SAMPLES).
    SET gMtnNext TO TIME:SECONDS + MTN_SCAN_T.
  }
  RETURN gMtnPeak.
}

// AoA schedule for the entry: the full high-alpha attitude only while there is
// orbital energy to throw away, tapering to a flyable angle as the ship slows.
// Holding 40 degrees all the way down is what parks a spaceplane at the top of
// the atmosphere with no speed left and then drops it.
FUNCTION entryAoA {
  LOCAL spd IS SHIP:AIRSPEED.
  IF spd >= ENTRY_AOA_HI { RETURN REENTRY_AOA. }
  IF spd <= ENTRY_AOA_LO { RETURN GLIDE_AOA_MAX. }
  LOCAL frac IS (spd - ENTRY_AOA_LO) / (ENTRY_AOA_HI - ENTRY_AOA_LO).
  RETURN GLIDE_AOA_MAX + frac * (REENTRY_AOA - GLIDE_AOA_MAX).
}

// Fly the final approach and return at the flare height.  Nose for speed, boards
// for the glideslope - the way a glider is flown.  The localiser term steers back
// onto the extended centreline: when the threshold bears right of the runway
// heading we are left of it, so we turn right.
FUNCTION finalApproach {
  UNTIL ALT:RADAR < FLARE_ALT OR SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" {
    LOCAL rng    IS groundRange(KSC_RWY).
    LOCAL agl    IS SHIP:ALTITUDE - RWY_ELEV.
    LOCAL tgtAgl IS rng * TAN(GLIDESLOPE).
    LOCAL spd    IS SHIP:AIRSPEED.

    // Still worth recovering while there is height to dive in; below that the
    // flare is the only card left and fighting for airspeed just flies it in.
    IF SHIP:DYNAMICPRESSURE < STALL_Q AND ALT:RADAR > 300 {
      stallRecover(200).
    } ELSE {
      IF agl > tgtAgl + 60 { BRAKES ON. }
      ELSE IF agl < tgtAgl + 15 { BRAKES OFF. }
      IF spd < APPR_SPEED - 10 { BRAKES OFF. }

      LOCAL hdgWant IS RUNWAY_HDG.
      IF rng > 400 {
        SET hdgWant TO RUNWAY_HDG +
              clampVal(-LOC_MAX, LOC_MAX, normAng(KSC_RWY:HEADING - RUNWAY_HDG) * LOC_GAIN).
      }
      setNav(hdgWant, APPR_AOA + (spd - APPR_SPEED) * AOA_PER_MS, GLIDE_AOA_MAX).
      SET gBank TO clampVal(-20, 20, gBank).
      IF ALT:RADAR < 120 { SET gBank TO 0. }               // wings level before the wheels
      IF TIME:SECONDS < gSteady { SET gBank TO 0. }        // ... and after a recovery

      // Thrust on final is for the speed and for a low approach, never for the
      // altitude: above the slope with the speed made good it comes straight off
      // again, or the ship arrives high, fast and floating down the runway.
      IF USE_JETS_SHORT AND resAmt("LiquidFuel") > JET_MIN_LF
         AND (agl < tgtAgl - 60 OR spd < APPR_SPEED - 20) {
        SET gThrot TO clampVal(0, 1, THROT_MIN_PWR + (APPR_SPEED - spd) * THROT_PER_MS).
        IF agl > tgtAgl + 60 AND spd > APPR_SPEED { SET gThrot TO 0. }
      } ELSE {
        SET gThrot TO 0.
      }
    }
    WAIT 0.05.
  }
}

// Out of height somewhere that is not the KSC.  Fly the wings level and the
// airspeed on the nose, straight ahead, and put it down wherever "ahead" is: a
// low, slow ship banking toward a runway it cannot reach arrives inverted
// instead of merely somewhere else.
//
// This is a landing, so the ship must go *down*.  The old rule - full throttle
// whenever the speed was below the approach speed - could never be satisfied,
// because the nose was already holding that speed: the thrust had nowhere to go
// but into altitude, and the reference flight climbed 660 m -> 8.9 km on it,
// stalled at the top, dived, recovered, climbed again, and repeated the cycle
// until the tanks were empty and the last dive reached the sea.  Thrust here
// buys speed only, and only while the ship is descending.
FUNCTION emergencyLanding {
  SET tNext TO 0.
  UNTIL ALT:RADAR < FLARE_ALT OR SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" {
    LOCAL spd IS SHIP:AIRSPEED.
    IF SHIP:DYNAMICPRESSURE < STALL_Q AND ALT:RADAR > 200 {
      stallRecover(150).
    } ELSE {
      setNav(flightHdg(), APPR_AOA + (spd - APPR_SPEED) * AOA_PER_MS, GLIDE_AOA_MAX).
      SET gBank TO 0.
      SET landHdg TO gHdg.
      IF USE_JETS_SHORT AND resAmt("LiquidFuel") > JET_MIN_LF AND spd < APPR_SPEED {
        SET gThrot TO clampVal(0, 1, THROT_MIN_PWR + (APPR_SPEED - spd) * THROT_PER_MS).
      } ELSE {
        SET gThrot TO 0.
      }
      IF VERTICALSPEED > 0 { SET gThrot TO 0. }     // a diversion descends
    }
    IF TIME:SECONDS > tNext {
      PRINT "  divert  alt " + ROUND(ALT:RADAR) + " m" +
            "  spd " + ROUND(SHIP:AIRSPEED) + " m/s".
      SET tNext TO TIME:SECONDS + 5.
    }
    WAIT 0.05.
  }
}

// ---------------------------------------------------------------------------
//  1. DEORBIT
// ---------------------------------------------------------------------------
PRINT "=== Mk3 Titan Heavy SSTO :: deorbit & landing autopilot ===".
PRINT "Target: KSC runway  " + KSC_RWY:LAT + " / " + KSC_RWY:LNG.
SAS OFF.
RCS ON.
LOCK THROTTLE TO 0.

// Sanity-check the local frame the whole atmospheric autopilot is built on.
// compassOf() has to agree with the bearing kOS itself reports for a known
// point; if east came out mirrored, every heading command would be reflected
// about north and the ship would fly away from the KSC.  The horizontal
// projection of the chord to a point on a sphere lies along the great-circle
// bearing to it, so the two must match exactly - but only check it from far
// enough away that the bearing is well conditioned.
IF groundRange(KSC_RWY) > 20000 {
  SET frameErr TO ABS(normAng(compassOf(KSC_RWY:POSITION) - KSC_RWY:HEADING)).
  IF frameErr > 2 {
    PRINT "!! FRAME CHECK FAILED: compassOf disagrees with KSC bearing by " +
          ROUND(frameErr, 1) + " deg.".
    PRINT "!! Heading guidance cannot be trusted - check eastVec().".
  }
}

// --- 1a. Get in position ----------------------------------------------------
// Attitude first, node second, warp third: on rails the ship cannot rotate, so
// it has to leave for the burn point already pointing the right way.
goClosedCycle().
PRINT "Orienting retrograde before planning the burn.".
LOCK STEERING TO SHIP:RETROGRADE.
SET tAlign TO TIME:SECONDS + 120.
WAIT UNTIL VANG(SHIP:FACING:VECTOR, SHIP:RETROGRADE:VECTOR) < ALIGN_TOL
        OR TIME:SECONDS > tAlign.
PRINT "  In position: steering error " +
      ROUND(VANG(SHIP:FACING:VECTOR, SHIP:RETROGRADE:VECTOR), 1) + " deg.".

// --- 1b. Calculate the node -------------------------------------------------
UNTIL NOT HASNODE { REMOVE NEXTNODE. WAIT 0. }

// Solve the burn angle from the range we want to hit the air at, unless one was
// forced.  Two passes: the coast arc wants the radius at the burn point, and the
// burn point is what the lead angle finds, so price it at the radius we are at
// now (a circular parking orbit makes that nearly exact) and refine it once.
IF DEORBIT_LEAD <= 0 {
  SET DEORBIT_LEAD TO deorbitLeadFor(BODY_R + SHIP:ALTITUDE).
  SET tLeadChk TO deorbitTimeAfter(TIME:SECONDS + 60).
  IF tLeadChk > 0 {
    SET DEORBIT_LEAD TO deorbitLeadFor(BODY_R + SHIP:BODY:ALTITUDEOF(POSITIONAT(SHIP, tLeadChk))).
  }
  PRINT "Deorbit lead solved: " + ROUND(DEORBIT_LEAD, 1) + " deg" +
        "  (" + ROUND(ENTRY_RANGE / 1000) + " km of entry range + " +
        ROUND(coastArcDeg(BODY_R + SHIP:ALTITUDE), 1) + " deg of coast arc).".
} ELSE {
  PRINT "Deorbit lead forced to " + ROUND(DEORBIT_LEAD, 1) + " deg (entry range " +
        ROUND((DEORBIT_LEAD - coastArcDeg(BODY_R + SHIP:ALTITUDE))
              * BODY_R * CONSTANT:DEGTORAD / 1000) + " km).".
}

// Size a rough burn first, so the search only offers deorbit points we can
// still reach with the warp lead and half the burn in hand.
SET tbGuess  TO burnTimeFor(deorbitDvAt(TIME:SECONDS + 60)).
SET tSoonest TO TIME:SECONDS + WARP_LEAD + tbGuess / 2 + 10.
SET tBurn    TO deorbitTimeAfter(tSoonest).

SET usingNode TO tBurn > 0.
IF NOT usingNode {
  PRINT "WARNING: no deorbit point found within " + SCAN_ORBITS +
        " orbits - falling back to a real-time wait.".
  WAIT UNTIL ABS(trackAngleToKSC() - DEORBIT_LEAD) < 2.
} ELSE {
  SET dvNeed  TO deorbitDvAt(tBurn).
  SET burnDur TO burnTimeFor(dvNeed).
  IF burnDur <= 0 {
    PRINT "WARNING: no thrust reported - assuming a 30 s burn for the timing.".
    SET burnDur TO 30.
  }

  SET nd TO NODE(tBurn, 0, 0, -dvNeed).
  ADD nd.

  PRINT "Deorbit node planted:".
  PRINT "  T-" + ROUND((tBurn - TIME:SECONDS) / 60, 1) + " min" +
        "   dV " + ROUND(dvNeed, 1) + " m/s" +
        "   burn " + ROUND(burnDur, 1) + " s".
  PRINT "  Predicted periapsis: " + ROUND(nd:ORBIT:PERIAPSIS / 1000, 1) + " km".

  // --- 1c. Warp to the burn -------------------------------------------------
  SET tStart TO tBurn - burnDur / 2.          // burn centred on the node
  SET tWarp  TO tStart - WARP_LEAD.
  IF tWarp > TIME:SECONDS + 5 {
    PRINT "Warping " + ROUND(tWarp - TIME:SECONDS) + " s to the burn point.".
    WARPTO(tWarp).
    WAIT UNTIL TIME:SECONDS >= tWarp.
    SET WARP TO 0.
    WAIT UNTIL KUNIVERSE:TIMEWARP:ISSETTLED.
  }

  // Warp freezes attitude in inertial space, so retrograde has drifted away
  // from the nose while we were on rails.  Re-settle on the node itself.
  PRINT "Out of warp. Settling on the node burn vector.".
  LOCK STEERING TO burnSteer().
  WAIT UNTIL VANG(SHIP:FACING:VECTOR, nd:BURNVECTOR) < ALIGN_TOL
          OR TIME:SECONDS >= tStart.
  WAIT UNTIL TIME:SECONDS >= tStart.
}

PRINT "Deorbit burn: lowering periapsis to " + ROUND(DEORBIT_PE/1000,1) + " km.".
SET dv0 TO V(0, 0, 0).
IF usingNode { SET dv0 TO nd:DELTAV. }
SET burning TO TRUE.
LOCK THROTTLE TO 1.
UNTIL NOT burning {
  IF SHIP:PERIAPSIS <= DEORBIT_PE { SET burning TO FALSE. }
  IF SHIP:PERIAPSIS < DEORBIT_PE + 6000 { LOCK THROTTLE TO 0.2. }
  IF usingNode {
    IF nd:DELTAV:MAG < 15 { LOCK THROTTLE TO 0.2. }
    IF VDOT(dv0, nd:DELTAV) < 0 { SET burning TO FALSE. }   // node flipped: done
  }
  WAIT 0.05.
}
LOCK THROTTLE TO 0.
IF usingNode { REMOVE nd. }
PRINT "Deorbit complete. Periapsis " + ROUND(SHIP:PERIAPSIS / 1000, 1) +
      " km. Coasting to atmospheric interface.".

// ---------------------------------------------------------------------------
//  2. REENTRY  (angle of attack held on the airstream, nose biased home)
// ---------------------------------------------------------------------------
// The three commands the atmospheric autopilot flies on.  They are globals so
// the steering lock can be set once per phase and re-read every physics tick
// while the guidance loop refreshes them at its own, slower rate.
SET gHdg   TO 90.
SET gAoa   TO REENTRY_AOA.
SET gBank  TO 0.
SET gThrot TO 0.
LOCK THROTTLE TO gThrot.

SET RWY_ELEV TO MAX(0, KSC_RWY:TERRAINHEIGHT).

// The aim point for everything below the interface is the final approach fix,
// not the threshold: the runway is where the ship stops, the fix is where it has
// to *arrive*, and the entry is already being flown for the energy it will have
// when it gets there.
SET FAF     TO geoOffset(KSC_RWY, RUNWAY_HDG - 180, FINAL_DIST).
SET FAF_AGL TO FINAL_DIST * TAN(GLIDESLOPE).

// --- 2a. Turn around and coast to the interface ------------------------------
// The burn is over, so there is nothing left to point the engines at.  The ship
// spends the whole coast in the attitude it holds now and then meets the air in
// it, so that attitude has to be the entry attitude: nose *prograde*, along the
// airstream it is about to fly into, not looking back down its own wake.  Turn
// first, then warp - on rails the ship cannot rotate.
PRINT "Turning prograde for entry.".
LOCK STEERING TO SHIP:SRFPROGRADE.
SET tAlign TO TIME:SECONDS + 120.
WAIT UNTIL VANG(SHIP:FACING:VECTOR, SHIP:SRFPROGRADE:VECTOR) < ALIGN_TOL
        OR TIME:SECONDS > tAlign
        OR SHIP:ALTITUDE < ENTRY_ALT.
PRINT "  Pointing prograde: steering error " +
      ROUND(VANG(SHIP:FACING:VECTOR, SHIP:SRFPROGRADE:VECTOR), 1) + " deg.".

// How long that coast is, solved rather than sat through.
SET tCoast TO coastTimeTo(ENTRY_ALT).
IF tCoast < 0 {
  PRINT "  No atmospheric crossing predicted - holding attitude and waiting.".
} ELSE {
  PRINT "  Atmospheric interface (" + ROUND(ENTRY_ALT / 1000) + " km) in " +
        ROUND(tCoast) + " s (" + ROUND(tCoast / 60, 1) + " min).".
  IF tCoast > COAST_WARP_MIN {
    SET tWarp TO TIME:SECONDS + tCoast - COAST_LEAD.
    PRINT "  Warping " + ROUND(tWarp - TIME:SECONDS) + " s to the interface.".
    WARPTO(tWarp).
    WAIT UNTIL TIME:SECONDS >= tWarp OR SHIP:ALTITUDE < ENTRY_ALT.
    SET WARP TO 0.
    WAIT UNTIL KUNIVERSE:TIMEWARP:ISSETTLED.
    // Rails froze the attitude in inertial space while the velocity vector went
    // on turning under it, so the nose has to be put back on the airstream with
    // the COAST_LEAD seconds that were kept back for exactly this.
    PRINT "  Out of warp. Re-settling on the airstream.".
    SET tAlign TO TIME:SECONDS + COAST_LEAD.
    WAIT UNTIL VANG(SHIP:FACING:VECTOR, SHIP:SRFPROGRADE:VECTOR) < ALIGN_TOL
            OR TIME:SECONDS > tAlign
            OR SHIP:ALTITUDE < ENTRY_ALT.
  }
}

WAIT UNTIL SHIP:ALTITUDE < ENTRY_ALT.             // entering the atmosphere
PRINT "Atmospheric interface. High-AoA reentry.".

SET gHdg TO flightHdg().
LOCK STEERING TO aeroSteer(gHdg, gAoa, gBank).

// Present the belly to the airstream to bleed orbital energy, leaning toward the
// KSC only as far as the yaw schedule allows: the nose must stay near the
// relative wind, because a hypersonic ship told to point at a bearing well off
// its own track has no wing left to fly on and simply tumbles.
//
// Range is controlled here, where there is energy to spend, and not left to the
// glide: alpha is drag, so the entry holds *more* of it while it has more energy
// than the range ahead needs, banks into S-turns to throw the surplus away, and
// gives alpha up to stretch the trajectory when it is short.  An entry that
// ignores this arrives over the runway with a Mach number - which no glide law
// can do anything with.
SET tNext   TO 0.
SET tFlip   TO TIME:SECONDS + ENTRY_STURN_T.
SET gSturn  TO 1.
SET gAoaCap TO REENTRY_AOA.       // the most alpha the ship has shown it can hold
SET gOffCnt TO 0.                 // consecutive passes flown wide of the command
SET gDepCnt TO 0.                 // ... and consecutive passes flown clean off it
SET gLdNext TO TIME:SECONDS + LD_SAMPLE.
UNTIL SHIP:AIRSPEED < ENTRY_END_SPD OR SHIP:ALTITUDE < ENTRY_FLOOR {
  LOCAL rngFAF IS groundRange(FAF).
  ldSample(rngFAF).
  LOCAL eRatio IS rangeCapability() / MAX(1000, rngFAF).
  LOCAL aoaCmd IS entryAoA().
  LOCAL hdgWant IS flightHdg().
  // Tracking error against the *command*, not against the bare alpha - see
  // cmdOff().  The lean the S-turn asks for is part of the command, so flying it
  // is not evidence that the ship cannot hold its alpha.
  LOCAL offBy  IS noseOff() - cmdOff().

  // Is the ship flying the attitude it was told to?  A command it cannot hold is
  // worse than useless: the belly is not where the plan thinks it is, the drag
  // is not the drag the plan priced, and the difference all comes out of range.
  // So the entry believes the airframe over the tunable - it gives alpha back
  // until the nose tracks the command again, and takes it back a degree at a
  // time once it does.
  IF offBy > ENTRY_TRIM_TOL { SET gOffCnt TO gOffCnt + 1. } ELSE { SET gOffCnt TO 0. }
  IF gOffCnt > 20 {                                    // 2 s of not flying it
    SET gAoaCap TO MAX(ENTRY_AOA_MIN, gAoaCap - ENTRY_AOA_STEP).
    SET gOffCnt TO 0.
    PRINT "  ** holding " + ROUND(noseOff()) + " deg off the airstream against a " +
          ROUND(cmdOff()) + " deg command - entry alpha capped at " +
          ROUND(gAoaCap) + ".".
  } ELSE IF offBy < 5 AND gAoaCap < REENTRY_AOA {
    SET gAoaCap TO MIN(REENTRY_AOA, gAoaCap + 0.01).   // ~1 deg per 10 s of good tracking
  }

  // Departed: the nose is not merely lagging the command, it has let go of the
  // airstream altogether.  The entry loop used to have no such test at all, and
  // a ship that departed at 40 km simply tumbled all the way to the handover
  // printing the alpha it wished it were flying - which is why the first thing
  // the glide ever saw of the reference flight was 62 degrees off the airstream
  // at Mach 2.  One second of it, not one frame: rotating into the entry
  // attitude is a transient, not a departure.
  IF offBy > ENTRY_HOLD_TOL { SET gDepCnt TO gDepCnt + 1. } ELSE { SET gDepCnt TO 0. }
  IF gDepCnt > 10 {
    stallRecover(GLIDE_FLOOR).
    SET gAoaCap TO MAX(ENTRY_AOA_MIN, gAoaCap - ENTRY_AOA_STEP).
    SET gDepCnt TO 0.
    SET gOffCnt TO 0.
  }

  IF eRatio > ENERGY_LONG      { SET aoaCmd TO MIN(REENTRY_AOA,  aoaCmd + 6). }
  ELSE IF eRatio < ENERGY_SHORT { SET aoaCmd TO MAX(ENTRY_AOA_MIN, aoaCmd - 8). }

  // Too much alpha too low down turns the entry into a skip: the ship balloons
  // back out, bleeds the last of its speed at 35 km and falls out of the sky.
  IF VERTICALSPEED > SKIP_VS { SET aoaCmd TO MAX(GLIDE_AOA_MAX, aoaCmd - 15). }

  // The boards are an energy dump, and an energy dump is only ever wanted when
  // there is energy to spare.  Left out for the whole entry - which is what
  // BRAKES ON before the loop amounted to - they take the range the ship is
  // about to discover it needed, and they take it hardest in the last thin,
  // fast minutes where it can least be replaced.
  IF eRatio > ENERGY_LONG { BRAKES ON. }
  ELSE IF eRatio < ENERGY_LONG - 0.05 { BRAKES OFF. }

  // Steer at the fix while it is far enough away for its bearing to mean
  // something.  Inside ENTRY_HOLD_RNG the bearing to a point nearly underneath
  // swings through 180 degrees in seconds, and chasing it at Mach 3 is how a
  // reentry becomes a tumble - so the nose is simply held on the airstream and
  // the overshoot is left for the glide to fly off.
  IF rngFAF > ENTRY_HOLD_RNG {
    SET hdgWant TO FAF:HEADING.
    IF eRatio > ENERGY_LONG {
      IF TIME:SECONDS > tFlip {
        SET gSturn TO -gSturn.
        SET tFlip  TO TIME:SECONDS + ENTRY_STURN_T.
      }
      SET hdgWant TO FAF:HEADING + gSturn * ENTRY_YAW_MAX.
    }
  }
  setNav(hdgWant, aoaCmd, gAoaCap).

  // RCS holds the attitude until there is enough air for the control surfaces.
  IF SHIP:DYNAMICPRESSURE > RCS_Q_OFF { RCS OFF. } ELSE { RCS ON. }

  IF TIME:SECONDS > tNext {
    PRINT "  entry  alt " + ROUND(SHIP:ALTITUDE/1000, 1) + " km" +
          "  spd " + ROUND(SHIP:AIRSPEED) + " m/s" +
          "  AoA " + ROUND(gAoa) + "/" + ROUND(noseOff()) + " deg" +
          "  rwy " + ROUND(alongTrackToRwy()/1000) + " km" +
          "  energy " + ROUND(eRatio, 2) +
          "  L/D " + ROUND(gLdMeas, 1).
    SET tNext TO TIME:SECONDS + 5.
  }
  WAIT 0.1.
}
RCS OFF.
PRINT "Reentry heating survived: " + ROUND(SHIP:AIRSPEED) + " m/s at " +
      ROUND(SHIP:ALTITUDE/1000, 1) + " km, " +
      ROUND(groundRange(FAF)/1000, 1) + " km to the fix.".
IF alongTrackToRwy() > 0 {
  PRINT "  Runway is " + ROUND(alongTrackToRwy()/1000, 1) + " km ahead, " +
        ROUND(ABS(crossTrackToRwy())/1000, 1) + " km off track.".
} ELSE {
  PRINT "  Runway is " + ROUND(-alongTrackToRwy()/1000, 1) + " km BEHIND - " +
        "the entry overflew it. Raise ENTRY_RANGE.".
}
BRAKES OFF.

// ---------------------------------------------------------------------------
//  3. ENERGY-MANAGED GLIDE  (home on the final approach fix)
// ---------------------------------------------------------------------------
// Aim at a point on the extended centreline, FINAL_DIST short of the threshold,
// rather than at the threshold itself: arriving over the runway still high and
// pointing anywhere is what a bearing-only homing law gives you.
//
// Nothing in here may command a turn the wing cannot pay for.  Every heading
// demand goes through setNav(), which bounds it to the airstream and limits the
// bank by airspeed, and the stall check runs *before* the guidance law each
// pass: a ship with no dynamic pressure under the wing is not a ship that is
// slightly off profile, it is a ship that has stopped flying, and no amount of
// navigation fixes that.
goAirBreathing().
SET gSturn  TO 1.
SET tFlip   TO TIME:SECONDS + STURN_PERIOD.
SET tNext   TO 0.
SET gDepart  TO 0.
SET gLowQ    TO 0.
SET gPowerOn TO FALSE.
SET gDeckOn  TO FALSE.

PRINT "Gliding home. Ground range to the runway: " +
      ROUND(groundRange(KSC_RWY)/1000, 1) + " km.".

// Measure the mountains before flying at them, not while hitting them.
SET gMtnNext TO 0.
PRINT "  Highest ground on the way in: " + ROUND(scanTerrain(FAF)) + " m" +
      "  (deck " + ROUND(gMtnPeak + MTN_CLEAR) + " m, runway is at " +
      ROUND(RWY_ELEV) + " m).".

// Start the L/D measurement over.  The entry's number is a hypersonic one and
// has no bearing on what the wing does at 190 m/s; what the glide is really
// getting, boards and all, is the number that says whether PLAN_LD is honest.
// Reported only - feeding it back into the profile is the loop that taught the
// entry to stop dumping because it was dumping.
SET gLdMeas TO 0.
SET gLdEh   TO 0.
SET gLdRng  TO 0.
SET gLdNext TO TIME:SECONDS + LD_SAMPLE.

UNTIL (groundRange(FAF) < FAF_CAPTURE AND SHIP:ALTITUDE - RWY_ELEV < FAF_AGL + 1000)
      OR SHIP:ALTITUDE - RWY_ELEV < GLIDE_FLOOR
      OR ALT:RADAR < TERRAIN_ABORT {
  LOCAL rngFAF  IS groundRange(FAF).
  LOCAL spd     IS SHIP:AIRSPEED.
  ldSample(rngFAF).
  // The profile is a statement about *energy*, and the ship's altitude is only
  // half of the energy it has.  Comparing bare altitude against it reads a ship
  // at 5 km doing Mach 2 - an energy height of 26 km, wildly long, and about to
  // fly straight past the field - as 2.5 km *below* profile, retracts the
  // boards, and lights the jets.  Both sides are energy heights here: what the
  // ship has, and what the range ahead of it costs.
  LOCAL ehNow   IS energyHeight().
  LOCAL profEh  IS FAF_AGL + APPR_SPEED * APPR_SPEED / (2 * G0) + rngFAF / PLAN_LD.

  // The ridge ahead is part of the profile, not an exception to it.  A glider
  // that arrives at high ground low does not clear it, so the highest terrain
  // between here and the fix sets a *deck* - an altitude the glide may not
  // descend through - and the energy plan is raised to whatever holding that
  // deck costs.  A ship that cannot pay that is short, and being told so 60 km
  // out is the difference between lighting the jets and hitting a mountain.
  LOCAL deckAlt IS scanTerrain(FAF) + MTN_CLEAR.
  LOCAL deckEh  IS MAX(0, deckAlt - RWY_ELEV) + APPR_SPEED * APPR_SPEED / (2 * G0).
  IF deckEh > profEh { SET profEh TO deckEh. }

  // A departure is the nose refusing to follow the airstream for several seconds
  // running, not one frame of it: the ship arrives here rotating down from the
  // entry attitude, and 40 degrees of alpha that is on its way to 8 is a
  // handover transient, not a stall.  (Loop period is 0.1 s: 10 passes of low Q,
  // 30 of a nose that will not follow.)
  IF SHIP:DYNAMICPRESSURE < STALL_Q { SET gLowQ TO gLowQ + 1. } ELSE { SET gLowQ TO 0. }
  IF noseOff() > DEPART_AOA { SET gDepart TO gDepart + 1. } ELSE { SET gDepart TO 0. }

  IF gLowQ > 10 OR gDepart > 30 {
    stallRecover(GLIDE_FLOOR).
    SET gLowQ   TO 0.
    SET gDepart TO 0.
  } ELSE {
    // Speed is flown on the nose: too fast, more alpha (drag and lift); too
    // slow, less alpha and let the ship trade height back into airspeed.  The
    // ceiling is the stall guard and it is never relaxed.  The target is the
    // speed that puts GLIDE_Q under *this* wing at *this* altitude, which at
    // 20 km is 450 m/s and over the runway is 110 - a single number here is a
    // stall order everywhere it was not measured.
    LOCAL spdTgt  IS glideSpeedTarget().
    LOCAL hdgWant IS FAF:HEADING.
    LOCAL ehErr   IS ehNow - profEh.

    // ...but a speed is not a flight path, and a nose that is only ever trimmed
    // for a speed does not track a profile - it phugoids.  AoA says nothing
    // about where the ship is *going*, and with only 0.05 deg of alpha per m/s
    // of speed error to couple them the ship is free to hold any flight path it
    // happens to be on.  The reference flight held two of them: a 45-degree dive
    // from 12.2 km to 7.9 km at 240 m/s, and then, having bottomed out of it,
    // level flight at 8 km for 45 km of ground while the profile it was meant to
    // be tracking sank from 15 km of energy height to 6 km underneath it.  It
    // arrived 10 km from the runway with three times the energy it needed and
    // nowhere to put it.
    //
    // So the nose flies a *sink rate*.  The rate is the glide ratio the energy
    // error asks for - LD_DUMP steep when high, LD_STRETCH shallow when short,
    // PLAN_LD on profile - and the speed error trims it.  Both terms are bounded
    // and the stall guard still outranks them both.
    LOCAL ldWant IS PLAN_LD.
    IF ehErr > 0 {
      SET ldWant TO PLAN_LD +
            clampVal(0, 1, ehErr / HIGH_MARGIN) * (LD_DUMP - PLAN_LD).
    } ELSE {
      SET ldWant TO PLAN_LD +
            clampVal(0, 1, -ehErr / LOW_MARGIN) * (LD_STRETCH - PLAN_LD).
    }
    LOCAL vHor   IS VXCL(SHIP:UP:VECTOR, SHIP:VELOCITY:SURFACE):MAG.
    LOCAL vsWant IS -vHor / MAX(0.5, ldWant).

    // The ground outranks the plan, in two layers.  Each is the most sink its
    // own reference can pay for: unbinding while there is height in hand,
    // bleeding to nothing as the ship closes on the limit, and turning into a
    // commanded climb below it.  The tighter of the two wins.
    //
    //   vsRadar - the hillside directly underneath.  Last resort, and on its own
    //             far too late: it cannot see a ridge until the ridge is under
    //             the ship, and 8 m/s of climb does not clear one from there.
    //   vsDeck  - the highest ground between here and the fix, measured while it
    //             is still 60 km away.  This is the one that flies the approach.
    LOCAL vsRadar IS MIN(TERRAIN_CLIMB, (TERRAIN_FLOOR - ALT:RADAR) / TERRAIN_TAU).
    LOCAL vsDeck  IS MIN(TERRAIN_CLIMB, (deckAlt - SHIP:ALTITUDE) / TERRAIN_TAU).
    LOCAL vsFloor IS MAX(vsRadar, vsDeck).
    LOCAL lowGnd  IS vsFloor > vsWant.
    SET vsWant TO MAX(vsWant, vsFloor).

    // Say it out loud, once each way.  A glide that quietly stops descending is
    // indistinguishable in the log from a glide that has stopped working, and
    // this script has printed both.
    IF lowGnd AND NOT gDeckOn {
      SET gDeckOn TO TRUE.
      PRINT "  ** high ground ahead (" + ROUND(gMtnPeak) + " m) - holding the " +
            ROUND(deckAlt) + " m deck.".
    } ELSE IF gDeckOn AND NOT lowGnd {
      SET gDeckOn TO FALSE.
      PRINT "  ** clear of the high ground - back on the energy profile.".
    }

    LOCAL aoaCmd IS GLIDE_AOA + (spd - spdTgt) * AOA_PER_MS
                    + clampVal(-GLIDE_VS_AUTH, GLIDE_VS_AUTH,
                               (vsWant - VERTICALSPEED) * GLIDE_VS_GAIN).

    // Ballooning while still fast throws the speed away at an altitude with
    // nothing left to fly on - the same trap the entry guards against.  Not when
    // the ground is the reason we are going up.
    IF VERTICALSPEED > SKIP_VS AND NOT lowGnd { SET aoaCmd TO MIN(aoaCmd, GLIDE_AOA). }

    // Drag and track length are the other two ways to spend energy, and unlike
    // the nose they cost no altitude.  Both only while there is enough air under
    // the wing to use them: S-turning a wing that is barely flying is what puts
    // a spaceplane on its back.
    //
    // Neither is gated on the deck, because the deck is already inside profEh -
    // a ship reading long is long *after* paying for the ridge, and what it has
    // spare at a fixed altitude is speed, which is what the boards are for.  The
    // S-turn is the exception: the terrain scan is along the direct track, so
    // 40 degrees off it is 40 degrees of ground nobody measured.
    IF ehErr > BOARD_MARGIN AND SHIP:DYNAMICPRESSURE > MANEUVER_Q {
      BRAKES ON.
    } ELSE IF ehErr < BOARD_OFF OR SHIP:DYNAMICPRESSURE < MANEUVER_Q {
      BRAKES OFF.
    }

    IF ehErr > HIGH_MARGIN AND SHIP:DYNAMICPRESSURE > MANEUVER_Q AND NOT lowGnd {
      IF rngFAF < HOLD_RADIUS {
        // Overhead and far too high: spiral the excess off rather than chase a
        // bearing that swings through 180 degrees as the field passes underneath.
        SET hdgWant TO flightHdg() + 25.
      } ELSE {
        IF TIME:SECONDS > tFlip {
          SET gSturn TO -gSturn.
          SET tFlip  TO TIME:SECONDS + STURN_PERIOD.
        }
        SET hdgWant TO FAF:HEADING + gSturn * STURN_OFFSET.
      }
    } ELSE {
      SET tFlip TO TIME:SECONDS + STURN_PERIOD.
    }

    // The jets are cheaper than a crater, and the ascent script reserves fuel
    // for exactly this - but they are for a ship that is short of energy, and
    // for nothing else.  The old arming rule latched them on for any wing below
    // MANEUVER_Q, which every stall recovery satisfies, so a ship six kilometres
    // of energy height LONG came out of a recovery with the jets lit; and the
    // throttle then held a THROT_MIN_PWR floor whenever they were armed, so it
    // burned the reserve at 24% for 30 km holding 190 m/s level at 8 km.  Thrust
    // now needs a real energy deficit, a wing that has genuinely stopped flying,
    // or the ground.
    IF ehErr < -LOW_MARGIN OR SHIP:DYNAMICPRESSURE < STALL_Q OR lowGnd {
      SET gPowerOn TO TRUE.
    } ELSE IF ehErr > 0 AND SHIP:DYNAMICPRESSURE > RECOVER_Q {
      SET gPowerOn TO FALSE.
    }
    LOCAL powered IS gPowerOn AND USE_JETS_SHORT
                     AND resAmt("LiquidFuel") > JET_MIN_LF
                     AND SHIP:ALTITUDE < JET_ARM_ALT.

    IF powered {
      // Under thrust the two control laws swap ends: the throttle holds the
      // speed and the nose holds the flight path.  The nose is already holding
      // it - the sink command above is flown powered or not - so all that is
      // left here is the speed, and thrust that is not buying speed is not
      // bought at all.  The floor is spool-up for a ship that is genuinely
      // short; there is no floor for a ship that is merely slow and high,
      // because the answer to slow and high is to point the nose down.
      SET gThrot TO clampVal(0, 1, (spdTgt - spd) * THROT_PER_MS).
      IF ehErr < -LOW_MARGIN OR SHIP:DYNAMICPRESSURE < STALL_Q OR lowGnd {
        SET gThrot TO MAX(gThrot, THROT_MIN_PWR).
      } ELSE IF ehErr > 0 {
        SET gThrot TO 0.
      }
    } ELSE {
      SET gThrot TO 0.
    }

    setNav(hdgWant, aoaCmd, glideAoaMax(spd)).

    // Straight out of a recovery the ship is at exactly the speed that just
    // failed, and the first thing the guidance law does is ask it for a 30-degree
    // banked turn onto the fix.  That is the loop that flew the reference ship
    // into the sea - six departures, each one entered from the recovery of the
    // last.  Wings level and no turn demand until it has a margin again.
    IF TIME:SECONDS < gSteady {
      setNav(flightHdg(), aoaCmd, glideAoaMax(spd)).
      SET gBank TO 0.
    }
  }

  // Read the state again for the log: a stall recovery can have run for tens of
  // seconds since the top of this pass, and a report of where the ship *was* is
  // exactly the kind of thing that hides a problem like this one.
  IF TIME:SECONDS > tNext {
    LOCAL rngNow IS groundRange(FAF).
    LOCAL profNow IS MAX(FAF_AGL + rngNow / PLAN_LD,
                         MAX(0, gMtnPeak + MTN_CLEAR - RWY_ELEV))
                     + APPR_SPEED * APPR_SPEED / (2 * G0).
    PRINT "  glide  alt " + ROUND(SHIP:ALTITUDE - RWY_ELEV) +
          "/" + ROUND(ALT:RADAR) + " m" +
          "  energy " + ROUND(energyHeight()) +
          "/" + ROUND(profNow) + " m" +
          "  spd " + ROUND(SHIP:AIRSPEED) + "/" + ROUND(glideSpeedTarget()) + " m/s" +
          "  vs " + ROUND(VERTICALSPEED) + " m/s" +
          "  AoA " + ROUND(noseOff()) + " deg" +
          "  thr " + ROUND(gThrot * 100) + "%" +
          "  L/D " + ROUND(gLdMeas, 1) +
          "  mtn " + ROUND(gMtnPeak) + " m" +
          "  rwy " + ROUND(alongTrackToRwy()/1000, 1) + " km".
    SET tNext TO TIME:SECONDS + 5.
  }
  WAIT 0.1.
}

// ---------------------------------------------------------------------------
//  4. FINAL APPROACH, FLARE, TOUCHDOWN
// ---------------------------------------------------------------------------
// The glide ends either at the fix or at the floor, and those are not the same
// thing.  Only the first one is an approach; the second is a ship out of height
// somewhere else, and flying a runway heading and a 5-degree glideslope from
// there does not make the runway any closer - it just banks a low, slow ship at
// a field it cannot reach.  Say which one happened and fly the one we are in.
SET landHdg  TO RUNWAY_HDG.
SET divertGo TO groundRange(KSC_RWY) > DIVERT_RANGE.

GEAR ON.
LIGHTS ON.
BRAKES OFF.

IF divertGo {
  PRINT "!! Out of height " + ROUND(groundRange(KSC_RWY)/1000, 1) +
        " km from the runway - no approach to fly.".
  PRINT "   Landing straight ahead on heading " + ROUND(flightHdg()) + ".".
  SET landHdg TO flightHdg().
  emergencyLanding().
} ELSE {
  PRINT "Runway captured. Final approach on heading " + RUNWAY_HDG + ".".
  finalApproach().
}

PRINT "Flare.".
BRAKES OFF.
SET gThrot TO 0.
LOCK STEERING TO dirFor(landHdg, FLARE_PITCH, 0).   // absolute attitude: arrest the sink
SET tStop TO TIME:SECONDS + 30.
WAIT UNTIL ALT:RADAR < TOUCHDOWN_ALT OR SHIP:STATUS = "LANDED"
        OR SHIP:STATUS = "SPLASHED" OR TIME:SECONDS > tStop.

PRINT "Touchdown. Braking.".
LOCK THROTTLE TO 0.
BRAKES ON.
// Keep the nose straight down the runway; let the wheels do the steering.
LOCK STEERING TO dirFor(landHdg, 0, 0).
SET tStop TO TIME:SECONDS + 60.
WAIT UNTIL SHIP:GROUNDSPEED < 60 OR SHIP:STATUS = "SPLASHED" OR TIME:SECONDS > tStop.
LOCK STEERING TO SHIP:SRFPROGRADE.            // relax to velocity vector as we slow
SET tStop TO TIME:SECONDS + 120.
WAIT UNTIL SHIP:GROUNDSPEED < 1 OR TIME:SECONDS > tStop.

// Zero the *pilot's* throttle before handing control back: UNLOCK THROTTLE
// restores whatever the manual throttle was left at, which after an ascent is
// usually 100% - and the RAPIERs are back on air and very much able to use it.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
UNLOCK STEERING.
UNLOCK THROTTLE.
RCS OFF.
SAS ON.

// Report what actually happened.  "Stopped on the runway" printed 19 km from the
// threshold, in the sea, is worse than no report at all: the whole point of the
// last line is to be the thing you can trust when you were not watching.
SET rngEnd  TO groundRange(KSC_RWY).
SET alongEnd TO alongTrackToRwy().
PRINT "======================================================".
IF SHIP:STATUS = "LANDED" AND rngEnd < 3000 {
  PRINT "STOPPED ON THE RUNWAY.".
} ELSE IF SHIP:STATUS = "LANDED" {
  PRINT "DOWN AND STOPPED, but " + ROUND(rngEnd/1000, 1) + " km from the runway.".
} ELSE IF SHIP:STATUS = "SPLASHED" {
  PRINT "!! DITCHED IN THE WATER " + ROUND(rngEnd/1000, 1) + " km from the runway.".
} ELSE {
  PRINT "!! NOT DOWN: still " + SHIP:STATUS + " at " +
        ROUND(SHIP:ALTITUDE - RWY_ELEV) + " m AGL, " +
        ROUND(SHIP:GROUNDSPEED) + " m/s.".
}
// Which side it missed on, and therefore which way to trim.  An unsigned range
// is the same number for a ship that stopped 250 km short and one that overflew
// the field by 250 km, and those two want opposite corrections.
IF rngEnd > 3000 {
  IF alongEnd > 0 {
    PRINT "  SHORT by " + ROUND(alongEnd/1000, 1) + " km along the approach" +
          " (" + ROUND(ABS(crossTrackToRwy())/1000, 1) + " km off the centreline).".
    PRINT "  The entry did not reach: lower ENTRY_RANGE (now " +
          ROUND(ENTRY_RANGE/1000) + " km) by about that much.".
  } ELSE {
    PRINT "  LONG by " + ROUND(-alongEnd/1000, 1) + " km past the threshold" +
          " (" + ROUND(ABS(crossTrackToRwy())/1000, 1) + " km off the centreline).".
    PRINT "  The entry out-flew its schedule: raise ENTRY_RANGE (now " +
          ROUND(ENTRY_RANGE/1000) + " km) by about that much.".
  }
}
PRINT "  Ground range to runway threshold: " + ROUND(rngEnd) + " m".
PRINT "  Fuel remaining -- LiquidFuel: " + ROUND(resAmt("LiquidFuel")) +
      " , Oxidizer: " + ROUND(resAmt("Oxidizer")).
IF SHIP:STATUS = "LANDED" AND rngEnd < 3000 { PRINT "  Welcome home.". }
PRINT "======================================================".
