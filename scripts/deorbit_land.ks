// ============================================================================
//  deorbit_land.ks  --  Deorbit, reentry, and guided glide back to the KSC
//                       runway for the Mk3 Titan Heavy SSTO.
// ----------------------------------------------------------------------------
//  From a circular Kerbin orbit this script:
//    1. Orients retrograde, *predicts* the moment the ground track reaches the
//       deorbit point, plants a maneuver node there, time-warps to it, and
//       executes the retrograde burn, lowering periapsis to ~32 km.
//    2. Flies a high angle-of-attack reentry to bleed orbital energy while
//       steering the nose toward the runway.
//    3. Transitions to an energy-managed glide, homing on a final approach fix
//       on the extended runway centreline.
//    4. Captures the runway heading, flies a glideslope, flares, drops the
//       gear, touches down, and brakes to a stop on the centreline.
//
//  ATTITUDE CONVENTION - every atmospheric command in this script is given
//  relative to the *airstream*, not to the horizon.  `aeroSteer(hdg, aoa, bank)`
//  puts the nose `aoa` degrees above the surface velocity vector, so the angle
//  of attack is what is actually held no matter how steeply the ship is
//  descending.  Commanding an absolute pitch instead is what makes a spaceplane
//  stall: -3 degrees "nose down" while falling at -25 degrees is +22 degrees of
//  AoA, well past the wing.  Heading commands are likewise bounded relative to
//  the direction of flight while hypersonic, so the ship can never be told to
//  point across or behind its own airstream.
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
SET DEORBIT_LEAD   TO 118.       // ground-track angle before KSC to start burn (deg)
SET WARP_LEAD      TO 45.        // come out of warp this long before the burn (s)
SET SCAN_STEP      TO 20.        // coarse step when hunting the deorbit point (s)
SET SCAN_ORBITS    TO 2.         // how many orbits ahead to search for it
SET ALIGN_TOL      TO 5.         // steering error accepted before burning (deg)
SET ISP_FALLBACK   TO 305.       // RAPIER closed-cycle Isp, if none can be read (s)

// --- Tunables: reentry ------------------------------------------------------
SET REENTRY_AOA    TO 40.        // AoA held while hypersonic (deg above the airstream)
SET ENTRY_AOA_HI   TO 2000.      // airspeed at/above which the full AoA is held (m/s)
SET ENTRY_AOA_LO   TO 500.       // airspeed at/below which AoA has tapered to glide AoA
SET ENTRY_YAW_MAX  TO 25.        // furthest the nose may be yawed off the airstream (deg)
SET ENTRY_BANK_MAX TO 20.        // bank allowed while hypersonic (deg)
SET ENTRY_END_ALT  TO 25000.     // hand over to the glide at this altitude ...
SET ENTRY_END_SPD  TO 700.       // ... or at this airspeed, whichever comes first (m/s)
SET SKIP_VS        TO 15.        // climbing faster than this = ballooning, unload AoA
SET RCS_Q_OFF      TO 0.02.      // dynamic pressure (atm) at which RCS is no longer needed

// --- Tunables: glide --------------------------------------------------------
SET GLIDE_SPEED    TO 160.       // target glide airspeed (m/s)
SET GLIDE_AOA      TO 6.         // trim AoA at the target glide speed (deg)
SET GLIDE_AOA_MIN  TO 1.         // AoA authority given to the speed loop (deg)
SET GLIDE_AOA_MAX  TO 14.        // hard stall guard - never command more than this
SET AOA_PER_MS     TO 0.05.      // AoA added per m/s of overspeed
SET BANK_MAX       TO 30.        // bank angle used to turn (deg)
SET BANK_PER_DEG   TO 1.5.       // bank commanded per degree of heading error
SET PLAN_LD        TO 4.5.       // glide ratio the energy plan assumes (m fwd per m down)
SET HIGH_MARGIN    TO 1500.      // metres above the profile before dumping energy
SET LOW_MARGIN     TO 400.       // metres below the profile before calling it short
SET STURN_OFFSET   TO 40.        // heading offset flown when high (deg)
SET STURN_PERIOD   TO 18.        // seconds between S-turn reversals
SET HOLD_RADIUS    TO 8000.      // inside this range, excess height is spiralled off (m)
// Absolute attitude envelope, applied to every atmospheric command as a last
// resort.  It must stay clear of the entry attitude (REENTRY_AOA above a nearly
// flat flight path) or it would quietly cut the alpha the entry is flown at -
// its job is only to make sure the nose is never commanded at the sky.
SET PITCH_MAX      TO 45.        // absolute nose-up limit (deg)
SET PITCH_MIN      TO -60.       // absolute nose-down limit (deg)

// --- Tunables: approach & landing -------------------------------------------
SET FINAL_DIST     TO 9000.      // final approach fix, this far short of the threshold (m)
SET GLIDESLOPE     TO 5.         // final-approach descent angle (deg)
SET APPR_SPEED     TO 110.       // target airspeed on final (m/s)
SET APPR_AOA       TO 6.         // trim AoA on final (deg)
SET LOC_GAIN       TO 2.5.       // heading correction per degree of localiser error
SET LOC_MAX        TO 25.        // largest intercept angle onto the centreline (deg)
SET FLARE_ALT      TO 28.        // radar altitude to begin the flare (m)
SET FLARE_PITCH    TO 7.         // absolute nose-up attitude in the flare (deg)
SET TOUCHDOWN_ALT  TO 3.         // radar altitude considered "on the wheels" (m)
SET USE_JETS_SHORT TO TRUE.      // relight the jets rather than land short
SET JET_ARM_ALT    TO 9000.      // only below this altitude is a jet save attempted (m)

// ---------------------------------------------------------------------------
//  Body / physical constants (cached once)
// ---------------------------------------------------------------------------
SET G0            TO 9.80665.
SET BODY_MU       TO SHIP:BODY:MU.
SET BODY_R        TO SHIP:BODY:RADIUS.
SET BODY_ROT_RATE TO 360 / SHIP:BODY:ROTATIONPERIOD.   // deg/s, eastward

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
  RETURN dirFor(hdgDeg, fpaNow() + clampVal(GLIDE_AOA_MIN, REENTRY_AOA, aoaDeg), bankDeg).
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

// The point `dist` metres from `origin` along compass heading `hdgDeg`.
FUNCTION geoOffset {
  PARAMETER origin, hdgDeg, dist.
  LOCAL dLat IS dist * COS(hdgDeg) / BODY_R * CONSTANT:RADTODEG.
  LOCAL dLng IS dist * SIN(hdgDeg) / (BODY_R * COS(origin:LAT)) * CONSTANT:RADTODEG.
  RETURN LATLNG(origin:LAT + dLat, origin:LNG + dLng).
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

LOCK STEERING TO SHIP:SRFRETROGRADE.
WAIT UNTIL SHIP:ALTITUDE < 70000.                 // entering the atmosphere
PRINT "Atmospheric interface. High-AoA reentry.".
BRAKES ON.                                        // airbrakes/spoilers add drag

SET gHdg TO compassOf(SHIP:VELOCITY:SURFACE).
LOCK STEERING TO aeroSteer(gHdg, gAoa, gBank).

// Present the belly to the airstream to bleed orbital energy, yawing toward the
// KSC only as far as ENTRY_YAW_MAX: the nose must stay near the relative wind,
// because a hypersonic ship told to point at a bearing 90 degrees off its own
// track has no wing left to fly on and simply tumbles.
SET tNext TO 0.
UNTIL SHIP:ALTITUDE < ENTRY_END_ALT OR SHIP:AIRSPEED < ENTRY_END_SPD {
  LOCAL velHdg IS compassOf(SHIP:VELOCITY:SURFACE).
  LOCAL hdgErr IS normAng(KSC_RWY:HEADING - velHdg).

  SET gHdg  TO velHdg + clampVal(-ENTRY_YAW_MAX, ENTRY_YAW_MAX, hdgErr).
  SET gBank TO clampVal(-ENTRY_BANK_MAX, ENTRY_BANK_MAX, hdgErr * BANK_PER_DEG).
  SET gAoa  TO entryAoA().

  // Too much alpha too low down turns the entry into a skip: the ship balloons
  // back out, bleeds the last of its speed at 35 km and falls out of the sky.
  IF VERTICALSPEED > SKIP_VS { SET gAoa TO MAX(GLIDE_AOA_MAX, gAoa - 15). }

  // RCS holds the attitude until there is enough air for the control surfaces.
  IF SHIP:DYNAMICPRESSURE > RCS_Q_OFF { RCS OFF. } ELSE { RCS ON. }

  IF TIME:SECONDS > tNext {
    PRINT "  entry  alt " + ROUND(SHIP:ALTITUDE/1000, 1) + " km" +
          "  spd " + ROUND(SHIP:AIRSPEED) + " m/s" +
          "  AoA " + ROUND(gAoa) + " deg" +
          "  range " + ROUND(groundRange(KSC_RWY)/1000) + " km".
    SET tNext TO TIME:SECONDS + 5.
  }
  WAIT 0.1.
}
RCS OFF.
PRINT "Reentry heating survived. Transitioning to glide.".
BRAKES OFF.

// ---------------------------------------------------------------------------
//  3. ENERGY-MANAGED GLIDE  (home on the final approach fix)
// ---------------------------------------------------------------------------
// Aim at a point on the extended centreline, FINAL_DIST short of the threshold,
// rather than at the threshold itself: arriving over the runway still high and
// pointing anywhere is what a bearing-only homing law gives you.
goAirBreathing().
SET FAF     TO geoOffset(KSC_RWY, RUNWAY_HDG - 180, FINAL_DIST).
SET FAF_AGL TO FINAL_DIST * TAN(GLIDESLOPE).
SET gSturn  TO 1.
SET tFlip   TO TIME:SECONDS + STURN_PERIOD.
SET tNext   TO 0.

PRINT "Gliding home. Ground range to the runway: " +
      ROUND(groundRange(KSC_RWY)/1000, 1) + " km.".

UNTIL (groundRange(FAF) < 1500 AND SHIP:ALTITUDE - RWY_ELEV < FAF_AGL + 1000)
      OR SHIP:ALTITUDE - RWY_ELEV < 600 {
  LOCAL rngFAF  IS groundRange(FAF).
  LOCAL agl     IS SHIP:ALTITUDE - RWY_ELEV.
  LOCAL profAgl IS FAF_AGL + rngFAF / PLAN_LD.     // height this range should be flown at
  LOCAL velHdg  IS compassOf(SHIP:VELOCITY:SURFACE).

  // Speed is flown on the nose: too fast, more alpha (drag and lift); too slow,
  // less alpha and let the ship trade height back into airspeed.  The clamp is
  // the stall guard and it is never relaxed.
  SET gAoa TO clampVal(GLIDE_AOA_MIN, GLIDE_AOA_MAX,
                       GLIDE_AOA + (SHIP:AIRSPEED - GLIDE_SPEED) * AOA_PER_MS).

  // The flight path is flown on drag and on track length, never on the nose.
  IF rngFAF < HOLD_RADIUS AND agl > profAgl + HIGH_MARGIN {
    // Overhead and far too high: spiral the excess off rather than chase a
    // bearing that swings through 180 degrees as the field passes underneath.
    BRAKES ON.
    SET gHdg  TO velHdg + 25.
    SET gBank TO BANK_MAX.
  } ELSE IF agl > profAgl + HIGH_MARGIN {
    BRAKES ON.
    IF TIME:SECONDS > tFlip {
      SET gSturn TO -gSturn.
      SET tFlip  TO TIME:SECONDS + STURN_PERIOD.
    }
    SET gHdg  TO FAF:HEADING + gSturn * STURN_OFFSET.
    SET gBank TO clampVal(-BANK_MAX, BANK_MAX, normAng(gHdg - velHdg) * BANK_PER_DEG).
  } ELSE {
    BRAKES OFF.
    SET tFlip TO TIME:SECONDS + STURN_PERIOD.
    SET gHdg  TO FAF:HEADING.
    SET gBank TO clampVal(-BANK_MAX, BANK_MAX, normAng(gHdg - velHdg) * BANK_PER_DEG).
  }
  IF SHIP:AIRSPEED < GLIDE_SPEED - 20 { BRAKES OFF. }   // boards out near the stall: no

  // Short of the profile with no height left to trade - the jets are cheaper
  // than a crater, and the ascent script reserves fuel for exactly this.
  IF USE_JETS_SHORT AND agl < profAgl - LOW_MARGIN AND SHIP:ALTITUDE < JET_ARM_ALT {
    SET gThrot TO 1.
  } ELSE {
    SET gThrot TO 0.
  }

  IF TIME:SECONDS > tNext {
    PRINT "  glide  alt " + ROUND(agl) + " m (profile " + ROUND(profAgl) + ")" +
          "  spd " + ROUND(SHIP:AIRSPEED) + " m/s" +
          "  to FAF " + ROUND(rngFAF/1000, 1) + " km".
    SET tNext TO TIME:SECONDS + 5.
  }
  WAIT 0.1.
}

// ---------------------------------------------------------------------------
//  4. FINAL APPROACH, FLARE, TOUCHDOWN
// ---------------------------------------------------------------------------
PRINT "Runway captured. Final approach on heading " + RUNWAY_HDG + ".".
GEAR ON.
LIGHTS ON.
BRAKES OFF.

// Nose for speed, boards for the glideslope - the way a glider is flown.  The
// localiser term steers back onto the extended centreline: when the threshold
// bears right of the runway heading we are left of it, so we turn right.
UNTIL ALT:RADAR < FLARE_ALT {
  LOCAL rng    IS groundRange(KSC_RWY).
  LOCAL agl    IS SHIP:ALTITUDE - RWY_ELEV.
  LOCAL tgtAgl IS rng * TAN(GLIDESLOPE).

  SET gAoa TO clampVal(GLIDE_AOA_MIN, GLIDE_AOA_MAX,
                       APPR_AOA + (SHIP:AIRSPEED - APPR_SPEED) * AOA_PER_MS).

  IF agl > tgtAgl + 60 { BRAKES ON. }
  ELSE IF agl < tgtAgl + 15 { BRAKES OFF. }
  IF SHIP:AIRSPEED < APPR_SPEED - 10 { BRAKES OFF. }

  IF rng > 400 {
    SET gHdg TO RUNWAY_HDG +
                clampVal(-LOC_MAX, LOC_MAX, normAng(KSC_RWY:HEADING - RUNWAY_HDG) * LOC_GAIN).
  } ELSE {
    SET gHdg TO RUNWAY_HDG.
  }
  SET gBank TO clampVal(-20, 20, normAng(gHdg - compassOf(SHIP:VELOCITY:SURFACE)) * BANK_PER_DEG).
  IF ALT:RADAR < 120 { SET gBank TO 0. }                 // wings level before the wheels

  IF USE_JETS_SHORT AND agl < tgtAgl - 60 { SET gThrot TO 0.6. } ELSE { SET gThrot TO 0. }

  WAIT 0.05.
}

PRINT "Flare.".
BRAKES OFF.
SET gThrot TO 0.
LOCK STEERING TO dirFor(RUNWAY_HDG, FLARE_PITCH, 0).   // absolute attitude: arrest the sink
WAIT UNTIL ALT:RADAR < TOUCHDOWN_ALT OR SHIP:STATUS = "LANDED".

PRINT "Touchdown. Braking.".
LOCK THROTTLE TO 0.
BRAKES ON.
// Keep the nose straight down the runway; let the wheels do the steering.
LOCK STEERING TO dirFor(RUNWAY_HDG, 0, 0).
WAIT UNTIL SHIP:GROUNDSPEED < 60.
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
PRINT "======================================================".
PRINT "STOPPED ON THE RUNWAY.".
PRINT "  Distance to runway threshold: " + ROUND(KSC_RWY:DISTANCE) + " m".
PRINT "  Fuel remaining -- LiquidFuel: " + ROUND(resAmt("LiquidFuel")) +
      " , Oxidizer: " + ROUND(resAmt("Oxidizer")).
PRINT "  Welcome home.".
PRINT "======================================================".
