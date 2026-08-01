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
//    3. Transitions to an energy-managed glide, homing on the runway bearing.
//    4. Captures the runway heading, flies a glideslope, flares, drops the
//       gear, touches down, and brakes to a stop on the centreline.
//
//  IMPORTANT - this is a *guidance* script, not a precision lander.  A
//  spaceplane's huge glide range absorbs deorbit-timing error, but the
//  constants below (deorbit lead angle, glide pitch, flare height) are the
//  things you will most likely trim for your exact mass / drag.  They are all
//  grouped at the top.
//
//  Run with:   RUN deorbit_land.
// ============================================================================

CLEARSCREEN.

// --- KSC runway (09 threshold, west end; land heading ~090) -----------------
SET KSC_RWY        TO LATLNG(-0.0485, -74.726).
SET RUNWAY_HDG     TO 90.

// --- Tunables ---------------------------------------------------------------
SET DEORBIT_PE     TO 32000.     // target periapsis for the deorbit burn (m)
SET DEORBIT_LEAD   TO 118.       // ground-track angle before KSC to start burn (deg)
SET WARP_LEAD      TO 45.        // come out of warp this long before the burn (s)
SET SCAN_STEP      TO 20.        // coarse step when hunting the deorbit point (s)
SET SCAN_ORBITS    TO 2.         // how many orbits ahead to search for it
SET ALIGN_TOL      TO 5.         // steering error accepted before burning (deg)
SET ISP_FALLBACK   TO 305.       // RAPIER closed-cycle Isp, if none can be read (s)
SET REENTRY_AOA    TO 40.        // nose-up angle above the horizon during reentry (deg)
SET REENTRY_END    TO 32000.     // altitude to end the high-AoA reentry phase (m)
SET GLIDE_SPEED    TO 160.       // target glide airspeed (m/s)
SET FINAL_DIST     TO 12000.     // range at which to capture the runway (m)
SET GLIDESLOPE     TO 5.         // final-approach descent angle (deg)
SET FLARE_ALT      TO 28.        // radar altitude to begin the flare (m)
SET TOUCHDOWN_ALT  TO 3.         // radar altitude considered "on the wheels" (m)

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

// ---------------------------------------------------------------------------
//  1. DEORBIT
// ---------------------------------------------------------------------------
PRINT "=== Mk3 Titan Heavy SSTO :: deorbit & landing autopilot ===".
PRINT "Target: KSC runway  " + KSC_RWY:LAT + " / " + KSC_RWY:LNG.
SAS OFF.
RCS ON.
LOCK THROTTLE TO 0.

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
  LOCK STEERING TO nd:BURNVECTOR.
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
//  2. REENTRY  (high angle of attack, nose toward the runway)
// ---------------------------------------------------------------------------
LOCK STEERING TO SHIP:RETROGRADE.
WAIT UNTIL SHIP:ALTITUDE < 70000.                 // entering the atmosphere
PRINT "Atmospheric interface. High-AoA reentry.".
RCS OFF.
BRAKES ON.                                        // airbrakes/spoilers add drag

// Hold the nose REENTRY_AOA above the horizon, yawed toward the KSC bearing, to
// present the belly and bleed speed while flying generally home.
UNTIL SHIP:ALTITUDE < REENTRY_END {
  LOCK STEERING TO HEADING(KSC_RWY:HEADING, REENTRY_AOA).
  WAIT 0.1.
}
PRINT "Reentry heating survived. Transitioning to glide.".
BRAKES OFF.

// ---------------------------------------------------------------------------
//  3. ENERGY-MANAGED GLIDE  (home on the runway)
// ---------------------------------------------------------------------------
PRINT "Gliding toward the KSC. Range: " + ROUND(KSC_RWY:DISTANCE/1000,1) + " km.".
UNTIL KSC_RWY:DISTANCE < FINAL_DIST {
  LOCAL hdg IS KSC_RWY:HEADING.                    // compass bearing to the runway

  // Pitch to hold the target glide airspeed: too slow -> lower the nose,
  // too fast -> raise it.  Bounded so we never depart controlled flight.
  LOCAL pitchCmd IS 0.
  IF SHIP:AIRSPEED > GLIDE_SPEED + 15 { SET pitchCmd TO 6. }
  ELSE IF SHIP:AIRSPEED < GLIDE_SPEED - 15 { SET pitchCmd TO -8. }
  ELSE { SET pitchCmd TO -3. }
  SET pitchCmd TO MAX(-12, MIN(15, pitchCmd)).

  // Energy check: if we are far too high for the range remaining, add drag and
  // S-turn slightly; if too low, stretch the glide toward best L/D.
  LOCAL rangeKm IS KSC_RWY:DISTANCE / 1000.
  LOCAL altKm   IS SHIP:ALTITUDE / 1000.
  IF altKm > rangeKm * 0.9 { BRAKES ON. SET pitchCmd TO pitchCmd - 4. }
  ELSE { BRAKES OFF. }

  LOCK STEERING TO HEADING(hdg, pitchCmd).
  WAIT 0.2.
}

// ---------------------------------------------------------------------------
//  4. FINAL APPROACH, FLARE, TOUCHDOWN
// ---------------------------------------------------------------------------
PRINT "Runway captured. Final approach on heading " + RUNWAY_HDG + ".".
GEAR ON.
LIGHTS ON.

// Fly the runway heading and hold the glideslope until the flare height.
UNTIL ALT:RADAR < FLARE_ALT {
  // descend at the commanded glideslope: pitch tracks a target vertical speed
  LOCAL targetVS IS -1 * SHIP:AIRSPEED * SIN(GLIDESLOPE).   // m/s (negative = down)
  LOCAL pitchCmd IS 0.
  IF VERTICALSPEED < targetVS - 3 { SET pitchCmd TO 4. }    // sinking too fast
  ELSE IF VERTICALSPEED > targetVS + 3 { SET pitchCmd TO -2. }
  ELSE { SET pitchCmd TO 1. }
  IF SHIP:AIRSPEED > GLIDE_SPEED + 20 { BRAKES ON. } ELSE { BRAKES OFF. }
  LOCK STEERING TO HEADING(RUNWAY_HDG, pitchCmd).
  WAIT 0.1.
}

PRINT "Flare.".
BRAKES OFF.
LOCK STEERING TO HEADING(RUNWAY_HDG, 7).      // raise the nose to arrest descent
WAIT UNTIL ALT:RADAR < TOUCHDOWN_ALT OR SHIP:STATUS = "LANDED".

PRINT "Touchdown. Braking.".
LOCK THROTTLE TO 0.
BRAKES ON.
// Keep the nose straight down the runway; let the wheels do the steering.
LOCK STEERING TO HEADING(RUNWAY_HDG, 0).
WAIT UNTIL SHIP:GROUNDSPEED < 60.
LOCK STEERING TO SHIP:SRFPROGRADE.            // relax to velocity vector as we slow
WAIT UNTIL SHIP:GROUNDSPEED < 1.

UNLOCK STEERING.
UNLOCK THROTTLE.
SAS ON.
PRINT "======================================================".
PRINT "STOPPED ON THE RUNWAY.".
PRINT "  Distance to runway threshold: " + ROUND(KSC_RWY:DISTANCE) + " m".
PRINT "  Fuel remaining -- LiquidFuel: " + ROUND(resAmt("LiquidFuel")) +
      " , Oxidizer: " + ROUND(resAmt("Oxidizer")).
PRINT "  Welcome home.".
PRINT "======================================================".
