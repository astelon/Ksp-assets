// ============================================================================
//  deorbit_land.ks  --  Deorbit, reentry, and guided glide back to the KSC
//                       runway for the Mk3 Titan Heavy SSTO.
// ----------------------------------------------------------------------------
//  From a circular Kerbin orbit this script:
//    1. Computes and executes a retrograde deorbit burn timed so the ground
//       track carries the ship toward the KSC, lowering periapsis to ~32 km.
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
SET REENTRY_AOA    TO 40.        // nose-up angle above the horizon during reentry (deg)
SET REENTRY_END    TO 32000.     // altitude to end the high-AoA reentry phase (m)
SET GLIDE_SPEED    TO 160.       // target glide airspeed (m/s)
SET FINAL_DIST     TO 12000.     // range at which to capture the runway (m)
SET GLIDESLOPE     TO 5.         // final-approach descent angle (deg)
SET FLARE_ALT      TO 28.        // radar altitude to begin the flare (m)
SET TOUCHDOWN_ALT  TO 3.         // radar altitude considered "on the wheels" (m)

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------
FUNCTION resAmt {
  PARAMETER rname.
  LOCAL t IS 0.
  FOR r IN SHIP:RESOURCES { IF r:NAME = rname { SET t TO r:AMOUNT. } }
  RETURN t.
}

// Signed ground-track angle (deg) from the ship's sub-point to the KSC, measured
// around the planet.  Uses longitude separation, corrected for orbit direction.
FUNCTION trackAngleToKSC {
  LOCAL d IS KSC_RWY:LNG - SHIP:GEOPOSITION:LNG.
  UNTIL d <= 180 { SET d TO d - 360. }
  UNTIL d > -180 { SET d TO d + 360. }
  RETURN d.
}

// ---------------------------------------------------------------------------
//  1. DEORBIT
// ---------------------------------------------------------------------------
PRINT "=== Mk3 Titan Heavy SSTO :: deorbit & landing autopilot ===".
PRINT "Target: KSC runway  " + KSC_RWY:LAT + " / " + KSC_RWY:LNG.
SAS OFF.
RCS ON.
LOCK THROTTLE TO 0.

// Point retrograde and wait until we are the lead-angle short of the KSC so the
// descent track ends up over the space centre.
PRINT "Orienting retrograde; waiting for the deorbit point.".
LOCK STEERING TO SHIP:RETROGRADE.
WAIT UNTIL VANG(SHIP:FACING:VECTOR, SHIP:RETROGRADE:VECTOR) < 5.

WAIT UNTIL ABS(trackAngleToKSC() - DEORBIT_LEAD) < 2.

PRINT "Deorbit burn: lowering periapsis to " + ROUND(DEORBIT_PE/1000,1) + " km.".
LOCK THROTTLE TO 1.
UNTIL SHIP:PERIAPSIS <= DEORBIT_PE {
  IF SHIP:PERIAPSIS < DEORBIT_PE + 6000 { LOCK THROTTLE TO 0.2. }
  WAIT 0.05.
}
LOCK THROTTLE TO 0.
PRINT "Deorbit complete. Coasting to atmospheric interface.".

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
