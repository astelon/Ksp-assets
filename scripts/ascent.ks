// ============================================================================
//  ascent.ks  --  Runway-to-orbit autopilot for the Mk3 Titan Heavy SSTO
// ----------------------------------------------------------------------------
//  Target : 100 km circular orbit over Kerbin, prograde (due east).
//  Engines: 12x CR-7 R.A.P.I.E.R.  (air-breathing on the runway, closed-cycle
//           for the final push to orbit).
//
//  Flight plan
//    0. Pre-flight : brakes, SAS off (kOS steers), lights, full throttle.
//    1. Runway roll: hold heading 090, rotate at ~80 m/s.
//    2. Air-breathing climb: shallow pitch, build speed to the edge of the
//       air-breathing envelope (~1400 m/s / ~20 km).
//    3. Mode switch : RAPIERs to closed cycle, pitch up, raise apoapsis to 100 km.
//    4. Coast       : throttle 0, coast to apoapsis.
//    5. Circularise : burn prograde at apoapsis until periapsis >= 99 km.
//    6. Report leftover fuel and hand back to the pilot.
//
//  Run with:   RUN ascent.        (from the archive or the ship's Script volume)
//  Tunables are grouped at the top so you can trim the profile to your build.
// ============================================================================

CLEARSCREEN.
SET TARGET_APOAPSIS TO 100000.      // desired apoapsis / orbit altitude (m)
SET CIRC_PE_TARGET  TO 99000.       // stop circularising when PE reaches this (m)
SET LAUNCH_HEADING  TO 90.          // due east - use Kerbin's rotation
SET ROTATE_SPEED    TO 80.          // surface speed to rotate the nose up (m/s)
SET SWITCH_ALT      TO 20000.       // altitude to force closed-cycle (m)
SET SWITCH_SPEED    TO 1400.        // OR airspeed at which to force closed-cycle
SET AB_CLIMB_PITCH  TO 12.          // pitch during air-breathing climb (deg)
SET ROCKET_PITCH_HI TO 25.          // initial pitch after mode switch (deg)

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------
FUNCTION resAmt {                    // total amount of a resource across the ship
  PARAMETER rname.
  LOCAL t IS 0.
  FOR r IN SHIP:RESOURCES { IF r:NAME = rname { SET t TO r:AMOUNT. } }
  RETURN t.
}

FUNCTION setRapierMode {             // force every RAPIER into one mode
  PARAMETER wantClosed.              // TRUE = ClosedCycle, FALSE = AirBreathing
  LIST ENGINES IN engs.
  FOR e IN engs {
    IF e:MULTIMODE {
      SET e:AUTOSWITCH TO FALSE.     // we manage the switch ourselves
      IF wantClosed AND e:MODE = "AirBreathing"  { e:TOGGLEMODE(). }
      IF (NOT wantClosed) AND e:MODE = "ClosedCycle" { e:TOGGLEMODE(). }
    }
  }
}

// ---------------------------------------------------------------------------
//  0. PRE-FLIGHT
// ---------------------------------------------------------------------------
PRINT "=== Mk3 Titan Heavy SSTO :: ascent autopilot ===".
PRINT "Payload + vessel mass: " + ROUND(SHIP:MASS, 1) + " t".
SAS OFF.
RCS OFF.
LIGHTS ON.
BRAKES ON.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
LOCK THROTTLE TO 1.
LOCK STEERING TO HEADING(LAUNCH_HEADING, 0).   // hold runway heading, wings level

PRINT "Ignition. RAPIERs to air-breathing.".
STAGE.                                // light the RAPIERs (bottom stage)
setRapierMode(FALSE).                 // ensure air-breathing for takeoff
WAIT 1.
BRAKES OFF.

// ---------------------------------------------------------------------------
//  1. RUNWAY ROLL + ROTATE
// ---------------------------------------------------------------------------
PRINT "Rolling. Rotate at " + ROTATE_SPEED + " m/s.".
WAIT UNTIL SHIP:VELOCITY:SURFACE:MAG > ROTATE_SPEED.
LOCK STEERING TO HEADING(LAUNCH_HEADING, 10).   // gentle rotation
PRINT "Rotate - climbing out.".
WAIT UNTIL SHIP:ALTITUDE > 100.                 // positively airborne

// ---------------------------------------------------------------------------
//  2. AIR-BREATHING CLIMB
// ---------------------------------------------------------------------------
PRINT "Air-breathing climb to the mode-switch point.".
LOCK STEERING TO HEADING(LAUNCH_HEADING, AB_CLIMB_PITCH).
GEAR OFF.                                        // retract landing gear

WAIT UNTIL (SHIP:ALTITUDE > SWITCH_ALT) OR (SHIP:AIRSPEED > SWITCH_SPEED).

// ---------------------------------------------------------------------------
//  3. CLOSED-CYCLE PUSH TO 100 km APOAPSIS
// ---------------------------------------------------------------------------
PRINT "Switching RAPIERs to closed cycle.".
setRapierMode(TRUE).
LOCK STEERING TO HEADING(LAUNCH_HEADING, ROCKET_PITCH_HI).

// Gradually shallow the pitch as apoapsis approaches the target (gravity turn).
UNTIL SHIP:APOAPSIS >= TARGET_APOAPSIS {
  LOCAL frac IS SHIP:APOAPSIS / TARGET_APOAPSIS.        // 0 -> 1 as we near target
  LOCAL pitchCmd IS MAX(5, ROCKET_PITCH_HI * (1 - frac) + 5).
  LOCK STEERING TO HEADING(LAUNCH_HEADING, pitchCmd).
  IF SHIP:APOAPSIS >= TARGET_APOAPSIS { BREAK. }
  WAIT 0.1.
}
LOCK THROTTLE TO 0.
PRINT "Apoapsis " + ROUND(SHIP:APOAPSIS/1000, 1) + " km reached. Coasting.".

// ---------------------------------------------------------------------------
//  4. COAST TO APOAPSIS (hold prograde, above the atmosphere)
// ---------------------------------------------------------------------------
LOCK STEERING TO SHIP:PROGRADE.
WAIT UNTIL SHIP:ALTITUDE > 70000.                // clear of the atmosphere (70 km)
PRINT "Vacuum. ETA to apoapsis: " + ROUND(ETA:APOAPSIS) + " s.".

// Start the circularisation burn a little before apoapsis so the burn straddles it.
WAIT UNTIL ETA:APOAPSIS < 12 OR SHIP:PERIAPSIS > 68000.

// ---------------------------------------------------------------------------
//  5. CIRCULARISE
// ---------------------------------------------------------------------------
PRINT "Circularising at apoapsis.".
LOCK STEERING TO SHIP:PROGRADE.
LOCK THROTTLE TO 1.
UNTIL SHIP:PERIAPSIS >= CIRC_PE_TARGET {
  // ease the throttle down in the last stretch for a precise cut-off
  IF SHIP:PERIAPSIS > CIRC_PE_TARGET - 4000 { LOCK THROTTLE TO 0.15. }
  WAIT 0.05.
}
LOCK THROTTLE TO 0.
UNLOCK STEERING.
SAS ON.

// ---------------------------------------------------------------------------
//  6. REPORT
// ---------------------------------------------------------------------------
PRINT "======================================================".
PRINT "ORBIT ACHIEVED".
PRINT "  Apoapsis : " + ROUND(SHIP:APOAPSIS/1000, 2) + " km".
PRINT "  Periapsis: " + ROUND(SHIP:PERIAPSIS/1000, 2) + " km".
PRINT "  Inclination: " + ROUND(SHIP:ORBIT:INCLINATION, 2) + " deg".
PRINT "  Fuel remaining -- LiquidFuel: " + ROUND(resAmt("LiquidFuel")) +
      " , Oxidizer: " + ROUND(resAmt("Oxidizer")) +
      " , Monoprop: " + ROUND(resAmt("MonoPropellant")).
PRINT "  Leftover dV is available for rendezvous / docking / deorbit.".
PRINT "======================================================".
PRINT "Autopilot complete. Ship handed back to pilot (SAS on).".
