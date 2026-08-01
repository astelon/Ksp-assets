// ============================================================================
//  ascent.ks  --  Runway-to-orbit autopilot for the Mk3 Titan Heavy SSTO
// ----------------------------------------------------------------------------
//  Target : REQUESTED_APOAPSIS (default 100 km) circular over Kerbin, due east.
//  Engines: CR-7 R.A.P.I.E.R. cluster (air-breathing on the runway, closed
//           cycle for the final push to orbit).
//
//  What this version adds over a fixed-profile ascent
//  --------------------------------------------------
//  * BUDGETING.  The script measures the ship it is actually flying -- mass,
//    weight, thrust, TWR, and remaining closed-cycle dV (rocket equation over
//    the usable LF/Ox pair, Isp read off the live engines) -- and it keeps a
//    running reserve for CIRCULARISATION + DEORBIT so it can never strand the
//    ship in an orbit it has no fuel to leave.
//  * SETTLE-FOR-LESS.  It tries for the requested orbit, but if the remaining
//    dV is no longer enough to climb there *and still* pay for circularisation
//    and deorbit, it says so and settles for the best orbit it can afford.
//    Settling is only allowed once the achieved altitude/apoapsis is above
//    SETTLE_MIN_ALT (80 km), i.e. safely clear of the atmosphere.  Below that
//    it warns and keeps pushing, because half an orbit is worth nothing.
//  * PROFILE SENSING.  Nothing about the climb is hard-coded to one airframe:
//      - the air-breathing climb hill-climbs its pitch attitude to maximise
//        the *measured* specific-energy rate (dE/dt), inside a dynamic-
//        pressure corridor, so it finds this ship's own best climb/accelerate
//        compromise at this payload;
//      - the mode switch fires when the jets actually run out of breath
//        (flameout / thrust decay / acceleration collapse / the jets starting
//        to eat the orbital dV budget), not at a fixed altitude;
//      - the closed-cycle push steers to hold a target time-to-apoapsis,
//        which is the cheap way to turn thrust into orbital velocity.
//
//  Flight plan
//    0. Pre-flight  : measure the ship; print mass / weight / TWR / dV budget.
//    1. Runway roll : hold heading, rotate at ROTATE_SPEED.
//    2. Air-breathing climb: energy-rate-seeking pitch, sensed mode switch.
//    3. Closed-cycle push  : ETA-to-apoapsis guidance, dV budget policing.
//    4. Coast       : throttle 0, coast to apoapsis.
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
SET SETTLE_MIN_ALT     TO 80000.    // above this we may settle for a lower orbit (m)

// --- dV budgeting -----------------------------------------------------------
SET DEORBIT_PE          TO 32000.   // periapsis deorbit_land.ks burns down to (m)
SET DV_MARGIN           TO 60.      // spare kept on top of circ + deorbit (m/s)
SET DV_HARD_FLOOR       TO 25.      // never plan to burn below this (m/s)
SET CLIMB_LOSS_FACTOR   TO 1.15.    // gravity/steering losses on the climb burn
SET ROCKET_ISP_FALLBACK TO 305.     // closed-cycle Isp assumed before the switch (s)
SET PLAN_MECO_PE        TO -50000.  // periapsis assumed when pricing an orbit we
                                    // have not reached yet (deliberately
                                    // pessimistic, so circ dV is over-quoted)
SET LFO_LF_RATIO        TO 9.       // stock LF:Ox mixture ratio, LF part
SET LFO_OX_RATIO        TO 11.      // ... Ox part

// --- Runway / air-breathing climb -------------------------------------------
SET ROTATE_SPEED   TO 80.           // surface speed to rotate the nose up (m/s)
SET AB_START_PITCH TO 12.           // initial climb pitch, then self-tuned (deg)
SET AB_PITCH_MIN   TO 2.            // pitch clamp, air-breathing phase (deg)
SET AB_PITCH_MAX   TO 25.
SET AB_TUNE_DT     TO 1.            // energy-rate sampling interval (s)
SET AB_PITCH_STEP  TO 0.8.          // hill-climb step per sample (deg)
SET AB_PS_DEADBAND TO 20.           // energy-rate noise floor (J/kg/s)
SET AB_Q_HIGH      TO 0.45.         // dynamic-pressure corridor, upper (atm)
SET AB_Q_LOW       TO 0.12.         // ... lower (atm)

// --- Mode-switch sensing ----------------------------------------------------
SET SW_THRUST_DECAY TO 0.4.         // switch when thrust < this x peak jet thrust
SET SW_MIN_ACCEL    TO 0.6.         // ... or when acceleration drops below (m/s^2)
SET SW_ARM_SPEED    TO 1100.        // decay/accel sensing only arms above (m/s)
SET SW_ARM_ALT      TO 14000.       // ... and above this altitude (m)
SET SW_CONFIRM      TO 3.           // consecutive samples needed to confirm
SET SW_ALT_HARD     TO 26000.       // hard backstop: switch by this altitude (m)
SET SW_SPEED_HARD   TO 1650.        // ... or this airspeed (m/s)

// --- Closed-cycle push ------------------------------------------------------
SET TGT_ETA_AP     TO 35.           // time-to-apoapsis to hold while burning (s)
SET CC_PITCH_MIN   TO -5.           // pitch clamp, rocket phase (deg)
SET CC_PITCH_MAX   TO 35.
SET CC_START_PITCH TO 20.

// --- Circularisation --------------------------------------------------------
SET CIRC_PE_TOL TO 1500.            // accept periapsis this far under apoapsis (m)
SET PE_SAFETY   TO 8000.            // keep periapsis this far above the atmosphere (m)
SET CIRC_LEAD   TO 12.              // start the burn this long before apoapsis (s)

// ---------------------------------------------------------------------------
//  Body / physical constants (cached once)
// ---------------------------------------------------------------------------
SET G0      TO 9.80665.
SET BODY_MU TO SHIP:BODY:MU.
SET BODY_R  TO SHIP:BODY:RADIUS.
SET ATM_TOP TO SHIP:BODY:ATM:HEIGHT.

// ---------------------------------------------------------------------------
//  Ship measurement helpers
// ---------------------------------------------------------------------------
FUNCTION resAmt {                    // total amount of a resource across the ship
  PARAMETER rname.
  LOCAL t IS 0.
  FOR r IN SHIP:RESOURCES { IF r:NAME = rname { SET t TO t + r:AMOUNT. } }
  RETURN t.
}

FUNCTION resDensity {                // tonnes per unit (0.005 for LF and Ox)
  PARAMETER rname.
  FOR r IN SHIP:RESOURCES {
    IF r:NAME = rname AND r:DENSITY > 0 { RETURN r:DENSITY. }
  }
  RETURN 0.005.
}

// Vacuum Isp of the engines that will do the rocket work.  While the RAPIERs
// are still breathing air their reported Isp is the jet figure, which would
// wildly overstate the budget, so until the switch we quote the fallback.
FUNCTION rocketIsp {
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  LOCAL wsum IS 0.
  LOCAL isum IS 0.
  FOR e IN engs {
    IF (NOT e:MULTIMODE) OR (e:MODE = "ClosedCycle") {
      LOCAL t IS e:MAXTHRUST.
      IF t > 0 AND e:VACUUMISP > 0 {
        SET wsum TO wsum + t.
        SET isum TO isum + t * e:VACUUMISP.
      }
    }
  }
  IF wsum > 0 { RETURN isum / wsum. }
  RETURN ROCKET_ISP_FALLBACK.
}

// Propellant mass (t) the closed-cycle engines can actually burn: LF and Ox
// only count in 9:11 pairs, so a lopsided tank state is not counted as dV.
FUNCTION rocketPropMass {
  LOCAL lf IS resAmt("LiquidFuel").
  LOCAL ox IS resAmt("Oxidizer").
  LOCAL lfUse IS MIN(lf, ox * LFO_LF_RATIO / LFO_OX_RATIO).
  LOCAL oxUse IS lfUse * LFO_OX_RATIO / LFO_LF_RATIO.
  RETURN lfUse * resDensity("LiquidFuel") + oxUse * resDensity("Oxidizer").
}

// Remaining closed-cycle dV (m/s): the rocket equation on the live ship.
FUNCTION rocketDv {
  LOCAL m0 IS SHIP:MASS.
  LOCAL m1 IS m0 - rocketPropMass().
  IF m0 <= 0 OR m1 <= 0 OR m1 >= m0 { RETURN 0. }
  RETURN rocketIsp() * G0 * LN(m0 / m1).
}

FUNCTION localG   { RETURN BODY_MU / ((BODY_R + SHIP:ALTITUDE) * (BODY_R + SHIP:ALTITUDE)). }
FUNCTION weightKN { RETURN SHIP:MASS * localG(). }        // 1 t x 1 m/s^2 = 1 kN
FUNCTION twrNow {
  LOCAL w IS weightKN().
  IF w <= 0 { RETURN 0. }
  RETURN SHIP:AVAILABLETHRUST / w.
}

// ---------------------------------------------------------------------------
//  Orbital mechanics helpers  (altitudes in m above the surface)
// ---------------------------------------------------------------------------
FUNCTION vCircAt {                   // circular orbital speed at an altitude
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
// here.  It compares speed magnitudes, so it is optimistic; CLIMB_LOSS_FACTOR
// pads it for gravity and steering losses.
FUNCTION dvToRaiseApTo {
  PARAMETER apAlt.
  LOCAL r IS BODY_R + SHIP:ALTITUDE.
  LOCAL rt IS BODY_R + apAlt.
  IF rt <= r OR SHIP:APOAPSIS >= apAlt { RETURN 0. }
  LOCAL smaT IS (r + rt) / 2.
  LOCAL vNeed IS SQRT(BODY_MU * (2 / r - 1 / smaT)).
  RETURN MAX(0, vNeed - SHIP:VELOCITY:ORBIT:MAG) * CLIMB_LOSS_FACTOR.
}

// Full price of an orbit at apAlt from where we are right now:
// climb there + circularise + deorbit later + margin.
FUNCTION priceOfOrbit {
  PARAMETER apAlt.
  RETURN dvToRaiseApTo(apAlt) + circDvAt(apAlt) + reserveDvFor(apAlt).
}

// Specific orbital energy (J/kg) - the quantity an efficient ascent maximises.
FUNCTION shipEnergy {
  LOCAL vOrb IS SHIP:VELOCITY:ORBIT:MAG.
  RETURN 0.5 * vOrb * vOrb - BODY_MU / (BODY_R + SHIP:ALTITUDE).
}

FUNCTION clampVal {
  PARAMETER x, lo, hi.
  RETURN MAX(lo, MIN(hi, x)).
}

// ---------------------------------------------------------------------------
//  Engine helpers
// ---------------------------------------------------------------------------
FUNCTION setRapierMode {             // force every multimode engine into one mode
  PARAMETER wantClosed.              // TRUE = ClosedCycle, FALSE = AirBreathing
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  FOR e IN engs {
    IF e:MULTIMODE {
      SET e:AUTOSWITCH TO FALSE.     // we manage the switch ourselves
      IF wantClosed AND e:MODE = "AirBreathing" { e:TOGGLEMODE(). }
      IF (NOT wantClosed) AND e:MODE = "ClosedCycle" { e:TOGGLEMODE(). }
    }
  }
}

FUNCTION anyFlameout {
  LOCAL engs IS LIST().
  LIST ENGINES IN engs.
  LOCAL out IS FALSE.
  FOR e IN engs { IF e:IGNITION AND e:FLAMEOUT { SET out TO TRUE. } }
  RETURN out.
}

FUNCTION budgetLine {                // one-line dV status, used throughout
  PARAMETER apAlt.
  RETURN "dV " + ROUND(rocketDv()) + " m/s vs need " + ROUND(priceOfOrbit(apAlt)) +
         " (climb " + ROUND(dvToRaiseApTo(apAlt)) +
         " + circ " + ROUND(circDvAt(apAlt)) +
         " + deorbit " + ROUND(deorbitDvFrom(apAlt)) +
         " + margin " + ROUND(DV_MARGIN) + ")".
}

// ---------------------------------------------------------------------------
//  0. PRE-FLIGHT  --  measure the ship we are actually flying
// ---------------------------------------------------------------------------
PRINT "=== Mk3 Titan Heavy SSTO :: ascent autopilot ===".
SAS OFF.
RCS OFF.
LIGHTS ON.
BRAKES ON.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.

SET thrCmd   TO 0.
SET pitchCmd TO 0.
LOCK THROTTLE TO thrCmd.
LOCK STEERING TO HEADING(LAUNCH_HEADING, pitchCmd).

PRINT "Ignition. RAPIERs to air-breathing.".
STAGE.                                // light the RAPIERs (bottom stage)
setRapierMode(FALSE).                 // ensure air-breathing for takeoff
SET thrCmd TO 1.
WAIT 1.

SET launchMass TO SHIP:MASS.
SET jetThrust  TO SHIP:AVAILABLETHRUST.
SET jetTwr     TO twrNow().
SET dvAtPad    TO rocketDv().         // closed-cycle dV sitting in the tanks
SET planCirc   TO circDvAt(REQUESTED_APOAPSIS).
SET planDeorb  TO deorbitDvFrom(REQUESTED_APOAPSIS).

PRINT "------------------------------------------------------".
PRINT "VEHICLE".
PRINT "  Mass         : " + ROUND(launchMass, 1) + " t  (dry " + ROUND(SHIP:DRYMASS, 1) + " t)".
PRINT "  Weight       : " + ROUND(weightKN()) + " kN".
PRINT "  Thrust (jet) : " + ROUND(jetThrust) + " kN    TWR " + ROUND(jetTwr, 2).
PRINT "  Rocket dV    : " + ROUND(dvAtPad) + " m/s  (Isp " + ROUND(rocketIsp()) +
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
SET pitchCmd TO 10.                             // gentle rotation
PRINT "Rotate - climbing out.".
WAIT UNTIL SHIP:ALTITUDE > 100.                 // positively airborne
GEAR OFF.

// ---------------------------------------------------------------------------
//  2. AIR-BREATHING CLIMB  --  self-tuning profile
// ---------------------------------------------------------------------------
//  The pitch is not scheduled, it is *searched*.  Every AB_TUNE_DT we measure
//  the specific-energy rate dE/dt (the rate at which the ship is banking the
//  speed+altitude it will need in orbit) and nudge the pitch in whichever
//  direction last improved it: a hill-climb flown on the real airframe, so it
//  adapts to payload, drag and air density instead of trusting a constant.
//  Two guards bound the search - a dynamic-pressure corridor (climb out of air
//  that is too thick, sink back into air too thin to breathe) and a sink guard
//  down low.
// ---------------------------------------------------------------------------
PRINT "Air-breathing climb: seeking the best energy-rate profile.".
SET pitchCmd TO AB_START_PITCH.

SET lastE     TO shipEnergy().
SET lastT     TO TIME:SECONDS.
SET lastV     TO SHIP:AIRSPEED.
SET lastPs    TO -1000000.
SET tuneDir   TO 1.
SET peakJetT  TO SHIP:AVAILABLETHRUST.
SET swVotes   TO 0.
SET switchNow TO FALSE.
SET swReason  TO "hard limit".
SET lfWarned  TO FALSE.
SET abReportT TO TIME:SECONDS.

UNTIL switchNow {
  WAIT AB_TUNE_DT.

  LOCAL nowT  IS TIME:SECONDS.
  LOCAL dt    IS MAX(0.05, nowT - lastT).
  LOCAL eNow  IS shipEnergy().
  LOCAL ps    IS (eNow - lastE) / dt.            // specific excess power (J/kg/s)
  LOCAL accel IS (SHIP:AIRSPEED - lastV) / dt.   // longitudinal acceleration
  SET lastE TO eNow.
  SET lastT TO nowT.
  SET lastV TO SHIP:AIRSPEED.
  SET peakJetT TO MAX(peakJetT, SHIP:AVAILABLETHRUST).

  // ---- hill-climb the pitch on the measured energy rate -------------------
  IF ps < lastPs - AB_PS_DEADBAND { SET tuneDir TO -tuneDir. }
  SET lastPs TO ps.
  SET pitchCmd TO pitchCmd + tuneDir * AB_PITCH_STEP.

  // ---- corridor guards (these outrank the search) -------------------------
  IF SHIP:Q > AB_Q_HIGH {
    SET pitchCmd TO pitchCmd + 2. SET tuneDir TO 1.          // too deep, climb
  } ELSE IF SHIP:Q < AB_Q_LOW AND SHIP:ALTITUDE > 12000 {
    SET pitchCmd TO pitchCmd - 1.5. SET tuneDir TO -1.       // too thin, accelerate
  }
  IF SHIP:ALTITUDE < 6000 AND SHIP:VERTICALSPEED < 5 {
    SET pitchCmd TO pitchCmd + 2. SET tuneDir TO 1.          // do not settle
  }
  SET pitchCmd TO clampVal(pitchCmd, AB_PITCH_MIN, AB_PITCH_MAX).

  // ---- book-keeping -------------------------------------------------------
  //  The jets run on liquid fuel, and the ship carries an LF-only reserve for
  //  exactly that.  Once LF drops to the level that pairs with the remaining
  //  oxidiser, every further second of jet burn is spent out of the rocket dV
  //  budget - worth knowing, though not worth switching for (see below).
  IF (NOT lfWarned) AND resAmt("LiquidFuel") <= resAmt("Oxidizer") * LFO_LF_RATIO / LFO_OX_RATIO {
    PRINT "  Note: LF-only reserve used up; jet burn now draws on rocket dV (" +
          ROUND(rocketDv()) + " m/s left).".
    SET lfWarned TO TRUE.
  }

  IF TIME:SECONDS - abReportT > 15 {
    PRINT "  alt " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km | " + ROUND(SHIP:AIRSPEED) +
          " m/s | pitch " + ROUND(pitchCmd, 1) + " | Q " + ROUND(SHIP:Q, 2) +
          " | dE/dt " + ROUND(ps) + " | dV " + ROUND(rocketDv()).
    SET abReportT TO TIME:SECONDS.
  }

  // ---- sense the end of the air-breathing envelope ------------------------
  //  Purely physical signals.  Note we deliberately do NOT switch on the dV
  //  budget here: jet Isp is ~10x the rocket's, so while the jets still make
  //  thrust they are always the cheaper way to buy speed, even though they
  //  burn liquid fuel the rocket phase would also like to have.
  LOCAL armed   IS SHIP:AIRSPEED > SW_ARM_SPEED AND SHIP:ALTITUDE > SW_ARM_ALT.
  LOCAL decayed IS SHIP:AVAILABLETHRUST < peakJetT * SW_THRUST_DECAY.
  LOCAL gasping IS accel < SW_MIN_ACCEL.
  LOCAL atCeiling IS SHIP:ALTITUDE > 12000 AND SHIP:VERTICALSPEED < 2 AND accel < 0.2.

  IF anyFlameout() {
    SET switchNow TO TRUE.
    SET swReason TO "flameout - out of intake air".
  } ELSE IF (armed AND (decayed OR gasping)) OR atCeiling {
    SET swVotes TO swVotes + 1.
    IF swVotes >= SW_CONFIRM {
      SET switchNow TO TRUE.
      IF atCeiling {
        SET swReason TO "air-breathing ceiling reached".
      } ELSE IF decayed {
        SET swReason TO "jet thrust decayed to " +
                        ROUND(100 * SHIP:AVAILABLETHRUST / MAX(1, peakJetT)) + "% of peak".
      } ELSE {
        SET swReason TO "acceleration fell to " + ROUND(accel, 2) + " m/s^2".
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

// ---------------------------------------------------------------------------
//  3. CLOSED-CYCLE PUSH  --  with live dV policing
// ---------------------------------------------------------------------------
PRINT "Mode switch: " + swReason + ".".
PRINT "  at " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km, " + ROUND(SHIP:AIRSPEED) +
      " m/s, pitch " + ROUND(pitchCmd, 1) + " deg.".
setRapierMode(TRUE).
WAIT 0.5.                                       // let the engines settle in mode
SET pitchCmd TO CC_START_PITCH.

PRINT "Closed cycle. " + budgetLine(REQUESTED_APOAPSIS).
PRINT "  Mass " + ROUND(SHIP:MASS, 1) + " t, thrust " + ROUND(SHIP:AVAILABLETHRUST) +
      " kN, TWR " + ROUND(twrNow(), 2) + ", Isp " + ROUND(rocketIsp()) + " s.".

SET targetAp    TO REQUESTED_APOAPSIS.
SET settled     TO FALSE.             // TRUE once we give up on the requested orbit
SET lowDvWarned TO FALSE.
SET reportT     TO TIME:SECONDS.

UNTIL SHIP:APOAPSIS >= targetAp {

  // ---- guidance: hold a target time-to-apoapsis ---------------------------
  //  Below the target the trajectory is too flat to keep the apoapsis rising;
  //  above it we are spending thrust on altitude we already have.  Holding
  //  ~TGT_ETA_AP is the cheap path from "fast and low" to "in orbit".
  LOCAL etaAp IS ETA:APOAPSIS.
  IF SHIP:APOAPSIS < SHIP:ALTITUDE + 100 { SET etaAp TO 0. }   // still climbing hard
  IF etaAp < TGT_ETA_AP - 5 { SET pitchCmd TO pitchCmd + 0.6. }
  ELSE IF etaAp > TGT_ETA_AP + 5 { SET pitchCmd TO pitchCmd - 0.6. }
  IF SHIP:ALTITUDE > ATM_TOP { SET pitchCmd TO pitchCmd - 0.3. }  // go horizontal
  SET pitchCmd TO clampVal(pitchCmd, CC_PITCH_MIN, CC_PITCH_MAX).

  // ---- dV policing --------------------------------------------------------
  LOCAL dvNow   IS rocketDv().
  LOCAL price   IS priceOfOrbit(targetAp).
  LOCAL bestAlt IS MAX(SHIP:ALTITUDE, SHIP:APOAPSIS).

  IF dvNow < price {
    IF bestAlt > SETTLE_MIN_ALT {
      // Above the settle floor and no longer able to afford the climb: stop
      // pushing and keep the circularisation + deorbit money.
      PRINT "======================================================".
      PRINT "!! NOT ENOUGH dV TO CLIMB TO " + ROUND(targetAp / 1000, 1) + " km.".
      PRINT "   " + budgetLine(targetAp).
      PRINT "   Best altitude " + ROUND(bestAlt / 1000, 1) + " km is above the " +
            ROUND(SETTLE_MIN_ALT / 1000) + " km floor, so settling for the".
      PRINT "   orbit we can afford and protecting circularisation + deorbit dV.".
      PRINT "======================================================".
      SET targetAp TO SHIP:APOAPSIS.
      SET settled TO TRUE.
      BREAK.
    } ELSE IF NOT lowDvWarned {
      // Too low to settle for anything: a lower orbit is not on offer yet, so
      // keep burning and get the apoapsis out of the atmosphere first.
      PRINT "!! dV budget short at " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km, below the " +
            ROUND(SETTLE_MIN_ALT / 1000) + " km settle floor.".
      PRINT "   Continuing the climb; the reserve is re-checked continuously.".
      PRINT "   " + budgetLine(targetAp).
      SET lowDvWarned TO TRUE.
    }
  }

  // Absolute floor: never burn the ship dry.
  IF dvNow <= DV_HARD_FLOOR {
    PRINT "!! Propellant exhausted during the climb.".
    SET targetAp TO SHIP:APOAPSIS.
    SET settled TO TRUE.
    BREAK.
  }

  // ---- periodic status ----------------------------------------------------
  IF TIME:SECONDS - reportT > 10 {
    PRINT "  AP " + ROUND(SHIP:APOAPSIS / 1000, 1) + " km | alt " +
          ROUND(SHIP:ALTITUDE / 1000, 1) + " km | pitch " + ROUND(pitchCmd, 1) +
          " | TWR " + ROUND(twrNow(), 2) + " | dV " + ROUND(dvNow).
    SET reportT TO TIME:SECONDS.
  }

  WAIT 0.2.
}

SET thrCmd TO 0.
SET orbitAlt TO SHIP:APOAPSIS.
PRINT "MECO at apoapsis " + ROUND(orbitAlt / 1000, 1) + " km. " + budgetLine(orbitAlt).

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
  PRINT "======================================================".
  UNLOCK STEERING.
  UNLOCK THROTTLE.
  SAS ON.
}

IF NOT abortSuborbital {
  LOCK STEERING TO SHIP:PROGRADE.
  WAIT UNTIL SHIP:ALTITUDE > ATM_TOP.           // clear of the atmosphere
  PRINT "Vacuum. ETA to apoapsis: " + ROUND(ETA:APOAPSIS) + " s.".

  // Start a little before apoapsis so the burn straddles it.
  WAIT UNTIL ETA:APOAPSIS < CIRC_LEAD OR SHIP:PERIAPSIS > ATM_TOP.

  // -------------------------------------------------------------------------
  //  5. CIRCULARISE  --  and stop at the deorbit reserve, whatever happens
  // -------------------------------------------------------------------------
  SET circPeTarget TO MAX(ATM_TOP + PE_SAFETY, orbitAlt - CIRC_PE_TOL).
  SET circPeTarget TO MIN(circPeTarget, orbitAlt - 500).
  PRINT "Circularising at " + ROUND(orbitAlt / 1000, 1) + " km; PE target " +
        ROUND(circPeTarget / 1000, 1) + " km.".
  PRINT "  Protecting " + ROUND(reserveDvFor(orbitAlt)) + " m/s (deorbit " +
        ROUND(deorbitDvFrom(orbitAlt)) + " + margin " + ROUND(DV_MARGIN) + ").".

  SET circStarved TO FALSE.
  SET circFloor   TO reserveDvFor(orbitAlt).   // apoapsis barely moves during the burn
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
  PRINT "  dV left    : " + ROUND(dvLeft) + " m/s".
  PRINT "  Deorbit    : " + ROUND(dvDeorb) + " m/s  ->  spare after deorbit " +
        ROUND(dvLeft - dvDeorb) + " m/s".
  PRINT "  Fuel -- LiquidFuel: " + ROUND(resAmt("LiquidFuel")) +
        " , Oxidizer: " + ROUND(resAmt("Oxidizer")) +
        " , Monoprop: " + ROUND(resAmt("MonoPropellant")).
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
