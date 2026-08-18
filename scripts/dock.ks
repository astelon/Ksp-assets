// ============================================================================
//  dock.ks  --  Port-to-port docking, from alongside to hard dock
// ----------------------------------------------------------------------------
//  Target : a DOCKING PORT on another vessel - not the vessel, the port.
//           Right-click it in flight (or pick it on the map) and "Set as
//           Target".  Control this ship from its own docking port: right-click
//           that port and "Control From Here".  If you forget, the script does
//           it for you whenever it can identify the port without guessing.
//
//  It starts where rendezvous.ks stops - a few hundred metres off with the
//  relative velocity dead - but it does not assume any of that.  It will start
//  from anywhere inside physics range, from any attitude, from in front of the
//  port or from behind the station, with the two ships pointing wherever they
//  happen to be pointing.
//
//  Run with:   RUN dock.              - both ports as they are set up in game
//              RUN dock("dorsal").    - name our port (part tag or title)
//              RUN dock("", 60).      - ...and force the standoff distance (m)
//
//  Design notes (why it flies the way it does)
//  -------------------------------------------
//  * DOCKING HAPPENS IN THE TARGET PORT'S FRAME, NOT THE TARGET VESSEL'S.  A
//    rendezvous is finished when two vessels share a position; a docking is
//    finished when two *ports* share a position, an axis and a roll.  So every
//    quantity below is measured from the target port's node - the axial gap
//    along its outward normal, the lateral offset across it, and the angle
//    between the two port axes.  The vessels' own attitudes never appear.
//  * WE FLY TO THE PORT'S AXIS FIRST AND DOWN IT SECOND.  The only direction a
//    port can be entered from is straight out in front of it.  Arriving from
//    anywhere else means crossing the station, so the approach is routed: out
//    to a clear radius, forward past the port's plane, in along the axis.  Each
//    leg either increases the distance to the station or holds it constant, so
//    the path cannot cut a corner through a solar panel.
//  * THE KEEP-OUT SPHERE IS MEASURED, NOT ASSUMED.  Both radii come from the
//    part trees - the farthest part of the target from its port, the farthest
//    part of ours from ours - so a 200-part station gets a wide berth and a
//    probe gets a small one, without anyone retuning anything.  A script that
//    hard-codes "stand off 50 m" is one big station away from a collision.
//  * THE APPROACH SPEED IS WHAT THE RCS CAN STOP FROM.  Same law rendezvous.ks
//    flies by, for the same reason: closing speed is capped at
//    sqrt(2*a*d/safety) where a is the translational acceleration the thrusters
//    *actually* deliver, measured in flight rather than read off a nameplate.
//    A loaded Mk3 approaches at a quarter of the speed of an empty one because
//    it is a quarter as able to stop.
//  * IT NEVER CLOSES WHILE MISALIGNED.  The final approach lives inside a cone:
//    the lateral error allowed shrinks with the range left, and the axial
//    closing rate is held at zero whenever the ship is outside the cone or the
//    port axes are more than a few degrees apart.  Correcting a lateral error
//    at 30 m costs a few centimetres per second; discovering it at 30 cm costs
//    a bent port.  The cone is the whole difference between a docking and a
//    collision that happened to be slow.
//  * THE LAST HALF METRE BELONGS TO THE MAGNETS.  Inside the port's own acquire
//    range the thrusters stop commanding anything and the ship coasts; the
//    magnets are stronger and better aimed than RCS is at that distance.  If
//    they do not bite, the script backs off and re-flies the approach rather
//    than pushing harder.
//  * MONOPROPELLANT IS THE CURRENCY, AND IT IS PRICED BEFORE ANYTHING MOVES.
//    The whole approach is flown on RCS.  Every leg is costed at two speed
//    changes (get moving, stop again), the bill is compared with what is in the
//    tanks, and the answer is GO / MARGINAL / NOT ENOUGH before the first
//    thruster fires.  Running mono dry ten metres off a station is worse than
//    running it dry a kilometre off one.
//  * THE MAIN ENGINE IS NEVER USED.  It points backwards out of a ship whose
//    nose is on the station, and at these distances a rocket motor is not a
//    docking aid.  If the ship is closing faster than the RCS can stop it, the
//    script nulls the relative velocity and re-flies the approach; there is
//    room to do that because the keep-out sphere is where the braking distance
//    is measured to, not the hull.
//
//  Flight plan
//    0. Pre-flight  : identify both ports, validate them against each other,
//                     measure both ships, price the mono bill, GO / NO-GO.
//    1. Set up      : open our shield if it has one, control from the port,
//                     match roll to the target, RCS on.
//    2. Corridor    : route to the standoff point on the target port's axis -
//                     out, forward, in - never through the station.
//    3. Align       : hold the standoff, null the relative velocity, bring the
//                     port axes anti-parallel.
//    4. Final       : close along the axis inside a cone that narrows with the
//                     range left; lateral error is corrected before axial gap.
//    5. Capture     : coast inside the acquire range and let the magnets pull.
//    6. Report      : resources, what is docked to what, what to do next.
//
//  Tunables are grouped at the top so you can trim them to your build.
// ============================================================================

// Our port, by part TAG (set one in the editor) or by part title - a substring
// is enough, and an ambiguous one is refused rather than guessed at.  Leave it
// out and the script uses the port you are controlling from.
//
// (Optional program parameters need kOS 1.0.1 or newer.  On anything older,
// drop the "IS" defaults and always pass both values.)
DECLARE PARAMETER portHint IS "".
DECLARE PARAMETER standoffMetres IS 0.

CLEARSCREEN.

// --- The corridor -----------------------------------------------------------
SET STANDOFF_MIN  TO 35.    // floor on the corridor entry distance (m).  The
                            // real one is measured from both ships and is
                            // bigger than this whenever either ship is
SET CLEARANCE     TO 10.    // slack added to every measured radius (m)
SET KEEPOUT_MIN   TO 20.    // floor on the keep-out sphere around the target
                            // port, whatever the part tree says (m)
SET STANDOFF_TOL  TO 2.5.   // how near the entry point counts as arrived (m)
SET CORR_CONE     TO 0.25.  // the corridor: lateral room allowed at a given
SET CORR_R_MIN    TO 1.5.   // axial range, as radius = cone*range, floored (m)
SET LAT_CONE      TO 0.05.  // the docking cone, same shape but much tighter -
SET LAT_TOL_MIN   TO 0.12.  // this one gates the axial closing rate (m)

// --- Alignment --------------------------------------------------------------
SET CTRL_PORT_TOL TO 5.     // how closely a port's axis must lie along the
                            // ship's forward axis to be recognised as the one
                            // the ship is being controlled from (deg)
SET ALIGN_TOL     TO 2.5.   // port-axis error accepted before closing (deg)
SET ALIGN_HOLD    TO 6.     // ... and the error that pauses a close already
                            // under way, so the two do not chatter (deg)
SET ROLL_MATCH    TO TRUE.  // hold our port's roll on the target port's.
                            // Stock ports mate at any roll; matching anyway
                            // keeps our translation axes still relative to the
                            // station, which makes every correction smaller
SET ROT_SAMPLE    TO 2.     // seconds spent measuring the target's rotation
SET ROT_WARN      TO 0.6.   // rotation rate that earns a warning (deg/s)
SET ROT_REFUSE    TO 4.     // ... and the one that stops the script (deg/s)

// --- Speeds -----------------------------------------------------------------
SET V_CORRIDOR    TO 5.     // cap while routing to the standoff point (m/s)
SET V_FINAL       TO 0.8.   // cap on the axial approach (m/s)
SET V_CREEP_ZONE  TO 3.     // inside this range the cap drops again (m)
SET V_CAPTURE     TO 0.25.  // ... to this (m/s)
SET V_CREEP       TO 0.08.  // never close slower than this once aligned, or an
                            // approach can converge to a hover (m/s)
SET V_LAT         TO 0.5.   // cap on lateral corrections (m/s)
SET KP_POS        TO 0.25.  // commanded speed per metre of position error
SET KP_LAT        TO 0.35.  // ... on the lateral axis, where it matters more
SET HOLD_V_TOL    TO 0.15.  // relative speed that counts as "stopped" (m/s)
SET BRAKE_SAFETY  TO 2.5.   // braking distance is padded by this factor

// --- Capture ----------------------------------------------------------------
SET CAPTURE_GAP   TO 0.     // node-to-node gap at which we stop pushing and
                            // let the magnets work (m).  0 = ask the port for
                            // its own acquire range, which is the honest answer
SET CAPTURE_MIN   TO 0.30.  // ... floored here if the port will not say (m)
SET CAPTURE_LAT   TO 0.20.  // lateral error that still counts as on-axis (m)
SET CAPTURE_WAIT  TO 20.    // how long to wait for the magnets to bite (s)
SET CAPTURE_TRIES TO 3.     // re-approaches allowed before giving up

// --- RCS and control --------------------------------------------------------
SET TRANS_GAIN    TO 6.     // RCS command per m/s of velocity error.  Much
                            // higher than the 1.2 rendezvous.ks parks on, and
                            // deliberately: out there a lazy 30% command costs
                            // a slow approach, whereas here it costs a ship
                            // that cannot hold the brake schedule it computed
                            // and arrives at the port still moving.  At 6, a
                            // sixth of a metre per second of error is already
                            // full authority
SET TRANS_DEAD    TO 0.02.  // velocity error below which an axis is left alone
                            // (m/s).  NOTE this is in m/s of error - the
                            // TRANS_DEAD in rendezvous.ks is in command units.
                            // Ten of these is one centimetre per second, which
                            // is below anything worth spending mono on
SET RCS_ACC_MIN   TO 0.02.  // never believe an RCS accel lower than this
SET RCS_THRUST_BLOCK TO 1.0.// thrust per RCS block (kN) - RV-105 is 1 kN
SET RCS_AXIS_FRAC TO 0.5.   // fraction of the installed blocks that push along
                            // any one axis.  A guess, and only ever the
                            // starting guess: it is replaced by measurement as
                            // soon as the ship moves
SET RCS_ISP_FALLBACK TO 240.// monopropellant thruster Isp (s)
SET MONO_RESERVE  TO 8.     // stop closing with less mono than this (units)
SET MONO_TRIM     TO 6.     // allowance for attitude, roll and station-keeping
                            // on top of the translation bill (m/s)
SET GO_MARGIN     TO 0.15.  // mono in hand, as a fraction of the bill, below
                            // which the answer is MARGINAL rather than GO
SET PREFLIGHT_HOLD TO 12.   // pause this long on a failed check (s)
SET STEER_STOP_T  TO 2.     // steering manager stopping time (s).  Slower than
                            // the default on purpose: a gentle rotation spends
                            // less mono and disturbs the translation less

// --- Safety and timeouts ----------------------------------------------------
SET MAX_START_RANGE TO 2000.// refuse to start farther out than this (m).  Past
                            // physics range the target's parts are not loaded
                            // and its port has no position to fly to
SET VREL_MAX_START TO 15.   // refuse to start with more relative speed than
                            // this (m/s) - that is a rendezvous, not a docking
SET T_CORRIDOR    TO 900.   // per-phase timeouts (s)
SET T_ALIGN       TO 300.
SET T_FINAL       TO 900.
SET T_TOTAL       TO 2400.  // ... and one on the whole job
SET SHIELD_WAIT   TO 8.     // seconds allowed for a shield animation

// --- Logging ----------------------------------------------------------------
SET LOG_TO_FILE   TO FALSE. // also append the resource snapshots to a file
SET LOG_PATH      TO "0:/dock_log.csv".

// ---------------------------------------------------------------------------
//  Constants and housekeeping
// ---------------------------------------------------------------------------
SET G0 TO 9.80665.

// kOS runs a fixed number of instructions per physics tick.  The control loop
// below reads a dozen part positions per pass and must not fall behind the
// physics it is steering against.  Restored on exit.
SET IPU_SAVED TO CONFIG:IPU.
SET CONFIG:IPU TO 1000.

SET STEER_SAVED TO 0.
SET STEER_TOUCHED TO FALSE.

// ---------------------------------------------------------------------------
//  Small helpers
// ---------------------------------------------------------------------------
//  NOTE ON NAMING: kOS refuses to compile a script that declares a variable
//  hiding one of its built-in functions (the CLOBBERBUILTINS check), and the
//  list is longer than it looks - NODE, LIST, BODY, STAGE, TRANSFER and VESSEL
//  are all built-ins, and all of them are words you would reach for while
//  writing a docking script.  Run `tools/check_kos.py` after editing; it checks
//  before the game does.

// Clamp `valIn` into [loV, hiV].  Same argument order as rdvClampVal and the
// opposite of dlClampVal in deorbit_land.ks - keep them straight when copying
// a line between files.
FUNCTION dckClampVal {
  PARAMETER valIn, loV, hiV.
  RETURN MAX(loV, MIN(hiV, valIn)).
}

// A unit vector perpendicular to `axV`, for the rare case where the geometry
// gives us no lateral direction of its own to use (dead on the port's axis but
// behind it, which means inside the station).
FUNCTION sideStepHat {
  PARAMETER axV.
  LOCAL cand IS VXCL(axV, SHIP:UP:VECTOR).
  IF cand:MAG < 0.1 { SET cand TO VXCL(axV, NORTH:VECTOR). }
  IF cand:MAG < 0.1 { SET cand TO VXCL(axV, SHIP:FACING:STARVECTOR). }
  IF cand:MAG < 0.1 { RETURN V(1, 0, 0). }
  RETURN cand:NORMALIZED.
}

// ---------------------------------------------------------------------------
//  Docking-port plumbing
//
//  Every suffix used here is checked with HASSUFFIX first.  kOS has grown its
//  DockingPort structure over the years and this repository targets KSP 1.2,
//  where an older kOS is entirely plausible; a suffix that is not there must
//  cost a fallback rather than a crashed script two metres off a station.
// ---------------------------------------------------------------------------

// Is this thing a docking port?  NODEPOSITION is the discriminator: vessels,
// bodies and ordinary parts do not have one.
FUNCTION isDockPort {
  PARAMETER thing.
  RETURN thing:HASSUFFIX("NODEPOSITION") AND thing:HASSUFFIX("STATE").
}

// The direction the port faces, i.e. the way a ship must come from to dock.
// PORTFACING exists for exactly this reason - a docking port's *part* facing is
// not the direction it mates along.  Failing that, the node sits proud of the
// part's centre along that same axis, which gives the vector geometrically.
FUNCTION portAxisOf {
  PARAMETER prt.
  IF prt:HASSUFFIX("PORTFACING") { RETURN prt:PORTFACING:FOREVECTOR. }
  LOCAL offs IS prt:NODEPOSITION - prt:POSITION.
  IF offs:MAG > 0.02 { RETURN offs:NORMALIZED. }
  RETURN prt:FACING:FOREVECTOR.
}

// The port's roll reference, used only to keep our translation axes still
// relative to the station.  Stock ports do not care about roll.
FUNCTION portUpOf {
  PARAMETER prt.
  IF prt:HASSUFFIX("PORTFACING") { RETURN prt:PORTFACING:TOPVECTOR. }
  RETURN sideStepHat(portAxisOf(prt)).
}

FUNCTION portState {
  PARAMETER prt.
  RETURN prt:STATE:TOLOWER.
}

FUNCTION portOccupied {
  PARAMETER prt.
  LOCAL st IS portState(prt).
  RETURN st:CONTAINS("docked") OR st:CONTAINS("preattached").
}

FUNCTION portReady {
  PARAMETER prt.
  RETURN portState(prt) = "ready".
}

// A shielded port reports "disabled" until its shield is open, which is the
// tidiest possible way to be told to open it.
FUNCTION portShielded {
  PARAMETER prt.
  RETURN portState(prt):CONTAINS("disabled").
}

FUNCTION portNodeType {
  PARAMETER prt.
  IF prt:HASSUFFIX("NODETYPE") { RETURN prt:NODETYPE:TOLOWER. }
  RETURN "".
}

// The distance at which the magnets take over.  kOS spelled this suffix without
// the first 'c' until 0.18 and with it ever since, so both names have to be
// allowed for - but the ORDER matters, and not for the reason it looks like.
//
// The obsolete AQUIRERANGE is still *registered* on modern kOS: HASSUFFIX says
// yes and then reading it throws "as of kOS 0.18.0, AQUIRERANGE is obsolete".
// A suffix existing is not the same as a suffix being readable, which is a
// sharper version of the trap the rest of this file's HASSUFFIX guards exist
// for.  So ask for the current name first and only reach for the old one when
// the current one is genuinely absent - which can only be an install older than
// 0.18, where the old name is the real one and reading it is safe.
FUNCTION portGrabRange {
  PARAMETER prt.
  IF prt:HASSUFFIX("ACQUIRERANGE") { RETURN prt:ACQUIRERANGE. }
  IF prt:HASSUFFIX("AQUIRERANGE")  { RETURN prt:AQUIRERANGE. }
  RETURN 0.
}

// Does this port have a shield at all?  Whichever way it is currently sitting,
// it offers the event for the other way round.
FUNCTION portHasShield {
  PARAMETER prt.
  FOR mname IN prt:MODULES {
    LOCAL pmod IS prt:GETMODULE(mname).
    IF pmod:HASEVENT("open shield")  { RETURN TRUE. }
    IF pmod:HASEVENT("close shield") { RETURN TRUE. }
  }
  RETURN FALSE.
}

// Open a shield if the part has one.  A closed shield offers an "open shield"
// event and an open one does not, so the event list is also the state test -
// no need to know which animation module a particular part or mod uses.
FUNCTION openPortShield {
  PARAMETER prt.
  LOCAL didIt IS FALSE.
  FOR mname IN prt:MODULES {
    LOCAL pmod IS prt:GETMODULE(mname).
    IF pmod:HASEVENT("open shield") {
      pmod:DOEVENT("open shield").
      SET didIt TO TRUE.
    } ELSE IF pmod:HASEVENT("deploy shield") {
      pmod:DOEVENT("deploy shield").
      SET didIt TO TRUE.
    } ELSE IF pmod:HASEVENT("open") {
      pmod:DOEVENT("open").
      SET didIt TO TRUE.
    }
  }
  RETURN didIt.
}

// Which vessel is a part on?  Part:SHIP answers directly on any recent kOS; the
// fallback walks the loaded vessels looking for the UID, which is slow but only
// ever runs once and only on installs old enough to need it.
FUNCTION vesselOfPart {
  PARAMETER prt.
  IF prt:HASSUFFIX("SHIP") { RETURN prt:SHIP. }
  LOCAL others IS LIST().
  LIST TARGETS IN others.
  FOR ves IN others {
    FOR other IN ves:PARTS {
      IF other:UID = prt:UID { RETURN ves. }
    }
  }
  RETURN SHIP.
}

// How far the ship extends from a given point - the radius of the smallest
// sphere centred there that contains every part.  This is where the keep-out
// distance comes from, for both ships.
FUNCTION spanAbout {
  PARAMETER prts, originVec.
  LOCAL worst IS 0.
  FOR prt IN prts {
    LOCAL dd IS (prt:POSITION - originVec):MAG.
    IF dd > worst { SET worst TO dd. }
  }
  RETURN worst.
}

// ---------------------------------------------------------------------------
//  Resources
//
//  Docking has one currency and it is monopropellant.  There is no payload
//  isolation to do here - unlike rendezvous.ks, nothing in this script touches
//  the main tanks - but the mono has to be counted honestly, because the whole
//  approach is flown out of it and there is no second way to stop.
// ---------------------------------------------------------------------------
FUNCTION dckResAmtShip {
  PARAMETER rname.
  LOCAL total IS 0.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = rname { SET total TO total + res:AMOUNT. }
  }
  RETURN total.
}

FUNCTION dckResDensity {               // tonnes per unit
  PARAMETER rname, fallback.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = rname AND res:DENSITY > 0 { RETURN res:DENSITY. }
  }
  RETURN fallback.
}

SET MONO_DENS TO dckResDensity("MonoPropellant", 0.004).

SET RCS_BLOCKS TO 0.
FOR prt IN SHIP:PARTS {
  FOR mname IN prt:MODULES {
    IF mname:TOLOWER:CONTAINS("modulercs") { SET RCS_BLOCKS TO RCS_BLOCKS + 1. }
  }
}

FUNCTION dckMonoDv {                   // what is in the mono tanks, as dV (m/s)
  LOCAL m0 IS SHIP:MASS.
  LOCAL m1 IS m0 - dckResAmtShip("MonoPropellant") * MONO_DENS.
  IF m0 <= 0 OR m1 <= 0 OR m1 >= m0 { RETURN 0. }
  RETURN RCS_ISP_FALLBACK * G0 * LN(m0 / m1).
}

// Translational acceleration, from the blocks installed and the mass aboard.
// Only some of the blocks push along any one axis, hence RCS_AXIS_FRAC - and
// this is only the opening guess.  The control loop measures what the ship
// really does and believes that instead.
SET rcsAccEst TO 0.15.

FUNCTION dckRcsAccelNominal {
  IF SHIP:MASS <= 0 { RETURN RCS_ACC_MIN. }
  RETURN MAX(RCS_ACC_MIN,
             RCS_BLOCKS * RCS_THRUST_BLOCK * RCS_AXIS_FRAC / SHIP:MASS).
}

FUNCTION dckApproachAccel {            // the figure every braking schedule uses
  RETURN MAX(RCS_ACC_MIN, rcsAccEst).
}

// What one leg of the approach costs: get moving, then stop again.  The speed
// it would be flown at is whichever of the cap and the braking schedule is
// lower, which is why a short leg is cheap and a long one saturates.
FUNCTION legCost {
  PARAMETER lenM, vCap.
  IF lenM <= 0.5 { RETURN 0. }
  LOCAL vLeg IS MIN(vCap, SQRT(2 * dckApproachAccel() * lenM / BRAKE_SAFETY)).
  RETURN 2 * vLeg.
}

SET logStarted TO FALSE.

FUNCTION dckResourceReport {
  PARAMETER label.
  LOCAL monoAmt IS dckResAmtShip("MonoPropellant").
  LOCAL ecAmt   IS dckResAmtShip("ElectricCharge").
  PRINT "------------------------------------------------------".
  PRINT "RESOURCES :: " + label.
  PRINT "  Mass  : " + ROUND(SHIP:MASS, 2) + " t".
  PRINT "  Mono  : " + ROUND(monoAmt, 1) + " u   ->  " + ROUND(dckMonoDv(), 1) +
        " m/s on RCS".
  PRINT "  EC    : " + ROUND(ecAmt, 1) + " u".
  PRINT "  RCS   : " + RCS_BLOCKS + " block(s), " +
        ROUND(dckApproachAccel(), 3) + " m/s^2 of translation".
  PRINT "------------------------------------------------------".

  IF LOG_TO_FILE {
    IF NOT logStarted {
      LOG "ut,label,mass_t,mono_u,ec_u,mono_dv_ms,rcs_acc" TO LOG_PATH.
      SET logStarted TO TRUE.
    }
    LOG ROUND(TIME:SECONDS, 1) + "," + label + "," + ROUND(SHIP:MASS, 3) + "," +
        ROUND(monoAmt, 2) + "," + ROUND(ecAmt, 2) + "," + ROUND(dckMonoDv(), 1) +
        "," + ROUND(dckApproachAccel(), 4) TO LOG_PATH.
  }
}

FUNCTION dckHandBack {
  SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
  LOCK THROTTLE TO 0.
  UNLOCK THROTTLE.
  UNLOCK STEERING.
  SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
  IF STEER_TOUCHED { SET STEERINGMANAGER:MAXSTOPPINGTIME TO STEER_SAVED. }
  SET CONFIG:IPU TO IPU_SAVED.
  SAS ON.
}

// ---------------------------------------------------------------------------
//  Geometry
//
//  All of it lives in the target port's frame:
//
//        axGap  : how far out in front of the target port we are, measured
//                 along its outward normal.  Negative means behind its plane,
//                 i.e. somewhere alongside or under the station.
//        latVec : how far off its axis we are, across it.  Zero is the
//                 corridor centreline.
//        alignErr : the angle between the two port axes once one is reversed.
//                 Zero is face to face.
//
//  A docking is finished when all three are zero at the same time, and the
//  order they are driven to zero in is what makes it safe: lateral first,
//  angle second, axial last.
// ---------------------------------------------------------------------------
SET tgtNodePos TO V(0, 0, 0).
SET tgtAxis    TO V(0, 0, 1).
SET ourNodePos TO V(0, 0, 0).
SET ourAxis    TO V(0, 0, 1).
SET sepVec     TO V(0, 0, 0).
SET axGap      TO 0.
SET latVec     TO V(0, 0, 0).
SET latGap     TO 0.
SET alignErr   TO 180.
SET vRel       TO V(0, 0, 0).
SET aimIsEntry TO FALSE.

// Read the whole geometry off the two ports, once per pass.  Everything below
// works from these and nothing re-reads a part position, so one tick's worth of
// control decisions is all taken against one consistent picture.
FUNCTION refreshGeom {
  SET tgtNodePos TO tgtPort:NODEPOSITION.
  SET tgtAxis    TO portAxisOf(tgtPort).
  SET ourNodePos TO ourPort:NODEPOSITION.
  SET ourAxis    TO portAxisOf(ourPort).
  SET sepVec     TO ourNodePos - tgtNodePos.
  SET axGap      TO VDOT(sepVec, tgtAxis).
  SET latVec     TO VXCL(tgtAxis, sepVec).
  SET latGap     TO latVec:MAG.
  SET alignErr   TO 180 - VANG(ourAxis, tgtAxis).
  SET vRel       TO relVel().
}

// Corridor half-width at a given axial range: wide far out, tight close in.
FUNCTION corridorRadiusAt {
  PARAMETER axIn.
  RETURN MAX(CORR_R_MIN, CORR_CONE * MAX(0, axIn)).
}

// The docking cone - the same shape an order of magnitude tighter.  This is the
// one that gates the closing rate.
FUNCTION latTolAt {
  PARAMETER axIn.
  RETURN MAX(LAT_TOL_MIN, LAT_CONE * MAX(0, axIn)).
}

// Are we in the pipe?  Only inside it may the ship close on the port; outside
// it, the keep-out sphere applies instead.
FUNCTION inCorridor {
  IF axGap <= 0 { RETURN FALSE. }
  RETURN latGap < corridorRadiusAt(axGap).
}

// Where to fly next, in three legs that between them cannot cross the station:
//
//   1. If we are level with the port's plane or behind it, and closer to the
//      axis than the keep-out radius, go straight out sideways.  Distance to
//      the station only increases.
//   2. Once out at a clear radius, slide forward past the port's plane at that
//      radius.  Distance to the station stays at least the keep-out radius.
//   3. In front of the plane, aim at the entry point itself.  Every point on
//      that line is at least STANDOFF out in front of the port, because both
//      of its ends are.
//
// The legs are not a plan; they are recomputed from live geometry every pass,
// so being pushed off one simply puts the ship back on the appropriate one.
FUNCTION corridorAim {
  LOCAL entryPt IS tgtNodePos + tgtAxis * STANDOFF.
  IF axGap >= STANDOFF * 0.95 {
    SET aimIsEntry TO TRUE.
    RETURN entryPt.
  }
  SET aimIsEntry TO FALSE.
  LOCAL latHat IS sideStepHat(tgtAxis).
  IF latGap > 1 { SET latHat TO latVec:NORMALIZED. }
  IF latGap < KEEP_R * 0.95 {
    // Leg 1: straight out sideways, holding station along the axis.
    RETURN tgtNodePos + tgtAxis * axGap + latHat * KEEP_R.
  }
  // Leg 2: forward at a radius that clears everything.
  RETURN tgtNodePos + tgtAxis * STANDOFF + latHat * MAX(latGap, KEEP_R).
}

// Relative velocity, ours minus theirs, at the centres of mass.  The port nodes
// differ from this by the rotation of each ship about its own centre, which is
// small for anything that is not spinning - and anything that *is* spinning was
// refused at pre-flight.
FUNCTION relVel {
  RETURN SHIP:VELOCITY:ORBIT - tgtVes:VELOCITY:ORBIT.
}

// Where the nose goes: our port's axis, reversed onto the target port's, with
// the roll matched to it so the ship's lateral axes stay still relative to the
// station.  Re-evaluated by the steering lock every tick, so it tracks a
// station that is slowly turning.
FUNCTION dockSteer {
  LOCAL aimV IS portAxisOf(tgtPort) * -1.
  IF NOT ROLL_MATCH { RETURN LOOKDIRUP(aimV, sideStepHat(aimV)). }
  LOCAL upV IS portUpOf(tgtPort).
  IF VANG(upV, aimV) < 5 OR VANG(upV, aimV) > 175 {
    SET upV TO sideStepHat(aimV).
  }
  RETURN LOOKDIRUP(aimV, upV).
}

// One command covers closing, braking and lateral nulling alike: fly the
// velocity we want to have, minus the one we have got, on whichever thrusters
// point that way.  Returns TRUE if any axis was commanded to the stop, which is
// the only condition under which the acceleration measurement means anything.
FUNCTION axisCmd {
  PARAMETER vErr, axisVec.
  LOCAL comp IS VDOT(vErr, axisVec).
  IF ABS(comp) < TRANS_DEAD { RETURN 0. }
  RETURN dckClampVal(comp * TRANS_GAIN, -1, 1).
}

FUNCTION translateTo {
  PARAMETER vWant, vNow.
  LOCAL vErr IS vWant - vNow.
  LOCAL fCmd IS axisCmd(vErr, SHIP:FACING:FOREVECTOR).
  LOCAL sCmd IS axisCmd(vErr, SHIP:FACING:STARVECTOR).
  LOCAL tCmd IS axisCmd(vErr, SHIP:FACING:TOPVECTOR).
  SET SHIP:CONTROL:FORE      TO fCmd.
  SET SHIP:CONTROL:STARBOARD TO sCmd.
  SET SHIP:CONTROL:TOP       TO tCmd.
  RETURN ABS(fCmd) > 0.95 OR ABS(sCmd) > 0.95 OR ABS(tCmd) > 0.95.
}

// Docked yet?  The port says so itself, and the part count says so
// independently - two vessels that dock become one, and ours grows by the
// other's part count.  Either is enough; both together survive a kOS version
// that words the state string differently.
SET tPartsChk TO 0.

FUNCTION dockedNow {
  IF portOccupied(ourPort) { RETURN TRUE. }
  // The part count is the independent check, but rebuilding the parts list is
  // the most expensive thing in this loop - half a second apart is plenty.
  IF TIME:SECONDS > tPartsChk {
    SET tPartsChk TO TIME:SECONDS + 0.5.
    IF SHIP:PARTS:LENGTH > PARTS_AT_START { RETURN TRUE. }
  }
  RETURN FALSE.
}

// ---------------------------------------------------------------------------
//  Which of our ports are we docking with?
//
//  In order of trust: the one you named, the one you are controlling from, the
//  only one there is.  The last resort is inference from attitude - "control
//  from here" on a port makes the ship's forward axis that port's axis, so the
//  port whose axis lies along SHIP:FACING is the one you picked.  A guess
//  between two of them is refused; docking the wrong end of a spaceplane into a
//  station is not an error you want a script to make on your behalf.
// ---------------------------------------------------------------------------
FUNCTION portMatchesHint {
  PARAMETER prt, hint.
  LOCAL want IS hint:TOLOWER.
  IF prt:TAG:LENGTH > 0 AND prt:TAG:TOLOWER:CONTAINS(want) { RETURN TRUE. }
  IF prt:TITLE:TOLOWER:CONTAINS(want) { RETURN TRUE. }
  RETURN prt:NAME:TOLOWER:CONTAINS(want).
}

FUNCTION freePorts {
  LOCAL out IS LIST().
  FOR prt IN SHIP:DOCKINGPORTS {
    IF NOT portOccupied(prt) { out:ADD(prt). }
  }
  RETURN out.
}

FUNCTION pickOurPort {
  PARAMETER hint.
  LOCAL cands IS freePorts().
  IF cands:LENGTH = 0 {
    PRINT "!! This ship has no free docking port.".
    PRINT "   Every port it has is already docked, or it has none at all.".
    RETURN FALSE.
  }

  IF hint <> "" {
    LOCAL hits IS LIST().
    FOR prt IN cands {
      IF portMatchesHint(prt, hint) { hits:ADD(prt). }
    }
    IF hits:LENGTH = 1 {
      SET ourPort TO hits[0].
      PRINT "Our port: " + ourPort:TITLE + " (matched """ + hint + """).".
      RETURN TRUE.
    }
    IF hits:LENGTH = 0 {
      PRINT "!! No free docking port matches """ + hint + """. Ports aboard:".
      FOR prt IN cands { PRINT "     " + prt:TITLE + "   tag: """ + prt:TAG + """". }
      RETURN FALSE.
    }
    PRINT "!! """ + hint + """ is ambiguous - " + hits:LENGTH + " ports match:".
    FOR prt IN hits { PRINT "     " + prt:TITLE + "   tag: """ + prt:TAG + """". }
    PRINT "   Give a part tag that picks one out.".
    RETURN FALSE.
  }

  // No hint.  Ask the game which part we are controlling from, if it will say.
  IF SHIP:HASSUFFIX("CONTROLPART") {
    LOCAL ctl IS SHIP:CONTROLPART.
    FOR prt IN cands {
      IF prt:UID = ctl:UID {
        SET ourPort TO prt.
        PRINT "Our port: " + ourPort:TITLE + " (the one you are controlling from).".
        RETURN TRUE.
      }
    }
  }

  IF cands:LENGTH = 1 {
    SET ourPort TO cands[0].
    PRINT "Our port: " + ourPort:TITLE + " (the only free one aboard).".
    RETURN TRUE.
  }

  // Infer it from the control axis instead.
  LOCAL bestP IS cands[0].
  LOCAL bestA IS 999.
  LOCAL nextA IS 999.
  FOR prt IN cands {
    LOCAL aa IS VANG(portAxisOf(prt), SHIP:FACING:FOREVECTOR).
    IF aa < bestA {
      SET nextA TO bestA.
      SET bestA TO aa.
      SET bestP TO prt.
    } ELSE IF aa < nextA { SET nextA TO aa. }
  }
  IF bestA <= CTRL_PORT_TOL AND nextA > CTRL_PORT_TOL * 3 {
    SET ourPort TO bestP.
    PRINT "Our port: " + ourPort:TITLE + " (it is the one the ship is " +
          "controlling from - its axis is " + ROUND(bestA, 1) + " deg off the nose).".
    RETURN TRUE.
  }
  PRINT "!! Cannot tell which of " + cands:LENGTH + " ports you mean:".
  FOR prt IN cands {
    PRINT "     " + prt:TITLE + "   " +
          ROUND(VANG(portAxisOf(prt), SHIP:FACING:FOREVECTOR), 1) +
          " deg off the nose,  tag: """ + prt:TAG + """".
  }
  PRINT "   Right-click the one you want -> Control From Here, and re-run;".
  PRINT "   or name it:  RUN dock(""dorsal"").".
  RETURN FALSE.
}

// ---------------------------------------------------------------------------
//  0. PRE-FLIGHT  --  identify both ports, measure both ships, price the job
// ---------------------------------------------------------------------------
PRINT "=== RAPIER SSTO :: docking autopilot ===".
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
LOCK THROTTLE TO 0.
SAS OFF.

SET PARTS_AT_START TO SHIP:PARTS:LENGTH.
SET tgtPort  TO SHIP.               // replaced below; typed placeholders only
SET ourPort  TO SHIP.
SET tgtVes   TO SHIP.
SET STANDOFF TO STANDOFF_MIN.
SET KEEP_R   TO KEEPOUT_MIN.
SET GRAB_GAP TO CAPTURE_MIN.
SET okToGo   TO TRUE.

// --- 0a. The target has to be a port, on somebody else, and free ------------
IF NOT HASTARGET {
  PRINT "!! Nothing is targeted.".
  PRINT "   Right-click the docking port you want to dock WITH and pick".
  PRINT "   ""Set as Target"" - the port, not the vessel. Then RUN dock.".
  SET okToGo TO FALSE.
} ELSE IF NOT isDockPort(TARGET) {
  PRINT "!! The target is not a docking port.".
  PRINT "   A vessel is a rendezvous target; docking needs the port itself.".
  PRINT "   Right-click the port -> Set as Target, then RUN dock.".
  PRINT "   (If you are still kilometres out, RUN rendezvous. first.)".
  SET okToGo TO FALSE.
} ELSE {
  SET tgtPort TO TARGET.
}

IF okToGo {
  LOCAL ours IS LEXICON().
  FOR prt IN SHIP:PARTS { ours:ADD(prt:UID, TRUE). }
  IF ours:HASKEY(tgtPort:UID) {
    PRINT "!! That port is on this ship. Target the other vessel's port.".
    SET okToGo TO FALSE.
  }
}

IF okToGo {
  SET tgtVes TO vesselOfPart(tgtPort).
  IF portOccupied(tgtPort) {
    PRINT "!! " + tgtPort:TITLE + " on " + tgtVes:NAME + " is already docked" +
          " to something (state: " + tgtPort:STATE + ").".
    PRINT "   Pick a free port.".
    SET okToGo TO FALSE.
  } ELSE IF portShielded(tgtPort) {
    PRINT "!! " + tgtPort:TITLE + " on " + tgtVes:NAME + " is shielded and shut.".
    PRINT "   Its shield has to be opened from that craft - we cannot do it".
    PRINT "   from here. Open it, then re-run.".
    SET okToGo TO FALSE.
  }
}

IF okToGo {
  IF tgtVes:BODY:NAME <> SHIP:BODY:NAME {
    PRINT "!! " + tgtVes:NAME + " is at " + tgtVes:BODY:NAME + " and we are at " +
          SHIP:BODY:NAME + ".".
    SET okToGo TO FALSE.
  } ELSE IF tgtVes:HASSUFFIX("LOADED") {
    // Nested rather than ANDed: kOS makes no promise that the left-hand side
    // of an AND stops the right-hand side being evaluated, and the whole point
    // of the HASSUFFIX is that reading the suffix might not be safe.
    IF NOT tgtVes:LOADED {
      PRINT "!! " + tgtVes:NAME + " is not loaded - it is outside physics range,".
      PRINT "   so its port has no position for us to fly to. RUN rendezvous.".
      SET okToGo TO FALSE.
    }
  }
}

// --- 0b. Are we close enough, and slow enough, for this to be a docking? ----
IF okToGo {
  SET vRel TO relVel().
  LOCAL rangeNow IS (tgtPort:NODEPOSITION - SHIP:POSITION):MAG.
  IF rangeNow > MAX_START_RANGE {
    PRINT "!! " + ROUND(rangeNow) + " m out. That is a rendezvous, not a".
    PRINT "   docking - RUN rendezvous. first and re-run this after it parks.".
    SET okToGo TO FALSE.
  } ELSE IF vRel:MAG > VREL_MAX_START {
    PRINT "!! Closing at " + ROUND(vRel:MAG, 1) + " m/s. Too fast to dock from.".
    PRINT "   Kill the relative velocity first (rendezvous.ks parks with it".
    PRINT "   dead) and re-run.".
    SET okToGo TO FALSE.
  }
}

// --- 0c. Our port -----------------------------------------------------------
IF okToGo {
  IF NOT pickOurPort(portHint) { SET okToGo TO FALSE. }
}

IF okToGo {
 IF portShielded(ourPort) {
  PRINT "Our port is shielded and shut - opening it.".
  IF openPortShield(ourPort) {
    LOCAL tShield IS TIME:SECONDS + SHIELD_WAIT.
    WAIT UNTIL (NOT portShielded(ourPort)) OR TIME:SECONDS > tShield.
  }
  IF portShielded(ourPort) {
    PRINT "!! The shield did not open. Open it by hand (action group 3 on".
    PRINT "   this ship) and re-run.".
    SET okToGo TO FALSE.
  } ELSE {
    PRINT "  Shield open.".
  }
 }
}

IF okToGo {
  IF NOT portReady(ourPort) {
    PRINT "!! Our port reports """ + ourPort:STATE + """ rather than Ready.".
    PRINT "   If it has just undocked, back off a few metres and re-run.".
    SET okToGo TO FALSE.
  }
}

// --- 0d. Will these two ports actually mate? --------------------------------
IF okToGo {
  LOCAL ourType IS portNodeType(ourPort).
  LOCAL tgtType IS portNodeType(tgtPort).
  IF ourType <> "" AND tgtType <> "" AND ourType <> tgtType {
    PRINT "!! Port sizes do not match: ours is """ + ourType +
          """, theirs is """ + tgtType + """.".
    PRINT "   These two will never latch, however well they are lined up.".
    SET okToGo TO FALSE.
  }
}

// --- 0e. Can we translate at all? -------------------------------------------
IF okToGo {
  SET rcsAccEst TO dckRcsAccelNominal().
  IF RCS_BLOCKS = 0 {
    PRINT "!! No RCS thrusters aboard. The approach is flown entirely on RCS;".
    PRINT "   there is no way to fly it without them.".
    SET okToGo TO FALSE.
  } ELSE IF dckResAmtShip("MonoPropellant") <= MONO_RESERVE {
    PRINT "!! Monopropellant is down to " +
          ROUND(dckResAmtShip("MonoPropellant"), 1) + " u, at or below the " +
          MONO_RESERVE + " u reserve.".
    SET okToGo TO FALSE.
  }
}

// --- 0f. Measure both ships, and let them set the corridor ------------------
//  The keep-out sphere is drawn around the *target port's node*, because that
//  is the origin every other quantity is measured from.  Its radius is how far
//  the station reaches from that node, plus how far we reach from ours, plus
//  slack.  Standing off inside that radius means some part of one ship is
//  inside some part of the other.
IF okToGo {
  SET ourSpan TO spanAbout(SHIP:PARTS, ourPort:NODEPOSITION).
  SET tgtSpan TO spanAbout(tgtVes:PARTS, tgtPort:NODEPOSITION).
  SET KEEP_R  TO MAX(KEEPOUT_MIN, ourSpan + tgtSpan + CLEARANCE).
  // Clear of the sphere, not on its surface.  The standoff point is where the
  // ship comes to a stop, and stopping exactly on the boundary would leave the
  // braking guard below with no room at all to measure.
  SET STANDOFF TO MAX(STANDOFF_MIN, KEEP_R + CLEARANCE).
  IF standoffMetres > 0 {
    SET STANDOFF TO standoffMetres.
    IF STANDOFF < KEEP_R + CLEARANCE {
      PRINT "!! A " + ROUND(STANDOFF) + " m standoff is inside the " +
            ROUND(KEEP_R) + " m the two ships measure - using " +
            ROUND(KEEP_R + CLEARANCE) + " m.".
      SET STANDOFF TO KEEP_R + CLEARANCE.
    }
  }

  SET GRAB_GAP TO CAPTURE_GAP.
  IF GRAB_GAP <= 0 { SET GRAB_GAP TO portGrabRange(ourPort). }
  IF GRAB_GAP <= 0 { SET GRAB_GAP TO portGrabRange(tgtPort). }
  SET GRAB_GAP TO MAX(CAPTURE_MIN, GRAB_GAP).
}

// --- 0g. Is the target holding still? ---------------------------------------
//  A station that is slowly turning can be docked with - the steering lock and
//  the waypoints are recomputed from live geometry every pass, so they follow
//  it round.  One that is spinning cannot, and finding that out at two metres
//  is finding it out too late.  Measured rather than assumed: sample the port's
//  axis twice, a couple of seconds apart.
IF okToGo {
  SET axis0 TO portAxisOf(tgtPort).
  SET t0Rot TO TIME:SECONDS.
  WAIT ROT_SAMPLE.
  SET rotRate TO VANG(axis0, portAxisOf(tgtPort)) /
                 MAX(0.1, TIME:SECONDS - t0Rot).
  IF rotRate > ROT_REFUSE {
    PRINT "!! " + tgtVes:NAME + "'s port is swinging at " +
          ROUND(rotRate, 2) + " deg/s.".
    PRINT "   That is a spinning target, not a docking target. Stabilise it".
    PRINT "   (SAS on that craft) and re-run.".
    SET okToGo TO FALSE.
  } ELSE IF rotRate > ROT_WARN {
    PRINT "!! Target port is turning at " + ROUND(rotRate, 2) + " deg/s.".
    PRINT "   The approach will track it, but expect a longer final and more".
    PRINT "   mono spent holding the cone. Stabilising the target is cheaper.".
  }
}

// --- 0h. Can this ship stop before the station from where it is? -----------
//  The absolute speed cap in 0b is a sanity check; this is the real one, and it
//  is the same arithmetic the in-flight guard uses: closing speed squared
//  against twice the translational authority times the room left before the
//  keep-out sphere.  It could not be asked earlier because the sphere is
//  measured, not assumed, and 0f is where it gets measured.  A ship that fails
//  this test cannot be saved by any control law - it arrives inside the station
//  whatever it does with its thrusters - so the answer is to stop first.
IF okToGo {
  refreshGeom().
  LOCAL room IS MAX(0, sepVec:MAG - KEEP_R).
  LOCAL toHat IS V(0, 0, 0).
  IF sepVec:MAG > 0.001 { SET toHat TO (-sepVec):NORMALIZED. }
  LOCAL closing IS VDOT(vRel, toHat).
  IF closing > 0.3 AND closing * closing > 2 * dckApproachAccel() * room {
    PRINT "!! Closing on " + tgtVes:NAME + " at " + ROUND(closing, 1) +
          " m/s with " + ROUND(room) + " m before the keep-out sphere.".
    PRINT "   This ship's RCS can stop " +
          ROUND(SQRT(2 * dckApproachAccel() * room), 1) +
          " m/s from there and no more - starting an approach now ends".
    PRINT "   inside " + tgtVes:NAME + " whatever the thrusters do.".
    PRINT "   Null the relative velocity first and re-run.".
    SET okToGo TO FALSE.
  }
}

// --- 0i. Price the job in monopropellant ------------------------------------
//  Three legs and a hold.  Each leg costs two speed changes - one to get moving
//  and one to stop - at whatever speed the braking schedule would actually
//  allow over its length.
IF okToGo {
  refreshGeom().

  SET legOut TO 0.
  SET legFwd TO 0.
  IF axGap >= STANDOFF * 0.95 {
    SET legFwd TO (tgtNodePos + tgtAxis * STANDOFF - ourNodePos):MAG.
  } ELSE {
    SET legOut TO MAX(0, KEEP_R - latGap).
    SET legFwd TO ABS(STANDOFF - axGap) + MAX(latGap, KEEP_R).
  }
  SET monoBill TO legCost(legOut, V_CORRIDOR) + legCost(legFwd, V_CORRIDOR) +
                  legCost(STANDOFF, V_FINAL) + vRel:MAG + MONO_TRIM.
  SET monoHave TO dckMonoDv().

  PRINT "------------------------------------------------------".
  PRINT "TARGET :: " + tgtPort:TITLE + " on " + tgtVes:NAME.
  PRINT "  Range     : " + ROUND(sepVec:MAG, 1) + " m port to port".
  PRINT "  Axial     : " + ROUND(axGap, 1) + " m in front of it" +
        " (negative = behind its plane)".
  PRINT "  Lateral   : " + ROUND(latGap, 1) + " m off its axis".
  PRINT "  Port axes : " + ROUND(alignErr, 1) + " deg apart".
  PRINT "  Rel speed : " + ROUND(vRel:MAG, 2) + " m/s".
  PRINT "  Spans     : ours " + ROUND(ourSpan, 1) + " m from its port, theirs " +
        ROUND(tgtSpan, 1) + " m from theirs".
  PRINT "  Keep-out  : " + ROUND(KEEP_R) + " m,  standoff " + ROUND(STANDOFF) +
        " m,  capture at " + ROUND(GRAB_GAP, 2) + " m".
  dckResourceReport("before docking").

  PRINT "======================================================".
  PRINT "MONO BUDGET".
  IF legOut > 0.5 {
    PRINT "  Clear out     : " + ROUND(legOut) + " m sideways to get out of" +
          " the station's way".
  }
  PRINT "  To standoff   : " + ROUND(legFwd) + " m to the corridor entry".
  PRINT "  Final         : " + ROUND(STANDOFF) + " m down the axis at " +
        ROUND(V_FINAL, 2) + " m/s or less".
  PRINT "  NEED  " + ROUND(monoBill, 1) + " m/s     HAVE  " +
        ROUND(monoHave, 1) + " m/s".

  SET monoOk TO monoHave >= monoBill.
  IF monoOk AND (monoHave - monoBill) >= monoBill * GO_MARGIN {
    PRINT "  => GO. " + ROUND(monoHave - monoBill, 1) + " m/s in hand.".
  } ELSE IF monoOk {
    PRINT "  => MARGINAL. Only " + ROUND(monoHave - monoBill, 1) +
          " m/s in hand (" +
          ROUND(100 * (monoHave - monoBill) / MAX(0.1, monoBill)) +
          "% of the bill).".
    PRINT "     A missed capture and a second approach spend that.".
    IF PREFLIGHT_HOLD > 0 { WAIT PREFLIGHT_HOLD. }
  } ELSE {
    PRINT "  => NOT ENOUGH mono - short by " +
          ROUND(monoBill - monoHave, 1) + " m/s.".
    PRINT "     Flying it anyway: the approach stops at MONO_RESERVE and".
    PRINT "     holds where it is rather than drifting. But it may not get".
    PRINT "     all the way in - top up from the station first if you can.".
    IF PREFLIGHT_HOLD > 0 { WAIT PREFLIGHT_HOLD. }
  }
  PRINT "======================================================".
}

// --- 0j. Control from the port ----------------------------------------------
//  Everything downstream assumes the ship's forward axis IS our port's axis:
//  the steering lock points the port, and the translation commands are resolved
//  in that frame.  Controlling from the cockpit instead would line the *nose*
//  up with the target port and fly the ship in sideways.
IF okToGo {
  IF VANG(SHIP:FACING:FOREVECTOR, portAxisOf(ourPort)) > CTRL_PORT_TOL {
    IF ourPort:HASSUFFIX("CONTROLFROM") {
      PRINT "Taking control from " + ourPort:TITLE + ".".
      ourPort:CONTROLFROM().
      WAIT 0.5.
    }
  }
  IF VANG(SHIP:FACING:FOREVECTOR, portAxisOf(ourPort)) > CTRL_PORT_TOL {
    PRINT "!! The ship is not being controlled from its docking port - the".
    PRINT "   forward axis is " +
          ROUND(VANG(SHIP:FACING:FOREVECTOR, portAxisOf(ourPort)), 1) +
          " deg off the port's.".
    PRINT "   Right-click " + ourPort:TITLE + " -> Control From Here, then".
    PRINT "   RUN dock. again.".
    SET okToGo TO FALSE.
  }
}

// ---------------------------------------------------------------------------
//  1..5  --  fly it
// ---------------------------------------------------------------------------
SET phaseNow   TO "corridor".
SET tPhaseEnd  TO 0.
SET emergency  TO FALSE.
SET captureTry TO 0.

FUNCTION setPhase {
  PARAMETER nameIn, budget.
  SET phaseNow  TO nameIn.
  SET tPhaseEnd TO TIME:SECONDS + budget.
}

IF okToGo {
  // Names for the report, taken now: once the two ships dock they are one
  // vessel and the target's identity stops being a separate thing to ask about.
  SET TGT_VES_NAME  TO tgtVes:NAME.
  SET TGT_PORT_NAME TO tgtPort:TITLE.

  PRINT "1. SET UP :: RCS on, controlling from " + ourPort:TITLE + ".".
  RCS ON.
  SAS OFF.
  IF STEERINGMANAGER:HASSUFFIX("MAXSTOPPINGTIME") {
    SET STEER_SAVED   TO STEERINGMANAGER:MAXSTOPPINGTIME.
    SET STEERINGMANAGER:MAXSTOPPINGTIME TO STEER_STOP_T.
    SET STEER_TOUCHED TO TRUE.
  }
  // The nose - which is to say our port - goes onto the reversed target axis
  // and stays there for the rest of the flight, in every phase.  Recomputed by
  // the lock every tick, so a target that turns slowly is followed round.
  LOCK STEERING TO dockSteer().

  refreshGeom().
  IF inCorridor() AND axGap > GRAB_GAP * 3 AND axGap <= STANDOFF + STANDOFF_TOL {
    // Already lined up in front of the port - which is where rendezvous.ks
    // sometimes leaves us.  Backing out to the standoff just to fly the same
    // metres again spends mono to arrive where we started.
    PRINT "2. CORRIDOR :: already inside it, " + ROUND(axGap, 1) + " m out and " +
          ROUND(latGap, 2) + " m off axis - straight to the final approach.".
    setPhase("final", T_FINAL).
  } ELSE {
    PRINT "2. CORRIDOR :: to the standoff point, " + ROUND(STANDOFF) +
          " m out in front of " + TGT_PORT_NAME + ".".
    setPhase("corridor", T_CORRIDOR).
  }

  SET tEnd     TO TIME:SECONDS + T_TOTAL.
  SET tNextSay TO 0.
  SET vLast    TO relVel().
  SET tLast    TO TIME:SECONDS.
  SET lastFull TO FALSE.
  SET running  TO TRUE.
  SET whyStop  TO "docked".

  UNTIL NOT running {
    refreshGeom().

    IF dockedNow() {
      SET running TO FALSE.
      SET whyStop TO "docked".
    } ELSE {
      // --- what the thrusters are really delivering -------------------------
      //  Believed over the nameplate figure, and sampled only across intervals
      //  spent at full command: acceleration measured through a half-throttled
      //  correction is half the authority the ship has, and feeding that back
      //  would ratchet the approach slower and slower for no reason.
      LOCAL dtM IS TIME:SECONDS - tLast.
      IF dtM > 0.5 {
        IF lastFull {
          LOCAL accM IS (vRel - vLast):MAG / dtM.
          IF accM > RCS_ACC_MIN AND accM < 5 {
            SET rcsAccEst TO rcsAccEst * 0.8 + accM * 0.2.
          }
        }
        SET vLast    TO vRel.
        SET tLast    TO TIME:SECONDS.
        SET lastFull TO FALSE.
      }

      LOCAL latHat IS V(0, 0, 0).
      IF latGap > 0.001 { SET latHat TO latVec:NORMALIZED. }
      LOCAL vWant IS V(0, 0, 0).

      IF phaseNow = "corridor" OR phaseNow = "align" {
        // Both phases fly to a point: the corridor routes to it around the
        // station, the align phase simply holds it while the attitude settles.
        LOCAL aimPt IS corridorAim().
        IF phaseNow = "align" {
          SET aimPt TO tgtNodePos + tgtAxis * STANDOFF.
          SET aimIsEntry TO TRUE.
        }
        LOCAL errVec IS aimPt - ourNodePos.
        LOCAL errMag IS errVec:MAG.
        IF errMag > 0.05 {
          LOCAL vCap IS MIN(V_CORRIDOR,
                            SQRT(2 * dckApproachAccel() * errMag / BRAKE_SAFETY)).
          SET vWant TO errVec:NORMALIZED * MIN(vCap, KP_POS * errMag).
        }

        IF phaseNow = "corridor" AND aimIsEntry AND errMag < STANDOFF_TOL
           AND vRel:MAG < HOLD_V_TOL {
          PRINT "3. ALIGN :: holding " + ROUND(axGap) + " m out; port axes " +
                ROUND(alignErr, 1) + " deg apart.".
          setPhase("align", T_ALIGN).
        } ELSE IF phaseNow = "align" AND alignErr < ALIGN_TOL
                  AND errMag < STANDOFF_TOL AND vRel:MAG < HOLD_V_TOL {
          PRINT "4. FINAL :: closing from " + ROUND(axGap) + " m at " +
                ROUND(V_FINAL, 2) + " m/s or less, inside a cone " +
                ROUND(latTolAt(axGap), 2) + " m wide here and " +
                ROUND(LAT_TOL_MIN, 2) + " m wide at the port.".
          setPhase("final", T_FINAL).
        }

      } ELSE IF phaseNow = "final" {
        // Lateral first, angle second, axial last.  The closing rate is zero
        // unless the ship is inside the cone AND lined up - which is what makes
        // this an approach rather than a slow collision.
        LOCAL latTol IS latTolAt(axGap).
        LOCAL vAx IS 0.
        IF alignErr < ALIGN_HOLD AND latGap < latTol AND axGap > GRAB_GAP {
          SET vAx TO MIN(V_FINAL,
                         SQRT(2 * dckApproachAccel() * MAX(0, axGap - GRAB_GAP) /
                              BRAKE_SAFETY)).
          IF axGap < V_CREEP_ZONE { SET vAx TO MIN(vAx, V_CAPTURE). }
          SET vAx TO MAX(vAx, V_CREEP).
        }
        LOCAL vLat IS 0.
        IF latGap > LAT_TOL_MIN * 0.5 { SET vLat TO MIN(V_LAT, KP_LAT * latGap). }
        SET vWant TO (-tgtAxis) * vAx - latHat * vLat.

        IF axGap <= GRAB_GAP AND latGap < CAPTURE_LAT {
          PRINT "5. CAPTURE :: " + ROUND(axGap, 2) + " m, " +
                ROUND(latGap, 2) + " m off axis - thrusters off, magnets on.".
          setPhase("capture", CAPTURE_WAIT).
        } ELSE IF NOT inCorridor() {
          PRINT "  !! Pushed out of the corridor (" + ROUND(latGap, 1) +
                " m off axis at " + ROUND(axGap, 1) + " m). Re-entering.".
          setPhase("corridor", T_CORRIDOR).
        }

      } ELSE {
        // Capture.  Barely any command at all: a creep to keep the nodes
        // together and a light hand on the lateral, and otherwise let the
        // magnets do what they are for.
        LOCAL vAx IS 0.
        IF axGap > GRAB_GAP * 0.4 { SET vAx TO V_CREEP. }
        LOCAL vLat IS 0.
        IF latGap > CAPTURE_LAT * 0.5 {
          SET vLat TO MIN(V_LAT * 0.4, KP_LAT * latGap).
        }
        SET vWant TO (-tgtAxis) * vAx - latHat * vLat.

        IF axGap > GRAB_GAP * 3 {
          PRINT "  Drifted back out to " + ROUND(axGap, 2) + " m - closing again.".
          setPhase("final", T_FINAL).
        } ELSE IF TIME:SECONDS > tPhaseEnd {
          SET captureTry TO captureTry + 1.
          IF captureTry >= CAPTURE_TRIES {
            SET running TO FALSE.
            SET whyStop TO "nocapture".
          } ELSE {
            PRINT "  !! No capture after " + ROUND(CAPTURE_WAIT) + " s (try " +
                  captureTry + " of " + CAPTURE_TRIES + "). Backing off to the".
            PRINT "     standoff and re-flying the approach.".
            setPhase("align", T_ALIGN).
          }
        }
      }

      // --- the guard that stops a hot arrival ---------------------------------
      //  Outside the corridor the station is a sphere to be missed, not a port
      //  to be aimed at.  If the closing speed is more than the thrusters can
      //  shed in the room left before that sphere, nothing else matters until
      //  it is not: null the relative velocity and let the routing start again.
      IF inCorridor() {
        SET emergency TO FALSE.
      } ELSE {
        LOCAL room IS 2 * dckApproachAccel() * MAX(0, sepVec:MAG - KEEP_R).
        LOCAL toHat IS V(0, 0, 0).
        IF sepVec:MAG > 0.001 { SET toHat TO (-sepVec):NORMALIZED. }
        LOCAL vTow IS VDOT(vRel, toHat).
        IF vTow > 0.3 AND vTow * vTow > room {
          IF NOT emergency {
            PRINT "  !! Closing at " + ROUND(vTow, 1) + " m/s with " +
                  ROUND(sepVec:MAG - KEEP_R) + " m of room - all stop.".
          }
          SET emergency TO TRUE.
        } ELSE IF emergency AND vTow * vTow < room * 0.5 {
          PRINT "  Back inside the braking envelope.".
          SET emergency TO FALSE.
        }
        IF emergency { SET vWant TO V(0, 0, 0). }
      }

      IF translateTo(vWant, vRel) { SET lastFull TO TRUE. }

      // --- say where we are ---------------------------------------------------
      IF TIME:SECONDS > tNextSay {
        SET tNextSay TO TIME:SECONDS + 2.
        PRINT "  [" + phaseNow + "] ax " + ROUND(axGap, 1) + " m  lat " +
              ROUND(latGap, 2) + " m  ang " + ROUND(alignErr, 1) + " deg  v " +
              ROUND(vRel:MAG, 2) + " m/s  mono " +
              ROUND(dckResAmtShip("MonoPropellant")) + " u".
      }

      // --- reasons to stop ----------------------------------------------------
      IF dckResAmtShip("MonoPropellant") < MONO_RESERVE {
        SET running TO FALSE.
        SET whyStop TO "mono".
      } ELSE IF TIME:SECONDS > tEnd {
        SET running TO FALSE.
        SET whyStop TO "timeout".
      } ELSE IF TIME:SECONDS > tPhaseEnd AND phaseNow <> "capture" {
        SET running TO FALSE.
        SET whyStop TO "stuck".
      }
    }
    WAIT 0.05.
  }

  SET SHIP:CONTROL:NEUTRALIZE TO TRUE.

  // -------------------------------------------------------------------------
  //  6. REPORT
  // -------------------------------------------------------------------------
  PRINT "======================================================".
  IF whyStop = "docked" {
    PRINT "DOCKED to " + TGT_VES_NAME + " at " + TGT_PORT_NAME + ".".
    PRINT "  Approach flown on " + ROUND(dckApproachAccel(), 3) +
          " m/s^2 of measured RCS authority.".
    dckResourceReport("docked").
    PRINT "  Crew transfer, fuel transfer and cargo are yours from here.".
    PRINT "  Before you leave: undock, back off past " + ROUND(KEEP_R) +
          " m, then RUN deorbit_land.".
    IF portHasShield(ourPort) {
      PRINT "  Close the port shield (action group 3) before reentry.".
    }
  } ELSE {
    PRINT "NOT DOCKED - handing back where we are.".
    IF whyStop = "mono" {
      PRINT "  Monopropellant reached the " + MONO_RESERVE + " u reserve at " +
            ROUND(axGap, 1) + " m. Stopped rather than drifted.".
    } ELSE IF whyStop = "timeout" {
      PRINT "  Ran out of the " + ROUND(T_TOTAL / 60) + " min total budget.".
    } ELSE IF whyStop = "stuck" {
      PRINT "  The " + phaseNow + " phase ran out of time. Most often this is".
      PRINT "  a target that will not hold still, or a cone too tight for the".
      PRINT "  authority available - raise LAT_CONE or stabilise the target.".
    } ELSE IF whyStop = "nocapture" {
      PRINT "  " + CAPTURE_TRIES + " approaches reached " + ROUND(GRAB_GAP, 2) +
            " m without the magnets biting.".
      PRINT "  Check both ports are open and free, and that they are the same".
      PRINT "  size. Finish it by hand from here - the ship is lined up.".
    }
    PRINT "  Axial " + ROUND(axGap, 1) + " m, lateral " + ROUND(latGap, 2) +
          " m, axes " + ROUND(alignErr, 1) + " deg apart.".
    dckResourceReport("docking incomplete").
  }
  PRINT "======================================================".

  dckHandBack().
  IF whyStop = "docked" { RCS OFF. } ELSE { RCS ON. }
}

// Every path that stopped early - no target, wrong kind of target, an occupied
// port, no mono, an ambiguous port of our own - lands here.  Leaving the IPU
// raised and the throttle locked after the program ends is how a pilot finds
// the stick dead.
IF NOT okToGo {
  dckHandBack().
  PRINT "Not docking - the ship has not been moved.".
}
