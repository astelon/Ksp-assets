// ============================================================================
//  intercept.ks  --  Runway to hard dock, on the target's clock
// ----------------------------------------------------------------------------
//  Target : another vessel in orbit around the body we are sitting on, picked
//           on the map (or named).  The ship waits on the runway until the
//           moment that makes the rest of the mission cheap, then flies the
//           whole thing: ascent.ks to orbit, rendezvous.ks to the doorstep,
//           dock.ks to a hard dock.
//
//  Run with:   RUN intercept.                    - whatever is selected on the map
//              RUN intercept("Station Alpha").   - or by name (substring is enough)
//              RUN intercept("Alpha", 250).      - ...and a park distance, in metres
//              RUN intercept("Alpha", 200, "dorsal").  - ...and which port is ours
//
//  Select the target on the map before running it.  Selecting a *docking port*
//  rather than the vessel works too, and is better: the script keeps that port
//  all the way through and hands it to dock.ks at the end.
//
//  Design notes (why it waits, and for what)
//  -----------------------------------------
//  * THE WAIT IS THE WHOLE SCRIPT.  Everything after the countdown is three
//    scripts that already exist, run in order.  What this one adds is the
//    launch time - and a launch time is worth more than any amount of clever
//    flying afterwards, because the ascent puts the ship at a fixed place on
//    the clock and the target is somewhere on its own orbit whether we like it
//    or not.  Launch at the wrong minute and the ship arrives in a perfectly
//    good orbit half a lap behind a station it will then spend hours, or a
//    phasing orbit's worth of propellant, catching.  Launching at the right
//    minute costs nothing but the waiting, and the waiting can be warped.
//  * WHAT IS AIMED AT IS THE PHASE AT INSERTION, NOT THE PHASE AT LAUNCH.  The
//    quantity that matters is where the target will be when we *arrive* in
//    orbit, roughly twenty minutes after the brakes come off.  So the script
//    models the ascent as "the ship arrives ASC_TIME seconds later, ASC_ARC
//    degrees further round the body", propagates the target with kOS's own
//    POSITIONAT, and searches for the launch time that puts the two in the
//    relationship a Hohmann transfer wants.  Both of those ascent numbers are
//    measured on every flight and written to disk, so the model is calibrated
//    by flying rather than by argument.
//  * THE AIM POINT IS DELIBERATELY EARLY.  We do not aim to insert at the
//    transfer lead angle; we aim to insert a little short of it and let the
//    parking orbit drift the rest of the way.  This is not caution for its own
//    sake - the error is asymmetric.  Arriving early costs the drift time, and
//    drift time is free.  Arriving *late* means the lead angle has already gone
//    past, and the next one is a full synodic lap away - twelve hours, for a
//    parking orbit 20 km under a station - so it gets paid for with a phasing
//    orbit instead.  The buffer is sized from the ascent's own repeatability:
//    an ascent that runs a minute late puts the target omega_target * 60
//    degrees further round, and the buffer is however long the drift takes to
//    undo that.  See itcDriftBuffer().
//  * THE PARKING ORBIT IS CHOSEN, NOT INHERITED.  ascent.ks will fly to any
//    apoapsis it is given, so this script gives it one: under the target, where
//    there is room for it, far enough under that the drift rate is useful.  A
//    parking orbit at the *target's* altitude drifts at zero - the phase you
//    arrive with is the phase you keep - and that turns every second of ascent
//    scatter into a phasing orbit somebody has to pay for.  When the target is
//    too low to park under (it is inside our floor over the atmosphere) the
//    ship parks above it instead and lets the target do the catching up.
//  * IT LAUNCHES DUE EAST, AND SAYS WHAT THAT COSTS.  ascent.ks flies the
//    runway heading, which is what a spaceplane on a runway can do: rolling
//    down the centreline and then turning to a launch azimuth is a different
//    ascent script, not a parameter.  So the ship reaches an equatorial orbit
//    and any inclination the target has is paid for in orbit, by rendezvous.ks,
//    at 2*v*sin(di/2).  The important part is that this cost does NOT depend on
//    the launch time: an equatorial orbit crosses the target's plane at the
//    target's own nodes whenever we launch.  There is therefore no plane
//    window to wait for and no trade against the phase window - but there is a
//    bill, it can be hundreds of m/s, and the script prints it before the
//    countdown rather than after the climb.
//  * EVERY CHECK THAT CAN BE MADE ON THE GROUND IS MADE ON THE GROUND.  Waiting
//    forty minutes and *then* discovering that the ship has no free docking
//    port, or that dock.ks is not on the volume, or that the target is orbiting
//    another planet, is the specific failure this script exists to prevent.  So
//    the target, the scripts, the ports, the RCS and the monopropellant are all
//    checked before the clock starts, and the things that can only be checked
//    later - the climb's own funding, the station's free ports - are named as
//    such.
//  * IT HANDS OVER, IT DOES NOT DUPLICATE.  The climb is ascent.ks's, the
//    transfer is rendezvous.ks's, the approach is dock.ks's, and each of them
//    prices, refuses and reports for itself.  This script checks between stages
//    that the stage actually happened - an ascent that settled for a lower
//    orbit, or a rendezvous that stopped on its reserve, must not be followed
//    by the next stage as if nothing had happened.
//  * ITS NAMES ARE ITS OWN.  Everything global here is prefixed ITC_ or itc,
//    and that is not decoration.  kOS RUN shares one global scope, so the three
//    scripts this one calls will overwrite any name they also use - and they do
//    not agree with each other: clampVal takes (value, lo, hi) in ascent.ks and
//    (lo, hi, value) in deorbit_land.ks.  A helper called clampVal here would
//    silently become a different function halfway through the mission.
//
//  Flight plan
//    0. Pre-flight : acquire the target, check the ship, the ports and the
//                    scripts, measure the launch site, price the plane change.
//    1. Window     : choose the parking orbit, compute the insertion phase the
//                    transfer wants, and search for the launch time that
//                    produces it.  Report it, with the alternatives.
//    2. Countdown  : warp the wait out, drop to real time, hold to T-0.
//    3. Ascent     : RUN ascent to the chosen parking apoapsis.
//    4. Calibrate  : measure what the ascent actually did, compare it with what
//                    was predicted, and write it back to disk for next time.
//    5. Rendezvous : RUN rendezvous to the park distance.
//    6. Port       : pick a free, compatible port on the target and target it.
//    7. Dock       : RUN dock.
//    8. Report     : the whole mission, end to end, and what to trim.
//
//  Tunables are grouped at the top so you can trim them to your build.
// ============================================================================

// The vessel to meet.  Leave it out and the script uses the map selection.
//
// (Optional program parameters need kOS 1.0.1 or newer.  On anything older,
// drop the "IS" defaults and always pass all three values.)
DECLARE PARAMETER targetName IS "".
DECLARE PARAMETER parkMetres IS 200.
DECLARE PARAMETER ourPortHint IS "".

CLEARSCREEN.

// --- The ascent model -------------------------------------------------------
//  Two numbers describe the climb, and the launch time is only as good as they
//  are.  ITC_ASC_TIME is how long ascent.ks takes from the moment it is handed
//  control to the moment the circularisation burn ends; ITC_ASC_ARC is how far
//  round the body the ship has travelled by then, measured in inertial space
//  from where the launch site was at T-0 (so it already contains the body's own
//  rotation during the climb - it is not a ground track).
//
//  The values below are seeds for a Mk3 RAPIER SSTO climbing to 80-100 km on
//  Kerbin.  Every flight measures both and writes them to ITC_CAL_PATH, and
//  every later flight reads them back, so the first flight is the only one that
//  uses a guess.  They are printed at the end either way.
SET ITC_ASC_TIME  TO 1250.        // runway to circular orbit (s)
SET ITC_ASC_ARC   TO 165.         // inertial arc swept while doing it (deg)
SET ITC_ASC_SIGMA TO 120.         // how repeatable that time is, flight to
                                  // flight (s).  This is the number the safety
                                  // buffer is built from - see itcDriftBuffer.
                                  // Drop it once a few flights have shown you
                                  // the real scatter; the report prints it

// --- Parking orbit ----------------------------------------------------------
SET ITC_PARK_BELOW  TO 25000.     // aim to park this far under the target (m).
                                  // Bigger = faster drift = more tolerant of a
                                  // scattered ascent, at a few more m/s on the
                                  // transfer.  Smaller = a cheaper transfer and
                                  // a longer wait in orbit
SET ITC_PARK_ABOVE  TO 25000.     // ... and this far over it, when under is not
                                  // available (m)
SET ITC_PARK_FLOOR  TO 10000.     // never park closer than this to the top of
                                  // the atmosphere (m)
SET ITC_PARK_GAP    TO 8000.      // a parking orbit within this of the target's
                                  // radius drifts too slowly to be worth having;
                                  // go to the other side of it instead (m)
SET ITC_PARK_CEIL   TO 500000.    // refuse to plan a parking orbit above this
                                  // (m) - past here the ascent is the problem,
                                  // not the window

// --- The launch window ------------------------------------------------------
SET ITC_BUF_K     TO 1.5.         // safety buffer, in multiples of the drift
                                  // time that undoes one sigma of ascent
                                  // scatter.  Simulated over 600 geometries:
                                  // 0.5 overshoots the transfer point on 22% of
                                  // flights (median 34 min of drift), 1.0 on
                                  // 7.5% (48 min), 1.5 on 0.8% (66 min), and 2
                                  // buys 0.5% more for another 19 minutes.
                                  // 1.5 is the knee
SET ITC_BUF_MIN   TO 420.         // ...but never plan less drift than this (s)
SET ITC_BUF_MAX   TO 5400.        // ...nor more than this (s).  Against a
                                  // co-orbital target the formula wants
                                  // infinity; this is where it gives up and
                                  // lets rendezvous.ks solve the phasing.
                                  // Keep it under rendezvous.ks's
                                  // MAX_TOTAL_TIME or the wait this script
                                  // plans for is a wait that script will not
                                  // sit through
SET ITC_MIN_LEAD  TO 90.          // no window closer than this - the pre-flight
                                  // checks and ascent.ks's own take time (s)
SET ITC_MAX_WAIT  TO 21600.       // how far ahead to look for one (s)
SET ITC_SCAN_STEP TO 20.          // fallback scan resolution (s).  Only used if
                                  // the closed-form solution fails, and capped
                                  // at a 24th of a sweep either way so it can
                                  // never step over a crossing
SET ITC_BISECT    TO 30.          // bisection rounds once a crossing is bracketed
SET ITC_WIN_TOL   TO 0.05.        // insertion phase error accepted from the
                                  // closed form before it is refined (deg)
SET ITC_SWEEP_TOL TO 0.10.        // how far the measured sweep rate may differ
                                  // from the one the orbital periods say it
                                  // must be, as a fraction, before the target
                                  // prediction is declared untrustworthy
SET ITC_TGT_MODEL TO "auto".      // how the target's future position is found:
                                  // "posat"  - kOS POSITIONAT, exact for any
                                  //            orbit but built on kOS's own
                                  //            frame handling, which a flight
                                  //            has shown cannot be assumed from
                                  //            a landed ship
                                  // "rotate" - rotate its present position
                                  //            about its orbit normal, exact
                                  //            for a circular orbit and free of
                                  //            any frame assumption at all
                                  // "auto"   - rotate for a near-circular
                                  //            target, posat for an eccentric
                                  //            one that the cross-check agrees
                                  //            with.  Either way the two are
                                  //            compared and the answer printed
SET ITC_ECC_CIRC  TO 0.01.        // eccentricity under which "circular" is a
                                  // fair description of the target's orbit

// --- Handover checks --------------------------------------------------------
SET ITC_PLANE_SKIP_DI TO 0.05.    // relative inclination under this is nothing
                                  // (deg) - matches rendezvous.ks
SET ITC_PLANE_WARN_DV TO 150.     // say so loudly over this much plane change (m/s)
SET ITC_ABORT_IF_PLANE TO FALSE.  // TRUE = refuse to launch at all when the
                                  // plane change is over the warning.  FALSE =
                                  // print the bill, hold, and let the pilot see
                                  // it before the countdown starts
SET ITC_RDV_RANGE_OK TO 2000.     // dock.ks is only started if rendezvous.ks
                                  // left us inside this range (m)
SET ITC_RDV_VREL_OK  TO 2.        // ...and under this relative speed (m/s)
SET ITC_MONO_DOCK    TO 40.       // rough monopropellant the docking approach
                                  // wants (units).  dock.ks prices it properly;
                                  // this only catches a ship launched dry
SET ITC_HOLD         TO 12.       // pause this long on a failed check (s)
SET ITC_PARK_FIXED   TO 0.        // metres.  Only for an ascent script that
                                  // cannot be told an apoapsis: set it to the
                                  // orbit yours actually flies to, and the
                                  // window is planned around that instead of
                                  // around an orbit we can choose

// --- Chaining ---------------------------------------------------------------
SET ITC_HOME     TO "".           // volume/directory the three scripts live in,
                                  // e.g. "0:/".  Empty = the current volume
SET ITC_ASCENT   TO "ascent".
SET ITC_RENDEZ   TO "rendezvous".
SET ITC_DOCK     TO "dock".
SET ITC_RUN_ASC  TO TRUE.         // set all three FALSE to rehearse the window
SET ITC_RUN_RDV  TO TRUE.         // solver and the pre-flight checks without
SET ITC_RUN_DOCK TO TRUE.         // flying, or waiting, for any of it
SET ITC_CHECK_ARGS TO TRUE.       // read the three scripts and count the
                                  // parameters they declare, so they are called
                                  // the way they are actually written rather
                                  // than the way this script expects.  Set
                                  // FALSE (and fix the counts below) if your
                                  // kOS cannot read a file as text
SET ITC_ASC_PARAMS  TO -1.        // -1 = detect. Otherwise the number of
SET ITC_RDV_PARAMS  TO -1.        // parameters each script takes; the versions
SET ITC_DOCK_PARAMS TO -1.        // in this repository take 1, 2 and 2

// --- Calibration ------------------------------------------------------------
SET ITC_USE_CAL  TO TRUE.
SET ITC_CAL_PATH TO "0:/intercept_cal.json".

// --- Warp / logging ---------------------------------------------------------
SET ITC_USE_WARP  TO TRUE.
SET ITC_WARP_LEAD TO 25.          // drop out of warp this long before T-0 (s)
SET ITC_TICK      TO 5.           // countdown print interval (s)

// ---------------------------------------------------------------------------
//  Body / physical constants (cached once)
// ---------------------------------------------------------------------------
SET ITC_MU     TO SHIP:BODY:MU.
SET ITC_BODY_R TO SHIP:BODY:RADIUS.
SET ITC_ATM    TO SHIP:BODY:ATM:HEIGHT.
SET ITC_TROT   TO SHIP:BODY:ROTATIONPERIOD.
SET ITC_WSPIN  TO 360 / MAX(1, ITC_TROT).      // deg/s of body rotation

// The searches below are the only instruction-hungry thing here, and they are
// modest by this repository's standards.  Restored on exit - and restored from
// OUR global, because ascent.ks saves and restores the IPU too, and by the time
// it hands back its idea of "the value before" is the value we raised it to.
SET ITC_IPU_SAVED TO CONFIG:IPU.
SET CONFIG:IPU TO 800.

// ---------------------------------------------------------------------------
//  Small helpers
//
//  NOTE ON NAMING: every name in this file is prefixed, because kOS RUN shares
//  one global scope and the scripts this one calls define helpers with the same
//  obvious names - and with different argument orders.  See the header.
// ---------------------------------------------------------------------------
FUNCTION itcClamp {
  PARAMETER valIn, loV, hiV.
  RETURN MAX(loV, MIN(hiV, valIn)).
}

FUNCTION itcWrap {                  // wrap an angle into (-180, 180]
  PARAMETER angIn.
  LOCAL ang IS angIn.
  UNTIL ang <= 180 { SET ang TO ang - 360. }
  UNTIL ang > -180 { SET ang TO ang + 360. }
  RETURN ang.
}

FUNCTION itcHMS {                   // seconds as h:mm:ss, for anything long
  PARAMETER secIn.
  LOCAL s IS MAX(0, ROUND(secIn)).
  LOCAL h IS FLOOR(s / 3600).
  LOCAL m IS FLOOR((s - h * 3600) / 60).
  LOCAL sec IS s - h * 3600 - m * 60.
  LOCAL mm IS "" + m.
  LOCAL rr IS "" + sec.
  IF m < 10 { SET mm TO "0" + mm. }
  IF sec < 10 { SET rr TO "0" + rr. }
  RETURN h + ":" + mm + ":" + rr.
}

// ---------------------------------------------------------------------------
//  Where the launch site will be
//
//  A landed vessel's "orbit" is a fiction that follows the surface, so kOS's
//  POSITIONAT cannot be asked where the runway will be in twenty minutes.  It
//  does not need to be: a point on the surface moves on a circle about the
//  body's spin axis, and both the axis and the phase can be measured right now,
//  off the ship that is sitting on it.
//
//  The measurement is the whole trick.  While landed and stationary the ship's
//  *orbital* velocity is exactly the velocity the body's rotation gives it, so
//
//      k = r x v                 is the spin axis, signed the right way round
//      omega = 360 / rotperiod   is how fast it turns
//
//  and the rotation of any vector about k in the direction the surface actually
//  travels is then a two-line construction.  Note that this needs no knowledge
//  of kOS's coordinate frame or its handedness: k is built with a cross product
//  and every later use of it sits inside a second cross product or a triple
//  product, so a global sign convention on the operator cancels out.  The same
//  reason phaseAngleAt() in rendezvous.ks is frame-free.
//
//  The measurement is taken again whenever it is used, and that is not
//  belt-and-braces.  KSP holds the world still around a landed vessel by
//  turning the *frame* with the body, so a direction vector read on the runway
//  and used forty minutes later has had the planet rotate underneath it while
//  its numbers stayed the same.  Predictions made from a stale one are out by
//  the whole rotation since it was taken - up to forty degrees over a long
//  wait, which is exactly the size of the thing being computed.  Everything
//  here therefore predicts forward from *now*, and "now" is re-read before each
//  batch of predictions.  (The spin axis itself is immune - a rotation about an
//  axis leaves that axis alone - but it costs nothing to take it again.)
//
//  The same three lines mean something slightly different once the ship is off
//  the ground, and usefully so: r x v is the spin axis for a vessel that is
//  standing on the surface and the *orbit normal* for one that is flying.  So
//  after the ascent this measures the plane we actually ended up in, which is
//  the plane the phase to the target should be measured in - the convention
//  rendezvous.ks uses.  What it does NOT do off the ground is make itcSiteAt()
//  mean anything; nothing calls it after T-0.
// ---------------------------------------------------------------------------
FUNCTION itcRefresh {
  SET ITC_TREF TO TIME:SECONDS.
  SET ITC_R0   TO SHIP:POSITION - SHIP:BODY:POSITION.
  SET ITC_V0   TO SHIP:VELOCITY:ORBIT.
  SET ITC_KHAT TO VCRS(ITC_R0, ITC_V0):NORMALIZED.
}

// The same reading, taken of the target: where it is, which way round its orbit
// goes, and how fast.  Two vectors and a rate are enough to say where it will
// be, and unlike a stored prediction they are re-read whenever we ask.
FUNCTION itcTgtCapture {
  SET ITC_TGT_R0 TO ITC_TGT:POSITION - SHIP:BODY:POSITION.
  SET ITC_TGT_V0 TO ITC_TGT:VELOCITY:ORBIT.
  SET ITC_H_TGT  TO VCRS(ITC_TGT_R0, ITC_TGT_V0).
  SET ITC_H_HAT  TO ITC_H_TGT:NORMALIZED.
  SET ITC_W_TGT  TO 360 / MAX(1, ITC_TGT:OBT:PERIOD).
}

// Rotate a vector about an axis, in the direction that carries it the way it is
// already travelling.  Handedness-free for the same reason the rest of this
// file is: the axis is itself a cross product of a position and a velocity, and
// this uses it inside a second one, so a sign convention on the operator
// cancels.  (r x v) x r = v (r.r) under either rule.
FUNCTION itcRotAbout {
  PARAMETER vecIn, axisHat, degIn.
  LOCAL par  IS axisHat * VDOT(vecIn, axisHat).
  LOCAL perp IS vecIn - par.
  IF perp:MAG < 0.001 { RETURN vecIn. }
  LOCAL side IS VCRS(axisHat, perp).    // 90 deg on, the way the motion goes
  RETURN par + perp * COS(degIn) + side * SIN(degIn).
}

FUNCTION itcSpinVec {               // rotate vecIn about the spin axis by degIn
  PARAMETER vecIn, degIn.
  RETURN itcRotAbout(vecIn, ITC_KHAT, degIn).
}

FUNCTION itcSiteAt {                // where the launch site is at UT tUT
  PARAMETER tUT.
  RETURN itcSpinVec(ITC_R0, ITC_WSPIN * (tUT - ITC_TREF)).
}

FUNCTION itcInsertAt {              // ...and where the ascent puts us if we go at tUT
  PARAMETER tUT.
  RETURN itcSpinVec(ITC_R0, ITC_WSPIN * (tUT - ITC_TREF) + ITC_ASC_ARC).
}

// Where another vessel is, relative to the body's centre, at a future time.
// The body is itself moving through kOS's frame, so its own future position has
// to come out - identical to relPosAt() in rendezvous.ks.
FUNCTION itcRelPosAt {
  PARAMETER orbtbl, tUT.
  RETURN POSITIONAT(orbtbl, tUT) - POSITIONAT(SHIP:BODY, tUT).
}

// ---------------------------------------------------------------------------
//  Where the TARGET will be - two ways, and they are checked against each other
//
//  The obvious way is POSITIONAT, which is what rendezvous.ks uses and what
//  this script used to use.  The other way needs nothing but a position, an
//  orbit normal and a period: on a circular orbit the future position is the
//  present one rotated about the normal, which is the same construction the
//  launch site gets, for the same reason.
//
//  Both are computed and compared at pre-flight, because a launch window is
//  built entirely on this prediction and a prediction that is quietly not
//  advancing produces a window that looks perfectly reasonable and is a
//  half-day out.  A disagreement is not resolved by picking a favourite: the
//  rotation is exact for a circular orbit and needs no frame assumptions, so
//  that is what a failed cross-check falls back to, and it is said out loud.
// ---------------------------------------------------------------------------
FUNCTION itcTgtRotAt {              // circular propagation from the reading now
  PARAMETER tUT.
  RETURN itcRotAbout(ITC_TGT_R0, ITC_H_HAT, ITC_W_TGT * (tUT - ITC_TREF)).
}

FUNCTION itcTgtPosAt {
  PARAMETER tUT.
  IF ITC_TGT_MODEL = "rotate" { RETURN itcTgtRotAt(tUT). }
  RETURN itcRelPosAt(ITC_TGT, tUT).
}

// How far apart the two models are, a third of an orbit out - far enough that a
// prediction which is not moving cannot hide, near enough that eccentricity has
// not had time to separate them much.
FUNCTION itcTgtModelGap {
  LOCAL tAhead IS ITC_TREF + ITC_TGT:OBT:PERIOD / 3.
  RETURN VANG(itcTgtRotAt(tAhead), itcRelPosAt(ITC_TGT, tAhead)).
}

// Signed phase angle from us to them, measured in the plane the ascent will
// leave us in (the equator, near enough, for a due-east launch).  Positive
// means the target is ahead - the same convention rendezvous.ks uses, which
// matters, because that is the script this hands over to.
FUNCTION itcPhaseOf {
  PARAMETER ourVec, theirVec.
  LOCAL a IS VXCL(ITC_KHAT, ourVec).
  LOCAL b IS VXCL(ITC_KHAT, theirVec).
  IF a:MAG < 1 OR b:MAG < 1 { RETURN 0. }
  LOCAL ang IS VANG(a, b).
  IF VDOT(VCRS(a, b), ITC_KHAT) < 0 { RETURN -ang. }
  RETURN ang.
}

// ---------------------------------------------------------------------------
//  Orbital arithmetic (circular; the target is assumed near-circular, which is
//  checked, and the residue is cleaned up by rendezvous.ks either way)
// ---------------------------------------------------------------------------
FUNCTION itcVcirc {
  PARAMETER rr.
  IF rr <= 0 { RETURN 0. }
  RETURN SQRT(ITC_MU / rr).
}

FUNCTION itcPeriodAtR {
  PARAMETER rr.
  IF rr <= 0 { RETURN 0. }
  RETURN 2 * CONSTANT:PI * SQRT(rr ^ 3 / ITC_MU).
}

FUNCTION itcHohmannT {              // half the period of the transfer ellipse
  PARAMETER rFrom, rTo.
  LOCAL smaT IS (rFrom + rTo) / 2.
  IF smaT <= 0 { RETURN 0. }
  RETURN CONSTANT:PI * SQRT(smaT ^ 3 / ITC_MU).
}

FUNCTION itcHohmannDv {
  PARAMETER rFrom, rTo.
  IF rFrom <= 0 OR rTo <= 0 { RETURN 0. }
  IF ABS(rTo - rFrom) < 1 { RETURN 0. }
  LOCAL smaT IS (rFrom + rTo) / 2.
  LOCAL vA IS SQRT(ITC_MU * (2 / rFrom - 1 / smaT)).
  LOCAL vB IS SQRT(ITC_MU * (2 / rTo - 1 / smaT)).
  RETURN ABS(vA - itcVcirc(rFrom)) + ABS(itcVcirc(rTo) - vB).
}

// ---------------------------------------------------------------------------
//  The parking orbit, and the drift it buys
// ---------------------------------------------------------------------------
//  Under the target where there is room, over it where there is not, and never
//  level with it.  A parking orbit at the target's own radius has no period
//  difference, so the phase we insert with is the phase we keep: the launch
//  window would have to be exact, and no launch window is exact.
// ---------------------------------------------------------------------------
FUNCTION itcParkSide {              // which side of the target we ended up on
  IF ITC_R_PARK < ITC_R_TGT { RETURN "under". }
  RETURN "over".
}

FUNCTION itcParkRadius {
  PARAMETER rTgt.
  LOCAL rFloor IS ITC_BODY_R + ITC_ATM + ITC_PARK_FLOOR.
  LOCAL rr IS MAX(rFloor, rTgt - ITC_PARK_BELOW).
  IF ABS(rr - rTgt) < ITC_PARK_GAP { SET rr TO rTgt + ITC_PARK_ABOVE. }
  RETURN rr.
}

//  How much drift to leave ourselves at insertion.
//
//  An ascent that runs ITC_ASC_SIGMA seconds late does not put us in the wrong
//  place so much as it lets the target run on: it will be sigma * omega_target
//  degrees further round than the plan said.  The buffer is the time the
//  parking orbit's drift takes to undo that, times a safety factor - which is
//  to say the margin is bought in the currency that actually pays for the
//  mistake.  Sizing it as "twenty minutes" instead would be far too much
//  against a fast-drifting orbit and nowhere near enough against a slow one.
//
//  It is asymmetric on purpose.  Early is free; late is a synodic lap.
FUNCTION itcDriftBuffer {
  PARAMETER rPark, omegaTgt.
  LOCAL drift IS ABS(360 / MAX(1, itcPeriodAtR(rPark)) - omegaTgt).
  IF drift < 0.000001 { RETURN ITC_BUF_MAX. }
  RETURN itcClamp(ITC_BUF_K * ITC_ASC_SIGMA * omegaTgt / drift,
                  ITC_BUF_MIN, ITC_BUF_MAX).
}

//  The phase we want the target to be at when we arrive in the parking orbit.
//
//      phi_burn = 180 - omega_target * t_hohmann     the classic lead angle
//      phi_ins  = phi_burn + (omega_park - omega_target) * buffer
//
//  The second line is the buffer above, expressed as an angle: insert with the
//  target that much further round than the transfer wants, and the drift brings
//  it back to the transfer point after exactly `buffer` seconds.  The sign
//  works out for both cases without a special case - parked under the target we
//  gain on it, parked over it we fall behind, and the same formula puts us on
//  the correct side either way.
FUNCTION itcWantedPhase {
  PARAMETER rPark, rTgt, omegaTgt, bufS.
  LOCAL wPark IS 360 / MAX(1, itcPeriodAtR(rPark)).
  LOCAL phiBurn IS 180 - omegaTgt * itcHohmannT(rPark, rTgt).
  RETURN itcWrap(phiBurn + (wPark - omegaTgt) * bufS).
}

// The phase this launch time would produce at insertion, under the ascent model.
FUNCTION itcPhaseIfLaunchedAt {
  PARAMETER tUT.
  RETURN itcPhaseOf(itcInsertAt(tUT), itcTgtPosAt(tUT + ITC_ASC_TIME)).
}

FUNCTION itcWindowErrAt {           // how wrong that would be, in degrees
  PARAMETER tUT.
  RETURN itcWrap(itcPhaseIfLaunchedAt(tUT) - ITC_PHASE_WANT).
}

// ---------------------------------------------------------------------------
//  The window search
//
//  The error is a straight line in the launch time, and that is not an
//  approximation: our insertion point goes round at the body's rotation rate,
//  the target goes round at its own, and the difference between them is a
//  constant.  So the root is a division, not a search:
//
//      sweep    = d(error) / d(launch time)    measured over a short baseline
//      t_window = t_start - error(t_start) / sweep,  brought forward a lap at
//                                                    a time until it is ahead
//
//  MEASURED, not assumed, and that word is doing real work.  The sweep can also
//  be written down in one line from the two periods - and if the prediction the
//  script is actually using disagrees with that line, then something in the
//  prediction is wrong and every launch time built on it is wrong with it.  So
//  the rate is taken from the very function the root will be found in, the
//  analytic value is computed beside it, and the two are printed together.  A
//  blind scan cannot do that: it returns a time either way, and says nothing
//  about whether the world it searched resembles the one outside.
//
//  The closed form is then VERIFIED rather than trusted - an eccentric target
//  does not sweep quite uniformly - and a local bisection cleans up whatever
//  the eccentricity left.  Only if all of that fails does it fall back to
//  sweeping the whole horizon, and then with a step small enough that it cannot
//  alias.  A fixed step against a fast sweep steps straight over crossings,
//  eventually finds one that happens to land inside a step, and reports a
//  window hours away with complete confidence.
//
//  Two things can still legitimately produce no window.  A target in a
//  synchronous orbit turns at the same rate as the body underneath it, so the
//  error never sweeps and there is nothing to wait for - the phasing has to be
//  solved in orbit, which is what rendezvous.ks is for.  A near-synchronous one
//  has a window, but hours out.  Both are reported as what they are.
// ---------------------------------------------------------------------------
FUNCTION itcSweepRate {             // deg of insertion phase per second of delay
  LOCAL tA IS ITC_TREF + ITC_MIN_LEAD.
  LOCAL dt IS 30.
  LOCAL eA IS itcPhaseIfLaunchedAt(tA).
  LOCAL eB IS itcPhaseIfLaunchedAt(tA + dt).
  // Wrapped, so a baseline that happens to straddle +-180 does not read as a
  // 360 deg per 30 s sweep.  Thirty seconds is unambiguous up to 6 deg/s, which
  // is thirty times faster than anything in low orbit sweeps.
  RETURN itcWrap(eB - eA) / dt.
}

FUNCTION itcSweepAnalytic {         // what that rate has to be, from the periods
  RETURN ITC_W_TGT - ITC_WSPIN.
}

FUNCTION itcBisectWindow {          // bisect a bracket known to hold a crossing
  PARAMETER loIn, hiIn.
  LOCAL lo IS loIn.
  LOCAL hi IS hiIn.
  FROM { LOCAL i IS 0. } UNTIL i >= ITC_BISECT STEP { SET i TO i + 1. } DO {
    LOCAL mid IS (lo + hi) / 2.
    IF itcWindowErrAt(lo) * itcWindowErrAt(mid) <= 0 {
      SET hi TO mid.
    } ELSE {
      SET lo TO mid.
    }
  }
  RETURN (lo + hi) / 2.
}

FUNCTION itcRefineWindow {          // hunt a crossing either side of a guess
  PARAMETER tGuess, lapS.
  LOCAL step IS MAX(1, lapS / 48).
  LOCAL tPrev IS MAX(ITC_TREF + ITC_MIN_LEAD, tGuess - lapS / 4).
  LOCAL ePrev IS itcWindowErrAt(tPrev).
  LOCAL tNow IS tPrev + step.
  LOCAL stop IS tGuess + lapS / 4.
  UNTIL tNow > stop {
    LOCAL eNow IS itcWindowErrAt(tNow).
    IF ePrev * eNow < 0 AND ABS(eNow - ePrev) < 90 {
      RETURN LIST(itcBisectWindow(tPrev, tNow), TRUE).
    }
    SET ePrev TO eNow.
    SET tPrev TO tNow.
    SET tNow TO tNow + step.
  }
  RETURN LIST(tGuess, FALSE).
}

FUNCTION itcScanWindow {            // the fallback: sweep the whole horizon
  PARAMETER stepIn.
  LOCAL step IS MAX(1, stepIn).
  LOCAL tStart IS ITC_TREF + ITC_MIN_LEAD.
  LOCAL ePrev IS itcWindowErrAt(tStart).
  LOCAL tBest IS tStart.
  LOCAL eBest IS ABS(ePrev).
  LOCAL tPrev IS tStart.
  LOCAL tNow IS tStart + step.
  UNTIL tNow > ITC_TREF + ITC_MAX_WAIT {
    LOCAL eNow IS itcWindowErrAt(tNow).
    IF ABS(eNow) < eBest {
      SET eBest TO ABS(eNow).
      SET tBest TO tNow.
    }
    // A sign change with a small step in it is a crossing.  One with 300
    // degrees in it is the wrap, and bisecting that lands on the launch time
    // that is worst - exactly half a lap from the one we want.
    IF ePrev * eNow < 0 AND ABS(eNow - ePrev) < 90 {
      RETURN LIST(itcBisectWindow(tPrev, tNow), TRUE).
    }
    SET ePrev TO eNow.
    SET tPrev TO tNow.
    SET tNow TO tNow + step.
  }
  RETURN LIST(tBest, FALSE).
}

FUNCTION itcFindWindow {
  SET ITC_SWEEP TO itcSweepRate().
  SET ITC_LAP   TO 0.
  LOCAL tStart IS ITC_TREF + ITC_MIN_LEAD.

  IF ABS(ITC_SWEEP) > 0.000001 {
    SET ITC_LAP TO ABS(360 / ITC_SWEEP).
    LOCAL dtRoot IS -itcWindowErrAt(tStart) / ITC_SWEEP.
    UNTIL dtRoot >= 0 { SET dtRoot TO dtRoot + ITC_LAP. }
    IF dtRoot <= ITC_MAX_WAIT - ITC_MIN_LEAD {
      LOCAL tRoot IS tStart + dtRoot.
      IF ABS(itcWindowErrAt(tRoot)) <= ITC_WIN_TOL { RETURN LIST(tRoot, TRUE). }
      // The closed form landed near it but not on it - an eccentric target, or
      // a sweep that is not quite constant.  Hunt the crossing around the guess.
      LOCAL fix IS itcRefineWindow(tRoot, ITC_LAP).
      IF fix[1] { RETURN LIST(fix[0], TRUE). }
    }
  }

  // No usable rate, or nothing to refine.  Sweep, with a step that cannot step
  // over a crossing.
  LOCAL step IS ITC_SCAN_STEP.
  IF ITC_LAP > 0 { SET step TO MIN(step, ITC_LAP / 24). }
  RETURN itcScanWindow(step).
}

// ---------------------------------------------------------------------------
//  The plane, and why there is no window for it
//
//  Reported, not waited for.  A launch site crosses the target's orbital plane
//  twice a day, and a rocket that can pick its azimuth launches at exactly that
//  moment and pays nothing for the inclination.  A spaceplane on a runway
//  cannot: it goes where the centreline points, ascent.ks flies that heading,
//  and the orbit it delivers is equatorial whenever it launches.  An equatorial
//  orbit crosses the target's plane at the target's own nodes - always, at the
//  same relative inclination, regardless of the launch time - so waiting buys
//  nothing here and the plane change is simply part of the bill.
//
//  The crossing time and the azimuth that would have been free are printed
//  anyway, because a pilot who wants to hand-fly a plane-matched climb is owed
//  the numbers, and because seeing "the plane crossing is 3 hours away and the
//  phase window is 20 minutes away" makes it obvious why the script did not try
//  to have both.
// ---------------------------------------------------------------------------
FUNCTION itcPlaneOffsetAt {         // how far the site is out of the target's plane
  PARAMETER tUT.
  RETURN VDOT(itcSiteAt(tUT):NORMALIZED, ITC_H_HAT).
}

FUNCTION itcNextPlaneCross {        // UT of the next time the site is in that plane
  LOCAL step IS ITC_TROT / 240.
  LOCAL tPrev IS ITC_TREF.
  LOCAL ePrev IS itcPlaneOffsetAt(tPrev).
  LOCAL tNow IS ITC_TREF + step.
  UNTIL tNow > ITC_TREF + ITC_TROT * 1.05 {
    LOCAL eNow IS itcPlaneOffsetAt(tNow).
    IF ePrev * eNow < 0 {
      LOCAL lo IS tPrev.
      LOCAL hi IS tNow.
      FROM { LOCAL i IS 0. } UNTIL i >= 24 STEP { SET i TO i + 1. } DO {
        LOCAL mid IS (lo + hi) / 2.
        IF itcPlaneOffsetAt(lo) * itcPlaneOffsetAt(mid) <= 0 {
          SET hi TO mid.
        } ELSE {
          SET lo TO mid.
        }
      }
      RETURN (lo + hi) / 2.
    }
    SET ePrev TO eNow.
    SET tPrev TO tNow.
    SET tNow TO tNow + step.
  }
  RETURN -1.
}

//  sin(azimuth) = cos(inclination) / cos(latitude), which is the whole of
//  launch-azimuth theory for a circular orbit.  A polar orbit gives azimuth
//  zero, which is a real answer and not a failure - hence the sentinel below
//  rather than a zero test.
FUNCTION itcPlaneAzimuth {          // heading that would match that plane, if any
  PARAMETER incDeg.
  LOCAL cl IS COS(SHIP:LATITUDE).
  IF ABS(cl) < 0.0001 { RETURN 90. }
  LOCAL sinAz IS COS(incDeg) / cl.
  IF ABS(sinAz) > 1 { RETURN -999. }  // the site is nearer the pole than the
                                      // orbit ever gets; no heading reaches it
  RETURN ARCSIN(sinAz).
}

// ---------------------------------------------------------------------------
//  What do the scripts we are about to call actually take?
//
//  This one is here because of a flight that got all the way to T-0 and then
//  stopped with "Number of arguments passed in didn't match the number of
//  DECLARE PARAMETERs" - after the countdown, on the runway, with the window
//  spent.  The three scripts this one drives are ordinary files that people
//  edit, replace with their own, and copy between saves, and a caller that
//  assumes their signatures is a caller that fails at the worst possible
//  moment.  kOS has no way to catch that error, so the only defence is to look
//  first, and to look on the ground.
//
//  Program parameters are declared at the top of a file, before any function,
//  so the search stops at the first FUNCTION.  Comments are not stripped -
//  which is why the whole phrase "DECLARE PARAMETER" is required rather than
//  the word on its own; prose about parameters is common, that exact statement
//  in a comment is not.
// ---------------------------------------------------------------------------
FUNCTION itcFilePath {
  PARAMETER pathIn.
  IF EXISTS(pathIn + ".ks") { RETURN pathIn + ".ks". }
  IF EXISTS(pathIn) { RETURN pathIn. }
  RETURN "".
}

FUNCTION itcParamCount {
  PARAMETER pathIn.
  LOCAL fp IS itcFilePath(pathIn).
  IF fp = "" { RETURN -1. }
  LOCAL vf IS OPEN(fp).
  IF NOT vf:HASSUFFIX("READALL") { RETURN -1. }
  LOCAL cont IS vf:READALL.
  IF NOT cont:HASSUFFIX("STRING") { RETURN -1. }
  // Not "up": UP is a bound variable in kOS and declaring over it is an error
  // the compiler is entirely right to make.
  LOCAL txt IS cont:STRING:TOUPPER.
  LOCAL cut IS txt:FIND("FUNCTION ").
  IF cut > 0 { SET txt TO txt:SUBSTRING(0, cut). }

  LOCAL total IS 0.
  LOCAL at IS txt:FIND("DECLARE PARAMETER").
  UNTIL at < 0 {
    LOCAL from IS at + 17.
    LOCAL rest IS txt:SUBSTRING(from, txt:LENGTH - from).
    LOCAL stop IS rest:FIND(".").
    IF stop < 0 { SET stop TO rest:LENGTH. }
    // One statement may name several: `DECLARE PARAMETER a, b.`  A default of
    // 0.5 puts a full stop inside the number and cuts the statement short,
    // which loses nothing that is being counted.
    SET total TO total + rest:SUBSTRING(0, stop):SPLIT(","):LENGTH.
    SET txt TO rest.
    SET at TO txt:FIND("DECLARE PARAMETER").
  }
  RETURN total.
}

// ---------------------------------------------------------------------------
//  Fuel network  --  which tanks are actually ours?
//
//  The same part-tree walk ascent.ks and rendezvous.ks do, and here for the
//  same reason: a fuelled payload must not be counted as budget.  This script
//  does not spend any of it - it only reports, and refuses a mission that is
//  obviously unfunded before the pilot spends forty minutes waiting for it.
// ---------------------------------------------------------------------------
FUNCTION itcIsSeparator {
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

FUNCTION itcMapFeed {
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
    IF NOT itcIsSeparator(prt) {
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

FUNCTION itcResShip {
  PARAMETER rname.
  LOCAL total IS 0.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = rname { SET total TO total + res:AMOUNT. }
  }
  RETURN total.
}

FUNCTION itcResCore {
  PARAMETER rname.
  LOCAL total IS 0.
  FOR prt IN ITC_CORE_TANKS {
    FOR res IN prt:RESOURCES {
      IF res:NAME = rname { SET total TO total + res:AMOUNT. }
    }
  }
  RETURN total.
}

FUNCTION itcResDensity {
  PARAMETER rname, fallback.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = rname AND res:DENSITY > 0 { RETURN res:DENSITY. }
  }
  RETURN fallback.
}

FUNCTION itcRocketIsp {
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
  RETURN 305.
}

FUNCTION itcRocketDv {              // closed-cycle dV in our own tanks (m/s)
  LOCAL lfAmt IS itcResCore("LiquidFuel").
  LOCAL oxAmt IS itcResCore("Oxidizer").
  LOCAL lfUse IS MIN(lfAmt, oxAmt * 9 / 11).
  LOCAL oxUse IS lfUse * 11 / 9.
  LOCAL m0 IS SHIP:MASS.
  LOCAL m1 IS m0 - (lfUse * ITC_LF_DENS + oxUse * ITC_OX_DENS).
  IF m0 <= 0 OR m1 <= 0 OR m1 >= m0 { RETURN 0. }
  RETURN ITC_RKT_ISP * 9.80665 * LN(m0 / m1).
}

// ---------------------------------------------------------------------------
//  Docking ports  --  ours before the launch, theirs after the rendezvous
// ---------------------------------------------------------------------------
// The same test dock.ks uses, deliberately: if dock.ks would accept the map
// selection as a port then so must this, or a port the pilot picked gets
// treated as a vessel and every suffix read off it afterwards is wrong.
FUNCTION itcIsPort {
  PARAMETER thing.
  RETURN thing:HASSUFFIX("NODEPOSITION") AND thing:HASSUFFIX("STATE").
}

FUNCTION itcPortFree {
  PARAMETER prt.
  LOCAL st IS prt:STATE:TOLOWER.
  RETURN NOT (st:CONTAINS("docked") OR st:CONTAINS("preattached")).
}

// A shielded port reports "disabled" until its shield is open.  Ours we can
// open (dock.ks does); theirs we cannot, so a shut one is not a candidate.
FUNCTION itcPortShut {
  PARAMETER prt.
  RETURN prt:STATE:TOLOWER:CONTAINS("disabled").
}

FUNCTION itcPortType {
  PARAMETER prt.
  IF prt:HASSUFFIX("NODETYPE") { RETURN prt:NODETYPE:TOLOWER. }
  RETURN "".
}

FUNCTION itcPortAxis {
  PARAMETER prt.
  IF prt:HASSUFFIX("PORTFACING") { RETURN prt:PORTFACING:FOREVECTOR. }
  LOCAL offs IS prt:NODEPOSITION - prt:POSITION.
  IF offs:MAG > 0.02 { RETURN offs:NORMALIZED. }
  RETURN prt:FACING:FOREVECTOR.
}

FUNCTION itcOurFreePorts {
  LOCAL out IS LIST().
  FOR prt IN SHIP:DOCKINGPORTS {
    IF itcPortFree(prt) { out:ADD(prt). }
  }
  RETURN out.
}

// ---------------------------------------------------------------------------
//  Reporting
// ---------------------------------------------------------------------------
//  withDv is FALSE once the ship is docked.  SHIP is then the whole stack -
//  station included - so its mass is not the mass any of our burns would be
//  flown at, and a dV figure computed from it would be a fiction.  The
//  propellant in our own tanks is still our own propellant, so that is still
//  worth printing.
FUNCTION itcResourceLine {
  PARAMETER label, withDv.
  LOCAL tail IS " u".
  IF withDv { SET tail TO " u  -> " + ROUND(itcRocketDv()) + " m/s". }
  PRINT "  " + label + ": mass " + ROUND(SHIP:MASS, 2) + " t,  LF/Ox " +
        ROUND(itcResCore("LiquidFuel")) + "/" + ROUND(itcResCore("Oxidizer")) +
        tail + ",  mono " + ROUND(itcResShip("MonoPropellant")) + " u.".
}

FUNCTION itcHandBack {
  LOCK THROTTLE TO 0.
  UNLOCK THROTTLE.
  UNLOCK STEERING.
  SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
  SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
  SET CONFIG:IPU TO ITC_IPU_SAVED.
}

// ---------------------------------------------------------------------------
//  Calibration file
//
//  The ascent model is two numbers, and the only honest source for them is the
//  last flight.  They are stored per body and per parking altitude band,
//  because a climb to 80 km over Kerbin is not the climb to 250 km over Laythe,
//  and quietly reusing one for the other is worse than the seed value.
// ---------------------------------------------------------------------------
FUNCTION itcCalKey {
  PARAMETER parkAlt.
  RETURN SHIP:BODY:NAME + "_" + ROUND(parkAlt / 10000) * 10.
}

FUNCTION itcCalLoad {
  PARAMETER parkAlt.
  IF NOT ITC_USE_CAL { RETURN FALSE. }
  IF NOT EXISTS(ITC_CAL_PATH) { RETURN FALSE. }
  LOCAL cal IS READJSON(ITC_CAL_PATH).
  LOCAL key IS itcCalKey(parkAlt).
  IF NOT cal:HASKEY(key) { RETURN FALSE. }
  LOCAL rec IS cal[key].
  IF NOT rec:HASKEY("t_asc") OR NOT rec:HASKEY("arc_deg") { RETURN FALSE. }
  // A calibration file is a file, and files get edited, copied between saves
  // and truncated by crashes.  Anything outside these bounds is not a
  // measurement of an ascent, so the seed is better than it is.
  IF rec["t_asc"] < 120 OR rec["t_asc"] > 7200 { RETURN FALSE. }
  IF ABS(rec["arc_deg"]) > 3600 { RETURN FALSE. }
  SET ITC_ASC_TIME TO rec["t_asc"].
  SET ITC_ASC_ARC  TO rec["arc_deg"].
  IF rec:HASKEY("flights") { SET ITC_CAL_N TO rec["flights"]. }
  RETURN TRUE.
}

FUNCTION itcCalSave {
  PARAMETER parkAlt, tAsc, arcDeg.
  IF NOT ITC_USE_CAL { RETURN FALSE. }
  // Writing to the archive needs a link to the KSC.  In orbit over the far side
  // with no relay that link is down, and the write throws rather than failing
  // quietly, so ask first.  The numbers are printed either way.
  IF ITC_CAL_PATH:STARTSWITH("0:") AND NOT HOMECONNECTION:ISCONNECTED { RETURN FALSE. }
  LOCAL cal IS LEXICON().
  IF EXISTS(ITC_CAL_PATH) { SET cal TO READJSON(ITC_CAL_PATH). }
  LOCAL key IS itcCalKey(parkAlt).
  LOCAL rec IS LEXICON().
  // A running mean, not the last flight: one climb flown through a payload
  // change or a bad hand-off is not the airframe's number.
  LOCAL n IS 0.
  IF cal:HASKEY(key) {
    LOCAL old IS cal[key].
    IF old:HASKEY("flights") { SET n TO old["flights"]. }
    IF n > 0 AND old:HASKEY("t_asc") AND old:HASKEY("arc_deg") {
      LOCAL w IS MIN(n, 4).          // forget slowly, but do forget
      rec:ADD("t_asc",   (old["t_asc"] * w + tAsc) / (w + 1)).
      rec:ADD("arc_deg", (old["arc_deg"] * w + arcDeg) / (w + 1)).
    }
  }
  IF NOT rec:HASKEY("t_asc") {
    rec:ADD("t_asc", tAsc).
    rec:ADD("arc_deg", arcDeg).
  }
  rec:ADD("flights", n + 1).
  rec:ADD("last_t_asc", tAsc).
  rec:ADD("last_arc_deg", arcDeg).
  IF cal:HASKEY(key) { SET cal[key] TO rec. } ELSE { cal:ADD(key, rec). }
  WRITEJSON(cal, ITC_CAL_PATH).
  RETURN TRUE.
}

// ---------------------------------------------------------------------------
//  0. PRE-FLIGHT
// ---------------------------------------------------------------------------
PRINT "=== RAPIER SSTO :: intercept - runway to hard dock ===".
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
LOCK THROTTLE TO 0.
SAS OFF.
BRAKES ON.

SET ITC_GO      TO TRUE.            // cleared by any check that fails
SET ITC_WHY     TO "".              // ...and why, for the closing report
SET ITC_T0      TO TIME:SECONDS.
SET ITC_CAL_N   TO 0.
SET ITC_TGT     TO SHIP.            // typed placeholders, replaced below
SET ITC_TGT_PORT TO SHIP.
SET ITC_HAVE_TGT_PORT TO FALSE.
// Names are kept as strings, not read off the objects at the end.  A hard dock
// MERGES the two vessels: the target's Vessel object stops referring to
// anything, and a report that dereferences it at the finish line throws on the
// one flight that went perfectly.
SET ITC_TGT_NAME TO "".
SET ITC_BERTH    TO "".
SET ITC_PARTS_PRE TO SHIP:PARTS:LENGTH.
SET ITC_T_LAUNCH TO ITC_T0.
// Every variable that appears in a compound condition below is given a value
// here, whether that branch will reach it or not.  kOS does NOT short-circuit:
// in `IF ITC_GO AND ITC_WAIT > 0` the right-hand side is evaluated even when
// ITC_GO is false, so a check that failed before the window was solved would
// crash the script on an undefined name instead of skipping the countdown.
SET ITC_WAIT   TO 0.
SET ITC_T_GO   TO ITC_T0.
SET ITC_EXACT_WIN TO FALSE.
SET ITC_DOCKED TO FALSE.
SET ITC_SWEEP  TO 0.
SET ITC_LAP    TO 0.
SET ITC_TGT_R0 TO V(1, 0, 0).
SET ITC_TGT_V0 TO V(0, 1, 0).
SET ITC_H_TGT  TO V(0, 0, 1).
SET ITC_H_HAT  TO V(0, 0, 1).
SET ITC_W_TGT  TO 0.
SET ITC_MODEL_GAP TO 0.
SET ITC_TGT_ECC TO 0.
SET ITC_ARGS_GUESSED TO FALSE.

// --- 0a. Which vessel are we meeting? ---------------------------------------
//  Same acquisition rules as rendezvous.ks, with one addition: if the map
//  selection is a docking PORT rather than a vessel, that is a better answer
//  than the vessel is.  It names the ship and the berth in one go, and dock.ks
//  needs the port anyway - so it is carried through the whole mission instead
//  of being re-guessed at the far end.
SET ITC_FOUND TO FALSE.

IF targetName = "" {
  IF HASTARGET {
    IF itcIsPort(TARGET) {
      IF TARGET:HASSUFFIX("SHIP") {
        SET ITC_TGT_PORT TO TARGET.
        SET ITC_HAVE_TGT_PORT TO TRUE.
        SET ITC_TGT TO ITC_TGT_PORT:SHIP.
        SET ITC_FOUND TO TRUE.
        PRINT "Target port taken from the map: " + ITC_TGT_PORT:TITLE +
              " on " + ITC_TGT:NAME + ".".
      } ELSE {
        // Pre-1.0 kOS cannot say which vessel a part belongs to, and guessing
        // is not on: the answer decides what we launch at.
        PRINT "!! A port is targeted, but this kOS cannot say which vessel it".
        PRINT "   is on. Target the vessel instead and the script will pick".
        PRINT "   the berth itself once it is alongside.".
      }
    } ELSE {
      SET ITC_TGT TO TARGET.
      SET ITC_FOUND TO TRUE.
      PRINT "Target taken from the map selection: " + ITC_TGT:NAME.
    }
  } ELSE {
    PRINT "!! No target. Select the vessel (or its docking port) on the map,".
    PRINT "   or name it:".
    PRINT "   RUN intercept(""Station Alpha"").".
  }
} ELSE {
  SET ITC_VLIST TO LIST().
  LIST TARGETS IN ITC_VLIST.
  SET ITC_WANT TO targetName:TOLOWER.
  SET ITC_EXACT TO LIST().
  SET ITC_PART  TO LIST().
  FOR ves IN ITC_VLIST {
    LOCAL nm IS ves:NAME:TOLOWER.
    IF nm = ITC_WANT { ITC_EXACT:ADD(ves). }
    ELSE IF nm:CONTAINS(ITC_WANT) { ITC_PART:ADD(ves). }
  }
  IF ITC_EXACT:LENGTH >= 1 {
    SET ITC_TGT TO ITC_EXACT[0].
    SET ITC_FOUND TO TRUE.
  } ELSE IF ITC_PART:LENGTH = 1 {
    SET ITC_TGT TO ITC_PART[0].
    SET ITC_FOUND TO TRUE.
    PRINT "Matched """ + targetName + """ to " + ITC_TGT:NAME + ".".
  } ELSE IF ITC_PART:LENGTH > 1 {
    PRINT "!! """ + targetName + """ is ambiguous - " + ITC_PART:LENGTH + " matches:".
    FOR ves IN ITC_PART { PRINT "     " + ves:NAME. }
    PRINT "   Give more of the name, or select it on the map.".
  } ELSE {
    PRINT "!! No vessel matching """ + targetName + """ is in flight.".
  }
}

IF ITC_FOUND {
  SET ITC_TGT_NAME TO ITC_TGT:NAME.
} ELSE {
  SET ITC_GO TO FALSE.
  SET ITC_WHY TO "no target".
}

// --- 0b. Is this a mission this script can fly? -----------------------------
IF ITC_GO {
  SET TARGET TO ITC_TGT.            // so the map and the navball agree with us
  IF ITC_TGT:BODY:NAME <> SHIP:BODY:NAME {
    PRINT "!! " + ITC_TGT:NAME + " orbits " + ITC_TGT:BODY:NAME + ", not " +
          SHIP:BODY:NAME + ".".
    PRINT "   This is a launch-to-rendezvous script, not a transfer window.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "target is at another body".
  } ELSE IF ITC_TGT:OBT:ECCENTRICITY >= 1 {
    PRINT "!! " + ITC_TGT:NAME + " is on an escape trajectory, not an orbit.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "target is not in orbit".
  } ELSE IF ITC_TGT:OBT:PERIAPSIS < ITC_ATM {
    PRINT "!! " + ITC_TGT:NAME + " has a periapsis of " +
          ROUND(ITC_TGT:OBT:PERIAPSIS / 1000, 1) + " km, inside the atmosphere.".
    PRINT "   It is coming down; there is nothing to launch at.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "target orbit decays into the atmosphere".
  }
}

// --- 0c. Are we actually on a runway, stopped? ------------------------------
//  The launch site is measured off the ship, and the measurement only means
//  anything if the ship is sitting still on the ground.  A ship already in
//  orbit does not want this script at all - it wants rendezvous.ks - and saying
//  so is more useful than solving a launch window for a vessel that has already
//  launched.
IF ITC_GO {
  IF SHIP:STATUS = "ORBITING" OR SHIP:STATUS = "ESCAPING" OR SHIP:STATUS = "SUB_ORBITAL" {
    PRINT "!! This ship is already flying (" + SHIP:STATUS + ").".
    PRINT "   The launch window is behind you - RUN rendezvous. instead.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "already in flight".
  } ELSE IF SHIP:STATUS <> "LANDED" AND SHIP:STATUS <> "PRELAUNCH" {
    PRINT "!! Ship status is " + SHIP:STATUS + ", which is neither landed nor".
    PRINT "   pre-launch. Put it on the runway first.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "not on the ground".
  } ELSE IF SHIP:GROUNDSPEED > 1 {
    PRINT "!! The ship is moving at " + ROUND(SHIP:GROUNDSPEED, 1) + " m/s.".
    PRINT "   The launch site is measured off a stationary ship. Stop first.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "ship is rolling".
  }
}

// --- 0d. Are the scripts we are going to call here, and what do they take? --
//  Cheap to check, and the alternative is finding out after the countdown.
IF ITC_GO {
  SET ITC_MISSING TO "".
  FOR nm IN LIST(ITC_ASCENT, ITC_RENDEZ, ITC_DOCK) {
    IF itcFilePath(ITC_HOME + nm) = "" { SET ITC_MISSING TO ITC_MISSING + " " + nm. }
  }
  IF ITC_MISSING <> "" {
    PRINT "!! Cannot find these scripts on """ + ITC_HOME + """:" + ITC_MISSING.
    PRINT "   Copy them alongside this one, or set ITC_HOME (e.g. ""0:/"").".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "missing scripts".
  }
}

IF ITC_GO AND ITC_CHECK_ARGS {
  IF ITC_ASC_PARAMS  < 0 { SET ITC_ASC_PARAMS  TO itcParamCount(ITC_HOME + ITC_ASCENT). }
  IF ITC_RDV_PARAMS  < 0 { SET ITC_RDV_PARAMS  TO itcParamCount(ITC_HOME + ITC_RENDEZ). }
  IF ITC_DOCK_PARAMS < 0 { SET ITC_DOCK_PARAMS TO itcParamCount(ITC_HOME + ITC_DOCK). }
  PRINT "Scripts: " + ITC_ASCENT + "(" + ITC_ASC_PARAMS + ")  " +
        ITC_RENDEZ + "(" + ITC_RDV_PARAMS + ")  " +
        ITC_DOCK + "(" + ITC_DOCK_PARAMS + ") parameters.".
}

IF ITC_GO {
  // Anything that could not be read is assumed to be the version in this
  // repository, which is the only assumption available - but it is an
  // assumption, so it is printed rather than made silently.
  IF ITC_ASC_PARAMS  < 0 { SET ITC_ASC_PARAMS  TO 1. SET ITC_ARGS_GUESSED TO TRUE. }
  IF ITC_RDV_PARAMS  < 0 { SET ITC_RDV_PARAMS  TO 2. SET ITC_ARGS_GUESSED TO TRUE. }
  IF ITC_DOCK_PARAMS < 0 { SET ITC_DOCK_PARAMS TO 2. SET ITC_ARGS_GUESSED TO TRUE. }
  IF ITC_ARGS_GUESSED {
    PRINT "Scripts: could not read their parameter lists - assuming the".
    PRINT "  signatures this repository ships (1, 2, 2). If yours differ, set".
    PRINT "  ITC_ASC_PARAMS / ITC_RDV_PARAMS / ITC_DOCK_PARAMS by hand.".
  }

  // An ascent script that takes no apoapsis cannot be sent to a parking orbit
  // of our choosing, and the whole plan is built on choosing one.  It can still
  // be flown - but only if we are told where it goes.
  IF ITC_ASC_PARAMS = 0 AND ITC_PARK_FIXED <= 0 AND ITC_RUN_ASC {
    PRINT "!! " + ITC_ASCENT + " declares no parameters, so it cannot be told".
    PRINT "   which orbit to fly to - and the launch window is built around".
    PRINT "   choosing one under the target.".
    PRINT "   Either use the ascent.ks in this repository, which takes an".
    PRINT "   apoapsis, or set ITC_PARK_FIXED to the altitude yours actually".
    PRINT "   reaches (in metres) and the window will be planned around that.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "ascent script cannot be given a parking orbit".
  }
}

// --- 0e. Measure the ship ---------------------------------------------------
IF ITC_GO {
  SET ITC_CORE_UIDS  TO itcMapFeed().
  SET ITC_CORE_TANKS TO LIST().
  SET ITC_PAY_MASS   TO 0.
  SET ITC_RCS_BLOCKS TO 0.
  FOR prt IN SHIP:PARTS {
    LOCAL holds IS FALSE.
    FOR res IN prt:RESOURCES {
      IF res:NAME = "LiquidFuel" OR res:NAME = "Oxidizer" { SET holds TO TRUE. }
    }
    FOR mname IN prt:MODULES {
      IF mname:TOLOWER:CONTAINS("modulercs") { SET ITC_RCS_BLOCKS TO ITC_RCS_BLOCKS + 1. }
    }
    IF ITC_CORE_UIDS:HASKEY(prt:UID) {
      IF holds { ITC_CORE_TANKS:ADD(prt). }
    } ELSE {
      SET ITC_PAY_MASS TO ITC_PAY_MASS + prt:MASS.
    }
  }
  SET ITC_LF_DENS TO itcResDensity("LiquidFuel", 0.005).
  SET ITC_OX_DENS TO itcResDensity("Oxidizer", 0.005).
  SET ITC_RKT_ISP TO itcRocketIsp().

  // A docking mission with nothing to dock with is worth catching now.
  SET ITC_OUR_PORTS TO itcOurFreePorts().
  IF ITC_OUR_PORTS:LENGTH = 0 AND ITC_RUN_DOCK {
    PRINT "!! This ship has no free docking port - every one it has is".
    PRINT "   already docked, or it has none. Nothing to dock with.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "no free docking port aboard".
  } ELSE IF ITC_RCS_BLOCKS = 0 AND ITC_RUN_DOCK {
    PRINT "!! No RCS thrusters. The approach and the docking are flown on RCS.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "no RCS".
  } ELSE IF itcResShip("MonoPropellant") < ITC_MONO_DOCK AND ITC_RUN_DOCK {
    PRINT "!! Only " + ROUND(itcResShip("MonoPropellant")) +
          " units of monopropellant aboard; the approach and docking".
    PRINT "   want about " + ROUND(ITC_MONO_DOCK) + ". Fill up before launching.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "not enough monopropellant".
  }
}

// ---------------------------------------------------------------------------
//  1. THE WINDOW
// ---------------------------------------------------------------------------
IF ITC_GO {
  // --- 1a. Measure the launch site ------------------------------------------
  itcRefresh().

  // Cross-check the axis against the body's own rotation period.  If the ship
  // is not really stationary - brakes off on a slope, a wheel still turning -
  // the measured rate disagrees with the published one, and every launch time
  // computed from it is wrong in a way that looks perfectly reasonable.
  SET ITC_RPERP TO (ITC_R0 - ITC_KHAT * VDOT(ITC_R0, ITC_KHAT)):MAG.
  SET ITC_WMEAS TO 0.
  IF ITC_RPERP > 1 { SET ITC_WMEAS TO CONSTANT:RADTODEG * ITC_V0:MAG / ITC_RPERP. }
  IF ITC_WSPIN > 0 AND ABS(ITC_WMEAS - ITC_WSPIN) > 0.05 * ITC_WSPIN {
    PRINT "!! The launch site's measured rotation rate (" + ROUND(ITC_WMEAS, 5) +
          " deg/s) disagrees".
    PRINT "   with " + SHIP:BODY:NAME + "'s (" + ROUND(ITC_WSPIN, 5) +
          " deg/s). The ship is probably still moving.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "launch site measurement failed".
  }
}

IF ITC_GO {
  // --- 1a'. Read the target, and decide how to predict it -------------------
  //  Both models are evaluated a third of an orbit ahead and compared.  They
  //  describe the same vessel with the same orbit, so they should agree to
  //  within the eccentricity; a large disagreement means one of them is not
  //  advancing, and a launch window built on a prediction that is not advancing
  //  is a launch window half a day out that looks entirely reasonable.
  itcTgtCapture().
  SET ITC_MODEL_GAP TO itcTgtModelGap().
  SET ITC_TGT_ECC   TO ITC_TGT:OBT:ECCENTRICITY.
  IF ITC_TGT_MODEL = "auto" {
    IF ITC_TGT_ECC <= ITC_ECC_CIRC {
      // Circular, or near enough: the rotation is exact, and it is the only one
      // of the two that cannot be wrong about the frame it is working in.
      SET ITC_TGT_MODEL TO "rotate".
    } ELSE IF ITC_MODEL_GAP <= 2 {
      SET ITC_TGT_MODEL TO "posat".
    } ELSE {
      SET ITC_TGT_MODEL TO "rotate".
      PRINT "!! " + ITC_TGT:NAME + " is on an eccentric orbit (e " +
            ROUND(ITC_TGT_ECC, 3) + ") AND the two ways of".
      PRINT "   predicting it disagree by " + ROUND(ITC_MODEL_GAP, 1) +
            " deg. Using the circular one, which".
      PRINT "   the eccentricity will cost accuracy - expect a longer phasing".
      PRINT "   wait in orbit.".
    }
  }
  PRINT "Target model: " + ITC_TGT_MODEL + "  (the two predictions differ by " +
        ROUND(ITC_MODEL_GAP, 2) + " deg a third of an orbit out).".

  // --- 1b. The parking orbit, and the calibration for a climb to it ---------
  SET ITC_R_TGT   TO ITC_TGT:OBT:SEMIMAJORAXIS.
  SET ITC_R_PARK  TO itcParkRadius(ITC_R_TGT).
  SET ITC_PARK_AP TO ITC_R_PARK - ITC_BODY_R.
  IF ITC_PARK_FIXED > 0 AND ITC_ASC_PARAMS = 0 {
    // An ascent script that cannot be told an apoapsis flies the one it flies;
    // the plan has to be built around that rather than around a choice.
    SET ITC_PARK_AP TO ITC_PARK_FIXED.
    SET ITC_R_PARK  TO ITC_BODY_R + ITC_PARK_AP.
    PRINT "Parking orbit fixed at " + ROUND(ITC_PARK_AP / 1000, 1) +
          " km - the one your ascent script flies to.".
  }

  IF ITC_PARK_AP > ITC_PARK_CEIL {
    PRINT "!! " + ITC_TGT:NAME + " is at " +
          ROUND((ITC_R_TGT - ITC_BODY_R) / 1000) + " km, which puts the parking".
    PRINT "   orbit at " + ROUND(ITC_PARK_AP / 1000) + " km - above the " +
          ROUND(ITC_PARK_CEIL / 1000) + " km ceiling this script will plan to.".
    PRINT "   Launch to a low orbit and transfer up in stages instead.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "target too high to launch straight at".
  } ELSE {
    IF itcCalLoad(ITC_PARK_AP) {
      PRINT "Ascent model: calibrated from " + ITC_CAL_N + " flight(s) - " +
            ROUND(ITC_ASC_TIME) + " s, " + ROUND(ITC_ASC_ARC, 1) + " deg of arc.".
    } ELSE {
      PRINT "Ascent model: uncalibrated seed - " + ROUND(ITC_ASC_TIME) + " s, " +
            ROUND(ITC_ASC_ARC, 1) + " deg of arc.".
      PRINT "  (this flight measures both and writes them to " + ITC_CAL_PATH + ")".
    }
  }
}

IF ITC_GO {
  // --- 1c. What we want, and when we can have it ----------------------------
  SET ITC_BUF TO itcDriftBuffer(ITC_R_PARK, ITC_W_TGT).
  SET ITC_PHASE_WANT TO itcWantedPhase(ITC_R_PARK, ITC_R_TGT, ITC_W_TGT, ITC_BUF).

  SET ITC_SOLN  TO itcFindWindow().
  SET ITC_T_GO  TO ITC_SOLN[0].
  SET ITC_EXACT_WIN TO ITC_SOLN[1].
  SET ITC_WAIT  TO ITC_T_GO - TIME:SECONDS.

  // The plane, for information (see itcNextPlaneCross above for why it is only
  // information).  The normal came off the capture above; nothing precesses in
  // KSP, so it is as good later as it is now.
  SET ITC_DI    TO VANG(ITC_H_TGT, ITC_KHAT).
  SET ITC_PLANE_DV TO 0.
  IF ITC_DI > ITC_PLANE_SKIP_DI {
    SET ITC_PLANE_DV TO 2 * itcVcirc(ITC_R_PARK) * SIN(ITC_DI / 2).
  }

  // The transfer, priced the way rendezvous.ks will fly it.
  SET ITC_XFER_DV TO itcHohmannDv(ITC_R_PARK, ITC_R_TGT).
  SET ITC_ORB_DV  TO ITC_PLANE_DV + ITC_XFER_DV + 40.   // +40 for the arrival
                                                        // and the mid-course,
                                                        // as rendezvous.ks budgets

  PRINT "------------------------------------------------------".
  PRINT "TARGET :: " + ITC_TGT:NAME.
  PRINT "  Orbit     : " + ROUND(ITC_TGT:OBT:APOAPSIS / 1000, 1) + " x " +
        ROUND(ITC_TGT:OBT:PERIAPSIS / 1000, 1) + " km, inc " +
        ROUND(ITC_TGT:OBT:INCLINATION, 2) + " deg, period " +
        ROUND(ITC_TGT:OBT:PERIOD) + " s".
  PRINT "  Rel inc   : " + ROUND(ITC_DI, 3) + " deg to the orbit we can launch into".
  PRINT "PLAN".
  SET ITC_GAP TO ABS(ITC_R_PARK - ITC_R_TGT).
  PRINT "  Park at   : " + ROUND(ITC_PARK_AP / 1000, 1) + " km (" +
        ROUND(ITC_GAP / 1000, 1) + " km " + itcParkSide() +
        " the target), period " + ROUND(itcPeriodAtR(ITC_R_PARK)) + " s".
  SET ITC_DRIFT TO ABS(360 / itcPeriodAtR(ITC_R_PARK) - ITC_W_TGT).
  SET ITC_SYNODIC TO 360 / MAX(0.0000001, ITC_DRIFT).
  PRINT "  Drift     : " + ROUND(ITC_DRIFT, 5) +
        " deg/s  ->  a full lap of phase every " + itcHMS(ITC_SYNODIC).
  PRINT "  Aim       : insert with the target " + ROUND(ITC_PHASE_WANT, 1) +
        " deg ahead, then " + itcHMS(ITC_BUF) + " of drift to the transfer.".
  SET ITC_SWEEP_WANT TO itcSweepAnalytic().
  PRINT "  Sweep     : " + ROUND(ITC_SWEEP, 5) + " deg/s measured (" +
        ROUND(ITC_SWEEP_WANT, 5) + " from the periods)  ->  a window every " +
        itcHMS(ITC_LAP) + ".".
  // The two numbers above are the same quantity computed two ways.  If they
  // part company, the prediction the window was solved in is not the orbit the
  // target is on, and the launch time is worthless - so it is said here, in
  // front of the launch time, rather than left to be inferred from a rendezvous
  // that goes wrong an hour later.
  IF ABS(ITC_SWEEP_WANT) > 0.000001 AND
     ABS(ITC_SWEEP - ITC_SWEEP_WANT) > ITC_SWEEP_TOL * ABS(ITC_SWEEP_WANT) {
    PRINT "  !! Those two disagree. The predicted motion of the target is not".
    PRINT "     the motion its orbital period implies, so the window below is".
    PRINT "     not trustworthy. Set ITC_TGT_MODEL TO ""rotate"" and re-run,".
    PRINT "     and see docs/INTERCEPT.md.".
  }
}

IF ITC_GO {
  // --- 1c'. A target going the other way round is not a launch problem ------
  //  Same refusal rendezvous.ks makes, made here instead - before the ship has
  //  spent a drop, rather than after a climb that cannot be used.  Anything
  //  over 90 degrees of relative inclination means the orbit is retrograde
  //  relative to ours, and turning our orbit round costs more than the launch
  //  did.  A runway launch cannot be aimed at it at all.
  IF ITC_DI > 90 {
    PRINT "!! " + ITC_TGT:NAME + " orbits the other way round (" +
          ROUND(ITC_DI, 1) + " deg of relative inclination).".
    PRINT "   Matching it from an eastward runway launch costs more than the".
    PRINT "   whole ascent. This needs a retrograde launch, which is not a".
    PRINT "   profile ascent.ks flies. Stopping.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "target orbits retrograde".
  }
}

IF ITC_GO {
  // --- 1d. Say it, including what it is not ---------------------------------
  IF ITC_EXACT_WIN {
    PRINT "WINDOW".
    PRINT "  Launch in : " + itcHMS(ITC_WAIT) + "   (UT " + ROUND(ITC_T_GO) + ")".
    PRINT "  Insertion : " + itcHMS(ITC_ASC_TIME) + " later, with the target " +
          ROUND(itcPhaseIfLaunchedAt(ITC_T_GO), 1) + " deg ahead.".
    LOCAL phiNow IS itcPhaseOf(ITC_R0, itcRelPosAt(ITC_TGT, ITC_TREF)).
    LOCAL tAsap IS ITC_TREF + ITC_MIN_LEAD.
    PRINT "  Right now : the target is " + ROUND(phiNow, 1) +
          " deg round from the runway, and".
    PRINT "              going as soon as possible would insert at " +
          ROUND(itcPhaseIfLaunchedAt(tAsap), 1) + " deg - " +
          ROUND(ABS(itcWindowErrAt(tAsap))) + " deg out.".
  } ELSE {
    PRINT "!! NO WINDOW inside " + itcHMS(ITC_MAX_WAIT) + ".".
    PRINT "   The phase at insertion sweeps at the difference between the".
    PRINT "   target's orbital rate and the body's rotation, and for this".
    PRINT "   target that difference is nearly zero - a synchronous or".
    PRINT "   near-synchronous orbit. No launch time is much better than any".
    PRINT "   other; the phasing has to be solved in orbit, which is what".
    PRINT "   rendezvous.ks does.".
    PRINT "   Going at the best time found: " + itcHMS(ITC_WAIT) + " from now, " +
          ROUND(ABS(itcWindowErrAt(ITC_T_GO))) + " deg from the ideal.".
  }

  SET ITC_PLANE_T TO itcNextPlaneCross().
  IF ITC_DI > ITC_PLANE_SKIP_DI {
    PRINT "PLANE".
    PRINT "  Change    : " + ROUND(ITC_PLANE_DV) +
          " m/s, paid in orbit by rendezvous.ks at the target's node.".
    IF ITC_PLANE_T > 0 {
      LOCAL az IS itcPlaneAzimuth(ITC_DI).
      PRINT "  Note      : the runway passes through the target's plane in " +
            itcHMS(ITC_PLANE_T - TIME:SECONDS) + ",".
      IF az > -900 {
        PRINT "              where a launch on heading " + ROUND(az, 1) + " or " +
              ROUND(180 - az, 1) + " would have cost nothing.".
      } ELSE {
        PRINT "              but the site's latitude is over the target's".
        PRINT "              inclination, so no launch azimuth reaches it.".
      }
      PRINT "              ascent.ks flies the runway heading, so this script".
      PRINT "              does not wait for that - it cannot use it.".
    }
  }

  PRINT "BUDGET".
  itcResourceLine("Aboard", TRUE).
  PRINT "  On orbit  : plane " + ROUND(ITC_PLANE_DV) + " + transfer " +
        ROUND(ITC_XFER_DV) + " + arrival ~40  =  " + ROUND(ITC_ORB_DV) +
        " m/s after the climb.".
  PRINT "              (the climb itself is priced by ascent.ks at T-0, and the".
  PRINT "               deorbit reserve is policed by both of them.)".
  PRINT "------------------------------------------------------".

  IF ITC_PLANE_DV > ITC_PLANE_WARN_DV {
    PRINT "!! The plane change alone is " + ROUND(ITC_PLANE_DV) + " m/s.".
    PRINT "   A runway launch cannot avoid it: the ship reaches an equatorial".
    PRINT "   orbit whenever it goes, and " + ROUND(ITC_DI, 1) + " deg of".
    PRINT "   inclination has to be bought in orbit. Waiting does not help.".
    PRINT "   If the ship cannot afford it, launch a rocket at this one.".
    IF ITC_ABORT_IF_PLANE {
      SET ITC_GO TO FALSE.
      SET ITC_WHY TO "plane change over the limit".
    } ELSE {
      PRINT "   Holding " + ROUND(ITC_HOLD) + " s - this is your chance to stop it.".
      WAIT ITC_HOLD.
    }
  }
}

// ---------------------------------------------------------------------------
//  2. THE COUNTDOWN
// ---------------------------------------------------------------------------
//  The checks above are not instantaneous - the plane warning deliberately
//  holds, the parameter check reads three files, and the plane-crossing scan
//  walks a whole rotation of the body.  If all of that has eaten the lead time,
//  the window solved for is in the past, and launching anyway would be the
//  exact mistake this script exists to prevent.  Solve the next one.
SET ITC_WILL_FLY TO ITC_RUN_ASC OR ITC_RUN_RDV OR ITC_RUN_DOCK.
IF ITC_GO AND NOT ITC_WILL_FLY {
  PRINT "Rehearsal only - nothing will be flown, so nothing is waited for.".
  SET ITC_WAIT TO 0.
}

IF ITC_GO AND ITC_WILL_FLY {
  IF TIME:SECONDS > ITC_T_GO - 10 {
    PRINT "The pre-flight took longer than the lead time. Re-solving.".
    itcRefresh().
    itcTgtCapture().
    SET ITC_SOLN TO itcFindWindow().
    SET ITC_T_GO TO ITC_SOLN[0].
    SET ITC_EXACT_WIN TO ITC_SOLN[1].
    SET ITC_WAIT TO ITC_T_GO - TIME:SECONDS.
    PRINT "  Next window: " + itcHMS(ITC_WAIT) + " from now.".
  }
}

IF ITC_GO AND ITC_WAIT > 0 {
  IF ITC_USE_WARP AND ITC_T_GO - ITC_WARP_LEAD > TIME:SECONDS + 5 {
    PRINT "Warping to T-" + ROUND(ITC_WARP_LEAD) + " s.".
    WARPTO(ITC_T_GO - ITC_WARP_LEAD).
    WAIT UNTIL TIME:SECONDS >= ITC_T_GO - ITC_WARP_LEAD.
    SET WARP TO 0.
    WAIT UNTIL KUNIVERSE:TIMEWARP:ISSETTLED.
  }

  // Re-measure and re-check from where things actually are now.  This is the
  // number that catches a frame problem, a warp overshoot or a ship that moved
  // while we were on rails: the prediction is remade from a fresh reading of
  // the runway, and if the solver and the world still agree it should come out
  // at very nearly zero.
  itcRefresh().
  itcTgtCapture().
  SET ITC_ERR_GO TO itcWindowErrAt(ITC_T_GO).
  PRINT "Countdown. Insertion error re-checked at T-" +
        ROUND(ITC_T_GO - TIME:SECONDS) + " s: " + ROUND(ITC_ERR_GO, 2) + " deg.".
  IF ABS(ITC_ERR_GO) > 5 {
    PRINT "  !! That should be near zero. Something moved while we waited -".
    PRINT "     the launch time below is the one that was solved for, and it".
    PRINT "     is now " + ROUND(ABS(ITC_ERR_GO)) + " deg off. Expect a longer".
    PRINT "     phasing wait in orbit.".
  }

  SET ITC_NEXT_TICK TO 0.
  UNTIL TIME:SECONDS >= ITC_T_GO {
    IF TIME:SECONDS >= ITC_NEXT_TICK {
      PRINT "  T-" + ROUND(ITC_T_GO - TIME:SECONDS) + " s".
      SET ITC_NEXT_TICK TO TIME:SECONDS + ITC_TICK.
    }
    WAIT 0.2.
  }
}

// ---------------------------------------------------------------------------
//  3. ASCENT
// ---------------------------------------------------------------------------
IF ITC_GO AND ITC_RUN_ASC {
  PRINT "======================================================".
  PRINT "T-0. Handing over to " + ITC_ASCENT + " for a " +
        ROUND(ITC_PARK_AP / 1000, 1) + " km orbit.".
  PRINT "======================================================".
  SET ITC_T_LAUNCH TO TIME:SECONDS.
  // Longitude, not a vector.  The arc has to be measured across twenty minutes
  // of flight, and a direction vector does not survive that trip - KSP turns
  // the world frame with the body while the ship is low and stops when it is
  // high, so the two readings would not be in the same frame.  A geographic
  // longitude is body-fixed by definition, and the inertial arc is then the
  // ground arc plus whatever the body turned through while we flew it.
  SET ITC_LNG_LAUNCH TO SHIP:LONGITUDE.
  SET CONFIG:IPU TO ITC_IPU_SAVED.   // ascent.ks sets its own; give it the
                                     // value it expects to find and restore
  // Called the way it is written, not the way we would like it written.  kOS
  // treats an argument count mismatch as a fatal error and there is no way to
  // catch it, which on a launch script means the window is gone.
  IF ITC_ASC_PARAMS >= 1 {
    RUNPATH(ITC_HOME + ITC_ASCENT, ITC_PARK_AP).
  } ELSE {
    RUNPATH(ITC_HOME + ITC_ASCENT).
  }
  SET CONFIG:IPU TO 800.

  // --- 4. Did it work, and what did it actually do? -------------------------
  SET ITC_T_ORBIT TO TIME:SECONDS.
  IF SHIP:PERIAPSIS < ITC_ATM OR SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" {
    PRINT "!! The ascent did not leave a stable orbit (periapsis " +
          ROUND(SHIP:PERIAPSIS / 1000, 1) + " km, status " + SHIP:STATUS + ").".
    PRINT "   Read the ascent report above. Not starting the rendezvous.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "ascent did not reach orbit".
  } ELSE {
    SET ITC_ASC_REAL TO ITC_T_ORBIT - ITC_T_LAUNCH.
    // Eastward, so a negative wrap is a ground track that has gone past 180.
    SET ITC_GROUND_ARC TO itcWrap(SHIP:LONGITUDE - ITC_LNG_LAUNCH).
    IF ITC_GROUND_ARC < 0 { SET ITC_GROUND_ARC TO ITC_GROUND_ARC + 360. }
    SET ITC_ARC_REAL TO ITC_GROUND_ARC + ITC_WSPIN * ITC_ASC_REAL.
    itcRefresh().                   // we are in orbit now; the basis moves with us
    itcTgtCapture().
    SET ITC_OUR_VEC TO SHIP:POSITION - SHIP:BODY:POSITION.
    SET ITC_TGT_VEC TO ITC_TGT:POSITION - SHIP:BODY:POSITION.
    SET ITC_PHASE_REAL TO itcPhaseOf(ITC_OUR_VEC, ITC_TGT_VEC).

    PRINT "------------------------------------------------------".
    PRINT "IN ORBIT :: " + ROUND(SHIP:APOAPSIS / 1000, 1) + " x " +
          ROUND(SHIP:PERIAPSIS / 1000, 1) + " km.".
    PRINT "  Ascent took " + itcHMS(ITC_ASC_REAL) + " (model said " +
          itcHMS(ITC_ASC_TIME) + ") over " + ROUND(ITC_ARC_REAL, 1) +
          " deg of arc (model said " + ROUND(ITC_ASC_ARC, 1) + ").".
    SET ITC_PHASE_ERR TO itcWrap(ITC_PHASE_REAL - ITC_PHASE_WANT).
    SET ITC_DRIFT_REAL TO ABS(360 / MAX(1, SHIP:OBT:PERIOD) - ITC_W_TGT).
    SET ITC_ERR_TIME TO ABS(ITC_PHASE_ERR) / MAX(0.0000001, ITC_DRIFT_REAL).
    PRINT "  Target is " + ROUND(ITC_PHASE_REAL, 1) + " deg ahead; the plan " +
          "wanted " + ROUND(ITC_PHASE_WANT, 1) + " deg.".
    PRINT "  Phase error " + ROUND(ITC_PHASE_ERR, 1) + " deg - about " +
          itcHMS(ITC_ERR_TIME) + " of drift.".
    IF itcCalSave(ITC_PARK_AP, ITC_ASC_REAL, ITC_ARC_REAL) {
      PRINT "  Calibration written to " + ITC_CAL_PATH + ".".
    } ELSE {
      PRINT "  Calibration NOT written (no link, or disabled). To set it by".
      PRINT "  hand: ITC_ASC_TIME " + ROUND(ITC_ASC_REAL) + ", ITC_ASC_ARC " +
            ROUND(ITC_ARC_REAL, 1) + ".".
    }
    PRINT "------------------------------------------------------".
  }
}

// ---------------------------------------------------------------------------
//  5. RENDEZVOUS
// ---------------------------------------------------------------------------
IF ITC_GO AND ITC_RUN_RDV {
  SET TARGET TO ITC_TGT.            // rendezvous.ks with no name uses the map
                                    // selection, and the map selection is the
                                    // vessel we chose - not a name that might
                                    // match two stations
  // Nothing waits here.  The drift buffer this whole launch time was built
  // around is time rendezvous.ks is meant to spend: "stay in this orbit and
  // wait" is one of the candidates its phasing solver prices, and if the
  // window did its job that is the candidate that wins - at zero dV, warped
  // out, and re-solved against the geometry we actually arrived with rather
  // than the one that was predicted on the runway.
  PRINT "Handing over to " + ITC_RENDEZ + ".".
  SET CONFIG:IPU TO ITC_IPU_SAVED.
  IF ITC_RDV_PARAMS >= 2 {
    RUNPATH(ITC_HOME + ITC_RENDEZ, "", parkMetres).
  } ELSE IF ITC_RDV_PARAMS = 1 {
    RUNPATH(ITC_HOME + ITC_RENDEZ, "").
  } ELSE {
    RUNPATH(ITC_HOME + ITC_RENDEZ).
  }
  SET CONFIG:IPU TO 800.

  SET ITC_RANGE TO (ITC_TGT:POSITION):MAG.
  SET ITC_VREL  TO (SHIP:VELOCITY:ORBIT - ITC_TGT:VELOCITY:ORBIT):MAG.
  IF ITC_RANGE > ITC_RDV_RANGE_OK OR ITC_VREL > ITC_RDV_VREL_OK {
    PRINT "!! The rendezvous did not finish: " + ROUND(ITC_RANGE) +
          " m out at " + ROUND(ITC_VREL, 2) + " m/s.".
    PRINT "   Read its report above. Not starting the docking - dock.ks is an".
    PRINT "   approach script, not a rendezvous one.".
    SET ITC_GO TO FALSE.
    SET ITC_WHY TO "rendezvous did not park alongside".
  }
}

// ---------------------------------------------------------------------------
//  6. PICK A BERTH
//
//  dock.ks needs a PORT as the target, not a vessel, and until now the station
//  has been out of physics range - its parts could not even be read.  Now it
//  can, so the choice is made from what is actually there: free, not shielded
//  shut, and the same size as the port we are bringing.  Between equals, the
//  one that is pointing at us wins, because dock.ks's corridor goes out, round
//  and in, and a port on the far side is the long way round a station.
// ---------------------------------------------------------------------------
IF ITC_GO AND ITC_RUN_DOCK {
  IF ITC_HAVE_TGT_PORT {
    PRINT "Berth: the port you picked on the map - " + ITC_TGT_PORT:TITLE + ".".
  } ELSE {
    SET ITC_OUR_TYPE TO "".
    IF ITC_OUR_PORTS:LENGTH > 0 { SET ITC_OUR_TYPE TO itcPortType(ITC_OUR_PORTS[0]). }
    IF ourPortHint <> "" {
      FOR prt IN ITC_OUR_PORTS {
        IF prt:TAG:TOLOWER:CONTAINS(ourPortHint:TOLOWER) OR
           prt:TITLE:TOLOWER:CONTAINS(ourPortHint:TOLOWER) {
          SET ITC_OUR_TYPE TO itcPortType(prt).
        }
      }
    }

    SET ITC_CANDS TO LIST().
    IF ITC_TGT:HASSUFFIX("DOCKINGPORTS") {
      FOR prt IN ITC_TGT:DOCKINGPORTS {
        IF itcPortFree(prt) AND NOT itcPortShut(prt) {
          LOCAL tp IS itcPortType(prt).
          IF tp = "" OR ITC_OUR_TYPE = "" OR tp = ITC_OUR_TYPE { ITC_CANDS:ADD(prt). }
        }
      }
    }

    IF ITC_CANDS:LENGTH = 0 {
      PRINT "!! " + ITC_TGT:NAME + " has no free port we can use - they are".
      PRINT "   occupied, shielded shut, or the wrong size for ours.".
      PRINT "   (Open a shield from the station's side and RUN dock. by hand.)".
      SET ITC_GO TO FALSE.
      SET ITC_WHY TO "no usable port on the target".
    } ELSE {
      SET ITC_BEST TO ITC_CANDS[0].
      SET ITC_BEST_A TO 999.
      PRINT "Berths available on " + ITC_TGT:NAME + ":".
      FOR prt IN ITC_CANDS {
        LOCAL toUs IS SHIP:POSITION - prt:POSITION.
        LOCAL aa IS VANG(itcPortAxis(prt), toUs).
        PRINT "  " + prt:TITLE + "  " + ROUND(aa) + " deg off facing us, " +
              ROUND(toUs:MAG) + " m.".
        IF aa < ITC_BEST_A {
          SET ITC_BEST_A TO aa.
          SET ITC_BEST TO prt.
        }
      }
      SET ITC_TGT_PORT TO ITC_BEST.
      SET ITC_HAVE_TGT_PORT TO TRUE.
      PRINT "Berth: " + ITC_TGT_PORT:TITLE + " (" + ROUND(ITC_BEST_A) +
            " deg off facing us).".
    }
  }
}

// ---------------------------------------------------------------------------
//  7. DOCK
// ---------------------------------------------------------------------------
IF ITC_GO AND ITC_RUN_DOCK {
  SET TARGET TO ITC_TGT_PORT.
  SET ITC_BERTH TO ITC_TGT_PORT:TITLE.
  SET ITC_PARTS_PRE TO SHIP:PARTS:LENGTH.
  PRINT "Handing over to " + ITC_DOCK + ".".
  SET CONFIG:IPU TO ITC_IPU_SAVED.
  IF ITC_DOCK_PARAMS >= 2 {
    RUNPATH(ITC_HOME + ITC_DOCK, ourPortHint, 0).
  } ELSE IF ITC_DOCK_PARAMS = 1 {
    RUNPATH(ITC_HOME + ITC_DOCK, ourPortHint).
  } ELSE {
    RUNPATH(ITC_HOME + ITC_DOCK).
  }
  SET CONFIG:IPU TO 800.
}

// ---------------------------------------------------------------------------
//  8. REPORT
// ---------------------------------------------------------------------------
PRINT "======================================================".
PRINT "INTERCEPT :: report".
IF ITC_FOUND {
  PRINT "  Target    : " + ITC_TGT_NAME.
}
IF ITC_GO {
  // Part count, not port state: the port we docked to now belongs to this ship
  // and the vessel it was on no longer exists.  Growing a part count is the one
  // test that survives the merge - the same one dock.ks watches.
  SET ITC_DOCKED TO SHIP:PARTS:LENGTH > ITC_PARTS_PRE.
  PRINT "  Mission   : " + itcHMS(TIME:SECONDS - ITC_T0) +
        " from the runway, " + itcHMS(TIME:SECONDS - ITC_T_LAUNCH) +
        " of it flying, " + itcHMS(ITC_T_LAUNCH - ITC_T0) + " waiting for it.".
  IF ITC_RUN_DOCK {
    IF ITC_DOCKED {
      PRINT "  DOCKED to " + ITC_BERTH + " on " + ITC_TGT_NAME + ".".
    } ELSE {
      PRINT "  Not docked - read the docking report above.".
    }
  }
  itcResourceLine("Left", NOT ITC_DOCKED).
  IF ITC_RUN_ASC {
    PRINT "  Ascent    : " + itcHMS(ITC_ASC_REAL) + " / " +
          ROUND(ITC_ARC_REAL, 1) + " deg of arc.".
    PRINT "  Window    : insertion phase error " + ROUND(ITC_PHASE_ERR, 1) +
          " deg, worth " + itcHMS(ITC_ERR_TIME) + " of drift.".
    IF ABS(ITC_PHASE_ERR) > 10 {
      PRINT "              That is the ascent model, not the solver: the".
      PRINT "              measurement above has been written back, and the".
      PRINT "              next flight to this orbit will aim with it.".
    }
  }
} ELSE {
  PRINT "  Stopped   : " + ITC_WHY + ".".
  PRINT "  The ship is where the last completed stage left it - each of them".
  PRINT "  reports for itself above, and none of them leave it somewhere it".
  PRINT "  cannot fly out of.".
}
PRINT "======================================================".

itcHandBack().
