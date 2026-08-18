// ============================================================================
//  ascent.ks  --  Runway-to-orbit autopilot for a RAPIER spaceplane
// ----------------------------------------------------------------------------
//  Target : a circular orbit due east, at the apoapsis passed on the command
//           line (metres) or 100 km if none is given.  It must clear the
//           launch body's atmosphere; the script checks and says so if not.
//  Engines: any CR-7 R.A.P.I.E.R. cluster (air-breathing on the runway, closed
//           cycle for the final push to orbit).
//
//  This script is not tied to one airframe.  It measures the ship it is flying
//  and derives the profile from what that ship actually does, so it will fly
//  any RAPIER spaceplane that has the performance to reach orbit at all.
//
//  Design notes (why it flies the way it does)
//  -------------------------------------------
//  * THE AIR-BREATHING CLIMB RIDES A DYNAMIC-PRESSURE CORRIDOR.  A jet's thrust
//    is bought with air, so the altitude the ship should be at is whatever
//    altitude keeps Q near a target as speed builds - which is a climb that
//    starts fast and flattens right out as the ship goes hypersonic.  The
//    command is derived, not hill-climbed: holding Q constant needs a vertical
//    speed of 2*H*a/v (H = the atmosphere's scale height, measured in flight),
//    plus a proportional term that closes any standing Q error.
//      The previous version raised the commanded climb rate whenever
//    acceleration was healthy.  On a ship with thrust to spare that ramps the
//    command to its clamp in seconds: in flight the nose pegged at the 28 deg
//    limit from 13 km up and the ship arrived at 24 km doing 1000 m/s, with the
//    jets down to 25% of peak thrust and nothing left to breathe.  It then had
//    to dive 2 km to restart them.  Spare acceleration is not a reason to
//    climb; it is a reason to go faster in air the engine can still use.
//  * ACCELERATION IS STILL THE RED LINE.  The Q corridor is overruled by the
//    guards: never mush into the ground, never descend once high and fast (a
//    hypersonic dive back into thick air is the most expensive manoeuvre in the
//    flight), and never chase Q the intakes cannot deliver.
//  * THE ROCKET PHASE NEVER STEEPENS FOR SPARE THRUST.  Above the drag layer a
//    steeper climb buys nothing an engine burn would not buy more cheaply, and
//    every degree of it is a cosine loss plus g*sin(gamma) of gravity loss.  So
//    the flight-path command only ever shallows, and it is capped by a schedule
//    that flattens with altitude.  (The previous version steepened on high
//    acceleration, which is exactly backwards: in flight it pushed the command
//    to 17.8 deg and the nose to 31.8 deg at 29 km, ending at MECO with 461 m/s
//    of vertical velocity - speed bought at rocket prices and pointed the wrong
//    way.)
//  * ONE STABLE INNER LOOP.  Pitch is not bumped up and down by a bang-bang
//    test.  A single controller converts a commanded flight-path angle into a
//    pitch command: feed-forward gamma plus a slow, clamped, anti-windup trim
//    that learns this airframe's angle of attack.  Time-to-apoapsis is *not*
//    used as a control variable - it is discontinuous when apoapsis is near the
//    ship's own altitude, which is exactly where the rocket phase starts, and
//    steering on it produced the +-25 deg pitch oscillation seen in flight.
//  * THE MODE SWITCH IS AN ECONOMIC DECISION, NOT A THRESHOLD.  Every sample
//    the script prices both modes in the same currency - tonnes of
//    rocket-equivalent propellant per m/s of speed bought - and breathes for
//    exactly as long as the jets are the cheaper way to buy speed.  Drag is not
//    guessed: it is read out of the force balance the ship is already flying
//    (D = T - m*(a + g*sin(gamma))), which is what makes the closed-cycle side
//    of the comparison honest.  Once the LF-only reserve is gone a tonne of
//    liquid fuel costs the rocket phase a tonne of LF *and* the 1.22 t of
//    oxidiser it would have paired with, so the jet's price rises by 20/9 at
//    that moment and the comparison knows it.
//      A fixed threshold cannot do this.  The previous version refused to look
//    at thrust decay at all below 1150 m/s of airspeed - and a stalling ship is
//    slow by definition.  In flight that gate deadlocked: at 24 km the jets
//    were at 25% thrust with no acceleration left, the switch was disarmed
//    because the ship was only doing 1020 m/s, and the profile dived 2 km to
//    buy the airspeed that would have armed it.
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
//    2. Air-breathing climb: self-tuning dynamic-pressure corridor.
//    3. Closed-cycle push  : flight-path-angle schedule, measured dV policing.
//    4. Coast       : hold the apoapsis through the last of the air, then warp
//                     to the circularisation burn.
//    5. Circularise : fly the velocity error at apoapsis; stop when it stops
//                     shrinking, at the target, or at the deorbit reserve.
//    6. Report      : final orbit, dV left, deorbit funding check.
//    7. Post-flight : re-price the mission from what the ship actually did and
//                     say what the airframe needs changing.
//
//  Run with:   RUN ascent.          - 100 km, the default
//              RUN ascent(150000).  - or any apoapsis, in metres
//              (from the archive or the ship's Script volume)
//  Tunables are grouped at the top so you can trim the profile to your build.
// ============================================================================

// The orbit to aim for, in metres above the surface.  Optional - omit it and
// the script flies the 100 km default.  It has to clear the atmosphere of
// whatever body we are launching from, which is checked once the body has been
// measured (see "Mission target" below); the parameter is only read here.
//
//    RUN ascent.              // 100 km
//    RUN ascent(150000).      // 150 km
//    RUNPATH("ascent", 85000).
//
// (Optional program parameters need kOS 1.0.1 or newer.  On anything older,
// drop the "IS 100000" and always pass a value.)
DECLARE PARAMETER requestedAp IS 100000.

CLEARSCREEN.

// --- Mission ----------------------------------------------------------------
SET REQUESTED_APOAPSIS TO requestedAp.  // orbit we would *like* to reach (m)
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
SET DEPLOY_BEFORE_DEORBIT TO TRUE.  // the payload is released in orbit, so the
                                    // deorbit burn is flown on the light ship and
                                    // the reserve is priced accordingly.  FALSE
                                    // reserves enough to deorbit with the cargo
                                    // still aboard - set it for any flight that
                                    // might have to bring the payload home
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
SET PLAN_SWITCH_ALT  TO 21000.      // assumed altitude at the mode switch (m)
SET PLAN_SWITCH_SPD  TO 1400.       // ... and surface speed there (m/s)
SET PLAN_JET_DV      TO 0.          // jet-phase dV equivalent incl. drag (m/s);
                                    // 0 = derive it from this ship's jet TWR
SET PLAN_JET_DV_K    TO 2.2.        // jet dV per m/s of handover speed at TWR
                                    // 0.5; a draggy, slow-accelerating airframe
                                    // spends far longer fighting drag, so the
                                    // multiplier falls as take-off TWR rises
SET PLAN_LOSS_FACTOR TO 1.36.       // gravity/drag/steering losses, rocket climb.
                                    // This prices the climb *from the handover*,
                                    // so it has to include the opening, which is
                                    // where nearly all of it is spent.  It is the
                                    // whole-climb figure the script prints as
                                    // "over the whole climb" at the end of a
                                    // flight - never the trailing-window one,
                                    // which describes the cheap end of the climb
                                    // and is a different question.
                                    // 1.65 came from a flight flown before the
                                    // AoA cap, which bought its apoapsis at 6-16
                                    // deg of body drag.  The 85 km flight of the
                                    // same airframe with the cap in measured
                                    // x1.36 end to end and reached orbit with
                                    // fuel to spare, so the old number now
                                    // over-states this ascent by ~250 m/s and
                                    // writes off orbits the ship can fly.  See
                                    // docs/CLIMB_LOSS_REVIEW.md.
SET PLAN_TWR_MIN     TO 1.05.       // closed-cycle TWR wanted at the handover
SET PLAN_JET_ISP     TO 3200.       // air-breathing Isp if it cannot be read (s)
SET PLAN_GO_MARGIN   TO 0.08.       // dV in hand, as a fraction of the bill,
                                    // below which the answer is MARGINAL and
                                    // not GO - the model is not accurate to
                                    // better than this
SET PREFLIGHT_HOLD   TO 12.         // pause this long on a failed check (s)

// --- Attitude controller (shared by both powered phases) --------------------
SET AOA_KI       TO 0.30.           // trim gain, deg of trim per deg of error per s
SET AOA_TRIM_MAX TO 14.             // trim authority clamp (deg)

// --- Runway / air-breathing climb -------------------------------------------
//  The profile is a dynamic-pressure corridor: fly whatever climb rate keeps Q
//  near AB_Q_TARGET as the ship accelerates.  That is the altitude band where
//  the intakes are fed and the RAPIER's velocity curve pays, and it produces the
//  classic spaceplane ascent for free - a brisk climb while subsonic, then a
//  long shallow acceleration that drifts up to ~20 km by 1400 m/s.
SET ROTATE_SPEED    TO 80.          // surface speed to rotate the nose up (m/s)
SET ROTATE_PITCH    TO 10.          // fixed pitch used until guidance engages (deg)
SET AB_GUIDE_SPEED  TO 160.         // hand over to the vs controller above this (m/s)
//  Two separate limits, and the distinction matters.  AB_FPA_MAX bounds how
//  steeply the ship may *climb* - that is the physics, and it is the number
//  that decides whether the jets get fed.  AB_PITCH_MAX only bounds where the
//  nose may point, which at low speed is mostly angle of attack the trim needs
//  in order to fly at all.  Conflating them is what let the reference flight
//  sit on a 28 deg pitch limit and call it guidance.
//
//  The 12 deg cap is taken from the profile that actually reached orbit on this
//  airframe: a flat 12 deg *pitch* command, which at cruise AoA is roughly an
//  8 deg flight path.  At 800 m/s that ship climbed 7.4 km per 100 m/s of speed
//  gained; the reference flight, at 28 deg, climbed 21.7 km for the same 100
//  m/s.  That ratio is the entire difference between the two flights.
SET AB_PITCH_MIN    TO -6.          // pitch clamp, air-breathing phase (deg)
SET AB_PITCH_MAX    TO 25.          // ... nose authority, not a climb limit
SET AB_FPA_MAX      TO 12.          // commanded flight path never steeper (deg)
SET AB_TUNE_DT      TO 1.           // profile re-scheduling interval (s)
SET AB_VS_MAX       TO 200.         // commanded climb rate, clamp (m/s)
SET AB_VS_RATE      TO 25.          // how fast the command may move (m/s per s)
SET AB_Q_TARGET     TO 0.30.        // dynamic pressure the corridor starts on (atm)
SET AB_Q_KP         TO 0.04.        // corridor gain (1/s) on ln(Q/Qtarget)
SET AB_ACC_SMOOTH   TO 0.4.         // low-pass on measured accel, 0..1 (1 = raw)

//  The corridor's outer loop: which Q is right for *this* airframe?  A fixed
//  number cannot answer that - it depends on how the ship's drag trades against
//  its intake area, which is a property of the build.  So the target is trimmed
//  slowly against sustained acceleration: plenty of acceleration means the ship
//  can afford thinner air (less drag, more efficient), and sagging acceleration
//  means it is starving and needs to come back down.  Slow, clamped, and only
//  trimmed while the inner loop is actually holding the corridor, so the two
//  cannot wind against each other.
SET AB_Q_ADAPT      TO TRUE.        // FALSE = hold AB_Q_TARGET exactly
SET AB_ACC_TARGET   TO 1.5.         // acceleration the jet phase aims to hold (m/s^2)
SET AB_Q_ADAPT_DT   TO 10.          // outer-loop interval (s)
SET AB_Q_ADAPT_STEP TO 0.06.        // fractional move of the target per interval
SET AB_Q_ADAPT_BAND TO 0.15.        // dead band around AB_ACC_TARGET, fractional
SET AB_Q_TGT_MIN    TO 0.10.        // ... clamped, thin end (atm)
SET AB_Q_TGT_MAX    TO 0.45.        // ... clamped, thick end (atm)
SET AB_Q_ADAPT_TOL  TO 0.25.        // only trim when |ln(Q/target)| is under this
SET AB_FLOOR_ALT    TO 8000.        // hold at least AB_VS_FLOOR under this (m)
SET AB_VS_FLOOR     TO 25.          // ... that minimum climb rate (m/s)
SET AB_Q_MAX        TO 0.55.        // hard ceiling on Q: climb out regardless (atm)
SET AB_VS_QCLIMB    TO 70.          // climb rate forced by the high-Q guard (m/s)
//  Descending is allowed only low and transonic, where a shallow push through
//  the drag rise is cheap.  It is forbidden high up: a hypersonic dive back into
//  thick air spends far more on drag than the airspeed is worth, and it is what
//  the old "too thin to breathe, sink back in" guard actually did.
SET AB_DIVE_ALT_MAX TO 12000.       // no descending above this altitude (m)
SET AB_DIVE_SPEED   TO 280.         // ... nor below this airspeed (m/s)
SET AB_VS_DIVE      TO -15.         // shallow dive rate where it is allowed (m/s)

// --- Mode-switch sensing ----------------------------------------------------
//  Priced, not thresholded.  Both modes are costed in tonnes of
//  rocket-equivalent propellant per m/s of speed bought, and the jets keep
//  running for exactly as long as they are the cheaper of the two.
SET SW_COST_RATIO   TO 1.0.         // switch when jet cost > this x rocket cost
                                    // (>1 = hang on to the jets a little past
                                    // break-even, <1 = leave early)
SET SW_MIN_ACCEL    TO 0.25.        // jets making less than this are done (m/s^2).
                                    // This one is pragmatism, not economics: in
                                    // level flight the jets stay *genuinely*
                                    // cheaper per m/s down to a crawl, so the
                                    // priced test alone will happily cruise for
                                    // ten minutes buying 0.2 m/s^2.  The break-
                                    // even on the reference airframe was 0.39,
                                    // so the economics normally fire first and
                                    // this only catches the pathological case.
SET SW_CONFIRM      TO 3.           // consecutive samples needed to confirm
SET SW_ARM_ALT      TO 6000.        // no economic sensing below this (m) - the
                                    // ship is still climbing out and the
                                    // measurement is noise
SET SW_ARM_DECAY    TO 0.85.        // ... nor until jet thrust is under this
                                    // fraction of its own peak.  This is not a
                                    // switch test, it is a gate against the
                                    // transonic hole: pushing through Mach 1 the
                                    // drag rise can hold acceleration under
                                    // SW_MIN_ACCEL for several seconds while the
                                    // RAPIER's velocity curve is still climbing.
                                    // Thrust does not fall off peak until the
                                    // ship is genuinely high and fast, so this
                                    // costs nothing and rules that window out.
//  Backstops.  The priced test is the right answer but it is not guaranteed to
//  terminate: in level flight the jets stay genuinely cheaper per m/s down to a
//  crawl, so on paper the ship will cruise forever buying 0.2 m/s^2.  The
//  profile that reached orbit on this airframe simply switched at 20 km or
//  1400 m/s, whichever came first, and never had the problem.  These are the
//  same idea with room for a better airframe to earn more.
SET SW_ALT_HARD     TO 25000.       // hard backstop: switch by this altitude (m)
SET SW_SPEED_HARD   TO 1700.        // ... or this airspeed (m/s)
SET SW_Q_HARD       TO 0.04.        // ... or this dynamic pressure (atm).  Below
                                    // this the intakes are starved whatever the
                                    // economics say - the reference flight sat
                                    // at Q 0.05 with the jets at 25% of peak.

// --- Closed-cycle push ------------------------------------------------------
SET CC_FPA_HI      TO 0.            // commanded flight-path angle at the switch
                                    // (deg); 0 = derive it from the handover TWR
SET CC_FPA_TWR_A   TO 24.           // derived CC_FPA_HI = A - B * TWR: a heavy,
SET CC_FPA_TWR_B   TO 8.            // low-TWR ship burns for longer and has to
                                    // climb harder to stay out of the air; a
                                    // punchy one should stay flat and go fast
SET CC_FPA_LO      TO 0.            // ... once the apoapsis target is reached (deg)
SET CC_FPA_FLAT    TO 2.            // floor the altitude cap decays to (deg)
SET CC_FLAT_ALT    TO 45000.        // by this altitude drag is gone and there is
                                    // no reason left to climb rather than
                                    // accelerate: cap the command at CC_FPA_FLAT
SET CC_PITCH_MIN   TO -8.           // pitch clamp, rocket phase (deg)
SET CC_PITCH_MAX   TO 40.
SET CC_ACC_DT      TO 0.5.          // acceleration sampling interval (s)
SET CC_ACC_LOW     TO 1.5.          // below this accel, shallow out (m/s^2)
SET CC_BIAS_STEP   TO 0.5.          // bias change per sample (deg)
SET CC_BIAS_DOWN   TO 15.           // how far the bias may shallow the climb (deg)
SET CC_VS_ATM_MIN  TO 50.           // never stop climbing inside the atmosphere (m/s)
SET CC_TAP_MIN     TO 25.           // keep at least this long to the apex (s)
SET CC_TAP_GAIN    TO 0.35.         // deg of extra climb per second short of it
SET CC_TAP_MAX_ADD TO 12.           // ... clamped (deg)
SET CC_POLICE_DT   TO 0.5.          // dV re-pricing interval (s)
// --- angle of attack in the rocket phase ------------------------------------
//  The trim loop will spend whatever angle it takes to hold the commanded
//  flight path, and just after the handover that is all of it.  On the
//  reference flight the trim sat on its 14 deg clamp and the airframe met the
//  flow at 16.4 deg while still in 0.2 atm of dynamic pressure: 2335 kN of
//  drag, 9.5 of the 10.2 m/s^2 the engines were making, 330 m/s of rocket fuel
//  spent to buy 40 m/s of orbital energy.  Body drag on a Mk3 stack grows with
//  the sine of the angle it presents, so the angle has to be rationed while the
//  air can still charge for it.  Setting CC_AOA_HIQ to AOA_TRIM_MAX restores
//  the old behaviour.
SET CC_AOA_HIQ     TO 6.            // AoA allowed while the air still bites (deg)
SET CC_AOA_Q_HI    TO 0.10.         // ... at and above this dynamic pressure (atm)
SET CC_AOA_Q_LO    TO 0.02.         // ... opening to AOA_TRIM_MAX by this one
SET CC_DEAD_CONFIRM TO 4.           // consecutive no-thrust samples before the
                                    // rocket phase calls the engine dead
SET FEAS_ARM_DV    TO 350.          // rocket dV spent before the *whole-climb*
                                    // efficiency figure is worth quoting (m/s)
SET FEAS_CONFIRM   TO 4.            // consecutive infeasible samples to act on

// --- Measured climb efficiency ----------------------------------------------
//  The loss factor is measured over a *trailing window* of rocket dV, not
//  cumulatively from the mode switch.  The distinction is the whole point: the
//  first half-minute of a closed-cycle push happens at the bottom of the rocket
//  climb, at the lowest TWR the ship will ever have and in the last of the air,
//  and it is genuinely awful - the reference flight measured x6.37 there.  Half
//  a minute later, at 31 km with a lighter ship, the same measurement reads
//  x1.3.  A cumulative average is dominated by the bad part for the whole burn,
//  and pricing the *remaining* vacuum climb at it condemns orbits that are
//  comfortably in reach.  A window answers the question that actually matters:
//  what is a m/s buying right now?
//  The window figure is used raw, with no smoothing on top.  120 m/s of burn is
//  already a fifteen-to-twenty-second average, not a noisy instant, and a filter
//  over it would only add lag - in the wrong direction, because climb efficiency
//  improves monotonically as the air thins and the ship lightens.  A lagging
//  figure is a systematically over-priced one, which is the failure being fixed.
SET EFF_WINDOW_DV  TO 120.          // rocket dV per efficiency window (m/s)
SET EFF_MIN_WINDOWS TO 2.           // windows needed before the terminal "cannot
                                    // reach orbit" verdict may be pronounced
SET EFF_VERDICT_P  TO 0.005.        // ...and the ambient pressure it may be
                                    // pronounced below, as a fraction of the
                                    // body's sea-level pressure.  Above this the
                                    // ship is still in the drag layer, where the
                                    // measurement is real but says nothing about
                                    // the vacuum climb it would be charged to.
SET EFF_RESUME_DV  TO 25.           // dV in hand, over the bill, before a flat
                                    // burn is allowed to resume climbing (m/s)

// --- Coast / circularisation ------------------------------------------------
SET USE_WARP    TO TRUE.            // time-warp the coast to apoapsis
SET WARP_LEAD   TO 25.              // drop out of warp this long before the burn (s)
SET CIRC_PE_TOL TO 1500.            // accept periapsis this far under apoapsis (m)
SET PE_SAFETY   TO 8000.            // keep periapsis this far above the atmosphere (m)

//  The coast out of the atmosphere is not free.  MECO is called on an apoapsis
//  reading taken at 30 km at orbital speed, and the air between there and
//  ATM_TOP goes on taking it back: the 85 km flight cut at 85.3 km and reached
//  vacuum with 81.6.  Apoapsis can only be bought from below it, so this is the
//  last chance to hold on to it, and it is a cheap one - a quarter throttle
//  pulsed against the deficit, spent only while the orbit and the deorbit are
//  still fully funded without it.  FALSE flies the old profile and accepts
//  whatever apoapsis the coast delivers.
SET COAST_TRIM     TO TRUE.
SET COAST_TRIM_TOL TO 250.          // let apoapsis sag this far before trimming (m)
SET COAST_TRIM_THR TO 0.25.         // throttle used to hold it

//  The circularisation burn is flown against the velocity it is trying to
//  reach, not against a periapsis reading.  The distinction is the difference
//  between a 9 s burn and a runaway.  A finite burn does not happen at a point:
//  the ship drifts past apoapsis while it is thrusting and starts falling, and
//  every m/s of that descent is eccentricity that pure prograde thrust cannot
//  take back out.  Periapsis then saturates a few km under the target - which
//  a "burn until periapsis reaches X" loop cannot ever satisfy, so it keeps
//  thrusting, and everything after the circular point goes into *apoapsis*.
//  The 85 km flight that prompted this spent 360 m/s on a 163 m/s burn, ended
//  at 431 x 82 km, and blamed the deorbit reserve for stopping it.
//
//  So: measure what is left to do as a vector - the circular velocity here
//  minus the velocity we have - burn along it, and stop when it stops getting
//  smaller.  That figure includes the descent, so it is honest about a burn
//  that started late, and it turns over the moment the ship is faster than
//  circular, which is the only overshoot guard that cannot be argued with.
SET CIRC_DV_TOL    TO 1.5.          // remaining circularisation dV that is "done" (m/s)
SET CIRC_GIVEUP_DV TO 3.            // ... and how far it may climb back off its
                                    // own best before the burn is called over
SET CIRC_AP_TOL    TO 5000.         // backstop: apoapsis this far over the target
                                    // ends the burn whatever else is true (m)
SET CIRC_TAPER_T   TO 2.            // throttle down over the last this much burn (s)
SET CIRC_THR_MIN   TO 0.05.         // ... never below this throttle
SET CIRC_AIM_MIN   TO 4.            // below this remaining dV the aim is frozen:
                                    // it is a small difference of two big
                                    // vectors and the direction goes to noise
SET CIRC_ARM_T     TO 3.            // let the steering settle this long before the
                                    // give-up test is allowed to fire (s)

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

// The highest orbit that is still an orbit around *this* body.  Past the sphere
// of influence the ship is on an escape trajectory and none of the vis-viva
// pricing below means anything.
SET MAX_ORBIT_ALT TO (SHIP:BODY:SOIRADIUS - BODY_R) * 0.8.

// ---------------------------------------------------------------------------
//  Mission target  --  validate the requested orbit against the body
// ---------------------------------------------------------------------------
//  An apoapsis inside the atmosphere is not an orbit, it is a ballistic arc
//  that reenters on its own, and every dV figure in this script - circularise,
//  deorbit, the reserve that funds it - is meaningless for one.  So the request
//  is checked against the body actually being launched from rather than against
//  a hardcoded 70 km, and raised to the lowest real orbit if it falls short.
//
//  It is clamped rather than refused because the ship is on the runway with
//  nothing spent: flying the nearest sane mission and saying so loudly is more
//  use than a kOS program that has no clean way to stop.
SET apRequestBad TO FALSE.
IF REQUESTED_APOAPSIS < MIN_ORBIT_ALT {
  SET apRequestBad TO TRUE.
  // Accurate for both cases this catches: an apoapsis inside the air, and one
  // that clears the air but leaves no room for the periapsis safety margin the
  // circularisation targets (which would make circPeTarget exceed apoapsis).
  PRINT "!! Requested apoapsis " + ROUND(REQUESTED_APOAPSIS) +
        " m is below the lowest orbit this can fly.".
  IF ATM_TOP > 0 {
    PRINT "   " + SHIP:BODY:NAME + "'s atmosphere ends at " + ROUND(ATM_TOP / 1000, 1) +
          " km, and circularisation targets " + ROUND(PE_SAFETY / 1000, 1) +
          " km above that,".
    PRINT "   so the floor is " + ROUND(MIN_ORBIT_ALT / 1000, 1) +
          " km. Below it there is nothing to circularise into.".
  }
  IF REQUESTED_APOAPSIS > 0 AND REQUESTED_APOAPSIS < 1000 {
    PRINT "   That looks like kilometres - this parameter is in METRES, so".
    PRINT "   " + ROUND(REQUESTED_APOAPSIS) + " km would be RUN ascent(" +
          ROUND(REQUESTED_APOAPSIS * 1000) + ").".
  }
  SET REQUESTED_APOAPSIS TO MIN_ORBIT_ALT.
  PRINT "   Flying for " + ROUND(REQUESTED_APOAPSIS / 1000, 1) + " km instead.".
} ELSE IF REQUESTED_APOAPSIS > MAX_ORBIT_ALT {
  SET apRequestBad TO TRUE.
  PRINT "!! Requested apoapsis " + ROUND(REQUESTED_APOAPSIS / 1000) +
        " km is outside " + SHIP:BODY:NAME + "'s sphere of influence.".
  SET REQUESTED_APOAPSIS TO MAX_ORBIT_ALT.
  PRINT "   Flying for " + ROUND(REQUESTED_APOAPSIS / 1000, 1) + " km instead.".
}
IF apRequestBad AND PREFLIGHT_HOLD > 0 { WAIT PREFLIGHT_HOLD. }

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
FUNCTION ascIsSeparator {
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
FUNCTION ascMapFeedNetwork {
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
    IF NOT ascIsSeparator(prt) {
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

SET CORE_UIDS   TO ascMapFeedNetwork().
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
    IF ascIsSeparator(prt) { SEPARATORS:ADD(prt). }
  } ELSE {
    SET PAY_PARTS TO PAY_PARTS + 1.
    IF holds { PAY_TANKS:ADD(prt). }
  }
}

// Resource amount inside the engines' own feed network only.
FUNCTION ascCoreResAmt {
  PARAMETER rname.
  LOCAL total IS 0.
  FOR prt IN CORE_TANKS {
    FOR res IN prt:RESOURCES {
      IF res:NAME = rname { SET total TO total + res:AMOUNT. }
    }
  }
  RETURN total.
}

FUNCTION ascListResAmt {               // same, over an arbitrary part list
  PARAMETER prts, rname.
  LOCAL total IS 0.
  FOR prt IN prts {
    FOR res IN prt:RESOURCES {
      IF res:NAME = rname { SET total TO total + res:AMOUNT. }
    }
  }
  RETURN total.
}

FUNCTION ascResAmtShip {               // whole-ship total (reporting only)
  PARAMETER rname.
  LOCAL total IS 0.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = rname { SET total TO total + res:AMOUNT. }
  }
  RETURN total.
}

FUNCTION ascResDensity {               // tonnes per unit (0.005 for LF and Ox)
  PARAMETER rname.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = rname AND res:DENSITY > 0 { RETURN res:DENSITY. }
  }
  RETURN 0.005.
}

SET LF_DENS TO ascResDensity("LiquidFuel").
SET OX_DENS TO ascResDensity("Oxidizer").

// Switch crossfeed off on every separator, so the stack physically cannot pull
// propellant across the payload joint even if the budget maths were wrong.
FUNCTION ascBlockCrossfeed {
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
FUNCTION ascLockPayloadTanks {
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

FUNCTION ascMeasureRocketIsp {
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
FUNCTION ascRocketPropMass {
  LOCAL lfAmt IS ascCoreResAmt("LiquidFuel").
  LOCAL oxAmt IS ascCoreResAmt("Oxidizer").
  LOCAL lfUse IS MIN(lfAmt, oxAmt * LFO_LF_RATIO / LFO_OX_RATIO).
  LOCAL oxUse IS lfUse * LFO_OX_RATIO / LFO_LF_RATIO.
  RETURN lfUse * LF_DENS + oxUse * OX_DENS.
}

// Remaining closed-cycle dV (m/s): the rocket equation on the live ship.  The
// payload's mass counts against us; the payload's fuel does not count for us.
FUNCTION ascRocketDv {
  LOCAL m0 IS SHIP:MASS.
  LOCAL m1 IS m0 - ascRocketPropMass().
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

FUNCTION ascBurnTimeFor {              // seconds to spend dvNeed at full throttle
  PARAMETER dvNeed.
  LOCAL thr IS SHIP:AVAILABLETHRUST.
  IF thr <= 0 OR dvNeed <= 0 { RETURN 0. }
  LOCAL vEx IS RKT_ISP * G0.
  RETURN SHIP:MASS * vEx * (1 - CONSTANT:E ^ (-dvNeed / vEx)) / thr.
}

FUNCTION ascClampVal {
  PARAMETER val, lo, hi.
  RETURN MAX(lo, MIN(hi, val)).
}

// ---------------------------------------------------------------------------
//  Atmosphere  --  measured off the body, never assumed
// ---------------------------------------------------------------------------
FUNCTION atmPressureAt {            // ambient pressure at an altitude (atm)
  PARAMETER altM.
  IF altM >= ATM_TOP { RETURN 0. }
  RETURN MAX(0, SHIP:BODY:ATM:ALTITUDEPRESSURE(MAX(0, altM))).
}

// Pressure scale height here, in metres, from a finite difference on the body's
// own pressure curve.  Density and pressure share a scale height closely enough
// for guidance, and measuring it means the Q corridor works on Kerbin, Laythe,
// Eve or a rescaled system without a single retuned constant.
FUNCTION atmScaleHeight {
  LOCAL pHere IS atmPressureAt(SHIP:ALTITUDE).
  LOCAL pUp   IS atmPressureAt(SHIP:ALTITUDE + 1000).
  IF pHere <= 0 OR pUp <= 0 OR pUp >= pHere { RETURN 5000. }
  RETURN ascClampVal(1000 / LN(pHere / pUp), 1500, 20000).
}

// Closed-cycle thrust *here*: the vacuum figure de-rated by ambient pressure,
// interpolated between the two readings taken on the runway.
FUNCTION ccThrustHere {
  RETURN ccThrustVac + (ccThrustAsl - ccThrustVac) *
         ascClampVal(atmPressureAt(SHIP:ALTITUDE), 0, 1).
}

// What a tonne of liquid fuel costs the rocket phase.  While an LF-only reserve
// remains the jets are drinking fuel no oxidiser could ever pair with, so it is
// free in dV terms; once that is gone every tonne the jets take also strands
// 11/9 t of oxidiser, and the true price is 20/9 t of rocket propellant.
FUNCTION lfPriceFactor {
  IF ascCoreResAmt("LiquidFuel") >
     ascCoreResAmt("Oxidizer") * LFO_LF_RATIO / LFO_OX_RATIO { RETURN 1. }
  RETURN (LFO_LF_RATIO + LFO_OX_RATIO) / LFO_LF_RATIO.
}

// Drag (kN) read straight out of the force balance the ship is already flying:
// m*a = T - D - m*g*sin(gamma).  No drag model, no part survey - whatever is
// slowing this airframe down at this Mach and this Q is in the number.
FUNCTION dragNow {
  PARAMETER accMeasured.
  LOCAL sinF IS ascClampVal(SHIP:VERTICALSPEED / MAX(30, SHIP:VELOCITY:SURFACE:MAG), -1, 1).
  RETURN MAX(0, SHIP:AVAILABLETHRUST - SHIP:MASS * (accMeasured + localG() * sinF)).
}

// Gravity net of the centrifugal relief the ship has already earned.  At 1800
// m/s of horizontal speed this is more than half of g on Kerbin, so ignoring it
// badly understates how long the ship will keep coasting upward.
FUNCTION gEffective {
  LOCAL rr IS BODY_R + SHIP:ALTITUDE.
  LOCAL vTot IS SHIP:VELOCITY:ORBIT:MAG.
  LOCAL vv IS SHIP:VERTICALSPEED.
  LOCAL vh IS SQRT(MAX(0, vTot * vTot - vv * vv)).
  RETURN localG() - vh * vh / rr.
}

// Seconds until the ship stops climbing, if it stopped thrusting now.  Smooth
// and monotone everywhere, unlike ETA:APOAPSIS, which jumps exactly where the
// rocket phase lives - with apoapsis sitting just above the ship.
FUNCTION apexTimeEst {
  LOCAL ge IS gEffective().
  IF SHIP:VERTICALSPEED <= 0 { RETURN 0. }
  IF ge <= 0.05 { RETURN 9999. }    // already orbital: it is not coming down
  RETURN SHIP:VERTICALSPEED / ge.
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

// The velocity change that would circularise the orbit *here*, as a vector.
// Everything the circularisation burn needs to know is in it: how much speed is
// missing, and how much of the descent the ship has picked up since apoapsis has
// to be cancelled.  That second part is what a prograde-only burn can never fix
// - radial velocity is eccentricity, and burning prograde with the nose on a
// falling velocity vector adds to it rather than taking it out.
FUNCTION circBurnVec {
  LOCAL rHat IS SHIP:UP:VECTOR.               // straight up from the body centre
  LOCAL vNow IS SHIP:VELOCITY:ORBIT.
  LOCAL vHor IS VXCL(rHat, vNow).             // the part that is actually orbiting
  IF vHor:MAG < 1 { RETURN V(0, 0, 0). }
  RETURN vHor:NORMALIZED * vCircAt(SHIP:ALTITUDE) - vNow.
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

// A dV figure quoted for the *light* ship, converted into what it costs on the
// gauge here, with the cargo still aboard.
//
// Every dV number in this script is a reading on the ship as it is now.  The
// deorbit burn is not: it happens after the payload is released, on a ship that
// can be half the mass, where the same tonnes of propellant are worth far more.
// Reserving the post-release figure against the pre-release gauge sets aside
// roughly twice the propellant the burn will actually use, and on a 27% payload
// fraction that is enough to price a reachable orbit out of reach.  The
// reference flight is the proof: it reached orbit holding 54 m/s - under the
// 158 m/s reserve this script demands - released 72 t of payload, found itself
// holding 103 m/s, and deorbited on it comfortably.
FUNCTION heavyEquivalentAt {
  PARAMETER dvLight, mHeavy.
  IF NOT DEPLOY_BEFORE_DEORBIT OR payloadMass <= 1 { RETURN dvLight. }
  LOCAL vEx IS RKT_ISP * G0.
  LOCAL mLight IS mHeavy - payloadMass.
  IF mLight <= 0 OR vEx <= 0 { RETURN dvLight. }
  LOCAL prop IS mLight * (1 - CONSTANT:E ^ (-dvLight / vEx)).
  IF prop <= 0 OR mHeavy <= prop { RETURN dvLight. }
  RETURN MIN(dvLight, vEx * LN(mHeavy / (mHeavy - prop))).
}

FUNCTION ascHeavyEquivalentOf {
  PARAMETER dvLight.
  RETURN heavyEquivalentAt(dvLight, SHIP:MASS).
}

// What must still be in the tanks *after* we are circular at apAlt.
FUNCTION reserveDvFor {
  PARAMETER apAlt.
  RETURN ascHeavyEquivalentOf(deorbitDvFrom(apAlt) + DV_MARGIN).
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

// The deorbit reserve as the *handover* gauge would read it.  In flight this is
// reserveDvFor(), which prices the post-release burn at the mass the ship has
// right now; on the runway there is no such reading to take, so the mass at the
// end of the climb is estimated from the handover mass and the dV about to be
// spent getting there.  Skipping that step is not a rounding error: it charges
// the pre-flight check the full post-release figure against a ship still
// carrying every tonne of cargo and propellant, which on the reference airframe
// is 146 m/s of reserve for a burn worth 80.  That is most of the gap the check
// then reports, and it is the reason a flight that reached orbit with fuel to
// spare was told on the runway that it could not.
FUNCTION reserveFromHandover {
  PARAMETER apAlt, dvToOrbit.
  LOCAL vEx IS RKT_ISP * G0.
  LOCAL mOrb IS handoverMass(0, 0, 0) * CONSTANT:E ^ (-MAX(0, dvToOrbit) / vEx).
  RETURN heavyEquivalentAt(deorbitDvFrom(apAlt) + DV_MARGIN, mOrb).
}

// The same price as priceOfOrbit(), but for an *assumed* handover state rather
// than the live one.  The live-state version above is what the in-flight
// policing needs; this is what the pre-flight and post-flight sizing advice
// needs, because after the flight the live state is orbit and dvToRaiseApTo()
// would read zero.
FUNCTION priceFromHandover {
  PARAMETER apAlt, altM, spd, lossF.
  LOCAL dvClimb IS dvRaiseApFrom(altM, spd, apAlt) * lossF.
  LOCAL dvCirc  IS circDvFrom(altM, apAlt).
  RETURN dvClimb + dvCirc + reserveFromHandover(apAlt, dvClimb + dvCirc).
}

FUNCTION affordableApFrom {
  PARAMETER dvHave, altM, spd, lossF.
  IF priceFromHandover(MIN_ORBIT_ALT, altM, spd, lossF) > dvHave { RETURN 0. }
  IF priceFromHandover(REQUESTED_APOAPSIS, altM, spd, lossF) <= dvHave {
    RETURN REQUESTED_APOAPSIS.
  }
  LOCAL lo IS MIN_ORBIT_ALT.
  LOCAL hi IS REQUESTED_APOAPSIS.
  FROM { LOCAL iter IS 0. } UNTIL iter >= 16 STEP { SET iter TO iter + 1. } DO {
    LOCAL mid IS (lo + hi) / 2.
    IF priceFromHandover(mid, altM, spd, lossF) <= dvHave { SET lo TO mid. }
    ELSE { SET hi TO mid. }
  }
  RETURN lo.
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

// May the rocket phase pronounce the mission dead?  Re-targeting downward is
// cheap and reversible, so it is always allowed; declaring that no orbit at all
// is reachable is neither, and it needs evidence that is worth something.
//
// Two conditions, and the reference flight failed both.  First, enough windows
// to know the measurement is not a transient.  Second - the one that matters -
// the ship must be clear of the drag layer.  A loss factor measured at 29 km is
// a true statement about 29 km and a worthless one about the 50 km of vacuum
// climb it would be charged to: the reference flight measured x6.37 down there,
// priced a 78 km apoapsis at 4869 m/s of climb against 1121 m/s in the tanks,
// and levelled off - while flying at 9.4 m/s^2 out of the 10.3 its engines
// could produce, which is a loss factor of x1.1.  The orbit it had just written
// off was affordable.  So the verdict waits until the air is thin enough that
// what is being measured is what is about to be flown.
//
// The exception is a ship that is spending dV and buying nothing at all, window
// after window: sustained, that is not a regime artefact at any altitude, and
// there is nothing to be learned by climbing further to confirm it.  It has to
// be sustained, though.  The single window straight after the handover reads the
// ceiling on almost every spaceplane - the jets have just cut, the rocket is
// still spooling, and the ship is briefly *losing* speed - and treating that one
// sample as proof would reintroduce the very abort this gate exists to prevent.
FUNCTION verdictAllowed {
  IF effWindows < EFF_MIN_WINDOWS { RETURN FALSE. }
  IF deadWindows >= EFF_MIN_WINDOWS { RETURN TRUE. }
  RETURN atmPressureAt(SHIP:ALTITUDE) <= EFF_VERDICT_P * atmPressureAt(0).
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
  RETURN 1 - CONSTANT:E ^ (-planJetDv / (jetIsp * G0)).
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

// Rocket dV at the handover if the payload's tanks were counted as ours.  This
// is not an upgrade, so it does not go through dvHandoverSplit: the propellant
// and the tank holding it are already aboard and already being lifted, and
// nothing is being added to the ship.  Only the pool the engines are allowed to
// drink from changes.
//
// It is worth printing because it is the single largest number in the whole
// budget, and because it is the difference between two flights of this airframe
// that read as contradictions.  The PR #2 autopilot had no payload isolation and
// reached 104 x 99 km; roughly a third of everything it burned came out of the
// cargo tanks.  Whether that is a success or a theft is a question about the
// mission, not about the ascent, so the script states the number and leaves the
// decision to ISOLATE_PAYLOAD.
FUNCTION dvHandoverWithPayloadFuel {
  IF PAY_TANKS:LENGTH = 0 { RETURN 0. }
  LOCAL mH IS handoverMass(0, 0, 0).
  LOCAL lfLeft IS lfMass0 + ascListResAmt(PAY_TANKS, "LiquidFuel") * LF_DENS -
                  jetBurnWith(0, 0, 0).
  LOCAL oxLeft IS oxMass0 + ascListResAmt(PAY_TANKS, "Oxidizer") * OX_DENS.
  LOCAL lfUse IS MIN(lfLeft, oxLeft * LFO_LF_RATIO / LFO_OX_RATIO).
  IF lfUse <= 0 { RETURN 0. }
  LOCAL prop IS lfUse * (LFO_LF_RATIO + LFO_OX_RATIO) / LFO_LF_RATIO.
  IF mH <= prop { RETURN 0. }
  RETURN RKT_ISP * G0 * LN(mH / (mH - prop)).
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
SET aoaCap  TO AOA_TRIM_MAX.        // rationed on Q once the rocket phase starts

FUNCTION speedNow {
  IF SHIP:ALTITUDE > ATM_TOP { RETURN MAX(30, SHIP:VELOCITY:ORBIT:MAG). }
  RETURN MAX(30, SHIP:VELOCITY:SURFACE:MAG).
}

FUNCTION vsToFpa {                  // a vertical speed, as a flight-path angle
  PARAMETER vsWanted.
  RETURN ARCSIN(ascClampVal(vsWanted / speedNow(), -0.95, 0.95)).
}

FUNCTION ascFpaNow {
  RETURN ARCSIN(ascClampVal(SHIP:VERTICALSPEED / speedNow(), -0.95, 0.95)).
}

// How much angle of attack the flow may be charged for right now.  In the jet
// phase this is the full trim authority and never binds - an air-breathing climb
// holds its corridor at a few degrees.  In the rocket phase it is scheduled on
// dynamic pressure, because that is what turns angle into drag.
FUNCTION aoaLimitNow {
  IF SHIP:Q >= CC_AOA_Q_HI { RETURN CC_AOA_HIQ. }
  IF SHIP:Q <= CC_AOA_Q_LO { RETURN AOA_TRIM_MAX. }
  RETURN CC_AOA_HIQ + (AOA_TRIM_MAX - CC_AOA_HIQ) *
         (CC_AOA_Q_HI - SHIP:Q) / (CC_AOA_Q_HI - CC_AOA_Q_LO).
}

FUNCTION steerFpa {
  PARAMETER fpaWanted, pitchLo, pitchHi, dtStep.
  LOCAL err IS fpaWanted - ascFpaNow().
  LOCAL raw IS fpaWanted + aoaTrim.
  IF (raw > pitchLo AND raw < pitchHi) OR
     (raw <= pitchLo AND err > 0) OR
     (raw >= pitchHi AND err < 0) {
    SET aoaTrim TO ascClampVal(aoaTrim + AOA_KI * err * dtStep, -AOA_TRIM_MAX, AOA_TRIM_MAX).
  }
  // Limit the angle the airframe actually presents to the flow, not the trim.
  // They are not the same number: while the trim is winding up the flight path
  // lags the command, and the real angle of attack is the trim *plus* that lag.
  // The reference flight sat on a 14 deg trim clamp and flew at 16.4 deg.
  LOCAL cmd IS ascClampVal(fpaWanted + aoaTrim,
                        ascFpaNow() - aoaCap, ascFpaNow() + aoaCap).
  RETURN ascClampVal(cmd, pitchLo, pitchHi).
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
  LOCAL resNote IS "".
  IF DEPLOY_BEFORE_DEORBIT AND payloadMass > 1 {
    SET resNote TO "*".             // priced after the payload comes off
  }
  RETURN "dV " + ROUND(ascRocketDv()) + " vs need " + ROUND(priceOfOrbit(apAlt, lossFactor)) +
         " (climb " + ROUND(dvToRaiseApTo(apAlt) * lossFactor) +
         " + circ " + ROUND(circDvAt(apAlt)) +
         " + reserve " + ROUND(reserveDvFor(apAlt)) + resNote +
         ") @ loss x" + ROUND(lossFactor, 2).
}

// ---------------------------------------------------------------------------
//  0. PRE-FLIGHT  --  measure the ship we are actually flying
// ---------------------------------------------------------------------------
PRINT "=== RAPIER SSTO :: ascent autopilot ===".
PRINT "Target: " + ROUND(REQUESTED_APOAPSIS / 1000, 1) + " km circular over " +
      SHIP:BODY:NAME + ", heading " + ROUND(LAUNCH_HEADING) + ".".
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
        ROUND(ascListResAmt(PAY_TANKS, "LiquidFuel")) + " / Ox " +
        ROUND(ascListResAmt(PAY_TANKS, "Oxidizer")) + " - NOT counted as dV.".
}
IF ISOLATE_PAYLOAD {
  SET xfBlocked TO ascBlockCrossfeed().
  PRINT "  Crossfeed disabled on " + xfBlocked + " separator module(s).".
  // The separator blocking crossfeed is the clean isolation and it leaves the
  // payload's own plumbing untouched.  Only if that is unavailable do we fall
  // back to locking the payload's tanks - which the user must undo on deploy.
  IF PAY_TANKS:LENGTH > 0 AND (xfBlocked = 0 OR FORCE_TANK_LOCK) {
    SET tkLocked TO ascLockPayloadTanks().
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
SET RKT_ISP     TO ascMeasureRocketIsp().
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
SET lfMass0    TO ascCoreResAmt("LiquidFuel") * LF_DENS.
SET oxMass0    TO ascCoreResAmt("Oxidizer") * OX_DENS.

// The jet phase does not cost "handover speed" - it costs handover speed plus
// everything drag took on the way, and how much that is depends almost entirely
// on how long the ship spends in the air.  A 0.5-TWR airframe accelerates at a
// crawl and pays drag for every second of it; a 0.8-TWR one is through the same
// band in half the time.  Scaling the multiplier on take-off TWR is crude, but
// it is far closer than a fixed 3000 m/s: on the reference flight the real
// figure was 3637 m/s at TWR 0.50, which the default below reproduces.
SET planJetDv TO PLAN_JET_DV.
IF planJetDv <= 0 {
  SET planJetDv TO PLAN_SWITCH_SPD * PLAN_JET_DV_K * ascClampVal(0.5 / MAX(0.15, jetTwr), 0.7, 1.4).
}
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
SET planClimbDv TO planClimb * PLAN_LOSS_FACTOR.
SET planReserve TO reserveFromHandover(REQUESTED_APOAPSIS, planClimbDv + planCirc).
SET dvRequired TO planClimbDv + planCirc + planReserve.
SET dvAtPad    TO ascRocketDv().                         // every paired unit aboard
SET dvHandover TO dvAtHandover(0, 0).                 // what survives the jets
SET jetLfPlan  TO jetBurnWith(0, 0, 0).
SET lfSpare    TO lfMass0 - oxMass0 * LFO_LF_RATIO / LFO_OX_RATIO.

PRINT "FEASIBILITY for a " + ROUND(REQUESTED_APOAPSIS / 1000) + " km orbit".
PRINT "  Assumes handover at " + ROUND(PLAN_SWITCH_ALT / 1000, 1) + " km / " +
      ROUND(PLAN_SWITCH_SPD) + " m/s after ~" + ROUND(planJetDv) + " m/s of jet dV,".
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
SET planResNote TO "".
IF DEPLOY_BEFORE_DEORBIT AND payloadMass > 1 { SET planResNote TO "*". }
PRINT "  NEED  climb " + ROUND(planClimbDv) + " + circ " + ROUND(planCirc) +
      " + reserve " + ROUND(planReserve) + planResNote + "  =  " +
      ROUND(dvRequired) + " m/s".
IF planResNote <> "" {
  PRINT "        (reserve* = deorbit " + ROUND(planDeorb) + " + margin " +
        ROUND(DV_MARGIN) + ", flown after " + ROUND(payloadMass, 1) +
        " t of payload comes off,".
  PRINT "         priced against the gauge this ship will be reading in orbit.)".
}
// Note the handover figure can legitimately come out *above* the pad figure:
// burning an LF-only reserve lightens the ship without touching a single
// paired unit, which is exactly what that reserve is for.
PRINT "  HAVE  " + ROUND(dvHandover) + " m/s at the handover (tanks hold " +
      ROUND(dvAtPad) + " m/s of pairs on the pad)".
PRINT "  TWR   " + ROUND(twrAtHandover(0, 0, 0), 2) + " closed cycle at the handover".

IF dvHandover >= dvRequired {
  // "In hand" has to mean more than the model's own error bars.  A handful of
  // m/s on a 1500 m/s bill is not a margin, it is a rounding difference between
  // two estimates - and reporting it as GO is how a ship gets to 24 km before
  // anyone finds out.  Anything under PLAN_GO_MARGIN of the bill is MARGINAL.
  SET dvSpare TO dvHandover - dvRequired.
  IF dvSpare >= dvRequired * PLAN_GO_MARGIN {
    PRINT "  => GO. " + ROUND(dvSpare) + " m/s in hand (" +
          ROUND(100 * dvSpare / MAX(1, dvRequired)) + "%).".
  } ELSE {
    PRINT "  => MARGINAL. Only " + ROUND(dvSpare) + " m/s in hand (" +
          ROUND(100 * dvSpare / MAX(1, dvRequired)) + "% of the bill).".
    PRINT "     This model is not accurate to better than ~" +
          ROUND(100 * PLAN_GO_MARGIN) + "%, so treat that as a coin toss:".
    PRINT "     a handover " + ROUND(0.1 * PLAN_SWITCH_SPD) +
          " m/s slow, or drag " + ROUND(10) + "% worse than assumed, spends it.".
    PRINT "     Add LF, cut payload, or lower REQUESTED_APOAPSIS before you fly.".
  }
  IF twrAtHandover(0, 0, 0) < PLAN_TWR_MIN {
    PRINT "  !! ...and closed-cycle TWR is below " + ROUND(PLAN_TWR_MIN, 2) +
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

  // --- or stop treating the payload's propellant as cargo --------------------
  //  Almost always the biggest single number on this list, because the payload
  //  tanks on a tanker are most of the propellant aboard.  It is offered last
  //  and never taken automatically: the script cannot know whether that fuel is
  //  meant to arrive in orbit or is simply fuel.
  IF PAY_TANKS:LENGTH > 0 AND ISOLATE_PAYLOAD {
    LOCAL dvUnlocked IS dvHandoverWithPayloadFuel().
    IF dvUnlocked > dvHandover + 1 {
      PRINT "   * PAYLOAD FUEL: the cargo tanks hold " +
            ROUND(ascListResAmt(PAY_TANKS, "LiquidFuel")) + " LF / " +
            ROUND(ascListResAmt(PAY_TANKS, "Oxidizer")) + " Ox, locked out as cargo.".
      PRINT "     Counting them would put " + ROUND(dvUnlocked) +
            " m/s at the handover instead of " + ROUND(dvHandover) + ".".
      PRINT "     SET ISOLATE_PAYLOAD TO FALSE. only if that fuel is propellant".
      PRINT "     and not deliverable cargo - the engines will drink it.".
    }
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
//  2. AIR-BREATHING CLIMB  --  ride a dynamic-pressure corridor
// ---------------------------------------------------------------------------
//  One commanded quantity: vertical speed, and one thing it is trying to do -
//  hold dynamic pressure at AB_Q_TARGET while the ship accelerates.  Q is the
//  air the intakes are being fed, so a Q corridor is the same thing as "stay
//  where the jets work", and it is the altitude schedule a good spaceplane
//  ascent has anyway: steep while slow and low, then flattening out into a long
//  hypersonic acceleration that drifts up as speed builds.
//
//  The command is derived rather than searched for.  Q = 0.5*rho*v^2 and
//  rho ~ exp(-h/H), so
//        d(ln Q)/dt  =  2a/v  -  vs/H
//  and the vertical speed that holds Q exactly is vs = 2*H*a/v.  Add a
//  proportional term on the standing error and that is the whole controller:
//        vs = H * ( 2a/v + Kp * ln(Q / Qtarget) )
//  H is measured off the body's own pressure curve each sample, so nothing here
//  is tuned to Kerbin.  Note what it does with spare thrust: high a *does* buy
//  climb, but only the climb needed to keep Q from rising - not all of it.
//
//  Guards outrank the corridor: climb out of air too thick to push through,
//  never mush into the ground, and never descend once high and fast.  The pitch
//  that delivers the commanded climb comes from the shared flight-path
//  controller, so the nose moves smoothly instead of hunting.
// ---------------------------------------------------------------------------
PRINT "Air-breathing climb: Q corridor from " + ROUND(AB_Q_TARGET, 2) +
      " atm, climb capped at " + ROUND(AB_FPA_MAX) + " deg.".
// Start from the climb the ship is already flying, not from a constant: at
// handover speed a fixed command is a 40 deg flight path, and the nose would
// slam to the clamp before the trim had learned anything.
SET vsCmd     TO ascClampVal(SHIP:VERTICALSPEED, 30, AB_VS_MAX).
SET qTgt      TO AB_Q_TARGET.
SET aoaTrim   TO 0.
SET lastLoopT TO TIME:SECONDS.
SET lastTuneT TO TIME:SECONDS.
SET lastQTrimT TO TIME:SECONDS.
SET lastSpd   TO SHIP:AIRSPEED.
SET accelNow  TO 0.
SET accelSm   TO 0.
SET peakJetT  TO SHIP:AVAILABLETHRUST.
SET costJet   TO 0.
SET costRkt   TO 1.
SET swVotes   TO 0.
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
    // The corridor's feed-forward term is proportional to acceleration, and a
    // one-second finite difference is noisy, so it is filtered before it steers
    // anything.  The switch economics read the same filtered number.
    SET accelSm  TO accelSm + AB_ACC_SMOOTH * (accelNow - accelSm).
    SET peakJetT TO MAX(peakJetT, SHIP:AVAILABLETHRUST).

    // ---- the Q corridor -----------------------------------------------------
    LOCAL scaleH IS atmScaleHeight().
    LOCAL qNow   IS MAX(0.0005, SHIP:Q).

    // Outer loop: trim the target itself against sustained acceleration, so the
    // corridor ends up where this airframe's drag and intakes actually balance
    // rather than where a constant guessed they would.
    IF AB_Q_ADAPT AND nowT - lastQTrimT >= AB_Q_ADAPT_DT {
      SET lastQTrimT TO nowT.
      // The dead band matters: without one the target steps every interval
      // whatever the ship is doing, and settles into a permanent +-6% hunt
      // around the right answer instead of stopping on it.
      IF ABS(LN(qNow / qTgt)) < AB_Q_ADAPT_TOL {
        IF accelSm > AB_ACC_TARGET * (1 + AB_Q_ADAPT_BAND) {
          SET qTgt TO qTgt * (1 - AB_Q_ADAPT_STEP).   // room to spare: go thinner
        } ELSE IF accelSm < AB_ACC_TARGET * (1 - AB_Q_ADAPT_BAND) {
          SET qTgt TO qTgt * (1 + AB_Q_ADAPT_STEP).   // starving: come back down
        }
        SET qTgt TO ascClampVal(qTgt, AB_Q_TGT_MIN, AB_Q_TGT_MAX).
      }
    }

    LOCAL vsHold IS 2 * scaleH * accelSm / MAX(50, SHIP:AIRSPEED).
    LOCAL vsWant IS vsHold + scaleH * AB_Q_KP * LN(qNow / qTgt).
    // Keep the command inside what the airframe can actually fly.  A vs command
    // the pitch clamp cannot deliver is not guidance, it is integral windup.
    LOCAL vsFlyable IS SHIP:AIRSPEED * SIN(AB_FPA_MAX).
    SET vsWant TO ascClampVal(vsWant, -vsFlyable, vsFlyable).
    // Move toward it at a bounded rate so the nose is never asked to jump.
    SET vsCmd TO vsCmd + ascClampVal(vsWant - vsCmd, -AB_VS_RATE * tuneDt, AB_VS_RATE * tuneDt).

    // Guards, in priority order over the corridor.
    IF SHIP:Q > AB_Q_MAX { SET vsCmd TO MAX(vsCmd, AB_VS_QCLIMB). }
    IF SHIP:ALTITUDE < AB_FLOOR_ALT { SET vsCmd TO MAX(vsCmd, AB_VS_FLOOR). }
    // Descending is a low-and-transonic move only.  Thin air high up is not a
    // reason to dive back into thick air at Mach 4 - it is the end of the
    // air-breathing envelope, and the economics below will say so.
    LOCAL vsFloor IS 0.
    IF SHIP:ALTITUDE < AB_DIVE_ALT_MAX AND SHIP:AIRSPEED > AB_DIVE_SPEED {
      SET vsFloor TO AB_VS_DIVE.
    }
    SET vsCmd TO ascClampVal(vsCmd, vsFloor, AB_VS_MAX).

    // ---- book-keeping ------------------------------------------------------
    //  The jets run on liquid fuel, and the ship carries an LF-only reserve for
    //  exactly that.  Once LF drops to the level that pairs with the remaining
    //  oxidiser, every further second of jet burn is spent out of the rocket dV
    //  budget - worth knowing, though not worth switching for (see below).
    IF (NOT lfWarned) AND
       ascCoreResAmt("LiquidFuel") <= ascCoreResAmt("Oxidizer") * LFO_LF_RATIO / LFO_OX_RATIO {
      PRINT "  Note: LF-only reserve used up; jet burn now draws on rocket dV (" +
            ROUND(ascRocketDv()) + " m/s left).".
      SET lfWarned TO TRUE.
    }

    // ---- price both modes, and fly the cheaper one --------------------------
    //  The question is never "are the jets finished?" but "is a m/s of speed
    //  still cheaper out of the jets than out of the rocket?", and both sides
    //  of that are measurable right now.  Drag comes out of the force balance
    //  the ship is already flying, so the closed-cycle estimate is the same
    //  airframe at the same Mach and Q - only the engine changes.
    LOCAL dragKn IS dragNow(accelSm).
    LOCAL thrRkt IS ccThrustHere().
    LOCAL sinFpa IS ascClampVal(SHIP:VERTICALSPEED / MAX(50, SHIP:AIRSPEED), -1, 1).
    LOCAL accRkt IS (thrRkt - dragKn) / SHIP:MASS - localG() * sinFpa.
    // Flow in tonnes/s, priced in rocket-equivalent propellant: jet fuel is
    // free while an LF-only reserve remains and costs 20/9 once it is gone.
    LOCAL flowJet IS SHIP:AVAILABLETHRUST / (JET_ISP * G0) * lfPriceFactor().
    LOCAL flowRkt IS thrRkt / (RKT_ISP * G0).
    SET costJet TO flowJet / MAX(0.01, accelSm).
    SET costRkt TO flowRkt / MAX(0.01, accRkt).

    LOCAL armed  IS SHIP:ALTITUDE > SW_ARM_ALT AND
                    SHIP:AVAILABLETHRUST < peakJetT * SW_ARM_DECAY.
    LOCAL priced IS accRkt > 0 AND
                    (accelSm < SW_MIN_ACCEL OR costJet > costRkt * SW_COST_RATIO).

    IF anyFlameout() OR SHIP:AVAILABLETHRUST < 1 {
      SET switchNow TO TRUE.
      SET swReason TO "flameout - out of intake air".
    } ELSE IF armed AND priced {
      SET swVotes TO swVotes + 1.
      IF swVotes >= SW_CONFIRM {
        SET switchNow TO TRUE.
        IF accelSm < SW_MIN_ACCEL {
          SET swReason TO "jets out of acceleration (" + ROUND(accelSm, 2) + " m/s^2)".
        } ELSE {
          SET swReason TO "rocket is now cheaper - " + ROUND(1000 * costJet, 1) +
                          " vs " + ROUND(1000 * costRkt, 1) + " kg of propellant per m/s".
        }
      }
    } ELSE {
      SET swVotes TO 0.
    }

    IF SHIP:ALTITUDE > SW_ALT_HARD OR SHIP:AIRSPEED > SW_SPEED_HARD {
      SET switchNow TO TRUE.
      SET swReason TO "hard backstop (altitude/speed limit)".
    } ELSE IF SHIP:Q < SW_Q_HARD AND SHIP:ALTITUDE > SW_ARM_ALT {
      SET switchNow TO TRUE.
      SET swReason TO "intakes starved - Q down to " + ROUND(SHIP:Q, 3) + " atm".
    }
  }

  // ---- attitude ------------------------------------------------------------
  SET pitchCmd TO steerFpa(vsToFpa(vsCmd), AB_PITCH_MIN, AB_PITCH_MAX, dtStep).

  IF nowT - abReportT > 15 {
    PRINT "  alt " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km | " + ROUND(SHIP:AIRSPEED) +
          " m/s | acc " + ROUND(accelSm, 2) + " | Q " + ROUND(SHIP:Q, 2) + "/" +
          ROUND(qTgt, 2).
    PRINT "      vs " + ROUND(SHIP:VERTICALSPEED) + "/" + ROUND(vsCmd) +
          " | pitch " + ROUND(pitchCmd, 1) + " | thr " +
          ROUND(100 * SHIP:AVAILABLETHRUST / MAX(1, peakJetT)) + "% | dV " + ROUND(ascRocketDv()).
    PRINT "      cost/m/s: jet " + ROUND(1000 * costJet, 1) + " vs rocket " +
          ROUND(1000 * costRkt, 1) + " kg" .
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
// PLAN_SWITCH_ALT / PLAN_SWITCH_SPD / PLAN_JET_DV_K to these numbers and the
// next pre-flight check will be about this airframe rather than a generic one.
SET jetLfUsed TO launchMass - SHIP:MASS.
SET jetDvReal TO JET_ISP * G0 * LN(launchMass / MAX(0.001, SHIP:MASS)).
PRINT "  Jet phase used " + ROUND(jetLfUsed, 1) + " t of LF = " +
      ROUND(jetDvReal) + " m/s of jet dV (planned " + ROUND(planJetDv) + ").".
PRINT "  Handover " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km / " + ROUND(SHIP:AIRSPEED) +
      " m/s (planned " + ROUND(PLAN_SWITCH_ALT / 1000, 1) + " / " +
      ROUND(PLAN_SWITCH_SPD) + ").".

setRapierMode(TRUE).
WAIT 0.5.                           // let the engines settle in mode
SET RKT_ISP TO ascMeasureRocketIsp().  // now the live engines quote rocket Isp

SET targetAp    TO REQUESTED_APOAPSIS.
SET lossFactor  TO CLIMB_LOSS_FACTOR.   // replaced by the measured value below
SET lossMeasured TO FALSE.              // ...if the burn lasts long enough to arm it
SET apAtSwitch  TO SHIP:APOAPSIS.
SET dvAtSwitch  TO ascRocketDv().
// The efficiency yardstick is pinned to a fixed reference altitude, not to the
// live target.  If it tracked the target, re-targeting downward would itself
// look like progress, the measured loss factor would jump, the orbit would look
// affordable again, and the decision would oscillate.
SET EFF_REF_AP  TO REQUESTED_APOAPSIS.
SET price0      TO dvToRaiseApTo(EFF_REF_AP).
// Two efficiency figures, because two different questions are being asked.
// lossFactor is the trailing-window one and prices the climb that is *left* -
// it is what every in-flight decision is made on.  lossWhole is cumulative from
// the switch and describes the climb as a whole, which is the number the next
// pre-flight check should be planned with; quoting the window figure there
// would tell the next flight the ascent is cheaper than it is.
SET lossWhole   TO CLIMB_LOSS_FACTOR.
SET wholeMeasured TO FALSE.
SET effMarkDv    TO dvAtSwitch.     // dV reading at the head of the live window
SET effMarkPrice TO price0.         // ...and the reference price there
SET effWindows   TO 0.              // completed windows so far
SET deadWindows  TO 0.              // ...of which consecutive ones bought nothing

PRINT "Closed cycle. " + budgetLine(targetAp, lossFactor).
IF DEPLOY_BEFORE_DEORBIT AND payloadMass > 1 {
  PRINT "  (reserve* = deorbit " + ROUND(deorbitDvFrom(targetAp)) + " + margin " +
        ROUND(DV_MARGIN) + " flown after " + ROUND(payloadMass, 1) +
        " t of payload comes off.)".
}
PRINT "  Mass " + ROUND(SHIP:MASS, 1) + " t, thrust " + ROUND(SHIP:AVAILABLETHRUST) +
      " kN, TWR " + ROUND(twrNow(), 2) + ", Isp " + ROUND(RKT_ISP) + " s.".

// How steeply to leave the handover.  A ship with thrust to spare is through
// the drag layer in seconds and should spend its nose on speed; a ship at TWR
// 1.1 burns for a minute and a half and has to climb or it will sink back into
// the air before apoapsis.  Deriving it beats a constant that is right for one
// airframe.
SET ccFpaHi TO CC_FPA_HI.
IF ccFpaHi <= 0 {
  SET ccFpaHi TO ascClampVal(CC_FPA_TWR_A - CC_FPA_TWR_B * twrNow(), 5, 20).
}
// The handover state, kept for the post-flight sizing advice: this is the point
// the whole mission is priced from, and now it is measured rather than assumed.
SET swAlt      TO SHIP:ALTITUDE.
SET swSpeed    TO SHIP:AIRSPEED.
SET twrAtSwitch TO twrNow().
PRINT "  Initial climb command " + ROUND(ccFpaHi, 1) + " deg, flattening to " +
      ROUND(CC_FPA_FLAT, 1) + " deg by " + ROUND(CC_FLAT_ALT / 1000) + " km.".

SET aoaTrim   TO 0.                 // fresh trim: different regime, different AoA
SET aoaCap    TO aoaLimitNow().
SET fpaBias   TO 0.
SET emergency TO FALSE.             // orbit priced out: fly for periapsis instead
SET lastLoopT TO TIME:SECONDS.
SET lastAccT  TO TIME:SECONDS.
SET lastPolT  TO TIME:SECONDS.
SET lastSpd   TO SHIP:VELOCITY:ORBIT:MAG.
SET accelNow  TO 0.
SET reportT   TO TIME:SECONDS.
SET settled   TO FALSE.
SET infeasVotes TO 0.
SET infeasWarned TO FALSE.
SET infeasNoted TO FALSE.
SET deadVotes TO 0.
SET ccDone    TO FALSE.
SET stopReason TO "apoapsis reached".

UNTIL ccDone {
  LOCAL nowT IS TIME:SECONDS.
  LOCAL dtStep IS MAX(0.02, nowT - lastLoopT).
  SET lastLoopT TO nowT.

  IF SHIP:APOAPSIS >= targetAp { SET ccDone TO TRUE. BREAK. }

  // ---- acceleration sampling: a one-way bias, shallower only ---------------
  IF nowT - lastAccT >= CC_ACC_DT {
    LOCAL accDt IS nowT - lastAccT.
    SET lastAccT TO nowT.
    SET accelNow TO (SHIP:VELOCITY:ORBIT:MAG - lastSpd) / accDt.
    SET lastSpd  TO SHIP:VELOCITY:ORBIT:MAG.
    // The bias only ever shallows.  Spare acceleration is never a reason to
    // point the nose further from the horizon: above the drag layer a steeper
    // climb costs cos(gamma) of the thrust and g*sin(gamma) of gravity loss to
    // buy altitude the engine could buy more cheaply as orbital speed.
    IF accelNow < CC_ACC_LOW {
      SET fpaBias TO MAX(fpaBias - CC_BIAS_STEP, -CC_BIAS_DOWN).
    } ELSE {
      SET fpaBias TO MIN(0, fpaBias + CC_BIAS_STEP).
    }
  }

  // ---- guidance: a flight-path schedule that flattens as apoapsis arrives --
  //  Progress, not time-to-apoapsis.  Apoapsis is a smooth, monotone measure of
  //  how much of the job is done; ETA:APOAPSIS is discontinuous exactly here,
  //  where apoapsis sits just above the ship, and steering on it oscillates.
  LOCAL prog IS 1.
  IF targetAp > apAtSwitch {
    SET prog TO ascClampVal((SHIP:APOAPSIS - apAtSwitch) / (targetAp - apAtSwitch), 0, 1).
  }
  LOCAL fpaWant IS ccFpaHi + (CC_FPA_LO - ccFpaHi) * prog + fpaBias.

  //  A second, independent cap on altitude.  The *only* reason to climb under
  //  power is to get out of air that is charging for the speed we are buying;
  //  once that air is gone the climb has no payer left, so the command is
  //  capped by a schedule that decays from the handover altitude to CC_FPA_FLAT
  //  by CC_FLAT_ALT.  Whichever of the two schedules is shallower wins.
  LOCAL altFrac IS ascClampVal((CC_FLAT_ALT - SHIP:ALTITUDE) /
                            MAX(1, CC_FLAT_ALT - swAlt), 0, 1).
  SET fpaWant TO MIN(fpaWant, CC_FPA_FLAT + (ccFpaHi - CC_FPA_FLAT) * altFrac).

  //  Emergency: the orbit has been priced out of reach.  Raising apoapsis now
  //  actively hurts - it makes the circularisation we already cannot afford
  //  more expensive - so the objective changes to the highest periapsis the
  //  remaining fuel can buy, and that is bought by burning flat.
  IF emergency { SET fpaWant TO MIN(fpaWant, CC_FPA_FLAT). }

  IF SHIP:ALTITUDE < ATM_TOP {
    // Whatever the caps say, we are leaving the atmosphere.
    SET fpaWant TO MAX(fpaWant, vsToFpa(CC_VS_ATM_MIN)).
  }

  //  Floor: never let the apex arrive before the burn does.  A ship that tops
  //  out and starts falling mid-burn pays for the same altitude twice.  This is
  //  the one place a time is used, and it is the smooth vs/g_eff estimate, not
  //  ETA:APOAPSIS - and g_eff is gravity net of centrifugal relief, which at
  //  1800 m/s is over half of it.
  //
  //  It is skipped in the emergency flat burn, where it is exactly backwards.
  //  Once apoapsis has stopped being the objective, arriving at the apex is not
  //  a failure - it is the plan, and flying through it flat is how periapsis
  //  gets bought.  Applied there it also runs away: the flatter the ship flies
  //  the sooner the apex arrives, so the floor demands more climb, which is the
  //  one thing the emergency exists to stop.  On the reference flight it drove
  //  the commanded angle from the 2 deg the emergency had just set to 10.5 deg,
  //  and the pitch command to +24 deg, in a burn whose stated objective was to
  //  stop buying altitude.
  IF NOT emergency {
    LOCAL tapShort IS CC_TAP_MIN - apexTimeEst().
    IF tapShort > 0 {
      SET fpaWant TO fpaWant + ascClampVal(CC_TAP_GAIN * tapShort, 0, CC_TAP_MAX_ADD).
    }
  }

  SET aoaCap TO aoaLimitNow().
  SET pitchCmd TO steerFpa(fpaWant, CC_PITCH_MIN, CC_PITCH_MAX, dtStep).

  // ---- dV policing --------------------------------------------------------
  IF nowT - lastPolT >= CC_POLICE_DT {
    SET lastPolT TO nowT.
    LOCAL dvNow IS ascRocketDv().

    // Floor 0: the engine has stopped.  ascRocketDv() prices the propellant in our
    // tanks; it cannot tell whether the engines can still reach it.  When they
    // cannot, every exit below is unreachable - apoapsis stops rising, so the
    // target is never met, and the dV reading freezes, so the glide reserve is
    // never crossed - and this loop spins at full throttle while the ship coasts
    // over the top and falls back into the air, with the ascent autopilot still
    // holding the controls and the reentry script never run.  The jet phase has
    // guarded exactly this since it was written; the rocket phase had no guard
    // at all.  Total loss only: a single flamed-out RAPIER out of fourteen is
    // not a reason to end a climb the other thirteen are still flying.
    IF SHIP:AVAILABLETHRUST < 1 {
      SET deadVotes TO deadVotes + 1.
      IF deadVotes >= CC_DEAD_CONFIRM {
        SET settled TO TRUE.
        SET stopReason TO "thrust lost with " + ROUND(dvNow) + " m/s still on the gauge".
        SET targetAp TO SHIP:APOAPSIS.
        SET ccDone TO TRUE.
        PRINT "======================================================".
        PRINT "!! THRUST LOST at " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km with the".
        PRINT "   throttle open. Ending the climb here so the ship is flown".
        PRINT "   rather than left to fall under an autopilot that is still".
        PRINT "   commanding a burn.".
        IF dvNow > DV_GLIDE_RESERVE {
          PRINT "   The tanks still price " + ROUND(dvNow) + " m/s, so this is a feed".
          PRINT "   or ignition fault, not an empty ship:".
          PRINT "     core LF " + ROUND(ascCoreResAmt("LiquidFuel")) + " / Ox " +
                ROUND(ascCoreResAmt("Oxidizer")) + "  (burned as " + ROUND(LFO_LF_RATIO) +
                ":" + ROUND(LFO_OX_RATIO) + " pairs)".
          IF anyFlameout() { PRINT "     at least one engine reports FLAMEOUT.". }
          IF PAY_TANKS:LENGTH > 0 {
            PRINT "     payload tanks (locked out) hold LF " +
                  ROUND(ascListResAmt(PAY_TANKS, "LiquidFuel")) + " / Ox " +
                  ROUND(ascListResAmt(PAY_TANKS, "Oxidizer")) + ".".
          }
        }
        PRINT "======================================================".
        BREAK.
      }
    } ELSE {
      SET deadVotes TO 0.
    }

    // Floor 1: an orbit is genuinely in reach, so the deorbit money is real
    // money.  Stop climbing and bank the orbit we can still pay to leave.
    //
    // The two halves of that test are not the same question, and conflating
    // them is what stranded the reference flight.  Money must only be reserved
    // for a deorbit burn we are actually going to need, i.e. one from an orbit
    // we can still afford to close.  If the circularisation itself is already
    // out of reach, there is no orbit to leave, so there is nothing to reserve
    // and nothing is gained by cutting the engine here - climbing higher only
    // makes the circularisation dearer.  Fly flat for periapsis instead.
    IF SHIP:APOAPSIS >= MIN_ORBIT_ALT {
      LOCAL circHere IS circDvAt(SHIP:APOAPSIS).
      IF dvNow <= circHere + reserveDvFor(SHIP:APOAPSIS) AND dvNow > circHere {
        SET settled TO TRUE.
        SET stopReason TO "stopped on the circularise + deorbit reserve".
        SET targetAp TO SHIP:APOAPSIS.
        SET ccDone TO TRUE.
        BREAK.
      }
      IF dvNow <= circHere AND NOT emergency {
        SET emergency TO TRUE.
        PRINT "  Cannot afford to circularise at " + ROUND(SHIP:APOAPSIS / 1000, 1) +
              " km (" + ROUND(dvNow) + " m/s vs " + ROUND(circHere) + ").".
        PRINT "  Flying flat for the highest periapsis the fuel will buy.".
      }
    }

    // Flying flat can succeed: periapsis climbing clear of the air *is* an
    // orbit, and there is nothing left to buy up here.
    IF emergency AND SHIP:PERIAPSIS >= MIN_ORBIT_ALT {
      SET settled TO TRUE.
      SET stopReason TO "periapsis reached orbit on the flat burn".
      SET targetAp TO SHIP:APOAPSIS.
      SET ccDone TO TRUE.
      BREAK.
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
    //  the bill did the dV we have spent actually pay off?  That ratio re-prices
    //  the rest of the climb, and it is the number the settle decision is made
    //  on - which is why it is measured over a window rather than cumulatively.
    //  How lossy the climb *has been* is a fact about a burn that is over.  What
    //  the settle decision needs is how lossy it is *now*, because that is what
    //  the m/s still in the tanks are going to be spent at.
    LOCAL spent IS dvAtSwitch - dvNow.
    LOCAL priceNow IS dvToRaiseApTo(EFF_REF_AP).

    //  The whole-climb figure, cumulative from the switch.  Nothing in flight is
    //  decided on it - it exists so the post-flight advice can tell the next
    //  pre-flight check what this ascent really cost, end to end.
    IF spent > FEAS_ARM_DV {
      LOCAL paidAll IS price0 - priceNow.
      IF paidAll > 1 {
        SET lossWhole TO ascClampVal(spent / paidAll, 1, 12).
      } ELSE {
        SET lossWhole TO 12.
      }
      SET wholeMeasured TO TRUE.
    }

    //  The live figure, over a trailing window.  Each completed window rolls the
    //  head forward, so what comes out is the efficiency of the last
    //  EFF_WINDOW_DV m/s rather than an average dragged down by the worst part
    //  of the burn, which is always the first part.
    LOCAL spentWin IS effMarkDv - dvNow.
    IF spentWin >= EFF_WINDOW_DV {
      LOCAL paidWin IS effMarkPrice - priceNow.
      LOCAL winLoss IS 12.          // spending dV and buying nothing
      IF paidWin > 1 { SET winLoss TO ascClampVal(spentWin / paidWin, 1, 12). }
      SET lossFactor TO winLoss.
      IF winLoss >= 12 { SET deadWindows TO deadWindows + 1. }
      ELSE { SET deadWindows TO 0. }
      SET effWindows   TO effWindows + 1.
      SET effMarkDv    TO dvNow.
      SET effMarkPrice TO priceNow.
      SET lossMeasured TO TRUE.
    }

    IF lossMeasured {
      IF dvNow < priceOfOrbit(targetAp, lossFactor) {
        SET infeasVotes TO infeasVotes + 1.
      } ELSE {
        SET infeasVotes TO 0.
      }

      //  The verdict is not a one-way door.  A flat burn entered on a reading
      //  taken in the drag layer must be allowed to end when the air thins and
      //  the reading improves; latching it means no later evidence, however
      //  good, can ever restore the climb.  Requiring the orbit to clear the
      //  bill by EFF_RESUME_DV keeps it from chattering across the boundary.
      IF emergency AND
         priceOfOrbit(MIN_ORBIT_ALT, lossFactor) + EFF_RESUME_DV <= dvNow {
        SET emergency TO FALSE.
        SET infeasWarned TO FALSE.
        SET infeasVotes TO 0.
        PRINT "  Orbit back in budget at the measured x" + ROUND(lossFactor, 2) +
              " - resuming the climb.".
        PRINT "  " + budgetLine(MIN_ORBIT_ALT, lossFactor).
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
        } ELSE IF bestAp = 0 AND verdictAllowed() {
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
              PRINT "   Levelling off: every remaining m/s now goes into periapsis,".
              PRINT "   not apoapsis. The glide reserve is still protected.".
            }
            PRINT "======================================================".
            SET infeasWarned TO TRUE.
          }
          // Apoapsis has stopped being the objective.  Height we cannot pay to
          // circularise at is height that has to be shed again on reentry; the
          // remaining fuel is worth far more spent flat, raising periapsis.
          SET emergency TO TRUE.
          IF ABORT_IF_INFEASIBLE {
            SET settled TO TRUE.
            SET stopReason TO "aborted - orbit priced out of reach".
            SET targetAp TO SHIP:APOAPSIS.
            SET ccDone TO TRUE.
            BREAK.
          }
        } ELSE IF bestAp = 0 AND NOT infeasNoted {
          // Priced out on a reading taken too low to mean anything yet.  Say so
          // once, keep climbing, and let the thinner air answer the question.
          PRINT "  Priced out at x" + ROUND(lossFactor, 2) + ", but still in the".
          PRINT "  drag layer at " + ROUND(SHIP:ALTITUDE / 1000, 1) +
                " km - climbing on before calling it.".
          SET infeasNoted TO TRUE.
        }
      }
    }
  }

  // ---- periodic status ----------------------------------------------------
  IF nowT - reportT > 10 {
    PRINT "  AP " + ROUND(SHIP:APOAPSIS / 1000, 1) + "/" + ROUND(targetAp / 1000) +
          " km | alt " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km | " +
          ROUND(SHIP:VELOCITY:ORBIT:MAG) + " m/s".
    PRINT "      fpa " + ROUND(ascFpaNow(), 1) + "/" + ROUND(fpaWant, 1) + " | pitch " +
          ROUND(pitchCmd, 1) + " | acc " + ROUND(accelNow, 2) + " | TWR " +
          ROUND(twrNow(), 2) + " | dV " + ROUND(ascRocketDv()).
    // AoA and the drag it is buying, because this is where the climb is won or
    // lost and neither was visible on the flight that spent 330 m/s to buy 40.
    PRINT "      AoA " + ROUND(pitchCmd - ascFpaNow(), 1) + "/" + ROUND(aoaCap, 1) +
          " | Q " + ROUND(SHIP:Q, 3) + " | drag " + ROUND(dragNow(accelNow)) + " kN".
    // The loss factor every cycle, not only when it triggers a verdict.  On the
    // reference flights the first anyone heard of it was the abort message.
    IF lossMeasured {
      PRINT "      loss x" + ROUND(lossFactor, 2) + " over the last " +
            ROUND(EFF_WINDOW_DV) + " m/s (" + effWindows + " windows)".
    }
    LOCAL flatNote IS "".
    IF emergency { SET flatNote TO " | FLAT-BURN (orbit priced out)". }
    PRINT "      PE " + ROUND(SHIP:PERIAPSIS / 1000, 1) + " km | apex in " +
          ROUND(MIN(999, apexTimeEst())) + " s" + flatNote.
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
  PRINT "   Fuel kept back: " + ROUND(ascRocketDv()) + " m/s.".
  PRINT "======================================================".
  UNLOCK STEERING.
  UNLOCK THROTTLE.
  SAS ON.
}

IF NOT abortSuborbital {
  LOCK STEERING TO SHIP:PROGRADE.
  // Clear of the atmosphere - but never wait on that unconditionally.  If the
  // ship is already past apoapsis and coming down, that condition will never
  // become true and the autopilot would hold the ship nose-prograde all the way
  // into the ground.
  //
  // And hold the apoapsis on the way up.  MECO fires the moment the apoapsis
  // reading reaches the target, but the ship is still at 30 km doing orbital
  // speed with 40 km of thinning air to climb through, and that air keeps
  // charging: the 85 km flight cut the engines at 85.3 km of apoapsis and
  // arrived in vacuum with 81.6.  Nothing downstream can give it back - thrust
  // at apoapsis buys periapsis, not apoapsis - so it has to be held here, while
  // the ship is still under its apoapsis and prograde thrust still moves it.
  // Only ever with money that is not already promised to the circularisation
  // and the deorbit, and never below the glide reserve.
  SET trimDv0 TO ascRocketDv().
  SET trimOn  TO FALSE.
  SET trimT   TO TIME:SECONDS.
  UNTIL SHIP:ALTITUDE > ATM_TOP OR SHIP:VERTICALSPEED < 0 {
    IF COAST_TRIM AND TIME:SECONDS - trimT > 0.25 {
      SET trimT TO TIME:SECONDS.
      LOCAL apLack IS targetAp - SHIP:APOAPSIS.
      LOCAL keep IS MAX(circDvAt(targetAp) + reserveDvFor(targetAp), DV_GLIDE_RESERVE).
      LOCAL dvHere IS ascRocketDv().     // read once - it is not cheap in kOS
      IF apLack > COAST_TRIM_TOL AND dvHere > keep {
        SET thrCmd TO COAST_TRIM_THR.
        SET trimOn TO TRUE.
      } ELSE IF apLack <= 0 OR dvHere <= keep {
        SET thrCmd TO 0.
      }
    }
    WAIT 0.1.
  }
  SET thrCmd TO 0.
  IF trimOn {
    PRINT "Coast trim held the apoapsis through the air for " +
          ROUND(trimDv0 - ascRocketDv()) + " m/s.".
  }
  IF SHIP:ALTITUDE < ATM_TOP {
    SET abortSuborbital TO TRUE.
    PRINT "!! Apoapsis passed inside the atmosphere - no vacuum burn is possible.".
    PRINT "   Handing back to the pilot with " + ROUND(ascRocketDv()) + " m/s in hand.".
    UNLOCK STEERING.
    UNLOCK THROTTLE.
    SAS ON.
  }
}

IF NOT abortSuborbital {
  // Circularise at the apoapsis the ship actually arrived with, not the one it
  // had at MECO.  Whatever the coast trim could not hold on to is gone, and a
  // periapsis target built on the stale figure sits *above* the apoapsis the
  // ship has - which no burn can ever reach.  That is what made the 85 km
  // flight burn until the fuel floor: a 83.8 km periapsis target against an
  // 81.6 km apoapsis, with every m/s after the circular point going into
  // apoapsis instead.  It finished at 431 x 82 km.
  IF SHIP:APOAPSIS < orbitAlt - 100 {
    PRINT "Apoapsis is " + ROUND(SHIP:APOAPSIS / 1000, 1) + " km, not the " +
          ROUND(orbitAlt / 1000, 1) + " km at MECO - " +
          ROUND((orbitAlt - SHIP:APOAPSIS) / 1000, 1) + " km lost to the coast.".
  }
  SET orbitAlt TO SHIP:APOAPSIS.
  SET circDv   TO circDvAt(orbitAlt).
  // Half the burn before apoapsis and half after is what keeps the ship near the
  // altitude it is circularising at.  The taper at the end is part of the burn,
  // so it is part of what has to be centred.
  SET circBurn TO ascBurnTimeFor(circDv) + CIRC_TAPER_T.
  SET circLead TO ascClampVal(circBurn / 2, 5, 150).
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

  // ETA:APOAPSIS jumps to nearly a full orbit the instant apoapsis is behind
  // us, so the falling case has to be an exit condition of its own.
  WAIT UNTIL ETA:APOAPSIS < circLead OR SHIP:PERIAPSIS > ATM_TOP
             OR (SHIP:VERTICALSPEED < 0 AND SHIP:ALTITUDE < ATM_TOP).

  // ...and having made it an exit condition, it must not fall straight through
  // into the burn.  Apoapsis is behind us and the ship is back in the air:
  // prograde now points downwards, so a "circularisation" burn would drive the
  // nose into the descent and spend the glide reserve doing it.
  IF SHIP:VERTICALSPEED < 0 AND SHIP:ALTITUDE < ATM_TOP AND SHIP:PERIAPSIS < ATM_TOP {
    SET abortSuborbital TO TRUE.
    PRINT "!! Descending back into the atmosphere with apoapsis behind us -".
    PRINT "   there is no circularisation to make. Keeping " + ROUND(ascRocketDv()) +
          " m/s and handing back for the glide.".
    SET thrCmd TO 0.
    UNLOCK STEERING.
    UNLOCK THROTTLE.
    SAS ON.
  }
}

// ---------------------------------------------------------------------------
//  5. CIRCULARISE  --  fly the velocity error, stop when it stops shrinking,
//  and never eat the deorbit reserve
// ---------------------------------------------------------------------------
SET circStarved TO FALSE.           // stopped on the fuel floor
SET circStalled TO FALSE.           // stopped because it was no longer helping
SET circGo      TO 0.               // dV still wanted to be circular here
IF NOT abortSuborbital {
  SET circPeTarget TO MAX(ATM_TOP + PE_SAFETY, orbitAlt - CIRC_PE_TOL).
  SET circPeTarget TO MIN(circPeTarget, orbitAlt - 500).
  PRINT "Circularising at " + ROUND(orbitAlt / 1000, 1) + " km; PE target " +
        ROUND(circPeTarget / 1000, 1) + " km.".
  IF DEPLOY_BEFORE_DEORBIT AND payloadMass > 1 {
    PRINT "  Protecting " + ROUND(reserveDvFor(orbitAlt)) + " m/s here = deorbit " +
          ROUND(deorbitDvFrom(orbitAlt)) + " + margin " + ROUND(DV_MARGIN) +
          " once the payload is off.".
  } ELSE {
    PRINT "  Protecting " + ROUND(reserveDvFor(orbitAlt)) + " m/s (deorbit " +
          ROUND(deorbitDvFrom(orbitAlt)) + " + margin " + ROUND(DV_MARGIN) + ").".
  }

  SET dvCheckT TO TIME:SECONDS.
  SET circT0   TO TIME:SECONDS.
  SET circVec  TO circBurnVec().
  SET circGo   TO circVec:MAG.
  SET circBest TO circGo.
  SET circAim  TO SHIP:PROGRADE:VECTOR.
  IF circGo > CIRC_AIM_MIN { SET circAim TO circVec. }
  LOCK STEERING TO circAim.

  SET circDone TO FALSE.
  SET thrCmd   TO 1.
  UNTIL circDone {
    SET circVec TO circBurnVec().
    SET circGo  TO circVec:MAG.

    IF circGo <= CIRC_DV_TOL OR SHIP:PERIAPSIS >= circPeTarget {
      // Round enough.  Either the velocity error is inside the tolerance or the
      // periapsis the mission asked for is already bought.
      SET thrCmd TO 0.
      SET circDone TO TRUE.
    } ELSE {
      // Point at what is left to do, not at prograde.  Near apoapsis the two are
      // the same vector; once the ship has picked up any descent they are not,
      // and the difference is the part prograde thrust cannot fix.  Below
      // CIRC_AIM_MIN the direction is a small difference of two ~2 km/s vectors,
      // so it is frozen rather than chased.
      IF circGo > CIRC_AIM_MIN {
        SET circAim TO circVec.
        // ...with one thing taken back out.  The burn starts before apoapsis, so
        // for its first half the ship is still climbing, and "circularise here"
        // then contains a downward component that would spend thrust cancelling
        // a climb the ascent has already paid for.  While the ship is rising,
        // fly the flat part of the correction only; the vertical term is what
        // the *second* half of the burn is for.
        IF SHIP:VERTICALSPEED > 0 AND VDOT(circAim, SHIP:UP:VECTOR) < 0 {
          LOCAL flatAim IS VXCL(SHIP:UP:VECTOR, circAim).
          IF flatAim:MAG > 0.1 * circGo { SET circAim TO flatAim. }
        }
      }

      // Taper over the last CIRC_TAPER_T seconds of burn instead of dropping to
      // a fixed low throttle on a periapsis threshold.  The old ease-down fired
      // 4 km out - which on an 85 km orbit is the last 4 m/s of a 163 m/s burn -
      // and stretched exactly the part of the burn where drifting off apoapsis
      // costs the most.
      LOCAL accNow IS SHIP:AVAILABLETHRUST / MAX(1, SHIP:MASS).
      IF accNow <= 0.01 {
        SET thrCmd TO 1.
      } ELSE {
        SET thrCmd TO ascClampVal(circGo / (accNow * CIRC_TAPER_T), CIRC_THR_MIN, 1).
      }

      IF circGo < circBest { SET circBest TO circGo. }

      // The burn is over when it stops buying a rounder orbit.  Past the
      // circular point every further m/s goes into apoapsis, and the ship is
      // paying rocket fuel to make the orbit *worse* - which is precisely what
      // ran 85 km out to 431 km.  The apoapsis ceiling is a second, blunter
      // statement of the same thing, in case the velocity figure is ever fooled.
      //
      // Neither test may fire until periapsis is clear of the air.  Below that
      // there is no orbit yet to protect, only a reentry with a good view, and
      // a ship still climbing towards its apoapsis does drag periapsis up with
      // it even while it is over circular speed.  Until it has an orbit the only
      // things that stop this burn are the fuel floors.
      IF SHIP:PERIAPSIS > ATM_TOP AND TIME:SECONDS - circT0 > CIRC_ARM_T AND
         (circGo > circBest + CIRC_GIVEUP_DV OR
          SHIP:APOAPSIS > orbitAlt + CIRC_AP_TOL) {
        SET thrCmd TO 0.
        SET circStalled TO TRUE.
        SET circDone TO TRUE.
      }

      // Hard stop: we will not eat the deorbit money to buy a rounder orbit.
      // (Polled a few times a second - ascRocketDv() is not cheap in kOS.)
      //
      // But the deorbit reserve is only worth protecting while there is going to
      // be an orbit to deorbit *from*.  Until periapsis is clear of the air the
      // ship is on a reentry path already, no deorbit burn will ever be needed,
      // and holding fuel back for one just guarantees the reentry is steeper than
      // it had to be.  So the floor is the glide reserve until periapsis is up,
      // and the full reserve only once it is.
      IF NOT circDone AND TIME:SECONDS - dvCheckT > 0.25 {
        SET dvCheckT TO TIME:SECONDS.
        LOCAL circFloor IS DV_GLIDE_RESERVE.
        IF SHIP:PERIAPSIS >= ATM_TOP {
          SET circFloor TO MAX(reserveDvFor(orbitAlt), DV_GLIDE_RESERVE).
        }
        IF ascRocketDv() <= circFloor {
          SET thrCmd TO 0.
          SET circStarved TO TRUE.
          SET circDone TO TRUE.
        }
      }
    }
    IF NOT circDone { WAIT 0.05. }
  }
  SET thrCmd TO 0.
  UNLOCK STEERING.
  SAS ON.
}

// ---------------------------------------------------------------------------
//  6. REPORT  --  unconditionally.  Every abort path above ends the flight
//  somewhere the pilot has to fly out of, and "what state am I in, and with how
//  much fuel" is exactly the question those paths leave open.  The banners above
//  say why the ascent stopped; this says where it stopped.
// ---------------------------------------------------------------------------
// Did we actually end up in orbit?  Apoapsis alone does not answer that - a
// ship can have a 78 km apoapsis and a periapsis 100 km underground, which is
// a reentry trajectory with a good view.  Periapsis clear of the atmosphere is
// the only test that means anything, and everything below is reported against it.
SET inOrbit TO SHIP:PERIAPSIS > ATM_TOP.

// A burn that stopped because it had stopped helping is not a burn that ran out
// of money, and saying so mattered: the flight that ran 85 km out to 431 km was
// reported as "stopped on the deorbit reserve", which reads as "bring more fuel"
// when the fuel was never the problem.
IF circStalled {
  PRINT "!! Circularisation stopped where it stopped helping - " + ROUND(circGo, 1) +
        " m/s from circular".
  PRINT "   at " + ROUND(SHIP:ALTITUDE / 1000, 1) + " km, and no longer closing.".
  IF SHIP:APOAPSIS > orbitAlt + CIRC_AP_TOL {
    PRINT "   Apoapsis ran to " + ROUND(SHIP:APOAPSIS / 1000, 1) + " km against a " +
          ROUND(orbitAlt / 1000, 1) + " km target: the burn was".
    PRINT "   past circular and pushing apoapsis, so it was cut. Fuel is intact.".
  } ELSE {
    PRINT "   Anything further would have gone into apoapsis, not periapsis.".
  }
}
IF circStarved {
  IF inOrbit {
    PRINT "!! Circularisation stopped on the deorbit reserve.".
    PRINT "   PE " + ROUND(SHIP:PERIAPSIS / 1000, 1) + " km - the orbit stays elliptical,".
    PRINT "   but the deorbit burn is still funded.".
  } ELSE {
    PRINT "!! Circularisation ran out of fuel with PE " +
          ROUND(SHIP:PERIAPSIS / 1000, 1) + " km, still inside the atmosphere.".
    PRINT "   Everything above the glide reserve went into periapsis; the ship".
    PRINT "   is on a reentry path and needs no deorbit burn.".
  }
}

SET dvLeft  TO ascRocketDv().
SET dvDeorb TO deorbitDvFrom(SHIP:APOAPSIS).
IF NOT inOrbit { SET dvDeorb TO 0. }   // nothing to deorbit from
PRINT "======================================================".
IF NOT inOrbit {
  PRINT "!! NO ORBIT. Periapsis " + ROUND(SHIP:PERIAPSIS / 1000, 1) +
        " km is inside the atmosphere -".
  PRINT "   this is a suborbital arc, not an orbit. The ship will reenter on".
  PRINT "   its own; do NOT run deorbit_land. Fly it home as a glider.".
} ELSE IF settled OR SHIP:APOAPSIS < REQUESTED_APOAPSIS - 1000 {
  // Say it whichever way the ship ended up low: the climb settling for a cheaper
  // orbit and the coast quietly costing a few km read identically on the map,
  // and both are worth knowing before anyone plans a rendezvous on it.
  PRINT "ORBIT ACHIEVED (below the requested " +
        ROUND(REQUESTED_APOAPSIS / 1000) + " km)".
} ELSE {
  PRINT "ORBIT ACHIEVED".
}
PRINT "  Apoapsis   : " + ROUND(SHIP:APOAPSIS / 1000, 2) + " km".
PRINT "  Periapsis  : " + ROUND(SHIP:PERIAPSIS / 1000, 2) + " km".
PRINT "  Inclination: " + ROUND(SHIP:ORBIT:INCLINATION, 2) + " deg".
PRINT "  Mass       : " + ROUND(SHIP:MASS, 1) + " t  (burned " +
      ROUND(launchMass - SHIP:MASS, 1) + " t)".
IF wholeMeasured {
  PRINT "  Climb loss : x" + ROUND(lossWhole, 2) + " over the whole climb, x" +
        ROUND(lossFactor, 2) + " at the end of it".
} ELSE IF lossMeasured {
  PRINT "  Climb loss : x" + ROUND(lossFactor, 2) + " over the last " +
        ROUND(EFF_WINDOW_DV) + " m/s (too short a burn for a whole-climb figure)".
} ELSE {
  PRINT "  Climb loss : not measured (rocket burn shorter than " +
        ROUND(EFF_WINDOW_DV) + " m/s)".
}
PRINT "  dV left    : " + ROUND(dvLeft) + " m/s  (our tanks only)".
PRINT "  Deorbit    : " + ROUND(dvDeorb) + " m/s  ->  spare after deorbit " +
      ROUND(dvLeft - dvDeorb) + " m/s".
// The same propellant, priced after the cargo is gone.  Every dV figure above is
// computed at the ship's *current* mass, payload included, but the deorbit burn
// happens after the payload is released - on a much lighter ship, where the same
// tonnes in the tank are worth far more.  Budgeting the reserve at the heavy
// mass is a real reserve the ship does not need to carry, and on a 27% payload
// fraction it is most of the reserve.
IF payloadMass > 1 AND ascRocketPropMass() > 0 {
  LOCAL mLight IS SHIP:MASS - payloadMass.
  IF mLight > ascRocketPropMass() {
    LOCAL dvLight IS RKT_ISP * G0 * LN(mLight / (mLight - ascRocketPropMass())).
    PRINT "  After release of " + ROUND(payloadMass, 1) + " t of payload the same".
    PRINT "  propellant is worth " + ROUND(dvLight) + " m/s (x" +
          ROUND(dvLight / MAX(1, dvLeft), 2) + ") - deploy before deorbiting.".
  }
}
PRINT "  Fuel -- LF: " + ROUND(ascCoreResAmt("LiquidFuel")) +
      " , Ox: " + ROUND(ascCoreResAmt("Oxidizer")) +
      " , Mono: " + ROUND(ascResAmtShip("MonoPropellant")).
IF PAY_TANKS:LENGTH > 0 {
  PRINT "  Payload LF/Ox (locked out): " + ROUND(ascListResAmt(PAY_TANKS, "LiquidFuel")) +
        " / " + ROUND(ascListResAmt(PAY_TANKS, "Oxidizer")).
}
// Answer the funding question at the mass the burn will be flown at.  dvLeft is
// a reading on the ship as it sits, cargo aboard; comparing it against a deorbit
// figure that will be flown after release is what made the reference flight look
// stranded at 54 m/s when it was holding enough for a comfortable deorbit.
IF NOT inOrbit {
  PRINT "  No deorbit burn required - the trajectory already reenters.".
} ELSE {
  LOCAL needFunded IS ascHeavyEquivalentOf(dvDeorb + DV_MARGIN).
  LOCAL needBare   IS ascHeavyEquivalentOf(dvDeorb).
  IF DEPLOY_BEFORE_DEORBIT AND payloadMass > 1 {
    PRINT "  Deorbit costs " + ROUND(needBare) + " m/s of what is on the gauge now" +
          " (" + ROUND(dvDeorb) + " m/s after release).".
  }
  IF dvLeft >= needFunded {
    PRINT "  DEORBIT FUNDED - RUN deorbit_land. when you are ready.".
  } ELSE IF dvLeft >= needBare {
    PRINT "  Deorbit funded but with no margin - deorbit promptly.".
  } ELSE {
    PRINT "  !! DEORBIT NOT FUNDED - refuel, or lower PE on RCS monopropellant.".
  }
  IF DEPLOY_BEFORE_DEORBIT AND payloadMass > 1 {
    PRINT "  RELEASE THE PAYLOAD FIRST - that is what these numbers assume.".
  }
}
PRINT "======================================================".
PRINT "Autopilot complete. Ship handed back to pilot (SAS on).".

// Hand the controls back for real.  On the abort paths this already happened;
// on the normal path the throttle was still LOCKed to thrCmd, and leaving a
// lock behind after the program ends is how a pilot finds the stick dead.
// PILOTMAINTHROTTLE was zeroed on the runway, so nothing lights up here.
SET thrCmd TO 0.
UNLOCK THROTTLE.
UNLOCK STEERING.

// ---------------------------------------------------------------------------
//  7. POST-FLIGHT  --  what would this ship need, going by what it just did?
// ---------------------------------------------------------------------------
//  The pre-flight check answers this from a model.  By now the model is
//  redundant: the ship has flown the mission and every assumption in it has a
//  measured counterpart.  So the same sizing solvers are re-run with the real
//  numbers substituted in - the jet phase's actual fuel fraction, the actual
//  handover, the actual climb losses - and they price the mission the ship
//  really flew rather than the one it was expected to.
//
//  This is the block to read when deciding what to change in the SPH.  The
//  calibration constants are printed first so the next pre-flight check is
//  about this airframe instead of a generic one.
IF jetLfUsed > 0 {
  PRINT "======================================================".
  PRINT "POST-FLIGHT :: measured, not modelled".
  PRINT "  Copy these into the tunables to calibrate the pre-flight check:".
  PRINT "    SET PLAN_SWITCH_ALT  TO " + ROUND(swAlt / 100) * 100 + ".".
  PRINT "    SET PLAN_SWITCH_SPD  TO " + ROUND(swSpeed / 10) * 10 + ".".
  PRINT "    SET PLAN_JET_DV      TO " + ROUND(jetDvReal / 10) * 10 +
        ".   // was " + ROUND(planJetDv).
  IF wholeMeasured {
    // The *whole-climb* figure, not the trailing window.  PLAN_LOSS_FACTOR
    // prices a climb that starts at the handover, so it has to include the
    // expensive bottom of it; the window figure describes the cheap end and
    // would tell the next pre-flight check the ascent costs less than it does.
    PRINT "    SET PLAN_LOSS_FACTOR TO " + ROUND(lossWhole, 2) + ".".
  } ELSE {
    // The whole-climb yardstick only arms after FEAS_ARM_DV of rocket burn.  A
    // short push never armed it, so lossWhole is still the default it was
    // seeded with - quoting that back as a measurement would be a lie.
    PRINT "    (climb losses not measured - the rocket burn was under " +
          ROUND(FEAS_ARM_DV) + " m/s, so PLAN_LOSS_FACTOR is unchanged.)".
  }

  // Re-price the mission the way the pre-flight check does, but with the jet
  // phase's *measured* appetite and the *measured* climb losses.  Everything
  // else the solvers need (pad propellant, tankage, payload) is unchanged -
  // they are properties of the design, which is what we are advising on.
  SET JET_FRAC   TO jetLfUsed / MAX(0.001, launchMass).
  SET planSwSpd  TO swSpeed + ROT_BONUS.
  SET dvRequired TO priceFromHandover(REQUESTED_APOAPSIS, swAlt, planSwSpd, lossWhole).
  SET dvHandover TO dvAtHandover(0, 0).

  PRINT "  For a " + ROUND(REQUESTED_APOAPSIS / 1000) + " km orbit with deorbit fuel," +
        " this flight says you need".
  PRINT "  " + ROUND(dvRequired) + " m/s at the handover and the design delivers " +
        ROUND(dvHandover) + " m/s.".

  IF dvHandover >= dvRequired {
    PRINT "  => The airframe is sufficient (" + ROUND(dvHandover - dvRequired) +
          " m/s spare). No changes required.".
  } ELSE {
    PRINT "  => SHORT BY " + ROUND(dvRequired - dvHandover) + " m/s. To fix it:".
    SET fixLf TO solveLfTopUp().
    IF fixLf > 0.05 {
      PRINT "     * +" + ROUND(fixLf, 1) + " t of LIQUID FUEL ONLY (" +
            ROUND(fixLf / LF_DENS) + " units). The jets ate into paired".
      PRINT "       propellant; LF burns at jet Isp, so this is the cheap fix.".
    }
    SET fixProp TO solveProp(dvRequired, 0).
    IF fixProp < 0 {
      PRINT "     * Fuel alone cannot close it - the mass ratio tops out at " +
            ROUND(dvAtHandover(20000, 0)) + " m/s.".
      PRINT "       Cut dry mass or payload; more tanks will not help.".
    } ELSE {
      PRINT "     * or +" + ROUND(fixProp, 1) + " t of balanced LF/Ox plus ~" +
            ROUND(fixProp * TANK_K, 1) + " t of tank to hold it.".
    }
    SET fixCut TO solveMassCut(dvRequired).
    IF fixCut > 0 {
      PRINT "     * or fly " + ROUND(fixCut, 1) + " t lighter (of " +
            ROUND(payloadMass, 1) + " t of payload aboard).".
    }
    SET affordAp TO affordableApFrom(dvHandover, swAlt, planSwSpd, lossWhole).
    IF affordAp > 0 AND affordAp < REQUESTED_APOAPSIS {
      PRINT "     * or set REQUESTED_APOAPSIS TO " + ROUND(affordAp / 1000) * 1000 +
            ". - that orbit this ship can already afford.".
    }
  }
  IF wholeMeasured AND lossWhole > 2 {
    PRINT "  !! Climb losses of x" + ROUND(lossWhole, 2) + " are TWR-bound, not".
    PRINT "     guidance-bound. Closed-cycle TWR was " + ROUND(twrAtSwitch, 2) +
          " at the handover; another".
    PRINT "     engine buys more than any profile change can.".
  }
  PRINT "======================================================".
}

SET CONFIG:IPU TO IPU_SAVED.
