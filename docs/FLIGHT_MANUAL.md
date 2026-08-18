# Flight manual — Mk3 "Titan" Heavy-Lift SSTO

How to fly the ship, both with the kOS autopilot and by hand.

## Recommended action groups

Set these in the SPH (they make manual flight and script backup easy):

| Group | Action | Why |
|---|---|---|
| **1** | Toggle RAPIER **mode** (air-breathing ⇄ closed cycle) | manual backup to the ascent script's auto-switch |
| **2** | Toggle **cargo bay** doors (both CRG-100s) | load / deploy payload |
| **3** | Toggle **shielded docking port** shield | open before docking, close before reentry |
| **4** | Toggle **antennas** (extend/retract) | deploy in orbit |
| **Gear (G)** | Landing gear | — |
| **Brakes (B)** | Wheel brakes + airbrakes | reentry drag & rollout |
| **Lights (U)** | Illuminators | night ops |
| **RCS (R)** | RCS | docking |
| **SAS (T)** | SAS | manual flight |

The kOS scripts drive throttle, steering, gear, brakes, lights, RCS, and the
RAPIER mode **directly**, so they work even if you forget the action groups —
but Group 1 is a handy manual override.

---

## Auto flight (kOS)

### To orbit

```
RUN ascent.             // 100 km, the default
RUN ascent(150000).     // or any apoapsis, in metres
```

The apoapsis is optional and is given in **metres**. It must clear the
atmosphere of the body being launched from: the script compares it against that
body rather than a hardcoded 70 km, and if it falls short it says so, explains
where the lowest real orbit is, and flies that instead. A value outside the
body's sphere of influence is clamped the same way. (Passing `150` when you
meant 150 km is caught explicitly.)

Sequence: map the fuel network and isolate the payload → measure both engine
modes and run the feasibility check → roll → rotate at 80 m/s → air-breathing
climb along a self-tuning dynamic-pressure corridor → switch to closed cycle
when the rocket becomes the cheaper way to buy speed → rocket push to the
requested apoapsis → coast (time-warped) → circularise → report, including what
to change on the ship. Hands off the whole way.

### The pre-flight feasibility check

Before the brakes come off, the script answers the question the autopilot can't
answer once it's airborne: **is there enough ΔV aboard, and if not, how much
more of what?** It ignites at zero throttle, flips the RAPIERs to closed cycle
just long enough to read their real thrust and Isp, flips back, and prices the
mission.

The model is explicit about the two things that make a spaceplane different
from a rocket:

* **The jets drink the rocket's liquid fuel.** The ΔV figure you'd read on the
  pad counts every paired LF/Ox unit in the tanks, which badly overstates what
  survives to the mode switch. The check burns the jet phase *first* (rocket
  equation at the measured air-breathing Isp over `PLAN_JET_DV`), then prices
  the rocket phase against what's left. Note this can legitimately come out
  *higher* than the pad figure — burning an LF-only reserve lightens the ship
  without touching a single paired unit, which is what that reserve is for.
* **Adding propellant is not free.** Tanks have structure, so x tonnes of fuel
  costs x(1+k) on the runway, where k is measured from *this ship's own* tanks
  (wet wings are not the same deal as a plain Mk3 fuselage). That's why the
  answer to "how much more fuel?" isn't linear — and why there's a hard
  ceiling: as tanks are added the mass ratio tends to (1+k)/k, and a design
  needing more than that ceiling **cannot be fixed with fuel at all**.

If the ship comes up short it says by how much, then ranks the fixes:

1. **Liquid fuel first**, if the jets are eating paired LF. This is the most
   common way a sound SSTO comes up short, and it's much the cheapest fix — LF
   burns at Isp ~3200 in the jets, while oxidizer is just cargo. The check
   quotes the LF-only top-up and what it buys.
2. **Balanced LF/Ox**, converged against the TWR it costs: if the extra
   propellant drops closed-cycle TWR below `PLAN_TWR_MIN`, it adds RAPIERs and
   re-solves the fuel to haul them, iterating until both close.
3. **Less payload**, or a lower orbit.

It also flags **oxidizer that can never be paired** — pure dead mass carried to
orbit and back.

The check assumes a *healthy* handover (`PLAN_SWITCH_ALT` 20 km,
`PLAN_SWITCH_SPD` 1450 m/s, `PLAN_JET_DV` 3000 m/s). If your airframe hands over
somewhere else the verdict will be off, so the script prints the **actual**
handover altitude, speed and jet ΔV at the real mode switch — trim the three
assumptions to those numbers and the next check is about your ship rather than a
generic one. A failed check does not stop the launch; it warns, pauses
`PREFLIGHT_HOLD` seconds so you can read it, and flies anyway, because the
in-flight budget will settle for whatever it can reach.

### In flight

The script is not tuned to one airframe — it will fly any RAPIER spaceplane that
has the performance to reach orbit at all. Four things it decides for itself:

* **Whose fuel is whose.** It walks the part tree outward from the engines and
  refuses to cross any decoupler, separator or docking port. Only tanks inside
  that feed network count as ΔV, and crossfeed is switched off on every
  separator, so a **fuelled payload can neither inflate the budget nor be
  siphoned during the climb**. If a separator exposes no crossfeed toggle, the
  script falls back to disabling flow on the payload's tanks and tells you to
  re-enable them after you release the payload.
* **The flight profile — ride the air, don't climb out of it.** The
  air-breathing phase holds a **dynamic-pressure corridor**: Q is the air the
  intakes are being fed, so holding Q is the same thing as staying where the
  jets work. The command is derived rather than searched for — since
  Q = ½ρv² and ρ ~ e^(−h/H), the vertical speed that holds Q is 2Ha/v, plus a
  proportional term on the standing error — and H is measured off the body's own
  pressure curve, so nothing is tuned to Kerbin. The target Q is itself trimmed
  slowly against sustained acceleration, so the corridor ends up where *this*
  airframe's drag and intakes balance. The climb is capped at 12° of flight
  path, which is separate from the 25° pitch clamp: flight path is the physics,
  pitch is mostly the angle of attack the trim needs to fly at all. A single
  controller turns the command into pitch — feed-forward γ plus a slow, clamped
  trim that learns the airframe's AoA — so the nose moves smoothly instead of
  hunting. The rocket phase only ever *shallows*: spare thrust is never a reason
  to point further from the horizon.
* **When the jets are done — a price, not a threshold.** Every sample both modes
  are costed in the same currency: tonnes of rocket-equivalent propellant per
  m/s of speed bought. Drag is read out of the force balance the ship is already
  flying (D = T − m(a + g·sin γ)), so the closed-cycle side of the comparison is
  the same airframe at the same Mach and Q with only the engine swapped. Once
  the LF-only reserve is gone, a tonne of liquid fuel also strands 11/9 t of
  oxidiser, so the jet's price rises by 20/9 and the comparison knows it. The
  jets keep running for exactly as long as they are the cheaper way to buy
  speed. Three backstops bound it — 25 km, 1700 m/s, or Q below 0.04 atm — plus
  an acceleration floor, because in level flight the jets stay cheaper per m/s
  down to a crawl and the priced test alone would cruise forever.
* **What orbit it can actually afford.** It flies for the requested apoapsis
  (the `RUN ascent(...)` argument, 100 km by default) but *measures* what the
  climb really costs — how much of the bill each m/s of spent ΔV retires — and
  re-prices the orbit from that instead of trusting an impulsive estimate. If
  the target is priced out it settles for the best orbit it can afford and says
  so; if even a minimum orbit is gone it levels off and spends what is left
  raising periapsis, which is what buys a survivable reentry.
* **What to change on the ship.** After the flight every assumption in the
  pre-flight model has a measured counterpart, so the same sizing solvers are
  re-run with the real jet fuel fraction, handover state and climb losses. It
  prints the calibration constants to paste back into the tunables, then what
  the airframe actually needs: liquid fuel only, balanced LF/Ox plus tankage,
  payload to shed, or the apoapsis it can already afford.

**It never burns dry.** Two hard floors police every burn. Once an orbit is
genuinely in reach the ascent stops at *circularise + deorbit + margin*, and
below that a glide reserve (`DV_GLIDE_RESERVE`, 120 m/s) is kept back no matter
what. The circularisation burn stops dead on the deorbit reserve rather than
spending it — you may end up slightly elliptical, but always able to come home.

**And it never burns long.** The circularisation is flown against the velocity
error — the difference between the circular velocity where the ship is and the
velocity it has — not against a periapsis reading taken from a target that may
have moved. It aims along that error, so it can pitch up to arrest the descent a
finite burn always picks up, and it stops the moment the error stops shrinking:
past the circular point every further m/s goes into *apoapsis*, and that is a
burn making the orbit worse. See `docs/CIRCULARISE_REVIEW.md` for the flight
that went to 431 × 82 km without it.

**The apoapsis is held through the last of the air.** MECO fires on an apoapsis
reading taken at ~30 km at orbital speed, and the climb out to the vacuum costs
some of it back — 3.7 km on the reference 85 km flight. Apoapsis can only be
bought from below it, so `COAST_TRIM` pulses a quarter throttle against any sag
on the way up, spending only what is not already promised to the circularisation
and the deorbit. Set it `FALSE` to accept whatever the coast delivers.

If the ship is so short of ΔV that the apoapsis never clears the atmosphere, the
script says so, skips circularisation, and hands back a glider with fuel still in
the tanks — no deorbit burn is needed from there.

### Meeting a station

```
RUN rendezvous.                    // whatever is selected as target on the map
RUN rendezvous("Station Alpha").   // or by name - a substring is enough
RUN rendezvous("Alpha", 250).      // ...and a park distance, in metres
```

Both parameters are optional. With no name the script uses the map's target
selection; with one it matches case-insensitively on any part of the vessel
name, and **refuses an ambiguous match** rather than guessing — guessing between
two stations is how a ship ends up alongside the wrong one after a two-hour
phasing wait. It stops a few hundred metres out and hands over; it does **not**
dock.

Sequence: acquire and validate the target → map the fuel network and isolate the
payload → price the whole rendezvous and log the resources aboard → plane match
→ tidy the orbit if the ascent left it eccentric → solve and fly a phasing orbit
→ warp the wait → transfer injection, refined against the real geometry →
mid-course correction → kill the relative velocity at closest approach → RCS
approach to the park distance → report and hand over.

The part worth understanding before you run it: **the phasing orbit is solved,
not picked.** A Hohmann transfer needs the target at a particular lead angle
when you leave, and that angle comes round at a rate set by the difference
between the two orbital periods. If you are at 100 km and the station is at
100 km, that rate is *zero* — the angle never arrives, and a script that waits
for it waits forever. So the script sweeps candidate phasing radii, prices each
(two burns in and out, against the wait it buys), and flies the cheapest one
whose total time fits `MAX_TOTAL_TIME`. Your current orbit is one of the
candidates, so "just wait here" wins whenever it is affordable.

**If a plan looks expensive, give it more time.** `MAX_TOTAL_TIME` defaults to
two hours; raising it routinely finds plans that cost a fraction as much. The
budget block prints the phasing orbit it chose and what the detour cost, so the
trade is visible before anything burns.

Two safety properties are worth knowing about:

* **Nothing spends the deorbit reserve.** Every burn is policed against the fuel
  needed to get home, priced at the mass the deorbit will actually be flown at —
  i.e. *after* the cargo comes off at the station. A burn that would breach it
  stops and says so.
* **The approach speed is what the ship can stop from**, computed from the
  translational authority the RCS really has at the mass really aboard. If the
  ship somehow arrives closing faster than the thrusters can brake, the main
  engine comes back on and the nose swings onto the braking vector until it is
  back inside the envelope.

Monopropellant is budgeted separately and reported separately — the approach is
flown on RCS, and running mono dry a hundred metres off a station leaves the ship
drifting. See [`docs/RENDEZVOUS.md`](RENDEZVOUS.md) for the full derivation, the
tunables, and how the geometry and the approach law were verified outside the
game.

### Docking

```
RUN dock.               // both ports as they are set up in the game
RUN dock("dorsal").     // or name our port, by part tag or title
RUN dock("", 60).       // ...and force the standoff distance, in metres
```

**Two things to set in the game first**, and the script will tell you off for
either one missing:

1. **Target the port**, not the vessel — right-click the port you want to dock
   with and *Set as Target*.
2. **Control from your port** — right-click the ship's own docking port and
   *Control From Here*. The script does this for you when it can work out which
   port you mean, but it will not guess between two of them.

Neither ship has to be lined up. The script assumes they are not, and will start
from anywhere inside physics range: in front of the port, alongside, or behind
the station.

Sequence: identify and validate both ports → open our shield → measure both
craft and draw a keep-out sphere from their part trees → price the approach in
monopropellant, GO / MARGINAL / NOT ENOUGH → route to a standoff point on the
target port's axis, *around* the station rather than through it → match the port
axes → close down the corridor inside a narrowing cone → coast the last half
metre onto the magnets → report.

The parts worth understanding before you run it:

* **The approach is routed, not aimed.** The only way into a port is straight
  out in front of it, so the path is three legs — out sideways to a clear
  radius, forward past the port's plane, then in along the axis. Each leg either
  increases the distance to the station or holds it constant.
* **The keep-out sphere is measured.** Farthest part of the target from its
  port, plus farthest part of ours from ours, plus slack. A big station gets a
  wide berth automatically, and so does a long spaceplane swinging its tail
  round at the standoff.
* **It never closes while misaligned.** The axial closing rate is zero unless
  the ship is inside a cone that narrows with the range left *and* the two port
  axes are within a few degrees. Lateral first, angle second, gap last.
* **The speed is what the RCS can stop from**, measured in flight, exactly as in
  the rendezvous. An arrival it *cannot* stop from is refused before anything
  moves rather than discovered at two metres.

If it stops without docking it says why and leaves the ship lined up and still,
which is the easiest possible state to finish by hand. See
[`docs/DOCKING.md`](DOCKING.md) for the geometry, the tunables, and the
simulation that found two real defects in the first version of the control law.

### Home again

```
RUN deorbit_land.
```

Sequence: orient retrograde → predict the deorbit point (lead angle before the
KSC) and plant a maneuver node there → time-warp to the burn → burn periapsis
down to ~32 km → **turn prograde and warp the coast down to the interface** →
high-AoA reentry → energy-managed glide homing on the final approach fix →
capture heading 090 → glideslope → flare → gear down → touchdown → brake to a
stop.

The deorbit point is *solved*, not waited out — and so is the angle it is solved
for. A retrograde burn makes the burn point the apoapsis of the ellipse it
creates, so the coast from there to the interface is fixed by two radii, and
`coastArcDeg()` returns the ground track that coast sweeps (73.1° and 7.25 min on
a 100 km orbit aiming at 32 km). `DEORBIT_LEAD` is that arc plus `ENTRY_RANGE`
converted to degrees, which reproduces the hand-trimmed 149° to within half a
degree and, unlike it, follows the parking altitude. The script then scans the
next two orbits with `POSITIONAT`, correcting for Kerbin's own rotation, bisects
the moment the ground track is `DEORBIT_LEAD` short of the KSC, prices the burn off
the orbit it will be on at that instant, and warps there. It gets pointed
retrograde *before* warping, because on rails the ship cannot rotate — and
re-settles on the node burn vector after dropping out, since inertial attitude
drifts away from retrograde during the coast. Warp ends `WARP_LEAD` seconds
early to leave room for that. If no crossing is found (a wildly inclined or
non-circular orbit), it says so and falls back to the old real-time wait.

The burn over, the ship turns **prograde** — there is nothing left to point the
engines at, and the attitude it holds through the coast is the attitude it meets
the air in, so it had better be the entry attitude and not a view of its own
wake. Then the coast itself is solved rather than sat through: `coastTimeTo()`
bisects the predicted trajectory for the moment it falls through the atmosphere
height, prints it, and warps to `COAST_LEAD` seconds short of it whenever that
is worth more than `COAST_WARP_MIN`. Rails freeze attitude, so the nose is put
back on the airstream with the lead that was kept back for it.

#### How the atmospheric part is flown

Everything below the interface is commanded **relative to the airstream**, not
to the horizon. `aeroSteer(hdg, aoa, bank)` puts the nose `aoa` degrees above
the surface velocity vector and re-derives the pitch every physics tick, so the
angle of attack is what is actually held however steeply the ship is descending.
An absolute pitch command is what stalls a spaceplane: "3° nose down" while
falling at −25° is +22° of AoA, well past the wing. The AoA clamp
(`GLIDE_AOA_MAX`, 14°) is the stall guard and no phase relaxes it.

That convention is necessary but not sufficient, because a guidance law can still
ask for an attitude the ship has no *speed* to fly to. Two more rules sit over
the top of it, and `docs/REENTRY_REVIEW.md` is the flight that proves why:

* **The envelope.** Every phase hands its heading and AoA demand to `setNav()`,
  which folds the heading into the airstream on a speed schedule — 25° of lean
  while hypersonic, 15° once subsonic — limits bank on the same schedule, and
  takes bank to **zero** at the stall. That limit is *sideslip*, not turn rate:
  the bank commanded from the same error does the turning and costs no sideslip
  at all, so a tight yaw bound slows no turn down. Bounding the command costs
  nothing either way — the demand is re-referenced to the current airstream every
  pass, so the ship still comes round to any heading asked for, it just flies the
  turn instead of snapping the nose to the answer.
* **Recovery outranks navigation.** With less than `STALL_Q` of dynamic pressure
  under the wing, or with the nose more than `DEPART_AOA` off the airstream for
  three seconds, the ship is not flying and no amount of navigation fixes that:
  `stallRecover()` unloads the wing, levels the wings, stops asking for a turn,
  lights the jets (only below `JET_ARM_ALT`, where jets are worth their fuel),
  puts RCS back on (at 40 m/s the control surfaces have nothing to bite) and
  dives until the wing is back. It is back when the pressure has returned **and**
  the nose is within `RECOVER_NOSE` of the airstream, and the turn demand stays
  off for `STEADY_TIME` after that — a recovery that ends on the pressure gauge
  alone hands the guidance law a ship at exactly the speed that just failed, and
  the next turn command departs it again. See `docs/GLIDE_REVIEW.md`.
* **A wing flies on pressure, not on a number.** Every target speed below the
  entry is `GLIDE_Q` converted into an airspeed in the air the ship is actually
  in. A fixed 160 m/s target is 0.037 atm at 12 km — below this script's own
  minimum for a banked turn — and commanding it is what made the glide stall,
  spin, recover and stall again the whole way down.

* **Entry** holds `REENTRY_AOA` while there is orbital energy to throw away and
  tapers it toward the glide angle between `ENTRY_AOA_HI` and `ENTRY_AOA_LO`
  airspeed. Holding 40° all the way down balloons the ship back out, leaves it
  at the top of the atmosphere with no speed, and drops it. The nose leans toward
  the KSC only as far as the yaw schedule allows. Entry also owns the **range**
  problem, because that is where the energy to solve it is: it prices
  `rangeCapability()` — energy height × the L/D it will average shedding it,
  `ENTRY_LD` while hypersonic — against the range still to fly, and holds more
  alpha (alpha is drag), puts the boards out and S-turns when long, gives all
  three up to stretch when short. The L/D is not taken on trust: the entry
  measures the range it is actually buying per metre of energy height and never
  credits itself with more than it is getting, because a nominal model that
  credits a slowing ship with a rising L/D at the rate the extra drag is
  destroying it reads "long" all the way to the point where it is 292 km short.
  Nor is the *attitude* taken on trust — alpha the airframe cannot hold is capped
  back to what it can, and a nose that lets go of the airstream is recovered
  rather than logged. Inside `ENTRY_HOLD_RNG` it stops chasing a bearing that swings through
  180° as the field passes underneath. Entry ends at `ENTRY_END_SPD`;
  `ENTRY_FLOOR` is a backstop, not an alternative — handing a Mach 3 ship to a
  glide law is what the old "whichever comes first" rule did, and it cost a ship.
* **Glide** aims at a *final approach fix* `FINAL_DIST` short of the threshold on
  the extended centreline, not at the threshold itself — a bearing-only homing
  law arrives overhead still high and pointing anywhere. The nose flies a **sink
  rate** — the glide ratio the energy error asks for, `LD_DUMP` steep when high
  and `LD_STRETCH` shallow when short, `PLAN_LD` on profile — with the speed
  error trimming it. A nose that only ever holds a speed holds no flight path at
  all, and a ship handed that command phugoids between a 45° dive and a level
  cruise. Drag and track length spend the rest: airbrakes above `BOARD_MARGIN`
  and ± `STURN_OFFSET` S-turns above `HIGH_MARGIN`, both only with `MANEUVER_Q`
  of air under the wing, and inside `HOLD_RADIUS` excess height is spiralled off
  over the field. With `USE_JETS_SHORT` set the RAPIERs go back to
  **air-breathing** and push — the throttle holds the speed, the nose goes on
  holding the flight path. Thrust needs a real energy deficit, a wing that has
  genuinely stopped flying, or the ground: a ship that is merely slow and high is
  answered by pointing the nose down, not by burning the reserve to hold it up
  there.
* **Rationing.** The fuel is budgeted twice over, because thrust on final cannot
  be substituted for by anything and thrust at 60 km out usually can.
  * **The arming line is the stretch line, not the plan line.** Being below the
    `PLAN_LD` profile is not the same as being unable to get there — flown at
    `LD_STRETCH` the same energy covers a third more ground, which is 4.4 km of
    energy height at 80 km out and 370 m at 10 km. That difference is the
    airframe's own reserve, and the nose gets first refusal on it: it shallows
    toward `LD_STRETCH` by itself, and the jets light only when even that will
    not reach. The line converges on the plan as the range closes, so the glide
    is free early and the fuel is there late.
  * **`JET_RESERVE_FRAC` of the fuel aboard at handover is the approach's**, and
    the glide cannot reach it. `finalApproach()`, `emergencyLanding()` and
    `stallRecover()` can — they are the cases where thrust has no substitute.
    The closing report says whether the reserve survived.
* **Terrain**, in two layers, because they answer different questions. Every
  number in the energy plan is referenced to the runway, and height above the
  runway is not height above the ground — the approach from the west crosses
  ridges over 2 km high.
  * **Predictive.** The glide samples the ground at `MTN_SAMPLES` points all the
    way to the fix and raises a **deck** at the highest of them plus `MTN_CLEAR`.
    The deck bounds the sink command *and* raises the energy profile to whatever
    holding it costs, so a ship that cannot clear the ridge reads short 60 km out
    and lights the jets while that still means something. This is the layer that
    flies the approach: a glider does not climb over a ridge, it arrives already
    above it.
  * **Reactive.** `ALT:RADAR` — the ground directly underneath — bounds the sink
    on the same construction (`TERRAIN_FLOOR`, bled off over `TERRAIN_TAU`, a
    commanded `TERRAIN_CLIMB` below it), floors every stall recovery, and below
    `TERRAIN_ABORT` ends the glide outright. On its own it is far too late: at
    190 m/s a ridge face is eight seconds wide.

  Both outrank the plan, because the plan cannot see. The one thing still gated
  on the deck is the S-turn — the scan runs along the direct track, so 40° off it
  is 40° of ground nobody measured. The boards are not: the deck is already
  inside the profile, so a ship reading long is long *after* paying for the ridge.
* **Final** flies nose-for-speed, boards-for-glideslope — the way a glider is
  flown — with a localiser term (`LOC_GAIN`, `LOC_MAX`) closing on the
  centreline, wings level below 120 m, then the flare at `FLARE_ALT`.
* **Divert.** The glide's `GLIDE_FLOOR` is not a runway capture. Out of height
  more than `DIVERT_RANGE` from the field, the script says so and flies a
  wings-level landing straight ahead, because banking a low, slow ship toward a
  runway it cannot reach arrives inverted rather than merely somewhere else. It
  is a landing, so it descends: thrust buys speed only, and only while the ship
  is going down. The closing report reads `SHIP:STATUS`: it prints `STOPPED ON THE RUNWAY` only when
  that is what happened, and `DITCHED` or `DOWN AND STOPPED n km from the runway`
  when it is not.

Ranges are measured as **ground range**, never `geo:DISTANCE`: the latter is
3-D and dominated by altitude, so at 30 km overhead the runway reads "30 km
away" and any range test using it fires at the wrong moment.

The script raises `CONFIG:IPU` to 500 so the control loop stays crisp, and zeroes
the pilot throttle before handing back — after an ascent the manual throttle is
usually still at 100%, and the jets are back on air by then.

Both scripts expose their tunables at the top — trim
`ROTATE_SPEED`, `DV_MARGIN`, `DV_GLIDE_RESERVE`, `DEORBIT_LEAD`, `WARP_LEAD`,
`GLIDE_Q`, `FLARE_ALT`, etc. to taste. `DEORBIT_PE` appears in *both* scripts
and should match: the ascent script reserves the ΔV that the deorbit script will
spend.

The ascent tunables worth knowing if a flight goes wrong:

| Tunable | Default | What it does |
|---|---|---|
| `AB_ACC_LOW` / `AB_ACC_HIGH` | 1.2 / 3.0 m/s² | The acceleration deadband the air-breathing climb rate is traded against. Raise `AB_ACC_LOW` to fly flatter and reach the mode switch faster. |
| `SW_ARM_SPEED` | 1150 m/s | No thrust/acceleration sensing below this, so the jets are never given up early. |
| `SW_SPEED_HARD` | 1600 m/s | Hard backstop on the mode switch. |
| `CC_FPA_HI` / `CC_FPA_LO` | 15° / 0° | Rocket-phase flight-path schedule, tapering as apoapsis approaches target. |
| `CC_ACC_LOW` | 1.5 m/s² | Below this the rocket phase shallows out rather than fighting gravity. |
| `DV_GLIDE_RESERVE` | 120 m/s | The floor the ascent will never burn through. |
| `EFF_WINDOW_DV` | 120 m/s | Rocket ΔV per climb-efficiency window. The measured loss factor describes the last window only, not the burn so far — see `docs/CLIMB_LOSS_REVIEW.md`. Shorten it for a twitchier reading, lengthen it for a steadier one. |
| `EFF_VERDICT_P` | 0.005 | Ambient pressure, as a fraction of sea level, below which the rocket phase is allowed to declare no orbit reachable. ≈33 km on Kerbin. Raise it only if you want that call made earlier and on worse evidence. |
| `EFF_MIN_WINDOWS` | 2 | Windows of evidence needed before that call, and the number of consecutive dead windows that override the pressure gate. |
| `CC_DEAD_CONFIRM` | 4 | Consecutive no-thrust samples (0.5 s apart) before the rocket phase decides the engine has stopped and ends the climb. |
| `PLAN_SWITCH_ALT` / `PLAN_SWITCH_SPD` | 20 km / 1450 m/s | Handover state the pre-flight check assumes. Trim to what your ship actually achieves — the script prints it at the real switch. |
| `PLAN_JET_DV` | 3000 m/s | Jet-phase ΔV equivalent, incl. drag. Sets how much LF the check expects the jets to burn. |
| `PLAN_LOSS_FACTOR` | 1.65 | Whole-climb gravity/drag/steering losses from the handover, pre-flight only. Measured ×1.66 end-to-end on this airframe. Do **not** set it from the in-flight window figure (×1.31) — that describes the cheap end of the climb. See `docs/CLIMB_LOSS_REVIEW.md`. |
| `CC_AOA_HIQ` | 6° | Angle of attack the rocket phase may present to the flow while `SHIP:Q` is above `CC_AOA_Q_HI`. The reference flight flew the opening at 16.4° and paid 2335 kN of drag for it. Set to `AOA_TRIM_MAX` (14°) to restore the old behaviour. |
| `DEPLOY_BEFORE_DEORBIT` | `TRUE` | Prices the deorbit reserve at the mass the burn is actually flown at — after the payload is released. The same propellant is worth ~1.9x on the light ship, and reserving the light-ship figure against the heavy-ship gauge sets aside twice what the burn uses. `FALSE` reserves enough to deorbit with the cargo still aboard. |
| `ISOLATE_PAYLOAD` | `TRUE` | Keeps the payload's propellant out of the ΔV budget and out of the engines. `FALSE` lets the ascent drink it — on the Mk3 Titan that is 1491 → 3168 m/s at the handover, and 34 t of cargo that does not arrive. |
| `PLAN_TWR_MIN` | 1.05 | Closed-cycle TWR the sizing advice tries to hold at handover. |
| `PREFLIGHT_HOLD` | 12 s | Pause on a failed check so you can read it. 0 to skip. |
| `ABORT_IF_INFEASIBLE` | `FALSE` | `TRUE` cuts the burn the moment orbit is priced out of reach, keeping the most fuel for a return. |

The landing tunables worth knowing if a reentry goes wrong:

| Tunable | Default | What it does |
|---|---|---|
| `ENTRY_RANGE` | 800 km | Ground range from the atmospheric interface to the KSC — **the** number to trim, and the one the whole entry is about. The ship makes ~880 km from the interface flown clean and ~530 km flown at full alpha with the boards out, so 800 km sits mid-band with dumping authority in hand either way. Overflying the field by *n* km means raising this by about *n*; landing short by *n* means lowering it. The report says which. |
| `DEORBIT_LEAD` | 0 (solved) | Ground-track angle before the KSC at which the burn is made. Left at 0 it is solved from `ENTRY_RANGE` plus the coast arc the deorbit ellipse sweeps between the burn and the interface — 73.1° on a 100 km orbit, but 63° from 90 km and 85° from 120 km, which is why a hand-trimmed angle silently stops being right when the parking altitude or `DEORBIT_PE` changes. Set it non-zero to force one. See `docs/GLIDE_REVIEW.md`. |
| `REENTRY_AOA` | 40° | Alpha held while hypersonic. Lower it if the ship balloons back out of the atmosphere. |
| `ENTRY_AOA_HI` / `ENTRY_AOA_LO` | 2000 / 500 m/s | Airspeeds the entry AoA is tapered between. Raise `ENTRY_AOA_LO` if the ship is still fast when the glide starts. |
| `ENTRY_END_SPD` / `ENTRY_FLOOR` | 650 m/s / 15 km | Handover to the glide. The **speed** is the handover; the altitude is only a backstop. Do not turn this back into "whichever comes first". |
| `ENTRY_LD` | 3.0 | Ground range flown per metre of energy height while hypersonic — measured, not the glide ratio. The *nominal* figure only: the entry measures what it is actually getting (printed as `L/D`) and never credits itself with more than that. |
| `LD_SAMPLE` / `LD_FILTER` | 5 s / 0.35 | How often the flown L/D is measured and how hard each sample is believed. |
| `ENERGY_LONG` / `ENERGY_SHORT` | 1.15 / 1.00 | Range-capability ratios at which the entry starts dumping energy (boards, alpha, S-turns) or stretching. The ratio is printed every 5 s as `energy`. |
| `ENTRY_HOLD_TOL` / `ENTRY_TRIM_TOL` | 25° / 12° | How far past the commanded alpha the nose may actually sit before the entry calls it departed, and before it caps the alpha it asks for. |
| `GLIDE_AOA_MAX` / `FAST_AOA_MAX` | 14° / 8° | Hard stall guard, and the tighter ceiling above `FAST_SPD` where 14° is not a stall risk but a lift spike. No phase is allowed past either. |
| `STALL_Q` / `MANEUVER_Q` / `RECOVER_Q` | 0.025 / 0.045 / 0.050 atm | Dynamic pressure at which the wing is considered gone, at which full bank and energy-dumping are allowed, and which a recovery flies back to. Pressure, not airspeed: 76 m/s at 9 km and 76 m/s over the runway are not the same flight state, and any airspeed threshold that catches the first is above the 110 m/s this ship flies its approach at. |
| `DEPART_AOA` | 30° | Actual nose-to-airstream angle in the glide that counts as departed, held for 3 s. |
| `RECOVER_NOSE` / `STEADY_TIME` | 20° / 6 s | A recovery ends when the wing has air **and** the nose is following the airstream, and no turn is demanded for `STEADY_TIME` afterwards. Ending on pressure alone is how one stall becomes six. |
| `COAST_WARP_MIN` / `COAST_LEAD` | 60 / 30 s | The coast from the deorbit burn to the interface is solved and warped away when it is longer than the first; warp ends the second before the air. |
| `JET_ARM_ALT` | 12 km | Ceiling for a jet save. Below it the jets light on low pressure under the wing *or* on being below profile. |
| `YAW_SPD_HI` / `YAW_SPD_LO` / `YAW_SUB_MAX` | 1200 / 250 m/s / 15° | The **sideslip** schedule: pinned to `ENTRY_YAW_MAX` above the first, opened to `YAW_SUB_MAX` below the second. The bank commanded from the same heading error is what turns the ship, so this can be tight without costing turn rate. At 60° it was not a bound at all — a 40° S-turn demand departed the ship at 58° off the airstream. |
| `GLIDE_Q` | 0.07 atm | The dynamic pressure the glide is flown at. **This is the glide speed** — constant q is constant lift coefficient, which is best-glide at every altitude: 110 m/s over the runway, 160 at 6 km, 250 at 12 km, 450 at 20 km. |
| `GLIDE_SPD_MIN` / `GLIDE_SPD_MAX` | 110 / 450 m/s | The ends of the range that answer is allowed to come out in. |
| `APPR_SPEED` | 110 m/s | Target airspeed on final, where `GLIDE_Q` and 110 m/s are the same thing. |
| `LD_DUMP` / `LD_STRETCH` | 1.5 / 6.0 | The glide ratios the nose is asked to *fly* at `HIGH_MARGIN` above and `LOW_MARGIN` below the profile, interpolated through `PLAN_LD` on it. This is what makes the glide track a profile instead of phugoiding around one. |
| `GLIDE_VS_GAIN` / `GLIDE_VS_AUTH` | 0.15°/m/s / 6° | Sink-rate loop gain, and the most AoA it may add or take. Raise the gain for tighter path tracking, lower it if the nose hunts; the stall guard still outranks both. |
| `BOARD_MARGIN` / `BOARD_OFF` | 500 / 150 m | Energy height above profile that puts the boards out, and below which they come back in. Narrower than `HIGH_MARGIN`: drag is the cheapest energy dump there is, so it is spent first and the S-turn is held back. |
| `THROT_PER_MS` / `THROT_MIN_PWR` | 0.02 / 0.25 | Powered speed loop: throttle per m/s of error, and a spool-up floor held only while genuinely below the arming line or low over the ground. There is no floor for a ship that is slow and *high*. |
| `JET_RESERVE_FRAC` | 0.35 | Fraction of the LiquidFuel aboard at glide handover that the glide may **not** spend. It belongs to the final approach, the flare and any late correction — the parts of the flight where thrust has no substitute. Raise it if the ship keeps arriving at the fix with nothing left; lower it if it lands with the reserve untouched *and* the glide was fighting to reach the field. `JET_MIN_LF` (5) is the hard floor underneath, and the emergency paths can reach past the reserve. |
| `MTN_CLEAR` / `MTN_SAMPLES` / `MTN_SCAN_T` | 350 m / 20 / 2 s | Predictive terrain. The ground is sampled at `MTN_SAMPLES` points between the ship and the fix; the highest plus `MTN_CLEAR` is a **deck** the glide may not descend through, and the energy profile is raised to whatever holding it costs. Raise `MTN_CLEAR` if the ship skims ridges; raise `MTN_SAMPLES` if it clips one between samples (20 samples over 90 km is a point every 4.5 km). |
| `TERRAIN_FLOOR` / `TERRAIN_TAU` / `TERRAIN_CLIMB` / `TERRAIN_ABORT` | 500 m / 12 s / 8 m/s / 250 m | Reactive terrain, on `ALT:RADAR`. The sink command is bled to nothing as the ship closes on the floor and becomes a climb below it; `TERRAIN_ABORT` ends the glide. Last guard only — it cannot see a ridge until the ridge is underneath. |
| `AOA_PER_MS` | 0.05°/m/s | Speed-loop gain. Raise for a tighter speed hold, lower if the nose hunts. |
| `PLAN_LD` | 4.5 | Glide ratio the energy plan assumes. Too optimistic and the ship lands short; too pessimistic and it S-turns the whole way home. The glide profile it defines is compared against the ship's **energy height**, not its altitude — a ship at 5 km doing Mach 2 is long, not low. |
| `HIGH_MARGIN` / `LOW_MARGIN` | 1500 / 400 m | Deadband either side of the profile before energy is dumped or the jets are lit. |
| `HOLD_RADIUS` | 8 km | Inside this, excess height is spiralled off over the field instead of flown off in S-turns. |
| `FINAL_DIST` | 9 km | How far short of the threshold the final approach fix sits. |
| `BANK_MAX` / `BANK_PER_DEG` | 30° / 1.5 | Turn authority and how hard heading error is banked on. Both are still subject to the speed schedule. |
| `GLIDE_FLOOR` / `DIVERT_RANGE` | 600 m / 12 km | AGL at which the glide must hand over, and how far from the runway that has to be before it is flown as a divert rather than an approach. |
| `PITCH_MIN` | −85° | Absolute nose-down backstop. It must stay clear of a recovery dive along a near-vertical flight path — at −60 it silently ate the AoA guard and held the ship in the stall. |
| `USE_JETS_SHORT` | `TRUE` | `FALSE` keeps the return a pure glide and accepts landing short. |

### Before you edit a script

```
python3 tools/check_kos.py scripts/*.ks
```

kOS compiles a script only when you `RUN` it — mid-flight — and it stops at the
**first** error, so a typo costs a reload and a rollout, and you find the next
one the same way. The checker catches what can be seen from the text: unbalanced
braces, calls to undefined functions or with the wrong argument count, and
**built-in shadowing**.

That last one has bitten this repo twice. kOS refuses to compile a script that
declares a variable hiding one of its built-in functions, and it isn't only the
one-letter ones — `QUEUE`, `STACK`, `LIST`, `RANGE`, `NODE`, `PATH`, `BODY` and
a long tail of ordinary words are built-ins too, so `LOCAL queue IS LIST().`
will not compile. Assigning a *bound* variable is fine (`SET WARP TO 0.` is the
documented way out of time warp); declaring one over it is not.

Writing `rendezvous.ks` walked straight into the same trap from a new direction:
`TRANSFER`, `VESSEL`, `NODE` and `TARGET` are every one of them words you reach
for while writing a rendezvous, and all four are taken. Run the checker.

One thing the checker **cannot** see, and `dock.ks` is full of reasons to care:
do not assume `AND` and `OR` stop evaluating once the answer is known. A guard
written as `IF thing:HASSUFFIX("X") AND thing:X > 0` is not a guard at all if
the right-hand side is evaluated anyway — and the whole point of asking
`HASSUFFIX` is that reading the suffix might not be safe. Nest the `IF`s
instead. Every suffix in `dock.ks` that a given kOS version might not have is
reached that way.

---

## Manual flight (no kOS)

### Ascent

1. **SAS on**, brakes off, throttle 100 %, RAPIERs in **air-breathing** mode.
2. Roll to ~80 m/s, gently pull to **~10–12° pitch**, lift off, gear up.
3. Hold a shallow climb and **build speed**. Around 400–500 m/s you'll punch
   through transonic — keep the nose low until you're supersonic, then resume
   climbing.
4. Ride the jets up. When thrust starts to sag (~20 km, ~1400 m/s), **switch
   RAPIERs to closed cycle** (Action Group 1) and pitch up to ~20°.
5. Fly a normal gravity turn, keeping apoapsis climbing toward **100 km**. Cut
   throttle when apoapsis = 100 km.
6. Coast to apoapsis, then **burn prograde** to raise periapsis to ~100 km.

### Reentry & landing

1. In orbit, **close the docking-port shield** (AG 3), **retract antennas**
   (AG 4), stow any cargo.
2. Over the far side of Kerbin from the KSC, burn **retrograde** to drop
   periapsis to ~30–35 km.
3. Reentry: hold **~40° angle of attack** (nose well above retrograde), belly
   into the airstream, brakes on for extra drag. Keep it pointed generally
   toward the KSC.
4. Below ~30 km and subsonic-ish, fly it like a glider: aim for the runway,
   **best glide ~150–160 m/s**. If you're high/fast, use airbrakes and gentle
   S-turns to burn energy.
5. Line up with **runway 09** (heading 090), fly a shallow (~5°) glideslope,
   **gear down**, flare at ~20–30 m, touch down, **brakes on**, keep straight,
   roll to a stop.

---

## Payload & docking notes

* **Cargo**: up to **100 t** in the two CRG-100 bays. Keep the payload CoM near
  the ship's CoM (roughly over the main gear). Secure long stacks so they don't
  shift.
* **Docking**: the shielded port is on the **top** of the ship, behind the
  cockpit. Open the shield (AG 3), enable **RCS (R)**, approach your station's
  port from below/alongside, and dock. The 8 RV-105 blocks + monopropellant give
  full translation control.
  `RUN rendezvous.` parks a few hundred metres out with the relative velocity
  dead and RCS on; `RUN dock.` takes it from there to a hard dock, once you have
  targeted the station's port and set Control From Here on ours. Close the
  shield again (AG 3) before reentry.
* **Power**: the 4 RTGs supply power indefinitely; the Z-4K banks cover peaks.
  You can leave the ship parked without worrying about batteries.
