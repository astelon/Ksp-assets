# Rendezvous — how `scripts/rendezvous.ks` meets a station

This is the design note for the third flight script. `ascent.ks` puts the ship
in orbit and `deorbit_land.ks` brings it home; this one takes the orbit ascent
delivered and turns it into a **parking spot a few hundred metres off another
vessel**, with the relative velocity dead and the nose on the target.

It deliberately stops there. Docking needs a chosen port at both ends and a
different control law, and that belongs in its own script.

```
RUN rendezvous.                    // whatever is selected as target on the map
RUN rendezvous("Station Alpha").   // or by name - a substring is enough
RUN rendezvous("Alpha", 250).      // ...and a park distance, in metres
```

---

## 1. The problem is timing, not steering

Getting into the target's orbit is easy and almost useless. An orbit is a place
you are *at a time*, and two ships in the same orbit half a lap apart are as far
from a rendezvous as it is possible to be — they will stay exactly that far
apart forever, because they are going the same speed.

Everything in the script exists to control **when** the ship arrives. The burns
that do it are ordinary Hohmann burns; the difficulty is entirely in choosing
the moment to start one.

The classic transfer needs the target sitting at a particular **lead angle**
when we leave:

```
phi_required = 180 deg  -  omega_target * t_transfer
t_transfer   = pi * sqrt(a_transfer^3 / mu)      a_transfer = (r1 + r2) / 2
```

and that angle comes round at a rate set by the difference between the two
orbital periods:

```
d(phi)/dt = omega_target - omega_ship
```

**Which is the whole difficulty.** `ascent.ks` parks at 100 km. A station at
100 km has *no* period difference from us, so `d(phi)/dt` is zero, the lead
angle we need never arrives, and a script that simply waits for it waits until
the heat death of the save file.

## 2. So the phasing orbit is solved, not picked

The script does not wait. It sweeps candidate phasing radii and prices each one
— the two burns to get in and out, against the wait it produces:

| For each candidate radius `rP` | |
|---|---|
| Cost to get there | `hohmannDv(r_ship, rP)` |
| Time to get there | `hohmannTime(r_ship, rP)`, during which we sweep exactly 180° and the target moves `omega_t * t` |
| Lead angle on arrival | `phi_now + omega_t * t_move - 180` |
| Wait there | `(phi_required - phi_arrival) / (omega_t - omega_P)`, wrapped to be positive |
| Cost to leave | `hohmannDv(rP, r_target)` |

Then it takes the **cheapest candidate whose total time fits `MAX_TOTAL_TIME`**.
Our current radius is in the candidate list, so "just wait where we are" is one
of the options and wins whenever it is affordable in time — which, against a
station in a genuinely different orbit, it usually is.

Dropping to a lower orbit to catch up is therefore a decision with a number
attached, and the script prints the number.

The two knobs that matter:

* **`MAX_TOTAL_TIME`** (default 2 h). Patience is cheap and hurry is expensive.
  Raising it lets the solver find plans that cost far less ΔV.
* **`PHASE_R_SPAN`** (default 0.35). How far from the target's radius the search
  may roam. A phasing orbit's periapsis is never allowed inside the atmosphere
  plus `PE_SAFETY`.

### Why the low-target case gets expensive

For a station in a *low* orbit there is not much room underneath it to phase
from, so the solver goes *above* instead and lets the target catch up. Against
an 80 km station from a 150 km orbit it will happily spend 390 m/s to do it in
two hours — and about a quarter of that given six. If a plan looks dear, the
first thing to try is more time.

## 3. Plane matching does not go through a maneuver node

Relative inclination is measured frame-free, as the angle between the two orbit
normals (`VCRS(position, velocity)` for each ship). The relative node line —
where the two planes cross — is the cross product of those two normals.

The burn itself is flown as a **vector**, not as a node's normal component. KSP's
coordinate system is left-handed and the sign convention for a node's normal
axis is exactly the kind of thing that is right on one install and mirrored on
the next; a plane change that comes out backwards *doubles* the relative
inclination instead of removing it, at full price. So `planeBurnVec()` rotates
the velocity vector about the radius vector — tries it both ways round, and
keeps whichever one actually points our orbit normal at the target's.

The burn then terminates on **the thing it is for**: relative inclination
itself, stopped the moment it stops falling. Burning to a fixed ΔV would trust
the arithmetic; this checks it.

Below `PLANE_SKIP_DI` (0.05°) the whole phase is skipped. `ascent.ks` launches
due east, so against an equatorial station the relative inclination is usually
already negligible, and 3 m/s spent chasing a hundredth of a degree is worse
than the hundredth of a degree.

## 4. The closed form plans, a numerical search aims

Vis-viva assumes two circular coplanar orbits and an impulsive burn. The real
problem is none of those things: the target's orbit is slightly eccentric, the
plane match left a residue, and the burn takes half a minute.

So the transfer node is **planted** from the closed form and then **refined**
numerically. kOS's `POSITIONAT` accounts for planned maneuver nodes on the
active vessel, so the script can nudge the node and read the closest approach it
would actually produce. It hill-climbs on prograde, normal, and node time with a
halving step until the miss is inside `INTERCEPT_DIST` or the step runs out.

Two details that are easy to get wrong and are handled explicitly:

* A node's `ETA` is a **countdown, not a timestamp**. It shrinks while the search
  runs, so restoring a rejected trial by writing the old ETA back would quietly
  walk the node later on every round. The wanted burn time is held as an absolute
  UT and the countdown derived from it each time it is written.
* The refinement is **guarded, not trusted**. Before using it the script plants a
  throwaway node and checks that the prediction actually moves. If it does not,
  the closed-form plan flies unrefined and the mid-course correction — measured
  off the real post-burn orbit, with no node involved — cleans up behind it.

### Aiming to miss

`INTERCEPT_DIST` is 2 km, not zero. Two conics that both ships are only
approximately on will not deliver a zero-mile pass, and a plan that asks for one
buys a fast crossing with no time to brake. Aiming a little wide costs nothing —
the arrival burn nulls the relative velocity either way — and it leaves room to
be wrong in.

## 5. The approach speed is what the ship can stop from

The terminal phase closes at no more than

```
v_close  <=  sqrt( 2 * a_rcs * (distance - PARK_DIST) / BRAKE_SAFETY )
```

where `a_rcs` is the acceleration the RCS **actually** produces on the mass
**actually** aboard. A loaded Mk3 with 100 t in the bays has a fraction of the
translational authority it has empty, and it gets a proportionally gentler
approach without anyone retuning anything.

The estimate starts from the installed block count and is then trimmed in flight
from what the thrusters really deliver — but only sampled while the last interval
was spent at *full* command. Acceleration measured through a half-throttled
correction is half the authority the ship has, and feeding that back would
ratchet the approach slower and slower, each sluggish sample justifying the next.

One command covers closing, braking and lateral nulling: fly the velocity we
want to have, minus the one we have got, projected onto whichever thrusters
point that way.

```
v_want = line_of_sight * v_cmd
v_err  = v_want - v_relative
FORE      = v_err . facing:forevector  * gain
STARBOARD = v_err . facing:starvector  * gain
TOP       = v_err . facing:topvector   * gain
```

Past the park point the sign flips and the same law backs the ship out again, so
a drift inward is station-keeping rather than a slow collision nobody commanded.

### The guard that stops a hot arrival

**This one was found by simulation, not by inspection**, and it is the most
important safety property in the script.

The braking law above is correct, but correct is not the same as sufficient: a
ship that reaches the terminal phase still closing at 12 m/s with 0.03 m/s² of
RCS needs 2.4 km to stop and has 1.8. The thrusters brake at full authority the
whole way down and it flies into the station anyway.

Two things fix it:

1. **The handover speed is computed, not fixed.** The main-engine arrival burn
   drives the relative velocity down to
   `min(RCS_HANDOVER, sqrt(2 * a_rcs * gap / BRAKE_SAFETY))` — whatever the
   thrusters can actually brake away in the distance that will be left.
2. **The main engine comes back on if it has to.** If `v_close² > 2·a·gap` the
   ship cannot stop on RCS however finely it is flown, so `brakeHard` is set, the
   nose comes round onto the braking vector, and the main engine is used. It
   comes off again with hysteresis, once there is real room in hand, so the two
   modes cannot chatter.

## 6. It never burns dry

Same rule `ascent.ks` flies by, for the same reason. Every burn is policed
against a reserve that funds the deorbit and the glide home, and no manoeuvre
may breach it. A ship parked next to a station with no way home has not
completed a rendezvous.

The reserve is priced at **the mass the deorbit burn will actually be flown at**.
On a station run the cargo comes off at the station, so the deorbit happens on a
ship that can be half the mass on the gauge now, where the same tonnes of
propellant are worth far more. Reserving the post-delivery figure against the
pre-delivery gauge sets aside roughly twice the propellant the burn will use —
enough to price a perfectly reachable rendezvous out of reach. `DEPLOY_BEFORE_DEORBIT`
is what selects this; set it `FALSE` for any flight that might bring the payload
home again.

**Monopropellant is a second, independent currency.** The terminal approach is
flown on RCS, and running the mono dry a hundred metres off a station leaves the
ship drifting with no way to stop. It is budgeted and reported separately, and
the approach stops at `MONO_RESERVE` and holds rather than spending the last of
it.

## 7. Payload fuel is not our fuel

The same part-tree walk `ascent.ks` does — flood-fill out from the engines,
refusing to cross anything that can separate — and it matters *more* here. On a
station run the payload very often **is** propellant, and counting the cargo's
tanks as budget would fund a rendezvous with fuel that is supposed to be left
behind.

---

## Verification

The script cannot be run outside KSP, so the two pieces of it that are pure
arithmetic were checked outside the game before the first flight.

**The phasing solver** was reimplemented in Python exactly as the script
computes it, and the plans it produced were then flown by an independent
propagation of both vessels. Across same-orbit, higher-target and lower-target
geometries at eight starting phase angles, every plan put the two vessels in the
same place at the same time to within **0.0000°**. That is what validates the
sign conventions in `phaseAngleAt()` and `phaseWaitFor()`, which are the two
places a rendezvous script silently sails past its target.

**The terminal approach law** was simulated as a point mass with per-axis RCS
saturation, from nine starting states spanning heavy and light ships, long and
short ranges, lateral drift, and hot arrivals. The first run **found a real
collision**: the 12 m/s hot arrival on a heavy ship reached 9.8 m from the
target. That is what produced the computed handover speed and the `brakeHard`
main-engine guard in §5. After those changes all nine cases park cleanly, and
the closest any of them comes to the target is 100 m — in the one case that
*starts* inside the park distance and backs out.

Neither of these is a substitute for flying it. What they establish is that the
geometry and the control law are right, which is the part that cannot be trimmed
out afterwards by nudging a constant.

## What to trim after the first flight

As with the other two scripts, the constants are grouped at the top and these
are the ones a real flight will have opinions about:

| Constant | What it decides | Symptom that it is wrong |
|---|---|---|
| `MAX_TOTAL_TIME` | How patient the solver may be | Plans cost far more ΔV than they should |
| `INTERCEPT_DIST` | What the transfer aims for | Arrivals are consistently long or hot |
| `BRAKE_SAFETY` | Padding on the stopping distance | Approach feels sluggish, or arrives too fast |
| `V_CLOSE_MAX` | Cap on closing speed | Approach takes forever from long range |
| `RCS_THRUST_BLOCK` | Starting guess at translational authority | First approach is badly mis-paced (it self-corrects after a few seconds) |
| `PARK_DIST` | Where it stops | Too close or too far for your docking script |
| `CORR_DV_ALLOW` | Budgeted correction allowance | Budget verdict is consistently pessimistic |

`LOG_TO_FILE` appends every resource snapshot to `0:/rendezvous_log.csv` — mass,
LF, Ox, mono, EC, and both ΔV currencies, at pre-flight, at intercept, and
parked. That is the raw material for the same kind of measured review the other
constants in this repository came from.
