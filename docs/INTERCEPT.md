# Intercept — how `scripts/intercept.ks` launches at a moving target

This is the design note for the script that ties the other four together. It
sits on the runway with the brakes on, waits for the minute that makes the rest
of the mission cheap, and then flies the whole thing: `ascent.ks` to orbit,
`rendezvous.ks` to the doorstep, `dock.ks` to a hard dock.

```
RUN intercept.                          // whatever is selected on the map
RUN intercept("Station Alpha").         // or by name - a substring is enough
RUN intercept("Alpha", 250).            // ...and a park distance, in metres
RUN intercept("Alpha", 200, "dorsal").  // ...and which of our ports is ours
```

Select the target on the map before running it. Selecting a **docking port**
rather than the vessel works too, and is better: the script keeps that port all
the way through and hands it to `dock.ks` at the far end instead of choosing a
berth itself.

---

## 1. The wait is the whole script

Everything after the countdown is three scripts that already exist, run in
order, each of which prices, refuses and reports for itself. What this one adds
is **the launch time** — and against a target in orbit that is worth more than
any amount of clever flying afterwards.

The reason is that an ascent is not steerable in the dimension that matters.
`ascent.ks` takes about twenty minutes and puts the ship at a particular place
on the clock; the station is wherever its own orbit has taken it by then, and
nothing the autopilot does on the way up changes that. Launch at the wrong
minute and the ship arrives in a perfectly good orbit half a lap behind the
target, and then pays for it — hours of waiting, or a phasing orbit's worth of
propellant, or both.

Launching at the *right* minute costs nothing except the waiting, and the
waiting happens on the ground under time warp where it is free.

Simulated over 600 target geometries (see [Verification](#verification)):

| | Median drift left to spend in orbit | Flights where the transfer point had already gone past |
|---|---|---|
| Launch when the pilot happens to type the command | 185 min | **54 %** |
| Launch in the window this script solves | 66 min | **0.8 %** |

That second column is the one that matters. "The transfer point has gone past"
means the lead angle the Hohmann transfer needs is now behind the ship, and the
next one is a full synodic lap away — **twelve hours** for a parking orbit 20 km
under a station. `rendezvous.ks` will not sit through that; it will buy a
phasing orbit instead, and that is propellant spent to fix a launch time.

## 2. What is aimed at is the phase at insertion, not the phase at launch

The quantity being controlled is where the target will be when we **arrive in
orbit**, not where it is when the brakes come off. So the script models the
climb as two numbers,

```
ASC_TIME   how long ascent.ks takes, runway to circular orbit
ASC_ARC    how far round the body it gets in that time, measured in inertial
           space from where the launch site was at T-0
```

propagates the target with kOS's own `POSITIONAT`, and searches for the launch
time `t` at which

```
phase( insertion_position(t),  target_position(t + ASC_TIME) )  =  phase_wanted
```

`ASC_ARC` is an *inertial* arc, not a ground track: it already contains the
body's own rotation during the climb. On Kerbin an SSTO covers something like
165° of it, which is a reminder of how small Kerbin is — the ship is nearly half
way round the planet by the time it circularises.

Both numbers are **measured on every flight and written to disk**
(`0:/intercept_cal.json`, keyed by body and parking altitude band, kept as a
running mean over the last few flights). The seeds in the script are only ever
used once. The measured values are printed at the end whether the file could be
written or not, so they can be typed in by hand.

**The root is a division, not a search.** The error is a straight line in the
launch time — our insertion point goes round at the body's rotation rate, the
target at its own, and the difference between them is a constant. So the script
measures that sweep rate over a 30-second baseline of the very function it is
about to solve, and divides:

```
t_window = t_start - error(t_start) / sweep      brought forward a lap at a time
```

then **verifies** the answer (an eccentric target does not sweep quite
uniformly) and refines it locally if it needs to. Only if that fails does it
sweep the horizon, and then with a step capped at a 24th of a lap so it cannot
step over a crossing.

Measuring the rate rather than assuming it is what makes the cross-check in §6
possible, and that check has already earned its place — see [What a flight
found](#what-a-flight-found).

**The window recurs quickly.** The phase error sweeps through 360° at the
difference between the target's orbital rate and the body's rotation rate —
about 0.17 °/s against a 100 km station on Kerbin. So:

| Target altitude | Window recurs about every |
|---|---|
| 100 km | 36 min |
| 250 km | 46 min |
| 400 km | 62 min |
| 2000 km | 10 h |
| 2863 km (keostationary) | **never** — see below |

At synchronous altitude the target turns with the planet underneath it, the
error never sweeps, and there is no window at all. The script says so in as many
words rather than returning the first launch time it happened to sample, and
goes at the best time inside its horizon; the phasing then has to be solved in
orbit, which is exactly what `rendezvous.ks` is for.

## 3. The aim point is deliberately early

The script does **not** aim to insert at the transfer lead angle. It aims a
little short of it and lets the parking orbit's drift carry the rest.

This is not caution for its own sake — the error is asymmetric:

* Arriving **early** costs drift time. Drift time is free: the ship is in a
  stable orbit with the reserve intact, and `rendezvous.ks` warps it out.
* Arriving **late** means the lead angle has already gone past, and the next one
  is a synodic lap away. That gets paid for in propellant.

So the margin is bought in the currency that pays for the mistake. An ascent
that runs `ASC_SIGMA` seconds late does not put the *ship* in the wrong place so
much as it lets the *target* run on — by `ASC_SIGMA × omega_target` degrees —
and the buffer is however long the parking orbit's drift takes to undo that:

```
drift      = |omega_park - omega_target|                  deg/s
buffer     = clamp( K * ASC_SIGMA * omega_target / drift,  BUF_MIN, BUF_MAX )
phase_burn = 180 - omega_target * t_hohmann               the classic lead angle
phase_want = phase_burn + (omega_park - omega_target) * buffer
```

The last line is the buffer expressed as an angle. The sign needs no special
case: parked under the target we gain on it, parked over it we fall behind, and
the same expression puts the aim point on the correct side either way.

Sizing the buffer as a flat "twenty minutes" instead would be far too much
against a fast-drifting orbit and nowhere near enough against a slow one. `K`
was chosen by simulating the whole thing 600 times per value:

| `ITC_BUF_K` | Median drift left in orbit | Flights that overshot the transfer point |
|---|---|---|
| 0.5 | 34 min | 22.5 % |
| 1.0 | 48 min | 7.5 % |
| **1.5** | **66 min** | **0.8 %** |
| 2.0 | 85 min | 0.3 % |
| 3.0 | 92 min | 0.3 % |

1.5 is the knee. Below it the overshoot rate climbs fast; above it each extra
percent of safety costs twenty minutes of loitering.

The drift the script plans for is not wasted time, incidentally — it is time
`rendezvous.ks` is *meant* to spend. "Stay in this orbit and wait" is one of the
candidates its phasing solver prices, and if the window did its job that is the
candidate that wins: zero ΔV, warped out, and re-solved against the geometry the
ship actually arrived with rather than the one predicted on the runway.

## 4. The parking orbit is chosen, not inherited

`ascent.ks` will fly to any apoapsis it is given, so this script gives it one
rather than defaulting to 100 km: **under the target where there is room, over
the target where there is not, and never level with it.**

```
r_park = max( r_atmosphere + PARK_FLOOR,  r_target - PARK_BELOW )
if |r_park - r_target| < PARK_GAP:   r_park = r_target + PARK_ABOVE
```

A parking orbit at the target's own radius has *no period difference*. The phase
you insert with is the phase you keep, forever, and the launch window would have
to be exact — which no launch window is, because the ascent it is built on
scatters by a minute or two. The gap is what turns a launch-time error into a
short wait instead of a phasing orbit.

The gap is a trade and both ends of it are real: 25 km under a station gives
about 0.008 °/s of drift (a synodic lap of ~12 hours, so ~2 minutes of drift per
degree of error) and costs about 25 m/s to close with a Hohmann transfer.
Widening it buys a faster drift for a few more m/s.

## 5. The plane: there is no window to wait for, and here is why

This is the part that most obviously *looks* like it should be a launch window,
and is not.

A launch site crosses the target's orbital plane twice a day. A rocket that can
pick its azimuth launches at exactly that moment, flies the matching heading, and
pays **nothing** for the target's inclination. That is the classic launch window
and it is worth hundreds of m/s.

A spaceplane on a runway cannot do it. It goes where the centreline points,
`ascent.ks` flies that heading the whole way up, and rolling down the runway and
then turning onto a launch azimuth is a different ascent profile, not a
parameter. So the ship reaches an **equatorial** orbit whenever it launches.

And an equatorial orbit crosses the target's plane at the target's own nodes —
always, at the same relative inclination, whatever time we left. **The plane
change costs the same regardless of the launch time**, so there is nothing to
wait for and no trade against the phase window. `rendezvous.ks` pays it at the
cheaper node, at `2 v sin(di/2)`.

What the script does instead is *say so*, before the countdown rather than after
the climb:

* it prints the bill, because at 10° of inclination it is about 390 m/s and that
  is a mission-sized number the pilot should see while the ship is still on the
  ground;
* it prints when the runway *does* pass through the target's plane, and the
  azimuth that would have been free there, for anyone who wants to hand-fly it;
* it holds for `ITC_HOLD` seconds over `ITC_PLANE_WARN_DV` so the warning cannot
  scroll past unread (`ITC_ABORT_IF_PLANE` makes it a refusal instead);
* and it **refuses outright** above 90° of relative inclination. A retrograde
  target cannot be reached from an eastward runway launch at any price worth
  paying, and that refusal belongs on the ground, before a drop is spent.

## 6. Where the launch site will be

A landed vessel's "orbit" is a fiction that follows the surface, so `POSITIONAT`
cannot be asked where the runway will be in twenty minutes. It does not need to
be. A point on the surface moves on a circle about the body's spin axis, and
both the axis and the phase can be measured right now, off the ship standing on
it: while landed and stationary, the ship's *orbital* velocity is exactly the
velocity the body's rotation gives it, so

```
k = r x v                        the spin axis, signed the right way round
omega = 360 / rotation period    how fast it turns
```

and rotating any vector about `k` in the direction the surface actually travels
is a two-line construction. The measured rate is cross-checked against the
body's published rotation period, which is what catches a ship that is not
really stopped — a wheel still turning makes every launch time wrong in a way
that looks entirely reasonable.

**None of this needs to know kOS's coordinate frame or its handedness.** The
axis is itself built with a cross product, and every later use of it sits inside
a second cross product or a triple product, so a global sign convention on the
operator cancels: `(r x v) x r = v (r·r)` under either rule. That is the same
property that makes `phaseAngleAt()` in `rendezvous.ks` frame-free, and it is
why the verification below can use ordinary right-handed vector algebra to check
a script written against a left-handed one.

### The frame does move, though

KSP holds the world still around a landed vessel by turning the **frame** with
the body. A direction vector read on the runway and used forty minutes later has
had the planet rotate underneath it while its numbers stayed the same, and a
prediction built on it is out by the whole rotation since it was taken — up to
40° over a long wait, which is the size of the very thing being computed.

So every prediction here is made forward from *now*, and "now" is re-read
(`itcRefresh()`) before each batch. After the warp, the countdown re-solves the
insertion error from a fresh reading and prints it: if the solver and the world
still agree it comes out at very nearly zero, and if it does not, that is a
number on the screen rather than a mystery in orbit.

The same three lines mean something usefully different once the ship is flying:
`r x v` is the spin axis for a vessel standing on the surface and the **orbit
normal** for one in flight. So the post-insertion phase is automatically
measured in the plane the ship actually ended up in — the convention
`rendezvous.ks` uses.

### And the target is predicted two ways, which are checked against each other

The same argument applies to the *target*, and this is where it stopped being
theoretical. There are two ways to say where a station will be in twenty
minutes:

* **`POSITIONAT`** — kOS's own prediction. Exact for any orbit, eccentric ones
  included, and the idiom `POSITIONAT(x, t) - POSITIONAT(BODY, t)` is what
  `rendezvous.ks` uses. But it is built on kOS's frame handling, and this script
  asks it from a *landed* ship, which is the one situation where the frame is
  not the inertial one.
* **Rotation** — take the target's position and orbit normal now, and rotate the
  one about the other at its own orbital rate. Exact for a circular orbit,
  needs no frame assumption whatsoever, and is the same construction the launch
  site gets.

Both are evaluated a third of an orbit ahead and compared. For a near-circular
target the rotation is used (it is exact, and it cannot be wrong about a frame);
for an eccentric one `POSITIONAT` is used if the two agree. The disagreement is
printed either way, and a second, independent guard sits behind it: the
**measured** sweep rate is printed next to the rate the two orbital periods say
it must be. Those are the same number computed two ways, and if they part
company the launch time is worthless — which is a thing to say in front of the
launch time, not to be inferred from a rendezvous that goes wrong an hour later.

The ascent arc is measured a third way again, and deliberately: **in longitudes,
not vectors.** It has to be measured across twenty minutes of flight, during
which KSP switches from the rotating frame to the inertial one, so the two ends
of a stored vector would not be in the same frame. A geographic longitude is
body-fixed by definition, and the inertial arc is then the ground arc plus
whatever the body turned through while we flew it.

## 7. What it checks before the clock starts

Waiting forty minutes and *then* discovering something that was knowable on the
ground is the specific failure this script exists to prevent. So before the
countdown:

| Checked | Because |
|---|---|
| The target exists, orbits **this** body, is not escaping, is not decaying into the atmosphere | An interplanetary transfer is a different problem, and a target with a periapsis inside the air is not a destination |
| Relative inclination ≤ 90° | A retrograde target is unreachable from an eastward runway launch |
| The ship is **landed or pre-launch, and stopped** | The launch site is measured off a stationary ship; a ship already in orbit wants `rendezvous.ks`, and is told so |
| `ascent`, `rendezvous` and `dock` are all present on the volume | Finding out after the countdown is the whole thing we are avoiding |
| ...and **how many parameters each of them declares** | See [What a flight found](#what-a-flight-found). kOS treats an argument-count mismatch as a fatal error and gives no way to catch it, so the three scripts are read on the ground and called the way they are actually written |
| A **free docking port** aboard, RCS blocks fitted, monopropellant over a floor | You cannot dock without them, and all three are trivially fixable on the ground and impossible to fix in orbit |
| The parking orbit is inside `ITC_PARK_CEIL` | Past that the ascent is the problem, not the window |

The things that *cannot* honestly be checked yet are named rather than faked.
The climb's own funding is `ascent.ks`'s pre-flight check, which runs at T-0 with
the engines lit and prices the whole thing properly. The station's free ports
cannot be read at all until the ship is within physics range of it — see §8.

What the script *does* price on the runway is the part it knows: the plane
change, the transfer, and the arrival allowance `rendezvous.ks` budgets, printed
against the ΔV in the tanks.

## 8. The handovers, and the three traps in them

Each stage is checked before the next one starts, because a stage that ended
early must not be followed by one that assumes it did not:

* **After the ascent** — periapsis above the atmosphere and the ship not back on
  the ground. `ascent.ks` will settle for a lower orbit than it was asked for
  rather than strand the ship, and that is fine; what is not fine is starting a
  rendezvous from a suborbital arc.
* **After the rendezvous** — inside `ITC_RDV_RANGE_OK` at under
  `ITC_RDV_VREL_OK`. `dock.ks` is an approach script, not a rendezvous one.
* **Before the docking** — a berth is chosen (below).

**Trap one: the berth cannot be chosen until it can be seen.** A station's parts
cannot be read while it is out of physics range, so the target port is picked
*after* the rendezvous, from what is actually there: free, not shielded shut
(we cannot open a shield from this side), and the same node size as ours. Among
equals the one pointing at us wins, because `dock.ks`'s corridor goes out,
forward and in, and a port on the far side of a station is the long way round
it. Every candidate is printed with its angle. If the pilot targeted a port on
the map in the first place, that choice is carried through untouched.

**Trap two: docking merges the two vessels.** The moment the magnets bite, the
target's `Vessel` stops referring to anything, and a closing report that
dereferences it throws on the one flight that went perfectly. So the target's
name and the berth's title are kept as strings from the start, and the docking is
detected by the ship's part count growing — the same test `dock.ks` uses. The
final ΔV figure is suppressed once docked, too: `SHIP` is then the whole stack,
and a ΔV computed from a station's mass is a fiction.

**Trap three: `RUN` shares one global scope.** Every name in this file is
prefixed `ITC_` or `itc`, and that is not decoration. The scripts this one calls
define helpers with the same obvious names, and they do not agree with each
other — `clampVal` takes `(value, lo, hi)` in `ascent.ks` and `(lo, hi, value)`
in `deorbit_land.ks`. A helper called `clampVal` here would silently become a
different function halfway through the mission. The instruction limit gets the
same treatment: `ascent.ks` saves and restores `CONFIG:IPU` itself, so by the
time it hands back, *its* idea of "the value before" is the value we raised it
to. The IPU is restored from our own saved copy.

---

## What a flight found

The first real flight of this script, against a station at 108 × 110 km over
Kerbin, produced two failures. Both are fixed; both are worth writing down,
because both were invisible to a static check and to the simulation.

**1. It waited five and a half hours for a window that should have been
minutes away.** The report was internally consistent — the plan, the buffer, the
aim point and the predicted insertion phase all agreed with each other — and
completely wrong. Against a 108 km target the window recurs every **36
minutes**; the script announced one **5:32:15** out.

Reconstructing it: 5:32 is one Kerbin day minus the initial phase error divided
by Kerbin's *rotation* rate. In other words the only thing sweeping in that
solve was the runway turning underneath a target that was not moving — the
target's predicted position was not advancing with time. That inference comes
from the reported numbers rather than from a debugger, and the cause inside kOS
was not chased further; what is certain is that this script is the only one in
the repository that asks `POSITIONAT` for a prediction **from a landed ship**,
which is the one situation where the world frame is turning with the body.
`rendezvous.ks` asks the same question from orbit, where it is not.

The old solver could not notice. It scanned for a sign change with a fixed
20-second step and reported whatever it found, so a prediction that had stopped
moving looked exactly like a prediction that was moving slowly. The rewrite
makes the sweep rate an explicit, measured quantity, prints it next to the value
the two orbital periods demand, and predicts the target by rotation rather than
by `POSITIONAT` whenever the orbit is near-circular — which removes the
dependency entirely for the case that matters. Both numbers are on screen before
the countdown starts. Reproducing the fault in the propagation confirms the
signature exactly: sweep −0.01671 °/s against an expected +0.16404, "a window
every 5.99 h", launch time hours out.

**2. It reached T-0 and stopped with an argument-count error.** The install's
`ascent.ks` did not declare a parameter, `RUNPATH(..., ITC_PARK_AP)` passed one,
and kOS treats that as fatal — after the wait, on the runway, with the window
spent. kOS gives no way to catch it, so the script now *reads* the three scripts
it is going to call, counts the `DECLARE PARAMETER` statements in each, and
calls them the way they are actually written. An ascent script that takes no
apoapsis cannot be sent to a parking orbit of our choosing, so that is refused
on the ground with the fix spelled out: use the `ascent.ks` in this repository,
or set `ITC_PARK_FIXED` to the altitude yours reaches and the window is planned
around that instead.

The general lesson is the one the rest of this repository keeps learning: a
number that is only ever computed one way cannot be checked, and a handover that
assumes the far side's signature is a handover that fails at the worst possible
moment. Both fixes are cross-checks, not corrections.

---

## Verification

The script cannot be run outside KSP, so the part of it that is pure arithmetic
— where the launch site will be, where the target will be, and which launch time
puts the two in the right relationship at insertion — was reimplemented in
Python exactly as the script computes it, and then **flown** by an independent
Kepler propagation of both vessels (eccentric targets included, which the
script's circular model does not assume it can predict).

**Sign conventions.** With a perfect ascent model, across four target altitudes,
three inclinations and eight starting phase angles, the launch time the solver
returns produces the insertion phase it was asked for to within **0.000000°**,
and the wait it then plans matches the buffer it asked for to within a second.
Flying the resulting transfer puts the two vessels in the same place at the same
time to within **1 metre** for an equatorial target. This is the check worth
having: a phase angle with the wrong sign produces a launch window exactly half a
lap out, which is the most expensive mistake a script like this can make, and it
looks completely plausible until the ship gets there. A separate counter watched
for waits longer than three quarters of a synodic period — where a sign error
would put *every* case — and found none in 1200 flights.

**The solver.** The closed-form solution was checked over 300 randomised
geometries (seven target altitudes, three inclinations, eccentricities to 0.02),
under both prediction models: every case returns a launch time whose insertion
phase error is **under 0.05°**, which is the tolerance it verifies against, and
none falls back to the horizon scan. Fed a deliberately broken prediction — a
target that does not advance — the sweep cross-check reports −0.0167 °/s against
an expected +0.164 °/s, which is the failure described above and is now printed
rather than flown.

**Ascent scatter.** With the ascent time scattered by ±2 minutes and the arc by
±12° (a calibrated ascent), over 600 randomised geometries: median drift left to
spend in orbit **66 minutes**, worst 168, and **5 flights in 600** overshot the
transfer point. The same 600 geometries launched immediately: median 185
minutes, and **324 in 600** overshot. On a first, uncalibrated flight (±5
minutes, ±40°) the window still holds the overshoot rate to 14 %, which is why
the calibration file exists and why it is worth flying twice.

**What the model does not capture**, quantified rather than waved away: the
phase is measured through a projection into the equatorial plane, and for an
*inclined* target that projection is not uniform — the equatorial longitude of a
point in an inclined orbit runs ahead of and behind its argument of latitude by
up to `(1 - cos i)/2`. The propagation puts the resulting error at **1 metre** of
along-track miss at 0° of inclination, 13 km at 12°, and 70 km at 28°. Against
the tens of km `rendezvous.ks` corrects as a matter of course, and against the
hundreds of m/s the plane change itself costs at those inclinations, this is not
the thing to fix.

Neither of these is a substitute for flying it. What they establish is that the
geometry is right, which is the part that cannot be trimmed out afterwards by
nudging a constant.

## What to trim after the first flight

The first flight is the one that calibrates the ascent model, and the report
prints everything needed to judge it. In order of how often you will touch them:

| Constant | What it decides | Symptom that it is wrong |
|---|---|---|
| `ITC_ASC_TIME` / `ITC_ASC_ARC` | The ascent model the window is built on | Insertion phase error is consistently in one direction. Normally you never touch these — the calibration file does it |
| `ITC_ASC_SIGMA` | How repeatable you think the ascent is | Drift waits feel long (too high) or transfers keep being missed (too low). Lower it once a few flights show the real scatter |
| `ITC_BUF_K` | Safety factor on that buffer | See the table in §3 |
| `ITC_PARK_BELOW` | The altitude gap, and so the drift rate | Slow drift and long waits (too small); a transfer that costs more than it should (too large) |
| `ITC_MAX_WAIT` | How far ahead a window may be | It launches at a bad time rather than waiting for a good one, against a high target |
| `ITC_PLANE_WARN_DV` | When the inclination bill is shouted about | Warnings for plane changes your ship can easily afford |
| `ITC_RDV_RANGE_OK` | How close counts as a finished rendezvous | Docking is refused after a rendezvous that actually went fine |
| `ITC_MONO_DOCK` | Pre-launch monopropellant floor | Launches refused on a ship that had enough |
| `ITC_HOME` | Where the other three scripts live | "Cannot find these scripts" — set it to `"0:/"` |
| `ITC_TGT_MODEL` | How the target's future position is predicted: `"rotate"`, `"posat"` or `"auto"` | The two models disagree, or the measured sweep does not match the periods. `"rotate"` is the safe answer for any station |
| `ITC_PARK_FIXED` | The parking altitude, when your ascent script cannot be told one | The script refuses to launch because `ascent` declares no parameters |
| `ITC_ASC_PARAMS` / `ITC_RDV_PARAMS` / `ITC_DOCK_PARAMS` | Parameter counts, if they cannot be read from the files | "could not read their parameter lists" in the pre-flight, and your scripts do not take (1, 2, 2) |

`ITC_RUN_ASC`, `ITC_RUN_RDV` and `ITC_RUN_DOCK` can each be set `FALSE` to
rehearse the window solver and the pre-flight checks without flying anything —
the cheapest way to see what the script thinks of a target before committing an
afternoon to it.
