# Glide review — why it never glided, and why the fuel went into the sky

A reference flight of `deorbit_land.ks` deorbited on time, survived the heat,
and then flew 22 km of altitude and a full tank of jet fuel into the ocean
248.6 km from the runway. The log is unusually clear about how, because the ship
narrated every stage of it: six stalls in the glide, a `!! Out of height 282.2 km
from the runway - no approach to fly`, twenty-one `divert` lines climbing from
660 m to 7 459 m at a fixed 105 m/s, another climb to 8 922 m, and
`LiquidFuel: 0`.

This is the analysis of that telemetry and what was changed because of it. It is
the sequel to `docs/REENTRY_REVIEW.md`: the envelope rules that review installed
all worked. What failed this time is everything that decides *what to ask the
envelope for*.

## What the log says

| Phase | Sample | Reading |
|---|---|---|
| Entry | interface | alt 68.8 km, spd 2 097 m/s, 803 km to the fix, `energy 1.09` |
| Entry | 45.5 km | spd 1 940 m/s, 430 km to the fix, `energy 1.69`, AoA 40° |
| Entry | 24.0 km | spd 812 m/s, 295 km to the fix, `energy 0.82` |
| Entry | handover | spd 650 m/s at 22.8 km, **292 km to the fix**, `energy 0.66` |
| Glide | first print | alt 22 727 m, profile 65 660 m, **AoA 62°** |
| Glide | six times | `** stalled at 488 / 203 / 142 / 107 / 98 / 50 m/s` — 129–164° off the airstream |
| Glide | floor | `!! Out of height 282.2 km from the runway` |
| Divert | 660 → 7 459 m | 21 prints, airspeed pinned at 103–110 m/s the whole climb |
| Divert | second climb | to 8 922 m, then 8 064 → 6 814 → 6 938 m, stall, dive, repeat |
| Report | — | `DITCHED IN THE WATER 248.6 km from the runway`, LiquidFuel 0 |

Five separate faults, each of which would have been enough on its own.

## 1. The entry spent the range it was going to need

`rangeCapability()` prices energy height — altitude plus the height the speed is
worth — at an `ENTRY_LD` that tapers from 3.0 hypersonic to `PLAN_LD` 4.5
subsonic. At the interface that reads 293 km of energy height × 3.0 = 879 km of
capability against 803 km to fly: `energy 1.09`, marginally long. The entry
therefore did what it is told to do when long — went to `REENTRY_AOA` 40°, held
the S-turn, and flew the whole descent with the airbrakes out, because `BRAKES
ON` was issued *once*, above the loop, and never reconsidered.

It then stayed long on the gauge for eight solid minutes — 1.09, 1.31, 1.69,
1.72 — while destroying the very range the number was promising. The model is
what hid it. As the ship slows, the modelled L/D *rises* toward 4.5, and it rises
at almost exactly the rate the extra drag was eating the energy, so the ratio
holds steady while the ship falls out from under it. It crossed from 1.11 to 0.66
in the last 90 seconds of the entry, which is far too late for a spaceplane to do
anything about being 100 km short.

The flight can be scored directly. Between the interface and the handover it flew
511 km of ground range for 249 km of energy height: an end-to-end L/D of **2.05**,
against the 3.0 the plan was budgeting. At the 45.5 km sample the instantaneous
figure was worse still — 18 km of range for 10.4 km of energy height, **1.73**,
while the model was crediting it with 3.06.

**Changed.** The entry now measures what it is actually getting. `ldSample()`
takes the ground range flown per metre of energy height spent every `LD_SAMPLE`
seconds, filters it, and `rangeCapability()` never credits itself with more than
the measurement — the nominal figure is only ever an upper bound. The measurement
is taken while flying for energy loss rather than for range, so it is a floor on
what the ship could do and not a forecast of what it will; believing only the
pessimistic half of it is the point. Replayed against this flight, the 45.5 km
sample reads 411 km of capability against 430 km to go — `energy 0.96`, short —
and the entry stops digging eight minutes before it used to notice.

Two supporting changes: the boards are now an energy dump like any other, out
only while `energy` is above `ENERGY_LONG` and retracted with hysteresis when it
is not, and `ENERGY_LONG` / `ENERGY_SHORT` move from 1.08 / 0.95 to 1.15 / 1.00 —
dumping energy has proved expensive enough to deserve a real margin before it
starts, and a dead band where the ship simply flies.

## 2. The entry never checked whether the ship was flying its own command

The first thing the glide ever printed about that flight was `AoA 62 deg`, at
643 m/s. `noseOff()` is the *actual* angle between the nose and the airstream, and
the entry had just handed over a ship 62° off it — with the entry's own last
command being 12°. The glide's departure detector fired on the very first sample.

The entry loop had no such detector. It printed `gAoa`, the command, and the
command is a wish: a ship that departed at 40 km would tumble all the way to the
handover with the log cheerfully reporting the alpha it wished it were flying.
Every downstream number — drag, L/D, the range plan above — was priced on an
attitude the ship was not in.

**Changed.** The entry now compares `noseOff()` with what it asked for, every
pass:

* more than `ENTRY_TRIM_TOL` (12°) past the command for two seconds and the entry
  caps its own alpha demand by `ENTRY_AOA_STEP`, taking it back a degree per ten
  seconds of good tracking. The airframe outranks the tunable.
* more than `ENTRY_HOLD_TOL` (25°) past it for one second and the ship has
  departed, not lagged: `stallRecover()` runs, and the alpha cap comes down.

## 3. The glide was flying a number, not a wing

`GLIDE_SPEED` was 160 m/s — one number, for every altitude from the handover at
22 km to the flare. A wing does not fly on airspeed, it flies on dynamic
pressure, and the script says so itself: the comment above `STALL_Q` works out
that the ship needs about 0.07 atm to hold level flight, "which is 110 m/s at sea
level and 160 m/s at 6 km, the two speeds this script already flies". 160 m/s was
therefore correct — at 6 km, the altitude it was measured at.

At 12.7 km, where the log records `** stalled at 203 m/s`, 160 m/s is about
0.027 atm. This script calls 0.025 "the wing is gone" and 0.045 "the least air
you may bank in". The glide's own target speed at that altitude was a stall
order, and the guidance law kept issuing it: nose down for speed until 160, hold,
ask for a 30° banked turn onto the fix, depart, dive, recover, ask again. Six
times, 129° to 164° off the airstream, all the way down.

**Changed.** `glideSpeedTarget()` returns the speed that puts `GLIDE_Q` under the
wing *here*: dynamic pressure goes as ρv², and both are readable — the ship is
flying in the answer — so the target is the current speed scaled by the square
root of the pressure ratio, and no density model is needed. That is 110 m/s over
the runway, 160 at 6 km, 250 at 12 km, 450 at 20 km. Constant q is constant lift
coefficient, which is the best-glide condition at every altitude, so the
`PLAN_LD` profile the glide is flown against becomes true at all of them rather
than at one.

## 4. Recovery handed the ship straight back to the thing that broke it

`stallRecover()` returned the moment `SHIP:DYNAMICPRESSURE` crossed back over
`RECOVER_Q`. That is one condition short. At 12 km that threshold is 187 m/s in a
steep dive with the nose still well off the airstream — and the first thing the
glide law then did was demand a bank onto a fix 277 km away. Every one of the six
departures was entered from the recovery of the last one; the ship never got more
than 2 km of altitude out of a cycle.

**Changed.** A recovery ends when the wing has air **and** the nose is back within
`RECOVER_NOSE` (20°) of the airstream, or at `RECOVER_MAX_T` — at which point it
says so instead of pretending. `gSteady` then holds the turn demand off for
`STEADY_TIME` seconds while the ship builds a margin over the speed that just
failed. The recovery also stops lighting the jets above `JET_ARM_ALT`: full
throttle at 20 km in a tumble is the glide reserve going out of the back of the
ship, and three of the six recoveries were flown there.

## 5. Under thrust, the glider law flies the ship into the sky

The clearest twenty-one lines in the log:

```
divert  alt 660 m   spd 219 m/s
divert  alt 1302 m  spd 104 m/s
divert  alt 3081 m  spd 107 m/s
divert  alt 5290 m  spd 109 m/s
divert  alt 7459 m  spd 110 m/s
** stalled at 109 m/s (5 deg off the airstream), 8583 m AGL
```

Altitude monotonically up, airspeed pinned. Both the glide and the diversion
lit the jets at **full throttle** whenever the airspeed was below target, and both
held the airspeed **on the nose**. The two rules cannot both be satisfied: the
nose is already holding the target speed, so the thrust has nowhere to go but
into altitude. The ship climbed to 7.5 km, stalled, dived, recovered, climbed
again to 8.9 km, and repeated it until the tanks were empty — at which point the
last dive had no recovery in it. That is the "grasping for air" and the final
crash, and it is one sign error in the division of labour.

**Changed.** With the engines lit the two control laws swap ends, which is how a
powered aeroplane is flown:

* the **throttle** holds the speed — `THROT_MIN_PWR` biased, `THROT_PER_MS` per
  m/s of error, and off entirely when the ship is fast;
* the **nose** holds the vertical speed — `POWER_CLIMB_VS` (5 m/s) when below the
  profile, level otherwise, and never a climb above `POWER_CEILING` (7 km),
  because a ship short of the runway cannot climb its way there.

The diversion is stricter still. It is a landing, so it descends: thrust buys
speed, and only while the vertical speed is negative. Final approach gets the same
treatment — thrust for a low or slow approach, cut the moment the ship is above
the slope with the speed made good.

## Also changed: the coast is no longer flown backwards

Unrelated to the crash but visible in the same flight: after the deorbit burn the
ship held `SRFRETROGRADE` all the way to the interface — engines first, looking
back down its own wake — and it sat there in real time for the whole coast.

The burn over, there is nothing left to point the engines at, and the attitude
held through the coast is the attitude the ship meets the air in. It now turns
**prograde** as soon as the burn ends, then solves the coast rather than sitting
through it: `coastTimeTo()` bisects the predicted trajectory for the moment it
falls through the atmosphere height, prints it, and warps to `COAST_LEAD` seconds
short of it whenever there is more than `COAST_WARP_MIN` (60 s) to wait. Attitude
first, warp second — on rails the ship cannot rotate — and the lead is kept back
to put the nose back on the airstream afterwards.

## 6. Nobody could tell which side it missed on

The question this review was opened to answer — *is the entry scheduling enough
distance to slow down and still reach the KSC?* — could not be answered from any
line the script has ever printed, because every one of them is an unsigned range.
`groundRange()` returns a distance, not a displacement, so
`DITCHED IN THE WATER 248.6 km from the runway` is the identical line whether the
ship stopped 248 km short of the field or sailed 248 km beyond it. Those two
failures want opposite corrections, and telling them apart was left to whoever
was watching the map.

For the flight above the answer turns out to be **short**, and the log proves it
without the map: the glide's range to the fix falls to 273.3 km and then stops
falling — 275.5, 275.4, 275.5, 275.2, 275.1 — the ship was not flying anywhere
near the field, it was sinking. It never got closer than 273 km, and the
diversion closed from 282.2 km to 248.6 km before it hit the water, i.e. it was
still crawling *toward* the KSC when it ditched.

**Changed.** `alongTrackToRwy()` resolves the range onto the landing direction:
positive is the runway still ahead (short), negative is the runway behind
(overflown). It goes in the entry line, the glide line, a note at the end of the
entry, and the closing report, which now says `SHORT by 248.6 km along the
approach` or `LONG by …`, plus the cross-track miss, plus which way to trim.

## 7. Was it scheduling enough distance? — and `DEORBIT_LEAD` is now solved

`DEORBIT_LEAD` was a hand-trimmed 149°, and a lead angle is a proxy for the thing
that actually matters: the ground range from the atmospheric interface to the
KSC, which is the distance the ship has to bleed 2 100 m/s across. That distance
is now the tunable — `ENTRY_RANGE`, 800 km — and the burn angle is derived from
it.

The derivation is exact and has nothing to do with the airframe. A retrograde
burn makes the burn point the *apoapsis* of the ellipse it creates, so the coast
from there to the interface is fixed by two radii — where the burn happens and
the periapsis it aims at. `coastArcDeg()` solves the eccentric anomaly at the
interface radius, turns it into a true anomaly for the arc and a mean anomaly for
the time, and hands the planet back the rotation it performs underneath the ship
meanwhile. For a 100 km parking orbit aiming at a 32 km periapsis that is
**73.1° of ground track over 7.25 minutes**, and

```
lead = 800 km / 10.47 km-per-deg + 73.1° = 149.5°
```

— the hand-trimmed 149° to within half a degree, which is a good sign for both
numbers. It also shows what a fixed lead angle was hiding: the coast arc is 63.4°
from a 90 km parking orbit and 85.4° from a 120 km one, so a lead trimmed at one
parking altitude is **230 km wrong** at the other, and changing `DEORBIT_PE`
moves it too. Setting `DEORBIT_LEAD` non-zero still overrides the solution.

So: is 800 km enough? The two reference flights bracket the airframe rather than
the geometry:

| Flight | How it was flown | Range made from the interface |
|---|---|---|
| `REENTRY_REVIEW` | mostly clean, low alpha | ~880 km |
| this one | 40° alpha + boards, the whole way | ~530 km |

800 km sits mid-band: about 80 km of surplus for the entry to dump if it flies
clean, and 270 km of dumping authority in hand if it does not. Both are inside
what the energy loop can steer with — which is the property to hold onto, because
a schedule the ship cannot correct from *either* side is not a schedule. The
overshoots that prompted the question are the other end of the same failure as
this crash: an entry that cannot tell how much range it is really buying cannot
tell how much to throw away, and it is as capable of sailing past the field as of
falling short of it.

One more thing had to change for a long arrival to be survivable. The glide
compared bare **altitude** against the profile, and altitude is only half the
energy the ship has: a ship at 5 km doing Mach 2 — energy height 26 km, wildly
long, and about to fly straight past the KSC — read as 2.5 km *below* profile,
which retracts the boards and lights the jets. Both sides of that comparison are
now energy heights, so a fast arrival dumps energy instead of being handed
thrust.

## What to watch on the next flight

1. `rwy` in the entry and glide lines, and the `SHORT by` / `LONG by` line in the
   report. This is the one number that decides `ENTRY_RANGE`, and the correction
   is roughly one-for-one: overflying the field by 200 km means raising
   `ENTRY_RANGE` by about 200 km.
2. `energy` and the new `L/D` field in the entry line. If the measured L/D sits
   near 2 with the boards in and the alpha low, `ENTRY_LD` is simply optimistic
   for this airframe and should come down to match; the range plan will then be
   honest from the interface rather than from 24 km.
3. The `AoA 12/14 deg` pair in the entry line — command and actual. They should
   track within a few degrees. A persistent gap means `REENTRY_AOA` is past what
   this ship can trim to, and the cap will now say so out loud.
4. `spd 251/247 m/s` in the glide line — actual and target. The target should
   climb with altitude; if the ship cannot hold it in the thin air just after
   handover that is expected, and it will converge as it descends.
5. Whether the jets ever run with a positive vertical speed. They should not,
   except inside a stall recovery.
