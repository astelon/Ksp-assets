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

## What to watch on the next flight

1. `energy` and the new `L/D` field in the entry line. If the measured L/D sits
   near 2 with the boards in and the alpha low, `ENTRY_LD` is simply optimistic
   for this airframe and should come down to match; the range plan will then be
   honest from the interface rather than from 24 km.
2. The `AoA 12/14 deg` pair in the entry line — command and actual. They should
   track within a few degrees. A persistent gap means `REENTRY_AOA` is past what
   this ship can trim to, and the cap will now say so out loud.
3. `spd 251/247 m/s` in the glide line — actual and target. The target should
   climb with altitude; if the ship cannot hold it in the thin air just after
   handover that is expected, and it will converge as it descends.
4. Whether the jets ever run with a positive vertical speed. They should not,
   except inside a stall recovery.
5. `DEORBIT_LEAD` is unchanged at 149°. It was trimmed against a flight whose
   entry was throwing its range away; with the range plan fixed the ship should
   arrive long rather than short, and 149 is now the first number to reconsider —
   downward — if it does.
