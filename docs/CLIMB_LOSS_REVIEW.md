# Climb-loss review — Perfect Cargo SSTO MkIV v5, 100 km target

Written against the flight that printed **"!! CANNOT REACH ORBIT"** at 29 km and
then fell back into the atmosphere with the ascent loop still running. Every
number below is from that flight's own console output.

## What the ship was

The pre-flight check declared the airframe short on the runway:

```
!! NOT ENOUGH dV - SHORT BY 275 m/s
   (needs 1766, has 1491 at handover)
```

**That verdict is wrong, and it is wrong for the same reason the flight failed.**
See [The pre-flight check was wrong too](#the-pre-flight-check-was-wrong-too) at
the end — `PLAN_LOSS_FACTOR` had been calibrated from a number the broken
measurement produced.

270.6 t launch mass, 72 t of it payload — a 27% payload fraction, with 1491 m/s
of paired propellant in the ship's own tanks at the handover.

What follows is the four ways the rocket phase then threw away an orbit it could
afford.

## Defect 1 — the loss factor was measured cumulatively and spent forward

The handover was good: 23.2 km at 1419 m/s, better than the planned 21 km /
1400 m/s, with 1499 m/s in the tanks. Thirty seconds later:

```
!! CANNOT REACH ORBIT. Even a 78 km apoapsis is out of budget.
   dV 1121 vs need 5132 (climb 4869 + circ 122 + deorbit 40 + margin 100) @ loss x6.37
```

A ship the pre-flight priced 275 m/s short was now, apparently, 4011 m/s short.
Nothing had broken. The x6.37 came from `spent / paid`, both measured
cumulatively from the mode switch, and it was then charged against the entire
remaining climb.

The first 30 seconds after a RAPIER handover is the worst the rocket will ever
perform: lowest TWR, heaviest ship, and the last of the drag layer. It is also,
under a cumulative average, the *only* thing the measurement can see when the
first verdict is due. `FEAS_ARM_DV` of 350 m/s guaranteed the call was made
there and nowhere else.

The flight's own telemetry, four reports later, refutes it:

| | alt | speed | accel | TWR | ΔV |
|---|---|---|---|---|---|
| verdict | ~29 km | ~1655 m/s | 6.05 | 1.28 | 1121 |
| +20 s | 30.7 km | 1704 m/s | **9.45** | 1.34 | 1050 |
| +30 s | 32.9 km | 1791 m/s | **9.36** | 1.41 | 929 |

2520 kN on ~240 t is about 10.3 m/s² of thrust acceleration. Delivering 9.45 of
it is a loss factor of **×1.1**, not ×6.37 — measured *after* the script had
declared the mission impossible.

**Fix.** The loss factor is measured over a trailing window of `EFF_WINDOW_DV`
(120 m/s) of rocket ΔV, rolled forward each time it fills, and used raw. It
answers "what is a m/s buying *now*", which is the only question the settle
decision needs, because now is when the remaining m/s get spent. Replaying the
flight's own telemetry through it:

| alt | ΔV left | cumulative (old) | window (new) |
|---|---|---|---|
| 24.0 km | 1378 | — | ×12.0 |
| 26.0 km | 1257 | — | ×2.84 |
| 28.7 km | 1136 | **×5.60** | ×1.81 |
| 31.4 km | 1014 | ×3.04 | ×1.30 |
| 33.7 km | 892 | ×2.39 | **×1.29** |

The whole-climb figure is still computed, under the name `lossWhole`, because
the post-flight `PLAN_LOSS_FACTOR` advice prices a climb that starts at the
handover and must include the expensive bottom of it. Quoting the window figure
there would tell the next flight the ascent is cheaper than it is.

## Defect 2 — the verdict was pronounced in the drag layer, and latched

Even a correct measurement taken at 29 km says nothing useful about 50 km of
vacuum climb. Two changes:

- **A pressure gate.** The terminal "no orbit is reachable" verdict may only be
  pronounced once ambient pressure is under `EFF_VERDICT_P` (0.005) of the
  body's sea-level pressure — ≈33 km on Kerbin. Below that the script may still
  re-target downward, which is cheap and reversible; it may not write the
  mission off. A ship spending ΔV and buying nothing for two consecutive
  windows overrides the gate, because that is not a regime artefact.
  The override needs two windows: the single window straight after a handover
  reads the ceiling on almost every spaceplane, because the jets have cut and
  the rocket has not yet spooled.
- **No latch.** `emergency` was set once and never cleared, so no later evidence
  could restore the climb. It now clears when the measured factor brings a
  minimum orbit back inside the budget by `EFF_RESUME_DV`.

Replayed against the flight, the verdict is deferred to 33.7 km and reads
*"need 930 vs have 892"* — short by **38 m/s**, not 4011. That is an honest
number a pilot can act on, and it is within the model's own noise. Note that
the 930 includes a fully funded deorbit burn and 100 m/s of untouched margin;
the ship was ~38 m/s short of an orbit *plus* the fuel to leave it.

## Defect 3 — the emergency flat burn commanded the nose up

Having decided apoapsis was no longer the objective, the script clamped the
commanded flight-path angle to `CC_FPA_FLAT` (2°) — and then, four lines later,
added up to `CC_TAP_MAX_ADD` (12°) back on, because the apex-time floor is
applied unconditionally. The log shows the demand climbing 2 → 3.8 → 7.6 → 9.4
→ 10.5° and the pitch command reaching **+24.5°**, in a burn whose stated
purpose was to stop buying altitude:

```
fpa 0.4/9.4 | pitch 23.4 | ... | FLAT-BURN (orbit priced out)
```

It is also a runaway: the flatter the ship flies, the sooner the apex arrives,
so the floor demands more climb. The apex floor is now skipped while in
emergency, where reaching the apex is the plan rather than a failure.

## Defect 4 — the loop had no exit for a dead engine

From 34.4 km onward every report reads `TWR 0`, negative acceleration, and ΔV
frozen at exactly **859 m/s**:

```
AP 36/100 km | alt 36 km | 1791 m/s
    fpa -0.1/10.5 | pitch 24.5 | acc -1.82 | TWR 0 | dV 859
    PE -341.7 km | apex in 0 s | FLAT-BURN (orbit priced out)
```

The engines had stopped, but `rocketDv()` prices the propellant in our tanks —
it cannot tell whether the engines can still reach it. With thrust gone, both
loop exits become unreachable: apoapsis stops rising so the target is never met,
and the ΔV reading freezes so the glide reserve is never crossed. The loop spun
at full throttle while the ship coasted over the top and fell back into the air,
with the ascent autopilot still holding the controls and the reentry script
never run.

The jet phase has guarded exactly this since it was written
(`IF anyFlameout() OR SHIP:AVAILABLETHRUST < 1`). The rocket phase had no guard
at all. It now ends the climb after `CC_DEAD_CONFIRM` consecutive no-thrust
samples and hands over to the suborbital-glide path, printing the core LF/Ox
state on the way out — a frozen ΔV reading next to zero thrust means a feed or
ignition fault rather than an empty ship, and the next flight should show which.

Total thrust loss only: one flamed-out RAPIER out of fourteen is not a reason to
end a climb the other thirteen are still flying.

## The pre-flight check was wrong too

The same ship, at the same 270.6 t, flown by the **PR #2 autopilot** — the
147-line fixed-schedule original, no ΔV budgeting, no feasibility check, a flat
12° air-breathing climb and a 25°→5° rocket taper — reached orbit:

```
ORBIT ACHIEVED
  Apoapsis : 104.07 km
  Periapsis: 99.01 km
  Inclination: 0.05 deg
  Fuel remaining -- LiquidFuel: 8097 , Oxidizer: 9071 , Monoprop: 100
```

Reconciling that against the pre-flight print of the pad state (1337 m/s of
pairs in our tanks, a 4929-unit LF-only reserve, 5760 LF / 7040 Ox of cargo):

| | |
|---|---|
| Burned | 11371 LF / 8698 Ox = **100.3 t** |
| In orbit | **170.3 t** |
| Jets took | 4254 LF — inside the LF-only reserve, 675 units spare |
| Rocket burned | 79.1 t: 249.3 t → 170.3 t = **1141 m/s** for climb + circularisation |

The model priced that same climb and circularisation at **1608 m/s** (climb 1540
+ circ 68). Solving `1141 = 880 × L + 68` for the impulsive 880 m/s climb gives
a real loss factor of **×1.22**, against the `PLAN_LOSS_FACTOR` of 1.75 the check
was using — and 1.75 came from the ×3.21 recorded in `ASCENT_REVIEW.md`, which
was produced by the identical cumulative `spent / paid` calculation this review
is about. **The broken measurement poisoned the pre-flight constant, and the
pre-flight constant then condemned a ship that flies.**

`PLAN_LOSS_FACTOR` is now **1.30** — the measured 1.22 plus a little back, for
the reason below. On this airframe that reads *needs 1370, has 1491* — a GO with
8.8% in hand, where 1.75 read *short by 275*.

### The caveat, which is not small

PR #2 has no payload isolation, and stock fuel flow is proportional across every
connected tank. Of the 100.3 t it burned, **34.1 t came out of the cargo tanks**
— about a third of the entire ascent was flown on the payload's propellant. The
current script blocks that deliberately, so the isolated ship carries those 34 t
all the way to orbit and will pay a little more in gravity loss for it. Hence
1.30 rather than 1.22; the difference is an estimate, not a measurement, and the
first flight that gets to the top under the fixed script will replace it with
the `PLAN_LOSS_FACTOR` line the post-flight block now prints.

The pre-flight advice states the choice outright rather than making it:

```
* PAYLOAD FUEL: the cargo tanks hold 5760 LF / 7040 Ox, locked out as cargo.
  Counting them would put 3168 m/s at the handover instead of 1491.
  SET ISOLATE_PAYLOAD TO FALSE. only if that fuel is propellant
  and not deliverable cargo - the engines will drink it.
```

3168 against 1491 is why PR #2 sailed to 104 km with no budgeting at all. Whether
that is a success or a theft is a question about the mission, not the ascent.

### Still worth doing

The jets ate **107 units of paired LF** — rocket ΔV burnt as jet fuel. 121 units
of LF-only reserve (+0.6 t) closes that leak and lifts the handover by 19 m/s.
It is the cheapest fix on the list and it is real whatever else is decided.
