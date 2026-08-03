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

It over-states, but not by as much as it looks: a later flight of the same
airframe reached a 92 km orbit and deorbited, which puts the real shortfall at
roughly **117 m/s** rather than 275. See
[the confirmation flight](#confirmation-flight--main-manual-takeover) below.

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


---

# Confirmation flight — main, manual takeover

A second flight of the same airframe on `main` (without any of the fixes above),
where the pilot cancelled the autopilot at the abort and flew the rest by hand.
It reached **92.2 × 91.6 km**, and after releasing the payload had enough left to
deorbit. This is the flight that settles the arguments.

## The contradiction, in the ship's own words

At the mode switch:

```
Mode switch: jets out of acceleration (0.12 m/s^2).
  at 23.2 km, 1418 m/s, pitch 8.5 deg.
  Handover 23.2 km / 1418 m/s (planned 21 / 1400).
Closed cycle. dV 1499 vs need 1267 (climb 970 + circ 139 + deorbit 58 + margin 100) @ loss x1.15
  Mass 245.8 t, thrust 2517 kN, TWR 1.13, Isp 305 s.
```

**Feasible, with 232 m/s to spare.** Thirty-one seconds and 378 m/s later:

```
!! CANNOT REACH ORBIT. Even a 78 km apoapsis is out of budget.
   dV 1121 vs need 5154 (climb 4891 + circ 122 + deorbit 40 + margin 100) @ loss x6.39
```

The impulsive climb estimate barely moved — 843 m/s at the switch, 765 at the
abort, because the ship had made real progress. Only the loss factor moved, 1.15
→ 6.39, and it multiplied a number that was already nearly right into one four
times the size of the tanks.

## What the fixes in this PR would have done

Splitting the flight at the abort and pricing each half against its own
telemetry:

| | spent | impulsive work | measured |
|---|---|---|---|
| Opening — handover → 27.9 km | 330 m/s | 40 m/s of orbital energy | **×8.2** |
| Rest — 27.9 km → 92 km orbit (by hand) | 1115 m/s | climb 810 + circ 56 | **×1.31** |
| Whole rocket phase | 1445 m/s | climb 843 + circ 66 | **×1.66** |

- The **trailing window** reads ×1.31 for the second half. That is the number the
  settle decision needed and it says the orbit was affordable — which it was,
  because the pilot then flew it. The cumulative ×6.39 said it was 4033 m/s out
  of reach.
- The **pressure gate** would have held the terminal verdict until ≈33 km. The
  abort fired at 29 km.
- **×8.2 in the opening is real, not an artefact.** The old code's mistake was
  extrapolating it, not measuring it.

## The opening is where the mission is lost

330 m/s of the ship's 1499 went into the first 31 seconds and bought 40 m/s of
orbital energy — **12% efficient**. Working the loss back out of the force
balance:

| | |
|---|---|
| Thrust available | 2517 kN on 245.8 t = **10.2 m/s²** |
| Loss (drag + cosine; gravity is already in the energy sum) | **9.5 m/s²** ≈ 2335 kN |
| Dynamic pressure | 0.20 atm |
| Implied Cd·A | **≈115 m²** |
| Commanded pitch 21.8° at fpa 5.4° | **16.4° of angle of attack** |

`AOA_TRIM_MAX` is 14°, so the trim was saturated and the real angle was larger
still — the trim clamp limits the trim, not the angle the airframe presents,
and while the flight path lags the command the difference is the fpa error.
Body drag on a Mk3 stack scales with the sine of that angle, and a 40 m fuselage
at 16° presents roughly five times the area it does at 0°. That is where the
2335 kN comes from, and it is 93% of the engines' output.

**Fix.** `CC_AOA_HIQ` (6°) caps the angle presented to the flow while `SHIP:Q` is
above `CC_AOA_Q_HI` (0.10 atm), opening to the full `AOA_TRIM_MAX` by
`CC_AOA_Q_LO` (0.02 atm). The clamp is applied to `pitch − fpa`, not to the trim.
The rocket phase also now reports AoA, Q and drag every cycle, because none of
this was visible on either flight.

> **This is the one change in this PR with no flight data behind it.** The
> reasoning is sound and the instrumentation will confirm or refute it in one
> flight, but a ship at TWR 1.13 that is told to fly flatter will accelerate
> instead of climbing, and wants watching the first time. `CC_AOA_HIQ = 14`
> restores the old behaviour exactly.

## `PLAN_LOSS_FACTOR` — corrected again, to 1.65

An earlier revision of this document set it to **1.30** on the strength of the
PR #2 flight. That was wrong, and the mistake is worth naming because it is the
same one the old code made: **1.30 is a marginal-climb figure and
`PLAN_LOSS_FACTOR` is a whole-climb constant.** It prices a climb starting at the
handover, so it has to carry the opening, and the opening is ×8.2.

This flight measures the whole thing end to end: 1445 m/s spent from a 1499 m/s
handover, against an impulsive 843 climb + 66 circ, with 54 m/s left in an
92 km orbit. Solving gives **×1.66**.

| `PLAN_LOSS_FACTOR` | bill | vs the 1499 it had | verdict |
|---|---|---|---|
| 1.30 | 1321 | +178 | GO — wrong, it did not have 178 spare |
| **1.65** | **1616** | **−117** | short by ~117 — matches the ~129 it really was |
| 1.75 (original) | 1700 | −201 | over-states by ~70 |

The PR #2 number (×1.22) was not a lie, it was a different flight: no payload
isolation, so a third of what it burned came out of the cargo tanks, and a ship
shedding mass that fast climbs at a TWR this one never sees.

---

# What to change on the ship

Ranked by ΔV per unit of effort, all from the two flights' own telemetry.

### 1. The deorbit reserve was priced at the wrong mass — now fixed in code

This one was not advice, it was a defect, and it is the difference between the
fixed script flying this mission and still giving up on it.

Replaying the flight through the fixed code: at 33.7 km — the first point where
the pressure gate lets the terminal verdict be pronounced — the window reads
×1.29 and the bill comes to

```
climb 720 + circ 39 + deorbit 40 + margin 100 = 899   vs   892 in the tanks
```

**Short by 7 m/s.** So even with every fix above, the script writes off an orbit
the pilot then flew by hand. Seven metres per second.

The seven are an artefact. Every ΔV figure in the script is a reading on the ship
*as it is now*, cargo aboard. The deorbit burn is not: it is flown after the
payload is released, on a ship less than half the mass, where the same tonnes of
propellant are worth roughly **1.9×** as much. The flight is the proof — it
reached orbit holding **54 m/s**, under the 158 m/s reserve the script demands,
released 72 t, found itself holding **103 m/s**, and deorbited on it comfortably.

Priced at the mass the burn is actually flown at, that 140 m/s of reserve is
4.0 t of propellant, which is **76 m/s** on the gauge at 33.7 km:

```
climb 720 + circ 39 + reserve 76 = 835   vs   892   ->   affordable by 57
```

**Fix.** `reserveDvFor()` converts the post-release requirement into its
heavy-mass equivalent, gated by `DEPLOY_BEFORE_DEORBIT` (default `TRUE`). The
budget lines mark the converted figure with `*` and the closed-cycle print spells
out what it means. The final funding verdict does the same conversion, so a ship
holding 54 m/s is no longer told it is stranded when it is not.

`DV_GLIDE_RESERVE` (120 m/s) is deliberately left alone as an absolute floor, so
in practice the ship still arrives holding ~120 m/s rather than the ~76 the
deorbit strictly needs.

*Set `DEPLOY_BEFORE_DEORBIT TO FALSE.` for any flight that might have to bring
the payload home. Even then the deorbit stays funded — 76 m/s covers a 40 m/s
burn — but the margin is the thing you lose.*

### 2. Hand over earlier — the jets are ridden past the point of profit

```
alt 21.2 km | 1296 m/s | acc 3.21 | vs 67/65  | cost/m/s: jet 16.7 vs rocket 128
alt 22.8 km | 1370 m/s | acc 1.79 | vs 34/48  | cost/m/s: jet 18.2 vs rocket 108
alt 23.2 km | 1387 m/s | acc 1.12 | vs 21/17  | cost/m/s: jet 24.8 vs rocket 108
alt 23.4 km | 1405 m/s | acc 1.26 | vs -2/5   | cost/m/s: jet 20.5 vs rocket 103
alt 23.2 km | 1417 m/s | acc 0.52 | vs -13/0  | cost/m/s: jet 50.3 vs rocket 114
```

The priced switch test compares **fuel per m/s of speed**, and by that measure
the jets are still 2.3× cheaper at the switch (50.3 vs 114 kg/m/s) — so the test
never fires. What ends the phase is `SW_MIN_ACCEL`, at 0.12 m/s².

But look at the vertical speed column: the last 1.6 km of jet climb turned +67
m/s of climb into **−13 m/s of sink** while buying 121 m/s of airspeed. The
rocket then inherits a *descending* ship at TWR 1.13, in air thick enough to
charge 0.2 atm, and has to arrest the sink and climb out — which is the ×8.2
opening. The switch test prices the fuel and ignores the **state** it hands over.

Cheapest experiment: leave the code alone and set `SW_MIN_ACCEL` to ~1.5 m/s², or
`SW_ALT_HARD` to 22000. Either hands over while the jets still have climb
authority. The instrumentation added in this PR will show whether the opening
gets cheaper.

### 3. The airframe — 27% payload fraction is the real constraint

RAPIER SSTOs to LKO typically manage 10–15%. Everything above is recovering
margin the design does not have. In rough order of value:

- **+0.6 t of LF-only** (121 units). The jets ate **107 units of paired LF** —
  rocket ΔV burned at jet Isp. Cheapest fix on the list, worth ~19 m/s, and it
  costs almost nothing.
- **More thrust, not more fuel.** Closed-cycle TWR at handover was **1.13**. The
  opening loss is a TWR problem as much as a drag problem: a 31-second burn at
  1.13 spends its whole first minute unable to climb and accelerate at once. Two
  or three more RAPIERs change the shape of the opening in a way no guidance
  change can.
- **Or ~10 t less payload**, or a **~90 km target**. The ship made 92 km by hand
  with fuel to deorbit; that is the orbit this airframe currently has, and
  `RUN ascent(90000).` asks for it honestly instead of discovering it at 29 km.

### 4. What is already fine

- The **air-breathing phase is efficient** — it holds its Q corridor at 0–5° of
  AoA and 9–10 m/s² of acceleration for most of the climb, and delivers 2932 m/s
  of jet ΔV for a 1418 m/s handover. Do not tune it.
- The **LF-only reserve accounting works**: rocket ΔV *rose* from 1432 to 1506
  during the jet climb, exactly as intended — the jets burning unpaired LF
  lighten the ship without touching a paired unit.
