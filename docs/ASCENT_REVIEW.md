# Ascent review — reference flight, Mk3 Titan, 100 km target

Written against the flight that ended with **apoapsis 78.05 km, periapsis
−100.61 km** and the autopilot printing *"ORBIT ACHIEVED"*. Every number below
is from that flight's own console output.

## What actually happened

| | planned | actual |
|---|---|---|
| Handover altitude / speed | 20 km / 1450 m/s | **22 km / 1260 m/s** |
| Jet-phase dV | 3000 m/s | **3637 m/s** |
| LF burnt by the jets | 24.6 t | **29.6 t** |
| Rocket dV at handover | 1503 m/s | **1316 m/s** |
| Rocket climb losses | ×1.50 | **×3.21** |
| Final orbit | 100 km circular | 78 × −101 km (**reentry**) |

The jet phase spent **21% more fuel to arrive 190 m/s slower**. That extra 5.0 t
of liquid fuel, paired 9:11, is 11.1 t of rocket propellant — **≈142 m/s of
rocket dV** on a 240 t ship. Combined with the 190 m/s speed shortfall, the
air-breathing phase alone accounts for roughly **330 m/s**. The gap at MECO was
598 − 337 = **261 m/s**. Fixing the air-breathing profile closes it on its own.

## The root cause was not switching too late

This was the first suspicion, and it is worth being precise about, because the
fix is different from what it looks like.

Priced properly, the dive from 24.1 km to 22.0 km *made money*: it consumed
169 m/s of rocket dV and bought 240 m/s of airspeed plus 16 m/s of altitude
equivalent — a return of **1.5 m/s per m/s spent**, at a moment when the rocket
was returning about 0.3 (losses were ×3.21). Air-breathing was still by far the
cheaper way to buy speed. Hanging on was right.

What was wrong is that the ship *should never have been at 24 km doing 1000 m/s*
and needing to dive at all.

### Defect 1 — the profile climbed instead of accelerating

`AB_ACC_HIGH` was 3.0 m/s². With 1333 kN on 270 t the ship makes ~4.9 m/s² off
the runway, so the "spare acceleration buys altitude" rule ramped the commanded
climb by 10 m/s **every second** straight to its clamp. The log shows the nose
pegged at the 28° `AB_PITCH_MAX` limit from 13 km to 21 km, and Q falling away
the whole time:

```
alt 10.3 km   496 m/s   Q 0.39   pitch 21.2   thr 100%
alt 16.5 km   630 m/s   Q 0.32   pitch 28     thr  84%
alt 19.8 km   690 m/s   Q 0.23   pitch 27.4   thr  97%
alt 24.1 km  1020 m/s   Q 0.06   pitch 13.1   thr  25%
```

The ship traded its thrust for altitude and arrived somewhere the intakes
couldn't feed the engines. **Spare thrust is a reason to go faster, not higher.**

### Defect 2 — the switch test was deadlocked

`SW_ARM_SPEED = 1150` disabled all thrust and acceleration sensing below
1150 m/s of airspeed. At 24.1 km the jets were at 25% of peak thrust with no
acceleration left — `decayed` was true, `gasping` was true — but the ship was
only doing 1020 m/s, so **the switch was disarmed**. A stalling ship is slow by
definition; gating the stall detector on speed is a catch-22.

The 20-second `SW_STALL_TIME` backstop didn't fire either, because it required
`vsCmd <= vsFloor + 1` and the guard below kept moving `vsCmd`.

### Defect 3 — the "too thin to breathe" guard dived the wrong way

```kos
IF SHIP:Q < AB_Q_MIN AND SHIP:ALTITUDE > 14000 { SET vsCmd TO MIN(vsCmd, -10). }
LOCAL vsFloor IS 0.
IF SHIP:ALTITUDE > AB_DIVE_ALT { SET vsFloor TO AB_VS_DIVE. }   // dive allowed ABOVE 15 km
```

Descent was permitted **only above 15 km** — i.e. exactly where a dive is most
expensive (re-entering thick air at Mach 4) and forbidden where it is cheapest
(a shallow push through the transonic drag rise down low). That guard is what
produced the 24 → 22 km dive, and its interaction with Defect 2 made it a loop:
dive to gain the speed that arms the switch, but gaining speed means re-entering
denser air, which restores thrust above the 45% decay threshold, which disarms
the switch again. The magic `14000` was also inconsistent with `AB_DIVE_ALT`
(15000), so between 14 and 15 km the commanded dive was clamped straight back
to zero.

### Defect 4 — the rocket phase steepened when it had thrust to spare

`CC_ACC_HIGH = 5.0` with `CC_BIAS_UP = 6` meant high acceleration bought a
*steeper* climb. Late in the burn acceleration reached 11.3 m/s², so the bias sat
saturated at +6° for the whole phase:

```
AP 38.6/100 km   alt 29.4 km   1587 m/s   fpa 12.5/17.8   pitch 31.8   acc 6.2
AP 66.1/100 km   alt 40.5 km   1869 m/s   fpa 14.3/12.5   pitch 24.5   acc 11.3
```

This is backwards. Above the drag layer a steeper climb costs cos(γ) of the
thrust *and* g·sin(γ) of gravity loss to buy altitude an engine burn buys more
cheaply as orbital speed. At MECO the ship was carrying **461 m/s of vertical
velocity** — bought at rocket prices, pointed the wrong way — while being
471 m/s short of circular horizontally.

## Inconsistencies found alongside the profile

**The final report was wrong.** `abortSuborbital` only tested apoapsis
(`orbitAlt < ATM_TOP + 2000`). With apoapsis at 78 km that passed, so a
trajectory with periapsis 100 km *underground* was reported as
`ORBIT ACHIEVED (settled below the requested 100 km)`, followed by
`Deorbit funded but with no margin — deorbit promptly`. Apoapsis alone never
establishes an orbit; only periapsis clear of the atmosphere does.

**The deorbit reserve was protected on a reentry path.** Circularisation stopped
with 137 m/s in hand to fund a 96 m/s deorbit burn from an orbit the ship was
never in. You cannot simultaneously be short of circularisation dV and need
deorbit dV — holding that fuel back only guaranteed a steeper reentry.

**MECO fired on a floor that couldn't help.** Floor 1 cut the engine when
`dV(337) ≤ circ(458) + reserve(140)`. But 337 < 458: the circularisation was
already unaffordable, so there was no orbit to reserve deorbit fuel for, and
cutting the engine changed nothing except leaving 337 m/s unspent.

**"Cannot reach orbit" was declared, then ignored.** The warning fired at ~29 km
with 943 m/s left, and the ship kept climbing to a 78 km apoapsis anyway,
spending 606 m/s to make the circularisation it couldn't afford *more*
expensive. Raising apoapsis when you cannot circularise is strictly harmful.

**"GO with 26 m/s in hand"** on a 1477 m/s bill is 1.8% — inside the model's own
error bars, not a margin.

**Centrifugal relief was ignored.** At MECO, v_h²/r was 5.12 m/s² against 8.61
m/s² of gravity. Anything reasoning about how long the ship keeps coasting
upward using plain `g` is out by more than a factor of two.

## What changed in `scripts/ascent.ks`

**Air-breathing now flies a dynamic-pressure corridor.** Q is the air the intakes
are being fed, so holding Q is the same thing as staying where the jets work.
The command is derived, not searched: since Q = ½ρv² and ρ ~ e^(−h/H),

```
d(ln Q)/dt = 2a/v − vs/H        →        vs = H · ( 2a/v + Kp·ln(Q/Q_target) )
```

H is measured off the body's own pressure curve each sample, so nothing is tuned
to Kerbin. This produces the classic spaceplane ascent for free — brisk climb
while slow and low, then a long shallow acceleration drifting up as speed builds.
Spare thrust still buys climb, but only the climb needed to keep Q from rising.

**The mode switch is priced, not thresholded.** Both modes are costed in tonnes
of rocket-equivalent propellant per m/s of speed bought, every sample. Drag is
read out of the force balance the ship is already flying —
`D = T − m(a + g·sin γ)` — so the closed-cycle estimate is the same airframe at
the same Mach and Q with only the engine changed. Once the LF-only reserve is
gone, a tonne of liquid fuel also strands 11/9 t of oxidiser, so the jet's price
rises by 20/9 at that moment and the comparison knows it.

Checked against the logged flight, this fires exactly where the old test
deadlocked and nowhere earlier:

| state | jet | rocket | verdict |
|---|---|---|---|
| 16.5 km, 630 m/s, 84% thr | 16.7 kg/(m/s) | 101.6 | keep breathing |
| 22.0 km, 1238 m/s, 46% thr | 15.5 | 80.2 | keep breathing |
| 23.3 km, 1050 m/s, 33% thr | 42.7 | 92.4 | keep breathing |
| **24.1 km, 1020 m/s, 25% thr** | **236.0** | **95.2** | **switch** |

**Dive rules inverted.** Descent is now allowed only below `AB_DIVE_ALT_MAX`
(12 km) and above `AB_DIVE_SPEED` (280 m/s) — cheap and transonic. Thin air high
up is no longer a reason to dive; it is the end of the envelope, and the
economics say so.

**The rocket phase never steepens.** `CC_BIAS_UP` is gone; the bias only shallows.
A second independent cap decays the commanded flight path from the handover
angle to `CC_FPA_FLAT` (2°) by `CC_FLAT_ALT` (45 km), whichever is shallower
wins, and the initial angle is derived from handover TWR
(`24 − 8·TWR`, clamped 5–20°) rather than fixed at 15°. The only floor is an
apex-time guard using v_vertical / g_effective, with g_effective *net of
centrifugal relief*.

**Emergency mode.** When the orbit is priced out, apoapsis stops being the
objective: the ship levels off and spends everything above the glide reserve on
periapsis, which is what buys a survivable reentry.

**Floors corrected.** Deorbit fuel is reserved only when there is going to be an
orbit to deorbit from — i.e. when circularisation is still affordable and once
periapsis is actually clear of the air.

**Reporting corrected.** `inOrbit` is `SHIP:PERIAPSIS > ATM_TOP` and everything
is reported against it. A suborbital arc now says so and explicitly tells you not
to run `deorbit_land`.

**Pre-flight recalibrated.** `PLAN_LOSS_FACTOR` 1.50 → 1.75, and `PLAN_JET_DV` is
derived from take-off TWR rather than fixed at 3000 (a 0.5-TWR airframe pays drag
for every second of a very long acceleration). A margin under `PLAN_GO_MARGIN`
(8%) now reports **MARGINAL**, not GO.

## Verification, and its limits

`tools/check_kos.py` passes. The control law was also flown in a point-mass sim
against the old one. Two honest caveats:

- The sim reproduces the **shape** of both laws well — the old one hunts
  (`vsCmd` swinging 70 / 0 / −20 with thrust collapsing, exactly as in the log),
  the new one locks Q to target within ~90 s and gives a smooth monotone
  command with no oscillation. That is the stability result worth having.
- It does **not** reproduce the reference flight's absolute numbers (it stalls
  around 16 km where the real ship reached 24), so no quantitative claim about
  the new handover speed is made here. The +330 m/s figure at the top comes from
  the flight log's own arithmetic, not the sim.

---

# Part 2 — comparison with the PR #2 profile

The script on `refs/pull/2/head` reached 100 km and deorbited on this airframe.
It is 140 lines with no feedback at all, and it beat the adaptive version. That
is worth taking seriously rather than explaining away.

## What PR #2 does

```kos
SET AB_CLIMB_PITCH  TO 12.          // constant pitch, whole air-breathing phase
SET SWITCH_ALT      TO 20000.       // switch at 20 km ...
SET SWITCH_SPEED    TO 1400.        // ... or 1400 m/s, whichever comes first
SET ROCKET_PITCH_HI TO 25.
...
LOCAL frac IS SHIP:APOAPSIS / TARGET_APOAPSIS.
LOCAL pitchCmd IS MAX(5, ROCKET_PITCH_HI * (1 - frac) + 5).
```

Three constants and a linear fade. No dV budget, no feasibility check, no
re-targeting, no floors, no state estimation.

## Why it won

**It flew a shallow climb.** This is the whole thing. A flat 12° pitch is roughly
an 8° flight path at cruise AoA:

| airspeed | PR #2 (~8° path) | reference flight (28° pitch) |
|---|---|---|
| 400 m/s | 56 m/s | 163 m/s |
| 800 m/s | 111 m/s | 325 m/s |
| 1200 m/s | 167 m/s | 488 m/s |

At 800 m/s and 1.5 m/s² of acceleration, PR #2 climbs **7.4 km** per 100 m/s of
speed gained. The reference flight climbed **21.7 km**. That is why one arrived
at the switch with air still going into the intakes and the other arrived at
24 km with the jets at 25% thrust.

**Its switch always fires.** `alt > 20 km OR airspeed > 1400` cannot deadlock.
The adaptive version's arming gate could, and did.

**It has nothing that can refuse to make progress.** Every failure mode found in
Part 1 — the disarmed switch, the dive loop, the MECO floor that cut the engine
without helping, the "cannot reach orbit" warning followed by 606 m/s of
climbing anyway — is a *decision* the simple script never makes. Complexity that
can veto progress is worse than no complexity.

## Where PR #2 is genuinely worse

Not things to copy back:

- `STAGE.` unconditionally — on a craft with a payload separator, that drops the
  cargo on the runway.
- **`resAmt()` is broken**: `SET t TO r:AMOUNT` overwrites instead of
  accumulating, so the fuel report shows only the last tank found.
- Counts payload fuel as its own, and lets the engines drink it.
- No dV policing anywhere. It will burn the tanks dry to circularise and leave
  nothing to deorbit with — it only got home because there happened to be fuel
  left.
- `PRINT "ORBIT ACHIEVED"` unconditionally, with no periapsis check. Same bug as
  Part 1, in a script that just cannot notice.
- Kerbin hardcoded throughout: 70000, 68000, 99000, 20000.
- Commands *pitch*, not flight path, so its climb angle silently changes with
  mass and air density. It works on the airframe it was tuned for.

That last point is the real trade. PR #2's 12° is right for this ship; it is not
derived from anything, so on a different SSTO it is a guess.

## What was changed as a result

**The climb cap came down to PR #2's proven band.** `AB_FPA_MAX` 22° → **12°**,
and it is now clearly separated from `AB_PITCH_MAX` (28° → 25°). These are
different quantities: flight path is the physics that decides whether the jets
get fed; pitch is mostly the angle of attack the trim needs to fly at all.
Conflating them is how the reference flight sat on a pitch limit and called it
guidance. The old `AB_FPA_MAX` of 22° was barely better than the 28° that failed.

**The switch is now guaranteed to terminate.** The priced test is still the
primary decision, but it provably does not always fire: in level flight the jets
stay genuinely cheaper per m/s down to a crawl, so on paper the ship cruises
forever buying 0.2 m/s². Three backstops now bound it, in PR #2's spirit:
`SW_ALT_HARD` 28 km → **25 km**, a new `SW_Q_HARD` at **0.04 atm** (the intakes
are starved regardless of what the economics say), and `SW_MIN_ACCEL` at
0.25 m/s². In simulation the acceleration floor fires where the priced test
alone would have cruised indefinitely.

**The Q target is no longer a constant.** This is the answer to "it is
hardcoded", and it applies to *my* 0.30 as much as to PR #2's 12°. The right
dynamic pressure depends on how a build's drag trades against its intake area,
which the script can measure. An outer loop now trims the target every 10 s
against sustained acceleration — plenty of acceleration means the ship can
afford thinner air, sagging acceleration means it is starving and must come back
down — clamped to 0.10–0.45 atm and only trimmed while the inner loop is
actually holding the corridor, so the two cannot wind against each other. In
simulation it pins against a clamp rather than running away when the airframe
cannot meet the acceleration target, which is the correct degenerate behaviour.

**A post-flight sizing report.** The pre-flight check answers "what does this
ship need?" from a model. After the flight every assumption in that model has a
measured counterpart, so the same solvers are re-run with the real jet fuel
fraction, the real handover state and the real loss factor. It prints the
calibration constants to paste back, then concrete advice — add this much LF, or
this much LF/Ox plus tankage, or fly this much lighter, or set
`REQUESTED_APOAPSIS` to the orbit the ship can already afford. If losses came out
above ×2 it says so and points at TWR, because no profile change fixes that.

## The synthesis

PR #2 is a good *envelope* and a bad *autopilot*. The closed-loop version is the
right architecture — it is the only one that can measure a ship it has not seen
before and tell you what is wrong with it — but it needed PR #2's discipline
bolted on: bounded attitude, guaranteed termination, and no test that can veto
progress. Every adaptive decision in the script now has a fallback that fires on
its own, and the two hard limits it flies inside (12° climb, 25 km ceiling) are
the ones the working profile demonstrated.

---

## The part the script cannot fix

The vehicle is the binding constraint. 72 t of payload on 270 t is a **27%
payload fraction**; RAPIER SSTOs to LKO typically manage 10–15%. Closed-cycle TWR
at handover was 1.15, which means a ~90 second insertion burn and **~200 m/s of
gravity loss that no guidance law can avoid**. The tanks held 1339 m/s of paired
propellant on the pad against a 1477 m/s bill that the pre-flight check itself
computed.

With both profile fixes this airframe should reach a low orbit. It will not reach
100 km with deorbit margin at this payload. Lower `REQUESTED_APOAPSIS` to ~80 km,
or take roughly 10 t out of the payload.
