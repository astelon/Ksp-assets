# Circularisation review — why an 85 km order came back as 431 × 82 km

Written against the 85 km flight of the Perfect Cargo SSTO MkIV v5 that reached
orbit, spent **360 m/s on a 163 m/s burn**, and finished on a 431 × 82 km
ellipse it had been asked to make circular at 85. Every number below is from
that flight's own console output.

The flight also opened with a pre-flight check declaring the ship **short by 142
m/s** for a mission it then flew with fuel to spare. That is the second half of
this document, and the two halves are unrelated defects that happened to appear
on the same run.

## What the log says

```
MECO at apoapsis 85.3 km - apoapsis reached.
  dV 480 vs need 221 (climb 0 + circ 136 + reserve 85*) @ loss x1.23
Vacuum. ETA to apoapsis 158 s; circ burn ~9 s, starting 5 s early.
Warping 128 s toward the burn.
Out of warp. Settling on prograde.
Circularising at 85.3 km; PE target 83.8 km.
  Protecting 85 m/s here = deorbit 46 + margin 100 once the payload is off.
!! Circularisation stopped on the deorbit reserve.
   PE 81.6 km - the orbit stays elliptical, but the deorbit burn is still funded.

ORBIT ACHIEVED
  Apoapsis   : 431.13 km
  Periapsis  : 81.63 km
  dV left    : 120 m/s
```

A 9-second burn was quoted. 360 m/s went out of the tanks — enough for the burn
twice over and change — and the ship stopped only because it hit the fuel floor.

## Defect 1 — the burn was aimed at an apoapsis the ship no longer had

`orbitAlt` is frozen at MECO, and every target built on it is frozen with it:

```
SET orbitAlt     TO SHIP:APOAPSIS.              // 85.3 km, read at 30.4 km altitude
SET circPeTarget TO MAX(ATM_TOP + PE_SAFETY, orbitAlt - CIRC_PE_TOL).   // 83.8 km
UNTIL SHIP:PERIAPSIS >= circPeTarget { ... }
```

MECO fired at 30.4 km, doing 2151 m/s, with 387 kN of drag on the airframe one
report earlier. Between there and the top of the atmosphere is **40 km of air
flown at orbital speed**, and it keeps charging: the ship left the rocket phase
with 85.3 km of apoapsis and arrived in vacuum with about **81.6**.

Periapsis can never exceed apoapsis. An 83.8 km periapsis target against an
81.6 km apoapsis is not a demanding target, it is an **impossible** one, and the
loop that chases it has no other way out:

* the burn reaches circular at ~81.6 km, where periapsis stops rising;
* every m/s after that goes into **apoapsis**, which is the one number nothing
  was watching;
* `SHIP:PERIAPSIS >= circPeTarget` stays false for ever, so the loop keeps
  thrusting until the deorbit-reserve floor catches it, 200 m/s later.

The mid-burn screenshot is the proof: apoapsis **138.7 km**, periapsis **81.1
km**, time-to-apoapsis half a period — the ship sitting at the bottom of an
orbit it was still pumping energy into.

Reproducing it numerically from the flight's own state (174 t, 2517 kN, 81.6 km
apoapsis, 83.8 km target) gives 400 m/s spent and a 529 km apoapsis before the
floor stops it. The flight got 360 and 431. The mechanism is not in doubt.

**Fixes.**

1. **Hold the apoapsis while it can still be held** (`COAST_TRIM`, default on).
   Apoapsis is only purchasable from below it, so the coast between MECO and
   `ATM_TOP` is the last chance. A quarter throttle is pulsed against any deficit
   over `COAST_TRIM_TOL` (250 m), and only while the circularisation *and* the
   deorbit reserve are still funded without it. Roughly 40 m/s buys back the
   3.7 km this flight lost.
2. **Re-read the apoapsis before circularising.** Whatever the trim could not
   hold is gone; the burn is now planned against `SHIP:APOAPSIS` as it actually
   is on arrival, and the loss is printed rather than silently absorbed.
3. **Report it.** An orbit more than 1 km under the request now says so in the
   `ORBIT ACHIEVED` line instead of reading as a clean success.

## Defect 2 — a burn with no idea when to stop

Even with an achievable target, the old loop had exactly two exits: periapsis
reached, or fuel floor. Nothing in it could notice that thrust had stopped
helping. Two consequences, both visible on this flight:

* **The steering could not fix its own error.** `LOCK STEERING TO SHIP:PROGRADE`
  for the whole burn. A burn is not impulsive: the ship drifts past apoapsis and
  starts falling, and that radial velocity *is* eccentricity — at 2270 m/s
  circular, 30 m/s of descent puts periapsis 9 km under the ship. Prograde
  thrust cannot take it out, because prograde points down the descent.
* **The throttle ease-down fired 4 km out.** On an 85 km orbit the last 4 km of
  periapsis is the last **4 m/s** of a 163 m/s burn. Dropping to 15% throttle
  there stretched precisely the part of the burn where drifting off apoapsis is
  most expensive.

**Fix — fly the velocity error, not the periapsis reading.** `circBurnVec()`
returns the vector difference between the circular velocity *here* and the
velocity the ship has. It answers all three questions at once:

| quantity | old | new |
|---|---|---|
| where to point | prograde | along the remaining error — pitches up to arrest the descent |
| when to throttle back | periapsis within 4 km | over the last `CIRC_TAPER_T` seconds of burn |
| when to stop | periapsis ≥ target | error ≤ `CIRC_DV_TOL`, **or** it stops shrinking |

The give-up test is the one that matters. Past the circular point the error
turns around and starts growing, and that is unarguable evidence that the engine
is now making the orbit worse. `CIRC_AP_TOL` (apoapsis 5 km over target) says
the same thing more bluntly, as a backstop. Neither fires until periapsis is
clear of the air: below that there is no orbit to protect, and a ship still
climbing towards apoapsis does drag periapsis up with it even while it is over
circular speed. Until then, only the fuel floors stop the burn.

While the ship is still rising the aim keeps only the flat part of the
correction — the first half of the burn should not spend thrust cancelling a
climb the ascent has already paid for.

Simulated against the flight's own state:

| case | old | new |
|---|---|---|
| the flight as flown (81.6 km apoapsis, 83.8 km target) | 400 m/s, **529 × 82 km**, fuel floor | 132 m/s, **81.6 × 79.9 km**, normal exit |
| target and apoapsis agreed (85.3 km) | 135 m/s, 85.3 × 83.8 km | 135 m/s, 85.3 × 83.5 km |
| one third the thrust | 138 m/s, 88.4 × 83.8 km | 138 m/s, 87.1 × 83.8 km |

Where the old loop worked it is unchanged. Where it ran away it now stops.

## Defect 3 — the failure was reported as a fuel shortage

```
!! Circularisation stopped on the deorbit reserve.
```

It did, but only because a control defect had been feeding it fuel for two
minutes. The message reads as *bring more propellant*, and the ship had 120 m/s
in hand and had just wasted 200. A burn that stops because it stopped helping
now says so, prints how far from circular it gave up, and says plainly that the
fuel is intact.

## The pre-flight check: short by 142 m/s on a flight that had 170 to spare

```
!! NOT ENOUGH dV - SHORT BY 142 m/s
   (needs 1634, has 1491 at handover)
```

The same script, at the end of the same flight:

```
For a 85 km orbit with deorbit fuel, this flight says you need
1339 m/s at the handover and the design delivers 1507 m/s.
  => The airframe is sufficient (168 m/s spare). No changes required.
```

Two numbers, one flight, 295 m/s apart. Both are wrong in the same direction and
for two separate reasons.

### `PLAN_LOSS_FACTOR` was calibrated on a worse ascent

1.65 was measured end to end on a flight flown **before** the angle-of-attack
cap, which bought its apoapsis at 6–16 degrees of body drag. This flight, with
the cap in, measured **×1.36** over the whole climb by the script's own
yardstick — the same `spent / paid` ratio, over the same span, and the same
quantity `priceFromHandover()` multiplies. It spent 1019 m/s of rocket climb
against an impulsive 830.

1.65 now over-states this airframe's climb by roughly 250 m/s, which is most of
the gap the check reports. It is set to **1.36**, which is exactly what the
flight's own post-flight block advised.

### The reserve was priced against the wrong ship

`reserveDvFor()` was fixed for the in-flight budget in an earlier PR: the deorbit
burn is flown after 72 t of payload comes off, so quoting the post-release figure
against the loaded gauge reserves about twice the propellant the burn will use.
In flight this ship reads **85 m/s**, not 146.

`priceFromHandover()` — the pre-flight and post-flight solver — never got the
same treatment, and there is no live gauge on the runway to take a reading from.
It now estimates the mass at the end of the climb from the handover mass and the
dV about to be spent reaching orbit, and prices the reserve there:
`reserveFromHandover()`. For this airframe that is **80 m/s** rather than 146.

### Together

| | climb | circ | reserve | bill | vs 1491 |
|---|---|---|---|---|---|
| as flown | 1432 | 56 | 146 | 1634 | **short by 143** |
| fixed | 1180 | 56 | 80 | 1317 | **GO, 174 in hand** |

The fixed bill lands within 6 m/s of what the same script's post-flight solver
concluded from the flight's own telemetry, and the flight itself demonstrated
the margin by throwing 200 m/s away and still reaching orbit with a funded
deorbit.

## What is still open

**MECO is called on an apoapsis reading taken in the drag layer.** The coast trim
patches the symptom well, but the honest fix is for the rocket phase to know
what the remaining climb through the air will cost it and aim above the target
by that much. That needs a drag-decay estimate the script does not currently
carry, and it wants a flight's worth of `COAST_TRIM` telemetry — how much it
actually spends, at what altitudes — before it is worth attempting.
