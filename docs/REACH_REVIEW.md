# Reach review — why the ship believed it was long while it was falling short

A reference flight of `deorbit_land.ks` (deorbit at MET 02:24, interface at
02:26) crossed the atmospheric interface 811 km from the KSC, reported
`energy 1.98` — nearly twice the range it needed — continuously from 47 km down
to 21 km, and then handed the glide a ship that was **25 km of energy height
short** with 271 km still to fly. It phugoided into the mountains and stopped
150.6 km from the runway.

The entry was not unlucky. It was flying on two numbers that were wrong in the
same direction, and it had no way to find out until the moment the arithmetic
stopped being about the future.

## What the log says

| Phase | Sample | Reading |
|---|---|---|
| Entry | interface | alt 70 km, spd 2092 m/s, **811 km** to the runway, `energy 1.27`, `L/D 0` |
| Entry | 63 km | `AoA 40/47`, `energy 1.51`, **`L/D 10`** |
| Entry | 54.8 km | `AoA 40/66`, `energy 1.72`, `L/D 6.6` |
| Entry | — | `holding 70 deg off the airstream against a 46 deg command — entry alpha capped at 38` |
| Entry | — | `stalled at 2076 m/s (79 deg off the airstream)` |
| Entry | 47.7 km | `AoA 36/80`, **`energy 1.99`**, `L/D 7.8` |
| Entry | — | `stalled at 2018 m/s (106 deg off the airstream)` |
| Entry | 40.3 km | `AoA 34/101`, `energy 1.99`, `L/D 5.8` |
| Entry | 29.3 km | `AoA 32/69`, `energy 1.98`, `L/D 4.3` |
| Entry | — | `stalled at 1078 m/s (71 deg off the airstream)`, 21 053 m AGL |
| Glide | handover | alt 20 877 m, **`energy 36754/61671`**, spd 558, **vs −284 m/s** |
| Glide | 5 s later | `rwy 271.8 km` |
| Glide | 9.8 km | `energy 12605/59813`, **`rwy 271.3 km`** |
| Glide | 7.0 km | `energy 7063/59397`, `rwy 269.3 km` |
| Glide | 3.9 km | `holding the 3656 m deck`, stall at 82 → recover at 121 → stall, repeating |
| Report | — | `Out of height 150.6 km from the runway — no approach to fly.` |

Two things are visible without any modelling. **Five departures between 70 km
and 21 km** — the entry spent the whole hypersonic phase tumbling, at 66°, 80°,
101°, 69° off its own airstream. And between the handover and 7 km the ship
spent **29.7 km of energy height to cover 11 km of ground**: a lift-to-drag
ratio of 0.37. It was not gliding. It was falling with a heading.

## Fault 1 — the L/D was measured where there is no air

`ldSample()` began sampling at the interface. At 60–70 km the ship spends almost
no energy per five-second sample while covering 10 km of ground, so the ratio
pegged at the clamp — the log prints `L/D 10` at 63 km — and the 0.35 filter then
needed a dozen honest samples to walk it back down. It read 5.8 at 40 km. The
entry was really flying **1.1**.

Because `rangeCapability()` uses `MIN(ldNominal, gLdMeas)`, a *high* measurement
is not conservative — it hands the decision straight back to the nominal model,
which credits a slowing ship with a rising L/D at exactly the rate the extra drag
is destroying it.

Fixed by gating the sample on `LD_MIN_Q` of dynamic pressure — and dropping the
baselines while below it, so the first real sample is measured across air rather
than across the coast that preceded it — lowering the clamp to `LD_MEAS_MAX`, and
seeding the filter from the model rather than from a single flattering sample.

## Fault 2 — the altitude column was priced at the glider's ratio

`rangeCapability()` split the energy budget at the handover *speed*: everything
above `ENTRY_END_SPD` was the entry's, everything below was the glider's. That
leaves the whole altitude column in the glider's half. At 40 km and Mach 5 it
credited 40 km of height at 4.5 to 1 — **180 km of range from air the wing cannot
fly in**. That height is not glider energy; it is what has to be spent getting
down to where the wing works.

Split at `GLIDE_CEIL` instead. The glider's share is the energy it will have when
it is actually gliding: at or below the ceiling, at the handover speed, and never
more than the ship has. Everything above is the entry's, spent at the entry's
measured ratio.

What the two fixes are worth, at the same three points on the same flight:

| Point | old | with an honest L/D of 1.1 | + ceiling split |
|---|---|---|---|
| 40.3 km, 1746 m/s, 366 km to go | `1.95` long | `1.16` long | **`0.97` short** |
| 29.3 km, 1097 m/s, 302 km to go | `1.27` long | `0.90` short | **`0.80` short** |
| 21.0 km, 558 m/s, 271 km to go | `0.61` short | `0.61` short | `0.61` short |

The old code was correct only at the last row, which is the handover — the exact
moment there is nothing left to do about it. The new one is short at 40 km, with
the whole entry left to fix it in.

## Fault 3 — the entry's default attitude was the dump attitude

`REENTRY_AOA` was 40°, held whenever airspeed was above 2000 m/s. That is the
configuration the reference data credits with **530 km** of range; flown clean the
ship makes 880 km. It was the default posture for meeting the atmosphere.

The airframe could not hold it. Commanded 40° at 2092 m/s the nose went to 46,
51, 66, then 70 — and the guard that exists for this walked the cap down 2° per
2 s, reaching 38 by the time the ship was 79° off the airstream and gone. A 24°
overshoot against a 12° tolerance was answered with a 2° correction.

Three changes: the entry's *nominal* alpha is now `ENTRY_AOA_NOM` (28°) with
`REENTRY_AOA` retained only as the ceiling the energy loop opens toward when
there is genuine surplus; the tolerances came in to 20/8; and the give-back is
proportional — an airframe 24° past an 8° tolerance is told to stop asking for
16° of alpha, not for 2.

This is also the direct answer to *stay higher, do not slow down so
aggressively*: less alpha is less drag is a shallower, faster, longer entry.

## Fault 4 — the terrain deck was a stall order the ship could not refuse

At 3.9 km with no energy left, the glide held a `3656 m deck` — commanding a
climb, stalling at 82 m/s, diving, recovering at 121 m/s, climbing, stalling,
over and over, all the way into the ridge. The predictive deck is the right idea
and it is the reason two earlier flights are not repeated; but a wing that is
barely flying cannot be sent over a mountain, and ordering it to try is not a
terrain guard.

The predictive deck now stands down below `MANEUVER_Q`. The ridge then belongs to
the divert logic, which is where a ridge that cannot be climbed belongs.
`vsRadar` — the hillside directly underneath — is not gated and never will be.

## Fault 5 — there was exactly one place the script noticed it was not going home

`GLIDE_FLOOR`, 600 m above the runway. Everything before that was spent flying at
a field that had been out of reach for four minutes, and the "diversion" was a
heading hold over whatever happened to be underneath at that instant.

A glider that knows it is 100 km short has one real decision left — *where* — and
minutes of height in which to make it, fly to it, and arrive lined up. The glide
now keeps a live answer to "can I still get there": `rangeCapability()`, priced
off the L/D actually being flown, plus a deliberately pessimistic valuation of
the jet allowance (`JET_RANGE_PER_LF`; the reserve is not counted at all). Short
by `DIVERT_RATIO` for `DIVERT_COMMIT` seconds running is not a bad patch of air,
and the script picks somewhere it can reach.

Sites are scored on **how flat the ground actually is** — the spread of touch-down
height around a ring of radius `SITE_RING` — because a spaceplane arriving at
100 m/s needs a couple of kilometres that do not change height much and does not
care about elevation. Open water enters that same scale at `SITE_WATER_PEN`:
perfectly flat, which is most of what matters, but a ditching, so it loses to
real flat land and beats a hillside. A gentle `SITE_HOME_GAIN` breaks ties toward
the space centre. The run-in heading is then chosen from the terrain — the
smoothest corridor through the site, among headings within 90° of the way the
ship is already coming, because a low glider does not get to turn round for a
nicer field.

The divert is **not a latch**. It is re-tested every pass and the runway is taken
back the moment it comes into reach; the site itself is re-picked every
`SITE_RESCAN_T` while the shortfall stands, because the reach estimate sharpens
as the ship descends. Gear stays *up* for a planned ditching — a gear leg in the
water at 100 m/s is a pivot.

## Fault 6 — the recovery chased a tumbling velocity vector

What makes a recovery violent is not the dive. A nose held 2° off a steep
airstream is exactly what unloading the wing looks like, and it is correct. It is
the *heading*: `flightHdg()` reads the surface velocity vector, a tumbling ship's
velocity vector swings through the whole compass, and re-reading it every pass
hands the steering manager a new bearing several times a second.

The recovery now latches the heading it began with and walks it toward the
airstream at `RECOVER_HDG_RATE`, and only once the nose is within
`RECOVER_HDG_OK` of the air for the reading to mean anything. RCS is also gated
on `RCS_Q_OFF` — left on into dense air it fights the aerodynamic controls and
adds exactly the snatch the recovery is trying to take out.

And the steady window after a recovery is now flown *descending* unless the
ground itself is the reason to climb: straight out of a recovery the ship is at
the speed that just failed, and a commanded climb there is the way directly back
into the stall.

## Fault 7 — `geoOffset()` was flat-earth

Not implicated in this flight, but it became load-bearing with the site search.
The tangent-plane approximation is wrong by a tenth of a metre for a 9 km final
approach fix on the equator, which is what it was written for. The site search
lays candidates out to 150 km and beyond, and the error grows with the square of
the distance and with latitude:

| distance | equatorial | from a 20° return |
|---|---|---|
| 9 km | 0.1 m | 28 m |
| 60 km | 2.6 m | 1.3 km |
| 150 km | 16 m | **8.8 km** |
| 250 km | 45 m | **26 km** |

Scoring the flatness of ground eight kilometres from where the ship is actually
going is not a terrain scan, it is a coincidence. Solved on the sphere now.

## ENTRY_RANGE

Interface at 811 km, stopped 150.6 km short: **660 km of achieved range**, against
the 800 km the schedule assumed. Some of that deficit was the tumble and is fixed
above, but not all of it — 800 km has now been asked for three times and
delivered once, and a schedule that is wrong on the short side has no correction
available to it. Long does: boards, alpha, S-turns and, in the last 8 km, a
spiral.

`ENTRY_RANGE` is now **700 km** — the measured 660 plus a little of what the entry
fixes should hand back, leaving ~180 km of dumping authority against the
clean-flight figure. That is the side of the schedule that has a control on it.

## What to watch on the next flight

- `L/D` in the entry log should now start blank and settle near **1–1.5**, not
  near 10. If it still opens high, raise `LD_MIN_Q`.
- `energy` should cross below 1.0 *during* the entry if the flight is going to be
  short, not at the handover.
- The handover should arrive near `ENTRY_END_SPD` with a sane sink rate. A
  `vs −284 m/s` handover is a tumble, not a glide.
- If the ship now lands **long**, raise `ENTRY_RANGE` by the reported miss. The
  end-of-flight report prints the signed distance and the direction to trim.
