# Path review — the glide that never went anywhere, and the mountain

The flight this documents is the best one `deorbit_land.ks` has flown and the
one that ends worst. The deorbit was on time, the entry survived the heat and
put the ship **89.7 km from the runway, 0.7 km off the centreline** — which is
the arrival every previous review was trying to buy — and then the glide spent
45 km of ground and most of the jet reserve holding 8 km of altitude, arrived
10 km from the KSC with three times the energy it needed, stalled, dived, and
hit the ridge line west of the field.

It is the sequel to `docs/GLIDE_REVIEW.md`. The rules that review installed all
held: the ship flew a dynamic pressure instead of an airspeed, both sides of the
profile comparison were energy heights, and the jets never once climbed at full
throttle. What failed is subtler and it is the same fault in four places — the
autopilot had no control over its **flight path**, and no idea the **ground**
was there.

## What the log says

| Phase | Sample | Reading |
|---|---|---|
| Entry | 56.3 km | `AoA 40/51 deg`, `** holding only 52 deg of the 40 asked for - entry alpha capped at 38` |
| Entry | 55.4 → 49.2 km | the same message four more times: cap walked 38 → 36 → 34 → 32 → 30 |
| Entry | 49.2 km | 333 km to the fix, `energy 2.29 L/D 4.4` — then the measured L/D collapses |
| Entry | 19.6 km | `energy 0.56 L/D 1.1` — reads badly short, stops dumping |
| Entry | handover | 648 m/s at 19.7 km, **runway 89.7 km ahead, 0.7 km off track** |
| Glide | first print | `energy 40985/19317` — 21 km of energy height LONG |
| Glide | 19.8 km | `** stalled at 449 m/s (58 deg off the airstream)` |
| Glide | recovery | `** still 111 deg off the airstream after 45 s - re-thinking` |
| Glide | 15.2 km | `** stalled at 154 m/s (83 deg off the airstream)` |
| Glide | 12.2 → 7.9 km | 4 380 m down in 25 s for 4.1 km of ground — a **45° dive** at 240 m/s |
| Glide | 7.9 → 8.6 km | **45 km of level flight**, `thr 24%`, `spd 191/190` |
| Glide | rwy 47 km | `energy 10374/9944` — the profile passes underneath, unnoticed |
| Glide | rwy 27.7 km | `energy 9208/5845`, `thr 100%` — 3.4 km long and adding thrust |
| Glide | rwy 10.1 km | `energy 8439/2638`, alt 7 609 m — 5.8 km of surplus, 10 km to lose it in |
| Glide | rwy 9.5 km | `** stalled at 95 m/s` → dive → 6 709 → 1 275 m in 5.5 km of ground |
| — | — | ground |

Five faults. The third one is the crash; the other four are how it got there.

## 1. A speed is not a flight path

The glide's whole nose law was one line:

```
LOCAL aoaCmd IS GLIDE_AOA + (spd - spdTgt) * AOA_PER_MS.
```

Angle of attack says what the ship is flying *at*. It says nothing about where
the ship is *going*, and with `AOA_PER_MS` 0.05 there is 1.5° of alpha in a
30 m/s speed error — not enough coupling to matter. The trajectory is therefore
free, and an aeroplane handed a free trajectory does not pick a good one; it
phugoids. The log shows both ends of the same phugoid:

* **12 233 m → 7 853 m in 25 s, 4.1 km of ground.** A 45° dive at 240 m/s, held
  for the whole 25 s because the speed was *below* target the entire time, so
  the only term in the law was asking for less alpha, not more.
* **7 853 m → 8 573 m over the next 45 km.** The bottom of the same phugoid,
  and then level flight, because at 190 m/s at 8 km the trim alpha happens to
  carry the weight.

Neither is a glide. The second is worse than the first, because the profile the
ship was supposed to be tracking is a function of *range*, and range was closing
at 190 m/s the whole time. `energy 9586/14924` at rwy 69.7 km is a ship 5 km
short. `energy 9208/5845` at rwy 27.7 km is the same ship, at the same energy,
3.4 km long. It never moved; the profile swept past underneath it.

**Changed.** The nose flies a **sink rate**, and the sink rate comes from the
energy error:

```
ldWant = PLAN_LD -> LD_DUMP    as the ship goes HIGH_MARGIN above the profile
ldWant = PLAN_LD -> LD_STRETCH as it goes LOW_MARGIN below it
vsWant = -horizontal speed / ldWant
aoaCmd = GLIDE_AOA + speed error * AOA_PER_MS
                   + clamp(±GLIDE_VS_AUTH, path error * GLIDE_VS_GAIN)
```

Both terms are bounded, `glideAoaMax()` still caps the result, and the stall
guard still outranks all of it. In steady flight at the plan's glide ratio the
two terms agree; when they disagree, the ship is off profile and the disagreement
is the correction. Level flight is now something the ship can only do on purpose.

## 2. Thrust for a ship that was neither slow nor short

The 45 km of level cruise was flown at `thr 24%` and it burned most of the
reserve. Two rules put it there and both were too generous.

The jets latched on for **any** wing below `MANEUVER_Q`:

```
IF ehNow < profEh - LOW_MARGIN OR SHIP:DYNAMICPRESSURE < MANEUVER_Q { gPowerOn TRUE }
```

Every stall recovery satisfies the second clause, so the ship came out of its
recovery at 12 km with the jets armed while six kilometres of surplus energy
height was still ahead of it. And once armed, the throttle carried a floor:

```
SET gThrot TO clampVal(0, 1, THROT_MIN_PWR + (spdTgt - spd) * THROT_PER_MS).
```

`THROT_MIN_PWR` is 0.25. With the speed exactly on target the ship burned 25%,
forever, and 24% is what the log prints for thirty kilometres.

**Changed.** Thrust needs a real energy deficit (`ehErr < -LOW_MARGIN`), a wing
that has genuinely stopped flying (`STALL_Q`, not `MANEUVER_Q`), or the ground.
The floor is spool-up and applies only in those cases; a ship that is above the
profile gets zero throttle whatever its airspeed, because the answer to slow and
high is to point the nose down, which the sink law now does.

## 3. The ground is not at runway elevation

This is the crash, and it is one substitution.

`energyHeight()`, the profile, the glide's floor and `stallRecover()`'s floor all
measured height as `SHIP:ALTITUDE - RWY_ELEV`: height above the **runway**. The
KSC runway is 68 m above the sea. The approach to it from the west crosses ridges
over 2 km high. "600 m of height to recover in" over one of those is 1 400 m
underground, and a descent that is exactly, correctly on the energy profile flies
straight through the ridge with every gauge reading green.

Which is what the last two minutes are. At rwy 10.1 km the ship is at 7 609 m
with 8 439 m of energy against 2 638 m needed — hopeless, and the only thing it
can do is dive. It stalls, `stallRecover()` dives it at `RECOVER_AOA` toward a
floor 600 m above a runway 10 km away, and 5 434 m of descent later, in 5.5 km of
ground, it is in the mountains.

**Changed, reactively.** `ALT:RADAR` — height above whatever is actually
underneath — now bounds the sink command directly:

```
vsRadar = MIN(TERRAIN_CLIMB, (TERRAIN_FLOOR - ALT:RADAR) / TERRAIN_TAU)
vsWant  = MAX(vsWant, vsRadar)
```

That is unbinding with height under the ship, bleeds the allowed sink to nothing
as it closes on `TERRAIN_FLOOR` (500 m), and becomes a commanded `TERRAIN_CLIMB`
below it. `stallRecover()`'s floor is radar altitude now, in every one of its
four call sites, and below `TERRAIN_ABORT` the glide stops being a glide and
hands over to the landing.

**And that is not enough, which is the whole point.** `ALT:RADAR` answers "how
high is the ground *under* me", and that is the wrong question by exactly the
time it takes to act on the answer. At 190 m/s a ridge face is eight seconds
wide, and 8 m/s of climb buys 64 m in those eight seconds against a face that
rises 2 000. A glider does not fly over a ridge. It arrives already above it, or
it does not arrive.

So the ridge is measured while it is still sixty kilometres away:

```
FUNCTION terrainPeakTo { PARAMETER geo, samples. ... }   // max terrain, here -> geo
deckAlt = scanTerrain(FAF) + MTN_CLEAR
deckEh  = (deckAlt - RWY_ELEV) + APPR_SPEED^2 / 2g
profEh  = MAX(profEh, deckEh)
vsDeck  = MIN(TERRAIN_CLIMB, (deckAlt - SHIP:ALTITUDE) / TERRAIN_TAU)
```

`MTN_SAMPLES` (20) points along the direct track to the fix, rescanned every
`MTN_SCAN_T` (2 s, which is 400 m of closure — the ridge does not move, only our
distance to it does). The highest of them plus `MTN_CLEAR` is a **deck**, and the
deck does two things:

* it bounds the sink command, on the same construction as the radar floor, and
  the tighter of the two wins;
* it raises the **energy profile** to whatever holding that deck costs. This is
  the half that matters. A ship that cannot pay for the ridge now reads short
  sixty kilometres out, which arms the jets while thrust is still worth
  something, instead of reading perfectly on profile right up to the impact.

The one thing still gated on the deck binding (`lowGnd`) is the S-turn, because
the scan runs along the direct track and 40° off it is 40° of ground nobody
measured. The boards are deliberately *not* gated on it: the deck is inside
`profEh` now, so a ship reading long is long after paying for the ridge, and what
it has spare while pinned at a fixed altitude is speed — which is exactly what
the boards are for.

The plan does not get a vote here. It cannot see.

## 4. Two measurements that were lying

Both are small and both changed the flight materially.

**The entry capped its own alpha for flying correctly.** `offBy` was
`noseOff() - gAoa` — the angle between the nose and the airstream, minus the
*alpha* asked for. But the command is an attitude, and the S-turn's command is
40° of alpha **and** 25° of lean, which is 47° off the relative wind when flown
perfectly. So every bank reversal read as a 7-12° failure to hold the command,
and after two seconds of it the entry gave a degree of alpha back. Five times:
40 → 38 → 36 → 34 → 32 → 30, and the drag the range plan had already spent went
with it. `offBy` is now `noseOff() - cmdOff()`, where `cmdOff()` is the angle
between the *commanded direction* and the airstream. Tracking error against the
command, which is what the test was always for.

**The entry priced the glide at the entry's L/D.** `rangeCapability()` took the
measured L/D — which settles near 1.1 during a high-alpha entry, correctly, most
of that energy leaves as heat — and applied it to the whole energy budget,
including the 70 km of altitude that was going to be spent gliding at 4.5. At
49 km it read `energy 0.56`: badly short. The entry did what it is told to do
when short, gave up alpha and stowed the boards, and then arrived over the
runway 21 km of energy height **long**. The budget is now split where the flight
regime splits it — kinetic energy above `ENTRY_END_SPD` at the measured entry
L/D, everything under it at `PLAN_LD` — and the same moment reads 1.52, long,
while there is still air to dump into.

## 5. Sixty degrees of sideslip is not a bound

Every stall in this flight happened on an energy-dumping turn command, and the
numbers say why. `setNav()` bounds the heading demand by `yawAuthority()`, whose
subsonic limit was `YAW_SUB_MAX` = 60°. But `dirFor()` points the nose at the
heading it is given, so that limit is **sideslip** — air across the fuselage —
not turn rate. The S-turn asks for `FAF:HEADING ± 40`, and 40° is inside 60°, so
the ship was handed 40° of sideslip in a single pass at 648 m/s. It departed at
58° off the airstream. The same command at 154 m/s put it 83° off.

The turn was never coming from the yaw. `setNav()` also sets
`gBank = hdgErr * BANK_PER_DEG`, and 15° of error is already 22.5° of bank
against a `BANK_MAX` of 30 — the bank saturates well inside the new limit, so
`YAW_SUB_MAX` = 15° costs no turn rate at all. It only stops the ship being
commanded to fly sideways.

## What to watch on the next flight

1. **`vs` in the glide line**, which is new. It should be negative and it should
   track roughly `horizontal speed / 4.5` — about −40 m/s at 190 m/s — steepening
   when the `energy` pair says high and shallowing when it says short. A `vs`
   sitting near zero for more than a few prints means the sink law is not
   authoritative enough: raise `GLIDE_VS_GAIN` or `GLIDE_VS_AUTH`.
2. **The two altitudes**, also new: `alt 7853/612 m` is height above the runway
   and height above the ground. Watch the second one on the run in from the west.
   If it pins at 500 and the throttle sits at 25% for a long stretch, the ship is
   terrain-following a ridge it should have crossed higher — which is a signal to
   raise `ENTRY_RANGE` no differently from landing short.
3. **`Highest ground on the way in`**, printed once as the glide starts, and
   `mtn` in every glide line. This is the number two flights have now died of not
   having. It should read a couple of thousand metres on a westerly approach and
   fall toward the runway elevation as the ridge goes behind. If it reads 0 all
   the way in, the scan is not finding terrain and the deck is doing nothing —
   check that the approach really is over water.
4. **`** high ground ahead ... holding the N m deck`** and its matching
   `** clear of the high ground`. One pair of these on a westerly approach is
   normal and is the system working. Several pairs in quick succession means the
   ship is porpoising against the deck: lengthen `TERRAIN_TAU`.
5. **`L/D` in the glide line**, which is now the *glide's* number and not the
   entry's — the filter is reset at handover. This is the direct answer to
   "it is having a hard time gliding". Clean, boards in, on speed, it should sit
   near `PLAN_LD` (4.5). If it settles well below that with the boards stowed,
   `PLAN_LD` is optimistic for this airframe and every profile in the script is
   built on it: lower it and the ship will start dumping earlier and arriving
   higher. It is reported only — nothing feeds it back into the profile, because
   feeding a measurement taken while dumping into the decision to dump is the
   loop that taught the entry to stop dumping because it was dumping.
6. **The `energy` pair converging.** The two numbers should close on each other
   as the range closes. Diverging — either way — is the fault this review is
   about, and it will now show up as a sustained `vs` at the `LD_DUMP` or
   `LD_STRETCH` limit rather than as a quiet cruise.
7. **`entry alpha capped at`.** It should be rare now, and when it fires the two
   angles it prints are nose-off-airstream against command-off-airstream. A gap
   there is a real trim failure and `REENTRY_AOA` should come down.
8. **`energy` at the glide handover.** This flight started the glide 21 km of
   energy height long. With the split capability model the entry should dump more
   and hand over nearer 1.0; if it now hands over *short*, `ENTRY_RANGE` (800 km)
   is the number to lower, not `ENTRY_LD`.
9. **Fuel remaining in the closing report.** The 24%-for-30-km burn should be
   gone entirely. Any significant jet time with the `energy` pair showing the
   ship at or above profile is a regression in the arming logic.
