# Reentry review — why the ship spun, stalled, and ditched

A reference flight of `deorbit_land.ks` (MET 00:29 deorbit, MET 00:38 at 46 km)
never glided at all. It tumbled through the reentry, arrived over the KSC with a
Mach number still on the clock, stalled at around 10 km, fell the rest of the way
with the jets running at full throttle, broke up in the water 19.5 km short — and
printed `STOPPED ON THE RUNWAY.`

This is the analysis of that telemetry and what was changed because of it.

## What the log says

The relevant numbers, straight from the on-screen log:

| Phase | Sample | Reading |
|---|---|---|
| Entry | interface | alt 70 km, spd 2086 m/s, **552 km** to the KSC |
| Entry | last print | alt 26.2 km, spd **959 m/s**, range 4 km — *and growing* |
| Glide | first print | alt 24 889 m, spd **822 m/s**, profile 4368 m, 16.1 km to the fix |
| Glide | 15 407 m | spd 200 m/s, **28.9 km** to the fix |
| Glide | 9 265 m | spd **76 m/s**, 29.9 km to the fix |
| Glide | 5 648 m | spd **38 m/s**, 30.2 km to the fix |
| Glide | 880 m | spd 181 m/s, 28.6 km to the fix |
| Report | — | `STOPPED ON THE RUNWAY. Distance to runway threshold: 19497 m`, LiquidFuel 0 |

Three things are visible without any modelling at all. The range to the target
*stopped decreasing* at 29.8 km and stayed there — the ship was not flying
anywhere, it was falling. The airspeed between 15 km and 5 km is 38–76 m/s, less
than half the 160 m/s glide speed and far below anything the wing can carry.
And the last line of the report is simply false.

## Five separate faults, in the order they fired

### 1. The entry handed a Mach 3 ship to a bearing-chasing glide law

The handover was `ENTRY_END_ALT (25 km) OR ENTRY_END_SPD (700 m/s)`, *whichever
came first*. Altitude came first, at **959 m/s**. The entry loop bounded its
heading command to `ENTRY_YAW_MAX` off the airstream for a documented reason —
"a hypersonic ship told to point at a bearing 90 degrees off its own track has no
wing left to fly on and simply tumbles" — and then handed the ship, still at
Mach 3, to a glide loop that had no such bound and commanded `FAF:HEADING`
directly, with up to 30 degrees of bank.

### 2. …and the fix was *behind* the ship when it did

Entry range ran 552 → 1 km and then back up to 4 km: the ship crossed the field
at 26 km. So the first thing the unbounded glide law asked for was very nearly a
180-degree turn, at 822 m/s. That is the spin.

### 3. Nothing stopped it S-turning at the stall

Speed collapsed. At 10 km the ship was still "above profile", so it kept the
airbrakes out and kept commanding ±40-degree S-turns with 30 degrees of bank —
at 113 m/s, and then at 76, and then at 38. The one guard present
(`IF AIRSPEED < GLIDE_SPEED - 20 { BRAKES OFF }`) retracted the boards and left
the bank and the turn demand untouched.

### 4. The stall guard was defeated by the pitch clamp

`aeroSteer()` holds AoA on the airstream, which is exactly right, but its output
went through `dirFor()`, which clamped absolute pitch to `PITCH_MIN = -60`.
Falling at a flight path of −80 degrees with a commanded AoA of 2, the ship asked
for −78 and got −60 — i.e. **+20 degrees of real angle of attack, deep in the
stall**, from the code whose entire job was to prevent that. The script's own
header warns against absolute pitch commands; the clamp was one.

### 5. The jets were armed by the wrong test, and the report lied

Thrust was only allowed below `JET_ARM_ALT` (9 km) *and* below profile, so it lit
at ~6.3 km on a ship already stalled at 70 m/s — the least useful moment
possible — and ran the LiquidFuel reserve to zero. Then the glide exited on its
`agl < 600` floor 28.6 km from the fix, printed `Runway captured. Final approach
on heading 90`, flew a 5-degree glideslope at open water, and reported success.

## What the same telemetry says the ship can do

The overshoot is measurable. Over the field the ship had 26.2 km of altitude and
959 m/s, an energy height of

```
26 200 + 959² / (2·9.80665) = 73.1 km
```

worth another `73.1 × 4.5 = 329 km` of glide at the planned L/D. Its true range
from the interface was therefore `552 + 329 = 881 km`, against the 552 km the
118-degree lead angle gave it. Two constants come straight out of that:

* **`DEORBIT_LEAD` 118 → 149.** 881 km is 84 degrees of Kerbin ground track
  (10.47 km/deg), and the interface sat at 52.7. The burn has to happen ~31
  degrees earlier.
* **`ENTRY_LD` = 3.0.** 881 km flown for 292 km of energy height at the
  interface. This is *not* the glide ratio — most of the energy at Mach 6 leaves
  as heat, never as distance — and it is what lets the entry price its own range
  honestly while still hypersonic.

## What changed

| Fault | Fix |
|---|---|
| Handover at Mach 3 | Entry ends on **airspeed** (`ENTRY_END_SPD` 650); `ENTRY_FLOOR` 15 km is a backstop, not an alternative |
| Unbounded heading demands | **Every** phase now goes through `setNav()`, which folds the demand into the airstream on a speed schedule (25° of lean hypersonic, 60° subsonic) and limits bank the same way, to zero at the stall |
| S-turning near the stall | Energy dumping requires `MANEUVER_Q` under the wing; brakes, bank and turn demand all drop together |
| Stall guard eaten by the clamp | `PITCH_MIN` −60 → −85, so a recovery dive along a near-vertical flight path is flyable and the AoA law always wins |
| No recovery at all | `stallRecover()` outranks navigation: below `STALL_Q` of dynamic pressure for 1 s, or with the nose >`DEPART_AOA` off the airstream for 3 s, it unloads the wing, levels the wings, lights the jets, turns RCS back on for attitude authority at 40 m/s, and dives until the ship flies again |
| Jets armed too late | Armed on the pressure under the wing as well as on the profile, under a `JET_ARM_ALT` raised to 12 km, so thrust arrives while it is still worth something |
| Arriving over the field with energy | The entry now prices `rangeCapability()` against range-to-go every pass and holds more alpha, or S-turns, or stretches, accordingly — plus the corrected lead angle |
| Bearing chased over the target | Inside `ENTRY_HOLD_RNG` (60 km) the entry stops chasing a bearing that swings through 180° and just holds the airstream |
| False capture and false report | The glide floor is no longer treated as a runway capture: out of height >`DIVERT_RANGE` from the field, it says so and flies a wings-level landing where it is. The closing report reads `SHIP:STATUS` and prints `DITCHED`, `DOWN AND STOPPED n km from the runway`, or `STOPPED ON THE RUNWAY` — whichever is true |

## Trimming it after the next flight

The two numbers above are calibrated from **one** entry. If the next flight still
crosses the field with energy to spare, raise `DEORBIT_LEAD`; if it arrives short
and finishes on the jets, lower it. The entry prints its energy ratio every five
seconds — `energy 1.00` at the interface means the lead angle is right, and a
ratio that stays above `ENERGY_LONG` all the way down means the burn is still
happening too late.
