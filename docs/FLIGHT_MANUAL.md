# Flight manual — Mk3 "Titan" Heavy-Lift SSTO

How to fly the ship, both with the kOS autopilot and by hand.

## Recommended action groups

Set these in the SPH (they make manual flight and script backup easy):

| Group | Action | Why |
|---|---|---|
| **1** | Toggle RAPIER **mode** (air-breathing ⇄ closed cycle) | manual backup to the ascent script's auto-switch |
| **2** | Toggle **cargo bay** doors (both CRG-100s) | load / deploy payload |
| **3** | Toggle **shielded docking port** shield | open before docking, close before reentry |
| **4** | Toggle **antennas** (extend/retract) | deploy in orbit |
| **Gear (G)** | Landing gear | — |
| **Brakes (B)** | Wheel brakes + airbrakes | reentry drag & rollout |
| **Lights (U)** | Illuminators | night ops |
| **RCS (R)** | RCS | docking |
| **SAS (T)** | SAS | manual flight |

The kOS scripts drive throttle, steering, gear, brakes, lights, RCS, and the
RAPIER mode **directly**, so they work even if you forget the action groups —
but Group 1 is a handy manual override.

---

## Auto flight (kOS)

### To orbit

```
RUN ascent.
```

Sequence: map the fuel network and isolate the payload → measure both engine
modes and run the feasibility check → roll → rotate at 80 m/s →
accelerate-first air-breathing climb
→ switch to closed cycle when the jets actually run out of breath → rocket push
to the requested apoapsis → coast (time-warped) → circularise → report. Hands off
the whole way.

### The pre-flight feasibility check

Before the brakes come off, the script answers the question the autopilot can't
answer once it's airborne: **is there enough ΔV aboard, and if not, how much
more of what?** It ignites at zero throttle, flips the RAPIERs to closed cycle
just long enough to read their real thrust and Isp, flips back, and prices the
mission.

The model is explicit about the two things that make a spaceplane different
from a rocket:

* **The jets drink the rocket's liquid fuel.** The ΔV figure you'd read on the
  pad counts every paired LF/Ox unit in the tanks, which badly overstates what
  survives to the mode switch. The check burns the jet phase *first* (rocket
  equation at the measured air-breathing Isp over `PLAN_JET_DV`), then prices
  the rocket phase against what's left. Note this can legitimately come out
  *higher* than the pad figure — burning an LF-only reserve lightens the ship
  without touching a single paired unit, which is what that reserve is for.
* **Adding propellant is not free.** Tanks have structure, so x tonnes of fuel
  costs x(1+k) on the runway, where k is measured from *this ship's own* tanks
  (wet wings are not the same deal as a plain Mk3 fuselage). That's why the
  answer to "how much more fuel?" isn't linear — and why there's a hard
  ceiling: as tanks are added the mass ratio tends to (1+k)/k, and a design
  needing more than that ceiling **cannot be fixed with fuel at all**.

If the ship comes up short it says by how much, then ranks the fixes:

1. **Liquid fuel first**, if the jets are eating paired LF. This is the most
   common way a sound SSTO comes up short, and it's much the cheapest fix — LF
   burns at Isp ~3200 in the jets, while oxidizer is just cargo. The check
   quotes the LF-only top-up and what it buys.
2. **Balanced LF/Ox**, converged against the TWR it costs: if the extra
   propellant drops closed-cycle TWR below `PLAN_TWR_MIN`, it adds RAPIERs and
   re-solves the fuel to haul them, iterating until both close.
3. **Less payload**, or a lower orbit.

It also flags **oxidizer that can never be paired** — pure dead mass carried to
orbit and back.

The check assumes a *healthy* handover (`PLAN_SWITCH_ALT` 20 km,
`PLAN_SWITCH_SPD` 1450 m/s, `PLAN_JET_DV` 3000 m/s). If your airframe hands over
somewhere else the verdict will be off, so the script prints the **actual**
handover altitude, speed and jet ΔV at the real mode switch — trim the three
assumptions to those numbers and the next check is about your ship rather than a
generic one. A failed check does not stop the launch; it warns, pauses
`PREFLIGHT_HOLD` seconds so you can read it, and flies anyway, because the
in-flight budget will settle for whatever it can reach.

### In flight

The script is not tuned to one airframe — it will fly any RAPIER spaceplane that
has the performance to reach orbit at all. Four things it decides for itself:

* **Whose fuel is whose.** It walks the part tree outward from the engines and
  refuses to cross any decoupler, separator or docking port. Only tanks inside
  that feed network count as ΔV, and crossfeed is switched off on every
  separator, so a **fuelled payload can neither inflate the budget nor be
  siphoned during the climb**. If a separator exposes no crossfeed toggle, the
  script falls back to disabling flow on the payload's tanks and tells you to
  re-enable them after you release the payload.
* **The flight profile — accelerate first.** Both powered phases obey one rule:
  climb only as steeply as the ship can climb *while still gaining speed*. The
  air-breathing phase commands a vertical speed, raising it while acceleration
  is healthy and lowering it the moment acceleration sags; the rocket phase
  commands a flight-path angle biased the same way. A single controller turns
  that command into pitch — feed-forward γ plus a slow, clamped trim that learns
  this airframe's angle of attack — so the nose moves smoothly instead of
  hunting. Time-to-apoapsis is deliberately *not* a control variable: it is
  discontinuous exactly where the rocket phase starts.
* **When the jets are done.** Air-breathing Isp is roughly ten times the
  rocket's, so the switch is deliberately reluctant: flameout, thrust decayed
  off its own measured peak, airspeed falling away from its peak, or no
  acceleration left *after* the profile has already flattened out. One low
  acceleration sample while the nose is up is not evidence, and switching early
  throws away hundreds of m/s of nearly-free speed.
* **What orbit it can actually afford.** It flies for `REQUESTED_APOAPSIS`
  (100 km by default) but *measures* what the climb really costs — how much of
  the bill each m/s of spent ΔV retires — and re-prices the orbit from that
  instead of trusting an impulsive estimate. If the target is priced out it
  settles for the best orbit it can afford and says so.

**It never burns dry.** Two hard floors police every burn. Once an orbit is
genuinely in reach the ascent stops at *circularise + deorbit + margin*, and
below that a glide reserve (`DV_GLIDE_RESERVE`, 120 m/s) is kept back no matter
what. The circularisation burn stops dead on the deorbit reserve rather than
spending it — you may end up slightly elliptical, but always able to come home.

If the ship is so short of ΔV that the apoapsis never clears the atmosphere, the
script says so, skips circularisation, and hands back a glider with fuel still in
the tanks — no deorbit burn is needed from there.

### Home again

```
RUN deorbit_land.
```

Sequence: orient retrograde → predict the deorbit point (lead angle before the
KSC) and plant a maneuver node there → time-warp to the burn → burn periapsis
down to ~32 km → high-AoA reentry → energy-managed glide homing on the runway →
capture heading 090 → glideslope → flare → gear down → touchdown → brake to a
stop.

The deorbit point is *solved*, not waited out: the script scans the next two
orbits with `POSITIONAT`, correcting for Kerbin's own rotation, bisects the
moment the ground track is `DEORBIT_LEAD` short of the KSC, prices the burn off
the orbit it will be on at that instant, and warps there. It gets pointed
retrograde *before* warping, because on rails the ship cannot rotate — and
re-settles on the node burn vector after dropping out, since inertial attitude
drifts away from retrograde during the coast. Warp ends `WARP_LEAD` seconds
early to leave room for that. If no crossing is found (a wildly inclined or
non-circular orbit), it says so and falls back to the old real-time wait.

Both scripts expose their tunables at the top — trim `REQUESTED_APOAPSIS`,
`ROTATE_SPEED`, `DV_MARGIN`, `DV_GLIDE_RESERVE`, `DEORBIT_LEAD`, `WARP_LEAD`,
`GLIDE_SPEED`, `FLARE_ALT`, etc. to taste. `DEORBIT_PE` appears in *both* scripts
and should match: the ascent script reserves the ΔV that the deorbit script will
spend.

The ascent tunables worth knowing if a flight goes wrong:

| Tunable | Default | What it does |
|---|---|---|
| `AB_ACC_LOW` / `AB_ACC_HIGH` | 1.2 / 3.0 m/s² | The acceleration deadband the air-breathing climb rate is traded against. Raise `AB_ACC_LOW` to fly flatter and reach the mode switch faster. |
| `SW_ARM_SPEED` | 1150 m/s | No thrust/acceleration sensing below this, so the jets are never given up early. |
| `SW_SPEED_HARD` | 1600 m/s | Hard backstop on the mode switch. |
| `CC_FPA_HI` / `CC_FPA_LO` | 15° / 0° | Rocket-phase flight-path schedule, tapering as apoapsis approaches target. |
| `CC_ACC_LOW` | 1.5 m/s² | Below this the rocket phase shallows out rather than fighting gravity. |
| `DV_GLIDE_RESERVE` | 120 m/s | The floor the ascent will never burn through. |
| `PLAN_SWITCH_ALT` / `PLAN_SWITCH_SPD` | 20 km / 1450 m/s | Handover state the pre-flight check assumes. Trim to what your ship actually achieves — the script prints it at the real switch. |
| `PLAN_JET_DV` | 3000 m/s | Jet-phase ΔV equivalent, incl. drag. Sets how much LF the check expects the jets to burn. |
| `PLAN_LOSS_FACTOR` | 1.50 | Gravity/drag/steering losses assumed on the rocket climb, pre-flight only. |
| `PLAN_TWR_MIN` | 1.05 | Closed-cycle TWR the sizing advice tries to hold at handover. |
| `PREFLIGHT_HOLD` | 12 s | Pause on a failed check so you can read it. 0 to skip. |
| `ABORT_IF_INFEASIBLE` | `FALSE` | `TRUE` cuts the burn the moment orbit is priced out of reach, keeping the most fuel for a return. |

### Before you edit a script

```
python3 tools/check_kos.py scripts/*.ks
```

kOS compiles a script only when you `RUN` it — mid-flight — and it stops at the
**first** error, so a typo costs a reload and a rollout, and you find the next
one the same way. The checker catches what can be seen from the text: unbalanced
braces, calls to undefined functions or with the wrong argument count, and
**built-in shadowing**.

That last one has bitten this repo twice. kOS refuses to compile a script that
declares a variable hiding one of its built-in functions, and it isn't only the
one-letter ones — `QUEUE`, `STACK`, `LIST`, `RANGE`, `NODE`, `PATH`, `BODY` and
a long tail of ordinary words are built-ins too, so `LOCAL queue IS LIST().`
will not compile. Assigning a *bound* variable is fine (`SET WARP TO 0.` is the
documented way out of time warp); declaring one over it is not.

---

## Manual flight (no kOS)

### Ascent

1. **SAS on**, brakes off, throttle 100 %, RAPIERs in **air-breathing** mode.
2. Roll to ~80 m/s, gently pull to **~10–12° pitch**, lift off, gear up.
3. Hold a shallow climb and **build speed**. Around 400–500 m/s you'll punch
   through transonic — keep the nose low until you're supersonic, then resume
   climbing.
4. Ride the jets up. When thrust starts to sag (~20 km, ~1400 m/s), **switch
   RAPIERs to closed cycle** (Action Group 1) and pitch up to ~20°.
5. Fly a normal gravity turn, keeping apoapsis climbing toward **100 km**. Cut
   throttle when apoapsis = 100 km.
6. Coast to apoapsis, then **burn prograde** to raise periapsis to ~100 km.

### Reentry & landing

1. In orbit, **close the docking-port shield** (AG 3), **retract antennas**
   (AG 4), stow any cargo.
2. Over the far side of Kerbin from the KSC, burn **retrograde** to drop
   periapsis to ~30–35 km.
3. Reentry: hold **~40° angle of attack** (nose well above retrograde), belly
   into the airstream, brakes on for extra drag. Keep it pointed generally
   toward the KSC.
4. Below ~30 km and subsonic-ish, fly it like a glider: aim for the runway,
   **best glide ~150–160 m/s**. If you're high/fast, use airbrakes and gentle
   S-turns to burn energy.
5. Line up with **runway 09** (heading 090), fly a shallow (~5°) glideslope,
   **gear down**, flare at ~20–30 m, touch down, **brakes on**, keep straight,
   roll to a stop.

---

## Payload & docking notes

* **Cargo**: up to **100 t** in the two CRG-100 bays. Keep the payload CoM near
  the ship's CoM (roughly over the main gear). Secure long stacks so they don't
  shift.
* **Docking**: the shielded port is on the **top** of the ship, behind the
  cockpit. Open the shield (AG 3), enable **RCS (R)**, approach your station's
  port from below/alongside, and dock. The 8 RV-105 blocks + monopropellant give
  full translation control.
* **Power**: the 4 RTGs supply power indefinitely; the Z-4K banks cover peaks.
  You can leave the ship parked without worrying about batteries.
