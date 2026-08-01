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

Sequence: measure the ship (mass, weight, TWR, ΔV) and print the budget → roll →
rotate at 80 m/s → **self-tuning** air-breathing climb → switch to closed cycle
when the jets actually run out of breath → rocket push to the requested apoapsis
→ coast → circularise → report. Hands off the whole way.

Three things the script decides for itself:

* **ΔV budget.** Before takeoff it reads the real mass, thrust, TWR and the
  closed-cycle ΔV in the tanks (rocket equation over the usable LF/Ox pair, Isp
  read off the live engines), and it prints what it is reserving for
  circularisation and deorbit.
* **Settling for a lower orbit.** It flies for `REQUESTED_APOAPSIS` (100 km by
  default), but re-prices the trip every fraction of a second. If the remaining
  ΔV can no longer buy *climb + circularise + deorbit + margin*, it says so.
  Above `SETTLE_MIN_ALT` (80 km) it stops climbing and banks the orbit it can
  afford, keeping the circularisation and deorbit ΔV intact; below 80 km it
  warns and keeps pushing, because half an orbit is worth nothing. The
  circularisation burn also stops dead on the deorbit reserve rather than
  spending it — you may end up slightly elliptical, but always able to come
  home.
* **The flight profile.** The air-breathing pitch is *searched*, not scheduled:
  every second the script measures the specific-energy rate dE/dt and nudges
  pitch in whichever direction improved it, inside a dynamic-pressure corridor.
  The mode switch fires on flameout / thrust decay / acceleration collapse /
  ceiling rather than at a fixed altitude, and the rocket phase steers to hold
  ~35 s of time-to-apoapsis. So a heavy ship and a light one fly different,
  appropriate profiles with no retuning.

If the ship is so short of ΔV that the apoapsis never clears the atmosphere, the
script says so, skips circularisation, and hands back a glider — no deorbit burn
is needed from there.

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
`ROTATE_SPEED`, `DV_MARGIN`, `SETTLE_MIN_ALT`, `TGT_ETA_AP`, `DEORBIT_LEAD`,
`WARP_LEAD`, `GLIDE_SPEED`, `FLARE_ALT`, etc. to taste. `DEORBIT_PE` appears in *both*
scripts and should match: the ascent script reserves the ΔV that the deorbit
script will spend.

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
