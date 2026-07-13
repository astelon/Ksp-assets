# Design notes — Mk3 "Titan" Heavy-Lift SSTO

This document explains *why* the ship is built the way it is, and shows the mass
and ΔV budget that says it closes to a 100 km orbit with the full 100 t payload.

## 1. Requirements → design decisions

| Requirement | Decision |
|---|---|
| 100 t payload, 3 stacked Jumbo-64 | Two **CRG-100** cargo bays back-to-back → 20 m of clear 3.75 m bay. A Rockomax Jumbo-64 is ~3.75 m tall, so three stack with room to strap them down. |
| SSTO to 100 km with margin | **14 × CR-7 R.A.P.I.E.R.** — air-breathing to ~1.4–1.6 km/s so most of orbital velocity is bought with atmospheric oxygen, then closed-cycle for the vacuum push. |
| ≥ 4 crew | **Mk3 Cockpit** (4). |
| Dock to a station | **Clamp-O-Tron Shielded Docking Port** on the dorsal spine — the shield keeps it off the reentry airstream. |
| Power that never runs out | **4 × PB-NUK RTG** (continuous) + **2 × Z-4K** banks (surge/eclipse). |
| Comms | **Communotron 88-88** (relay) + 2 × **Communotron 16**. |
| Attitude control | Cockpit torque + **Advanced Reaction Wheel Module, Large**, plus 8 × **RV-105 RCS** for docking. |
| Scripted control | **kOS processor** (radial). |

## 2. Layout

A single Mk3 spine, nose to tail:

```
[Mk3 Cockpit] [CRG-100] [CRG-100] [LFO-Long] [LFO-Long] [LFO-Long] [LF-Long] [Mk3→2.5m] [Rx-Wheel] [Quad] ==> 4 rear RAPIERs
      4 crew    cargo      cargo     rocket      rocket      rocket    air-LF    adapter    torque   mount
```

* **Wings**: 2 × Big-S Delta + 2 × Big-S Strake, low-mounted, with 4 × Big-S
  Elevon 2 for pitch/roll. Two Big-S rudders as vertical tails.
* **Engines**: 4 on the rear quad-adapter cluster + 10 surface-mounted on the
  LFO-tank flanks = **14 RAPIERs**, fed by **8 shock-cone intakes**.
* **Gear**: nose wheel + 2 mains (LY-60 class).
* The **LF-only long tank** exists so the air-breathing phase has a dedicated
  liquid-fuel reserve and doesn't strand oxidizer that the closed-cycle phase
  needs.

## 3. Mass budget (stock 1.2 part masses)

| Group | Mass |
|---|---|
| Structure / dry (no fuel, no payload) | **≈ 82 t** |
| Onboard propellant (LF 18 625 u + Ox 17 875 u + mono) | **≈ 184 t** |
| Vessel wet, no payload | **≈ 265 t** |
| Payload | **100 t** |
| **Launch mass (wet + 100 t payload)** | **≈ 365 t** |

## 4. ΔV budget to 100 km LKO

Kerbin low orbit needs ~3400 m/s of *total* ΔV from the pad, but an
air-breathing SSTO buys most of that with intake air:

| Phase | Provides / needs |
|---|---|
| Air-breathing climb (RAPIER jet mode) | Reaches **~1.4–1.6 km/s** at ~20 km on liquid fuel only — effectively "free" velocity (jet Isp ~3200 s). |
| Rocket ΔV still needed to LKO | ≈ (2287 − 1400) orbital + ~350 gravity/drag losses ≈ **1240 m/s**. |
| Closed-cycle ΔV **available** (Isp 305 s, from the mode-switch mass) | **≈ 1900 m/s**. |
| **Margin at 100 t payload** | **≈ 660 m/s** left for circularise + deorbit + docking. |

Rocket ΔV is the rocket equation on the closed-cycle segment:

```
ve   = 305 s × 9.80665 = 2991 m/s
dV   = ve × ln(m_switch / m_MECO)
     = 2991 × ln(361 / 199)  ≈ 1900 m/s
```

**Thrust-to-weight**

* Take-off is on air-breathing mode, where the 14 RAPIERs make far more than
  their vacuum thrust at low altitude (intake air + jet multiplier), so the
  runway acceleration is strong.
* Closed-cycle vacuum thrust = 14 × 180 kN = **2520 kN**. At the 365 t launch
  mass that's a vacuum-referenced TWR of ~0.70 — normal for a heavy spaceplane
  that lights its rockets already moving at 1.5 km/s and horizontal — rising to
  **~1.4** by MECO as the tanks empty.

**Flying light**: with no payload the closed-cycle ΔV is **> 3200 m/s**, so the
ship has enormous margin for lighter cargo, higher orbits, or rendezvous.

## 5. Reentry & landing rationale

* **Deorbit**: retrograde burn to ~32 km periapsis, started a lead-angle
  (~118° of ground track) before the KSC so the descent track ends over the
  space centre.
* **Reentry**: high angle-of-attack (~40°) presents the belly and the Big-S
  wings as a big draggy brake, bleeding orbital energy without overheating,
  while yawing the nose toward the runway.
* **Glide**: a spaceplane with this wing area has a wide glide footprint, so the
  guidance script actively homes on the runway bearing and manages energy
  (airbrakes/S-turns if high) — this absorbs deorbit-timing error.
* **Landing**: capture runway heading 090, ~5° glideslope, flare at ~28 m
  radar altitude, gear down, touchdown, wheel-braking to a stop on centreline.

## 6. Regenerating the craft

The `.craft` is emitted by `tools/build_craft.py`. Edit the `STACK_PLAN` list or
the engine/wing/utility sections and re-run:

```
python3 tools/build_craft.py
```

It prints a part summary and the bounding size, and rewrites
`craft/Mk3_Titan_SSTO.craft`.
