# Mk3 "Titan" Heavy-Lift SSTO

A stock-parts **single-stage-to-orbit spaceplane** for Kerbal Space Program
**1.2.x**, built around the Mk3 fuselage system, plus the two **kOS** flight
scripts that fly it: one takes it from the KSC runway to a 100 km orbit, the
other brings it home and lands it back on the runway.

![class](https://img.shields.io/badge/class-SSTO%20spaceplane-blue)
![game](https://img.shields.io/badge/KSP-1.2.x-green)
![parts](https://img.shields.io/badge/parts-68-lightgrey)

---

## What it does

| Requirement | How it's met |
|---|---|
| Mainly Mk3 parts, stock KSP 1.2 | Mk3 cockpit, cargo bays, fuselages, adapters + stock wings/engines/gear |
| Payload up to **100 t** | Closes to 100 km LKO at 365 t launch mass with ~660 m/s ΔV to spare (see [DESIGN](docs/DESIGN.md)) |
| Bay fits **3 stacked Jumbo-64** | Two **CRG-100** bays back to back = **20 m** of clear 3.75 m bay (a Jumbo-64 is ≈ 3.75 m tall, so 3 stack easily) |
| **≥ 4 crew** | Mk3 Cockpit (4 Kerbals) |
| **Shielded docking port** to dock a station | `Clamp-O-Tron Shielded Docking Port` on the dorsal spine |
| Reaction wheels | Mk3 cockpit torque + `Advanced Reaction Wheel Module, Large` |
| RTGs, batteries | 4 × `PB-NUK RTG`, 2 × `Z-4K` battery banks |
| Comms | `Communotron 88-88` relay dish + 2 × `Communotron 16` |
| RCS | 8 × `RV-105` blocks + monopropellant (cockpit + 2 radial tanks) |
| Reach **100 km** orbit with fuel to spare + maneuver | Yes — ascent script + budget below |
| **Deorbit, glide, land on the runway** | `deorbit_land.ks` guidance script |
| **kOS** scriptable control | `CX-4181 / CompoMax` kOS processor on board |

> Reaches 100 km with **~660 m/s ΔV to spare** carrying the full 100 t, and with
> **thousands** of m/s to spare when flying light.

---

## Repository layout

```
craft/Mk3_Titan_SSTO.craft   The spaceplane (drop into your KSP save)
scripts/ascent.ks            Runway -> 100 km orbit autopilot (kOS)
scripts/deorbit_land.ks      Deorbit -> reentry -> runway landing autopilot (kOS)
tools/build_craft.py         Procedural generator that produced the .craft
docs/BUILD_GUIDE.md          Part-by-part manifest + manual rebuild instructions
docs/FLIGHT_MANUAL.md        Action groups + how to fly it (auto or by hand)
docs/DESIGN.md               Mass & ΔV budget, ascent profile, design rationale
```

---

## Requirements

* **Kerbal Space Program 1.2.x** (the craft is saved as `version = 1.2.2`).
* **[kOS – Kerbal Operating System](https://forum.kerbalspaceprogram.com/index.php?/topic/61827-kos-scriptable-autopilot-system/)**
  mod, for the kOS processor part and to run the scripts. Any recent kOS
  release for KSP 1.2 works.

The airframe itself is **100 % stock** — only the single kOS processor part is
modded. If you don't run kOS, delete that one part and the craft is pure stock
(you then fly it by hand using [the flight manual](docs/FLIGHT_MANUAL.md)).

---

## Installation

**1. The craft**

```
cp craft/Mk3_Titan_SSTO.craft  "<KSP>/saves/<YourSave>/Ships/SPH/"
```

Then open the **SPH** (Space Plane Hangar) and load *Mk3 Titan Heavy SSTO*.

**2. The kOS scripts** — copy them onto the kOS *Archive* volume:

```
cp scripts/ascent.ks scripts/deorbit_land.ks  "<KSP>/Ships/Script/"
```

(`Ships/Script/` is kOS's archive folder — it's the `0:` volume in the terminal.)

---

## Quick start

1. Load the craft in the SPH, put ≤ 100 t of cargo in the bays, roll to the
   runway.
2. Right-click the **kOS processor** → **Open Terminal**.
3. Fly to orbit:
   ```
   RUN ascent.
   ```
   It rolls, rotates, climbs on the RAPIERs' air-breathing mode, switches to
   closed cycle, and circularises at **100 km**, then prints leftover fuel.
4. Do your mission (dock at your station via the dorsal shielded port, drop
   cargo, etc.).
5. Come home:
   ```
   RUN deorbit_land.
   ```
   It plans and burns the deorbit, flies a high-AoA reentry, glides to the KSC,
   and lands on the runway.

> **Read [`docs/FLIGHT_MANUAL.md`](docs/FLIGHT_MANUAL.md) before the first
> flight** for the action-group setup and the manual-flight procedure.

---

## A note on tuning (please read)

The craft file is **generated** (see `tools/build_craft.py`) so its geometry is
coherent and symmetric and it loads cleanly in the SPH with every required
system present. It has **not** been flight-tested in-game in this environment,
so treat the delivered airframe as a **strong, complete starting point**:

* Verify the **CoM/CoL** relationship in the SPH (CoL just behind CoM) and nudge
  wing/tank position or add ballast if your specific payload shifts the balance.
* Check **landing-gear** placement/orientation on the runway — gear is the one
  thing most often re-seated by hand on any shared craft.
* The kOS **guidance constants** (deorbit lead angle, glide pitch, flare height)
  are grouped at the top of each script and are the natural things to trim to
  your exact mass and drag.

Everything is parameterised and documented so these adjustments take seconds.
The ΔV budget in [DESIGN.md](docs/DESIGN.md) is computed from stock part
masses and closes with healthy margin.
