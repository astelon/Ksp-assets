# Build guide — Mk3 "Titan" Heavy-Lift SSTO

The ready-made craft is in [`craft/Mk3_Titan_SSTO.craft`](../craft/Mk3_Titan_SSTO.craft)
— just drop it in your save. This guide is here so you can **rebuild or repair it
by hand** in the SPH, and so you know exactly what every part is.

## Part manifest

| Qty | Part (in-game name) | Internal name |
|----:|---------------------|---------------|
| 1 | Mk3 Cockpit | `mk3Cockpit.Shuttle` |
| 2 | Mk3 Cargo Bay CRG-100 | `mk3CargoBayL` |
| 3 | Mk3 Rocket Fuel Fuselage (Long) | `mk3FuselageLFO.100` |
| 1 | Mk3 Liquid Fuel Fuselage (Long) | `mk3FuselageLF.100` |
| 1 | Mk3 to 2.5m Adapter | `adapterMk3-Size2` |
| 1 | Rockomax Brand Adapter (2.5m→4×1.25m) | `adapterLargeSmallQuad` |
| 14 | CR-7 R.A.P.I.E.R. Engine | `RAPIER` |
| 8 | Shock Cone Intake | `shockConeIntake` |
| 2 | Big-S Delta Wing | `wingShuttleDelta` |
| 2 | Big-S Wing Strake | `wingShuttleStrake` |
| 4 | Big-S Elevon 2 | `wingShuttleElevon2` |
| 2 | Big-S Spaceplane Tail Fin | `wingShuttleRudder` |
| 1 | Advanced Reaction Wheel Module, Large | `asasmodule1-2` |
| 3 | LY-60 Landing Gear | `GearLarge` |
| 1 | Clamp-O-Tron Shielded Docking Port | `dockingPortShielded` |
| 1 | kOS Radial Processor (mod) | `kOSMachineRad` |
| 4 | PB-NUK Radioisotope Generator | `rtg` |
| 2 | Z-4K Rechargeable Battery Bank | `batteryBankLarge` |
| 1 | Communotron 88-88 | `HighGainAntenna` |
| 2 | Communotron 16 | `longAntenna` |
| 8 | RV-105 RCS Thruster Block | `RCSBlock` |
| 2 | FL-R25 RCS Fuel Tank | `radialRCSTank` |
| 2 | Illuminator Mk2 | `spotLight2` |

**Total: 68 parts** — stock KSP 1.2 except the one **kOS** processor.

> The kOS part name can differ slightly between kOS releases (`kOSMachineRad`,
> `kOSMachine1m`, `kOSMachine0m`). If the craft complains about a missing part
> on load, it's this one — place whichever kOS processor your install has, on
> the spine behind the cockpit.

## Assembly order (manual rebuild)

Build the **central spine first**, nose → tail, all in line:

1. **Mk3 Cockpit** (root).
2. **CRG-100** cargo bay ×2 (this is your 20 m payload bay — fits 3 stacked
   Jumbo-64).
3. **Mk3 Rocket Fuel Fuselage (Long)** ×3.
4. **Mk3 Liquid Fuel Fuselage (Long)** ×1 (air-breathing fuel reserve).
5. **Mk3 → 2.5 m Adapter**.
6. **Advanced Reaction Wheel Module, Large** (2.5 m).
7. **Rockomax quad adapter** (2.5 m → 4 × 1.25 m).

Then the **engines** (turn on mirror symmetry, ×2, and radial symmetry where
useful):

8. **4 × RAPIER** on the four quad-adapter nodes (rear cluster).
9. **10 × RAPIER** surface-mounted on the flanks of the three LFO tanks
   (2 per side on the outer two tanks, 1 per side on the middle tank).
10. **8 × Shock Cone Intake** on the forward fuselage, facing forward — feed the
    jets.

**Wings & tails** (mirror symmetry):

11. **2 × Big-S Delta Wing**, low on the mid-fuselage sides.
12. **2 × Big-S Wing Strake** forward of the deltas as leading-edge blends.
13. **4 × Big-S Elevon 2** on the wing trailing edges (pitch + roll).
14. **2 × Big-S Tail Fin** as vertical stabilisers at the rear top corners.

**Landing gear**:

15. **1 × LY-60** nose gear on the belly under the front bay; **2 × LY-60**
    mains under the mid-fuselage, straddling the CoM.

**Systems** (mostly on the dorsal spine):

16. **Clamp-O-Tron Shielded Docking Port** on top of the front bay, facing up.
17. **kOS processor** on the spine behind the cockpit.
18. **4 × PB-NUK RTG** and **2 × Z-4K** batteries on the dorsal flanks.
19. **Communotron 88-88** dish + **2 × Communotron 16** whips.
20. **8 × RV-105 RCS** blocks (4 fore, 4 aft, both sides) + **2 × FL-R25**
    monopropellant tanks.
21. **2 × Illuminator Mk2** landing lights on the nose belly.

## Balance check before flight

* In the SPH, toggle the **CoM** and **CoL** markers. With the bays **empty**,
  CoL should sit **just behind** CoM. Load your payload and confirm CoM stays
  ahead of CoL — if a heavy payload pushes CoM back, shift it forward in the bay
  or add a little nose ballast.
* Make sure all 14 RAPIERs, the reaction wheel, and RCS are **not** blocked by
  the cargo bay doors, and that the **fuel crossfeed** runs the length of the
  spine (it does through stacked Mk3 parts).
* Set the **action groups** listed in [FLIGHT_MANUAL.md](FLIGHT_MANUAL.md).
