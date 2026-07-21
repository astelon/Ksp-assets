# Design Review — Mk3 "Titan" Heavy SSTO (`craft/Mk3_Titan_SSTO.craft`)

**Verdict: the delivered craft is broken and should not be presented as flight‑ready.**
The in‑SPH screenshots are correct — parts are scattered, the fuselage has gaps,
control surfaces and engines sit at wrong positions/angles, and the vessel loads
with **"Warning: No Control."** This document explains *why*, tied to specific
lines in `tools/build_craft.py`, and what a trustworthy fix actually requires.

This review supersedes the reassuring "strong, complete starting point" language
in the README and `DESIGN.md`. That claim was not earned: the file was never
opened in the SPH before shipping.

---

## The one root cause behind everything

A KSP `.craft` stores, for every part, an **absolute position (`pos`) and
rotation (`rot`) relative to the root part.** When the editor loads a craft it
**places each part exactly at those stored values.** It does *not* re‑snap parts
onto their neighbours' attachment nodes, and it does *not* recompute geometry
from the tree. The attachment graph (`link` / `attN` / `srfN`) only records
parent/child relationships; it does not move anything.

Consequence: **every coordinate and quaternion in the file has to be correct on
its own.** `tools/build_craft.py` computes all of them by hand from assumed part
dimensions and assumed local axes — none of it verified against the real stock
part `.cfg` files or against the editor. When those assumptions are wrong (they
are, in several places), the parts render precisely where the bad numbers put
them. That is the "scattered" look. It is not a rendering glitch; the file is
literally describing a scattered vessel.

---

## Defect 1 — Fuselage gaps/overlaps (wrong stack lengths)

`STACK_PLAN` (build_craft.py:87‑98) assigns each fuselage segment a **guessed
half‑length**, and each part is placed adjacent to the previous one using those
guesses (build_craft.py:103‑113):

```
("mk3Cockpit.Shuttle", 1.875)   # modelled as 3.75 m long
("mk3CargoBayL",       5.00)    # modelled as 10 m long
("mk3FuselageLFO.100", 5.00)    # modelled as 10 m long
...
```

These half‑lengths do **not** match the real stock Mk3 part models. The stack
ends up spanning **z = 0 → −66.3 m** (see `size = 21.75,7.75,71.615` in the
craft header) — roughly **twice** the length of a real Mk3 airframe of this part
count. Because KSP draws each part's real model at the stored `pos`, wherever the
assumed length ≠ the real length you get a **visible seam**: overlap where the
guess was too short, a gap where it was too long. That is the banding/gapping
visible along the fuselage in the screenshots, and why the tail sections land far
aft of where the visible hull ends.

**Correct value source:** `node_stack_top` / `node_stack_bottom` in each part's
`part.cfg`. The spacing between two stacked parts must equal
`|bottom‑node offset of the front part| + |top‑node offset of the rear part|`,
not a hand‑picked half‑length.

## Defect 2 — Scattered wings, tails, engines, intakes, gear (wrong surface‑attach placement)

Everything hung on the fuselage flanks is surface‑attached with a **hand‑guessed
radial offset and a hand‑guessed quaternion**:

- Wings/strakes/elevons (build_craft.py:156‑169)
- Vertical tails (build_craft.py:171‑175)
- 10 flank RAPIERs (build_craft.py:134‑141)
- 8 shock‑cone intakes (build_craft.py:143‑151)
- Landing gear (build_craft.py:177‑184)
- RTGs, batteries, antennas, RCS, lights (build_craft.py:186‑234)

Two things are wrong here:

1. **Orientation.** `FACE_RIGHT`/`FACE_LEFT`/`GEAR`/`STACK_REAR` (build_craft.py:38‑42)
   assume each part's model long‑axis and surface‑attach normal in a way that
   isn't checked against the real parts. A shuttle delta wing, a rudder, a RAPIER
   and a landing‑gear leg each have *different* local axes; applying one guessed
   quaternion family to all of them tips them to wrong angles. This is why the
   tails appear mid‑fuselage instead of at the rear, the wings splay outward, and
   the gear/engines dangle at odd angles in the screenshots.

2. **Position.** The radial offset uses a flat `FUSE_R = 1.875` (the fuselage
   *centreline* radius) for parts whose attach point is not on that cylinder, so
   parts float off the skin or sink into it. And because the *z* anchors are taken
   from the Defect‑1 stack (whose segment centres are in the wrong places), even a
   correctly‑oriented part would be attached to the wrong station along the hull.

There is no realistic way to get ~50 surface‑attach quaternions right by
inspection. In practice these are placed by the editor's own snapping.

## Defect 3 — "Warning: No Control" at launch

The only control sources on the craft are:

- the **manned** `mk3Cockpit.Shuttle` — provides control **only when crewed**, and
- the **`kOSMachineRad`** processor — a *modded* part that only provides control if
  the kOS mod is installed **and** the internal part name is correct.

There is **no stock probe core** as a fallback. So if the craft is launched with
an empty crew manifest (common for a freshly imported craft) and kOS is absent or
the name doesn't resolve, KSP has zero valid command modules → the exact dialog in
the screenshot: *"This vessel has no remote‑controlled or manned command modules.
It won't be controllable."*

Two things to fix regardless of the geometry work:
- Add a **stock probe core** (e.g. an OKTO2/HECS) so the airframe is controllable
  unmanned and doesn't depend on a mod being present.
- Confirm the kOS radial part's exact internal `name` against the installed kOS
  version before relying on it (do **not** assume `kOSMachineRad`).

## Defect 4 — Root part carries a 90° rotation

The root cockpit is emitted with `rot = 0.70710678,0,0,0.70710678` (a 90° pitch,
from applying `STACK` to every part including the root, build_craft.py:108). A
normally‑placed Mk3 cockpit root is identity (`0,0,0,1`). Rotating the root is
another unverified assumption about the Mk3 parts' native axis; it interacts with
Defects 1–2 so that "up," "forward," and the belly/dorsal offsets used for gear,
docking port, RTGs, etc. are all measured against an axis that may not be the real
one. This needs to be pinned down from the cockpit `.cfg`, not guessed.

---

## Why I'm not shipping a "fixed" regeneration in this pass

I can see the bugs, but I **cannot verify a corrected craft** — there is no KSP
install in this environment to open the SPH and confirm parts snap where intended.
Editing the coordinates blind would just produce a *differently* wrong file that I
could not honestly call fixed. Shipping an unverified craft asserted to be good is
the precise mistake that produced this situation; repeating it would be worse than
saying so plainly.

## The reliable fix path

**Option A (recommended): build it in the editor, then export.**
Use `docs/BUILD_GUIDE.md` as the *parts manifest only* and assemble the craft in
the SPH so KSP's own node‑snapping places everything. Save, and the exported
`.craft` is correct by construction. Then re‑derive the CoM/CoL and ΔV numbers in
`DESIGN.md` from the real, in‑editor mass — the current budget is computed from
assumed masses and hasn't been checked against a loadable vessel either.

**Option B: fix the generator, but verify each stage in‑game.**
If the procedural generator is worth keeping, it has to be corrected against real
data and checked in the editor incrementally:
1. Replace every `STACK_PLAN` half‑length with the real `node_stack_*` offsets
   from each part's `.cfg`; load the bare fuselage stack and confirm no seams.
2. Pin the true local axes of the cockpit and each surface part; fix the
   quaternion constants; add parts one family at a time (engines, then wings, …),
   loading after each to confirm placement.
3. Add a stock probe core; confirm control on the pad.
4. Only then re‑enable the dorsal/gear/RCS clusters.

Either way the deliverable isn't trustworthy until it has been **opened in the SPH
and launched at least once.**

---

## Screenshot → defect map

| What you see | Cause |
|---|---|
| Long fuselage with dark seams/bands between segments | Defect 1 (wrong stack lengths) |
| Tail fins sitting mid‑body; wings splayed outward | Defects 2 + 1 (wrong surface rot + wrong z anchors) |
| RAPIER nacelles and landing gear dangling below/around the hull | Defect 2 (wrong surface rot/offset) |
| Whole vessel looks tipped/askew | Defect 4 (root rotation) compounding the rest |
| "Warning: No Control" dialog | Defect 3 (no stock probe core; empty crew / kOS not resolved) |
