#!/usr/bin/env python3
"""
build_craft.py - Procedural generator for the Mk3 heavy-lift SSTO ".craft" file.

Why a generator instead of a hand-written .craft?
------------------------------------------------
A KSP ".craft" file is a flat list of PART blocks.  Each block records a part's
internal name, its position/orientation relative to the root part, and its place
in the attachment tree (which part it hangs off, via a stack node `attN` or a
surface attach `srfN`).  Part *mass*, *fuel*, and *MODULE* data are NOT stored in
the craft - the game regenerates them from the part .cfg at load time.  That means
a valid, loadable craft only needs:

  * correct stock/kOS internal part names,
  * self-consistent geometry (stack nodes that actually meet), and
  * a consistent parent/child tree (`link` on the parent, `attN`/`srfN` on the child).

Authoring ~60 parts with hand-typed 3D coordinates is hopelessly error prone, so
this script lays the ship out on a coherent grid and emits the file.  Re-run it to
regenerate craft/Mk3_Titan_SSTO.craft after any change.

Coordinate convention (matches the SPH):
    +Z = forward (nose)      -Z = aft (engines)
    +Y = up (dorsal)         -Y = down (belly)
    +X = right               -X = left
The root part (the cockpit) sits at the origin; every other part's `pos` is world
space relative to it.  Fuselage stack parts are modelled vertically in their .cfg
(long axis = local +Y) and rotated -90 deg about X so their long axis lies along Z;
that rotation is the quaternion STACK below.
"""

import math

# ---------------------------------------------------------------------------
# Quaternions are stored in KSP order: (x, y, z, w).
# ---------------------------------------------------------------------------
IDENT = (0.0, 0.0, 0.0, 1.0)                       # no rotation (dorsal parts face +Y)
STACK = (0.70710678, 0.0, 0.0, 0.70710678)         # +90 deg about X: local +Y -> world +Z
STACK_REAR = (0.70710678, 0.0, 0.0, 0.70710678)    # engines: top node forward, nozzle -Z
GEAR = (-0.70710678, 0.0, 0.0, 0.70710678)         # -90 about X: gear hangs toward -Y
FACE_RIGHT = (0.0, 0.0, -0.70710678, 0.70710678)   # local +Y -> world +X (starboard)
FACE_LEFT = (0.0, 0.0, 0.70710678, 0.70710678)     # local +Y -> world -X (port)

FUSE_R = 1.875        # Mk3 fuselage radius (3.75 m diameter)


class Part:
    _uid = 4293000000  # descending unique ids, KSP-style

    def __init__(self, name, pos, rot=IDENT, istg=-1):
        Part._uid -= 137
        self.name = name
        self.uid = "%s_%d" % (name, Part._uid)
        self.pos = pos
        self.rot = rot
        self.istg = istg
        self.parent = None          # (parent_part, child_node_id, parent_node_id|None)
        self.links = []             # child parts
        self.resources = []         # list of (resname, amount, maxamount)

    def attach_stack(self, parent, child_node="top", parent_node="bottom"):
        """Attach this part to `parent` via stack nodes."""
        self.parent = (parent, child_node, parent_node)
        parent.links.append(self)
        return self

    def attach_srf(self, parent):
        """Surface-attach this part to `parent`."""
        self.parent = (parent, "srfAttach", None)
        parent.links.append(self)
        return self


PARTS = []


def add(name, pos, rot=IDENT, istg=-1):
    p = Part(name, pos, rot, istg)
    PARTS.append(p)
    return p


# ===========================================================================
# 1. CENTRAL FUSELAGE STACK  (nose +Z  ->  tail -Z)
# ===========================================================================
# (internal name, half-length along Z in metres)
STACK_PLAN = [
    ("mk3Cockpit.Shuttle", 1.875),   # 4-crew cockpit  (ROOT)
    ("mk3CargoBayL",       5.00),    # CRG-100 cargo bay #1
    ("mk3CargoBayL",       5.00),    # CRG-100 cargo bay #2  -> 20 m of bay, fits 3 stacked Jumbo-64
    ("mk3FuselageLFO.100", 5.00),    # rocket fuel (LF+Ox) long
    ("mk3FuselageLFO.100", 5.00),    # rocket fuel (LF+Ox) long
    ("mk3FuselageLFO.100", 5.00),    # rocket fuel (LF+Ox) long
    ("mk3FuselageLF.100",  5.00),    # liquid-fuel only long  (air-breathing reserve)
    ("adapterMk3-Size2",   1.875),   # Mk3 -> 2.5 m adapter (also LFO)
    ("asasmodule1-2",      0.16),    # Advanced Reaction Wheel Module, Large (2.5 m)
    ("adapterLargeSmallQuad", 0.31), # 2.5 m -> 4x 1.25 m engine mount
]

stack_parts = []
z = 0.0
prev_half = None
for i, (name, half) in enumerate(STACK_PLAN):
    if i == 0:
        z = 0.0
    else:
        z = prev_center - (prev_half + half)
    p = add(name, (0.0, 0.0, round(z, 4)), STACK)
    if i > 0:
        p.attach_stack(stack_parts[-1], child_node="top", parent_node="bottom")
    stack_parts.append(p)
    prev_center = z
    prev_half = half

cockpit = stack_parts[0]
bay1, bay2 = stack_parts[1], stack_parts[2]
lfo1, lfo2, lfo3 = stack_parts[3], stack_parts[4], stack_parts[5]
lf1 = stack_parts[6]
tail_adapter = stack_parts[7]
reaction_wheel = stack_parts[8]
quad = stack_parts[9]
quad_z = quad.pos[2]

# ===========================================================================
# 2. ENGINES  (14x CR-7 R.A.P.I.E.R.)
# ===========================================================================
# 2a. Four on the rear quad adapter (1.25 m nodes in a 2x2, ~0.63 m offset).
rapier_half = 1.05
rz = quad_z - 0.31 - rapier_half
for k, (dx, dy) in enumerate([(0.63, 0.63), (-0.63, 0.63), (0.63, -0.63), (-0.63, -0.63)]):
    r = add("RAPIER", (dx, dy, round(rz, 4)), STACK_REAR, istg=0)
    r.attach_stack(quad, child_node="top", parent_node="bottom%02d" % (k + 1))

# 2b. Ten surface-mounted on the LFO fuselage flanks:
#     2 per side on the outer tanks (lfo1, lfo3) + 1 per side on the middle (lfo2).
for section, offsets in ((lfo1, (+2.4, -2.4)), (lfo3, (+2.4, -2.4)), (lfo2, (0.0,))):
    sz = section.pos[2]
    for side in (+1, -1):
        for dz in offsets:
            r = add("RAPIER", (side * FUSE_R, -0.4, round(sz + dz, 4)), STACK_REAR, istg=0)
            r.attach_srf(section)

# ===========================================================================
# 3. AIR INTAKES  (8x Shock Cone Intake, forward-facing)
# ===========================================================================
for side in (+1, -1):
    for dz, dy in ((0.0, 0.9), (0.0, 0.3), (0.0, -0.3), (0.0, -0.9)):
        intake = add("shockConeIntake",
                     (side * FUSE_R, dy, round(bay1.pos[2] + 4.0, 4)),
                     FACE_RIGHT if side > 0 else FACE_LEFT)
        intake.attach_srf(bay1)

# ===========================================================================
# 4. WINGS  (Big-S Delta Wing + Strake + Elevons)  and  TAILS (Big-S Rudder)
# ===========================================================================
wing_z = lfo1.pos[2] - 2.0
for side in (+1, -1):
    q = FACE_RIGHT if side > 0 else FACE_LEFT
    # main delta wing, low-mounted
    wing = add("wingShuttleDelta", (side * FUSE_R, -0.6, round(wing_z, 4)), q)
    wing.attach_srf(lfo1)
    # forward strake / leading-edge blend
    strake = add("wingShuttleStrake", (side * FUSE_R, -0.4, round(bay2.pos[2], 4)), q)
    strake.attach_srf(bay2)
    # two elevons on the wing trailing edge (roll/pitch control)
    for dz in (+1.6, -1.6):
        elev = add("wingShuttleElevon2",
                   (side * (FUSE_R + 4.5), -0.6, round(wing_z - 3.0 + dz, 4)), q)
        elev.attach_srf(wing)

# vertical tail fins (2x Big-S rudder) at the rear top corners
for side in (+1, -1):
    q = FACE_RIGHT if side > 0 else FACE_LEFT
    fin = add("wingShuttleRudder", (side * (FUSE_R - 0.3), 1.4, round(lf1.pos[2], 4)), q)
    fin.attach_srf(lf1)

# ===========================================================================
# 5. LANDING GEAR  (3x LY-60 GearLarge : 1 nose + 2 main)
# ===========================================================================
nose_gear = add("GearLarge", (0.0, -FUSE_R, round(bay1.pos[2] + 3.5, 4)), GEAR)
nose_gear.attach_srf(bay1)
for side in (+1, -1):
    mg = add("GearLarge", (side * 1.6, -FUSE_R, round(lfo1.pos[2] - 1.0, 4)), GEAR)
    mg.attach_srf(lfo1)

# ===========================================================================
# 6. DOCKING, POWER, COMMS, RCS, CONTROL
# ===========================================================================
# 6a. Shielded docking port on the dorsal spine, facing up (+Y) - station docking.
# NOTE: the stock cfg folder is "dockingPortShielded" but the part's internal
# `name` (what .craft files must reference) is "dockingPort1".
dock = add("dockingPort1", (0.0, FUSE_R, round(bay1.pos[2], 4)), IDENT)
dock.attach_srf(bay1)

# 6b. kOS scriptable control unit (radial), on the spine behind the cockpit.
kos = add("kOSMachineRad", (0.0, FUSE_R, round(bay1.pos[2] + 3.5, 4)), IDENT)
kos.attach_srf(bay1)

# 6c. Four PB-NUK RTGs (radioisotope generators) on the dorsal flanks.
for side in (+1, -1):
    for dz in (+1.2, -1.2):
        rtg = add("rtg", (side * 0.7, FUSE_R, round(lfo1.pos[2] + dz, 4)), IDENT)
        rtg.attach_srf(lfo1)

# 6d. Two Z-4K battery banks (surface), for night / eclipse reserve.
for side in (+1, -1):
    bat = add("batteryBankLarge", (side * 0.9, FUSE_R, round(bay2.pos[2], 4)), IDENT)
    bat.attach_srf(bay2)

# 6e. Comms: one Communotron 88-88 (relay dish) + two Communotron-16 whips.
dish = add("HighGainAntenna", (0.0, FUSE_R, round(bay2.pos[2] + 3.0, 4)), IDENT)
dish.attach_srf(bay2)
for side in (+1, -1):
    ant = add("longAntenna", (side * 0.6, FUSE_R, round(bay1.pos[2] - 3.5, 4)), IDENT)
    ant.attach_srf(bay1)

# 6f. RCS: 8x RV-105 blocks (fore + aft, both sides) + 2x radial monopropellant tanks.
for side in (+1, -1):
    for zc, host in ((bay1.pos[2] - 1.0, bay1), (lf1.pos[2] + 1.0, lf1)):
        rcs = add("RCSBlock", (side * FUSE_R, 0.0, round(zc, 4)),
                  FACE_RIGHT if side > 0 else FACE_LEFT)
        rcs.attach_srf(host)
        rcs2 = add("RCSBlock", (side * FUSE_R, -0.9, round(zc, 4)),
                   FACE_RIGHT if side > 0 else FACE_LEFT)
        rcs2.attach_srf(host)
for side in (+1, -1):
    mp = add("radialRCSTank", (side * FUSE_R, 0.9, round(lf1.pos[2], 4)),
             FACE_RIGHT if side > 0 else FACE_LEFT)
    mp.attach_srf(lf1)

# 6g. Two landing spotlights on the nose belly (night runway approaches).
for side in (+1, -1):
    lite = add("spotLight2", (side * 0.8, -FUSE_R, round(bay1.pos[2] + 4.2, 4)), IDENT)
    lite.attach_srf(bay1)

# ===========================================================================
# EMIT
# ===========================================================================
HOUSEKEEPING = """\tpartName = Part
\tpos = {px},{py},{pz}
\tattPos = 0,0,0
\tattPos0 = {px},{py},{pz}
\trot = {rx},{ry},{rz},{rw}
\tattRot = 0,0,0,1
\tattRot0 = {rx},{ry},{rz},{rw}
\tmir = 1,1,1
\tsymMethod = Mirror
\tautostrutMode = Off
\trigidAttachment = False
\tistg = {istg}
\tresPri = 0
\tdstg = 0
\tsidx = -1
\tsqor = -1
\tsepI = -1
\tattm = {attm}
\tmodCost = 0
\tmodMass = 0
\tmodSize = 0,0,0"""


def emit_part(p):
    px, py, pz = p.pos
    rx, ry, rz, rw = p.rot
    out = ["PART", "{", "\tpart = %s" % p.uid]
    # surface-attached parts use attm = 1
    attm = 1 if (p.parent and p.parent[1] == "srfAttach") else 0
    out.append(HOUSEKEEPING.format(px=px, py=py, pz=pz,
                                   rx=rx, ry=ry, rz=rz, rw=rw,
                                   istg=p.istg, attm=attm))
    # links to direct children
    for c in p.links:
        out.append("\tlink = %s" % c.uid)
    # parent-side stack node reference (mirrors KSP's bidirectional node record)
    for c in p.links:
        if c.parent and c.parent[1] != "srfAttach" and c.parent[2] is not None:
            out.append("\tattN = %s,%s" % (c.parent[2], c.uid))
    # this part's own attachment to its parent
    if p.parent:
        parent, child_node, _ = p.parent
        if child_node == "srfAttach":
            out.append("\tsrfN = srfAttach,%s" % parent.uid)
        else:
            out.append("\tattN = %s,%s" % (child_node, parent.uid))
    out += ["\tEVENTS", "\t{", "\t}", "\tACTIONS", "\t{", "\t}",
            "\tPARTDATA", "\t{", "\t}"]
    for (rn, amt, mx) in p.resources:
        out += ["\tRESOURCE", "\t{",
                "\t\tname = %s" % rn,
                "\t\tamount = %s" % amt,
                "\t\tmaxAmount = %s" % mx,
                "\t\tflowState = True", "\t\tisTweakable = True",
                "\t\thideFlow = False", "\t\tisVisible = True",
                "\t\tflowMode = Both", "\t}"]
    out.append("}")
    return "\n".join(out)


def main():
    xs = [p.pos[0] for p in PARTS]
    ys = [p.pos[1] for p in PARTS]
    zs = [p.pos[2] for p in PARTS]
    size = (round(max(xs) - min(xs) + 9, 3),
            round(max(ys) - min(ys) + 4, 3),
            round(max(zs) - min(zs) + 4, 3))
    header = [
        "ship = Mk3 Titan Heavy SSTO",
        "version = 1.2.2",
        "description = Mk3 heavy-lift SSTO spaceplane. 100 t cargo bay (fits 3 stacked "
        "Jumbo-64), 4-crew cockpit, shielded docking port, kOS control, RTG power. "
        "Ascent + deorbit/landing kOS scripts included. Generated by tools/build_craft.py.",
        "type = SPH",
        "size = %s,%s,%s" % size,
    ]
    body = "\n".join(emit_part(p) for p in PARTS)
    text = "\n".join(header) + "\n" + body + "\n"
    with open("craft/Mk3_Titan_SSTO.craft", "w") as fh:
        fh.write(text)
    # summary to stderr-ish stdout
    from collections import Counter
    counts = Counter(p.name for p in PARTS)
    print("Wrote craft/Mk3_Titan_SSTO.craft  (%d parts)" % len(PARTS))
    for name, n in sorted(counts.items(), key=lambda kv: -kv[1]):
        print("  %3d  %s" % (n, name))
    print("bounding size (x,y,z) = %s" % (size,))


if __name__ == "__main__":
    main()
