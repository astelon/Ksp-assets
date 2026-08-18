# Docking — how `scripts/dock.ks` joins two ports

This is the design note for the fourth flight script, and it starts exactly
where [`rendezvous.ks`](RENDEZVOUS.md) stops. That one delivers the ship to the
doorstep — a few hundred metres off the target with the relative velocity dead
and the nose on it — and deliberately goes no further, because docking needs a
chosen port at both ends and a different control law. This is that control law.

```
RUN dock.               // both ports as they are set up in the game
RUN dock("dorsal").     // name our port by part tag or title
RUN dock("", 60).       // ...and force the standoff distance, in metres
```

**Set-up, in the game, before you run it:**

1. Right-click the port you want to dock *with* → **Set as Target**. The port,
   not the vessel.
2. Right-click your own port → **Control From Here**. Forget this one and the
   script does it for you whenever it can identify the port without guessing.

Neither ship needs to be pointing anywhere in particular. The script assumes the
two are misaligned, and often that the ship is nowhere near the port's axis.

---

## 1. The frame that matters is the target port's

A rendezvous is finished when two *vessels* share a position. A docking is
finished when two *ports* share a position, an axis, and (for anything fussier
than a stock Clamp-O-Tron) a roll. Those are different problems, and the second
one is not the first one done more carefully.

So every quantity in the script is measured in the target port's frame, from its
node, along its outward normal:

```
sep    = ourPort:NODEPOSITION - tgtPort:NODEPOSITION
axGap  = sep * tgtAxis                    how far out in front of the port we are
latVec = sep - tgtAxis*(sep * tgtAxis)    how far off its axis, across it
alignErr = 180 - angle(ourAxis, tgtAxis)  how far from face-to-face
```

A docking is `axGap`, `latVec` and `alignErr` all zero at the same moment. The
two vessels' own attitudes never enter into it — which is why the script asks
you to control from the port. Once you do, the ship's forward axis *is* the
port's axis, and steering the port is the same thing as steering the ship.

> `PORTFACING` exists in kOS precisely because a docking port's **part** facing
> is not the direction it mates along. Where an older kOS does not have it, the
> script derives the axis geometrically instead: the node sits proud of the
> part's centre along exactly that axis, so `NODEPOSITION - POSITION` normalised
> is the same vector.

## 2. The approach is routed, not aimed

The only direction a port can be entered from is straight out in front of it.
Arriving from anywhere else means crossing the station — and "fly toward the
port" is how a script puts a spaceplane through a solar array.

So the path is three legs, and which one is being flown is recomputed from live
geometry every pass rather than planned once:

| Where we are | Where we aim | Why it is safe |
|---|---|---|
| Level with the port's plane or behind it, and closer in than the keep-out radius | Straight out sideways, holding station along the axis | Distance to the station only increases |
| Out at a clear radius, still behind the plane | Forward to the entry plane, at that same radius | Distance stays at least the keep-out radius |
| In front of the entry plane | The entry point itself | Both ends of that line are at least `STANDOFF` in front of the port, so every point on it is |

Being pushed off a leg does not need recovery logic: the next pass simply reads
the geometry again and picks the leg that fits.

### The keep-out sphere is measured, not assumed

Its radius is the farthest part of the target from its port, plus the farthest
part of ours from ours, plus `CLEARANCE`:

```
KEEP_R   = max(KEEPOUT_MIN, ourSpan + tgtSpan + CLEARANCE)
STANDOFF = max(STANDOFF_MIN, KEEP_R + CLEARANCE)
```

Both spans come from walking the two part trees. A 200-part station gets a wide
berth and a probe gets a small one, without anyone retuning anything, and the
number covers the ship *rotating* at the standoff as well as sitting there — the
Mk3's tail sweeps a 20 m radius about a dorsal port when the nose comes round.

A script that hard-codes "stand off 50 m" is one big station away from a
collision. The standoff sits a further `CLEARANCE` outside the sphere rather
than on it, so that the braking guard in §4 has room to measure.

## 3. The corridor, the cone, and the order things are zeroed in

Two nested tubes around the port's axis:

```
corridor radius = max(1.5 m,  0.25 * axGap)     where closing is permitted at all
docking cone    = max(0.12 m, 0.05 * axGap)     where the axial rate is allowed to be non-zero
```

Inside the final approach the closing rate is **zero** unless the ship is inside
the cone *and* the port axes are within `ALIGN_HOLD`. Lateral error is corrected
first, angle second, axial gap last. Correcting 2 m of lateral error at 40 m
costs a few centimetres per second; discovering it at 30 cm costs a bent port.
The cone is the whole difference between a docking and a collision that happened
to be slow.

Drift out of the corridor entirely and the script says so and re-enters it,
rather than pressing on from wherever it has ended up.

## 4. The approach speed is what the RCS can stop from

Same law `rendezvous.ks` flies by, and for the same reason:

```
v_max = sqrt(2 * a * d / BRAKE_SAFETY)
```

where `d` is the distance left and `a` is the translational acceleration the
thrusters **actually deliver** — started from the blocks installed and the mass
aboard, then replaced by measurement as soon as the ship moves. A loaded Mk3
approaches at a quarter of the speed of an empty one because it is a quarter as
able to stop, and nobody has to remember to change a constant.

Only intervals spent at *full* command are sampled. Acceleration measured
through a half-throttled correction is half the authority the ship has, and
feeding that back would ratchet the approach slower and slower, each sluggish
sample justifying the next.

### The gain is not the rendezvous gain

`rendezvous.ks` chases a velocity error at gain 1.2 and the RCS deadband is in
command units. Out there a lazy 30 % command costs a slightly slower approach.
Here it costs a ship that computes a correct braking schedule and then **fails
to fly it** — the simulation in *Verification* below caught exactly that, a
heavy ship sailing 2.3 m past the port with its thrusters doing their
conscientious best. `dock.ks` uses gain 6 with the deadband expressed in m/s of
error, so a sixth of a metre per second is already full authority.

### The guard, and the question asked before anything moves

Outside the corridor the station is a sphere to be missed, not a port to be
aimed at. If the closing speed is more than the thrusters can shed in the room
left before that sphere, everything else stops mattering: the script nulls the
relative velocity and lets the routing start again.

The same arithmetic is asked **at pre-flight**, once the sphere has been
measured:

```
closing^2 > 2 * a * (range - KEEP_R)   ->   refuse to start
```

A ship that fails that test cannot be saved by any control law. It arrives
inside the station whatever it does with its thrusters, so the honest answer is
to stop first — which is what `rendezvous.ks` leaves it doing.

## 5. The last half metre belongs to the magnets

Inside the port's own acquire range — read off the part, not guessed — the
thrusters stop commanding anything but a creep and the ship coasts. The magnets
are stronger and better aimed than RCS is at that distance.

If they do not bite within `CAPTURE_WAIT`, the script backs off to the standoff
and re-flies the approach rather than pushing harder, up to `CAPTURE_TRIES`
times. Pushing harder into a port that is not latching is how two ships end up
bouncing.

## 6. Monopropellant is the currency, and it is priced first

The whole approach is flown on RCS; the main engine is never used, since it
points backwards out of a ship whose nose is on the station. So mono is the only
currency, and each leg costs two speed changes — get moving, stop again — at
whatever speed the braking schedule would allow over its length:

```
legCost(len, cap) = 2 * min(cap, sqrt(2 * a * len / BRAKE_SAFETY))
```

The bill is the three legs plus the relative velocity to be killed plus
`MONO_TRIM` for attitude and station-keeping, and the verdict is **GO /
MARGINAL / NOT ENOUGH** before the first thruster fires. A shortfall does not
stop the flight — the approach stops at `MONO_RESERVE` and *holds* rather than
drifting — but it is said out loud first.

## 7. What it refuses to start

Every one of these is a failure that is cheaper to find on the pad than at two
metres:

| Check | Why |
|---|---|
| Target is a docking port, not a vessel | A vessel is a rendezvous target |
| ...on another ship, and not already docked | Including "that port is on this ship" |
| ...not shielded shut | Its shield can only be opened from that craft |
| ...on a vessel that is loaded | Outside physics range a port has no position to fly to |
| Node types match | A Sr. and a regular will never latch, however well lined up |
| Our port identified without guessing | Docking the wrong end of a spaceplane is not a guess worth making |
| Our shield open | Opened automatically if the port offers the event |
| RCS blocks aboard, mono above reserve | There is no second way to translate |
| Target not spinning | `ROT_REFUSE`, measured over two seconds rather than assumed |
| We can stop before the keep-out sphere | §4 |

## Verification

The script cannot be run outside KSP, so the guidance was flown outside the game
first: the geometry, the routing, the phase machine and the translation
controller reimplemented exactly as the script computes them, against a point
mass with per-axis RCS saturation, an attitude that settles at a finite rate,
and a target port that can rotate.

Thirteen starting states: abeam at 200 m and 400 m, 300 m directly behind the
station, under-and-behind, hot arrivals light and heavy, a 100 t loaded ship at
a quarter of the translational authority, a target turning at 0.5 °/s, a 2 m/s
lateral drift, a start already inside the corridor, and three that start
*inside* the keep-out sphere.

**The first run found two real defects**, both now fixed and both invisible from
reading the code:

* **The inherited gain could not hold the brake schedule.** At `rendezvous.ks`'s
  gain of 1.2 a velocity error of 0.27 m/s produces about a third of full
  thrust, so the ship tracked its commanded profile with a long lag and arrived
  at the port still moving — the 100 t case overshooting the node by **2.3 m**,
  which is a collision that the magnets happened to hide. Hence gain 6 and a
  deadband in m/s.
* **A fixed start-speed cap accepted unstoppable arrivals.** 12 m/s on a light
  ship and 8 m/s on a heavy one are both under the old 15 m/s limit and both
  *physically* unstoppable in the room available — 0.15 m/s² needs 480 m to shed
  12 m/s. Hence the pre-flight braking test, which now refuses both.

After those changes, across all thirteen:

| | Result |
|---|---|
| Dockings | 11 of 11 attempted; the other 2 refused at pre-flight, correctly |
| Contact state | port nodes together, **≤ 0.06 m** off axis, **0.10–0.13 m/s** closing |
| Overshoot past the port plane | **0.00 m**, every case |
| Keep-out sphere entered while off the approach axis | **never** |
| Cases that start inside the sphere | never go deeper than they started — straight out, round, then in |
| Monopropellant spent | 2.1–18.2 m/s, against bills of 13–18 m/s: the estimate is conservative in every case |

This is not a substitute for flying it. What it establishes is that the geometry
and the control law are right, which is the part that cannot be trimmed out
afterwards by nudging a constant.

## What to trim after the first flight

| Constant | What it decides | Symptom that it is wrong |
|---|---|---|
| `TRANS_GAIN` | How hard the thrusters chase a velocity error | Overshoots the port (too low), thruster chatter and wasted mono (too high) |
| `BRAKE_SAFETY` | Padding on every stopping distance | Approach feels sluggish, or arrives hot |
| `V_FINAL` / `V_CAPTURE` | Closing rates down the corridor and at the port | Final takes forever, or contact is harder than you like |
| `LAT_CONE` | The docking cone | Never closes at all (too tight), or contact off-centre (too loose) |
| `ALIGN_TOL` | Attitude accepted before closing starts | Sits at the standoff waiting for an alignment the ship cannot hold |
| `STANDOFF_MIN` / `CLEARANCE` | The corridor entry distance | Approach starts absurdly far out, or too close for comfort |
| `CAPTURE_MIN` | Fallback acquire range when the port will not say | Pushes into the port, or stops short of the magnets |
| `MONO_RESERVE` | When to stop rather than drift | Stops short with mono still worth spending |
| `ROT_REFUSE` | How lively a target may be | Refuses a station you could have docked with |
| `RCS_AXIS_FRAC` | Starting guess at translational authority | First few seconds badly paced (it self-corrects from measurement) |

`LOG_TO_FILE` appends every resource snapshot to `0:/dock_log.csv` — mass, mono,
EC, mono ΔV and the measured RCS acceleration, before and after. That is the raw
material for the same kind of measured review the other constants in this
repository came from.
