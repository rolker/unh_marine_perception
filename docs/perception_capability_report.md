# BizzyBoat — Perception & Collision-Avoidance Capability

*A plain-language, evidence-backed snapshot of how well BizzyBoat — an uncrewed survey boat —
currently sees and avoids obstacles. For operators, partners, and management. Every figure and
number below is drawn from BizzyBoat's own logged field data (see "How we measured this").*

*UNH Center for Coastal and Ocean Mapping / Joint Hydrographic Center · 2026-06*

---

## The short answer

**BizzyBoat reliably detects boats and steers around them, is conservative by design, and is
always operated under human supervision.**

- In logged field data the onboard AI flagged **moored sailboats dead ahead at 92–98 %
  confidence**. Across a season we ground-truthed **850 real obstacle encounters** — boats,
  buoys, docks, lobster-pot floats — to measure this honestly.
- Its path-planner **actively reshapes the boat's route around detected obstacles**; in field
  trials it routed around a moored sailboat. An independent **emergency-stop** layer halts the
  boat for close-range objects (stopping distance ≈ **2 m at survey speed, under a meter at
  slow, close-quarters speed**).
- It **errs toward caution** — when unsure, it slows or stops rather than pressing on.
- **A human operator always monitors a live feed and can take manual control at any moment.**
  The boat is uncrewed, not unsupervised.

![A moored sailboat ~8 m dead ahead (left) and the AI's segmentation (right): the boat is
detected as an obstacle at 0.98 confidence.](figures/moored_boat_dead_ahead.png)

*Above: a moored sailboat directly ahead, detected as an obstacle (red) at 0.98 confidence.*

**Honest limitation:** on calm, sunny water the AI sometimes slows or stops unnecessarily when
sun or cloud reflections on the surface look like an object. This is a nuisance, not a hazard —
it errs toward stopping, never toward collision — and we have operator controls plus a software
fix in progress for it (below).

**For a first visit to a mooring field** we would run a slow, supervised familiarization pass
before any autonomous operation near boats.

---

## How it works

BizzyBoat carries four cameras covering all directions. An onboard AI labels every pixel as
**water, sky, or obstacle**. Detected obstacles feed **two independent safety layers**: a
**planner/avoider** that reshapes the boat's path to go around obstacles with margin, and a
fast **emergency-stop reflex** that halts the boat if something is close ahead. Both run
continuously, under operator supervision.

## What it detects well

Measured detection confidence on real, logged obstacles (higher = more certain):

| Object | Detection confidence | Notes |
|---|---|---|
| **Boats (moored)** | **0.97** | the planner routes around these |
| **Buoys** | **0.94** | detected reliably at close range |
| **Lobster-pot floats** | **0.85** | directly relevant to survey work |
| Docks / piers / pilings | 0.92–0.99 | detected confidently; handled by the planner |

![A buoy on the water (left) detected as an obstacle (right) at 0.98
confidence.](figures/buoy_detection.png)

## How it avoids obstacles

- **Planner / avoider (primary layer):** continuously reshapes the followed path to pass
  detected obstacles — boats, docks, structures — with margin. This is the layer that handles a
  mooring field.
- **Emergency-stop reflex (backstop):** a fast, last-resort stop for close water-level objects
  directly ahead (buoys, floats, debris, a swimmer). Stopping distance ≈ 2 m at survey speed,
  sub-meter when slow.
- **Conservative tuning:** uncertainty resolves toward slowing/stopping.
- **Operator supervision:** live monitoring with instant manual takeover.

## Honest limitations

- **Calm-water reflections** (sun/cloud mirrored on glassy freshwater) can be mistaken for
  obstacles, causing occasional unnecessary slowdowns/stops. The cause is understood — the AI
  was trained on ocean imagery and is new to mirror-calm freshwater — and it errs toward
  caution. *Being addressed* by operator-selectable sensitivity modes (now) and an AI update
  (durable fix, in progress).
- **Small floating debris** (sticks, weed) is detected less reliably than solid objects — it is
  small, low-contrast, and resembles surface texture.
- **Tall structures** (docks, large boats) are seen confidently and handled by the planner; the
  close-range emergency reflex is not the right tool for them — the planner is.
- **Always operator-supervised** — this is a snapshot of an assisted system, not a claim of
  unsupervised autonomy.

![A calm-water scene where cloud reflections on the surface are mis-read as obstacles (red),
the main source of unnecessary slowdowns.](figures/calm_water_reflection_false_alarm.png)

*Above: cloud reflections on calm water mis-classified as obstacle — the false-alarm case the
operator modes and the AI update address.*

## Operator controls & what's improving

- **Sensitivity modes (radar-style, available now):** a *Balanced* default and a
  *Suppress-reflection* mode the operator can select when calm water causes nuisance stops — the
  same idea as switching on sea-clutter rejection on a marine radar.
- **AI model update (durable fix, in progress):** retraining the perception model on in-domain
  freshwater imagery — including the exact reflection cases — to remove the false alarms at the
  source. This is the permanent fix and improves every layer at once.

## How we measured this

These numbers are measurements, not estimates. We mined a full season of BizzyBoat's own logged
runs — roughly **100 recorded missions** — extracted every obstacle-detection and stop event,
and **visually ground-truthed 850 encounters** against the synchronized camera footage, labeled
by object type. Detection confidence, false-alarm rates, and stopping distances are all derived
from that record. The same pipeline re-runs each deployment, so this snapshot updates as the
system improves.

---

*Prepared from the 2026-06 perception / collision-avoidance analysis. Detailed figures and
methodology available on request.*
