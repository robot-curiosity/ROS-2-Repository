# 3D Printing Notes (Bambu Lab H2S)

Lessons and settings from printing the "PRINT -" parts of the Curiosity robot (Onshape doc "Curiosity", part-studio element 26418003fafd25e7a79bb4eb). Printer: Bambu Lab H2S, 340x320x340 mm, single nozzle.

## Incident: sonar side holder printed with a messy underside

**Part:** `PRINT - Sonar holder, side facing (HC-SR04)` (partId R5BD, bbox 18x54x80 mm). Four copies on plate 1, printed on its +X face (80x54x18 mm).

**What happened:** The underside of the holder came out messy because the slicer generated no supports for it.

**Cause:**
- In this orientation the part has about 499 mm² of flat, down-facing ceiling at 15 mm height, plus about 101 mm² at 5.5 mm. Both lie over roughly x 27-53, y 4-50 mm, which is probably the internal pocket for the sensor board. I did not verify whether the pocket is open to the side.
- Plate 1 was sliced with tree supports set to "On build plate only". That mode cannot support a ceiling that sits above the part's own floor, so those faces bridged over air.

**Orientation comparison for this part** (bed contact mm² / overhang mm²):

| Orientation | Contact | Overhang |
|---|---|---|
| +Z | 270 | 421 |
| -Z | 270 | 399 |
| +X (used) | 4302 | 600 |
| -X | 735 | 4166 |
| ±Y | 738 | 405 |

+X gives the best bed contact and the least overhang, but its overhang is an internal ceiling, so supports must be able to start from the part.

**Fixes (neither done yet):**
1. Reprint the four holders with "On build plate only" turned off (support everywhere).
2. Redesign the pocket roof in CAD as a 45° chamfer so it needs no support.

## General lessons

- Check every part's overhang ceilings against the support mode. "On build plate only" fails for internal pockets and ceilings.
- Keep at least 10 mm between tree-supported parts on a plate. Gaps of 6-8 mm caused path-conflict warnings, and plate 4's hubs needed 20 mm.
- The other parts have not been scanned for unreachable ceilings.

## Plates and settings used

| Plate | File | Material | Weight | Time |
|---|---|---|---|---|
| 1 | Curiosity_H2S_plate1_PLA_v2.3mf (32 objects) | PLA Basic | 421.9 g | 12h42m |
| 2a | ..._plate2a_PLA_battery_MDD10A.3mf | PLA Basic | 151.5 g | 4h04m |
| 2b | ..._plate2b_PLA_powerbank_Pi5.3mf | PLA Basic | 143.7 g | 3h52m |
| 3 | ..._plate3_PLA_arm_shelf.3mf | PLA Basic | 104.0 g | 2h41m |
| 4 | ..._plate4_PETG_hubs_clips_v2.3mf | PETG Basic | 462.6 g (83.5 g support) | 18h17m |
| 5 | ..._plate5_TPU_tires.3mf | TPU 95A HF | 258.1 g | 9h39m |

Totals: about 821 g PLA, 462.6 g PETG, 258.1 g TPU, roughly 1.54 kg and 51 h.

- PLA: 3 walls, 25% gyroid
- PETG (plate 4): 4 walls, 30% gyroid
- TPU (plate 5): 15% gyroid, supports off
- Not applied: per-object overrides (4 walls / 100% infill) for bag clips and corner blocks.
