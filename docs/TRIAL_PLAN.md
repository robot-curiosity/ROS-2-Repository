# Field Trial Plan (Dec 30 to Jan 12)

Plan for the field trials that feed the LACSEF/ISEF 2026 results (submission
Jan 19-20, 2027). Everything marked "TODO: Bryan to fill in" is a decision or
fact only Bryan has. Numbers marked "default" are agent suggestions, not
decisions; change them freely.

## Research question and success criterion

- Research question: TODO: Bryan to fill in
- Success criterion: TODO: Bryan to fill in
- Default pickup target (labeled default, from the project notes): at least
  80 percent of litter items attempted end up bagged. `tools/status.py` and the
  sim tests use this same 0.80 value.
- First milestone before the trials: one can bagged 3 of 10 by Dec 1.

## Before every trial day

1. `ros2 run trash_vision hardware_check` shows PASS for every subsystem in use.
2. `python3 tools/status.py` shows no FAIL rows in `docs/HARDWARE_TESTS.md`.
3. Battery charged, fuse in, bag empty, arm clear, hand near the power switch.
4. Behavior started with `enabled:=true`. Trial logging writes to
   `~/curiosity_trials/run_*.jsonl` (one file per behavior start).

## Litter types and counts (default)

Counts are per run (one run = one behavior start, one layout, one surface).
Type names must match the detector's `litter_type` / `class_name` so the
summary groups them correctly. TODO: Bryan to fill in the real list.

| Litter type | Count per run | Notes |
|-------------|---------------|-------|
| can (aluminium) | 4 | the type the pick zone was calibrated on |
| plastic bottle | 3 | TODO: Bryan to confirm size |
| paper / wrapper | 3 | likely hardest for grasping and detection |

Total 10 items per run (default), matching the 3 of 10 milestone.

## Placement layouts

Use the same marked layouts every time so conditions are comparable. Mark item
positions on the ground with tape and photograph each layout.

| Layout | Description |
|--------|-------------|
| A open | 10 items spread in an open area, at least 1 m from walls and from each other |
| B wall | 5 items within 0.3 m of a wall or curb (tests side sonars), 5 in the open |
| C cluster | 10 items in 3 groups of 3 to 4, items about 0.3 m apart |
| D obstacle | layout A plus 2 to 3 fixed obstacles (cones, box) on the search path |

## Surfaces

TODO: Bryan to fill in which surfaces are available. Default list: smooth
concrete or asphalt, short grass, gravel or dirt. Record the surface and the
lighting (sun, shade, overcast) for each run, since both affect the detector.

## Runs per condition (default)

- 3 runs for each layout x surface pair actually available.
- Each run ends when all items are bagged, a 10 minute timeout, or an
  emergency stop. Report stops and timeouts; never drop a failed run.
- Randomize which layout and surface goes first each day, so time of day and
  battery level do not line up with one condition.
- With 3 runs of 10 items, one condition gives 30 attempts. TODO: Bryan to say
  if that is enough for the claim being made.

## What to record per run

Automatic (the trial log, one JSON line per pickup attempt):

| Field | Meaning |
|-------|---------|
| run | run id (log file name) |
| t_start, t_end | attempt start and end time |
| detected | always true (an attempt starts when a target is acted on) |
| approached | robot reached the pick position |
| picked | arm grabbed the item |
| success | item deposited in the bag |
| litter_type | detector class, or `unknown` |
| confidence | detector confidence |
| failure_reason | why a failed attempt failed, null on success |

By hand, on the sheet below, per run: date, start time, layout, surface,
lighting, items placed, items bagged (compare with the summary), battery
voltage at start and end, software version (git commit), operator
interventions (what and when), and notes. Items the robot never detected do
not appear in the log, so "items placed" minus the attempt count is the
number of items missed by search or detection; record it.

## Running the summary

    python3 tools/status.py ~/curiosity_trials     # hardware rows, unset config, pickup rate vs 80 percent
    ros2 run trash_vision trial_summary            # pickup rate by litter type and per run (default dir ~/curiosity_trials)
    ros2 run trash_vision trial_summary run_a.jsonl run_b.jsonl   # chosen files only

Copy the log files off the Pi after each day and keep them with this sheet.
Do not edit them.

## Results sheet (fill in)

| Date | Run file | Layout | Surface | Light | Placed | Attempts | Bagged | Missed (placed - attempts) | Pickup rate | Interventions | Notes |
|------|----------|--------|---------|-------|--------|----------|--------|----------------------------|-------------|---------------|-------|
|      |          |        |         |       |        |          |        |                            |             |               |       |
|      |          |        |         |       |        |          |        |                            |             |               |       |
|      |          |        |         |       |        |          |        |                            |             |               |       |
|      |          |        |         |       |        |          |        |                            |             |               |       |

Pickup rate here is bagged / placed (end to end). The summary tool reports
bagged / attempts; quote both and say which is which.

## Per-condition totals (fill in)

| Layout | Surface | Runs | Placed | Bagged | End-to-end rate | Meets criterion? |
|--------|---------|------|--------|--------|-----------------|------------------|
|        |         |      |        |        |                 | TODO: Bryan to fill in |
