# Curiosity: project docs

Curiosity is a low-cost autonomous litter-collection robot for LACSEF 2027 (and ISEF 2027 if it qualifies). This folder is the single place to find where the project stands.

Last consolidated: 2026-11-11.

| File | What it answers |
|---|---|
| [STATE.md](STATE.md) | What exists today: deadlines, hardware, CAD, software, simulation results |
| [DECISIONS.md](DECISIONS.md) | What has been decided, what is deliberately deferred, and what is still open |
| [RISKS.md](RISKS.md) | What is likely to go wrong, ranked, with a mitigation for each |
| [REPO-REVIEW.md](REPO-REVIEW.md) | What is in this repo, and how it compares with the simulation results |
| [FABLE-STUDY-BRIEF.md](FABLE-STUDY-BRIEF.md) | The brief given for a deep study of the pick-up, obstacle detection, recognition, selection and navigation algorithms |
| [FABLE-STUDY-REPORT.md](FABLE-STUDY-REPORT.md) | The study's result: a two-week plan, a section per question, a dated schedule to Jan 20 and five experiments. Recommendations, not decisions |

## Where the other pieces live

| Piece | Where |
|---|---|
| CAD (Onshape document "Curiosity") | https://cad.onshape.com/documents/6b928e27503cbc7da1ba3570/w/d8a623637ff4828a6fd765d3 |
| ROS 2 robot parameters (frames, drive, sonar, arm, footprint, wiring) | Claude doc "Curiosity: ROS 2 Robot Parameters" (https://claude.ai/artifact/Nmd1tS9WCJEd3bL92Xf5vJ) |
| SO-101 arm studio (poses, macros, virtual arm, servo load readout) | https://github.com/robot-curiosity/arm-sim |
| This repo (ROS 2 workspace) | https://github.com/robot-curiosity/ROS-2-Repository |

## Rules for keeping this current

- Change a decision in DECISIONS.md the same day you change it in the robot.
- Put measured numbers in STATE.md with the date they were measured. Estimates say "estimate".
- Do not commit `build/`, `install/`, `log/`, core dumps or model weights. See REPO-REVIEW.md.
