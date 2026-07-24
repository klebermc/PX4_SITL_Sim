# PX4_SITL_Sim

> **Note:** All code in this repository (Dockerfile, shell scripts, and the `offb` ROS package) was written by Kleber Cabral. The README documentation and inline code comments were added with AI assistance (Claude).

A Dockerized PX4/ROS Noetic/Gazebo software-in-the-loop (SITL) drone simulation environment, plus a custom ROS "offboard" controller package (`offb`) for position/velocity control of a simulated (or real) PX4 vehicle via MAVROS.

## Structure

```
PX4_SITL_Sim/
├── Dockerfile              # PX4 + ROS Noetic + Gazebo SITL image
├── install.sh              # (re)builds the ros-test image
├── clean.sh                # removes the image + dangling images
├── start.sh                # runs the container, mounting catkin_ws/ and host/
├── terminal.sh              # opens an extra shell into the running container
├── commands_for_sitl        # cheat-sheet of commands used inside the container
├── worlds/
│   └── danger_zones.world   # custom Gazebo world
├── catkin_ws/
│   └── offboard_package/    # ROS catkin workspace, mounted live into the container
│       ├── settings/        # rviz config
│       └── src/
│           ├── offb/        # main catkin package (offboard controller node, launch files)
│           └── python/      # standalone MAVROS python scripts (origin-setting, offboard demo)
├── host/                    # runtime mount point (created by start.sh); holds recorded
│                             # rosbags and a working copy of the world file — not source, gitignored
└── legacy/
    └── start_old.sh          # superseded container-launch script, kept for reference
```

`catkin_ws/offboard_package/build/`, `devel/`, `logs/`, and `.catkin_tools/` are catkin-generated build artifacts (gitignored) — they're recreated automatically the first time you build the workspace inside the container.

## What it does

- **`Dockerfile`** builds a ROS Noetic image with a full PX4-Autopilot clone (built via its Ubuntu setup script), MAVROS, and a non-root `px4devel` user.
- **`start.sh`** runs that image, bind-mounting `catkin_ws/` and `host/` (created next to the script) into the container as `~/catkin_ws` and `~/host`, with X11 forwarding for GUI apps (Gazebo, rviz, rqt).
- **`catkin_ws/offboard_package`** is a ROS catkin package (`offb`) containing:
  - `offb/src/offb.py`, `offb/src/main_FB_hdw.py`, `offb/src/main_veltuning_hdw.py` — offboard control nodes (position/velocity setpoint control loops against MAVROS topics/services), at different stages of development (hardware feedback control, velocity tuning).
  - `offb/src/TSG.py` — a `Controller` class, referenced by the main control scripts.
  - `offb/src/utils/px4_utilities_FB.py`, `offb/src/utils/animate.py` — MAVROS helper functions and plotting/animation utilities.
  - `python/offb.py`, `python/offb_python_script.py`, `python/set_origin.py` — standalone (non-catkin-package) MAVROS scripts, including one that sends `SET_GPS_GLOBAL_ORIGIN`/`SET_HOME_POSITION` for SITL.
- **`worlds/danger_zones.world`** is a custom Gazebo world used for SITL runs (see `commands_for_sitl` — it's loaded via `PX4_SITL_WORLD`).
- **`commands_for_sitl`** is a plain-text notes file of commands run inside the container (launching SITL/Gazebo, `roslaunch mavros`, `rqt_plot` setpoint-tracking plots, sourcing the workspace, `rostopic echo`, RC-loss parameter tuning) — useful as a runbook, not a script.

## Setup / Run

```bash
./install.sh        # builds the ros-test Docker image
./start.sh           # runs the container (mounts catkin_ws/ and host/, forwards X11)
```

Inside the container, build the workspace and source it:

```bash
cd ~/catkin_ws/offboard_package
catkin build
source devel/setup.bash
rosrun offb main_FB_hdw.py
```

See `commands_for_sitl` for the PX4 SITL/Gazebo launch commands and useful debugging one-liners (`rqt_plot`, `rostopic echo`, etc.).

`terminal.sh` opens an additional shell into whichever container is currently running. `clean.sh` removes the built image plus any dangling images.

## Key dependencies

- Docker
- Everything else (ROS Noetic, PX4-Autopilot, Gazebo, MAVROS) is installed inside the image by the `Dockerfile` — no host-side ROS install required.

## Status

Working SITL setup, last actively used mid-2022. Several development-stage control scripts exist side by side (`main_FB_hdw.py` vs `main_veltuning_hdw.py`) rather than one finished entry point — check `commands_for_sitl` and the script contents to see which was the active one for a given experiment. `legacy/start_old.sh` is an earlier, non-catkin_ws-aware container launch script superseded by `start.sh`.

`offb/src/offb.py`, `python/offb.py`, and `python/offb_python_script.py` are all the same square-pattern offboard demo (the two `python/` copies are byte-identical to each other) — `main_FB_hdw.py`/`main_veltuning_hdw.py` are the more developed control scripts that superseded this demo. `TSG.py`'s flocking/danger-zone-avoidance controller (used by `main_FB_hdw.py`) depends on `sigma_norm`/`sigma_1`/`rho_h` helper math that's defined within `TSG.py` itself, so it runs standalone.

**Bugs fixed (2026-07-24):** `px4_utilities_FB.py`'s `VelocitySetpoints.updateSp` referenced an undefined `vel` instead of its own `velocity` parameter (dead code path — never actually called, only `updateSp2` is used, but fixed regardless). `main_FB_hdw.py` used to hard-`quit()` the process the instant it got within 0.1m of the mission waypoint instead of cleanly landing; removed — the existing `FB_Cont.done()` check (0.25m threshold) already transitions cleanly to the IDLE state, which ramps down and triggers `AUTO.LAND`.

## Cleanup notes (2026-07-24)

During reorganization, the following were removed as redundant/regenerable:
- `backup.zip` (340MB) and `SITL_docker_FB_controller.zip` (170MB) — point-in-time backups of this same workspace; both exceeded GitHub's 100MB file size limit anyway. Both contained a `git init` with **zero commits** (verified before deleting) — no history was lost.
- `kleber_docker_controller_V1.zip`/`V2.zip` — small superseded snapshots of the controller source, predating the current `catkin_ws/offboard_package/src`.
- A stale top-level `offboard_package/` directory that duplicated `catkin_ws/offboard_package/` but was missing files present in the latter (the live/mounted copy).
- An empty, commit-less `.git/` directory nested inside `catkin_ws/offboard_package/` (would otherwise behave like a broken submodule link once this project became its own repo).

## License

MIT — see [LICENSE](LICENSE).
