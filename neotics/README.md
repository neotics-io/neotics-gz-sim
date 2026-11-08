# Neotics kit for Gazebo Sim

This folder is Neotics' own work, added to our copy of [Gazebo Sim](https://github.com/gazebosim/gz-sim). It turns any Neotics scene into a Gazebo world with a position-controlled arm.

It is one part of the Neotics stack. Every fork in [neotics-io](https://github.com/neotics-io) reads the same two files:

- a **Neotics Scene** (`samples/depot.scene.json`): the room, the obstacles, the arm and the named places;
- a **Neotics Episode** (`samples/depot.episode.json`): a run recorded in MuJoCo, with time, joint positions and commands.

The depot is a sample made for these tools. It is not an entry or an answer for any [Neotics challenge](https://lab.neotics.io).

## Use it

Run from this folder:

```bash
python scene_to_sdf.py samples/depot.scene.json depot.sdf
gz sim -r depot.sdf
```

## How it's checked

Validates the world with Gazebo Harmonic, runs it, confirms the crate stays on the table and that the arm follows a command. This runs on GitHub in `.github/workflows/neotics.yml`, on every change to this folder.

## Licence

See [`LICENSE`](LICENSE). It covers this folder only.
