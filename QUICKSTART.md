# Project AirSim Quick Start

## Launch Blocks

In the first terminal:

```bash
cd /home/tom/Documents/ProjectAirSim
./packages/Blocks/Development/Linux/Blocks.sh -log
```

Wait for the Blocks environment to open.

## Run the drone smoke test

In a second terminal:

```bash
cd /home/tom/Documents/ProjectAirSim
source airsim-venv/bin/activate
cd client/python/example_user_scripts
python hello_drone.py
```

The example should spawn `Drone1`, display its camera feeds, take off, move up
and down, and land. Keep Blocks running while using client scripts.

Other examples are available in `client/python/example_user_scripts`, including:

```bash
python hello_rover.py
```

## Headless launch options

Run with off-screen rendering (camera rendering remains available):

```bash
./packages/Blocks/Development/Linux/Blocks.sh -RenderOffScreen
```

Run without rendering (camera images are unavailable):

```bash
./packages/Blocks/Development/Linux/Blocks.sh -nullrhi
```

Use the Development package for normal testing and the Shipping package for
performance testing.
