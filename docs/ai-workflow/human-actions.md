# Human actions

## Required for M14 onward (arm)
Sessions cannot use `sudo`. Simulated with `apt-get install -s`: 61 new packages for the first three names, with no removals.
```bash
sudo apt install --no-install-recommends ros-jazzy-moveit ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers
```
About 13 GB of disk was free on 2026-11-01. Then start the loop:
```bash
rm STOP
docs/ai-workflow/run_applepic.sh 23:30
```

## Publishing (when you decide to)
Create the GitHub repo and push yourself; sessions never do.

## After M21
Run `make demo` once on the machine with a screen and check the windows open and the arm picks apples.
