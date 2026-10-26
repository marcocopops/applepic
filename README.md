# ApplePic

POC en simulation (ROS 2 Jazzy + Gazebo Harmonic) d'un robot ramasseur de pommes tombées au sol.
Le robot suit une boucle de pommiers, cherche les pommes au sol à la caméra, les ramasse,
puis va vider son panier à une zone de dépôt.

## Prérequis
- Ubuntu 24.04 avec ROS 2 Jazzy et `ros-jazzy-ros-gz` installés (`/opt/ros/jazzy`).
- Optionnel : Docker (`make docker-image`, puis `USE_DOCKER=1 make build`).

## Commandes
```bash
make build          # compile le workspace
make test           # tests unitaires
make world          # régénère le monde du verger
make sim-headless   # lance la simulation sans GUI
```

État d'avancement : voir `docs/BILAN.md` et `docs/PROGRESS.md`.
