# Progression

## Session 1 (2026-10-26)
- M0 fait : git, workspace `src/` avec 6 paquets ament_python, Makefile, Dockerfile optionnel, docs.
  - Vérifié : `make build` (6 paquets OK), `make test` (6 tests, 0 échec).
  - Vérifié : rendu caméra headless avec `gz sim -s -r --headless-rendering` (monde jouet, image reçue sur `gz topic`).

## Session 2 (2026-10-26)
- M1 fait : générateur de verger `applepic_gazebo/orchard.py` + CLI `generate_world` (`make world`),
  config `src/applepic_gazebo/config/orchard.yaml`, monde `worlds/orchard.sdf` (6 arbres, 29 pommes)
  et manifeste `worlds/orchard_apples.yaml`.
  - Vérifié : `gz sdf -k` → Valid ; `gz sim -s -r` : 29 modèles `apple_*`, pose de `apple_t0_0`
    identique au manifeste après ~10 s (pommes stables) ; capture vue de dessus
    `scripts/capture_overhead.py` → `artifacts/orchard_overhead.png` (sol, dépôt, arbres, pommes visibles).
  - `make test` : 12 tests, 0 échec (6 tests du générateur).
- Remarque : `run_applepic.sh` a une modif locale non commitée faite par l'humain ; laissée telle quelle.

## Prochaine tâche : M2 — Robot
- `applepic_description/urdf/applepic.urdf.xacro` : base diff ~0,60 x 0,45 m, 2 roues motrices + roulette,
  caméra avant inclinée vers le sol (~0,5 m de haut, tilt ~45°), panier (visuel), préhenseur (link avant bas).
- Plugins gz : DiffDrive (`/cmd_vel`), OdometryPublisher ou odom du DiffDrive, JointStatePublisher, caméra.
- Bridge ros_gz (`applepic_gazebo/config/bridge.yaml`), launch `applepic_bringup/launch/sim.launch.py`
  (monde orchard + spawn robot au dépôt + robot_state_publisher + bridge), utilisé par `make sim-headless`.
- Vérifier : publier `cmd_vel`, voir `odom` évoluer ; sauver une image caméra robot en PNG.
