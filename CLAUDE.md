# ApplePic — contexte pour les sessions Claude

POC en simulation d'un robot ramasseur de pommes (base différentielle ~60x45 cm, caméra,
panier, préhenseur "magique"). Consignes complètes : `KICKOFF.md`. Mémoire inter-sessions :
`docs/PROGRESS.md` (état + prochaine tâche), `docs/DECISIONS.md`, `docs/BLOCKERS.md`, git log.

## Environnement
- Ubuntu 24.04, ROS 2 **Jazzy** + **gz Harmonic (gz sim 8)** installés nativement dans `/opt/ros/jazzy`.
- Workflow natif par défaut (Docker optionnel : `make docker-image` puis `USE_DOCKER=1 make ...`).
- GPU NVIDIA dispo : le rendu caméra headless (`gz sim -s --headless-rendering`) fonctionne (vérifié en session 1).
- Topics robot : `/cmd_vel`, `/odom` (vérité terrain, repère monde), `/wheel_odom`, `/tf`,
  `/joint_states`, `/camera/image_raw`, `/camera/camera_info`, `/clock`.
- Pas installés : Nav2, robot_localization (voir `docs/TODO_HUMAIN.md`).

## Commandes
- `make build` : `colcon build --symlink-install`
- `make test` : build + `colcon test` + résultats
- `make world` : régénère le monde SDF du verger depuis la config YAML
- `make sim-headless` : lance la simu headless pendant `SIM_SECONDS` (défaut 20)
- `scripts/check_drive.sh [x y yaw]` : smoke test conduite (cmd_vel → odom) + captures caméra
- `scripts/capture_camera.py out.png` : sauve une image de `/camera/image_raw` (simu lancée)
- Arrêter une simu lancée en arrière-plan : par PGID (`set -m` puis `kill -INT -- -$PID`), jamais par motif.
- Shell manuel : `source /opt/ros/jazzy/setup.bash && source install/setup.bash`

## Arborescence
- `src/applepic_description` : URDF/xacro du robot
- `src/applepic_gazebo` : générateur de monde (verger), mondes SDF, bridge ros_gz
- `src/applepic_navigation` : suivi de waypoints
- `src/applepic_perception` : détection HSV + projection pixel→sol
- `src/applepic_mission` : machine à états de mission
- `src/applepic_bringup` : launch files de haut niveau
Tous les paquets sont `ament_python`. La logique pure (sans rclpy) est dans des modules
séparés testables par pytest sans ROS lancé.

## Conventions
- Code/noms/commentaires en anglais ; docs en français.
- Conventional commits en anglais, petits et fréquents ; jamais de réécriture d'historique, jamais de push.
- Pas de sudo. Actions humaines → `docs/TODO_HUMAIN.md`.

## Vérifier
- `make test` doit passer (0 failures).
- Simu : `make sim-headless`, puis dans un autre shell `gz topic -l` / `ros2 topic list`.
- Captures caméra : sauvegardées en PNG dans `artifacts/` (ignoré par git).
