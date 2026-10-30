# ApplePic — contexte pour les sessions Claude

POC en simulation d'un robot ramasseur de pommes (base différentielle ~60x45 cm, caméra,
panier, préhenseur "magique"). Consignes complètes : `KICKOFF.md`. Mémoire inter-sessions :
`docs/PROGRESS.md` (état + prochaine tâche), `docs/DECISIONS.md`, `docs/BLOCKERS.md`, git log.

## Environnement
- Ubuntu 24.04, ROS 2 **Jazzy** + **gz Harmonic (gz sim 8)** installés nativement dans `/opt/ros/jazzy`.
- Workflow natif par défaut (Docker optionnel : `make docker-image` puis `USE_DOCKER=1 make ...`).
- GPU NVIDIA dispo : le rendu caméra headless (`gz sim -s --headless-rendering`) fonctionne (vérifié en session 1).
- Topics robot : `/cmd_vel`, `/odom` (vérité terrain, repère monde), `/wheel_odom`, `/tf`,
  `/joint_states`, `/camera/image_raw`, `/camera/camera_info`, `/clock`, `/imu`, `/gps/raw` (fix parfait),
  `/gps/fix` (bruité, mode gps).
- Troncs : `/trees/trunks` (repère robot, `apple_detector`), `/localization/trunk_fix` (`localizer`).
- Pose pour tous les nœuds : `/localization/odom` (repère `map`) + TF `map → odom → base_footprint`,
  publiés par `localizer` (`localization:=truth` : vérité + identité ; `gps` : EKF GPS + roues + gyro).
  Seul le préhenseur magique lit `/odom` (vérité terrain).
- Pas installés : Nav2, robot_localization (voir `docs/TODO_HUMAIN.md`).

## Commandes
- `make build` : `colcon build --symlink-install`
- `make test` : build + `colcon test` + résultats
- `make world` : régénère le monde SDF du verger depuis la config YAML
- `make sim-headless` : lance la simu headless pendant `SIM_SECONDS` (défaut 20)
- `scripts/check_drive.sh [x y yaw]` : smoke test conduite (cmd_vel → odom) + captures caméra
- `scripts/check_navigation.sh [timeout]` : boucle complète headless (waypoint_follower) + `artifacts/nav_trajectory.png`
- `scripts/check_perception.sh [x y yaw name min]` : détection HSV headless comparée au manifeste + `artifacts/perception_<name>.png`
- `ros2 launch applepic_bringup perception.launch.py x:=2.6 y:=4.0` : simu + `apple_detector` (`/apples/detections`, `/apples/debug_image`)
- `scripts/check_picking.sh [x y yaw min timeout capacity]` : ramassage headless (detector + gripper + apple_picker), recoupe `/basket/*` et `gz model --list`
- `ros2 launch applepic_bringup picking.launch.py x:=9.6 y:=9.5 yaw:=1.57 [capacity:=10]` : simu + détection + ramassage
- `make e2e [E2E_SEED=n]` / `scripts/e2e_mission.sh [min_ratio sim_limit rtf capacity wall_timeout seed]` : scénario M7
  de bout en bout headless (1 boucle, RTF 3 par défaut, ~5 min), PASS/FAIL, métriques `artifacts/e2e_metrics.json`
  (dont poses finales des pommes restées au sol), tracé `artifacts/e2e_trajectory.png` ; avec un seed, monde
  régénéré dans `artifacts/orchard_seed<n>.sdf`. Diagnostic : `grep -E "gave up|left tree" artifacts/e2e.log`
- gz-transport Python : `GzNode.request` garde le GIL pendant l'attente (gèle tout le nœud) → pour un service gz,
  utiliser `gz service` en sous-processus (cf. `gripper.py`)
- Localisation GPS : `make e2e|demo LOCALIZATION=gps [GPS_NOISE=0.02 GPS_BIAS=0.0]`, ou
  `localization:=gps gps_noise:=… gps_bias:=… gps_bias_tau:=… gps_seed:=…` sur tous les launch ;
  `artifacts/e2e_metrics.json` → clé `localization` (erreur vs vérité), tracé vérité en orange.
- `scripts/eval_trunks.py [--output f]` à côté d'une simu : erreur de `/trees/trunks` contre la vérité (arrêt : SIGTERM).
  Lancer une série e2e en arrière-plan : éviter `wait` sans garde-fou sur un script auxiliaire (une boucle a déjà bloqué).
- `make demo [DEMO_RTF=2]` / `ros2 launch applepic_bringup demo.launch.py [rtf:=2 rviz:=false headless:=true]` :
  démo M8 (GUI Gazebo + RViz, nœud `mission_viz` : `/mission_viz/markers|path`)
- `scripts/check_demo.sh [timeout rtf]` : démo headless sans RViz, valide marqueurs/HUD/trajectoire
- Capturer une fenêtre (pas de Xvfb/ImageMagick) : `DISPLAY=:1 xwd -id <win>` (id via `xwininfo -root -tree`),
  puis décodage XWD en Python (PIL ne lit pas le XWD)
- `ros2 launch applepic_bringup mission.launch.py [headless:=false capacity:=10 loops:=1]` : simu + détection + mission
- `ros2 launch applepic_bringup navigation.launch.py loops:=1 [headless:=false]` : simu + suivi de boucle
- `scripts/capture_camera.py out.png` : sauve une image de `/camera/image_raw` (simu lancée)
- Arrêter une simu lancée en arrière-plan : par PGID (`set -m` puis `kill -INT -- -$PID`), jamais par motif.
  `set -m` doit être exécuté AVANT la ligne `... &` (pas dans la chaîne backgroundée), sinon `$!` n'est pas le PGID.
- Shell manuel : `source /opt/ros/jazzy/setup.bash && source install/setup.bash`

## Arborescence
- `src/applepic_description` : URDF/xacro du robot
- `src/applepic_gazebo` : générateur de monde (verger), mondes SDF, bridge ros_gz
- `src/applepic_navigation` : `controller.py` (suivi de point), `route.py` (boucle d'arbres), nœud `waypoint_follower`,
  `gps.py` (ENU ↔ géodésique, modèle d'erreur GPS), `localization.py` (EKF map → odom, dead reckoning,
  `map_shift`), nœuds `gps_noise` et `localizer`
- `src/applepic_perception` : `detector.py` (HSV), `trunk.py` (pied de tronc), `projection.py` (pixel→sol), nœud `apple_detector`
- `src/applepic_mission` : `basket.py`, `picking.py`, `apple_tracker.py` (fusion des détections), `approach.py`
  (préhenseur → point), nœuds `gripper` (ramassage magique : `/basket/count|full|picked|picked_point`,
  srv `/basket/unload`), `apple_picker` (démo M5), `coverage.py` (empreinte caméra au sol + grille de couverture par arbre),
  `mission.py` (machine à états pure) et nœud `mission`
  (`/mission/state|status|stats`), `viz.py` + nœud `mission_viz` (marqueurs RViz de la démo)
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
