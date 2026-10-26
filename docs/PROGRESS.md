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

## Session 3 (2026-10-26)
- M2 fait : `applepic_description/urdf/applepic.urdf.xacro` (base 0,60 x 0,45 m, roues r=0,10 m,
  écart 0,52 m, roulette avant sans frottement, caméra à 0,50 m inclinée de 0,85 rad, 640x480 @ 10 Hz,
  panier visuel, `gripper_link` à x=0,46 m / z=0,04 m de `base_footprint`). Plugins gz : DiffDrive
  (`/cmd_vel`, odom roues sur `/wheel_odom`), OdometryPublisher (vérité terrain `/odom` + `/tf`),
  JointStatePublisher, caméra. Bridge `applepic_gazebo/config/bridge.yaml`, launch
  `applepic_bringup/launch/sim.launch.py` (args `headless`, `world`, `x`, `y`, `yaw`).
  - Vérifié : `scripts/check_drive.sh` → 3 s à 0,5 m/s, `/odom` x : 0 → 1,46 m ; images caméra
    `artifacts/robot_camera_{start,end}.png` (sol + préhenseur visibles).
  - Vérifié : spawn en (2,6 ; 4,0) → `/odom` = (2,6 ; 4,0) : **`/odom` est en repère monde** (frame `odom`
    ≡ monde). TF `odom → camera_optical_frame` complète ; image `frame_id` = `camera_optical_frame`.
    Capture `artifacts/robot_camera_tree0.png` : tronc + 2 pommes rouges bien visibles.
  - `make test` : 17 tests, 0 échec (5 tests URDF). `make sim-headless` s'arrête sans processus orphelin.
- `check_drive.sh` : suppression du `pkill -f` (règle 7) → arrêt par groupe de processus (`set -m`, PGID = `$!`).
- Avertissement bénin : `gz_frame_id` « not defined in SDF » au chargement, mais le frame_id est bien appliqué.

## Session 4 (2026-10-26)
- M3 fait : contrôleur de point `applepic_navigation/controller.py`, construction de route
  `route.py` (points d'approche à 2,2 m des troncs côté intérieur + détours automatiques), nœud
  `waypoint_follower` (params `loops` (0 = infini), `dwell`, `trajectory_csv`, statut sur
  `/waypoint_follower/status`), launch `applepic_bringup/launch/navigation.launch.py` (simu + suivi).
  - Vérifié : `scripts/check_navigation.sh` → boucle complète en simu headless : les 6 arbres puis
    le dépôt atteints (erreur finale ≤ 0,15 m), 40,5 m parcourus en 102 s simulées, distance minimale
    aux troncs 2,09 m ; tracé `artifacts/nav_trajectory.png`. `navigation.launch.py` vérifié aussi
    (démarre et atteint les premiers points), arrêt par PGID sans processus orphelin.
  - `make test` : 24 tests, 0 échec (7 tests navigation dont une simulation unicycle de la boucle).
- Limite connue : la route d'entrée depuis le dépôt fait un petit zigzag (2 détours autour de l'arbre 0) ;
  sans gravité, à affiner si besoin.

## Session 5 (2026-10-26)
- M4 fait : `applepic_perception/detector.py` (HSV rouge 0–10 ∪ 170–180, S ≥ 100, V ≥ 50, ouverture
  morphologique, filtres aire/remplissage, rejet des blobs coupés par le bord de l'image),
  `projection.py` (rayon pixel → plan z = 0,04 m via K et la TF `odom → camera_optical_frame`),
  nœud `apple_detector` (`/apples/detections` PoseArray en `odom`, `/apples/debug_image` annotée),
  launch `applepic_bringup/launch/perception.launch.py` (args `headless`, `x`, `y`, `yaw`).
  - Vérifié : `scripts/check_perception.sh` (sim headless + `scripts/eval_detections.py` qui apparie
    les détections au manifeste) : arbre 0 depuis (2,6 ; 4,0) → 2 pommes, erreur moyenne 4 mm ;
    arbre 4 depuis (9,6 ; 9,5 ; yaw π/2) → 4 pommes (dont 2 à 18 cm l'une de l'autre), erreur 5 mm ;
    0 faux positif (tronc marron et préhenseur gris ignorés). Robot en rotation à 0,3 rad/s :
    erreur moyenne 5 mm, max 1,5 cm. Images annotées `artifacts/perception_{tree0,tree4,moving}.png`.
  - `make test` : 39 tests, 0 échec (16 tests perception : images synthétiques, projection aller-retour,
    rendu synthétique → détection → projection < 6 cm).
- Limites : champ utile ~0,4–2,3 m devant le robot ; pas de suivi/fusion temporelle des détections
  (chaque image republie toutes les pommes vues) ; pommes au contact fusionneraient en un blob.

## Prochaine tâche : M5 — Ramassage magique
- Module pur `applepic_mission/apple_tracker.py` (ou dans perception) : fusion des détections
  successives en une liste de pommes connues en repère monde (association au plus proche < 0,15 m,
  moyenne glissante, compteur de vues ≥ 2 pour confirmer, marquage « ramassée »). Tests pytest.
- Approche : réutiliser `applepic_navigation.controller.compute_command` pour amener `gripper_link`
  (0,46 m devant `base_footprint`) sur la pomme : viser un point tel que le préhenseur coïncide avec
  la pomme (cible base = pomme − 0,46 m dans l'axe d'approche). Attention : la pomme sort du champ
  caméra sous ~0,4 m → finir l'approche en boucle ouverte sur la position mémorisée.
- Ramassage : nœud `gripper` qui, quand la distance préhenseur–pomme (pose vérité terrain des pommes ou
  position estimée) < ~6 cm, supprime l'entité via le service gz `/world/orchard/remove`
  (plugin UserCommands déjà dans le monde, D3 ; appel par `gz service` ou ros_gz `RemoveEntity` si
  disponible dans Jazzy), incrémente le compteur, publie `/basket/count`, refuse au-delà de la capacité.
- Vérif : script headless qui spawne le robot près de l'arbre 0, ramasse ≥ 1 pomme, vérifie que le
  modèle `apple_t0_*` a disparu (`gz model --list`) et que le compteur vaut 1.
