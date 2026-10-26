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

## Session 6 (2026-10-26)
- M5 fait (dans `applepic_mission`) : modules purs `basket.py` (capacité), `picking.py` (position du
  préhenseur, pomme à portée), `apple_tracker.py` (fusion des détections : association gloutonne
  < 0,15 m, moyenne glissante, confirmation à 2 vues, pistes conservées hors champ, états
  picked/failed), `approach.py` (amène le préhenseur, 0,46 m devant la base, sur un point ; recule
  si dépassement). Nœuds `gripper` (ramassage magique : pose vérité terrain des pommes via le topic gz
  `/world/orchard/pose/info`, suppression via le service gz `/world/orchard/remove` avec les bindings
  Python gz-transport ; `/basket/count`, `/basket/full`, `/basket/picked`, service `/basket/unload`)
  et `apple_picker` (va vers la pomme confirmée la plus proche, abandon après 30 s d'approche ou 3 s
  sur place sans ramassage). Launch `applepic_bringup/launch/picking.launch.py` (args `x`, `y`, `yaw`,
  `capacity`, `headless`).
  - Vérifié : `scripts/check_picking.sh` (+ `scripts/eval_picking.py` qui recoupe `/basket/picked`,
    `/basket/count` et `gz model --list`) : arbre 0 depuis (2,6 ; 4,0) → 2/2 pommes visibles ramassées
    en 16 s ; arbre 4 depuis (9,6 ; 9,5 ; π/2) → 4/4 en 25 s ; modèles supprimés de gz, compteur
    cohérent. Capacité 1 : le picker s'arrête (« basket full ») ; appel manuel de `/basket/unload` →
    compteur à 0 et reprise du ramassage (2ᵉ pomme ramassée).
  - `make test` : 53 tests, 0 échec (15 tests mission dont convergence de l'approche en simulation unicycle).
- Limites : le picker ne cherche pas (il ne ramasse que les pommes déjà vues) ; le châssis (bas à
  5 cm, pomme haute de 8 cm) peut pousser des pommes situées sur les côtés du chemin ; ramassage
  automatique dès qu'une pomme passe sous le préhenseur, même en transit (voulu).

## Session 7 (2026-10-26)
- M6 fait : machine à états pure `applepic_mission/mission.py` (GOTO_TREE → SEARCH → PICK → SEARCH …
  → arbre suivant ; fin de boucle → GOTO_BASE → UNLOAD → nouvelle boucle / DONE ; panier plein →
  GOTO_BASE → UNLOAD → retour au même arbre). SEARCH = 8 points sur un cercle de 2,2 m autour du tronc,
  face au tronc, 1,5 s d'observation ; PICK = pomme confirmée la plus proche à < 1,9 m du tronc dont
  l'approche droite reste à ≥ 0,45 m du tronc. Nœud `mission` (remplace `apple_picker`, réutilise
  `gripper`) : `/mission/state`, `/mission/status`, `/mission/stats` (JSON : ramassées, déposées,
  échecs, distance, durée), param `trajectory_csv`, `exit_when_done`. Launch
  `applepic_bringup/launch/mission.launch.py` (args `headless`, `x`, `y`, `yaw`, `capacity`, `loops`,
  `search_radius`, `trajectory_csv`). `route.plan_path` (détours) exposé pour la mission.
  - Vérifié : `scripts/check_mission.sh` (+ `scripts/eval_mission.py`) → une boucle complète headless
    depuis le dépôt : **29/29 pommes ramassées et déposées**, 0 échec, 2 retours « panier plein »
    (capacité 10) + dépôt final, état DONE, 0 modèle `apple_*` restant dans gz ; 888 s simulées
    (~900 s réelles), 208 m parcourus, distance min. aux troncs 0,75 m. Tracé
    `artifacts/mission_trajectory.png`, métriques `artifacts/mission_metrics.json`.
  - `make test` : 65 tests, 0 échec (12 tests mission : transitions unitaires + 3 simulations
    unicycle complètes avec faux détecteur/préhenseur, dont le verger généré).
- Limites : une boucle prend ~15 min (8 points par arbre même sans pomme restante) ; pas d'évitement
  d'obstacles autre que les troncs connus ; un double SIGINT à l'arrêt affiche une trace
  KeyboardInterrupt bénigne du nœud `mission`.

## Session 8 (2026-10-26)
- M7 fait (travail entamé par la session précédente, interrompue par la limite d'usage avant commit ;
  repris, vérifié, commité) : `scripts/e2e_mission.sh` (cible `make e2e`, variables `E2E_MIN_RATIO`,
  `E2E_SIM_LIMIT`, `E2E_RTF`) remplace `check_mission.sh` ; RTF réglé à 3 via le service gz
  `/world/orchard/set_physics` ; `scripts/eval_e2e.py` chronomètre en temps simulé (`/clock`, depuis la
  1ʳᵉ transition) et note le run avec le module pur `applepic_mission/metrics.py` (PASS/FAIL + détails
  par arbre) ; `scripts/plot_e2e.py` trace trajectoire + pommes ramassées/restantes.
  - Vérifié : `make e2e` → **PASS** : 29/29 pommes déposées (seuil 80 %), 0 échec, mission DONE en
    887,7 s simulées (limite 1200 s) pour 302 s réelles (RTF effectif 2,94), 207,6 m, 0 pomme restante
    dans gz, compteurs cohérents. `artifacts/e2e_metrics.json`, `artifacts/e2e_trajectory.png`.
    Aucun processus orphelin après le run.
  - `make test` : 69 tests, 0 échec (4 tests `metrics`).
- Limites : une seule configuration (seed 42) testée ; la mission reste lente en temps simulé (~15 min).

## Session 9 (2026-10-26)
- M8 fait (code écrit par la session précédente, interrompue avant commit ; repris, vérifié,
  commité) : `applepic_bringup/launch/demo.launch.py` (mission + GUI Gazebo + RViz, args `headless`,
  `rviz`, `rtf`, `x`, `y`, `yaw`, `capacity`, `loops`, `search_radius`), config `rviz/demo.rviz`
  (marqueurs, trajectoire, RobotModel, pommes détectées, image caméra annotée), module pur
  `applepic_mission/viz.py` + nœud `mission_viz` (`/mission_viz/markers` latched, `/mission_viz/path`),
  cible `make demo` (`DEMO_RTF`), section « Démo pas à pas » du README.
  - Ménage : la session précédente avait laissé **deux démos orphelines** tournant en parallèle
    (même domaine ROS → `/clock` incohérent, « jump back in time » en boucle, captures inutilisables) ;
    arrêtées par PGID.
  - Vérifié : `scripts/check_demo.sh` (+ `scripts/eval_demo.py`) → PASS : 26 marqueurs de scène + HUD,
    trajectoire qui croît, 1ʳᵉ pomme ramassée, RTF 2 appliqué, aucune erreur dans le log.
  - Vérifié GUI : `make demo`-équivalent sur DISPLAY :1 pendant 90 s → fenêtres « Gazebo Sim » et RViz
    ouvertes, aucun saut de temps, 4 pommes ramassées ; captures `artifacts/demo_{rviz,gazebo}.png`
    (HUD « SEARCH / basket 4/10 », cercles, dépôt, trajectoire visibles ; couleurs faussées par mon
    décodeur XWD, pas par la démo). Arrêt par PGID, aucun processus restant.
  - `make test` : 73 tests, 0 échec (4 tests `viz`).
- Non vérifié : démo GUI avec `rtf:=2` ; le texte d'état ne rafraîchit distance/temps qu'aux transitions.
- Leçon : enregistrer `$!` avec `set -m` exécuté AVANT la ligne backgroundée (sinon mauvais PGID,
  cf. CLAUDE.md) — je me suis encore fait avoir une fois, rattrapé via `ps -o pgid`.

## Session 10 (2026-10-26)
- M9 (1ʳᵉ amélioration) fait : recherche guidée par la couverture caméra (D11). Module pur
  `applepic_mission/coverage.py` (empreinte caméra au sol depuis la géométrie URDF, grille de la
  couronne 0,4–1,7 m par arbre, occultation par le tronc) ; `mission.py` : points déjà vus sautés,
  arbre quitté à 98 % couvert sauf piste active restante, ≤ 3 vues de complément ; cercle 2,0 m,
  regard 1,0 s, rotation 1,2 rad/s (défauts des launch passés à 2,0).
  - Constat : l'empreinte réelle est 0,44–1,69 m devant la base (pas 2,3 m) ; vérifié sur les
    captures M4 (pommes vues/non vues cohérentes avec le calcul).
  - Réglage par balayage dans un simulateur unicycle (verger généré + 8 vergers aléatoires, 304
    pommes) : 871 → 645 s sur le verger généré, 0 pomme perdue.
  - Bug corrigé dans `gripper` : sous RTF 3, la requête gz `remove` expirait (1 s) alors que gz
    supprimait la pomme → 2 pommes perdues au 1ᵉʳ run (27/29). Délai 3 s + confirmation par le flux
    de poses. Ce chemin de confirmation n'a pas été déclenché lors des runs suivants (non exercé).
  - Vérifié : `make e2e` ×2 → **PASS 29/29, 656 s et 660 s simulées** (avant : 888 s), 176–178 m
    (avant : 208 m), 0 échec ; `scripts/check_demo.sh` PASS ; aucun processus orphelin.
  - `make test` : 85 tests, 0 échec (6 tests `coverage`, 6 tests mission couverture dont
    comparaison de vitesse sur 3 vergers ; le faux détecteur des tests utilise l'empreinte réelle).
- Non atteint : objectif < 600 s (les seuils qui y arrivaient perdaient des pommes) ; restent
  GOTO_TREE + GOTO_BASE ≈ 240 s et les retours au point de recherche après chaque ramassage.

## Session 11 (2026-10-26)
- HUD continu : `/mission/stats` republié toutes les `stats_period` s (1 s simulée ; mesuré 3,0 Hz réels
  à RTF 3) ; `scripts/check_demo.sh` PASS, HUD « 5 m 14 s » avant la 1ʳᵉ transition.
- `make e2e E2E_SEED=<n>` : monde régénéré dans `artifacts/orchard_seed<n>.sdf` (+ manifeste),
  argument `world` dans `perception/mission.launch.py` ; `eval_e2e.py` enregistre le seed et les
  poses finales gz des pommes restées au sol ; la mission journalise les abandons de piste et, au départ
  de chaque arbre, couverture + pistes en attente.
- Robustesse multi-seeds (D12) : premier passage sur les seeds 7/123/2024 → PASS mais 1 à 3 pommes
  perdues par verger. Quatre causes trouvées et corrigées, chacune par diagnostic sur les journaux :
  1. zones jamais vues (pied du tronc, bord de couronne) → cercle 1,9 m, couverture 99,5 %, vues de
     complément sur 3 rayons × 5 caps (balayage unicycle 20 vergers / 640 pommes avec détecteur strict :
     7 perdues → 0, +13 % de temps) ;
  2. pommes poussées par le châssis (bas à 5 cm < 8 cm) → garde au sol 10 cm ;
  3. pommes éjectées à 60–160 m : la roulette roulait dessus quand le robot pivotait vers une pomme
     passée sous le châssis → l'approche s'éloigne en ligne droite avant de pivoter ;
  4. préhenseur : `GzNode.request` (Python) garde le GIL → nœud gelé jusqu'à 3 s par ramassage,
     confirmations tardives attribuées à la mauvaise piste → `gz service` en sous-processus dans un
     thread, comptage par le flux de poses, nouveau topic `/basket/picked_point` utilisé par la mission.
  - Vérifié (code final) : `make e2e` seeds 123 / 42 / 2024 / 7 → **PASS 100 % partout, 128/128
    pommes, 0 abandon** ; 870 / 725 / 864 / 804 s simulées (limite 1200 s), 188–239 m.
    `scripts/check_picking.sh` OK, `scripts/check_demo.sh` PASS, aucun processus orphelin.
  - `make test` : 97 tests, 0 échec (dont détecteur strict sur 4 vergers générés, approche sans
    toucher la roulette, rapport de ramassage tardif).
- Coût : mission plus lente sur le verger par défaut (725 s contre 656 s) : on vise tout le sol.
- Non vérifié : démo GUI après ces changements (seulement `check_demo.sh` headless).

## Prochaine tâche : M9 — GPS simulé bruité ou obstacles
- GPS : capteur navsat dans l'URDF + `/gps/fix` ; robot_localization non installé → écrire la commande
  dans TODO_HUMAIN et faire en attendant un petit filtre maison (EKF 2D odom roues + GPS) testable en pur
  Python ; mesurer l'erreur de pose et son effet sur le ramassage (`make e2e`).
- Option : réduire le temps de mission (retours au point de recherche après chaque ramassage).
