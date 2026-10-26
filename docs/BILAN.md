# Bilan (pour l'humain)

## 1. État
- M0 : infrastructure en place (workspace ROS 2 Jazzy natif, Makefile, docs). `make test` passe.
- Le rendu caméra Gazebo headless fonctionne sur cette machine (GPU NVIDIA).
- M1 : monde du verger généré depuis YAML (6 arbres, 29 pommes, zone de dépôt) ; vérifié en
  headless (SDF valide, pommes stables, capture vue de dessus dans `artifacts/orchard_overhead.png`).
- M2 : robot (base diff 60x45 cm, caméra vers le sol, panier, préhenseur) qui roule sur `/cmd_vel`
  dans le verger ; vérifié par `scripts/check_drive.sh` (1,46 m parcourus) et captures caméra où
  les pommes sont visibles (`artifacts/robot_camera_tree0.png`).
- M3 : le robot fait seul la boucle des 6 pommiers puis revient au dépôt (contrôleur maison, pose
  vérité terrain) ; vérifié en headless par `scripts/check_navigation.sh` (40,5 m, 102 s simulées,
  jamais à moins de 2 m d'un tronc ; tracé `artifacts/nav_trajectory.png`).
- M4 : détection des pommes par couleur dans l'image caméra et position au sol en repère monde ;
  vérifié en headless contre les positions réelles (`scripts/check_perception.sh`) : erreur ~5 mm à
  l'arrêt, ≤ 1,5 cm en rotation, aucun faux positif (images annotées `artifacts/perception_*.png`).
- M5 : le robot va chercher les pommes qu'il voit et les ramasse (« magiquement » : la pomme disparaît
  du monde quand le préhenseur passe dessus), compteur et capacité du panier, vidage par service ;
  vérifié en headless par `scripts/check_picking.sh` (arbre 0 : 2/2 pommes visibles, arbre 4 : 4/4,
  modèles bien supprimés de Gazebo, panier plein → arrêt, vidage → reprise).
- M6 : **la mission complète tourne seule** : pour chaque pommier, le robot fait le tour (8 points
  à 2,2 m, face au tronc), ramasse les pommes vues, passe à l'arbre suivant ; panier plein → dépôt,
  vidage, retour à l'arbre ; fin de boucle → dépôt. Vérifié en headless par `scripts/check_mission.sh` :
  29/29 pommes ramassées et déposées en ~15 min simulées, 208 m.
- M7 : **test de bout en bout chronométré** `make e2e` (headless, simulation accélérée ×3, ~5 min
  réelles) : PASS si ≥ 80 % des pommes déposées en ≤ 1200 s simulées, recoupé avec Gazebo. Dernier
  run : 29/29 déposées, 888 s simulées, 208 m (`artifacts/e2e_metrics.json`, `artifacts/e2e_trajectory.png`).

## 2. Lancer
```bash
cd ~/Documents/ApplePic
make build && make test
make world          # régénère le verger (config: src/applepic_gazebo/config/orchard.yaml)
gz sim src/applepic_gazebo/worlds/orchard.sdf   # voir le verger seul (GUI)
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch applepic_bringup sim.launch.py headless:=false   # verger + robot (GUI)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"   # le faire avancer
scripts/check_drive.sh   # test automatique headless de la conduite
ros2 launch applepic_bringup navigation.launch.py headless:=false loops:=1   # boucle des pommiers (GUI)
scripts/check_navigation.sh   # test automatique headless de la boucle (+ tracé PNG)
ros2 launch applepic_bringup perception.launch.py headless:=false x:=2.6 y:=4.0   # détection
ros2 run rqt_image_view rqt_image_view /apples/debug_image   # image annotée (autre terminal)
scripts/check_perception.sh   # test automatique headless de la détection
ros2 launch applepic_bringup picking.launch.py headless:=false x:=9.6 y:=9.5 yaw:=1.57   # ramassage
ros2 topic echo /basket/count   # compteur du panier (autre terminal)
scripts/check_picking.sh   # test automatique headless du ramassage
ros2 launch applepic_bringup mission.launch.py headless:=false   # MISSION COMPLÈTE (GUI)
ros2 topic echo /mission/status   # transitions de la mission (autre terminal)
make e2e   # test de bout en bout headless chronométré (~5 min) -> PASS/FAIL + métriques
```
(la démo avec RViz arrivera au jalon M8)

## 3. Décisions à relire
- D1 : environnement natif plutôt que Docker (voir `docs/DECISIONS.md`).
- D3 : monde généré + commité, pommes dynamiques, dépôt sans collision.
- D4 : `/odom` = pose vérité terrain en repère monde ; odométrie roues à part sur `/wheel_odom`.
- D5 : navigation par contrôleur maison (pas Nav2) ; le robot passe à 2,2 m des troncs côté
  intérieur de la boucle pour ne pas rouler sur les pommes.
- D6 : perception HSV + projection sur sol plat ; publication brute des pommes vues (pas de suivi).
- D7 : ramassage = suppression du modèle de pomme quand le préhenseur est à < 7 cm (pose vérité
  terrain des pommes, comme un capteur de contact) ; ramassage automatique même en transit.
- D8 : machine à états maison (pas de BehaviorTree) ; recherche en cercle de 8 points autour de
  chaque arbre ; le robot ne vise une pomme que si son trajet reste à ≥ 0,45 m du tronc.
- D9 : test M7 chronométré en temps simulé, simulation accélérée ×3 ; seuil 80 % et limite 1200 s
  (modifiables : `make e2e E2E_MIN_RATIO=0.9 E2E_SIM_LIMIT=1000`).

## 4. Cassé / bloqué
- Rien pour l'instant. Voir `docs/TODO_HUMAIN.md` (optionnel).

## 5. Prochaines étapes
- M8 démo GUI Gazebo + RViz avec instructions pas à pas.
- Accélérer la mission (une boucle ≈ 15 min : beaucoup de temps passé à regarder des zones vides).
