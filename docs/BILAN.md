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
```
(la démo complète arrivera au jalon M8)

## 3. Décisions à relire
- D1 : environnement natif plutôt que Docker (voir `docs/DECISIONS.md`).
- D3 : monde généré + commité, pommes dynamiques, dépôt sans collision.
- D4 : `/odom` = pose vérité terrain en repère monde ; odométrie roues à part sur `/wheel_odom`.
- D5 : navigation par contrôleur maison (pas Nav2) ; le robot passe à 2,2 m des troncs côté
  intérieur de la boucle pour ne pas rouler sur les pommes.
- D6 : perception HSV + projection sur sol plat ; publication brute des pommes vues (pas de suivi).
- D7 : ramassage = suppression du modèle de pomme quand le préhenseur est à < 7 cm (pose vérité
  terrain des pommes, comme un capteur de contact) ; ramassage automatique même en transit.

## 4. Cassé / bloqué
- Rien pour l'instant. Voir `docs/TODO_HUMAIN.md` (optionnel).

## 5. Prochaines étapes
- M6 machine à états de mission (aller à l'arbre, chercher autour, ramasser, arbre suivant, dépôt),
  puis M7 test de bout en bout chronométré et M8 démo avec GUI.
