# Bilan (pour l'humain)

## 1. État
- M0 : infrastructure en place (workspace ROS 2 Jazzy natif, Makefile, docs). `make test` passe.
- Le rendu caméra Gazebo headless fonctionne sur cette machine (GPU NVIDIA).
- M1 : monde du verger généré depuis YAML (6 arbres, 29 pommes, zone de dépôt) ; vérifié en
  headless (SDF valide, pommes stables, capture vue de dessus dans `artifacts/orchard_overhead.png`).
- M2 : robot (base diff 60x45 cm, caméra vers le sol, panier, préhenseur) qui roule sur `/cmd_vel`
  dans le verger ; vérifié par `scripts/check_drive.sh` (1,46 m parcourus) et captures caméra où
  les pommes sont visibles (`artifacts/robot_camera_tree0.png`).

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
```
(la démo complète arrivera au jalon M8)

## 3. Décisions à relire
- D1 : environnement natif plutôt que Docker (voir `docs/DECISIONS.md`).
- D3 : monde généré + commité, pommes dynamiques, dépôt sans collision.
- D4 : `/odom` = pose vérité terrain en repère monde ; odométrie roues à part sur `/wheel_odom`.

## 4. Cassé / bloqué
- Rien pour l'instant. Voir `docs/TODO_HUMAIN.md` (optionnel).

## 5. Prochaines étapes
- M3 navigation (contrôleur de waypoints maison, Nav2 non installé), puis M4 perception HSV.
