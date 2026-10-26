# Décisions

## D1 — Environnement d'exécution natif plutôt que Docker (session 1)
- Contexte : Docker est utilisable sans sudo, mais ROS 2 Jazzy + gz Harmonic + ros_gz sont déjà
  installés nativement ; il ne reste que ~13 Go libres sur le disque ; Docker n'a pas le runtime
  NVIDIA, donc le rendu caméra en conteneur serait logiciel (lent/fragile). En natif, le rendu
  headless GPU marche (vérifié).
- Décision : workflow natif par défaut via `Makefile`. Un `docker/Dockerfile` léger
  (ros-base + ros_gz) reste disponible (`USE_DOCKER=1`), non utilisé pour la vérification.
- Écarté : `osrf/ros:jazzy-desktop-full` (taille disque, pas de GPU).

## D2 — Tous les paquets en ament_python
- Contexte : rien à compiler en C++ pour le POC.
- Décision : `ament_python` partout, logique pure séparée des nœuds rclpy pour être testée avec pytest.
- Écarté : ament_cmake pour description/gazebo (inutile, data_files suffit).

## D3 — Monde du verger généré et versionné (session 2)
- Contexte : M1 demande un monde paramétrable et reproductible.
- Décision : `applepic_gazebo/orchard.py` (Python pur) génère `worlds/orchard.sdf` + un manifeste
  vérité terrain `worlds/orchard_apples.yaml` depuis `config/orchard.yaml` (seed fixe). Les deux
  fichiers générés sont commités (`make world` pour régénérer). 6 arbres en boucle, 4 à 7 pommes
  par arbre dans un anneau 0,45–1,6 m, zone de dépôt bleue (visuel seul, sans collision) en (0,0).
- Pommes = sphères dynamiques (r = 4 cm, 150 g) : stables au repos sur sol plan (vérifié), le robot
  pourra les pousser. Plugins monde déjà inclus : UserCommands (suppression d'entité pour le
  ramassage magique), Sensors ogre2, Imu, NavSat + `spherical_coordinates` (pour le GPS du M9).
- Écarté : pommes statiques (irréaliste si le robot les touche), modèles Fuel (téléchargement réseau,
  inutile pour HSV), herbe texturée (le vert uni suffit à la segmentation).

## D4 — Robot et odométrie (session 3)
- Contexte : M2, navigation en coordonnées monde d'abord.
- Décision : `/odom` = OdometryPublisher gz (pose vérité terrain, repère `odom` ≡ monde, vérifié avec un
  spawn hors origine) ; l'odométrie des roues du DiffDrive est publiée à part sur `/wheel_odom` (TF non
  bridgée) pour le futur robot_localization. Caméra montée à 0,50 m, tilt 0,85 rad (~49°), HFOV 1,3 rad :
  voit le sol d'environ 0,2 à 2,5 m devant ; le préhenseur apparaît en bas de l'image.
  Préhenseur = simple frame `gripper_link` (le ramassage magique M5 se basera sur sa distance aux pommes).
- Écarté : roues + gz_ros2_control (inutile pour un POC), panier physique avec collisions.

## D5 — Navigation par contrôleur maison (session 4)
- Contexte : M3, Nav2 non installé, monde plat sans obstacles autres que les troncs, pose vérité terrain.
- Décision : `applepic_navigation/controller.py` (rotation sur place si erreur de cap > 0,6 rad, puis
  avance proportionnelle à la distance, plafonnée à 0,5 m/s ; tolérance 0,15 m) + `route.py` qui
  construit la boucle : un point d'approche par arbre à 2,2 m du tronc **côté intérieur de la boucle**
  (vers le centroïde des arbres, donc hors de l'anneau de pommes ≤ 1,6 m), puis retour au dépôt ; tout
  segment passant à moins de 2,0 m d'un tronc reçoit un point de détour (`via`). Le nœud
  `waypoint_follower` lit les arbres dans `applepic_gazebo/config/orchard.yaml`.
- Écarté : Nav2 (non installé, lourd pour une boucle de 6 points sur sol plat), pure pursuit
  (inutile à cette échelle), carte d'occupation.
- Conséquence pour M6 : la mission réutilisera `compute_command` pour aller aux pommes et aux points
  de recherche ; le nœud `waypoint_follower` sert de démo/validation M3.
