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

## D6 — Perception HSV + projection sur plan (session 5)
- Contexte : M4, pommes rouges unies sur herbe verte, sol plat, pose caméra connue par TF (vérité terrain).
- Décision : segmentation HSV (deux bandes de teinte autour du rouge, seuils S/V qui excluent le tronc
  marron, teinte ≈ 18) + centroïde des blobs ; projection du centroïde sur le plan z = rayon de pomme
  (0,04 m), ce qui approxime le centre de la sphère (erreur mesurée ≤ 1,5 cm). Blobs touchant le bord
  de l'image ignorés (centroïde biaisé), portée max 4 m. Sortie en `geometry_msgs/PoseArray` (frame
  `odom`), sans identité ni suivi : la fusion temporelle se fera côté M5/M6.
- Écarté : détection de cercles (Hough, fragile avec l'ombrage), estimation de distance par le rayon
  apparent (moins précise que la projection sur sol plat), message custom (PoseArray suffit, évite un
  paquet ament_cmake d'interfaces).

## D7 — Ramassage magique par suppression d'entité, pose vérité terrain des pommes (session 6)
- Contexte : M5, ramassage « magique » autorisé ; il faut savoir quand le préhenseur touche une pomme.
- Décision : nœud `gripper` qui compare la position du préhenseur (`/odom` + 0,46 m dans l'axe) aux
  poses **vérité terrain** des pommes lues sur le topic gz `/world/orchard/pose/info` ; sous 7 cm
  (horizontal), il supprime le modèle via le service gz `/world/orchard/remove` (bindings Python
  `gz.transport13` / `gz.msgs10` livrés avec Harmonic, pas de bridge à configurer) et incrémente le
  panier. Ramassage automatique dès qu'une pomme est à portée (même en transit), refusé si panier plein ;
  `/basket/unload` (std_srvs/Trigger) vide le panier, la mission décidera où l'appeler.
  Le préhenseur dépasse de 6 cm l'avant du châssis : une pomme centrée est ramassée avant d'être touchée.
- Côté décision : `apple_picker` fusionne les détections (`AppleTracker`) et approche avec
  `approach.py` (robot face à la pomme à 0,46 m), finissant en aveugle sur la position mémorisée quand
  la pomme sort du champ caméra.
- Écarté : DetachableJoint (il faut une articulation par pomme déclarée dans le SDF, et une pomme
  attachée resterait visible devant la caméra), position estimée par la caméra pour déclencher
  (la vérité terrain simule un capteur de contact du préhenseur), `ros_gz_sim remove`/`delete_entity`
  en sous-processus (lent, et `remove` boucle s'il ne trouve pas de monde).

## D8 — Machine à états de mission pure + motif de recherche en cercle (session 7)
- Contexte : M6, enchaîner arbres, recherche, ramassage, dépôt et vidage ; transitions testables sans ROS.
- Décision : classe pure `applepic_mission.mission.Mission` (états GOTO_TREE, SEARCH, PICK, GOTO_BASE,
  UNLOAD, DONE) qui reçoit des événements (`on_detections`, `on_picked`, `on_basket_full`,
  `on_unloaded`) et renvoie la commande vitesse à chaque `step(pose, t)` ; le nœud `mission` n'est
  qu'un adaptateur ROS (il remplace `apple_picker`, gardé pour la démo M5).
  - SEARCH : 8 points sur un cercle de 2,2 m autour du tronc (départ au plus proche du robot), à
    chacun le robot se tourne vers le tronc et regarde 1,5 s ; le champ caméra (0,4–2,3 m devant)
    couvre alors l'anneau de pommes (0,45–1,6 m) côté robot. Déplacements entre points en gardant
    1,9 m du tronc (le châssis ne roule pas sur l'anneau).
  - PICK : pomme confirmée (`AppleTracker`, 2 vues) à < 1,9 m du tronc courant, la plus proche, et
    seulement si l'approche en ligne droite garde la base à ≥ 0,45 m de l'axe du tronc (arrêt à
    ≥ 0,5 m) ; sinon elle sera prise depuis un autre point du cercle. Après chaque ramassage, le
    robot revient au point de recherche courant et regarde à nouveau.
  - Un ramassage signalé sur `/basket/picked` est associé à la piste la plus proche du préhenseur
    (gère les pommes ramassées « en passant »).
  - Panier plein → GOTO_BASE puis retour au même arbre (recherche recommencée) ; fin de boucle →
    GOTO_BASE → UNLOAD → nouvelle boucle ou DONE (`loops`, 0 = infini).
- Écarté : Nav2/BehaviorTree (non installé, surdimensionné), spirale ou rotation sur place depuis
  l'arbre (plus de risque d'écraser les pommes), SMACH/YASMIN (dépendance de plus pour 6 états).
