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

## D9 — Scénario de bout en bout M7 accéléré par RTF (session 8)
- Contexte : une boucle de mission dure ~15 min simulées ; il faut un test M7 chronométré,
  reproductible et rapide à relancer.
- Décision : `scripts/e2e_mission.sh` (cible `make e2e`) lance la mission headless (1 boucle, départ
  au dépôt) et règle le facteur temps réel à 3 via le service gz `/world/orchard/set_physics`
  (le rendu caméra plafonne vers ~4 sur cette machine). Le chronométrage est en **temps simulé**
  (depuis la première transition de la mission), limite 1200 s, seuil de pommes déposées 80 %.
  Score calculé par le module pur `applepic_mission/metrics.py` (testé par pytest) : PASS si mission
  DONE, ratio atteint, dans la limite, gz interrogé, pommes signalées ramassées absentes de gz et
  compteurs cohérents. Sorties `artifacts/e2e_metrics.json` + tracé `e2e_trajectory.png`.
  Remplace `check_mission.sh` / `eval_mission.py`.
- Écarté : launch_testing (plus lourd à déboguer, pas de gain ici) ; chronométrage en temps réel
  (dépend de la charge machine) ; RTF illimité (`real_time_factor: 0`) : capteurs caméra moins
  réguliers, mission non vérifiée dans ce mode.

## D10 — Démo M8 : launch dédié + nœud de visualisation (session 9)
- Contexte : l'humain doit pouvoir regarder la mission ; RViz n'a rien à afficher du verger
  (pas de carte) et la mission ne publie que des états texte.
- Décision : `demo.launch.py` inclut `mission.launch.py` (GUI Gazebo par défaut) + RViz
  (`rviz/demo.rviz`, repère fixe `odom` ≡ monde) + nœud `mission_viz` (`applepic_mission`) qui publie
  `~/markers` (troncs, couronnes, cercles de recherche, dépôt, étiquettes depuis `orchard.yaml`,
  texte d'état au-dessus du robot) et `~/path` (trajectoire décimée à 5 cm, depuis `/odom`). Logique
  pure dans `viz.py` (testée). Argument `rtf` pour accélérer via `/world/orchard/set_physics`
  (défaut 1 : la GUI suit sans difficulté). Vérification automatique headless :
  `scripts/check_demo.sh` (contenu des topics RViz pendant la mission).
- Écarté : afficher les pommes réelles (vérité terrain) dans RViz (ce serait tricher visuellement
  sur la perception) ; plugin RViz personnalisé pour le HUD (texte 3D suffisant) ;
  enregistrement vidéo automatique (pas d'outil de capture d'écran installé, pas de Xvfb).

## D11 — Recherche guidée par la couverture caméra (session 10)
- Contexte : une boucle prenait 888 s simulées, dont 531 s en SEARCH (8 points fixes par arbre,
  même quand tout était déjà vu). L'empreinte réelle de la caméra au sol, calculée depuis l'URDF,
  est 0,44–1,69 m devant la base (et non 0,4–2,3 m comme supposé en session 7) : 8 vues fixes à
  2,2 m ne couvrent que ~75 % de la couronne où tombent les pommes ; c'est en roulant vers les
  pommes que le reste était vu.
- Décision : module pur `coverage.py` : empreinte = projection des coins de l'image (réduite à
  85 % : marge pour les blobs coupés au bord) sur le plan des pommes ; grille 10 cm de la couronne
  0,4–1,7 m autour de chaque tronc, marquée à chaque tick en SEARCH/PICK (occultation par le tronc
  incluse). La mission saute les points dont la vue ajouterait < 1 % de la couronne, quitte l'arbre
  dès 98 % vus, sauf s'il reste une piste active (même vue une seule fois) près de l'arbre ; une fois
  le cercle fini, jusqu'à 3 vues « de complément » (meilleur gain parmi 24 positions × 3 caps sur le
  cercle, gain ≥ 2 %). Cercle ramené à 2,0 m (le bout de l'empreinte atteint le tronc), regard 1,0 s,
  rotation vers le tronc à 1,2 rad/s. Réglage choisi par balayage dans un simulateur unicycle
  (9 vergers, 304 pommes) puis vérifié dans Gazebo : 656 s au lieu de 888 s, 29/29.
- Écarté : arrêter après N points sans nouvelle pomme (aucune garantie de couverture) ; planificateur
  « next-best-view » complet (plus lent à mettre au point, gain marginal mesuré) ; seuils plus
  agressifs (couverture 90–95 %, gain 3–4 %) : 600–620 s mais 1 à 8 pommes perdues sur 304 ;
  rouler plus près des troncs (risque de pousser les pommes avec le châssis).

## D12 — Robustesse multi-seeds : couverture complète, châssis relevé, pas de pivot sur une pomme (session 11)
- Contexte : `make e2e` sur d'autres vergers (seeds 7, 123, 2024 ; mêmes arbres, autres pommes)
  passait (≥ 80 %) mais laissait 1 à 3 pommes. Deux causes distinctes, établies par les poses
  finales des pommes dans gz et le rejeu de la trajectoire :
  1. **Pommes jamais vues** (restées à leur place) : l'arbre était quitté à 94–98 % de couverture.
     Depuis le cercle à 2,0 m face au tronc, l'empreinte (0,48–1,44 m devant) laisse deux zones
     aveugles : pied du tronc (r < 0,6 m) et bord de la couronne (r ≈ 1,5–1,7 m) ; les vues de
     complément (gain ≥ 2 %) ne les rattrapaient pas.
  2. **Pommes poussées ou éjectées** (retrouvées à 60–160 m) : le bas du châssis était à 5 cm, une
     pomme fait 8 cm ; surtout, en pivotant sur place vers une pomme passée sous le châssis, la
     roulette avant lui roulait dessus et le moteur physique la projetait.
- Décision :
  - Couverture : cercle 1,9 m, objectif 99,5 %, vues de complément jusqu'à 4 avec gain ≥ 0,5 %,
    candidates sur 3 rayons (1,75 / 1,9 / 2,1 m) × 24 positions × 5 caps. Choisi par balayage
    unicycle sur 20 vergers générés (640 pommes) avec un faux détecteur strict (exactement
    l'empreinte planifiée) : 7 pommes perdues → 0, mais +13 % de temps de mission.
  - Robot : garde au sol 10 cm (le châssis passe au-dessus des pommes ; caméra et préhenseur,
    définis par rapport à `base_footprint`, inchangés).
  - Approche : si la pomme est à moins de `offset + 0,1 m` de la base et mal alignée, le robot
    s'en éloigne en ligne droite avant de pivoter (la roulette balaie un cercle de 0,32 m).
  - Préhenseur : la requête gz `remove` via le binding Python gz-transport garde le GIL pendant
    son attente : tout le nœud (flux de poses gz, `/odom`) gelait jusqu'à 3 s par ramassage, la
    réponse expirait souvent (alors que gz supprimait la pomme) et la mission abandonnait la pomme
    suivante. Désormais : `gz service` lancé en sous-processus depuis un thread ; la pomme est
    comptée dès qu'elle disparaît du flux de poses (ou à la réponse positive), relance possible
    après `remove_timeout + confirm_timeout` si elle est toujours là. Le préhenseur publie aussi
    `/basket/picked_point` (position de la pomme ramassée), que la mission utilise pour retirer la
    bonne piste : avec la position courante du préhenseur, une confirmation tardive retirait la
    piste visée suivante (pomme laissée au sol, cas observé avec seed 42).
- Écarté : supprimer les collisions pommes/robot (masquerait un vrai problème de conception) ;
  préhenseur « magique » à plus grande portée (idem) ; garder 98 % (plus rapide, pommes perdues).

## D13 — GPS simulé + localisation maison (EKF sur map → odom), gyroscope pour le cap (session 12)
- Contexte : M9 « GPS simulé bruité + robot_localization ». robot_localization n'est pas installé
  (pas de sudo). Le bruit du capteur gz NavSat s'exprime **en degrés** (σ = 1 → ~100 km, mesuré),
  identique en latitude et longitude (donc anisotrope en mètres), et n'est pas reproductible.
- Décision :
  - Capteur gz `navsat` sans bruit (`/gps/raw`) + nœud `gps_noise` : bruit blanc + biais
    Gauss-Markov (σ, τ), isotrope en mètres, avec seed. Défaut : RTK (σ = 2 cm, sans biais), comme les
    robots tondeuses du commerce.
  - Nœud `localizer` : EKF à 3 états sur la transformation map → odom (et non sur la pose), mise à jour
    par chaque fix GPS (porte de Mahalanobis), prédiction = dérive odométrique (variance ∝ distance,
    angle, temps). Cap initial connu (robot à quai au dépôt, arg `yaw`). Après 20 fixes rejetés de
    suite, redémarrage sur le fix courant.
  - Odométrie : vitesse des roues + **gyroscope** (capteur gz IMU, bruit + biais, seed par
    `gz sim --seed`). Mesuré : les roues sous-estiment de **7 %** les pivots sur place (glissement) ;
    sans gyroscope, le cap dérivait de 0,3 rad en 100 s et le filtre divergeait (10 m, 24/29 pommes).
  - Un seul contrat pour les consommateurs : `/localization/odom` (repère `map`) et TF
    `map → odom → base_footprint`, quel que soit le mode (`localization:=truth` publie l'identité et
    republie la vérité terrain). Le préhenseur « magique » garde la vérité terrain (c'est la physique).
- Écarté : robot_localization (non installé ; à reconsidérer si l'humain l'installe, cf. TODO_HUMAIN) ;
  bruit gz en degrés (anisotrope, non reproductible) ; filtre sur la pose complète (même résultat,
  mais l'offset map → odom donne directement la TF standard REP-105) ; magnétomètre (le gyroscope suffit
  avec les corrections GPS).

## D14 — Pommes ancrées au repère odométrique (session 12)
- Contexte : avec un GPS biaisé (σ 0,3 m + biais 0,3 m), l'estimation map → odom bouge pendant
  l'approche d'une pomme : la pomme redétectée tombait hors de la porte d'association (0,15 m) →
  pistes fantômes, 8 abandons, 27/29.
- Décision : le nœud mission applique chaque changement de map → odom à toutes les pistes (et aux
  comptes rendus de ramassage tardifs), via `localization.map_shift`. La géométrie robot ↔ pomme est
  alors purement odométrique, insensible au GPS ; l'arbre, le cercle de recherche et la couverture
  restent en repère map. Le préhenseur publie le point ramassé relativement au robot
  (`base_footprint`, horodaté au déclenchement) et la mission le replace avec sa propre estimation.
- Résultat : même scénario → 29/29, 0 abandon.
- Écarté : tout faire en repère odom (arbres et couverture en dériveraient) ; élargir la porte
  d'association (fusionnerait des pommes voisines).
- Limite mesurée : avec un GPS « grand public » (σ 0,5 m + biais 1 m), la recherche autour des arbres
  est décalée de 1 à 2 m → zones marquées « vues » sans l'avoir été, 23/29 (FAIL). Il faudrait du RTK,
  ou recaler la pose sur les troncs vus par la caméra.

## D15 — Recalage de la pose sur les troncs vus par la caméra (session 13)
- Contexte : GPS « grand public » (σ 0,5 m + biais 1 m) → 23/29, recherche décalée de 1 à 2 m (D14). La
  caméra regarde le sol : seul le pied du tronc est visible, et il est juste hors champ depuis le cercle
  de recherche (1,9 m).
- Décision :
  - Détection du pied : ajustement d'un cercle de rayon connu sur l'arête basse du blob marron projetée
    au sol (non biaisé vu de côté, contrairement au centroïde), points de silhouette écartés.
  - EKF : même modèle de mesure que le GPS, le point mesuré étant le tronc. Association à l'arbre le plus
    proche (< 2,8 m ; arbres à 6 m). Innovation improbable → covariance gonflée, pas de rejet : un biais GPS
    rend le filtre trop confiant, et le tronc est la mesure fiable.
  - Nouvel état `ANCHOR` à l'arrivée à chaque arbre : face au tronc, avance lente (≤ 1,2 m, arrêt à 0,7 m
    estimé), regard à ± 0,8 rad, puis SEARCH même sans recalage. Actif seulement si σ ou biais > 0,1 m
    (constaté : avec RTK, 27/29 et 806 s contre 29/29 et 723 s sans).
- Écarté : réduire le cercle de recherche ou relever la caméra (réglages de couverture et perception à
  refaire) ; modéliser le biais GPS comme état du filtre (plus complexe, à reconsidérer pour réduire l'erreur
  entre deux arbres).
- Résultat : 84/87 pommes sur 3 seeds de bruit (≥ 27/29 chacun), au prix de ~20–30 % de temps en plus.

## D16 — Verger difficile séparé, herbe seulement visuelle (session 14)
- Contexte : M9 « herbe haute, pommes cachées ou abîmées ». Le verger par défaut sert de référence de non-régression.
- Décision : config à part `orchard_hard.yaml`, aux mêmes positions de pommes (tirages d'apparence sur un générateur
  aléatoire distinct). Brins d'herbe sans collision : le robot les traverse. Bande rouge élargie à 0–14 et troncs
  remontés à 15 (teintes rendues mesurées : pomme orange 12, tronc 18). Fermeture morphologique de 7 px.
- Écarté : modifier le verger par défaut (perdrait la comparaison) ; herbe en collision (coût physique, sans intérêt pour
  la caméra) ; apprentissage (YOLO) : la couleur suffit encore sur ce monde.
- Limite : marge de teinte de 3–4 valeurs entre pomme orange et tronc ; deux pommes à moins de 7 px fusionnent.

## D17 — Pare-chocs à contact et recul (session 14)
- Contexte : en GPS grossier, l'erreur de pose peut mener le robot dans un tronc ; il y restait indéfiniment (20/29).
- Décision : capteur de contact gz sur le châssis et sur une bande plus large que les roues, comme sur les robots tondeuses
  du commerce. Tout contact hors sol et pommes déclenche un recul de 2 s (avancée si toujours bloqué), puis l'abandon de la
  pomme ou de la vue visée. Délai de 40 s sur les vues de SEARCH en secours. Le plugin Contact est dans le monde.
- Écarté : capteurs sur les roues (touchent toujours le sol : 2000 messages/s sur `/bumper`) ; délai seul (40 s perdues à
  chaque choc, et le robot continue de pousser) ; détection de patinage (les roues glissent sans signal fiable).
- Limite : le choc n'améliore pas la pose, et la pomme visée est perdue pour la boucle.

