# ApplePic — consignes de session autonome

Tu travailles SEUL, sans humain disponible, dans le dossier courant (le repo ApplePic).
Tu es relancé en boucle par un script : chaque session repart avec un contexte vide.
Ta mémoire entre sessions, ce sont les fichiers du repo (surtout `CLAUDE.md`,
`docs/PROGRESS.md`, `docs/DECISIONS.md`) et l'historique git.

## Le projet

ApplePic est un POC en simulation d'un robot ramasseur de pommes tombées au sol :
- Format robot tondeuse (base différentielle, environ 60 x 45 cm), équipé d'un GPS,
  d'une caméra, d'un panier et d'un mécanisme de ramassage.
- Il suit une boucle de waypoints GPS ; chaque waypoint = un pommier.
- À chaque pommier, il cherche les pommes au sol autour de l'arbre (caméra), les ramasse,
  et passe au pommier suivant quand il n'en détecte plus.
- En fin de boucle, il va à une zone de dépôt, vide son panier, et recommence.

Décisions déjà prises par l'humain (ne pas les remettre en cause) :
- Stack : ROS 2 + Gazebo nouvelle génération (gz sim, PAS Gazebo Classic).
  Distro alignée sur l'OS (Jazzy + Harmonic si Ubuntu 24.04, sinon le couple officiel adapté).
- Ramassage "magique" pour le POC : quand le préhenseur est à moins de quelques cm
  d'une pomme, on l'attache au robot (DetachableJoint) ou on la retire du monde et on
  incrémente le compteur du panier. Pas de manipulation physique réaliste à ce stade.
- Navigation : waypoints en coordonnées monde d'abord (odométrie vérité terrain),
  GPS simulé (navsat + robot_localization) dans un second temps.
- Perception : segmentation couleur (HSV) d'abord, apprentissage (YOLO) éventuellement plus tard.

## Règles d'autonomie (impératives)

1. Ne pose JAMAIS de question et n'attends jamais de validation. Tranche toi-même,
   choisis l'option la plus simple qui marche, et note chaque décision non triviale
   dans `docs/DECISIONS.md` (format court : contexte / décision / alternatives écartées).
2. N'utilise JAMAIS `sudo`. Ne modifie rien en dehors de ce dossier, à l'exception
   des images/conteneurs Docker, des environnements virtuels Python et des caches de paquets.
   Ne fais jamais de `git push`, ne configure aucun remote, n'envoie rien sur Internet
   hors téléchargement de paquets/images.
3. Si une chose nécessite une action humaine (installation système, sudo, GPU...),
   écris-la dans `docs/TODO_HUMAIN.md` avec la commande exacte, puis contourne et continue.
4. Si tu restes bloqué plus d'environ 45 minutes sur un même problème, documente-le dans
   `docs/BLOCKERS.md` (symptôme, ce que tu as essayé, pistes) et passe à la tâche suivante.
5. Ne réécris jamais l'historique git. Commits petits et fréquents, messages au format
   conventional commits en anglais.
6. Code, noms et commentaires en anglais. Documentation (`docs/`, README) en français.

## Environnement d'exécution (à décider en session 1)

Détecte : version d'Ubuntu (ou WSL), présence de `/opt/ros`, de `gz`, de Docker utilisable
sans sudo (`docker run --rm hello-world`), d'un GPU.
Ordre de préférence :
1. Docker disponible sans sudo → environnement de dev conteneurisé (image ROS 2 officielle
   avec Gazebo, ex. `osrf/ros:<distro>-desktop-full` ou équivalent + ros_gz). Tout se build,
   se teste et se lance en headless dans le conteneur. Fournis un `Makefile` qui masque Docker.
2. Sinon ROS 2 + gz déjà installés nativement → les utiliser.
3. Sinon → écris quand même tout le code, plus un script d'installation dans `TODO_HUMAIN.md`,
   et vérifie ce qui peut l'être sans ROS (logique pure Python, tests unitaires, perception
   sur images synthétiques).

Tu ne vois pas d'écran : tout doit être vérifiable en headless (`gz sim -s`, `ros2 topic echo`,
`ros2 bag`, tests pytest/launch_testing, captures d'image de la caméra sauvegardées en PNG).
Si le rendu caméra headless échoue (EGL/OpenGL), documente-le et avance sur le reste avec
des images simulées ; ne t'acharne pas.

## Protocole de chaque session

1. Lis `CLAUDE.md`, `docs/PROGRESS.md`, `docs/BLOCKERS.md` et `git log --oneline -20`.
   Si ces fichiers n'existent pas, tu es la première session : commence par le jalon M0.
2. Choisis LA prochaine tâche la plus utile dans la feuille de route ci-dessous.
3. Implémente-la, puis VÉRIFIE-LA réellement (build, tests, lancement headless).
   Une tâche non vérifiée n'est pas terminée : dis-le honnêtement dans PROGRESS.
4. Commit.
5. Mets à jour `docs/PROGRESS.md` (fait / vérifié comment / ce qui ne marche pas / prochaine
   tâche précise) et `docs/BILAN.md` (voir plus bas). Commit.
6. Termine la session après environ un jalon (ou plus tôt si ton contexte devient lourd).
   La boucle te relancera avec un contexte neuf.

## Feuille de route

- **M0 — Infrastructure** : `git init`, arborescence d'un workspace ROS 2 (`src/`),
  `.gitignore` (inclure `logs/`, `build/`, `install/`, `log/`), environnement d'exécution
  choisi (Dockerfile + Makefile avec `make build`, `make test`, `make sim-headless`),
  `README.md`, `CLAUDE.md` (contexte projet, commandes, conventions, comment vérifier),
  `docs/PROGRESS.md`, `docs/DECISIONS.md`, `docs/BILAN.md`, `docs/TODO_HUMAIN.md`.
  Paquets suggérés : `applepic_description`, `applepic_gazebo`, `applepic_navigation`,
  `applepic_perception`, `applepic_mission`, `applepic_bringup`.
- **M1 — Verger** : générateur Python de monde SDF paramétrable (N pommiers sur une boucle,
  pommes rouges réparties aléatoirement autour de chaque arbre avec seed, zone de dépôt
  visible, sol herbeux). Fichier de config YAML pour les positions des arbres.
- **M2 — Robot** : URDF/xacro de la base différentielle avec caméra orientée vers le sol
  à l'avant, panier, préhenseur simple. Plugins gz (diff drive, caméra, odométrie), bridge
  ros_gz. Vérifier qu'une commande `cmd_vel` fait bouger le robot.
- **M3 — Navigation** : suivi de la boucle de waypoints (Nav2 waypoint follower ou contrôleur
  maison plus simple, à toi de trancher et de justifier).
- **M4 — Perception** : détection des pommes par HSV, projection pixel → sol (hypothèse de
  sol plan + extrinsèques caméra), publication des pommes détectées dans le repère monde,
  tests unitaires sur images synthétiques et sur captures de la simu.
- **M5 — Ramassage** : approche d'une pomme détectée, déclenchement du ramassage "magique",
  compteur et capacité du panier.
- **M6 — Mission** : machine à états GOTO_TREE → SEARCH (motif de recherche autour de l'arbre
  dans un rayon configurable) → PICK → arbre suivant ; fin de boucle → GOTO_BASE → UNLOAD →
  nouvelle boucle. Transitions testées unitairement.
- **M7 — Test de bout en bout** : scénario headless chronométré qui vérifie qu'au moins X %
  des pommes sont ramassées et déposées ; sortie de métriques (pommes ramassées, temps,
  distance parcourue).
- **M8 — Démo** : launch "démo" avec GUI Gazebo + RViz pour que l'humain la regarde ce soir,
  instructions pas à pas dans le README.
- **M9+ — Améliorations** (à prioriser toi-même) : GPS simulé bruité + robot_localization,
  obstacles et herbe haute, pommes partiellement cachées ou abîmées, génération d'un dataset
  annoté depuis la simu pour entraîner un détecteur, CI (`.gitlab-ci.yml`), bras réel avec
  MoveIt, gestion de batterie et retour à la base.

## docs/BILAN.md

Ce fichier sera lu par l'humain ce soir. Garde-le à jour, court et honnête :
1. État en 5 lignes : qu'est-ce qui marche vraiment, vérifié comment.
2. Comment lancer la démo (commandes exactes, copiables).
3. Décisions importantes prises à sa place, qu'il devrait relire.
4. Ce qui est cassé ou bloqué, et ce qui attend une action de sa part (renvoi vers TODO_HUMAIN).
5. Prochaines étapes suggérées.

Si toute la feuille de route est terminée et vérifiée, et que tu ne vois plus d'amélioration
utile, crée un fichier `STOP` à la racine pour arrêter la boucle.
