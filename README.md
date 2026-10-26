# ApplePic

POC en simulation (ROS 2 Jazzy + Gazebo Harmonic) d'un robot ramasseur de pommes tombées au sol.
Le robot suit une boucle de pommiers, cherche les pommes au sol à la caméra, les ramasse,
puis va vider son panier à une zone de dépôt.

## Prérequis
- Ubuntu 24.04 avec ROS 2 Jazzy et `ros-jazzy-ros-gz` installés (`/opt/ros/jazzy`).
- Optionnel : Docker (`make docker-image`, puis `USE_DOCKER=1 make build`).

## Commandes
```bash
make build          # compile le workspace
make test           # tests unitaires
make world          # régénère le monde du verger
make sim-headless   # lance la simulation sans GUI
make e2e            # test de bout en bout headless (~5 min) -> PASS/FAIL + métriques
make demo           # démo : mission complète avec GUI Gazebo + RViz
```

## Démo pas à pas (GUI Gazebo + RViz)
Sur la machine avec écran (pas en SSH sans X) :

1. Compiler et vérifier :
   ```bash
   cd ~/Documents/ApplePic
   make test
   ```
2. Lancer la démo (ouvre deux fenêtres : Gazebo et RViz ; il faut ~15 s pour que tout apparaisse) :
   ```bash
   make demo
   ```
   Variante plus rapide (simulation ×2, non testée avec la GUI, à essayer si la machine suit) :
   ```bash
   make demo DEMO_RTF=2
   ```
   Équivalent sans make, avec options : `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
   puis `ros2 launch applepic_bringup demo.launch.py [rtf:=2] [capacity:=10] [loops:=1] [rviz:=false]`.
3. Ce qu'on voit :
   - **Gazebo** : le verger (6 pommiers, pommes rouges au sol, dépôt bleu) et le robot. Les pommes
     disparaissent quand le préhenseur passe dessus (ramassage « magique »).
   - **RViz** (vue de dessus du verger) : troncs et couronnes, cercle jaune de recherche autour de
     chaque arbre, dépôt, trajectoire du robot (bleu clair), pommes détectées (flèches rouges),
     et au-dessus du robot un texte d'état : état de la mission, arbre courant, panier `n/10`,
     pommes ramassées / déposées, distance et temps (rafraîchis à chaque changement d'état).
     Panneau en haut à gauche : image caméra annotée (détections encadrées).
4. Déroulé : le robot part du dépôt, fait le tour de chaque pommier (jusqu'à 8 points
   d'observation à 2 m, en sautant ceux dont le sol a déjà été vu par la caméra), ramasse les
   pommes vues, revient vider son panier quand il est plein (10 pommes), puis au dépôt en fin de
   boucle. Une boucle complète prend ~11 min à vitesse réelle.
5. Suivre la mission dans un autre terminal (après `source install/setup.bash`) :
   ```bash
   ros2 topic echo /mission/status      # transitions de la machine à états
   ros2 topic echo /basket/count        # contenu du panier
   ```
6. Arrêter : `Ctrl+C` dans le terminal de `make demo` (ferme Gazebo et RViz).

Vérification automatique sans écran de la même démo : `scripts/check_demo.sh`.

État d'avancement : voir `docs/BILAN.md` et `docs/PROGRESS.md`.
