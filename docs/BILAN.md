# Bilan (pour l'humain)

## 1. État
- M0 : infrastructure en place (workspace ROS 2 Jazzy natif, Makefile, docs). `make test` passe.
- Le rendu caméra Gazebo headless fonctionne sur cette machine (GPU NVIDIA).

## 2. Lancer
```bash
cd ~/Documents/ApplePic
make build && make test
```
(la démo arrivera au jalon M8)

## 3. Décisions à relire
- D1 : environnement natif plutôt que Docker (voir `docs/DECISIONS.md`).

## 4. Cassé / bloqué
- Rien pour l'instant. Voir `docs/TODO_HUMAIN.md` (optionnel).

## 5. Prochaines étapes
- M1 verger, M2 robot.
