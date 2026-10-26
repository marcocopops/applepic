# Bilan (pour l'humain)

## 1. État
- M0 : infrastructure en place (workspace ROS 2 Jazzy natif, Makefile, docs). `make test` passe.
- Le rendu caméra Gazebo headless fonctionne sur cette machine (GPU NVIDIA).
- M1 : monde du verger généré depuis YAML (6 arbres, 29 pommes, zone de dépôt) ; vérifié en
  headless (SDF valide, pommes stables, capture vue de dessus dans `artifacts/orchard_overhead.png`).

## 2. Lancer
```bash
cd ~/Documents/ApplePic
make build && make test
make world          # régénère le verger (config: src/applepic_gazebo/config/orchard.yaml)
gz sim src/applepic_gazebo/worlds/orchard.sdf   # voir le verger (GUI)
```
(la démo complète arrivera au jalon M8)

## 3. Décisions à relire
- D1 : environnement natif plutôt que Docker (voir `docs/DECISIONS.md`).
- D3 : monde généré + commité, pommes dynamiques, dépôt sans collision.

## 4. Cassé / bloqué
- Rien pour l'instant. Voir `docs/TODO_HUMAIN.md` (optionnel).

## 5. Prochaines étapes
- M2 robot (URDF + diff drive + caméra), puis M3 navigation.
