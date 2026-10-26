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
