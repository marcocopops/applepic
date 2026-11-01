# Audit du projet (2026-11-01)

## État réel (rejoué aujourd'hui sur le commit `f6610b5`)
1. `make test` : 163 tests, 0 échec, 22 s.
2. Verger par défaut, pose vraie, `scripts/e2e_mission.sh` joué 2 fois : **29/29 PASS** en 722 et 721 s simulées (~4 min 20 réelles).
3. GPS grossier (0,5 m + biais 1 m), verger seed 7, jamais testé avant : **31/31 PASS**, 1000 s, 4 abandons de piste.
4. GPS grossier + obstacles, bruit seed 0 : **26/29 PASS**, 949 s (la doc annonce 27/29 ; même ordre de grandeur, runs non déterministes).
5. `scripts/check_demo.sh` : PASS, mais il s'arrête après **14 s simulées** et 5 m parcourus (12 s réelles). Il prouve que les topics RViz existent, pas que la démo va au bout.
6. Démo GUI non vérifiée par cet audit, ni par les sessions depuis la session 14. Un démarrage sur 9 restait muet, cause inconnue.
7. La doc est fiable sur les chiffres : aucune affirmation contredite. Elle dit moins clairement ce qui suit.
8. **Le robot n'a pas de bras ni de pince.** Le « préhenseur » est un repère : sous 7 cm de la vérité terrain, la pomme est supprimée du monde.
9. Les roues traversent les pommes (D19) et le monde reste facile : sol plat, couleurs unies, aucun distracteur rouge.
10. Conclusion : c'est une bonne démo de navigation et de recherche au sol. Ce n'est pas encore un ramasseur de pommes.

## Cause du ralentissement
Mesuré dans `logs/runner.log` et git : 22 lancements, 10,6 h de sessions, 52 commits.

| | 26/10 | 30/10 |
|---|---|---|
| Durée d'une session utile | 4 à 20 min (M1–M8), puis 33 et 125 min | 48 à 98 min |
| Commits par heure | 5,7 | 3,7 |
| Résultat | M0 à M8 en ~2 h 10 | 5 sous-sujets de M9 en 6,5 h |

1. **Feuille de route épuisée après ~2 h.** M9+ est une liste de 7 idées sans critère, « à prioriser toi-même ». Rien n'y définit « fini ».
2. **Dérive vers un scénario auto-imposé.** Le GPS RTK donnait déjà 29/29 en session 12. Les sessions 13 à 16 (~5 h) ont traité le GPS grossier, et chaque correctif a ouvert le suivant : troncs → `ANCHOR` → blocage → pare-chocs → obstacles → A*. Gain obtenu : ~94–95 % sur un scénario que personne n'a demandé.
3. **Métrique saturée.** Le scénario de référence est à 100 % depuis la session 8. Les sessions n'ont plus de signal et créent des scénarios plus durs pour en retrouver un.
4. **Attente de simulation.** Une série de validation dure 35 à 45 min (session 15 : ~80 min sur 96), et les runs sont non déterministes avec un seul run par seed. Le 18 → 29/29 de la session 16 surestime le gain, comme PROGRESS le reconnaît.
5. **Le bras n'est jamais choisi.** Dernier de la liste, il dépend de paquets non installés (`sudo`, interdit aux sessions) : les sessions prennent ce qu'elles peuvent finir.
6. **La règle d'arrêt ne s'applique pas.** C'est la session qui juge s'il reste « une amélioration utile », et il en reste toujours. Elle n'a jamais créé `STOP`. Les deux `STOP` (26/10 17:45, 30/10 23:24) ne viennent ni des sessions ni d'un script, donc sans doute de toi.
7. **Pas de boucle depuis le 30/10 23:37.** Le `STOP` est toujours présent : `run_applepic.sh` sortirait aussitôt.

Écarté : régressions (`make test` n'a cassé qu'une fois, `c5c0951`, corrigé au commit suivant), sessions trop courtes (les 2 s et 52–61 s du début viennent de la limite d'usage et des `pkill -f`, réglés le 26/10).
Redécouvert à chaque session : l'éjection des pommes (sessions 11, 13, 14, résolue en 15 par D19) et « démo GUI non relancée » (répété 5 fois, jamais traité). `BLOCKERS.md` est resté vide malgré plusieurs causes inconnues.

## Points forts
- Logique pure séparée des nœuds : 97–100 % de couverture sur `mission`, `route`, `localization`, `detector`, `coverage`, `tracker` ; tests rapides.
- `e2e_mission.sh` recoupe le score avec Gazebo, produit des métriques et donne les poses des pommes perdues.
- Décisions (D1–D20) et limites écrites honnêtement, causes établies par mesures.
- Historique propre : commits petits, aucun secret, aucun fichier lourd (voir étape 5 du recadrage).

## Problèmes, du plus grave au moins grave
**Critique**
1. Pas de manipulation : ni bras, ni pince, ni contact. Le cœur du POC n'est pas simulé.
2. Feuille de route sans critère, voir ci-dessus.

**Élevé**
3. Validation trop faible : seuil de 80 %, un run par seed, limite passée de 1200 à 1500 s selon les séries, `check_demo.sh` qui ne regarde que 14 s, pas de CI.
4. Perception jamais mise en difficulté : aucun distracteur, éclairage fixe, 3–4 valeurs de teinte entre pomme orange et tronc, position exacte des arbres supposée connue.
5. `mission.py` : classe unique de 667 lignes (303 à sa création), une quarantaine de paramètres, plus de 30 attributs d'état. Y ajouter le bras sans découpage la rendrait ingérable.
6. Nœuds ROS à 0 % en test unitaire (`gripper`, `mission_node`, `localizer`, `apple_detector`), couverts seulement par l'e2e.

**Moyen**
7. Bug latent : `mission`, `localizer` et `mission_viz` lisent toujours `orchard.yaml` par défaut, les launch ne passent jamais `orchard_config`. Un verger avec d'autres arbres se désynchronise sans erreur (masqué : les 3 configs ont les mêmes arbres).
8. Constantes dupliquées entre URDF, `CameraParams` et les nœuds (caméra, `offset` 0,46 m).
9. `ANCHOR` s'active d'après le niveau de bruit passé au launch : le robot « connaît » la qualité de son GPS par configuration.
10. Pile maison (contrôleur, EKF, A*) choisie parce que Nav2 et `robot_localization` ne sont pas installés, pas sur le fond.

**Faible**
11. Code de jalons dépassés : `apple_picker`, `waypoint_follower`, leurs launch et scripts ; import inutile dans `waypoint_follower.py`.
12. Docs lourdes : PROGRESS 340 lignes (30 Ko) relues à chaque session, BILAN écrit « pour ce soir » et mal ordonné. Dockerfile jamais vérifié.
13. `run_applepic.sh` : logs réduits au dernier message (impossible à diagnostiquer après coup), pas de pause après une erreur tardive (`code 1` à 495–503 s), heure limite déjà passée = sortie muette, pas de plafond par session, pas de verrou contre deux boucles, modifications locales jamais commitées.

## Recommandations
**Garder** : architecture logique pure + nœuds, verger paramétrable, détection HSV, tracker, couverture, `e2e_mission.sh` et ses métriques, localisation GPS (telle quelle), règles de commit.

**Refaire** :
- la feuille de route, avec un critère vérifiable par jalon ;
- `mission.py`, découpé avant d'ajouter le bras ;
- le préhenseur : vrai bras, vraie pince, et `/odom` vérité terrain retiré du ramassage ;
- D19 (roues traversant les pommes), à réévaluer une fois le bras en place ;
- la validation : plusieurs runs par scénario, score médian, seuil relevé, `check_demo.sh` jusqu'au premier dépôt ;
- `run_applepic.sh` et le format de PROGRESS / BILAN.

**Geler** : réglages GPS grossier et obstacles. Plus de travail dessus sans demande explicite ; le scénario sert de non-régression.

**Supprimer** : `apple_picker`, `waypoint_follower`, `picking.launch.py`, `navigation.launch.py`, `check_picking.sh`, `eval_picking.py`, `check_navigation.sh`, `plot_trajectory.py`, `build_route` et `approach_points` ; le Dockerfile s'il reste non vérifié.
