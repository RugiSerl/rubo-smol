# Simulation

Deux outils font tourner `robot/Robot.h` sur l'ordinateur, sans robot :

| Outil                     | Ce qu'il fait                                                                           | TP            |
| ------------------------- | --------------------------------------------------------------------------------------- | ------------- |
| `python3 tools/sim.py`    | Joue une suite de commandes dans une scène, aussi vite que possible, et trace les courbes | 1, 5, 6, 7    |
| `node client/simrobot.js` | Fait tourner le robot simulé en temps réel, derrière la page du téléphone               | 1, 2, 3, 5, 7 |

Dans les deux cas, le code de `Robot.h` est celui qu'on chargera ensuite sur le robot. Il est compilé tel quel ; seuls les fichiers du matériel (`Motors.h`, `TofSensor.h`, `UltrasonicSensor.h`, `Clock.h`, etc.) sont remplacés par ceux de `tools/sim/stubs/`. `Config.h`, et `Config.local.h` s'il existe, sont lus tels quels : un capteur désactivé n'y mesure rien, comme sur le robot.

Le robot simulé a les défauts d'un vrai robot : un côté plus faible que l'autre, des roues qui réagissent avec retard, des capteurs qui mesurent à leur rythme et perdent parfois une mesure. Le télémètre à ultrasons, à l'avant, ne reçoit pas d'écho d'une surface trop oblique ; le télémètre laser, sur le flanc gauche, ne voit rien au-delà de 1,3 m. Les valeurs du modèle sont inventées (elles sont listées dans `tools/sim/stubs/SimWorld.h`) : les comportements sont ceux du robot, mais pas les amplitudes, les périodes ni les distances d'arrêt. Il faudra les mesurer à nouveau sur le robot.

## Les scènes

| Scène        | Contenu                                                                                | Départ par défaut (`--start`)             |
| ------------ | -------------------------------------------------------------------------------------- | ----------------------------------------- |
| `mur`        | Un mur droit, sans fin, à gauche du robot                                              | flanc gauche à 260 mm du mur              |
| `patrouille` | Le même mur, une boîte de 300 mm sur le chemin à x = 1,5 m, un mur en travers à x = 3,5 m | flanc gauche à 260 mm du mur              |
| `obstacle`   | Un mur en travers du chemin                                                            | avant du robot à 800 mm du mur            |
| `salle`      | Une pièce fermée de 3 m sur 2,4 m, le robot sur sa ligne du milieu                     | avant du robot à 1400 mm du mur d'en face |

Le robot part toujours face à l'axe x. La boîte de `patrouille` et le mur de `obstacle` sont l'« obstacle » de la scène : on peut le retirer et le remettre pendant que le robot roule.

## Jouer une suite de commandes : `tools/sim.py`

### Lancer une simulation

1. Pour une mission qui suit un mur, mettre `SIDE_SENSOR_ENABLED` à `true` dans `robot/Config.h`, et dans `robot/Config.local.h` s'il existe. Sinon le capteur latéral ne renvoie rien, comme sur le robot.
2. Depuis la racine du projet :

   ```sh
   python3 tools/sim.py
   ```

3. Ouvrir `sim/run.svg` depuis l'explorateur de VS Code : VS Code affiche le graphique. Le garder ouvert à côté de `Robot.h`, par exemple en faisant glisser son onglet vers la droite de l'éditeur. L'affichage se met à jour à chaque nouvelle simulation.

Sans autre option, la simulation se déroule dans la scène `mur` et envoie `{"mission": 3}` au robot. `--scene` choisit une autre scène ; chacune envoie ses propres commandes, sauf si on en donne d'autres avec `--send` ou `--mission` :

| Scène        | Commandes envoyées au départ                                        |
| ------------ | ------------------------------------------------------------------- |
| `mur`        | `{"mission":3}`                                                     |
| `patrouille` | `{"mission":5}`                                                     |
| `obstacle`   | `{"mission":1}`, puis `{"left":200,"right":200,"duration":3000}`    |
| `salle`      | `{"mission":2}`                                                     |

Le robot roule ensuite 20 s. La simulation s'arrête plus tôt si le robot touche un mur, s'il s'éloigne de plus de 3 m du mur (scènes `mur` et `patrouille`), ou si ses roues ne tournent plus depuis 3 s alors qu'aucun événement n'est plus prévu.

Comme sur le robot, les commandes arrivent bien après le démarrage : `begin()` est appelé à 2 s, et la première commande arrive à 5 s. Au premier passage de la mission, `nowMs()` vaut donc 5000 et les capteurs ont déjà une mesure. Une variable membre ajoutée sans valeur de départ vaut 0, comme sur le robot. Les temps du résumé, des options et des fichiers produits comptent à partir de la première commande.

La simulation produit trois fichiers :

| Fichier       | Contenu                                                                                         |
| ------------- | ----------------------------------------------------------------------------------------------- |
| `sim/run.svg` | Trajectoire vue de dessus, puis dans le temps : mesures des capteurs, cap et commandes des roues |
| `sim/run.csv` | Une ligne toutes les 10 ms : position, cap, commandes, mesures, distances réelles aux murs      |
| `sim/run.log` | Chaque ligne envoyée au robot (`>`), chaque ligne qu'il a renvoyée (`<`), et les événements (`-`) |

`sim/run.log` se lit comme la console du robot, avec le temps en tête de ligne :

```text
   0.000 > {"mission":1}
   0.000 > {"left":200,"right":200,"duration":3000}
   0.000 < {"seq":-1,"ms":5000,"left":8,"right":8,"moving":true,"mission":1,"front_mm":799,...}
   ...
   2.161 < {"seq":-1,"ms":7161,"left":0,"right":0,"moving":false,"mission":1,"front_mm":144,...,"done":true}
```

Le terminal affiche un résumé :

```text
end      time up at 20.00 s
position x = 4709 mm, clearance 248 mm, heading 4.3 deg
nearest  240 mm between the body and the closest wall
driven   4.72 m
front    333 readings, 333 missing
side     500 readings, 7 missing
safety   0 stops while the wheels were turning
```

| Ligne      | Contenu |
| ---------- | ------- |
| `position` | Scènes `mur` et `patrouille` : position le long du mur, `clearance` et cap. `clearance` est la distance entre le flanc gauche et le mur, au niveau de l'essieu, mesurée comme si le robot était parallèle au mur : c'est la colonne `clearance_mm` du `.csv`. Autres scènes : position x et y du centre du robot, et cap |
| `nearest`  | Distance entre le mur le plus proche et le point du robot le plus proche de lui : un coin, quand le robot est de biais. Un robot qui touche un mur finit avec `nearest 0 mm` |
| `front`, `side` | Nombre de mesures de chaque capteur activé dans `Config.h`, et nombre de mesures manquantes. Dans la scène `mur`, rien ne se trouve devant le robot : le capteur avant ne mesure jamais rien |
| `safety`   | Arrêts des roues en marche, qu'ils viennent d'`enforceSafety()` ou de la mission |

Une ligne `warning` s'ajoute si des lignes `"done":true` partent alors que les roues tournent encore : c'est le cas d'une mission qui appelle `sendTelemetry(true)` à chaque passage. Une ligne `note` s'ajoute dans la scène `mur` si une mission a été lancée mais que les roues n'ont jamais tourné : une mission qui attend une mesure du capteur avant n'y bouge pas. La mission 5 se simule dans la scène `patrouille`.

Deux autres fins s'affichent à la place du résumé :

| Ligne     | Cause                                                                                   |
| --------- | --------------------------------------------------------------------------------------- |
| `stuck`   | Une boucle de `Robot.h` ne se termine jamais : le temps simulé n'avance plus           |
| `crashed` | Le programme s'est arrêté : accès hors d'un tableau, division entière par zéro, etc.   |

Dans les deux cas, les trois fichiers contiennent ce qui a été enregistré jusque-là, s'il y a quelque chose. Le temps simulé avance dans la boucle principale, dans `sleep_ms()` et au fil des appels à `nowMs()` ; un capteur ne change que dans son `update()`. Une boucle qui attend une mesure sans appeler `front.update()` ou `side.update()` ne se termine donc jamais, sur le robot comme en simulation.

### Le graphique

Le fichier `run.svg` contient, de haut en bas :

| Graphique | Contenu |
| --------- | ------- |
| Vue de dessus | Scènes `mur` et `patrouille` : le milieu du flanc gauche, axe vertical étiré (voir la note ci-dessous), avec la boîte et le mur du fond quand le robot s'en approche. Scènes `obstacle` et `salle` : la pièce à l'échelle, le chemin du centre du robot, le robot au départ (en pâle) et à la fin, et l'axe du capteur avant en pointillés jusqu'au premier mur. L'obstacle est en brun, en pointillés s'il a été retiré. Un anneau rouge marque l'endroit où le robot a touché un mur |
| Choc, à l'échelle | Seulement si le robot a touché le mur de gauche, dans les scènes `mur` et `patrouille` : le robot dessiné à sa position finale, avec le faisceau du capteur latéral |
| Mesure avant | Les mesures de `front_mm`, et en pointillés la distance réelle du capteur au premier mur droit devant lui (colonne `front_wall_mm`). Les barres rouges marquent l'absence de mesure valide. Absent de la scène `mur` |
| Mesure latérale | Les mesures de `side_mm`, et en pointillés la distance réelle du capteur au mur, mesurée perpendiculairement au mur (colonne `wall_mm`). Présent dans les scènes `mur` et `patrouille`, et dans les autres si le capteur latéral a mesuré quelque chose |
| Cap | L'angle du robot, positif quand il a tourné vers la gauche : vers le mur dans les scènes `mur` et `patrouille` (colonne `heading_deg`) |
| Commandes des roues | Les valeurs passées aux moteurs, de −255 à 255 |

Sur les graphiques en fonction du temps, une zone grisée marque une liaison coupée ou un capteur débranché, et un trait brun en pointillés le moment où l'obstacle a été retiré ou remis.

**Note.** Dans les scènes `mur` et `patrouille`, la vue de dessus n'est pas à l'échelle : l'axe vertical est très étiré pour rendre les oscillations visibles, ce qui aplatit les angles. Le cap se lit sur le graphique « Cap ».

**Note.** Chaque capteur mesure le point le plus proche dans son cône. Quand le robot est de biais par rapport au mur, la mesure latérale dépasse la distance perpendiculaire : comparer les points aux pointillés. Quand un mur entre par le côté du cône, la mesure avant passe sous la distance droit devant.

### Options

| Option                      | Effet                                                                                  |
| --------------------------- | -------------------------------------------------------------------------------------- |
| `--scene obstacle`          | Choisir la scène : `mur` (par défaut), `patrouille`, `obstacle`, `salle`               |
| `--send 2:'{"stop":true}'`  | Envoyer une ligne au robot 2 s après le départ, comme dans la console ; à répéter pour en envoyer plusieurs. Remplace les commandes de la scène |
| `--mission 4`               | Envoyer `{"mission":4}` au départ. Remplace les commandes de la scène                  |
| `--offline 6:8`             | Couper la liaison de 6 s à 8 s ; à répéter pour plusieurs coupures                     |
| `--remove 10`               | Retirer l'obstacle à 10 s (scènes `obstacle` et `patrouille`)                          |
| `--place 14`                | Remettre l'obstacle à 14 s                                                             |
| `--unplug front:6:7`        | Débrancher le capteur avant (`front`) ou latéral (`side`) de 6 s à 7 s ; à répéter     |
| `--start 300`               | Distance de départ en mm : du flanc gauche au mur (`mur`, `patrouille`), de l'avant du robot au mur d'en face (`obstacle`, `salle`) |
| `--heading 10`              | Angle de départ en degrés, positif vers la gauche : vers le mur dans `mur` et `patrouille` |
| `--door 2000:2900`          | Une ouverture dans le mur de gauche, de x = 2000 mm à x = 2900 mm (`mur`, `patrouille`) |
| `--duration 30`             | Durée en secondes                                                                      |
| `--out sim/nom`             | Écrire `sim/nom.svg`, `sim/nom.csv` et `sim/nom.log` au lieu de `sim/run.*`             |
| `--compare sim/nom.csv`     | Dessiner en gris, derrière, une simulation précédente                                  |
| `-D NOM=VALEUR`             | Définir une constante à la compilation, par exemple un gain                            |
| `--seed 2`                  | Autre tirage du bruit des capteurs                                                     |

Dans le terminal, une ligne JSON passée à `--send` s'entoure d'apostrophes, pour que le shell la transmette telle quelle. Les temps de `--send`, `--offline`, `--remove`, `--place` et `--unplug` sont en secondes, comptées à partir du départ ; une commande ne peut pas partir pendant une coupure de la liaison.

`-D` ne remplace une valeur que si `Robot.h` ou `Config.h` la définit avec `#ifndef`, comme `OBSTACLE_STOP_MM` :

```cpp
#ifndef WALL_GAIN_NUM
#define WALL_GAIN_NUM  1
#endif
```

**Note.** Toute erreur de compilation s'affiche avec le chemin de `robot/Robot.h` : on la corrige comme pour `tools/build.py`.

## Le robot derrière la page : `client/simrobot.js`

1. Depuis la racine du projet :

   ```sh
   node client/simrobot.js
   ```

   `Robot.h` est compilé comme pour `tools/sim.py`, puis le robot démarre dans la scène `salle`. `--scene` en choisit une autre.

2. Ouvrir dans le navigateur de l'ordinateur :

   | Adresse                      | Contenu                                                    |
   | ---------------------------- | ---------------------------------------------------------- |
   | `http://localhost:8080/`     | La page du téléphone, telle que le robot la sert           |
   | `http://localhost:8080/sim`  | La même page, à côté de la scène vue de dessus             |

3. Arrêter par ctrl-C.

Le robot simulé se comporte comme le robot en WiFi :

- `robot/web/` est relu à chaque rechargement de la page : une modification de `index.html`, `missions.js` ou `robot.js` s'essaie sans recompiler. Une modification de `Robot.h`, elle, demande de recompiler (bouton ci-dessous).
- Les requêtes `POST /api/command` et `GET /api/lines` ont les mêmes limites que sur le robot : 8 commandes par requête, 16 lignes gardées.
- Sans requête de la page pendant 1,5 s (`WEB_TIMEOUT_MS`), la liaison est perdue et les roues s'arrêtent.
- Les `printf` de `Robot.h` s'affichent dans le terminal.
- Un `Robot.h` bloqué dans une boucle, ou qui s'arrête sur une erreur, ne répond plus à rien : la page perd la liaison, et le terminal dit pourquoi.

La vue `/sim` montre le robot, son chemin, et le cône de chaque capteur jusqu'à sa mesure (en pâle quand il ne mesure rien). Le contour du robot devient rouge quand il pousse contre un mur : il y reste, roues qui patinent, au lieu d'arrêter la simulation. Sous la scène :

| Commande                           | Effet                                                                                  |
| ---------------------------------- | -------------------------------------------------------------------------------------- |
| Recompiler Robot.h et redémarrer   | Comme recompiler et recharger le robot : il redémarre, et la page le voit redémarrer. Si la compilation échoue, l'erreur s'affiche et le robot garde son programme |
| Replacer le robot                  | Le reposer à son point de départ, sans le redémarrer                                   |
| Retirer l'obstacle                 | Retirer ou remettre l'obstacle (scènes `obstacle` et `patrouille`)                     |
| capteur avant / latéral débranché  | Débrancher un capteur, puis le rebrancher                                              |

La ligne du dessous donne l'état de la liaison, les commandes des roues, les deux mesures, la position du centre du robot (en mm) et le temps écoulé depuis le démarrage.

| Option              | Effet                                                                                          |
| ------------------- | ---------------------------------------------------------------------------------------------- |
| `--scene NOM`       | Choisir la scène : `salle` (par défaut), `obstacle`, `mur`, `patrouille`                       |
| `--console`         | Taper les commandes dans le terminal, comme dans `tools/pico.py console` (voir ci-dessous)     |
| `--sans-firmware`   | Ne pas compiler `Robot.h` (voir ci-dessous)                                                    |
| `--start`, `--heading`, `--door`, `-D` | Comme pour `tools/sim.py`                                                   |
| `--http-port 8000`  | Servir la page sur un autre port                                                               |

**Console.** Avec `--console`, chaque ligne tapée dans le terminal part au robot, et chaque ligne qu'il renvoie s'affiche, comme avec le câble USB. La liaison reste active tant que la console est ouverte ; ctrl-D la ferme, comme quitter `tools/pico.py console`, et le robot perd la liaison. La page reste utilisable pendant ce temps : tant qu'elle est ouverte, ses requêtes maintiennent aussi la liaison.

**Sans firmware.** Un `Robot.h` qui ne compile pas empêche le robot simulé de démarrer. `node client/simrobot.js --sans-firmware` sert alors la page avec un robot de remplacement, sans `Robot.h` : il accepte et refuse les mêmes commandes que le robot, mais il invente ses mesures (un mur toujours à 500 mm environ, avec une mesure manquante de temps en temps) et ses roues ne déplacent rien. Il n'a pas de vue `/sim`.

**Note.** Le réseau de l'ordinateur vers lui-même ne coûte presque rien : les délais de la page en simulation ne disent rien de ceux du WiFi.

## Le TP1 en simulation

Les étapes 1 à 4 se font sur le robot. Pour la suite, lancer le robot simulé face à un mur, avec la console :

```sh
node client/simrobot.js --scene obstacle --console
```

puis ouvrir `http://localhost:8080/sim` pour le voir rouler.

1. Étape 5 : taper les commandes dans le terminal. Le robot simulé n'a pas de câblage à vérifier : ses roues tournent toujours dans le bon sens.
2. Étape 6 : envoyer des commandes de plus en plus faibles, `{"left": 60, "right": 60}` par exemple, et suivre la position dans la vue.
3. Étape 7 : avancer par petits pas vers le mur et suivre `front_mm` dans la télémétrie.
4. Étape 9 : après avoir écrit la mission 1, cliquer sur « Recompiler Robot.h et redémarrer » dans la vue, puis taper les deux commandes de l'étape. La même suite se joue d'un coup avec `tools/sim.py`, qui l'envoie par défaut dans la scène `obstacle` :

   ```sh
   python3 tools/sim.py --scene obstacle
   ```

   La ligne `"done":true` est dans `sim/run.log`, et la mesure avant sur `sim/run.svg`.
5. Étape 10 : utiliser la page dans la vue `/sim`. Le choix de `RADIO` dans `Config.h` est sans effet en simulation.

Pour les questions :

- Question 1 : avec un mur assez loin pour ne pas l'atteindre, lire sur le graphique « Commandes des roues » quand les roues s'arrêtent :

  ```sh
  python3 tools/sim.py --scene obstacle --start 3000 --send 0:'{"left":255,"right":255,"duration":60000}'
  ```

- Question 2 : en console, ctrl-D pendant un mouvement, la page fermée. Ou, avec `tools/sim.py`, couper la liaison 1 s après le départ :

  ```sh
  python3 tools/sim.py --scene obstacle --start 3000 --send 0:'{"left":200,"right":200,"duration":3000}' --offline 1:3
  ```

- Question 3 : `ConfigCheck.h` refuse la configuration à la compilation, pour `tools/build.py` comme pour le simulateur.
- Question 4 : en console, la mission 1 active, avancer jusqu'au mur, puis essayer `{"left":-150,"right":-150}`.
- Question 5 : dans la vue `/sim`, maintenir ▲, puis fermer l'onglet sans lâcher le bouton.

## Le TP2 en simulation

1. Étape 1 : lancer `node client/simrobot.js` et ouvrir `http://localhost:8080/sim`. Le robot simulé exécute le `Robot.h` du binôme, avec la mission 1 du TP1.
2. Étape 3 : les missions s'essaient dans la scène `salle`, assez grande pour un carré. « Replacer le robot » le ramène au départ entre deux essais.
   - `avancer-tourner` : le robot simulé tourne, mais la durée d'un quart de tour trouvée en simulation est celle du modèle. La mesurer à nouveau au sol.
   - `avancer-obstacle` : le mur d'en face est à 1,4 m.
   - `carre` : une mission interrompue par un obstacle se teste en lançant le simulateur avec `--scene obstacle`.
3. Étapes 4 et 5 : sur le robot.

Pour les questions :

- Question 1 : le capteur avant simulé perd environ 2 % de ses mesures. Retirer `t.front_ok` de la condition et relancer `avancer-obstacle` plusieurs fois.
- Question 3 : fermer l'onglet pendant une mission, ou arrêter le simulateur par ctrl-C.

## Le TP3 en simulation

Le TP3 se fait contre `node client/simrobot.js`, comme l'indique l'énoncé. Avec le robot simulé :

- Exercice 1 : la courbe de `front_mm` suit le robot qui avance vers un mur, par exemple avec `--scene obstacle`.
- Exercice 2 : les délais sont ceux de l'ordinateur ; voir la note sur le réseau ci-dessus.
- Exercice 3 : mesurer la distance parcourue avec la position affichée sous la vue `/sim`, avant et après chaque mouvement.
- Exercice 4 : en plus de ctrl-C, le bouton « Recompiler Robot.h et redémarrer » fait redémarrer le robot sans arrêter le simulateur.

## Le TP5 en simulation

1. Étape 1 : remplacée par le réglage de `SIDE_SENSOR_ENABLED` ci-dessus.
2. Étape 2 : `node client/simrobot.js --scene mur`, puis le mode impulsion du TP3 dans la vue `/sim`.
3. Étapes 3 et 4 : sans changement. Lancer la version en tout ou rien :

   ```sh
   python3 tools/sim.py --out sim/tout-ou-rien
   ```

4. Étape 5 : mesurer l'amplitude et la période sur `sim/tout-ou-rien.svg`. Pour des valeurs précises, ouvrir le fichier `.csv` dans un tableur.
5. Étape 6 : lancer la version proportionnelle et la comparer à la précédente :

   ```sh
   python3 tools/sim.py --out sim/proportionnel --compare sim/tout-ou-rien.csv
   ```

6. Étape 7 : chercher un gain trop faible et un gain trop fort, en relançant avec `-D` ou en modifiant `Robot.h`.

Pour les questions :

- Question 1 : ajouter `--door 2000:2900` et regarder ce que fait le robot pendant et après l'ouverture.
- Question 2 : après le remplacement de `drive()` par `motors.set()`, lire la ligne `safety` du résumé.
- Question 3 : se traite sur papier, avec les mesures du TP4.

**Watch out.** Un départ loin de la consigne (par exemple `--start 450` pour une consigne de 250 mm) peut finir dans le mur, même avec un gain correct. Regarder `side_mm` pendant que le robot tourne vers le mur : que mesure le capteur quand le robot n'est plus parallèle au mur ?

## Le TP6 en simulation

La mission 4 se simule comme la mission 3. Définir les gains avec `#ifndef`, comme au TP5, permet de les changer avec `-D` sans modifier `Robot.h`.

1. Étape 1 : simuler la mission 3, puis la mission 4 une fois écrite, en superposant les deux :

   ```sh
   python3 tools/sim.py --mission 3 --duration 30 --out sim/p
   python3 tools/sim.py --mission 4 --duration 30 --out sim/pid --compare sim/p.csv
   ```

2. Étapes 2 et 3 : relancer la mission 4 après chaque terme ajouté, avec `--compare` sur la simulation précédente. Pour l'étape 3, calculer dans un tableur la moyenne de `side_mm` sur les 20 dernières secondes de `sim/pid.csv`, et la comparer à la distance voulue.
3. Étape 4 : passer chaque valeur essayée avec `-D`, et garder pour chacune les fichiers produits avec un nom de sortie différent.
4. Bruit du capteur : `python3 tools/sim.py --mission 0 --out sim/immobile` laisse le robot immobile pendant 3 s ; `sim/immobile.csv` contient les mesures de `side_mm` de cette période.

Pour les questions :

- Questions 1 et 2 : à faire sur le robot. La simulation exécute la boucle une fois par milliseconde simulée, quel que soit le code : `dt` y est toujours le même, et un `printf` n'y ralentit rien.
- Question 3 : comparer la moyenne de `side_mm` avec et sans terme I. Refaire la comparaison avec `-D SIM_LEFT_STRENGTH=1`, qui équilibre les moteurs du robot simulé : le terme I sert-il encore ?
- Question 4 : sans changement.

**Note.** Les gains trouvés en simulation ne sont pas ceux du robot. Refaire le réglage de l'étape 4 sur le robot une fois le capteur monté.

## Le TP7 en simulation

La scène `patrouille` remplace la préparation : le robot longe le mur, une boîte se trouve sur son chemin à x = 1,5 m, et un mur en travers ferme le couloir à x = 3,5 m. Sans autre commande, `tools/sim.py` y envoie `{"mission":5}`.

1. Étape 3 : tester les transitions une par une.
   - Obstacle loin, puis approché, puis maintenu en place : `python3 tools/sim.py --scene patrouille`. Le robot doit s'arrêter devant la boîte et y rester.
   - Obstacle retiré : ajouter `--remove 10`. Le robot doit repartir, puis s'arrêter devant le mur du fond.
   - Capteur débranché brièvement : ajouter `--unplug front:3:3.5`, ou `--unplug side:3:3.5`.
2. Étape 4 : arrêter la mission pendant l'arrêt devant la boîte, retirer la boîte, puis relancer la mission. Le robot ne doit repartir qu'à la dernière commande :

   ```sh
   python3 tools/sim.py --scene patrouille --send 0:'{"mission":5}' --send 8:'{"stop":true}' --remove 10 --send 13:'{"mission":5}'
   ```

   Refaire l'essai avec `{"mission":0}`, puis avec une coupure de la liaison au lieu de la commande : `--offline 8:9 --remove 10`.

Les mêmes essais se font à la main avec `node client/simrobot.js --scene patrouille` : bouton `patrouille` de la page, bouton « Retirer l'obstacle » et cases « débranché » de la vue `/sim`. Pour couper la liaison, fermer l'onglet, puis le rouvrir.

Pour les questions :

- Question 1 : sans changement.
- Question 2 : modifier le seuil de reprise et relancer `--scene patrouille --remove 10`. Pour la mesure avant invalide, `--unplug front:6:12` débranche le capteur avant pendant six secondes.
- Question 3 : sans changement.

**Note.** Le capteur avant simulé perd environ 2 % de ses mesures, et le capteur latéral 1 % : sur les graphiques, chaque barre rouge est une mesure invalide. Compter les arrêts que la mission en déduit sur le graphique « Commandes des roues ».
