 Code source des robots d'UTCoupe 2018
=======

# Configuration

Avant toute chose, il faut cloner le répertoire sur votre ordinateur :
```
git clone git@github.com:utcoupe/coupe18.git
```

### Configurer l'environnement de développement

Un script d'installation automatique est disponible. Allez dans le dossier coupe18, et lancer simplement :
```
./scripts/install_utcoupe_setup.sh
```

Si c'est votre première installation, répondez "y" à toutes les questions.

### Compiler le système

Tout le système de cette année repose sur ROS (http://www.ros.org/), il faut donc compiler le système après l'avoir récupéré et configuré :
```
cd coupe18/ros_ws
catkin_make
```

# Lancement

//TODO

ou must then source the workspace with `source devel/setup.bash` or `source devel/setup.zsh` each
time you open a new terminal. Adding this line to your `~/.bashrc` or `~/.zshrc` (with the full
path to the setup file) will simply automate this step.

# Règles et Guidelines

Afin d'avoir un projet organisé et fonctionnel, voici quelques règles (par convention ou importantes pour le 
fonctionnement du projet) à suivre pour la création de branches git, paquets, noeuds ros, etc :

### Git

- Créer des branches sur git de la forme `namespace/package` si la branche correspond à un paquet ROS. (e.g. `ai/scheduler`, `memory/map`, etc)

### Paquets ROS

- Créer des paquets ROS nommés de la forme `namespace_package` (utile une fois qu'ils seront tous ensemble, ils seront ordonnés par
ordre alphabétique : plus visuel)

- Créer des serveurs de `topics`/`services`/`actions` nommés de la forme `/namespace/package/server_name` s'ils peuvent être accédés par des paquets 
extérieurs (ATTENTION : avec un `/` au début pour créer un nom absolu), `server_name` s'ils sont internes.

- Nommer les fichiers de définition `.msg`/`.srv`/`.action` en PascalCase (e.g. `GetValues.srv`) et les variables dedans en minuscules (format `var_name`).

### Python

- Afin de respecter le PEP8 : 4 espaces d'intentation (et non tabs).

### Données

- Unités de distance en mètres, transportées par des `float32`.

- Lors de la description d'une position d'une forme (cercle, ligne, rectangle, point...), donner la position par rapport au centre de la forme (sauf précision explicite et nécessaire). Par exemple, donner la position du centre d'un rectangle et non d'un coin.

# Webclient

Pour installer les dépendances du webclient :
```
cd webclient
npm install --only=prod
```

Pour lancer le webclient :
```
npm start
```

S'assurer que le noeud ROS `rosbridge_server` est bien lancé.

Le webclient peut être lancé depuis le robot, ou depuis un ordinateur connecté à la Raspberry.

Si le serveur est lancé sur le robot, se rendre sur `http://<ip_de_la_raspi>:8080`.

Sinon, se rendre sur [http://localhost:8080](http://localhost:8080) et vérifier que le client se connecte bien à l'IP du robot dans les paramètres.
