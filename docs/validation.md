# Vérifications de la version réorganisée

Vérifications effectuées le 4 septembre 2026 sur macOS ARM64, avec le compilateur C fourni par Zig 0.16.0, utilisé temporairement dans le dossier de travail.

## Vérifié

- Configuration, construction et exécution CTest via CMake 4.4.3 et Ninja pour les deux valeurs par défaut FAST et EXACT.
- Compilation C99 avec `-Wall -Wextra -Wpedantic -Werror`, sans avertissement.
- Exécution de la suite `test_filter.c` pour les configurations par défaut FAST et EXACT : **21 900 assertions réussies par configuration**. Ce nombre inclut des boucles de contrôle et ne désigne pas 21 900 scénarios indépendants.
- Rotations connues et comparaison de l’intégrateur exact avec un calcul indépendant en double précision.
- Non-équivalence de l’intégration exacte et d’Euler normalisé pour un pas fini.
- Conservation de la norme, rotation nulle et très petites vitesses.
- Contrainte verticale et optimalité de la projection sur 500 orientations/directions déterministes, dont les six axes et les pôles.
- Équivalence des représentations `q` et `−q`, fusion et entrées invalides.
- Tests de norme et de variation de direction de l’accélération, convergence statique.
- Régressions Madgwick : le gradient nul et l’accélération nulle ne figent plus le gyroscope.
- Décodage de trames série successives, bruit de synchronisation, valeurs non finies et récupération du flux.
- Compilation du fichier de visualisation en objet avec l’en-tête officiel raylib 5.5.
- Inclusion de l’API publique dans un programme C++17.

L’exemple minimal donne après 1 seconde de rotation à 90 °/s autour de Z :

```text
q [w,x,y,z] = [0.707107, 0.000000, 0.000000, 0.707106]
```

## Non vérifié sur matériel

- Téléversement du sketch et fonctionnement électrique sur une carte Arduino réelle.
- Liaison complète et exécution de la fenêtre raylib avec le capteur connecté.
- Précision absolue, dérive à long terme et temps de calcul sur microcontrôleur.

Les fichiers de tests et une configuration GitHub Actions sont inclus. Le lancement de GitHub Actions nécessitera la publication du dépôt ; aucun résultat de CI distante n’est revendiqué ici.
