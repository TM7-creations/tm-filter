# Portée de la licence et code de comparaison

La licence MIT du fichier `LICENSE` s’applique aux contributions propres au projet TM IMU Orientation Filter, sous réserve des exclusions ci-dessous. Elle ne remplace aucune licence ni aucun droit d’un tiers.

## Fichiers Madgwick exclus de la licence MIT du projet

- `examples/desktop/madgwick.c`
- `examples/desktop/madgwick.h`

Ces fichiers de comparaison sont exclus de la licence MIT accordée par ce projet tant que la provenance et les conditions de réutilisation du code initial ne sont pas clarifiées. Aucune nouvelle autorisation de réutilisation de ces fichiers n’est accordée par le fichier `LICENSE` du projet.

Le code a été réorganisé à partir du fichier `madgwick.c` fourni avec le prototype. Ce fichier initial ne contenait ni mention de licence ni notice de copyright permettant d’identifier une implémentation d’origine. La référence au rapport scientifique de Sebastian O. H. Madgwick documente l’algorithme ; elle n’établit pas la provenance du code.

Cette exclusion ne règle pas à elle seule la question de leur redistribution. Avant publication de ces fichiers, confirmer s’il s’agit d’une implémentation originale ou d’un code adapté ; dans ce dernier cas, retrouver et respecter la licence ainsi que les notices d’origine.

## Dépendances externes

raylib et les composants Arduino utilisés par les exemples ne sont pas inclus dans les sources de ce dépôt. Ils conservent leurs propres licences ; la licence MIT du projet TM ne les modifie pas.
