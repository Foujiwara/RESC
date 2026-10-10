# Audit du code et indépendance du cœur FOC

Date de l'audit : 10 octobre 2026  
Référence comparée : firmware VESC `release_7_00`, commit
`20cbb362687291242ab90b99f25fbfe8835540fc`, dépôt officiel
<https://github.com/vedderb/bldc/tree/release_7_00>.

## Conclusion

Le cœur moteur RESC ne présente pas les marqueurs d'une copie textuelle du cœur
FOC de VESC. La comparaison de `firmware/src/core/` avec les fichiers C et H de
VESC 7.00 n'a trouvé, dans `foc.c`, `observer.c`, `angle_sensors.c` et `ctrl.c` :

- aucune ligne non triviale identique après retrait des commentaires et espaces ;
- aucune séquence identique de 12 jetons C consécutifs ;
- aucune séquence identique de 20 jetons C consécutifs.

À huit jetons, `foc.c` ne partage que 2 séquences avec `mcpwm_foc.c` (0,02 % des
séquences locales), ce qui est compatible avec le bruit attendu dans deux
programmes C de commande moteur. L'architecture et les noms internes divergent
aussi nettement : le cœur RESC est réparti entre quatre modules courts, alors que
le seul `motor/mcpwm_foc.c` de VESC compte 5 419 lignes et dépend notamment de
`foc_math.c` et d'un état moteur beaucoup plus large.

Ce résultat est un indice technique fort d'une implémentation indépendante. Il
ne constitue toutefois ni une preuve historique absolue ni un avis juridique :
le dossier fourni ne contient pas le répertoire `.git`, donc l'historique de
création, les auteurs et les dates ne peuvent pas être vérifiés.

Le dépôt complet n'est en revanche pas indépendant de VESC. Il contient des
éléments de compatibilité et des données dérivées qui doivent être attribués et
licenciés correctement. VESC 7.00 est annoncé sous GPLv3 par son dépôt officiel.

## Périmètre et méthode

L'audit a porté sur :

- le cœur moteur : `firmware/src/core/*.c` et `*.h` ;
- le chemin matériel critique : `firmware/src/bsp/motor_hw.c` ;
- la configuration et les commandes : `firmware/src/app/`,
  `firmware/src/comm/` et `firmware/src/vesc/` ;
- les cartes et limites matérielles dans `hw/` ;
- la compilation ARM et la simulation hôte.

La comparaison a retiré commentaires et espaces, tokenisé le C, puis cherché des
lignes identiques et des n-grammes de 8, 12 et 20 jetons contre les sources C/H
VESC. Une recherche distincte a été faite sur les couches de compatibilité.

Limites : la méthode détecte bien le copier-coller inchangé ou légèrement
reformaté. Elle ne peut pas exclure une réécriture complète avec renommage de
tous les symboles. Les équations Clarke/Park, PI d/q, SVPWM min-max et
observateurs de flux sont par ailleurs des techniques publiques et leur présence
commune n'est pas, seule, un indice de copie.

## Résultats sur la provenance

### Cœur FOC

Les ressemblances conceptuelles avec VESC sont celles attendues pour deux
contrôleurs FOC : transformations alpha/bêta et d/q, régulation PI, observateur
de flux, PLL, SVPWM et traitement Hall/codeur. Leur expression concrète est
différente : structures, découpage, machine d'états, démarrage I/f, relais vers
l'observateur et identification moteur ne suivent pas le texte de VESC.

Verdict : **pas d'indice de copie du code FOC VESC dans le cœur RESC**.

### Éléments VESC hors cœur FOC

La situation est différente dans la compatibilité :

- `firmware/src/vesc/vesc_defaults.h` partage 1 643 n-grammes de 16 jetons avec
  `datatypes.h` de VESC, soit 41,81 % de ses séquences locales ;
- `firmware/src/vesc/vesc_conf.h` en partage 147, soit 13,75 % ;
- le dépôt annonce lui-même la reprise du protocole, du format de configuration
  VESC Tool et de nombreuses définitions de cartes ;
- aucun fichier `LICENSE`, `COPYING` ou `NOTICE` n'est présent dans le dossier
  audité, et les fichiers dérivés ne portent pas l'en-tête GPL/copyright VESC.

La phrase du README « Seuls les protocoles CAN et UART de VESC sont repris » est
donc trop restrictive : les structures/défauts de configuration et des données
matérielles VESC sont également repris ou dérivés.

Action requise avant diffusion : faire examiner le périmètre dérivé, ajouter la
licence GPLv3 et les attributions nécessaires, documenter précisément les
fichiers générés/importés et publier le code source correspondant aux binaires.

## Constats de sûreté et de qualité

### [Critique] Protection surintensité aveugle pendant certaines fenêtres PWM

Sur la carte Ubox auditée, `BOARD_BKIN_PIN` vaut `HW_PIN_NONE` et
`BOARD_HW_OCP` vaut 0. La protection indépendante n'est donc pas disponible.
Dans `foc.c`, seules les phases dont l'échantillon est déclaré valide sont
comparées au seuil. Si moins de deux mesures sont valides, le courant est prédit
par le modèle. Dans `motor_hw.c`, le watchdog analogique de chaque phase est lui
aussi désarmé lorsque la fenêtre low-side est insuffisante.

Conséquence : un défaut de courant sur une phase non mesurable peut rester
invisible pendant une ou plusieurs périodes, précisément sur une carte sans
coupure surintensité matérielle indépendante. La simulation actuelle ne modélise
pas ce cas de commutation/défaut.

Recommandation : ne pas faire d'essai de puissance sans limitation externe et
coupure matérielle indépendante. Ajouter une stratégie garantissant au moins
deux mesures valides, une limitation de modulation adaptée, des tests de défaut
aux limites de fenêtre et, idéalement, une vraie entrée break issue d'un
comparateur matériel.

### [Élevé] Une configuration reçue en marche désynchronise `dt` et le PWM réel

`COMM_SET_MCCONF` appelle `resc_conf_apply()` sans imposer l'arrêt. Quand le
moteur est en RUN/START, `resc_conf_apply()` évite de changer TIM1 mais transmet
quand même les nouveaux paramètres au cœur FOC. À l'ISR suivante,
`apply_params()` remplace `S.dt` par l'inverse de la nouvelle fréquence demandée,
alors que le timer continue à l'ancienne fréquence.

Conséquence : intégrateurs, observateur, rampes, fenêtres de mesure et délais
emploient un pas de temps faux ; les autres paramètres peuvent également changer
brutalement sous couple.

Recommandation : refuser ou différer atomiquement toute configuration moteur
jusqu'à l'état IDLE, puis changer le timer et le bloc complet de paramètres dans
une transaction unique.

### [Élevé] La régulation lente ignore la source d'angle réellement sélectionnée

Le chemin RUN choisit correctement `S.theta` et `S.omega` depuis l'observateur,
les Hall ou le codeur. La télémétrie publiée à la fin de l'ISR réécrit cependant
toujours `theta` et `omega` depuis `S.obs`. `ctrl_update()` utilise ces champs
pour la vitesse, le freinage, le PID RPM et le tachymètre.

Conséquence : en mode Hall/codeur, surtout à basse vitesse lorsque l'observateur
sensorless est peu fiable, la boucle lente peut calculer le signe de freinage et
l'erreur de vitesse avec une source différente de celle qui pilote le FOC.

Recommandation : publier `S.theta`/`S.omega` comme angle/vitesse de commande et,
si nécessaire, exposer séparément les valeurs brutes de l'observateur.

### [Élevé] Les trames CAN moteur ne vérifient pas toujours leur DLC

Les commandes CAN `SET_DUTY`, `SET_CURRENT`, `SET_CURRENT_BRAKE`, `SET_RPM` et
plusieurs commandes relatives lisent quatre octets sans vérifier `fr->len >= 4`.
Les commandes de limites lisent aussi leurs champs sans contrôle de longueur
adapté.

Conséquence : une trame CAN tronquée adressée au contrôleur peut être interprétée
à partir d'octets hors DLC et créer une consigne ou une configuration imprévue.

Recommandation : vérifier exactement la longueur minimale de chaque commande
avant toute lecture et ajouter des tests avec DLC 0 à 8 pour chaque identifiant.

### [Élevé] Réarmement automatique sans validation de la cause

Après 0,5 s avec une consigne relâchée, `ctrl_update()` demande le réarmement de
presque tous les défauts. Dans l'état FAULT, `foc_isr()` efface alors le défaut
sans vérifier que la surtension, sous-tension, température ou ligne driver a
réellement disparu. Une protection mesurée à chaque ISR peut retomber en défaut
au cycle suivant, mais une faute driver déclenchée sur front peut ne pas être
redétectée tant que la ligne reste active.

Recommandation : rendre certains défauts verrouillés jusqu'à une action explicite
et vérifier chaque condition physique avant d'autoriser le réarmement.

### [Moyen] Les valeurs flottantes non finies ne sont pas rejetées

Le bornage de configuration utilise des comparaisons ordinaires. Avec un NaN,
les comparaisons sont fausses et `fm_sat()` renvoie le NaN inchangé. Une valeur
non finie provenant d'une configuration reçue peut donc atteindre fréquence,
limites, gains ou rapports cycliques.

Recommandation : valider `isfinite()` et les domaines de tous les champs externes
avant de modifier la configuration active.

### [Moyen] Validation encore insuffisante pour annoncer « prêt au banc »

La compilation ARM réussit et les 12 scénarios de simulation passent. L'analyse
statique ne remonte pas d'erreur directe dans le cœur FOC, hormis une affectation
inutile ; ses autres alertes concernent surtout le démarrage bas niveau et une
lecture avant test de borne dans le descripteur USB.

Ces résultats ne couvrent toutefois ni le timing réel ADC/TIM1, ni les défauts
de grille, ni les transitoires bus, ni les fenêtres low-side extrêmes. Le README
indique lui-même que le firmware n'a jamais tourné sur la carte réelle. Le statut
approprié reste donc « prototype simulé, essai basse énergie à préparer », pas
« prêt pour un essai de puissance ».

## Vérifications exécutées

- `python firmware/build.py --sim` : 12 scénarios, tous réussis ;
- `python firmware/build.py` : compilation STM32F405 réussie, chemin FOC en RAM
  confirmé ;
- occupation : 98 520 octets de flash, 44 696 octets de RAM et 1 248 octets de
  CCM d'après le script de construction ;
- `cppcheck` : aucune erreur fonctionnelle détectée dans le cœur FOC ; alertes
  à trier dans le démarrage et l'USB ;
- comparaison lexicale contre le commit VESC précisé en tête de document.

## Priorités avant essai matériel

1. Ajouter une coupure surintensité matérielle indépendante ou limiter l'essai à
   une source de laboratoire fortement limitée en courant.
2. Corriger l'application de configuration en marche et la télémétrie de la
   source d'angle.
3. Valider tous les DLC CAN et toutes les valeurs flottantes reçues.
4. Revoir la politique de réarmement des défauts.
5. Régulariser licence, attribution, historique et nomenclature des composants
   logiciels avant toute diffusion.

## Suivi des constats (après l'audit)

| Constat | Suite donnée |
|---|---|
| Licence et attribution | `LICENSE` (GPLv3) et `NOTICE.md` ajoutés ; README précisé sur ce qui est repris de VESC |
| Surintensité aveugle hors fenêtres | **atténué** : la modulation est bornée pour garder en permanence deux phases mesurables (`resc_conf_build`). **Reste ouvert** : pas de coupure matérielle indépendante sur l'Ubox |
| Configuration appliquée en marche | **corrigé** : seules les limites s'appliquent en marche ; fréquence PWM, source d'angle et paramètres du cœur attendent l'arrêt (`resc_conf_poll`) |
| Télémétrie et source d'angle | **corrigé** : `theta` et `omega` publient l'angle et la vitesse de commande ; l'observateur seul est publié à part (`obs_theta`, `obs_omega`) |
| Longueur des trames CAN | **corrigé** : longueur minimale vérifiée pour chaque commande |
| Réarmement automatique | **corrigé** : réarmement seulement si la cause a disparu ; défauts driver, break, offsets et watchdog verrouillés jusqu'à `clear`, qui est refusé si la ligne de défaut du driver est active |
| Valeurs non finies | **corrigé** : NaN et infinis remplacés par les valeurs par défaut pour tous les champs utilisés |
| Statut « prêt au banc » | premier essai réel réalisé à basse énergie (connexion, rotation après détection du flux) ; la validation continue |

