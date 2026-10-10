# 8. Informations matérielles manquantes

> **Mise à jour :** le firmware cible désormais les cartes VESC existantes, en premier la **Ubox SINGLE 85_200**. Le brochage, les shunts, les gains, les diviseurs, les NTC, le dead-time et les limites sont repris des fichiers VESC (doc 11) : **le schéma n'est plus nécessaire** pour ces points. Il reste à obtenir ou mesurer les éléments du §8.0. Les tableaux suivants restent la liste de contrôle pour une carte qui ne viendrait pas de VESC.

## 8.0 Ce qui reste à confirmer pour l'Ubox 85_200

| # | Point | Pourquoi | Comment |
|---|---|---|---|
| U1 | Références des gate drivers et MOSFET, désaturation ou OCP interne éventuelle | Savoir s'il existe une protection de niveau 0 (doc 07 §7.8) | Lecture des composants sur la carte |
| U2 | Mesure de T_win (établissement de l'ampli après commutation) | f_PWM max, m_max (doc 04) | Oscilloscope sur la sortie d'ampli, phase 3 |
| U3 | VDDA réelle (VESC indique 3,44 V) | Gain de toutes les mesures | Multimètre |
| U4 | Diviseur des tensions de phase (= diviseur Vbus ?) | Observateur, reprise au vol | Mesure à pont arrêté avec tension connue |
| U5 | PB12 accessible (pour un éventuel BKIN, option A) | Protection matérielle | Inspection du PCB |
| U6 | Moteur(s) d'essai : R, Ld, Lq, ψ, paires de pôles, courant max | Réglages, sécurité | Fiche moteur + identification en phase 6 |
| U7 | Application, batterie, BMS | Comportement en défaut, régénération | Utilisateur |
| U8 | Version de VESC Tool et équipements CAN à interfacer | Validation du protocole | Utilisateur |

Pour une carte **hors VESC** : sans les informations ci-dessous, il est impossible de fixer les seuils de protection, la fréquence PWM, les fenêtres ADC et les limites absolues. Les points 🔴 sont **bloquants** avant toute mise sous puissance.

## 8.1 Schéma et PCB

| # | Information | Pourquoi | Prio |
|---|---|---|---|
| 1 | Schéma complet + brochage MCU | Valider le multiplexage (doc 1 §1.3), BKIN, ADC simultanés | 🔴 |
| 2 | Le BKIN est-il câblé, et à quelles sources ? Avec quel filtrage RC ? | Existence de la protection absolue | 🔴 |
| 3 | Pull-up/pull-down des entrées du driver et de l'EN | État pendant le reset et le boot | 🔴 |
| 4 | Capteur de courant bus DC ? Capteur de tension de phase ? | Angle mort high-side (doc 7 §7.1), reprise au vol | 🔴 |
| 5 | Layout de la maille de puissance (inductance parasite estimée), snubbers | Surtension à la coupure, ringing (T_win) | 🟠 |

## 8.2 Étage de puissance

| # | Information | Prio |
|---|---|---|
| 6 | Référence des MOSFET (R_DS(on)(T), I_D, **I_DM impulsionnel et durée**, Q_g, tenue en avalanche, SOA) | 🔴 |
| 7 | Nombre de MOSFET en parallèle par interrupteur | 🔴 |
| 8 | Gate driver : référence, délais de propagation, courants de grille, **V_DS monitoring / DESAT (seuil, blanking)**, UVLO, sortie nFAULT, mode de défaut (latch ou auto-retry), polarité des entrées, interface SPI éventuelle | 🔴 |
| 9 | Bootstrap ou alimentation isolée (contrainte d = 100 %, durée d'ASC possible) | 🟠 |
| 10 | Capacité de bus (valeur, ESR, courant d'ondulation admissible) | 🟠 |
| 11 | Tension bus min/max absolue (tenue des MOSFET, condensateurs, régulateurs) | 🔴 |
| 12 | Hacheur ou résistance de freinage présents ? | 🟠 |

## 8.3 Mesure

| # | Information | Prio |
|---|---|---|
| 13 | Valeur des shunts, tolérance, coefficient de température, puissance | 🔴 |
| 14 | Ampli de shunt : référence, gain, bias (unidirectionnel/bidirectionnel), **bande passante et temps d'établissement**, slew rate, plage de sortie | 🔴 |
| 15 | Référence de tension ADC (VREF+ = VDDA ?), précision | 🟠 |
| 16 | Diviseurs de tension (bus, phases) : ratios et constantes de temps des filtres | 🟠 |
| 17 | Capteurs de température (type NTC, β, montage, position) : FET, moteur | 🟠 |
| 18 | **Mesure à l'oscilloscope** du signal de shunt autour d'une commutation (ringing, temps d'établissement) → T_win | 🔴 (à faire en phase 3) |

## 8.4 Moteur(s) cible(s)

| # | Information | Prio |
|---|---|---|
| 19 | Type (SPM/IPM, inrunner/outrunner/hub), nombre de paires de pôles | 🔴 |
| 20 | R phase, Ld, Lq (et leur variation avec le courant), ψ_PM (et variation thermique) | 🔴 |
| 21 | Courant max continu/crête, courant de démagnétisation | 🔴 |
| 22 | Vitesse max mécanique, ERPM max → f_e max | 🔴 |
| 23 | Inertie (moteur + charge typique) | 🟠 |
| 24 | Capteurs présents (halls, codeur, type) | 🟠 |

## 8.5 Système

| # | Information | Prio |
|---|---|---|
| 25 | Application (trottinette, vélo, kart, robotique…) → profils de charge, exigences de freinage, comportement de sécurité attendu (roue libre ou freinage en cas de défaut) | 🔴 |
| 26 | Batterie : chimie, tension, courant de charge max, présence d'un BMS qui peut ouvrir le circuit | 🔴 |
| 27 | Équipements VESC à interfacer (display, télécommande, maître CAN, version de VESC Tool) | 🟠 |
| 28 | Mono ou bi-moteur (TIM8) ? | 🟠 |
| 29 | Bootloader : souhaite-t-on la mise à jour via VESC Tool (impose la disposition flash VESC) ? | 🟠 |
| 30 | Exigences de sécurité fonctionnelle visées (aucune, ou inspiration ISO 13849/26262 sans certification) | 🟢 |
| 31 | Licence souhaitée du firmware (impact de la réimplémentation du protocole GPLv3) | 🟢 |
