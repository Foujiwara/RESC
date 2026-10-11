# Demande pour RESC Tool : reconnaître les choix par leur nom, pas par leur position

## Contexte

Le firmware RESC, en profil RESC, envoie sa propre description de configuration (commande 250). Il retire des listes déroulantes les choix qu'il ne prend pas en charge. Les positions des choix restants changent donc, et le firmware traduit position ↔ valeur interne.

Deux listes ne peuvent pas encore être réduites, parce que RESC Tool choisit les pages affichées d'après la **position** du choix :

| Code | Liste | Test actuel |
|---|---|---|
| `mainwindow.cpp`, `updateMotortype` | `motor_type` | pages BLDC = 0, DC = 1, FOC = 2, GPDrive = 3 |
| `mainwindow.cpp`, `updateAppToUse` | `app_to_use` | PPM = 1, 4, 8 ; ADC = 2, 5, 8, 10 ; UART = 3, 4, 5, 8 ; VESC Remote = 0, 3, 6, 7, 8 ; etc. |
| `setupwizardmotor.cpp`, `boardsetupwindow.cpp`, `setupwizardapp.cpp`, `mobile/SetupWizard*.qml` | `motor_type`, `app_to_use` | `getParamEnum(...)` comparé à des numéros, `updateParamEnum(..., n)` |

Le firmware RESC ne pilote qu'en FOC. Il n'a ni Nunchuk I2C, ni NRF24, ni application personnalisée. Ces choix doivent disparaître de la liste.

## À faire dans RESC Tool

1. **Décider à partir du texte du choix**, et non de sa position : `getParamEnumNames(name)[getParamEnum(name)]`. On peut aussi déduire la page de chaque nom (par exemple « FOC » → page FOC ; un nom qui contient « PPM » → page PPM ; « ADC » → page ADC ; « UART » → page UART ; « PAS » → page PAS).
2. **Pour écrire une valeur**, chercher la position du nom voulu (`indexOf("UART")`) au lieu d'écrire un numéro fixe.
3. **Garder le comportement actuel** quand la description intégrée VESC 7.00 est utilisée (VESC officiel).
4. **Masquer une page ou un onglet vide** (par exemple Nrf, IMU, BLDC, DC, GPDrive quand tous leurs paramètres sont retirés).

## Côté firmware, ensuite

Dès que la version de RESC Tool qui fait cela est publiée, le firmware réduira `motor_type` à « FOC » et `app_to_use` aux applications réelles : `ENUM_KEEP` dans `tools/resc_conf/make_resc_xml.py`, une ligne chacune.
