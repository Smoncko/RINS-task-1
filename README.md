# RINS-task-1



## Pomembno:
### 
- sprejemanje slik med vožnjo in uporaba detected_faces.py zadeve, da zazna obraz na sliki in nam da njegovo lokacijo (pomoje je rešitev kar subscription na detect_faces.py in potem pretvorba tega v usable koordinate - nisem pa si dovolj pogledal)
- postavit marker tja al nekaj takega - pač zapomknit si, kje je (transform_points.py zgled)

- it pred ta marker (prepend v navigation_list)
- obrnit se proti sliki (glede na našo lokacijo in lokacijso slike izračunamo spin. Prepend tega cilja v navigation list.)
- rečt živjo
- iti nazaj na zadnji goal na poti, ki smo ga preden smo šli do slike še imeli.



Postransko:
- levo zgoraj (ob koncu predpisane poti) se robot zatakne. Spremenit bi bilo treba severity levels ste al kaj ze, pomoje nekaj takega. Da si bo pac upal it skoz.
- spreminjanje hitrosti (ima prehitrost vpliv na zaznavo? Če nima, bi šli morda lahko hitreje? Zdi se, da bi se ta max_velocity dal v config/nav2.yaml naštimat)
- se da narediti, da pri navigiranju pose objectu ne damo fi-ja, in pač pusti robota v katerikoli smeri je pac obrnjen ko pride do ciljne točke? In ga mi potem recimo obrnemo za 360, da pogledamo za obraze, in gre potem naprej - ker zdaj se ze toliko obraca, da ga je prav wasteful it 360 obracat.
- either way, lahko bi rocno dodali obracanja za vsako mesto postanka posebej in bi se actually obrnil proti stenam, ne vseh 360.