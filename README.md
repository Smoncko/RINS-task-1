# RINS-task-1

## DONE:
- osnovna navigacija po mapi (robot_commander.py)
- zaznavanje obrazov (detect_people.py in transform_point.py), markerji publishani na topic /detected_faces

## TO-DO:
- obrnit se proti sliki (glede na našo lokacijo in lokacijso slike izračunamo spin. Prepend tega cilja v navigation list.)
- rečt živjo
- iti nazaj na zadnji goal na poti, ki smo ga preden smo šli do slike še imeli.

## Narejeno:
- potovanje po prostoru
- predvajanje zvoka



### Postransko:
- narediti da say_hi() deluje tudi če zaženemo iz drugje kot ~/ros_ws
- levo zgoraj (ob koncu predpisane poti) se robot zatakne. Spremenit bi bilo treba severity levels ste al kaj ze, pomoje nekaj takega. Da si bo pac upal it skoz.
- spreminjanje hitrosti (ima prehitrost vpliv na zaznavo? Če nima, bi šli morda lahko hitreje? Zdi se, da bi se ta max_velocity dal v config/nav2.yaml naštimat)
- se da narediti, da pri navigiranju pose objectu ne damo fi-ja, in pač pusti robota v katerikoli smeri je pac obrnjen ko pride do ciljne točke? In ga mi potem recimo obrnemo za 360, da pogledamo za obraze, in gre potem naprej - ker zdaj se ze toliko obraca, da ga je prav wasteful it 360 obracat.
- either way, lahko bi rocno dodali obracanja za vsako mesto postanka posebej in bi se actually obrnil proti stenam, ne vseh 360.
