# Diplomski-rad
U ovom radu napravljena je nadogradnja postojećeg prototipa sigurnosnog robota penjača. Nadogradnja uključuje kompletnu izradu i integraciju periferne elektronike na robota penjača (senzori, mikroupravljač, radio modul), izradu programskog koda za praćenje senzora i implementaciju upravljačkog algoritma. Za upravljanje pozicijom robota odabrana je kaskadna struktura sa PI regulatorom brzine vrtnje kotača i P regulatorom položaja, za koje su izračunati parametri prema optimumu dvostrukog odnosa. Izrađeno je i implementirano programsko rješenje za sinkronizaciju gibanja četiri sigurnosna robota penjača, te rješenje za akviziciju mjerenja sa senzora na Teensy 4.0 mikroupravljač. Upravljački algoritmi su verificirani uz pomoć OptiTrack sustava kamera za praćenje pozicije objekata u prostoru. Konačno, identificirana su konstrukcijska rješenja na robotu koja uzrokuju smanjenu preciznost praćenja položaja preko enkodera kotača i senzora optičkog pomaka.

<b>Step 1.</b>&nbsp;&nbsp;Creating the dataset
<br>
<p align="center"><img src="https://raw.githubusercontent.com/PP1801/slike_temp/images%20for%20GitHub/IMG_20230703_124657_c.jpg" width="420px"></p>
<br>
