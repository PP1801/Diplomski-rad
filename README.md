# Diplomski-rad
U ovom radu napravljena je nadogradnja postojećeg prototipa sigurnosnog robota penjača. Nadogradnja uključuje kompletnu izradu i integraciju periferne elektronike na robota penjača (senzori, mikroupravljač, radio modul), izradu programskog koda za praćenje senzora i implementaciju upravljačkog algoritma. Za upravljanje pozicijom robota odabrana je kaskadna struktura sa PI regulatorom brzine vrtnje kotača i P regulatorom položaja, za koje su izračunati parametri prema optimumu dvostrukog odnosa. Izrađeno je i implementirano programsko rješenje za sinkronizaciju gibanja četiri sigurnosna robota penjača, te rješenje za akviziciju mjerenja sa senzora na Teensy 4.0 mikroupravljač. Upravljački algoritmi su verificirani uz pomoć OptiTrack sustava kamera za praćenje pozicije objekata u prostoru. Konačno, identificirana su konstrukcijska rješenja na robotu koja uzrokuju smanjenu preciznost praćenja položaja preko enkodera kotača i senzora optičkog pomaka.

<b></b>&nbsp;&nbsp;Robotski sustav na ispitnom stupu
<br>
<p align="center"><img src="https://raw.githubusercontent.com/PP1801/Diplomski-rad/main/slike_temp/IMG_20230703_124657_c.jpg" width="520px"></p>
<br>

<b></b>&nbsp;&nbsp;Dva robota na horizontalnom ispitnom postavu
<br>
<p align="center"><img src="https://raw.githubusercontent.com/PP1801/Diplomski-rad/main/slike_temp/IMG_20230709_184751.jpg" width="520px"></p>
<br>
