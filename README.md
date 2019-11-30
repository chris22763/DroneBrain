# Dronebrain

Dronebrain ist ein Projekt dessen Ziel es ist eine künstliche Intelligenz zu kreieren die eine komplexe Aufgabe durch geschicktes delegieren von Arbeitern effizient lösen kann.

Für den Prototypen wird sich auf Flugdrohnen konzentriert.
Deren "Brains" basieren auf der Nvidia Jetson Nano Plattform, mit einer Intel RealSense Kamera und Sensoren zur Orientierung im Raum.
Diese Kombination soll es ermöglichen alle benötigten Daten zur Erfüllung der Testaufgaben an einen Hauptrechner zu schicken der diese Verarbeitet und basierend darauf entscheidet was als nächstes ansteht.

Um dem Namen "DroneBrain" gerecht zu werden wurden in /the-real-skynet/drone_controller, dem Projekt Ordner der Flugsteuerung, die Klassen nach Arealen des menschlichen Gehirns benannt. 
Dabei wurde auch versucht die Areale passend zu ihren Aufgaben auszuwählen. So ist der Name der Flugsteuerung Cerebelum (Kleinhirn) dem hauptsächlich der Gleichgewichtssinn und die motorische und koordinatorische rolle zu gesprochen wird. 
Die Main Klasse ist dem entsprechend des Stammhirns, oder Truncus Cerebri, der älteste Teil des Codes und sozusagen das zu beginn stehende Leitwerk. 
