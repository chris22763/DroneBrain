# Dronebrain

Dronebrain ist ein Projekt dessen Ziel es ist ein künstliche Intelligenz zu kreieren die eine komplexe Aufgabe durch geschicktes delegieren von Arbeitern effizient lösen kann.

Für für den Prototypen wird sich auf Flugdrohnen konzentriert.
Deren "Brains" basieren auf der Aaeon UP-Board Plattform, mit einer Intel RealSense Kamera und Sensoren zur Orientierung im Raum.
Diese Kombination soll es ermöglichen alle benötigten Daten zur Erfüllung der Testaufgaben an einen Hauptrechner zu schicken der diese Verarbeitet und basierend darauf entscheidet was als nächstes ansteht.

Die UP-Boards kommunizieren über wifi und übertragen alle gesammelten Daten in einer kompakten raw-Bilddatei. Diese wird dann als Byte-Stream verschickt.
Das speichern der gesamten Daten in einer Datei zusammen mit dem Bildern wurde von der Stenographie inspiriert, aber wegen der nicht vorhandenen Notwendigkeit für eine Verschlüsselung werden die Daten in RGB werte konvertiert und dem Bild als zusätzliche Pixelreihe angehängt.
Zur dekodierung werden vorher definierte flags gesetzt. Diese flags sind werte die durch die Codierung nicht erreicht werden können und sind somit eineindeutig.
