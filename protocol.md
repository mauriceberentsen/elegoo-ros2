````markdown
# Command Protocol

## N = 1 — Motorbesturing (Eenvoudig)
```json
{
  "N": 1,
  "D1": <motor keuze: 0=beide, 1=links, 2=rechts>,
  "D2": <snelheid: 0..250>,
  "D3": <richting: 0=stop, 1=vooruit, 2=achteruit>
}
````

**Voorbeeld:**

```json
{ "N": 1, "D1": 0, "D2": 100, "D3": 1 }
```

* Beide motoren, snelheid=100, vooruit.

---

## N = 2 — Auto Besturing (Tijdslimiet)

```json
{
  "N": 2,
  "D1": <auto richting: 1=links vooruit, 2=rechts vooruit, 3=vooruit, 4=achteruit>,
  "D2": <snelheid: 0..255>,
  "T": <tijd in ms>
}
```

**Voorbeeld:**

```json
{ "N": 2, "D1": 3, "D2": 150, "T": 3000 }
```

* Beweeg vooruit met snelheid=150 voor 3000 ms, stop daarna en keer terug naar de "programmerings" staat.

---

## N = 3 — Auto Besturing (Geen Tijdslimiet)

```json
{
  "N": 3,
  "D1": <auto richting: 1=links vooruit, 2=rechts vooruit, 3=vooruit, 4=achteruit>,
  "D2": <snelheid: 0..255>
}
```

**Voorbeeld:**

```json
{ "N": 3, "D1": 4, "D2": 130 }
```

* Beweeg achteruit met snelheid=130 totdat een nieuw commando komt.

---

## N = 4 — Motorbesturing (Snelheidsmodus)

```json
{
  "N": 4,
  "D1": <linkse snelheid: 0..255>,
  "D2": <rechtse snelheid: 0..255>
}
```

**Voorbeeld:**

```json
{ "N": 4, "D1": 80, "D2": 200 }
```

* Linkse motor=80, rechtse motor=200, beide vooruit.

---

## N = 5 — Servo Besturing (Exacte Hoek)

```json
{
  "N": 5,
  "D1": <servo index>,
  "D2": <hoek: 0..180>
}
```

**Voorbeeld:**

```json
{ "N": 5, "D1": 1, "D2": 90 }
```

* Beweeg servo #1 naar 90°.

---

## N = 7 — Verlichtingsbesturing (Tijdslimiet)

```json
{
  "N": 7,
  "D1": <welke lichten: 0=alle, 1=links, 2=voor, 3=rechts, 4=achter, 5=centrum>,
  "D2": <rood: 0..255>,
  "D3": <groen: 0..255>,
  "D4": <blauw: 0..255>,
  "T": <tijd in ms>
}
```

**Voorbeeld:**

```json
{ "N": 7, "D1": 0, "D2": 255, "D3": 0, "D4": 0, "T": 3000 }
```

* Zet alle LED's op rood voor 3 seconden, daarna terug naar de oorspronkelijke staat.

---

## N = 8 — Verlichtingsbesturing (Geen Tijdslimiet)

```json
{
  "N": 8,
  "D1": <welke lichten>,
  "D2": <rood>,
  "D3": <groen>,
  "D4": <blauw>
}
```

**Voorbeeld:**

```json
{ "N": 8, "D1": 4, "D2": 255, "D3": 255, "D4": 0 }
```

* Zet de achterlichten op geel, onbeperkt.

---

## N = 21 — Ultrasoon Module Status

```json
{
  "N": 21,
  "D1": <1=controleer obstakel, 2=afstand in cm>
}
```

**Voorbeeld:**

```json
{ "N": 21, "D1": 2 }
```

* Robot antwoordt met iets als `{_XX}` waarbij XX de gemeten afstand in cm is.

---

## N = 22 — Trace (Lijnvolg) Sensor Status

```json
{
  "N": 22,
  "D1": <0=links sensor, 1=centraal, 2=rechts>
}
```

**Voorbeeld:**

```json
{ "N": 22, "D1": 0 }
```

* Robot geeft de ruwe analoge waarde van de linker lijnsensor terug.

---

## N = 23 — Controleer of de Auto van de Grond is

```json
{ "N": 23 }
```

* Robot antwoordt met `{_true}` als de auto op de grond staat, of `{_false}` als hij is opgetild.

---

## N = 100 — Stop Alle Functies → Standby

```json
{ "N": 100 }
```

* Stopt onmiddellijk alles en gaat naar de standby modus.

---

## N = 110 — Stop Alle Functies → Programmeren Modus

```json
{ "N": 110 }
```

* Stopt alles, maar zet de interne “programmerings” modus aan.

---

## N = 101 — Schakel naar een Autonome Modus

```json
{
  "N": 101,
  "D1": <1=lijnvolg, 2=obstakel-vermijd, 3=volg>
}
```

**Voorbeeld:**

```json
{ "N": 101, "D1": 2 }
```

* Zet de robot in de obstakel-vermijd modus.

---

## N = 102 — Rocker (Joystick) Auto Beweging

```json
{
  "N": 102,
  "D1": <1=vooruit, 2=achteruit, 3=links, 4=rechts, 5=links vooruit, 6=links achteruit, 7=rechts vooruit, 8=rechts achteruit, 9=stop>
}
```

* De robot blijft in die richting bewegen totdat een nieuw commando komt.

---

## N = 105 — Pas LED Helderheid Aan

```json
{
  "N": 105,
  "D1": <1=verhoog met 5, 2=verlaag met 5>
}
```

**Voorbeeld:**

```json
{ "N": 105, "D1": 1 }
```

* Verhoog de helderheid met 5.

---

## N = 106 — Rocker-Gestuurde Servo

```json
{
  "N": 106,
  "D1": <1..5 voor verschillende kantelstappen (omhoog/omlaag/centraal)>
}
```

* Verplaatst de servo in kleine stappen in plaats van een directe hoekinstelling.
