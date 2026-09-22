# Tools

Este directorio contiene utilidades auxiliares para trabajar con datos de CAN y decodificación de identificadores de comunicación usados en sistemas FRC / REV Robotics.

## Contenido

### `decode_arbitration_id.py`
Este script decodifica un CAN Arbitration ID extendido de 29 bits y lo separa en sus campos principales:

- Device Type
- Manufacturer Code
- API Class
- API Index
- Device Number

Es especialmente útil para interpretar IDs de dispositivos como los motores SPARK MAX y otros componentes de la plataforma REV.

---

## ¿Qué hace?

El script recibe un ID hexadecimal o decimal, lo interpreta como un valor de 29 bits y luego muestra:

- El valor completo del Arbitration ID
- Su representación binaria
- Cada campo extraído del ID con:
  - valor decimal
  - valor binario

Esto permite analizar rápidamente la estructura interna del identificador sin tener que hacer la separación manual bit a bit.

---

## Estructura del ID

El formato estándar que se decodifica es:

- Bits 28-24 → Device Type
- Bits 23-16 → Manufacturer Code
- Bits 15-10 → API Class
- Bits 9-6 → API Index
- Bits 5-0 → Device Number

---

## Uso

Ejecuta el script desde la terminal:

```bash
python decode_arbitration_id.py
