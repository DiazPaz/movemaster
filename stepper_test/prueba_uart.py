#!/usr/bin/env python3
"""
Prueba manual de UART, sin ROS2.
Abre el puerto UNA sola vez, espera a que la ESP32 termine de
arrancar (el reset automático por DTR ya habrá pasado), y
DESPUÉS manda el comando de ángulo.
"""

import serial
import time

PUERTO = '/dev/ttyUSB0'
BAUD = 115200

print(f'Abriendo {PUERTO} @ {BAUD}...')
ser = serial.Serial(PUERTO, BAUD, timeout=1)

print('Esperando a que la ESP32 termine de reiniciar (3 segundos)...')
time.sleep(3)

# Vacía cualquier basura del buffer (mensajes de boot, etc.)
ser.reset_input_buffer()

comando = 'A180\n'
print(f'Enviando: {comando.strip()}')
ser.write(comando.encode('utf-8'))

# Espera un poco y muestra si la ESP32 responde algo
time.sleep(1)
if ser.in_waiting > 0:
    print('Respuesta de la ESP32:', ser.read(ser.in_waiting))
else:
    print('La ESP32 no mandó ninguna respuesta (puede ser normal si el sketch no imprime nada).')

ser.close()
print('Listo.')
