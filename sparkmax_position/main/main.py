from sparkmax import SparkMax
import time

if __name__ == "__main__":
    # Asegúrate de que el bus can0 esté activo (sudo ip link set can0 up type can bitrate 1000000)
    # NOTA: Verifica que can_id coincida con el de tu Hardware Client. Normalmente es 1, no 0.
    
    try:
        # El bloque 'with' arranca automáticamente los hilos de control y lectura
        with SparkMax(can_id=1, channel="can0") as motor:
            
            print("Conexión establecida con el Spark Max. Presiona Ctrl+C para salir.")
            
            while True:
                # 1. Imprimir la telemetría actual (actualizada en tiempo real por el hilo secundario)
                pos = motor.get_position()
                vel = motor.get_velocity()
                print(f"\n[Telemetría] Posición: {pos:.4f} rot | Velocidad: {vel:.4f} RPM")
                
                # 2. Pedir el nuevo Setpoint
                entrada = input("Introduce Setpoint de Posición (o 'q' para salir): ")
                
                if entrada.lower() == 'q':
                    print("Saliendo del control...")
                    break # Rompe el bucle while
                
                try:
                    SP = float(entrada)
                    # 3. Enviar el Setpoint al hilo de control
                    motor.set_position(SP)
                    print(f"--> Setpoint enviado: {SP} rotaciones")
                    
                    # Pequeña pausa para dar tiempo a que el motor empiece a moverse antes de la próxima lectura
                    time.sleep(0.5) 
                    
                except ValueError:
                    print("Error: Por favor introduce un valor numérico válido.")
                    
    except KeyboardInterrupt:
        print("\nPrograma detenido por el usuario (Ctrl+C).")
        
    # Al salir de la indentación del 'with', se detienen los hilos y se apaga el bus automáticamente.
    print("Comunicación CAN cerrada de forma segura.")