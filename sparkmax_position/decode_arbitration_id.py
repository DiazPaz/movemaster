"""
Decodificador de CAN Arbitration ID (formato extendido FRC / REV Robotics, 29 bits)

Estructura del ID:
    Bits 28-24 (5 bits) -> Device Type
    Bits 23-16 (8 bits) -> Manufacturer Code
    Bits 15-10 (6 bits) -> API Class
    Bits 9-6   (4 bits) -> API Index
    Bits 5-0   (6 bits) -> Device Number
"""

# Ancho en bits y posición inicial (bit menos significativo) de cada campo
CAMPOS = {
    "Device Type":       {"ancho": 5, "shift": 24},
    "Manufacturer Code": {"ancho": 8, "shift": 16},
    "API Class":         {"ancho": 6, "shift": 10},
    "API Index":         {"ancho": 4, "shift": 6},
    "Device Number":     {"ancho": 6, "shift": 0},
}


def decodificar(arb_id: int) -> dict:
    """Recibe el arbitration ID como entero y regresa cada campo en decimal y binario."""
    resultado = {}
    for nombre, info in CAMPOS.items():
        ancho = info["ancho"]
        shift = info["shift"]
        mascara = (1 << ancho) - 1
        valor = (arb_id >> shift) & mascara
        resultado[nombre] = {
            "decimal": valor,
            "binario": format(valor, f"0{ancho}b"),
        }
    return resultado


def main():
    entrada = input("Introduce el CAN Arbitration ID en hexadecimal (ej. 0x2050200 o 2050200): ").strip()
    entrada = entrada.lower().replace("0x", "")

    try:
        arb_id = int(entrada, 16)
    except ValueError:
        print("Entrada inválida. Asegúrate de escribir un valor hexadecimal (ej. 2050200).")
        return

    if arb_id >= (1 << 29):
        print(f"Advertencia: el valor 0x{arb_id:X} ocupa más de 29 bits; "
              f"un ID extendido FRC/REV solo usa los 29 bits menos significativos.\n")

    campos = decodificar(arb_id)

    print(f"\nArbitration ID: 0x{arb_id:07X}  ({arb_id} decimal)")
    print(f"Binario (29 bits): {format(arb_id & ((1 << 29) - 1), '029b')}\n")

    print(f"{'Campo':<18}{'Decimal':<10}{'Binario'}")
    print("-" * 40)
    for nombre, valores in campos.items():
        print(f"{nombre:<18}{valores['decimal']:<10}{valores['binario']}")


if __name__ == "__main__":
    main()
