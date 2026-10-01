#!/usr/bin/env python3
import os
import re

home_dir = os.path.expanduser('~')

# Mantenemos la barra final ("/") por si el resto de tu código une los textos directamente
MAPS_DIR = os.path.join(home_dir, "papa", "Mappeo") + "/"

LAUNCH_FILE = os.path.join(home_dir, "papa", "papa_ws", "src", "papa_description", "launch", "display.launch.py")

def main():
    print("\nBuscando mapas disponibles...")
    
    # 1. Validar directorio y buscar archivos .yaml
    if not os.path.exists(MAPS_DIR):
        print(f"Error: El directorio {MAPS_DIR} no existe.")
        return

    map_files = [f for f in os.listdir(MAPS_DIR) if f.endswith('.yaml') and f != 'nav2_params.yaml']
    
    if not map_files:
        print(f"No se encontraron mapas en {MAPS_DIR}.")
        return

    map_files.sort()

    # 2. Mostrar la lista al usuario
    print("\n" + "="*40)
    print(" MAPAS DISPONIBLES")
    print("="*40)
    for i, file in enumerate(map_files, 1):
        print(f"  [{i}] {file}")
    print("="*40)

    # 3. Solicitar selección
    try:
        seleccion = int(input("\nIngresa el número del mapa que deseas utilizar: "))
        if seleccion < 1 or seleccion > len(map_files):
            print("Número fuera de rango. Operación cancelada.")
            return
    except ValueError:
        print("Entrada no válida. Operación cancelada.")
        return

    mapa_seleccionado = map_files[seleccion - 1]
    nueva_ruta = os.path.join(MAPS_DIR, mapa_seleccionado)

    # 4. Modificar el archivo display.launch.py
    if not os.path.exists(LAUNCH_FILE):
        print(f"Error: No se encuentra el archivo {LAUNCH_FILE}.")
        return

    with open(LAUNCH_FILE, 'r') as file:
        contenido = file.read()

    # Patrón Regex: Busca "mapa_path = 'ruta'" y reemplaza el string interno
    patron = r'(mapa_path\s*=\s*)["\'][^"\']+["\']'
    nuevo_contenido = re.sub(patron, rf'\1"{nueva_ruta}"', contenido)

    if contenido == nuevo_contenido:
        print("\nAdvertencia: No se detectaron cambios. Es posible que el archivo ya tenga este mapa o que la variable 'mapa_path' tenga otro formato.")
        return

    with open(LAUNCH_FILE, 'w') as file:
        file.write(nuevo_contenido)

    print(f"\n✅ ¡Éxito! El robot ahora utilizará el mapa: {mapa_seleccionado}")
    print("Cierra tu nodo de navegación actual y vuelve a lanzar display.launch.py para aplicar los cambios.")

if __name__ == "__main__":
    main()
