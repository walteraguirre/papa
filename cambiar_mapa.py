#!/usr/bin/env python3
import os
import re

home_dir = os.path.expanduser('~')
MAPS_DIR = os.path.join(home_dir, "papa", "Mappeo")
LAUNCH_FILE = os.path.join(home_dir, "papa", "papa_ws", "src", "papa_description", "launch", "display.launch.py")

def main():
    print("\nBuscando mapas disponibles...")
    
    if not os.path.exists(MAPS_DIR):
        print(f"Error: El directorio {MAPS_DIR} no existe.")
        return

    # Filtro inteligente: Solo mostramos archivos .yaml que tengan un archivo .pgm con el mismo nombre
    # Esto excluye automáticamente nav2_params.yaml y archivos de waypoints sueltos
    map_files = []
    for f in os.listdir(MAPS_DIR):
        if f.endswith('.yaml'):
            base_name = f[:-5] # Quita el '.yaml'
            if os.path.exists(os.path.join(MAPS_DIR, f"{base_name}.pgm")):
                map_files.append(f)
    
    if not map_files:
        print(f"No se encontraron mapas válidos (YAML + PGM) en {MAPS_DIR}.")
        return

    map_files.sort()

    print("\n" + "="*40)
    print(" MAPAS DISPONIBLES")
    print("="*40)
    for i, file in enumerate(map_files, 1):
        print(f"  [{i}] {file}")
    print("="*40)

    try:
        seleccion = int(input("\nIngresa el número del mapa que deseas utilizar: "))
        if seleccion < 1 or seleccion > len(map_files):
            print("Número fuera de rango. Operación cancelada.")
            return
    except ValueError:
        print("Entrada no válida. Operación cancelada.")
        return

    mapa_seleccionado = map_files[seleccion - 1]

    if not os.path.exists(LAUNCH_FILE):
        print(f"Error: No se encuentra el archivo {LAUNCH_FILE}.")
        return

    with open(LAUNCH_FILE, 'r') as file:
        contenido = file.read()

    # Nuevo Patrón Regex: Busca exactamente "mapa_path = os.path.join(..., 'nombre_mapa.yaml')"
    # y cambia solo el nombre del archivo al final.
    patron = r'(mapa_path\s*=\s*os\.path\.join\([^,]+,\s*[^,]+,\s*[^,]+,\s*)["\'][^"\']+["\']\)'
    
    # Verificamos si el patrón existe antes de reemplazar
    if not re.search(patron, contenido):
         print("\nError fatal: No se encontró la estructura 'mapa_path = os.path.join(...)' en tu display.launch.py.")
         print("Asegúrate de que la línea se vea así:")
         print('mapa_path = os.path.join(home_dir, "papa", "Mappeo", "nombre.yaml")')
         return

    nuevo_contenido = re.sub(patron, rf'\1"{mapa_seleccionado}")', contenido)

    if contenido == nuevo_contenido:
        print(f"\nEl robot ya está utilizando el mapa '{mapa_seleccionado}'. No se hicieron cambios.")
        return

    with open(LAUNCH_FILE, 'w') as file:
        file.write(nuevo_contenido)

    print(f"\n✅ ¡Éxito! El robot ahora utilizará el mapa: {mapa_seleccionado}")
    print("Cierra tu nodo de navegación actual y vuelve a lanzar display.launch.py para aplicar los cambios.")

if __name__ == "__main__":
    main()
