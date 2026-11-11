#!/usr/bin/env python3
import os
import re

home_dir = os.path.expanduser('~')
MAPS_DIR = os.path.join(home_dir, "papa", "Mappeo")
LAUNCH_DIR = os.path.join(
    home_dir, "papa", "papa_ws", "src", "papa_description", "launch"
)
LAUNCH_FILES = (
    "display.launch.py",
    "map_only.launch.py",
    "simulation.launch.py",
)
BRIDGE_FILE = os.path.join(
    home_dir,
    "papa",
    "papa_ws",
    "src",
    "papa_description",
    "scripts",
    "nav_ollama_bridge.py",
)


def _map_has_obstacle_layer(map_filename):
    base_name = map_filename[:-5]
    return all(
        os.path.exists(os.path.join(MAPS_DIR, f"{base_name}_obs{extension}"))
        for extension in (".yaml", ".pgm")
    )

def main():
    print("\nBuscando mapas disponibles...")
    
    if not os.path.exists(MAPS_DIR):
        print(f"Error: El directorio {MAPS_DIR} no existe.")
        return

    # Filtro inteligente: Solo mostramos archivos .yaml que tengan un archivo .pgm con el mismo nombre
    # Esto excluye automáticamente nav2_params.yaml y archivos de waypoints sueltos
    map_files = []
    for f in os.listdir(MAPS_DIR):
        if f.endswith('.yaml') and not f[:-5].endswith("_obs"):
            base_name = f[:-5] # Quita el '.yaml'
            if (
                os.path.exists(os.path.join(MAPS_DIR, f"{base_name}.pgm"))
                and _map_has_obstacle_layer(f)
            ):
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
    mapa_base = mapa_seleccionado[:-5]
    waypoints_file = f"{mapa_base}_waypoints.yaml"
    context_file = f"{mapa_base}_context.txt"
    waypoints_path = os.path.join(MAPS_DIR, waypoints_file)

    if not os.path.exists(waypoints_path):
        print(
            f"\nError: El mapa '{mapa_seleccionado}' no tiene archivo de "
            f"waypoints ({waypoints_file})."
        )
        print("Ejecuta record_waypoints.py para crear sus waypoints.")
        return

    patron = re.compile(
        r"""(mapa_path\s*=\s*os\.path\.join\(.*?["']Mappeo["']\s*,\s*["'])"""
        r"""[^"']+(["']\s*\))""",
        re.DOTALL,
    )
    archivos_actualizados = []

    for launch_filename in LAUNCH_FILES:
        launch_file = os.path.join(LAUNCH_DIR, launch_filename)
        if not os.path.exists(launch_file):
            print(f"Error: No se encuentra el archivo {launch_file}.")
            return

        with open(launch_file, 'r') as file:
            contenido = file.read()

        nuevo_contenido, reemplazos = patron.subn(
            lambda match: f'{match.group(1)}{mapa_seleccionado}{match.group(2)}',
            contenido,
            count=1,
        )
        if reemplazos != 1:
            print(
                f"\nError fatal: No se encontró la ruta 'mapa_path' en "
                f"{launch_filename}."
            )
            return

        if contenido != nuevo_contenido:
            with open(launch_file, 'w') as file:
                file.write(nuevo_contenido)
            archivos_actualizados.append(launch_filename)

    if not os.path.exists(BRIDGE_FILE):
        print(f"Error: No se encuentra el archivo {BRIDGE_FILE}.")
        return

    with open(BRIDGE_FILE, "r", encoding="utf-8") as file:
        bridge_content = file.read()

    bridge_patterns = (
        (
            r'(^\s*WAYPOINTS_FILE\s*=\s*MAPS_DIR\s*/\s*)"[^"]+"',
            f'\\1"{waypoints_file}"',
        ),
        (
            r'(^\s*CONTEXT_FILE\s*=\s*MAPS_DIR\s*/\s*)"[^"]+"',
            f'\\1"{context_file}"',
        ),
    )
    updated_bridge_content = bridge_content
    for bridge_pattern, replacement in bridge_patterns:
        updated_bridge_content, replacements = re.subn(
            bridge_pattern,
            replacement,
            updated_bridge_content,
            count=1,
            flags=re.MULTILINE,
        )
        if replacements != 1:
            print(
                "\nError fatal: No se encontraron las rutas "
                "WAYPOINTS_FILE/CONTEXT_FILE en nav_ollama_bridge.py."
            )
            return

    bridge_changed = bridge_content != updated_bridge_content
    if bridge_changed:
        with open(BRIDGE_FILE, "w", encoding="utf-8") as file:
            file.write(updated_bridge_content)

    if not archivos_actualizados and not bridge_changed:
        print(
            f"\nTodos los launch ya utilizan el mapa '{mapa_seleccionado}' "
            f"y el bridge ya utiliza '{waypoints_file}'. No se hicieron cambios."
        )
        return

    print(
        f"\n✅ ¡Éxito! Los launch de visualización/navegación ahora utilizarán "
        f"el mapa: {mapa_seleccionado}"
    )
    print("También cargarán automáticamente su capa '_obs' correspondiente.")
    if archivos_actualizados:
        print("Launch actualizados: " + ", ".join(archivos_actualizados))
    else:
        print("No fue necesario modificar los launch.")
    print(
        "mapeo.launch.py no se modifica porque ejecuta SLAM y no consume un "
        "mapa guardado."
    )
    print(f"Waypoints configurados: {waypoints_file}")
    if os.path.exists(os.path.join(MAPS_DIR, context_file)):
        print(f"Contexto configurado: {context_file}")
    else:
        print(
            f"Aviso: no existe {context_file}; el bridge iniciará sin "
            "contexto adicional."
        )
    print("Cierra los nodos actuales y vuelve a lanzar el archivo que necesites.")

if __name__ == "__main__":
    main()
