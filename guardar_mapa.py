#!/usr/bin/env python3
import os
import subprocess

# Usamos expanduser por si corres el script con distintos usuarios (perez_010 o papa)
MAPS_DIR = os.path.expanduser("~/papa/Mappeo/")

def main():
    print("\n" + "="*50)
    print(" 📸 GUARDADO DE MAPA - SLAM")
    print("="*50)
    
    mapa_nombre = input("▶ Ingresa el nombre para el nuevo mapa (ej. living_1): ").strip()
    if not mapa_nombre:
        print("❌ Nombre vacío. Operación cancelada.")
        return

    # Asegurarnos de que la carpeta exista
    os.makedirs(MAPS_DIR, exist_ok=True)
    
    ruta_completa = os.path.join(MAPS_DIR, mapa_nombre)
    
    print(f"\nExtrayendo mapa de SLAM Toolbox...")
    print(f"Destino: {ruta_completa}")
    
    # Ejecutar map_saver_cli
    comando = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", ruta_completa]
    
    try:
        # check=True hace que arroje error si el comando falla
        subprocess.run(comando, check=True)
        print("\n✅ ¡Mapa guardado exitosamente!")
        print("Ya puedes cerrar RViz2 en la otra terminal para apagar el robot.")
    except subprocess.CalledProcessError:
        print("\n❌ Error crítico al guardar el mapa.")
        print("¿Asegúrate de que mapeo.launch.py está corriendo en este momento?")

if __name__ == "__main__":
    main()
