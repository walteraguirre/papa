import json
import ollama

# 1. Cargar tus objetivos
with open('/home/perez_010/papa/Mappeo/waypoints.json', 'r') as f:
    waypoints = json.load(f)

# 2. Construir el prompt para Llama 3.1
prompt = f"""
Eres un asistente de navegación. Aquí tienes una lista de objetivos disponibles con sus coordenadas:
{json.dumps(waypoints, indent=2)}

El usuario te dirá a dónde quiere ir. Tú debes responder EXCLUSIVAMENTE con el nombre exacto de la llave del objetivo en el JSON. 
Si no encuentras el objetivo, responde "NO_ENCONTRADO".
"""

def seleccionar_objetivo(solicitud_usuario):
    response = ollama.chat(model='llama3.1:8b', messages=[
        {'role': 'system', 'content': prompt},
        {'role': 'user', 'content': solicitud_usuario},
    ])
    
    eleccion = response['message']['content'].strip()
    return eleccion

# Ejemplo de uso:
destino = seleccionar_objetivo("Quiero ir al baño de hombres")
print(f"El modelo eligió: {destino}")

if destino in waypoints:
    print(f"Coordenadas: {waypoints[destino]}")
