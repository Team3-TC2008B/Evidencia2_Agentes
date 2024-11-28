from flask import Flask
from flask import Flask, jsonify
from model import TrafficModel

app = Flask(__name__)

# Inicializa el modelo con los parámetros necesarios
traffic_model = TrafficModel(M=24, N=24, light_interval=10)

@app.route("/")
def index():
    return jsonify({"Message": "Welcome to the Traffic Simulation API!"})

@app.route("/positions")
def positions():
    """
    Devuelve las posiciones de los agentes en el modelo.
    """
    traffic_model.step()  # Avanza un paso en la simulación
    positions = traffic_model.get_positions()  # Obtiene las posiciones actuales
    return jsonify(positions)

@app.route("/getSemaforosEstado", methods=['GET'])
def getSemaforosEstado():

    """
    Devuelve los estados de los semáforos.
    """
    traffic_model.step()# Avanza un paso en la simulación
    semaforos_estado = [
        {
            "id": int(light.unique_id.split('_')[-1]),  # Extrae el número del id (light_0 -> 0)
            "position": list(light.pos),  # Asegura que position esté correctamente formateado como una lista
            "estado": light.state
        }
        for light in traffic_model.traffic_lights.values()
    ]
    
    return jsonify(semaforos_estado)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000, debug=True)