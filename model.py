from mesa import Agent, Model
from mesa.time import SimultaneousActivation
from mesa.space import MultiGrid
from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer
import random
import networkx as nx  


class BoundaryAgent(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class BusAgent(Agent):
    def __init__(self, unique_id, model, route, bus_stops):
        super().__init__(unique_id, model)
        self.route = route  
        self.bus_stops = bus_stops  
        self.current_stop_index = 0  
        self.stop_counter = 0  
        self.happiness = 100 

    def step(self):
       
        if self.pos == self.bus_stops[self.current_stop_index]:
            if self.stop_counter < 5: 
                self.stop_counter += 1
                self.happiness += 1  
                return
            else:
                self.stop_counter = 0  # Reiniciar el contador después de detenerse
                self.happiness -= 2  # Perder felicidad por esperar demasiado
                # Cambiar al siguiente destino
                self.current_stop_index = (self.current_stop_index + 1) % len(self.bus_stops)
                self.update_route()

        # Detenerse si está en un semáforo en rojo
        if self.at_traffic_light() and not self.model.is_light_green(self.direction_to_next(), self.pos):
            self.happiness -= 1  # Perder felicidad al esperar
            return  # No moverse si el semáforo está en rojo

        # Moverse hacia el próximo destino
        self.move_along_route()

    def update_route(self):
        """Actualizar la ruta para dirigirse al siguiente destino."""
        current_pos = self.pos
        next_stop = self.bus_stops[self.current_stop_index]
        try:
            # Calcular la nueva ruta utilizando el grafo
            self.route = nx.shortest_path(self.model.graph, source=current_pos, target=next_stop)
        except nx.NetworkXNoPath:
            print(f"No hay camino entre {current_pos} y {next_stop}")
            self.route = []

    def move_along_route(self):
        """Moverse a lo largo de la ruta calculada."""
        if len(self.route) > 1:  # Asegurarse de que haya más pasos en la ruta
            next_pos = self.route[1]  # Próxima posición en la ruta
            if self.can_move(next_pos):
                self.model.grid.move_agent(self, next_pos)
                self.route.pop(0)  # Eliminar la posición actual de la ruta
                self.happiness += 0.1  # Incremento leve de felicidad al moverse
        else:
            # Si la ruta está vacía, recalcular (por si hubo un problema)
            self.update_route()

    def at_traffic_light(self):
        return self.pos in self.model.traffic_light_positions

    def direction_to_next(self):
        if len(self.route) > 1:
            next_pos = self.route[1]
            dx = next_pos[0] - self.pos[0]
            dy = next_pos[1] - self.pos[1]
            if dx != 0:
                return "horizontal"
            elif dy != 0:
                return "vertical"
        return None

    def can_move(self, next_pos):
        if not self.model.grid.is_cell_empty(next_pos):
            contents = self.model.grid.get_cell_list_contents([next_pos])
            for obj in contents:
                if isinstance(obj, BoundaryAgent):
                    # Permitir moverse si es un estacionamiento
                    if "parking" in obj.unique_id:
                        return True
                    return False  # No se puede mover a otros límites
                if isinstance(obj, CarAgent) or isinstance(obj, AggressiveDriverAgent) or isinstance(obj, BusAgent):
                    return False  # No se puede mover a celdas ocupadas por carros, conductores agresivos u otros autobuses
        return True


class AggressiveDriverAgent(Agent):
    def __init__(self, unique_id, model, route):
        super().__init__(unique_id, model)
        self.route = route  # Ruta del conductor
        self.current_step = 0  # Índice actual en la ruta
        self.happiness = 100  # Felicidad inicial del conductor

    def step(self):
        # Comportamiento en semáforos: ignorar el 80% de las veces si están en rojo
        if self.at_traffic_light() and not self.model.is_light_green(self.direction_to_next(), self.pos):
            if random.random() < 0.8:  # Probabilidad del 80% de ignorar el semáforo
                self.happiness += 1  # Felicidad por avanzar a pesar del semáforo
            else:
                self.happiness -= 1  # Perder felicidad por respetar el semáforo
                return  # No moverse si decide respetar el semáforo

        # Moverse agresivamente a lo largo de la ruta
        self.move_aggressively()

    def at_traffic_light(self):
        """Verificar si el conductor está en un semáforo."""
        return self.pos in self.model.traffic_light_positions

    def direction_to_next(self):
        if len(self.route) > 1:
            next_pos = self.route[self.current_step + 1]
            dx = next_pos[0] - self.pos[0]
            dy = next_pos[1] - self.pos[1]
            if dx != 0:
                return "horizontal"
            elif dy != 0:
                return "vertical"
        return None

    def move_aggressively(self):
        steps_to_take = 2
        for _ in range(steps_to_take):
            if self.current_step < len(self.route) - 1:
                next_pos = self.route[self.current_step + 1]
                if self.can_move(next_pos):
                    self.model.grid.move_agent(self, next_pos)
                    self.current_step += 1
                    self.happiness -= 0.5  
                else:
                    break  # Detenerse si no puede moverse al siguiente nodo
            else:
                break  # Ruta completada

    def can_move(self, next_pos):
        if not self.model.grid.is_cell_empty(next_pos):
            contents = self.model.grid.get_cell_list_contents([next_pos])
            for obj in contents:
                if isinstance(obj, BoundaryAgent):
                    # Permitir moverse si es un estacionamiento
                    if "parking" in obj.unique_id:
                        return True
                    return False  # No se puede mover a otros límites
                if isinstance(obj, CarAgent) or isinstance(obj, AggressiveDriverAgent)  or isinstance(obj, BusAgent):
                    return False  # No se puede mover a otras posiciones ocupadas por carros
        return True



class CarAgent(Agent):
    def __init__(self, unique_id, model, route):
        super().__init__(unique_id, model)
        self.route = route  
        self.current_step = 0 
        self.happiness = 100  
        self.state = "happy"

    def step(self):
        if self.happiness > 80:
            self.state = "happy"
        elif self.happiness < 50:
            self.state = "angry"

       
        if self.state == "happy":
            self.happiness += 0.2  
        elif self.state == "angry":
            self.happiness -= 0.5  

        
        if self.current_step < len(self.route) - 1:
            next_pos = self.route[self.current_step + 1]

            
            dir = self.direction(next_pos)

            
            if self.at_traffic_light() and not self.model.is_light_green(dir, self.pos):
                self.happiness -= 1 if self.state == "happy" else 2
                return  

            
            if self.can_move(next_pos):
                self.model.grid.move_agent(self, next_pos)
                self.current_step += 1
                self.happiness += 0.1
        else:
            pass

    def at_traffic_light(self):
        return self.pos in self.model.traffic_light_positions

    def direction(self, next_pos):
        dx = next_pos[0] - self.pos[0]
        dy = next_pos[1] - self.pos[1]
        if dx > 0:
            return "horizontal"
        elif dx < 0:
            return "horizontal"
        elif dy > 0:
            return "vertical"
        elif dy < 0:
            return "vertical"
        else:
            return None

    def can_move(self, next_pos):
        
        if not self.model.grid.is_cell_empty(next_pos):
            contents = self.model.grid.get_cell_list_contents([next_pos])
            for obj in contents:
                if isinstance(obj, BoundaryAgent):
                    
                    if "parking" in obj.unique_id:
                        return True
                    return False  
                if isinstance(obj, CarAgent)  or isinstance(obj, BusAgent) or isinstance(obj, AggressiveDriverAgent) :
                    return False  
        return True


class EmergencyVehicleAgent(Agent):
    def __init__(self, unique_id, model, route):
        super().__init__(unique_id, model)
        self.route = route  # Ruta del vehículo de emergencia
        self.current_step = 0  # Índice actual en la ruta
        self.happiness = 100  # Felicidad inicial

    def step(self):
        # Verificar si se llegó al final de la ruta
        if self.current_step >= len(self.route) - 1:
            return  # No realizar ninguna acción si no hay más ruta

        # Moverse agresivamente a lo largo de la ruta
        self.move_emergency()

    def move_emergency(self):
        """Moverse a lo largo de la ruta, ignorando semáforos y límites."""
        step_size = 2  # Mayor velocidad para vehículos de emergencia
        for _ in range(step_size):
            if self.current_step < len(self.route) - 1:
                next_pos = self.route[self.current_step + 1]
                if self.can_move(next_pos):  # Verificar si puede moverse
                    self.model.grid.move_agent(self, next_pos)
                    self.current_step += 1
                    self.happiness += 0.5  # Incrementar felicidad al avanzar
                else:
                    return  # Detenerse si no puede avanzar
            else:
                break  # Detener el movimiento si se llega al final de la ruta

    def can_move(self, next_pos):
        """Verificar si el vehículo de emergencia puede moverse a la siguiente posición."""
        if not self.model.grid.is_cell_empty(next_pos):
            contents = self.model.grid.get_cell_list_contents([next_pos])
            for obj in contents:
                if isinstance(obj, BoundaryAgent):
                    # Permitir moverse si es un estacionamiento
                    if "parking" in obj.unique_id:
                        return True
                    return False  # No moverse a otros límites
                if isinstance(obj, CarAgent) or isinstance(obj, AggressiveDriverAgent) or isinstance(obj, BusAgent):
                    return False  # No moverse a celdas ocupadas por carros, autobuses u otros conductores
        return True
    

# Traffic light agent class
class TrafficLightAgent(Agent):
    def __init__(self, unique_id, model, pos, orientation, smart=False):
        super().__init__(unique_id, model)
        self.pos = pos
        self.state = "red"
        self.orientation = orientation
        self.smart = smart  # Indica si el semáforo es inteligente
        self.light_interval = model.light_interval
        self.step_count = 0

    def turn_green(self):
        self.state = "green"

    def turn_red(self):
        self.state = "red"

    def step(self):
        self.step_count += 1
        if self.smart:
            emergency_nearby = False
            for emergency_vehicle in self.model.emergency_vehicles:
                dist = abs(emergency_vehicle.pos[0] - self.pos[0]) + abs(emergency_vehicle.pos[1] - self.pos[1])
                if dist <= 3:  # Si la ambulancia está a 3 celdas o menos
                    emergency_nearby = True
                    break
            if emergency_nearby:
                self.turn_green()
                return
        # Comportamiento normal del semáforo
        if self.step_count % (2 * self.light_interval) < self.light_interval:
            if self.orientation == "horizontal":
                self.turn_green()
            else:
                self.turn_red()
        else:
            if self.orientation == "vertical":
                self.turn_green()
            else:
                self.turn_red()

# Main traffic model
class TrafficModel(Model):
    def __init__(self, M, N, light_interval):
        self.grid = MultiGrid(M, N, torus=False)  # Set torus to False to prevent wrapping
        self.schedule = SimultaneousActivation(self)
        self.running = True
        self.light_interval = light_interval
        self.step_count = 0

        # Crear el grafo
        self.graph = nx.DiGraph()
        # Añadir todos los nodos en la cuadrícula
        for x in range(M):
            for y in range(N):
                self.graph.add_node((x, y))

        # Definir calles de un solo sentido
        self.one_way_streets = {

            
            **{(22, y): "north" for y in range(2, 8)},
            **{(22, y): "north" for y in range(10, 16)},
            **{(22, y): "north" for y in range(17, 22)},
            **{(23, y): "north" for y in range(2, 22)},


            **{(x, 0): "east" for y in range(0, 23)},
            **{(x, 1): "east" for y in range(1, 5)},
            **{(x, 1): "east" for y in range(8, 11)},
            **{(x, 1): "east" for y in range(8, 11)},
            **{(x, 1): "east" for y in range(16, 17)},
            **{(x, 1): "east" for y in range(19, 20)},
            
            
            **{(x, 22): "west" for x in range(22, 15, -1)},
            **{(x, 22): "west" for x in range(11, 7, -1)},
            **{(x, 22): "west" for x in range(5, 1, -1)},
            **{(x, 23): "west" for x in range(22, 1, -1)},

            
            **{(0, y): "south" for y in range(23, 1, -1)},
            **{(1, y): "south" for y in range(22, 11, -1)},
            **{(1, y): "south" for y in range(7, 5, -1)},
            **{(1, y): "south" for y in range(3, 1, -1)},
            
            
            


            **{(14, y): "north" for y in range(2, 9)},
            **{(14, y): "north" for y in range(12, 23)},
            **{(15, y): "north" for y in range(2, 8)},
            **{(15, y): "north" for y in range(12, 23)},
            **{(15, 9): "north"},
            **{(2, 9): "east" for x in range(2, 13)},
            **{(x, 9): "east" for x in range(2, 13)},
            **{(x, 8): "east" for x in range(2, 6)},
            **{(x, 8): "east" for x in range(7, 12)},
            **{(x, 9): "east" for x in range(16, 22)},
            **{(x, 8): "east" for x in range(16, 22)},
            **{(13, 8): "east"},
            **{(x, 10): "west" for x in range(21, 14, -1)},
            **{(x, 11): "west" for x in range(21, 15, -1)},
            **{(x, 10): "west" for x in range(11, 0, -1)},
            **{(x, 11): "west" for x in range(11, 0, -1)},
            **{(14, 11): "west"},
            **{(12, y): "south" for y in range(7, 0, -1)},
            **{(13, y): "south" for y in range(7, 0, -1)},
            **{(12, y): "south" for y in range(21, 11, -1)},
            **{(13, y): "south" for y in range(21, 10, -1)},
            **{(12, 10): "south"},


            **{(18, y): "north" for y in range(2, 8)},
            **{(19, y): "north" for y in range(2, 8)},

            
            **{(6, y): "south" for y in range(7, 6, -1)},
            **{(6, y): "south" for y in range(3, 1, -1)},
            **{(7, y): "south" for y in range(7, 6, -1)},
            **{(7, y): "south" for y in range(3, 1, -1)},

            
            **{(x, 5): "west" for x in range(6, 1, -1)},
            **{(x, 4): "west" for x in range(5, 1, -1)},

            
            **{(x, 5): "east" for x in range(7, 12)},
            **{(x, 4): "east" for x in range(8, 12)},

            
            **{(6, y): "north" for y in range(11, 17)},
            **{(7, y): "north" for y in range(11, 17)},
            **{(6, y): "north" for y in range(19, 22)},
            **{(7, y): "north" for y in range(19, 22)},

            
            **{(x, 17): "west" for x in range(11, 7, -1)},
            **{(11, y): "west" for y in range(8, 10)},

            **{(x, 17): "west" for x in range(20, 15, -1)},
            **{(x, 16): "west" for x in range(20, 15, -1)},

        }


        
        self.turn_restrictions = {
            
        }

        # Crear aristas con consideraciones de calles de un solo sentido y restricciones de giro
        self.create_graph_edges(M, N)

        
        self.traffic_light_positions = [
            (5, 0), (5, 1),  
            (6, 2), (7, 2),  
            (0, 6), (1, 6), 
            (2, 4), (2, 5),  
            (18, 7), (19, 7),  
            (17, 8), (17, 9), 
            (6, 16), (7, 16),  
            (8, 17), (8, 18),  
            (6, 21), (7, 21),  
            (8, 22), (8, 23),  
        ]
        self.traffic_lights = {}

        # Crear semáforos
        for i, pos in enumerate(self.traffic_light_positions):
            # Definir la orientación basada en posiciones específicas
            if pos in [
                (5, 0), (5, 1), (2, 4), (2, 5), (8, 22), (8, 23),
                (17, 8), (17, 9), (8, 17), (8, 18)
            ]:
                orientation = "horizontal"
            elif pos in [
                (6, 2), (7, 2), (0, 6), (1, 6), (18, 7), (19, 7),
                (6, 21), (7, 21), (6, 16), (7, 16)
            ]:
                orientation = "vertical"
            else:
                orientation = "horizontal"  # Orientación por defecto

            
            smart = pos in [(18, 7), (19, 7), (17, 8), (17, 9)]

            light = TrafficLightAgent(f"light_{i}", self, pos, orientation, smart=smart)
            self.traffic_lights[pos] = light
            self.grid.place_agent(light, pos)
            self.schedule.add(light)

        # Edificios (áreas azules)
        building_positions = [
            [(x, y) for x in range(2, 6) for y in range(2, 3)],
            [(x, y) for x in range(2, 4) for y in range(3, 4)],
            [(x, y) for x in range(5, 6) for y in range(3, 4)],

            [(x, y) for x in range(2, 3) for y in range(6, 8)],
            [(x, y) for x in range(3, 6) for y in range(7, 8)],
            [(x, y) for x in range(4, 6) for y in range(6, 7)],

            [(x, y) for x in range(8, 9) for y in range(2, 4)],
            [(x, y) for x in range(9, 12) for y in range(3, 4)],
            [(x, y) for x in range(10, 12) for y in range(2, 3)],

            [(x, y) for x in range(8, 12) for y in range(6, 7)],
            [(x, y) for x in range(8, 10) for y in range(7, 8)],
            [(x, y) for x in range(11, 12) for y in range(7, 8)],

            [(x, y) for x in range(16, 18) for y in range(2, 4)],
            [(x, y) for x in range(16, 17) for y in range(4, 8)],
            [(x, y) for x in range(17, 18) for y in range(5, 6)],
            [(x, y) for x in range(17, 18) for y in range(7, 8)],

            [(x, y) for x in range(20, 22) for y in range(2, 4)],
            [(x, y) for x in range(21, 22) for y in range(4, 5)],
            [(x, y) for x in range(20, 22) for y in range(5, 8)],

            [(x, y) for x in range(2, 4) for y in range(12, 14)],
            [(x, y) for x in range(5, 6) for y in range(12, 13)],
            [(x, y) for x in range(4,6) for y in range(13, 17)],
            [(x, y) for x in range(2,4) for y in range(15, 21)],
            [(x, y) for x in range(2,3 ) for y in range(21,22)],
            [(x, y) for x in range(4,6 ) for y in range(18, 22)],
            [(x, y) for x in range(3, 4) for y in range(14, 15)],
            [(x, y) for x in range(4,5 ) for y in range(17,18 )],

            [(x, y) for x in range(8, 10) for y in range(12, 15)],
            [(x, y) for x in range(10, 12) for y in range(13, 17)],
            [(x, y) for x in range(11, 12) for y in range(12, 13)],
            [(x, y) for x in range(9, 10) for y in range(15, 17)],
            [(x, y) for x in range(8, 9) for y in range(16, 17)],

            [(x, y) for x in range(8, 10) for y in range(19, 22)],
            [(x, y) for x in range(10, 12) for y in range(20, 22)],
            [(x, y) for x in range(11, 12) for y in range(19, 20)],

            [(x, y) for x in range(16, 20) for y in range(18, 21)],
            [(x, y) for x in range(16,17 ) for y in range(21, 22)],
            [(x, y) for x in range(18, 22) for y in range(21, 22)],
            [(x, y) for x in range(21, 22 ) for y in range(18, 21)],
            [(x, y) for x in range(20, 21 ) for y in range(19, 21)],

            [(x, y) for x in range(16, 20 ) for y in range(12, 16)],
            [(x, y) for x in range(21, 22) for y in range(12, 16 )],
            [(x, y) for x in range(20, 21 ) for y in range(12, 15)],
        ]
        for idx, positions in enumerate(building_positions):
            for pos in positions:
                boundary = BoundaryAgent(f"building_{idx}_{pos[0]}_{pos[1]}", self)
                self.grid.place_agent(boundary, pos)
                # Remover nodos correspondientes a edificios del grafo
                if pos in self.graph:
                    self.graph.remove_node(pos)

        # Estacionamientos (áreas amarillas)
        parking_lots = [
            (2,14), (3,21), (3, 6), (4,12), (4,3), (5,17), (8, 15),
            (9,2), (10, 19), (10,12),  (10,7), (17, 21), (17,6),
            (17, 4), (20,18), (20,15), (20,4)
        ]
        for idx, lot in enumerate(parking_lots):
            boundary = BoundaryAgent(f"parking_{idx}_{lot[0]}_{lot[1]}", self)
            self.grid.place_agent(boundary, lot)
            

        # Rotonda (área marrón en el centro)
        roundabout_positions = [
            (14, 10), (13,10),
            (14,9), (13,9),
        ]
        for idx, pos in enumerate(roundabout_positions):
            boundary = BoundaryAgent(f"roundabout_{idx}_{pos[0]}_{pos[1]}", self)
            self.grid.place_agent(boundary, pos)
            # Remover nodos correspondientes a la rotonda del grafo
            if pos in self.graph:
                self.graph.remove_node(pos)

        # Remover aristas que están bloqueadas por obstáculos
        self.remove_edges_blocked_by_obstacles()

        
        # Añadir múltiples carros regulares
        car_start_positions = [
            (18,10), (15,2), (11,0), (23,9), (20,22), (0,0) 
        ]
        for i, start_pos in enumerate(car_start_positions):
            # Seleccionar un destino aleatorio de los estacionamientos
            destino = random.choice(parking_lots)
            
            if destino in self.graph.nodes:
                try:
                    # Calcular la ruta más corta desde el punto de inicio al destino
                    route = nx.shortest_path(self.graph, source=start_pos, target=destino)
                    car = CarAgent(f"car_{i}", self, route)
                    self.grid.place_agent(car, start_pos)
                    self.schedule.add(car)
                except nx.NetworkXNoPath:
                    print(f"No hay camino entre {start_pos} y {destino}")

        bus_stops = [(15, 4), (15, 13), (8,8)]  # Ejemplo de paradas de autobuses

        
        for i, start_pos in enumerate(bus_stops):  # Comenzar cada autobús en una parada
            bus = BusAgent(f"bus_{i}", self, [], bus_stops)
            self.grid.place_agent(bus, start_pos)
            self.schedule.add(bus)


        
        aggressive_start_positions = [(14, 2), (20, 10), (0, 23)]
        for i, start_pos in enumerate(aggressive_start_positions):
            # Seleccionar un destino aleatorio de los estacionamientos
            destino = random.choice(parking_lots)
            
            # Asegurarte de que el destino sea válido (exista en el grafo)
            if destino in self.graph.nodes:
                try:
                    # Calcular la ruta más corta desde el punto de inicio al destino
                    route = nx.shortest_path(self.graph, source=start_pos, target=destino)
                    car = AggressiveDriverAgent(f"aggressive_{i}", self, route)
                    self.grid.place_agent(car, start_pos)
                    self.schedule.add(car)
                except nx.NetworkXNoPath:
                    print(f"No hay camino entre {start_pos} y {destino}")

        self.emergency_vehicles = []
        emergency_start_positions = [(10,11), (18,6)]
        for i, start_pos in enumerate(emergency_start_positions):
            # Seleccionar un destino aleatorio de los estacionamientos
            destino = random.choice(parking_lots)
            
            # Asegurarte de que el destino sea válido (exista en el grafo)
            if destino in self.graph.nodes:
                try:
                    # Calcular la ruta más corta desde el punto de inicio al destino
                    route = nx.shortest_path(self.graph, source=start_pos, target=destino)
                    car = EmergencyVehicleAgent(f"emergency_{i}", self, route)
                    self.grid.place_agent(car, start_pos)
                    self.schedule.add(car)
                    self.emergency_vehicles.append(car)
                except nx.NetworkXNoPath:
                    print(f"No hay camino entre {start_pos} y {destino}")

    def get_positions(self):
        """
        Devuelve una lista combinada de las posiciones de todos los agentes en el modelo,
        excluyendo los semáforos (TrafficLightAgent).
        El formato es una lista de diccionarios con claves 'x' y 'z'.
        """
        points = []
        for agent in self.schedule.agents:
            # Excluir TrafficLightAgent
            if not isinstance(agent, TrafficLightAgent):
                points.append({"x": agent.pos[0], "z": agent.pos[1]})
        return {"points": points}
    
    def create_graph_edges(self, M, N):
        for x in range(M):
            for y in range(N):
                current_pos = (x, y)
                if current_pos not in self.graph:
                    continue  # Skip if the node was removed (obstacle)

                # Check if current position is a one-way street
                if current_pos in self.one_way_streets:
                    direction = self.one_way_streets[current_pos]
                    if direction == "east" and x < M - 1 and (x + 1, y) in self.graph:
                        self.graph.add_edge(current_pos, (x + 1, y))
                    elif direction == "west" and x > 0 and (x - 1, y) in self.graph:
                        self.graph.add_edge(current_pos, (x - 1, y))
                    elif direction == "north" and y < N - 1 and (x, y + 1) in self.graph:
                        self.graph.add_edge(current_pos, (x, y + 1))
                    elif direction == "south" and y > 0 and (x, y - 1) in self.graph:
                        self.graph.add_edge(current_pos, (x, y - 1))
                else:
                    # Two-way street
                    # Add edges in all possible directions
                    if x < M - 1 and (x + 1, y) in self.graph:
                        self.graph.add_edge(current_pos, (x + 1, y))
                        self.graph.add_edge((x + 1, y), current_pos)
                    if x > 0 and (x - 1, y) in self.graph:
                        self.graph.add_edge(current_pos, (x - 1, y))
                        self.graph.add_edge((x - 1, y), current_pos)
                    if y < N - 1 and (x, y + 1) in self.graph:
                        self.graph.add_edge(current_pos, (x, y + 1))
                        self.graph.add_edge((x, y + 1), current_pos)
                    if y > 0 and (x, y - 1) in self.graph:
                        self.graph.add_edge(current_pos, (x, y - 1))
                        self.graph.add_edge((x, y - 1), current_pos)


            # Añadir restricciones de giro
            self.add_edges_with_turn_restrictions()

    def add_edges_with_turn_restrictions(self):
        # Ajustar las aristas en las intersecciones según las restricciones de giro
        for node in self.graph.nodes():
            if node not in self.graph:
                continue  # Saltar si el nodo fue removido
            restrictions = self.turn_restrictions.get(node, {})
            x, y = node
            # Movimientos posibles desde el nodo actual
            moves = {
                "north": (x, y + 1),
                "south": (x, y - 1),
                "east": (x + 1, y),
                "west": (x - 1, y)
            }
            for from_dir, from_pos in moves.items():
                if from_pos in self.graph:
                    prohibited_turns = restrictions.get(from_dir, [])
                    # Movimiento recto
                    if "no_straight" not in prohibited_turns and self.graph.has_edge(node, from_pos):
                        continue  # El movimiento recto ya está permitido
                    else:
                        # Remover arista si el movimiento recto está prohibido
                        if self.graph.has_edge(node, from_pos):
                            self.graph.remove_edge(node, from_pos)
                    # Giro a la izquierda
                    if "no_left_turn" in prohibited_turns:
                        left_turn_pos = self.get_left_turn_node(node, from_dir)
                        if left_turn_pos and self.graph.has_edge(node, left_turn_pos):
                            self.graph.remove_edge(node, left_turn_pos)
                    # Giro a la derecha
                    if "no_right_turn" in prohibited_turns:
                        right_turn_pos = self.get_right_turn_node(node, from_dir)
                        if right_turn_pos and self.graph.has_edge(node, right_turn_pos):
                            self.graph.remove_edge(node, right_turn_pos)


    def get_left_turn_node(self, node, from_direction):
        x, y = node
        if from_direction == "north":
            return (x - 1, y)
        elif from_direction == "south":
            return (x + 1, y)
        elif from_direction == "east":
            return (x, y + 1)
        elif from_direction == "west":
            return (x, y - 1)
        return None

    def get_right_turn_node(self, node, from_direction):
        x, y = node
        if from_direction == "north":
            return (x + 1, y)
        elif from_direction == "south":
            return (x - 1, y)
        elif from_direction == "east":
            return (x, y - 1)
        elif from_direction == "west":
            return (x, y + 1)
        return None

    def remove_edges_blocked_by_obstacles(self):
        # Remover aristas que conducen a nodos bloqueados por obstáculos
        for node in self.graph.nodes():
            x, y = node
            # Verificar vecinos
            neighbors = list(self.graph.successors(node))
            for neighbor in neighbors:
                if neighbor not in self.graph:
                    self.graph.remove_edge(node, neighbor)

    def is_light_green(self, direction, pos):
        if pos in self.traffic_light_positions:
            light = self.traffic_lights[pos]
            if light.state == "green" and light.orientation == direction:
                return True
            else:
                return False
        return True  

    def step(self):
        self.step_count += 1
        self.schedule.step()



def agent_portrayal(agent):

    portrayal = {}
    if isinstance(agent, BoundaryAgent):
        if "building" in agent.unique_id:
            portrayal = {
                "Shape": "rect",
                "Filled": "true",
                "Layer": 0,
                "Color": "blue",
                "w": 1,
                "h": 1,
            }
        elif "roundabout" in agent.unique_id:
            portrayal = {
                "Shape": "rect",
                "Filled": "true",
                "Layer": 0,
                "Color": "brown",
                "w": 1,
                "h": 1,
            }
        elif "parking" in agent.unique_id:
            portrayal = {
                "Shape": "rect",
                "Filled": "true",
                "Layer": 0,
                "Color": "yellow",
                "w": 1,
                "h": 1,
            }
        else:
            portrayal = {
                "Shape": "rect",
                "Filled": "true",
                "Layer": 0,
                "Color": "gray",  
                "h": 1,
            }
    elif isinstance(agent, CarAgent):
        portrayal = {
            "Shape": "circle",
            "Filled": "true",
            "Layer": 1,
            "Color": "blue" if agent.state == "happy" else "red",
            "r": 0.5,
            "text": f"{int(agent.happiness)}",
            "text_color": "white",
        }
    elif isinstance(agent, BusAgent):  # Visualización de autobuses
        portrayal = {
            "Shape": "circle",
            "Filled": "true",
            "Layer": 1,
            "Color": "green",  
            "r": 0.7,  
            "text": f"Bus", 
            "text_color": "white",
        }
    elif isinstance(agent, AggressiveDriverAgent):
        portrayal = {
            "Shape": "circle",
            "Filled": "true",
            "Layer": 1,
            "Color": "orange",  
            "r": 0.5,
            "text": "Agg",
            "text_color": "white",
    }

    elif isinstance(agent, EmergencyVehicleAgent):
        portrayal = {
            "Shape": "circle",
            "Filled": "true",
            "Layer": 1,
            "Color": "red",  #
            "r": 0.6,
            "text": "E",
            "text_color": "white",
    }
    elif isinstance(agent, TrafficLightAgent):
        portrayal = {
            "Shape": "circle",
            "Filled": "true",
            "Layer": 1,
            "Color": "green" if agent.state == "green" else "red",
            "r": 0.5,
        }
    return portrayal


M, N = 24, 24
light_interval = 10

grid = CanvasGrid(agent_portrayal, M, N, 600, 600)
server = ModularServer(
    TrafficModel,
    [grid],
    "Traffic Simulation with Various Vehicles",
    {
        "M": M,
        "N": N,
        "light_interval": light_interval,
    },
)

server.port = 8523

if __name__ == "__main__":
    server.launch()