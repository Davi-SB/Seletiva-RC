from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
import math

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

    def decision(self):
        if len(self.targets) == 0:  # Nenhum alvo disponível
            return

        # Calcula qual robô azul está mais próximo do alvo
        closest_robot_id = None
        closest_distance = float('inf')

        for robot_id, robot in self.teammates.items():
            # Calcula a distância do robô ao alvo
            distance = Point(robot.x, robot.y).dist_to(self.targets[0])
            if distance < closest_distance:
                closest_distance = distance
                closest_robot_id = robot_id

        # Verifica se o robô atual é o mais próximo
        if closest_robot_id != self.id:
            return

        # Posição atual e alvo
        current_position = Point(self.robot.x, self.robot.y)
        target_position = self.targets[0]

        # Ajusta a rota para evitar colisões com robôs amarelos
        adjusted_target = self.avoid_obstacles(current_position, target_position)

        # Calcula a velocidade para ir ao alvo ajustado
        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, adjusted_target)
        velocity_factor = 0.5

        # Define velocidades
        self.set_vel(target_velocity * velocity_factor)
        self.set_angle_vel(target_angle_velocity)
        return

    def avoid_obstacles(self, current_position, target_position):
        safe_distance = 0.35  # Distância mínima segura dos obstáculos
        adjustment_factor = 1.7  # Intensidade do desvio
        adjusted_position = target_position

        for robot_id, opponent in self.opponents.items():
            obstacle_position = Point(opponent.x, opponent.y)
            distance_to_obstacle = current_position.dist_to(obstacle_position)

            # Verifica se o obstáculo está dentro da zona de risco
            if distance_to_obstacle < safe_distance:
                # Calcula o vetor de repulsão
                angle_to_obstacle = math.atan2(
                    obstacle_position.y - current_position.y,
                    obstacle_position.x - current_position.x,
                )
                repulsion_x = -math.cos(angle_to_obstacle) * adjustment_factor
                repulsion_y = -math.sin(angle_to_obstacle) * adjustment_factor

                # Ajusta o destino final para desviar do obstáculo
                adjusted_position = Point(
                    adjusted_position.x + repulsion_x,
                    adjusted_position.y + repulsion_y,
                )

        return adjusted_position

    def post_decision(self):
        pass