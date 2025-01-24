from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
import math
import numpy as np

# from scipy.optimize import linear_sum_assignment  # Algoritmo de Hungarian - https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.linear_sum_assignment.html#scipy.optimize.linear_sum_assignment
from hungarian import my_linear_sum_assignment as assign_targets

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.assignment = dict()  # Dicionário de atribuição {robot_id: target}

    def decision(self):
        if len(self.targets) == 0:  # Nenhum alvo disponível
            return

        # Obter lista de IDs dos robôs disponíveis na equipe
        my_agents = list(self.teammates.keys())

        # Passo 1: Calcular a matriz de custos (distâncias) entre robôs e alvos
        cost_matrix = self.calculate_cost_matrix(my_agents)

        # Passo 2: Resolver o problema de atribuição com o algoritmo Hungarian
        #print(cost_matrix, end='\n\n----------------\n\n')
        robot_indices, target_indices = assign_targets(cost_matrix)

        # Atualizar a atribuição
        self.assignment = {my_agents[robot]: self.targets[target] 
                           for robot, target in zip(robot_indices, target_indices)}

        # Passo 3: Executar a tarefa se o robô atual tem um alvo atribuído
        if self.id in self.assignment:
            assigned_target = self.assignment[self.id]

            # Posição atual e alvo
            current_position = Point(self.robot.x, self.robot.y)
            adjusted_target = self.avoid_obstacles(current_position, assigned_target)

            # Calcular velocidades para o alvo ajustado
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, adjusted_target)
            velocity_factor = 0.5

            # Definir velocidades
            self.set_vel(target_velocity * velocity_factor)
            self.set_angle_vel(target_angle_velocity)

    def calculate_cost_matrix(self, my_agents):
        """
        Calcula a matriz de custos entre os robôs e os alvos.
        O custo é baseado na distância euclidiana entre cada robô e cada alvo.
        """
        num_agents = len(my_agents)
        num_targets = len(self.targets)

        # Inicializa a matriz de custos com zeros
        
        cost_matrix = np.zeros((num_agents, num_targets))

        for i, robot_id in enumerate(my_agents):
            robot = self.teammates[robot_id]  # Posição do robô
            for j, target in enumerate(self.targets):
                # Calcula a distância euclidiana entre o robô e o alvo
                cost_matrix[i, j] = Point(robot.x, robot.y).dist_to(target)

        return cost_matrix

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
        """
        Garantir que qualquer robô sem alvo atribuído vá para o alvo mais próximo disponível.
        """
        if self.id not in self.assignment:  # Robô não tem alvo atribuído
            # Encontre o alvo mais próximo
            closest_target = None
            closest_distance = float('inf')
            current_position = Point(self.robot.x, self.robot.y)

            for target in self.targets:
                distance = current_position.dist_to(target)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_target = target

            if closest_target:
                # Calcular rota até o alvo mais próximo
                adjusted_target = self.avoid_obstacles(current_position, closest_target)
                target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, adjusted_target)
                velocity_factor = 0.5

                # Definir velocidades
                self.set_vel(target_velocity * velocity_factor)
                self.set_angle_vel(target_angle_velocity)

        return
