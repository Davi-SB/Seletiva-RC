from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
import math
import numpy as np

from hungarian import Hungarian

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.assignment = dict()  # Dicionário de atribuição de robôs para alvos

    def decision(self):
        # Nenhum alvo disponível, decision() não faz nada
        if len(self.targets) == 0:
            # Early return, never nested
            return

        # Obter lista de IDs dos robôs disponíveis na equipe
        my_agents = list(self.teammates.keys())

        # Calcular a matriz de custos (distâncias) entre robôs e targets
        cost_matrix = self.calculate_cost_matrix(my_agents)

        # Resolver o problema de atribuição com o algoritmo Hungarian
        assignments = Hungarian.solve(cost_matrix)

        # Atualizar as atribuições
        self.assignment = {my_agents[robot_ID]: self.targets[target_ID] 
                           for robot_ID, target_ID in assignments}

        # Se o robô atual não tiver um alvo atribuído, decision() não faz nada
        if self.id not in self.assignment:
            return
        
        assigned_target  = self.assignment[self.id]
        current_position = Point(self.robot.x, self.robot.y)

        # Ajustar a rota para desviar de obstáculos
        adjusted_target = self.avoid_obstacles(current_position, assigned_target)

        # Calcular e definir velocidades do robô
        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, adjusted_target)
        velocity_factor = 0.5

        self.set_vel(target_velocity * velocity_factor)
        self.set_angle_vel(target_angle_velocity)
        return
    
    def post_decision(self):
        # Se o robô já tem um alvo atribuído, ele deve continuar sua tarefa
        if self.id in self.assignment:
            # Early return, never nested
            return
        
        # Caso contrário, o robô deve se mover para o alvo mais próximo.
        # Se torna um melhor candidato para a atribuição na próxima iteração.
        closest_target = None
        closest_distance = float('inf')
        current_position = Point(self.robot.x, self.robot.y)

        for target in self.targets:
            distance = current_position.dist_to(target)
            if distance < closest_distance:
                closest_distance = distance
                closest_target = target

        if not closest_target:
            return
        
        # Calcular e definir velocidades do robô
        adjusted_target = self.avoid_obstacles(current_position, closest_target)
        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, adjusted_target)
        velocity_factor = 0.5

        self.set_vel(target_velocity * velocity_factor)
        self.set_angle_vel(target_angle_velocity)
        return

    # Calcula a matriz de custos entre os robôs e os alvos.
    # O custo é baseado na distância euclidiana entre cada robô e alvo.
    def calculate_cost_matrix(self, my_agents):
        num_agents = len(my_agents)
        num_targets = len(self.targets)

        # Inicializa as dimensões da matriz de custos
        cost_matrix = np.zeros((num_agents, num_targets))

        # Preenche a matriz de custos
        for i, robot_id in enumerate(my_agents):
            robot = self.teammates[robot_id]  # Posição do robô
            for j, target in enumerate(self.targets):
                # Calcula a distância euclidiana entre o robô e o alvo
                cost_matrix[i, j] = Point(robot.x, robot.y).dist_to(target)

        return cost_matrix

    # Ajusta a rota para desviar de obstáculos
    def avoid_obstacles(self, current_position, target_position):
        # Parâmetro que define a distância mínima segura que os robôs devem manter de obstáculos. 
        # Essa distância é usada para determinar se um robô será ou não repelido por um obstáculo.
        safe_distance = 0.35

        # Fator de ajuste que interfere na intensidade do desvio que será aplicado à rota do robô. 
        # Ajusta o módulo do vetor repulsão.
        # Um valor maior implica em um desvio mais significativo.
        adjustment_factor = 1.7
        
        adjusted_position = target_position
        for robot_id, opponent in self.opponents.items():
            obstacle_position = Point(opponent.x, opponent.y)
            distance_to_obstacle = current_position.dist_to(obstacle_position)

            # Se o não obstáculo estiver dentro da zona de risco, não é necessário desviar dele
            if not distance_to_obstacle < safe_distance:
                # Never nested
                continue
            
            # Obstáculo está dentro da zona de risco e o robô deve desviar dele.
            # Calcula o ângulo entre o robô atual e o obstáculo.
            angle_to_obstacle = math.atan2(
                obstacle_position.y - current_position.y,
                obstacle_position.x - current_position.x,
            )

            # Calcula as componentes do vetor de repulsão em X e Y baseado no seno 
            # e cosseno (projeções nos eixos) do ângulo.
            repulsion_x = -math.cos(angle_to_obstacle) * adjustment_factor
            repulsion_y = -math.sin(angle_to_obstacle) * adjustment_factor

            # Ajusta o destino final para desviar do obstáculo.
            # Rota desvia do obstáculo e continua a busca pelo alvo original.
            adjusted_position = Point(
                adjusted_position.x + repulsion_x,
                adjusted_position.y + repulsion_y,
            )

        return adjusted_position
