from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
import math
import numpy as np

# from scipy.optimize import linear_sum_assignment # Algoritmo de Hungarian - https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.linear_sum_assignment.html#scipy.optimize.linear_sum_assignment
from hungarian import my_linear_sum_assignment as assign_targets

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        # Além dos atributos herdados de BaseAgent, ExampleAgent possui um atributo adicional:
        # Dicionário de atribuição {robot_id: target}
        # Relaciona cada ID de robô com o alvo atribuído a ele
        # Caso um robô não tenha alvo atribuído, seu ID estará presente no dicionário
        self.assignment = dict()  # Dicionário de atribuição {robot_id: target}

    def decision(self):
        # Se não houver alvos disponíveis, não há decisão a ser tomada, agente fica parado
        if len(self.targets) == 0:  
            # Early return, never nested
            return

        # Obtem lista de apenas os IDs dos robôs disponíveis na equipe
        my_agents = list(self.teammates.keys())
        
        # A primeira etapa é calcular a matriz de custos (distâncias) entre robôs e alvos
        # Cada linha da matriz representa um robô disponível
        # Cada coluna da matriz representa um alvo disponível
        cost_matrix = self.calculate_cost_matrix(my_agents)

        # A segunda etapa é resolver o problema de atribuição: qual robô deve ser atribuído a qual alvo
        # Para isso, utilizamos assign_targets, com algoritmo baseado no Hungarian Algorithm
        # O algoritmo retorna uma lista de tuplas (robot_ID, target_ID) que indica qual robô deve 
        # ser atribuído a qual alvo
        # É recaluado a cada decisão devido à dinamicidade do ambiente, garantindo eficiencia e adaptabilidade
        assignments = assign_targets(cost_matrix)

        # Por isso, a terceira etapa é atualizar o dicionário de atribuições
        # Cada agente é relacionado ao alvo que lhe foi atribuído (objeto, e não mais ID)
        self.assignment = {my_agents[robot_ID]: self.targets[target_ID] 
                           for robot_ID, target_ID in assignments}

        # A quarta e última etapa do método decision é executar a tarefa se o robô atual 
        # tiver um alvo atribuído. 
        # Se o robô não tiver alvo atribuído, o método decision retorna, sem atribuir tarefas ao robô
        if self.id not in self.assignment:
            # Never nested
            return
        
        # Nesse ponto, o robô tem um alvo atribuído
        # Portanto, por boas práticas e legibilidade, o objeto alvo é armazenado em assigned_target 
        # e a posição atual do robô é armazenada em current_position
        assigned_target  = self.assignment[self.id]
        current_position = Point(self.robot.x, self.robot.y)

        # Porém, o robô deve evitar obstáculos em vez de ir diretamente ao alvo
        # Para isso, é calculado um alvo ajustado, que leva em consideração a presença de obstáculos
        # e causa um desvio na rota do robô
        adjusted_target = self.avoid_obstacles(current_position, assigned_target)

        # goToPoint, da biblioteca de navegação, calcula as velocidades (linear e angular) para o 
        # alvo ajustado, levando em consideração a situacao atual do robô (posição, orientação, etc)
        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, adjusted_target)
        
        # Por fim, as velocidades calculadas são ajustadas por um fator e definidas para o robô.
        # - Esse fator é utilizado para ajustar a velocidade do agente em relação à padrão 
        # considerada por goToPoint. 
        # - A existencia e ajuste desse fator garante uma velocidade mais segura e controlada 
        # para o robô, pois permite que o vetor repulsão resultante de avoid_obstacles interfira 
        # mais (e corretamente) em sua velocidade
        # - Apesar de ajustada, a velocidade final ainda garante eficiencia e rapidez na execução
        velocity_factor = 0.5
        self.set_vel(target_velocity * velocity_factor)
        self.set_angle_vel(target_angle_velocity)
        return

    # Esse método é chamado após o método decision e atribui tarefas (alvos) aos robôs ociosos, 
    # num momento em que já alcançou seu alvo atribuído, mas ainda há alvos disponíveis no round.
    # Apesar de já ter conquistado seu alvo atribuído incialmente, o robô ainda pode ser útil para
    # auxiliar outros robôs a alcançarem seus alvos, maximizando ainda mais a eficiencia do time.
    #
    # Isso ocorre visto que, mesmo que não seja o melhor candidato para um alvo levando em consideração
    # as distâncias (matriz de custos), ele pode se tornar um bom canditado na medida em que se aproxima 
    # do alvo mais próximo e/ou na medida em que obstáculos atrapalham o caminho de outros robôs
    def post_decision(self):
        # Se o robô já tem um alvo atribuído, ele deve continuar sua tarefa
        if self.id in self.assignment:
            # Early return, never nested
            return
        
        # Como o robô não tem alvo atribuído, procura-se o mais próximo dele
        closest_target   = None
        closest_distance = float('inf') # valor arbitrariamente alto
        current_position = Point(self.robot.x, self.robot.y)

        for target in self.targets:
            # Calcula a distância entre o robô e o alvo
            distance = current_position.dist_to(target)
            # Se a distância for menor que a menor distância já encontrada, atualiza a menor distância
            if distance < closest_distance:
                closest_distance = distance
                closest_target   = target

        # Se não houver alvo, o robô não tem tarefa a ser atribuída
        if not closest_target:
            # Never nested
            return
        
        # Calcula a rota como feito no método decision e ajusta o alvo para evitar obstáculos
        adjusted_target = self.avoid_obstacles(current_position, closest_target)
        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, adjusted_target)

        velocity_factor = 0.5
        self.set_vel(target_velocity * velocity_factor)
        self.set_angle_vel(target_angle_velocity)
        return

    # Calcula a matriz de custos (distâncias) entre os robôs e os alvos.
    # Cada linha da matriz representa um robô disponível, 
    # bem como cada coluna representa um alvo disponível
    #
    # O custo é baseado na distância euclidiana (distância em linha reta, 
    # por definição, a menor possível) entre cada robô e cada alvo.
    def calculate_cost_matrix(self, my_agents):
        num_agents = len(my_agents)
        num_targets = len(self.targets)
        
        # Inicializa a matriz de custos (linha, coluna) com zeros
        cost_matrix = np.zeros((num_agents, num_targets))

        # Calcula a distância entre cada robô e alvo e armazena na matriz de custos
        for i, robot_id in enumerate(my_agents):
            robot = self.teammates[robot_id]
            # Para cada robô, calcula-se sua distância para cada alvo
            for j, target in enumerate(self.targets):
                cost_matrix[i, j] = Point(robot.x, robot.y).dist_to(target)

        return cost_matrix

    def avoid_obstacles(self, current_position, target_position):
        # Parâmetro que define a distância mínima segura que os robôs devem manter de obstáculos. 
        # Essa distância é usada para determinar se um robô será ou não repelido por um obstáculo.
        safe_distance = 0.35

        # Fator de ajuste que interfere na intensidade do desvio que será aplicado à rota do robô. 
        # Ajusta o módulo do vetor repulsão.
        # Um valor maior implica em um desvio mais significativo.
        adjustment_factor = 1.7
        
        # Inicializa a posição ajustada como sendo igual ao destino desejado originalmente.
        # Essa variável será modificada na medida em que forem detectados obstáculos 
        # dentro da zona de risco.
        adjusted_position = target_position

        # Itera sobre todos os oponentes, que são tratados como obstáculos em potencial.
        for robot_id, opponent in self.opponents.items():
            # Calcula a distância do robô atual até o obstáculo para verificar se ele 
            # está dentro da zona de risco.
            obstacle_position = Point(opponent.x, opponent.y)
            distance_to_obstacle = current_position.dist_to(obstacle_position)

            # Se o não obstáculo estiver dentro da zona de risco, não é necessário desviar dele
            if not distance_to_obstacle < safe_distance:
                # Never nested
                continue
            
            # Obstáculo está dentro da zona de risco e o robô deve desviar dele.
            # Calcula o ângulo entre o robô atual e o obstáculo, usando arctan2.
            # Esse ângulo é usado para determinar a direção do vetor de repulsão.
            angle_to_obstacle = math.atan2(
                obstacle_position.y - current_position.y,  # Diferença de coordenada Y.
                obstacle_position.x - current_position.x,  # Diferença de coordenada X.
            )

            # Calcula as componentes do vetor de repulsão em X e Y baseado no seno 
            # e cosseno (projeções nos eixos) do ângulo.
            # A repulsão é calculada de forma oposta à direção do obstáculo (por isso o sinal negativo).
            repulsion_x = -math.cos(angle_to_obstacle) * adjustment_factor
            repulsion_y = -math.sin(angle_to_obstacle) * adjustment_factor

            # Ajusta o destino final para desviar do obstáculo, adicionando as componentes
            # do vetor de repulsão à posição de destino. 
            # Isso resulta em uma rota que desvia do obstáculo e continua a busca pelo alvo original.
            adjusted_position = Point(
                adjusted_position.x + repulsion_x,  # Ajusta a coordenada X do destino.
                adjusted_position.y + repulsion_y,  # Ajusta a coordenada Y do destino.
            )

        return adjusted_position