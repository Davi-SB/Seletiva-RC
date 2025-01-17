from rsoccer_gym.Entities import Robot
from utils.Point import Point

class BaseAgent:
    """Abstract Agent."""

    def __init__(self, id=0, yellow=False):
        self.id = id  # ID único do robô
        self.robot = Robot()  # Objeto representando o robô controlado
        self.pos = Point(0, 0)  # Posição inicial do robô
        self.vel = Point(0, 0)  # Velocidade inicial do robô
        self.body_angle = float(0)  # Ângulo inicial do corpo do robô
        self.targets = []  # Lista de alvos para o robô
        self.yellow = yellow  # Cor do time (True para amarelo)
        self.opponents = dict()  # Dicionário de robôs adversários
        self.teammates = dict()  # Dicionário de robôs aliados

        # Controle de movimento
        self.next_vel = Point(0, 0)  # Próxima velocidade a ser atribuída
        self.angle_vel = float(0)  # Velocidade angular a ser atribuída


    def step(self, 
             self_robot: Robot, 
             opponents:  dict[int, Robot] = dict(), 
             teammates:  dict[int, Robot] = dict(), 
             targets:    list[Point] = [], 
             keep_targets=False) -> Robot:

        # Zera as velocidades linear e angular do robô
        self.reset()
        
        # Atualiza as informações do estado atual do robô
        self.pos = Point(self_robot.x, self_robot.y)
        self.vel = Point(self_robot.v_x, self_robot.v_y)
        self.body_angle = self_robot.theta

        if len(targets) > 0:
            self.targets = targets.copy()
        elif len(self.targets) == 0 or not keep_targets:
            self.targets = []
            
        self.robot = self_robot
        self.opponents = opponents.copy()
        self.teammates = teammates.copy()

        self.decision()
        self.post_decision()

        return Robot( id=self.id, yellow=self.yellow,
                      v_x=self.next_vel.x, v_y=self.next_vel.y, v_theta=self.angle_vel)

    def reset(self):
        self.next_vel = Point(0, 0)
        self.angle_vel = 0

    def decision(self):
        raise NotImplementedError()
    
    def post_decision(self):
        raise NotImplementedError()
    
    def set_vel(self, vel: Point):
        self.next_vel = vel
    
    def set_angle_vel(self, angle_vel: float):
        self.angle_vel = angle_vel


""" 
"from rsoccer_gym.Entities import Robot" -->

@dataclass()
class Robot:
    yellow: bool = None
    id: int = None
    x: float = None
    y: float = None
    z: float = None
    theta: float = None
    v_x: float = 0
    v_y: float = 0
    v_theta: float = 0
    kick_v_x: float = 0
    kick_v_z: float = 0
    dribbler: bool = False
    infrared: bool = False
    wheel_speed: bool = False
    v_wheel0: float = 0 # rad/s
    v_wheel1: float = 0 # rad/s
    v_wheel2: float = 0 # rad/s
    v_wheel3: float = 0 # rad/s
"""