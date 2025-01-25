# Projeto: Agente de Planejamento de Trajetórias e Atribuição de Tarefas

Bem-vindo ao repositório do projeto! Este documento detalha o desenvolvimento de um agente autônomo que realiza planejamento de trajetórias e atribuição de tarefas em um ambiente simulado. O projeto explora soluções para problemas complexos, como evitar colisões, desvio de obstáculos dinâmicos e distribuição de tarefas.

## **Visão Geral**

### Objetivos do Projeto

- Implementar lógicas para que o robô desvie de obstáculos enquanto navega até um alvo.
- Distribuir tarefas entre vários robôs, minimizando o tempo total de conclusão e maximizando a eficiência.
- **Execução Eficiente:** Garantir que todos os robôs estejam sempre cooperando, sem ficarem ociosos.

## **Arquivos Importantes**

### 1. **agent.py**

O arquivo principal que define o comportamento do agente.

- **Módulos Implementados:**
  - **Planejamento de Trajetória:** Cálculo de velocidades para o robô ir ao alvo enquanto desvia de obstáculos.
  - **Atribuição de Tarefas:** Uso do algoritmo húngaro para distribuir alvos da melhor forma possível.
  - **Desvio de Obstáculos:** Implementação de desvio baseado em vetores de repulsão.

### 2. **hungarian.py**

Implementação manual do algoritmo húngaro para resolver problemas de atribuição linear.

- **Funções Chave:**
  - `solve`: Resolve a matriz de custos e retorna os pares ótimos de atribuição.
  - `mark_matrix` e `adjust_matrix`: Lógicas internas para cobrir zeros e ajustar a matriz durante a iteração.

## **Funcionalidades do Agente**

### 1. **Atribuição de Tarefas com o Algoritmo Húngaro**

- Resolve o problema de atribuição de robôs aos alvos para minimizar o custo total (tempo e distância).
- Suporte para matrizes de custo retangulares (número de robôs e alvos diferente).
- Evita soluções subótimas, garantindo que o tempo total de coleta dos alvos seja minimizado.
- Eficiência temporal de ordem polinomial

### 2. **Desvio de Obstáculos Dinâmicos**

- Implementação de repulsão vetorial para desviar de robôs oponentes.
- Parâmetros configuráveis para distância segura e intensidade do desvio:
  - `safe_distance`: Define o raio de detecção de obstáculos.
  - `adjustment_factor`: Controla a intensidade do desvio.

### 3. **Manutenção de Atividade**

- Nenhum robô fica ocioso. Robôs sem tarefas atribuídas procuram o alvo mais próximo disponível.
- Implementado no método `post_decision`.

## **Como executar**

Siga as instruções abaixo para configurar o projeto.

### **Dependências**

Certifique-se de instalar as dependências listadas no arquivo `requirements.txt`:

```bash
pip install -r requirements.txt
```

## Como rodar?

Para rodar, basta executar o arquivo `start.py`.
```bash
  python3 start.py
```

Como o projeto possui 4 fases, é possível escolher qual fase rodar utilizando a flag `-d` com o argumento de dificuldade, que vai de 1 a 4:

```bash
  python3 start.py -d [DIFICULDADE]
```

Para tirar dúvidas, use o comando com a flag `-h`:

```bash
  python3 start.py -h
```

⚠️ *OBS:* Caso a instalação das dependências não tenha sido feita em um ambiente virtual e os comandos para rodar não estejam funcionando, tente usar `python3.10` ao invés de `python3`. 
⚠️ *OBS:* Caso tenha problemas com a instalação das dependências do pacote `rc-robosim`ou de CMAKE no ambiente Linux, experimente atualizar o sistema de pacotes do sistema e reinstalar a biblioteca ODE (Open Dynamics Engine) com os comandos: 
```bash
  sudo apt update
  sudo apt upgrade
  sudo apt install libode-dev
```

## **Contribuições**

Sugestões e melhorias são bem-vindas! Sinta-se à vontade para enviar um *pull request*.

## **Licença**

Este projeto está licenciado sob a [MIT License](LICENSE).

## **Contato**

Para mais informações, entre em contato pelo e-mail [davibrilhante0102@gmail.com](mailto\:davibrilhante0102@gmail.com).

