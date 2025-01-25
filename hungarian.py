# OBS: idealmente, esse arquivo estaria salvo na pasta "utils" do projeto. 
#      Porém, para simplificar a compreensão do que foi alterado/criado por mim, 
#      ele foi salvo na pasta raiz do projeto.

import numpy as np

class Hungarian:
    @staticmethod
    def min_zero_row(zero_matrix, marked_zeros):
        """
        Encontra a linha com o menor número de zeros disponíveis e marca
        um desses zeros (linha, coluna) para a solução.
        """
        min_row = [float('inf'), -1]  # Armazena o menor número de zeros e o índice da linha correspondente

        # Identificar a linha com o menor número de zeros
        for row in range(zero_matrix.shape[0]):
            num_zeros = np.sum(zero_matrix[row])  # Conta o número de zeros na linha atual
            if 0 < num_zeros < min_row[0]:  # Procura a linha com o menor número de zeros
                min_row = [num_zeros, row]

        # Marcar o zero encontrado e eliminar sua linha e coluna
        col_index = np.where(zero_matrix[min_row[1]])[0][0]  # Localiza o índice da coluna do zero
        marked_zeros.append((min_row[1], col_index))  # Adiciona a marcação do zero encontrado
        zero_matrix[min_row[1], :] = False  # Remove todos os zeros da linha
        zero_matrix[:, col_index] = False  # Remove todos os zeros da coluna

    @staticmethod
    def mark_matrix(matrix):
        """
        Determina os zeros que serão marcados como solução inicial e
        quais linhas e colunas devem ser cobertas.
        """
        zero_matrix = (matrix == 0)  # Cria uma matriz booleana onde zeros são True
        marked_zeros = []  # Lista para armazenar as marcações dos zeros

        # Marcar zeros válidos
        while np.any(zero_matrix):  # Continua enquanto existirem zeros na matriz
            Hungarian.min_zero_row(zero_matrix, marked_zeros)

        # Identificar linhas e colunas a serem cobertas
        marked_zero_rows = {row for row, _ in marked_zeros}  # Linhas com zeros marcados
        marked_zero_cols = {col for _, col in marked_zeros}  # Colunas com zeros marcados

        uncovered_rows = set(range(matrix.shape[0])) - marked_zero_rows  # Linhas não cobertas
        covered_cols = set()  # Colunas cobertas

        # Iterativamente encontrar colunas cobertas e atualizar linhas não cobertas
        while True:
            changed = False
            for row in list(uncovered_rows):
                for col in range(matrix.shape[1]):
                    if matrix[row, col] == 0 and col not in covered_cols:
                        covered_cols.add(col)
                        changed = True
            for row, col in marked_zeros:
                if row not in uncovered_rows and col in covered_cols:
                    uncovered_rows.add(row)
                    changed = True
            if not changed:
                break

        covered_rows = set(range(matrix.shape[0])) - uncovered_rows  # Linhas cobertas
        return covered_rows, covered_cols

    @staticmethod
    def adjust_matrix(matrix, covered_rows, covered_cols):
        """
        Ajusta a matriz subtraindo o menor elemento não coberto e adicionando
        aos elementos cobertos duas vezes.
        """
        # Coleta todos os valores não cobertos
        uncovered_values = [
            matrix[row, col]
            for row in range(matrix.shape[0])
            for col in range(matrix.shape[1])
            if row not in covered_rows and col not in covered_cols
        ]
        min_value = min(uncovered_values)  # Menor valor não coberto

        # Subtrai o menor valor dos elementos não cobertos e soma aos cobertos duas vezes
        for row in range(matrix.shape[0]):
            for col in range(matrix.shape[1]):
                if row not in covered_rows and col not in covered_cols:
                    matrix[row, col] -= min_value
                elif row in covered_rows and col in covered_cols:
                    matrix[row, col] += min_value

    @staticmethod
    def solve(cost_matrix):
        """
        Resolve o problema de atribuição linear utilizando o algoritmo húngaro.
        Retorna os pares de índices das atribuições.
        """
        # Garantir que a matriz seja quadrada
        num_rows, num_cols = cost_matrix.shape
        if num_rows != num_cols:
            size = max(num_rows, num_cols)  # Determina o tamanho da matriz quadrada
            padded_matrix = np.zeros((size, size))  # Cria uma matriz quadrada preenchida com zeros
            padded_matrix[:num_rows, :num_cols] = cost_matrix  # Copia a matriz original para a nova matriz
            cost_matrix = padded_matrix

        # Passo 1: Normalizar a matriz
        for row in range(cost_matrix.shape[0]):
            cost_matrix[row] -= np.min(cost_matrix[row])  # Subtrai o menor valor de cada linha

        for col in range(cost_matrix.shape[1]):
            cost_matrix[:, col] -= np.min(cost_matrix[:, col])  # Subtrai o menor valor de cada coluna

        # Passos 2 e 3: Iterar até que todas as linhas e colunas sejam cobertas
        while True:
            covered_rows, covered_cols = Hungarian.mark_matrix(cost_matrix)  # Identifica linhas e colunas cobertas
            total_covered = len(covered_rows) + len(covered_cols)

            if total_covered >= cost_matrix.shape[0]:  # Se todas as linhas ou colunas forem cobertas, sair do loop
                break

            Hungarian.adjust_matrix(cost_matrix, covered_rows, covered_cols)  # Ajusta a matriz para cobrir mais zeros

        # Passo 4: Identificar a solução final (pares de índices)
        zero_matrix = (cost_matrix == 0)  # Cria uma matriz booleana para identificar zeros
        marked_zeros = []
        while np.any(zero_matrix):  # Enquanto existirem zeros na matriz
            Hungarian.min_zero_row(zero_matrix, marked_zeros)

        row_indices, col_indices = zip(*marked_zeros)  # Extrai as linhas e colunas das marcações

        # Filtrar apenas as atribuições válidas (dentro do tamanho original da matriz)
        row_indices = np.array(row_indices)
        col_indices = np.array(col_indices)
        valid_indices = (row_indices < num_rows) & (col_indices < num_cols)

        return zip(row_indices[valid_indices], col_indices[valid_indices])
