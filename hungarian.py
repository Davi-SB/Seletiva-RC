import numpy as np

# OBS: idealmente, esse arquivo estaria salvo na pasta "utils" do projeto. 
#      Porém, para simplificar a compreensão do que foi alterado/criado por mim, 
#      ele foi salvo na pasta raiz do projeto.

def my_linear_sum_assignment(cost_matrix):
    def min_zero_row(zero_matrix, marked_zeros):
        """
        Encontra a linha com o menor número de zeros disponíveis e marca
        um desses zeros (linha, coluna) para a solução.
        """
        min_row = [float('inf'), -1]

        # Identificar a linha com o menor número de zeros
        for row in range(zero_matrix.shape[0]):
            num_zeros = np.sum(zero_matrix[row])
            if 0 < num_zeros < min_row[0]:
                min_row = [num_zeros, row]

        # Marcar o zero encontrado e eliminar sua linha e coluna
        col_index = np.where(zero_matrix[min_row[1]])[0][0]
        marked_zeros.append((min_row[1], col_index))
        zero_matrix[min_row[1], :] = False
        zero_matrix[:, col_index] = False

    def mark_matrix(matrix):
        """
        Determina os zeros que serão marcados como solução inicial e
        quais linhas e colunas devem ser cobertas.
        """
        zero_matrix = (matrix == 0)
        marked_zeros = []

        # Marcar zeros válidos
        while np.any(zero_matrix):
            min_zero_row(zero_matrix, marked_zeros)

        # Identificar linhas e colunas a serem cobertas
        marked_zero_rows = {row for row, _ in marked_zeros}
        marked_zero_cols = {col for _, col in marked_zeros}

        uncovered_rows = set(range(matrix.shape[0])) - marked_zero_rows
        covered_cols = set()

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

        covered_rows = set(range(matrix.shape[0])) - uncovered_rows
        return covered_rows, covered_cols

    def adjust_matrix(matrix, covered_rows, covered_cols):
        """
        Ajusta a matriz subtraindo o menor elemento não coberto e adicionando
        aos elementos cobertos duas vezes.
        """
        uncovered_values = [
            matrix[row, col]
            for row in range(matrix.shape[0])
            for col in range(matrix.shape[1])
            if row not in covered_rows and col not in covered_cols
        ]
        min_value = min(uncovered_values)

        for row in range(matrix.shape[0]):
            for col in range(matrix.shape[1]):
                if row not in covered_rows and col not in covered_cols:
                    matrix[row, col] -= min_value
                elif row in covered_rows and col in covered_cols:
                    matrix[row, col] += min_value

    # Garantir que a matriz seja quadrada
    num_rows, num_cols = cost_matrix.shape
    if num_rows != num_cols:
        size = max(num_rows, num_cols)
        padded_matrix = np.zeros((size, size))
        padded_matrix[:num_rows, :num_cols] = cost_matrix
        cost_matrix = padded_matrix

    # Passo 1: Normalizar a matriz
    for row in range(cost_matrix.shape[0]):
        cost_matrix[row] -= np.min(cost_matrix[row])

    for col in range(cost_matrix.shape[1]):
        cost_matrix[:, col] -= np.min(cost_matrix[:, col])

    # Passos 2 e 3: Iterar até que todas as linhas e colunas sejam cobertas
    while True:
        covered_rows, covered_cols = mark_matrix(cost_matrix)
        total_covered = len(covered_rows) + len(covered_cols)

        if total_covered >= cost_matrix.shape[0]:
            break

        adjust_matrix(cost_matrix, covered_rows, covered_cols)

    # Passo 4: Identificar a solução final (pares de índices)
    zero_matrix = (cost_matrix == 0)
    marked_zeros = []
    while np.any(zero_matrix):
        min_zero_row(zero_matrix, marked_zeros)

    row_indices, col_indices = zip(*marked_zeros)

    # Filtrar apenas as atribuições válidas (dentro do tamanho original da matriz)
    row_indices = np.array(row_indices)
    col_indices = np.array(col_indices)
    valid_indices = (row_indices < num_rows) & (col_indices < num_cols)

    return zip(row_indices[valid_indices], col_indices[valid_indices])
