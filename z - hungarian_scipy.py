from scipy.optimize import linear_sum_assignment
import numpy as np


cost = np.array([ [5, 8, 3], [9, 5, 18], [17, 3, 1] ])
row_ind, col_ind = linear_sum_assignment(cost)

print(row_ind, col_ind, end="\n\n")
print(cost[row_ind, col_ind].sum())