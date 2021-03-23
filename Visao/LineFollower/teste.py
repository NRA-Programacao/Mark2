import numpy as np

a = np.array([1.2, -2.1, 3.])

b = np.array([1.1, 0, 2.5])
dif = a-b
value = a[np.less(dif, 0.1)]


print(value)