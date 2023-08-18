from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df_test=pd.DataFrame(data=np.random.normal(0,1,(20,3)),columns=['a','b','c'])
fig=plt.figure()
ax=fig.add_subplot(111,projection='3d')
ax.scatter=(df_test['a'],df_test['b'],df_test['c'])
ax.set_xlabel('a')
ax.set_ylabel('b')
ax.set_zlabel('c')
plt.show()