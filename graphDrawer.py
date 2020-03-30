import matplotlib.pyplot as plt
 
baselineFile = open('entropy_data_baseline.txt','r')
baselineData = baselineFile.readlines()

selfAlgoFile = open('entropy_data_selfAlgo.txt','r')
selfAlgoData = selfAlgoFile.readlines()

x = range(max([len(baselineData), len(selfAlgoData)]))
 
fig,ax = plt.subplots()
 
plt.xlabel('time')
plt.ylabel('entropy')
 
minValue = float(min([min(baselineData), min(selfAlgoData)]))
maxValue = float(max([max(baselineData), max(selfAlgoData)]))
yticks = range(int(minValue),int(maxValue),1)
#yticks = range(10,110,10)
ax.set_yticks(yticks)
 
ax.set_ylim([minValue,maxValue])
ax.set_xlim([0, len(x)])
 
plt.plot(x,baselineData,"x-",label="baseline Data")
plt.plot(x,selfAlgoData,"+-",label="selfAlgo Data")
 
"""open the grid"""
plt.grid(True)
 
plt.legend(bbox_to_anchor=(1.0, 1), loc=1, borderaxespad=0.)
 
plt.show()

