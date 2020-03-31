import matplotlib.pyplot as plt
 
baselineFile = open('entropy_data_baseline.txt','r')
baselineData = baselineFile.readlines()
baselineData = filter(None,map(lambda x: x.strip('\n'), baselineData))
print(len(baselineData))

selfAlgoFile = open('entropy_data_selfAlgo.txt','r')
selfAlgoData = selfAlgoFile.readlines()
selfAlgoData = filter(None,map(lambda x: x.strip('\n'), selfAlgoData))
print(len(selfAlgoData))


x = range(max([len(baselineData), len(selfAlgoData)]))
 
fig,ax = plt.subplots()
 
plt.xlabel('time')
plt.ylabel('entropy')
 
minValue = float(min([min(baselineData),min(selfAlgoData)]))
maxValue = float(max([max(baselineData), max(selfAlgoData)]))

yticks = range(int(minValue),int(maxValue),1000)
ax.set_yticks(yticks)
 
ax.set_ylim([minValue,maxValue])
ax.set_xlim([0, len(x)])
 
plt.plot(x,baselineData,"x-",label="Baseline Algorithm")
plt.plot(x,selfAlgoData,"+-",label="Wave Font Detection Algorithm")

plt.grid(True)
 
plt.legend(bbox_to_anchor=(1.0, 1), loc=1, borderaxespad=0.)
 
plt.show()

