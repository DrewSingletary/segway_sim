import pdb
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle 
from matplotlib.animation import FuncAnimation
import scipy.io as sio
from tempfile import TemporaryFile



def saveGit(name, xaxis, variableAnimate, color, labels, yLimits):
	fig = plt.figure()
	fig.set_tight_layout(True)
	ax = plt.axes()
	lineList = []
	for i in range(0, len(variableAnimate)):
		line, = ax.plot([], [], color[i], label=labels[i],zorder=1)
		lineList.append(line)
	plt.legend()

	def update(i):
		for j in range(0, len(variableAnimate)):
			lineList[j].set_data(xaxis[0:i], variableAnimate[j][0:i])
			ax.set(xlim=(0, i+1), ylim=yLimits)
	dataPoints = variableAnimate[0].shape[0]

	anim = FuncAnimation(fig, update, frames=np.arange(0, dataPoints+1), interval=100)
	anim.save(name+'.gif', dpi=80, writer='imagemagick')

def main():
	# 	saveGit('prob', time_belief, [np.array(probMiss), probObstArray[:,0], probObstArray[:,1], probObstArray[:,2], probObstArray[:,3]], ['-k','-ob','--sb','-og','--sg'],['Mission', 'R1', 'R2', 'G1', 'G2'], (-0.1, 1.3))
	with open('test.npy', 'rb') as f:
		a = np.load(f)
		b = np.load(f)
		c = np.load(f)
	
	pdb.set_trace()
	T = 100
	saveGit('compTime', a[0:T], [np.array(b[0:T]), np.array(c[0:T])], ['-k','-r'],['Segway MPC', 'Wheeled bot MPC'], (-0.05, 0.5))

main()

## Read and plot STATE
