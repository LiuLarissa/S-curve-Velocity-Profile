import matplotlib.pyplot as plt
import matplotlib as mpl
import pylab

def plotting(time,value,name_x,name_y,name_graph,label):
    plt.plot(time,value,label=label)
    plt.xlabel(name_x,fontsize=13)
    plt.ylabel(name_y,fontsize=13)
    plt.title(name_graph,fontsize=13)
    plt.show()
