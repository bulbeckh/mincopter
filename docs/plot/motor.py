
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def plot_motors(mc):

    fig = plt.figure(figsize=(4,1))
    gs = gridspec.GridSpec(4,1, figure=fig)

    ax = fig.add_subplot(gs[0,0])
    ax.plot(list([e[0] for e in mc.mot1]), list([e[1] for e in mc.mot1]), color='blue', linestyle='--', label='Motor 1 (rad/s)')
    ax.plot(range(0,len(mc.mot1)), len(mc.mot1)*[1100], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.mot1)), len(mc.mot1)*[1900], color='red', linestyle='solid')
    ax.set_title(f'Motor 1 (rad/s)')
    ax.legend()
    ax.grid(True)

    plt.show()
    return

