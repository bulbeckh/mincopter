
## Heads-up display for interpreting control and planner variables 

import time
import matplotlib.pyplot as plt

# state variables

class CopterState:

    def __init__(self):

        loop_iter=0

        ## Planner desired x,y,z
        self.planner_loiterstep = []
        self.planner_target_x = []
        self.planner_target_y = []
        self.planner_target_z = []

        ## Roll, Pitch, Yaw, Throttle desired values
        self.controller_desired_r = []
        self.controller_desired_p = []
        self.controller_desired_y = []
        self.controller_desired_t = []

        return

    def parse_state(self, ln):
        pass

    def parse_planner(self, ln):
        parsed = ln.split(",")
        self.planner_loiterstep.append(parsed[0])
        self.planner_target_x.append(parsed[1])
        self.planner_target_y.append(parsed[2])
        self.planner_target_z.append(parsed[3])
        return
    
    def parse_controller(self, ln):
        parsed = ln.split(",")
        self.controller_desired_r.append(parsed[0])
        self.controller_desired_p.append(parsed[1])
        self.controller_desired_y.append(parsed[2])
        self.controller_desired_t.append(parsed[3])
        return

def replot(copter, fig, ax):

    [a.clear() for a in x]

    ax[0].plot(copter.loop_iter, copter.controller_desired_r, market='o')
    ax[0].set_title('roll')

    ax[1].plot(copter.loop_iter, copter.controller_desired_p, market='o')
    ax[1].set_title('pitch')

    ax[2].plot(copter.loop_iter, copter.controller_desired_y, market='o')
    ax[2].set_title('yaw')

    fig.tight_layout()
    fig.canvas.draw()
    fig.canvas.flush_events()
    
    return

def parse_line(copter, line):
    if line[0]=='s':
        copter.parse_state(line[1:])
    elif line[0]=='p':
        copter.parse_planner(line[1:])
    elif line[0]=='m':
        copter.parse_motor(line[1:])
    elif line[0]=='c':
        copter.parse_controller(line[1:])
    elif line[0]=='i':
        ## increase iteration
        copter.loop_iter = int(line[1:])
    else:
        print("ERROR: incorrect variables")


if __name__=="__main__":
    plt.ion()
    fig, ax =  plt.subplots(3,1, figsize=(10,4))

    copter = CopterState()

    with open('../../build/standard.mcsimlog', 'r') as rfile:
        rfile.seek(0,2)
        linecounter=0
        while True:
            line = rfile.readline()
            if len(line)==0:
                time.sleep(0.05)
                continue
            else:
                linecounter+=1
                print(linecounter)
            parse_line(copter, line)

            replot(copter, fig, ax)
            plt.pause(0.05)






