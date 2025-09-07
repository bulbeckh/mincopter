

data = []

if __name__=="__main__":

    with open('./screenlog.0', 'r') as rfile:
        for idx, g in enumerate(rfile.readlines()):
            i = g.split(',')

            data.append({
                'ax': float(i[0]),
                'ay': float(i[1]),
                'az': float(i[2]),
                'gx': float(i[3]),
                'gy': float(i[4]),
                'gz': float(i[5]),
                'mx': float(i[6]),
                'my': float(i[7]),
                'mz': float(i[8]),
                'dt': (0 if idx==0 else (int(i[9].strip('\n')) - last_time))
                })

            last_time = int(i[9].strip('\n'))

    print(data)

    ## Implement EKF and other code


