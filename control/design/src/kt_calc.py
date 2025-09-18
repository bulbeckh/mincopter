
## Estimating KT (motor thrust coefficient in Gazebo


pwm_hover = 1610

raw_cmd = (pwm_hover-1100)/(800)

if raw_cmd>1:
    raw_cmd=1
elif raw_cmd<0:
    raw_cmd=0

cmd_out = raw_cmd*838

print(f"At hover, the rotor velocity is: {cmd_out}")

## Calculation of kt

m = 2.43
g = 9.81

## F = mg = 4*kt*(w^2)

kt = m*g / (4*cmd_out*cmd_out)
print(kt)


