import numpy as np

decimal_point = 4
print(f'Convert radian to degree ({decimal_point})')
while (_in:=input('Enter: ')) != 'q':
    # print(_in)
    val = float(_in)
    # print(f'from {val} to {round((val * np.pi / 180.0), decimal_point)}')
    print(f'from {val} to {round(val * 180 / np.pi, decimal_point)}')