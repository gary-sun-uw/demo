from time import time
import os.path

fname='data.csv'
legend='[s]it s[t]and [r]est [o]ther'

if not os.path.isfile(fname):
    with open(fname,'w') as f:
        f.write('time,state\n')

idx=0

while True:
    with open(fname,'a') as f:
        try:
            state=input(f'Enter one of {legend}: ')
            f.write(f'{idx},{time()},{state}\n')
            idx+=1
        except EOFError:
            break
