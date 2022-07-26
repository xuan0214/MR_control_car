import keyboard

def act(x) :
    W_1 = keyboard.KeyboardEvent('up',17,'w')
    A_1 = keyboard.KeyboardEvent('up',30,'a')
    S_1 = keyboard.KeyboardEvent('up',31,'s')
    D_1 = keyboard.KeyboardEvent('up',32,'d')

    W_2 = keyboard.KeyboardEvent('down',17,'w')
    A_2 = keyboard.KeyboardEvent('down',30,'a')
    S_2 = keyboard.KeyboardEvent('down',31,'s')
    D_2 = keyboard.KeyboardEvent('down',32,'d')

    if x.event_type == 'down' and x.name == W_2.name :
        print('1')

    elif x.event_type == 'down' and x.name == A_2.name :
        print('2')

    elif x.event_type == 'down' and x.name == D_2.name :
        print('3')

    elif x.event_type == 'down' and x.name == S_2.name :
        print('4')
    
    elif x.event_type == 'up' :
        print('5')


print('start program')


keyboard.hook(act)
keyboard.wait()

