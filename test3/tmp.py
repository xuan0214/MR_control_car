import keyboard


def act(x):
    e = keyboard.KeyboardEvent('down', 28, 'enter')

    if x.event_type == 'down' and x.name == e.name:
        print("你按下了 enter 键")

keyboard.hook(act)
keyboard.wait
