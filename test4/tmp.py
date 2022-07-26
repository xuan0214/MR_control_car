ch = input("front: w, back: s, left: a, right: d\n")
car_v = 0
v_mode = 0

while True :
    if ch == 'w' :
        if v_mode != 1 :
            v_mode = 1
            car_v = 1
	    	
        else :
            car_v += 1
		
        print('Front!\n')

    elif ch == 'a' :
        if v_mode != 2 :
            v_mode = 2
            car_v = 1
	    	
        else :
            car_v += 1
        print('Left!\n')

    elif ch == 'd':
        if v_mode != 3 :
            v_mode = 3
            car_v = 1
	    	
        else :
            car_v += 1
        print('Right!\n')

    elif ch == 's' :
        if v_mode != 4 :
            v_mode = 4
            car_v = 1
	    	
        else :
            car_v += 1
        print('Back!\n')

    elif ch == ' ' :
        v_mode = 0
        car_v = 0
        print('Stop!\n')
    
    print('mode = %d\n',v_mode)
    print('car_v = %d\n',car_v)
    ch = input("front: w, back: s, left: a, right: d\n")
