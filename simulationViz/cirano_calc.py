import math

w = 22
h = 17

def calcFixedForwards(h1, o1):
    o1 = o1 * 3.14159 / 180.
    l1 = h1/math.cos(o1)
    l2 = math.sqrt(math.pow(l1*math.sin(o1) - w, 2) + math.pow(h1 + h,2))
    o2 = math.acos((h1 + h)/l2)
    print(f"{o1 * 180 / 3.14159}, {l1}, {o2 * 180 / 3.14159}, {l2}")
    
calcFixedForwards(26.6, 31)
