import math
th1,th2,th3=-1.363, 0.530, -0.992
l1,l2,l3=0.4,0.4,0.3
n=l2*math.cos(th2)+l3*math.cos(th3+th2)
print(n)
x = math.cos(th1) * n
y = math.sin(th2) * n
z= 0.1 + l1 + l2*math.sin(th2)+l3*math.sin(th3+th2)
print(x,',',y,'',z)