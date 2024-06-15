import math
from Kins import ForwardKins, InvKins
test = ForwardKins([0, -math.pi, 0, 0,0, 0], 400, 250, 100, 'PUMA', re=0, rf=0, e=0,
                                f=0)
print(test)
test = InvKins([[-149.09, 0, -392.8799999999999,0, 0, 0]], 400, 250, 100, [], 'PUMA', re=0, rf=0, e=0,
                                f=0)
print(test)