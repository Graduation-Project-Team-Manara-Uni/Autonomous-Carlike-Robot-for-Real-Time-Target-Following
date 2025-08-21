
a = [10,10,10,5,5,5,10]
L = min(a)
ids = [i for i,val in enumerate(a) if val == L]
print(L)
print(ids)
