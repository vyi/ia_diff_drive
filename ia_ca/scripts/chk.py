def getXbot(xb):
    indices = [2,3,4,5,0,1]
    temp = [0 for i in xb]

    jdx = 0
    for idx in indices:
        temp[jdx] = xb[idx]
        jdx = jdx + 1

    return temp

if __name__ == '__main__':
    #print(getXbot(['c','d','e','f','a','b']))
    print(getXbot(['a','b','c','d','e','f']))