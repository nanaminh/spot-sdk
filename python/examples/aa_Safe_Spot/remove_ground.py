import numpy as np
#from sklearn import linear_model

#--- Read the plane data 

with open('left_ground_pitch-0.1.txt', 'r') as infile, open('left_ground_pitch_0620test.txt', 'w') as outfile:
    temp = infile.read().replace("[", "")
    temp = temp.replace("]", "")
    outfile.write(temp)

data = np.loadtxt("left_ground_pitch_0620test.txt", dtype=float)


#print(data)

#--- Linear regression (use x axis as the out put because that is the height)
X_data = data[:, 1:]
Y_data = data[:, 0]

reg = linear_model.LinearRegression().fit(X_data, Y_data)
print("coefficients of equation of plane, (a1, a2): ", reg.coef_)

print("value of intercept, c:", reg.intercept_)

