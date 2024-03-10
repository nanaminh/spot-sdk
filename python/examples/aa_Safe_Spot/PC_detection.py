import numpy as np  
import matplotlib.pyplot as plt
#from sklearn.cluster import DBSCAN

def remove_ground(coefficient, PCs, threshold, body_height=0):
    no_ground = (PCs[:, 0] < (coefficient[0]*PCs[:,1] + coefficient[1]*PCs[:, 2] + coefficient[2] -threshold + body_height ))
    PCs = PCs[no_ground,:]

    return PCs

def scatter_plot(data, data_2):

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(xs= data[:, 0], ys=data[:, 1], zs=data[:, 2], c= '#808080')
    ax.scatter(xs= data_2[:, 0], ys=data_2[:, 1], zs=data_2[:, 2], c= 'b')
    #ax.scatter(xs= data_3[:, 0], ys=data_3[:, 1], zs=data_3[:, 2], c= 'r')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

def remove_outliers(PCs, epsilon= 0.3, min_samples= 10, ):
    db = DBSCAN(eps=epsilon, min_samples=min_samples)
    result = db.fit_predict(PCs)

    return result

if __name__ == "__main__":
    data_left = np.loadtxt("left_ground_pitch_0620test.txt", dtype=float)
    data_right = np.loadtxt("right_ground_with_human2.txt", dtype=float)

    # Ground plane coefficient [a1, a2, c]
    # x = a1*y + a2*z +c 
    coeff_left = [-0.10336304, -0.60649112, 0.7770675459889381]
    coeff_right= [0.24249883, -0.35360388, 0.5720567587076608]

    print(data_left)

    output_left = remove_ground(coeff_left, data_left, 0.15, 0)
    output_right = remove_ground(coeff_right, data_right, 0.15, 0)

    scatter_plot(data_left, output_left)
    #scatter_plot(data_right, output_right)

