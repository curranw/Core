from keras.layers import Input, Dense
from keras.models import Model
from keras.datasets import mnist
from numpy import genfromtxt
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from libcpp.vector cimport vector
########################################################
#
# Data Preparation
#
########################################################

cdef public class NNDimReduce[object NNDimReduce, type NNDimReduceType]:

    def __init__(self):
        #print("Initializing")
        self.data = list()
        self.cumulative_decoded = list()
        self.encoders = list()

        self.all_encoded = list()
    cdef public int add_data(self, vector[double] data_vec):
        self.data.append(data_vec)
        return 1


    cdef public void build_dim_reduction(self, int dim_max):
        print(len(self.data))
        self.x_train = np.array(self.data)
        self.cumulative_decoded = self.x_train - self.x_train


        for i in range(0,dim_max):
            encoding_dim = 1
            input_img = Input(shape=(dim_max, ))
            encoded = Dense(encoding_dim, activation='sigmoid')(input_img) 
            decoded = Dense(dim_max, activation='linear')(encoded)
            autoencoder = Model(input=input_img, output=decoded)
            encoder = Model(input=input_img, output=encoded)

            encoded_input = Input(shape=(encoding_dim,))
            decoder_layer = autoencoder.layers[-1]
            decoder = Model(input=encoded_input, output=decoder_layer(encoded_input))
            autoencoder.compile(optimizer='RMSprop', loss='mean_absolute_error')

            reconstructed = self.x_train - self.cumulative_decoded

            autoencoder.fit(self.x_train, reconstructed,
                            nb_epoch=100,
                            verbose=0,
                            batch_size=2056,
                            shuffle=True,
                            validation_split=.2)

            encoded_imgs = encoder.predict(self.x_train) #the reduced dimensionality representation: 1 dim
            self.cumulative_decoded += decoder.predict(encoded_imgs) #the reconstructed representation: 4 dim

            self.compute_mse()
            self.encoders.append(encoder)

#            if self.debug:
#                self.all_encoded.append(encoded_imgs)
#                if i == dim_max-1:
#                    print(str(self.all_encoded[0][0]) + "," + str(self.all_encoded[1][0]) + "," + str(self.all_encoded[2][0]))
#                    self.ax.scatter(self.all_encoded[0][0:1000, 0], [0] * 1000, [0] * 1000, c='blue', cmap=plt.cm.spectral)
#                    self.ax.scatter(self.all_encoded[0][0:1000, 0], self.all_encoded[1][0:1000, 0], [0] * 1000, c='green', cmap=plt.cm.spectral)
#                    self.ax.scatter(self.all_encoded[0][0:1000, 0], self.all_encoded[1][0:1000, 0], self.all_encoded[2][0:1000, 0], c='yellow', cmap=plt.cm.spectral)
#                    # self.ax.scatter(self.cumulative_decoded[0:1000, 0], self.cumulative_decoded[0:1000, 1], self.cumulative_decoded[0:1000, 2], c='pink', cmap=plt.cm.spectral)
#                    # self.ax.scatter(self.x_train[0:1000, 0], self.x_train[0:1000, 1], self.x_train[0:1000, 2], c='red', cmap=plt.cm.spectral)
#                    plt.show()

    cdef public void compute_mse(self):
        all_error = 0
        tot = 0
        for i in range(0,len(self.x_train)):
            ele = self.x_train[i]
            ele2 = self.cumulative_decoded[i]
            tot = ele - ele2
            tot = tot * tot
            all_error += sum(tot)
        print(all_error/len(self.x_train))


    cdef public vector[double] transform_down(self, vector[double] state, int dim):
        state = np.array(state)
        state = state.reshape((1, -1))
        transformed = list()
        for i in range(0, dim):
            transformed.append(self.encoders[i].predict(state))
        #print(transformed)
        return transformed





cdef public api NNDimReduce* buildNNDimReduce():
    cdef NNDimReduce dim_r = NNDimReduce()
    return dim_r

cdef public api int accessor_add_data(NNDimReduce input, vector[double] data):
    return input.add_data(data)


cdef public api int accessor_build_dim_reduction(NNDimReduce input, int dim_max):
    input.build_dim_reduction(dim_max)


cdef public api vector[double] accessor_transform_down(NNDimReduce input, vector[double] state, int dim):
    return input.transform_down(state, dim)

#For testing
# if __name__ == "__main__":
#     nn_dim_reduce = NNDimReduce()

#     filename = 'mountain_car_3d_good_1.csv'

#     programData = list()

#     with open(filename, 'rb') as f:
#         reader = csv.reader(f)
#         for row in reader:
#             data_row = list()
#             for ele in row:
#                 data_row.append(float(ele))
#             nn_dim_reduce.add_data(data_row)
#             programData.append(data_row)

#     programData_np = np.array(programData)
#     nn_dim_reduce.build_dim_reduction(3)

#     print(nn_dim_reduce.transform_down(programData_np[0], 1))
#     print(nn_dim_reduce.transform_down(programData[0], 2))
#     print(nn_dim_reduce.transform_down(programData[0], 3))
