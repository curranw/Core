from keras.layers import Input, Dense
from keras.models import Model, Sequential
from keras.datasets import mnist
from numpy import genfromtxt
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
from kerasify import export_model

########################################################
#
# Data Preparation
#
########################################################

class NNDimReduce(object):

    def __init__(self):
        self.data = list()
        self.cumulative_decoded = list()
        self.encoders = list()

        self.debug = False
        if self.debug:
            self.fig = plt.figure(1, figsize=(4, 3))
            self.ax = Axes3D(self.fig, rect=[0, 0, .95, 1], elev=48, azim=134)

        self.all_encoded = list()
    def add_data(self, data_vec):
        self.data.append(data_vec)
        return 1


    def build_dim_reduction(self, dim_max):
        print(len(self.data))
        self.x_train = np.array(self.data)
        self.cumulative_decoded = self.x_train - self.x_train

        #Test

        encoding_dim = dim_max-1
        input_img = Input(shape=(dim_max, ))
        encoded = Dense(encoding_dim, activation='sigmoid', input_shape=(dim_max,)) 
        decoded = Dense(dim_max, activation='linear')#, input_shape=(encoding_dim,))
        

        model_ae = Sequential()
        model_ae.add(encoded)
        model_ae.add(decoded)
        
        model_e = Sequential()
        model_e.add(encoded)

        model_d = Sequential()
        model_d.add(decoded)

        model_ae.compile(optimizer='RMSprop', loss='mean_absolute_error')


        reconstructed = self.x_train - self.cumulative_decoded

        model_ae.fit(self.x_train, reconstructed,
                        nb_epoch=1000,
                        verbose=0,
                        batch_size=2056,
                        shuffle=True,
                        validation_split=.2)

        encoded_imgs = model_e.predict(self.x_train) #the reduced dimensionality representation: 1 dim
        self.cumulative_decoded += model_ae.predict(self.x_train) #the reconstructed representation: 4 dim

        self.compute_mse()

        self.encoders.append(model_e)
        #Real
        # encoding_dim = 1
        # input_img = Input(shape=(dim_max, ))
        # encoded = Dense(encoding_dim, activation='sigmoid')(input_img) 
        # decoded = Dense(dim_max, activation='linear')(encoded)
        # autoencoder = Model(input=input_img, output=decoded)
        # encoder = Model(input=input_img, output=encoded)

        # encoded_input = Input(shape=(encoding_dim,))
        # decoder_layer = autoencoder.layers[-1]
        # decoder = Model(input=encoded_input, output=decoder_layer(encoded_input))
        # autoencoder.compile(optimizer='RMSprop', loss='mean_absolute_error')

        # reconstructed = self.x_train - self.cumulative_decoded

        # autoencoder.fit(self.x_train, reconstructed,
        #                 nb_epoch=100,
        #                 verbose=0,
        #                 batch_size=2056,
        #                 shuffle=True,
        #                 validation_split=.2)

        # encoded_imgs = encoder.predict(self.x_train) #the reduced dimensionality representation: 1 dim
        # self.cumulative_decoded += decoder.predict(encoded_imgs) #the reconstructed representation: 4 dim

        # self.compute_mse()
        # self.encoders.append(encoder)

        if self.debug:
            self.all_encoded.append(encoded_imgs)
            if i == dim_max-1:
                print(str(self.all_encoded[0][0]) + "," + str(self.all_encoded[1][0]) + "," + str(self.all_encoded[2][0]))
                self.ax.scatter(self.all_encoded[0][0:1000, 0], [0] * 1000, [0] * 1000, c='blue', cmap=plt.cm.spectral)
                self.ax.scatter(self.all_encoded[0][0:1000, 0], self.all_encoded[1][0:1000, 0], [0] * 1000, c='green', cmap=plt.cm.spectral)
                self.ax.scatter(self.all_encoded[0][0:1000, 0], self.all_encoded[1][0:1000, 0], self.all_encoded[2][0:1000, 0], c='yellow', cmap=plt.cm.spectral)
                # self.ax.scatter(self.cumulative_decoded[0:1000, 0], self.cumulative_decoded[0:1000, 1], self.cumulative_decoded[0:1000, 2], c='pink', cmap=plt.cm.spectral)
                # self.ax.scatter(self.x_train[0:1000, 0], self.x_train[0:1000, 1], self.x_train[0:1000, 2], c='red', cmap=plt.cm.spectral)
                plt.show()

        export_model(model_e, 'encoder.model.single')

    def compute_mse(self):
        all_error = 0
        tot = 0
        for i in range(0,len(self.x_train)):
            ele = self.x_train[i]
            ele2 = self.cumulative_decoded[i]
            tot = ele - ele2
            tot = tot * tot
            all_error += sum(tot)
        print(all_error/len(self.x_train))


    def transform_down(self, state, dim):
        state = np.array(state)
        state = state.reshape((1, -1))
        transformed = list()
        for i in range(0, dim):
            transformed.append(self.encoders[i].predict(state))
        #print(transformed)
        return transformed



#For testing
if __name__ == "__main__":
    nn_dim_reduce = NNDimReduce()

    filename = sys.argv[1:][0]
    programData = list()
    print(filename)
    for i in range(0, len(sys.argv[1:])):
        filename = sys.argv[1:][i]
        with open(filename, 'rb') as f:
            reader = csv.reader(f)
            for row in reader:
                data_row = list()
                for ele in row:
                    data_row.append(float(ele))
                nn_dim_reduce.add_data(data_row)
                programData.append(data_row)

    programData_np = np.array(programData)
    nn_dim_reduce.build_dim_reduction(len(programData[0]))

    print(nn_dim_reduce.transform_down(programData_np[0], 1))
    print(nn_dim_reduce.transform_down(programData[0], 2))
    print(nn_dim_reduce.transform_down(programData[0], 3))