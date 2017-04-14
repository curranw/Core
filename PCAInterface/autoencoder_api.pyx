from keras.layers import Input, Dense
from keras.models import Model
from keras.datasets import mnist
from numpy import genfromtxt
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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

        self.debug = True
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


        for i in range(0,dim_max):
            encoding_dim = 1
            input_img = Input(shape=(4, ))
            encoded = Dense(encoding_dim, activation='sigmoid')(input_img) 
            decoded = Dense(4, activation='linear')(encoded)
            autoencoder = Model(input=input_img, output=decoded)
            encoder = Model(input=input_img, output=encoded)

            encoded_input = Input(shape=(encoding_dim,))
            decoder_layer = autoencoder.layers[-1]
            decoder = Model(input=encoded_input, output=decoder_layer(encoded_input))
            autoencoder.compile(optimizer='RMSprop', loss='mean_absolute_error')

            reconstructed = self.x_train - self.cumulative_decoded

            autoencoder.fit(self.x_train, reconstructed,
                            nb_epoch=500,
                            verbose=0,
                            batch_size=2056,
                            shuffle=True,
                            validation_split=.2)

            encoded_imgs = encoder.predict(self.x_train) #the reduced dimensionality representation: 1 dim
            self.cumulative_decoded += decoder.predict(encoded_imgs) #the reconstructed representation: 4 dim

            self.compute_mse()
            self.encoders.append(encoder)

            if self.debug:
                self.all_encoded.append(encoded_imgs)
                if i == dim_max-1:
                    print(str(self.all_encoded[0][0]) + "," + str(self.all_encoded[1][0]) + "," + str(self.all_encoded[2][0]))
                    self.ax.scatter(self.all_encoded[0][0:1000, 0], [0] * 1000, [0] * 1000, c='blue', cmap=plt.cm.spectral)
                    self.ax.scatter(self.all_encoded[0][0:1000, 0], self.all_encoded[1][0:1000, 0], [0] * 1000, c='green', cmap=plt.cm.spectral)
                    self.ax.scatter(self.all_encoded[0][0:1000, 0], self.all_encoded[1][0:1000, 0], self.all_encoded[2][0:1000, 0], c='yellow', cmap=plt.cm.spectral)
                    self.ax.scatter(self.cumulative_decoded[0:1000, 0], self.cumulative_decoded[0:1000, 1], self.cumulative_decoded[0:1000, 2], c='pink', cmap=plt.cm.spectral)
                    self.ax.scatter(self.x_train[0:1000, 0], self.x_train[0:1000, 1], self.x_train[0:1000, 2], c='red', cmap=plt.cm.spectral)
                    plt.show()

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
        print(state)
        state = np.array(state)
        print(state)
        state = state.reshape((1, -1))
        print(state)
        transformed = list()
        for i in range(0, dim):
            transformed.append(self.encoders[i].predict(state))
        return transformed



#For testing
if __name__ == "__main__":
    nn_dim_reduce = NNDimReduce()

    filename = 'mountain_car_3d_good_1.csv'

    programData = list()

    with open(filename, 'rb') as f:
        reader = csv.reader(f)
        for row in reader:
            data_row = list()
            for ele in row:
                data_row.append(float(ele))
            nn_dim_reduce.add_data(data_row)
            programData.append(data_row)

    programData_np = np.array(programData)
    nn_dim_reduce.build_dim_reduction(3)

    print(nn_dim_reduce.transform_down(programData_np[0], 1))
    print(nn_dim_reduce.transform_down(programData[0], 2))
    print(nn_dim_reduce.transform_down(programData[0], 3))



# ########################################################
# #
# # Autoencoder construction
# #
# ########################################################

# encoding_dim = 1
# input_img = Input(shape=(4, ))
# encoded = Dense(encoding_dim, activation='sigmoid')(input_img) 
# decoded = Dense(4, activation='linear')(encoded)
# autoencoder = Model(input=input_img, output=decoded)
# encoder = Model(input=input_img, output=encoded)

# encoded_input = Input(shape=(encoding_dim,))
# decoder_layer = autoencoder.layers[-1]
# decoder = Model(input=encoded_input, output=decoder_layer(encoded_input))
# autoencoder.compile(optimizer='RMSprop', loss='mean_absolute_error')

# #######################################################
# #
# # Training
# #
# #######################################################

# x_train = programData_np #original dataset
# x_test = x_train #test data is the original
# emptyDataSet = x_train - x_train #placeholder for the comparator

# # print programData_np[0][0]

# # Epoch: A full pass over all of your training data # Loss: A scalar value that we attempt to minimize during our training of the model. The lower the loss, the closer our predictions are to the true labels.
# # Batch Size: defines the number of samples that are going to be propagated through the network. # note that the smaller the batch, the less accurate the estimate of the gradient
# autoencoder.fit(x_train, x_train,
#                 nb_epoch=500,
#                 verbose=0,
#                 batch_size=2056,
#                 shuffle=True,
#                 validation_split=.2)

# encoded_imgs = encoder.predict(x_test) #the reduced dimensionality representation: 1 dim
# decoded_imgs = decoder.predict(encoded_imgs) #the reconstructed representation: 4 dim

# errorCheck = True #this keeps the program iterating through another training till the error is equivalent to the original dataset
# error = emptyDataSet #initializes the error, fills it with an empty dataset with the same shape as the 
# error = decoded_imgs
# reconstructed = None

# fig = plt.figure(1, figsize=(4, 3))
# ax = Axes3D(fig, rect=[0, 0, .95, 1], elev=48, azim=134)

# encoded_imgs_1 = encoded_imgs

# all_error = 0
# tot = 0
# for i in range(0,len(x_train)):
#     ele = x_train[i]
#     ele2 = decoded_imgs[i]
#     tot = ele - ele2
#     tot = tot * tot
#     all_error += sum(tot)

# print(all_error)
# print(all_error/len(x_train))


# ax.scatter(encoded_imgs_1[0:1000, 0], [0] * 1000, [0] * 1000, c='navy', cmap=plt.cm.spectral)
# # ax.scatter(encoded_imgs_1[0:1000, 0], encoded_imgs_1[0:1000, 1], [0] * 1000, c='turquoise', cmap=plt.cm.spectral)
# # ax.scatter(encoded_imgs_1[0:1000, 0], encoded_imgs_1[0:1000, 1], encoded_imgs_1[0:1000, 2], c='yellow', cmap=plt.cm.spectral)
# # ax.scatter(encoded_imgs_1[0:1000, 0], [0] * 1000, [0] * 1000, c='navy', cmap=plt.cm.spectral)
# #ax.scatter(decoded_imgs[0:1000, 0], decoded_imgs[0:1000, 1], decoded_imgs[0:1000, 2], c='red', cmap=plt.cm.spectral)
# # ax.scatter(decoded_imgs[0:1000, 0], decoded_imgs[0:1000, 1], decoded_imgs[0:1000, 2], c='blue', cmap=plt.cm.spectral)
# # ax.scatter(x_train[0:1000, 0], x_train[0:1000, 1], x_train[0:1000, 2], c='red', cmap=plt.cm.spectral)
# # plt.show()
# # exit()
# print x_train[0]
# print error[0]

# #################################################################################################################################################


# ########################################################
# #
# # Autoencoder construction
# #
# ########################################################

# encoding_dim = 1
# input_img = Input(shape=(4, ))
# encoded = Dense(encoding_dim, activation='sigmoid')(input_img) 
# decoded = Dense(4, activation='linear')(encoded)
# autoencoder = Model(input=input_img, output=decoded)
# encoder = Model(input=input_img, output=encoded)

# encoded_input = Input(shape=(encoding_dim,))
# decoder_layer = autoencoder.layers[-1]
# decoder = Model(input=encoded_input, output=decoder_layer(encoded_input))
# autoencoder.compile(optimizer='RMSprop', loss='mean_absolute_error')

# reconstructed = x_train - decoded_imgs #sets the new training data as the reconstructed version of the last fit

# autoencoder.fit(x_train, reconstructed,
#             	nb_epoch=500,
#                 verbose=0,
#             	batch_size=2056,
#             	shuffle=True,
#                 validation_split=.2)


# #this is the "error" we want the error to reach 0

# encoded_imgs = encoder.predict(x_test) #the reduced dimensionality representation: 1 dim
# decoded_imgs = decoded_imgs + decoder.predict(encoded_imgs) #the reconstructed representation: 4 dim

# encoded_imgs_2 = decoder.predict(encoded_imgs)
# error = decoded_imgs #concatenates the next amount of error



# ax.scatter(encoded_imgs_1[0:1000, 0], encoded_imgs_2[0:1000, 0], [0] * 1000, c='turquoise', cmap=plt.cm.spectral)

# all_error = 0
# tot = 0
# for i in range(0,len(x_train)):
#     ele = x_train[i]
#     ele2 = decoded_imgs[i]
#     tot = ele - ele2
#     tot = tot * tot
#     all_error += sum(tot)

# print(all_error)
# print(all_error/len(x_train))




# encoding_dim = 1
# input_img = Input(shape=(4, ))
# encoded = Dense(encoding_dim, activation='sigmoid')(input_img) 
# decoded = Dense(4, activation='linear')(encoded)
# autoencoder = Model(input=input_img, output=decoded)
# encoder = Model(input=input_img, output=encoded)

# encoded_input = Input(shape=(encoding_dim,))
# decoder_layer = autoencoder.layers[-1]
# decoder = Model(input=encoded_input, output=decoder_layer(encoded_input))
# autoencoder.compile(optimizer='RMSprop', loss='mean_absolute_error')

# reconstructed = x_train - decoded_imgs #sets the new training data as the reconstructed version of the last fit

# autoencoder.fit(x_train, reconstructed,
#                 nb_epoch=500,
#                 verbose=0,
#                 batch_size=2056,
#                 shuffle=True,
#                 validation_split=.2)


# #this is the "error" we want the error to reach 0

# encoded_imgs = encoder.predict(x_test) #the reduced dimensionality representation: 1 dim
# decoded_imgs = decoded_imgs + decoder.predict(encoded_imgs) #the reconstructed representation: 4 dim

# encoded_imgs_3 = decoder.predict(encoded_imgs)
# error = decoded_imgs #concatenates the next amount of error


# ax.scatter(encoded_imgs_1[0:1000, 0], encoded_imgs_2[0:1000, 0], encoded_imgs_3[0:1000, 0], c='yellow', cmap=plt.cm.spectral)

# ax.scatter(decoded_imgs[0:1000, 0], decoded_imgs[0:1000, 1], decoded_imgs[0:1000, 2], c='pink', cmap=plt.cm.spectral)
# ax.scatter(x_train[0:1000, 0], x_train[0:1000, 1], x_train[0:1000, 2], c='red', cmap=plt.cm.spectral)

# all_error = 0
# tot = 0
# for i in range(0,len(x_train)):
#     ele = x_train[i]
#     ele2 = decoded_imgs[i]
#     tot = ele - ele2
#     tot = tot * tot
#     all_error += sum(tot)

# print(all_error)
# print(all_error/len(x_train))

# plt.show()
# exit()
# print x_train[0][0]
# print error[0][0]
# print error
# print x_train


# # if (x_train[0][0] - error[0][0]) < 0.001:
# # 	break

# #################################################################################################################################################

# print "You reached the end"
# print error
# print x_train
