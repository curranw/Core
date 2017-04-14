#!/usr/bin/python

from sklearn.decomposition import PCA, KernelPCA
from sklearn.manifold import Isomap
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
import random
from sklearn import random_projection

class DimReductionClass:
    def __init__(self):
        self.data = list()
        self.all_dim_reduction = list()
        self.use_PCA = False
        self.use_KPCA = False
        self.use_RANDOM = False

    def add_data(self, data_vec):
        self.data.append(data_vec)
        return 1

    def build_dim_reduction(self, dim_i, dim_max):
        print(len(self.data))
        self.data_np = np.array(self.data)


        if dim_i == 0:
            print("Use PCA!")
            self.use_PCA = True
            for i in range(1,dim_max):
                print(i)
                pca = PCA(n_components=i, svd_solver='auto')
                pca.fit(self.data_np)
                self.all_dim_reduction.append(pca)
        if dim_i == 1:
            print("Use KPCA!")
            self.use_KPCA = True
            for i in range(1,dim_max):
                print(i)
                kpca = KernelPCA(n_components=i, kernel="poly", degree=4)
                kpca.fit(self.data_np[1:10000])
                self.all_dim_reduction.append(kpca)
        if dim_i == 2:
            print("Use Random!")
            self.use_RANDOM = True
            for i in range(1,dim_max):
                print(i)
                random_proj = random_projection.SparseRandomProjection(n_components=i, dense_output=True)
                random_proj.fit(self.data_np)
                self.all_dim_reduction.append(random_proj)

        return 1

    def transform_down(self, state, dim):
        state = np.array(state)
        state = state.reshape(1,-1)
        transformed = self.all_dim_reduction[dim-1].transform(state)[0]
        return transformed.tolist()
