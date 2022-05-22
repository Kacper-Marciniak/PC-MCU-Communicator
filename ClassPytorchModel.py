import math
import os
import torch
from torch import nn
from torch.utils.data import DataLoader
from torch.utils.data import Dataset
from torchvision import datasets, transforms
from torch import optim

import numpy as np

import ClassNeuralNetwork as CNN

class CPytorchModel():
    def __init__(self):
        self._initDevice()
        self._initModel()
        #Learning parameters
        self.loss_function = nn.MSELoss()
        self.optimizer = optim.SGD(self.getParams(), lr=1e-6)
        self.list_loss_training = list()
        self.list_loss_validation = list()

    def setLossFunction(self, LossFunction):
        self.loss_function = LossFunction
    
    def setOptimizer(self, Optimizer):
        self.optimizer = Optimizer

    def getParams(self):
        return self.model.parameters()

    def trainModel(self, TrainDataLoader, TestDataLoader={}, Epochs=1000):
        print("STARTING TRAINING")
        print(f"Epochs: {Epochs}\nTraining dataset size: {len(TrainDataLoader.dataset)}")
        if TestDataLoader:
            print(f"Testing dataset size: {len(TestDataLoader.dataset)}")
        for t in range(Epochs):
            print(f"[{t+1}/{Epochs}]\n-------------------------------")
            loss_train = self._train(TrainDataLoader)
            self.list_loss_training.append(loss_train)
            print(f"\tTraining loss: {loss_train:.3E}")
            if TestDataLoader: 
                loss_test = self._test(TestDataLoader)
                self.list_loss_validation.append(loss_test)
                print(f"\tValidating loss: {loss_test:.3E}")
            else:
                self.list_loss_validation.append(loss_train)
        print("Done!")

    def saveModelState(self, Path):
        torch.save(self.model.state_dict(), Path)

    def loadModelState(self, Path):
        if not os.path.exists(Path): # check if path exists
            print("Path does not exist!!!")
        else: 
            self.model.load_state_dict(torch.load(Path, map_location=self.device))
            self.model.eval()

    def getDevice(self):
        return self.device

    def getListLoss(self):
        return self.list_loss_training, self.list_loss_validation

    def getLastLoss(self):
        if len(self.list_loss_training) > 0:
            return self.list_loss_training[-1], self.list_loss_validation[-1]
        else: return math.inf, math.inf

    def _train(self, DataLoader):
        train_loss = 0.0
        num_batches = len(DataLoader)
        for input, label in DataLoader:
            # Compute prediction and loss
            pred = self.predict(input)
            loss = self.loss_function(pred, label)
            
            # Backpropagation
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

            train_loss += loss.item()

        return train_loss / num_batches

    def _test(self, DataLoader):
        num_batches = len(DataLoader)
        test_loss = 0.0

        with torch.no_grad():
            for input, label in DataLoader:
                pred = self.predict(input)
                test_loss += self.loss_function(pred, label).item()

        return test_loss / num_batches

    def _initDevice(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using {self.device} device")

    def _initModel(self):
        self.model = CNN.CNeuralNetwork().to(self.device)
        print(self.model)

    def getModelInfo(self):
        buffer = str(
            "Device: "+str(self.getDevice())+
            "\n*************************"+
            "\nLoss function: "+str(self.loss_function)+
            "\n*************************"+
            "\nOptimizer: "+str(self.optimizer)
        )
        return  buffer

    def predict(self, data):
        return self.model(data) 

class CDataset(Dataset):
    def __init__(self, PathToDir, Device):
        super(Dataset, self).__init__()
        data_list = list()
        label_list = list()
        for filename in os.listdir(PathToDir): #read all files from working directory
            if r".csv" in filename or r".txt" in filename:
                data, label = self.ReadDataFromFile(PathToDir, filename)
                data_list.append(data)
                label_list.append(label)
        self.data = torch.Tensor(data_list).to(Device)
        self.data_labels = torch.Tensor(label_list).to(Device).unsqueeze(1)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, Idx):
        return self.data[Idx], self.data_labels[Idx]

    def ReadDataFromFile(self, PathToDir, Filename):
        File = open(PathToDir+Filename)
        if File.closed == True: return False
        else:
            Data = np.empty((5,50))
            Label = float(File.readline())
            Temperature = float(File.readline())
            for idx in range(50):
                line = File.readline().replace("\n", "")
                if not line:    break #EOF
                data_line = line.split(";")
                for i in range(5):
                    Data[i][idx] = float(data_line[i])
            return Data, Label

def createModel():    
    return CPytorchModel()

def createDataLoader(Data, BatchSize=1, Shuffle=False):
    return DataLoader(Data, batch_size=BatchSize, shuffle=Shuffle)

def getItemFromTensor(Tensor):
    return Tensor.detach().item()

def compareModels(Model1, Model2):
    # compare two models, check last value of LOSS function (training)
    [M1LossT, M1LossV] = Model1.getLastLoss()
    [M2LossT, M2LossV] = Model2.getLastLoss()
    return M1LossT>M2LossT, M1LossV>M2LossV
    