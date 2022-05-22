import torch
from torch import nn
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from torch import optim
from torch.nn import functional as F
import math

class CNeuralNetwork(nn.Module):
    def __init__(self):
        super(CNeuralNetwork, self).__init__()
        self.input_size = 50
        self.output_size = 1
        self.kernel_size = 11
        self.padding_size = math.floor(self.kernel_size/2)

        self.stack_full_convolution = nn.Sequential(
            nn.Conv1d( in_channels=5, out_channels=5*128, kernel_size=self.kernel_size, stride=1, dilation=1, padding=self.padding_size ),
            nn.BatchNorm1d(5*128),
            nn.ReLU(),
            nn.Conv1d( in_channels=5*128, out_channels=5*256, kernel_size=self.kernel_size, stride=1, dilation=1, padding=self.padding_size ),
            nn.BatchNorm1d(5*256),
            nn.ReLU(),
            nn.Conv1d( in_channels=5*256, out_channels=5*128, kernel_size=self.kernel_size, stride=1, dilation=1, padding=self.padding_size ),
            nn.BatchNorm1d(5*128),
            nn.ReLU(),
        )

        self.avg_global_pool1d = nn.Sequential(
            nn.AvgPool1d(kernel_size=self.input_size, stride=1),
            nn.Flatten(1,-1),
        )

        self.stack_lin_out = nn.Sequential(      
            nn.Linear(5*128, self.output_size),
            nn.Sigmoid(),
        )
        

    def forward(self, x):
        x = self.stack_full_convolution(x)
        x = self.avg_global_pool1d(x)
        x = self.stack_lin_out(x)
        return x

def calculate_out_len_conv1D(length_in, kernel_size, stride=1, padding=0, dilation=1):
    return (length_in + 2 * padding - dilation * (kernel_size - 1) - 1) // stride + 1

def calculate_out_len_AvgPool1D(length_in, kernel_size, stride=1, padding=0):
    return (length_in + 2 * padding - kernel_size - 1) // stride + 1