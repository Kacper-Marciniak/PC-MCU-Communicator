# WERSJA 2022-04-29 9:00
from logging import raiseExceptions
import serial
import serial.serialutil 
import serial.tools.list_ports as list_ports
from datetime import datetime
import sys
import time
import os
from PIL import ImageTk, Image

from sklearn.decomposition import PCA
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.naive_bayes import GaussianNB

import numpy as np

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import tkinter as tk
import tkinter.messagebox  as msg
from tkinter import E, SINGLE, scrolledtext as tkscroll
from tkinter import filedialog as fd
from tkinter import ttk
from tkinter import _tkinter

import math

import random

import torch
from torch import nn
#from torch.utils.data import DataLoader
#from torchvision import datasets, transforms
from torch import optim
import ClassPytorchModel as NN

import shutil

import gc

def getFilePath():
    if getattr(sys, 'frozen', False):
        application_path = os.path.dirname(sys.executable)
    elif __file__:
        application_path = os.path.dirname(__file__)
    return application_path
#########################################################################################
# DEBUG
#########################################################################################
DEBUG_TRANSMISSION = False
#########################################################################################
# USE NN
#########################################################################################
CONST_BOOL_USE_NN_MODULE = False
#########################################################################################
# WINDOW TITLES
#########################################################################################
CONST_PROGRAM_TITLE = r"TruffleSniffer: PC-MCU Communicator"
CONST_NNPANEL_TITLE = r"Neural Network Control Panel"
#########################################################################################
# PATHS
#########################################################################################
CONST_PROGRAM_PATH = getFilePath()
CONST_RESOURCES_PATH =  os.path.join(CONST_PROGRAM_PATH,"TS-Resources")
CONST_ICON_PATH = os.path.join(CONST_RESOURCES_PATH,"TruffleSnifferTextShort.ico")
CONST_SPLASHSCREEN_PATH =  os.path.join(CONST_RESOURCES_PATH,"TruffleSnifferText.png")
CONST_FAN_SEQUENCE_INFO_PATH =  os.path.join(CONST_RESOURCES_PATH,"FanSequenceInfo.png")
global_path_to_output_dir = os.path.join(CONST_PROGRAM_PATH,"TS-Output\\")

global_path_to_dataset_dir = os.path.join(CONST_PROGRAM_PATH,"TS-NN\\Datasets\\")
global_path_to_NNmodels_dir = os.path.join(CONST_PROGRAM_PATH,"TS-NN\\Models\\")
if not os.path.exists(global_path_to_output_dir):   os.makedirs(global_path_to_output_dir)
if not os.path.exists(global_path_to_dataset_dir):  os.makedirs(global_path_to_dataset_dir)
if not os.path.exists(global_path_to_NNmodels_dir): os.makedirs(global_path_to_NNmodels_dir)
#########################################################################################
# SENSORS
#########################################################################################
CONST_LIST_SENSORS = ["Sensor 1","Sensor 2","Sensor 3","Sensor 4","Sensor 5"]
CONST_N_SENSORS = 5
#########################################################################################
# DEFAULT PARAMETERS
#########################################################################################
global_baudrate = 115200 
global_freq = 10 # Sampling frequency in Hz
global_timeout = .1 # Timeout in s
CONST_INT_FREQ_MIN = 1
CONST_INT_FREQ_MAX = 1000
CONST_FLOAT_ADC_MAX_VALUE = 1023.0
CONST_MAX_VAL = CONST_FLOAT_ADC_MAX_VALUE*10
CONST_MIN_VAL = 0.0
CONST_EXPECTED_OUTPUT_LEN = 7 # Expected length of output to MCU -> IMPORTANT!
CONST_INT_MINIMUM_FAN_SPEED = 0
CONST_TIME_WAIT_MAX = 10.0
CONST_INPUT_BUFFER_SERIAL_SIZE = 50
#########################################################################################
# GUI STYLE
#########################################################################################
CONST_TTK_STYLES_LIST = ['alt', 'clam', 'classic', 'default', 'vista', 'winnative', 'xpnative']
CONST_TTK_STYLE = 'vista'
CONST_DEFAULT_FONT = ("Segoe UI", 9)
#########################################################################################
# GUI SIZE
#########################################################################################
CONST_INT_TEXT_ELEMENT_WIDTH = 9
CONST_INT_TEXT_PARAM_ELEMENT_WIDTH = 50
CONST_INT_BUTTON_ELEMENT_WIDTH = 20
CONST_INT_SCROLLEDTEXT_LOG_HEIGHT = 15
CONST_INT_MAX_LOGGEDTEXT_HEIGHT = 100
#########################################################################################
# GUI TEXT
#########################################################################################
CONST_TEXT_PROGRAM_INFO = "Truffle Sniffer Communicator"
CONST_TEXT_PORT_NOT_OPEN = "Port is not open"
CONST_TEXT_FILE_NOT_OPEN = "File is not open"
CONST_TEXT_ATTEMPTING_TO_DISCONNECT = "Attempting to disconnect"
CONST_TEXT_RESET_DEVICE  = "Attempting to reset the device"
CONST_TEXT_TRANSMISSION_STOPPED = "Transmission stopped by user"
CONST_TEXT_ERROR = "!ERROR!"
CONST_TEXT_EXCEPTION_DETECTED = "An unhandled exception has occurred in program\nError info:"
CONST_TEXT_TRANSMISSION_ENDED = "Transmission ended"
CONST_TEXT_MCU_NOT_FOUND = "Could not find MCU"
CONST_TEXT_PORT_NOT_FOUND = "No ports detected"
CONST_TEXT_DETECTED_DEVICES = "Detected devices:"
CONST_TEXT_STARTED_LISTENING_TO = "Started listening to "
CONST_TEXT_STOPPED_LISTENING_TO = "Stopped listening to "
CONST_TEXT_PAIRED_WITH = "Paired with "
CONST_FAILED_TO_PAIR = "Failed to pair with selected device"
CONST_TEXT_FILE_CLOSED = "File closed"
CONST_TEXT_OPENED = "opened"
CONST_TEXT_CHOSEN = "chosen"
CONST_TEXT_FILENAME = "Filename"
CONST_TEXT_FILENAME_ASK_RENAME = "Save additional renamed copy?"
CONST_TEXT_BUTTON_SEARCHCONNECT = "Search&Connect"
CONST_TEXT_BUTTON_DISCONNECT = "Disconnect"
CONST_TEXT_BUTTON_FORCE_DISCONNECT = "Force disconnect"
CONST_TEXT_BUTTON_START = "START TEST"
CONST_TEXT_BUTTON_STOP = "STOP TEST"
CONST_TEXT_BUTTON_AIR_INTAKE = "GET AIR SAMPLE"
CONST_TEXT_BUTTON_AIR_CHAMBER = "CLEAR SENSOR CHAMBER"
CONST_TEXT_BUTTON_AIR_TEST_SEQUENCE = "FULL TEST SEQUENCE"
CONST_TEXT_LABEL_AIR_SPEED = "Fan speed [%]"
CONST_TEXT_LABEL_AIR_TIME = "Time [s]"
CONST_TEXT_LABEL_AIR_SAMPLE_LABEL = "Air sampling:"
CONST_TEXT_LABEL_AIR_CLEAR_LABEL = "Chamber clearing:"
CONST_TEXT_LABEL_AIR_SEQUENCE_TEST_LABEL = "Test:"
CONST_TEXT_LABEL_AIR_SEQUENCE_RECOVERY_LABEL = "Sensor recovery:"
CONST_TEXT_LABEL_AIR_INTAKE_LABEL_AS  = "Label as class:"
CONST_TEXT_LABEL_MARK_NEXT_SAMPLES_LEN = "Samples to mark:"
CONST_TEXT_LABEL_MARK_NEXT_SAMPLES_TIME_LEN = "Time to mark:"
CONST_TEXT_LABEL_AIR_SEQUENCE_MODE_LABEL = "SEQUENTIAL MODE - AIR SAMPLING CONTROL"
CONST_TEXT_LABEL_AIR_CONTINUES_MODE_LABEL = "CONTINUES MODE - LABEL CONTROL"
CONST_TEXT_BUTTON_MARK_NEXT_SAMPLES = "MARK NEXT SAMPLES"
CONST_TEXT_BUTTON_FAN_ON = "TURN FAN ON"
CONST_TEXT_BUTTON_FAN_OFF = "TURN FAN OFF"
CONST_TEXT_BUTTON_FAN_AIRING = "TOGGLE AIRING"
CONST_TEXT_BUTTON_FAN_AIRING_TOGGLE_OFF = "TOGGLE AIRING OFF"
CONST_TEXT_BUTTON_FAN_APPLY_SLIDER_SPEED = "SEND TO MCU"
CONST_TEXT_BUTTONSCONNECT_LABEL = "CONNECTION AND TEST CONTROL:"
CONST_TEXT_BUTTONSFAN_LABEL = "FAN CONTROL:"
CONST_TEXT_LABEL_FAN_SPEED_SLIDER  = "Fan speed control [%]"
CONST_TEXT_LABEL_FAN_SPEED_REPORTED = "Reported fan speed [%]"
CONST_TEXT_CHECK_SAVE_TO_FILE = "Save to file"
CONST_TEXT_CHECK_LOG_DATA = "Log all data"
CONST_TEXT_LOG_LABEL = "LOGGED DATA:"
CONST_TEXT_CHECK_CLEAR = "Clear logs"
CONST_TEXT_LABEL_BAUD = "Baudrate:"
CONST_TEXT_LABEL_FREQ = "Frequency:"
CONST_TEXT_LABEL_PATH = "Path to save:"
CONST_TEXT_BUTTON_PATH = "Get new path"
CONST_TEXT_BUTTON_APPLY_PARAMS = "Apply parameters"
CONST_TEXT_LABEL_SIZE_PLOT = "Width of data window:"
CONST_TEXT_LABEL_SIZE_BUFFER = "Width of pre-processing buffer:"
CONST_TEXT_LABEL_FREQPLOT = "Target sampling frequency (for data window):"
CONST_TEXT_LABEL_ANALYSIS = "Size of analysis window (in samples):"
CONST_TEXT_LABEL_TITLE = "Real-time data processing and plotting module:"
CONST_TEXT_NN_OUT = "Neural Network output:"
CONST_TEXT_LABEL_TIME = "Time:"
CONST_TEXT_LABEL_TEMPERATURE = "Temperature:"
CONST_TEXT_LABEL_CLASS  = "Class:"
CONST_TEXT_CHECK_AUTOSCALE = "Autoscale"
CONST_TEXT_LOADING = "LOADING..."
CONST_TEXT_LABEL_TITLE_NN = "Neural Network Control Panel"
CONST_TEXT_BUTTON_NN_LOAD = "Load model"
CONST_TEXT_BUTTON_NN_NEW = "New model"
CONST_TEXT_BUTTON_NN_ADD = "Add new array to datasets"
CONST_TEXT_BUTTON_NN_SAVE = "Save model as"
CONST_TEXT_BUTTON_NN_RETRAIN = "Retrain network"
CONST_TEXT_LABEL_NN_ENTRY_LR = "Learning rate:"
CONST_TEXT_LABEL_NN_ENTRY_EPOCHS = "Epochs:"
CONST_TEXT_LABEL_NN_SETTINGS_TITLE = "Settings"
CONST_TEXT_LINE_SKIPPED = "WARNING: Line skipped"
CONST_TEXT_CONNECTED = "Connected to:"
CONST_TEXT_EXCEPTION_FAILED_TO_PAIR = "Error. Device is unresponsive. Failed to pair with selected device. Restart both the application and the device."
CONST_TEXT_EXCEPTION_DEVICE_UNRESPONSIVE = "Error. Device is unresponsive. Failed to receive feedback answer. Restart both the application and the device."
CONST_TEXT_SAVE_COPY_AS = "Save copy as"
CONST_TEXT_SAVED_COPY_OF = "Saved copy of"
#########################################################################################
# CLASS DEFINITIONS
#########################################################################################

class CSerialPort():
    def __init__(self,root):
        self.m_resetSerialPort()
        self.param_baudrate = None
        self.param_timeout = None
        self.param_freq = None
        self.mainwindow = None
        self.m_setMasterWindow(root)

    def m_clearInputBuffer(self):
        self.pserial.reset_input_buffer()
    
    def m_clearOutputBuffer(self):
        self.pserial.reset_output_buffer()

    def m_resetSerialPort(self):
        self.port = None
        self.pserial = None
        self.devicename = "None"

    def m_setMasterWindow(self, root):
        self.mainwindow = root

    def m_updateParameters(self):
        self.param_baudrate = global_baudrate
        self.param_timeout = global_timeout
        self.param_freq = int(global_freq)

    def m_setPort(self, Port):
        self.port = Port
        if self.m_checkPortValid():
            self.devicename = str(Port.device)
        else : self.devicename = "None"

    def m_checkPortValid(self):
        if self.port == None: return False
        else: return True

    def m_checkPserialOpen(self):
        if self.pserial == None: return False
        return self.pserial.is_open

    def m_openPort(self):
        if not self.m_checkPserialOpen():
            try:
                self.pserial = serial.Serial(self.port.device, baudrate=self.param_baudrate, timeout=self.param_timeout)
                self.mainwindow.m_print(str(self.devicename)+" opened")
                return True
            except serial.SerialException:
                self.mainwindow.m_print(str(self.devicename)+" access denied")
                return False

    def m_closePort(self):
        if self.pserial == None: pass
        elif self.m_checkPserialOpen():
            self.pserial.close()       
            self.mainwindow.m_print(str(self.devicename)+" closed")

    def m_pairDevices(self):
        output = f"D1SP"
        expected_input = f"D2SP"
        self.m_transmitAndWaitFor(output, expected_input, bRemoveSpaces=True) # Wait for D2SP signal
        self.m_transmitString("D1CP")   # Transmit D1CP signal to confirm pairing completion
        return True

    def m_resetDevice(self):  
        self.m_transmitString("_RST") 

    def m_transmitString(self, string=str()):
        string = f"{string:8s}"
        size = self.pserial.write(string.encode())
        self.m_clearOutputBuffer()
        if DEBUG_TRANSMISSION: print(size, string.encode())

    def m_transmitAndWaitFor(self, output=str(), expected_input=str(), bRemoveSpaces=True):
        if bRemoveSpaces: expected_input = expected_input.replace(" ","")
        time_start = time.time()
        while True:
            self.m_clearInputBuffer()
            self.m_transmitString(output)
            input = self.m_editLine(self.m_listenPortLine())
            if bRemoveSpaces: input = input.replace(" ","")
            if expected_input in input:
                break    
            elif time.time() - time_start > CONST_TIME_WAIT_MAX:
                programRaiseException(CONST_TEXT_EXCEPTION_DEVICE_UNRESPONSIVE)
        self.m_clearOutputBuffer()
        return input

    def m_transmitFanSpeed(self, speed):
        speed = int(speed)
        if speed < 0: speed = 0
        elif speed > 255: speed = 255
        self.m_transmitString(f"FANS{speed:d}")

    def m_listenPortLine(self):
        #line = self.pserial.read(size=CONST_INPUT_BUFFER_SERIAL_SIZE)
        line = self.pserial.read_until(expected=b'\n')
        if DEBUG_TRANSMISSION: print(len(line), str(line))
        return self.m_editLine(str(line))
    
    def m_requestMCUdata(self):
        output = "DATA"
        expected_input = "DATA"
        line = self.m_transmitAndWaitFor(output, expected_input, bRemoveSpaces=True)
        return line.replace("DATA","")

    def m_editLine(self, line=str()):
        line = line.replace("b","")
        line = line.replace("\'","")
        line = line.replace("\\n","")
        line = line.replace("\\r","")
        line = line.replace("\\x00","")
        return line

    def m_setTimeout(self, timeout):
        self.pserial.timeout = timeout

class CDataFile():
    def __init__(self,root):
        self.working_dir = global_path_to_output_dir
        self.filename = None
        self.file = None
        self.mainwindow = None
        self.m_setMasterWindow(root)

    def m_clear(self):
        self.m_closeFile()
        self.filename = None
        self.file = None

    def m_setMasterWindow(self, root):
        self.mainwindow = root

    def m_checkFileValid(self):
        return not (self.file == None)

    def m_checkFileOpen(self):
        if self.file == None: return False
        return not (self.file.closed)

    def m_createNewFilename(self):
        curr_date = str(datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.filename = f"{curr_date}.csv"

    def m_getFilename(self):
        return self.filename

    def m_getFullPath(self):
        return str(os.path.realpath(self.file.name))

    def m_setFilename(self, filename):
        self.filename = f"{filename}.csv"

    def m_openNewFile(self, bWriteCaption=True, bCreateNewName=True):
        if bCreateNewName: self.m_createNewFilename()
        self.file = open(self.working_dir + self.filename, "w")
        if self.file.closed:   
            self.mainwindow.m_print(CONST_TEXT_ERROR+", "+CONST_TEXT_FILE_CLOSED)
            return None
        self.mainwindow.m_print(str(self.filename)+" "+CONST_TEXT_OPENED)
        if bWriteCaption: self.m_writeLineToFile(f"Time;{CONST_LIST_SENSORS[0]};{CONST_LIST_SENSORS[1]};{CONST_LIST_SENSORS[2]};{CONST_LIST_SENSORS[3]};{CONST_LIST_SENSORS[4]};Temp;Fan[%];Class")

    def m_writeLineToFile(self, line):
        self.file.write(line+"\n")
    
    def m_writeToFile(self, line):
        self.file.write(line)

    def m_closeFile(self):
        if self.file == None: pass
        elif not self.file.closed:
            self.file.close()
            self.mainwindow.m_print(CONST_TEXT_FILE_CLOSED)
        #self.filename = None
        #self.file = None

    def m_saveRenamedCopy(self):
        copy_file = fd.asksaveasfile(initialfile = f"{self.filename}",
        defaultextension=".csv",
        filetypes=[("All Files","*.*"),("CSV","*.csv")],
        title=CONST_TEXT_SAVE_COPY_AS
        )
        if copy_file == None: return self.m_getFullPath()
        dst_path = str(os.path.realpath(copy_file.name))
        if self.m_getFullPath() != dst_path:
            shutil.copy(self.m_getFullPath(), dst_path)
        return dst_path

def getTimeSec():
    return ((time.time_ns() + 500000) // 1000000)/1000.0 #MS/1000

def listAllPorts():
    list_port_names = list_ports.comports()
    if len(list_port_names) > 0: 
        ObjMainWindow.m_print(CONST_TEXT_DETECTED_DEVICES)
        for p in list_port_names:
            ObjMainWindow.m_print(f"-> {p}")
        ObjMainWindow.m_print("")
    if len(list_port_names) < 1:
        ObjMainWindow.m_print(CONST_TEXT_PORT_NOT_FOUND)
        return None
    ObjMainWindow.m_update()
    return list_port_names

def choosePort(list_port_names):
    while True:
        window_select = tk.Toplevel(master=ObjMainWindow.root)
        window_select.geometry("+0+0")
        window_select.title("Select a port")
        tk_window_label = ttk.Label(master=window_select,text="Select a port:",font=CONST_DEFAULT_FONT)
        tk_list_port_names = tk.Listbox(master=window_select, selectmode=SINGLE, height=len(list_port_names),width=50,font=CONST_DEFAULT_FONT)

        for i,p in enumerate(list_port_names):
            tk_list_port_names.insert(i, str(p))

        tk_window_label.pack()
        tk_list_port_names.pack()
        while window_select.winfo_exists():
            window_select.update()
            port_idx = ()
            port_idx = tk_list_port_names.curselection()
            if len(port_idx) != 0: 
                break
            else: window_select.update()
        window_select.destroy()
        if len(port_idx) != 0:
            port_idx = port_idx[0]
            break

    Port = list_port_names[port_idx]
    ObjMainWindow.m_print(str(Port.device)+" "+CONST_TEXT_CHOSEN)
    return Port

def editLine(line):
    line = line.replace("b","")
    line = line.replace("\'","")
    line = line.replace("\\n","")
    line = line.replace("\\r","")
    line = line.replace(" ","")
    line = line.replace("!","")
    return line

def getValues(line):
    global global_current_time
    global global_current_inputs
    global global_current_inputs_2
    global global_current_inputs_3
    global global_current_temperature
    global global_waiting_buffer_input_list
    global global_sample_counter
    global global_data_window_input_list
    global global_data_window_aprox_list
    global global_first_input
    global global_current_fan_speed

    global global_air_sequence_data_sensors, global_air_sequence_time, global_air_sequence_data_temperature, global_air_sequence_fan_speed
    global global_tmp_current_temperature, global_tmp_current_inputs, global_tmp_current_fan_speed

    # GET CURRENT DATA
    if not ";" in line:  
        return False
    data = line.split(";")
    if len(data) == CONST_EXPECTED_OUTPUT_LEN: 
        try:
            for i in range(5):
                global_tmp_current_inputs[i] = float(data[i])/10.0
            global_tmp_current_temperature = round(100*(float(data[5]) * 500.0 / CONST_FLOAT_ADC_MAX_VALUE)/10.0)/100
            global_tmp_current_fan_speed = float(data[6]) / 255.0 * 100.0
            if not (True in [(val > CONST_MAX_VAL or val < CONST_MIN_VAL) for val in global_tmp_current_inputs]): #skip line if weird values spotted
                global_current_inputs = global_tmp_current_inputs
                global_current_temperature = global_tmp_current_temperature
                global_current_fan_speed = global_tmp_current_fan_speed
        except ValueError:
            ObjMainWindow.m_print(CONST_TEXT_LINE_SKIPPED)  
    else: ObjMainWindow.m_print(CONST_TEXT_LINE_SKIPPED)  

    # BUFFER WITH N LAST SAMPLES (N=global_input_waiting_buffer_size)
    for i in range(5):
        global_waiting_buffer_input_list[i][:-1] =  global_waiting_buffer_input_list[i][1:]
        global_waiting_buffer_input_list[i][-1] = global_current_inputs[i]

    if global_first_input:
            global_first_input = False
            for i in range(5):
                global_waiting_buffer_input_list[i][:] = global_current_inputs[i]
                global_data_window_input_list[i][:] = global_current_inputs[i]
            global_data_window_fanspeed_list[:] = 0
            global_data_window_class_list[:] = CONST_LABEL_CLASS_NONE
    
    global_sample_counter +=1
    if global_sample_counter >= global_data_window_skip_samples:
        for i in range(5):
            global_data_window_input_list[i][:-1] = global_data_window_input_list[i][1:]
            global_data_window_input_list[i][-1] = np.round(np.sum(global_buffer_kernel*global_waiting_buffer_input_list[i])*10.0)/10.0 # MEAN - using Gauss shaped kernel
            # DATA PROCESSING
            global_current_inputs_2[i] = global_data_window_input_list[i][-1]
            global_current_inputs_3[i] = np.min(global_data_window_input_list[i])
            global_current_inputs_4[i] = np.max(global_data_window_input_list[i])
        # CLASS LABEL
        global_data_window_class_list[:-1] = global_data_window_class_list[1:]
        global_data_window_class_list[-1] = global_current_class_label
        # FAN SPEED
        global_data_window_fanspeed_list[:-1] = global_data_window_fanspeed_list[1:]
        global_data_window_fanspeed_list[-1]  = global_current_fan_speed

        global_sample_counter = 0
        ObjMainWindow.m_plot()
        
    # SAVE DATA DURING AIR TEST SEQUENCE
    if True in global_test_sequence_status:
        for i in range(5):
            global_air_sequence_data_sensors[i].append(global_current_inputs[i])
        global_air_sequence_time.append(global_current_time)
        global_air_sequence_data_temperature.append(global_current_temperature)
        global_air_sequence_fan_speed.append(global_current_fan_speed)

def createLineToSave():
    return f"{global_current_time:.6f};{global_current_inputs[0]};{global_current_inputs[1]};{global_current_inputs[2]};{global_current_inputs[3]};{global_current_inputs[4]};{global_current_temperature};{int(global_current_fan_speed)};{global_current_class_label}"

def exitProgram():
    ObjFile.m_closeFile()
    ObjPort.m_closePort()
    ObjMainWindow.m_destroyWindow()
    exit()

def programGetCurrentDate():
    return str(datetime.now().strftime("%Y%m%d_%H%M%S"))

def programConnect():    
    programDisconnect()
    # List all port names
    list_port_names = listAllPorts()
    if list_port_names == None:    
            TKcreateWindow(CONST_TEXT_ERROR, CONST_TEXT_PORT_NOT_FOUND)
            return False # Error
    # Choose and open port
    port = choosePort(list_port_names)
    if port == None: 
            ObjMainWindow.m_print(CONST_TEXT_MCU_NOT_FOUND)   
            TKcreateWindow(CONST_TEXT_ERROR, CONST_TEXT_MCU_NOT_FOUND)
            ObjMainWindow.m_update()
            return False # Error
    ObjPort.m_setPort(port)
    ObjPort.m_updateParameters()
    if ObjPort.m_openPort():
        ObjMainWindow.m_update()
        if ObjPort.m_pairDevices():
            ObjMainWindow.fparam_button_APPLY.configure(state="disabled")
            ObjMainWindow.ObjPlotter.fdata_button_FREQPLOT.configure(state="disabled")
            ObjMainWindow.fbuttons_button_START.configure(state="normal")
            ObjMainWindow.fbuttons_check_SAVETOFILE.configure(state="normal")
            ObjMainWindow.fbuttons_check_LOGDATA.configure(state="normal") 
            ObjMainWindow.m_update()
            ObjMainWindow.m_setFanTestSpeed()
        else:
            ObjMainWindow.m_print(CONST_FAILED_TO_PAIR)
            ObjMainWindow.m_update()
            programRaiseException(CONST_TEXT_EXCEPTION_FAILED_TO_PAIR)
            return False
    else: ObjPort.m_resetSerialPort()
    ObjMainWindow.m_updateDeviceName(ObjPort.devicename)
    ObjMainWindow.m_print(CONST_TEXT_PAIRED_WITH+str(ObjPort.devicename))
    ObjMainWindow.m_update()
    del list_port_names

def programDisconnect():
    try:
        if ObjPort.m_checkPserialOpen():
            ObjMainWindow.m_print(CONST_TEXT_ATTEMPTING_TO_DISCONNECT)
            ObjMainWindow.m_update()
            ObjPort.m_transmitAndWaitFor("D1DC", "D2DC")
            ObjPort.m_resetDevice()
            ObjPort.m_closePort()
        ObjPort.m_resetSerialPort()
        ObjMainWindow.m_updateDeviceName(ObjPort.devicename)
        ObjMainWindow.fparam_button_APPLY.configure(state="normal")
        ObjMainWindow.ObjPlotter.fdata_button_FREQPLOT.configure(state="normal")
        ObjMainWindow.fbuttons_button_START.configure(state="disabled")
        ObjMainWindow.fbuttons_button_STOP.configure(state="disabled")
        ObjMainWindow.fbuttons_check_SAVETOFILE.configure(state="disabled")
        ObjMainWindow.fbuttons_check_LOGDATA.configure(state="disabled") 
        ObjMainWindow.m_update()
    except _tkinter.TclError: #ignore "window destroyed" error
        pass
    
def programCommunicationCycle():
    global global_transmission_state
    global global_air_sampling_status
    global global_air_clearing_status
    global global_current_class_label
    global global_test_date_start
    global global_time_test_start
    global global_current_time
    ObjPort.m_setTimeout(global_timeout)
    global_test_date_start = programGetCurrentDate()
    # TK ####################################################################   
    ObjMainWindow.fbuttons_button_DISCONNECT.configure(state="disabled")
    ObjMainWindow.fbuttons_button_SEARCHCONNECT.configure(state="disabled")
    ObjMainWindow.fair_button_INTAKE_AIR_SAMPLE.configure(state="normal")
    ObjMainWindow.fair_button_INTAKE_CLEAR_CHAMBER.configure(state="normal")
    ObjMainWindow.fair_button_INTAKE_TEST_SEQUENCE.configure(state="normal")
    ObjMainWindow.fbuttons_button_START.configure(state="disabled")
    ObjMainWindow.fbuttons_button_STOP.configure(state="normal")
    ObjMainWindow.fbuttons_check_SAVETOFILE.configure(state="disabled")
    ObjMainWindow.fcont_button_MARK_SAMPLE_AS.configure(state="normal")
    ObjMainWindow.fcont_button_MARK_SAMPLE_AS_TIME.configure(state="normal")
    ########################################################################
    time_last_reading = 0.0
    global_time_test_start = getTimeSec()
    sys.stdout.flush() # Flush the stream
    # Create new file to save
    if ObjPort.m_checkPserialOpen() == True:
        if global_save_to_file:
            ObjFile.m_setFilename(global_test_date_start)
            ObjFile.m_openNewFile()
            if ObjFile.m_checkFileOpen() == False:
                TKcreateWindow(CONST_TEXT_ERROR, CONST_TEXT_FILE_NOT_OPEN)    
                ObjMainWindow.m_print(CONST_TEXT_FILE_NOT_OPEN)
                return False # Error
    else:       
        TKcreateWindow(CONST_TEXT_ERROR, CONST_TEXT_PORT_NOT_OPEN)
        ObjMainWindow.m_print(CONST_TEXT_PORT_NOT_OPEN)
        return False # Error
        # If port is open - read from serial and save to file
    global_transmission_state = True
    while ObjPort.m_checkPserialOpen():
        ObjPort.m_clearInputBuffer()
        if (getTimeSec() - time_last_reading - global_time_test_start) >= (1.0 / global_freq): #1.0/Freq passed
            global_current_time = getTimeSec() - global_time_test_start
            line = ObjPort.m_requestMCUdata()
            if True in global_test_sequence_status: # full test sequence
                programFanTransmissionSequence()
            elif global_air_sampling_status: # sampling
                programFanTransmissionAirSampling()
            elif global_air_clearing_status: # clearing
                programFanTransmissionAirClearing()
            elif global_class_name_override_status: # cont test mark as
                programFanTransmissionMarkAs()
            else:
                global_current_class_label = CONST_LABEL_CLASS_NONE # change back to NONE after sampling or clearing
                programFanTransmissionUserSpeed()
            line = editLine(line)
            getValues(line)
            ObjMainWindow.m_updateInfoFrame()
            time_last_reading = global_current_time
            if global_log_data:
                ObjMainWindow.m_print(line)
            if global_save_to_file: 
                ObjFile.m_writeLineToFile(createLineToSave())
            if global_transmission_state == False: # STOP button                    
                ObjMainWindow.m_print(CONST_TEXT_TRANSMISSION_STOPPED)
                break
        ObjMainWindow.m_update() # UPDATE WINDOW
        gc.collect()
            
    if global_save_to_file: 
        ObjFile.m_closeFile()
        answer = TKcreateWindowYesNo(CONST_TEXT_TRANSMISSION_ENDED, f"{CONST_TEXT_TRANSMISSION_ENDED}.\n{CONST_TEXT_FILENAME}: {ObjFile.filename}\n{CONST_TEXT_FILENAME_ASK_RENAME}", "info")
        if answer:
            dst = ObjFile.m_saveRenamedCopy()
            ObjMainWindow.m_print(f"{CONST_TEXT_SAVED_COPY_OF}:\n{ObjFile.m_getFullPath()}\nas:\n{dst}")
        else:
            pass
        ObjFile.m_clear()
    else: 
        TKcreateWindow(CONST_TEXT_TRANSMISSION_ENDED, f"{CONST_TEXT_TRANSMISSION_ENDED}.", "info")
    # TK ####################################################################  
    ObjMainWindow.fbuttons_button_DISCONNECT.configure(state="normal")
    ObjMainWindow.fbuttons_button_SEARCHCONNECT.configure(state="normal")
    ObjMainWindow.fair_button_INTAKE_AIR_SAMPLE.configure(state="disabled")
    ObjMainWindow.fair_button_INTAKE_CLEAR_CHAMBER.configure(state="disabled")
    ObjMainWindow.fair_button_INTAKE_TEST_SEQUENCE.configure(state="disabled")
    ObjMainWindow.fbuttons_button_START.configure(state="normal")
    ObjMainWindow.fbuttons_button_STOP.configure(state="disabled")
    ObjMainWindow.fbuttons_check_SAVETOFILE.configure(state="normal")
    ObjMainWindow.fcont_button_MARK_SAMPLE_AS.configure(state="disabled")
    ObjMainWindow.fcont_button_MARK_SAMPLE_AS_TIME.configure(state="disabled")
    ########################################################################

def programFanTransmissionSequence():
    global global_test_sequence_status
    global global_current_class_label
    curr_time = time.time() - global_test_sequence_time_start
    if global_test_sequence_status[0]: # sampling
        global_current_class_label = global_test_sequence_class_labels[0]
        if  curr_time < global_test_sequence_times[0]:
            ObjPort.m_transmitFanSpeed(global_air_sampling_speed)
        else:
            global_test_sequence_status[0] = False
    elif global_test_sequence_status[1]: # test
        global_current_class_label = global_test_sequence_class_labels[1]
        if curr_time < sum(global_test_sequence_times[0:2]):
            ObjPort.m_transmitFanSpeed(global_air_test_speed)
        else:
            global_test_sequence_status[1] = False
    elif global_test_sequence_status[2]: # clearing
        global_current_class_label = global_test_sequence_class_labels[2]
        if curr_time < sum(global_test_sequence_times[0:3]):
            ObjPort.m_transmitFanSpeed(global_air_clearing_speed)
        else:
            global_test_sequence_status[2] = False
    else: # recovery
        global_current_class_label = global_test_sequence_class_labels[3]
        if curr_time < sum(global_test_sequence_times):
            speed_tmp = (global_fan_speed_user-global_air_recovery_speed)/(global_test_sequence_times[3])
            speed_tmp = speed_tmp*(curr_time - sum(global_test_sequence_times[:3]))+global_air_recovery_speed
            ObjPort.m_transmitFanSpeed(speed_tmp)
        else:
            global_test_sequence_status[3] = False
            ObjMainWindow.m_endAirTestSequence()

def programFanTransmissionMarkAs():
    global global_class_name_override_status
    global global_class_name_override_samples

    if global_class_name_override_samples > 0:
        global_class_name_override_samples -= 1
        ObjPort.m_transmitFanSpeed(global_fan_speed_user)
    else:
        global_class_name_override_status = False

def programFanTransmissionAirSampling():
    global global_air_sampling_status
    if time.time()-global_air_sampling_time_start < global_air_sampling_time:
        ObjPort.m_transmitFanSpeed(global_air_sampling_speed)
    else: 
        global_air_sampling_status = False

def programFanTransmissionAirClearing():
    global global_air_clearing_status
    if time.time()-global_air_clearing_time_start < global_air_clearing_time:
        ObjPort.m_transmitFanSpeed(global_air_clearing_speed)
    else: 
        global_air_clearing_status = False

def programFanTransmissionUserSpeed():
    ObjPort.m_transmitFanSpeed(global_fan_speed_user)

def getWaitingBuffer():
    global global_waiting_buffer_input_list
    global global_waiting_buffer_size
    bufferSize = int(round(global_freq*global_data_window_resampling_delay*2))
    if bufferSize%2==0:
        bufferSize += 1
    global_waiting_buffer_input_list=np.zeros((5,bufferSize)).astype(int)
    global_waiting_buffer_size = bufferSize
    return global_waiting_buffer_input_list, global_waiting_buffer_size

def calcBufferGaussKernel():
    global global_buffer_kernel
    x_mod = 3
    x = np.linspace(-x_mod,+x_mod,global_waiting_buffer_size)
    kernel = 1/(3 * math.sqrt(2*math.pi))*np.exp(-(x)**2/(2*3**2))
    global_buffer_kernel = kernel/np.sum(kernel)
    return global_buffer_kernel

def getSamplesToSkip():
    return int(round(global_freq*global_data_window_resampling_delay))

def clearAllCachedData():
    global global_data_window_input_list
    global global_data_window_aprox_list
    global global_data_window_fanspeed_list
    global global_first_input
    getWaitingBuffer()
    global_data_window_fanspeed_list = np.zeros((global_data_window_size)).astype(float) 
    global_data_window_input_list = np.zeros((5,global_data_window_size)).astype(float)
    global_data_window_aprox_list = np.zeros((5,global_data_window_size)).astype(float) 
    global_first_input = True

def programRaiseException(exception):
    TKcreateWindow(CONST_TEXT_ERROR, f"{CONST_TEXT_EXCEPTION_DETECTED}\n\n{exception}") 
    raise Exception(exception)

def programExceptionHandler(exception):
    #TKcreateWindow(CONST_TEXT_ERROR, f"{CONST_TEXT_EXCEPTION_DETECTED}\n\n{exception}") 
    if ObjFile != None:
        if ObjFile.m_checkFileOpen():
            ObjFile.m_closeFile()
    if ObjMainWindow != None:
        ObjMainWindow.root.destroy()
    exit()

def programSetBaseline(baseline_vector):
    global global_baseline_data_sensors
    global_baseline_data_sensors = baseline_vector

class CTkMainWindow():
    def __init__(self):
        self.root = tk.Tk() # TK main window
        if os.path.exists(CONST_ICON_PATH): self.root.iconbitmap(CONST_ICON_PATH)
        self.root.resizable(width=False, height=False)
        self.m_placeTempImg()
        self.root.geometry(f"+{0}+{0}")

        #Setstyle
        self.style = ttk.Style(self.root)
        self.style.theme_use(CONST_TTK_STYLE)

        self.window_state = True

        self.m_initTK()

        #Initialize additional plotting window
        self.ObjPlotter = CTkPlotter(self)
        self.ObjPlotter.m_initPlot()
        self.ObjPlotter.m_plot()
        
        #Initialize additional NN window
        if CONST_BOOL_USE_NN_MODULE:
            self.ObjNN = CTkNNPanel(self)

        self.root.title(CONST_PROGRAM_TITLE) # set window title
        
    def m_initTK(self):
        self.var_save_to_file_state = tk.BooleanVar(value=global_save_to_file) # Tkinter var
        self.var_log_data_state = tk.BooleanVar(value=global_log_data)     
        ################
        # MASTER FRAME #
        ################
        self.masterFrame = tk.Frame(master=self.root, highlightbackground="black", highlightthickness=1, width=400)
        self.masterFrame.grid(row=0, column=0, sticky="NESW")
        #self.masterFrame.grid_propagate(0)
        
        ###################
        # MAIN SUB-FRAMES #
        ###################
        self.frameProgramInfo = tk.Frame(master=self.masterFrame, relief='flat', borderwidth=5) # GUI frame elements
        self.frameParameters = tk.Frame(master=self.masterFrame, relief='flat', borderwidth=5)
        self.frameButtons = tk.Frame(master=self.masterFrame, relief='flat', borderwidth=5)
        self.frameButtonsFanControl = tk.Frame(master=self.masterFrame, relief='flat', borderwidth=5)
        self.frameTestAirIntake = tk.Frame(master=self.masterFrame, relief='flat', borderwidth=5)
        self.frameContTest = tk.Frame(master=self.masterFrame, relief='flat', borderwidth=5)
        self.frameBaseline = tk.Frame(master=self.masterFrame, relief='flat', borderwidth=5)
        self.frameData = tk.Frame(master=self.masterFrame, relief='flat', borderwidth=5) 
        self.frameLogs = tk.Frame(master=self.masterFrame, relief='flat', borderwidth=5) 

        # MAIN SUB-FRAMES GRID
        self.frameProgramInfo.grid(row=0, column=0, columnspan=2, sticky="NESW")
        self.frameParameters.grid(row=1, column=0,  columnspan=2, sticky="NESW")
        self.frameButtons.grid(row=2, column=0, sticky="NESW")
        self.frameContTest.grid(row=2, column=1, sticky="NESW")
        self.frameButtonsFanControl.grid(row=3, column=1, sticky="NESW")
        self.frameTestAirIntake.grid(row=3, column=0, sticky="NESW")
        self.frameData.grid(row=4, column=0, columnspan=2, sticky="NESW")
        self.frameLogs.grid(row=5, column=0, columnspan=2, sticky="NESW")
        
        self.masterFrame.grid_columnconfigure(0, weight=1)
        self.masterFrame.grid_columnconfigure(1, weight=1)

        #self.masterFrame.grid_rowconfigure(0, weight=1)
        self.masterFrame.grid_rowconfigure(1, weight=1)
        self.masterFrame.grid_rowconfigure(2, weight=1)
        self.masterFrame.grid_rowconfigure(3, weight=1)
        self.masterFrame.grid_rowconfigure(4, weight=1)
        self.masterFrame.grid_rowconfigure(5, weight=1)

        self.m_initProgramInfoFrame()
        self.m_initParametersFrame()
        self.m_initButtonsConnectionFrame()
        self.m_initButtonsFanFrame()
        self.m_initButtonsAirIntakeFrame()
        self.m_initContModeFrame()
        self.m_initDataFrame()
        self.m_initLogsFrame()

    def m_initProgramInfoFrame(self):
        ################
        # PROGRAM INFO #
        ################
        self.fpinfo_programLogoText = ImageTk.PhotoImage(Image.open(CONST_ICON_PATH).resize((50, 50)))
        self.fpinfo_label_icon = ttk.Label(master=self.frameProgramInfo, image=self.fpinfo_programLogoText, background="white")
        self.fpinfo_label_info = ttk.Label(master=self.frameProgramInfo, text=CONST_TEXT_PROGRAM_INFO, background="white")
        
        self.fpinfo_label_info.config(font=("Bahnschrift Bold Condensed", 36), anchor="center")
        self.frameProgramInfo.configure(bg="white")

        # GRID
        self.fpinfo_label_icon.grid(row=0,column=0,sticky="NSW")
        self.fpinfo_label_info.grid(row=0,column=1,sticky="NESW")
        self.frameProgramInfo.grid_columnconfigure(1, weight=1)

    def m_initParametersFrame(self):
        ####################
        # PARAMETERS FRAME #
        ####################
        self.fparam_entry_BAUD = ttk.Entry(master=self.frameParameters) # GUI minor elements (INPUT)
        self.fparam_entry_FREQ = ttk.Entry(master=self.frameParameters)
        self.fparam_entry_PATH = ttk.Entry(master=self.frameParameters)
        
        self.fparam_entry_BAUD.insert(10, global_baudrate) # default values for "entry" elements
        self.fparam_entry_FREQ.insert(10, global_freq)
        self.fparam_entry_PATH.insert(10, global_path_to_output_dir)
        
        self.fparam_label_BAUD = ttk.Label(master=self.frameParameters, text=CONST_TEXT_LABEL_BAUD)
        self.fparam_label_FREQ = ttk.Label(master=self.frameParameters, text=CONST_TEXT_LABEL_FREQ)
        self.fparam_label_PATH = ttk.Label(master=self.frameParameters, text=CONST_TEXT_LABEL_PATH)
        self.fparam_button_PATH = ttk.Button(master=self.frameParameters, text=CONST_TEXT_BUTTON_PATH, command=self.m_getDirectoryToSave)
        self.fparam_button_APPLY = ttk.Button(master=self.frameParameters, text=CONST_TEXT_BUTTON_APPLY_PARAMS, command=self.m_getDataFromEntry)        
        
        # GRID
        self.fparam_label_PATH.grid(row=0, padx=5, sticky="E") # using grid to place elemets (INPUT)
        self.fparam_label_BAUD.grid(row=1, padx=5, sticky="E")
        self.fparam_label_FREQ.grid(row=2, padx=5, sticky="E")
        self.fparam_entry_PATH.grid(row=0, column=1, padx=5, sticky="WE")
        self.fparam_entry_BAUD.grid(row=1, column=1, padx=5, sticky="WE")
        self.fparam_entry_FREQ.grid(row=2, column=1, padx=5, sticky="WE")
        self.fparam_button_PATH.grid(row=0, column=2, rowspan=1, padx=5, sticky="NSWE")
        self.fparam_button_APPLY.grid(row=1, column=2, rowspan=2, padx=5, sticky="NSWE")

        self.frameParameters.grid_rowconfigure(0, weight=1)
        self.frameParameters.grid_rowconfigure(1, weight=1)
        self.frameParameters.grid_rowconfigure(2, weight=1)

        self.frameParameters.grid_columnconfigure(1, weight=1)

    def m_initButtonsConnectionFrame(self):
        #######################################
        # BUTTONS CONNECTION AND TEST CONTROL #
        #######################################
        # SUBFRAMES
        self.fbuttons_label_CONNECTTITLE = ttk.Label(master=self.frameButtons, text=CONST_TEXT_BUTTONSCONNECT_LABEL, anchor="center")
        self.fbuttons_frame_CONNECTIONCONTROL= ttk.Frame(master=self.frameButtons, relief='solid', borderwidth=1, padding=5)
        self.fbuttons_frame_TESTCONTROL = ttk.Frame(master=self.frameButtons, relief='solid', borderwidth=1, padding=5)

        # GRID SUBFRAMES
        self.fbuttons_label_CONNECTTITLE.grid(row=0, columnspan=3, sticky="NEW") 
        self.fbuttons_frame_CONNECTIONCONTROL.grid(row=1, column=0, sticky="NSEW") 
        self.fbuttons_frame_TESTCONTROL.grid(row=1, column=2,  sticky="NSEW") 
        self.frameButtons.grid_columnconfigure(0, weight=1) # Fill entire space
        self.frameButtons.grid_columnconfigure(1, minsize=10)
        self.frameButtons.grid_columnconfigure(2, weight=1)
        #self.frameButtons.grid_rowconfigure(0, weight=1)
        self.frameButtons.grid_rowconfigure(1, weight=1)
        
        # Subframe 1
        self.fbuttons_button_SEARCHCONNECT = ttk.Button(master=self.fbuttons_frame_CONNECTIONCONTROL, text=CONST_TEXT_BUTTON_SEARCHCONNECT, command=programConnect)
        self.fbuttons_button_DISCONNECT = ttk.Button(master=self.fbuttons_frame_CONNECTIONCONTROL, text=CONST_TEXT_BUTTON_DISCONNECT, command=programDisconnect)
        self.fbuttons_button_FORCEDISCONNECT = ttk.Button(master=self.fbuttons_frame_CONNECTIONCONTROL, text=CONST_TEXT_BUTTON_FORCE_DISCONNECT, command=programDisconnect)
        self.fbuttons_text_DEVICEINFO = ttk.Label(master=self.fbuttons_frame_CONNECTIONCONTROL)
        self.fbuttons_label_CONNECTED_TO = ttk.Label(master=self.fbuttons_frame_CONNECTIONCONTROL, text=CONST_TEXT_CONNECTED, anchor="center")
        
        self.fbuttons_text_DEVICEINFO.configure(text="None", background="white", anchor="center", borderwidth=1, relief="solid")

        self.fbuttons_button_SEARCHCONNECT.grid(row=0, padx=10, sticky="EW")
        self.fbuttons_button_DISCONNECT.grid(row=1, padx=10, sticky="EW")
        self.fbuttons_button_FORCEDISCONNECT.grid(row=2, padx=10, sticky="EW")
        self.fbuttons_label_CONNECTED_TO.grid(row=3, padx=10, sticky="SEW")  
        self.fbuttons_text_DEVICEINFO.grid(row=4, padx=10, sticky="NEW") 
        self.fbuttons_frame_CONNECTIONCONTROL.grid_columnconfigure(0, weight=1) # Fill entire space
        self.fbuttons_frame_CONNECTIONCONTROL.grid_rowconfigure(0, weight=1) # Fill entire space 
        self.fbuttons_frame_CONNECTIONCONTROL.grid_rowconfigure(1, weight=1) # Fill entire space 
        self.fbuttons_frame_CONNECTIONCONTROL.grid_rowconfigure(2, weight=1) # Fill entire space 
        self.fbuttons_frame_CONNECTIONCONTROL.grid_rowconfigure(3, weight=1) # Fill entire space 
        self.fbuttons_frame_CONNECTIONCONTROL.grid_rowconfigure(4, weight=1) # Fill entire space 
        
        # Subframe 2
        self.fbuttons_button_START = ttk.Button(master=self.fbuttons_frame_TESTCONTROL, text=CONST_TEXT_BUTTON_START, command=self.m_startCommunication)        
        self.fbuttons_button_STOP = ttk.Button(master=self.fbuttons_frame_TESTCONTROL, text=CONST_TEXT_BUTTON_STOP, command=self.m_stopCommunication)
        self.fbuttons_check_SAVETOFILE = ttk.Checkbutton(master=self.fbuttons_frame_TESTCONTROL, text=CONST_TEXT_CHECK_SAVE_TO_FILE,variable=self.var_save_to_file_state, onvalue=True, offvalue=False, command=self.m_toggleSaveFile)
        self.fbuttons_check_LOGDATA = ttk.Checkbutton(master=self.fbuttons_frame_TESTCONTROL, text=CONST_TEXT_CHECK_LOG_DATA,variable=self.var_log_data_state, onvalue=True, offvalue=False, command=self.m_toggleLogData)

        self.fbuttons_button_START.configure(state="disabled")
        self.fbuttons_button_STOP.configure(state="disabled")
        self.fbuttons_check_SAVETOFILE.configure(state="disabled")
        self.fbuttons_check_LOGDATA.configure(state="disabled") 
        
        self.fbuttons_button_START.grid(row=0, padx=10, sticky="EW")
        self.fbuttons_button_STOP.grid(row=1, padx=10, sticky="EW")   
        self.fbuttons_check_SAVETOFILE.grid(row=2, padx=10, sticky="W")
        self.fbuttons_check_LOGDATA.grid(row=3, padx=10, sticky="W")     
        self.fbuttons_frame_TESTCONTROL.grid_columnconfigure(0, weight=1) # Fill entire space
        self.fbuttons_frame_TESTCONTROL.grid_rowconfigure(0, weight=1)
        self.fbuttons_frame_TESTCONTROL.grid_rowconfigure(1, weight=1)
        self.fbuttons_frame_TESTCONTROL.grid_rowconfigure(2, weight=1)
        self.fbuttons_frame_TESTCONTROL.grid_rowconfigure(3, weight=1)
    
    def m_initButtonsFanFrame(self):            
        #######################
        # BUTTONS FAN CONTROL #
        #######################
        # SUBFRAMES
        self.fbuttons_label_FANTITLE = ttk.Label(master=self.frameButtonsFanControl, text=CONST_TEXT_BUTTONSFAN_LABEL, anchor="center")
        self.fbuttons_frame_mainframe = ttk.Frame(master=self.frameButtonsFanControl, relief='solid', borderwidth=1, padding=5)
        self.fbuttons_frame_FAN_ONOFF = ttk.Frame(master=self.fbuttons_frame_mainframe)
        self.fbuttons_frame_FAN_SPEED = ttk.Frame(master=self.fbuttons_frame_mainframe)
        
        # GRID SUBFRAMES
        self.fbuttons_label_FANTITLE.grid(row=0, sticky="NEW")
        self.fbuttons_frame_mainframe.grid(row=1, sticky="NESW")
        self.frameButtonsFanControl.grid_columnconfigure(0, weight=1) # Fill entire space
        self.frameButtonsFanControl.grid_rowconfigure(1, weight=1)

        self.fbuttons_frame_FAN_ONOFF.grid(row=0, sticky="NESW")
        self.fbuttons_frame_FAN_SPEED.grid(row=1, sticky="NESW")
        self.fbuttons_frame_mainframe.grid_columnconfigure(0, weight=1) # Fill entire space
        self.fbuttons_frame_mainframe.grid_rowconfigure(0, weight=1)
        self.fbuttons_frame_mainframe.grid_rowconfigure(1, weight=1)

        # Subframe 1
        self.fbuttons_label_FAN_STATUS = ttk.Label(master=self.fbuttons_frame_FAN_ONOFF)
        self.fbuttons_button_FAN_ON = ttk.Button(master=self.fbuttons_frame_FAN_ONOFF, text=CONST_TEXT_BUTTON_FAN_ON, command=self.m_startFan)
        self.fbuttons_button_FAN_OFF = ttk.Button(master=self.fbuttons_frame_FAN_ONOFF, text=CONST_TEXT_BUTTON_FAN_OFF, command=self.m_stopFan)       
        
        self.fbuttons_label_FAN_STATUS.grid(row=0, column=0, rowspan=2, sticky="NSEW") 
        self.fbuttons_button_FAN_ON.grid(row=0, column=1, sticky="EW") 
        self.fbuttons_button_FAN_OFF.grid(row=1, column=1,sticky="EW")
        self.fbuttons_frame_FAN_ONOFF.grid_columnconfigure(1, weight=1) # Fill entire space

        self.fbuttons_label_FAN_STATUS.configure(background="white", foreground="green", text="ON", width=5, anchor="center", borderwidth=1, relief="solid")
        self.fbuttons_button_FAN_ON.configure(state="disabled") 
        self.fbuttons_button_FAN_OFF.configure(state="normal")
        
        # Subframe 2
        self.fbuttons_label_FAN_SPEED_LABEL_SLIDER = ttk.Label(master=self.fbuttons_frame_FAN_SPEED, text=CONST_TEXT_LABEL_FAN_SPEED_SLIDER, anchor="center")
        self.fbuttons_slider_FAN_SPEED_SLIDER = tk.Scale(master=self.fbuttons_frame_FAN_SPEED)       
        self.fbuttons_button_FAN_APPLY_SPEED_SLIDER = ttk.Button(master=self.fbuttons_frame_FAN_SPEED, text=CONST_TEXT_BUTTON_FAN_APPLY_SLIDER_SPEED, command=self.m_setFanTestSpeed)     
        self.fbuttons_label_FAN_SPEED_LABEL_REPORTED = ttk.Label(master=self.fbuttons_frame_FAN_SPEED, text=CONST_TEXT_LABEL_FAN_SPEED_REPORTED, anchor="center")
        self.fbuttons_label_FAN_SPEED = ttk.Label(master=self.fbuttons_frame_FAN_SPEED) 

        self.fbuttons_slider_FAN_SPEED_SLIDER.configure(from_=20, to=100, orient='horizontal', tickinterval=10, resolution=10)
        self.fbuttons_slider_FAN_SPEED_SLIDER.set(round(global_fan_speed_user/255.0*1000)/10)
        self.fbuttons_label_FAN_SPEED.configure(background="white", text=f"{global_current_fan_speed:.1f}", anchor="center", width=5, borderwidth=1, relief="solid")
        
        self.fbuttons_label_FAN_SPEED_LABEL_SLIDER.grid(row=0, column=0, columnspan=2, sticky="NSEW")
        self.fbuttons_slider_FAN_SPEED_SLIDER.grid(row=1, column=0, columnspan=2, sticky="NSEW")  
        self.fbuttons_button_FAN_APPLY_SPEED_SLIDER.grid(row=2, column=0, columnspan=2, sticky="NSEW") 
        self.fbuttons_label_FAN_SPEED.grid(row=3, column=0, sticky="NSEW", pady=5)
        self.fbuttons_label_FAN_SPEED_LABEL_REPORTED.grid(row=3, column=1, sticky="NSW")
        self.fbuttons_frame_FAN_SPEED.grid_columnconfigure(1, weight=1) # Fill entire space        
    
    def m_initButtonsAirIntakeFrame(self): 
        ######################
        # BUTTONS AIR INTAKE #
        ######################
        # SUBFRAMES
        self.fair_label_TITLE = ttk.Label(master=self.frameTestAirIntake, text=CONST_TEXT_LABEL_AIR_SEQUENCE_MODE_LABEL, anchor="center")
        self.fair_frame_MAINFRAME = ttk.Frame(master=self.frameTestAirIntake, relief='solid', padding=5, borderwidth=1)

        self.fair_label_TITLE.grid(row=0, column=0, sticky="NEW")
        self.fair_frame_MAINFRAME.grid(row=1, column=0, sticky="NSEW")
        self.frameTestAirIntake.grid_rowconfigure(1, weight=1) # Fill entire space
        self.frameTestAirIntake.grid_columnconfigure(0, weight=1)

        self.fair_frame_buttons = ttk.Frame(master=self.fair_frame_MAINFRAME)
        self.fair_frame_entry = ttk.Frame(master=self.fair_frame_MAINFRAME)

        self.fair_frame_entry.grid(row=0, column=0, sticky="NESW")
        self.fair_frame_buttons.grid(row=1, column=0, sticky="EW")
        self.fair_frame_MAINFRAME.grid_rowconfigure(0, weight=1) # Fill entire space
        self.fair_frame_MAINFRAME.grid_columnconfigure(0, weight=1)

        #Subframe entry
        self.fair_label_SPEED_LABEL = ttk.Label(master=self.fair_frame_entry, text=CONST_TEXT_LABEL_AIR_SPEED, anchor="center")
        self.fair_label_TIME_LABEL = ttk.Label(master=self.fair_frame_entry, text=CONST_TEXT_LABEL_AIR_TIME, anchor="center")

        self.fair_label_INTAKE_AIR_SAMPLE_LABEL = ttk.Label(master=self.fair_frame_entry, text=CONST_TEXT_LABEL_AIR_SAMPLE_LABEL)
        self.fair_entry_INTAKE_AIR_SAMPLE_TIME_VALUE = ttk.Entry(master=self.fair_frame_entry, width=10)
        self.fair_entry_INTAKE_AIR_SAMPLE_SPEED_VALUE = ttk.Entry(master=self.fair_frame_entry, width=10)
        
        self.fair_label_INTAKE_AIR_CLEAR_LABEL = ttk.Label(master=self.fair_frame_entry, text=CONST_TEXT_LABEL_AIR_CLEAR_LABEL)
        self.fair_entry_INTAKE_AIR_CLEAR_TIME_VALUE = ttk.Entry(master=self.fair_frame_entry, width=10)  
        self.fair_entry_INTAKE_AIR_CLEAR_SPEED_VALUE = ttk.Entry(master=self.fair_frame_entry, width=10)      

        self.fair_label_INTAKE_AIR_SEQUENCE_TEST_LABEL = ttk.Label(master=self.fair_frame_entry, text=CONST_TEXT_LABEL_AIR_SEQUENCE_TEST_LABEL)
        self.fair_entry_INTAKE_AIR_SEQUENCE_TEST_TIME_VALUE = ttk.Entry(master=self.fair_frame_entry, width=10)
        self.fair_entry_INTAKE_AIR_SEQUENCE_TEST_SPEED_VALUE = ttk.Entry(master=self.fair_frame_entry, width=10) 

        self.fair_label_INTAKE_AIR_SEQUENCE_RECOVERY_LABEL = ttk.Label(master=self.fair_frame_entry, text=CONST_TEXT_LABEL_AIR_SEQUENCE_RECOVERY_LABEL)
        self.fair_entry_INTAKE_AIR_SEQUENCE_RECOVERY_TIME_VALUE = ttk.Entry(master=self.fair_frame_entry, width=10)  
        self.fair_entry_INTAKE_AIR_SEQUENCE_RECOVERY_SPEED_VALUE = ttk.Entry(master=self.fair_frame_entry, width=10)  

        self.fair_label_INTAKE_LABELAS_LABEL = ttk.Label(master=self.fair_frame_entry, text=CONST_TEXT_LABEL_AIR_INTAKE_LABEL_AS)
        self.fair_entry_INTAKE_LABELAS_VALUE = ttk.Entry(master=self.fair_frame_entry, width=10, justify="center")
        
        self.fair_entry_INTAKE_AIR_SAMPLE_TIME_VALUE.insert(0, f"{global_air_sampling_time:.1f}")
        self.fair_entry_INTAKE_AIR_CLEAR_TIME_VALUE.insert(0, f"{global_air_clearing_time:.1f}")
        self.fair_entry_INTAKE_AIR_SEQUENCE_TEST_TIME_VALUE.insert(0, f"{global_air_test_time:.1f}")
        self.fair_entry_INTAKE_AIR_SEQUENCE_RECOVERY_TIME_VALUE.insert(0, f"{global_air_recovery_time:.1f}")

        self.fair_entry_INTAKE_AIR_SAMPLE_SPEED_VALUE.insert(0, f"{(global_air_sampling_speed)/255*100.0:.1f}")
        self.fair_entry_INTAKE_AIR_CLEAR_SPEED_VALUE.insert(0, f"{(global_air_clearing_speed)/255*100.0:.1f}")
        self.fair_entry_INTAKE_AIR_SEQUENCE_TEST_SPEED_VALUE.insert(0, f"{(global_air_test_speed)/255*100.0:.1f}")
        self.fair_entry_INTAKE_AIR_SEQUENCE_RECOVERY_SPEED_VALUE.insert(0, f"{(global_air_recovery_speed)/255*100.0:.1f}")

        self.fair_entry_INTAKE_LABELAS_VALUE.insert(0, f"{CONST_LABEL_CLASS_DEFAULT}")

        self.fair_label_SPEED_LABEL.grid(row=0, column=2, sticky="NSEW")
        self.fair_label_TIME_LABEL.grid(row=0, column=1, sticky="NSEW")

        self.fair_label_INTAKE_AIR_SAMPLE_LABEL.grid(row=1, column=0, sticky="E")
        self.fair_entry_INTAKE_AIR_SAMPLE_TIME_VALUE.grid(row=1,column=1, sticky="EW", padx=5)
        self.fair_entry_INTAKE_AIR_SAMPLE_SPEED_VALUE.grid(row=1,column=2, sticky="EW", padx=5)

        self.fair_label_INTAKE_AIR_CLEAR_LABEL.grid(row=2, column=0, sticky="E")
        self.fair_entry_INTAKE_AIR_CLEAR_TIME_VALUE.grid(row=2,column=1, sticky="EW", padx=5)
        self.fair_entry_INTAKE_AIR_CLEAR_SPEED_VALUE.grid(row=2,column=2, sticky="EW", padx=5)

        self.fair_label_INTAKE_AIR_SEQUENCE_TEST_LABEL.grid(row=3, column=0, sticky="E")
        self.fair_entry_INTAKE_AIR_SEQUENCE_TEST_TIME_VALUE.grid(row=3, column=1, sticky="EW", padx=5)
        self.fair_entry_INTAKE_AIR_SEQUENCE_TEST_SPEED_VALUE.grid(row=3, column=2, sticky="EW", padx=5)

        self.fair_label_INTAKE_AIR_SEQUENCE_RECOVERY_LABEL.grid(row=4, column=0, sticky="E")
        self.fair_entry_INTAKE_AIR_SEQUENCE_RECOVERY_TIME_VALUE.grid(row=4, column=1, sticky="EW", padx=5)
        self.fair_entry_INTAKE_AIR_SEQUENCE_RECOVERY_SPEED_VALUE.grid(row=4, column=2, sticky="EW", padx=5)
        
        self.fair_label_INTAKE_LABELAS_LABEL.grid(row=5, column=0, sticky="E")
        self.fair_entry_INTAKE_LABELAS_VALUE.grid(row=5, column=1, columnspan=2, sticky="EW", padx=5)

        self.fair_frame_entry.grid_columnconfigure(1, weight=1)
        self.fair_frame_entry.grid_columnconfigure(2, weight=1)

        #Subframe buttons
        self.fair_button_INTAKE_AIR_SAMPLE = ttk.Button(master=self.fair_frame_buttons, text=CONST_TEXT_BUTTON_AIR_INTAKE, command=self.m_getAirSample)
        self.fair_button_INTAKE_CLEAR_CHAMBER = ttk.Button(master=self.fair_frame_buttons, text=CONST_TEXT_BUTTON_AIR_CHAMBER, command=self.m_clearSensorChamber)
        self.fair_button_INTAKE_TEST_SEQUENCE = ttk.Button(master=self.fair_frame_buttons, text=CONST_TEXT_BUTTON_AIR_TEST_SEQUENCE, command=self.m_startAirTestSequence)

        self.fair_button_INTAKE_AIR_SAMPLE.configure(state="disabled")
        self.fair_button_INTAKE_CLEAR_CHAMBER.configure(state="disabled")
        self.fair_button_INTAKE_TEST_SEQUENCE.configure(state="disabled")

        self.fair_button_INTAKE_AIR_SAMPLE.grid(row=0,column=0, sticky="NSEW")
        self.fair_button_INTAKE_CLEAR_CHAMBER.grid(row=1,column=0, sticky="NSEW")
        self.fair_button_INTAKE_TEST_SEQUENCE.grid(row=2,column=0, sticky="NSEW")

        self.fair_frame_buttons.grid_columnconfigure(0, weight=1)

    def m_initContModeFrame(self): 
        #####################
        # CONT MODE CONTROL #
        #####################
        self.fcont_label_TITLE = ttk.Label(master=self.frameContTest, text = CONST_TEXT_LABEL_AIR_CONTINUES_MODE_LABEL, anchor = "center")
        self.fcont_frame_MAINFRAME = ttk.Frame(master=self.frameContTest, relief='solid', padding=5, borderwidth=1)
        
        self.fcont_label_TITLE.grid(row=0, column=0, sticky="NSEW")
        self.fcont_frame_MAINFRAME.grid(row=1, column=0, sticky="NSEW")

        self.frameContTest.grid_columnconfigure(0,weight=1)
        self.frameContTest.grid_rowconfigure(1,weight=1)

        self.fcont_label_LABELAS_LABEL = ttk.Label(master=self.fcont_frame_MAINFRAME, text=CONST_TEXT_LABEL_AIR_INTAKE_LABEL_AS)
        self.fcont_entry_LABELAS_VALUE = ttk.Entry(master=self.fcont_frame_MAINFRAME, justify="center")
        self.fcont_label_MARK_SAMPLES_LEN = ttk.Label(master=self.fcont_frame_MAINFRAME, text=CONST_TEXT_LABEL_MARK_NEXT_SAMPLES_LEN)
        self.fcont_entry_MARK_SAMPLES_LEN = ttk.Entry(master=self.fcont_frame_MAINFRAME, justify="center")
        self.fcont_button_MARK_SAMPLE_AS = ttk.Button(master=self.fcont_frame_MAINFRAME, text=CONST_TEXT_BUTTON_MARK_NEXT_SAMPLES, command=self.m_markNextSamples)
        self.fcont_label_MARK_SAMPLES_TIME_LEN = ttk.Label(master=self.fcont_frame_MAINFRAME, text=CONST_TEXT_LABEL_MARK_NEXT_SAMPLES_TIME_LEN)
        self.fcont_entry_MARK_SAMPLES_TIME_LEN = ttk.Entry(master=self.fcont_frame_MAINFRAME, justify="center")
        self.fcont_button_MARK_SAMPLE_AS_TIME = ttk.Button(master=self.fcont_frame_MAINFRAME, text=CONST_TEXT_BUTTON_MARK_NEXT_SAMPLES, command=self.m_markNextSamplesTime)


        self.fcont_entry_LABELAS_VALUE.insert(0, f"{CONST_LABEL_CLASS_DEFAULT}")
        self.fcont_entry_MARK_SAMPLES_LEN.insert(0, f"{global_class_name_override_samples}") 
        self.fcont_entry_MARK_SAMPLES_TIME_LEN.insert(0, f"{global_class_name_override_samples/global_freq}")        

        self.fcont_button_MARK_SAMPLE_AS.configure(state="disabled")
        self.fcont_button_MARK_SAMPLE_AS_TIME.configure(state="disabled")

        self.fcont_label_LABELAS_LABEL.grid(row=1, column=0, sticky="NSE", padx=5)
        self.fcont_entry_LABELAS_VALUE.grid(row=1, column=1, sticky="NSEW")
        self.fcont_label_MARK_SAMPLES_LEN.grid(row=2, column=0, sticky="NSE", padx=5)
        self.fcont_entry_MARK_SAMPLES_LEN.grid(row=2, column=1, sticky="NSEW")
        self.fcont_button_MARK_SAMPLE_AS.grid(row=3, column=0, columnspan=2, sticky="NSEW")
        self.fcont_label_MARK_SAMPLES_TIME_LEN.grid(row=4, column=0, sticky="NSE", padx=5)
        self.fcont_entry_MARK_SAMPLES_TIME_LEN.grid(row=4, column=1, sticky="NSEW")
        self.fcont_button_MARK_SAMPLE_AS_TIME.grid(row=5, column=0, columnspan=2, sticky="NSEW")

        self.fcont_frame_MAINFRAME.grid_columnconfigure(1,weight=1)
        self.fcont_frame_MAINFRAME.grid_rowconfigure(0,weight=1)
        self.fcont_frame_MAINFRAME.grid_rowconfigure(1,weight=1)
        self.fcont_frame_MAINFRAME.grid_rowconfigure(2,weight=1)
        self.fcont_frame_MAINFRAME.grid_rowconfigure(3,weight=1)
        self.fcont_frame_MAINFRAME.grid_rowconfigure(4,weight=1)
        self.fcont_frame_MAINFRAME.grid_rowconfigure(5,weight=1)

    def m_initDataFrame(self):
        ##############
        # DATA FRAME #
        ##############
        #SUBFRAMES
        self.fdata_label_LOGTITLE = ttk.Label(master=self.frameData, text=CONST_TEXT_LOG_LABEL, anchor="center") # GUI minor elements (LOG)
        self.fdata_subframe_DATA = ttk.Frame(master=self.frameData, relief='solid', borderwidth=1, padding=5)
        self.fdata_subframe_DATASENSORS = ttk.Frame(master=self.frameData, relief='solid', borderwidth=1, padding=5)

        self.fdata_label_LOGTITLE.grid(row=0, columnspan=3, sticky="EW")
        self.fdata_subframe_DATA.grid(row=1, column=0, sticky="NSEW")
        self.fdata_subframe_DATASENSORS.grid(row=1, column=2, sticky="NSEW")
        self.frameData.grid_columnconfigure(0, weight=1) # Fill entire space
        self.frameData.grid_columnconfigure(1, minsize=10)
        self.frameData.grid_columnconfigure(2, weight=1)
        self.frameData.grid_rowconfigure(0, weight=1)
        self.frameData.grid_rowconfigure(1, weight=1)
        
        # Subframe 1
        self.fdata_label_TIME = ttk.Label(master=self.fdata_subframe_DATA, text = CONST_TEXT_LABEL_TIME, anchor="center")
        self.fdata_label_NN_OUT = ttk.Label(master=self.fdata_subframe_DATA, text = CONST_TEXT_NN_OUT, anchor="center")
        self.flogs_label_TEMPERATURE = ttk.Label(master=self.fdata_subframe_DATA, text = CONST_TEXT_LABEL_TEMPERATURE, anchor="center")
        self.flogs_label_CLASS = ttk.Label(master=self.fdata_subframe_DATA, text = CONST_TEXT_LABEL_CLASS, anchor="center")
        self.fdata_text_TIME = tk.Text(master=self.fdata_subframe_DATA, height=1, width=CONST_INT_BUTTON_ELEMENT_WIDTH, state="disabled", font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey")
        self.fdata_text_NN_OUT = tk.Text(master=self.fdata_subframe_DATA, height=1, width=CONST_INT_BUTTON_ELEMENT_WIDTH, state="disabled", font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey")
        self.flogs_text_TEMPERATURE = tk.Text(master=self.fdata_subframe_DATA, height=1, width=CONST_INT_BUTTON_ELEMENT_WIDTH, state="disabled", font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey")
        self.flogs_text_CLASS = tk.Text(master=self.fdata_subframe_DATA, height=1, width=CONST_INT_BUTTON_ELEMENT_WIDTH, state="disabled", font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey")
    

        self.fdata_label_TIME.grid(row=0, column=0, padx=5, sticky="NSE")
        self.fdata_text_TIME.grid(row=0, column=1, sticky="EW")
        self.flogs_label_TEMPERATURE.grid(row=1, column=0, padx=5, sticky="NSE")
        self.flogs_text_TEMPERATURE.grid(row=1, column=1, sticky="EW")
        self.flogs_label_CLASS.grid(row=2, column=0, padx=5, sticky="NSE")
        self.flogs_text_CLASS.grid(row=2, column=1, sticky="EW")
        self.fdata_label_NN_OUT.grid(row=3, column=0, padx=5, sticky="NSE")
        self.fdata_text_NN_OUT.grid(row=3, column=1, sticky="EW")

        #self.fdata_subframe_DATA.grid_columnconfigure(0, weight=1) # Fill entire space
        self.fdata_subframe_DATA.grid_columnconfigure(1, weight=1)
        self.fdata_subframe_DATA.grid_rowconfigure(0, weight=1)
        self.fdata_subframe_DATA.grid_rowconfigure(1, weight=1)
        self.fdata_subframe_DATA.grid_rowconfigure(2, weight=1)
        self.fdata_subframe_DATA.grid_rowconfigure(3, weight=1)

        # Subframe 2
        self.fdata_label_INPUTNAME = ttk.Label(master=self.fdata_subframe_DATASENSORS,text = "Input:")
        self.fdata_label_PARAMNAME1 = ttk.Label(master=self.fdata_subframe_DATASENSORS,text = "Parameter 1:")
        self.fdata_label_PARAMNAME2 = ttk.Label(master=self.fdata_subframe_DATASENSORS,text = "Parameter 2:")
        self.fdata_label_PARAMNAME3 = ttk.Label(master=self.fdata_subframe_DATASENSORS,text = "Parameter 3:")
        
        self.fdata_label_INPUTS = list()
        self.fdata_text_INPUTS = list()
        self.fdata_text_INPUTS_2 = list()
        self.fdata_text_INPUTS_3 = list()
        self.fdata_text_INPUTS_4 = list()

        for i in range(5):
            self.fdata_label_INPUTS.append(ttk.Label(master=self.fdata_subframe_DATASENSORS, text = f"{CONST_LIST_SENSORS[i]}"))
            self.fdata_text_INPUTS.append(tk.Text(master=self.fdata_subframe_DATASENSORS, width=CONST_INT_TEXT_ELEMENT_WIDTH,height=1,state="disabled",font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey"))
            self.fdata_text_INPUTS_2.append(tk.Text(master=self.fdata_subframe_DATASENSORS, width=CONST_INT_TEXT_ELEMENT_WIDTH,height=1,state="disabled",font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey"))
            self.fdata_text_INPUTS_3.append(tk.Text(master=self.fdata_subframe_DATASENSORS, width=CONST_INT_TEXT_ELEMENT_WIDTH,height=1,state="disabled",font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey"))
            self.fdata_text_INPUTS_4.append(tk.Text(master=self.fdata_subframe_DATASENSORS, width=CONST_INT_TEXT_ELEMENT_WIDTH,height=1,state="disabled",font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey"))

        # Placing inside subframe "flogs_frame_DATA"
        self.fdata_label_INPUTNAME.grid(row=0, column=1, columnspan=1)  
        self.fdata_label_PARAMNAME1.grid(row=0, column=2, columnspan=1)  
        self.fdata_label_PARAMNAME2.grid(row=0, column=3, columnspan=1)    
        self.fdata_label_PARAMNAME3.grid(row=0, column=4, columnspan=1)  
        for i,_ in enumerate(self.fdata_label_INPUTS):
            self.fdata_label_INPUTS[i].grid(row=i+1, column=0, sticky="E", padx=1)
            self.fdata_text_INPUTS[i].grid(row=i+1, column=1, sticky="EW", padx=1)
            self.fdata_text_INPUTS_2[i].grid(row=i+1, column=2, sticky="EW", padx=1)
            self.fdata_text_INPUTS_3[i].grid(row=i+1, column=3, sticky="EW", padx=1)
            self.fdata_text_INPUTS_4[i].grid(row=i+1, column=4, sticky="EW", padx=1)
        #self.fdata_subframe_DATASENSORS.grid_columnconfigure(0, weight=1) # Fill entire space
        self.fdata_subframe_DATASENSORS.grid_columnconfigure(1, weight=1)
        self.fdata_subframe_DATASENSORS.grid_columnconfigure(2, weight=1)
        self.fdata_subframe_DATASENSORS.grid_columnconfigure(3, weight=1)
        self.fdata_subframe_DATASENSORS.grid_columnconfigure(4, weight=1)
        self.fdata_subframe_DATASENSORS.grid_rowconfigure(0, weight=1)
        self.fdata_subframe_DATASENSORS.grid_rowconfigure(1, weight=1)
        self.fdata_subframe_DATASENSORS.grid_rowconfigure(2, weight=1)
        self.fdata_subframe_DATASENSORS.grid_rowconfigure(3, weight=1)
        self.fdata_subframe_DATASENSORS.grid_rowconfigure(4, weight=1)

    def m_initLogsFrame(self):
        #############
        # TEXT LOGS #
        #############
        self.flogs_scrolltext_LOGGEDTEXT = tkscroll.ScrolledText(master=self.frameLogs, height=CONST_INT_SCROLLEDTEXT_LOG_HEIGHT, font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey")
        self.flogs_button_CLEARLOGS = ttk.Button(master=self.frameLogs, text=CONST_TEXT_CHECK_CLEAR, command=self.m_clearLogs)
        
        self.frameLogs.grid_columnconfigure(0, weight=1)

        self.flogs_scrolltext_LOGGEDTEXT.grid(row=0,sticky="EW")
        self.flogs_button_CLEARLOGS.grid(row=1,pady=5,sticky="NSEW")       

    def m_get_root_size(self):
        w,h = self.root.winfo_width(), self.root.winfo_height()
        return w,h

    def m_placeTempImg(self):
        self.root.geometry(f"+{0}+{0}")
        self.root.title(CONST_TEXT_LOADING)
        img = ImageTk.PhotoImage(Image.open(CONST_SPLASHSCREEN_PATH).resize((500, 500)))
        label = ttk.Label(master=self.root, image = img, compound='center')
        label.config(text=CONST_TEXT_LOADING, background="white", foreground="white", compound='center', font=("Bahnschrift Bold Condensed", 16))
        label.grid(row=0,column=0)
        self.root.update()

    def m_plot(self):
        self.ObjPlotter.m_plot()

    def m_print(self, record):
        # prints text to LOG
        if int(self.flogs_scrolltext_LOGGEDTEXT.index('end-1c').split('.')[0]) > CONST_INT_MAX_LOGGEDTEXT_HEIGHT:
            self.m_clearLogs()
        self.flogs_scrolltext_LOGGEDTEXT.insert(tk.END, f"{record}\n")
        self.flogs_scrolltext_LOGGEDTEXT.see("end")
        print(str(record))
    
    def m_updateInfoFrame(self):     
        self.fdata_text_TIME.configure(state="normal") # set state of text widgets to normal
        for i,_ in enumerate(self.fdata_text_INPUTS):
            self.fdata_text_INPUTS[i].configure(state="normal")
            self.fdata_text_INPUTS_2[i].configure(state="normal")
            self.fdata_text_INPUTS_3[i].configure(state="normal")
            self.fdata_text_INPUTS_4[i].configure(state="normal")
        self.flogs_text_TEMPERATURE.configure(state="normal")
        self.fdata_text_NN_OUT.configure(state="normal")
        self.flogs_text_CLASS.configure(state="normal")
            
        self.fdata_text_TIME.delete('1.0', tk.END)
        self.fdata_text_TIME.insert(tk.END, f"{global_current_time:.3f}")
        for i,_ in enumerate(self.fdata_text_INPUTS):
            self.fdata_text_INPUTS[i].delete('1.0', tk.END)
            self.fdata_text_INPUTS[i].insert(tk.END, f"{global_current_inputs[i]}")
            self.fdata_text_INPUTS_2[i].delete('1.0', tk.END)
            self.fdata_text_INPUTS_2[i].insert(tk.END, f"{global_current_inputs_2[i]}")
            self.fdata_text_INPUTS_3[i].delete('1.0', tk.END)
            self.fdata_text_INPUTS_3[i].insert(tk.END, f"{global_current_inputs_3[i]}")
            self.fdata_text_INPUTS_4[i].delete('1.0', tk.END)
            self.fdata_text_INPUTS_4[i].insert(tk.END, f"{global_current_inputs_4[i]}")
        self.flogs_text_TEMPERATURE.delete('1.0', tk.END)
        self.flogs_text_TEMPERATURE.insert(tk.END, f"{global_current_temperature}")
        self.fdata_text_NN_OUT.delete('1.0', tk.END)
        self.fdata_text_NN_OUT.insert(tk.END, f"{global_NN_output}")
        self.flogs_text_CLASS.delete('1.0', tk.END)
        self.flogs_text_CLASS.insert(tk.END, f"{global_current_class_label}")
        
        self.fdata_text_TIME.configure(state="disabled") # set state of text widgets to disabled
        for i,_ in enumerate(self.fdata_text_INPUTS):
            self.fdata_text_INPUTS[i].configure(state="disabled")
            self.fdata_text_INPUTS_2[i].configure(state="disabled")
            self.fdata_text_INPUTS_3[i].configure(state="disabled")
            self.fdata_text_INPUTS_4[i].configure(state="disabled")
        self.flogs_text_TEMPERATURE.configure(state="disabled")  
        self.fdata_text_NN_OUT.configure(state="disabled")   
        self.flogs_text_CLASS.configure(state="disabled")

        self.fbuttons_label_FAN_SPEED.configure(text=f"{global_current_fan_speed:.1f}" )      

    def m_getDataFromEntry(self):
        # gets data from entry elements and updates UART parameters
        global global_baudrate
        global global_freq
        global global_timeout
        global global_path_to_output_dir
        global global_waiting_buffer_size
        global global_waiting_buffer_input_list
        global global_data_window_skip_samples
        global global_buffer_kernel
        global global_data_window_resampling_delay

        e1 = int(self.fparam_entry_BAUD.get())
        e2 = int(self.fparam_entry_FREQ.get())
        e3 = str(self.fparam_entry_PATH.get())
        if e2 < CONST_INT_FREQ_MIN: e2 = CONST_INT_FREQ_MIN
        elif e2 > CONST_INT_FREQ_MAX: e2 = CONST_INT_FREQ_MAX
        self.ObjPlotter.m_getDataFromDataFrameEntry()
        global_baudrate = e1
        global_freq = e2
        if global_data_window_resampling_delay < (1.0 / global_freq):
            global_data_window_resampling_delay = 1.0 / global_freq
        #global_timeout = 1.0 / global_freq * 5 # Calculate timeout
        global_path_to_output_dir = e3
        global_waiting_buffer_input_list, global_waiting_buffer_size = getWaitingBuffer()
        global_data_window_skip_samples = getSamplesToSkip()
        global_buffer_kernel = calcBufferGaussKernel()
        self.m_print("Baudrate set to " + str(global_baudrate) + ", Frequency set to " + str(global_freq) + "Hz, Timeout set to " + str(global_timeout) + "s\nPATH: " + str(global_path_to_output_dir))
        self.ObjPlotter.m_updateDataFrame()
        self.ObjPlotter.m_resetPlot()

    def m_getDirectoryToSave(self):
        global global_path_to_output_dir
        global_path_to_output_dir = fd.askdirectory(initialdir=global_path_to_output_dir)

    def m_toggleSaveFile(self):
        global global_save_to_file
        global_save_to_file = self.var_save_to_file_state.get()

    def m_toggleLogData(self):
        global global_log_data
        global_log_data = self.var_log_data_state.get()

    def m_startCommunication(self):
        global global_transmission_state
        global global_sample_counter
        global_sample_counter = 0
        clearAllCachedData()
        if global_transmission_state == False:
            programCommunicationCycle()

    def m_startFan(self):
        global global_fan_speed_user
        global global_fan_status

        self.fbuttons_button_FAN_ON.configure(state="disabled") 
        self.fbuttons_button_FAN_OFF.configure(state="normal")
        self.fbuttons_label_FAN_STATUS.configure(text="ON", foreground="green")
        
        global_fan_status = True
        self.m_setFanTestSpeed()

    def m_stopFan(self):
        global global_fan_speed_user
        global global_fan_status

        self.fbuttons_button_FAN_ON.configure(state="normal") 
        self.fbuttons_button_FAN_OFF.configure(state="disabled") 
        self.fbuttons_label_FAN_STATUS.configure(text="OFF", foreground="red")
        
        global_fan_status = False

        self.m_setFanTestSpeed()

    def m_setFanTestSpeed(self, force_val=None):
        global global_fan_speed_user
        global global_fan_status 

        if not global_fan_status:
            global_fan_speed_user = CONST_INT_MINIMUM_FAN_SPEED
        elif force_val != None:
            global_fan_speed_user = int(force_val)
        else:
            global_fan_speed_user = self.m_getFanSpeedSlider()

        if ObjPort.m_checkPortValid(): # If not in test state transmit new fan speed to MCU and wait for confirmation
            output = f"FANS{global_fan_speed_user:d}"
            expected_input = output
            ObjPort.m_transmitAndWaitFor(output, expected_input, bRemoveSpaces=True)

    def m_setFanControlSpeed(self, force_val=None):
        self.m_setFanTestSpeed()

    def m_markNextSamples(self):
        global global_class_name_override_status

        global_class_name_override_status = True
        self.m_getClassLabelContMode()
        self.m_getNumberOfSamplesToMark()

    def m_markNextSamplesTime(self):
        global global_class_name_override_status

        global_class_name_override_status = True
        self.m_calcSamplesToMark()
        self.m_getClassLabelContMode()


    def m_calcSamplesToMark(self):
        global global_class_name_override_samples
        time = float(self.fcont_entry_MARK_SAMPLES_TIME_LEN.get())
        if time <= 1.0/global_freq:
           samples = 1
           time = 1.0/global_freq
        else: 
            samples = int(time * global_freq)
        self.fcont_entry_MARK_SAMPLES_TIME_LEN.delete(0,tk.END)
        self.fcont_entry_MARK_SAMPLES_TIME_LEN.insert(0, f"{time}")    
        self.fcont_entry_MARK_SAMPLES_LEN.delete(0,tk.END)
        self.fcont_entry_MARK_SAMPLES_LEN.insert(0, f"{samples}")
        global_class_name_override_samples = samples

    def m_getNumberOfSamplesToMark(self):
        global global_class_name_override_samples
        global_class_name_override_samples = int(self.fcont_entry_MARK_SAMPLES_LEN.get())
        if global_class_name_override_samples <= 0:
            global_class_name_override_samples = 1
        self.fcont_entry_MARK_SAMPLES_LEN.delete(0,tk.END)
        self.fcont_entry_MARK_SAMPLES_LEN.insert(0, f"{global_class_name_override_samples}")
        self.fcont_entry_MARK_SAMPLES_TIME_LEN.delete(0,tk.END)
        self.fcont_entry_MARK_SAMPLES_TIME_LEN.insert(0, f"{global_class_name_override_samples/global_freq}")    

    def m_getAirSample(self):
        global global_air_sampling_status
        global global_air_sampling_time_start
        global global_air_sampling_time
        global global_current_class_label

        global_air_sampling_status = True
        self.m_getAirTimes()
        self.m_getClassLabel()
        self.m_getAirSpeeds()
        global_air_sampling_time_start = time.time()
    
    def m_clearSensorChamber(self):
        global global_air_clearing_status
        global global_air_clearing_time
        global global_air_clearing_time_start
        global global_current_class_label

        global_air_clearing_status = True
        self.m_getAirTimes()
        self.m_getAirSpeeds()
        global_current_class_label = CONST_LABEL_CLASS_CLEARING
        global_air_clearing_time_start = time.time()
    
    def m_getAirSpeeds(self):
        global global_air_clearing_speed
        global global_air_sampling_speed
        global global_air_test_speed
        global global_air_recovery_speed

        global_air_clearing_speed = int(round(float(self.fair_entry_INTAKE_AIR_CLEAR_SPEED_VALUE.get()) / 100.0 * 255.0))
        global_air_sampling_speed = int(round(float(self.fair_entry_INTAKE_AIR_SAMPLE_SPEED_VALUE.get()) / 100.0 * 255.0))
        global_air_test_speed = int(round(float(self.fair_entry_INTAKE_AIR_SEQUENCE_TEST_SPEED_VALUE.get()) / 100.0 * 255.0))
        global_air_recovery_speed = int(round(float(self.fair_entry_INTAKE_AIR_SEQUENCE_RECOVERY_SPEED_VALUE.get()) / 100.0 * 255.0))

    def m_getAirTimes(self):
        global global_air_sampling_time
        global global_air_clearing_time
        global global_air_recovery_time
        global global_air_test_time

        global_air_clearing_time = float(self.fair_entry_INTAKE_AIR_CLEAR_TIME_VALUE.get())
        global_air_sampling_time = float(self.fair_entry_INTAKE_AIR_SAMPLE_TIME_VALUE.get())
        global_air_recovery_time = float(self.fair_entry_INTAKE_AIR_SEQUENCE_RECOVERY_TIME_VALUE.get())
        global_air_test_time = float(self.fair_entry_INTAKE_AIR_SEQUENCE_TEST_TIME_VALUE.get())

    def m_getClassLabel(self):
        global global_current_class_label
        global_current_class_label = str(self.fair_entry_INTAKE_LABELAS_VALUE.get())
        if global_current_class_label.lower() in [CONST_LABEL_CLASS_TEST, CONST_LABEL_CLASS_CLEARING, CONST_LABEL_CLASS_RECOVERY, CONST_LABEL_CLASS_NONE]:
            global_current_class_label = CONST_LABEL_CLASS_DEFAULT

    def m_getClassLabelContMode(self):
        global global_current_class_label
        global_current_class_label = str(self.fcont_entry_LABELAS_VALUE.get())
        if global_current_class_label.lower() in [CONST_LABEL_CLASS_TEST, CONST_LABEL_CLASS_CLEARING, CONST_LABEL_CLASS_RECOVERY, CONST_LABEL_CLASS_NONE]:
            global_current_class_label = CONST_LABEL_CLASS_DEFAULT

    def m_startAirTestSequence(self):
        global global_test_sequence_status
        global global_test_sequence_times
        global global_test_sequence_class_labels
        global global_test_sequence_time_start
        global global_air_sequence_data_sensors
        global global_air_sequence_time
        global global_air_sequence_data_temperature
        global global_air_sequence_fan_speed

        global_air_sequence_data_sensors = [list(),list(),list(),list(),list()]
        global_air_sequence_time = list()
        global_air_sequence_data_temperature = list()
        global_air_sequence_fan_speed = list()

        self.fair_button_INTAKE_AIR_SAMPLE.configure(state="disabled")
        self.fair_button_INTAKE_CLEAR_CHAMBER.configure(state="disabled")
        self.fair_button_INTAKE_TEST_SEQUENCE.configure(state="disabled")
        
        global_test_sequence_status = [True, True, True, True]
        self.m_getAirTimes()
        self.m_getClassLabel()
        self.m_getAirSpeeds()
        global_test_sequence_times = [global_air_sampling_time, global_air_test_time, global_air_clearing_time, global_air_recovery_time]
        global_test_sequence_class_labels[0] = global_current_class_label

        global_test_sequence_time_start = time.time()

    def m_endAirTestSequence(self):
        global global_current_class_label
        global global_test_sequence_status
        global global_air_sequence_time
        global global_air_sequence_data_sensors
        global global_air_sequence_data_temperature
        global global_air_sequence_fan_speed
        
        global_test_sequence_status = [False, False, False, False]
        global_current_class_label = CONST_LABEL_CLASS_NONE

        global_air_sequence_time = np.array(global_air_sequence_time)
        global_air_sequence_data_sensors = np.array(global_air_sequence_data_sensors)
        global_air_sequence_data_temperature = np.array(global_air_sequence_data_temperature)
        global_air_sequence_fan_speed = np.array(global_air_sequence_fan_speed)
        
        global_air_sequence_time -= global_air_sequence_time[0]
        global_air_sequence_data_sensors
        global_air_sequence_data_temperature
        global_air_sequence_fan_speed

        CTkPlotterTestResults(root=ObjMainWindow).m_plot()

        del global_air_sequence_time
        del global_air_sequence_data_sensors
        del global_air_sequence_data_temperature
        del global_air_sequence_fan_speed

        self.fair_button_INTAKE_AIR_SAMPLE.configure(state="normal")
        self.fair_button_INTAKE_CLEAR_CHAMBER.configure(state="normal")
        self.fair_button_INTAKE_TEST_SEQUENCE.configure(state="normal")

    def m_getFanSpeedSlider(self):
        return int(round((self.fbuttons_slider_FAN_SPEED_SLIDER.get()) / 100.0 * 255.0))

    def m_stopCommunication(self):
        # changes global bool var to stop communication
        global global_transmission_state
        global_transmission_state = False
    
    def m_updateDeviceName(self, devicename):
        self.fbuttons_text_DEVICEINFO.configure(text=str(devicename))

    def m_update(self):
        # updates window frame
        self.root.update()

    def m_mainloop(self):
        self.root.mainloop()

    def m_clearLogs(self):
        # clears LOG
        self.flogs_scrolltext_LOGGEDTEXT.delete('1.0', tk.END)
    
    def m_destroyWindow(self):
        self.root.destroy()

    def m_windowExists(self):
        return self.root.winfo_exists()

class CTkPlotter():
    def __init__(self, root):
        self.m_setMasterWindow(root)
        self.masterFrame = tk.Frame(master=self.mainwindow.root)
        self.masterFrame.grid(row=0,column=1,sticky="NESW")

        self.label_TITLE = ttk.Label(master=self.masterFrame, padding=5, background="grey", text=CONST_TEXT_LABEL_TITLE, anchor="center")
        self.pyplotFrame = ttk.Frame(master=self.masterFrame, padding=5)
        self.dataFrame= ttk.Frame(master=self.masterFrame, padding=5)
        self.pyplotSettingsFrame = ttk.Frame(master=self.masterFrame, padding=5)
        self.pcaFrame = ttk.Frame(master=self.masterFrame, padding=5)

        self.label_TITLE.grid(row=0, column=0, sticky="NSEW")
        self.dataFrame.grid(row=1, column=0, sticky="EW")
        self.pyplotSettingsFrame.grid(row=2, column=0, sticky="EW")
        self.pcaFrame.grid(row=3, column=0, sticky="NSEW")
        self.pyplotFrame.grid(row=4, column=0, sticky="NS")

        self.masterFrame.grid_columnconfigure(0, weight=1)
        self.masterFrame.grid_rowconfigure(3, weight=1)

        self.boolCanvasPlaced = False
        self.axesLimitsList = [[-50,1050],[-50,1050],[-50,1050],[-50,1050],[-50,1050],[0,100]]

        self.figs_list = [None,None,None,None,None,None]
        self.axes_list = [None,None,None,None,None,None]
        self.canvas_list = [None,None,None,None,None,None]
        self.backgrounds_list = [None,None,None,None,None,None]

        #Init all elements (frames and plots)
        self.m_initDataFrame()
        self.m_initSettingsFrame()
        self.m_initPlot()
        self.m_initPCAFrame()
    
    def m_initSettingsFrame(self):
        self.var_save_to_file_state = tk.BooleanVar(value=True)
        self.autoscale_plot = True

        self.fsettings_label_title = ttk.Label(master=self.pyplotSettingsFrame, text="Y-Axis settings:", anchor="center")
        self.fsettings_masterframe = ttk.Frame(master=self.pyplotSettingsFrame, relief='solid', borderwidth=1, padding=5)

        self.fsettings_label_title.grid(row=0, sticky="EW")
        self.fsettings_masterframe.grid(row=1, sticky="NSEW")
        self.pyplotSettingsFrame.grid_rowconfigure(0,  weight=1)
        self.pyplotSettingsFrame.grid_rowconfigure(1,  weight=1)

        self.fsettings_label_sensor = list()
        self.fsettings_entry_min = list()
        self.fsettings_entry_max = list()
        self.fsettings_entry_baseline = list()
        self.fsettings_label_min = ttk.Label(master=self.fsettings_masterframe, text="Min:", anchor="center")
        self.fsettings_label_max = ttk.Label(master=self.fsettings_masterframe, text="Max:", anchor="center")
        self.fsettings_label_baseline = ttk.Label(master=self.fsettings_masterframe, text="Base:", anchor="center")
        self.fsettings_button_apply = ttk.Button(master=self.fsettings_masterframe, text=CONST_TEXT_BUTTON_APPLY_PARAMS, command=self.m_buttonUpdatePlotSettings)
        self.fsettings_button_baseline = ttk.Button(master=self.fsettings_masterframe, text="Set baseline", command=self.m_buttonRemoveBaseline)
        self.fsettings_check_autoscale_plot = ttk.Checkbutton(master=self.fsettings_masterframe, text=CONST_TEXT_CHECK_AUTOSCALE, variable=self.var_save_to_file_state, onvalue=True, offvalue=False, command=None)
        
        self.fsettings_label_min.grid(row=1,column=0,padx=5, sticky="E")
        self.fsettings_label_max.grid(row=2,column=0,padx=5, sticky="E")
        self.fsettings_label_baseline.grid(row=3,column=0,padx=5, sticky="E")
        self.fsettings_check_autoscale_plot.grid(row=0,rowspan=4,column=7, sticky="NSE")
        self.fsettings_button_apply.grid(row=1,rowspan=2,column=8, sticky="NSEW")
        self.fsettings_button_baseline.grid(row=3,rowspan=2,column=8, sticky="NSEW") 
        for i in range(5):
            self.fsettings_label_sensor.append(ttk.Label(master=self.fsettings_masterframe, text=CONST_LIST_SENSORS[i]))
            self.fsettings_entry_min.append(ttk.Entry(master=self.fsettings_masterframe, width=10))
            self.fsettings_entry_max.append(ttk.Entry(master=self.fsettings_masterframe, width=10))
            self.fsettings_entry_baseline.append(ttk.Entry(master=self.fsettings_masterframe, width=10))

            self.fsettings_label_sensor[i].grid(row=0,column=i+1,padx=5)
            self.fsettings_entry_min[i].grid(row=1,column=i+1,padx=5)
            self.fsettings_entry_max[i].grid(row=2,column=i+1,padx=5)
            self.fsettings_entry_baseline[i].grid(row=3,column=i+1,padx=5)
            
            self.fsettings_entry_min[i].delete(0, 'end')
            self.fsettings_entry_max[i].delete(0, 'end')
            self.fsettings_entry_baseline[i].delete(0, 'end')
            self.fsettings_entry_min[i].insert(10, f"{self.axesLimitsList[i][0]}")
            self.fsettings_entry_max[i].insert(10, f"{self.axesLimitsList[i][1]}")
            self.fsettings_entry_baseline[i].insert(10, f"{global_baseline_data_sensors[i]}")

    def m_initDataFrame(self):
        self.fdata_entry_SIZE_PLOT_SAMPLES = ttk.Entry(master=self.dataFrame, width=20)
        self.fdata_text_SIZEPLOTSECONDS = tk.Text(master=self.dataFrame, height=1 ,width=CONST_INT_BUTTON_ELEMENT_WIDTH,font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey")
        self.fdata_label_SIZEPLOTSAMPLES = ttk.Label(master=self.dataFrame, text=CONST_TEXT_LABEL_SIZE_PLOT, anchor="center")

        self.fdata_text_SIZEBUFFERSAMPLES = tk.Text(master=self.dataFrame,height=1,width=CONST_INT_BUTTON_ELEMENT_WIDTH,font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey")
        self.fdata_text_SIZEBUFFERSECONDS = tk.Text(master=self.dataFrame,height=1,width=CONST_INT_BUTTON_ELEMENT_WIDTH,font=CONST_DEFAULT_FONT, relief='flat', highlightthickness=1, highlightbackground="grey")
        self.fdata_label_SIZEBUFFERSAMPLES = ttk.Label(master=self.dataFrame, text=CONST_TEXT_LABEL_SIZE_BUFFER, anchor="center")

        self.fdata_text_SIZEBUFFERSAMPLES.configure(state="disabled", foreground="grey")
        self.fdata_text_SIZEBUFFERSECONDS.configure(state="disabled", foreground="grey")

        self.fdata_entry_FREQ_PLOT_SAMPLES = ttk.Entry(master=self.dataFrame)
        self.fdata_label_FREQPLOT = ttk.Label(master=self.dataFrame, text=CONST_TEXT_LABEL_FREQPLOT)
        self.fdata_button_FREQPLOT = ttk.Button(master=self.dataFrame, text=CONST_TEXT_BUTTON_APPLY_PARAMS, command=self.mainwindow.m_getDataFromEntry)

        self.fdata_entry_SIZE_ANALYSIS_WINDOW = ttk.Entry(master=self.dataFrame)
        self.fdata_label_SIZE_ANALYSIS_WINDOW = ttk.Label(master=self.dataFrame, text=CONST_TEXT_LABEL_ANALYSIS)
        
        self.fdata_label_SIZEBUFFERSAMPLES.grid(row=0, column=0, sticky="NSE", pady=1)  
        self.fdata_text_SIZEBUFFERSAMPLES.grid(row=0, column=1, sticky="EW", padx=5)
        self.fdata_text_SIZEBUFFERSECONDS.grid(row=0, column=2, sticky="EW", padx=5)      
        
        self.fdata_label_SIZEPLOTSAMPLES.grid(row=1, column=0, sticky="NSE", pady=1)
        self.fdata_entry_SIZE_PLOT_SAMPLES.grid(row=1, column=1, sticky="EW", padx=5)
        self.fdata_text_SIZEPLOTSECONDS.grid(row=1, column=2, sticky="EW", padx=5)

        self.fdata_label_FREQPLOT.grid(row=2, column=0, sticky="NSE", pady=1)
        self.fdata_entry_FREQ_PLOT_SAMPLES.grid(row=2, column=1, sticky="EW", padx=5)
        self.fdata_button_FREQPLOT.grid(row=2, column=2, rowspan=2, sticky="NSEW", padx=5)

        self.fdata_label_SIZE_ANALYSIS_WINDOW.grid(row=3, column=0, sticky="NSE", pady=1)
        self.fdata_entry_SIZE_ANALYSIS_WINDOW.grid(row=3, column=1, sticky="EW", padx=5)

        self.dataFrame.grid_columnconfigure(1, weight=1)
        self.dataFrame.grid_columnconfigure(2, weight=1)


        self.fdata_entry_SIZE_PLOT_SAMPLES.insert(10, f"{global_data_window_size}")
        self.fdata_entry_FREQ_PLOT_SAMPLES.insert(10, f"{1.0/global_data_window_resampling_delay:.1f}")

        self.m_updateDataFrame()

    def m_initPCAFrame(self):
        self.pcaFrame_MASTERFRAME = ttk.Frame(master=self.pcaFrame, padding=5, relief="solid", borderwidth=1)

        self.pcaFrame_MASTERFRAME.grid(row=0,column=0,sticky="NSEW")
        self.pcaFrame.grid_columnconfigure(0, weight=1)
        self.pcaFrame.grid_rowconfigure(0, weight=1)

        self.pcaFrame_BUTTON_PLOT = ttk.Button(master=self.pcaFrame_MASTERFRAME, text="Calculate and plot PCA", command=self.m_plotPCA)
        self.pcaFrame_BUTTON_APPEND = ttk.Button(master=self.pcaFrame_MASTERFRAME, text="Append input vector as", command=self.m_appendPCADataVector)
        self.pcaFrame_ENTRY_APPEND = ttk.Entry(master=self.pcaFrame_MASTERFRAME)
        self.pcaFrame_BUTTON_CLEAR = ttk.Button(master=self.pcaFrame_MASTERFRAME, text="Clear input vector", command=self.m_clearPCADataVector)
        self.pcaFrame_BUTTON_SAVE = ttk.Button(master=self.pcaFrame_MASTERFRAME, text="Save input vector", command=self.m_savePCADataVector)

        self.pcaFrame_BUTTON_PLOT.configure(state="disabled")
        self.pcaFrame_ENTRY_APPEND.delete(0, 'end')
        self.pcaFrame_ENTRY_APPEND.insert(10, f"{CONST_LABEL_CLASS_DEFAULT}")

        self.pcaFrame_BUTTON_PLOT.grid(row=0, column=0, sticky="NSEW")
        self.pcaFrame_BUTTON_APPEND.grid(row=0, column=1, sticky="NSEW")
        self.pcaFrame_ENTRY_APPEND.grid(row=0, column=2, sticky="NSEW")
        self.pcaFrame_BUTTON_CLEAR.grid(row=0, column=3, sticky="NSEW")
        self.pcaFrame_BUTTON_SAVE.grid(row=0, column=4, sticky="NSEW")

        self.pcaFrame_MASTERFRAME.grid_columnconfigure(0, weight=1)
        self.pcaFrame_MASTERFRAME.grid_columnconfigure(1, weight=1)
        self.pcaFrame_MASTERFRAME.grid_columnconfigure(2, weight=1)
        self.pcaFrame_MASTERFRAME.grid_columnconfigure(3, weight=1)
        self.pcaFrame_MASTERFRAME.grid_rowconfigure(0, weight=1)

    def m_updateAxesLimits(self, idx_list=list(range(0,CONST_N_SENSORS))):
        if self.autoscale_plot:
            for i in idx_list:
                ymin = np.min(global_data_window_input_list[i])
                ymax = np.max(global_data_window_input_list[i])
               
                self.m_setAxesLimits(i, ymin, ymax)
        else:
            for i in idx_list:
                ymin = int(self.fsettings_entry_min[i].get())
                ymax = int(self.fsettings_entry_max[i].get())

                self.m_setAxesLimits(i, ymin, ymax)

    def m_setAxesLimits(self, idx, ymin, ymax):  
        if ymin >= ymax: ymin = ymax-1
        ymin = math.floor((ymin-5)/10.0)*10
        ymax = math.ceil((ymax+5)/10.0)*10
        if ymin < -10: ymin = -50
        if ymax > 1050: ymax = 1100        
        self.axesLimitsList[idx] = [ymin, ymax]

    def m_buttonUpdatePlotSettings(self):
        self.autoscale_plot = self.var_save_to_file_state.get()
        self.m_updateAxesLimits()
        self.m_updateAxesSettings()
    
    def m_buttonRemoveBaseline(self):
        baseline_list = self.m_getBaselinePlot()
        programSetBaseline(baseline_list)
        self.m_updateDataFrameEntryBaseline()
        #self.m_buttonUpdatePlotSettings()

    def m_getBaselinePlot(self):
        return [float(global_data_window_input_list[i][-1]) for i in range(len(global_data_window_input_list))]

    def m_updateAxesSettings(self, idx_list=list(range(0,CONST_N_SENSORS))):
        self.m_updateDataFrameEntry(idx_list=idx_list)        
        #self.m_resetPlot(idx_list=idx_list) 
        self.m_updatePlot(idx_list=idx_list)            

    def m_updateDataFrameEntry(self, idx_list=list(range(0,CONST_N_SENSORS))):
        for i in idx_list:
            [ymin, ymax] = self.axesLimitsList[i]
            self.fsettings_entry_min[i].delete(0, 'end')
            self.fsettings_entry_max[i].delete(0, 'end')
            self.fsettings_entry_min[i].insert(10, f"{ymin}")
            self.fsettings_entry_max[i].insert(10, f"{ymax}")
    
    def m_updateDataFrameEntryBaseline(self):
        for i in range(len(self.fsettings_entry_max)):
            self.fsettings_entry_baseline[i].delete(0, 'end')
            self.fsettings_entry_baseline[i].insert(10, f"{global_baseline_data_sensors[i]}")

    def m_updateDataFrame(self):
        self.fdata_text_SIZEBUFFERSAMPLES.configure(state="normal")
        self.fdata_text_SIZEBUFFERSECONDS.configure(state="normal")
        self.fdata_text_SIZEPLOTSECONDS.configure(state="normal")
            
        self.fdata_text_SIZEPLOTSECONDS.delete('1.0', tk.END)
        self.fdata_text_SIZEPLOTSECONDS.insert(tk.END, f"{global_data_window_size*global_data_window_resampling_delay} seconds")

        self.fdata_text_SIZEBUFFERSAMPLES.delete('1.0', tk.END)
        self.fdata_text_SIZEBUFFERSAMPLES.insert(tk.END, f"{global_waiting_buffer_size} samples")
        self.fdata_text_SIZEBUFFERSECONDS.delete('1.0', tk.END)
        self.fdata_text_SIZEBUFFERSECONDS.insert(tk.END, f"{global_waiting_buffer_size/global_freq:.1f} seconds")

        self.fdata_entry_SIZE_ANALYSIS_WINDOW.delete(0, tk.END)
        self.fdata_entry_SIZE_ANALYSIS_WINDOW.insert(tk.END, f"{global_PCA_data_vector_size:d}")

        self.fdata_text_SIZEBUFFERSAMPLES.configure(state="disabled")
        self.fdata_text_SIZEBUFFERSECONDS.configure(state="disabled")
        self.fdata_text_SIZEPLOTSECONDS.configure(state="disabled")

    def m_getDataFromDataFrameEntry(self):
        global global_data_window_resampling_delay
        global global_data_window_size
        global global_PCA_data_vector_size

        size_data_window = int(self.fdata_entry_SIZE_PLOT_SAMPLES.get())
        freq_resampling = float(self.fdata_entry_FREQ_PLOT_SAMPLES .get())
        pca_vector_size = int(self.fdata_entry_SIZE_ANALYSIS_WINDOW.get())

        if size_data_window < 10: size_data_window = 10
        elif size_data_window > 1000: size_data_window = 1000

        if size_data_window < .1: size_data_window = .1
        elif freq_resampling > global_freq: freq_resampling = global_freq

        if pca_vector_size < 0: pca_vector_size = 1
        elif pca_vector_size > size_data_window: pca_vector_size = size_data_window

        global_data_window_resampling_delay = 1.0 / freq_resampling
        global_data_window_size = size_data_window
        global_PCA_data_vector_size = pca_vector_size

        self.fdata_entry_SIZE_PLOT_SAMPLES.delete(0, 'end')
        self.fdata_entry_FREQ_PLOT_SAMPLES.delete(0, 'end')
        self.fdata_entry_SIZE_ANALYSIS_WINDOW.delete(0, 'end')
        self.fdata_entry_SIZE_PLOT_SAMPLES.insert(10, global_data_window_size)
        self.fdata_entry_FREQ_PLOT_SAMPLES.insert(10, f"{1.0/global_data_window_resampling_delay}")
        self.fdata_entry_SIZE_ANALYSIS_WINDOW.insert(10, f"{global_PCA_data_vector_size}")

    def m_resetPlot(self, idx_list=list(range(0,CONST_N_SENSORS+1))):
        self.x_ticks = np.arange(0, global_data_window_size, 1)
        plt.close('all') #close all figures
        for i in idx_list:
            self.m_resetFigure(i)        
        self.boolCanvasPlaced = True

    def m_updatePlot(self, idx_list=list(range(0,CONST_N_SENSORS+1))):
        self.x_ticks = np.arange(0, global_data_window_size, 1)
        for i in idx_list:
            self.m_updateFigure(i)
        self.boolCanvasPlaced = True

    def m_resetFigure(self,idx):
        #self.figs_list[idx], self.axes_list[idx] = plt.subplots(nrows=1,ncols=1,figsize=(6.0,1.175), dpi=100)    
        self.figs_list[idx] = plt.figure(figsize=(6.0,1.175), dpi=100, num=idx, clear=True)
        self.axes_list[idx] = self.figs_list[idx].add_subplot(111)
        self.figs_list[idx].patch.set_facecolor(self.faceColor)
        self.figs_list[idx].subplots_adjust(left=0.1, right=0.99, top=0.95, bottom=0.05)
        matplotlib.rc('ytick', labelsize=5) 
        matplotlib.rc('xtick', labelsize=5) 
        self.m_setParametersAx(idx)
        if self.boolCanvasPlaced: self.canvas_list[idx].get_tk_widget().destroy()
        self.canvas_list[idx] = FigureCanvasTkAgg(self.figs_list[idx], self.pyplotFrame)  
        self.canvas_list[idx].get_tk_widget().grid(column=0, row=idx)
        self.canvas_list[idx].draw()
        self.backgrounds_list[idx] = self.figs_list[idx].canvas.copy_from_bbox(self.axes_list[idx].bbox)

    def m_updateFigure(self,idx):
        self.m_setParametersAx(idx)
        self.canvas_list[idx].draw()
        self.backgrounds_list[idx] = self.figs_list[idx].canvas.copy_from_bbox(self.axes_list[idx].bbox)

    def m_setMasterWindow(self, root):
        self.mainwindow = root

    def m_initPlot(self):
        self.plotColor1 = "#00c800"
        self.plotColor2 = "white"
        self.plotColor3 = "yellow"
        self.faceColor = (0.941,0.941,0.941)
        self.gridColor = "#008242"
        self.m_resetPlot()

    def m_setParametersAx(self,idx):
        self.axes_list[idx].clear()
        [ymin, ymax] = self.axesLimitsList[idx]
        self.axes_list[idx].grid(visible=True, color=self.gridColor, linestyle='-', linewidth=1)
        self.axes_list[idx].set_ylim(ymin, ymax)
        self.offset_plot_text = (0.10*global_data_window_size)
        self.axes_list[idx].set_xlim(0, global_data_window_size-1+self.offset_plot_text)
        y_ticks = np.linspace(ymin, ymax, 11)
        self.axes_list[idx].set_yticks(y_ticks)
        self.axes_list[idx].set_xticks(self.x_ticks)
        self.axes_list[idx].set_yticklabels(y_ticks, rotation=0, fontsize=5)
        self.axes_list[idx].set_xticklabels([])
        if idx <5: self.axes_list[idx].set_ylabel(str(CONST_LIST_SENSORS[idx]), rotation=90)
        else: self.axes_list[idx].set_ylabel("Fan speed", rotation=90)
        self.axes_list[idx].set_facecolor((0.0, 0.0, 0.0))

    def m_plot(self):
        for i,_ in enumerate(self.canvas_list):
            for line in self.axes_list[i].get_lines(): # clear lines:
                line.remove()
            self.canvas_list[i].restore_region(self.backgrounds_list[i])
            # Min and Max:
            if i < 5:
                self.axes_list[i].draw_artist(self.axes_list[i].axhline(global_baseline_data_sensors[i], color=self.plotColor3, linewidth=1, linestyle="solid"))
                ymin = np.min(global_data_window_input_list[i])
                ymax = np.max(global_data_window_input_list[i])
                xmin = np.argwhere(global_data_window_input_list[i] == ymin)[-1]
                xmax = np.argwhere(global_data_window_input_list[i] == ymax)[-1]
                self.axes_list[i].draw_artist(self.axes_list[i].axhline(ymin, color=self.plotColor2, linewidth=1, linestyle="dashed"))
                self.axes_list[i].draw_artist(self.axes_list[i].axvline(xmin, color=self.plotColor2, linewidth=1, linestyle="dashed"))
                self.axes_list[i].draw_artist(self.axes_list[i].text(
                    xmin, self.axesLimitsList[i][0]+.05*(self.axesLimitsList[i][1]-self.axesLimitsList[i][0]), 
                    f"{ymin:.1f}", 
                    color=self.plotColor2,
                    horizontalalignment="left",
                    verticalalignment="bottom",
                    fontsize="x-small",
                    bbox=dict(facecolor=(0.0, 0.0, 0.0), alpha=0.5, pad=0.1)
                    )
                )
                if ymin != ymax:
                    self.axes_list[i].draw_artist(self.axes_list[i].axhline(ymax, color=self.plotColor2, linewidth=1, linestyle="dashed"))
                    self.axes_list[i].draw_artist(self.axes_list[i].axvline(xmax, color=self.plotColor2, linewidth=1, linestyle="dashed"))
                    self.axes_list[i].draw_artist(self.axes_list[i].text(
                        xmax, self.axesLimitsList[i][1]-.05*(self.axesLimitsList[i][1]-self.axesLimitsList[i][0]),
                        f"{ymax:.1f}", 
                        color=self.plotColor2,
                        horizontalalignment="left",
                        verticalalignment="top",
                        fontsize="x-small",
                        bbox=dict(facecolor=(0.0, 0.0, 0.0), alpha=0.5, pad=0.1)
                        )
                    )
                # Plot data
                self.axes_list[i].draw_artist(self.axes_list[i].plot(global_data_window_input_list[i], color=self.plotColor1, linewidth=1, linestyle="solid")[-1])
                self.axes_list[i].draw_artist(self.axes_list[i].text(
                    global_data_window_size-1+self.offset_plot_text, global_data_window_input_list[i][-1], 
                    f"{global_data_window_input_list[i][-1]:.1f}", 
                    color=self.plotColor2,
                    horizontalalignment="right",
                    verticalalignment="center",
                    fontsize="small",
                    bbox=dict(facecolor=self.plotColor1, alpha=0.5, pad=0.1),
                    )
                )
                # Plot pca window
                self.axes_list[i].draw_artist(self.axes_list[i].plot(
                    np.arange(global_data_window_size-global_PCA_data_vector_size, global_data_window_size, 1),
                    global_data_window_input_list[i][-global_PCA_data_vector_size:], 
                    color=self.plotColor1,
                    linewidth=0,
                    marker=".")[-1]
                )
            else: # Fan speed
                self.axes_list[i].draw_artist(self.axes_list[i].plot(global_data_window_fanspeed_list, color=self.plotColor1, linewidth=1, linestyle="solid")[-1])
                self.axes_list[i].draw_artist(self.axes_list[i].text(
                    global_data_window_size-1+self.offset_plot_text, global_data_window_fanspeed_list[-1], 
                    f"{global_data_window_fanspeed_list[-1]:.1f}", 
                    color=self.plotColor2, 
                    horizontalalignment="right",
                    verticalalignment="center",
                    fontsize="small",
                    bbox=dict(facecolor=self.plotColor1, alpha=0.5, pad=0.1),
                    )
                )
            self.canvas_list[i].blit(self.axes_list[i].bbox)
            if self.autoscale_plot and i < 5:    
                [ymin_limits, ymax_limits] = self.axesLimitsList[i]
                #ymin, ymax = np.min(global_data_window_input_list[i]), np.max(global_data_window_input_list[i])
                if ymin <= ymin_limits:
                    self.m_setAxesLimits(i, ymin-10, ymax)
                    self.m_updateAxesSettings(idx_list=[i])
                elif ymax >= ymax_limits:
                    self.m_setAxesLimits(i, ymin, ymax+10)
                    self.m_updateAxesSettings(idx_list=[i])

    def m_savePCADataVector(self):
        PCAfile = CDataFile(root=ObjMainWindow)
        PCAfile.m_setFilename(f"{global_test_date_start}_PCA_input")
        PCAfile.m_openNewFile(bWriteCaption=False,bCreateNewName=False)
        PCAfile.m_writeLineToFile(f"PCA INPUT VECTOR")
        for idx,data in enumerate(global_PCA_data_vector):
            PCAfile.m_writeLineToFile(f"{global_PCA_class_label_list[idx]}") # Class label
            for sensor_idx, sensor_data in enumerate(data):
                PCAfile.m_writeToFile(f"{CONST_LIST_SENSORS[sensor_idx]};")
                for sensor_data_reading in sensor_data:
                    PCAfile.m_writeToFile(f"{sensor_data_reading};")
                PCAfile.m_writeToFile("\n")
        PCAfile.m_closeFile()
        dst = PCAfile.m_saveRenamedCopy()
        ObjMainWindow.m_print(f"{CONST_TEXT_SAVED_COPY_OF}:\n{PCAfile.m_getFullPath()}\nas:\n{dst}")
        del PCAfile

    def m_appendPCADataVector(self):
        global global_PCA_data_vector
        global global_PCA_class_label_list
        # Get label
        global_PCA_class_label_list.append(str(self.pcaFrame_ENTRY_APPEND.get()))
        # Get values
        vector_values = np.array([global_data_window_input_list[i][-global_PCA_data_vector_size:] for i in range(len(global_data_window_input_list))])
        for i,_ in enumerate(vector_values):
            vector_values[i] = vector_values[i] - global_baseline_data_sensors[i] #remove baseline
        
        global_PCA_data_vector.append(vector_values)
        self.pcaFrame_BUTTON_PLOT.configure(state="normal")

    def m_clearPCADataVector(self):
        global global_PCA_data_vector
        global global_PCA_class_label_list
        global_PCA_data_vector = list()
        global_PCA_class_label_list = list()
        self.pcaFrame_BUTTON_PLOT.configure(state="disabled")

    def m_plotPCA(self):
        if len(global_PCA_class_label_list) > 0:
            self.m_savePCADataVector()
            ObjMainWindow.m_stopCommunication()
            CTkPlotterPCA(root=self.mainwindow)

class CTkPlotterPCA():
    def __init__(self, root):
        self.m_setMasterWindow(root)
        self.windowPCA = tk.Toplevel(master=self.mainwindow.root)
        self.windowPCA.geometry(f"+{0}+{0}")
        self.windowPCA.title("PCA")

        self.canvasPCA = None
        self.canvasINPUT = None

        self.m_PCA()
        self.m_prepareLabels()
        self.m_prepareLabels()
        self.m_plotPCA()        

    def m_setMasterWindow(self, root):
        self.mainwindow = root

    def m_PCA(self):
        self.vector_PCA_input = list()

        for i,data in enumerate(global_PCA_data_vector):
            self.vector_PCA_input.append(np.array(data).flatten())

        self.vector_PCA_input = np.array(self.vector_PCA_input)

        PCA_pipeline = make_pipeline(StandardScaler(), PCA(n_components=3))
        PCA_pipeline.fit(self.vector_PCA_input)

        PCA_model = PCA_pipeline.named_steps["pca"] 
        PCA_scaler = PCA_pipeline.named_steps["standardscaler"]

        self.vector_PCA_output = PCA_pipeline.transform(self.vector_PCA_input)
        self.PCA_var_ratio_norm = np.array(PCA_model.explained_variance_ratio_)
        self.PCA_var_ratio_norm /= np.sum(self.PCA_var_ratio_norm)

    def m_prepareLabels(self):
        self.label_list = list()
        self.data_label_index_list = list()
        self.label_color_list = list()
        self.label_not_placed_list = list()
        for i in range(len(self.vector_PCA_output)):
            label = global_PCA_class_label_list[i]
            if not label in self.label_list:
               self.label_list.append(label)
               self.label_not_placed_list.append(True)
               self.label_color_list.append(self.m_getRandomColor())
            self.data_label_index_list.append(self.label_list.index(label))
        print(self.label_list)

    def m_getRandomColor(self):
        r = random.random()
        g = random.random()  
        b = random.random()
        return (r,g,b)

    def m_plotPCA(self):
        fig = plt.figure(figsize=(10.0,6.0))
        axPCA = fig.add_subplot(111, projection='3d')

        for i, (pc1, pc2, pc3) in enumerate(self.vector_PCA_output):
            color = self.label_color_list[self.data_label_index_list[i]]
            label = self.label_list[self.data_label_index_list[i]]
            if self.label_not_placed_list[(self.data_label_index_list[i])]:
                axPCA.scatter(pc1, pc2, pc3, color=color, label=label)
                self.label_not_placed_list[(self.data_label_index_list[i])] = False
            else: axPCA.scatter(pc1, pc2, pc3, color=color)

        axPCA.set_xlabel(f"PC1 ({self.PCA_var_ratio_norm[0]:.3f})")
        axPCA.set_ylabel(f"PC2 ({self.PCA_var_ratio_norm[1]:.3f})")
        axPCA.set_zlabel(f"PC3 ({self.PCA_var_ratio_norm[2]:.3f})")
        axPCA.set_title("PCA Projection (n=3)")
        axPCA.grid()
        #fig.legend(rect=[0,0,0.75,1])
        fig.legend()

        self.canvasPCA = FigureCanvasTkAgg(fig, self.windowPCA)  
        self.canvasPCA.get_tk_widget().grid(column=0, row=0)
        self.canvasPCA.draw()
        self.windowPCA.update()

class CTkPlotterTestResults():
    def __init__(self, root):
        self.m_setMasterWindow(root)
        self.window = tk.Toplevel(master=self.mainwindow.root)
        x,y = self.mainwindow.m_get_root_size()
        self.window.geometry(f"+{x}+{0}")
        self.window.title('Results')
        self.masterFrame = tk.Frame(master=self.window)
        self.masterFrame.grid(row=0,column=1,sticky="NESW")

        self.masterFrame.grid_columnconfigure(0, weight=1)
        self.masterFrame.grid_rowconfigure(0, weight=1)

        self.canvas = None

        self.m_initPlot()

    def m_initFigure(self):
        self.fig, self.axes = plt.subplots(nrows=6,ncols=1,figsize=(7.0,10.0), dpi=100)    
        self.fig.patch.set_facecolor(self.faceColor)
        self.fig.subplots_adjust(left=0.1, right=0.99, top=0.95, bottom=0.05)
        self.canvas  = FigureCanvasTkAgg(self.fig, self.masterFrame)  
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1)
        #self.canvas.draw()

    def m_setMasterWindow(self, root):
        self.mainwindow = root

    def m_initPlot(self):
        self.plotColor1 = "#00c800"
        self.plotColor2 = "white"
        self.faceColor = (0.941,0.941,0.941)
        self.gridColor = "#008242"
        self.time_x_lim = [math.floor(global_air_sequence_time[0]), math.ceil(global_air_sequence_time[-1])]
        if self.time_x_lim[-1]>60:
            self.x_ticks = np.arange(self.time_x_lim[0], self.time_x_lim[-1], 5)
        elif self.time_x_lim[-1]>30:
            self.x_ticks = np.arange(self.time_x_lim[0], self.time_x_lim[-1], 1)
        elif self.time_x_lim[-1]>5:
            self.x_ticks = np.arange(self.time_x_lim[0], self.time_x_lim[-1], .5)
        else:
            self.x_ticks = np.arange(self.time_x_lim[0], self.time_x_lim[-1], .1)
        self.m_initFigure()        

    def m_setParametersAx(self,idx):
        if idx < 5:
            ymin, ymax = min(global_air_sequence_data_sensors[idx])-5, max(global_air_sequence_data_sensors[idx])+5
            yrange = ymax - ymin
            ymin, ymax = math.floor((ymin - .05*yrange)*10)/10, math.ceil((ymax+.05*yrange)*10)/10
            #y_ticks = np.arange(ymin, ymax, 5)
        else:
            ymin, ymax = -10, 110
            #y_ticks = np.arange(0, 110, 10)
        self.axes[idx].grid(visible=True, color=self.gridColor, linestyle='-', linewidth=1)
        self.axes[idx].set_ylim(ymin, ymax)
        #self.axes[idx].set_yticks(ticks=y_ticks)
        self.axes[idx].set_xticks(ticks=self.x_ticks)
        #self.axes[idx].set_yticklabels(y_ticks)
        #self.axes[idx].set_xticklabels(self.x_ticks)
        if idx <5: self.axes[idx].set_ylabel(str(CONST_LIST_SENSORS[idx]), rotation=90)
        else: self.axes[idx].set_ylabel("Fan speed", rotation=90)
        self.axes[idx].set_facecolor((0.0, 0.0, 0.0))

    def m_plot(self):
        for i,_ in enumerate(self.axes):
            if i < 5:
                self.axes[i].plot(
                    global_air_sequence_time,
                    global_air_sequence_data_sensors[i],
                    color=self.plotColor1,
                    linewidth=1,
                    linestyle="solid")
            else: # Fan speed
                self.axes[i].plot(
                    global_air_sequence_time,
                    global_air_sequence_fan_speed,
                    color=self.plotColor1, 
                    linewidth=1, 
                    linestyle="solid")
            self.m_setParametersAx(i)
            self.canvas.draw()

class CTkNNPanel():
    def __init__(self, root):
        self.m_setMasterWindow(root)
        self.root = tk.Toplevel(master=self.mainwindow.root)
        self.root.resizable(width=False, height=False)
        x, y = self.mainwindow.m_get_root_size()
        self.root.geometry(f"+{x}+{0}")
        
        self.root.title(CONST_NNPANEL_TITLE) # set window title
        if os.path.exists(CONST_ICON_PATH): self.root.iconbitmap(CONST_ICON_PATH)

        self.m_placeTempImg()

        self.masterFrame = tk.Frame(master=self.root)
        self.masterFrame.grid(column=0, row=0)

        self.label_TITLE = tk.Label(master=self.masterFrame, pady=5, bg="grey", text=CONST_TEXT_LABEL_TITLE_NN)
        self.settingsFrame = tk.Frame(master=self.masterFrame, borderwidth=5)
        self.textFrame = tk.Frame(master=self.masterFrame, borderwidth=5)

        self.label_TITLE.grid(column=0, row=0, columnspan=2, sticky="NSWE")
        self.settingsFrame.grid(column=0, row=1, sticky="NWE")
        self.textFrame.grid(column=1, row=1, sticky="NWE")
        
        self.buttonsFrame = tk.Frame(master=self.settingsFrame)
        self.parametersFrame = tk.Frame(master=self.settingsFrame)

        self.buttonsFrame.grid(column=0, row=1, sticky="N")
        self.parametersFrame.grid(column=0, row=2, sticky="N")
      
        
        self.m_initSettingsFrame()
        self.m_initTextFrame()
        self.m_initParametersFrame()

        self.m_initModelNN()
    
    def m_initDevice(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

    def m_initSettingsFrame(self):
        self.fsettings_label_title = tk.Label(master=self.buttonsFrame, text=CONST_TEXT_LABEL_NN_SETTINGS_TITLE)
        self.fsettings_label_title.grid(row=0, column=0, columnspan=2, sticky="WE", pady=1)

        self.fsettings_button_load = tk.Button(master=self.buttonsFrame, text=CONST_TEXT_BUTTON_NN_LOAD, bg="green", fg="white", command=self.m_loadNN)
        self.fsettings_button_load.grid(row=1, column=0, sticky="WE", padx=1, pady=1)

        self.fsettings_button_new = tk.Button(master=self.buttonsFrame, text=CONST_TEXT_BUTTON_NN_NEW, bg="red", fg="white", command=self.m_newNN)
        self.fsettings_button_new.grid(row=1, column=1, sticky="WE", padx=1, pady=1)

        self.fsettings_button_add_to_dataset = tk.Button(master=self.buttonsFrame, text=CONST_TEXT_BUTTON_NN_ADD, bg="grey", command=self.m_addDatasetNN)
        self.fsettings_button_add_to_dataset.grid(row=2, column=0, columnspan=2, sticky="WE", padx=1, pady=1)

        self.fsettings_button_retrain = tk.Button(master=self.buttonsFrame, text=CONST_TEXT_BUTTON_NN_RETRAIN, bg="yellow", command=self.m_retrainNN)
        self.fsettings_button_retrain.grid(row=3, column=0, columnspan=2, sticky="WE", padx=1, pady=1)    

        self.fsettings_button_save = tk.Button(master=self.buttonsFrame, text=CONST_TEXT_BUTTON_NN_SAVE, bg="green", fg="white", command=self.m_saveNN)
        self.fsettings_button_save.grid(row=4, column=0, columnspan=2, sticky="WE", padx=1, pady=1) 

    def m_initParametersFrame(self):
        self.fparam_label_lr = tk.Label(master=self.parametersFrame, text=CONST_TEXT_LABEL_NN_ENTRY_LR)
        self.fparam_label_lr.grid(row=0, column=0, sticky="E")

        self.fparam_entry_lr = tk.Entry(master=self.parametersFrame, width=15)
        self.fparam_entry_lr.grid(row=0, column=1, sticky="WE")

        self.fparam_label_epochs = tk.Label(master=self.parametersFrame, text=CONST_TEXT_LABEL_NN_ENTRY_EPOCHS)
        self.fparam_label_epochs.grid(row=1, column=0, sticky="E")

        self.fparam_entry_epochs = tk.Entry(master=self.parametersFrame, width=15)
        self.fparam_entry_epochs.grid(row=1, column=1, sticky="WE")

    def m_initTextFrame(self):
        self.ftext_label_text = tk.Label(master=self.textFrame, text="INFO")
        self.ftext_label_text.grid(row=0, column=0, sticky="WE", pady=1)

        self.ftext_text = tk.Text(master=self.textFrame, width=30, height=25,font=CONST_DEFAULT_FONT)
        self.ftext_text.grid(row=1, column=0, sticky="WE")

    def m_setMasterWindow(self, root):
        self.mainwindow = root

    def m_initModelNN(self):
        self.m_initDevice()
        self.model = NN.createModel()
        if os.path.exists(global_path_to_NNmodels_dir+"curr_model.pth"): 
            self.model.loadModelState(global_path_to_NNmodels_dir+"curr_model.pth")
        self.learning_rate = 1e-6
        self.epochs = 1
        self.model.setOptimizer( optim.SGD(params=self.model.getParams(), lr=self.learning_rate, momentum=.995) ) # Set optimizer
        self.model.setLossFunction( nn.BCELoss() ) # Set LOSS function
        self.m_saveCurrNN()
        self.m_clear_print(self.model.getModelInfo())
        self.m_updateValuesEntry()

    def m_getValuesEntry(self):
        epochs = int(self.fparam_entry_epochs.get())
        lr = float(self.fparam_entry_lr.get())
        if lr <= 0.0: lr = 1e-15
        if lr >= 1000.0: lr=1000.0
        if epochs <= 0: epochs = 1
        if epochs >= 999999: epochs = 999999

        self.learning_rate = lr
        self.epochs = epochs

    def m_updateValuesEntry(self):
        self.fparam_entry_epochs.delete(0, 'end')
        self.fparam_entry_lr.delete(0, 'end')
        self.fparam_entry_epochs.insert(10, self.epochs)
        self.fparam_entry_lr.insert(10, self.learning_rate)

    def m_newNN(self):
        if TKcreateWindowYesNo("Create new model", "Do you want to create new empty model?", windowIcon = "warning"):
            self.model = NN.createModel()
            self.m_saveCurrNN()
        self.m_clear_print(self.model.getModelInfo())
        self.m_updateValuesEntry()

    def m_saveCurrNN(self):
        self.model.saveModelState(global_path_to_NNmodels_dir+"curr_model.pth")

    def m_loadNN(self):
        filetypes = (
        ('NN models', '*.pth'),
        ('All files', '*.*')
        )
        filename = fd.askopenfilename(title='Open a model',
        initialdir=global_path_to_NNmodels_dir,
        filetypes=filetypes
        )
        if filename: self.model.loadModelState(filename)
        self.m_clear_print(self.model.getModelInfo())
        self.m_updateValuesEntry()

    def m_saveNN(self):
        filetypes = (
        ('NN models', '*.pth'),
        ('All files', '*.*')
        )
        filename = fd.asksaveasfilename(title='Save a model',
        initialdir=global_path_to_NNmodels_dir,
        filetypes=filetypes
        )
        if filename: self.model.saveModelState(filename)    

    def m_addDatasetNN(self):
        boolTMP = TKcreateWindowYesNo("Add array to existing dataset", "Label = 1.0?", windowIcon = "warning")
        if boolTMP:    
            pass

    def m_retrainNN(self):
        self.m_getValuesEntry()
        self.m_updateValuesEntry()
        dataset_train = NN.CDataset(global_path_to_dataset_dir, Device=self.model.getDevice())    
        # PREPARE DATALOADERS
        dataloader_train = NN.createDataLoader(dataset_train,1,True)
        self.model.setOptimizer( optim.SGD(params=self.model.getParams(), lr=self.learning_rate, momentum=.995) ) # Set optimizer
        self.model.setLossFunction( nn.BCELoss() ) # Set LOSS function
        self.model.trainModel(dataloader_train,Epochs=self.epochs)
        self.m_saveCurrNN()
        TKcreateWindow("Network retrained", "Network succesfully retrained!", windowIcon = "info")
        self.m_clear_print(self.model.getModelInfo())
        self.m_plot_loss()

    def m_placeTempImg(self):
        img = ImageTk.PhotoImage(Image.open(CONST_SPLASHSCREEN_PATH).resize((300, 300)))
        label = tk.Label(master=self.root, image = img, compound='center')
        label.config(text=CONST_TEXT_LOADING, fg="white", bg="white", compound='center', font=("Bahnschrift Bold Condensed", 16))
        label.grid(row=0,column=0)
        self.root.update()

    def m_clear_print(self, record):
        self.ftext_text.delete('1.0', tk.END)
        self.ftext_text.insert(tk.END,f"{record}\n")
        self.ftext_text.see("end")
        print(str(record))

    def m_print(self, record):
        self.ftext_text.insert(tk.END, f"{record}\n")
        self.ftext_text.see("end")
        print(str(record))

    def m_plot_loss(self):
        w, h = 400, 400
        plotColor = "#00c800"
        faceColor = (0.941,0.941,0.941)
        gridColor = "#008242"
        window_plot = tk.Toplevel(master=self.root)
        window_plot.geometry(f"{w}x{h}")
        window_plot.title(CONST_NNPANEL_TITLE) # set window title
        fig, ax = plt.subplots(nrows=1,ncols=1,figsize=(3.0,4.0), dpi=100)
        fig.patch.set_facecolor(faceColor)
        ax.plot(self.model.getListLoss(), linewidth=2, color=plotColor)         
        ax.grid(visible=True, color=gridColor, linestyle='-', linewidth=1)
        ax.set_ylabel('Loss function')
        ax.set_xlabel('Epochs')
        ax.set_facecolor((0.0, 0.0, 0.0))
        canvas = FigureCanvasTkAgg(fig, window_plot)  
        canvas.get_tk_widget().pack(side=tk.TOP ,fill=tk.BOTH)
        canvas.draw()

def TKcreateWindow(windowTitle, windowText, windowIcon = "warning"):
    msg.showinfo(windowTitle, windowText, icon=windowIcon)

def TKcreateWindowYesNo(windowTitle, windowText, windowIcon = "warning"):
    window = msg.askquestion(windowTitle, windowText, icon=windowIcon)
    if window == "yes": return True
    else: return False

#########################################################################################
# Global vars
#########################################################################################
CONST_LABEL_CLASS_CLEARING = "clearing"
CONST_LABEL_CLASS_TEST = "test"
CONST_LABEL_CLASS_RECOVERY = "recovery"
CONST_LABEL_CLASS_NONE = "none"
CONST_LABEL_CLASS_DEFAULT = "undefined"

global_time_test_start = 0.0

global_current_class_label = CONST_LABEL_CLASS_NONE

global_fan_speed_user = int(50 / 100.0 * 255)

global_fan_status = True
global_air_sampling_status = False
global_air_sampling_speed = 255
global_air_sampling_time = 2.5
global_air_sampling_time_start = 0.0

global_air_clearing_status = False
global_air_clearing_speed = 255
global_air_clearing_time = 2.5
global_air_clearing_time_start = 0.0

global_air_test_time = 15.0
global_air_test_speed = int(20 / 100.0 * 255)

global_air_recovery_time = 40.0
global_air_recovery_speed = int(20 / 100.0 * 255)

global_test_sequence_status = [False, False, False, False]
global_test_sequence_time_start = 0.0
global_test_sequence_times = [global_air_sampling_time, global_air_test_time, global_air_clearing_time, global_air_recovery_time]
global_test_sequence_class_labels = [global_current_class_label, CONST_LABEL_CLASS_TEST, CONST_LABEL_CLASS_CLEARING, CONST_LABEL_CLASS_RECOVERY]
#[Air sampling, Test, Clearing, Recovery]

global_class_name_override_status = False
global_class_name_override_samples = 1
global_cont_mode_sampling_freq = 1

global_transmission_state = False
global_save_to_file = True
global_log_data = False
global_plot_real_time = False

global_air_sequence_data_sensors = [list()]*CONST_N_SENSORS
global_air_sequence_time = list()
global_air_sequence_data_temperature = list()
global_air_sequence_fan_speed = list()

global_baseline_data_sensors = np.zeros(5).astype(float)

global_current_time = 0.0
global_current_temperature = 0.0
global_current_inputs = np.zeros(CONST_N_SENSORS).astype(float)
global_current_inputs_2 = np.zeros(CONST_N_SENSORS).astype(float)
global_current_inputs_3 = np.zeros(CONST_N_SENSORS).astype(float)
global_current_inputs_4 = np.zeros(CONST_N_SENSORS).astype(float)
global_current_fan_speed = 0.0

global_tmp_current_temperature = 0.0
global_tmp_current_fan_speed = 0.0
global_tmp_current_inputs = np.zeros(CONST_N_SENSORS).astype(float)

global_PCA_data_vector = list()
global_PCA_class_label_list = list()
global_PCA_data_vector_size = 25

global_NN_output = 0.0

global_data_window_resampling_delay = 0.5 #in seconds - T for window resampling (f_rs=1/T)
global_waiting_buffer_input_list, global_waiting_buffer_size = getWaitingBuffer() # Buffer with N=(scale*FREQ) last samples
global_data_window_size = 100 #number of samples in window (analysis and plotting)
global_data_window_skip_samples = getSamplesToSkip()

global_data_window_fanspeed_list = np.zeros((global_data_window_size)).astype(float)
global_data_window_input_list = np.zeros((CONST_N_SENSORS, global_data_window_size)).astype(float)
global_data_window_aprox_list = np.zeros((CONST_N_SENSORS, global_data_window_size)).astype(float)
global_data_window_time_list = np.linspace(0,global_data_window_size*global_data_window_resampling_delay,global_data_window_size)
global_data_window_class_list = np.zeros((global_data_window_size)).astype(str)
global_sample_counter = 0
global_buffer_kernel = calcBufferGaussKernel()
global_first_input = True

global_test_date_start = programGetCurrentDate()

try:
    try:
#########################################################################################
# Init objects
#########################################################################################
        ObjMainWindow = None
        ObjPort = None
        ObjFile = None
        ObjMainWindow = CTkMainWindow()
        ObjPort = CSerialPort(ObjMainWindow)
        ObjFile = CDataFile(ObjMainWindow)
        ObjMainWindow.m_update()
#ObjNN = CTkNNPanel(ObjMainWindow)
#########################################################################################
# MainLoop
#########################################################################################
        ObjMainWindow.m_mainloop()
        programDisconnect()
    except serial.serialutil.SerialException as e:    
        programRaiseException(e)
except Exception as e:
    programExceptionHandler(e)  

