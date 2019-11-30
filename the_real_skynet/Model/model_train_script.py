import glob
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D
from keras.utils import to_categorical
from keras.preprocessing import image
import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split


width = 64
height = 64

def preprocess_input_stream(filename):
    
    seperate = tf.strings.split(filename, '/')
    label = seperate[-2]

    img = image.load_img(filename,target_size=(width,height,3))
    img = image.img_to_array(img)
    img = img/255

    return img, label


def model():

    model = Sequential()
    model.add(Conv2D(filters=16, kernel_size=2, activation="relu", input_shape=(width,height,3)))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))
    model.add(Conv2D(filters=32, kernel_size=2, activation='relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))
    model.add(Conv2D(filters=64, kernel_size=2, activation="relu"))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))
    model.add(Conv2D(filters=64, kernel_size=2, activation='relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))
    model.add(Flatten())
    model.add(Dense(128, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(64, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(10, activation='softmax'))

    opt = keras.optimizers.Adam(learning_rate=0.001)

    model.compile(
        optimizer= opt,
        loss='categorial_crossentropy',
        metrics=['accuracy']
    )

    return model

if __name__ == "__main__":
    
    path_to_data = tf.data.Dataset.list_files(str('path_to_data'))
    img, label = path_to_data.map(preprocess_input_stream)

    train_img, vali_img, train_label, vali_label = train_test_split(img, label, test_size=0.4)
    #train = zip(train_img,train_label)
    #vali = zip(vali_img, vali_label)
    train_label_enc = keras.utils.to_categorical(train_label, num_classes=10)
    vali_label_enc = keras.utils.to_categorical(vali_label, num_classes=10)

    train_model = model()

    train_model.fit(train_img, y=train_label_enc, epochs=100, batch_size= 100)
    train_model.save()

    print("Test set accuracy = " + str(evaluate(vali_img, y=vali_label_enc, batch_size= 100)))

    

    
