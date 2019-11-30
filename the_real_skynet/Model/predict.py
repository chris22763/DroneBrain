from keras.preprocessing import image
from keras.models import load_model
import keras.models
import os

class Predict:

    def __init__(self):
        print("Classification Started")

    def predict(self, file, ext, args):

        if args['delete']:
            os.remove(file)

        predictions = []
        file = file+ext
        print("Reading {}".format(file))
        
        img = image.load_img(file,target_size=(64,64,3))
        img = image.img_to_array(img)
        img = img/255

        model = load_model("path_to_saved_model")

        predict = model.predict(img)

        predictions.append(predict)

        return predictions




