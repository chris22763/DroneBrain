import findspark
findspark.init("/opt/spark")
import pyspark
from pyspark import SparkContext, SparkConf
import PIL.Image
import numpy as np
from keras.applications.imagenet_utils import preprocess_input
from sparkdl.estimators.keras_image_file_estimator import KerasImageFileEstimator
from pyspark.ml.classification import LogisticRegression
from pyspark.ml.evaluation import MulticlassClassificationEvaluator
from pyspark.ml import Pipeline
from sparkdl import DeepImageFeaturizer
from pyspark.ml.image import ImageSchema
from pyspark.sql.functions import lit, shuffle
from sparkdl.image import imageIO
from pyspark.sql.DataFrame import union, repartition
from pyspark.ml.classification import LogisticRegression
from pyspark.ml import Pipeline
from sparkdl import DeepImageFeaturizer 
from pyspark.ml.evaluation import MulticlassClassificationEvaluator
from sparkdl import KerasImageFileTransformer
from keras.preprocessing.image import img_to_array, load_img

###
#   With this class we test our pretrained CNN against the pretrained Keras InceptionV3 Model on a Spark Cluster
#   with a different Dataset but hardcoded classes
###

processed_images_train=[]
processed_images_vali=[]


def load_images_path_and_shuffle():

    annual_crop_df = ImageSchema.readImages("path_to_images").withColumn("label", lit(0))
    forest_df = ImageSchema.readImages("path_to_images").withColumn("label", lit(1))
    herb_veg_df = ImageSchema.readImages("path_to_images").withColumn("label", lit(2))
    highway_df = ImageSchema.readImages("path_to_images").withColumn("label", lit(3))
    industrial_df = ImageSchema.readImages("path_to_images").withColumn("label", lit(4))
    pasture_df = ImageSchema.readImages("path_to_images").withColumn("label", lit(5))
    perm_crop_df = ImageSchema.readImages("path_to_images").withColumn("label", lit(6))
    residential_df = ImageSchema.readImages("path_to_images").withColumn("label", lit(7))
    river_df = ImageSchema.readImages("path_to_images").withColumn("label", lit(8))
    sea_lake_df = ImageSchema.readImages("path_to_images").withColumn("label", lit(9))

    annual_crop_train, annual_crop_vali = annual_crop_df.randomSplit([0.4, 0.6])
    forest_train, forest_vali = forest_df.randomSplit([0.4, 0.6])
    herb_veg_train, herb_veg_vail = herb_veg_df.randomSplit([0.4, 0.6])
    highway_train, highway_vali = highway_df.randomSplit([0.4, 0.6])
    industrial_train, industrial_vali = industrial_df.randomSplit([0.4, 0.6])
    pasture_train, pasture_vali = pasture_df.randomSplit([0.4, 0.6])
    perm_crop_train, perm_crop_vali = perm_crop_df.randomSplit([0.4, 0.6])
    residential_train, residential_vali = residential_df.randomSplit([0.4, 0.6])
    river_train, river_vali = river_df.randomSplit([0.4, 0.6])
    sea_lake_train, sea_lake_vali = sea_lake_df.randomSplit([0.4, 0.6])

    train_df_phase1 = annual_crop_train.union(forest_train)
    vali_df_pase1 = annual_crop_vali.union(forest_vali)

    train_df_phase2 = train_df_phase1.union(herb_veg_train)
    vali_df_pase2 = vali_df_pase1.union(herb_veg_vail)

    shuffle(train_df_phase2)
    shuffle(vali_df_pase2)

    train_df_phase3 = train_df_phase2.union(highway_train)
    vali_df_phase3 = vali_df_pase2.union(highway_vali)

    train_df_phase4 = train_df_phase3.union(industrial_train)
    vali_df_phase4 = vali_df_phase3.union(industrial_vali)

    shuffle(train_df_phase4)
    shuffle(vali_df_phase4)

    train_df_phase5 = train_df_phase4.union(pasture_train)
    vali_df_phase5 = vali_df_phase4.union(pasture_vali)

    shuffle(train_df_phase5)
    shuffle(vali_df_phase5)

    train_df_phase6 = train_df_phase5.union(perm_crop_train)
    vali_df_phase6 = vali_df_phase5.union(perm_crop_vali)

    train_df_phase7 = train_df_phase6.union(residential_train)
    vali_df_phase7 = vali_df_phase6.union(residential_vali)

    shuffle(train_df_phase7)
    shuffle(vali_df_phase7)

    train_df_phase8 = train_df_phase7.union(river_train)
    vali_df_phase8 = vali_df_phase7.union(river_vali)

    train_df = train_df_phase8.union(sea_lake_train)
    vali_df = vali_df_phase8.union(sea_lake_vali)

    train = shuffle(train_df)
    vali = shuffle(vali_df)

    for img in train[0]
        element = preprocess_img(img)
        processed_images_train.append(element)

    for img in sorted(range(len(vali[0]), key = vali.__getitem__))
        element = preprocess_img(img)
        processed_images_vali.append(element)
    

def transfer_learning():

    load_images_path_and_shuffle()
    
    processed_images_train = processed_images_train.repartition(100)
    processed_images_vali = processed_images_vali.repartition(100)

    featurizer = DeepImageFeaturizer(inputCol="image", outputCol="features", modelName="InceptionV3")
    lr = LogisticRegression(maxIter=10, regParam=0.05, elasticNetParam=0.3, labelCol="label")
    p = Pipeline(stages=[featurizer, lr])

    transfer_model = p.fit(processed_images_train)

    validated_df = transfer_model.transform(processed_images_vali)
    evaluator = MulticlassClassificationEvaluator(metricName="accuracy")
    
    print("Test set accuracy = " + str(evaluator.evaluate(validated_df.select("prediction", "label"))))

def fit_custom_model_distributed():
    
    transform = KerasImageFileTransformer(inputCol="uri", outputCol="predictions",
                                        modelFile='path_to_pretrained_model', 
                                        imageLoader=load_images,
                                        outputMode="vector")

def preprocess_img(img):

    image = img_to_array(load_img(img, target_size=(299, 299))) 
    image = np.expand_dims(image, axis=0)
    image = image / 255
    return preprocess_input(image)



if __name__ == "__main__":


    #Initialisiert ApacheSpark im lokalen Modus mit zwei Cores/Threads
    conf = SparkConf().setMaster("local[2]").setAppName("DL_Model")
    sc = SparkContext(conf)

    transfer_learning()

    #fit_custom_model_distributed()



