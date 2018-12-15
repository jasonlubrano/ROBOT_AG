# from __future__ import absolute_import
# from __future__ import division
# from __future__ import print_function

import argparse
import sys
import time
import cv2
import os

import numpy as np
import tensorflow as tf


os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
file_name = "temp.jpg"
model_file = "retrained_graph.pb"
label_file = "retrained_labels.txt"
input_height = 224
input_width = 224
input_mean = 128
input_std = 128
input_layer = "input"
output_layer = "final_result"


def load_graph(model_file):
    graph = tf.Graph()
    graph_def = tf.GraphDef()

    with open(model_file, "rb") as f:
        graph_def.ParseFromString(f.read())
    with graph.as_default():
        tf.import_graph_def(graph_def)

    return graph

graph = load_graph(model_file)

def read_tensor_from_image_file(file_name, input_height=299, input_width=299, input_mean=0, input_std=255):
    input_name = "file_reader"
    file_reader = tf.read_file(file_name, input_name)
    if file_name.endswith(".png"):
        image_reader = tf.image.decode_png(file_reader, channels = 3, name='png_reader')
    elif file_name.endswith(".gif"):
        image_reader = tf.squeeze(tf.image.decode_gif(file_reader, name='gif_reader'))
    elif file_name.endswith(".bmp"):
        image_reader = tf.image.decode_bmp(file_reader, name='bmp_reader')
    else:
        image_reader = tf.image.decode_jpeg(file_reader, channels = 3, name='jpeg_reader')
    float_caster = tf.cast(image_reader, tf.float32)
    dims_expander = tf.expand_dims(float_caster, 0);
    resized = tf.image.resize_bilinear(dims_expander, [input_height, input_width])
    normalized = tf.divide(tf.subtract(resized, [input_mean]), [input_std])
    sess = tf.Session()
    result = sess.run(normalized)

    return result

def load_labels(label_file):
    label = []
    proto_as_ascii_lines = tf.gfile.GFile(label_file).readlines()
    for l in proto_as_ascii_lines:
        label.append(l.rstrip())
    return label

def classifyImage(inputFile):
    t = read_tensor_from_image_file(inputFile,
                                    input_height=input_height,
                                    input_width=input_width,
                                    input_mean=input_mean,
                                    input_std=input_std)

    input_name = "import/" + input_layer
    output_name = "import/" + output_layer
    input_operation = graph.get_operation_by_name(input_name)
    output_operation = graph.get_operation_by_name(output_name)

    with tf.Session(graph=graph) as sess:
        start = time.time()
        results = sess.run(output_operation.outputs[0], {input_operation.outputs[0]: t})
        end=time.time()
    results = np.squeeze(results)

    top_k = results.argsort()[-5:][::-1]
    labels = load_labels(label_file)

    print('\nEvaluation time (1-image): {:.3f}s\n'.format(end-start))
    template = "{} (score={:0.5f})"
    for i in top_k:
        print(template.format(labels[i], results[i]))
    return labels[top_k[0]]


def take_picture(cell1, cell2, cell3, cell4):
    field = [None] * 4
    cv2.imwrite('temp0.jpg',cell1)
    cv2.imwrite('temp1.jpg',cell2)
    cv2.imwrite('temp2.jpg',cell3)
    cv2.imwrite('temp3.jpg',cell4)
    time.sleep(2)
    for i in range(4):
        classified = classifyImage('temp' + str(i) + '.jpg')
        if(classified == 'empty'):
            field[i] = 0
        elif(classified == 'lisianthus'):
            field[i] = 1
        elif(classified == 'rose'):
            field[i] = 2
        elif(classified == 'carnation'):
            field[i] = 3
        elif(classified == 'gardenia'):
            field[i] = 4
        elif(classified == 'weed1'):
            print("A weed has been detected in cell " + str(i))
            field[i] = 5
    print("field", field)


def classifyImages():

    cap = cv2.VideoCapture(1)
    while(True):
        _, frame = cap.read()
        cv2.imshow('frame',frame)
        k = cv2.waitKey(5) & 0xFF
        height, width = frame.shape[:2]
        cell1 = frame[int(0):int(height * .5) , int(0):int(width * .5)]
        cv2.imshow("Cell1", cell1)

        cell2 = frame[int(0):int(height * .5) , int(width * .5):int(width)]
        cv2.imshow("Cell2", cell2) 

        cell3 = frame[int((height * .5)):int(height) , int(0):int(width * .5)]
        cv2.imshow("Cell3", cell3) 

        cell4 = frame[int((height * .5)):int(height) , int(width * .5):int(width)]
        cv2.imshow("Cell4", cell4)  

        if k == 49:
            take_picture(cell1, cell2, cell3, cell4)
            break
    cv2.destroyAllWindows()

classifyImages()
